#include "sarx/anatomy.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <thread>
#include <unordered_map>

namespace sarx {
namespace {

constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();

// Persistent fork-join pool for the conflict-free solver partitions. Work is
// split into fixed contiguous chunks; since no two items in a partition touch
// the same particle, the result is independent of the thread count.
class WorkerPool {
public:
    explicit WorkerPool(unsigned threads) : count_(std::max(1u, threads)) {
        for (unsigned i = 1; i < count_; ++i) workers_.emplace_back([this, i] { loop(i); });
    }
    ~WorkerPool() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            ++generation_;
        }
        cv_.notify_all();
        for (auto& t : workers_) t.join();
    }
    [[nodiscard]] unsigned size() const { return count_; }

    void run(std::size_t n, const std::function<void(std::size_t, std::size_t)>& job) {
        if (count_ == 1 || n < 256) {
            job(0, n);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            job_ = &job;
            n_ = n;
            remaining_.store(static_cast<int>(count_ - 1), std::memory_order_relaxed);
            ++generation_;
        }
        cv_.notify_all();
        run_chunk(0);
        while (remaining_.load(std::memory_order_acquire) > 0) std::this_thread::yield();
        job_ = nullptr;
    }

private:
    void run_chunk(unsigned index) {
        const std::size_t per = (n_ + count_ - 1) / count_;
        const std::size_t b = std::min(n_, per * index);
        const std::size_t e = std::min(n_, b + per);
        if (b < e) (*job_)(b, e);
    }

    void loop(unsigned index) {
        std::uint64_t seen = 0;
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [&] { return generation_ != seen; });
                seen = generation_;
                if (stop_) return;
            }
            run_chunk(index);
            remaining_.fetch_sub(1, std::memory_order_acq_rel);
        }
    }

    unsigned count_;
    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::uint64_t generation_{0};
    bool stop_{false};
    const std::function<void(std::size_t, std::size_t)>* job_{nullptr};
    std::size_t n_{0};
    std::atomic<int> remaining_{0};
};

WorkerPool& shared_pool(unsigned requested) {
    static std::mutex guard;
    static std::unique_ptr<WorkerPool> pool;
    unsigned want = requested == 0 ? std::max(1u, std::min(8u, std::thread::hardware_concurrency())) : requested;
    std::lock_guard<std::mutex> lock(guard);
    if (!pool || pool->size() != want) {
        pool.reset();
        pool = std::make_unique<WorkerPool>(want);
    }
    return *pool;
}
constexpr double kReferenceVoxel = 0.02;

// Corner bit layout: bit0 = +x, bit1 = +y, bit2 = +z (matches Lin 2025, Alg. 2).
constexpr int corner_bit(int axis) { return 1 << axis; }


struct V3f {
    float x{}, y{}, z{};
};

inline V3f operator+(V3f a, V3f b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline V3f operator-(V3f a, V3f b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline V3f operator*(V3f a, float s) { return {a.x * s, a.y * s, a.z * s}; }
inline float dotf(V3f a, V3f b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3f crossf(V3f a, V3f b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline float lenf(V3f a) { return std::sqrt(dotf(a, a)); }

std::uint64_t mix_hash(std::uint64_t h, std::uint64_t v) {
    h ^= v + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
    return h;
}

// Deterministic per-index pseudo random in [-1, 1].
double hash_unit(std::uint64_t seed) {
    seed = mix_hash(seed, 0xD1B54A32D192ED03ull);
    seed ^= seed >> 33;
    seed *= 0xff51afd7ed558ccdull;
    seed ^= seed >> 33;
    return static_cast<double>(seed & 0xFFFFFF) / static_cast<double>(0x7FFFFF) - 1.0;
}

bool segment_triangle(
    const Vec3& p0, const Vec3& p1,
    const Vec3& a, const Vec3& b, const Vec3& c,
    double& t_out, double& u_out, double& v_out) {

    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 d = p1 - p0;
    const Vec3 pv = cross(d, e2);
    const double det = dot(e1, pv);
    if (std::abs(det) < 1e-14) return false;
    const double inv = 1.0 / det;
    const Vec3 tv = p0 - a;
    const double u = dot(tv, pv) * inv;
    if (u < 0.0 || u > 1.0) return false;
    const Vec3 qv = cross(tv, e1);
    const double v = dot(d, qv) * inv;
    if (v < 0.0 || u + v > 1.0) return false;
    const double t = dot(e2, qv) * inv;
    if (t < 0.0 || t > 1.0) return false;
    t_out = t;
    u_out = u;
    v_out = v;
    return true;
}

} // namespace

const char* tissue_name(Tissue tissue) {
    switch (tissue) {
    case Tissue::Skin: return "skin";
    case Tissue::Fat: return "fat";
    case Tissue::Muscle: return "muscle";
    case Tissue::Tendon: return "tendon";
    case Tissue::Bone: return "bone";
    case Tissue::Marrow: return "marrow";
    case Tissue::Brain: return "brain";
    case Tissue::Heart: return "heart";
    case Tissue::Lung: return "lung";
    case Tissue::Liver: return "liver";
    case Tissue::Gut: return "gut";
    case Tissue::Count: break;
    }
    return "unknown";
}

TissueTable default_tissue_table() {
    TissueTable t{};
    auto set = [&](Tissue id, float density, float shape, float k,
                   float hp, float tear, float ballistic, float tone, Rgb8 color) {
        auto& p = t[static_cast<std::size_t>(id)];
        p.density = density;
        p.shape_stiffness = shape;
        p.bond_stiffness = k;
        p.cut_hp = hp;
        p.tear_strain = tear;
        p.ballistic_cost = ballistic;
        p.rig_tone = tone;
        p.color = color;
    };
    //        tissue          rho   shape  bond  cutHP  tear  ballistic tone   colour
    set(Tissue::Skin,   1100, 0.70f, 1.00f, 3.0f, 1.10f, 10.0f, 0.04f, {222, 170, 138});
    set(Tissue::Fat,     920, 0.45f, 0.80f, 0.8f, 1.00f,  5.0f, 0.03f, {246, 220, 146});
    set(Tissue::Muscle, 1060, 0.60f, 1.00f, 2.5f, 0.80f, 16.0f, 0.06f, {168,  38,  44});
    set(Tissue::Tendon, 1120, 0.85f, 1.00f, 6.0f, 1.00f, 24.0f, 0.08f, {236, 226, 212});
    set(Tissue::Bone,   1900, 1.00f, 1.00f, 30.f, 0.90f, 140.f, 0.60f, {240, 234, 212});
    set(Tissue::Marrow, 1000, 0.90f, 1.00f, 25.f, 0.90f, 70.0f, 0.50f, {196,  74,  64});
    set(Tissue::Brain,  1040, 0.25f, 0.60f, 0.3f, 0.60f,  4.0f, 0.02f, {232, 182, 188});
    set(Tissue::Heart,  1060, 0.60f, 1.00f, 2.0f, 0.70f, 14.0f, 0.05f, {128,  18,  30});
    set(Tissue::Lung,    400, 0.30f, 0.70f, 0.5f, 0.60f,  3.0f, 0.02f, {214, 118, 130});
    set(Tissue::Liver,  1070, 0.40f, 0.80f, 0.9f, 0.60f,  7.0f, 0.03f, {104,  28,  30});
    set(Tissue::Gut,    1040, 0.30f, 0.70f, 1.0f, 0.90f,  5.0f, 0.02f, {220, 148, 138});
    return t;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

AnatomyBody::AnatomyBody(const AnatomyDesc& desc, const TissueTable& tissues)
    : tissues_(tissues),
      origin_(desc.origin),
      h_(desc.voxel_size),
      r_(static_cast<float>(desc.voxel_size * 0.5)),
      joint_names_(desc.joint_names),
      root_joints_(desc.root_joints) {

    if (!(h_ > 0.0)) throw std::invalid_argument("anatomy voxel size must be positive");
    if (desc.voxels.empty()) throw std::invalid_argument("anatomy has no voxels");

    std::array<std::int32_t, 3> lo{std::numeric_limits<std::int32_t>::max(),
                                   std::numeric_limits<std::int32_t>::max(),
                                   std::numeric_limits<std::int32_t>::max()};
    std::array<std::int32_t, 3> hi{std::numeric_limits<std::int32_t>::min(),
                                   std::numeric_limits<std::int32_t>::min(),
                                   std::numeric_limits<std::int32_t>::min()};
    for (const auto& v : desc.voxels) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], v.cell[k]);
            hi[k] = std::max(hi[k], v.cell[k]);
        }
    }
    grid_min_ = lo;
    for (int k = 0; k < 3; ++k) grid_dim_[k] = hi[k] - lo[k] + 1;
    grid_.assign(static_cast<std::size_t>(grid_dim_[0]) * grid_dim_[1] * grid_dim_[2], kNone);

    const std::size_t n = desc.voxels.size();
    cell_.resize(n);
    tissue_.resize(n);
    alive_.assign(n, 1u);
    rig_enabled_.assign(n, 0u);
    joints_.resize(n);
    weights_.resize(n);
    tear_scale_.resize(n);
    rest_cx_.resize(n);
    rest_cy_.resize(n);
    rest_cz_.resize(n);
    target_x_.resize(n);
    target_y_.resize(n);
    target_z_.resize(n);
    target_m_.assign(n, {1, 0, 0, 0, 1, 0, 0, 0, 1});
    voxel_inv_mass_.resize(n);
    face_bond_.assign(n, {kNone, kNone, kNone, kNone, kNone, kNone});
    face_wounds_.assign(n, 0u);

    px_.resize(n * 8);
    py_.resize(n * 8);
    pz_.resize(n * 8);
    vx_.assign(n * 8, 0.0f);
    vy_.assign(n * 8, 0.0f);
    vz_.assign(n * 8, 0.0f);

    for (std::size_t i = 0; i < n; ++i) {
        const auto& d = desc.voxels[i];
        const auto gx = d.cell[0] - lo[0];
        const auto gy = d.cell[1] - lo[1];
        const auto gz = d.cell[2] - lo[2];
        const std::size_t gi = static_cast<std::size_t>(gx)
            + static_cast<std::size_t>(grid_dim_[0])
                * (static_cast<std::size_t>(gy) + static_cast<std::size_t>(grid_dim_[1]) * gz);
        if (grid_[gi] != kNone) throw std::invalid_argument("duplicate anatomy voxel cell");
        grid_[gi] = static_cast<std::uint32_t>(i);

        cell_[i] = d.cell;
        tissue_[i] = d.tissue;
        joints_[i] = d.joints;
        weights_[i] = d.weights;
        tear_scale_[i] = std::max(0.1f, d.tear_scale);

        float wsum = 0.0f;
        for (int k = 0; k < 4; ++k) {
            if (d.joints[k] != kNoJoint) {
                if (d.joints[k] >= joint_names_.size())
                    throw std::invalid_argument("anatomy voxel references unknown joint");
                wsum += d.weights[k];
            }
        }
        if (wsum > 1e-6f) {
            for (auto& w : weights_[i]) w /= wsum;
            rig_enabled_[i] = 1u;
        }

        const Vec3 c = origin_ + Vec3{(d.cell[0] + 0.5) * h_, (d.cell[1] + 0.5) * h_, (d.cell[2] + 0.5) * h_};
        rest_cx_[i] = static_cast<float>(c.x);
        rest_cy_[i] = static_cast<float>(c.y);
        rest_cz_[i] = static_cast<float>(c.z);
        target_x_[i] = rest_cx_[i];
        target_y_[i] = rest_cy_[i];
        target_z_[i] = rest_cz_[i];

        const double mass = tissues_[static_cast<std::size_t>(d.tissue)].density * h_ * h_ * h_;
        voxel_inv_mass_[i] = static_cast<float>(1.0 / mass);

        for (int corner = 0; corner < 8; ++corner) {
            const std::size_t p = i * 8 + static_cast<std::size_t>(corner);
            px_[p] = rest_cx_[i] + ((corner & 1) ? r_ : -r_);
            py_[p] = rest_cy_[i] + ((corner & 2) ? r_ : -r_);
            pz_[p] = rest_cz_[i] + ((corner & 4) ? r_ : -r_);
        }
    }
    ox_ = px_;
    oy_ = py_;
    oz_ = pz_;

    pose_.assign(joint_names_.size(), JointTransform{});
    kinematic_ = rig_enabled_;
    build_bonds();
    compute_rig_targets();
    topology_dirty_ = true;
}

void AnatomyBody::build_bonds() {
    bond_a_.clear();
    bond_b_.clear();
    bond_axis_.clear();
    for (int axis = 0; axis < 3; ++axis) {
        axis_begin_[axis] = bond_a_.size();
        for (std::uint32_t v = 0; v < tissue_.size(); ++v) {
            const std::uint32_t u = neighbor(v, axis, 1);
            if (u == kNone) continue;
            const std::size_t b = bond_a_.size();
            bond_a_.push_back(v);
            bond_b_.push_back(u);
            bond_axis_.push_back(static_cast<std::uint8_t>(axis));
            face_bond_[v][axis * 2 + 1] = static_cast<std::uint32_t>(b);
            face_bond_[u][axis * 2 + 0] = static_cast<std::uint32_t>(b);
        }
    }
    axis_begin_[3] = bond_a_.size();

    const std::size_t m = bond_a_.size();
    bond_active_.assign(m, 1u);
    bond_hp_.resize(m);
    bond_max_hp_.resize(m);
    bond_stiffness_.resize(m);
    bond_tear_.resize(m);
    const float area_scale = static_cast<float>((h_ / kReferenceVoxel) * (h_ / kReferenceVoxel));
    for (std::size_t b = 0; b < m; ++b) {
        const auto& ta = tissues_[static_cast<std::size_t>(tissue_[bond_a_[b]])];
        const auto& tb = tissues_[static_cast<std::size_t>(tissue_[bond_b_[b]])];
        // Interfaces fail at the weaker tissue: organs shear out of cavities,
        // fat planes separate before muscle.
        bond_max_hp_[b] = std::min(ta.cut_hp, tb.cut_hp) * area_scale;
        bond_hp_[b] = bond_max_hp_[b];
        bond_stiffness_[b] = 0.5f * (ta.bond_stiffness + tb.bond_stiffness);
        bond_tear_[b] = std::min(ta.tear_strain, tb.tear_strain)
            * 0.5f * (tear_scale_[bond_a_[b]] + tear_scale_[bond_b_[b]]);

        // Bone-to-bone bonds between different rig joints are articulations
        // (capsule + ligaments), not fused bone: tougher in tension, softer.
        const auto hard = [](Tissue t) { return t == Tissue::Bone || t == Tissue::Marrow; };
        const std::uint32_t va = bond_a_[b], vb = bond_b_[b];
        if (hard(tissue_[va]) && hard(tissue_[vb]) && joints_[va][0] != joints_[vb][0]) {
            const auto& lig = tissues_[static_cast<std::size_t>(Tissue::Tendon)];
            bond_max_hp_[b] = lig.cut_hp * area_scale;
            bond_hp_[b] = bond_max_hp_[b];
            bond_stiffness_[b] = 0.6f;
            bond_tear_[b] = 1.4f;
        }
    }
}

std::uint32_t AnatomyBody::neighbor(std::uint32_t v, int axis, int dir) const {
    std::array<std::int32_t, 3> c = cell_[v];
    c[axis] += dir ? 1 : -1;
    std::array<std::int32_t, 3> g{};
    for (int k = 0; k < 3; ++k) {
        g[k] = c[k] - grid_min_[k];
        if (g[k] < 0 || g[k] >= grid_dim_[k]) return kNone;
    }
    return grid_[static_cast<std::size_t>(g[0])
        + static_cast<std::size_t>(grid_dim_[0])
            * (static_cast<std::size_t>(g[1]) + static_cast<std::size_t>(grid_dim_[1]) * g[2])];
}

std::size_t AnatomyBody::bond_across(std::uint32_t v, int axis, int dir) const {
    const std::uint32_t b = face_bond_[v][axis * 2 + dir];
    return b == kNone ? std::numeric_limits<std::size_t>::max() : b;
}

bool AnatomyBody::face_wounded(std::uint32_t v, int axis, int dir) const {
    return (face_wounds_[v] >> (axis * 2 + dir)) & 1u;
}

std::array<std::int32_t, 3> AnatomyBody::voxel_cell(std::uint32_t v) const { return cell_[v]; }

Vec3 AnatomyBody::voxel_center(std::uint32_t v) const {
    double x = 0, y = 0, z = 0;
    const std::size_t base = static_cast<std::size_t>(v) * 8;
    for (int c = 0; c < 8; ++c) {
        x += px_[base + c];
        y += py_[base + c];
        z += pz_[base + c];
    }
    return {x / 8.0, y / 8.0, z / 8.0};
}

Vec3 AnatomyBody::voxel_rest_center(std::uint32_t v) const {
    return {rest_cx_[v], rest_cy_[v], rest_cz_[v]};
}

Vec3 AnatomyBody::voxel_velocity(std::uint32_t v) const {
    double x = 0, y = 0, z = 0;
    const std::size_t base = static_cast<std::size_t>(v) * 8;
    for (int c = 0; c < 8; ++c) {
        x += vx_[base + c];
        y += vy_[base + c];
        z += vz_[base + c];
    }
    return {x / 8.0, y / 8.0, z / 8.0};
}

Vec3 AnatomyBody::corner(std::uint32_t v, int c) const {
    const std::size_t p = static_cast<std::size_t>(v) * 8 + static_cast<std::size_t>(c);
    return {px_[p], py_[p], pz_[p]};
}

// ---------------------------------------------------------------------------
// Rig
// ---------------------------------------------------------------------------

void AnatomyBody::set_pose(const std::vector<JointTransform>& joints) {
    if (joints.size() != joint_names_.size())
        throw std::invalid_argument("pose joint count does not match anatomy rig");
    pose_ = joints;
    compute_rig_targets();
}

void AnatomyBody::compute_rig_targets() {
    for (std::size_t v = 0; v < tissue_.size(); ++v) {
        if (!rig_enabled_[v]) continue;
        const Vec3 rest{rest_cx_[v], rest_cy_[v], rest_cz_[v]};
        Vec3 t{};
        std::array<float, 9> m{};
        for (int k = 0; k < 4; ++k) {
            const auto j = joints_[v][k];
            if (j == kNoJoint || weights_[v][k] <= 0.0f) continue;
            t += pose_[j].apply(rest) * static_cast<double>(weights_[v][k]);
            for (int e = 0; e < 9; ++e) m[e] += static_cast<float>(pose_[j].m[e]) * weights_[v][k];
        }
        target_m_[v] = m;
        target_x_[v] = static_cast<float>(t.x);
        target_y_[v] = static_cast<float>(t.y);
        target_z_[v] = static_cast<float>(t.z);
    }
}

void AnatomyBody::snap_to_pose() {
    for (std::size_t v = 0; v < tissue_.size(); ++v) {
        if (!alive_[v] || !rig_enabled_[v]) continue;
        const auto& m = target_m_[v];
        for (int c = 0; c < 8; ++c) {
            const double ox = (c & 1) ? r_ : -r_;
            const double oy = (c & 2) ? r_ : -r_;
            const double oz = (c & 4) ? r_ : -r_;
            const std::size_t p = v * 8 + static_cast<std::size_t>(c);
            px_[p] = static_cast<float>(target_x_[v] + m[0] * ox + m[3] * oy + m[6] * oz);
            py_[p] = static_cast<float>(target_y_[v] + m[1] * ox + m[4] * oy + m[7] * oz);
            pz_[p] = static_cast<float>(target_z_[v] + m[2] * ox + m[5] * oy + m[8] * oz);
            vx_[p] = vy_[p] = vz_[p] = 0.0f;
        }
    }
    ox_ = px_;
    oy_ = py_;
    oz_ = pz_;
}

void AnatomyBody::release_rig() {
    rig_released_ = true;
    std::fill(rig_enabled_.begin(), rig_enabled_.end(), 0u);
    std::fill(kinematic_.begin(), kinematic_.end(), 0u);
    topology_dirty_ = true;
}

void AnatomyBody::set_sim_mode(AnatomySimMode mode) {
    mode_ = mode;
    if (mode == AnatomySimMode::Dynamic) {
        for (auto& k : kinematic_)
            if (k == 1) k = 0u;
    }
    // Switching back to Hybrid never re-freezes simulated voxels; intact voxels
    // keep whatever residency they had.
}

std::size_t AnatomyBody::activate_sphere(const Vec3& center, double radius) {
    std::size_t promoted = 0;
    bool woke = false;
    const double r2 = radius * radius;
    for (std::uint32_t v = 0; v < tissue_.size(); ++v) {
        if (!alive_[v] || !kinematic_[v]) continue;
        const bool near = kinematic_[v] == 1
            ? (length_squared(Vec3{target_x_[v], target_y_[v], target_z_[v]} - center) <= r2
               || length_squared(voxel_center(v) - center) <= r2)
            : length_squared(voxel_center(v) - center) <= r2;
        if (!near) continue;
        if (kinematic_[v] == 2) woke = true;
        kinematic_[v] = 0u;
        ++promoted;
    }
    if (woke) {
        // Wake whole sleeping islands, not just the touched voxels.
        if (topology_dirty_) rebuild_components();
        for (const auto& comp : components_) {
            bool any_awake = false, any_asleep = false;
            for (const auto v : comp.voxels) {
                any_awake |= kinematic_[v] == 0;
                any_asleep |= kinematic_[v] == 2;
            }
            if (!(any_awake && any_asleep)) continue;
            for (const auto v : comp.voxels) {
                if (kinematic_[v] == 2) {
                    kinematic_[v] = 0u;
                }
            }
        }
    }
    return promoted;
}

void AnatomyBody::update_sleep(const AnatomyStepConfig& config) {
    if (config.sleep_speed <= 0.0 || config.sleep_frames <= 0) return;
    const float limit2 = static_cast<float>(config.sleep_speed * config.sleep_speed);
    if (topology_dirty_) rebuild_components();
    // A settled pile keeps a little solver jitter in scattered voxels, so an
    // island sleeps after `sleep_frames` consecutive frames in which at least
    // 97% of its dynamic voxels are below the sleep speed.
    std::unordered_map<std::uint32_t, int> next;
    for (const auto& comp : components_) {
        if (comp.rig_authoritative || comp.voxels.empty()) continue;
        std::size_t dynamic = 0, calm = 0;
        for (const auto v : comp.voxels) {
            if (kinematic_[v] != 0) continue;
            ++dynamic;
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            float mx = 0, my = 0, mz = 0;
            for (std::size_t p = b; p < b + 8; ++p) { mx += vx_[p]; my += vy_[p]; mz += vz_[p]; }
            mx *= 0.125f; my *= 0.125f; mz *= 0.125f;
            if (mx * mx + my * my + mz * mz < limit2) ++calm;
        }
        if (dynamic == 0) continue;
        const std::uint32_t key = comp.voxels.front();
        const auto it = island_calm_.find(key);
        const int streak = static_cast<double>(calm) >= 0.97 * static_cast<double>(dynamic)
            ? (it == island_calm_.end() ? 1 : it->second + 1) : 0;
        if (streak < config.sleep_frames) {
            next[key] = streak;
            continue;
        }
        for (const auto v : comp.voxels) {
            kinematic_[v] = 2u;
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            for (std::size_t p = b; p < b + 8; ++p) vx_[p] = vy_[p] = vz_[p] = 0.0f;
        }
    }
    island_calm_ = std::move(next);
}

std::size_t AnatomyBody::dynamic_voxel_count() const {
    std::size_t n = 0;
    for (std::size_t v = 0; v < tissue_.size(); ++v)
        if (alive_[v] && !kinematic_[v]) ++n;
    return n;
}

void AnatomyBody::drive_kinematic(double dt) {
    const float inv = static_cast<float>(1.0 / dt);
    for (std::size_t v = 0; v < tissue_.size(); ++v) {
        if (!alive_[v] || kinematic_[v] != 1) continue;
        const auto& m = target_m_[v];
        const std::size_t base = v * 8;
        for (int c = 0; c < 8; ++c) {
            const float ox = (c & 1) ? r_ : -r_;
            const float oy = (c & 2) ? r_ : -r_;
            const float oz = (c & 4) ? r_ : -r_;
            const std::size_t p = base + static_cast<std::size_t>(c);
            const float nx = target_x_[v] + m[0] * ox + m[3] * oy + m[6] * oz;
            const float ny = target_y_[v] + m[1] * ox + m[4] * oy + m[7] * oz;
            const float nz = target_z_[v] + m[2] * ox + m[5] * oy + m[8] * oz;
            vx_[p] = (nx - px_[p]) * inv;
            vy_[p] = (ny - py_[p]) * inv;
            vz_[p] = (nz - pz_[p]) * inv;
            px_[p] = ox_[p] = nx;
            py_[p] = oy_[p] = ny;
            pz_[p] = oz_[p] = nz;
        }
    }
}

void AnatomyBody::apply_rig(double fraction) {
    for (const std::uint32_t v : dyn_voxels_) {
        if (!alive_[v] || !rig_enabled_[v]) continue;
        const float k = static_cast<float>(
            tissues_[static_cast<std::size_t>(tissue_[v])].rig_tone * fraction);
        const std::size_t base = v * 8;
        float cx = 0, cy = 0, cz = 0;
        for (int c = 0; c < 8; ++c) {
            cx += px_[base + c];
            cy += py_[base + c];
            cz += pz_[base + c];
        }
        const float dx = (target_x_[v] - cx * 0.125f) * k;
        const float dy = (target_y_[v] - cy * 0.125f) * k;
        const float dz = (target_z_[v] - cz * 0.125f) * k;
        for (int c = 0; c < 8; ++c) {
            px_[base + c] += dx;
            py_[base + c] += dy;
            pz_[base + c] += dz;
        }
    }
}

// ---------------------------------------------------------------------------
// Grabs
// ---------------------------------------------------------------------------

std::size_t AnatomyBody::add_grab(const Vec3& center, double radius, double stiffness) {
    AnatomyGrab g;
    g.target = center;
    g.stiffness = stiffness;
    for (std::uint32_t v = 0; v < tissue_.size(); ++v) {
        if (!alive_[v]) continue;
        const Vec3 c = voxel_center(v);
        if (length_squared(c - center) <= radius * radius) {
            g.voxels.push_back(v);
            g.offsets.push_back(c - center);
        }
    }
    activate_sphere(center, radius + activation_halo_);
    grabs_.push_back(std::move(g));
    return grabs_.size() - 1;
}

void AnatomyBody::set_grab_target(std::size_t grab, const Vec3& target) {
    grabs_.at(grab).target = target;
}

void AnatomyBody::release_grab(std::size_t grab) { grabs_.at(grab).active = false; }

void AnatomyBody::apply_grabs() {
    for (const auto& g : grabs_) {
        if (!g.active) continue;
        const float k = static_cast<float>(g.stiffness);
        for (std::size_t i = 0; i < g.voxels.size(); ++i) {
            const std::uint32_t v = g.voxels[i];
            if (!alive_[v]) continue;
            const Vec3 want = g.target + g.offsets[i];
            const Vec3 have = voxel_center(v);
            offset_voxel(v, (want - have) * static_cast<double>(k));
        }
    }
}

// ---------------------------------------------------------------------------
// Solver
// ---------------------------------------------------------------------------

void AnatomyBody::compact_active() {
    dyn_voxels_.clear();
    for (std::uint32_t v = 0; v < tissue_.size(); ++v)
        if (alive_[v] && !kinematic_[v]) dyn_voxels_.push_back(v);
    for (int axis = 0; axis < 3; ++axis) {
        auto& list = dyn_bonds_[static_cast<std::size_t>(axis)];
        list.clear();
        for (std::size_t b = axis_begin_[axis]; b < axis_begin_[axis + 1]; ++b)
            if (bond_active_[b] && (!kinematic_[bond_a_[b]] || !kinematic_[bond_b_[b]]))
                list.push_back(static_cast<std::uint32_t>(b));
    }
}

void AnatomyBody::solve_vgs(std::size_t begin, std::size_t end) {
    // Local restrict pointers: writes to particle arrays must not force reloads
    // of tissue parameters or r_ (aliasing cost ~4x in profiling).
    float* __restrict X = px_.data();
    float* __restrict Y = py_.data();
    float* __restrict Z = pz_.data();
    const std::uint32_t* __restrict list = dyn_voxels_.data();
    const Tissue* __restrict tissue = tissue_.data();
    const float r = r_;
    std::array<float, kTissueCount> stiffness{};
    for (std::size_t t = 0; t < kTissueCount; ++t)
        stiffness[t] = std::clamp(tissues_[t].shape_stiffness, 0.0f, 1.0f);

    for (std::size_t i = begin; i < end; ++i) {
        const std::size_t v = list[i];
        const float k = stiffness[static_cast<std::size_t>(tissue[v])];
        const std::size_t b = v * 8;
        V3f p[8];
        for (int c = 0; c < 8; ++c) p[c] = {X[b + c], Y[b + c], Z[b + c]};

        // Half-edge vectors along each lattice axis (average of four edges / 2),
        // Lin 2025 Alg. 2 lines 3-6.
        const V3f v0 = ((p[1] - p[0]) + (p[3] - p[2]) + (p[5] - p[4]) + (p[7] - p[6])) * 0.125f;
        const V3f v1 = ((p[2] - p[0]) + (p[3] - p[1]) + (p[6] - p[4]) + (p[7] - p[5])) * 0.125f;
        const V3f v2 = ((p[4] - p[0]) + (p[5] - p[1]) + (p[6] - p[2]) + (p[7] - p[3])) * 0.125f;
        const V3f cen = ((p[0] + p[1]) + (p[2] + p[3]) + ((p[4] + p[5]) + (p[6] + p[7]))) * 0.125f;

        // Symmetric (order-free) Gram-Schmidt. The sequential form of Alg. 2
        // lines 7-9 builds u1 from the already-updated u0, biasing the voxel's
        // rotation on every projection; iterated, that ratchets into spurious
        // spin. Here every axis is projected against the current set
        // simultaneously (Jacobi style); two passes approach the closest
        // orthogonal (Lowdin) frame.
        V3f u0 = v0, u1 = v1, u2 = v2;
        bool collapsed = false;
        for (int pass = 0; pass < 2; ++pass) {
            const float e0 = dotf(u0, u0), e1 = dotf(u1, u1), e2 = dotf(u2, u2);
            if (e0 < 1e-18f || e1 < 1e-18f || e2 < 1e-18f) { collapsed = true; break; }
            const float i0 = 0.5f / e0, i1 = 0.5f / e1, i2 = 0.5f / e2;
            const float d01 = dotf(u0, u1), d02 = dotf(u0, u2), d12 = dotf(u1, u2);
            const V3f n0 = u0 - (u1 * (d01 * i1) + u2 * (d02 * i2));
            const V3f n1 = u1 - (u0 * (d01 * i0) + u2 * (d12 * i2));
            const V3f n2 = u2 - (u0 * (d02 * i0) + u1 * (d12 * i1));
            u0 = n0;
            u1 = n1;
            u2 = n2;
        }
        if (!collapsed) {
            const float l0 = lenf(u0), l1 = lenf(u1), l2 = lenf(u2);
            collapsed = l0 < 1e-9f || l1 < 1e-9f || l2 < 1e-9f;
            if (!collapsed) {
                // Unit half-edges (lines 10-12 with beta = 1); the orthogonal
                // frame then has exact rest volume (lines 13-17 with delta = 1).
                u0 = u0 * (r / l0);
                u1 = u1 * (r / l1);
                u2 = u2 * (r / l2);
            }
        }
        if (collapsed) {
            // Degenerate: rebuild an axis-aligned cube around the centroid.
            u0 = {r, 0, 0};
            u1 = {0, r, 0};
            u2 = {0, 0, r};
        }

        // SARX addition: the published VGS keeps handedness through |det|, so a
        // voxel crushed through itself stays mirrored and rips its neighbours.
        // Re-complete the frame right-handed.
        const V3f c01 = crossf(u0, u1);
        if (dotf(c01, u2) < 0.0f) {
            const float l01 = lenf(c01);
            if (l01 > 1e-20f) u2 = c01 * (r / l01);
        }

        // Soft tissue: move a fraction k of the way to the goal shape.
        for (int c = 0; c < 8; ++c) {
            const V3f q = cen + u0 * ((c & 1) ? 1.0f : -1.0f)
                              + u1 * ((c & 2) ? 1.0f : -1.0f)
                              + u2 * ((c & 4) ? 1.0f : -1.0f);
            X[b + c] += (q.x - X[b + c]) * k;
            Y[b + c] += (q.y - Y[b + c]) * k;
            Z[b + c] += (q.z - Z[b + c]) * k;
        }
    }
}

void AnatomyBody::solve_bond_axis(int axis, std::size_t begin, std::size_t end) {
    const int bit = corner_bit(axis);
    float* __restrict X = px_.data();
    float* __restrict Y = py_.data();
    float* __restrict Z = pz_.data();
    const std::uint32_t* __restrict list = dyn_bonds_[static_cast<std::size_t>(axis)].data();
    const std::uint32_t* __restrict A = bond_a_.data();
    const std::uint32_t* __restrict B = bond_b_.data();
    const std::uint8_t* __restrict active = bond_active_.data();
    const std::uint8_t* __restrict kin = kinematic_.data();
    const float* __restrict inv_mass = solve_w_.data();
    const float* __restrict stiff = bond_stiffness_.data();
    for (std::size_t i = begin; i < end; ++i) {
        const std::size_t bi = list[i];
        if (!active[bi]) continue;
        const std::uint32_t va = A[bi];
        const std::uint32_t vb = B[bi];
        const float wa = kin[va] ? 0.0f : inv_mass[va];
        const float wb = kin[vb] ? 0.0f : inv_mass[vb];
        (void)kin;
        if (wa + wb <= 0.0f) continue;
        const float k = stiff[bi] / (wa + wb);
        const float fa = k * wa;
        const float fb = k * wb;
        for (int c = 0; c < 8; ++c) {
            if (!(c & bit)) continue;           // +axis corner of the lower voxel
            const std::size_t pa = static_cast<std::size_t>(va) * 8 + static_cast<std::size_t>(c);
            const std::size_t pb = static_cast<std::size_t>(vb) * 8 + static_cast<std::size_t>(c & ~bit);
            const float dx = X[pb] - X[pa];
            const float dy = Y[pb] - Y[pa];
            const float dz = Z[pb] - Z[pa];
            X[pa] += dx * fa; Y[pa] += dy * fa; Z[pa] += dz * fa;
            X[pb] -= dx * fb; Y[pb] -= dy * fb; Z[pb] -= dz * fb;
        }
    }
}

void AnatomyBody::collide_ground(const AnatomyStepConfig& config) {
    // Contact is resolved per voxel, not per corner: clamping individual
    // corners squashes the voxel, VGS re-expands it below the plane, and the
    // clamp turns that into upward velocity every substep (an energy pump).
    // Translating the whole voxel out keeps its shape and is dissipative.
    const float g = static_cast<float>(config.ground_height);
    const float mu = static_cast<float>(config.ground_friction);
    for (const std::uint32_t v : dyn_voxels_) {
        if (!alive_[v]) continue;
        const std::size_t b = static_cast<std::size_t>(v) * 8;
        float lowest = py_[b];
        for (std::size_t p = b + 1; p < b + 8; ++p) lowest = std::min(lowest, py_[p]);
        if (lowest >= g) continue;
        const float lift = g - lowest;
        float dx = 0.0f, dz = 0.0f;
        for (std::size_t p = b; p < b + 8; ++p) {
            dx += px_[p] - ox_[p];
            dz += pz_[p] - oz_[p];
        }
        dx *= 0.125f * mu;
        dz *= 0.125f * mu;
        for (std::size_t p = b; p < b + 8; ++p) {
            py_[p] += lift;
            px_[p] -= dx;
            pz_[p] -= dz;
        }
    }
}

std::size_t AnatomyBody::tear_overstretched() {
    compact_active();
    return tear_active_bonds();
}

std::size_t AnatomyBody::tear_active_bonds() {
    // Strain is measured between the two voxel centroids, not between the
    // coincident corners: the bond pass runs last, so corner gaps stay near
    // zero while the voxels themselves elongate under load.
    std::size_t torn = 0;
    const float h = static_cast<float>(h_);
    auto centroid = [&](std::uint32_t v) {
        const std::size_t b = static_cast<std::size_t>(v) * 8;
        V3f c{};
        for (std::size_t p = b; p < b + 8; ++p) c = c + V3f{px_[p], py_[p], pz_[p]};
        return c * 0.125f;
    };
    for (int axis = 0; axis < 3; ++axis) {
        for (const std::uint32_t bi : dyn_bonds_[static_cast<std::size_t>(axis)]) {
            if (!bond_active_[bi]) continue;
            const std::uint32_t va = bond_a_[bi];
            const std::uint32_t vb = bond_b_[bi];
            if (kinematic_[va] && kinematic_[vb]) continue;
            const float stretch = lenf(centroid(vb) - centroid(va)) - h;
            const float limit = bond_tear_[bi] * h;
            if (stretch <= limit) continue;
            // Sustained overload drains HP (about four substeps at 2x the tear
            // strain); gross separation parts the bond immediately.
            const float overload = stretch / limit;
            bond_hp_[bi] -= bond_max_hp_[bi] * 0.12f * (overload - 1.0f + 0.5f);
            if (overload > 3.0f || bond_hp_[bi] <= 0.0f) {
                const Vec3 pos = (voxel_center(va) + voxel_center(vb)) * 0.5;
                const float ha = tissues_[static_cast<std::size_t>(tissue_[va])].tear_strain;
                const float hb = tissues_[static_cast<std::size_t>(tissue_[vb])].tear_strain;
                recent_tears_.push_back({bi, pos, ha <= hb ? tissue_[va] : tissue_[vb]});
                break_bond(bi);
                ++torn;
            }
        }
    }
    return torn;
}

void AnatomyBody::step(double dt, const AnatomyStepConfig& config) {
    if (!(dt > 0.0) || config.substeps < 1 || config.iterations < 1)
        throw std::invalid_argument("invalid anatomy step");
    if (topology_dirty_) rebuild_components();
    recent_tears_.clear();
    drive_kinematic(dt);
    compact_active();

    // Adaptive substepping: keep per-substep *relative* motion under half a
    // voxel. Rigid translation needs no substeps, so the estimate uses the
    // speed difference across live bonds, plus absolute speed only for voxels
    // near the ground plane.
    int substeps = config.substeps;
    {
        auto mean_v = [&](std::uint32_t v) {
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            return V3f{(vx_[b] + vx_[b + 7]) * 0.5f, (vy_[b] + vy_[b + 7]) * 0.5f, (vz_[b] + vz_[b + 7]) * 0.5f};
        };
        float rel2 = 0.0f;
        for (const auto& list : dyn_bonds_)
            for (const std::uint32_t bi : list) {
                if (!bond_active_[bi]) continue;
                const V3f d = mean_v(bond_a_[bi]) - mean_v(bond_b_[bi]);
                rel2 = std::max(rel2, dotf(d, d));
            }
        const float near = static_cast<float>(config.ground_height + 3.0 * h_);
        for (const std::uint32_t v : dyn_voxels_) {
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            const V3f m = mean_v(v);
            if (py_[b] + std::min(0.0f, m.y) * static_cast<float>(dt) < near) rel2 = std::max(rel2, dotf(m, m));
        }
        const double travel = std::sqrt(static_cast<double>(rel2)) * dt;
        const int need = static_cast<int>(std::ceil(travel / (0.5 * h_)));
        substeps = std::clamp(need, config.substeps, std::max(config.substeps, config.max_substeps));
    }
    last_substeps_ = substeps;

    WorkerPool& pool = shared_pool(config.threads);
    ground_height_ = static_cast<float>(config.ground_height);
    stack_k_ = static_cast<float>(config.stack_mass_scaling);
    const float hs = static_cast<float>(dt / substeps);
    const float gx = static_cast<float>(config.gravity.x) * hs;
    const float gy = static_cast<float>(config.gravity.y) * hs;
    const float gz = static_cast<float>(config.gravity.z) * hs;
    const float keep = static_cast<float>(config.velocity_retention);
    const float inv = 1.0f / hs;
    const std::size_t nd = dyn_voxels_.size();

    const std::function<void(std::size_t, std::size_t)> integrate = [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            const std::size_t base = static_cast<std::size_t>(dyn_voxels_[i]) * 8;
            for (std::size_t p = base; p < base + 8; ++p) {
                vx_[p] += gx; vy_[p] += gy; vz_[p] += gz;
                ox_[p] = px_[p]; oy_[p] = py_[p]; oz_[p] = pz_[p];
                px_[p] += vx_[p] * hs; py_[p] += vy_[p] * hs; pz_[p] += vz_[p] * hs;
            }
        }
    };
    const float damp = static_cast<float>(std::clamp(config.deformation_damping, 0.0, 1.0));
    const std::function<void(std::size_t, std::size_t)> velocities = [&](std::size_t b, std::size_t e) {
        for (std::size_t i = b; i < e; ++i) {
            const std::size_t base = static_cast<std::size_t>(dyn_voxels_[i]) * 8;
            float mx = 0, my = 0, mz = 0;
            for (std::size_t p = base; p < base + 8; ++p) {
                vx_[p] = (px_[p] - ox_[p]) * inv * keep;
                vy_[p] = (py_[p] - oy_[p]) * inv * keep;
                vz_[p] = (pz_[p] - oz_[p]) * inv * keep;
                mx += vx_[p]; my += vy_[p]; mz += vz_[p];
            }
            mx *= 0.125f; my *= 0.125f; mz *= 0.125f;
            for (std::size_t p = base; p < base + 8; ++p) {
                vx_[p] -= (vx_[p] - mx) * damp;
                vy_[p] -= (vy_[p] - my) * damp;
                vz_[p] -= (vz_[p] - mz) * damp;
            }
        }
    };
    const std::function<void(std::size_t, std::size_t)> vgs = [&](std::size_t b, std::size_t e) { solve_vgs(b, e); };
    std::array<std::function<void(std::size_t, std::size_t)>, 3> bonds;
    for (int axis = 0; axis < 3; ++axis)
        bonds[static_cast<std::size_t>(axis)] = [this, axis](std::size_t b, std::size_t e) { solve_bond_axis(axis, b, e); };

    for (int s = 0; s < substeps; ++s) {
        pool.run(nd, integrate);

        if (config.enable_rig && !rig_released_) apply_rig(1.0);
        apply_grabs();
        refresh_solve_weights();

        for (int it = 0; it < config.iterations; ++it) {
            pool.run(nd, vgs);
            // Bone fragments are matched before the bond pass so that tissue
            // coupling (and through it the ground) still pushes back on bone.
            if (config.bone_rigidity > 0.0) {
                solve_bone_clusters(static_cast<float>(config.bone_rigidity));
                for (int sweep = 0; sweep < 3; ++sweep) solve_bone_joints();
            }
            // Ground before the bond passes: with stack mass scaling and the
            // y-bonds ordered bottom-to-top, ground support climbs the whole
            // column in one Gauss-Seidel sweep instead of one layer/iteration.
            collide_ground(config);
            for (std::size_t axis = 0; axis < 3; ++axis) pool.run(dyn_bonds_[axis].size(), bonds[axis]);
            collide_ground(config);
        }

        collide_ground(config);
        if (config.enable_component_contacts && (s % 2 == 0 || s + 1 == substeps)) solve_component_contacts();
        pool.run(nd, velocities);

        if (config.enable_tearing) tear_active_bonds();
    }
    if (topology_dirty_) rebuild_components();
    update_sleep(config);
}

void AnatomyBody::solve_component_contacts() {
    // Uniform hash over live voxel centres (cell = voxel size). Only pairs from
    // different components interact; bonded neighbours are always same-component.
    const std::size_t n = tissue_.size();
    if (components_.size() < 2) return;
    const double cell = h_;
    std::size_t table = 1;
    while (table < n * 2) table <<= 1;
    std::vector<std::uint32_t> head(table, kNone);
    std::vector<std::uint32_t> next(n, kNone);
    std::vector<Vec3> centers(n);
    auto key = [&](std::int64_t x, std::int64_t y, std::int64_t z) {
        std::uint64_t k = static_cast<std::uint64_t>(x) * 73856093ull
            ^ static_cast<std::uint64_t>(y) * 19349663ull
            ^ static_cast<std::uint64_t>(z) * 83492791ull;
        return static_cast<std::size_t>(k & (table - 1));
    };
    auto coord = [&](double x) { return static_cast<std::int64_t>(std::floor(x / cell)); };
    for (std::uint32_t v = 0; v < n; ++v) {
        if (!alive_[v]) continue;
        centers[v] = voxel_center(v);
        const std::size_t k = key(coord(centers[v].x), coord(centers[v].y), coord(centers[v].z));
        next[v] = head[k];
        head[k] = v;
    }
    const double min_d = 0.9 * h_;
    for (const std::uint32_t v : dyn_voxels_) {
        if (!alive_[v]) continue;
        const Vec3 c = centers[v];
        const std::int64_t cx = coord(c.x), cy = coord(c.y), cz = coord(c.z);
        for (std::int64_t dz = -1; dz <= 1; ++dz)
            for (std::int64_t dy = -1; dy <= 1; ++dy)
                for (std::int64_t dx = -1; dx <= 1; ++dx) {
                    for (std::uint32_t u = head[key(cx + dx, cy + dy, cz + dz)]; u != kNone; u = next[u]) {
                        if (u == v || component_of_[u] == component_of_[v]) continue;
                        // Each dynamic pair is resolved once (from the lower id).
                        if (kinematic_[u] == 0 && u < v) continue;
                        const Vec3 d = centers[v] - centers[u];
                        const double dist2 = length_squared(d);
                        if (dist2 >= min_d * min_d || dist2 < 1e-16) continue;
                        const double dist = std::sqrt(dist2);
                        const double wv = effective_inv_mass(v), wu = effective_inv_mass(u);
                        if (wv + wu <= 0.0) continue;
                        // Capped so overlapping fragments separate over a few
                        // substeps instead of being launched.
                        const double push = std::min(min_d - dist, 0.2 * h_);
                        const Vec3 corr = d * (push / dist / (wv + wu));
                        offset_voxel(v, corr * wv);
                        offset_voxel(u, corr * (-wu));
                        centers[v] += corr * wv;
                        centers[u] -= corr * wu;
                    }
                }
    }
}

namespace {

using Quat4 = std::array<double, 4>;

std::array<double, 9> quat_matrix(const Quat4& q) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    // Column-major.
    return {1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w),
            2 * (x * y - z * w), 1 - 2 * (x * x + z * z), 2 * (y * z + x * w),
            2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)};
}

// Rotation extraction (Mueller, Bender, Chentanez, Macklin 2016), warm started.
void extract_rotation(const std::array<double, 9>& A, Quat4& q, int iterations) {
    for (int it = 0; it < iterations; ++it) {
        const auto R = quat_matrix(q);
        Vec3 omega{};
        double denom = 0.0;
        for (int c = 0; c < 3; ++c) {
            const Vec3 r{R[c * 3], R[c * 3 + 1], R[c * 3 + 2]};
            const Vec3 a{A[c * 3], A[c * 3 + 1], A[c * 3 + 2]};
            omega += cross(r, a);
            denom += dot(r, a);
        }
        omega = omega / (std::abs(denom) + 1e-9);
        const double w = length(omega);
        if (w < 1e-7) break;
        const Vec3 axis = omega / w;
        const double s = std::sin(0.5 * w), c = std::cos(0.5 * w);
        const Quat4 dq{axis.x * s, axis.y * s, axis.z * s, c};
        Quat4 nq{
            dq[3] * q[0] + dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1],
            dq[3] * q[1] - dq[0] * q[2] + dq[1] * q[3] + dq[2] * q[0],
            dq[3] * q[2] + dq[0] * q[1] - dq[1] * q[0] + dq[2] * q[3],
            dq[3] * q[3] - dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2]};
        const double len = std::sqrt(nq[0] * nq[0] + nq[1] * nq[1] + nq[2] * nq[2] + nq[3] * nq[3]);
        for (auto& e : nq) e /= len;
        q = nq;
    }
}

} // namespace

void AnatomyBody::rebuild_bone_clusters() {
    const std::size_t n = tissue_.size();
    std::vector<std::uint32_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0u);
    auto find = [&](std::uint32_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };
    auto hard = [&](std::uint32_t v) {
        return alive_[v] && (tissue_[v] == Tissue::Bone || tissue_[v] == Tissue::Marrow);
    };
    for (std::size_t b = 0; b < bond_a_.size(); ++b) {
        if (!bond_active_[b]) continue;
        const std::uint32_t va = bond_a_[b], vb = bond_b_[b];
        if (!hard(va) || !hard(vb) || joints_[va][0] != joints_[vb][0]) continue;
        const std::uint32_t ra = find(va), rb = find(vb);
        if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb);
    }

    // Preserve warm-start rotations by the cluster's lowest voxel id.
    std::vector<std::pair<std::uint32_t, Quat4>> previous;
    for (const auto& c : bone_clusters_)
        if (!c.voxels.empty()) previous.push_back({c.voxels.front(), c.q});

    bone_clusters_.clear();
    std::vector<std::uint32_t> slot(n, kNone);
    for (std::uint32_t v = 0; v < n; ++v) {
        if (!hard(v)) continue;
        const std::uint32_t r = find(v);
        if (slot[r] == kNone) {
            slot[r] = static_cast<std::uint32_t>(bone_clusters_.size());
            bone_clusters_.emplace_back();
        }
        bone_clusters_[slot[r]].voxels.push_back(v);
    }
    std::vector<std::uint32_t> cluster_of(n, kNone);
    for (std::uint32_t ci = 0; ci < bone_clusters_.size(); ++ci)
        for (const auto v : bone_clusters_[ci].voxels) cluster_of[v] = ci;

    // Articulations: active bone-bone bonds across different rig joints.
    bone_joints_.clear();
    {
        std::vector<std::pair<std::uint64_t, Vec3>> acc;
        for (std::size_t b = 0; b < bond_a_.size(); ++b) {
            if (!bond_active_[b]) continue;
            const std::uint32_t va = bond_a_[b], vb = bond_b_[b];
            if (!hard(va) || !hard(vb) || joints_[va][0] == joints_[vb][0]) continue;
            const std::uint32_t ca = cluster_of[va], cb = cluster_of[vb];
            if (ca == kNone || cb == kNone || ca == cb) continue;
            const std::uint64_t key = (static_cast<std::uint64_t>(std::min(ca, cb)) << 32) | std::max(ca, cb);
            acc.push_back({key, (voxel_rest_center(va) + voxel_rest_center(vb)) * 0.5});
        }
        std::sort(acc.begin(), acc.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
        for (std::size_t i = 0; i < acc.size();) {
            std::size_t j = i;
            Vec3 sum{};
            while (j < acc.size() && acc[j].first == acc[i].first) sum += acc[j++].second;
            BoneJoint joint;
            joint.a = static_cast<std::uint32_t>(acc[i].first >> 32);
            joint.b = static_cast<std::uint32_t>(acc[i].first & 0xFFFFFFFFu);
            joint.rest_anchor = sum / static_cast<double>(j - i);
            bone_joints_.push_back(joint);
            i = j;
        }
    }

    for (auto& c : bone_clusters_) {
        Vec3 com{};
        double mass = 0.0;
        for (const auto v : c.voxels) {
            com += voxel_rest_center(v);
            mass += 1.0 / voxel_inv_mass_[v];
        }
        com = com / static_cast<double>(c.voxels.size());
        c.rest_com = com;
        c.com = com;
        c.inv_mass = mass > 0.0 ? 1.0 / mass : 0.0;
        c.rest_offsets.reserve(c.voxels.size());
        for (const auto v : c.voxels) c.rest_offsets.push_back(voxel_rest_center(v) - com);
        for (const auto& [id, q] : previous) {
            if (std::binary_search(c.voxels.begin(), c.voxels.end(), id)) {
                c.q = q;
                break;
            }
        }
    }
    bone_clusters_dirty_ = false;
}

void AnatomyBody::refresh_solve_weights() {
    solve_w_.resize(tissue_.size());
    for (const std::uint32_t v : dyn_voxels_) {
        const std::size_t b = static_cast<std::size_t>(v) * 8;
        const float y = 0.125f * (py_[b] + py_[b + 1] + py_[b + 2] + py_[b + 3] + py_[b + 4] + py_[b + 5] + py_[b + 6] + py_[b + 7]);
        const float hgt = std::max(0.0f, y - ground_height_);
        solve_w_[v] = voxel_inv_mass_[v] * (stack_k_ > 0.0f ? std::exp(std::min(stack_k_ * hgt, 20.0f)) : 1.0f);
    }
}

void AnatomyBody::solve_bone_joints() {
    // Translational ball joints between fragment frames (PBD, mass weighted).
    // Kinematic (rig-driven) fragments are immovable anchors.
    for (const auto& j : bone_joints_) {
        auto& A = bone_clusters_[j.a];
        auto& B = bone_clusters_[j.b];
        // Shock propagation: fragments resting on the ground act ~20x heavier
        // so the skeleton stacks on them instead of driving them into it.
        auto scale = [&](const BoneCluster& c) {
            const double hgt = std::max(0.0, c.com.y - ground_height_);
            return stack_k_ > 0.0f ? std::exp(std::min(static_cast<double>(stack_k_) * hgt, 20.0)) : 1.0;
        };
        const double wa = A.kinematic ? 0.0 : A.inv_mass * (A.grounded ? 0.05 : 1.0) * scale(A);
        const double wb = B.kinematic ? 0.0 : B.inv_mass * (B.grounded ? 0.05 : 1.0) * scale(B);
        if (wa + wb <= 0.0) continue;
        auto anchor = [](const BoneCluster& c, const Vec3& rest) {
            const Vec3 d = rest - c.rest_com;
            const auto& R = c.R;
            return c.com + Vec3{R[0] * d.x + R[3] * d.y + R[6] * d.z,
                                R[1] * d.x + R[4] * d.y + R[7] * d.z,
                                R[2] * d.x + R[5] * d.y + R[8] * d.z};
        };
        const Vec3 err = anchor(B, j.rest_anchor) - anchor(A, j.rest_anchor);
        const Vec3 da = err * (wa / (wa + wb));
        const Vec3 db = err * (-wb / (wa + wb));
        if (wa > 0.0) {
            for (const auto v : A.voxels) if (!kinematic_[v]) offset_voxel(v, da);
            A.com += da;
        }
        if (wb > 0.0) {
            for (const auto v : B.voxels) if (!kinematic_[v]) offset_voxel(v, db);
            B.com += db;
        }
    }
}

void AnatomyBody::solve_bone_clusters(float stiffness) {
    if (bone_clusters_dirty_) rebuild_bone_clusters();
    for (auto& c : bone_clusters_) {
        bool any_dynamic = false;
        for (const auto v : c.voxels) any_dynamic |= kinematic_[v] == 0;
        c.kinematic = !any_dynamic;
        c.grounded = false;
        for (const auto v : c.voxels) {
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            for (std::size_t p = b; p < b + 8 && !c.grounded; ++p)
                c.grounded = py_[p] <= ground_height_ + r_;
            if (c.grounded) break;
        }
        if (!any_dynamic || c.voxels.size() < 2) {
            if (!c.voxels.empty()) c.com = voxel_center(c.voxels.front()) - c.rest_offsets.front();
            continue;
        }

        Vec3 com{};
        std::vector<Vec3> centers(c.voxels.size());
        for (std::size_t i = 0; i < c.voxels.size(); ++i) {
            centers[i] = voxel_center(c.voxels[i]);
            com += centers[i];
        }
        com = com / static_cast<double>(c.voxels.size());

        std::array<double, 9> A{};
        for (std::size_t i = 0; i < c.voxels.size(); ++i) {
            const Vec3 p = centers[i] - com;
            const Vec3& r = c.rest_offsets[i];
            // A = sum p r^T (column-major: column k = p * r_k).
            A[0] += p.x * r.x; A[1] += p.y * r.x; A[2] += p.z * r.x;
            A[3] += p.x * r.y; A[4] += p.y * r.y; A[5] += p.z * r.y;
            A[6] += p.x * r.z; A[7] += p.y * r.z; A[8] += p.z * r.z;
        }
        // Include each voxel's own frame so single-row bones keep their twist.
        for (const auto v : c.voxels) {
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            const Vec3 e0{(px_[b + 1] - px_[b]) * 0.25, (py_[b + 1] - py_[b]) * 0.25, (pz_[b + 1] - pz_[b]) * 0.25};
            const Vec3 e1{(px_[b + 2] - px_[b]) * 0.25, (py_[b + 2] - py_[b]) * 0.25, (pz_[b + 2] - pz_[b]) * 0.25};
            const Vec3 e2{(px_[b + 4] - px_[b]) * 0.25, (py_[b + 4] - py_[b]) * 0.25, (pz_[b + 4] - pz_[b]) * 0.25};
            const double s = 0.5 * h_;
            A[0] += e0.x * s; A[1] += e0.y * s; A[2] += e0.z * s;
            A[3] += e1.x * s; A[4] += e1.y * s; A[5] += e1.z * s;
            A[6] += e2.x * s; A[7] += e2.y * s; A[8] += e2.z * s;
        }
        // Warm-started; early exit keeps the common case at 1-3 iterations,
        // while a cold start (new fragment, large rotation) still converges.
        extract_rotation(A, c.q, 40);
        const auto R = quat_matrix(c.q);
        c.R = R;
        c.com = com;
        auto rot = [&](const Vec3& x) {
            return Vec3{R[0] * x.x + R[3] * x.y + R[6] * x.z,
                        R[1] * x.x + R[4] * x.y + R[7] * x.z,
                        R[2] * x.x + R[5] * x.y + R[8] * x.z};
        };
        for (std::size_t i = 0; i < c.voxels.size(); ++i) {
            const std::uint32_t v = c.voxels[i];
            if (kinematic_[v]) continue;
            const Vec3 target = com + rot(c.rest_offsets[i]);
            const std::size_t b = static_cast<std::size_t>(v) * 8;
            for (int k = 0; k < 8; ++k) {
                const Vec3 off{(k & 1) ? r_ : -r_, (k & 2) ? r_ : -r_, (k & 4) ? r_ : -r_};
                const Vec3 t = target + rot(off);
                const std::size_t p = b + static_cast<std::size_t>(k);
                px_[p] += (static_cast<float>(t.x) - px_[p]) * stiffness;
                py_[p] += (static_cast<float>(t.y) - py_[p]) * stiffness;
                pz_[p] += (static_cast<float>(t.z) - pz_[p]) * stiffness;
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Topology
// ---------------------------------------------------------------------------

void AnatomyBody::break_bond(std::size_t bond) {
    if (!bond_active_[bond]) return;
    bond_active_[bond] = 0u;
    bone_clusters_dirty_ = true;
    bond_hp_[bond] = 0.0f;
    const int axis = bond_axis_[bond];
    face_wounds_[bond_a_[bond]] |= static_cast<std::uint8_t>(1u << (axis * 2 + 1));
    face_wounds_[bond_b_[bond]] |= static_cast<std::uint8_t>(1u << (axis * 2 + 0));
    mark_topology_dirty();
}

void AnatomyBody::kill_voxel(std::uint32_t v) {
    if (!alive_[v]) return;
    alive_[v] = 0u;
    bone_clusters_dirty_ = true;
    for (int slot = 0; slot < 6; ++slot) {
        const std::uint32_t b = face_bond_[v][slot];
        if (b != kNone) break_bond(b);
    }
    mark_topology_dirty();
}

void AnatomyBody::rebuild_components() {
    const std::size_t n = tissue_.size();
    std::vector<std::uint32_t> parent(n);
    std::iota(parent.begin(), parent.end(), 0u);
    auto find = [&](std::uint32_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };
    for (std::size_t b = 0; b < bond_a_.size(); ++b) {
        if (!bond_active_[b]) continue;
        const std::uint32_t ra = find(bond_a_[b]);
        const std::uint32_t rb = find(bond_b_[b]);
        if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb);
    }

    components_.clear();
    component_of_.assign(n, kNone);
    std::vector<std::uint32_t> root_to_component(n, kNone);
    for (std::uint32_t v = 0; v < n; ++v) {
        if (!alive_[v]) continue;
        const std::uint32_t r = find(v);
        if (root_to_component[r] == kNone) {
            root_to_component[r] = static_cast<std::uint32_t>(components_.size());
            components_.emplace_back();
        }
        const std::uint32_t ci = root_to_component[r];
        component_of_[v] = ci;
        auto& comp = components_[ci];
        comp.voxels.push_back(v);
        const double m = 1.0 / voxel_inv_mass_[v];
        comp.mass += m;
        comp.linear_momentum += voxel_velocity(v) * m;

    }

    // Rig authority: the component holding the most live root-joint bone keeps
    // the rig. A cut through the root bone itself therefore leaves exactly one
    // animated component rather than two competing ones.
    if (!rig_released_) {
        std::vector<std::size_t> root_bone(components_.size(), 0);
        for (std::uint32_t v = 0; v < n; ++v) {
            if (!alive_[v] || !rig_enabled_[v]) continue;
            if (tissue_[v] != Tissue::Bone && tissue_[v] != Tissue::Marrow) continue;
            int best = 0;
            for (int k = 1; k < 4; ++k)
                if (weights_[v][k] > weights_[v][best]) best = k;
            if (std::find(root_joints_.begin(), root_joints_.end(), joints_[v][best]) != root_joints_.end())
                ++root_bone[component_of_[v]];
        }
        std::size_t winner = components_.size();
        for (std::size_t c = 0; c < components_.size(); ++c)
            if (root_bone[c] > 0 && (winner == components_.size() || root_bone[c] > root_bone[winner]))
                winner = c;
        if (winner < components_.size()) components_[winner].rig_authoritative = true;
    }

    // A component that lost every root anchor becomes a free physical island.
    // Authority is never regained (matches SARX Body semantics).
    for (const auto& comp : components_) {
        if (comp.rig_authoritative) continue;
        for (const auto v : comp.voxels) {
            rig_enabled_[v] = 0u;
            if (kinematic_[v] == 1) kinematic_[v] = 0u;
        }
    }
    topology_dirty_ = false;
}

const std::vector<AnatomyComponent>& AnatomyBody::components() {
    if (topology_dirty_) rebuild_components();
    return components_;
}

std::size_t AnatomyBody::component_of(std::uint32_t voxel) {
    if (topology_dirty_) rebuild_components();
    const auto c = component_of_[voxel];
    return c == kNone ? std::numeric_limits<std::size_t>::max() : c;
}

// ---------------------------------------------------------------------------
// Momentum helpers
// ---------------------------------------------------------------------------

void AnatomyBody::add_voxel_velocity(std::uint32_t v, const Vec3& dv) {
    const std::size_t b = static_cast<std::size_t>(v) * 8;
    for (int c = 0; c < 8; ++c) {
        vx_[b + c] += static_cast<float>(dv.x);
        vy_[b + c] += static_cast<float>(dv.y);
        vz_[b + c] += static_cast<float>(dv.z);
    }
}

void AnatomyBody::offset_voxel(std::uint32_t v, const Vec3& dx) {
    const std::size_t b = static_cast<std::size_t>(v) * 8;
    for (int c = 0; c < 8; ++c) {
        px_[b + c] += static_cast<float>(dx.x);
        py_[b + c] += static_cast<float>(dx.y);
        pz_[b + c] += static_cast<float>(dx.z);
    }
}

Vec3 AnatomyBody::total_linear_momentum() const {
    Vec3 p{};
    for (std::uint32_t v = 0; v < tissue_.size(); ++v) {
        if (!alive_[v]) continue;
        p += voxel_velocity(v) * (1.0 / voxel_inv_mass_[v]);
    }
    return p;
}

double AnatomyBody::total_mass() const {
    double m = 0.0;
    for (std::uint32_t v = 0; v < tissue_.size(); ++v)
        if (alive_[v]) m += 1.0 / voxel_inv_mass_[v];
    return m;
}

// ---------------------------------------------------------------------------
// Blades
// ---------------------------------------------------------------------------

BladeResult AnatomyBody::apply_blade(const BladeStroke& stroke) {
    BladeResult result;
    if (stroke.poses.size() < 2 || !(stroke.energy > 0.0) || !(stroke.sharpness > 0.0))
        return result;

    const std::size_t n = tissue_.size();
    std::vector<Vec3> centers(n);
    for (std::uint32_t v = 0; v < n; ++v)
        if (alive_[v]) centers[v] = voxel_center(v);

    struct Hit {
        double key;
        std::size_t bond;
        Vec3 point;
        Vec3 normal;
        Vec3 motion;
    };
    std::vector<Hit> hits;

    for (std::size_t k = 0; k + 1 < stroke.poses.size(); ++k) {
        const Vec3 h0 = stroke.poses[k].hilt;
        const Vec3 t0 = stroke.poses[k].tip;
        const Vec3 h1 = stroke.poses[k + 1].hilt;
        const Vec3 t1 = stroke.poses[k + 1].tip;

        Vec3 lo{std::min({h0.x, t0.x, h1.x, t1.x}), std::min({h0.y, t0.y, h1.y, t1.y}), std::min({h0.z, t0.z, h1.z, t1.z})};
        Vec3 hi{std::max({h0.x, t0.x, h1.x, t1.x}), std::max({h0.y, t0.y, h1.y, t1.y}), std::max({h0.z, t0.z, h1.z, t1.z})};
        lo -= Vec3{h_ * 1.5, h_ * 1.5, h_ * 1.5};
        hi += Vec3{h_ * 1.5, h_ * 1.5, h_ * 1.5};

        const Vec3 patch_normal = normalized(cross(t0 - h0, h1 - h0) + cross(t1 - h1, t1 - t0));

        for (std::size_t b = 0; b < bond_a_.size(); ++b) {
            if (!bond_active_[b]) continue;
            const Vec3& ca = centers[bond_a_[b]];
            const Vec3& cb = centers[bond_b_[b]];
            const Vec3 mid = (ca + cb) * 0.5;
            if (mid.x < lo.x || mid.y < lo.y || mid.z < lo.z || mid.x > hi.x || mid.y > hi.y || mid.z > hi.z)
                continue;

            double t = 0, u = 0, w = 0;
            double s_param = 0.0, u_param = 0.0;
            bool hit = false;
            // Triangle 1: (h0, t0, t1) -> s = w, u = u + w.
            if (segment_triangle(ca, cb, h0, t0, t1, t, u, w)) {
                s_param = w;
                u_param = u + w;
                hit = true;
            } else if (segment_triangle(ca, cb, h0, t1, h1, t, u, w)) {
                // Triangle 2: (h0, t1, h1) -> s = u + w, u = u.
                s_param = u + w;
                u_param = u;
                hit = true;
            }
            if (!hit) continue;
            const Vec3 point = ca + (cb - ca) * t;
            const Vec3 motion = (h1 - h0) * (1.0 - u_param) + (t1 - t0) * u_param;
            hits.push_back({static_cast<double>(k) + s_param, b, point, patch_normal, motion});
        }
    }

    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) {
        if (a.key != b.key) return a.key < b.key;
        return a.bond < b.bond;
    });

    double energy = stroke.energy;
    result.stop_parameter = static_cast<double>(stroke.poses.size() - 1);
    result.stop_point = stroke.poses.back().tip;
    const double dt = stroke.duration > 0.0 ? stroke.duration : 1.0 / 60.0;

    for (const Hit& hit : hits) {
        if (!bond_active_[hit.bond]) continue;
        ++result.bonds_crossed;
        const double cost = bond_hp_[hit.bond] / stroke.sharpness;
        const std::uint32_t va = bond_a_[hit.bond];
        const std::uint32_t vb = bond_b_[hit.bond];
        const Tissue weaker =
            tissues_[static_cast<std::size_t>(tissue_[va])].cut_hp
                <= tissues_[static_cast<std::size_t>(tissue_[vb])].cut_hp
            ? tissue_[va] : tissue_[vb];
        const Tissue harder = weaker == tissue_[va] ? tissue_[vb] : tissue_[va];

        double spent = 0.0;
        if (kinematic_[va] || kinematic_[vb]) activate_sphere(hit.point, activation_halo_);
        if (energy >= cost) {
            spent = cost;
            energy -= cost;
            break_bond(hit.bond);
            ++result.bonds_parted;
            ++result.parted_by_tissue[static_cast<std::size_t>(weaker)];

            // Wedge the faces apart along the blade normal.
            const Vec3 ab = centers[vb] - centers[va];
            const double side = dot(hit.normal, ab) >= 0.0 ? 1.0 : -1.0;
            const Vec3 push = hit.normal * (side * stroke.wedge * h_ * 0.5);
            offset_voxel(va, -push);
            offset_voxel(vb, push);
        } else {
            spent = energy;
            bond_hp_[hit.bond] = static_cast<float>(
                std::max(0.0, bond_hp_[hit.bond] - energy * stroke.sharpness));
            energy = 0.0;
            ++result.bonds_damaged;
        }

        // Momentum carried into the tissue along the swing.
        if (spent > 0.0) {
            const double speed = length(hit.motion) / dt;
            if (speed > 1e-9) {
                const Vec3 dir = hit.motion / (speed * dt);
                const double impulse = stroke.momentum_transfer * spent / std::max(speed, 1.0);
                for (const std::uint32_t v : {va, vb})
                    add_voxel_velocity(v, dir * (0.5 * impulse * voxel_inv_mass_[v]));
            }
        }

        if (energy <= 0.0) {
            result.lodged = true;
            result.stop_parameter = hit.key;
            result.stop_point = hit.point;
            result.stopped_in = harder;
            break;
        }
    }
    result.energy_spent = stroke.energy - energy;
    return result;
}

// ---------------------------------------------------------------------------
// Ballistics
// ---------------------------------------------------------------------------

BulletResult AnatomyBody::apply_bullet(const BulletShot& shot) {
    BulletResult result;
    const Vec3 dir = normalized(shot.direction);
    if (length_squared(dir) < 0.5 || !(shot.speed > 0.0) || !(shot.mass > 0.0)) return result;

    const double channel = 0.5 * h_ + shot.radius;
    struct Hit {
        double t;
        std::uint32_t v;
    };
    std::vector<Hit> hits;
    const std::size_t n = tissue_.size();
    std::vector<Vec3> centers(n);
    for (std::uint32_t v = 0; v < n; ++v) {
        if (!alive_[v]) continue;
        const Vec3 c = voxel_center(v);
        centers[v] = c;
        const double t = dot(c - shot.origin, dir);
        if (t < 0.0 || t > shot.max_distance) continue;
        const Vec3 closest = shot.origin + dir * t;
        if (length_squared(c - closest) <= channel * channel) hits.push_back({t, v});
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) {
        return a.t != b.t ? a.t < b.t : a.v < b.v;
    });

    double energy = 0.5 * shot.mass * shot.speed * shot.speed;
    result.energy_in = energy;
    const double length_scale = h_ / kReferenceVoxel;
    Vec3 last_point = shot.origin;
    std::vector<std::pair<Vec3, double>> deposits;  // (point, energy)

    for (const Hit& hit : hits) {
        if (!alive_[hit.v]) continue;
        const Tissue tissue = tissue_[hit.v];
        const double cost = tissues_[static_cast<std::size_t>(tissue)].ballistic_cost * length_scale;
        const Vec3 point = shot.origin + dir * hit.t;
        if (!result.entered) {
            result.entered = true;
            result.entry = point;
        }
        activate_sphere(point, activation_halo_);
        last_point = point;

        if (energy >= cost) {
            energy -= cost;
            deposits.push_back({point, cost});
            const double speed_here = std::sqrt(2.0 * energy / shot.mass);
            AnatomyDebris d;
            d.position = centers[hit.v];
            d.tissue = tissue;
            const Vec3 jitter{hash_unit(hit.v * 3 + 0), hash_unit(hit.v * 3 + 1), hash_unit(hit.v * 3 + 2)};
            d.velocity = voxel_velocity(hit.v)
                + dir * std::min(40.0, 0.08 * speed_here)
                + jitter * std::min(6.0, 0.02 * speed_here);
            result.debris.push_back(d);
            std::size_t live_bonds = 0;
            for (int slot = 0; slot < 6; ++slot) {
                const std::uint32_t b = face_bond_[hit.v][slot];
                if (b != kNone && bond_active_[b]) ++live_bonds;
            }
            result.bonds_broken += live_bonds;
            kill_voxel(hit.v);
            ++result.voxels_destroyed;
            ++result.destroyed_by_tissue[static_cast<std::size_t>(tissue)];

            // Hard tissue shatters: fracture bonds around the impact.
            if (tissue == Tissue::Bone || tissue == Tissue::Marrow) {
                for (int slot = 0; slot < 6; ++slot) {
                    const std::uint32_t nb = neighbor(hit.v, slot / 2, slot % 2);
                    if (nb == kNone || !alive_[nb]) continue;
                    for (int s2 = 0; s2 < 6; ++s2) {
                        const std::uint32_t b2 = face_bond_[nb][s2];
                        if (b2 == kNone || !bond_active_[b2]) continue;
                        const Tissue t2 = tissue_[bond_a_[b2] == nb ? bond_b_[b2] : bond_a_[b2]];
                        if (t2 == Tissue::Bone || t2 == Tissue::Marrow) {
                            bond_hp_[b2] -= static_cast<float>(0.5 * cost);
                            if (bond_hp_[b2] <= 0.0f) {
                                break_bond(b2);
                                ++result.bonds_broken;
                            }
                        }
                    }
                }
            }
        } else {
            deposits.push_back({point, energy});
            energy = 0.0;
            result.lodged = true;
            result.exit_point = point;
            // Lodged round still damages the bonds of the voxel it stopped in.
            for (int slot = 0; slot < 6; ++slot) {
                const std::uint32_t b = face_bond_[hit.v][slot];
                if (b != kNone && bond_active_[b])
                    bond_hp_[b] = std::max(0.0f, bond_hp_[b] - static_cast<float>(0.25 * cost));
            }
            break;
        }
    }

    // Momentum lost by the round is deposited into tissue near the wound
    // channel: forward push plus a radial temporary-cavity impulse.
    const double exit_speed = std::sqrt(2.0 * energy / shot.mass);
    const double momentum_lost = shot.mass * (shot.speed - exit_speed);
    if (!deposits.empty() && momentum_lost > 0.0) {
        const double total_e = std::accumulate(deposits.begin(), deposits.end(), 0.0,
            [](double acc, const auto& d) { return acc + d.second; });
        const double radius = 3.0 * h_;
        for (const auto& [point, e] : deposits) {
            const double share = total_e > 0.0 ? momentum_lost * e / total_e : 0.0;
            std::vector<std::pair<std::uint32_t, double>> near;
            double wsum = 0.0;
            for (std::uint32_t v = 0; v < n; ++v) {
                if (!alive_[v]) continue;
                const double d2 = length_squared(centers[v] - point);
                if (d2 > radius * radius) continue;
                const double w = 1.0 / (1.0 + std::sqrt(d2) / h_);
                near.push_back({v, w});
                wsum += w;
            }
            if (wsum <= 0.0) continue;
            for (const auto& [v, w] : near) {
                const Vec3 rel = centers[v] - point;
                const Vec3 radial = normalized(rel - dir * dot(rel, dir));
                const Vec3 push = normalized(dir * 0.6 + radial * 0.4);
                add_voxel_velocity(v, push * (share * w / wsum * voxel_inv_mass_[v]));
            }
        }
    }

    result.energy_out = energy;
    if (result.entered && !result.lodged) {
        result.exited = true;
        result.exit_point = last_point + dir * (0.5 * h_);
        result.exit_velocity = dir * exit_speed;
    } else if (!result.entered) {
        result.exit_velocity = dir * shot.speed;
        result.energy_out = result.energy_in;
    }
    return result;
}

// ---------------------------------------------------------------------------
// Stats
// ---------------------------------------------------------------------------

AnatomyStats AnatomyBody::stats() const {
    AnatomyStats s;
    s.voxels = tissue_.size();
    s.particles = px_.size();
    s.bonds = bond_a_.size();
    for (std::size_t v = 0; v < tissue_.size(); ++v) {
        if (!alive_[v]) continue;
        ++s.live_voxels;
        ++s.voxels_by_tissue[static_cast<std::size_t>(tissue_[v])];
    }
    for (const auto a : bond_active_) s.live_bonds += a;
    return s;
}

std::uint64_t AnatomyBody::state_hash() const {
    std::uint64_t h = 1469598103934665603ull;
    auto q = [](float x) { return static_cast<std::uint64_t>(static_cast<std::int64_t>(std::llround(x * 1e5f))); };
    for (std::size_t p = 0; p < px_.size(); ++p) {
        if (!alive_[p / 8]) continue;
        h = mix_hash(h, q(px_[p]));
        h = mix_hash(h, q(py_[p]));
        h = mix_hash(h, q(pz_[p]));
    }
    for (std::size_t b = 0; b < bond_active_.size(); ++b)
        h = mix_hash(h, bond_active_[b] | (static_cast<std::uint64_t>(std::llround(bond_hp_[b] * 1000.0f)) << 1));
    for (const auto a : alive_) h = mix_hash(h, a);
    return h;
}

// ---------------------------------------------------------------------------
// Rig math helpers
// ---------------------------------------------------------------------------

JointTransform rotation_about(const Vec3& pivot, const Vec3& axis_in, double angle) {
    const Vec3 a = normalized(axis_in);
    const double c = std::cos(angle), s = std::sin(angle), t = 1.0 - c;
    JointTransform out;
    // Column-major Rodrigues rotation.
    out.m = {
        t * a.x * a.x + c,       t * a.x * a.y + s * a.z, t * a.x * a.z - s * a.y,
        t * a.x * a.y - s * a.z, t * a.y * a.y + c,       t * a.y * a.z + s * a.x,
        t * a.x * a.z + s * a.y, t * a.y * a.z - s * a.x, t * a.z * a.z + c
    };
    const Vec3 rp = out.apply(pivot);  // with t = 0
    out.t = pivot - rp;
    return out;
}

JointTransform compose(const JointTransform& o, const JointTransform& i) {
    JointTransform out;
    for (int col = 0; col < 3; ++col)
        for (int row = 0; row < 3; ++row) {
            double v = 0.0;
            for (int k = 0; k < 3; ++k) v += o.m[k * 3 + row] * i.m[col * 3 + k];
            out.m[col * 3 + row] = v;
        }
    JointTransform rot_only = o;
    rot_only.t = {};
    out.t = rot_only.apply(i.t) + o.t;
    return out;
}

// ---------------------------------------------------------------------------
// Procedural humanoid anatomy
// ---------------------------------------------------------------------------

namespace {

enum HumanoidJoint : std::uint16_t {
    J_Pelvis, J_Spine, J_Chest, J_Neck, J_Head,
    J_LShoulder, J_LElbow, J_LWrist,
    J_RShoulder, J_RElbow, J_RWrist,
    J_LHip, J_LKnee, J_LAnkle,
    J_RHip, J_RKnee, J_RAnkle,
    J_Count
};

const std::array<const char*, J_Count> kJointNames{
    "pelvis", "spine", "chest", "neck", "head",
    "shoulder_l", "elbow_l", "wrist_l",
    "shoulder_r", "elbow_r", "wrist_r",
    "hip_l", "knee_l", "ankle_l",
    "hip_r", "knee_r", "ankle_r"};

const std::array<int, J_Count> kJointParents{
    -1, J_Pelvis, J_Spine, J_Chest, J_Neck,
    J_Chest, J_LShoulder, J_LElbow,
    J_Chest, J_RShoulder, J_RElbow,
    J_Pelvis, J_LHip, J_LKnee,
    J_Pelvis, J_RHip, J_RKnee};

// Character's left is -x; the figure faces +z.
const std::array<Vec3, J_Count> kJointRest{{
    {0.00, 0.96, 0.00},   // pelvis
    {0.00, 1.10, -0.02},  // spine
    {0.00, 1.30, -0.02},  // chest
    {0.00, 1.50, -0.01},  // neck
    {0.00, 1.60, 0.00},   // head
    {-0.20, 1.44, -0.01}, // shoulder_l
    {-0.29, 1.16, -0.01}, // elbow_l
    {-0.35, 0.92, 0.02},  // wrist_l
    {0.20, 1.44, -0.01},
    {0.29, 1.16, -0.01},
    {0.35, 0.92, 0.02},
    {-0.10, 0.92, 0.00},  // hip_l
    {-0.11, 0.50, 0.01},  // knee_l
    {-0.11, 0.09, -0.01}, // ankle_l
    {0.10, 0.92, 0.00},
    {0.11, 0.50, 0.01},
    {0.11, 0.09, -0.01},
}};

struct Segment {
    std::uint16_t joint;
    Vec3 a, b;
    double ra, rb;     // flesh radius at ends
    double bone;       // bone radius (0 = none)
};

double seg_param(const Vec3& p, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const double d = length_squared(ab);
    return d > 1e-12 ? std::clamp(dot(p - a, ab) / d, 0.0, 1.0) : 0.0;
}

double seg_dist(const Vec3& p, const Vec3& a, const Vec3& b) {
    const double t = seg_param(p, a, b);
    return length(p - (a + (b - a) * t));
}

std::vector<Segment> humanoid_segments() {
    const auto& J = kJointRest;
    std::vector<Segment> s;
    // Limbs: flesh radius tapers; bone runs along the axis.
    auto limb = [&](std::uint16_t j, Vec3 a, Vec3 b, double ra, double rb, double bone) {
        s.push_back({j, a, b, ra, rb, bone});
    };
    for (int side = 0; side < 2; ++side) {
        const double sx = side == 0 ? -1.0 : 1.0;
        const std::uint16_t sh = side == 0 ? J_LShoulder : J_RShoulder;
        const std::uint16_t el = side == 0 ? J_LElbow : J_RElbow;
        const std::uint16_t wr = side == 0 ? J_LWrist : J_RWrist;
        const std::uint16_t hp = side == 0 ? J_LHip : J_RHip;
        const std::uint16_t kn = side == 0 ? J_LKnee : J_RKnee;
        const std::uint16_t an = side == 0 ? J_LAnkle : J_RAnkle;
        limb(sh, J[sh], J[el], 0.050, 0.041, 0.014);
        limb(el, J[el], J[wr], 0.040, 0.030, 0.012);
        limb(wr, J[wr], J[wr] + Vec3{sx * 0.025, -0.16, 0.02}, 0.030, 0.022, 0.009);
        limb(hp, J[hp], J[kn], 0.080, 0.056, 0.017);
        limb(kn, J[kn], J[an], 0.056, 0.038, 0.015);
        limb(an, J[an] + Vec3{0, -0.03, -0.03}, J[an] + Vec3{0, -0.05, 0.17}, 0.040, 0.030, 0.011);
    }
    // Neck and head skin envelope handled separately; spine column carries bone.
    limb(J_Neck, J[J_Neck] + Vec3{0, -0.05, 0.0}, J[J_Head], 0.055, 0.052, 0.016);
    return s;
}

struct Ellipsoid {
    Vec3 c;
    Vec3 r;
    [[nodiscard]] double q(const Vec3& p) const {
        const Vec3 d = p - c;
        return std::sqrt((d.x / r.x) * (d.x / r.x) + (d.y / r.y) * (d.y / r.y) + (d.z / r.z) * (d.z / r.z));
    }
};

// Torso half-extents by height (x half-width, z half-depth).
bool torso_extent(double y, double& wx, double& wz) {
    static const std::array<std::array<double, 3>, 8> table{{
        {0.84, 0.160, 0.115},
        {0.94, 0.170, 0.115},
        {1.04, 0.145, 0.100},
        {1.14, 0.140, 0.100},
        {1.26, 0.160, 0.110},
        {1.38, 0.175, 0.110},
        {1.45, 0.170, 0.095},
        {1.50, 0.080, 0.070},
    }};
    if (y < table.front()[0] || y > table.back()[0]) return false;
    for (std::size_t i = 0; i + 1 < table.size(); ++i) {
        if (y <= table[i + 1][0]) {
            const double t = (y - table[i][0]) / (table[i + 1][0] - table[i][0]);
            wx = table[i][1] + (table[i + 1][1] - table[i][1]) * t;
            wz = table[i][2] + (table[i + 1][2] - table[i][2]) * t;
            return true;
        }
    }
    return false;
}

const Ellipsoid kHead{{0.0, 1.665, 0.01}, {0.085, 0.112, 0.100}};
const Ellipsoid kHeart{{0.025, 1.255, 0.035}, {0.055, 0.060, 0.045}};
const Ellipsoid kLungL{{-0.075, 1.295, -0.005}, {0.065, 0.115, 0.070}};
const Ellipsoid kLungR{{0.080, 1.295, -0.005}, {0.060, 0.115, 0.070}};
const Ellipsoid kLiver{{0.050, 1.085, 0.015}, {0.100, 0.050, 0.070}};
const Ellipsoid kGut{{0.0, 0.975, 0.030}, {0.115, 0.080, 0.070}};
const Ellipsoid kHips{{0.0, 0.90, -0.01}, {0.175, 0.105, 0.120}};
const Ellipsoid kPelvisBowl{{0.0, 0.95, -0.01}, {0.125, 0.060, 0.085}};

bool humanoid_occupied(const Vec3& p, const std::vector<Segment>& segs) {
    double wx = 0, wz = 0;
    if (torso_extent(p.y, wx, wz)) {
        const double qx = p.x / wx, qz = (p.z + 0.005) / wz;
        if (qx * qx + qz * qz <= 1.0) return true;
    }
    if (kHips.q(p) <= 1.0) return true;
    if (kHead.q(p) <= 1.0) return true;
    // Shoulder yoke.
    if (seg_dist(p, {-0.20, 1.43, -0.01}, {0.20, 1.43, -0.01}) <= 0.062) return true;
    for (const auto& s : segs) {
        const double t = seg_param(p, s.a, s.b);
        const double r = s.ra + (s.rb - s.ra) * t;
        if (length(p - (s.a + (s.b - s.a) * t)) <= r) return true;
    }
    return false;
}

} // namespace

std::vector<Vec3> humanoid_anatomy_joint_rest_positions() {
    return {kJointRest.begin(), kJointRest.end()};
}

std::vector<int> humanoid_anatomy_joint_parents() {
    return {kJointParents.begin(), kJointParents.end()};
}

AnatomyDesc build_humanoid_anatomy(const HumanoidAnatomySpec& spec) {
    if (!(spec.voxel_size > 0.004) || spec.voxel_size > 0.1)
        throw std::invalid_argument("humanoid anatomy voxel size out of range");

    const double h = spec.voxel_size;
    const Vec3 lo{-0.52, 0.0, -0.20};
    const Vec3 hi{0.52, 1.80, 0.26};
    const int nx = static_cast<int>(std::ceil((hi.x - lo.x) / h));
    const int ny = static_cast<int>(std::ceil((hi.y - lo.y) / h));
    const int nz = static_cast<int>(std::ceil((hi.z - lo.z) / h));
    auto idx = [&](int x, int y, int z) {
        return static_cast<std::size_t>(x) + static_cast<std::size_t>(nx) * (static_cast<std::size_t>(y) + static_cast<std::size_t>(ny) * z);
    };
    auto center = [&](int x, int y, int z) {
        return lo + Vec3{(x + 0.5) * h, (y + 0.5) * h, (z + 0.5) * h};
    };

    const auto segs = humanoid_segments();
    std::vector<std::uint8_t> occ(static_cast<std::size_t>(nx) * ny * nz, 0u);
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x)
                occ[idx(x, y, z)] = humanoid_occupied(center(x, y, z), segs) ? 1u : 0u;

    // Chamfer distance (metres) from each occupied cell to the exterior.
    const double inf = 1e9;
    std::vector<double> depth(occ.size(), inf);
    std::deque<std::array<int, 3>> queue;
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                if (!occ[idx(x, y, z)]) continue;
                bool boundary = false;
                for (int k = 0; k < 6 && !boundary; ++k) {
                    const int dx = k == 0 ? -1 : k == 1 ? 1 : 0;
                    const int dy = k == 2 ? -1 : k == 3 ? 1 : 0;
                    const int dz = k == 4 ? -1 : k == 5 ? 1 : 0;
                    const int xx = x + dx, yy = y + dy, zz = z + dz;
                    if (xx < 0 || yy < 0 || zz < 0 || xx >= nx || yy >= ny || zz >= nz || !occ[idx(xx, yy, zz)])
                        boundary = true;
                }
                if (boundary) {
                    depth[idx(x, y, z)] = 0.5 * h;
                    queue.push_back({x, y, z});
                }
            }
    while (!queue.empty()) {
        const auto [x, y, z] = queue.front();
        queue.pop_front();
        const double d0 = depth[idx(x, y, z)];
        for (int dz = -1; dz <= 1; ++dz)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    if (!dx && !dy && !dz) continue;
                    const int xx = x + dx, yy = y + dy, zz = z + dz;
                    if (xx < 0 || yy < 0 || zz < 0 || xx >= nx || yy >= ny || zz >= nz) continue;
                    const std::size_t j = idx(xx, yy, zz);
                    if (!occ[j]) continue;
                    const double step = h * std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
                    if (d0 + step < depth[j] - 1e-12) {
                        depth[j] = d0 + step;
                        queue.push_back({xx, yy, zz});
                    }
                }
    }

    const auto& J = kJointRest;
    const double skin_limit = std::max(0.5 * h + 1e-9, spec.skin_thickness);
    const double fat_limit = skin_limit + spec.fat_thickness;
    // Bone radius never drops below half a voxel so every bone stays connected.
    auto bone_r = [&](double r) { return std::max(r, 0.55 * h); };

    AnatomyDesc desc;
    desc.origin = lo;
    desc.voxel_size = h;
    desc.root_joints = {J_Pelvis};
    for (const char* name : kJointNames) desc.joint_names.emplace_back(name);

    struct Bone {
        std::uint16_t joint;
        Vec3 a, b;
        double r;
    };
    std::vector<Bone> bones;
    for (const auto& s : segs)
        if (s.bone > 0.0) bones.push_back({s.joint, s.a, s.b, bone_r(s.bone)});
    // Axial skeleton: vertebral column behind the organs, clavicles, pelvis.
    bones.push_back({J_Pelvis, {0, 0.90, -0.075}, J[J_Spine] + Vec3{0, 0, -0.065}, bone_r(0.022)});
    bones.push_back({J_Spine, J[J_Spine] + Vec3{0, 0, -0.065}, J[J_Chest] + Vec3{0, 0, -0.075}, bone_r(0.022)});
    bones.push_back({J_Chest, J[J_Chest] + Vec3{0, 0, -0.075}, J[J_Neck] + Vec3{0, 0, -0.035}, bone_r(0.020)});
    bones.push_back({J_Chest, {-0.18, 1.455, 0.02}, {-0.02, 1.445, 0.06}, bone_r(0.011)});
    bones.push_back({J_Chest, {0.18, 1.455, 0.02}, {0.02, 1.445, 0.06}, bone_r(0.011)});
    bones.push_back({J_Chest, J[J_LShoulder], {-0.15, 1.40, -0.07}, bone_r(0.012)});   // scapula
    bones.push_back({J_Chest, J[J_RShoulder], {0.15, 1.40, -0.07}, bone_r(0.012)});
    bones.push_back({J_Pelvis, J[J_LHip], {-0.05, 1.00, -0.05}, bone_r(0.022)});      // ilium
    bones.push_back({J_Pelvis, J[J_RHip], {0.05, 1.00, -0.05}, bone_r(0.022)});

    // Flesh segments used for skinning (joint owns the segment it starts).
    struct SkinSeg {
        std::uint16_t joint;
        Vec3 a, b;
    };
    std::vector<SkinSeg> skin_segs;
    for (const auto& s : segs) skin_segs.push_back({s.joint, s.a, s.b});
    skin_segs.push_back({J_Pelvis, {0, 0.84, 0}, J[J_Spine]});
    skin_segs.push_back({J_Pelvis, J[J_LHip], J[J_RHip]});
    skin_segs.push_back({J_Spine, J[J_Spine], J[J_Chest]});
    skin_segs.push_back({J_Chest, J[J_Chest], J[J_Neck]});
    skin_segs.push_back({J_Chest, J[J_LShoulder], J[J_RShoulder]});
    skin_segs.push_back({J_Head, J[J_Head], J[J_Head] + Vec3{0, 0.16, 0}});

    const std::array<Vec3, 4> tendon_sites{J[J_LElbow], J[J_RElbow], J[J_LKnee], J[J_RKnee]};

    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                if (!occ[idx(x, y, z)]) continue;
                const Vec3 p = center(x, y, z);
                const double d = depth[idx(x, y, z)];

                AnatomyVoxelDesc vd;
                vd.cell = {x, y, z};

                Tissue t = Tissue::Muscle;
                std::uint16_t bone_joint = kNoJoint;
                const double head_q = kHead.q(p);
                const bool in_head = head_q <= 1.0 && p.y > 1.56;

                if (d <= skin_limit) {
                    t = Tissue::Skin;
                } else if (in_head) {
                    // Scalp over skull over brain.
                    t = head_q < 0.74 ? Tissue::Brain : head_q < 0.92 ? Tissue::Bone : Tissue::Fat;
                    if (t == Tissue::Bone) bone_joint = J_Head;
                } else {
                    // Default soft tissue by depth.
                    // Subcutaneous fat is thicker over the trunk, buttocks and thighs.
                    double wx0 = 0, wz0 = 0;
                    const bool trunk = torso_extent(p.y, wx0, wz0) || kHips.q(p) <= 1.0 || p.y > 0.62;
                    const double fat_here = skin_limit + spec.fat_thickness * (trunk ? 2.0 : 1.0);
                    t = d <= std::max(fat_limit, fat_here) ? Tissue::Fat : Tissue::Muscle;

                    // Rib cage: banded ellipsoidal shell around the thoracic cavity.
                    bool rib = false;
                    if (spec.ribs && p.y > 1.08 && p.y < 1.44) {
                        double wx = 0, wz = 0;
                        if (torso_extent(p.y, wx, wz)) {
                            const double qx = p.x / (wx - 0.022), qz = (p.z + 0.005) / (wz - 0.020);
                            const double q = std::sqrt(qx * qx + qz * qz);
                            const double shell = 1.2 * h / std::min(wx, wz);
                            const double band = std::fmod((p.y - 1.08) / 0.036, 1.0);
                            const bool sternum = std::abs(p.x) < 0.022 && p.z > 0.0 && p.y > 1.18;
                            if (q <= 1.0 && q >= 1.0 - shell && (band < 0.55 || sternum)) rib = true;
                        }
                    }
                    if (rib) {
                        t = Tissue::Bone;
                        bone_joint = p.y > 1.22 ? J_Chest : J_Spine;
                    }

                    if (t != Tissue::Bone && spec.organs) {
                        if (kHeart.q(p) <= 1.0) t = Tissue::Heart;
                        else if (kLungL.q(p) <= 1.0 || kLungR.q(p) <= 1.0) t = Tissue::Lung;
                        else if (kLiver.q(p) <= 1.0) t = Tissue::Liver;
                        else if (kGut.q(p) <= 1.0 && kPelvisBowl.q(p) > 0.8) t = Tissue::Gut;
                    }

                    for (const auto& b : bones) {
                        if (seg_dist(p, b.a, b.b) <= b.r) {
                            t = Tissue::Bone;
                            bone_joint = b.joint;
                            break;
                        }
                    }
                    if (t == Tissue::Muscle) {
                        for (const auto& site : tendon_sites)
                            if (length(p - site) < 0.045) t = Tissue::Tendon;
                    }
                }

                if (bone_joint != kNoJoint) {
                    vd.joints = {bone_joint, kNoJoint, kNoJoint, kNoJoint};
                    vd.weights = {1.0f, 0, 0, 0};
                } else {
                    // Two nearest skinning segments, inverse-distance^4 blend.
                    std::array<std::pair<double, std::uint16_t>, 2> best{{{1e9, kNoJoint}, {1e9, kNoJoint}}};
                    for (const auto& s : skin_segs) {
                        const double dd = seg_dist(p, s.a, s.b);
                        if (dd < best[0].first) {
                            if (best[0].second != s.joint) best[1] = best[0];
                            best[0] = {dd, s.joint};
                        } else if (dd < best[1].first && s.joint != best[0].second) {
                            best[1] = {dd, s.joint};
                        }
                    }
                    const double w0 = 1.0 / std::pow(best[0].first + 0.01, 4.0);
                    const double w1 = best[1].second == kNoJoint ? 0.0 : 1.0 / std::pow(best[1].first + 0.01, 4.0);
                    vd.joints = {best[0].second, best[1].second, kNoJoint, kNoJoint};
                    vd.weights = {static_cast<float>(w0 / (w0 + w1)), static_cast<float>(w1 / (w0 + w1)), 0, 0};
                }
                vd.tissue = t;
                // Slack soft tissue over articulations: a limp limb swinging
                // 0.3 rad shears flesh 5 cm from the joint by ~1.5 cm.
                if (t != Tissue::Bone && t != Tissue::Marrow) {
                    for (int j = 1; j < J_Count; ++j) {
                        if (length(p - J[static_cast<std::size_t>(j)]) < 0.07) {
                            vd.tear_scale = 2.0f;
                            break;
                        }
                    }
                }
                desc.voxels.push_back(vd);
            }

    // Marrow: bone voxels fully enclosed by bone.
    {
        std::vector<std::int32_t> lookup(occ.size(), -1);
        for (std::size_t i = 0; i < desc.voxels.size(); ++i) {
            const auto& c = desc.voxels[i].cell;
            lookup[idx(c[0], c[1], c[2])] = static_cast<std::int32_t>(i);
        }
        std::vector<std::size_t> marrow;
        for (std::size_t i = 0; i < desc.voxels.size(); ++i) {
            if (desc.voxels[i].tissue != Tissue::Bone) continue;
            const auto& c = desc.voxels[i].cell;
            bool enclosed = true;
            for (int k = 0; k < 6 && enclosed; ++k) {
                const int dx = k == 0 ? -1 : k == 1 ? 1 : 0;
                const int dy = k == 2 ? -1 : k == 3 ? 1 : 0;
                const int dz = k == 4 ? -1 : k == 5 ? 1 : 0;
                const int xx = c[0] + dx, yy = c[1] + dy, zz = c[2] + dz;
                if (xx < 0 || yy < 0 || zz < 0 || xx >= nx || yy >= ny || zz >= nz) { enclosed = false; break; }
                const auto j = lookup[idx(xx, yy, zz)];
                if (j < 0 || desc.voxels[static_cast<std::size_t>(j)].tissue != Tissue::Bone) enclosed = false;
            }
            // Skull and ribs are thin plates; only long bones get a marrow core.
            if (enclosed && desc.voxels[i].joints[0] != J_Head && desc.voxels[i].joints[0] != J_Chest)
                marrow.push_back(i);
        }
        for (const auto i : marrow) desc.voxels[i].tissue = Tissue::Marrow;
    }
    return desc;
}

std::vector<JointTransform> humanoid_anatomy_pose(
    double phase, double amount, const Vec3& root_offset) {

    std::array<JointTransform, J_Count> local{};
    const Vec3 x_axis{1, 0, 0};
    const Vec3 y_axis{0, 1, 0};
    const Vec3 z_axis{0, 0, 1};
    const auto& J = kJointRest;
    const double s = std::sin(phase);

    local[J_Spine] = rotation_about(J[J_Spine], y_axis, 0.08 * amount * s);
    local[J_Chest] = rotation_about(J[J_Chest], y_axis, 0.10 * amount * s);
    local[J_Neck] = rotation_about(J[J_Neck], y_axis, -0.12 * amount * s);
    local[J_LShoulder] = compose(rotation_about(J[J_LShoulder], x_axis, 0.45 * amount * s),
                                 rotation_about(J[J_LShoulder], z_axis, 0.05));
    local[J_RShoulder] = compose(rotation_about(J[J_RShoulder], x_axis, -0.45 * amount * s),
                                 rotation_about(J[J_RShoulder], z_axis, -0.05));
    local[J_LElbow] = rotation_about(J[J_LElbow], x_axis, -0.25 * amount * (1.0 + s));
    local[J_RElbow] = rotation_about(J[J_RElbow], x_axis, -0.25 * amount * (1.0 - s));
    local[J_LHip] = rotation_about(J[J_LHip], x_axis, -0.40 * amount * s);
    local[J_RHip] = rotation_about(J[J_RHip], x_axis, 0.40 * amount * s);
    local[J_LKnee] = rotation_about(J[J_LKnee], x_axis, 0.55 * amount * std::max(0.0, s));
    local[J_RKnee] = rotation_about(J[J_RKnee], x_axis, 0.55 * amount * std::max(0.0, -s));

    JointTransform root;
    root.t = root_offset + Vec3{0.0, 0.015 * amount * std::abs(std::cos(phase)), 0.0};
    std::vector<JointTransform> global(J_Count);
    for (int j = 0; j < J_Count; ++j) {
        const int parent = kJointParents[j];
        global[j] = parent < 0 ? compose(root, local[j]) : compose(global[static_cast<std::size_t>(parent)], local[j]);
    }
    return global;
}

} // namespace sarx
