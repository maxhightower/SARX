#include "sarx/voxel_skin.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace sarx {
namespace {

double det3(const Vec3& a, const Vec3& b, const Vec3& c) { return dot(a, cross(b, c)); }

// Rows of the inverse of the matrix whose columns are a, b, c.
std::array<Vec3, 3> inverse_rows(const Vec3& a, const Vec3& b, const Vec3& c) {
    const double d = det3(a, b, c);
    return {cross(b, c) / d, cross(c, a) / d, cross(a, b) / d};
}

double segment_distance_sq(const Vec3& q, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const double denom = length_squared(ab);
    const double t = denom > 1e-12 ? std::clamp(dot(q - a, ab) / denom, 0.0, 1.0) : 0.0;
    return length_squared(q - (a + ab * t));
}

} // namespace

void EmbeddedVoxelSkin::build(
    const CharacterRuntime& runtime,
    const DetailVoxelSet& detail,
    const Vec3& world_offset) {

    const auto& particles = runtime.body().particles();
    if (particles.empty()) throw std::invalid_argument("skin needs a non-empty body");

    voxel_size_ = detail.voxel_size;
    rest_particles_.clear();
    rest_particles_.reserve(particles.size());
    for (const auto& p : particles) rest_particles_.push_back(p.position);

    neighbours_.assign(particles.size(), {});
    neighbour_links_.assign(particles.size(), {});
    const auto& links = runtime.body().structural_constraints();
    for (ConstraintId id = 0; id < links.size(); ++id) {
        neighbours_[links[id].a].push_back(links[id].b);
        neighbour_links_[links[id].a].push_back(id);
        neighbours_[links[id].b].push_back(links[id].a);
        neighbour_links_[links[id].b].push_back(id);
    }

    voxels_.clear();
    voxels_.reserve(detail.voxels.size());
    nx_ = ny_ = nz_ = 0;
    for (const auto& d : detail.voxels) {
        SkinVoxel v;
        v.local = d.local;
        v.rest = d.local + world_offset;
        v.gx = d.gx;
        v.gy = d.gy;
        v.gz = d.gz;
        v.tissue = d.tissue;

        double best = std::numeric_limits<double>::infinity();
        for (ParticleId p = 0; p < rest_particles_.size(); ++p) {
            const double dist = length_squared(rest_particles_[p] - v.rest);
            if (dist < best) {
                best = dist;
                v.anchor = p;
            }
        }
        nx_ = std::max(nx_, d.gx + 1);
        ny_ = std::max(ny_, d.gy + 1);
        nz_ = std::max(nz_, d.gz + 1);
        voxels_.push_back(v);
    }

    grid_.assign(static_cast<std::size_t>(nx_) * ny_ * nz_, -1);
    for (std::size_t i = 0; i < voxels_.size(); ++i) {
        const auto& v = voxels_[i];
        grid_[v.gx + static_cast<std::size_t>(nx_) * (v.gy + static_cast<std::size_t>(ny_) * v.gz)] =
            static_cast<std::int32_t>(i);
    }

    refresh_topology(runtime);
}

void EmbeddedVoxelSkin::rebuild_frames(const CharacterRuntime& runtime) {
    const auto& links = runtime.body().structural_constraints();
    frames_.assign(rest_particles_.size(), {});

    std::vector<std::pair<double, ParticleId>> candidates;
    for (ParticleId p = 0; p < rest_particles_.size(); ++p) {
        const IslandId island = runtime.island_of_particle(p);
        candidates.clear();
        for (std::size_t k = 0; k < neighbours_[p].size(); ++k) {
            const ParticleId q = neighbours_[p][k];
            if (!links[neighbour_links_[p][k]].active) continue;
            if (runtime.island_of_particle(q) != island) continue;
            candidates.emplace_back(length_squared(rest_particles_[q] - rest_particles_[p]), q);
        }
        std::sort(candidates.begin(), candidates.end());
        if (candidates.size() > 12) candidates.resize(12);

        double best = 0.0;
        Frame frame;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            for (std::size_t j = i + 1; j < candidates.size(); ++j) {
                for (std::size_t k = j + 1; k < candidates.size(); ++k) {
                    const Vec3 a = rest_particles_[candidates[i].second] - rest_particles_[p];
                    const Vec3 b = rest_particles_[candidates[j].second] - rest_particles_[p];
                    const Vec3 c = rest_particles_[candidates[k].second] - rest_particles_[p];
                    const double d = std::abs(det3(a, b, c));
                    if (d > best) {
                        best = d;
                        frame.others = {candidates[i].second, candidates[j].second, candidates[k].second};
                    }
                }
            }
        }
        if (best > 1e-9) {
            const Vec3 a = rest_particles_[frame.others[0]] - rest_particles_[p];
            const Vec3 b = rest_particles_[frame.others[1]] - rest_particles_[p];
            const Vec3 c = rest_particles_[frame.others[2]] - rest_particles_[p];
            frame.inverse_rows = inverse_rows(a, b, c);
            frame.valid = true;
        }
        frames_[p] = frame;
    }
}

void EmbeddedVoxelSkin::rebind(std::size_t index) {
    auto& v = voxels_[index];
    const Vec3 offset = v.rest - rest_particles_[v.anchor];
    const Frame& frame = frames_[v.anchor];
    if (frame.valid) {
        v.frame_coords = {dot(frame.inverse_rows[0], offset), dot(frame.inverse_rows[1], offset),
                          dot(frame.inverse_rows[2], offset)};
    } else {
        v.frame_coords = offset;
    }
}

void EmbeddedVoxelSkin::rebuild_surface(const CharacterRuntime& runtime) {
    surface_.clear();
    auto lookup = [&](int x, int y, int z) -> std::int32_t {
        if (x < 0 || y < 0 || z < 0 || x >= nx_ || y >= ny_ || z >= nz_) return -1;
        return grid_[x + static_cast<std::size_t>(nx_) * (y + static_cast<std::size_t>(ny_) * z)];
    };
    static constexpr int kDirs[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};

    for (std::size_t i = 0; i < voxels_.size(); ++i) {
        auto& v = voxels_[i];
        v.surface = false;
        v.exposed = false;
        if (!v.alive) continue;
        const IslandId island = runtime.island_of_particle(v.anchor);
        for (const auto& d : kDirs) {
            const std::int32_t n = lookup(v.gx + d[0], v.gy + d[1], v.gz + d[2]);
            if (n < 0) {
                v.surface = true;
                continue;
            }
            const auto& other = voxels_[static_cast<std::size_t>(n)];
            if (!other.alive || runtime.island_of_particle(other.anchor) != island) {
                v.surface = true;
                v.exposed = true;
            }
        }
        if (v.surface) surface_.push_back(static_cast<std::uint32_t>(i));
    }
}

void EmbeddedVoxelSkin::refresh_topology(const CharacterRuntime& runtime) {
    rebuild_frames(runtime);
    for (std::size_t i = 0; i < voxels_.size(); ++i) rebind(i);
    rebuild_surface(runtime);
    update(runtime);
}

Vec3 EmbeddedVoxelSkin::world_position(const CharacterRuntime& runtime, std::size_t index) const {
    const auto& particles = runtime.body().particles();
    const auto& v = voxels_[index];
    const Vec3 p0 = particles[v.anchor].position;
    const Frame& frame = frames_[v.anchor];
    if (!frame.valid) return p0 + v.frame_coords;
    const Vec3 a = particles[frame.others[0]].position - p0;
    const Vec3 b = particles[frame.others[1]].position - p0;
    const Vec3 c = particles[frame.others[2]].position - p0;
    return p0 + a * v.frame_coords.x + b * v.frame_coords.y + c * v.frame_coords.z;
}

Vec3 EmbeddedVoxelSkin::deform_rest_point(
    const CharacterRuntime& runtime,
    std::size_t index,
    const Vec3& rest_point) const {

    const auto& particles = runtime.body().particles();
    const ParticleId anchor = voxels_[index].anchor;
    const Vec3 p0 = particles[anchor].position;
    const Vec3 offset = rest_point - rest_particles_[anchor];
    const Frame& frame = frames_[anchor];
    if (!frame.valid) return p0 + offset;
    const Vec3 u{dot(frame.inverse_rows[0], offset), dot(frame.inverse_rows[1], offset),
                 dot(frame.inverse_rows[2], offset)};
    const Vec3 a = particles[frame.others[0]].position - p0;
    const Vec3 b = particles[frame.others[1]].position - p0;
    const Vec3 c = particles[frame.others[2]].position - p0;
    return p0 + a * u.x + b * u.y + c * u.z;
}

void EmbeddedVoxelSkin::update(const CharacterRuntime& runtime) {
    const auto& particles = runtime.body().particles();
    for (const std::uint32_t index : surface_) {
        auto& v = voxels_[index];
        const Vec3 p0 = particles[v.anchor].position;
        const Frame& frame = frames_[v.anchor];
        if (!frame.valid) {
            v.center = p0 + v.frame_coords;
            v.axes = {Vec3{voxel_size_, 0, 0}, Vec3{0, voxel_size_, 0}, Vec3{0, 0, voxel_size_}};
            continue;
        }
        const Vec3 a = particles[frame.others[0]].position - p0;
        const Vec3 b = particles[frame.others[1]].position - p0;
        const Vec3 c = particles[frame.others[2]].position - p0;
        v.center = p0 + a * v.frame_coords.x + b * v.frame_coords.y + c * v.frame_coords.z;
        // Deformation gradient F = [a b c] * inverse(rest); column k of F
        // maps the rest axis e_k.
        const auto& r = frame.inverse_rows;
        for (int k = 0; k < 3; ++k) {
            const double ck[3] = {k == 0 ? r[0].x : k == 1 ? r[0].y : r[0].z,
                                  k == 0 ? r[1].x : k == 1 ? r[1].y : r[1].z,
                                  k == 0 ? r[2].x : k == 1 ? r[2].y : r[2].z};
            v.axes[k] = (a * ck[0] + b * ck[1] + c * ck[2]) * voxel_size_;
        }
        // Thin lattice regions can invert (reflect) under contact. A cube is
        // symmetric, so keep the drawn basis right-handed; the centre, which
        // carries the actual shape, is unaffected.
        if (dot(v.axes[0], cross(v.axes[1], v.axes[2])) < 0.0) v.axes[2] = -v.axes[2];
    }
}

std::size_t EmbeddedVoxelSkin::carve(
    const CharacterRuntime& runtime,
    const DamageCommand& command,
    const SkinCarveConfig& config) {

    std::size_t removed = 0;
    for (std::size_t i = 0; i < voxels_.size(); ++i) {
        auto& v = voxels_[i];
        if (!v.alive) continue;
        const Vec3 x = world_position(runtime, i);
        bool hit = false;
        switch (command.kind) {
        case DamageCommandKind::Capsule: {
            const double r = command.capsule.radius * config.capsule_radius_scale;
            hit = segment_distance_sq(x, command.capsule.a, command.capsule.b) <= r * r;
            break;
        }
        case DamageCommandKind::PlaneCut: {
            const Vec3 n = normalized(command.plane_cut.normal);
            const Vec3 d = x - command.plane_cut.center;
            const double along = dot(d, n);
            const Vec3 radial = d - n * along;
            hit = std::abs(along) <= config.kerf_voxels * voxel_size_
                && length_squared(radial) <= command.plane_cut.radius * command.plane_cut.radius;
            break;
        }
        case DamageCommandKind::Sphere: {
            const double r = command.sphere.radius * config.sphere_radius_scale;
            hit = length_squared(x - command.sphere.center) <= r * r;
            break;
        }
        case DamageCommandKind::Strain:
            break;
        }
        if (hit) {
            v.alive = false;
            ++removed;
        }
    }
    return removed;
}

std::size_t EmbeddedVoxelSkin::alive_count() const {
    return static_cast<std::size_t>(std::count_if(voxels_.begin(), voxels_.end(),
                                                  [](const SkinVoxel& v) { return v.alive; }));
}

} // namespace sarx
