// Layered anatomical voxel body demonstrations.
//
//   hack   - repeated heavy chops through the lower torso until it separates
//   limb   - two sword chops through a walking character's upper arm
//   shoot  - pistol and rifle rounds: through-and-through, lodged, shattered
//            bone, and a rifle burst that removes an arm
//   rip    - a dead body is grabbed at chest and pelvis and torn in half
//
// Writes PPM frames plus summary.json per scenario. Frame rendering is not
// included in the reported simulation timings.

#include "sarx/anatomy.hpp"
#include "sarx/anatomy_render.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace sarx;

namespace {

struct Args {
    std::string scenario{"all"};
    std::string output{"sarx_anatomy_frames"};
    int frames{-1};
    int every{1};
    double voxel{0.02};
    unsigned width{960};
    unsigned height{540};
    bool render{true};
};

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("missing value for " + k);
            return argv[++i];
        };
        if (k == "--scenario") a.scenario = next();
        else if (k == "--output") a.output = next();
        else if (k == "--frames") a.frames = std::stoi(next());
        else if (k == "--every") a.every = std::max(1, std::stoi(next()));
        else if (k == "--voxel") a.voxel = std::stod(next());
        else if (k == "--width") a.width = static_cast<unsigned>(std::stoi(next()));
        else if (k == "--height") a.height = static_cast<unsigned>(std::stoi(next()));
        else if (k == "--no-render") a.render = false;
        else throw std::runtime_error("unknown argument " + k);
    }
    return a;
}

struct Particle {
    Vec3 p, v;
    Rgb8 c;
    double life;
};

struct Timing {
    double total_ms{0};
    double max_ms{0};
    int frames{0};
    std::size_t max_dynamic{0};
    int max_substeps{0};
    void add(double ms, std::size_t dynamic, int substeps) {
        total_ms += ms;
        max_ms = std::max(max_ms, ms);
        ++frames;
        max_dynamic = std::max(max_dynamic, dynamic);
        max_substeps = std::max(max_substeps, substeps);
    }
};

std::string tissues_json(const std::array<std::size_t, kTissueCount>& counts) {
    std::ostringstream o;
    o << "{";
    bool first = true;
    for (std::size_t t = 0; t < kTissueCount; ++t) {
        if (!counts[t]) continue;
        o << (first ? "" : ", ") << "\"" << tissue_name(static_cast<Tissue>(t)) << "\": " << counts[t];
        first = false;
    }
    o << "}";
    return o.str();
}

std::string vec_json(const Vec3& v) {
    std::ostringstream o;
    o.precision(4);
    o << "[" << v.x << ", " << v.y << ", " << v.z << "]";
    return o.str();
}

class Scenario {
public:
    Scenario(const Args& args, const std::string& name)
        : args_(args), name_(name), dir_(std::filesystem::path(args.output) / name) {
        std::filesystem::create_directories(dir_);
        HumanoidAnatomySpec spec;
        spec.voxel_size = args.voxel;
        body_ = AnatomyBody(build_humanoid_anatomy(spec));
        cam_.width = args.width;
        cam_.height = args.height;
    }

    AnatomyBody& body() { return body_; }
    AnatomyCamera& camera() { return cam_; }
    std::vector<AnatomyOverlayLine>& persistent_lines() { return persistent_; }
    std::vector<AnatomyOverlayLine>& transient_lines() { return transient_; }

    void emit_debris(const std::vector<AnatomyDebris>& debris) {
        for (const auto& d : debris)
            particles_.push_back({d.position, d.velocity, body_.tissues()[static_cast<std::size_t>(d.tissue)].color, 1.6});
    }

    void spray(const Vec3& at, const Vec3& dir, int count, double speed, std::uint32_t seed) {
        for (int i = 0; i < count; ++i) {
            const double a = std::sin(seed * 12.9898 + i * 78.233) * 43758.5453;
            const double b = std::sin(seed * 4.1414 + i * 11.137) * 24634.6345;
            const double c = std::sin(seed * 7.7 + i * 3.3) * 9123.77;
            const Vec3 j{a - std::floor(a) - 0.5, b - std::floor(b) - 0.5, c - std::floor(c) - 0.5};
            particles_.push_back({at, normalized(dir + j * 1.6) * (speed * (0.5 + (a - std::floor(a)))),
                                  Rgb8{150, 14, 20}, 1.2});
        }
    }

    void step(const AnatomyStepConfig& cfg) {
        const auto t0 = std::chrono::steady_clock::now();
        body_.step(1.0 / 60.0, cfg);
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        timing_.add(ms, body_.dynamic_voxel_count(), body_.last_substeps());
        for (auto& p : particles_) {
            p.v.y -= 9.81 / 60.0;
            p.p += p.v * (1.0 / 60.0);
            if (p.p.y < 0.004) {
                p.p.y = 0.004;
                p.v = {p.v.x * 0.3, -p.v.y * 0.15, p.v.z * 0.3};
            }
            p.life -= 1.0 / 60.0;
        }
        particles_.erase(std::remove_if(particles_.begin(), particles_.end(), [](const Particle& p) {
            return p.life <= 0.0;
        }), particles_.end());
    }

    void frame(int index, const std::string& caption = {}) {
        if (!args_.render || index % args_.every != 0) {
            transient_.clear();
            return;
        }
        AnatomyFrame f;
        f.bodies = {&body_};
        f.lines = persistent_;
        f.lines.insert(f.lines.end(), transient_.begin(), transient_.end());
        for (const auto& p : particles_) f.points.push_back({p.p, p.c, 0.012});
        f.caption = caption;
        char name[64];
        std::snprintf(name, sizeof(name), "frame_%04d.ppm", written_++);
        render_anatomy(f, cam_).write_ppm((dir_ / name).string());
        transient_.clear();
    }

    void event(const std::string& json) { events_.push_back(json); }

    void finish() {
        std::ofstream out(dir_ / "summary.json");
        const auto st = body_.stats();
        const auto& comps = body_.components();
        std::size_t free_islands = 0;
        for (const auto& c : comps) free_islands += c.rig_authoritative ? 0 : 1;
        out << "{\n  \"scenario\": \"" << name_ << "\",\n";
        out << "  \"voxel_size_m\": " << body_.voxel_size() << ",\n";
        out << "  \"voxels\": " << st.voxels << ",\n  \"particles\": " << st.particles << ",\n";
        out << "  \"bonds\": " << st.bonds << ",\n  \"live_voxels\": " << st.live_voxels << ",\n";
        out << "  \"live_bonds\": " << st.live_bonds << ",\n";
        out << "  \"components\": " << comps.size() << ",\n  \"free_islands\": " << free_islands << ",\n";
        out << "  \"sim_ms_per_frame_avg\": " << (timing_.frames ? timing_.total_ms / timing_.frames : 0.0) << ",\n";
        out << "  \"sim_ms_per_frame_max\": " << timing_.max_ms << ",\n";
        out << "  \"max_dynamic_voxels\": " << timing_.max_dynamic << ",\n";
        out << "  \"max_substeps\": " << timing_.max_substeps << ",\n";
        out << "  \"frames\": " << timing_.frames << ",\n";
        out << "  \"events\": [\n";
        for (std::size_t i = 0; i < events_.size(); ++i)
            out << "    " << events_[i] << (i + 1 < events_.size() ? ",\n" : "\n");
        out << "  ]\n}\n";
        std::cout << name_ << ": " << timing_.frames << " frames, avg "
                  << (timing_.frames ? timing_.total_ms / timing_.frames : 0.0) << " ms, max " << timing_.max_ms
                  << " ms, components " << comps.size() << ", free islands " << free_islands << "\n";
    }

private:
    Args args_;
    std::string name_;
    std::filesystem::path dir_;
    AnatomyBody body_;
    AnatomyCamera cam_;
    std::vector<Particle> particles_;
    std::vector<AnatomyOverlayLine> persistent_;
    std::vector<AnatomyOverlayLine> transient_;
    std::vector<std::string> events_;
    Timing timing_;
    int written_{0};
};

std::string blade_json(int frame, const BladeResult& r) {
    std::ostringstream o;
    o << "{\"frame\": " << frame << ", \"type\": \"blade\", \"bonds_crossed\": " << r.bonds_crossed
      << ", \"bonds_parted\": " << r.bonds_parted << ", \"bonds_damaged\": " << r.bonds_damaged
      << ", \"energy_spent_J\": " << r.energy_spent << ", \"lodged\": " << (r.lodged ? "true" : "false")
      << ", \"stopped_in\": \"" << (r.lodged ? tissue_name(r.stopped_in) : "-") << "\""
      << ", \"parted_by_tissue\": " << tissues_json(r.parted_by_tissue) << "}";
    return o.str();
}

std::string bullet_json(int frame, const std::string& weapon, const BulletResult& r) {
    std::ostringstream o;
    o << "{\"frame\": " << frame << ", \"type\": \"bullet\", \"weapon\": \"" << weapon << "\""
      << ", \"entered\": " << (r.entered ? "true" : "false") << ", \"exited\": " << (r.exited ? "true" : "false")
      << ", \"lodged\": " << (r.lodged ? "true" : "false") << ", \"energy_in_J\": " << r.energy_in
      << ", \"energy_out_J\": " << r.energy_out << ", \"voxels_destroyed\": " << r.voxels_destroyed
      << ", \"exit_speed_mps\": " << length(r.exit_velocity)
      << ", \"destroyed_by_tissue\": " << tissues_json(r.destroyed_by_tissue) << "}";
    return o.str();
}

// Horizontal chop: blade lies along -x, sweeping front (+z) to back (-z).
BladeStroke chop(double y, double z0, double z1, double energy, double sharpness) {
    BladeStroke s;
    s.energy = energy;
    s.sharpness = sharpness;
    const int n = 8;
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const double z = z0 + (z1 - z0) * t;
        s.poses.push_back({{0.62, y + 0.01 * t, z}, {-0.40, y - 0.01 * t, z}});
    }
    return s;
}

void scenario_hack(const Args& args) {
    Scenario sc(args, "hack");
    auto& body = sc.body();
    sc.camera().position = {1.55, 1.35, 2.25};
    sc.camera().target = {0.0, 0.85, 0.0};
    const int frames = args.frames > 0 ? args.frames : 300;
    AnatomyStepConfig cfg;
    body.set_pose(humanoid_anatomy_pose(0.0, 0.12));
    body.snap_to_pose();
    const std::vector<int> chops{30, 75, 120, 165, 210};
    double lodged_z = 0.4;
    int show_blade = -1;
    BladeResult last{};
    for (int f = 0; f < frames; ++f) {
        body.set_pose(humanoid_anatomy_pose(f / 60.0 * 2.0, 0.12));
        if (std::find(chops.begin(), chops.end(), f) != chops.end()) {
            // Each chop re-enters the same wound and continues deeper.
            auto s = chop(1.045, 0.40, -0.40, 180.0, 0.9);
            last = body.apply_blade(s);
            sc.event(blade_json(f, last));
            lodged_z = last.lodged ? last.stop_point.z : -0.40;
            show_blade = 14;
            sc.spray(Vec3{0.0, 1.045, lodged_z}, {0, 0.3, 1}, 30, 2.5, static_cast<std::uint32_t>(f));
        }
        if (show_blade-- > 0)
            sc.transient_lines().push_back({{0.62, 1.045, lodged_z}, {-0.40, 1.045, lodged_z}, {205, 210, 220}, 3});
        sc.step(cfg);
        sc.frame(f);
    }
    sc.finish();
}

void scenario_limb(const Args& args) {
    Scenario sc(args, "limb");
    auto& body = sc.body();
    sc.camera().position = {1.6, 1.4, 1.5};
    sc.camera().target = {0.15, 1.0, 0.0};
    const int frames = args.frames > 0 ? args.frames : 240;
    AnatomyStepConfig cfg;
    const auto names = body.joint_names();
    const auto rest = humanoid_anatomy_joint_rest_positions();
    const std::size_t shoulder = static_cast<std::size_t>(std::find(names.begin(), names.end(), "shoulder_r") - names.begin());
    body.set_pose(humanoid_anatomy_pose(0.0, 0.6));
    body.snap_to_pose();
    // Point on the right upper arm, 60% of the way to the elbow.
    const Vec3 arm_rest = rest[shoulder] + (Vec3{0.29, 1.16, -0.01} - rest[shoulder]) * 0.6;
    for (int f = 0; f < frames; ++f) {
        const double phase = f / 60.0 * 3.0;
        const auto pose = humanoid_anatomy_pose(phase, 0.6);
        body.set_pose(pose);
        if (f == 50 || f == 100) {
            const Vec3 target = pose[shoulder].apply(arm_rest);
            BladeStroke s;
            s.energy = 70.0;
            s.sharpness = 1.0;
            for (int i = 0; i <= 8; ++i) {
                const double t = i / 8.0;
                const double x = target.x + 0.30 - 0.36 * t;   // outside -> in, stop before ribs
                s.poses.push_back({{x, target.y + 0.02, target.z + 0.45}, {x, target.y - 0.02, target.z - 0.35}});
            }
            const auto r = body.apply_blade(s);
            sc.event(blade_json(f, r));
            sc.spray(target, {1, 0.2, 0}, 24, 2.0, static_cast<std::uint32_t>(f));
            for (int k = 0; k < 10; ++k)
                sc.transient_lines().push_back({r.stop_point + Vec3{0, 0, 0.45}, r.stop_point + Vec3{0, 0, -0.35}, {205, 210, 220}, 3});
        }
        sc.step(cfg);
        sc.frame(f);
    }
    sc.finish();
}

void scenario_shoot(const Args& args) {
    Scenario sc(args, "shoot");
    auto& body = sc.body();
    sc.camera().position = {0.9, 1.45, 1.55};
    sc.camera().target = {-0.05, 1.12, 0.0};
    const int frames = args.frames > 0 ? args.frames : 260;
    AnatomyStepConfig cfg;
    body.set_pose(humanoid_anatomy_pose(0.0, 0.05));
    body.snap_to_pose();

    struct Shot {
        int frame;
        const char* weapon;
        Vec3 from;
        Vec3 to;
        double speed, mass;
    };
    const double pistol_v = 370.0, pistol_m = 0.008;     // 9x19 mm
    const double rifle_v = 940.0, rifle_m = 0.004;       // 5.56x45 mm
    std::vector<Shot> shots{
        {30, "9mm", {0.06, 1.30, 3.0}, {0.06, 1.30, -3.0}, pistol_v, pistol_m},     // chest through lung
        {55, "9mm", {-0.05, 1.02, 3.0}, {-0.05, 1.02, -3.0}, pistol_v, pistol_m},   // abdomen
        {80, "9mm", {0.0, 1.20, 3.0}, {0.0, 1.20, -3.0}, pistol_v, pistol_m},       // into the spine
        {105, "5.56", {-0.11, 0.72, 3.0}, {-0.11, 0.72, -3.0}, rifle_v, rifle_m},   // femur
    };
    // Rifle burst stitched across the left upper arm until it is shot away.
    for (int row = 0; row < 2; ++row)
        for (int i = 0; i < 6; ++i) {
            const double x = -0.205 - 0.02 * i;
            const double y = 1.25 + 0.02 * row;
            shots.push_back({140 + (row * 6 + i) * 3, "5.56", {x, y, 3.0}, {x, y, -3.0}, rifle_v, rifle_m});
        }
    for (int f = 0; f < frames; ++f) {
        body.set_pose(humanoid_anatomy_pose(f / 60.0, 0.05));
        for (const auto& s : shots) {
            if (s.frame != f) continue;
            BulletShot b;
            b.origin = s.from;
            b.direction = s.to - s.from;
            b.speed = s.speed;
            b.mass = s.mass;
            b.radius = std::string(s.weapon) == "9mm" ? 0.0045 : 0.0028;
            const auto r = body.apply_bullet(b);
            sc.event(bullet_json(f, s.weapon, r));
            sc.emit_debris(r.debris);
            const Vec3 dir = normalized(s.to - s.from);
            const Vec3 end = r.exited ? r.exit_point + dir * 1.2 : (r.entered ? r.exit_point : s.from + dir * 6.0);
            for (int k = 0; k < 4; ++k)
                sc.transient_lines().push_back({s.from + dir * 1.5, end, {255, 214, 90}, 1});
            if (r.exited) sc.spray(r.exit_point, dir, 20, 3.0, static_cast<std::uint32_t>(f));
        }
        sc.step(cfg);
        sc.frame(f);
    }
    sc.finish();
}

void scenario_rip(const Args& args) {
    // Tear-in-half experiment. The body keeps muscle tone and is pulled apart
    // through its own rig: pelvis and legs follow one hand, the spine and
    // everything above it follow the other. Once the torso parts, the upper
    // half loses root authority (it goes limp) and hangs from the chest grab.
    Scenario sc(args, "rip");
    auto& body = sc.body();
    sc.camera().position = {3.0, 1.7, 2.3};
    sc.camera().target = {0.0, 1.35, 0.0};
    const int frames = args.frames > 0 ? args.frames : 300;
    AnatomyStepConfig cfg;
    const Vec3 hang{0.0, 0.40, 0.0};
    body.set_sim_mode(AnatomySimMode::Dynamic);
    body.set_pose(humanoid_anatomy_pose(0.0, 0.0, hang));
    body.snap_to_pose();
    const Vec3 chest = Vec3{0.0, 1.34, -0.01} + hang;
    const Vec3 pelvis = Vec3{0.0, 0.90, -0.01} + hang;
    const auto g_chest = body.add_grab(chest, 0.13, 0.15);
    const auto parents = humanoid_anatomy_joint_parents();
    std::vector<bool> upper(parents.size(), false);
    for (std::size_t j = 1; j < parents.size(); ++j) {
        int k = static_cast<int>(j);
        while (k > 0 && k != 1) k = parents[static_cast<std::size_t>(k)];
        upper[j] = k == 1;  // descends from the spine joint
    }
    std::size_t tears = 0;
    int split_frame = -1;
    for (int f = 0; f < frames; ++f) {
        const double t = f / 60.0;
        const double pull = std::clamp((t - 0.5) * 0.35, 0.0, 0.55);
        const Vec3 up{0.0, pull, -0.25 * pull};
        const Vec3 down{0.0, -0.6 * pull, 0.30 * pull};
        auto pose = humanoid_anatomy_pose(0.0, 0.0, hang + down);
        JointTransform shift;
        shift.t = up - down;
        for (std::size_t j = 0; j < pose.size(); ++j)
            if (upper[j]) pose[j] = compose(shift, pose[j]);
        body.set_pose(pose);
        body.set_grab_target(g_chest, chest + up);
        sc.step(cfg);
        tears += body.recent_tears().size();
        if (split_frame < 0) {
            std::size_t big = 0;
            for (const auto& c : body.components()) big += c.voxels.size() > 800 ? 1 : 0;
            if (big >= 2) {
                split_frame = f;
                std::ostringstream o;
                o << "{\"frame\": " << f << ", \"type\": \"split\", \"torn_bonds_so_far\": " << tears << "}";
                sc.event(o.str());
            }
        }
        for (const auto& te : body.recent_tears())
            if ((te.bond % 7) == 0) sc.spray(te.position, {0, -0.2, 0.3}, 1, 0.8, static_cast<std::uint32_t>(te.bond));
        sc.transient_lines().push_back({chest + up + Vec3{0, 0.9, 0}, chest + up, {90, 90, 96}, 2});
        sc.transient_lines().push_back({pelvis + down + Vec3{0, 0, 0.9}, pelvis + down, {90, 90, 96}, 2});
        sc.frame(f);
    }
    std::ostringstream o;
    o << "{\"type\": \"total_tears\", \"bonds\": " << tears << "}";
    sc.event(o.str());
    sc.finish();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse(argc, argv);
        const std::vector<std::string> all{"hack", "limb", "shoot", "rip"};
        for (const auto& name : all) {
            if (args.scenario != "all" && args.scenario != name) continue;
            if (name == "hack") scenario_hack(args);
            else if (name == "limb") scenario_limb(args);
            else if (name == "shoot") scenario_shoot(args);
            else if (name == "rip") scenario_rip(args);
        }
        if (args.scenario != "all" && std::find(all.begin(), all.end(), args.scenario) == all.end())
            throw std::runtime_error("unknown scenario " + args.scenario);
    } catch (const std::exception& e) {
        std::cerr << "sarx_anatomy_demo: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
