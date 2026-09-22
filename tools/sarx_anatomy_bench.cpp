// Frame-cost benchmark for the V0.5 anatomical voxel body.
//
// Reports simulation milliseconds per 60 Hz frame (rendering excluded) for the
// cases a game actually hits:
//   intact   - rig-driven walking body, hybrid residency (no damage)
//   wounded  - the same after a torso chop and a gunshot (local simulation)
//   dynamic  - the whole body simulated (worst case, e.g. a grabbed body)
//   corpse   - a dead body dropped lying on the floor, until it sleeps
//
// Usage: sarx_anatomy_bench [--voxel 0.02] [--frames 240] [--threads 0]

#include "sarx/anatomy.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

using namespace sarx;

namespace {

struct Result {
    double avg{0}, p95{0}, max{0};
    std::size_t dynamic_max{0};
};

template <class F>
Result time_frames(AnatomyBody& body, int frames, const AnatomyStepConfig& cfg, F&& before) {
    std::vector<double> ms;
    Result r;
    for (int f = 0; f < frames; ++f) {
        before(f);
        const auto t0 = std::chrono::steady_clock::now();
        body.step(1.0 / 60.0, cfg);
        ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        r.dynamic_max = std::max(r.dynamic_max, body.dynamic_voxel_count());
    }
    std::vector<double> sorted = ms;
    std::sort(sorted.begin(), sorted.end());
    for (const double m : ms) r.avg += m;
    r.avg /= static_cast<double>(ms.size());
    r.p95 = sorted[static_cast<std::size_t>(0.95 * static_cast<double>(sorted.size() - 1))];
    r.max = sorted.back();
    return r;
}

void print(const char* name, const Result& r, std::size_t voxels) {
    std::printf("  %-8s avg %7.3f ms  p95 %7.3f ms  max %7.3f ms  dynamic voxels (max) %zu / %zu\n", name, r.avg,
                r.p95, r.max, r.dynamic_max, voxels);
}

} // namespace

int main(int argc, char** argv) {
    double voxel = 0.02;
    int frames = 240;
    unsigned threads = 0;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string k = argv[i];
        if (k == "--voxel") voxel = std::stod(argv[i + 1]);
        else if (k == "--frames") frames = std::stoi(argv[i + 1]);
        else if (k == "--threads") threads = static_cast<unsigned>(std::stoi(argv[i + 1]));
    }
    HumanoidAnatomySpec spec;
    spec.voxel_size = voxel;
    const auto t0 = std::chrono::steady_clock::now();
    const AnatomyDesc desc = build_humanoid_anatomy(spec);
    const double build_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    AnatomyStepConfig cfg;
    cfg.threads = threads;
    {
        AnatomyBody probe(desc);
        const auto st = probe.stats();
        std::printf("voxel %.3f m: %zu voxels, %zu particles, %zu bonds, %.1f kg, build %.1f ms\n", voxel, st.voxels,
                    st.particles, st.bonds, probe.total_mass(), build_ms);
    }
    auto walk = [&](AnatomyBody& b) {
        return [&b](int f) { b.set_pose(humanoid_anatomy_pose(f / 60.0 * 5.0, 1.0)); };
    };
    {
        AnatomyBody body(desc);
        body.set_pose(humanoid_anatomy_pose(0.0, 1.0));
        body.snap_to_pose();
        print("intact", time_frames(body, frames, cfg, walk(body)), body.voxel_count());
    }
    {
        AnatomyBody body(desc);
        body.set_pose(humanoid_anatomy_pose(0.0, 1.0));
        body.snap_to_pose();
        BladeStroke s;
        s.energy = 150.0;
        for (int i = 0; i <= 8; ++i) {
            const double z = 0.4 - 0.8 * i / 8.0;
            s.poses.push_back({{0.6, 1.045, z}, {-0.4, 1.045, z}});
        }
        (void)body.apply_blade(s);
        BulletShot b;
        b.origin = {0.05, 1.3, 3.0};
        (void)body.apply_bullet(b);
        print("wounded", time_frames(body, frames, cfg, walk(body)), body.voxel_count());
    }
    {
        AnatomyBody body(desc);
        body.set_sim_mode(AnatomySimMode::Dynamic);
        body.set_pose(humanoid_anatomy_pose(0.0, 1.0));
        body.snap_to_pose();
        print("dynamic", time_frames(body, frames, cfg, walk(body)), body.voxel_count());
    }
    {
        AnatomyBody body(desc);
        const auto r = rotation_about({0, 0.96, 0}, {1, 0, 0}, -1.5708);
        JointTransform lift;
        lift.t = {0, -0.96 + 0.14 + 0.3, 0.96};
        body.set_pose(std::vector<JointTransform>(body.joint_count(), compose(lift, r)));
        body.snap_to_pose();
        body.release_rig();
        const auto res = time_frames(body, frames, cfg, [](int) {});
        print("corpse", res, body.voxel_count());
        std::printf("  corpse after %d frames: %zu voxels still simulated (rest asleep)\n", frames, body.dynamic_voxel_count());
    }
    return 0;
}
