#include "sarx/character_runtime.hpp"
#include "sarx/motion_viability.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

using sarx::CharacterRuntime;
using sarx::DamageCommand;
using sarx::DamageCommandKind;
using sarx::DamageMode;
using sarx::IslandId;
using sarx::IslandState;
using sarx::Vec3;

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

constexpr double kDt = 1.0 / 60.0;

DamageCommand shoulder_cut(const sarx::HumanoidRuntime& h) {
    DamageCommand command;
    command.kind = DamageCommandKind::PlaneCut;
    command.plane_cut.center = Vec3{0.36, 1.34, 0.0} + h.world_offset;
    command.plane_cut.normal = {1.0, 0.0, 0.0};
    command.plane_cut.radius = 0.24;
    command.plane_cut.energy = 4.0;
    return command;
}

const IslandState* island_containing_region(
    const CharacterRuntime& runtime,
    const std::string& region) {

    const auto& particles = runtime.region_particles(region);
    return runtime.find_island(runtime.island_of_particle(particles.front()));
}

void test_initial_humanoid_is_single_rig_island() {
    auto h = sarx::build_humanoid_runtime({}, {2.0, 0.0, -3.0});
    const auto& runtime = h.runtime;

    check(runtime.islands().size() == 1, "intact humanoid should be one island");
    check(runtime.islands().front().rig_authoritative,
          "intact humanoid island should be rig-authoritative");
    check(runtime.islands().front().id != sarx::kNoIsland,
          "island ids must be non-zero");

    std::size_t total = 0;
    std::vector<int> seen(runtime.body().particles().size(), 0);
    for (const auto& name : runtime.region_names()) {
        const auto& particles = runtime.region_particles(name);
        check(!particles.empty(), "region should not be empty: " + name);
        for (auto p : particles) ++seen[p];
        total += particles.size();
    }
    check(std::all_of(seen.begin(), seen.end(), [](int c) { return c <= 1; }),
          "humanoid bone regions must be disjoint");
    check(total > runtime.body().particles().size() / 2,
          "humanoid regions should cover most particles");

    const auto anatomy = runtime.island_anatomy(runtime.islands().front().id);
    check(anatomy.size() == runtime.region_names().size(),
          "island anatomy must report every region");

    const auto& p0 = runtime.body().particles().front().position;
    check(p0.x > 0.5 && p0.z < -2.0, "world offset should translate particles");
}

void test_plane_cut_creates_stable_child_island() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;
    const IslandId root = runtime.islands().front().id;

    for (int i = 0; i < 5; ++i) runtime.step(kDt);

    const auto report = runtime.apply_damage(shoulder_cut(h));
    check(report.broken_count() > 0, "shoulder cut should break constraints");
    check(runtime.islands().size() == 2, "shoulder cut should create two islands");

    const IslandState* body = island_containing_region(runtime, "torso");
    const IslandState* arm = island_containing_region(runtime, "right_hand");
    check(body && arm && body != arm, "arm must separate from torso");
    if (!body || !arm) return;

    check(body->id == root, "larger island must keep the original id");
    check(arm->parent == root, "detached island must record its parent");
    check(arm->created_by_event == report.event_id,
          "detached island must record the creating damage event");
    check(body->rig_authoritative, "torso island stays rig-authoritative");
    check(!arm->rig_authoritative, "detached arm loses rig authority");

    check(runtime.topology_events().size() == 1, "one topology event expected");
    const auto& event = runtime.topology_events().front();
    check(event.parent == root && event.children.size() == 2
              && event.children.front() == root
              && event.children.back() == arm->id,
          "topology event should list parent then new child");

    const auto status = runtime.region_status("right_hand");
    check(status.islands.size() == 1 && status.islands.front() == arm->id,
          "right hand region should live entirely in the arm island");
    check(status.rig_connected_particles == 0,
          "right hand should not be rig connected after severance");

    // A second, unrelated step keeps ids stable.
    const IslandId arm_id = arm->id;
    for (int i = 0; i < 10; ++i) runtime.step(kDt);
    check(runtime.find_island(arm_id) != nullptr, "arm id must remain stable");
    check(runtime.find_island(root) != nullptr, "root id must remain stable");
}

void test_split_preserves_momentum_and_positions() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;

    // Walk the rig so the body carries momentum.
    for (int i = 0; i < 30; ++i) {
        for (sarx::BoneId b = 0; b < h.rest_bone_positions.size(); ++b) {
            runtime.body().set_bone_target(
                b,
                h.rest_bone_positions[b] + Vec3{0.0, 0.0, 0.02 * (i + 1)});
        }
        runtime.step(kDt);
    }

    const Vec3 before = runtime.body().total_linear_momentum();
    const auto positions_before = runtime.body().particles();
    (void)runtime.apply_damage(shoulder_cut(h));

    Vec3 after{};
    for (const auto& island : runtime.islands()) {
        after += island.linear_velocity * island.mass;
    }
    check(sarx::nearly_equal(before, after, 1e-9),
          "island momenta must sum to pre-split momentum");
    check(before.z > 0.1, "walking body should carry forward momentum");

    bool unchanged = true;
    for (std::size_t i = 0; i < positions_before.size(); ++i) {
        unchanged = unchanged
            && sarx::nearly_equal(
                positions_before[i].position,
                runtime.body().particles()[i].position,
                0.0);
    }
    check(unchanged, "damage must not move particles");

    const IslandState* arm = island_containing_region(runtime, "right_hand");
    check(arm && arm->linear_velocity.z > 0.5,
          "detached arm should inherit forward velocity");
}

void test_rig_authority_toggle() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;
    runtime.config().ground.enabled = false;
    runtime.config().step.gravity = {};

    const double start_z = runtime.islands().front().center_of_mass.z;
    runtime.set_rig_authority(false);
    for (sarx::BoneId b = 0; b < h.rest_bone_positions.size(); ++b) {
        runtime.body().set_bone_target(b, h.rest_bone_positions[b] + Vec3{0, 0, 1});
    }
    for (int i = 0; i < 20; ++i) runtime.step(kDt);
    check(std::abs(runtime.islands().front().center_of_mass.z - start_z) < 1e-6,
          "passive body must ignore rig targets");

    runtime.set_rig_authority(true);
    for (int i = 0; i < 40; ++i) runtime.step(kDt);
    check(runtime.islands().front().center_of_mass.z - start_z > 0.8,
          "rig authority should pull the body toward targets");
}

void test_ground_contact_rests_detached_island() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;
    (void)runtime.apply_damage(shoulder_cut(h));

    for (int i = 0; i < 180; ++i) runtime.step(kDt);
    const IslandState* arm = island_containing_region(runtime, "right_hand");
    check(arm && arm->bounds_min.y >= -1e-9, "arm must not fall through ground");
    check(arm && arm->bounds_max.y < 0.45, "arm should come to rest on ground");
    check(arm && sarx::length(arm->linear_velocity) < 0.25,
          "arm should settle under friction");
}

void test_external_island_acceleration_is_local() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;
    (void)runtime.apply_damage(shoulder_cut(h));
    for (int i = 0; i < 120; ++i) runtime.step(kDt);

    const IslandState* arm = island_containing_region(runtime, "right_hand");
    const IslandState* body = island_containing_region(runtime, "torso");
    const IslandId arm_id = arm->id;
    const IslandId body_id = body->id;
    const Vec3 arm_start = arm->center_of_mass;
    const Vec3 body_start = body->center_of_mass;

    for (int i = 0; i < 30; ++i) {
        runtime.add_island_acceleration(arm_id, {0.0, 12.0, 4.0});
        runtime.step(kDt);
    }
    check(!runtime.has_pending_external_acceleration(),
          "external acceleration must clear after a step");

    const Vec3 arm_end = runtime.find_island(arm_id)->center_of_mass;
    const Vec3 body_end = runtime.find_island(body_id)->center_of_mass;
    check(arm_end.z - arm_start.z > 0.2, "accelerated island should move");
    check(sarx::length(body_end - body_start) < 0.05,
          "other islands should not receive external acceleration");
}

void test_region_integrity_tracks_local_damage() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;
    runtime.define_sphere_region("probe", {0.0, 1.72, 0.0}, 0.15);

    const auto before = runtime.region_status("probe");
    check(before.total_particles >= 8, "probe region should contain particles");
    check(before.integrity() == 1.0, "undamaged region integrity is 1");

    DamageCommand shot;
    shot.kind = DamageCommandKind::Capsule;
    shot.capsule.a = {0.0, 1.72, 1.0};
    shot.capsule.b = {0.0, 1.72, -1.0};
    shot.capsule.radius = 0.09;
    shot.capsule.energy = 6.0;
    shot.capsule.mode = DamageMode::Cut;
    (void)runtime.apply_damage(shot);

    const auto after = runtime.region_status("probe");
    check(after.integrity() < 0.8, "capsule through region should reduce integrity");

    const auto torso = runtime.region_status("torso");
    check(torso.integrity() == 1.0, "distant region must be unaffected");
}

void test_viability_uses_island_anatomy() {
    auto h = sarx::build_humanoid_runtime();
    auto& runtime = h.runtime;
    (void)runtime.apply_damage(shoulder_cut(h));

    sarx::MotionCapability reach;
    reach.motion_id = "reach_with_right_arm";
    reach.required_regions = {{"right_arm", 0.6}, {"torso", 0.6}};

    const auto body_island = island_containing_region(runtime, "torso")->id;
    const auto arm_island = island_containing_region(runtime, "right_hand")->id;

    const auto on_body = sarx::evaluate_motion_viability(
        reach, runtime.island_anatomy(body_island));
    const auto on_arm = sarx::evaluate_motion_viability(
        reach, runtime.island_anatomy(arm_island));
    check(on_body.state == sarx::MotionViability::Invalid,
          "torso island lost the right arm");
    check(on_arm.state == sarx::MotionViability::Invalid,
          "arm island has no torso");

    sarx::MotionCapability crawl;
    crawl.required_regions = {{"right_arm", 0.6}};
    check(sarx::evaluate_motion_viability(crawl, runtime.island_anatomy(arm_island))
              .state == sarx::MotionViability::Viable,
          "arm island satisfies an arm-only capability");
}

void test_linear_damping_is_opt_in() {
    sarx::RuntimeConfig config;
    config.ground.enabled = false;
    config.step.gravity = {};
    auto undamped = sarx::build_humanoid_runtime({}, {}, config);
    config.linear_damping = 2.0;
    auto damped = sarx::build_humanoid_runtime({}, {}, config);

    for (auto* h : {&undamped, &damped}) {
        h->runtime.set_rig_authority(false);
        for (auto& p : h->runtime.body().particles()) p.velocity = {0.0, 0.0, 1.0};
        for (int i = 0; i < 30; ++i) h->runtime.step(kDt);
    }
    const double v_free = undamped.runtime.islands().front().linear_velocity.z;
    const double v_damped = damped.runtime.islands().front().linear_velocity.z;
    check(std::abs(v_free - 1.0) < 1e-9, "default runtime keeps Body's undamped motion");
    check(v_damped < 0.5 && v_damped > 0.2, "linear damping decays velocity at ~exp(-c t)");
}

sarx::HumanoidRuntime run_scripted(bool replay_from_log,
                                   const std::vector<sarx::RuntimeLogEntry>& log) {
    auto h = sarx::build_humanoid_runtime({}, {0.0, 0.0, 1.0});
    auto& runtime = h.runtime;
    std::size_t next = 0;
    for (int i = 0; i < 90; ++i) {
        if (replay_from_log) {
            while (next < log.size() && log[next].step_index == runtime.step_index()) {
                (void)runtime.apply_damage(log[next++].command);
            }
        } else if (i == 20) {
            auto cut = shoulder_cut(h);
            cut.plane_cut.center.z += 0.01 * (i - 1);
            (void)runtime.apply_damage(cut);
        } else if (i == 50) {
            DamageCommand neck;
            neck.kind = DamageCommandKind::PlaneCut;
            neck.plane_cut.center =
                Vec3{0.0, 1.55, 0.01 * (i - 1)} + h.world_offset;
            neck.plane_cut.normal = {0.0, 1.0, 0.0};
            neck.plane_cut.radius = 0.22;
            neck.plane_cut.energy = 4.0;
            (void)runtime.apply_damage(neck);
        }
        for (sarx::BoneId b = 0; b < h.rest_bone_positions.size(); ++b) {
            runtime.body().set_bone_target(
                b, h.rest_bone_positions[b] + Vec3{0.0, 0.0, 0.01 * i});
        }
        if (i % 3 == 0 && runtime.islands().size() > 1) {
            runtime.add_island_acceleration(runtime.islands().back().id, {0, 3, 1});
        }
        runtime.step(kDt);
    }
    return h;
}

void test_deterministic_replay() {
    const auto first = run_scripted(false, {});
    const auto second = run_scripted(false, {});
    const auto replayed = run_scripted(true, first.runtime.damage_log());

    for (const auto* other : {&second, &replayed}) {
        const auto& a = first.runtime.body().particles();
        const auto& b = other->runtime.body().particles();
        bool identical = a.size() == b.size();
        for (std::size_t i = 0; identical && i < a.size(); ++i) {
            identical = a[i].position.x == b[i].position.x
                && a[i].position.y == b[i].position.y
                && a[i].position.z == b[i].position.z;
        }
        check(identical, "scripted runs must be bitwise identical");

        const auto& ia = first.runtime.islands();
        const auto& ib = other->runtime.islands();
        bool same_islands = ia.size() == ib.size();
        for (std::size_t i = 0; same_islands && i < ia.size(); ++i) {
            same_islands = ia[i].id == ib[i].id
                && ia[i].parent == ib[i].parent
                && ia[i].particles == ib[i].particles;
        }
        check(same_islands, "scripted runs must produce identical island ids");
    }
    check(first.runtime.islands().size() == 3,
          "shoulder + neck cuts should produce three islands");
    check(first.runtime.damage_log().size() == 2, "damage log records both cuts");
}

} // namespace

int main() {
    test_initial_humanoid_is_single_rig_island();
    test_plane_cut_creates_stable_child_island();
    test_split_preserves_momentum_and_positions();
    test_rig_authority_toggle();
    test_ground_contact_rests_detached_island();
    test_external_island_acceleration_is_local();
    test_region_integrity_tracks_local_damage();
    test_viability_uses_island_anatomy();
    test_deterministic_replay();
    test_linear_damping_is_opt_in();

    if (failures > 0) {
        std::cerr << failures << " runtime test failure(s)\n";
        return 1;
    }
    std::cout << "sarx runtime tests passed\n";
    return 0;
}
