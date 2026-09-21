#include "sarx/body.hpp"
#include "sarx/damage.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using sarx::Body;
using sarx::CapsuleDamage;
using sarx::DamageMode;
using sarx::DamageSystem;
using sarx::MaterialResponse;
using sarx::SphereDamage;
using sarx::StepConfig;
using sarx::Vec3;

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

double distance(const Vec3& a, const Vec3& b) {
    return sarx::length(a - b);
}

StepConfig no_gravity() {
    StepConfig cfg;
    cfg.substeps = 4;
    cfg.solver_iterations = 12;
    cfg.gravity = {};
    return cfg;
}

void test_compliant_animation_target() {
    Body body;
    const auto root = body.add_bone(sarx::kNoParent, {0.0, 0.0, 0.0});
    const auto p = body.add_particle({0.0, 0.0, 0.0}, 1.0);
    body.add_attachment(p, root, {}, 1e-8);

    body.set_bone_target(root, {1.0, 0.0, 0.0});
    for (int i = 0; i < 8; ++i) {
        body.step(1.0 / 60.0, no_gravity());
    }

    check(distance(body.particles()[p].position, {1.0, 0.0, 0.0}) < 1e-3,
          "root-connected particle should follow its animated target");
}

void test_progressive_structural_damage() {
    Body body;
    const auto a = body.add_particle({0.0, 0.0, 0.0});
    const auto b = body.add_particle({1.0, 0.0, 0.0});
    const auto c = body.add_structural_constraint(a, b, 0.0, 1.0);

    body.damage_structural(c, 0.4);
    check(body.structural_constraints()[c].active,
          "constraint should survive sub-threshold damage");

    body.damage_structural(c, 0.6);
    check(!body.structural_constraints()[c].active,
          "constraint should fail at its damage threshold");
}

void test_severance_creates_free_dynamic_island() {
    Body body;

    const auto root_bone = body.add_bone(sarx::kNoParent, {0.0, 0.0, 0.0});
    const auto arm_bone = body.add_bone(root_bone, {2.0, 0.0, 0.0});

    const auto torso0 = body.add_particle({0.0, 0.0, 0.0});
    const auto torso1 = body.add_particle({1.0, 0.0, 0.0});
    const auto arm0 = body.add_particle({2.0, 0.0, 0.0});
    const auto arm1 = body.add_particle({3.0, 0.0, 0.0});

    body.add_structural_constraint(torso0, torso1);
    const auto shoulder_bridge = body.add_structural_constraint(torso1, arm0);
    body.add_structural_constraint(arm0, arm1);

    body.add_attachment(torso0, root_bone, {0.0, 0.0, 0.0}, 1e-8);
    body.add_attachment(torso1, root_bone, {1.0, 0.0, 0.0}, 1e-8);
    body.add_attachment(arm0, arm_bone, {0.0, 0.0, 0.0}, 1e-8);
    body.add_attachment(arm1, arm_bone, {1.0, 0.0, 0.0}, 1e-8);

    auto initial = body.islands();
    check(initial.size() == 1, "intact body should be one physical island");
    check(initial[0].rig_authoritative, "intact body should be rig authoritative");

    body.particles()[arm0].velocity = {0.0, 2.0, 0.0};
    body.particles()[arm1].velocity = {0.0, 2.0, 0.0};

    body.break_structural(shoulder_bridge);
    body.break_bone_joint(arm_bone);

    auto severed = body.islands();
    check(severed.size() == 2, "severance should split the physical graph into two islands");

    bool found_free_arm = false;
    for (const auto& island : severed) {
        if (island.particles.size() == 2
            && island.particles[0] >= arm0
            && !island.rig_authoritative) {
            found_free_arm = true;
            check(std::abs(island.linear_momentum.y - 4.0) < 1e-9,
                  "detached island should retain its pre-severance momentum");
        }
    }
    check(found_free_arm, "detached arm should be classified as a free dynamic island");

    const Vec3 arm0_before = body.particles()[arm0].position;
    body.set_bone_target(arm_bone, {50.0, 50.0, 0.0});
    body.step(1.0 / 60.0, no_gravity());

    check(body.particles()[arm0].position.x < 10.0,
          "disconnected arm must not snap toward an unreachable animation target");
    check(body.particles()[arm0].position.y > arm0_before.y,
          "detached arm should continue along its physical velocity");
}

void test_attachment_failure_removes_animation_authority() {
    Body body;
    const auto root = body.add_bone(sarx::kNoParent, {0.0, 0.0, 0.0});
    const auto p = body.add_particle({0.0, 0.0, 0.0});
    const auto attachment = body.add_attachment(p, root, {}, 1e-8, 1.0);

    check(body.islands()[0].rig_authoritative,
          "active root attachment should establish rig authority");

    body.damage_attachment(attachment, 1.0);

    check(!body.islands()[0].rig_authoritative,
          "failed attachment should remove rig authority from an isolated particle");
}

void test_blade_sweep_automatically_severs_shoulder() {
    Body body;
    DamageSystem damage;

    constexpr sarx::MaterialId flesh = 1;
    damage.materials().set(flesh, MaterialResponse{1.0, 1.5});

    const auto root_bone = body.add_bone(
        sarx::kNoParent, {0.0, 0.0, 0.0}, 1.0, flesh);
    const auto arm_bone = body.add_bone(
        root_bone, {2.0, 0.0, 0.0}, 1.0, flesh);

    const auto torso0 = body.add_particle({0.0, 0.0, 0.0});
    const auto torso1 = body.add_particle({1.0, 0.0, 0.0});
    const auto arm0 = body.add_particle({2.0, 0.0, 0.0});
    const auto arm1 = body.add_particle({3.0, 0.0, 0.0});

    body.add_structural_constraint(torso0, torso1, 0.0, 1.0, flesh);
    const auto shoulder_bridge =
        body.add_structural_constraint(torso1, arm0, 0.0, 1.0, flesh);
    body.add_structural_constraint(arm0, arm1, 0.0, 1.0, flesh);

    body.add_attachment(torso0, root_bone, {}, 1e-8, 1.0, flesh);
    body.add_attachment(torso1, root_bone, {1.0, 0.0, 0.0}, 1e-8, 1.0, flesh);
    body.add_attachment(arm0, arm_bone, {}, 1e-8, 1.0, flesh);
    body.add_attachment(arm1, arm_bone, {1.0, 0.0, 0.0}, 1e-8, 1.0, flesh);

    body.particles()[arm0].velocity = {0.0, 2.0, 0.0};
    body.particles()[arm1].velocity = {0.0, 2.0, 0.0};

    CapsuleDamage blade;
    blade.a = {1.5, -1.0, 0.0};
    blade.b = {1.5, 1.0, 0.0};
    blade.radius = 0.1;
    blade.energy = 1.25;
    blade.mode = DamageMode::Cut;

    const auto report = damage.apply_capsule(body, blade);

    check(!body.structural_constraints()[shoulder_bridge].active,
          "blade geometry should break the intersected shoulder bridge");
    check(!body.bones()[arm_bone].joint_to_parent_active,
          "blade geometry should break the intersected shoulder rig joint");
    check(report.broken_count() >= 2,
          "damage report should expose physical and rig fracture events");

    const auto islands = body.islands();
    check(islands.size() == 2,
          "automatic spatial damage should create a detached arm island");

    bool free_arm = false;
    for (const auto& island : islands) {
        if (island.particles.size() == 2 && island.particles[0] == arm0) {
            free_arm = !island.rig_authoritative;
            check(std::abs(island.linear_momentum.y - 4.0) < 1e-9,
                  "automatic severance must preserve detached arm momentum");
        }
    }
    check(free_arm,
          "automatically severed arm should lose root animation authority");
}

void test_material_cut_resistance_changes_failure() {
    Body body;
    DamageSystem damage;

    constexpr sarx::MaterialId flesh = 1;
    constexpr sarx::MaterialId bone = 2;
    damage.materials().set(flesh, MaterialResponse{1.0, 1.0});
    damage.materials().set(bone, MaterialResponse{3.0, 3.0});

    const auto f0 = body.add_particle({0.0, 0.0, 0.0});
    const auto f1 = body.add_particle({1.0, 0.0, 0.0});
    const auto b0 = body.add_particle({0.0, 0.0, 0.05});
    const auto b1 = body.add_particle({1.0, 0.0, 0.05});

    const auto flesh_link =
        body.add_structural_constraint(f0, f1, 0.0, 1.0, flesh);
    const auto bone_link =
        body.add_structural_constraint(b0, b1, 0.0, 1.0, bone);

    CapsuleDamage blade;
    blade.a = {0.5, -1.0, 0.025};
    blade.b = {0.5, 1.0, 0.025};
    blade.radius = 0.1;
    blade.energy = 1.4;
    blade.mode = DamageMode::Cut;

    const auto report = damage.apply_capsule(body, blade);
    check(!report.events.empty(),
          "material response fixture should emit spatial damage events");

    check(!body.structural_constraints()[flesh_link].active,
          "same blade should sever low-resistance tissue");
    check(body.structural_constraints()[bone_link].active,
          "same blade should not sever high-resistance material");
    check(body.structural_constraints()[bone_link].damage > 0.0,
          "resistant material should still accumulate sub-threshold damage");
}

void test_sphere_damage_is_spatially_local() {
    Body body;
    DamageSystem damage;

    const auto n0 = body.add_particle({0.0, 0.0, 0.0});
    const auto n1 = body.add_particle({1.0, 0.0, 0.0});
    const auto f0 = body.add_particle({3.0, 0.0, 0.0});
    const auto f1 = body.add_particle({4.0, 0.0, 0.0});

    const auto near_link = body.add_structural_constraint(n0, n1);
    const auto far_link = body.add_structural_constraint(f0, f1);

    SphereDamage impact;
    impact.center = {0.5, 0.0, 0.0};
    impact.radius = 0.3;
    impact.energy = 1.2;
    impact.mode = DamageMode::Blunt;

    const auto report = damage.apply_sphere(body, impact);

    check(!body.structural_constraints()[near_link].active,
          "sphere damage should break a directly intersected link");
    check(body.structural_constraints()[far_link].active,
          "sphere damage should not affect distant topology");
    check(report.broken_count() == 1,
          "localized impact should report only the nearby break in this fixture");
}


void test_anisotropic_cut_response() {
    Body body;
    DamageSystem damage;

    constexpr sarx::MaterialId fiber = 7;
    MaterialResponse response;
    response.cut_resistance = 1.0;
    response.blunt_resistance = 1.0;
    response.fiber_direction = {1.0, 0.0, 0.0};
    response.longitudinal_cut_multiplier = 2.0;
    response.transverse_cut_multiplier = 0.5;
    damage.materials().set(fiber, response);

    const auto x0 = body.add_particle({-1.0, 0.0, 0.0});
    const auto x1 = body.add_particle({1.0, 0.0, 0.0});
    const auto y0 = body.add_particle({0.0, -1.0, 0.0});
    const auto y1 = body.add_particle({0.0, 1.0, 0.0});

    const auto along =
        body.add_structural_constraint(x0, x1, 0.0, 1.0, fiber);
    const auto across =
        body.add_structural_constraint(y0, y1, 0.0, 1.0, fiber);

    CapsuleDamage blade;
    blade.a = {0.0, 0.0, -1.0};
    blade.b = {0.0, 0.0, 1.0};
    blade.radius = 0.1;
    blade.energy = 0.8;
    blade.mode = DamageMode::Cut;

    const auto report = damage.apply_capsule(body, blade);
    check(report.events.size() == 2,
          "anisotropy fixture should touch both crossing constraints");
    check(body.structural_constraints()[along].active,
          "cut along strong fibers should remain sub-threshold");
    check(!body.structural_constraints()[across].active,
          "same cut across weak fiber direction should fail");
}

void test_strain_driven_brittle_fracture() {
    Body body;
    DamageSystem damage;

    constexpr sarx::MaterialId brittle = 8;
    MaterialResponse response;
    response.tensile_yield_strain = 0.10;
    response.tensile_break_strain = 0.25;
    response.strain_damage_rate = 1.0;
    damage.materials().set(brittle, response);

    const auto p0 = body.add_particle({0.0, 0.0, 0.0});
    const auto p1 = body.add_particle({1.0, 0.0, 0.0});
    const auto link =
        body.add_structural_constraint(p0, p1, 0.0, 1.0, brittle);

    body.particles()[p1].position = {1.30, 0.0, 0.0};

    sarx::StrainDamage strain;
    strain.dt = 1.0 / 60.0;
    const auto report = damage.apply_strain(body, strain);

    check(!body.structural_constraints()[link].active,
          "strain above brittle break threshold should fracture immediately");
    check(report.broken_count() == 1,
          "brittle strain fracture should be represented in the event stream");
    check(!report.events.empty()
              && report.events[0].source == sarx::DamageSource::Strain,
          "strain fracture should identify its non-spatial source");
}

void test_progressive_strain_damage() {
    Body body;
    DamageSystem damage;

    constexpr sarx::MaterialId tissue = 9;
    MaterialResponse response;
    response.tensile_yield_strain = 0.10;
    response.tensile_break_strain = 1.0;
    response.strain_damage_rate = 1.0;
    damage.materials().set(tissue, response);

    const auto p0 = body.add_particle({0.0, 0.0, 0.0});
    const auto p1 = body.add_particle({1.0, 0.0, 0.0});
    const auto link =
        body.add_structural_constraint(p0, p1, 0.0, 1.0, tissue);

    body.particles()[p1].position = {1.30, 0.0, 0.0};

    sarx::StrainDamage strain;
    strain.dt = 1.0;

    for (int i = 0; i < 4; ++i) {
        const auto report = damage.apply_strain(body, strain);
        check(report.broken_count() == 0,
              "subcritical overstrain should accumulate before terminal tearing");
    }

    check(body.structural_constraints()[link].active,
          "progressive tissue should still be intact before accumulated threshold");

    const auto terminal = damage.apply_strain(body, strain);
    check(!body.structural_constraints()[link].active,
          "repeated overstrain should eventually tear progressive tissue");
    check(terminal.broken_count() == 1,
          "terminal progressive tear should be emitted exactly once");
}

void test_damage_history_replays_deterministically() {
    Body original;
    DamageSystem author;

    constexpr sarx::MaterialId tissue = 10;
    author.materials().set(tissue, MaterialResponse{1.0, 1.0});

    const auto a0 = original.add_particle({0.0, 0.0, 0.0});
    const auto a1 = original.add_particle({1.0, 0.0, 0.0});
    const auto b0 = original.add_particle({2.0, 0.0, 0.0});
    const auto b1 = original.add_particle({3.0, 0.0, 0.0});
    original.add_structural_constraint(a0, a1, 0.0, 1.0, tissue);
    original.add_structural_constraint(b0, b1, 0.0, 1.0, tissue);

    Body replayed = original;

    CapsuleDamage cut;
    cut.a = {0.5, -1.0, 0.0};
    cut.b = {0.5, 1.0, 0.0};
    cut.radius = 0.1;
    cut.energy = 0.6;
    cut.event_id = 100;

    SphereDamage impact;
    impact.center = {2.5, 0.0, 0.0};
    impact.radius = 0.2;
    impact.energy = 1.2;
    impact.event_id = 101;

    const auto first = author.apply_capsule(original, cut);
    const auto second = author.apply_sphere(original, impact);

    check(first.event_id == 100 && second.event_id == 101,
          "caller-supplied damage IDs should remain authoritative");
    check(author.history().size() == 2,
          "authoritative damage operations should be recorded as replay commands");

    DamageSystem replica;
    replica.materials().set(tissue, MaterialResponse{1.0, 1.0});
    const auto reports = replica.replay(replayed, author.history());

    check(reports.size() == 2
              && reports[0].event_id == 100
              && reports[1].event_id == 101,
          "replay should preserve event ordering and IDs");

    for (std::size_t i = 0; i < original.structural_constraints().size(); ++i) {
        const auto& lhs = original.structural_constraints()[i];
        const auto& rhs = replayed.structural_constraints()[i];
        check(lhs.active == rhs.active,
              "replay should reproduce structural topology");
        check(std::abs(lhs.damage - rhs.damage) < 1e-12,
              "replay should reproduce accumulated damage");
    }
}

} // namespace

int main() {
    test_compliant_animation_target();
    test_progressive_structural_damage();
    test_severance_creates_free_dynamic_island();
    test_attachment_failure_removes_animation_authority();
    test_blade_sweep_automatically_severs_shoulder();
    test_material_cut_resistance_changes_failure();
    test_sphere_damage_is_spatially_local();
    test_anisotropic_cut_response();
    test_strain_driven_brittle_fracture();
    test_progressive_strain_damage();
    test_damage_history_replays_deterministically();

    if (failures != 0) {
        std::cerr << failures << " SARX test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "SARX V0.3 core tests passed.\n";
    return EXIT_SUCCESS;
}
