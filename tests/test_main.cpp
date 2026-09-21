#include "sarx/body.hpp"
#include "sarx/damage.hpp"
#include "sarx/broad_phase.hpp"
#include "sarx/volume.hpp"
#include "sarx/adaptive.hpp"
#include "sarx/soa.hpp"
#include "sarx/humanoid.hpp"

#include <algorithm>
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


void test_persistent_wound_descriptor() {
    Body body;
    DamageSystem damage;

    const auto p0 = body.add_particle({-1.0, 0.0, 0.0});
    const auto p1 = body.add_particle({1.0, 0.0, 0.0});
    body.add_structural_constraint(p0, p1);

    CapsuleDamage blade;
    blade.a = {0.0, -1.0, 0.0};
    blade.b = {0.0, 1.0, 0.0};
    blade.radius = 0.1;
    blade.energy = 1.2;
    blade.mode = DamageMode::Cut;
    blade.event_id = 500;
    blade.cut_normal = {0.0, 0.0, 1.0};

    const auto report = damage.apply_capsule(body, blade);

    check(report.broken_count() == 1,
          "wound fixture should produce one fracture");
    check(damage.wounds().size() == 1,
          "successful cut should create one persistent wound descriptor");

    if (!damage.wounds().empty()) {
        const auto& wound = damage.wounds()[0];
        check(wound.event_id == 500,
              "wound should retain authoritative damage event ID");
        check(sarx::nearly_equal(wound.normal, {0.0, 0.0, 1.0}),
              "wound should preserve normalized cut-surface normal");
        check(wound.broken_target_count == 1,
              "wound should record how many authoritative targets failed");
    }

    damage.clear_wounds();
    check(damage.wounds().empty(),
          "wound history should be independently clearable");
}

void test_broad_phase_matches_full_scan() {
    Body indexed_body;

    for (int i = 0; i < 64; ++i) {
        const double x = static_cast<double>(i) * 4.0;
        const auto a = indexed_body.add_particle({x, 0.0, 0.0});
        const auto b = indexed_body.add_particle({x + 1.0, 0.0, 0.0});
        indexed_body.add_structural_constraint(a, b);
    }

    Body full_scan_body = indexed_body;

    SphereDamage impact;
    impact.center = {0.5, 0.0, 0.0};
    impact.radius = 0.3;
    impact.energy = 1.2;
    impact.mode = DamageMode::Blunt;
    impact.event_id = 700;

    sarx::DamageBroadPhase broad_phase;
    broad_phase.rebuild(indexed_body, 1.0);
    const auto query = broad_phase.query_sphere(impact);

    check(broad_phase.indexed_primitives() == 64,
          "broad phase should index each active structural primitive");
    check(query.candidates.size() < broad_phase.indexed_primitives() / 4,
          "localized query should reject most distant constraints before exact testing");

    DamageSystem indexed_damage;
    DamageSystem full_damage;

    const auto indexed_report =
        indexed_damage.apply_sphere(indexed_body, impact, query.candidates);
    const auto full_report =
        full_damage.apply_sphere(full_scan_body, impact);

    check(indexed_report.broken_count() == full_report.broken_count(),
          "broad-phase and full-scan paths should report the same break count");

    for (std::size_t i = 0; i < indexed_body.structural_constraints().size(); ++i) {
        const auto& lhs = indexed_body.structural_constraints()[i];
        const auto& rhs = full_scan_body.structural_constraints()[i];
        check(lhs.active == rhs.active,
              "broad-phase routing must preserve exact topology outcome");
        check(std::abs(lhs.damage - rhs.damage) < 1e-12,
              "broad-phase routing must preserve exact accumulated damage");
    }
}


void test_voxel_lattice_generation() {
    sarx::VoxelLatticeSpec spec;
    spec.origin = {0.0, 0.0, 0.0};
    spec.nx = 3;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.5;
    spec.include_diagonals = false;
    spec.default_material = 1;

    sarx::MaterialRegion right_side;
    right_side.min = {0.75, -1.0, -1.0};
    right_side.max = {2.0, 2.0, 2.0};
    right_side.material = 5;
    right_side.priority = 10;

    auto lattice = sarx::build_voxel_lattice(spec, {right_side});

    check(lattice.particle_count() == 27,
          "3x3x3 lattice should generate 27 particles");
    check(lattice.body.structural_constraints().size() == 54,
          "3x3x3 axial lattice should generate 54 unique neighbor links");

    check(lattice.particle(2, 1, 1) == 14,
          "lattice index mapping should remain deterministic");
    check(lattice.particle_materials[lattice.particle(2, 1, 1)] == 5,
          "particle material region should override default material");

    bool found_region_constraint = false;
    for (const auto& constraint : lattice.body.structural_constraints()) {
        if (constraint.material == 5) {
            found_region_constraint = true;
            break;
        }
    }
    check(found_region_constraint,
          "constraint midpoint should inherit material-region assignment");
}

void test_voxel_lattice_diagonal_connectivity() {
    sarx::VoxelLatticeSpec axial;
    axial.nx = 3;
    axial.ny = 3;
    axial.nz = 3;
    axial.spacing = 0.5;
    axial.include_diagonals = false;

    sarx::VoxelLatticeSpec isotropic = axial;
    isotropic.include_diagonals = true;

    const auto axial_lattice = sarx::build_voxel_lattice(axial);
    const auto isotropic_lattice = sarx::build_voxel_lattice(isotropic);

    check(isotropic_lattice.body.structural_constraints().size()
              > axial_lattice.body.structural_constraints().size(),
          "diagonal lattice mode should add shear/body-diagonal support");
}

void test_automatic_bone_embedding() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 3;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.5;
    spec.include_diagonals = false;

    auto lattice = sarx::build_voxel_lattice(spec);

    const Vec3 center{0.5, 0.5, 0.5};
    const auto embedded = sarx::embed_bone(
        lattice,
        sarx::kNoParent,
        center,
        0.51,
        1e-8,
        1.0,
        3);

    check(embedded.bone == 0,
          "first embedded bone should receive deterministic bone ID zero");
    check(embedded.attachments.size() == 7,
          "radius 0.51 should attach center plus six axial neighbors");
    check(lattice.body.bones().size() == 1,
          "bone embedding should add the rig bone to the physical body");

    for (const auto attachment_id : embedded.attachments) {
        const auto& attachment = lattice.body.attachments()[attachment_id];
        const Vec3 particle_position =
            lattice.body.particles()[attachment.particle].position;
        const Vec3 reconstructed =
            lattice.body.bones()[embedded.bone].animated_position
            + attachment.local_offset;

        check(sarx::nearly_equal(particle_position, reconstructed),
              "embedded attachment offset should reconstruct its particle rest pose");
    }
}

void test_embedded_child_bone_uses_rig_parent() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 5;
    spec.ny = 2;
    spec.nz = 2;
    spec.spacing = 0.5;
    spec.include_diagonals = false;

    auto lattice = sarx::build_voxel_lattice(spec);

    const auto root = sarx::embed_bone(
        lattice,
        sarx::kNoParent,
        {0.5, 0.25, 0.25},
        0.8);

    const auto child = sarx::embed_bone(
        lattice,
        root.bone,
        {1.5, 0.25, 0.25},
        0.8);

    check(lattice.body.bones()[child.bone].parent == root.bone,
          "embedded child bone should preserve requested rig parent");
    check(lattice.body.bone_root_connected(child.bone),
          "embedded child should initially inherit root animation authority");
    check(!child.attachments.empty(),
          "embedded child should automatically capture nearby physical particles");
}


double signed_tet_volume(
    const Vec3& p0,
    const Vec3& p1,
    const Vec3& p2,
    const Vec3& p3) {
    return sarx::dot(p1 - p0, sarx::cross(p2 - p0, p3 - p0)) / 6.0;
}

void test_material_frame_follows_deformation() {
    Body body;
    DamageSystem damage;

    constexpr sarx::MaterialId fibers = 20;
    MaterialResponse material;
    material.cut_resistance = 1.0;
    material.blunt_resistance = 1.0;
    material.fiber_direction = {1.0, 0.0, 0.0};
    material.longitudinal_cut_multiplier = 2.0;
    material.transverse_cut_multiplier = 0.5;
    damage.materials().set(fibers, material);

    const auto p0 = body.add_particle({-0.5, 0.0, 0.0});
    const auto p1 = body.add_particle({0.5, 0.0, 0.0});
    const auto link =
        body.add_structural_constraint(p0, p1, 0.0, 1.0, fibers);

    check(sarx::nearly_equal(
              body.structural_constraints()[link].rest_direction,
              {1.0, 0.0, 0.0}),
          "structural constraint should retain its rest material axis");

    body.particles()[p0].position = {0.0, -0.5, 0.0};
    body.particles()[p1].position = {0.0, 0.5, 0.0};

    CapsuleDamage blade;
    blade.a = {0.0, 0.0, -1.0};
    blade.b = {0.0, 0.0, 1.0};
    blade.radius = 0.1;
    blade.energy = 0.8;
    blade.mode = DamageMode::Cut;

    const auto report = damage.apply_capsule(body, blade);
    check(!report.events.empty(),
          "deformed material-frame fixture should receive cut damage");
    check(body.structural_constraints()[link].active,
          "fiber direction should rotate with the deformed link and retain longitudinal resistance");
    check(body.structural_constraints()[link].damage < 0.5,
          "transported longitudinal fibers should reduce damage after a 90-degree body rotation");
}

void test_joint_capsule_extends_damage_geometry() {
    Body body;
    DamageSystem damage;

    const auto root = body.add_bone(
        sarx::kNoParent,
        {0.0, 0.0, 0.0});
    const auto child = body.add_bone(
        root,
        {1.0, 0.0, 0.0},
        1.0,
        sarx::kDefaultMaterial,
        0.25);

    CapsuleDamage blade;
    blade.a = {0.5, -1.0, 0.20};
    blade.b = {0.5, 1.0, 0.20};
    blade.radius = 0.05;
    blade.energy = 4.0;
    blade.mode = DamageMode::Cut;

    const auto report = damage.apply_capsule(body, blade);

    check(report.broken_count() == 1,
          "blade inside joint capsule radius should break the joint");
    check(!body.bones()[child].joint_to_parent_active,
          "joint capsule should make the parent-child link physically hittable away from its center line");
}

void test_joint_capsule_is_indexed_by_broad_phase() {
    Body body;

    const auto root = body.add_bone(
        sarx::kNoParent,
        {0.0, 0.0, 0.0});
    const auto child = body.add_bone(
        root,
        {1.0, 0.0, 0.0},
        1.0,
        sarx::kDefaultMaterial,
        0.25);

    sarx::DamageBroadPhase broad_phase;
    broad_phase.rebuild(body, 0.1);

    SphereDamage query;
    query.center = {0.5, 0.20, 0.0};
    query.radius = 0.05;
    query.energy = 1.0;

    const auto result = broad_phase.query_sphere(query);

    check(std::find(
              result.candidates.bone_joints.begin(),
              result.candidates.bone_joints.end(),
              child) != result.candidates.bone_joints.end(),
          "broad phase should index the full capsule AABB rather than only the joint center line");
}

void test_tetrahedral_volume_constraint_restores_volume() {
    Body body;

    const auto p0 = body.add_particle({0.0, 0.0, 0.0});
    const auto p1 = body.add_particle({1.0, 0.0, 0.0});
    const auto p2 = body.add_particle({0.0, 1.0, 0.0});
    const auto p3 = body.add_particle({0.0, 0.0, 1.0});

    const auto tet =
        body.add_tetrahedral_constraint(p0, p1, p2, p3, 0.0);

    const double rest =
        body.tetrahedral_constraints()[tet].rest_volume;

    body.particles()[p3].position = {0.0, 0.0, 1.6};
    const double deformed = signed_tet_volume(
        body.particles()[p0].position,
        body.particles()[p1].position,
        body.particles()[p2].position,
        body.particles()[p3].position);

    check(std::abs(deformed - rest) > 0.05,
          "tet volume fixture should begin substantially deformed");

    body.step(1.0 / 60.0, no_gravity());

    const double corrected = signed_tet_volume(
        body.particles()[p0].position,
        body.particles()[p1].position,
        body.particles()[p2].position,
        body.particles()[p3].position);

    check(std::abs(corrected - rest) < 1e-6,
          "zero-compliance tetrahedral XPBD constraint should restore signed rest volume");
}

void test_voxel_lattice_generates_tetrahedral_cells() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 3;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.5;
    spec.include_diagonals = false;
    spec.include_tetrahedra = true;

    const auto lattice = sarx::build_voxel_lattice(spec);

    check(lattice.body.tetrahedral_constraints().size() == 48,
          "3x3x3 lattice should generate six tetrahedra for each of eight cells");

    for (const auto& tet : lattice.body.tetrahedral_constraints()) {
        check(std::abs(tet.rest_volume) > 1e-12,
              "generated tetrahedra must have non-zero signed rest volume");
    }
}


void test_cut_releases_tetrahedral_connectivity() {
    Body body;
    DamageSystem damage;

    const auto p0 = body.add_particle({0.0, 0.0, 0.0});
    const auto p1 = body.add_particle({1.0, 0.0, 0.0});
    const auto p2 = body.add_particle({0.0, 1.0, 0.0});
    const auto p3 = body.add_particle({0.0, 0.0, 1.0});

    const auto tet =
        body.add_tetrahedral_constraint(p0, p1, p2, p3, 0.0, 1.0);

    check(body.islands().size() == 1,
          "an active tetrahedral volume constraint should keep its particles in one physical island");

    CapsuleDamage blade;
    blade.a = {0.1, 0.1, -1.0};
    blade.b = {0.1, 0.1, 1.0};
    blade.radius = 0.02;
    blade.energy = 1.2;
    blade.mode = DamageMode::Cut;
    blade.event_id = 900;

    sarx::DamageBroadPhase broad_phase;
    broad_phase.rebuild(body, 0.25);
    const auto query = broad_phase.query_capsule(blade);

    check(std::find(
              query.candidates.tetrahedral.begin(),
              query.candidates.tetrahedral.end(),
              tet) != query.candidates.tetrahedral.end(),
          "broad phase should return an intersectable tetrahedral cell");

    const auto report =
        damage.apply_capsule(body, blade, query.candidates);

    check(!body.tetrahedral_constraints()[tet].active,
          "cut centerline entering a tetrahedron should deactivate the volume constraint");
    check(body.islands().size() == 4,
          "after the only tetrahedral connection is cut, its four particles should become independent islands");

    bool found_tet_break = false;
    for (const auto& event : report.events) {
        if (event.target_kind == sarx::DamageTargetKind::TetrahedralConstraint
            && event.target_id == tet
            && event.broke) {
            found_tet_break = true;
            break;
        }
    }
    check(found_tet_break,
          "tetrahedral topology failure should be explicit in the fracture event stream");
}


void test_anatomical_region_shapes_and_priority() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 5;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.5;
    spec.include_diagonals = false;
    spec.include_tetrahedra = false;
    spec.default_material = 1;

    sarx::MaterialRegion muscle;
    muscle.shape = sarx::RegionShape::Capsule;
    muscle.a = {0.0, 0.5, 0.5};
    muscle.b = {2.0, 0.5, 0.5};
    muscle.radius = 0.30;
    muscle.material = 30;
    muscle.priority = 5;
    muscle.fiber_direction = {1.0, 0.0, 0.0};

    sarx::MaterialRegion bone;
    bone.shape = sarx::RegionShape::Sphere;
    bone.center = {1.0, 0.5, 0.5};
    bone.radius = 0.26;
    bone.material = 31;
    bone.priority = 10;

    const auto lattice =
        sarx::build_voxel_lattice(spec, {muscle, bone});

    const auto center = lattice.particle(2, 1, 1);
    const auto muscle_node = lattice.particle(1, 1, 1);

    check(lattice.particle_materials[center] == 31,
          "higher-priority spherical bone region should override enclosing muscle capsule");
    check(lattice.particle_materials[muscle_node] == 30,
          "capsule-shaped muscle should assign material away from the bone core");
    check(sarx::nearly_equal(
              lattice.particle_fibers[muscle_node],
              {1.0, 0.0, 0.0}),
          "anatomical muscle region should assign its rest-space fiber direction");

    const auto left = lattice.particle(0, 1, 1);
    bool found_muscle_link = false;
    for (const auto& constraint : lattice.body.structural_constraints()) {
        const bool matches =
            (constraint.a == left && constraint.b == muscle_node)
            || (constraint.a == muscle_node && constraint.b == left);
        if (!matches) continue;

        found_muscle_link = true;
        check(constraint.material == 30,
              "constraint midpoint inside muscle capsule should inherit muscle material");
        check(sarx::nearly_equal(
                  constraint.material_fiber_rest,
                  {1.0, 0.0, 0.0}),
              "constraint should retain region-local anatomical fiber orientation");
        break;
    }
    check(found_muscle_link,
          "expected axial muscle constraint should exist in generated lattice");
}

void test_box_region_backwards_compatibility() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 2;
    spec.ny = 2;
    spec.nz = 2;
    spec.spacing = 1.0;
    spec.include_tetrahedra = false;

    sarx::MaterialRegion box;
    box.min = {0.5, -1.0, -1.0};
    box.max = {2.0, 2.0, 2.0};
    box.material = 44;
    box.priority = 1;

    const auto lattice = sarx::build_voxel_lattice(spec, {box});

    check(lattice.particle_materials[lattice.particle(1, 0, 0)] == 44,
          "default region shape should remain box for V0.4A compatibility");
    check(lattice.particle_materials[lattice.particle(0, 0, 0)]
              == sarx::kDefaultMaterial,
          "box region should not leak outside its bounds");
}


void test_adaptive_damage_domain_is_local() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 12;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.25;
    spec.include_diagonals = false;
    spec.include_tetrahedra = true;

    const auto lattice = sarx::build_voxel_lattice(spec);

    const auto domain = sarx::select_damage_domain(
        lattice.body,
        {0.5, 0.25, 0.25},
        0.30);

    check(!domain.particles.empty(),
          "adaptive damage domain should include nearby particles");
    check(!domain.tetrahedral.empty(),
          "adaptive damage domain should include nearby volume cells");
    check(domain.particles.size() < lattice.body.particles().size(),
          "localized damage domain should not awaken the entire body");
    check(domain.tetrahedral.size()
              < lattice.body.tetrahedral_constraints().size(),
          "localized damage domain should reject distant tetrahedral cells");

    sarx::WoundDescriptor wound;
    wound.center = {0.5, 0.25, 0.25};
    wound.radius = 0.10;

    const auto wound_domain =
        sarx::select_damage_domain(lattice.body, wound, 0.20);

    check(std::abs(wound_domain.radius - 0.30) < 1e-12,
          "wound domain should expand persistent wound radius by requested halo");
    check(wound_domain.particles == domain.particles,
          "equivalent wound+halo and explicit sphere should select identical particles");
}

void test_body_soa_snapshot_matches_authoritative_state() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 3;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.5;
    spec.include_diagonals = false;
    spec.include_tetrahedra = true;

    auto lattice = sarx::build_voxel_lattice(spec);

    const auto root = sarx::embed_bone(
        lattice,
        sarx::kNoParent,
        {0.5, 0.5, 0.5},
        0.51,
        1e-8,
        1.0,
        2,
        1.0,
        3,
        0.15);

    check(!root.attachments.empty(),
          "SoA fixture should contain embedded attachments");

    lattice.body.particles()[0].velocity = {1.0, 2.0, 3.0};
    lattice.body.damage_structural(0, 0.25);
    lattice.body.damage_tetrahedral(0, 0.40);

    const auto soa = sarx::snapshot_body_soa(lattice.body);

    check(soa.particles.px.size() == lattice.body.particles().size(),
          "SoA particle count should match authoritative body");
    check(soa.structural.a.size()
              == lattice.body.structural_constraints().size(),
          "SoA structural count should match authoritative body");
    check(soa.tetrahedral.a.size()
              == lattice.body.tetrahedral_constraints().size(),
          "SoA tetrahedral count should match authoritative body");
    check(soa.bones.parent.size() == lattice.body.bones().size(),
          "SoA bone count should match authoritative body");
    check(soa.attachments.particle.size()
              == lattice.body.attachments().size(),
          "SoA attachment count should match authoritative body");

    check(std::abs(soa.particles.vx[0] - 1.0) < 1e-12
              && std::abs(soa.particles.vy[0] - 2.0) < 1e-12
              && std::abs(soa.particles.vz[0] - 3.0) < 1e-12,
          "SoA snapshot should preserve particle velocity components");
    check(std::abs(soa.structural.damage[0] - 0.25) < 1e-12,
          "SoA snapshot should preserve structural damage");
    check(std::abs(soa.tetrahedral.damage[0] - 0.40) < 1e-12,
          "SoA snapshot should preserve tetrahedral damage");
    check(std::abs(soa.bones.joint_radius[root.bone] - 0.15) < 1e-12,
          "SoA snapshot should preserve joint capsule radius");
}


void test_restricted_solver_matches_full_on_selected_fixture() {
    Body full;
    Body restricted;

    auto build_fixture = [](Body& body) {
        for (int chain = 0; chain < 8; ++chain) {
            const double base = static_cast<double>(chain) * 10.0;
            const auto a = body.add_particle({base + 0.0, 0.0, 0.0});
            const auto b = body.add_particle({base + 1.0, 0.0, 0.0});
            const auto c = body.add_particle({base + 2.0, 0.0, 0.0});
            body.add_structural_constraint(a, b, 0.0);
            body.add_structural_constraint(b, c, 0.0);
        }
    };

    build_fixture(full);
    restricted = full;

    full.particles()[1].position = {1.0, 0.8, 0.0};
    restricted.particles()[1].position = {1.0, 0.8, 0.0};

    StepConfig cfg = no_gravity();
    cfg.substeps = 2;
    cfg.solver_iterations = 10;

    full.step(1.0 / 60.0, cfg);

    sarx::SolverDomain domain;
    domain.particles = {0, 1, 2};
    domain.structural = {0, 1};

    const auto stats =
        restricted.step_restricted(1.0 / 60.0, domain, cfg);

    for (std::size_t i = 0; i < 3; ++i) {
        check(sarx::nearly_equal(
                  restricted.particles()[i].position,
                  full.particles()[i].position,
                  1e-10),
              "restricted solver should match full solver inside an equivalent isolated active fixture");
        check(sarx::nearly_equal(
                  restricted.particles()[i].velocity,
                  full.particles()[i].velocity,
                  1e-10),
              "restricted solver should match full solver velocity inside active fixture");
    }

    for (std::size_t i = 3; i < restricted.particles().size(); ++i) {
        check(sarx::nearly_equal(
                  restricted.particles()[i].position,
                  Body(full).particles()[i].position,
                  1e-10),
              "inactive restricted particles should remain unchanged");
    }

    check(stats.active_particles == 3,
          "restricted step should report only selected active particles");
    check(stats.structural_constraints == 2,
          "restricted step should report only selected structural constraints");

    const std::size_t full_visits =
        static_cast<std::size_t>(cfg.substeps)
        * static_cast<std::size_t>(cfg.solver_iterations)
        * full.structural_constraints().size();

    check(stats.solver_constraint_visits < full_visits / 4,
          "localized restricted solve should visit far fewer constraints than full-body solve");
}

void test_restricted_solver_uses_frozen_boundary_anchors() {
    Body body;

    const auto left = body.add_particle({0.0, 0.0, 0.0});
    const auto middle = body.add_particle({1.0, 0.0, 0.0});
    const auto right = body.add_particle({2.0, 0.0, 0.0});

    const auto c0 = body.add_structural_constraint(left, middle, 0.0);
    const auto c1 = body.add_structural_constraint(middle, right, 0.0);

    body.particles()[middle].position = {1.0, 1.0, 0.0};

    const Vec3 left_before = body.particles()[left].position;
    const Vec3 right_before = body.particles()[right].position;
    const Vec3 middle_before = body.particles()[middle].position;

    sarx::SolverDomain domain;
    domain.particles = {middle};
    domain.structural = {c0, c1};

    const auto stats =
        body.step_restricted(1.0 / 60.0, domain, no_gravity());

    check(sarx::nearly_equal(body.particles()[left].position, left_before),
          "inactive left boundary particle should remain frozen");
    check(sarx::nearly_equal(body.particles()[right].position, right_before),
          "inactive right boundary particle should remain frozen");
    check(body.particles()[middle].position.y < middle_before.y,
          "active particle should be corrected against frozen boundary constraints");
    check(stats.active_particles == 1,
          "boundary fixture should advance only one active particle");
}

void test_adaptive_domain_tracker_incremental_union() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 16;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.25;
    spec.include_diagonals = false;
    spec.include_tetrahedra = true;

    auto lattice = sarx::build_voxel_lattice(spec);

    sarx::AdaptiveDomainTracker tracker;
    tracker.reset(lattice.body);

    sarx::WoundDescriptor left;
    left.event_id = 1000;
    left.center = {0.5, 0.25, 0.25};
    left.radius = 0.10;

    sarx::WoundDescriptor right;
    right.event_id = 1001;
    right.center = {3.0, 0.25, 0.25};
    right.radius = 0.10;

    tracker.upsert_wound(lattice.body, left, 0.20);
    const auto left_only = tracker.combined_solver_domain();

    tracker.upsert_wound(lattice.body, right, 0.20);
    const auto both = tracker.combined_solver_domain();

    check(tracker.active_wound_count() == 2,
          "tracker should retain two independent wound domains");
    check(both.particles.size() > left_only.particles.size(),
          "adding a distant wound should incrementally grow the active particle union");
    check(both.tetrahedral.size() > left_only.tetrahedral.size(),
          "adding a distant wound should grow active tetrahedral work");

    check(tracker.remove_wound(left.event_id),
          "removing an existing wound should report success");

    const auto right_only = tracker.combined_solver_domain();
    check(tracker.active_wound_count() == 1,
          "removing one wound should decrement active wound count");
    check(right_only.particles.size() < both.particles.size(),
          "removing a wound should shrink the combined active domain");

    check(!tracker.remove_wound(999999),
          "removing an unknown wound should be a no-op");
}

void test_adaptive_domain_tracker_upsert_and_refit() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 10;
    spec.ny = 2;
    spec.nz = 2;
    spec.spacing = 0.25;
    spec.include_diagonals = false;
    spec.include_tetrahedra = true;

    auto lattice = sarx::build_voxel_lattice(spec);

    sarx::AdaptiveDomainTracker tracker;
    tracker.reset(lattice.body);

    sarx::WoundDescriptor wound;
    wound.event_id = 2000;
    wound.center = {0.25, 0.0, 0.0};
    wound.radius = 0.10;

    tracker.upsert_wound(lattice.body, wound, 0.20);
    const auto before = tracker.combined_solver_domain();

    wound.center = {1.75, 0.0, 0.0};
    tracker.upsert_wound(lattice.body, wound, 0.20);
    const auto moved = tracker.combined_solver_domain();

    check(tracker.active_wound_count() == 1,
          "upserting the same event ID should replace rather than duplicate a wound");
    check(before.particles != moved.particles,
          "moving an existing wound should replace its active-domain contribution");

    for (auto& particle : lattice.body.particles()) {
        particle.position += Vec3{100.0, 0.0, 0.0};
    }

    tracker.refit(lattice.body);
    const auto refit = tracker.combined_solver_domain();

    check(refit.particles.empty(),
          "refit should update active particle membership after large deformation");
    check(refit.structural.empty()
              && refit.tetrahedral.empty()
              && refit.attachments.empty(),
          "refit should remove primitives that no longer overlap stored wound domains");
}


void check_body_soa_particle_parity(
    const Body& body,
    const sarx::BodySoA& soa,
    double eps,
    const std::string& context) {

    check(soa.particles.px.size() == body.particles().size(),
          context + ": particle count mismatch");

    const std::size_t n =
        std::min(soa.particles.px.size(), body.particles().size());

    for (std::size_t i = 0; i < n; ++i) {
        const auto& p = body.particles()[i];
        check(std::abs(soa.particles.px[i] - p.position.x) <= eps
                  && std::abs(soa.particles.py[i] - p.position.y) <= eps
                  && std::abs(soa.particles.pz[i] - p.position.z) <= eps,
              context + ": position parity failure");
        check(std::abs(soa.particles.vx[i] - p.velocity.x) <= eps
                  && std::abs(soa.particles.vy[i] - p.velocity.y) <= eps
                  && std::abs(soa.particles.vz[i] - p.velocity.z) <= eps,
              context + ": velocity parity failure");
    }
}

void test_soa_full_solver_matches_body() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 3;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.4;
    spec.include_diagonals = true;
    spec.include_tetrahedra = true;
    spec.structural_compliance = 1e-6;
    spec.volume_compliance = 1e-7;

    auto lattice = sarx::build_voxel_lattice(spec);

    const auto root = sarx::embed_bone(
        lattice,
        sarx::kNoParent,
        {0.4, 0.4, 0.4},
        0.5,
        1e-7);

    Body cpu = lattice.body;
    auto soa = sarx::snapshot_body_soa(lattice.body);

    cpu.particles()[0].velocity = {0.5, 0.25, -0.1};
    cpu.particles()[13].position += Vec3{0.05, 0.08, -0.03};

    soa.particles.vx[0] = 0.5;
    soa.particles.vy[0] = 0.25;
    soa.particles.vz[0] = -0.1;
    soa.particles.px[13] += 0.05;
    soa.particles.py[13] += 0.08;
    soa.particles.pz[13] -= 0.03;

    check(!root.attachments.empty(),
          "SoA full parity fixture should include animation attachments");

    StepConfig cfg;
    cfg.substeps = 3;
    cfg.solver_iterations = 12;
    cfg.gravity = {0.0, -2.0, 0.0};

    cpu.step(1.0 / 60.0, cfg);
    const auto stats = sarx::step_soa(soa, 1.0 / 60.0, cfg);

    check_body_soa_particle_parity(
        cpu,
        soa,
        1e-9,
        "full CPU/SoA solver");

    check(stats.active_particles == cpu.particles().size(),
          "full SoA solver should report all particles active");
    check(stats.tetrahedral_constraints
              == cpu.tetrahedral_constraints().size(),
          "full SoA solver should visit all tetrahedral constraints");
}

void test_soa_restricted_solver_matches_body() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 8;
    spec.ny = 3;
    spec.nz = 3;
    spec.spacing = 0.25;
    spec.include_diagonals = false;
    spec.include_tetrahedra = true;
    spec.structural_compliance = 1e-6;
    spec.volume_compliance = 1e-7;

    auto lattice = sarx::build_voxel_lattice(spec);

    sarx::WoundDescriptor wound;
    wound.event_id = 3000;
    wound.center = {0.50, 0.25, 0.25};
    wound.radius = 0.15;

    const auto adaptive =
        sarx::select_damage_domain(lattice.body, wound, 0.25);
    const auto domain = sarx::solver_domain(adaptive);

    Body cpu = lattice.body;
    auto soa = sarx::snapshot_body_soa(lattice.body);

    check(!domain.particles.empty()
              && !domain.structural.empty()
              && !domain.tetrahedral.empty(),
          "restricted CPU/SoA parity fixture should have an active local domain");

    const auto moved_particle = domain.particles.front();
    cpu.particles()[moved_particle].position += Vec3{0.02, 0.04, 0.01};
    soa.particles.px[moved_particle] += 0.02;
    soa.particles.py[moved_particle] += 0.04;
    soa.particles.pz[moved_particle] += 0.01;

    StepConfig cfg = no_gravity();
    cfg.substeps = 2;
    cfg.solver_iterations = 10;

    const auto cpu_stats =
        cpu.step_restricted(1.0 / 60.0, domain, cfg);
    const auto soa_stats =
        sarx::step_soa_restricted(soa, 1.0 / 60.0, domain, cfg);

    check_body_soa_particle_parity(
        cpu,
        soa,
        1e-9,
        "restricted CPU/SoA solver");

    check(cpu_stats.active_particles == soa_stats.active_particles
              && cpu_stats.structural_constraints
                  == soa_stats.structural_constraints
              && cpu_stats.tetrahedral_constraints
                  == soa_stats.tetrahedral_constraints
              && cpu_stats.attachment_constraints
                  == soa_stats.attachment_constraints
              && cpu_stats.solver_constraint_visits
                  == soa_stats.solver_constraint_visits,
          "CPU and SoA restricted solvers should report identical work accounting");
}


void test_tetrahedral_cut_honors_capsule_radius() {
    auto build_tet = []() {
        Body body;
        const auto p0 = body.add_particle({0.0, 0.0, 0.0});
        const auto p1 = body.add_particle({1.0, 0.0, 0.0});
        const auto p2 = body.add_particle({0.0, 1.0, 0.0});
        const auto p3 = body.add_particle({0.0, 0.0, 1.0});
        body.add_tetrahedral_constraint(
            p0, p1, p2, p3, 0.0, 1.0);
        return body;
    };

    Body grazing = build_tet();
    Body miss = build_tet();

    DamageSystem grazing_damage;
    DamageSystem miss_damage;

    CapsuleDamage blade;
    blade.a = {0.20, -0.05, -0.20};
    blade.b = {0.20, -0.05, 0.40};
    blade.radius = 0.10;
    blade.energy = 2.5;
    blade.mode = DamageMode::Cut;

    const auto grazing_report =
        grazing_damage.apply_capsule(grazing, blade);

    check(!grazing.tetrahedral_constraints()[0].active,
          "finite-radius blade should damage a tetrahedron even when its centerline only grazes the surface");
    check(grazing_report.broken_count() == 1,
          "grazing finite-radius tet cut should emit one terminal fracture");

    blade.radius = 0.02;
    const auto miss_report =
        miss_damage.apply_capsule(miss, blade);

    check(miss.tetrahedral_constraints()[0].active,
          "same blade centerline should miss the tetrahedron when radius is below the surface distance");
    check(miss_report.events.empty(),
          "out-of-radius tetrahedron should not receive a fracture event");
}


void test_adaptive_domain_closes_over_detached_free_island() {
    Body body;

    const auto root_bone =
        body.add_bone(sarx::kNoParent, {0.0, 0.0, 0.0});

    const auto left0 = body.add_particle({0.0, 0.0, 0.0});
    const auto left1 = body.add_particle({1.0, 0.0, 0.0});
    const auto right0 = body.add_particle({2.0, 0.0, 0.0});
    const auto right1 = body.add_particle({3.0, 0.0, 0.0});
    const auto right2 = body.add_particle({4.0, 0.0, 0.0});

    body.add_structural_constraint(left0, left1);
    const auto bridge =
        body.add_structural_constraint(left1, right0);
    body.add_structural_constraint(right0, right1);
    body.add_structural_constraint(right1, right2);

    body.add_attachment(
        left0,
        root_bone,
        {},
        1e-8);

    body.break_structural(bridge);

    sarx::AdaptiveDamageDomain seed;
    seed.particles = {right0};

    const auto closed =
        sarx::close_over_free_islands(body, seed);

    check(closed.particles.size() == 3,
          "touching one particle of a detached free island should awaken the entire island");
    check(std::find(
              closed.particles.begin(),
              closed.particles.end(),
              right2) != closed.particles.end(),
          "free-island closure should include distal particles beyond the wound radius");
    check(std::find(
              closed.particles.begin(),
              closed.particles.end(),
              left0) == closed.particles.end(),
          "free-island closure should not promote the rig-authoritative body side");

    check(closed.structural.size() == 2,
          "free-island closure should include all surviving internal structural constraints");
}


void test_plane_cut_cleanly_separates_generated_volume() {
    sarx::VoxelLatticeSpec spec;
    spec.nx = 4;
    spec.ny = 2;
    spec.nz = 2;
    spec.spacing = 0.5;
    spec.include_diagonals = true;
    spec.include_tetrahedra = true;
    spec.structural_break_damage = 1.0;
    spec.volume_break_damage = 1.0;

    auto lattice = sarx::build_voxel_lattice(spec);
    Body replayed = lattice.body;

    check(lattice.body.islands().size() == 1,
          "plane-cut fixture should start as one connected volume");

    DamageSystem damage;

    sarx::PlaneCutDamage cut;
    cut.center = {0.75, 0.25, 0.25};
    cut.normal = {1.0, 0.0, 0.0};
    cut.radius = 0.50;
    cut.energy = 2.0;
    cut.event_id = 4000;

    const auto report =
        damage.apply_plane_cut(lattice.body, cut);

    const auto islands = lattice.body.islands();
    check(islands.size() == 2,
          "bounded planar cut between lattice columns should produce exactly two coherent islands");
    check(report.broken_count() > 0,
          "planar cut should emit terminal fracture events");

    std::size_t left_count = 0;
    std::size_t right_count = 0;
    for (const auto& island : islands) {
        double mean_x = 0.0;
        for (const auto id : island.particles) {
            mean_x += lattice.body.particles()[id].position.x;
        }
        mean_x /= static_cast<double>(island.particles.size());

        if (mean_x < cut.center.x) {
            left_count = island.particles.size();
        } else {
            right_count = island.particles.size();
        }
    }

    check(left_count == 8 && right_count == 8,
          "clean planar cut should keep every particle assigned to one of the two main halves");

    DamageSystem replica;
    const auto replay_reports =
        replica.replay(replayed, damage.history());

    check(replay_reports.size() == 1
              && replay_reports[0].event_id == 4000,
          "planar cut should participate in deterministic damage replay");
    check(replayed.islands().size() == 2,
          "replayed planar cut should reproduce the same topology split");
}


void test_humanoid_fixture_is_connected_and_shaped() {
    const auto fixture = sarx::build_humanoid_fixture();

    check(fixture.body.particles().size() > 250,
          "humanoid fixture should generate a substantial sparse particle volume");
    check(fixture.body.particles().size() < 2000,
          "humanoid fixture should remain sparse rather than fill its bounding box");

    check(fixture.body.structural_constraints().size()
              > fixture.body.particles().size(),
          "humanoid fixture should contain a connected structural network");

    check(!fixture.body.tetrahedral_constraints().empty(),
          "humanoid fixture should contain explicit volumetric tetrahedra");

    const auto islands = fixture.body.islands();
    check(islands.size() == 1,
          "intact generated humanoid should begin as one physical island");

    check(fixture.bones.right_shoulder != sarx::kNoParent
              && fixture.bones.right_elbow != sarx::kNoParent
              && fixture.bones.right_hand != sarx::kNoParent,
          "humanoid fixture should expose a complete right-arm rig chain");
}

void test_humanoid_shoulder_cut_detaches_arm_cleanly() {
    auto fixture = sarx::build_humanoid_fixture();
    DamageSystem damage;

    MaterialResponse tissue;
    tissue.cut_resistance = 0.75;
    tissue.blunt_resistance = 1.0;
    damage.materials().set(1, tissue);

    sarx::PlaneCutDamage cut;
    cut.center = fixture.right_shoulder_cut_center;
    cut.normal = fixture.right_shoulder_cut_normal;
    cut.radius = fixture.right_shoulder_cut_radius;
    cut.energy = 3.0;
    cut.event_id = 5000;

    const auto report =
        damage.apply_plane_cut(fixture.body, cut);

    check(report.broken_count() > 0,
          "humanoid shoulder cut should break physical/rig topology");

    const auto islands = fixture.body.islands();

    check(islands.size() == 2,
          "humanoid shoulder cut should create exactly body + detached arm islands");

    std::size_t smaller = std::numeric_limits<std::size_t>::max();
    std::size_t larger = 0;

    for (const auto& island : islands) {
        smaller = std::min(smaller, island.particles.size());
        larger = std::max(larger, island.particles.size());
    }

    check(smaller > 10,
          "detached humanoid arm should remain a coherent multi-particle volume");
    check(larger > smaller,
          "main humanoid body should remain the larger connected component");

    check(!fixture.body.bone_root_connected(
              fixture.bones.right_shoulder),
          "right arm rig chain should lose root authority after shoulder cut");

    check(damage.wounds().size() == 1,
          "one humanoid shoulder plane cut should create one wound descriptor");
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
    test_persistent_wound_descriptor();
    test_broad_phase_matches_full_scan();
    test_voxel_lattice_generation();
    test_voxel_lattice_diagonal_connectivity();
    test_automatic_bone_embedding();
    test_embedded_child_bone_uses_rig_parent();
    test_material_frame_follows_deformation();
    test_joint_capsule_extends_damage_geometry();
    test_joint_capsule_is_indexed_by_broad_phase();
    test_tetrahedral_volume_constraint_restores_volume();
    test_voxel_lattice_generates_tetrahedral_cells();
    test_cut_releases_tetrahedral_connectivity();
    test_anatomical_region_shapes_and_priority();
    test_box_region_backwards_compatibility();
    test_adaptive_damage_domain_is_local();
    test_body_soa_snapshot_matches_authoritative_state();
    test_restricted_solver_matches_full_on_selected_fixture();
    test_restricted_solver_uses_frozen_boundary_anchors();
    test_adaptive_domain_tracker_incremental_union();
    test_adaptive_domain_tracker_upsert_and_refit();
    test_soa_full_solver_matches_body();
    test_soa_restricted_solver_matches_body();
    test_tetrahedral_cut_honors_capsule_radius();
    test_adaptive_domain_closes_over_detached_free_island();
    test_plane_cut_cleanly_separates_generated_volume();
    test_humanoid_fixture_is_connected_and_shaped();
    test_humanoid_shoulder_cut_detaches_arm_cleanly();

    if (failures != 0) {
        std::cerr << failures << " SARX test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "SARX V0.4C execution tests passed.\n";
    return EXIT_SUCCESS;
}
