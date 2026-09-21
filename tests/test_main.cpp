#include "sarx/body.hpp"
#include "sarx/damage.hpp"
#include "sarx/broad_phase.hpp"
#include "sarx/volume.hpp"

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

    if (failures != 0) {
        std::cerr << failures << " SARX test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "SARX V0.4B mechanics tests passed.\n";
    return EXIT_SUCCESS;
}
