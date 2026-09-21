#include "sarx/body.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

using sarx::Body;
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

} // namespace

int main() {
    test_compliant_animation_target();
    test_progressive_structural_damage();
    test_severance_creates_free_dynamic_island();
    test_attachment_failure_removes_animation_authority();

    if (failures != 0) {
        std::cerr << failures << " SARX test(s) failed.\n";
        return EXIT_FAILURE;
    }

    std::cout << "SARX V0 tests passed.\n";
    return EXIT_SUCCESS;
}
