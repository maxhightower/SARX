#pragma once

#include "sarx/volume.hpp"

#include <vector>

namespace sarx {

struct HumanoidSpec {
    double spacing{0.10};
    double particle_mass{0.08};

    MaterialId tissue_material{1};
    double structural_compliance{2e-6};
    double structural_break_damage{1.0};
    double volume_compliance{5e-7};
    double volume_break_damage{1.0};
    double attachment_compliance{1e-7};
    double attachment_break_damage{1.0};
    double joint_break_damage{1.0};
    double joint_radius{0.07};
};

struct HumanoidBones {
    BoneId pelvis{kNoParent};
    BoneId spine{kNoParent};
    BoneId chest{kNoParent};
    BoneId neck{kNoParent};
    BoneId head{kNoParent};

    BoneId left_shoulder{kNoParent};
    BoneId left_elbow{kNoParent};
    BoneId left_hand{kNoParent};

    BoneId right_shoulder{kNoParent};
    BoneId right_elbow{kNoParent};
    BoneId right_hand{kNoParent};

    BoneId left_hip{kNoParent};
    BoneId left_knee{kNoParent};
    BoneId left_ankle{kNoParent};

    BoneId right_hip{kNoParent};
    BoneId right_knee{kNoParent};
    BoneId right_ankle{kNoParent};
};

struct HumanoidFixture {
    Body body;
    HumanoidBones bones;

    Vec3 right_shoulder_cut_center{0.36, 1.34, 0.0};
    Vec3 right_shoulder_cut_normal{1.0, 0.0, 0.0};
    double right_shoulder_cut_radius{0.24};

    Vec3 right_shoulder_rest{0.46, 1.34, 0.0};
    Vec3 right_elbow_rest{0.70, 1.10, 0.0};
    Vec3 right_hand_rest{0.88, 0.86, 0.0};
};

[[nodiscard]] HumanoidFixture build_humanoid_fixture(
    const HumanoidSpec& spec = {});

} // namespace sarx
