#pragma once

#include "sarx/action_capability.hpp"
#include "sarx/gltf_character.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sarx {

enum class AnimationAuthoritySource {
    BaseAnimation,
    ReplacementAnimation,
    Physics,
    Disabled
};

struct JointAuthorityAssignment {
    std::string joint;
    AnimationAuthoritySource source{
        AnimationAuthoritySource::BaseAnimation};
};

struct AnimationAuthorityPlan {
    std::vector<JointAuthorityAssignment> joints;

    [[nodiscard]] std::size_t count(
        AnimationAuthoritySource source) const;
};

[[nodiscard]] AnimationAuthorityPlan
build_action_authority_plan(
    const std::vector<CharacterJointInfo>& joints,
    const ActionCapability& replacement,
    const std::vector<std::string>& physics_joint_roots);

[[nodiscard]] const char* animation_authority_source_name(
    AnimationAuthoritySource source);

} // namespace sarx
