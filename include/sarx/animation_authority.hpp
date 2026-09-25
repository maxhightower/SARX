#pragma once

#include "sarx/action_capability.hpp"
#include "sarx/gltf_character.hpp"
#include "sarx/voxel_character.hpp"

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

struct AuthoredChainContribution {
    std::string joint;
    // Max effector world displacement caused by this joint's own authored
    // local channels (measured by freezing them at the clip's first frame).
    double effector_contribution{};
};

// Which part of the skeleton an authored action actually uses to move its
// striking effector. Derived from the clip, not from a hard-coded joint list.
struct AuthoredAuthorityAnalysis {
    std::string effector_joint;
    std::string authority_root;
    double effector_travel{};
    double contribution_threshold{};
    // Effector -> skeleton root order.
    std::vector<AuthoredChainContribution> chain;
};

// Walk the effector's ancestor chain; the authority root is the most
// proximal ancestor whose own authored channels move the effector by at
// least `contribution_fraction` of the effector's total authored travel.
// Its whole subtree (minus physics-owned branches) is the replacement region.
[[nodiscard]] AuthoredAuthorityAnalysis
derive_authored_authority_root(
    const GltfCharacter& character,
    std::size_t animation,
    const std::string& effector_joint,
    double contribution_fraction = 0.10,
    double sample_rate = 60.0);

// Physics roots derived from actual voxel topology: a skin joint is
// physics-owned when most of the voxels it dominates are no longer attached
// and at least one of them belongs to a detached component. Returned joints
// are the most proximal such joints (their parents are not physics-owned).
[[nodiscard]] std::vector<std::string>
physics_joint_roots_from_voxels(
    const std::vector<CharacterJointInfo>& joints,
    const VoxelizedCharacter& voxels,
    const std::vector<DetachedVoxelComponent>& detached_components);

[[nodiscard]] AnimationAuthorityPlan
build_action_authority_plan(
    const std::vector<CharacterJointInfo>& joints,
    const ActionCapability& replacement,
    const std::vector<std::string>& physics_joint_roots);

// Physics-owned joints never take base or replacement channels. They take
// `physics_hold` (the local pose captured when the branch detached) so
// surviving stump voxels bound to them stay rigid with their attached
// parent. Without a hold entry the joint keeps the base pose passed in, so
// callers must capture a hold at detachment.
[[nodiscard]] std::vector<CharacterNodeLocalPose>
compose_action_local_poses(
    const std::vector<CharacterNodeLocalPose>& base,
    const std::vector<CharacterNodeLocalPose>& replacement,
    const AnimationAuthorityPlan& authority,
    double replacement_blend,
    const std::vector<CharacterNodeLocalPose>& physics_hold = {});

[[nodiscard]] AnimationAuthoritySource authority_source_for(
    const AnimationAuthorityPlan& plan,
    const std::string& joint);

[[nodiscard]] const char* animation_authority_source_name(
    AnimationAuthoritySource source);

} // namespace sarx
