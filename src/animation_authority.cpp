#include "sarx/animation_authority.hpp"

#include <algorithm>
#include <unordered_map>

namespace sarx {
namespace {

bool descends_from(
    const std::vector<CharacterJointInfo>& joints,
    const std::unordered_map<std::string, std::size_t>& by_name,
    std::size_t joint,
    const std::string& root) {

    std::string current =
        joints[joint].name;

    std::size_t guard = 0;

    while (!current.empty()) {
        if (current == root) {
            return true;
        }

        const auto found =
            by_name.find(current);

        if (found == by_name.end()) {
            return false;
        }

        if (++guard > joints.size()) {
            return false;
        }

        current =
            joints[found->second].parent;
    }

    return false;
}

bool descends_from_any(
    const std::vector<CharacterJointInfo>& joints,
    const std::unordered_map<std::string, std::size_t>& by_name,
    std::size_t joint,
    const std::vector<std::string>& roots) {

    return std::any_of(
        roots.begin(),
        roots.end(),
        [&](const std::string& root) {
            return descends_from(
                joints,
                by_name,
                joint,
                root);
        });
}

} // namespace

std::size_t AnimationAuthorityPlan::count(
    AnimationAuthoritySource source) const {

    return static_cast<std::size_t>(
        std::count_if(
            joints.begin(),
            joints.end(),
            [&](const JointAuthorityAssignment& assignment) {
                return assignment.source == source;
            }));
}

AnimationAuthorityPlan
build_action_authority_plan(
    const std::vector<CharacterJointInfo>& joints,
    const ActionCapability& replacement,
    const std::vector<std::string>& physics_joint_roots) {

    AnimationAuthorityPlan plan;

    std::unordered_map<std::string, std::size_t>
        by_name;

    by_name.reserve(
        joints.size());

    for (std::size_t i = 0;
         i < joints.size();
         ++i) {
        by_name.emplace(
            joints[i].name,
            i);
    }

    plan.joints.reserve(
        joints.size());

    for (std::size_t i = 0;
         i < joints.size();
         ++i) {

        JointAuthorityAssignment assignment;
        assignment.joint =
            joints[i].name;

        if (descends_from_any(
                joints,
                by_name,
                i,
                physics_joint_roots)) {

            assignment.source =
                AnimationAuthoritySource::Physics;
        } else if (
            descends_from_any(
                joints,
                by_name,
                i,
                replacement.authority_joint_roots)) {

            assignment.source =
                AnimationAuthoritySource::ReplacementAnimation;
        } else {
            assignment.source =
                AnimationAuthoritySource::BaseAnimation;
        }

        plan.joints.push_back(
            std::move(assignment));
    }

    return plan;
}

const char* animation_authority_source_name(
    AnimationAuthoritySource source) {

    switch (source) {
    case AnimationAuthoritySource::BaseAnimation:
        return "BaseAnimation";
    case AnimationAuthoritySource::ReplacementAnimation:
        return "ReplacementAnimation";
    case AnimationAuthoritySource::Physics:
        return "Physics";
    case AnimationAuthoritySource::Disabled:
        return "Disabled";
    }

    return "Unknown";
}

} // namespace sarx
