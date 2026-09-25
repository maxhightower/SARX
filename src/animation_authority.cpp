#include "sarx/animation_authority.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace sarx {
namespace {

std::array<double, 4> normalized_quaternion(
    std::array<double, 4> q) {

    const double length =
        std::sqrt(
            q[0] * q[0]
            + q[1] * q[1]
            + q[2] * q[2]
            + q[3] * q[3]);

    if (length <= 1e-12) {
        return {0.0, 0.0, 0.0, 1.0};
    }

    for (double& value : q) {
        value /= length;
    }

    return q;
}

std::array<double, 4> slerp_quaternion(
    std::array<double, 4> a,
    std::array<double, 4> b,
    double t) {

    a = normalized_quaternion(a);
    b = normalized_quaternion(b);

    double dot =
        a[0] * b[0]
        + a[1] * b[1]
        + a[2] * b[2]
        + a[3] * b[3];

    if (dot < 0.0) {
        dot = -dot;
        for (double& value : b) {
            value = -value;
        }
    }

    if (dot > 0.9995) {
        std::array<double, 4> out{};

        for (std::size_t i = 0;
             i < 4;
             ++i) {
            out[i] =
                a[i]
                + (b[i] - a[i]) * t;
        }

        return normalized_quaternion(
            out);
    }

    const double theta0 =
        std::acos(
            std::clamp(
                dot,
                -1.0,
                1.0));

    const double theta =
        theta0 * t;

    const double sin_theta =
        std::sin(theta);

    const double sin_theta0 =
        std::sin(theta0);

    const double s0 =
        std::cos(theta)
        - dot
            * sin_theta
            / sin_theta0;

    const double s1 =
        sin_theta
        / sin_theta0;

    return {
        s0 * a[0] + s1 * b[0],
        s0 * a[1] + s1 * b[1],
        s0 * a[2] + s1 * b[2],
        s0 * a[3] + s1 * b[3]
    };
}

Vec3 lerp_vec3(
    const Vec3& a,
    const Vec3& b,
    double t) {

    return a * (1.0 - t)
        + b * t;
}


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

AuthoredAuthorityAnalysis
derive_authored_authority_root(
    const GltfCharacter& character,
    std::size_t animation,
    const std::string& effector_joint,
    double contribution_fraction,
    double sample_rate) {

    AuthoredAuthorityAnalysis analysis;
    analysis.effector_joint = effector_joint;

    const auto& joints =
        character.skin_joints();

    std::unordered_map<std::string, std::size_t> by_name;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        by_name.emplace(joints[i].name, i);
    }

    if (by_name.find(effector_joint) == by_name.end()) {
        throw std::runtime_error(
            "authored authority effector joint not in skeleton: "
            + effector_joint);
    }

    std::vector<std::string> chain;
    for (std::string current = effector_joint;
         !current.empty();) {
        const auto found = by_name.find(current);
        if (found == by_name.end()
            || chain.size() > joints.size()) {
            break;
        }
        chain.push_back(current);
        current = joints[found->second].parent;
    }

    const double duration =
        character.animation_duration(animation);

    const int samples =
        std::max(
            2,
            static_cast<int>(
                std::ceil(duration * sample_rate))
                + 1);

    const auto first =
        character.sample_node_local_poses(
            animation, 0.0, false);

    std::unordered_map<std::string, CharacterNodeLocalPose> first_by_name;
    for (const auto& pose : first) {
        first_by_name.emplace(pose.name, pose);
    }

    const Vec3 effector_start =
        character.node_world_position_with_local_poses(
            first, effector_joint);

    std::vector<double> contribution(
        chain.size(), 0.0);

    for (int s = 0; s < samples; ++s) {
        const double time =
            duration * static_cast<double>(s)
            / static_cast<double>(samples - 1);

        const auto poses =
            character.sample_node_local_poses(
                animation, time, false);

        const Vec3 effector =
            character.node_world_position_with_local_poses(
                poses, effector_joint);

        analysis.effector_travel =
            std::max(
                analysis.effector_travel,
                length(effector - effector_start));

        for (std::size_t c = 0; c < chain.size(); ++c) {
            auto frozen = poses;
            for (auto& pose : frozen) {
                if (pose.name == chain[c]) {
                    const auto found = first_by_name.find(pose.name);
                    if (found != first_by_name.end()) {
                        pose = found->second;
                    }
                }
            }

            const Vec3 frozen_effector =
                character.node_world_position_with_local_poses(
                    frozen, effector_joint);

            contribution[c] =
                std::max(
                    contribution[c],
                    length(frozen_effector - effector));
        }
    }

    analysis.contribution_threshold =
        analysis.effector_travel * contribution_fraction;

    analysis.authority_root = effector_joint;

    for (std::size_t c = 0; c < chain.size(); ++c) {
        analysis.chain.push_back(
            {chain[c], contribution[c]});

        if (contribution[c]
            >= analysis.contribution_threshold) {
            analysis.authority_root = chain[c];
        }
    }

    return analysis;
}

std::vector<std::string>
physics_joint_roots_from_voxels(
    const std::vector<CharacterJointInfo>& joints,
    const VoxelizedCharacter& voxels,
    const std::vector<DetachedVoxelComponent>& detached_components) {

    std::unordered_map<std::string, std::size_t> total;
    std::unordered_map<std::string, std::size_t> not_attached;
    std::unordered_map<std::string, std::size_t> in_component;

    for (const auto& voxel : voxels.voxels()) {
        const auto& joint = voxel.skin_binding.dominant_joint;
        ++total[joint];
        if (voxel.state != CharacterVoxelState::Attached) {
            ++not_attached[joint];
        }
    }

    for (const auto& component : detached_components) {
        for (const auto index : component.voxel_indices) {
            ++in_component[
                voxels.voxels()[index].skin_binding.dominant_joint];
        }
    }

    std::unordered_map<std::string, bool> physics;
    for (const auto& joint : joints) {
        const std::size_t count = total[joint.name];
        physics[joint.name] =
            count > 0
            && in_component[joint.name] > 0
            && not_attached[joint.name] * 2 > count;
    }

    std::unordered_map<std::string, std::size_t> by_name;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        by_name.emplace(joints[i].name, i);
    }

    std::vector<std::string> roots;
    for (const auto& joint : joints) {
        if (!physics[joint.name]) {
            continue;
        }

        // Skip if any ancestor is already physics-owned.
        bool nested = false;
        std::string parent = joint.parent;
        std::size_t guard = 0;
        while (!parent.empty() && guard++ <= joints.size()) {
            if (physics[parent]) {
                nested = true;
                break;
            }
            const auto found = by_name.find(parent);
            if (found == by_name.end()) {
                break;
            }
            parent = joints[found->second].parent;
        }

        if (!nested) {
            roots.push_back(joint.name);
        }
    }

    return roots;
}

AnimationAuthoritySource authority_source_for(
    const AnimationAuthorityPlan& plan,
    const std::string& joint) {

    const auto found =
        std::find_if(
            plan.joints.begin(),
            plan.joints.end(),
            [&](const JointAuthorityAssignment& assignment) {
                return assignment.joint == joint;
            });

    return found == plan.joints.end()
        ? AnimationAuthoritySource::Disabled
        : found->source;
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

std::vector<CharacterNodeLocalPose>
compose_action_local_poses(
    const std::vector<CharacterNodeLocalPose>& base,
    const std::vector<CharacterNodeLocalPose>& replacement,
    const AnimationAuthorityPlan& authority,
    double replacement_blend,
    const std::vector<CharacterNodeLocalPose>& physics_hold) {

    const double blend =
        std::clamp(
            replacement_blend,
            0.0,
            1.0);

    std::unordered_map<std::string, const CharacterNodeLocalPose*>
        replacement_by_name;

    replacement_by_name.reserve(
        replacement.size());

    for (const auto& pose
         : replacement) {
        replacement_by_name.emplace(
            pose.name,
            &pose);
    }

    std::unordered_map<std::string, AnimationAuthoritySource>
        authority_by_name;

    authority_by_name.reserve(
        authority.joints.size());

    for (const auto& assignment
         : authority.joints) {
        authority_by_name.emplace(
            assignment.joint,
            assignment.source);
    }

    std::unordered_map<std::string, const CharacterNodeLocalPose*>
        hold_by_name;

    for (const auto& pose : physics_hold) {
        hold_by_name.emplace(pose.name, &pose);
    }

    std::vector<CharacterNodeLocalPose>
        output = base;

    for (auto& pose : output) {
        const auto authority_it =
            authority_by_name.find(
                pose.name);

        if (authority_it != authority_by_name.end()
            && authority_it->second
                == AnimationAuthoritySource::Physics) {
            const auto hold_it = hold_by_name.find(pose.name);
            if (hold_it != hold_by_name.end()) {
                pose.translation = hold_it->second->translation;
                pose.rotation = hold_it->second->rotation;
                pose.scale = hold_it->second->scale;
            }
            continue;
        }

        if (authority_it
                == authority_by_name.end()
            || authority_it->second
                != AnimationAuthoritySource::ReplacementAnimation) {
            continue;
        }

        const auto replacement_it =
            replacement_by_name.find(
                pose.name);

        if (replacement_it
            == replacement_by_name.end()) {
            continue;
        }

        const CharacterNodeLocalPose& target =
            *replacement_it->second;

        pose.translation =
            lerp_vec3(
                pose.translation,
                target.translation,
                blend);

        pose.rotation =
            slerp_quaternion(
                pose.rotation,
                target.rotation,
                blend);

        pose.scale =
            lerp_vec3(
                pose.scale,
                target.scale,
                blend);
    }

    return output;
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
