#include "sarx/animation_authority.hpp"

#include <algorithm>
#include <array>
#include <cmath>
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
    double replacement_blend) {

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

    std::vector<CharacterNodeLocalPose>
        output = base;

    for (auto& pose : output) {
        const auto authority_it =
            authority_by_name.find(
                pose.name);

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
