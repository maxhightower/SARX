#include "sarx/procedural_limp.hpp"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <stdexcept>

namespace sarx {
namespace {

constexpr double kPi =
    3.14159265358979323846;

std::string lower_copy(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) {
            return static_cast<char>(
                std::tolower(c));
        });
    return value;
}

Vec3 rotate_x_about(
    const Vec3& point,
    const Vec3& pivot,
    double angle) {

    const Vec3 local =
        point - pivot;

    const double c =
        std::cos(angle);

    const double s =
        std::sin(angle);

    return pivot + Vec3{
        local.x,
        local.y * c - local.z * s,
        local.y * s + local.z * c
    };
}

bool is_torso_region(
    const std::string& region) {

    return region == "pelvis"
        || region == "spine_01"
        || region == "spine_02"
        || region == "spine_03"
        || region == "neck_01"
        || region == "Head"
        || region == "upperarm_l"
        || region == "lowerarm_l"
        || region == "hand_l"
        || region == "upperarm_r"
        || region == "lowerarm_r"
        || region == "hand_r";
}

bool is_injured_leg_region(
    const std::string& region) {

    return region == "thigh_l"
        || region == "calf_l"
        || region == "foot_l";
}

void damp_injured_stride(
    double stride_scale,
    const std::vector<CharacterVoxel>& voxels,
    const std::vector<Vec3>& neutral_centers,
    std::vector<Vec3>& centers) {

    for (std::size_t i = 0;
         i < centers.size();
         ++i) {

        if (voxels[i].state
                != CharacterVoxelState::Attached
            || !is_injured_leg_region(
                voxels[i].anatomical_region)) {
            continue;
        }

        centers[i] =
            neutral_centers[i]
            + (centers[i]
               - neutral_centers[i])
                * stride_scale;
    }
}

void translate_torso(
    const std::vector<CharacterVoxel>& voxels,
    std::vector<Vec3>& centers,
    const Vec3& delta) {

    for (std::size_t i = 0;
         i < centers.size();
         ++i) {

        if (voxels[i].state
                != CharacterVoxelState::Attached
            || !is_torso_region(
                voxels[i].anatomical_region)) {
            continue;
        }

        centers[i] += delta;
    }
}

void rotate_region_about(
    const std::string& region,
    const Vec3& pivot,
    double angle,
    const std::vector<CharacterVoxel>& voxels,
    std::vector<Vec3>& centers) {

    for (std::size_t i = 0;
         i < centers.size();
         ++i) {

        if (voxels[i].state
                != CharacterVoxelState::Attached
            || voxels[i].anatomical_region
                != region) {
            continue;
        }

        centers[i] =
            rotate_x_about(
                centers[i],
                pivot,
                angle);
    }
}

void rotate_injured_leg_about_hip(
    const Vec3& hip,
    double angle,
    const std::vector<CharacterVoxel>& voxels,
    std::vector<Vec3>& centers) {

    for (std::size_t i = 0;
         i < centers.size();
         ++i) {

        if (voxels[i].state
                != CharacterVoxelState::Attached
            || (voxels[i].anatomical_region
                    != "thigh_l"
                && voxels[i].anatomical_region
                    != "calf_l"
                && voxels[i].anatomical_region
                    != "foot_l")) {
            continue;
        }

        centers[i] =
            rotate_x_about(
                centers[i],
                hip,
                angle);
    }
}

} // namespace

ProceduralLimpVariant
parse_procedural_limp_variant(
    const std::string& name) {

    const std::string lower =
        lower_copy(name);

    if (lower == "guarded") {
        return ProceduralLimpVariant::Guarded;
    }

    if (lower == "stiff"
        || lower == "stiff-leg"
        || lower == "stiff_leg") {
        return ProceduralLimpVariant::StiffLeg;
    }

    if (lower == "hop"
        || lower == "hop-step"
        || lower == "hop_step") {
        return ProceduralLimpVariant::HopStep;
    }

    throw std::invalid_argument(
        "unknown procedural limp variant: "
        + name);
}

const char*
procedural_limp_variant_name(
    ProceduralLimpVariant variant) {

    switch (variant) {
    case ProceduralLimpVariant::Guarded:
        return "guarded";
    case ProceduralLimpVariant::StiffLeg:
        return "stiff-leg";
    case ProceduralLimpVariant::HopStep:
        return "hop-step";
    }

    return "unknown";
}

ProceduralLimpMetrics
apply_procedural_limp(
    ProceduralLimpVariant variant,
    const ProceduralLimpInput& input,
    const std::vector<CharacterVoxel>& voxels,
    const std::vector<Vec3>& neutral_centers,
    std::vector<Vec3>& animated_centers) {

    if (voxels.size()
            != animated_centers.size()
        || neutral_centers.size()
            != animated_centers.size()) {

        throw std::invalid_argument(
            "procedural limp point count mismatch");
    }

    const double phase =
        input.cycle_phase
        - std::floor(
            input.cycle_phase);

    const double wave =
        std::sin(
            phase
            * 2.0
            * kPi);

    const double support =
        0.5
        * (1.0
           + std::cos(
                phase
                * 2.0
                * kPi));

    ProceduralLimpMetrics metrics;

    switch (variant) {
    case ProceduralLimpVariant::Guarded: {
        metrics.injured_stride_scale = 0.38;
        metrics.body_shift =
            input.voxel_size * 0.72;
        metrics.body_lift =
            input.voxel_size
            * (0.18 + 0.22 * support);
        metrics.knee_flexion_radians =
            0.12 + 0.08 * support;

        damp_injured_stride(
            metrics.injured_stride_scale,
            voxels,
            neutral_centers,
            animated_centers);

        rotate_region_about(
            "calf_l",
            input.injured_knee,
            metrics.knee_flexion_radians,
            voxels,
            animated_centers);

        translate_torso(
            voxels,
            animated_centers,
            {
                metrics.body_shift,
                metrics.body_lift,
                0.0
            });
        break;
    }

    case ProceduralLimpVariant::StiffLeg: {
        metrics.injured_stride_scale = 0.16;
        metrics.body_shift =
            input.voxel_size * 0.92;
        metrics.body_lift =
            input.voxel_size
            * (0.24 + 0.34 * support);
        metrics.knee_flexion_radians = 0.03;

        damp_injured_stride(
            metrics.injured_stride_scale,
            voxels,
            neutral_centers,
            animated_centers);

        // Treat thigh + calf almost as one rigid pendulum. This suppresses
        // ordinary knee flexion and creates a circumduction-like stiff-leg
        // gait rather than simply speeding up or slowing down Walk.
        rotate_injured_leg_about_hip(
            input.injured_hip,
            0.10 * wave,
            voxels,
            animated_centers);

        // Small counter-rotation keeps the residual calf bend minimal.
        rotate_region_about(
            "calf_l",
            input.injured_knee,
            metrics.knee_flexion_radians,
            voxels,
            animated_centers);

        translate_torso(
            voxels,
            animated_centers,
            {
                metrics.body_shift,
                metrics.body_lift,
                0.0
            });
        break;
    }

    case ProceduralLimpVariant::HopStep: {
        metrics.injured_stride_scale = 0.10;
        metrics.body_shift =
            input.voxel_size * 1.02;

        const double hop =
            std::max(
                0.0,
                std::sin(
                    phase
                    * 2.0
                    * kPi));

        metrics.body_lift =
            input.voxel_size
            * (0.30 + 1.45 * hop);

        metrics.knee_flexion_radians =
            0.72 + 0.18 * support;

        damp_injured_stride(
            metrics.injured_stride_scale,
            voxels,
            neutral_centers,
            animated_centers);

        rotate_injured_leg_about_hip(
            input.injured_hip,
            -0.18 + 0.06 * wave,
            voxels,
            animated_centers);

        rotate_region_about(
            "calf_l",
            input.injured_knee,
            metrics.knee_flexion_radians,
            voxels,
            animated_centers);

        translate_torso(
            voxels,
            animated_centers,
            {
                metrics.body_shift,
                metrics.body_lift,
                0.0
            });
        break;
    }
    }

    return metrics;
}

} // namespace sarx
