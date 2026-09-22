#include "sarx/gltf_character.hpp"
#include "sarx/voxel_character.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
    std::string character{"assets/quaternius/character.glb"};
    std::string animations{"assets/quaternius/animations.glb"};
    double fps{60.0};
    double voxel_size{0.045};
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string value = argv[i];
        if (value == "--character" && i + 1 < argc) {
            args.character = argv[++i];
        } else if (value == "--animations" && i + 1 < argc) {
            args.animations = argv[++i];
        } else if (value == "--fps" && i + 1 < argc) {
            args.fps = std::stod(argv[++i]);
        } else if (value == "--voxel-size" && i + 1 < argc) {
            args.voxel_size = std::stod(argv[++i]);
        } else if (value == "--help") {
            std::cout
                << "sarx_character_action_audit"
                << " [--character FILE]"
                << " [--animations FILE]"
                << " [--fps N]"
                << " [--voxel-size N]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete action audit argument");
        }
    }
    if (args.fps <= 0.0 || args.voxel_size <= 0.0) {
        throw std::invalid_argument(
            "action audit settings must be positive");
    }
    return args;
}

sarx::Vec3 region_centroid(
    const sarx::VoxelizedCharacter& voxels,
    const std::vector<sarx::Vec3>& centers,
    const std::string& region) {

    sarx::Vec3 result{};
    std::size_t count = 0;

    const auto& anatomy = voxels.voxels();

    for (std::size_t i = 0; i < anatomy.size(); ++i) {
        if (anatomy[i].anatomical_region != region) {
            continue;
        }
        result += centers[i];
        ++count;
    }

    if (count == 0) {
        throw std::runtime_error(
            "action audit region has no voxels: " + region);
    }

    return result / static_cast<double>(count);
}

struct SideMetrics {
    double peak_relative_speed{};
    double max_reach{};
    double initial_reach{};
    double max_relative_displacement{};
    double max_extension_time{};
};

SideMetrics audit_side(
    const sarx::GltfCharacter& character,
    std::size_t clip,
    const sarx::VoxelizedCharacter& voxels,
    const std::string& hand_region,
    const std::string& upperarm_region,
    double fps,
    double duration) {

    const int frames =
        std::max(
            2,
            static_cast<int>(
                std::ceil(duration * fps)) + 1);

    const double dt = 1.0 / fps;

    SideMetrics metrics;

    sarx::Vec3 initial_relative{};
    sarx::Vec3 previous_relative{};
    bool have_previous = false;

    for (int frame = 0; frame < frames; ++frame) {
        const double time =
            std::min(
                duration,
                static_cast<double>(frame) * dt);

        const auto centers =
            voxels.sample_centers(
                character,
                clip,
                time,
                false);

        const sarx::Vec3 hand =
            region_centroid(
                voxels,
                centers,
                hand_region);

        const sarx::Vec3 upperarm =
            region_centroid(
                voxels,
                centers,
                upperarm_region);

        const sarx::Vec3 pelvis =
            region_centroid(
                voxels,
                centers,
                "pelvis");

        const sarx::Vec3 relative =
            hand - pelvis;

        const double reach =
            sarx::length(
                hand - upperarm);

        if (frame == 0) {
            initial_relative = relative;
            previous_relative = relative;
            metrics.initial_reach = reach;
            metrics.max_reach = reach;
            metrics.max_extension_time = time;
            have_previous = true;
        } else {
            const double relative_speed =
                sarx::length(
                    relative - previous_relative)
                / std::max(dt, 1e-9);

            metrics.peak_relative_speed =
                std::max(
                    metrics.peak_relative_speed,
                    relative_speed);

            const double displacement =
                sarx::length(
                    relative - initial_relative);

            metrics.max_relative_displacement =
                std::max(
                    metrics.max_relative_displacement,
                    displacement);

            if (reach > metrics.max_reach) {
                metrics.max_reach = reach;
                metrics.max_extension_time = time;
            }

            previous_relative = relative;
        }

        if (!have_previous) {
            previous_relative = relative;
            have_previous = true;
        }
    }

    return metrics;
}

double effector_score(const SideMetrics& metrics) {
    const double reach_gain =
        std::max(
            0.0,
            metrics.max_reach
                - metrics.initial_reach);

    return metrics.peak_relative_speed
        + metrics.max_relative_displacement * 2.0
        + reach_gain * 3.0;
}

void audit_clip(
    const sarx::GltfCharacter& character,
    const std::string& clip_name,
    double fps,
    double voxel_size) {

    const std::size_t clip =
        character.find_animation(
            clip_name);

    const double duration =
        character.animation_duration(
            clip);

    sarx::VoxelizedCharacter voxels;
    voxels.build(
        character,
        clip,
        0.0,
        voxel_size);

    const auto left =
        audit_side(
            character,
            clip,
            voxels,
            "hand_l",
            "upperarm_l",
            fps,
            duration);

    const auto right =
        audit_side(
            character,
            clip,
            voxels,
            "hand_r",
            "upperarm_r",
            fps,
            duration);

    const double left_score =
        effector_score(left);

    const double right_score =
        effector_score(right);

    const bool right_primary =
        right_score >= left_score;

    const SideMetrics& primary =
        right_primary ? right : left;

    const double contact_phase =
        duration > 1e-9
        ? primary.max_extension_time / duration
        : 0.0;

    std::cout
        << "SARX action audit:"
        << " clip=" << character.animation_names()[clip]
        << " duration=" << duration
        << " primary_effector="
        << (right_primary ? "RightHand" : "LeftHand")
        << " contact_phase_estimate="
        << contact_phase
        << " left_peak_speed="
        << left.peak_relative_speed
        << " left_max_displacement="
        << left.max_relative_displacement
        << " left_reach_gain="
        << (left.max_reach - left.initial_reach)
        << " left_score="
        << left_score
        << " right_peak_speed="
        << right.peak_relative_speed
        << " right_max_displacement="
        << right.max_relative_displacement
        << " right_reach_gain="
        << (right.max_reach - right.initial_reach)
        << " right_score="
        << right_score
        << '\n';
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args =
            parse_args(argc, argv);

        sarx::GltfCharacter character;
        character.load(
            args.character,
            args.animations);

        audit_clip(
            character,
            "Punch_Jab",
            args.fps,
            args.voxel_size);

        audit_clip(
            character,
            "Punch_Cross",
            args.fps,
            args.voxel_size);

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_action_audit: "
            << error.what()
            << '\n';
        return EXIT_FAILURE;
    }
}
