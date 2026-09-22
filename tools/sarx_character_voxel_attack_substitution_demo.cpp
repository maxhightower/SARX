#include "sarx/action_capability.hpp"
#include "sarx/action_substitution.hpp"
#include "sarx/animation_authority.hpp"
#include "sarx/character_render.hpp"
#include "sarx/detached_articulation.hpp"
#include "sarx/gltf_character.hpp"
#include "sarx/voxel_character.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
    std::string character{"assets/quaternius/character.glb"};
    std::string animations{"assets/quaternius/animations.glb"};
    std::string clip{"Punch_Jab"};

    std::string replacement_animations{
        "assets/cmu_combat/CMU_ElbowLeft.glb"};
    std::string replacement_clip{
        "CMU_ElbowLeft"};

    double replacement_start_seconds{0.0};

    std::filesystem::path output{
        "media/raw/v17b_hand_loss_to_elbow_frames"};

    int frames{120};
    int cut_frame{5};
    double fps{30.0};
    double voxel_size{0.045};

    bool require_damage{false};
    bool require_detachment{false};
    bool require_substitution{false};
    bool require_contact{false};
    bool require_authority{false};
    bool require_isolation{false};
};

Args parse_args(int argc, char** argv) {
    Args args;

    for (int i = 1; i < argc; ++i) {
        const std::string value = argv[i];

        if (value == "--character" && i + 1 < argc) {
            args.character = argv[++i];
        } else if (value == "--animations" && i + 1 < argc) {
            args.animations = argv[++i];
        } else if (value == "--clip" && i + 1 < argc) {
            args.clip = argv[++i];
        } else if (
            value == "--replacement-animations"
            && i + 1 < argc) {
            args.replacement_animations = argv[++i];
        } else if (
            value == "--replacement-clip"
            && i + 1 < argc) {
            args.replacement_clip = argv[++i];
        } else if (
            value == "--replacement-start"
            && i + 1 < argc) {
            args.replacement_start_seconds =
                std::stod(argv[++i]);
        } else if (
            value == "--output"
            && i + 1 < argc) {
            args.output = argv[++i];
        } else if (
            value == "--frames"
            && i + 1 < argc) {
            args.frames = std::stoi(argv[++i]);
        } else if (
            value == "--cut-frame"
            && i + 1 < argc) {
            args.cut_frame = std::stoi(argv[++i]);
        } else if (
            value == "--fps"
            && i + 1 < argc) {
            args.fps = std::stod(argv[++i]);
        } else if (
            value == "--voxel-size"
            && i + 1 < argc) {
            args.voxel_size = std::stod(argv[++i]);
        } else if (value == "--require-damage") {
            args.require_damage = true;
        } else if (value == "--require-detachment") {
            args.require_detachment = true;
        } else if (value == "--require-substitution") {
            args.require_substitution = true;
        } else if (value == "--require-contact") {
            args.require_contact = true;
        } else if (value == "--require-authority") {
            args.require_authority = true;
        } else if (value == "--require-isolation") {
            args.require_isolation = true;
        } else if (value == "--help") {
            std::cout
                << "sarx_character_voxel_attack_substitution_demo"
                << " [--replacement-animations FILE]"
                << " [--replacement-clip NAME]"
                << " [--replacement-start SECONDS]"
                << " [--output DIR]"
                << " [--frames N]"
                << " [--cut-frame N]"
                << " [--fps N]"
                << " [--voxel-size N]"
                << " [--require-damage]"
                << " [--require-detachment]"
                << " [--require-substitution]"
                << " [--require-contact]"
                << " [--require-authority]"
                << " [--require-isolation]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete attack substitution demo argument");
        }
    }

    if (args.frames <= 0
        || args.cut_frame < 1
        || args.cut_frame >= args.frames
        || args.fps <= 0.0
        || args.voxel_size <= 0.0
        || args.replacement_start_seconds < 0.0) {
        throw std::invalid_argument(
            "invalid attack substitution demo settings");
    }

    return args;
}

std::string frame_path(
    const std::filesystem::path& directory,
    int frame) {

    std::ostringstream name;
    name << "frame_"
         << std::setw(4)
         << std::setfill('0')
         << frame
         << ".ppm";

    return (directory / name.str()).string();
}

struct Bounds {
    sarx::Vec3 min{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    };

    sarx::Vec3 max{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()
    };
};

Bounds bounds_of(
    const sarx::CharacterMeshFrame& frame) {

    Bounds bounds;

    for (const auto& point : frame.positions) {
        bounds.min.x =
            std::min(bounds.min.x, point.x);
        bounds.min.y =
            std::min(bounds.min.y, point.y);
        bounds.min.z =
            std::min(bounds.min.z, point.z);

        bounds.max.x =
            std::max(bounds.max.x, point.x);
        bounds.max.y =
            std::max(bounds.max.y, point.y);
        bounds.max.z =
            std::max(bounds.max.z, point.z);
    }

    return bounds;
}

std::vector<std::uint8_t> used_vertices(
    const sarx::CharacterMeshFrame& frame) {

    std::vector<std::uint8_t> used(
        frame.positions.size(),
        0u);

    for (const auto index : frame.indices) {
        if (index < used.size()) {
            used[index] = 1u;
        }
    }

    return used;
}

sarx::Vec3 used_centroid(
    const sarx::CharacterMeshFrame& frame) {

    const auto used =
        used_vertices(frame);

    sarx::Vec3 center{};
    std::size_t count = 0;

    for (std::size_t i = 0;
         i < used.size();
         ++i) {

        if (!used[i]) {
            continue;
        }

        center += frame.positions[i];
        ++count;
    }

    if (count == 0) {
        throw std::runtime_error(
            "branch owns no mesh vertices");
    }

    return center
        / static_cast<double>(count);
}

sarx::Vec3 nearest_anchor(
    const sarx::CharacterMeshFrame& body,
    const sarx::CharacterMeshFrame& detached) {

    const auto body_used =
        used_vertices(body);

    const auto detached_used =
        used_vertices(detached);

    double best =
        std::numeric_limits<double>::infinity();

    sarx::Vec3 body_point{};
    sarx::Vec3 detached_point{};

    for (std::size_t i = 0;
         i < body_used.size();
         ++i) {

        if (!body_used[i]) {
            continue;
        }

        for (std::size_t j = 0;
             j < detached_used.size();
             ++j) {

            if (!detached_used[j]) {
                continue;
            }

            const double distance =
                sarx::length_squared(
                    body.positions[i]
                    - detached.positions[j]);

            if (distance < best) {
                best = distance;
                body_point =
                    body.positions[i];
                detached_point =
                    detached.positions[j];
            }
        }
    }

    if (!std::isfinite(best)) {
        throw std::runtime_error(
            "could not infer punch wrist anchor");
    }

    return (body_point + detached_point)
        * 0.5;
}

std::pair<std::size_t, std::size_t>
farthest_pair(
    const std::vector<sarx::Vec3>& points) {

    if (points.size() < 2) {
        throw std::runtime_error(
            "detached hand needs two or more voxels");
    }

    std::size_t a = 0;
    std::size_t b = 1;
    double best = -1.0;

    for (std::size_t i = 0;
         i < points.size();
         ++i) {

        for (std::size_t j = i + 1;
             j < points.size();
             ++j) {

            const double distance =
                sarx::length_squared(
                    points[j] - points[i]);

            if (distance > best) {
                best = distance;
                a = i;
                b = j;
            }
        }
    }

    return {a, b};
}

struct DetachedHand {
    sarx::DetachedVoxelComponent component;
    std::vector<sarx::Vec3> rest_centers;
    sarx::DetachedArticulatedChain articulation;
};

std::vector<sarx::Vec3>
detached_world_centers(
    const DetachedHand& hand) {

    std::vector<sarx::Vec3> centers;

    centers.reserve(
        hand.rest_centers.size());

    for (const auto& point
         : hand.rest_centers) {

        centers.push_back(
            hand.articulation.transform_point(
                0,
                point));
    }

    return centers;
}

void append_target_cube(
    sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& center,
    double half) {

    const std::uint32_t base =
        static_cast<std::uint32_t>(
            frame.positions.size());

    const sarx::Vec3 corners[8] = {
        {center.x - half, center.y - half, center.z - half},
        {center.x + half, center.y - half, center.z - half},
        {center.x + half, center.y + half, center.z - half},
        {center.x - half, center.y + half, center.z - half},
        {center.x - half, center.y - half, center.z + half},
        {center.x + half, center.y - half, center.z + half},
        {center.x + half, center.y + half, center.z + half},
        {center.x - half, center.y + half, center.z + half}
    };

    frame.positions.insert(
        frame.positions.end(),
        std::begin(corners),
        std::end(corners));

    constexpr std::uint32_t indices[36] = {
        0, 1, 2, 0, 2, 3,
        4, 6, 5, 4, 7, 6,
        0, 4, 5, 0, 5, 1,
        1, 5, 6, 1, 6, 2,
        2, 6, 7, 2, 7, 3,
        3, 7, 4, 3, 4, 0
    };

    for (const auto index : indices) {
        frame.indices.push_back(
            base + index);
    }
}

sarx::Vec3 region_centroid(
    const sarx::VoxelizedCharacter& character,
    const std::vector<sarx::Vec3>& centers,
    const std::vector<std::string>& regions) {

    sarx::Vec3 center{};
    std::size_t count = 0;

    for (std::size_t i = 0;
         i < character.voxels().size();
         ++i) {

        const auto& voxel =
            character.voxels()[i];

        if (voxel.state
                != sarx::CharacterVoxelState::Attached
            || std::find(
                regions.begin(),
                regions.end(),
                voxel.anatomical_region)
                == regions.end()) {
            continue;
        }

        center += centers[i];
        ++count;
    }

    if (count == 0) {
        return {};
    }

    return center
        / static_cast<double>(count);
}

double torso_override_rms(
    const sarx::VoxelizedCharacter& character,
    const std::vector<sarx::Vec3>& base_centers,
    const std::vector<sarx::Vec3>& composed_centers) {

    double squared = 0.0;
    std::size_t count = 0;

    for (std::size_t i = 0;
         i < character.voxels().size();
         ++i) {

        const auto& voxel =
            character.voxels()[i];

        if (voxel.state
                != sarx::CharacterVoxelState::Attached
            || (voxel.anatomical_region != "pelvis"
                && voxel.anatomical_region != "spine_01"
                && voxel.anatomical_region != "spine_02"
                && voxel.anatomical_region != "spine_03")) {
            continue;
        }

        squared +=
            sarx::length_squared(
                composed_centers[i]
                - base_centers[i]);

        ++count;
    }

    return count > 0
        ? std::sqrt(
            squared
            / static_cast<double>(count))
        : 0.0;
}

sarx::Vec3 normalized_or_throw(
    const sarx::Vec3& vector,
    const std::string& label) {

    const double magnitude =
        sarx::length(vector);

    if (magnitude <= 1e-9) {
        throw std::runtime_error(
            label + " has zero magnitude");
    }

    return vector / magnitude;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args =
            parse_args(argc, argv);

        std::filesystem::create_directories(
            args.output);

        sarx::GltfCharacter base_character;
        base_character.load(
            args.character,
            args.animations);

        sarx::GltfCharacter replacement_character;
        replacement_character.load(
            args.character,
            args.replacement_animations);

        const std::size_t base_clip =
            base_character.find_animation(
                args.clip);

        const std::size_t replacement_clip =
            replacement_character.find_animation(
                args.replacement_clip);

        const double base_duration =
            base_character.animation_duration(
                base_clip);

        const double replacement_duration =
            replacement_character.animation_duration(
                replacement_clip);

        const auto base_capability =
            sarx::describe_authored_action(
                base_character
                    .animation_names()[base_clip]);

        if (base_capability.family
                != sarx::ActionFamily::Punch
            || base_capability.side
                != sarx::ActionSide::Left) {
            throw std::runtime_error(
                "canonical M2-E1 fixture expects left-handed Punch_Jab");
        }

        const auto elbow_capability =
            sarx::make_elbow_strike_capability(
                replacement_character
                    .animation_names()[replacement_clip],
                sarx::ActionSide::Left);

        const auto opposite_punch =
            sarx::describe_authored_action(
                "Punch_Cross");

        const auto authority =
            sarx::build_action_authority_plan(
                base_character.skin_joints(),
                elbow_capability,
                {"hand_l"});

        const auto rest =
            base_character.sample(
                base_clip,
                0.0,
                false);

        const Bounds bounds =
            bounds_of(rest);

        const sarx::Vec3 character_center =
            (bounds.min + bounds.max)
            * 0.5;

        const double scale =
            std::max({
                bounds.max.x - bounds.min.x,
                bounds.max.y - bounds.min.y,
                bounds.max.z - bounds.min.z,
                0.5
            });

        sarx::VoxelizedCharacter voxel_character;
        voxel_character.build(
            base_character,
            base_clip,
            0.0,
            args.voxel_size);

        const double dt =
            1.0 / args.fps;

        const double cut_time =
            static_cast<double>(
                args.cut_frame)
            / args.fps;

        // Establish the canonical attack direction from the real Quaternius
        // Jab's measured peak extension.
        const double jab_peak_time =
            base_duration * 0.326923;

        const auto jab_peak_pose =
            base_character.sample_node_local_poses(
                base_clip,
                jab_peak_time,
                false);

        const sarx::Vec3 jab_peak_hand =
            base_character
                .node_world_position_with_local_poses(
                    jab_peak_pose,
                    "hand_l");

        const sarx::Vec3 jab_peak_pelvis =
            base_character
                .node_world_position_with_local_poses(
                    jab_peak_pose,
                    "pelvis");

        const sarx::Vec3 attack_direction =
            normalized_or_throw(
                jab_peak_hand
                    - jab_peak_pelvis,
                "Punch_Jab attack direction");

        // Target placement is derived from the composed pose, not from the
        // replacement clip in isolation. This prevents a false contact proof
        // when base-torso authority changes the elbow trajectory.
        sarx::Vec3 target{};
        double best_projection =
            -std::numeric_limits<double>::infinity();

        const double scan_duration =
            std::min(
                2.0,
                std::max(
                    0.0,
                    replacement_duration
                        - args.replacement_start_seconds));

        for (double t = 0.0;
             t <= scan_duration + 1e-9;
             t += 1.0 / 60.0) {

            const auto base_pose =
                base_character.sample_node_local_poses(
                    base_clip,
                    cut_time + t,
                    false);

            const auto replacement_pose =
                replacement_character.sample_node_local_poses(
                    replacement_clip,
                    args.replacement_start_seconds + t,
                    false);

            const auto composed =
                sarx::compose_action_local_poses(
                    base_pose,
                    replacement_pose,
                    authority,
                    1.0);

            const sarx::Vec3 elbow =
                base_character
                    .node_world_position_with_local_poses(
                        composed,
                        "lowerarm_l");

            const sarx::Vec3 pelvis =
                base_character
                    .node_world_position_with_local_poses(
                        composed,
                        "pelvis");

            const double projection =
                sarx::dot(
                    elbow - pelvis,
                    attack_direction);

            if (projection > best_projection) {
                best_projection = projection;
                target = elbow;
            }
        }

        if (!std::isfinite(best_projection)) {
            throw std::runtime_error(
                "could not derive composed elbow target");
        }

        const double target_radius =
            std::max(
                0.065,
                args.voxel_size * 1.45);

        std::size_t destroyed_total = 0;
        int detached_frame = -1;
        int punch_invalidated_frame = -1;
        int substitution_selected_frame = -1;
        int elbow_contact_frame = -1;

        std::optional<DetachedHand>
            detached_hand;

        sarx::ActionSubstitutionPlan
            substitution_plan;

        double max_torso_override = 0.0;
        double min_elbow_target_distance =
            std::numeric_limits<double>::infinity();
        double elbow_contact_velocity = 0.0;

        sarx::Vec3 previous_elbow{};
        bool have_previous_elbow = false;

        std::size_t unrelated_changed_voxels = 0;

        for (int frame = 0;
             frame < args.frames;
             ++frame) {

            const double time =
                static_cast<double>(frame)
                / args.fps;

            const double base_time =
                std::min(
                    time,
                    base_duration);

            const auto base_pose =
                base_character.sample_node_local_poses(
                    base_clip,
                    base_time,
                    false);

            auto voxel_centers =
                voxel_character.sample_centers(
                    base_character,
                    base_clip,
                    base_time,
                    false);

            const auto base_centers =
                voxel_centers;

            const auto hand_split =
                base_character.sample_split_branch(
                    base_clip,
                    base_time,
                    "hand_l",
                    false);

            const sarx::Vec3 hand_center =
                used_centroid(
                    hand_split.detached);

            const sarx::Vec3 wrist =
                nearest_anchor(
                    hand_split.body,
                    hand_split.detached);

            if (!detached_hand
                && frame >= args.cut_frame
                && frame < args.cut_frame + 8) {

                const sarx::Vec3 hand_axis =
                    hand_center - wrist;

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        wrist,
                        hand_axis,
                        args.voxel_size * 0.82,
                        args.voxel_size * 4.0,
                        0.55,
                        {"hand_l", "lowerarm_l"});

                destroyed_total +=
                    voxel_character.damage_anatomical_interface(
                        "hand_l",
                        {"lowerarm_l"},
                        0.55);

                auto component =
                    voxel_character
                        .detach_anatomical_region_if_disconnected(
                            "hand_l",
                            {"lowerarm_l"},
                            6);

                if (component) {
                    DetachedHand hand;
                    hand.component =
                        std::move(*component);

                    const auto previous_centers =
                        voxel_character.sample_centers(
                            base_character,
                            base_clip,
                            std::max(
                                0.0,
                                base_time - dt),
                            false);

                    for (const auto index
                         : hand.component.voxel_indices) {

                        hand.rest_centers.push_back(
                            voxel_character.voxel_center(
                                index,
                                voxel_centers));
                    }

                    std::vector<sarx::Vec3>
                        previous_component_centers;

                    previous_component_centers.reserve(
                        hand.component
                            .voxel_indices
                            .size());

                    for (const auto index
                         : hand.component.voxel_indices) {

                        previous_component_centers.push_back(
                            voxel_character.voxel_center(
                                index,
                                previous_centers));
                    }

                    const auto [a, b] =
                        farthest_pair(
                            hand.rest_centers);

                    sarx::DetachedArticulationConfig
                        articulation;

                    articulation.rest_anchors = {
                        hand.rest_centers[a],
                        hand.rest_centers[b]
                    };

                    articulation.previous_anchors = {
                        previous_component_centers[a],
                        previous_component_centers[b]
                    };

                    articulation.masses = {
                        0.5,
                        0.5
                    };

                    articulation.ground_radius =
                        args.voxel_size * 0.47;
                    articulation.restitution = 0.08;
                    articulation.tangential_damping = 0.64;
                    articulation.contact_velocity_scale = 0.20;
                    articulation.global_velocity_damping = 0.994;
                    articulation.contact_iterations = 8;
                    articulation.substeps = 6;
                    articulation.solver_iterations = 14;

                    hand.articulation.initialize(
                        articulation,
                        dt);

                    detached_frame = frame;
                    detached_hand =
                        std::move(hand);
                }
            }

            if (detached_hand
                && frame > detached_frame) {
                detached_hand
                    ->articulation
                    .step(dt);
            }

            const auto current_viability =
                sarx::evaluate_action_viability(
                    base_capability,
                    voxel_character
                        .anatomy_availability());

            if (current_viability.state
                    == sarx::MotionViability::Invalid
                && punch_invalidated_frame < 0) {

                punch_invalidated_frame =
                    frame;

                sarx::ActionExecutionState
                    execution;

                execution.motion_id =
                    base_capability.motion_id;

                execution.normalized_phase =
                    base_duration > 1e-9
                    ? base_time / base_duration
                    : 0.0;

                substitution_plan =
                    sarx::plan_action_substitution(
                        sarx::BehavioralIntent::Attack,
                        base_capability,
                        {
                            elbow_capability,
                            opposite_punch
                        },
                        voxel_character
                            .anatomy_availability(),
                        execution);

                if (substitution_plan.transition_required
                    && substitution_plan.selected.motion_id
                        == elbow_capability.motion_id) {

                    substitution_selected_frame =
                        frame;
                }
            }

            if (substitution_selected_frame >= 0
                && frame >= substitution_selected_frame) {

                const double elapsed =
                    static_cast<double>(
                        frame - substitution_selected_frame)
                    / args.fps;

                const auto replacement_pose =
                    replacement_character
                        .sample_node_local_poses(
                            replacement_clip,
                            args.replacement_start_seconds
                                + elapsed,
                            false);

                const double raw_blend =
                    std::clamp(
                        elapsed / 0.15,
                        0.0,
                        1.0);

                const double blend =
                    raw_blend
                    * raw_blend
                    * (3.0 - 2.0 * raw_blend);

                const auto composed_pose =
                    sarx::compose_action_local_poses(
                        base_pose,
                        replacement_pose,
                        authority,
                        blend);

                voxel_centers =
                    voxel_character
                        .sample_centers_with_node_local_poses(
                            base_character,
                            composed_pose);

                max_torso_override =
                    std::max(
                        max_torso_override,
                        torso_override_rms(
                            voxel_character,
                            base_centers,
                            voxel_centers));

                const sarx::Vec3 elbow =
                    base_character
                        .node_world_position_with_local_poses(
                            composed_pose,
                            "lowerarm_l");

                const double distance =
                    sarx::length(
                        elbow - target);

                min_elbow_target_distance =
                    std::min(
                        min_elbow_target_distance,
                        distance);

                double elbow_speed = 0.0;

                if (have_previous_elbow) {
                    elbow_speed =
                        sarx::length(
                            elbow
                            - previous_elbow)
                        / dt;
                }

                previous_elbow =
                    elbow;

                have_previous_elbow = true;

                if (elbow_contact_frame < 0
                    && distance <= target_radius) {

                    elbow_contact_frame =
                        frame;

                    elbow_contact_velocity =
                        elbow_speed;
                }
            }

            sarx::CharacterMeshFrame visible =
                voxel_character.render(
                    voxel_centers);

            if (detached_hand) {
                const auto detached_centers =
                    detached_world_centers(
                        *detached_hand);

                auto detached_mesh =
                    voxel_character
                        .render_component(
                            detached_hand->component,
                            detached_centers);

                const std::uint32_t base =
                    static_cast<std::uint32_t>(
                        visible.positions.size());

                visible.positions.insert(
                    visible.positions.end(),
                    detached_mesh.positions.begin(),
                    detached_mesh.positions.end());

                for (const auto index
                     : detached_mesh.indices) {
                    visible.indices.push_back(
                        base + index);
                }
            }

            append_target_cube(
                visible,
                target,
                target_radius * 0.55);

            sarx::CharacterRenderCamera camera;

            camera.target =
                character_center
                + sarx::Vec3{
                    0.0,
                    -scale * 0.03,
                    0.0
                };

            camera.position =
                camera.target
                + sarx::Vec3{
                    scale * 0.70,
                    scale * 0.18,
                    scale * 1.45
                };

            camera.vertical_fov_degrees =
                31.0;
            camera.width = 960;
            camera.height = 720;

            sarx::write_character_ppm(
                frame_path(
                    args.output,
                    frame),
                visible,
                camera,
                true);
        }

        for (const auto& voxel
             : voxel_character.voxels()) {

            if (voxel.state
                    == sarx::CharacterVoxelState::Attached
                || voxel.anatomical_region == "hand_l"
                || voxel.anatomical_region == "lowerarm_l") {
                continue;
            }

            ++unrelated_changed_voxels;
        }

        const double hand_fraction =
            voxel_character.attached_fraction(
                "hand_l");

        const double lowerarm_fraction =
            voxel_character.attached_fraction(
                "lowerarm_l");

        const double upperarm_fraction =
            voxel_character.attached_fraction(
                "upperarm_l");

        const auto stats =
            voxel_character.stats();

        if (args.require_damage
            && destroyed_total == 0) {
            throw std::runtime_error(
                "attack fixture destroyed no wrist voxels");
        }

        if (args.require_detachment
            && !detached_hand) {
            throw std::runtime_error(
                "attack fixture never detached the punching hand");
        }

        if (args.require_substitution
            && (punch_invalidated_frame < 0
                || substitution_selected_frame < 0
                || substitution_plan.selected.family
                    != sarx::ActionFamily::ElbowStrike
                || substitution_plan.selected.side
                    != sarx::ActionSide::Left)) {

            throw std::runtime_error(
                "hand loss did not select same-side authored elbow substitution");
        }

        if (args.require_contact
            && (elbow_contact_frame < 0
                || !std::isfinite(
                    min_elbow_target_distance)
                || min_elbow_target_distance
                    > target_radius)) {

            throw std::runtime_error(
                "replacement elbow never reached authoritative target");
        }

        if (args.require_authority
            && (authority.count(
                    sarx::AnimationAuthoritySource::ReplacementAnimation)
                    == 0
                || authority.count(
                    sarx::AnimationAuthoritySource::Physics)
                    == 0
                || max_torso_override > 1e-6)) {

            throw std::runtime_error(
                "regional animation authority proof failed");
        }

        if (args.require_isolation
            && (unrelated_changed_voxels != 0
                || hand_fraction > 0.05
                || lowerarm_fraction < 0.75
                || upperarm_fraction < 0.95)) {

            throw std::runtime_error(
                "wrist cut damaged unrelated/proximal combat anatomy");
        }

        std::cout
            << "SARX action substitution demo complete:"
            << " base_action="
            << base_capability.motion_id
            << " base_effector="
            << sarx::action_effector_name(
                base_capability.effector)
            << " replacement_action="
            << substitution_plan.selected.motion_id
            << " replacement_effector="
            << sarx::action_effector_name(
                substitution_plan.selected.effector)
            << " total_voxels="
            << stats.total_voxels
            << " destroyed_voxels="
            << stats.destroyed_voxels
            << " detached_voxels="
            << stats.detached_voxels
            << " hand_detached_frame="
            << detached_frame
            << " punch_invalidated_frame="
            << punch_invalidated_frame
            << " substitution_selected_frame="
            << substitution_selected_frame
            << " hand_l_attached_fraction="
            << hand_fraction
            << " lowerarm_l_attached_fraction="
            << lowerarm_fraction
            << " upperarm_l_attached_fraction="
            << upperarm_fraction
            << " base_authority_joints="
            << authority.count(
                sarx::AnimationAuthoritySource::BaseAnimation)
            << " replacement_authority_joints="
            << authority.count(
                sarx::AnimationAuthoritySource::ReplacementAnimation)
            << " physics_authority_joints="
            << authority.count(
                sarx::AnimationAuthoritySource::Physics)
            << " max_torso_override_rms="
            << max_torso_override
            << " min_elbow_target_distance="
            << min_elbow_target_distance
            << " elbow_contact_frame="
            << elbow_contact_frame
            << " elbow_contact_velocity="
            << elbow_contact_velocity
            << " detached_hand_ground_contacts="
            << (detached_hand
                ? detached_hand
                    ->articulation
                    .ground_contacts()
                : 0)
            << " unrelated_changed_voxels="
            << unrelated_changed_voxels
            << " frames="
            << args.frames
            << " output="
            << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_voxel_attack_substitution_demo: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}
