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
    std::filesystem::path output{
        "media/raw/v17c_arm_loss_to_cross_frames"};
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
        } else if (value == "--output" && i + 1 < argc) {
            args.output = argv[++i];
        } else if (value == "--frames" && i + 1 < argc) {
            args.frames = std::stoi(argv[++i]);
        } else if (value == "--cut-frame" && i + 1 < argc) {
            args.cut_frame = std::stoi(argv[++i]);
        } else if (value == "--fps" && i + 1 < argc) {
            args.fps = std::stod(argv[++i]);
        } else if (value == "--voxel-size" && i + 1 < argc) {
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
                << "sarx_character_voxel_arm_loss_attack_demo"
                << " [--output DIR] [--frames N] [--cut-frame N]"
                << " [--fps N] [--voxel-size N]"
                << " [--require-damage] [--require-detachment]"
                << " [--require-substitution] [--require-contact]"
                << " [--require-authority] [--require-isolation]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete arm-loss attack argument");
        }
    }

    if (args.frames <= 0
        || args.cut_frame < 1
        || args.cut_frame >= args.frames
        || args.fps <= 0.0
        || args.voxel_size <= 0.0) {
        throw std::invalid_argument(
            "invalid arm-loss attack settings");
    }

    return args;
}

std::string frame_path(
    const std::filesystem::path& directory,
    int frame) {

    std::ostringstream name;
    name << "frame_" << std::setw(4)
         << std::setfill('0') << frame << ".ppm";
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

Bounds bounds_of(const sarx::CharacterMeshFrame& frame) {
    Bounds bounds;
    for (const auto& p : frame.positions) {
        bounds.min.x = std::min(bounds.min.x, p.x);
        bounds.min.y = std::min(bounds.min.y, p.y);
        bounds.min.z = std::min(bounds.min.z, p.z);
        bounds.max.x = std::max(bounds.max.x, p.x);
        bounds.max.y = std::max(bounds.max.y, p.y);
        bounds.max.z = std::max(bounds.max.z, p.z);
    }
    return bounds;
}

std::vector<std::uint8_t>
used_vertices(const sarx::CharacterMeshFrame& frame) {
    std::vector<std::uint8_t> used(frame.positions.size(), 0u);
    for (const auto index : frame.indices) {
        if (index < used.size()) used[index] = 1u;
    }
    return used;
}

sarx::Vec3 used_centroid(
    const sarx::CharacterMeshFrame& frame) {

    const auto used = used_vertices(frame);
    sarx::Vec3 center{};
    std::size_t count = 0;

    for (std::size_t i = 0; i < used.size(); ++i) {
        if (!used[i]) continue;
        center += frame.positions[i];
        ++count;
    }

    if (count == 0) {
        throw std::runtime_error("branch has no used vertices");
    }

    return center / static_cast<double>(count);
}

sarx::Vec3 nearest_anchor(
    const sarx::CharacterMeshFrame& body,
    const sarx::CharacterMeshFrame& detached) {

    const auto body_used = used_vertices(body);
    const auto detached_used = used_vertices(detached);

    double best = std::numeric_limits<double>::infinity();
    sarx::Vec3 a{};
    sarx::Vec3 b{};

    for (std::size_t i = 0; i < body_used.size(); ++i) {
        if (!body_used[i]) continue;

        for (std::size_t j = 0; j < detached_used.size(); ++j) {
            if (!detached_used[j]) continue;

            const double d2 =
                sarx::length_squared(
                    body.positions[i] - detached.positions[j]);

            if (d2 < best) {
                best = d2;
                a = body.positions[i];
                b = detached.positions[j];
            }
        }
    }

    if (!std::isfinite(best)) {
        throw std::runtime_error(
            "could not infer arm joint anchor");
    }

    return (a + b) * 0.5;
}

sarx::Vec3 farthest_used_point(
    const sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& from) {

    const auto used = used_vertices(frame);
    double best = -1.0;
    sarx::Vec3 result{};

    for (std::size_t i = 0; i < used.size(); ++i) {
        if (!used[i]) continue;

        const double d2 =
            sarx::length_squared(frame.positions[i] - from);

        if (d2 > best) {
            best = d2;
            result = frame.positions[i];
        }
    }

    if (best < 0.0) {
        throw std::runtime_error(
            "could not infer hand endpoint");
    }

    return result;
}

sarx::CharacterMeshFrame combine(
    const sarx::CharacterMeshFrame& a,
    const sarx::CharacterMeshFrame& b) {

    sarx::CharacterMeshFrame out = a;
    const auto base =
        static_cast<std::uint32_t>(out.positions.size());

    out.positions.insert(
        out.positions.end(),
        b.positions.begin(),
        b.positions.end());

    for (const auto index : b.indices) {
        out.indices.push_back(base + index);
    }

    return out;
}

void append_target_cube(
    sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& center,
    double half) {

    const std::uint32_t base =
        static_cast<std::uint32_t>(frame.positions.size());

    const sarx::Vec3 corners[8] = {
        {center.x-half, center.y-half, center.z-half},
        {center.x+half, center.y-half, center.z-half},
        {center.x+half, center.y+half, center.z-half},
        {center.x-half, center.y+half, center.z-half},
        {center.x-half, center.y-half, center.z+half},
        {center.x+half, center.y-half, center.z+half},
        {center.x+half, center.y+half, center.z+half},
        {center.x-half, center.y+half, center.z+half}
    };

    frame.positions.insert(
        frame.positions.end(),
        std::begin(corners),
        std::end(corners));

    constexpr std::uint32_t triangles[36] = {
        0,1,2, 0,2,3, 4,6,5, 4,7,6,
        0,4,5, 0,5,1, 1,5,6, 1,6,2,
        2,6,7, 2,7,3, 3,7,4, 3,4,0
    };

    for (const auto index : triangles) {
        frame.indices.push_back(base + index);
    }
}

struct DetachedArm {
    sarx::DetachedVoxelComponent component;
    std::vector<sarx::Vec3> rest_centers;
    std::vector<std::size_t> segment_by_voxel;
    sarx::DetachedArticulatedChain articulation;
};

std::vector<sarx::Vec3>
detached_world_centers(const DetachedArm& arm) {
    if (arm.rest_centers.size()
        != arm.segment_by_voxel.size()) {
        throw std::logic_error(
            "detached arm segment map mismatch");
    }

    std::vector<sarx::Vec3> centers;
    centers.reserve(arm.rest_centers.size());

    for (std::size_t i = 0; i < arm.rest_centers.size(); ++i) {
        centers.push_back(
            arm.articulation.transform_point(
                arm.segment_by_voxel[i],
                arm.rest_centers[i]));
    }

    return centers;
}

double torso_override_rms(
    const sarx::VoxelizedCharacter& character,
    const std::vector<sarx::Vec3>& base,
    const std::vector<sarx::Vec3>& composed) {

    double squared = 0.0;
    std::size_t count = 0;

    for (std::size_t i = 0; i < character.voxels().size(); ++i) {
        const auto& voxel = character.voxels()[i];

        if (voxel.state != sarx::CharacterVoxelState::Attached
            || (voxel.anatomical_region != "pelvis"
                && voxel.anatomical_region != "spine_01"
                && voxel.anatomical_region != "spine_02"
                && voxel.anatomical_region != "spine_03")) {
            continue;
        }

        squared +=
            sarx::length_squared(composed[i] - base[i]);
        ++count;
    }

    return count
        ? std::sqrt(squared / static_cast<double>(count))
        : 0.0;
}

sarx::Vec3 normalized_or_throw(
    const sarx::Vec3& value,
    const char* label) {

    const double magnitude =
        sarx::length(value);

    if (magnitude <= 1e-9) {
        throw std::runtime_error(
            std::string(label)
            + " has zero magnitude");
    }

    return value / magnitude;
}


double point_segment_distance(
    const sarx::Vec3& point,
    const sarx::Vec3& a,
    const sarx::Vec3& b) {

    const sarx::Vec3 ab =
        b - a;

    const double ab_length_squared =
        sarx::length_squared(ab);

    if (ab_length_squared <= 1e-12) {
        return sarx::length(
            point - a);
    }

    const double t =
        std::clamp(
            sarx::dot(
                point - a,
                ab)
                / ab_length_squared,
            0.0,
            1.0);

    return sarx::length(
        point
        - (a + ab * t));
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);

        std::filesystem::create_directories(args.output);

        sarx::GltfCharacter character;
        character.load(args.character, args.animations);

        const std::size_t jab =
            character.find_animation("Punch_Jab");
        const std::size_t cross =
            character.find_animation("Punch_Cross");

        const double jab_duration =
            character.animation_duration(jab);
        const double cross_duration =
            character.animation_duration(cross);

        const auto jab_capability =
            sarx::describe_authored_action("Punch_Jab");
        const auto cross_capability =
            sarx::describe_authored_action("Punch_Cross");

        if (jab_capability.side != sarx::ActionSide::Left
            || cross_capability.side != sarx::ActionSide::Right) {
            throw std::runtime_error(
                "audited Jab/Cross side metadata is inconsistent");
        }

        const auto rejected_elbow =
            sarx::make_elbow_strike_capability(
                "Elbow_Left",
                sarx::ActionSide::Left);

        const auto authority =
            sarx::build_action_authority_plan(
                character.skin_joints(),
                cross_capability,
                {"upperarm_l"});

        const auto rest = character.sample(jab, 0.0, false);
        const auto bounds = bounds_of(rest);

        const sarx::Vec3 character_center =
            (bounds.min + bounds.max) * 0.5;

        const double scale = std::max({
            bounds.max.x - bounds.min.x,
            bounds.max.y - bounds.min.y,
            bounds.max.z - bounds.min.z,
            0.5
        });

        sarx::VoxelizedCharacter voxel_character;
        voxel_character.build(
            character,
            jab,
            0.0,
            args.voxel_size);

        const double dt = 1.0 / args.fps;
        const double cut_time =
            static_cast<double>(args.cut_frame) / args.fps;

        // Freeze one opponent torso volume before damage. It is centered
        // between the audited intact Jab and intact Cross contact points, so
        // both authored attacks demonstrably reach the same opponent without
        // placing the target on the damaged fallback trajectory itself.
        const double jab_peak_time =
            jab_duration * 0.326923;

        const double cross_peak_time =
            cross_duration * 0.266667;

        const auto jab_peak_pose =
            character.sample_node_local_poses(
                jab,
                jab_peak_time,
                false);

        const auto cross_peak_pose =
            character.sample_node_local_poses(
                cross,
                cross_peak_time,
                false);

        const sarx::Vec3 jab_peak_hand =
            character.node_world_position_with_local_poses(
                jab_peak_pose,
                "hand_l");

        const sarx::Vec3 cross_peak_hand =
            character.node_world_position_with_local_poses(
                cross_peak_pose,
                "hand_r");

        const sarx::Vec3 target =
            (jab_peak_hand
             + cross_peak_hand)
            * 0.5;

        const double baseline_separation =
            sarx::length(
                jab_peak_hand
                - cross_peak_hand);

        const double target_radius =
            std::max(
                baseline_separation * 0.5
                    + 0.06,
                args.voxel_size * 2.5);

        std::size_t destroyed_total = 0;
        int detached_frame = -1;
        int jab_invalidated_frame = -1;
        int substitution_selected_frame = -1;
        int contact_frame = -1;

        std::optional<DetachedArm> detached_arm;
        sarx::ActionSubstitutionPlan substitution_plan;

        double max_torso_override = 0.0;
        double min_target_distance =
            std::numeric_limits<double>::infinity();
        double contact_velocity = 0.0;

        sarx::Vec3 previous_hand{};
        bool have_previous_hand = false;

        for (int frame = 0; frame < args.frames; ++frame) {
            const double time =
                static_cast<double>(frame) / args.fps;

            const double base_time =
                std::min(time, jab_duration);

            const auto base_pose =
                character.sample_node_local_poses(
                    jab,
                    base_time,
                    false);

            auto voxel_centers =
                voxel_character.sample_centers(
                    character,
                    jab,
                    base_time,
                    false);

            const auto base_centers = voxel_centers;

            const auto upperarm_split =
                character.sample_split_branch(
                    jab,
                    base_time,
                    "upperarm_l",
                    false);

            const auto lowerarm_split =
                character.sample_split_branch(
                    jab,
                    base_time,
                    "lowerarm_l",
                    false);

            const auto hand_split =
                character.sample_split_branch(
                    jab,
                    base_time,
                    "hand_l",
                    false);

            const sarx::Vec3 shoulder =
                nearest_anchor(
                    upperarm_split.body,
                    upperarm_split.detached);

            const sarx::Vec3 elbow =
                nearest_anchor(
                    lowerarm_split.body,
                    lowerarm_split.detached);

            const sarx::Vec3 wrist =
                nearest_anchor(
                    hand_split.body,
                    hand_split.detached);

            const sarx::Vec3 hand_tip =
                farthest_used_point(
                    hand_split.detached,
                    wrist);

            const sarx::Vec3 arm_seed =
                used_centroid(
                    lowerarm_split.detached);

            if (!detached_arm
                && frame >= args.cut_frame
                && frame < args.cut_frame + 6) {

                const sarx::Vec3 upperarm_axis =
                    normalized_or_throw(
                        elbow - shoulder,
                        "left upper-arm axis");

                const sarx::Vec3 shoulder_cut_center =
                    shoulder
                    + upperarm_axis
                        * (args.voxel_size * 0.45);

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        shoulder_cut_center,
                        upperarm_axis,
                        args.voxel_size * 1.05,
                        args.voxel_size * 4.8,
                        1.05,
                        {"upperarm_l"});

                auto component =
                    voxel_character
                        .detach_component_near_anatomical(
                            voxel_centers,
                            arm_seed,
                            20);

                if (component) {
                    DetachedArm arm;
                    arm.component = std::move(*component);

                    const double previous_time =
                        std::max(0.0, base_time - dt);

                    const auto previous_upper =
                        character.sample_split_branch(
                            jab,
                            previous_time,
                            "upperarm_l",
                            false);

                    const auto previous_lower =
                        character.sample_split_branch(
                            jab,
                            previous_time,
                            "lowerarm_l",
                            false);

                    const auto previous_hand_split =
                        character.sample_split_branch(
                            jab,
                            previous_time,
                            "hand_l",
                            false);

                    const sarx::Vec3 prev_shoulder =
                        nearest_anchor(
                            previous_upper.body,
                            previous_upper.detached);

                    const sarx::Vec3 prev_elbow =
                        nearest_anchor(
                            previous_lower.body,
                            previous_lower.detached);

                    const sarx::Vec3 prev_wrist =
                        nearest_anchor(
                            previous_hand_split.body,
                            previous_hand_split.detached);

                    const sarx::Vec3 prev_hand_tip =
                        farthest_used_point(
                            previous_hand_split.detached,
                            prev_wrist);

                    arm.rest_centers.reserve(
                        arm.component.voxel_indices.size());

                    arm.segment_by_voxel.reserve(
                        arm.component.voxel_indices.size());

                    for (const auto index :
                         arm.component.voxel_indices) {

                        arm.rest_centers.push_back(
                            voxel_character.voxel_center(
                                index,
                                voxel_centers));

                        const std::string& region =
                            voxel_character.voxels()[index]
                                .anatomical_region;

                        if (region == "upperarm_l") {
                            arm.segment_by_voxel.push_back(0);
                        } else if (region == "lowerarm_l") {
                            arm.segment_by_voxel.push_back(1);
                        } else if (region == "hand_l") {
                            arm.segment_by_voxel.push_back(2);
                        } else {
                            throw std::runtime_error(
                                "detached shoulder component contains non-left-arm anatomy: "
                                + region);
                        }
                    }

                    const double elbow_angle =
                        sarx::articulated_joint_angle(
                            shoulder,
                            elbow,
                            wrist);

                    const double wrist_angle =
                        sarx::articulated_joint_angle(
                            elbow,
                            wrist,
                            hand_tip);

                    sarx::DetachedArticulationConfig config;

                    config.rest_anchors = {
                        shoulder,
                        elbow,
                        wrist,
                        hand_tip
                    };

                    config.previous_anchors = {
                        prev_shoulder,
                        prev_elbow,
                        prev_wrist,
                        prev_hand_tip
                    };

                    config.masses = {
                        0.32, 0.30, 0.22, 0.16
                    };

                    config.joints = {
                        sarx::PassiveJointProfile{
                            1,
                            std::max(0.15, elbow_angle - 0.90),
                            std::min(3.10, elbow_angle + 0.90),
                            0.08, 0.70, 0.22
                        },
                        sarx::PassiveJointProfile{
                            2,
                            std::max(0.20, wrist_angle - 0.75),
                            std::min(3.10, wrist_angle + 0.75),
                            0.09, 0.72, 0.24
                        }
                    };

                    config.ground_radius =
                        args.voxel_size * 0.47;
                    config.restitution = 0.08;
                    config.tangential_damping = 0.64;
                    config.contact_velocity_scale = 0.20;
                    config.global_velocity_damping = 0.994;
                    config.contact_iterations = 8;
                    config.substeps = 6;
                    config.solver_iterations = 14;

                    arm.articulation.initialize(config, dt);

                    detached_frame = frame;
                    detached_arm = std::move(arm);
                }
            }

            if (detached_arm && frame > detached_frame) {
                detached_arm->articulation.step(dt);
            }

            const auto current_viability =
                sarx::evaluate_action_viability(
                    jab_capability,
                    voxel_character.anatomy_availability());

            if (current_viability.state
                    == sarx::MotionViability::Invalid
                && jab_invalidated_frame < 0) {

                jab_invalidated_frame = frame;

                sarx::ActionExecutionState execution;
                execution.motion_id = jab_capability.motion_id;
                execution.normalized_phase =
                    jab_duration > 1e-9
                    ? base_time / jab_duration
                    : 0.0;

                substitution_plan =
                    sarx::plan_action_substitution(
                        sarx::BehavioralIntent::Attack,
                        jab_capability,
                        {
                            rejected_elbow,
                            cross_capability
                        },
                        voxel_character.anatomy_availability(),
                        execution);

                if (substitution_plan.transition_required
                    && substitution_plan.selected.motion_id
                        == cross_capability.motion_id) {
                    substitution_selected_frame = frame;
                }
            }

            if (substitution_selected_frame >= 0
                && frame >= substitution_selected_frame) {

                const double elapsed =
                    static_cast<double>(
                        frame - substitution_selected_frame)
                    / args.fps;

                const auto replacement_pose =
                    character.sample_node_local_poses(
                        cross,
                        std::min(elapsed, cross_duration),
                        false);

                const double raw_blend =
                    std::clamp(elapsed / 0.15, 0.0, 1.0);

                const double blend =
                    raw_blend * raw_blend
                    * (3.0 - 2.0 * raw_blend);

                const auto composed =
                    sarx::compose_action_local_poses(
                        base_pose,
                        replacement_pose,
                        authority,
                        blend);

                voxel_centers =
                    voxel_character
                        .sample_centers_with_node_local_poses(
                            character,
                            composed);

                max_torso_override =
                    std::max(
                        max_torso_override,
                        torso_override_rms(
                            voxel_character,
                            base_centers,
                            voxel_centers));

                const sarx::Vec3 hand =
                    character.node_world_position_with_local_poses(
                        composed,
                        "hand_r");

                const double endpoint_distance =
                    sarx::length(
                        hand - target);

                double swept_distance =
                    endpoint_distance;

                double speed = 0.0;

                if (have_previous_hand) {
                    speed =
                        sarx::length(
                            hand - previous_hand)
                        / dt;

                    swept_distance =
                        point_segment_distance(
                            target,
                            previous_hand,
                            hand);
                }

                min_target_distance =
                    std::min(
                        min_target_distance,
                        swept_distance);

                previous_hand = hand;
                have_previous_hand = true;

                if (contact_frame < 0
                    && swept_distance
                        <= target_radius) {
                    contact_frame = frame;
                    contact_velocity = speed;
                }
            }

            sarx::CharacterMeshFrame visible =
                voxel_character.render(voxel_centers);

            if (detached_arm) {
                const auto centers =
                    detached_world_centers(*detached_arm);

                visible = combine(
                    visible,
                    voxel_character.render_component(
                        detached_arm->component,
                        centers));
            }

            append_target_cube(
                visible,
                target,
                target_radius * 0.55);

            sarx::CharacterRenderCamera camera;

            camera.target =
                character_center
                + sarx::Vec3{0.0, -scale * 0.03, 0.0};

            camera.position =
                camera.target
                + sarx::Vec3{
                    scale * 0.70,
                    scale * 0.18,
                    scale * 1.45
                };

            camera.vertical_fov_degrees = 31.0;
            camera.width = 960;
            camera.height = 720;

            sarx::write_character_ppm(
                frame_path(args.output, frame),
                visible,
                camera,
                true);
        }

        std::size_t unrelated_changed_voxels = 0;

        for (const auto& voxel : voxel_character.voxels()) {
            if (voxel.state == sarx::CharacterVoxelState::Attached
                || voxel.anatomical_region == "upperarm_l"
                || voxel.anatomical_region == "lowerarm_l"
                || voxel.anatomical_region == "hand_l") {
                continue;
            }
            ++unrelated_changed_voxels;
        }

        const double upperarm_l =
            voxel_character.attached_fraction("upperarm_l");
        const double lowerarm_l =
            voxel_character.attached_fraction("lowerarm_l");
        const double hand_l =
            voxel_character.attached_fraction("hand_l");

        const double upperarm_r =
            voxel_character.attached_fraction("upperarm_r");
        const double lowerarm_r =
            voxel_character.attached_fraction("lowerarm_r");
        const double hand_r =
            voxel_character.attached_fraction("hand_r");

        const auto stats =
            voxel_character.stats();

        auto authority_source_for =
            [&](const std::string& joint) {

                const auto found =
                    std::find_if(
                        authority.joints.begin(),
                        authority.joints.end(),
                        [&](const sarx::JointAuthorityAssignment& assignment) {
                            return assignment.joint == joint;
                        });

                return found == authority.joints.end()
                    ? sarx::AnimationAuthoritySource::Disabled
                    : found->source;
            };

        const bool torso_joint_authority_ok =
            authority_source_for("pelvis")
                    == sarx::AnimationAuthoritySource::BaseAnimation
            && authority_source_for("spine_01")
                    == sarx::AnimationAuthoritySource::BaseAnimation
            && authority_source_for("spine_02")
                    == sarx::AnimationAuthoritySource::BaseAnimation
            && authority_source_for("spine_03")
                    == sarx::AnimationAuthoritySource::BaseAnimation;

        std::cout
            << "SARX whole-arm substitution diagnostics:"
            << " destroyed_voxels=" << stats.destroyed_voxels
            << " detached_voxels=" << stats.detached_voxels
            << " arm_detached_frame=" << detached_frame
            << " jab_invalidated_frame=" << jab_invalidated_frame
            << " substitution_selected_frame="
            << substitution_selected_frame
            << " upperarm_l_attached_fraction=" << upperarm_l
            << " lowerarm_l_attached_fraction=" << lowerarm_l
            << " hand_l_attached_fraction=" << hand_l
            << " upperarm_r_attached_fraction=" << upperarm_r
            << " lowerarm_r_attached_fraction=" << lowerarm_r
            << " hand_r_attached_fraction=" << hand_r
            << " target_radius=" << target_radius
            << " baseline_contact_separation=" << baseline_separation
            << " min_target_distance=" << min_target_distance
            << " contact_frame=" << contact_frame
            << " max_torso_override_rms=" << max_torso_override
            << " torso_joint_authority_ok="
            << (torso_joint_authority_ok ? 1 : 0)
            << '\n';

        if (args.require_damage && destroyed_total == 0) {
            throw std::runtime_error(
                "shoulder cut destroyed no voxels");
        }

        if (args.require_detachment && !detached_arm) {
            throw std::runtime_error(
                "whole left arm never detached");
        }

        if (args.require_substitution
            && (jab_invalidated_frame < 0
                || substitution_selected_frame < 0
                || substitution_plan.selected.family
                    != sarx::ActionFamily::Punch
                || substitution_plan.selected.side
                    != sarx::ActionSide::Right
                || substitution_plan.selected.motion_id
                    != "Punch_Cross")) {
            throw std::runtime_error(
                "whole-arm loss did not reject elbow and select right Punch_Cross");
        }

        if (args.require_contact
            && (contact_frame < 0
                || !std::isfinite(min_target_distance)
                || min_target_distance > target_radius)) {
            throw std::runtime_error(
                "opposite-hand Cross never reached target");
        }

        if (args.require_authority
            && (authority.count(
                    sarx::AnimationAuthoritySource::ReplacementAnimation)
                    == 0
                || authority.count(
                    sarx::AnimationAuthoritySource::Physics)
                    == 0
                || !torso_joint_authority_ok
                || max_torso_override
                    > args.voxel_size * 0.12)) {
            throw std::runtime_error(
                "whole-arm regional authority proof failed");
        }

        if (args.require_isolation
            && (unrelated_changed_voxels != 0
                || upperarm_l > 0.10
                || lowerarm_l > 0.05
                || hand_l > 0.05
                || upperarm_r < 0.95
                || lowerarm_r < 0.95
                || hand_r < 0.95)) {
            throw std::runtime_error(
                "shoulder cut altered unrelated or surviving attack anatomy");
        }

        std::cout
            << "SARX whole-arm attack substitution complete:"
            << " base_action=Punch_Jab"
            << " detached_side=Left"
            << " replacement_action="
            << substitution_plan.selected.motion_id
            << " replacement_effector="
            << sarx::action_effector_name(
                substitution_plan.selected.effector)
            << " total_voxels=" << stats.total_voxels
            << " destroyed_voxels=" << stats.destroyed_voxels
            << " detached_voxels=" << stats.detached_voxels
            << " arm_detached_frame=" << detached_frame
            << " jab_invalidated_frame=" << jab_invalidated_frame
            << " substitution_selected_frame="
            << substitution_selected_frame
            << " upperarm_l_attached_fraction=" << upperarm_l
            << " lowerarm_l_attached_fraction=" << lowerarm_l
            << " hand_l_attached_fraction=" << hand_l
            << " upperarm_r_attached_fraction=" << upperarm_r
            << " lowerarm_r_attached_fraction=" << lowerarm_r
            << " hand_r_attached_fraction=" << hand_r
            << " base_authority_joints="
            << authority.count(
                sarx::AnimationAuthoritySource::BaseAnimation)
            << " replacement_authority_joints="
            << authority.count(
                sarx::AnimationAuthoritySource::ReplacementAnimation)
            << " physics_authority_joints="
            << authority.count(
                sarx::AnimationAuthoritySource::Physics)
            << " max_torso_override_rms=" << max_torso_override
            << " min_target_distance=" << min_target_distance
            << " contact_frame=" << contact_frame
            << " contact_velocity=" << contact_velocity
            << " detached_arm_ground_contacts="
            << (detached_arm
                ? detached_arm->articulation.ground_contacts()
                : 0)
            << " detached_arm_elbow_delta="
            << (detached_arm
                ? detached_arm->articulation.max_joint_angle_delta(0)
                : 0.0)
            << " detached_arm_wrist_delta="
            << (detached_arm
                ? detached_arm->articulation.max_joint_angle_delta(1)
                : 0.0)
            << " unrelated_changed_voxels="
            << unrelated_changed_voxels
            << " frames=" << args.frames
            << " output=" << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_voxel_arm_loss_attack_demo: "
            << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
