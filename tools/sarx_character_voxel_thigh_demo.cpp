#include "sarx/character_render.hpp"
#include "sarx/detached_articulation.hpp"
#include "sarx/gltf_character.hpp"
#include "sarx/motion_viability.hpp"
#include "sarx/motion_recovery.hpp"
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
    std::string clip{"Walk"};
    std::filesystem::path output{
        "media/raw/v09_quaternius_voxel_thigh_cut_frames"};
    int frames{210};
    int cut_frame{75};
    double fps{30.0};
    double voxel_size{0.045};
    bool require_damage{false};
    bool require_detachment{false};
    bool require_ground_contact{false};
    bool require_walk_invalidation{false};
    bool require_anatomical_isolation{false};
    bool require_stable_camera{false};
    bool require_articulated_leg{false};
    bool require_fall_transition{false};
    bool require_grounded_recovery{false};
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
        } else if (value == "--require-ground-contact") {
            args.require_ground_contact = true;
        } else if (value == "--require-walk-invalidation") {
            args.require_walk_invalidation = true;
        } else if (value == "--require-anatomical-isolation") {
            args.require_anatomical_isolation = true;
        } else if (value == "--require-stable-camera") {
            args.require_stable_camera = true;
        } else if (value == "--require-articulated-leg") {
            args.require_articulated_leg = true;
        } else if (value == "--require-fall-transition") {
            args.require_fall_transition = true;
        } else if (value == "--require-grounded-recovery") {
            args.require_grounded_recovery = true;
        } else if (value == "--help") {
            std::cout
                << "sarx_character_voxel_thigh_demo"
                << " [--character FILE]"
                << " [--animations FILE]"
                << " [--clip NAME]"
                << " [--output DIR]"
                << " [--frames N]"
                << " [--cut-frame N]"
                << " [--fps N]"
                << " [--voxel-size N]"
                << " [--require-damage]"
                << " [--require-detachment]"
                << " [--require-ground-contact]"
                << " [--require-walk-invalidation]"
                << " [--require-anatomical-isolation]"
                << " [--require-stable-camera]"
                << " [--require-articulated-leg]"
                << " [--require-fall-transition]"
                << " [--require-grounded-recovery]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete thigh demo argument");
        }
    }

    if (args.frames <= 0
        || args.fps <= 0.0
        || args.voxel_size <= 0.0
        || args.cut_frame < 1
        || args.cut_frame >= args.frames) {
        throw std::invalid_argument(
            "invalid thigh demo settings");
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
            "branch owns no vertices");
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

        if (!body_used[i]) continue;

        for (std::size_t j = 0;
             j < detached_used.size();
             ++j) {

            if (!detached_used[j]) continue;

            const double d2 =
                sarx::length_squared(
                    body.positions[i]
                    - detached.positions[j]);

            if (d2 < best) {
                best = d2;
                body_point =
                    body.positions[i];
                detached_point =
                    detached.positions[j];
            }
        }
    }

    if (!std::isfinite(best)) {
        throw std::runtime_error(
            "could not infer anatomical joint anchor");
    }

    return
        (body_point + detached_point)
        * 0.5;
}

sarx::CharacterMeshFrame combine(
    const sarx::CharacterMeshFrame& a,
    const sarx::CharacterMeshFrame& b) {

    sarx::CharacterMeshFrame out;

    out.positions = a.positions;
    out.indices = a.indices;

    const std::uint32_t base =
        static_cast<std::uint32_t>(
            out.positions.size());

    out.positions.insert(
        out.positions.end(),
        b.positions.begin(),
        b.positions.end());

    for (const auto index
         : b.indices) {
        out.indices.push_back(
            base + index);
    }

    return out;
}

sarx::Vec3 farthest_used_point(
    const sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& from) {

    const auto used =
        used_vertices(frame);

    double best = -1.0;
    sarx::Vec3 result{};

    for (std::size_t i = 0;
         i < used.size();
         ++i) {

        if (!used[i]) {
            continue;
        }

        const double distance =
            sarx::length_squared(
                frame.positions[i] - from);

        if (distance > best) {
            best = distance;
            result = frame.positions[i];
        }
    }

    if (best < 0.0) {
        throw std::runtime_error(
            "could not infer distal foot endpoint");
    }

    return result;
}

struct DetachedLeg {
    sarx::DetachedVoxelComponent component;
    std::vector<sarx::Vec3> rest_centers;
    std::vector<std::size_t> segment_by_voxel;
    sarx::DetachedArticulatedChain articulation;
};

std::vector<sarx::Vec3>
detached_world_centers(
    const DetachedLeg& leg) {

    if (leg.rest_centers.size()
        != leg.segment_by_voxel.size()) {
        throw std::logic_error(
            "detached leg segment mapping mismatch");
    }

    std::vector<sarx::Vec3> centers;
    centers.reserve(
        leg.rest_centers.size());

    for (std::size_t i = 0;
         i < leg.rest_centers.size();
         ++i) {

        centers.push_back(
            leg.articulation.transform_point(
                leg.segment_by_voxel[i],
                leg.rest_centers[i]));
    }

    return centers;
}

sarx::Vec3 centroid(
    const std::vector<sarx::Vec3>& points) {

    if (points.empty()) {
        return {};
    }

    sarx::Vec3 center{};

    for (const auto& point : points) {
        center += point;
    }

    return center
        / static_cast<double>(
            points.size());
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args =
            parse_args(argc, argv);

        std::filesystem::create_directories(
            args.output);

        sarx::GltfCharacter character;
        character.load(
            args.character,
            args.animations);

        const std::size_t clip =
            character.find_animation(
                args.clip);

        const auto initial =
            character.sample(
                clip,
                0.0,
                true);

        sarx::VoxelizedCharacter voxel_character;
        voxel_character.build(
            character,
            clip,
            0.0,
            args.voxel_size);

        const Bounds bounds =
            bounds_of(initial);

        const sarx::Vec3 character_center =
            (bounds.min + bounds.max)
            * 0.5;

        const double width =
            bounds.max.x - bounds.min.x;

        const double height =
            bounds.max.y - bounds.min.y;

        const double depth =
            bounds.max.z - bounds.min.z;

        const double scale =
            std::max({
                width,
                height,
                depth,
                0.5
            });

        const double travel =
            std::max(
                height * 1.15,
                scale);

        auto world_offset_for =
            [&](int frame) {

                const double progress =
                    args.frames > 1
                    ? static_cast<double>(frame)
                        / static_cast<double>(
                            args.frames - 1)
                    : 0.0;

                return sarx::Vec3{
                    0.0,
                    0.0,
                    -travel * 0.5
                        + travel * progress
                };
            };

        const double dt =
            1.0 / args.fps;

        std::size_t destroyed_total = 0;
        int detached_frame = -1;
        int walk_invalidated_frame = -1;

        bool normal_walk_authority = true;

        double motion_time_seconds = 0.0;
        int motion_frame = 0;

        sarx::MotionRecoveryPlan recovery_plan;
        std::optional<std::size_t> recovery_clip;
        std::vector<sarx::Vec3> recovery_source_centers;
        sarx::Vec3 recovery_start_world_offset{};
        sarx::Vec3 recovery_world_offset{};
        sarx::Vec3 recovery_root_velocity{};
        sarx::Vec3 inherited_root_velocity{};
        double recovery_time_seconds = 0.0;
        double recovery_clip_duration = 0.0;
        double root_velocity_discontinuity = 0.0;
        double fall_centroid_drop = 0.0;
        double max_fall_pose_rms = 0.0;
        double max_fall_floor_projection = 0.0;
        int recovery_selected_frame = -1;
        int recovery_completed_frame = -1;

        sarx::GroundedRecoveryPlan
            grounded_recovery_plan;

        bool grounded_recovery_planned = false;
        std::optional<std::size_t> grounded_recovery_clip;
        std::vector<sarx::Vec3>
            grounded_recovery_source_centers;

        sarx::Vec3 grounded_recovery_start_world_offset{};
        sarx::Vec3 grounded_recovery_world_offset{};
        sarx::Vec3 grounded_recovery_root_velocity{};
        sarx::Vec3 grounded_inherited_root_velocity{};

        double grounded_recovery_time_seconds = 0.0;
        double grounded_velocity_discontinuity = 0.0;
        double max_grounded_pose_rms = 0.0;
        double grounded_centroid_rise = 0.0;
        double max_grounded_floor_projection = 0.0;

        std::size_t fall_body_ground_contacts = 0;
        double fall_body_min_y =
            std::numeric_limits<double>::infinity();

        int grounded_recovery_selected_frame = -1;
        int grounded_pose_engaged_frame = -1;

        std::optional<DetachedLeg>
            detached_leg;

        sarx::Vec3 previous_camera_target{};
        sarx::Vec3 previous_camera_world_offset{};
        bool have_previous_camera = false;
        double max_camera_tracking_error = 0.0;

        for (int frame = 0;
             frame < args.frames;
             ++frame) {

            const double requested_seconds =
                static_cast<double>(frame)
                / args.fps;

            if (normal_walk_authority) {
                motion_time_seconds =
                    requested_seconds;
                motion_frame = frame;
            }

            const sarx::Vec3 world_offset =
                world_offset_for(
                    motion_frame);

            const auto animated =
                character.sample(
                    clip,
                    motion_time_seconds,
                    true,
                    world_offset);

            auto voxel_centers =
                voxel_character.sample_centers(
                    character,
                    clip,
                    motion_time_seconds,
                    true,
                    world_offset);

            const auto thigh_split =
                character.sample_split_branch(
                    clip,
                    motion_time_seconds,
                    "thigh_l",
                    true,
                    world_offset);

            const auto calf_split =
                character.sample_split_branch(
                    clip,
                    motion_time_seconds,
                    "calf_l",
                    true,
                    world_offset);

            const auto foot_split =
                character.sample_split_branch(
                    clip,
                    motion_time_seconds,
                    "foot_l",
                    true,
                    world_offset);

            const sarx::Vec3 hip =
                nearest_anchor(
                    thigh_split.body,
                    thigh_split.detached);

            const sarx::Vec3 knee =
                nearest_anchor(
                    calf_split.body,
                    calf_split.detached);

            const sarx::Vec3 ankle =
                nearest_anchor(
                    foot_split.body,
                    foot_split.detached);

            const sarx::Vec3 foot_tip =
                farthest_used_point(
                    foot_split.detached,
                    ankle);

            const sarx::Vec3 cut_center =
                (hip + knee) * 0.5;

            const sarx::Vec3 thigh_axis =
                knee - hip;

            const sarx::Vec3 calf_seed =
                used_centroid(
                    calf_split.detached);

            if (!detached_leg
                && frame >= args.cut_frame
                && frame < args.cut_frame + 6) {

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        cut_center,
                        thigh_axis,
                        args.voxel_size * 0.78,
                        args.voxel_size * 4.2,
                        0.60,
                        {"thigh_l"});

                auto component =
                    voxel_character
                        .detach_component_near_anatomical(
                            voxel_centers,
                            calf_seed,
                            12);

                if (component) {
                    DetachedLeg leg;
                    leg.component =
                        std::move(*component);

                    const double previous_seconds =
                        std::max(
                            0.0,
                            motion_time_seconds - dt);

                    const int previous_frame =
                        std::max(
                            0,
                            motion_frame - 1);

                    const sarx::Vec3 previous_offset =
                        world_offset_for(
                            previous_frame);

                    const auto previous_thigh_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            "thigh_l",
                            true,
                            previous_offset);

                    const auto previous_calf_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            "calf_l",
                            true,
                            previous_offset);

                    const auto previous_foot_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            "foot_l",
                            true,
                            previous_offset);

                    const sarx::Vec3 previous_hip =
                        nearest_anchor(
                            previous_thigh_split.body,
                            previous_thigh_split.detached);

                    const sarx::Vec3 previous_knee =
                        nearest_anchor(
                            previous_calf_split.body,
                            previous_calf_split.detached);

                    const sarx::Vec3 previous_ankle =
                        nearest_anchor(
                            previous_foot_split.body,
                            previous_foot_split.detached);

                    const sarx::Vec3 previous_foot_tip =
                        farthest_used_point(
                            previous_foot_split.detached,
                            previous_ankle);

                    const sarx::Vec3 previous_cut_center =
                        (previous_hip
                         + previous_knee)
                        * 0.5;

                    leg.rest_centers.reserve(
                        leg.component
                            .voxel_indices
                            .size());

                    leg.segment_by_voxel.reserve(
                        leg.component
                            .voxel_indices
                            .size());

                    for (const std::size_t index
                         : leg.component
                               .voxel_indices) {

                        leg.rest_centers.push_back(
                            voxel_character
                                .voxel_center(
                                    index,
                                    voxel_centers));

                        const std::string& region =
                            voxel_character
                                .voxels()[index]
                                .anatomical_region;

                        if (region == "thigh_l") {
                            leg.segment_by_voxel.push_back(0);
                        } else if (region == "calf_l") {
                            leg.segment_by_voxel.push_back(1);
                        } else if (region == "foot_l") {
                            leg.segment_by_voxel.push_back(2);
                        } else {
                            throw std::runtime_error(
                                "detached leg contains non-leg anatomical region: "
                                + region);
                        }
                    }

                    const double ankle_rest_angle =
                        sarx::articulated_joint_angle(
                            knee,
                            ankle,
                            foot_tip);

                    sarx::DetachedArticulationConfig articulation;
                    articulation.rest_anchors = {
                        cut_center,
                        knee,
                        ankle,
                        foot_tip
                    };

                    articulation.previous_anchors = {
                        previous_cut_center,
                        previous_knee,
                        previous_ankle,
                        previous_foot_tip
                    };

                    articulation.masses = {
                        0.30,
                        0.30,
                        0.22,
                        0.18
                    };

                    articulation.joints = {
                        sarx::PassiveJointProfile{
                            1,
                            0.35,
                            3.10,
                            0.085,
                            0.70,
                            0.20
                        },
                        sarx::PassiveJointProfile{
                            2,
                            std::max(
                                0.65,
                                ankle_rest_angle - 0.55),
                            std::min(
                                3.10,
                                ankle_rest_angle + 0.55),
                            0.10,
                            0.72,
                            0.24
                        }
                    };

                    articulation.ground_radius =
                        args.voxel_size * 0.47;

                    articulation.restitution = 0.06;
                    articulation.tangential_damping = 0.58;
                    articulation.contact_velocity_scale = 0.20;
                    articulation.global_velocity_damping = 0.992;
                    articulation.contact_iterations = 10;
                    articulation.substeps = 8;
                    articulation.solver_iterations = 16;

                    leg.articulation.initialize(
                        articulation,
                        dt);

                    detached_frame = frame;
                    detached_leg =
                        std::move(leg);
                }
            }

            const auto viability =
                sarx::evaluate_motion_viability(
                    character.animation_names()[clip],
                    voxel_character.anatomy_availability());

            if (viability.state
                    == sarx::MotionViability::Invalid
                && normal_walk_authority) {

                normal_walk_authority = false;
                walk_invalidated_frame = frame;

                const int previous_frame =
                    std::max(
                        0,
                        frame - 1);

                inherited_root_velocity =
                    (world_offset
                     - world_offset_for(
                         previous_frame))
                    / dt;

                sarx::MotionPhysicalState
                    physical_state;

                physical_state.root_velocity =
                    inherited_root_velocity;
                physical_state.grounded = true;
                physical_state.airborne = false;
                physical_state.support_contacts = 1;

                recovery_plan =
                    sarx::plan_motion_recovery(
                        sarx::BehavioralIntent::MoveForward,
                        character.animation_names()[clip],
                        character.animation_names(),
                        voxel_character.anatomy_availability(),
                        physical_state);

                if (recovery_plan.transition_required
                    && recovery_plan.strategy
                        == sarx::MotionStrategy::Fall
                    && !recovery_plan.procedural) {

                    recovery_clip =
                        character.find_animation(
                            recovery_plan.motion_id);

                    recovery_clip_duration =
                        character.animation_duration(
                            *recovery_clip);

                    recovery_source_centers =
                        voxel_centers;

                    recovery_start_world_offset =
                        world_offset;

                    recovery_world_offset =
                        world_offset;

                    recovery_root_velocity =
                        inherited_root_velocity;

                    recovery_selected_frame =
                        frame;
                }
            }

            sarx::Vec3 body_world_offset =
                world_offset;

            if (recovery_clip
                && !grounded_recovery_clip
                && frame > recovery_selected_frame) {

                const sarx::Vec3 previous_offset =
                    recovery_world_offset;

                // Preserve the exact pre-failure root velocity on the
                // first recovery step. Damping begins only after that
                // step so the transition cannot teleport or shed
                // momentum at the authority handoff.
                recovery_world_offset +=
                    recovery_root_velocity * dt;

                if (frame
                    == recovery_selected_frame + 1) {

                    const sarx::Vec3 realized_velocity =
                        (recovery_world_offset
                         - previous_offset)
                        / dt;

                    root_velocity_discontinuity =
                        sarx::length(
                            realized_velocity
                            - inherited_root_velocity);
                }

                const double horizontal_damping =
                    std::exp(
                        -1.10 * dt);

                recovery_root_velocity.x *=
                    horizontal_damping;

                recovery_root_velocity.z *=
                    horizontal_damping;

                recovery_time_seconds =
                    std::min(
                        recovery_clip_duration,
                        recovery_time_seconds + dt);

                auto recovery_centers =
                    voxel_character.sample_centers(
                        character,
                        *recovery_clip,
                        recovery_time_seconds,
                        false,
                        recovery_world_offset);

                const double raw_blend =
                    std::clamp(
                        recovery_time_seconds / 0.24,
                        0.0,
                        1.0);

                const double blend =
                    raw_blend
                    * raw_blend
                    * (3.0 - 2.0 * raw_blend);

                double pose_error_squared = 0.0;
                std::size_t pose_count = 0;

                sarx::Vec3 source_centroid{};
                sarx::Vec3 current_centroid{};
                std::size_t centroid_count = 0;

                const sarx::Vec3 world_delta =
                    recovery_world_offset
                    - recovery_start_world_offset;

                for (std::size_t i = 0;
                     i < voxel_centers.size();
                     ++i) {

                    const sarx::Vec3 carried_source =
                        recovery_source_centers[i]
                        + world_delta;

                    voxel_centers[i] =
                        carried_source
                        * (1.0 - blend)
                        + recovery_centers[i]
                        * blend;

                    if (voxel_character
                            .voxels()[i]
                            .state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    const sarx::Vec3 local_source =
                        recovery_source_centers[i]
                        - recovery_start_world_offset;

                    const sarx::Vec3 local_current =
                        voxel_centers[i]
                        - recovery_world_offset;

                    const sarx::Vec3 delta =
                        local_current
                        - local_source;

                    pose_error_squared +=
                        sarx::length_squared(
                            delta);

                    ++pose_count;
                    source_centroid +=
                        local_source;
                    current_centroid +=
                        local_current;
                    ++centroid_count;
                }

                double attached_min_y =
                    std::numeric_limits<double>::infinity();

                for (std::size_t i = 0;
                     i < voxel_centers.size();
                     ++i) {

                    if (voxel_character
                            .voxels()[i]
                            .state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    attached_min_y =
                        std::min(
                            attached_min_y,
                            voxel_centers[i].y);
                }

                const double floor_center_y =
                    args.voxel_size * 0.50;

                const double floor_projection =
                    std::isfinite(attached_min_y)
                    ? std::max(
                        0.0,
                        floor_center_y
                            - attached_min_y)
                    : 0.0;

                if (floor_projection > 0.0) {
                    for (std::size_t i = 0;
                         i < voxel_centers.size();
                         ++i) {

                        if (voxel_character
                                .voxels()[i]
                                .state
                            != sarx::CharacterVoxelState::Attached) {
                            continue;
                        }

                        voxel_centers[i].y +=
                            floor_projection;
                    }

                    current_centroid.y +=
                        floor_projection
                        * static_cast<double>(
                            centroid_count);

                    max_fall_floor_projection =
                        std::max(
                            max_fall_floor_projection,
                            floor_projection);
                }

                if (pose_count > 0) {
                    max_fall_pose_rms =
                        std::max(
                            max_fall_pose_rms,
                            std::sqrt(
                                pose_error_squared
                                / static_cast<double>(
                                    pose_count)));
                }

                if (centroid_count > 0) {
                    source_centroid =
                        source_centroid
                        / static_cast<double>(
                            centroid_count);

                    current_centroid =
                        current_centroid
                        / static_cast<double>(
                            centroid_count);

                    fall_centroid_drop =
                        std::max(
                            fall_centroid_drop,
                            source_centroid.y
                            - current_centroid.y);
                }

                body_world_offset =
                    recovery_world_offset;

                if (recovery_time_seconds
                        >= recovery_clip_duration
                    && recovery_completed_frame < 0) {

                    recovery_completed_frame =
                        frame;
                }
            }

            if (recovery_completed_frame >= 0
                && !grounded_recovery_planned) {

                for (std::size_t i = 0;
                     i < voxel_centers.size();
                     ++i) {

                    if (voxel_character
                            .voxels()[i]
                            .state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    fall_body_min_y =
                        std::min(
                            fall_body_min_y,
                            voxel_centers[i].y);

                    if (voxel_centers[i].y
                        <= args.voxel_size * 0.80) {
                        ++fall_body_ground_contacts;
                    }
                }

                sarx::MotionPhysicalState
                    grounded_physical_state;

                grounded_physical_state.root_velocity =
                    recovery_root_velocity;

                grounded_physical_state.grounded =
                    fall_body_ground_contacts > 0
                    || fall_body_min_y
                        <= args.voxel_size * 1.20;

                grounded_physical_state.airborne =
                    !grounded_physical_state.grounded;

                grounded_physical_state.support_contacts =
                    fall_body_ground_contacts;

                grounded_recovery_plan =
                    sarx::plan_grounded_recovery(
                        sarx::BehavioralIntent::MoveForward,
                        sarx::GroundedPosture::Prone,
                        character.animation_names(),
                        voxel_character.anatomy_availability(),
                        grounded_physical_state);

                grounded_recovery_planned = true;

                if (grounded_recovery_plan.transition_required
                    && grounded_recovery_plan.strategy
                        == sarx::MotionStrategy::Kneel
                    && !grounded_recovery_plan.procedural) {

                    grounded_recovery_clip =
                        character.find_animation(
                            grounded_recovery_plan.motion_id);

                    grounded_recovery_source_centers =
                        voxel_centers;

                    grounded_recovery_start_world_offset =
                        body_world_offset;

                    grounded_recovery_world_offset =
                        body_world_offset;

                    grounded_inherited_root_velocity =
                        recovery_root_velocity;

                    grounded_recovery_root_velocity =
                        grounded_inherited_root_velocity;

                    grounded_recovery_selected_frame =
                        frame;
                }
            }

            if (grounded_recovery_clip
                && frame
                    > grounded_recovery_selected_frame) {

                const sarx::Vec3 previous_offset =
                    grounded_recovery_world_offset;

                grounded_recovery_world_offset +=
                    grounded_recovery_root_velocity
                    * dt;

                if (frame
                    == grounded_recovery_selected_frame + 1) {

                    const sarx::Vec3 realized_velocity =
                        (grounded_recovery_world_offset
                         - previous_offset)
                        / dt;

                    grounded_velocity_discontinuity =
                        sarx::length(
                            realized_velocity
                            - grounded_inherited_root_velocity);
                }

                const double settle_damping =
                    std::exp(
                        -4.0 * dt);

                grounded_recovery_root_velocity.x *=
                    settle_damping;

                grounded_recovery_root_velocity.z *=
                    settle_damping;

                grounded_recovery_time_seconds +=
                    dt;

                auto kneeling_centers =
                    voxel_character.sample_centers(
                        character,
                        *grounded_recovery_clip,
                        grounded_recovery_time_seconds,
                        true,
                        grounded_recovery_world_offset);

                const double raw_blend =
                    std::clamp(
                        grounded_recovery_time_seconds
                            / 0.38,
                        0.0,
                        1.0);

                const double blend =
                    raw_blend
                    * raw_blend
                    * (3.0 - 2.0 * raw_blend);

                const sarx::Vec3 world_delta =
                    grounded_recovery_world_offset
                    - grounded_recovery_start_world_offset;

                double pose_error_squared = 0.0;
                std::size_t pose_count = 0;

                sarx::Vec3 source_centroid{};
                sarx::Vec3 current_centroid{};
                std::size_t centroid_count = 0;

                for (std::size_t i = 0;
                     i < voxel_centers.size();
                     ++i) {

                    const sarx::Vec3 carried_source =
                        grounded_recovery_source_centers[i]
                        + world_delta;

                    voxel_centers[i] =
                        carried_source
                        * (1.0 - blend)
                        + kneeling_centers[i]
                        * blend;

                    if (voxel_character
                            .voxels()[i]
                            .state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    const sarx::Vec3 local_source =
                        grounded_recovery_source_centers[i]
                        - grounded_recovery_start_world_offset;

                    const sarx::Vec3 local_current =
                        voxel_centers[i]
                        - grounded_recovery_world_offset;

                    const sarx::Vec3 delta =
                        local_current
                        - local_source;

                    pose_error_squared +=
                        sarx::length_squared(
                            delta);

                    ++pose_count;
                    source_centroid +=
                        local_source;
                    current_centroid +=
                        local_current;
                    ++centroid_count;
                }

                double attached_min_y =
                    std::numeric_limits<double>::infinity();

                for (std::size_t i = 0;
                     i < voxel_centers.size();
                     ++i) {

                    if (voxel_character
                            .voxels()[i]
                            .state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    attached_min_y =
                        std::min(
                            attached_min_y,
                            voxel_centers[i].y);
                }

                const double floor_center_y =
                    args.voxel_size * 0.50;

                const double floor_projection =
                    std::isfinite(attached_min_y)
                    ? std::max(
                        0.0,
                        floor_center_y
                            - attached_min_y)
                    : 0.0;

                if (floor_projection > 0.0) {
                    for (std::size_t i = 0;
                         i < voxel_centers.size();
                         ++i) {

                        if (voxel_character
                                .voxels()[i]
                                .state
                            != sarx::CharacterVoxelState::Attached) {
                            continue;
                        }

                        voxel_centers[i].y +=
                            floor_projection;
                    }

                    current_centroid.y +=
                        floor_projection
                        * static_cast<double>(
                            centroid_count);

                    max_grounded_floor_projection =
                        std::max(
                            max_grounded_floor_projection,
                            floor_projection);
                }

                if (pose_count > 0) {
                    max_grounded_pose_rms =
                        std::max(
                            max_grounded_pose_rms,
                            std::sqrt(
                                pose_error_squared
                                / static_cast<double>(
                                    pose_count)));
                }

                if (centroid_count > 0) {
                    source_centroid =
                        source_centroid
                        / static_cast<double>(
                            centroid_count);

                    current_centroid =
                        current_centroid
                        / static_cast<double>(
                            centroid_count);

                    grounded_centroid_rise =
                        std::max(
                            grounded_centroid_rise,
                            current_centroid.y
                            - source_centroid.y);
                }

                if (raw_blend >= 1.0
                    && grounded_pose_engaged_frame < 0) {

                    grounded_pose_engaged_frame =
                        frame;
                }

                body_world_offset =
                    grounded_recovery_world_offset;
            }

            if (detached_leg
                && frame > detached_frame) {

                detached_leg
                    ->articulation
                    .step(dt);
            }

            sarx::CharacterMeshFrame visible =
                voxel_character.render(
                    voxel_centers);

            std::vector<sarx::Vec3>
                detached_centers;

            if (detached_leg) {
                detached_centers =
                    detached_world_centers(
                        *detached_leg);

                visible =
                    combine(
                        visible,
                        voxel_character
                            .render_component(
                                detached_leg
                                    ->component,
                                detached_centers));
            }

            // Evidence camera follows only the character's deliberate
            // world translation. Animated hip/knee motion, footsteps,
            // the wound, and the detached leg never feed back into the
            // camera transform, so gait motion cannot create camera shake.
            sarx::CharacterRenderCamera camera;

            camera.target =
                character_center
                + body_world_offset
                + sarx::Vec3{
                    0.0,
                    -scale * 0.12,
                    0.0
                };

            camera.position =
                camera.target
                + sarx::Vec3{
                    scale * 0.88,
                    scale * 0.22,
                    scale * 1.78
                };

            camera.vertical_fov_degrees =
                34.0;

            camera.width = 960;
            camera.height = 720;

            if (have_previous_camera) {
                const sarx::Vec3 camera_delta =
                    camera.target
                    - previous_camera_target;

                const sarx::Vec3 world_delta =
                    body_world_offset
                    - previous_camera_world_offset;

                max_camera_tracking_error =
                    std::max(
                        max_camera_tracking_error,
                        sarx::length(
                            camera_delta
                            - world_delta));
            }

            previous_camera_target =
                camera.target;

            previous_camera_world_offset =
                body_world_offset;

            have_previous_camera = true;

            sarx::write_character_ppm(
                frame_path(
                    args.output,
                    frame),
                visible,
                camera,
                true);
        }

        const auto stats =
            voxel_character.stats();

        std::size_t unrelated_changed_voxels = 0;

        for (const auto& voxel
             : voxel_character.voxels()) {

            if (voxel.state
                    == sarx::CharacterVoxelState::Attached
                || voxel.anatomical_region == "thigh_l"
                || voxel.anatomical_region == "calf_l"
                || voxel.anatomical_region == "foot_l") {
                continue;
            }

            ++unrelated_changed_voxels;
        }

        const auto final_viability =
            sarx::evaluate_motion_viability(
                character.animation_names()[clip],
                voxel_character.anatomy_availability());

        if (args.require_damage
            && destroyed_total == 0) {
            throw std::runtime_error(
                "left thigh cut destroyed no voxels");
        }

        if (args.require_detachment
            && !detached_leg) {
            throw std::runtime_error(
                "left thigh cut never detached the distal leg");
        }

        if (args.require_ground_contact
            && (!detached_leg
                || !detached_leg
                    ->articulation
                    .ever_grounded())) {
            throw std::runtime_error(
                "detached distal leg never hit the floor");
        }

        if (args.require_walk_invalidation
            && (walk_invalidated_frame < 0
                || final_viability.state
                    != sarx::MotionViability::Invalid)) {
            throw std::runtime_error(
                "normal Walk was not invalidated by left thigh loss");
        }

        if (args.require_anatomical_isolation
            && unrelated_changed_voxels != 0) {
            throw std::runtime_error(
                "left thigh cut altered unrelated anatomy: "
                + std::to_string(
                    unrelated_changed_voxels));
        }

        if (args.require_stable_camera
            && max_camera_tracking_error > 1e-9) {
            throw std::runtime_error(
                "thigh evidence camera inherited non-locomotion motion: "
                + std::to_string(
                    max_camera_tracking_error));
        }

        if (args.require_articulated_leg
            && detached_leg
            && detached_leg
                   ->articulation
                   .max_joint_angle_delta(0) < 0.08
            && detached_leg
                   ->articulation
                   .max_joint_angle_delta(1) < 0.08) {
            throw std::runtime_error(
                "detached voxel leg remained rigid after severance");
        }

        if (args.require_fall_transition
            && (recovery_selected_frame < 0
                || recovery_plan.strategy
                    != sarx::MotionStrategy::Fall
                || recovery_plan.procedural
                || recovery_plan.motion_id.empty())) {
            throw std::runtime_error(
                "invalid Walk did not select an authored fall recovery");
        }

        if (args.require_fall_transition
            && root_velocity_discontinuity > 1e-9) {
            throw std::runtime_error(
                "fall transition failed to inherit root momentum: "
                + std::to_string(
                    root_velocity_discontinuity));
        }

        if (args.require_fall_transition
            && max_fall_pose_rms
                < args.voxel_size * 1.5) {
            throw std::runtime_error(
                "fall transition did not visibly leave the frozen Walk pose");
        }

        if (args.require_grounded_recovery
            && (!grounded_recovery_planned
                || fall_body_ground_contacts == 0)) {
            throw std::runtime_error(
                "fall did not establish measured attached-body ground contact");
        }

        if (args.require_grounded_recovery
            && fall_body_min_y
                < args.voxel_size * 0.49) {
            throw std::runtime_error(
                "attached body penetrated the floor at grounded recovery: "
                + std::to_string(
                    fall_body_min_y));
        }

        if (args.require_grounded_recovery
            && (grounded_recovery_selected_frame < 0
                || grounded_recovery_plan.strategy
                    != sarx::MotionStrategy::Kneel
                || grounded_recovery_plan.target_posture
                    != sarx::GroundedPosture::Kneeling
                || grounded_recovery_plan.procedural
                || grounded_recovery_plan.motion_id.empty())) {
            throw std::runtime_error(
                "grounded recovery did not select authored kneeling");
        }

        if (args.require_grounded_recovery
            && grounded_recovery_selected_frame
                < recovery_completed_frame) {
            throw std::runtime_error(
                "grounded recovery started before the fall completed");
        }

        if (args.require_grounded_recovery
            && grounded_velocity_discontinuity > 1e-9) {
            throw std::runtime_error(
                "grounded recovery failed to preserve handoff velocity: "
                + std::to_string(
                    grounded_velocity_discontinuity));
        }

        if (args.require_grounded_recovery
            && (grounded_pose_engaged_frame < 0
                || max_grounded_pose_rms
                    < args.voxel_size * 1.5)) {
            throw std::runtime_error(
                "grounded recovery never visibly left the prone fall pose");
        }

        std::cout
            << "SARX Quaternius voxel thigh cut demo complete:"
            << " total_voxels="
            << stats.total_voxels
            << " destroyed_voxels="
            << stats.destroyed_voxels
            << " detached_voxels="
            << stats.detached_voxels
            << " attached_voxels="
            << stats.attached_voxels
            << " thigh_cut_voxels="
            << destroyed_total
            << " detached_frame="
            << detached_frame
            << " walk_invalidated_frame="
            << walk_invalidated_frame
            << " recovery_selected_frame="
            << recovery_selected_frame
            << " recovery_completed_frame="
            << recovery_completed_frame
            << " recovery_strategy="
            << sarx::motion_strategy_name(
                recovery_plan.strategy)
            << " recovery_motion="
            << (recovery_plan.motion_id.empty()
                ? std::string{"none"}
                : recovery_plan.motion_id)
            << " recovery_procedural="
            << (recovery_plan.procedural ? 1 : 0)
            << " root_velocity_discontinuity="
            << root_velocity_discontinuity
            << " fall_centroid_drop="
            << fall_centroid_drop
            << " fall_pose_rms="
            << max_fall_pose_rms
            << " max_fall_floor_projection="
            << max_fall_floor_projection
            << " fall_body_ground_contacts="
            << fall_body_ground_contacts
            << " fall_body_min_y="
            << fall_body_min_y
            << " grounded_recovery_selected_frame="
            << grounded_recovery_selected_frame
            << " grounded_pose_engaged_frame="
            << grounded_pose_engaged_frame
            << " grounded_strategy="
            << sarx::motion_strategy_name(
                grounded_recovery_plan.strategy)
            << " grounded_motion="
            << (grounded_recovery_plan.motion_id.empty()
                ? std::string{"none"}
                : grounded_recovery_plan.motion_id)
            << " grounded_procedural="
            << (grounded_recovery_plan.procedural ? 1 : 0)
            << " grounded_velocity_discontinuity="
            << grounded_velocity_discontinuity
            << " grounded_pose_rms="
            << max_grounded_pose_rms
            << " max_grounded_floor_projection="
            << max_grounded_floor_projection
            << " grounded_centroid_rise="
            << grounded_centroid_rise
            << " grounded_followup_options="
            << grounded_recovery_plan
                   .followup_locomotion_options
                   .size()
            << " left_thigh_attached_fraction="
            << voxel_character.attached_fraction(
                "thigh_l")
            << " left_calf_attached_fraction="
            << voxel_character.attached_fraction(
                "calf_l")
            << " left_foot_attached_fraction="
            << voxel_character.attached_fraction(
                "foot_l")
            << " unrelated_changed_voxels="
            << unrelated_changed_voxels
            << " leg_ground_contacts="
            << (detached_leg
                ? detached_leg
                    ->articulation
                    .ground_contacts()
                : 0)
            << " knee_delta_rad="
            << (detached_leg
                ? detached_leg
                    ->articulation
                    .max_joint_angle_delta(0)
                : 0.0)
            << " ankle_delta_rad="
            << (detached_leg
                ? detached_leg
                    ->articulation
                    .max_joint_angle_delta(1)
                : 0.0)
            << " leg_max_rotation_rad="
            << (detached_leg
                ? std::max({
                    detached_leg
                        ->articulation
                        .segment_rotation_radians(0),
                    detached_leg
                        ->articulation
                        .segment_rotation_radians(1),
                    detached_leg
                        ->articulation
                        .segment_rotation_radians(2)})
                : 0.0)
            << " max_camera_tracking_error="
            << max_camera_tracking_error
            << " voxel_size="
            << stats.voxel_size
            << " frames="
            << args.frames
            << " output="
            << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_voxel_thigh_demo: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}
