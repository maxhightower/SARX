#include "sarx/character_render.hpp"
#include "sarx/detached_articulation.hpp"
#include "sarx/gltf_character.hpp"
#include "sarx/motion_recovery.hpp"
#include "sarx/motion_viability.hpp"
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
    std::string injury_animations{"assets/cmu/CMU_HurtLegWalk.glb"};
    std::string injury_clip{"CMU_HurtLegWalk"};
    std::filesystem::path output{
        "media/raw/v13_quaternius_foot_authored_injury_frames"};
    int frames{240};
    int cut_frame{75};
    double fps{30.0};
    double voxel_size{0.045};
    bool require_damage{false};
    bool require_detachment{false};
    bool require_walk_invalidation{false};
    bool require_limp{false};
    bool require_isolated_foot{false};
    bool require_ground_contact{false};
    bool require_stable_camera{false};
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
        } else if (value == "--injury-animations" && i + 1 < argc) {
            args.injury_animations = argv[++i];
        } else if (value == "--injury-clip" && i + 1 < argc) {
            args.injury_clip = argv[++i];
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
        } else if (value == "--require-walk-invalidation") {
            args.require_walk_invalidation = true;
        } else if (value == "--require-limp") {
            args.require_limp = true;
        } else if (value == "--require-isolated-foot") {
            args.require_isolated_foot = true;
        } else if (value == "--require-ground-contact") {
            args.require_ground_contact = true;
        } else if (value == "--require-stable-camera") {
            args.require_stable_camera = true;
        } else if (value == "--help") {
            std::cout
                << "sarx_character_voxel_foot_injury_demo"
                << " [--injury-animations FILE]"
                << " [--injury-clip NAME]"
                << " [--output DIR]"
                << " [--frames N]"
                << " [--cut-frame N]"
                << " [--fps N]"
                << " [--voxel-size N]"
                << " [--require-damage]"
                << " [--require-detachment]"
                << " [--require-walk-invalidation]"
                << " [--require-limp]"
                << " [--require-isolated-foot]"
                << " [--require-ground-contact]"
                << " [--require-stable-camera]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete foot limp demo argument");
        }
    }

    if (args.frames <= 0
        || args.cut_frame < 1
        || args.cut_frame >= args.frames
        || args.fps <= 0.0
        || args.voxel_size <= 0.0) {
        throw std::invalid_argument(
            "invalid foot limp demo settings");
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
        bounds.min.x = std::min(bounds.min.x, point.x);
        bounds.min.y = std::min(bounds.min.y, point.y);
        bounds.min.z = std::min(bounds.min.z, point.z);
        bounds.max.x = std::max(bounds.max.x, point.x);
        bounds.max.y = std::max(bounds.max.y, point.y);
        bounds.max.z = std::max(bounds.max.z, point.z);
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

    sarx::Vec3 result{};
    std::size_t count = 0;

    for (std::size_t i = 0;
         i < used.size();
         ++i) {

        if (!used[i]) {
            continue;
        }

        result += frame.positions[i];
        ++count;
    }

    if (count == 0) {
        throw std::runtime_error(
            "branch owns no vertices");
    }

    return result
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
                body_point = body.positions[i];
                detached_point =
                    detached.positions[j];
            }
        }
    }

    if (!std::isfinite(best)) {
        throw std::runtime_error(
            "could not infer anatomical anchor");
    }

    return (body_point + detached_point)
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

    out.indices.reserve(
        out.indices.size()
        + b.indices.size());

    for (const auto index : b.indices) {
        out.indices.push_back(
            base + index);
    }

    return out;
}

std::pair<std::size_t, std::size_t>
farthest_pair(
    const std::vector<sarx::Vec3>& points) {

    if (points.size() < 2) {
        throw std::runtime_error(
            "detached foot needs at least two voxels");
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

struct DetachedFoot {
    sarx::DetachedVoxelComponent component;
    std::vector<sarx::Vec3> rest_centers;
    sarx::DetachedArticulatedChain articulation;
};

std::vector<sarx::Vec3>
detached_world_centers(
    const DetachedFoot& foot) {

    std::vector<sarx::Vec3> centers;
    centers.reserve(
        foot.rest_centers.size());

    for (const auto& point
         : foot.rest_centers) {

        centers.push_back(
            foot.articulation.transform_point(
                0,
                point));
    }

    return centers;
}

double attached_pose_rms(
    const std::vector<sarx::CharacterVoxel>& voxels,
    const std::vector<sarx::Vec3>& a,
    const std::vector<sarx::Vec3>& b) {

    double squared = 0.0;
    std::size_t count = 0;

    for (std::size_t i = 0;
         i < voxels.size();
         ++i) {

        if (voxels[i].state
            != sarx::CharacterVoxelState::Attached) {
            continue;
        }

        squared +=
            sarx::length_squared(
                a[i] - b[i]);
        ++count;
    }

    return count > 0
        ? std::sqrt(
            squared
            / static_cast<double>(count))
        : 0.0;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args =
            parse_args(argc, argv);

        std::filesystem::create_directories(
            args.output);

        sarx::GltfCharacter walk_character;
        walk_character.load(
            args.character,
            args.animations);

        sarx::GltfCharacter injury_character;
        injury_character.load(
            args.character,
            args.injury_animations);

        const std::size_t clip =
            walk_character.find_animation(
                args.clip);

        const std::size_t injury_clip =
            injury_character.find_animation(
                args.injury_clip);

        const auto rest =
            walk_character.sample(
                clip,
                0.0,
                true);

        sarx::VoxelizedCharacter voxel_character;
        voxel_character.build(
            walk_character,
            clip,
            0.0,
            args.voxel_size);

        const Bounds bounds =
            bounds_of(rest);

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
                height * 1.35,
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
        int injury_selected_frame = -1;
        int authored_pose_engaged_frame = -1;

        bool normal_walk_authority = true;

        sarx::MotionRecoveryPlan
            locomotion_plan;

        std::optional<DetachedFoot>
            detached_foot;

        std::vector<sarx::Vec3>
            transition_source_centers;

        sarx::Vec3 transition_world_offset{};
        sarx::Vec3 transition_core_center{};
        sarx::Vec3 injury_initial_core_center{};

        double max_authored_pose_rms = 0.0;
        double max_floor_projection = 0.0;
        double max_camera_tracking_error = 0.0;
        double min_head_above_pelvis =
            std::numeric_limits<double>::infinity();

        std::size_t right_support_contacts = 0;
        std::size_t authored_evaluated_frames = 0;
        std::size_t authored_grounded_frames = 0;

        sarx::Vec3 previous_camera_target{};
        sarx::Vec3 previous_camera_world_offset{};
        bool have_previous_camera = false;

        for (int frame = 0;
             frame < args.frames;
             ++frame) {

            const double motion_time_seconds =
                static_cast<double>(frame)
                / args.fps;

            const sarx::Vec3 world_offset =
                world_offset_for(frame);

            auto voxel_centers =
                voxel_character.sample_centers(
                    walk_character,
                    clip,
                    motion_time_seconds,
                    true,
                    world_offset);

            const auto thigh_split =
                walk_character.sample_split_branch(
                    clip,
                    motion_time_seconds,
                    "thigh_l",
                    true,
                    world_offset);

            const auto calf_split =
                walk_character.sample_split_branch(
                    clip,
                    motion_time_seconds,
                    "calf_l",
                    true,
                    world_offset);

            const auto foot_split =
                walk_character.sample_split_branch(
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

            const sarx::Vec3 foot_center =
                used_centroid(
                    foot_split.detached);

            if (!detached_foot
                && frame >= args.cut_frame
                && frame < args.cut_frame + 10) {

                const sarx::Vec3 foot_axis =
                    foot_center - ankle;

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        ankle,
                        foot_axis,
                        args.voxel_size * 0.82,
                        args.voxel_size * 3.8,
                        0.62,
                        {"foot_l"});

                auto component =
                    voxel_character
                        .detach_component_near_anatomical(
                            voxel_centers,
                            foot_center,
                            6);

                if (component) {
                    for (const auto index
                         : component->voxel_indices) {

                        if (voxel_character
                                .voxels()[index]
                                .anatomical_region
                            != "foot_l") {

                            throw std::runtime_error(
                                "foot cut detached non-foot anatomy");
                        }
                    }

                    DetachedFoot foot;
                    foot.component =
                        std::move(*component);

                    const auto previous_centers =
                        voxel_character.sample_centers(
                            walk_character,
                            clip,
                            std::max(
                                0.0,
                                motion_time_seconds - dt),
                            true,
                            world_offset_for(
                                std::max(
                                    0,
                                    frame - 1)));

                    std::vector<sarx::Vec3>
                        previous_component_centers;

                    for (const auto index
                         : foot.component.voxel_indices) {

                        foot.rest_centers.push_back(
                            voxel_character.voxel_center(
                                index,
                                voxel_centers));

                        previous_component_centers.push_back(
                            voxel_character.voxel_center(
                                index,
                                previous_centers));
                    }

                    const auto [a, b] =
                        farthest_pair(
                            foot.rest_centers);

                    sarx::DetachedArticulationConfig
                        articulation;

                    articulation.rest_anchors = {
                        foot.rest_centers[a],
                        foot.rest_centers[b]
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
                    articulation.restitution = 0.06;
                    articulation.tangential_damping = 0.62;
                    articulation.contact_velocity_scale = 0.20;
                    articulation.global_velocity_damping = 0.993;
                    articulation.contact_iterations = 8;
                    articulation.substeps = 6;
                    articulation.solver_iterations = 14;

                    foot.articulation.initialize(
                        articulation,
                        dt);

                    detached_frame = frame;
                    detached_foot =
                        std::move(foot);
                }
            }

            if (detached_foot
                && frame > detached_frame) {

                detached_foot
                    ->articulation
                    .step(dt);
            }

            const auto walk_viability =
                sarx::evaluate_motion_viability(
                    walk_character.animation_names()[clip],
                    voxel_character.anatomy_availability());

            if (walk_viability.state
                    == sarx::MotionViability::Invalid
                && normal_walk_authority) {

                normal_walk_authority = false;
                walk_invalidated_frame = frame;

                sarx::MotionPhysicalState
                    physical_state;

                physical_state.root_velocity =
                    (world_offset_for(frame)
                     - world_offset_for(
                         std::max(0, frame - 1)))
                    / dt;

                physical_state.grounded = true;
                physical_state.airborne = false;
                physical_state.support_contacts = 1;

                locomotion_plan =
                    sarx::plan_authored_injury_locomotion(
                        sarx::BehavioralIntent::MoveForward,
                        walk_character.animation_names()[clip],
                        injury_character.animation_names(),
                        voxel_character.anatomy_availability(),
                        physical_state);

                if (locomotion_plan.strategy
                        == sarx::MotionStrategy::Limp
                    && locomotion_plan.motion_id
                        == injury_character
                            .animation_names()[injury_clip]) {

                    injury_selected_frame =
                        frame;

                    transition_source_centers =
                        voxel_centers;

                    transition_world_offset =
                        world_offset;

                    auto core_centroid =
                        [&](const std::vector<sarx::Vec3>& centers) {

                            sarx::Vec3 center{};
                            std::size_t count = 0;

                            for (std::size_t i = 0;
                                 i < centers.size();
                                 ++i) {

                                if (voxel_character
                                        .voxels()[i]
                                        .state
                                    != sarx::CharacterVoxelState::Attached) {
                                    continue;
                                }

                                const std::string& region =
                                    voxel_character
                                        .voxels()[i]
                                        .anatomical_region;

                                if (region != "pelvis"
                                    && region != "spine_01"
                                    && region != "spine_02"
                                    && region != "spine_03") {
                                    continue;
                                }

                                center += centers[i];
                                ++count;
                            }

                            if (count == 0) {
                                throw std::runtime_error(
                                    "authored injury transition has no attached core voxels");
                            }

                            return center
                                / static_cast<double>(count);
                        };

                    transition_core_center =
                        core_centroid(
                            transition_source_centers);

                    const auto injury_initial_centers =
                        voxel_character.sample_centers(
                            injury_character,
                            injury_clip,
                            0.0,
                            true,
                            world_offset);

                    injury_initial_core_center =
                        core_centroid(
                            injury_initial_centers);
                }
            }

            if (injury_selected_frame >= 0
                && frame > injury_selected_frame) {

                const double injury_time =
                    static_cast<double>(
                        frame - injury_selected_frame)
                    / args.fps;

                auto injury_centers =
                    voxel_character.sample_centers(
                        injury_character,
                        injury_clip,
                        injury_time,
                        true,
                        world_offset);

                auto core_centroid =
                    [&](const std::vector<sarx::Vec3>& centers) {

                        sarx::Vec3 center{};
                        std::size_t count = 0;

                        for (std::size_t i = 0;
                             i < centers.size();
                             ++i) {

                            if (voxel_character
                                    .voxels()[i]
                                    .state
                                != sarx::CharacterVoxelState::Attached) {
                                continue;
                            }

                            const std::string& region =
                                voxel_character
                                    .voxels()[i]
                                    .anatomical_region;

                            if (region != "pelvis"
                                && region != "spine_01"
                                && region != "spine_02"
                                && region != "spine_03") {
                                continue;
                            }

                            center += centers[i];
                            ++count;
                        }

                        if (count == 0) {
                            throw std::runtime_error(
                                "authored injury motion has no attached core voxels");
                        }

                        return center
                            / static_cast<double>(count);
                    };

                const sarx::Vec3 raw_core =
                    core_centroid(
                        injury_centers);

                const sarx::Vec3 world_delta =
                    world_offset
                    - transition_world_offset;

                const sarx::Vec3 desired_core{
                    transition_core_center.x
                        + world_delta.x,
                    raw_core.y
                        + (transition_core_center.y
                           - injury_initial_core_center.y),
                    transition_core_center.z
                        + world_delta.z
                };

                const sarx::Vec3 alignment =
                    desired_core - raw_core;

                for (auto& center : injury_centers) {
                    center += alignment;
                }

                const double raw_blend =
                    std::clamp(
                        injury_time / 0.30,
                        0.0,
                        1.0);

                const double blend =
                    raw_blend
                    * raw_blend
                    * (3.0 - 2.0 * raw_blend);

                double pose_error_squared = 0.0;
                std::size_t pose_count = 0;

                for (std::size_t i = 0;
                     i < voxel_centers.size();
                     ++i) {

                    const sarx::Vec3 carried_source =
                        transition_source_centers[i]
                        + world_delta;

                    voxel_centers[i] =
                        carried_source
                            * (1.0 - blend)
                        + injury_centers[i]
                            * blend;

                    if (voxel_character
                            .voxels()[i]
                            .state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    pose_error_squared +=
                        sarx::length_squared(
                            voxel_centers[i]
                            - carried_source);

                    ++pose_count;
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

                    max_floor_projection =
                        std::max(
                            max_floor_projection,
                            floor_projection);
                }

                if (pose_count > 0) {
                    max_authored_pose_rms =
                        std::max(
                            max_authored_pose_rms,
                            std::sqrt(
                                pose_error_squared
                                / static_cast<double>(
                                    pose_count)));
                }

                if (raw_blend >= 1.0
                    && authored_pose_engaged_frame < 0) {

                    authored_pose_engaged_frame =
                        frame;
                }
            }

            std::size_t
                right_support_contacts_this_frame = 0;

            for (std::size_t i = 0;
                 i < voxel_character.voxels().size();
                 ++i) {

                const auto& voxel =
                    voxel_character.voxels()[i];

                if (voxel.state
                        == sarx::CharacterVoxelState::Attached
                    && voxel.anatomical_region
                        == "foot_r"
                    && voxel_centers[i].y
                        <= args.voxel_size * 1.3) {

                    ++right_support_contacts;
                    ++right_support_contacts_this_frame;
                }
            }

            if (authored_pose_engaged_frame >= 0
                && frame >= authored_pose_engaged_frame) {

                ++authored_evaluated_frames;

                if (right_support_contacts_this_frame > 0) {
                    ++authored_grounded_frames;
                }

                sarx::Vec3 pelvis_center{};
                sarx::Vec3 head_center{};
                std::size_t pelvis_count = 0;
                std::size_t head_count = 0;

                for (std::size_t i = 0;
                     i < voxel_character.voxels().size();
                     ++i) {

                    const auto& voxel =
                        voxel_character.voxels()[i];

                    if (voxel.state
                        != sarx::CharacterVoxelState::Attached) {
                        continue;
                    }

                    if (voxel.anatomical_region
                        == "pelvis") {
                        pelvis_center += voxel_centers[i];
                        ++pelvis_count;
                    } else if (
                        voxel.anatomical_region
                        == "Head") {
                        head_center += voxel_centers[i];
                        ++head_count;
                    }
                }

                if (pelvis_count > 0
                    && head_count > 0) {

                    pelvis_center =
                        pelvis_center
                        / static_cast<double>(
                            pelvis_count);

                    head_center =
                        head_center
                        / static_cast<double>(
                            head_count);

                    min_head_above_pelvis =
                        std::min(
                            min_head_above_pelvis,
                            head_center.y
                            - pelvis_center.y);
                }
            }

            sarx::CharacterMeshFrame visible =
                voxel_character.render(
                    voxel_centers);

            if (detached_foot) {
                const auto detached_centers =
                    detached_world_centers(
                        *detached_foot);

                visible =
                    combine(
                        visible,
                        voxel_character
                            .render_component(
                                detached_foot->component,
                                detached_centers));
            }

            sarx::CharacterRenderCamera camera;

            camera.target =
                character_center
                + world_offset
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

            camera.vertical_fov_degrees = 34.0;
            camera.width = 960;
            camera.height = 720;

            if (have_previous_camera) {
                const auto camera_delta =
                    camera.target
                    - previous_camera_target;

                const auto world_delta =
                    world_offset
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
                world_offset;

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
                || voxel.anatomical_region
                    == "foot_l") {
                continue;
            }

            ++unrelated_changed_voxels;
        }

        if (args.require_damage
            && destroyed_total == 0) {
            throw std::runtime_error(
                "foot cut destroyed no voxels");
        }

        if (args.require_detachment
            && !detached_foot) {
            throw std::runtime_error(
                "foot cut never detached a component");
        }

        if (args.require_walk_invalidation
            && walk_invalidated_frame < 0) {
            throw std::runtime_error(
                "foot loss did not invalidate normal Walk");
        }

        if (args.require_limp
            && (injury_selected_frame < 0
                || authored_pose_engaged_frame < 0
                || locomotion_plan.strategy
                    != sarx::MotionStrategy::Limp
                || locomotion_plan.procedural
                || max_authored_pose_rms
                    < args.voxel_size * 0.40)) {
            throw std::runtime_error(
                "authored injury motion never became visible authority");
        }

        if (args.require_limp
            && (!std::isfinite(
                    min_head_above_pelvis)
                || min_head_above_pelvis < 0.20)) {
            throw std::runtime_error(
                "authored injury motion lost upright torso orientation: "
                + std::to_string(
                    min_head_above_pelvis));
        }

        if (args.require_limp
            && (authored_evaluated_frames == 0
                || authored_grounded_frames * 5
                    < authored_evaluated_frames)) {
            throw std::runtime_error(
                "authored injury motion lost sustained intact-foot support: "
                + std::to_string(
                    authored_grounded_frames)
                + "/"
                + std::to_string(
                    authored_evaluated_frames));
        }

        if (args.require_isolated_foot
            && unrelated_changed_voxels != 0) {
            throw std::runtime_error(
                "foot cut altered unrelated anatomy");
        }

        if (args.require_ground_contact
            && (!detached_foot
                || !detached_foot
                    ->articulation
                    .ever_grounded()
                || right_support_contacts == 0)) {
            throw std::runtime_error(
                "foot/remaining support never established ground contact");
        }

        if (args.require_stable_camera
            && max_camera_tracking_error > 1e-9) {
            throw std::runtime_error(
                "limp evidence camera inherited gait motion");
        }

        std::cout
            << "SARX Quaternius authored foot injury demo complete:"
            << " injury_motion="
            << injury_character.animation_names()[injury_clip]
            << " total_voxels="
            << stats.total_voxels
            << " destroyed_voxels="
            << stats.destroyed_voxels
            << " detached_voxels="
            << stats.detached_voxels
            << " detached_frame="
            << detached_frame
            << " walk_invalidated_frame="
            << walk_invalidated_frame
            << " injury_selected_frame="
            << injury_selected_frame
            << " authored_pose_engaged_frame="
            << authored_pose_engaged_frame
            << " locomotion_strategy="
            << sarx::motion_strategy_name(
                locomotion_plan.strategy)
            << " left_foot_attached_fraction="
            << voxel_character.attached_fraction(
                "foot_l")
            << " left_calf_attached_fraction="
            << voxel_character.attached_fraction(
                "calf_l")
            << " left_thigh_attached_fraction="
            << voxel_character.attached_fraction(
                "thigh_l")
            << " unrelated_changed_voxels="
            << unrelated_changed_voxels
            << " detached_foot_ground_contacts="
            << (detached_foot
                ? detached_foot
                    ->articulation
                    .ground_contacts()
                : 0)
            << " right_support_contacts="
            << right_support_contacts
            << " authored_grounded_frames="
            << authored_grounded_frames
            << " authored_evaluated_frames="
            << authored_evaluated_frames
            << " min_head_above_pelvis="
            << min_head_above_pelvis
            << " max_authored_pose_rms="
            << max_authored_pose_rms
            << " max_floor_projection="
            << max_floor_projection
            << " max_camera_tracking_error="
            << max_camera_tracking_error
            << " frames="
            << args.frames
            << " output="
            << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_voxel_foot_injury_demo: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}
