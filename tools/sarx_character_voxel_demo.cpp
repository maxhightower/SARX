#include "sarx/body.hpp"
#include "sarx/character_render.hpp"
#include "sarx/gltf_character.hpp"
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
    std::filesystem::path output{
        "media/raw/v08_quaternius_voxel_hand_detach_frames"};
    int frames{210};
    int damage_frame{75};
    double fps{30.0};
    double voxel_size{0.045};
    bool require_damage{false};
    bool require_detachment{false};
    bool require_ground_contact{false};
    bool require_anatomical_isolation{false};
    bool require_walk_viability{false};
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
        } else if (value == "--damage-frame" && i + 1 < argc) {
            args.damage_frame = std::stoi(argv[++i]);
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
        } else if (value == "--require-anatomical-isolation") {
            args.require_anatomical_isolation = true;
        } else if (value == "--require-walk-viability") {
            args.require_walk_viability = true;
        } else if (value == "--help") {
            std::cout
                << "sarx_character_voxel_demo"
                << " [--character FILE]"
                << " [--animations FILE]"
                << " [--clip NAME]"
                << " [--output DIR]"
                << " [--frames N]"
                << " [--damage-frame N]"
                << " [--fps N]"
                << " [--voxel-size N]"
                << " [--require-damage]"
                << " [--require-detachment]"
                << " [--require-ground-contact]"
                << " [--require-anatomical-isolation]"
                << " [--require-walk-viability]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete voxel demo argument");
        }
    }

    if (args.frames <= 0
        || args.fps <= 0.0
        || args.voxel_size <= 0.0
        || args.damage_frame < 1
        || args.damage_frame >= args.frames) {
        throw std::invalid_argument(
            "invalid voxel demo settings");
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
            "voxel hand target owns no vertices");
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
                body_point = body.positions[i];
                detached_point =
                    detached.positions[j];
            }
        }
    }

    if (!std::isfinite(best)) {
        throw std::runtime_error(
            "could not infer voxel wrist cut point");
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

    out.indices.reserve(
        out.indices.size()
        + b.indices.size());

    for (const auto index : b.indices) {
        out.indices.push_back(
            base + index);
    }

    return out;
}

struct DetachedHand {
    sarx::DetachedVoxelComponent component;
    std::vector<sarx::Vec3> rest_centers;

    sarx::Vec3 rest_a{};
    sarx::Vec3 rest_b{};

    sarx::Body motion;
    sarx::ParticleId particle_a{};
    sarx::ParticleId particle_b{};

    std::size_t ground_contacts{};
    bool ever_grounded{false};
    double max_rotation_radians{};
};

std::pair<std::size_t, std::size_t>
farthest_pair(
    const std::vector<sarx::Vec3>& points) {

    if (points.size() < 2) {
        throw std::runtime_error(
            "detached voxel hand needs at least two voxels");
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

std::vector<sarx::Vec3>
detached_world_centers(
    const DetachedHand& hand) {

    const sarx::Vec3 current_a =
        hand.motion
            .particles()[hand.particle_a]
            .position;

    const sarx::Vec3 current_b =
        hand.motion
            .particles()[hand.particle_b]
            .position;

    const sarx::Vec3 rest_axis =
        hand.rest_b - hand.rest_a;

    const sarx::Vec3 current_axis =
        current_b - current_a;

    std::vector<sarx::Vec3> centers;
    centers.reserve(
        hand.rest_centers.size());

    for (const sarx::Vec3& rest_center
         : hand.rest_centers) {

        centers.push_back(
            current_a
            + sarx::rotate_between(
                rest_axis,
                current_axis,
                rest_center - hand.rest_a));
    }

    return centers;
}

sarx::Vec3 centroid(
    const std::vector<sarx::Vec3>& points) {

    if (points.empty()) {
        return {};
    }

    sarx::Vec3 result{};

    for (const auto& point : points) {
        result += point;
    }

    return result
        / static_cast<double>(
            points.size());
}

double axis_rotation(
    const DetachedHand& hand) {

    const sarx::Vec3 rest_axis =
        sarx::normalized(
            hand.rest_b - hand.rest_a);

    const sarx::Vec3 current_axis =
        sarx::normalized(
            hand.motion
                .particles()[hand.particle_b]
                .position
            - hand.motion
                .particles()[hand.particle_a]
                .position);

    if (sarx::length_squared(rest_axis)
            <= 1e-12
        || sarx::length_squared(current_axis)
            <= 1e-12) {
        return 0.0;
    }

    return std::acos(
        std::clamp(
            sarx::dot(
                rest_axis,
                current_axis),
            -1.0,
            1.0));
}

void step_detached_hand(
    DetachedHand& hand,
    double dt,
    double voxel_size) {

    sarx::StepConfig config;
    config.substeps = 6;
    config.solver_iterations = 14;
    config.gravity = {0.0, -9.81, 0.0};

    hand.motion.step(
        dt,
        config);

    auto& a =
        hand.motion
            .particles()[hand.particle_a];

    auto& b =
        hand.motion
            .particles()[hand.particle_b];

    const sarx::Vec3 before_a =
        a.position;
    const sarx::Vec3 before_b =
        b.position;

    const sarx::Vec3 rest_axis =
        hand.rest_b - hand.rest_a;

    const double rest_axis_squared =
        std::max(
            sarx::length_squared(
                rest_axis),
            1e-12);

    const double ground_height =
        voxel_size * 0.47;

    bool contacted = false;

    for (int iteration = 0;
         iteration < 8;
         ++iteration) {

        const sarx::Vec3 current_axis =
            b.position - a.position;

        for (const sarx::Vec3& rest_center
             : hand.rest_centers) {

            const sarx::Vec3 world_center =
                a.position
                + sarx::rotate_between(
                    rest_axis,
                    current_axis,
                    rest_center - hand.rest_a);

            if (world_center.y
                >= ground_height) {
                continue;
            }

            const double penetration =
                ground_height
                - world_center.y;

            const double t =
                std::clamp(
                    sarx::dot(
                        rest_center - hand.rest_a,
                        rest_axis)
                    / rest_axis_squared,
                    0.0,
                    1.0);

            const double wa =
                1.0 - t;
            const double wb = t;

            const double denominator =
                std::max(
                    wa * wa + wb * wb,
                    1e-12);

            a.position.y +=
                penetration * wa
                / denominator;

            b.position.y +=
                penetration * wb
                / denominator;

            contacted = true;
        }
    }

    if (contacted) {
        a.velocity +=
            (a.position - before_a)
            / dt
            * 0.20;

        b.velocity +=
            (b.position - before_b)
            / dt
            * 0.20;

        constexpr double restitution = 0.08;
        constexpr double friction = 0.64;

        for (auto* particle : {&a, &b}) {
            if (particle->velocity.y < 0.0) {
                particle->velocity.y =
                    -particle->velocity.y
                    * restitution;
            }

            particle->velocity.x *= friction;
            particle->velocity.z *= friction;
            particle->velocity *= 0.994;
        }

        ++hand.ground_contacts;
        hand.ever_grounded = true;
    }

    hand.max_rotation_radians =
        std::max(
            hand.max_rotation_radians,
            axis_rotation(hand));
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

        const auto rest =
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

        std::size_t destroyed_total = 0;
        std::size_t active_after_damage = 0;
        int detached_frame = -1;
        int walk_invalidated_frame = -1;
        bool normal_walk_authority = true;

        std::optional<DetachedHand>
            detached_hand;

        const double dt =
            1.0 / args.fps;

        double motion_time_seconds = 0.0;
        int motion_frame = 0;

        const std::vector<std::string>
            wrist_regions{
                "hand_l",
                "lowerarm_l"
            };

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

            const auto voxel_centers =
                voxel_character.sample_centers(
                    character,
                    clip,
                    motion_time_seconds,
                    true,
                    world_offset);

            const auto hand_split =
                character.sample_split_branch(
                    clip,
                    motion_time_seconds,
                    "hand_l",
                    true,
                    world_offset);

            const sarx::Vec3 animated_hand_center =
                used_centroid(
                    hand_split.detached);

            const sarx::Vec3 wrist =
                nearest_anchor(
                    hand_split.body,
                    hand_split.detached);

            if (!detached_hand
                && frame >= args.damage_frame
                && frame < args.damage_frame + 8) {

                const sarx::Vec3 hand_axis =
                    animated_hand_center - wrist;

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        wrist,
                        hand_axis,
                        args.voxel_size * 0.82,
                        args.voxel_size * 4.00,
                        0.55,
                        wrist_regions);

                destroyed_total +=
                    voxel_character.damage_anatomical_interface(
                        "hand_l",
                        {"lowerarm_l"},
                        0.55);

                auto component =
                    voxel_character
                        .detach_anatomical_region_if_disconnected(
                            "hand_l",
                            6);

                if (component) {
                    DetachedHand hand;
                    hand.component =
                        std::move(*component);

                    hand.rest_centers.reserve(
                        hand.component
                            .voxel_indices
                            .size());

                    std::vector<sarx::Vec3>
                        previous_component_centers;

                    previous_component_centers.reserve(
                        hand.component
                            .voxel_indices
                            .size());

                    const auto previous_voxel_centers =
                        voxel_character.sample_centers(
                            character,
                            clip,
                            std::max(
                                0.0,
                                motion_time_seconds - dt),
                            true,
                            world_offset_for(
                                std::max(
                                    0,
                                    motion_frame - 1)));

                    for (const std::size_t index
                         : hand.component
                               .voxel_indices) {

                        hand.rest_centers.push_back(
                            voxel_character
                                .voxel_center(
                                    index,
                                    voxel_centers));

                        previous_component_centers.push_back(
                            voxel_character
                                .voxel_center(
                                    index,
                                    previous_voxel_centers));
                    }

                    const auto [a_index, b_index] =
                        farthest_pair(
                            hand.rest_centers);

                    hand.rest_a =
                        hand.rest_centers[a_index];
                    hand.rest_b =
                        hand.rest_centers[b_index];

                    hand.particle_a =
                        hand.motion.add_particle(
                            hand.rest_a,
                            0.5);

                    hand.particle_b =
                        hand.motion.add_particle(
                            hand.rest_b,
                            0.5);

                    hand.motion
                        .particles()[hand.particle_a]
                        .velocity =
                            (hand.rest_a
                             - previous_component_centers[a_index])
                            / dt;

                    hand.motion
                        .particles()[hand.particle_b]
                        .velocity =
                            (hand.rest_b
                             - previous_component_centers[b_index])
                            / dt;

                    hand.motion.add_structural_constraint(
                        hand.particle_a,
                        hand.particle_b,
                        1e-9);

                    detached_frame = frame;
                    detached_hand =
                        std::move(hand);
                }
            }

            if (detached_hand
                && frame > detached_frame) {

                step_detached_hand(
                    *detached_hand,
                    dt,
                    args.voxel_size);
            }

            const auto motion_viability =
                sarx::evaluate_motion_viability(
                    character.animation_names()[clip],
                    voxel_character.anatomy_availability());

            if (motion_viability.state
                    == sarx::MotionViability::Invalid
                && normal_walk_authority) {
                normal_walk_authority = false;
                walk_invalidated_frame = frame;
            }

            sarx::CharacterMeshFrame visible =
                voxel_character.render(
                    voxel_centers);

            std::vector<sarx::Vec3>
                detached_centers;

            if (detached_hand) {
                detached_centers =
                    detached_world_centers(
                        *detached_hand);

                visible =
                    combine(
                        visible,
                        voxel_character
                            .render_component(
                                detached_hand
                                    ->component,
                                detached_centers));
            }

            sarx::Vec3 camera_target =
                wrist;

            if (detached_hand
                && !detached_centers.empty()) {

                camera_target =
                    (wrist
                     + centroid(
                         detached_centers))
                    * 0.5;
            } else {
                camera_target =
                    (wrist
                     + animated_hand_center)
                    * 0.5;
            }

            // Follow the actual left-wrist cut in a close three-quarter
            // view so the voxel removal, separation, fall, and impact
            // remain visible instead of happening off-camera.
            sarx::CharacterRenderCamera camera;
            camera.target =
                camera_target
                + sarx::Vec3{
                    0.0,
                    -scale * 0.05,
                    0.0
                };

            camera.position =
                camera.target
                + sarx::Vec3{
                    scale * 0.52,
                    scale * 0.14,
                    scale * 1.05
                };

            camera.vertical_fov_degrees =
                30.0;

            camera.width = 960;
            camera.height = 720;

            sarx::write_character_ppm(
                frame_path(
                    args.output,
                    frame),
                visible,
                camera,
                true);

            if (frame
                == args.damage_frame + 10) {
                active_after_damage =
                    voxel_character
                        .stats()
                        .active_voxels;
            }
        }

        std::size_t unrelated_changed_voxels = 0;

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

        const double left_thigh_fraction =
            voxel_character.attached_fraction(
                "thigh_l");

        const auto final_viability =
            sarx::evaluate_motion_viability(
                character.animation_names()[clip],
                voxel_character.anatomy_availability());

        const auto stats =
            voxel_character.stats();

        if (args.require_damage
            && destroyed_total == 0) {
            throw std::runtime_error(
                "Quaternius voxel wrist cut destroyed no voxels");
        }

        if (args.require_detachment
            && !detached_hand) {
            throw std::runtime_error(
                "left voxel hand never disconnected from main body; wrist_cut_voxels="
                + std::to_string(destroyed_total)
                + " attached_voxels="
                + std::to_string(stats.attached_voxels));
        }

        if (args.require_ground_contact
            && (!detached_hand
                || !detached_hand
                    ->ever_grounded)) {
            throw std::runtime_error(
                "detached voxel hand never hit the ground");
        }

        if (args.require_anatomical_isolation
            && unrelated_changed_voxels != 0) {
            throw std::runtime_error(
                "left wrist cut altered anatomically unrelated voxels: "
                + std::to_string(
                    unrelated_changed_voxels));
        }

        if (args.require_walk_viability
            && final_viability.state
                == sarx::MotionViability::Invalid) {
            throw std::runtime_error(
                "normal Walk became invalid during isolated hand severance");
        }

        std::cout
            << "SARX Quaternius voxel hand detachment demo complete:"
            << " total_voxels="
            << stats.total_voxels
            << " active_voxels="
            << stats.active_voxels
            << " attached_voxels="
            << stats.attached_voxels
            << " detached_voxels="
            << stats.detached_voxels
            << " destroyed_voxels="
            << stats.destroyed_voxels
            << " active_after_damage="
            << active_after_damage
            << " wrist_cut_voxels="
            << destroyed_total
            << " detached_frame="
            << detached_frame
            << " hand_ground_contacts="
            << (detached_hand
                ? detached_hand
                    ->ground_contacts
                : 0)
            << " hand_max_rotation_rad="
            << (detached_hand
                ? detached_hand
                    ->max_rotation_radians
                : 0.0)
            << " unrelated_changed_voxels="
            << unrelated_changed_voxels
            << " left_thigh_attached_fraction="
            << left_thigh_fraction
            << " walk_invalidated_frame="
            << walk_invalidated_frame
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
            << "sarx_character_voxel_demo: "
            << error.what()
            << '\n';

        return EXIT_FAILURE;
    }
}
