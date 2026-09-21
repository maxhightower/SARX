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
                << " [--require-anatomical-isolation]\n";
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

std::pair<std::size_t, std::size_t>
farthest_pair(
    const std::vector<sarx::Vec3>& points) {

    if (points.size() < 2) {
        throw std::runtime_error(
            "detached thigh component needs at least two voxels");
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

            const double d2 =
                sarx::length_squared(
                    points[j] - points[i]);

            if (d2 > best) {
                best = d2;
                a = i;
                b = j;
            }
        }
    }

    return {a, b};
}

struct DetachedLeg {
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

double axis_rotation(
    const DetachedLeg& leg) {

    const sarx::Vec3 rest_axis =
        sarx::normalized(
            leg.rest_b
            - leg.rest_a);

    const sarx::Vec3 current_axis =
        sarx::normalized(
            leg.motion
                .particles()[leg.particle_b]
                .position
            - leg.motion
                .particles()[leg.particle_a]
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

std::vector<sarx::Vec3>
detached_world_centers(
    const DetachedLeg& leg) {

    const sarx::Vec3 current_a =
        leg.motion
            .particles()[leg.particle_a]
            .position;

    const sarx::Vec3 current_b =
        leg.motion
            .particles()[leg.particle_b]
            .position;

    const sarx::Vec3 rest_axis =
        leg.rest_b - leg.rest_a;

    const sarx::Vec3 current_axis =
        current_b - current_a;

    std::vector<sarx::Vec3> centers;

    centers.reserve(
        leg.rest_centers.size());

    for (const auto& rest_center
         : leg.rest_centers) {

        centers.push_back(
            current_a
            + sarx::rotate_between(
                rest_axis,
                current_axis,
                rest_center - leg.rest_a));
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

void step_detached_leg(
    DetachedLeg& leg,
    double dt,
    double voxel_size) {

    sarx::StepConfig config;
    config.substeps = 8;
    config.solver_iterations = 16;
    config.gravity = {0.0, -9.81, 0.0};

    leg.motion.step(
        dt,
        config);

    auto& a =
        leg.motion
            .particles()[leg.particle_a];

    auto& b =
        leg.motion
            .particles()[leg.particle_b];

    const sarx::Vec3 before_a =
        a.position;

    const sarx::Vec3 before_b =
        b.position;

    const sarx::Vec3 rest_axis =
        leg.rest_b - leg.rest_a;

    const double rest_axis_squared =
        std::max(
            sarx::length_squared(rest_axis),
            1e-12);

    const double ground_height =
        voxel_size * 0.47;

    bool contacted = false;

    for (int iteration = 0;
         iteration < 10;
         ++iteration) {

        const sarx::Vec3 current_axis =
            b.position - a.position;

        for (const auto& rest_center
             : leg.rest_centers) {

            const sarx::Vec3 world_center =
                a.position
                + sarx::rotate_between(
                    rest_axis,
                    current_axis,
                    rest_center - leg.rest_a);

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
                        rest_center - leg.rest_a,
                        rest_axis)
                    / rest_axis_squared,
                    0.0,
                    1.0);

            const double wa = 1.0 - t;
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
            * 0.18;

        b.velocity +=
            (b.position - before_b)
            / dt
            * 0.18;

        constexpr double restitution = 0.06;
        constexpr double friction = 0.58;

        for (auto* particle : {&a, &b}) {
            if (particle->velocity.y < 0.0) {
                particle->velocity.y =
                    -particle->velocity.y
                    * restitution;
            }

            particle->velocity.x *= friction;
            particle->velocity.z *= friction;
            particle->velocity *= 0.992;
        }

        ++leg.ground_contacts;
        leg.ever_grounded = true;
    }

    leg.max_rotation_radians =
        std::max(
            leg.max_rotation_radians,
            axis_rotation(leg));
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

        std::optional<DetachedLeg>
            detached_leg;

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

            const sarx::Vec3 hip =
                nearest_anchor(
                    thigh_split.body,
                    thigh_split.detached);

            const sarx::Vec3 knee =
                nearest_anchor(
                    calf_split.body,
                    calf_split.detached);

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

                    std::vector<sarx::Vec3>
                        previous_centers =
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

                    leg.rest_centers.reserve(
                        leg.component
                            .voxel_indices
                            .size());

                    std::vector<sarx::Vec3>
                        previous_component_centers;

                    previous_component_centers.reserve(
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

                        previous_component_centers.push_back(
                            voxel_character
                                .voxel_center(
                                    index,
                                    previous_centers));
                    }

                    const auto [a_index, b_index] =
                        farthest_pair(
                            leg.rest_centers);

                    leg.rest_a =
                        leg.rest_centers[a_index];

                    leg.rest_b =
                        leg.rest_centers[b_index];

                    leg.particle_a =
                        leg.motion.add_particle(
                            leg.rest_a,
                            0.55);

                    leg.particle_b =
                        leg.motion.add_particle(
                            leg.rest_b,
                            0.45);

                    leg.motion
                        .particles()[leg.particle_a]
                        .velocity =
                            (leg.rest_a
                             - previous_component_centers[a_index])
                            / dt;

                    leg.motion
                        .particles()[leg.particle_b]
                        .velocity =
                            (leg.rest_b
                             - previous_component_centers[b_index])
                            / dt;

                    leg.motion.add_structural_constraint(
                        leg.particle_a,
                        leg.particle_b,
                        1e-9);

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
            }

            if (detached_leg
                && frame > detached_frame) {

                step_detached_leg(
                    *detached_leg,
                    dt,
                    args.voxel_size);
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

            sarx::Vec3 camera_target =
                cut_center;

            if (detached_leg
                && !detached_centers.empty()) {

                camera_target =
                    (cut_center
                     + centroid(
                         detached_centers))
                    * 0.5;
            }

            sarx::CharacterRenderCamera camera;

            camera.target =
                camera_target
                + sarx::Vec3{
                    0.0,
                    -scale * 0.18,
                    0.0
                };

            camera.position =
                camera.target
                + sarx::Vec3{
                    scale * 0.80,
                    scale * 0.20,
                    scale * 1.55
                };

            camera.vertical_fov_degrees =
                32.0;

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
                    ->ever_grounded)) {
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
                    ->ground_contacts
                : 0)
            << " leg_max_rotation_rad="
            << (detached_leg
                ? detached_leg
                    ->max_rotation_radians
                : 0.0)
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
