#include "sarx/body.hpp"
#include "sarx/character_render.hpp"
#include "sarx/gltf_character.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

struct Args {
    std::string character{
        "assets/quaternius/character.glb"};
    std::string animations{
        "assets/quaternius/animations.glb"};
    std::string clip{"Walk"};
    std::string detached_root{"upperarm_r"};

    std::filesystem::path output{
        "media/raw/v05_real_character_severance_frames"};

    int frames{210};
    int cut_frame{90};
    double fps{30.0};
    bool validate_branch_only{false};
    bool require_ground_contact{false};
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
        } else if (value == "--detach-root" && i + 1 < argc) {
            args.detached_root = argv[++i];
        } else if (value == "--output" && i + 1 < argc) {
            args.output = argv[++i];
        } else if (value == "--frames" && i + 1 < argc) {
            args.frames = std::stoi(argv[++i]);
        } else if (value == "--cut-frame" && i + 1 < argc) {
            args.cut_frame = std::stoi(argv[++i]);
        } else if (value == "--fps" && i + 1 < argc) {
            args.fps = std::stod(argv[++i]);
        } else if (value == "--validate-branch-only") {
            args.validate_branch_only = true;
        } else if (value == "--require-ground-contact") {
            args.require_ground_contact = true;
        } else if (value == "--help") {
            std::cout
                << "sarx_character_severance_demo"
                << " [--character FILE]"
                << " [--animations FILE]"
                << " [--clip NAME_FRAGMENT]"
                << " [--detach-root JOINT]"
                << " [--output DIR]"
                << " [--frames N]"
                << " [--cut-frame N]"
                << " [--fps N]"
                << " [--validate-branch-only]"
                << " [--require-ground-contact]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete severance demo argument");
        }
    }

    if (args.frames <= 0
        || args.fps <= 0.0
        || args.cut_frame < 1
        || args.cut_frame >= args.frames) {
        throw std::invalid_argument(
            "invalid severance demo frame/fps settings");
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

Bounds bounds_of(const sarx::CharacterMeshFrame& frame) {
    if (frame.positions.empty()) {
        throw std::runtime_error(
            "character frame has no vertices");
    }

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

sarx::Vec3 used_centroid(
    const sarx::CharacterMeshFrame& frame) {

    if (frame.indices.empty()) {
        throw std::runtime_error(
            "detached mesh has no triangles");
    }

    std::vector<std::uint8_t> used(
        frame.positions.size(),
        0u);

    for (const std::uint32_t index : frame.indices) {
        if (index < used.size()) {
            used[index] = 1u;
        }
    }

    sarx::Vec3 center{};
    std::size_t count = 0;

    for (std::size_t i = 0; i < used.size(); ++i) {
        if (!used[i]) continue;
        center += frame.positions[i];
        ++count;
    }

    if (count == 0) {
        throw std::runtime_error(
            "detached mesh owns no vertices");
    }

    return center / static_cast<double>(count);
}

double used_min_y(
    const sarx::CharacterMeshFrame& frame) {

    if (frame.indices.empty()) {
        throw std::runtime_error(
            "detached mesh has no triangles");
    }

    std::vector<std::uint8_t> used(
        frame.positions.size(),
        0u);

    for (const std::uint32_t index : frame.indices) {
        if (index < used.size()) {
            used[index] = 1u;
        }
    }

    double min_y =
        std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < used.size(); ++i) {
        if (!used[i]) continue;
        min_y = std::min(
            min_y,
            frame.positions[i].y);
    }

    if (!std::isfinite(min_y)) {
        throw std::runtime_error(
            "detached mesh owns no vertices");
    }

    return min_y;
}

void translate_mesh(
    sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& delta) {

    for (auto& position : frame.positions) {
        position += delta;
    }
}

sarx::CharacterMeshFrame combine(
    const sarx::CharacterMeshFrame& body,
    const sarx::CharacterMeshFrame& detached) {

    sarx::CharacterMeshFrame out;

    out.positions = body.positions;
    out.indices = body.indices;

    const std::uint32_t base =
        static_cast<std::uint32_t>(
            out.positions.size());

    out.positions.insert(
        out.positions.end(),
        detached.positions.begin(),
        detached.positions.end());

    out.indices.reserve(
        out.indices.size()
        + detached.indices.size());

    for (const std::uint32_t index : detached.indices) {
        out.indices.push_back(base + index);
    }

    return out;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);

        std::filesystem::create_directories(
            args.output);

        sarx::GltfCharacter character;
        character.load(
            args.character,
            args.animations);

        const std::size_t clip =
            character.find_animation(args.clip);

        if (args.validate_branch_only) {
            const double duration =
                character.animation_duration(clip);

            const double sample_times[] = {
                0.0,
                duration * 0.33,
                duration * 0.66
            };

            std::size_t minimum_detached =
                std::numeric_limits<std::size_t>::max();
            std::size_t minimum_body =
                std::numeric_limits<std::size_t>::max();
            std::size_t maximum_boundary = 0;

            for (const double sample_time
                 : sample_times) {

                const auto split =
                    character.sample_split_branch(
                        clip,
                        sample_time,
                        args.detached_root,
                        true);

                const std::size_t detached_triangles =
                    split.detached.indices.size() / 3;
                const std::size_t body_triangles =
                    split.body.indices.size() / 3;

                if (detached_triangles == 0
                    || body_triangles == 0) {
                    throw std::runtime_error(
                        "branch split produced empty body or detached mesh");
                }

                minimum_detached =
                    std::min(
                        minimum_detached,
                        detached_triangles);
                minimum_body =
                    std::min(
                        minimum_body,
                        body_triangles);
                maximum_boundary =
                    std::max(
                        maximum_boundary,
                        split.boundary_triangles_removed);
            }

            if (maximum_boundary == 0) {
                throw std::runtime_error(
                    "branch split removed no boundary triangles");
            }

            std::cout
                << "SARX branch validation:"
                << " detach_root=" << args.detached_root
                << " samples=3"
                << " min_detached_triangles="
                << minimum_detached
                << " min_body_triangles="
                << minimum_body
                << " max_boundary_triangles="
                << maximum_boundary
                << "\n";

            return EXIT_SUCCESS;
        }

        const auto initial =
            character.sample(
                clip,
                0.0,
                true);

        const Bounds bounds =
            bounds_of(initial);

        const sarx::Vec3 center =
            (bounds.min + bounds.max) * 0.5;

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
            std::max(height * 1.15, scale);

        sarx::CharacterRenderCamera camera;
        camera.target = {
            center.x,
            center.y,
            center.z
        };
        // The Quaternius fixture faces +Z. Render it from the side
        // so translational travel follows the direction it faces.
        camera.position =
            camera.target
            + sarx::Vec3{
                scale * 4.9,
                scale * 0.45,
                0.0
            };
        camera.vertical_fov_degrees = 34.0;
        camera.width = 960;
        camera.height = 720;

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

        sarx::CharacterMeshFrame detached_snapshot;
        sarx::Vec3 detached_origin{};
        double detached_local_min_y = 0.0;

        sarx::Body detached_motion;
        sarx::ParticleId detached_particle = 0;
        bool cut = false;

        std::size_t boundary_triangles = 0;
        std::size_t detached_triangles = 0;
        std::size_t ground_contacts = 0;
        bool ever_grounded = false;

        sarx::StepConfig detached_step;
        detached_step.substeps = 2;
        detached_step.solver_iterations = 1;
        detached_step.gravity = {0.0, -1.8, 0.0};

        const double dt = 1.0 / args.fps;

        for (int frame = 0; frame < args.frames; ++frame) {
            const double seconds =
                static_cast<double>(frame) / args.fps;

            const sarx::Vec3 world_offset =
                world_offset_for(frame);

            sarx::CharacterMeshFrame visible;

            if (!cut && frame < args.cut_frame) {
                visible =
                    character.sample(
                        clip,
                        seconds,
                        true,
                        world_offset);
            } else {
                const auto split =
                    character.sample_split_branch(
                        clip,
                        seconds,
                        args.detached_root,
                        true,
                        world_offset);

                if (!cut) {
                    const double previous_seconds =
                        static_cast<double>(frame - 1)
                        / args.fps;

                    const auto previous_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            args.detached_root,
                            true,
                            world_offset_for(frame - 1));

                    detached_snapshot =
                        split.detached;

                    detached_origin =
                        used_centroid(
                            detached_snapshot);

                    detached_local_min_y =
                        used_min_y(detached_snapshot)
                        - detached_origin.y;

                    const sarx::Vec3 previous_center =
                        used_centroid(
                            previous_split.detached);

                    const sarx::Vec3 inherited_velocity =
                        (detached_origin - previous_center)
                        / dt;

                    detached_particle =
                        detached_motion.add_particle(
                            detached_origin,
                            1.0);

                    detached_motion
                        .particles()[detached_particle]
                        .velocity =
                            inherited_velocity;

                    boundary_triangles =
                        split.boundary_triangles_removed;

                    detached_triangles =
                        split.detached.indices.size() / 3;

                    cut = true;
                } else {
                    detached_motion.step(
                        dt,
                        detached_step);

                    // This is intentionally a narrow ground-plane contact
                    // model for the evidence demo, not a claim of general
                    // character/world collision support.
                    auto& particle =
                        detached_motion
                            .particles()[detached_particle];

                    const double lowest_y =
                        particle.position.y
                        + detached_local_min_y;

                    if (lowest_y < 0.0) {
                        particle.position.y -= lowest_y;

                        if (particle.velocity.y < 0.0) {
                            constexpr double restitution = 0.18;
                            particle.velocity.y =
                                -particle.velocity.y
                                * restitution;
                        }

                        constexpr double tangential_damping = 0.72;
                        particle.velocity.x *= tangential_damping;
                        particle.velocity.z *= tangential_damping;

                        if (std::abs(particle.velocity.y) < 0.06) {
                            particle.velocity.y = 0.0;
                        }

                        ++ground_contacts;
                        ever_grounded = true;
                    }
                }

                sarx::CharacterMeshFrame detached =
                    detached_snapshot;

                const sarx::Vec3 delta =
                    detached_motion
                        .particles()[detached_particle]
                        .position
                    - detached_origin;

                translate_mesh(
                    detached,
                    delta);

                visible =
                    combine(
                        split.body,
                        detached);
            }

            sarx::write_character_ppm(
                frame_path(
                    args.output,
                    frame),
                visible,
                camera,
                true);
        }

        if (args.require_ground_contact
            && !ever_grounded) {
            throw std::runtime_error(
                "detached Quaternius limb never reached the floor");
        }

        std::cout
            << "SARX real character severance demo complete:"
            << " clip="
            << character.animation_names()[clip]
            << " detach_root="
            << args.detached_root
            << " boundary_triangles="
            << boundary_triangles
            << " detached_triangles="
            << detached_triangles
            << " ground_contacts="
            << ground_contacts
            << " frames="
            << args.frames
            << " output="
            << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_severance_demo: "
            << error.what()
            << '\n';
        return EXIT_FAILURE;
    }
}
