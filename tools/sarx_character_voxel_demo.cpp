#include "sarx/character_render.hpp"
#include "sarx/gltf_character.hpp"
#include "sarx/voxel_character.hpp"

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
    std::string character{"assets/quaternius/character.glb"};
    std::string animations{"assets/quaternius/animations.glb"};
    std::string clip{"Walk"};
    std::filesystem::path output{"media/raw/v07_quaternius_voxel_damage_frames"};
    int frames{210};
    int damage_frame{90};
    double fps{30.0};
    double voxel_size{0.055};
    bool require_damage{false};
};

Args parse_args(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string value = argv[i];
        if (value == "--character" && i + 1 < argc) args.character = argv[++i];
        else if (value == "--animations" && i + 1 < argc) args.animations = argv[++i];
        else if (value == "--clip" && i + 1 < argc) args.clip = argv[++i];
        else if (value == "--output" && i + 1 < argc) args.output = argv[++i];
        else if (value == "--frames" && i + 1 < argc) args.frames = std::stoi(argv[++i]);
        else if (value == "--damage-frame" && i + 1 < argc) args.damage_frame = std::stoi(argv[++i]);
        else if (value == "--fps" && i + 1 < argc) args.fps = std::stod(argv[++i]);
        else if (value == "--voxel-size" && i + 1 < argc) args.voxel_size = std::stod(argv[++i]);
        else if (value == "--require-damage") args.require_damage = true;
        else if (value == "--help") {
            std::cout << "sarx_character_voxel_demo"
                      << " [--character FILE] [--animations FILE] [--clip NAME]"
                      << " [--output DIR] [--frames N] [--damage-frame N]"
                      << " [--fps N] [--voxel-size N] [--require-damage]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("unknown or incomplete voxel demo argument");
        }
    }
    if (args.frames <= 0 || args.fps <= 0.0 || args.voxel_size <= 0.0
        || args.damage_frame < 1 || args.damage_frame >= args.frames) {
        throw std::invalid_argument("invalid voxel demo settings");
    }
    return args;
}

std::string frame_path(const std::filesystem::path& directory, int frame) {
    std::ostringstream name;
    name << "frame_" << std::setw(4) << std::setfill('0') << frame << ".ppm";
    return (directory / name.str()).string();
}

struct Bounds {
    sarx::Vec3 min{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    sarx::Vec3 max{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
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

sarx::Vec3 used_centroid(const sarx::CharacterMeshFrame& frame) {
    std::vector<std::uint8_t> used(frame.positions.size(), 0u);
    for (const auto index : frame.indices) {
        if (index < used.size()) used[index] = 1u;
    }

    sarx::Vec3 center{};
    std::size_t count = 0;
    for (std::size_t i = 0; i < used.size(); ++i) {
        if (!used[i]) continue;
        center += frame.positions[i];
        ++count;
    }
    if (count == 0) throw std::runtime_error("damage branch owns no vertices");
    return center / static_cast<double>(count);
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output);

        sarx::GltfCharacter character;
        character.load(args.character, args.animations);
        const std::size_t clip = character.find_animation(args.clip);

        const auto rest = character.sample(clip, 0.0, true);
        sarx::VoxelizedCharacter voxel_character;
        voxel_character.build(rest, args.voxel_size);

        const Bounds bounds = bounds_of(rest);
        const sarx::Vec3 center = (bounds.min + bounds.max) * 0.5;
        const double width = bounds.max.x - bounds.min.x;
        const double height = bounds.max.y - bounds.min.y;
        const double depth = bounds.max.z - bounds.min.z;
        const double scale = std::max({width, height, depth, 0.5});
        const double travel = std::max(height * 1.15, scale);

        sarx::CharacterRenderCamera camera;
        camera.target = center;
        camera.position = camera.target + sarx::Vec3{
            scale * 2.25, scale * 0.45, scale * 4.9};
        camera.vertical_fov_degrees = 34.0;
        camera.width = 960;
        camera.height = 720;

        auto world_offset_for = [&](int frame) {
            const double progress = args.frames > 1
                ? static_cast<double>(frame) / static_cast<double>(args.frames - 1)
                : 0.0;
            return sarx::Vec3{0.0, 0.0, -travel * 0.5 + travel * progress};
        };

        std::size_t destroyed_total = 0;
        std::size_t active_after_damage = 0;

        for (int frame = 0; frame < args.frames; ++frame) {
            const double seconds = static_cast<double>(frame) / args.fps;
            const sarx::Vec3 world_offset = world_offset_for(frame);
            const auto animated = character.sample(
                clip, seconds, true, world_offset);

            // Use the actual animated right forearm as the world-space
            // target. Damage is then applied to persistent voxel IDs,
            // not to the rig branch itself.
            if (frame >= args.damage_frame
                && frame < args.damage_frame + 5) {

                const auto forearm = character.sample_split_branch(
                    clip,
                    seconds,
                    "lowerarm_r",
                    true,
                    world_offset);

                const sarx::Vec3 target =
                    used_centroid(forearm.detached)
                    + sarx::Vec3{0.015, 0.0, 0.0};

                destroyed_total +=
                    voxel_character.damage_sphere(
                        animated,
                        target,
                        args.voxel_size * 1.35,
                        0.28);
            }

            const auto voxel_mesh =
                voxel_character.render(animated);

            sarx::write_character_ppm(
                frame_path(args.output, frame),
                voxel_mesh,
                camera,
                true);

            if (frame == args.damage_frame + 6) {
                active_after_damage =
                    voxel_character.stats().active_voxels;
            }
        }

        const auto stats = voxel_character.stats();

        if (args.require_damage && destroyed_total == 0) {
            throw std::runtime_error(
                "Quaternius voxel demo destroyed no individual voxels");
        }
        if (args.require_damage
            && stats.active_voxels >= stats.total_voxels) {
            throw std::runtime_error(
                "Quaternius voxel damage did not persist");
        }

        std::cout << "SARX Quaternius voxel damage demo complete:"
                  << " total_voxels=" << stats.total_voxels
                  << " active_voxels=" << stats.active_voxels
                  << " active_after_damage=" << active_after_damage
                  << " destroyed_voxels=" << destroyed_total
                  << " voxel_size=" << stats.voxel_size
                  << " frames=" << args.frames
                  << " output=" << args.output.string()
                  << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "sarx_character_voxel_demo: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
