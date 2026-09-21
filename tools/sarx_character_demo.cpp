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

namespace {

struct Args {
    std::string character{
        "assets/quaternius/character.glb"};
    std::string animations{
        "assets/quaternius/animations.glb"};
    std::string clip{"Walk"};

    std::filesystem::path output{
        "media/raw/v05_real_character_walk_frames"};

    int frames{180};
    double fps{30.0};
};

Args parse_args(int argc, char** argv) {
    Args args;

    for (int i = 1; i < argc; ++i) {
        const std::string value = argv[i];

        if (value == "--character"
            && i + 1 < argc) {
            args.character = argv[++i];
        } else if (
            value == "--animations"
            && i + 1 < argc) {
            args.animations = argv[++i];
        } else if (
            value == "--clip"
            && i + 1 < argc) {
            args.clip = argv[++i];
        } else if (
            value == "--output"
            && i + 1 < argc) {
            args.output = argv[++i];
        } else if (
            value == "--frames"
            && i + 1 < argc) {
            args.frames = std::stoi(argv[++i]);
        } else if (
            value == "--fps"
            && i + 1 < argc) {
            args.fps = std::stod(argv[++i]);
        } else if (value == "--help") {
            std::cout
                << "sarx_character_demo"
                << " [--character FILE]"
                << " [--animations FILE]"
                << " [--clip NAME_FRAGMENT]"
                << " [--output DIR]"
                << " [--frames N]"
                << " [--fps N]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete character demo argument");
        }
    }

    if (args.frames <= 0 || args.fps <= 0.0) {
        throw std::invalid_argument(
            "frames and fps must be positive");
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

        std::size_t clip = 0;

        try {
            clip =
                character.find_animation(
                    args.clip);
        } catch (const std::exception&) {
            std::cerr
                << "No animation matching '"
                << args.clip
                << "'. Available clips:\n";

            for (const auto& name
                 : character.animation_names()) {
                std::cerr << "  " << name << '\n';
            }
            throw;
        }

        const auto restish =
            character.sample(
                clip,
                0.0,
                true);

        const double clip_duration =
            character.animation_duration(clip);

        const auto motion_sample =
            character.sample(
                clip,
                clip_duration * 0.25,
                true);

        if (motion_sample.positions.size()
            != restish.positions.size()) {
            throw std::runtime_error(
                "walk motion sample changed vertex count");
        }

        double motion_squared = 0.0;
        for (std::size_t i = 0;
             i < restish.positions.size();
             ++i) {
            motion_squared +=
                sarx::length_squared(
                    motion_sample.positions[i]
                    - restish.positions[i]);
        }

        const double motion_rms =
            std::sqrt(
                motion_squared
                / static_cast<double>(
                    std::max<std::size_t>(
                        1,
                        restish.positions.size())));

        const Bounds bounds =
            bounds_of(restish);

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

        if (motion_rms <= scale * 0.001) {
            throw std::runtime_error(
                "selected animation does not visibly deform the skinned mesh");
        }

        const double travel =
            std::max(
                height * 1.05,
                scale);

        sarx::CharacterRenderCamera camera;
        camera.target = {
            center.x,
            center.y + 0.02 * height,
            center.z
        };

        camera.position =
            camera.target
            + sarx::Vec3{
                scale * 2.2,
                scale * 0.45,
                scale * 4.8
            };

        camera.vertical_fov_degrees = 34.0;
        camera.width = 960;
        camera.height = 720;

        for (int frame = 0;
             frame < args.frames;
             ++frame) {

            const double seconds =
                static_cast<double>(frame)
                / args.fps;

            const double progress =
                args.frames > 1
                ? static_cast<double>(frame)
                    / static_cast<double>(
                        args.frames - 1)
                : 0.0;

            const double x =
                -travel * 0.5
                + travel * progress;

            const auto mesh =
                character.sample(
                    clip,
                    seconds,
                    true,
                    {x, 0.0, 0.0});

            sarx::write_character_ppm(
                frame_path(
                    args.output,
                    frame),
                mesh,
                camera,
                true);
        }

        const auto& stats =
            character.stats();

        std::cout
            << "SARX real character demo complete:"
            << " clip="
            << character.animation_names()[clip]
            << " duration="
            << clip_duration
            << " motion_rms="
            << motion_rms
            << " vertices=" << stats.vertices
            << " triangles=" << stats.triangles
            << " joints=" << stats.skin_joints
            << " clips=" << stats.animation_clips
            << " frames=" << args.frames
            << " output=" << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_demo: "
            << error.what()
            << '\n';
        return EXIT_FAILURE;
    }
}
