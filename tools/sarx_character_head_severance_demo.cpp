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
    std::string character{"assets/quaternius/character.glb"};
    std::string animations{"assets/quaternius/animations.glb"};
    std::string clip{"Walk"};
    std::filesystem::path output{"media/raw/v06_real_character_head_severance_frames"};
    int frames{210};
    int cut_frame{90};
    double fps{30.0};
    bool require_ground_contact{false};
    bool require_rotation{false};
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
        else if (value == "--cut-frame" && i + 1 < argc) args.cut_frame = std::stoi(argv[++i]);
        else if (value == "--fps" && i + 1 < argc) args.fps = std::stod(argv[++i]);
        else if (value == "--require-ground-contact") args.require_ground_contact = true;
        else if (value == "--require-rotation") args.require_rotation = true;
        else if (value == "--help") {
            std::cout << "sarx_character_head_severance_demo"
                      << " [--character FILE] [--animations FILE] [--clip NAME]"
                      << " [--output DIR] [--frames N] [--cut-frame N] [--fps N]"
                      << " [--require-ground-contact] [--require-rotation]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("unknown or incomplete head severance argument");
        }
    }
    if (args.frames <= 0 || args.fps <= 0.0 || args.cut_frame < 1 || args.cut_frame >= args.frames) {
        throw std::invalid_argument("invalid head severance frame/fps settings");
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

std::vector<std::uint8_t> used_vertices(const sarx::CharacterMeshFrame& frame) {
    std::vector<std::uint8_t> used(frame.positions.size(), 0u);
    for (const auto index : frame.indices) {
        if (index < used.size()) used[index] = 1u;
    }
    return used;
}

sarx::Vec3 nearest_anchor(
    const sarx::CharacterMeshFrame& a,
    const sarx::CharacterMeshFrame& b) {

    const auto used_a = used_vertices(a);
    const auto used_b = used_vertices(b);
    double best = std::numeric_limits<double>::infinity();
    sarx::Vec3 pa{}, pb{};

    for (std::size_t i = 0; i < used_a.size(); ++i) {
        if (!used_a[i]) continue;
        for (std::size_t j = 0; j < used_b.size(); ++j) {
            if (!used_b[j]) continue;
            const double d2 = sarx::length_squared(a.positions[i] - b.positions[j]);
            if (d2 < best) {
                best = d2;
                pa = a.positions[i];
                pb = b.positions[j];
            }
        }
    }
    if (!std::isfinite(best)) throw std::runtime_error("could not infer head cut anchor");
    return (pa + pb) * 0.5;
}

sarx::Vec3 farthest_used_point(
    const sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& from) {

    const auto used = used_vertices(frame);
    double best = -1.0;
    sarx::Vec3 point{};
    for (std::size_t i = 0; i < used.size(); ++i) {
        if (!used[i]) continue;
        const double d2 = sarx::length_squared(frame.positions[i] - from);
        if (d2 > best) {
            best = d2;
            point = frame.positions[i];
        }
    }
    if (best < 0.0) throw std::runtime_error("could not infer head orientation anchor");
    return point;
}

sarx::CharacterMeshFrame transform_segment(
    const sarx::CharacterMeshFrame& rest,
    const sarx::Vec3& rest_a,
    const sarx::Vec3& rest_b,
    const sarx::Vec3& current_a,
    const sarx::Vec3& current_b) {

    sarx::CharacterMeshFrame out = rest;
    const sarx::Vec3 rest_axis = rest_b - rest_a;
    const sarx::Vec3 current_axis = current_b - current_a;
    for (auto& position : out.positions) {
        position = current_a
            + sarx::rotate_between(rest_axis, current_axis, position - rest_a);
    }
    return out;
}

sarx::CharacterMeshFrame combine(
    const sarx::CharacterMeshFrame& body,
    const sarx::CharacterMeshFrame& detached) {

    sarx::CharacterMeshFrame out;
    out.positions = body.positions;
    out.indices = body.indices;
    const auto base = static_cast<std::uint32_t>(out.positions.size());
    out.positions.insert(out.positions.end(), detached.positions.begin(), detached.positions.end());
    for (const auto index : detached.indices) out.indices.push_back(base + index);
    return out;
}

bool project_head_floor(
    sarx::Particle& neck,
    sarx::Particle& crown,
    double radius) {

    bool hit = false;
    constexpr double samples[] = {0.0, 0.20, 0.40, 0.60, 0.80, 1.0};
    for (const double t : samples) {
        const double a = 1.0 - t;
        const double b = t;
        const double y = neck.position.y * a + crown.position.y * b;
        if (y >= radius) continue;
        const double denom = a * a + b * b;
        const double correction = (radius - y) / std::max(denom, 1e-12);
        neck.position.y += correction * a;
        crown.position.y += correction * b;
        hit = true;
    }
    return hit;
}

double angle_between(const sarx::Vec3& a, const sarx::Vec3& b) {
    const auto na = sarx::normalized(a);
    const auto nb = sarx::normalized(b);
    if (sarx::length_squared(na) <= 1e-12 || sarx::length_squared(nb) <= 1e-12) return 0.0;
    return std::acos(std::clamp(sarx::dot(na, nb), -1.0, 1.0));
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output);

        sarx::GltfCharacter character;
        character.load(args.character, args.animations);
        const std::size_t clip = character.find_animation(args.clip);

        const auto initial = character.sample(clip, 0.0, true);
        const Bounds bounds = bounds_of(initial);
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

        sarx::CharacterMeshFrame head_snapshot;
        sarx::Vec3 rest_neck{}, rest_crown{};
        sarx::Body head_motion;
        sarx::ParticleId neck_particle = 0;
        sarx::ParticleId crown_particle = 0;
        bool cut = false;
        bool ever_grounded = false;
        std::size_t ground_contacts = 0;
        double max_rotation = 0.0;

        sarx::StepConfig config;
        config.substeps = 6;
        config.solver_iterations = 16;
        config.gravity = {0.0, -9.81, 0.0};

        const double dt = 1.0 / args.fps;

        for (int frame = 0; frame < args.frames; ++frame) {
            const double seconds = static_cast<double>(frame) / args.fps;
            const sarx::Vec3 world_offset = world_offset_for(frame);
            sarx::CharacterMeshFrame visible;

            if (!cut && frame < args.cut_frame) {
                visible = character.sample(clip, seconds, true, world_offset);
            } else {
                const auto split = character.sample_split_branch(
                    clip, seconds, "Head", true, world_offset);

                if (!cut) {
                    const double previous_seconds = static_cast<double>(frame - 1) / args.fps;
                    const auto previous_split = character.sample_split_branch(
                        clip, previous_seconds, "Head", true, world_offset_for(frame - 1));

                    head_snapshot = split.detached;
                    rest_neck = nearest_anchor(split.body, split.detached);
                    rest_crown = farthest_used_point(split.detached, rest_neck);

                    const sarx::Vec3 previous_neck =
                        nearest_anchor(previous_split.body, previous_split.detached);
                    const sarx::Vec3 previous_crown =
                        farthest_used_point(previous_split.detached, previous_neck);

                    neck_particle = head_motion.add_particle(rest_neck, 0.55);
                    crown_particle = head_motion.add_particle(rest_crown, 0.45);
                    head_motion.particles()[neck_particle].velocity =
                        (rest_neck - previous_neck) / dt;
                    head_motion.particles()[crown_particle].velocity =
                        (rest_crown - previous_crown) / dt;
                    head_motion.add_structural_constraint(
                        neck_particle, crown_particle, 1e-9);
                    cut = true;
                } else {
                    head_motion.step(dt, config);

                    auto& neck = head_motion.particles()[neck_particle];
                    auto& crown = head_motion.particles()[crown_particle];
                    const sarx::Vec3 before_neck = neck.position;
                    const sarx::Vec3 before_crown = crown.position;

                    const bool hit = project_head_floor(neck, crown, 0.09);
                    if (hit) {
                        constexpr double restitution = 0.10;
                        constexpr double friction = 0.68;

                        neck.velocity += (neck.position - before_neck) / dt * 0.18;
                        crown.velocity += (crown.position - before_crown) / dt * 0.18;

                        for (auto* p : {&neck, &crown}) {
                            if (p->velocity.y < 0.0) p->velocity.y = -p->velocity.y * restitution;
                            p->velocity.x *= friction;
                            p->velocity.z *= friction;
                            p->velocity *= 0.992;
                        }

                        ++ground_contacts;
                        ever_grounded = true;
                    }
                }

                const sarx::Vec3 current_neck =
                    head_motion.particles()[neck_particle].position;
                const sarx::Vec3 current_crown =
                    head_motion.particles()[crown_particle].position;

                max_rotation = std::max(
                    max_rotation,
                    angle_between(rest_crown - rest_neck, current_crown - current_neck));

                const auto detached = transform_segment(
                    head_snapshot,
                    rest_neck,
                    rest_crown,
                    current_neck,
                    current_crown);

                visible = combine(split.body, detached);
            }

            sarx::write_character_ppm(
                frame_path(args.output, frame),
                visible,
                camera,
                true);
        }

        if (args.require_ground_contact && !ever_grounded) {
            throw std::runtime_error("detached Quaternius head never reached the floor");
        }
        if (args.require_rotation && max_rotation < 0.12) {
            throw std::runtime_error("detached Quaternius head never visibly rotated");
        }

        std::cout << "SARX real character head severance demo complete:"
                  << " clip=" << character.animation_names()[clip]
                  << " ground_contacts=" << ground_contacts
                  << " max_rotation_rad=" << max_rotation
                  << " frames=" << args.frames
                  << " output=" << args.output.string()
                  << '\n';
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "sarx_character_head_severance_demo: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
