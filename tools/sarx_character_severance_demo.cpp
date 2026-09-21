#include "sarx/body.hpp"
#include "sarx/character_render.hpp"
#include "sarx/detached_articulation.hpp"
#include "sarx/gltf_character.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
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
    bool require_articulated_limb{false};
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
        } else if (value == "--require-articulated-limb") {
            args.require_articulated_limb = true;
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

std::vector<std::uint8_t> used_vertices(
    const sarx::CharacterMeshFrame& frame) {

    std::vector<std::uint8_t> used(
        frame.positions.size(),
        0u);

    for (const std::uint32_t index : frame.indices) {
        if (index < used.size()) {
            used[index] = 1u;
        }
    }

    return used;
}

using Triangle = std::array<std::uint32_t, 3>;

std::set<Triangle> triangle_set(
    const sarx::CharacterMeshFrame& frame) {

    std::set<Triangle> out;

    for (std::size_t tri = 0;
         tri + 2 < frame.indices.size();
         tri += 3) {
        out.insert({
            frame.indices[tri + 0],
            frame.indices[tri + 1],
            frame.indices[tri + 2]
        });
    }

    return out;
}

sarx::CharacterMeshFrame triangle_difference(
    const sarx::CharacterMeshFrame& outer,
    const sarx::CharacterMeshFrame& inner) {

    const auto inner_triangles =
        triangle_set(inner);

    sarx::CharacterMeshFrame result;
    result.positions = outer.positions;

    for (std::size_t tri = 0;
         tri + 2 < outer.indices.size();
         tri += 3) {

        const Triangle key{
            outer.indices[tri + 0],
            outer.indices[tri + 1],
            outer.indices[tri + 2]
        };

        if (!inner_triangles.contains(key)) {
            result.indices.insert(
                result.indices.end(),
                key.begin(),
                key.end());
        }
    }

    if (result.indices.empty()) {
        throw std::runtime_error(
            "articulated limb segment owns no triangles");
    }

    return result;
}

sarx::Vec3 nearest_anchor(
    const sarx::CharacterMeshFrame& a,
    const sarx::CharacterMeshFrame& b) {

    const auto used_a = used_vertices(a);
    const auto used_b = used_vertices(b);

    double best =
        std::numeric_limits<double>::infinity();

    sarx::Vec3 best_a{};
    sarx::Vec3 best_b{};

    for (std::size_t i = 0;
         i < used_a.size();
         ++i) {
        if (!used_a[i]) continue;

        for (std::size_t j = 0;
             j < used_b.size();
             ++j) {
            if (!used_b[j]) continue;

            const double d2 =
                sarx::length_squared(
                    a.positions[i]
                    - b.positions[j]);

            if (d2 < best) {
                best = d2;
                best_a = a.positions[i];
                best_b = b.positions[j];
            }
        }
    }

    if (!std::isfinite(best)) {
        throw std::runtime_error(
            "could not infer articulated limb joint anchor");
    }

    return (best_a + best_b) * 0.5;
}

sarx::Vec3 farthest_used_point(
    const sarx::CharacterMeshFrame& frame,
    const sarx::Vec3& from) {

    const auto used = used_vertices(frame);

    double best = -1.0;
    sarx::Vec3 point{};

    for (std::size_t i = 0;
         i < used.size();
         ++i) {
        if (!used[i]) continue;

        const double d2 =
            sarx::length_squared(
                frame.positions[i] - from);

        if (d2 > best) {
            best = d2;
            point = frame.positions[i];
        }
    }

    if (best < 0.0) {
        throw std::runtime_error(
            "could not infer detached hand endpoint");
    }

    return point;
}

sarx::CharacterMeshFrame transform_segment(
    const sarx::CharacterMeshFrame& rest,
    const sarx::Vec3& rest_a,
    const sarx::Vec3& rest_b,
    const sarx::Vec3& current_a,
    const sarx::Vec3& current_b) {

    sarx::CharacterMeshFrame out = rest;

    const sarx::Vec3 rest_axis =
        rest_b - rest_a;
    const sarx::Vec3 current_axis =
        current_b - current_a;

    for (auto& position : out.positions) {
        position =
            current_a
            + sarx::rotate_between(
                rest_axis,
                current_axis,
                position - rest_a);
    }

    return out;
}

double joint_angle(
    const sarx::Vec3& a,
    const sarx::Vec3& joint,
    const sarx::Vec3& b) {

    const sarx::Vec3 u =
        sarx::normalized(a - joint);
    const sarx::Vec3 v =
        sarx::normalized(b - joint);

    if (sarx::length_squared(u) <= 1e-12
        || sarx::length_squared(v) <= 1e-12) {
        return 0.0;
    }

    return std::acos(
        std::clamp(
            sarx::dot(u, v),
            -1.0,
            1.0));
}

sarx::Vec3 rotate_axis_angle(
    const sarx::Vec3& value,
    const sarx::Vec3& axis,
    double angle) {

    const sarx::Vec3 n =
        sarx::normalized(axis);

    if (sarx::length_squared(n) <= 1e-12) {
        return value;
    }

    const double c = std::cos(angle);
    const double si = std::sin(angle);

    return value * c
        + sarx::cross(n, value) * si
        + n * (
            sarx::dot(n, value)
            * (1.0 - c));
}

void project_distance(
    sarx::Particle& a,
    sarx::Particle& b,
    double rest_length,
    double strength = 1.0) {

    const sarx::Vec3 delta =
        b.position - a.position;

    const double length =
        sarx::length(delta);

    if (length <= 1e-12) {
        return;
    }

    const sarx::Vec3 correction =
        delta
        * ((length - rest_length)
           / length
           * 0.5
           * strength);

    a.position += correction;
    b.position -= correction;
}

void project_passive_joint(
    sarx::Particle& a,
    sarx::Particle& joint,
    sarx::Particle& b,
    double rest_angle,
    double min_angle,
    double max_angle,
    double passive_strength,
    double limit_strength) {

    const sarx::Vec3 u =
        a.position - joint.position;
    const sarx::Vec3 v =
        b.position - joint.position;

    if (sarx::length_squared(u) <= 1e-12
        || sarx::length_squared(v) <= 1e-12) {
        return;
    }

    const double angle =
        joint_angle(
            a.position,
            joint.position,
            b.position);

    double target = rest_angle;
    double strength = passive_strength;

    if (angle < min_angle) {
        target = min_angle;
        strength = limit_strength;
    } else if (angle > max_angle) {
        target = max_angle;
        strength = limit_strength;
    }

    const sarx::Vec3 axis =
        sarx::cross(u, v);

    if (sarx::length_squared(axis) <= 1e-12) {
        return;
    }

    const double half_correction =
        (target - angle)
        * strength
        * 0.5;

    const sarx::Vec3 new_u =
        rotate_axis_angle(
            u,
            axis,
            -half_correction);

    const sarx::Vec3 new_v =
        rotate_axis_angle(
            v,
            axis,
            half_correction);

    a.position =
        joint.position + new_u;

    b.position =
        joint.position + new_v;
}

void damp_passive_joint(
    sarx::Particle& a,
    sarx::Particle& joint,
    sarx::Particle& b,
    double damping) {

    const sarx::Vec3 u =
        sarx::normalized(
            a.position - joint.position);
    const sarx::Vec3 v =
        sarx::normalized(
            b.position - joint.position);

    if (sarx::length_squared(u) <= 1e-12
        || sarx::length_squared(v) <= 1e-12) {
        return;
    }

    const sarx::Vec3 rel_a =
        a.velocity - joint.velocity;
    const sarx::Vec3 rel_b =
        b.velocity - joint.velocity;

    const sarx::Vec3 tangent_a =
        rel_a
        - u * sarx::dot(rel_a, u);

    const sarx::Vec3 tangent_b =
        rel_b
        - v * sarx::dot(rel_b, v);

    a.velocity -= tangent_a * damping;
    b.velocity -= tangent_b * damping;

    joint.velocity +=
        (tangent_a + tangent_b)
        * (damping * 0.15);
}

bool project_segment_floor(
    sarx::Particle& a,
    sarx::Particle& b,
    double radius) {

    bool contacted = false;

    constexpr double samples[] = {
        0.15,
        0.35,
        0.50,
        0.65,
        0.85
    };

    for (const double t : samples) {
        const double wa = 1.0 - t;
        const double wb = t;

        const double y =
            a.position.y * wa
            + b.position.y * wb;

        if (y >= radius) {
            continue;
        }

        const double denominator =
            wa * wa + wb * wb;

        if (denominator <= 1e-12) {
            continue;
        }

        const double correction =
            (radius - y)
            / denominator;

        a.position.y += correction * wa;
        b.position.y += correction * wb;
        contacted = true;
    }

    return contacted;
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
        // Keep locomotion aligned with +Z while returning the
        // evidence camera to the original three-quarter corner view.
        camera.position =
            camera.target
            + sarx::Vec3{
                scale * 2.25,
                scale * 0.45,
                scale * 4.9
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

        sarx::CharacterMeshFrame upperarm_snapshot;
        sarx::CharacterMeshFrame forearm_snapshot;
        sarx::CharacterMeshFrame hand_snapshot;

        std::array<sarx::Vec3, 4> rest_anchors{};
        sarx::DetachedArticulatedChain detached_articulation;
        bool cut = false;

        std::size_t boundary_triangles = 0;
        std::size_t detached_triangles = 0;
        std::size_t ground_contacts = 0;
        bool ever_grounded = false;
        int first_ground_contact_frame = -1;

        double initial_elbow_angle = 0.0;
        double initial_wrist_angle = 0.0;
        double max_elbow_angle_delta = 0.0;
        double max_wrist_angle_delta = 0.0;

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
                    const bool right_arm =
                        args.detached_root == "upperarm_r";
                    const bool left_arm =
                        args.detached_root == "upperarm_l";

                    if (!right_arm && !left_arm) {
                        throw std::runtime_error(
                            "articulated arm evidence path requires upperarm_r or upperarm_l");
                    }

                    const std::string lowerarm_root =
                        right_arm ? "lowerarm_r" : "lowerarm_l";
                    const std::string hand_root =
                        right_arm ? "hand_r" : "hand_l";

                    const double previous_seconds =
                        static_cast<double>(frame - 1)
                        / args.fps;

                    const sarx::Vec3 previous_offset =
                        world_offset_for(frame - 1);

                    const auto previous_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            args.detached_root,
                            true,
                            previous_offset);

                    const auto lower_split =
                        character.sample_split_branch(
                            clip,
                            seconds,
                            lowerarm_root,
                            true,
                            world_offset);

                    const auto hand_split =
                        character.sample_split_branch(
                            clip,
                            seconds,
                            hand_root,
                            true,
                            world_offset);

                    const auto previous_lower_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            lowerarm_root,
                            true,
                            previous_offset);

                    const auto previous_hand_split =
                        character.sample_split_branch(
                            clip,
                            previous_seconds,
                            hand_root,
                            true,
                            previous_offset);

                    upperarm_snapshot =
                        triangle_difference(
                            split.detached,
                            lower_split.detached);

                    forearm_snapshot =
                        triangle_difference(
                            lower_split.detached,
                            hand_split.detached);

                    hand_snapshot =
                        hand_split.detached;

                    const auto previous_upperarm =
                        triangle_difference(
                            previous_split.detached,
                            previous_lower_split.detached);

                    const auto previous_forearm =
                        triangle_difference(
                            previous_lower_split.detached,
                            previous_hand_split.detached);

                    const auto previous_hand =
                        previous_hand_split.detached;

                    rest_anchors[0] =
                        nearest_anchor(
                            split.body,
                            upperarm_snapshot);
                    rest_anchors[1] =
                        nearest_anchor(
                            upperarm_snapshot,
                            forearm_snapshot);
                    rest_anchors[2] =
                        nearest_anchor(
                            forearm_snapshot,
                            hand_snapshot);
                    rest_anchors[3] =
                        farthest_used_point(
                            hand_snapshot,
                            rest_anchors[2]);

                    std::array<sarx::Vec3, 4> previous_anchors{};
                    previous_anchors[0] =
                        nearest_anchor(
                            previous_split.body,
                            previous_upperarm);
                    previous_anchors[1] =
                        nearest_anchor(
                            previous_upperarm,
                            previous_forearm);
                    previous_anchors[2] =
                        nearest_anchor(
                            previous_forearm,
                            previous_hand);
                    previous_anchors[3] =
                        farthest_used_point(
                            previous_hand,
                            previous_anchors[2]);

                    initial_elbow_angle =
                        sarx::articulated_joint_angle(
                            rest_anchors[0],
                            rest_anchors[1],
                            rest_anchors[2]);

                    initial_wrist_angle =
                        sarx::articulated_joint_angle(
                            rest_anchors[1],
                            rest_anchors[2],
                            rest_anchors[3]);

                    const double wrist_min =
                        std::max(
                            0.80,
                            initial_wrist_angle - 0.75);

                    const double wrist_max =
                        std::min(
                            3.05,
                            initial_wrist_angle + 0.75);

                    sarx::DetachedArticulationConfig articulation;
                    articulation.rest_anchors.assign(
                        rest_anchors.begin(),
                        rest_anchors.end());

                    articulation.previous_anchors.assign(
                        previous_anchors.begin(),
                        previous_anchors.end());

                    articulation.masses = {
                        0.32,
                        0.28,
                        0.22,
                        0.18
                    };

                    articulation.joints = {
                        sarx::PassiveJointProfile{
                            1,
                            0.35,
                            3.05,
                            0.070,
                            0.60,
                            0.18
                        },
                        sarx::PassiveJointProfile{
                            2,
                            wrist_min,
                            wrist_max,
                            0.090,
                            0.65,
                            0.24
                        }
                    };

                    articulation.ground_radius = 0.040;
                    articulation.restitution = 0.08;
                    articulation.tangential_damping = 0.66;
                    articulation.contact_velocity_scale = 0.22;
                    articulation.global_velocity_damping = 0.992;
                    articulation.contact_iterations = 8;
                    articulation.substeps = 6;
                    articulation.solver_iterations = 16;

                    detached_articulation.initialize(
                        articulation,
                        dt);

                    boundary_triangles =
                        split.boundary_triangles_removed;

                    detached_triangles =
                        split.detached.indices.size() / 3;

                    cut = true;
                } else {
                    const std::size_t contacts_before =
                        detached_articulation
                            .ground_contacts();

                    detached_articulation.step(dt);

                    ground_contacts =
                        detached_articulation
                            .ground_contacts();

                    ever_grounded =
                        detached_articulation
                            .ever_grounded();

                    if (first_ground_contact_frame < 0
                        && contacts_before == 0
                        && ground_contacts > 0) {
                        first_ground_contact_frame = frame;
                    }

                    max_elbow_angle_delta =
                        detached_articulation
                            .max_joint_angle_delta(0);

                    max_wrist_angle_delta =
                        detached_articulation
                            .max_joint_angle_delta(1);
                }

                const sarx::Vec3 shoulder =
                    detached_articulation
                        .anchor_position(0);
                const sarx::Vec3 elbow =
                    detached_articulation
                        .anchor_position(1);
                const sarx::Vec3 wrist =
                    detached_articulation
                        .anchor_position(2);
                const sarx::Vec3 hand_tip =
                    detached_articulation
                        .anchor_position(3);

                const auto upperarm =
                    transform_segment(
                        upperarm_snapshot,
                        rest_anchors[0],
                        rest_anchors[1],
                        shoulder,
                        elbow);

                const auto forearm =
                    transform_segment(
                        forearm_snapshot,
                        rest_anchors[1],
                        rest_anchors[2],
                        elbow,
                        wrist);

                const auto hand =
                    transform_segment(
                        hand_snapshot,
                        rest_anchors[2],
                        rest_anchors[3],
                        wrist,
                        hand_tip);

                visible =
                    combine(
                        combine(
                            combine(
                                split.body,
                                upperarm),
                            forearm),
                        hand);
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

        if (args.require_articulated_limb
            && max_elbow_angle_delta < 0.08
            && max_wrist_angle_delta < 0.08) {
            throw std::runtime_error(
                "detached Quaternius arm never articulated after severance");
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
            << " first_ground_contact_frame="
            << first_ground_contact_frame
            << " max_elbow_delta_rad="
            << max_elbow_angle_delta
            << " max_wrist_delta_rad="
            << max_wrist_angle_delta
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
