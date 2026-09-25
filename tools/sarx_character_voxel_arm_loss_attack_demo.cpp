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
#include <cstdint>
#include <filesystem>
#include <fstream>
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
    int cut_frame{-1}; // -1: derived from audited action semantics
    double fps{30.0};
    double voxel_size{0.045};
    bool require_damage{false};
    bool require_detachment{false};
    bool require_substitution{false};
    bool require_contact{false};
    bool require_authority{false};
    bool require_isolation{false};
    bool require_continuity{false};
    // E1-B: "whole-arm" (shoulder severance). E1-A: "hand" (wrist severance).
    std::string injury{"whole-arm"};
    // Asset-binding interface for E1-A. A clip is bound to the left elbow
    // capability slot only with --certified-elbow, which asserts that the
    // clip passed provenance/licence review and visual audit on Quaternius.
    std::string elbow_animations;
    std::string elbow_clip;
    bool certified_elbow{false};
    bool require_authored_elbow{false};
    std::filesystem::path evidence;
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
        } else if (value == "--require-continuity") {
            args.require_continuity = true;
        } else if (value == "--injury" && i + 1 < argc) {
            args.injury = argv[++i];
        } else if (value == "--elbow-animations" && i + 1 < argc) {
            args.elbow_animations = argv[++i];
        } else if (value == "--elbow-clip" && i + 1 < argc) {
            args.elbow_clip = argv[++i];
        } else if (value == "--certified-elbow") {
            args.certified_elbow = true;
        } else if (value == "--require-authored-elbow") {
            args.require_authored_elbow = true;
        } else if (value == "--evidence" && i + 1 < argc) {
            args.evidence = argv[++i];
        } else if (value == "--help") {
            std::cout
                << "sarx_character_voxel_arm_loss_attack_demo"
                << " [--output DIR] [--frames N] [--cut-frame N]"
                << " [--fps N] [--voxel-size N]"
                << " [--require-damage] [--require-detachment]"
                << " [--require-substitution] [--require-contact]"
                << " [--require-authority] [--require-isolation]"
                << " [--require-continuity] [--evidence FILE.tsv]"
                << " [--injury whole-arm|hand]"
                << " [--elbow-animations GLB --elbow-clip NAME --certified-elbow]"
                << " [--require-authored-elbow]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument(
                "unknown or incomplete arm-loss attack argument");
        }
    }

    if (args.frames <= 0
        || (args.cut_frame != -1
            && (args.cut_frame < 1
                || args.cut_frame >= args.frames))
        || (args.injury != "whole-arm" && args.injury != "hand")
        || (args.certified_elbow
            && (args.elbow_animations.empty() || args.elbow_clip.empty()))
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
    const sarx::CharacterMeshFrame& b,
    std::uint8_t tag_b = 1u) {

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

    out.triangle_tags.resize(a.indices.size() / 3, 0u);
    out.triangle_tags.insert(
        out.triangle_tags.end(),
        b.indices.size() / 3,
        tag_b);

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

    frame.triangle_tags.resize(frame.indices.size() / 3, 0u);

    for (const auto index : triangles) {
        frame.indices.push_back(base + index);
    }

    frame.triangle_tags.resize(frame.indices.size() / 3, 2u);
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

std::uint64_t fnv1a(
    std::uint64_t hash,
    const std::vector<sarx::Vec3>& points) {

    for (const auto& p : points) {
        for (const double value : {p.x, p.y, p.z}) {
            const auto quantized =
                static_cast<std::int64_t>(
                    std::llround(value * 1e5));
            for (int byte = 0; byte < 8; ++byte) {
                hash ^= static_cast<std::uint64_t>(
                    (quantized >> (byte * 8)) & 0xff);
                hash *= 1099511628211ull;
            }
        }
    }

    return hash;
}

bool same_local_pose(
    const sarx::CharacterNodeLocalPose& a,
    const sarx::CharacterNodeLocalPose& b) {

    return a.translation.x == b.translation.x
        && a.translation.y == b.translation.y
        && a.translation.z == b.translation.z
        && a.rotation == b.rotation
        && a.scale.x == b.scale.x
        && a.scale.y == b.scale.y
        && a.scale.z == b.scale.z;
}

const sarx::ActionCapability& library_entry(
    const std::vector<sarx::ActionCapability>& library,
    sarx::ActionFamily family,
    sarx::ActionSide side) {

    for (const auto& capability : library) {
        if (capability.family == family
            && capability.side == side) {
            return capability;
        }
    }

    throw std::runtime_error(
        "attack library has no requested family/side entry");
}

double max_pelvis_step(
    const sarx::GltfCharacter& character,
    std::size_t animation,
    double fps) {

    const double duration =
        character.animation_duration(animation);

    double best = 0.0;
    sarx::Vec3 previous{};
    bool have_previous = false;

    for (int f = 0;
         static_cast<double>(f) / fps <= duration + 1e-9;
         ++f) {
        const sarx::Vec3 pelvis =
            character.node_world_position_with_local_poses(
                character.sample_node_local_poses(
                    animation,
                    static_cast<double>(f) / fps,
                    false),
                "pelvis");
        if (have_previous) {
            best = std::max(best, sarx::length(pelvis - previous));
        }
        previous = pelvis;
        have_previous = true;
    }

    return best;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);

        std::filesystem::create_directories(args.output);

        sarx::GltfCharacter character;
        character.load(args.character, args.animations);

        const auto& joints =
            character.skin_joints();

        // Certified attack library. Authority regions of every authored
        // entry are derived from its own clip (effector-chain contribution
        // analysis), not from a hand-written joint list.
        auto library =
            sarx::quaternius_attack_action_library();

        std::vector<sarx::AuthoredAuthorityAnalysis> derivations;

        std::optional<sarx::GltfCharacter> elbow_character;

        if (args.certified_elbow) {
            elbow_character.emplace();
            elbow_character->load(args.character, args.elbow_animations);

            for (auto& capability : library) {
                if (capability.family == sarx::ActionFamily::ElbowStrike
                    && capability.side == sarx::ActionSide::Left) {
                    capability =
                        sarx::bind_authored_motion(capability, args.elbow_clip);
                }
            }
        }

        // Resolve an action's authored clip to the character that holds it.
        auto resolve_clip =
            [&](const std::string& motion_id)
            -> std::pair<const sarx::GltfCharacter*, std::size_t> {
            if (elbow_character && motion_id == args.elbow_clip) {
                return {&*elbow_character,
                        elbow_character->find_animation(motion_id)};
            }
            return {&character, character.find_animation(motion_id)};
        };

        for (auto& capability : library) {
            if (!capability.authored_motion_available) {
                continue;
            }

            const auto clip = resolve_clip(capability.motion_id);

            const auto analysis =
                sarx::derive_authored_authority_root(
                    *clip.first,
                    clip.second,
                    capability.effector_joint);

            capability.authority_joint_roots = {
                analysis.authority_root};

            derivations.push_back(analysis);
        }

        const auto jab_capability =
            library_entry(
                library,
                sarx::ActionFamily::Punch,
                sarx::ActionSide::Left);

        const auto cross_capability =
            library_entry(
                library,
                sarx::ActionFamily::Punch,
                sarx::ActionSide::Right);

        if (jab_capability.motion_id != "Punch_Jab"
            || cross_capability.motion_id != "Punch_Cross"
            || jab_capability.effector != sarx::ActionEffector::LeftHand
            || cross_capability.effector != sarx::ActionEffector::RightHand) {
            throw std::runtime_error(
                "audited Jab/Cross library semantics are inconsistent");
        }

        const std::size_t jab =
            character.find_animation(jab_capability.motion_id);
        const std::size_t cross =
            character.find_animation(cross_capability.motion_id);

        const double jab_duration =
            character.animation_duration(jab);
        const double cross_duration =
            character.animation_duration(cross);

        // Predeclared sever phase: midpoint of the Jab's committed,
        // pre-contact interval [commitment_phase, contact_phase_begin).
        // The strike has started but has not reached its contact window.
        const double declared_sever_phase =
            0.5 * (jab_capability.commitment_phase
                   + jab_capability.contact_phase_begin);

        const int cut_frame =
            args.cut_frame >= 1
            ? args.cut_frame
            : static_cast<int>(
                std::lround(
                    declared_sever_phase * jab_duration * args.fps));

        if (cut_frame < 1 || cut_frame >= args.frames) {
            throw std::runtime_error("derived sever frame out of range");
        }

        const double actual_sever_phase =
            static_cast<double>(cut_frame) / args.fps / jab_duration;

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

        // Shared opponent volume, frozen before any damage: centered between
        // the audited intact Jab and intact Cross contact points, so both
        // authored attacks reach the same opponent. The target is never
        // moved onto the fallback trajectory.
        const auto jab_peak_pose =
            character.sample_node_local_poses(
                jab,
                jab_duration * 0.326923,
                false);

        const auto cross_peak_pose =
            character.sample_node_local_poses(
                cross,
                cross_duration * 0.266667,
                false);

        const sarx::Vec3 jab_peak_hand =
            character.node_world_position_with_local_poses(
                jab_peak_pose,
                jab_capability.effector_joint);

        const sarx::Vec3 cross_peak_hand =
            character.node_world_position_with_local_poses(
                cross_peak_pose,
                cross_capability.effector_joint);

        const sarx::Vec3 target =
            (jab_peak_hand + cross_peak_hand) * 0.5;

        const double baseline_separation =
            sarx::length(jab_peak_hand - cross_peak_hand);

        const double target_radius =
            std::max(
                baseline_separation * 0.5 + 0.06,
                args.voxel_size * 2.5);

        // Declared continuity thresholds (model/clip scale, fixed before
        // the evidence run): the composed pelvis may not step further per
        // frame than either intact authored clip plus half a voxel, and the
        // planted lead foot may not drift more than half a voxel.
        const double pelvis_step_limit =
            std::max(
                max_pelvis_step(character, jab, args.fps),
                max_pelvis_step(character, cross, args.fps))
            + args.voxel_size * 0.5;

        const double planted_foot_limit =
            args.voxel_size * 0.5;

        sarx::ActionCapability substitute;
        const sarx::GltfCharacter* substitute_character = nullptr;
        std::size_t substitute_clip = 0;
        double substitute_duration = 1.0;

        sarx::ActionExecutionController controller(
            sarx::BehavioralIntent::Attack,
            jab_capability,
            library);

        std::size_t destroyed_total = 0;
        int detached_frame = -1;
        int jab_invalidated_frame = -1;
        int substitution_selected_frame = -1;
        int replacement_engaged_frame = -1;
        int contact_frame = -1;
        double contact_phase = -1.0;
        double contact_velocity = 0.0;
        double min_target_distance =
            std::numeric_limits<double>::infinity();
        double min_distance_phase = -1.0;

        std::optional<DetachedArm> detached_arm;
        std::vector<std::string> physics_roots;
        std::vector<sarx::CharacterNodeLocalPose> physics_hold;
        sarx::AnimationAuthorityPlan authority;
        bool have_replacement_authority = false;

        std::string attachment_before;
        std::string attachment_after;

        std::size_t physics_pose_violations = 0;
        std::size_t detached_voxels_reattached = 0;
        double max_pelvis_frame_step = 0.0;
        double max_root_displacement = 0.0;
        double max_lead_foot_drift = 0.0;
        double detached_min_height =
            std::numeric_limits<double>::infinity();
        double detached_final_height = 0.0;
        double detached_travel = 0.0;
        double limb_contact_radius = 0.0;
        double detached_initial_speed = 0.0;
        sarx::Vec3 cut_plane_center{};
        sarx::Vec3 cut_plane_normal{};
        std::size_t distal_stump_voxels = 0;
        double cut_half_thickness = 0.0;
        const std::string severed_root =
            args.injury == "hand" ? "hand_l" : "upperarm_l";
        const std::string stump_region =
            args.injury == "hand" ? "lowerarm_l" : "upperarm_l";
        std::string segment_radii_text;
        double detached_rest_min_height = 0.0;

        sarx::Vec3 previous_hand{};
        bool have_previous_hand = false;
        sarx::Vec3 previous_pelvis{};
        bool have_previous_pelvis = false;
        sarx::Vec3 initial_root{};
        sarx::Vec3 initial_lead_foot{};
        sarx::Vec3 detached_initial_centroid{};

        std::uint64_t digest = 1469598103934665603ull;

        sarx::CharacterRenderCamera camera;
        camera.target =
            character_center
            + sarx::Vec3{0.0, -scale * 0.02, scale * 0.12};
        camera.position =
            camera.target
            + sarx::Vec3{
                scale * 1.55,
                scale * 0.30,
                scale * 1.75
            };
        camera.vertical_fov_degrees = 38.0;
        camera.width = 960;
        camera.height = 720;

        const sarx::Vec3 fixed_camera_position = camera.position;
        const sarx::Vec3 fixed_camera_target = camera.target;

        std::ostringstream frames_tsv;
        frames_tsv
            << "frame\ttime\tactive_motion\tactive_phase\treplacement_blend"
            << "\thand_r_x\thand_r_y\thand_r_z\tswept_target_distance"
            << "\tpelvis_x\tpelvis_y\tpelvis_z\tlead_foot_drift"
            << "\tdetached_centroid_x\tdetached_centroid_y\tdetached_centroid_z\tdetached_min_y"
            << "\tattached_voxels\tbase_joints\treplacement_joints\tphysics_joints\n";

        auto attachment_summary = [&]() {
            std::ostringstream out;
            out << "upperarm_l="
                << voxel_character.attached_fraction("upperarm_l")
                << ",lowerarm_l="
                << voxel_character.attached_fraction("lowerarm_l")
                << ",hand_l="
                << voxel_character.attached_fraction("hand_l");
            return out.str();
        };

        attachment_before = attachment_summary();

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

            // Replacement channels exist only once a substitute is active.
            double blend = 0.0;
            double replacement_elapsed = 0.0;
            std::vector<sarx::CharacterNodeLocalPose> replacement_pose;

            if (substitution_selected_frame >= 0) {
                replacement_elapsed =
                    static_cast<double>(
                        frame - substitution_selected_frame)
                    / args.fps;

                replacement_pose =
                    substitute_character->sample_node_local_poses(
                        substitute_clip,
                        std::min(replacement_elapsed, substitute_duration),
                        false);

                const double raw_blend =
                    std::clamp(replacement_elapsed / 0.15, 0.0, 1.0);

                blend =
                    raw_blend * raw_blend * (3.0 - 2.0 * raw_blend);
            }

            const auto composed =
                have_replacement_authority || detached_arm
                ? sarx::compose_action_local_poses(
                    base_pose,
                    replacement_pose.empty() ? base_pose : replacement_pose,
                    authority,
                    blend,
                    physics_hold)
                : base_pose;

            if (detached_arm) {
                for (const auto& pose : composed) {
                    if (sarx::authority_source_for(authority, pose.name)
                        != sarx::AnimationAuthoritySource::Physics) {
                        continue;
                    }
                    const auto hold =
                        std::find_if(
                            physics_hold.begin(),
                            physics_hold.end(),
                            [&](const sarx::CharacterNodeLocalPose& h) {
                                return h.name == pose.name;
                            });
                    if (hold == physics_hold.end()
                        || !same_local_pose(pose, *hold)) {
                        ++physics_pose_violations;
                    }
                }
            }

            auto voxel_centers =
                voxel_character.sample_centers_with_node_local_poses(
                    character,
                    composed);

            if (!detached_arm
                && args.injury == "whole-arm"
                && frame >= cut_frame
                && frame < cut_frame + 6) {

                const auto upperarm_split =
                    character.sample_split_branch(
                        jab, base_time, "upperarm_l", false);
                const auto lowerarm_split =
                    character.sample_split_branch(
                        jab, base_time, "lowerarm_l", false);
                const auto hand_split =
                    character.sample_split_branch(
                        jab, base_time, "hand_l", false);

                const sarx::Vec3 shoulder =
                    nearest_anchor(upperarm_split.body, upperarm_split.detached);
                const sarx::Vec3 elbow =
                    nearest_anchor(lowerarm_split.body, lowerarm_split.detached);
                const sarx::Vec3 wrist =
                    nearest_anchor(hand_split.body, hand_split.detached);
                const sarx::Vec3 hand_tip =
                    farthest_used_point(hand_split.detached, wrist);
                const sarx::Vec3 arm_seed =
                    used_centroid(lowerarm_split.detached);

                const sarx::Vec3 upperarm_axis =
                    normalized_or_throw(
                        elbow - shoulder,
                        "left upper-arm axis");

                const sarx::Vec3 shoulder_cut_center =
                    shoulder + upperarm_axis * (args.voxel_size * 0.45);

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        shoulder_cut_center,
                        upperarm_axis,
                        args.voxel_size * 1.05,
                        args.voxel_size * 4.8,
                        1.05,
                        {"upperarm_l"});

                cut_plane_center = shoulder_cut_center;
                cut_plane_normal = upperarm_axis;
                cut_half_thickness = args.voxel_size * 1.05;

                auto component =
                    voxel_character.detach_component_near_anatomical(
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
                            jab, previous_time, "upperarm_l", false);
                    const auto previous_lower =
                        character.sample_split_branch(
                            jab, previous_time, "lowerarm_l", false);
                    const auto previous_hand_split =
                        character.sample_split_branch(
                            jab, previous_time, "hand_l", false);

                    const sarx::Vec3 prev_shoulder =
                        nearest_anchor(previous_upper.body, previous_upper.detached);
                    const sarx::Vec3 prev_elbow =
                        nearest_anchor(previous_lower.body, previous_lower.detached);
                    const sarx::Vec3 prev_wrist =
                        nearest_anchor(previous_hand_split.body, previous_hand_split.detached);
                    const sarx::Vec3 prev_hand_tip =
                        farthest_used_point(previous_hand_split.detached, prev_wrist);

                    arm.rest_centers.reserve(arm.component.voxel_indices.size());
                    arm.segment_by_voxel.reserve(arm.component.voxel_indices.size());

                    for (const auto index : arm.component.voxel_indices) {
                        arm.rest_centers.push_back(
                            voxel_character.voxel_center(index, voxel_centers));

                        const std::string& region =
                            voxel_character.voxels()[index].anatomical_region;

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

                    // Articulation anchors are the skeleton joint centers
                    // (the split-mesh anchors above lie on the surface and
                    // only define the cut). The hand tip keeps the farthest
                    // detached-mesh point as its distal extent.
                    const auto previous_pose =
                        character.sample_node_local_poses(
                            jab, previous_time, false);

                    auto joint_at =
                        [&](const std::vector<sarx::CharacterNodeLocalPose>& pose,
                            const char* joint) {
                            return character.node_world_position_with_local_poses(
                                pose, joint);
                        };

                    const sarx::Vec3 j_shoulder = joint_at(composed, "upperarm_l");
                    const sarx::Vec3 j_elbow = joint_at(composed, "lowerarm_l");
                    const sarx::Vec3 j_wrist = joint_at(composed, "hand_l");
                    const sarx::Vec3 j_tip = hand_tip;

                    const sarx::Vec3 jp_shoulder = joint_at(previous_pose, "upperarm_l");
                    const sarx::Vec3 jp_elbow = joint_at(previous_pose, "lowerarm_l");
                    const sarx::Vec3 jp_wrist = joint_at(previous_pose, "hand_l");
                    const sarx::Vec3 jp_tip = prev_hand_tip;

                    const double elbow_angle =
                        sarx::articulated_joint_angle(j_shoulder, j_elbow, j_wrist);
                    const double wrist_angle =
                        sarx::articulated_joint_angle(j_elbow, j_wrist, j_tip);

                    sarx::DetachedArticulationConfig config;
                    config.rest_anchors = {j_shoulder, j_elbow, j_wrist, j_tip};
                    config.previous_anchors = {
                        jp_shoulder, jp_elbow, jp_wrist, jp_tip};
                    config.masses = {0.32, 0.30, 0.22, 0.16};
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
                    // Floor-contact capsules are the detached limb's
                    // measured cross-section: per segment, the farthest
                    // voxel center from the segment axis plus the voxel
                    // half-extent. A hard-coded thin radius lets the limb
                    // sink into the floor.
                    {
                        const sarx::Vec3 axis_points[4] = {
                            j_shoulder, j_elbow, j_wrist, j_tip};
                        config.segment_ground_radii.assign(3, args.voxel_size * 0.5);
                        for (std::size_t v = 0; v < arm.rest_centers.size(); ++v) {
                            // Each voxel belongs to the capsule whose axis
                            // is nearest (region labels blur at the joints).
                            std::size_t seg = 0;
                            double best = std::numeric_limits<double>::infinity();
                            for (std::size_t k = 0; k < 3; ++k) {
                                const double d =
                                    point_segment_distance(
                                        arm.rest_centers[v],
                                        axis_points[k],
                                        axis_points[k + 1]);
                                if (d < best) {
                                    best = d;
                                    seg = k;
                                }
                            }
                            config.segment_ground_radii[seg] =
                                std::max(
                                    config.segment_ground_radii[seg],
                                    best + args.voxel_size * 0.5);
                        }
                        for (const double r : config.segment_ground_radii) {
                            segment_radii_text += std::to_string(r) + ",";
                        }
                        limb_contact_radius =
                            *std::max_element(
                                config.segment_ground_radii.begin(),
                                config.segment_ground_radii.end());
                    }
                    config.ground_radius = limb_contact_radius;
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

            if (!detached_arm
                && args.injury == "hand"
                && frame >= cut_frame
                && frame < cut_frame + 6) {

                // E1-A injury: wrist severance (hand + fingers only).
                const auto hand_split =
                    character.sample_split_branch(
                        jab, base_time, "hand_l", false);

                const sarx::Vec3 wrist =
                    character.node_world_position_with_local_poses(
                        composed, "hand_l");

                const sarx::Vec3 hand_axis =
                    normalized_or_throw(
                        used_centroid(hand_split.detached) - wrist,
                        "left hand axis");

                cut_plane_center = wrist;
                cut_plane_normal = hand_axis;
                cut_half_thickness = args.voxel_size * 0.82;

                destroyed_total +=
                    voxel_character.damage_cut_disk(
                        voxel_centers,
                        wrist,
                        hand_axis,
                        cut_half_thickness,
                        args.voxel_size * 4.0,
                        0.55,
                        {"hand_l", "lowerarm_l"});

                destroyed_total +=
                    voxel_character.damage_anatomical_interface(
                        "hand_l",
                        {"lowerarm_l"},
                        0.55);

                auto component =
                    voxel_character.detach_anatomical_region_if_disconnected(
                        "hand_l",
                        {"lowerarm_l"},
                        6);

                if (component) {
                    DetachedArm hand;
                    hand.component = std::move(*component);

                    const auto previous_centers =
                        voxel_character.sample_centers_with_node_local_poses(
                            character,
                            character.sample_node_local_poses(
                                jab, std::max(0.0, base_time - dt), false));

                    std::vector<sarx::Vec3> previous_component;
                    for (const auto index : hand.component.voxel_indices) {
                        hand.rest_centers.push_back(
                            voxel_character.voxel_center(index, voxel_centers));
                        previous_component.push_back(
                            voxel_character.voxel_center(index, previous_centers));
                        hand.segment_by_voxel.push_back(0);
                    }

                    // Single rigid segment along the hand's longest extent.
                    std::size_t ia = 0;
                    std::size_t ib = 0;
                    double far = -1.0;
                    for (std::size_t i = 0; i < hand.rest_centers.size(); ++i) {
                        for (std::size_t j = i + 1; j < hand.rest_centers.size(); ++j) {
                            const double d2 =
                                sarx::length_squared(
                                    hand.rest_centers[i] - hand.rest_centers[j]);
                            if (d2 > far) {
                                far = d2;
                                ia = i;
                                ib = j;
                            }
                        }
                    }

                    sarx::DetachedArticulationConfig config;
                    config.rest_anchors = {
                        hand.rest_centers[ia], hand.rest_centers[ib]};
                    config.previous_anchors = {
                        previous_component[ia], previous_component[ib]};
                    config.masses = {0.5, 0.5};

                    double radius = args.voxel_size * 0.5;
                    for (const auto& center : hand.rest_centers) {
                        radius =
                            std::max(
                                radius,
                                point_segment_distance(
                                    center,
                                    hand.rest_centers[ia],
                                    hand.rest_centers[ib])
                                + args.voxel_size * 0.5);
                    }
                    limb_contact_radius = radius;
                    segment_radii_text = std::to_string(radius) + ",";
                    config.ground_radius = radius;
                    config.restitution = 0.08;
                    config.tangential_damping = 0.64;
                    config.contact_velocity_scale = 0.20;
                    config.global_velocity_damping = 0.994;
                    config.contact_iterations = 8;
                    config.substeps = 6;
                    config.solver_iterations = 14;

                    hand.articulation.initialize(config, dt);

                    detached_frame = frame;
                    detached_arm = std::move(hand);
                }
            }

            if (detached_arm && detached_frame == frame) {
            for (std::size_t k = 0; k < detached_arm->articulation.anchor_count(); ++k) {
                const auto v = detached_arm->articulation.anchor_velocity(k);
                detached_initial_speed =
                    std::max(detached_initial_speed, sarx::length(v));
            }

            // Physics ownership comes from the actual detached
            // topology; the hold pose is the pose at detachment.
            physics_roots =
                sarx::physics_joint_roots_from_voxels(
                    joints,
                    voxel_character,
                    {detached_arm->component});

            physics_hold.clear();
            for (const auto& pose : composed) {
                physics_hold.push_back(pose);
            }

            authority =
                sarx::build_action_authority_plan(
                    joints,
                    sarx::ActionCapability{},
                    physics_roots);

            attachment_after = attachment_summary();

            // Structural stump check: every surviving upper-arm
            // voxel must lie proximal to the cut disk.
            for (std::size_t v = 0; v < voxel_character.voxels().size(); ++v) {
                const auto& voxel = voxel_character.voxels()[v];
                if (voxel.state != sarx::CharacterVoxelState::Attached
                    || voxel.anatomical_region != stump_region) {
                    continue;
                }
                const double signed_distance =
                    sarx::dot(
                        voxel_character.voxel_center(v, voxel_centers)
                            - cut_plane_center,
                        cut_plane_normal);
                if (signed_distance > cut_half_thickness) {
                    ++distal_stump_voxels;
                }
            }

            const auto centers = detached_world_centers(*detached_arm);
            sarx::Vec3 c{};
            for (const auto& p : centers) c += p;
            detached_initial_centroid =
                c / static_cast<double>(centers.size());
            }

            if (detached_arm && frame > detached_frame) {
                detached_arm->articulation.step(dt);
            }

            // Re-evaluate the active action every frame against the
            // current anatomy (not only at action start).
            const std::string active_before =
                controller.active().motion_id;

            const double active_phase =
                substitution_selected_frame >= 0
                ? replacement_elapsed / substitute_duration
                : base_time / jab_duration;

            if (controller.update(
                    frame,
                    voxel_character.anatomy_availability(),
                    active_phase)) {

                if (active_before == jab_capability.motion_id) {
                    jab_invalidated_frame = frame;
                }

                if (!controller.active().motion_id.empty()
                    && controller.active().motion_id
                        != jab_capability.motion_id
                    && substitution_selected_frame < 0) {
                    substitution_selected_frame = frame;

                    substitute = controller.active();
                    const auto clip = resolve_clip(substitute.motion_id);
                    substitute_character = clip.first;
                    substitute_clip = clip.second;
                    substitute_duration =
                        substitute_character->animation_duration(clip.second);

                    authority =
                        sarx::build_action_authority_plan(
                            joints,
                            controller.active(),
                            physics_roots);

                    have_replacement_authority = true;
                }
            }

            if (replacement_engaged_frame < 0 && blend > 0.0) {
                replacement_engaged_frame = frame;
            }

            // Striking effector of the active substitute (hand for a punch,
            // lower-arm origin for an elbow). Before substitution the
            // opposite-hand joint is tracked only for continuity.
            const sarx::Vec3 hand =
                character.node_world_position_with_local_poses(
                    composed,
                    substitute.effector_joint.empty()
                        ? cross_capability.effector_joint
                        : substitute.effector_joint);

            double swept_distance =
                sarx::length(hand - target);

            if (substitution_selected_frame >= 0
                && frame > substitution_selected_frame) {

                double speed = 0.0;

                if (have_previous_hand) {
                    speed = sarx::length(hand - previous_hand) / dt;
                    swept_distance =
                        point_segment_distance(target, previous_hand, hand);
                }

                const double cross_phase =
                    replacement_elapsed / substitute_duration;

                if (swept_distance < min_target_distance) {
                    min_target_distance = swept_distance;
                    min_distance_phase = cross_phase;
                }

                if (contact_frame < 0
                    && swept_distance <= target_radius) {
                    contact_frame = frame;
                    contact_phase = cross_phase;
                    contact_velocity = speed;
                }
            }

            previous_hand = hand;
            have_previous_hand = true;

            const sarx::Vec3 pelvis =
                character.node_world_position_with_local_poses(
                    composed, "pelvis");
            const sarx::Vec3 root =
                character.node_world_position_with_local_poses(
                    composed, "root");
            const sarx::Vec3 lead_foot =
                character.node_world_position_with_local_poses(
                    composed, "foot_l");

            if (frame == 0) {
                initial_root = root;
                initial_lead_foot = lead_foot;
            }

            if (have_previous_pelvis) {
                max_pelvis_frame_step =
                    std::max(
                        max_pelvis_frame_step,
                        sarx::length(pelvis - previous_pelvis));
            }
            previous_pelvis = pelvis;
            have_previous_pelvis = true;

            max_root_displacement =
                std::max(
                    max_root_displacement,
                    sarx::length(root - initial_root));

            const double lead_foot_drift =
                sarx::length(lead_foot - initial_lead_foot);

            max_lead_foot_drift =
                std::max(max_lead_foot_drift, lead_foot_drift);

            sarx::CharacterMeshFrame visible =
                voxel_character.render(voxel_centers);

            digest = fnv1a(digest, voxel_centers);

            sarx::Vec3 detached_centroid{};
        double detached_frame_min_y = 0.0;

            if (detached_arm) {
                const auto centers =
                    detached_world_centers(*detached_arm);

                double min_y = std::numeric_limits<double>::infinity();
                for (const auto& p : centers) {
                    detached_centroid += p;
                    min_y = std::min(min_y, p.y);
                }
                detached_centroid =
                    detached_centroid / static_cast<double>(centers.size());

                detached_min_height =
                    std::min(detached_min_height, min_y);
                detached_frame_min_y = min_y;
                detached_rest_min_height = min_y;
                detached_final_height = detached_centroid.y;
                detached_travel =
                    sarx::length(detached_centroid - detached_initial_centroid);

                for (const auto index : detached_arm->component.voxel_indices) {
                    if (voxel_character.voxels()[index].state
                        == sarx::CharacterVoxelState::Attached) {
                        ++detached_voxels_reattached;
                    }
                }

                digest = fnv1a(digest, centers);

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

            // Fixed camera: no follow, no shake.
            camera.position = fixed_camera_position;
            camera.target = fixed_camera_target;

            sarx::write_character_ppm(
                frame_path(args.output, frame),
                visible,
                camera,
                true);

            frames_tsv
                << frame << '\t' << time << '\t'
                << (controller.active().motion_id.empty()
                    ? std::string("<none>")
                    : controller.active().motion_id)
                << '\t' << active_phase << '\t' << blend
                << '\t' << hand.x << '\t' << hand.y << '\t' << hand.z
                << '\t' << swept_distance
                << '\t' << pelvis.x << '\t' << pelvis.y << '\t' << pelvis.z
                << '\t' << lead_foot_drift
                << '\t' << detached_centroid.x
                << '\t' << detached_centroid.y
                << '\t' << detached_centroid.z
                << '\t' << detached_frame_min_y
                << '\t' << voxel_character.stats().attached_voxels
                << '\t' << authority.count(sarx::AnimationAuthoritySource::BaseAnimation)
                << '\t' << authority.count(sarx::AnimationAuthoritySource::ReplacementAnimation)
                << '\t' << authority.count(sarx::AnimationAuthoritySource::Physics)
                << '\n';
        }

        std::size_t unrelated_changed_voxels = 0;

        for (const auto& voxel : voxel_character.voxels()) {
            const bool injured_region =
                args.injury == "hand"
                ? (voxel.anatomical_region == "lowerarm_l"
                   || voxel.anatomical_region == "hand_l")
                : (voxel.anatomical_region == "upperarm_l"
                   || voxel.anatomical_region == "lowerarm_l"
                   || voxel.anatomical_region == "hand_l");
            if (voxel.state == sarx::CharacterVoxelState::Attached
                || injured_region) {
                continue;
            }
            ++unrelated_changed_voxels;
        }

        const double upperarm_l = voxel_character.attached_fraction("upperarm_l");
        const double lowerarm_l = voxel_character.attached_fraction("lowerarm_l");
        const double hand_l = voxel_character.attached_fraction("hand_l");
        const double upperarm_r = voxel_character.attached_fraction("upperarm_r");
        const double lowerarm_r = voxel_character.attached_fraction("lowerarm_r");
        const double hand_r = voxel_character.attached_fraction("hand_r");

        const auto stats = voxel_character.stats();

        const sarx::ActionSubstitutionPlan* substitution_plan =
            controller.transitions().empty()
            ? nullptr
            : &controller.transitions().front().plan;

        std::string elbow_rejection = "<not evaluated>";
        bool elbow_rejected_for_anatomy = false;
        bool elbow_blocked_on_authored_motion = false;

        if (substitution_plan) {
            for (const auto& entry : substitution_plan->trace) {
                if (entry.capability.family == sarx::ActionFamily::ElbowStrike
                    && entry.capability.side == sarx::ActionSide::Left) {
                    elbow_rejection =
                        std::string(sarx::action_candidate_disposition_name(
                            entry.disposition))
                        + ": " + entry.reason;
                    elbow_rejected_for_anatomy =
                        entry.disposition
                        == sarx::ActionCandidateDisposition::RejectedAnatomy;
                    elbow_blocked_on_authored_motion =
                        entry.disposition
                            == sarx::ActionCandidateDisposition::RejectedAuthoredMotionUnavailable
                        && substitution_plan->preferred_blocked_on_authored_motion
                        && substitution_plan->preferred_unavailable.capability.motion_id
                            == entry.capability.motion_id;
                }
            }
        }

        // Physics must own the whole detached branch; the derived
        // replacement root must be replacement-owned; the skeleton root
        // stays on base animation.
        // Every joint in the severed skeletal subtree must be physics-owned.
        bool physics_owns_branch = true;
        for (const auto& joint : joints) {
            std::string current = joint.name;
            bool in_severed_subtree = false;
            for (std::size_t guard = 0;
                 !current.empty() && guard <= joints.size();
                 ++guard) {
                if (current == severed_root) {
                    in_severed_subtree = true;
                    break;
                }
                const auto parent =
                    std::find_if(
                        joints.begin(), joints.end(),
                        [&](const sarx::CharacterJointInfo& j) {
                            return j.name == current;
                        });
                current = parent == joints.end() ? std::string() : parent->parent;
            }
            if (in_severed_subtree
                && sarx::authority_source_for(authority, joint.name)
                    != sarx::AnimationAuthoritySource::Physics) {
                physics_owns_branch = false;
            }
        }

        const std::string derived_substitute_root =
            substitute.authority_joint_roots.empty()
            ? std::string()
            : substitute.authority_joint_roots.front();

        const bool authority_ok =
            physics_owns_branch
            && !derived_substitute_root.empty()
            && sarx::authority_source_for(authority, derived_substitute_root)
                == sarx::AnimationAuthoritySource::ReplacementAnimation
            && sarx::authority_source_for(authority, substitute.effector_joint)
                == sarx::AnimationAuthoritySource::ReplacementAnimation
            && sarx::authority_source_for(authority, "root")
                == sarx::AnimationAuthoritySource::BaseAnimation
            && physics_pose_violations == 0;

        // "Whole arm lost" is judged against the same-side elbow's own
        // anatomical requirement: the proximal remnant must be too small
        // to carry an elbow strike.
        double left_elbow_upperarm_requirement = 0.0;
        for (const auto& requirement :
             library_entry(
                 library,
                 sarx::ActionFamily::ElbowStrike,
                 sarx::ActionSide::Left).required_regions) {
            if (requirement.region == "upperarm_l") {
                left_elbow_upperarm_requirement =
                    requirement.minimum_attached_fraction;
            }
        }

        const bool contact_in_window =
            contact_frame >= 0
            && contact_phase >= substitute.contact_phase_begin
            && contact_phase <= substitute.contact_phase_end;

        const bool continuity_ok =
            max_pelvis_frame_step <= pelvis_step_limit
            && max_root_displacement <= 1e-9
            && max_lead_foot_drift <= planted_foot_limit;

        // Declared: no detached voxel center may sink more than half a
        // voxel below the floor (its cube face may touch, not submerge), and
        // at rest the lowest voxel may not hover more than 1.5 voxels up.
        const bool detached_physical =
            detached_arm
            && detached_voxels_reattached == 0
            && detached_arm->articulation.ground_contacts() > 0
            && detached_min_height >= -args.voxel_size * 0.5
            && detached_rest_min_height <= args.voxel_size * 1.5;

        std::ostringstream summary;
        summary
            << "fixture="
            << (args.injury == "hand"
                ? "E1-A hand_loss_committed_jab"
                : "E1-B whole_arm_loss_to_opposite_cross") << '\n'
            << "elbow_binding="
            << (args.certified_elbow
                ? args.elbow_clip + " from " + args.elbow_animations
                : std::string("<none certified>")) << '\n'
            << "character=" << args.character << '\n'
            << "initial_action=" << jab_capability.motion_id << '\n'
            << "initial_effector="
            << sarx::action_effector_name(jab_capability.effector) << '\n'
            << "declared_sever_phase=" << declared_sever_phase
            << " (midpoint of commitment_phase="
            << jab_capability.commitment_phase
            << " and contact_phase_begin="
            << jab_capability.contact_phase_begin << ")\n"
            << "sever_frame=" << cut_frame << '\n'
            << "sever_phase_actual=" << actual_sever_phase << '\n'
            << "region_lost="
            << (args.injury == "hand"
                ? "left hand at wrist (hand_l + fingers)"
                : "left upper arm at shoulder (upperarm_l,lowerarm_l,hand_l)")
            << '\n'
            << "attachment_before=" << attachment_before << '\n'
            << "attachment_after=" << attachment_after << '\n'
            << "arm_detached_frame=" << detached_frame << '\n'
            << "physics_joint_roots=";
        for (const auto& r : physics_roots) summary << r << ' ';
        summary
            << '\n'
            << "original_action_invalidated_frame=" << jab_invalidated_frame << '\n'
            << "same_side_elbow=" << elbow_rejection << '\n'
            << "planner_trace="
            << (substitution_plan
                ? sarx::describe_action_plan(*substitution_plan)
                : std::string("<none>")) << '\n'
            << "selected_substitute=" << controller.active().motion_id << '\n'
            << "substitute_effector="
            << sarx::action_effector_name(controller.active().effector) << '\n'
            << "substitution_selected_frame=" << substitution_selected_frame << '\n'
            << "replacement_engaged_frame=" << replacement_engaged_frame << '\n';

        for (const auto& d : derivations) {
            summary << "authored_authority " << d.effector_joint
                    << " root=" << d.authority_root
                    << " travel=" << d.effector_travel
                    << " threshold=" << d.contribution_threshold
                    << " chain=";
            for (const auto& c : d.chain) {
                summary << c.joint << ':' << c.effector_contribution << ',';
            }
            summary << '\n';
        }

        summary
            << "authority_base_joints="
            << authority.count(sarx::AnimationAuthoritySource::BaseAnimation) << '\n'
            << "authority_replacement_joints="
            << authority.count(sarx::AnimationAuthoritySource::ReplacementAnimation) << '\n'
            << "authority_physics_joints="
            << authority.count(sarx::AnimationAuthoritySource::Physics) << '\n'
            << "authority_by_joint=";
        for (const auto& a : authority.joints) {
            summary << a.joint << ':'
                    << sarx::animation_authority_source_name(a.source) << ',';
        }
        summary
            << '\n'
            << "physics_pose_violations=" << physics_pose_violations << '\n'
            << "target=" << target.x << ',' << target.y << ',' << target.z << '\n'
            << "target_radius=" << target_radius << '\n'
            << "min_target_distance=" << min_target_distance << '\n'
            << "min_distance_substitute_phase=" << min_distance_phase << '\n'
            << "contact_frame=" << contact_frame << '\n'
            << "contact_substitute_phase=" << contact_phase << '\n'
            << "substitute_contact_window=" << substitute.contact_phase_begin
            << ',' << substitute.contact_phase_end << '\n'
            << "contact_velocity=" << contact_velocity << '\n'
            << "max_pelvis_frame_step=" << max_pelvis_frame_step
            << " limit=" << pelvis_step_limit << '\n'
            << "max_root_displacement=" << max_root_displacement << '\n'
            << "max_lead_foot_drift=" << max_lead_foot_drift
            << " limit=" << planted_foot_limit << '\n'
            << "camera_fixed=1\n"
            << "detached_arm_ground_contacts="
            << (detached_arm ? detached_arm->articulation.ground_contacts() : 0) << '\n'
            << "detached_arm_initial_max_anchor_speed=" << detached_initial_speed << '\n'
            << "detached_arm_travel=" << detached_travel << '\n'
            << "detached_arm_contact_radius=" << limb_contact_radius
            << " segment_radii=" << segment_radii_text << '\n'
            << "detached_arm_min_height=" << detached_min_height << '\n'
            << "detached_arm_rest_min_height=" << detached_rest_min_height << '\n'
            << "detached_arm_final_centroid_height=" << detached_final_height << '\n'
            << "detached_voxels_reattached=" << detached_voxels_reattached << '\n'
            << "detached_arm_elbow_delta="
            << (detached_arm && detached_arm->articulation.joint_count() > 0
                ? detached_arm->articulation.max_joint_angle_delta(0)
                : 0.0) << '\n'
            << "unrelated_changed_voxels=" << unrelated_changed_voxels << '\n'
            << "surviving_stump_voxels_distal_to_cut=" << distal_stump_voxels << '\n'
            << "total_voxels=" << stats.total_voxels
            << " destroyed=" << stats.destroyed_voxels
            << " detached=" << stats.detached_voxels << '\n'
            << "fractions upperarm_l=" << upperarm_l
            << " lowerarm_l=" << lowerarm_l
            << " hand_l=" << hand_l
            << " upperarm_r=" << upperarm_r
            << " lowerarm_r=" << lowerarm_r
            << " hand_r=" << hand_r << '\n'
            << "determinism_digest=" << std::hex << digest << std::dec << '\n'
            << "gate_authority=" << (authority_ok ? 1 : 0)
            << " gate_contact_in_window=" << (contact_in_window ? 1 : 0)
            << " gate_continuity=" << (continuity_ok ? 1 : 0)
            << " gate_detached_physical=" << (detached_physical ? 1 : 0)
            << " gate_elbow_rejected_for_anatomy="
            << (elbow_rejected_for_anatomy ? 1 : 0)
            << " elbow_blocked_on_authored_motion="
            << (elbow_blocked_on_authored_motion ? 1 : 0) << '\n';

        std::cout << summary.str();

        if (!args.evidence.empty()) {
            std::ofstream out(args.evidence);
            out << frames_tsv.str();
            std::ofstream summary_out(
                args.evidence.string() + ".summary.txt");
            summary_out << summary.str();
        }

        if (args.require_damage && destroyed_total == 0) {
            throw std::runtime_error("shoulder cut destroyed no voxels");
        }

        if (args.require_detachment && !detached_physical) {
            throw std::runtime_error(
                "whole left arm never detached into independent physics");
        }

        const bool elbow_selected =
            controller.active().family == sarx::ActionFamily::ElbowStrike
            && controller.active().side == sarx::ActionSide::Left
            && controller.active().authored_motion_available;

        const bool cross_selected =
            controller.active().family == sarx::ActionFamily::Punch
            && controller.active().side == sarx::ActionSide::Right
            && controller.active().motion_id == "Punch_Cross";

        const bool immediate =
            jab_invalidated_frame == detached_frame
            && substitution_selected_frame == jab_invalidated_frame;

        // Expected planner outcome per injury:
        //  whole-arm: elbow rejected for anatomy, opposite Cross selected.
        //  hand, elbow certified: same-side elbow selected.
        //  hand, no certified elbow: elbow is the preferred (anatomically
        //    viable) continuation but blocked on authored motion, so the
        //    planner falls back to the opposite Cross.
        const bool substitution_ok =
            immediate
            && (args.injury == "whole-arm"
                ? (elbow_rejected_for_anatomy && cross_selected)
                : (args.certified_elbow
                    ? elbow_selected
                    : (elbow_blocked_on_authored_motion && cross_selected)));

        if (args.require_substitution && !substitution_ok) {
            throw std::runtime_error(
                "planner did not produce the expected anatomy-driven substitution");
        }

        if (args.require_authored_elbow && !elbow_selected) {
            throw std::runtime_error(
                "E1-A requires a certified authored elbow strike; none is bound");
        }

        if (args.require_contact
            && (!contact_in_window
                || !std::isfinite(min_target_distance)
                || min_target_distance > target_radius)) {
            throw std::runtime_error(
                "opposite-hand Cross did not reach target inside its authored contact window");
        }

        if (args.require_authority && !authority_ok) {
            throw std::runtime_error(
                "whole-arm regional authority proof failed");
        }

        if (args.require_continuity && !continuity_ok) {
            throw std::runtime_error(
                "root/pelvis/planted-foot continuity failed");
        }

        double left_elbow_lowerarm_requirement = 0.0;
        for (const auto& requirement :
             library_entry(
                 library,
                 sarx::ActionFamily::ElbowStrike,
                 sarx::ActionSide::Left).required_regions) {
            if (requirement.region == "lowerarm_l") {
                left_elbow_lowerarm_requirement =
                    requirement.minimum_attached_fraction;
            }
        }

        const bool isolation_ok =
            unrelated_changed_voxels == 0
            && distal_stump_voxels == 0
            && hand_l <= 0.05
            && upperarm_r >= 0.95
            && lowerarm_r >= 0.95
            && hand_r >= 0.95
            && (args.injury == "hand"
                // The elbow chain must survive a wrist cut.
                ? (upperarm_l >= 0.95
                   && lowerarm_l >= left_elbow_lowerarm_requirement)
                : (upperarm_l < left_elbow_upperarm_requirement
                   && lowerarm_l <= 0.05));

        if (args.require_isolation && !isolation_ok) {
            throw std::runtime_error(
                "severance altered unrelated or surviving attack anatomy");
        }

        std::cout
            << "SARX attack substitution fixture complete: injury=" << args.injury
            << " frames="
            << args.frames << " output=" << args.output.string() << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_character_voxel_arm_loss_attack_demo: "
            << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
