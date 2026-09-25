#pragma once

#include "sarx/math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sarx {

struct CharacterMeshFrame {
    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
    // Optional evidence tag per triangle (0 = body, 1 = physics-owned
    // detached anatomy, 2 = target). Empty means every triangle is body.
    std::vector<std::uint8_t> triangle_tags;
};

struct CharacterSplitFrame {
    CharacterMeshFrame body;
    CharacterMeshFrame detached;
    std::size_t boundary_triangles_removed{};
};

struct CharacterJointInfo {
    std::string name;
    std::string parent;
    Vec3 rest_world_position{};
};

struct CharacterNodeLocalPose {
    std::string name;
    std::string parent;
    Vec3 translation{};
    std::array<double, 4> rotation{0.0, 0.0, 0.0, 1.0};
    Vec3 scale{1.0, 1.0, 1.0};
};

struct CharacterPointInfluence {
    int joint_node{-1};
    std::string joint_name;
    Vec3 joint_local_point{};
    double weight{};
};

struct CharacterPointBinding {
    std::array<CharacterPointInfluence, 4> influences{};
    std::size_t influence_count{};
    std::string dominant_joint;
};

struct CharacterAssetStats {
    std::size_t vertices{};
    std::size_t triangles{};
    std::size_t character_nodes{};
    std::size_t skin_joints{};
    std::size_t animation_clips{};
};

class GltfCharacter {
public:
    GltfCharacter();
    ~GltfCharacter();

    GltfCharacter(GltfCharacter&&) noexcept;
    GltfCharacter& operator=(GltfCharacter&&) noexcept;

    GltfCharacter(const GltfCharacter&) = delete;
    GltfCharacter& operator=(const GltfCharacter&) = delete;

    void load(
        const std::string& character_glb,
        const std::string& animation_glb);

    [[nodiscard]] const CharacterAssetStats& stats() const;
    [[nodiscard]] const std::vector<std::string>& animation_names() const;
    [[nodiscard]] const std::vector<CharacterJointInfo>& skin_joints() const;

    [[nodiscard]] std::size_t find_animation(
        const std::string& name_fragment) const;

    [[nodiscard]] double animation_duration(
        std::size_t animation) const;

    [[nodiscard]] std::vector<std::string>
    animation_target_nodes(
        std::size_t animation) const;

    [[nodiscard]] CharacterMeshFrame sample(
        std::size_t animation,
        double time_seconds,
        bool loop = true,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] std::vector<CharacterNodeLocalPose>
    sample_node_local_poses(
        std::size_t animation,
        double time_seconds,
        bool loop = true) const;

    [[nodiscard]] CharacterMeshFrame
    sample_with_node_local_poses(
        const std::vector<CharacterNodeLocalPose>& node_poses,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] Vec3 node_world_position_with_local_poses(
        const std::vector<CharacterNodeLocalPose>& node_poses,
        const std::string& joint_fragment,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] CharacterSplitFrame sample_split_branch(
        std::size_t animation,
        double time_seconds,
        const std::string& detached_root_joint_fragment,
        bool loop = true,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] std::vector<CharacterPointBinding>
    bind_points_to_skin(
        std::size_t animation,
        double time_seconds,
        const std::vector<Vec3>& world_points,
        bool loop = true) const;

    [[nodiscard]] std::vector<Vec3>
    sample_bound_points(
        const std::vector<CharacterPointBinding>& bindings,
        std::size_t animation,
        double time_seconds,
        bool loop = true,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] std::vector<Vec3>
    sample_bound_points_with_node_local_poses(
        const std::vector<CharacterPointBinding>& bindings,
        const std::vector<CharacterNodeLocalPose>& node_poses,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] double binding_branch_weight(
        const CharacterPointBinding& binding,
        const std::string& root_joint_fragment) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sarx
