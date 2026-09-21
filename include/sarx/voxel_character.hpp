#pragma once

#include "sarx/gltf_character.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace sarx {

enum class CharacterVoxelState : std::uint8_t {
    Attached,
    Detached,
    Destroyed
};

struct CharacterVoxel {
    Vec3 rest_center{};
    Vec3 anchor_offset{};
    std::size_t anchor_vertex{};
    int grid_x{};
    int grid_y{};
    int grid_z{};
    double damage{0.0};
    double break_damage{1.0};
    CharacterVoxelState state{CharacterVoxelState::Attached};
};

struct DetachedVoxelComponent {
    std::vector<std::size_t> voxel_indices;
};

struct VoxelizedCharacterStats {
    std::size_t total_voxels{};
    std::size_t active_voxels{};
    std::size_t attached_voxels{};
    std::size_t detached_voxels{};
    std::size_t destroyed_voxels{};
    double voxel_size{};
};

class VoxelizedCharacter {
public:
    void build(
        const CharacterMeshFrame& rest_mesh,
        double voxel_size = 0.055);

    [[nodiscard]] CharacterMeshFrame render(
        const CharacterMeshFrame& animated_mesh) const;

    [[nodiscard]] CharacterMeshFrame render_component(
        const DetachedVoxelComponent& component,
        const std::vector<Vec3>& world_centers) const;

    std::size_t damage_sphere(
        const CharacterMeshFrame& animated_mesh,
        const Vec3& center,
        double radius,
        double damage = 1.0);

    std::size_t damage_cut_disk(
        const CharacterMeshFrame& animated_mesh,
        const Vec3& center,
        const Vec3& normal,
        double half_thickness,
        double radius,
        double damage = 1.0);

    [[nodiscard]] std::optional<DetachedVoxelComponent>
    detach_component_near(
        const CharacterMeshFrame& animated_mesh,
        const Vec3& seed_world_point,
        std::size_t minimum_voxels = 4);

    [[nodiscard]] Vec3 voxel_center(
        std::size_t voxel_index,
        const CharacterMeshFrame& animated_mesh) const;

    [[nodiscard]] VoxelizedCharacterStats stats() const;

    [[nodiscard]] const std::vector<CharacterVoxel>& voxels() const {
        return voxels_;
    }

private:
    [[nodiscard]] Vec3 current_center(
        const CharacterVoxel& voxel,
        const CharacterMeshFrame& animated_mesh) const;

    double voxel_size_{0.055};
    std::vector<CharacterVoxel> voxels_;
};

} // namespace sarx
