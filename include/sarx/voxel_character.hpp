#pragma once

#include "sarx/gltf_character.hpp"

#include <cstddef>
#include <vector>

namespace sarx {

struct CharacterVoxel {
    Vec3 rest_center{};
    Vec3 anchor_offset{};
    std::size_t anchor_vertex{};
    double damage{0.0};
    double break_damage{1.0};
    bool active{true};
};

struct VoxelizedCharacterStats {
    std::size_t total_voxels{};
    std::size_t active_voxels{};
    double voxel_size{};
};

class VoxelizedCharacter {
public:
    void build(
        const CharacterMeshFrame& rest_mesh,
        double voxel_size = 0.055);

    [[nodiscard]] CharacterMeshFrame render(
        const CharacterMeshFrame& animated_mesh) const;

    std::size_t damage_sphere(
        const CharacterMeshFrame& animated_mesh,
        const Vec3& center,
        double radius,
        double damage = 1.0);

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
