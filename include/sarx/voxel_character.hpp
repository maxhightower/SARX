#pragma once

#include "sarx/gltf_character.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace sarx {

enum class CharacterVoxelState : std::uint8_t {
    Attached,
    Detached,
    Destroyed
};

struct CharacterVoxel {
    Vec3 rest_center{};
    CharacterPointBinding skin_binding{};
    std::string anatomical_region{"other"};
    int grid_x{};
    int grid_y{};
    int grid_z{};
    double damage{0.0};
    double break_damage{1.0};
    CharacterVoxelState state{CharacterVoxelState::Attached};
};

struct DetachedVoxelComponent {
    std::vector<std::size_t> voxel_indices;
    std::string anatomical_region;
};

struct AnatomicalAvailability {
    std::string region;
    std::size_t total_voxels{};
    std::size_t attached_voxels{};

    [[nodiscard]] double attached_fraction() const {
        return total_voxels > 0
            ? static_cast<double>(attached_voxels)
                / static_cast<double>(total_voxels)
            : 1.0;
    }
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
        const GltfCharacter& character,
        std::size_t animation,
        double binding_time_seconds,
        double voxel_size = 0.055);

    [[nodiscard]] std::vector<Vec3> sample_centers(
        const GltfCharacter& character,
        std::size_t animation,
        double time_seconds,
        bool loop = true,
        const Vec3& world_offset = {}) const;

    [[nodiscard]] CharacterMeshFrame render(
        const std::vector<Vec3>& world_centers) const;

    [[nodiscard]] CharacterMeshFrame render_component(
        const DetachedVoxelComponent& component,
        const std::vector<Vec3>& world_centers) const;

    std::size_t damage_sphere(
        const std::vector<Vec3>& world_centers,
        const Vec3& center,
        double radius,
        double damage = 1.0,
        const std::vector<std::string>& allowed_regions = {});

    std::size_t damage_cut_disk(
        const std::vector<Vec3>& world_centers,
        const Vec3& center,
        const Vec3& normal,
        double half_thickness,
        double radius,
        double damage = 1.0,
        const std::vector<std::string>& allowed_regions = {});

    std::size_t damage_anatomical_interface(
        const std::string& distal_region,
        const std::vector<std::string>& proximal_regions,
        double damage = 1.0);

    [[nodiscard]] std::optional<DetachedVoxelComponent>
    detach_anatomical_region_if_disconnected(
        const std::string& anatomical_region,
        const std::vector<std::string>& proximal_regions,
        std::size_t minimum_voxels = 4);

    [[nodiscard]] std::optional<DetachedVoxelComponent>
    detach_component_near_anatomical(
        const std::vector<Vec3>& world_centers,
        const Vec3& seed_world_point,
        std::size_t minimum_voxels = 4);

    [[nodiscard]] Vec3 voxel_center(
        std::size_t voxel_index,
        const std::vector<Vec3>& world_centers) const;

    [[nodiscard]] VoxelizedCharacterStats stats() const;

    [[nodiscard]] std::vector<AnatomicalAvailability>
    anatomy_availability() const;

    [[nodiscard]] double attached_fraction(
        const std::string& anatomical_region) const;

    [[nodiscard]] const std::vector<CharacterVoxel>& voxels() const {
        return voxels_;
    }

private:
    double voxel_size_{0.055};
    std::vector<CharacterVoxel> voxels_;
};

} // namespace sarx
