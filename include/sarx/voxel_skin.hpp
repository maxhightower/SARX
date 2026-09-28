#pragma once

// EmbeddedVoxelSkin: a fine visual voxel representation driven by a coarse
// SARX physical lattice.
//
// Each detail voxel is bound to a local affine frame of four physical
// particles (its nearest particle plus three structurally linked neighbours
// in the same island). Every frame the voxel's centre and orientation follow
// that frame's deformation gradient, so rotated/stretched pieces stay solid.
//
// Damage carves the skin geometrically (the same DamageCommand SARX applies
// to the lattice), and voxels facing a carved hole or a torn topology
// boundary are flagged "exposed" so consumers can render wound interiors.
// The skin is appearance only: it never feeds back into physics.

#include "sarx/anatomical_humanoid.hpp"
#include "sarx/character_runtime.hpp"
#include "sarx/damage.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace sarx {

struct SkinCarveConfig {
    double capsule_radius_scale{0.6};
    double sphere_radius_scale{0.8};
    // Half-thickness of a plane-cut kerf, in voxel sizes.
    double kerf_voxels{0.9};
};

struct SkinVoxel {
    Vec3 local{};  // rest position in fixture space
    Vec3 rest{};   // rest position in world space
    int gx{};
    int gy{};
    int gz{};
    DetailTissue tissue{DetailTissue::Soft};

    bool alive{true};
    bool surface{false};  // any 6-neighbour missing, carved or in another island
    bool exposed{false};  // faces a carved hole or a torn island boundary

    ParticleId anchor{};
    Vec3 frame_coords{};  // coordinates in the anchor frame at rest

    // Updated by EmbeddedVoxelSkin::update() for surface voxels.
    Vec3 center{};
    std::array<Vec3, 3> axes{};  // deformed voxel edge vectors (right-handed)
};

class EmbeddedVoxelSkin {
public:
    void build(
        const CharacterRuntime& runtime,
        const DetailVoxelSet& detail,
        const Vec3& world_offset);

    // Recompute particle frames and surface flags after topology changes.
    void refresh_topology(const CharacterRuntime& runtime);

    // Removes voxels intersecting the damage geometry (current pose).
    // Returns the number of voxels removed. Call refresh_topology after.
    std::size_t carve(
        const CharacterRuntime& runtime,
        const DamageCommand& command,
        const SkinCarveConfig& config = {});

    // Deforms all surface voxels to the runtime's current particle state.
    void update(const CharacterRuntime& runtime);

    [[nodiscard]] const std::vector<SkinVoxel>& voxels() const { return voxels_; }
    [[nodiscard]] const std::vector<std::uint32_t>& surface() const { return surface_; }
    [[nodiscard]] double voxel_size() const { return voxel_size_; }
    [[nodiscard]] std::size_t alive_count() const;
    [[nodiscard]] IslandId island_of(const CharacterRuntime& runtime, std::size_t voxel) const {
        return runtime.island_of_particle(voxels_[voxel].anchor);
    }

    // World position of any alive voxel (surface or interior).
    [[nodiscard]] Vec3 world_position(const CharacterRuntime& runtime, std::size_t voxel) const;

private:
    struct Frame {
        std::array<ParticleId, 3> others{};
        // Inverse of the rest edge matrix (columns: others - anchor).
        std::array<Vec3, 3> inverse_rows{};
        bool valid{false};
    };

    void rebuild_frames(const CharacterRuntime& runtime);
    void rebuild_surface(const CharacterRuntime& runtime);
    void rebind(std::size_t voxel);

    double voxel_size_{0.02};
    std::vector<SkinVoxel> voxels_;
    std::vector<std::uint32_t> surface_;
    std::vector<Vec3> rest_particles_;
    std::vector<std::vector<ParticleId>> neighbours_;
    std::vector<std::vector<ConstraintId>> neighbour_links_;
    std::vector<Frame> frames_;

    int nx_{0};
    int ny_{0};
    int nz_{0};
    std::vector<std::int32_t> grid_;  // voxel index or -1
};

} // namespace sarx
