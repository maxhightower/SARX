#pragma once

#include "sarx/body.hpp"

#include <cstddef>
#include <vector>

namespace sarx {

enum class RegionShape {
    Box,
    Sphere,
    Capsule
};

struct MaterialRegion {
    RegionShape shape{RegionShape::Box};

    // Box representation.
    Vec3 min{};
    Vec3 max{};

    // Sphere representation.
    Vec3 center{};
    double radius{0.0};

    // Capsule representation.
    Vec3 a{};
    Vec3 b{};

    MaterialId material{kDefaultMaterial};
    int priority{0};

    // Optional rest-space anatomical fiber direction.
    Vec3 fiber_direction{};
};

struct VoxelLatticeSpec {
    Vec3 origin{};
    std::size_t nx{2};
    std::size_t ny{2};
    std::size_t nz{2};
    double spacing{0.1};
    double particle_mass{1.0};

    MaterialId default_material{kDefaultMaterial};
    double structural_compliance{0.0};
    double structural_break_damage{1.0};

    // Axial + face/body diagonal links improve isotropy over a 6-neighbor grid.
    bool include_diagonals{true};

    // Six deterministic tetrahedra per lattice cell, sharing the 000->111 diagonal.
    bool include_tetrahedra{true};
    double volume_compliance{0.0};
    double volume_break_damage{1.0};
};

class VoxelLattice {
public:
    Body body;
    Vec3 origin{};
    std::size_t nx{};
    std::size_t ny{};
    std::size_t nz{};
    double spacing{};

    std::vector<MaterialId> particle_materials;
    std::vector<Vec3> particle_fibers;

    [[nodiscard]] ParticleId particle(
        std::size_t x,
        std::size_t y,
        std::size_t z) const;

    [[nodiscard]] std::size_t particle_count() const {
        return particle_materials.size();
    }
};

struct EmbeddedBoneResult {
    BoneId bone{kNoParent};
    std::vector<ConstraintId> attachments;
};

[[nodiscard]] VoxelLattice build_voxel_lattice(
    const VoxelLatticeSpec& spec,
    const std::vector<MaterialRegion>& regions = {});

[[nodiscard]] EmbeddedBoneResult embed_bone(
    VoxelLattice& lattice,
    BoneId parent,
    const Vec3& animated_position,
    double influence_radius,
    double attachment_compliance = 1e-7,
    double attachment_break_damage = 1.0,
    MaterialId attachment_material = kDefaultMaterial,
    double joint_break_damage = 1.0,
    MaterialId joint_material = kDefaultMaterial,
    double joint_radius = 0.0);

} // namespace sarx
