#pragma once

#include "sarx/body.hpp"

#include <cstddef>
#include <vector>

namespace sarx {

struct MaterialRegion {
    Vec3 min{};
    Vec3 max{};
    MaterialId material{kDefaultMaterial};
    int priority{0};
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
    MaterialId joint_material = kDefaultMaterial);

} // namespace sarx
