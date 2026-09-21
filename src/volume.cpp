#include "sarx/volume.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sarx {
namespace {

bool contains(const MaterialRegion& region, const Vec3& p) {
    return p.x >= region.min.x && p.x <= region.max.x
        && p.y >= region.min.y && p.y <= region.max.y
        && p.z >= region.min.z && p.z <= region.max.z;
}

MaterialId material_at(
    const Vec3& p,
    MaterialId fallback,
    const std::vector<MaterialRegion>& regions) {

    MaterialId result = fallback;
    int best_priority = std::numeric_limits<int>::min();

    for (const auto& region : regions) {
        if (!contains(region, p)) continue;
        if (region.priority < best_priority) continue;
        best_priority = region.priority;
        result = region.material;
    }
    return result;
}

Vec3 node_position(
    const VoxelLatticeSpec& spec,
    std::size_t x,
    std::size_t y,
    std::size_t z) {

    return spec.origin + Vec3{
        static_cast<double>(x) * spec.spacing,
        static_cast<double>(y) * spec.spacing,
        static_cast<double>(z) * spec.spacing
    };
}

} // namespace

ParticleId VoxelLattice::particle(
    std::size_t x,
    std::size_t y,
    std::size_t z) const {

    if (x >= nx || y >= ny || z >= nz) {
        throw std::out_of_range("voxel lattice particle index out of range");
    }
    return x + nx * (y + ny * z);
}

VoxelLattice build_voxel_lattice(
    const VoxelLatticeSpec& spec,
    const std::vector<MaterialRegion>& regions) {

    if (spec.nx < 2 || spec.ny < 2 || spec.nz < 2) {
        throw std::invalid_argument("voxel lattice dimensions must be >= 2");
    }
    if (spec.spacing <= 0.0 || spec.particle_mass <= 0.0) {
        throw std::invalid_argument("voxel lattice spacing/mass must be positive");
    }
    if (spec.structural_compliance < 0.0
        || spec.structural_break_damage <= 0.0
        || spec.volume_compliance < 0.0
        || spec.volume_break_damage <= 0.0) {
        throw std::invalid_argument("invalid voxel lattice structural parameters");
    }

    VoxelLattice lattice;
    lattice.origin = spec.origin;
    lattice.nx = spec.nx;
    lattice.ny = spec.ny;
    lattice.nz = spec.nz;
    lattice.spacing = spec.spacing;
    lattice.particle_materials.reserve(spec.nx * spec.ny * spec.nz);

    for (std::size_t z = 0; z < spec.nz; ++z) {
        for (std::size_t y = 0; y < spec.ny; ++y) {
            for (std::size_t x = 0; x < spec.nx; ++x) {
                const Vec3 p = node_position(spec, x, y, z);
                lattice.body.add_particle(p, spec.particle_mass);
                lattice.particle_materials.push_back(
                    material_at(p, spec.default_material, regions));
            }
        }
    }

    std::vector<std::array<int, 3>> offsets;
    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                // Keep one direction from each symmetric pair.
                if (dz < 0
                    || (dz == 0 && dy < 0)
                    || (dz == 0 && dy == 0 && dx <= 0)) {
                    continue;
                }

                const int manhattan = std::abs(dx) + std::abs(dy) + std::abs(dz);
                if (!spec.include_diagonals && manhattan != 1) {
                    continue;
                }
                offsets.push_back({dx, dy, dz});
            }
        }
    }

    for (std::size_t z = 0; z < spec.nz; ++z) {
        for (std::size_t y = 0; y < spec.ny; ++y) {
            for (std::size_t x = 0; x < spec.nx; ++x) {
                const ParticleId a = lattice.particle(x, y, z);
                const Vec3 p0 = lattice.body.particles()[a].position;

                for (const auto& offset : offsets) {
                    const int bx = static_cast<int>(x) + offset[0];
                    const int by = static_cast<int>(y) + offset[1];
                    const int bz = static_cast<int>(z) + offset[2];

                    if (bx < 0 || by < 0 || bz < 0
                        || bx >= static_cast<int>(spec.nx)
                        || by >= static_cast<int>(spec.ny)
                        || bz >= static_cast<int>(spec.nz)) {
                        continue;
                    }

                    const ParticleId b = lattice.particle(
                        static_cast<std::size_t>(bx),
                        static_cast<std::size_t>(by),
                        static_cast<std::size_t>(bz));
                    const Vec3 p1 = lattice.body.particles()[b].position;
                    const Vec3 mid = (p0 + p1) * 0.5;
                    const MaterialId material =
                        material_at(mid, spec.default_material, regions);

                    lattice.body.add_structural_constraint(
                        a,
                        b,
                        spec.structural_compliance,
                        spec.structural_break_damage,
                        material);
                }
            }
        }
    }

    if (spec.include_tetrahedra) {
        for (std::size_t z = 0; z + 1 < spec.nz; ++z) {
            for (std::size_t y = 0; y + 1 < spec.ny; ++y) {
                for (std::size_t x = 0; x + 1 < spec.nx; ++x) {
                    const ParticleId v000 = lattice.particle(x, y, z);
                    const ParticleId v100 = lattice.particle(x + 1, y, z);
                    const ParticleId v010 = lattice.particle(x, y + 1, z);
                    const ParticleId v110 = lattice.particle(x + 1, y + 1, z);
                    const ParticleId v001 = lattice.particle(x, y, z + 1);
                    const ParticleId v101 = lattice.particle(x + 1, y, z + 1);
                    const ParticleId v011 = lattice.particle(x, y + 1, z + 1);
                    const ParticleId v111 = lattice.particle(x + 1, y + 1, z + 1);

                    const Vec3 cell_center =
                        spec.origin + Vec3{
                            (static_cast<double>(x) + 0.5) * spec.spacing,
                            (static_cast<double>(y) + 0.5) * spec.spacing,
                            (static_cast<double>(z) + 0.5) * spec.spacing
                        };
                    const MaterialId material =
                        material_at(cell_center, spec.default_material, regions);

                    const std::array<std::array<ParticleId, 4>, 6> tets{{
                        {v000, v100, v110, v111},
                        {v000, v110, v010, v111},
                        {v000, v010, v011, v111},
                        {v000, v011, v001, v111},
                        {v000, v001, v101, v111},
                        {v000, v101, v100, v111}
                    }};

                    for (const auto& tet : tets) {
                        lattice.body.add_tetrahedral_constraint(
                            tet[0],
                            tet[1],
                            tet[2],
                            tet[3],
                            spec.volume_compliance,
                            spec.volume_break_damage,
                            material);
                    }
                }
            }
        }
    }

    return lattice;
}

EmbeddedBoneResult embed_bone(
    VoxelLattice& lattice,
    BoneId parent,
    const Vec3& animated_position,
    double influence_radius,
    double attachment_compliance,
    double attachment_break_damage,
    MaterialId attachment_material,
    double joint_break_damage,
    MaterialId joint_material,
    double joint_radius) {

    if (influence_radius <= 0.0) {
        throw std::invalid_argument("bone influence radius must be positive");
    }

    EmbeddedBoneResult result;
    result.bone = lattice.body.add_bone(
        parent,
        animated_position,
        joint_break_damage,
        joint_material,
        joint_radius);

    const double radius_sq = influence_radius * influence_radius;

    for (ParticleId id = 0; id < lattice.body.particles().size(); ++id) {
        const Vec3 p = lattice.body.particles()[id].position;
        if (length_squared(p - animated_position) > radius_sq) {
            continue;
        }

        result.attachments.push_back(lattice.body.add_attachment(
            id,
            result.bone,
            p - animated_position,
            attachment_compliance,
            attachment_break_damage,
            attachment_material));
    }

    return result;
}

} // namespace sarx
