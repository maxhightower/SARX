#include "sarx/voxel_character.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace sarx {
namespace {

struct Bounds {
    Vec3 min{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()};
    Vec3 max{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()};
};

Bounds bounds_of(const CharacterMeshFrame& mesh) {
    if (mesh.positions.empty()) {
        throw std::invalid_argument(
            "cannot voxelize an empty character mesh");
    }

    Bounds bounds;
    for (const Vec3& p : mesh.positions) {
        bounds.min.x = std::min(bounds.min.x, p.x);
        bounds.min.y = std::min(bounds.min.y, p.y);
        bounds.min.z = std::min(bounds.min.z, p.z);
        bounds.max.x = std::max(bounds.max.x, p.x);
        bounds.max.y = std::max(bounds.max.y, p.y);
        bounds.max.z = std::max(bounds.max.z, p.z);
    }
    return bounds;
}

bool ray_triangle(
    const Vec3& origin,
    const Vec3& direction,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    double& distance) {

    constexpr double epsilon = 1e-10;
    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 p = cross(direction, ac);
    const double det = dot(ab, p);

    if (std::abs(det) <= epsilon) {
        return false;
    }

    const double inv_det = 1.0 / det;
    const Vec3 t = origin - a;
    const double u = dot(t, p) * inv_det;
    if (u < -epsilon || u > 1.0 + epsilon) {
        return false;
    }

    const Vec3 q = cross(t, ab);
    const double v = dot(direction, q) * inv_det;
    if (v < -epsilon || u + v > 1.0 + epsilon) {
        return false;
    }

    const double hit = dot(ac, q) * inv_det;
    if (hit <= epsilon) {
        return false;
    }

    distance = hit;
    return true;
}

bool point_inside_mesh(
    const CharacterMeshFrame& mesh,
    const Vec3& point) {

    const Vec3 direction =
        normalized(Vec3{1.0, 0.000371, 0.000917});

    std::size_t hits = 0;
    for (std::size_t tri = 0;
         tri + 2 < mesh.indices.size();
         tri += 3) {

        const auto i0 = mesh.indices[tri + 0];
        const auto i1 = mesh.indices[tri + 1];
        const auto i2 = mesh.indices[tri + 2];

        if (i0 >= mesh.positions.size()
            || i1 >= mesh.positions.size()
            || i2 >= mesh.positions.size()) {
            continue;
        }

        double distance = 0.0;
        if (ray_triangle(
                point,
                direction,
                mesh.positions[i0],
                mesh.positions[i1],
                mesh.positions[i2],
                distance)) {
            ++hits;
        }
    }

    return (hits % 2u) == 1u;
}

Vec3 closest_point_triangle(
    const Vec3& p,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c) {

    const Vec3 ab = b - a;
    const Vec3 ac = c - a;
    const Vec3 ap = p - a;

    const double d1 = dot(ab, ap);
    const double d2 = dot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) return a;

    const Vec3 bp = p - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) return b;

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return a + ab * v;
    }

    const Vec3 cp = p - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) return c;

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return a + ac * w;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0
        && (d4 - d3) >= 0.0
        && (d5 - d6) >= 0.0) {

        const Vec3 bc = c - b;
        const double w =
            (d4 - d3)
            / ((d4 - d3) + (d5 - d6));
        return b + bc * w;
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return a + ab * v + ac * w;
}

bool near_surface(
    const CharacterMeshFrame& mesh,
    const Vec3& point,
    double radius) {

    const double radius_squared = radius * radius;

    for (std::size_t tri = 0;
         tri + 2 < mesh.indices.size();
         tri += 3) {

        const auto i0 = mesh.indices[tri + 0];
        const auto i1 = mesh.indices[tri + 1];
        const auto i2 = mesh.indices[tri + 2];

        if (i0 >= mesh.positions.size()
            || i1 >= mesh.positions.size()
            || i2 >= mesh.positions.size()) {
            continue;
        }

        const Vec3 closest =
            closest_point_triangle(
                point,
                mesh.positions[i0],
                mesh.positions[i1],
                mesh.positions[i2]);

        if (length_squared(closest - point)
            <= radius_squared) {
            return true;
        }
    }

    return false;
}

std::size_t nearest_vertex(
    const CharacterMeshFrame& mesh,
    const Vec3& point) {

    double best =
        std::numeric_limits<double>::infinity();
    std::size_t best_index = 0;

    for (std::size_t i = 0;
         i < mesh.positions.size();
         ++i) {

        const double d2 =
            length_squared(
                mesh.positions[i] - point);

        if (d2 < best) {
            best = d2;
            best_index = i;
        }
    }

    return best_index;
}

void append_cube(
    CharacterMeshFrame& mesh,
    const Vec3& center,
    double half) {

    const std::uint32_t base =
        static_cast<std::uint32_t>(
            mesh.positions.size());

    const std::array<Vec3, 8> corners{{
        {-half, -half, -half},
        { half, -half, -half},
        { half,  half, -half},
        {-half,  half, -half},
        {-half, -half,  half},
        { half, -half,  half},
        { half,  half,  half},
        {-half,  half,  half}
    }};

    for (const Vec3& corner : corners) {
        mesh.positions.push_back(
            center + corner);
    }

    constexpr std::uint32_t triangles[] = {
        0, 2, 1, 0, 3, 2,
        4, 5, 6, 4, 6, 7,
        0, 1, 5, 0, 5, 4,
        3, 7, 6, 3, 6, 2,
        0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5
    };

    for (const std::uint32_t index : triangles) {
        mesh.indices.push_back(base + index);
    }
}

} // namespace

void VoxelizedCharacter::build(
    const CharacterMeshFrame& rest_mesh,
    double voxel_size) {

    if (voxel_size <= 0.0) {
        throw std::invalid_argument(
            "character voxel size must be positive");
    }
    if (rest_mesh.indices.size() % 3 != 0) {
        throw std::invalid_argument(
            "character voxelization requires triangle indices");
    }

    voxel_size_ = voxel_size;
    voxels_.clear();

    const Bounds bounds = bounds_of(rest_mesh);
    const Vec3 padding{
        voxel_size,
        voxel_size,
        voxel_size};

    const Vec3 grid_min =
        bounds.min - padding;
    const Vec3 grid_max =
        bounds.max + padding;

    const std::size_t nx =
        static_cast<std::size_t>(
            std::ceil(
                (grid_max.x - grid_min.x)
                / voxel_size_))
        + 1;

    const std::size_t ny =
        static_cast<std::size_t>(
            std::ceil(
                (grid_max.y - grid_min.y)
                / voxel_size_))
        + 1;

    const std::size_t nz =
        static_cast<std::size_t>(
            std::ceil(
                (grid_max.z - grid_min.z)
                / voxel_size_))
        + 1;

    const std::size_t total =
        nx * ny * nz;

    if (total > 400000) {
        throw std::runtime_error(
            "character voxelization grid is unexpectedly large");
    }

    const double surface_radius =
        voxel_size_ * 0.62;

    for (std::size_t z = 0; z < nz; ++z) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = 0; x < nx; ++x) {

                const Vec3 center{
                    grid_min.x
                        + (static_cast<double>(x) + 0.5)
                            * voxel_size_,
                    grid_min.y
                        + (static_cast<double>(y) + 0.5)
                            * voxel_size_,
                    grid_min.z
                        + (static_cast<double>(z) + 0.5)
                            * voxel_size_
                };

                const bool occupied =
                    point_inside_mesh(
                        rest_mesh,
                        center)
                    || near_surface(
                        rest_mesh,
                        center,
                        surface_radius);

                if (!occupied) {
                    continue;
                }

                const std::size_t anchor =
                    nearest_vertex(
                        rest_mesh,
                        center);

                CharacterVoxel voxel;
                voxel.rest_center = center;
                voxel.anchor_vertex = anchor;
                voxel.anchor_offset =
                    center
                    - rest_mesh.positions[anchor];

                voxels_.push_back(voxel);
            }
        }
    }

    if (voxels_.empty()) {
        throw std::runtime_error(
            "Quaternius voxelization produced no voxels");
    }
}

Vec3 VoxelizedCharacter::current_center(
    const CharacterVoxel& voxel,
    const CharacterMeshFrame& animated_mesh) const {

    if (voxel.anchor_vertex
        >= animated_mesh.positions.size()) {
        throw std::out_of_range(
            "animated character mesh changed vertex count");
    }

    return
        animated_mesh.positions[
            voxel.anchor_vertex]
        + voxel.anchor_offset;
}

CharacterMeshFrame VoxelizedCharacter::render(
    const CharacterMeshFrame& animated_mesh) const {

    CharacterMeshFrame out;

    const std::size_t active =
        static_cast<std::size_t>(
            std::count_if(
                voxels_.begin(),
                voxels_.end(),
                [](const CharacterVoxel& voxel) {
                    return voxel.active;
                }));

    out.positions.reserve(
        active * 8);
    out.indices.reserve(
        active * 36);

    const double half =
        voxel_size_ * 0.47;

    for (const CharacterVoxel& voxel : voxels_) {
        if (!voxel.active) {
            continue;
        }

        append_cube(
            out,
            current_center(
                voxel,
                animated_mesh),
            half);
    }

    return out;
}

std::size_t VoxelizedCharacter::damage_sphere(
    const CharacterMeshFrame& animated_mesh,
    const Vec3& center,
    double radius,
    double damage) {

    if (radius <= 0.0 || damage < 0.0) {
        throw std::invalid_argument(
            "invalid character voxel damage sphere");
    }

    const double radius_squared =
        radius * radius;

    std::size_t destroyed = 0;

    for (CharacterVoxel& voxel : voxels_) {
        if (!voxel.active) {
            continue;
        }

        const Vec3 position =
            current_center(
                voxel,
                animated_mesh);

        if (length_squared(
                position - center)
            > radius_squared) {
            continue;
        }

        voxel.damage += damage;
        if (voxel.damage
            >= voxel.break_damage) {
            voxel.active = false;
            ++destroyed;
        }
    }

    return destroyed;
}

VoxelizedCharacterStats VoxelizedCharacter::stats() const {
    VoxelizedCharacterStats out;
    out.total_voxels = voxels_.size();
    out.active_voxels =
        static_cast<std::size_t>(
            std::count_if(
                voxels_.begin(),
                voxels_.end(),
                [](const CharacterVoxel& voxel) {
                    return voxel.active;
                }));
    out.voxel_size = voxel_size_;
    return out;
}

} // namespace sarx
