#include "sarx/voxel_character.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace sarx {
namespace {

struct Bounds {
    Vec3 min{
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::infinity()
    };

    Vec3 max{
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity()
    };
};

Bounds bounds_of(
    const CharacterMeshFrame& mesh) {

    if (mesh.positions.empty()) {
        throw std::invalid_argument(
            "cannot voxelize an empty character mesh");
    }

    Bounds bounds;

    for (const Vec3& p : mesh.positions) {
        bounds.min.x =
            std::min(bounds.min.x, p.x);
        bounds.min.y =
            std::min(bounds.min.y, p.y);
        bounds.min.z =
            std::min(bounds.min.z, p.z);

        bounds.max.x =
            std::max(bounds.max.x, p.x);
        bounds.max.y =
            std::max(bounds.max.y, p.y);
        bounds.max.z =
            std::max(bounds.max.z, p.z);
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
    const Vec3 p =
        cross(direction, ac);

    const double determinant =
        dot(ab, p);

    if (std::abs(determinant)
        <= epsilon) {
        return false;
    }

    const double inverse_determinant =
        1.0 / determinant;

    const Vec3 t =
        origin - a;

    const double u =
        dot(t, p)
        * inverse_determinant;

    if (u < -epsilon
        || u > 1.0 + epsilon) {
        return false;
    }

    const Vec3 q =
        cross(t, ab);

    const double v =
        dot(direction, q)
        * inverse_determinant;

    if (v < -epsilon
        || u + v > 1.0 + epsilon) {
        return false;
    }

    const double hit =
        dot(ac, q)
        * inverse_determinant;

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
        normalized(
            Vec3{
                1.0,
                0.000371,
                0.000917
            });

    std::size_t hits = 0;

    for (std::size_t tri = 0;
         tri + 2 < mesh.indices.size();
         tri += 3) {

        const auto i0 =
            mesh.indices[tri + 0];
        const auto i1 =
            mesh.indices[tri + 1];
        const auto i2 =
            mesh.indices[tri + 2];

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

    if (d1 <= 0.0
        && d2 <= 0.0) {
        return a;
    }

    const Vec3 bp = p - b;
    const double d3 = dot(ab, bp);
    const double d4 = dot(ac, bp);

    if (d3 >= 0.0
        && d4 <= d3) {
        return b;
    }

    const double vc =
        d1 * d4 - d3 * d2;

    if (vc <= 0.0
        && d1 >= 0.0
        && d3 <= 0.0) {

        const double v =
            d1 / (d1 - d3);

        return a + ab * v;
    }

    const Vec3 cp = p - c;
    const double d5 = dot(ab, cp);
    const double d6 = dot(ac, cp);

    if (d6 >= 0.0
        && d5 <= d6) {
        return c;
    }

    const double vb =
        d5 * d2 - d1 * d6;

    if (vb <= 0.0
        && d2 >= 0.0
        && d6 <= 0.0) {

        const double w =
            d2 / (d2 - d6);

        return a + ac * w;
    }

    const double va =
        d3 * d6 - d5 * d4;

    if (va <= 0.0
        && (d4 - d3) >= 0.0
        && (d5 - d6) >= 0.0) {

        const Vec3 bc = c - b;

        const double w =
            (d4 - d3)
            / ((d4 - d3)
               + (d5 - d6));

        return b + bc * w;
    }

    const double denominator =
        1.0 / (va + vb + vc);

    const double v =
        vb * denominator;

    const double w =
        vc * denominator;

    return
        a + ab * v + ac * w;
}

bool near_surface(
    const CharacterMeshFrame& mesh,
    const Vec3& point,
    double radius) {

    const double radius_squared =
        radius * radius;

    for (std::size_t tri = 0;
         tri + 2 < mesh.indices.size();
         tri += 3) {

        const auto i0 =
            mesh.indices[tri + 0];
        const auto i1 =
            mesh.indices[tri + 1];
        const auto i2 =
            mesh.indices[tri + 2];

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

        if (length_squared(
                closest - point)
            <= radius_squared) {
            return true;
        }
    }

    return false;
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

    for (const Vec3& corner
         : corners) {
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

    for (const std::uint32_t index
         : triangles) {
        mesh.indices.push_back(
            base + index);
    }
}

bool region_allowed(
    const std::string& region,
    const std::vector<std::string>& allowed) {

    if (allowed.empty()) {
        return true;
    }

    return std::find(
        allowed.begin(),
        allowed.end(),
        region)
        != allowed.end();
}

std::string classify_binding(
    const GltfCharacter& character,
    const CharacterPointBinding& binding) {

    static const std::array<
        const char*,
        18> ordered_regions{{
        "hand_l",
        "hand_r",
        "lowerarm_l",
        "lowerarm_r",
        "upperarm_l",
        "upperarm_r",
        "foot_l",
        "foot_r",
        "calf_l",
        "calf_r",
        "thigh_l",
        "thigh_r",
        "Head",
        "neck_01",
        "pelvis",
        "spine_03",
        "spine_02",
        "spine_01"
    }};

    for (const char* region
         : ordered_regions) {

        if (character.binding_branch_weight(
                binding,
                region)
            >= 0.5) {
            return region;
        }
    }

    return "other";
}

bool anatomical_neighbors(
    const std::string& a,
    const std::string& b) {

    if (a == b) {
        return true;
    }

    const auto matches =
        [&](const char* x,
            const char* y) {
            return (a == x && b == y)
                || (a == y && b == x);
        };

    return
        matches("pelvis", "thigh_l")
        || matches("pelvis", "thigh_r")
        || matches("thigh_l", "calf_l")
        || matches("thigh_r", "calf_r")
        || matches("calf_l", "foot_l")
        || matches("calf_r", "foot_r")
        || matches("spine_03", "upperarm_l")
        || matches("spine_03", "upperarm_r")
        || matches("upperarm_l", "lowerarm_l")
        || matches("upperarm_r", "lowerarm_r")
        || matches("lowerarm_l", "hand_l")
        || matches("lowerarm_r", "hand_r")
        || matches("spine_01", "spine_02")
        || matches("spine_02", "spine_03")
        || matches("spine_03", "neck_01")
        || matches("neck_01", "Head")
        || matches("pelvis", "spine_01");
}

std::uint64_t voxel_key(
    int x,
    int y,
    int z) {

    constexpr std::uint64_t mask =
        (std::uint64_t{1} << 21) - 1;

    return
        (static_cast<std::uint64_t>(x)
            & mask)
            << 42
        | (static_cast<std::uint64_t>(y)
            & mask)
            << 21
        | (static_cast<std::uint64_t>(z)
            & mask);
}

} // namespace

void VoxelizedCharacter::build(
    const GltfCharacter& character,
    std::size_t animation,
    double binding_time_seconds,
    double voxel_size) {

    if (voxel_size <= 0.0) {
        throw std::invalid_argument(
            "character voxel size must be positive");
    }

    const CharacterMeshFrame rest_mesh =
        character.sample(
            animation,
            binding_time_seconds,
            true);

    if (rest_mesh.indices.size() % 3 != 0) {
        throw std::invalid_argument(
            "character voxelization requires triangle indices");
    }

    voxel_size_ = voxel_size;
    voxels_.clear();

    const Bounds bounds =
        bounds_of(rest_mesh);

    const Vec3 padding{
        voxel_size,
        voxel_size,
        voxel_size
    };

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

    struct PendingVoxel {
        Vec3 center{};
        int x{};
        int y{};
        int z{};
    };

    std::vector<PendingVoxel>
        pending;

    std::vector<Vec3>
        occupied_centers;

    for (std::size_t z = 0;
         z < nz;
         ++z) {

        for (std::size_t y = 0;
             y < ny;
             ++y) {

            for (std::size_t x = 0;
                 x < nx;
                 ++x) {

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

                pending.push_back({
                    center,
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(z)
                });

                occupied_centers.push_back(
                    center);
            }
        }
    }

    if (pending.empty()) {
        throw std::runtime_error(
            "Quaternius voxelization produced no voxels");
    }

    const auto bindings =
        character.bind_points_to_skin(
            animation,
            binding_time_seconds,
            occupied_centers,
            true);

    if (bindings.size()
        != pending.size()) {
        throw std::logic_error(
            "voxel binding count mismatch");
    }

    voxels_.reserve(
        pending.size());

    for (std::size_t i = 0;
         i < pending.size();
         ++i) {

        CharacterVoxel voxel;
        voxel.rest_center =
            pending[i].center;

        voxel.skin_binding =
            bindings[i];

        voxel.anatomical_region =
            classify_binding(
                character,
                voxel.skin_binding);

        voxel.grid_x =
            pending[i].x;
        voxel.grid_y =
            pending[i].y;
        voxel.grid_z =
            pending[i].z;

        voxels_.push_back(
            std::move(voxel));
    }
}

std::vector<Vec3>
VoxelizedCharacter::sample_centers(
    const GltfCharacter& character,
    std::size_t animation,
    double time_seconds,
    bool loop,
    const Vec3& world_offset) const {

    std::vector<CharacterPointBinding>
        bindings;

    bindings.reserve(
        voxels_.size());

    for (const CharacterVoxel& voxel
         : voxels_) {
        bindings.push_back(
            voxel.skin_binding);
    }

    return character.sample_bound_points(
        bindings,
        animation,
        time_seconds,
        loop,
        world_offset);
}

std::vector<Vec3>
VoxelizedCharacter::sample_centers_with_node_local_poses(
    const GltfCharacter& character,
    const std::vector<CharacterNodeLocalPose>& node_poses,
    const Vec3& world_offset) const {

    std::vector<CharacterPointBinding>
        bindings;

    bindings.reserve(
        voxels_.size());

    for (const CharacterVoxel& voxel
         : voxels_) {
        bindings.push_back(
            voxel.skin_binding);
    }

    return character
        .sample_bound_points_with_node_local_poses(
            bindings,
            node_poses,
            world_offset);
}

CharacterMeshFrame
VoxelizedCharacter::render(
    const std::vector<Vec3>& world_centers) const {

    if (world_centers.size()
        != voxels_.size()) {
        throw std::invalid_argument(
            "voxel center count mismatch");
    }

    CharacterMeshFrame out;

    const std::size_t attached =
        static_cast<std::size_t>(
            std::count_if(
                voxels_.begin(),
                voxels_.end(),
                [](const CharacterVoxel& voxel) {
                    return voxel.state
                        == CharacterVoxelState::Attached;
                }));

    out.positions.reserve(
        attached * 8);

    out.indices.reserve(
        attached * 36);

    const double half =
        voxel_size_ * 0.47;

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        if (voxels_[i].state
            != CharacterVoxelState::Attached) {
            continue;
        }

        append_cube(
            out,
            world_centers[i],
            half);
    }

    return out;
}

CharacterMeshFrame
VoxelizedCharacter::render_component(
    const DetachedVoxelComponent& component,
    const std::vector<Vec3>& world_centers) const {

    if (component.voxel_indices.size()
        != world_centers.size()) {
        throw std::invalid_argument(
            "detached voxel component center count mismatch");
    }

    CharacterMeshFrame out;

    out.positions.reserve(
        component.voxel_indices.size()
        * 8);

    out.indices.reserve(
        component.voxel_indices.size()
        * 36);

    const double half =
        voxel_size_ * 0.47;

    for (std::size_t i = 0;
         i < component.voxel_indices.size();
         ++i) {

        const std::size_t voxel_index =
            component.voxel_indices[i];

        if (voxel_index
            >= voxels_.size()) {
            throw std::out_of_range(
                "detached voxel index out of range");
        }

        append_cube(
            out,
            world_centers[i],
            half);
    }

    return out;
}

std::size_t
VoxelizedCharacter::damage_sphere(
    const std::vector<Vec3>& world_centers,
    const Vec3& center,
    double radius,
    double damage,
    const std::vector<std::string>& allowed_regions) {

    if (world_centers.size()
        != voxels_.size()) {
        throw std::invalid_argument(
            "voxel center count mismatch");
    }

    if (radius <= 0.0
        || damage < 0.0) {
        throw std::invalid_argument(
            "invalid character voxel damage sphere");
    }

    const double radius_squared =
        radius * radius;

    std::size_t destroyed = 0;

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        CharacterVoxel& voxel =
            voxels_[i];

        if (voxel.state
                != CharacterVoxelState::Attached
            || !region_allowed(
                voxel.anatomical_region,
                allowed_regions)) {
            continue;
        }

        if (length_squared(
                world_centers[i]
                - center)
            > radius_squared) {
            continue;
        }

        voxel.damage += damage;

        if (voxel.damage
            >= voxel.break_damage) {

            voxel.state =
                CharacterVoxelState::Destroyed;

            ++destroyed;
        }
    }

    return destroyed;
}

std::size_t
VoxelizedCharacter::damage_cut_disk(
    const std::vector<Vec3>& world_centers,
    const Vec3& center,
    const Vec3& normal,
    double half_thickness,
    double radius,
    double damage,
    const std::vector<std::string>& allowed_regions) {

    if (world_centers.size()
        != voxels_.size()) {
        throw std::invalid_argument(
            "voxel center count mismatch");
    }

    if (half_thickness <= 0.0
        || radius <= 0.0
        || damage < 0.0) {
        throw std::invalid_argument(
            "invalid character voxel cut disk");
    }

    const Vec3 axis =
        normalized(normal);

    if (length_squared(axis)
        <= 1e-12) {
        throw std::invalid_argument(
            "character voxel cut disk normal must be non-zero");
    }

    const double radius_squared =
        radius * radius;

    std::size_t destroyed = 0;

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        CharacterVoxel& voxel =
            voxels_[i];

        if (voxel.state
                != CharacterVoxelState::Attached
            || !region_allowed(
                voxel.anatomical_region,
                allowed_regions)) {
            continue;
        }

        const Vec3 relative =
            world_centers[i]
            - center;

        const double axial =
            dot(relative, axis);

        if (std::abs(axial)
            > half_thickness) {
            continue;
        }

        const Vec3 radial =
            relative
            - axis * axial;

        if (length_squared(radial)
            > radius_squared) {
            continue;
        }

        voxel.damage += damage;

        if (voxel.damage
            >= voxel.break_damage) {

            voxel.state =
                CharacterVoxelState::Destroyed;

            ++destroyed;
        }
    }

    return destroyed;
}

std::size_t
VoxelizedCharacter::damage_anatomical_interface(
    const std::string& distal_region,
    const std::vector<std::string>& proximal_regions,
    double damage) {

    if (distal_region.empty()
        || proximal_regions.empty()
        || damage < 0.0) {
        throw std::invalid_argument(
            "invalid anatomical interface damage request");
    }

    std::unordered_map<
        std::uint64_t,
        std::size_t> grid;

    grid.reserve(
        voxels_.size() * 2);

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        if (voxels_[i].state
            != CharacterVoxelState::Attached) {
            continue;
        }

        grid.emplace(
            voxel_key(
                voxels_[i].grid_x,
                voxels_[i].grid_y,
                voxels_[i].grid_z),
            i);
    }

    constexpr int directions[6][3] = {
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };

    std::vector<std::size_t>
        interface_voxels;

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        const CharacterVoxel& voxel =
            voxels_[i];

        if (voxel.state
                != CharacterVoxelState::Attached
            || voxel.anatomical_region
                != distal_region) {
            continue;
        }

        bool interface = false;

        for (const auto& direction
             : directions) {

            const int nx =
                voxel.grid_x + direction[0];
            const int ny =
                voxel.grid_y + direction[1];
            const int nz =
                voxel.grid_z + direction[2];

            if (nx < 0
                || ny < 0
                || nz < 0) {
                continue;
            }

            const auto found =
                grid.find(
                    voxel_key(
                        nx,
                        ny,
                        nz));

            if (found == grid.end()) {
                continue;
            }

            const CharacterVoxel& neighbor =
                voxels_[found->second];

            if (neighbor.state
                    != CharacterVoxelState::Attached
                || std::find(
                    proximal_regions.begin(),
                    proximal_regions.end(),
                    neighbor.anatomical_region)
                    == proximal_regions.end()) {
                continue;
            }

            interface = true;
            break;
        }

        if (interface) {
            interface_voxels.push_back(i);
        }
    }

    std::size_t destroyed = 0;

    for (const std::size_t index
         : interface_voxels) {

        CharacterVoxel& voxel =
            voxels_[index];

        voxel.damage += damage;

        if (voxel.damage
            >= voxel.break_damage) {
            voxel.state =
                CharacterVoxelState::Destroyed;
            ++destroyed;
        }
    }

    return destroyed;
}

std::optional<DetachedVoxelComponent>
VoxelizedCharacter::detach_anatomical_region_if_disconnected(
    const std::string& anatomical_region,
    const std::vector<std::string>& proximal_regions,
    std::size_t minimum_voxels) {

    if (anatomical_region.empty()
        || proximal_regions.empty()
        || minimum_voxels == 0) {
        throw std::invalid_argument(
            "invalid anatomical voxel detachment request");
    }

    std::unordered_map<
        std::uint64_t,
        std::size_t> grid;

    grid.reserve(
        voxels_.size() * 2);

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        if (voxels_[i].state
            != CharacterVoxelState::Attached) {
            continue;
        }

        grid.emplace(
            voxel_key(
                voxels_[i].grid_x,
                voxels_[i].grid_y,
                voxels_[i].grid_z),
            i);
    }

    std::vector<std::size_t>
        candidate;

    constexpr int directions[6][3] = {
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        const CharacterVoxel& voxel =
            voxels_[i];

        if (voxel.state
                != CharacterVoxelState::Attached
            || voxel.anatomical_region
                != anatomical_region) {
            continue;
        }

        candidate.push_back(i);

        for (const auto& direction
             : directions) {

            const int nx =
                voxel.grid_x
                + direction[0];

            const int ny =
                voxel.grid_y
                + direction[1];

            const int nz =
                voxel.grid_z
                + direction[2];

            if (nx < 0
                || ny < 0
                || nz < 0) {
                continue;
            }

            const auto found =
                grid.find(
                    voxel_key(
                        nx,
                        ny,
                        nz));

            if (found == grid.end()) {
                continue;
            }

            const CharacterVoxel& neighbor =
                voxels_[found->second];

            if (neighbor.anatomical_region
                    == anatomical_region) {
                continue;
            }

            if (std::find(
                    proximal_regions.begin(),
                    proximal_regions.end(),
                    neighbor.anatomical_region)
                != proximal_regions.end()) {
                return std::nullopt;
            }

            // Face contact with an anatomically unrelated region is
            // contact/proximity, not a tissue bridge. It must never keep
            // a severed component under animation authority.
        }
    }

    if (candidate.size()
        < minimum_voxels) {
        return std::nullopt;
    }

    DetachedVoxelComponent detached;
    detached.voxel_indices =
        candidate;

    detached.anatomical_region =
        anatomical_region;

    for (const std::size_t index
         : detached.voxel_indices) {

        voxels_[index].state =
            CharacterVoxelState::Detached;
    }

    return detached;
}

std::optional<DetachedVoxelComponent>
VoxelizedCharacter::detach_component_near_anatomical(
    const std::vector<Vec3>& world_centers,
    const Vec3& seed_world_point,
    std::size_t minimum_voxels) {

    if (world_centers.size()
            != voxels_.size()
        || minimum_voxels == 0) {
        throw std::invalid_argument(
            "invalid anatomical component detachment request");
    }

    std::unordered_map<
        std::uint64_t,
        std::size_t> grid;

    grid.reserve(
        voxels_.size() * 2);

    std::size_t seed_index =
        std::numeric_limits<std::size_t>::max();

    double seed_distance =
        std::numeric_limits<double>::infinity();

    for (std::size_t i = 0;
         i < voxels_.size();
         ++i) {

        if (voxels_[i].state
            != CharacterVoxelState::Attached) {
            continue;
        }

        grid.emplace(
            voxel_key(
                voxels_[i].grid_x,
                voxels_[i].grid_y,
                voxels_[i].grid_z),
            i);

        const double distance =
            length_squared(
                world_centers[i]
                - seed_world_point);

        if (distance < seed_distance) {
            seed_distance = distance;
            seed_index = i;
        }
    }

    if (seed_index
        == std::numeric_limits<std::size_t>::max()) {
        return std::nullopt;
    }

    constexpr int directions[6][3] = {
        { 1,  0,  0},
        {-1,  0,  0},
        { 0,  1,  0},
        { 0, -1,  0},
        { 0,  0,  1},
        { 0,  0, -1}
    };

    std::vector<int> component_of(
        voxels_.size(),
        -1);

    std::vector<std::vector<std::size_t>>
        components;

    for (std::size_t start = 0;
         start < voxels_.size();
         ++start) {

        if (voxels_[start].state
                != CharacterVoxelState::Attached
            || component_of[start] >= 0) {
            continue;
        }

        const int component_id =
            static_cast<int>(
                components.size());

        components.push_back({});

        std::vector<std::size_t> queue;
        queue.push_back(start);
        component_of[start] = component_id;

        std::size_t cursor = 0;

        while (cursor < queue.size()) {
            const std::size_t current =
                queue[cursor++];

            components.back().push_back(
                current);

            const CharacterVoxel& voxel =
                voxels_[current];

            for (const auto& direction
                 : directions) {

                const int nx =
                    voxel.grid_x + direction[0];
                const int ny =
                    voxel.grid_y + direction[1];
                const int nz =
                    voxel.grid_z + direction[2];

                if (nx < 0
                    || ny < 0
                    || nz < 0) {
                    continue;
                }

                const auto found =
                    grid.find(
                        voxel_key(
                            nx,
                            ny,
                            nz));

                if (found == grid.end()) {
                    continue;
                }

                const std::size_t neighbor =
                    found->second;

                if (component_of[neighbor] >= 0) {
                    continue;
                }

                if (!anatomical_neighbors(
                        voxel.anatomical_region,
                        voxels_[neighbor]
                            .anatomical_region)) {
                    continue;
                }

                component_of[neighbor] =
                    component_id;

                queue.push_back(
                    neighbor);
            }
        }
    }

    if (components.empty()) {
        return std::nullopt;
    }

    const int seed_component_id =
        component_of[seed_index];

    if (seed_component_id < 0) {
        return std::nullopt;
    }

    std::size_t largest_component = 0;

    for (std::size_t i = 1;
         i < components.size();
         ++i) {

        if (components[i].size()
            > components[largest_component].size()) {
            largest_component = i;
        }
    }

    const std::size_t seed_component =
        static_cast<std::size_t>(
            seed_component_id);

    if (seed_component == largest_component
        || components[seed_component].size()
            < minimum_voxels) {
        return std::nullopt;
    }

    DetachedVoxelComponent detached;
    detached.voxel_indices =
        components[seed_component];

    detached.anatomical_region =
        voxels_[seed_index]
            .anatomical_region;

    for (const std::size_t index
         : detached.voxel_indices) {

        voxels_[index].state =
            CharacterVoxelState::Detached;
    }

    return detached;
}

Vec3 VoxelizedCharacter::voxel_center(
    std::size_t voxel_index,
    const std::vector<Vec3>& world_centers) const {

    if (voxel_index
            >= voxels_.size()
        || world_centers.size()
            != voxels_.size()) {
        throw std::out_of_range(
            "character voxel index or center count invalid");
    }

    return world_centers[
        voxel_index];
}

VoxelizedCharacterStats
VoxelizedCharacter::stats() const {

    VoxelizedCharacterStats out;
    out.total_voxels =
        voxels_.size();

    for (const CharacterVoxel& voxel
         : voxels_) {

        switch (voxel.state) {
        case CharacterVoxelState::Attached:
            ++out.attached_voxels;
            ++out.active_voxels;
            break;

        case CharacterVoxelState::Detached:
            ++out.detached_voxels;
            ++out.active_voxels;
            break;

        case CharacterVoxelState::Destroyed:
            ++out.destroyed_voxels;
            break;
        }
    }

    out.voxel_size =
        voxel_size_;

    return out;
}

std::vector<AnatomicalAvailability>
VoxelizedCharacter::anatomy_availability() const {

    std::unordered_map<
        std::string,
        AnatomicalAvailability> by_region;

    for (const CharacterVoxel& voxel
         : voxels_) {

        auto& availability =
            by_region[
                voxel.anatomical_region];

        availability.region =
            voxel.anatomical_region;

        ++availability.total_voxels;

        if (voxel.state
            == CharacterVoxelState::Attached) {
            ++availability.attached_voxels;
        }
    }

    std::vector<AnatomicalAvailability>
        result;

    result.reserve(
        by_region.size());

    for (auto& [name, availability]
         : by_region) {
        (void)name;
        result.push_back(
            std::move(availability));
    }

    std::sort(
        result.begin(),
        result.end(),
        [](const auto& a, const auto& b) {
            return a.region < b.region;
        });

    return result;
}

double VoxelizedCharacter::attached_fraction(
    const std::string& anatomical_region) const {

    std::size_t total = 0;
    std::size_t attached = 0;

    for (const CharacterVoxel& voxel
         : voxels_) {

        if (voxel.anatomical_region
            != anatomical_region) {
            continue;
        }

        ++total;

        if (voxel.state
            == CharacterVoxelState::Attached) {
            ++attached;
        }
    }

    if (total == 0) {
        return 1.0;
    }

    return
        static_cast<double>(attached)
        / static_cast<double>(total);
}

} // namespace sarx
