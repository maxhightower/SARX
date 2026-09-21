#include "sarx/humanoid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace sarx {
namespace {

struct CapsuleShape {
    Vec3 a{};
    Vec3 b{};
    double radius{};
};

struct SphereShape {
    Vec3 center{};
    double radius{};
};

struct BoneAnchor {
    BoneId bone{kNoParent};
    Vec3 position{};
    double influence_radius{};
};

double point_segment_distance_squared(
    const Vec3& p,
    const Vec3& a,
    const Vec3& b) {

    const Vec3 ab = b - a;
    const double denom = length_squared(ab);
    const double t = denom > 1e-12
        ? std::clamp(dot(p - a, ab) / denom, 0.0, 1.0)
        : 0.0;

    return length_squared(p - (a + ab * t));
}

bool inside(
    const Vec3& p,
    const CapsuleShape& capsule) {

    return point_segment_distance_squared(
        p,
        capsule.a,
        capsule.b)
        <= capsule.radius * capsule.radius;
}

bool inside(
    const Vec3& p,
    const SphereShape& sphere) {

    return length_squared(p - sphere.center)
        <= sphere.radius * sphere.radius;
}

bool occupied(const Vec3& p) {
    static const std::array<CapsuleShape, 13> capsules{{
        // Pelvis and torso.
        {{-0.18, 0.78, 0.0}, {0.18, 0.78, 0.0}, 0.22},
        {{0.0, 0.86, 0.0}, {0.0, 1.32, 0.0}, 0.27},
        {{-0.28, 1.31, 0.0}, {0.28, 1.31, 0.0}, 0.21},
        {{0.0, 1.42, 0.0}, {0.0, 1.55, 0.0}, 0.10},

        // Left arm.
        {{-0.28, 1.34, 0.0}, {-0.62, 1.18, 0.0}, 0.14},
        {{-0.62, 1.18, 0.0}, {-0.82, 0.90, 0.0}, 0.115},

        // Right arm.
        {{0.28, 1.34, 0.0}, {0.62, 1.18, 0.0}, 0.14},
        {{0.62, 1.18, 0.0}, {0.82, 0.90, 0.0}, 0.115},

        // Left leg.
        {{-0.15, 0.72, 0.0}, {-0.16, 0.43, 0.0}, 0.145},
        {{-0.16, 0.43, 0.0}, {-0.16, 0.13, 0.0}, 0.11},

        // Right leg.
        {{0.15, 0.72, 0.0}, {0.16, 0.43, 0.0}, 0.145},
        {{0.16, 0.43, 0.0}, {0.16, 0.13, 0.0}, 0.11},

        // Feet share a forward depth direction.
        {{-0.16, 0.09, 0.0}, {-0.16, 0.09, 0.16}, 0.10}
    }};

    static const CapsuleShape right_foot{
        {0.16, 0.09, 0.0},
        {0.16, 0.09, 0.16},
        0.10
    };

    static const std::array<SphereShape, 3> spheres{{
        {{0.0, 1.72, 0.0}, 0.20},
        {{-0.89, 0.82, 0.0}, 0.12},
        {{0.89, 0.82, 0.0}, 0.12}
    }};

    for (const auto& capsule : capsules) {
        if (inside(p, capsule)) return true;
    }

    if (inside(p, right_foot)) return true;

    for (const auto& sphere : spheres) {
        if (inside(p, sphere)) return true;
    }

    return false;
}

std::size_t flat_index(
    std::size_t x,
    std::size_t y,
    std::size_t z,
    std::size_t nx,
    std::size_t ny) {

    return x + nx * (y + ny * z);
}

} // namespace

HumanoidFixture build_humanoid_fixture(
    const HumanoidSpec& spec) {

    if (spec.spacing <= 0.0
        || spec.particle_mass <= 0.0
        || spec.structural_compliance < 0.0
        || spec.structural_break_damage <= 0.0
        || spec.volume_compliance < 0.0
        || spec.volume_break_damage <= 0.0
        || spec.attachment_compliance < 0.0
        || spec.attachment_break_damage <= 0.0
        || spec.joint_break_damage <= 0.0
        || spec.joint_radius < 0.0) {
        throw std::invalid_argument("invalid humanoid fixture spec");
    }

    HumanoidFixture fixture;

    const Vec3 origin{-1.0, 0.0, -0.30};
    const std::size_t nx =
        static_cast<std::size_t>(std::ceil(2.0 / spec.spacing)) + 1;
    const std::size_t ny =
        static_cast<std::size_t>(std::ceil(1.95 / spec.spacing)) + 1;
    const std::size_t nz =
        static_cast<std::size_t>(std::ceil(0.60 / spec.spacing)) + 1;

    const ParticleId invalid =
        std::numeric_limits<ParticleId>::max();

    std::vector<ParticleId> grid(nx * ny * nz, invalid);

    for (std::size_t z = 0; z < nz; ++z) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = 0; x < nx; ++x) {
                const Vec3 position =
                    origin + Vec3{
                        static_cast<double>(x) * spec.spacing,
                        static_cast<double>(y) * spec.spacing,
                        static_cast<double>(z) * spec.spacing
                    };

                if (!occupied(position)) continue;

                const ParticleId id =
                    fixture.body.add_particle(
                        position,
                        spec.particle_mass);

                grid[flat_index(x, y, z, nx, ny)] = id;
            }
        }
    }

    const auto grid_particle =
        [&](int x, int y, int z) -> ParticleId {

        if (x < 0 || y < 0 || z < 0
            || x >= static_cast<int>(nx)
            || y >= static_cast<int>(ny)
            || z >= static_cast<int>(nz)) {
            return invalid;
        }

        return grid[flat_index(
            static_cast<std::size_t>(x),
            static_cast<std::size_t>(y),
            static_cast<std::size_t>(z),
            nx,
            ny)];
    };

    std::vector<std::array<int, 3>> offsets;

    for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;

                if (dz < 0
                    || (dz == 0 && dy < 0)
                    || (dz == 0 && dy == 0 && dx <= 0)) {
                    continue;
                }

                offsets.push_back({dx, dy, dz});
            }
        }
    }

    for (std::size_t z = 0; z < nz; ++z) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = 0; x < nx; ++x) {
                const ParticleId a = grid_particle(
                    static_cast<int>(x),
                    static_cast<int>(y),
                    static_cast<int>(z));

                if (a == invalid) continue;

                for (const auto& offset : offsets) {
                    const ParticleId b = grid_particle(
                        static_cast<int>(x) + offset[0],
                        static_cast<int>(y) + offset[1],
                        static_cast<int>(z) + offset[2]);

                    if (b == invalid) continue;

                    fixture.body.add_structural_constraint(
                        a,
                        b,
                        spec.structural_compliance,
                        spec.structural_break_damage,
                        spec.tissue_material);
                }
            }
        }
    }

    for (std::size_t z = 0; z + 1 < nz; ++z) {
        for (std::size_t y = 0; y + 1 < ny; ++y) {
            for (std::size_t x = 0; x + 1 < nx; ++x) {
                const ParticleId v000 =
                    grid_particle(x, y, z);
                const ParticleId v100 =
                    grid_particle(x + 1, y, z);
                const ParticleId v010 =
                    grid_particle(x, y + 1, z);
                const ParticleId v110 =
                    grid_particle(x + 1, y + 1, z);
                const ParticleId v001 =
                    grid_particle(x, y, z + 1);
                const ParticleId v101 =
                    grid_particle(x + 1, y, z + 1);
                const ParticleId v011 =
                    grid_particle(x, y + 1, z + 1);
                const ParticleId v111 =
                    grid_particle(x + 1, y + 1, z + 1);

                const std::array<ParticleId, 8> corners{
                    v000, v100, v010, v110,
                    v001, v101, v011, v111
                };

                if (std::find(
                        corners.begin(),
                        corners.end(),
                        invalid) != corners.end()) {
                    continue;
                }

                const std::array<std::array<ParticleId, 4>, 6> tets{{
                    {v000, v100, v110, v111},
                    {v000, v110, v010, v111},
                    {v000, v010, v011, v111},
                    {v000, v011, v001, v111},
                    {v000, v001, v101, v111},
                    {v000, v101, v100, v111}
                }};

                for (const auto& tet : tets) {
                    fixture.body.add_tetrahedral_constraint(
                        tet[0],
                        tet[1],
                        tet[2],
                        tet[3],
                        spec.volume_compliance,
                        spec.volume_break_damage,
                        spec.tissue_material);
                }
            }
        }
    }

    auto& bones = fixture.bones;

    bones.pelvis = fixture.body.add_bone(
        kNoParent,
        {0.0, 0.80, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.spine = fixture.body.add_bone(
        bones.pelvis,
        {0.0, 1.04, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.chest = fixture.body.add_bone(
        bones.spine,
        {0.0, 1.30, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.neck = fixture.body.add_bone(
        bones.chest,
        {0.0, 1.50, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.head = fixture.body.add_bone(
        bones.neck,
        {0.0, 1.72, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.left_shoulder = fixture.body.add_bone(
        bones.chest,
        {-0.46, 1.34, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.left_elbow = fixture.body.add_bone(
        bones.left_shoulder,
        {-0.70, 1.10, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.left_hand = fixture.body.add_bone(
        bones.left_elbow,
        {-0.88, 0.86, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.right_shoulder = fixture.body.add_bone(
        bones.chest,
        fixture.right_shoulder_rest,
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.right_elbow = fixture.body.add_bone(
        bones.right_shoulder,
        fixture.right_elbow_rest,
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.right_hand = fixture.body.add_bone(
        bones.right_elbow,
        fixture.right_hand_rest,
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.left_hip = fixture.body.add_bone(
        bones.pelvis,
        {-0.15, 0.70, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.left_knee = fixture.body.add_bone(
        bones.left_hip,
        {-0.16, 0.42, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.left_ankle = fixture.body.add_bone(
        bones.left_knee,
        {-0.16, 0.12, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.right_hip = fixture.body.add_bone(
        bones.pelvis,
        {0.15, 0.70, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.right_knee = fixture.body.add_bone(
        bones.right_hip,
        {0.16, 0.42, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    bones.right_ankle = fixture.body.add_bone(
        bones.right_knee,
        {0.16, 0.12, 0.0},
        spec.joint_break_damage,
        spec.tissue_material,
        spec.joint_radius);

    const std::array<BoneAnchor, 17> anchors{{
        {bones.pelvis, {0.0, 0.80, 0.0}, 0.34},
        {bones.spine, {0.0, 1.04, 0.0}, 0.30},
        {bones.chest, {0.0, 1.30, 0.0}, 0.34},
        {bones.neck, {0.0, 1.50, 0.0}, 0.16},
        {bones.head, {0.0, 1.72, 0.0}, 0.24},

        {bones.left_shoulder, {-0.46, 1.34, 0.0}, 0.25},
        {bones.left_elbow, {-0.70, 1.10, 0.0}, 0.22},
        {bones.left_hand, {-0.88, 0.86, 0.0}, 0.16},

        {bones.right_shoulder, fixture.right_shoulder_rest, 0.25},
        {bones.right_elbow, fixture.right_elbow_rest, 0.22},
        {bones.right_hand, fixture.right_hand_rest, 0.16},

        {bones.left_hip, {-0.15, 0.70, 0.0}, 0.23},
        {bones.left_knee, {-0.16, 0.42, 0.0}, 0.19},
        {bones.left_ankle, {-0.16, 0.12, 0.0}, 0.16},

        {bones.right_hip, {0.15, 0.70, 0.0}, 0.23},
        {bones.right_knee, {0.16, 0.42, 0.0}, 0.19},
        {bones.right_ankle, {0.16, 0.12, 0.0}, 0.16}
    }};

    for (ParticleId id = 0;
         id < fixture.body.particles().size();
         ++id) {

        const Vec3 position =
            fixture.body.particles()[id].position;

        const BoneAnchor* best = nullptr;
        double best_score =
            std::numeric_limits<double>::infinity();

        for (const auto& anchor : anchors) {
            const double distance =
                length(position - anchor.position);
            const double score =
                distance / anchor.influence_radius;

            if (score <= 1.0 && score < best_score) {
                best = &anchor;
                best_score = score;
            }
        }

        if (!best) continue;

        fixture.body.add_attachment(
            id,
            best->bone,
            position - best->position,
            spec.attachment_compliance,
            spec.attachment_break_damage,
            spec.tissue_material);
    }

    if (fixture.body.particles().empty()) {
        throw std::logic_error(
            "humanoid fixture generated no particles");
    }

    return fixture;
}

} // namespace sarx
