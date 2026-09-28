#include "sarx/anatomical_humanoid.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace sarx {
namespace {

enum class PrimitiveKind { Ellipsoid, TaperedCapsule, Box };

struct Primitive {
    PrimitiveKind kind{PrimitiveKind::Ellipsoid};
    Vec3 a{};       // ellipsoid centre / capsule start / box min
    Vec3 b{};       // ellipsoid radii / capsule end / box max
    double ra{0.0};  // capsule start radius
    double rb{0.0};  // capsule end radius
    bool physical{true};  // part of the simulated lattice (else detail only)
};

Primitive ellipsoid(Vec3 c, Vec3 r, bool physical = true) {
    return {PrimitiveKind::Ellipsoid, c, r, 0.0, 0.0, physical};
}

Primitive capsule(Vec3 a, Vec3 b, double ra, double rb, bool physical = true) {
    return {PrimitiveKind::TaperedCapsule, a, b, ra, rb, physical};
}

Primitive box(Vec3 lo, Vec3 hi, bool physical = true) {
    return {PrimitiveKind::Box, lo, hi, 0.0, 0.0, physical};
}

Vec3 mirror(Vec3 v) { return {-v.x, v.y, v.z}; }

// Rest-pose bone positions (right side; left is mirrored).
constexpr double kPelvisY = 0.95;
const Vec3 kShoulder{0.20, 1.44, 0.0};
const Vec3 kElbow{0.23, 1.40, 0.28};
const Vec3 kHand{0.22, 1.36, 0.52};
const Vec3 kHip{0.10, 0.90, 0.0};
const Vec3 kKnee{0.11, 0.50, 0.01};
const Vec3 kAnkle{0.11, 0.09, 0.0};

const std::vector<Primitive>& shape() {
    static const std::vector<Primitive> primitives = [] {
        std::vector<Primitive> p;
        p.reserve(48);
        const auto& lm = anatomical_landmarks();

        // Head, jaw, neck.
        p.push_back(ellipsoid(lm.head_center, {0.085, 0.11, 0.10}));
        p.push_back(ellipsoid({0.0, 1.60, 0.055}, {0.065, 0.045, 0.07}));
        p.push_back(capsule({0.0, 1.49, 0.0}, {0.0, 1.62, 0.02}, 0.055, 0.052));
        // Face details.
        p.push_back(box({-0.014, 1.675, 0.10}, {0.014, 1.725, 0.145}, false));  // nose
        p.push_back(ellipsoid({0.088, 1.69, 0.02}, {0.016, 0.03, 0.02}, false));
        p.push_back(ellipsoid({-0.088, 1.69, 0.02}, {0.016, 0.03, 0.02}, false));

        // Torso.
        p.push_back(ellipsoid({0.0, 1.34, 0.0}, {0.18, 0.15, 0.11}));
        p.push_back(ellipsoid({0.0, 1.13, 0.0}, {0.15, 0.14, 0.10}));
        p.push_back(ellipsoid({0.0, 0.93, 0.0}, {0.17, 0.10, 0.11}));

        for (int side = 0; side < 2; ++side) {
            auto m = [&](Vec3 v) { return side == 0 ? v : mirror(v); };

            // Shoulder cap, upper arm, forearm, palm.
            p.push_back(capsule(m({0.10, 1.44, 0.0}), m(kShoulder), 0.06, 0.058));
            p.push_back(capsule(m(kShoulder), m(kElbow), 0.055, 0.045));
            p.push_back(capsule(m(kElbow), m({0.22, 1.365, 0.50}), 0.045, 0.034));
            p.push_back(ellipsoid(m({0.22, 1.355, 0.56}), {0.045, 0.02, 0.05}));
            // Fingers and thumb (detail only).
            for (int f = 0; f < 4; ++f) {
                const double x = 0.195 + 0.017 * f;
                p.push_back(capsule(m({x, 1.352, 0.595}), m({x + 0.004, 1.335, 0.675}),
                                    0.0095, 0.008, false));
            }
            p.push_back(capsule(m({0.18, 1.355, 0.545}), m({0.165, 1.35, 0.605}),
                                0.011, 0.009, false));

            // Thigh, calf, foot.
            p.push_back(capsule(m({0.10, 0.92, 0.0}), m(kKnee), 0.082, 0.058));
            p.push_back(capsule(m(kKnee), m({0.11, 0.10, 0.0}), 0.056, 0.04));
            p.push_back(capsule(m({0.11, 0.045, -0.035}), m({0.11, 0.045, 0.13}), 0.045, 0.04));
            p.push_back(box(m({0.065, 0.0, -0.06}), m({0.155, 0.075, 0.175}), false));
        }
        // Mirrored boxes need ordered corners.
        for (auto& prim : p) {
            if (prim.kind != PrimitiveKind::Box) continue;
            if (prim.a.x > prim.b.x) std::swap(prim.a.x, prim.b.x);
        }
        return p;
    }();
    return primitives;
}

bool inside(const Primitive& prim, const Vec3& q, double pad) {
    switch (prim.kind) {
    case PrimitiveKind::Ellipsoid: {
        const Vec3 d = q - prim.a;
        const double x = d.x / (prim.b.x + pad);
        const double y = d.y / (prim.b.y + pad);
        const double z = d.z / (prim.b.z + pad);
        return x * x + y * y + z * z <= 1.0;
    }
    case PrimitiveKind::TaperedCapsule: {
        const Vec3 ab = prim.b - prim.a;
        const double denom = length_squared(ab);
        const double t = denom > 1e-12 ? std::clamp(dot(q - prim.a, ab) / denom, 0.0, 1.0) : 0.0;
        const double r = prim.ra + (prim.rb - prim.ra) * t + pad;
        return length_squared(q - (prim.a + ab * t)) <= r * r;
    }
    case PrimitiveKind::Box:
        return q.x >= prim.a.x - pad && q.x <= prim.b.x + pad
            && q.y >= prim.a.y - pad && q.y <= prim.b.y + pad
            && q.z >= prim.a.z - pad && q.z <= prim.b.z + pad;
    }
    return false;
}

bool occupied(const Vec3& q, double pad, bool physical_only) {
    for (const auto& prim : shape()) {
        if (physical_only && !prim.physical) continue;
        if (inside(prim, q, pad)) return true;
    }
    return false;
}

double segment_distance(const Vec3& q, const Vec3& a, const Vec3& b) {
    const Vec3 ab = b - a;
    const double denom = length_squared(ab);
    const double t = denom > 1e-12 ? std::clamp(dot(q - a, ab) / denom, 0.0, 1.0) : 0.0;
    return length(q - (a + ab * t));
}

double ellipsoid_norm(const Vec3& q, const Vec3& c, const Vec3& r) {
    const Vec3 d = q - c;
    return std::sqrt((d.x / r.x) * (d.x / r.x) + (d.y / r.y) * (d.y / r.y) + (d.z / r.z) * (d.z / r.z));
}

DetailTissue classify_tissue(const Vec3& q) {
    const auto& lm = anatomical_landmarks();

    const double head = ellipsoid_norm(q, lm.head_center, {0.085, 0.11, 0.10});
    if (head <= 1.0) {
        if (head < 0.72 && q.y > 1.63) return DetailTissue::Neural;
        if (head >= 0.72 && head <= 0.9 && q.y > 1.60) return DetailTissue::Bone;  // skull
    }
    const double jaw = ellipsoid_norm(q, {0.0, 1.60, 0.055}, {0.065, 0.045, 0.07});
    if (jaw <= 1.0 && jaw >= 0.62 && jaw <= 0.88 && q.y < 1.62) return DetailTissue::Bone;

    struct Bone { Vec3 a; Vec3 b; double r; };
    static const std::vector<Bone> skeleton = [] {
        std::vector<Bone> s{
            {{0.0, 0.88, -0.035}, {0.0, 1.56, -0.03}, 0.021},  // spine
            {{-0.11, 0.93, -0.01}, {0.11, 0.93, -0.01}, 0.024},  // pelvic girdle
        };
        for (int side = 0; side < 2; ++side) {
            auto m = [&](Vec3 v) { return side == 0 ? v : mirror(v); };
            s.push_back({m({0.02, 1.47, 0.03}), m({0.19, 1.45, 0.0}), 0.012});  // clavicle
            s.push_back({m(kShoulder), m(kElbow), 0.016});                       // humerus
            s.push_back({m(kElbow), m({0.22, 1.365, 0.50}), 0.013});           // forearm
            s.push_back({m(kHip), m(kKnee), 0.021});                             // femur
            s.push_back({m(kKnee), m(kAnkle), 0.017});                           // tibia
        }
        return s;
    }();
    for (const auto& bone : skeleton) {
        if (segment_distance(q, bone.a, bone.b) <= bone.r) return DetailTissue::Bone;
    }

    // Ribs: banded shell inside the chest.
    const double chest = ellipsoid_norm(q, {0.0, 1.34, 0.0}, {0.18, 0.15, 0.11});
    if (chest >= 0.78 && chest <= 0.9 && q.y > 1.22 && q.y < 1.47) {
        const double band = std::fmod((q.y - 1.22) / 0.045, 1.0);
        if (band < 0.45) return DetailTissue::Bone;
    }
    return DetailTissue::Soft;
}

std::size_t flat(std::size_t x, std::size_t y, std::size_t z, std::size_t nx, std::size_t ny) {
    return x + nx * (y + ny * z);
}

} // namespace

const AnatomicalLandmarks& anatomical_landmarks() {
    static const AnatomicalLandmarks landmarks{};
    return landmarks;
}

HumanoidFixture build_anatomical_humanoid_fixture(const AnatomicalHumanoidSpec& spec) {
    if (spec.spacing <= 0.0 || spec.density <= 0.0 || spec.lattice_padding < 0.0
        || spec.structural_compliance < 0.0 || spec.structural_break_damage <= 0.0
        || spec.volume_compliance < 0.0 || spec.volume_break_damage <= 0.0
        || spec.attachment_compliance < 0.0 || spec.attachment_break_damage <= 0.0
        || spec.joint_break_damage <= 0.0 || spec.joint_radius < 0.0) {
        throw std::invalid_argument("invalid anatomical humanoid spec");
    }

    HumanoidFixture fixture;
    Body& body = fixture.body;

    const Vec3 origin{-0.50, 0.0, -0.25};
    const Vec3 extent{1.0, 1.85, 1.0};
    const std::size_t nx = static_cast<std::size_t>(std::ceil(extent.x / spec.spacing)) + 1;
    const std::size_t ny = static_cast<std::size_t>(std::ceil(extent.y / spec.spacing)) + 1;
    const std::size_t nz = static_cast<std::size_t>(std::ceil(extent.z / spec.spacing)) + 1;
    const ParticleId invalid = std::numeric_limits<ParticleId>::max();
    std::vector<ParticleId> grid(nx * ny * nz, invalid);

    const double mass = spec.density * spec.spacing * spec.spacing * spec.spacing;
    for (std::size_t z = 0; z < nz; ++z) {
        for (std::size_t y = 0; y < ny; ++y) {
            for (std::size_t x = 0; x < nx; ++x) {
                const Vec3 q = origin + Vec3{x * spec.spacing, y * spec.spacing, z * spec.spacing};
                if (!occupied(q, spec.lattice_padding, true)) continue;
                grid[flat(x, y, z, nx, ny)] = body.add_particle(q, mass);
            }
        }
    }
    if (body.particles().empty()) throw std::logic_error("anatomical humanoid has no particles");

    auto at = [&](long x, long y, long z) -> ParticleId {
        if (x < 0 || y < 0 || z < 0 || x >= static_cast<long>(nx) || y >= static_cast<long>(ny)
            || z >= static_cast<long>(nz)) {
            return invalid;
        }
        return grid[flat(x, y, z, nx, ny)];
    };

    // 26-neighbourhood structural links (each pair once).
    for (long z = 0; z < static_cast<long>(nz); ++z) {
        for (long y = 0; y < static_cast<long>(ny); ++y) {
            for (long x = 0; x < static_cast<long>(nx); ++x) {
                const ParticleId a = at(x, y, z);
                if (a == invalid) continue;
                for (long dz = 0; dz <= 1; ++dz) {
                    for (long dy = -1; dy <= 1; ++dy) {
                        for (long dx = -1; dx <= 1; ++dx) {
                            if (dz == 0 && (dy < 0 || (dy == 0 && dx <= 0))) continue;
                            const ParticleId b = at(x + dx, y + dy, z + dz);
                            if (b == invalid) continue;
                            body.add_structural_constraint(
                                a, b, spec.structural_compliance, spec.structural_break_damage,
                                spec.tissue_material);
                        }
                    }
                }
            }
        }
    }

    // Six tetrahedra per fully occupied cell.
    for (long z = 0; z + 1 < static_cast<long>(nz); ++z) {
        for (long y = 0; y + 1 < static_cast<long>(ny); ++y) {
            for (long x = 0; x + 1 < static_cast<long>(nx); ++x) {
                const std::array<ParticleId, 8> c{
                    at(x, y, z), at(x + 1, y, z), at(x, y + 1, z), at(x + 1, y + 1, z),
                    at(x, y, z + 1), at(x + 1, y, z + 1), at(x, y + 1, z + 1), at(x + 1, y + 1, z + 1)};
                if (std::find(c.begin(), c.end(), invalid) != c.end()) continue;
                const std::array<std::array<ParticleId, 4>, 6> tets{{
                    {c[0], c[1], c[3], c[7]}, {c[0], c[3], c[2], c[7]}, {c[0], c[2], c[6], c[7]},
                    {c[0], c[6], c[4], c[7]}, {c[0], c[4], c[5], c[7]}, {c[0], c[5], c[1], c[7]}}};
                for (const auto& t : tets) {
                    body.add_tetrahedral_constraint(t[0], t[1], t[2], t[3], spec.volume_compliance,
                                                    spec.volume_break_damage, spec.tissue_material);
                }
            }
        }
    }

    // Bones: same topology as the reference fixture.
    auto& b = fixture.bones;
    auto bone = [&](BoneId parent, Vec3 p) {
        return body.add_bone(parent, p, spec.joint_break_damage, spec.tissue_material, spec.joint_radius);
    };
    b.pelvis = bone(kNoParent, {0.0, kPelvisY, 0.0});
    b.spine = bone(b.pelvis, {0.0, 1.15, 0.0});
    b.chest = bone(b.spine, {0.0, 1.35, 0.0});
    b.neck = bone(b.chest, {0.0, 1.55, 0.01});
    b.head = bone(b.neck, {0.0, 1.68, 0.03});
    b.left_shoulder = bone(b.chest, mirror(kShoulder));
    b.left_elbow = bone(b.left_shoulder, mirror(kElbow));
    b.left_hand = bone(b.left_elbow, mirror(kHand));
    b.right_shoulder = bone(b.chest, kShoulder);
    b.right_elbow = bone(b.right_shoulder, kElbow);
    b.right_hand = bone(b.right_elbow, kHand);
    b.left_hip = bone(b.pelvis, mirror(kHip));
    b.left_knee = bone(b.left_hip, mirror(kKnee));
    b.left_ankle = bone(b.left_knee, mirror(kAnkle));
    b.right_hip = bone(b.pelvis, kHip);
    b.right_knee = bone(b.right_hip, kKnee);
    b.right_ankle = bone(b.right_knee, kAnkle);

    // Each particle is attached to the bone that terminates its nearest
    // skeletal segment (parent -> bone), so everything distal to a cut on a
    // segment belongs to bones whose root path crosses that segment's joint.
    // Terminal pseudo-segments cover the pelvis, crown, hands and feet.
    const auto& bones = body.bones();
    struct Segment { BoneId bone; Vec3 a; Vec3 b; };
    std::vector<Segment> segments;
    for (BoneId id = 0; id < bones.size(); ++id) {
        if (bones[id].parent == kNoParent) continue;
        segments.push_back({id, bones[bones[id].parent].animated_position, bones[id].animated_position});
    }
    const Vec3 pelvis = bones[b.pelvis].animated_position;
    segments.push_back({b.pelvis, pelvis + Vec3{0.0, -0.06, 0.0}, pelvis});
    const Vec3 head = bones[b.head].animated_position;
    segments.push_back({b.head, head, head + Vec3{0.0, 0.14, 0.0}});
    for (const BoneId hand : {b.left_hand, b.right_hand}) {
        const Vec3 h = bones[hand].animated_position;
        segments.push_back({hand, h, h + Vec3{0.0, -0.02, 0.14}});
    }
    for (const BoneId ankle : {b.left_ankle, b.right_ankle}) {
        const Vec3 f = bones[ankle].animated_position;
        segments.push_back({ankle, f, f + Vec3{0.0, -0.05, 0.16}});
    }

    auto adjacent = [&](BoneId a, BoneId c) {
        return a != c && (bones[a].parent == c || bones[c].parent == a
                          || (bones[a].parent != kNoParent && bones[a].parent == bones[c].parent));
    };

    for (ParticleId id = 0; id < body.particles().size(); ++id) {
        const Vec3 q = body.particles()[id].position;
        const Segment* best = &segments.front();
        const Segment* second = nullptr;
        double best_distance = std::numeric_limits<double>::infinity();
        double second_distance = std::numeric_limits<double>::infinity();
        for (const auto& segment : segments) {
            const double d = segment_distance(q, segment.a, segment.b);
            if (d < best_distance) {
                if (best->bone != segment.bone) {
                    second = best;
                    second_distance = best_distance;
                }
                best_distance = d;
                best = &segment;
            } else if (d < second_distance && segment.bone != best->bone) {
                second_distance = d;
                second = &segment;
            }
        }

        // Primary attachment first: CharacterRuntime treats a particle's
        // first attachment as its home bone (anatomical region).
        double primary_weight = 1.0;
        const bool blend = spec.joint_blend_width > 0.0 && second
            && adjacent(best->bone, second->bone)
            && second_distance - best_distance < spec.joint_blend_width;
        if (blend) {
            primary_weight = 0.5 + 0.5 * (second_distance - best_distance) / spec.joint_blend_width;
        }
        const Vec3 anchor = bones[best->bone].animated_position;
        body.add_attachment(id, best->bone, q - anchor, spec.attachment_compliance / primary_weight,
                            spec.attachment_break_damage, spec.tissue_material);
        if (blend) {
            const Vec3 other = bones[second->bone].animated_position;
            body.add_attachment(id, second->bone, q - other,
                                spec.attachment_compliance / (1.0 - primary_weight),
                                spec.attachment_break_damage, spec.tissue_material);
        }
    }

    const auto& lm = anatomical_landmarks();
    fixture.right_shoulder_cut_center = lm.right_shoulder_cut_center;
    fixture.right_shoulder_cut_normal = normalized(lm.right_shoulder_cut_normal);
    fixture.right_shoulder_cut_radius = lm.shoulder_cut_radius;
    fixture.right_shoulder_rest = kShoulder;
    fixture.right_elbow_rest = kElbow;
    fixture.right_hand_rest = kHand;
    return fixture;
}

DetailVoxelSet build_anatomical_detail_voxels(double voxel_size) {
    if (voxel_size <= 0.0) throw std::invalid_argument("voxel size must be positive");

    static std::unordered_map<long long, DetailVoxelSet> cache;
    const long long key = std::llround(voxel_size * 1e6);
    if (const auto it = cache.find(key); it != cache.end()) return it->second;

    DetailVoxelSet set;
    set.voxel_size = voxel_size;
    const Vec3 lo{-0.40, 0.0, -0.20};
    const int nx = static_cast<int>(std::ceil(0.80 / voxel_size));
    const int ny = static_cast<int>(std::ceil(1.85 / voxel_size));
    const int nz = static_cast<int>(std::ceil(0.95 / voxel_size));
    for (int z = 0; z < nz; ++z) {
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                const Vec3 q = lo + Vec3{(x + 0.5) * voxel_size, (y + 0.5) * voxel_size, (z + 0.5) * voxel_size};
                if (!occupied(q, 0.0, false)) continue;
                set.voxels.push_back({q, x, y, z, classify_tissue(q)});
            }
        }
    }
    cache.emplace(key, set);
    return set;
}

HumanoidRuntime build_anatomical_humanoid_runtime(
    const AnatomicalHumanoidSpec& spec,
    const Vec3& world_offset,
    const RuntimeConfig& config) {

    HumanoidFixture fixture = build_anatomical_humanoid_fixture(spec);
    for (auto& p : fixture.body.particles()) p.position += world_offset;

    HumanoidRuntime result;
    result.world_offset = world_offset;
    for (BoneId b = 0; b < fixture.body.bones().size(); ++b) {
        const Vec3 target = fixture.body.bones()[b].animated_position + world_offset;
        fixture.body.set_bone_target(b, target);
        result.rest_bone_positions.push_back(target);
    }
    result.bones = fixture.bones;
    result.runtime = CharacterRuntime(std::move(fixture.body), config);
    define_humanoid_regions(result.runtime, result.bones);
    return result;
}

} // namespace sarx
