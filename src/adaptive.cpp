#include "sarx/adaptive.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sarx {
namespace {

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

double point_aabb_distance_squared(
    const Vec3& p,
    const Vec3& lo,
    const Vec3& hi) {

    auto axis_distance = [](double v, double a, double b) {
        if (v < a) return a - v;
        if (v > b) return v - b;
        return 0.0;
    };

    const double dx = axis_distance(p.x, lo.x, hi.x);
    const double dy = axis_distance(p.y, lo.y, hi.y);
    const double dz = axis_distance(p.z, lo.z, hi.z);
    return dx * dx + dy * dy + dz * dz;
}

Vec3 min_vec(const Vec3& a, const Vec3& b) {
    return {
        std::min(a.x, b.x),
        std::min(a.y, b.y),
        std::min(a.z, b.z)
    };
}

Vec3 max_vec(const Vec3& a, const Vec3& b) {
    return {
        std::max(a.x, b.x),
        std::max(a.y, b.y),
        std::max(a.z, b.z)
    };
}

} // namespace

std::size_t AdaptiveDamageDomain::primitive_count() const {
    return structural.size()
        + tetrahedral.size()
        + attachments.size()
        + bone_joints.size();
}

AdaptiveDamageDomain select_damage_domain(
    const Body& body,
    const Vec3& center,
    double radius) {

    if (radius <= 0.0) {
        throw std::invalid_argument("adaptive damage radius must be positive");
    }

    AdaptiveDamageDomain domain;
    domain.center = center;
    domain.radius = radius;
    const double radius_sq = radius * radius;

    for (ParticleId id = 0; id < body.particles().size(); ++id) {
        if (length_squared(body.particles()[id].position - center) <= radius_sq) {
            domain.particles.push_back(id);
        }
    }

    for (ConstraintId id = 0; id < body.structural_constraints().size(); ++id) {
        const auto& c = body.structural_constraints()[id];
        if (!c.active) continue;
        const Vec3 a = body.particles()[c.a].position;
        const Vec3 b = body.particles()[c.b].position;
        if (point_segment_distance_squared(center, a, b) <= radius_sq) {
            domain.structural.push_back(id);
        }
    }

    for (ConstraintId id = 0; id < body.tetrahedral_constraints().size(); ++id) {
        const auto& t = body.tetrahedral_constraints()[id];
        if (!t.active) continue;

        const Vec3 p0 = body.particles()[t.a].position;
        const Vec3 p1 = body.particles()[t.b].position;
        const Vec3 p2 = body.particles()[t.c].position;
        const Vec3 p3 = body.particles()[t.d].position;

        const Vec3 lo = min_vec(min_vec(p0, p1), min_vec(p2, p3));
        const Vec3 hi = max_vec(max_vec(p0, p1), max_vec(p2, p3));

        if (point_aabb_distance_squared(center, lo, hi) <= radius_sq) {
            domain.tetrahedral.push_back(id);
        }
    }

    for (ConstraintId id = 0; id < body.attachments().size(); ++id) {
        const auto& a = body.attachments()[id];
        if (!a.active) continue;

        const Vec3 p0 = body.particles()[a.particle].position;
        const Vec3 p1 =
            body.bones()[a.bone].animated_position + a.local_offset;
        if (point_segment_distance_squared(center, p0, p1) <= radius_sq) {
            domain.attachments.push_back(id);
        }
    }

    for (BoneId id = 0; id < body.bones().size(); ++id) {
        const auto& bone = body.bones()[id];
        if (bone.parent == kNoParent || !bone.joint_to_parent_active) continue;

        const Vec3 p0 = body.bones()[bone.parent].animated_position;
        const Vec3 p1 = bone.animated_position;
        const double reach = radius + bone.joint_radius;
        if (point_segment_distance_squared(center, p0, p1)
            <= reach * reach) {
            domain.bone_joints.push_back(id);
        }
    }

    return domain;
}

AdaptiveDamageDomain select_damage_domain(
    const Body& body,
    const WoundDescriptor& wound,
    double halo) {

    if (halo < 0.0) {
        throw std::invalid_argument("adaptive wound halo must be non-negative");
    }
    return select_damage_domain(
        body,
        wound.center,
        wound.radius + halo);
}

} // namespace sarx
