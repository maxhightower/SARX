#include "sarx/damage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sarx {
namespace {

struct SegmentDistanceResult {
    double distance_squared{};
    Vec3 point_a{};
    Vec3 point_b{};
};

double clamp01(double v) {
    return std::clamp(v, 0.0, 1.0);
}

SegmentDistanceResult segment_segment_distance(
    const Vec3& p1,
    const Vec3& q1,
    const Vec3& p2,
    const Vec3& q2) {

    constexpr double eps = 1e-12;
    const Vec3 d1 = q1 - p1;
    const Vec3 d2 = q2 - p2;
    const Vec3 r = p1 - p2;
    const double a = dot(d1, d1);
    const double e = dot(d2, d2);
    const double f = dot(d2, r);

    double s = 0.0;
    double t = 0.0;

    if (a <= eps && e <= eps) {
        return {length_squared(p1 - p2), p1, p2};
    }

    if (a <= eps) {
        t = clamp01(f / e);
    } else {
        const double c = dot(d1, r);
        if (e <= eps) {
            s = clamp01(-c / a);
        } else {
            const double b = dot(d1, d2);
            const double denom = a * e - b * b;
            if (std::abs(denom) > eps) {
                s = clamp01((b * f - c * e) / denom);
            }

            const double t_nom = b * s + f;
            if (t_nom < 0.0) {
                t = 0.0;
                s = clamp01(-c / a);
            } else if (t_nom > e) {
                t = 1.0;
                s = clamp01((b - c) / a);
            } else {
                t = t_nom / e;
            }
        }
    }

    const Vec3 c1 = p1 + d1 * s;
    const Vec3 c2 = p2 + d2 * t;
    return {length_squared(c1 - c2), c1, c2};
}

SegmentDistanceResult point_segment_distance(
    const Vec3& point,
    const Vec3& a,
    const Vec3& b) {

    const Vec3 ab = b - a;
    const double denom = dot(ab, ab);
    const double t = denom > 1e-12
        ? clamp01(dot(point - a, ab) / denom)
        : 0.0;
    const Vec3 closest = a + ab * t;
    return {length_squared(point - closest), point, closest};
}

double resistance_for(const MaterialResponse& response, DamageMode mode) {
    return mode == DamageMode::Cut
        ? response.cut_resistance
        : response.blunt_resistance;
}

double damage_from_distance(
    double distance_squared,
    double radius,
    double energy,
    double resistance) {

    if (radius <= 0.0 || energy < 0.0 || resistance <= 0.0) {
        throw std::invalid_argument("invalid spatial damage parameters");
    }

    const double distance = std::sqrt(std::max(0.0, distance_squared));
    if (distance > radius) {
        return 0.0;
    }

    const double proximity = 1.0 - (distance / radius);
    return energy * std::max(0.0, proximity) / resistance;
}

Vec3 midpoint(const Vec3& a, const Vec3& b) {
    return (a + b) * 0.5;
}

void append_event(
    DamageReport& report,
    DamageTargetKind kind,
    std::size_t id,
    MaterialId material,
    const Vec3& position,
    double applied_damage,
    bool broke) {

    if (applied_damage <= 0.0) {
        return;
    }

    report.events.push_back(FractureEvent{
        kind,
        id,
        material,
        position,
        applied_damage,
        broke
    });
}

} // namespace

void MaterialTable::set(MaterialId material, const MaterialResponse& response) {
    if (response.cut_resistance <= 0.0 || response.blunt_resistance <= 0.0) {
        throw std::invalid_argument("material resistances must be positive");
    }
    responses_[material] = response;
}

MaterialResponse MaterialTable::get(MaterialId material) const {
    const auto it = responses_.find(material);
    return it == responses_.end() ? MaterialResponse{} : it->second;
}

std::size_t DamageReport::broken_count() const {
    return static_cast<std::size_t>(std::count_if(
        events.begin(),
        events.end(),
        [](const FractureEvent& e) { return e.broke; }));
}

DamageReport DamageSystem::apply_capsule(Body& body, const CapsuleDamage& damage) const {
    if (damage.radius <= 0.0 || damage.energy < 0.0) {
        throw std::invalid_argument("invalid capsule damage");
    }

    DamageReport report;

    const auto structural_snapshot = body.structural_constraints();
    for (ConstraintId id = 0; id < structural_snapshot.size(); ++id) {
        const auto& c = structural_snapshot[id];
        if (!c.active) continue;

        const Vec3 p0 = body.particles()[c.a].position;
        const Vec3 p1 = body.particles()[c.b].position;
        const auto hit = segment_segment_distance(damage.a, damage.b, p0, p1);
        const auto material = materials_.get(c.material);
        const double amount = damage_from_distance(
            hit.distance_squared,
            damage.radius,
            damage.energy,
            resistance_for(material, damage.mode));

        if (amount <= 0.0) continue;
        const bool was_active = c.active;
        body.damage_structural(id, amount);
        const bool broke = was_active && !body.structural_constraints()[id].active;
        append_event(
            report,
            DamageTargetKind::StructuralConstraint,
            id,
            c.material,
            midpoint(hit.point_a, hit.point_b),
            amount,
            broke);
    }

    const auto attachment_snapshot = body.attachments();
    for (ConstraintId id = 0; id < attachment_snapshot.size(); ++id) {
        const auto& a = attachment_snapshot[id];
        if (!a.active) continue;

        const Vec3 p0 = body.particles()[a.particle].position;
        const Vec3 p1 = body.bones()[a.bone].animated_position + a.local_offset;
        const auto hit = segment_segment_distance(damage.a, damage.b, p0, p1);
        const auto material = materials_.get(a.material);
        const double amount = damage_from_distance(
            hit.distance_squared,
            damage.radius,
            damage.energy,
            resistance_for(material, damage.mode));

        if (amount <= 0.0) continue;
        const bool was_active = a.active;
        body.damage_attachment(id, amount);
        const bool broke = was_active && !body.attachments()[id].active;
        append_event(
            report,
            DamageTargetKind::AttachmentConstraint,
            id,
            a.material,
            midpoint(hit.point_a, hit.point_b),
            amount,
            broke);
    }

    const auto bone_snapshot = body.bones();
    for (BoneId id = 0; id < bone_snapshot.size(); ++id) {
        const auto& bone = bone_snapshot[id];
        if (bone.parent == kNoParent || !bone.joint_to_parent_active) continue;

        const Vec3 p0 = bone_snapshot[bone.parent].animated_position;
        const Vec3 p1 = bone.animated_position;
        const auto hit = segment_segment_distance(damage.a, damage.b, p0, p1);
        const auto material = materials_.get(bone.joint_material);
        const double amount = damage_from_distance(
            hit.distance_squared,
            damage.radius,
            damage.energy,
            resistance_for(material, damage.mode));

        if (amount <= 0.0) continue;
        const bool was_active = bone.joint_to_parent_active;
        body.damage_bone_joint(id, amount);
        const bool broke = was_active && !body.bones()[id].joint_to_parent_active;
        append_event(
            report,
            DamageTargetKind::BoneJoint,
            id,
            bone.joint_material,
            midpoint(hit.point_a, hit.point_b),
            amount,
            broke);
    }

    return report;
}

DamageReport DamageSystem::apply_sphere(Body& body, const SphereDamage& damage) const {
    if (damage.radius <= 0.0 || damage.energy < 0.0) {
        throw std::invalid_argument("invalid sphere damage");
    }

    DamageReport report;

    const auto structural_snapshot = body.structural_constraints();
    for (ConstraintId id = 0; id < structural_snapshot.size(); ++id) {
        const auto& c = structural_snapshot[id];
        if (!c.active) continue;

        const Vec3 p0 = body.particles()[c.a].position;
        const Vec3 p1 = body.particles()[c.b].position;
        const auto hit = point_segment_distance(damage.center, p0, p1);
        const auto material = materials_.get(c.material);
        const double amount = damage_from_distance(
            hit.distance_squared,
            damage.radius,
            damage.energy,
            resistance_for(material, damage.mode));

        if (amount <= 0.0) continue;
        const bool was_active = c.active;
        body.damage_structural(id, amount);
        const bool broke = was_active && !body.structural_constraints()[id].active;
        append_event(
            report,
            DamageTargetKind::StructuralConstraint,
            id,
            c.material,
            hit.point_b,
            amount,
            broke);
    }

    const auto attachment_snapshot = body.attachments();
    for (ConstraintId id = 0; id < attachment_snapshot.size(); ++id) {
        const auto& a = attachment_snapshot[id];
        if (!a.active) continue;

        const Vec3 p0 = body.particles()[a.particle].position;
        const Vec3 p1 = body.bones()[a.bone].animated_position + a.local_offset;
        const auto hit = point_segment_distance(damage.center, p0, p1);
        const auto material = materials_.get(a.material);
        const double amount = damage_from_distance(
            hit.distance_squared,
            damage.radius,
            damage.energy,
            resistance_for(material, damage.mode));

        if (amount <= 0.0) continue;
        const bool was_active = a.active;
        body.damage_attachment(id, amount);
        const bool broke = was_active && !body.attachments()[id].active;
        append_event(
            report,
            DamageTargetKind::AttachmentConstraint,
            id,
            a.material,
            hit.point_b,
            amount,
            broke);
    }

    const auto bone_snapshot = body.bones();
    for (BoneId id = 0; id < bone_snapshot.size(); ++id) {
        const auto& bone = bone_snapshot[id];
        if (bone.parent == kNoParent || !bone.joint_to_parent_active) continue;

        const Vec3 p0 = bone_snapshot[bone.parent].animated_position;
        const Vec3 p1 = bone.animated_position;
        const auto hit = point_segment_distance(damage.center, p0, p1);
        const auto material = materials_.get(bone.joint_material);
        const double amount = damage_from_distance(
            hit.distance_squared,
            damage.radius,
            damage.energy,
            resistance_for(material, damage.mode));

        if (amount <= 0.0) continue;
        const bool was_active = bone.joint_to_parent_active;
        body.damage_bone_joint(id, amount);
        const bool broke = was_active && !body.bones()[id].joint_to_parent_active;
        append_event(
            report,
            DamageTargetKind::BoneJoint,
            id,
            bone.joint_material,
            hit.point_b,
            amount,
            broke);
    }

    return report;
}

} // namespace sarx
