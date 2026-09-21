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

double effective_cut_resistance(
    const MaterialResponse& response,
    const Vec3& target_direction) {

    const double fiber_len = length(response.fiber_direction);
    const double target_len = length(target_direction);
    if (fiber_len <= 1e-12 || target_len <= 1e-12) {
        return response.cut_resistance;
    }

    const double alignment = std::abs(dot(
        response.fiber_direction / fiber_len,
        target_direction / target_len));
    const double aligned_weight = alignment * alignment;
    const double multiplier =
        response.transverse_cut_multiplier * (1.0 - aligned_weight)
        + response.longitudinal_cut_multiplier * aligned_weight;

    return response.cut_resistance * multiplier;
}

double resistance_for(
    const MaterialResponse& response,
    DamageMode mode,
    const Vec3& target_direction) {

    if (mode == DamageMode::Cut) {
        return effective_cut_resistance(response, target_direction);
    }
    return response.blunt_resistance;
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

DamageCandidates all_candidates(const Body& body) {
    DamageCandidates candidates;
    candidates.structural.reserve(body.structural_constraints().size());
    candidates.attachments.reserve(body.attachments().size());
    candidates.bone_joints.reserve(body.bones().size());

    for (ConstraintId id = 0; id < body.structural_constraints().size(); ++id) {
        if (body.structural_constraints()[id].active) {
            candidates.structural.push_back(id);
        }
    }
    for (ConstraintId id = 0; id < body.attachments().size(); ++id) {
        if (body.attachments()[id].active) {
            candidates.attachments.push_back(id);
        }
    }
    for (BoneId id = 0; id < body.bones().size(); ++id) {
        const auto& bone = body.bones()[id];
        if (bone.parent != kNoParent && bone.joint_to_parent_active) {
            candidates.bone_joints.push_back(id);
        }
    }
    return candidates;
}

void append_event(
    DamageReport& report,
    DamageEventId event_id,
    DamageSource source,
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
        event_id,
        source,
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
    if (response.cut_resistance <= 0.0
        || response.blunt_resistance <= 0.0
        || response.longitudinal_cut_multiplier <= 0.0
        || response.transverse_cut_multiplier <= 0.0
        || response.tensile_yield_strain < 0.0
        || response.tensile_break_strain < response.tensile_yield_strain
        || response.strain_damage_rate < 0.0) {
        throw std::invalid_argument("invalid material response");
    }
    responses_[material] = response;
}

MaterialResponse MaterialTable::get(MaterialId material) const {
    const auto it = responses_.find(material);
    return it == responses_.end() ? MaterialResponse{} : it->second;
}

std::size_t DamageCandidates::size() const {
    return structural.size() + attachments.size() + bone_joints.size();
}

std::size_t DamageReport::broken_count() const {
    return static_cast<std::size_t>(std::count_if(
        events.begin(),
        events.end(),
        [](const FractureEvent& e) { return e.broke; }));
}

DamageEventId DamageSystem::resolve_event_id(DamageEventId requested) {
    if (requested == 0) {
        return next_event_id_++;
    }

    next_event_id_ = std::max(next_event_id_, requested + 1);
    return requested;
}

DamageReport DamageSystem::apply_capsule(Body& body, const CapsuleDamage& input) {
    return apply_capsule(body, input, all_candidates(body));
}

DamageReport DamageSystem::apply_capsule(
    Body& body,
    const CapsuleDamage& input,
    const DamageCandidates& candidates) {

    if (input.radius <= 0.0 || input.energy < 0.0) {
        throw std::invalid_argument("invalid capsule damage");
    }

    CapsuleDamage damage = input;
    damage.event_id = resolve_event_id(damage.event_id);
    history_.push_back(DamageCommand{
        DamageCommandKind::Capsule,
        damage,
        {},
        {}
    });

    DamageReport report;
    report.event_id = damage.event_id;

    const auto structural_snapshot = body.structural_constraints();
    for (ConstraintId id : candidates.structural) {
        if (id >= structural_snapshot.size()) continue;
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
            resistance_for(material, damage.mode, p1 - p0));

        if (amount <= 0.0) continue;
        const bool was_active = c.active;
        body.damage_structural(id, amount);
        const bool broke = was_active && !body.structural_constraints()[id].active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::StructuralConstraint,
            id,
            c.material,
            midpoint(hit.point_a, hit.point_b),
            amount,
            broke);
    }

    const auto attachment_snapshot = body.attachments();
    for (ConstraintId id : candidates.attachments) {
        if (id >= attachment_snapshot.size()) continue;
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
            resistance_for(material, damage.mode, p1 - p0));

        if (amount <= 0.0) continue;
        const bool was_active = a.active;
        body.damage_attachment(id, amount);
        const bool broke = was_active && !body.attachments()[id].active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::AttachmentConstraint,
            id,
            a.material,
            midpoint(hit.point_a, hit.point_b),
            amount,
            broke);
    }

    const auto bone_snapshot = body.bones();
    for (BoneId id : candidates.bone_joints) {
        if (id >= bone_snapshot.size()) continue;
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
            resistance_for(material, damage.mode, p1 - p0));

        if (amount <= 0.0) continue;
        const bool was_active = bone.joint_to_parent_active;
        body.damage_bone_joint(id, amount);
        const bool broke = was_active && !body.bones()[id].joint_to_parent_active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::BoneJoint,
            id,
            bone.joint_material,
            midpoint(hit.point_a, hit.point_b),
            amount,
            broke);
    }

    if (damage.mode == DamageMode::Cut && report.broken_count() > 0) {
        Vec3 center{};
        std::size_t broken = 0;
        for (const auto& event : report.events) {
            if (!event.broke) continue;
            center += event.position;
            ++broken;
        }
        if (broken > 0) {
            center = center / static_cast<double>(broken);
            wounds_.push_back(WoundDescriptor{
                damage.event_id,
                center,
                normalized(damage.cut_normal),
                damage.radius,
                broken
            });
        }
    }

    return report;
}

DamageReport DamageSystem::apply_sphere(Body& body, const SphereDamage& input) {
    return apply_sphere(body, input, all_candidates(body));
}

DamageReport DamageSystem::apply_sphere(
    Body& body,
    const SphereDamage& input,
    const DamageCandidates& candidates) {

    if (input.radius <= 0.0 || input.energy < 0.0) {
        throw std::invalid_argument("invalid sphere damage");
    }

    SphereDamage damage = input;
    damage.event_id = resolve_event_id(damage.event_id);
    history_.push_back(DamageCommand{
        DamageCommandKind::Sphere,
        {},
        damage,
        {}
    });

    DamageReport report;
    report.event_id = damage.event_id;

    const auto structural_snapshot = body.structural_constraints();
    for (ConstraintId id : candidates.structural) {
        if (id >= structural_snapshot.size()) continue;
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
            resistance_for(material, damage.mode, p1 - p0));

        if (amount <= 0.0) continue;
        const bool was_active = c.active;
        body.damage_structural(id, amount);
        const bool broke = was_active && !body.structural_constraints()[id].active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::StructuralConstraint,
            id,
            c.material,
            hit.point_b,
            amount,
            broke);
    }

    const auto attachment_snapshot = body.attachments();
    for (ConstraintId id : candidates.attachments) {
        if (id >= attachment_snapshot.size()) continue;
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
            resistance_for(material, damage.mode, p1 - p0));

        if (amount <= 0.0) continue;
        const bool was_active = a.active;
        body.damage_attachment(id, amount);
        const bool broke = was_active && !body.attachments()[id].active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::AttachmentConstraint,
            id,
            a.material,
            hit.point_b,
            amount,
            broke);
    }

    const auto bone_snapshot = body.bones();
    for (BoneId id : candidates.bone_joints) {
        if (id >= bone_snapshot.size()) continue;
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
            resistance_for(material, damage.mode, p1 - p0));

        if (amount <= 0.0) continue;
        const bool was_active = bone.joint_to_parent_active;
        body.damage_bone_joint(id, amount);
        const bool broke = was_active && !body.bones()[id].joint_to_parent_active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::BoneJoint,
            id,
            bone.joint_material,
            hit.point_b,
            amount,
            broke);
    }

    return report;
}

DamageReport DamageSystem::apply_strain(Body& body, const StrainDamage& input) {
    if (input.dt <= 0.0) {
        throw std::invalid_argument("strain damage dt must be positive");
    }

    StrainDamage damage = input;
    damage.event_id = resolve_event_id(damage.event_id);
    history_.push_back(DamageCommand{
        DamageCommandKind::Strain,
        {},
        {},
        damage
    });

    DamageReport report;
    report.event_id = damage.event_id;

    const auto snapshot = body.structural_constraints();
    for (ConstraintId id = 0; id < snapshot.size(); ++id) {
        const auto& c = snapshot[id];
        if (!c.active || c.rest_length <= 1e-12) continue;

        const Vec3 p0 = body.particles()[c.a].position;
        const Vec3 p1 = body.particles()[c.b].position;
        const double current_length = length(p1 - p0);
        const double tensile_strain =
            std::max(0.0, current_length / c.rest_length - 1.0);

        const auto material = materials_.get(c.material);
        double amount = 0.0;

        if (tensile_strain >= material.tensile_break_strain) {
            amount = std::max(0.0, c.break_damage - c.damage);
        } else if (tensile_strain > material.tensile_yield_strain
                   && material.strain_damage_rate > 0.0) {
            amount = (tensile_strain - material.tensile_yield_strain)
                * material.strain_damage_rate
                * damage.dt;
        }

        if (amount <= 0.0) continue;

        const bool was_active = c.active;
        body.damage_structural(id, amount);
        const bool broke = was_active && !body.structural_constraints()[id].active;
        append_event(
            report,
            damage.event_id,
            DamageSource::Strain,
            DamageTargetKind::StructuralConstraint,
            id,
            c.material,
            midpoint(p0, p1),
            amount,
            broke);
    }

    return report;
}

DamageReport DamageSystem::apply(Body& body, const DamageCommand& command) {
    switch (command.kind) {
    case DamageCommandKind::Capsule:
        return apply_capsule(body, command.capsule);
    case DamageCommandKind::Sphere:
        return apply_sphere(body, command.sphere);
    case DamageCommandKind::Strain:
        return apply_strain(body, command.strain);
    }
    throw std::invalid_argument("unknown damage command");
}

std::vector<DamageReport> DamageSystem::replay(
    Body& body,
    const std::vector<DamageCommand>& commands) {

    std::vector<DamageReport> reports;
    reports.reserve(commands.size());
    for (const auto& command : commands) {
        reports.push_back(apply(body, command));
    }
    return reports;
}

void DamageSystem::clear_history() {
    history_.clear();
    next_event_id_ = 1;
}

void DamageSystem::clear_wounds() {
    wounds_.clear();
}

} // namespace sarx
