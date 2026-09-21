#include "sarx/damage.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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

double signed_tetra_volume(
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    const Vec3& d) {
    return dot(b - a, cross(c - a, d - a)) / 6.0;
}

bool point_in_tetra(
    const Vec3& p,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    const Vec3& d) {

    const double total = signed_tetra_volume(a, b, c, d);
    if (std::abs(total) <= 1e-12) return false;

    const double w0 = signed_tetra_volume(p, b, c, d) / total;
    const double w1 = signed_tetra_volume(a, p, c, d) / total;
    const double w2 = signed_tetra_volume(a, b, p, d) / total;
    const double w3 = signed_tetra_volume(a, b, c, p) / total;
    constexpr double eps = 1e-9;

    return w0 >= -eps && w1 >= -eps && w2 >= -eps && w3 >= -eps
        && w0 <= 1.0 + eps && w1 <= 1.0 + eps
        && w2 <= 1.0 + eps && w3 <= 1.0 + eps;
}

bool segment_triangle_intersection(
    const Vec3& p,
    const Vec3& q,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    Vec3& hit) {

    constexpr double eps = 1e-10;
    const Vec3 dir = q - p;
    const Vec3 e1 = b - a;
    const Vec3 e2 = c - a;
    const Vec3 h = cross(dir, e2);
    const double det = dot(e1, h);

    if (std::abs(det) <= eps) return false;

    const double inv_det = 1.0 / det;
    const Vec3 s = p - a;
    const double u = dot(s, h) * inv_det;
    if (u < -eps || u > 1.0 + eps) return false;

    const Vec3 qv = cross(s, e1);
    const double v = dot(dir, qv) * inv_det;
    if (v < -eps || u + v > 1.0 + eps) return false;

    const double t = dot(e2, qv) * inv_det;
    if (t < -eps || t > 1.0 + eps) return false;

    hit = p + dir * std::clamp(t, 0.0, 1.0);
    return true;
}

Vec3 closest_point_on_triangle(
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

SegmentDistanceResult segment_triangle_distance(
    const Vec3& p,
    const Vec3& q,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c) {

    Vec3 hit{};
    if (segment_triangle_intersection(p, q, a, b, c, hit)) {
        return {0.0, hit, hit};
    }

    SegmentDistanceResult best{
        std::numeric_limits<double>::infinity(),
        {},
        {}
    };

    const auto consider = [&best](const SegmentDistanceResult& candidate) {
        if (candidate.distance_squared < best.distance_squared) {
            best = candidate;
        }
    };

    const Vec3 p_triangle = closest_point_on_triangle(p, a, b, c);
    consider({length_squared(p - p_triangle), p, p_triangle});

    const Vec3 q_triangle = closest_point_on_triangle(q, a, b, c);
    consider({length_squared(q - q_triangle), q, q_triangle});

    consider(segment_segment_distance(p, q, a, b));
    consider(segment_segment_distance(p, q, b, c));
    consider(segment_segment_distance(p, q, c, a));

    return best;
}

SegmentDistanceResult segment_tetra_distance(
    const Vec3& p,
    const Vec3& q,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    const Vec3& d) {

    Vec3 hit{};
    if (point_in_tetra(p, a, b, c, d)) {
        return {0.0, p, p};
    }
    if (point_in_tetra(q, a, b, c, d)) {
        return {0.0, q, q};
    }

    const std::array<std::array<Vec3, 3>, 4> faces{{
        {a, b, c},
        {a, b, d},
        {a, c, d},
        {b, c, d}
    }};

    SegmentDistanceResult best{
        std::numeric_limits<double>::infinity(),
        {},
        {}
    };

    for (const auto& face : faces) {
        const auto candidate = segment_triangle_distance(
            p, q, face[0], face[1], face[2]);
        if (candidate.distance_squared < best.distance_squared) {
            best = candidate;
        }
        if (best.distance_squared <= 1e-20) {
            break;
        }
    }

    return best;
}

struct PlaneDiskHit {
    bool hit{false};
    Vec3 position{};
    double radial_distance_squared{};
};

PlaneDiskHit segment_plane_disk_hit(
    const Vec3& a,
    const Vec3& b,
    const Vec3& center,
    const Vec3& normal,
    double radius) {

    constexpr double eps = 1e-10;
    const double da = dot(a - center, normal);
    const double db = dot(b - center, normal);

    // A segment lying in the cut plane is not a crossing connection.
    if (std::abs(da) <= eps && std::abs(db) <= eps) {
        return {};
    }

    const double denom = da - db;
    if (std::abs(denom) <= eps) {
        return {};
    }

    const double t = da / denom;
    if (t < -eps || t > 1.0 + eps) {
        return {};
    }

    const Vec3 hit =
        a + (b - a) * std::clamp(t, 0.0, 1.0);
    const double radial_sq = length_squared(hit - center);

    if (radial_sq > radius * radius) {
        return {};
    }

    return {true, hit, radial_sq};
}

PlaneDiskHit tetra_plane_disk_hit(
    const Vec3& center,
    const Vec3& normal,
    double radius,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    const Vec3& d) {

    if (point_in_tetra(center, a, b, c, d)) {
        return {true, center, 0.0};
    }

    const std::array<std::array<Vec3, 2>, 6> edges{{
        {a, b},
        {a, c},
        {a, d},
        {b, c},
        {b, d},
        {c, d}
    }};

    PlaneDiskHit best;
    best.radial_distance_squared =
        std::numeric_limits<double>::infinity();

    for (const auto& edge : edges) {
        const auto hit = segment_plane_disk_hit(
            edge[0],
            edge[1],
            center,
            normal,
            radius);

        if (hit.hit
            && hit.radial_distance_squared
                < best.radial_distance_squared) {
            best = hit;
        }
    }

    return best;
}

DamageCandidates all_candidates(const Body& body) {
    DamageCandidates candidates;
    candidates.structural.reserve(body.structural_constraints().size());
    candidates.tetrahedral.reserve(body.tetrahedral_constraints().size());
    candidates.attachments.reserve(body.attachments().size());
    candidates.bone_joints.reserve(body.bones().size());

    for (ConstraintId id = 0; id < body.structural_constraints().size(); ++id) {
        if (body.structural_constraints()[id].active) {
            candidates.structural.push_back(id);
        }
    }
    for (ConstraintId id = 0; id < body.tetrahedral_constraints().size(); ++id) {
        if (body.tetrahedral_constraints()[id].active) {
            candidates.tetrahedral.push_back(id);
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
    return structural.size()
        + tetrahedral.size()
        + attachments.size()
        + bone_joints.size();
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
    DamageCommand command;
    command.kind = DamageCommandKind::Capsule;
    command.capsule = damage;
    history_.push_back(command);

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
        auto material = materials_.get(c.material);
        const Vec3 rest_fiber =
            length_squared(c.material_fiber_rest) > 1e-12
            ? c.material_fiber_rest
            : material.fiber_direction;
        material.fiber_direction = rotate_between(
            c.rest_direction,
            p1 - p0,
            rest_fiber);
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

    if (damage.mode == DamageMode::Cut) {
        const auto tetra_snapshot = body.tetrahedral_constraints();
        for (ConstraintId id : candidates.tetrahedral) {
            if (id >= tetra_snapshot.size()) continue;
            const auto& t = tetra_snapshot[id];
            if (!t.active) continue;

            const Vec3 p0 = body.particles()[t.a].position;
            const Vec3 p1 = body.particles()[t.b].position;
            const Vec3 p2 = body.particles()[t.c].position;
            const Vec3 p3 = body.particles()[t.d].position;

            const auto proximity = segment_tetra_distance(
                damage.a,
                damage.b,
                p0,
                p1,
                p2,
                p3);

            const auto material = materials_.get(t.material);
            const double amount = damage_from_distance(
                proximity.distance_squared,
                damage.radius,
                damage.energy,
                material.cut_resistance);
            if (amount <= 0.0) continue;

            const bool was_active = t.active;
            body.damage_tetrahedral(id, amount);
            const bool broke =
                was_active && !body.tetrahedral_constraints()[id].active;

            append_event(
                report,
                damage.event_id,
                DamageSource::Spatial,
                DamageTargetKind::TetrahedralConstraint,
                id,
                t.material,
                midpoint(proximity.point_a, proximity.point_b),
                amount,
                broke);
        }
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
            damage.radius + bone.joint_radius,
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

DamageReport DamageSystem::apply_plane_cut(
    Body& body,
    const PlaneCutDamage& input) {

    if (input.radius <= 0.0
        || input.energy < 0.0
        || length_squared(input.normal) <= 1e-12) {
        throw std::invalid_argument("invalid plane cut damage");
    }

    PlaneCutDamage damage = input;
    damage.normal = normalized(damage.normal);
    damage.event_id = resolve_event_id(damage.event_id);

    DamageCommand command;
    command.kind = DamageCommandKind::PlaneCut;
    command.plane_cut = damage;
    history_.push_back(command);

    DamageReport report;
    report.event_id = damage.event_id;

    const auto structural_snapshot = body.structural_constraints();
    for (ConstraintId id = 0; id < structural_snapshot.size(); ++id) {
        const auto& constraint = structural_snapshot[id];
        if (!constraint.active) continue;

        const Vec3 p0 = body.particles()[constraint.a].position;
        const Vec3 p1 = body.particles()[constraint.b].position;
        const auto hit = segment_plane_disk_hit(
            p0,
            p1,
            damage.center,
            damage.normal,
            damage.radius);
        if (!hit.hit) continue;

        auto material = materials_.get(constraint.material);
        const Vec3 rest_fiber =
            length_squared(constraint.material_fiber_rest) > 1e-12
            ? constraint.material_fiber_rest
            : material.fiber_direction;
        material.fiber_direction = rotate_between(
            constraint.rest_direction,
            p1 - p0,
            rest_fiber);

        const double amount =
            damage.energy
            / resistance_for(
                material,
                DamageMode::Cut,
                p1 - p0);
        if (amount <= 0.0) continue;

        const bool was_active = constraint.active;
        body.damage_structural(id, amount);
        const bool broke =
            was_active && !body.structural_constraints()[id].active;

        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::StructuralConstraint,
            id,
            constraint.material,
            hit.position,
            amount,
            broke);
    }

    const auto tetra_snapshot = body.tetrahedral_constraints();
    for (ConstraintId id = 0; id < tetra_snapshot.size(); ++id) {
        const auto& tet = tetra_snapshot[id];
        if (!tet.active) continue;

        const Vec3 p0 = body.particles()[tet.a].position;
        const Vec3 p1 = body.particles()[tet.b].position;
        const Vec3 p2 = body.particles()[tet.c].position;
        const Vec3 p3 = body.particles()[tet.d].position;

        const auto hit = tetra_plane_disk_hit(
            damage.center,
            damage.normal,
            damage.radius,
            p0,
            p1,
            p2,
            p3);
        if (!hit.hit) continue;

        const auto material = materials_.get(tet.material);
        const double amount =
            damage.energy / material.cut_resistance;
        if (amount <= 0.0) continue;

        const bool was_active = tet.active;
        body.damage_tetrahedral(id, amount);
        const bool broke =
            was_active && !body.tetrahedral_constraints()[id].active;

        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::TetrahedralConstraint,
            id,
            tet.material,
            hit.position,
            amount,
            broke);
    }

    const auto attachment_snapshot = body.attachments();
    for (ConstraintId id = 0; id < attachment_snapshot.size(); ++id) {
        const auto& attachment = attachment_snapshot[id];
        if (!attachment.active) continue;

        const Vec3 p0 =
            body.particles()[attachment.particle].position;
        const Vec3 p1 =
            body.bones()[attachment.bone].animated_position
            + attachment.local_offset;

        const auto hit = segment_plane_disk_hit(
            p0,
            p1,
            damage.center,
            damage.normal,
            damage.radius);
        if (!hit.hit) continue;

        const auto material = materials_.get(attachment.material);
        const double amount =
            damage.energy
            / resistance_for(
                material,
                DamageMode::Cut,
                p1 - p0);
        if (amount <= 0.0) continue;

        const bool was_active = attachment.active;
        body.damage_attachment(id, amount);
        const bool broke =
            was_active && !body.attachments()[id].active;

        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::AttachmentConstraint,
            id,
            attachment.material,
            hit.position,
            amount,
            broke);
    }

    const auto bone_snapshot = body.bones();
    for (BoneId id = 0; id < bone_snapshot.size(); ++id) {
        const auto& bone = bone_snapshot[id];
        if (bone.parent == kNoParent
            || !bone.joint_to_parent_active) {
            continue;
        }

        const Vec3 p0 =
            bone_snapshot[bone.parent].animated_position;
        const Vec3 p1 = bone.animated_position;

        const auto hit = segment_plane_disk_hit(
            p0,
            p1,
            damage.center,
            damage.normal,
            damage.radius + bone.joint_radius);
        if (!hit.hit) continue;

        const auto material =
            materials_.get(bone.joint_material);
        const double amount =
            damage.energy
            / resistance_for(
                material,
                DamageMode::Cut,
                p1 - p0);
        if (amount <= 0.0) continue;

        const bool was_active = bone.joint_to_parent_active;
        body.damage_bone_joint(id, amount);
        const bool broke =
            was_active && !body.bones()[id].joint_to_parent_active;

        append_event(
            report,
            damage.event_id,
            DamageSource::Spatial,
            DamageTargetKind::BoneJoint,
            id,
            bone.joint_material,
            hit.position,
            amount,
            broke);
    }

    if (report.broken_count() > 0) {
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
                damage.normal,
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
    DamageCommand command;
    command.kind = DamageCommandKind::Sphere;
    command.sphere = damage;
    history_.push_back(command);

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
    DamageCommand command;
    command.kind = DamageCommandKind::Strain;
    command.strain = damage;
    history_.push_back(command);

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
    case DamageCommandKind::PlaneCut:
        return apply_plane_cut(body, command.plane_cut);
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
