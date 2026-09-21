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

SolverDomain solver_domain(const AdaptiveDamageDomain& domain) {
    SolverDomain result;
    result.particles = domain.particles;
    result.structural = domain.structural;
    result.tetrahedral = domain.tetrahedral;
    result.attachments = domain.attachments;
    return result;
}

AdaptiveDamageDomain close_over_free_islands(
    const Body& body,
    const AdaptiveDamageDomain& seed) {

    AdaptiveDamageDomain result = seed;

    std::vector<std::uint8_t> selected(
        body.particles().size(),
        0u);

    for (const ParticleId id : seed.particles) {
        if (id >= selected.size()) {
            throw std::out_of_range(
                "adaptive free-island seed particle out of range");
        }
        selected[id] = 1u;
    }

    bool added_free_island = false;

    for (const auto& island : body.islands()) {
        if (island.rig_authoritative) {
            continue;
        }

        bool intersects_seed = false;
        for (const ParticleId id : island.particles) {
            if (selected[id]) {
                intersects_seed = true;
                break;
            }
        }

        if (!intersects_seed) {
            continue;
        }

        for (const ParticleId id : island.particles) {
            selected[id] = 1u;
        }
        added_free_island = true;
    }

    if (!added_free_island) {
        return result;
    }

    result.particles.clear();
    for (ParticleId id = 0; id < selected.size(); ++id) {
        if (selected[id]) {
            result.particles.push_back(id);
        }
    }

    auto append_unique_active_primitives = [&body, &selected, &result]() {
        std::vector<std::uint8_t> structural_seen(
            body.structural_constraints().size(),
            0u);
        for (const auto id : result.structural) {
            if (id < structural_seen.size()) structural_seen[id] = 1u;
        }

        for (ConstraintId id = 0;
             id < body.structural_constraints().size();
             ++id) {

            const auto& constraint =
                body.structural_constraints()[id];
            if (!constraint.active || structural_seen[id]) continue;

            if (selected[constraint.a] && selected[constraint.b]) {
                result.structural.push_back(id);
                structural_seen[id] = 1u;
            }
        }

        std::vector<std::uint8_t> tetrahedral_seen(
            body.tetrahedral_constraints().size(),
            0u);
        for (const auto id : result.tetrahedral) {
            if (id < tetrahedral_seen.size()) tetrahedral_seen[id] = 1u;
        }

        for (ConstraintId id = 0;
             id < body.tetrahedral_constraints().size();
             ++id) {

            const auto& tet =
                body.tetrahedral_constraints()[id];
            if (!tet.active || tetrahedral_seen[id]) continue;

            if (selected[tet.a]
                && selected[tet.b]
                && selected[tet.c]
                && selected[tet.d]) {
                result.tetrahedral.push_back(id);
                tetrahedral_seen[id] = 1u;
            }
        }

        std::vector<std::uint8_t> attachment_seen(
            body.attachments().size(),
            0u);
        for (const auto id : result.attachments) {
            if (id < attachment_seen.size()) attachment_seen[id] = 1u;
        }

        for (ConstraintId id = 0;
             id < body.attachments().size();
             ++id) {

            const auto& attachment = body.attachments()[id];
            if (!attachment.active || attachment_seen[id]) continue;

            if (selected[attachment.particle]) {
                result.attachments.push_back(id);
                attachment_seen[id] = 1u;
            }
        }
    };

    append_unique_active_primitives();

    std::sort(result.structural.begin(), result.structural.end());
    result.structural.erase(
        std::unique(result.structural.begin(), result.structural.end()),
        result.structural.end());

    std::sort(result.tetrahedral.begin(), result.tetrahedral.end());
    result.tetrahedral.erase(
        std::unique(result.tetrahedral.begin(), result.tetrahedral.end()),
        result.tetrahedral.end());

    std::sort(result.attachments.begin(), result.attachments.end());
    result.attachments.erase(
        std::unique(result.attachments.begin(), result.attachments.end()),
        result.attachments.end());

    return result;
}

void AdaptiveDomainTracker::reset(const Body& body) {
    particle_count_ = body.particles().size();
    structural_count_ = body.structural_constraints().size();
    tetrahedral_count_ = body.tetrahedral_constraints().size();
    attachment_count_ = body.attachments().size();

    particle_refs_.assign(particle_count_, 0);
    structural_refs_.assign(structural_count_, 0);
    tetrahedral_refs_.assign(tetrahedral_count_, 0);
    attachment_refs_.assign(attachment_count_, 0);
    entries_.clear();
}

void AdaptiveDomainTracker::validate_shape(const Body& body) const {
    if (particle_count_ != body.particles().size()
        || structural_count_ != body.structural_constraints().size()
        || tetrahedral_count_ != body.tetrahedral_constraints().size()
        || attachment_count_ != body.attachments().size()) {
        throw std::logic_error(
            "adaptive domain tracker body topology changed; reset required");
    }
}

void AdaptiveDomainTracker::add_domain_refs(
    const AdaptiveDamageDomain& domain) {

    for (const auto id : domain.particles) {
        ++particle_refs_.at(id);
    }
    for (const auto id : domain.structural) {
        ++structural_refs_.at(id);
    }
    for (const auto id : domain.tetrahedral) {
        ++tetrahedral_refs_.at(id);
    }
    for (const auto id : domain.attachments) {
        ++attachment_refs_.at(id);
    }
}

void AdaptiveDomainTracker::remove_domain_refs(
    const AdaptiveDamageDomain& domain) {

    auto decrement = [](auto& refs, const auto& ids) {
        for (const auto id : ids) {
            auto& value = refs.at(id);
            if (value == 0) {
                throw std::logic_error(
                    "adaptive domain reference count underflow");
            }
            --value;
        }
    };

    decrement(particle_refs_, domain.particles);
    decrement(structural_refs_, domain.structural);
    decrement(tetrahedral_refs_, domain.tetrahedral);
    decrement(attachment_refs_, domain.attachments);
}

void AdaptiveDomainTracker::upsert_wound(
    const Body& body,
    const WoundDescriptor& wound,
    double halo) {

    if (halo < 0.0) {
        throw std::invalid_argument("adaptive wound halo must be non-negative");
    }

    if (particle_refs_.empty()
        && structural_refs_.empty()
        && tetrahedral_refs_.empty()
        && attachment_refs_.empty()
        && entries_.empty()) {
        reset(body);
    } else {
        validate_shape(body);
    }

    auto existing = entries_.find(wound.event_id);
    if (existing != entries_.end()) {
        remove_domain_refs(existing->second.domain);
        entries_.erase(existing);
    }

    Entry entry;
    entry.wound = wound;
    entry.halo = halo;
    entry.domain = select_damage_domain(body, wound, halo);

    add_domain_refs(entry.domain);
    entries_.emplace(wound.event_id, std::move(entry));
}

bool AdaptiveDomainTracker::remove_wound(DamageEventId event_id) {
    const auto it = entries_.find(event_id);
    if (it == entries_.end()) {
        return false;
    }

    remove_domain_refs(it->second.domain);
    entries_.erase(it);
    return true;
}

void AdaptiveDomainTracker::refit(const Body& body) {
    validate_shape(body);

    std::fill(particle_refs_.begin(), particle_refs_.end(), 0);
    std::fill(structural_refs_.begin(), structural_refs_.end(), 0);
    std::fill(tetrahedral_refs_.begin(), tetrahedral_refs_.end(), 0);
    std::fill(attachment_refs_.begin(), attachment_refs_.end(), 0);

    for (auto& [event_id, entry] : entries_) {
        (void)event_id;
        entry.domain = select_damage_domain(
            body,
            entry.wound,
            entry.halo);
        add_domain_refs(entry.domain);
    }
}

SolverDomain AdaptiveDomainTracker::combined_solver_domain() const {
    SolverDomain result;

    auto collect = [](const auto& refs, auto& out) {
        for (std::size_t id = 0; id < refs.size(); ++id) {
            if (refs[id] > 0) {
                out.push_back(id);
            }
        }
    };

    collect(particle_refs_, result.particles);
    collect(structural_refs_, result.structural);
    collect(tetrahedral_refs_, result.tetrahedral);
    collect(attachment_refs_, result.attachments);

    return result;
}

} // namespace sarx
