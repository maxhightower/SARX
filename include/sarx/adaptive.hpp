#pragma once

#include "sarx/damage.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace sarx {

struct AdaptiveDamageDomain {
    Vec3 center{};
    double radius{};

    std::vector<ParticleId> particles;
    std::vector<ConstraintId> structural;
    std::vector<ConstraintId> tetrahedral;
    std::vector<ConstraintId> attachments;
    std::vector<BoneId> bone_joints;

    [[nodiscard]] std::size_t primitive_count() const;
};

[[nodiscard]] AdaptiveDamageDomain select_damage_domain(
    const Body& body,
    const Vec3& center,
    double radius);

[[nodiscard]] AdaptiveDamageDomain select_damage_domain(
    const Body& body,
    const WoundDescriptor& wound,
    double halo);

[[nodiscard]] SolverDomain solver_domain(
    const AdaptiveDamageDomain& domain);

[[nodiscard]] AdaptiveDamageDomain close_over_free_islands(
    const Body& body,
    const AdaptiveDamageDomain& seed);

class AdaptiveDomainTracker {
public:
    void reset(const Body& body);

    void upsert_wound(
        const Body& body,
        const WoundDescriptor& wound,
        double halo);

    [[nodiscard]] bool remove_wound(DamageEventId event_id);

    // Re-evaluate all stored wound domains after large body deformation.
    void refit(const Body& body);

    [[nodiscard]] AdaptiveDamageDomain combined_damage_domain() const;
    [[nodiscard]] SolverDomain combined_solver_domain() const;
    [[nodiscard]] std::size_t active_wound_count() const {
        return entries_.size();
    }

private:
    struct Entry {
        WoundDescriptor wound{};
        double halo{};
        AdaptiveDamageDomain domain{};
    };

    void validate_shape(const Body& body) const;
    void add_domain_refs(const AdaptiveDamageDomain& domain);
    void remove_domain_refs(const AdaptiveDamageDomain& domain);

    std::size_t particle_count_{};
    std::size_t structural_count_{};
    std::size_t tetrahedral_count_{};
    std::size_t attachment_count_{};

    std::vector<std::size_t> particle_refs_;
    std::vector<std::size_t> structural_refs_;
    std::vector<std::size_t> tetrahedral_refs_;
    std::vector<std::size_t> attachment_refs_;

    std::unordered_map<DamageEventId, Entry> entries_;
};

} // namespace sarx
