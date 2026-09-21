#pragma once

#include "sarx/damage.hpp"

#include <cstddef>
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

} // namespace sarx
