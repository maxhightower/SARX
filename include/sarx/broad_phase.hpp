#pragma once

#include "sarx/damage.hpp"

#include <cstddef>
#include <memory>

namespace sarx {

struct BroadPhaseQuery {
    DamageCandidates candidates;
    std::size_t visited_cells{};
    std::size_t indexed_primitives{};
};

class DamageBroadPhase {
public:
    DamageBroadPhase();
    ~DamageBroadPhase();

    DamageBroadPhase(DamageBroadPhase&&) noexcept;
    DamageBroadPhase& operator=(DamageBroadPhase&&) noexcept;

    DamageBroadPhase(const DamageBroadPhase&) = delete;
    DamageBroadPhase& operator=(const DamageBroadPhase&) = delete;

    void rebuild(const Body& body, double cell_size = 0.5);

    [[nodiscard]] BroadPhaseQuery query_capsule(const CapsuleDamage& damage) const;
    [[nodiscard]] BroadPhaseQuery query_sphere(const SphereDamage& damage) const;

    [[nodiscard]] double cell_size() const;
    [[nodiscard]] std::size_t indexed_primitives() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sarx
