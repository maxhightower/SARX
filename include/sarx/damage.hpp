#pragma once

#include "sarx/body.hpp"

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace sarx {

using DamageEventId = std::uint64_t;

enum class DamageMode {
    Cut,
    Blunt
};

enum class DamageTargetKind {
    StructuralConstraint,
    AttachmentConstraint,
    BoneJoint
};

enum class DamageSource {
    Spatial,
    Strain
};

enum class DamageCommandKind {
    Capsule,
    Sphere,
    Strain
};

struct MaterialResponse {
    double cut_resistance{1.0};
    double blunt_resistance{1.0};

    // Optional world-space fiber model. A zero vector disables anisotropy.
    Vec3 fiber_direction{};
    double longitudinal_cut_multiplier{1.0};
    double transverse_cut_multiplier{1.0};

    // Structural self-failure model.
    double tensile_yield_strain{1.0e9};
    double tensile_break_strain{1.0e9};
    double strain_damage_rate{0.0};
};

class MaterialTable {
public:
    void set(MaterialId material, const MaterialResponse& response);
    [[nodiscard]] MaterialResponse get(MaterialId material) const;

private:
    std::unordered_map<MaterialId, MaterialResponse> responses_;
};

struct CapsuleDamage {
    Vec3 a{};
    Vec3 b{};
    double radius{0.05};
    double energy{1.0};
    DamageMode mode{DamageMode::Cut};
    DamageEventId event_id{0};

    // Optional persistent cut-surface normal for downstream wound rendering.
    Vec3 cut_normal{};
};

struct SphereDamage {
    Vec3 center{};
    double radius{0.1};
    double energy{1.0};
    DamageMode mode{DamageMode::Blunt};
    DamageEventId event_id{0};
};

struct StrainDamage {
    double dt{1.0 / 60.0};
    DamageEventId event_id{0};
};

struct DamageCommand {
    DamageCommandKind kind{DamageCommandKind::Capsule};
    CapsuleDamage capsule{};
    SphereDamage sphere{};
    StrainDamage strain{};
};

struct FractureEvent {
    DamageEventId event_id{};
    DamageSource source{DamageSource::Spatial};
    DamageTargetKind target_kind{DamageTargetKind::StructuralConstraint};
    std::size_t target_id{};
    MaterialId material{kDefaultMaterial};
    Vec3 position{};
    double applied_damage{};
    bool broke{false};
};

struct DamageReport {
    DamageEventId event_id{};
    std::vector<FractureEvent> events;

    [[nodiscard]] std::size_t broken_count() const;
};

struct WoundDescriptor {
    DamageEventId event_id{};
    Vec3 center{};
    Vec3 normal{};
    double radius{};
    std::size_t broken_target_count{};
};

class DamageSystem {
public:
    MaterialTable& materials() { return materials_; }
    [[nodiscard]] const MaterialTable& materials() const { return materials_; }

    [[nodiscard]] DamageReport apply_capsule(Body& body, const CapsuleDamage& damage);
    [[nodiscard]] DamageReport apply_sphere(Body& body, const SphereDamage& damage);
    [[nodiscard]] DamageReport apply_strain(Body& body, const StrainDamage& damage);
    [[nodiscard]] DamageReport apply(Body& body, const DamageCommand& command);

    [[nodiscard]] std::vector<DamageReport> replay(
        Body& body,
        const std::vector<DamageCommand>& commands);

    [[nodiscard]] const std::vector<DamageCommand>& history() const { return history_; }
    [[nodiscard]] const std::vector<WoundDescriptor>& wounds() const { return wounds_; }

    void clear_history();
    void clear_wounds();

private:
    [[nodiscard]] DamageEventId resolve_event_id(DamageEventId requested);

    MaterialTable materials_;
    DamageEventId next_event_id_{1};
    std::vector<DamageCommand> history_;
    std::vector<WoundDescriptor> wounds_;
};

} // namespace sarx
