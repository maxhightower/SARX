#pragma once

#include "sarx/body.hpp"

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace sarx {

enum class DamageMode {
    Cut,
    Blunt
};

enum class DamageTargetKind {
    StructuralConstraint,
    AttachmentConstraint,
    BoneJoint
};

struct MaterialResponse {
    double cut_resistance{1.0};
    double blunt_resistance{1.0};
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
};

struct SphereDamage {
    Vec3 center{};
    double radius{0.1};
    double energy{1.0};
    DamageMode mode{DamageMode::Blunt};
};

struct FractureEvent {
    DamageTargetKind target_kind{DamageTargetKind::StructuralConstraint};
    std::size_t target_id{};
    MaterialId material{kDefaultMaterial};
    Vec3 position{};
    double applied_damage{};
    bool broke{false};
};

struct DamageReport {
    std::vector<FractureEvent> events;

    [[nodiscard]] std::size_t broken_count() const;
};

class DamageSystem {
public:
    MaterialTable& materials() { return materials_; }
    [[nodiscard]] const MaterialTable& materials() const { return materials_; }

    [[nodiscard]] DamageReport apply_capsule(Body& body, const CapsuleDamage& damage) const;
    [[nodiscard]] DamageReport apply_sphere(Body& body, const SphereDamage& damage) const;

private:
    MaterialTable materials_;
};

} // namespace sarx
