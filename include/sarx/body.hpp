#pragma once

#include "sarx/math.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace sarx {

using ParticleId = std::size_t;
using BoneId = std::size_t;
using ConstraintId = std::size_t;
using MaterialId = std::uint32_t;

inline constexpr BoneId kNoParent = std::numeric_limits<BoneId>::max();
inline constexpr MaterialId kDefaultMaterial = 0;

struct Particle {
    Vec3 position{};
    Vec3 velocity{};
    double inverse_mass{1.0};
};

struct Bone {
    BoneId parent{kNoParent};
    Vec3 animated_position{};
    bool joint_to_parent_active{true};
    double joint_damage{0.0};
    double joint_break_damage{1.0};
    MaterialId joint_material{kDefaultMaterial};
};

struct StructuralConstraint {
    ParticleId a{};
    ParticleId b{};
    double rest_length{};
    double compliance{0.0};
    double damage{0.0};
    double break_damage{1.0};
    double lambda{0.0};
    bool active{true};
    MaterialId material{kDefaultMaterial};
};

struct AttachmentConstraint {
    ParticleId particle{};
    BoneId bone{};
    Vec3 local_offset{};
    double compliance{0.0};
    double damage{0.0};
    double break_damage{1.0};
    Vec3 lambda{};
    bool active{true};
    MaterialId material{kDefaultMaterial};
};

struct Island {
    std::vector<ParticleId> particles;
    bool rig_authoritative{false};
    double mass{0.0};
    Vec3 linear_momentum{};
};

struct StepConfig {
    int substeps{2};
    int solver_iterations{8};
    Vec3 gravity{0.0, -9.81, 0.0};
};

class Body {
public:
    ParticleId add_particle(const Vec3& position, double mass = 1.0);

    BoneId add_bone(
        BoneId parent,
        const Vec3& animated_position,
        double joint_break_damage = 1.0,
        MaterialId joint_material = kDefaultMaterial);

    ConstraintId add_structural_constraint(
        ParticleId a,
        ParticleId b,
        double compliance = 0.0,
        double break_damage = 1.0,
        MaterialId material = kDefaultMaterial);

    ConstraintId add_attachment(
        ParticleId particle,
        BoneId bone,
        const Vec3& local_offset = {},
        double compliance = 1e-7,
        double break_damage = 1.0,
        MaterialId material = kDefaultMaterial);

    void set_bone_target(BoneId bone, const Vec3& animated_position);

    void damage_structural(ConstraintId constraint, double amount);
    void damage_attachment(ConstraintId constraint, double amount);
    void damage_bone_joint(BoneId bone, double amount);

    void break_structural(ConstraintId constraint);
    void break_attachment(ConstraintId constraint);
    void break_bone_joint(BoneId bone);

    [[nodiscard]] bool bone_root_connected(BoneId bone) const;
    [[nodiscard]] std::vector<Island> islands() const;
    [[nodiscard]] Vec3 total_linear_momentum() const;

    void step(double dt, const StepConfig& config = {});

    [[nodiscard]] const std::vector<Particle>& particles() const { return particles_; }
    [[nodiscard]] std::vector<Particle>& particles() { return particles_; }
    [[nodiscard]] const std::vector<Bone>& bones() const { return bones_; }
    [[nodiscard]] const std::vector<StructuralConstraint>& structural_constraints() const { return structural_; }
    [[nodiscard]] const std::vector<AttachmentConstraint>& attachments() const { return attachments_; }

private:
    void solve_structural(double h);
    void solve_attachments(double h);

    std::vector<Particle> particles_;
    std::vector<Bone> bones_;
    std::vector<StructuralConstraint> structural_;
    std::vector<AttachmentConstraint> attachments_;
};

} // namespace sarx
