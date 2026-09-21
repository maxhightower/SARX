#pragma once

#include "sarx/body.hpp"

#include <cstdint>
#include <vector>

namespace sarx {

struct ParticleSoA {
    std::vector<double> px, py, pz;
    std::vector<double> vx, vy, vz;
    std::vector<double> inverse_mass;
};

struct StructuralSoA {
    std::vector<ParticleId> a, b;
    std::vector<double> rest_length;
    std::vector<double> damage;
    std::vector<double> break_damage;
    std::vector<std::uint8_t> active;
    std::vector<MaterialId> material;
};

struct TetrahedralSoA {
    std::vector<ParticleId> a, b, c, d;
    std::vector<double> rest_volume;
    std::vector<double> damage;
    std::vector<double> break_damage;
    std::vector<std::uint8_t> active;
    std::vector<MaterialId> material;
};

struct BoneSoA {
    std::vector<BoneId> parent;
    std::vector<double> px, py, pz;
    std::vector<double> joint_damage;
    std::vector<double> joint_break_damage;
    std::vector<double> joint_radius;
    std::vector<std::uint8_t> joint_active;
    std::vector<MaterialId> joint_material;
};

struct AttachmentSoA {
    std::vector<ParticleId> particle;
    std::vector<BoneId> bone;
    std::vector<double> offset_x, offset_y, offset_z;
    std::vector<double> damage;
    std::vector<double> break_damage;
    std::vector<std::uint8_t> active;
    std::vector<MaterialId> material;
};

struct BodySoA {
    ParticleSoA particles;
    StructuralSoA structural;
    TetrahedralSoA tetrahedral;
    BoneSoA bones;
    AttachmentSoA attachments;
};

[[nodiscard]] BodySoA snapshot_body_soa(const Body& body);

} // namespace sarx
