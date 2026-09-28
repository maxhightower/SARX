#pragma once

// Anatomically proportioned humanoid for SARX consumers.
//
// Two representations of the same body, following SARX's rule that physical
// resolution and visual resolution are separate:
//
//   * a physical lattice Body (default 5 cm spacing, ~70 kg, same 17-bone
//     topology and HumanoidBones struct as the reference fixture), and
//   * a fine detail voxel set (default 2 cm) with anatomical materials
//     (soft tissue, bone, neural tissue) for rendering via EmbeddedVoxelSkin.
//
// Rest pose: ~1.78 m tall, facing +Z, arms reaching forward.

#include "sarx/character_runtime.hpp"
#include "sarx/humanoid.hpp"

#include <cstdint>
#include <vector>

namespace sarx {

struct AnatomicalHumanoidSpec {
    double spacing{0.05};
    // Particle mass = density * spacing^3.
    double density{985.0};
    // Extra radius added to every primitive for the physical lattice so thin
    // limbs stay connected at the chosen spacing.
    double lattice_padding{0.012};

    MaterialId tissue_material{1};
    double structural_compliance{2e-6};
    double structural_break_damage{1.0};
    double volume_compliance{5e-7};
    double volume_break_damage{1.0};
    double attachment_compliance{1e-7};
    double attachment_break_damage{1.0};
    double joint_break_damage{1.0};
    double joint_radius{0.035};
    // Particles whose nearest two skeletal segments belong to adjacent bones
    // and are within this distance of each other get a second, blended
    // attachment (skinning-style), so bent joints deform smoothly instead of
    // shearing the lattice. 0 disables blending.
    double joint_blend_width{0.05};
};

enum class DetailTissue : std::uint8_t {
    Soft,
    Bone,
    Neural
};

struct DetailVoxel {
    Vec3 local{};  // rest position in fixture space (before world offset)
    int gx{};
    int gy{};
    int gz{};
    DetailTissue tissue{DetailTissue::Soft};
};

struct DetailVoxelSet {
    double voxel_size{0.02};
    std::vector<DetailVoxel> voxels;
};

// Rest-space landmarks consumers may need (cuts, brain placement).
struct AnatomicalLandmarks {
    Vec3 head_center{0.0, 1.69, 0.03};
    Vec3 brain_center{0.0, 1.70, 0.02};
    double brain_radius{0.065};
    Vec3 neck_cut_center{0.0, 1.535, 0.01};
    Vec3 right_shoulder_cut_center{0.2075, 1.43, 0.09};
    Vec3 right_shoulder_cut_normal{0.1, -0.14, 1.0};
    double shoulder_cut_radius{0.085};
    double thigh_cut_radius{0.11};
    double neck_cut_radius{0.10};
};

[[nodiscard]] const AnatomicalLandmarks& anatomical_landmarks();

[[nodiscard]] HumanoidFixture build_anatomical_humanoid_fixture(
    const AnatomicalHumanoidSpec& spec = {});

[[nodiscard]] DetailVoxelSet build_anatomical_detail_voxels(double voxel_size = 0.02);

[[nodiscard]] HumanoidRuntime build_anatomical_humanoid_runtime(
    const AnatomicalHumanoidSpec& spec = {},
    const Vec3& world_offset = {},
    const RuntimeConfig& config = {});

} // namespace sarx
