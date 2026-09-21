#pragma once

#include "sarx/voxel_character.hpp"

#include <string>
#include <vector>

namespace sarx {

enum class ProceduralLimpVariant {
    Guarded,
    StiffLeg,
    HopStep
};

struct ProceduralLimpInput {
    double cycle_phase{};
    double voxel_size{0.045};

    Vec3 injured_hip{};
    Vec3 injured_knee{};
    Vec3 injured_ankle{};
};

struct ProceduralLimpMetrics {
    double body_shift{};
    double body_lift{};
    double injured_stride_scale{};
    double knee_flexion_radians{};
};

[[nodiscard]] ProceduralLimpVariant
parse_procedural_limp_variant(
    const std::string& name);

[[nodiscard]] const char*
procedural_limp_variant_name(
    ProceduralLimpVariant variant);

[[nodiscard]] ProceduralLimpMetrics
apply_procedural_limp(
    ProceduralLimpVariant variant,
    const ProceduralLimpInput& input,
    const std::vector<CharacterVoxel>& voxels,
    const std::vector<Vec3>& neutral_centers,
    std::vector<Vec3>& animated_centers);

} // namespace sarx
