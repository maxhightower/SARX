#pragma once

#include "sarx/voxel_character.hpp"

#include <string>
#include <vector>

namespace sarx {

enum class MotionViability {
    Viable,
    Degraded,
    Invalid
};

struct MotionViabilityResult {
    MotionViability state{MotionViability::Viable};
    std::vector<std::string> failed_regions;
};

[[nodiscard]] MotionViabilityResult
evaluate_motion_viability(
    const std::string& animation_name,
    const std::vector<AnatomicalAvailability>& anatomy);

} // namespace sarx
