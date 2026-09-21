#pragma once

#include "sarx/voxel_character.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sarx {

enum class MotionViability {
    Viable,
    Degraded,
    Invalid
};

enum class MotionLocomotionType {
    Other,
    BipedalLocomotion,
    Collapse,
    Kneeling,
    InjuryLocomotion,
    Hop,
    Crawl
};

struct AnatomicalRequirement {
    std::string region;
    double minimum_attached_fraction{1.0};
};

struct AnatomicalSubstitution {
    std::string label;
    std::vector<std::string> candidate_regions;
    double minimum_attached_fraction{1.0};
    std::size_t minimum_count{1};
};

struct AnatomicalAlternativeChain {
    std::string label;
    std::vector<std::vector<AnatomicalRequirement>>
        alternatives;
    std::size_t minimum_complete_alternatives{1};
};

struct MotionCapability {
    std::string motion_id;
    std::string semantic_intent;
    MotionLocomotionType locomotion_type{
        MotionLocomotionType::Other};

    std::vector<AnatomicalRequirement>
        required_regions;

    std::vector<AnatomicalRequirement>
        optional_regions;

    std::vector<AnatomicalSubstitution>
        allowed_substitutions;

    std::vector<AnatomicalAlternativeChain>
        alternative_chains;

    std::size_t minimum_support_contacts{};
    bool requires_grounded{false};
    bool allows_airborne{true};
};

struct MotionPhysicalContext {
    bool has_support_contacts{false};
    std::size_t support_contacts{};

    bool has_grounded_state{false};
    bool grounded{true};
};

struct MotionViabilityResult {
    MotionViability state{MotionViability::Viable};
    std::vector<std::string> failed_regions;
    std::vector<std::string> failed_requirements;
    std::vector<std::string> usable_substitutions;
    std::size_t missing_support_contacts{};
    double transition_urgency{};
};

[[nodiscard]] MotionCapability
describe_motion_capability(
    const std::string& animation_name);

[[nodiscard]] MotionViabilityResult
evaluate_motion_viability(
    const std::string& animation_name,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalContext& physical = {});

[[nodiscard]] MotionViabilityResult
evaluate_motion_viability(
    const MotionCapability& capability,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalContext& physical = {});

} // namespace sarx
