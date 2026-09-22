#pragma once

#include "sarx/action_capability.hpp"
#include "sarx/motion_recovery.hpp"

#include <string>
#include <vector>

namespace sarx {

struct ActionExecutionState {
    std::string motion_id;
    double normalized_phase{};
    bool contact_already_occurred{false};
};

struct ActionCandidateScore {
    ActionCapability capability;
    MotionViabilityResult viability;

    double intent_preservation{};
    double anatomical_feasibility{};
    double action_continuity{};
    double effector_appropriateness{};
    double total{};
};

struct ActionSubstitutionPlan {
    bool transition_required{false};
    bool attack_already_complete{false};

    MotionViabilityResult current_viability{};

    ActionCapability selected;
    double score{};

    std::vector<ActionCandidateScore> candidates;
};

[[nodiscard]] ActionSubstitutionPlan
plan_action_substitution(
    BehavioralIntent intent,
    const ActionCapability& current_action,
    const std::vector<ActionCapability>& available_actions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const ActionExecutionState& execution);

} // namespace sarx
