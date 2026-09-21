#pragma once

#include "sarx/math.hpp"
#include "sarx/motion_viability.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sarx {

enum class BehavioralIntent {
    None,
    MoveForward,
    RecoverBalance,
    Stand,
    Attack,
    Reach,
    Turn,
    Climb
};

enum class MotionStrategy {
    ContinueCurrent,
    Fall,
    Kneel,
    Prone,
    GetUp,
    Limp,
    Hop,
    Crawl,
    Stop
};

struct MotionPhysicalState {
    Vec3 root_velocity{};
    bool grounded{true};
    bool airborne{false};
    std::size_t support_contacts{};
};

struct MotionCandidateScore {
    MotionStrategy strategy{MotionStrategy::Stop};
    std::string motion_id;
    bool procedural{false};
    double intent_preservation{};
    double physical_feasibility{};
    double transition_continuity{};
    double total{};
};

struct MotionRecoveryPlan {
    bool transition_required{false};
    MotionStrategy strategy{MotionStrategy::ContinueCurrent};
    std::string motion_id;
    bool procedural{false};
    double score{};
    MotionViabilityResult current_viability{};
    std::vector<MotionCandidateScore> candidates;
};

enum class GroundedPosture {
    Unknown,
    Prone,
    Kneeling,
    Standing
};

struct GroundedRecoveryPlan {
    GroundedPosture current_posture{GroundedPosture::Prone};
    GroundedPosture target_posture{GroundedPosture::Prone};
    bool transition_required{false};

    MotionStrategy strategy{MotionStrategy::Prone};
    std::string motion_id{"HoldProne"};
    bool procedural{true};
    double score{};

    std::vector<MotionCandidateScore> candidates;
    std::vector<MotionCandidateScore> followup_locomotion_options;
};

[[nodiscard]] GroundedRecoveryPlan plan_grounded_recovery(
    BehavioralIntent intent,
    GroundedPosture current_posture,
    const std::vector<std::string>& available_motions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalState& physical_state);

[[nodiscard]] MotionRecoveryPlan plan_authored_injury_locomotion(
    BehavioralIntent intent,
    const std::string& current_motion,
    const std::vector<std::string>& available_motions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalState& physical_state);

[[nodiscard]] MotionRecoveryPlan plan_motion_recovery(
    BehavioralIntent intent,
    const std::string& current_motion,
    const std::vector<std::string>& available_motions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalState& physical_state);

[[nodiscard]] const char* motion_strategy_name(
    MotionStrategy strategy);

} // namespace sarx
