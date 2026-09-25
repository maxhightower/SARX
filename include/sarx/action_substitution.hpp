#pragma once

#include "sarx/action_capability.hpp"
#include "sarx/motion_recovery.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sarx {

struct ActionExecutionState {
    std::string motion_id;
    double normalized_phase{};
    bool contact_already_occurred{false};
};

// Why a candidate was or was not chosen. Anatomy is evaluated first: a
// candidate whose anatomy fails is rejected regardless of animation
// availability, so the trace never claims an asset is the only blocker when
// the body cannot perform the action.
enum class ActionCandidateDisposition {
    Selected,
    ViableLowerPriority,
    RejectedCurrentAction,
    RejectedIntentMismatch,
    RejectedAnatomy,
    RejectedAuthoredMotionUnavailable
};

struct ActionCandidateScore {
    ActionCapability capability;
    MotionViabilityResult viability;
    ActionCandidateDisposition disposition{
        ActionCandidateDisposition::ViableLowerPriority};

    double intent_preservation{};
    double anatomical_feasibility{};
    double action_continuity{};
    double effector_appropriateness{};
    double total{};

    // Human-readable reason; failed regions are listed for anatomy rejections.
    std::string reason;
};

struct ActionSubstitutionPlan {
    bool transition_required{false};
    bool attack_already_complete{false};

    // Transition was required but no executable candidate survives. The
    // caller must not animate any strike (no phantom limb, no restored
    // anatomy); intent fails safely.
    bool intent_failed{false};

    MotionViabilityResult current_viability{};

    ActionCapability selected;
    double score{};

    // Executable, anatomically viable candidates (scored).
    std::vector<ActionCandidateScore> candidates;

    // Every candidate considered, including rejected ones, in input order.
    std::vector<ActionCandidateScore> trace;

    // Highest-scoring candidate that the surviving anatomy could perform but
    // which has no certified authored motion. Set only when it would have
    // outscored the selected candidate.
    bool preferred_blocked_on_authored_motion{false};
    ActionCandidateScore preferred_unavailable;
};

[[nodiscard]] ActionSubstitutionPlan
plan_action_substitution(
    BehavioralIntent intent,
    const ActionCapability& current_action,
    const std::vector<ActionCapability>& available_actions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const ActionExecutionState& execution);

[[nodiscard]] const char* action_candidate_disposition_name(
    ActionCandidateDisposition disposition);

[[nodiscard]] std::string describe_action_plan(
    const ActionSubstitutionPlan& plan);

// Re-evaluates the active action against current anatomy on every update so
// an injury during execution invalidates it, and a later injury can
// invalidate the substitute in turn. The controller never restores an action
// it has abandoned for anatomical reasons.
class ActionExecutionController {
public:
    enum class Status {
        Executing,
        IntentFailed,
        Complete
    };

    struct Transition {
        std::int64_t tick{};
        std::string from_motion;
        std::string to_motion;
        double from_phase{};
        ActionSubstitutionPlan plan;
    };

    ActionExecutionController(
        BehavioralIntent intent,
        ActionCapability initial_action,
        std::vector<ActionCapability> library);

    // Returns true if the active action changed or intent failed this tick.
    // `active_phase` is the normalized phase of the currently active action.
    bool update(
        std::int64_t tick,
        const std::vector<AnatomicalAvailability>& anatomy,
        double active_phase,
        bool contact_already_occurred = false);

    [[nodiscard]] const ActionCapability& active() const {
        return active_;
    }

    [[nodiscard]] Status status() const {
        return status_;
    }

    [[nodiscard]] std::int64_t active_since_tick() const {
        return active_since_tick_;
    }

    [[nodiscard]] const std::vector<Transition>& transitions() const {
        return transitions_;
    }

private:
    BehavioralIntent intent_;
    ActionCapability active_;
    std::vector<ActionCapability> library_;
    std::vector<std::string> abandoned_;
    std::vector<Transition> transitions_;
    Status status_{Status::Executing};
    std::int64_t active_since_tick_{};
};

} // namespace sarx
