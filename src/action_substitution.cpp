#include "sarx/action_substitution.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

namespace sarx {
namespace {

bool is_opposite_side(
    ActionSide a,
    ActionSide b) {

    return (a == ActionSide::Left
            && b == ActionSide::Right)
        || (a == ActionSide::Right
            && b == ActionSide::Left);
}

double continuity_score(
    const ActionCapability& current,
    const ActionCapability& candidate,
    const ActionExecutionState& execution) {

    const double phase =
        std::clamp(
            execution.normalized_phase,
            0.0,
            1.0);

    const bool committed =
        phase >= current.commitment_phase
        && !execution.contact_already_occurred;

    if (committed
        && current.family == ActionFamily::Punch
        && candidate.family == ActionFamily::ElbowStrike
        && current.side == candidate.side) {

        // Preserve the already-committed shoulder/torso trajectory.
        return 1.0;
    }

    if (!committed
        && candidate.family == ActionFamily::Punch
        && is_opposite_side(
            current.side,
            candidate.side)) {

        // Before commitment, changing to the healthy hand is cheapest.
        return 1.0;
    }

    if (candidate.family == current.family
        && candidate.side == current.side) {
        return 0.90;
    }

    if (candidate.family == ActionFamily::Punch
        && is_opposite_side(
            current.side,
            candidate.side)) {
        return committed ? 0.70 : 0.95;
    }

    if (candidate.family == ActionFamily::ElbowStrike
        && candidate.side == current.side) {
        return committed ? 0.98 : 0.72;
    }

    return 0.45;
}

double effector_score(
    const ActionCapability& current,
    const ActionCapability& candidate,
    const ActionExecutionState& execution) {

    const bool committed =
        execution.normalized_phase >= current.commitment_phase
        && !execution.contact_already_occurred;

    if (candidate.family == ActionFamily::ElbowStrike
        && candidate.side == current.side) {
        return committed ? 1.0 : 0.78;
    }

    if (candidate.family == ActionFamily::Punch
        && is_opposite_side(
            current.side,
            candidate.side)) {
        return committed ? 0.88 : 1.0;
    }

    if (candidate.side == current.side) {
        return 0.82;
    }

    return 0.70;
}

std::string joined(
    const std::vector<std::string>& values) {

    std::string out;

    for (const auto& value : values) {
        if (!out.empty()) {
            out += ",";
        }
        out += value;
    }

    return out;
}

} // namespace

ActionSubstitutionPlan
plan_action_substitution(
    BehavioralIntent intent,
    const ActionCapability& current_action,
    const std::vector<ActionCapability>& available_actions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const ActionExecutionState& execution) {

    ActionSubstitutionPlan plan;
    plan.selected = current_action;

    plan.current_viability =
        evaluate_action_viability(
            current_action,
            anatomy);

    if (execution.contact_already_occurred) {
        // The requested attack has already delivered its authoritative hit.
        // Damage after contact should not invent a second strike.
        plan.attack_already_complete = true;
        return plan;
    }

    if (plan.current_viability.state
        != MotionViability::Invalid) {
        return plan;
    }

    plan.transition_required = true;

    for (const auto& candidate : available_actions) {
        ActionCandidateScore score;
        score.capability = candidate;

        if (candidate.motion_id
            == current_action.motion_id) {
            score.disposition =
                ActionCandidateDisposition::RejectedCurrentAction;
            score.reason = "invalidated current action";
            plan.trace.push_back(std::move(score));
            continue;
        }

        if (candidate.semantic_intent
            != current_action.semantic_intent) {
            score.disposition =
                ActionCandidateDisposition::RejectedIntentMismatch;
            score.reason =
                "intent " + candidate.semantic_intent
                + " does not preserve " + current_action.semantic_intent;
            plan.trace.push_back(std::move(score));
            continue;
        }

        score.viability =
            evaluate_action_viability(
                candidate,
                anatomy);

        if (score.viability.state
            == MotionViability::Invalid) {
            score.disposition =
                ActionCandidateDisposition::RejectedAnatomy;
            score.reason =
                "required anatomy unavailable: "
                + joined(score.viability.failed_regions);
            plan.trace.push_back(std::move(score));
            continue;
        }

        score.intent_preservation =
            intent == BehavioralIntent::Attack
            ? 1.0
            : 0.0;

        score.anatomical_feasibility =
            score.viability.state == MotionViability::Viable
            ? 1.0
            : 0.75;

        score.action_continuity =
            continuity_score(
                current_action,
                candidate,
                execution);

        score.effector_appropriateness =
            effector_score(
                current_action,
                candidate,
                execution);

        score.total =
            score.intent_preservation * 0.30
            + score.anatomical_feasibility * 0.35
            + score.action_continuity * 0.20
            + score.effector_appropriateness * 0.15;

        if (!candidate.authored_motion_available) {
            score.disposition =
                ActionCandidateDisposition::RejectedAuthoredMotionUnavailable;
            score.reason =
                "anatomy viable; "
                + (candidate.availability_note.empty()
                    ? std::string("no authored motion bound")
                    : candidate.availability_note);
            plan.trace.push_back(std::move(score));
            continue;
        }

        score.disposition =
            ActionCandidateDisposition::ViableLowerPriority;
        score.reason = "viable";

        plan.trace.push_back(score);
        plan.candidates.push_back(std::move(score));
    }

    const auto best =
        std::max_element(
            plan.candidates.begin(),
            plan.candidates.end(),
            [](const ActionCandidateScore& a,
               const ActionCandidateScore& b) {
                return a.total < b.total;
            });

    const ActionCandidateScore* best_unavailable = nullptr;

    for (const auto& entry : plan.trace) {
        if (entry.disposition
                == ActionCandidateDisposition::RejectedAuthoredMotionUnavailable
            && (!best_unavailable
                || entry.total > best_unavailable->total)) {
            best_unavailable = &entry;
        }
    }

    if (best_unavailable
        && (best == plan.candidates.end()
            || best_unavailable->total > best->total)) {
        plan.preferred_blocked_on_authored_motion = true;
        plan.preferred_unavailable = *best_unavailable;
    }

    if (best == plan.candidates.end()) {
        plan.selected = ActionCapability{};
        plan.score = 0.0;
        plan.intent_failed = true;
        return plan;
    }

    plan.selected = best->capability;
    plan.score = best->total;

    for (auto& entry : plan.trace) {
        if (entry.disposition
                == ActionCandidateDisposition::ViableLowerPriority
            && entry.capability.motion_id
                == plan.selected.motion_id) {
            entry.disposition =
                ActionCandidateDisposition::Selected;
            entry.reason = "selected";
        }
    }

    return plan;
}

const char* action_candidate_disposition_name(
    ActionCandidateDisposition disposition) {

    switch (disposition) {
    case ActionCandidateDisposition::Selected:
        return "Selected";
    case ActionCandidateDisposition::ViableLowerPriority:
        return "ViableLowerPriority";
    case ActionCandidateDisposition::RejectedCurrentAction:
        return "RejectedCurrentAction";
    case ActionCandidateDisposition::RejectedIntentMismatch:
        return "RejectedIntentMismatch";
    case ActionCandidateDisposition::RejectedAnatomy:
        return "RejectedAnatomy";
    case ActionCandidateDisposition::RejectedAuthoredMotionUnavailable:
        return "RejectedAuthoredMotionUnavailable";
    }

    return "Unknown";
}

std::string describe_action_plan(
    const ActionSubstitutionPlan& plan) {

    std::ostringstream out;

    out << "transition_required=" << (plan.transition_required ? 1 : 0)
        << " intent_failed=" << (plan.intent_failed ? 1 : 0)
        << " selected="
        << (plan.selected.motion_id.empty()
            ? std::string("<none>")
            : plan.selected.motion_id)
        << " current_failed_regions="
        << joined(plan.current_viability.failed_regions);

    for (const auto& entry : plan.trace) {
        out << " | " << entry.capability.motion_id
            << " [" << action_family_name(entry.capability.family)
            << "/" << action_effector_name(entry.capability.effector)
            << "] " << action_candidate_disposition_name(entry.disposition)
            << " total=" << entry.total
            << " (" << entry.reason << ")";
    }

    if (plan.preferred_blocked_on_authored_motion) {
        out << " | preferred_blocked_on_authored_motion="
            << plan.preferred_unavailable.capability.motion_id;
    }

    return out.str();
}

ActionExecutionController::ActionExecutionController(
    BehavioralIntent intent,
    ActionCapability initial_action,
    std::vector<ActionCapability> library)
    : intent_(intent),
      active_(std::move(initial_action)),
      library_(std::move(library)) {}

bool ActionExecutionController::update(
    std::int64_t tick,
    const std::vector<AnatomicalAvailability>& anatomy,
    double active_phase,
    bool contact_already_occurred) {

    if (status_ != Status::Executing) {
        return false;
    }

    ActionExecutionState execution;
    execution.motion_id = active_.motion_id;
    execution.normalized_phase = active_phase;
    execution.contact_already_occurred = contact_already_occurred;

    std::vector<ActionCapability> candidates;
    candidates.reserve(library_.size());

    for (const auto& capability : library_) {
        if (std::find(
                abandoned_.begin(),
                abandoned_.end(),
                capability.motion_id)
            == abandoned_.end()) {
            candidates.push_back(capability);
        }
    }

    auto plan =
        plan_action_substitution(
            intent_,
            active_,
            candidates,
            anatomy,
            execution);

    if (plan.attack_already_complete) {
        status_ = Status::Complete;
        return false;
    }

    if (!plan.transition_required) {
        return false;
    }

    Transition transition;
    transition.tick = tick;
    transition.from_motion = active_.motion_id;
    transition.from_phase = active_phase;
    transition.to_motion = plan.selected.motion_id;

    abandoned_.push_back(active_.motion_id);

    if (plan.intent_failed) {
        status_ = Status::IntentFailed;
        active_ = ActionCapability{};
    } else {
        active_ = plan.selected;
    }

    active_since_tick_ = tick;
    transition.plan = std::move(plan);
    transitions_.push_back(std::move(transition));

    return true;
}

} // namespace sarx
