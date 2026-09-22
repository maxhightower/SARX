#include "sarx/action_substitution.hpp"

#include <algorithm>
#include <cmath>

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
        phase >= 0.08
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
        execution.normalized_phase >= 0.08
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
        if (candidate.motion_id
            == current_action.motion_id) {
            continue;
        }

        const auto viability =
            evaluate_action_viability(
                candidate,
                anatomy);

        if (viability.state
            == MotionViability::Invalid) {
            continue;
        }

        ActionCandidateScore score;
        score.capability = candidate;
        score.viability = viability;

        score.intent_preservation =
            intent == BehavioralIntent::Attack
            ? 1.0
            : 0.0;

        score.anatomical_feasibility =
            viability.state == MotionViability::Viable
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

        plan.candidates.push_back(
            std::move(score));
    }

    const auto best =
        std::max_element(
            plan.candidates.begin(),
            plan.candidates.end(),
            [](const ActionCandidateScore& a,
               const ActionCandidateScore& b) {
                return a.total < b.total;
            });

    if (best == plan.candidates.end()) {
        plan.selected = ActionCapability{};
        plan.score = 0.0;
        return plan;
    }

    plan.selected = best->capability;
    plan.score = best->total;

    return plan;
}

} // namespace sarx
