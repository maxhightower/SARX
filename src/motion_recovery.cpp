#include "sarx/motion_recovery.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>

namespace sarx {
namespace {

std::string lower_copy(std::string value) {
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c) {
            return static_cast<char>(
                std::tolower(c));
        });
    return value;
}

const std::string* find_motion(
    const std::vector<std::string>& motions,
    const std::vector<std::string>& preferred_fragments) {

    for (const auto& fragment
         : preferred_fragments) {

        const std::string needle =
            lower_copy(fragment);

        for (const auto& motion
             : motions) {

            if (lower_copy(motion).find(needle)
                != std::string::npos) {
                return &motion;
            }
        }
    }

    return nullptr;
}

double speed_of(const Vec3& velocity) {
    return std::sqrt(
        velocity.x * velocity.x
        + velocity.y * velocity.y
        + velocity.z * velocity.z);
}

MotionCandidateScore score_fall_candidate(
    const std::string& motion_id,
    bool procedural,
    BehavioralIntent intent,
    const MotionPhysicalState& physical_state) {

    MotionCandidateScore candidate;
    candidate.strategy = MotionStrategy::Fall;
    candidate.motion_id = motion_id;
    candidate.procedural = procedural;

    candidate.intent_preservation =
        intent == BehavioralIntent::RecoverBalance
        ? 0.80
        : intent == BehavioralIntent::MoveForward
        ? 0.30
        : 0.55;

    candidate.physical_feasibility =
        physical_state.grounded
        ? 0.98
        : 0.88;

    const double speed =
        speed_of(
            physical_state.root_velocity);

    candidate.transition_continuity =
        std::clamp(
            0.84 + std::min(0.12, speed * 0.03),
            0.0,
            1.0);

    if (procedural) {
        // Procedural fall remains the deterministic fallback if the
        // animation library has no usable collapse-like clip.
        candidate.transition_continuity =
            std::min(
                1.0,
                candidate.transition_continuity + 0.03);
        candidate.physical_feasibility -= 0.04;
    }

    candidate.total =
        candidate.intent_preservation * 0.30
        + candidate.physical_feasibility * 0.50
        + candidate.transition_continuity * 0.20;

    return candidate;
}

} // namespace

MotionRecoveryPlan plan_motion_recovery(
    BehavioralIntent intent,
    const std::string& current_motion,
    const std::vector<std::string>& available_motions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalState& physical_state) {

    MotionRecoveryPlan plan;
    plan.motion_id = current_motion;
    plan.current_viability =
        evaluate_motion_viability(
            current_motion,
            anatomy);

    if (plan.current_viability.state
        != MotionViability::Invalid) {
        return plan;
    }

    plan.transition_required = true;

    // M2-1 deliberately begins with a small deterministic recovery set.
    // Prefer an explicitly named fall if one exists. The current free
    // Quaternius Standard library has no dedicated Fall clip, so Death01
    // is accepted as a collapse presentation alias. This does not change
    // behavioral intent to "die"; it is only the authored body-collapse
    // motion currently available to the renderer.
    if (const std::string* fall =
            find_motion(
                available_motions,
                {"Fall", "Death01", "Death"})) {

        const auto viability =
            evaluate_motion_viability(
                *fall,
                anatomy);

        if (viability.state
            != MotionViability::Invalid) {

            plan.candidates.push_back(
                score_fall_candidate(
                    *fall,
                    false,
                    intent,
                    physical_state));
        }
    }

    // A procedural fall is always available as the final deterministic
    // safety net. It is represented explicitly rather than silently
    // freezing or pretending the invalid clip remains authoritative.
    plan.candidates.push_back(
        score_fall_candidate(
            "ProceduralFall",
            true,
            intent,
            physical_state));

    const auto best =
        std::max_element(
            plan.candidates.begin(),
            plan.candidates.end(),
            [](const MotionCandidateScore& a,
               const MotionCandidateScore& b) {
                if (a.total != b.total) {
                    return a.total < b.total;
                }

                // Prefer authored evidence when scores tie.
                return a.procedural && !b.procedural;
            });

    if (best == plan.candidates.end()) {
        plan.strategy = MotionStrategy::Stop;
        plan.motion_id = current_motion;
        plan.procedural = false;
        plan.score = 0.0;
        return plan;
    }

    plan.strategy = best->strategy;
    plan.motion_id = best->motion_id;
    plan.procedural = best->procedural;
    plan.score = best->total;

    return plan;
}

const char* motion_strategy_name(
    MotionStrategy strategy) {

    switch (strategy) {
    case MotionStrategy::ContinueCurrent:
        return "ContinueCurrent";
    case MotionStrategy::Fall:
        return "Fall";
    case MotionStrategy::Kneel:
        return "Kneel";
    case MotionStrategy::Prone:
        return "Prone";
    case MotionStrategy::Hop:
        return "Hop";
    case MotionStrategy::Crawl:
        return "Crawl";
    case MotionStrategy::Stop:
        return "Stop";
    }

    return "Unknown";
}

} // namespace sarx
