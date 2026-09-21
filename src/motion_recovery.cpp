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

MotionCandidateScore score_grounded_candidate(
    MotionStrategy strategy,
    const std::string& motion_id,
    bool procedural,
    BehavioralIntent intent,
    double physical_feasibility,
    double transition_continuity) {

    MotionCandidateScore candidate;
    candidate.strategy = strategy;
    candidate.motion_id = motion_id;
    candidate.procedural = procedural;

    switch (strategy) {
    case MotionStrategy::Prone:
        candidate.intent_preservation =
            intent == BehavioralIntent::RecoverBalance
            ? 0.35
            : intent == BehavioralIntent::MoveForward
            ? 0.15
            : 0.30;
        break;

    case MotionStrategy::Kneel:
        candidate.intent_preservation =
            intent == BehavioralIntent::RecoverBalance
            ? 0.90
            : intent == BehavioralIntent::Stand
            ? 0.78
            : intent == BehavioralIntent::MoveForward
            ? 0.45
            : 0.62;
        break;

    case MotionStrategy::GetUp:
        candidate.intent_preservation =
            intent == BehavioralIntent::Stand
            ? 1.0
            : intent == BehavioralIntent::MoveForward
            ? 0.90
            : 0.72;
        break;

    case MotionStrategy::Crawl:
        candidate.intent_preservation =
            intent == BehavioralIntent::MoveForward
            ? 0.90
            : 0.48;
        break;

    case MotionStrategy::Hop:
        candidate.intent_preservation =
            intent == BehavioralIntent::MoveForward
            ? 0.88
            : 0.42;
        break;

    default:
        candidate.intent_preservation = 0.40;
        break;
    }

    candidate.physical_feasibility =
        physical_feasibility;

    candidate.transition_continuity =
        transition_continuity;

    candidate.total =
        candidate.intent_preservation * 0.30
        + candidate.physical_feasibility * 0.50
        + candidate.transition_continuity * 0.20;

    return candidate;
}

bool viability_allows(
    const std::string& motion_id,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalState& physical_state) {

    MotionPhysicalContext context;
    context.has_support_contacts = true;
    context.support_contacts =
        physical_state.support_contacts;
    context.has_grounded_state = true;
    context.grounded =
        physical_state.grounded;

    return evaluate_motion_viability(
        motion_id,
        anatomy,
        context).state
        != MotionViability::Invalid;
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

GroundedRecoveryPlan plan_grounded_recovery(
    BehavioralIntent intent,
    GroundedPosture current_posture,
    const std::vector<std::string>& available_motions,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalState& physical_state) {

    GroundedRecoveryPlan plan;
    plan.current_posture = current_posture;

    // M2-C only chooses post-fall recovery posture. Intent-preserving
    // locomotion alternatives are reported separately for M2-D so a
    // recovery-state planner cannot silently turn into a locomotion
    // controller.
    plan.candidates.push_back(
        score_grounded_candidate(
            MotionStrategy::Prone,
            "HoldProne",
            true,
            intent,
            physical_state.grounded ? 1.0 : 0.35,
            1.0));

    if (physical_state.grounded) {
        if (const std::string* kneel =
                find_motion(
                    available_motions,
                    {"Kneel"})) {

            if (viability_allows(
                    *kneel,
                    anatomy,
                    physical_state)) {

                plan.candidates.push_back(
                    score_grounded_candidate(
                        MotionStrategy::Kneel,
                        *kneel,
                        false,
                        intent,
                        0.94,
                        0.82));
            }
        }

        if (const std::string* get_up =
                find_motion(
                    available_motions,
                    {
                        "GetUp",
                        "Get_Up",
                        "Stand_Up",
                        "StandUp"
                    })) {

            if (viability_allows(
                    *get_up,
                    anatomy,
                    physical_state)) {

                plan.candidates.push_back(
                    score_grounded_candidate(
                        MotionStrategy::GetUp,
                        *get_up,
                        false,
                        intent,
                        0.90,
                        0.72));
            }
        }

        if (viability_allows(
                "ProceduralCrawl",
                anatomy,
                physical_state)) {

            plan.followup_locomotion_options.push_back(
                score_grounded_candidate(
                    MotionStrategy::Crawl,
                    "ProceduralCrawl",
                    true,
                    intent,
                    0.82,
                    0.55));
        }

        if (viability_allows(
                "ProceduralHop",
                anatomy,
                physical_state)) {

            plan.followup_locomotion_options.push_back(
                score_grounded_candidate(
                    MotionStrategy::Hop,
                    "ProceduralHop",
                    true,
                    intent,
                    0.78,
                    0.50));
        }
    }

    const auto best =
        std::max_element(
            plan.candidates.begin(),
            plan.candidates.end(),
            [](const MotionCandidateScore& a,
               const MotionCandidateScore& b) {

                if (a.total != b.total) {
                    return a.total < b.total;
                }

                return a.procedural && !b.procedural;
            });

    if (best == plan.candidates.end()) {
        return plan;
    }

    plan.strategy = best->strategy;
    plan.motion_id = best->motion_id;
    plan.procedural = best->procedural;
    plan.score = best->total;

    switch (plan.strategy) {
    case MotionStrategy::Kneel:
        plan.target_posture =
            GroundedPosture::Kneeling;
        break;
    case MotionStrategy::GetUp:
        plan.target_posture =
            GroundedPosture::Standing;
        break;
    default:
        plan.target_posture =
            GroundedPosture::Prone;
        break;
    }

    plan.transition_required =
        plan.target_posture
        != current_posture;

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
    case MotionStrategy::GetUp:
        return "GetUp";
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
