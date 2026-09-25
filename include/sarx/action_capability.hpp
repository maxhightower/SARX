#pragma once

#include "sarx/motion_viability.hpp"

#include <string>
#include <vector>

namespace sarx {

enum class ActionFamily {
    Unknown,
    Punch,
    ElbowStrike,
    ForearmStrike,
    ShoulderStrike,
    Kick,
    KneeStrike,
    HeadStrike
};

enum class ActionSide {
    None,
    Left,
    Right
};

enum class ActionEffector {
    Unknown,
    LeftHand,
    RightHand,
    LeftElbow,
    RightElbow,
    LeftForearm,
    RightForearm,
    LeftKnee,
    RightKnee,
    LeftFoot,
    RightFoot,
    Head,
    Torso
};

struct ActionCapability {
    std::string motion_id;
    std::string semantic_intent{"attack"};

    ActionFamily family{ActionFamily::Unknown};
    ActionSide side{ActionSide::None};
    ActionEffector effector{ActionEffector::Unknown};

    std::vector<AnatomicalRequirement>
        required_regions;

    std::vector<AnatomicalRequirement>
        optional_regions;

    // Skeletal roots whose authored channels may replace the base action.
    std::vector<std::string>
        authority_joint_roots;

    // Anatomical regions allowed to make the authoritative hit.
    std::vector<std::string>
        contact_regions;

    // Skeletal joint whose world position is the striking point
    // (hand joint for a punch, elbow/lower-arm origin for an elbow).
    std::string effector_joint;

    double contact_phase_begin{0.65};
    double contact_phase_end{0.90};

    // Normalized phase after which the action is committed: the body has
    // started delivering this strike, so a same-effector-chain continuation
    // is preferred over restarting with another limb.
    double commitment_phase{0.08};

    // Whether a certified authored clip is bound to this capability.
    // A capability may be semantically and anatomically valid while still
    // being unexecutable because no acceptable authored motion exists.
    bool authored_motion_available{true};
    std::string availability_note;
};

[[nodiscard]] ActionCapability make_hand_punch_capability(
    const std::string& motion_id,
    ActionSide side);

[[nodiscard]] ActionCapability make_elbow_strike_capability(
    const std::string& motion_id,
    ActionSide side);

[[nodiscard]] ActionCapability describe_authored_action(
    const std::string& motion_id);

// Certified attack library for the real Quaternius integration character.
// Punch_Jab / Punch_Cross bind to audited Quaternius clips. Elbow slots are
// semantic capabilities with no certified authored motion yet; binding a
// real clip later is a data change (motion_id + availability), not a
// planner change.
[[nodiscard]] std::vector<ActionCapability>
quaternius_attack_action_library();

// Bind a certified authored clip to an existing semantic capability slot.
[[nodiscard]] ActionCapability bind_authored_motion(
    ActionCapability capability,
    const std::string& motion_id);

[[nodiscard]] MotionViabilityResult evaluate_action_viability(
    const ActionCapability& capability,
    const std::vector<AnatomicalAvailability>& anatomy);

[[nodiscard]] const char* action_family_name(
    ActionFamily family);

[[nodiscard]] const char* action_effector_name(
    ActionEffector effector);

} // namespace sarx
