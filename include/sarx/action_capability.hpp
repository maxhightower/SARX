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

    double contact_phase_begin{0.65};
    double contact_phase_end{0.90};
};

[[nodiscard]] ActionCapability make_hand_punch_capability(
    const std::string& motion_id,
    ActionSide side);

[[nodiscard]] ActionCapability make_elbow_strike_capability(
    const std::string& motion_id,
    ActionSide side);

[[nodiscard]] MotionViabilityResult evaluate_action_viability(
    const ActionCapability& capability,
    const std::vector<AnatomicalAvailability>& anatomy);

[[nodiscard]] const char* action_family_name(
    ActionFamily family);

[[nodiscard]] const char* action_effector_name(
    ActionEffector effector);

} // namespace sarx
