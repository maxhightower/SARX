#include "sarx/action_capability.hpp"

#include <algorithm>
#include <cctype>

namespace sarx {

ActionCapability make_hand_punch_capability(
    const std::string& motion_id,
    ActionSide side) {

    ActionCapability capability;
    capability.motion_id = motion_id;
    capability.family = ActionFamily::Punch;
    capability.side = side;

    if (side == ActionSide::Left) {
        capability.effector =
            ActionEffector::LeftHand;

        capability.required_regions = {
            {"pelvis", 0.45},
            {"spine_01", 0.45},
            {"upperarm_l", 0.70},
            {"lowerarm_l", 0.65},
            {"hand_l", 0.60}
        };

        capability.optional_regions = {
            {"upperarm_r", 0.35},
            {"lowerarm_r", 0.35}
        };

        // A punch is not an isolated arm gesture. When it is selected as
        // a replacement action, its authored torso twist must come with it.
        // Physics-owned severed branches still override descendants.
        capability.authority_joint_roots = {
            "spine_01"
        };

        capability.contact_regions = {
            "hand_l"
        };
    } else {
        capability.side =
            ActionSide::Right;

        capability.effector =
            ActionEffector::RightHand;

        capability.required_regions = {
            {"pelvis", 0.45},
            {"spine_01", 0.45},
            {"upperarm_r", 0.70},
            {"lowerarm_r", 0.65},
            {"hand_r", 0.60}
        };

        capability.optional_regions = {
            {"upperarm_l", 0.35},
            {"lowerarm_l", 0.35}
        };

        // Opposite-hand punch substitution needs the authored torso
        // mechanics as well as the striking arm. Pelvis/legs remain base.
        capability.authority_joint_roots = {
            "spine_01"
        };

        capability.contact_regions = {
            "hand_r"
        };
    }

    return capability;
}

ActionCapability make_elbow_strike_capability(
    const std::string& motion_id,
    ActionSide side) {

    ActionCapability capability;
    capability.motion_id = motion_id;
    capability.family = ActionFamily::ElbowStrike;
    capability.side = side;

    if (side == ActionSide::Left) {
        capability.effector =
            ActionEffector::LeftElbow;

        capability.required_regions = {
            {"pelvis", 0.40},
            {"spine_01", 0.40},
            {"upperarm_l", 0.70},
            // The elbow is represented by the proximal lower-arm region.
            // A hand is intentionally not required.
            {"lowerarm_l", 0.15}
        };

        capability.authority_joint_roots = {
            "clavicle_l",
            "upperarm_l",
            "lowerarm_l"
        };

        capability.contact_regions = {
            "upperarm_l",
            "lowerarm_l"
        };
    } else {
        capability.side =
            ActionSide::Right;

        capability.effector =
            ActionEffector::RightElbow;

        capability.required_regions = {
            {"pelvis", 0.40},
            {"spine_01", 0.40},
            {"upperarm_r", 0.70},
            {"lowerarm_r", 0.15}
        };

        capability.authority_joint_roots = {
            "clavicle_r",
            "upperarm_r",
            "lowerarm_r"
        };

        capability.contact_regions = {
            "upperarm_r",
            "lowerarm_r"
        };
    }

    capability.contact_phase_begin = 0.55;
    capability.contact_phase_end = 0.85;

    return capability;
}

ActionCapability describe_authored_action(
    const std::string& motion_id) {

    std::string lower = motion_id;

    std::transform(
        lower.begin(),
        lower.end(),
        lower.begin(),
        [](unsigned char value) {
            return static_cast<char>(
                std::tolower(value));
        });

    if (lower.find("punch_jab")
        != std::string::npos) {

        auto capability =
            make_hand_punch_capability(
                motion_id,
                ActionSide::Left);

        // Measured from the real Quaternius asset by
        // sarx_character_action_audit at 60 Hz:
        // max left-hand extension is near normalized phase 0.327.
        capability.contact_phase_begin = 0.24;
        capability.contact_phase_end = 0.40;

        return capability;
    }

    if (lower.find("punch_cross")
        != std::string::npos) {

        auto capability =
            make_hand_punch_capability(
                motion_id,
                ActionSide::Right);

        // Measured max right-hand extension is near phase 0.267.
        capability.contact_phase_begin = 0.18;
        capability.contact_phase_end = 0.36;

        return capability;
    }

    if (lower.find("elbow")
        != std::string::npos) {

        const ActionSide side =
            lower.find("left")
                    != std::string::npos
                || lower.find("_l")
                    != std::string::npos
            ? ActionSide::Left
            : ActionSide::Right;

        return make_elbow_strike_capability(
            motion_id,
            side);
    }

    return ActionCapability{};
}

MotionViabilityResult evaluate_action_viability(
    const ActionCapability& capability,
    const std::vector<AnatomicalAvailability>& anatomy) {

    MotionCapability motion;
    motion.motion_id = capability.motion_id;
    motion.semantic_intent = capability.semantic_intent;
    motion.required_regions = capability.required_regions;
    motion.optional_regions = capability.optional_regions;

    return evaluate_motion_viability(
        motion,
        anatomy);
}

const char* action_family_name(
    ActionFamily family) {

    switch (family) {
    case ActionFamily::Punch:
        return "Punch";
    case ActionFamily::ElbowStrike:
        return "ElbowStrike";
    case ActionFamily::ForearmStrike:
        return "ForearmStrike";
    case ActionFamily::ShoulderStrike:
        return "ShoulderStrike";
    case ActionFamily::Kick:
        return "Kick";
    case ActionFamily::KneeStrike:
        return "KneeStrike";
    case ActionFamily::HeadStrike:
        return "HeadStrike";
    case ActionFamily::Unknown:
        break;
    }

    return "Unknown";
}

const char* action_effector_name(
    ActionEffector effector) {

    switch (effector) {
    case ActionEffector::LeftHand:
        return "LeftHand";
    case ActionEffector::RightHand:
        return "RightHand";
    case ActionEffector::LeftElbow:
        return "LeftElbow";
    case ActionEffector::RightElbow:
        return "RightElbow";
    case ActionEffector::LeftForearm:
        return "LeftForearm";
    case ActionEffector::RightForearm:
        return "RightForearm";
    case ActionEffector::LeftKnee:
        return "LeftKnee";
    case ActionEffector::RightKnee:
        return "RightKnee";
    case ActionEffector::LeftFoot:
        return "LeftFoot";
    case ActionEffector::RightFoot:
        return "RightFoot";
    case ActionEffector::Head:
        return "Head";
    case ActionEffector::Torso:
        return "Torso";
    case ActionEffector::Unknown:
        break;
    }

    return "Unknown";
}

} // namespace sarx
