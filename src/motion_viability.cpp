#include "sarx/motion_viability.hpp"

#include <algorithm>
#include <cctype>
#include <string>
#include <unordered_map>

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

double fraction_for(
    const std::unordered_map<std::string, double>& fractions,
    const std::string& region) {

    const auto found =
        fractions.find(region);

    if (found == fractions.end()) {
        return 1.0;
    }

    return found->second;
}

bool contains_any(
    const std::string& name,
    const std::vector<std::string>& fragments) {

    for (const auto& fragment
         : fragments) {
        if (name.find(fragment)
            != std::string::npos) {
            return true;
        }
    }

    return false;
}

} // namespace

MotionCapability
describe_motion_capability(
    const std::string& animation_name) {

    MotionCapability capability;
    capability.motion_id = animation_name;

    const std::string name =
        lower_copy(animation_name);

    if (contains_any(
            name,
            {"walk", "jog", "run", "sprint"})) {

        capability.semantic_intent =
            "move";
        capability.locomotion_type =
            MotionLocomotionType::BipedalLocomotion;

        capability.required_regions = {
            {"thigh_l", 0.60},
            {"calf_l", 0.60},
            {"foot_l", 0.55},
            {"thigh_r", 0.60},
            {"calf_r", 0.60},
            {"foot_r", 0.55}
        };

        capability.optional_regions = {
            {"upperarm_l", 0.50},
            {"lowerarm_l", 0.50},
            {"upperarm_r", 0.50},
            {"lowerarm_r", 0.50}
        };

        capability.minimum_support_contacts = 1;
        capability.allows_airborne = false;
        return capability;
    }

    if (contains_any(
            name,
            {"fall", "death"})) {

        capability.semantic_intent =
            "fall";
        capability.locomotion_type =
            MotionLocomotionType::Collapse;

        capability.required_regions = {
            {"pelvis", 0.35},
            {"spine_01", 0.35}
        };

        capability.allows_airborne = true;
        return capability;
    }

    if (name.find("kneel")
        != std::string::npos) {

        capability.semantic_intent =
            "recover_balance";
        capability.locomotion_type =
            MotionLocomotionType::Kneeling;

        capability.required_regions = {
            {"pelvis", 0.50},
            {"spine_01", 0.45}
        };

        capability.allowed_substitutions = {
            {
                "usable_kneeling_leg",
                {"thigh_l", "thigh_r"},
                0.60,
                1
            }
        };

        capability.requires_grounded = true;
        capability.minimum_support_contacts = 1;
        return capability;
    }

    if (name.find("hop")
        != std::string::npos) {

        capability.semantic_intent =
            "move";
        capability.locomotion_type =
            MotionLocomotionType::Hop;

        capability.required_regions = {
            {"pelvis", 0.55},
            {"spine_01", 0.45}
        };

        capability.allowed_substitutions = {
            {
                "usable_hop_thigh",
                {"thigh_l", "thigh_r"},
                0.70,
                1
            },
            {
                "usable_hop_calf",
                {"calf_l", "calf_r"},
                0.70,
                1
            },
            {
                "usable_hop_foot",
                {"foot_l", "foot_r"},
                0.65,
                1
            }
        };

        capability.minimum_support_contacts = 1;
        return capability;
    }

    if (name.find("crawl")
        != std::string::npos) {

        capability.semantic_intent =
            "move";
        capability.locomotion_type =
            MotionLocomotionType::Crawl;

        capability.required_regions = {
            {"pelvis", 0.45},
            {"spine_01", 0.45}
        };

        capability.allowed_substitutions = {
            {
                "usable_crawl_upper_limb",
                {
                    "upperarm_l",
                    "upperarm_r"
                },
                0.55,
                1
            },
            {
                "usable_crawl_lower_limb",
                {
                    "lowerarm_l",
                    "lowerarm_r"
                },
                0.55,
                1
            }
        };

        capability.requires_grounded = true;
        capability.minimum_support_contacts = 1;
        return capability;
    }

    capability.semantic_intent =
        "unspecified";

    return capability;
}

MotionViabilityResult
evaluate_motion_viability(
    const std::string& animation_name,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalContext& physical) {

    return evaluate_motion_viability(
        describe_motion_capability(
            animation_name),
        anatomy,
        physical);
}

MotionViabilityResult
evaluate_motion_viability(
    const MotionCapability& capability,
    const std::vector<AnatomicalAvailability>& anatomy,
    const MotionPhysicalContext& physical) {

    std::unordered_map<std::string, double>
        fractions;

    fractions.reserve(
        anatomy.size());

    for (const auto& region : anatomy) {
        fractions.emplace(
            region.region,
            region.attached_fraction());
    }

    MotionViabilityResult result;

    for (const auto& requirement
         : capability.required_regions) {

        if (fraction_for(
                fractions,
                requirement.region)
            >= requirement
                .minimum_attached_fraction) {
            continue;
        }

        result.failed_regions.push_back(
            requirement.region);

        result.failed_requirements.push_back(
            "required_region:"
            + requirement.region);
    }

    for (const auto& substitution
         : capability.allowed_substitutions) {

        std::size_t usable = 0;

        for (const auto& region
             : substitution.candidate_regions) {

            if (fraction_for(
                    fractions,
                    region)
                >= substitution
                    .minimum_attached_fraction) {
                ++usable;
            }
        }

        if (usable
            >= substitution.minimum_count) {

            result.usable_substitutions.push_back(
                substitution.label);
        } else {
            result.failed_requirements.push_back(
                "substitution:"
                + substitution.label);
        }
    }

    if (physical.has_support_contacts
        && physical.support_contacts
            < capability
                .minimum_support_contacts) {

        result.missing_support_contacts =
            capability
                .minimum_support_contacts
            - physical.support_contacts;

        result.failed_requirements.push_back(
            "support_contacts");
    }

    if (physical.has_grounded_state
        && capability.requires_grounded
        && !physical.grounded) {

        result.failed_requirements.push_back(
            "grounded_state");
    }

    if (!result.failed_requirements.empty()) {
        result.state =
            MotionViability::Invalid;
        result.transition_urgency = 1.0;
        return result;
    }

    for (const auto& optional
         : capability.optional_regions) {

        if (fraction_for(
                fractions,
                optional.region)
            < optional
                .minimum_attached_fraction) {

            result.state =
                MotionViability::Degraded;
            result.transition_urgency =
                std::max(
                    result.transition_urgency,
                    0.35);
        }
    }

    return result;
}

} // namespace sarx
