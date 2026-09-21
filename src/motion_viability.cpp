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

} // namespace

MotionViabilityResult
evaluate_motion_viability(
    const std::string& animation_name,
    const std::vector<AnatomicalAvailability>& anatomy) {

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

    const std::string name =
        lower_copy(animation_name);

    if (name.find("walk")
            == std::string::npos
        && name.find("run")
            == std::string::npos) {
        return result;
    }

    struct Requirement {
        const char* region;
        double minimum_fraction;
    };

    static constexpr Requirement
        critical[] = {
            {"thigh_l", 0.60},
            {"calf_l", 0.60},
            {"foot_l", 0.55},
            {"thigh_r", 0.60},
            {"calf_r", 0.60},
            {"foot_r", 0.55}
        };

    for (const auto& requirement
         : critical) {

        if (fraction_for(
                fractions,
                requirement.region)
            < requirement.minimum_fraction) {

            result.failed_regions.push_back(
                requirement.region);
        }
    }

    if (!result.failed_regions.empty()) {
        result.state =
            MotionViability::Invalid;
        return result;
    }

    const double left_arm =
        std::min({
            fraction_for(
                fractions,
                "upperarm_l"),
            fraction_for(
                fractions,
                "lowerarm_l")
        });

    const double right_arm =
        std::min({
            fraction_for(
                fractions,
                "upperarm_r"),
            fraction_for(
                fractions,
                "lowerarm_r")
        });

    if (left_arm < 0.50
        || right_arm < 0.50) {
        result.state =
            MotionViability::Degraded;
    }

    return result;
}

} // namespace sarx
