#pragma once

#include <string>
#include <vector>

namespace sarx {

struct AuthoredRootMotionSample {
    double time_seconds{};
    double distance_m{};
    double vertical_m{};
};

struct AuthoredRootMotionValue {
    double distance_m{};
    double vertical_m{};
};

class AuthoredRootMotionCurve {
public:
    void load_csv(const std::string& path);

    [[nodiscard]] bool empty() const {
        return samples_.empty();
    }

    [[nodiscard]] double duration_seconds() const;

    [[nodiscard]] double total_distance_m() const;

    [[nodiscard]] AuthoredRootMotionValue sample(
        double time_seconds) const;

    [[nodiscard]] const std::vector<AuthoredRootMotionSample>&
    samples() const {
        return samples_;
    }

private:
    std::vector<AuthoredRootMotionSample> samples_;
};

} // namespace sarx
