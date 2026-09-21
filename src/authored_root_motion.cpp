#include "sarx/authored_root_motion.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace sarx {
namespace {

std::vector<std::string> split_csv_line(
    const std::string& line) {

    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;

    while (std::getline(stream, field, ',')) {
        fields.push_back(field);
    }

    return fields;
}

double lerp(
    double a,
    double b,
    double t) {

    return a + (b - a) * t;
}

} // namespace

void AuthoredRootMotionCurve::load_csv(
    const std::string& path) {

    std::ifstream input(path);

    if (!input) {
        throw std::runtime_error(
            "could not open authored root-motion CSV: "
            + path);
    }

    std::string line;

    if (!std::getline(input, line)) {
        throw std::runtime_error(
            "authored root-motion CSV is empty: "
            + path);
    }

    if (line.find("time_seconds") == std::string::npos
        || line.find("distance_m") == std::string::npos
        || line.find("vertical_m") == std::string::npos) {

        throw std::runtime_error(
            "authored root-motion CSV has unexpected header: "
            + path);
    }

    std::vector<AuthoredRootMotionSample> next;

    while (std::getline(input, line)) {
        if (line.empty()) {
            continue;
        }

        const auto fields =
            split_csv_line(line);

        if (fields.size() < 3) {
            throw std::runtime_error(
                "malformed authored root-motion row: "
                + line);
        }

        AuthoredRootMotionSample sample;

        try {
            sample.time_seconds =
                std::stod(fields[0]);

            sample.distance_m =
                std::stod(fields[1]);

            sample.vertical_m =
                std::stod(fields[2]);
        } catch (const std::exception&) {
            throw std::runtime_error(
                "non-numeric authored root-motion row: "
                + line);
        }

        if (!std::isfinite(sample.time_seconds)
            || !std::isfinite(sample.distance_m)
            || !std::isfinite(sample.vertical_m)) {

            throw std::runtime_error(
                "non-finite authored root-motion row: "
                + line);
        }

        if (!next.empty()) {
            if (sample.time_seconds
                <= next.back().time_seconds) {

                throw std::runtime_error(
                    "authored root-motion time must be strictly increasing");
            }

        }

        next.push_back(sample);
    }

    if (next.size() < 2) {
        throw std::runtime_error(
            "authored root-motion CSV needs at least two samples");
    }

    samples_ =
        std::move(next);
}

double
AuthoredRootMotionCurve::duration_seconds() const {

    return samples_.empty()
        ? 0.0
        : samples_.back().time_seconds;
}

double
AuthoredRootMotionCurve::total_distance_m() const {

    return samples_.empty()
        ? 0.0
        : samples_.back().distance_m;
}

AuthoredRootMotionValue
AuthoredRootMotionCurve::sample(
    double time_seconds) const {

    if (samples_.empty()) {
        throw std::logic_error(
            "cannot sample empty authored root-motion curve");
    }

    if (time_seconds
        <= samples_.front().time_seconds) {

        return {
            samples_.front().distance_m,
            samples_.front().vertical_m
        };
    }

    if (time_seconds
        >= samples_.back().time_seconds) {

        return {
            samples_.back().distance_m,
            samples_.back().vertical_m
        };
    }

    const auto upper =
        std::upper_bound(
            samples_.begin(),
            samples_.end(),
            time_seconds,
            [](double time,
               const AuthoredRootMotionSample& sample) {
                return time < sample.time_seconds;
            });

    const auto lower =
        upper - 1;

    const double span =
        upper->time_seconds
        - lower->time_seconds;

    const double t =
        span > 1e-12
        ? (time_seconds
            - lower->time_seconds)
            / span
        : 0.0;

    return {
        lerp(
            lower->distance_m,
            upper->distance_m,
            t),
        lerp(
            lower->vertical_m,
            upper->vertical_m,
            t)
    };
}

} // namespace sarx
