#pragma once

#include "sarx/body.hpp"

#include <cstddef>
#include <vector>

namespace sarx {

struct PassiveJointProfile {
    std::size_t anchor_index{};
    double min_angle{0.0};
    double max_angle{3.14159265358979323846};
    double passive_strength{0.08};
    double limit_strength{0.65};
    double damping{0.20};
};

struct DetachedArticulationConfig {
    std::vector<Vec3> rest_anchors;
    std::vector<Vec3> previous_anchors;
    std::vector<double> masses;
    std::vector<PassiveJointProfile> joints;

    double structural_compliance{1e-8};
    double segment_length_strength{0.98};

    double ground_radius{0.04};
    // Optional per-segment floor-contact radii (segment i spans anchors i and
    // i+1). Empty: every segment uses ground_radius.
    std::vector<double> segment_ground_radii;
    double restitution{0.08};
    double tangential_damping{0.66};

    double contact_velocity_scale{0.22};
    double global_velocity_damping{0.992};

    int contact_iterations{8};
    int substeps{6};
    int solver_iterations{16};

    Vec3 gravity{0.0, -9.81, 0.0};
};

[[nodiscard]] double articulated_joint_angle(
    const Vec3& a,
    const Vec3& joint,
    const Vec3& b);

class DetachedArticulatedChain {
public:
    void initialize(
        const DetachedArticulationConfig& config,
        double dt);

    void step(double dt);

    [[nodiscard]] std::size_t anchor_count() const {
        return particle_ids_.size();
    }

    [[nodiscard]] std::size_t segment_count() const {
        return rest_lengths_.size();
    }

    [[nodiscard]] std::size_t joint_count() const {
        return joint_profiles_.size();
    }

    [[nodiscard]] Vec3 anchor_position(
        std::size_t index) const;

    [[nodiscard]] Vec3 anchor_velocity(
        std::size_t index) const;

    [[nodiscard]] double joint_angle(
        std::size_t joint_profile_index) const;

    [[nodiscard]] double initial_joint_angle(
        std::size_t joint_profile_index) const;

    [[nodiscard]] double max_joint_angle_delta(
        std::size_t joint_profile_index) const;

    [[nodiscard]] Vec3 transform_point(
        std::size_t segment_index,
        const Vec3& rest_point) const;

    [[nodiscard]] double segment_rotation_radians(
        std::size_t segment_index) const;

    [[nodiscard]] std::size_t ground_contacts() const {
        return ground_contacts_;
    }

    [[nodiscard]] bool ever_grounded() const {
        return ground_contacts_ > 0;
    }

    [[nodiscard]] const std::vector<Vec3>&
    rest_anchors() const {
        return rest_anchors_;
    }

private:
    Body body_;
    DetachedArticulationConfig config_{};

    std::vector<ParticleId> particle_ids_;
    std::vector<Vec3> rest_anchors_;
    std::vector<double> rest_lengths_;
    std::vector<PassiveJointProfile> joint_profiles_;
    std::vector<double> initial_joint_angles_;
    std::vector<double> max_joint_angle_deltas_;

    std::size_t ground_contacts_{};
};

} // namespace sarx
