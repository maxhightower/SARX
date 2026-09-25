#include "sarx/detached_articulation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sarx {
namespace {

Vec3 rotate_axis_angle(
    const Vec3& value,
    const Vec3& axis,
    double angle) {

    const Vec3 n =
        normalized(axis);

    if (length_squared(n) <= 1e-12) {
        return value;
    }

    const double c = std::cos(angle);
    const double s = std::sin(angle);

    return value * c
        + cross(n, value) * s
        + n * (
            dot(n, value)
            * (1.0 - c));
}

void project_distance(
    Particle& a,
    Particle& b,
    double rest_length,
    double strength) {

    const Vec3 delta =
        b.position - a.position;

    const double current_length =
        length(delta);

    if (current_length <= 1e-12) {
        return;
    }

    const Vec3 correction =
        delta
        * ((current_length - rest_length)
           / current_length
           * 0.5
           * strength);

    a.position += correction;
    b.position -= correction;
}

void project_passive_joint(
    Particle& a,
    Particle& joint,
    Particle& b,
    double rest_angle,
    const PassiveJointProfile& profile) {

    const Vec3 u =
        a.position - joint.position;

    const Vec3 v =
        b.position - joint.position;

    if (length_squared(u) <= 1e-12
        || length_squared(v) <= 1e-12) {
        return;
    }

    const double angle =
        articulated_joint_angle(
            a.position,
            joint.position,
            b.position);

    double target = rest_angle;
    double strength =
        profile.passive_strength;

    if (angle < profile.min_angle) {
        target = profile.min_angle;
        strength = profile.limit_strength;
    } else if (angle > profile.max_angle) {
        target = profile.max_angle;
        strength = profile.limit_strength;
    }

    const Vec3 axis =
        cross(u, v);

    if (length_squared(axis) <= 1e-12) {
        return;
    }

    const double half_correction =
        (target - angle)
        * strength
        * 0.5;

    a.position =
        joint.position
        + rotate_axis_angle(
            u,
            axis,
            -half_correction);

    b.position =
        joint.position
        + rotate_axis_angle(
            v,
            axis,
            half_correction);
}

void damp_passive_joint(
    Particle& a,
    Particle& joint,
    Particle& b,
    double damping) {

    const Vec3 u =
        normalized(
            a.position - joint.position);

    const Vec3 v =
        normalized(
            b.position - joint.position);

    if (length_squared(u) <= 1e-12
        || length_squared(v) <= 1e-12) {
        return;
    }

    const Vec3 rel_a =
        a.velocity - joint.velocity;

    const Vec3 rel_b =
        b.velocity - joint.velocity;

    const Vec3 tangent_a =
        rel_a
        - u * dot(rel_a, u);

    const Vec3 tangent_b =
        rel_b
        - v * dot(rel_b, v);

    a.velocity -=
        tangent_a * damping;

    b.velocity -=
        tangent_b * damping;

    joint.velocity +=
        (tangent_a + tangent_b)
        * (damping * 0.15);
}

bool project_segment_floor(
    Particle& a,
    Particle& b,
    double radius) {

    bool contacted = false;

    constexpr double samples[] = {
        0.15,
        0.35,
        0.50,
        0.65,
        0.85
    };

    for (const double t : samples) {
        const double wa = 1.0 - t;
        const double wb = t;

        const double y =
            a.position.y * wa
            + b.position.y * wb;

        if (y >= radius) {
            continue;
        }

        const double denominator =
            wa * wa + wb * wb;

        if (denominator <= 1e-12) {
            continue;
        }

        const double correction =
            (radius - y)
            / denominator;

        a.position.y +=
            correction * wa;

        b.position.y +=
            correction * wb;

        contacted = true;
    }

    return contacted;
}

} // namespace

double articulated_joint_angle(
    const Vec3& a,
    const Vec3& joint,
    const Vec3& b) {

    const Vec3 u =
        normalized(a - joint);

    const Vec3 v =
        normalized(b - joint);

    if (length_squared(u) <= 1e-12
        || length_squared(v) <= 1e-12) {
        return 0.0;
    }

    return std::acos(
        std::clamp(
            dot(u, v),
            -1.0,
            1.0));
}

void DetachedArticulatedChain::initialize(
    const DetachedArticulationConfig& config,
    double dt) {

    if (dt <= 0.0
        || config.rest_anchors.size() < 2
        || config.previous_anchors.size()
            != config.rest_anchors.size()) {
        throw std::invalid_argument(
            "invalid detached articulation initialization");
    }

    config_ = config;

    rest_anchors_ =
        config.rest_anchors;

    joint_profiles_ =
        config.joints;

    body_ = Body{};
    particle_ids_.clear();
    rest_lengths_.clear();
    initial_joint_angles_.clear();
    max_joint_angle_deltas_.clear();
    ground_contacts_ = 0;

    particle_ids_.reserve(
        rest_anchors_.size());

    const bool explicit_masses =
        config.masses.size()
            == rest_anchors_.size();

    for (std::size_t i = 0;
         i < rest_anchors_.size();
         ++i) {

        const double mass =
            explicit_masses
            ? config.masses[i]
            : 1.0;

        const ParticleId id =
            body_.add_particle(
                rest_anchors_[i],
                mass);

        body_.particles()[id].velocity =
            (rest_anchors_[i]
             - config.previous_anchors[i])
            / dt;

        particle_ids_.push_back(id);
    }

    rest_lengths_.reserve(
        rest_anchors_.size() - 1);

    for (std::size_t i = 0;
         i + 1 < rest_anchors_.size();
         ++i) {

        const double rest_length =
            length(
                rest_anchors_[i + 1]
                - rest_anchors_[i]);

        rest_lengths_.push_back(
            rest_length);

        body_.add_structural_constraint(
            particle_ids_[i],
            particle_ids_[i + 1],
            config.structural_compliance);
    }

    initial_joint_angles_.reserve(
        joint_profiles_.size());

    max_joint_angle_deltas_.assign(
        joint_profiles_.size(),
        0.0);

    for (const auto& profile
         : joint_profiles_) {

        if (profile.anchor_index == 0
            || profile.anchor_index + 1
                >= rest_anchors_.size()
            || profile.min_angle
                > profile.max_angle) {
            throw std::invalid_argument(
                "invalid passive joint profile");
        }

        initial_joint_angles_.push_back(
            articulated_joint_angle(
                rest_anchors_[
                    profile.anchor_index - 1],
                rest_anchors_[
                    profile.anchor_index],
                rest_anchors_[
                    profile.anchor_index + 1]));
    }
}

void DetachedArticulatedChain::step(
    double dt) {

    if (dt <= 0.0
        || particle_ids_.size() < 2) {
        throw std::invalid_argument(
            "detached articulation step requires initialized chain");
    }

    StepConfig step_config;
    step_config.substeps =
        config_.substeps;

    step_config.solver_iterations =
        config_.solver_iterations;

    step_config.gravity =
        config_.gravity;

    body_.step(
        dt,
        step_config);

    auto& particles =
        body_.particles();

    std::vector<Vec3> before_contact(
        particle_ids_.size());

    std::vector<bool> touched(
        particle_ids_.size(),
        false);

    for (std::size_t i = 0;
         i < particle_ids_.size();
         ++i) {

        before_contact[i] =
            particles[
                particle_ids_[i]]
                .position;
    }

    for (int iteration = 0;
         iteration < config_.contact_iterations;
         ++iteration) {

        for (std::size_t segment = 0;
             segment < rest_lengths_.size();
             ++segment) {

            const bool hit =
                project_segment_floor(
                    particles[
                        particle_ids_[segment]],
                    particles[
                        particle_ids_[segment + 1]],
                    segment < config_.segment_ground_radii.size()
                        ? config_.segment_ground_radii[segment]
                        : config_.ground_radius);

            if (hit) {
                touched[segment] = true;
                touched[segment + 1] = true;
            }
        }

        for (std::size_t segment = 0;
             segment < rest_lengths_.size();
             ++segment) {

            project_distance(
                particles[
                    particle_ids_[segment]],
                particles[
                    particle_ids_[segment + 1]],
                rest_lengths_[segment],
                config_.segment_length_strength);
        }

        for (std::size_t joint = 0;
             joint < joint_profiles_.size();
             ++joint) {

            const auto& profile =
                joint_profiles_[joint];

            project_passive_joint(
                particles[
                    particle_ids_[
                        profile.anchor_index - 1]],
                particles[
                    particle_ids_[
                        profile.anchor_index]],
                particles[
                    particle_ids_[
                        profile.anchor_index + 1]],
                initial_joint_angles_[joint],
                profile);
        }

        for (std::size_t segment = 0;
             segment < rest_lengths_.size();
             ++segment) {

            project_distance(
                particles[
                    particle_ids_[segment]],
                particles[
                    particle_ids_[segment + 1]],
                rest_lengths_[segment],
                config_.segment_length_strength);
        }
    }

    for (std::size_t joint = 0;
         joint < joint_profiles_.size();
         ++joint) {

        const auto& profile =
            joint_profiles_[joint];

        damp_passive_joint(
            particles[
                particle_ids_[
                    profile.anchor_index - 1]],
            particles[
                particle_ids_[
                    profile.anchor_index]],
            particles[
                particle_ids_[
                    profile.anchor_index + 1]],
            profile.damping);
    }

    for (std::size_t i = 0;
         i < particle_ids_.size();
         ++i) {

        Particle& particle =
            particles[
                particle_ids_[i]];

        const Vec3 positional_impulse =
            (particle.position
             - before_contact[i])
            / dt;

        particle.velocity +=
            positional_impulse
            * config_.contact_velocity_scale;

        particle.velocity *=
            config_.global_velocity_damping;

        if (!touched[i]) {
            continue;
        }

        if (particle.velocity.y < 0.0) {
            particle.velocity.y =
                -particle.velocity.y
                * config_.restitution;
        }

        particle.velocity.x *=
            config_.tangential_damping;

        particle.velocity.z *=
            config_.tangential_damping;

        if (std::abs(
                particle.velocity.y)
            < 0.05) {
            particle.velocity.y = 0.0;
        }

        ++ground_contacts_;
    }

    for (std::size_t joint = 0;
         joint < joint_profiles_.size();
         ++joint) {

        const double delta =
            std::abs(
                joint_angle(joint)
                - initial_joint_angles_[joint]);

        max_joint_angle_deltas_[joint] =
            std::max(
                max_joint_angle_deltas_[joint],
                delta);
    }
}

Vec3 DetachedArticulatedChain::anchor_position(
    std::size_t index) const {

    if (index >= particle_ids_.size()) {
        throw std::out_of_range(
            "detached articulation anchor out of range");
    }

    return body_.particles()[
        particle_ids_[index]]
        .position;
}

Vec3 DetachedArticulatedChain::anchor_velocity(
    std::size_t index) const {

    if (index >= particle_ids_.size()) {
        throw std::out_of_range(
            "detached articulation anchor out of range");
    }

    return body_.particles()[
        particle_ids_[index]]
        .velocity;
}

double DetachedArticulatedChain::joint_angle(
    std::size_t joint_profile_index) const {

    if (joint_profile_index
        >= joint_profiles_.size()) {
        throw std::out_of_range(
            "detached articulation joint out of range");
    }

    const auto& profile =
        joint_profiles_[
            joint_profile_index];

    return articulated_joint_angle(
        anchor_position(
            profile.anchor_index - 1),
        anchor_position(
            profile.anchor_index),
        anchor_position(
            profile.anchor_index + 1));
}

double DetachedArticulatedChain::initial_joint_angle(
    std::size_t joint_profile_index) const {

    if (joint_profile_index
        >= initial_joint_angles_.size()) {
        throw std::out_of_range(
            "detached articulation joint out of range");
    }

    return initial_joint_angles_[
        joint_profile_index];
}

double DetachedArticulatedChain::max_joint_angle_delta(
    std::size_t joint_profile_index) const {

    if (joint_profile_index
        >= max_joint_angle_deltas_.size()) {
        throw std::out_of_range(
            "detached articulation joint out of range");
    }

    return max_joint_angle_deltas_[
        joint_profile_index];
}

Vec3 DetachedArticulatedChain::transform_point(
    std::size_t segment_index,
    const Vec3& rest_point) const {

    if (segment_index
            >= rest_lengths_.size()
        || segment_index + 1
            >= rest_anchors_.size()) {
        throw std::out_of_range(
            "detached articulation segment out of range");
    }

    const Vec3 current_a =
        anchor_position(segment_index);

    const Vec3 current_b =
        anchor_position(
            segment_index + 1);

    const Vec3 rest_axis =
        rest_anchors_[segment_index + 1]
        - rest_anchors_[segment_index];

    const Vec3 current_axis =
        current_b - current_a;

    return
        current_a
        + rotate_between(
            rest_axis,
            current_axis,
            rest_point
            - rest_anchors_[segment_index]);
}

double DetachedArticulatedChain::segment_rotation_radians(
    std::size_t segment_index) const {

    if (segment_index
        >= rest_lengths_.size()) {
        throw std::out_of_range(
            "detached articulation segment out of range");
    }

    const Vec3 rest_axis =
        normalized(
            rest_anchors_[segment_index + 1]
            - rest_anchors_[segment_index]);

    const Vec3 current_axis =
        normalized(
            anchor_position(segment_index + 1)
            - anchor_position(segment_index));

    if (length_squared(rest_axis) <= 1e-12
        || length_squared(current_axis) <= 1e-12) {
        return 0.0;
    }

    return std::acos(
        std::clamp(
            dot(rest_axis, current_axis),
            -1.0,
            1.0));
}

} // namespace sarx
