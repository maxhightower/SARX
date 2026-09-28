#include "sarx/body.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace sarx {
namespace {

double mass_of(const Particle& p) {
    return p.inverse_mass > 0.0 ? 1.0 / p.inverse_mass : 0.0;
}

class DisjointSet {
public:
    explicit DisjointSet(std::size_t n) : parent_(n), rank_(n, 0) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }

    std::size_t find(std::size_t x) {
        if (parent_[x] != x) {
            parent_[x] = find(parent_[x]);
        }
        return parent_[x];
    }

    void unite(std::size_t a, std::size_t b) {
        a = find(a);
        b = find(b);
        if (a == b) return;

        if (rank_[a] < rank_[b]) std::swap(a, b);
        parent_[b] = a;
        if (rank_[a] == rank_[b]) ++rank_[a];
    }

private:
    std::vector<std::size_t> parent_;
    std::vector<unsigned char> rank_;
};

} // namespace

ParticleId Body::add_particle(const Vec3& position, double mass) {
    if (mass < 0.0) {
        throw std::invalid_argument("particle mass must be non-negative");
    }

    Particle p;
    p.position = position;
    p.inverse_mass = mass > 0.0 ? 1.0 / mass : 0.0;
    particles_.push_back(p);
    return particles_.size() - 1;
}

BoneId Body::add_bone(
    BoneId parent,
    const Vec3& animated_position,
    double joint_break_damage,
    MaterialId joint_material,
    double joint_radius) {

    if (parent != kNoParent && parent >= bones_.size()) {
        throw std::out_of_range("bone parent must already exist");
    }
    if (joint_break_damage <= 0.0 || joint_radius < 0.0) {
        throw std::invalid_argument("invalid joint parameters");
    }

    Bone bone;
    bone.parent = parent;
    bone.animated_position = animated_position;
    bone.joint_break_damage = joint_break_damage;
    bone.joint_material = joint_material;
    bone.joint_radius = joint_radius;
    bones_.push_back(bone);
    return bones_.size() - 1;
}

ConstraintId Body::add_structural_constraint(
    ParticleId a,
    ParticleId b,
    double compliance,
    double break_damage,
    MaterialId material,
    const Vec3& material_fiber_rest) {

    if (a >= particles_.size() || b >= particles_.size() || a == b) {
        throw std::out_of_range("invalid structural constraint endpoints");
    }
    if (compliance < 0.0 || break_damage <= 0.0) {
        throw std::invalid_argument("invalid structural constraint parameters");
    }

    StructuralConstraint c;
    c.a = a;
    c.b = b;
    const Vec3 rest_delta = particles_[b].position - particles_[a].position;
    c.rest_length = length(rest_delta);
    c.rest_direction = normalized(rest_delta);
    c.compliance = compliance;
    c.break_damage = break_damage;
    c.material = material;
    c.material_fiber_rest = material_fiber_rest;
    structural_.push_back(c);
    return structural_.size() - 1;
}

ConstraintId Body::add_tetrahedral_constraint(
    ParticleId a,
    ParticleId b,
    ParticleId c,
    ParticleId d,
    double compliance,
    double break_damage,
    MaterialId material) {

    const std::size_t n = particles_.size();
    if (a >= n || b >= n || c >= n || d >= n
        || a == b || a == c || a == d
        || b == c || b == d || c == d) {
        throw std::out_of_range("invalid tetrahedral constraint particles");
    }
    if (compliance < 0.0 || break_damage <= 0.0) {
        throw std::invalid_argument("invalid tetrahedral parameters");
    }

    const Vec3& p0 = particles_[a].position;
    const Vec3& p1 = particles_[b].position;
    const Vec3& p2 = particles_[c].position;
    const Vec3& p3 = particles_[d].position;
    const double rest_volume =
        dot(p1 - p0, cross(p2 - p0, p3 - p0)) / 6.0;

    if (std::abs(rest_volume) <= 1e-12) {
        throw std::invalid_argument("tetrahedral rest volume must be non-zero");
    }

    tetrahedral_.push_back(TetrahedralConstraint{
        a,
        b,
        c,
        d,
        rest_volume,
        compliance,
        0.0,
        break_damage,
        0.0,
        true,
        material
    });
    return tetrahedral_.size() - 1;
}

ConstraintId Body::add_attachment(
    ParticleId particle,
    BoneId bone,
    const Vec3& local_offset,
    double compliance,
    double break_damage,
    MaterialId material) {

    if (particle >= particles_.size() || bone >= bones_.size()) {
        throw std::out_of_range("invalid attachment");
    }
    if (compliance < 0.0 || break_damage <= 0.0) {
        throw std::invalid_argument("invalid attachment parameters");
    }

    AttachmentConstraint a;
    a.particle = particle;
    a.bone = bone;
    a.local_offset = local_offset;
    a.compliance = compliance;
    a.break_damage = break_damage;
    a.material = material;
    attachments_.push_back(a);
    return attachments_.size() - 1;
}

void Body::set_bone_target(BoneId bone, const Vec3& animated_position) {
    if (bone >= bones_.size()) {
        throw std::out_of_range("invalid bone");
    }
    bones_[bone].animated_position = animated_position;
}

void Body::set_bone_pose(BoneId bone, const Vec3& animated_position, const Rotation& animated_rotation) {
    if (bone >= bones_.size()) {
        throw std::out_of_range("invalid bone");
    }
    bones_[bone].animated_position = animated_position;
    bones_[bone].animated_rotation = normalized(animated_rotation);
}

void Body::damage_structural(ConstraintId constraint, double amount) {
    if (constraint >= structural_.size()) {
        throw std::out_of_range("invalid structural constraint");
    }
    if (amount < 0.0) {
        throw std::invalid_argument("damage amount must be non-negative");
    }

    auto& c = structural_[constraint];
    c.damage += amount;
    if (c.damage >= c.break_damage) {
        c.active = false;
        c.lambda = 0.0;
    }
}

void Body::damage_tetrahedral(ConstraintId constraint, double amount) {
    if (constraint >= tetrahedral_.size()) {
        throw std::out_of_range("invalid tetrahedral constraint");
    }
    if (amount < 0.0) {
        throw std::invalid_argument("damage amount must be non-negative");
    }

    auto& t = tetrahedral_[constraint];
    t.damage += amount;
    if (t.damage >= t.break_damage) {
        t.active = false;
        t.lambda = 0.0;
    }
}

void Body::damage_attachment(ConstraintId constraint, double amount) {
    if (constraint >= attachments_.size()) {
        throw std::out_of_range("invalid attachment constraint");
    }
    if (amount < 0.0) {
        throw std::invalid_argument("damage amount must be non-negative");
    }

    auto& c = attachments_[constraint];
    c.damage += amount;
    if (c.damage >= c.break_damage) {
        c.active = false;
        c.lambda = {};
    }
}

void Body::damage_bone_joint(BoneId bone, double amount) {
    if (bone >= bones_.size()) {
        throw std::out_of_range("invalid bone");
    }
    if (amount < 0.0) {
        throw std::invalid_argument("damage amount must be non-negative");
    }

    auto& b = bones_[bone];
    if (b.parent == kNoParent) {
        return;
    }

    b.joint_damage += amount;
    if (b.joint_damage >= b.joint_break_damage) {
        b.joint_to_parent_active = false;
    }
}

void Body::break_structural(ConstraintId constraint) {
    const auto& c = structural_.at(constraint);
    damage_structural(constraint, std::max(0.0, c.break_damage - c.damage));
}

void Body::break_tetrahedral(ConstraintId constraint) {
    const auto& t = tetrahedral_.at(constraint);
    damage_tetrahedral(
        constraint,
        std::max(0.0, t.break_damage - t.damage));
}

void Body::break_attachment(ConstraintId constraint) {
    const auto& a = attachments_.at(constraint);
    damage_attachment(constraint, std::max(0.0, a.break_damage - a.damage));
}

void Body::break_bone_joint(BoneId bone) {
    const auto& b = bones_.at(bone);
    if (b.parent == kNoParent) {
        return;
    }
    damage_bone_joint(bone, std::max(0.0, b.joint_break_damage - b.joint_damage));
}

void Body::relax_rest_state(const std::vector<std::uint8_t>& selected) {
    const bool all = selected.empty();
    if (!all && selected.size() != particles_.size()) {
        throw std::invalid_argument("relax selection must cover every particle");
    }
    auto chosen = [&](ParticleId p) { return all || selected[p] != 0u; };

    for (auto& c : structural_) {
        if (!c.active || !chosen(c.a) || !chosen(c.b)) continue;
        const Vec3 delta = particles_[c.b].position - particles_[c.a].position;
        const double len = length(delta);
        if (len <= 1e-9) continue;
        c.rest_length = len;
        c.rest_direction = delta / len;
        c.lambda = 0.0;
    }
    for (auto& t : tetrahedral_) {
        if (!t.active || !chosen(t.a) || !chosen(t.b) || !chosen(t.c) || !chosen(t.d)) continue;
        const Vec3& p0 = particles_[t.a].position;
        const double volume = dot(particles_[t.b].position - p0,
                                  cross(particles_[t.c].position - p0, particles_[t.d].position - p0)) / 6.0;
        // Keep orientation: never relax into an inverted element.
        if (std::abs(volume) <= 1e-12 || (volume > 0.0) != (t.rest_volume > 0.0)) continue;
        t.rest_volume = volume;
        t.lambda = 0.0;
    }
}

bool Body::bone_root_connected(BoneId bone) const {
    if (bone >= bones_.size()) {
        return false;
    }

    std::size_t guard = 0;
    BoneId current = bone;
    while (current != kNoParent) {
        if (++guard > bones_.size()) {
            return false;
        }

        const auto& b = bones_[current];
        if (b.parent == kNoParent) {
            return true;
        }
        if (!b.joint_to_parent_active) {
            return false;
        }
        current = b.parent;
    }
    return false;
}

std::vector<Island> Body::islands() const {
    if (particles_.empty()) {
        return {};
    }

    DisjointSet dsu(particles_.size());
    for (const auto& c : structural_) {
        if (c.active) {
            dsu.unite(c.a, c.b);
        }
    }

    for (const auto& t : tetrahedral_) {
        if (!t.active) continue;
        dsu.unite(t.a, t.b);
        dsu.unite(t.a, t.c);
        dsu.unite(t.a, t.d);
    }

    std::unordered_map<std::size_t, std::size_t> root_to_island;
    std::vector<Island> result;
    result.reserve(particles_.size());

    for (ParticleId p = 0; p < particles_.size(); ++p) {
        const auto root = dsu.find(p);
        auto [it, inserted] = root_to_island.emplace(root, result.size());
        if (inserted) {
            result.push_back({});
        }

        auto& island = result[it->second];
        island.particles.push_back(p);
        const double m = mass_of(particles_[p]);
        island.mass += m;
        island.linear_momentum += particles_[p].velocity * m;
    }

    for (const auto& a : attachments_) {
        if (!a.active || !bone_root_connected(a.bone)) {
            continue;
        }

        const auto root = dsu.find(a.particle);
        const auto it = root_to_island.find(root);
        if (it != root_to_island.end()) {
            result[it->second].rig_authoritative = true;
        }
    }

    return result;
}

Vec3 Body::total_linear_momentum() const {
    Vec3 p{};
    for (const auto& particle : particles_) {
        p += particle.velocity * mass_of(particle);
    }
    return p;
}

void Body::step(double dt, const StepConfig& config) {
    SolverDomain domain;

    domain.particles.resize(particles_.size());
    std::iota(domain.particles.begin(), domain.particles.end(), ParticleId{0});

    domain.structural.resize(structural_.size());
    std::iota(domain.structural.begin(), domain.structural.end(), ConstraintId{0});

    domain.tetrahedral.resize(tetrahedral_.size());
    std::iota(domain.tetrahedral.begin(), domain.tetrahedral.end(), ConstraintId{0});

    domain.attachments.resize(attachments_.size());
    std::iota(domain.attachments.begin(), domain.attachments.end(), ConstraintId{0});

    (void)step_restricted(dt, domain, config);
}

StepStats Body::step_restricted(
    double dt,
    const SolverDomain& input_domain,
    const StepConfig& config) {

    if (dt <= 0.0) {
        throw std::invalid_argument("dt must be positive");
    }
    if (config.substeps <= 0 || config.solver_iterations <= 0) {
        throw std::invalid_argument("step configuration counts must be positive");
    }

    auto canonicalize = [](auto ids, std::size_t limit, const char* label) {
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        for (const auto id : ids) {
            if (id >= limit) {
                throw std::out_of_range(label);
            }
        }
        return ids;
    };

    SolverDomain domain;
    domain.particles = canonicalize(
        input_domain.particles,
        particles_.size(),
        "restricted particle id out of range");
    domain.structural = canonicalize(
        input_domain.structural,
        structural_.size(),
        "restricted structural id out of range");
    domain.tetrahedral = canonicalize(
        input_domain.tetrahedral,
        tetrahedral_.size(),
        "restricted tetrahedral id out of range");
    domain.attachments = canonicalize(
        input_domain.attachments,
        attachments_.size(),
        "restricted attachment id out of range");

    std::vector<std::uint8_t> active_particles(particles_.size(), 0u);
    for (const ParticleId id : domain.particles) {
        active_particles[id] = 1u;
    }

    StepStats stats;
    stats.active_particles = domain.particles.size();
    stats.structural_constraints = domain.structural.size();
    stats.tetrahedral_constraints = domain.tetrahedral.size();
    stats.attachment_constraints = domain.attachments.size();

    const double h = dt / static_cast<double>(config.substeps);

    for (int substep = 0; substep < config.substeps; ++substep) {
        std::vector<Vec3> before(particles_.size());

        for (const ParticleId id : domain.particles) {
            auto& p = particles_[id];
            before[id] = p.position;

            if (p.inverse_mass == 0.0) {
                p.velocity = {};
                continue;
            }

            p.velocity += config.gravity * h;
            p.position += p.velocity * h;
        }

        for (const ConstraintId id : domain.structural) {
            structural_[id].lambda = 0.0;
        }
        for (const ConstraintId id : domain.tetrahedral) {
            tetrahedral_[id].lambda = 0.0;
        }
        for (const ConstraintId id : domain.attachments) {
            attachments_[id].lambda = {};
        }

        std::vector<Vec3> attachment_targets(domain.attachments.size());
        std::vector<std::uint8_t> attachment_connected(domain.attachments.size(), 0u);
        for (std::size_t k = 0; k < domain.attachments.size(); ++k) {
            const auto& a = attachments_[domain.attachments[k]];
            if (!a.active || !bone_root_connected(a.bone)) continue;
            attachment_connected[k] = 1u;
            attachment_targets[k] = attachment_target(a);
        }

        for (int iteration = 0; iteration < config.solver_iterations; ++iteration) {
            solve_structural(h, domain.structural, active_particles);
            solve_tetrahedral(h, domain.tetrahedral, active_particles);
            solve_attachments(h, domain.attachments, active_particles,
                              attachment_targets, attachment_connected);

            stats.solver_constraint_visits +=
                domain.structural.size()
                + domain.tetrahedral.size()
                + domain.attachments.size();
        }

        for (const ParticleId id : domain.particles) {
            auto& p = particles_[id];
            if (p.inverse_mass == 0.0) {
                continue;
            }
            p.velocity = (p.position - before[id]) / h;
        }
    }

    return stats;
}

void Body::solve_structural(
    double h,
    const std::vector<ConstraintId>& ids,
    const std::vector<std::uint8_t>& active_particles) {

    for (const ConstraintId id : ids) {
        auto& c = structural_[id];
        if (!c.active) continue;

        auto& a = particles_[c.a];
        auto& b = particles_[c.b];

        const double wa = active_particles[c.a] ? a.inverse_mass : 0.0;
        const double wb = active_particles[c.b] ? b.inverse_mass : 0.0;
        if (wa + wb <= 1e-12) continue;

        const Vec3 delta = b.position - a.position;
        const double len = length(delta);
        if (len <= 1e-12) continue;

        const Vec3 n = delta / len;
        const double C = len - c.rest_length;
        const double alpha = c.compliance / (h * h);
        const double denom = wa + wb + alpha;
        if (denom <= 1e-12) continue;

        const double dlambda = (-C - alpha * c.lambda) / denom;
        c.lambda += dlambda;

        if (wa > 0.0) {
            a.position += (-n) * (wa * dlambda);
        }
        if (wb > 0.0) {
            b.position += n * (wb * dlambda);
        }
    }
}

void Body::solve_tetrahedral(
    double h,
    const std::vector<ConstraintId>& ids,
    const std::vector<std::uint8_t>& active_particles) {

    for (const ConstraintId id : ids) {
        auto& t = tetrahedral_[id];
        if (!t.active) continue;

        auto& p0 = particles_[t.a];
        auto& p1 = particles_[t.b];
        auto& p2 = particles_[t.c];
        auto& p3 = particles_[t.d];

        const double w0 = active_particles[t.a] ? p0.inverse_mass : 0.0;
        const double w1 = active_particles[t.b] ? p1.inverse_mass : 0.0;
        const double w2 = active_particles[t.c] ? p2.inverse_mass : 0.0;
        const double w3 = active_particles[t.d] ? p3.inverse_mass : 0.0;

        if (w0 + w1 + w2 + w3 <= 1e-12) continue;

        const Vec3 e10 = p1.position - p0.position;
        const Vec3 e20 = p2.position - p0.position;
        const Vec3 e30 = p3.position - p0.position;

        const double volume = dot(e10, cross(e20, e30)) / 6.0;
        const double C = volume - t.rest_volume;

        const Vec3 g1 = cross(e20, e30) / 6.0;
        const Vec3 g2 = cross(e30, e10) / 6.0;
        const Vec3 g3 = cross(e10, e20) / 6.0;
        const Vec3 g0 = -(g1 + g2 + g3);

        const double weighted_gradient =
            w0 * length_squared(g0)
            + w1 * length_squared(g1)
            + w2 * length_squared(g2)
            + w3 * length_squared(g3);

        const double alpha = t.compliance / (h * h);
        const double denom = weighted_gradient + alpha;
        if (denom <= 1e-12) continue;

        const double dlambda = (-C - alpha * t.lambda) / denom;
        t.lambda += dlambda;

        if (w0 > 0.0) p0.position += g0 * (w0 * dlambda);
        if (w1 > 0.0) p1.position += g1 * (w1 * dlambda);
        if (w2 > 0.0) p2.position += g2 * (w2 * dlambda);
        if (w3 > 0.0) p3.position += g3 * (w3 * dlambda);
    }
}

void Body::solve_attachments(
    double h,
    const std::vector<ConstraintId>& ids,
    const std::vector<std::uint8_t>& active_particles,
    const std::vector<Vec3>& targets,
    const std::vector<std::uint8_t>& connected) {

    for (std::size_t k = 0; k < ids.size(); ++k) {
        if (!connected[k]) continue;
        auto& a = attachments_[ids[k]];
        if (!active_particles[a.particle]) continue;

        auto& p = particles_[a.particle];
        if (p.inverse_mass == 0.0) continue;

        const Vec3 C = p.position - targets[k];
        const double alpha = a.compliance / (h * h);
        const double denom = p.inverse_mass + alpha;
        if (denom <= 1e-12) continue;

        const Vec3 dlambda = (-C - a.lambda * alpha) / denom;
        a.lambda += dlambda;
        p.position += dlambda * p.inverse_mass;
    }
}

} // namespace sarx
