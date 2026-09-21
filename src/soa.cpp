#include "sarx/soa.hpp"

#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <vector>

namespace sarx {
namespace {

Vec3 particle_position(const ParticleSoA& particles, ParticleId id) {
    return {
        particles.px[id],
        particles.py[id],
        particles.pz[id]
    };
}

void set_particle_position(
    ParticleSoA& particles,
    ParticleId id,
    const Vec3& p) {

    particles.px[id] = p.x;
    particles.py[id] = p.y;
    particles.pz[id] = p.z;
}

Vec3 particle_velocity(const ParticleSoA& particles, ParticleId id) {
    return {
        particles.vx[id],
        particles.vy[id],
        particles.vz[id]
    };
}

void set_particle_velocity(
    ParticleSoA& particles,
    ParticleId id,
    const Vec3& v) {

    particles.vx[id] = v.x;
    particles.vy[id] = v.y;
    particles.vz[id] = v.z;
}

bool bone_root_connected(const BoneSoA& bones, BoneId bone) {
    if (bone >= bones.parent.size()) {
        return false;
    }

    std::size_t guard = 0;
    BoneId current = bone;
    while (current != kNoParent) {
        if (++guard > bones.parent.size()) {
            return false;
        }

        const BoneId parent = bones.parent[current];
        if (parent == kNoParent) {
            return true;
        }
        if (!bones.joint_active[current]) {
            return false;
        }
        current = parent;
    }
    return false;
}

void validate_soa(const BodySoA& body) {
    const auto n = body.particles.px.size();
    if (body.particles.py.size() != n
        || body.particles.pz.size() != n
        || body.particles.vx.size() != n
        || body.particles.vy.size() != n
        || body.particles.vz.size() != n
        || body.particles.inverse_mass.size() != n) {
        throw std::logic_error("malformed particle SoA");
    }

    const auto s = body.structural.a.size();
    if (body.structural.b.size() != s
        || body.structural.rest_length.size() != s
        || body.structural.compliance.size() != s
        || body.structural.damage.size() != s
        || body.structural.break_damage.size() != s
        || body.structural.active.size() != s
        || body.structural.material.size() != s) {
        throw std::logic_error("malformed structural SoA");
    }

    const auto t = body.tetrahedral.a.size();
    if (body.tetrahedral.b.size() != t
        || body.tetrahedral.c.size() != t
        || body.tetrahedral.d.size() != t
        || body.tetrahedral.rest_volume.size() != t
        || body.tetrahedral.compliance.size() != t
        || body.tetrahedral.damage.size() != t
        || body.tetrahedral.break_damage.size() != t
        || body.tetrahedral.active.size() != t
        || body.tetrahedral.material.size() != t) {
        throw std::logic_error("malformed tetrahedral SoA");
    }

    const auto b = body.bones.parent.size();
    if (body.bones.px.size() != b
        || body.bones.py.size() != b
        || body.bones.pz.size() != b
        || body.bones.joint_damage.size() != b
        || body.bones.joint_break_damage.size() != b
        || body.bones.joint_radius.size() != b
        || body.bones.joint_active.size() != b
        || body.bones.joint_material.size() != b) {
        throw std::logic_error("malformed bone SoA");
    }

    const auto a = body.attachments.particle.size();
    if (body.attachments.bone.size() != a
        || body.attachments.offset_x.size() != a
        || body.attachments.offset_y.size() != a
        || body.attachments.offset_z.size() != a
        || body.attachments.compliance.size() != a
        || body.attachments.damage.size() != a
        || body.attachments.break_damage.size() != a
        || body.attachments.active.size() != a
        || body.attachments.material.size() != a) {
        throw std::logic_error("malformed attachment SoA");
    }
}

} // namespace

BodySoA snapshot_body_soa(const Body& body) {
    BodySoA out;

    const auto& particles = body.particles();
    out.particles.px.reserve(particles.size());
    out.particles.py.reserve(particles.size());
    out.particles.pz.reserve(particles.size());
    out.particles.vx.reserve(particles.size());
    out.particles.vy.reserve(particles.size());
    out.particles.vz.reserve(particles.size());
    out.particles.inverse_mass.reserve(particles.size());

    for (const auto& p : particles) {
        out.particles.px.push_back(p.position.x);
        out.particles.py.push_back(p.position.y);
        out.particles.pz.push_back(p.position.z);
        out.particles.vx.push_back(p.velocity.x);
        out.particles.vy.push_back(p.velocity.y);
        out.particles.vz.push_back(p.velocity.z);
        out.particles.inverse_mass.push_back(p.inverse_mass);
    }

    const auto& structural = body.structural_constraints();
    for (const auto& c : structural) {
        out.structural.a.push_back(c.a);
        out.structural.b.push_back(c.b);
        out.structural.rest_length.push_back(c.rest_length);
        out.structural.compliance.push_back(c.compliance);
        out.structural.damage.push_back(c.damage);
        out.structural.break_damage.push_back(c.break_damage);
        out.structural.active.push_back(c.active ? 1u : 0u);
        out.structural.material.push_back(c.material);
    }

    const auto& tetrahedral = body.tetrahedral_constraints();
    for (const auto& t : tetrahedral) {
        out.tetrahedral.a.push_back(t.a);
        out.tetrahedral.b.push_back(t.b);
        out.tetrahedral.c.push_back(t.c);
        out.tetrahedral.d.push_back(t.d);
        out.tetrahedral.rest_volume.push_back(t.rest_volume);
        out.tetrahedral.compliance.push_back(t.compliance);
        out.tetrahedral.damage.push_back(t.damage);
        out.tetrahedral.break_damage.push_back(t.break_damage);
        out.tetrahedral.active.push_back(t.active ? 1u : 0u);
        out.tetrahedral.material.push_back(t.material);
    }

    const auto& bones = body.bones();
    for (const auto& b : bones) {
        out.bones.parent.push_back(b.parent);
        out.bones.px.push_back(b.animated_position.x);
        out.bones.py.push_back(b.animated_position.y);
        out.bones.pz.push_back(b.animated_position.z);
        out.bones.joint_damage.push_back(b.joint_damage);
        out.bones.joint_break_damage.push_back(b.joint_break_damage);
        out.bones.joint_radius.push_back(b.joint_radius);
        out.bones.joint_active.push_back(b.joint_to_parent_active ? 1u : 0u);
        out.bones.joint_material.push_back(b.joint_material);
    }

    const auto& attachments = body.attachments();
    for (const auto& a : attachments) {
        out.attachments.particle.push_back(a.particle);
        out.attachments.bone.push_back(a.bone);
        out.attachments.offset_x.push_back(a.local_offset.x);
        out.attachments.offset_y.push_back(a.local_offset.y);
        out.attachments.offset_z.push_back(a.local_offset.z);
        out.attachments.compliance.push_back(a.compliance);
        out.attachments.damage.push_back(a.damage);
        out.attachments.break_damage.push_back(a.break_damage);
        out.attachments.active.push_back(a.active ? 1u : 0u);
        out.attachments.material.push_back(a.material);
    }

    return out;
}

StepStats step_soa(
    BodySoA& body,
    double dt,
    const StepConfig& config) {

    validate_soa(body);

    SolverDomain domain;

    domain.particles.resize(body.particles.px.size());
    std::iota(domain.particles.begin(), domain.particles.end(), ParticleId{0});

    domain.structural.resize(body.structural.a.size());
    std::iota(domain.structural.begin(), domain.structural.end(), ConstraintId{0});

    domain.tetrahedral.resize(body.tetrahedral.a.size());
    std::iota(domain.tetrahedral.begin(), domain.tetrahedral.end(), ConstraintId{0});

    domain.attachments.resize(body.attachments.particle.size());
    std::iota(domain.attachments.begin(), domain.attachments.end(), ConstraintId{0});

    return step_soa_restricted(body, dt, domain, config);
}

StepStats step_soa_restricted(
    BodySoA& body,
    double dt,
    const SolverDomain& input_domain,
    const StepConfig& config) {

    validate_soa(body);

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
        body.particles.px.size(),
        "SoA restricted particle id out of range");
    domain.structural = canonicalize(
        input_domain.structural,
        body.structural.a.size(),
        "SoA restricted structural id out of range");
    domain.tetrahedral = canonicalize(
        input_domain.tetrahedral,
        body.tetrahedral.a.size(),
        "SoA restricted tetrahedral id out of range");
    domain.attachments = canonicalize(
        input_domain.attachments,
        body.attachments.particle.size(),
        "SoA restricted attachment id out of range");

    std::vector<std::uint8_t> active_particles(
        body.particles.px.size(),
        0u);
    for (const ParticleId id : domain.particles) {
        active_particles[id] = 1u;
    }

    StepStats stats;
    stats.active_particles = domain.particles.size();
    stats.structural_constraints = domain.structural.size();
    stats.tetrahedral_constraints = domain.tetrahedral.size();
    stats.attachment_constraints = domain.attachments.size();

    std::vector<double> structural_lambda(body.structural.a.size(), 0.0);
    std::vector<double> tetrahedral_lambda(body.tetrahedral.a.size(), 0.0);
    std::vector<Vec3> attachment_lambda(body.attachments.particle.size());

    const double h = dt / static_cast<double>(config.substeps);

    for (int substep = 0; substep < config.substeps; ++substep) {
        std::vector<Vec3> before(body.particles.px.size());

        for (const ParticleId id : domain.particles) {
            const Vec3 position = particle_position(body.particles, id);
            before[id] = position;

            if (body.particles.inverse_mass[id] == 0.0) {
                set_particle_velocity(body.particles, id, {});
                continue;
            }

            Vec3 velocity = particle_velocity(body.particles, id);
            velocity += config.gravity * h;
            set_particle_velocity(body.particles, id, velocity);
            set_particle_position(
                body.particles,
                id,
                position + velocity * h);
        }

        for (const ConstraintId id : domain.structural) {
            structural_lambda[id] = 0.0;
        }
        for (const ConstraintId id : domain.tetrahedral) {
            tetrahedral_lambda[id] = 0.0;
        }
        for (const ConstraintId id : domain.attachments) {
            attachment_lambda[id] = {};
        }

        for (int iteration = 0; iteration < config.solver_iterations; ++iteration) {
            for (const ConstraintId id : domain.structural) {
                if (!body.structural.active[id]) continue;

                const ParticleId ia = body.structural.a[id];
                const ParticleId ib = body.structural.b[id];

                const double wa = active_particles[ia]
                    ? body.particles.inverse_mass[ia]
                    : 0.0;
                const double wb = active_particles[ib]
                    ? body.particles.inverse_mass[ib]
                    : 0.0;
                if (wa + wb <= 1e-12) continue;

                Vec3 pa = particle_position(body.particles, ia);
                Vec3 pb = particle_position(body.particles, ib);
                const Vec3 delta = pb - pa;
                const double len = length(delta);
                if (len <= 1e-12) continue;

                const Vec3 n = delta / len;
                const double C = len - body.structural.rest_length[id];
                const double alpha =
                    body.structural.compliance[id] / (h * h);
                const double denom = wa + wb + alpha;
                if (denom <= 1e-12) continue;

                const double dlambda =
                    (-C - alpha * structural_lambda[id]) / denom;
                structural_lambda[id] += dlambda;

                if (wa > 0.0) {
                    pa += (-n) * (wa * dlambda);
                    set_particle_position(body.particles, ia, pa);
                }
                if (wb > 0.0) {
                    pb += n * (wb * dlambda);
                    set_particle_position(body.particles, ib, pb);
                }
            }

            for (const ConstraintId id : domain.tetrahedral) {
                if (!body.tetrahedral.active[id]) continue;

                const ParticleId i0 = body.tetrahedral.a[id];
                const ParticleId i1 = body.tetrahedral.b[id];
                const ParticleId i2 = body.tetrahedral.c[id];
                const ParticleId i3 = body.tetrahedral.d[id];

                const double w0 = active_particles[i0]
                    ? body.particles.inverse_mass[i0]
                    : 0.0;
                const double w1 = active_particles[i1]
                    ? body.particles.inverse_mass[i1]
                    : 0.0;
                const double w2 = active_particles[i2]
                    ? body.particles.inverse_mass[i2]
                    : 0.0;
                const double w3 = active_particles[i3]
                    ? body.particles.inverse_mass[i3]
                    : 0.0;
                if (w0 + w1 + w2 + w3 <= 1e-12) continue;

                Vec3 p0 = particle_position(body.particles, i0);
                Vec3 p1 = particle_position(body.particles, i1);
                Vec3 p2 = particle_position(body.particles, i2);
                Vec3 p3 = particle_position(body.particles, i3);

                const Vec3 e10 = p1 - p0;
                const Vec3 e20 = p2 - p0;
                const Vec3 e30 = p3 - p0;

                const double volume =
                    dot(e10, cross(e20, e30)) / 6.0;
                const double C =
                    volume - body.tetrahedral.rest_volume[id];

                const Vec3 g1 = cross(e20, e30) / 6.0;
                const Vec3 g2 = cross(e30, e10) / 6.0;
                const Vec3 g3 = cross(e10, e20) / 6.0;
                const Vec3 g0 = -(g1 + g2 + g3);

                const double weighted_gradient =
                    w0 * length_squared(g0)
                    + w1 * length_squared(g1)
                    + w2 * length_squared(g2)
                    + w3 * length_squared(g3);

                const double alpha =
                    body.tetrahedral.compliance[id] / (h * h);
                const double denom = weighted_gradient + alpha;
                if (denom <= 1e-12) continue;

                const double dlambda =
                    (-C - alpha * tetrahedral_lambda[id]) / denom;
                tetrahedral_lambda[id] += dlambda;

                if (w0 > 0.0) {
                    p0 += g0 * (w0 * dlambda);
                    set_particle_position(body.particles, i0, p0);
                }
                if (w1 > 0.0) {
                    p1 += g1 * (w1 * dlambda);
                    set_particle_position(body.particles, i1, p1);
                }
                if (w2 > 0.0) {
                    p2 += g2 * (w2 * dlambda);
                    set_particle_position(body.particles, i2, p2);
                }
                if (w3 > 0.0) {
                    p3 += g3 * (w3 * dlambda);
                    set_particle_position(body.particles, i3, p3);
                }
            }

            for (const ConstraintId id : domain.attachments) {
                if (!body.attachments.active[id]) continue;

                const ParticleId particle = body.attachments.particle[id];
                const BoneId bone = body.attachments.bone[id];

                if (!active_particles[particle]
                    || !bone_root_connected(body.bones, bone)) {
                    continue;
                }

                const double inverse_mass =
                    body.particles.inverse_mass[particle];
                if (inverse_mass == 0.0) continue;

                Vec3 position =
                    particle_position(body.particles, particle);
                const Vec3 target{
                    body.bones.px[bone]
                        + body.attachments.offset_x[id],
                    body.bones.py[bone]
                        + body.attachments.offset_y[id],
                    body.bones.pz[bone]
                        + body.attachments.offset_z[id]
                };

                const Vec3 C = position - target;
                const double alpha =
                    body.attachments.compliance[id] / (h * h);
                const double denom = inverse_mass + alpha;
                if (denom <= 1e-12) continue;

                const Vec3 dlambda =
                    (-C - attachment_lambda[id] * alpha) / denom;
                attachment_lambda[id] += dlambda;
                position += dlambda * inverse_mass;
                set_particle_position(
                    body.particles,
                    particle,
                    position);
            }

            stats.solver_constraint_visits +=
                domain.structural.size()
                + domain.tetrahedral.size()
                + domain.attachments.size();
        }

        for (const ParticleId id : domain.particles) {
            if (body.particles.inverse_mass[id] == 0.0) {
                continue;
            }

            const Vec3 position =
                particle_position(body.particles, id);
            set_particle_velocity(
                body.particles,
                id,
                (position - before[id]) / h);
        }
    }

    return stats;
}

} // namespace sarx
