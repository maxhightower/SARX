#include "sarx/soa.hpp"

namespace sarx {

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
        out.attachments.damage.push_back(a.damage);
        out.attachments.break_damage.push_back(a.break_damage);
        out.attachments.active.push_back(a.active ? 1u : 0u);
        out.attachments.material.push_back(a.material);
    }

    return out;
}

} // namespace sarx
