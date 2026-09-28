#include "sarx/anatomical_humanoid.hpp"
#include "sarx/voxel_skin.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>

using namespace sarx;

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::cerr << "FAIL: " << message << '\n';
    }
}

DamageCommand plane(Vec3 c, Vec3 n, double r) {
    DamageCommand d;
    d.kind = DamageCommandKind::PlaneCut;
    d.plane_cut.center = c;
    d.plane_cut.normal = normalized(n);
    d.plane_cut.radius = r;
    d.plane_cut.energy = 4.0;
    return d;
}

DamageCommand shot(Vec3 at, double radius) {
    DamageCommand d;
    d.kind = DamageCommandKind::Capsule;
    d.capsule.a = at + Vec3{0, 0, 4};
    d.capsule.b = at + Vec3{0, 0, -4};
    d.capsule.radius = radius;
    d.capsule.energy = 6.0;
    d.capsule.mode = DamageMode::Cut;
    return d;
}

void test_anatomical_body_is_connected_and_proportioned() {
    auto h = build_anatomical_humanoid_runtime();
    const auto& r = h.runtime;
    check(r.islands().size() == 1 && r.islands().front().rig_authoritative,
          "anatomical body is a single rig island");
    check(r.body().attachments().size() == r.body().particles().size(),
          "every anatomical particle is attached to a bone");
    check(r.islands().front().mass > 60.0 && r.islands().front().mass < 100.0,
          "anatomical body mass is human-scale");
    check(r.islands().front().bounds_max.y > 1.75 && r.islands().front().bounds_max.y < 1.85,
          "anatomical body is ~1.8 m tall");
    for (const auto& name : r.region_names()) {
        check(!r.region_particles(name).empty(), "anatomical region populated: " + name);
    }
}

void test_anatomical_landmark_cuts() {
    const auto& lm = anatomical_landmarks();
    auto h = build_anatomical_humanoid_runtime();
    auto& r = h.runtime;
    r.define_sphere_region("brain", lm.brain_center, lm.brain_radius);

    (void)r.apply_damage(plane(lm.right_shoulder_cut_center, lm.right_shoulder_cut_normal,
                               lm.shoulder_cut_radius));
    const auto hand = r.region_status("right_hand");
    check(r.islands().size() == 2, "shoulder cut creates an arm island");
    check(hand.islands.size() == 1 && hand.rig_connected_particles == 0,
          "severed arm is not rig connected");

    (void)r.apply_damage(plane(lm.neck_cut_center, {0, 1, 0}, lm.neck_cut_radius));
    const auto head = r.region_status("head");
    const auto brain = r.region_status("brain");
    check(head.islands.size() == 1 && head.rig_connected_particles == 0, "neck cut frees the head");
    check(brain.islands == head.islands && brain.integrity() == 1.0,
          "brain rides intact inside the severed head");

    auto h2 = build_anatomical_humanoid_runtime();
    h2.runtime.define_sphere_region("brain", lm.brain_center, lm.brain_radius);
    (void)h2.runtime.apply_damage(shot(lm.brain_center, 0.05));
    check(h2.runtime.region_status("brain").integrity() < 0.5, "5 cm shot through the brain destroys it");
    auto h3 = build_anatomical_humanoid_runtime();
    h3.runtime.define_sphere_region("brain", lm.brain_center, lm.brain_radius);
    (void)h3.runtime.apply_damage(shot(lm.brain_center + Vec3{0, -0.17, 0}, 0.05));
    check(h3.runtime.region_status("brain").integrity() == 1.0, "shot 17 cm lower spares the brain");
}

void test_detail_voxels() {
    const auto a = build_anatomical_detail_voxels(0.02);
    const auto b = build_anatomical_detail_voxels(0.02);
    check(a.voxels.size() > 5000, "2 cm detail set has thousands of voxels");
    check(a.voxels.size() == b.voxels.size(), "detail voxels are deterministic");
    const auto count = [&](DetailTissue t) {
        return std::count_if(a.voxels.begin(), a.voxels.end(), [&](const DetailVoxel& v) { return v.tissue == t; });
    };
    check(count(DetailTissue::Bone) > 300, "detail set contains skeleton voxels");
    check(count(DetailTissue::Neural) > 50, "detail set contains brain voxels");
}

void test_skin_follows_body_and_carves() {
    const Vec3 offset{1.0, 0.0, -2.0};
    auto h = build_anatomical_humanoid_runtime({}, offset);
    auto& r = h.runtime;
    EmbeddedVoxelSkin skin;
    skin.build(r, build_anatomical_detail_voxels(0.02), offset);

    double err = 0.0;
    for (std::size_t i = 0; i < skin.voxels().size(); ++i) {
        err = std::max(err, length(skin.world_position(r, i) - skin.voxels()[i].rest));
    }
    check(err < 1e-9, "skin reconstructs the rest pose exactly");
    check(skin.surface().size() > 1000 && skin.surface().size() < skin.voxels().size(),
          "only boundary voxels are surface voxels");

    for (int i = 0; i < 40; ++i) {
        for (BoneId b = 0; b < h.rest_bone_positions.size(); ++b) {
            r.body().set_bone_target(b, h.rest_bone_positions[b] + Vec3{0.0, 0.0, 0.01 * (i + 1)});
        }
        r.step(1.0 / 60.0);
    }
    skin.update(r);
    double drift = 0.0;
    for (const auto index : skin.surface()) {
        const auto& v = skin.voxels()[index];
        drift = std::max(drift, length(v.center - (v.rest + Vec3{0.0, 0.0, 0.40})));
    }
    check(drift < 0.06, "skin follows the driven body");
    bool right_handed = true;
    for (const auto index : skin.surface()) {
        const auto& a = skin.voxels()[index].axes;
        right_handed = right_handed && dot(a[0], cross(a[1], a[2])) > 0.0;
    }
    check(right_handed, "drawn voxel bases are right-handed");

    const auto& lm = anatomical_landmarks();
    const auto cut = plane(lm.right_shoulder_cut_center + offset + Vec3{0, 0, 0.40},
                           lm.right_shoulder_cut_normal, lm.shoulder_cut_radius);
    (void)r.apply_damage(cut);
    const std::size_t carved = skin.carve(r, cut);
    skin.refresh_topology(r);
    check(carved > 10, "plane cut carves a kerf through the skin");
    const auto exposed = std::count_if(skin.voxels().begin(), skin.voxels().end(),
                                       [](const SkinVoxel& v) { return v.exposed; });
    check(exposed > 20, "cut exposes wound voxels");

    const IslandId arm = r.region_status("right_hand").islands.front();
    std::size_t arm_voxels = 0;
    for (std::size_t i = 0; i < skin.voxels().size(); ++i) {
        if (skin.voxels()[i].alive && skin.island_of(r, i) == arm) ++arm_voxels;
    }
    check(arm_voxels > 300, "detail voxels travel with the severed arm island");

    const auto headshot = shot(lm.brain_center + offset + Vec3{0, 0, 0.40}, 0.05);
    (void)r.apply_damage(headshot);
    check(skin.carve(r, headshot) > 5, "gunshot carves a hole");
    skin.refresh_topology(r);
    const auto exposed_brain = std::count_if(skin.voxels().begin(), skin.voxels().end(), [](const SkinVoxel& v) {
        return v.alive && v.exposed && v.tissue == DetailTissue::Neural;
    });
    check(exposed_brain > 0, "headshot exposes brain tissue");
}

} // namespace

int main() {
    test_anatomical_body_is_connected_and_proportioned();
    test_anatomical_landmark_cuts();
    test_detail_voxels();
    test_skin_follows_body_and_carves();
    if (failures > 0) {
        std::cerr << failures << " anatomical/skin test failure(s)\n";
        return 1;
    }
    std::cout << "sarx anatomical/skin tests passed\n";
    return 0;
}
