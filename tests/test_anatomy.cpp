// Behavioural tests for the layered anatomical voxel body (V0.5).

#include "sarx/anatomy.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

AnatomyDesc block(int nx, int ny, int nz, Tissue tissue, Vec3 origin = {}) {
    AnatomyDesc d;
    d.origin = origin;
    d.voxel_size = 0.02;
    for (int z = 0; z < nz; ++z)
        for (int y = 0; y < ny; ++y)
            for (int x = 0; x < nx; ++x) {
                AnatomyVoxelDesc v;
                v.cell = {x, y, z};
                v.tissue = tissue;
                d.voxels.push_back(v);
            }
    return d;
}

// Bar along x: muscle sheath around an optional bone core, no rig.
AnatomyDesc limb_bar(bool bone_core) {
    AnatomyDesc d;
    d.voxel_size = 0.02;
    d.origin = {0.0, 0.5, 0.0};
    for (int z = 0; z < 5; ++z)
        for (int y = 0; y < 5; ++y)
            for (int x = 0; x < 20; ++x) {
                AnatomyVoxelDesc v;
                v.cell = {x, y, z};
                const bool core = y >= 1 && y <= 3 && z >= 1 && z <= 3;
                v.tissue = bone_core && core ? Tissue::Bone : Tissue::Muscle;
                d.voxels.push_back(v);
            }
    return d;
}

AnatomyStepConfig zero_g() {
    AnatomyStepConfig c;
    c.gravity = {};
    c.ground_height = -100.0;
    return c;
}

double max_speed(const AnatomyBody& b) {
    double v = 0.0;
    for (std::uint32_t i = 0; i < b.voxel_count(); ++i)
        if (b.voxel_alive(i)) v = std::max(v, length(b.voxel_velocity(i)));
    return v;
}

// Vertical cut plane crossing the bar at x = 0.201 (between voxel columns),
// swept along -z so bonds are reached front to back.
BladeStroke bar_chop(double energy) {
    BladeStroke s;
    s.energy = energy;
    for (int i = 0; i <= 6; ++i) {
        const double z = 0.14 - 0.18 * i / 6.0;
        s.poses.push_back({{0.201, 0.70, z}, {0.201, 0.40, z}});
    }
    return s;
}

void test_humanoid_layers_are_anatomical() {
    const AnatomyDesc desc = build_humanoid_anatomy({});
    AnatomyBody body(desc);
    const auto st = body.stats();
    for (std::size_t t = 0; t < kTissueCount; ++t)
        check(st.voxels_by_tissue[t] > 0, std::string("humanoid should contain tissue ") + tissue_name(static_cast<Tissue>(t)));
    const double mass = body.total_mass();
    check(mass > 55.0 && mass < 95.0, "humanoid mass should be adult-like, got " + std::to_string(mass));

    // Every externally exposed face belongs to skin.
    std::size_t exposed = 0, exposed_skin = 0;
    for (std::uint32_t v = 0; v < body.voxel_count(); ++v)
        for (int a = 0; a < 3; ++a)
            for (int d = 0; d < 2; ++d)
                if (body.neighbor(v, a, d) == UINT32_MAX) {
                    ++exposed;
                    exposed_skin += body.voxel_tissue(v) == Tissue::Skin;
                }
    check(exposed > 0 && exposed_skin == exposed, "the outer surface should be entirely skin");

    const auto& comps = body.components();
    check(comps.size() == 1, "intact humanoid should be one component");
    check(!comps.empty() && comps[0].rig_authoritative, "intact humanoid should hold rig authority");
}

void test_resting_block_is_stable_and_holds_shape() {
    AnatomyBody body(block(8, 6, 8, Tissue::Muscle));
    AnatomyStepConfig cfg;
    cfg.sleep_speed = 0.0;
    for (int f = 0; f < 180; ++f) body.step(1.0 / 60.0, cfg);
    double top = 0.0;
    for (std::uint32_t v = 0; v < body.voxel_count(); ++v) top = std::max(top, body.voxel_center(v).y);
    check(max_speed(body) < 0.05, "a resting block must not gain energy");
    check(top > 0.85 * 0.11, "a resting 6-voxel block should keep >85% of its height, got " + std::to_string(top));
    check(body.stats().live_bonds == body.stats().bonds, "a resting block must not tear");
}

void test_blade_lodges_and_later_strokes_continue_the_wound() {
    AnatomyBody body(limb_bar(false));
    // 25 bonds cross the plane; muscle bonds cost 2.5 J each.
    const auto first = body.apply_blade(bar_chop(21.0));
    check(first.lodged, "a 21 J stroke should lodge in a 25-bond muscle bar");
    check(first.bonds_parted == 8, "21 J should part 8 muscle bonds (2.5 J each)");
    check(first.bonds_damaged == 1, "the bond where the blade stops keeps partial damage");
    check(body.components().size() == 1, "a lodged blade must not sever the bar");

    // Find the partially damaged bond: its HP persists between strokes.
    bool partial = false;
    for (std::size_t b = 0; b < body.bond_count(); ++b)
        partial |= body.bond_active(b) && body.bond_hp(b) < body.bond_max_hp(b) - 1e-4f;
    check(partial, "partial bond damage should persist after a lodged stroke");

    const auto second = body.apply_blade(bar_chop(21.0));
    check(second.bonds_parted >= 8, "the second stroke continues the same wound");
    const auto third = body.apply_blade(bar_chop(21.0));
    const auto fourth = body.apply_blade(bar_chop(21.0));
    check(!fourth.lodged || third.bonds_parted + fourth.bonds_parted > 0, "later strokes keep cutting");
    check(body.components().size() == 2, "four 21 J strokes should hack through the bar");
}

void test_bone_resists_blades() {
    AnatomyBody soft(limb_bar(false));
    AnatomyBody boned(limb_bar(true));
    const auto a = soft.apply_blade(bar_chop(70.0));
    const auto b = boned.apply_blade(bar_chop(70.0));
    check(soft.components().size() == 2, "70 J severs an all-muscle bar");
    check(boned.components().size() == 1, "70 J does not sever the same bar with a bone core");
    check(b.lodged && b.stopped_in == Tissue::Bone, "the blade should lodge in the bone core");
    check(a.bonds_parted > b.bonds_parted, "bone should absorb the stroke's energy");
}

void test_severed_limb_loses_rig_authority_and_falls() {
    AnatomyBody body(build_humanoid_anatomy({}));
    body.set_pose(humanoid_anatomy_pose(0.0, 0.0));
    body.snap_to_pose();
    AnatomyStepConfig cfg;
    for (int f = 0; f < 5; ++f) body.step(1.0 / 60.0, cfg);
    check(body.dynamic_voxel_count() == 0, "intact rig-driven body should be fully kinematic in hybrid mode");

    // Horizontal chops through the right upper arm (outside -> in).
    for (int k = 0; k < 4; ++k) {
        BladeStroke s;
        s.energy = 120.0;
        for (int i = 0; i <= 8; ++i) {
            const double x = 0.52 - 0.345 * i / 8.0;
            s.poses.push_back({{x, 1.305, 0.35}, {x, 1.305, -0.35}});
        }
        (void)body.apply_blade(s);
    }
    const auto& comps = body.components();
    std::size_t free_islands = 0, authoritative = 0;
    std::uint32_t hand = UINT32_MAX;
    for (std::uint32_t v = 0; v < body.voxel_count(); ++v) {
        const Vec3 c = body.voxel_rest_center(v);
        if (c.x > 0.33 && c.y < 0.90 && c.y > 0.80 && body.voxel_tissue(v) == Tissue::Skin) { hand = v; break; }
    }
    for (const auto& c : comps) {
        free_islands += c.rig_authoritative ? 0 : 1;
        authoritative += c.rig_authoritative ? 1 : 0;
    }
    check(free_islands >= 1, "severing the upper arm should create a free island");
    check(authoritative == 1, "exactly one component keeps rig authority");
    check(hand != UINT32_MAX && body.voxel_dynamic(hand), "the severed hand should be simulated");
    const double y0 = hand != UINT32_MAX ? body.voxel_center(hand).y : 0.0;
    const std::size_t dynamic = body.dynamic_voxel_count();
    check(dynamic < body.voxel_count() / 4, "damage activation should stay local");
    for (int f = 0; f < 20; ++f) body.step(1.0 / 60.0, cfg);
    check(hand != UINT32_MAX && body.voxel_center(hand).y < y0 - 0.1, "the free arm should fall under gravity");
}

void test_bullets_penetrate_lodge_and_carry_momentum() {
    // Soft tissue: a 9 mm round passes through 10 cm of muscle.
    AnatomyBody soft(block(5, 5, 5, Tissue::Muscle));
    BulletShot shot;
    shot.origin = {0.05, 0.05, 1.0};
    shot.direction = {0, 0, -1};
    const Vec3 p0 = soft.total_linear_momentum();
    const auto r = soft.apply_bullet(shot);
    check(r.entered && r.exited && !r.lodged, "pistol round should pass through 10 cm of muscle");
    check(r.energy_out < r.energy_in && r.energy_out > 0.3 * r.energy_in, "exit energy should be reduced but substantial");
    check(r.voxels_destroyed >= 5, "the round should tunnel a wound channel");
    check(!r.debris.empty(), "tunnelling should eject debris");
    const Vec3 dp = soft.total_linear_momentum() - p0;
    const double lost = shot.mass * (shot.speed - length(r.exit_velocity));
    // Destroyed voxels leave the body; the rest of the momentum is deposited.
    check(dp.z < 0.0 && std::abs(dp.z) <= lost * 1.01 && std::abs(dp.z) > 0.3 * lost,
          "deposited momentum should be a bounded share of what the round lost");

    // Bone: a pistol round lodges in 10 cm of bone, a rifle round does not.
    AnatomyBody bone_a(block(5, 5, 5, Tissue::Bone));
    const auto pistol = bone_a.apply_bullet(shot);
    check(pistol.lodged && !pistol.exited, "pistol round should lodge in 10 cm of bone");
    AnatomyBody bone_b(block(5, 5, 3, Tissue::Bone));
    BulletShot rifle = shot;
    rifle.speed = 940.0;
    rifle.mass = 0.004;
    const auto rr = bone_b.apply_bullet(rifle);
    check(rr.exited, "rifle round should pass through 6 cm of bone");
    check(bone_b.components().size() >= 1 && rr.bonds_broken > rr.voxels_destroyed, "bone hits should fracture surrounding bone");

    // A miss leaves the round untouched for the world.
    AnatomyBody miss(block(3, 3, 3, Tissue::Muscle));
    BulletShot wide = shot;
    wide.origin = {1.0, 1.0, 1.0};
    const auto m = miss.apply_bullet(wide);
    check(!m.entered && std::abs(m.energy_out - m.energy_in) < 1e-9, "a miss keeps all of its energy");
}

std::uint64_t scripted_run(unsigned threads) {
    AnatomyBody body(build_humanoid_anatomy({}));
    body.set_pose(humanoid_anatomy_pose(0.0, 0.5));
    body.snap_to_pose();
    AnatomyStepConfig cfg;
    cfg.threads = threads;
    for (int f = 0; f < 40; ++f) {
        body.set_pose(humanoid_anatomy_pose(f / 60.0 * 4.0, 0.5));
        if (f == 10) {
            BladeStroke s;
            s.energy = 150.0;
            for (int i = 0; i <= 6; ++i) {
                const double z = 0.4 - 0.8 * i / 6.0;
                s.poses.push_back({{0.6, 1.045, z}, {-0.4, 1.045, z}});
            }
            (void)body.apply_blade(s);
        }
        if (f == 20) {
            BulletShot b;
            b.origin = {0.05, 1.3, 3.0};
            (void)body.apply_bullet(b);
        }
        body.step(1.0 / 60.0, cfg);
    }
    return body.state_hash();
}

void test_determinism_across_runs_and_threads() {
    const auto a = scripted_run(1);
    const auto b = scripted_run(1);
    const auto c = scripted_run(4);
    check(a == b, "identical scripted runs must produce identical state");
    check(a == c, "thread count must not change the result (conflict-free partitions)");
}

void test_pulling_apart_tears_tissue() {
    AnatomyBody body(block(4, 20, 4, Tissue::Muscle, {0.0, 0.5, 0.0}));
    const auto top = body.add_grab({0.04, 0.5 + 0.37, 0.04}, 0.05, 0.3);
    const auto bottom = body.add_grab({0.04, 0.5 + 0.03, 0.04}, 0.05, 0.3);
    AnatomyStepConfig cfg = zero_g();
    std::size_t tears = 0;
    for (int f = 0; f < 120; ++f) {
        const double d = std::min(0.4, f / 60.0 * 0.4);
        body.set_grab_target(top, {0.04, 0.87 + d, 0.04});
        body.set_grab_target(bottom, {0.04, 0.53 - d, 0.04});
        body.step(1.0 / 60.0, cfg);
        tears += body.recent_tears().size();
    }
    check(tears > 0, "overstretched tissue should tear");
    check(body.components().size() >= 2, "pulling a bar apart should split it");
}

void test_settled_fragments_sleep_and_wake() {
    AnatomyBody body(block(4, 4, 4, Tissue::Muscle, {0.0, 0.2, 0.0}));
    AnatomyStepConfig cfg;
    for (int f = 0; f < 200; ++f) body.step(1.0 / 60.0, cfg);
    check(body.voxel_asleep(0), "a settled free fragment should fall asleep");
    check(body.dynamic_voxel_count() == 0, "sleeping fragments cost nothing");
    BulletShot b;
    b.origin = {0.04, 0.03, 1.0};
    b.direction = {0, 0, -1};
    (void)body.apply_bullet(b);
    check(body.dynamic_voxel_count() > 0, "damage should wake a sleeping fragment");
}

} // namespace

int main() {
    test_humanoid_layers_are_anatomical();
    test_resting_block_is_stable_and_holds_shape();
    test_blade_lodges_and_later_strokes_continue_the_wound();
    test_bone_resists_blades();
    test_severed_limb_loses_rig_authority_and_falls();
    test_bullets_penetrate_lodge_and_carry_momentum();
    test_determinism_across_runs_and_threads();
    test_pulling_apart_tears_tissue();
    test_settled_fragments_sleep_and_wake();

    if (failures != 0) {
        std::cerr << failures << " anatomy test(s) failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "SARX V0.5 anatomy tests passed.\n";
    return EXIT_SUCCESS;
}
