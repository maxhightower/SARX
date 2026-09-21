#include "sarx/adaptive.hpp"
#include "sarx/damage.hpp"
#include "sarx/debug_render.hpp"
#include "sarx/humanoid.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

struct Args {
    std::filesystem::path output{"media/raw/v04d_humanoid_frames"};
    int frames{150};
    int cut_frame{60};
};

Args parse_args(int argc, char** argv) {
    Args args;

    for (int i = 1; i < argc; ++i) {
        const std::string value = argv[i];

        if (value == "--output" && i + 1 < argc) {
            args.output = argv[++i];
        } else if (value == "--frames" && i + 1 < argc) {
            args.frames = std::stoi(argv[++i]);
        } else if (value == "--cut-frame" && i + 1 < argc) {
            args.cut_frame = std::stoi(argv[++i]);
        } else if (value == "--help") {
            std::cout
                << "sarx_humanoid_demo [--output DIR] [--frames N]"
                << " [--cut-frame N]\n";
            std::exit(EXIT_SUCCESS);
        } else {
            throw std::invalid_argument("unknown or incomplete demo argument");
        }
    }

    if (args.frames <= 0) {
        throw std::invalid_argument("frames must be positive");
    }
    if (args.cut_frame < 0 || args.cut_frame >= args.frames) {
        throw std::invalid_argument("cut-frame must be within frame range");
    }

    return args;
}

std::string frame_path(
    const std::filesystem::path& directory,
    int frame) {

    std::ostringstream name;
    name << "frame_"
         << std::setw(4)
         << std::setfill('0')
         << frame
         << ".ppm";
    return (directory / name.str()).string();
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output);

        sarx::HumanoidSpec spec;
        auto fixture = sarx::build_humanoid_fixture(spec);
        auto& body = fixture.body;

        sarx::DamageSystem damage;

        sarx::MaterialResponse tissue;
        tissue.cut_resistance = 0.75;
        tissue.blunt_resistance = 1.0;
        tissue.tensile_yield_strain = 0.30;
        tissue.tensile_break_strain = 0.90;
        tissue.strain_damage_rate = 0.20;
        damage.materials().set(spec.tissue_material, tissue);

        sarx::StepConfig step;
        step.substeps = 3;
        step.solver_iterations = 10;
        step.gravity = {0.0, -0.40, 0.0};

        sarx::DebugCamera camera;
        camera.position = {2.8, 1.55, 4.5};
        camera.target = {0.0, 0.92, 0.0};
        camera.pixels_per_unit = 235.0;
        camera.width = 960;
        camera.height = 720;

        sarx::DebugRenderOptions render;
        render.structural = true;
        render.tetrahedral = false;
        render.particles = true;
        render.bones = true;
        render.wounds = true;

        sarx::AdaptiveDomainTracker tracker;
        bool adaptive = false;
        std::size_t restricted_visits = 0;
        std::size_t full_equivalent_visits = 0;

        for (int frame = 0; frame < args.frames; ++frame) {
            if (frame < args.cut_frame) {
                const double phase =
                    static_cast<double>(frame)
                    / static_cast<double>(std::max(1, args.cut_frame));

                const double wave =
                    std::sin(
                        phase
                        * 3.14159265358979323846
                        * 2.0);

                body.set_bone_target(
                    fixture.bones.right_shoulder,
                    fixture.right_shoulder_rest
                        + sarx::Vec3{0.0, 0.03 * wave, 0.0});

                body.set_bone_target(
                    fixture.bones.right_elbow,
                    fixture.right_elbow_rest
                        + sarx::Vec3{0.0, 0.08 * wave, 0.0});

                body.set_bone_target(
                    fixture.bones.right_hand,
                    fixture.right_hand_rest
                        + sarx::Vec3{0.0, 0.14 * wave, 0.0});
            }

            if (frame == args.cut_frame) {
                sarx::PlaneCutDamage cut;
                cut.center = fixture.right_shoulder_cut_center;
                cut.normal = fixture.right_shoulder_cut_normal;
                cut.radius = fixture.right_shoulder_cut_radius;
                cut.energy = 3.0;

                (void)damage.apply_plane_cut(body, cut);

                tracker.reset(body);
                for (const auto& wound : damage.wounds()) {
                    tracker.upsert_wound(body, wound, 0.20);
                }

                adaptive = true;
            }

            if (!adaptive) {
                body.step(1.0 / 60.0, step);
            } else {
                const auto seed = tracker.combined_damage_domain();
                const auto active =
                    sarx::close_over_free_islands(body, seed);
                const auto domain = sarx::solver_domain(active);

                const auto stats =
                    body.step_restricted(
                        1.0 / 60.0,
                        domain,
                        step);

                restricted_visits +=
                    stats.solver_constraint_visits;

                full_equivalent_visits +=
                    static_cast<std::size_t>(step.substeps)
                    * static_cast<std::size_t>(step.solver_iterations)
                    * (body.structural_constraints().size()
                       + body.tetrahedral_constraints().size()
                       + body.attachments().size());
            }

            sarx::write_debug_ppm(
                frame_path(args.output, frame),
                body,
                damage.wounds(),
                camera,
                render);
        }

        const auto islands = body.islands();

        std::cout
            << "SARX humanoid demo complete: frames=" << args.frames
            << " particles=" << body.particles().size()
            << " wounds=" << damage.wounds().size()
            << " islands=" << islands.size()
            << " restricted_visits=" << restricted_visits
            << " full_equivalent_visits=" << full_equivalent_visits;

        if (full_equivalent_visits > 0) {
            const double ratio =
                static_cast<double>(restricted_visits)
                / static_cast<double>(full_equivalent_visits);
            std::cout << " adaptive_visit_ratio=" << ratio;
        }

        std::cout
            << " output=" << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr
            << "sarx_humanoid_demo: "
            << error.what()
            << '\n';
        return EXIT_FAILURE;
    }
}
