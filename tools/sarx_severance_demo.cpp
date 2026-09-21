#include "sarx/damage.hpp"
#include "sarx/debug_render.hpp"
#include "sarx/volume.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {

struct Args {
    std::filesystem::path output{"media/raw/v05_arm_severance_frames"};
    int frames{120};
    int cut_frame{45};
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
                << "sarx_severance_demo [--output DIR] [--frames N]"
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

void apply_severance_cut(
    sarx::Body& body,
    sarx::DamageSystem& damage) {

    const double ys[] = {0.075, 0.225};
    const double zs[] = {0.075, 0.225};

    for (const double y : ys) {
        sarx::CapsuleDamage blade;
        blade.a = {0.75, y, -0.30};
        blade.b = {0.75, y, 0.60};
        blade.radius = 0.11;
        blade.energy = 3.0;
        blade.mode = sarx::DamageMode::Cut;
        blade.cut_normal = {1.0, 0.0, 0.0};
        (void)damage.apply_capsule(body, blade);
    }

    for (const double z : zs) {
        sarx::CapsuleDamage blade;
        blade.a = {0.75, -0.30, z};
        blade.b = {0.75, 0.60, z};
        blade.radius = 0.11;
        blade.energy = 3.0;
        blade.mode = sarx::DamageMode::Cut;
        blade.cut_normal = {1.0, 0.0, 0.0};
        (void)damage.apply_capsule(body, blade);
    }
}

} // namespace

int main(int argc, char** argv) {
    try {
        const Args args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output);

        constexpr sarx::MaterialId tissue = 1;

        sarx::VoxelLatticeSpec spec;
        spec.origin = {0.0, 0.0, 0.0};
        spec.nx = 11;
        spec.ny = 3;
        spec.nz = 3;
        spec.spacing = 0.15;
        spec.particle_mass = 0.15;
        spec.default_material = tissue;
        spec.structural_compliance = 2e-6;
        spec.structural_break_damage = 1.0;
        spec.include_diagonals = true;
        spec.include_tetrahedra = true;
        spec.volume_compliance = 5e-7;
        spec.volume_break_damage = 1.0;

        auto lattice = sarx::build_voxel_lattice(spec);
        auto& body = lattice.body;

        const auto root = body.add_bone(
            sarx::kNoParent,
            {0.30, 0.15, 0.15},
            1.0,
            tissue,
            0.08);

        const auto distal = body.add_bone(
            root,
            {1.05, 0.15, 0.15},
            1.0,
            tissue,
            0.08);

        for (sarx::ParticleId id = 0;
             id < body.particles().size();
             ++id) {

            const auto position = body.particles()[id].position;

            if (position.x <= 0.60) {
                body.add_attachment(
                    id,
                    root,
                    position - body.bones()[root].animated_position,
                    1e-7,
                    1.0,
                    tissue);
            } else if (position.x >= 0.90) {
                body.add_attachment(
                    id,
                    distal,
                    position - body.bones()[distal].animated_position,
                    1e-7,
                    1.0,
                    tissue);
            }
        }

        sarx::DamageSystem damage;

        sarx::MaterialResponse tissue_response;
        tissue_response.cut_resistance = 0.75;
        tissue_response.blunt_resistance = 1.0;
        tissue_response.tensile_yield_strain = 0.25;
        tissue_response.tensile_break_strain = 0.80;
        tissue_response.strain_damage_rate = 0.25;
        damage.materials().set(tissue, tissue_response);

        sarx::StepConfig step;
        step.substeps = 3;
        step.solver_iterations = 10;
        step.gravity = {0.0, -3.0, 0.0};

        sarx::DebugCamera camera;
        camera.position = {2.8, 1.8, 3.1};
        camera.target = {0.75, 0.10, 0.15};
        camera.pixels_per_unit = 330.0;
        camera.width = 960;
        camera.height = 540;

        for (int frame = 0; frame < args.frames; ++frame) {
            if (frame < args.cut_frame) {
                const double phase =
                    static_cast<double>(frame)
                    / static_cast<double>(std::max(1, args.cut_frame));
                const double lift =
                    0.045 * std::sin(phase * 3.14159265358979323846 * 2.0);

                body.set_bone_target(
                    distal,
                    {1.05, 0.15 + lift, 0.15});
            }

            if (frame == args.cut_frame) {
                apply_severance_cut(body, damage);
            }

            body.step(1.0 / 60.0, step);

            sarx::write_debug_ppm(
                frame_path(args.output, frame),
                body,
                damage.wounds(),
                camera);
        }

        const auto islands = body.islands();
        std::cout
            << "SARX demo complete: frames=" << args.frames
            << " wounds=" << damage.wounds().size()
            << " islands=" << islands.size()
            << " output=" << args.output.string()
            << '\n';

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "sarx_severance_demo: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
