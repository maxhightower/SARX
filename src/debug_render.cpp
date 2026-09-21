#include "sarx/debug_render.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace sarx {
namespace {

struct Color {
    std::uint8_t r{};
    std::uint8_t g{};
    std::uint8_t b{};
};

struct ScreenPoint {
    double x{};
    double y{};
    double depth{};
    bool valid{false};
};

class Canvas {
public:
    Canvas(std::size_t width, std::size_t height)
        : width_(width),
          height_(height),
          pixels_(width * height * 3, 248u) {}

    void set(int x, int y, Color c) {
        if (x < 0 || y < 0
            || x >= static_cast<int>(width_)
            || y >= static_cast<int>(height_)) {
            return;
        }

        const auto index =
            (static_cast<std::size_t>(y) * width_
             + static_cast<std::size_t>(x)) * 3;
        pixels_[index + 0] = c.r;
        pixels_[index + 1] = c.g;
        pixels_[index + 2] = c.b;
    }

    void line(
        double x0,
        double y0,
        double x1,
        double y1,
        Color c,
        int thickness = 1) {

        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const int steps = std::max(
            1,
            static_cast<int>(std::ceil(
                std::max(std::abs(dx), std::abs(dy)))));

        for (int i = 0; i <= steps; ++i) {
            const double t =
                static_cast<double>(i) / static_cast<double>(steps);
            const int x = static_cast<int>(std::lround(x0 + dx * t));
            const int y = static_cast<int>(std::lround(y0 + dy * t));
            disc(x, y, thickness, c);
        }
    }

    void disc(int cx, int cy, int radius, Color c) {
        const int r = std::max(1, radius);
        for (int y = -r; y <= r; ++y) {
            for (int x = -r; x <= r; ++x) {
                if (x * x + y * y <= r * r) {
                    set(cx + x, cy + y, c);
                }
            }
        }
    }

    void ring(int cx, int cy, int radius, Color c) {
        const int r = std::max(2, radius);
        const int inner = std::max(0, r - 2);
        for (int y = -r; y <= r; ++y) {
            for (int x = -r; x <= r; ++x) {
                const int d2 = x * x + y * y;
                if (d2 <= r * r && d2 >= inner * inner) {
                    set(cx + x, cy + y, c);
                }
            }
        }
    }

    void write(const std::string& path) const {
        std::ofstream out(path, std::ios::binary);
        if (!out) {
            throw std::runtime_error("failed to open debug PPM output");
        }
        out << "P6\n" << width_ << ' ' << height_ << "\n255\n";
        out.write(
            reinterpret_cast<const char*>(pixels_.data()),
            static_cast<std::streamsize>(pixels_.size()));
        if (!out) {
            throw std::runtime_error("failed to write debug PPM output");
        }
    }

private:
    std::size_t width_{};
    std::size_t height_{};
    std::vector<std::uint8_t> pixels_;
};

struct Projector {
    Vec3 position{};
    Vec3 right{};
    Vec3 up{};
    Vec3 forward{};
    double pixels_per_unit{};
    double cx{};
    double cy{};

    ScreenPoint project(const Vec3& point) const {
        const Vec3 relative = point - position;
        const double depth = dot(relative, forward);

        const Vec3 from_target_plane =
            point - (position + forward * depth);

        return {
            cx + dot(from_target_plane, right) * pixels_per_unit,
            cy - dot(from_target_plane, up) * pixels_per_unit,
            depth,
            depth > 0.0
        };
    }
};

Projector projector_for(const DebugCamera& camera) {
    const Vec3 forward = normalized(camera.target - camera.position);
    if (length_squared(forward) <= 1e-12) {
        throw std::invalid_argument("debug camera position equals target");
    }

    Vec3 right = normalized(cross(forward, camera.world_up));
    if (length_squared(right) <= 1e-12) {
        right = normalized(cross(forward, Vec3{0.0, 0.0, 1.0}));
    }
    const Vec3 up = normalized(cross(right, forward));

    return {
        camera.position,
        right,
        up,
        forward,
        camera.pixels_per_unit,
        static_cast<double>(camera.width) * 0.5,
        static_cast<double>(camera.height) * 0.5
    };
}

void draw_segment(
    Canvas& canvas,
    const Projector& projector,
    const Vec3& a,
    const Vec3& b,
    Color color,
    int thickness = 1) {

    const auto pa = projector.project(a);
    const auto pb = projector.project(b);
    if (!pa.valid || !pb.valid) return;

    canvas.line(
        pa.x,
        pa.y,
        pb.x,
        pb.y,
        color,
        thickness);
}

} // namespace

void write_debug_ppm(
    const std::string& path,
    const Body& body,
    const std::vector<WoundDescriptor>& wounds,
    const DebugCamera& camera,
    const DebugRenderOptions& options) {

    if (camera.width == 0 || camera.height == 0
        || camera.pixels_per_unit <= 0.0) {
        throw std::invalid_argument("invalid debug camera dimensions");
    }

    Canvas canvas(camera.width, camera.height);
    const Projector projector = projector_for(camera);

    if (options.tetrahedral) {
        const Color active{210, 214, 220};
        const Color broken{236, 190, 190};

        for (const auto& t : body.tetrahedral_constraints()) {
            const Color color = t.active ? active : broken;

            const Vec3 p0 = body.particles()[t.a].position;
            const Vec3 p1 = body.particles()[t.b].position;
            const Vec3 p2 = body.particles()[t.c].position;
            const Vec3 p3 = body.particles()[t.d].position;

            draw_segment(canvas, projector, p0, p1, color);
            draw_segment(canvas, projector, p0, p2, color);
            draw_segment(canvas, projector, p0, p3, color);
            draw_segment(canvas, projector, p1, p2, color);
            draw_segment(canvas, projector, p1, p3, color);
            draw_segment(canvas, projector, p2, p3, color);
        }
    }

    if (options.structural) {
        for (const auto& constraint : body.structural_constraints()) {
            Color color{72, 78, 86};
            if (!constraint.active) {
                color = {205, 75, 75};
            } else if (constraint.damage > 0.0) {
                color = {196, 126, 45};
            }

            draw_segment(
                canvas,
                projector,
                body.particles()[constraint.a].position,
                body.particles()[constraint.b].position,
                color,
                constraint.active ? 1 : 2);
        }
    }

    if (options.bones) {
        for (BoneId id = 0; id < body.bones().size(); ++id) {
            const auto& bone = body.bones()[id];
            if (bone.parent == kNoParent) {
                const auto point = projector.project(bone.animated_position);
                if (point.valid) {
                    canvas.disc(
                        static_cast<int>(std::lround(point.x)),
                        static_cast<int>(std::lround(point.y)),
                        5,
                        {45, 104, 196});
                }
                continue;
            }

            const Color color = bone.joint_to_parent_active
                ? Color{45, 104, 196}
                : Color{210, 55, 55};

            draw_segment(
                canvas,
                projector,
                body.bones()[bone.parent].animated_position,
                bone.animated_position,
                color,
                3);
        }
    }

    if (options.particles) {
        for (const auto& particle : body.particles()) {
            const auto point = projector.project(particle.position);
            if (!point.valid) continue;

            canvas.disc(
                static_cast<int>(std::lround(point.x)),
                static_cast<int>(std::lround(point.y)),
                2,
                {28, 31, 36});
        }
    }

    if (options.wounds) {
        for (const auto& wound : wounds) {
            const auto point = projector.project(wound.center);
            if (!point.valid) continue;

            const int radius = std::max(
                4,
                static_cast<int>(std::lround(
                    wound.radius * camera.pixels_per_unit)));

            canvas.ring(
                static_cast<int>(std::lround(point.x)),
                static_cast<int>(std::lround(point.y)),
                radius,
                {190, 35, 125});
        }
    }

    canvas.write(path);
}

} // namespace sarx
