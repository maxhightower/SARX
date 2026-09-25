#include "sarx/character_render.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace sarx {
namespace {

struct Color {
    std::uint8_t r{};
    std::uint8_t g{};
    std::uint8_t b{};
};

struct Projected {
    double x{};
    double y{};
    double z{};
    bool valid{false};
};

class Canvas {
public:
    Canvas(std::size_t width, std::size_t height)
        : width_(width),
          height_(height),
          pixels_(width * height * 3, 246u),
          depth_(
              width * height,
              std::numeric_limits<double>::infinity()) {}

    void set(int x, int y, double depth, Color color) {
        if (x < 0 || y < 0
            || x >= static_cast<int>(width_)
            || y >= static_cast<int>(height_)) {
            return;
        }

        const std::size_t pixel =
            static_cast<std::size_t>(y) * width_
            + static_cast<std::size_t>(x);

        if (depth >= depth_[pixel]) {
            return;
        }

        depth_[pixel] = depth;

        const std::size_t index = pixel * 3;
        pixels_[index + 0] = color.r;
        pixels_[index + 1] = color.g;
        pixels_[index + 2] = color.b;
    }

    void line(
        double x0,
        double y0,
        double z0,
        double x1,
        double y1,
        double z1,
        Color color) {

        const double dx = x1 - x0;
        const double dy = y1 - y0;

        const int steps = std::max(
            1,
            static_cast<int>(
                std::ceil(
                    std::max(
                        std::abs(dx),
                        std::abs(dy)))));

        for (int i = 0; i <= steps; ++i) {
            const double t =
                static_cast<double>(i)
                / static_cast<double>(steps);

            const int x =
                static_cast<int>(
                    std::lround(x0 + dx * t));
            const int y =
                static_cast<int>(
                    std::lround(y0 + dy * t));
            const double z =
                z0 + (z1 - z0) * t;

            set(x, y, z, color);
        }
    }

    void write(const std::string& path) const {
        std::ofstream out(path, std::ios::binary);
        if (!out) {
            throw std::runtime_error(
                "failed to open character render output");
        }

        out << "P6\n"
            << width_ << ' '
            << height_ << "\n255\n";

        out.write(
            reinterpret_cast<const char*>(
                pixels_.data()),
            static_cast<std::streamsize>(
                pixels_.size()));

        if (!out) {
            throw std::runtime_error(
                "failed to write character render output");
        }
    }

private:
    std::size_t width_{};
    std::size_t height_{};

    std::vector<std::uint8_t> pixels_;
    std::vector<double> depth_;
};

struct Projector {
    Vec3 position{};
    Vec3 right{};
    Vec3 up{};
    Vec3 forward{};

    double focal{};
    double cx{};
    double cy{};

    Projected project(const Vec3& point) const {
        const Vec3 relative = point - position;

        const double z =
            dot(relative, forward);

        if (z <= 0.02) {
            return {};
        }

        const double x =
            dot(relative, right);
        const double y =
            dot(relative, up);

        return {
            cx + x * focal / z,
            cy - y * focal / z,
            z,
            true
        };
    }
};

Projector projector_for(
    const CharacterRenderCamera& camera) {

    if (camera.width == 0
        || camera.height == 0
        || camera.vertical_fov_degrees <= 1.0
        || camera.vertical_fov_degrees >= 170.0) {
        throw std::invalid_argument(
            "invalid character render camera");
    }

    const Vec3 forward =
        normalized(
            camera.target - camera.position);

    if (length_squared(forward) <= 1e-12) {
        throw std::invalid_argument(
            "character camera position equals target");
    }

    Vec3 right =
        normalized(
            cross(
                forward,
                camera.world_up));

    if (length_squared(right) <= 1e-12) {
        right =
            normalized(
                cross(
                    forward,
                    Vec3{0.0, 0.0, 1.0}));
    }

    const Vec3 up =
        normalized(
            cross(
                right,
                forward));

    const double radians =
        camera.vertical_fov_degrees
        * 3.14159265358979323846
        / 180.0;

    const double focal =
        (static_cast<double>(camera.height) * 0.5)
        / std::tan(radians * 0.5);

    return {
        camera.position,
        right,
        up,
        forward,
        focal,
        static_cast<double>(camera.width) * 0.5,
        static_cast<double>(camera.height) * 0.5
    };
}

double edge(
    double ax,
    double ay,
    double bx,
    double by,
    double px,
    double py) {

    return
        (px - ax) * (by - ay)
        - (py - ay) * (bx - ax);
}

Color shaded_character_color(
    const Vec3& normal) {

    const Vec3 light =
        normalized(
            Vec3{-0.35, 0.80, 0.50});

    const double amount =
        std::clamp(
            0.28
            + 0.72
                * std::max(
                    0.0,
                    dot(normal, light)),
            0.0,
            1.0);

    return {
        static_cast<std::uint8_t>(
            std::lround(78.0 + 105.0 * amount)),
        static_cast<std::uint8_t>(
            std::lround(102.0 + 112.0 * amount)),
        static_cast<std::uint8_t>(
            std::lround(110.0 + 116.0 * amount))
    };
}

void draw_floor(
    Canvas& canvas,
    const Projector& projector) {

    const Color grid{220, 223, 226};

    for (int i = -8; i <= 8; ++i) {
        const double v =
            static_cast<double>(i) * 0.25;

        const auto a =
            projector.project(
                {-2.0, 0.0, v});
        const auto b =
            projector.project(
                {2.0, 0.0, v});

        if (a.valid && b.valid) {
            canvas.line(
                a.x, a.y, a.z,
                b.x, b.y, b.z,
                grid);
        }

        const auto c =
            projector.project(
                {v, 0.0, -2.0});
        const auto d =
            projector.project(
                {v, 0.0, 2.0});

        if (c.valid && d.valid) {
            canvas.line(
                c.x, c.y, c.z,
                d.x, d.y, d.z,
                grid);
        }
    }
}

} // namespace

void write_character_ppm(
    const std::string& path,
    const CharacterMeshFrame& frame,
    const CharacterRenderCamera& camera,
    bool draw_floor_grid) {

    if (frame.indices.size() % 3 != 0) {
        throw std::invalid_argument(
            "character frame index count is not triangular");
    }

    Canvas canvas(
        camera.width,
        camera.height);

    const Projector projector =
        projector_for(camera);

    if (draw_floor_grid) {
        draw_floor(
            canvas,
            projector);
    }

    for (std::size_t triangle = 0;
         triangle < frame.indices.size();
         triangle += 3) {

        const std::uint32_t i0 =
            frame.indices[triangle + 0];
        const std::uint32_t i1 =
            frame.indices[triangle + 1];
        const std::uint32_t i2 =
            frame.indices[triangle + 2];

        if (i0 >= frame.positions.size()
            || i1 >= frame.positions.size()
            || i2 >= frame.positions.size()) {
            continue;
        }

        const Vec3 p0 = frame.positions[i0];
        const Vec3 p1 = frame.positions[i1];
        const Vec3 p2 = frame.positions[i2];

        Vec3 normal =
            normalized(
                cross(
                    p1 - p0,
                    p2 - p0));

        if (length_squared(normal) <= 1e-12) {
            continue;
        }

        const Vec3 center =
            (p0 + p1 + p2) / 3.0;

        if (dot(
                normal,
                camera.position - center) < 0.0) {
            normal = -normal;
        }

        const Projected s0 =
            projector.project(p0);
        const Projected s1 =
            projector.project(p1);
        const Projected s2 =
            projector.project(p2);

        if (!s0.valid
            || !s1.valid
            || !s2.valid) {
            continue;
        }

        const double area =
            edge(
                s0.x, s0.y,
                s1.x, s1.y,
                s2.x, s2.y);

        if (std::abs(area) <= 1e-10) {
            continue;
        }

        const int min_x =
            std::max(
                0,
                static_cast<int>(
                    std::floor(
                        std::min({
                            s0.x,
                            s1.x,
                            s2.x
                        }))));

        const int max_x =
            std::min(
                static_cast<int>(
                    camera.width) - 1,
                static_cast<int>(
                    std::ceil(
                        std::max({
                            s0.x,
                            s1.x,
                            s2.x
                        }))));

        const int min_y =
            std::max(
                0,
                static_cast<int>(
                    std::floor(
                        std::min({
                            s0.y,
                            s1.y,
                            s2.y
                        }))));

        const int max_y =
            std::min(
                static_cast<int>(
                    camera.height) - 1,
                static_cast<int>(
                    std::ceil(
                        std::max({
                            s0.y,
                            s1.y,
                            s2.y
                        }))));

        Color color =
            shaded_character_color(normal);

        const std::size_t triangle_index = triangle / 3;

        if (triangle_index < frame.triangle_tags.size()
            && frame.triangle_tags[triangle_index] != 0) {
            // Evidence tint: keep the Lambert shading, shift the hue.
            const double shade =
                (static_cast<double>(color.r) - 78.0) / 105.0;
            if (frame.triangle_tags[triangle_index] == 1) {
                color = {
                    static_cast<std::uint8_t>(std::lround(150.0 + 95.0 * shade)),
                    static_cast<std::uint8_t>(std::lround(80.0 + 70.0 * shade)),
                    static_cast<std::uint8_t>(std::lround(30.0 + 40.0 * shade))};
            } else {
                color = {
                    static_cast<std::uint8_t>(std::lround(120.0 + 80.0 * shade)),
                    static_cast<std::uint8_t>(std::lround(40.0 + 40.0 * shade)),
                    static_cast<std::uint8_t>(std::lround(50.0 + 40.0 * shade))};
            }
        }

        for (int y = min_y;
             y <= max_y;
             ++y) {
            for (int x = min_x;
                 x <= max_x;
                 ++x) {

                const double px =
                    static_cast<double>(x) + 0.5;
                const double py =
                    static_cast<double>(y) + 0.5;

                const double w0 =
                    edge(
                        s1.x, s1.y,
                        s2.x, s2.y,
                        px, py)
                    / area;

                const double w1 =
                    edge(
                        s2.x, s2.y,
                        s0.x, s0.y,
                        px, py)
                    / area;

                const double w2 =
                    1.0 - w0 - w1;

                constexpr double epsilon = -1e-8;

                if (w0 < epsilon
                    || w1 < epsilon
                    || w2 < epsilon) {
                    continue;
                }

                const double depth =
                    w0 * s0.z
                    + w1 * s1.z
                    + w2 * s2.z;

                canvas.set(
                    x,
                    y,
                    depth,
                    color);
            }
        }
    }

    canvas.write(path);
}

} // namespace sarx
