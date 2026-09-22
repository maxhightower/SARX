#include "sarx/anatomy_render.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace sarx {
namespace {

struct Projector {
    Vec3 eye, fwd, right, up;
    double focal;
    double cx, cy;

    [[nodiscard]] bool project(const Vec3& p, double& sx, double& sy, double& depth) const {
        const Vec3 d = p - eye;
        depth = dot(d, fwd);
        if (depth < 0.05) return false;
        sx = cx + focal * dot(d, right) / depth;
        sy = cy - focal * dot(d, up) / depth;
        return true;
    }
};

Projector make_projector(const AnatomyCamera& cam) {
    Projector p;
    p.eye = cam.position;
    p.fwd = normalized(cam.target - cam.position);
    p.right = normalized(cross(p.fwd, cam.world_up));
    p.up = cross(p.right, p.fwd);
    const double fov = cam.vertical_fov_degrees * 3.14159265358979 / 180.0;
    p.focal = 0.5 * static_cast<double>(cam.height) / std::tan(0.5 * fov);
    p.cx = 0.5 * static_cast<double>(cam.width);
    p.cy = 0.5 * static_cast<double>(cam.height);
    return p;
}

struct Raster {
    AnatomyImage& img;

    void pixel(int x, int y, float z, Rgb8 c) {
        if (x < 0 || y < 0 || x >= static_cast<int>(img.width()) || y >= static_cast<int>(img.height())) return;
        const std::size_t i = static_cast<std::size_t>(y) * img.width() + static_cast<std::size_t>(x);
        if (z >= img.depth()[i]) return;
        img.depth()[i] = z;
        img.mutable_rgb()[i * 3 + 0] = c.r;
        img.mutable_rgb()[i * 3 + 1] = c.g;
        img.mutable_rgb()[i * 3 + 2] = c.b;
    }

    void triangle(const std::array<double, 3>& xs, const std::array<double, 3>& ys,
                  const std::array<double, 3>& zs, Rgb8 c) {
        const double minx = std::floor(std::min({xs[0], xs[1], xs[2]}));
        const double maxx = std::ceil(std::max({xs[0], xs[1], xs[2]}));
        const double miny = std::floor(std::min({ys[0], ys[1], ys[2]}));
        const double maxy = std::ceil(std::max({ys[0], ys[1], ys[2]}));
        const int x0 = std::max(0, static_cast<int>(minx));
        const int x1 = std::min(static_cast<int>(img.width()) - 1, static_cast<int>(maxx));
        const int y0 = std::max(0, static_cast<int>(miny));
        const int y1 = std::min(static_cast<int>(img.height()) - 1, static_cast<int>(maxy));
        const double area = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (xs[2] - xs[0]) * (ys[1] - ys[0]);
        if (std::abs(area) < 1e-9) return;
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const double px = x + 0.5, py = y + 0.5;
                const double w0 = ((xs[1] - px) * (ys[2] - py) - (xs[2] - px) * (ys[1] - py)) / area;
                const double w1 = ((xs[2] - px) * (ys[0] - py) - (xs[0] - px) * (ys[2] - py)) / area;
                const double w2 = 1.0 - w0 - w1;
                if (w0 < -1e-6 || w1 < -1e-6 || w2 < -1e-6) continue;
                const double z = w0 * zs[0] + w1 * zs[1] + w2 * zs[2];
                pixel(x, y, static_cast<float>(z), c);
            }
        }
    }

    void line(double xa, double ya, double za, double xb, double yb, double zb, Rgb8 c, int thick) {
        const int steps = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(xb - xa), std::abs(yb - ya)))));
        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            const double x = xa + (xb - xa) * t, y = ya + (yb - ya) * t, z = za + (zb - za) * t;
            for (int oy = -thick / 2; oy <= thick / 2; ++oy)
                for (int ox = -thick / 2; ox <= thick / 2; ++ox)
                    pixel(static_cast<int>(x) + ox, static_cast<int>(y) + oy, static_cast<float>(z - 0.02), c);
        }
    }
};

Rgb8 shade(Rgb8 base, double lambert, double tint_r = 1.0) {
    auto ch = [&](std::uint8_t v, double m) {
        return static_cast<std::uint8_t>(std::clamp(v * m, 0.0, 255.0));
    };
    return {ch(base.r, lambert * tint_r), ch(base.g, lambert), ch(base.b, lambert)};
}

// Corners on each face (axis, dir) in counter-clockwise order seen from outside.
std::array<int, 4> face_corners(int axis, int dir) {
    switch (axis * 2 + dir) {
    case 0: return {0, 4, 6, 2};  // -x
    case 1: return {1, 3, 7, 5};  // +x
    case 2: return {0, 1, 5, 4};  // -y
    case 3: return {2, 6, 7, 3};  // +y
    case 4: return {0, 2, 3, 1};  // -z
    default: return {4, 5, 7, 6}; // +z
    }
}

} // namespace

AnatomyImage::AnatomyImage(std::size_t width, std::size_t height)
    : width_(width), height_(height), rgb_(width * height * 3, 0u),
      depth_(width * height, std::numeric_limits<float>::infinity()) {}

void AnatomyImage::write_ppm(const std::string& path) const {
    std::ofstream out(path, std::ios::binary);
    if (!out) throw std::runtime_error("cannot write " + path);
    out << "P6\n" << width_ << ' ' << height_ << "\n255\n";
    out.write(reinterpret_cast<const char*>(rgb_.data()), static_cast<std::streamsize>(rgb_.size()));
}

AnatomyImage render_anatomy(const AnatomyFrame& frame, const AnatomyCamera& camera) {
    AnatomyImage img(camera.width, camera.height);
    // Background gradient.
    for (std::size_t y = 0; y < camera.height; ++y) {
        const double t = static_cast<double>(y) / static_cast<double>(camera.height);
        const Rgb8 c{static_cast<std::uint8_t>(34 + 26 * t), static_cast<std::uint8_t>(38 + 24 * t),
                     static_cast<std::uint8_t>(46 + 20 * t)};
        for (std::size_t x = 0; x < camera.width; ++x) {
            const std::size_t i = (y * camera.width + x) * 3;
            img.mutable_rgb()[i] = c.r;
            img.mutable_rgb()[i + 1] = c.g;
            img.mutable_rgb()[i + 2] = c.b;
        }
    }

    const Projector proj = make_projector(camera);
    Raster raster{img};
    const Vec3 light = normalized(Vec3{0.45, 0.85, 0.55});

    auto quad = [&](const std::array<Vec3, 4>& q, Rgb8 base, double tint_r) {
        std::array<double, 4> sx{}, sy{}, sz{};
        for (int i = 0; i < 4; ++i)
            if (!proj.project(q[i], sx[i], sy[i], sz[i])) return;
        const Vec3 n = normalized(cross(q[1] - q[0], q[3] - q[0]));
        const double lambert = 0.35 + 0.65 * std::max(0.0, dot(n, light));
        const Rgb8 c = shade(base, lambert, tint_r);
        raster.triangle({sx[0], sx[1], sx[2]}, {sy[0], sy[1], sy[2]}, {sz[0], sz[1], sz[2]}, c);
        raster.triangle({sx[0], sx[2], sx[3]}, {sy[0], sy[2], sy[3]}, {sz[0], sz[2], sz[3]}, c);
    };

    if (frame.draw_floor) {
        const double tile = 0.25;
        for (int ix = -12; ix < 12; ++ix)
            for (int iz = -12; iz < 12; ++iz) {
                const bool dark = ((ix + iz) & 1) != 0;
                const Rgb8 c = dark ? Rgb8{74, 78, 84} : Rgb8{92, 96, 102};
                const double x0 = ix * tile, z0 = iz * tile;
                quad({Vec3{x0, 0, z0}, Vec3{x0, 0, z0 + tile}, Vec3{x0 + tile, 0, z0 + tile}, Vec3{x0 + tile, 0, z0}}, c, 1.0);
            }
    }

    for (const AnatomyBody* body : frame.bodies) {
        if (!body) continue;
        const auto& tissues = body->tissues();
        for (std::uint32_t v = 0; v < body->voxel_count(); ++v) {
            if (!body->voxel_alive(v)) continue;
            const bool clipped = frame.cutaway && body->voxel_rest_center(v).z > frame.clip_z;
            if (clipped) continue;
            const Rgb8 base = tissues[static_cast<std::size_t>(body->voxel_tissue(v))].color;
            for (int axis = 0; axis < 3; ++axis)
                for (int dir = 0; dir < 2; ++dir) {
                    const std::uint32_t nb = body->neighbor(v, axis, dir);
                    bool exposed = nb == std::numeric_limits<std::uint32_t>::max() || !body->voxel_alive(nb);
                    if (!exposed) {
                        const std::size_t b = body->bond_across(v, axis, dir);
                        exposed = b == std::numeric_limits<std::size_t>::max() || !body->bond_active(b);
                    }
                    if (!exposed && frame.cutaway && body->voxel_rest_center(nb).z > frame.clip_z) exposed = true;
                    if (!exposed) continue;
                    const auto fc = face_corners(axis, dir);
                    const std::array<Vec3, 4> q{body->corner(v, fc[0]), body->corner(v, fc[1]),
                                                body->corner(v, fc[2]), body->corner(v, fc[3])};
                    // Fresh wound surfaces read wet and slightly redder.
                    const double tint = body->face_wounded(v, axis, dir) ? 1.12 : 1.0;
                    quad(q, base, tint);
                }
        }
    }

    for (const auto& pt : frame.points) {
        double sx, sy, sz;
        if (!proj.project(pt.position, sx, sy, sz)) continue;
        const int r = std::max(1, static_cast<int>(pt.size * proj.focal / sz * 0.5));
        for (int oy = -r; oy <= r; ++oy)
            for (int ox = -r; ox <= r; ++ox)
                if (ox * ox + oy * oy <= r * r)
                    raster.pixel(static_cast<int>(sx) + ox, static_cast<int>(sy) + oy, static_cast<float>(sz), pt.color);
    }

    for (const auto& ln : frame.lines) {
        double xa, ya, za, xb, yb, zb;
        if (!proj.project(ln.a, xa, ya, za) || !proj.project(ln.b, xb, yb, zb)) continue;
        raster.line(xa, ya, za, xb, yb, zb, ln.color, ln.thickness);
    }
    return img;
}

} // namespace sarx
