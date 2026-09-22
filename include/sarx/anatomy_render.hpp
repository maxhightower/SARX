#pragma once

#include "sarx/anatomy.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sarx {

struct AnatomyCamera {
    Vec3 position{1.6, 1.3, 2.6};
    Vec3 target{0.0, 0.95, 0.0};
    Vec3 world_up{0.0, 1.0, 0.0};
    double vertical_fov_degrees{38.0};
    std::size_t width{960};
    std::size_t height{540};
};

struct AnatomyOverlayLine {
    Vec3 a{};
    Vec3 b{};
    Rgb8 color{40, 40, 40};
    int thickness{2};
};

struct AnatomyOverlayPoint {
    Vec3 position{};
    Rgb8 color{150, 20, 20};
    double size{0.012};
};

struct AnatomyFrame {
    std::vector<const AnatomyBody*> bodies;
    std::vector<AnatomyOverlayLine> lines;
    std::vector<AnatomyOverlayPoint> points;
    bool draw_floor{true};
    // Clip everything with z > clip_z (cutaway view). Disabled when NaN.
    double clip_z{0.0};
    bool cutaway{false};
    std::string caption;
};

class AnatomyImage {
public:
    AnatomyImage(std::size_t width, std::size_t height);
    [[nodiscard]] std::size_t width() const { return width_; }
    [[nodiscard]] std::size_t height() const { return height_; }
    [[nodiscard]] const std::vector<std::uint8_t>& rgb() const { return rgb_; }
    void write_ppm(const std::string& path) const;

    std::vector<std::uint8_t>& mutable_rgb() { return rgb_; }
    std::vector<float>& depth() { return depth_; }

private:
    std::size_t width_;
    std::size_t height_;
    std::vector<std::uint8_t> rgb_;
    std::vector<float> depth_;
};

[[nodiscard]] AnatomyImage render_anatomy(const AnatomyFrame& frame, const AnatomyCamera& camera);

} // namespace sarx
