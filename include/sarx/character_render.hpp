#pragma once

#include "sarx/gltf_character.hpp"

#include <cstddef>
#include <string>

namespace sarx {

struct CharacterRenderCamera {
    Vec3 position{2.5, 1.4, 4.0};
    Vec3 target{0.0, 0.9, 0.0};
    Vec3 world_up{0.0, 1.0, 0.0};

    double vertical_fov_degrees{38.0};
    std::size_t width{960};
    std::size_t height{720};
};

void write_character_ppm(
    const std::string& path,
    const CharacterMeshFrame& frame,
    const CharacterRenderCamera& camera = {},
    bool draw_floor = true);

} // namespace sarx
