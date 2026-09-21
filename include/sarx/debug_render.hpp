#pragma once

#include "sarx/damage.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace sarx {

struct DebugCamera {
    Vec3 position{3.0, 2.0, 4.0};
    Vec3 target{0.75, 0.15, 0.15};
    Vec3 world_up{0.0, 1.0, 0.0};
    double pixels_per_unit{360.0};
    std::size_t width{960};
    std::size_t height{540};
};

struct DebugRenderOptions {
    bool structural{true};
    bool tetrahedral{true};
    bool particles{true};
    bool bones{true};
    bool wounds{true};
};

void write_debug_ppm(
    const std::string& path,
    const Body& body,
    const std::vector<WoundDescriptor>& wounds = {},
    const DebugCamera& camera = {},
    const DebugRenderOptions& options = {});

} // namespace sarx
