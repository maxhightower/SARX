#pragma once

#include "sarx/math.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sarx {

struct CharacterMeshFrame {
    std::vector<Vec3> positions;
    std::vector<std::uint32_t> indices;
};

struct CharacterAssetStats {
    std::size_t vertices{};
    std::size_t triangles{};
    std::size_t character_nodes{};
    std::size_t skin_joints{};
    std::size_t animation_clips{};
};

class GltfCharacter {
public:
    GltfCharacter();
    ~GltfCharacter();

    GltfCharacter(GltfCharacter&&) noexcept;
    GltfCharacter& operator=(GltfCharacter&&) noexcept;

    GltfCharacter(const GltfCharacter&) = delete;
    GltfCharacter& operator=(const GltfCharacter&) = delete;

    void load(
        const std::string& character_glb,
        const std::string& animation_glb);

    [[nodiscard]] const CharacterAssetStats& stats() const;
    [[nodiscard]] const std::vector<std::string>& animation_names() const;

    [[nodiscard]] std::size_t find_animation(
        const std::string& name_fragment) const;

    [[nodiscard]] double animation_duration(
        std::size_t animation) const;

    [[nodiscard]] CharacterMeshFrame sample(
        std::size_t animation,
        double time_seconds,
        bool loop = true,
        const Vec3& world_offset = {}) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sarx
