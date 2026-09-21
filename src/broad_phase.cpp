#include "sarx/broad_phase.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace sarx {
namespace {

struct CellKey {
    int x{};
    int y{};
    int z{};

    bool operator==(const CellKey&) const = default;
};

struct CellKeyHash {
    std::size_t operator()(const CellKey& key) const noexcept {
        std::size_t h = static_cast<std::size_t>(key.x) * 73856093u;
        h ^= static_cast<std::size_t>(key.y) * 19349663u;
        h ^= static_cast<std::size_t>(key.z) * 83492791u;
        return h;
    }
};

enum class IndexedKind : unsigned char {
    Structural,
    Attachment,
    BoneJoint
};

struct Entry {
    IndexedKind kind{};
    std::size_t id{};
};

struct EntryKey {
    IndexedKind kind{};
    std::size_t id{};

    bool operator==(const EntryKey&) const = default;
};

struct EntryKeyHash {
    std::size_t operator()(const EntryKey& value) const noexcept {
        return (static_cast<std::size_t>(value.kind) << 56)
            ^ (value.id * 11400714819323198485ull);
    }
};

Vec3 min_vec(const Vec3& a, const Vec3& b) {
    return {
        std::min(a.x, b.x),
        std::min(a.y, b.y),
        std::min(a.z, b.z)
    };
}

Vec3 max_vec(const Vec3& a, const Vec3& b) {
    return {
        std::max(a.x, b.x),
        std::max(a.y, b.y),
        std::max(a.z, b.z)
    };
}

} // namespace

struct DamageBroadPhase::Impl {
    double cell_size{0.5};
    std::size_t indexed_primitives{};
    std::unordered_map<CellKey, std::vector<Entry>, CellKeyHash> cells;

    [[nodiscard]] CellKey key_for(const Vec3& p) const {
        return {
            static_cast<int>(std::floor(p.x / cell_size)),
            static_cast<int>(std::floor(p.y / cell_size)),
            static_cast<int>(std::floor(p.z / cell_size))
        };
    }

    void insert_aabb(const Vec3& min_p, const Vec3& max_p, Entry entry) {
        const CellKey lo = key_for(min_p);
        const CellKey hi = key_for(max_p);

        for (int z = lo.z; z <= hi.z; ++z) {
            for (int y = lo.y; y <= hi.y; ++y) {
                for (int x = lo.x; x <= hi.x; ++x) {
                    cells[CellKey{x, y, z}].push_back(entry);
                }
            }
        }
    }

    [[nodiscard]] BroadPhaseQuery query_aabb(
        const Vec3& min_p,
        const Vec3& max_p) const {

        BroadPhaseQuery result;
        result.indexed_primitives = indexed_primitives;

        const CellKey lo = key_for(min_p);
        const CellKey hi = key_for(max_p);
        std::unordered_set<EntryKey, EntryKeyHash> seen;

        for (int z = lo.z; z <= hi.z; ++z) {
            for (int y = lo.y; y <= hi.y; ++y) {
                for (int x = lo.x; x <= hi.x; ++x) {
                    ++result.visited_cells;
                    const auto it = cells.find(CellKey{x, y, z});
                    if (it == cells.end()) continue;

                    for (const Entry& entry : it->second) {
                        if (!seen.insert(EntryKey{entry.kind, entry.id}).second) {
                            continue;
                        }

                        switch (entry.kind) {
                        case IndexedKind::Structural:
                            result.candidates.structural.push_back(entry.id);
                            break;
                        case IndexedKind::Attachment:
                            result.candidates.attachments.push_back(entry.id);
                            break;
                        case IndexedKind::BoneJoint:
                            result.candidates.bone_joints.push_back(entry.id);
                            break;
                        }
                    }
                }
            }
        }

        auto sort_ids = [](auto& ids) {
            std::sort(ids.begin(), ids.end());
        };
        sort_ids(result.candidates.structural);
        sort_ids(result.candidates.attachments);
        sort_ids(result.candidates.bone_joints);
        return result;
    }
};

DamageBroadPhase::DamageBroadPhase()
    : impl_(std::make_unique<Impl>()) {}

DamageBroadPhase::~DamageBroadPhase() = default;
DamageBroadPhase::DamageBroadPhase(DamageBroadPhase&&) noexcept = default;
DamageBroadPhase& DamageBroadPhase::operator=(DamageBroadPhase&&) noexcept = default;

void DamageBroadPhase::rebuild(const Body& body, double cell_size) {
    if (cell_size <= 0.0) {
        throw std::invalid_argument("broad-phase cell size must be positive");
    }

    impl_->cell_size = cell_size;
    impl_->indexed_primitives = 0;
    impl_->cells.clear();

    for (ConstraintId id = 0; id < body.structural_constraints().size(); ++id) {
        const auto& c = body.structural_constraints()[id];
        if (!c.active) continue;
        const Vec3 a = body.particles()[c.a].position;
        const Vec3 b = body.particles()[c.b].position;
        impl_->insert_aabb(min_vec(a, b), max_vec(a, b), Entry{IndexedKind::Structural, id});
        ++impl_->indexed_primitives;
    }

    for (ConstraintId id = 0; id < body.attachments().size(); ++id) {
        const auto& a = body.attachments()[id];
        if (!a.active) continue;
        const Vec3 p0 = body.particles()[a.particle].position;
        const Vec3 p1 = body.bones()[a.bone].animated_position + a.local_offset;
        impl_->insert_aabb(min_vec(p0, p1), max_vec(p0, p1), Entry{IndexedKind::Attachment, id});
        ++impl_->indexed_primitives;
    }

    for (BoneId id = 0; id < body.bones().size(); ++id) {
        const auto& bone = body.bones()[id];
        if (bone.parent == kNoParent || !bone.joint_to_parent_active) continue;
        const Vec3 p0 = body.bones()[bone.parent].animated_position;
        const Vec3 p1 = bone.animated_position;
        const Vec3 r{bone.joint_radius, bone.joint_radius, bone.joint_radius};
        impl_->insert_aabb(
            min_vec(p0, p1) - r,
            max_vec(p0, p1) + r,
            Entry{IndexedKind::BoneJoint, id});
        ++impl_->indexed_primitives;
    }
}

BroadPhaseQuery DamageBroadPhase::query_capsule(const CapsuleDamage& damage) const {
    if (damage.radius <= 0.0) {
        throw std::invalid_argument("capsule radius must be positive");
    }
    const Vec3 r{damage.radius, damage.radius, damage.radius};
    return impl_->query_aabb(
        min_vec(damage.a, damage.b) - r,
        max_vec(damage.a, damage.b) + r);
}

BroadPhaseQuery DamageBroadPhase::query_sphere(const SphereDamage& damage) const {
    if (damage.radius <= 0.0) {
        throw std::invalid_argument("sphere radius must be positive");
    }
    const Vec3 r{damage.radius, damage.radius, damage.radius};
    return impl_->query_aabb(damage.center - r, damage.center + r);
}

double DamageBroadPhase::cell_size() const {
    return impl_->cell_size;
}

std::size_t DamageBroadPhase::indexed_primitives() const {
    return impl_->indexed_primitives;
}

} // namespace sarx
