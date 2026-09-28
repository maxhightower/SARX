#include "sarx/character_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace sarx {
namespace {

double mass_of(const Particle& p) {
    return p.inverse_mass > 0.0 ? 1.0 / p.inverse_mass : 0.0;
}

template <typename T>
std::vector<std::size_t> iota_ids(const std::vector<T>& items) {
    std::vector<std::size_t> ids(items.size());
    std::iota(ids.begin(), ids.end(), std::size_t{0});
    return ids;
}

} // namespace

CharacterRuntime::CharacterRuntime(Body body, RuntimeConfig config)
    : body_(std::move(body)), config_(config) {

    if (config_.step.substeps <= 0 || config_.step.solver_iterations <= 0) {
        throw std::invalid_argument("runtime step counts must be positive");
    }
    if (config_.linear_damping < 0.0) {
        throw std::invalid_argument("linear damping must be non-negative");
    }

    home_bone_.assign(body_.particles().size(), kNoParent);
    // A particle's first attachment defines its home bone (additional,
    // blended attachments do not change its anatomical region).
    for (const auto& attachment : body_.attachments()) {
        if (home_bone_[attachment.particle] == kNoParent) {
            home_bone_[attachment.particle] = attachment.bone;
        }
    }

    external_accel_.assign(body_.particles().size(), Vec3{});
    particle_island_.assign(body_.particles().size(), kNoIsland);

    all_structural_ = iota_ids(body_.structural_constraints());
    all_tetrahedral_ = iota_ids(body_.tetrahedral_constraints());
    all_attachments_ = iota_ids(body_.attachments());

    previous_bone_targets_.reserve(body_.bones().size());
    previous_bone_rotations_.reserve(body_.bones().size());
    for (const auto& bone : body_.bones()) {
        previous_bone_targets_.push_back(bone.animated_position);
        previous_bone_rotations_.push_back(bone.animated_rotation);
    }

    rebuild_topology(0);
}

// --- Anatomy -----------------------------------------------------------

void CharacterRuntime::define_region(
    const std::string& name,
    std::vector<ParticleId> particles) {

    if (name.empty()) {
        throw std::invalid_argument("region name must not be empty");
    }

    std::sort(particles.begin(), particles.end());
    particles.erase(
        std::unique(particles.begin(), particles.end()),
        particles.end());
    for (const ParticleId p : particles) {
        if (p >= body_.particles().size()) {
            throw std::out_of_range("region particle out of range");
        }
    }

    std::vector<std::uint8_t> member(body_.particles().size(), 0u);
    for (const ParticleId p : particles) member[p] = 1u;

    Region region;
    region.name = name;
    region.particles = std::move(particles);
    const auto& structural = body_.structural_constraints();
    for (ConstraintId id = 0; id < structural.size(); ++id) {
        if (member[structural[id].a] && member[structural[id].b]) {
            region.internal_structural.push_back(id);
        }
    }

    const auto existing = std::find_if(
        regions_.begin(),
        regions_.end(),
        [&](const Region& r) { return r.name == name; });
    if (existing != regions_.end()) {
        *existing = std::move(region);
    } else {
        regions_.push_back(std::move(region));
    }
}

void CharacterRuntime::define_bone_region(
    const std::string& name,
    const std::vector<BoneId>& bones) {

    std::vector<ParticleId> particles;
    for (ParticleId p = 0; p < home_bone_.size(); ++p) {
        if (std::find(bones.begin(), bones.end(), home_bone_[p])
            != bones.end()) {
            particles.push_back(p);
        }
    }
    define_region(name, std::move(particles));
}

void CharacterRuntime::define_sphere_region(
    const std::string& name,
    const Vec3& center,
    double radius) {

    if (radius <= 0.0) {
        throw std::invalid_argument("sphere region radius must be positive");
    }

    std::vector<ParticleId> particles;
    const double r2 = radius * radius;
    for (ParticleId p = 0; p < body_.particles().size(); ++p) {
        if (length_squared(body_.particles()[p].position - center) <= r2) {
            particles.push_back(p);
        }
    }
    define_region(name, std::move(particles));
}

bool CharacterRuntime::has_region(const std::string& name) const {
    return region(name) != nullptr;
}

std::vector<std::string> CharacterRuntime::region_names() const {
    std::vector<std::string> names;
    names.reserve(regions_.size());
    for (const auto& r : regions_) names.push_back(r.name);
    return names;
}

const CharacterRuntime::Region* CharacterRuntime::region(
    const std::string& name) const {

    for (const auto& r : regions_) {
        if (r.name == name) return &r;
    }
    return nullptr;
}

const std::vector<ParticleId>& CharacterRuntime::region_particles(
    const std::string& name) const {

    const Region* r = region(name);
    if (!r) throw std::out_of_range("unknown region: " + name);
    return r->particles;
}

RegionStatus CharacterRuntime::region_status(const std::string& name) const {
    const Region* r = region(name);
    if (!r) throw std::out_of_range("unknown region: " + name);

    RegionStatus status;
    status.name = r->name;
    status.total_particles = r->particles.size();
    status.internal_constraints = r->internal_structural.size();
    for (const ConstraintId id : r->internal_structural) {
        if (body_.structural_constraints()[id].active) {
            ++status.active_internal_constraints;
        }
    }

    for (const ParticleId p : r->particles) {
        const IslandId island = particle_island_[p];
        const IslandState* state = find_island(island);
        if (state && state->rig_authoritative) {
            ++status.rig_connected_particles;
        }
        if (std::find(status.islands.begin(), status.islands.end(), island)
            == status.islands.end()) {
            status.islands.push_back(island);
        }
    }
    std::sort(status.islands.begin(), status.islands.end());
    return status;
}

std::vector<AnatomicalAvailability> CharacterRuntime::island_anatomy(
    IslandId island) const {

    std::vector<AnatomicalAvailability> result;
    result.reserve(regions_.size());
    for (const auto& r : regions_) {
        AnatomicalAvailability availability;
        availability.region = r.name;
        availability.total_voxels = r.particles.size();
        for (const ParticleId p : r.particles) {
            if (particle_island_[p] == island) ++availability.attached_voxels;
        }
        result.push_back(std::move(availability));
    }
    return result;
}

std::vector<AnatomicalAvailability> CharacterRuntime::rig_anatomy() const {
    std::vector<std::uint8_t> rig_island(next_island_id_, 0u);
    for (const auto& island : islands_) {
        if (island.rig_authoritative) rig_island[island.id] = 1u;
    }

    std::vector<AnatomicalAvailability> result;
    result.reserve(regions_.size());
    for (const auto& r : regions_) {
        AnatomicalAvailability availability;
        availability.region = r.name;
        availability.total_voxels = r.particles.size();
        for (const ParticleId p : r.particles) {
            if (rig_island[particle_island_[p]]) ++availability.attached_voxels;
        }
        result.push_back(std::move(availability));
    }
    return result;
}

// --- Damage / topology -------------------------------------------------

DamageReport CharacterRuntime::apply_damage(const DamageCommand& command) {
    DamageReport report = damage_.apply(body_, command);

    RuntimeLogEntry entry;
    entry.step_index = step_index_;
    entry.command = damage_.history().back();
    damage_log_.push_back(entry);

    if (report.broken_count() > 0) {
        rebuild_topology(report.event_id);
    }
    return report;
}

void CharacterRuntime::rebuild_topology(DamageEventId event_id) {
    const std::vector<Island> raw = body_.islands();

    std::vector<IslandState> next;
    next.reserve(raw.size());

    const bool initial = islands_.empty();

    // Connectivity only ever decreases, so every new island is a subset of
    // exactly one previous island. Group new islands by their predecessor.
    std::unordered_map<IslandId, std::vector<std::size_t>> by_parent;
    std::vector<IslandId> parent_order;

    for (std::size_t i = 0; i < raw.size(); ++i) {
        IslandState state;
        state.particles = raw[i].particles;
        state.rig_authoritative = raw[i].rig_authoritative;
        next.push_back(std::move(state));

        const IslandId previous = initial
            ? kNoIsland
            : particle_island_[raw[i].particles.front()];
        if (!initial) {
            for (const ParticleId p : raw[i].particles) {
                if (particle_island_[p] != previous) {
                    throw std::logic_error(
                        "island merged across previous topology");
                }
            }
        }
        auto [it, inserted] = by_parent.try_emplace(previous);
        if (inserted) parent_order.push_back(previous);
        it->second.push_back(i);
    }

    for (const IslandId previous : parent_order) {
        auto& group = by_parent[previous];

        if (initial) {
            for (const std::size_t i : group) {
                next[i].id = next_island_id_++;
                next[i].created_by_event = event_id;
                next[i].created_at_step = step_index_;
            }
            continue;
        }

        const IslandState* old = find_island(previous);

        // The largest child inherits the predecessor's id; ties resolve to
        // the child containing the lowest particle id (raw order).
        std::size_t heir = group.front();
        for (const std::size_t i : group) {
            if (next[i].particles.size() > next[heir].particles.size()) {
                heir = i;
            }
        }

        next[heir].id = previous;
        next[heir].parent = old ? old->parent : kNoIsland;
        next[heir].created_by_event = old ? old->created_by_event : 0;
        next[heir].created_at_step = old ? old->created_at_step : 0;

        if (group.size() == 1) continue;

        TopologyEvent event;
        event.event_id = event_id;
        event.step_index = step_index_;
        event.parent = previous;
        event.children.push_back(previous);
        for (const std::size_t i : group) {
            if (i == heir) continue;
            next[i].id = next_island_id_++;
            next[i].parent = previous;
            next[i].created_by_event = event_id;
            next[i].created_at_step = step_index_;
            event.children.push_back(next[i].id);
        }
        topology_events_.push_back(std::move(event));
    }

    std::sort(next.begin(), next.end(), [](const auto& a, const auto& b) {
        return a.id < b.id;
    });

    islands_ = std::move(next);
    for (const auto& island : islands_) {
        for (const ParticleId p : island.particles) {
            particle_island_[p] = island.id;
        }
    }
    refresh_island_kinematics();
}

void CharacterRuntime::refresh_island_kinematics() {
    const auto& particles = body_.particles();
    const double inf = std::numeric_limits<double>::infinity();

    for (auto& island : islands_) {
        island.mass = 0.0;
        Vec3 weighted_position{};
        Vec3 momentum{};
        island.bounds_min = {inf, inf, inf};
        island.bounds_max = {-inf, -inf, -inf};

        for (const ParticleId id : island.particles) {
            const auto& p = particles[id];
            const double m = mass_of(p);
            island.mass += m;
            weighted_position += p.position * m;
            momentum += p.velocity * m;
            island.bounds_min.x = std::min(island.bounds_min.x, p.position.x);
            island.bounds_min.y = std::min(island.bounds_min.y, p.position.y);
            island.bounds_min.z = std::min(island.bounds_min.z, p.position.z);
            island.bounds_max.x = std::max(island.bounds_max.x, p.position.x);
            island.bounds_max.y = std::max(island.bounds_max.y, p.position.y);
            island.bounds_max.z = std::max(island.bounds_max.z, p.position.z);
        }

        if (island.mass > 0.0) {
            island.center_of_mass = weighted_position / island.mass;
            island.linear_velocity = momentum / island.mass;
        }
    }
}

// --- Islands -------------------------------------------------------------

const IslandState* CharacterRuntime::find_island(IslandId id) const {
    const auto it = std::lower_bound(
        islands_.begin(),
        islands_.end(),
        id,
        [](const IslandState& s, IslandId value) { return s.id < value; });
    return it != islands_.end() && it->id == id ? &*it : nullptr;
}

IslandId CharacterRuntime::island_of_particle(ParticleId particle) const {
    return particle < particle_island_.size()
        ? particle_island_[particle]
        : kNoIsland;
}

BoneId CharacterRuntime::home_bone(ParticleId particle) const {
    return particle < home_bone_.size() ? home_bone_[particle] : kNoParent;
}

// --- Motor influence -------------------------------------------------------

void CharacterRuntime::add_particle_acceleration(
    ParticleId particle,
    const Vec3& accel) {

    if (particle >= external_accel_.size()) {
        throw std::out_of_range("particle out of range");
    }
    external_accel_[particle] += accel;
    has_external_accel_ = true;
}

void CharacterRuntime::add_island_acceleration(
    IslandId island,
    const Vec3& accel) {

    const IslandState* state = find_island(island);
    if (!state) throw std::out_of_range("unknown island");
    for (const ParticleId p : state->particles) {
        external_accel_[p] += accel;
    }
    has_external_accel_ = true;
}

bool CharacterRuntime::has_pending_external_acceleration() const {
    return has_external_accel_;
}

// --- Simulation ------------------------------------------------------------

void CharacterRuntime::apply_ground_contact() {
    const auto& ground = config_.ground;
    for (auto& p : body_.particles()) {
        if (p.inverse_mass == 0.0 || p.position.y >= ground.height) continue;

        p.position.y = ground.height;
        const double normal_change =
            p.velocity.y < 0.0 ? -p.velocity.y * (1.0 + ground.restitution) : 0.0;
        p.velocity.y = p.velocity.y < 0.0
            ? -p.velocity.y * ground.restitution
            : p.velocity.y;

        const double tangential = std::sqrt(
            p.velocity.x * p.velocity.x + p.velocity.z * p.velocity.z);
        if (tangential <= 1e-12) continue;

        // Coulomb bound on the tangential impulse; at rest this reproduces a
        // deceleration of friction * gravity.
        const double removable = ground.friction * normal_change;
        const double scale = removable >= tangential
            ? 0.0
            : (tangential - removable) / tangential;
        p.velocity.x *= scale;
        p.velocity.z *= scale;
    }
}

void CharacterRuntime::step(double dt) {
    if (dt <= 0.0) throw std::invalid_argument("dt must be positive");

    const int substeps = config_.step.substeps;
    const double h = dt / static_cast<double>(substeps);

    StepConfig single = config_.step;
    single.substeps = 1;

    SolverDomain domain;
    domain.particles = iota_ids(body_.particles());
    domain.structural = all_structural_;
    domain.tetrahedral = all_tetrahedral_;
    if (rig_authority_) domain.attachments = all_attachments_;

    // Bone targets set by the consumer describe the pose at the end of this
    // step. Interpolate them across substeps so rig-driven particles carry a
    // continuous velocity instead of jumping in the first substep and
    // resting in the last one (which would erase momentum at handoff).
    const std::size_t bone_count = body_.bones().size();
    std::vector<Vec3> desired_targets(bone_count);
    std::vector<Rotation> desired_rotations(bone_count);
    for (BoneId b = 0; b < bone_count; ++b) {
        desired_targets[b] = body_.bones()[b].animated_position;
        desired_rotations[b] = body_.bones()[b].animated_rotation;
    }

    for (int s = 0; s < substeps; ++s) {
        const double t =
            static_cast<double>(s + 1) / static_cast<double>(substeps);
        for (BoneId b = 0; b < bone_count; ++b) {
            body_.set_bone_pose(
                b,
                previous_bone_targets_[b]
                    + (desired_targets[b] - previous_bone_targets_[b]) * t,
                slerp(previous_bone_rotations_[b], desired_rotations[b], t));
        }

        if (has_external_accel_) {
            auto& particles = body_.particles();
            for (std::size_t i = 0; i < particles.size(); ++i) {
                if (particles[i].inverse_mass == 0.0) continue;
                particles[i].velocity += external_accel_[i] * h;
            }
        }

        (void)body_.step_restricted(h, domain, single);

        if (config_.linear_damping > 0.0) {
            const double keep = std::max(0.0, 1.0 - config_.linear_damping * h);
            for (auto& p : body_.particles()) p.velocity *= keep;
        }

        if (config_.ground.enabled) apply_ground_contact();
    }

    if (has_external_accel_) {
        std::fill(external_accel_.begin(), external_accel_.end(), Vec3{});
        has_external_accel_ = false;
    }

    previous_bone_targets_ = std::move(desired_targets);
    previous_bone_rotations_ = std::move(desired_rotations);
    ++step_index_;
    refresh_island_kinematics();
}

RuntimeStats CharacterRuntime::stats() const {
    RuntimeStats s;
    s.particles = body_.particles().size();
    s.structural_constraints = body_.structural_constraints().size();
    for (const auto& c : body_.structural_constraints()) {
        if (c.active) ++s.active_structural_constraints;
    }
    s.tetrahedral_constraints = body_.tetrahedral_constraints().size();
    for (const auto& t : body_.tetrahedral_constraints()) {
        if (t.active) ++s.active_tetrahedral_constraints;
    }
    s.attachments = body_.attachments().size();
    for (const auto& a : body_.attachments()) {
        if (a.active) ++s.active_attachments;
    }
    s.islands = islands_.size();
    for (const auto& island : islands_) {
        if (island.rig_authoritative) ++s.rig_authoritative_islands;
    }
    s.steps = step_index_;
    return s;
}

// --- Humanoid helpers ------------------------------------------------------

HumanoidRuntime build_humanoid_runtime(
    const HumanoidSpec& spec,
    const Vec3& world_offset,
    const RuntimeConfig& config) {

    HumanoidFixture fixture = build_humanoid_fixture(spec);

    for (auto& p : fixture.body.particles()) {
        p.position += world_offset;
    }

    HumanoidRuntime result;
    result.world_offset = world_offset;
    for (BoneId b = 0; b < fixture.body.bones().size(); ++b) {
        const Vec3 target = fixture.body.bones()[b].animated_position + world_offset;
        fixture.body.set_bone_target(b, target);
        result.rest_bone_positions.push_back(target);
    }

    result.bones = fixture.bones;
    result.runtime = CharacterRuntime(std::move(fixture.body), config);
    define_humanoid_regions(result.runtime, result.bones);
    return result;
}

void define_humanoid_regions(
    CharacterRuntime& runtime,
    const HumanoidBones& b) {

    runtime.define_bone_region("head", {b.head});
    runtime.define_bone_region("neck", {b.neck});
    runtime.define_bone_region("torso", {b.pelvis, b.spine, b.chest});
    runtime.define_bone_region("left_arm", {b.left_shoulder, b.left_elbow});
    runtime.define_bone_region("right_arm", {b.right_shoulder, b.right_elbow});
    runtime.define_bone_region("left_hand", {b.left_hand});
    runtime.define_bone_region("right_hand", {b.right_hand});
    runtime.define_bone_region("left_leg", {b.left_hip, b.left_knee});
    runtime.define_bone_region("right_leg", {b.right_hip, b.right_knee});
    runtime.define_bone_region("left_foot", {b.left_ankle});
    runtime.define_bone_region("right_foot", {b.right_ankle});
}

} // namespace sarx
