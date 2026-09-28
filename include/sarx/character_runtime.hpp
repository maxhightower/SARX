#pragma once

// Consumer-facing runtime facade over the SARX reference Body.
//
// CharacterRuntime owns one Body plus its DamageSystem and adds the small
// amount of bookkeeping a game consumer needs without re-implementing any
// SARX physics or topology logic:
//
//   - stable physical-island identifiers across topology changes,
//   - a topology event stream (which island split into which islands),
//   - named anatomical regions (particle sets) with availability/integrity
//     queries compatible with sarx::evaluate_motion_viability,
//   - an optional ground half-space contact applied between substeps,
//   - external per-particle accelerations (generic "external motor
//     influence") and a global rig-authority switch (active/passive).
//
// It deliberately contains no game semantics: there is no notion of life,
// death, brains, enemies, or scoring here. Consumers own those.

#include "sarx/body.hpp"
#include "sarx/damage.hpp"
#include "sarx/humanoid.hpp"
#include "sarx/voxel_character.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sarx {

using IslandId = std::uint32_t;
inline constexpr IslandId kNoIsland = 0;

struct GroundContact {
    bool enabled{true};
    double height{0.0};
    // Coulomb-style coefficient: tangential velocity removed per contact is
    // bounded by friction * normal velocity change.
    double friction{0.8};
    double restitution{0.0};
};

struct RuntimeConfig {
    StepConfig step{};
    GroundContact ground{};
    // Optional linear velocity damping in 1/s applied every substep
    // (v *= 1 - linear_damping * h). 0 keeps the undamped Body behaviour.
    double linear_damping{0.0};
};

struct IslandState {
    IslandId id{kNoIsland};
    // Island this one split away from (kNoIsland for the initial island).
    IslandId parent{kNoIsland};
    DamageEventId created_by_event{0};
    std::uint64_t created_at_step{0};

    std::vector<ParticleId> particles;
    bool rig_authoritative{false};

    double mass{0.0};
    Vec3 center_of_mass{};
    Vec3 linear_velocity{};
    Vec3 bounds_min{};
    Vec3 bounds_max{};
};

struct TopologyEvent {
    DamageEventId event_id{0};
    std::uint64_t step_index{0};
    IslandId parent{kNoIsland};
    // parent keeps its id (it is always children.front()); the remaining
    // entries are newly created islands.
    std::vector<IslandId> children;
};

struct RegionStatus {
    std::string name;
    std::size_t total_particles{};
    std::size_t internal_constraints{};
    std::size_t active_internal_constraints{};
    // Particles of this region that currently live in rig-authoritative
    // islands.
    std::size_t rig_connected_particles{};
    // Islands that currently contain at least one particle of this region,
    // ordered by island id.
    std::vector<IslandId> islands;

    [[nodiscard]] double integrity() const {
        return internal_constraints > 0
            ? static_cast<double>(active_internal_constraints)
                / static_cast<double>(internal_constraints)
            : 1.0;
    }
};

struct RuntimeLogEntry {
    std::uint64_t step_index{0};
    DamageCommand command{};
};

struct RuntimeStats {
    std::size_t particles{};
    std::size_t structural_constraints{};
    std::size_t active_structural_constraints{};
    std::size_t tetrahedral_constraints{};
    std::size_t active_tetrahedral_constraints{};
    std::size_t attachments{};
    std::size_t active_attachments{};
    std::size_t islands{};
    std::size_t rig_authoritative_islands{};
    std::uint64_t steps{};
};

class CharacterRuntime {
public:
    CharacterRuntime() = default;
    explicit CharacterRuntime(Body body, RuntimeConfig config = {});

    // --- Anatomy -------------------------------------------------------

    // Registers (or replaces) a named region from an explicit particle set.
    void define_region(
        const std::string& name,
        std::vector<ParticleId> particles);

    // Region = every particle whose attachment targets one of `bones`.
    void define_bone_region(
        const std::string& name,
        const std::vector<BoneId>& bones);

    // Region = every particle currently within `radius` of `center`.
    void define_sphere_region(
        const std::string& name,
        const Vec3& center,
        double radius);

    [[nodiscard]] bool has_region(const std::string& name) const;
    [[nodiscard]] std::vector<std::string> region_names() const;
    [[nodiscard]] const std::vector<ParticleId>& region_particles(
        const std::string& name) const;
    [[nodiscard]] RegionStatus region_status(const std::string& name) const;

    // Availability of every region inside one island (attached_voxels is the
    // number of region particles in that island). Every registered region is
    // always reported, so evaluate_motion_viability never treats a missing
    // region as intact.
    [[nodiscard]] std::vector<AnatomicalAvailability> island_anatomy(
        IslandId island) const;

    // Availability of every region across all rig-authoritative islands.
    [[nodiscard]] std::vector<AnatomicalAvailability> rig_anatomy() const;

    // --- Damage / topology ---------------------------------------------

    DamageReport apply_damage(const DamageCommand& command);

    [[nodiscard]] const std::vector<TopologyEvent>& topology_events() const {
        return topology_events_;
    }
    [[nodiscard]] const std::vector<RuntimeLogEntry>& damage_log() const {
        return damage_log_;
    }

    // --- Islands -------------------------------------------------------

    [[nodiscard]] const std::vector<IslandState>& islands() const {
        return islands_;
    }
    [[nodiscard]] const IslandState* find_island(IslandId id) const;
    [[nodiscard]] IslandId island_of_particle(ParticleId particle) const;

    // --- Motor influence -----------------------------------------------

    // When disabled, attachment constraints are excluded from the solve and
    // the body is purely passive. Attachments are not damaged or broken.
    void set_rig_authority(bool enabled) { rig_authority_ = enabled; }
    [[nodiscard]] bool rig_authority() const { return rig_authority_; }

    // Accumulated for the next step() only, then cleared.
    void add_particle_acceleration(ParticleId particle, const Vec3& accel);
    void add_island_acceleration(IslandId island, const Vec3& accel);
    [[nodiscard]] bool has_pending_external_acceleration() const;

    // --- Simulation ----------------------------------------------------

    void step(double dt);

    [[nodiscard]] Body& body() { return body_; }
    [[nodiscard]] const Body& body() const { return body_; }
    [[nodiscard]] DamageSystem& damage_system() { return damage_; }
    [[nodiscard]] RuntimeConfig& config() { return config_; }
    [[nodiscard]] const RuntimeConfig& config() const { return config_; }
    [[nodiscard]] std::uint64_t step_index() const { return step_index_; }
    [[nodiscard]] RuntimeStats stats() const;

    // Particles with an attachment to `bone` (the particle's "home" bone).
    [[nodiscard]] BoneId home_bone(ParticleId particle) const;

private:
    struct Region {
        std::string name;
        std::vector<ParticleId> particles;
        std::vector<ConstraintId> internal_structural;
    };

    [[nodiscard]] const Region* region(const std::string& name) const;
    void rebuild_topology(DamageEventId event_id);
    void refresh_island_kinematics();
    void apply_ground_contact();

    Body body_;
    DamageSystem damage_;
    RuntimeConfig config_{};

    std::vector<Region> regions_;
    std::vector<BoneId> home_bone_;

    std::vector<IslandState> islands_;
    std::vector<IslandId> particle_island_;
    IslandId next_island_id_{1};

    std::vector<TopologyEvent> topology_events_;
    std::vector<RuntimeLogEntry> damage_log_;

    std::vector<Vec3> previous_bone_targets_;
    std::vector<Rotation> previous_bone_rotations_;
    std::vector<Vec3> external_accel_;
    bool has_external_accel_{false};
    bool rig_authority_{true};
    std::uint64_t step_index_{0};

    std::vector<ConstraintId> all_structural_;
    std::vector<ConstraintId> all_tetrahedral_;
    std::vector<ConstraintId> all_attachments_;
};

// Builds the procedural SARX humanoid fixture translated by `world_offset`
// (particles and bone targets) and wraps it in a runtime.
struct HumanoidRuntime {
    CharacterRuntime runtime;
    HumanoidBones bones;
    std::vector<Vec3> rest_bone_positions;
    Vec3 world_offset{};
};

[[nodiscard]] HumanoidRuntime build_humanoid_runtime(
    const HumanoidSpec& spec = {},
    const Vec3& world_offset = {},
    const RuntimeConfig& config = {});

// Generic coarse anatomical regions for the humanoid fixture:
// head, neck, torso, left_arm, right_arm, left_hand, right_hand,
// left_leg, right_leg, left_foot, right_foot.
void define_humanoid_regions(
    CharacterRuntime& runtime,
    const HumanoidBones& bones);

} // namespace sarx
