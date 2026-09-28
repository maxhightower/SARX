# SARX consumer runtime facade (`sarx::CharacterRuntime`)

Added on the Townward integration line (`claude/townward-0-foundation-sarx-rrjsay`,
based on the validated PR #7 head `7c3cc0af089d021821b910d76b1d701de3ac8d0a`).

The facade is **generic**. It exists so a game can consume SARX without
re-implementing physics, topology, or anatomy bookkeeping. It contains no
game semantics (no life/death, brains, enemies, waves, or scoring).

## What it adds on top of `sarx::Body` / `sarx::DamageSystem`

| Responsibility | API |
| --- | --- |
| Body creation | `build_humanoid_runtime(spec, world_offset, config)` wraps the procedural humanoid fixture; `CharacterRuntime(Body, RuntimeConfig)` wraps any body. |
| Geometry-driven damage | `apply_damage(DamageCommand)` (capsule / plane cut / sphere / strain) → `DamageReport` with the existing fracture-event stream. |
| Topology events | `topology_events()` — each split lists the parent island and its children. |
| Stable island identifiers | `islands()` / `find_island(id)` / `island_of_particle(p)`. Connectivity only decreases, so each new island is a subset of one previous island; the largest child keeps the parent id, the others get new ids that record `parent`, `created_by_event`, `created_at_step`. |
| Per-island physical state | mass, centre of mass, linear velocity, bounds, `rig_authoritative`. |
| Anatomical regions | `define_region`, `define_bone_region`, `define_sphere_region`, `define_humanoid_regions`. |
| Region status / viability input | `region_status(name)` (particle count, internal structural integrity, owning islands, rig-connected particles); `island_anatomy(id)` and `rig_anatomy()` return `AnatomicalAvailability` for **every** region, ready for `evaluate_motion_viability`. |
| External motor influence | `add_particle_acceleration`, `add_island_acceleration` (cleared after each step). |
| Active ↔ passive | `set_rig_authority(bool)` excludes attachment constraints from the solve without damaging them; particle state is untouched. |
| Ground contact | Optional half-space with Coulomb-bounded friction, applied between substeps. |
| Damping | Optional `linear_damping` (1/s, default 0 = unchanged Body behaviour). Useful because the reference solver has no rolling resistance: a capsule-shaped passive torso otherwise rocks indefinitely. |
| Deterministic replay | `damage_log()` records each command with the step index at which it was applied. |

## Momentum continuity

`step()` interpolates consumer-set bone targets linearly across substeps.
Without this, a kinematic target that jumps once per frame is reached in the
first substep and the final substep reports ~zero velocity, so a walking body
would hand almost no momentum to a severed limb. With interpolation, the
driven-body velocity matches the target velocity (see
`test_split_preserves_momentum_and_positions`).

## Build

`SARX_BUILD_TOOLS=OFF` builds only the library and unit tests (what consumers
such as Townward use). Default builds are unchanged.

~~~bash
cmake -S . -B build -DSARX_BUILD_TOOLS=OFF
cmake --build build
ctest --test-dir build --output-on-failure
~~~

## Anatomical humanoid and embedded voxel skin

`sarx/anatomical_humanoid.hpp` adds a second, human-proportioned humanoid body
(~1.8 m, ~83 kg, arms-forward rest pose). It has the same 17-bone topology and
`HumanoidBones` as the reference fixture and is simulated on a 5 cm lattice
(676 particles). Each particle attaches to the bone that ends its nearest
skeletal segment, so any cut on a segment detaches everything distal to it.
`anatomical_landmarks()` gives rest-space cut and brain placements.

`build_anatomical_detail_voxels(0.02)` generates a 2 cm appearance voxel set
(~7.9k voxels). It adds detail the lattice does not simulate (face, ears,
fingers, feet) and tags each voxel with a tissue type: soft tissue, bone
(skull, spine, ribs, limb bones) or neural (brain).

`sarx/voxel_skin.hpp` `EmbeddedVoxelSkin` binds each detail voxel to an affine
frame of four lattice particles in the same island. It deforms the voxels with
that frame's deformation gradient, and carves them with the same
`DamageCommand` SARX applies to the lattice. It also flags "exposed" voxels:
those facing a carved hole or a torn island boundary. The skin is appearance
only and never feeds back into physics. Tests: `tests/test_anatomical_skin.cpp`.
