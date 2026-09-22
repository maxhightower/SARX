# SARX Reference Architecture

## Purpose

SARX is a deterministic reference model for the coupling layer between conventional skeletal animation and real-time topology-changing deformable simulation.

The CPU implementation defines authoritative semantics before those semantics are migrated to GPU compute.

## Core representations

### Animation rig

Bones expose animated world-space targets and parent relationships. Parent joints carry progressive damage, break thresholds, and material IDs. A bone only retains animation authority while it can reach a root through active parent links.

### Physical particles

Particles own physical position, velocity, and inverse mass. Animation never writes particle positions directly.

### Structural constraints

Structural constraints connect pairs of particles and store rest length, compliance, XPBD multiplier, progressive damage, break threshold, material ID, and active topology state.

### Breakable animation attachments

Attachments couple physical particles to bone targets through compliant constraints. Their own damage state can fail independently of structural tissue.

## Dynamic islands

Physical connectivity is computed from active structural constraints **and active tetrahedral volume cells**. Each connected component reports its particles, mass, linear momentum, and whether a surviving attachment reaches a root-connected bone.

A component with no root-connected attachment is a free dynamic island. Topology changes never reset its velocity, so detachment preserves existing motion.

## Spatial damage authority

SARX supports capsule/blade sweeps and spherical damage volumes. Spatial primitives are tested against:

1. particle-to-particle structural segments,
2. tetrahedral volume cells,
3. particle-to-bone-target attachment segments,
4. radius-bearing parent/child joint capsules.

Damage authority remains in the exact router: geometry establishes contact, material response converts event energy into damage, and body state decides whether a threshold was crossed.

## V0.3 deterministic event stream

Every damage operation receives a `DamageEventId`.

If a caller supplies an ID, SARX preserves it. If the ID is zero, the damage system allocates one monotonically. Applied operations are recorded as replayable commands.

The same command stream can be applied to an identical initial body to reproduce accumulated damage and topology. This is the intended future contract for:

- deterministic debugging,
- save/replay ledgers,
- multiplayer authority experiments,
- CPU/GPU parity testing.

Strain-driven failures use the same event stream as spatial weapon/impact damage.

## V0.3 anisotropic materials

A material can define a rest-space fiber direction plus separate longitudinal and transverse cut multipliers. Structural constraints also support a per-region rest-fiber override.

For structural tissue, SARX transports that rest fiber by the minimal rotation from the constraint's rest axis to its current deformed axis. Effective cut resistance is then interpolated from the squared alignment between the current transported fiber and the current tissue segment.

This lets the same blade energy produce different outcomes when cutting:

- along strong fibers,
- across fibers.

This makes anisotropy body-relative: rotating or deforming tissue rotates its material orientation rather than leaving fiber direction frozen in world space.

## V0.3 strain-driven failure

Structural constraints can now fail without an external spatial event.

Material response defines:

- tensile yield strain,
- tensile break strain,
- progressive strain-damage rate.

Below yield, no damage is added. Between yield and break, damage accumulates over time. At or above break strain, the structural constraint fails immediately.

This provides two important regimes:

- ductile/progressive tissue tearing,
- brittle fracture-like failure.

## Fracture events

Each touched target can emit a `FractureEvent` containing:

- event ID,
- source (spatial or strain),
- target kind,
- target ID,
- material ID,
- world-space position,
- applied damage,
- whether the event crossed the failure threshold.

These events synchronize downstream systems without making those systems authoritative over topology.

## Persistent wound descriptors

A successful cut can create a persistent `WoundDescriptor` containing:

- originating event ID,
- wound center,
- optional normalized cut-surface normal,
- radius,
- number of authoritative targets broken.

Fracture events answer **what failed**. Wound descriptors answer **what persistent cut surface downstream rendering/fluids should continue representing**.

## V0.3 damage broad phase

A `DamageBroadPhase` uniform grid caches the current pose's damage-receiving segments.

The grid indexes:

- active structural constraints,
- active tetrahedral volume cells,
- active animation attachments,
- full AABBs of active parent-child joint capsules.

A local capsule/sphere query produces a `DamageCandidates` set. Only those IDs are passed to the exact damage router.

The broad phase does **not** decide intersection, damage, or fracture. It can only reject distant primitives. Therefore full-scan and candidate-restricted execution are required to produce identical authoritative outcomes.

The cache is explicit because the body is deformable: callers rebuild it when the indexed pose has changed enough to invalidate its cells. Future GPU work can replace this with incremental/refit structures.

## Current reference transition

    animated + connected
            |
            | spatial cut OR physical overstrain
            v
       deterministic event
            |
            v
    material-aware failure
      /        |        \
 structure  attachment  rig joint
      \        |        /
       surviving connectivity
              |
        connected components
              |
      root authority test
         /          \
 animated island   free island

## Why CPU first?

The eventual target is GPU-oriented, but the CPU implementation is the authoritative specification for:

- XPBD coupling behavior,
- damage thresholds,
- anisotropic material response,
- progressive/brittle strain failure,
- graph partitioning,
- rig authority,
- momentum-preserving handoff,
- spatial damage routing,
- deterministic event ordering,
- replay,
- wound creation,
- broad-phase parity.

A GPU implementation should be tested against this reference rather than independently redefining these semantics.

## V0.4A generated body fixture

SARX can now generate a reusable three-dimensional lattice rather than requiring tests to hand-author every particle and constraint.

A `VoxelLatticeSpec` defines:

- origin,
- X/Y/Z particle dimensions,
- spacing,
- particle mass,
- default structural material,
- structural compliance/break threshold,
- whether diagonal links are included.

The builder creates deterministic particle IDs and unique structural links. Axial-only mode provides a simple reference topology; diagonal mode adds face/body-diagonal support for a more isotropic spring network.

### Spatial material regions

`MaterialRegion` assigns materials by world-space bounds and priority.

Particle materials are sampled at node positions. Structural materials are sampled at constraint midpoints, allowing a generated body to contain different tissue classes without hand-authoring constraint IDs.

### Automatic bone embedding

`embed_bone` adds a rig bone and attaches every lattice particle inside an influence radius.

Each attachment stores:

    local_offset = particle_rest_position - animated_bone_position

so its target reconstructs the original physical rest pose exactly. Embedded child bones preserve the requested rig parent and therefore participate in the existing dynamic-rig-island semantics.

V0.4A is intentionally still a particle/link reference volume, not the final continuum model. Its purpose is to create deterministic 3D fixtures on which the next mechanics can be developed and benchmarked.

## V0.4B anatomical deformable volume

V0.4B converts the V0.4A lattice from a spring-network fixture into a first anatomical-volume reference.

### Transported material frames

Every structural constraint retains its rest direction. Region generation can also assign a per-constraint rest-fiber orientation. At damage time SARX rotates the rest fiber from the rest structural axis into the current deformed axis before evaluating longitudinal/transverse cut response.

### Joint capsules

Parent-child rig links now have an explicit radius. The physical joint representation is a capsule from the parent's animated target to the child's animated target rather than an infinitely thin line. The same radius is honored by exact damage, broad-phase indexing, and adaptive-domain selection.

### Tetrahedral volume constraints

Body supports explicit XPBD signed-volume constraints over four particles. Each tetrahedron stores particle IDs, signed rest volume, compliance, progressive damage, break threshold, active topology state, and material ID.

Generated lattice cells are deterministically decomposed into six tetrahedra sharing the cell's 000-to-111 diagonal. Active tetrahedra participate in connected-component authority because they physically couple their particles even if distance links fail.

### Cut-to-volume topology

A cutting segment is tested against candidate tetrahedra. When a cut enters a tet, material-scaled damage is applied to the volume cell and a tetrahedral fracture event is emitted if it crosses its break threshold.

This prevents a severed region from remaining invisibly connected by an intact volume constraint after surrounding distance links have been cut.

### Anatomical region primitives

MaterialRegion now supports boxes, spheres, capsules, priority-based overlap, material assignment, and optional rest-space fiber direction.

Node positions sample particle material/fiber state, structural midpoints sample link material/fiber state, and lattice-cell centers assign tetrahedral material.

### Adaptive damage domains

select_damage_domain produces a wound-local set of particles, structural constraints, tetrahedral cells, attachments, and joint capsules. It is bookkeeping for future adaptive simulation and GPU work; it does not decide fracture.

### GPU-oriented SoA mirror

snapshot_body_soa converts the authoritative Body into structure-of-arrays buffers for particles, structural constraints, tetrahedral cells, bones/joints, and attachments. The SoA representation is currently an export boundary, not an independent solver.

## Next milestone: V0.4C / V0.5 preparation

The next work should focus on execution and visualization:

- incremental/refittable adaptive-domain indices,
- actual restricted-domain solver scheduling,
- GPU compute kernels over the SoA layout,
- CPU/GPU numerical and topology parity harness,
- tet-aware cut halo / near-face capsule intersection,
- richer local-frame transport,
- first visual debug renderer for particles, tets, constraints, bones, wounds, and islands,
- a small rigged humanoid fixture exercising animation -> deformation -> cut -> severance.

## V0.5 layered anatomical voxel body (`AnatomyBody`)

`include/sarx/anatomy.hpp` and `src/anatomy.cpp` add a second physical model next to the V0.4 `Body`. It trades the per-pair distance/tet topology for a voxel lattice that is cheaper per unit volume and maps directly to GPU passes.

### Representation

- **Voxels and particles.** Each voxel owns 8 corner particles (corner bit 0 = +x, 1 = +y, 2 = +z). Particle arrays are float SoA.
- **Bonds.** Each face bond joins the 4 coincident corner pairs of two neighbouring voxels (zero rest length). Bonds are stored grouped by axis.
- **Tissue.** Each voxel carries a tissue: skin, fat, muscle, tendon, bone, marrow, brain, heart, lung, liver or gut. Tissue parameters are density, shape stiffness, bond stiffness, cut HP, tear strain, ballistic cost, rig tone and colour.
- **Bond properties.** Bond HP and tear strain take the weaker tissue of the pair, and a per-voxel `tear_scale` gives slack skin over joints. Bone-to-bone bonds across different rig joints are articulations with ligament properties.

### Solver step (per 60 Hz frame)

1. Topology is refreshed if dirty. Kinematic voxels are set by linear-blend skinning, and velocity is taken from the pose delta. Dynamic voxels and bonds are stream-compacted.
2. Adaptive substep count: enough that the largest relative bond speed (or absolute speed near the ground) moves less than half a voxel per substep, between `substeps` and `max_substeps`.
3. Each substep runs:
   1. integrate;
   2. rig tone on dynamic, rig-authoritative voxels;
   3. grabs;
   4. `iterations` of: VGS pass → bone-fragment shape matching → ball joints (3 sweeps) → ground → bond passes x, y, z → ground;
   5. contacts between separate pieces;
   6. velocity update with per-voxel deformation damping;
   7. tear check.
4. Components are rebuilt if tearing changed topology. Settled free islands go to sleep.

The VGS pass and the three bond passes are each conflict-free and run on a persistent worker pool. The result does not depend on the thread count.

### Authority and residency

- **Rig authority.** Among components containing root-joint bone, the one with the most root-joint bone keeps the rig. Every other component permanently becomes a free island, with the V0.4 semantics. Detached voxels keep their velocity.
- **Residency.**
  - `Hybrid` (the default) keeps intact rig-authoritative voxels kinematic.
  - Damage promotes voxels within `activation_halo` (8 cm) of each event to dynamic.
  - Grabs promote their sphere.
  - Losing rig authority promotes the whole island.
  - `Dynamic` simulates everything.
- **Sleep.** A free island sleeps after `sleep_frames` consecutive frames with at least 97% of its voxels below `sleep_speed`. Damage or a grab wakes the whole island.

### Damage

- **Blades.** `apply_blade` sweeps bilinear patches between consecutive blade poses. Bonds whose centre-to-centre segment crosses a patch are sorted by sweep parameter and processed in that order, and each costs `hp / sharpness`.
  - A bond the blade can't afford takes partial damage, and the blade lodges there.
  - Parted faces are wedged apart along the patch normal.
  - A share of the spent energy becomes momentum along the swing.
- **Bullets.** `apply_bullet` collects voxels within the round's channel and processes them in order along the ray. Each costs its tissue's ballistic energy, scaled by voxel size.
  - Destroyed voxels become debris records.
  - Bone hits damage the surrounding bone bonds.
  - Lost momentum is deposited near the channel (forward plus radial).
  - The result carries the exit point and velocity for the host world.
- **Tearing.** A bond tears when the centroid distance between its two voxels exceeds `(1 + tear_strain) · h`. Measuring between centroids is independent of solve order; corner gaps stay near zero because bonds are solved last.
  - Sustained overload drains HP over a few substeps.
  - Overload above 3x parts the bond at once.

### Known limitations

- There is no self-collision inside one connected piece.
- An upright limp body still tears some tissue while collapsing.
- Tissue values are calibrated for plausible gameplay, not measured.
