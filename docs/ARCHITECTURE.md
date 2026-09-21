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

Physical connectivity is computed only from active structural constraints. Each connected component reports its particles, mass, linear momentum, and whether a surviving attachment reaches a root-connected bone.

A component with no root-connected attachment is a free dynamic island. Topology changes never reset its velocity, so detachment preserves existing motion.

## Spatial damage authority

SARX supports capsule/blade sweeps and spherical damage volumes. Spatial primitives are tested against:

1. particle-to-particle structural segments,
2. particle-to-bone-target attachment segments,
3. parent-target-to-child-target rig-joint segments.

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

A material can optionally define a world-space fiber direction plus separate longitudinal and transverse cut multipliers.

For a structural/attachment/joint segment, effective cut resistance is interpolated from the squared alignment between the target segment and the fiber direction.

This lets the same blade energy produce different outcomes when cutting:

- along strong fibers,
- across fibers.

The current reference uses world-space fibers for simplicity. A later anatomical representation should transport local material frames with the deforming body.

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
- active animation attachments,
- active parent-child bone joints.

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

## Next milestone: V0.4B

Continue from the generated lattice toward an anatomical deformable volume:

- local material/fiber frames that move with deformation,
- bone/joint capsule geometry rather than point-only bone targets,
- explicit volumetric/tetrahedral constraints for volume preservation,
- anatomical region construction,
- adaptive damage-domain activation,
- GPU-friendly structure-of-arrays buffers,
- CPU/GPU parity harness,
- first visual debug renderer for particles, constraints, bones, wounds, and islands.
