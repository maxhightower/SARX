# SARX V0 Architecture

## Purpose

SARX is a deterministic reference model for the coupling layer missing between conventional skeletal animation and real-time topology-changing deformable simulation.

The CPU implementation exists to make semantics precise before GPU migration.

## Core representations

### 1. Animation rig

Bones contain an animated world-space target and a parent relationship. Parent joints carry progressive damage, a break threshold, and a material ID. A bone only has animation authority while it can still reach a root through active parent links.

This is the first form of a **dynamic rig island**.

### 2. Physical particles

Particles own physical position, velocity, and inverse mass. Animation never writes positions directly.

### 3. Structural constraints

Structural constraints connect pairs of physical particles. They store:

- rest length,
- compliance,
- accumulated XPBD multiplier,
- progressive damage,
- break threshold,
- active/inactive topology state,
- material ID.

Failure modifies the physical connectivity graph rather than editing a render mesh.

### 4. Breakable animation attachments

An attachment couples a physical particle to a bone target through a compliant positional constraint. It has progressive damage, a break threshold, and a material ID.

An attachment is solved only when:

1. the attachment itself remains intact, and
2. its bone is still connected to a rig root.

Animation is therefore a physical influence, not absolute authority.

## V0.2 spatial damage authority

Spatial damage converts world-space geometry into damage on the constraint graph.

SARX currently supports:

- capsule/blade sweeps,
- spherical damage volumes,
- cut versus blunt damage modes,
- material-dependent cut/blunt resistance,
- progressive sub-threshold damage,
- fracture event records.

A damage primitive is tested against three live geometric representations:

1. a structural constraint is represented by its particle-to-particle segment,
2. an animation attachment is represented by its particle-to-target segment,
3. a bone joint is represented by its parent-target-to-child-target segment.

The router computes proximity, converts event energy into normalized damage using material resistance, applies that damage to the authoritative body state, and records the result.

This means severance no longer needs an anatomy-specific command such as "remove arm." A blade crossing the shoulder can independently break both the physical bridge and the rig parent link because both occupy intersected world-space segments.

## Material response

Materials currently define:

- cut resistance,
- blunt resistance.

This is intentionally minimal. Later material models can add anisotropy, tensile/shear/compressive response, strain-rate effects, fracture toughness, and fiber direction without changing the spatial damage contract.

## Fracture events

Every touched target can emit a `FractureEvent` containing:

- target kind,
- target ID,
- material ID,
- approximate world-space hit position,
- applied damage,
- whether this event crossed the failure threshold.

These events are the synchronization boundary for later systems such as:

- wound rendering,
- particles/fluids,
- audio,
- gameplay damage,
- GPU topology updates,
- replay/debug ledgers.

## Detachment semantics

A body is partitioned into connected components using only active structural constraints.

For each component SARX computes:

- member particles,
- total mass,
- linear momentum,
- whether any surviving attachment reaches a root-connected bone.

A component with no root-connected attachment is a **free dynamic island**.

No velocity reset occurs when topology changes. Severance therefore preserves the physical state present at the moment of detachment.

## Reference transition

An intact limb has both:

- a structural path to the torso, and
- a rig path to the root.

A spatial cut can now destroy those paths automatically.

    animated + connected
            |
            | spatial blade event
            v
    material-aware damage routing
        /               \
physical bridge      rig joint
   failure             failure
        \               /
         dynamic island
              |
              v
        free simulation

## Why CPU first?

The eventual target is a GPU-oriented solver, but the CPU implementation establishes a small authoritative specification for:

- compliance behavior,
- material response,
- break thresholds,
- graph partitioning,
- rig authority,
- momentum-preserving handoff,
- spatial damage routing,
- fracture-event ordering/contents.

A future GPU implementation should be tested for parity against this reference model.

## Next milestone

V0.3 should make the damage model mechanically richer:

- anisotropic tissue response,
- strain-driven spontaneous tearing,
- bone-like brittle fracture,
- cut planes with persistent wound topology,
- event IDs and deterministic replay,
- broad-phase acceleration so damage does not scan every constraint,
- active-damage-region bookkeeping as groundwork for GPU/adaptive simulation.
