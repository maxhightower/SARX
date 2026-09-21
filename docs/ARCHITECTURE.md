# SARX V0 Architecture

## Purpose

SARX V0 is a deterministic CPU reference model for the coupling layer missing between conventional skeletal animation and real-time topology-changing deformable simulation.

The V0 milestone is not intended to be visually impressive. It exists to make the semantics precise before they are moved to GPU compute.

## Core representations

### 1. Animation rig

Bones contain an animated world-space target and a parent relationship. A parent joint can become inactive. A bone only has animation authority while it can still reach a root through active parent links.

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
- active/inactive topology state.

Failure therefore modifies the physical connectivity graph rather than editing a render mesh.

### 4. Breakable animation attachments

An attachment couples a physical particle to a bone target through a compliant positional constraint. It also has its own progressive damage and failure state.

An attachment is solved only when:

1. the attachment itself remains intact, and
2. its bone is still connected to a rig root.

This means animation is a physical influence, not absolute authority.

## Detachment semantics

A body is partitioned into connected components using only active structural constraints.

For each component SARX computes:

- member particles,
- total mass,
- linear momentum,
- whether any surviving attachment reaches a root-connected bone.

A component with no root-connected attachment is a **free dynamic island**.

No velocity reset occurs when topology changes. Therefore severance preserves the physical state already present at the moment of detachment.

## V0 transition

An intact limb has both:

- a structural path to the torso, and
- a rig path to the root.

A complete severance breaks both paths.

    animated + connected
            |
            | damage
            v
    structural bridge fails
            |
            | joint/attachment failure
            v
      free dynamic island

Future damage-routing code will infer these failures from geometry/material state rather than explicit test calls.

## Why CPU first?

The eventual target is a GPU-oriented solver, but the CPU implementation establishes a small authoritative specification for:

- compliance behavior,
- break thresholds,
- graph partitioning,
- rig authority,
- momentum-preserving handoff.

A future GPU implementation should be tested for parity against this reference model.

## Next milestone

V0.2 should add spatial damage events:

- segment/blade cuts,
- spherical and capsule damage volumes,
- material-dependent failure thresholds,
- automatic coupling of physical cuts to rig-link failure,
- fracture event records suitable for renderer synchronization.
