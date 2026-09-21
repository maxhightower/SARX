# SARX

**Real-time destructible dynamics for animated characters.**

SARX is an experimental character-dynamics system that joins skeletal animation with breakable volumetric simulation. The central research question is whether an animated character can transition locally and continuously from rig-driven motion to physically simulated deformation, fracture, tearing, and detached-body dynamics without rebuilding the rig or visible mesh every frame.

## Research direction

SARX separates four representations:

1. **Animation rig** — ordinary bone transforms provide moving targets.
2. **Breakable physical body** — particles/cells and XPBD-style constraints provide deformation and fracture authority.
3. **Dynamic islands** — topology failure creates connected components that can lose rig authority and become free physics.
4. **Render representation** — visual detail will remain decoupled from physical resolution.

## Current reference milestone: V0.4B

The renderer-independent CPU reference now covers:

- compliant bone-target attachments,
- progressive structural, attachment, and rig-joint damage,
- connected-component detachment with momentum preservation,
- capsule/blade and sphere damage primitives,
- material-dependent cut/blunt resistance,
- anisotropic fiber-aware cutting,
- strain-driven progressive tearing,
- brittle break-strain failure,
- deterministic damage-event IDs and command replay,
- fracture-event records,
- persistent wound descriptors,
- uniform-grid broad-phase candidate rejection,
- full-scan versus accelerated-path parity tests,
- generated 3D voxel-style particle lattices,
- deterministic lattice indexing and structural connectivity,
- spatial material regions,
- optional diagonal/shear connectivity,
- automatic root/child bone embedding and particle attachment generation,
- transported rest-space material/fiber orientation,
- radius-bearing bone/joint capsules,
- XPBD tetrahedral volume preservation,
- damageable/cuttable tetrahedral topology,
- box/sphere/capsule anatomical material regions,
- region-specific fiber orientation,
- adaptive wound-local damage-domain selection,
- GPU-friendly structure-of-arrays state snapshots.

The next stage moves these validated reference semantics into restricted-domain scheduling, GPU compute/parity work, and a visual rigged-character fixture.

## Status

Early research prototype. API and file formats are not stable.
