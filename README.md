# SARX

**Real-time destructible dynamics for animated characters.**

SARX is an experimental character-dynamics system that joins skeletal animation with breakable volumetric simulation. The central research question is whether an animated character can transition locally and continuously from rig-driven motion to physically simulated deformation, fracture, tearing, and detached-body dynamics without rebuilding the rig or visible mesh every frame.

## Research direction

SARX separates four representations:

1. **Animation rig** — ordinary bone transforms provide moving targets.
2. **Breakable physical body** — particles/cells and XPBD-style constraints provide deformation and fracture authority.
3. **Dynamic islands** — topology failure creates connected components that can lose rig authority and become free physics.
4. **Render representation** — visual detail will remain decoupled from physical resolution.

## Current reference milestone: V0.3

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
- full-scan versus accelerated-path parity tests.

The next milestone moves from generic graph fixtures toward a real destructible volumetric character representation and GPU-oriented data layout.

## Status

Early research prototype. API and file formats are not stable.
