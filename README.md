# SARX

**Real-time destructible dynamics for animated characters.**

SARX is an experimental character-dynamics system that joins skeletal animation with breakable volumetric simulation. The core research question is whether an animated character can transition locally and continuously from rig-driven motion to physically simulated deformation, fracture, tearing, and detached-body dynamics without rebuilding the character rig or render mesh every frame.

## Research direction

SARX is being built around four separable representations:

1. **Animation rig** — ordinary bone transforms provide moving targets.
2. **Breakable physical volume** — particles/cells and XPBD-style constraints provide deformation and fracture authority.
3. **Dynamic islands** — connected components created by topology failure transition from rig-driven to free physical motion.
4. **Render representation** — later work will decouple visual detail from physical resolution using mesh/SDF/splat techniques.

The first milestone is deliberately renderer-independent: prove animated target constraints, progressive damage, constraint failure, connected-component separation, and momentum-preserving detachment in a deterministic CPU reference implementation.

## Status

Early research prototype. API and file formats are not stable.
