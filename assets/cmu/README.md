# CMU injury motion integration

SARX uses selected authored injury locomotion motions from the Carnegie Mellon
University Graphics Lab Motion Capture Database as M2 recovery references.

Selected clips:

| SARX ID | CMU clip | Description |
| --- | --- | --- |
| CMU_Limp | 91_16 | Limp |
| CMU_HurtLegWalk | 91_24 | HurtLegWalk |
| CMU_DragBadLegWalk | 91_25 | DragBadLegWalk — retained for other severe leg cases, excluded from isolated foot-loss selection |
| CMU_WalkWoundedLeg | 139_19 | Walk Wounded Leg |
| CMU_PainfulLeftKnee | 142_12 | Painfulleftknee |

The source motion data is from https://mocap.cs.cmu.edu/ and is obtained in FBX
form through the public `gbionics/cmu-fbx` dataset. That dataset documents that
its FBX conversions were retargeted in Blender using a CC0 Quaternius character.

CMU states that its motion capture data is free for use, including in
commercially sold products, but the motion data itself may not be resold
directly, even in converted form. Published work should acknowledge:

"The data used in this project was obtained from mocap.cs.cmu.edu. The database
was created with funding from NSF EIA-0196217."

SARX's vendor workflow downloads only the selected source clips, converts them
to GLB, and validates that their animated node names map to the SARX Quaternius
integration character before they can become authoritative evidence.


## SARX conversion policy

The authored CMU source skeleton is explicitly mapped onto 19 major SARX
Quaternius joints. The vendor gate requires at least 17 mapped target joints.
External translation and scale animation channels are stripped after Blender
conversion: CMU contributes authored joint rotations, while the SARX Quaternius
character remains authoritative for rest offsets, bone lengths, scale, and
world/root translation.

The current validated converted assets have these SHA-256 values:

```
bf0a3a04ca8db6eacbd6a6118814f0d7f22623280ee7b5e347d1e39b52ab61a8  assets/cmu/CMU_DragBadLegWalk.glb
a426327f5bdb9c3eecf016a32abf2afac3b2999821c43e2b7dc47354f427e241  assets/cmu/CMU_HurtLegWalk.glb
553321a90d4fd5cdb2cd636d2cd96d3d04a3b650c99bc9ac9d3c55429880106d  assets/cmu/CMU_Limp.glb
```

The validator also rejects implausible converted motion scale before an asset
can be committed.


## Retargeting method

The CMU FBX skeleton and SARX Quaternius skeleton use different bone names and
rest axes. SARX therefore does not transfer absolute source world rotations.
The conversion workflow computes each authored source bone's animation delta
relative to its own rest pose, changes that delta into the corresponding
Quaternius bone's rest-space basis, and keys only the resulting target-local
rotation. Translation/root travel remains under SARX runtime authority.

This distinction is evidence-critical: an earlier absolute-world retarget
mapped node names successfully but could rotate the entire target hierarchy
sideways/upside-down while still passing structural tests. The media gate now
also requires an upright head-over-pelvis relationship and sustained intact-foot
support after the authored transition.

The authored CMU hip/root rotation is intentionally not transferred; SARX preserves the agent's existing world-facing direction across the injury-motion handoff.


### Root-motion extraction

Post-injury world travel is no longer a fixed SARX translation. The vendor
pipeline measures CMU hip/root travel, detects and removes the FBX
bind-to-motion discontinuity, normalizes source units by measured source-vs-
Quaternius skeleton height, smooths the horizontal path over a short window,
and exports cumulative path length plus vertical displacement as a
`.root.csv` sidecar. Runtime applies a short post-severance deceleration
before authored root travel begins.


The converter also removes the initial CMU capture calibration pose. For the
currently integrated injury clips the detector trims exactly one source frame,
so the authored handoff begins at the first non-calibration pose without
discarding meaningful gait startup.
