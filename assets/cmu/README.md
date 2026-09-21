# CMU injury motion integration

SARX uses selected authored injury locomotion motions from the Carnegie Mellon
University Graphics Lab Motion Capture Database as M2 recovery references.

Selected clips:

| SARX ID | CMU clip | Description |
| --- | --- | --- |
| CMU_Limp | 91_16 | Limp |
| CMU_HurtLegWalk | 91_24 | HurtLegWalk |
| CMU_DragBadLegWalk | 91_25 | DragBadLegWalk |

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
