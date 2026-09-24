# v11 — Quaternius fall to grounded recovery

- **Source:** GitHub Actions "Render SARX demo media" run #228 (run id 35717272895), artifact `sarx-character-integration-media`
- **Commit:** PR #8 head `b3570def387f578093438cf7015e2ae669e8b28e` (`research/m2-action-substitution-v1`); CI merge commit `8b5a3208ebb03c280cc669d89c754233b791286c`
- **Fixture:** Quaternius character voxelized at `voxel_size=0.045` (1601 voxels), Walk clip, left-thigh cut
- **Frames:** 270 at 30 fps, 960×720, libx264 CRF 20
- **Backend:** CPU Body
- **Timeline:** cut and detach at frame 76 → `Fall` strategy (`Death01`), completed frame 149 → grounded `Kneel` strategy (`Fixing_Kneeling`), pose engaged frame 161
- **Key metrics:** destroyed_voxels=37, detached_voxels=205, unrelated_changed_voxels=0, fall_body_ground_contacts=3, grounded_centroid_rise=0.641

CI writes this same clip as `v09_quaternius_voxel_left_thigh_cut.mp4` and `v10_quaternius_walk_failure_to_fall.mp4` too; all three files are identical because one demo run covers the cut, the fall and the recovery.

GIF preview: 12 fps, 480 px wide, 64-color palette (reduced from the default recipe in `media/README.md` to keep the README preview about 2.6 MB).
