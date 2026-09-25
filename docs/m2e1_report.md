# M2-E1 report: anatomy-aware action substitution and regional animation authority

**Verdict: M2-E1: PARTIAL. E1-B PASS, E1-A BLOCKED_ON_AUTHORED_ELBOW_MOTION.**

PR #8 must stay draft. E1-A cannot pass until a legitimately licensed,
authored elbow-strike clip exists and passes visual audit on Quaternius.
Every other part of E1-A is in place, so adding that clip is a data change,
not an architecture change (see "Binding a certified elbow" below).

## Repository state

| | |
|---|---|
| Continued from | PR #8 head `b3570def387f578093438cf7015e2ae669e8b28e` (`research/m2-action-substitution-v1`), checked against the remote before any change |
| Stacked on | certified M2-D1 `7c3cc0af` (`research/m2-authored-injury-motion-v1`) |
| Work branch | `claude/sarx-m2e1-continuation-b0iedm`, a fast-forward of the PR #8 head |

## Baseline at `b3570de` (before any change)

| Check | Result |
|---|---|
| Build (Release, tests on) | OK |
| `ctest` (16 tests) | 16/16 passed |
| Action audit | Jab: LeftHand, 0.866667 s, contact 0.326923. Cross: RightHand, 1.0 s, contact 0.266667. Unchanged by this work. |
| E1-B media gate (the workflow's own command, `--cut-frame 5`) | **FAILED**: `opposite-hand Cross never reached target`. `min_target_distance=0.32211`, `target_radius=0.180278`, `torso_joint_authority_ok=0`. Log: `m2e1_evidence/baseline_b3570de_e1b.log`. |

The E1-B gate was already failing before this work: it was a pre-existing
failure, not a regression. It also contradicted itself. Commit `39912c5` gave
the Cross `spine_01` authority, while the gate from `d193845` still required
`spine_01` to stay on base animation.

**Root cause.** The Quaternius punches put their hip drive on `pelvis`.
`spine_01` has the same, static quaternion in both clips. The Cross's
authored mechanics were therefore cut off at the wrong joint, and the right
hand fell 0.32 m short. The fix was not to tune the fixture. It was to
derive the authority region from the authored clip itself (below).

## E1-B: whole-arm loss becomes an opposite-hand authored Cross (PASS)

Fixture: `sarx_character_voxel_arm_loss_attack_demo --injury whole-arm`,
run on the real Quaternius character with the voxel/damage authority active.

**Sever timing, declared before the evidence run.** The sever phase is the
midpoint of the Jab's committed, pre-contact interval
`[commitment_phase 0.08, contact_phase_begin 0.24)`, which is 0.16. The
fixture computes the frame from that rule:
`lround(0.16 × 0.866667 × 30) = 4`, giving actual phase 0.1538. No
`--cut-frame` is passed. In the authored Jab, frames 0–4 are the
anticipation pull-back and the whole extension happens between frames 4
and 6. The detached arm therefore carries little forward velocity (maximum
anchor speed 0.25 m/s), and it drops mostly downward. That is physically
consistent with the clip at that instant.

| Evidence | Value |
|---|---|
| Initial action / effector | `Punch_Jab` / LeftHand |
| Region lost | left arm at the shoulder (`upperarm_l`, `lowerarm_l`, `hand_l`) |
| Attached fraction, before → after | upperarm_l 1 → 0.153 (proximal stump), lowerarm_l 1 → 0, hand_l 1 → 0 |
| Detach frame / Jab invalidated / substitute selected / replacement engaged | 4 / 4 / 4 / 5 |
| Physics roots (from voxel topology) | `upperarm_l`: 23 joints are physics-owned |
| Same-side elbow | `RejectedAnatomy: required anatomy unavailable: upperarm_l,lowerarm_l` |
| Selected substitute | `Punch_Cross` (RightHand), score 0.8345 |
| Other candidates | Right elbow: anatomy viable, score 0.7575, `RejectedAuthoredMotionUnavailable`. It also scores lower than the Cross. |
| Authority | Base 1 (`root`), Replacement 41 (derived root `pelvis` subtree), Physics 23 |
| Physics-pose violations | 0 (detached joints held at the detachment pose on every frame) |
| Target | fixed before damage: `(-0.0319, 1.3591, 0.7094)`, radius 0.180 |
| Contact | frame 11, Cross phase 0.233, inside the Cross contact window [0.18, 0.36], hand speed 7.0 m/s |
| Closest approach | 0.103 m at Cross phase 0.333 |
| Largest pelvis step per frame | 0.053 m (limit 0.083) |
| Root joint displacement | 0 |
| Planted lead-foot drift | 0.0025 m (limit 0.0225) |
| Camera | fixed |
| Detached arm | 394 ground contacts, travels 1.20 m, lowest voxel centre 0.022 m during the fall and 0.027 m at rest, no re-attached voxels |
| Unrelated anatomy changed | 0 voxels. Right arm at 1.0 / 1.0 / 1.0. |
| Determinism | digest `b9a5a46a6fe5390d`, identical on an independent replay |

Full data is in `m2e1_evidence/e1b_summary.txt` and
`m2e1_evidence/e1b_frames.tsv` (one row per frame: active action, phase,
blend, effector position, swept target distance, pelvis position, lead-foot
drift, detached centroid and minimum height, authority counts).

MP4: `media/mp4/m2e1_b_quaternius_arm_loss_to_opposite_cross.mp4`.
Detached anatomy renders orange, the target red.

**Visual review: accepted.**
- Frames 0–3: the Jab starts from guard.
- Frame 4: the left arm is severed at the shoulder and turns orange (physics).
- The arm falls and comes to rest on the floor. It neither sinks nor floats.
- Only a shoulder stump remains, and no left arm reappears.
- The right Cross extends into the target around frames 10–15, then returns to guard.
- The root does not teleport, the lead foot does not skate, and the camera does not move.

Two defects were found during review and fixed at the source, not hidden:
1. **The detached arm sank about 11 cm into the floor.** Articulation anchors
   were surface points from the split mesh, and a hard-coded contact radius of
   0.47 × voxel size was used. The fix: anchors are now the joint centres, and
   each segment gets a floor-contact radius measured from the limb's actual
   cross-section.
2. **Spurious 1.2 m/s velocity on the hand-tip anchor.** The previous-frame
   anchor mixed two different reference points. Fixed.

**Gate changes, stated openly.**
- The old `torso_joint_authority_ok` gate (torso must stay base) is replaced
  by the derived-authority check. The authored Cross needs pelvis authority.
- The old `upperarm_l ≤ 0.10` stump threshold was calibrated to frame 5. At
  the derived frame 4 the stump is 15.3%. The replacement check is
  structural and applies to every voxel: no surviving `upperarm_l` voxel may
  lie distal to the cut disk (0 found), and the stump must be too small for
  the same-side elbow's own `upperarm_l ≥ 0.70` requirement.
- The target volume and its radius are unchanged from the PR head.

## Generalizing the action system

**Capability model** (`action_capability.*`)
- New fields: `effector_joint`, `commitment_phase`,
  `authored_motion_available`, `availability_note`.
- `quaternius_attack_action_library()` is the data-driven attack library.
  It binds the audited Jab and Cross and holds left and right elbow slots
  marked "no certified authored elbow-strike clip".
- `bind_authored_motion()` is the only way to make an elbow executable.
- Elbow capabilities now declare the same optional guard regions (opposite
  arm) as punches. An adversarial test showed the opposite elbow had been
  outranking the opposite hand punch only because of this modelling
  asymmetry. This was a real planner defect, and it is now fixed.

**Planner** (`action_substitution.*`)
- A structured per-candidate trace records `Selected`,
  `ViableLowerPriority`, `RejectedCurrentAction`, `RejectedIntentMismatch`,
  `RejectedAnatomy` (with the failed regions) and
  `RejectedAuthoredMotionUnavailable`.
- Anatomy is always evaluated before asset availability, so a missing asset
  is never reported when the body could not perform the action anyway.
- `preferred_blocked_on_authored_motion` is set when the desired
  continuation exists semantically but has no clip.
- `intent_failed` is set when no chain survives. No strike is selected and
  no limb is animated.
- `describe_action_plan()` prints a readable trace.
- `ActionExecutionController` re-evaluates on every tick. It supports
  repeated substitution (a second or third injury) and never reselects an
  action it abandoned for anatomical reasons.

**Animation authority** (`animation_authority.*`)
- `derive_authored_authority_root()` walks the effector's ancestor chain,
  freezes each joint's authored channels in turn, and measures how far the
  effector moves as a result. The most proximal joint that moves it by at
  least 10% of the effector's travel becomes the replacement root. For both
  Jab and Cross this is `pelvis` (Cross contributions: pelvis 0.239 m,
  spine_01 0, spine_02 0.146 m, spine_03 0.355 m). The replacement region
  therefore comes from the authored motion, not a hand-picked joint list.
- `physics_joint_roots_from_voxels()` makes a joint physics-owned when most
  of the voxels it dominates are no longer attached and some of them are in
  a detached component.
- `compose_action_local_poses(..., physics_hold)`: physics-owned joints take
  the pose held at detachment. That makes the attached stump rigid with its
  parent, and neither base nor replacement animation can move a detached
  branch.

**No case-specific logic.** The fixture never branches on an animation name
to pick a substitute. The only name checks are assertions of the expected
outcome after planning.

**Adversarial tests** (`tests/test_main.cpp`, all passing)
- Library binding.
- Committed hand loss: the elbow is identified but blocked, and the fallback
  is the Cross.
- Pre-commit hand loss: the opposite hand is chosen. Once an elbow clip is
  bound, a committed hand loss selects the elbow.
- Whole-arm loss on both sides: the elbow is rejected for `upperarm`
  anatomy.
- Elbow boundary: 15/100 forearm voxels is viable, 14/100 is invalid, on
  both sides. A damaged but still connected chain stays viable.
- No surviving chain: intent fails, and restored anatomy data does not
  bring it back.
- Replanning through three injuries in sequence.
- The physics hold blocks both base and replacement channels.
- A five-state capability matrix.

**Real-character ctest entries:** `sarx_m2e1_whole-arm_substitution` and
`sarx_m2e1_hand_substitution`.

**Animation cannot outrun anatomy (Task 7).**
- Detached descendants are physics-owned by topology.
- Their local poses are frozen at detachment, which is checked every frame
  (`physics_pose_violations=0`).
- The detached component is rendered only by its own articulation.
- Voxel re-attachment is checked every frame (0).
- A substitute whose chain contains detached anatomy is rejected at the
  capability stage.

Known limitation: `evaluate_motion_viability` treats a region missing from
the anatomy list as fully attached. Fixtures always pass the full voxel
anatomy, but callers must not pass partial lists.

## E1-A: hand loss during a committed Jab becomes a same-side authored elbow (BLOCKED)

The infrastructure runs today with `--injury hand`:
- The wrist is severed at the same derived phase (0.1538).
- The hand becomes physics, with `physics_joint_roots=hand_l` and 21 physics joints.
- The Jab is invalidated at frame 4.
- The trace shows `ElbowStrike_Left` as anatomy viable with total score 1.0,
  but `RejectedAuthoredMotionUnavailable`, with
  `preferred_blocked_on_authored_motion=ElbowStrike_Left_Uncertified`.
- The executable fallback is the Cross, which reaches contact at phase 0.233.
- Evidence: `m2e1_evidence/e1a_blocked_summary.txt` and
  `media/mp4/m2e1_a_quaternius_hand_loss_elbow_blocked_fallback.mp4`.
- The media workflow also runs `--require-authored-elbow`, which fails as
  intended, and records `E1A_STATUS=BLOCKED_ON_AUTHORED_ELBOW_MOTION`.

**This is not E1-A evidence.** It shows the blocked path behaving correctly.
No elbow motion was synthesised, relabelled or keyframed.

**Binding a certified elbow.** Once a clip passes review, run the fixture with:

```
--injury hand --elbow-animations <retargeted.glb> --elbow-clip <name> \
  --certified-elbow --require-authored-elbow <all --require-* gates>
```

The fixture then binds the left elbow slot, derives its authority region from
the clip (effector `lowerarm_l`), and applies the same target, contact-window,
authority, continuity and determinism gates.

Open item for whoever supplies the asset: an elbow's reach is much shorter
than a hand's. The planner does not model reach yet, so the fixed shared
target may be out of range for a genuine elbow thrown from Jab distance. If
that happens the result is a FAIL. Do not move the target.

## E1-A asset search (this session)

See the ledger in `docs/m2e1_combat_motion_sources.md`. No candidate passed.
Nothing was downloaded or converted.

## Regression results (local, this container)

| Check | Result |
|---|---|
| `ctest` (18 tests: unit, real-character smokes, both M2-E1 injury modes) | 18/18 passed |
| Action audit | unchanged |
| E1-B full gates, 120 frames | pass. Replay digest identical. |
| E1-A blocked-path gates | pass. The strict authored-elbow gate fails, as intended. |
| M2-D1 CMU_Limp complete-foot-loss gates (workflow command) | pass. Diagnostics identical to the `b3570de` binary, and frame 150 is byte-identical. Log: `m2e1_evidence/m2d1_regression.log`. |

GitHub Actions results for this branch are recorded in the PR/session report.
