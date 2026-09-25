# M2-E1 Combat Motion Sources

## Certified Quaternius base actions

The built-in Quaternius animation library was measured directly by
`sarx_character_action_audit` at 60 Hz.

| Motion | Primary effector | Measured peak-extension phase | Role |
| --- | --- | ---: | --- |
| `Punch_Jab` | Left hand | 0.326923 | canonical interrupted attack |
| `Punch_Cross` | Right hand | 0.266667 | opposite-arm fallback |

These semantics are pinned in unit tests and should not be inferred from the
animation names alone.

## CMU elbow-source audition

The following CMU motions were legally downloaded from the CMU conversion
dataset and retargeted onto the real Quaternius character for visual audition:

- `CMU_PunchStrike` — CMU 2_05
- `CMU_Empi_A` — CMU 135_02
- `CMU_Empi_B` — CMU 135_03
- `CMU_LeftPunchSequence` — CMU 144_13
- `CMU_PunchSequence` — CMU 144_20

**None is certified as the M2-E1 elbow replacement.**

The punch sequences are hand-strike sequences. The Empi trials are broader
karate kata motions rather than a clean isolated elbow strike suitable for the
canonical hand-loss substitution proof. They remain negative research evidence
only and are not production action capabilities.

Do not rename, relabel, or automatically promote these motions to
`ElbowStrike` merely to satisfy the E1-A test.

## Elbow candidate ledger (2026-09-25 search)

Bounded search for a legitimately licensed, authored, isolated elbow strike.
**No candidate was accepted for audition.** No motion file was downloaded.

Network notes: the session egress proxy blocked huggingface.co (so the
gbionics/cmu-fbx tree could not be listed), mocap.cs.cmu.edu, quaternius.com,
itch.io, opengameart.org and accad.osu.edu. Candidates were assessed from
mirrored indices on GitHub and from published clip lists.

| Source | Licence | Semantic assessment | Verdict |
| --- | --- | --- | --- |
| CMU mocap, full trial index (every subject) | CMU terms (free use, no resale) | No trial description mentions an elbow strike. The "elbow" hits are 13_06 (hands to chin), 18_05/06 and 19_05/06 (pulling a partner by the elbow), and 49_22 (dance). Subject 135: Bassai, Empi ×2, front kick, gedanbarai, heian shodan, mawashigeri, oiduki, syutouuke, yokogeri. No isolated elbow. Subject 144: punches, kicks, blocks. | REJECT: no elbow trial beyond the already-rejected Empi kata |
| Quaternius UAL1 (in repo, CC0) | CC0 | Combat clips are only Punch_Jab, Punch_Cross and Sword_* | REJECT |
| Quaternius UAL2 Standard | CC0 | Melee_Hook, OverhandThrow, Sword_*. No elbow. | REJECT |
| Quaternius UAL Pro/Source tiers | CC0, but paid | Clip list could not be verified | UNVERIFIED: best clean-licence lead if purchased and checked |
| HumanML3D captions over AMASS | AMASS: non-commercial research only | The only "elbow" fight caption, HDM05 `HDM_tr_03-02_02`, is outvoted by two other annotators (kick/shadowbox) | REJECT (annotation noise, licence) |
| BABEL action labels | AMASS licence | No elbow category | REJECT |
| ACCAD Male2 MartialArts | CC BY 3.0 | Jab, cross, hook, uppercut, backfist, blocks. No elbow. | REJECT |
| Bandai Namco Research Motiondataset 1/2 | CC BY-NC 4.0 | Punch, kick, slash. No elbow. | REJECT |
| LAFAN1 | CC BY-NC-ND 4.0 | Mixed fight clips. No-derivatives forbids retargeting. | REJECT (licence) |
| SFU, KIT, BMLmovi, HumanAct12, TotalCapture | Various, mostly non-commercial | No elbow strikes | REJECT |
| Kyokushin karate dataset (figshare) | CC BY 4.0 | Gyaku-zuki and three kicks only. Raw C3D markers. | REJECT |
| SLMP combat dataset (arXiv 2603.01294) | Unknown | Paper mentions knees and elbows (kickboxing), but the data is unreleased | REJECT for now: re-check when released |
| MAAIP QwanKiDo | Unknown | Elbows mentioned. No public release. | REJECT |
| MocapFlow free library | CC BY 4.0 badge, but provenance unclear and downloads behind ads | No elbow clip confirmed | REJECT |
| Mixamo | Adobe terms forbid redistributing standalone animation files | Unverified | REJECT (licence) |
| Commercial stores (Rapa, MoCap Online, Reallusion, Fab) | Store licences | May contain elbows | REJECT unless acquired separately with terms recorded |

Realistic routes to unblock E1-A:
1. Buy Quaternius UAL Pro/Source after confirming an elbow clip in its viewer.
2. Wait for the SLMP combat dataset release and check its licence.
3. Commission an artist-authored elbow clip and record its provenance.

Whichever route is taken, the clip must then pass the audition workflow and
`--injury hand --certified-elbow --require-authored-elbow`.

## Production elbow requirement

E1-A remains open until SARX has a legally usable authored clip whose visual
motion is genuinely an elbow/forearm strike after retargeting to Quaternius.
The clip must then pass the real hand-severance fixture:

`Punch_Jab -> left hand severed -> left elbow replacement -> same original target`.

The target must remain the original Jab target; it must not be repositioned onto
the replacement trajectory.

## Asset policy

Third-party motion data must have verifiable rights for the intended SARX use
before it is committed as production content. Paid/store assets must not be
copied from scraper, mirror, or piracy repositories. If a licensed asset is
acquired separately, record its source/license terms here before certification.
