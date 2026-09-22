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
