# SARX result media

This directory is reserved for reproducible visual results from validated SARX milestones.

## Layout

- `media/mp4/` — final compressed MP4 captures suitable for direct viewing.
- `media/gif/` — short README-friendly previews generated from the matching MP4.
- `media/raw/` — local/raw captures; gitignored.

Do not add synthetic placeholder results. A clip should correspond to a documented executable/configuration and a commit SHA.

## Naming

Use:

```text
<milestone>_<fixture>_<behavior>.<ext>
```

Examples:

```text
v05_humanoid_shoulder_severance.mp4
v05_humanoid_shoulder_severance.gif
v05_humanoid_partial_cut_then_detach.mp4
```

## MP4 -> GIF preview

Once a final MP4 exists, a compact two-pass ffmpeg conversion is recommended:

```bash
ffmpeg -i media/mp4/v05_humanoid_shoulder_severance.mp4 \
  -vf "fps=15,scale=720:-1:flags=lanczos,palettegen" \
  /tmp/sarx_palette.png

ffmpeg -i media/mp4/v05_humanoid_shoulder_severance.mp4 \
  -i /tmp/sarx_palette.png \
  -lavfi "fps=15,scale=720:-1:flags=lanczos[x];[x][1:v]paletteuse" \
  media/gif/v05_humanoid_shoulder_severance.gif
```

The MP4 remains the higher-quality primary result; the GIF is only a convenient README preview.

## Reproducibility note

For each published clip, record in the README or a sibling Markdown file:

- SARX commit SHA,
- fixture/configuration,
- timestep/substeps/solver iterations,
- active-domain settings,
- renderer/capture settings,
- whether the run used CPU Body, CPU SoA, or a future GPU backend.
