# GelSight Mini optical calibration — provenance

Vendored 2026-08-20 from the HydroShear repository (MIT license, see
`LICENSE` in this directory):

- repo: https://github.com/MMintLab/hydroshear
- commit: `f815b82fdf3451852acd918933020a82cede1f3b` (master, 2026-07-06)
- `polycalib.npz` <- `rl/tacsl_sensors/tactile_utils/gs_mini_data/polycalib.npz`
- `bg.png` <- `rl/tacsl_sensors/tactile_utils/gs_mini_data/`
  `mmint_gs_mini_2D9C-N227_07072025_bg_R_no_dots.png`
  (a real GelSight Mini background image, unit 2D9C-N227, captured
  2025-07-07, markers removed)

The calibration is a Taxim-style example-based table (Taxim:
arXiv 2109.04027), recalibrated by the HydroShear authors (MMint Lab,
University of Michigan) against their own GelSight Mini. HydroShear's
renderer carries these constants for it (`conf_gs_mini` in
`rl/tacsl_sensors/tactile_utils/gelsight_render.py`, itself derived from
NVIDIA TacSL's tensorized Taxim implementation):

- image 320 (h) x 240 (w), `pixmm = 0.065` mm/pixel
- `numBins = 120` used for gradient binning at render time

## Schema of `polycalib.npz`

| key | shape | meaning |
|---|---|---|
| `bins` | scalar int (125) | bin count the table was fit with |
| `grad_r` | (125, 125, 6) | per-(mag_bin, dir_bin) polynomial coeffs, R |
| `grad_g` | (125, 125, 6) | same, G |
| `grad_b` | (125, 125, 6) | same, B |

Lookup (replicated in `sim/gelsight_render.py`): height map in pixel
units -> `np.gradient` -> gradient magnitude squashed by `arctan`, binned
over [0, pi/2] with `numBins=120`; direction `arctan2(dzdx, dzdy)` binned
over [-pi, pi]. Each pixel evaluates its bin's 6-vector against
`[x^2, y^2, x*y, x, y, 1]` of the PIXEL COORDINATES (position-dependent
illumination), summed per channel, added to `bg.png`, clipped to [0, 255].

Note: the table is 125x125 but binning uses 120 bins (the HydroShear/TacSL
convention) — indices stay in range because the extra rows are headroom.

Fallback source if this calibration is ever found lacking: TacEx's
`gpu_taxim` assets (arXiv 2411.04776).

## gsmini_shell.obj (not shipped)

The renders can show GelSight Inc.'s official GS Mini housing instead of the parametric case box. The housing CAD is
`docs/CAD/gsmini_shell.zip` (one STEP file, `gsmini_shell.stp`) in the
[gelsightinc/gsrobotics](https://github.com/gelsightinc/gsrobotics) repository, which is licensed under GPL-3.0, so
it is **not distributed with this MIT-licensed package**. Install it locally:

```bash
pip install cascadio                                   # STEP reader for trimesh
hydrogel housing --download                            # fetch gsmini_shell.zip, convert, install
hydrogel housing ~/Downloads/gsmini_shell.zip          # or from a file you downloaded (.zip or .stp)
```

`hydrogel housing` converts the STEP file to one triangle mesh in metres, checks its size (32.0 × 28.25 × 28.89 mm,
the CAD's own frame: the elastomer at +y, the mounting plate at -y) and saves it here as `gsmini_shell.obj` (the
repository's `.gitignore` keeps it out of commits). `force_bench.tactile.rig` then turns it 180° about its centre,
drops the CAD's elastomer (the simulated gel takes its place) and mounts it on each finger. It is visual only:
collision and mass stay on the parametric case box, which the housing envelopes, so results do not change. Without the
file, or with `GS_SHELL_VISUAL=0`, the case box is rendered.
