# Third-party notices

The code and assets in this repository are released under the MIT license (see `LICENSE`), except for the
items below, which keep their own licenses.

## Included in this repository

| item | where | source | license |
|---|---|---|---|
| GelSight Mini optical calibration table and background image | `packages/hydrogel/src/hydrogel/data/gs_mini/polycalib.npz`, `bg.png` | [HydroShear](https://github.com/MMintLab/hydroshear) (MMint Lab, University of Michigan), commit `f815b82` | MIT (copy in that directory, with `PROVENANCE.md`) |

## Downloaded at run time (not included)

| item | fetched by | source | license |
|---|---|---|---|
| Franka FR3 arm and hand description (URDF and meshes) | `newton.utils.download_asset("franka_emika_panda")` on first use | [newton-assets](https://github.com/newton-physics/newton-assets) | Apache-2.0 |

## Not included

| item | why | how to add it |
|---|---|---|
| GelSight Inc. GS Mini housing CAD | it is distributed in [gelsightinc/gsrobotics](https://github.com/gelsightinc/gsrobotics) under GPL-3.0 | optional and visual only; see `packages/hydrogel/src/hydrogel/data/gs_mini/PROVENANCE.md` |

## Methods and dependencies

The simulation is built on [Newton](https://github.com/newton-physics/newton) (Apache-2.0),
[MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) and [MuJoCo](https://github.com/google-deepmind/mujoco)
(Apache-2.0) and [NVIDIA Warp](https://github.com/NVIDIA/warp) (Apache-2.0), installed from PyPI. The tactile image
rendering follows Taxim (Si and Yuan, "Taxim: An example-based simulation model for GelSight tactile sensors",
IEEE RA-L 2022) as implemented in HydroShear and NVIDIA TacSL; the gel contact uses the hydroelastic pressure-field
model of Elandt et al. (IROS 2019) as implemented in Newton.
