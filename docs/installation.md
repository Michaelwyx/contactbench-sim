# Installation

The simulation runs on an NVIDIA GPU under Linux. Everything installs with `pip` into one Python environment; there
is no build step.

## Requirements

| | |
|---|---|
| OS | Linux x86_64, tested on Ubuntu 22.04. |
| GPU | NVIDIA GPU with a CUDA 12 capable driver, tested on an RTX 5090 (32 GB). One task process uses about 1 to 2 GB of GPU memory; runs with GelSight pads use more. |
| Python | 3.10 or newer, tested with 3.12. |
| Disk | About 150 MB for the asset meshes in this repository, about 100 MB for the Franka FR3 description downloaded on first use, about 1 GB for Warp's kernel cache, plus outputs (a task writes 10 to 100 MB of video, logs and state). |
| Network | Needed once, on the first run, to download the FR3 description from GitHub (see below). |

Running tasks needs no display: videos are rendered on the GPU by Newton's ray-traced camera sensor. The
teleoperation window (`force-bench teleop-ui --ui gl`) needs a display; the browser interface (`--ui web`) does not.

## Install

```bash
git clone https://github.com/Michaelwyx/contactbench-sim.git
cd contactbench-sim

conda create -n contactbench python=3.12 -y      # or: python3.12 -m venv .venv && source .venv/bin/activate
conda activate contactbench

pip install -e packages/hydrogel -e "packages/force-bench[hydrogel,teleop]"
```

This installs two packages in editable mode:

- `force-bench` (import name `force_bench`): the assets, the tasks, the scripted policies, the success checks, the
  renderer, teleoperation and data recording. It provides the `force-bench` command.
- `hydrogel`: the simulated GelSight Mini sensor, with the `hydrogel` command. Tasks use it only with
  `TB_GELSIGHT=1`, but the tactile scenes and tests import it, so install it unless you are sure you do not need it.

The physics stack is pinned to exact versions: `newton==1.5.0`, `warp-lang==1.16.0`, `mujoco-warp==3.11.0`,
`mujoco==3.11.0`. `hydrogel` patches one MuJoCo Warp kernel of that Newton release, and the assets' contact
parameters are calibrated on it, so do not upgrade these packages independently.

Optional extras of `force-bench`:

| extra | adds | needed for |
|---|---|---|
| `hydrogel` | the `hydrogel` package | GelSight pads (`TB_GELSIGHT=1`) |
| `teleop` | `pyglet`, `imgui-bundle`, `websockets` | `force-bench teleop-ui` (GL window and browser page) |
| `usd` | `usd-core`, `newton-usd-schemas` 0.4.1 (the schema release Newton 1.5.0 is built against) | `force-bench kit export-usd`; `force_bench.kit.usd.add_asset_usd` and `force-bench kit check --from-usd`, which load the stages into Newton ([custom_scenes.md](custom_scenes.md)) |
| `dev` | `pytest`, `ruff` | running the tests and the linter |

Install several at once, for example `pip install -e "packages/force-bench[hydrogel,teleop,usd,dev]"`.

### The asset meshes

The meshes of the printed objects (`assets/kit/`, the final kit as the asset page publishes it, and
`assets/bodies/`, `assets/bodies_kit/`, meshes some tasks load besides the kit) and their physical parameters
(`assets/kit/sim.yaml`, `assets/manifest.yaml`) live at the repository root, not in the Python package.
`force_bench` finds them by walking up from its own source directory, which works for an editable install from a
checkout. If you install the package some other way, point `TB_ASSETS_ROOT` at the directory that contains
`assets/manifest.yaml`.

### The robot description (downloaded on first use)

The Franka FR3 arm and hand come from the `franka_emika_panda` folder of
[newton-assets](https://github.com/newton-physics/newton-assets). The first task you run fetches it with
`newton.utils.download_asset`, which clones that folder at the commit pinned by Newton 1.5.0 with GitPython, so the
`git` executable must be on your `PATH`. The download is cached in `~/.cache/newton` and reused afterwards; set
`NEWTON_CACHE_PATH` to cache it elsewhere. On a machine without internet access, run any task once on a connected
machine and copy the cache directory over.

### The GelSight housing (optional)

The simulated GelSight Mini collides and renders as a box-shaped case. GelSight's own housing CAD (GPL-3.0, from
[gelsightinc/gsrobotics](https://github.com/gelsightinc/gsrobotics)) is not distributed with this repository; to
render the real housing, install it locally:

```bash
pip install cascadio                              # STEP reader for trimesh
hydrogel housing --download                       # fetch gsmini_shell.zip, convert, install
hydrogel housing ~/Downloads/gsmini_shell.zip     # or from a file you downloaded (.zip or .stp)
```

The housing changes only the renders; collision and mass stay on the box. Details:
[tactile.md](tactile.md#the-sensor-housing-gelsights-cad-installed-locally).

## Check the installation

```bash
force-bench list                      # the benchmark tasks; pure Python, no GPU needed
pytest -q packages                    # unit and physics-contract tests (needs the dev extra and a GPU)
force-bench run plug_in_usb_2         # one full task: about 3 minutes on an RTX 5090
```

The first run of anything that touches the GPU compiles Warp kernels, which takes a few minutes. The compiled kernels
are cached in `~/.cache/warp`, so later runs start in seconds. Tasks with mesh SDF colliders also build those SDFs on
first use and cache them in `$TMPDIR/tactile_bench_sdf_cache` (`/tmp/...` by default).

`force-bench run plug_in_usb_2` ends with a line like

```
[appendix] Plug in USB: PASS -> .../outputs/appendix_tasks/plug_in_usb_2/appendix_qa.json
```

and leaves a video at `outputs/appendix_tasks/plug_in_usb_2/rollout.mp4`. [getting_started.md](getting_started.md)
explains what else is in that folder.

## Troubleshooting

**`ImportError: GitPython package is required` or a git error on the first run.** The FR3 description could not be
downloaded. Check that `git` is installed and that `github.com` is reachable, or copy a populated `~/.cache/newton`
from another machine (see above).

**`FileNotFoundError: force_bench cannot find the benchmark assets`.** The package cannot see `assets/manifest.yaml`.
Install in editable mode from the checkout, run from inside the checkout, or set
`TB_ASSETS_ROOT=/path/to/contactbench-sim`.

**CUDA out of memory.** Run fewer tasks at once (`force-bench run-all -j 1`), close other GPU programs, or render
smaller draft videos with `TB_RENDER_FAST=1`.

**The first run seems to hang.** It is most likely compiling Warp kernels or building SDFs; both are cached
afterwards. The run prints its log as it proceeds.

**Different numbers on a different GPU.** Contact simulation on the GPU is not bitwise reproducible across GPU models
and driver versions. The success checks have margins for this, but small differences in the logged forces are
expected.
