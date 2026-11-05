# Getting started

This page assumes the installation in [installation.md](installation.md) is done. All commands run from the
repository root.

## The tasks

The tasks are the ContactBench 1.0 tasks of the paper, with the paper's numbering and success criteria: 22 base tasks
with a scripted policy and an automatic verdict, in 73 rows, and task 2 as a scene without a scripted policy, for
teleoperation and policies of your own. `force-bench list` shows the rows; [tasks.md](tasks.md) describes each task
and lists the tasks that are not included, and why.

## Run a task

```bash
force-bench list                          # what is available
force-bench info plug_in_usb_2            # what this row does, how it starts, how it differs from the real task
force-bench run plug_in_usb_2             # run it
```

`force-bench run` builds the scene, runs the scripted policy, records everything and writes the verdict. It prints
the logged contact forces as it goes and ends with a line such as

```
[appendix] Plug in USB: PASS -> .../outputs/appendix_tasks/plug_in_usb_2/appendix_qa.json
```

The `QA PASS` / `QA FAIL` line printed just before it summarises `rollout_qa.json`: the generic physics checks of the
rollout and, for some tasks, checks of the scene setup that are not part of the paper's criterion. It is not the
benchmark result. Only the physics-integrity checks enter the verdict, so a task can pass while `rollout_qa.json`
reports a failed setup check (for example a minimum overhang before the pinch).

`force-bench info` also prints the Python module that implements a row and its arguments;
`python -m force_bench.appendix_tasks.plug_in_usb --usb 2` runs the same as `force-bench run plug_in_usb_2`. Options of
the module itself pass through `force-bench run` (for example `force-bench run plug_in_ethernet --port 3`).

## Variations and random starts

The rows of `force-bench list` are presets of the variation axes the project website lists for each task (a stiffer
spring, another USB stick, a longer screw). `--set` runs any other value of an axis, under the website's names and
the asset page's labels, and the tasks that support it take `--seed N` for a reproducible random start within the
paper's 0.15 × 0.15 m square:

```bash
force-bench variations plug_in_usb_2                          # the axes this row can vary, its own choices marked *
force-bench run plug_in_usb_2 --set "Insertion force=0.6mm"   # -> outputs/appendix_tasks/plug_in_usb_2__force-0.6mm-indent-spring/
force-bench run plug_in_ethernet --seed 3                     # -> outputs/appendix_tasks/plug_in_ethernet__seed3/
```

[variations.md](variations.md) lists every task's axes and values, the tasks with random starts and their ranges.

## Other fingers

The tasks run with the FR3's stock fingers. A task that supports another finger set (an *embodiment*) runs with it
under `--embodiment`, into its own output folder; the option combines with `--seed` and `--set`:

```bash
force-bench embodiments                                          # the finger sets, and the tasks that support each
force-bench run retrieve_utensil_from_bin --embodiment finray    # -> outputs/appendix_tasks/retrieve_utensil_from_bin__finray/
```

[embodiments.md](embodiments.md) describes the finger sets and how to add your own.

## What a run writes

Everything goes to `outputs/appendix_tasks/<name>/`:

| file | contents |
|---|---|
| `rollout.mp4` | the main camera, 1280 × 960 at 60 fps |
| `rollout_multiview.mp4` | the main camera and a second view side by side (turn off with `TB_MULTIVIEW=0`) |
| `rollout_forces.csv` | one row per frame: time and the logged contact forces in newtons, both total and frictional, per probed body (for example each finger pad, the manipulated object, robot-object contact), plus task-specific measurements such as seat depth or rotation angle |
| `rollout_forces.png` | a plot of the same forces |
| `rollout_states.npz` | per frame: `t`, `body_q` (position + quaternion `x, y, z, qx, qy, qz, qw` of every moving body, in metres), `body_qd` (velocities), and `body_labels` naming the bodies (robot links and task objects) |
| `appendix_qa.json` | the benchmark verdict: the paper's criterion, each check with its measurements, the physics-integrity checks and the overall `pass` (see [tasks.md](tasks.md#how-a-run-is-judged)) |
| `rollout_qa.json` | the generic physics checks of the rollout (finite values, velocity spikes, and the scene's own checks) |
| `scene.json` | task 2 only, in place of the two verdicts: the arrangement, the fill, the start and the state of the paper's criterion at the end |

Some tasks write extra files, for example `expert_log.json` with the phases of the scripted policy. Units are SI
throughout (metres, kilograms, newtons, seconds, radians) unless a column name says otherwise (`_mm`, `_deg`).

Reading a verdict:

```python
import json
qa = json.load(open("outputs/appendix_tasks/plug_in_usb_2/appendix_qa.json"))
print(qa["pass"])
for name, check in qa["criteria"].items():
    print(name, check["pass"])
print(qa["physics_integrity"])
```

## Run many tasks

```bash
force-bench run-all -j 3                                 # every row with a scripted policy, three processes at a time
force-bench run-all -j 2 plug_in_usb_1 turn_dial_to_target
force-bench run-all --summary                            # only re-collect the verdicts
```

Each row runs in its own process, so rows never share state. Logs go to `outputs/appendix_tasks/_logs/<name>.log`,
and `outputs/appendix_tasks/summary.json` collects every verdict. `-j` defaults to 3. One process uses about 1 to
2 GB of GPU memory; parallel runs are limited by the GPU's compute, which three or four processes saturate on an
RTX 5090.

## Watch, drive and record

- **Teleoperation.** `force-bench teleop-ui --scene <task>` opens a ContactBench task at its start for driving the
  arm by hand with a keyboard, gamepad or SpaceMouse, with live contact forces and alarms; `--ui web` serves the same
  view to a browser. See [teleoperation.md](teleoperation.md).
- **Demonstrations from the scripted policies.** `force-bench collect --task <name> --episodes N` runs a task in
  fresh processes and stores aligned observations, actions and contact forces per episode, with the task's verdict
  as the episode outcome. See [data.md](data.md).
- **Tactile sensing.** `TB_GELSIGHT=1` mounts simulated GelSight Mini sensors on the fingers of the tasks that
  support it (`TB_GELSIGHT=1 force-bench run retrieve_card_from_deck`) and records tactile images, pressure and shear
  fields. See [tactile.md](tactile.md).

## Environment variables

| variable | effect |
|---|---|
| `TB_OUT_ROOT=<dir>` | write all outputs under `<dir>` instead of `outputs/` (useful for repeated runs, so they do not overwrite each other) |
| `TB_RENDER_FAST=1` | render 800 × 600 draft videos without supersampling |
| `TB_MULTIVIEW=0` | skip the second camera view |
| `TB_LOOK=studio` | render with the generic studio palette instead of the kit colours (see [assets.md](assets.md#colours)) |
| `TB_GELSIGHT=1` | mount the simulated GelSight Mini sensors (the tasks marked `[TB_GELSIGHT=1]` in `force-bench list`) |
| `TB_GELSIGHT_RENDER=0` | with `TB_GELSIGHT=1`, skip the tactile image rendering and keep the pressure and shear fields |
| `TB_ASSETS_ROOT=<dir>` | where to find `assets/manifest.yaml` if the package cannot find it by itself |
| `NEWTON_CACHE_PATH=<dir>` | where Newton caches the downloaded robot description |

Some task modules read further `TB_*` variables (solver settings, alternative camera poses, debugging stops); each is
documented in the docstring of the module that reads it. None is needed for normal use.

## Use the kit's objects in your own scene

All 49 assets of the printable kit, with every variant on the project's asset page, are a library
(`force_bench.kit`):

```bash
force-bench kit list                      # the assets, their kind and their variants
force-bench kit info usb-slot             # parts, sizes, estimated print masses, variants, physical model
python examples/kit_scene/cup_stack.py    # a minimal scene: three kit cups dropped into each other
```

```python
from force_bench.rig import Rig
from force_bench.kit import add_asset, place_on_table

rig = Rig("my_scene")                     # the scene builder the tasks use
h = add_asset(rig, "outlet", "0.85mm", pose=place_on_table("outlet", xy=(0.45, 0.0)))
```

[assets.md](assets.md#the-kit-library) explains how each asset is simulated, and
[examples/kit_scene](../examples/kit_scene/README.md) walks through a scene.

An asset can also go into a Newton scene of your own, next to your own robot, as a USD stage with its joints, springs
and contact settings (needs the `usd` extra, [installation.md](installation.md)):

```bash
force-bench kit export-usd doorknob-module --variant "Handle 1"   # -> outputs/kit_usd/doorknob-module/Handle_1.usda
python examples/custom_scene/ur10_door_handle.py                  # Newton's UR10 pushes the stage's lever handle
```

`force_bench.kit.usd.add_asset_usd` adds a stage to any `newton.ModelBuilder`. [custom_scenes.md](custom_scenes.md)
describes both routes (Python and USD), the solver settings the kit's calibration assumes, and what the USD route
reproduces.

## Change a physical parameter

Physical parameters are Python constants at the top of the task, scene and kit-mechanism modules (masses, friction
coefficients, spring stiffnesses and preloads, start offsets), each with a comment on its unit and origin. Many tasks
also take the asset page's variants as options (`--set`, above, or the module's own `--spring`, `--unlock`, `--usb`,
...; `force-bench info <row>` lists a row's arguments and axes). To vary a constant, set it before calling the
module's `main()`, and send the outputs elsewhere so the default run is not overwritten:

```python
# TB_OUT_ROOT=outputs/stiff_lever python stiff_lever.py
import warp as wp
import force_bench.appendix_tasks.turn_door_handle_open_door_flat as T
from force_bench.kit.mechanisms import doorknob_module

# the lever's printed 5 mm return spring: K [N·m/rad], preload [N·m] (measured fit: 0.48, 0.15)
doorknob_module.SPRINGS["5mm spring"] = (0.60, 0.20)
wp.init()
T.main([])
```

The scripted policies are tuned at the default values. Their feedback loops usually absorb moderate changes; large
ones can make them fail, which the verdict reports. [examples/force_response](../examples/force_response/README.md)
shows the force a robot feels as a spring's stiffness and preload change, and lists values fitted to measurements of
printed parts. [physics_notes.md](physics_notes.md) collects the contact-modelling choices that matter when you change
geometry or parameters.
