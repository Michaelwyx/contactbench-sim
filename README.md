<p align="center">
  <a href="https://jingyi-xiang.github.io/contactbench/"><img src="docs/media/teaser.jpg" width="100%"
    alt="ContactBench 1.0: 3D-printable assets for contact-rich manipulation tasks, the simulation suite, and robot evaluation"></a>
</p>

<h2 align="center">ContactBench 1.0: A Benchmark for Everyday Contact-Rich Robotic Manipulation</h2>

<h3 align="center">
  <a href="https://arxiv.org/abs/2610.24512">Paper</a> |
  <a href="https://jingyi-xiang.github.io/contactbench/">Project Page</a> |
  <a href="https://jingyi-xiang.github.io/contactbench/tasks.html">Benchmark Tasks</a> |
  <a href="https://jingyi-xiang.github.io/contactbench/assets.html">3D-Printable Asset Models</a>
</h3>

ContactBench defines everyday manipulation tasks around eight contact-rich skills, with 3D-printable objects
whose mechanisms (springs, latches, threads, gates) produce the forces a robot has to deal with. This repository
is the simulated counterpart: the same objects and tasks in [Newton](https://github.com/newton-physics/newton)
with the [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) solver, driven by a Franka FR3 arm.

What is included:

- **22 of the paper's 31 base tasks, as 73 runnable rows**, each with a scripted policy and an automatic check of
  the paper's success criterion, and **task 2 as a scene** without a scripted policy, for teleoperation and policies of
  your own. Why the other tasks are not included: [docs/tasks.md](docs/tasks.md#tasks-not-included).
- **The whole printable kit, sim-ready**: all 49 assets of the project's
  [asset page](https://jingyi-xiang.github.io/contactbench/assets.html) with every variant, as calibrated
  mechanisms (printed springs as spring–damper joints with stiffness and preload fitted to measured forces or
  estimated from the CAD, signed-distance contact for threads, slots and latches), loadable into any Newton scene and
  exportable as USD.
- **Task variations and random starts**: the project website's variation axes (insertion force, rotation amount,
  geometry, number of objects) as task options, and seeded random object placements.
- **Other fingers**: a 3D-printed Fin Ray finger on a printable adapter, and a simulated GelSight Mini tactile
  sensor.
- **Teleoperation and data collection**: keyboard, gamepad or SpaceMouse control with live contact forces, and
  aligned observation / action / contact-force episodes from the scripted policies and from teleoperation.

Policy learning and evaluation pipelines are not part of this release.

Contents: [Installation](#installation) · [Run a task](#run-a-task) · [Tasks](#tasks) ·
[Assets and task variants](#customize-assets-and-task-variants) · [Embodiment](#customize-the-embodiment) ·
[Scene](#customize-the-scene) · [Data and teleoperation](#data-collection-and-teleoperation) ·
[Documentation](#documentation)

![ContactBench tasks in simulation](docs/media/tasks_overview.jpg)

## Installation

Requirements: Linux, an NVIDIA GPU with a CUDA 12 driver, Python 3.10 or newer.

```bash
git clone https://github.com/Michaelwyx/contactbench-sim.git && cd contactbench-sim
conda create -n contactbench python=3.12 -y && conda activate contactbench
pip install -e packages/hydrogel -e "packages/force-bench[hydrogel,teleop]"

force-bench list                  # the benchmark tasks; no GPU needed
force-bench run plug_in_usb_2     # one full task, about 3 minutes on an RTX 5090
```

Optional extras of `force-bench`: `hydrogel` (GelSight sensor), `teleop` (teleoperation window and browser page),
`usd` (USD export and import of the kit's assets), `dev` (tests and linter); for example
`pip install -e "packages/force-bench[hydrogel,teleop,usd,dev]"`. Newton, MuJoCo Warp and Warp are pinned to the
versions the assets are calibrated on. Details and troubleshooting: [docs/installation.md](docs/installation.md).

## Run a task

```bash
force-bench list                          # every task row, by paper task number
force-bench info plug_in_usb_2            # what the row does, how it starts, how it differs from the real task
force-bench run plug_in_usb_2             # run it -> outputs/appendix_tasks/plug_in_usb_2/
force-bench run-all -j 3                  # every scripted row, 3 at a time -> outputs/appendix_tasks/summary.json
```

A run writes to `outputs/appendix_tasks/<row>/`: the videos (`rollout.mp4`, `rollout_multiview.mp4`), per-frame
contact forces (`rollout_forces.csv`, `.png`), body poses and velocities (`rollout_states.npz`), and the verdict
`appendix_qa.json`, with the paper's success criterion checked item by item:

```python
import json
qa = json.load(open("outputs/appendix_tasks/plug_in_usb_2/appendix_qa.json"))
print(qa["pass"], {name: check["pass"] for name, check in qa["criteria"].items()})
```

`TB_RENDER_FAST=1` renders draft videos, `TB_OUT_ROOT=<dir>` redirects all outputs. More in
[docs/getting_started.md](docs/getting_started.md).

## Tasks

Every task's specification (the paper's criterion, the simulated setup, the checks, the options and the documented
differences from the real task) is in **[docs/tasks.md](docs/tasks.md)**.

| # | task | skills | rows |
|---|---|---|---|
| 2 | Retrieve single utensil from bin | clutter, thin | `retrieve_utensil_from_bin` (scene only: no scripted policy) |
| 4 | Retrieve top cup from stack | clutter | `retrieve_top_cup_from_stack` |
| 5 | Retrieve single card from deck | clutter, thin | `retrieve_card_from_deck` |
| 6 | Pick up object from platform | thin | `pick_up_object_on_platform`, `pick_up_object_on_platform_key` |
| 9 | Hang object | placing | `hang_keychain`, `hang_wrench`, `hang_ruler` |
| 13, 14 | Tighten / loosen screw with Allen key | extrinsic, insertion | `tighten_screw_with_allen_key_from_table`, `loosen_screw_with_allen_key_from_table` |
| 15, 16 | Tighten / loosen screw with screwdriver | extrinsic, insertion | `tighten_screw_with_screwdriver(_from_table)`, `tighten_screw_with_screwdriver_longer_screw`, `loosen_screw_with_screwdriver(_from_table)` |
| 17, 18 | Tighten / loosen nut with wrench | extrinsic, insertion | `tighten_nut_with_wrench_from_hook`, `loosen_nut_with_wrench_from_hook` |
| 19 | Put bottle cap onto bottle | insertion, articulated | `put_bottle_cap_onto_bottle`, `_2`, `_3`, `_4rev` |
| 20 | Thread nut onto screw | insertion, articulated | `screw_nut_onto_screw`, `thread_nut_onto_screw_from_table` |
| 22 | Plug in charger | insertion | `plug_in_charger`, `plug_in_charger_block1` ... `4`, `plug_in_charger_block1_spring_0.85mm` |
| 23 | Place card into holder | placing | `place_card_into_holder`, `place_card_into_holder_from_platform` |
| 24 | Stack cups | placing | `stack_cups` |
| 25 | Plug in Ethernet | insertion | `plug_in_ethernet` |
| 26 | Plug in USB | insertion | `plug_in_usb_1` ... `4`, with stiffer springs `plug_in_usb_N_spring04`, `_spring06` |
| 27 | Turn key to unlock door | articulated | `turn_key_unlock_door_flat`, `_lock180`, `_lock270`, `_lock360` |
| 28 | Turn doorknob to open door | articulated | `turn_doorknob_open_door_flat` and 7 variant rows (unlock angle, spring, knob) |
| 29 | Turn door handle to open door | articulated | `turn_door_handle_open_door_flat` and 5 variant rows (handle, unlock angle, spring) |
| 30 | Turn dial to target position | articulated | `turn_dial_to_target` (push-down dial) and 5 rows for the other dials and knobs |
| 31 | Install light bulb | insertion, articulated | `install_light_bulb`, `install_light_bulb_from_table`, `_4rev`, `_2_50mm`, `_3_70mm` |

Numbers follow Table C1 of the paper. Skills: grasping thin or small objects (thin), grasping from clutter
(clutter), placing under environmental constraints (placing), extrinsic contact control (extrinsic), articulated object
manipulation (articulated), insertion and alignment (insertion). Tasks 1, 3, 7, 8, 10, 11, 12 and 21 and the
longer-horizon tasks are not included; [docs/tasks.md](docs/tasks.md#tasks-not-included) says why for each.

## Customize assets and task variants

**Task variants.** Each task page of the project website lists variation axes, such as *Insertion force*,
*Required rotation amount*, *Object geometry* or *Number of objects*, and the kit parts that realise each value.
Here they are options of the task, named as on the website:

```bash
force-bench variations                                    # every task's axes and values
force-bench variations plug_in_charger_block1             # one row; its own choices are marked *
force-bench run plug_in_charger_block1 --set "Insertion force=0.85mm" --set "Object geometry=block 3"
force-bench run turn_doorknob_open_door_flat --set "Required rotation amount=80°"
force-bench run turn_dial_to_target --set "Object geometry=Round knob" --set "Target position=45"
```

![The four charging blocks of task 22](docs/media/readme/variants_charger.jpg)

*Task 22's Object geometry axis: the four charging blocks, each selected with `--set` on `plug_in_charger_block1`.*

A run with `--set` writes to `outputs/appendix_tasks/<row>__<choices>/`. The rows of `force-bench list` are the
combinations the scripted policies are tuned for; any other combination runs with the same policy and checks.
Details: [docs/variations.md](docs/variations.md).

**Assets.** The kit library holds every asset of the asset page with its variants, its physical model and its print
colours:

```bash
force-bench kit list                      # the 49 assets, their kind and their variants
force-bench kit info usb-slot             # parts, sizes, print masses, variants, physical model
force-bench kit check outlet              # settle and probe checks (spring force against displacement, ...)
```

![The printable ContactBench kit](docs/media/readme/kit_assets.jpg)

*The printable kit (render of the final CAD); every part is in the kit library, with its variants.*

A variant's physical parameters (spring stiffness and preload, friction, masses) come from `assets/kit/sim.yaml` and
the asset's mechanism model (`force_bench.kit.mechanisms`). `add_asset(..., **params)` and
`force-bench kit export-usd --param NAME=VALUE` override them; re-running the task or `force-bench kit check` shows
the effect. How each printed mechanism is modelled, and how to add an asset: [docs/assets.md](docs/assets.md),
[docs/physics_notes.md](docs/physics_notes.md).

![Force profiles of the outlet and doorknob-module variants](docs/media/readme/variant_forces.jpg)

*Probe traces from `force-bench kit check`: the outlet's four springs, the doorknob module's springs and unlock angles.*

## Customize the embodiment

The FR3 arm can carry other fingers. `--embodiment` picks them for the tasks that support them:

```bash
force-bench embodiments                                                     # the finger sets, and their tasks
force-bench teleop-ui --scene retrieve_utensil_from_bin --embodiment finray # task 2's scene with Fin Ray fingers
```

![Franka fingers, GelSight Mini pads with their tactile images, Fin Ray fingers, and the Fin Ray adapter](docs/media/readme/embodiment.jpg)

*Stock fingers over task 2's bin, GelSight Mini pads (housing mesh from `hydrogel housing`) lifting task 5's card
with both pads' tactile images at the same moment, Fin Ray fingers over the same bin, and the Fin Ray adapter.*

- **Fin Ray finger** (`finray`): a 3D-printed TPU Fin Ray finger on each carriage of the Franka Hand, through a
  printable adapter (CAD and STL in [assets/finray](assets/finray/README.md)), simulated as a compliant linkage of
  its walls and ribs. Supported by task 2's scene, which has no scripted policy: drive it by teleoperation or with a
  policy of your own. See [docs/finray.md](docs/finray.md).
- **GelSight Mini tactile sensor**: `TB_GELSIGHT=1` mounts simulated GelSight Mini pads on the fingertips of the tasks
  that support them (`[TB_GELSIGHT=1]` in `force-bench list`) and records tactile images, pressure and shear fields
  (`TB_GELSIGHT=1 force-bench run retrieve_card_from_deck`). The sensor's housing CAD is GelSight's own (GPL-3.0) and is
  installed separately with `pip install cascadio && hydrogel housing --download`; without it the case renders as a
  plain box. See [docs/tactile.md](docs/tactile.md#the-sensor-housing-gelsights-cad-installed-locally).
- **Your own fingers**: an embodiment is an entry in `force_bench.embodiments` with a class that builds and attaches
  the fingers and tells the tasks where the fingertips are; each task names the embodiments it supports.
  Step-by-step instructions with a code skeleton: [docs/embodiments.md](docs/embodiments.md).

## Customize the scene

**Random initialization.** The paper's protocol places the task objects uniformly within a 0.15 × 0.15 m square at
the centre of the board. `--seed N` draws such a start, reproducibly, for tasks 2, 4, 15, 16, 19, 24, 25, 26, 30
and 31:

```bash
force-bench run plug_in_usb_2 --seed 3                     # -> outputs/appendix_tasks/plug_in_usb_2__seed3/
force-bench run stack_cups --seed 1
force-bench run turn_dial_to_target --seed 2 --set "Object geometry=Round knob"   # combines with --set and --embodiment
```

![Nominal and seeded starts of plug_in_usb_1 and stack_cups](docs/media/readme/random_starts.jpg)

*First frames of `plug_in_usb_1` and `stack_cups` from the nominal start and with `--seed 1` and `--seed 2`.*

Each task draws the positions (and, where the task allows, the yaw) of its objects within its own ranges, rejects
overlapping objects, and records the drawn start in its verdict; without `--seed` it starts from its nominal pose.
What each task draws: [docs/variations.md](docs/variations.md#random-starts). A task of your own draws its start the
same way, with `Placement` and `sample_start` from `force_bench.appendix_tasks.common`
([docs/developing.md](docs/developing.md#adding-a-task)).

**Your own scene and robot.** The kit's calibrated assets load into a scene of your own, next to your own robot,
from Python or as USD stages. In Python, `Rig` is the scene builder the tasks use: it holds a Newton `ModelBuilder`
(`rig.builder`), `rig.finalize(...)` builds the model and the solver with the settings the kit is calibrated with,
and `rig.run(...)` steps the scene and records it:

```python
from force_bench.rig import Rig
from force_bench.kit import add_asset, place_on_table

rig = Rig("my_scene")                       # rig.builder is a Newton ModelBuilder
rig.table(center_xy=(0.5, 0.0), half=0.5)
rig.builder.add_urdf("my_arm.urdf", floating=False)      # your robot (add_urdf / add_mjcf / add_usd)
h = add_asset(rig, "doorknob-module", "Handle 1",
              pose=place_on_table("doorknob-module", "Handle 1", xy=(0.8, 0.0)))
# then rig.finalize(...) and step with rig.run(...) or your own loop (docs/custom_scenes.md)
```

```bash
pip install -e "packages/force-bench[usd]"
force-bench kit export-usd doorknob-module --variant "Handle 1"   # -> outputs/kit_usd/doorknob-module/Handle_1.usda
python examples/custom_scene/ur10_door_handle.py                  # Newton's UR10 pushes the lever with a rod
```

![Newton's UR10 pushing the lever of the doorknob module loaded from USD with a rod on its flange](docs/media/readme/custom_scene_ur10.jpg)

*The scene of `examples/custom_scene/ur10_door_handle.py`, rendered offscreen: a rod on the UR10's flange turns
the USD-loaded lever.*

A stage carries the asset's joints, springs, stops and contact settings; `force_bench.kit.usd.add_asset_usd` adds
it to any `newton.ModelBuilder` and applies the solver settings the calibration assumes. Both routes, the settings
to keep, and what to expect in other simulators: [docs/custom_scenes.md](docs/custom_scenes.md).

## Data collection and teleoperation

**Demonstrations from the scripted policies.** Each episode runs in a fresh process and stores aligned
observations, actions and contact forces, with the task's verdict as its outcome. For a task that takes `--seed`,
every episode starts from its own random start (`--nominal-start` keeps the nominal one):

```bash
force-bench collect --task plug_in_usb_2 --episodes 20 --seed 0 --output outputs/datasets/usb
force-bench dataset outputs/datasets/usb --success-only      # check every episode and summarise
```

```python
from force_bench.data.episode import discover_episodes, load_episode

for path in discover_episodes("outputs/datasets/usb", success_only=True):
    ep = load_episode(path)
    obs, act = ep["observations"], ep["actions"]
```

**Teleoperation.** Drive the arm through any task by keyboard, gamepad or 3Dconnexion SpaceMouse, with live contact
forces and alarms, in a local window or a browser, and record demonstrations in the same episode format. The task
starts as `force-bench run` starts it (`--seed`, `--set` and `--embodiment` work as there), and you drive it instead
of the scripted policy:

```bash
force-bench teleop-ui --list-scenes                             # the tasks of force-bench list, and a sandbox
force-bench teleop-ui --scene plug_in_usb_2                     # GL window
force-bench teleop-ui --scene plug_in_usb_2 --ui web            # then open http://127.0.0.1:8765/
force-bench teleop-ui --scene plug_in_usb_2 --seed 3 --input spacemouse
force-bench teleop-ui --scene plug_in_usb_2 --collect-dir outputs/datasets/teleop   # record demonstrations
```

Formats and options: [docs/data.md](docs/data.md), [docs/teleoperation.md](docs/teleoperation.md).

## Documentation

| | |
|---|---|
| [installation.md](docs/installation.md) | requirements, install, first run, troubleshooting |
| [getting_started.md](docs/getting_started.md) | running tasks, outputs, verdicts, environment variables, changing parameters |
| [tasks.md](docs/tasks.md) | every task: paper criterion, simulated setup, checks, options, documented differences |
| [variations.md](docs/variations.md) | the website's variation axes as task options (`--set`), random starts (`--seed`) |
| [assets.md](docs/assets.md) | the kit library (`force-bench kit`), how printed mechanisms are modelled, checks, colours |
| [custom_scenes.md](docs/custom_scenes.md) | the kit's assets in your own scene and with your own robot, from Python or as USD stages |
| [embodiments.md](docs/embodiments.md), [finray.md](docs/finray.md) | other fingers on the FR3 hand (`--embodiment`), how to add your own; the Fin Ray finger and its model |
| [tactile.md](docs/tactile.md), [gelsight_method.md](docs/gelsight_method.md) | the GelSight Mini model: usage, validation, method and parameters |
| [data.md](docs/data.md) | recording demonstrations and the episode format |
| [teleoperation.md](docs/teleoperation.md), [teleop_guard.md](docs/teleop_guard.md) | driving tasks by hand; the safety guard and acceptance gate |
| [physics_notes.md](docs/physics_notes.md) | modelling rules and pitfalls for building new assets |
| [developing.md](docs/developing.md) | code layout, anatomy of a task, adding a task, tests |
| [examples/force_response](examples/force_response/README.md) | force–displacement probes of the spring models |
| [examples/kit_scene](examples/kit_scene/README.md) | a minimal scene built from the kit library |
| [examples/custom_scene](examples/custom_scene/README.md) | a kit mechanism loaded from its USD stage, pushed by Newton's UR10 |

## Repository layout

```
assets/                    the printable kit as published (assets/kit), the Fin Ray finger and adapter (assets/finray),
                           meshes some tasks load besides the kit (assets/bodies, assets/bodies_kit), physical defaults
packages/force-bench/      tasks, the kit library, embodiments, rendering, teleoperation, data (the force-bench command)
packages/hydrogel/         the GelSight Mini sensor model (the hydrogel command)
examples/                  standalone scripts
docs/                      documentation
```

## Citation

```bibtex
@article{xiang2026contactbench,
  title   = {ContactBench 1.0: A Benchmark for Everyday Contact-Rich Robotic Manipulation},
  author  = {Xiang, Jingyi and Wang, Yixuan and Bohg, Jeannette and Yuan, Wenzhen},
  journal = {arXiv preprint arXiv:2610.24512},
  year    = {2026}
}
```

## License

MIT, see [LICENSE](LICENSE). A few third-party files keep their own licenses, and the robot description is
downloaded at run time; see [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
