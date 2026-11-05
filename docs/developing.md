# Developing

This page is for people who want to change a task, add one, or build a new asset. It describes how the code is
organised and the conventions every task follows.

## Repository layout

```
assets/                          the printable kit (assets/kit), other meshes, physical defaults, Fin Ray CAD
docs/                            documentation
examples/force_response/         force–displacement probes of the spring models
examples/kit_scene/              a minimal scene built from the kit library
examples/custom_scene/           a kit mechanism from its USD stage, pushed by Newton's UR10
packages/force-bench/            the simulation package (import name force_bench)
packages/hydrogel/               the GelSight Mini sensor model (docs/tactile.md)
outputs/                         everything a run writes (created on demand, not tracked)
```

Inside `packages/force-bench/src/force_bench/`:

| module | role |
|---|---|
| `rig.py` | `Rig`, the scene builder: a Newton model (table, robot, assets), the MuJoCo Warp solver with the settings the kit is calibrated with ([custom_scenes.md](custom_scenes.md#the-settings-the-calibration-assumes)), the control loop every task uses (`Rig.run(duration, control, extra)`), recording, and `Rig.finish(qa)` |
| `robot/franka.py` | the FR3 import, joint drives, inverse kinematics, grasp orientations |
| `assets.py` | loading the meshes of `assets/bodies/` (cm → m), signed-distance colliders, masses |
| `geometry.py` | small geometry helpers shared by several tasks |
| `paths.py` | where the assets and the outputs are (`TB_ASSETS_ROOT`, `TB_OUT_ROOT`) |
| `kit/` | the kit library ([assets.md](assets.md#the-kit-library)): `catalog.py` (assets, parts, variants, print masses), `build.py` (`add_asset`, `place_on_table`), `mechanisms/` (one module per mechanism model), `check.py` (`force-bench kit check`), `importer.py` (`force-bench kit import`), `export.py` and `usd.py` (USD stages and `add_asset_usd`, [custom_scenes.md](custom_scenes.md)), `usd_check.py` (the comparison of the two routes), and `cli.py` (the `force-bench kit` command) |
| `tasks/` | scene modules that several tasks build on (card holder, slide-off platform, keychain hook, nut and bolt, wrench, screwdriver drive, charger and outlet): each builds a printed mechanism with its contact model and runs a scripted rollout |
| `appendix_tasks/` | the ContactBench tasks ([tasks.md](tasks.md)); `registry.py` lists them, `variations.py` declares their variation axes ([variations.md](variations.md)), `common.py` and `derive.py` hold the shared helpers |
| `recorder.py` | offscreen rendering (Newton's ray-traced camera sensor), video, force and state logs |
| `probes.py` | `ForceProbe`: total and frictional contact force on a body, per frame |
| `qa.py` | the generic physics checks (finite values, velocity spikes, penetration, workspace) |
| `kit_look.py` | the kit colours ([assets.md](assets.md#colours)) |
| `embodiments/` | the finger sets the FR3 hand can carry, and the hooks a task calls for them ([embodiments.md](embodiments.md)) |
| `finray/` | the Fin Ray finger: geometry, models, the `finray` embodiment ([finray.md](finray.md)) |
| `tactile/` | the GelSight bridge, active under `TB_GELSIGHT=1` ([tactile.md](tactile.md)) |
| `teleop/` | the teleoperation guard, acceptance gate and UI ([teleoperation.md](teleoperation.md), [teleop_guard.md](teleop_guard.md)) |
| `data/` | demonstration episodes: recording, format, collection, inspection ([data.md](data.md)) |
| `cli.py` | the `force-bench` command |

## Anatomy of a task

A task module builds its scene with `Rig`, runs a scripted policy through `Rig.run`, and writes a verdict. `Rig`
handles the physics loop, the recording and the generic physics checks; the task supplies three things.

1. **The scene.** Add the robot first (`rig.add_robot(base_xform)`), then the objects, with
   `force_bench.kit.add_asset` or as bodies and shapes on `rig.builder` (a `newton.ModelBuilder`), then call
   `rig.finalize(...)` with the camera pose and the bodies to probe for contact forces.
2. **A control callback**, `control(t)`, called once per frame (60 Hz). It reads measured state
   (`rig.s0.body_q`, joint coordinates, the force probes) and commands the arm with
   `rig.drive_arm(tcp_pos, tcp_quat, fingers)` (inverse kinematics to FR3 joint targets) or writes joint
   targets directly. The physics runs 32 substeps per frame.
3. **A verdict.** After the run, the task measures what the success criterion asks for and calls
   `save_appendix_verdict(rig, task=..., source=..., spec=..., criteria=..., changes=..., physics_qa=...)`
   (`appendix_tasks/common.py`), which writes `appendix_qa.json`. Each criterion is a dict with `pass` and the
   numbers it was decided on.

`appendix_tasks/common.py` has the helpers every task uses: `appendix_rig(name)` (a `Rig` whose outputs go to
`outputs/appendix_tasks/<name>/`, or to the folder `force-bench run` names for a run with `--set`, `--seed` or
`--name`), `RobotContactProbe` (contact force between any robot link and the task objects), `retreat_pose`,
`check_released_and_clear`, `run_until` (end a rollout as soon as a condition holds), and `Placement` /
`sample_start` (seeded random starts, below).

A new task is a module that builds its own scene and takes its objects from the kit library with
`force_bench.kit.add_asset`, so that they come with the published geometry, print colours and masses;
`retrieve_top_cup_from_stack.py` and `stack_cups.py` are compact examples. Some tasks instead build on a scene module
of `force_bench.tasks`: `appendix_tasks/derive.py` runs the scene's own rollout with its outputs redirected, its
control callback cut, overridden or extended, and the benchmark checks added, without editing the scene module
(`pick_up_object_on_platform.py`, `retrieve_card_from_deck.py`).

Conventions:

- **SI units** everywhere in code (m, kg, N, s, rad); the meshes in `assets/kit/` are in mm and those in
  `assets/bodies/` in cm, both converted on load.
- **Every physical constant is named** at the top of the module with its unit and where it comes from
  (measured, CAD, fitted, chosen for stability).
- **Verdicts measure, they do not trust the script.** A criterion is decided on simulator state after the
  fact (positions, contact forces, probe torques), never on "the script reached phase X".
- **Approximations are written down.** Anything that departs from the real task or object goes into the
  `changes` list of the verdict and the module docstring.
- **Scene modules are not edited by tasks.** A task that needs a different start or ending of a scene in
  `force_bench.tasks` derives from it.

## Adding a task

1. Write `force_bench/appendix_tasks/<name>.py` with a `main(argv=None)` that builds the scene, runs the policy and
   writes the verdict as above. Start from the task in [tasks.md](tasks.md) closest to yours.
2. Add a row (`AppendixSpec`) to `appendix_tasks/registry.py`: the row's name, the module, the paper task title and
   number, the scene module it builds on (`source`, or `"new"`), and as needed `argv` (the module's arguments),
   `variant`, `skills`, `start` (the start state), `notes` (the documented differences), `gelsight=True` for a row that
   supports GelSight pads, and `scripted=False` for a scene without a scripted policy.
3. If the module realises variation axes of the task's website page, declare them in
   `appendix_tasks/variations.py`: an entry `AXES["<module>"]` with one `Axis` per axis, under the website's name,
   each `Value` labelled as on the asset page and mapped to the module options it sets (`Value("0.6mm indent spring",
   ("--spring", "0.6"))`), or a numeric axis with the option that takes the number (`flag=`, `unit=`).
   `force-bench run <row> --set "axis=value"`, `force-bench info` and `force-bench variations` then know them, and
   `force-bench info` marks the values a registry row's `argv` selects ([variations.md](variations.md)).
4. To support a random start (`--seed N`), describe each object's start with `common.Placement` (nominal `xy` and
   `yaw`, the half-ranges `dxy` and `dyaw` to sample in, and the footprint `radius` that keeps objects apart), give
   the module a `--seed` option and draw the start with `common.sample_start(placements, seed)`, which returns the
   nominal poses when the seed is `None`. An object is drawn around its nominal position unless its `Placement` gives
   a `centre`: objects scattered over one square (the cups of `stack_cups`) all give the square's centre, so that
   each can land anywhere in it. Keep the ranges inside what the scripted policy is checked on, write the draw into
   the verdict as an information-only criterion, document the ranges in the module docstring and in
   [tasks.md](tasks.md), and add the module to `variations.SEEDED` so that `force-bench run` passes `--seed` on.
5. Objects from the kit library are drawn in their print colours. For other shapes, add colour rules to
   `kit_look.py` under the task's output name; shapes of a task without rules keep the studio palette. Inspect the
   shape and body labels with `print(list(rig.model.shape_label), list(rig.model.body_label))` after `finalize`.
6. To let the task run with other fingers (`--embodiment`), follow
   [embodiments.md](embodiments.md#how-an-embodiment-is-integrated).
7. Run it (`force-bench run <name>`), watch `rollout.mp4`, and read `appendix_qa.json`. A task is ready when it
   passes repeatedly (`TB_OUT_ROOT=<dir>` or `--name <folder>` keeps repeated runs apart) and its
   `physics_integrity` checks pass.
8. Document it in [tasks.md](tasks.md).

## Building a new asset

[physics_notes.md](physics_notes.md) lists the modelling rules and pitfalls for simulated assets: unit conversion,
where a replaced spring must act, which bodies belong to one articulation, how fine signed-distance fields must be
for thin parts, when contact must be computed every substep, contact buffer limits, and more. Read it before
modelling a new mechanism.

A mechanism of the kit is a module in `force_bench/kit/mechanisms/`, named by `model:` in `assets/kit/sim.yaml`. Its
`build(rig, asset, handle, **params)` adds the asset's bodies, shapes and joints at `handle.pose` for the variant
`handle.choice` and fills `handle.bodies`, `joints`, `shapes`, `params` and `info`; the optional
`add_probe(rig, handle)` and `run_probe(rig, handle)` give it a test in `force-bench kit check`. `outlet.py` is a
compact example. [examples/force_response](../examples/force_response/README.md) shows how to measure the force
response of a mechanism in isolation, which is the quickest way to tune a spring against a real print.

## Tests and checks

```bash
pip install -e "packages/force-bench[dev]"
pytest -q packages                        # unit tests and physics contracts (some need a GPU)
ruff check packages examples              # syntax errors and undefined names
force-bench kit check -j 2                # every kit asset and variant settles; mechanisms pass their probes
force-bench run-all -j 3                  # the full benchmark; every row with a scripted policy should report PASS
```
