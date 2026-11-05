# Tactile sensing: a simulated GelSight Mini

The simulation can mount a GelSight Mini on each FR3 fingertip. The sensor is computed from the solver's own contact
mechanics, not from a depth camera, and produces tactile RGB images with markers, pressure and indentation maps,
shear displacement, and the net normal force on each finger.

This page covers what the sensor is, how to check it, how to turn it on in a task, and what it measures. The
derivation and every parameter are in [gelsight_method.md](gelsight_method.md).

**Requirements.** A CUDA GPU (Newton's hydroelastic contact has no CPU path) and both packages installed:

```bash
pip install -e packages/hydrogel -e "packages/force-bench[hydrogel]"
```

See [installation.md](installation.md) for the full setup. The timings on this page are for an NVIDIA RTX 5090.

---

## 1. What it is

Three layers, from physics to picture:

```
  Newton hydroelastic contact          pressure and penetration on each contact face
        ↓  the gel's stiffness passed to MuJoCo per contact, in force space
  hydrogel.sensor                      splat onto a 96×128 gel raster:
        ↓  about 2 ms per frame        pressure, normal displacement, shear displacement
  hydrogel.render                      Taxim RGB and field-following markers, 320×240:
                                       the picture only
```

Everything else rests on one property: **the solver honours the gel law**. The gel is a mesh shape, so Newton exports
the true indentation on its side of the contact, and each hydroelastic contact passes MuJoCo a force-space `solref`
carrying its secant stiffness `k·A`. The contact's compliance therefore follows the gel stiffness `gel_kh`, not the
inertia of the bodies involved; with MuJoCo's default, acceleration-space `solref`, the gel would act as a rigid
wall. The ratio of the solver's force to the pressure-law integral (`solver_over_law`) is 1.1 to 1.3 where the gel
does not bottom out. It is written to the outputs as a diagnostic channel (`gel_*_law_N`, `solver_over_law`), so a
departure from the law stays visible.

The mechanics channels are the observation. **The RGB image is a view of them; it is never an input to the physics or
to the verdict.**

Other parts of the model:

- **Mount.** The real GS Mini: an aluminium case (32×28×24 mm, 40 g, from the datasheet), a 25.25×20.75 mm gel pad,
  an 18.6×14.3 mm imaged field of view, and a sensing face 5 mm proud of the bare FR3 fingertip. The FR3's rubber-tip
  box stays in the model as the gel's **rigid backing**: it stops an object that bottoms the gel out. `GS_MOUNT=case`
  is the default mount.
- **Markers.** Field-following: markers are attached to the gel surface and carried by the shear-displacement field,
  so a twist produces a vortex and an edge produces a crease. They are rendered by inverse-warping the marker texture.
- **Camera.** A pinhole at 24 mm standoff, so a pure press dilates the marker pattern as the real sensor's does. An
  orthographic renderer does not.
- **Shear.** A Coulomb-bounded stored surface displacement (μ = 2.0, TacSL's calibrated value for a tacky gel),
  stiffness 3.0e7 Pa/m, capped at 2 mm, relaxing over 25 ms after contact is lost.

---

## 2. Checking the sensor on its own: `hydrogel validate`

Start here. The sensor bench has **no solver, no robot and no task**: a kinematic indenter follows a prescribed
trajectory against one static pad, and every frame runs collision, sampling and rendering. The video shows the
commanded and measured values in a side panel, and where a closed-form force exists it is printed next to the
sensor's.

```bash
# all 16 experiments: {press, steps, shear, rotate} × {sphere, box, rod, key_bow}
hydrogel validate

# one or a few
hydrogel validate --exp press_sphere,shear_box

# parameter sweeps: the same shear_sphere trajectory with 3 values of one parameter, side by side
hydrogel validate --sweep marker_gain
hydrogel validate --sweep all
```

This writes `outputs/gelsight/validate/<exp>.mp4` and `<exp>.json` (per-frame metrics), and `sweep_<param>.mp4` for
sweeps. The videos are upscaled 2× by default (`GS_VIDEO_SCALE`), because the native 320×240 tiles are too small to
inspect.

The bench is **deterministic**: re-running an experiment reproduces its JSON byte for byte. It has no solver, and so
none of the GPU's nondeterministic contact ordering, which makes it the place to test the sensor itself. The simulated
tasks are not bit-reproducible ([§3](#run-to-run-variation)).

**The four motions.** `press`: depth ramp 0 → 1.5 mm, hold, retract. `steps`: a staircase at 0.3 / 0.6 / 0.9 /
1.2 mm. `shear`: press 0.8 mm, slide +x 2.5 mm, back, then +y. `rotate`: press 0.8 mm, twist 45° about the normal,
back.

**Sweepable parameters** (`--sweep <p>`, 3 values each): `gel_kh`, `stiffen`, `shear_k`, `skirt`, `blur`,
`contrast`, `bulge`, `marker_gain`, `lambda_d`. The sweep videos are the fastest way to see what a parameter does
before you change it; the parameter table in [gelsight_method.md](gelsight_method.md#parameter-table) names the sweep
for each.

### What the bench measures

Each frame of `<exp>.json` holds the commanded depth, the sensor's force `F_sensor_N` and, for the sphere and the
flat box, the closed form `F_analytic_N`: `F = πkRd²` for the sphere and `F = kAd` for the box, both with the
stiffening term integrated numerically. The mesh indenters (rod, key bow) have no closed form. The bench shows three
things:

1. The sensor force follows the closed form to within 3.5 % on every indenter that has one, so the pressure law and
   the splat integral agree.
2. A pure press moves the markers with zero shear. That is the pinhole camera's dilation, which an orthographic
   renderer cannot produce.
3. Shear and rotation store about 1.2 mm of surface displacement (against the 2 mm cap) and produce very different
   marker fields at the same force: a coherent slide against a vortex. This is why the marker mechanics are
   simulated rather than drawn as a texture.

### Cheaper checks

```bash
pytest packages/hydrogel/tests -v             # sensor behaviour tests (skipped without CUDA)
hydrogel render-selftest                      # flat-field and marker-advection check of the renderer
hydrogel bench                                # kernel latency and throughput
hydrogel grip-rig                             # two-jaw regression rig, 5 asserted checks
python -m force_bench.tactile.grip_task       # the reference FR3 grasp-lift-shake probe with two pads
python -m force_bench.tactile.press_probe     # objects pressed into one pad: the sensor's force against the solver's
```

On an RTX 5090 the GPU kernels take about 0.2 ms to splat 20k contact faces onto the 96×128 raster and 0.04 ms to
shade a 320×240 image. The per-frame cost of a tactile run is therefore host work (array copies, per-frame Python,
image encoding), not GPU compute.

---

## 3. GelSight on the tasks: `TB_GELSIGHT=1`

`TB_GELSIGHT=1` turns a task into its tactile variant: both FR3 fingertips carry a GelSight Mini, the grasped object
is felt by the gel, and the run records the tactile fields next to the rollout. With the variable unset, the task runs
exactly as it does without the sensor.

```bash
TB_GELSIGHT=1 force-bench run retrieve_card_from_deck
```

### The sensor housing: GelSight's CAD, installed locally

Each finger carries the sensor in a case: a parametric box that collides and carries the sensor's mass. For the
renders, the official GS Mini housing CAD can be drawn over it; the figures in this repository show it. That CAD is
GelSight Inc.'s (`docs/CAD/gsmini_shell.zip` in [gelsightinc/gsrobotics](https://github.com/gelsightinc/gsrobotics),
GPL-3.0), so it is not shipped here. To use it:

```bash
pip install cascadio                              # STEP reader for trimesh
hydrogel housing --download                       # fetch gsmini_shell.zip, convert, install
hydrogel housing ~/Downloads/gsmini_shell.zip     # or from a file you downloaded (.zip or .stp)
```

The command converts the STEP file to a mesh in metres, checks its size (32.0 × 28.25 × 28.89 mm) and saves it as
`packages/hydrogel/src/hydrogel/data/gs_mini/gsmini_shell.obj`, which `.gitignore` keeps out of commits. Every
tactile run after that renders the housing; nothing else changes, since collision and mass stay on the box.
`GS_SHELL_VISUAL=0` goes back to the box. Details: `packages/hydrogel/src/hydrogel/data/gs_mini/PROVENANCE.md`.

### Which tasks support it

`force-bench list` marks the tasks validated with the sensor, `[TB_GELSIGHT=1]`: their verdict, including the three
gel checks of [§4](#4-what-the-tactile-variants-measure), passes with the pads mounted. In this release that is task
5, *Retrieve single card from deck* (`retrieve_card_from_deck`). The `tactile` line of `force-bench info <task>` says
which applies to a task. The other tasks run with the bare FR3 fingers: `TB_GELSIGHT=1` is not validated on them, and
`force-bench collect --gelsight` refuses them.

### What a tactile run writes

A tactile run writes to the task's own folder, `outputs/appendix_tasks/<task>/`, the one the bare run writes to: the
usual `rollout.mp4`, `rollout_multiview.mp4`, `rollout_forces.csv/png`, `rollout_states.npz`, `rollout_qa.json` and
`appendix_qa.json`, plus

- `tactile_left.npz`, `tactile_right.npz`: each pad's per-frame mechanics, frame for frame with the rollout:
  `pressure_pa`, `normal_displacement_m`, `shear_displacement_m`, `shear_traction_pa`, `contact_mask`,
  `total_normal_force_n`, `center_of_pressure_m`;
- `tactile_scene.npz`: what is needed to re-render the tactile images without re-simulating;
- `tactile_strip.mp4`: left RGB with markers | right RGB with markers | left pressure | right pressure;
- `rollout_panel_tactile.mp4`: the multiview rollout with the tactile strip underneath.

To keep both a bare run and a tactile run of the same task, give one of them `--name <folder>` or
`TB_OUT_ROOT=<dir>`.

### Run-to-run variation

Tactile runs are not bit-reproducible, because the GPU's contact ordering is not deterministic. The crosscheck ratio
is stable to a few percent from run to run, while a ratio that depends on where the gel bottoms out
(`solver_over_law` on a hard, small feature) can move by tens of percent, and peak transients, such as the largest
finger force during a grasp, can differ by more than a factor of two. Expect verdicts to be stable and the third digit
of a number not to be.

### Useful variations

```bash
# mechanics only: skip the tactile strip and panel videos
TB_GELSIGHT=1 TB_GELSIGHT_RENDER=0 force-bench run retrieve_card_from_deck

# redirect every output under another root (keeps the bare run's folder)
TB_GELSIGHT=1 TB_OUT_ROOT=/tmp/gsrun force-bench run retrieve_card_from_deck

# re-render the tactile images of a finished run without re-simulating
python -m hydrogel.render --task-dir outputs/appendix_tasks/retrieve_card_from_deck --panel

# the solver-matched control: the tactile run's solver settings without pads, to separate
# "the pads changed the result" from "the solver settings changed the result"
TB_MJ_IMPRATIO=10 TB_COLLIDE_SUBSTEP=1 force-bench run retrieve_card_from_deck --name retrieve_card_from_deck_control
```

A tactile run takes about twice as long as the bare one: it runs 32 substeps with collision every substep, and the
pads add their own contact.

### Recording tactile demonstrations

```bash
force-bench collect --task retrieve_card_from_deck --gelsight --episodes 2 --seed 0 \
    --output outputs/datasets/card_gelsight
```

Each episode holds the robot observations and joint targets with timestamps, the two pads' pressure tensors with
validity masks, the configuration, and the full run under `artifacts/appendix_tasks/<task>/` (tactile fields,
videos, verdict). An episode that fails its verdict is kept and marked as failed rather than published as a success.
See [data.md](data.md) for the episode format.

### How a task opts in

This part is for adding the sensor to another task. `force_bench.tactile.task` is the opt-in layer
(`import force_bench.tactile.task as GS`); with `TB_GELSIGHT` unset every call below does nothing, so the bare task
is unchanged. A scene opts in with five calls:

1. `FINGER_* += GS.finger_offset()` on every finger-joint target, so the commanded gap between the sensing faces is
   the same as between the bare fingertips (the sensing face sits 5 mm proud).
2. `rig = GS.make_rig(name)` in place of the scene's plain rig: the same rig with tactile sampling in its run loop,
   32 substeps, collision every substep and MuJoCo `impratio=10`.
3. `GS.mount(rig)` before `finalize()` puts the GS Mini case and gel pad on each FR3 finger.
4. `GS.hydroelastic(rig, "<label>")` marks the grasped object. Newton only builds a hydroelastic contact surface when
   both shapes opt in; without this call the gel reads zero.
5. `GS.save_tactile(rig, objects, qa=..., window=(t0, t1), probe_forces=...)` writes the tactile files and adds the
   three checks of §4 to the verdict (`window` is the grasp, `probe_forces` the solver's force on each finger).

A scene that rebuilds the collision pipeline after `finalize()` must use `GS.make_pipeline` to keep the hydroelastic
surface output, and one that replaces the solver must keep `rig.solver_options`. Mount the pads after the target
geometry is added and before the finger collision exclusions, so the gel and its housing get the same collision
filtering as the original fingertips. A task derived from a scene module through `appendix_tasks/derive.py`
([developing.md](developing.md)) needs nothing more: its tactile outputs land in the task's own folder. Once the
task's verdict passes with the pads, set `gelsight=True` on its row in `force_bench/appendix_tasks/registry.py`, so
`force-bench list` marks it and `force-bench collect --gelsight` accepts it.

---

## 4. What the tactile variants measure

Every tactile run adds three gel checks to its verdict:

- **`gel_pads_loaded`**: both pads carry the grasp through the task window: mean force above 0.5 N, and contact on
  at least 85 % of the frames.
- **`gel_coulomb_bounded`**: `|shear traction| ≤ μ·pressure` on every cell of every frame.
- **`gel_probe_crosscheck`**: the solver's contact force on each finger against the gel's own pressure integral; the
  mean ratio over the window must lie in [0.5, 2.0]. This catches a sensor that no longer describes the contact the
  solver is computing.

In `appendix_qa.json` the three checks are part of `physics_integrity`, so a tactile run passes only if they do. On
task 5 the crosscheck ratio is close to 1 (the solver and the sensor describe the same contact), and both pads stay
loaded while the card is carried.

---

## 5. Known limits

- **Thin parts need a fine SDF.** Task 5's 1.2 mm card uses a 512-cell tactile SDF: the default of 96 cells leaves
  fewer than two samples through the card's thickness and causes brief grip losses. A thin part in another task needs
  the same (`shape_sdf_max_resolution` of its shapes).
- **One task is validated with pads.** The other tasks run with the bare FR3 fingers; a task is marked
  `[TB_GELSIGHT=1]` only once its verdict passes with them.
- **The gel bottoms out on hard, small features** (a key bow, nut flats). Peak indentation reaches the 4 mm gel
  thickness and the rigid backing takes over. The forces stay correct; the gel law is no longer the right model there.
- **Many parallel environments are limited by the physics, not by the sensor.** At the kernel costs of §2 a hundred
  pads sample and shade in under a millisecond, while one environment's substep chain takes about 170 ms per frame.
  The largest tactile cost is the marker-warp layer, which runs on the CPU (about 6 ms per tile).
- **CUDA only.** The hydroelastic contact has no CPU path.

---

## 6. Where everything lives

| | |
|---|---|
| `packages/hydrogel/src/hydrogel/sensor.py` | the sensor core: hydroelastic sample → gel raster |
| `packages/hydrogel/src/hydrogel/pressure.py` | the stiffening pressure law |
| `packages/hydrogel/src/hydrogel/mujoco_force_space.py` | per-contact gel stiffness realised in force space under MuJoCo |
| `packages/hydrogel/src/hydrogel/render.py`, `render_kernels.py` | Taxim RGB and markers, GPU shading |
| `packages/hydrogel/src/hydrogel/markers.py` | field-following marker mechanics |
| `packages/hydrogel/src/hydrogel/validate.py` | the sensor bench of §2 (`hydrogel validate`) |
| `packages/hydrogel/src/hydrogel/calibrate.py` | `hydrogel calibrate --sweep / --fit` against real ball presses |
| `packages/hydrogel/src/hydrogel/bench_parallel.py` | kernel latency and throughput bench (`hydrogel bench`) |
| `packages/hydrogel/src/hydrogel/grip_rig.py` | two-jaw regression rig (`hydrogel grip-rig`) |
| `packages/hydrogel/src/hydrogel/housing.py` | `hydrogel housing`: installs the GS Mini housing CAD locally |
| `packages/hydrogel/src/hydrogel/data/gs_mini/` | vendored Taxim calibration (see `PROVENANCE.md` and `LICENSE` there); the official GS Mini housing CAD is GPL-3.0 and not included, see `PROVENANCE.md` for adding it locally |
| `packages/force-bench/src/force_bench/tactile/rig.py` | the pads on the FR3 fingers, the case mount, sampling with the solver's forces |
| `packages/force-bench/src/force_bench/tactile/task.py` | the `TB_GELSIGHT=1` opt-in layer (does nothing when off) |
| `packages/force-bench/src/force_bench/tactile/grip_task.py` | the reference FR3 grasp-lift-shake probe |
| `packages/force-bench/src/force_bench/tactile/press_probe.py` | contact-model audit (solver force against the gel law) |
| `outputs/gelsight/` | the sensor's own evaluation (`validate/`, grip probe, rig, demos) |
| `outputs/appendix_tasks/<task>/` | a task run with pads: the tactile fields next to the rollout and the verdict |

`TB_OUT_ROOT=<dir>` moves every output under `<dir>`, so a second run does not overwrite the first.

**Further reading:** [gelsight_method.md](gelsight_method.md) for the derivation and the full parameter table (every
parameter, its physical meaning, its effect, and the tool that checks it); [physics_notes.md](physics_notes.md) for
the solver rules the tactile model depends on.
