# hydrogel — hydroelastic GelSight Mini simulation for Newton

`hydrogel` turns NVIDIA Newton's dense **hydroelastic contact surface** into
the three signals a GelSight-class sensor measures — pressure, normal
displacement and history-dependent shear — on a 96×128 gel raster, and
renders them as GelSight Mini RGB images with Taxim shading and
field-following markers.  Both the gel and the touching object stay Newton
hydroelastic SDF shapes: this is a sensor model on top of the physics, not a
replacement contact solver.

```
Newton hydroelastic contact ──► HydroelasticGelSight.sample()  ──► TaximRenderer
(pressure per contact face)     96×128 pressure / indent / shear     320×240 RGB + markers
                                ~2 ms                                ~6 ms
```

`hydrogel` is the tactile half of the ContactBench 1.0 simulation; the tasks that use it live in the sibling
`force-bench` package (`force_bench.tactile`).

## Install

From the root of this repository:

```bash
pip install -e packages/hydrogel
```

`hydrogel` pins `newton==1.5.0`, `warp-lang==1.16.0`, `mujoco-warp==3.11.0` and `mujoco==3.11.0`: it patches a
MuJoCo-Warp kernel of that exact Newton release (`hydrogel.mujoco_force_space`), so other versions are not
supported.  A CUDA GPU is required (Newton's hydroelastic path has no CPU fallback).  The vendored GelSight Mini
optical calibration ships as package data (`hydrogel/data/gs_mini`; provenance and license in that directory's
`PROVENANCE.md` and `LICENSE`). GelSight's official housing CAD is GPL-3.0 and is not included; the sensor housing
renders as a box unless you add it locally (see `PROVENANCE.md`).  Full instructions: [docs/installation.md](../../docs/installation.md).

## Use

```python
import newton, warp as wp
from hydrogel.sensor import GelSightConfig, attach_gelsight_pad, HydroelasticGelSight
from hydrogel.sensor import winning_hydroelastic_config

cfg = GelSightConfig(rows=96, cols=128, gel_kh_pa_per_m=4.0e7)
builder = newton.ModelBuilder()
# ... add the bodies the gel must feel FIRST (the gel has to be shape B of every pair) ...
gel = attach_gelsight_pad(builder, finger_body, cfg, xform=..., label="gel")
model = builder.finalize()
pipeline = newton.CollisionPipeline(model, sdf_hydroelastic_config=winning_hydroelastic_config(model, cfg, [gel]))
sensor = HydroelasticGelSight(model, pipeline, gel, cfg)
# each frame, after pipeline.collide(state, contacts):
frame = sensor.sample(state, dt)       # frame.pressure_pa, .normal_displacement_m, .shear_displacement_m, .contact_mask
```

The command line exposes the sensor's own evaluation:

```bash
hydrogel validate            # the solver-free sensor bench (press/steps/shear/rotate × 4 indenters)
hydrogel demo                # press an indenter into a pad, save a frame + video
hydrogel grip-rig            # two-jaw regression rig, 5 asserted checks
hydrogel calibrate --fit ... # gel stiffness against real ball presses
hydrogel bench               # kernel latency / throughput
```

Outputs go to `outputs/gelsight/` at the repository root (or `$TB_OUT_ROOT/gelsight`).  Each subcommand forwards
its arguments to the module's own parser, so `hydrogel validate --help` lists its options.

## Modules

| module | what |
|---|---|
| `hydrogel.sensor` | `GelSightConfig`, `attach_gelsight_pad`, `HydroelasticGelSight` (the raster), hydroelastic pipeline config |
| `hydrogel.sensor_kernels` | GPU rasterizer (`GS_SENSOR_GPU`) |
| `hydrogel.pressure` | the stiffening gel pressure law |
| `hydrogel.render`, `render_kernels` | Taxim RGB, pinhole camera, bulge ring, GPU shading |
| `hydrogel.markers` | HydroShear field-following markers |
| `hydrogel.mujoco_force_space` | per-contact hydroelastic stiffness realised in force space under SolverMuJoCo |
| `hydrogel.validate`, `grip_rig`, `calibrate`, `bench_parallel` | the benches |

How to run the sensor and what it measures: [docs/tactile.md](../../docs/tactile.md).  The derivation and the
full parameter table: [docs/gelsight_method.md](../../docs/gelsight_method.md).  The integration with the
simulated tasks (`TB_GELSIGHT=1`) lives in the sibling `force-bench` package (`force_bench.tactile`).
