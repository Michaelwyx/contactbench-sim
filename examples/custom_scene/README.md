# A kit mechanism from USD, next to a robot of your own

`ur10_door_handle.py` puts the kit's doorknob module (lever handle 1) into a scene that uses nothing of
ContactBench's FR3 setup: a plain `newton.ModelBuilder`, Newton's UR10
(`newton.utils.download_asset("universal_robots_ur10")`, added with `add_usd`), Newton's IK, and a plain
`SolverMuJoCo` with the settings the kit is calibrated with. The module comes from its USD stage through
`force_bench.kit.usd.add_asset_usd` (see [docs/custom_scenes.md](../../docs/custom_scenes.md)).

```bash
force-bench kit export-usd doorknob-module --variant "Handle 1"   # -> outputs/kit_usd/doorknob-module/Handle_1.usda
python examples/custom_scene/ur10_door_handle.py
```

`--stage <path>` loads another stage of the doorknob module with the lever handle, for example one with the
7.5 mm spring's fit, and `--seconds` sets the simulated time (default: push, hold and return):

```bash
force-bench kit export-usd doorknob-module --variant "Handle 1" --param K=0.67 --param tau0=0.25 --out my_stages
python examples/custom_scene/ur10_door_handle.py --stage my_stages/doorknob-module/Handle_1.usda
```

The run takes about a minute on a recent GPU after the first one, which compiles kernels and downloads the UR10.

The UR10 carries a push rod on its tool flange (`add_push_rod`: a mounting plate and a 12 mm × 100 mm rod, two
cylinders on the flange body) and pushes the lever with it in a straight line 85 mm from the spindle at 25 mm/s, as
in the measurement the handle's spring was fitted to (a UR arm with a wrist force/torque sensor,
[examples/force_response](../force_response/README.md)). The handle turns against the printed spiral spring and the
pinion pulls the latch in through the rack until the handle reaches its stop; when the rod backs off, the spring
turns the handle back and the latch springs out. The script prints the handle angle, the latch travel, the force
between the rod and the handle, and the torque that force puts on the spindle next to the calibrated spring's,
tau0 + K · angle (0.15 N·m + 0.48 N·m/rad for the default 5 mm spring):

```
   t [s]  handle [deg]  latch [mm]  push force [N]  torque on spindle [N.m]  spring tau0 + K a [N.m]
   1.27        10.33        4.06            3.37                    0.227                    0.237
   3.52        36.88       14.48            5.94                    0.455                    0.459
   6.02        27.00       10.60            7.05                    0.377                    0.376
{'max_handle_deg': 39.13, 'max_latch_mm': 15.37, 'final_handle_deg': -0.0, 'final_latch_mm': -0.0,
 'spindle_torque_over_spring_median': 0.978}
latch retracted past the jamb (15.24 mm) and returned
```

The torque follows the spring within a few percent, on the way in (the rod turns the handle) and on the way out (the
spring turns it back against the retreating rod). Single frames jump where the rod's contact with the handle's convex
hull changes, and on the way out the handle can lose the rod for a frame. The push slows near the handle's stop and
ends 0.3° short of it, since a stiff arm driven into a hard stop only builds force. Any tool goes on the flange the
same way: shapes added to the `ee_link` body before `finalize()`.

The scene in a few calls:

```python
builder = newton.ModelBuilder()
newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
builder.add_usd(ur10_usd, ...)                                   # your robot
h = add_asset_usd(builder, stage, pose=pose)                     # the kit asset, its CAD frame at `pose`
model = builder.finalize()
h.prepare_model(model)                                           # contact stiffness, damping, condim of the calibration
solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, **h.solver_options())
h.after_solver(model, solver)                                    # warns where the solver differs from the calibration
# step at 60 fps x 32 substeps, colliding every substep, as the calibration does
```

Any other kit stage goes in the same way; `h.joints` and `h.bodies` name the mechanism's parts by role
(`h.joints["knob"]`, `h.joints["latch"]` here) and `h.params` holds the variant's physical parameters.
