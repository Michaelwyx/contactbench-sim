# The kit in your own scenes

The printable kit's simulated assets ([assets.md](assets.md)) are built for NVIDIA Newton and are not tied to
ContactBench's Franka setup: any of them, with its springs, gears, stops and contact settings, can go next to your
own robot in your own scene. There are two routes.

- **Python**: build the asset with `force_bench.kit.add_asset`, as the benchmark tasks do, in a scene built with
  `force_bench.rig.Rig`, and load your own robot with Newton's `add_urdf`, `add_mjcf` or `add_usd`.
- **USD**: export the asset once as a self-contained USD stage (`force-bench kit export-usd`) and add the stage to
  any Newton `ModelBuilder` with `force_bench.kit.usd.add_asset_usd`. This route needs neither the kit's STL files nor
  its Python build code at run time, and a stage can be versioned, shared and inspected like any other asset.

Both routes give the same model, for every variant of every asset
([Comparing a stage with the Python-built asset](#comparing-a-stage-with-the-python-built-asset)).

## The settings the calibration assumes

Every spring rate, preload, friction and contact setting of the kit is fitted or checked with one solver
configuration: the one `force_bench.rig.Rig` builds, through `force_bench.rig.make_solver`. Forces measured in another
configuration differ, by little or by much depending on the setting:

| setting | value | why it matters |
|---|---|---|
| solver | `newton.solvers.SolverMuJoCo` (MuJoCo Warp), `use_mujoco_contacts=False` | Newton computes the contacts (signed-distance fields of the meshes); MuJoCo's own collision would reduce every mesh to its convex hull |
| timestep | 1/1920 s: 60 frames per second, 32 substeps | MuJoCo's contact and limit stiffness are bound to the timestep (no time constant below 2 timesteps) |
| MuJoCo options | `solver="newton"`, `integrator="implicitfast"`, `cone="elliptic"`, `iterations=15` (24 in the mechanism probes of `force-bench kit check`), `ls_iterations=100`, `nconmax=8000`, `njmax=32000` | |
| `impratio` | 1; 10 in a scene with a dial | the dials hold their angle by friction, which creeps at 1 |
| contact stiffness, damping | 1e7 N/m, 1e4 N·s/m on every shape (the USB slot's jaws 1e9 N/m), `condim` 4 | `make_solver` sets them on the whole scene, the robot's shapes included |
| collision | every substep (the doorknob and lock probes once per frame); `CollisionPipeline(reduce_contacts=True, broad_phase="sap", rigid_contact_max=20000)` | |
| joint limits | the door, doorknob and lock modules' stops are hard (`mjc:solreflimit` = 2 timesteps, damping ratio 0.5); a scene with a dial or the Ethernet plug makes every joint limit hard | a default MuJoCo limit is soft enough that a strong drive goes through it (the Ethernet plug's latch tab presses against its stop) |

Each USD stage records these settings (`customData["contactbench"]["calibration"]`), and `add_asset_usd`'s handle
warns where a scene differs ([Import](#import)).

## Route 1: Python, your robot in a ContactBench scene

`force_bench.rig.Rig` is the scene builder the benchmark tasks use. It holds a Newton `ModelBuilder` (`rig.builder`)
with a ground plane, and `rig.finalize` builds the model, the solver with the settings above, the collision pipeline
and, with `record=True`, the camera and recorder. `rig.add_robot` adds the FR3 the tasks use; any other robot goes
straight into `rig.builder`:

```python
import numpy as np
import warp as wp
import newton
import newton.utils
from force_bench.rig import Rig
from force_bench.kit import add_asset, place_on_table

rig = Rig("my_scene")                       # rig.builder: a ModelBuilder with a ground plane
rig.table(center_xy=(0.5, 0.0), half=0.5)   # an optional work surface, top at z = 0
newton.solvers.SolverMuJoCo.register_custom_attributes(rig.builder)

# your robot, from URDF, MJCF or USD (here Newton's UR10)
ur10 = newton.utils.download_asset("universal_robots_ur10") / "usd" / "ur10_instanceable.usda"
rig.builder.add_usd(str(ur10), xform=wp.transform((0.0, 0.0, 0.001), wp.quat_identity()),
                    collapse_fixed_joints=False, enable_self_collisions=False)
# rig.builder.add_urdf("my_arm.urdf", xform=..., floating=False)
# rig.builder.add_mjcf("my_arm.xml", xform=...)

# kit assets: pose = where the asset's CAD frame goes; place_on_table centres it on z = 0
h = add_asset(rig, "doorknob-module", "Handle 1", pose=place_on_table("doorknob-module", "Handle 1", xy=(0.8, 0.0)))
h.bodies, h.joints, h.params              # what was built, and the variant's physical parameters

rig.finalize(probes=[], cam_pos=np.array([1.5, -1.0, 0.8]), cam_target=np.array([0.6, 0.0, 0.1]), record=False)
# step with rig.run(seconds, control_callback), or with your own loop over rig.solver, rig.pipeline, rig.contacts:
#   rig.pipeline.collide(rig.s0, rig.contacts); rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, dt)
```

`rig.finalize(probes, cam_pos, cam_target, record=True)` takes the labels of the bodies whose contact force is logged
every frame, and the camera's position and target. `rig.run(seconds, control)` calls `control(t)` once per frame and
steps the physics; with `record=True` it renders every frame, and `rig.rec.save("rollout")` then writes the video,
the state log and the force log to `rig.out_dir` (`outputs/task/<name>/`).

Your robot's joints, drives and collision filters are yours to set: `rig.add_robot`, `rig.robot_grip`,
`rig.drive_arm` and the inverse kinematics behind it are FR3-specific. `rig.finalize` sets the contact stiffness,
damping and `condim` of every shape in the scene, your robot's included, to the calibration's values.

## Route 2: USD stages

### Export

```bash
force-bench kit export-usd                          # every variant -> outputs/kit_usd/<asset>/<variant>.usda
force-bench kit export-usd outlet door --out my_stages
force-bench kit export-usd doorknob-module --variant "Handle 1"
force-bench kit export-usd doorknob-module --variant "Handle 1" --param K=0.6 --param tau0=0.2   # a re-tuned spring
force-bench kit export-usd cup --format usdc        # binary crate files
```

`--variant` takes a variant's label or a unique part of one. `--param` takes any parameter of the mechanism's builder
(`force_bench.kit.mechanisms.<model>.build`), as `add_asset(..., **params)` does, so a re-tuned asset can be saved
and shared as a stage. From Python: `force_bench.kit.usd.export_variant("outlet", "0.85mm", "outlet.usda",
ke=45000.0)`.

A stage holds one variant, standing on z = 0 and centred on the origin; metres, kilograms, Z up. It has no
`PhysicsScene`, so adding it to a scene leaves that scene's gravity and solver options unchanged. Its content:

| in the stage | USD | Newton reads it as |
|---|---|---|
| rigid bodies | `UsdPhysics.RigidBodyAPI`, `MassAPI` (mass, centre of mass, principal inertia) and `NewtonMassAPI` (`newton:inertia`, the full tensor) | body pose, mass, inertia |
| collision shapes | `UsdGeom.Mesh` / `Cube` with `CollisionAPI`; `MeshCollisionAPI` (`none`, or `convexHull` for the doorknob grips); `NewtonCollisionAPI` (`newton:contactMargin`, `newton:contactGap`); `NewtonSDFCollisionAPI` (resolution, narrow band, padding); `mjc:condim` | the shape, its signed-distance field (built at `finalize`), contact margin and gap |
| contact material | a bound physics `Material` with `PhysicsMaterialAPI` (friction, restitution) and `NewtonMaterialAPI` (torsional and rolling friction, contact stiffness and damping) | `ShapeConfig.mu`, `mu_torsional`, `mu_rolling`, `ke`, `kd` |
| drawn-only parts | plain `UsdGeom.Mesh` (the printed springs, inlays), print colour as a `UsdPreviewSurface` | visual shapes |
| invisible colliders | `purpose = "guide"` (the grips' convex hulls, the door's strike pockets) | colliders without the VISIBLE flag |
| joints | `PhysicsRevoluteJoint`, `PhysicsPrismaticJoint`, a generic `PhysicsJoint` with `PhysicsLimitAPI` per axis (the push-down dial's knob); limits; `PhysicsDriveAPI` (the printed springs: stiffness, damping, target); `NewtonJointAPI` (`newton:armature`, `newton:friction`, limit stiffness and damping, velocity limit); `mjc:solreflimit`, `mjc:solreffriction` | joints, drives, stops, dry friction |
| gear couplings | `MjcEqualityJointAPI` and `NewtonMimicAPI` on the follower joint (`newton:mimicJoint`, `newton:mimicCoef1` = minus the pinion's pitch radius, `mjc:solref`, `mjc:solimp`) | a MuJoCo joint equality and a Newton mimic constraint |
| collision filters | `FilteredPairsAPI` | shape filter pairs |
| articulations | `PhysicsArticulationRootAPI` on the joint that ties a part to the world (a sprung pad, a knob, a spindle), or on the root body of a floating base: the Ethernet plug, whose latch tab is a second body hinged to it (a body without a joint to the world is free, so the plug's free joint has no prim) | articulations; for the floating base, a free joint for the plug and the tab's hinge in one articulation, as `add_asset` builds it |
| initial joint positions | `newton:linear:position` / `newton:angular:position` and PhysX's `state:linear:physics:position` / `state:angular:physics:position` (the standard dial's brake pads start pushed out by the hub, the seated Ethernet plug's latch tab 5° pressed) | `joint_q` |

Values that USD has no attribute for are stored under `contactbench:` and applied by `add_asset_usd`:

| `contactbench:` | what |
|---|---|
| `jointForce` on a joint | a constant joint torque: the doorknob module's spring preload (`tau0`) |
| `actuatorMode` on a joint | Newton's actuator mode where the drive's gains imply another: a position drive with zero stiffness (the dial knobs, whose probes raise the stiffness at run time) reads back as a velocity drive |
| `sdfMaxResolution` on a shape | the exact resolution of the shape's signed-distance field; Newton's importer only takes multiples of 8, so `newton:sdfMaxResolution` holds the next one |
| `solverContactStiffness` on a shape | a contact stiffness the mechanism sets once the solver exists (the USB slot's jaws, 1e9 N/m) |
| root `customData["contactbench"]` | the asset, its variant and label, the pose of its CAD frame, the variant's physical parameters and measured geometry (`handle`, the same `params` and `info` as `add_asset` gives), the settings the calibration assumes (`calibration`), and an exact record of every value the builder held (`record`) |

### Import

```python
import warp as wp
import newton
from force_bench.kit.usd import add_asset_usd

builder = newton.ModelBuilder()
# ... your robot, your table ...
h = add_asset_usd(builder, "outputs/kit_usd/doorknob-module/Handle_1.usda",
                  xform=wp.transform((0.8, 0.0, 0.0), wp.quat_identity()))   # the stage's z = 0 on your table top
model = builder.finalize()
h.prepare_model(model)          # the calibration's contact stiffness, damping and condim, on every shape
solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, **h.solver_options())
h.after_solver(model, solver)   # re-applies the asset's contact settings, warns where the solver differs
h.check_scene(dt=1.0 / 1920.0, collide_every_substep=True)   # ... and where your stepping differs
```

`add_asset_usd` registers `SolverMuJoCo`'s custom attributes on the builder (needed for the `mjc:` attributes and
the gear couplings), calls `builder.add_usd`, applies the `contactbench:` extras, and returns a handle with the same
fields as `add_asset`'s (`h.bodies`, `h.joints`, `h.shapes` by role, `h.params`, `h.info`, `h.pose`), indexed in your
builder. Instead of `xform` you can give `pose=`, the world pose of the asset's CAD frame as `add_asset` takes it.
Given a `force_bench.rig.Rig` instead of a builder, it also makes the scene settings `add_asset` makes (hard joint
limits for a dial or the Ethernet plug, and `impratio` 10 for a dial). `h.apply_preload(control)` restores the
constant joint forces (the doorknob's spring preload) into a `Control` whose `joint_f` you zeroed.

The same stage also loads with Newton's importer alone:

```python
newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
builder.add_usd("outputs/kit_usd/outlet/0.4mm_indent_spring.usda", xform=...)
```

which reproduces most of the model ([next section](#what-the-usd-route-reproduces-and-what-the-helper-adds)).
Newton 1.5 then warns once that `newton:linear:position` (or `angular`) is a non-schema attribute (the dials'
starting pad positions, the Ethernet tab's); `add_asset_usd` silences that warning.

Exporting needs `usd-core`; adding a stage to a Newton scene also needs Newton's USD schemas. Both come with the
`usd` extra, `pip install -e "packages/force-bench[usd]"`: usd-core and newton-usd-schemas 0.4.1, the schema release
Newton 1.5.0 is built against (tested with usd-core 26.11). Newton's own `pip install "newton[importers]"` installs
the schemas too, with mesh-processing packages the kit does not need.

[examples/custom_scene](../examples/custom_scene/README.md) does all of this with a robot other than the FR3:
Newton's UR10, with a push rod on its tool flange, pushes the doorknob module's lever handle 85 mm from the spindle
(the distance its spring was fitted at) until the handle reaches its stop and the latch has retracted past the door's
jamb, then lets it spring back. The torque the rod puts on the spindle follows the calibrated spring,
tau0 + K · angle, within a few percent.

### What the USD route reproduces, and what the helper adds

A stage added with `add_asset_usd` gives the same model as `add_asset`, field by field, for every variant.
`force-bench kit check --from-usd --structure` compares the two on the CPU, without simulating, and writes
`outputs/usd_roundtrip/structure.json`.

Newton's `add_usd` alone reproduces, to 32-bit rounding, every body's pose, mass, centre of mass and inertia; every
shape's geometry, pose, friction, contact stiffness, margin and gap, signed-distance-field settings (except some
resolutions, below), visibility and colour; every joint's frames, axis, limits, drive, armature, dry friction, limit
stiffness and MuJoCo impedances; the initial joint positions; the gear couplings (as `SolverMuJoCo` joint equalities
and Newton mimic constraints, with their impedance); the collision filters; the door's five-joint articulation; and
the Ethernet plug's floating base (the plug's free joint and the latch tab's hinge in one articulation). For many free
objects and fixtures that is the whole model. The helper adds the rest:

- an articulation for each joint that ties a single body to the world: the outlet's and USB slot's sprung pads, the
  dials' knobs, pads and plates, and the doorknob and lock modules' joints when they stand alone. UsdPhysics does not
  list the body of a one-body articulation, so Newton imports the joint outside any articulation: `SolverMuJoCo`
  still simulates it (and warns), but `newton.eval_fk` skips it;
- the doorknob module's spring preload, as `builder.joint_f` (every doorknob-module variant and the door);
- position drives with zero stiffness (the dials' knobs), which Newton reads back as velocity drives;
- signed-distance-field resolutions that are not a multiple of 8: Newton's importer only takes multiples of 8, so the
  stage holds the next one, a field up to 8 cells finer;
- the doorknob grips' convex hulls, which the importer recomputes from the authored points;
- the push-down dial's knob: its unlimited rotation (a UsdPhysics limit of ±inf reads back as ±1.7e8 rad), and the
  2.5 × 10⁻³ kg·m² armature on its rotation, which keeps the solver finite while the knob rests on its spring bumps.
  `newton:armature` is one value per joint and the knob joint's slide and rotation carry different values, so a scene
  that imports this stage with `add_usd` alone has to set the armature itself;
- the starting pose of the standard and snap dials' brake pads and of the seated Ethernet plug's latch tab: the stage
  holds them at zero joint coordinates, where Newton's importer anchors a world joint, with the starting joint
  position as `newton:linear:position` or `newton:angular:position`, and `newton.eval_fk` puts them back;
- labels and roles: Newton names bodies, joints and shapes by prim path, `add_asset` by role (`outlet/pad_wide`,
  `kit/outlet/outlet_shell`); the kit's probes and tasks look them up by those names;
- the values Newton's importer passes through 32-bit floats in degrees (revolute limits and per-degree gains, mimic
  coefficients), exactly.

### Comparing a stage with the Python-built asset

`force-bench kit check --from-usd --compare` exports every variant, adds each stage with `add_asset_usd` to the scene
`force-bench kit check` builds, runs the same checks and mechanism probes on it and on the Python-built asset, and
writes `outputs/usd_roundtrip/report.json` and `report.md` with every probe's numbers side by side.

```bash
force-bench kit check --from-usd --compare outlet       # both routes for one asset, then the comparison
force-bench kit check --from-usd --compare --summary    # compare the existing reports only
force-bench kit check --from-usd outlet                 # only the USD route -> outputs/usd_roundtrip/usd/
force-bench kit check --from-usd --no-export outlet     # with the stages already in --usd-dir (default outputs/kit_usd)
```

A mechanism matches when both probes pass and

- every calibration number (a spring rate or preload, a friction torque, a sliding force averaged over a depth
  window, a press force, a stop, a gear ratio, a detent, a door angle) agrees with the Python value within 5 % for
  forces, torques and spring rates, 2 % for gear ratios and 1 % for angles and travel, or within a floor for small
  values (0.5 N, 0.005 N·m, 0.05 mm, 0.5°);
- every transient number (a force peak, or a force sampled on one frame at one depth) agrees in the mean of repeated
  runs within the same tolerance or three standard errors of the difference, whichever is larger.

A free, fixture or assembly variant matches when its settle checks pass on both routes and the end state agrees
(lowest point within 0.1 mm, speeds within 1 mm/s).

Contact on the GPU is not bitwise reproducible. Where contact sticks and slips, as when a charger blade wedges the
outlet's pad open, two runs of the same Python-built model differ by several percent in single-frame samples (the
outlet's wedge-in peak by up to 20 %), while depth-averaged insertion forces agree within 1 %. Compare such numbers
over repeated runs: the comparison also reads `outputs/usd_roundtrip/python_repeat*/` and `usd_repeat*/`, written by
`force-bench kit check --out <that folder>` and `force-bench kit check --from-usd --no-export --out <that folder>`.
The doorknob module, lock module and door, whose probes run through joints and gear couplings, give identical numbers
from both routes. The Ethernet probe drives a second plug, built in Python, into port 1 while the stage's own plug
sits latched in port 3.

## Other simulators

The stages are plain UsdPhysics underneath: bodies, masses, joints, limits, drives, materials and collision filters
are standard, so tools that read UsdPhysics (Isaac Sim, Isaac Lab, usdview) should load them. They are not tested in
Isaac Sim or Isaac Lab; what follows is what PhysX's handling of these attributes implies, not a measured result.
PhysX ignores every `newton:`, `mjc:` and `contactbench:` attribute, so in Isaac Sim

- there is no signed-distance collision unless you add it (`PhysxSDFMeshCollisionAPI`), and a triangle-mesh collider
  on a moving body is approximated by its convex hull;
- the gear couplings (MuJoCo joint equalities) are dropped: the doorknob's latch and the lock's deadbolt no longer
  follow the spindle;
- the doorknob's spring preload, the contact stiffness and the hard joint stops are not applied.

The calibration uses MuJoCo Warp with Newton's contacts, so forces differ in PhysX, and the springs and contacts need
re-tuning against it before they mean anything.
