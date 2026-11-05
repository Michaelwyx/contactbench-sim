# Embodiments: the fingers on the FR3 hand

An **embodiment** is the set of fingers the simulated FR3's Franka Hand carries. The arm, the hand housing, its two
position-driven finger carriages and the TCP frame (`fr3_hand_tcp`) are the same for every embodiment. What changes
is what the carriages carry: the fingers' geometry, where the fingertips are relative to the TCP, how compliant the
fingers are, their contact material, the solver settings they need, and the checks they add to a verdict.

Every task runs with the stock fingers, `franka`, the default. A task runs with another embodiment only if it
supports it: its scene then mounts that embodiment's fingers, a scripted policy uses their geometry, and the verdict
includes the embodiment's own checks. In this release, task 2's scene (`retrieve_utensil_from_bin`, a scene without a
scripted policy) supports the `finray` fingers; it is driven by teleoperation or by a policy of your own.

## Available embodiments

```bash
force-bench embodiments          # the embodiments, and the tasks that support each
```

| | `franka` (default) | `finray` |
|---|---|---|
| fingers | the Franka Hand's own fingers, rubber pads on rigid fingertip blocks | a 3D-printed Fin Ray finger (TPU 95A) on each carriage, through a printed adapter ([finray.md](finray.md)) |
| width across the closing axis | 17.5 mm | 25.8 mm |
| fingertip beyond the TCP, along the approach | 9.5 mm | 100.8 mm |
| fingertip thickness along the closing axis | 15.2 mm (block) | 5.3 mm over its lowest 6 mm, widening to 40 mm at the base |
| opening (gap between the contact faces) | 0 to 80 mm | 0 to 80 mm (the contact face is on the stock pad plane) |
| compliance | rigid | pseudo-rigid-body linkage of the finger's walls and ribs, 71 bodies per finger; TPU modulus 26 MPa, a data-sheet estimate |
| contact | elastomer pads, friction set by each task | TPU, μ = 1.0 |
| solver needs | none | MuJoCo `impratio` 10, collision every substep, stiffened loop welds |
| added verdict checks | none | `finray_deformation` (the finger's bending, gated) |
| run time, relative | 1 | about 1.5 |
| tasks | every task | task 2, `retrieve_utensil_from_bin` (a scene without a scripted policy), every arrangement |

The `finray` fingertip is the lowest point of the simulated linkage; the printed finger's tip is 1.0 mm lower, because
the solid wedge below the junction of its two walls is not modelled.

## Run a task with an embodiment

```bash
force-bench info retrieve_utensil_from_bin                          # the "embodiment" line lists what it supports
force-bench teleop-ui --scene retrieve_utensil_from_bin --embodiment finray    # drive it by hand
force-bench run retrieve_utensil_from_bin --embodiment finray --seed 2
                                                  # -> outputs/appendix_tasks/retrieve_utensil_from_bin__finray_seed2/
python -m force_bench.appendix_tasks.retrieve_utensil_from_bin --embodiment finray     # the task module directly
```

`--embodiment` combines with `--seed` and `--set`, in `force-bench run` and in `force-bench teleop-ui`. A run with a
non-default embodiment writes to its own folder, `outputs/appendix_tasks/<row>__<embodiment>[_<settings>][_seed<N>]/`,
so it never overwrites the row's default run; `--name` chooses another folder. `--embodiment franka` is the default
and changes nothing.

The run records its embodiment. Task 2 writes it to `scene.json`. A task with a scripted policy writes it to its
verdict, `appendix_qa.json`: the embodiment under `setup.embodiment`, what it changes under `changes_vs_suite_task`,
and its checks under `criteria`, which the overall `pass` includes.

A task that does not support an embodiment, or an embodiment name that does not exist, is refused before anything is
built:

```
$ force-bench run stack_cups --embodiment finray
force-bench run stack_cups: this task does not support the 'finray' embodiment (Fin Ray TPU fingers on a printed
adapter); it supports franka (see force-bench embodiments)
```

## Which tasks support which

| task | `franka` | `finray` |
|---|---|---|
| 2. Retrieve single utensil from bin, `retrieve_utensil_from_bin` (a scene without a scripted policy) | yes | yes, every arrangement |
| every other task | yes | no |

How the `finray` fingers sit in task 2's scene is described in
[finray.md](finray.md#retrieve-single-utensil-from-bin-task-2).

## How an embodiment is integrated

This section and the next are for adding an embodiment of your own. An embodiment consists of three parts.

1. **A registry entry** in `force_bench.embodiments` (`packages/force-bench/src/force_bench/embodiments/__init__.py`).
   `EMBODIMENTS` maps each name to an `Embodiment` record: its title and summary, the numbers `force-bench
   embodiments` shows (finger width, fingertip depth beyond the TCP, fingertip thickness, opening), its compliance and
   contact in words, the solver options it needs, the checks it adds, and `impl`, the `"module:Class"` of its
   simulation code. `TASKS` maps each task module that supports more than the default to the embodiments it
   supports. The registry imports no simulation code, so the command line reads it to list embodiments, to describe
   them and to refuse an unsupported combination.
2. **A fingers class**, a subclass of `force_bench.embodiments.fingers.Fingers`, created by `embodiments.load(name)`.
   `Fingers` itself is the stock hand and changes nothing. A subclass overrides what differs; the `finray` column
   below is the reference implementation, `force_bench.finray.embodiment.FinRayFingers`:

   | member | what a task uses it for | `franka` | `finray` |
   |---|---|---|---|
   | `mount(rig)` | build the fingers, right after the robot | nothing | switches the stock finger shapes off; builds the adapter, base block and linkage on each carriage; sets `impratio` 10 and collision every substep |
   | `finalize(rig)` | solver settings that need the built solver | nothing | stiffens the 32 loop welds (MuJoCo's default is a 20 ms spring) and checks they were all found |
   | `is_finger(label)` | which robot bodies keep contact with the task objects when a task filters the arm | `"finger" in label` | also the linkage bodies (`fr3/finray_*`) |
   | `finger_bodies(model, side)` | the bodies of one finger | the carriage body | the carriage and its linkage bodies |
   | `pad_probe(model, side, shapes)` | the force on one finger from the task object | one body | every body of the finger, summed |
   | `tip_below_tcp`, `face_offset`, `tip_half_width`, `tip_thickness`, `tip_top` | grasp planning in the TCP frame | the fingertip block | the linkage at rest |
   | `face_bottom_above_tip` | where the flat contact face ends, above the fingertip's lowest point | 0 | 0.53 mm |
   | `underside(y)` | the height of the finger's underside above its lowest point, `y` outward from the face (planning a fingertip over parts lying under it) | flat under the block | the wedge-shaped tip, rising to the base block |
   | `boxes(q)` | the fingers' envelope opened to `q`, for clearance and set-down planning | two boxes per finger | seven slabs per finger, from the tip to the adapter |
   | `checks(states, windows)` | the embodiment's own verdict checks | none | `finray_deformation` |

3. **The task.** A task that supports embodiments takes `--embodiment` (`embodiments.add_argument(parser, module)`),
   loads the fingers and calls their hooks in this order:

   ```python
   fingers = embodiments.load(name)            # refused earlier if the task does not support `name`
   # ... add the FR3 with the task's finger drives
   fingers.mount(rig)                          # right after the robot
   # ... the scene; arm links filtered from the task objects unless fingers.is_finger(label)
   rig.finalize(...)
   fingers.finalize(rig)
   rig.probes["padL"] = fingers.pad_probe(rig.model, "L", target_shapes)
   rig.probes["padR"] = fingers.pad_probe(rig.model, "R", target_shapes)
   # ... the episode, planned with tip_below_tcp, face_offset, boxes(q), face_bottom_above_tip, underside(y)
   criteria.update(fingers.checks(states_npz, {"grasp": [...], "free": [...]}))
   ```

   The verdict also records the embodiment under `setup.embodiment` and its lines under `changes_vs_suite_task`.

For the default embodiment every hook leaves the task's scene and values unchanged, so a row run without
`--embodiment` builds the same scene and runs the same policy; the tests check the values and the envelope against
the task's own constants.

Task 2 has no policy: it builds the scene with the fingers mounted, fills the bin and hands the scene to
teleoperation or to your own policy (`Task.setup()`); `fingers.checks(...)` judges a recorded episode.

## Adding an embodiment

The steps below add a finger set called `flatpad`, a flat rigid pad on each carriage; replace the numbers with your
finger's.

**1. Geometry and attachment.** Write a `Fingers` subclass whose `mount(rig)` builds the fingers on the two carriage
bodies, `fr3/fr3_leftfinger` and `fr3/fr3_rightfinger`. Shapes are given in the carriage's frame (the URDF finger
link): x across the finger, y outward from the gripper's centre, z along the approach, away from the hand. In this
frame the stock pad face is the plane y = 0 (to 0.08 mm) and the TCP is at z = 45.0 mm. Put your contact face on
y = 0 (then `face_offset` = 0 and the faces are `2 q` apart) or set `face_offset` to where it is. Switch the stock
finger shapes off (`shape_flags = 0`) if yours replace them. `mount` is called right after the robot is added, before
any other body or joint, so new bodies can extend the robot's articulation. Label every new body `fr3/<something>` so
the tasks' robot-contact probes count it.

```python
# my_fingers.py
import numpy as np
import warp as wp
import newton

from force_bench.embodiments.fingers import FINGER_LINKS, Fingers


class FlatPadFingers(Fingers):
    """A rigid 20 x 4 x 30 mm pad on each carriage, its contact face on the stock pad plane."""
    tip_below_tcp = 0.012          # the pad's lowest point beyond the TCP: z = 45 + 12 mm in the carriage frame
    face_offset = 0.0              # the contact face on the stock pad plane
    tip_half_width = 0.010
    tip_thickness = 0.004
    tip_top = 0.018                # its top, behind the TCP: z = 45 - 18 mm

    def mount(self, rig):
        b = rig.builder
        for side, link in FINGER_LINKS.items():
            body = b.body_label.index(link)
            for sid in b.body_shapes.get(body, []):
                b.shape_flags[sid] = 0                                  # the stock finger is gone
            cfg = newton.ModelBuilder.ShapeConfig(mu=1.2, density=0.0)  # the carriage keeps the scene's mass
            b.add_shape_box(body, xform=wp.transform(wp.vec3(0.0, 0.002, 0.042), wp.quat_identity()),
                            hx=self.tip_half_width, hy=0.002, hz=0.015, cfg=cfg, label=f"fr3/flatpad_{side}")
        rig.solver_options.update(self.info.solver)
        if self.info.collide_every_substep:
            rig.collide_every_substep = True

    def boxes(self, q):
        out = []
        for s in (+1.0, -1.0):
            y0, y1 = sorted([s * q, s * (q + self.tip_thickness)])
            out.append((np.array([-self.tip_half_width, y0, -self.tip_top]),
                        np.array([self.tip_half_width, y1, self.tip_below_tcp])))
        return out
```

**2. Compliance, if the finger is soft.** Model it as bodies on joints hung from the carriage. The Fin Ray is a
pseudo-rigid-body linkage, rigid links on spring-loaded revolute joints (`force_bench.finray.prb` and `.model`).
Under MuJoCo:

- the new tree joints must extend the robot's articulation (`builder.joint_articulation`, `articulation_end`);
  `force_bench.finray.model.mount_on_fr3` shows how;
- a closed loop (a rib welded to both walls) is a FIXED joint outside every articulation
  (`joint_articulation = -1`), which MuJoCo turns into a WELD equality whose default `solref` is a 20 ms spring: set
  it in `finalize(rig)` after the solver exists (`force_bench.finray.model.tune_solver`);
- light links on stiff springs need armature for the explicit spring to stay stable. The Fin Ray uses
  `a = k (dt / 0.35)²` per joint, so each spring's ω·dt is 0.35 whatever its rate; this changes the dynamics, not
  the statics;
- links of a fraction of a gram make soft MuJoCo contacts and weak friction, because contact stiffness scales with
  the body's mass ([physics_notes.md](physics_notes.md), rule 2.7): the Fin Ray needs `impratio` 10.

Give the fingers' bodies the labels your `is_finger`, `finger_bodies` and `pad_probe` recognise.

**3. Grasp reference.** Set `tip_below_tcp` (the lowest point of the finger beyond the TCP, along the approach),
`face_offset`, `tip_half_width`, `tip_thickness` and `tip_top`, and override `boxes(q)` with an envelope that
contains every colliding shape of both fingers opened to `q` (the tasks use it to keep the hand clear of the bin and
to find a set-down height). If the flat contact face does not reach the finger's lowest point, or the finger is not
flat underneath, also set `face_bottom_above_tip` and override `underside(y)`. Compute them from the same geometry
the simulation collides with (`force_bench.finray.embodiment.rest_points` and `underside_table` do this for the
linkage).

**4. Contact and solver settings.** Choose the contact material (`mu`, `gap` of the shapes) and put what the solver
needs in the registry record (`solver`, `collide_every_substep`); `mount` applies them to the rig. They apply to the
whole scene, so a task that supports the embodiment runs all of it with them.

**5. Check the finger on its own.** Before any task, press it. The Fin Ray has a cylinder-press probe
(`python -m force_bench.finray.probe --z -60`), compared with a beam reference and a quasi-static solution of the same
linkage ([finray.md](finray.md#validation)). Record force against indentation and how the finger moves; if you can,
press the printed finger the same way and scale the modulus.

**6. Register it.** Add a record to `EMBODIMENTS` in `force_bench/embodiments/__init__.py` (`impl` names a module
that Python can import, and the class in it):

```python
Embodiment(
    name="flatpad", title="Flat rigid pads",
    summary="a rigid 20 x 4 x 30 mm pad on each carriage",
    impl="my_fingers:FlatPadFingers",
    finger_width_mm=20.0, tip_below_tcp_mm=12.0, tip_thickness_mm=4.0, opening_mm=(0.0, 80.0),
    compliance="rigid", contact="silicone, mu 1.2", solver={}, checks=()),
```

If it adds verdict checks, implement `checks(states_npz, windows)`: `windows` maps `"grasp"` (fingers closed on the
part) and `"free"` (fingers open) to lists of `(t0, t1)`; return `{name: {"pass": bool, ...measurements}}`.

**7. Make a task support it.** In the task module:

- add the module and its embodiments to `TASKS` in the registry, and call
  `embodiments.add_argument(ap, "<module>")` in its `main()`; pass the name on to the task, and default the output
  folder to `embodiments.run_name(<row>, name)`;
- in the task, `fingers = embodiments.load(name)`; call `fingers.mount(rig)` right after the robot,
  `fingers.finalize(rig)` right after `rig.finalize`, filter arm links with `fingers.is_finger`, build the pad probes
  with `fingers.pad_probe`, and plan with the fingers' geometry instead of fixed fingertip constants;
- put what the policy must do differently for the new finger in one table keyed by embodiment (how far the fingers
  close on a part, how deep the tips go), with the default's entry holding the values the task uses for the stock
  fingers, and refuse the settings you did not adapt the policy to;
- record the embodiment in the verdict's `setup` and its changes in `changes_vs_suite_task`, and add
  `fingers.checks(...)` to the criteria;
- run the row with the new embodiment from the nominal start and with a few `--seed` values, and once without
  `--embodiment` to confirm that the default run is unchanged.

**8. Tests and docs.** Extend `packages/force-bench/tests/test_embodiments.py` (the registry's numbers agree with
the class; the task's default values are unchanged; the command line builds the right folder) and write a reference
page like [finray.md](finray.md): the physical finger, the model and how it compares with a reference, what it
changes in the tasks that support it, and its limits.

## Limits

- Only the fingers change. The FR3 arm, the Franka Hand housing and its two independent position-driven carriages
  (in task 2: 8 kN/m drive, 15 N limit per finger, the scene's finger mass) are the same for every embodiment; a
  different gripper (another stroke, coupled or underactuated fingers) is not an embodiment here.
- A task with a scripted policy supports an embodiment only once its policy is adapted to it. No scripted policy is
  adapted to `finray` in this release; task 2's scene, which has no policy, supports it. Adding an embodiment never
  changes the default's settings.
- An embodiment's solver needs apply to the whole scene. For `finray` these are `impratio` 10 and collision every
  substep, which task 2 uses with the stock fingers as well.
- A soft finger costs run time: the two Fin Ray fingers add 142 bodies, 142 joints and 32 welds, and a scene takes
  about 1.5 times as long as with the stock fingers.
- MuJoCo warns that the joint order is not depth-first, because the linkages' joints come after the hand's two
  finger joints; the simulated finger still agrees with the quasi-static solution of the same linkage
  ([finray.md](finray.md#validation)).
