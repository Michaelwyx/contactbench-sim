# Teleoperation guard and acceptance gate

Two pieces make the tasks usable by something other than their scripted policies:

- the **guard** (`force_bench.teleop.guard`), a command filter and live monitor that sits between any operator or
  policy and the robot. It is on by default in `force-bench teleop-ui` ([teleoperation.md](teleoperation.md)) and in
  the minimal viewer `force-bench teleop`;
- the **acceptance gate** (`force_bench.teleop.gate`, `force-bench gate`), which drives a task with a fixed abusive
  operator through the guard and fails if the *model* loses integrity.

Rule numbers refer to [physics_notes.md](physics_notes.md).

## Why the tasks need a guard

Every task's scripted policy moves the way the contact model needs: rate-limited approaches, force-servoed presses,
gentle grips, commands closed on the measured part state. These are the conditions under which the simulated physics
is valid. A human operator, or any unscripted policy, violates them within seconds, and the failures are **silent**:

- **Tunnelling** (rule 6.1). A strong drive punches a small part through a compliant contact within one frame, and the
  sampled contact force then reads 0 N. The part passes through a wall while the force channel says "no contact".
- **Grip force is not the commanded number** (rules 4.5, 6.5). It depends on the finger overdrive and on the scene, and
  squeezing harder can eject a thin part instead of holding it.
- **A position lead is a force** (rule 6.2): 10–50 N per mm of lead through the arm, about 400 N/mm for a fingertip
  against a rigid wall at short reach. A few millimetres of lead into a blocked feature is hundreds of newtons.
- **Rollout QA is after the fact.** The penetration, flying-object and envelope checks of `rollout_qa.json` run on
  the saved states, so a broken teleoperation session would only be known as broken after it ends.

## The guard

A pure command filter plus a passive monitor. It never touches the scene, the solver or the assets, so it can sit in
front of any command source: keyboard, mouse gizmo, SpaceMouse, gamepad or a policy.

```
raw command [x, y, z, yaw, finger width]
            ->  AdmittanceFilter   (b)  EMA + deadband
            ->  SpeedGovernor      (a)  contact-aware rate and lead clamp + press servo
            ->  GripForceLimiter   (c)  force-limited fingers
            ->  robot (IK + joint-rate limit)
            ->  LiveQA             (d)  online violations + HUD
```

### (a) Contact-aware speed governor

The command's per-frame TCP displacement **and** its lead over the *measured* TCP are both clamped to a fraction of
the smallest mechanical feature near the gripper. Feature sizes are declared per scene through a `FeatureMap` of
spheres and boxes, because the smallest clearance is a property of the CAD, not something the solver can report.
Clamping the **displacement** bounds tunnelling; clamping the **lead** bounds the press force.

The per-frame step is `0.25 × feature`, kept between 4 mm/s and 250 mm/s; the lead is `1.0 × feature`, kept between
3 mm and 20 mm; the yaw rate scales from 1.5 rad/s in free air down to 0.05 rad/s. At the defaults and 60 fps:

```
free air (50 mm)      4.167 mm/frame  (250 mm/s)   lead <= 20.0 mm
2.1 mm slot           0.525 mm/frame  ( 32 mm/s)   lead <=  3.0 mm (the floor)
1.5 mm hard surface   0.375 mm/frame  ( 22 mm/s)   lead <=  3.0 mm
0.10 mm plug blade    0.067 mm/frame  (  4 mm/s, the floor)   lead <= 3.0 mm
```

At 400 N/mm a lead clamp alone is not enough, so the governor also runs a **force servo** on the lead budget: while
the measured contact force (the larger of the pad forces and the rest of the hand) exceeds `press_force_max_N`
(15 N), the budget is bled down and, if the force keeps rising, driven **negative**, down to −4 mm: the command
retreats behind the measured TCP. Below the ceiling the budget recovers by 0.04 mm per frame.

Zones are declared in **TCP coordinates** and must be padded toward the robot: the FR3 fingertip assembly reaches
about 25 mm behind the pad face, so a gripper open to 40 mm is about 60 mm wide around the TCP and touches a wall long
before the TCP gets near it. A clamp that engages on contact is already too late; `pad` grows each zone outward, and
overlapping zones take the smallest feature.

### (b) Admittance / smoothing filter

An exponential moving average plus a deadband on the operator input: `alpha = 0.25` (a cut-off of about 2.7 Hz at
60 fps) and a deadband of 0.25 mm (0.002 rad of yaw, 0.2 mm of finger width). The per-frame step cap is the
governor's, which knows the local feature. Smoothing comes first, so the governor is not fed operator chatter, and the
filter's state follows the *clamped* command, so the average cannot wind up against an unreachable target.

### (c) Force-limited finger drive

A leaky integrator on the measured per-pad excess opens the finger command whenever the squeeze exceeds
`grip_force_max_N` (6 N per pad, the top of the grip range that holds a thin rod without ejecting it). Closing is
rate-limited to 50 mm/s; opening is never limited, because opening is always safe. The finger drives are position
servos, so the squeeze is set by overdrive against a scene-dependent stiffness (rule 6.5) and the commanded width is the
only actuator the guard has.

### (d) Live QA

The checks of `RolloutQA`, run online:

- **Penetration proxy.** The solver's own contact geometry, evaluated the way the contact kernels do it:
  `d = dot(-n, p0_w - m0*(-n) - (p1_w + m1*(-n)))`, with `d < 0` meaning overlap (`m0`, `m1` are the shapes' contact
  margins, zero for the surface points MuJoCo reports). Pairs of shapes on the *same* rigid body are excluded
  (rule 3.6), and so are the shape pairs listed in `qa_ignore_shape_pairs` (interfaces that overlap by design). It is a
  proxy: it sees only the pairs the broad phase generated; an exact signed distance between the meshes, as in
  `rollout_qa.json`, is the ground truth.
- **Displacement-velocity spike.** Frame-to-frame position speed above 2.5 m/s, the flying-object detector (rule 9.1).
- **Workspace envelope.** Tracked bodies stay inside a world box.
- **Non-finite state.**
- **Over-grip alarm** at 1.25× the limiter's ceiling, so a guard that is *holding* the squeeze at its limit reads as
  working, not as violating.

Violations surface immediately through `hud_lines()` or an `on_violation` callback and accumulate in `summary()`:

```
[GUARD ON ] zone=rod_grasp    feat=  1.50mm vmax=  22.5mm/s lead<=  3.00mm F=   4.2N pad= 5.83N clip=RLG
            QA pen= 0.221mm vel= 0.01m/s viol=   4 {'grip_force': 4}
```

`zone` and `feat` are the zone and feature size at the command, `vmax` and `lead` the resulting limits, `F` the
contact force the press servo reads, `pad` the larger pad force; `clip` lists the active clamps (`R` rate, `L` lead,
`Y` yaw, `P` press servo, `G` grip over its ceiling).

### Design rules

The guard follows six rules. Each prevents a failure that looks exactly like an asset defect.

1. **The lead clamp holds, it never follows.** When the lead is exceeded, the command keeps its previous value; it is
   never moved onto the measured TCP. The arm's steady gravity droop is itself a lead, so a command that follows the
   measured TCP lets the arm droop further, follows it down again, and walks it into the floor. Only the force servo
   may put the command behind the measured TCP, and only because a measured force says so.
2. **The lead floor is above the droop.** A lead limit tighter than the arm's own droop (0.5–2 mm) measures the arm's
   weight, not contact. The floor is 3 mm; fine press bounding is the force servo's job, which reads newtons instead
   of inferring them from a position error.
3. **The grip relief has anti-windup.** A force that does not fall as the jaws open is not a grip; integrating
   against it drives the finger command onto its own joint stop. The relief integrates only while opening still
   lowers the force, and the command stays 1 mm clear of the stop.
4. **Every output is rate-limited**, the force servo's retreat included (up to 4× the advance rate). A retreat that
   jumps to its target every frame ping-pongs about the measured TCP.
5. **Pad force is measured from the contact arrays**, on the bodies whose labels end in `leftfinger` and
   `rightfinger`, never looked up by a scene's probe names: a renamed probe reads 0 N, and the limiter never engages.
6. **Joint targets are rate-limited after the IK** (rule 6.4), to the FR3's datasheet joint velocity limits
   (`FR3_QD_MAX` in `force_bench.teleop.gate`: 2.175 rad/s for joints 1–4, 2.61 rad/s for joints 5–7). The
   damped-least-squares IK, seeded from its own previous answer, wanders through near-singular configurations when
   asked for an unreachable pose and steps the joint targets faster than the arm can move, while the Cartesian
   command sits still.

A Cartesian-space safety layer alone is not sufficient: bounding the TCP command bounds neither the joint-space path
the IK chooses to reach it nor the forces a stiff drive develops on the way. Rule 6 is applied between the IK and the
drives (`drive_arm_guarded`, below), not inside `TeleopGuard`.

```bash
force-bench guard-selftest             # the guard's own self-test: no GPU, no scene
```

The self-test checks the free-air and tight-zone clamps, the lead clamp, the grip relief and the deadband, and prints
the resulting limits.

## Using the guard

### In an interactive session

The guard is on by default in both teleoperation front ends. Press **T** to toggle it live, or start with `--no-guard`
to bypass the clamps while live QA and the alarms keep running.

```bash
force-bench teleop --viewer gl                                # the minimal viewer on the playground scene
force-bench teleop --viewer gl --no-guard
force-bench teleop --viewer null --num-frames 300 --test      # headless
force-bench teleop-ui --scene plug_in_usb_2                   # the full UI (teleoperation.md)
```

The minimal viewer prints the HUD twice a second and the session summary on exit. Its other options are those of
Newton's examples (`--viewer gl|usd|rtx|rerun|null|viser`, `--num-frames`, `--headless`, ...;
`force-bench teleop --help`), plus `--collect-dir` to save the session as a demonstration episode.

### In front of a policy or another command source

```python
from force_bench.teleop.guard import TeleopGuard, GuardCfg, FeatureMap
from force_bench.teleop.gate import drive_arm_guarded

fmap = FeatureMap(free_air_m=0.05)
fmap.add_aabb("slot", lo, hi, feature_m=0.0021, pad=0.03)   # CAD numbers
guard = TeleopGuard(GuardCfg(fps=rig.fps, substeps=rig.substeps), rig.model, feature_map=fmap)
guard.reset(cmd0)                       # cmd = [x, y, z, yaw, finger_width]

# every control frame
cmd = guard.filter(raw_cmd, measured_tcp=tcp, pad_forces=(fL, fR),
                   contact_force=max(fL, fR, hand_force))
drive_arm_guarded(rig, cmd[:3], tool_quat, cmd[4])   # IK + joint-rate limit (design rule 9.1)
...  step the physics ...
guard.observe(state, contacts, t, pad_forces=(fL, fR))
for line in guard.hud_lines(t=t):
    print(line)
```

`raw_cmd` is the TCP position in world coordinates (m), the yaw about world z (rad) and the finger width (m, 0 to
0.04). `tool_quat` is the tool orientation for the guarded yaw; the gate turns the task's start orientation by the yaw
about world z. `drive_arm_guarded(rig, pos, quat, fingers)` needs a `Rig` with the FR3 (`rig.ik`). `measured_tcp` is
the TCP position of the current state, `pad_forces` the contact force on each fingertip (N), and `contact_force` what
the press servo reads (N). The guard governs x, y, z, yaw and the jaw; any other rotation axis passes through it
unclamped (the teleoperation session limits pitch and roll to the governor's local yaw rate). `guard.enabled = False`
bypasses every clamp while live QA keeps running, and `guard.summary()` returns the clamp counts, the peak pad force
and the live QA report. Scripted policies, human demonstrations and learned policies can all pass through the same
filter.

The main `GuardCfg` fields, with their defaults:

| field | default | meaning |
|---|---|---|
| `fps`, `substeps` | 60, 16 | the control rate; set them to the rig's |
| `feature_fraction` | 0.25 | per-frame step as a fraction of the local feature |
| `lead_fraction` | 1.0 | lead budget as a fraction of the local feature |
| `v_max_free`, `v_min` | 0.25, 0.004 m/s | speed ceiling in free air, speed floor at the tightest feature |
| `lead_max_free`, `lead_min` | 0.020, 0.003 m | lead ceiling and floor |
| `yaw_rate_free`, `yaw_rate_min` | 1.5, 0.05 rad/s | yaw rate in free air and at the tightest feature |
| `press_force_max_N` | 15 | contact force above which the press servo cuts the lead |
| `press_retreat_max` | 0.004 m | how far behind the measured TCP the servo may pull the command |
| `ema_alpha`, `deadband_m` | 0.25, 0.00025 m | admittance filter |
| `grip_force_max_N` | 6 | grip ceiling per pad |
| `grip_close_rate` | 0.05 m/s | finger closing rate limit |
| `finger_range` | (0, 0.04) m | finger command range |
| `qa_pen_max_m`, `qa_vel_max` | 0.0015 m, 2.5 m/s | live QA penetration budget and flying-object speed |
| `qa_grip_force_N` | 1.25 × `grip_force_max_N` | over-grip alarm |
| `qa_stride` | 3 | frames between penetration reads |
| `qa_envelope_lo`, `qa_envelope_hi`, `qa_track_bodies` | none, none, all | workspace box and the bodies it applies to |
| `qa_ignore_shape_pairs` | none | shape-index pairs excluded from the penetration proxy |

`FeatureMap(free_air_m)` takes zones with `add_sphere(name, center, radius, feature_m, pad)`,
`add_aabb(name, lo, hi, feature_m, pad)` and `add_mesh_bounds(name, trimesh, center, feature_m, pad)`;
`feature_at(p)` returns the feature and zone at a point. For a ContactBench task,
`force_bench.teleop.scenes.task_featuremap(rig, source)` builds the zones the teleoperation UI and the gate use.

## The acceptance gate

An acceptance criterion has to be applied identically to every task. That is `force_bench.teleop.gate`.

### The criterion

> **A task passes the gate if a fixed adversarial session produces ZERO integrity violations. Task success is
> explicitly NOT required.**

An operator who yanks the gripper sideways *should* fail to unlock the door; that is the task being hard, not the
model being broken. What must never happen is the **model** losing integrity:

| violation | what it means | threshold |
|---|---|---|
| `tunnelling` | a part passed through another part; the sampled contact force then reads about 0 N (rule 6.1), so this failure is silent, which is why it needs a gate | a contact pair's overlap grows more than 3 mm beyond its rest depth |
| `floating` | a free body holds or gains height while nothing touches it: it is carried by a constraint that should not exist | more than 8 mm above its start with less than 0.05 N of contact, for 8 consecutive frames |
| `envelope_escape` | a part left the workspace | outside the box of all start positions, grown by 0.5 m |
| `flying_object` | a body moves too fast | frame-to-frame displacement speed above 2.5 m/s |
| `non_finite_state` | the solver diverged | any non-finite body state; the run stops there |

**A gate failure is an asset or mechanism defect, not a policy problem.** No controller can be trusted to avoid a
state the model cannot represent. A physics-QA failure of a scripted rollout, by contrast, is a tuning problem.

The thresholds are integrity-scale, not the 0.25 mm visual standard of `rollout_qa.json`. The tunnelling threshold
follows from the geometry: 3 mm is several signed-distance cells (the kit's default cell is 0.4 mm) and beyond
anything contact compliance explains, so a pair that overlaps 3 mm deeper than at rest has a part inside another.

It is **3 mm of growth, not of absolute overlap** (rule 9.5). Several interfaces overlap at rest by design (a square
drive inserted in its socket, a rack sliding in its bore, a key in its keyway), so the gate holds the task's start pose
through a 0.6 s settle phase, records the rest depth of every contact pair, and only flags a pair whose overlap grows
more than 3 mm beyond that baseline. Both the absolute worst depth and the worst growth are reported whether or not
either trips.

### How it drives a task without touching its code

Every task funnels its rollout through one `Rig.run(...)` call. The gate imports the task module, with the task's own
arguments as `force-bench run` resolves them for the row, and intercepts `Rig.run` (plus any `Rig` subclass in that
module that overrides it), so that at the moment the scene is finalized the scripted policy is replaced by the
adversarial session, and the task's `main()` is stopped before it saves or checks anything. No task file is modified.

The adversary is the abusive operator made task-agnostic. Every phase holds a constant command, so each transition is
a single-frame step:

| phase | duration | command |
|---|---|---|
| settle | 0.6 s | hold the start pose (the rest baseline is taken here) |
| jab down | 2.4 s | 100 mm down |
| over-squeeze | 1.6 s | fingers to 0 in one frame |
| jab in | 2.4 s | 100 mm along the tool's own approach axis |
| yank up and yaw | 2.0 s | 100 mm up and 115° of yaw |
| shake | 2.4 s | ±30 mm reversals every 3 frames |
| release | 0.6 s | back to the start pose, fingers to their start opening |

The task's own start TCP **and orientation** are the anchor, so "jab in" means into the work for a sideways reach and
for a vertical pick alike. The gate takes the controls in whatever state the task left them (an object already in the
pinch stays there), rather than forcing a fixed gripper opening at t = 0. The yaw passes through the guard like the
position, and the joint targets through `drive_arm_guarded`.

### Feature declarations

The governor needs the smallest relevant CAD clearance in each region. The zones are the teleoperation UI's
(`teleop.scenes.task_featuremap`): one box per body, padded by 60 mm (zones are in TCP coordinates and the fingertip
assembly is about 60 mm wide around the TCP), tagged with a declared clearance where the scene module a task builds on
declares one in `TASKS` in `teleop/gate.py`, and with the 1.5 mm hard-surface standoff otherwise. The table and the
floor are hard surfaces.

| region | clearance | provenance | tasks |
|---|---|---|---|
| nut on bolt | 0.23 mm | constant helical clearance over 5 turns (`force_bench.tasks.nut_bolt`) | 20 |
| card in holder slot | 0.75 mm | 1.5 mm of slot play | 5 |
| plug blade in face plate | 0.10 mm | 0.45 mm x-play, 0.10 mm y-play per side | 22 |
| key ring, keys and hanging board | 0.5 mm | the threading corridor | 9 (`hang_keychain`) |
| any other part | 1.5 mm | sub-millimetre printed fits; the hard-surface standoff | all |

Each report carries `feature_coverage`: the bodies, the declared and undeclared labels, declarations that match no
body (`stale_declarations`) and the zones built from the model, so a body nobody wrote a number for is visible in the
output instead of silently defaulted (rule 9.6).

### Running it

```bash
force-bench gate --list                           # every task, its scene module, and whether clearances are declared
force-bench gate plug_in_usb_2                    # one task
force-bench gate plug_in_usb_2 --repeat 6         # verdict = AND over 6 runs
force-bench gate --all                            # every task with a scripted policy
force-bench gate plug_in_usb_2 --duration 6       # stop the session after 6 s (default 12 s: the whole profile)
force-bench gate plug_in_usb_2 --mode unguarded   # the same session without the guard: a diagnostic, not a verdict
```

The gate runs each task at its row's defaults. Each run writes `outputs/teleop_gate/<task>/teleop_gate.json`
(`TB_OUT_ROOT` moves it): `pass`, the violations by kind and the first 40 of them, the worst overlap, growth, speed
and floating run with the bodies involved, the rest overlap, `feature_coverage`, the guard's clamp counts and peak pad
force, and the thresholds. The console prints one line per task: `PASS`, `FAIL` or `INCON`, the worst overlap and
speed, and the declared features.

### Repeated runs

Contact ordering in the collision reduction is not bit-reproducible on the GPU, so a task near the divergence
boundary can pass one run and fail the next. `--repeat N` runs the gate `N` times and its verdict is the **AND** over
all runs, with every run's result recorded (`run_verdicts`, `flaky`). A verdict that flips is a failing verdict;
resampling until it passes is the one thing an acceptance criterion must never permit.

The repeat count has to match the flip rate. At a 20% per-run failure rate, `--repeat 2` catches it 36% of the time,
`--repeat 6` 74% and `--repeat 10` 89%. Use `--repeat 6` or more, and read a single-run PASS on a task known to be
marginal as "not yet failed", not as "clean".

### Adding a task

A new task gets zones on every part at the hard-surface standoff without any work. Where a part has a smaller
clearance the manipulation has to respect, declare it in `TASKS` in `teleop/gate.py`, keyed by the scene module the
task builds on (its registry `source`) and the mesh label it registers (`rig.meshes`). That is CAD bookkeeping, not
tuning. Then run `force-bench gate <task> --repeat 6` and check `feature_coverage` in the report.

## Known limits

- Clearances are declared by hand; a part without a declaration gets the 1.5 mm standoff. They are not derived from
  the assembly's geometry (pairwise signed distances).
- The press servo bounds *sustained* force well and *impact* force only as well as the speed clamp allows; a first
  touch at the free-air ceiling is still an impact. Declaring a surface zone is the fix, which is why every scene
  declares its table and the floor and why "free air" must not extend to the table.
- The penetration proxy sees only broad-phase pairs; it does not replace an exact signed-distance check.
- The guard is a per-frame CPU filter for one environment; nothing in it is vectorized.
