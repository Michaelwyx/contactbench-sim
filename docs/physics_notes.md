# Physics modelling rules

These rules apply when you build a new asset, mechanism or scene for this simulator: Newton's collision pipeline,
which collides the meshes through signed-distance fields, and the MuJoCo Warp solver (`SolverMuJoCo`) consuming those
contacts, as `force_bench.rig.Rig` and `force_bench.rig.make_solver` set them up. Each rule is stated with its reason.
The solver configuration the rules assume, and with which the kit's parameters were fitted, is listed in
[custom_scenes.md](custom_scenes.md#the-settings-the-calibration-assumes).

Each rule is numbered by its section, and the other docs refer to it by that number ("rule 2.3").

Related: the benchmark tasks in [tasks.md](tasks.md), how the kit's mechanisms are modelled in
[assets.md](assets.md), the tactile sensor in [tactile.md](tactile.md), the guard and gate in
[teleop_guard.md](teleop_guard.md), and adding tasks and assets in [developing.md](developing.md).

## 1. The robot

The Franka FR3 with its hand, from the URDF of Newton's asset repository, is imported with
`collapse_fixed_joints=True` (rule 4.4) and driven through `force_bench.robot.franka`: a kinematic IK model feeds the
position drives of the dynamic arm, so contact forces act back through the arm. `Rig.add_robot` adds it before
anything else (joint coordinates 0 to 8). The finger drives are position servos with a 20 N force clamp
(`set_finger_effort_limit`, MuJoCo's actuator force range), the tasks set each finger's mass to 0.12 kg (rule 4.5), and
`make_solver` gives the robot's nine joints an armature of 0.1 (`TB_MJ_ARMATURE`). The fingertip geometry a grasp has
to respect is in rules 7.1, 7.2 and 7.3.

## 2. Units and the solver

### 2.1 Work in metres

Newton's units are metres, kilograms and seconds. The kit's STL files (`assets/kit`) are in millimetres; the Fusion
exports in `assets/bodies` and the kit's `.obj` visual models are in centimetres, and `force_bench.assets` scales
`assets/bodies` by 0.01 when it loads them.

### 2.2 Keep Newton's contacts

`make_solver` builds `SolverMuJoCo` with `use_mujoco_contacts=False`, so MuJoCo consumes the contacts Newton computes
from the meshes' signed-distance fields. MuJoCo's own collision reduces every mesh to its convex hull: a thread becomes
two smooth cylinders, and a key ring cannot link a key.

### 2.3 Size the contact buffers

Newton's pipeline keeps up to 20000 rigid contacts (`Rig.finalize`), and MuJoCo is given room for 8000 contacts and
32000 constraint rows (`nconmax`, `njmax`; `TB_MJ_NCON`, `TB_MJ_NJMAX`). Left to itself, MuJoCo sizes its buffers from
the initial state; an overflow only prints a warning, the extra contacts are dropped, and the arm can walk through the
table. Raise the limits for a scene with many meshes in contact.

### 2.4 Create contact sensors before the contacts

A `SensorContact` must be constructed before `CollisionPipeline.contacts()` is called (`Rig.finalize` creates its probes
first); code that takes over a finished scene cannot add one and reads the contact arrays directly instead. Call
`solver.update_contacts()` before reading forces.

### 2.5 Friction needs two MuJoCo settings

MuJoCo's default contact (`condim` 3) has no torsional friction, so a pinch resists a twist about the contact normal
only through the spread of its contact patch; `make_solver` sets `condim` 4 (`TB_MJ_CONDIM`). A `condim` 4 contact needs
a non-zero torsional coefficient (`mu_torsional`; zero gives a MuJoCo NaN warning); the tasks that set it use 0.002 m.
The coefficient has units of length (about the contact patch radius), so keep it physical rather than raising it until a
grasp holds. At MuJoCo's default `impratio` of 1 the friction rows are much softer than the normal ones, and a pinch
creeps tangentially even when its Coulomb capacity is many times the load. Scenes that hold a part or an angle by
friction raise it (`rig.solver_options["impratio"]`; several tasks and the Fin Ray fingers use 10).

### 2.6 Stiffen equality constraints, and give light, stiffly sprung links armature

A coupling between joints (a rack and pinion: a MuJoCo joint equality, Newton's mimic) and a closed loop (a fixed joint
outside every articulation, `joint_articulation = -1`, which MuJoCo turns into a weld) are soft by default, with a 20 ms
time constant. Set their `solref` to two substeps, critically damped; the door modules also use `solimp` (0.99, 0.999)
and an armature on the pinion. A light link on a stiff spring needs armature to stay stable: the Fin Ray fingers use `a
= k (dt / 0.35)²` per joint ([finray.md](finray.md)), which changes the dynamics, not the statics.

### 2.7 Contact stiffness is bound to the timestep and to the mass

A MuJoCo contact's time constant cannot go below about two substeps, so its stiffness rises with the substep count and
scales with the effective mass of the body it acts on. `make_solver` sets a contact stiffness of 1e7 N/m and damping of
1e4 N·s/m on every shape (`TB_MJ_KE`, `TB_MJ_KD`): much softer contacts tunnel under load, and much stiffer ones are too
stiff for the substep and tunnel too. The scenes run 32 substeps at 60 fps (`TB_SUBSTEPS`); a loaded contact still sinks
measurably (about 0.17 mm at 10 N). A very light part makes a soft contact: when it has to deflect a stiff spring
through contact, give it a conditioning mass (the USB sticks and the Ethernet plug carry 15 g) or stiffen the contact on
the spring's own shapes only (`force_bench.kit.mechanisms.usb_slot.stiffen_jaw_contacts`), and check the spring's
deflection, not just the part's position.

## 3. Geometry and collision

### 3.1 Know what each margin does

`make_newton_mesh(margin=)`, forwarded to `Mesh.build_sdf`, only pads the mesh's bounding box before the signed-distance
field is discretized, and `ShapeConfig.gap` only widens the broad phase's detection range. The field that would inflate
the contact surface, `ShapeConfig.margin`, is 0 in these scenes, so there is no designed standoff between surfaces. With
the resolution, the SDF padding sets the field's cell size, `(extent + 2 × margin) / resolution`: the floor on how
faithfully the field reproduces a thin wall. An overlap of about one cell is a representation effect; an overlap of many
cells is real. Give thin walls and tight fits several cells: the kit uses 0.4 mm cells by default, 0.2 mm for the cup's
1 mm wall and 0.12 mm for the charging blocks and USB sticks ([assets.md](assets.md#how-an-asset-is-simulated)).

### 3.2 Subdivide meshes before they collide

Contact samples a mesh's vertices against the counterpart's signed-distance field, so a face larger than the wall it
meets can pass through it without a contact. Subdivide to edges of at most 3.5 mm (`force_bench.geometry.subdiv`,
through `trimesh.remesh.subdivide_to_size`) before building the Newton mesh; parts that meet thin walls or fine features
use 1 to 3 mm.

### 3.3 Model sub-millimetre features as larger ones

Pockets, gates, detents and clearances of a few tenths of a millimetre or less are below what the contact resolves at a
usable time step: the signed-distance field and the contact gap fill them. Model such features as macro features with
the same kinematics and document the change in the asset's module. If the feature is what a task is about, leave the
task out rather than change it ([tasks.md](tasks.md#tasks-not-included)).

### 3.4 Give a compliant entry a ramp steeper than the friction angle

A compliant member that the moving part must wedge open self-locks if its entry ramp is shallower than atan(μ): a 4°
ramp at μ = 0.4 jams at any force. A 30° flare with a vertical grip face works, and that is also how real contact
springs are shaped.

### 3.5 The same holds for every compliant member the moving part must wedge open

(rule 3.4): an entry angle steeper than the friction cone, or the mechanism self-locks.

### 3.6 Exclude same-body shape pairs from contact-derived penetration

Two shapes on the same rigid body overlap by design: `collapse_fixed_joints` merges the FR3's hand shells into
`fr3_link7`, and they overlap by 22.2 mm in the stock URDF. With same-body pairs excluded (static–static pairs fall out
of the same rule), the contact arrays give a cheap online penetration proxy, `d = dot(-n, p0_w - m0*(-n) - (p1_w +
m1*(-n)))`, with `d < 0` meaning overlap. `SolverMuJoCo.update_contacts` writes surface points into the contact buffer
and leaves its margin fields from earlier contents, so take the margins `m0`, `m1` as zero with MuJoCo. The proxy sees
only broad-phase pairs; an exact signed distance between the meshes stays the ground truth (rule 9.1). The guard's live
QA uses it ([teleop_guard.md](teleop_guard.md#d-live-qa)).

## 4. Springs, joints and mass

### 4.1 Replace printed springs with joint drives

A printed spring is not simulated as a deforming body. Leave out its bodies (`force_bench.assets` skips files matching
`*spring*`) and reproduce it with a drive (`target_ke`, `target_kd`) on a prismatic or revolute joint: of the part the
spring moves, or of a rigid block riding such a joint where the spring meets the moving part (the outlet's sprung pads,
the USB slot's jaws). Fit the rates to measured forces where they exist
([examples/force_response](../examples/force_response/README.md));
[assets.md](assets.md#from-printed-part-to-simulated-mechanism) lists how each kit mechanism is modelled. Damp every
spring drive: an undamped return spring makes its link a pendulum that rings.

### 4.2 A replacement spring touches only the part it moves

Collision-filter it against the housing it lives in and against the other parts of the mechanism, so that it carries no
load the real spring does not.

### 4.3 Change masses with `set_body_mass`

`force_bench.assets.set_body_mass` updates the mass, the inverse mass and the inertia together. Writing
`builder.body_mass` alone leaves the other two stale and destabilizes the solver.

### 4.4 Never weld to a massless frame link

Import robots with `collapse_fixed_joints=True`, which merges massless URDF frame links (the FR3's `fr3_hand_tcp`,
`fr3_link8` and hand) into a link with mass; welding a payload to a massless frame body makes the arm's state
non-finite. Attach payloads to a link with real mass.

### 4.5 Give driven bodies a realistic mass

What a fingertip pinch can transmit is set by the driven body's inertia, not only by the drive's stiffness, and a light
body also makes a soft contact (rule 2.7). The URDF fingertips are too light to hold a tool against a contact reaction;
the tasks set each FR3 finger to 0.12 kg with `set_body_mass`. Set the mass before raising gains, and move the force
thresholds with it (rule 6.3).

### 4.6 Grip through finger contact, never through a joint

A gripped body keeps its own joint (a free joint, or its mechanism's) and is held by fingertip contact only:
`Rig.robot_grip(body, filter_fingers=False)` filters every arm link against it and lets the fingertips touch it. Every
body has exactly one joint leading to it; a second joint from the hand is a loop that MuJoCo refuses ("Multiple joints
lead to body N"), and such a coupling can carry a part the fingers do not hold. A part that drives a joint chain of its
own (a knob on its spindle) is gripped the same way.

### 4.7 Start every body off its joint limits, and keep limits outside the mechanism's clearance

A body that starts exactly on a joint limit is pinned there; offset the limit, or the start, by a small margin. A limit
tighter than the mechanism's own clearance takes the load that belongs to the mechanism: a door stop set inside the
bolt's play carries the reaction, and the bolt-to-strike force reads 0 N.

### 4.8 Put the command envelope in the joint limits

If a driven joint's command only ever uses part of its travel, make the joint's limits that range, with a little
headroom. A clip in the controller does not exist for anyone else who drives the scene, and a rate limit cannot replace
it: a slow target that still ends far past the work pushes as hard. Under MuJoCo a joint limit is a soft constraint
unless it is made hard: set `rig.hard_joint_limits = True` before `rig.finalize()` (a raw `solreflimit` of two substeps
with damping ratio 0.5, `TB_MJ_LIMIT_ZETA`), and give a strong position drive a force clamp (`joint_effort_limit`) as
well; neither alone stops a stiff drive. Do not raise `solimplimit`'s `dmax` to harden a limit: Newton's conversion
divides by `1 - dmax`, which makes the limit softer.

## 5. Collision cadence

### 5.1 The collision cadence is a modelling parameter

`Rig.run` collides once per frame and steps every substep on those contacts, unless `rig.collide_every_substep = True`,
which collides before every substep. Contacts found once per frame are stale for the rest of the frame, so once per
frame is only valid while nothing lands, breaks contact or turns quickly within a frame: a rotating grasp can pivot
through stale contacts, and a part sliding on a surface can lose its friction on most substeps. Collide every substep
unless a scene or a phase has been verified with once per frame, and set it at build time (rule 5.3). Most tasks collide
every substep; the mechanism probes, in which nothing lands or breaks contact, collide once per frame.
`TB_COLLIDE_SUBSTEP=0|1` forces either cadence for a run, and `--collide-substep` for a teleoperation session.

### 5.2 Collide every substep while anything falls or settles

A body released from rest falls `0.5 g (1/60 s)² = 1.36 mm` in its first frame, more than the contact gap of any shape
in these scenes (at most 0.8 mm). When contacts are found once per frame, the contact found on the frame it lands is
already deep, and every substep resolves that same stale depth again: the body is ejected instead of settling, and the
deepest overlap of the rollout becomes a settle artefact.

### 5.3 Set model-integrity parameters at build time

The acceptance gate and the teleoperation UI replace the task's control callback, so anything the scripted policy sets
per frame does not exist when someone else drives the scene. Set the safe value at build time and let the policy relax
it only for the phases it has verified: `rig.collide_every_substep = True` next to the solver construction, couplings as
build-time joint equalities (rule 2.6), the envelope as joint limits (rule 4.8). Ask of every line that matters for
integrity what happens to it when someone else drives the scene.

### 5.4 Change the cadence as rarely as possible

Switching from every substep to once per frame can eject a body that rests untouched on a sharp edge: on the switch its
contact on the edge is lost and found again already deep. A wider contact gap does not fix it. Run a phase at once per
frame only where it has been verified, and check every body in the scene during it, not just the gripped one.

## 6. Drives, forces and probes

### 6.1 Strong drives punch small parts through contacts

A drive with much more authority than a contact's breakthrough force pushes a small part through it within one frame,
and the sampled force then reads 0 N, which looks like a missing contact. A blocking feature holds only if the drive
that attacks it is weaker than what the contact can resist; manipulation that tests a lock or a gate must be gentle.

### 6.2 A position-command lead is a force

The arm turns a lead of the command over the measured TCP into force: 10 to 50 N per mm through the arm, about 400 N/mm
for a fingertip against a rigid wall at short reach, so a 1 mm lead can be hundreds of newtons. Anything that presses (a
scripted policy, an operator, a learned policy) must close a servo on the measured contact force and let the lead budget
go negative, so that the command retreats behind the measured TCP; a lead clamp alone does not bound the force. The
guard does this ([teleop_guard.md](teleop_guard.md#a-contact-aware-speed-governor)).

### 6.3 Gains and force thresholds are one system

Raising a drive's authority (a finger mass, a gain) raises every contact force it produces, and can lift them above the
policy's own force interlocks, which then stop the task. Change gains and newton-valued thresholds together.

### 6.4 Rate-limit joint targets after the IK

The damped-least-squares IK (`FrankaIK.solve`), seeded from its own previous answer, wanders through near-singular
configurations when asked for a pose the arm cannot reach and steps the joint targets faster than the FR3's datasheet
limits, while the Cartesian command sits still; the stiff arm drives chasing them make the state non-finite at modest
contact forces. Clamp the joint targets to the arm's velocity limits after every solve (`FR3_QD_MAX` and
`drive_arm_guarded` in `force_bench.teleop.gate`). A Cartesian limit bounds neither the joint path the IK chooses nor
the force a stiff drive develops on the way.

### 6.5 Measure grip force in the scene

The force a finger drive delivers depends on its overdrive, the contact, the masses and the solver settings, not only on
its gains. Measure it in the scene, as the pad contact force through a probe resolved by counterpart (rule 6.6) or with
a free-hang sag test, instead of computing it from nominal gains.

### 6.6 Resolve force probes by counterpart

A whole-body probe (`SensorContact.total_force`) sums every contact on the sensing body, so a control law that servos on
it reads its own grip as the load, and a symmetric pinch cancels in the vector sum. Restrict the probe to the
counterpart (`counterpart_bodies` or `counterpart_shapes`, read per pair from `force_matrix`), and sum magnitudes where
a pinch should read as contact (`force_bench.appendix_tasks.common.RobotContactProbe`).

## 7. Grasps

### 7.1 Check a grasp pose against the fingertip envelope, and assert it

The FR3's collision fingertips reach 9.93 mm below the TCP point, and the pad faces sit at ±(dof − 0.085 mm), so the pad
gap is 2·dof − 0.17 mm. Assert at build time that the pads close on the part and not on the air beside it: a grasp that
misses reads as zero pull force, which points at the mechanism rather than the grasp. A free part cannot wait for the
fingers: write the finger coordinates into `builder.joint_q`, so that the pinch exists from the first frame. The sag
budget has a sign: the part sags down, so the pads walk up the feature.

### 7.2 Check containers against the whole finger, opened

The FR3's collision fingers are wider above their tips than at them: the tip is 15.2 mm thick (outer face at dof + 15.18
mm) for its last 18.5 mm, then the finger flares at 30° to 24.8 mm, and the screw mount reaches 26 mm. A hand that fits
a container closed can drive both fingers into its walls when it opens to let go. A per-pair readout (robot link ×
shape, from `SensorContact.force_matrix`) shows it. The same envelope rules out reaching the floor of the kit's fastener
bin (task 3).

### 7.3 Match undercuts to the hand in height, not only in width

Before designing a hooking grasp, scan the part for undercuts (cross-sections every 0.5 mm) and the hand for its own
upward-facing ledge: the FR3 fingertip has a 2.53 mm ledge 19.3 mm above its tip. A grasp can fit in plan view and be
impossible in elevation, because the tip cannot go below the table. Measure both, and measure the hand as carefully as
the part.

### 7.4 A stiff grasp transmits the arm's tracking error

A form-closed grasp (pads straddling a bar, so that pull and push are both pad normals) is stiff in both directions, so
the arm's steady Cartesian tracking error at that reach is delivered straight into whatever the object is attached to: a
kinematically perfect grasp on a lever can lean hundreds of newtons on its latch without moving it. Measure the steady
tracking error at the working pose before choosing the grasp; a grasp that can slip in the unwanted direction is often
the safer one.

### 7.5 Remove a prying torque instead of out-squeezing it

A top-down pinch on a flat-lying part grips its two narrow edges, so a roll about the part's own axis has to spread the
pads; what resists it is the finger drive's stiffness, not the clamp force. Often neither helps: a torque applied by the
environment (a sideways move while the far end of the part is still in its container) has to be removed by the
trajectory. If changing the clamp force barely moves the outcome, stop changing the clamp force.

### 7.6 Hold a tapered part with an admittance grip

A position-driven parallel gripper generates force from interference, and most handles, shanks and utensils taper. When
the part creeps toward its thin end, the interference drops, the force drops and the creep accelerates: a runaway that
looks like a friction failure. The grip lost per millimetre of slip is the drive stiffness times the taper's change of
radius per millimetre. Hold the measured pad force in a deadband by advancing the finger command (capped), grasp as far
from the thin end as the task allows, and never bracket a "grip force" on a tapered part open loop: the bracket measures
the taper, not the grip.

### 7.7 Budget the slide of a friction grasp

On a long pull through a friction grasp the part advances less than the hand; measure the slip ratio (commanded travel
per millimetre of part travel). If `stroke × slip_ratio` exceeds the pad slide the part offers, the stroke needs a
re-grasp or form closure, and no servo work can substitute.

### 7.8 A re-grasp needs something to hold the part while the hand is open

That is a property of the mechanism, not of the controller. Before designing a multi-bite stroke, ask what holds the
part between bites. If nothing does (for example when the one feature that could hold it is the one the task disables),
the fix is form closure or a detent in the asset, not a better servo or a supporting spring.

### 7.9 A pinch on a narrow flat face is weak in roll

A parallel-jaw pinch on a flat face (a key ring's square section) resists sliding well and roll about the face's own
tangent poorly: the moment arm is half the face width, so the capacity is about the pad force times half the width (8.8
mN·m at 3.5 N per pad on a 2.5 mm face). Loads hanging off the part on a longer arm overwhelm it, and clamp force cannot
make up the difference without crushing the section. Change the trajectory instead: arrive and release rather than
arrive and lower.

### 7.10 Resolve gravity in the pad frame before tuning a grasp

A load that presses the part onto the pads as a normal force is a shelf; the same load carried tangentially is held by
friction alone, which creeps (rule 2.5). Which component of a part's weight the pads carry depends on the part's
attitude, and matters more than μ, the clamp force or the collision cadence. If all of it is tangential, change the
trajectory, not the grip.

### 7.11 Measure the contact regime the task is in

A probe that slides a pinch continuously against a fixed part measures sliding friction; a task that pulls
quasi-statically on a part that can move depends on holding friction, and the first does not predict the second. Build
the probe to match the task's regime.

## 8. Commanding the arm

### 8.1 Close millimetre-scale engagements on the measured part position

The arm's gravity droop (0.5 to 2 mm at the tasks' reaches) swallows millimetre-scale engagement windows. When a
mechanism needs a part held within a small window, close the TCP command on the measured part position (an integrator)
instead of trusting the open-loop command.

### 8.2 A near-zero-clearance passage cannot be held open loop

A part that must pass a slot with hundredths of a millimetre to spare has to be held within that window; the arm's droop
alone buries it in a wall, and stiffening the grasp only transmits the droop better. Close the TCP command on the
measured part pose (rule 8.1), so that the arm, not the slot wall, reacts the mechanism's spring load.

### 8.3 Latch a follow-the-part command one way past the run-out

Closing a command on the measured part position (rule 8.1) is right while the mechanism guides the part and wrong once
it stops: past the run-out any dip pulls the tool down, and the tool drags the part with it. Latch the command one way,
in the direction the task is going.

### 8.4 Integrate droop only while the command is stationary

A correction that integrates `commanded − measured` TCP during a transport leg integrates the joint tracker's own
dynamic lag (a second-order tracker running at `v` settles `v²/2a` behind): the command leads, the arm chases, and a
carried part is shaken out of the pinch. Gate the integrator on a stationary command (a step below 0.05 mm per frame),
so that it measures the arm's weight. Use a quintic smootherstep rather than a cubic for any leg that carries a pendulum
(the cubic leaves an acceleration step at every waypoint), and budget each transport leg twice: against the contact
model and against the joint tracker's velocity limit (`DQ_MAX`).

### 8.5 Budget transport speed for the loosest body in the scene

A fast transport can keep the gripped part perfectly tracked while a body held by nothing (a key hanging on a ring by
contact) is thrown away, and a success check that looks only at the gripped part still passes. Contact-breaking
transitions, such as a part peeling off the table, are the critical moments; collide every substep through them (rule
22). Budget every leg against whatever is held by nothing, and check every task object in the verdict.

## 9. Checks

### 9.1 Detect flying objects on displacement, and measure penetration on geometry

Raw solver body velocities show single-substep impulse spikes that are artefacts; frame-to-frame displacement speed is
the flying-object detector. Measure penetration geometrically, with signed distances between the meshes (trimesh),
independently of the solver. `force_bench.qa.RolloutQA` does both on saved rollouts.

### 9.2 Check a simulated thread against its mesh

Before building a task on a thread, measure, in a rig with nothing else attached, how far the part travels per turn and
how many turns it takes to disengage, and compare them with the lead and the thread length measured on the mesh. A
travel that disagrees with the mesh is a model defect, not a property of the part.

### 9.3 A detector must not gate its own signal, and a go/no-go must be allowed to say no

A test for a loss (a part falling back) must not run only while a condition that the loss itself breaks still holds;
track the peak instead. A check that can answer "not yet" needs code for that answer. Normalise such tests against a
measured quantity, not a commanded one: the arm is a stiff spring (about 80 kN/m) that swallows much of a small
commanded motion.

### 9.4 Hold the start pose before anything else

A scripted policy's first command moves the arm, which can hide a start pose that is not statically stable (a fingertip
inside the floor). A teleoperator who pauses, or a policy whose first action is small, lands there at once. Hold the
start pose for half a second (30 frames) in QA; the gate and the teleoperation session do.

### 9.5 Acceptance tests measure degradation and hand over the controls

An absolute penetration threshold fails every interface that overlaps by design (a key in its broach), so record a
per-pair rest baseline during a settle phase and flag only growth beyond it. Take the controls in whatever state the
scene left them: forcing the gripper to a fixed opening at t = 0 is a step command no operator gave. An external wrench
applied straight to a part (`body_f`) is not an operator action, and rigid-body models tunnel under one at 60 fps
whether healthy or not, so it belongs in a diagnostic, never in an acceptance test.

### 9.6 Declare the clearances the gate needs, on the exact labels

The guard scales its travel and lead caps by the smallest declared feature near the TCP. An undeclared part gets the 1.5
mm hard-surface standoff, which is too loose for a sub-millimetre fit, and a declaration keyed to a label that does not
exist matches nothing while reading as coverage. Declare clearances on the exact `rig.meshes` labels in `TASKS` of
`force_bench.teleop.gate`, and check `declared`, `undeclared_labels` and `stale_declarations` in the gate's
`feature_coverage` ([teleop_guard.md](teleop_guard.md#feature-declarations)).

## 10. Rendering

### 10.1 Render quality comes from supersampling and lighting, not from resolution

Newton's tiled camera takes one hit-shaded sample per pixel, so silhouettes stair-step at any resolution.
`force_bench.recorder.RolloutRecorder` renders 2 × 2 samples per output pixel and box-filters them in a Warp kernel,
lights the scene with a key and a fill directional light (only the key casts shadows), and composites a backdrop
gradient from the shape-index image, because the renderer's own background ignores `clear_color`. The tasks are drawn in
the printed kit's colours (`force_bench.kit_look`; `TB_LOOK=studio` colours shapes by role instead). Frames stream into
an H.264 writer (CRF 16) and are kept JPEG-compressed for contact sheets. Rollouts render at 1280 × 960;
`TB_RENDER_FAST=1` renders 800 × 600 drafts without supersampling, and `TB_MULTIVIEW=0` turns the second view off.
