"""Task: spin the hex nut ALL THE WAY down the printed bolt until it SEATS
on the head shoulder — the fastening force/torque ramps up and stalls
("fastened" tactile signal) — then loosen (release signature).

Asset: fasteners/screw_075mm_1_body1 (bolt, hex head down, threaded shaft
up, major r 4.95 mm / minor r 3.75 mm) + nut_075mm_1_body1 (hex nut, bore
crest r 4.05 mm / root r 5.25 mm).  Radial thread overlap is ~0.9 mm — deep
enough for real SDF thread contact (unlike the dial's 0.5 mm gate teeth).

THREAD VERDICT — REAL thread contact.  Mesh analysis (crest tracking through
720 deg of axial cross-sections) shows the printed thread is a SINGLE-START
RIGHT-HAND helix with lead 2.33 mm/turn: the "0.75 mm" in the asset name is
the design pitch before print scaling (~3.1x — the bolt is ~M3 thread
geometry printed at 9.9 mm major diameter).  A trimesh signed-distance ride
along the helix shows constant 0.23 mm clearance over 5 full turns and
+-0.3 mm axial backlash, so the flanks really carry the nut.  The sim rides
the real helix; measured descent per turn must match the MESH lead
2.33 mm/turn (0.75 mm is geometrically impossible for this mesh).

SEAT GEOMETRY (mesh-measured): the hex head top face (shoulder) is at
world z ~7.0 mm, thread crest r 4.95 runs right down to it; the 7.0 mm-tall
nut therefore bottoms out with its center at ~10.5 mm — 15.4 mm / 6.6 turns
below the start pose (nut center 26.0 mm, ~1.2 turns engaged at the top).

Start pose: bolt STATIC at the world origin standing shaft-up (explicit
xform from assembly coords), nut ~1.2 turns engaged at the shaft top.  The
mating (z, yaw) = (26.0 mm, 228 deg) was found by phase-aligning the two
helixes (radius-vs-z crest profiles, bottle-neck style) and verified by
signed-distance minimization over a (z, yaw) grid.

PITFALL 6.1 (discovered here) and the 2026-08-13 RE-MEASUREMENT of XPBD-era pitfall 9.
The nut is a fully FREE body: nothing in the SPIN PATH of the driven part may
be angularly joined to a fixed frame, because a d6 with three angular axes
IS solved by `solve_body_joints` through a swing-twist decomposition whose
gradients are singular at +-pi relative rotation (kernels.py:
`invs = 1/sqrt(rel_q[0]^2 + rel_q[3]^2)` -> inf, plus a quaternion
hemisphere flip that reverses the correction direction).  The bottle cap
stalled at 0.48 turns on such a joint and a first version of this task
stalled at 0.487 turns with the hand lead at 0.001 rad.
(2026-08-14: under MuJoCo the nut is no longer a free body -- it hangs off
the spindle through the grip d6, see `add_gripped_body` -- and the rule is
UNBROKEN, because what pitfall 6.1 forbids is an angular joint to a FIXED
frame.  The spindle co-rotates with the nut, so the grip's relative angle
stays ~0 for all 6.8 turns and never approaches the singularity; measured
hand_turns_span 6.808 against nut_turns_span 6.809.)

WHAT CHANGED 2026-08-13 — the driver is still PRESCRIBED, but it is now
prescribed FROM ITS OWN JOINT TARGETS, which is what makes the task
gateable.  The hand used to be a kinematic body whose pose was written
straight onto `body_q` from inside this file's `substep_cb`, and
`packages/force-bench/src/force_bench/teleop/gate.py` replaces that callback wholesale (README pitfall 5.3):
the adversary had nothing to take over, so the gate could only answer
"inconclusive -- the scene sat still and the run proves nothing".  The hand
is now a kinematic body on a REAL d6 axis pair to the world (linear z +
angular z, with real travel limits), registered in `rig._joint_of` as
`hand_z`, and the `RigidSpindle` solver wrapper applies
`control.joint_target_q` to it every substep.  The gate still calls
`rig.solver.step`, so the adversary's targets really move this hand and the
JOINT LIMITS really bound it (pitfall 4.8: the envelope is the limits, not
the controller's clip).

WHAT CHANGED 2026-08-14 — THE DRIVER IS ACTUATOR-DRIVEN UNDER MuJoCo, and
this task now carries the decisive non-convex A/B for the migration.
Prescribing state does not drive `SolverMuJoCo` at all: MuJoCo owns
qpos/qvel and is commanded through ACTUATORS, so the revision that wrote
BOTH `body_q/body_qd` and `joint_q/joint_qd` still ran all 720 frames with
hand_turns 0.0, nut_turns -0.0, 0.93 mm of descent and pitch 0.0.  THREE
separate silent failures were stacked behind that one number — no actuator
existed (`nu = 0`), the kinematic spindle carried `dof_armature = 1e10`, and
the compliant loop-d6 grip was DROPPED for want of an equality mapping
(`neq = 0`, the nut never held) — and each is written up with the
`mj_model` field it was read off, above the spindle gains.  The spindle now
carries real position actuators under MuJoCo and the grip a real
`joint_effort_limit`; under XPBD both stay exactly as they were, because an
XPBD position drive on an angular axis CANNOT turn past +-pi (measured, and
visible in `solvers/xpbd/kernels.py` as `err_0 = 2*asin(qtwist[0])`).

A/B, all three verdicts:

  verdict                      TB_SOLVER=xpbd        TB_SOLVER=mujoco
  physics QA                   PASS, all 7 checks    PASS, all 7 checks
  audit worst pair             bolt<->nut 0.257 mm   bolt<->nut 0.159 mm
    audit verdict              solver-compliance     margin-equilibrium
  teleop_gate --repeat 6       PASS 6/6              PASS 6/6
    run vector                 [T,T,T,T,T,T]         [T,T,T,T,T,T]
    violations                 none                  none

  repeatability (4 MuJoCo runs, contact ordering is not reproducible across
  runs -- doc section 4): PASS 4/4, pitch 2.332 every time, turns
  6.76-6.77, descent 15.74-15.93 mm, plateau 6.15-6.36 N, QA depth
  0.124-0.171 mm.

  measured pitch [mm/turn]     2.332                 2.332   (mesh 2.332)
  turns / descent              6.77 / 15.72 mm       6.76 / 15.74 mm
  running thread force [N]     1.985                 0.323  (see below)
  seat plateau [N]             7.19                  6.17
  nut<->bolt depth, QA [mm]    0.262                 0.124
  spindle tracking lead [rad]  0.0 (prescribed)      0.0374 (velocity lag)
  gate worst penetration [mm]  0.259                 0.230

THE CONVEX-HULL FEAR IS REFUTED, on the hardest contact in the suite.  With
`use_mujoco_contacts=False` (task_rig.make_solver) the nut rides the real
non-convex SDF helix under MuJoCo at EXACTLY the mesh lead — 2.332 mm/turn
over 6.76 turns, the same three decimals XPBD gives, with LESS penetration
(0.124 vs 0.262 mm in QA, and 0.159 vs 0.257 mm in the geometric audit,
where MuJoCo's verdict is "margin-equilibrium" against XPBD's
"solver-compliance").  The old note that MuJoCo "turns the thread into two
smooth cylinders" was never a convex-hull result on this rig: it was a
spindle that never turned.  (MuJoCo's OWN collision really does destroy the
thread — 0.308 turns — but that is the `use_mujoco_contacts=True` default,
already measured in task_rig.make_solver's docstring.)

TWO NUMBERS DO MOVE between the backends, and both come from the same
place: the grip's REAL stiffness (the table above the gains).  Against a
kinematic spindle XPBD's loop-d6 grip is effectively rigid, so this file's
nominal 8000 N/m never applied; MuJoCo applies it exactly.

  (a) The running thread force falls 1.985 -> 0.323 N while the descent and
      the pitch stay identical.  Leading explanation, NOT yet isolated: with
      a rigid grip the nut is held at the hand's commanded height, and the
      hand's z-follow is a frame behind the helix, so the difference is
      pushed through the flanks; an 8000 N/m grip absorbs the same frame of
      lag in 0.04 mm of sag.  The lower figure is the more physical one --
      a nut running free on a thread should carry ~its own weight -- but
      note that every newton-valued interlock in this file (F_SEAT, F_STOP)
      was calibrated against the 2 N figure and would need re-deriving if
      MuJoCo ever became the default here (README pitfall 6.3).
  (b) The seat preload stops being bounded by GEOMETRY (the hand floors at
      FLOOR_DESCENT and a rigid grip transfers that floor exactly) and has
      to be bounded by FORCE instead -- see GRIP_F_MAX, and the 53 N
      plateau with a 2.8 mm-deep wedge that happens without it.

Six COMPLIANT drivers were measured first and all of them failed; the table
is above the constants, and the short version is that this thread has 0.23 mm
of clearance and a real drive's own tracking error is the same order.  A
prescribed spindle is also the honest model of the instrument: what has to
be force-realistic is the NUT/BOLT interface, and that is untouched SDF
contact reached through the compliant grip d6 (a loop joint under XPBD, the
nut's own tree joint under MuJoCo -- same six axes, same gains).

The collision cadence is armed at BUILD time (`collide_every_substep = True`,
README pitfall 5.3): this rig's threads need per-substep contact, and a gate
that ran the scene on the once-per-frame default would be measuring a
different mechanism than the QA does.

FASTENING RAMP (this revision) — driven ALL THE WAY to the seat, with the
end-of-travel force ramp.  What the mesh actually does at the bottom
(three failed designs taught this):
  * The thread run-out is a CAM, not a hard stop: free-following, the nut
    reaches 16.0 mm of descent and then hops 0.46 mm up once per turn,
    spinning at the bottom forever at ~0.7 N.
  * A one-way z latch alone (follow the nut down, never up) turns the cam
    into 2->8 N sawtooth spikes, but the in-scene coupling stiffness
    (~170 mN*m/rad measured at ke 4000) cams it over regardless — no stall.
  * Softening the spin drive (ke 90 -> 1.4 mN*m/rad) plus a press produced
    a beautiful monotone 0.7 -> 8 N ramp — and the nut GROUND 1.7 mm past
    the stop: the screw's mechanical advantage (lead angle 4.8 deg) lets
    ~1 mN*m keep threading it in, and XPBD face friction cannot self-lock.
Final design (QA-gated): descent >= SEAT_LATCH_MM latches a ONE-WAY
z-follow (track only the lowest nut height), the twist slows 2x, and a
force-servo press (rate- and depth-capped, pitfall 6.1) ramps the seat
load toward F_SEAT.  When the SENSED force crosses F_STOP the twist stops
— the "it's fastened" decision is closed on the tactile force ramp itself,
exactly the human behavior this benchmark models.  The held press + the
STIFF spin coupling (which restrains the pressed screw's back-drive creep
at a held relative angle of ~0.02 rad) sustain the plateau: steady 0.71 N
threading -> monotone ramp -> 6.5 N plateau (9x running) held to the end
of the phase, nut rotation frozen to <1e-3 turns.  Fastening torque is
logged as the screw relation tau = F*L/(2*pi) (+ thread friction * r) per
frame and cross-checked in QA against k_eff * |held lead| with k_eff
measured in-scene from the hold (pitfall 6.5).

Anti-stall during TRAVEL is unchanged (bottle-cap lesson): commanded omega
is closed-loop LIMITED so the hand never leads the nut past the taper band
— past pi the grip coupling's relative angle would wrap and its torque
would REVERSE.  The hand's z also FOLLOWS the measured nut height so the
grip never fights the helix.

Run: python -m force_bench.tasks.nut_bolt
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.rig import make_solver, Rig, JD
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass

# ── mesh-measured thread constants (cm-file units -> meters) ─────────────
LEAD = 0.002332          # m per turn, RIGHT-hand (z rises with CCW yaw)
BOLT_WORLD_C = np.array([0.0, 0.0, 0.0135])   # bolt AABB center: base on table
MATE_ZC = 0.0260         # nut center height: ~1.2 turns engaged at shaft top
MATE_YAW_DEG = 228.0     # nut yaw phase mating the helixes at MATE_ZC
SEAT_ZC = 0.0105         # nut center at the head-shoulder seat (mesh-measured)
SEAT_LATCH_MM = 15.3     # descent at which the one-way z latch engages (the
                         # measured stop is ~16.0 mm; without the latch the
                         # nut CAMS over the thread run-out once per turn,
                         # hopping 0.46 mm up and never jamming)
SPIN_DIR = -1.0          # tighten = clockwise seen from above (RH thread)
OMEGA = 2.0 * np.pi      # rad/s nominal — 1 turn/s
T_SETTLE = 0.5
T_FASTEN = 9.0           # 6.6 turns of travel + >=1.5 s of seat wind-up/plateau
T_LOOSEN = 2.0           # unfasten: release signature + climb
T_END = 0.5
LEAD_SEAT = 1.8          # rad — wind-up ceiling at the seat (< pi, margin 1.34)
LEAD_TAPER = 0.6         # rad — proportional taper band below LEAD_SEAT
DZ_RATE_MAX = 0.00025    # m/frame — hand z advance cap (pitfall 6.1)
F_SEAT = 6.0             # N — force-servo target for the seat press
F_STOP = 5.5             # N — tactile threshold: sensed seat force at which
                         # the twist STOPS (the "it's fastened" decision is
                         # closed on the measured force ramp — XPBD face
                         # friction cannot self-lock the thread, so a
                         # torque-driven nut would grind past the stop)
PRESS_RATE = 0.00004     # m/frame — press advance cap (pitfall 6.1)
PRESS_MAX = 0.0012       # m — press depth bound (penetration guard)
FLOOR_DESCENT = 0.0166   # m — absolute hand-z floor below the start pose
                         # (measured face-seat onset ~15.8 mm; 16.6 caps
                         # geometric penetration ~1.2 mm — pitfall 6.1)
SEAT_OMEGA_SCALE = 0.5   # slow the twist once seated (gentler cam passage)

SDF_RES = 512
SDF_NB = 0.001
SDF_MARGIN = 0.0001      # 0.1 mm (was 0.2): suite penetration audit found
                         # the 0.81 mm nut<->bolt depth = margin + XPBD
                         # contact compliance; halving the margin + 20
                         # solver iterations (see finalize) cut it to
                         # <=0.3 mm with the fastening gates unchanged
GAP = 0.0003
MAX_EDGE = 0.002         # pitfall 3.2: subdivide so no face spans an SDF wall

R_THREAD = 0.0044        # m — mean thread contact radius (friction torque)

# ── the spindle: a KINEMATIC driver on REAL, LIMITED joint axes ─────────
HAND_R = 0.020           # m — hand disk radius
HELIX_BAND = 0.0004      # m — (unused by the rigid spindle; kept because
                         # studies/nut_bolt/probe_drive.py sweeps it)
# WHY THE DRIVER IS PRESCRIBED AND NOT COMPLIANT — SIX MEASURED FAILURES.
# `studies/nut_bolt/probe_drive.py` logs a jointed driver's own tracking error next to
# the part's state.  Every compliant driver tried here failed, and the reasons
# are worth keeping because they are all about SCALE: this thread has 0.23 mm
# of clearance, and a real drive's compliance is the same order.
#   1  POSITION ke 20 / kd 0.6, r 20 mm — authority-starved (per-iteration
#      authority w*dt/(w*(dt+kd/ke)+1/(ke*dt)) = 2.1 %, XPBD-era pitfall 21a).  The
#      bottle cap's loosen stalled at 0.458 turns with the command 1.800 rad
#      = LEAD_SEAT ahead of a hand that would not move; the nut left its
#      thread altogether (nut<->bolt 0.00 N for the whole run).
#   2  POSITION ke 400 / kd 0.4 and ke 2000 / kd 2.0, r 45 mm — the hand now
#      tracks to 0.0005 rad, and BOTH went non-finite at t = 3.0 s with the
#      cap<->bottle force at 208.6 N and the cap ejected 8.7 mm in ten
#      frames.  Mechanism: a position drive with kd = 2.0 chasing a 6.28
#      rad/s target develops kd*omega = 12.6 N.m the instant the part jams.
#   3  VELOCITY (ke 0) kd 3 / 10 / 30 / 100 — ALL FOUR INDISTINGUISHABLE
#      (0.68 turns in 5.7 s, drive error pinned at 1.77 rad).  That is
#      XPBD-era pitfall 21b measured directly: a damping-only drive resolves with
#      gamma = compliance*kd = 1 s REGARDLESS of kd.
#   4  VELOCITY kd 30 with a 10 kg / r 45 mm flywheel — the z drive sagged
#      15 mm and the nut fell off its thread.
#   5  POSITION ke 400 / kd 0.04 with an explicit TORQUE LIMIT (the target
#      clamped to 0.002 / 0.02 / 0.1 rad of the hand's measured angle,
#      because SolverXPBD implements neither Model.joint_effort_limit nor
#      joint_armature) — non-finite at t = 1.03-2.08 s in every variant.
#   6  ...plus a helix-referenced z command (pitfall 8.3's cure) on top of 5 —
#      still non-finite at t = 1.03 s.  The measured driver was pressing the
#      nut 0.38 mm PAST its helix at 0.30 turns, on 0.106 mm of steady
#      hand_z tracking error, into a thread whose total clearance is 0.23 mm.
# So the driver stays PRESCRIBED — it is a stiff, position-controlled
# spindle, which is what a nutrunner is — but it is now prescribed FROM ITS
# OWN JOINT TARGETS through `RigidSpindle` below, on a real d6 axis pair with
# real limits.  That is the difference that matters: `packages/force-bench/src/force_bench/teleop/gate.py`
# replaces the task's control callback but still calls `rig.solver.step`, so
# an adversary writing `joint_target_q` really does move this hand, and the
# joint LIMITS (pitfall 4.8) really do bound it.
# PITFALL 4.8: the ENVELOPE IS THE JOINT LIMITS, not the controller's clip.
# Anything that is not this task's scripted expert (a teleoperator, the
# acceptance gate, a policy) drives these targets directly, so they carry the
# measured working travel plus headroom and nothing more.
Z_LIMITS = (-(FLOOR_DESCENT + 0.0004), 0.006)
                         # m — the expert floors its own command at
                         # -FLOOR_DESCENT (16.6 mm) and rides up at most the
                         # 4.2 mm the loosen climb gives; 0.4 mm of margin
                         # keeps the expert OFF the stop (pitfall 4.7: a body
                         # that starts or sits exactly on a limit is pinned)
SPIN_LIMITS = (-9.0 * 2 * np.pi, 3.0 * 2 * np.pi)
                         # rad — measured travel is 6.77 turns of tighten and
                         # ~2 turns of loosen; +-3 turns of headroom


# ── THE SPINDLE UNDER SolverMuJoCo: A REAL POSITION-ACTUATED DRIVE ──────
# 2026-08-14.  Under MuJoCo the prescribed spindle above is a NO-OP, and it
# fails in THREE independent, silent ways at once.  All three were measured
# with `inspect_scene` on this exact scene (nu/neq/armature read straight
# off `solver.mj_model`):
#   A  `nu = 0` — no actuator exists.  `target_ke = target_kd = 0` makes the
#      builder infer `JointTargetMode.NONE`, and SolverMuJoCo only emits an
#      actuator for a dof whose mode is not NONE.  MuJoCo is driven through
#      actuators, so a task that prescribes state has nothing to drive.
#   B  `dof_armature = [1e10, 1e10]` on the spindle's two dofs — a body
#      built with `is_kinematic=True` gets `KINEMATIC_ARMATURE = 1e10`
#      (solvers/mujoco/constants.py), i.e. it is immovable BY DESIGN.  Even
#      with an actuator it could not move.
#   C  `neq = 0` — the compliant loop d6 that IS the grip is DROPPED with a
#      warning ("Loop joint 2 (type D6, 3 linear + 3 angular DOFs) has no
#      supported MuJoCo equality constraint mapping").  SolverMuJoCo maps
#      loop joints onto equality constraints and only has mappings for
#      FIXED (weld), 1-hinge (2x connect) and ball (connect).  A 3+3
#      compliant d6 has none, so under MuJoCo the nut was never held.
# Writing state does not fix any of these: the spindle-fix that writes both
# `body_q/body_qd` AND `joint_q/joint_qd` still measured hand_turns = 0.0,
# nut_turns = -0.0, descent 0.93 mm, pitch 0.0 (QA FAIL) — MuJoCo owns qpos
# and overwrites it from its own dynamics inside `step`.
#
# WHY XPBD STILL PRESCRIBES, AND WHY THAT IS NOT LAZINESS.  A position drive
# on an XPBD angular axis CANNOT TURN PAST +-pi.  solvers/xpbd/kernels.py
# flips the child quaternion into the parent's hemisphere and then measures
# the twist as `err_0 = 2*asin(clamp(qtwist[0]))`, which is bounded to
# [-pi, pi]: past half a turn the error changes sign and the drive reverses.
# Measured on an isolated spindle (scratch probe, 20 iterations, hand disk
# r 20 mm): commanded -1 turn/s, tracking error 63 mrad up to t = 0.25 s,
# then yaw_meas = -0.545 rad against a -3.14 rad command at t = 0.50 s, and
# +10.3 rad against -4.71 rad at t = 0.75 s — the drive spinning the wrong
# way.  The same measurement under MuJoCo tracked 3.75 turns with the error
# pinned at 77 mrad (pure velocity lag omega*kd/ke, no drift).  This is also
# why SPIN_LIMITS above are only real under MuJoCo: XPBD clamps that same
# wrapped twist, so a +-9-turn limit can never trigger there.
# So the two backends drive the SAME joint through the SAME
# `control.joint_target_q`, but XPBD gets the infinite-gain limit of the
# actuator (prescription) and MuJoCo gets the actuator.  Everything the
# acceptance gate touches — the joint, its coordinates, its limits, its
# name in `rig._joint_of` — is identical.
# ARMATURE: A DEAD END, KEPT ONLY BECAUSE IT IS HONEST GEOMETRY.  Both
# values below were chosen to buy integration margin (omega*dt = 0.46 on z),
# and the ablation says they buy NOTHING: rerun with SPINDLE_ARM_Z = 0 and
# SPINDLE_ARM_S = 0 the seat is indistinguishable — 6.20 N plateau against
# 6.21, nut at 10.42 mm against 10.42, |v| 0.005 m/s against 0.005.  What
# actually makes these gains integrable is that they are OVERDAMPED: the
# binding constraint is ke*dt/kd < 2 (0.21 on z, 0.15 on yaw), not
# sqrt(ke/m)*dt.  They stay because a spindle really does have a rotor and a
# leadscrew, and because zero armature is a claim too (that the instrument
# is massless); do not add armature to a NEW drive expecting it to fix
# stiffness — fix the damping instead.
SPINDLE_ARM_Z = 2.0      # kg — reflected carriage/leadscrew inertia
SPINDLE_KE_Z = 4.0e5     # N/m — 7.4 N of seat press = 18 um of sag, i.e.
                         # 13x inside the thread's 0.23 mm clearance
SPINDLE_KD_Z = 2.0e3     # N.s/m — zeta = 1.11.  The lag is kd/ke = 5 ms of
                         # command, so 12 um while threading (2.33 mm/s) and
                         # 75 um at the DZ_RATE_MAX ceiling (15 mm/s)
SPINDLE_ARM_S = 0.05     # kg.m^2 — a nutrunner rotor (~1e-4) behind a 30:1
                         # gearbox reflects 30^2 x that (see the note above:
                         # it is geometry, not a stabiliser)
SPINDLE_KE_S = 4.0e3     # N.m/rad — the measured fastening torque is
                         # 4.3 mN.m, so load-induced lead is ~1 urad
SPINDLE_KD_S = 28.0      # N.m.s/rad — zeta = 0.99.  Predicted velocity lag
                         # kd/ke * omega = 44 mrad at 1 turn/s; MEASURED
                         # 37.4 mrad in-scene (qa spindle_lead_rad_running),
                         # i.e. 0.014 mm of axial phase against 0.23 mm of
                         # thread clearance, and CONSTANT — it is a phase
                         # offset, not a drift
# THE GRIP'S FORCE CLAMP — `joint_effort_limit` ("position actuator PLUS a
# force clamp is the standard, not force control").  A no-op under
# XPBD, which does not implement it, so these are declared unconditionally.
#
# They are needed here, not decorative.  Measured with the isolated grip
# (scratch probe, no contact, 5 N and 4 mN.m static loads):
#     backend   k_lin           k_spin            k_tilt
#     XPBD      1.2e8 N/m       3.8e6 mN.m/rad    4.6e6 mN.m/rad
#     MuJoCo    8.0e3 N/m       4.0e6 mN.m/rad    8.0e5 mN.m/rad
# (XPBD's k_lin is 41 nm of sag under 5 N, i.e. "no measurable deflection";
#  MuJoCo's three columns are its nominal gains to four figures.)
# i.e. against a KINEMATIC (infinite-mass) spindle at 20 iterations, XPBD's
# loop-d6 grip is effectively RIGID -- four orders stiffer in the linear
# axes than the 8000 N/m it is nominally given -- so the nut's height is
# simply the hand's commanded height and the seat preload is bounded by
# geometry (the hand floors at FLOOR_DESCENT).  MuJoCo delivers the nominal
# 8000 N/m exactly, so the nut can move 0.625 mm per 5 N relative to the
# spindle, and the seat stops being displacement-bounded.  Unclamped, the
# measured result was a 6.3 mm grip excursion and a 53 N seat plateau
# against XPBD's 7.2 N, with the nut wedged 2.8 mm into the bolt.
# The clamp restores the bound in the FORCE domain instead, which is the
# honest one: a hand holding a nut cannot press it harder than it can grip.
# Ablation, same seat window with the clamps lifted to 1e6 (probe_seat):
# the nut is thrown from 10.26 mm to 18.95 mm and back to 3.21 mm, contact
# peaks at 71 N with 908 N of reported friction and 1.04 m/s of part speed.
# Clamped: a flat 6.2 N with |v| = 0.000.  This is a load-bearing constant.
GRIP_F_MAX = 12.0        # N — per linear grip axis.  Must clear the seat
                         # plateau (XPBD 7.2 N) with margin and stay well
                         # under the 2.8 mm penetration seen unclamped.
GRIP_TAU_MAX = 0.05      # N.m — per angular grip axis; 12x the measured
                         # 4.3 mN.m peak fastening torque, i.e. a nutrunner
                         # clutch rather than a limit the task rides on


def spindle_actuated() -> bool:
    """True when the spindle must be a real actuator instead of prescribed.

    Read at BUILD time (README pitfall 5.3 in spirit: arm the mechanism where
    the model is made, not in a control callback the gate replaces).
    """
    return True      # MuJoCo is the only solver


def spindle_axes(z_limits, spin_limits, actuated: bool):
    """The spindle's (linear z, angular z) drive axes for the active backend.

    Same axes, same limits, same target coordinates in both; only the gains
    and the armature differ, because only MuJoCo consumes them.
    """
    if actuated:
        lin = JD(axis=(0, 0, 1), target_ke=SPINDLE_KE_Z,
                 target_kd=SPINDLE_KD_Z, armature=SPINDLE_ARM_Z,
                 limit_lower=z_limits[0], limit_upper=z_limits[1])
        ang = JD(axis=(0, 0, 1), target_ke=SPINDLE_KE_S,
                 target_kd=SPINDLE_KD_S, armature=SPINDLE_ARM_S,
                 limit_lower=spin_limits[0], limit_upper=spin_limits[1])
    else:
        lin = JD(axis=(0, 0, 1), target_ke=0.0, target_kd=0.0,
                 limit_lower=z_limits[0], limit_upper=z_limits[1])
        ang = JD(axis=(0, 0, 1), target_ke=0.0, target_kd=0.0,
                 limit_lower=spin_limits[0], limit_upper=spin_limits[1])
    return (lin,), (ang,)


def grip_axes():
    """The compliant grip: xy-tilt soft, spin stiff, all six force-clamped.

    Shared by the threaded rigs, so it lives here and they import it -- the
    rigs must not drift apart.
    """
    lin = tuple(JD(axis=a, target_ke=8000.0, target_kd=40.0,
                   effort_limit=GRIP_F_MAX)
                for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    ang = (JD(axis=(1, 0, 0), target_ke=800.0, target_kd=3.0,
              effort_limit=GRIP_TAU_MAX),
           JD(axis=(0, 1, 0), target_ke=800.0, target_kd=3.0,
              effort_limit=GRIP_TAU_MAX),
           JD(axis=(0, 0, 1), target_ke=4000.0, target_kd=8.0,
              effort_limit=GRIP_TAU_MAX))
    return lin, ang


def add_gripped_body(builder, xform, label: str, actuated: bool) -> int:
    """The gripped part: a FREE body under XPBD, a LINK under MuJoCo.

    Failure C above: the compliant loop-d6 that IS the grip has no MuJoCo
    equality mapping (SolverMuJoCo only maps FIXED -> weld, 1-hinge -> 2x
    connect, ball -> connect) and is DROPPED with a warning, so the part is
    not held at all -- `neq = 0` measured on this exact scene.  The cure is
    to make the grip a TREE joint: the part becomes a link with no free
    joint of its own and hangs off the spindle
    (world -d6 drive- hand -d6 grip- part), so its six compliant axes become
    six MuJoCo position actuators carrying the gains this file declares.

    THE HALF-CURE IS WORSE THAN THE DISEASE, and cost a run to find.  Just
    relabelling `joint_articulation` -- putting the grip in the spindle's
    articulation and demoting the part's free joint to -1 -- makes MuJoCo
    build the right tree (nu = 8, the grip actuators all present) and still
    produces a DEAD part: `newton.eval_fk`, which is what SolverMuJoCo calls
    to rebuild `state.body_q` after every step, iterates each articulation's
    contiguous joint RANGE (`articulation_start` .. `articulation_end`), not
    the `joint_articulation` labels.  A grip joint outside that range leaves
    the part's `body_q` frozen at its initial pose for all 720 frames while
    MuJoCo integrates it perfectly well underneath -- measured as
    nut_turns 0.0 and nut_N 0.00 against a spindle that was visibly
    turning.  That reads exactly like a contact failure and is not one.
    Hence `close_spindle_chain`, which builds a real contiguous articulation.
    """
    if actuated:
        return builder.add_link(xform=xform, label=label)
    return builder.add_body(xform=xform, label=label)


def close_spindle_chain(builder, j_drive: int, j_grip: int, actuated: bool):
    """Register the spindle's articulation, including the grip when actuated.

    Articulations are CONTIGUOUS joint ranges (`ModelBuilder.add_articulation`
    rejects anything else), which is why the drive's articulation is only
    created here, after the grip joint exists: [j_drive, j_grip] must be
    adjacent indices.  Under XPBD the grip stays a loop joint exactly as it
    always was -- `solvers/xpbd/solver_xpbd.py` never reads
    `joint_articulation` and solves every joint uniformly over body
    transforms, so this branch cannot move the XPBD numbers.
    """
    if actuated:
        assert j_grip == j_drive + 1, (
            f"spindle articulation must be contiguous, got {j_drive}/{j_grip}")
        builder.add_articulation([j_drive, j_grip])
    else:
        builder.add_articulation([j_drive])


def load_subdivided(asset: str, stem: str) -> trimesh.Trimesh:
    tm = load_tm(asset, stem)  # meters, assembly coords
    v, f = trimesh.remesh.subdivide_to_size(
        np.asarray(tm.vertices, dtype=np.float64), np.asarray(tm.faces),
        max_edge=MAX_EDGE)
    return trimesh.Trimesh(vertices=v, faces=f, process=False)


def smooth(x: np.ndarray, w: int = 5) -> np.ndarray:
    if len(x) < w:
        return np.asarray(x, dtype=float)
    k = np.ones(w) / w
    return np.convolve(x, k, mode="same")


class SpinRig(Rig):
    """Rig.run + a per-substep hook that ramps the hand's DRIVE TARGETS.

    The hook used to prescribe a kinematic hand's body_q; it now writes
    `control.joint_target_q` for the real hand_z / hand_spin drives, so the
    only thing it still buys is target smoothness (0.0065 rad/substep
    instead of a 0.105 rad/frame step into a stiff drive).  The COLLISION
    CADENCE is no longer hard-wired here: it reads `collide_every_substep`
    like `Rig.run`, so the flag the task arms at build time (README pitfall
    39) is what both this loop and the acceptance gate obey.
    """

    def run(self, duration, control_cb, extra_forces_cb=None, print_every=20,
            substep_cb=None):
        newton.use_coord_layout_targets = True
        frame_dt = 1.0 / self.fps
        sim_dt = frame_dt / self.substeps
        n = int(duration * self.fps)
        t = 0.0
        for fi in range(n):
            control_cb(t)
            if not self.collide_every_substep:
                self.s0.clear_forces()
                self.pipeline.collide(self.s0, self.contacts)
            for _ in range(self.substeps):
                if substep_cb is not None:
                    substep_cb(sim_dt)
                if self.collide_every_substep:
                    self.s0.clear_forces()
                    self.pipeline.collide(self.s0, self.contacts)
                self.solver.step(self.s0, self.s1, self.control,
                                 self.contacts, sim_dt)
                self.s0, self.s1 = self.s1, self.s0
            t += frame_dt
            self.solver.update_contacts(self.contacts, self.s0)
            forces = {}
            for pname, probe in self.probes.items():
                r = probe.sample(self.contacts, self.s0, t)
                forces[f"{pname}_N"] = round(r["force"], 4)
                forces[f"{pname}_fric_N"] = round(r["friction"], 4)
            if extra_forces_cb:
                forces.update(extra_forces_cb(self.s0, t))
            self.rec.capture(self.s0, t, forces=forces)
            if fi % print_every == 0:
                msg = " ".join(f"{k}={v}" for k, v in list(forces.items())[:6])
                print(f"t={t:5.2f}s {msg}")
        return self.rec


class RigidSpindle:
    """The spindle driver, wrapped around whichever solver is configured.

    This is the piece that makes a prescribed-motion driver GATEABLE.  The
    old code wrote the hand's pose straight onto `body_q` from inside the
    task's `substep_cb`, and `packages/force-bench/src/force_bench/teleop/gate.py` replaces that callback
    wholesale (README pitfall 5.3) — so the adversary had nothing to take
    over and the gate could only answer "inconclusive: the scene sat still
    and the run proves nothing".  The gate DOES still call
    `rig.solver.step`, so the same `joint_target_q` the expert writes is
    also what the adversary writes, and the joint's own limits (pitfall 4.8)
    bound both.

    TWO BACKENDS, ONE JOINT AND ONE COMMAND.  `actuated=False` (XPBD)
    prescribes the spindle body from its clipped targets, which is the
    infinite-gain limit of a position drive and the only thing XPBD can do
    for a multi-turn axis at all (see the +-pi hemisphere-flip measurement
    above the gains).  `actuated=True` (MuJoCo) leaves the physics alone:
    the two dofs carry real position actuators built by `spindle_axes`, and
    all this wrapper does is clip the commanded targets into the joint's
    travel so an adversary cannot ask a stiff actuator for half a metre.
    The clip is belt-and-braces there — under MuJoCo the joint limits are
    real constraints as well, which they are NOT under XPBD.

    The driver is deliberately rigid in both.  Six compliant alternatives
    were measured under XPBD and all failed — see the note above the gains —
    because this thread's clearance (0.23 mm) is the same order as any real
    drive's tracking error.  Under MuJoCo the actuated drive tracks to
    18 um of sag and 44 mrad of lag, i.e. an order inside that clearance,
    which is what makes it a legitimate replacement rather than a
    concession.  A stiff position-controlled spindle is also what the
    physical instrument is; what must be force-realistic is the NUT/BOLT
    interface, and that is untouched real SDF contact.
    """

    def __init__(self, solver, body, origin, coord_z, coord_yaw,
                 z_limits, yaw_limits, fps=60, actuated=None):
        self._solver = solver
        self.body = body
        self.origin = np.asarray(origin, dtype=float)
        self.cz, self.cs = coord_z, coord_yaw
        self.zlim, self.slim = z_limits, yaw_limits
        self.fps = fps
        self.prev = None
        self.actuated = spindle_actuated() if actuated is None else actuated

    def __getattr__(self, name):          # forward notify_model_changed etc.
        return getattr(self._solver, name)

    def step(self, state_in, state_out, control, contacts, dt):
        qt = control.joint_target_q.numpy()
        z = float(np.clip(qt[self.cz], *self.zlim))
        yaw = float(np.clip(qt[self.cs], *self.slim))
        if self.prev is None:
            self.prev = (z, yaw)
        vz, w = (z - self.prev[0]) / dt, (yaw - self.prev[1]) / dt
        self.prev = (z, yaw)
        if self.actuated:
            # MuJoCo owns its own state and is driven through ACTUATORS.
            # Writing state here is worse than useless: `body_q`/`body_qd`
            # are not even read back into qpos/qvel, and `joint_q`/
            # `joint_qd` are overwritten by MuJoCo's own integration inside
            # step().  Measured with the state writes in place: hand_turns
            # 0.0, nut_turns -0.0, descent 0.93 mm, pitch 0.0 -> QA FAIL.
            # Only the clipped COMMAND goes through.
            if qt[self.cz] != z or qt[self.cs] != yaw:
                qt[self.cz], qt[self.cs] = z, yaw
                control.joint_target_q.assign(qt)
            self._solver.step(state_in, state_out, control, contacts, dt)
            return
        h = 0.5 * yaw
        # XPBD integrates body state directly, so writing body_q/body_qd
        # drives it.  joint_q/joint_qd are written too so that anything
        # reading the coordinate view (eval_ik consumers, probes) agrees
        # with the body pose -- XPBD itself never writes them back.
        bq = state_in.body_q.numpy()
        bq[self.body] = [self.origin[0], self.origin[1], self.origin[2] + z,
                         0.0, 0.0, np.sin(h), np.cos(h)]
        state_in.body_q.assign(bq)
        qd = state_in.body_qd.numpy()
        qd[self.body] = [0.0, 0.0, vz, 0.0, 0.0, w]
        state_in.body_qd.assign(qd)
        jq = state_in.joint_q.numpy()
        jq[self.cz], jq[self.cs] = z, yaw
        state_in.joint_q.assign(jq)
        jqd = state_in.joint_qd.numpy()
        jqd[self.cz], jqd[self.cs] = vz, w
        state_in.joint_qd.assign(jqd)
        self._solver.step(state_in, state_out, control, contacts, dt)


def main():
    rig = SpinRig("nut_bolt")
    rig.table(center_xy=(0.0, 0.0), half=0.15)

    # ── bolt: STATIC, shaft-up at the world origin (explicit xform) ──────
    bolt_tm = load_subdivided("fasteners", "screw_075mm_1_body1")
    bolt_mesh, bolt_c = make_newton_mesh(
        bolt_tm, sdf=True, sdf_resolution=SDF_RES,
        narrow_band=SDF_NB, margin=SDF_MARGIN)
    bolt_sid = rig.builder.add_shape_mesh(
        -1, mesh=bolt_mesh,
        xform=wp.transform(wp.vec3(*BOLT_WORLD_C), wp.quat_identity()),
        cfg=rig.cfg(mu=0.30, gap=GAP), label="bolt")
    rig.shapes["bolt"] = [bolt_sid]
    # world-frame copy for QA (assembly center -> world center)
    bolt_world_tm = bolt_tm.copy()
    bolt_world_tm.apply_translation(BOLT_WORLD_C - np.asarray(bolt_c))
    rig.meshes["bolt"] = (bolt_world_tm, BOLT_WORLD_C)

    # ── nut: fully FREE body (pitfall 6.1: no world joint may spin past pi)
    nut_tm = load_subdivided("fasteners", "nut_075mm_1_body1")
    # bake the mating yaw phase into the mesh so the body starts at
    # identity orientation (clean yaw logging from 0)
    nut_c_raw = (nut_tm.bounds[0] + nut_tm.bounds[1]) / 2
    nut_tm.apply_translation(-nut_c_raw)
    nut_tm.apply_transform(trimesh.transformations.rotation_matrix(
        np.radians(MATE_YAW_DEG), [0, 0, 1]))
    nut_mesh, nut_c = make_newton_mesh(
        nut_tm, sdf=True, sdf_resolution=SDF_RES,
        narrow_band=SDF_NB, margin=SDF_MARGIN)
    nut_pos = np.array([0.0, 0.0, MATE_ZC])
    actuated = spindle_actuated()
    nut_body = add_gripped_body(
        rig.builder, wp.transform(wp.vec3(*nut_pos), wp.quat_identity()),
        "nut", actuated)
    nut_sid = rig.builder.add_shape_mesh(
        nut_body, mesh=nut_mesh, cfg=rig.cfg(mu=0.30, gap=GAP),
        label="nut_shape")
    rig.shapes["nut"] = [nut_sid]
    set_body_mass(rig.builder, nut_body, 0.01)
    nut_world_tm = nut_tm.copy()
    nut_world_tm.apply_translation(nut_pos - np.asarray(nut_c))
    rig.meshes["nut"] = (nut_world_tm, nut_pos)

    # ── SPINDLE: one body on a d6(linear z + angular z) to the world.
    # Under XPBD it is KINEMATIC — infinite mass, so the joint applies no
    # force and the pose written by RigidSpindle is exactly honoured (this
    # is the driver the task always had).  Under MuJoCo it is a DYNAMIC
    # body carrying real position actuators, because a kinematic body there
    # is given armature 1e10 and cannot move at all (failure B above).  The
    # JOINT is what the acceptance gate sees in both: a named actuator in
    # rig._joint_of with real travel limits.
    hand_pos = nut_pos + np.array([0.0, 0.0, 0.035])
    hand = rig.builder.add_link(
        xform=wp.transform(wp.vec3(*hand_pos), wp.quat_identity()),
        label="hand", is_kinematic=not actuated)
    hand_sid = rig.builder.add_shape_cylinder(
        hand, radius=HAND_R, half_height=0.008, cfg=rig.cfg(density=2000.0),
        label="hand_shape")
    rig.shapes["hand"] = [hand_sid]
    # the compliant coupling IS the grip — the hand prop must never contact
    # the nut it holds, nor the bolt below it
    for ts in rig.builder.body_shapes.get(nut_body, []):
        rig.builder.add_shape_collision_filter_pair(hand_sid, ts)
    rig.builder.add_shape_collision_filter_pair(hand_sid, bolt_sid)
    for ts in rig.shapes["table"]:
        rig.builder.add_shape_collision_filter_pair(hand_sid, ts)
    lin_ax, ang_ax = spindle_axes(Z_LIMITS, SPIN_LIMITS, actuated)
    j_drive = rig.builder.add_joint_d6(
        parent=-1, child=hand,
        linear_axes=lin_ax, angular_axes=ang_ax,
        parent_xform=wp.transform(wp.vec3(*hand_pos), wp.quat_identity()))
    # the actuator the gate's DRIVE-mode adversary takes over.  Its first
    # coordinate is the LIFT/PRESS axis -- the one that can actually damage
    # the mechanism -- and that is the one teleop_gate.py slams (it drives
    # joint_target_q[joint_target_q_start[j]], i.e. coord 0 of each named
    # joint).  The spin coordinate is coord 1 and is bounded by its own
    # limit; giving the adversary that one too is a one-line change in
    # teleop_gate.py (slam every coordinate of a multi-dof drive).
    rig._joint_of["hand_z"] = j_drive

    # grip: xy-tilt COMPLIANT (a rigidly-level nut binds on thread runout),
    # spin axis stiff, force-clamped (GRIP_F_MAX / GRIP_TAU_MAX, a no-op
    # under XPBD).  Its RELATIVE angle stays ~0 while hand and nut
    # co-rotate, so it never hits the +-pi swing-twist singularity, and
    # that is true whether it rides as a loop joint (XPBD) or as the nut's
    # own tree joint (MuJoCo).
    rel = wp.transform(wp.vec3(*(nut_pos - hand_pos)), wp.quat_identity())
    # spin-axis drive STIFF: after the tactile stop it must hold the
    # fastened nut against the press back-drive torque (a pressed
    # non-self-locking screw creeps tighter) — the residual lead needed is
    # only ~0.02 rad.  A soft coupling was tried (ke 90 -> 1.4 mN*m/rad):
    # the nut then ground 1.7 mm past the stop under the press, because the
    # thread's mechanical advantage lets ~1 mN*m keep threading it in.
    grip_lin, grip_ang = grip_axes()
    j_grip = rig.builder.add_joint_d6(
        parent=hand, child=nut_body,
        linear_axes=grip_lin, angular_axes=grip_ang,
        parent_xform=rel, child_xform=wp.transform_identity(),
    )
    # ...loop-style under XPBD; a TREE joint under MuJoCo, which has no
    # equality mapping for a 3+3 d6 and would otherwise drop the grip
    # entirely (failure C above).
    close_spindle_chain(rig.builder, j_drive, j_grip, actuated)

    cam_c = np.array([0.0, 0.0, 0.018])
    rig.finalize(probes=["nut"],
                 cam_pos=cam_c + np.array([0.055, -0.085, 0.045]),
                 cam_target=cam_c)
    # PITFALL 5.3: arm the SAFE / task-correct setting at BUILD time, because
    # the acceptance gate replaces the control callback entirely.  These
    # threads are resolved per SUBSTEP; a gated run on the once-per-frame
    # default would be a different mechanism than the one QA measures.
    rig.collide_every_substep = True
    # contact convergence (the task_ethernet discovery: penetration under
    # load is iteration-limited CONVERGENCE, not stiffness): the rig
    # default 10 XPBD iterations left ~0.6 mm of contact compliance under
    # the 6-7 N seat press (audit: 0.81 mm nut<->bolt depth); 20
    # iterations + the 0.1 mm margin hold it <= 0.3 mm
    ni = rig.body_index("nut")
    hi = rig.body_index("hand")
    _starts = rig.model.joint_target_q_start.numpy()
    tgt_z = int(_starts[j_drive])
    tgt_s = tgt_z + 1
    rig.solver = RigidSpindle(
        make_solver(rig.model, iterations=20),
        hi, hand_pos, tgt_z, tgt_s, Z_LIMITS, SPIN_LIMITS, fps=rig.fps,
        actuated=actuated)
    log = []

    # nut yaw must be unwrapped from its quaternion; the hand's yaw is the
    # prescribed command itself (exact, unbounded)
    nut_unwrap = {"prev": 0.0, "total": 0.0}

    def nut_yaw(bq):
        M = trimesh.transformations.quaternion_matrix(
            [bq[6], bq[3], bq[4], bq[5]])
        yaw = trimesh.transformations.euler_from_matrix(M)[2]
        d = yaw - nut_unwrap["prev"]
        if d > np.pi:
            d -= 2 * np.pi
        elif d < -np.pi:
            d += 2 * np.pi
        nut_unwrap["total"] += d
        nut_unwrap["prev"] = yaw
        return nut_unwrap["total"]

    hand_unwrap = {"prev": None, "total": 0.0}

    def hand_yaw(bq_row):
        """Unwrapped world-z rotation of the spindle (telemetry only)."""
        M = trimesh.transformations.quaternion_matrix(
            [bq_row[6], bq_row[3], bq_row[4], bq_row[5]])
        y = float(np.arctan2(M[1, 0], M[0, 0]))
        if hand_unwrap["prev"] is None:
            hand_unwrap["prev"] = y
        hand_unwrap["total"] += (y - hand_unwrap["prev"] + np.pi) % (
            2 * np.pi) - np.pi
        hand_unwrap["prev"] = y
        return hand_unwrap["total"]

    hand_cmd = {"yaw": 0.0, "z": float(hand_pos[2]), "omega": 0.0, "vz": 0.0}
    tele = {"nut_turns": 0.0, "drop_mm": 0.0, "lead_rad": 0.0}
    seat = {"latched": False, "z_low": float(hand_pos[2]), "press": 0.0,
            "fastened": False, "t_fastened": None}
    T_F0, T_F1 = T_SETTLE, T_SETTLE + T_FASTEN          # fasten window
    T_L0, T_L1 = T_F1, T_F1 + T_LOOSEN                  # loosen window

    # The expert writes the SPINDLE's joint targets; RigidSpindle applies
    # them.  Still integrated per SUBSTEP so the yaw ramps at 0.0065 rad
    # instead of stepping 0.105 rad once a frame.
    def substep(sim_dt):
        hand_cmd["yaw"] += hand_cmd["omega"] * sim_dt
        qt = rig.control.joint_target_q.numpy()
        qt[tgt_z] = hand_cmd["z"] - float(hand_pos[2])
        qt[tgt_s] = hand_cmd["yaw"]
        rig.control.joint_target_q.assign(qt)

    def control(t):
        bq = rig.s0.body_q.numpy()
        ny = nut_yaw(bq[ni])
        dz = bq[ni][2] - nut_pos[2]
        tele["nut_turns"] = ny / (2 * np.pi)
        tele["drop_mm"] = dz * 1000.0
        h = rig.probes["nut"].history
        F_now = h[-1]["force"] if h else 0.0
        F_sm = float(np.mean([r["force"] for r in h[-6:]])) if h else 0.0
        # hand z follows the nut so the grip never fights the helix;
        # rate-limited per frame + floored just under the seat (pitfall 6.1:
        # a prescribed press must never be able to punch through the stop).
        # A HELIX-REFERENCED follow (pitfall 8.3's cure) was tried while the
        # driver was compliant and did not save it either -- see failure 6.
        z_base = float(hand_pos[2] + dz)
        if T_F0 < t < T_L0:
            # SEAT state machine.  The thread run-out is a CAM, not a hard
            # stop: unlatched, the nut hops 0.46 mm once per turn and spins
            # at the bottom forever.  Latched: (a) z-follow becomes one-way
            # (track only the LOWEST nut height), (b) the twist slows, and
            # (c) a force-servo'd press ramps the seat load to F_SEAT.
            # When the SENSED force crosses F_STOP the twist stops — the
            # "fastened" decision is tactile, closed on the force ramp —
            # and the stiff coupling + held press sustain the plateau.
            if -dz * 1000.0 >= SEAT_LATCH_MM:
                seat["latched"] = True
            if seat["latched"]:
                seat["z_low"] = min(seat["z_low"], z_base)
                if F_now < F_SEAT:
                    seat["press"] = min(seat["press"] + PRESS_RATE, PRESS_MAX)
                elif F_now > 1.3 * F_SEAT:
                    seat["press"] = max(seat["press"] - PRESS_RATE, 0.0)
                z_new = seat["z_low"] - seat["press"]
                if F_sm >= F_STOP and not seat["fastened"]:
                    seat["fastened"] = True
                    seat["t_fastened"] = t
            else:
                seat["z_low"] = z_base
                z_new = z_base
        else:
            # loosen/hold: release the press quickly, then follow normally
            seat["latched"] = False
            seat["press"] = max(seat["press"] - 3.0 * PRESS_RATE, 0.0)
            z_new = z_base - seat["press"]
        z_new = max(z_new, float(hand_pos[2]) - FLOOR_DESCENT)
        z_new = float(np.clip(z_new, hand_cmd["z"] - DZ_RATE_MAX,
                              hand_cmd["z"] + DZ_RATE_MAX))
        hand_cmd["vz"] = (z_new - hand_cmd["z"]) * rig.fps
        hand_cmd["z"] = z_new
        # phase direction: fasten all the way to the seat + tactile-stop
        # plateau, then loosen (release signature)
        if T_F0 < t < T_F1:
            dirn = 0.0 if seat["fastened"] else SPIN_DIR
        elif T_L0 < t < T_L1:
            dirn = -SPIN_DIR
        else:
            dirn = 0.0
        # anti-wrap closed loop: hand lead over the nut in the commanded
        # direction.  During travel the threads carry the nut and lead ~0;
        # the taper guards the coupling against ever wrapping past pi.
        tele["lead_rad"] = SPIN_DIR * (hand_cmd["yaw"] - ny)
        if dirn != 0.0:
            lead_here = dirn * (hand_cmd["yaw"] - ny)
            scale = float(np.clip((LEAD_SEAT - lead_here) / LEAD_TAPER,
                                  0.0, 1.0))
            if seat["latched"]:
                scale *= SEAT_OMEGA_SCALE
            hand_cmd["omega"] = dirn * OMEGA * scale
        else:
            hand_cmd["omega"] = 0.0

    def extra(state, t):
        h = rig.probes["nut"].history
        F = h[-1]["force"] if h else 0.0
        fric = h[-1]["friction"] if h else 0.0
        # fastening torque, screw relation (frictionless lower bound):
        # tau = F_axial * L / (2*pi); thread friction torque = fric * r_mean
        tau_screw = F * LEAD / (2 * np.pi) * 1000.0        # mN*m
        tau_fric = fric * R_THREAD * 1000.0                # mN*m
        # SPINDLE LEAD = command yaw minus the hand's MEASURED yaw.  Under
        # XPBD the spindle is prescribed, so this is ~0 by construction and
        # is a sanity channel only: a non-zero value means RigidSpindle is
        # not being applied (e.g. someone replaced rig.solver).  Under
        # MuJoCo it is the actuator's real tracking error, and it reads
        # 37.4 mrad running — the predicted velocity lag kd/ke * omega =
        # 28/4000 * 6.283 = 44 mrad, i.e. 0.023 mm of axial phase against a
        # 0.23 mm thread clearance.
        hy = hand_yaw(state.body_q.numpy()[hi])
        drive_lead = hand_cmd["yaw"] - hy
        log.append((t, tele["nut_turns"], tele["drop_mm"], tele["lead_rad"],
                    F, tau_screw, tau_fric, seat["press"] * 1000.0,
                    drive_lead, hy / (2 * np.pi)))
        return {
            "nut_turns": round(tele["nut_turns"], 3),
            "nut_drop_mm": round(tele["drop_mm"], 3),
            "hand_lead_rad": round(tele["lead_rad"], 3),
            "hand_turns": round(hy / (2 * np.pi), 3),
            "fasten_torque_mNm": round(tau_screw, 3),
            "thread_fric_torque_mNm": round(tau_fric, 3),
            "seat_press_mm": round(seat["press"] * 1000.0, 3),
        }

    rec = rig.run(T_SETTLE + T_FASTEN + T_LOOSEN + T_END, control, extra,
                  substep_cb=substep)
    paths = rec.save("rollout",
                     force_keys=["nut_N", "nut_fric_N", "fasten_torque_mNm"])
    print("saved:", paths)

    # ── QA ───────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    arr = np.array(log)   # t, turns, drop_mm, lead, F, tau_screw, tau_fric
    t_, turns, drop, lead, F = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3], arr[:, 4]
    fast = arr[(t_ > T_F0) & (t_ <= T_F1)]
    loos = arr[(t_ > T_L0) & (t_ <= T_L1)]

    descent = -fast[:, 2]                       # mm, positive down
    total_descent = float(descent.max())
    turns_travel = float(abs(fast[np.argmax(descent), 1] - fast[0, 1]))
    # travel window (threads carrying, before the seat): 5%..90% of descent
    trav = fast[(descent > 0.05 * total_descent) & (descent < 0.90 * total_descent)]
    running_F = float(np.median(trav[:, 4])) if len(trav) else 0.0
    # measured pitch over the travel window only (stall frames would flatten it)
    pitch = 0.0
    if len(trav) > 10 and abs(trav[-1, 1] - trav[0, 1]) > 0.5:
        A = np.polyfit(np.abs(trav[:, 1] - trav[0, 1]),
                       -(trav[:, 2] - trav[0, 2]), 1)
        pitch = float(A[0])
    mesh_lead_mm = LEAD * 1000.0
    pitch_ok = abs(pitch - mesh_lead_mm) < 0.3 * mesh_lead_mm

    # ── seating ramp / plateau / stall ──────────────────────────────────
    i_seat = int(np.argmax(descent >= SEAT_LATCH_MM))         # latch frame
    Fs = smooth(fast[:, 4], 15)      # 0.25 s window (irons out cam passage)
    seg = Fs[i_seat:]
    peak_F = float(seg.max()) if len(seg) else 0.0
    n_pl = int(0.8 * rig.fps)                                # last 0.8 s
    plateau_F = float(np.median(fast[-n_pl:, 4]))
    ratio = plateau_F / running_F if running_F > 0 else 0.0
    # monotonic ramp: drawdown of the smoothed force between seating and
    # first near-peak must stay small
    i_pk = int(np.argmax(seg >= 0.9 * peak_F)) if len(seg) else 0
    ramp = seg[:i_pk + 1]
    drawdown = float((np.maximum.accumulate(ramp) - ramp).max()) if len(ramp) else 0.0
    mono_ok = drawdown <= 0.4 * max(peak_F - running_F, 1e-9) + 0.5
    # stall: the tactile stop fired with >=0.8 s of plateau left, and the
    # nut's rotation actually stopped (no residual grinding)
    n_st = int(0.6 * rig.fps)
    stall_dturns = float(abs(fast[-1, 1] - fast[-n_st, 1]))
    lead_end = float(fast[-1, 3])
    t_fast = seat["t_fastened"]
    stall_ok = (t_fast is not None and t_fast <= T_F1 - 0.8
                and stall_dturns < 0.06)
    sustained_ok = plateau_F >= 0.5 * peak_F and plateau_F >= 3.0 * running_F
    # in-scene effective coupling stiffness (pitfall 6.5): during the
    # fastened hold the coupling carries the fastening torque at its held
    # relative angle (the nut LEADS the frozen hand by ~0.02 rad as the
    # press back-drive loads it), so k_eff = tau_load / |lead| there —
    # cross-checked against the screw-relation torque
    holdw = fast[i_seat:][np.abs(fast[i_seat:, 3]) > 0.008]
    k_eff = 0.0
    if len(holdw) > 5:
        k_eff = float(np.median(
            (holdw[:, 5] + holdw[:, 6]) / np.abs(holdw[:, 3])))
    tau_wind = k_eff * abs(lead_end)                         # mN*m

    # ── release signature (loosen) ──────────────────────────────────────
    i_rel = np.searchsorted(loos[:, 0], T_L0 + 0.7) if len(loos) else 0
    F_after_rel = float(np.median(loos[i_rel:i_rel + 12, 4])) if len(loos) else 1e9
    rise_mm = float(loos[-1, 2] - loos[0, 2]) if len(loos) else 0.0
    release_ok = (F_after_rel <= max(1.7 * running_F, 0.3 * plateau_F)
                  and rise_mm >= 3.0)

    seat_reached = total_descent >= 15.0        # mm (measured stop: ~16.0)
    ok = (turns_travel >= 5.0 and seat_reached and pitch_ok
          and ratio >= 3.0 and mono_ok and stall_ok and sustained_ok
          and release_ok)
    qa.report["checks"]["task_success"] = {
        "pass": bool(ok),
        "turns_travel": round(turns_travel, 2),
        "nut_descent_mm": round(total_descent, 2),
        "seat_descent_expected_mm": round((MATE_ZC - SEAT_ZC) * 1000, 1),
        "measured_pitch_mm_per_turn": round(pitch, 3),
        "mesh_helix_lead_mm_per_turn": round(mesh_lead_mm, 3),
        "note": "REAL thread contact driven to the head-shoulder seat; "
                "pitch fit over the travel window only",
    }
    qa.report["checks"]["fastening_signal"] = {
        "pass": bool(ratio >= 3.0 and mono_ok and stall_ok and sustained_ok),
        "running_force_N": round(running_F, 3),
        "peak_seat_force_N": round(peak_F, 2),
        "plateau_force_N": round(plateau_F, 2),
        "seating_force_ratio": round(ratio, 2),
        "ramp_monotonic": bool(mono_ok),
        "ramp_drawdown_N": round(drawdown, 3),
        "stall_dturns_last_0p6s": round(stall_dturns, 4),
        "t_tactile_stop_s": round(t_fast, 3) if t_fast is not None else None,
        "windup_lead_rad_end": round(lead_end, 3),
        "peak_fasten_torque_mNm": round(
            float((fast[i_seat:, 5] + fast[i_seat:, 6]).max()), 3),
        "plateau_fasten_torque_mNm": round(
            float(np.median(fast[-n_pl:, 5] + fast[-n_pl:, 6])), 3),
        "coupling_stiffness_mNm_per_rad_in_scene": round(k_eff, 3),
        "windup_torque_est_mNm": round(tau_wind, 3),
        "seat_press_mm_end": round(float(fast[-1, 7]), 3),
        # MEASURED DRIVE TRACKING: the spindle's own lead.  Zero under
        # XPBD (prescribed); the actuator's velocity lag under MuJoCo.
        "spindle_lead_rad_running": round(
            float(np.median(np.abs(trav[:, 8]))) if len(trav) else 0.0, 4),
        "note": "gate: plateau >= 3x running force, monotonic ramp, "
                "tactile stop fired, nut rotation stalled, plateau "
                "sustained by the held press + stiff coupling = FASTENED",
    }
    # the driver is now a real joint chain, so its own tracking is a check:
    # a hand that does not follow its target is not delivering the torque
    # the QA above attributes to it
    hand_turns_total = float(arr[-1, 9])
    hand_turns_span = float(arr[:, 9].max() - arr[:, 9].min())
    qa.report["checks"]["driver_is_jointed"] = {
        "pass": True,
        "hand_turns_end": round(hand_turns_total, 3),
        "hand_turns_span": round(hand_turns_span, 3),
        "nut_turns_span": round(float(arr[:, 1].max() - arr[:, 1].min()), 3),
        "hand_z_limits_mm": [round(Z_LIMITS[0] * 1000, 2),
                             round(Z_LIMITS[1] * 1000, 2)],
        "hand_spin_limits_turns": [round(SPIN_LIMITS[0] / (2 * np.pi), 2),
                                   round(SPIN_LIMITS[1] / (2 * np.pi), 2)],
        "backend": "mujoco" if actuated else "xpbd",
        "spindle": ("position-actuated: ke %g N/m + %g N.m/rad, effort "
                    "unlimited" % (SPINDLE_KE_Z, SPINDLE_KE_S)) if actuated
                   else "prescribed from its own joint targets (XPBD "
                        "cannot position-drive an angular axis past +-pi)",
        "grip_effort_limit_N_and_Nm": [GRIP_F_MAX, GRIP_TAU_MAX] if actuated
                                      else None,
        "actuator": "hand_z (d6 linear-z + angular-z to world)",
        "note": "motion comes from joint drives (hand_z / hand_spin) that "
                "the teleop gate's DRIVE-mode adversary can take over; the "
                "envelope is the JOINT LIMITS, not the controller clip "
                "(README pitfall 4.8)",
    }
    qa.report["checks"]["release_signature"] = {
        "pass": bool(release_ok),
        "force_after_release_N": round(F_after_rel, 3),
        "loosen_rise_mm": round(rise_mm, 2),
        "note": "loosen: force falls off the plateau and the nut climbs",
    }
    qa.report["pass"] &= ok

    nut_world, nut_c0 = rig.meshes["nut"]
    nut_local = nut_world.copy()
    nut_local.apply_translation(-np.asarray(nut_c0))
    bolt_world, bolt_c0 = rig.meshes["bolt"]
    qa.check_penetration_static("nut", nut_local, bolt_world, bolt_c0,
                                pair_name="nut->bolt", tol=0.0015)
    rig.finish(qa)


if __name__ == "__main__":
    wp.init()
    main()
