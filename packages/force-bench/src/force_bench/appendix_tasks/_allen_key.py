"""Allen-key screw tasks (appendix Table I #13 "Tighten screw with Allen key",
#14 "Loosen screw with Allen key"), bare FR3 gripper.  The key starts LYING
FLAT ON THE TABLE, is picked up, drives the screw, and is put back where it
was (user request); the appendix itself starts with the key in hand.

    tighten: "A screw is placed into the screw slot of an assembly block in
    the workspace, but the threads are unengaged.  Use the Allen key grasped
    in hand to tighten the screw."  Success: "The screw is tightened and
    remains stationary when a small force is applied in an attempt to rotate
    the screw.  The robot movement stops after terminating the tightening
    motion, moving away from, and breaking contact with the screw."
    loosen: "... is fully tightened.  Use the Allen key grasped in hand to
    loosen the screw."  Success: "The screw is loosened and can rotate
    freely when a small force is applied ..." (same ending).

ASSETS (finalized STEP kit, metres, STEP frame; see their provenance.json)
  assets/bodies_kit/allen_key/allen_key.stl  L-key, 5.0 x 5.0 mm SQUARE
      section (0.5 mm edge fillets: corner 3.33 mm from the axis), long leg
      along +x (x -2.5..65, axis z = 30.5, 0.5 mm end chamfer), short leg
      along z (tip z = 3, axis x = 0).  PLA, 2.73 g.
  assets/bodies_kit/screw_block/  screw_6_5 (15 mm round head z 20..27, flat
      top r 4..6.5 mm, a 5.50 x 5.50 mm square recess 4.0 mm deep with an
      entry chamfer; thread crests every 2.34 mm, right-handed; 2.51 g),
      block_top / block_bottom (80 x 30 x 20 mm, screw at STEP x = +25).
      The key's 5.0 mm end fits the 5.5 mm recess with 0.25 mm per side:
      +-7.7 deg of backlash for the filleted section (support function of
      the mesh section; a sharp square would give +-5.9 deg), and the key
      would have to push its corners 0.575 mm into the recess walls to cam
      out (at 45 deg).

WHY THE LONG LEG GOES IN THE SCREW, AND WHY THE HAND IS HORIZONTAL
------------------------------------------------------------------
Lying flat, BOTH legs are horizontal, and whichever leg drives the screw has
to end up vertical: every route from the table to the screw contains a 90 deg
turn about a horizontal axis.  A top-down parallel gripper cannot supply it
by itself, and the options were measured / worked through offline:
  * pivoting the key in a light pinch under gravity: the pads are flat on a
    flat 5 mm face, so the contact patch's friction torque (~1.4 x F x 3 mm)
    beats the key's 0.63 mN.m gravity torque unless F < ~0.05 N/pad --
    below what a position-driven finger can regulate; and both pivot axes
    available on the flat key either swing the short leg INTO a finger
    (pinch across the long leg) or leave the tip pointing UP (pinch across
    the short leg).  Rejected.
  * short leg in the screw (the human's high-torque "lever" mode) needs the
    pads on the 25 mm of short leg between the screw head and the long leg:
    a 17.5 mm pad plus clearances does not fit with a horizontal hand.
  * a key stand would avoid the reorientation, but the key would no longer
    start on the table.  Not needed.
  * CHOSEN: a top-down pinch across the LONG leg's 5 mm width while it lies
    flat (pads on the leg's inner/outer faces, 22 mm from the bend -- next
    to the 23.7 mm centre of mass, so gravity puts almost no moment on the
    pinch), then, rigidly held, a 90 deg tilt of the hand about its closing
    axis in free air: the long leg hangs vertical, end down, the short leg
    horizontal on top -- exactly how the person in the reference video
    holds the key for the fast spin-down -- and the hand is horizontal.
    The long leg's end goes into the recess and the hand ORBITS the screw
    axis to turn it.  FrankaIK scan (offline, elbow-up seed, base 0.45 m
    from the screw): the orbit is continuous from hand angle +81 deg down to
    -258 deg (>= 0.17 rad joint margin, tightest on joint 6 near -90 deg);
    the tighten strokes use [-87, +30] deg, the loosen strokes [-45, +58]
    deg (draft runs), and a kinematic dry run of the whole pick -> tilt
    (TCP 0.13 m) -> 7 strokes -> orbit to PSI_UNTILT -> untilt -> place
    path keeps >= 0.16 rad on every joint (untilting at the last stroke's
    -88 deg instead left 0.07 rad on joint 6 with the tilt at 0.17 m).
The paper's "lift Allen key -> align with screw -> turn Allen key" cycle is
the stroke loop: turn ~96 deg, dwell 0.3 s (the arm lags the orbit command
by ~8 deg), UNWIND the wind-up (orbit back until the drive torque measured
at the recess is gone), lift the end 3 mm clear of the head, orbit back to
the angle where the key's flats are congruent with the recess again (read
from the screw's measured angle), re-measure the in-hand pose, hold still
while the end loop re-centres the key over the recess and learns the arm's
vertical droop, re-insert, turn again.

THE SCREW MECHANISM (reduced model, all of it in ThreadModel; same model as
the screwdriver-from-table tasks on the same kit screw, _screwdriver_table)
---------------------------------------------------------------------------
world -prismatic z-> carrier -revolute (about -z)-> screw.  Both dofs are
free joints in the solver; the THREAD between them is a per-substep
generalised force pair written to Control.joint_f by one Warp kernel:
  * helix: E = 1/2 k (z + L q)^2, L = pitch / 2 pi = 0.372 mm/rad: the head
    sinks 2.34 mm per clockwise turn (q > 0 = clockwise from above =
    tightening for this right-hand thread);
  * thread friction: elasto-plastic stiction on q (bristle 2 N.m/rad)
    capped at TAU_RUN0 + MU_RUN x |axial thread load|: load-dependent
    running friction, MU_RUN (1.5 mN.m/N) >> L (0.37 mN.m/N) so a press can
    not back-drive the thread; plus SCREW_KD of viscous drag;
  * seat: q = 0 is the HARD upper stop of the revolute (Rig.hard_joint_
    limits; a soft MuJoCo limit overshoots 9-18 deg) and z = 0 the hard
    lower stop of the carrier (head on the block top);
  * seat preload: once seated, the stiction cap is raised by PRELOAD_FRAC x
    the drive torque MEASURED at the recess while seating (DriveContacts),
    no floor (an untorqued seat gets no preload); released once the screw
    is backed off LOOSE_FREE_DEG.  A loosen run starts seated with
    LOOSE_PRELOAD0 of breakaway;
  * probe: PROBE_TAU on q once the robot has gone, loosen-way then
    tighten-way.
"THREADS UNENGAGED": the kit's captive nut spans z 0.5..7.5 mm and the
seated shank tip is at z 3.5, so a screw resting on the nut stands 4.0 mm
higher = 1.71 turns (615 deg) short of the seat; the tighten run starts
there, head 4.0 mm proud of the block.  The helix engages from the first
degree (no free spin before the thread catches) -- a stated simplification.
The screw's rotary dof carries 2e-7 kg.m^2 of conditioning armature (the
2.5 g screw's own 5.2e-8 kg.m^2 would make the explicit bristle unstable).

EXPERT (one continuous rollout; privileged state for closed-loop control only)
  pick    settle -> descend (top-down, yaw 45 deg) -> close (target 1.5 mm
          inside the leg, 8 kN/m, 15 N clamp, ~9.5 N/pad; a slow admittance
          trim keeps the mean pad force in GRIP_BAND) -> lift 80 mm; the
          in-hand pose (TCP in the key frame) is MEASURED while it hangs
  carry   to above the screw (TCP 0.13 m), then TILT the hand 90 deg about
          its closing axis in free air (3 s): long leg vertical, end down
  insert  hover 10 mm over the MEASURED head (it stands 4 mm proud at the
          start), xy loop on the measured key end, align (static: xy loop +
          the arm's vertical droop learned on the end), descend 10 / 3
          (entry chamfer) / 6 / 2 mm/s to 0.1 mm above the floor
  strokes orbit the hand about the screw axis (60 deg/s, ramped) with a
          gentle press servo on the measured key->screw force (0.5 N; command
          bounded 0.4 mm below the measured floor, which follows the head
          down the thread) and an xy loop keeping the pinch point on the
          axis; dwell; unwind (<= 4 deg, until the drive torque at the
          recess is relieved, stopping if it reverses or the screw moves
          back); between strokes lift out / orbit back / re-measure the
          in-hand pose / align / re-insert.
          tighten: once the MEASURED screw angle is at the seat, creep on at
          12 deg/s until the drive torque measured at the recess reaches
          30 mN.m (or 25 deg), hold 0.5 s -> seat preload = 0.7 x that ->
          unwind (<= 20 deg: the in-hand twist and the recess give come out
          first).  loosen: until the screw is 250 deg out -> unwind.
  return  extract 15 mm, orbit the free key to PSI_UNTILT while rising,
          UNTILT in free air, carry back over the start pose, lower on the
          MEASURED key height to 0.3 mm above the table, open, retreat
          80 mm, stand still; then the screw probe.

THE PINCH, AND WHY THE GRIPPED STRETCH OF THE KEY IS A BOX
  The pads close across the long leg's 5 mm width while it lies flat, so
  they cover 3.5 of the leg's 5 mm side faces (fingertips 1.5 mm above the
  table) and the pad box's bottom EDGE rests on the face.  With the key's
  mesh there, newton's per-triangle GJK/MPR mesh-vs-box contacts came back
  with ~45 deg normals next to that edge (per-contact dump: force-weighted
  |n_z| 0.43-0.48 of 14-16 N of pad force) -- the squeeze pushed the key
  4-6 N into the table and, lifted, it crept out of the pinch at ~0.9 mm/s
  and fell out during the tilt, with the tips 1.7, 0.5 and 0.2 mm above the
  table alike.  The straight stretch of the long leg (x 9..52 mm) is
  therefore an ANALYTIC BOX collider (box-box: MPR + clipped face manifold
  -> 4 corner contacts per pad, face normals, no slip through the lift, the
  tilt and the strokes); only that stretch's 0.5 mm edge fillets are lost.
  The bend / short leg and the 13 mm drive end stay SDF meshes (the drive
  end at ~0.04 mm cells against the 0.25 mm fit) and the CAD mesh is drawn.
  IN-HAND TWIST AND SLIP: the drive torque rolls the square section between
  the two flat pads (the corners dig into the soft MuJoCo contacts and push
  the fingers apart) -- the simulated counterpart of the paper's "Allen key
  tilts in-hand" signal.  A square pinched across flats resists that roll
  with a normal-force couple of at most F x 5 mm, so the squeeze sets the
  torque the key can deliver: at ~6 N/pad (first draft of this round) the
  seat torque-up flattened at 15.8 mN.m after 15 deg of over-travel, at
  ~9.5 N/pad it reaches 30 mN.m after 2.6 deg (twist <= 5.6 deg).  While the
  pinch slips in roll, the light press creeps the key ~0.7 mm per stroke
  along its leg through the pads (~4 mm over the seven strokes, ~6 mm at
  6 N/pad); the expert re-measures the in-hand pose between strokes and the
  put-back uses the measured grasp, so the key still lands where it was
  picked up.  The box-box pinch also chatters: tens of single-frame pad
  force jumps > 1 N per rollout (reported in expert_info).  The pad<->key
  overlap is part of the penetration check (SAT depth of the pad box vs
  the leg box, every frame).

GELSIGHT READINESS: finger targets are the named constants F_TOUCH_DOF /
F_CLOSE_DOF / F_OPEN_DOF / TIP_CLEAR / KEY_XP (add finger_offset() to the
dof ones); the grip carries a force band (GRIP_BAND, admittance trim on the
pad PairProbes); the 28 mm GS case clears the short leg in both the flat
pick and the tilted pose; the key contact sits in the last 3.5 mm of the
fingertip, outside the gel's imaged window (as in the nut task) -- a
deeper grasp or GS_CASE_TIP_EXTEND would be needed for a non-empty image.

MEASURED (draft runs of this module, TB_RENDER_FAST=1 -- identical physics;
the publication runs' numbers are in outputs/appendix_tasks/<name>/
appendix_qa.json)
  tighten  head 4.00 mm proud, 615 deg to go; 7 strokes of ~93.5 deg, the
           head sinks 0.62 mm per stroke; seated, torqued to 30.0 mN.m at
           the recess -> 21.0 mN.m preload; unwound 11.3 deg before the
           extraction (0.9 mN.m left, screw moved 0.05 deg); probe +-5 mN.m
           with the robot gone: max 0.09 / 0.19 deg; key vs recess flats
           <= 11.9 deg; pads 9.5 N mean; key put back 0.5 mm from its start.
  loosen   seated with 20 mN.m breakaway; broken at 25.9 mN.m; 3 strokes
           back it out 257 deg (head up 1.67 mm, 1.67 mm expected from the
           helix); probe: 46 deg each way in 1 s; key put back 0.2 mm from
           its start.

Run:  python -m force_bench.appendix_tasks.tighten_screw_with_allen_key_from_table
      python -m force_bench.appendix_tasks.loosen_screw_with_allen_key_from_table
      (TB_OUT_ROOT=<dir> TB_RENDER_FAST=1 TB_MULTIVIEW=0 for iterations;
       AK_STOP=<phase> ends the rollout when that phase starts, QA skipped)
"""

from __future__ import annotations

import os

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv_processed as subdiv, world_T
from force_bench.paths import ASSETS_ROOT
from force_bench.recorder import RolloutRecorder
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.screwdriver import DriveContacts
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, check_released_and_clear, save_appendix_verdict,
    StopRun, CONTACT_FREE_N)
from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._usb_kit import crop_box

MM = 1e-3
KIT = ASSETS_ROOT / "bodies_kit"
R_DOWN = HP.R_of_q(down_grasp_quat(0.0))


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


# ── geometry (STEP frames, metres; measured off the meshes) ─────────────
KEY_HALF = 2.5 * MM            # half of the 5.0 mm square section
KEY_LEG_Z = 30.5 * MM          # long-leg axis height in the key frame K
KEY_END_X = 65.0 * MM          # long-leg end face
KEY_TIP_CROP_X = 52.0 * MM     # fine-SDF drive-end shape: x >= this
KEY_BOX_X0 = 9.0 * MM          # analytic leg box: x 9..52 (leg straight from 7.5)
KEY_DENSITY = 1240.0           # PLA -> 2.73 g
KEY_MU = 0.35
BACKLASH_DEG = 7.7             # key end in the recess, filleted section (header)
CAMOUT_MM = 0.575              # corner interference needed to cam out (45 deg)

SCREW_AXIS_STEP = np.array([24.9885, 0.0455]) * MM   # recess centre (section fit)
HEAD_TOP_Z = 27.0 * MM         # seated
RECESS_FLOOR_Z = 23.0 * MM     # seated
RECESS_HALF = 2.75 * MM
HEAD_R = 7.5 * MM
HEAD_CROP_Z = 19.0 * MM
SCREW_MU = 0.35

# ── cell (world, metres) ───────────────────────────────────────────────
Z0 = 0.0                       # table top
SCREW_XY = np.array([0.0, 0.0])
BLOCK_YAW = np.radians(45.0)   # seated recess flats at 45 deg
ROBOT_BASE = np.array([0.0, 0.45, Z0])
ROBOT_YAW = -np.pi / 2         # facing the screw
KEY_PINCH_XY = np.array([0.12, 0.12])   # where the long leg is pinched
KEY_YAW0 = np.radians(45.0)    # long leg (bend -> end) direction on the table
REST_CLEAR = 0.05 * MM
ARM_SEED = np.array([0.0, 0.3, 0.0, -2.2, 0.0, 2.4, 0.785])   # elbow-up IK branch

# ── fingers / grasp (GelSight phase: every *_DOF target += finger_offset())
FINGER_KE, FINGER_KD = HP.FINGER_KE, HP.FINGER_KD   # 8 kN/m, 60
FINGER_F_MAX = 15.0            # N per pad, actuator force range
PAD_FACE_OFF = HP.PAD_FACE_OFF
TIP_BELOW_TCP = HP.TIP_BELOW_TCP   # 9.5 mm
KEY_XP = 22.0 * MM             # pinch point along the long leg from the bend
TIP_CLEAR = _env("AK_TIP_CLEAR", 1.5 * MM)
# fingertips above the table at the pick (the suite's 1.5 mm).  The pads then
# cover 3.5 of the leg's 5 mm side faces; the gripped stretch is an analytic
# box (see build_scene for why a mesh there pushed the key out of the pinch).
TCP_H = TIP_BELOW_TCP + TIP_CLEAR - KEY_HALF   # TCP above the leg axis (8.5 mm)
SQUEEZE = _env("AK_SQUEEZE", 1.5 * MM)          # ~9-10 N/pad realised at 8 kN/m
# (1.0 mm / ~6 N in the first draft: over seven strokes the drive torque kept
# the pinch slipping in roll and the 1-3 N press crept the key 6.3 mm up
# through the pads, the pad overlap shrank and the pinch softened to 5.3 N; a
# square pinched across flats resists a roll torque with a normal-force couple
# of at most F x 5 mm, so 30 mN.m needs > 6 N/pad)
OPEN_EXTRA = 8.0 * MM
F_TOUCH_DOF = KEY_HALF + PAD_FACE_OFF
F_CLOSE_DOF = F_TOUCH_DOF - SQUEEZE
F_OPEN_DOF = F_TOUCH_DOF + OPEN_EXTRA
GRIP_BAND = (7.0, 14.0)        # N, mean pad force: the admittance trim only acts
GRIP_TRIM_STEP = 0.005 * MM    # outside it (m per frame) ...
GRIP_TRIM_MAX = 1.0 * MM       # ... within +-1 mm of F_CLOSE_DOF
IMPRATIO = 10.0                # small-part pinch (plug_in_usb / hang_* lesson)
# pad fingertip collision box in the finger frame (FR3 URDF shape 47 / 53)
PAD_BOX_C = np.array([0.0, 7.58, 45.25]) * MM
PAD_BOX_H = np.array([8.75, 7.6, 9.25]) * MM

# ── screw mechanism (ThreadModel) ──────────────────────────────────────
PITCH = 2.34 * MM              # m/turn (measured crest spacing, right hand)
LEAD = PITCH / (2 * np.pi)     # m/rad
UNENGAGED_RISE = _env("AK_RISE_MM", 4.0) * MM   # head above the seat, threads unengaged
K_HELIX, C_HELIX = 3.0e4, 15.0  # N/m, N.s/m (explicit: k dt^2/m 0.65, c dt/m 0.63)
TAU_RUN0 = 0.0015              # N.m running thread drag, unloaded
MU_RUN = 0.0015                # N.m per N of axial thread load
SCREW_KD = 0.004               # N.m.s/rad viscous part of the thread
SCREW_ARMATURE = 2.0e-7        # kg.m^2 conditioning inertia for the explicit bristle
CARRIER_MASS = 0.010
STICTION_K = 2.0               # N.m/rad bristle: 5 mN.m -> 0.14 deg
PRELOAD_FRAC, PRELOAD_MIN, PRELOAD_MAX = 0.7, 0.0, 0.040
LOOSE_PRELOAD0 = 0.020
LOOSE_FREE_DEG = 10.0
PROBE_TAU = 0.005
LOOSEN_Q0_DEG = -0.3           # seated (just off the stop, pitfall 4.7)
SEAT_TOL_DEG = 0.6
LOOSEN_TARGET_DEG = 250.0      # back it off this far (>= 180 counts as loosened)
LOOSENED_MIN_DEG = 180.0       # the screwdriver-from-table tasks' threshold
TIGHT_TURN_MIN_DEG = 600.0     # of the 615 deg from the unengaged start

# ── expert ──────────────────────────────────────────────────────────────
STROKE_DEG = 96.0              # hand travel per stroke (90 + the ~3-4 deg of play)
STROKE_RATE = np.radians(60.0) # rad/s of hand orbit while turning
PRESS_LIM = (-0.2 * MM, 0.4 * MM)   # commanded end depth below the floor (servo)
T_RAMP = 0.25                  # s to ramp the orbit up to STROKE_RATE
TAU_TIGHT = 0.030              # N.m: once seated, creep the hand on until the drive
                               # torque MEASURED at the recess reaches this ...
OT_MAX_DEG = 25.0              # ... or this much hand over-travel (in-hand twist +
                               # recess give + the arm's ~8 deg orbit lag)
OT_RATE = np.radians(12.0)     # rad/s of hand orbit while torquing up
T_DWELL = 0.3                  # s at the end of a stroke: the arm lags the orbit
                               # command by ~8 deg at 60 deg/s and catches up here
UNWIND_RATE = np.radians(10.0) # rad/s of hand orbit back after a stroke, until the
TAU_UNWOUND = 0.0010           # recess carries less than this in the stroke's sense
UNWIND_MAX_DEG = {"stroke": 4.0, "seat": 20.0}
# a run-down stroke unwinds at most 4 deg (inside the +-7.7 deg play) and stops
# as soon as the drive torque reverses or the screw moves back: rotating the key
# back against the wall it leans on drags the free screw back with it (first
# draft: 20 deg of unwind turned the screw back 8 deg -- wall friction ~1.9 mN.m
# vs 1.7 mN.m of thread friction); after the seat, the 0.7 x seating-torque
# preload holds the screw and the in-hand twist + recess give (~20 deg at
# 30 mN.m) must come out before the extraction
PRESS_UNWIND_N = 0.3           # press while unwinding
PSI_START = {"tighten": np.radians(30.0), "loosen": np.radians(-60.0)}
                               # stroke start = the congruent hand angle nearest
                               # this (tighten turns clockwise, loosen counter-cw)
PSI_UNTILT = {"tighten": np.radians(-30.0), "loosen": np.radians(0.0)}
                               # the free key is orbited here before the untilt
                               # (joint margin 0.43-0.48 rad there vs 0.14 at -90)
MAX_STROKES = 10
HOVER_DZ = 10.0 * MM           # key end above the head top at the hover
LIFTOUT_DZ = 3.0 * MM          # key end above the head top between strokes
EXTRACT_DZ = 15.0 * MM
TILT_Z = 0.13                  # TCP height for the tilt / untilt
PRESS_N = 0.5                  # key -> screw vertical force while turning (light: an
                               # axial load on a pinch that slips in roll creeps it)
V_INSERT_FAST, V_INSERT_SLOW, V_INSERT_IN, V_INSERT_END = 10e-3, 3e-3, 6e-3, 2e-3
# the last 1.5 mm at 2 mm/s and the press servo only from 3.5 mm deep: at
# 6 mm/s onto the floor the key hit 17.7 N, above the pinch's ~17 N axial
# friction, and slid 2 mm up in the hand
T = dict(settle=0.5, descend=1.0, pre=0.3, close=0.6, hold=0.3, lift=0.9, lifthold=0.3,
         carry=1.4, tilt=3.0, hover=1.2, align=0.5, liftout=0.4, back=1.6, unwind=2.5,
         seat_hold=0.5, extract=0.6, up=1.2, untilt=3.0, carryback=1.6, lower=1.4,
         open=0.5, retreat=1.0, still=1.0, probe=1.0)
T_STILL_SETTLE = 0.4           # the arm tracker finishes the retreat in here
PLACE_ABOVE = 0.025            # carry back to this far above the place pose
T_INSERT_MAX = 3.0
T_BUDGET = 140.0
# cameras.  Primary: the whole cell from the -x side (the hand orbits on the
# +x side of the screw, so it is always BEHIND the key); second view: a
# close-up of the head from (-x, -y), low, so the key end in the recess, the
# and the head's height are readable.  Both share one fov.
CAM_TGT = np.array([_env("AK_CAM_TX", 0.050), _env("AK_CAM_TY", 0.050), Z0 + _env("AK_CAM_TZ", 0.075)])
CAM_OFF = np.array([_env("AK_CAM_OX", -0.52), _env("AK_CAM_OY", -0.20), _env("AK_CAM_OZ", 0.30)])
CAM2_TGT = np.array([_env("AK_CAM2_TX", 0.0), _env("AK_CAM2_TY", 0.0), Z0 + _env("AK_CAM2_TZ", 0.036)])
CAM2_OFF = np.array([_env("AK_CAM2_OX", -0.085), _env("AK_CAM2_OY", -0.060), _env("AK_CAM2_OZ", 0.063)])
CAM_FOV = _env("AK_CAM_FOV", 40.0)
KIT_COLORS = {"block_top": (0.93, 0.80, 0.10), "block_bottom": (0.93, 0.80, 0.10),
              "screw_visual": (0.46, 0.47, 0.50),
              "key_visual": (0.12, 0.12, 0.13)}
ORDER = ["settle", "descend", "pre", "close", "hold", "lift", "lifthold", "carry", "tilt",
         "hover", "align", "insert", "turn", "unwind", "liftout", "back", "reinsert",
         "seat_hold", "extract", "up", "untilt", "carryback", "lower", "open", "retreat",
         "still", "probe"]
HELD = ("hold", "lift", "lifthold", "carry", "tilt", "hover", "align", "insert", "turn",
        "unwind", "liftout", "back", "reinsert", "seat_hold", "extract", "up", "untilt",
        "carryback", "lower")
T_PLACE_MAX = 4.0


# ── small rotation helpers ──────────────────────────────────────────────
def Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def Tm(R, p):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = p
    return M


def slerpR(Ra, Rb, u):
    qa, qb = HP.quat_of_R(Ra), HP.quat_of_R(Rb)
    if np.dot(qa, qb) < 0:
        qb = -qb
    r = wp.quat_slerp(wp.quat(*qa.tolist()), wp.quat(*qb.tolist()), float(u))
    return HP.R_of_q(np.array([r[0], r[1], r[2], r[3]]))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def wrap90(a):
    return (np.asarray(a, float) + 45.0) % 90.0 - 45.0


# The F1 grasp: TCP pose in the key frame K.  Approach (TCP z) = -K_y (down
# onto the flat key), closing (TCP y) = K_z (across the long leg's inner /
# outer faces), pad width (TCP x) = K_x (along the leg, toward its end).
G_NOM = Tm(np.column_stack([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]),
           np.array([KEY_XP, TCP_H, KEY_LEG_Z]))


def R_key_flat(phi):
    """Key lying flat (K_y up), long leg pointing along yaw phi."""
    return Rz(phi) @ Rx(np.pi / 2)


def R_key_drive(psi):
    """Long leg vertical, end down; the key's flats (and the hand) at psi:
    K_y -> (cos psi, sin psi, 0), the hand points along -K_y."""
    return Rz(psi) @ Ry(np.pi / 2) @ Rx(np.pi / 2)


def key_flat_pose(phi, pinch_xy):
    R = R_key_flat(phi)
    p = np.array([pinch_xy[0], pinch_xy[1], Z0 + KEY_HALF + REST_CLEAR]) \
        - R @ np.array([KEY_XP, 0.0, KEY_LEG_Z])
    return Tm(R, p)


def key_drive_pose(psi, end_z):
    R = R_key_drive(psi)
    p = np.array([SCREW_XY[0], SCREW_XY[1], end_z]) - R @ np.array([KEY_END_X, 0.0, KEY_LEG_Z])
    return Tm(R, p)


def flats_psi_deg(q_screw_rad):
    """World angle of the recess flats (mod 90) for screw angle q."""
    return np.degrees(BLOCK_YAW) - np.degrees(q_screw_rad)


def congruent_psi(q_screw_rad, near_deg):
    f = flats_psi_deg(q_screw_rad)
    return float(f + 90.0 * np.round((near_deg - f) / 90.0))


def q_unengaged():
    """Screw angle (rad, < 0) with the head UNENGAGED_RISE above the seat."""
    return -UNENGAGED_RISE / LEAD


# ── the screw's thread / seat / probe model ─────────────────────────────
@wp.kernel
def _ak_thread_kernel(joint_q: wp.array(dtype=wp.float32), joint_qd: wp.array(dtype=wp.float32),
                      joint_f: wp.array(dtype=wp.float32), st: wp.array(dtype=wp.float32),
                      cz: int, cq: int, dz: int, dq: int, k_h: float, c_h: float, lead: float,
                      tau0: float, mu_r: float, k_b: float):
    # st = [anchor, preload, probe, F_axial, tau_friction, |tau_friction| peak]
    z = joint_q[cz]
    q = joint_q[cq]
    e = z - lead * q
    ed = joint_qd[dz] - lead * joint_qd[dq]
    fs = k_h * e + c_h * ed
    cap = tau0 + mu_r * wp.abs(fs) + st[1]
    anchor = st[0]
    d = q - anchor
    if wp.abs(k_b * d) > cap:
        anchor = q - wp.sign(d) * cap / k_b
    tau_f = -k_b * (q - anchor)
    st[0] = anchor
    st[3] = fs
    st[4] = tau_f
    st[5] = wp.max(st[5], wp.abs(tau_f))
    joint_f[dz] = -fs
    joint_f[dq] = fs * lead + tau_f + st[2]


class ThreadModel:
    """Per-substep generalised forces on the screw's two dofs: helix spring
    z = -LEAD q (this module's q is about -z, so clockwise = +q = down the
    thread), load-dependent elasto-plastic thread friction, seat preload,
    external probe torque.  One Warp kernel per substep (no host round
    trip); the host reads the state once per frame.  Copied from
    _screwdriver_table.ThreadModel (same kit screw) with this module's sign
    convention and constants."""

    def __init__(self, solver, model, j_z, j_q, q0, preload0):
        self._solver = solver
        qs = model.joint_q_start.numpy()
        ds = model.joint_qd_start.numpy()
        self.cz, self.cq = int(qs[j_z]), int(qs[j_q])
        self.dz, self.dq = int(ds[j_z]), int(ds[j_q])
        self.st = wp.array(np.array([q0, preload0, 0.0, 0.0, 0.0, 0.0], np.float32),
                           dtype=wp.float32, device=model.device)
        self._probe = 0.0
        self.refresh()

    def refresh(self):
        a = self.st.numpy()
        self.preload, self.F_ax, self.tau_f, self.peak_fric = (float(a[1]), float(a[3]),
                                                               float(a[4]), float(a[5]))

    def _set(self, idx_vals):
        a = self.st.numpy()
        for i, v in idx_vals:
            a[i] = v
        self.st.assign(a)

    def set_preload(self, tau, q_now):
        self._set([(1, float(tau)), (0, float(q_now))])
        self.preload = float(tau)

    @property
    def probe(self):
        return self._probe

    @probe.setter
    def probe(self, v):
        if float(v) != self._probe:
            self._probe = float(v)
            self._set([(2, self._probe)])

    def step(self, state_in, state_out, control, contacts, dt):
        wp.launch(_ak_thread_kernel, dim=1,
                  inputs=[state_in.joint_q, state_in.joint_qd, control.joint_f, self.st,
                          self.cz, self.cq, self.dz, self.dq, K_HELIX, C_HELIX, -LEAD,
                          TAU_RUN0, MU_RUN, STICTION_K])
        return self._solver.step(state_in, state_out, control, contacts, dt)

    def __getattr__(self, name):
        return getattr(self._solver, name)


# ── scene ───────────────────────────────────────────────────────────────
def _load(rel):
    return trimesh.load(KIT / rel, force="mesh")


def step_to_world() -> np.ndarray:
    """The toy-block STEP assembly frame -> world: screw axis on SCREW_XY,
    block yawed by BLOCK_YAW, block bottom on the table."""
    a = np.array([SCREW_AXIS_STEP[0], SCREW_AXIS_STEP[1], 0.0])
    return Tm(Rz(BLOCK_YAW), np.array([SCREW_XY[0], SCREW_XY[1], Z0])) @ Tm(np.eye(3), -a)


def build_scene(rig, mode: str) -> dict:
    b = rig.builder
    rig.add_robot(wp.transform(wp.vec3(*ROBOT_BASE.tolist()),
                               wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), ROBOT_YAW)))
    for dof in range(7):
        b.joint_target_kd[dof] = HP.ARM_KD_TASK
    for dof in (7, 8):
        b.joint_target_ke[dof] = FINGER_KE
        b.joint_target_kd[dof] = FINGER_KD
    set_finger_effort_limit(b, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = b.body_label.index(bl)
        set_body_mass(b, bi, HP.FINGER_MASS)
        for sid in b.body_shapes.get(bi, []):
            b.shape_material_mu[sid] = HP.PAD_MU

    rig.table(center_xy=(0.03, 0.10), half=0.32)

    # ── block (static scenery) ──────────────────────────────────────────
    T_sw = step_to_world()
    for stem in ("block_top", "block_bottom"):
        tm = _load(f"screw_block/{stem}.stl")
        tm.apply_transform(T_sw)
        mesh, c = make_newton_mesh(tm, sdf=True, sdf_resolution=256, narrow_band=0.0015,
                                   margin=0.0008)
        sid = b.add_shape_mesh(-1, mesh=mesh, xform=wp.transform(wp.vec3(*np.asarray(c, float)),
                                                                 wp.quat_identity()),
                               cfg=rig.cfg(mu=0.5, gap=0.0005, mu_torsional=HP.MU_TORSION),
                               label=stem)
        rig.shapes[stem] = [sid]
        rig.meshes[stem] = (tm, np.asarray(c, float))

    # ── screw: world -prismatic z-> carrier -revolute (-z)-> screw ──────
    # the THREAD between the two dofs is ThreadModel (helix + friction);
    # q = 0 / z = 0 is the seat (both hard stops)
    scr_full = _load("screw_block/screw_6_5.stl")
    head = subdiv(crop_box(scr_full, (None, None, HEAD_CROP_Z), (None, None, None)), 0.0008)
    head_w = head.copy()
    head_w.apply_transform(T_sw)
    scr_mesh, scr_c = make_newton_mesh(head_w, sdf=True, sdf_resolution=384,
                                       narrow_band=0.0015, margin=0.0002)
    scr_c = np.asarray(scr_c, float)
    car = b.add_link(xform=wp.transform(wp.vec3(*scr_c), wp.quat_identity()),
                     label="screw_carrier")
    b.add_shape_box(car, hx=0.002, hy=0.002, hz=0.002,
                    cfg=rig.cfg(density=1000.0, has_shape_collision=False, is_visible=False),
                    label="screw_carrier_shape")
    set_body_mass(b, car, CARRIER_MASS)
    j_z = b.add_joint_prismatic(
        parent=-1, child=car, axis=(0, 0, 1),
        parent_xform=wp.transform(wp.vec3(*scr_c), wp.quat_identity()),
        target_ke=0.0, target_kd=0.0, limit_lower=-0.00005,
        limit_upper=UNENGAGED_RISE + 0.0006)
    # the mesh vertices were baked in WORLD orientation (T_sw applied), so the
    # screw body starts at identity rotation; the joint rotates it about -z
    scr_body = b.add_link(xform=wp.transform(wp.vec3(*scr_c), wp.quat_identity()),
                          label="screw")
    s_scr = b.add_shape_mesh(scr_body, mesh=scr_mesh,
                             cfg=rig.cfg(mu=SCREW_MU, gap=0.0005, density=KEY_DENSITY,
                                         mu_torsional=HP.MU_TORSION, is_visible=False),
                             label="screw_shape")
    rig.shapes["screw"] = [s_scr]
    rig.meshes["screw"] = (head_w, scr_c)
    # visual: the whole screw (the shank shows between head and block while
    # the threads are unengaged)
    scr_vis = scr_full.copy()
    scr_vis.apply_transform(T_sw)
    vis_m, vis_c = make_newton_mesh(scr_vis, sdf=False)
    b.add_shape_mesh(scr_body, mesh=vis_m,
                     xform=wp.transform(wp.vec3(*(np.asarray(vis_c, float) - scr_c)),
                                        wp.quat_identity()),
                     cfg=rig.cfg(density=0.0, has_shape_collision=False), label="screw_visual")
    j_scr = b.add_joint_revolute(
        parent=car, child=scr_body, axis=(0.0, 0.0, -1.0),
        parent_xform=wp.transform_identity(), child_xform=wp.transform_identity(),
        target_ke=0.0, target_kd=SCREW_KD, armature=SCREW_ARMATURE,
        limit_lower=-12 * np.pi, limit_upper=0.0)
    b.add_articulation([j_z, j_scr])
    set_body_mass(b, scr_body, float(scr_full.volume * KEY_DENSITY))
    rig._joint_of["screw_z"] = j_z
    rig._joint_of["screw_joint"] = j_scr
    for scen in ("block_top", "block_bottom", "table"):
        rig.filter_labels("screw", scen)

    # ── the Allen key: a free body; colliders = bend mesh + leg BOX + tip mesh ─
    # The gripped stretch of the long leg (x 9..52 mm, a straight 5 x 5 mm
    # prism with 0.5 mm edge fillets) is an ANALYTIC BOX: newton collides
    # mesh-vs-box per TRIANGLE with GJK/MPR, and the fillet triangles beside
    # the pad's bottom edge came back with ~45 deg normals (per-contact dump:
    # force-weighted |n_z| 0.43-0.48, the key pushed 4-6 N into the table and
    # creeping out of the lifted pinch at ~0.9 mm/s, dropped in the tilt --
    # with the fingertips 1.7, 0.5 and 0.2 mm above the table alike).  Box-box
    # goes through MPR + a clipped face manifold: face normals.  The bend /
    # short leg (x <= 9) and the drive end (x >= 52, fine SDF) stay meshes,
    # and the full CAD mesh is the visual.  Only the fillets of that stretch
    # are dropped (0.21 mm^2 of a 25 mm^2 section).
    key_raw = _load("allen_key/allen_key.stl")
    c_K = 0.5 * (key_raw.bounds[0] + key_raw.bounds[1])        # body origin in K
    tip = subdiv(crop_box(key_raw, (KEY_TIP_CROP_X, None, None), (None, None, None)), 0.0006)
    bend = subdiv(crop_box(key_raw, (None, None, None), (KEY_BOX_X0, None, None)), 0.0012)
    T_K0 = key_flat_pose(KEY_YAW0, KEY_PINCH_XY)
    T_B0 = T_K0 @ Tm(np.eye(3), c_K)
    key_body = b.add_body(xform=wp.transform(wp.vec3(*T_B0[:3, 3]),
                                             wp.quat(*HP.quat_of_R(T_B0[:3, :3]).tolist())),
                          label="allen_key")
    col = dict(mu=KEY_MU, gap=0.0005, density=KEY_DENSITY, mu_torsional=HP.MU_TORSION,
               is_visible=False)
    shapes = []
    for part, (tm, res, nb, mg) in {"key_tip": (tip, 256, 0.0012, 0.0002),
                                    "key_bend": (bend, 384, 0.0015, 0.0005)}.items():
        mesh, c = make_newton_mesh(tm, sdf=True, sdf_resolution=res, narrow_band=nb, margin=mg)
        off = np.asarray(c, float) - c_K
        shapes.append(b.add_shape_mesh(
            key_body, mesh=mesh, xform=wp.transform(wp.vec3(*off), wp.quat_identity()),
            cfg=rig.cfg(**col), label=part))
    box_c = np.array([0.5 * (KEY_BOX_X0 + KEY_TIP_CROP_X), 0.0, KEY_LEG_Z]) - c_K
    box_h = np.array([0.5 * (KEY_TIP_CROP_X - KEY_BOX_X0), KEY_HALF, KEY_HALF])
    shapes.append(b.add_shape_box(
        key_body, xform=wp.transform(wp.vec3(*box_c), wp.quat_identity()),
        hx=float(box_h[0]), hy=float(box_h[1]), hz=float(box_h[2]),
        cfg=rig.cfg(**col), label="key_leg"))
    vis_mesh, vis_c = make_newton_mesh(key_raw, sdf=False)
    b.add_shape_mesh(key_body, mesh=vis_mesh,
                     xform=wp.transform(wp.vec3(*(np.asarray(vis_c, float) - c_K)),
                                        wp.quat_identity()),
                     cfg=rig.cfg(density=0.0, has_shape_collision=False), label="key_visual")
    rig.shapes["allen_key"] = shapes
    set_body_mass(b, key_body, float(key_raw.volume * KEY_DENSITY))
    key_full = subdiv(key_raw, 0.0015)
    rig.meshes["allen_key"] = (key_full, c_K)
    HP.filter_arm_links(rig, "allen_key")
    HP.filter_statics_and_ground(rig)

    return {"T_sw": T_sw, "c_K": c_K, "T_K0": T_K0, "key_raw": key_raw, "tip": tip,
            "head_w": head_w, "scr_c": scr_c, "j_scr": j_scr, "j_z": j_z,
            "box_c": box_c, "box_h": box_h}


# ── analysis helpers ────────────────────────────────────────────────────
def _R_of_q(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def sat_overlap(Ra, ca, ha, Rb, cb, hb) -> float:
    """Penetration depth of two oriented boxes (separating-axis test over the
    15 axes; < 0 = separated by that much)."""
    axes = [Ra[:, i] for i in range(3)] + [Rb[:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            c = np.cross(Ra[:, i], Rb[:, j])
            n = np.linalg.norm(c)
            if n > 1e-6:
                axes.append(c / n)
    d = cb - ca
    best = 1e9
    for L in axes:
        ov = float(np.sum(ha * np.abs(Ra.T @ L)) + np.sum(hb * np.abs(Rb.T @ L)) - abs(d @ L))
        if ov < 0.0:
            return ov
        best = min(best, ov)
    return best


def pad_key_overlap(states_npz, box_c, box_h):
    """Per-frame SAT overlap (m) of each fingertip pad box with the key's leg
    box: the pad<->key penetration the pinch realises."""
    d = np.load(states_npz, allow_pickle=True)
    lab = [str(x) for x in d["body_labels"]]
    bq = d["body_q"]
    ik = lab.index("allen_key")
    fl = [lab.index("fr3/fr3_leftfinger"), lab.index("fr3/fr3_rightfinger")]
    out = np.zeros((len(d["t"]), 2))
    for f in range(len(d["t"])):
        Rk = _R_of_q(bq[f, ik, 3:])
        ck = bq[f, ik, :3] + Rk @ box_c
        for s, fi in enumerate(fl):
            Rf = _R_of_q(bq[f, fi, 3:])
            out[f, s] = sat_overlap(Rf, bq[f, fi, :3] + Rf @ PAD_BOX_C, PAD_BOX_H, Rk, ck, box_h)
    return np.asarray(d["t"], float), out


# ── the rollout ─────────────────────────────────────────────────────────
def run(mode: str):
    assert mode in ("tighten", "loosen")
    name = f"{mode}_screw_with_allen_key_from_table"
    stop_at = os.environ.get("AK_STOP", "")
    rig = appendix_rig(name)
    rig.hard_joint_limits = True              # the seat is a HARD stop
    rig.solver_options["impratio"] = IMPRATIO
    h = build_scene(rig, mode)
    b = rig.builder
    c_K = h["c_K"]
    T_K0 = h["T_K0"]

    # ── arm start: parked above the pick, elbow-up branch ───────────────
    T_tcp_pick = T_K0 @ G_NOM
    p_pick, R_pick = T_tcp_pick[:3, 3], T_tcp_pick[:3, :3]
    p_park = p_pick + np.array([0.0, 0.0, 0.060])
    jq = rig.ik.joint_q.numpy()
    jq[0, :7] = ARM_SEED
    rig.ik.joint_q.assign(jq)
    q_start = rig.solve_arm_start(p_park, HP.quat_of_R(R_pick))
    b.joint_q[7:9] = [F_OPEN_DOF, F_OPEN_DOF]
    b.joint_target_q[7:9] = [F_OPEN_DOF, F_OPEN_DOF]
    T_off = HP.tcp_offset_from_link7(rig)
    print(f"[{name}] arm start q {np.round(q_start[:7], 3)}, TCP err "
          f"{np.linalg.norm(rig.ik.tcp_pose()[:3] - p_park) / MM:.2f} mm")

    # screw start: tighten = threads unengaged (head UNENGAGED_RISE up),
    # loosen = seated
    j_scr, j_z = h["j_scr"], h["j_z"]
    q0 = q_unengaged() if mode == "tighten" else np.radians(LOOSEN_Q0_DEG)
    z0 = -LEAD * q0
    b.joint_q[int(b.joint_q_start[j_scr])] = float(q0)
    b.joint_q[int(b.joint_q_start[j_z])] = float(z0)
    print(f"[{name}] screw start {np.degrees(q0):.1f} deg, head {z0 / MM:.2f} mm above the seat")

    cam_t, cam2_t = CAM_TGT.copy(), CAM2_TGT.copy()
    cam_p, cam2_p = cam_t + CAM_OFF, cam2_t + CAM2_OFF
    if os.environ.get("AK_PREVIEW"):
        # camera contact sheet from saved states, no physics (HP.preview_cameras)
        rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
        HP.preview_cameras(
            rig, os.environ["AK_PREVIEW"],
            [float(v) for v in os.environ.get("AK_PREVIEW_T", "2.6,8.6,12,16,20").split(",")],
            HP.parse_cams(os.environ.get(
                "AK_PREVIEW_CAMS",
                f"{','.join(map(str, cam_t))}:{','.join(map(str, cam_p - cam_t))},{CAM_FOV};"
                f"{','.join(map(str, cam2_t))}:{','.join(map(str, cam2_p - cam2_t))},{CAM_FOV}")),
            os.environ.get("AK_PREVIEW_OUT", str(rig.out_dir / "camera_preview.png")),
            fov_deg=CAM_FOV)
        return None
    rig.finalize(probes=["allen_key"], cam_pos=cam_p, cam_target=cam_t, record=False)
    rig.rec = RolloutRecorder(rig.model, rig.out_dir, cam_pos=cam_p, cam_target=cam_t,
                              fps=rig.fps, fov_deg=float(CAM_FOV),
                              cam2_pos=cam2_p, cam2_target=cam2_t)
    model = rig.model
    # kit colours (after the recorder's studio palette): yellow block, grey
    # screw, black key
    sc = model.shape_color.numpy()
    labels = list(model.shape_label)
    for lbl, colr in KIT_COLORS.items():
        sc[labels.index(lbl)] = colr
    model.shape_color.assign(sc)
    rig.collide_every_substep = True          # per-substep collide throughout
    rig.solver = make_solver(rig.model, iterations=20, **rig.solver_options)
    q_scr = int(model.joint_q_start.numpy()[j_scr])
    q_car = int(model.joint_q_start.numpy()[j_z])
    for state in (rig.s0, rig.s1):
        jqs = state.joint_q.numpy()
        jqs[q_scr] = float(q0)
        jqs[q_car] = float(z0)
        state.joint_q.assign(jqs)
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
    thread = ThreadModel(rig.solver, model, j_z, j_scr, float(q0),
                         LOOSE_PRELOAD0 if mode == "loosen" else 0.0)
    rig.solver = thread

    key_lbls = ["key_tip", "key_leg", "key_bend"]
    rig.probes["padL"] = HP.PairProbe(model, "fr3/fr3_leftfinger", key_lbls)
    rig.probes["padR"] = HP.PairProbe(model, "fr3/fr3_rightfinger", key_lbls)
    rig.probes["key_table"] = HP.PairProbe(model, "allen_key", ["table"])
    rig.probes["key_block"] = HP.PairProbe(model, "allen_key", ["block_top", "block_bottom"])
    rig.probes["key_screw"] = HP.PairProbe(model, "allen_key", ["screw_shape"])
    rig.probes["robot_key"] = RobotContactProbe(model, ["allen_key"])
    rig.probes["robot_screw"] = RobotContactProbe(model, ["screw"],
                                                  static_shapes=["block_top", "block_bottom"])
    drive = DriveContacts(model, rig.shapes["allen_key"], rig.shapes["screw"])

    ki = rig.body_index("allen_key")
    i7 = rig.body_index("fr3/fr3_link7")
    T_bK = Tm(np.eye(3), -c_K)                # key body frame -> K
    weight = float(h["key_raw"].volume * KEY_DENSITY * 9.81)
    v_low = np.asarray(h["key_raw"].vertices[::5], float)
    trk = HP.JointTracker(rig, q_start)
    droop = HP.DroopComp()

    def last(nm, key="force"):
        hh = rig.probes[nm].history
        return hh[-1][key] if hh else 0.0

    def last_vec(nm):
        hh = rig.probes[nm].history
        return np.asarray(hh[-1]["vec"], float) if hh else np.zeros(3)

    def key_T(bq):
        return world_T(bq[ki]) @ T_bK

    def tcp_T(bq):
        return world_T(bq[i7]) @ T_off

    def screw_qz():
        jq_ = rig.s0.joint_q.numpy()
        return float(jq_[q_scr]), float(jq_[q_car])

    def drive_tau():
        return abs(float(drive.history[-1]["tau"])) if drive.history else 0.0

    def drive_tau_dir():
        """Drive torque at the recess in the stroke's sense (> 0 = the key
        still turns the screw the way the stroke did).  DriveContacts' tau is
        about +z; clockwise (+q here) is -z."""
        return -sign * float(drive.history[-1]["tau"]) if drive.history else 0.0

    # ── expert state ────────────────────────────────────────────────────
    sign = 1.0 if mode == "tighten" else -1.0        # screw q direction
    dpsi = -sign                                      # hand orbit direction
    near0 = np.degrees(PSI_START[mode])
    psi0 = np.radians(congruent_psi(q0, near0))
    st = dict(ph="settle", t0=0.0, t_start={"settle": 0.0}, G=G_NOM.copy(), G_n=0,
              corr=np.zeros(3), ef=np.zeros(3), p=p_park.copy(), R=R_pick.copy(),
              f=F_OPEN_DOF, psi=float(psi0), psi_start=float(psi0), stroke=0,
              z_end=0.0, press=0.0, seated=False, t_seat=None, tau_seat=0.0,
              preload=LOOSE_PRELOAD0 if mode == "loosen" else 0.0, trim=0.0,
              released=(mode == "tighten"), done_turning=False, final=False,
              probe=0.0, from_p=None, from_R=None, psi_end=None, place_z=None,
              ins_depth=[], G_log=[], strokes=[], unwinds=[], turn_sub="run", t_sub=0.0,
              psi_seat=None, tau_pk=0.0, unw_ok=0)
    order = ORDER
    log = []

    def go(ph, t):
        q_, z_ = screw_qz()
        print(f"t={t:6.2f}s -> {ph}   screw {np.degrees(q_):8.2f} deg (head +{z_ / MM:5.2f} mm)  "
              f"pads {last('padL'):5.2f}/{last('padR'):5.2f} N  key-screw {last('key_screw'):5.2f} N"
              f"  tau {drive_tau() * 1000:5.1f} mN.m")
        st["ph"], st["t0"] = ph, t
        st["t_start"].setdefault(ph, t)
        st.setdefault("t_starts", []).append((ph, t))
        st["from_p"], st["from_R"] = st["p"].copy(), st["R"].copy()
        if stop_at and ph == stop_at:
            raise StopRun

    def frac(t, ph=None):
        return min(1.0, (t - st["t0"]) / T[ph or st["ph"]])

    def end_loop(err, gain=0.06, step=0.25 * MM, db=0.08 * MM, alpha=0.25):
        """Rate-limited integral on a measured position error (xy)."""
        e = np.array([err[0], err[1], 0.0])
        st["ef"] = st["ef"] + alpha * (e - st["ef"])
        ee = np.sign(st["ef"]) * np.maximum(np.abs(st["ef"]) - db, 0.0)
        st["corr"] = np.clip(st["corr"] + np.clip(gain * ee, -step, step), -6 * MM, 6 * MM)

    def tcp_for_key(T_K_des):
        T_ = T_K_des @ st["G"]
        return T_[:3, 3] + st["corr"], T_[:3, :3]

    def regrasp_measure(TK, Tt):
        """Accumulate the in-hand pose (TCP in K) while the key hangs free."""
        st.setdefault("G_acc", []).append(np.linalg.inv(TK) @ Tt)

    def regrasp_commit():
        acc = st.pop("G_acc", [])
        if not acc:
            return
        Gn = acc[-1].copy()
        Gn[:3, 3] = np.mean([g[:3, 3] for g in acc], axis=0)
        d = (Gn[:3, 3] - st["G"][:3, 3]) / MM
        # a slip of the key in the pinch is now in G itself, so the vertical
        # correction learned against the old G is dropped (re-learned in the
        # next insertion); the xy correction is re-learned by the end loop
        st["G"] = Gn
        st["corr"][2] = 0.0
        st["G_log"].append(np.round(Gn[:3, 3] / MM, 2).tolist())
        print(f"   in-hand pose re-measured: TCP in K moved {np.round(d, 2)} mm")

    def press_servo(fz, target=PRESS_N, rate=0.5e-5):
        err = target - fz
        if abs(err) > 0.3:
            st["press"] = float(np.clip(st["press"] + rate * np.sign(err), *PRESS_LIM))

    def set_preload(tau):
        st["preload"] = float(tau)
        thread.set_preload(tau, screw_qz()[0])

    def grip_cmd():
        """Finger target: F_CLOSE_DOF plus a slow admittance trim that only
        acts when the mean pad force leaves GRIP_BAND (GelSight readiness)."""
        F = 0.5 * (last("padL") + last("padR"))
        if F < GRIP_BAND[0]:
            st["trim"] = max(-GRIP_TRIM_MAX, st["trim"] - GRIP_TRIM_STEP)
        elif F > GRIP_BAND[1]:
            st["trim"] = min(GRIP_TRIM_MAX, st["trim"] + GRIP_TRIM_STEP)
        return F_CLOSE_DOF + st["trim"]

    def end_stroke(t, q):
        """Bookkeeping at the end of a stroke's dwell / torque-up."""
        st["stroke"] += 1
        k0 = st["stroke_q0"]
        st["strokes"].append({"stroke": st["stroke"],
                              "hand_from_deg": round(float(np.degrees(st["psi_turn0"])), 1),
                              "hand_to_deg": round(float(np.degrees(st["psi"])), 1),
                              "screw_from_deg": round(float(np.degrees(k0)), 1),
                              "screw_to_deg": round(float(np.degrees(q)), 1),
                              "peak_drive_torque_mNm": round(st["tau_pk"] * 1000, 2)})

    def control(t):
        bq = rig.s0.body_q.numpy()
        TK = key_T(bq)
        Tt = tcp_T(bq)
        E = TK[:3, :3] @ np.array([KEY_END_X, 0.0, KEY_LEG_Z]) + TK[:3, 3]
        P = TK[:3, :3] @ np.array([KEY_XP, 0.0, KEY_LEG_Z]) + TK[:3, 3]
        q, zsc = screw_qz()
        head = Z0 + HEAD_TOP_Z + zsc            # MEASURED head top / recess floor
        floor = Z0 + RECESS_FLOOR_Z + zsc
        ph = st["ph"]
        s5 = HP.smooth5
        f_cmd = st["f"]
        p, R = st["p"], st["R"]

        if ph == "settle":
            f_cmd = F_OPEN_DOF
            if t - st["t0"] >= T["settle"]:
                go("descend", t)
        elif ph == "descend":
            p = st["from_p"] + (p_pick - st["from_p"]) * s5(frac(t))
            if t - st["t0"] >= T["descend"]:
                go("pre", t)
        elif ph == "pre":
            p = p_pick
            if t - st["t0"] >= T["pre"]:
                go("close", t)
        elif ph == "close":
            p = p_pick
            f_cmd = F_OPEN_DOF + (F_CLOSE_DOF - F_OPEN_DOF) * s5(frac(t))
            if t - st["t0"] >= T["close"]:
                go("hold", t)
        elif ph == "hold":
            p = p_pick
            if t - st["t0"] >= T["hold"]:
                go("lift", t)
        elif ph == "lift":
            p = p_pick + np.array([0.0, 0.0, 0.08]) * s5(frac(t))
            if t - st["t0"] >= T["lift"]:
                go("lifthold", t)
        elif ph == "lifthold":
            # measure the real in-hand pose (static): G = TK^-1 * T_tcp
            Gm = np.linalg.inv(TK) @ Tt
            if t - st["t0"] > 0.1:
                n = st["G_n"]
                Gp = Gm.copy()
                if n:
                    Gp[:3, 3] = (st["G"][:3, 3] * n + Gm[:3, 3]) / (n + 1)
                st["G"], st["G_n"] = Gp, n + 1
            if t - st["t0"] >= T["lifthold"]:
                g = st["G"]
                print(f"   in-hand grasp: TCP in K {np.round(g[:3, 3] / MM, 2)} mm "
                      f"(nominal {np.round(G_NOM[:3, 3] / MM, 2)})")
                go("carry", t)
        elif ph == "carry":
            T_h = key_drive_pose(st["psi_start"], head + HOVER_DZ)
            p_h, _ = tcp_for_key(T_h)
            p_t = np.array([p_h[0], p_h[1], Z0 + TILT_Z])
            R_t = Rz(st["psi_start"]) @ R_DOWN
            u = s5(frac(t))
            p = st["from_p"] + (p_t - st["from_p"]) * u
            R = slerpR(st["from_R"], R_t, u)
            if t - st["t0"] >= T["carry"]:
                go("tilt", t)
        elif ph == "tilt":
            T_h = key_drive_pose(st["psi_start"], head + HOVER_DZ)
            _, R_h = tcp_for_key(T_h)
            R = slerpR(st["from_R"], R_h, s5(frac(t)))
            p = st["from_p"]
            if t - st["t0"] >= T["tilt"]:
                go("hover", t)
        elif ph in ("hover", "align"):
            z_h = st["z_end"] if st["stroke"] > 0 else head + HOVER_DZ
            T_h = key_drive_pose(st["psi"], z_h)
            p_h, R_h = tcp_for_key(T_h)
            if ph == "hover":
                u = s5(frac(t))
                p = st["from_p"] + (p_h - st["from_p"]) * u
                R = slerpR(st["from_R"], R_h, u)
                if u > 0.6:
                    end_loop(SCREW_XY - E[:2])
            else:
                # static: the xy loop on the measured key end, and the arm's
                # vertical droop learned on the same end (stationary command)
                p, R = p_h, R_h
                end_loop(SCREW_XY - E[:2])
                st["corr"][2] = float(np.clip(
                    st["corr"][2] + np.clip(0.15 * (z_h - E[2]), -0.05 * MM, 0.05 * MM),
                    -4 * MM, 4 * MM))
            if t - st["t0"] >= T[ph]:
                st["z_end"] = z_h
                go("align" if ph == "hover" else ("insert" if st["stroke"] == 0 else "reinsert"), t)
        elif ph in ("insert", "reinsert"):
            # fast to 1.5 mm above the MEASURED head, slow through the entry
            # chamfer, then down to the floor; the xy loop on the measured key
            # end runs until the end is at the mouth, then the recess guides
            # it.  The vertical droop was learned while hovering still
            # ("align"), so the end is driven open-loop down to 0.1 mm above
            # the floor (an integrator here wound up against the arm's lag and
            # dumped 1 mm onto the floor at the turn start: 16.9 N, the key
            # slid 2 mm in the pinch -- pitfall 8.4)
            dz_mouth = E[2] - head
            depth = -dz_mouth
            v = (V_INSERT_FAST if dz_mouth > 1.5 * MM else
                 V_INSERT_SLOW if dz_mouth > -1.0 * MM else
                 V_INSERT_IN if depth < 2.5 * MM else V_INSERT_END)
            st["z_end"] = max(floor + 0.1 * MM, st["z_end"] - v / rig.fps)
            fz = last_vec("key_screw")[2]
            if dz_mouth > 0.8 * MM:
                end_loop(SCREW_XY - E[:2])
            st["press"] = 0.0
            T_d = key_drive_pose(st["psi"], st["z_end"])
            p, R = tcp_for_key(T_d)
            if depth > 3.5 * MM or fz > 0.3 or t - st["t0"] > T_INSERT_MAX:
                st["press"] = -0.1 * MM
                st["ins_depth"].append(round(float(depth / MM), 2))
                st["psi_turn0"] = st["psi"]
                st["stroke_q0"] = q
                st["tau_pk"] = 0.0
                st["turn_sub"] = "run"
                go("turn", t)
        elif ph == "turn":
            # vertical press servo on the measured key -> screw force; the
            # floor it is bounded against follows the head down its thread
            fz = last_vec("key_screw")[2]
            press_servo(fz)
            sub = st["turn_sub"]
            tau_now = drive_tau()
            st["tau_pk"] = max(st["tau_pk"], tau_now)
            if sub == "run":
                ramp = min(1.0, (t - st["t0"]) / T_RAMP)
                st["psi"] = st["psi"] + dpsi * STROKE_RATE * ramp / rig.fps
                if abs(np.degrees(st["psi"] - st["psi_turn0"])) >= STROKE_DEG or \
                        (mode == "loosen" and np.degrees(q0 - q) >= LOOSEN_TARGET_DEG):
                    st["turn_sub"], st["t_sub"] = "dwell", t
            elif sub == "ot":
                # torque up: creep on until the recess carries TAU_TIGHT
                st["psi"] = st["psi"] + dpsi * OT_RATE / rig.fps
                if tau_now >= TAU_TIGHT or abs(np.degrees(st["psi"] - st["psi_seat"])) >= OT_MAX_DEG:
                    print(f"   torqued up: {tau_now * 1000:.1f} mN.m after "
                          f"{abs(np.degrees(st['psi'] - st['psi_seat'])):.1f} deg of over-travel")
                    st["ot_deg"] = round(abs(float(np.degrees(st["psi"] - st["psi_seat"]))), 1)
                    end_stroke(t, q)
                    go("seat_hold", t)
            T_d = key_drive_pose(st["psi"], floor - st["press"])
            p, R = tcp_for_key(T_d)
            end_loop(SCREW_XY - P[:2], gain=0.03)
            if mode == "tighten" and not st["seated"] and np.degrees(q) >= -SEAT_TOL_DEG \
                    and st["ph"] == "turn":
                st["seated"], st["t_seat"] = True, t
                st["psi_seat"] = st["psi"]
                st["turn_sub"] = "ot"
                print(f"   seated at t={t:.2f}s, hand command {np.degrees(st['psi']):.1f} deg")
            if st["ph"] == "turn" and st["turn_sub"] == "dwell" and t - st["t_sub"] >= T_DWELL:
                end_stroke(t, q)
                st["final"] = (mode == "loosen" and np.degrees(q0 - q) >= LOOSEN_TARGET_DEG) \
                    or st["stroke"] >= MAX_STROKES
                st["psi_unw0"], st["q_unw0"] = st["psi"], q
                st["unw_ok"] = 0
                go("unwind", t)
        elif ph == "unwind":
            # relieve the stroke's wind-up before the key leaves the recess:
            # orbit back until the recess carries (almost) no torque in the
            # stroke's sense -- lifting a wound-up key drags on the recess
            # walls and slips it in the pinch (reviewer, round 1: 28.6 mN.m
            # at the start of the extraction, 6.7 N on the walls)
            fz = last_vec("key_screw")[2]
            press_servo(fz, PRESS_UNWIND_N)
            tau_dir = drive_tau_dir()
            done_deg = abs(np.degrees(st["psi"] - st["psi_unw0"]))
            lim = UNWIND_MAX_DEG["seat" if st["final"] and st["seated"] else "stroke"]
            back_deg = -sign * np.degrees(q - st["q_unw0"])     # screw dragged back
            stop = ""
            if tau_dir < TAU_UNWOUND:
                st["unw_ok"] += 1
            else:
                st["unw_ok"] = 0
            if st["unw_ok"] >= 2:
                stop = "relieved"
            elif tau_dir < -0.5 * TAU_UNWOUND or back_deg > 0.3:
                stop = "reversed"
            elif done_deg >= lim:
                stop = "limit"
            elif t - st["t0"] >= T["unwind"]:
                stop = "timeout"
            if not stop:
                st["psi"] = st["psi"] - dpsi * UNWIND_RATE / rig.fps
            T_d = key_drive_pose(st["psi"], floor - st["press"])
            p, R = tcp_for_key(T_d)
            end_loop(SCREW_XY - P[:2], gain=0.03)
            if stop:
                st["unwinds"].append({"stroke": st["stroke"], "unwound_deg": round(done_deg, 1),
                                      "stop": stop,
                                      "drive_torque_left_mNm": round(tau_dir * 1000, 2),
                                      "screw_dragged_back_deg": round(float(back_deg), 2)})
                print(f"   unwound {done_deg:.1f} deg ({stop}), {tau_dir * 1000:.2f} mN.m left, "
                      f"screw back {back_deg:.2f} deg")
                if st["final"]:
                    st["done_turning"] = True
                    go("extract", t)
                else:
                    go("liftout", t)
        elif ph == "liftout":
            if "lo_z0" not in st or st.get("lo_t0") != st["t0"]:
                st["lo_z0"], st["lo_t0"] = floor - st["press"], st["t0"]
            z1_ = head + LIFTOUT_DZ
            st["z_end"] = st["lo_z0"] + (z1_ - st["lo_z0"]) * s5(frac(t))
            T_d = key_drive_pose(st["psi"], st["z_end"])
            p, R = tcp_for_key(T_d)
            if t - st["t0"] >= T["liftout"]:
                st["psi_from"] = st["psi"]
                st["psi_to"] = np.radians(congruent_psi(q, np.degrees(st["psi_start"])))
                print(f"   back: screw {np.degrees(q):.1f} deg -> re-insert at hand "
                      f"{np.degrees(st['psi_to']):.1f} deg")
                go("back", t)
        elif ph == "back":
            # orbit back with the end out of the recess (no xy loop: what it
            # sees here is the arm's transient lag, not an offset)
            st["psi"] = st["psi_from"] + (st["psi_to"] - st["psi_from"]) * s5(frac(t))
            T_d = key_drive_pose(st["psi"], st["z_end"])
            p, R = tcp_for_key(T_d)
            if frac(t) > 0.8:
                regrasp_measure(TK, Tt)
            if t - st["t0"] >= T["back"]:
                regrasp_commit()
                st["press"] = 0.0
                go("align", t)
        elif ph == "seat_hold":
            T_d = key_drive_pose(st["psi"], floor - st["press"])
            p, R = tcp_for_key(T_d)
            if t - st["t0"] >= T["seat_hold"]:
                st["preload"] = float(np.clip(PRELOAD_FRAC * st["tau_seat"],
                                              PRELOAD_MIN, PRELOAD_MAX))
                set_preload(st["preload"])
                print(f"   seat torque {st['tau_seat'] * 1000:.1f} mN.m -> preload "
                      f"{st['preload'] * 1000:.1f} mN.m")
                st["final"] = True
                st["psi_unw0"], st["q_unw0"] = st["psi"], q
                st["unw_ok"] = 0
                go("unwind", t)
        elif ph == "extract":
            if "ex_z0" not in st:
                st["ex_z0"] = floor - st["press"]
            st["z_end"] = st["ex_z0"] + (head + EXTRACT_DZ - st["ex_z0"]) * s5(frac(t))
            T_d = key_drive_pose(st["psi"], st["z_end"])
            p, R = tcp_for_key(T_d)
            if t - st["t0"] >= T["extract"]:
                st["psi_end"] = st["psi"]
                st["up_tcpz0"] = float(p[2])
                go("up", t)
        elif ph == "up":
            # rise to the untilt height while orbiting the free key to
            # PSI_UNTILT (more joint margin there for the untilt)
            u = s5(frac(t))
            psi_u = st["psi_end"] + (PSI_UNTILT[mode] - st["psi_end"]) * u
            p, R = tcp_for_key(key_drive_pose(psi_u, st["z_end"]))
            p = np.array([p[0], p[1], st["up_tcpz0"] + (Z0 + TILT_Z - st["up_tcpz0"]) * u])
            if t - st["t0"] >= T["up"]:
                go("untilt", t)
        elif ph == "untilt":
            R_t = Rz(PSI_UNTILT[mode]) @ R_DOWN
            R = slerpR(st["from_R"], R_t, s5(frac(t)))
            p = st["from_p"]
            if t - st["t0"] >= T["untilt"]:
                go("carryback", t)
        elif ph == "carryback":
            T_pl = T_K0 @ st["G"]
            p_t = T_pl[:3, 3] + np.array([0.0, 0.0, PLACE_ABOVE])
            u = s5(frac(t))
            p = st["from_p"] + (p_t - st["from_p"]) * u
            R = slerpR(st["from_R"], T_pl[:3, :3], u)
            if t - st["t0"] >= T["carryback"]:
                st["corr"] = np.zeros(3)
                go("lower", t)
        elif ph == "lower":
            # lower on the MEASURED key: fast until its lowest point is 3 mm
            # above the table, then slow, until it is 0.3 mm up or the table
            # carries half its weight; then open
            low = float((TK[:3, :3] @ v_low.T).T[:, 2].min() + TK[2, 3]) - Z0
            v = 0.015 if low > 3.0 * MM else 0.003
            p = st["p"] - np.array([0.0, 0.0, v / rig.fps])
            R = (T_K0 @ st["G"])[:3, :3]
            if low <= 0.3 * MM or last_vec("key_table")[2] > 0.5 * weight \
                    or t - st["t0"] > T_PLACE_MAX:
                st["place_low_mm"] = round(low / MM, 3)
                st["f_open0"] = st["f"]
                go("open", t)
        elif ph == "open":
            f_cmd = st["f_open0"] + (F_OPEN_DOF - st["f_open0"]) * s5(frac(t))
            if t - st["t0"] >= T["open"]:
                go("retreat", t)
        elif ph == "retreat":
            p = st["from_p"] + np.array([0.0, 0.0, 0.08]) * s5(frac(t))
            if t - st["t0"] >= T["retreat"]:
                go("still", t)
        elif ph == "still":
            if t - st["t0"] >= T["still"]:
                go("probe", t)
        elif ph == "probe":
            # loosen-way, then tighten-way (+q = tighten-way)
            k = (t - st["t0"]) / T["probe"]
            st["probe"] = (-PROBE_TAU if k < 1.0 else (PROBE_TAU if k < 2.0 else 0.0))
            if k >= 2.3:
                raise StopRun

        # fingers: closed (admittance-trimmed) through the whole carry/turn
        if st["ph"] in HELD:
            f_cmd = grip_cmd()
        if st["ph"] in ("retreat", "still", "probe"):
            f_cmd = F_OPEN_DOF

        # seat bookkeeping and the preload model
        if mode == "tighten" and st["seated"] and st["ph"] in ("turn", "seat_hold"):
            st["tau_seat"] = max(st["tau_seat"], drive_tau())
        if (not st["released"]) and np.degrees(q) <= np.degrees(q0) - LOOSE_FREE_DEG:
            st["released"] = True
            set_preload(0.0)
            print(f"   preload released at t={t:.2f}s (screw {np.degrees(q):.1f} deg)")
        thread.probe = st["probe"]

        st["p"], st["R"], st["f"] = np.asarray(p, float), np.asarray(R, float), f_cmd
        pick_ph = st["ph"] in ("settle", "descend", "pre", "close", "hold")
        if pick_ph:
            droop.update(st["ph"] in ("settle", "pre"), p, Tt[:3, 3], st.get("p_last"))
        st["p_last"] = np.asarray(p, float)
        p_out = st["p"] + droop.v
        trk.step(p_out, HP.quat_of_R(st["R"]), f_cmd)

    def extra(state, t):
        thread.refresh()
        bq = state.body_q.numpy()
        TK = key_T(bq)
        E = TK[:3, :3] @ np.array([KEY_END_X, 0.0, KEY_LEG_Z]) + TK[:3, 3]
        P = TK[:3, :3] @ np.array([KEY_XP, 0.0, KEY_LEG_Z]) + TK[:3, 3]
        ky = TK[:3, :3] @ np.array([0.0, 1.0, 0.0])
        kx = TK[:3, :3] @ np.array([1.0, 0.0, 0.0])
        jq_ = state.joint_q.numpy()
        q, zsc = float(jq_[q_scr]), float(jq_[q_car])
        c = drive.sample(rig.contacts, state, SCREW_XY, t)
        Tt = tcp_T(bq)
        lowz = float(((TK[:3, :3] @ h["key_raw"].vertices[::5].T).T + TK[:3, 3])[:, 2].min())
        psi_key = float(np.arctan2(ky[1], ky[0]))
        R7 = world_T(bq[i7])[:3, :3]
        row = dict(t=t, q=q, z=zsc, Ex=E[0], Ey=E[1], Ez=E[2], Px=P[0], Py=P[1], Pz=P[2],
                   kx_z=kx[2], ky_z=ky[2], key_x=TK[0, 3], key_y=TK[1, 3], key_z=TK[2, 3],
                   com_z=float((TK[:3, :3] @ h["key_raw"].center_mass + TK[:3, 3])[2]),
                   low_z=lowz, padL=last("padL"), padR=last("padR"),
                   table=last("key_table"), table_fz=last_vec("key_table")[2],
                   block=last("key_block"), kscrew=last("key_screw"),
                   kscrew_fz=last_vec("key_screw")[2], tau=c["tau"], F=c["F"],
                   robot_key=last("robot_key"), robot_screw=last("robot_screw"),
                   tcp_x=Tt[0, 3], tcp_y=Tt[1, 3], tcp_z=Tt[2, 3], probe=st["probe"],
                   preload=thread.preload, F_ax=thread.F_ax, tau_f=thread.tau_f,
                   psi=st["psi"], psi_key=psi_key,
                   # key K_y seen from link7 (in-hand twist about the leg)
                   ky7x=float((R7.T @ ky)[0]), ky7y=float((R7.T @ ky)[1]),
                   ky7z=float((R7.T @ ky)[2]),
                   f_cmd=st["f"], ph=order.index(st["ph"]))
        log.append(row)
        if int(round(t * rig.fps)) % 20 == 0:
            print(f"  [{t:6.2f} {st['ph']:9s}] q {np.degrees(q):8.2f} z {zsc / MM:5.2f}  pads "
                  f"{row['padL']:5.2f}/{row['padR']:5.2f}  table {row['table']:5.3f}  kscrew "
                  f"{row['kscrew']:5.2f} (fz {row['kscrew_fz']:5.2f}) tau {c['tau'] * 1000:6.2f} mNm"
                  f"  fric {thread.tau_f * 1000:5.2f} Fax {thread.F_ax:5.2f}  end "
                  f"{np.round((E - np.r_[SCREW_XY, Z0 + RECESS_FLOOR_Z + zsc]) / MM, 2)} mm  "
                  f"kx_z {kx[2]:+.3f}  corr {np.round(st['corr'] / MM, 2)} trim {st['trim'] / MM:+.3f}")
        return {"padL_N": round(row["padL"], 3), "padR_N": round(row["padR"], 3),
                "key_screw_N": round(row["kscrew"], 3), "key_table_N": round(row["table"], 4),
                "drive_torque_mNm": round(c["tau"] * 1000, 2),
                "screw_deg": round(np.degrees(q), 3),
                "screw_rise_mm": round(zsc / MM, 4),
                "thread_friction_mNm": round(thread.tau_f * 1000, 3),
                "robot_key_N": round(row["robot_key"], 4),
                "robot_screw_N": round(row["robot_screw"], 4),
                "probe_torque_mNm": round(st["probe"] * 1000, 2),
                "seat_preload_mNm": round(thread.preload * 1000, 2)}

    try:
        rec = rig.run(T_BUDGET, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{name}] rollout ended at t={len(rec.times) / rig.fps:.2f}s (phase {st['ph']})")
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "key_screw_N",
                                            "drive_torque_mNm", "probe_torque_mNm"])
    print("saved:", paths)
    np.savez_compressed(rig.out_dir / "expert_log.npz",
                        **{k: np.array([r[k] for r in log]) for k in log[0]},
                        phases=np.array(order, dtype=object),
                        t_starts=np.array([(p_, t_) for p_, t_ in st.get("t_starts", [])],
                                          dtype=object))
    print("strokes:", st["strokes"])
    print("unwinds:", st["unwinds"])
    if stop_at:
        print("AK_STOP set: QA skipped")
        return None
    return verdict(rig, mode, name, paths, log, st, h, weight, q0)


SPEC = {
    "tighten": ("The screw is tightened and remains stationary when a small force is "
                "applied in an attempt to rotate the screw. The robot movement stops after "
                "terminating the tightening motion, moving away from, and breaking contact "
                "with the screw."),
    "loosen": ("The screw is loosened and can rotate freely when a small force is applied "
               "in an attempt to rotate the screw. The robot movement stops after "
               "terminating the loosening motion, moving away from, and breaking contact "
               "with the screw."),
}
TITLE = {"tighten": "Tighten screw with Allen key", "loosen": "Loosen screw with Allen key"}


def verdict(rig, mode, name, paths, log, st, h, weight, q0):
    """appendix_qa.json: the appendix criterion item by item, plus the user's
    additions (picked up from the table, put back on the table)."""
    qa = rig.qa(paths)
    L = {k: np.array([r[k] for r in log]) for k in log[0]}
    t = L["t"]
    ts = st["t_start"]
    q_deg = np.degrees(L["q"])
    z_mm = L["z"] / MM
    t_end = float(t[-1])
    ph_names = np.array([ORDER[int(i)] for i in L["ph"]])

    def win(a, b_):
        return (t >= a) & (t <= b_)

    crit = {}
    # ── picked up from the table by a fingertip pinch ───────────────────
    rest0 = win(0.2, ts["descend"])
    lift_w = win(ts["lift"], ts["tilt"])
    held = win(ts["lift"] + 0.3, ts["open"])
    both = float(((L["padL"][held] > 0.5) & (L["padR"][held] > 0.5)).mean())
    rise = float((L["com_z"][lift_w].max() - L["com_z"][0]) / MM)
    tab_air = float(L["table"][win(ts["lift"] + 0.5, ts["lower"])].max())
    crit["picked_up_from_table"] = {
        "pass": bool(abs(L["table_fz"][rest0].mean() - weight) < 0.3 * weight and rise > 50.0
                     and both > 0.98 and tab_air < 0.01),
        "table_support_at_start_N": round(float(L["table_fz"][rest0].mean()), 4),
        "key_weight_N": round(weight, 4),
        "lift_mm": round(rise, 1),
        "both_pads_loaded_fraction_while_held": round(both, 3),
        "pad_force_mean_while_held_N": [round(float(L["padL"][held].mean()), 2),
                                        round(float(L["padR"][held].mean()), 2)],
        "pad_force_min_while_held_N": [round(float(L["padL"][held].min()), 2),
                                       round(float(L["padR"][held].min()), 2)],
        "table_contact_in_air_max_N": round(tab_air, 4),
        "note": "lying flat on the table at t=0; a top-down pinch across the long leg's "
                "5 mm width; no joint, weld or kinematic attachment exists between the "
                "robot and the key"}

    # ── the screw ───────────────────────────────────────────────────────
    t_probe = ts.get("probe", t_end)
    gone = win(ts.get("still", t_end), t_probe - 1e-6)   # tool off, hand away, no probe
    ang_end = float(q_deg[gone].mean()) if gone.any() else float(q_deg[-1])
    z_end = float(z_mm[gone].mean()) if gone.any() else float(z_mm[-1])
    turn_m = ph_names == "turn"
    tau = L["tau"]
    tau_pk = float(np.abs(tau[turn_m]).max()) if turn_m.any() else 0.0

    def probe_moves(kind):
        out = {}
        for k, nm in ((0, "loosen_way"), (1, "tighten_way")):
            a_, b2 = t_probe + k * T["probe"], t_probe + (k + 1) * T["probe"]
            w = (t > a_ + 1e-6) & (t <= b2 + 1e-6)
            i_pre = int(np.searchsorted(t, a_ + 1e-6)) - 1
            if not w.any() or i_pre < 0:
                out[nm] = -1.0
                continue
            ref = q_deg[i_pre]
            out[nm] = round(float(np.abs(q_deg[w] - ref).max()) if kind == "max"
                            else float(abs(q_deg[w][-1] - ref)), 4)
        return out

    if mode == "tighten":
        turned = float(ang_end - q_deg[0])
        crit["screw_tightened"] = {
            "pass": bool(st["seated"] and ang_end >= -1.0 and turned >= TIGHT_TURN_MIN_DEG
                         and abs(z_end) < 0.2 and st["tau_seat"] >= 0.8 * TAU_TIGHT),
            "start_deg_from_seat": round(float(q_deg[0]), 1),
            "start_head_rise_mm": round(float(z_mm[0]), 3),
            "turned_deg": round(turned, 1), "turned_min_deg": TIGHT_TURN_MIN_DEG,
            "angle_from_seat_after_tool_left_deg": round(ang_end, 3), "max_from_seat_deg": 1.0,
            "head_rise_after_tool_left_mm": round(z_end, 4), "max_head_rise_mm": 0.2,
            "strokes": int(st["stroke"]),
            "insertion_depth_per_stroke_mm": st["ins_depth"],
            "seating_drive_torque_mNm": round(st["tau_seat"] * 1000, 2),
            "seating_torque_min_mNm": round(0.8 * TAU_TIGHT * 1000, 1),
            "over_travel_to_torque_deg": st.get("ot_deg"),
            "breakaway_preload_mNm": round(st["preload"] * 1000, 2),
            "note": "starts with the threads unengaged: head 4.0 mm above the seat = 615 deg "
                    "of the 2.34 mm right-hand thread; seat = hard stop q = 0 / z = 0; angle "
                    "and head height measured with the key out of the recess and the hand "
                    "away ('still' phase); the seating torque is measured at the recess"}
        moves = probe_moves("max")
        crit["stationary_under_small_torque"] = {
            "pass": bool(all(0.0 <= v < 1.0 for v in moves.values())),
            "probe_torque_mNm": PROBE_TAU * 1000, "max_rotation_deg": moves,
            "threshold_deg": 1.0, "window_s": T["probe"],
            "note": "max |q - q(just before the probe)| over each 1 s window, applied on the "
                    "screw dof after the robot left; loosen-way held by the seat-preload "
                    "model (0.7 x the measured seating torque, no floor), tighten-way by "
                    "the seat stop"}
    else:
        backed = float(q_deg[0] - ang_end)
        crit["screw_loosened"] = {
            "pass": bool(backed >= LOOSENED_MIN_DEG and st["released"]
                         and tau_pk > LOOSE_PRELOAD0
                         and z_end >= 0.8 * LEAD / MM * np.radians(backed)),
            "start_deg": round(float(q_deg[0]), 2),
            "angle_after_tool_left_deg": round(ang_end, 2),
            "backed_off_deg": round(backed, 1), "threshold_deg": LOOSENED_MIN_DEG,
            "head_rise_after_tool_left_mm": round(z_end, 3),
            "head_rise_expected_mm": round(LEAD / MM * np.radians(backed), 3),
            "strokes": int(st["stroke"]),
            "insertion_depth_per_stroke_mm": st["ins_depth"],
            "breakaway_preload_mNm": LOOSE_PRELOAD0 * 1000,
            "peak_drive_torque_mNm": round(tau_pk * 1000, 2),
            "preload_released": bool(st["released"]),
            "note": "starts seated with 20 mN.m of breakaway preload (stiction model), which "
                    "the first stroke must break (peak drive torque at the recess > preload); "
                    "the head rises up its thread as it backs out"}
        moves = probe_moves("end")
        crit["rotates_freely_under_small_torque"] = {
            "pass": bool(min(moves.values()) > 5.0), "probe_torque_mNm": PROBE_TAU * 1000,
            "rotation_deg_in_1s": moves, "threshold_deg": 5.0,
            "note": "applied on the screw dof after the robot left, loosen-way then "
                    "tighten-way (as the screwdriver-from-table tasks)"}

    # ── the tool in hand did the turning ────────────────────────────────
    pads_on = float(((L["padL"][turn_m] > 0.5) & (L["padR"][turn_m] > 0.5)).mean()) \
        if turn_m.any() else 0.0
    head_top = Z0 + HEAD_TOP_Z + L["z"]
    out_m = (L["Ez"] > head_top + 0.5 * MM) & (t < t_probe)
    dq_out = np.abs(np.diff(q_deg))[out_m[1:] & out_m[:-1]]
    rot_out = float(dq_out.sum()) if dq_out.size else 0.0
    rs_max = float(L["robot_screw"].max())
    crit["turned_by_the_tool_in_hand"] = {
        "pass": bool(pads_on > 0.98 and tau_pk > 0.003 and rs_max < CONTACT_FREE_N
                     and rot_out < 2.0),
        "both_pads_loaded_fraction_while_turning": round(pads_on, 3),
        "pad_force_min_while_turning_N": [round(float(L["padL"][turn_m].min()), 2),
                                          round(float(L["padR"][turn_m].min()), 2)]
        if turn_m.any() else None,
        "peak_drive_torque_at_recess_mNm": round(tau_pk * 1000, 2),
        "robot_screw_contact_max_whole_rollout_N": round(rs_max, 4),
        "screw_rotation_with_key_out_of_recess_deg": round(rot_out, 3),
        "strokes": st["strokes"], "unwinds": st["unwinds"],
        "note": "the screw only turns through the key end in the recess (the robot never "
                "touches the screw; the screw does not move while the key is out); the key "
                "only moves through the fingertip pads"}

    # ── the key stayed engaged (no cam-out) ─────────────────────────────
    eng = np.isin(ph_names, ["turn", "unwind", "seat_hold"]) & (head_top - L["Ez"] > 2.0 * MM)
    rel = wrap90(np.degrees(L["psi_key"]) - flats_psi_deg(L["q"]))
    radial = np.hypot(L["Ex"] - SCREW_XY[0], L["Ey"] - SCREW_XY[1]) / MM
    lim_deg = 15.0
    crit["no_cam_out"] = {
        "pass": bool(eng.any() and float(np.abs(rel[eng]).max()) < lim_deg
                     and float(radial[eng].max()) < 1.0),
        "key_vs_recess_flats_max_deg": round(float(np.abs(rel[eng]).max()), 2) if eng.any() else None,
        "threshold_deg": lim_deg, "geometric_backlash_deg": BACKLASH_DEG,
        "key_end_off_axis_max_mm": round(float(radial[eng].max()), 3) if eng.any() else None,
        "insertion_depth_min_while_engaged_mm":
            round(float((head_top - L["Ez"])[eng].min() / MM), 2) if eng.any() else None,
        "note": f"while the key end is > 2 mm into the recess and turning / unwinding / "
                f"holding: the key's flats stay within {lim_deg:.0f} deg of the recess flats "
                f"(+-{BACKLASH_DEG} deg of geometric play; 15 deg = 0.20 mm of corner "
                f"interference vs the {CAMOUT_MM} mm a cam-out needs) and its end within 1 mm "
                f"of the screw axis"}

    # ── the key left the screw ───────────────────────────────────────────
    off = t >= ts["up"]
    i_up = int(np.searchsorted(t, ts["up"]))
    rise_k = float((L["Ez"][i_up] - head_top[i_up]) / MM)
    fin_w = t >= ts.get("still", t_end)
    away = float(np.hypot(L["Ex"][fin_w] - SCREW_XY[0], L["Ey"][fin_w] - SCREW_XY[1]).min() / MM)
    crit["key_off_screw"] = {
        "pass": bool(L["kscrew"][off].max() < CONTACT_FREE_N and rise_k > 10.0 and away > 30.0),
        "key_screw_contact_after_extract_max_N": round(float(L["kscrew"][off].max()), 4),
        "key_end_above_head_after_extract_mm": round(rise_k, 1),
        "key_end_distance_from_screw_axis_final_mm": round(away, 1),
        "note": "from the end of the extraction on the key never touches the screw; at "
                "the end it lies on the table away from the screw"}

    # ── put back on the table ───────────────────────────────────────────
    fin = win(ts.get("still", t_end), t_end)
    p0 = np.array([L["key_x"][0], L["key_y"][0]])
    pf = np.array([L["key_x"][fin][-1], L["key_y"][fin][-1]])
    disp = float(np.linalg.norm(pf - p0) / MM)
    comv = float(np.abs(np.diff(L["com_z"][fin])).max() * rig.fps / MM) if fin.sum() > 1 else 0.0
    xyv = float(np.linalg.norm(np.diff(np.c_[L["key_x"][fin], L["key_y"][fin]], axis=0),
                               axis=1).max() * rig.fps / MM) if fin.sum() > 1 else 0.0
    flat = float(L["ky_z"][fin].min())
    lowf = float((L["low_z"][fin] - Z0).min() / MM)
    sup = float(L["table_fz"][fin].mean())
    crit["key_returned_to_table"] = {
        "pass": bool(abs(sup - weight) < 0.25 * weight and flat > 0.97 and abs(lowf) < 1.0
                     and disp < 30.0 and max(comv, xyv) < 2.0
                     and L["block"][fin].max() < 0.01 and L["kscrew"][fin].max() < 0.01
                     and st.get("place_low_mm") is not None and st["place_low_mm"] <= 2.0),
        "set_down_max_mm": 2.0,
        "table_support_final_N": round(sup, 4), "key_weight_N": round(weight, 4),
        "flat_cos_final": round(flat, 4),
        "lowest_point_above_table_mm": round(lowf, 3),
        "distance_from_start_position_mm": round(disp, 1),
        "key_speed_final_mm_s": round(max(comv, xyv), 3),
        "set_down_height_mm": st.get("place_low_mm"),
        "note": "PLACED, not dropped: the pads open with the key's lowest point <= 2 mm "
                "above the table; final window ('still' + probe phases): the table carries "
                "the key's weight, it lies flat (the L's normal within 14 deg of vertical), "
                "within 30 mm of where it was picked up, at rest, touching neither block "
                "nor screw"}

    # ── released and clear; robot stopped ───────────────────────────────
    robot_c = L["robot_key"] + L["robot_screw"]
    i_rel = int(np.searchsorted(t, ts["open"]))
    tcp = np.c_[L["tcp_x"], L["tcp_y"], L["tcp_z"]]
    moved = float(np.linalg.norm(tcp[-1] - tcp[i_rel]))
    check_released_and_clear(qa, t, robot_c, ts.get("still", t_end), extra={
        "tcp_moved_from_release_mm": round(moved / MM, 1), "min_move_mm": 50.0,
        "contact_counts": "robot links incl. fingertips vs key, screw and both block halves"})
    rel_c = qa.report["checks"]["released_and_clear"]
    rel_c["pass"] = bool(rel_c["pass"] and moved >= 0.050)
    crit["released_and_clear"] = rel_c
    d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
    labels = list(d["body_labels"])
    arm_ix = [i for i, l in enumerate(labels) if str(l).startswith("fr3/")]
    m = d["t"] >= ts.get("still", t_end) + T_STILL_SETTLE
    v_arm = float(np.linalg.norm(np.diff(d["body_q"][m][:, arm_ix, :3], axis=0),
                                 axis=-1).max()) * rig.fps if m.sum() > 1 else 0.0
    crit["robot_stopped"] = {"pass": bool(v_arm < 0.005),
                             "max_link_speed_final_window_mm_s": round(v_arm / MM, 3),
                             "threshold_mm_s": 5.0,
                             "window_s": [round(float(ts.get("still", t_end) + T_STILL_SETTLE), 2),
                                          round(t_end, 2)]}

    # ── info: strokes, torque, in-hand grasp ────────────────────────────
    # in-hand twist of the key about its own leg: K_y seen from link7,
    # relative to the pose measured at lifthold
    k7 = np.c_[L["ky7x"], L["ky7y"], L["ky7z"]]
    i_ref = int(np.searchsorted(t, ts["carry"]))
    ref = k7[i_ref] / np.linalg.norm(k7[i_ref])
    tw = np.degrees(np.arccos(np.clip(k7 @ ref / np.linalg.norm(k7, axis=1), -1.0, 1.0)))
    held_m = np.isin(ph_names, list(HELD)) & (t >= ts["carry"])
    spikes = {}
    for k in ("padL", "padR"):
        dev = L[k][1:-1] - 0.5 * (L[k][:-2] + L[k][2:])
        mm_ = held_m[1:-1]
        spikes[k] = {"count": int((np.abs(dev[mm_]) > 1.0).sum()),
                     "max_N": round(float(np.abs(dev[mm_]).max()), 2) if mm_.any() else 0.0}
    qa.report["checks"]["expert_info"] = {
        "pass": True, "info_only": True,
        "strokes": st["strokes"], "unwinds": st["unwinds"],
        "stroke_starts_s": [round(float(r[1]), 2) for r in st.get("t_starts", [])
                            if r[0] == "turn"],
        "key_screw_force_while_turning_mean_N": round(float(L["kscrew"][turn_m].mean()), 2)
        if turn_m.any() else 0.0,
        "thread_axial_load_while_turning_mean_N": round(float(np.abs(L["F_ax"][turn_m]).mean()), 3)
        if turn_m.any() else 0.0,
        "in_hand_twist_max_deg": round(float(tw[held_m].max()), 2) if held_m.any() else None,
        "in_hand_twist_at_release_deg": round(float(tw[min(i_rel, len(tw) - 1)]), 2),
        "in_hand_tcp_in_key_frame_mm": np.round(st["G"][:3, 3] / MM, 2).tolist(),
        "in_hand_pose_log_mm": st["G_log"],
        "grip_trim_final_mm": round(st["trim"] / MM, 3),
        "pad_force_single_frame_jumps_over_1N": spikes,
        "phase_start_s": {k: round(float(v), 2) for k, v in ts.items()}}

    # ── physics integrity ───────────────────────────────────────────────
    c_K = h["c_K"]
    key_loc = rig.meshes["allen_key"][0].copy()
    key_loc.apply_translation(-c_K)
    tip_loc = h["tip"].copy()
    tip_loc.apply_translation(-c_K)
    scr_loc = h["head_w"].copy()
    scr_loc.apply_translation(-h["scr_c"])
    rv = scr_loc.vertices
    rec_pts = rv[(np.abs(rv[:, 0]) < 0.0045) & (np.abs(rv[:, 1]) < 0.0045)
                 & (rv[:, 2] > (RECESS_FLOOR_Z - HEAD_CROP_Z) - 0.0042 - 0.001)]
    # the key end vs the recess, both ways, below the cam-out interference
    qa.check_penetration(
        [("allen_key", tip_loc, "screw", scr_loc),
         ("screw", trimesh.Trimesh(vertices=rec_pts, process=False), "allen_key", key_loc)],
        tol=0.0005, n_samples=240)
    for stem in ("block_top", "block_bottom"):
        tm, c = rig.meshes[stem]
        HP.pen_static_fast(qa, "allen_key", key_loc, tm, c, f"allen_key->{stem}", tol=0.0010)
    pen = qa.report["checks"]["penetration"]
    zmin = float((L["low_z"] - Z0).min())
    pen["pairs"]["allen_key->table_top"] = {"min_vertex_z_above_top_mm": round(zmin / MM, 3),
                                           "pass": bool(zmin > -0.0010)}
    # the pinch: fingertip pad boxes vs the key's leg box, every frame
    t_s, ov = pad_key_overlap(rig.out_dir / "rollout_states.npz", h["box_c"], h["box_h"])
    pad_tol = 0.00075
    for s, nm in enumerate(("fr3_leftfinger_pad->key_leg", "fr3_rightfinger_pad->key_leg")):
        j = int(np.argmax(ov[:, s]))
        pen["pairs"][nm] = {"max_depth_mm": round(float(max(ov[j, s], 0.0)) / MM, 3),
                            "at_t": round(float(t_s[j]), 2),
                            "tolerance_mm": pad_tol / MM,
                            "pass": bool(ov[j, s] < pad_tol)}
    pen["note"] = ("key end <-> recess both ways at 0.5 mm (below the 0.575 mm a cam-out "
                   "needs), key <-> block halves and table top at 1.0 mm, fingertip pads <-> "
                   "the key's leg box at 0.75 mm (SAT depth, every frame)")
    pen["pass"] = bool(all(v.get("pass", True) for v in pen["pairs"].values()))
    qa.check_envelope(["allen_key", "screw"], [-0.25, -0.25, Z0 - 0.01], [0.35, 0.45, Z0 + 0.40])
    for k_, c_ in crit.items():
        qa.report["checks"][k_] = c_
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    changes = [
        "starts with the Allen key lying flat on the table (appendix: in hand); picked up "
        "by a top-down pinch across the long leg, put back where it was at the end",
        "kit parts (assets/bodies_kit/allen_key, screw_block); no sim Allen-key task existed",
        "the long leg drives the screw: the key is tilted 90 deg in free air by the hand "
        "(horizontal hand), and the hand ORBITS the screw axis to turn it; strokes of "
        f"~{STROKE_DEG:.0f} deg, each ended by an unwind of the wind-up, with lift -> "
        "align -> re-insert between them",
        ("start: threads unengaged -- head 4.0 mm above the seat, 615 deg of the 2.34 mm "
         "right-hand thread to go" if mode == "tighten" else
         "start: SEATED with 20 mN.m of breakaway preload (stiction model)"),
        "thread model (as the screwdriver-from-table tasks on the same kit screw): helix "
        "spring between the screw's axial and rotary dofs, load-dependent elasto-plastic "
        "thread friction, hard seat stops, seat preload 0.7 x the drive torque measured at "
        "the recess while seating (no floor, capped 40 mN.m), released once backed off "
        "10 deg; the helix engages from the first degree",
        f"probe: {PROBE_TAU * 1000:.0f} mN.m on the screw after the robot left, loosen-way "
        "then tighten-way",
        "collider: the gripped straight stretch of the long leg (x 9..52 mm) is an analytic "
        "5 x 5 mm box (its 0.5 mm edge fillets dropped) -- mesh-vs-box pad contacts there "
        "produced ~45 deg edge normals that pushed the key out of the pinch; the bend and "
        "the drive end stay meshes, the CAD mesh is the visual",
        "visual only: the whole screw is drawn (the collider is its head); kit colours "
        "(yellow block, grey screw, black key)",
    ]
    return save_appendix_verdict(rig, task=TITLE[mode],
                                 source="new task (no suite counterpart); thread model after "
                                        "appendix_tasks/_screwdriver_table.py",
                                 spec=SPEC[mode], criteria=crit, physics_qa=report,
                                 changes=changes)
