"""Task: FRANKA threads the hex nut on the printed bolt with REGRASP CYCLES
— the real fingertip pads pinch the nut's hex flats, the wrist yaws 160 deg
(the nut rides its thread and visibly descends), the fingers OPEN, the wrist
yaws back, and the cycle repeats: 4 tighten strokes drive the nut into the
head-shoulder seat (fastening force RAMP through the fingers), then 4 loosen
strokes climb it back out.  The kinematic-hand task_nut_bolt.py stays
untouched as ground truth; this file reuses its mesh-measured thread
constants and QA structure.

REGRASP ARCHITECTURE (how a robot does multi-turn without a wrist that spins
forever):
  * stroke = [close fingers on the flats -> wrist yaw sweep 160 deg (the nut
    co-rotates; thread contact carries it down/up its helix) -> open fingers
    (thread contact HOLDS the nut in place) -> yaw back -> repeat].
  * The nut is on a JOINT-FREE spin path (XPBD-era pitfall 9/13): fully FREE body, NO
    grip d6 anywhere — a wrist->nut stabilizer coupling would accumulate
    +160 deg of RELATIVE angle per regrasp cycle and hit the XPBD swing-twist
    +-pi singularity on the second stroke.  Torque flows ONLY through real
    finger<->nut contact (form closure across the 15.5 mm hex flats).  The
    FR3's own joints sweep +-80 deg around home and stay far inside their
    +-3.0159 rad limits (IK verified exact over the window).
  * TCP z FOLLOWS the nut down its helix (closed loop, pitfall 8.1) — but
    the measured height is CONSTRAINED to a band around the helix
    prediction from the measured ROTATION: a raw-height follow is a
    positive-feedback loop through pad stiction that let the friction-
    gripped cap RATCHET over its thread crests.  Rate-capped (pitfall 6.1).
  * TCP xy runs a droop-compensation servo: the dynamic arm leans ~0.4 mm
    under load, which unbalances the pinch (and tilts/jams the part); the
    servo integrates measured-TCP-vs-part-axis error — a STATIC setpoint,
    so it cannot run away (contact-feedback servos did).
  * The wrist yaw advances only while the nut follows (lag limiter): the
    commanded sweep halts when (wrist - nut) lag exceeds LAG_STOP — that is
    the tactile "it stopped turning" stall signal at the seat, and it keeps
    excessive wrist authority from grinding the hex corners through the
    finger pads (pitfall 6.1; slow strokes <= 1.4 rad/s).

GRASP GEOMETRY (mesh-measured):
  * nut: 15.5 mm across flats (17.6 across corners), 7.0 mm tall; finger
    targets close 2.5 mm/side BELOW the flats (real squeeze, force logged
    and QA-gated > 3 N/finger during every stroke).
  * bolt head: r 8.79 mm up to z 7.5 mm; shaft r 4.95 mm to z 27 mm.  TCP
    rides 8.0 mm above the nut center so the pad bottoms (TCP-9.5 mm) stay
    clear of the head at the deepest seat press (floor at TCP0-4.1 mm keeps
    pad bottoms >= 7.9 mm); the shaft (r 4.95) clears the resting pad faces
    (~7.7 mm from axis) between them.
  * start pose: nut center z 13.5 mm — 12.5 mm below the ground-truth mate
    (26.0 mm, 228 deg), baked yaw 98.33 deg by the helix relation; verified
    by signed distance (0.23 mm clearance, +-0.3 mm backlash).  Seat budget:
    the thread run-out cam is at 3.5 mm of descent (~1.5 turns), inside the
    4-stroke (1.78 turn) tighten command.

FASTENING RAMP through the fingers: past LATCH_MM of descent a one-way z
latch engages (TCP tracks only the LOWEST follow height); after the final
tighten stroke the SEAT SERVO winds tightening torque at OMEGA_SEAT while
the SENSED nut->bolt force is below F_SEAT — the screw's mechanical
advantage (lead angle ~4.8 deg) converts finger torque into axial seat
preload.  An axial press through the pads was tried first and transmits
<1 N (pinch friction on vertical flats is the weak link); the torque path
is how humans actually fasten.  A small z press-assist keeps the latch
tight.  Gate: plateau >= 2x running thread force, sustained by the held
wind (the kinematic 10x reference pressed through a rigid z-spring grip;
XPBD thread friction cannot self-lock, so the wind equilibrium sits at
3.5-4 N before creep — measured value reported).

Run: python -m force_bench.tasks.nut_bolt_arm            (full 4+4 strokes)
     TB_STROKES=1,0 python -m force_bench.tasks.nut_bolt_arm   (smoke: 1 stroke)
"""

from __future__ import annotations

import os

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.probes import ForceProbe
# The grasp uses the LAST millimetres of the finger (TCP_OFF keeps the pads
# above the bolt head), which lands on the gel's UN-IMAGED edge: measured
# CoP at finger-length +9.1 mm vs the FOV edge at +7.15 (raster all zero).
# Extending the case 6 mm puts that band at +3.1 mm -- mid-FOV -- and the
# case tip still clears the table by ~1.9 mm at the deepest seat press
# (worst-case bare-tip height 7.9 mm).  setdefault: an explicit env wins.
os.environ.setdefault("GS_CASE_TIP_EXTEND", "6")
# TB_GELSIGHT=1 -> GelSight fingertips (sets the gel solver defaults at
# import, so this must precede task_nut_bolt / task_rig)
import force_bench.tactile.task as GS
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.nut_bolt import (
    load_subdivided,
    smooth,
    LEAD,
    BOLT_WORLD_C,
    MATE_ZC,
    MATE_YAW_DEG,
    SDF_RES,
    SDF_NB,
    SDF_MARGIN,
    GAP,
)
from force_bench.rig import make_solver, Rig

# ── start pose (derived from the ground-truth mate by the helix relation) ─
Z_START = 0.0135          # m — nut center: 12.5 mm below the mate, 3.5 mm of
                          # thread travel above the run-out stop (~1.5 turns)
BAKE_YAW_DEG = (MATE_YAW_DEG + (Z_START - MATE_ZC) / LEAD * 360.0) % 360.0
LATCH_MM = 2.8            # descent [mm] engaging the one-way z latch
                          # (= kinematic SEAT_LATCH 15.3 mm - our 12.5 offset)
FLOOR_DROP = 0.0041       # m — TCP floor below TCP0 (pad bottoms stay off
                          # the bolt head; = kinematic 16.6 mm - 12.5)

# ── regrasp stroke plan ──────────────────────────────────────────────────
YAW0 = np.deg2rad(80.0)   # strokes sweep +-80 deg = 160 deg = 0.444 turns
YAW_BIAS = np.deg2rad(-45.0)
"""Constant offset added to every commanded wrist yaw — i.e. WHERE the
+-YAW0 stroke window sits in the arm's own workspace.  It is 0 for the
bottle cap and -45 deg here, and that number is measured, not chosen:

  * `FrankaIK.solve` at this TCP and yaw = +80 deg (the old start pose)
    returns q7 = -3.015900 rad against a joint limit of -3.0159 — the start
    pose sits EXACTLY ON the wrist's roll stop.  README pitfall 4.7: a body
    that starts on a joint limit is pinned there.
  * Walking the same IK up from +80 deg at the task's own 1.29 deg/frame
    rate, the solution comes apart at +81.3 deg: the TCP residual jumps
    from 0.002 mm to 0.5-1.2 mm and, a few degrees further, to 300-1000 mm
    as the damped-least-squares solver reseeds itself through a
    near-singular configuration (README pitfall 6.4 — and 12 kN.m/rad drives
    chasing that go non-finite).  The loosen strokes command up to
    YAW0 + 0.4 = +102.9 deg, i.e. 21.6 deg PAST the cliff.
  * Downward there is no cliff at all: the same walk tracks to -200 deg
    with a 0.003 mm residual.

So the whole window is moved down.  With the bias the commanded yaw spans
[+35, -125] deg nominally, [+57.9, -147.9] under the lag clip and -239.6
at the seat servo's limit, and a fine IK walk over that whole envelope
([-239.6, +63.9] deg, at the task's own 1.29 deg/frame rate) holds
0.0033 mm of residual with 13.45 deg of margin on EVERY joint limit.
This also bounds the acceptance gate, which anchors its `yank_up_yaw`
phase on the task's own start orientation (pitfall 5.3: the gate replaces
the control callback, so a clip written in `control()` does not exist
there — only the START POSE does)."""
OMEGA = 1.35              # rad/s wrist sweep (slow strokes, pitfall 6.1)
N_TIGHT, N_LOOSE = 4, 4   # 1.78 commanded turns per direction.  The first
                          # loosen stroke releases the held seat preload;
                          # four strokes give a complete reverse rotation.
T_SETTLE, T_CLOSE, T_STROKE, T_OPEN, T_RET = 0.6, 0.45, 2.45, 0.35, 1.9
T_PRESS, T_END = 1.6, 1.0
LAG_STOP = 0.85           # rad — wrist-nut lag = tactile stall (hex grasp
                          # phase slack is at most pi/6 = 0.52)
LAG_TAPER = 0.25          # rad — proportional slowdown band below LAG_STOP

# ── grasp / follow / press ───────────────────────────────────────────────
TCP_OFF = 0.0080          # m — TCP above the nut center (pad-head clearance)
FINGER_OPEN = 0.030       # 60 mm gap: clears the 17.6 mm corners + shaft
FINGER_GRIP = 0.0072      # 14.2 mm gap target = 0.65 mm/side into the
                          # 15.5 mm flats.  WAS 0.0045 (3.33 mm/side), and
                          # that is what broke this task: the nut is
                          # radially captured by its own thread, so the pads
                          # close on something that cannot yield and an XPBD
                          # position drive then delivers whatever force the
                          # interference asks for.  MEASURED on Delta:
                          # 41-47 N per pad on every stroke against an
                          # F_PANIC interlock of 14 N, so `panic` held the
                          # sweep at scale = 0 for the entire rollout
                          # (d_cmd_turns = 0.000 on all 7 strokes, 0.023
                          # turns of a commanded 1.78), and on the run that
                          # did sweep the pinch reached 25-33 N/pad and the
                          # nut's state went NON-FINITE at t = 23.35 s.  The
                          # nominal is now sized at the force band and the
                          # admittance loop below holds it there.
FINGER_SEAT = 0.0066      # press phase: deeper squeeze for the seat torque
                          # (still force-limited by F_GRIP_MAX_SEAT)
FINGER_KE, FINGER_KD = 8000.0, 60.0   # firm pinch (dial-arm proven)
# GelSight variant: the sensing faces sit finger_offset() (3.5 mm) proud of
# the bare tips, so every finger dof sized for bare pads gets that added
# (the admittance band below then regulates the same pad force)
FINGER_OPEN += GS.finger_offset()
FINGER_GRIP += GS.finger_offset()
FINGER_SEAT += GS.finger_offset()
# ── ADMITTANCE GRIP (README pitfall 7.6's cure, pitfall 6.3's warning) ─────
# A position-commanded pinch on a part that cannot yield is a vise, and the
# force it makes is set by interference x a SCENE-dependent stiffness
# (pitfall 6.5) that no gain calculation predicts.  Hold the MEASURED pad
# force in a deadband instead, by moving the finger command both ways, hard
# capped.  Gains and newton-valued gates are one system (pitfall 6.3):
# F_GRIP_MAX must stay below F_PANIC or the interlock fights the servo.
F_GRIP_MAX = 6.5          # N — above this the pads open.  MEASURED
F_GRIP_OK = 3.5           # N — below this they close again.  The band was
                          # [5, 9] N and delivered 4.8-6.3 N/pad with 0.004
                          # slip — a perfectly good grip, but the pinch also
                          # PRESSES THE NUT ONTO ITS OWN THREAD, and that is
                          # the `running_force` the fastening ramp is a ratio
                          # against (2.55 N at that band).  The in-scene grip
                          # ladder (studies/nut_bolt/probe_grip.py) measures 3.24 N of
                          # pad force per mm of finger closure and 0.23 N of
                          # thread load per N of pad, so dropping the band
                          # ~1.5 N drops the denominator ~0.35 N.  The gate
                          # on the grip itself is 3 N/pad, so [3.5, 6.5]
                          # keeps real margin on both sides.
# LOOSEN STROKES GET THEIR OWN, HARDER BAND.  Measured 2026-09-11 (MuJoCo):
# after the seat the nut sits on the bolt shoulder with 0.2-0.6 N of axial
# load and three loosen strokes turned it 0.022 turns in total.  The
# admittance band above is the reason: a hex between flat pads is only
# form-closed while the pads hold their gap, and the moment its corners
# wedge the pads apart the force crosses 6.5 N and the relief valve OPENS
# them -- the valve removes exactly the closure the loosen needs (pad force
# 13 N at the close, 3.5 N two frames into the stroke, part 4 deg for 28 deg
# of wrist).  So while loosening the pads squeeze to the press depth and the
# band sits at 9-12 N, still under the 14 N panic interlock.
F_GRIP_MAX_LOOSEN = 12.0  # N
F_GRIP_OK_LOOSEN = 9.0    # N
F_GRIP_MAX_SEAT = 10.0    # N — the press is allowed a harder pinch (its
                          # torque wind IS the seat preload) but still under
                          # F_PANIC
RELAX_OPEN_MAX = 0.0016   # m — bound on the opening relief per side.  At
                          # the bound the gap is 17.4 mm against the nut's
                          # 17.6 mm across-corners, so the servo can never
                          # open its way off the part (only off the FLATS).
RELAX_CLOSE_MAX = 0.0018  # m — bound on the closing advance per side
RELAX_RATE = 0.00006      # m/frame — 0.06 mm/frame (pitfall 6.1: capped)
DZ_RATE = 0.00025         # m/frame — TCP z follow rate cap (pitfall 6.1)
XY_RATE = 0.0001          # m/frame — TCP droop-compensation servo cap.
                          # The dynamic arm droops/leans under load, so
                          # the ACTUAL grasp axis sits ~0.3-1 mm off the
                          # commanded one; an unbalanced pinch then tilts
                          # the part and (for the friction-gripped cap)
                          # ratchets it over its thread crests.  The servo
                          # integrates the error between the MEASURED TCP
                          # (from link7's scene pose) and the part's STATIC
                          # axis — a fixed setpoint, so it cannot run away
                          # (contact-force-feedback servos did).
XY_GAIN = 0.10            # error fraction per frame (rate-capped)
XY_MAX = 0.006            # m — xy servo authority bound (measured steady
                          # droop at the cap pose saturated a 2.5 mm bound)
F_PANIC = 14.0            # N — finger-force interlock: the IK null space
                          # occasionally migrates the elbow mid-sweep (a
                          # 20 mm elbow swing drags a ~4 mm TCP lurch);
                          # while either pad spikes above this, the stroke
                          # and seat wind PAUSE so the droop servo can
                          # recover instead of grinding through the jam
PRESS_RATE = 0.00004      # m/frame — z press-assist cap (pitfall 6.1)
PRESS_MAX = 0.0015        # m — z press-assist bound (friction transmits
                          # <1 N axially through the pads — the REAL seat
                          # load comes from the torque wind, see below)
F_SEAT = 4.5              # N — seat force-servo target.  Sized against the
                          # MEASURED running thread force, because the ramp
                          # gate is a RATIO and both halves moved when the
                          # grip did (pitfall 6.3).  Three points, all Delta:
                          #   3.5 N target -> plateau 3.89 N / running 2.55 N
                          #                  = 1.53x  (servo reached target
                          #                  and stopped: gate 2.0x missed)
                          #   7.0 N target -> plateau 2.03 N / running 2.36 N
                          #                  = 0.86x  (servo NEVER reached
                          #                  target, wound for the whole
                          #                  press and the load then CREPT
                          #                  out of the friction pinch --
                          #                  peak 5.56 N, plateau 2.03: XPBD
                          #                  friction cannot hold a static
                          #                  load, it has to be RE-FED)
                          #   5.0 N target, wind rate 0.9 rad/s, 2.4 s press
                          #                -> the servo OVER-TIGHTENED: the
                          #                nut jammed so hard that the loosen
                          #                block could not break it free
                          #                (turns_loosen 1.35 -> 0.066, pad
                          #                slip 0.004 -> 0.428, plateau
                          #                0.88 N).  That is correct physics
                          #                and a wrong rollout.
                          # So the seat target goes back to the gentle wind and
                          # the RATIO is fixed from the other side instead: the
                          # running force IS the pinch pressing the nut onto
                          # its own thread, so lowering the admittance band
                          # lowers the denominator.  4.5 N against a ~1.8 N
                          # running force is ~2.5x.
OMEGA_SEAT = 0.5          # rad/s — seat-servo wind rate (slow, pitfall 6.1).
                          # 0.9 rad/s over a 2.4 s press over-tightened; see
                          # F_SEAT above.
SEAT_LAG_BUDGET = 0.9     # rad — extra wind allowed by the seat servo
SEAT_YAW_EXTRA = 0.8      # rad — wrist may pass the stroke window to seat.
                          # With YAW_BIAS the deepest commanded yaw is then
                          # -(YAW0 + 0.8) - 45 = -170.8 deg.  IK-verified to
                          # 2.0 rad (-239.6 deg, 13.45 deg of margin, 0.0033
                          # mm residual) so there is headroom; 2.7 rad does
                          # NOT verify (at -279.7 deg q7 lands exactly on its
                          # stop and the residual jumps to 5.84 mm).
NUT_MU = 0.30


class PairProbe:
    """ForceProbe-compatible probe isolating ONE contact pair.

    SensorContact.total_force sums ALL contacts on the sensing body — here
    the fingers also touch the nut, so the fastening signal must come from
    the per-counterpart force_matrix instead.
    """

    def __init__(self, model, sensing_body, counterpart_bodies=None,
                 counterpart_shapes=None):
        kw = {}
        if counterpart_bodies is not None:
            kw["counterpart_bodies"] = counterpart_bodies
        if counterpart_shapes is not None:
            kw["counterpart_shapes"] = counterpart_shapes
        self.sensor = newton.sensors.SensorContact(
            model, sensing_bodies=[sensing_body], measure_total=False, **kw)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        self.sensor.update(state, contacts)
        F = self.sensor.force_matrix.numpy()[0].sum(axis=0)
        Ff = self.sensor.force_matrix_friction.numpy()[0].sum(axis=0)
        f, fr = float(np.linalg.norm(F)), float(np.linalg.norm(Ff))
        rec = {"t": t, "force": f, "friction": fr, "normal": f - fr,
               "vec": F.tolist()}
        self.history.append(rec)
        return rec


class YawTrack:
    """Unwrapped rotation of a body about world z (its x-axis heading)."""

    def __init__(self):
        self.prev = None
        self.total = 0.0

    def update(self, q_row) -> float:
        M = trimesh.transformations.quaternion_matrix(
            [q_row[6], q_row[3], q_row[4], q_row[5]])
        x = M[:3, 0]
        yaw = float(np.arctan2(x[1], x[0]))
        if self.prev is None:
            self.prev = yaw
        d = (yaw - self.prev + np.pi) % (2 * np.pi) - np.pi
        self.total += d
        self.prev = yaw
        return self.total


def ease(s: float) -> float:
    """Cosine ease 0..1 for open/close/return ramps."""
    s = min(1.0, max(0.0, s))
    return 0.5 - 0.5 * np.cos(np.pi * s)


def build_plan(blocks, t_settle=T_SETTLE, t_stroke=T_STROKE, t_lift=0.0):
    """Segment timeline from stroke BLOCKS: [(dirn, n_strokes, press)].
    Consecutive blocks chain with the fingers CLOSED (a human unscrews
    what they just fastened without letting go), as does a press slotted
    after its block.  t_lift > 0 appends a chained LIFT segment after the
    last block (the part is carried up and aside, never released) instead
    of the final open."""
    seg, t, closed, idx = [], 0.0, False, 0

    def add(kind, dur, **kw):
        nonlocal t
        seg.append(dict(kind=kind, t0=t, t1=t + dur, **kw))
        t += dur

    add("settle", t_settle)
    for dirn, n, press_here in blocks:
        for k in range(n):
            if not closed:
                add("close", T_CLOSE)
                closed = True
            add("stroke", t_stroke, dirn=dirn, idx=idx)
            idx += 1
            if k < n - 1:
                add("open", T_OPEN)
                add("return", T_RET, to=-dirn * YAW0)
                closed = False
        if press_here:
            add("press", T_PRESS)
    if t_lift > 0.0:
        add("lift", t_lift)
    else:
        add("open", T_OPEN)
    add("end", T_END)
    return seg, t


def seg_at(plan, t):
    for s in plan:
        if s["t0"] <= t < s["t1"]:
            return s
    return plan[-1]


class RegraspDriver:
    """Shared regrasp-cycle controller:
    finger open/close scheduling, lag-limited wrist sweeps (the tactile
    stall detector), closed-loop TCP z helix follow (pitfall 8.1) with a
    one-way seat latch + rate-capped force-servo press (pitfall 6.1), and
    per-stroke slip bookkeeping.  The part it drives is a FREE body — all
    torque flows through real finger contact."""

    def __init__(self, rig, plan, tcp0, part_index, wrist_index, part_z0,
                 yaw_init, finger_grip, latch_mm, floor_drop, f_seat,
                 lead_m, axis_xy, finger_seat=None, seat_probe="seat",
                 seat_dir=-1.0, seat_wind=True, diseng_mm=None,
                 cage_mm=None, finger_cage=None,
                 finger_open=FINGER_OPEN, omega=OMEGA, yaw0=YAW0,
                 lag_stop=LAG_STOP, lag_taper=LAG_TAPER, dz_rate=DZ_RATE,
                 press_rate=PRESS_RATE, press_max=PRESS_MAX, end_lift=0.010,
                 xy_max=XY_MAX, helix_band=0.0006, lift_h=0.070,
                 lift_rate=0.0006, aside=(0.0, 0.0), lead_up=0.0003,
                 yaw_bias=0.0, f_grip_max=None, f_grip_ok=None,
                 f_grip_max_seat=None, relax_open_max=0.0035,
                 relax_close_max=0.0, relax_rate=0.00006,
                 finger_loosen=None, f_grip_max_loosen=None,
                 f_grip_ok_loosen=None):
        self.rig = rig
        self.plan = plan
        self.tcp0 = np.asarray(tcp0, dtype=float)
        self.pi, self.wi = part_index, wrist_index
        self.part_z0 = float(part_z0)
        self.p = dict(finger_grip=finger_grip, latch_mm=latch_mm,
                      floor_drop=floor_drop, f_seat=f_seat,
                      finger_seat=(finger_seat if finger_seat is not None
                                   else finger_grip),
                      seat_probe=seat_probe, seat_dir=seat_dir,
                      finger_open=finger_open, omega=omega, yaw0=yaw0,
                      lag_stop=lag_stop, lag_taper=lag_taper,
                      dz_rate=dz_rate, press_rate=press_rate,
                      press_max=press_max, end_lift=end_lift, xy_max=xy_max,
                      lead_m=lead_m, helix_band=helix_band, lift_h=lift_h,
                      lift_rate=lift_rate, aside=np.asarray(aside, float),
                      seat_wind=seat_wind, diseng_mm=diseng_mm,
                      cage_mm=cage_mm, finger_cage=finger_cage,
                      lead_up=lead_up, yaw_bias=float(yaw_bias),
                      f_grip_max=f_grip_max, f_grip_ok=f_grip_ok,
                      f_grip_max_seat=(f_grip_max_seat
                                       if f_grip_max_seat is not None
                                       else f_grip_max),
                      relax_open_max=relax_open_max,
                      relax_close_max=relax_close_max,
                      relax_rate=relax_rate,
                      finger_loosen=finger_loosen,
                      f_grip_max_loosen=f_grip_max_loosen,
                      f_grip_ok_loosen=f_grip_ok_loosen)
        # admittance-grip state; 0.0 (and inert) unless f_grip_max is given
        self._relax = 0.0
        self._settle = {"vmax": 0.0, "body": None, "prev": None,
                        "frames": 0, "finite": True, "seat_F": 0.0}
        self.fl = rig.body_index("fr3/fr3_leftfinger")
        self.fr = rig.body_index("fr3/fr3_rightfinger")
        self.t_press0 = next((s["t0"] for s in plan if s["kind"] == "press"),
                             None)
        self.axis_xy = np.asarray(axis_xy, dtype=float)
        # TCP expressed in link7's frame (link7 pose from the kinematic IK
        # model at the solved start; the scene pose tracks it): lets the
        # servo MEASURE the true grasp-axis xy each frame
        l7 = rig.ik.body_pose("fr3/fr3_link7")
        Ml7 = trimesh.transformations.quaternion_matrix(
            [l7[6], l7[3], l7[4], l7[5]])
        self.tcp_in_l7 = Ml7[:3, :3].T @ (self.tcp0 - l7[:3])
        self.part_yaw = YawTrack()
        self.wrist_yaw = YawTrack()
        self.log = []
        self.strokes = []
        self.ctl = {"yaw": float(yaw_init), "z": float(tcp0[2]),
                    "fingers": finger_open, "latched": False,
                    "z_low": float(tcp0[2]), "press": 0.0, "fastened": False,
                    "t_stop": None, "seg": None, "stroke_ref": None,
                    "ret_from": 0.0, "end_z0": 0.0,
                    "xy": np.zeros(2), "aside": np.zeros(2),
                    "lifted": False, "lift_z0": 0.0, "done_spin": False}

    def control(self, t):
        rig, ctl, p = self.rig, self.ctl, self.p
        bq = rig.s0.body_q.numpy()
        ny = self.part_yaw.update(bq[self.pi])
        wy = self.wrist_yaw.update(bq[self.wi])
        dz = float(bq[self.pi][2] - self.part_z0)
        s = seg_at(self.plan, t)
        kind = s["kind"]
        if s is not ctl["seg"]:
            prev = ctl["seg"]
            if prev is not None and prev["kind"] == "stroke":
                r = ctl["stroke_ref"]
                self.strokes.append({
                    "idx": prev["idx"], "dirn": prev["dirn"],
                    "d_wrist_turns": (wy - r["wy0"]) / (2 * np.pi),
                    "d_part_turns": (ny - r["ny0"]) / (2 * np.pi),
                    "d_cmd_turns": (ctl["yaw"] - r["cmd0"]) / (2 * np.pi),
                    "descent_mm": -(dz - r["dz0"]) * 1000.0,
                })
            ctl["seg"] = s
            if kind == "stroke":
                ctl["stroke_ref"] = {"ny0": ny, "wy0": wy, "dz0": dz,
                                     "cmd0": ctl["yaw"], "t0": t}
                if (s["dirn"] > 0 and prev is not None
                        and prev["kind"] == "press" and ctl.get("press_ref")):
                    # THE FIRST LOOSEN STROKE STARTS WOUND UP.  The seat
                    # servo leaves up to SEAT_LAG_BUDGET of tightening wind
                    # in the wrist; measured from the wound yaw, unwinding
                    # it reads as pure lag, the limiter stops the sweep at
                    # LAG_STOP, and the stroke delivers nothing (2026-09-17:
                    # wrist 0.158 turns, nut 0.011).  Measure the lag from
                    # the pre-press reference instead, so the unwind is
                    # not charged against the budget.  Inert for the cap,
                    # whose press does not wind (seat_wind=False).
                    ctl["stroke_ref"]["cmd0"] = ctl["press_ref"]["cmd0"]
                    ctl["stroke_ref"]["ny0"] = ctl["press_ref"]["ny0"]
                if (ctl.get("helix_ref") is None
                        or s["dirn"] != ctl.get("last_dirn")):
                    # helix reference: measured (rotation, height) at the
                    # start of each direction block — bakes in the settled
                    # backlash and any seat-region deviation
                    ctl["helix_ref"] = {"ny0": ny, "dz0": dz}
                ctl["last_dirn"] = s["dirn"]
                if s["dirn"] > 0:            # loosen begins: release press
                    ctl["latched"] = False
            elif kind == "press":
                ctl["press_ref"] = {"cmd0": ctl["yaw"], "ny0": ny}
            elif kind == "return":
                ctl["ret_from"] = ctl["yaw"]
            elif kind == "lift":
                ctl["lift_z0"] = ctl["z"]
                ctl["lifted"] = True
            elif kind == "end":
                ctl["end_z0"] = ctl["z"]

        # part measured OFF its thread (fully unscrewed): stop spinning —
        # a disengaged part is only lightly keyed by the neck, and further
        # regrasp sweeps can knock it off; hold the grip until the lift
        if (p["diseng_mm"] is not None and kind == "stroke"
                and s["dirn"] > 0 and dz * 1000.0 >= p["diseng_mm"]):
            ctl["done_spin"] = True
        hold = ctl["done_spin"] and kind in ("stroke", "open", "return",
                                             "close")
        # JAM HOLD: once the tactile stall fires mid-tighten, the stored
        # pinch wind IS the fastening preload — an intervening regrasp
        # open would release it, so every segment until the press freezes
        # with the fingers closed
        if (ctl["fastened"] and self.t_press0 is not None
                and t < self.t_press0
                and kind in ("stroke", "open", "return", "close")):
            hold = True

        # ── fingers ──────────────────────────────────────────────────────
        # CAGED regrasps near disengagement: with little thread engagement
        # left, a lateral arm transient during an open regrasp can tilt
        # the part and cam it over its last crests — so above cage_mm of
        # climb the "open" only retreats ~1.5 mm/side and the part stays
        # caged between the pads
        fo, fg = p["finger_open"], p["finger_grip"]
        if (p["cage_mm"] is not None and p["finger_cage"] is not None
                and dz * 1000.0 >= p["cage_mm"]):
            fo = p["finger_cage"]
        if hold:
            ctl["fingers"] = fg
        elif kind == "close":
            a = ease((t - s["t0"]) / (0.85 * (s["t1"] - s["t0"])))
            ctl["fingers"] = min(ctl["fingers"],
                                 fo + (fg - fo) * a)
        elif kind == "open":
            a = ease((t - s["t0"]) / (0.85 * (s["t1"] - s["t0"])))
            ctl["fingers"] = fg + (fo - fg) * a
        elif kind == "press":
            # deep squeeze for the seat torque (a human grips harder for
            # the final tighten), ramped at the finger-close rate
            ctl["fingers"] = max(p["finger_seat"],
                                 ctl["fingers"] - 0.0002)
        elif (kind == "stroke" and s["dirn"] > 0
              and p.get("finger_loosen") is not None):
            # BREAKING A SEATED NUT LOOSE needs the hex corners to stay
            # form-closed between the pads; ramp to the deeper loosen
            # target at the finger-close rate (see F_GRIP_MAX_LOOSEN)
            ctl["fingers"] = max(p["finger_loosen"], ctl["fingers"] - 0.0002)
        elif kind in ("stroke", "lift"):
            ctl["fingers"] = fg
        elif kind == "end" and ctl["lifted"]:
            ctl["fingers"] = fg          # hold the picked part — no release
        else:
            ctl["fingers"] = fo

        # ── TCP droop compensation: measured grasp axis -> part axis ─────
        # (fixed setpoint: the bolt/bottle axis is STATIC; see XY_RATE;
        # frozen during lift/end — the carry deliberately leaves the axis)
        if kind not in ("end", "lift") and p["xy_max"] > 0.0:
            l7 = bq[self.wi]
            Ml7 = trimesh.transformations.quaternion_matrix(
                [l7[6], l7[3], l7[4], l7[5]])
            tcp_meas = l7[:3] + Ml7[:3, :3] @ self.tcp_in_l7
            err = self.axis_xy - tcp_meas[:2]
            ctl["xy"] = np.clip(
                ctl["xy"] + np.clip(XY_GAIN * err, -XY_RATE, XY_RATE),
                -p["xy_max"], p["xy_max"])

        # grip-force panic interlock (see F_PANIC)
        hL, hR = rig.probes["finL"].history, rig.probes["finR"].history
        panic = bool(hL and hR and max(hL[-1]["force"],
                                       hR[-1]["force"]) > F_PANIC)

        # ── wrist yaw: advance only while the part follows ───────────────
        if hold:
            pass                        # yaw frozen until the lift
        elif kind == "stroke":
            r = ctl["stroke_ref"]
            dirn = s["dirn"]
            lag = dirn * (ctl["yaw"] - r["cmd0"]) - dirn * (ny - r["ny0"])
            scale = float(np.clip((p["lag_stop"] - lag) / p["lag_taper"],
                                  0.0, 1.0))
            if (dirn < 0 and ctl["fastened"]) or panic:
                scale = 0.0
            # the sweep may overshoot the nominal window by 0.4 rad so a
            # lag-throttled stroke still delivers its full net rotation
            # (IK verified to +-95 deg and beyond)
            lim = p["yaw0"] + 0.4
            ctl["yaw"] = float(np.clip(
                ctl["yaw"] + dirn * p["omega"] * scale / rig.fps,
                -lim, lim))
            if (dirn < 0 and ctl["latched"] and lag >= p["lag_stop"]
                    and not ctl["fastened"]):
                ctl["fastened"] = True   # tactile stall: it stopped turning
                ctl["t_stop"] = t
        elif kind == "press" and ctl["latched"] and ctl.get("press_ref"):
            # SEAT SERVO — the fastening ramp comes from TORQUE WIND-UP:
            # pad friction transmits <1 N axially (measured), but the
            # screw's mechanical advantage (lead angle ~4.8 deg) converts
            # finger torque into axial preload (~8.5 mN*m -> 5 N).  Wind
            # slowly while the SENSED seat force is below target, bounded
            # by a lag budget and a yaw allowance (pitfall 6.1); the held
            # wind in the pinch sustains the plateau.
            r = ctl["press_ref"]
            dirn = p["seat_dir"]
            h = rig.probes[p["seat_probe"]].history
            F_sm = float(np.mean([q["force"] for q in h[-6:]])) if h else 0.0
            lag_p = dirn * (ctl["yaw"] - r["cmd0"]) - dirn * (ny - r["ny0"])
            if (p["seat_wind"] and F_sm < p["f_seat"]
                    and lag_p < SEAT_LAG_BUDGET and not panic):
                # seat_wind=False for the bottle cap: its shoulder-cone
                # seat FOLLOWS a wind downward (lag never builds, force
                # never ramps) — there the rotation stays frozen and the
                # AXIAL press through the high-mu pads loads the cone
                lim = p.get("seat_yaw_limit", p["yaw0"] + SEAT_YAW_EXTRA)
                ctl["yaw"] = float(np.clip(
                    ctl["yaw"] + dirn * OMEGA_SEAT / rig.fps, -lim, lim))
            if F_sm >= 0.85 * p["f_seat"] and not ctl["fastened"]:
                ctl["fastened"] = True   # tactile stop on the force ramp
                ctl["t_stop"] = t
        elif kind == "return":
            a = ease((t - s["t0"]) / (s["t1"] - s["t0"]))
            ctl["yaw"] = ctl["ret_from"] + (s["to"] - ctl["ret_from"]) * a

        # ── TCP z: follow the measured part helix (pitfall 8.1) ───────────
        # The raw measured height is CONSTRAINED to a band around the
        # helix prediction from the measured ROTATION: a raw-z follow is a
        # positive-feedback loop — pad stiction drags the part axially,
        # the TCP follows the dragged part, and a friction-gripped cap
        # RATCHETS over its thread crests (7.9 mm/turn observed vs the
        # 4.2 mm/turn helix).  Rotation-referenced height cannot ratchet.
        dz_eff = dz
        if ctl.get("helix_ref") is not None:
            hr = ctl["helix_ref"]
            dzh = hr["dz0"] + p["lead_m"] * (ny - hr["ny0"]) / (2 * np.pi)
            if abs(dz - dzh) > 2.5 * p["helix_band"]:
                # part is OFF its helix (seated / jammed / disengaged):
                # re-anchor instead of letting the prediction run away —
                # a runaway prediction once dragged the TCP to the floor
                # and slid the pads off their grip band
                ctl["helix_ref"] = {"ny0": ny, "dz0": dz}
                dzh = dz
            dz_eff = float(np.clip(dz, dzh - p["helix_band"],
                                   dzh + p["helix_band"]))
            # the TCP may LEAD the part upward by at most 0.3 mm: once the
            # part disengages its thread, rotation no longer climbs, and
            # an uncapped helix prediction would drag the free part up
            # off the neck through pad friction before the lift
            # lead_up: how far the TCP may LEAD the part upward along the
            # predicted helix.  Small (0.3 mm) where the grasp is loose,
            # because an uncapped prediction drags a disengaged part off
            # its neck through pad friction; larger where the pads hold
            # firmly and the part would otherwise ride DOWN over its own
            # thread crests instead of climbing (bottle cap run-out).
            dz_eff = min(dz_eff, dz + p["lead_up"])
        z_base = float(self.tcp0[2] + dz_eff)
        # one-way latch may only ENGAGE while actively tightening (a latch
        # condition re-checked during loosen would re-grab at max descent)
        tightening = (kind == "press"
                      or (kind == "stroke" and s["dirn"] < 0))
        if tightening and -dz * 1000.0 >= p["latch_mm"]:
            ctl["latched"] = True
        if ctl["latched"]:
            ctl["z_low"] = min(ctl["z_low"], z_base)
            z_tgt = ctl["z_low"]
        else:
            ctl["z_low"] = z_base
            z_tgt = z_base
        if kind == "press" and ctl["latched"]:
            # small z press-assist (keeps the latch tight; the seat load
            # itself comes from the torque wind above)
            h = rig.probes[p["seat_probe"]].history
            F = h[-1]["force"] if h else 0.0
            if F < p["f_seat"]:
                ctl["press"] = min(ctl["press"] + p["press_rate"],
                                   p["press_max"])
            elif F > 1.3 * p["f_seat"]:
                ctl["press"] = max(ctl["press"] - p["press_rate"], 0.0)
        else:
            ctl["press"] = max(ctl["press"] - 3.0 * p["press_rate"], 0.0)
        z_tgt -= ctl["press"]
        if kind == "lift":
            # PICK: carry the part straight up (own rate cap), drifting
            # aside over the second half; fingers stay closed throughout
            prog = (t - s["t0"]) / (s["t1"] - s["t0"])
            z_tgt = ctl["lift_z0"] + p["lift_h"] * ease(prog)
            ctl["aside"] = p["aside"] * ease((prog - 0.5) / 0.45)
            z_tgt = float(np.clip(z_tgt, ctl["z"] - p["lift_rate"],
                                  ctl["z"] + p["lift_rate"]))
        elif kind == "end":
            if ctl["lifted"]:
                z_tgt = ctl["end_z0"]        # hold the carried part
            else:
                z_tgt = ctl["end_z0"] + p["end_lift"] * ease(
                    (t - s["t0"]) / (s["t1"] - s["t0"]))
        if kind not in ("lift",) and not (kind == "end" and ctl["lifted"]):
            z_tgt = max(z_tgt, float(self.tcp0[2]) - p["floor_drop"])
            z_tgt = float(np.clip(z_tgt, ctl["z"] - p["dz_rate"],
                                  ctl["z"] + p["dz_rate"]))
        ctl["z"] = z_tgt

        # ── admittance grip: hold the MEASURED pad force in a deadband ────
        # (pitfall 7.6's cure, generalized: the finger command may move BOTH
        # ways around its nominal, so it is a force servo and not just a
        # relief valve.  Inert when f_grip_max is None.)
        if p["f_grip_max"] is not None:
            if kind in ("close", "stroke", "press", "lift") or (
                    kind == "end" and ctl["lifted"]):
                loosen = (kind == "stroke" and s["dirn"] > 0
                          and p.get("f_grip_max_loosen") is not None)
                f_hi = (p["f_grip_max_seat"] if kind == "press"
                        else p["f_grip_max_loosen"] if loosen
                        else p["f_grip_max"])
                f_lo = (p["f_grip_ok_loosen"] if loosen
                        and p.get("f_grip_ok_loosen") is not None
                        else p["f_grip_ok"])
                F = max(hL[-1]["force"], hR[-1]["force"]) if (hL and hR) \
                    else 0.0
                if F > f_hi:
                    self._relax = min(self._relax + p["relax_rate"],
                                      p["relax_open_max"])
                elif F < f_lo:
                    self._relax = max(self._relax - p["relax_rate"],
                                      -p["relax_close_max"])
            else:
                # decay toward the nominal while the hand is meant to be open
                self._relax -= float(np.clip(self._relax, -p["relax_rate"],
                                             p["relax_rate"]))

        rig.drive_arm(np.array(
            [self.tcp0[0] + ctl["xy"][0] + ctl["aside"][0],
             self.tcp0[1] + ctl["xy"][1] + ctl["aside"][1], ctl["z"]]),
            down_grasp_quat(ctl["yaw"] + p["yaw_bias"]),
            fingers=float(np.clip(ctl["fingers"] + self._relax, 0.0, 0.04)))
        ctl["_ny"], ctl["_wy"], ctl["_dz"] = ny, wy, dz

    def extra(self, state, t):
        ctl = self.ctl
        s = ctl["seg"]
        # ── START-POSE STATIC CHECK (README pitfall 9.4) ──────────────────
        # "A scripted expert can hide a start pose that is not statically
        # stable" — every first command moves, so nobody asks what happens
        # if it just HOLDS.  The plan already opens with a settle segment;
        # this measures it, so the answer lands in rollout_qa.json instead
        # of only in the acceptance gate.
        bq = state.body_q.numpy()
        if s is not None and s["kind"] == "settle":
            if self._settle["prev"] is not None:
                v = np.linalg.norm(bq[:, :3] - self._settle["prev"],
                                   axis=1) * self.rig.fps
                j = int(np.argmax(v))
                if v[j] > self._settle["vmax"]:
                    self._settle["vmax"] = float(v[j])
                    self._settle["body"] = self.rig.labels[j]
            self._settle["prev"] = bq[:, :3].copy()
            self._settle["frames"] += 1
            self._settle["finite"] &= bool(np.isfinite(bq).all())
            h = self.rig.probes[self.p["seat_probe"]].history
            if h:
                self._settle["seat_F"] = max(self._settle["seat_F"],
                                             h[-1]["force"])
        closed = 1.0 if (s and s["kind"] in ("stroke", "press")) else 0.0
        self.log.append((t, ctl["_ny"] / (2 * np.pi), ctl["_dz"] * 1000.0,
                         ctl["_wy"] / (2 * np.pi), ctl["yaw"],
                         ctl["press"] * 1000.0, closed,
                         1.0 if ctl["latched"] else 0.0))
        return {"part_turns": round(ctl["_ny"] / (2 * np.pi), 3),
                "part_dz_mm": round(ctl["_dz"] * 1000.0, 3),
                "wrist_turns": round(ctl["_wy"] / (2 * np.pi), 3),
                "press_mm": round(ctl["press"] * 1000.0, 3),
                "xy_mm": round(float(np.linalg.norm(ctl["xy"])) * 1000.0, 3),
                "phase": s["kind"] if s else "?"}

    def stroke_stats(self):
        """slip per stroke; held strokes (wrist frozen after the part
        disengaged) get slip=None and are excluded from the gates."""
        for s in self.strokes:
            dw = abs(s["d_wrist_turns"])
            s["slip"] = (1.0 - abs(s["d_part_turns"]) / dw
                         if dw > 0.05 else None)
        tight = [s for s in self.strokes if s["dirn"] < 0]
        loose = [s for s in self.strokes if s["dirn"] > 0]
        return tight, loose


def regrasp_qa(qa, rig, drv, plan, mesh_lead_mm, note, ramp_note,
               ratio_gate=2.0, grip_gate=3.0, turns_gate=1.2,
               slip_gate=0.40, drift_max_climb_mm=None):
    """Shared regrasp QA (both arm threading tasks): rotation per direction
    through finger contact, per-stroke slip, helix pitch vs the mesh lead,
    on-thread hold during regrasp opens, fastening ramp, grip force gate.
    Writes the checks into qa.report and returns the metric dict; the
    caller may add task-specific checks (e.g. bottle seat-touch)."""
    arr = np.array(drv.log)   # t, part_turns, dz_mm, wrist_turns, yaw_cmd,
    #                           press_mm, closed, latched
    t_ = arr[:, 0]
    tight, loose = drv.stroke_stats()
    turns_tight = -sum(s["d_part_turns"] for s in tight)
    turns_loose = sum(s["d_part_turns"] for s in loose)
    # the final tighten stroke ends at the seat by design — its slip is
    # the STALL, not grasp slip; exclude it from the slip gate
    last_tight_idx = max((s["idx"] for s in tight), default=-1)
    slip_pre = [s["slip"] for s in drv.strokes
                if s["idx"] != last_tight_idx and s["slip"] is not None]

    # helix pitch fit over CLOSED, pre-latch travel samples (turn RANGE,
    # not net change — a tighten+loosen round trip nets to ~zero)
    trav = arr[(arr[:, 6] > 0.5) & (arr[:, 7] < 0.5)]
    pitch = 0.0
    if len(trav) > 20 and trav[:, 1].max() - trav[:, 1].min() > 0.4:
        A = np.polyfit(trav[:, 1], trav[:, 2], 1)
        pitch = float(A[0])   # mm per (+)turn; RH thread: positive slope
    pitch_ok = abs(abs(pitch) - mesh_lead_mm) < 0.30 * mesh_lead_mm

    # on-thread during regrasp opens: no unthreading drift
    # drift_max_climb_mm: an "the thread holds it while the fingers are
    # off" gate is only meaningful where the thread CAN hold — above a
    # screw's run-out there is barely any engagement left and a released
    # part legitimately slides back (measured on the bottle cap: 0.04 mm of
    # open drift at +8.0 mm of climb, ~1 mm at +9 mm).  A caller that
    # unscrews all the way to disengagement passes the climb above which
    # opens are excluded, and documents why in its own header.
    drift = []
    excluded = 0
    for s in plan:
        if s["kind"] in ("open", "return"):
            w = arr[(t_ >= s["t0"]) & (t_ <= s["t1"])]
            if len(w) > 1:
                if (drift_max_climb_mm is not None
                        and w[:, 2].max() >= drift_max_climb_mm):
                    excluded += 1
                    continue
                drift.append((abs(w[-1, 1] - w[0, 1]),
                              abs(w[-1, 2] - w[0, 2])))
    max_dturn = max(d[0] for d in drift) if drift else 0.0
    max_ddz = max(d[1] for d in drift) if drift else 0.0
    on_thread_ok = max_dturn < 0.06 and max_ddz < 0.6

    # fastening ramp: running force during engaged travel vs seat plateau
    F_seat = np.array([r["force"] for r in rig.probes["seat"].history])
    tF = np.array([r["t"] for r in rig.probes["seat"].history])
    run_w = (arr[:, 6] > 0.5) & (arr[:, 7] < 0.5)
    running_F = float(np.median(F_seat[run_w[:len(F_seat)]])) \
        if run_w.any() else 0.0
    press_seg = [s for s in plan if s["kind"] == "press"][0]
    strk = [s for s in plan if s["kind"] == "stroke"]
    tight_segs = [s for s in strk if s["dirn"] < 0]
    last_tight_seg = tight_segs[-1] if tight_segs else press_seg
    pw = (tF >= press_seg["t1"] - 0.8) & (tF <= press_seg["t1"])
    plateau_F = float(np.median(F_seat[pw])) if pw.any() else 0.0
    pw_all = (tF >= last_tight_seg["t0"]) & (tF <= press_seg["t1"])
    peak_F = float(smooth(F_seat[pw_all], 9).max()) if pw_all.any() else 0.0
    # A RATIO NEEDS A DENOMINATOR.  The gate's meaning is "the seat load is
    # distinguishable from the load the thread carries while merely
    # running", and a grasp good enough to ride the helix without pressing
    # the part onto its own thread makes that running load EXACTLY ZERO --
    # which is the best possible outcome and used to score 0.00 and fail.
    # (Measured on a bottle cap after its grip was fixed: running 0.000 N,
    # plateau 2.14 N, ratio reported as 0.0.)  Below the seat probe's own
    # quantisation the denominator is floored and the report says so.
    F_NOISE = 0.05           # N — seat-probe quantisation
    running_eff = max(running_F, F_NOISE)
    ratio = plateau_F / running_eff
    ramp_ok = ratio >= ratio_gate
    descent_total = float(-arr[:, 2].min())

    # grip force through every stroke
    grip = {}
    for pname in ("finL", "finR"):
        h = rig.probes[pname].history
        vals = []
        for s in strk:
            f = np.array([r["force"] for r in h
                          if s["t0"] + 0.1 <= r["t"] <= s["t1"]])
            if len(f):
                vals.append({"mean": float(f.mean()),
                             "frac": float((f > 0.5).mean())})
        grip[pname] = vals
    grip_min_mean = min(v["mean"] for pn in grip for v in grip[pn])
    grip_ok = grip_min_mean > grip_gate and all(
        v["frac"] > 0.9 for pn in grip for v in grip[pn])

    turns_ok = turns_tight >= turns_gate and turns_loose >= turns_gate
    slip_ok = (np.mean(slip_pre) if slip_pre else 1.0) <= slip_gate
    ok = turns_ok and pitch_ok and on_thread_ok and ramp_ok and grip_ok \
        and slip_ok

    ctl = drv.ctl
    qa.report["checks"]["task_success"] = {
        "pass": bool(ok),
        "strokes_tighten": len(tight), "strokes_loosen": len(loose),
        "turns_tighten": round(turns_tight, 3),
        "turns_loosen": round(turns_loose, 3),
        "part_descent_max_mm": round(descent_total, 2),
        "measured_pitch_mm_per_turn": round(abs(pitch), 3),
        "mesh_helix_lead_mm_per_turn": round(mesh_lead_mm, 3),
        "per_stroke": [{k: (round(v, 4) if isinstance(v, float) else v)
                        for k, v in s.items()} for s in drv.strokes],
        "mean_slip_ratio_pre_stall": round(float(np.mean(slip_pre)), 4)
        if slip_pre else None,
        "note": note,
    }
    qa.report["checks"]["regrasp_on_thread"] = {
        "pass": bool(on_thread_ok),
        "max_open_drift_turns": round(max_dturn, 4),
        "max_open_drift_mm": round(max_ddz, 3),
        "opens_excluded_above_climb_mm": drift_max_climb_mm,
        "opens_excluded": excluded,
        "note": "thread contact must hold the part while the fingers are off",
    }
    qa.report["checks"]["fastening_signal"] = {
        "pass": bool(ramp_ok),
        "running_force_N": round(running_F, 3),
        "running_force_floored": bool(running_F < F_NOISE),
        "peak_seat_force_N": round(peak_F, 2),
        "plateau_force_N": round(plateau_F, 2),
        "seating_force_ratio": round(ratio, 2),
        "t_tactile_stall": round(ctl["t_stop"], 2) if ctl["t_stop"] else None,
        "press_mm_end": round(float(arr[-1, 5]), 3),
        "note": ramp_note,
    }
    qa.report["checks"]["finger_grasp_real"] = {
        "pass": bool(grip_ok),
        "min_stroke_mean_N": round(grip_min_mean, 2),
        "max_stroke_mean_N": round(
            max(v["mean"] for pn in grip for v in grip[pn]), 2),
        "finL_per_stroke_N": [round(v["mean"], 2) for v in grip["finL"]],
        "finR_per_stroke_N": [round(v["mean"], 2) for v in grip["finR"]],
        "note": "fingertip pads must squeeze the part through every stroke "
                "(honest grasp: NO wrist->part coupling, TB_STABILIZER "
                "irrelevant here — this task never builds one)",
    }
    # README pitfall 9.4: hold the start pose and say so in the report.
    st = drv._settle
    settle_ok = bool(st["finite"] and st["frames"] > 10
                     and st["vmax"] < 0.10)
    qa.report["checks"]["start_pose_static"] = {
        "pass": settle_ok,
        "hold_frames": st["frames"],
        "peak_displacement_velocity_m_s": round(st["vmax"], 4),
        "worst_body": st["body"],
        "peak_seat_force_N": round(st["seat_F"], 3),
        "finite": bool(st["finite"]),
        "note": "the scene must be statically stable while the expert "
                "merely HOLDS its start pose (README pitfall 9.4) — the "
                "cheapest scene-validity check there is",
    }
    ok = ok and settle_ok
    qa.report["pass"] &= ok
    return dict(ok=ok, turns_tighten=turns_tight, turns_loosen=turns_loose,
                pitch_mm=pitch, running_F=running_F, plateau_F=plateau_F,
                peak_F=peak_F, ratio=ratio, grip_min_mean=grip_min_mean,
                mean_slip=float(np.mean(slip_pre)) if slip_pre else None,
                descent_mm=descent_total, final_dz_mm=float(arr[-1, 2]))


def main():
    n_tight, n_loose = N_TIGHT, N_LOOSE
    if os.environ.get("TB_STROKES"):
        n_tight, n_loose = map(int, os.environ["TB_STROKES"].split(","))

    rig = GS.make_rig("nut_bolt_arm")
    base_xf = wp.transform(wp.vec3(0.0, 0.325, 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    GS.mount(rig)          # GelSight pads (no-op unless TB_GELSIGHT=1)
    rig.table(center_xy=(0.0, 0.16), half=0.35)

    # ── bolt: STATIC, shaft-up at the world origin ───────────────────────
    bolt_tm = load_subdivided("fasteners", "screw_075mm_1_body1")
    from force_bench.assets import make_newton_mesh, set_body_mass
    bolt_mesh, bolt_c = make_newton_mesh(
        bolt_tm, sdf=True, sdf_resolution=SDF_RES,
        narrow_band=SDF_NB, margin=SDF_MARGIN)
    bolt_sid = rig.builder.add_shape_mesh(
        -1, mesh=bolt_mesh,
        xform=wp.transform(wp.vec3(*BOLT_WORLD_C), wp.quat_identity()),
        cfg=rig.cfg(mu=NUT_MU, gap=GAP), label="bolt")
    rig.shapes["bolt"] = [bolt_sid]
    bolt_world_tm = bolt_tm.copy()
    bolt_world_tm.apply_translation(BOLT_WORLD_C - np.asarray(bolt_c))
    rig.meshes["bolt"] = (bolt_world_tm, BOLT_WORLD_C)

    # ── nut: fully FREE body, mating yaw baked into the mesh ─────────────
    nut_tm = load_subdivided("fasteners", "nut_075mm_1_body1")
    nut_c_raw = (nut_tm.bounds[0] + nut_tm.bounds[1]) / 2
    nut_tm.apply_translation(-nut_c_raw)
    nut_tm.apply_transform(trimesh.transformations.rotation_matrix(
        np.radians(BAKE_YAW_DEG), [0, 0, 1]))
    nut_mesh, nut_c = make_newton_mesh(
        nut_tm, sdf=True, sdf_resolution=SDF_RES,
        narrow_band=SDF_NB, margin=SDF_MARGIN)
    nut_pos = np.array([0.0, 0.0, Z_START])
    nut_body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*nut_pos), wp.quat_identity()), label="nut")
    nut_sid = rig.builder.add_shape_mesh(nut_body, mesh=nut_mesh,
                                         cfg=rig.cfg(mu=NUT_MU, gap=GAP),
                                         label="nut_shape")
    rig.shapes["nut"] = [nut_sid]
    GS.hydroelastic(rig, "nut")   # the gels must feel the hex flats
    set_body_mass(rig.builder, nut_body, 0.01)
    nut_world_tm = nut_tm.copy()
    nut_world_tm.apply_translation(nut_pos - np.asarray(nut_c))
    rig.meshes["nut"] = (nut_world_tm, nut_pos)

    # ── arm start: fingers open, straddling the nut, wrist at +YAW0 ──────
    # (the yaw is BIASED: +80 deg puts the wrist roll exactly on its
    # -3.0159 rad joint limit and 1.3 deg under the IK's tracking cliff —
    # see YAW_BIAS.  The gate anchors its abuse on this same start pose.)
    tcp0 = np.array([0.0, 0.0, Z_START + TCP_OFF])
    q_start = rig.solve_arm_start(tcp0, down_grasp_quat(YAW0 + YAW_BIAS))
    # START THE FINGERS WHERE THE FIRST COMMAND WILL PUT THEM (pitfall 7.1's
    # corollary).  `solve_arm_start` writes the IK's own 40 mm finger coords,
    # and the expert's very first frame asks for FINGER_OPEN = 30 mm — a
    # 10 mm step at t = 0 that snaps the fingertips at 0.109 m/s and makes
    # the pitfall-31 "hold the start pose" check fail on a command artifact
    # rather than on the scene.  A start pose is only static if it is also
    # the commanded pose.
    rig.builder.joint_q[7:9] = [FINGER_OPEN, FINGER_OPEN]
    rig.builder.joint_target_q[7:9] = [FINGER_OPEN, FINGER_OPEN]
    lo = rig.ik.model.joint_limit_lower.numpy()[:7]
    hi = rig.ik.model.joint_limit_upper.numpy()[:7]
    marg = np.degrees(np.minimum(q_start[:7] - lo, hi - q_start[:7]))
    print(f"start pose: q = {np.round(q_start[:7], 4).tolist()}")
    print(f"  joint-limit margin (deg) = {np.round(marg, 2).tolist()}"
          f"  min {marg.min():.2f} deg on q{int(np.argmin(marg)) + 1}")
    assert marg.min() > 3.0, (
        f"start pose is {marg.min():.2f} deg from a joint limit "
        f"(q{int(np.argmin(marg)) + 1}) — README pitfall 4.7")
    # NO wrist->nut coupling of any kind (see header) — contact only.

    cam_c = np.array([0.0, 0.0, 0.017])
    # the "nut" probe requests the contact force attribute BEFORE finalize
    # creates the Contacts buffer; it is then REPLACED by pair-resolved
    # probes (total_force would mix finger grip into the fastening signal)
    rig.finalize(probes=["nut"],
                 cam_pos=cam_c + np.array([0.07, -0.10, 0.055]),
                 cam_target=cam_c)
    rig.probes = {
        "seat": PairProbe(rig.model, "nut", counterpart_shapes=["bolt"]),
        "finL": ForceProbe(rig.model, sensing_body="fr3/fr3_leftfinger",
                           counterpart_bodies=["nut"]),
        "finR": ForceProbe(rig.model, sensing_body="fr3/fr3_rightfinger",
                           counterpart_bodies=["nut"]),
    }
    # contact convergence (ethernet discovery; matches task_nut_bolt): 10
    # XPBD iterations left ~0.5 mm of contact compliance in the thread
    # under the seat wind (audit: 0.71 mm nut<->bolt depth); 20 iterations
    # + the 0.1 mm SDF margin (imported) hold it <= 0.3 mm
    rig.solver = make_solver(rig.model, iterations=20)
    ni = rig.body_index("nut")
    wi = rig.body_index("fr3/fr3_link7")

    plan, T_TOTAL = build_plan([(-1.0, n_tight, True), (+1.0, n_loose, False)])
    print("plan:", [(s["kind"], round(s["t0"], 2)) for s in plan],
          "total", round(T_TOTAL, 2))

    drv = RegraspDriver(rig, plan, tcp0, ni, wi, part_z0=float(nut_pos[2]),
                        yaw_init=YAW0, finger_grip=FINGER_GRIP,
                        finger_seat=FINGER_SEAT, latch_mm=LATCH_MM,
                        floor_drop=FLOOR_DROP, f_seat=F_SEAT, lead_m=LEAD,
                        axis_xy=(0.0, 0.0), yaw_bias=YAW_BIAS,
                        f_grip_max=F_GRIP_MAX, f_grip_ok=F_GRIP_OK,
                        f_grip_max_seat=F_GRIP_MAX_SEAT,
                        relax_open_max=RELAX_OPEN_MAX,
                        relax_close_max=RELAX_CLOSE_MAX,
                        relax_rate=RELAX_RATE,
                        finger_loosen=FINGER_SEAT,
                        f_grip_max_loosen=F_GRIP_MAX_LOOSEN,
                        f_grip_ok_loosen=F_GRIP_OK_LOOSEN)

    rec = rig.run(T_TOTAL, drv.control, drv.extra)
    paths = rec.save("rollout", force_keys=["seat_N", "finL_N", "finR_N"])
    print("saved:", paths)

    # ── snapshot frames: the regrasp cycle must be VISIBLE ───────────────
    import imageio.v2 as imageio
    strk = [s for s in plan if s["kind"] == "stroke"]
    opens = [s for s in plan if s["kind"] == "open"]
    rets = [s for s in plan if s["kind"] == "return"]
    press = [s for s in plan if s["kind"] == "press"]
    snap_t = {"grip1": strk[0]["t0"] + 0.05,
              "stroke1_mid": 0.5 * (strk[0]["t0"] + strk[0]["t1"]),
              "stroke1_end": strk[0]["t1"] - 0.05}
    if opens:
        snap_t["open1"] = opens[0]["t1"] - 0.05
    if rets:
        snap_t["return1_mid"] = 0.5 * (rets[0]["t0"] + rets[0]["t1"])
    if press:
        snap_t["press_end"] = press[0]["t1"] - 0.05
    if len(strk) > n_tight:
        sl = strk[n_tight]
        snap_t["loosen1_mid"] = 0.5 * (sl["t0"] + sl["t1"])
    snap_t["end"] = T_TOTAL - 0.05
    times = np.array(rec.times)
    tiles = []
    for name, ts in snap_t.items():
        fi = int(np.argmin(np.abs(times - ts)))
        imageio.imwrite(rig.out_dir / f"frame_{name}.png", rec.frames[fi])
        tiles.append(rec.frames[fi])
    imageio.imwrite(rig.out_dir / "frames_check.png",
                    np.concatenate(tiles[:4], axis=1))
    if len(tiles) > 4:
        imageio.imwrite(rig.out_dir / "frames_check2.png",
                        np.concatenate(tiles[4:8], axis=1))

    # ── QA (shared regrasp checks) ───────────────────────────────────────
    qa = rig.qa(paths)
    m = regrasp_qa(
        qa, rig, drv, plan, mesh_lead_mm=LEAD * 1000.0,
        note="robot regrasp cycles: nut driven ONLY through fingertip "
             "contact on the hex flats; joint-free thread path",
        ramp_note="gate: plateau >= 2x running thread force — seat load "
                  "built by TORQUE WIND-UP through the pinch (screw "
                  "mechanical advantage); kinematic 10x reference used a "
                  "rigid-grip axial press")
    print("regrasp metrics:", {k: v for k, v in m.items()
                               if not isinstance(v, list)})

    nut_world, nut_c0 = rig.meshes["nut"]
    nut_local = nut_world.copy()
    nut_local.apply_translation(-np.asarray(nut_c0))
    bolt_world, bolt_c0 = rig.meshes["bolt"]
    qa.check_penetration_static("nut", nut_local, bolt_world, bolt_c0,
                                pair_name="nut->bolt", tol=0.0015)
    # tactile stacks + gel gates (no-op unless TB_GELSIGHT=1).  The pads
    # open on every return, so the loaded-window gate is the FIRST STROKE;
    # the cross-check uses the finL/finR finger<->nut probes the servo uses
    GS.save_tactile(
        rig, [GS.object_spec(rig, "nut")], qa=qa,
        window=(strk[0]["t0"] + 0.1, strk[0]["t1"] - 0.1),
        probe_forces={"gel_left": [r["force"] for r in rig.probes["finL"].history],
                      "gel_right": [r["force"] for r in rig.probes["finR"].history]})
    rig.finish(qa)


if __name__ == "__main__":
    wp.init()
    main()
