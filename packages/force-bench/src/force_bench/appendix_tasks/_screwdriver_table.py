"""Screwdriver tighten / loosen, tool picked up from the table and put back
(appendix Table I #15 / #16 + the user's additions).

    Tighten. Description: "A screw is placed into the screw slot of an
    assembly block in the workspace, but the threads are unengaged. Use the
    screwdriver grasped in hand to tighten the screw."  Success: "The screw
    is tightened and remains stationary when a small force is applied in an
    attempt to rotate the screw. The robot movement stops after terminating
    the tightening motion, moving away from, and breaking contact with the
    screw."
    Loosen. "... and is fully tightened. Use the screwdriver grasped in hand
    to loosen the screw."  Success: "The screw is loosened and can rotate
    freely when a small force is applied ..." (same ending).
    User additions: the screwdriver starts LYING ON THE TABLE beside the
    block; the robot picks it up, does the task, puts it back on the table,
    releases it and moves away.

This is a NEW scene: the screwdriver is a FREE body that only the
fingertips ever move (in ``tasks/screwdriver``, the scene module its
drive is calibrated in, it rides an undriven 2-dof carrier).  Filled in from the run
logs at the end of this docstring: what the expert does, what was measured.

ASSETS (finalized kit, what the reference videos show)
------------------------------------------------------
* block: ``assets/bodies_kit/screw_block/block_{top,bottom}.stl`` (yellow
  toy block, 80 x 30 x 20 mm, two 10.5 mm holes at x = +-25 mm), STATIC
  scenery: a 52 g block on a table under a <= 10 N press and <= 0.1 N.m of
  drive torque does not move (mu 0.5 x 10 N = 5 N lateral, ~0.1 N.m of
  friction torque about the screw); clamping it keeps the screw axis fixed.
* screw: ``screw_6_5.stl`` -- 15 mm round head (z 20..27 mm) with a 5.5 x
  5.5 mm SQUARE recess 4.0 mm deep (entry chamfer 7.3 -> 5.5 mm over
  ~0.6 mm); shank z 3.5..20 mm.  MEASURED THREAD: crests every 2.34 mm, the
  crest at 90 deg sits 0.59 mm above the one at 0 deg -> a RIGHT-hand thread
  of 2.34 mm pitch (tighten = clockwise seen from above).
* screwdriver: ``assets/bodies/screwdriver/part2_1_body1`` (the suite mesh;
  identical solid to ``bodies_kit/screwdriver_kit/screwdriver.stl``: 10.4996
  vs 10.5002 cm^3) so ``tasks.screwdriver.split_driver`` applies unchanged:
  the 20 mm tip carries its own SDF (0.04 mm cells -- the whole tool at res
  512 resolves the 0.25 mm drive clearance in ONE cell).  Measured
  sections: 5.0 mm square shank 1..48 mm from the tip (pyramidal lead-in
  3.6 -> 5.0 mm over the first 1 mm), collar 48..65 mm, and a HANDLE
  70..125 mm that is a ROUNDED SQUARE, 12.0 mm across flats (r 6.0 on the
  flats, 7.2 at the diagonals, ~3.1 mm corner radius) -- not a round
  handle, so it lies on a flat without rolling and the pads grip two flats.
  The tip flats are parallel to the handle flats.  PLA (1240 kg/m^3 ->
  13.0 g), centre of mass 82.4 mm from the tip.

THE SCREW MECHANISM (reduced model, all of it in ``ThreadModel``)
-----------------------------------------------------------------
world -prismatic z-> carrier -revolute z-> screw.  Both dofs are free
joints in the solver; the THREAD between them is a per-substep generalised
force pair written to ``Control.joint_f`` (the ``_screwdriver.Stiction``
solver-wrapper pattern):
  * helix: E = 1/2 k (z - L q)^2, L = pitch / 2 pi = 0.372 mm/rad, so the
    head sinks 2.34 mm per clockwise turn and an axial press reacts as
    (k e L) of torque -- a frictionless screw pair;
  * thread friction: elasto-plastic stiction on q (bristle 2 N.m/rad)
    capped at TAU_RUN0 + MU_RUN * |axial load|, i.e. load-dependent running
    friction, MU_RUN (1.5 mN.m/N) >> L (0.37 mN.m/N) so the press cannot
    back-drive the thread (a real 5 deg-lead thread is self-locking);
  * seat: q = 0 is the HARD lower stop of the revolute (Rig.hard_joint_limits;
    a soft MuJoCo limit overshoots 9-18 deg, _screwdriver.py) and z = 0 the
    hard lower stop of the carrier (head on the block top);
  * seat preload: once seated, the stiction cap is raised to a breakaway of
    PRELOAD_FRAC x the drive torque MEASURED at the tip while seating
    (clamped 8..40 mN.m, as in _screwdriver.py); it is released once the
    screw is backed off LOOSE_FREE_DEG.  A loosen run starts seated with
    LOOSE_PRELOAD0;
  * probe: a small torque (PROBE_TAU) on q once the robot has gone.
"THREADS UNENGAGED": the CAD's captive nut spans z 0.5..7.5 mm and the
seated shank tip is at z 3.5, so a screw resting on the nut is 4.0 mm
higher = 1.71 turns (615 deg) short of the seat.  The tighten run starts
there.  The model engages the helix from the first degree (no free-spin
before the thread catches) -- a stated simplification.
The screw inertia is the suite's 20 g screw (the printed one is 2.5 g); the
explicit stiction bristle needs k dt^2 / I < 4.

GRASP PLAN: ONE GRASP FROM PICK TO PUT-BACK
-------------------------------------------
A parallel gripper cannot pick a tool lying flat and hold it tip-down in a
top-down grasp: the top-down pinch across the handle (closing axis
horizontal, perpendicular to the tool), pitched 90 deg about the closing
axis, gives a SIDE grasp (fingers horizontal, tool vertical).  The expert
keeps that grasp for the whole task:
  1. pick: top-down pinch on the two side flats of the handle 90 mm from
     the tip, fingertips ~2 mm above the table (the TCP is then 6 mm above
     the handle axis); lift; pitch 90 deg about the closing axis;
  2. carry over the screw, turning the hand about the vertical tool axis
     until the tip flats are congruent (mod 90 deg) with the MEASURED recess;
     an integral servo on the measured tool pose (rotation about the
     measured tip + a rate-limited tip-position step) aligns and lowers it;
     insert with a force servo (lands at > 0.6 N, presses 6 N, done when the
     tip is >= 3 mm into the 4 mm recess and the tool within 1 deg of
     vertical);
  3. drive: each stroke turns the recorded hand pose about the SCREW axis
     (the tool turns about its own axis wherever the grasp holds it) with a
     force-servoed 6 N press, from the current approach azimuth to the edge
     of the IK window [W_LO, W_HI] = [35, 145] deg (>= 0.35 rad of joint
     margin along the whole orbit, warm-started IK); then the press comes
     off, the grasp unwinds 4 deg inside the tip's +-6 deg backlash (lifting
     a tip that still carries the stroke torque drags the screw up its
     thread: 41 N and the head pulled 0.6 mm up, measured), the tool is
     lifted 12 mm out of the recess IN THE GRIP, swung back 90 deg in the air
     about its own axis (a multiple of 90 deg, so its flats stay congruent)
     and re-inserted.  The first stroke absorbs the arbitrary recess phase,
     after that every stroke is ~90 deg (94 deg of wrist with the re-
     alignment).  Tighten creeps onto the seat (15 deg/s) and snugs to
     40 mN.m at the tip;
  4. unwind, lift the tip out, carry back, pitch the tool flat -- the hand
     poses come from the MEASURED in-hand transform, so the tool, not the
     hand, retraces the pick -- lower until the tool's lowest mesh point is
     ~0.5 mm off the table, open, retreat 10 cm, hold still; then the probe
     torque.
The joint tracker under the IK must not trail a moving target: its
deceleration-limited approach (v <= sqrt(2 a |err|)) lags v^2/2a, ~20 mm
at the TCP at 1 rad/s (measured: the first in-air swing wandered 9 mm off
the tool axis, the tip dipped onto the head and dragged the screw back
27 deg), so it is used only for genuine IK jumps.
TWO REGRASP DESIGNS WERE TRIED FIRST AND FAILED, both measured:
  * leaving the tool standing in the recess (open cage) while the hand
    turns to a top-down grasp: on its 3.6 mm tip face the tool rocks, the
    rocking grows, and it falls onto the recess rim at 16 deg within ~1 s
    with nothing touching it (tool contact force = its own weight);
  * rotating the hand around the tool with a light pinch (1.6 N/pad): the
    pads' rotational slip hands the tool a friction torque that only the
    recess can react -- with the hand held still the tool leans to a 7-9 deg
    jam and the handle slides to the pad edge (12 mm off-axis; the next
    stroke lost it); with the hand following the tool, the tip levers out
    of the 4 mm recess and the tool turns over with the hand (78 deg).
IK (FrankaIK, warm-started along the whole path, base at the origin): the
side grasp only exists with the fingers pointing roughly +y (pointing away
from the base puts joint 6 on its 4.52 rad stop), which is why the tool
lies along +y with its tip at -y.

RESULTS (publication runs, 2026-10-16, outputs/appendix_tasks/<name>/; both PASS)
--------------------------------------------------------------------------------
                               tighten_..._from_table   loosen_..._from_table
  rollout                      76.7 s                   48.0 s
  table support at the start   0.128 N (tool 0.128 N)   0.128 N
  pads through the carry       11.4 / 11.4 N            11.4 / 11.4 N
  lowest tool point lifted     98.4 mm, 0 N table       98.4 mm, 0 N table
  screw                        615.4 deg -> -0.01 deg   0.0 -> 301.0 deg
                               (seat), head 3.99 ->     (head up 1.95 mm),
                               -0.01 mm                 preload released
  strokes (wrist)              -60, 6 x -94, -27 (creep) +35, +95, +94, +94
  screw per stroke             88-89 deg / 94 deg       30 / 88 / 88 / 88 deg
  insertions                   8, all first try,        4, all first try,
                               4.01-4.02 mm @ 5.8-5.95 N 4.01-4.02 mm @ 5.7-5.8 N
  tip torque                   seating 51.1 mN.m (70.0   breakaway 21.8 mN.m
                               in the snug hold) ->     (preload 20)
                               preload 35.8 mN.m
  probe +-5 mN.m, 1 s          0.08 / 0.18 deg (< 1)    83.1 / 89.3 deg (> 5)
  tool back on the table       1.5 mm from where it lay, flat (90.0 deg), at
                               rest (< 0.01 mm/s), support = weight, 0 N block
  released and clear           0.0 N robot<->{tool, screw, block} over the
                               last 3 s, TCP 100 mm away, links < 1.5 mm/s
  integrity                    finite; peak displacement speed 0.32 m/s (link7);
                               tip<->recess 0.20/0.15 mm (tighten), 0.22/0.18 mm
                               (loosen) of 1.5 allowed; tool<->block 0;
                               tool below table top 0.013 mm; envelope ok
Running thread torque at the tip: ~5 mN.m tightening (the 6 N press back-
drives the helix a little), ~9.5 mN.m loosening (the press opposes the head
rising); the thread friction cap at 6 N is ~10 mN.m.

GelSight readiness: every finger target is a named constant + GS.finger_
offset(); the grip is force-servoed on the pad PairProbes; the approach
corridors leave >= 8 mm per side around the handle; the side grasp keeps the
fingers (and the GelSight housings behind them) >= 85 mm above the block.
"""

from __future__ import annotations

import os

import numpy as np
import trimesh
import warp as wp

import force_bench.tactile.task as GS     # before force_bench.rig (GelSight opt-in)
import newton

from force_bench.assets import load_tm, make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv, world_T
from force_bench.paths import ASSETS_ROOT
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.screwdriver import split_driver
from force_bench.tasks.screwdriver import subdiv as subdiv_p
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, StopRun, CONTACT_FREE_N, save_appendix_verdict)
from force_bench.appendix_tasks._hang_plate import (
    PairProbe, quat_of_R, R_of_q, smooth5, tcp_offset_from_link7)

MM = 1000.0
KIT = ASSETS_ROOT / "bodies_kit"


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


# ── cell layout (world, m; table top at z = 0) ──────────────────────────
ROBOT_BASE = np.array([0.0, 0.0, 0.0])
BLOCK_XY = np.array([0.475, 0.100])        # kit-frame origin (block centre)
SCREW_KIT_XY = np.array([0.02499, 0.000046])   # screw axis in the kit frame
SCREW_XY = BLOCK_XY + SCREW_KIT_XY            # block yaw 0
RECESS_PHASE = np.radians(20.0)   # seated recess orientation (screw turned about its axis)
TOOL_GRASP_XY = np.array([_env("SDT_TOOL_X", 0.430), _env("SDT_TOOL_Y", -0.030)])
# the tool lies along +y (tip at -y), on a handle flat; see GRASP PLAN

# ── screw mechanism ─────────────────────────────────────────────────────
PITCH = 0.00234                 # m/turn (measured crest spacing, right hand)
LEAD = PITCH / (2 * np.pi)      # m/rad
UNENGAGED_RISE = 0.0040         # head height above the seat with the threads unengaged
Q_UNENGAGED = UNENGAGED_RISE / LEAD           # 10.74 rad = 615 deg
Q_MAX = Q_UNENGAGED + 0.6
K_HELIX, C_HELIX = 3.0e4, 15.0  # N/m, N.s/m (explicit: k dt^2/m = 0.27, c dt/m = 0.26)
TAU_RUN0 = 0.0015               # N.m running thread drag, unloaded
MU_RUN = 0.0015                 # N.m per N of axial thread load (r_m 4.3 mm x mu_eff 0.35)
STICTION_K = 2.0                # N.m/rad bristle (5 mN.m -> 0.14 deg)
PRELOAD_FRAC, PRELOAD_MIN, PRELOAD_MAX = 0.7, 0.008, 0.040
LOOSE_PRELOAD0 = 0.020
LOOSE_FREE_DEG = 10.0
SCREW_MASS = 0.020
SCREW_KD = 0.002                # N.m.s/rad viscous part of the thread
SEAT_TOL = np.radians(1.0)
PROBE_TAU = 0.005               # N.m, the "small force"
T_PROBE = 1.0

# ── tool geometry (suite mesh body frame: origin = AABB centre, +y toward the butt) ─
TIP_Y = -0.0625
GRASP_D = 0.090                 # grasp point, m from the tip (suite GRIP_D "handle")
GRASP_Y = TIP_Y + GRASP_D
HANDLE_HALF = 0.0060            # half across flats
HANDLE_DROP = 0.0060            # TCP above the handle axis at the pick (tips ~2 mm off the table)

# ── fingers (MuJoCo-era suite values; GelSight adds finger_offset to every target) ─
FINGER_KE, FINGER_KD = 8000.0, 60.0
FINGER_F_MAX = 20.0             # N per pad (actuator force range)
FINGER_MASS = 0.12              # kg, README pitfall 4.5
PAD_MU = 1.0                    # the URDF pad value
PAD_FACE_OFF = 0.000085         # pad face = dof - 0.085 mm
SQUEEZE = 0.0015                # grip target inside the flat -> ~12 N at FINGER_KE
F_OPEN_PICK = HANDLE_HALF + 0.0080 + PAD_FACE_OFF + GS.finger_offset()
F_GRIP = HANDLE_HALF + PAD_FACE_OFF - SQUEEZE + GS.finger_offset()
F_WIDE = min(0.040, 0.020 + GS.finger_offset())
GRIP_BAND = (7.0, 16.0)         # N per pad, admittance band
GRIP_TRIM_STEP = 0.00001        # m per frame
GRIP_TRIM_MAX = 0.0015

# ── tool and contact materials ──────────────────────────────────────────
TOOL_MU = 0.5
SCREW_MU = 0.4
TABLE_MU = 0.5
MU_TORSION = 0.002              # condim-4 torsional row (0 -> MuJoCo NaN warning)
IMPRATIO = _env("SDT_IMPRATIO", 10.0)
ITERATIONS = 20

# ── arm tracking ────────────────────────────────────────────────────────
ARM_KD_TASK = 400.0
QD_MAX = 0.8 * np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
QDD_MAX = 15.0                  # rad/s^2
JUMP_RAD = 0.15                 # an IK jump, not a trajectory

# ── expert ──────────────────────────────────────────────────────────────
Z_HOVER = 0.090                 # TCP above the pick
Z_CARRY = 0.155                 # TCP while carrying the vertical tool
TIP_ABOVE = 0.020               # tip above the recess entry before the descent
PLACE_ABOVE = 0.025             # tool lowest point above the table at the end of pitch_back
FINGERTIP_FLOOR = 0.0015        # lowest fingertip point while placing (m above the table)
TIP_ENVELOPE = 0.00993          # fingertip collision box bottom below the TCP (pitfall 7.1)
PAD_HALF_W = 0.0089             # pad half-width across the approach
TIP_THICK = 0.0152              # fingertip thickness behind the pad face
F_TOUCH = 0.6                   # N tool<->screw: the tip has landed
F_PRESS = 6.0                   # N press while turning
PRESS_GAIN = 0.0006             # m/s per N of press error
PRESS_LEAD_MAX = 0.004
W_LO, W_HI = np.radians(35.0), np.radians(145.0)   # side-grasp approach azimuths
                                # with >= 0.35 rad of FR3 joint margin along the orbit
CREEP_RATE = np.radians(15.0)   # rad/s onto the seat
LIFT_OUT = 0.0120               # tip lift between strokes (recess 4 mm deep)
UNWIND = np.radians(4.0)        # back-off inside the +-6 deg tip backlash before a lift
SEAT_SLOW = np.radians(15.0)    # slow the wrist this close to the seat
SNUG_TAU = 0.040                # N.m at the tip: snug, stop turning
SNUG_OVER = np.radians(20.0)    # ... or this far past the seat on the wrist
MAX_STROKES = 10
LOOSE_GOAL = np.radians(300.0)   # backed off 300 deg (2 strokes)
TIMES = dict(settle=0.5, descend=1.2, close=0.7, hold=0.3, lift=1.0, pitch=2.2,
             carry=2.2, align=0.6, lower=1.0, insert_max=4.0, unpress=0.5,
             s_turn90=2.0, s_unpress=0.35, s_lift=0.5, s_swing90=2.2,
             snug_hold=0.3, extract=1.0, carry_back=2.4, pitch_back=2.2,
             place_max=3.0, open=0.6, retreat=1.4, still=1.0)


# ── small rotation helpers ──────────────────────────────────────────────
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def qaxis(axis, ang):
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    return np.r_[axis * np.sin(ang / 2), np.cos(ang / 2)]


def qrot(q, v):
    x, y, z, w = (float(c) for c in q)
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def slerp(a, b, s):
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    if np.dot(a, b) < 0:
        b = -b
    d = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if d > 0.99995:
        r = a + s * (b - a)
        return r / np.linalg.norm(r)
    th = np.arccos(d)
    return (np.sin((1 - s) * th) * a + np.sin(s * th) * b) / np.sin(th)


def rotvec(R):
    """Axis * angle of a rotation matrix."""
    c = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    th = np.arccos(c)
    if th < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return th * w / (2.0 * np.sin(th))


def expmap(rv):
    th = float(np.linalg.norm(rv))
    if th < 1e-12:
        return np.eye(3)
    k = rv / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def congruent(target, ref, period=np.pi / 2):
    """The angle congruent to ``target`` (mod period) nearest to ``ref``."""
    return ref + (((target - ref) + period / 2) % period - period / 2)


def tool_R_vertical(psi):
    """Tool body rotation: tip down (body +y -> world +z), body x at azimuth psi."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, 0.0, s], [s, 0.0, -c], [0.0, 1.0, 0.0]])


# ── the screw's thread / seat / probe model ─────────────────────────────
@wp.kernel
def _thread_kernel(joint_q: wp.array(dtype=wp.float32), joint_qd: wp.array(dtype=wp.float32),
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
    """Per-substep generalised forces on the screw's two dofs (see the
    header): helix spring, load-dependent elasto-plastic thread friction,
    seat preload, external probe torque.  One Warp kernel per substep (no
    host round trip); the host reads the state once per frame."""

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
        wp.launch(_thread_kernel, dim=1,
                  inputs=[state_in.joint_q, state_in.joint_qd, control.joint_f, self.st,
                          self.cz, self.cq, self.dz, self.dq, K_HELIX, C_HELIX, LEAD,
                          TAU_RUN0, MU_RUN, STICTION_K])
        return self._solver.step(state_in, state_out, control, contacts, dt)

    def __getattr__(self, name):
        return getattr(self._solver, name)


class DriveSensor:
    """``tasks.screwdriver.DriveContacts`` (force and torque about the screw
    axis carried by the tool<->screw contacts), with the pair filter
    vectorised (the original loops over every contact in Python)."""

    def __init__(self, model, tool_shapes, screw_shapes):
        self.shape_body = model.shape_body.numpy()
        self.d = np.array(sorted(int(x) for x in tool_shapes))
        self.s = np.array(sorted(int(x) for x in screw_shapes))
        self.history: list[dict] = []

    def sample(self, contacts, state, axis_xy, t):
        rec = {"t": t, "n": 0, "F": 0.0, "Fz": 0.0, "tau": 0.0}
        n = int(contacts.rigid_contact_count.numpy()[0])
        if n <= 0 or getattr(contacts, "force", None) is None:
            self.history.append(rec)
            return rec
        n = min(n, len(contacts.rigid_contact_shape0))
        s0 = contacts.rigid_contact_shape0.numpy()[:n]
        s1 = contacts.rigid_contact_shape1.numpy()[:n]
        keep = (np.isin(s0, self.d) & np.isin(s1, self.s)) | (np.isin(s0, self.s) & np.isin(s1, self.d))
        if not keep.any():
            self.history.append(rec)
            return rec
        f = contacts.force.numpy()[:n][keep].astype(float)
        p0 = contacts.rigid_contact_point0.numpy()[:n][keep].astype(float)
        b0 = self.shape_body[s0[keep]]
        bq = state.body_q.numpy()
        qv = bq[b0, 3:7]
        u, w = qv[:, :3], qv[:, 3:4]
        w0 = bq[b0, :3] + p0 + 2.0 * np.cross(u, np.cross(u, p0) + w * p0)
        r = w0.copy()
        r[:, 0] -= axis_xy[0]
        r[:, 1] -= axis_xy[1]
        sgn = np.where(np.isin(s0[keep], self.s), 1.0, -1.0)[:, None]
        fs = f * sgn                                   # force ON THE SCREW
        rec.update(n=int(keep.sum()), F=float(np.linalg.norm(f, axis=1).sum()),
                   Fz=float(fs[:, 2].sum()),
                   tau=float((r[:, 0] * fs[:, 1] - r[:, 1] * fs[:, 0]).sum()))
        self.history.append(rec)
        return rec


class JointTracker:
    """Velocity- and acceleration-limited joint command downstream of the IK
    (README pitfall 6.4): the limited pose is written back as the IK warm
    start, so a null-space jump of the 7-dof solution never reaches the
    drives.  The deceleration-limited approach (v <= sqrt(2 a |err|)) is only
    used for a JUMP (|err| > JUMP_RAD): on a smoothly moving target it makes
    the command trail by v^2 / 2a -- 0.04 rad at 1 rad/s, ~20 mm at the TCP
    (measured: the in-air swing wandered 9 mm off the tool axis and the tip
    dragged the screw back 27 deg)."""

    def __init__(self, rig, q0):
        self.rig = rig
        self.q = np.asarray(q0[:7], float).copy()
        self.dq = np.zeros(7)
        self.dt = 1.0 / rig.fps
        self.lag_peak = 0.0

    def step(self, p, q, fingers, iters=16):
        ik = self.rig.ik
        sol = ik.solve(p, q, iterations=iters)
        err = np.asarray(sol[:7], float) - self.q
        v_lim = np.where(np.abs(err) > JUMP_RAD,
                         np.minimum(QD_MAX, np.sqrt(2.0 * QDD_MAX * np.abs(err))), QD_MAX)
        v_t = np.clip(err / self.dt, -v_lim, v_lim)
        self.dq = self.dq + np.clip(v_t - self.dq, -QDD_MAX * self.dt, QDD_MAX * self.dt)
        self.q = self.q + self.dq * self.dt
        self.lag_peak = max(self.lag_peak, float(np.abs(err).max()))
        jq = ik.joint_q.numpy()
        jq[0, :7] = self.q
        ik.joint_q.assign(jq)
        qt = self.rig.control.joint_target_q.numpy()
        qt[:7] = self.q
        qt[7:9] = fingers
        self.rig.control.joint_target_q.assign(qt)


# ── scene ───────────────────────────────────────────────────────────────
def _kit_mesh(name, stem, max_edge):
    tm = trimesh.load(KIT / name / f"{stem}.stl", force="mesh")
    return subdiv(tm, max_edge)


def build_scene(rig, mode: str) -> dict:
    """FR3 + table + kit block + kit screw (thread model) + free screwdriver."""
    b = rig.builder
    rig.add_robot(wp.transform(wp.vec3(*ROBOT_BASE.tolist()), wp.quat_identity()))
    for dof in range(7):
        b.joint_target_kd[dof] = ARM_KD_TASK
    for dof in (7, 8):
        b.joint_target_ke[dof] = FINGER_KE
        b.joint_target_kd[dof] = FINGER_KD
    set_finger_effort_limit(b, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = b.body_label.index(bl)
        set_body_mass(b, bi, FINGER_MASS)
        for sid in b.body_shapes.get(bi, []):
            b.shape_material_mu[sid] = PAD_MU
            b.shape_material_mu_torsional[sid] = max(MU_TORSION, b.shape_material_mu_torsional[sid])

    # table (top at z = 0)
    sid = b.add_shape_box(-1, xform=wp.transform(wp.vec3(0.46, 0.0, -0.005), wp.quat_identity()),
                          hx=0.26, hy=0.30, hz=0.005,
                          cfg=rig.cfg(mu=TABLE_MU, mu_torsional=MU_TORSION, gap=0.0005),
                          label="table")
    rig.shapes["table"] = [sid]

    # block: kit frame = world frame shifted to BLOCK_XY (static scenery)
    blk = {}
    for part in ("block_top", "block_bottom"):
        tm = _kit_mesh("screw_block", part, 0.0030)
        tm.apply_translation([BLOCK_XY[0], BLOCK_XY[1], 0.0])
        mesh, c = make_newton_mesh(tm, sdf=True, sdf_resolution=256, narrow_band=0.0015,
                                   margin=0.0008)
        s = b.add_shape_mesh(-1, mesh=mesh, xform=wp.transform(wp.vec3(*np.asarray(c, float)),
                                                               wp.quat_identity()),
                             cfg=rig.cfg(mu=TABLE_MU, mu_torsional=MU_TORSION, gap=0.0005),
                             label=part)
        rig.shapes[part] = [s]
        rig.meshes[part] = (tm, np.asarray(c, float))
        blk[part] = s

    # screw: world -prismatic z-> carrier -revolute z-> screw
    scr_tm = _kit_mesh("screw_block", "screw_6_5", 0.0010)
    scr_tm.apply_translation([BLOCK_XY[0], BLOCK_XY[1], 0.0])
    scr_mesh, scr_c = make_newton_mesh(scr_tm, sdf=True, sdf_resolution=512, narrow_band=0.0015,
                                       margin=0.0002)
    scr_c = np.asarray(scr_c, float)
    q_rest = qaxis([0, 0, 1], RECESS_PHASE)
    carrier = b.add_link(xform=wp.transform(wp.vec3(*scr_c), wp.quat_identity()),
                         label="screw_carrier")
    b.add_shape_box(carrier, hx=0.002, hy=0.002, hz=0.002,
                    cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False,
                                                        density=1000.0),
                    label="screw_carrier_shape")
    set_body_mass(b, carrier, 0.010)
    j_z = b.add_joint_prismatic(
        parent=-1, child=carrier, axis=(0, 0, 1),
        parent_xform=wp.transform(wp.vec3(*scr_c), wp.quat_identity()),
        target_ke=0.0, target_kd=0.0, limit_lower=-0.00005,
        limit_upper=UNENGAGED_RISE + 0.0006)
    screw = b.add_link(xform=wp.transform(wp.vec3(*scr_c), wp.quat(*q_rest)), label="screw")
    s_scr = b.add_shape_mesh(screw, mesh=scr_mesh,
                             cfg=rig.cfg(mu=SCREW_MU, mu_torsional=MU_TORSION),
                             label="screw_shape")
    rig.shapes["screw"] = [s_scr]
    rig.meshes["screw"] = (scr_tm, scr_c)
    j_q = b.add_joint_revolute(
        parent=carrier, child=screw, axis=(0.0, 0.0, 1.0),
        parent_xform=wp.transform_identity(),
        child_xform=wp.transform(wp.vec3(0.0, 0.0, 0.0),
                                 wp.quat(*qaxis([0, 0, 1], -RECESS_PHASE))),
        target_ke=0.0, target_kd=SCREW_KD, limit_lower=0.0, limit_upper=Q_MAX)
    b.add_articulation([j_z, j_q])
    set_body_mass(b, screw, SCREW_MASS)
    for part in ("block_top", "block_bottom", "table"):
        rig.filter_labels("screw", part)

    # screwdriver: a FREE body, lying on a handle flat, tip toward -y
    drv_raw = load_tm("screwdriver", "part2_1_body1")
    drv_c = 0.5 * (drv_raw.bounds[0] + drv_raw.bounds[1])
    tip_tm, rest_tm = split_driver(drv_raw, 0.020)
    tip_tm, rest_tm = subdiv_p(tip_tm, 0.0012), subdiv_p(rest_tm, 0.0020)
    tip_mesh, tip_c = make_newton_mesh(tip_tm, sdf=True, sdf_resolution=512, narrow_band=0.0015,
                                       margin=0.0002)
    rest_mesh, rest_c = make_newton_mesh(rest_tm, sdf=True, sdf_resolution=512,
                                         narrow_band=0.002, margin=0.0005)
    p0 = np.array([TOOL_GRASP_XY[0], TOOL_GRASP_XY[1] - GRASP_Y, HANDLE_HALF + 0.00005])
    tool = b.add_body(xform=wp.transform(wp.vec3(*p0), wp.quat_identity()), label="tool")
    tcfg = dict(mu=TOOL_MU, mu_torsional=MU_TORSION)
    s_tip = b.add_shape_mesh(tool, mesh=tip_mesh, cfg=rig.cfg(**tcfg),
                             xform=wp.transform(wp.vec3(*(np.asarray(tip_c) - drv_c)),
                                                wp.quat_identity()), label="tool_tip")
    s_rest = b.add_shape_mesh(tool, mesh=rest_mesh, cfg=rig.cfg(**tcfg),
                              xform=wp.transform(wp.vec3(*(np.asarray(rest_c) - drv_c)),
                                                 wp.quat_identity()), label="tool_shape")
    rig.shapes["tool"] = [s_tip, s_rest]
    drv_tm = subdiv_p(drv_raw, 0.0015)
    rig.meshes["tool"] = (drv_tm, drv_c)
    # arm links never touch the tool, the fingertips do (Rig.robot_grip policy)
    rig._filter_arm_from(tool, filter_fingers=False)

    # static-static pairs and the ground plane carry nothing here
    statics = list(b.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            rig.filter_pair(statics[i], statics[j])
    ground = statics[0]
    for bb, sids in b.body_shapes.items():
        for s in sids:
            if s != ground:
                rig.filter_pair(ground, s)

    # start state of the screw
    q0 = Q_UNENGAGED if mode == "tighten" else 0.0
    rig._joint_of["screw_z"] = j_z
    rig._joint_of["screw_q"] = j_q
    return {"j_z": j_z, "j_q": j_q, "q0": q0, "scr_c": scr_c, "tool_p0": p0,
            "drv_c": np.asarray(drv_c, float), "drv_tm": drv_tm, "scr_tm": scr_tm,
            "tip_tm": tip_tm, "tip_c": np.asarray(tip_c, float)}


# ── measurements ────────────────────────────────────────────────────────
class Meas:
    """Everything the expert reads from the simulator each frame."""

    def __init__(self, rig, h):
        self.rig, self.h = rig, h
        self.i_tool = rig.body_index("tool")
        self.i_scr = rig.body_index("screw")
        self.i7 = rig.body_index("fr3/fr3_link7")
        qs = rig.model.joint_q_start.numpy()
        self.cz, self.cq = int(qs[h["j_z"]]), int(qs[h["j_q"]])
        self.T_off = tcp_offset_from_link7(rig)
        loc = h["drv_tm"].vertices - h["drv_c"]
        self.v_low = loc[np.random.RandomState(0).choice(len(loc), min(600, len(loc)),
                                                           replace=False)]

    def update(self, state):
        bq = state.body_q.numpy()
        jq = state.joint_q.numpy()
        T = world_T(bq[self.i_tool])
        self.R, self.p = T[:3, :3], T[:3, 3]
        self.tip = self.p + self.R @ np.array([0.0, TIP_Y, 0.0])
        self.grasp = self.p + self.R @ np.array([0.0, GRASP_Y, 0.0])
        self.axis = self.R[:, 1]                       # tip -> butt
        xh = self.R[:, 0] - self.R[2, 0] * np.array([0.0, 0.0, 1.0])
        self.flats_yaw = float(np.arctan2(xh[1], xh[0]))
        self.tilt = float(np.degrees(np.arccos(np.clip(self.axis[2], -1, 1))))
        self.low = float((self.v_low @ self.R[2] + self.p[2]).min())   # lowest tool point
        self.q = float(jq[self.cq])
        self.z = float(jq[self.cz])
        self.recess_yaw = float(RECESS_PHASE + self.q)
        self.T_tcp = world_T(bq[self.i7]) @ self.T_off
        self.tcp = self.T_tcp[:3, 3]
        self.bq = bq
        return self

    def T_tool(self):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.p
        return T


def run(mode: str):
    assert mode in ("tighten", "loosen")
    name = f"{mode}_screw_with_screwdriver_from_table"
    rig = appendix_rig(name)
    rig.collide_every_substep = True
    rig.hard_joint_limits = True
    rig.solver_options["impratio"] = IMPRATIO
    h = build_scene(rig, mode)
    b = rig.builder

    # arm start: hovering over the pick point
    q_pick = down_grasp_quat(np.pi / 2)            # x_tcp = +y: along the tool, tip at -x_tcp
    tcp_pick = np.array([TOOL_GRASP_XY[0], TOOL_GRASP_XY[1], HANDLE_HALF + HANDLE_DROP])
    tcp_hover = tcp_pick + np.array([0.0, 0.0, Z_HOVER - tcp_pick[2]])
    q_start = rig.solve_arm_start(tcp_hover, q_pick)
    b.joint_q[7:9] = [F_OPEN_PICK, F_OPEN_PICK]
    b.joint_target_q[7:9] = [F_OPEN_PICK, F_OPEN_PICK]

    cam_t = np.array([_env("SDT_CAM_TX", 0.470), _env("SDT_CAM_TY", 0.045),
                      _env("SDT_CAM_TZ", 0.055)])
    cam_p = cam_t + np.array([_env("SDT_CAM_OX", 0.30), _env("SDT_CAM_OY", 0.28),
                              _env("SDT_CAM_OZ", 0.18)])
    rig.finalize(probes=["tool"], cam_pos=cam_p, cam_target=cam_t)
    rig.solver = make_solver(rig.model, iterations=ITERATIONS, **rig.solver_options)
    model = rig.model

    # kit colours: yellow block, grey screw, black screwdriver
    sc = model.shape_color.numpy()
    labels = list(model.shape_label)
    for lbl, col in (("block_top", (0.93, 0.80, 0.10)), ("block_bottom", (0.93, 0.80, 0.10)),
                     ("screw_shape", (0.42, 0.43, 0.45)), ("tool_tip", (0.10, 0.10, 0.11)),
                     ("tool_shape", (0.10, 0.10, 0.11))):
        sc[labels.index(lbl)] = col
    model.shape_color.assign(sc)

    # start state of the screw (unengaged: 615 deg short, head 4 mm up)
    qs = model.joint_q_start.numpy()
    cz, cq = int(qs[h["j_z"]]), int(qs[h["j_q"]])
    for state in (rig.s0, rig.s1):
        jq = state.joint_q.numpy()
        jq[cq] = h["q0"]
        jq[cz] = LEAD * h["q0"]
        state.joint_q.assign(jq)
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
    thread = ThreadModel(rig.solver, model, h["j_z"], h["j_q"], h["q0"],
                         LOOSE_PRELOAD0 if mode == "loosen" else 0.0)
    rig.solver = thread

    # probes
    tool_shapes = ["tool_tip", "tool_shape"]
    rig.probes["padL"] = PairProbe(model, "fr3/fr3_leftfinger", tool_shapes)
    rig.probes["padR"] = PairProbe(model, "fr3/fr3_rightfinger", tool_shapes)
    rig.probes["tool_table"] = PairProbe(model, "tool", ["table"])
    rig.probes["tool_block"] = PairProbe(model, "tool", ["block_top", "block_bottom"])
    rig.probes["robot_obj"] = RobotContactProbe(model, ["tool", "screw"],
                                                static_shapes=["block_top", "block_bottom"])
    rig.probes["robot_table"] = RobotContactProbe(model, [], static_shapes=["table"])
    drive = DriveSensor(model, rig.shapes["tool"], rig.shapes["screw"])

    meas = Meas(rig, h)
    trk = JointTracker(rig, q_start)
    expert = Expert(rig, meas, thread, drive, mode, q_pick, tcp_pick, tcp_hover)
    stop_at = os.environ.get("SDT_STOP", "")
    log = []

    def last(name, key="force"):
        hh = rig.probes[name].history
        return hh[-1][key] if hh else 0.0

    def control(t):
        meas.update(rig.s0)
        p, q, f = expert.step(t, last)
        if expert.finished or (stop_at and expert.phase == stop_at):
            raise StopRun
        trk.step(p, q, f)

    def extra(state, t):
        thread.refresh()
        m = meas.update(state)
        c = drive.sample(rig.contacts, state, SCREW_XY, t)
        tool_low = m.low
        row = dict(t=t, q=m.q, z=m.z, tipx=m.tip[0], tipy=m.tip[1], tipz=m.tip[2],
                   gx=m.grasp[0], gy=m.grasp[1], gz=m.grasp[2], px=m.p[0], py=m.p[1], pz=m.p[2],
                   tilt=m.tilt, flats=m.flats_yaw, tcpx=m.tcp[0], tcpy=m.tcp[1], tcpz=m.tcp[2],
                   padL=last("padL"), padR=last("padR"), table=last("tool_table"),
                   block=last("tool_block"), robot_obj=last("robot_obj"),
                   robot_table=last("robot_table"), tau=c["tau"], Fd=c["F"], Fz=c["Fz"],
                   F_ax=thread.F_ax, tau_f=thread.tau_f, probe=thread.probe,
                   preload=thread.preload, low=tool_low, phase=expert.phase_id())
        log.append(row)
        k = int(round(t * rig.fps))
        if k % 15 == 0:
            print(f"  [{t:6.2f} {expert.phase:11s}] pads {row['padL']:5.2f}/{row['padR']:5.2f} "
                  f"tbl {row['table']:5.3f} drv {c['F']:5.2f}N tau {c['tau']*1000:6.2f}mNm "
                  f"q {np.degrees(m.q):7.1f} z {m.z*MM:5.2f} tilt {m.tilt:5.1f} "
                  f"tip {np.round(m.tip*MM,1)} tcp {np.round(m.tcp*MM,1)} "
                  f"rob {row['robot_obj']:5.2f} tblR {row['robot_table']:5.2f} "
                  f"fric {thread.tau_f*1000:5.2f} pre {thread.preload*1000:4.1f} "
                  f"f {expert.f_cmd*MM:5.2f} err {expert.k.get('tool_err', '')}")
        return {"padL_N": round(row["padL"], 3), "padR_N": round(row["padR"], 3),
                "tool_table_N": round(row["table"], 4), "tool_block_N": round(row["block"], 4),
                "drive_force_N": round(c["F"], 3), "drive_torque_mNm": round(c["tau"] * 1000, 3),
                "robot_object_contact_N": round(row["robot_obj"], 4),
                "screw_deg": round(float(np.degrees(m.q)), 3),
                "screw_rise_mm": round(m.z * MM, 4),
                "thread_friction_mNm": round(thread.tau_f * 1000, 3),
                "probe_torque_mNm": round(thread.probe * 1000, 3),
                "seat_preload_mNm": round(thread.preload * 1000, 3),
                "tool_tilt_deg": round(m.tilt, 3)}

    budget = _env("SDT_BUDGET", 120.0)
    try:
        rec = rig.run(budget, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{name}] stopped at t={len(rec.times) / rig.fps:.2f}s (phase {expert.phase})")
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "drive_force_N",
                                            "drive_torque_mNm", "tool_table_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    if not expert.finished:
        print("expert did not finish: QA skipped")
        np.save(rig.out_dir / "partial_log.npy", np.array([[r[k] for k in LOG_KEYS] for r in log]))
        return None
    try:
        return score(rig, h, expert, thread, log, paths, mode, name)
    except Exception:
        import traceback
        traceback.print_exc()
        raise


LOG_KEYS = ("t", "q", "z", "tipx", "tipy", "tipz", "gx", "gy", "gz", "px", "py", "pz", "tilt",
            "flats", "tcpx", "tcpy", "tcpz", "padL", "padR", "table", "block", "robot_obj",
            "robot_table", "tau", "Fd", "Fz", "F_ax", "tau_f", "probe", "preload", "low", "phase")


# ── the expert ──────────────────────────────────────────────────────────
PHASES = ["settle", "descend", "close", "hold", "lift", "pitch", "carry", "align", "lower",
          "s_insert", "s_turn", "snug_hold", "s_unpress", "s_lift", "s_swing",
          "end_unpress", "extract", "carry_back", "pitch_back", "place", "open", "retreat",
          "still", "probe", "done"]


class Expert:
    """Phase machine; returns (tcp_pos, tcp_quat, finger_dof) every frame.

    One grasp from pick to put-back: the side grasp the pitch produces.
    Strokes turn the hand pose about the screw axis inside the IK window of
    approach azimuths [W_LO, W_HI]; between strokes the tool is lifted out
    IN THE GRIP, swung back 90 deg in the air about its own axis (its flats
    stay congruent with the recess) and re-inserted.  The tool never stands
    unheld (a free-standing tool rocks and falls in ~1 s, and a light-pinch
    regrasp around it levers the tip out of the recess -- both measured)."""

    def __init__(self, rig, meas, thread, drive, mode, q_pick, tcp_pick, tcp_hover):
        self.rig, self.m, self.thread, self.drive, self.mode = rig, meas, thread, drive, mode
        self.q_pick, self.tcp_pick, self.tcp_hover = q_pick, tcp_pick, tcp_hover
        self.phase, self.t0 = "settle", 0.0
        self.t_start = {"settle": 0.0}
        self.finished = False
        self.p_cmd, self.q_cmd = tcp_hover.copy(), q_pick.copy()
        self.f_cmd = F_OPEN_PICK
        self.trim = 0.0
        self.k = {}                    # phase-local memory
        self.strokes = 0
        self.press = 0.0               # press lead below the landing height (m)
        self.z_land = 0.0
        self.corr = np.zeros(3)        # grasp-point xy integral
        self.k_swing = 0.0
        self.seated = False
        self.tau_seat = 0.0
        self.released = mode == "tighten"
        self.t_release = None
        self.t_probe = None
        self.place_gap = None
        self.stroke_log = []
        self.insert_log = []
        self.retries = 0

    # -- bookkeeping --
    def goto(self, ph, t):
        self.phase, self.t0 = ph, t
        self.t_start.setdefault(ph, t)
        self.k = {}
        print(f"t={t:6.2f}s -> {ph}")

    def phase_id(self):
        return PHASES.index(self.phase)

    def s(self, t, dur):
        return min(1.0, (t - self.t0) / dur)

    def fingertip_low(self):
        """Lowest point of the two fingertip boxes (actual TCP pose, current
        finger opening): the placement floor, valid for a tilted hand."""
        R = self.m.T_tcp[:3, :3]
        best = 9.0
        half = max(self.f_cmd - PAD_FACE_OFF - GS.finger_offset(), 0.0)
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for dy in (half, half + TIP_THICK):
                    v = np.array([sx * PAD_HALF_W, sy * dy, TIP_ENVELOPE])
                    best = min(best, float(self.m.tcp[2] + R[2] @ v))
        return best

    def Fz(self):
        """Downward push of the tool on the screw (N)."""
        return -self.drive.history[-1]["Fz"] if self.drive.history else 0.0

    def grip_cmd(self, last, base=F_GRIP):
        F = 0.5 * (last("padL") + last("padR"))
        lo, hi = GRIP_BAND
        if F < lo:
            self.trim = max(-GRIP_TRIM_MAX, self.trim - GRIP_TRIM_STEP)
        elif F > hi:
            self.trim = min(GRIP_TRIM_MAX, self.trim + GRIP_TRIM_STEP)
        return base + self.trim

    def ramp_f(self, t, dur, f1):
        u = smooth5(self.s(t, dur))
        f0 = self.k.setdefault("f0", self.f_cmd)
        return f0 + (f1 - f0) * u

    def approach_az(self, q=None):
        z = qrot(self.q_cmd if q is None else q, [0, 0, 1])
        return float(np.arctan2(z[1], z[0]))

    # -- poses --
    def tcp_for_tool(self, R_des, p_tip_des, H):
        T_des = np.eye(4)
        T_des[:3, :3] = R_des
        T_des[:3, 3] = p_tip_des - R_des @ np.array([0.0, TIP_Y, 0.0])
        T_cmd = T_des @ np.linalg.inv(H)
        return T_cmd[:3, 3], quat_of_R(T_cmd[:3, :3])

    def in_hand(self):
        return np.linalg.inv(self.m.T_tcp) @ self.m.T_tool()

    def insert_target(self):
        """Vertical tool, flats on the MEASURED recess, the 90 deg branch
        nearest the tool's current flats (<= 45 deg about the tool axis)."""
        psi = congruent(self.m.recess_yaw, self.m.flats_yaw)
        return psi, tool_R_vertical(psi)

    def recess_center(self):
        return np.array([SCREW_XY[0], SCREW_XY[1], 0.027 + self.m.z])

    def tool_servo(self, tip_des, z_free=True, gain_p=0.08, gain_r=0.06):
        """Integral servo on the MEASURED tool: each frame the commanded hand
        pose moves by a fraction of the tool's pose error -- a rotation about
        the measured tip (a tilt or yaw fix does not swing the tip), then a
        rate-limited step on the tip position error, plus the target's own
        motion as feedforward.  Absorbs arm droop and pad slip."""
        m = self.m
        tip_des = np.asarray(tip_des, float)
        prev = self.k.get("tip_des_prev", tip_des)
        self.k["tip_des_prev"] = tip_des.copy()
        p = self.p_cmd + (tip_des - prev)
        _, R_des = self.insert_target()
        rv = rotvec(R_des @ m.R.T)
        step = gain_r * rv
        lim = np.radians(0.4)
        if np.linalg.norm(step) > lim:
            step = step * lim / np.linalg.norm(step)
        Rd = expmap(step)
        R_cmd = Rd @ R_of_q(self.q_cmd)
        p = m.tip + Rd @ (p - m.tip)
        e = tip_des - m.tip
        if not z_free:
            e[2] = 0.0
        p = p + np.clip(gain_p * e, -0.0003, 0.0003)
        self.k["tool_err"] = (round(float(np.linalg.norm(e[:2])) * MM, 3),
                              round(float(np.degrees(np.linalg.norm(rv))), 2))
        return p, quat_of_R(R_cmd)

    # -- the driving grasp: the hand pose turned about the screw axis --
    def drive_pose(self, F_target, phi, gripped=True):
        """The hand pose recorded when the tip seated (k_ref: p0, R0) turned
        by phi about the VERTICAL SCREW AXIS: the tool turns about its own
        axis wherever the grasp holds it.  Press = force servo along -z below
        the landing height; a slow xy integral keeps the tool's grasp point
        over the axis (arm droop)."""
        m = self.m
        self.press = float(np.clip(self.press + PRESS_GAIN * (F_target - self.Fz()) / self.rig.fps,
                                   -PRESS_LEAD_MAX, PRESS_LEAD_MAX))
        if gripped:
            exy = np.array([SCREW_XY[0] - m.grasp[0], SCREW_XY[1] - m.grasp[1], 0.0])
            self.corr = np.clip(self.corr + np.clip(0.03 * exy, -0.0001, 0.0001), -0.008, 0.008)
        p0, R0 = self.k_ref
        Rz = expmap(np.array([0.0, 0.0, phi]))
        c = np.array([SCREW_XY[0], SCREW_XY[1], p0[2]])
        p = c + Rz @ (p0 - c)
        p[2] = self.z_land - self.press
        return p + self.corr, quat_of_R(Rz @ R0)

    # -- the step --
    def step(self, t, last):
        m = self.m
        ph = self.phase
        T = TIMES
        p, q, f = self.p_cmd, self.q_cmd, self.f_cmd
        if ph == "settle":
            p, q, f = self.tcp_hover, self.q_pick, F_OPEN_PICK
            if t - self.t0 >= T["settle"]:
                self.T_tool0 = m.T_tool().copy()        # where it lay: put it back there
                self.goto("descend", t)
        elif ph == "descend":
            u = smooth5(self.s(t, T["descend"]))
            p = self.tcp_hover + (self.tcp_pick - self.tcp_hover) * u
            if t - self.t0 >= T["descend"]:
                self.goto("close", t)
        elif ph == "close":
            u = smooth5(self.s(t, T["close"]))
            f = F_OPEN_PICK + (F_GRIP - F_OPEN_PICK) * u
            if t - self.t0 >= T["close"]:
                self.goto("hold", t)
        elif ph == "hold":
            f = self.grip_cmd(last)
            if t - self.t0 >= T["hold"]:
                self.goto("lift", t)
        elif ph == "lift":
            f = self.grip_cmd(last)
            u = smooth5(self.s(t, T["lift"]))
            p = self.tcp_pick + np.array([0.0, 0.0, (Z_HOVER + 0.02 - self.tcp_pick[2]) * u])
            if t - self.t0 >= T["lift"]:
                self.k_lift_p = p.copy()
                self.goto("pitch", t)
        elif ph == "pitch":
            # hand + tool rotate 90 deg about the closing axis: the tip swings
            # down, the fingers end up pointing +y (side grasp)
            f = self.grip_cmd(last)
            u = smooth5(self.s(t, T["pitch"]))
            y_ax = qrot(self.q_pick, [0, 1, 0])
            q_side = qmul(qaxis(y_ax, np.pi / 2), self.q_pick)
            if qrot(q_side, [1, 0, 0])[2] < 0:
                q_side = qmul(qaxis(y_ax, -np.pi / 2), self.q_pick)
            q = slerp(self.q_pick, q_side, u)
            p = self.k_lift_p + np.array([0.0, 0.0, (Z_CARRY - self.k_lift_p[2]) * u])
            if t - self.t0 >= T["pitch"]:
                self.goto("carry", t)
        elif ph == "carry":
            f = self.grip_cmd(last)
            if "H" not in self.k:
                H = self.in_hand()
                psi, R = self.insert_target()
                tgt = self.recess_center() + np.array([0.0, 0.0, TIP_ABOVE])
                p1, q1 = self.tcp_for_tool(R, tgt, H)
                self.k.update(H=H, p0=self.p_cmd.copy(), q0=self.q_cmd.copy(), p1=p1, q1=q1)
                print(f"    carry: recess {np.degrees(m.recess_yaw) % 360:.1f} deg, tool flats "
                      f"{np.degrees(m.flats_yaw):.1f} -> {np.degrees(psi):.1f} deg, approach "
                      f"azimuth {np.degrees(self.approach_az(q1)):.1f} deg")
            u = smooth5(self.s(t, T["carry"]))
            p = self.k["p0"] + (self.k["p1"] - self.k["p0"]) * u
            p = p + np.array([0.0, 0.0, 0.02 * np.sin(np.pi * u)])
            q = slerp(self.k["q0"], self.k["q1"], u)
            if t - self.t0 >= T["carry"]:
                self.goto("align", t)
        elif ph == "align":
            f = self.grip_cmd(last)
            p, q = self.tool_servo(self.recess_center() + [0.0, 0.0, TIP_ABOVE])
            if t - self.t0 >= T["align"]:
                self.goto("lower", t)
        elif ph == "lower":
            f = self.grip_cmd(last)
            u = smooth5(self.s(t, T["lower"]))
            p, q = self.tool_servo(self.recess_center()
                                   + [0.0, 0.0, TIP_ABOVE + (0.0010 - TIP_ABOVE) * u])
            if t - self.t0 >= T["lower"]:
                self.goto("s_insert", t)
        elif ph == "s_insert":
            # descend at 4 mm/s until the tip lands, then press; done once
            # the tip is >= 3 mm into the 4 mm recess and the tool is vertical
            f = self.grip_cmd(last)
            k = self.k
            if "z0" not in k:
                k.update(z0=self.p_cmd[2], dz=0.0, landed=False, t_land=None)
            if not k["landed"]:
                above = m.tip[2] - self.recess_center()[2]       # tip over the entry
                k["dz"] -= (0.010 if above > 0.002 else 0.005) / self.rig.fps
                if self.Fz() > F_TOUCH:
                    k["landed"], k["t_land"] = True, t
                    k["z_land"] = k["z0"] + k["dz"]
            else:
                k["dz"] -= float(np.clip(PRESS_GAIN * (F_PRESS - self.Fz()), -0.003, 0.003)) / self.rig.fps
            k["dz"] = max(k["dz"], -(LIFT_OUT + 0.010))   # descent budget
            p, q = self.tool_servo(self.recess_center(), z_free=False)
            p[2] = k["z0"] + k["dz"]
            depth = self.recess_center()[2] - m.tip[2]
            ok = k["landed"] and t - k["t_land"] > 0.25 and depth > 0.0030 and m.tilt < 1.0
            if (not ok) and t - self.t0 >= T["insert_max"] and self.retries < 2:
                self.retries += 1
                print(f"    insertion not seated (depth {depth*MM:.2f} mm, press "
                      f"{self.Fz():.2f} N): lift and retry {self.retries}")
                self.k_swing = 0.0
                self.goto("s_lift", t)
            elif ok or t - self.t0 >= T["insert_max"]:
                self.retries = 0
                self.insert_log.append({"t": round(t, 2), "depth_mm": round(depth * MM, 2),
                                        "press_N": round(self.Fz(), 2), "ok": bool(ok),
                                        "tilt_deg": round(m.tilt, 2),
                                        "approach_az_deg": round(np.degrees(self.approach_az(q)), 1)})
                print(f"    inserted: tip depth {depth*MM:.2f} mm, press {self.Fz():.2f} N, "
                      f"tilt {m.tilt:.2f} deg, ok {ok}")
                self.z_land = float(k["z_land"]) if k.get("z_land") is not None else p[2]
                self.press = float(self.z_land - p[2])
                self.corr = np.zeros(3)
                self.k_ref = (np.asarray(p, float).copy(), R_of_q(q))
                self.goto("s_turn", t)
        elif ph in ("s_turn", "snug_hold", "s_unpress", "s_lift", "s_swing"):
            p, q, f = self.stroke_step(t, last)
        elif ph == "end_unpress":
            # as s_unpress: press off and unwind inside the backlash, then lift
            sgn = -1.0 if self.mode == "tighten" else 1.0
            u = smooth5(self.s(t, T["unpress"]))
            p, q = self.drive_pose(0.2, self.phi_end - sgn * UNWIND * u, gripped=False)
            f = self.grip_cmd(last)
            if t - self.t0 >= T["unpress"]:
                self.goto("extract", t)
        elif ph == "extract":
            f = self.grip_cmd(last)
            p0 = self.k.setdefault("p0", self.p_cmd.copy())
            u = smooth5(self.s(t, T["extract"]))
            p = p0 + np.array([0.0, 0.0, 0.030 * u])
            if t - self.t0 >= T["extract"]:
                self.goto("carry_back", t)
        elif ph == "carry_back":
            # back over the pick point with the tool vertical; the hand poses
            # come from the MEASURED in-hand transform so the tool, not the
            # hand, retraces the pick: carry_back ends at the tool pose the
            # forward pitch produced, pitch_back at the tool lying flat
            # PLACE_ABOVE over where it lay
            f = self.grip_cmd(last)
            if "p0" not in self.k:
                H = self.in_hand()
                # the tool may lie on any of its four handle flats: take the
                # roll that leaves the hand closest to fingers-down
                best = None
                for kr in range(4):
                    Tf = self.T_tool0.copy()
                    Tf[:3, :3] = Tf[:3, :3] @ expmap(np.array([0.0, kr * np.pi / 2, 0.0]))
                    Tf[2, 3] += PLACE_ABOVE
                    Tc = Tf @ np.linalg.inv(H)
                    if best is None or Tc[2, 2] < best[0]:
                        best = (float(Tc[2, 2]), Tf, Tc)
                _, T_flat, Tc_flat = best
                g = T_flat[:3, :3] @ np.array([0.0, GRASP_Y, 0.0]) + T_flat[:3, 3]
                Rx = expmap(np.array([np.pi / 2, 0.0, 0.0]))      # the forward pitch
                T_up = np.eye(4)
                T_up[:3, :3] = Rx @ T_flat[:3, :3]
                T_up[:3, 3] = np.array([g[0], g[1], Z_CARRY]) + Rx @ (T_flat[:3, 3] - g)
                Tc_up = T_up @ np.linalg.inv(H)
                self.k_back = {"up": (Tc_up[:3, 3], quat_of_R(Tc_up[:3, :3])),
                               "flat": (Tc_flat[:3, 3], quat_of_R(Tc_flat[:3, :3]))}
                self.k.update(p0=self.p_cmd.copy(), q0=self.q_cmd.copy())
            p1, q1 = self.k_back["up"]
            u = smooth5(self.s(t, T["carry_back"]))
            p = self.k["p0"] + (p1 - self.k["p0"]) * u
            p = p + np.array([0.0, 0.0, 0.02 * np.sin(np.pi * u)])
            q = slerp(self.k["q0"], q1, u)
            if t - self.t0 >= T["carry_back"]:
                self.goto("pitch_back", t)
        elif ph == "pitch_back":
            f = self.grip_cmd(last)
            p0, q0 = self.k_back["up"]
            p1, q1 = self.k_back["flat"]
            u = smooth5(self.s(t, T["pitch_back"]))
            q = slerp(q0, q1, u)
            p = p0 + (p1 - p0) * u
            if t - self.t0 >= T["pitch_back"]:
                self.goto("place", t)
        elif ph == "place":
            # lower until the tool's measured lowest point is 0.4 mm above the table
            f = self.grip_cmd(last)
            gap = m.low
            v = float(np.clip(1.5 * (gap - 0.0004), 0.0015, 0.015))
            p = self.p_cmd - np.array([0.0, 0.0, v / self.rig.fps])
            floor = self.fingertip_low() <= FINGERTIP_FLOOR
            if gap < 0.0005 or last("tool_table") > 0.05 or floor \
                    or t - self.t0 >= T["place_max"]:
                self.place_gap = float(gap)
                print(f"    placed: lowest tool point {gap*MM:.2f} mm above the table "
                      f"(fingertips {self.fingertip_low()*MM:.1f} mm, floor {floor}), table "
                      f"force {last('tool_table'):.3f} N, tilt {m.tilt:.1f}")
                self.goto("open", t)
        elif ph == "open":
            f = self.ramp_f(t, T["open"], F_WIDE)
            if t - self.t0 >= T["open"]:
                self.t_release = t
                self.goto("retreat", t)
        elif ph == "retreat":
            p0 = self.k.setdefault("p0", self.p_cmd.copy())
            u = smooth5(self.s(t, T["retreat"]))
            p = p0 + np.array([0.0, 0.0, 0.10 * u])
            f = F_WIDE
            if t - self.t0 >= T["retreat"]:
                self.goto("still", t)
        elif ph == "still":
            f = F_WIDE
            if t - self.t0 >= T["still"]:
                self.t_probe = t
                self.goto("probe", t)
        elif ph == "probe":
            f = F_WIDE
            kk = (t - self.t0) / T_PROBE
            # +q = loosen-way (counter-clockwise), -q = tighten-way
            self.thread.probe = PROBE_TAU if kk < 1.0 else (-PROBE_TAU if kk < 2.0 else 0.0)
            if kk >= 2.0:
                self.thread.probe = 0.0
                self.goto("done", t)
        elif ph == "done":
            self.finished = True
        # seat preload bookkeeping (loosen: released once backed off)
        if (not self.released) and m.q >= np.radians(LOOSE_FREE_DEG):
            self.released = True
            self.thread.set_preload(0.0, m.q)
        self.p_cmd, self.q_cmd, self.f_cmd = np.asarray(p, float), np.asarray(q, float), float(f)
        return self.p_cmd, self.q_cmd, self.f_cmd

    # -- strokes --
    def stroke_step(self, t, last):
        m = self.m
        T = TIMES
        ph = self.phase
        sgn = -1.0 if self.mode == "tighten" else 1.0     # tighten = clockwise = -yaw
        f = self.grip_cmd(last)
        p, q = self.p_cmd, self.q_cmd
        if ph == "s_turn":
            if "q0" not in self.k:
                az = self.approach_az()
                span = (az - W_LO) if self.mode == "tighten" else (W_HI - az)
                span = float(np.clip(span, 0.0, W_HI - W_LO))
                self.k.update(q0=m.q, az0=az, span=span, u=0.0, phi=0.0, seat_phi=None,
                              dur=max(0.8, T["s_turn90"] * span / (np.pi / 2)))
            kt = self.k
            near = self.mode == "tighten" and m.q < SEAT_SLOW
            if near:                                    # creep onto the seat
                kt["phi"] = max(-kt["span"], kt["phi"] - CREEP_RATE / self.rig.fps)
            else:
                kt["u"] = min(1.0, kt["u"] + 1.0 / (kt["dur"] * self.rig.fps))
                kt["phi"] = sgn * kt["span"] * smooth5(kt["u"])
            p, q = self.drive_pose(F_PRESS, kt["phi"])
            tau = abs(self.drive.history[-1]["tau"]) if self.drive.history else 0.0
            goal = False
            if self.mode == "tighten":
                if m.q <= SEAT_TOL:
                    self.seated = True
                    if kt["seat_phi"] is None:
                        kt["seat_phi"] = kt["phi"]
                    self.tau_seat = max(self.tau_seat, tau)
                    goal = tau >= SNUG_TAU or abs(kt["phi"] - kt["seat_phi"]) >= SNUG_OVER
            else:
                goal = m.q >= LOOSE_GOAL
            at_end = abs(kt["phi"]) >= kt["span"] - 1e-6
            self.phi_end = kt["phi"]
            if goal or at_end:
                self.strokes += 1
                self.stroke_log.append({"stroke": self.strokes,
                                        "screw_from_deg": round(np.degrees(kt["q0"]), 1),
                                        "screw_to_deg": round(np.degrees(m.q), 1),
                                        "wrist_deg": round(np.degrees(kt["phi"]), 1),
                                        "approach_az_from_deg": round(np.degrees(kt["az0"]), 1)})
                if goal or self.strokes >= MAX_STROKES:
                    self.goto("snug_hold", t)
                else:
                    self.k_swing = -sgn * np.pi / 2       # back 90 deg in the air
                    self.goto("s_unpress", t)
        elif ph == "snug_hold":
            p, q = self.p_cmd, self.q_cmd                # hold the snugged pose
            if t - self.t0 >= T["snug_hold"]:
                if self.mode == "tighten" and self.seated:
                    pre = float(np.clip(PRELOAD_FRAC * self.tau_seat, PRELOAD_MIN, PRELOAD_MAX))
                    self.thread.set_preload(pre, m.q)
                    print(f"    seated: seating torque {self.tau_seat*1000:.1f} mN.m -> "
                          f"breakaway preload {pre*1000:.1f} mN.m")
                self.goto("end_unpress", t)
        elif ph == "s_unpress":
            # press off, and unwind the grasp a few degrees inside the tip's
            # +-6 deg backlash: lifting a tip that still carries the stroke's
            # torque drags the screw up its thread by friction (measured at
            # the seat: 41 N, 81 mN.m and the head pulled 0.6 mm up)
            u = smooth5(self.s(t, T["s_unpress"]))
            p, q = self.drive_pose(0.2, self.phi_end - sgn * UNWIND * u, gripped=False)
            if t - self.t0 >= T["s_unpress"] and self.mode == "loosen" and m.q >= LOOSE_GOAL:
                # the screw ran on past the goal as the stroke's wind-up let go:
                # done -- the press is off and the grasp unwound, pull it out
                self.goto("extract", t)
            elif t - self.t0 >= T["s_unpress"]:
                self.goto("s_lift", t)
        elif ph == "s_lift":
            p0 = self.k.setdefault("p0", self.p_cmd.copy())
            u = smooth5(self.s(t, T["s_lift"]))
            lift = LIFT_OUT if self.k_swing != 0.0 else 0.004
            p = p0 + np.array([0.0, 0.0, lift * u])
            if t - self.t0 >= T["s_lift"]:
                self.goto("s_swing" if self.k_swing != 0.0 else "s_insert", t)
        elif ph == "s_swing":
            # in the air: the hand turns about the TOOL's own (vertical) axis
            if "p0" not in self.k:
                self.k.update(p0=self.p_cmd.copy(), R0=R_of_q(self.q_cmd), c=m.tip.copy())
            dphi = self.k_swing
            dur = max(0.6, T["s_swing90"] * abs(dphi) / (np.pi / 2))
            u = smooth5(self.s(t, dur))
            Rz = expmap(np.array([0.0, 0.0, dphi * u]))
            c = self.k["c"]
            p = c + Rz @ (self.k["p0"] - c)
            q = quat_of_R(Rz @ self.k["R0"])
            if t - self.t0 >= dur:
                self.goto("s_insert", t)
        return p, q, f


# ── verdict ─────────────────────────────────────────────────────────────
def score(rig, h, expert, thread, log, paths, mode, name):
    qa = rig.qa(paths)
    L = {k: np.array([r[k] for r in log], float) for k in LOG_KEYS}
    t = L["t"]
    ts = expert.t_start
    weight = 9.81 * float(rig.model.body_mass.numpy()[rig.body_index("tool")])
    crit = {}

    def win(a, b):
        return (t >= a) & (t <= b)

    # picked up from the table with real pad contact
    rest0 = win(0.2, ts["descend"])
    carry = win(ts["lift"] + 0.3, ts["s_insert"])
    lift_mm = float((L["low"][win(ts["lift"], ts["s_insert"])].max() - L["low"][rest0].mean()) * MM)
    both = float(((L["padL"][carry] > 0.5) & (L["padR"][carry] > 0.5)).mean())
    table_after = float(L["table"][win(ts["lift"] + 0.5, ts["s_insert"])].max())
    crit["picked_up_from_table"] = {
        "pass": bool(abs(L["table"][rest0].mean() - weight) < 0.3 * weight and lift_mm > 40.0
                     and both > 0.98 and table_after < 0.01),
        "table_support_at_start_N": round(float(L["table"][rest0].mean()), 4),
        "tool_weight_N": round(weight, 4), "lift_of_lowest_point_mm": round(lift_mm, 1),
        "both_pads_loaded_fraction_carry": round(both, 3),
        "pad_force_mean_carry_N": [round(float(L["padL"][carry].mean()), 2),
                                   round(float(L["padR"][carry].mean()), 2)],
        "table_contact_during_carry_max_N": round(table_after, 4),
        "note": "a fingertip pinch on the two side flats of the handle; no joint, weld or "
                "kinematic attachment exists between the robot and the tool"}

    # the task itself
    t_end_turn = ts.get("end_unpress", ts.get("extract", t[-1]))
    t_probe = expert.t_probe or t[-1]
    pre = (t >= t_probe - 0.3) & (t < t_probe)
    q_deg = np.degrees(L["q"])
    ang_end = float(q_deg[pre].mean()) if pre.any() else float(q_deg[-1])
    turn_m = win(ts.get("s_turn", t_end_turn), t_end_turn)
    turning = turn_m & (L["phase"] == PHASES.index("s_turn"))
    tau_pk = float(np.abs(L["tau"][turning]).max()) if turning.any() else 0.0
    if mode == "tighten":
        crit["screw_tightened"] = {
            "pass": bool(expert.seated and ang_end < np.degrees(SEAT_TOL) + 0.5
                         and float(q_deg[0] - ang_end) > 600.0
                         and abs(float(L["z"][pre].mean())) < 0.0002),
            "start_deg_from_seat": round(float(q_deg[0]), 1),
            "start_head_rise_mm": round(float(L["z"][0]) * MM, 3),
            "turned_deg": round(float(q_deg[0] - ang_end), 1),
            "angle_from_seat_after_release_deg": round(ang_end, 3),
            "head_rise_after_release_mm": round(float(L["z"][pre].mean()) * MM, 4),
            "strokes": expert.strokes, "stroke_log": expert.stroke_log,
            "seating_torque_peak_mNm": round(expert.tau_seat * 1000, 2),
            "tip_torque_peak_incl_snug_hold_mNm": round(float(np.abs(L["tau"][turn_m]).max()) * 1000, 2)
            if turn_m.any() else None,
            "breakaway_preload_mNm": round(float(L["preload"][pre].mean()) * 1000, 2),
            "note": "starts with the threads unengaged: head 4.0 mm above the seat = 1.71 "
                    "turns of the 2.34 mm right-hand thread; seat = hard stop q = 0"}
        moves = {}
        for k, lbl in ((0, "loosen_way"), (1, "tighten_way")):
            w = win(t_probe + k * T_PROBE, t_probe + (k + 1) * T_PROBE)
            moves[lbl] = round(float(np.ptp(q_deg[w])) if w.any() else 0.0, 4)
        crit["stationary_under_small_torque"] = {
            "pass": bool(max(moves.values()) < 1.0), "probe_torque_mNm": PROBE_TAU * 1000,
            "rotation_deg": moves, "threshold_deg": 1.0, "window_s": T_PROBE,
            "note": "applied to the screw dof after the robot and the tool had left; "
                    "loosen-way held by the seat preload model, tighten-way by the seat"}
    else:
        backed = float(ang_end - q_deg[0])
        crit["screw_loosened"] = {
            "pass": bool(backed >= 180.0 and expert.released and tau_pk > LOOSE_PRELOAD0),
            "start_deg_from_seat": round(float(q_deg[0]), 2),
            "backed_off_deg_after_release": round(backed, 1),
            "head_rise_after_release_mm": round(float(L["z"][pre].mean()) * MM, 3),
            "breakaway_preload_mNm": LOOSE_PRELOAD0 * 1000,
            "peak_drive_torque_at_tip_mNm": round(tau_pk * 1000, 2),
            "preload_released": bool(expert.released),
            "strokes": expert.strokes, "stroke_log": expert.stroke_log,
            "threshold_deg": 180.0}
        moves = {}
        for k, lbl in ((0, "loosen_way"), (1, "tighten_way")):
            w = win(t_probe + k * T_PROBE, t_probe + (k + 1) * T_PROBE)
            moves[lbl] = round(float(abs(q_deg[w][-1] - q_deg[w][0])) if w.any() else 0.0, 3)
        crit["rotates_freely_under_small_torque"] = {
            "pass": bool(min(moves.values()) > 5.0), "probe_torque_mNm": PROBE_TAU * 1000,
            "rotation_deg_in_1s": moves, "threshold_deg": 5.0,
            "note": "applied to the screw dof after the robot and the tool had left"}

    # the tool did the turning: pads loaded, torque through the tip
    pads_on = float(((L["padL"][turning] > 0.5) & (L["padR"][turning] > 0.5)).mean()) \
        if turning.any() else 0.0
    crit["turned_by_the_tool_in_hand"] = {
        "pass": bool(tau_pk > 0.003 and pads_on > 0.98),
        "peak_tip_torque_mNm": round(tau_pk * 1000, 2),
        "both_pads_loaded_fraction_while_turning": round(pads_on, 3),
        "strokes": expert.strokes, "insertions": expert.insert_log,
        "note": "the screw only turns through the tool tip in the recess; the tool only "
                "moves through the fingertip pads; between strokes it is lifted out in the "
                "grip, swung back in the air and re-inserted"}

    # tool returned to the table
    t_end = float(t[-1])
    t_rel = expert.t_release or t_end
    fin = win(ts.get("still", t_end - 1.0), t_end)
    d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
    labels = list(d["body_labels"])
    ti = labels.index("tool")
    tq = d["body_q"][:, ti]
    fm = d["t"] >= ts.get("still", t_end - 1.0)
    v_tool = float(np.linalg.norm(np.diff(tq[fm][:, :3], axis=0), axis=-1).max()) * rig.fps \
        if fm.sum() > 1 else 9.9
    start_xy = np.array([L["px"][0], L["py"][0]])
    end_xy = np.array([L["px"][fin][-1], L["py"][fin][-1]])
    dist = float(np.linalg.norm(end_xy - start_xy))
    tilt_end = float(L["tilt"][fin][-1])
    crit["tool_returned_to_table"] = {
        "pass": bool(abs(L["table"][fin].mean() - weight) < 0.3 * weight
                     and L["block"][fin].max() < CONTACT_FREE_N and v_tool < 0.002
                     and dist < 0.040 and abs(tilt_end - 90.0) < 10.0),
        "table_support_final_N": round(float(L["table"][fin].mean()), 4),
        "tool_weight_N": round(weight, 4),
        "tool_block_contact_final_max_N": round(float(L["block"][fin].max()), 4),
        "tool_speed_final_max_mm_s": round(v_tool * MM, 3),
        "distance_from_start_pose_mm": round(dist * MM, 1),
        "tool_axis_from_vertical_deg": round(tilt_end, 2),
        "note": "lying on the table (support = weight), not on the block, at rest, within "
                "40 mm of where it was picked up"}

    # released and clear, robot stopped
    i_rel = int(np.searchsorted(t, t_rel))
    tcp = np.column_stack([L["tcpx"], L["tcpy"], L["tcpz"]])
    moved = float(np.linalg.norm(tcp[-1] - tcp[min(i_rel, len(t) - 1)]))
    t_clear = ts.get("still", t_end - 1.0)
    clear = t >= t_clear
    peak = float(L["robot_obj"][clear].max()) if clear.any() else 9.9
    crit["released_and_clear"] = {
        "pass": bool(peak < CONTACT_FREE_N and moved >= 0.050),
        "window_s": [round(t_clear, 2), round(t_end, 2)],
        "robot_object_contact_peak_N": round(peak, 4), "threshold_N": CONTACT_FREE_N,
        "tcp_moved_from_release_mm": round(moved * MM, 1), "min_move_mm": 50.0,
        "objects": "tool, screw, block"}
    arm_ix = [i for i, l in enumerate(labels) if str(l).startswith("fr3/")]
    v_arm = float(np.linalg.norm(np.diff(d["body_q"][fm][:, arm_ix, :3], axis=0),
                                 axis=-1).max()) * rig.fps if fm.sum() > 1 else 9.9
    crit["robot_stopped"] = {"pass": bool(v_arm < 0.005),
                             "max_link_speed_final_window_mm_s": round(v_arm * MM, 3),
                             "threshold_mm_s": 5.0}
    off = t >= ts.get("carry_back", t_end)
    crit["tool_off_screw"] = {
        "pass": bool(L["Fd"][off].max() < CONTACT_FREE_N) if off.any() else False,
        "tool_screw_contact_after_extraction_peak_N": round(float(L["Fd"][off].max()), 4)
        if off.any() else None}

    for k_, c_ in crit.items():
        qa.report["checks"][k_] = c_
    # penetration: tool tip <-> screw (both ways), tool <-> block halves, tool <-> table top
    drv_local = h["drv_tm"].copy()
    drv_local.apply_translation(-h["drv_c"])
    scr_local = h["scr_tm"].copy()
    scr_local.apply_translation(-h["scr_c"])
    tip_pts = drv_local.vertices[drv_local.vertices[:, 1] < TIP_Y + 0.008]
    head_pts = scr_local.vertices[scr_local.vertices[:, 2] > 0.0045]
    qa.check_penetration(
        [("tool", trimesh.Trimesh(vertices=tip_pts, process=False), "screw", scr_local),
         ("screw", trimesh.Trimesh(vertices=head_pts, process=False), "tool", drv_local)],
        tol=0.0015)
    for part in ("block_top", "block_bottom"):
        tm, c = rig.meshes[part]
        qa.check_penetration_static("tool", drv_local, tm, c, f"tool->{part}", tol=0.0015)
    vlow = drv_local.vertices[::7]
    zmin = 9.0
    for f_ in range(0, len(d["t"]), 3):
        Tq = world_T(tq[f_])
        zmin = min(zmin, float(((Tq[:3, :3] @ vlow.T).T + Tq[:3, 3])[:, 2].min()))
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["tool->table_top"] = {"min_vertex_z_mm": round(zmin * MM, 3),
                                       "pass": bool(zmin > -0.0010)}
    pen["pass"] = bool(pen["pass"] and zmin > -0.0010)
    qa.check_envelope(["tool", "screw"], np.array([0.20, -0.30, -0.01]),
                      np.array([0.72, 0.30, 0.35]))
    qa.report["checks"]["expert_info"] = {
        "pass": True, "info_only": True,
        "phase_start_s": {k: round(v, 2) for k, v in ts.items()},
        "insertions": expert.insert_log,
        "thread_friction_peak_mNm": round(thread.peak_fric * 1000, 2)}
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    title = ("Tighten screw with screwdriver" if mode == "tighten"
             else "Loosen screw with screwdriver") + " (tool picked up from the table)"
    spec = ("The screw is tightened and remains stationary when a small force is applied in "
            "an attempt to rotate the screw. The robot movement stops after terminating the "
            "tightening motion, moving away from, and breaking contact with the screw."
            if mode == "tighten" else
            "The screw is loosened and can rotate freely when a small force is applied in an "
            "attempt to rotate the screw. The robot movement stops after terminating the "
            "loosening motion, moving away from, and breaking contact with the screw.")
    changes = [
        "new scene (finalized-kit yellow block + round-head screw with a 5.5 mm square "
        "recess; the screwdriver mesh = kit screwdriver), the screwdriver a free body",
        "user addition: the screwdriver starts lying on the table; it is picked up by a "
        "fingertip pinch, reoriented tip-down, used, and laid back on the table, released, "
        "and the hand moves away",
        ("start: threads unengaged -- head 4.0 mm above the seat, 615 deg of a 2.34 mm "
         "right-hand thread" if mode == "tighten" else
         "start: seated with a 20 mN.m breakaway preload"),
        "thread model: helix spring between the screw's axial and rotary dofs, "
        "load-dependent elasto-plastic thread friction, hard seat stop, seat preload from "
        "the measured seating torque (_screwdriver.py's model), probe torque",
        "one grasp from pick to put-back: the picked tool is pitched 90 deg into a side "
        "grasp (tip down) and driven from it -- each stroke turns the hand pose about the "
        "screw axis inside the IK window of approach azimuths 35-145 deg; between strokes "
        "the tool is lifted out in the grip, swung back 90 deg in the air and re-inserted",
        f"probe: +-{PROBE_TAU*1000:.0f} mN.m on the screw after the robot stopped",
    ]
    return save_appendix_verdict(rig, task=title, source="new task (kit assets); mechanism "
                                 "model after force_bench.appendix_tasks._screwdriver",
                                 spec=spec, criteria=crit, physics_qa=report, changes=changes)
