"""Shared scene, mechanism model and expert for the two wrench tasks that take
the wrench from the hook shelf (appendix Table I, #17 / #18, plus the user's
2026-10-16 additions: start by taking the wrench off the hanging board, end
by hanging it back).

    Tighten nut with wrench.  "A nut is threaded onto a screw head in the
    workspace but is left untightened.  Use the wrench grasped in hand to
    tighten the nut."  Success: "The nut is tightened and remains stationary
    when a small force is applied in an attempt to rotate the nut.  The robot
    movement stops after terminating the tightening motion, moving away from,
    and breaking contact with the nut."
    Loosen nut with wrench.  "A nut is threaded onto a screw head in the
    workspace and is fully tightened.  Use the wrench grasped in hand to
    loosen the screw [nut]."  Success: "The nut is loosened and can rotate
    freely when a small force is applied in an attempt to rotate the nut.
    The robot movement stops after terminating the loosening motion, moving
    away from, and breaking contact with the nut."

SCENE (every part is finalized-kit CAD, metres)
------------------------------------------------
* the bench block, hanging board and robot cell of ``_hang_plate`` (bench
  top Z0 = 0.12 m, FR3 on a pedestal at (0.45, 0.25) yawed at the hook);
* the kit wrench (``bodies_kit/wrench``, PLA 13.5 g) HANGING on the middle
  hook at its free-hang pose (hole on the flat bar, tilt 90 deg) -- the end
  state of ``hang_wrench``;
* the kit bolt block (``bodies_kit/bolt_block``): the 80 x 30 x 20 mm toy
  block with the 0.75mm_screw's hex head captive in its bottom and the
  threaded shaft standing up through the top (thread z 18.5..43.45 mm, the
  block top at z 20 is the nut's seat -- there is NO bolt head under the
  nut for the jaw to hit, which was one of the two causes of the suite
  wrench task's failure);  the free 0.75mm_nut on the real SDF thread.
  Measured off the meshes (scratch scans, 2026-10-16): single-start RH
  helix, lead 2.332 mm/turn (the suite's thread), 0.25 mm radial clearance
  at the mating yaw  yaw(z_top) = 258.5 deg + (z_top - 30.00 mm) / 2.332 mm
  x 360 deg  (nut frame = nut.stl's own STEP frame, z_top = its top face);
  seated = nut bottom on the block top (z_top 27.01 mm, yaw 156.9 deg).

THE WRENCH (measured off the kit mesh with wrench_nut_arm.measure_jaw)
----------------------------------------------------------------------
16.15 mm across the jaw on the 15.50 mm nut (0.65 mm clearance), jaw axis
10.1 deg off the shank, parallel jaw faces from 2 mm behind to 7 mm in front
of the measured seat (the nut is centred SEAT_X = 3 mm out, its throat-side
corner then 1.7 mm clear of the throat).  Balance point 46.1 mm from the nut
centre.  The pads pinch the 11.96 mm handle ACROSS ITS WIDTH at the balance
point (hang_wrench's grasp, but 5 mm deeper so the pads cover the whole
7.5 mm edge), so the swing load lands on pad NORMAL force and gravity puts no
moment on the pinch in any attitude.  The jaw is 7.5 mm thick and the nut
7.01 mm: the jaw plane is held at Z_JAW (its bottom face 1.0 mm above the
block top), which still covers 6.0 mm of a seated nut.

MEASURED WHILE BUILDING IT (scratch runs, 2026-10-16; each cost a run)
----------------------------------------------------------------------
* The free nut spins on its thread at ~0.05 N of jaw contact, so a jaw that
  touches it before the swing turns it -- and an angle servo that chases the
  turning nut is positive feedback (35 deg of rotation in one slide-on).  A
  square, centred jaw slides on with 0.32 mm per side and never touches it;
  the slide-on therefore advances only while the MEASURED jaw is square and
  centred on the MEASURED nut, slowly inside 11 mm, and backs off on touch.
  (Two bugs hid behind this: the nut's helix yaw left out of the flat phase,
  and a lateral error that mixed commanded and measured jaw angles.)
* The thread has 0.6 mm of axial backlash: a nut on its seat turns ~90 deg
  FREE before the clamp builds, so "seated" is decided on the measured jaw
  torque, not the height.  Then the torque rises steeply (~7 -> 36 mN.m in
  ~4 deg), and the seat carries ~4 N of clamp after the wrench leaves.
* The pinch carries the wrench's weight in friction in every attitude, and
  MuJoCo friction creeps under a steady load: with hang_wrench's grasp
  (tips 2 mm short of the far face, impratio 10) the wrench slid 5.9 mm
  toward the tips and pitched 8.5 deg over 42 s, then fell out in the
  re-hang.  Pads over the whole edge + impratio 20: < 1 deg over the task.
* Flattening the wrench beside the board swung its hole end onto the hook
  (25 deg knocked into the pinch): the carry first moves 60 mm off the board.
* With the nut at x = 0.02 the hand at the balance point sat under the hook
  tips (668 N hand<->hook, arm blocked 12 mm short): the nut is at x = 0.06.

MECHANISM MODEL (reduced, documented)
-------------------------------------
* Thread, seat and jaw are real SDF contact; the nut is a FREE body.
* SEAT PRELOAD = elasto-plastic stiction on the nut's free-joint world-z
  torque dof, written to Control.joint_f every substep (``NutSeat``): a
  bristle spring STICTION_K anchored where the nut sticks, capped at the
  breakaway torque (the anchor slides when the cap is exceeded).  MuJoCo's
  contact friction is a soft constraint and cannot hold a wedged nut
  statically by itself (an unpreloaded seated nut spun ~1000 deg under a
  2 mN.m probe in the suite task).
    tighten: no preload while the nut runs down; when the expert's
             torque-up stalls (jaw torque >= TAU_TIGHT measured from the
             jaw<->nut contact forces, nut seated) the breakaway becomes
             PRELOAD_FRAC x the peak seated jaw torque (clamped);
    loosen:  starts seated with LOOSE_PRELOAD breakaway (what a ~30 mN.m
             tightening leaves), released once the nut has backed off
             LOOSE_FREE_DEG.
* THE PROBE ("a small force applied in an attempt to rotate the nut"),
  after the robot has hung the wrench back and stopped: a torque-limited
  drag on the same dof -- a reference angle advancing at PROBE_RATE,
  PD-tracked with |torque| <= PROBE_TAU -- 1 s loosening way, then 1 s
  tightening way.  A tight nut saturates the probe and stays put; a loose
  one follows the reference at a fraction of the cap.

Nothing is welded: the wrench is carried and turned by fingertip friction
alone, the nut only through the jaw.  The expert reads privileged state
(poses, contact forces) for closed-loop control only.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import world_T
from force_bench.rig import make_solver
from force_bench.recorder import RolloutRecorder
from force_bench.tasks.screwdriver import DriveContacts
from force_bench.tasks.nut_bolt_arm import YawTrack
from force_bench.tasks.wrench_nut_arm import measure_jaw, hex_flat_phase
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, save_appendix_verdict, StopRun,
    check_released_and_clear, CONTACT_FREE_N)
from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._hang_plate import (
    MM, Plate, HookPath, HangCfg, HangExpert, HANG_PHASES, DroopComp, JointTracker,
    PairProbe, hang_frame, tcp_pose_for, smooth5, pierce, quat_of_R, R_of_q, unit,
    rest_drop, Z0)
from force_bench.appendix_tasks.hang_wrench import handle_halfwidths, slerp


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


KIT = HP.KIT_DIR

# ── the hook ────────────────────────────────────────────────────────────
PEG_Y = HP.PEG_YS[1]                 # middle hook (where hang_wrench hangs it)
BAR_X = -0.1070                      # hang station on the flat bar
CFG = HangCfg(tilt_hi=np.radians(86.0), bar_x=BAR_X, hover_u=-0.028)

# ── the wrench (hang_wrench's values) ───────────────────────────────────
DENSITY = 1240.0                     # PLA -> 13.5 g
WRENCH_MU = 0.5
W_SUBDIV = 0.0020                    # vertex density (pitfall 3.2); 2 mm for the jaw
W_SDF = dict(sdf=True, sdf_resolution=768, narrow_band=0.0015, margin=0.0005)
                                     # 0.14 mm voxels: the jaw clearance is
                                     # 0.33 mm per side (wrench_nut_arm)
GAP_W = 0.0005
REST_CLEAR = 0.00005
HAND_FLIP = True                     # hang_wrench: joint 7 margin through the bend
IMPRATIO = _env("WH_IMPRATIO", 20.0)    # 10 in hang_wrench; the pinch here is held ~45 s
                                     # and creeps under the weight (measured 0.14 mm/s at 10)

# ── the bolt block and the nut ──────────────────────────────────────────
NUT_AXIS = np.array([_env("WH_NUT_X", 0.060), _env("WH_NUT_Y", 0.000)])
                                     # 60 mm out: at x 0.02 the hand at the balance
                                     # point sat under the hook tips (measured 668 N
                                     # hand<->hook, arm blocked 12 mm short)
BLOCK_TOP = Z0 + 0.0200              # nut seat (block top face)
BOLT_AXIS_KIT = np.array([-0.0250, 0.0])   # bolt axis in the block's STEP frame
LEAD = 0.002332                      # m/turn, right hand (z rises with CCW yaw)
MATE_ZTOP, MATE_YAW = 0.0300, 258.5  # nut top z (block frame) <-> mating yaw (deg)
NUT_H = 0.00701
NUT_TOP_OFF = 0.0035115              # nut AABB centre -> top face (mesh bounds)
NUT_MASS = 0.010                     # the suite's validated value for this mesh
                                     # pair (PLA would be 1.15 g: the explicit
                                     # stiction element would then exceed its
                                     # stability bound k dt^2 / I ~ 5)
NUT_MU, BOLT_MU, BLOCK_MU = 0.30, 0.30, 0.50
F_SUBDIV = 0.0020
F_SDF = dict(sdf=True, sdf_resolution=512, narrow_band=0.0010, margin=0.0001)
B_SDF = dict(sdf=True, sdf_resolution=384, narrow_band=0.0015, margin=0.0005)
GAP_F = 0.0003
SEAT_TOL = 0.00012                   # nut bottom this close to the block top = seated

# ── jaw geometry / stroke ladder ────────────────────────────────────────
SEAT_X = 0.0030                      # nut centre in the measure_jaw frame (+x = mouth)
JAW_CLEAR = 0.0010                   # jaw bottom face above the block top
Z_JAW = BLOCK_TOP + 0.00375 + JAW_CLEAR
CLEAR_E = 0.020                      # radial retreat: arm tips 19 mm from the axis
STROKE = np.radians(60.0)            # hex index
ALIGN_TOL = 1.5                      # deg, gate at first contact (slide-on 2.1 deg)
ALIGN_RATE = np.radians(0.4)         # rad/frame cap on the angle servo
K_ALIGN = 0.15
LAT_TOL = 0.00015                    # m lateral jaw error before sliding on
F_SLIDE = 1.5                        # N jaw<->nut force that pauses the slide-on
F_TOUCH = 0.20                       # N: any jaw<->nut contact before the swing means
                                     # off-square/off-centre -> back off 1 mm, re-align
                                     # (a free nut on its thread spins at ~0.05 N)
E_SLOW = 0.011                       # m: inside this the arms straddle the flats
V_ENG_FAST, V_ENG_SLOW = 0.020, 0.008   # m/s slide-on
TOOL_GAIN, TOOL_RATE, TOOL_MAX = 0.10, 0.00006, 0.006
T_ENG, T_SWING, T_OFF, T_IDX = 1.4, 2.0, 0.6, 1.1
N_MAX_STROKES = _env("WH_MAX_STROKES", 7)

# torque-up (tighten)
TAU_TIGHT = _env("WH_TAU_TIGHT", 0.030)     # N.m measured jaw torque = "tight"
TAU_ONSET = 0.006                    # N.m (2 frames): the clamp has started to build.  A nut on
                                     # its seat first turns ~90 deg FREE while the
                                     # thread's 0.6 mm axial backlash is taken up
                                     # (measured), so height alone does not say tight
TORQUE_RATE = np.radians(8.0)        # rad/s wrist advance once the clamp builds
SEATED_RATE = np.radians(25.0)       # rad/s cap while a seated nut takes up its backlash
N_STALL = 6                          # frames above TAU_TIGHT
PRELOAD_FRAC, PRELOAD_MIN, PRELOAD_MAX = 0.7, 0.008, 0.040
# loosen
LOOSE_PRELOAD = 0.020                # N.m breakaway of the tight start
LOOSE_FREE_DEG = 15.0
LOOSEN_DONE_DEG = _env("WH_LOOSEN_DEG", 100.0)
STICTION_K = 1.0                     # N.m/rad bristle (k dt^2 / I = 0.6)

# probe
PROBE_TAU = 0.005                    # N.m cap
PROBE_RATE = np.radians(60.0)        # rad/s reference drag
PROBE_KP, PROBE_KD = 0.20, 6.0e-4    # PD on the nut yaw (explicit, per substep)
T_PROBE = 1.0
STILL_MAX_DEG = 1.0                  # "stationary"
FREE_MIN_DEG = 30.0                  # "rotates freely" (half the reference)

# ── fingers (GelSight readiness: every target is a named constant) ─────
TIP_CLEAR = -0.0030                  # fingertips 3 mm PAST the wrench's far face, so the
                                     # pads cover the whole 7.5 mm edge: this pinch carries
                                     # the weight in friction in every attitude and creeps
                                     # toward the tips (measured 5.9 mm over 42 s with the
                                     # hang_wrench grasp, whose tips stop 2 mm short of the
                                     # far face; the wrench then slid out)
SQUEEZE = 0.0008                     # initial finger target inside the handle
OPEN_EXTRA = 0.0070                  # pad clearance per side when open
GRIP_LO, GRIP_HI = 6.0, 9.0          # N/pad admittance band (padL/padR mean)
GRIP_RATE = 0.000008                 # m/frame finger-target step
SQUEEZE_MIN, SQUEEZE_MAX = 0.0002, 0.0020

# ── timings ─────────────────────────────────────────────────────────────
T_SETTLE, T_APPROACH, T_PRE, T_CLOSE, T_HOLD = 0.6, 1.2, 0.4, 0.6, 0.4
UNSEAT_RATE, T_UNSEAT_MAX = 0.0015, 2.5
V_PATH, V_BEND, A_PATH = 0.012, 0.0035, 0.030
T_CARRY, CARRY_UP, T_ABOVE, T_DESC, T_JAW_SETTLE, T_LOWER = 2.2, 0.035, 0.4, 1.0, 0.4, 0.6
T_AWAY, AWAY = 1.0, np.array([0.060, 0.0, 0.0])   # off the hook, AWAY from the
                                     # board before the wrench is flattened: flattening
                                     # beside the board swings the hole end down onto
                                     # the hook (measured: 25 deg knocked into the pinch)
T_ENG_HOLD = 0.3                     # static servo hold before every slide-on
LOWER_FROM = 0.0030
T_LIFT, LIFT, T_LIFT_HOLD, T_CARRY_BACK = 1.0, 0.045, 0.3, 2.4
T_BUDGET = 150.0


def kit_T(bq_row, center):
    """kit->world 4x4 of a body whose mesh was centred at `center`."""
    T = world_T(bq_row)
    T[:3, 3] = T[:3, 3] - T[:3, :3] @ np.asarray(center)
    return T


def pose_of_T(T):
    return T[:3, 3].copy(), quat_of_R(T[:3, :3])


def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Mode:
    name: str
    title: str
    spec: str
    tighten: bool
    z_above_seat: float              # nut start height above the seat
    theta_pref_handle_deg: float     # handle direction where strokes start
    changes: list = field(default_factory=list)


# ═════════════════════════════════════════════════════════════════════════
class NutSeat:
    """Seat preload + probe on the nut's free-joint world-z torque dof,
    written to Control.joint_f every SUBSTEP (solver wrapper)."""

    def __init__(self, solver, body, dof, sim_dt):
        self._solver = solver
        self.body, self.dof, self.dt = body, dof, sim_dt
        self.tau_b = 0.0
        self.anchor = 0.0
        self.prev = None
        self.yaw = 0.0
        self.yaw0 = None
        self.omega = 0.0
        self.tau = 0.0
        self.peak = 0.0
        self.released_at = None
        self.release_deg = None      # loosen: release once backed off this far
        self.probe_dir = 0
        self.probe_ref = 0.0
        self.tau_probe = 0.0

    def engage(self, tau_b):
        self.tau_b = float(tau_b)
        self.anchor = self.yaw

    def set_probe(self, direction):
        if direction != self.probe_dir:
            self.probe_dir = int(direction)
            self.probe_ref = self.yaw

    def step(self, state_in, state_out, control, contacts, dt):
        q = state_in.body_q.numpy()[self.body]
        raw = float(np.arctan2(2 * (q[6] * q[5] + q[3] * q[4]),
                               1 - 2 * (q[4] ** 2 + q[5] ** 2)))
        if self.prev is None:
            self.prev = raw
            self.yaw = self.anchor = 0.0
            self.yaw0 = 0.0
        d = (raw - self.prev + np.pi) % (2 * np.pi) - np.pi
        self.prev = raw
        self.yaw += d
        self.omega = d / dt
        if self.tau_b > 0.0:
            if self.release_deg is not None and \
                    np.degrees(self.yaw - self.yaw0) > self.release_deg:
                self.tau_b, self.tau = 0.0, 0.0
                self.released_at = float(np.degrees(self.yaw - self.yaw0))
            else:
                e = self.yaw - self.anchor
                if abs(STICTION_K * e) > self.tau_b:
                    self.anchor = self.yaw - np.sign(e) * self.tau_b / STICTION_K
                self.tau = -STICTION_K * (self.yaw - self.anchor)
                self.peak = max(self.peak, abs(self.tau))
        else:
            self.tau = 0.0
        if self.probe_dir:
            self.probe_ref += self.probe_dir * PROBE_RATE * dt
            self.tau_probe = float(np.clip(PROBE_KP * (self.probe_ref - self.yaw)
                                           - PROBE_KD * self.omega, -PROBE_TAU, PROBE_TAU))
        else:
            self.tau_probe = 0.0
        jf = control.joint_f.numpy()
        jf[self.dof] = self.tau + self.tau_probe
        control.joint_f.assign(jf)
        return self._solver.step(state_in, state_out, control, contacts, dt)

    def __getattr__(self, name):
        return getattr(self._solver, name)


# ═════════════════════════════════════════════════════════════════════════
class WrenchGeom:
    """Kit-frame measurements of the wrench: the hang plate (hole, balance
    point) and the open-end jaw (measure_jaw on the kit mesh)."""

    def __init__(self):
        raw = HP.load_kit_mesh("wrench")
        self.plate = Plate.measure("wrench", raw, DENSITY)
        pl = self.plate
        d_ang = np.arctan2(pl.d[1], pl.d[0])
        # measure_jaw expects the long axis on its `hi` axis with the jaw at
        # the low end: rotate the kit mesh so the hole->COM direction is -y
        Rz = trimesh.transformations.rotation_matrix(-np.pi / 2 - d_ang, [0, 0, 1])
        w2 = raw.copy()
        w2.apply_transform(Rz)
        self.jaw = measure_jaw(w2)
        assert abs(self.jaw["af"] - 0.01615) < 3e-4, "jaw geometry moved"
        self.T_jk = self.jaw["T"] @ Rz              # kit -> jaw frame
        self.T_kj = np.linalg.inv(self.T_jk)
        self.seat_kit = (self.T_kj @ np.array([SEAT_X, 0.0, 0.0, 1.0]))[:3]
        self.jx_kit = self.T_kj[:3, 0].copy()       # jaw +x (mouth) in kit
        self.jy_kit = self.T_kj[:3, 1].copy()
        w_pos, w_neg = handle_halfwidths(pl, pl.com[:2], pl.d)
        e_w = np.cross([0.0, 0.0, 1.0], pl.d)
        self.grasp_kit = np.array([*pl.com[:2], pl.z_mid]) + 0.5 * (w_pos - w_neg) * e_w
        self.half_w = 0.5 * (w_pos + w_neg)
        self.L_grip = float((self.grasp_kit - pl.hole) @ pl.d)
        self.off_w = float((self.grasp_kit - pl.hole) @ e_w)
        self.h_tcp = HP.TIP_BELOW_TCP + TIP_CLEAR - 0.5 * pl.thick
        self.raw = raw
        gj = (self.T_jk @ np.r_[self.grasp_kit, 1.0])[:3]
        self.grip_r = float(np.linalg.norm(gj[:2] - [SEAT_X, 0.0]))
        self.handle_off = float(np.arctan2(gj[1], gj[0] - SEAT_X))   # handle dir - jaw dir

    def flat_T(self, A, theta, e):
        """kit->world for the wrench lying flat (kit +z up), jaw +x at world
        angle theta, the nut axis point A at jaw-frame x = SEAT_X + e."""
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        T_wj = np.eye(4)
        T_wj[:3, :3] = R
        T_wj[:3, 3] = np.asarray(A, float) - R @ np.array([SEAT_X + e, 0.0, 0.0])
        return T_wj @ self.T_jk

    def hang_T(self, H, n, d):
        """kit->world with the hole centre at H, normal n, hang direction d."""
        R = self.plate.R_world(n, d)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = np.asarray(H, float) - R @ self.plate.hole
        return T

    def nominal_tcp(self, H, n, d):
        g = H + self.L_grip * d + self.off_w * np.cross(n, d)
        return tcp_pose_for(g, n, d, self.h_tcp, flip=HAND_FLIP)


# ═════════════════════════════════════════════════════════════════════════
def build(rig, mode: Mode, wg: WrenchGeom):
    """Scene: bench + board + robot (``_hang_plate``), the hung wrench, the
    bolt block with its nut.  Returns the handles the expert needs."""
    HP.setup_robot(rig)
    HP.build_bench_and_board(rig)
    b = rig.builder
    pl = wg.plate

    # ── bolt block: block top (collider + seat), bottom (visual), bolt ──
    off = np.array([NUT_AXIS[0] - BOLT_AXIS_KIT[0], NUT_AXIS[1] - BOLT_AXIS_KIT[1], Z0])
    top = trimesh.load(KIT / "bolt_block" / "block_top.stl", force="mesh")
    top.apply_translation(off)
    v, f = trimesh.remesh.subdivide_to_size(top.vertices, top.faces, max_edge=0.0025)
    top = trimesh.Trimesh(v, f, process=False)
    m_top, c_top = make_newton_mesh(top, **B_SDF)
    sid = b.add_shape_mesh(-1, mesh=m_top, xform=wp.transform(wp.vec3(*c_top), wp.quat_identity()),
                           cfg=rig.cfg(mu=BLOCK_MU, gap=0.0005, mu_torsional=HP.MU_TORSION),
                           label="block_top")
    rig.shapes["block_top"] = [sid]
    rig.meshes["block_top"] = (top, np.asarray(c_top))
    bot = trimesh.load(KIT / "bolt_block" / "block_bottom.stl", force="mesh")
    bot.apply_translation(off)
    m_bot, c_bot = make_newton_mesh(bot, sdf=False)
    b.add_shape_mesh(-1, mesh=m_bot, xform=wp.transform(wp.vec3(*c_bot), wp.quat_identity()),
                     cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False),
                     label="block_bottom")
    bolt = trimesh.load(KIT / "bolt_block" / "bolt.stl", force="mesh")
    bolt.apply_translation(off)
    v, f = trimesh.remesh.subdivide_to_size(bolt.vertices, bolt.faces, max_edge=F_SUBDIV)
    bolt = trimesh.Trimesh(v, f, process=False)
    m_bolt, c_bolt = make_newton_mesh(bolt, **F_SDF)
    sid = b.add_shape_mesh(-1, mesh=m_bolt, xform=wp.transform(wp.vec3(*c_bolt), wp.quat_identity()),
                           cfg=rig.cfg(mu=BOLT_MU, gap=GAP_F, mu_torsional=HP.MU_TORSION),
                           label="bolt")
    rig.shapes["bolt"] = [sid]
    rig.meshes["bolt"] = (bolt, np.asarray(c_bolt))

    # ── the nut: free body on the thread, helix-consistent pose ─────────
    nut = trimesh.load(KIT / "bolt_block" / "nut.stl", force="mesh")
    v, f = trimesh.remesh.subdivide_to_size(nut.vertices, nut.faces, max_edge=F_SUBDIV)
    nut = trimesh.Trimesh(v, f, process=False)
    flat_phase = hex_flat_phase(nut)
    m_nut, c_nut = make_newton_mesh(nut, **F_SDF)
    c_nut = np.asarray(c_nut, float)
    z_top = 0.0200 + NUT_H + mode.z_above_seat           # block frame
    yaw0 = np.radians(MATE_YAW + (z_top - MATE_ZTOP) / LEAD * 360.0)
    Rn = trimesh.transformations.rotation_matrix(yaw0, [0, 0, 1])[:3, :3]
    p_nut = np.array([NUT_AXIS[0], NUT_AXIS[1], Z0 + z_top]) + Rn @ c_nut
    q_nut = quat_of_R(Rn)
    nb = b.add_body(xform=wp.transform(wp.vec3(*p_nut), wp.quat(*q_nut)), label="nut")
    sid = b.add_shape_mesh(nb, mesh=m_nut, cfg=rig.cfg(mu=NUT_MU, gap=GAP_F,
                                                       mu_torsional=HP.MU_TORSION),
                           label="nut_shape")
    rig.shapes["nut"] = [sid]
    set_body_mass(b, nb, NUT_MASS)
    rig.meshes["nut"] = (nut, c_nut)

    # ── the wrench, hanging on the hook ─────────────────────────────────
    tm = trimesh.Trimesh(*trimesh.remesh.subdivide_to_size(
        wg.raw.vertices, wg.raw.faces, max_edge=W_SUBDIV), process=False)
    pl.tm = tm
    mesh, c = make_newton_mesh(tm, **W_SDF)
    pl.center = np.asarray(c, float)
    H0 = np.array([BAR_X, PEG_Y, Z0 + HP.BAR_Z - rest_drop(pl.r_hole) + REST_CLEAR])
    n0, d0 = hang_frame(np.pi / 2)
    p0, q0 = pl.body_pose(H0, n0, d0)
    wb = b.add_body(xform=wp.transform(wp.vec3(*p0), wp.quat(*q0)), label="wrench")
    sid = b.add_shape_mesh(wb, mesh=mesh, cfg=rig.cfg(mu=WRENCH_MU, gap=GAP_W,
                                                      mu_torsional=HP.MU_TORSION),
                           label="wrench_shape")
    rig.shapes["wrench"] = [sid]
    set_body_mass(b, wb, pl.mass)
    rig.meshes["wrench"] = (tm, pl.center)
    HP.filter_arm_links(rig, "wrench")
    HP.filter_statics_and_ground(rig)
    return {"flat_phase": flat_phase, "nut_c": c_nut, "z_top0": z_top, "yaw0": yaw0,
            "H0": H0}


# ═════════════════════════════════════════════════════════════════════════
class Expert:
    """The whole rollout as one phase machine (see the task modules' docs)."""

    PICK = ["settle", "approach", "pre", "close", "hold", "unseat", "unhook",
            "carry", "above", "descend", "jaw_settle", "lower"]

    def __init__(self, rig, mode: Mode, wg: WrenchGeom, h: dict, seat: NutSeat):
        self.rig, self.mode, self.wg, self.h, self.seat = rig, mode, wg, h, seat
        pl = wg.plate
        self.pl = pl
        self.wi = rig.body_index("wrench")
        self.ni = rig.body_index("nut")
        self.i7 = rig.body_index("fr3/fr3_link7")
        self.T_off = HP.tcp_offset_from_link7(rig)
        self.hook = HookPath(PEG_Y)
        self.poly = self.hook.polyline()
        self.weight = 9.81 * pl.mass
        self.f_touch = wg.half_w + HP.PAD_FACE_OFF
        self.f_cmd = self.f_touch + OPEN_EXTRA
        self.f_open = self.f_touch + OPEN_EXTRA
        self.dirn = -1.0 if mode.tighten else +1.0          # tighten = clockwise
        # a flat normal in the WORLD at t = 0: the mesh's own flat phase plus the
        # helix yaw the nut body was placed at (YawTrack counts from zero)
        self.flat_phase = h["flat_phase"] + h["yaw0"]
        self.nut_yaw = YawTrack()
        self.A = np.array([NUT_AXIS[0], NUT_AXIS[1], Z_JAW])
        th_pref = np.radians(mode.theta_pref_handle_deg) - wg.handle_off
        self.theta_pref = float(th_pref)
        # state
        self.k = "settle"
        self.t0 = 0.0
        self.t_start = {"settle": 0.0}
        self.finished = False
        self.droop = DroopComp()
        self.corr = np.zeros(3)
        self.X = None                                       # TCP in the wrench kit frame
        self.p_cmd = None
        self.q_cmd = None
        self.trk = None
        self.u = None
        self.v = 0.0
        self.ef = np.zeros(3)
        self.theta = self.theta_pref
        self.e = CLEAR_E
        self.ue = 0.0
        self.strokes = []
        self.stroke = None
        self.n_loaded = 0
        self.n_free = 0
        self.unseat_z = 0.0
        self.tilt_now = np.pi / 2
        self.grasp = None
        self.torque = {"mode": False, "n_hi": 0, "peak": 0.0, "stalled": False,
                       "t_stall": None, "preload": 0.0, "hold_until": None}
        self.loosen = {"start_yaw": None}
        self.hx = None
        self.lift_from = None
        self.jaw_hist = {"tau": 0.0, "F": 0.0}
        self.done_reason = None
        self.t_jaw_off = None
        self.first_contact_align = None
        self.hang_handover = None
        self.corr_hover = np.zeros(3)

    # ── measurement helpers ──────────────────────────────────────────────
    def meas(self):
        bq = self.rig.s0.body_q.numpy()
        T_wk = kit_T(bq[self.wi], self.pl.center)
        T_wt = world_T(bq[self.i7]) @ self.T_off
        return bq, T_wk, T_wt

    def latch_X(self, T_wk, T_wt):
        self.X = np.linalg.inv(T_wk) @ T_wt

    def tcp_of(self, T_wk_des):
        T = T_wk_des @ self.X
        return pose_of_T(T)

    def tcp_hang(self, H, th):
        """HangExpert's tcp_for: wrench with the hole at H and tilt th."""
        n, d = hang_frame(th)
        return self.tcp_of(self.wg.hang_T(H, n, d))

    def jaw_frame(self, T_wk):
        R = T_wk[:3, :3]
        x = R @ self.wg.jx_kit
        y = R @ self.wg.jy_kit
        S = (T_wk @ np.r_[self.wg.seat_kit, 1.0])[:3]
        return S, x, y, float(np.arctan2(x[1], x[0]))

    def engage_angle(self, nu, prefer):
        base = self.flat_phase + nu - np.pi / 2.0
        k = np.round((prefer - base) / (np.pi / 3.0))
        return float(base + k * np.pi / 3.0)

    def align_err(self, theta, nu):
        e = (theta + np.pi / 2.0) - (self.flat_phase + nu)
        return float((e + np.pi / 6.0) % (np.pi / 3.0) - np.pi / 6.0)

    def nut_bottom(self, bq):
        """Lowest nut z (the body origin is the AABB centre)."""
        return float(bq[self.ni, 2] - NUT_TOP_OFF)

    def go(self, k, t):
        self.k, self.t0 = k, t
        self.t_start[k] = t
        print(f"t={t:.2f}s -> {k}")

    # ── grip admittance on the pad pair forces (GelSight readiness) ─────
    def grip(self, padL, padR, active=True):
        if not active:
            return
        f = 0.5 * (padL + padR)
        lo_lim = self.f_touch - SQUEEZE_MAX
        hi_lim = self.f_touch - SQUEEZE_MIN
        if f < GRIP_LO:
            self.f_cmd = max(lo_lim, self.f_cmd - GRIP_RATE)
        elif f > GRIP_HI:
            self.f_cmd = min(hi_lim, self.f_cmd + GRIP_RATE)

    # ── the per-frame controller ─────────────────────────────────────────
    def control(self, t, probes):
        rig, wg, pl = self.rig, self.wg, self.pl
        fps = rig.fps
        bq, T_wk, T_wt = self.meas()
        tcp_meas = T_wt[:3, 3]
        H_meas, n_meas, d_meas = pl.frame_of(bq[self.wi])
        nu = self.nut_yaw.update(bq[self.ni])
        # the nut sits on its thread with 0.25 mm of radial play: aim at the
        # MEASURED nut centre, not the bolt axis
        self.A = np.array([bq[self.ni, 0], bq[self.ni, 1], Z_JAW])
        padL, padR = probes["padL"], probes["padR"]
        k, s = self.k, None
        dt_k = t - self.t0

        def frac(T):
            return min(1.0, dt_k / T)

        p = q = None
        fingers = self.f_cmd
        learn_droop = False
        if self.trk is None:
            self.trk = JointTracker(rig, rig.ik.joint_q.numpy()[0, :7])

        # ── 1. take the wrench off the hook ──────────────────────────────
        if k in ("settle", "approach", "pre", "close", "hold"):
            if self.grasp is None or k == "settle":
                # grasp computed from the MEASURED hanging pose
                tcp_g, q_g = wg.nominal_tcp(H_meas, n_meas, d_meas)
                self.grasp = (tcp_g, q_g, tcp_g + 0.060 * n_meas, n_meas.copy())
            tcp_g, q_g, tcp_park, n_g = self.grasp
            if k == "settle":
                fingers = 0.04 + (self.f_open - 0.04) * smooth5(frac(T_SETTLE))
                p, q = tcp_park, q_g
                learn_droop = True
                if dt_k >= T_SETTLE:
                    self.go("approach", t)
            elif k == "approach":
                fingers = self.f_open
                p, q = tcp_park + (tcp_g - tcp_park) * smooth5(frac(T_APPROACH)), q_g
                if dt_k >= T_APPROACH:
                    self.go("pre", t)
            elif k == "pre":
                fingers = self.f_open
                p, q = tcp_g, q_g
                learn_droop = True
                if dt_k >= T_PRE:
                    self.go("close", t)
            elif k == "close":
                f_close = self.f_touch - SQUEEZE
                self.f_cmd = self.f_open + (f_close - self.f_open) * smooth5(frac(T_CLOSE))
                fingers = self.f_cmd
                p, q = tcp_g, q_g
                if dt_k >= T_CLOSE:
                    self.go("hold", t)
            elif k == "hold":
                p, q = tcp_g, q_g
                self.grip(padL, padR)
                fingers = self.f_cmd
                if dt_k >= T_HOLD:
                    self.latch_X(T_wk, T_wt)
                    self.corr = self.droop.v.copy()
                    self.unseat_z = 0.0
                    self.tilt_now = float(np.arctan2(n_meas[0], n_meas[2]))
                    self.H_rest = H_meas.copy()
                    self.go("unseat", t)
            if k in ("settle", "approach", "pre", "close", "hold"):
                self.droop.update(learn_droop, p, tcp_meas,
                                  self.p_cmd if self.p_cmd is not None else p)
                p_out = p + self.droop.v
                self._out(p, q, p_out, fingers)
                return

        if k == "unseat":
            # raise the hole from its rest on the bar to the bar centre-line,
            # tilt from the measured hang to the path's bar tilt; force-gated
            u_bar = self.hook.u_of_bar_x(BAR_X)
            H_line = self.hook.point(u_bar)
            th_line = self.hook.tilt(u_bar, CFG.tilt_lo, CFG.tilt_hi)
            need = float(H_line[2] - self.H_rest[2])
            self.unseat_z = min(need, self.unseat_z + UNSEAT_RATE / fps)
            a = self.unseat_z / max(need, 1e-6)
            H_cmd = self.H_rest + (H_line - self.H_rest) * a
            th = self.tilt_now + (th_line - self.tilt_now) * a
            fz = probes["board_fz"]
            self.n_free = self.n_free + 1 if abs(fz) < 0.1 * self.weight else 0
            self._hole_loop(H_cmd, H_meas)
            p, q = self.tcp_hang(H_cmd, th)
            self.grip(padL, padR)
            self._out(p, q, p + self.corr, self.f_cmd)
            if (a >= 0.999 and self.n_free >= 3) or dt_k >= T_UNSEAT_MAX:
                self.u = u_bar
                self.v = 0.0
                self.go("unhook", t)
            return

        if k == "unhook":
            # the hang path backwards: bar -> bend -> stub -> past the tip
            F_board = probes["board"]
            gate = float(np.clip((CFG.f_gate_hi - F_board) / (CFG.f_gate_hi - CFG.f_gate_lo),
                                 0.15, 1.0))
            rem = max(0.0, self.u - CFG.hover_u)
            v_t = min(self._rev_speed(self.u),
                      np.sqrt(2.0 * A_PATH * rem) + 0.0005) * gate
            self.v = float(np.clip(v_t, self.v - A_PATH / fps, self.v + A_PATH / fps))
            self.u = max(CFG.hover_u, self.u - self.v / fps)
            H_cmd = self.hook.point(self.u)
            th = self.hook.tilt(self.u, CFG.tilt_lo, CFG.tilt_hi)
            self._hole_loop(H_cmd, H_meas)
            p, q = self.tcp_hang(H_cmd, th)
            self.grip(padL, padR)
            self._out(p, q, p + self.corr, self.f_cmd)
            if self.u <= CFG.hover_u + 1e-6:
                self.corr_hover = self.corr.copy()      # the arm's sag at the hover
                self.away_from = (p.copy(), q.copy())
                self.go("away", t)
            return

        if k == "away":
            # straight out from the board, hover attitude kept
            a = smooth5(frac(T_AWAY))
            p0, q0 = self.away_from
            p = p0 + AWAY * a
            self.grip(padL, padR)
            self._out(p, q0, p + self.corr, self.f_cmd)
            if dt_k >= T_AWAY:
                self.carry_from = (p.copy(), q0.copy())
                self.theta = self.engage_angle(nu, self.theta_pref)
                T_up = wg.flat_T(self.A + np.array([0, 0, CARRY_UP]), self.theta, CLEAR_E)
                self.carry_to = self.tcp_of(T_up)
                self.go("carry", t)
            return

        # ── 2. carry to the nut, lower the jaw beside it ─────────────────
        if k == "carry":
            a = smooth5(frac(T_CARRY))
            p0, q0 = self.carry_from
            p1, q1 = self.carry_to
            p = p0 + (p1 - p0) * a + np.array([0.0, 0.0, 0.02 * np.sin(np.pi * a)])
            q = slerp(q0, q1, a)
            self.grip(padL, padR)
            self._out(p, q, p + self.corr, self.f_cmd)
            if dt_k >= T_CARRY:
                self.go("above", t)
            return

        if k in ("above", "descend", "jaw_settle", "lower"):
            if k == "above":
                z_off = CARRY_UP
                if dt_k >= T_ABOVE:
                    # in-hand pose re-measured on a static, unloaded wrench
                    self._relatch(T_wk, T_wt)
                    self.go("descend", t)
            elif k == "descend":
                z_off = LOWER_FROM + (CARRY_UP - LOWER_FROM) * (1.0 - smooth5(frac(T_DESC)))
                if dt_k >= T_DESC:
                    self.go("jaw_settle", t)
            elif k == "jaw_settle":
                z_off = LOWER_FROM
                if dt_k >= T_JAW_SETTLE:
                    self.go("lower", t)
            else:
                z_off = LOWER_FROM * (1.0 - smooth5(frac(T_LOWER)))
                if dt_k >= T_LOWER:
                    self.ue = 0.0
                    self.first_contact_align = None
                    self.go("engage", t)
            A_des = self.A + np.array([0.0, 0.0, z_off])
            static = k in ("above", "jaw_settle")
            self._tool_servo(T_wk, A_des, self.theta, CLEAR_E, radial=static,
                             lateral=True, vertical=static)
            self.theta = self._align_step(T_wk, nu, self.theta)
            p, q = self.tcp_of(wg.flat_T(A_des, self.theta, CLEAR_E))
            self.grip(padL, padR)
            self._out(p, q, p + self.corr, self.f_cmd)
            return

        # ── 3. the stroke ladder ─────────────────────────────────────────
        if k in ("engage", "swing", "off", "index"):
            self._ladder(t, dt_k, T_wk, T_wt, bq, nu, probes)
            return

        # ── 4. lift off, carry back, hang ────────────────────────────────
        if k == "lift":
            a = smooth5(frac(T_LIFT))
            p0, q0 = self.lift_from
            p = p0 + np.array([0.0, 0.0, LIFT * a])
            self.grip(padL, padR)
            self._out(p, q0, p, self.f_cmd, corr_in=False)
            if dt_k >= T_LIFT:
                self.lift_top = (p.copy(), q0.copy())
                self.go("lift_hold", t)
            return

        if k == "lift_hold":
            p0, q0 = self.lift_top
            self.grip(padL, padR)
            self._out(p0, q0, p0, self.f_cmd, corr_in=False)
            if dt_k >= T_LIFT_HOLD:
                # in-hand pose re-measured on the static, unloaded wrench; the
                # hover target uses it, the sag measured at the hover on the
                # way out is the hang loop's starting correction
                self.latch_X(T_wk, T_wt)
                self.hx = HangExpert(CFG, self.hook, pl, self.tcp_hang,
                                     self.f_cmd, self.f_open, fps)
                p_h, q_h = self.hx.hover()
                p_h = p_h + self.corr_hover
                self.carry_back = (p0.copy(), q0.copy(), p_h + AWAY, q_h, p_h)
                self.go("carry_back", t)
            return

        if k == "carry_back":
            a = smooth5(frac(T_CARRY_BACK))
            p0, q0, p1, q1, _ = self.carry_back
            p = p0 + (p1 - p0) * a + np.array([0.0, 0.0, 0.03 * np.sin(np.pi * a)])
            q = slerp(q0, q1, a)
            self.grip(padL, padR)
            self._out(p, q, p, self.f_cmd, corr_in=False)
            if dt_k >= T_CARRY_BACK:
                self.go("toward", t)
            return

        if k == "toward":
            a = smooth5(frac(T_AWAY))
            _, _, p1, q1, p_h = self.carry_back
            p = p1 + (p_h - p1) * a
            self.grip(padL, padR)
            self._out(p, q1, p, self.f_cmd, corr_in=False)
            if dt_k >= T_AWAY:
                self.hx.f_close = self.f_cmd
                self.hx.begin(self.corr_hover)
                self.hang_handover = t
                self.go("dwell", t)
            return

        if k in HANG_PHASES:
            hx = self.hx
            dur = hx.durations()
            s = min(1.0, dt_k / dur[k]) if k in dur else 0.0
            p, q, fingers, _ = hx.step(k, s, H_meas, probes["board"], probes["board_fz"],
                                       self.p_cmd, self.q_cmd)
            if k in ("dwell", "thread", "bend", "seat", "seat_hold"):
                self.grip(padL, padR)
                hx.f_close = self.f_cmd
            self._out(p, q, p, fingers, corr_in=False)
            end = (dt_k >= dur[k]) if k in dur else hx.done(k)
            if end:
                i = HANG_PHASES.index(k)
                if i + 1 < len(HANG_PHASES):
                    self.go(HANG_PHASES[i + 1], t)
                else:
                    self.go("probe", t)
            return

        if k == "probe":
            self._out(self.p_cmd, self.q_cmd, self.p_out, self.fing_out, corr_in=False)
            if dt_k < T_PROBE:
                self.seat.set_probe(+1)            # loosening way (CCW)
            elif dt_k < 2 * T_PROBE:
                self.seat.set_probe(-1)            # tightening way
            else:
                self.seat.set_probe(0)
            if dt_k >= 2 * T_PROBE + 0.3:
                self.finished = True
            return

    # ── pieces ───────────────────────────────────────────────────────────
    def _rev_speed(self, u):
        """Path speed for the way OUT: slow only where the plate turns (the
        tilt ramp of HookPath.tilt, +-1 mm), fast on the straight bar and
        stub (HookPath.speed is slow on the whole bar)."""
        h = self.hook
        lo = h.u_bend0 + h.TILT_RAMP[0] - 0.001
        hi = h.u_bend1 + h.TILT_RAMP[1] + 0.001
        return V_BEND if lo <= u <= hi else V_PATH

    def _out(self, p, q, p_out, fingers, corr_in=True):
        """Record the nominal command and send p_out to the joint tracker."""
        self.p_cmd = np.asarray(p, float).copy()
        self.q_cmd = np.asarray(q, float).copy()
        self.p_out = np.asarray(p_out, float).copy()
        self.fing_out = float(fingers)
        self.trk.step(self.p_out, self.q_cmd, self.fing_out)

    def _hole_loop(self, H_cmd, H_meas):
        """HangExpert's hole-centre integral (rate-limited, bounded)."""
        c = CFG
        e = np.asarray(H_cmd, float) - np.asarray(H_meas, float)
        self.ef = self.ef + c.corr_alpha * (e - self.ef)
        ee = np.sign(self.ef) * np.maximum(np.abs(self.ef) - c.corr_db, 0.0)
        self.corr = np.clip(self.corr + np.clip(c.corr_k * ee, -c.corr_step, c.corr_step),
                            -c.corr_max, c.corr_max)

    def _relatch(self, T_wk, T_wt):
        """Re-measure the in-hand pose at the end of a STATIC, converged
        hold.  The correction becomes the measured arm sag (commanded minus
        measured TCP): the new mapping then commands the TCP pose that puts
        the wrench where the servo had already put it, so nothing jumps."""
        sag = self.p_out - T_wt[:3, 3]
        self.latch_X(T_wk, T_wt)
        self.corr = np.clip(sag, -TOOL_MAX, TOOL_MAX)

    def _tool_servo(self, T_wk, A_des, theta, e, radial, lateral, vertical):
        """Close the jaw seat on the MEASURED nut (wrench_nut_arm's tool-pose
        servo, here in all three jaw axes, each gated).  Errors are taken in
        the MEASURED jaw frame: the nut centre's offset from the jaw's own
        centre-line (lateral), its depth in the mouth minus the commanded
        retreat (radial), and the jaw plane height.  (Mixing in the commanded
        jaw angle biased the lateral error by e * sin(d_theta): 0.35 mm at
        1 deg and e = 20 mm, more than the 0.32 mm jaw clearance.)"""
        S, x, y, _ = self.jaw_frame(T_wk)
        rel = np.asarray(A_des, float) - S
        err_r = float(rel @ x) - e
        err_l = float(rel @ y)
        err_z = float(rel[2])
        dv = np.zeros(3)
        if radial:
            dv += np.clip(TOOL_GAIN * err_r, -TOOL_RATE, TOOL_RATE) * x
        if lateral:
            dv += np.clip(TOOL_GAIN * err_l, -TOOL_RATE, TOOL_RATE) * y
        if vertical:
            dv[2] += np.clip(TOOL_GAIN * err_z, -TOOL_RATE, TOOL_RATE)
        self.corr = np.clip(self.corr + dv, -TOOL_MAX, TOOL_MAX)
        self.tool_err = np.array([err_r, err_l, err_z])
        self.lat_err = err_l

    def _align_step(self, T_wk, nu, theta):
        """Turn the commanded jaw angle until the MEASURED jaw is square to
        the nearest flat pair of the MEASURED nut."""
        _, _, _, th_meas = self.jaw_frame(T_wk)
        al = self.align_err(th_meas, nu)
        self.align = al
        return float(theta - np.clip(K_ALIGN * al, -ALIGN_RATE, ALIGN_RATE))

    def _ladder(self, t, dt_k, T_wk, T_wt, bq, nu, probes):
        rig, wg = self.rig, self.wg
        fps = rig.fps
        k = self.k
        jawF = probes["jaw"]
        tau_t = self.dirn * probes["jaw_tau"]           # + = in the task's direction
        seated = self.nut_bottom(bq) - BLOCK_TOP <= SEAT_TOL
        lat_ok = True
        if k == "engage":
            hold = dt_k < T_ENG_HOLD
            self._tool_servo(T_wk, self.A, self.theta, self.e, radial=hold,
                             lateral=jawF < 0.15, vertical=True)
            self.theta = self._align_step(T_wk, nu, self.theta)
            if hold:
                if dt_k + 1.0 / fps >= T_ENG_HOLD:
                    self._relatch(T_wk, T_wt)
                self._send_ladder()
                return
            al_deg = np.degrees(self.align)
            lat_ok = abs(self.lat_err) < LAT_TOL or self.e > E_SLOW
            if self.first_contact_align is None and jawF > 0.15:
                self.first_contact_align = al_deg
            # A square, centred jaw slides onto the flats WITHOUT touching the
            # nut (0.32 mm per side).  Any touch before the swing is a corner:
            # back off and re-align -- pushing on spins the nut, and an angle
            # servo chasing a spinning nut is positive feedback (measured: the
            # nut turned 35 deg during one slide-on).
            if jawF > F_TOUCH and self.e > 0.0005:
                self.n_touch = getattr(self, "n_touch", 0) + 1
                self.e = min(CLEAR_E, self.e + V_ENG_SLOW / fps)
                self.hold_adv = 8
            elif getattr(self, "hold_adv", 0) > 0:
                self.hold_adv -= 1
            elif (abs(al_deg) <= 0.5 * ALIGN_TOL and lat_ok) or \
                    dt_k >= T_ENG_HOLD + 3.0 * T_ENG:
                v_e = V_ENG_FAST if self.e > E_SLOW else V_ENG_SLOW
                self.e = max(0.0, self.e - v_e / fps)
            self.ue = 1.0 if self.e <= 0.0 else 0.0
            if self.ue >= 1.0:
                self.stroke = {"n": len(self.strokes), "t0": t, "theta0": self.theta,
                               "touches_in_slide_on": getattr(self, "n_touch", 0),
                               "nu0": nu, "z0": self.nut_bottom(bq),
                               "align_first_contact_deg": self.first_contact_align
                               if self.first_contact_align is not None else al_deg,
                               "swung": 0.0}
                self.go("swing", t)
        elif k == "swing":
            self._tool_servo(T_wk, self.A, self.theta, 0.0, radial=False, lateral=False,
                             vertical=True)
            st = self.stroke
            T = self.torque
            stop = bool(T["stalled"])                   # holding the stall
            T["n_on"] = T.get("n_on", 0) + 1 if tau_t >= TAU_ONSET else 0
            if self.mode.tighten and seated and not T["stalled"] and \
                    (T["mode"] or T["n_on"] >= 2):
                if not T["mode"]:
                    T["mode"] = True
                    print(f"t={t:.2f}s nut SEATED and the clamp is building (bottom "
                          f"{1000*(self.nut_bottom(bq)-BLOCK_TOP):+.3f} mm, jaw torque "
                          f"{1000*tau_t:.1f} mN.m) -> torque-up at {np.degrees(TORQUE_RATE):.0f} deg/s")
                T["peak"] = max(T["peak"], tau_t)
                T["n_hi"] = T["n_hi"] + 1 if tau_t >= TAU_TIGHT else 0
                if T["n_hi"] >= N_STALL and not T["stalled"]:
                    T["stalled"], T["t_stall"] = True, t
                    T["preload"] = float(np.clip(PRELOAD_FRAC * T["peak"], PRELOAD_MIN,
                                                 PRELOAD_MAX))
                    print(f"t={t:.2f}s STALL: jaw torque {1000*tau_t:.1f} mN.m (peak "
                          f"{1000*T['peak']:.1f}) -> seat preload {1000*T['preload']:.1f} mN.m")
                    stop = True
            if not stop and not T["stalled"]:
                u = min(1.0, dt_k / T_SWING)
                target = STROKE * smooth5(u)
                if T["mode"]:
                    target = min(target, st["swung"] + TORQUE_RATE / fps)
                elif self.mode.tighten and seated:
                    target = min(target, st["swung"] + SEATED_RATE / fps)
                st["swung"] = max(st["swung"], target)
                self.theta = st["theta0"] + self.dirn * st["swung"]
                stop = st["swung"] >= STROKE - 1e-6
            if stop:
                if T["stalled"] and T["hold_until"] is None:
                    T["hold_until"] = t + 0.25
                if not T["stalled"] or t >= T["hold_until"]:
                    if T["stalled"] and self.seat.tau_b == 0.0:
                        # the preload takes over where the jaw lets go (anchored
                        # unloaded at the end of the stall hold)
                        self.seat.engage(T["preload"])
                    d_nu = np.degrees(nu - st["nu0"])
                    st.update(t1=t, nut_deg=round(float(d_nu), 2),
                              descent_mm=round(float((st["z0"] - self.nut_bottom(bq)) * MM), 3),
                              wrench_deg=round(float(np.degrees(st["swung"])), 2),
                              follow=round(float(self.dirn * d_nu / max(np.degrees(st["swung"]), 1e-3)), 3))
                    st["align_first_contact_deg"] = round(float(st["align_first_contact_deg"]), 3)
                    self.strokes.append(st)
                    print(f"t={t:.2f}s stroke {st['n']}: wrench {st['wrench_deg']:.1f} deg -> nut "
                          f"{st['nut_deg']:+.1f} deg, down {st['descent_mm']:+.3f} mm, square at "
                          f"contact {st['align_first_contact_deg']:+.2f} deg")
                    self.go("off", t)
        elif k == "off":
            self.e = CLEAR_E * smooth5(min(1.0, dt_k / T_OFF))
            if dt_k >= T_OFF:
                done = self._task_done(bq, nu)
                if done or len(self.strokes) >= N_MAX_STROKES:
                    self.done_reason = done or "stroke budget"
                    self.t_jaw_off = t
                    _, q_now = self.tcp_of(wg.flat_T(self.A, self.theta, CLEAR_E))
                    self.lift_from = (self.p_out.copy(), q_now)
                    print(f"t={t:.2f}s ladder done ({self.done_reason}) after "
                          f"{len(self.strokes)} strokes")
                    self.go("lift", t)
                    self._send_ladder()
                    return
                self.idx_from = self.theta
                self.go("index", t)
        elif k == "index":
            self.n_touch = 0
            to = self.engage_angle(nu, self.theta_pref)
            self.theta = self.idx_from + (to - self.idx_from) * smooth5(min(1.0, dt_k / T_IDX))
            if dt_k >= T_IDX:
                self.theta = to
                self.ue = 0.0
                self.first_contact_align = None
                self.go("engage", t)
        self._send_ladder()

    def _send_ladder(self):
        p, q = self.tcp_of(self.wg.flat_T(self.A, self.theta, self.e))
        self._out(p, q, p + self.corr, self.f_cmd)

    def _task_done(self, bq, nu):
        if self.mode.tighten:
            return "tightened (stall)" if self.torque["stalled"] else None
        if self.loosen["start_yaw"] is None:
            return None
        backed = np.degrees(nu - self.loosen["start_yaw"])
        return f"backed off {backed:.0f} deg" if backed >= LOOSEN_DONE_DEG else None


# ═════════════════════════════════════════════════════════════════════════
def run(mode: Mode):
    stop_phase = os.environ.get("WH_STOP_PHASE", "")
    stop_t = _env("WH_STOP_T", 0.0)
    rig = appendix_rig(mode.name)
    rig.collide_every_substep = True
    rig.solver_options["impratio"] = IMPRATIO
    wg = WrenchGeom()
    h = build(rig, mode, wg)
    pl = wg.plate
    print(f"wrench {pl.mass*1000:.1f} g; jaw {wg.jaw['af']*1000:.2f} mm, tilt "
          f"{np.degrees(wg.jaw['tilt']):.2f} deg; handle at the balance point "
          f"{2*wg.half_w*MM:.2f} mm wide, grip radius {wg.grip_r*MM:.1f} mm, handle "
          f"{np.degrees(wg.handle_off):.1f} deg off the jaw axis")
    print(f"nut starts {mode.z_above_seat*MM:.2f} mm above the seat, yaw "
          f"{np.degrees(h['yaw0']) % 360:.1f} deg; flat phase {np.degrees(h['flat_phase']):.2f}")

    # robot start: parked 60 mm in front of the hung wrench, fingers open
    H0 = h["H0"]
    n0, d0 = hang_frame(np.pi / 2)
    tcp_g, q_g = wg.nominal_tcp(H0, n0, d0)
    rig.solve_arm_start(tcp_g + 0.060 * n0, q_g)
    rig.builder.joint_q[7:9] = [0.04, 0.04]
    rig.builder.joint_target_q[7:9] = [0.04, 0.04]

    cam_t = np.array([_env("WH_CAM_TX", -0.035), _env("WH_CAM_TY", -0.005),
                      Z0 + _env("WH_CAM_TZ", 0.085)])
    cam_p = cam_t + np.array([_env("WH_CAM_OX", 0.40), _env("WH_CAM_OY", -0.50),
                              _env("WH_CAM_OZ", 0.28)])
    cam2_t = np.array([NUT_AXIS[0] - 0.012, NUT_AXIS[1], BLOCK_TOP + 0.006])
    cam2_p = cam2_t + np.array([_env("WH_CAM2_OX", 0.16), _env("WH_CAM2_OY", -0.20),
                                _env("WH_CAM2_OZ", 0.13)])
    rig.finalize(probes=["wrench"], cam_pos=cam_p, cam_target=cam_t)
    fov = _env("WH_CAM_FOV", 40.0)
    if rig.rec is not None:
        rig.rec = RolloutRecorder(rig.model, rig.out_dir, cam_pos=cam_p, cam_target=cam_t,
                                  fps=rig.fps, fov_deg=fov, cam2_pos=cam2_p,
                                  cam2_target=cam2_t)
    rig.solver = make_solver(rig.model, iterations=20)
    m = rig.model
    rig.probes["padL"] = PairProbe(m, "fr3/fr3_leftfinger", ["wrench_shape"])
    rig.probes["padR"] = PairProbe(m, "fr3/fr3_rightfinger", ["wrench_shape"])
    rig.probes["board"] = PairProbe(m, "wrench", ["board"])
    rig.probes["bench"] = PairProbe(m, "wrench", ["table", "block_top"])
    rig.probes["jaw"] = PairProbe(m, "nut", ["wrench_shape"])
    rig.probes["thread"] = PairProbe(m, "nut", ["bolt"])
    rig.probes["seat"] = PairProbe(m, "nut", ["block_top"])
    rig.probes["robot_object"] = RobotContactProbe(
        m, ["wrench", "nut"], static_shapes=["bolt", "block_top", "board"])
    rig.probes["robot_nut"] = RobotContactProbe(m, ["nut"], static_shapes=["bolt"])
    rig.probes["robot_board"] = RobotContactProbe(m, [], static_shapes=["board", "block_top"])
    jaw_tq = DriveContacts(m, rig.shapes["wrench"], rig.shapes["nut"])
    # the nut-on-thread and wrench-on-hook mesh pairs overflow the default
    # 1M triangle-pair buffer (measured 1.4M); rebuilt AFTER the sensors so
    # the contacts carry the force attribute they request
    rig.pipeline = newton.CollisionPipeline(
        m, rigid_contact_max=20000, max_triangle_pairs=4_000_000,
        reduce_contacts=True, broad_phase="sap")
    rig.contacts = rig.pipeline.contacts()

    ni = rig.body_index("nut")
    nj = int(np.flatnonzero(m.joint_child.numpy() == ni)[0])
    seat = NutSeat(rig.solver, ni, int(m.joint_qd_start.numpy()[nj]) + 5,
                   1.0 / rig.fps / rig.substeps)
    if not mode.tighten:
        seat.engage(LOOSE_PRELOAD)
        seat.release_deg = LOOSE_FREE_DEG
    rig.solver = seat

    ex = Expert(rig, mode, wg, h, seat)
    if not mode.tighten:
        ex.loosen["start_yaw"] = 0.0
    log = []
    wi = ex.wi
    v_low = pl.local_mesh().vertices[::7]

    def last(name, key="force"):
        hh = rig.probes[name].history
        return hh[-1][key] if hh else 0.0

    lost = {"n": 0}

    def control(t):
        if stop_phase and ex.k == stop_phase:
            raise StopRun
        if ex.finished:
            raise StopRun
        # the tool has left the pinch: nothing left to measure, stop the run
        held_phase = ex.k not in ("settle", "approach", "pre", "close", "open", "retreat",
                                  "still", "probe")
        lost["n"] = lost["n"] + 1 if (held_phase and last("padL") < 0.3
                                      and last("padR") < 0.3) else 0
        if lost["n"] > 30:
            print(f"t={t:.2f}s TOOL LOST in phase {ex.k}: both pads unloaded for 0.5 s")
            ex.done_reason = f"tool lost in {ex.k}"
            raise StopRun
        pr = {"padL": last("padL"), "padR": last("padR"), "board": last("board"),
              "board_fz": rig.probes["board"].history[-1]["vec"][2]
              if rig.probes["board"].history else 0.0,
              "jaw": last("jaw"), "jaw_tau": ex.jaw_hist["tau"]}
        ex.control(t, pr)

    def extra(state, t):
        c = jaw_tq.sample(rig.contacts, state, NUT_AXIS, t)
        ex.jaw_hist = c
        bq = state.body_q.numpy()
        H, n, _ = pl.frame_of(bq[wi])
        tilt = float(np.degrees(np.arctan2(n[0], n[2])))
        off, px = pierce(pl, bq[wi], ex.poly)
        tcp = HP.actual_tcp(bq, ex.i7, ex.T_off)
        T = world_T(bq[wi])
        low_z = float(((T[:3, :3] @ v_low.T).T + T[:3, 3])[:, 2].min())
        nb = ex.nut_bottom(bq)
        # diagnostics: in-hand rotation vs the latched grasp, TCP orientation error
        T_wk = kit_T(bq[wi], pl.center)
        T_wt = world_T(bq[ex.i7]) @ ex.T_off
        inhand = tcp_err = 0.0
        if ex.X is not None:
            Xn = np.linalg.inv(T_wk) @ T_wt
            dR = ex.X[:3, :3].T @ Xn[:3, :3]
            inhand = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
        if ex.q_cmd is not None:
            dR = R_of_q(ex.q_cmd).T @ T_wt[:3, :3]
            tcp_err = float(np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))))
        row = dict(t=t, k=ex.k, inhand_deg=inhand, tcp_rot_err_deg=tcp_err, Hx=H[0], Hy=H[1], Hz=H[2], tilt=tilt, pierce_off=off,
                   pierce_x=px, padL=last("padL"), padR=last("padR"), board=last("board"),
                   board_fz=rig.probes["board"].history[-1]["vec"][2],
                   bench=last("bench"), jaw=last("jaw"), jaw_tau=c["tau"],
                   thread=last("thread"), seat=last("seat"), robot_obj=last("robot_object"),
                   robot_nut=last("robot_nut"), robot_board=last("robot_board"), tcp_x=tcp[0], tcp_y=tcp[1], tcp_z=tcp[2],
                   low_z=low_z, nut_bottom=nb, nut_yaw=seat.yaw,
                   stiction=seat.tau, probe=seat.tau_probe, tau_b=seat.tau_b,
                   fing=ex.fing_out if hasattr(ex, "fing_out") else 0.0)
        log.append(row)
        if int(round(t * rig.fps)) % 15 == 0:
            print(f"  [{t:6.2f} {ex.k:10s}] pads {row['padL']:5.2f}/{row['padR']:5.2f} board "
                  f"{row['board']:6.3f} bench {row['bench']:5.3f} jaw {row['jaw']:6.3f} N "
                  f"tau {1000*c['tau']:+6.2f} mNm | nut z {1000*(nb-BLOCK_TOP):+.3f} mm yaw "
                  f"{np.degrees(seat.yaw):+8.2f} stic {1000*seat.tau:+5.1f} | tilt {tilt:5.1f} "
                  f"pierce {off*MM:4.1f} | th {np.degrees(ex.theta):7.2f} e {ex.e*MM:5.1f} "
                  f"al {np.degrees(getattr(ex, 'align', 0.0)):+5.2f} lat "
                  f"{getattr(ex, 'lat_err', 0.0)*MM:+5.2f} corr {np.round(ex.corr*MM, 2)} "
                  f"f {ex.f_cmd*MM:.2f} | inhand {inhand:5.2f} tcp_rot_err {tcp_err:5.2f} deg "
                  f"robot-board {row['robot_board']:.2f} N")
        return {"padL_N": round(row["padL"], 3), "padR_N": round(row["padR"], 3),
                "board_N": round(row["board"], 4), "jaw_N": round(row["jaw"], 4),
                "jaw_torque_mNm": round(1000 * c["tau"], 3),
                "thread_N": round(row["thread"], 4), "seat_N": round(row["seat"], 4),
                "robot_object_contact_N": round(row["robot_obj"], 4),
                "nut_dz_mm": round(1000 * (nb - BLOCK_TOP), 4),
                "nut_yaw_deg": round(float(np.degrees(seat.yaw)), 3),
                "seat_preload_mNm": round(1000 * seat.tau, 3),
                "probe_torque_mNm": round(1000 * seat.tau_probe, 3),
                "phase": ex.k}

    dur = stop_t if stop_t > 0 else T_BUDGET
    try:
        rec = rig.run(dur, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{mode.name}] stopped at t={len(rec.times) / rig.fps:.2f}s (phase {ex.k})")
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "board_N", "jaw_N",
                                            "jaw_torque_mNm", "seat_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    import json
    (rig.out_dir / "expert_log.json").write_text(json.dumps(
        {"t_start": {k: round(v, 3) for k, v in ex.t_start.items()},
         "strokes": ex.strokes, "torque": ex.torque, "done_reason": ex.done_reason},
        indent=1, default=float))
    import pickle
    with open(rig.out_dir / "expert_trace.pkl", "wb") as fh:
        pickle.dump({"log": log, "t_start": ex.t_start, "strokes": ex.strokes,
                     "torque": ex.torque, "done_reason": ex.done_reason,
                     "t_jaw_off": ex.t_jaw_off, "dq_peak": ex.trk.dq_peak,
                     "seat": {"peak": seat.peak, "released_at": seat.released_at,
                              "tau_b": seat.tau_b}, "finished": ex.finished}, fh)
    if not ex.finished:
        print("stopped early: QA skipped")
        return None
    return verdict(rig, mode, wg, h, ex, seat, log, paths)


# ═════════════════════════════════════════════════════════════════════════
def pen_wrench_board(qa, wg, board_tm, board_c, hook, windows, pair_name, tol=0.0010,
                     stride=4):
    """Wrench <-> board interpenetration, both ways, where the wrench meets
    the hook: ``_hang_plate.pen_plate_board``'s test (plate vertices near the
    HOLE inside the board, the board's HOOK vertices inside the plate), but
    on the frames of the hook-interaction windows only (every `stride`-th),
    against the RAW kit mesh.  pen_plate_board samples every 3rd frame within
    130 mm of the hook -- most of this 50 s rollout -- against the subdivided
    sim mesh and took > 30 min."""
    pl = wg.plate
    raw = wg.raw.copy()
    raw.apply_translation(-pl.center)              # body-local, raw tessellation
    v = raw.vertices
    hole_b = pl.hole - pl.center
    va = v[np.linalg.norm(v[:, :2] - hole_b[:2], axis=1) < 0.012]
    if len(va) > 1000:
        va = va[np.random.RandomState(0).choice(len(va), 1000, replace=False)]
    bl = board_tm.copy()
    bl.apply_translation(-np.asarray(board_c))
    lo, hi = bl.bounds[0] - 1e-6, bl.bounds[1] + 1e-6
    pq_b = trimesh.proximity.ProximityQuery(bl)
    pq_p = trimesh.proximity.ProximityQuery(raw)
    bv = board_tm.vertices
    hk = (np.abs(bv[:, 1] - hook.y) < 0.006) & (bv[:, 0] > HP.BAR_X_WALL - 0.002) & \
         (bv[:, 2] > Z0 + 0.145)
    hook_w = bv[hk]
    bi = qa._idx("wrench")
    frames = set()
    for a, b in windows:
        frames |= set(np.flatnonzero((qa.t >= a) & (qa.t <= b))[::stride].tolist())
    frames = sorted(frames)
    worst, worst_t, worst_dir = 0.0, None, None
    for f in frames:
        T = world_T(qa.body_q[f, bi])
        pts = (T[:3, :3] @ va.T).T + T[:3, 3] - np.asarray(board_c)
        pts = pts[((pts > lo) & (pts < hi)).all(axis=1)]
        if len(pts):
            dmax = float(pq_b.signed_distance(pts).max())
            if dmax > worst:
                worst, worst_t, worst_dir = dmax, float(qa.t[f]), "wrench in board"
        hl = (T[:3, :3].T @ (hook_w - T[:3, 3]).T).T
        m = ((hl > raw.bounds[0]) & (hl < raw.bounds[1])).all(axis=1)
        if m.any():
            dmax = float(pq_p.signed_distance(hl[m]).max())
            if dmax > worst:
                worst, worst_t, worst_dir = dmax, float(qa.t[f]), "hook in wrench"
    entry = {"max_depth_mm": round(worst * 1000, 3), "at_t": worst_t, "where": worst_dir,
             "frames_checked": len(frames), "pass": bool(worst < tol)}
    pen = qa.report["checks"].setdefault(
        "penetration", {"pass": True, "tolerance_mm": tol * 1000, "pairs": {}})
    pen["pairs"][pair_name] = entry
    pen["pass"] = bool(pen["pass"] and entry["pass"])
    qa.report["pass"] &= entry["pass"]


def pen_jaw_nut(qa, wg, nut_local, windows, tol=0.0015, stride=6):
    """Wrench jaw <-> nut interpenetration, both ways, on the stroke-ladder
    frames only (every `stride`-th), with an exact bounding-box prefilter
    (points outside the other mesh's box cannot be inside it) and the RAW
    wrench mesh.  RolloutQA.check_penetration samples the whole rollout,
    where the wrench hangs 200 mm from the nut, and trimesh's closest-point
    search on far points is what made it take tens of minutes."""
    pl = wg.plate
    raw = wg.raw.copy()
    raw.apply_translation(-pl.center)
    seat_b = wg.seat_kit - pl.center
    rs = np.random.RandomState(0)
    jaw_v = raw.vertices[np.linalg.norm(raw.vertices[:, :2] - seat_b[:2], axis=1) < 0.018]
    if len(jaw_v) > 1500:
        jaw_v = jaw_v[rs.choice(len(jaw_v), 1500, replace=False)]
    nut_v = nut_local.vertices
    if len(nut_v) > 1500:
        nut_v = nut_v[rs.choice(len(nut_v), 1500, replace=False)]
    pq_n = trimesh.proximity.ProximityQuery(nut_local)
    pq_w = trimesh.proximity.ProximityQuery(raw)
    n_lo, n_hi = nut_local.bounds[0] - 1e-4, nut_local.bounds[1] + 1e-4
    j_lo = jaw_v.min(axis=0) - 1e-4
    j_hi = jaw_v.max(axis=0) + 1e-4
    wi, ni = qa._idx("wrench"), qa._idx("nut")
    frames = set()
    for a, b in windows:
        frames |= set(np.flatnonzero((qa.t >= a) & (qa.t <= b))[::stride].tolist())
    worst, worst_t, worst_dir = 0.0, None, None
    for f in sorted(frames):
        Tw, Tn = world_T(qa.body_q[f, wi]), world_T(qa.body_q[f, ni])
        Twn = np.linalg.inv(Tn) @ Tw                    # wrench -> nut frame
        p = (Twn[:3, :3] @ jaw_v.T).T + Twn[:3, 3]
        p = p[((p > n_lo) & (p < n_hi)).all(axis=1)]
        if len(p):
            dmax = float(pq_n.signed_distance(p).max())
            if dmax > worst:
                worst, worst_t, worst_dir = dmax, float(qa.t[f]), "jaw in nut"
        Tnw = np.linalg.inv(Twn)
        p = (Tnw[:3, :3] @ nut_v.T).T + Tnw[:3, 3]
        p = p[((p > j_lo) & (p < j_hi)).all(axis=1)]
        if len(p):
            dmax = float(pq_w.signed_distance(p).max())
            if dmax > worst:
                worst, worst_t, worst_dir = dmax, float(qa.t[f]), "nut in jaw"
    entry = {"max_depth_mm": round(worst * 1000, 3), "at_t": worst_t, "where": worst_dir,
             "frames_checked": len(frames), "pass": bool(worst < tol)}
    pen = qa.report["checks"].setdefault(
        "penetration", {"pass": True, "tolerance_mm": tol * 1000, "pairs": {}})
    pen["pairs"]["wrench_jaw<->nut"] = entry
    pen["pass"] = bool(pen["pass"] and entry["pass"])
    qa.report["pass"] &= entry["pass"]


def verdict(rig, mode, wg, h, ex, seat, log, paths):
    pl = wg.plate
    qa = rig.qa(paths)
    keys = [k for k in log[0] if k != "k"]
    L = {k: np.array([r[k] for r in log], float) for k in keys}
    phase = np.array([r["k"] for r in log])
    t = L["t"]
    ts = ex.t_start
    weight = 9.81 * pl.mass

    def win(a, b):
        return (t >= a) & (t <= b)

    # ── penetration (solver-independent, trimesh signed distance) ───────
    nut_tm, nut_c = rig.meshes["nut"]
    nut_local = nut_tm.copy()
    nut_local.apply_translation(-np.asarray(nut_c))
    ts_ = ex.t_start
    t_ladder_end = ex.t_jaw_off if ex.t_jaw_off is not None else float(t[-1])
    pen_jaw_nut(qa, wg, nut_local, [(ts_["lower"], t_ladder_end + 0.3)], tol=0.0015)
    qa.check_penetration_static("nut", nut_local, rig.meshes["bolt"][0],
                                rig.meshes["bolt"][1] * 0.0, "nut->bolt", tol=0.0015)
    qa.check_penetration_static("nut", nut_local, rig.meshes["block_top"][0],
                                np.zeros(3), "nut->block_top", tol=0.0015)
    hook_windows = [(ts_["approach"], ts_["away"]), (ts_["toward"], ts_["still"] + 0.5)]
    pen_wrench_board(qa, wg, *rig.meshes["board"], ex.hook, hook_windows, "wrench<->board",
                     tol=0.0010, stride=10)
    zmin = float(L["low_z"].min() - Z0)
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["wrench->bench_top"] = {"min_vertex_z_above_top_mm": round(zmin * MM, 3),
                                        "pass": bool(zmin > -0.0010)}
    pen["pass"] = bool(pen["pass"] and zmin > -0.0010)
    qa.check_envelope(["wrench", "nut"], [-0.20, -0.25, Z0 - 0.01], [0.20, 0.25, Z0 + 0.40])
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())

    crit = {}
    # ── 1. taken from the hook ───────────────────────────────────────────
    hung0 = win(0.3, ts["approach"])
    held = win(ts["hold"], ts["open"])
    after_unhook = win(ts["carry"], ts["thread"])
    both = float(((L["padL"][held] > 0.5) & (L["padR"][held] > 0.5)).mean())
    crit["wrench_taken_from_hook"] = {
        "pass": bool(abs(L["board_fz"][hung0].mean() - weight) < 0.3 * weight
                     and (L["pierce_off"][hung0] < pl.r_hole).all()
                     and (L["pierce_off"][after_unhook] > pl.r_hole + 0.001).all()
                     and L["board"][after_unhook].max() < 0.01
                     and both > 0.98),
        "board_support_while_hanging_N": round(float(L["board_fz"][hung0].mean()), 4),
        "weight_N": round(weight, 4),
        "hook_through_hole_at_start": bool((L["pierce_off"][hung0] < pl.r_hole).all()),
        "hook_clear_of_hole_from_carry_to_rehang": bool(
            (L["pierce_off"][after_unhook] > pl.r_hole + 0.001).all()),
        "board_contact_max_after_unhook_N": round(float(L["board"][after_unhook].max()), 4),
        "both_pads_loaded_fraction_hold_to_release": round(both, 4),
        "pad_force_mean_N": [round(float(L["padL"][held].mean()), 2),
                             round(float(L["padR"][held].mean()), 2)],
        "note": "hanging on the middle hook at t=0 (the board carries its weight, the hook "
                "passes through the hole); pinched at the balance point and carried back out "
                "along the hook centre-line (bar -> bend -> stub -> past the tip); a "
                "fingertip pinch only"}
    tool_held = win(ts["hold"] + 0.2, ts["open"])
    bench_max = float(L["bench"][win(ts["unseat"], ts["open"])].max())
    crit["wrench_held_and_clear_of_the_block"] = {
        "pass": bool(both > 0.98 and bench_max < 0.05),
        "wrench_vs_bench_or_block_contact_max_N": round(bench_max, 4),
        "note": "the wrench stays in the pinch from the hook to the re-hang and never "
                "rests on the bench or the block (the jaw rides 1 mm above the block top)"}

    # ── 2. the nut ───────────────────────────────────────────────────────
    t_off = ex.t_jaw_off if ex.t_jaw_off is not None else t[-1]
    t_probe = ts.get("probe", t[-1])
    rest = win(t_off + T_OFF + 0.5, t_probe - 0.05)
    yaw_deg = np.degrees(L["nut_yaw"])
    dz_mm = (L["nut_bottom"] - BLOCK_TOP) * MM
    jaw_after = L["jaw"][t >= t_off + 0.3]
    crit["wrench_off_the_nut"] = {
        "pass": bool(jaw_after.max() < CONTACT_FREE_N
                     and L["robot_nut"][t >= t_off].max() < CONTACT_FREE_N),
        "jaw_contact_peak_after_off_N": round(float(jaw_after.max()), 4),
        "robot_nut_contact_peak_after_off_N": round(float(L["robot_nut"][t >= t_off].max()), 4)}
    pm = {}
    for kk, nm in ((0, "loosen_way"), (1, "tighten_way")):
        w = win(t_probe + kk * T_PROBE + 0.02, t_probe + (kk + 1) * T_PROBE)
        if w.any():
            pm[nm] = {"rotation_deg": round(float(yaw_deg[w][-1] - yaw_deg[w][0]), 3),
                      "excursion_deg": round(float(np.ptp(yaw_deg[w])), 3),
                      "probe_torque_median_mNm": round(float(np.median(np.abs(L["probe"][w]))) * 1000, 3),
                      "probe_torque_max_mNm": round(float(np.abs(L["probe"][w]).max()) * 1000, 3)}
    stroke_turn = sum(s["nut_deg"] for s in ex.strokes)
    if mode.tighten:
        need = mode.z_above_seat * MM
        crit["nut_tightened"] = {
            "pass": bool(ex.torque["stalled"] and ex.torque["peak"] >= TAU_TIGHT
                         and abs(float(np.median(dz_mm[rest]))) <= SEAT_TOL * MM
                         and float(np.median(L["seat"][rest])) > 0.5 * NUT_MASS * 9.81),
            "start_above_seat_mm": round(need, 3),
            "final_gap_to_seat_mm": round(float(np.median(dz_mm[rest])), 4),
            "seat_contact_after_wrench_left_N": round(float(np.median(L["seat"][rest])), 4),
            "strokes": len(ex.strokes), "nut_turned_deg_by_strokes": round(stroke_turn, 1),
            "torque_up_peak_jaw_torque_mNm": round(ex.torque["peak"] * 1000, 2),
            "stall_threshold_mNm": TAU_TIGHT * 1000,
            "seat_preload_breakaway_mNm": round(ex.torque["preload"] * 1000, 2),
            "per_stroke": ex.strokes,
            "note": "run down through the jaw only (60 deg strokes re-seated on the measured "
                    "flats), torqued against the block-top seat until the measured jaw torque "
                    "stalled at the threshold; the preload model then holds "
                    f"{PRELOAD_FRAC:.1f} x the peak"}
        rest_mv = float(np.ptp(yaw_deg[rest])) if rest.any() else 0.0
        crit["nut_unmoved_after_wrench_left"] = {
            "pass": bool(rest_mv < 1.0 and float(np.ptp(dz_mm[rest])) < 0.05),
            "yaw_excursion_deg": round(rest_mv, 3),
            "height_excursion_mm": round(float(np.ptp(dz_mm[rest])), 4),
            "window_s": [round(float(t_off + T_OFF + 0.5), 2), round(float(t_probe), 2)]}
        crit["stationary_under_small_torque"] = {
            "pass": bool(pm and all(v["excursion_deg"] < STILL_MAX_DEG for v in pm.values())
                         and all(v["probe_torque_max_mNm"] > 0.9 * PROBE_TAU * 1000
                                 for v in pm.values())),
            "probe": pm, "probe_cap_mNm": PROBE_TAU * 1000,
            "probe_drag_deg_s": float(np.degrees(PROBE_RATE)), "threshold_deg": STILL_MAX_DEG,
            "note": "after the robot hung the wrench back and stopped: a torque-limited drag "
                    "(reference advancing at 60 deg/s, |torque| <= 5 mN.m) on the nut, "
                    "1 s each way; it must saturate and the nut must not move"}
    else:
        st0 = win(0.0, 0.3)
        crit["started_seated_and_tight"] = {
            "pass": bool(abs(float(np.median(dz_mm[st0]))) <= SEAT_TOL * MM
                         and seat.peak >= 0.9 * LOOSE_PRELOAD),
            "start_gap_to_seat_mm": round(float(np.median(dz_mm[st0])), 4),
            "breakaway_preload_mNm": LOOSE_PRELOAD * 1000,
            "peak_preload_reaction_mNm": round(seat.peak * 1000, 2),
            "preload_released_after_deg": seat.released_at,
            "peak_jaw_torque_mNm": round(float(np.abs(L["jaw_tau"]).max()) * 1000, 2),
            "note": "seated at t=0 with the stiction preload (module doc); the jaw had to "
                    "break it"}
        backed = float(np.median(yaw_deg[rest])) if rest.any() else float(yaw_deg[-1])
        crit["nut_loosened"] = {
            "pass": bool(backed >= 0.9 * LOOSEN_DONE_DEG and seat.tau_b == 0.0
                         and float(np.median(dz_mm[rest])) > 0.3),
            "backed_off_deg": round(backed, 1),
            "rise_off_seat_mm": round(float(np.median(dz_mm[rest])), 3),
            "strokes": len(ex.strokes), "per_stroke": ex.strokes}
        lw = pm.get("loosen_way", {})
        crit["rotates_freely_under_small_torque"] = {
            "pass": bool(lw and lw["rotation_deg"] >= FREE_MIN_DEG
                         and lw["probe_torque_median_mNm"] <= 0.5 * PROBE_TAU * 1000),
            "probe": pm, "probe_cap_mNm": PROBE_TAU * 1000,
            "probe_drag_deg_s": float(np.degrees(PROBE_RATE)), "min_rotation_deg": FREE_MIN_DEG,
            "note": "same torque-limited drag as the tighten task; a loose nut follows the "
                    "60 deg/s reference at a fraction of the 5 mN.m cap"}

    # ── 3. hung back on the hook, robot clear and stopped ───────────────
    t_end = float(t[-1])
    fin = win(ts["still"], t_end)
    after = t >= ts["retreat"]
    hz = L["Hz"][fin]
    swing = float(max(np.ptp(L["Hx"][fin]), np.ptp(L["Hy"][fin]), np.ptp(hz)))
    on_hook = bool((L["pierce_off"][after] < pl.r_hole - 0.001).all()
                   and (L["pierce_x"][after] >= HP.BAR_X_WALL - 0.001).all()
                   and (L["pierce_x"][after] <= HP.CAP_C[0] + HP.CAP_R).all())
    carried = bool(abs(L["board_fz"][fin].mean() - weight) < 0.15 * weight)
    off_bench = bool(L["low_z"][fin].min() > Z0 + 0.002)
    settled = bool(swing < 0.004 and np.abs(L["tilt"][fin] - 90.0).max() < 20.0)
    crit["wrench_returned_to_hook"] = {
        "pass": bool(on_hook and carried and off_bench and settled),
        "hook_through_hole_every_frame_after_release": on_hook,
        "hook_pierce_offset_mm_max": round(float(L["pierce_off"][after].max()) * MM, 2),
        "board_vertical_support_N": round(float(L["board_fz"][fin].mean()), 4),
        "weight_N": round(weight, 4),
        "lowest_point_above_bench_mm": round(float(L["low_z"][fin].min() - Z0) * MM, 1),
        "tilt_deg_final": round(float(L["tilt"][fin][-1]), 2),
        "residual_swing_mm_ptp": round(swing * MM, 3),
        "note": "_hang_plate's hung test: the hook passes through the hole from the "
                "retreat on, the board carries the weight, the wrench hangs clear of the "
                "bench and has settled"}
    i_rel = int(np.searchsorted(t, ts["open"]))
    tcp = np.column_stack([L["tcp_x"], L["tcp_y"], L["tcp_z"]])
    moved = float(np.linalg.norm(tcp[-1] - tcp[i_rel]))
    check_released_and_clear(qa, t, L["robot_obj"], ts["still"], extra={
        "tcp_moved_from_release_mm": round(moved * MM, 1), "min_move_mm": 50.0,
        "objects": "wrench, nut, bolt, block, board"})
    rel = qa.report["checks"]["released_and_clear"]
    rel["pass"] = bool(rel["pass"] and moved >= 0.050)
    crit["released_and_clear"] = rel
    d = np.load(paths["states"], allow_pickle=True)
    labels = list(d["body_labels"])
    arm_ix = [i for i, l in enumerate(labels) if str(l).startswith("fr3/")]
    msk = d["t"] >= ts["still"]
    v_arm = float(np.linalg.norm(np.diff(d["body_q"][msk][:, arm_ix, :3], axis=0),
                                 axis=-1).max()) * rig.fps
    crit["robot_stopped"] = {"pass": bool(v_arm < 0.005),
                             "max_link_speed_final_window_mm_s": round(v_arm * MM, 3),
                             "threshold_mm_s": 5.0}
    aligns = [abs(s["align_first_contact_deg"]) for s in ex.strokes]
    qa.report["checks"]["expert_tracking"] = {
        "pass": True, "info_only": True,
        "jaw_square_at_first_contact_deg": [s["align_first_contact_deg"] for s in ex.strokes],
        "worst_align_deg": round(max(aligns), 3) if aligns else None,
        "align_tolerance_deg": ALIGN_TOL,
        "peak_joint_cmd_step_rad": round(ex.trk.dq_peak, 4),
        "phase_start_s": {k: round(v, 2) for k, v in ts.items()},
        "done_reason": ex.done_reason}
    for k_, c_ in crit.items():
        qa.report["checks"].setdefault(k_, c_)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    return save_appendix_verdict(
        rig, task=mode.title, source="new task family (wrench from the hook shelf); scene "
        "from _hang_plate + the finalized-kit bolt block; jaw analysis from "
        "force_bench.tasks.wrench_nut_arm", spec=mode.spec, criteria=crit,
        physics_qa=report, changes=mode.changes)
