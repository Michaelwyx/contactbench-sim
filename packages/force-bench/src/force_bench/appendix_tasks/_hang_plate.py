"""Shared pieces of the "Hang object" appendix tasks whose object is a PLATE
with a round hole: ``hang_wrench`` and ``hang_ruler``.

Appendix Table I, task 9 "Hang object" (variants wrench / ruler / keychain).
Success: "The robot movement stops after placing the target object onto a
hanging hook. The robot should not remain in contact with the task objects."
The appendix starts with the object in hand; these variants start with it
lying on the bench and pick it up first (user request, 2026-10-15).

GEOMETRY (measured off the meshes, metres; the board's CAD frame, which the
scene lifts by Z0 onto a bench top -- everything world-side adds Z0)
---------------------------------------------------------------------------
hanging_board  ``assets/bodies/hanging_board/part1_1_body1`` -- the SAME solid
    as the finalized kit's ``hanging_board.step`` (kit tessellation shifted
    by (-125, -25, 0) mm lies on it within 0.13 mm max / 0.001 mm mean), so
    the sim STL is used.  Wall x -140..-130 mm on a 60 x 170 x 10 mm foot
    (x -155..-95); three identical hooks at y = -81 / -25 / +31 mm.  Each
    hook, in its own x-z section:
      * a 6 x 6 mm BAR (z 152..158, 2 mm corner fillets) whose flat top runs
        from the wall gusset (x -120.6) to the bend (x -99.04);
      * a 60 deg BEND: concentric arcs about (-99.04, 162.0) mm, radius 4
        (top) and 10 (underside), i.e. a 7 mm centre-line radius;
      * a straight STUB rising at 60 deg for 7.96 mm, ending in a radius-3
        CAP centred at (-89.0, 165.39) (crest z 168.39, outermost x -86.0).
wrench  ``assets/bodies_kit/wrench/wrench.stl`` (finalized kit STEP; the same
    solid as ``assets/bodies/unidentified/part1_1_body1``: ICP-aligned within
    0.19 mm, volume within 0.006 %).  110.2 x 38.4 x 7.5 mm, 10.0 mm hole.
ruler  ``assets/bodies_kit/ruler/ruler.stl`` (finalized kit STEP; no sim
    asset existed).  137.0 x 25.0 x 2.0 mm, 10.0 mm hole 9.5 mm from one end.

WHY THE HOLE IS GUIDED ONTO THE BAR, NOT DROPPED ON THE TIP
----------------------------------------------------------
A 10 mm hole on this 6 mm hook has 1.6 mm (corner fillets) to 2.0 mm (flats)
of radial play.  The stub stands at 60 deg, so a FLAT plate can only be
lowered onto it over the 3 mm cap; the stub's horizontal section is 6.9 mm
long and slides 0.58 mm sideways per mm of depth.  Tilting the plate 30 deg
(its hole axis = the stub axis) lets the stub pass straight through the
hole.  Releasing it there is not a hang for a THICK plate: a 7.5 mm sleeve on
a 60 deg rod, loaded 53 mm off-axis by its own centre of mass, wedges (the
classic drawer jam: edge reactions ~ W * 53 / 7.5 = 7 W, Coulomb capacity
2 * 0.5 * 7 W >> the 0.87 W pulling it down the stub).  So the hand carries
the hole down the stub, round the bend and onto the flat bar, turning the
plate with the hook's own tangent, and lets go only there.  The bar then
carries it; nothing is welded and the release is a plain pad opening.

Everything task-specific (the pick, the timings, the verdict) lives in the
task modules; this module holds the scene, the hook path, the grasp
geometry, the joint tracker and the "is it hung" test.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import load_tm, make_newton_mesh, set_body_mass
from force_bench.paths import ASSETS_ROOT
from force_bench.rig import set_finger_effort_limit
from force_bench.geometry import world_T

MM = 1000.0
KIT_DIR = ASSETS_ROOT / "bodies_kit"

# ── the cell: a bench block whose top is Z0 above the floor ────────────
# The suite's tabletop scenes put the work surface AT the ground plane, so
# the bench top z-fights the floor and its edge is invisible; the ruler
# variant's slide-to-the-edge needs an edge you can see (and air under it),
# so the whole cell stands on a bench block and the robot on a pedestal of
# the same height beside it.
Z0 = 0.120

# ── the board and its hooks (board CAD frame; see the header) ──────────
BOARD_ASSET = ("hanging_board", "part1_1_body1")
PEG_YS = (-0.081, -0.025, 0.031)
BEND_C = np.array([-0.09904, 0.16200])     # (x, z) centre of the bend arcs
R_BEND_IN, R_BEND_OUT = 0.004, 0.010
R_CL = 0.5 * (R_BEND_IN + R_BEND_OUT)      # 7 mm centre-line radius
BAR_Z = float(BEND_C[1] - R_CL)            # 0.155 (bar z 152..158)
BAR_HALF = 0.003
BAR_FILLET = 0.002
BAR_X_WALL = -0.1206                       # flat bar top: gusset end ...
BAR_X_BEND = float(BEND_C[0])              # ... to the bend
STUB_ANG = np.radians(60.0)
A_STUB = np.array([np.cos(STUB_ANG), np.sin(STUB_ANG)])     # (x, z)
CAP_C = np.array([-0.08900, 0.16539])      # stub cap centre (x, z)
CAP_R = 0.003
STUB_LEN = 0.00796                         # bend end -> cap centre
ARC_LEN = R_CL * STUB_ANG                  # 7.33 mm
BOARD_X_FRONT = -0.130                     # wall front face
BOARD_MU = 0.5                             # PLA

# hole resting on the bar: a 10 mm hole hangs on the two 2 mm top fillets
# (centres at y +-1, z 156): centre 156 - sqrt(3^2 - 1^2) = 153.17 mm
def rest_drop(r_hole: float) -> float:
    """How far below the bar centre-line a hole of radius r_hole hangs."""
    fc_y = BAR_HALF - BAR_FILLET
    fc_z = BAR_Z + BAR_HALF - BAR_FILLET
    return float(BAR_Z - (fc_z - np.sqrt((r_hole - BAR_FILLET) ** 2 - fc_y ** 2)))


# ── the bench ───────────────────────────────────────────────────────────
BENCH_X = (-0.200, 0.140)      # front edge (robot side) at +0.140: the ruler is
                               # pinched past it (IK: j4 keeps 0.27 rad there,
                               # 0.14 rad with the edge at +0.200)
BENCH_Y = (-0.250, 0.250)
BENCH_MU = 0.25                # laminated bench top
MU_TORSION = 0.002             # condim-4 torsional row (0 -> MuJoCo NaN warning)

# ── the robot cell ──────────────────────────────────────────────────────
# Scanned (IK, 12 random seeds per pose): with the base on the robot axis
# (y = 0) the horizontal-wrist poses the bar needs put joint 6 on its upper
# stop (4.517 rad) from every base position tried, x 0.32..0.55 m, z
# -0.2..0; moved 0.25 m to the side and yawed at the hook, the same poses
# keep >= 0.48 rad on every joint (thread 0.74, bar at 70/80/90 deg tilt
# 0.61/0.48/0.66 rad for the wrench).
ROBOT_BASE = np.array([0.45, 0.25, Z0])
ROBOT_YAW = float(np.arctan2(0.0 - ROBOT_BASE[1], -0.05 - ROBOT_BASE[0]))

# ── fingers (MuJoCo-era suite values: ethernet / wrench_nut_arm) ───────
FINGER_KE, FINGER_KD = 8000.0, 60.0
FINGER_F_MAX = 15.0            # N per pad (actuator force range)
FINGER_MASS = 0.12             # kg, README pitfall 4.5
PAD_MU = 1.4                   # elastomer pads
PAD_FACE_OFF = 0.00008         # pad face = dof - 0.08 mm (measured, mesh 43)
TIP_BELOW_TCP = 0.0095         # fingertip block bottom, along the approach
PAD_HALF_W = 0.0089            # pad half-width across the approach
ARM_KD_TASK = 400.0            # keychain_hang's smoothness value
DQ_MAX, DDQ_MAX = 0.030, 0.0060   # joint tracker (keychain_hang, pitfall 10.1)

SDF_BOARD = dict(sdf=True, sdf_resolution=384, narrow_band=0.0015, margin=0.0008)


def quat_of_R(R: np.ndarray) -> np.ndarray:
    M = np.eye(4)
    M[:3, :3] = R
    q = trimesh.transformations.quaternion_from_matrix(M)      # w x y z
    return np.array([q[1], q[2], q[3], q[0]])


def R_of_q(q) -> np.ndarray:
    return trimesh.transformations.quaternion_matrix([q[3], q[0], q[1], q[2]])[:3, :3]


def smooth5(u):
    """Quintic smootherstep (zero velocity AND acceleration at both ends;
    keychain_hang measured the cubic's acceleration steps ringing a held
    part)."""
    u = float(np.clip(u, 0.0, 1.0))
    return u ** 3 * (u * (6.0 * u - 15.0) + 10.0)


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


# ── the hook centre-line ────────────────────────────────────────────────
class HookPath:
    """Centre-line of one hook, parametrised by arc length ``u`` measured
    from the cap centre INWARD (toward the wall): u < 0 is outside, beyond
    the tip along the stub axis; 0 .. STUB_LEN the straight stub; then the
    60 deg bend; then the flat bar.  ``elev(u)`` is the local hook axis
    elevation (60 deg on the stub, 0 on the bar)."""

    def __init__(self, peg_y: float):
        self.y = float(peg_y)
        self.u_bend0 = STUB_LEN
        self.u_bend1 = STUB_LEN + ARC_LEN

    def xz(self, u: float) -> np.ndarray:
        if u <= self.u_bend0:
            return CAP_C - u * A_STUB
        if u <= self.u_bend1:
            phi = STUB_ANG - (u - self.u_bend0) / R_CL     # 60 -> 0 deg
            return BEND_C + R_CL * np.array([np.sin(phi), -np.cos(phi)])
        return np.array([BAR_X_BEND - (u - self.u_bend1), BAR_Z])

    def elev(self, u: float) -> float:
        if u <= self.u_bend0:
            return float(STUB_ANG)
        if u <= self.u_bend1:
            return float(STUB_ANG - (u - self.u_bend0) / R_CL)
        return 0.0

    def point(self, u: float) -> np.ndarray:
        x, z = self.xz(u)
        return np.array([x, self.y, z + Z0])

    def u_of_bar_x(self, x: float) -> float:
        return self.u_bend1 + (BAR_X_BEND - x)

    # The plate's tilt along the path.  Following the hook's own tangent
    # exactly would turn the plate 60 deg over the 7.3 mm bend arc, and the
    # FR3 wrist cannot do that at any useful speed (measured offline: j5/j7
    # need > 3 rad/s at 12 mm/s of path).  The hole's angular play is what
    # makes a gentler schedule legal: a sleeve of thickness t with a 10 mm
    # hole on the 6.8 mm (corner-to-corner) bar may sit +-atan(2*1.6/t) off
    # the bar axis -- +-23 deg for the 7.5 mm wrench, +-58 deg for the 2 mm
    # ruler -- so the tilt runs a cubic ramp from the stub-aligned 30 deg to
    # tilt_hi over [bend0 - 2 mm, bend1 + 1.5 mm], never more than 17 deg
    # behind the hook tangent (at the end of the arc).
    TILT_RAMP = (-0.0020, 0.0015)

    def tilt(self, u: float, tilt_lo: float, tilt_hi: float) -> float:
        ua = self.u_bend0 + self.TILT_RAMP[0]
        ub = self.u_bend1 + self.TILT_RAMP[1]
        s = float(np.clip((u - ua) / (ub - ua), 0.0, 1.0))
        s = 3.0 * s * s - 2.0 * s ** 3
        return float(tilt_lo + (tilt_hi - tilt_lo) * s)

    def speed(self, u: float, v_fast: float, v_bend: float) -> float:
        """Path speed: fast down the straight stub, slow wherever the plate
        turns (wrist joint rates stay < 1.8 rad/s)."""
        return v_fast if u < self.u_bend0 + self.TILT_RAMP[0] - 0.001 else v_bend

    def polyline(self, n: int = 160) -> np.ndarray:
        """3-D centre-line from the wall gusset to the cap tip (QA)."""
        u_wall = self.u_of_bar_x(BAR_X_WALL)
        us = np.linspace(-CAP_R, u_wall, n)
        return np.array([self.point(u) for u in us])


# ── plates ──────────────────────────────────────────────────────────────
def load_kit_mesh(name: str) -> trimesh.Trimesh:
    """Finalized-kit STL (metres, kit frame; see bodies_kit/<name>/)."""
    return trimesh.load(KIT_DIR / name / f"{name}.stl", force="mesh")


@dataclass
class Plate:
    """A flat part with a round through hole, lying with its thickness on
    the kit z axis.  Kit-frame quantities: ``hole`` (centre, mid-plane),
    ``com``; ``d`` = unit hole -> centre-of-mass direction in the plate plane
    (the way the part hangs), ``thick``, ``r_hole``.  ``center`` is the AABB
    centre make_newton_mesh puts at the body origin."""
    name: str
    tm: trimesh.Trimesh
    hole: np.ndarray
    r_hole: float
    com: np.ndarray
    d: np.ndarray
    thick: float
    z_mid: float
    center: np.ndarray
    mass: float

    @staticmethod
    def measure(name: str, tm: trimesh.Trimesh, density: float) -> "Plate":
        lo, hi = tm.bounds
        z_mid = 0.5 * (lo[2] + hi[2])
        sec = tm.section(plane_origin=[0, 0, z_mid], plane_normal=[0, 0, 1])
        loops = sorted(sec.discrete, key=lambda lp: np.ptp(lp[:, 0]))
        hole_lp = loops[0][:, :2]                     # the smallest loop
        A = np.c_[2 * hole_lp, np.ones(len(hole_lp))]
        s = np.linalg.lstsq(A, (hole_lp ** 2).sum(1), rcond=None)[0]
        c = s[:2]
        r = float(np.sqrt(s[2] + c @ c))
        hole = np.array([c[0], c[1], z_mid])
        com = np.asarray(tm.center_mass, float)
        d = com - hole
        d[2] = 0.0
        return Plate(name=name, tm=tm, hole=hole, r_hole=r, com=com, d=unit(d),
                     thick=float(hi[2] - lo[2]), z_mid=float(z_mid),
                     center=0.5 * (lo + hi), mass=float(tm.volume * density))

    # kit frame <-> world: R maps kit (z, d, z x d) onto world (n, d, n x d)
    def R_world(self, n, d) -> np.ndarray:
        ez = np.array([0.0, 0.0, 1.0])
        src = np.column_stack([ez, self.d, np.cross(ez, self.d)])
        dst = np.column_stack([n, d, np.cross(n, d)])
        return dst @ src.T

    def body_pose(self, H, n, d):
        """Body (AABB-centred) world pose that puts the hole centre at H with
        plate normal n and hang direction d."""
        R = self.R_world(n, d)
        p = np.asarray(H, float) - R @ (self.hole - self.center)
        return p, quat_of_R(R)

    def frame_of(self, bq_row):
        """(hole centre, normal n, hang direction d) of a body_q row."""
        T = world_T(bq_row)
        R, p = T[:3, :3], T[:3, 3]
        ez = np.array([0.0, 0.0, 1.0])
        H = p + R @ (self.hole - self.center)
        return H, R @ ez, R @ self.d

    def local_mesh(self) -> trimesh.Trimesh:
        m = self.tm.copy()
        m.apply_translation(-self.center)
        return m


def hang_frame(theta: float):
    """Plate normal n and hang direction d for a tilt theta about +y (0 =
    flat face-up with the hang direction along +x, pi/2 = hanging, face
    toward +x)."""
    n = np.array([np.sin(theta), 0.0, np.cos(theta)])
    d = np.array([np.cos(theta), 0.0, -np.sin(theta)])
    return n, d


def tcp_pose_for(grasp_pt, n, d, h_tcp, flip=False):
    """TCP pose for a top-down-style pinch across the plate: approach axis
    -n, closing axis in the plate plane perpendicular to d, TCP h_tcp above
    the grasp point (on the plate mid-plane) along n."""
    z = -unit(n)
    y = unit(np.cross(n, d))
    if flip:
        y = -y
    x = np.cross(y, z)
    return np.asarray(grasp_pt, float) + h_tcp * unit(n), quat_of_R(np.column_stack([x, y, z]))


# ── scene ───────────────────────────────────────────────────────────────
def robot_xform() -> wp.transform:
    return wp.transform(wp.vec3(*ROBOT_BASE.tolist()),
                        wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), ROBOT_YAW))


def setup_robot(rig):
    """FR3 first (coords 0..8), task drives and rubber pads."""
    rig.add_robot(robot_xform())
    for dof in range(7):
        rig.builder.joint_target_kd[dof] = ARM_KD_TASK
    for dof in (7, 8):
        rig.builder.joint_target_ke[dof] = FINGER_KE
        rig.builder.joint_target_kd[dof] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        set_body_mass(rig.builder, bi, FINGER_MASS)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU


def build_bench_and_board(rig, bench_x=BENCH_X, bench_y=BENCH_Y):
    """Analytic bench block (top at Z0), a visual pedestal of the same height
    under the arm, and the hanging board as a static SDF mesh at its CAD
    pose lifted by Z0 (foot on the bench)."""
    x0, x1 = bench_x
    y0, y1 = bench_y
    sid = rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * Z0),
                               wp.quat_identity()),
        hx=0.5 * (x1 - x0), hy=0.5 * (y1 - y0), hz=0.5 * Z0,
        cfg=rig.cfg(mu=BENCH_MU, gap=0.0005, mu_torsional=MU_TORSION), label="table")
    rig.shapes["table"] = [sid]
    rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(float(ROBOT_BASE[0]), float(ROBOT_BASE[1]), 0.5 * Z0),
                               wp.quat_identity()),
        hx=0.12, hy=0.12, hz=0.5 * Z0,
        cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False),
        label="arm_pedestal")
    tm = load_tm(*BOARD_ASSET)
    tm.apply_translation([0.0, 0.0, Z0])
    mesh, c = make_newton_mesh(tm, **SDF_BOARD)
    c = np.asarray(c)
    sid = rig.builder.add_shape_mesh(
        -1, mesh=mesh, xform=wp.transform(wp.vec3(*c), wp.quat_identity()),
        cfg=rig.cfg(mu=BOARD_MU, gap=0.0005, mu_torsional=MU_TORSION), label="board")
    rig.meshes["board"] = (tm, c)
    rig.shapes["board"] = [sid]


def filter_statics_and_ground(rig):
    """Static-static pairs flood the triangle midphase, and the ground
    plane pairs with every triangle; neither carries anything here."""
    statics = list(rig.builder.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            rig.filter_pair(statics[i], statics[j])
    ground = statics[0]
    for b, sids in rig.builder.body_shapes.items():
        for s in sids:
            if s != ground:
                rig.filter_pair(ground, s)


def filter_arm_links(rig, body_label: str, keep_fingers=True):
    """Arm links (not the fingertips) never touch the plate: they are
    filtered, so a link passing near the part cannot shove it.  The pads
    stay live -- they are the grasp."""
    bi = rig.builder.body_label.index(body_label)
    for b, lbl in enumerate(rig.builder.body_label):
        if not str(lbl).startswith("fr3/"):
            continue
        if keep_fingers and "finger" in lbl:
            continue
        for s in rig.builder.body_shapes.get(b, []):
            for ts in rig.builder.body_shapes.get(bi, []):
                rig.filter_pair(s, ts)


# ── probes ──────────────────────────────────────────────────────────────
class PairProbe:
    """Contact force on ONE sensing body from a set of counterpart shapes
    (README pitfall 6.6: a whole-body probe reads everything it touches)."""

    def __init__(self, model, sensing_body, counterpart_shapes):
        labels = list(model.shape_label)
        shapes = [labels.index(s) if isinstance(s, str) else int(s)
                  for s in counterpart_shapes]
        bodies = list(model.body_label)
        b = bodies.index(sensing_body) if isinstance(sensing_body, str) else int(sensing_body)
        self.sensor = newton.sensors.SensorContact(
            model, sensing_bodies=[b], counterpart_shapes=shapes, measure_total=False)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        self.sensor.update(state, contacts)
        F = self.sensor.force_matrix.numpy()[0].sum(axis=0)
        Ff = self.sensor.force_matrix_friction.numpy()[0].sum(axis=0)
        rec = {"t": t, "force": float(np.linalg.norm(F)),
               "friction": float(np.linalg.norm(Ff)),
               "normal": float(np.linalg.norm(F - Ff)), "vec": F.tolist()}
        self.history.append(rec)
        return rec


# ── arm command ─────────────────────────────────────────────────────────
class JointTracker:
    """Jerk-limited joint command (keychain_hang's drive_smooth, README
    pitfall 10.1): the LM-IK solution is tracked under a velocity and an
    acceleration cap, and the limited pose is written back as the IK warm
    start, so a null-space slide of the 7-dof solution can never become a
    one-frame arm whip."""

    def __init__(self, rig, q0):
        self.rig = rig
        self.q = np.asarray(q0[:7], float).copy()
        self.dq = np.zeros(7)
        self.dq_peak = 0.0
        self.ik_err_peak = 0.0

    def step(self, tcp_p, tcp_q, fingers, iters=16):
        ik = self.rig.ik
        q_sol = ik.solve(tcp_p, tcp_q, iterations=iters)
        err = np.asarray(q_sol[:7], float) - self.q
        v_lim = np.minimum(DQ_MAX, np.sqrt(2.0 * DDQ_MAX * np.abs(err)))
        dq_t = np.clip(err, -v_lim, v_lim)
        self.dq = self.dq + np.clip(dq_t - self.dq, -DDQ_MAX, DDQ_MAX)
        self.q = self.q + self.dq
        self.dq_peak = max(self.dq_peak, float(np.abs(self.dq).max()))
        jq = ik.joint_q.numpy()
        jq[0, :7] = self.q
        ik.joint_q.assign(jq)
        qt = self.rig.control.joint_target_q.numpy()
        qt[:7] = self.q
        qt[7:9] = fingers
        self.rig.control.joint_target_q.assign(qt)


def tcp_offset_from_link7(rig):
    """Fixed link7 -> TCP transform (the scene collapses the hand_tcp frame
    into link7), so the rollout can measure the ACTUAL TCP."""
    T7 = world_T(rig.ik.body_pose("fr3/fr3_link7"))
    Tt = world_T(rig.ik.tcp_pose())
    return np.linalg.inv(T7) @ Tt


def actual_tcp(bq, i_link7, T_off):
    return (world_T(bq[i_link7]) @ T_off)[:3, 3]


# ── "is it hung" ────────────────────────────────────────────────────────
def pierce(plate: Plate, bq_row, polyline) -> tuple[float, float]:
    """Where the hook centre-line crosses the plate's mid-plane, relative to
    the hole: (radial offset from the hole centre, x of the crossing).
    Offset 9.9 = the centre-line never crosses the plate plane."""
    H, n, _ = plate.frame_of(bq_row)
    s = (polyline - H) @ n
    best = (9.9, 9.9)
    idx = np.flatnonzero(np.sign(s[:-1]) * np.sign(s[1:]) < 0)
    for i in idx:
        f = s[i] / (s[i] - s[i + 1])
        p = polyline[i] + f * (polyline[i + 1] - polyline[i])
        off = float(np.linalg.norm(p - H))
        if off < best[0]:
            best = (off, float(p[0]))
    return best


def pen_static_fast(qa, body_label, body_local, static_tm, static_center,
                    pair_name, tol=0.0015, n_samples=24):
    """``RolloutQA.check_penetration_static`` with an exact prefilter (the
    same idea as hang_keychain's): same 800-vertex subsample, frames and
    report fields, but vertices outside the static mesh's bounding box are
    skipped -- their signed distance is negative for a closed mesh and the
    check only takes a maximum starting at 0.  trimesh's closest-point
    search on far-away points is what makes the plain version slow on the
    37k-face board."""
    static_local = static_tm.copy()
    static_local.apply_translation(-np.asarray(static_center))
    lo, hi = static_local.bounds[0] - 1e-6, static_local.bounds[1] + 1e-6
    pq = trimesh.proximity.ProximityQuery(static_local)
    bi = qa._idx(body_label)
    va = body_local.vertices
    if len(va) > 800:
        va = va[np.random.RandomState(0).choice(len(va), 800, replace=False)]
    va_h = np.hstack([va, np.ones((len(va), 1))])
    frames = np.linspace(0, len(qa.t) - 1, n_samples).astype(int)
    worst, worst_t = 0.0, None
    for f in frames:
        q = qa.body_q[f, bi]
        T = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
        T[:3, 3] = q[:3] - np.asarray(static_center)
        pts = (T @ va_h.T).T[:, :3]
        pts = pts[((pts > lo) & (pts < hi)).all(axis=1)]
        sd = pq.signed_distance(pts) if len(pts) else np.zeros(0)
        depth = float(sd.max()) if len(sd) else 0.0
        if depth > worst:
            worst, worst_t = depth, float(qa.t[f])
    entry = {"max_depth_mm": round(worst * 1000, 3), "at_t": worst_t,
             "pass": bool(worst < tol)}
    pen = qa.report["checks"].setdefault(
        "penetration", {"pass": True, "tolerance_mm": tol * 1000, "pairs": {}})
    pen["pairs"][pair_name] = entry
    pen["pass"] = bool(pen["pass"] and entry["pass"])
    qa.report["pass"] &= entry["pass"]


# ── the shared expert: hover -> thread -> bend -> seat -> release ──────
@dataclass
class HangCfg:
    """Everything the hang half of the expert needs (both plates)."""
    tilt_hi: float                     # plate tilt at the release (on the bar)
    bar_x: float = -0.1070             # release station on the flat bar
    hover_u: float = -0.035            # hover 35 mm out along the stub axis
    thread_u: float = 0.004            # threaded: 4 mm past the cap centre
    tilt_lo: float = float(np.pi / 2 - STUB_ANG)     # 30 deg: hole axis = stub axis
    v_path: float = 0.012              # m/s down the straight stub
    v_bend: float = 0.0035             # m/s wherever the plate turns
    a_path: float = 0.030
    seat_rate: float = 0.0015          # m/s the hole is lowered onto the bar ...
    seat_frac: float = 0.8             # ... until the bar carries this much of
    seat_max_extra: float = 0.0010     # the weight (or rest height + this)
    t_seat_max: float = 2.5
    corr_k: float = 0.06               # hole loop: integral gain per frame
    corr_step: float = 0.00025         # ... rate limit (m/frame)
    corr_max: float = 0.006            # ... authority (m)
    corr_db: float = 0.00015
    corr_alpha: float = 0.25           # error EMA
    f_gate_lo: float = 0.6             # N plate<->board: full path speed ...
    f_gate_hi: float = 2.5             # ... down to 15 % here
    t_dwell: float = 0.5
    t_seat: float = 0.6
    t_seat_hold: float = 0.5
    t_open: float = 0.5
    t_retreat: float = 1.6
    t_still: float = 1.2
    retreat_n: float = 0.060           # retreat along the plate normal ...
    retreat_up: float = 0.040          # ... then up
    open_extra: float = 0.006          # pads open this much past f_open


HANG_PHASES = ["dwell", "thread", "bend", "seat", "seat_hold", "open", "retreat", "still"]


class HangExpert:
    """Hang half of the expert, closed-loop on the MEASURED hole centre.

    The hole centre follows the hook centre-line (``HookPath``) at a path
    speed that is accel-limited, slow wherever the plate turns and
    throttled by the plate<->board contact force; the plate tilt follows
    ``HookPath.tilt``.  A rate-limited integral on (commanded - measured)
    hole centre shifts the TCP so the HOLE, not the hand, tracks the path
    (it absorbs arm sag and any slip in the pinch).  At the bar the hole is
    lowered onto the bar's top, the pads open, and the hand backs off along
    the plate normal and up."""

    def __init__(self, cfg: HangCfg, hook: HookPath, plate: Plate, tcp_for,
                 f_close: float, f_open: float, fps: int):
        self.cfg, self.hook, self.plate, self.tcp_for = cfg, hook, plate, tcp_for
        self.f_close, self.f_open, self.fps = f_close, f_open, fps
        self.u = cfg.hover_u
        self.v = 0.0
        self.seat = 0.0
        self.corr = np.zeros(3)
        self.ef = np.zeros(3)
        self.rel = None
        self.seated = False
        self.n_loaded = 0
        self.t_seat = 0.0
        self.weight = 9.81 * plate.mass
        self.u_bar = hook.u_of_bar_x(cfg.bar_x)
        self.H_cmd, self.theta = self.target(cfg.hover_u)

    def target(self, u):
        return self.hook.point(u), self.hook.tilt(u, self.cfg.tilt_lo, self.cfg.tilt_hi)

    def hover(self):
        H, th = self.target(self.cfg.hover_u)
        return self.tcp_for(H, th)

    def durations(self):
        c = self.cfg
        return {"dwell": c.t_dwell, "seat_hold": c.t_seat_hold,
                "open": c.t_open, "retreat": c.t_retreat, "still": c.t_still}

    def done(self, k) -> bool:
        if k == "thread":
            return self.u >= self.cfg.thread_u - 1e-6
        if k == "bend":
            return self.u >= self.u_bar - 1e-6
        if k == "seat":
            return self.seated
        return False

    def begin(self, droop):
        """Entering the closed loop: the pick's droop offset becomes the
        loop's initial correction (continuous command)."""
        self.corr = np.asarray(droop, float).copy()

    def _path_step(self, u_end, F_board):
        c = self.cfg
        gate = float(np.clip((c.f_gate_hi - F_board) / (c.f_gate_hi - c.f_gate_lo), 0.15, 1.0))
        rem = max(0.0, u_end - self.u)
        v_t = min(self.hook.speed(self.u, c.v_path, c.v_bend),
                  np.sqrt(2.0 * c.a_path * rem) + 0.0005) * gate
        dt = 1.0 / self.fps
        self.v = float(np.clip(v_t, self.v - c.a_path * dt, self.v + c.a_path * dt))
        self.u = min(u_end, self.u + self.v * dt)

    def step(self, k, s, H_meas, F_board, F_board_z, last_p, last_q):
        """Command for phase k (s = fraction of a fixed-duration phase).
        Returns (tcp_p, tcp_q, finger_dof, closed_loop)."""
        c = self.cfg
        if k in ("dwell", "thread", "bend", "seat", "seat_hold"):
            if k == "thread":
                self._path_step(c.thread_u, F_board)
            elif k == "bend":
                self._path_step(self.u_bar, F_board)
            elif k == "seat":
                # FORCE-GATED, and the hole loop is frozen: a position target
                # below the bar plus an integrator on the hole height pressed
                # the wrench into the bar at 12 N (measured).  Slow over the
                # last 0.4 mm before the expected touch (the bar is stiff:
                # 0.15 mm of overrun read 2.3 N).
                self.t_seat += 1.0 / self.fps
                self.n_loaded = self.n_loaded + 1 if F_board_z >= c.seat_frac * self.weight else 0
                drop0 = rest_drop(self.plate.r_hole)
                limit = drop0 + c.seat_max_extra
                if self.n_loaded >= 3 or self.seat >= limit or self.t_seat >= c.t_seat_max:
                    self.seated = True
                else:
                    rate = c.seat_rate if self.seat < drop0 - 0.0004 else 0.3 * c.seat_rate
                    self.seat = min(limit, self.seat + rate / self.fps)
            elif k == "seat_hold":
                # hand the weight to the bar: back off while the hand still
                # presses the hole into it, so opening the pads releases no
                # stored squeeze
                if F_board_z > 1.5 * self.weight:
                    self.seat -= 0.3 * c.seat_rate / self.fps
                elif F_board_z < 0.5 * self.weight:
                    self.seat += 0.3 * c.seat_rate / self.fps
            H, th = self.target(self.u)
            H = H - np.array([0.0, 0.0, self.seat])
            self.H_cmd, self.theta = H, th
            if k not in ("seat", "seat_hold"):
                e = H - np.asarray(H_meas, float)
                self.ef = self.ef + c.corr_alpha * (e - self.ef)
                ee = np.sign(self.ef) * np.maximum(np.abs(self.ef) - c.corr_db, 0.0)
                self.corr = np.clip(self.corr + np.clip(c.corr_k * ee, -c.corr_step, c.corr_step),
                                    -c.corr_max, c.corr_max)
            p, q = self.tcp_for(H, th)
            return p + self.corr, q, self.f_close, True
        if self.rel is None:
            self.rel = (np.asarray(last_p, float).copy(), np.asarray(last_q, float).copy(),
                        self.theta)
        p_r, q_r, th = self.rel
        n_rel, _ = hang_frame(th)
        f_wide = self.f_open + c.open_extra
        if k == "open":
            return p_r, q_r, self.f_close + (f_wide - self.f_close) * smooth5(s), False
        a = s if k == "retreat" else 1.0
        a1 = smooth5(min(1.0, 1.6 * a))
        a2 = smooth5(max(0.0, 1.6 * a - 0.6))
        p = p_r + c.retreat_n * a1 * n_rel + c.retreat_up * a2 * np.array([0.0, 0.0, 1.0])
        return p, q_r, f_wide, False


class PhaseMachine:
    """Ordered phases; each ends after its fixed duration or when
    ``done(phase)`` says so.  Records the start time of every phase."""

    def __init__(self, phases, durations: dict, done, verbose=True):
        self.phases, self.dur, self.done_fn = list(phases), dict(durations), done
        self.k, self.t0 = self.phases[0], 0.0
        self.t_start = {self.k: 0.0}
        self.finished = False
        self.verbose = verbose

    def advance(self, t):
        k = self.k
        end = (t - self.t0 >= self.dur[k]) if k in self.dur else bool(self.done_fn(k))
        if end:
            i = self.phases.index(k)
            if i + 1 >= len(self.phases):
                self.finished = True
                return
            self.k, self.t0 = self.phases[i + 1], t
            self.t_start[self.k] = t
            if self.verbose:
                print(f"t={t:.2f}s -> {self.k}")

    def frac(self, t):
        return min(1.0, (t - self.t0) / self.dur[self.k]) if self.k in self.dur else 0.0


class DroopComp:
    """Static arm sag, measured on a STATIC command only (keychain_hang,
    README pitfall 8.1) and never while the pads pinch a bench-supported
    part (an integrator chasing the sag through a locked pinch presses the
    part into the bench -- measured 3 N)."""

    def __init__(self, gain=0.35, step=0.0004, limit=0.008):
        self.gain, self.step_, self.limit = gain, step, limit
        self.v = np.zeros(3)
        self.p_prev = None

    def update(self, learn: bool, p_cmd, tcp_meas, last_cmd):
        static = self.p_prev is not None and np.linalg.norm(np.asarray(p_cmd) - self.p_prev) < 5e-5
        if learn and static and last_cmd is not None:
            e = np.asarray(last_cmd) - np.asarray(tcp_meas)
            self.v = np.clip(self.v + np.clip(self.gain * e, -self.step_, self.step_),
                             -self.limit, self.limit)
        self.p_prev = np.asarray(p_cmd, float).copy()


def pen_plate_board(qa, plate: Plate, body_label, board_tm, board_c, hook: HookPath,
                    pair_name, tol=0.0010, n_samples=40):
    """Plate <-> board interpenetration where it matters, both ways: the
    plate's vertices near the HOLE (+ 600 random others) inside the board,
    and the board's HOOK vertices inside the plate.  Sampled frames include
    every frame where the plate is within 25 mm of the hook."""
    loc = plate.local_mesh()
    v = loc.vertices
    hole_b = plate.hole - plate.center
    near = np.linalg.norm(v[:, :2] - hole_b[:2], axis=1) < 0.012
    rnd = np.random.RandomState(0).choice(len(v), min(600, len(v)), replace=False)
    va = np.vstack([v[near][::max(1, int(near.sum() // 1500))], v[rnd]])
    bl = board_tm.copy()
    bl.apply_translation(-np.asarray(board_c))
    lo, hi = bl.bounds[0] - 1e-6, bl.bounds[1] + 1e-6
    pq_b = trimesh.proximity.ProximityQuery(bl)
    pq_p = trimesh.proximity.ProximityQuery(loc)
    bv = board_tm.vertices
    hk = (np.abs(bv[:, 1] - hook.y) < 0.006) & (bv[:, 0] > BAR_X_WALL - 0.002) & \
         (bv[:, 2] > Z0 + 0.145)
    hook_w = bv[hk]
    bi = qa._idx(body_label)
    poly = hook.polyline()
    d_hook = np.array([np.min(np.linalg.norm(poly - qa.body_q[f, bi, :3], axis=1))
                       for f in range(len(qa.t))])
    frames = sorted(set(np.linspace(0, len(qa.t) - 1, n_samples).astype(int).tolist())
                    | set(np.flatnonzero(d_hook < 0.13).tolist()[::3]))
    worst, worst_t, worst_dir = 0.0, None, None
    for f in frames:
        T = world_T(qa.body_q[f, bi])
        pts = (T[:3, :3] @ va.T).T + T[:3, 3] - np.asarray(board_c)
        pts = pts[((pts > lo) & (pts < hi)).all(axis=1)]
        if len(pts):
            dmax = float(pq_b.signed_distance(pts).max())
            if dmax > worst:
                worst, worst_t, worst_dir = dmax, float(qa.t[f]), "plate in board"
        hl = (T[:3, :3].T @ (hook_w - T[:3, 3]).T).T
        m = ((hl > loc.bounds[0]) & (hl < loc.bounds[1])).all(axis=1)
        if m.any():
            dmax = float(pq_p.signed_distance(hl[m]).max())
            if dmax > worst:
                worst, worst_t, worst_dir = dmax, float(qa.t[f]), "hook in plate"
    entry = {"max_depth_mm": round(worst * 1000, 3), "at_t": worst_t, "where": worst_dir,
             "frames_checked": len(frames), "pass": bool(worst < tol)}
    pen = qa.report["checks"].setdefault(
        "penetration", {"pass": True, "tolerance_mm": tol * 1000, "pairs": {}})
    pen["pairs"][pair_name] = entry
    pen["pass"] = bool(pen["pass"] and entry["pass"])
    qa.report["pass"] &= entry["pass"]


class RunLog:
    """Per-frame record the verdict is computed from."""
    KEYS = ("t", "Hx", "Hy", "Hz", "nx", "ny", "nz", "tilt", "pierce_off", "pierce_x",
            "padL", "padR", "board", "board_fz", "bench", "robot_obj",
            "tcp_x", "tcp_y", "tcp_z", "tcp_err", "hole_err", "low_z")

    def __init__(self):
        self.rows = []

    def add(self, **kw):
        self.rows.append([float(kw[k]) for k in self.KEYS])

    def arr(self):
        a = np.array(self.rows)
        return {k: a[:, i] for i, k in enumerate(self.KEYS)}


def common_criteria(qa, rig, plate: Plate, body_label, hook: HookPath, L, tph, fps,
                    t_still, bench_top=Z0):
    """The appendix criterion for a hang, measured with the hand gone, plus
    "picked up from the bench by the fingertips".  L = RunLog.arr()."""
    weight = 9.81 * plate.mass
    t = L["t"]

    def win(a, b):
        return (t >= a) & (t <= b)

    crit = {}
    # picked up: resting on the bench at the start, lifted clear, both pads
    # loaded through the whole carry, never touching the bench again
    rest0 = win(0.2, min(tph.get("descend", 0.5), 0.5))
    carry = win(tph["lift"] + 0.3, tph["thread"])
    lift_mm = float((L["Hz"][win(tph["lift"], tph["thread"])].max() - L["Hz"][0]) * MM)
    both = float(((L["padL"][carry] > 0.5) & (L["padR"][carry] > 0.5)).mean())
    bench_after = float(L["bench"][t >= tph["lift"] + 0.5].max())
    crit["picked_up_from_bench"] = {
        "pass": bool(abs(L["bench"][rest0].mean() - weight) < 0.3 * weight and lift_mm > 50
                     and both > 0.98 and bench_after < 0.01),
        "bench_support_at_start_N": round(float(L["bench"][rest0].mean()), 4),
        "weight_N": round(weight, 4), "lift_mm": round(lift_mm, 1),
        "both_pads_loaded_fraction_carry": round(both, 3),
        "pad_force_mean_carry_N": [round(float(L["padL"][carry].mean()), 2),
                                   round(float(L["padR"][carry].mean()), 2)],
        "bench_contact_after_lift_max_N": round(bench_after, 4),
        "note": "a fingertip pinch; no joint, weld or kinematic attachment exists "
                "between the robot and the part in this scene"}
    # on the hook with the hand gone
    t_end = float(t[-1])
    fin = win(t_end - t_still + 0.2, t_end)
    after = t >= tph["retreat"]
    hz = L["Hz"][fin]
    swing = float(max(np.ptp(L["Hx"][fin]), np.ptp(L["Hy"][fin]), np.ptp(hz)))
    tilt_fin = L["tilt"][fin]
    on_hook = bool((L["pierce_off"][after] < plate.r_hole - 0.001).all()
                   and (L["pierce_x"][after] >= BAR_X_WALL - 0.001).all()
                   and (L["pierce_x"][after] <= CAP_C[0] + CAP_R).all())
    carried = bool(abs(L["board_fz"][fin].mean() - weight) < 0.15 * weight)
    off_bench = bool(L["low_z"][fin].min() > bench_top + 0.002)
    settled = bool(swing < 0.004 and np.abs(tilt_fin - 90.0).max() < 20.0)
    crit["hung_on_hook"] = {
        "pass": bool(on_hook and carried and off_bench and settled),
        "hook_through_hole_every_frame_after_release": on_hook,
        "hook_pierce_offset_mm_max": round(float(L["pierce_off"][after].max()) * MM, 2),
        "hole_radius_mm": round(plate.r_hole * MM, 2),
        "pierce_x_mm": [round(float(L["pierce_x"][after].min()) * MM, 1),
                        round(float(L["pierce_x"][after].max()) * MM, 1)],
        "hook_x_extent_mm": [round(BAR_X_WALL * MM, 1), round((CAP_C[0] + CAP_R) * MM, 1)],
        "board_vertical_support_N": round(float(L["board_fz"][fin].mean()), 4),
        "weight_N": round(weight, 4),
        "lowest_point_above_bench_mm": round(float(L["low_z"][fin].min() - bench_top) * MM, 1),
        "hole_centre_mm": [round(float(v) * MM, 2) for v in
                           (L["Hx"][fin][-1], L["Hy"][fin][-1], L["Hz"][fin][-1])],
        "free_hang_hole_z_mm": round((Z0 + BAR_Z - rest_drop(plate.r_hole)) * MM, 2),
        "tilt_deg_final": round(float(tilt_fin[-1]), 2),
        "residual_swing_mm_ptp": round(swing * MM, 3),
        "window_s": [round(float(t[fin][0]), 2), round(t_end, 2)],
        "note": "from the end of the retreat on, the hook centre-line passes through "
                "the plate's hole (crosses its mid-plane inside the hole) on the hook "
                "itself; over the final window the board's vertical reaction equals "
                "the weight, the plate hangs clear of the bench, and its hole moves "
                "< 4 mm peak-to-peak (settled, tilt within 20 deg of vertical)"}
    # released and clear; robot stopped
    i_rel = int(np.searchsorted(t, tph["open"]))
    tcp = np.column_stack([L["tcp_x"], L["tcp_y"], L["tcp_z"]])
    moved = float(np.linalg.norm(tcp[-1] - tcp[i_rel]))
    from force_bench.appendix_tasks.common import check_released_and_clear
    check_released_and_clear(qa, t, L["robot_obj"], tph["still"], extra={
        "tcp_moved_from_release_mm": round(moved * MM, 1), "min_move_mm": 50.0})
    rel = qa.report["checks"]["released_and_clear"]
    rel["pass"] = bool(rel["pass"] and moved >= 0.050)
    crit["released_and_clear"] = rel
    d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
    labels = list(d["body_labels"])
    arm_ix = [i for i, l in enumerate(labels) if str(l).startswith("fr3/")]
    ts = d["t"]
    m = ts >= t_end - t_still + 0.2
    v_arm = float(np.linalg.norm(np.diff(d["body_q"][m][:, arm_ix, :3], axis=0), axis=-1).max()) * fps
    crit["robot_stopped"] = {"pass": bool(v_arm < 0.005),
                             "max_link_speed_final_window_mm_s": round(v_arm * MM, 3),
                             "threshold_mm_s": 5.0}
    return crit


def set_camera_fov(rig, cam_pos, cam_target, fov_deg: float):
    """Rig.finalize builds the recorder at the suite's 40 deg fov; a narrower
    lens frames the hook closer without moving the camera into the arm's
    way.  Same renderer, palette, lights and resolution -- only the fov."""
    if abs(fov_deg - 40.0) < 1e-6 or rig.rec is None:
        return
    from force_bench.recorder import RolloutRecorder
    rig.rec = RolloutRecorder(rig.model, rig.out_dir, cam_pos=cam_pos, cam_target=cam_target,
                              fps=rig.fps, fov_deg=float(fov_deg))


def preview_cameras(rig, states_npz, times, cams, out_png, width=640, height=480,
                    fov_deg=40.0):
    """Camera contact sheet WITHOUT re-running physics: assign the saved
    body poses of ``states_npz`` at ``times`` to a fresh state and render
    them from each (target, offset) in ``cams`` (one row per camera).
    Same palette, lights and backdrop as the rollout video."""
    import os
    import tempfile
    from PIL import Image, ImageDraw
    from force_bench.recorder import RolloutRecorder
    os.environ["TB_MULTIVIEW"] = "0"
    d = np.load(states_npz, allow_pickle=True)
    t_arr, bq = d["t"], d["body_q"]
    st = rig.model.state()
    rows = []
    tmp = Path(tempfile.mkdtemp(prefix="hang_preview_"))
    for ci, (tgt, off) in enumerate(cams):
        tgt = np.asarray(tgt, float)
        fov = float(off[3]) if len(off) > 3 else fov_deg
        rec = RolloutRecorder(rig.model, tmp / f"c{ci}", cam_pos=tgt + np.asarray(off[:3], float),
                              cam_target=tgt, width=width, height=height, supersample=1,
                              fov_deg=fov)
        ims = []
        for t in times:
            f = int(np.argmin(np.abs(t_arr - t)))
            st.body_q.assign(bq[f].astype(np.float32))
            rec.capture(st, float(t_arr[f]))
            im = Image.fromarray(rec.frames[-1])
            ImageDraw.Draw(im).text((6, 6), f"cam{ci} t={t_arr[f]:.2f} fov {fov:.0f}",
                                    fill=(255, 255, 0))
            ims.append(im)
        if rec._writer is not None:
            rec._writer.close()
        rows.append(ims)
    sheet = Image.new("RGB", (width * len(times), height * len(cams)))
    for r, ims in enumerate(rows):
        for c, im in enumerate(ims):
            sheet.paste(im, (c * width, r * height))
    sheet.save(out_png)
    print(f"preview sheet -> {out_png}")


def parse_cams(spec: str):
    """'tx,ty,tz:ox,oy,oz;...' -> [(target, offset), ...]"""
    out = []
    for item in spec.split(";"):
        a, b = item.split(":")
        out.append((np.array([float(v) for v in a.split(",")]),
                    np.array([float(v) for v in b.split(",")])))
    return out


def save_json(path: Path, obj: dict):
    def _default(o):
        if isinstance(o, (np.bool_,)):
            return bool(o)
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, np.floating):
            return float(o)
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(type(o))
    Path(path).write_text(json.dumps(obj, indent=1, default=_default))
