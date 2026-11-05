"""Shared scene and expert pieces for the door tasks on the FINAL kit door (21, 27, 28, 29).

The door system is the kit library's ``door`` mechanism (``force_bench.kit.mechanisms.door``): the final printable
CAD of the leaf, both frame halves and the two modules, assembled at the poses the project site's assembled view
gives them, lying flat on the table like the real fixture.  The module variants (unlock angle, return spring, grip;
the lock's unlock angle) are the website's configuration options and come with their parameters from the CAD or the
measured spring fits (see the mechanism modules).

World layout (metres): the door's CAD frame is turned +90 deg about z and its frame footprint centred on
``DOOR_C``, so the hinge runs along world +y at the
LEFT edge seen from the camera (x = -0.122, z = 0.0225), the strike jamb is at +x, the knob or lever sits back-right
(+y), the lock front-right, both bolts retract along -x, both spindles are vertical and retracting is a CLOCKWISE turn
seen from above; opening LIFTS the strike edge (negative rotation about +y).

LOCKED IS PHYSICS.  The thrown bolts sit in the strike pockets of the frame (analytic walls on the CAD's pocket
volumes) and every task opens with an INVISIBLE LOCK PROBE: for ``LOCK_PROBE_S`` an opening
torque ``LOCK_PROBE_NM`` (about 3x the leaf's weight moment) is applied at the hinge while the arm hovers, and QA
gates that the leaf did not move.
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

from force_bench.appendix_tasks.common import appendix_rig
from force_bench.kit import add_asset, load_kit
from force_bench.kit.build import pose_matrix

ROBOT_BASE = (0.15, 0.58, 0.0)          # IK-swept for the knob, the lever and the key
TABLE_C, TABLE_HALF = (0.05, 0.24), 0.42
DOOR_C = np.array([0.0, 0.10])          # frame footprint centre on the table
DOOR_POSE = pose_matrix(xyz=(DOOR_C[0], DOOR_C[1], 0.0),
                        R=np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]))   # +90 deg about z
CAM_TARGET = np.array([0.01, 0.11, 0.10])
CAM_POS = np.array([0.10, -0.38, 0.42])
HINGE_AXIS = np.array([0.0, 1.0, 0.0])
SPINDLE = np.array([0.0, 0.0, 1.0])

LOCK_PROBE_S = (0.05, 0.45)
LOCK_PROBE_NM = 1.5
LOCK_PROBE_MAX_DEG = 0.6
_IDENT = np.array([0, 0, 0, 0, 0, 0, 1.0])


def make_rig(name: str):
    rig = appendix_rig(name)
    rig.add_robot(wp.transform(wp.vec3(*ROBOT_BASE), wp.quat_identity()))
    return rig


def build_door(rig, knob: dict | str | None = None, lock: str | dict | None = None, key_mode: str | None = None,
               knob_params: dict | None = None, lock_params: dict | None = None):
    """Table + the kit door (both modules) at DOOR_POSE; returns the door's AssetHandle."""
    rig.table(center_xy=TABLE_C, half=TABLE_HALF)
    mods = {}
    if knob is not None:
        mods["doorknob-module"] = knob
    if lock is not None:
        mods["lock-module"] = lock
    h = add_asset(rig, "door", pose=DOOR_POSE, modules=mods, key_mode=key_mode,
                  knob=knob_params, lock=lock_params)
    return h


def arm_filter(rig, body: int, keep_fingers: bool = True):
    """Never collide the arm (or, with keep_fingers=False, the fingers too) with ``body``: the gripped part touches
    only fingertips (no coupling joint)."""
    hand = rig.builder.body_label.index("fr3/fr3_link7")
    for ts in rig.builder.body_shapes.get(body, []):
        for bl in range(0, hand + 3):
            if keep_fingers and "finger" in rig.builder.body_label[bl]:
                continue
            for s in rig.builder.body_shapes.get(bl, []):
                rig.builder.add_shape_collision_filter_pair(s, ts)


def fingers_filter(rig, shape_ids):
    """Disable fingertip contact against specific shapes (the drawn meshes of a part whose hull carries the grasp)."""
    for bl, lab in enumerate(rig.builder.body_label):
        if "finger" not in lab:
            continue
        for s in rig.builder.body_shapes.get(bl, []):
            for t in shape_ids:
                rig.builder.add_shape_collision_filter_pair(s, t)


def drawn_shapes(rig, body: int, keep: list[int]) -> list[int]:
    """Every shape on ``body`` except ``keep`` (the drawn printed meshes next to a hull stand-in)."""
    return [s for s in rig.builder.body_shapes.get(body, []) if s not in set(keep)]


class HingeProbe:
    """Opening torque on the hinge dof, ADDED to the build-time joint torques (the knob spring's preload lives in
    Control.joint_f: a zero-initialised buffer would wipe it)."""

    def __init__(self, rig, h):
        self.rig = rig
        self.dof = int(rig.model.joint_qd_start.numpy()[h.joints["hinge"]])
        self.base = rig.control.joint_f.numpy().copy()
        self.tau = None

    def set(self, tau: float):
        if tau == self.tau:
            return
        f = self.base.copy()
        f[self.dof] += float(tau)
        self.rig.control.joint_f.assign(f)
        self.tau = tau


# ── measured-state readers ─────────────────────────────────────────────────────────────────────────────────────────
def quat_of(row):
    return np.array([row[3], row[4], row[5], row[6]], dtype=float)


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def rel_twist(child_row, parent_row, axis) -> float:
    q = qmul(qconj(quat_of(parent_row)), quat_of(child_row))
    return 2.0 * np.arctan2(float(q[:3] @ np.asarray(axis)), float(q[3]))


def door_angle(leaf_row) -> float:
    """Leaf rotation about the hinge [rad]; negative = open (lifted)."""
    return rel_twist(leaf_row, _IDENT, HINGE_AXIS)


class Hinge:
    """The hinge axis in the world (through ``point``, along +y) for arcs and lever arms."""

    def __init__(self, h):
        p = np.asarray(h.info["hinge_point_world"], dtype=float)
        self.x, self.z = float(p[0]), float(p[2])

    def arc(self, p, theta) -> np.ndarray:
        """Rotate a world point about the hinge axis by ``theta`` (about +y)."""
        p = np.asarray(p, dtype=float)
        dx, dz = p[0] - self.x, p[2] - self.z
        c, s = np.cos(theta), np.sin(theta)
        return np.array([self.x + c * dx + s * dz, p[1], self.z - s * dx + c * dz])

    def radius(self, p) -> float:
        p = np.asarray(p, dtype=float)
        return float(np.hypot(p[0] - self.x, p[2] - self.z))


def spin(p, phi, axis_xy) -> np.ndarray:
    """Rotate a point about a vertical spindle at ``axis_xy`` by ``phi`` (about +z), door shut."""
    p = np.asarray(p, dtype=float)
    dx, dy = p[0] - axis_xy[0], p[1] - axis_xy[1]
    c, s = np.cos(phi), np.sin(phi)
    return np.array([axis_xy[0] + c * dx - s * dy, axis_xy[1] + s * dx + c * dy, p[2]])


def _qaxis(axis, ang):
    a = np.asarray(axis, dtype=float)
    a = a / np.linalg.norm(a)
    s = np.sin(ang / 2.0)
    return np.array([a[0] * s, a[1] * s, a[2] * s, np.cos(ang / 2.0)])


Q_DOWN = _qaxis((1.0, 0.0, 0.0), np.pi)        # TCP +z -> world -z (fingers down)


def down_quat(yaw=0.0, door_th=0.0) -> np.ndarray:
    """Top-down TCP pose (xyzw): yaw = 0 puts the finger gap along world -y; ``door_th`` carries the grasp round the
    hinge with the leaf."""
    return qmul(_qaxis(HINGE_AXIS, door_th), qmul(_qaxis((0, 0, 1.0), yaw), Q_DOWN))


def smooth(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return u * u * (3.0 - 2.0 * u)


def joint_reader(rig, h):
    """{role: index into state.joint_q} for the door's joints."""
    qs = rig.model.joint_q_start.numpy()
    return {r: int(qs[j]) for r, j in h.joints.items()}


# ── QA ─────────────────────────────────────────────────────────────────────────────────────────────────────────────
def lock_probe_check(log_t, log_th, limit_deg=LOCK_PROBE_MAX_DEG):
    t = np.asarray(log_t)
    th = np.degrees(np.asarray(log_th))
    m = (t >= LOCK_PROBE_S[0]) & (t <= LOCK_PROBE_S[1] + 0.1)
    travel = float(np.abs(th[m]).max()) if m.any() else float("nan")
    return {"pass": bool(m.any() and travel < limit_deg), "probe_torque_Nm": LOCK_PROBE_NM,
            "probe_window_s": list(LOCK_PROBE_S), "peak_leaf_travel_deg": round(travel, 3), "limit_deg": limit_deg,
            "note": "opening torque ~3x the leaf's own weight moment applied at the hinge before the hand arrives; "
                    "the thrown bolts in their strike pockets must hold the leaf shut"}


def pads_check(rig, t0, t1, min_n=2.0, frames=3):
    """Per-pad contact force > ``min_n`` in every ``frames``-frame window (50 ms) of [t0, t1].

    The probes sample each pad's force once per frame, and under a stiff spring torque the pinch can see-saw between
    the pads within a frame: on a single frame one pad reads 0 while the other reads the pair's whole squeeze (measured
    on the 10 mm spring knob: 0 / 72 N, then 58 / 38 N, knob angle and lift unaffected).  A lost grasp is a RUN of
    frames, so the gate takes, per pad, the minimum over the run of its rolling ``frames``-frame maximum; the raw
    per-frame minimum is reported too."""
    out, segs = {}, {}
    for side in ("left", "right"):
        segs[side] = np.array([r["force"] for r in rig.probes[f"fr3/fr3_{side}finger"].history if t0 <= r["t"] <= t1])
        f = segs[side]
        roll = np.array([f[max(0, i - frames + 1):i + 1].max() for i in range(frames - 1, len(f))]) if len(f) else f
        out[f"{side}_min_N"] = round(float(f.min()), 2)
        out[f"{side}_min_over_{frames}_frames_N"] = round(float(roll.min()), 2)
        out[f"{side}_mean_N"] = round(float(f.mean()), 2)
        out[f"{side}_frames_below_{min_n:g}N"] = int((f < min_n).sum())
        segs[side] = roll
    out["pass"] = bool(min(segs["left"].min(), segs["right"].min()) > min_n)
    out["window_s"] = [round(t0, 2), round(t1, 2)]
    return out


def _local(tm_cad, offset=None):
    m = tm_cad.copy()
    if offset is not None:
        m.apply_transform(offset)
    return m


def door_penetration(qa, rig, h, n_frames: int = 48):
    """Geometric audit of the door system over the rollout (solver-independent, trimesh):

    * movers vs the leaf panel: FILTERED pairs (they turn/slide in their own bores); tolerance = printed clearance;
    * leaf vs the frame rails: FILTERED (the hinge is the joint); tolerance 1.2 mm;
    * bolt meshes vs the strike-pocket wall boxes: the live blocking contact; tolerance 0.6 mm.

    Body frames of the kit door are the parts' CAD frames, so the local meshes are the kit parts (the latch at its
    rest offset)."""
    from force_bench.kit.mechanisms import door as D
    from force_bench.kit.mechanisms.doorknob_module import LATCH_REST_MM
    from force_bench.kit.mechanisms._door_parts import MM, T
    kit = load_kit()
    hk, hl = h._modules["doorknob-module"], h._modules["lock-module"]
    door_parts = {p.name: p for p in kit["door"].parts()}
    leaf = door_parts["door"].mesh()
    km = {p.name: p for p in kit["doorknob-module"].parts({k: v for k, v in hk.choice.items() if v})}
    lm = {p.name: p for p in kit["lock-module"].parts({k: v for k, v in hl.choice.items() if v})}
    latch = _local(km[hk.info["latch_part"]].mesh(), T(np.asarray(LATCH_REST_MM) * MM))
    knob = trimesh.util.concatenate([km[hk.info["grip_part"]].mesh(),
                                     next(p for n, p in km.items() if n.startswith("doorknob_module_gear")).mesh()])
    plug = lm[hl.info["plug_part"]].mesh()
    bolt = lm[hl.info["deadbolt_part"]].mesh()
    L = {r: rig.model.body_label[b] for r, b in h.bodies.items()}
    Td = np.asarray(h.pose)
    # each mesh in its own body's frame (module CAD frames for the movers, the door CAD frame for the leaf)
    qa.check_penetration([(L["knob"], knob, L["leaf"], leaf), (L["latch"], latch, L["leaf"], leaf),
                          (L["plug"], plug, L["leaf"], leaf), (L["deadbolt"], bolt, L["leaf"], leaf)],
                         tol=0.0035)
    frames = np.linspace(0, len(qa.t) - 1, n_frames).astype(int)
    # leaf vs frame rails (static): the leaf body frame is the door CAD frame in the world
    for fname in ("door_frame_1", "door_frame_2"):
        tm = door_parts[fname].mesh()
        tm.apply_transform(Td)
        qa.check_penetration_static(L["leaf"], leaf, tm, np.zeros(3), f"leaf->{fname}", tol=0.0012)
    # bolts vs their pocket walls
    worst = {}
    for role, mesh, pocket in (("latch", latch, "latch"), ("deadbolt", bolt, "deadbolt")):
        v = np.asarray(mesh.vertices)
        boxes = []
        for c, half in D.pocket_walls(pocket):
            Tw = Td @ T(np.asarray(c) * MM)
            boxes.append((np.linalg.inv(Tw), np.asarray(half) * MM))
        bi = qa._idx(L[role])
        w = 0.0
        for f in frames:
            q = qa.body_q[f, bi]
            M = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
            M[:3, 3] = q[:3]
            pw = (M @ np.hstack([v, np.ones((len(v), 1))]).T).T
            for inv, half in boxes:
                p = (inv @ pw.T).T[:, :3]
                d = half[None, :] - np.abs(p)
                msk = (d > 0).all(axis=1)
                if msk.any():
                    w = max(w, float(d[msk].min(axis=1).max()))
        worst[role] = w
    pen = qa.report["checks"]["penetration"]
    for role, w in worst.items():
        e = {"max_depth_mm": round(w * 1000, 3), "tol_mm": 0.6, "pass": bool(w < 0.0006),
             "note": "bolt mesh vs the strike-pocket wall boxes: the live blocking contact"}
        pen["pairs"][f"{role}->strike_pocket"] = e
        pen["pass"] = bool(pen["pass"] and e["pass"])
        qa.report["pass"] &= e["pass"]
