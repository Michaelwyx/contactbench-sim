"""Shared pieces of the door-family mechanisms (``doorknob_module``, ``lock_module``, ``door``).

Frames.  Every body of a module is created with its frame AT the module's CAD frame (in its rest pose), so a part
placed by ``body_from_cad`` is the identity (or a pure rest offset) and every joint axis is a plain CAD axis.  A module
can hang off any parent body (the door leaf) or off the world (lying on the table): ``body_T`` reads the parent's
build-time pose and the joint frames are expressed relative to it.

Gears.  A rack-and-pinion is a MuJoCo joint equality (``mjEQ_JOINT``, Newton mimic) ``rack_q = ratio * pinion_q``: a
bilateral, build-time coupling, so the mechanism needs nothing from a task's control callback (physics_notes pitfall
39) and a load on the bolt is felt at the knob, and the other way round, like the printed spur pair.  MuJoCo's default
equality impedance is far too soft for it: the solver regularises the row with ``diagApprox``, which for a joint
equality adds the pinion's ROTATIONAL inverse inertia to the rack's inverse mass with no gear-ratio scaling, so a light
knob makes the coupling compliant.  Measured on a 15 mm pinion with a 15 g rack and 0.48 N.m/rad on the spindle, a
5 N load on the rack slipped it 4.7 mm against the pinion with ``solref (0.002, 1)`` and 15 mm (to its stop) with the
default ``(0.02, 1)``.  ``GEAR_SOLREF`` = (2 substeps, critically damped) with ``GEAR_SOLIMP`` (0.99, 0.999) and
``SPINDLE_ARMATURE`` on the pinion dof brings the same 5 N kicks to <= 0.06 mm of slip while the knob swings through
both of its stops, with finite state throughout.

Stops.  Joint limits carry the printed end stops; they are authored as raw MuJoCo ``solreflimit`` (2 substeps) at
build time so they are equally hard in ``force-bench kit check`` and in a task (``Rig.hard_joint_limits`` sets the
same value on every dof).
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.kit.build import add_part_shape, pose_matrix, to_wp

MM = 1e-3
GEAR_SOLIMP = (0.99, 0.999, 0.001, 0.5, 2.0)
GEAR_DAMPRATIO = 1.0
LIMIT_DAMPRATIO = 0.5
SPINDLE_ARMATURE = 1.0e-3     # kg.m^2 on a pinion dof: numerical ballast for the gear equality (see above); a
# printed 40 g knob has ~1.5e-5.  It costs 0.002 N.m at 2 rad/s^2 of spin-up, far below the 0.15 N.m spring preload.


def T(xyz=(0.0, 0.0, 0.0), R=None) -> np.ndarray:
    return pose_matrix(xyz=xyz, R=R)


def sim_dt(rig) -> float:
    return 1.0 / (rig.fps * rig.substeps)


def ensure_mujoco_attributes(rig) -> None:
    """Register SolverMuJoCo's custom attributes once (equalities, per-dof solreflimit).  Re-registering with the same
    specification is allowed, so Rig.finalize may do it again."""
    if not getattr(rig, "_kit_mjc_attrs", False):
        newton.solvers.SolverMuJoCo.register_custom_attributes(rig.builder)
        rig._kit_mjc_attrs = True


def picked(h) -> dict:
    """The handle's variant choice without the unused alternatives of a shared slot (``None``), as
    ``Asset.parts`` wants it."""
    return {k: v for k, v in (h.choice or {}).items() if v}


def body_T(rig, body: int) -> np.ndarray:
    """World pose of ``body`` as built (identity for the world, -1)."""
    if body < 0:
        return np.eye(4)
    q = np.asarray(rig.builder.body_q[body], dtype=float)      # px py pz qx qy qz qw
    M = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
    M[:3, 3] = q[:3]
    return M


def add_link(rig, world_T: np.ndarray, label: str, **kw) -> int:
    return rig.builder.add_link(xform=to_wp(world_T), label=label, **kw)


def place_parts(rig, body: int, asset, parts, world_from_cad: np.ndarray, label: str, collide: bool = True,
                mu: float | None = None, offset: np.ndarray | None = None, sdf_res: int | None = None) -> dict:
    """Parts as mesh shapes on ``body`` (kit colours, labels ``kit/<label>/<part>``).  ``offset`` is a CAD-frame
    transform applied before placing (a bolt's rest offset)."""
    body_from_cad = np.linalg.inv(body_T(rig, body)) @ world_from_cad
    if offset is not None:
        body_from_cad = body_from_cad @ offset
    out = {}
    for p in parts:
        out[p.name] = [add_part_shape(rig, body, asset, p, body_from_cad, label, collide=collide, mu=mu,
                                      sdf_res=sdf_res)]
    return out


def hull_shape(rig, body: int, pts_cad: np.ndarray, world_from_cad: np.ndarray, label: str, mu: float,
               offset: np.ndarray | None = None) -> int:
    """An invisible, massless convex hull of ``pts_cad`` (metres, CAD frame) on ``body``: the load-bearing collider
    for fingertips (convex-vs-triangle contact is exact and stiff; mesh-vs-mesh through an SDF is far too compliant
    for a pinch)."""
    hull = trimesh.convex.convex_hull(np.asarray(pts_cad, dtype=float))
    body_from_cad = np.linalg.inv(body_T(rig, body)) @ world_from_cad
    if offset is not None:
        body_from_cad = body_from_cad @ offset
    hull.apply_transform(body_from_cad)
    c = 0.5 * (hull.bounds[0] + hull.bounds[1])
    mesh = newton.Mesh(np.asarray(hull.vertices - c, dtype=np.float32), np.asarray(hull.faces.flatten(), dtype=np.int32))
    cfg = rig.cfg(mu=mu, density=0.0, is_visible=False)
    sid = rig.builder.add_shape_convex_hull(body, xform=wp.transform(wp.vec3(*c), wp.quat_identity()), mesh=mesh,
                                            cfg=cfg, label=label)
    return sid


def box_shape(rig, body: int, world_T: np.ndarray, half, label: str, mu: float = 0.4, visible: bool = False) -> int:
    """A box collider (centre/orientation ``world_T``) on ``body``; invisible and massless unless ``visible``."""
    local = np.linalg.inv(body_T(rig, body)) @ world_T
    cfg = rig.cfg(mu=mu, density=0.0, is_visible=visible)
    return rig.builder.add_shape_box(body, xform=to_wp(local), hx=float(half[0]), hy=float(half[1]),
                                     hz=float(half[2]), cfg=cfg, label=label)


def limit_attributes(rig, dampratio: float = LIMIT_DAMPRATIO) -> dict:
    from newton._src.solvers.mujoco.constants import SOLREF_MODE_RAW
    ensure_mujoco_attributes(rig)
    return {"mujoco:solreflimit": wp.vec2(2.0 * sim_dt(rig), float(dampratio)),
            "mujoco:solreflimit_mode": int(SOLREF_MODE_RAW)}


def joint(rig, kind: str, parent: int, child: int, world_anchor: np.ndarray, axis, label: str,
          hard_limits: bool = True, hard_friction: bool = False, **kw) -> int:
    """A revolute/prismatic joint whose anchor frame is ``world_anchor`` (axis in that frame) for both bodies.

    ``hard_friction``: the dof's dry friction (``friction=``, MuJoCo frictionloss) gets ``solreffriction`` = (2
    substeps, 1).  MuJoCo's default (0.02 s) makes frictionloss a VISCOUS force below a sliding speed set by the
    solver's regularisation, which on a light, gear-coupled slide never reaches the friction value: a 1.25 N deadbolt
    friction read 0.06 N at 4 mm/s (lock-module 360 deg probe)."""
    Tp, Tc = body_T(rig, parent), body_T(rig, child)
    px = to_wp(np.linalg.inv(Tp) @ world_anchor)
    cx = to_wp(np.linalg.inv(Tc) @ world_anchor)
    fn = rig.builder.add_joint_revolute if kind == "revolute" else rig.builder.add_joint_prismatic
    if hard_limits:
        kw.setdefault("custom_attributes", {}).update(limit_attributes(rig))
    if hard_friction:
        kw.setdefault("custom_attributes", {})["mujoco:solreffriction"] = wp.vec2(2.0 * sim_dt(rig), 1.0)
    j = fn(parent=parent, child=child, parent_xform=px, child_xform=cx, axis=tuple(float(a) for a in axis),
           label=label, **kw)
    rig._joint_of[label] = j
    return j


def gear(rig, follower: int, leader: int, ratio: float, label: str) -> int:
    """``follower_q = ratio * leader_q`` as a stiff MuJoCo joint equality (see the module docstring)."""
    from newton._src.core.types import vec5
    from newton._src.solvers.mujoco.utils import mjc_add_equality_mimic
    ensure_mujoco_attributes(rig)
    eq, _ = mjc_add_equality_mimic(rig.builder, follower, leader, [0.0, float(ratio), 0.0, 0.0, 0.0], label, True,
                                   {"mujoco:eq_solref": wp.vec2(2.0 * sim_dt(rig), GEAR_DAMPRATIO),
                                    "mujoco:eq_solimp": vec5(*GEAR_SOLIMP)})
    return eq


def dof_of(rig, j: int) -> int:
    return int(rig.builder.joint_qd_start[j])


def add_joint_force(rig, j: int, value: float) -> None:
    """A constant generalised force on joint ``j``'s dof, authored on the builder: it seeds ``Control.joint_f`` and
    stays there.  Anything that writes joint_f later must start from ``control.joint_f`` (not from zeros)."""
    rig.builder.joint_f[dof_of(rig, j)] += float(value)


# ── probes ─────────────────────────────────────────────────────────────────────────────────────────────────────────
class SpinDriver:
    """A probe motor: a light body on a world-root revolute with a stiff position drive, and a gear equality
    ``target = driver`` in JOINT space (so it drives a spindle on a moving leaf just as well; a world-root joint
    needs no articulation of its own).  Its drive torque ke*(target - q) - kd*qd is the torque the mechanism needs,
    i.e. a wrist F/T reading for a robot turning the part at a commanded speed."""

    def __init__(self, rig, world_anchor: np.ndarray, target_joint: int, label: str,
                 ke: float = 50.0, kd: float = 0.6, lo: float = -7.0, hi: float = 1.0):
        self.rig, self.label, self.ke, self.kd = rig, label, float(ke), float(kd)
        self.body = add_link(rig, world_anchor, label, mass=0.005,
                             inertia=wp.mat33(*(np.eye(3) * 1.0e-6).flatten().tolist()))
        self.joint = joint(rig, "revolute", -1, self.body, world_anchor, (0.0, 0.0, 1.0), label,
                           target_pos=0.0, target_ke=self.ke, target_kd=self.kd, armature=SPINDLE_ARMATURE,
                           limit_lower=lo, limit_upper=hi)
        rig.builder.add_articulation([self.joint])
        gear(rig, target_joint, self.joint, 1.0, f"{label}_coupling")

    def bind(self):
        """After finalize: indices into the state and control arrays."""
        m = self.rig.model
        self.q = int(m.joint_q_start.numpy()[self.joint])
        self.qd = int(m.joint_qd_start.numpy()[self.joint])
        self.tgt = int(m.joint_target_q_start.numpy()[self.joint])
        return self

    def torque(self, state, target: float) -> float:
        """The drive torque of the last substep.  MuJoCo applies the proportional term at the substep's START
        position (the damping implicitly at its end velocity), so q is taken back by qd * dt: without that the
        readout carries ke * qd * dt (0.04 N.m at 90 deg/s), more than a lock's whole friction."""
        q = float(state.joint_q.numpy()[self.q])
        qd = float(state.joint_qd.numpy()[self.qd])
        return self.ke * (target - (q - qd * sim_dt(self.rig))) - self.kd * qd


class SubstepTargets:
    """Solver wrapper that sets joint targets from ``schedule(t) -> {target index: value}`` every SUBSTEP, so a probe
    ramp is smooth (a per-frame step kicks a stiff drive by newtons, see examples/force_response)."""

    def __init__(self, inner, schedule):
        self._inner, self.schedule, self.t = inner, schedule, 0.0

    def step(self, s_in, s_out, control, contacts, dt):
        self.t += dt
        vals = self.schedule(self.t)
        if vals:
            qt = control.joint_target_q.numpy()
            for i, v in vals.items():
                qt[i] = v
            control.joint_target_q.assign(qt)
        return self._inner.step(s_in, s_out, control, contacts, dt)

    def __getattr__(self, n):
        return getattr(self._inner, n)


def ramp(t: float, t0: float, t1: float, a: float, b: float) -> float:
    """a -> b linearly over [t0, t1], held outside."""
    if t <= t0:
        return a
    if t >= t1:
        return b
    return a + (b - a) * (t - t0) / (t1 - t0)


def fit_line(x, y):
    """Least-squares y = c0 + c1 x; returns (c0, c1, rms)."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) < 2:
        return float("nan"), float("nan"), float("nan")
    A = np.column_stack([np.ones_like(x), x])
    c, *_ = np.linalg.lstsq(A, y, rcond=None)
    return float(c[0]), float(c[1]), float(np.sqrt(np.mean((A @ c - y) ** 2)))
