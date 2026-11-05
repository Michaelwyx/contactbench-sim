"""Newton models of the Fin Ray finger, alone or on the FR3's two carriages.

Two models, chosen by ``FinRayParams.model``:

``rigid``  the finger's STL as one SDF mesh on the carriage body: the
           geometry of the finger and nothing else (the minimum).
``prb``    the pseudo-rigid-body linkage of ``prb.py``: contact wall, back
           wall and ribs as rigid links on spring-loaded revolute joints
           about the finger's width axis, the ribs and the tip closed by
           FIXED loop joints (MuJoCo WELD equalities).  The joints are spring
           drives (target 0 = the printed shape) with damping and armature
           chosen for the solver, not the material:

  armature  a = k (dt / OMEGA_DT)^2 on every joint, so every spring's own
            omega*dt is OMEGA_DT whatever its rate.  The links are tenths of a
            gram on springs of up to 7 N.m/rad (the 6 mm back wall's root):
            bare, omega*dt would be ~8 and the explicit spring diverges.  All
            modes then sit at ~OMEGA_DT/dt rad/s (~670 rad/s, a 9 ms period
            at 32 substeps of a 60 fps frame): fast against a grasp, irrelevant
            for the statics.
  damping   kd = 2 ZETA sqrt(k a)  (TPU is lossy; ZETA is a choice)
  welds     MuJoCo's default equality solref is (0.02 s, 1): a 20 ms spring,
            far softer than the walls.  ``tune_solver`` sets the loop welds to
            (EQ_TC_DT * dt, 1) after the solver is built.

Mounting (finger frame of fr3_leftfinger / fr3_rightfinger, see
``geometry``): the stock finger's shapes are switched off, the carriage body
carries the printed adapter (boxes) and the finger's 10 mm base block, and
the linkage hangs below it.  The contact face is the stock pad plane, so the
finger coordinate q keeps its meaning (face gap = 2 q).

Body labels start with ``fr3/`` so every robot-vs-object probe in the bench
counts the new bodies as robot.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import warp as wp

import newton

from force_bench.finray import geometry as G
from force_bench.finray import prb
from force_bench.finray.frame_fe import E_TPU95A

FINGER_LINKS = ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger")
FINRAY_COLOR = (0.96, 0.52, 0.10)        # orange TPU
ADAPTER_COLOR = (0.22, 0.23, 0.26)       # dark PETG
PETG_DENSITY = 1270.0


@dataclass
class FinRayParams:
    model: str = "prb"                   # "prb" | "rigid"
    E_mpa: float = E_TPU95A              # ESTIMATE, data sheet TPU 95A
    k_scale: float = prb.K_SCALE
    c_rib: float = prb.C_RIB
    rib_stride: int = 1
    zeta: float = 0.5                    # joint damping ratio (choice)
    omega_dt: float = 0.35               # armature rule, see module doc
    eq_tc_dt: float = 2.0                # weld solref timeconst = this * dt
    substeps: int = 32                   # the rig's (the armature rule depends on the substep)
    fps: int = 60
    mu: float = 1.0                      # TPU on the bench's parts (an estimate)
    gap: float = 0.0004                  # m, contact gap of the finger shapes
    density: float = G.TPU_DENSITY
    keep_carriage_mass: bool = True      # on the FR3: the carriage keeps the scene's finger mass and inertia (its
                                         # drive's conditioning); False: the adapter's and base block's own

    @property
    def sim_dt(self) -> float:
        return 1.0 / self.fps / self.substeps


@dataclass
class FinRayFinger:
    side: str
    carriage: int                        # body index of the FR3 finger link
    bodies: dict[str, int] = field(default_factory=dict)      # link name -> body
    contact_shapes: list[int] = field(default_factory=list)   # contact-wall shapes
    shapes: list[int] = field(default_factory=list)           # every finger shape
    tree_joints: list[int] = field(default_factory=list)
    loop_joints: list[int] = field(default_factory=list)
    links: dict = field(default_factory=dict)                 # name -> prb.Link

    @property
    def all_bodies(self) -> list[int]:
        return [self.carriage] + list(self.bodies.values())


# ── geometry helpers (finger frame, metres) ──────────────────────────────
def _yz(p_step_xz) -> np.ndarray:
    """A profile point (STEP x, z mm) -> finger-frame (y, z) m."""
    return np.array([p_step_xz[0], G.Z_BASE - p_step_xz[1]]) / 1000.0


def _box_about_x(p0, p1):
    """Centre and rotation (about the finger x axis) of a box whose local z
    runs from p0 to p1 (finger-frame (y, z) points)."""
    d = p1 - p0
    a = float(np.arctan2(-d[0], d[1]))
    return 0.5 * (p0 + p1), wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), a)


ADAPTER_BOXES_MM = [
    # (x half, y0, y1, z0, z1)  finger frame, mm -- see adapter_finray_fr3.py
    (13.0, G.ADAPTER_INNER_Y, G.TAB_INNER_Y + G.TAB_THICK + G.POCKET_CLEAR + G.OUTER_CHEEK,
     G.ADAPTER_TOP_Z, G.POCKET_BOTTOM_Z),                         # clamp around the tab
    (G.WIDTH / 2 + G.CHEEK_CLEAR + G.CHEEK, G.ADAPTER_INNER_Y, G.DEPTH,
     G.POCKET_BOTTOM_Z, G.Z_BASE),                                 # bridge
]
CHEEK_BOXES_MM = [
    (s, G.ADAPTER_INNER_Y, G.DEPTH, G.Z_BASE, G.Z_BASE + G.BASE_H) for s in (-1, 1)
]


def _shape_cfg(params: FinRayParams, density: float, collide: bool = True,
               mu: float | None = None) -> newton.ModelBuilder.ShapeConfig:
    return newton.ModelBuilder.ShapeConfig(
        mu=params.mu if mu is None else mu, density=density, ke=1.0e5, kd=1.0e3,
        gap=params.gap, mu_torsional=0.005, mu_rolling=0.0,
        has_shape_collision=collide, has_particle_collision=False)


def _add_box(builder, body, center_mm_or_m, half_m, q, cfg, label, color):
    return builder.add_shape_box(body, xform=wp.transform(wp.vec3(*center_mm_or_m), q),
                                 hx=half_m[0], hy=half_m[1], hz=half_m[2], cfg=cfg,
                                 label=label, color=color)


def add_adapter_and_base(builder, body: int, params: FinRayParams, prefix: str,
                         base_block: bool = True) -> list[int]:
    """The printed adapter (and the finger's base block) on a carriage body.
    Shapes carry no density: the caller sets the body's mass properties."""
    sids = []
    cfg = _shape_cfg(params, 0.0, mu=0.5)
    for i, (xh, y0, y1, z0, z1) in enumerate(ADAPTER_BOXES_MM):
        c = np.array([0.0, 0.5 * (y0 + y1), 0.5 * (z0 + z1)]) / 1000.0
        h = np.array([xh, 0.5 * (y1 - y0), 0.5 * (z1 - z0)]) / 1000.0
        sids.append(_add_box(builder, body, c, h, wp.quat_identity(), cfg,
                             f"{prefix}_adapter{i}", ADAPTER_COLOR))
    for i, (s, y0, y1, z0, z1) in enumerate(CHEEK_BOXES_MM):
        xc = s * (G.WIDTH / 2 + G.CHEEK_CLEAR + G.CHEEK / 2)
        c = np.array([xc, 0.5 * (y0 + y1), 0.5 * (z0 + z1)]) / 1000.0
        h = np.array([G.CHEEK / 2, 0.5 * (y1 - y0), 0.5 * (z1 - z0)]) / 1000.0
        sids.append(_add_box(builder, body, c, h, wp.quat_identity(), cfg,
                             f"{prefix}_cheek{i}", ADAPTER_COLOR))
    if base_block:
        cfg_t = _shape_cfg(params, 0.0)
        c = np.array([0.0, G.DEPTH / 2, G.Z_BASE + G.BASE_H / 2]) / 1000.0
        h = np.array([G.WIDTH / 2, G.DEPTH / 2, G.BASE_H / 2]) / 1000.0
        sids.append(_add_box(builder, body, c, h, wp.quat_identity(), cfg_t,
                             f"{prefix}_base", FINRAY_COLOR))
    return sids


def _rigid_mass_props(params: FinRayParams, with_finger: bool):
    """(mass, com, inertia) of adapter + base block (+ the whole rigid finger),
    finger frame, from boxes / the STL."""
    parts = []
    for xh, y0, y1, z0, z1 in ADAPTER_BOXES_MM:
        parts.append((PETG_DENSITY, 2 * xh, y1 - y0, z1 - z0,
                      np.array([0.0, 0.5 * (y0 + y1), 0.5 * (z0 + z1)])))
    for s, y0, y1, z0, z1 in CHEEK_BOXES_MM:
        parts.append((PETG_DENSITY, G.CHEEK, y1 - y0, z1 - z0,
                      np.array([s * (G.WIDTH / 2 + G.CHEEK_CLEAR + G.CHEEK / 2), 0.5 * (y0 + y1),
                                0.5 * (z0 + z1)])))
    if not with_finger:
        parts.append((params.density, G.WIDTH, G.DEPTH, G.BASE_H,
                      np.array([0.0, G.DEPTH / 2, G.Z_BASE + G.BASE_H / 2])))
    m_tot, c_acc, I_tot = 0.0, np.zeros(3), np.zeros((3, 3))
    items = []
    for rho, a, b, c, ctr in parts:
        m = rho * a * b * c * 1e-9
        I = m / 12.0 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b]) * 1e-6
        items.append((m, ctr / 1000.0, I))
    if with_finger:
        import trimesh
        v, f = G.finger_mesh_vertices_m()
        tm = trimesh.Trimesh(v, f, process=False)
        m = params.density * tm.volume
        items.append((m, np.asarray(tm.center_mass), np.asarray(tm.moment_inertia) * params.density))
    for m, c, I in items:
        m_tot += m
        c_acc += m * c
    com = c_acc / m_tot
    for m, c, I in items:
        d = c - com
        I_tot += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m_tot, com, I_tot


def _set_mass(builder, body, m, com, I):
    builder.body_mass[body] = float(m)
    builder.body_inv_mass[body] = 1.0 / float(m)
    builder.body_com[body] = wp.vec3(*com)
    builder.body_inertia[body] = wp.mat33(*np.asarray(I).ravel())
    builder.body_inv_inertia[body] = wp.mat33(*np.linalg.inv(I).ravel())


def _body_world(builder, body) -> wp.transform:
    t = builder.body_q[body]
    return wp.transform(wp.vec3(t[0], t[1], t[2]), wp.quat(t[3], t[4], t[5], t[6]))


def add_rigid_finger(builder, body: int, params: FinRayParams, prefix: str) -> list[int]:
    """The whole finger as one SDF mesh on the carriage body."""
    from force_bench.assets import make_newton_mesh
    import trimesh
    v, f = G.finger_mesh_vertices_m()
    tm = trimesh.Trimesh(v, f, process=False)
    mesh, c = make_newton_mesh(tm, sdf=True, sdf_resolution=256, narrow_band=0.002,
                               margin=0.0005)
    sid = builder.add_shape_mesh(body, mesh=mesh, xform=wp.transform(wp.vec3(*c), wp.quat_identity()),
                                 cfg=_shape_cfg(params, 0.0), label=f"{prefix}_finray",
                                 color=FINRAY_COLOR)
    return [sid]


def add_prb_finger(builder, carriage: int, params: FinRayParams, prefix: str,
                   side: str) -> FinRayFinger:
    """The PRB linkage hung below the base block of a carriage body.  Adds
    TREE joints only (the caller assigns them to the carriage's
    articulation); the loop welds are returned for ``add_loops``."""
    L = prb.layout(E=params.E_mpa, k_scale=params.k_scale, c_rib=params.c_rib,
                   rib_stride=params.rib_stride)
    fin = FinRayFinger(side=side, carriage=carriage, links=L.by_name())
    X_c = _body_world(builder, carriage)
    dt = params.sim_dt
    mass_scale = (G.VOLUME_CM3 * 1e-6 * params.density
                  - params.density * G.WIDTH * G.DEPTH * G.BASE_H * 1e-9) / sum(prb.link_masses(L, params.density).values())
    for l in L.links:
        p0, p1 = _yz(l.p0), _yz(l.p1)
        origin = np.array([0.0, p0[0], p0[1]])          # body frame: at p0, finger-aligned
        X = wp.transform_multiply(X_c, wp.transform(wp.vec3(*origin), wp.quat_identity()))
        b = builder.add_link(xform=X, label=f"{prefix}_{l.name}")
        fin.bodies[l.name] = b
        ctr, q = _box_about_x(np.zeros(2), p1 - p0)
        collide = l.kind != "rib"
        m = prb.link_masses(L, params.density)[l.name] * mass_scale
        vol = (l.t * l.length * G.WIDTH) * 1e-9
        cfg = _shape_cfg(params, m / vol, collide=collide)
        sid = _add_box(builder, b, (0.0, ctr[0], ctr[1]),
                       (G.WIDTH / 2000.0, l.t / 2000.0, l.length / 2000.0), q, cfg,
                       f"{prefix}_{l.name}_shape", FINRAY_COLOR)
        fin.shapes.append(sid)
        if l.kind == "contact":
            fin.contact_shapes.append(sid)
    by = L.by_name()
    for l in L.links:
        b = fin.bodies[l.name]
        p0 = _yz(l.p0)
        if l.parent is None:
            parent, rel = carriage, np.array([0.0, p0[0], p0[1]])
        else:
            pl = by[l.parent]
            parent, rel = fin.bodies[l.parent], np.r_[0.0, p0 - _yz(pl.p0)]
        a = l.k * (dt / params.omega_dt) ** 2
        kd = 2.0 * params.zeta * np.sqrt(l.k * a)
        j = builder.add_joint_revolute(
            parent=parent, child=b, axis=(1.0, 0.0, 0.0),
            parent_xform=wp.transform(wp.vec3(*rel), wp.quat_identity()),
            child_xform=wp.transform_identity(),
            target_pos=0.0, target_ke=float(l.k), target_kd=float(kd), armature=float(a),
            label=f"{prefix}_{l.name}_j")
        fin.tree_joints.append(j)
    fin._loops = L.loops
    return fin


def add_loops(builder, fin: FinRayFinger, prefix: str) -> None:
    """Weld the rib halves and the tip (loop-closing FIXED joints)."""
    for a, b, pt in fin._loops:
        pa = _yz(fin.links[a].p0)
        pb = _yz(fin.links[b].p0)
        ptm = _yz(pt)
        j = builder.add_joint_fixed(
            parent=fin.bodies[a], child=fin.bodies[b],
            parent_xform=wp.transform(wp.vec3(0.0, *(ptm - pa)), wp.quat_identity()),
            child_xform=wp.transform(wp.vec3(0.0, *(ptm - pb)), wp.quat_identity()),
            label=f"{prefix}_{a}_{b}_weld", collision_filter_parent=True)
        builder.joint_articulation[j] = -1
        fin.loop_joints.append(j)


def filter_finger(builder, fin: FinRayFinger, other_robot_bodies: list[int]) -> None:
    """No contact inside one finger, and none against the arm and hand."""
    own = set()
    for b in fin.all_bodies:
        own.update(builder.body_shapes.get(b, []))
    own = sorted(own)
    for i, a in enumerate(own):
        for c in own[i + 1:]:
            builder.add_shape_collision_filter_pair(a, c)
    for b in other_robot_bodies:
        for s in builder.body_shapes.get(b, []):
            for a in own:
                builder.add_shape_collision_filter_pair(s, a)


def mount_on_fr3(builder, params: FinRayParams | None = None) -> list[FinRayFinger]:
    """Replace both FR3 fingers.  Call RIGHT AFTER ``add_franka`` (before
    any other body or joint): the linkage's tree joints must extend the
    robot's articulation contiguously."""
    params = params or FinRayParams()
    labels = list(builder.body_label)
    fingers = []
    art = builder.joint_articulation[builder.joint_count - 1]
    assert art >= 0 and builder.articulation_end[art] == builder.joint_count, \
        "mount_on_fr3 must run right after add_franka"
    for side, lbl in zip("LR", FINGER_LINKS):
        body = labels.index(lbl)
        for sid in list(builder.body_shapes.get(body, [])):
            builder.shape_flags[sid] = 0               # stock finger: gone
        prefix = f"fr3/finray_{side}"
        add_adapter_and_base(builder, body, params, prefix, base_block=(params.model == "prb"))
        if params.model == "rigid":
            add_rigid_finger(builder, body, params, prefix)
            if not params.keep_carriage_mass:
                _set_mass(builder, body, *_rigid_mass_props(params, with_finger=True))
            fin = FinRayFinger(side=side, carriage=body)
            fin.shapes = list(builder.body_shapes.get(body, []))
            fin.contact_shapes = [fin.shapes[-1]]
        else:
            if not params.keep_carriage_mass:
                _set_mass(builder, body, *_rigid_mass_props(params, with_finger=False))
            fin = add_prb_finger(builder, body, params, prefix, side)
            fin.shapes = sorted(set(builder.body_shapes.get(body, [])) | set(fin.shapes))
        fingers.append(fin)
    if params.model == "prb":
        n = builder.joint_count
        for j in range(builder.articulation_end[art], n):
            builder.joint_articulation[j] = art
        builder.articulation_end[art] = n
        for fin in fingers:
            add_loops(builder, fin, f"fr3/finray_{fin.side}")
    robot = [i for i, l in enumerate(builder.body_label)
             if str(l).startswith("fr3/") and "finray" not in str(l) and i not in
             (fingers[0].carriage, fingers[1].carriage)]
    for fin in fingers:
        filter_finger(builder, fin, robot)
    builder._finray = (params, fingers)
    return fingers


def tune_solver(solver, params: FinRayParams | None = None) -> int:
    """Stiffen the Fin Ray loop welds: MuJoCo's default equality solref is
    (0.02 s, 1), a 20 ms spring far softer than the walls.  Call after EVERY
    solver build (``make_solver`` makes a fresh MuJoCo model).  Returns the
    number of weld rows set."""
    import mujoco
    params = params or FinRayParams()
    mjm = getattr(solver, "mj_model", None)
    mjw = getattr(solver, "mjw_model", None)
    if mjm is None or mjw is None or mjm.neq == 0:
        return 0
    rows = []
    for e in range(mjm.neq):
        if mjm.eq_type[e] != mujoco.mjtEq.mjEQ_WELD:
            continue
        b1 = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, int(mjm.eq_obj1id[e])) or ""
        b2 = mujoco.mj_id2name(mjm, mujoco.mjtObj.mjOBJ_BODY, int(mjm.eq_obj2id[e])) or ""
        if "finray" in b1 or "finray" in b2:
            rows.append(e)
    if not rows:
        return 0
    tc = (params.eq_tc_dt * params.sim_dt, 1.0)
    sr = mjw.eq_solref.numpy()
    if sr.ndim == 3:
        sr[:, rows] = tc
    else:
        sr[rows] = tc
    mjw.eq_solref.assign(sr)
    mjm.eq_solref[rows] = tc
    return len(rows)
