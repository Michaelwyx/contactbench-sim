"""Door: the frame, the leaf on its hinge, and the doorknob and lock modules, assembled from the final CAD and lying
flat on the table like the real fixture.

Door CAD frame (``assets/kit/door``, millimetres): the door lies flat, the frame's bottom at z = 0, z up.  The leaf
(228 x 224 x 14.5 mm) nests in the frame at z 17.5..32; its hinge knuckles at x +-(72.25..95.75) interleave with the
frame's at the y = +127 edge on a 3.2 mm pin whose axis is x through (y 122.0, z 22.47) (hole centres fitted on both
parts, +-0.05 mm).  The strike jamb (``door_frame_1``) at y -127..-97.5 holds both strike pockets; its face is at
y -97.51, 0.51 mm beyond the leaf's edge (y -97.0).  Opening LIFTS the strike edge about the horizontal hinge
(negative rotation about +x, i.e. about +y in a task's world).

Module placement: registered against the site's assembled view (``door.obj``, centimetres), every module part to
< 0.1 mm rms:

  doorknob module   translate (38.85, -49.75, 2.5) mm, no rotation (spindle axis +z, latch toward -y)
  lock module       translate (-60.65, -49.75, 7.5) mm, rotated -90 deg about z (plug +z, deadbolt toward -y)

(door.obj shows the latch 1.0 mm retracted: its rest against the cavity wall, ``doorknob_module.LATCH_REST_MM``).

Strike pockets (free volumes of ``door_frame_1``, mm):

  latch      x 27.86..49.84   y -114.0..-97.5   z 8.00..24.00   bolt 20.3 wide, top face at z 24.0 at rest
  deadbolt   x -76.64..-44.66 y -123.0..-97.5   z 12.76..24.74  bolt 29.4 x 9.7 (z 14.0..23.7)

The pocket walls that block the bolts are analytic boxes grown outward from those volumes (mouth open; box-vs-bolt
mesh contact is exact, an SDF of the 250 mm rail is 0.6 mm per cell); the bolts do not collide with the frame mesh.
Two departures, both documented in ``info["deviations"]``: the latch pocket's roof is 0.25 mm above the CAD's (the
resting latch's top face is flush with it), and the 30 deg entry ramp above the latch pocket's mouth (y -97.5..-100.4,
z 24..29, which cams a closing latch in) is not modelled: the lip is a vertical face in the jamb plane, so a lifted
leaf is blocked until the latch has retracted past the jamb (``doorknob_module.JAMB_CLEAR_MM``).

Engagement at rest (bolt tip beyond the jamb face): latch 15.24 mm (of 15.44 mm of travel), deadbolt 24.49 mm (of
25.01 mm).  Contact policy: the leaf nests in the frame and is filtered against it (the hinge is the
joint; its closed stop is the joint limit at 0, the CAD has no ledge under the strike edge), the bolts touch only the
pocket walls, the modules' moving parts are filtered against the leaf.  Masses: the kit's print estimates, the module
housings and springs added to the leaf.

``build(rig, asset, h, modules={"doorknob-module": {...} | "label", "lock-module": "180°"}, key_mode=None|"seated",
hinge_kd=0.15, knob=dict(...), lock=dict(...))``: ``knob`` / ``lock`` are passed on to the module
builders (``K``, ``tau0``, ``friction_N`` ...).  ``h._modules`` holds the two module handles (their bodies and joints
are also in ``h.bodies`` / ``h.joints`` under their roles: leaf, knob, latch, plug, deadbolt; joint hinge).

Probe (``force-bench kit check door``): an opening torque of 1.5 N.m (about three times the leaf's weight moment) is
applied at the hinge while both bolts are thrown, again after a probe motor has turned the knob to its stop (the
deadbolt alone must hold), then after a second one has turned the plug to its stop (the leaf must lift); the leaf is
let fall shut, the bolts are thrown again and the same torque must not move it.
"""

from __future__ import annotations

import numpy as np

from force_bench.assets import set_body_mass
from force_bench.kit.build import part_mass
from force_bench.kit.mechanisms._door_parts import (MM, SpinDriver, SubstepTargets, T, add_link, box_shape, joint,
                                                     picked, place_parts, ramp)

HINGE_YZ_MM = (122.0, 22.47)
JAMB_FACE_Y_MM = -97.51
KNOB_MODULE_T = T(np.array([38.85, -49.75, 2.5]) * MM)
LOCK_MODULE_T = T(np.array([-60.65, -49.75, 7.5]) * MM,
                  R=np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]))      # -90 deg about z
POCKETS_MM = {   # name -> (lo, hi) free volume, door CAD frame
    "latch": (np.array([27.86, -114.0, 8.00]), np.array([49.84, -97.5, 24.00])),
    "deadbolt": (np.array([-76.64, -123.0, 12.76]), np.array([-44.66, -97.5, 24.74])),
}
LATCH_ROOF_RAISE_MM = 0.25
POCKET_WALL_MM = 6.0
POCKET_MU = 0.4
HINGE_KD = 0.15                 # N.m.s/rad: light viscous hinge
HINGE_OPEN_LIMIT_DEG = 100.0
PANEL_MM = (np.array([-113.5, -96.5, 17.5]), np.array([113.5, 116.5, 32.0]))   # leaf slab, for fingertips
DEFAULT_MODULES = {"doorknob-module": {"Doorknob geometry": "Doorknob 1, 55mm diameter", "Rotation force": "5mm spring",
                                       "Amount of rotation to unlock": "60°"},
                   "lock-module": {"Amount of rotation to unlock": "90°"}}
DEVIATIONS = [
    "strike pocket walls are analytic boxes on the frame CAD's pocket volumes; the bolts collide only with them",
    "latch pocket roof 0.25 mm above the CAD's (the resting latch's top face is flush with the CAD roof)",
    "the latch pocket's 30 deg entry ramp is not modelled (vertical lip in the jamb plane): the leaf is held until "
    "the latch has retracted past the jamb face, 15.24 mm",
    "the hinge pin is the revolute joint and the closed stop its limit at 0; the leaf does not collide with the frame",
]


def _module_choice(asset_key, spec):
    from force_bench.kit import load_kit
    if spec is None:
        spec = DEFAULT_MODULES[asset_key]
    if isinstance(spec, str):
        spec = {"": spec}
    a = load_kit()[asset_key]
    return {k: v.label for k, v in a.resolve(spec).items() if v is not None}


def pocket_walls(name: str, wall: float = POCKET_WALL_MM):
    """[(centre, half extents)] in the door CAD frame (mm) of the walls around pocket ``name`` (mouth +y open)."""
    lo, hi = (np.array(v, dtype=float) for v in POCKETS_MM[name])
    if name == "latch":
        hi = hi + np.array([0.0, 0.0, LATCH_ROOF_RAISE_MM])
    c, e = 0.5 * (lo + hi), hi - lo
    w = wall / 2.0
    return [((c[0], c[1], hi[2] + w), (e[0] / 2 + wall, e[1] / 2, w)),            # roof (blocks a lifted leaf)
            ((c[0], c[1], lo[2] - w), (e[0] / 2 + wall, e[1] / 2, w)),            # floor
            ((c[0], lo[1] - w, c[2]), (e[0] / 2 + wall, w, e[2] / 2 + wall)),     # back wall
            ((lo[0] - w, c[1], c[2]), (w, e[1] / 2, e[2] / 2)),                   # side walls
            ((hi[0] + w, c[1], c[2]), (w, e[1] / 2, e[2] / 2))]


def build(rig, asset, h, modules: dict | None = None, key_mode: str | None = None,
          hinge_kd: float = HINGE_KD, knob: dict | None = None, lock: dict | None = None) -> None:
    from force_bench.kit import load_kit
    kit = load_kit()
    pose = np.asarray(h.pose, dtype=float)
    L = h.label
    modules = dict(modules or {})
    parts = {p.name: p for p in asset.parts()}

    # frame: static rails
    frame_sids = place_parts(rig, -1, asset, [parts["door_frame_1"], parts["door_frame_2"]], pose, L)
    h.shapes.update(frame_sids)
    frame_sids = [s for v in frame_sids.values() for s in v]

    # leaf on the hinge
    leaf = add_link(rig, pose, f"{L}/leaf")
    leaf_sids = place_parts(rig, leaf, asset, [parts["door"]], pose, L)["door"]
    h.shapes["door"] = list(leaf_sids)
    lo, hi = PANEL_MM
    panel = box_shape(rig, leaf, pose @ T(0.5 * (lo + hi) * MM), 0.5 * (hi - lo) * MM, f"{L}/panel_face", mu=0.5)
    leaf_sids.append(panel)
    h.shapes["panel_face"] = [panel]
    hinge_anchor = pose @ T(np.array([0.0, *HINGE_YZ_MM]) * MM)
    j_h = joint(rig, "revolute", -1, leaf, hinge_anchor, (1.0, 0.0, 0.0), f"{L}/hinge",
                target_pos=0.0, target_ke=0.0, target_kd=hinge_kd,
                limit_lower=-np.deg2rad(HINGE_OPEN_LIMIT_DEG), limit_upper=0.0)

    # the two modules, mounted on the leaf
    from force_bench.kit.build import add_asset
    hk = add_asset(rig, "doorknob-module", _module_choice("doorknob-module", modules.get("doorknob-module")),
                   pose=pose @ KNOB_MODULE_T, parent=leaf, **(knob or {}))
    hl = add_asset(rig, "lock-module", _module_choice("lock-module", modules.get("lock-module")),
                   pose=pose @ LOCK_MODULE_T, parent=leaf, key_mode=key_mode, **(lock or {}))
    rig.builder.add_articulation([j_h, *[hk.joints[r] for r in ("knob", "latch")], *hl.info["joints_in_order"]])

    # leaf mass: the panel, the module housings and springs that ride on it (print estimates)
    static_parts = [(asset, parts["door"])]
    km, lm = kit["doorknob-module"], kit["lock-module"]
    static_parts += [(km, p) for p in km.parts(picked(hk)) if p.name.startswith(("doorknob_module_bottom",
                                                                                     "doorknob_module_top",
                                                                                     "doorknob_module_spring"))]
    static_parts += [(lm, p) for p in lm.parts(picked(hl)) if p.name in ("lock_bottom", "lock_top",
                                                                           "lock_friction_spring")]
    leaf_mass = sum(part_mass(a, p) for a, p in static_parts)
    set_body_mass(rig.builder, leaf, leaf_mass)

    # strike pockets: static walls, touching only the bolts
    pocket_sids = {}
    for name in POCKETS_MM:
        sids = []
        for i, (c, half) in enumerate(pocket_walls(name)):
            sids.append(box_shape(rig, -1, pose @ T(np.asarray(c) * MM), np.asarray(half) * MM,
                                  f"{L}/strike_{name}_{i}", mu=POCKET_MU))
        pocket_sids[name] = sids
    h.shapes.update({f"strike_{n}": s for n, s in pocket_sids.items()})
    all_pockets = [s for v in pocket_sids.values() for s in v]

    # contact policy
    flt = rig.builder.add_shape_collision_filter_pair
    knob_sids = hk.info["knob_shapes"]
    latch_sids = hk.info["latch_shapes"]
    plug_sids = hl.info["plug_shapes"] + hl.info["key_shapes"]
    bolt_sids = hl.info["deadbolt_shapes"]
    housing_sids = hk.info["housing_shapes"] + hl.info["housing_shapes"]
    for a in leaf_sids + housing_sids:
        for b in frame_sids + all_pockets:
            flt(a, b)
    for a in knob_sids + latch_sids + plug_sids + bolt_sids:
        for b in leaf_sids + frame_sids:
            flt(a, b)
    for a in knob_sids + plug_sids:
        for b in all_pockets:
            flt(a, b)
    for a in latch_sids:
        for b in pocket_sids["deadbolt"]:
            flt(a, b)
    for a in bolt_sids:
        for b in pocket_sids["latch"]:
            flt(a, b)
    for a in knob_sids + latch_sids:
        for b in plug_sids + bolt_sids + hl.info["housing_shapes"]:
            flt(a, b)
    for a in plug_sids + bolt_sids:
        for b in hk.info["housing_shapes"]:
            flt(a, b)

    h._modules = {"doorknob-module": hk, "lock-module": hl}      # handles (not JSON: kept off info)
    h.bodies.update({"leaf": leaf, **{f"{r}": b for r, b in hk.bodies.items()}, **{f"{r}": b for r, b in hl.bodies.items()}})
    h.joints.update({"hinge": j_h, **hk.joints, **hl.joints})
    h.params = {"doorknob_module": dict(hk.params), "lock_module": dict(hl.params), "hinge_kd": hinge_kd,
                "leaf_mass_kg": round(leaf_mass, 4)}
    R = pose[:3, :3]
    lat_lo, lat_hi = POCKETS_MM["latch"]
    h.info.update({
        "module_labels": {"doorknob-module": hk.label, "lock-module": hl.label},
        "hinge_point_world": hinge_anchor[:3, 3].tolist(), "hinge_axis_world": (R @ np.array([1.0, 0.0, 0.0])).tolist(),
        "jamb_face_y_cad_mm": JAMB_FACE_Y_MM,
        "latch_engagement_mm": round(_engagement(pose, hk.info["latch_tip_rest_world"]), 3),
        "deadbolt_engagement_mm": round(_engagement(pose, hl.info["deadbolt_tip_rest_world"]), 3),
        "latch_pocket_cad_mm": [lat_lo.tolist(), lat_hi.tolist()],
        "pocket_shapes": pocket_sids, "leaf_shapes": leaf_sids, "frame_shapes": frame_sids,
        "deviations": DEVIATIONS,
    })


def _engagement(pose, tip_world) -> float:
    """Depth [mm] of a bolt tip beyond the jamb face, measured along the door's -y."""
    tip_cad = np.linalg.inv(pose) @ np.append(np.asarray(tip_world, dtype=float), 1.0)
    return float(JAMB_FACE_Y_MM - tip_cad[1] / MM)


# ── probe: locked, latch only, unlocked, relocked ──────────────────────────────────────────────────────────────────
PROBE_NM = 1.5
LOCK_LIMIT_DEG = 0.6
OPEN_MIN_DEG = 10.0
OPEN_STOP_DEG = 15.0
KNOB_SPEED = np.deg2rad(60.0)
PLUG_SPEED = np.deg2rad(120.0)


def add_probe(rig, h) -> None:
    hk, hl = h._modules["doorknob-module"], h._modules["lock-module"]
    h._motors = (SpinDriver(rig, np.asarray(hk.pose), h.joints["knob"], f"{h.label}/knob_motor"),
                 SpinDriver(rig, np.asarray(hl.pose), h.joints["plug"], f"{h.label}/plug_motor"))
    h.bodies["knob_motor"] = h._motors[0].body
    h.bodies["plug_motor"] = h._motors[1].body
    h.info["probe_bodies"] = [f"{hk.label}/latch", f"{hl.label}/deadbolt"]


def run_probe(rig, h) -> dict:
    from force_bench.kit.mechanisms._door_parts import sim_dt
    from force_bench.rig import make_solver
    km, pm = (d.bind() for d in h._motors)
    hk, hl = h._modules["doorknob-module"], h._modules["lock-module"]
    m = rig.model
    qs = m.joint_q_start.numpy()
    hq, kq, lq, pq, bq = (int(qs[h.joints[r]]) for r in ("hinge", "knob", "latch", "plug", "deadbolt"))
    hdof = int(m.joint_qd_start.numpy()[h.joints["hinge"]])
    k_end = -(hk.info["stop_rad"] + np.deg2rad(2.0))
    p_end = -(hl.info["stop_rad"] + np.deg2rad(2.0))
    # schedule [s]
    t_lock = (0.3, 0.8)
    t_k = (0.9, 0.9 + abs(k_end) / KNOB_SPEED)
    t_dead = (t_k[1] + 0.2, t_k[1] + 0.7)
    t_p = (t_dead[1] + 0.1, t_dead[1] + 0.1 + abs(p_end) / PLUG_SPEED)
    t_open = (t_p[1] + 0.2, t_p[1] + 1.4)
    t_shut = (t_open[1], t_open[1] + 1.2)
    t_back = (t_shut[1], t_shut[1] + max(abs(k_end) / KNOB_SPEED, abs(p_end) / PLUG_SPEED))
    t_relock = (t_back[1] + 0.3, t_back[1] + 0.8)
    t_end = t_relock[1] + 0.2

    def k_tgt(t):
        return ramp(t, *t_k, 0.0, k_end) if t < t_back[0] else ramp(t, *t_back, k_end, 0.0)

    def p_tgt(t):
        return ramp(t, *t_p, 0.0, p_end) if t < t_back[0] else ramp(t, *t_back, p_end, 0.0)

    saved, cadence = rig.solver, rig.collide_every_substep
    rig.solver = SubstepTargets(make_solver(m, iterations=24), lambda t: {km.tgt: k_tgt(t), pm.tgt: p_tgt(t)})
    base_f = rig.control.joint_f.numpy().copy()
    st = {"open_on": True, "peak": 0.0}
    rows = []
    li = rig.labels.index(f"{h.label}/leaf")
    axis = np.asarray(h.info["hinge_axis_world"])

    def leaf_deg(state):
        q = state.body_q.numpy()[li][3:7]                   # the leaf's twist about the hinge (x, y, z, w)
        return float(np.degrees(-2.0 * np.arctan2(float(q[:3] @ axis), float(q[3]))))

    def control(t):
        rig.collide_every_substep = t < 0.25
        th = leaf_deg(rig.s0)
        tau = 0.0
        if any(a <= t < b for a, b in (t_lock, t_dead, t_relock)):
            tau = -PROBE_NM
        if t_open[0] <= t < t_open[1] and st["open_on"]:
            if th >= OPEN_STOP_DEG:
                st["open_on"] = False
            else:
                tau = -PROBE_NM
        f = base_f.copy()
        f[hdof] += tau
        rig.control.joint_f.assign(f)

    def extra(state, t):
        q = state.joint_q.numpy()
        rows.append((t, leaf_deg(state), float(q[kq]), float(q[lq]), float(q[pq]), float(q[bq])))
        return {}

    rig.run(t_end, control, extra, print_every=10 ** 9)
    rig.solver, rig.collide_every_substep = saved, cadence
    rig.control.joint_f.assign(base_f)
    a = np.array(rows)
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "probe_door.csv", np.column_stack([a[:, 0], a[:, 1], np.degrees(-a[:, 2]), a[:, 3] * 1e3,
                                                        np.degrees(-a[:, 4]), a[:, 5] * 1e3]),
               delimiter=",", header="t,leaf_deg,knob_deg,latch_mm,plug_deg,deadbolt_mm", comments="")
    t = a[:, 0]

    def peak(win):
        msk = (t >= win[0]) & (t < win[1] + 0.1)
        return float(np.abs(a[msk, 1]).max())

    opened = float(a[(t >= t_open[0]) & (t < t_open[1]), 1].max())
    shut = float(np.abs(a[(t >= t_shut[1] - 0.3) & (t < t_shut[1]), 1]).max())
    probes = {
        "locked_both_bolts_deg": round(peak(t_lock), 3),
        "locked_deadbolt_only_deg": round(peak(t_dead), 3),
        "opened_unlocked_deg": round(opened, 2),
        "shut_after_release_deg": round(shut, 3),
        "relocked_deg": round(peak(t_relock), 3),
        "latch_retracted_at_open_mm": round(float(a[(t >= t_open[0]) & (t < t_open[1]), 3].min()) * 1e3, 3),
        "deadbolt_retracted_at_open_mm": round(float(a[(t >= t_open[0]) & (t < t_open[1]), 5].min()) * 1e3, 3),
        "final_latch_mm": round(float(a[-1, 3]) * 1e3, 3), "final_deadbolt_mm": round(float(a[-1, 5]) * 1e3, 3),
        "probe_torque_Nm": PROBE_NM, "lock_limit_deg": LOCK_LIMIT_DEG, "schedule_s": {
            "lock": t_lock, "knob": t_k, "deadbolt_only": t_dead, "plug": t_p, "open": t_open, "relock": t_relock},
        "dt": sim_dt(rig),
    }
    checks = {
        "locked_both_bolts": probes["locked_both_bolts_deg"] < LOCK_LIMIT_DEG,
        "locked_by_deadbolt_alone": probes["locked_deadbolt_only_deg"] < LOCK_LIMIT_DEG,
        "opens_when_unlocked": opened >= OPEN_MIN_DEG,
        "falls_shut": shut < 1.0,
        "relocked": probes["relocked_deg"] < LOCK_LIMIT_DEG,
        "finite": bool(np.isfinite(a).all()),
    }
    probes["checks"] = {k: bool(v) for k, v in checks.items()}
    probes["pass"] = bool(all(checks.values()))
    return probes
