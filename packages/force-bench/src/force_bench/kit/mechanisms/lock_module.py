"""Lock module: printed housing, lock cylinder (plug + pinion), deadbolt and friction spring, optionally with key 1.

Model (measured on the final CAD, ``assets/kit/lock-module``; millimetres in the module's CAD frame: the plug axis is
+z through the origin, the deadbolt points +x, the housing bottom is at z = 0):

* the plug and its pinion are one printed part (``gear_<n>_deg``): ONE link, revolute about +z.  Turning it CLOCKWISE
  seen from above (negative about +z) retracts the deadbolt.  The keyway is broached down the plug from its top face
  (z 41.5) to a floor at z 16.0: 25.5 mm deep, 3.0 x 9.5 mm, paracentric;
* the deadbolt is a frame around the pinion with a rack on its inner -y edge; it slides along -x (retract) in the
  bottom housing's cavity: a prismatic link.  Pinion -> rack is the stiff joint equality of ``_door_parts.gear``
  (``deadbolt_q = r * (-plug_q)``);
* there is no return spring.  The printed friction spring (a serpentine flexure under the deadbolt, drawn) is dry
  friction on the deadbolt's slide (MuJoCo ``frictionloss``), so the bolt stays where the key leaves it;
* stops: the deadbolt frame meets the cavity's -x wall after ``TRAVEL_MM`` = 25.01 mm (frame -22.49 vs wall -47.50)
  and its shoulders sit 0.33 mm from the +x stop at rest; both are plug joint limits.

Variants (``Amount of rotation to unlock``), every rack at the module-1 pitch pi mm (tooth centres fitted on the
CAD, residual <= 0.002 mm):

  label   pinion                                   r [mm]   stop (25.01 mm)   clears the door's jamb (24.49 mm)
  90°     32 teeth, module 1                       16.000   89.6 deg           87.7 deg
  180°    16 teeth, module 1                        8.000   179.1 deg          175.4 deg
  270°    10 teeth spaced 33.75 deg (one gap)       5.333   268.7 deg          263.1 deg
  360°     8 teeth, module 1                        4.000   358.2 deg          350.8 deg

  r is the rack travel per radian: pitch / tooth spacing (2 pi r / teeth for a full gear; the 270 deg pinion has
  module-1 teeth cut for a 12-tooth gear but spaced at 33.75 deg, i.e. 16/3 mm per radian).  Every pinion is sized
  for 8 pi = 25.13 mm of bolt travel at its nominal angle; the cavity stops the bolt at 25.01 mm, flush with the door
  leaf's edge.  The build checks each pinion's tip radius against the file.

Friction spring: ``FRICTION_N`` = 1.25 N of sliding friction on the deadbolt.  NOT MEASURED: an estimate, 0.02 N.m
of cylinder drag at the 90 deg pinion's 16 mm radius; the printed flexure stands 0.97 mm into the deadbolt's path in
the CAD.

Key 1 (``build(..., key_mode="seated")``): the kit's ``key-1`` in the keyway.  Its blade section was
fitted into the keyway polygon (2-D containment along the blade, unique orientation; 0.25 mm per side across the
2.5 mm blade, +-0.24 mm along the 9.5 mm one): key x -> module -z (tip down), key y -> -y, key z -> -x, centred at
x = 1.25 mm.  Seated, the tip stands on the keyway floor (z 16.0) with the bow's shoulder 0.76 mm above the plug face.
``"seated"`` welds it into the plug link: the key turns the plug.  Inserting the key is not modelled: the
0.01-0.13 mm keyway clearances are far below contact resolution (physics_notes pitfall 3.3).

Probe (``force-bench kit check lock-module``): a probe motor turns the plug at 60 deg/s from rest 3 deg past the stop
and back; the check measures the bolt travel against the plug angle (the ratio), the stop, the angle at which the
bolt has retracted ``JAMB_CLEAR_MM`` and the friction torque (half the gap between the two strokes' torques).
"""

from __future__ import annotations

import numpy as np

from force_bench.assets import set_body_mass
from force_bench.kit.build import part_mass
from force_bench.kit.mechanisms._door_parts import (MM, SPINDLE_ARMATURE, SpinDriver, SubstepTargets, add_link,
                                                     fit_line, gear, joint, picked, place_parts, ramp)

GEARS = {        # unlock-angle label -> (teeth, rack travel per radian [mm], nominal angle [deg], tip radius [mm])
    "90°": (32, 16.0, 90.0, 17.0),
    "180°": (16, 8.0, 180.0, 9.0),
    "270°": (10, 16.0 / 3.0, 270.0, 7.0),
    "360°": (8, 4.0, 360.0, 5.0),
}
TRAVEL_MM = 25.01                 # rest -> deadbolt frame against the cavity's -x wall
EXTEND_SLACK_MM = 0.33            # rest -> shoulders against the +x stop
JAMB_CLEAR_MM = 24.49             # in the door: retraction at which the tip passes the strike jamb face
FRICTION_N = 1.25                 # friction spring (estimated, see the docstring)
BOLT_SLACK_MM = 0.5               # the deadbolt joint's own limits sit outside the plug's (see doorknob_module)
BOLT_MU = 0.4
KEY_KEY = "key-1"
KEY_SEAT_MM = 25.5                # keyway depth: plug face z 41.5 -> floor z 16.0
KEY_TIP_X_MM = 38.66              # key-1 tip, key frame
PLUG_FACE_Z_MM = 41.5
KEY_R = np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]])    # key frame -> module frame
KEY_X_MM = 1.25                   # blade centred in the 3.0 mm keyway
HOLD_KE, HOLD_KD = 0.5, 0.01      # N.m/rad, N.m.s/rad: the plug hold of ``hold_retracted``


def variant_params(choice: dict) -> dict:
    unlock = choice.get("Amount of rotation to unlock") or "90°"
    teeth, r_mm, nominal, tip = GEARS[unlock]
    r = r_mm * MM
    return {"unlock_label": unlock, "pinion_teeth": teeth, "travel_per_rad_mm": round(r_mm, 4),
            "nominal_unlock_deg": nominal, "tip_radius_mm": tip,
            "stop_deg": round(float(np.degrees(TRAVEL_MM * MM / r)), 2),
            "jamb_clear_deg": round(float(np.degrees(JAMB_CLEAR_MM * MM / r)), 2),
            "deadbolt_travel_mm": TRAVEL_MM, "friction_N": FRICTION_N,
            "source": {"pinion": "CAD: teeth / spacing vs the rack's pi mm pitch, tip radius",
                       "travel": "CAD: deadbolt frame vs cavity walls",
                       "friction": "estimate: 0.02 N.m of cylinder drag at the 16 mm pinion (not measured)"}}


def key_pose_cad(depth_m: float) -> np.ndarray:
    """Key 1 in the lock module's CAD frame at insertion ``depth_m`` (0: tip at the plug face; seated: 25.5 mm)."""
    M = np.eye(4)
    M[:3, :3] = KEY_R
    M[:3, 3] = np.array([KEY_X_MM, 0.0, PLUG_FACE_Z_MM + KEY_TIP_X_MM]) * MM - np.array([0.0, 0.0, depth_m])
    return M


def _check_pinion(part, tip_mm):
    tm = part.mesh()
    s = tm.section(plane_origin=[0.0, 0.0, 0.0115], plane_normal=[0.0, 0.0, 1.0])
    r_tip = max(float(np.hypot(e[:, 0], e[:, 1]).max()) for e in s.discrete) / MM
    assert abs(r_tip - tip_mm) < 0.05, (part.name, r_tip, tip_mm)
    return r_tip


def build(rig, asset, h, parent: int = -1, key_mode: str | None = None,
          friction_N: float = FRICTION_N, armature: float = SPINDLE_ARMATURE, bolt_mu: float = BOLT_MU,
          key_mu: float = 0.6, hold_retracted: bool = False) -> None:
    """``hold_retracted``: the module as scenery, deadbolt withdrawn to its stop (the plug starts turned to its limit
    and a light hold keeps it there), e.g. for the doorknob tasks, where the latch alone gates the door."""
    from force_bench.kit import load_kit
    key = key_mode
    prm = variant_params(h.choice)
    prm.update(friction_N=float(friction_N), spindle_armature=armature)
    if friction_N != FRICTION_N:
        prm["source"]["friction"] = "override"
    r = prm["travel_per_rad_mm"] * MM
    parts = asset.parts(picked(h))
    housing = [p for p in parts if p.name in ("lock_bottom", "lock_top")]
    fspring = next(p for p in parts if p.name == "lock_friction_spring")
    plug_p = next(p for p in parts if p.name.startswith("gear_"))
    bolt_p = next(p for p in parts if p.name.startswith("deadbolt_"))
    _check_pinion(plug_p, prm["tip_radius_mm"])
    pose = np.asarray(h.pose, dtype=float)
    L = h.label

    h.shapes.update(place_parts(rig, parent, asset, housing, pose, L))
    h.shapes.update(place_parts(rig, parent, asset, [fspring], pose, L, collide=False))
    plug = add_link(rig, pose, f"{L}/plug")
    h.shapes.update(place_parts(rig, plug, asset, [plug_p], pose, L, collide=False))
    bolt = add_link(rig, pose, f"{L}/deadbolt")
    h.shapes.update(place_parts(rig, bolt, asset, [bolt_p], pose, L, mu=bolt_mu))

    stop = TRAVEL_MM * MM / r
    hold = bool(hold_retracted)
    j_p = joint(rig, "revolute", parent, plug, pose, (0.0, 0.0, 1.0), f"{L}/plug",
                target_pos=-stop if hold else 0.0, target_ke=HOLD_KE if hold else 0.0,
                target_kd=HOLD_KD if hold else 0.0, armature=armature,
                limit_lower=-stop, limit_upper=EXTEND_SLACK_MM * MM / r)
    j_b = joint(rig, "prismatic", parent, bolt, pose, (-1.0, 0.0, 0.0), f"{L}/deadbolt", hard_friction=True,
                target_ke=0.0, target_kd=0.0, friction=float(friction_N),
                limit_lower=-(EXTEND_SLACK_MM + BOLT_SLACK_MM) * MM, limit_upper=(TRAVEL_MM + BOLT_SLACK_MM) * MM)
    gear(rig, j_b, j_p, -r, f"{L}/gear")
    joints = [j_p, j_b]
    if hold:
        rig.builder.joint_q[int(rig.builder.joint_q_start[j_p])] = -stop
        rig.builder.joint_q[int(rig.builder.joint_q_start[j_b])] = stop * r
        prm.update(held_retracted=True, hold_ke=HOLD_KE)

    key_sids = []
    if key is not None:
        kit = load_kit()
        kp = kit[KEY_KEY].parts()[0]
        if key != "seated":
            raise ValueError(f"lock-module: key_mode must be None or 'seated', not {key!r}")
        world_key = pose @ key_pose_cad(KEY_SEAT_MM * MM)
        key_sids = place_parts(rig, plug, kit[KEY_KEY], [kp], world_key, L, mu=key_mu)[kp.name]
        set_body_mass(rig.builder, plug, part_mass(asset, plug_p) + part_mass(kit[KEY_KEY], kp))
        h.shapes["key"] = list(key_sids)
        prm.update(key=key, key_seat_mm=KEY_SEAT_MM)
    else:
        set_body_mass(rig.builder, plug, part_mass(asset, plug_p))
    set_body_mass(rig.builder, bolt, part_mass(asset, bolt_p))
    if parent < 0:
        rig.builder.add_articulation([j_p])
        rig.builder.add_articulation([j_b])

    housing_sids = [s for p in housing for s in h.shapes[p.name]]
    plug_sids = list(h.shapes[plug_p.name])
    bolt_sids = list(h.shapes[bolt_p.name])
    for a in plug_sids + bolt_sids + list(key_sids):
        for b in housing_sids:
            rig.builder.add_shape_collision_filter_pair(a, b)
    for a in plug_sids + list(key_sids):
        for b in bolt_sids:
            rig.builder.add_shape_collision_filter_pair(a, b)
    for a in key_sids:
        for b in plug_sids:
            rig.builder.add_shape_collision_filter_pair(a, b)

    h.bodies.update({"plug": plug, "deadbolt": bolt})
    h.joints.update({"plug": j_p, "deadbolt": j_b})
    h.params = prm
    tip_rest = np.array([72.25, 0.0, 0.0]) * MM
    h.info.update({
        "parent": parent, "joints_in_order": joints,
        "plug_axis_world": (pose[:3, :3] @ np.array([0.0, 0.0, 1.0])).tolist(), "plug_point_world": pose[:3, 3].tolist(),
        "retract_dir_world": (pose[:3, :3] @ np.array([-1.0, 0.0, 0.0])).tolist(),
        "deadbolt_tip_rest_world": (pose @ np.append(tip_rest, 1.0))[:3].tolist(),
        "stop_rad": float(stop), "travel_per_rad_m": float(r), "plug_part": plug_p.name, "deadbolt_part": bolt_p.name,
        "housing_shapes": housing_sids, "plug_shapes": plug_sids, "deadbolt_shapes": bolt_sids,
        "key_shapes": list(key_sids),
    })


# ── probe: turn the plug like a robot turning the key ──────────────────────────────────────────────────────────────
SPEED = np.deg2rad(60.0)
OVERDRIVE = np.deg2rad(3.0)


def add_probe(rig, h) -> None:
    h._probe_motor = SpinDriver(rig, np.asarray(h.pose, dtype=float), h.joints["plug"], f"{h.label}/probe_motor")
    h.bodies["probe_motor"] = h._probe_motor.body
    h.info["probe_bodies"] = []


def run_probe(rig, h) -> dict:
    from force_bench.rig import make_solver
    drv = h._probe_motor.bind()
    m = rig.model
    pq = int(m.joint_q_start.numpy()[h.joints["plug"]])
    bq = int(m.joint_q_start.numpy()[h.joints["deadbolt"]])
    stop = h.info["stop_rad"]
    r = h.info["travel_per_rad_m"]
    a_end = -(stop + OVERDRIVE)
    t0, t1 = 0.3, 0.3 + abs(a_end) / SPEED
    t2, t3 = t1 + 0.4, t1 + 0.4 + abs(a_end) / SPEED

    def target(t):
        return ramp(t, t0, t1, 0.0, a_end) if t < t2 else ramp(t, t2, t3, a_end, 0.0)

    saved, cadence = rig.solver, rig.collide_every_substep
    rig.solver = SubstepTargets(make_solver(m, iterations=24), lambda t: {drv.tgt: target(t)})
    rig.collide_every_substep = False
    rows = []

    def extra(state, t):
        q = state.joint_q.numpy()
        rows.append((t, float(q[pq]), float(q[bq]), drv.torque(state, target(t))))
        return {}

    rig.run(t3 + 0.4, lambda t: None, extra, print_every=10 ** 9)
    rig.solver, rig.collide_every_substep = saved, cadence
    a = np.array(rows)
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "probe_plug.csv", np.column_stack([a[:, 0], np.degrees(-a[:, 1]), a[:, 2] * 1e3, -a[:, 3]]),
               delimiter=",", header="t,plug_deg,deadbolt_mm,torque_Nm", comments="")
    th, x, tau = -a[:, 1], a[:, 2], -a[:, 3]
    opening = (a[:, 0] > t0 + 0.15) & (a[:, 0] < t1)
    closing = (a[:, 0] > t2 + 0.15) & (a[:, 0] < t3)
    inner = (th > np.deg2rad(5.0)) & (th < stop - np.deg2rad(5.0))
    _, ratio, ratio_rms = fit_line(th[opening & inner], x[opening & inner])
    t_open = float(np.median(tau[opening & inner]))
    t_close = float(np.median(tau[closing & inner]))
    fric = 0.5 * (t_open - t_close)
    th_stop = float(th[opening | ((a[:, 0] >= t1) & (a[:, 0] < t2))].max())
    x_max = float(x.max())
    clear = np.where(x >= JAMB_CLEAR_MM * MM)[0]
    clear_deg = float(np.degrees(th[clear[0]])) if len(clear) else float("nan")
    expect_fric = h.params["friction_N"] * r
    checks = {
        "gear_ratio": abs(ratio - r) / r < 0.02,
        "stop": abs(th_stop - stop) < np.deg2rad(1.5),
        "deadbolt_clears_jamb": x_max >= JAMB_CLEAR_MM * MM,
        "friction_torque": abs(fric - expect_fric) < 0.3 * expect_fric + 0.002,
        "returns_to_rest": abs(float(np.degrees(th[-1]))) < 1.0 and abs(float(x[-1])) < 0.3 * MM,
        "finite": bool(np.isfinite(a).all()),
    }
    return {"pass": bool(all(checks.values())), "checks": {k: bool(v) for k, v in checks.items()},
            "deadbolt_mm_per_rad": round(ratio * 1e3, 4), "travel_per_rad_mm": round(r * 1e3, 4),
            "ratio_fit_rms_mm": round(ratio_rms * 1e3, 4),
            "stop_deg": round(float(np.degrees(th_stop)), 2), "stop_cad_deg": round(float(np.degrees(stop)), 2),
            "deadbolt_travel_max_mm": round(x_max * 1e3, 3), "deadbolt_clears_jamb_at_deg": round(clear_deg, 2),
            "nominal_unlock_deg": h.params["nominal_unlock_deg"],
            "friction_torque_Nm": round(fric, 5), "expected_friction_torque_Nm": round(expect_fric, 5),
            "torque_opening_Nm": round(t_open, 5), "torque_closing_Nm": round(t_close, 5),
            "final_plug_deg": round(float(np.degrees(th[-1])), 3), "final_deadbolt_mm": round(float(x[-1]) * 1e3, 3)}
