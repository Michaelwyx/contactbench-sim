"""Doorknob module: printed housing, spiral return spring, pinion and latch, with a doorknob or a door handle.

Model (measured on the final CAD, ``assets/kit/doorknob-module``; millimetres in the module's CAD frame: the spindle
is +z through the origin, the latch bolt points -y, the module lies flat with its housing bottom at z = 0):

* the grip (knob or handle), the pinion and the spring's inner hub are pinned to one spindle: ONE link, revolute
  about +z.  Turning it CLOCKWISE seen from above (negative about +z) retracts the latch;
* the latch is a frame around the pinion with a rack on its inner -x edge; it slides along +y (retract) in the bottom
  housing's cavity and its bolt passes out through the housing's -y wall.  It is a prismatic link;
* pinion -> rack is a stiff joint equality ``latch_q = r_p * (-knob_q)`` (``_door_parts.gear``): bilateral like the
  printed spur pair, so a latch that is pushed in turns the knob, and the bolt's load is felt at the grip;
* the printed spiral spring (inner hub on the spindle, outer arm anchored in the housing wall) is a revolute spring on
  the spindle: tau = tau0 + K * theta, K as the drive stiffness (target 0) and tau0 as a constant joint torque
  (``builder.joint_f``, i.e. ``Control.joint_f``), never as a target past the stop (that flings the lever,
  examples/force_response).  The spring part is drawn, not simulated;
* the printed stops are the latch frame meeting the cavity walls; they are the knob joint's limits (rest 0, full turn
  ``-travel / r_p``), so the spring preload is reacted by a joint limit and never by the gear equality;
* the latch rests on the cavity floor (0.5 mm below its CAD pose: the cavity is 16.0 mm tall for a 15.5 mm latch) and
  against the cavity's -y wall (the CAD frame overlaps that wall by 0.88 mm; the site's assembled door, door.obj,
  shows it 1.0 mm back), so it sits ``LATCH_REST`` from its CAD pose.  From there it can retract
  ``LATCH_TRAVEL_MM`` = 15.44 mm before its frame meets the +y wall (cavity y -37.25..41.75, frame -38.13..25.43).

Variants and their parameters:

  Amount of rotation to unlock   pinion (module 1.25 mm)        r_p      stop (15.44 mm)   clears the door's jamb
  40 deg                         7-tooth sector of a 36-tooth   22.50    39.3 deg           38.8 deg
  48 deg                         7-tooth sector of a 30-tooth   18.75    47.2 deg           46.6 deg
  60 deg                         24 teeth                       15.00    59.0 deg           58.2 deg
  72 deg                         20 teeth                       12.50    70.8 deg           69.9 deg
  80 deg                         18 teeth                       11.25    78.6 deg           77.6 deg

  r_p from the tooth count (or the sector's tooth spacing, 10 / 12 deg) and the gear's tip radius r_p + 1.25 mm (the
  build checks the tip radius against the file).  Every pinion is sized so r_p x (nominal angle) = 5 pi = 15.71 mm of
  latch travel; the printed cavity stops the latch at 15.44 mm, and in the door the bolt clears the strike jamb after
  15.24 mm (``JAMB_CLEAR_MM``, door.py), so on the CAD the latch unlocks ~1.2-2.4 deg before the nominal angle and
  the knob stops 0.7-1.4 deg short of it.  The racks are cut at 3.900 mm pitch against the pinions' 3.927 mm circular
  pitch (0.7 %, inside the backlash over the <= 4 teeth a stroke uses); the ratio is the pinion's r_p.

  Rotation force (printed spring)   K [N.m/rad]   tau0 [N.m]   source
  5 mm spring                       0.48          0.15         fitted to measured lever push force
  7.5 mm spring                     0.67          0.25         (UR arm, wrist F/T, lever pushed 85 mm from the
  10 mm spring                      0.78          0.31          spindle at 10 mm/s; examples/force_response)

  The springs act on the spindle, so the fits hold for every grip and every pinion (they were taken with the
  default handle and the 60 deg pinion).  Damping ``SPRING_KD`` (~0.3 of critical on the spindle's inertia) is chosen
  for stability, not measured.

  Grip (handle 1 / 2, doorknob 1 / 2 at 50 / 55 / 60 mm): the printed part is drawn; the fingers touch an invisible
  massless convex hull of its graspable portion (doorknob 1: the cone and cylinder above its collar, z >= 35 mm;
  doorknob 2: the bulb above its neck, z >= 41 mm, an oblate spheroid a = d/2, c = 21.25 mm; handles: the bar and the
  boss as two hulls).  Hull-vs-fingertip contact is exact and stiff; the printed knob's own mesh does not collide.

Masses are the kit's print estimates (``force_bench.kit.build.part_mass``).  ``build(..., K=, tau0=)`` overrides the
spring, ``parent=`` mounts the module on a body (the door leaf) instead of the world.

Probe (``force-bench kit check doorknob-module``): a probe motor turns the spindle at 45 deg/s from rest 3 deg past the
stop and back, like a robot with a wrist F/T sensor; the check measures the latch travel against the knob angle (the
gear ratio), the stop, the angle at which the latch has retracted ``JAMB_CLEAR_MM``, and the spindle torque against
angle (the spring's K and tau0: line fits on the opening and the closing stroke, averaged so that the
velocity-dependent terms cancel).
"""

from __future__ import annotations

import numpy as np

from force_bench.assets import set_body_mass
from force_bench.kit.build import part_mass
from force_bench.kit.mechanisms._door_parts import (MM, SPINDLE_ARMATURE, SpinDriver, SubstepTargets, T,
                                                     add_joint_force, add_link, fit_line, gear, hull_shape,
                                                     joint, picked, place_parts, ramp)

GEARS = {        # unlock-angle label -> (teeth on the full circle, pitch radius [mm], nominal unlock angle [deg])
    "40°": (36, 22.50, 40.0),
    "48°": (30, 18.75, 48.0),
    "60°": (24, 15.00, 60.0),
    "72°": (20, 12.50, 72.0),
    "80°": (18, 11.25, 80.0),
}
GEAR_MODULE_MM = 1.25
SPRINGS = {      # rotation-force label -> (K [N.m/rad], tau0 [N.m]), measured (examples/force_response)
    "5mm spring": (0.48, 0.15),
    "7.5mm spring": (0.67, 0.25),
    "10mm spring": (0.78, 0.31),
}
SPRING_KD = 0.008                      # N.m.s/rad on the spindle (stability, not measured)
LATCH_REST_MM = (0.0, 0.88, -0.5)      # latch rest offset from its CAD pose (see the docstring)
LATCH_TRAVEL_MM = 15.44                # rest -> frame against the cavity's +y wall
LATCH_PROTRUSION_MM = 63.88 - 0.88 - 47.25   # 15.75: bolt tip beyond the housing's -y face at rest
JAMB_CLEAR_MM = 15.24                  # in the door: retraction at which the tip passes the strike jamb face
LATCH_SLACK_MM = 0.5                   # the latch joint's own limits sit this far outside the knob's (the knob's
# limits are the stops; overlapping limits on two coupled dofs would fight through the gear equality)
GRIP_MU, LATCH_MU = 0.6, 0.4
HOLD_KE = 5.0                          # N.m/rad: the spindle hold of ``hold_retracted`` (replaces the spring)
GRIPS = {        # part-name prefix -> graspable portion (hulls, CAD mm): list of vertex filters
    "doorknob_1": [lambda v: v[:, 2] >= 35.0],
    "doorknob_2": [lambda v: v[:, 2] >= 41.0],
    "door_handle_1": [lambda v: (v[:, 1] >= 10.0) & (v[:, 2] >= 67.4), lambda v: (v[:, 1] <= 12.6) & (v[:, 2] >= 28.0)],
    "door_handle_2": [lambda v: (v[:, 1] >= 10.0) & (v[:, 2] >= 67.9), lambda v: (v[:, 1] <= 10.1) & (v[:, 2] >= 28.0)],
}


def variant_params(choice: dict) -> dict:
    """The physical parameters of a variant choice (labels as in catalog.json), with provenance."""
    unlock = choice.get("Amount of rotation to unlock") or "40°"
    spring = choice.get("Rotation force") or "5mm spring"
    teeth, rp_mm, nominal = GEARS[unlock]
    K, tau0 = SPRINGS[spring]
    rp = rp_mm * MM
    return {"unlock_label": unlock, "pinion_teeth": teeth, "pitch_radius_mm": rp_mm, "nominal_unlock_deg": nominal,
            "stop_deg": round(float(np.degrees(LATCH_TRAVEL_MM * MM / rp)), 2),
            "jamb_clear_deg": round(float(np.degrees(JAMB_CLEAR_MM * MM / rp)), 2),
            "latch_travel_mm": LATCH_TRAVEL_MM, "spring_label": spring, "K_Nm_per_rad": K, "tau0_Nm": tau0,
            "source": {"pinion": "CAD: tooth count/spacing and tip radius r_p + 1.25 mm",
                       "travel": "CAD: latch frame vs cavity walls",
                       "spring": "fit to measured lever push force (examples/force_response)"}}


def _part(parts, prefix):
    hits = [p for p in parts if p.name.startswith(prefix)]
    return hits[0] if hits else None


def _check_pinion(gear_part, rp_mm):
    """The table's pitch radius against the file: tip radius = r_p + one module."""
    tm = gear_part.mesh()
    s = tm.section(plane_origin=[0.0, 0.0, tm.bounds.mean(0)[2]], plane_normal=[0.0, 0.0, 1.0])
    r_tip = max(float(np.hypot(e[:, 0], e[:, 1]).max()) for e in s.discrete) / MM
    assert abs(r_tip - (rp_mm + GEAR_MODULE_MM)) < 0.05, (gear_part.name, r_tip, rp_mm)
    return r_tip


def build(rig, asset, h, parent: int = -1, K: float | None = None, tau0: float | None = None,
          kd: float = SPRING_KD, armature: float = SPINDLE_ARMATURE, grip_mu: float = GRIP_MU,
          latch_mu: float = LATCH_MU, hold_retracted: bool = False) -> None:
    """``hold_retracted``: the module as scenery with its latch held back at the stop (the spindle starts turned to
    its limit and a stiff hold replaces the spring), e.g. for the key tasks, where the deadbolt alone gates the
    door."""
    prm = variant_params(h.choice)
    if K is not None or tau0 is not None:
        prm["source"]["spring"] = "override"
    K = prm["K_Nm_per_rad"] if K is None else float(K)
    tau0 = prm["tau0_Nm"] if tau0 is None else float(tau0)
    prm.update(K_Nm_per_rad=K, tau0_Nm=tau0, spring_kd=kd, spindle_armature=armature)
    rp = prm["pitch_radius_mm"] * MM
    parts = asset.parts(picked(h))
    housing = [p for p in parts if p.name in ("doorknob_module_bottom", "doorknob_module_top")]
    spring_p = _part(parts, "doorknob_module_spring")
    gear_p = _part(parts, "doorknob_module_gear")
    latch_p = _part(parts, "doorknob_module_latch")
    grip_p = next(p for p in parts if p.name.startswith(("doorknob_1", "doorknob_2", "door_handle")))
    prm["r_tip_mm"] = round(_check_pinion(gear_p, prm["pitch_radius_mm"]), 3)
    pose = np.asarray(h.pose, dtype=float)
    L = h.label

    # housing (+ the printed spiral spring, drawn) on the parent: static when lying alone on the table
    h.shapes.update(place_parts(rig, parent, asset, housing, pose, L))
    h.shapes.update(place_parts(rig, parent, asset, [spring_p], pose, L, collide=False))

    # spindle link: grip + pinion (drawn) + invisible hull(s) for the fingers
    knob = add_link(rig, pose, f"{L}/knob")
    h.shapes.update(place_parts(rig, knob, asset, [grip_p, gear_p], pose, L, collide=False))
    v = grip_p.mesh().vertices
    key = next(k for k in GRIPS if grip_p.name.startswith(k))
    grip_sids = [hull_shape(rig, knob, v[f(v / MM)], pose, f"kit/{L}/grip_hull_{i}", grip_mu)
                 for i, f in enumerate(GRIPS[key])]
    h.shapes["grip_hull"] = grip_sids
    set_body_mass(rig.builder, knob, part_mass(asset, grip_p) + part_mass(asset, gear_p))

    # latch: its own link, at its rest offset (body frame = module frame)
    rest = T(np.asarray(LATCH_REST_MM) * MM)
    latch = add_link(rig, pose, f"{L}/latch")
    h.shapes.update(place_parts(rig, latch, asset, [latch_p], pose, L, mu=latch_mu, offset=rest))
    set_body_mass(rig.builder, latch, part_mass(asset, latch_p))

    stop = LATCH_TRAVEL_MM * MM / rp
    hold = bool(hold_retracted)
    j_k = joint(rig, "revolute", parent, knob, pose, (0.0, 0.0, 1.0), f"{L}/knob",
                target_pos=-stop if hold else 0.0, target_ke=HOLD_KE if hold else K, target_kd=kd, armature=armature,
                limit_lower=-stop, limit_upper=0.0)
    j_l = joint(rig, "prismatic", parent, latch, pose, (0.0, 1.0, 0.0), f"{L}/latch",
                target_ke=0.0, target_kd=0.0, limit_lower=-LATCH_SLACK_MM * MM,
                limit_upper=(LATCH_TRAVEL_MM + LATCH_SLACK_MM) * MM)
    gear(rig, j_l, j_k, -rp, f"{L}/gear")
    if hold:
        rig.builder.joint_q[int(rig.builder.joint_q_start[j_k])] = -stop
        rig.builder.joint_q[int(rig.builder.joint_q_start[j_l])] = stop * rp
        prm.update(held_retracted=True, hold_ke=HOLD_KE)
    else:
        add_joint_force(rig, j_k, tau0)                   # preload, toward the rest stop (+z)
    if parent < 0:
        rig.builder.add_articulation([j_k])
        rig.builder.add_articulation([j_l])

    # contact policy: the spindle and the latch move inside their own housing (sub-mm printed clearances, and the
    # CAD latch overlaps its cavity wall): filtered against it and against each other
    housing_sids = [s for p in housing for s in h.shapes[p.name]]
    knob_sids = [s for n in (grip_p.name, gear_p.name) for s in h.shapes[n]] + grip_sids
    latch_sids = list(h.shapes[latch_p.name])
    for a in knob_sids + latch_sids:
        for b in housing_sids:
            rig.builder.add_shape_collision_filter_pair(a, b)
    for a in knob_sids:
        for b in latch_sids:
            rig.builder.add_shape_collision_filter_pair(a, b)

    h.bodies.update({"knob": knob, "latch": latch})
    h.joints.update({"knob": j_k, "latch": j_l})
    h.params = prm
    tip_rest = np.array([0.0, -63.88, 0.0]) * MM + np.asarray(LATCH_REST_MM) * MM
    h.info.update({
        "parent": parent, "spindle_axis_world": (pose[:3, :3] @ np.array([0.0, 0.0, 1.0])).tolist(),
        "spindle_point_world": pose[:3, 3].tolist(),
        "retract_dir_world": (pose[:3, :3] @ np.array([0.0, 1.0, 0.0])).tolist(),
        "latch_tip_rest_world": (pose @ np.append(tip_rest, 1.0))[:3].tolist(),
        "latch_rest_cad_mm": list(LATCH_REST_MM), "latch_protrusion_mm": round(LATCH_PROTRUSION_MM, 2),
        "stop_rad": float(stop), "pitch_radius_m": float(rp), "grip_part": grip_p.name, "latch_part": latch_p.name,
        "grip_hull_shapes": grip_sids, "housing_shapes": housing_sids, "knob_shapes": knob_sids,
        "latch_shapes": latch_sids,
    })


# ── probe: turn the spindle like a robot with a wrist F/T sensor ───────────────────────────────────────────────────
SPEED = np.deg2rad(45.0)
OVERDRIVE = np.deg2rad(3.0)


def add_probe(rig, h) -> None:
    pose = np.asarray(h.pose, dtype=float)
    h._probe_motor = SpinDriver(rig, pose, h.joints["knob"], f"{h.label}/probe_motor")    # not JSON: off info
    h.bodies["probe_motor"] = h._probe_motor.body
    h.info["probe_bodies"] = []


def run_probe(rig, h) -> dict:
    from force_bench.rig import make_solver
    drv = h._probe_motor.bind()
    m = rig.model
    kq = int(m.joint_q_start.numpy()[h.joints["knob"]])
    lq = int(m.joint_q_start.numpy()[h.joints["latch"]])
    stop = h.info["stop_rad"]
    rp = h.info["pitch_radius_m"]
    a_end = -(stop + OVERDRIVE)
    t0, t1 = 0.3, 0.3 + abs(a_end) / SPEED
    t2, t3 = t1 + 0.4, t1 + 0.4 + abs(a_end) / SPEED

    def target(t):
        return ramp(t, t0, t1, 0.0, a_end) if t < t2 else ramp(t, t2, t3, a_end, 0.0)

    saved, cadence = rig.solver, rig.collide_every_substep
    rig.solver = SubstepTargets(make_solver(m, iterations=24), lambda t: {drv.tgt: target(t)})
    rig.collide_every_substep = False        # nothing lands or breaks contact here
    rows = []

    def extra(state, t):
        q = state.joint_q.numpy()
        rows.append((t, float(q[kq]), float(q[lq]), drv.torque(state, target(t))))
        return {}

    rig.run(t3 + 0.4, lambda t: None, extra, print_every=10 ** 9)
    rig.solver, rig.collide_every_substep = saved, cadence
    a = np.array(rows)
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "probe_spindle.csv", np.column_stack([a[:, 0], np.degrees(-a[:, 1]), a[:, 2] * 1e3, -a[:, 3]]),
               delimiter=",", header="t,knob_deg,latch_mm,torque_Nm", comments="")
    th, x, tau = -a[:, 1], a[:, 2], -a[:, 3]      # opening angle (rad), latch travel (m), torque to hold it (N.m)
    opening = (a[:, 0] > t0 + 0.1) & (a[:, 0] < t1)
    closing = (a[:, 0] > t2 + 0.1) & (a[:, 0] < t3)
    inner = (th > np.deg2rad(3.0)) & (th < stop - np.deg2rad(3.0))
    _, ratio, ratio_rms = fit_line(th[opening & inner], x[opening & inner])
    c_open = fit_line(th[opening & inner], tau[opening & inner])
    c_close = fit_line(th[closing & inner], tau[closing & inner])
    tau0_fit, K_fit = 0.5 * (c_open[0] + c_close[0]), 0.5 * (c_open[1] + c_close[1])   # velocity terms cancel
    th_stop = float(th[opening | ((a[:, 0] >= t1) & (a[:, 0] < t2))].max())
    x_max = float(x.max())
    clear = np.where(x >= JAMB_CLEAR_MM * MM)[0]
    clear_deg = float(np.degrees(th[clear[0]])) if len(clear) else float("nan")
    back = float(np.degrees(th[-1]))
    prm = h.params
    checks = {
        "gear_ratio": abs(ratio - rp) / rp < 0.02,
        "stop": abs(th_stop - stop) < np.deg2rad(1.5),
        "latch_clears_jamb": x_max >= JAMB_CLEAR_MM * MM,
        "spring_K": abs(K_fit - prm["K_Nm_per_rad"]) < 0.15 * prm["K_Nm_per_rad"],
        "spring_tau0": abs(tau0_fit - prm["tau0_Nm"]) < 0.03,
        "returns_to_rest": abs(back) < 1.0 and abs(float(x[-1])) < 0.3 * MM,
        "finite": bool(np.isfinite(a).all()),
    }
    return {"pass": bool(all(checks.values())), "checks": {k: bool(v) for k, v in checks.items()},
            "latch_mm_per_rad": round(ratio * 1e3, 4), "pitch_radius_mm": round(rp * 1e3, 3),
            "ratio_fit_rms_mm": round(ratio_rms * 1e3, 4),
            "stop_deg": round(float(np.degrees(th_stop)), 2), "stop_cad_deg": round(float(np.degrees(stop)), 2),
            "latch_travel_max_mm": round(x_max * 1e3, 3), "latch_clears_jamb_at_deg": round(clear_deg, 2),
            "nominal_unlock_deg": prm["nominal_unlock_deg"],
            "K_fit_Nm_per_rad": round(K_fit, 4), "tau0_fit_Nm": round(tau0_fit, 4),
            "opening_fit_tau0_K": [round(c_open[0], 4), round(c_open[1], 4)],
            "closing_fit_tau0_K": [round(c_close[0], 4), round(c_close[1], 4)],
            "torque_at_stop_Nm": round(float(np.interp(stop - np.deg2rad(1.0), th[opening], tau[opening])), 4),
            "final_knob_deg": round(back, 3), "final_latch_mm": round(float(x[-1]) * 1e3, 3)}
