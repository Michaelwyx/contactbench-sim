"""Appendix task 24 "Stack cups".

    Description: Multiple cups are scattered around the workspace.  Collect the cups and place them inside one
    another to form a tight stack.
    Success: The robot movement stops after all cups form a tight, upright stack (not knocked over).  The robot
    should not remain in contact with the task objects.

NEW TASK on the finalized kit (``force_bench.kit``): three kit cups (68.6 mm rim, 40 mm base, 65 mm tall, 1 mm wall,
15 g each, printed PLA) stand upright on the bench.  The first is the base of the stack; the robot puts the second
into it and the third into the second.

PHYSICS.  Each cup is a free body colliding through a fine signed-distance field of its thin wall (0.2 mm cells,
0.15 mm margin and contact gap: nested walls are 1 mm thick and the stack is held by wall-on-wall contact).  Dropped
coaxially, a cup slides into the one below until the walls meet; measured without a robot, three cups settle at a
4.68 mm step, centred to 0.02 mm, at rest (``STACK_STEP``).

EXPERT: an INSIDE grasp.  The hand comes down over the cup with the fingers closed, puts the fingertips into the
mouth, and opens them until their outer faces press on the inner wall about 12 mm below the rim.  The fingers stay
inside the cup they hold, so nothing comes between the two cups while it is lowered into the stack (an outside
grasp would have to let go above the stack: only the top 4.7 mm of a nested cup's wall stays exposed).  Per cup:

  approach  over the cup, fingers closed            insert    fingertips into the mouth
  grip      fingers open onto the inner wall        lift      to clear the stack's rim
  carry     over the stack axis (measured)          lower     until the cup's base is just above its nested
                                                              height or the cups touch (contact probe)
  release   fingers close off the wall: the cup     rise      fingertips out of the cup
            settles into the stack under its own weight

then a still hold.  Positions are privileged (measured cup poses); the grip is open-loop on the finger targets,
loaded by the finger actuators' force limit.

SEEDED START (--seed N): the three cups are scattered as the paper's protocol scatters objects: each cup's centre
uniform in the 0.15 x 0.15 m square centred on the nominal layout (``SQUARE_C``), the three drawn jointly so that no
two rims come closer than 6.4 mm (``common.sample_start``; cups have no yaw).  The first cup drawn stays the stack's
base.  The hand starts at the nominal pose for every seed, and the camera does not move (the whole square stays in
its view).  The draw is recorded in the verdict ("start").

VERDICT (appendix_qa.json):
  cups_stacked        every cup in one stack: axes within 2 mm of the bottom cup's, base heights at
                      k x STACK_STEP within 1.5 mm, tilt < 3 deg
  stack_not_knocked   the bottom cup moved < 10 mm and stands upright (tilt < 3 deg)
  real_inside_grasp   each carried cup was held by finger contact (> 0.5 N mean) through its carry
  released_and_clear  robot <-> cup contact < 0.05 N over the final second, hand raised >= 50 mm
  robot_stopped       TCP speed < 1 mm/s at the end
plus the physics-integrity checks (finite, velocity spikes, penetration, workspace envelope).

OPTIONS  --seed N (the cups' positions, see SEEDED START), --name (output folder).

Run: force-bench run stack_cups [--seed N]
"""

from __future__ import annotations

import argparse

import numpy as np
import warp as wp

import newton

from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._hang_plate import (JointTracker, PairProbe, PhaseMachine, quat_of_R, smooth5,
                                                   tcp_offset_from_link7, actual_tcp)
from force_bench.appendix_tasks.common import (Placement, RobotContactProbe, StopRun, appendix_rig,
                                               check_released_and_clear, sample_start, save_appendix_verdict)
from force_bench.appendix_tasks.registry import CRITERIA
from force_bench.geometry import world_T
from force_bench.kit import add_asset, load_kit, pose_matrix
from force_bench.kit.catalog import print_mass

NAME = "stack_cups"
MM = 1000.0
Z0 = HP.Z0                              # bench top
CUP_XY = [np.array([0.000, -0.070]),     # the stack's base
          np.array([0.075, 0.050]),
          np.array([-0.070, 0.040])]
SQUARE_C = (0.0025, -0.010)             # seeded starts: the 0.15 x 0.15 m square around the nominal layout ...
SEED_DXY = 0.075                        # ... (each cup's centre uniform in +-75 mm per axis)
CUP_R = 0.035                           # footprint disc: the 34.3 mm rim + 0.7 mm (gap 5 mm: rims >= 6.4 mm apart)
STACK_STEP = 0.00468                    # measured nest step (see the docstring)
TIP_DEPTH = 0.012                       # fingertip bottom below the rim
TIP_OUTER = 0.01518                     # fingertip outer face beyond the finger dof (the URDF's rubber-tip box)
SQUEEZE_OUT = 0.00227                   # finger target beyond touch: the actuator force limit sets the load
F_CLOSED = 0.002
CLEAR_ABOVE_RIM = 0.025                 # carried cup's base above the stack's rim while moving
LOWER_STOP_ABOVE = 0.0008               # stop lowering this far above the nested height
CONTACT_STOP_N = 1.0                    # ...or when the carried cup touches the stack this hard
DUR = {"approach": 1.4, "insert": 0.9, "grip": 0.6, "lift": 1.0, "carry": 1.6, "release": 0.6,
       "settle": 0.4, "rise": 0.9}
LOWER_SPEED = 0.025                     # m/s while lowering into the stack
STILL_S = 1.0


def tcp_quat_down(closing_axis=(1.0, 0.0, 0.0)) -> np.ndarray:
    """Fingers pointing down (TCP z = -world z), closing along ``closing_axis`` (TCP y)."""
    z = np.array([0.0, 0.0, -1.0])
    y = np.asarray(closing_axis, float)
    y = y / np.linalg.norm(y)
    x = np.cross(y, z)
    return quat_of_R(np.column_stack([x, y, z]))


def build_bench(rig):
    x0, x1 = HP.BENCH_X
    y0, y1 = HP.BENCH_Y
    sid = rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.5 * Z0), wp.quat_identity()),
        hx=0.5 * (x1 - x0), hy=0.5 * (y1 - y0), hz=0.5 * Z0,
        cfg=rig.cfg(mu=HP.BENCH_MU, gap=0.0005, mu_torsional=HP.MU_TORSION), label="table")
    rig.shapes["table"] = [sid]
    rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(float(HP.ROBOT_BASE[0]), float(HP.ROBOT_BASE[1]), 0.5 * Z0),
                               wp.quat_identity()),
        hx=0.12, hy=0.12, hz=0.5 * Z0, cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False),
        label="arm_pedestal")


def cup_geometry():
    """Rim height, inner radius at a height (cup frame, base at z = 0), mass."""
    tm = load_kit()["cup"].parts()[0].mesh()
    tm.apply_translation([0.0, 0.0, -tm.bounds[0][2]])
    h = float(tm.bounds[1][2])
    zs = np.linspace(0.01, h - 0.001, 120)
    r_in = []
    for z in zs:
        loc, ri, _ = tm.ray.intersects_location(np.array([[0.0, 0.0, z]]), np.array([[1.0, 0.0, 0.0]]),
                                                multiple_hits=False)
        r_in.append(float(loc[0][0]) if len(loc) else np.nan)
    return tm, h, zs, np.asarray(r_in)


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, default=None, help="scatter the cups in the 0.15 x 0.15 m square (docstring)")
    ap.add_argument("--name", type=str, default=None, help="output folder name")
    return ap.parse_args(argv)


def draw_cups(seed: int | None) -> list[np.ndarray]:
    """The cups' start positions: the nominal layout without a seed, otherwise each cup anywhere in the square."""
    places = {f"cup_{c}": Placement(tuple(xy), dxy=SEED_DXY, radius=CUP_R, centre=SQUARE_C)
              for c, xy in zip("abc", CUP_XY)}
    start = sample_start(places, seed)
    return [np.array(start[k][:2]) for k in places]


def main(argv=None):
    a = parse(argv)
    cup_xy = draw_cups(a.seed)
    rig = appendix_rig(a.name or NAME)
    rig.collide_every_substep = True
    rig.solver_options["impratio"] = 10.0
    HP.setup_robot(rig)
    build_bench(rig)
    cup_tm, cup_h, zs, r_in = cup_geometry()
    cup_mass = print_mass(cup_tm)
    labels = [f"cup_{c}" for c in "abc"]
    cups = []
    for lbl, xy in zip(labels, cup_xy):
        P = pose_matrix(xyz=(xy[0], xy[1], Z0 + 0.0002))
        cups.append(add_asset(rig, "cup", pose=P, label=lbl))
    HP.filter_statics_and_ground(rig)
    for lbl in labels:
        HP.filter_arm_links(rig, lbl)

    z_grip = cup_h - TIP_DEPTH                                     # cup frame: fingertip bottom
    r_touch = float(np.interp(z_grip, zs, r_in))
    f_touch = r_touch - TIP_OUTER
    f_grip = f_touch + SQUEEZE_OUT
    q_down = tcp_quat_down((1.0, 0.0, 0.0))
    tcp_over = lambda xy, z_base, dz: np.array([xy[0], xy[1], z_base + z_grip + HP.TIP_BELOW_TCP + dz])  # noqa: E731
    print(f"cup: rim {cup_h*MM:.1f} mm, inner r at the grip {r_touch*MM:.2f} mm, finger touch {f_touch*MM:.2f} "
          f"grip {f_grip*MM:.2f} mm, mass {cup_mass*1e3:.1f} g")

    p_start = tcp_over(CUP_XY[1], Z0, 0.10)                        # the nominal start, for every seed
    q_start = rig.solve_arm_start(p_start, q_down)
    rig.builder.joint_q[7:9] = [F_CLOSED, F_CLOSED]
    rig.builder.joint_target_q[7:9] = [F_CLOSED, F_CLOSED]
    T_off = tcp_offset_from_link7(rig)

    cam_t = np.array([0.0, -0.005, Z0 + 0.060])
    cam_p = cam_t + np.array([0.27, -0.36, 0.20])
    probes = labels
    rig.finalize(probes=probes, cam_pos=cam_p, cam_target=cam_t)
    HP.set_camera_fov(rig, cam_p, cam_t, 42.0)
    shapes = {lbl: [s for s in rig.shapes[lbl]] for lbl in labels}
    lbls = list(rig.model.shape_label)
    for lbl in labels:
        names = [lbls[s] for s in shapes[lbl]]
        rig.probes[f"padL_{lbl}"] = PairProbe(rig.model, "fr3/fr3_leftfinger", names)
        rig.probes[f"padR_{lbl}"] = PairProbe(rig.model, "fr3/fr3_rightfinger", names)
    for i in (1, 2):
        rig.probes[f"stack_{labels[i]}"] = PairProbe(rig.model, labels[i],
                                                     [lbls[s] for j in range(i) for s in shapes[labels[j]]])
    rig.probes["robot_object"] = RobotContactProbe(rig.model, labels)
    ci = [rig.body_index(lbl) for lbl in labels]
    i7 = rig.body_index("fr3/fr3_link7")
    body_from_cad = []                    # cup body frame <- cup frame (base at z = 0): for the base height
    bq0 = rig.s0.body_q.numpy()
    for k in range(3):
        T = world_T(bq0[ci[k]])
        body_from_cad.append(np.linalg.inv(T) @ pose_matrix(xyz=(cup_xy[k][0], cup_xy[k][1], Z0 + 0.0002)))

    def cup_frame(bq, k):
        """World pose of cup k's base frame (base centre at the origin, axis = z)."""
        return world_T(bq[ci[k]]) @ body_from_cad[k]

    trk = JointTracker(rig, q_start)
    phases = []
    for k in (1, 2):
        phases += [f"{p}{k}" for p in ("approach", "insert", "grip", "lift", "carry", "lower", "release",
                                       "settle", "rise")]
    phases += ["still"]
    durations = {f"{p}{k}": d for k in (1, 2) for p, d in DUR.items()}
    durations["still"] = STILL_S
    st = {"p": p_start.copy(), "phase": None, "p_from": p_start.copy(), "t0": 0.0, "lower_done": False}
    log = []

    def lower_done(phase):
        return st["lower_done"]

    pm = PhaseMachine(phases, durations, lower_done)

    def last(name, key="force"):
        h = rig.probes[name].history
        return h[-1][key] if h else 0.0

    def control(t):
        pm.advance(t)
        if pm.finished:
            raise StopRun
        ph, s = pm.k, pm.frac(t)
        if ph == "still":
            trk.step(st["p"], q_down, F_CLOSED)
            return
        bq = rig.s0.body_q.numpy()
        k = int(ph[-1])
        name = ph[:-1]
        Tk, Tt = cup_frame(bq, k), cup_frame(bq, k - 1)        # carried cup, the cup it goes into
        xy_k, z_k = Tk[:2, 3], Tk[2, 3]
        xy_t, z_t = Tt[:2, 3], Tt[2, 3]
        if ph != st["phase"]:                                  # per-phase start state, goals frozen here
            st.update(phase=ph, p_from=st["p"].copy(), t0=t, lower_done=False,
                      dz_lift=(z_t + cup_h + CLEAR_ABOVE_RIM) - z_k, dxy=(xy_t - xy_k).copy())
        a = smooth5(s)
        fingers = f_grip
        if name == "approach":
            p = st["p_from"] + (tcp_over(xy_k, z_k, 0.030) - st["p_from"]) * a
            fingers = F_CLOSED
        elif name == "insert":
            p = st["p_from"] + (tcp_over(xy_k, z_k, 0.0) - st["p_from"]) * a
            fingers = F_CLOSED
        elif name == "grip":
            p = st["p_from"]
            fingers = F_CLOSED + (f_grip - F_CLOSED) * a
        elif name == "lift":                                   # carried cup's base CLEAR_ABOVE_RIM over the stack
            p = st["p_from"] + np.array([0.0, 0.0, st["dz_lift"]]) * a
        elif name == "carry":
            p = st["p_from"] + np.array([st["dxy"][0], st["dxy"][1], 0.0]) * a
        elif name == "lower":
            # down at LOWER_SPEED, xy servoed so the carried cup stays on the stack axis (measured)
            p = st["p"].copy()
            p[:2] += 0.2 * (xy_t - xy_k)
            p[2] = st["p_from"][2] - LOWER_SPEED * (t - st["t0"])
            touching = last(f"stack_{labels[k]}") > CONTACT_STOP_N
            if (z_k <= z_t + STACK_STEP + LOWER_STOP_ABOVE or touching) and not st["lower_done"]:
                st["lower_done"] = True
                p = st["p"]
                print(f"t={t:.2f}s cup {labels[k]} {(z_k - z_t - STACK_STEP)*MM:+.2f} mm from nested, contact "
                      f"{last(f'stack_{labels[k]}'):.2f} N")
        elif name == "release":
            p = st["p_from"]
            fingers = f_grip + (F_CLOSED - f_grip) * a
        elif name == "settle":
            p = st["p_from"]
            fingers = F_CLOSED
        else:                                                  # rise
            p = st["p_from"] + np.array([0.0, 0.0, 0.070]) * a
            fingers = F_CLOSED
        st["p"] = np.asarray(p, float)
        trk.step(st["p"], q_down, fingers)

    def extra(state, t):
        bq = state.body_q.numpy()
        row = {"t": t}
        for k in range(3):
            T = cup_frame(bq, k)
            row[f"x{k}"], row[f"y{k}"], row[f"z{k}"] = T[0, 3], T[1, 3], T[2, 3]
            row[f"tilt{k}"] = float(np.degrees(np.arccos(np.clip(T[2, 2], -1.0, 1.0))))
        tcp = actual_tcp(bq, i7, T_off)
        row["tcp_x"], row["tcp_y"], row["tcp_z"] = tcp
        for k in range(3):
            row[f"pad{k}"] = last(f"padL_{labels[k]}") + last(f"padR_{labels[k]}")
        row["robot_obj"] = last("robot_object")
        row["phase"] = phases.index(pm.k) if pm.k in phases else -1
        log.append(row)
        if int(round(t * rig.fps)) % 20 == 0:
            print(f"  [{t:5.2f} {pm.k:9s}] cups z {[round((row[f'z{k}'] - Z0) * MM, 2) for k in range(3)]} "
                  f"tilt {[round(row[f'tilt{k}'], 1) for k in range(3)]} "
                  f"pads {[round(row[f'pad{k}'], 2) for k in range(3)]} "
                  f"robot-obj {row['robot_obj']:.3f}")
        return {"pad_cup_b_N": round(row["pad1"], 3), "pad_cup_c_N": round(row["pad2"], 3),
                "robot_object_contact_N": round(row["robot_obj"], 4),
                "cup_b_base_mm": round((row["z1"] - Z0) * MM, 2), "cup_c_base_mm": round((row["z2"] - Z0) * MM, 2)}

    try:
        rec = rig.run(60.0, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{NAME}] expert finished at t={len(rec.times) / rig.fps:.2f}s")
    paths = rec.save("rollout", force_keys=["pad_cup_b_N", "pad_cup_c_N", "robot_object_contact_N"])
    print("saved:", paths)
    if not pm.finished:
        print("expert did not finish: QA skipped")
        return

    # ── verdict ─────────────────────────────────────────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    L = {k: np.array([r[k] for r in log]) for k in log[0]}
    tl = L["t"]
    end = len(tl) - 1
    # penetration: cup into cup and cup into the bench, on sampled frames
    local = {}
    for k, lbl in enumerate(labels):
        m = cup_tm.copy()                          # base frame -> body frame
        m.apply_transform(np.linalg.inv(body_from_cad[k]))
        local[lbl] = m
    qa.check_penetration([(labels[1], local[labels[1]], labels[0], local[labels[0]]),
                          (labels[2], local[labels[2]], labels[1], local[labels[1]])], tol=0.0008)
    bq_end = rig.s0.body_q.numpy()
    zmin = min(float((world_T(bq_end[ci[k]]) @ np.c_[local[labels[k]].vertices,
                                                    np.ones(len(local[labels[k]].vertices))].T)[2].min())
               for k in range(3)) - Z0
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["cups->bench_top"] = {"min_vertex_z_above_top_mm": round(zmin * MM, 3), "pass": bool(zmin > -0.0008)}
    pen["pass"] = bool(pen["pass"] and zmin > -0.0008)
    qa.check_envelope(labels, [-0.25, -0.30, Z0 - 0.01], [0.25, 0.30, Z0 + 0.40])

    crit = {}
    x0, y0, z0b = L["x0"][end], L["y0"][end], L["z0"][end]
    per = []
    ok_stack = True
    for k in (1, 2):
        off = float(np.hypot(L[f"x{k}"][end] - x0, L[f"y{k}"][end] - y0))
        dz = float(L[f"z{k}"][end] - (z0b + k * STACK_STEP))
        tilt = float(L[f"tilt{k}"][end])
        good = off < 0.002 and abs(dz) < 0.0015 and tilt < 3.0
        ok_stack &= good
        per.append({"cup": labels[k], "axis_offset_mm": round(off * MM, 3),
                    "height_error_mm": round(dz * MM, 3), "tilt_deg": round(tilt, 2), "pass": bool(good)})
    crit["cups_stacked"] = {"pass": bool(ok_stack), "stack_step_mm": STACK_STEP * MM, "cups": per}
    moved = float(np.hypot(L["x0"][end] - L["x0"][0], L["y0"][end] - L["y0"][0]))
    crit["stack_not_knocked"] = {"pass": bool(moved < 0.010 and L["tilt0"][end] < 3.0),
                                 "bottom_cup_moved_mm": round(moved * MM, 2),
                                 "bottom_cup_tilt_deg": round(float(L["tilt0"][end]), 2)}
    held = []
    ok_held = True
    for k in (1, 2):
        ph0, ph1 = phases.index(f"carry{k}"), phases.index(f"lower{k}")
        m = (L["phase"] >= ph0) & (L["phase"] <= ph1)
        mean = float(L[f"pad{k}"][m].mean()) if m.any() else 0.0
        lifted = float(L[f"z{k}"][m].max() - L[f"z{k}"][0]) if m.any() else 0.0
        good = mean > 0.5 and lifted > 0.05
        ok_held &= good
        held.append({"cup": labels[k], "mean_finger_force_N": round(mean, 2), "lifted_mm": round(lifted * MM, 1),
                     "pass": bool(good)})
    crit["real_inside_grasp"] = {"pass": bool(ok_held), "cups": held}
    t_still = pm.t_start["still"]
    tmp = type("Q", (), {"report": {"checks": {}, "pass": True}})()
    check_released_and_clear(tmp, tl, L["robot_obj"], t_still, tcp_z=L["tcp_z"], tcp_rise_min=0.0)
    rel = tmp.report["checks"]["released_and_clear"]
    rise = float(L["tcp_z"][end] - (L["z2"][end] + cup_h))
    rel["hand_above_stack_rim_mm"] = round(rise * MM, 1)
    rel["pass"] = bool(rel["pass"] and rise > 0.05)
    crit["released_and_clear"] = rel
    v_end = float(np.linalg.norm(np.diff(np.c_[L["tcp_x"], L["tcp_y"], L["tcp_z"]][-10:], axis=0), axis=1).mean()
                  * rig.fps)
    crit["robot_stopped"] = {"pass": bool(v_end < 0.001), "tcp_speed_mm_s": round(v_end * MM, 3)}
    for k_, c_ in crit.items():
        qa.report["checks"].setdefault(k_, c_)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    crit["start"] = {"pass": True, "seed": a.seed,
                     "cups_xy_mm": {lbl: [round(float(v) * MM, 1) for v in xy] for lbl, xy in zip(labels, cup_xy)},
                     "range": {"square_centre_mm": [round(v * MM, 1) for v in SQUARE_C],
                               "xy_mm": round(SEED_DXY * MM, 1), "footprint_radius_mm": round(CUP_R * MM, 1)}}
    seeded = ([f"seeded start {a.seed}: cups at " + ", ".join(f"({xy[0] * MM:+.1f}, {xy[1] * MM:+.1f})"
                                                             for xy in cup_xy)
               + " mm (each uniform in the 0.15 x 0.15 m square, drawn jointly without overlap)"]
              if a.seed is not None else [])
    save_appendix_verdict(
        rig, task="Stack cups", source="new task (finalized kit cups, force_bench.kit)", spec=CRITERIA[24],
        criteria=crit, physics_qa=report,
        changes=seeded + [
            "three kit cups (68.6 mm rim, 1 mm wall, 15 g) stand upright on the bench; the first is the stack's "
            "base, the robot nests the second into it and the third into the second",
            "inside grasp: fingertips in the cup mouth, opened onto the inner wall 12 mm below the rim, so the "
            "fingers never come between the cups being nested",
            "cups collide through 0.2 mm SDFs with 0.15 mm margin and contact gap (thin nested walls)",
            "the carried cup is lowered on the measured stack axis until it is within 0.8 mm of its nested "
            "height or touches the stack, then released to settle under its own weight",
        ])


if __name__ == "__main__":
    wp.init()
    main()
