"""Appendix task 4 "Retrieve top cup from stack".

    Description: A neatly organized stack of cups is placed in the workspace.  Pick up the top cup from the stack
    and place it on the table without disturbing the rest of the stack.
    Success: The robot movement stops after separating the top cup from the stack and placing it on the table.  The
    remaining cups should remain in a neat stack.  The robot should not remain in contact with the task objects.

NEW TASK on the finalized kit: three kit cups nested at their natural 4.68 mm step (the cup model and the inside
grasp of ``stack_cups``).  The expert puts its closed fingertips into the top cup's mouth, opens them onto its inner
wall, lifts it straight up out of the stack, carries it to free table space, lowers it until it stands on the
table, opens, rises and holds still.  Only the top cup is touched: the fingers stay inside it.

SEEDED START (--seed N): the stack's position is drawn uniformly within +-50 mm of its nominal place (a 0.10 x
0.10 m square inside the paper's 0.15 m one: the fixed camera frames the stack and the set-down spot, and at +-75 mm
the stack's near side leaves the frame); the cups have no yaw.  The cup is set down at its nominal offset from the
stack (131 mm, on the side away from the camera), and the hand starts at the nominal pose for every seed.  The draw
is recorded in the verdict ("start").

VERDICT (appendix_qa.json):
  top_cup_on_table     the top cup stands upright (tilt < 3 deg) on the table (base within 1 mm of the top),
                       at least 80 mm from the stack's axis
  stack_remains_neat   the two remaining cups are still nested: axes within 2 mm, step within 1.5 mm of 4.68 mm,
                       tilts < 3 deg, the bottom cup moved < 3 mm
  real_inside_grasp    the top cup was held by finger contact (> 0.5 N mean) while carried, lifted > 60 mm
  released_and_clear   robot <-> cup contact < 0.05 N over the final second, hand > 50 mm above the cup's rim
  robot_stopped        TCP speed < 1 mm/s at the end
plus the physics-integrity checks.

OPTIONS  --seed N (the stack's position, see SEEDED START), --name (output folder).

Run: force-bench run retrieve_top_cup_from_stack [--seed N]
"""

from __future__ import annotations

import argparse

import numpy as np
import warp as wp

from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._hang_plate import (JointTracker, PairProbe, PhaseMachine, smooth5,
                                                   tcp_offset_from_link7, actual_tcp)
from force_bench.appendix_tasks.common import (Placement, RobotContactProbe, StopRun, appendix_rig,
                                               check_released_and_clear, sample_start, save_appendix_verdict)
from force_bench.appendix_tasks.registry import CRITERIA
from force_bench.appendix_tasks.stack_cups import (STACK_STEP, TIP_DEPTH, TIP_OUTER, SQUEEZE_OUT, F_CLOSED, Z0,
                                                   build_bench, cup_geometry, tcp_quat_down)
from force_bench.geometry import world_T
from force_bench.kit import add_asset, pose_matrix

NAME = "retrieve_top_cup_from_stack"
MM = 1000.0
STACK_XY = np.array([0.000, -0.040])
PLACE_XY = np.array([0.020, 0.090])     # the set-down spot (seeded starts: the same offset from the drawn stack)
SEED_DXY = 0.050                        # seeded starts: the stack within +-50 mm of STACK_XY (camera framing)
LIFT_CLEAR = 0.020                      # top cup's base above the stack's rim while moving
PLACE_SPEED = 0.025
PLACE_STOP_ABOVE = 0.0005
DUR = {"approach": 1.4, "insert": 0.9, "grip": 0.6, "lift": 1.4, "carry": 1.6, "release": 0.6, "settle": 0.4,
       "rise": 0.9, "still": 1.0}


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seed", type=int, default=None, help="draw the stack's position (docstring)")
    ap.add_argument("--name", type=str, default=None, help="output folder name")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    stack_xy = np.array(sample_start({"stack": Placement(tuple(STACK_XY), dxy=SEED_DXY, radius=0.035)},
                                     a.seed)["stack"][:2])
    place_xy = PLACE_XY + (stack_xy - STACK_XY)                 # the nominal offset from the stack
    rig = appendix_rig(a.name or NAME)
    rig.collide_every_substep = True
    rig.solver_options["impratio"] = 10.0
    HP.setup_robot(rig)
    build_bench(rig)
    cup_tm, cup_h, zs, r_in = cup_geometry()
    labels = [f"cup_{c}" for c in "abc"]
    poses = [pose_matrix(xyz=(stack_xy[0], stack_xy[1], Z0 + 0.0002 + k * STACK_STEP)) for k in range(3)]
    for lbl, P in zip(labels, poses):
        add_asset(rig, "cup", pose=P, label=lbl)
    HP.filter_statics_and_ground(rig)
    for lbl in labels:
        HP.filter_arm_links(rig, lbl)

    z_grip = cup_h - TIP_DEPTH
    f_touch = float(np.interp(z_grip, zs, r_in)) - TIP_OUTER
    f_grip = f_touch + SQUEEZE_OUT
    q_down = tcp_quat_down((1.0, 0.0, 0.0))

    def tcp_over(xy, z_base, dz):
        return np.array([xy[0], xy[1], z_base + z_grip + HP.TIP_BELOW_TCP + dz])

    top = 2
    p_start = tcp_over(STACK_XY, Z0 + top * STACK_STEP, 0.10)    # the nominal start, for every seed
    q_start = rig.solve_arm_start(p_start, q_down)
    rig.builder.joint_q[7:9] = [F_CLOSED, F_CLOSED]
    rig.builder.joint_target_q[7:9] = [F_CLOSED, F_CLOSED]
    T_off = tcp_offset_from_link7(rig)
    cam_t = np.array([0.01, 0.025, Z0 + 0.060])
    cam_p = cam_t + np.array([0.28, -0.34, 0.20])
    rig.finalize(probes=labels, cam_pos=cam_p, cam_target=cam_t)
    HP.set_camera_fov(rig, cam_p, cam_t, 42.0)
    lbls = list(rig.model.shape_label)
    top_shapes = [lbls[s] for s in rig.shapes[labels[top]]]
    rig.probes["padL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", top_shapes)
    rig.probes["padR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", top_shapes)
    rig.probes["table_top_cup"] = PairProbe(rig.model, labels[top], ["table"])
    rig.probes["robot_object"] = RobotContactProbe(rig.model, labels)
    ci = [rig.body_index(lbl) for lbl in labels]
    i7 = rig.body_index("fr3/fr3_link7")
    bq0 = rig.s0.body_q.numpy()
    body_from_cad = [np.linalg.inv(world_T(bq0[ci[k]])) @ poses[k] for k in range(3)]

    def cup_frame(bq, k):
        return world_T(bq[ci[k]]) @ body_from_cad[k]

    phases = ["approach", "insert", "grip", "lift", "carry", "lower", "release", "settle", "rise", "still"]
    st = {"p": p_start.copy(), "phase": None, "p_from": p_start.copy(), "t0": 0.0, "done": False}
    pm = PhaseMachine(phases, DUR, lambda k: st["done"])
    trk = JointTracker(rig, q_start)
    log = []

    def last(name, key="force"):
        h = rig.probes[name].history
        return h[-1][key] if h else 0.0

    def control(t):
        pm.advance(t)
        if pm.finished:
            raise StopRun
        ph, s = pm.k, pm.frac(t)
        bq = rig.s0.body_q.numpy()
        T = cup_frame(bq, top)
        xy, z = T[:2, 3], T[2, 3]
        if ph != st["phase"]:                                  # per-phase start state, goals frozen here
            rim = Z0 + (top - 1) * STACK_STEP + cup_h          # the cup below's rim
            st.update(phase=ph, p_from=st["p"].copy(), t0=t, done=False,
                      dz_lift=rim + LIFT_CLEAR - z, dxy=(place_xy - xy).copy())
        a = smooth5(s)
        fingers = f_grip
        if ph == "approach":
            p = st["p_from"] + (tcp_over(xy, z, 0.030) - st["p_from"]) * a
            fingers = F_CLOSED
        elif ph == "insert":
            p = st["p_from"] + (tcp_over(xy, z, 0.0) - st["p_from"]) * a
            fingers = F_CLOSED
        elif ph == "grip":
            p = st["p_from"]
            fingers = F_CLOSED + (f_grip - F_CLOSED) * a
        elif ph == "lift":
            p = st["p_from"] + np.array([0.0, 0.0, st["dz_lift"]]) * a
        elif ph == "carry":
            p = st["p_from"] + np.array([st["dxy"][0], st["dxy"][1], 0.0]) * a
        elif ph == "lower":
            p = st["p"].copy()
            p[2] = st["p_from"][2] - PLACE_SPEED * (t - st["t0"])
            if (z <= Z0 + PLACE_STOP_ABOVE or last("table_top_cup") > 0.5) and not st["done"]:
                st["done"] = True
                p = st["p"]
                print(f"t={t:.2f}s top cup base {(z - Z0)*MM:+.2f} mm above the table, contact "
                      f"{last('table_top_cup'):.2f} N")
        elif ph == "release":
            p = st["p_from"]
            fingers = f_grip + (F_CLOSED - f_grip) * a
        elif ph == "settle":
            p, fingers = st["p_from"], F_CLOSED
        elif ph == "rise":
            p, fingers = st["p_from"] + np.array([0.0, 0.0, 0.070]) * a, F_CLOSED
        else:
            p, fingers = st["p_from"], F_CLOSED
        st["p"] = np.asarray(p, float)
        trk.step(st["p"], q_down, fingers)

    def extra(state, t):
        bq = state.body_q.numpy()
        row = {"t": t, "phase": phases.index(pm.k)}
        for k in range(3):
            T = cup_frame(bq, k)
            row[f"x{k}"], row[f"y{k}"], row[f"z{k}"] = T[0, 3], T[1, 3], T[2, 3]
            row[f"tilt{k}"] = float(np.degrees(np.arccos(np.clip(T[2, 2], -1.0, 1.0))))
        row["tcp_x"], row["tcp_y"], row["tcp_z"] = actual_tcp(bq, i7, T_off)
        row["pads"] = last("padL") + last("padR")
        row["robot_obj"] = last("robot_object")
        log.append(row)
        if int(round(t * rig.fps)) % 20 == 0:
            print(f"  [{t:5.2f} {pm.k:9s}] cups z {[round((row[f'z{k}'] - Z0) * MM, 2) for k in range(3)]} "
                  f"pads {row['pads']:.2f} N robot-obj {row['robot_obj']:.3f}")
        return {"pads_N": round(row["pads"], 3), "robot_object_contact_N": round(row["robot_obj"], 4),
                "top_cup_base_mm": round((row["z2"] - Z0) * MM, 2)}

    try:
        rec = rig.run(40.0, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{NAME}] expert finished at t={len(rec.times) / rig.fps:.2f}s")
    paths = rec.save("rollout", force_keys=["pads_N", "robot_object_contact_N"])
    if not pm.finished:
        print("expert did not finish: QA skipped")
        return

    qa = rig.qa(paths)
    L = {k: np.array([r[k] for r in log]) for k in log[0]}
    end = len(L["t"]) - 1
    local = {}
    for k, lbl in enumerate(labels):
        m = cup_tm.copy()
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
    d_axis = float(np.hypot(L["x2"][end] - L["x0"][end], L["y2"][end] - L["y0"][end]))
    on_table = abs(L["z2"][end] - Z0) < 0.001 and L["tilt2"][end] < 3.0 and d_axis > 0.080
    crit["top_cup_on_table"] = {"pass": bool(on_table), "base_above_table_mm": round((L["z2"][end] - Z0) * MM, 3),
                                "tilt_deg": round(float(L["tilt2"][end]), 2),
                                "distance_from_stack_axis_mm": round(d_axis * MM, 1)}
    off = float(np.hypot(L["x1"][end] - L["x0"][end], L["y1"][end] - L["y0"][end]))
    step = float(L["z1"][end] - L["z0"][end])
    moved = float(np.hypot(L["x0"][end] - L["x0"][0], L["y0"][end] - L["y0"][0]))
    neat = (off < 0.002 and abs(step - STACK_STEP) < 0.0015 and L["tilt0"][end] < 3.0 and L["tilt1"][end] < 3.0
            and moved < 0.003)
    crit["stack_remains_neat"] = {"pass": bool(neat), "axis_offset_mm": round(off * MM, 3),
                                  "step_mm": round(step * MM, 3), "bottom_cup_moved_mm": round(moved * MM, 2),
                                  "tilts_deg": [round(float(L["tilt0"][end]), 2), round(float(L["tilt1"][end]), 2)]}
    m = (L["phase"] >= phases.index("lift")) & (L["phase"] <= phases.index("lower"))
    mean = float(L["pads"][m].mean()) if m.any() else 0.0
    lifted = float(L["z2"][m].max() - L["z2"][0]) if m.any() else 0.0
    crit["real_inside_grasp"] = {"pass": bool(mean > 0.5 and lifted > 0.060), "mean_finger_force_N": round(mean, 2),
                                 "lifted_mm": round(lifted * MM, 1)}
    tmp = type("Q", (), {"report": {"checks": {}, "pass": True}})()
    check_released_and_clear(tmp, L["t"], L["robot_obj"], _phase_start(pm, "still"))
    rel = tmp.report["checks"]["released_and_clear"]
    above = float(L["tcp_z"][end] - (L["z2"][end] + cup_h))
    rel["hand_above_cup_rim_mm"] = round(above * MM, 1)
    rel["pass"] = bool(rel["pass"] and above > 0.05)
    crit["released_and_clear"] = rel
    v_end = float(np.linalg.norm(np.diff(np.c_[L["tcp_x"], L["tcp_y"], L["tcp_z"]][-10:], axis=0), axis=1).mean()
                  * rig.fps)
    crit["robot_stopped"] = {"pass": bool(v_end < 0.001), "tcp_speed_mm_s": round(v_end * MM, 3)}
    for k_, c_ in crit.items():
        qa.report["checks"].setdefault(k_, c_)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    crit["start"] = {"pass": True, "seed": a.seed, "stack_xy_mm": [round(float(v) * MM, 1) for v in stack_xy],
                     "set_down_xy_mm": [round(float(v) * MM, 1) for v in place_xy],
                     "range": {"xy_mm": round(SEED_DXY * MM, 1)}}
    seeded = ([f"seeded start {a.seed}: stack at ({stack_xy[0] * MM:+.1f}, {stack_xy[1] * MM:+.1f}) mm (uniform "
               f"within +-{SEED_DXY * MM:.0f} mm of its nominal place), set down at ({place_xy[0] * MM:+.1f}, "
               f"{place_xy[1] * MM:+.1f}) mm"] if a.seed is not None else [])
    save_appendix_verdict(
        rig, task="Retrieve top cup from stack", source="new task (finalized kit cups, force_bench.kit)",
        spec=CRITERIA[4], criteria=crit, physics_qa=report,
        changes=seeded + [
            "three kit cups nested at their natural 4.68 mm step (measured without a robot) start the episode",
            "inside grasp of the top cup (fingertips in its mouth, opened onto the inner wall 12 mm below the rim): "
            "only the top cup is touched",
            "cups collide through 0.2 mm SDFs with 0.15 mm margin and contact gap (thin nested walls)",
        ])


def _phase_start(pm, name):
    return pm.t_start.get(name, 0.0)


if __name__ == "__main__":
    wp.init()
    main()
