"""Appendix task "Hang object" (Table I, #9), WRENCH variant: the FR3 picks
the kit wrench up from the bench and hangs it by its 10 mm hole on the
middle hook of the hanging board.

    Success (appendix): "The robot movement stops after placing the target
    object onto a hanging hook. The robot should not remain in contact with
    the task objects."

The appendix starts with the wrench in hand; here it starts lying flat on
the bench (user request) and is picked up first.  Scene, hook geometry and
why the hole is guided onto the bar instead of dropped on the tip:
``_hang_plate.py``.

EXPERT (one continuous rollout, fingertip contact only, nothing welded):
  1. settle, parked 60 mm above the balance point, pads opening;
  2. descend: pads either side of the handle at the CENTRE OF MASS, tips
     2 mm above the bench (the wrench is 7.5 mm thick, so ~5 mm of each pad
     face lands on the handle's flat sides); a short static pause there
     lets the droop compensator learn the arm's sag before the pads touch;
  3. close to ~5.6 N/pad (finger target 0.6 mm inside the handle, 8 kN/m
     drive, 15 N actuator clamp).  The pinch is AT the centre of mass, so
     gravity puts no moment on it in any attitude;
  4. lift 80 mm, then carry to a hover 35 mm out along the hook's 60 deg
     stub axis, turning the wrench (yaw) so its hole end faces the board and
     tilting it 30 deg (hole axis = stub axis);
  5. thread straight down the stub axis until the hole is 4 mm past the cap
     centre -- the stub is now through the hole;
  6. follow the hook centre-line round the bend onto the flat bar while the
     tilt ramps 30 -> 86 deg (HookPath.tilt), slowly where it turns;
  7. seat: lower the hole until the bar carries the wrench (force-gated);
  8. open the pads, retreat along the plate normal and up, hold still.
Steps 4-6 are closed-loop on the MEASURED hole centre; the path progress of
5-6 is gated by the plate<->board contact force.

Run:  python -m force_bench.appendix_tasks.hang_wrench
      (TB_OUT_ROOT=<dir> for iterations; TB_RENDER_FAST=1 TB_MULTIVIEW=0;
       HW_STOP_T=<s> stops early and skips the QA; HW_PREVIEW=<states.npz>
       renders a camera contact sheet instead of simulating)
"""

from __future__ import annotations

import os

import numpy as np
import warp as wp

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv, world_T
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, save_appendix_verdict, StopRun)
from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._hang_plate import (
    MM, Plate, HookPath, HangCfg, HangExpert, HANG_PHASES, PhaseMachine, DroopComp,
    JointTracker, PairProbe, RunLog, hang_frame, tcp_pose_for, smooth5, pierce)

NAME = "hang_wrench"


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


PEG_Y = HP.PEG_YS[1]                 # middle hook
DENSITY = 1240.0                     # PLA: the kit is 3D printed -> 13.5 g
WRENCH_MU = 0.5
SUBDIV = 0.0030                      # vertex density for the SDF pairs (pitfall 3.2)
SDF_W = dict(sdf=True, sdf_resolution=512, narrow_band=0.0015, margin=0.0005)
GAP_W = 0.0005
IMPRATIO = _env("HW_IMPRATIO", 10.0)
# IMPRATIO 1 (the suite default) lets a friction pinch creep under any steady
# load (rig.py): measured on this grasp, the wrench turned 7 deg in the pads
# during the 30 deg carry tilt and 4 deg more while threading, with 5.6 N/pad
# and the pinch AT the centre of mass.  10 is what the GelSight rig and
# hang_keychain use; with it the same carry creeps < 1 deg.
HAND_FLIP = True
# the hand rolled 180 deg about its approach axis (the same grasp, fingers
# swapped): un-flipped, joint 7 runs into its stop in the bend (offline IK
# along the whole commanded path: 0.00 mm residual, >= 0.43 rad margin flipped)

# start: lying flat, balance point here, hang direction (hole -> COM) yawed
START_COM_XY = np.array([_env("HW_X", 0.060), _env("HW_Y", 0.060)])
START_YAW = np.radians(_env("HW_YAW_DEG", -40.0))
REST_CLEAR = 0.00005

# grasp
TIP_CLEAR = 0.0020                   # fingertips above the bench at the grasp
SQUEEZE = _env("HW_SQUEEZE", 0.0006)  # finger target inside the handle half-width
OPEN_EXTRA = 0.0070                  # pad clearance per side when open
LIFT_Z = 0.080

CFG = HangCfg(tilt_hi=np.radians(_env("HW_TILT_DEG", 86.0)),
              bar_x=_env("HW_BAR_X", -0.1070))

T_SETTLE, T_DESC, T_PRE, T_CLOSE, T_HOLD = 0.5, 1.0, 0.4, 0.6, 0.3
T_LIFT, T_CARRY = 1.0, 2.6
PICK_PHASES = ["settle", "descend", "pre", "close", "hold", "lift", "carry"]
PICK_DUR = {"settle": T_SETTLE, "descend": T_DESC, "pre": T_PRE, "close": T_CLOSE,
            "hold": T_HOLD, "lift": T_LIFT, "carry": T_CARRY}
T_BUDGET = 40.0                      # hard cap; the rollout stops after "still"


def handle_halfwidths(plate: Plate, point_xy, d):
    """Distances from point_xy to the outline along +-(z x d) in the kit
    mid-plane: where the handle's two flat sides are."""
    sec = plate.tm.section(plane_origin=[0, 0, plate.z_mid], plane_normal=[0, 0, 1])
    w = np.cross([0.0, 0.0, 1.0], d)[:2]
    out = []
    for sgn in (1.0, -1.0):
        best = None
        for lp in sec.discrete:
            P = lp[:, :2]
            for a, b in zip(P[:-1], P[1:]):
                M = np.column_stack([sgn * w, a - b])
                if abs(np.linalg.det(M)) < 1e-12:
                    continue
                s, r = np.linalg.solve(M, a - point_xy)
                if s > 0 and 0 <= r <= 1 and (best is None or s < best):
                    best = s
        out.append(best)
    return out[0], out[1]


def slerp(qa, qb, a):
    qa = np.asarray(qa, float)
    qb = np.asarray(qb, float)
    if np.dot(qa, qb) < 0:
        qb = -qb
    r = wp.quat_slerp(wp.quat(*qa.tolist()), wp.quat(*qb.tolist()), float(a))
    return np.array([r[0], r[1], r[2], r[3]])


def main():
    rig = appendix_rig(NAME)
    rig.collide_every_substep = True
    rig.solver_options["impratio"] = IMPRATIO
    HP.setup_robot(rig)
    HP.build_bench_and_board(rig)

    # ── the wrench ──────────────────────────────────────────────────────
    raw = HP.load_kit_mesh("wrench")
    plate = Plate.measure("wrench", raw, DENSITY)
    L_com = float(np.linalg.norm((plate.com - plate.hole)[:2]))
    w_pos, w_neg = handle_halfwidths(plate, plate.com[:2], plate.d)
    e_w = np.cross([0.0, 0.0, 1.0], plate.d)
    grasp_kit = np.array([*plate.com[:2], plate.z_mid]) + 0.5 * (w_pos - w_neg) * e_w
    half_w = 0.5 * (w_pos + w_neg)
    L_grip = float((grasp_kit - plate.hole) @ plate.d)
    off_w = float((grasp_kit - plate.hole) @ e_w)
    f_touch = half_w + HP.PAD_FACE_OFF
    f_close = f_touch - SQUEEZE
    f_open = f_touch + OPEN_EXTRA
    h_tcp = HP.TIP_BELOW_TCP + TIP_CLEAR - 0.5 * plate.thick
    tm = subdiv(raw, SUBDIV)
    plate.tm = tm
    print(f"wrench: {plate.mass*1000:.1f} g, hole r {plate.r_hole*MM:.3f} mm, hole->COM "
          f"{L_com*MM:.1f} mm, thick {plate.thick*MM:.2f} mm, {len(tm.faces)} faces")
    print(f"handle at the balance point {2*half_w*MM:.2f} mm wide (sides {w_pos*MM:.2f}/"
          f"{w_neg*MM:.2f}); grip {L_grip*MM:.1f} mm from the hole; finger dof touch "
          f"{f_touch*MM:.2f} close {f_close*MM:.2f} open {f_open*MM:.2f} mm")

    mesh, c = make_newton_mesh(tm, **SDF_W)
    plate.center = np.asarray(c, float)
    d0 = np.array([np.cos(START_YAW), np.sin(START_YAW), 0.0])
    n0 = np.array([0.0, 0.0, 1.0])
    H0 = np.array([*(START_COM_XY - L_com * d0[:2]), HP.Z0 + 0.5 * plate.thick + REST_CLEAR])
    p0, q0 = plate.body_pose(H0, n0, d0)
    body = rig.builder.add_body(xform=wp.transform(wp.vec3(*p0), wp.quat(*q0)), label="wrench")
    sid = rig.builder.add_shape_mesh(
        body, mesh=mesh, cfg=rig.cfg(mu=WRENCH_MU, gap=GAP_W, mu_torsional=HP.MU_TORSION),
        label="wrench_shape")
    rig.shapes["wrench"] = [sid]
    set_body_mass(rig.builder, body, plate.mass)
    rig.meshes["wrench"] = (tm, plate.center)
    HP.filter_arm_links(rig, "wrench")
    HP.filter_statics_and_ground(rig)

    hook = HookPath(PEG_Y)
    poly = hook.polyline()

    def tcp_for(H, theta=None, n=None, d=None):
        if theta is not None:
            n, d = hang_frame(theta)
        g = H + L_grip * d + off_w * np.cross(n, d)
        return tcp_pose_for(g, n, d, h_tcp, flip=HAND_FLIP)

    hx = HangExpert(CFG, hook, plate, tcp_for, f_close, f_open, rig.fps)

    # ── key poses ───────────────────────────────────────────────────────
    tcp_grasp, q_grasp = tcp_for(H0, n=n0, d=d0)
    tcp_park = tcp_grasp + np.array([0.0, 0.0, 0.060])
    tcp_lift = tcp_grasp + np.array([0.0, 0.0, LIFT_Z])
    tcp_hover, q_hover = hx.hover()

    q_start = rig.solve_arm_start(tcp_park, q_grasp)
    rig.builder.joint_q[7:9] = [0.04, 0.04]
    rig.builder.joint_target_q[7:9] = [0.04, 0.04]
    T_off = HP.tcp_offset_from_link7(rig)
    print(f"arm start TCP err {np.linalg.norm(rig.ik.tcp_pose()[:3]-tcp_park)*MM:.2f} mm")

    cam_t = np.array([_env("HW_CAM_TX", -0.030), _env("HW_CAM_TY", 0.010),
                      HP.Z0 + _env("HW_CAM_TZ", 0.090)])
    cam_p = cam_t + np.array([_env("HW_CAM_OX", 0.42), _env("HW_CAM_OY", -0.52),
                              _env("HW_CAM_OZ", 0.30)])
    if os.environ.get("HW_PREVIEW"):
        rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
        HP.preview_cameras(
            rig, os.environ["HW_PREVIEW"],
            [float(v) for v in os.environ.get("HW_PREVIEW_T", "1,4,8,11,13,15").split(",")],
            HP.parse_cams(os.environ.get("HW_PREVIEW_CAMS", f"{','.join(map(str, cam_t))}:"
                                         f"{','.join(map(str, cam_p - cam_t))}")),
            os.environ.get("HW_PREVIEW_OUT", str(rig.out_dir / "camera_preview.png")))
        return
    rig.finalize(probes=["wrench"], cam_pos=cam_p, cam_target=cam_t)
    HP.set_camera_fov(rig, cam_p, cam_t, _env("HW_CAM_FOV", 40.0))
    rig.probes["padL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", ["wrench_shape"])
    rig.probes["padR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", ["wrench_shape"])
    rig.probes["board"] = PairProbe(rig.model, "wrench", ["board"])
    rig.probes["bench"] = PairProbe(rig.model, "wrench", ["table"])
    rig.probes["robot_object"] = RobotContactProbe(rig.model, ["wrench"])
    wi = rig.body_index("wrench")
    i7 = rig.body_index("fr3/fr3_link7")
    v_low = plate.local_mesh().vertices[::5]

    trk = JointTracker(rig, q_start)
    droop = DroopComp()
    pm = PhaseMachine(PICK_PHASES + HANG_PHASES, {**PICK_DUR, **hx.durations()}, hx.done)
    st = {"p": tcp_park.copy(), "q": np.asarray(q_grasp, float), "begun": False}
    log = RunLog()

    def last(name, key="force"):
        h = rig.probes[name].history
        return h[-1][key] if h else 0.0

    def board_fz():
        h = rig.probes["board"].history
        return h[-1]["vec"][2] if h else 0.0

    def control(t):
        pm.advance(t)
        if pm.finished:
            raise StopRun
        k, s = pm.k, pm.frac(t)
        bq = rig.s0.body_q.numpy()
        H_meas, _, _ = plate.frame_of(bq[wi])
        tcp_meas = HP.actual_tcp(bq, i7, T_off)
        fingers = f_close
        if k == "settle":
            fingers = 0.04 + (f_open - 0.04) * smooth5(s)
            p, q = tcp_park, q_grasp
        elif k == "descend":
            fingers = f_open
            p, q = tcp_park + (tcp_grasp - tcp_park) * smooth5(s), q_grasp
        elif k == "pre":
            fingers = f_open
            p, q = tcp_grasp, q_grasp
        elif k == "close":
            fingers = f_open + (f_close - f_open) * smooth5(s)
            p, q = tcp_grasp, q_grasp
        elif k == "hold":
            p, q = tcp_grasp, q_grasp
        elif k == "lift":
            p, q = tcp_grasp + (tcp_lift - tcp_grasp) * smooth5(s), q_grasp
        elif k == "carry":
            a = smooth5(s)
            p = tcp_lift + (tcp_hover - tcp_lift) * a + np.array([0.0, 0.0, 0.03 * np.sin(np.pi * a)])
            q = slerp(q_grasp, q_hover, a)
        else:
            if not st["begun"]:
                hx.begin(droop.v)
                st["begun"] = True
                print(f"t={t:.2f}s droop {np.round(droop.v*MM, 2)} mm handed to the hole loop")
            p, q, fingers, _ = hx.step(k, s, H_meas, last("board"), board_fz(), st["p"], st["q"])
        if k in PICK_PHASES:
            droop.update(k in ("settle", "pre"), p, tcp_meas, st["p"])
            p_out = p + droop.v
        else:
            p_out = p
        st["p"], st["q"] = np.asarray(p, float), np.asarray(q, float)
        trk.step(p_out, q, fingers)

    def extra(state, t):
        bq = state.body_q.numpy()
        H, n, _ = plate.frame_of(bq[wi])
        tilt = float(np.degrees(np.arctan2(n[0], n[2])))
        off, px = pierce(plate, bq[wi], poly)
        tcp_a = HP.actual_tcp(bq, i7, T_off)
        k = pm.k
        tcp_cmd = st["p"] if k in PICK_PHASES else st["p"]
        err = float(np.linalg.norm(tcp_a - tcp_cmd))
        herr = float(np.linalg.norm(H - hx.H_cmd)) if k in HANG_PHASES[:5] else 0.0
        T = world_T(bq[wi])
        low_z = float(((T[:3, :3] @ v_low.T).T + T[:3, 3])[:, 2].min())
        log.add(t=t, Hx=H[0], Hy=H[1], Hz=H[2], nx=n[0], ny=n[1], nz=n[2], tilt=tilt,
                pierce_off=off, pierce_x=px, padL=last("padL"), padR=last("padR"),
                board=last("board"), board_fz=board_fz(), bench=last("bench"),
                robot_obj=last("robot_object"), tcp_x=tcp_a[0], tcp_y=tcp_a[1],
                tcp_z=tcp_a[2], tcp_err=err, hole_err=herr, low_z=low_z)
        if int(round(t * rig.fps)) % 15 == 0:
            print(f"  [{t:5.2f} {k:9s}] pads {last('padL'):5.2f}/{last('padR'):5.2f} N  board "
                  f"{last('board'):6.3f} (fz {board_fz():+.3f}) bench {last('bench'):5.3f}  "
                  f"hole {np.round(H*MM,1)} tilt {tilt:5.1f} herr {herr*MM:5.2f} "
                  f"u {hx.u*MM:6.2f} pierce {off*MM:5.2f}@{px*MM:6.1f} tcp_err {err*MM:5.2f} "
                  f"corr {np.round(hx.corr*MM, 2)}")
        return {"padL_N": round(last("padL"), 3), "padR_N": round(last("padR"), 3),
                "board_N": round(last("board"), 4), "board_fz_N": round(board_fz(), 4),
                "bench_N": round(last("bench"), 4),
                "robot_object_contact_N": round(last("robot_object"), 4),
                "hole_x_mm": round(H[0] * MM, 2), "hole_y_mm": round(H[1] * MM, 2),
                "hole_z_mm": round(H[2] * MM, 2), "tilt_deg": round(tilt, 2),
                "hole_err_mm": round(herr * MM, 3), "pierce_off_mm": round(off * MM, 2),
                "pierce_x_mm": round(px * MM, 1), "tcp_err_mm": round(err * MM, 2),
                "path_u_mm": round(hx.u * MM, 2)}

    stop_t = _env("HW_STOP_T", 0.0)
    try:
        rec = rig.run(stop_t if stop_t > 0 else T_BUDGET, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{NAME}] expert finished at t={len(rec.times) / rig.fps:.2f}s")
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "board_N", "bench_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    if stop_t > 0 or not pm.finished:
        print("stopped early: QA skipped")
        return

    # ── QA ──────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    L = log.arr()
    HP.pen_plate_board(qa, plate, "wrench", *rig.meshes["board"], hook, "wrench<->board",
                       tol=0.0010)
    zmin = float(L["low_z"].min() - HP.Z0)
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["wrench->bench_top"] = {"min_vertex_z_above_top_mm": round(zmin * MM, 3),
                                        "pass": bool(zmin > -0.0010)}
    pen["pass"] = bool(pen["pass"] and zmin > -0.0010)
    qa.check_envelope(["wrench"], [-0.20, -0.25, HP.Z0 - 0.01], [0.20, 0.25, HP.Z0 + 0.40])
    crit = HP.common_criteria(qa, rig, plate, "wrench", hook, L, pm.t_start, rig.fps,
                              CFG.t_still)
    hang_m = (L["t"] >= pm.t_start["thread"]) & (L["t"] < pm.t_start["open"])
    qa.report["checks"]["expert_tracking"] = {
        "pass": True, "info_only": True,
        "peak_joint_cmd_step_rad": round(trk.dq_peak, 4),
        "hole_err_peak_thread_to_seat_mm": round(float(L["hole_err"][hang_m].max()) * MM, 2),
        "board_contact_peak_thread_to_seat_N": round(float(L["board"][hang_m].max()), 3),
        "tilt_at_release_deg": round(float(L["tilt"][L["t"] <= pm.t_start["open"]][-1]), 2),
        "hole_corr_final_mm": np.round(hx.corr * MM, 2).tolist(),
        "phase_start_s": {k: round(v, 2) for k, v in pm.t_start.items()}}
    for k_, c_ in crit.items():
        qa.report["checks"].setdefault(k_, c_)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    save_appendix_verdict(
        rig, task="Hang object (wrench)",
        source="new task (no suite counterpart); hanging board as in "
               "force_bench.tasks.keychain_hang",
        spec="The robot movement stops after placing the target object onto a hanging "
             "hook. The robot should not remain in contact with the task objects.",
        criteria=crit, physics_qa=report,
        changes=[
            "starts with the wrench lying on the bench (appendix: in hand) -- picked up "
            "by a fingertip pinch across the handle at its balance point",
            "kit wrench (assets/bodies_kit/wrench, same solid as sim "
            "unidentified/part1), PLA 13.5 g; board = sim hanging_board (= kit STEP); "
            "middle hook",
            "hang: thread the 30-deg-tilted hole down the 60 deg stub, follow the hook "
            "round its bend onto the flat bar (tilt -> 86 deg), seat (force-gated), "
            "open, retreat, hold still",
        ])


if __name__ == "__main__":
    wp.init()
    main()
