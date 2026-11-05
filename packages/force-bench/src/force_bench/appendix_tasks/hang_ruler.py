"""Appendix task "Hang object" (Table I, #9), RULER variant: the FR3 slides the
kit ruler to the bench edge, picks it up by its overhanging end and hangs it
by its 10 mm hole on the middle hook of the hanging board.

    Success (appendix): "The robot movement stops after placing the target
    object onto a hanging hook. The robot should not remain in contact with
    the task objects."

The appendix starts with the ruler in hand; here it starts lying flat on the
bench (user request).  A 2 mm ruler lying flat cannot be pinched from the
bench: its long edges are 2 mm tall and the bench is where the pad tips would
have to go.  So, as in ``force_bench.tasks.slide_off_edge`` (whose press-drag
constants are reused): the closed fingers press on its face and DRAG it
toward the bench's front edge until its plain end overhangs by ~30 mm; then
the pads come down OUTSIDE the edge, either side of the overhanging end
(tips 4 mm below the bench top, in free air), and pinch its two long edges.
A face pinch (pads above and below, slide_off_edge's side grasp) was not
used: at the release the lower pad would sit under the ruler and follow it
down as it swings onto the hook; with the pads either side of it, the ruler
swings away between them.

Scene, hook geometry and the guided hang: ``_hang_plate.py``.  The ruler is
gripped 115 mm from its hole, so the hand would meet the bench below a
~70 deg release tilt (offline FK: 35 mm clear at 70, 3 mm at 86); it lets go
at 62 deg -- a 2 mm plate with this hole may sit on the bar anywhere from 32
to 148 deg -- and the ruler swings the rest of the way to vertical.

COLLIDERS: the visible kit mesh (hole, rounded end, engraving) is the
collider against the BOARD only; an invisible, massless analytic box over the
straight part (124.5 x 25 x 2 mm) is the collider against the bench and the
fingertips (card_pick / slide_off_edge key pattern: a 2 mm slab against a
flat bench through a sampled SDF chatters; box-box gives a clean manifold).
The two never compete: mesh-vs-bench/robot and box-vs-board are filtered.

Run:  python -m force_bench.appendix_tasks.hang_ruler
      (TB_OUT_ROOT=<dir>; TB_RENDER_FAST=1 TB_MULTIVIEW=0; HR_STOP_T=<s>;
       HR_PREVIEW=<states.npz> renders a camera contact sheet)
"""

from __future__ import annotations

import os

import numpy as np
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv, world_T
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, save_appendix_verdict, StopRun)
from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._hang_plate import (
    MM, Plate, HookPath, HangCfg, HangExpert, HANG_PHASES, PhaseMachine, DroopComp,
    JointTracker, PairProbe, RunLog, hang_frame, tcp_pose_for, smooth5, pierce, quat_of_R)
from force_bench.tasks import slide_off_edge as SOE

NAME = "hang_ruler"


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


PEG_Y = HP.PEG_YS[1]
DENSITY = 1240.0                     # PLA -> 7.9 g
RULER_MU = 0.25                      # pair mu: pad 0.825 / bench 0.25 / board 0.375
SUBDIV = 0.0030
SDF_R = dict(sdf=True, sdf_resolution=512, narrow_band=0.0012, margin=0.0004)
GAP_R = 0.0003
IMPRATIO = _env("HR_IMPRATIO", 10.0)  # see hang_wrench
HAND_FLIP = True                     # see hang_wrench (offline IK, whole path)
STRAIGHT_END_KIT_X = 0.05836         # kit x where the rounded hole end begins
                                     # (semicircle r 12.5 about x = 58.36 mm)

EDGE_X = HP.BENCH_X[1]
START_INSET = _env("HR_INSET", 0.025)  # plain end this far inside the edge
START_Y = 0.0
REST_CLEAR = 0.00005
OH_TARGET = _env("HR_OVERHANG", 0.030)  # overhang to stop the drag at

# ── the press-drag (slide_off_edge's card values unless noted) ─────────
F_PRESS_LO, F_PRESS_HI = 2.0, 4.0    # N summed over both pads
PRESS_WIN = SOE.PRESS_WIN
PRESS_RATE = 0.000015                # m/frame
PRESS_MAX, PRESS_MIN = SOE.PRESS_MAX, SOE.PRESS_MIN
PRESS_START = -0.0020
SLIDE_RATE = 0.00028                 # m/frame (17 mm/s), card value
PUSH_BACK = 0.015                    # blade centre this far behind the ruler centre
COAST_S, COAST_WIN, COAST_MAX = 0.06, 10, 0.002
F_PRESS_OFF, PUSH_STILL = 0.6, 0.0008
UP_BREAK, UP_SPLIT, UP_BACK = 0.020, 0.25, 0.010
SAFE_Z = 0.060

# ── the end pinch ───────────────────────────────────────────────────────
TIP_BELOW = 0.004                    # pad tips this far below the bench top
EDGE_MARGIN = 0.0025                 # fingertip block clear of the edge face
END_MARGIN = 0.0010                  # pad inside the ruler's end
SQUEEZE = _env("HR_SQUEEZE", 0.0006)
OPEN_EXTRA = 0.0070
LIFT_Z = 0.080

CFG = HangCfg(tilt_hi=np.radians(_env("HR_TILT_DEG", 62.0)),
              bar_x=_env("HR_BAR_X", -0.1070), retreat_n=0.050, retreat_up=0.050)

T_SETTLE, T_DOWN, T_PUSH_MAX, T_UNLOAD_MAX, T_UP = 0.6, 0.8, 8.0, 1.4, 1.1
T_TURN, T_DESC, T_PRE, T_CLOSE, T_HOLD, T_LIFT, T_CARRY = 1.6, 1.0, 0.3, 0.6, 0.3, 1.0, 3.0
PICK_PHASES = ["settle", "down", "push", "unload", "up", "turn", "descend", "pre", "close",
               "hold", "lift", "carry"]
PICK_DUR = {"settle": T_SETTLE, "down": T_DOWN, "up": T_UP, "turn": T_TURN,
            "descend": T_DESC, "pre": T_PRE, "close": T_CLOSE, "hold": T_HOLD,
            "lift": T_LIFT, "carry": T_CARRY}
T_BUDGET = 50.0


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

    # ── the ruler: visible kit mesh (board collider) + analytic box ────
    raw = HP.load_kit_mesh("ruler")
    plate = Plate.measure("ruler", raw, DENSITY)
    lo_k, hi_k = raw.bounds
    L_end = float(plate.hole[0] - lo_k[0])          # hole -> plain end, along d
    half_w = 0.5 * float(hi_k[1] - lo_k[1])
    tm = subdiv(raw, SUBDIV)
    plate.tm = tm
    mesh, c = make_newton_mesh(tm, **SDF_R)
    plate.center = np.asarray(c, float)
    print(f"ruler: {plate.mass*1000:.2f} g, {plate.thick*MM:.2f} mm thick, {2*half_w*MM:.1f} "
          f"mm wide, hole r {plate.r_hole*MM:.3f} mm, plain end {L_end*MM:.1f} mm from the "
          f"hole, {len(tm.faces)} faces")

    d0 = np.array([1.0, 0.0, 0.0])                   # hole end toward the board (-x)
    n0 = np.array([0.0, 0.0, 1.0])
    x_end0 = EDGE_X - START_INSET
    H0 = np.array([x_end0 - L_end, START_Y, HP.Z0 + 0.5 * plate.thick + REST_CLEAR])
    p0, q0 = plate.body_pose(H0, n0, d0)
    body = rig.builder.add_body(xform=wp.transform(wp.vec3(*p0), wp.quat(*q0)), label="ruler")
    s_mesh = rig.builder.add_shape_mesh(
        body, mesh=mesh, cfg=rig.cfg(mu=RULER_MU, gap=GAP_R, mu_torsional=HP.MU_TORSION),
        label="ruler_shape")
    # analytic slab over the straight part, in the body frame
    bx0 = float(lo_k[0] - plate.center[0])
    bx1 = float(STRAIGHT_END_KIT_X - plate.center[0])
    box_c = np.array([0.5 * (bx0 + bx1), float(0.5 * (lo_k[1] + hi_k[1]) - plate.center[1]),
                      float(plate.z_mid - plate.center[2])])
    s_box = rig.builder.add_shape_box(
        body, xform=wp.transform(wp.vec3(*box_c), wp.quat_identity()),
        hx=0.5 * (bx1 - bx0), hy=half_w, hz=0.5 * plate.thick,
        cfg=rig.cfg(mu=RULER_MU, gap=GAP_R, density=0.0, is_visible=False,
                    mu_torsional=HP.MU_TORSION),
        label="ruler_slab")
    rig.shapes["ruler"] = [s_mesh, s_box]
    set_body_mass(rig.builder, body, plate.mass)
    rig.meshes["ruler"] = (tm, plate.center)
    HP.filter_arm_links(rig, "ruler")
    # collider split: mesh <-> board only, slab <-> bench + fingertips only
    rig.filter_pair(s_box, rig.shapes["board"][0])
    rig.filter_pair(s_mesh, rig.shapes["table"][0])
    for b, lbl in enumerate(rig.builder.body_label):
        if "finger" in str(lbl):
            for s in rig.builder.body_shapes.get(b, []):
                rig.filter_pair(s, s_mesh)
    HP.filter_statics_and_ground(rig)

    hook = HookPath(PEG_Y)
    poly = hook.polyline()
    f_touch = half_w + HP.PAD_FACE_OFF
    f_close = f_touch - SQUEEZE
    f_open = f_touch + OPEN_EXTRA
    h_pinch = (HP.Z0 - TIP_BELOW + HP.TIP_BELOW_TCP) - (HP.Z0 + 0.5 * plate.thick)
    grip = {"L": None}                               # measured at the pinch

    def tcp_for(H, theta=None, n=None, d=None):
        if theta is not None:
            n, d = hang_frame(theta)
        return tcp_pose_for(H + grip["L"] * d, n, d, h_pinch, flip=HAND_FLIP)

    hx = HangExpert(CFG, hook, plate, tcp_for, f_close, f_open, rig.fps)

    # ── blade (closed fingers, closing axis along -x = the push) ───────
    zb = np.array([0.0, 0.0, -1.0])
    yb = np.array([-1.0, 0.0, 0.0])
    q_blade = quat_of_R(np.column_stack([np.cross(yb, zb), yb, zb]))
    top = HP.Z0 + plate.thick
    blade_touch_z = top + HP.TIP_BELOW_TCP
    ctr0 = x_end0 - 0.5 * float(hi_k[0] - lo_k[0])       # ruler centre (x)
    tcp_park = np.array([ctr0 - PUSH_BACK, START_Y, blade_touch_z + 0.030])
    q_start = rig.solve_arm_start(tcp_park, q_blade)
    rig.builder.joint_q[7:9] = [0.0, 0.0]
    rig.builder.joint_target_q[7:9] = [0.0, 0.0]
    T_off = HP.tcp_offset_from_link7(rig)
    print(f"arm start TCP err {np.linalg.norm(rig.ik.tcp_pose()[:3]-tcp_park)*MM:.2f} mm")

    cam_t = np.array([_env("HR_CAM_TX", 0.000), _env("HR_CAM_TY", 0.000),
                      HP.Z0 + _env("HR_CAM_TZ", 0.080)])
    cam_p = cam_t + np.array([_env("HR_CAM_OX", 0.30), _env("HR_CAM_OY", -0.55),
                              _env("HR_CAM_OZ", 0.28)])
    if os.environ.get("HR_PREVIEW"):
        rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
        HP.preview_cameras(
            rig, os.environ["HR_PREVIEW"],
            [float(v) for v in os.environ.get("HR_PREVIEW_T", "1,5,9,13,17,21").split(",")],
            HP.parse_cams(os.environ.get("HR_PREVIEW_CAMS", f"{','.join(map(str, cam_t))}:"
                                         f"{','.join(map(str, cam_p - cam_t))}")),
            os.environ.get("HR_PREVIEW_OUT", str(rig.out_dir / "camera_preview.png")))
        return
    rig.finalize(probes=["ruler"], cam_pos=cam_p, cam_target=cam_t)
    HP.set_camera_fov(rig, cam_p, cam_t, _env("HR_CAM_FOV", 40.0))
    rig.probes["padL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", ["ruler_slab"])
    rig.probes["padR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", ["ruler_slab"])
    rig.probes["board"] = PairProbe(rig.model, "ruler", ["board"])
    rig.probes["bench"] = PairProbe(rig.model, "ruler", ["table"])
    rig.probes["robot_object"] = RobotContactProbe(rig.model, ["ruler"])
    ri = rig.body_index("ruler")
    i7 = rig.body_index("fr3/fr3_link7")
    v_low = plate.local_mesh().vertices[::5]
    end_body = np.array([lo_k[0], 0.5 * (lo_k[1] + hi_k[1]), plate.z_mid]) - plate.center

    def ruler_end(bq_row):
        T = world_T(bq_row)
        return T[:3, :3] @ end_body + T[:3, 3]

    trk = JointTracker(rig, q_start)
    droop = DroopComp()
    st = {"p": tcp_park.copy(), "q": np.asarray(q_blade, float), "begun": False,
          "press": PRESS_START, "cmd_x": tcp_park[0], "oh_hist": [], "oh_stop": None,
          "hold_prev": None, "up_from": None, "pinch": None, "x_end_push0": None}

    def push_done(k):
        return st.get(f"{k}_done", False)

    pm = PhaseMachine(PICK_PHASES + HANG_PHASES, {**PICK_DUR, **hx.durations()},
                      lambda k: push_done(k) if k in ("push", "unload") else hx.done(k))
    log = RunLog()
    extra_log = []

    def last(name, key="force"):
        h = rig.probes[name].history
        return h[-1][key] if h else 0.0

    def board_fz():
        h = rig.probes["board"].history
        return h[-1]["vec"][2] if h else 0.0

    def pads_N():
        Lh = rig.probes["padL"].history[-PRESS_WIN:]
        Rh = rig.probes["padR"].history[-PRESS_WIN:]
        if not Lh:
            return 0.0
        return float(np.mean([a["force"] for a in Lh]) + np.mean([b["force"] for b in Rh]))

    def plan_pinch(bq):
        """Pinch pose from the MEASURED ruler after the drag."""
        x_end = float(ruler_end(bq[ri])[0])
        y_c = float(bq[ri, 1])
        lo_x = EDGE_X + EDGE_MARGIN + HP.PAD_HALF_W
        hi_x = x_end - END_MARGIN - HP.PAD_HALF_W
        x_t = 0.5 * (lo_x + hi_x)
        H, n, d = plate.frame_of(bq[ri])
        g = np.array([x_t, y_c, HP.Z0 + 0.5 * plate.thick])
        grip["L"] = float((g - H) @ d)
        tcp, q = tcp_pose_for(g, n0, d0, h_pinch, flip=HAND_FLIP)
        st["pinch"] = (tcp, q, x_t, x_end)
        print(f"pinch: ruler end {x_end*MM:.1f} mm ({(x_end-EDGE_X)*MM:.1f} mm over the edge), "
              f"pad x {(x_t-HP.PAD_HALF_W)*MM:.1f}..{(x_t+HP.PAD_HALF_W)*MM:.1f}, grip "
              f"{grip['L']*MM:.1f} mm from the hole (feasible span {(hi_x-lo_x)*MM:.1f} mm)")
        return tcp, q

    def control(t):
        pm.advance(t)
        if pm.finished:
            raise StopRun
        k, s = pm.k, pm.frac(t)
        bq = rig.s0.body_q.numpy()
        tcp_meas = HP.actual_tcp(bq, i7, T_off)
        x_end = float(ruler_end(bq[ri])[0])
        oh = x_end - EDGE_X
        fingers = 0.0
        q = q_blade
        if k == "settle":
            p = tcp_park
        elif k == "down":
            z = tcp_park[2] + (blade_touch_z - PRESS_START - tcp_park[2]) * smooth5(s)
            p = np.array([st["cmd_x"], START_Y, z])
        elif k == "push":
            if st["x_end_push0"] is None:
                st["x_end_push0"] = x_end
            F = pads_N()
            if F < F_PRESS_LO:
                st["press"] = min(PRESS_MAX, st["press"] + PRESS_RATE)
            elif F > F_PRESS_HI:
                st["press"] = max(PRESS_MIN, st["press"] - PRESS_RATE)
            st["oh_hist"] = (st["oh_hist"] + [oh])[-COAST_WIN:]
            h = st["oh_hist"]
            v = (h[-1] - h[0]) * rig.fps / max(1, len(h) - 1) if len(h) > 1 else 0.0
            coast = float(np.clip(v * COAST_S, 0.0, COAST_MAX))
            if oh + coast >= OH_TARGET or t - pm.t0 >= T_PUSH_MAX:
                st["push_done"] = True
                st["oh_stop"] = oh
                print(f"t={t:.2f}s drag done: overhang {oh*MM:.1f} mm, press offset "
                      f"{st['press']*MM:+.2f} mm at {F:.1f} N")
            elif F >= F_PRESS_LO:
                st["cmd_x"] += SLIDE_RATE
            p = np.array([st["cmd_x"], START_Y, blade_touch_z - st["press"]])
        elif k == "unload":
            # unload the press on a ramp until the ruler is still (slide_off_edge)
            v_end = 0.0 if st["hold_prev"] is None else abs(x_end - st["hold_prev"]) * rig.fps
            st["hold_prev"] = x_end
            st["press"] = max(PRESS_MIN, st["press"] - PRESS_RATE * 4)
            if (v_end < PUSH_STILL and pads_N() < F_PRESS_OFF and t - pm.t0 > 0.3) \
                    or t - pm.t0 >= T_UNLOAD_MAX:
                st["unload_done"] = True
            p = np.array([st["cmd_x"], START_Y, blade_touch_z - st["press"]])
        elif k == "up":
            z0 = blade_touch_z - st["press"]
            a = min(1.0, s / UP_SPLIT)
            b = smooth5(max(0.0, (s - UP_SPLIT) / (1.0 - UP_SPLIT)))
            z = z0 + UP_BREAK * a + (top + HP.TIP_BELOW_TCP + SAFE_Z - z0 - UP_BREAK) * b
            p = np.array([st["cmd_x"] - UP_BACK * b, START_Y, z])
            st["up_end"] = p.copy()
        elif k == "turn":
            if st["pinch"] is None:
                plan_pinch(bq)
            tcp_pin, q_pin = st["pinch"][:2]
            above = tcp_pin + np.array([0.0, 0.0, SAFE_Z])
            a = smooth5(s)
            p = st["up_end"] + (above - st["up_end"]) * a
            q = slerp(q_blade, q_pin, a)
            fingers = f_open * smooth5(min(1.0, 2.0 * s))
        else:
            tcp_pin, q_pin = st["pinch"][:2]
            above = tcp_pin + np.array([0.0, 0.0, SAFE_Z])
            lift = tcp_pin + np.array([0.0, 0.0, LIFT_Z])
            fingers = f_close
            q = q_pin
            if k == "descend":
                fingers = f_open
                p = above + (tcp_pin - above) * smooth5(s)
            elif k == "pre":
                fingers = f_open
                p = tcp_pin
            elif k == "close":
                fingers = f_open + (f_close - f_open) * smooth5(s)
                p = tcp_pin
            elif k == "hold":
                p = tcp_pin
            elif k == "lift":
                p = tcp_pin + (lift - tcp_pin) * smooth5(s)
            elif k == "carry":
                if "hover" not in st:
                    st["hover"] = hx.hover()
                tcp_hov, q_hov = st["hover"]
                a = smooth5(s)
                p = lift + (tcp_hov - lift) * a + np.array([0.0, 0.0, 0.03 * np.sin(np.pi * a)])
                q = slerp(q_pin, q_hov, a)
            else:
                if not st["begun"]:
                    hx.begin(droop.v)
                    st["begun"] = True
                    print(f"t={t:.2f}s droop {np.round(droop.v*MM, 2)} mm handed to the hole loop")
                H_meas, _, _ = plate.frame_of(bq[ri])
                p, q, fingers, _ = hx.step(k, s, H_meas, last("board"), board_fz(),
                                           st["p"], st["q"])
        if k in PICK_PHASES:
            # the sag is learned only on a static, contact-free command
            droop.update(k in ("settle", "pre"), p, tcp_meas, st["p"])
            p_out = p + droop.v
        else:
            p_out = p
        st["p"], st["q"] = np.asarray(p, float), np.asarray(q, float)
        trk.step(p_out, q, fingers)

    def extra(state, t):
        bq = state.body_q.numpy()
        H, n, _ = plate.frame_of(bq[ri])
        tilt = float(np.degrees(np.arctan2(n[0], n[2])))
        off, px = pierce(plate, bq[ri], poly)
        tcp_a = HP.actual_tcp(bq, i7, T_off)
        k = pm.k
        err = float(np.linalg.norm(tcp_a - st["p"]))
        herr = float(np.linalg.norm(H - hx.H_cmd)) if k in HANG_PHASES[:5] else 0.0
        T = world_T(bq[ri])
        low_z = float(((T[:3, :3] @ v_low.T).T + T[:3, 3])[:, 2].min())
        oh = float(ruler_end(bq[ri])[0]) - EDGE_X
        log.add(t=t, Hx=H[0], Hy=H[1], Hz=H[2], nx=n[0], ny=n[1], nz=n[2], tilt=tilt,
                pierce_off=off, pierce_x=px, padL=last("padL"), padR=last("padR"),
                board=last("board"), board_fz=board_fz(), bench=last("bench"),
                robot_obj=last("robot_object"), tcp_x=tcp_a[0], tcp_y=tcp_a[1],
                tcp_z=tcp_a[2], tcp_err=err, hole_err=herr, low_z=low_z)
        extra_log.append((t, oh, float(bq[ri, 1]), float(np.degrees(np.arctan2(T[1, 0], T[0, 0])))))
        if int(round(t * rig.fps)) % 15 == 0:
            print(f"  [{t:5.2f} {k:9s}] pads {last('padL'):5.2f}/{last('padR'):5.2f} N  board "
                  f"{last('board'):6.3f} (fz {board_fz():+.3f}) bench {last('bench'):5.3f}  "
                  f"overhang {oh*MM:6.1f} press {st['press']*MM:+.2f} hole {np.round(H*MM,1)} "
                  f"tilt {tilt:5.1f} herr {herr*MM:5.2f} u {hx.u*MM:6.2f} pierce "
                  f"{off*MM:5.2f}@{px*MM:6.1f} tcp_err {err*MM:5.2f} corr {np.round(hx.corr*MM, 2)}")
        return {"padL_N": round(last("padL"), 3), "padR_N": round(last("padR"), 3),
                "board_N": round(last("board"), 4), "board_fz_N": round(board_fz(), 4),
                "bench_N": round(last("bench"), 4),
                "robot_object_contact_N": round(last("robot_object"), 4),
                "overhang_mm": round(oh * MM, 2), "press_mm": round(st["press"] * MM, 3),
                "hole_x_mm": round(H[0] * MM, 2), "hole_y_mm": round(H[1] * MM, 2),
                "hole_z_mm": round(H[2] * MM, 2), "tilt_deg": round(tilt, 2),
                "hole_err_mm": round(herr * MM, 3), "pierce_off_mm": round(off * MM, 2),
                "pierce_x_mm": round(px * MM, 1), "tcp_err_mm": round(err * MM, 2),
                "path_u_mm": round(hx.u * MM, 2)}

    stop_t = _env("HR_STOP_T", 0.0)
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
    E = np.array(extra_log)
    HP.pen_plate_board(qa, plate, "ruler", *rig.meshes["board"], hook, "ruler<->board",
                       tol=0.0010)
    zmin = float(L["low_z"].min() - HP.Z0)
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["ruler->bench_top"] = {"min_vertex_z_above_top_mm": round(zmin * MM, 3),
                                       "pass": bool(zmin > -0.0010)}
    pen["pass"] = bool(pen["pass"] and zmin > -0.0010)
    qa.check_envelope(["ruler"], [-0.20, -0.25, HP.Z0 - 0.01], [0.25, 0.25, HP.Z0 + 0.40])
    tph = pm.t_start
    crit = HP.common_criteria(qa, rig, plate, "ruler", hook, L, tph, rig.fps, CFG.t_still)
    # the slide: started fully on the bench, dragged to the measured overhang,
    # never tipped (bench still carries it until the pinch lifts it)
    t_e = E[:, 0]
    before_pinch = (t_e >= tph["unload"]) & (t_e < tph["close"])
    bench_tt = L["bench"][(L["t"] >= tph["unload"]) & (L["t"] < tph["close"])]
    yaw_drift = float(np.abs(E[(t_e < tph["close"]), 3] - E[0, 3]).max())
    crit["slid_to_the_edge"] = {
        "pass": bool(E[0, 1] < 0 and st["oh_stop"] is not None and st["oh_stop"] > 0.022
                     and (E[before_pinch, 1] < 0.065).all()
                     and abs(bench_tt.mean() - 9.81 * plate.mass) < 0.3 * 9.81 * plate.mass),
        "start_overhang_mm": round(float(E[0, 1]) * MM, 1),
        "overhang_at_drag_stop_mm": round(float(st["oh_stop"] or 0) * MM, 1),
        "overhang_at_pinch_mm": round(float(st["pinch"][3] - EDGE_X) * MM, 1) if st["pinch"] else None,
        "drag_distance_mm": round(float(E[before_pinch, 1][-1] - E[0, 1]) * MM, 1),
        "bench_support_before_pinch_N": round(float(bench_tt.mean()), 4),
        "yaw_drift_deg": round(yaw_drift, 2),
        "note": "the ruler starts fully on the bench; the closed fingers press its face "
                "(force-servoed 2-4 N) and drag it until its plain end overhangs; it is "
                "still carried by the bench (no tipping) until the pads close"}
    hang_m = (L["t"] >= tph["thread"]) & (L["t"] < tph["open"])
    qa.report["checks"]["expert_tracking"] = {
        "pass": True, "info_only": True,
        "peak_joint_cmd_step_rad": round(trk.dq_peak, 4),
        "hole_err_peak_thread_to_seat_mm": round(float(L["hole_err"][hang_m].max()) * MM, 2),
        "board_contact_peak_thread_to_seat_N": round(float(L["board"][hang_m].max()), 3),
        "tilt_at_release_deg": round(float(L["tilt"][L["t"] <= tph["open"]][-1]), 2),
        "grip_from_hole_mm": round(grip["L"] * MM, 1),
        "hole_corr_final_mm": np.round(hx.corr * MM, 2).tolist(),
        "phase_start_s": {k: round(v, 2) for k, v in tph.items()}}
    for k_, c_ in crit.items():
        qa.report["checks"].setdefault(k_, c_)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    save_appendix_verdict(
        rig, task="Hang object (ruler)",
        source="new task (no suite counterpart); slide-to-edge press-drag from "
               "force_bench.tasks.slide_off_edge, hanging board as in keychain_hang",
        spec="The robot movement stops after placing the target object onto a hanging "
             "hook. The robot should not remain in contact with the task objects.",
        criteria=crit, physics_qa=report,
        changes=[
            "starts with the ruler lying flat on the bench 25 mm inside its edge "
            "(appendix: in hand); slid to the edge by a force-servoed press-drag, then "
            "pinched by its two long edges at the overhanging plain end",
            "kit ruler (assets/bodies_kit/ruler; no sim asset existed), PLA 7.9 g; "
            "mesh collider vs the board, massless analytic slab vs bench and pads",
            "hang: thread the 30-deg-tilted hole down the 60 deg stub, follow the hook "
            "onto the flat bar (tilt -> 62 deg), seat (force-gated), open, retreat, "
            "hold still; the ruler swings to vertical on the bar by itself",
        ])


if __name__ == "__main__":
    wp.init()
    main()
