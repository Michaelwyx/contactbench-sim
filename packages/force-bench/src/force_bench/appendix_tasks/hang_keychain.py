"""Appendix task "Hang object" (Table I, #9), KEYCHAIN variant, BARE fingers.

    Success (appendix): "The robot movement stops after placing the target
    object onto a hanging hook. The robot should not remain in contact with
    the task objects."

A fork of ``force_bench.tasks.keychain_hang`` (unchanged on disk): the scene
recipe (bench, board, wrench, ring + two contact-linked keys at the baked
pile rest, FR3 with rubberized pads, collision filters, probes) is
replicated line for line and every constant / helper is imported from it.
The pick, lift and carry are the suite expert's, unchanged -- under MuJoCo
the bare carry is solid (pads 4.0-4.2 N, ring attitude unchanged, |n_x|
0.32 throughout; the header's "ring rolls out in the carry" is XPBD-era).
Everything from the hover over the hook onward is new, because that is
where the bare suite task fails.

WHY THE SUITE'S RELEASE FAILS (two bare runs, 2026-09-11 and 2026-10-15 with
the current code; identical).  The suite opens the pads with the ring
19.5 mm ABOVE the hook so it "free-falls onto the hook tip".  The pinch is
on the ring's high side with the pads closing along the ring normal (27 deg
off vertical), so the lower pad is a shelf: relaxing the squeeze lets the
ring pivot about it, the keys swing into the peg (0.15-0.43 N, board kick
0.41 N), the ring is thrown 5 mm in -y, the peg never pierces the hole
(20-26 mm off-centre at the hook's height) and the keychain falls 173 mm to
the bench.  (The 09-11 run's "peak slip 144 mm" is that fall, measured
after the release.)  The GelSight variant's pass was the same mechanism
with luck: the ring stayed on the soft lower pad, the retreat lifted it and
it flipped onto the bar.

THE HANG THAT WORKS -- PLACE, THEN LET THE BAR TAKE IT.
  1. Same hover; lateral move at the hover height; 0.6 s dwell.
  2. DESCEND WITH THE RING STILL PINCHED (8 mm/s) to ring-centre z 165.5 mm
     at (-89.0, -81.5): the hook's crest is now inside the 21 mm hole and
     the ring's far (low) side hangs 2 mm above the bar's root.  Measured:
     no ring load and at most a 0.04 N key graze on the board during the
     descent (one key's blade hangs flat beside the hook's -y face).
  3. Hold 0.3 s, relax the squeeze to the just-touching gap (0.5 s).
  4. RELEASE = open AND pull.  The fingers open at 0.05 m/s per finger
     (the Franka Hand's 0.1 m/s), and at the same time the arm slides the
     hand 20 mm straight back along the horizontal pinch ray in 0.15 s.
     The ring's far side lands on the bar, its near side loses the lower
     pad at ARM speed, and the ring swings about the bar contact into the
     hang (bar through the hole, |n_x| 0.6-1.0, centre z 147.6 vs 147.5
     geometric).  WHY THE SPEED MATTERS (A/B, all else equal): while the
     lower pad still carries the near rim, the ring sits on two supports
     (bar edge + pad) and the keys' weight, 5.6 mm off the line between
     them, rolls it edge-on to the peg (|n_x| -> 0.04) and it slides off
     the bar's -y edge -- a 1.0 s smoothstep open and a 0.05 m/s/finger
     open alone both lost it; a 0.12 s finger snap (0.29 m/s per finger,
     6x the hand's speed) and the pull at the hand's real speed both hang.
  5. RETREAT radially out, up and sideways inside the arm's IK-feasible set
     (the FR3 has q1 on its 2.74 rad limit here; the suite's +x retreat
     leg is unreachable, 18-32 mm IK residual), then hold STILL.

Gates (rollout_qa.json): the suite's (finite, post-settle velocity,
key<->ring and keychain<->board/rod penetration, smoothness, task_success
= hang + keys linked + free swing + real grip) plus released_and_clear
(robot<->{ring, keys, board, rod} < 0.05 N over the still hold),
robot_moved_away and robot_still.  appendix_qa.json restates them against
the appendix criterion.  The static penetration check is the suite's with an
exact bounding-box prefilter (identical numbers, ~20 s instead of ~19 min).

Every knob is an ``HK_*`` environment variable so an A/B run needs no edit;
the defaults are the accepted settings.

Run:   python -m force_bench.appendix_tasks.hang_keychain
       (TB_OUT_ROOT=<dir> to keep iterations out of outputs/,
        TB_RENDER_FAST=1 TB_MULTIVIEW=0 while iterating,
        HK_STOP_T=<s> to stop early and skip the QA)
"""

from __future__ import annotations

import os
import time

import numpy as np
import trimesh
import warp as wp

import newton

if os.environ.get("TB_GELSIGHT", "0") == "1":
    raise SystemExit("hang_keychain is the BARE-gripper variant: unset TB_GELSIGHT")

import force_bench.tactile.task as GS                       # identity when bare
import force_bench.tasks.keychain_hang as KH
from force_bench.tasks.keychain_hang import (
    PEG_Y, PEG_X0, PEG_X1, R_TUBE, R_ID, HANG_Z, BAKED, BENCH_Z, BENCH_T,
    BENCH_A, BENCH_B, RING_MASS, KEY_MASS, W_TOTAL, KC_MU, ROD_MU, BOARD_MU,
    TCP_R, FINGER_KE, FINGER_KD, F_GRIP_DB, GRIP_STEP, FINGER_HOME,
    FINGER_OPEN, FINGER_TOUCH, FINGER_CLOSE, FINGER_TIGHT, FINGER_MIN,
    FINGER_MU, SUBDIV_KC, SUBDIV_ROD, SDF_KC, SDF_STAT, GAP_KC, T_GRAV,
    T_SETTLE, T_APPR, T_CAGE, T_CLOSE, T_TIGHT, T_PEEL, T_LIFT, T_TRANS,
    T_DWELL, T_HOVER0, T_HOVER, T_THREAD, DROOP_GAIN, DROOP_STEP, DROOP_MAX,
    CAGE_STAND, CORR_GAIN, CORR_STEP, CORR_ALPHA, CORR_DB,
    ARM_KD_TASK, DQ_MAX, DDQ_MAX, quat_of_R, wp_quat, R_of_q, smooth,
    gauss_link, key_geometry, PairProbe)
from force_bench.probes import ForceProbe
from force_bench.rig import make_solver
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv, world_T
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, run_until, check_released_and_clear,
    save_appendix_verdict)

assert not GS.GS_ON
MM = 1000.0
NAME = "hang_keychain"


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


# ── the new threading / release / retreat (all overridable) ─────────────
THREAD_X = _env("HK_THREAD_X", -0.0890)     # ring-centre target over the hook
THREAD_Y = _env("HK_THREAD_Y", -0.0815)
HOVER_Z = _env("HK_HOVER_Z", 0.1850)        # lateral approach height (suite)
T_LAT = T_THREAD - T_HOVER                  # 0.8 s lateral leg (suite)
T_DWELL2 = _env("HK_T_DWELL2", 0.6)         # keys settle above the hook
PLACE_Z = _env("HK_PLACE_Z", 0.1655)        # descent end (ring centre)
V_DESC = _env("HK_V_DESC", 0.008)           # m/s
F_RING_STOP = _env("HK_F_RING_STOP", 0.04)  # ring<->board N that ends it
T_HOLD = _env("HK_T_HOLD", 0.3)
T_RELAX = _env("HK_T_RELAX", 0.5)
# THE RELEASE.  The fingers open on a trapezoidal profile at OPEN_SPEED per
# finger (0.05 m/s = 0.1 m/s of width, the Franka Hand's speed) with
# OPEN_ACC s ramps, to FINGER_WIDE, WHILE the arm pulls the hand straight
# back along the horizontal pinch ray (PULL below).  HK_OPEN_SPEED=0 falls
# back to a smoothstep open of T_OPEN s (the measured A/B: 1.0 s fails, a
# 0.12 s "snap" -- 0.29 m/s per finger, 6x the hand's speed -- works).
OPEN_SPEED = _env("HK_OPEN_SPEED", 0.05)
OPEN_ACC = _env("HK_OPEN_ACC", 0.03)
T_OPEN = _env("HK_T_OPEN", 1.0)
FINGER_WIDE = _env("HK_FINGER_WIDE", 0.020)


def _trap(t, dist, v, ta):
    """Distance travelled at time t on a trapezoidal profile (cruise v,
    accel/decel time ta) that covers `dist`."""
    T = dist / v + ta
    t = float(np.clip(t, 0.0, T))
    a = v / ta
    if t < ta:
        x = 0.5 * a * t * t
    elif t <= T - ta:
        x = v * (t - 0.5 * ta)
    else:
        x = dist - 0.5 * a * (T - t) ** 2
    return float(np.clip(x, 0.0, dist))


T_OPEN_EFF = ((FINGER_WIDE - FINGER_TOUCH) / OPEN_SPEED + OPEN_ACC
              if OPEN_SPEED > 0 else T_OPEN)
# PULL release (HK_PULL > 0): while the fingers open, the hand also slides
# straight back along the horizontal pinch ray by HK_PULL metres, starting
# HK_PULL_T0 s into the open and taking HK_T_PULL s, so the lower pad leaves
# the rim at arm speed rather than at gripper speed.
PULL = _env("HK_PULL", 0.020)
PULL_T0 = _env("HK_PULL_T0", 0.03)
T_PULL = _env("HK_T_PULL", 0.15)
PULL_UP = _env("HK_PULL_UP", 0.0)
T_WAIT = _env("HK_T_WAIT", 0.8)
# The FR3 works at the edge of its envelope here (q1 sits on its 2.74 rad
# limit at the hook), so the retreat is chosen inside the IK-feasible set
# at the grasp attitude (residual < 0.01 mm, scanned offline): radially out
# along the pinch ray, up, and sideways along the ring tangent.  A +x leg
# toward the robot (the suite's retreat) is infeasible here: 18-32 mm IK
# residual, i.e. the arm strains against its limits and never arrives.
RET_OUT = _env("HK_RET_OUT", 0.030)         # radial (after the 20 mm pull)
RET_UP = _env("HK_RET_UP", 0.040)
RET_SIDE = _env("HK_RET_SIDE", 0.020)       # along the ring tangent (-x+y)
T_RET = _env("HK_T_RET", 2.4)
T_STILL = _env("HK_T_STILL", 1.6)
T_MAX = 40.0
# grip / solver / cadence
F_GRIP_TARGET = _env("HK_F_GRIP", KH.F_GRIP_TARGET)
IMPRATIO = _env("HK_IMPRATIO", 0.0)         # 0 = rig default (1.0)
CADENCE = os.environ.get("HK_CADENCE", "thread")   # suite | thread | always
STOP_T = _env("HK_STOP_T", 0.0)             # >0: stop early, skip the QA
# DEVELOPMENT AID ONLY (never used for a published rollout): the pick and
# carry are identical in every experiment on the release, so a run can save
# the full simulation + controller state at HK_CKPT_T and later runs restart
# from it.  SolverMuJoCo re-reads joint_q/joint_qd from the Newton state on
# every step, so the restored state is the physical state; only the solver's
# warm start is lost.  A restarted rollout does not start at t = 0, so it
# never gets the QA or the appendix verdict.
CKPT_SAVE = os.environ.get("HK_CKPT_SAVE", "")
CKPT_LOAD = os.environ.get("HK_CKPT_LOAD", "")
CKPT_T = _env("HK_CKPT_T", 14.0)
# primary camera: target and offset (m); HK_CAM="tx,ty,tz:ox,oy,oz"
# chosen on contact sheets of a passing run: pile + pick (lower right), the
# carry, the pinch over the hook and the hang (left) are all in frame and
# the arm comes in from the upper right without covering the hook
CAM_TARGET = np.array([-0.075, 0.005, 0.110])
CAM_OFFSET = np.array([0.160, -0.300, 0.120])
if os.environ.get("HK_CAM"):
    _a, _b = os.environ["HK_CAM"].split(":")
    CAM_TARGET = np.array([float(v) for v in _a.split(",")])
    CAM_OFFSET = np.array([float(v) for v in _b.split(",")])

PHASES = ["carry", "lateral", "dwell", "descend", "hold", "relax", "open",
          "wait", "retreat", "still", "done"]


def robot_keychain_clearance(rig, t_arr, bq_all, every=6, near=0.010):
    """Geometric robot<->keychain check on the RECORDED poses.

    The scene filters every non-finger FR3 link against the keychain (only
    the fingertips collide, inherited from the suite), so no solver contact
    probe can see an arm link pass through a key.  This samples every
    `every`-th frame, puts 600 vertices of each keychain body into each FR3
    collision shape's frame (meshes and boxes, exactly as the model holds
    them) and returns the largest signed distance (+ = inside, m) per
    (link, body) pair that came within `near`.
    """
    m = rig.model
    labels = list(m.body_label)
    sb, sty = m.shape_body.numpy(), m.shape_type.numpy()
    sx, ssc = m.shape_transform.numpy(), m.shape_scale.numpy()
    robot = {i for i, l in enumerate(labels) if str(l).startswith("fr3/")}
    shapes = []
    for s_ in range(len(sb)):
        if int(sb[s_]) not in robot:
            continue
        ty = int(sty[s_])
        if ty in (int(newton.GeoType.MESH), int(newton.GeoType.CONVEX_MESH)):
            src = m.shape_source[s_]
            tm = trimesh.Trimesh(np.asarray(src.vertices) * ssc[s_],
                                 np.asarray(src.indices).reshape(-1, 3), process=False)
        elif ty == int(newton.GeoType.BOX):
            tm = trimesh.creation.box(extents=2 * np.asarray(ssc[s_]))
        else:
            continue
        tm.apply_transform(world_T(np.r_[sx[s_][:3], sx[s_][3:7]]))
        shapes.append((int(sb[s_]), tm, trimesh.proximity.ProximityQuery(tm)))
    kc = {}
    for nm in ("ring", "key_a", "key_b"):
        tm, c = rig.meshes[nm]
        v = np.asarray(tm.vertices) - np.asarray(c)
        kc[nm] = v[np.random.RandomState(0).choice(len(v), min(600, len(v)), replace=False)]
    worst = {}
    for f in range(0, len(t_arr), every):
        for nm, v in kc.items():
            Tk = world_T(bq_all[f, labels.index(nm)])
            P = (Tk[:3, :3] @ v.T).T + Tk[:3, 3]
            for b, tm, pq in shapes:
                Tb = world_T(bq_all[f, b])
                Pl = (P - Tb[:3, 3]) @ Tb[:3, :3]
                mm = ((Pl > tm.bounds[0] - near) & (Pl < tm.bounds[1] + near)).all(1)
                if not mm.any():
                    continue
                val = float(pq.signed_distance(Pl[mm]).max())
                key = f"{labels[b]}<->{nm}"
                if val > -near and (key not in worst or val > worst[key][0]):
                    worst[key] = (val, float(t_arr[f]))
    return worst


def pen_static_fast(qa, body_label, body_local, static_tm, static_center,
                    pair_name, tol=0.0015, n_samples=24):
    """``RolloutQA.check_penetration_static`` with an EXACT prefilter.

    Same 800-vertex subsample (RandomState(0)), same frames, same verdict
    and report fields; vertices outside the static mesh's bounding box are
    skipped because their signed distance is negative (the board and rod
    meshes are closed) and the check only ever takes a maximum that starts
    at 0.  trimesh's closest-point search on far-away points is what made
    the suite's version take ~19 minutes on this scene.
    """
    static_local = static_tm.copy()
    static_local.apply_translation(-np.asarray(static_center))
    lo, hi = static_local.bounds[0] - 1e-6, static_local.bounds[1] + 1e-6
    pq = trimesh.proximity.ProximityQuery(static_local)
    bi = qa._idx(body_label)
    va = body_local.vertices
    if len(va) > 800:
        va = va[np.random.RandomState(0).choice(len(va), 800, replace=False)]
    va_h = np.hstack([va, np.ones((len(va), 1))])
    frames = np.linspace(0, len(qa.t) - 1, n_samples).astype(int)
    worst, worst_t = 0.0, None
    for f in frames:
        q = qa.body_q[f, bi]
        T = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
        T[:3, 3] = q[:3] - np.asarray(static_center)
        pts = (T @ va_h.T).T[:, :3]
        pts = pts[((pts > lo) & (pts < hi)).all(axis=1)]
        sd = pq.signed_distance(pts) if len(pts) else np.zeros(0)
        depth = float(sd.max()) if len(sd) else 0.0
        if depth > worst:
            worst, worst_t = depth, float(qa.t[f])
    entry = {"max_depth_mm": round(worst * 1000, 3), "at_t": worst_t,
             "pass": bool(worst < tol)}
    pen = qa.report["checks"].setdefault(
        "penetration", {"pass": True, "tolerance_mm": tol * 1000, "pairs": {}})
    pen["pairs"][pair_name] = entry
    pen["pass"] = bool(pen["pass"] and entry["pass"])
    qa.report["pass"] &= entry["pass"]


def main():
    t_wall0 = time.time()
    rig = appendix_rig(os.environ.get("HK_NAME", NAME))
    rig.substeps = 24
    rig.collide_every_substep = True
    if IMPRATIO > 0:
        rig.solver_options["impratio"] = IMPRATIO
    if CADENCE == "always":
        rig.collide_substep_override = True

    base_xf = wp.transform(wp.vec3(0.32, 0.0, 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    for dof in range(7):
        rig.builder.joint_target_kd[dof] = ARM_KD_TASK
    for dof in (7, 8):
        rig.builder.joint_target_ke[dof] = FINGER_KE
        rig.builder.joint_target_kd[dof] = FINGER_KD
    for bl, lbl in enumerate(rig.builder.body_label):
        if "finger" in lbl:
            for s in rig.builder.body_shapes.get(bl, []):
                rig.builder.shape_material_mu[s] = FINGER_MU

    def bench(box, label):
        x0, x1, y0, y1 = box
        sid = rig.builder.add_shape_box(
            -1, xform=wp.transform(
                wp.vec3((x0 + x1) / 2, (y0 + y1) / 2, BENCH_Z - BENCH_T),
                wp.quat_identity()),
            hx=(x1 - x0) / 2, hy=(y1 - y0) / 2, hz=BENCH_T,
            cfg=rig.cfg(), label=label)
        rig.shapes[label] = [sid]

    bench(BENCH_A, "table")
    bench(BENCH_B, "table_b")
    rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(0.32, 0.0, 0.006), wp.quat_identity()),
        hx=0.115, hy=0.115, hz=0.006,
        cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False),
        label="arm_pedestal")

    def add_static(asset, stem, label, max_edge, mu):
        tm = load_tm(asset, stem)
        if max_edge:
            tm = subdiv(tm, max_edge)
        mesh, c = make_newton_mesh(tm, **SDF_STAT)
        c = np.asarray(c)
        sid = rig.builder.add_shape_mesh(
            -1, mesh=mesh, xform=wp.transform(wp.vec3(*c), wp.quat_identity()),
            cfg=rig.cfg(mu=mu), label=label)
        rig.meshes[label] = (tm, c)
        rig.shapes[label] = [sid]
        return c

    add_static("hanging_board", "part1_1_body1", "board", None, BOARD_MU)
    add_static("unidentified", "part1_1_body1", "rod", SUBDIV_ROD, ROD_MU)
    statics = list(rig.builder.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            rig.filter_pair(statics[i], statics[j])

    cfg_kc = dict(cfg=rig.cfg(mu=KC_MU, gap=GAP_KC))
    ring_raw = load_tm("keyring", "part3_1_body1")
    ring_tm = subdiv(ring_raw, SUBDIV_KC)
    ring_mesh, ring_c = make_newton_mesh(ring_tm, **SDF_KC)
    ring_c = np.asarray(ring_c)
    p_ring0, q_ring0 = BAKED["ring"]
    ring_body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*p_ring0), wp_quat(q_ring0)), label="ring")
    rig.shapes["ring"] = [rig.builder.add_shape_mesh(
        ring_body, mesh=ring_mesh, label="ring_shape", **cfg_kc)]
    set_body_mass(rig.builder, ring_body, RING_MASS)
    ring_local = ring_tm.copy()
    ring_local.apply_translation(-ring_c)
    rig.meshes["ring"] = (ring_tm, ring_c)

    key_locals, key_holes = {}, {}
    for name, stem in (("key_a", "part1_1_body1"), ("key_b", "part3_1_body1")):
        raw = load_tm("key_1", stem)
        kn, khc, hole = key_geometry(raw)
        tm = subdiv(raw, SUBDIV_KC)
        mesh, c = make_newton_mesh(tm, **SDF_KC)
        c = np.asarray(c)
        pk, qk = BAKED[name]
        body = rig.builder.add_body(
            xform=wp.transform(wp.vec3(*pk), wp_quat(qk)), label=name)
        rig.shapes[name] = [rig.builder.add_shape_mesh(
            body, mesh=mesh, label=name + "_shape", **cfg_kc)]
        set_body_mass(rig.builder, body, KEY_MASS)
        loc = tm.copy()
        loc.apply_translation(-c)
        key_locals[name] = loc
        key_holes[name] = hole - c
        rig.meshes[name] = (tm, c)

    th_c = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    circle_local = np.stack([np.zeros_like(th_c), R_TUBE * np.cos(th_c),
                             R_TUBE * np.sin(th_c)], axis=1)

    ground_sid = statics[0]
    for b, sids in rig.builder.body_shapes.items():
        for s in sids:
            if s != ground_sid:
                rig.filter_pair(ground_sid, s)
    arm_bodies = [b for b, l in enumerate(rig.builder.body_label)
                  if l.startswith("fr3/")]
    keep_live = {rig.shapes["board"][0], rig.shapes["rod"][0]}
    for b in arm_bodies:
        for s in rig.builder.body_shapes.get(b, []):
            for sst in statics:
                if sst not in keep_live:
                    rig.filter_pair(s, sst)

    def tcp_quat(p_hat, n_hat):
        z_t = -p_hat
        y_t = n_hat / np.linalg.norm(n_hat)
        x_t = np.cross(y_t, z_t)
        return quat_of_R(np.column_stack([x_t, y_t, z_t]))

    R_ring0 = trimesh.transformations.quaternion_matrix(
        [q_ring0[3], q_ring0[0], q_ring0[1], q_ring0[2]])[:3, :3]
    RING_N = R_ring0 @ np.array([1.0, 0.0, 0.0])
    if RING_N[2] < 0:
        RING_N = -RING_N
    _arc = (R_ring0 @ np.stack(
        [np.zeros(360),
         R_TUBE * np.cos(np.linspace(0, 2 * np.pi, 360, endpoint=False)),
         R_TUBE * np.sin(np.linspace(0, 2 * np.pi, 360, endpoint=False))],
        axis=1).T).T + p_ring0
    P_TOP = _arc[int(np.argmax(_arc[:, 2]))]
    P_HAT_G = (P_TOP - p_ring0) / np.linalg.norm(P_TOP - p_ring0)
    Q_GRASP = tcp_quat(P_HAT_G, RING_N)
    R_HAT = np.array([P_HAT_G[0], P_HAT_G[1], 0.0])
    R_HAT /= np.linalg.norm(R_HAT)          # horizontal pinch ray (retreat)
    T_HAT = np.array([-R_HAT[1], R_HAT[0], 0.0])   # horizontal ring tangent
    print(f"grasp: ring normal {np.round(RING_N, 3)} pinch dir "
          f"{np.round(P_HAT_G, 3)}")

    tcp0 = p_ring0 + (TCP_R + CAGE_STAND + 0.008) * P_HAT_G \
        + np.array([0, 0, 0.045])
    q_start = rig.solve_arm_start(tcp0, Q_GRASP)
    tcp_fk = rig.ik.tcp_pose()
    _lf = rig.ik.body_pose("fr3/fr3_leftfinger")
    _rf = rig.ik.body_pose("fr3/fr3_rightfinger")
    FINGER_TCP_OFF = world_T(_lf)[:3, :3].T @ (
        tcp_fk[:3] - (_lf[:3] + _rf[:3]) / 2)

    for kb in (ring_body, *[rig.builder.body_label.index(n)
                            for n in ("key_a", "key_b")]):
        for ts in rig.builder.body_shapes.get(kb, []):
            for bl in arm_bodies:
                if "finger" in rig.builder.body_label[bl]:
                    continue
                for s in rig.builder.body_shapes.get(bl, []):
                    rig.builder.add_shape_collision_filter_pair(s, ts)

    cam_t = CAM_TARGET
    rig.finalize(probes=["ring", "key_a", "key_b",
                         "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"],
                 cam_pos=cam_t + CAM_OFFSET, cam_target=cam_t)
    rig.probes["fingertip_L"] = rig.probes.pop("fr3/fr3_leftfinger")
    rig.probes["fingertip_R"] = rig.probes.pop("fr3/fr3_rightfinger")
    _ring_shapes = rig.shapes["ring"]
    rig.probes["grip_L"] = PairProbe(rig.model, "fr3/fr3_leftfinger", _ring_shapes)
    rig.probes["grip_R"] = PairProbe(rig.model, "fr3/fr3_rightfinger", _ring_shapes)
    _env_shapes = rig.shapes["board"] + rig.shapes["rod"]
    rig.probes["ring_env"] = PairProbe(rig.model, "ring", _env_shapes)
    rig.probes["keyA_env"] = PairProbe(rig.model, "key_a", _env_shapes)
    rig.probes["keyB_env"] = PairProbe(rig.model, "key_b", _env_shapes)
    rig.probes["robot_object"] = RobotContactProbe(
        rig.model, ["ring", "key_a", "key_b"], ["board", "rod"])
    rod_probe = ForceProbe(rig.model, sensing_shape="rod")
    board_probe = ForceProbe(rig.model, sensing_shape="board")

    GS.make_pipeline(rig, rigid_contact_max=20000, max_triangle_pairs=4_000_000)
    rig.solver = make_solver(rig.model, iterations=15)
    print(f"[hang_keychain] solver {getattr(rig.solver, '_force_bench_effective_options', {})}"
          f" cadence={CADENCE}")

    _btm = rig.meshes["board"][0]
    PEG_CL = []
    for _x in np.arange(-0.1290, -0.0855, 0.0005):
        _sec = _btm.section(plane_origin=[_x, 0, 0], plane_normal=[1, 0, 0])
        if _sec is None:
            continue
        _q = np.vstack([l for l in _sec.discrete])
        _m = (_q[:, 1] > PEG_Y - 0.009) & (_q[:, 1] < PEG_Y + 0.009) \
            & (_q[:, 2] > 0.10)
        if not _m.any():
            continue
        _p = _q[_m]
        PEG_CL.append([_x, (_p[:, 1].min() + _p[:, 1].max()) / 2,
                       (_p[:, 2].min() + _p[:, 2].max()) / 2])
    PEG_CL = np.asarray(PEG_CL)
    assert len(PEG_CL) > 40, "peg centreline not found"

    ri = rig.body_index("ring")
    ki = {n: rig.body_index(n) for n in ("key_a", "key_b")}
    fli = rig.body_index("fr3/fr3_leftfinger")
    fri = rig.body_index("fr3/fr3_rightfinger")
    arm_ix = [i for i, l in enumerate(rig.labels) if l.startswith("fr3/")]

    st = {"corr": np.zeros(2), "err_f": np.zeros(2), "grasp": None,
          "g_done": False, "q_cmd": np.asarray(q_start[:7], dtype=float).copy(),
          "dq": np.zeros(7), "dq_peak": 0.0, "tcp_cmd": None,
          "tcp_err_peak": 0.0, "droop": np.zeros(3), "droop_hold": None,
          "c_cmd": None, "R_cmd": None, "slip_peak": np.zeros(3),
          "grip_adv": 0.0, "ik_res": 0.0}
    # event log of the state machine (times in s)
    E = {"phase": "carry", "u_lat": 0.0, "t": {}, "z_cmd": None,
         "f_relax0": None, "tcp_hold": None, "desc_end": None,
         "ring_env_at_stop": None, "fingers": FINGER_HOME}

    def enter(phase, t):
        E["phase"] = phase
        E["t"][phase] = float(t)
        print(f"t={t:6.2f}s phase -> {phase}")

    def drive_smooth(tcp_p, tcp_q, fingers):
        q_sol = rig.ik.solve(tcp_p, tcp_q, iterations=16)
        st["ik_res"] = float(np.linalg.norm(rig.ik.tcp_pose()[:3] - tcp_p))
        err = np.asarray(q_sol[:7], dtype=float) - st["q_cmd"]
        v_lim = np.minimum(DQ_MAX, np.sqrt(2.0 * DDQ_MAX * np.abs(err)))
        dq_t = np.clip(err, -v_lim, v_lim)
        dq = st["dq"] + np.clip(dq_t - st["dq"], -DDQ_MAX, DDQ_MAX)
        st["dq"] = dq
        st["q_cmd"] = st["q_cmd"] + dq
        st["dq_peak"] = max(st["dq_peak"], float(np.abs(dq).max()))
        jq = rig.ik.joint_q.numpy()
        jq[0, :7] = st["q_cmd"]
        rig.ik.joint_q.assign(jq)
        qt = rig.control.joint_target_q.numpy()
        qt[:7] = st["q_cmd"]
        qt[7:9] = fingers
        rig.control.joint_target_q.assign(qt)

    HOVER_C = np.array([-0.0700, THREAD_Y, HOVER_Z])
    THREAD_C = np.array([THREAD_X, THREAD_Y, HOVER_Z])

    def last(name):
        h = rig.probes[name].history
        return h[-1]["force"] if h else 0.0

    def save_ckpt(t):
        import copy
        import pickle
        snap = {"t": float(t),
                "state": {k: getattr(rig.s0, k).numpy().copy()
                          for k in ("body_q", "body_qd", "joint_q", "joint_qd")},
                "target_q": rig.control.joint_target_q.numpy().copy(),
                "target_qd": rig.control.joint_target_qd.numpy().copy(),
                "gravity": rig.model.gravity.numpy().copy(),
                "ik_q": rig.ik.joint_q.numpy().copy(),
                "st": copy.deepcopy(st), "E": copy.deepcopy(E),
                "last": {k: dict(p.history[-1]) for k, p in rig.probes.items()
                         if p.history},
                "last_rod": dict(rod_probe.history[-1]),
                "last_board": dict(board_probe.history[-1])}
        with open(CKPT_SAVE, "wb") as fh:
            pickle.dump(snap, fh)
        print(f"t={t:6.2f}s checkpoint saved -> {CKPT_SAVE}")

    def control(t):
        if CKPT_SAVE and not st.get("ckpt_saved") and t >= CKPT_T:
            st["ckpt_saved"] = True
            save_ckpt(t)
        bq = rig.s0.body_q.numpy()
        ring_p = bq[ri, :3]
        ph = E["phase"]
        after_lat = ph not in ("carry", "lateral")
        if CADENCE == "suite":
            rig.collide_every_substep = (t < T_PEEL) or (ph in (
                "open", "wait", "retreat", "still", "done"))
        else:   # "thread" (and "always", which the override pins anyway)
            rig.collide_every_substep = (t < T_PEEL) or after_lat

        if not st["g_done"]:
            s = float(np.clip(t / T_GRAV, 0.1, 1.0))
            rig.model.gravity.assign(
                np.array([[0.0, 0.0, -9.81 * s * s]], dtype=np.float32))
            if s >= 1.0:
                st["g_done"] = True

        if st["grasp"] is None and t >= T_SETTLE:
            st["grasp"] = ring_p.copy()
            print(f"t={t:.2f}s grasp target: ring centre {np.round(ring_p * MM, 2)} mm")
        g = st["grasp"]
        F_env = (rod_probe.history[-1]["force"] + board_probe.history[-1]["force"]) \
            if rod_probe.history else 0.0

        # ── state machine transitions (after the suite's carry) ─────────
        if ph == "carry" and t >= T_HOVER:
            enter("lateral", t)
        elif ph == "lateral" and E["u_lat"] >= 1.0:
            enter("dwell", t)
        elif ph == "dwell" and t >= E["t"]["dwell"] + T_DWELL2:
            E["z_cmd"] = HOVER_Z
            enter("descend", t)
        elif ph == "descend":
            f_ring = last("ring_env")
            if E["z_cmd"] <= PLACE_Z + 1e-9 or f_ring > F_RING_STOP:
                E["desc_end"] = float(E["z_cmd"])
                E["ring_env_at_stop"] = f_ring
                print(f"t={t:6.2f}s descent ends: ring-centre cmd "
                      f"{E['z_cmd']*MM:.1f} mm, ring<->board {f_ring:.3f} N, "
                      f"keys<->board {last('keyA_env'):.3f}/{last('keyB_env'):.3f} N")
                enter("hold", t)
        elif ph == "hold" and t >= E["t"]["hold"] + T_HOLD:
            enter("relax", t)
        elif ph == "relax" and t >= E["t"]["relax"] + T_RELAX:
            enter("open", t)
        elif ph == "open" and t >= E["t"]["open"] + T_OPEN_EFF:
            enter("wait", t)
        elif ph == "wait" and t >= E["t"]["wait"] + T_WAIT:
            enter("retreat", t)
        elif ph == "retreat" and t >= E["t"]["retreat"] + T_RET:
            enter("still", t)
        elif ph == "still" and t >= E["t"]["still"] + T_STILL:
            enter("done", t)
        ph = E["phase"]

        # ── fingers ─────────────────────────────────────────────────────
        if t < T_CAGE:
            fingers = FINGER_HOME + (FINGER_OPEN - FINGER_HOME) * smooth(t / 1.2)
        elif t < T_CLOSE:
            fingers = FINGER_OPEN + (FINGER_CLOSE - FINGER_OPEN) * \
                smooth((t - T_CAGE) / (T_CLOSE - T_CAGE))
        elif ph in ("carry", "lateral", "dwell", "descend", "hold"):
            fingers = FINGER_CLOSE + (FINGER_TIGHT - FINGER_CLOSE) * \
                smooth((t - T_CLOSE) / (T_TIGHT - T_CLOSE))
            if t >= T_TIGHT and rig.probes["grip_L"].history:
                f = min(last("grip_L"), last("grip_R"))
                if f < F_GRIP_TARGET - F_GRIP_DB:
                    st["grip_adv"] += GRIP_STEP
                elif f > F_GRIP_TARGET + F_GRIP_DB:
                    st["grip_adv"] -= GRIP_STEP
                st["grip_adv"] = float(np.clip(st["grip_adv"], 0.0,
                                               FINGER_TIGHT - FINGER_MIN))
            fingers -= st["grip_adv"]
        elif ph == "relax":
            if E["f_relax0"] is None:
                E["f_relax0"] = E["fingers"]
            fingers = E["f_relax0"] + (FINGER_TOUCH - E["f_relax0"]) * smooth(
                (t - E["t"]["relax"]) / T_RELAX)
        elif ph == "open":
            if OPEN_SPEED > 0:
                fingers = FINGER_TOUCH + _trap(t - E["t"]["open"],
                                               FINGER_WIDE - FINGER_TOUCH,
                                               OPEN_SPEED, OPEN_ACC)
            else:
                fingers = FINGER_TOUCH + (FINGER_WIDE - FINGER_TOUCH) * smooth(
                    (t - E["t"]["open"]) / T_OPEN)
        else:
            fingers = FINGER_WIDE
        E["fingers"] = fingers

        # ── TCP trajectory ──────────────────────────────────────────────
        tcp_q = Q_GRASP
        p_hat = P_HAT_G
        if t < T_APPR:
            appr = (g if g is not None else p_ring0) + (TCP_R + CAGE_STAND) * P_HAT_G
            s = smooth((t - T_SETTLE) / (T_APPR - T_SETTLE)) if t >= T_SETTLE else 0.0
            tcp_p = tcp0 + s * (appr - tcp0)
        else:
            c_pin = g.copy()
            lift = c_pin + [0, 0, 0.135 - c_pin[2]]
            wpts = [
                (T_APPR, c_pin), (T_CAGE, c_pin), (T_TIGHT, c_pin),
                (T_LIFT, lift),
                (T_TRANS, np.array([-0.0700, THREAD_Y, lift[2]])),
                (T_DWELL, np.array([-0.0700, THREAD_Y, lift[2]])),
                (T_HOVER0, HOVER_C), (T_HOVER, HOVER_C),
            ]

            def c_des(tt):
                if tt <= T_APPR:
                    return wpts[0][1]
                for (ta, pa), (tb, pb) in zip(wpts, wpts[1:]):
                    if tt <= tb:
                        s = 0.0 if tb == ta else np.clip((tt - ta) / (tb - ta), 0, 1)
                        return pa + smooth(s) * (pb - pa)
                return wpts[-1][1]

            stand = CAGE_STAND * (1.0 - smooth(
                (t - T_APPR) / (T_CAGE - T_APPR))) if t < T_CAGE else 0.0

            if ph == "carry":
                des = c_des(t)
            elif ph == "lateral":
                gate = float(np.clip((2.5 - F_env) / (2.5 - 0.8), 0.15, 1.0))
                E["u_lat"] = min(1.0, E["u_lat"] + gate / (T_LAT * rig.fps))
                des = HOVER_C + smooth(E["u_lat"]) * (THREAD_C - HOVER_C)
            elif ph == "dwell":
                des = THREAD_C.copy()
            elif ph == "descend":
                E["z_cmd"] = max(PLACE_Z, E["z_cmd"] - V_DESC / rig.fps)
                des = np.array([THREAD_X, THREAD_Y, E["z_cmd"]])
            else:
                des = np.array([THREAD_X, THREAD_Y,
                                E["desc_end"] if E["desc_end"] is not None
                                else E["z_cmd"]])

            # closed-loop x/y on the measured ring centre, armed from the
            # climb to the hover until the squeeze is relaxed (suite law)
            err = des[:2] - ring_p[:2]
            st["err_f"] = st["err_f"] + CORR_ALPHA * (err - st["err_f"])
            e = st["err_f"]
            e = np.sign(e) * np.maximum(np.abs(e) - CORR_DB, 0.0)
            if t >= T_HOVER0 and ph in ("carry", "lateral", "dwell", "descend", "hold"):
                st["corr"] += np.clip(CORR_GAIN * e, -CORR_STEP, CORR_STEP)
                st["corr"] = np.clip(st["corr"], -0.008, 0.008)

            if ph not in ("retreat", "still", "done"):
                tcp_p = des + (TCP_R + stand) * p_hat + np.array([*st["corr"], 0.0])
                if PULL > 0 and ph in ("open", "wait"):
                    u = smooth((t - E["t"]["open"] - PULL_T0) / T_PULL)
                    tcp_p = tcp_p + u * (PULL * R_HAT + np.array([0.0, 0.0, PULL_UP]))
                E["tcp_hold"] = tcp_p.copy()
            else:
                # radial OUT of the ring's footprint first (the lower pad is
                # under the rim), then UP and sideways
                s1 = float(np.clip((t - E["t"]["retreat"]) / T_RET, 0, 1))
                out = smooth(float(np.clip(s1 / 0.45, 0.0, 1.0)))
                up = smooth(float(np.clip((s1 - 0.30) / 0.45, 0.0, 1.0)))
                side = smooth(float(np.clip((s1 - 0.55) / 0.45, 0.0, 1.0)))
                tcp_p = E["tcp_hold"] + out * RET_OUT * R_HAT \
                    + up * RET_UP * np.array([0.0, 0.0, 1.0]) \
                    + side * RET_SIDE * T_HAT
            st["c_cmd"] = des + np.array([*st["corr"], 0.0])
            st["R_cmd"] = R_of_q(tcp_q)

        tcp_act = (bq[fli, :3] + bq[fri, :3]) / 2 \
            + world_T(bq[fli])[:3, :3] @ FINGER_TCP_OFF
        cmd_step = 0.0 if st["tcp_cmd"] is None else float(
            np.linalg.norm(tcp_p - st["tcp_cmd"]))
        if st["tcp_cmd"] is not None and t >= T_SETTLE:
            if t < T_HOVER and cmd_step < 5.0e-5:
                derr = st["tcp_cmd"] - tcp_act
                st["droop"] = np.clip(
                    st["droop"] + np.clip(DROOP_GAIN * derr, -DROOP_STEP, DROOP_STEP),
                    -DROOP_MAX, DROOP_MAX)
            elif t >= T_HOVER and st["droop_hold"] is None:
                st["droop_hold"] = st["droop"].copy()
                print(f"t={t:.2f}s droop lead frozen at {np.round(st['droop'] * MM, 2)} mm")
        st["tcp_cmd"] = tcp_p.copy()
        drive_smooth(tcp_p + st["droop"], tcp_q, fingers)

    def key_ring_angle(bq, name):
        Tk = world_T(bq[ki[name]])
        Tr = world_T(bq[ri])
        blade_r = Tr[:3, :3].T @ (Tk[:3, :3] @ np.array([0, 0, -1.0]))
        return np.degrees(np.arccos(np.clip(-blade_r[2], -1, 1)))

    def peg_link(bq_row_ring):
        T_r = world_T(bq_row_ring)
        n = T_r[:3, :3] @ np.array([1.0, 0.0, 0.0])
        c = T_r[:3, 3]
        d = (PEG_CL - c) @ n
        sg = np.sign(d)
        idx = np.flatnonzero(sg[:-1] * sg[1:] < 0)
        best = (9.9, 9.9)
        for i in idx:
            f = d[i] / (d[i] - d[i + 1])
            pt = PEG_CL[i] + f * (PEG_CL[i + 1] - PEG_CL[i])
            o = float(np.linalg.norm(pt - c))
            if o < best[0]:
                best = (o, float(pt[0]))
        return best[0], best[1], float(abs(n[0]))

    def extra(state, t):
        r = rod_probe.sample(rig.contacts, state, t)
        b = board_probe.sample(rig.contacts, state, t)
        bq = state.body_q.numpy()
        gl, gr = last("grip_L"), last("grip_R")
        tl, tr = last("fingertip_L"), last("fingertip_R")
        off, px, nx = peg_link(bq[ri])
        if st["c_cmd"] is not None and E["phase"] in (
                "carry", "lateral", "dwell", "descend", "hold"):
            dv = st["R_cmd"].T @ (bq[ri, :3] - st["c_cmd"])
            slip = np.array([dv[0], dv[1], -dv[2]])
            st["slip_peak"] = np.maximum(st["slip_peak"], np.abs(slip))
        else:
            slip = np.zeros(3)
        tcp_act = (bq[fli, :3] + bq[fri, :3]) / 2 + \
            (world_T(bq[fli])[:3, :3] @ FINGER_TCP_OFF)
        if st["tcp_cmd"] is not None:
            st["tcp_err_peak"] = max(st["tcp_err_peak"], float(
                np.linalg.norm(tcp_act - st["tcp_cmd"])))
        if int(round(t * rig.fps)) % 15 == 0:
            de = (tcp_act - st["tcp_cmd"]) * MM if st["tcp_cmd"] is not None \
                else np.zeros(3)
            print(f"  [{t:5.2f}] {E['phase']:>7} grip {gl:4.2f}/{gr:4.2f} N ring "
                  f"{np.round(bq[ri, :3]*MM, 1)} nx {nx:.2f} peg_off {off*MM:6.2f}"
                  f"@{px*MM:6.1f}  env r/kA/kB {last('ring_env'):.3f}/"
                  f"{last('keyA_env'):.3f}/{last('keyB_env'):.3f} brd {b['force']:.3f}"
                  f"  robot-obj {last('robot_object'):.3f}  tcp_err "
                  f"{np.round(de, 1)} ik {st['ik_res']*MM:.2f}")
        return {
            "ncon": int(rig.contacts.rigid_contact_count.numpy()[0]),
            "rod_N": round(r["force"], 4),
            "board_N": round(b["force"], 4),
            "grip_min_N": round(min(gl, gr), 3),
            "tip_max_N": round(max(tl, tr), 3),
            "ring_board_N": round(last("ring_env"), 4),
            "keys_board_N": round(last("keyA_env") + last("keyB_env"), 4),
            "robot_object_contact_N": round(last("robot_object"), 4),
            "ring_x_mm": round(bq[ri, 0] * MM, 1),
            "ring_y_mm": round(bq[ri, 1] * MM, 1),
            "ring_z_mm": round(bq[ri, 2] * MM, 1),
            "peg_offset_mm": round(off * MM, 2),
            "ring_nx": round(nx, 3),
            "slip_radial_mm": round(float(slip[2]) * MM, 3),
            "slip_tang_mm": round(float(slip[0]) * MM, 3),
            "key_a_rel_deg": round(key_ring_angle(bq, "key_a"), 1),
            "key_b_rel_deg": round(key_ring_angle(bq, "key_b"), 1),
            "phase": PHASES.index(E["phase"]),
            "finger_cmd_mm": round(E["fingers"] * MM, 3),
            "ik_residual_mm": round(st["ik_res"] * MM, 3),
        }

    def done(t):
        return E["phase"] == "done" or (STOP_T > 0 and t >= STOP_T)

    T0 = 0.0
    if CKPT_LOAD:
        import pickle
        with open(CKPT_LOAD, "rb") as fh:
            snap = pickle.load(fh)
        T0 = snap["t"]
        for k, a in snap["state"].items():
            getattr(rig.s0, k).assign(a)
            getattr(rig.s1, k).assign(a)
        rig.control.joint_target_q.assign(snap["target_q"])
        rig.control.joint_target_qd.assign(snap["target_qd"])
        rig.model.gravity.assign(snap["gravity"])
        rig.ik.joint_q.assign(snap["ik_q"])
        st.update(snap["st"])
        st["ckpt_saved"] = True
        E.update(snap["E"])
        for k, rec_ in snap["last"].items():
            rig.probes[k].history.append(rec_)
        rod_probe.history.append(snap["last_rod"])
        board_probe.history.append(snap["last_board"])
        print(f"[hang_keychain] RESTARTED from checkpoint {CKPT_LOAD} at t={T0:.3f}s "
              f"(phase {E['phase']}); no QA for a restarted rollout")
        _c, _x, _d = control, extra, done
        control = lambda t: _c(t + T0)             # noqa: E731
        extra = lambda state, t: {**_x(state, t + T0), "t_world": round(t + T0, 4)}  # noqa: E731
        done = lambda t: _d(t + T0)                # noqa: E731

    rec = run_until(rig, T_MAX - T0, control, extra, done)
    paths = rec.save("rollout", force_keys=["grip_L_N", "grip_R_N", "ring_board_N",
                                            "keys_board_N", "robot_object_contact_N"])
    print("saved:", paths, f"(sim wall {time.time() - t_wall0:.0f} s)")
    print("events:", {k: round(v, 2) for k, v in E["t"].items()})

    # ── QA ──────────────────────────────────────────────────────────────
    d = np.load(paths["states"], allow_pickle=True)
    t_arr = d["t"]
    bq_all = d["body_q"]
    t_end = float(t_arr[-1])

    def window(t0, t1):
        m = (t_arr >= t0) & (t_arr <= t1)
        fs = np.flatnonzero(m)
        pl = [peg_link(bq_all[f, ri]) for f in fs]
        rod = float(np.mean([r["force"] for r in rod_probe.history if t0 <= r["t"] <= t1]))
        brd = float(np.mean([r["force"] for r in board_probe.history if t0 <= r["t"] <= t1]))
        return dict(z=float(bq_all[m, ri, 2].mean()), off=float(max(p[0] for p in pl)),
                    px_lo=float(min(p[1] for p in pl)), px_hi=float(max(p[1] for p in pl)),
                    nx=float(min(p[2] for p in pl)), rod=rod, brd=brd)

    def hung(w):
        return bool(w["off"] < R_ID - 0.001 and PEG_X0 <= w["px_lo"]
                    and w["px_hi"] <= PEG_X1 and (w["rod"] + w["brd"]) >= 0.8 * W_TOTAL)

    def hang_block(w, note):
        return {"pass": bool(hung(w)), "ring_z_mm": round(w["z"] * MM, 1),
                "expected_rest_z_mm": round(HANG_Z * MM, 1),
                "peg_pierce_offset_mm": round(w["off"] * MM, 2),
                "ring_hole_radius_mm": round(R_ID * MM, 2),
                "peg_pierce_x_mm": [round(w["px_lo"] * MM, 1), round(w["px_hi"] * MM, 1)],
                "peg_x_extent_mm": [round(PEG_X0 * MM, 1), round(PEG_X1 * MM, 1)],
                "ring_normal_x": round(w["nx"], 3),
                "rod_contact_N": round(w["rod"], 4),
                "board_contact_N": round(w["brd"], 4), "note": note}

    W_F = window(t_end - 0.8, t_end)
    print("final hang window:", {k: round(v * (MM if k in ('z', 'off', 'px_lo', 'px_hi') else 1), 3)
                                 for k, v in W_F.items()}, "hung:", hung(W_F))
    if (STOP_T > 0 and E["phase"] != "done") or CKPT_LOAD:
        print(f"[hang_keychain] stopped in phase {E['phase']} "
              f"({'restarted from a checkpoint' if CKPT_LOAD else f'HK_STOP_T={STOP_T}'}), "
              f"QA skipped")
        return

    qa = rig.qa(paths)
    dt = np.diff(t_arr)
    dt[dt <= 0] = np.inf
    disp = np.linalg.norm(np.diff(bq_all[..., :3], axis=0), axis=-1)
    fd_vel = disp / dt[:, None]
    m_post = t_arr[:-1] >= T_SETTLE
    peak_fd = float(fd_vel[m_post].max())
    labels = list(d["body_labels"])
    qa.report["checks"]["velocity_spikes"] = {
        "pass": bool(np.isfinite(peak_fd) and peak_fd < 2.5),
        "peak_displacement_velocity_m_s": round(peak_fd, 3),
        "worst_body": labels[int(fd_vel[m_post].max(axis=0).argmax())],
        "note": "flying-object detector, evaluated after the ramped "
                "pile-assembly settle (t >= T_SETTLE)"}
    qa.report["pass"] = bool(all(v["pass"] for v in qa.report["checks"].values()))

    tE = E["t"]
    t_open_end = tE["wait"]
    W_S = window(tE["wait"] + 0.4, tE["retreat"] + 0.5 * T_RET)
    ok_hang, still_hung = hung(W_S), hung(W_F)

    def link_at(f, name):
        Tk = world_T(bq_all[f, ki[name]])
        Tr = world_T(bq_all[f, ri])
        loop_w = (Tk[:3, :3] @ key_holes[name].T).T + Tk[:3, 3]
        circ_w = (Tr[:3, :3] @ circle_local.T).T + Tr[:3, 3]
        return gauss_link(loop_w, circ_w)

    f_hang = int(np.searchsorted(t_arr, tE["retreat"]))
    links = {n: {"start": round(link_at(0, n), 3), "hang": round(link_at(f_hang, n), 3),
                 "end": round(link_at(len(t_arr) - 1, n), 3)} for n in ("key_a", "key_b")}
    ok_linked = all(abs(abs(v[k]) - 1.0) < 0.25 for v in links.values()
                    for k in ("hang", "end"))

    swings, ok_swing = {}, True
    tf = np.array([r["t"] for r in rec.force_rows])
    for n in ("key_a", "key_b"):
        ang = np.array([r[f"{n}_rel_deg"] for r in rec.force_rows])
        m = (tf >= T_TIGHT) & (tf <= tE["relax"])
        rng = float(ang[m].max() - ang[m].min())
        swings[n] = round(rng, 1)
        ok_swing &= rng > 10.0

    tg = np.array([r["t"] for r in rig.probes["grip_L"].history])
    gl = np.array([r["force"] for r in rig.probes["grip_L"].history])
    gr = np.array([r["force"] for r in rig.probes["grip_R"].history])
    mg = (tg >= T_TIGHT) & (tg <= tE["relax"])
    grip_min = float(np.minimum(gl, gr)[mg].min())
    grip_mean = float(((gl + gr) / 2)[mg].mean())
    ok_grip = grip_min > 0.5
    ok = bool(ok_hang and still_hung and ok_linked and ok_swing and ok_grip)
    qa.report["checks"]["task_success"] = {
        "pass": ok,
        "hang_phase": hang_block(W_S, "pads open, hand retreating: the peg pierces the "
                                      "ring's own plane INSIDE the 21 mm hole, within the "
                                      "peg's x extent, board carries >= 80 % of the weight"),
        "keys_linked_gauss": {**links, "pass": bool(ok_linked),
                              "note": "|L|=1 <=> the key's bow slot still encircles the ring"},
        "keys_free_swing": {**swings, "pass": bool(ok_swing),
                            "note": "key-vs-ring blade angle range close->relax > 10 deg"},
        "grip": {"min_N": round(grip_min, 3), "mean_N": round(grip_mean, 3),
                 "pass": bool(ok_grip),
                 "admittance_advance_mm": round(float(st["grip_adv"]) * MM, 3),
                 "peak_slip_tang_axial_radial_mm": [round(float(v) * MM, 2)
                                                    for v in st["slip_peak"]],
                 "note": "min(left,right) pair-resolved fingertip<->ring force, "
                         "tighten -> relax; slip = ring vs a rigid pad-ring relation "
                         "while held"},
        "final_hang": hang_block(W_F, "last 0.8 s, arm retreated and still"),
        "keychain_weight_N": round(W_TOTAL, 4),
    }
    qa.report["pass"] &= ok

    # smoothness (suite gate, windows mapped onto this protocol)
    dtf = float(np.median(np.diff(t_arr)))
    arm_q = [i for i, lbl in enumerate(qa.labels) if lbl.startswith("fr3/")]
    P = qa.body_q[:, arm_q, :3]
    V = np.diff(P, axis=0) / dtf
    A = np.linalg.norm(np.diff(V, axis=0) / dtf, axis=-1)
    free_m = np.ones(len(t_arr), bool)
    free_m[(t_arr < T_SETTLE) | ((t_arr > tE["relax"] - 0.1) & (t_arr < t_open_end + 0.2))
           | ((t_arr > T_CLOSE - 0.1) & (t_arr < T_TIGHT + 0.2))] = False
    peak_a = float(A[free_m[1:-1], :].max())

    def _jitter_rms(t0, t1):
        i7 = qa.labels.index("fr3/fr3_link7")
        m = (t_arr >= t0) & (t_arr <= t1)
        p = qa.body_q[m, i7, :3]
        if len(p) < 20:
            return 0.0
        w = max(3, int(round(0.25 / dtf)) | 1)
        ker = np.ones(w) / w
        resid = np.empty_like(p)
        for k in range(3):
            pad = np.pad(p[:, k], (w // 2, w // 2), mode="edge")
            resid[:, k] = p[:, k] - np.convolve(pad, ker, "valid")[:len(p)]
        return float(np.sqrt((np.linalg.norm(resid, axis=1) ** 2).mean()))

    j_hover = _jitter_rms(T_HOVER0 + 0.1, T_HOVER)
    j_hang = _jitter_rms(tE["still"], t_end)
    smooth_ok = j_hover < 0.0015 and j_hang < 0.0015 and peak_a < 15.0
    qa.report["checks"]["smoothness"] = {
        "pass": bool(smooth_ok), "tcp_jitter_hover_rms_mm": round(j_hover * MM, 3),
        "tcp_jitter_final_hold_rms_mm": round(j_hang * MM, 3),
        "peak_arm_accel_free_m_s2": round(peak_a, 2),
        "peak_arm_accel_all_m_s2": round(float(A.max()), 2),
        "peak_joint_cmd_step_rad": round(st["dq_peak"], 5),
        "note": "link7 detrended jitter RMS < 1.5 mm (hover, final hold); peak fr3-link "
                "accel < 15 m/s^2 outside settle / finger close / relax+open"}
    qa.report["pass"] &= smooth_ok

    from trimesh.proximity import ProximityQuery
    pq_ring = ProximityQuery(ring_local)
    frames = np.linspace(0, len(t_arr) - 1, 24).astype(int)
    pen, ok_pen = {}, True
    for n in ("key_a", "key_b"):
        sub = key_locals[n].vertices[::4]
        worst, worst_t = -9.0, None
        for f in frames:
            M = np.linalg.inv(world_T(bq_all[f, ri])) @ world_T(bq_all[f, ki[n]])
            pts = (M[:3, :3] @ sub.T).T + M[:3, 3]
            mm = ((pts > ring_local.bounds[0] - 0.003)
                  & (pts < ring_local.bounds[1] + 0.003)).all(axis=1)
            if not mm.any():
                continue
            sd = float(pq_ring.signed_distance(pts[mm]).max())
            if sd > worst:
                worst, worst_t = sd, float(t_arr[f])
        pen[f"{n}->ring"] = {"max_depth_mm": round(worst * MM, 3), "at_t": worst_t,
                             "pass": bool(worst < 0.0003)}
        ok_pen &= pen[f"{n}->ring"]["pass"]
    qa.report["checks"]["key_ring_penetration"] = {"pass": bool(ok_pen),
                                                   "tolerance_mm": 0.3, "pairs": pen}
    qa.report["pass"] &= ok_pen
    t_pen = time.time()
    for lbl_b, loc in (("ring", ring_local), ("key_a", key_locals["key_a"]),
                       ("key_b", key_locals["key_b"])):
        pen_static_fast(qa, lbl_b, loc, *rig.meshes["rod"], f"{lbl_b}->rod", tol=0.0015)
        pen_static_fast(qa, lbl_b, loc, *rig.meshes["board"], f"{lbl_b}->board", tol=0.0015)
    print(f"static penetration QA {time.time() - t_pen:.1f} s")

    t_cl = time.time()
    clr = robot_keychain_clearance(rig, t_arr, bq_all)
    arm_pairs = {k: v for k, v in clr.items() if "finger" not in k.split("<->")[0]}
    arm_worst = max([v[0] for v in arm_pairs.values()], default=-0.010)
    qa.report["checks"]["arm_keychain_clearance"] = {
        "pass": bool(arm_worst < 0.0),
        "non_finger_links_max_signed_distance_mm": round(arm_worst * MM, 2),
        "pairs_within_10mm": {k: {"max_signed_distance_mm": round(v[0] * MM, 2),
                                  "at_t": round(v[1], 2)} for k, v in clr.items()},
        "note": "the scene filters non-finger FR3 links against the keychain "
                "(fingertips only, as in the suite), so this re-checks on the recorded "
                "poses that no filtered link ever overlaps the ring or a key (+ = inside); "
                "finger<->ring values are the pinch contact itself"}
    qa.report["pass"] &= bool(arm_worst < 0.0)
    print(f"arm/keychain clearance check {time.time() - t_cl:.1f} s: "
          f"{ {k: round(v[0] * MM, 2) for k, v in clr.items()} }")

    # ── appendix criteria ──────────────────────────────────────────────
    hro = rig.probes["robot_object"].history
    t_rc = np.array([r["t"] for r in hro])
    f_rc = np.array([r["force"] for r in hro])
    t_clear = tE["still"]
    check_released_and_clear(qa, t_rc, f_rc, t_clear)
    clear = qa.report["checks"]["released_and_clear"]
    # robot still: every FR3 link's displacement speed over the last 0.5 s
    m_end = t_arr >= t_end - 0.5
    v_arm = np.linalg.norm(np.diff(bq_all[m_end][:, arm_ix, :3], axis=0), axis=-1) \
        / dtf
    arm_still = bool(v_arm.max() < 0.005)
    # moved away: TCP (finger midpoint) displacement from the release pose
    f_rel = int(np.searchsorted(t_arr, tE["retreat"]))
    mid = 0.5 * (bq_all[:, fli, :3] + bq_all[:, fri, :3])
    moved = float(np.linalg.norm(mid[-1] - mid[f_rel]))
    ring_end = bq_all[-1, ri, :3]
    gap_ring = float(np.linalg.norm(mid[-1] - ring_end))
    v_ring = float(np.linalg.norm(np.diff(bq_all[m_end][:, ri, :3], axis=0), axis=-1).max()
                   / dtf)
    crit = {
        "keychain_on_hook": {
            **hang_block(W_F, "last 0.8 s: the hook/peg passes through the ring's hole "
                              "(peg centreline pierces the ring plane inside the hole, "
                              "within the peg's x extent) and the board carries >= 80 % "
                              "of the keychain's weight"),
            "hang_phase_pass": bool(ok_hang),
            "ring_peak_speed_last_0.5s_mm_s": round(v_ring * MM, 1)},
        "keys_still_linked": {"pass": bool(ok_linked), **links},
        "real_fingertip_grasp": {"pass": bool(ok_grip), "min_N": round(grip_min, 3),
                                 "mean_N": round(grip_mean, 3),
                                 "note": "pair-resolved pad<->ring force while carried; no "
                                         "joint, weld or kinematic object motion exists in "
                                         "this scene"},
        "released_and_clear": {**clear},
        "robot_moved_away": {"pass": bool(moved >= 0.05 and gap_ring >= 0.05),
                             "tcp_moved_from_release_mm": round(moved * MM, 1),
                             "finger_midpoint_to_ring_mm": round(gap_ring * MM, 1),
                             "min_mm": 50.0},
        "no_hidden_arm_contact": {
            k: v for k, v in qa.report["checks"]["arm_keychain_clearance"].items()
            if k in ("pass", "non_finger_links_max_signed_distance_mm", "note")},
        "robot_still": {"pass": arm_still,
                        "peak_link_speed_last_0.5s_mm_s": round(float(v_arm.max()) * MM, 2),
                        "threshold_mm_s": 5.0},
    }
    for k in ("robot_moved_away", "robot_still"):
        qa.report["checks"][k] = crit[k]
    qa.report["pass"] = bool(all(c.get("pass", True) for c in qa.report["checks"].values()))
    qa.report["events_s"] = {k: round(v, 3) for k, v in E["t"].items()}
    qa.report["descent"] = {"ring_centre_cmd_end_mm": None if E["desc_end"] is None
                            else round(E["desc_end"] * MM, 2),
                            "ring_board_N_at_stop": E["ring_env_at_stop"]}
    report = rig.finish(qa)
    save_appendix_verdict(
        rig, task="Hang object (keychain on the board's hook)",
        source="force_bench.tasks.keychain_hang (scene recipe + pick/carry expert reused)",
        spec="The robot movement stops after placing the target object onto a hanging "
             "hook. The robot should not remain in contact with the task objects.",
        criteria=crit, physics_qa=report,
        changes=[
            "release: PLACE, don't drop -- the ring is lowered onto the hook while still "
            "pinched until the hook's crest is inside its hole (ring centre z 165.5 mm), "
            "the squeeze is relaxed, then the fingers open at the Franka Hand's speed "
            "(0.05 m/s per finger) while the arm slides the hand 20 mm straight back in "
            "0.15 s, so the ring swings off the lower pad about the bar into the hang "
            "(suite: opened 19.5 mm above the hook; both bare runs lost the keychain)",
            "lateral approach at the hover height + a 0.6 s dwell before the descent",
            "retreat: radially out of the ring's footprint, up and sideways, all inside "
            "the arm's IK-feasible set (the suite's +x leg is unreachable at this "
            "attitude: 18-32 mm IK residual), then a still hold",
            "collision every substep from the end of the lateral approach on",
            "primary camera moved so pile, carry, hook and hang are all in frame",
            "slip instrument: the closed-loop correction is applied in x/y (the suite's "
            "instrument added it to y/z)",
        ])
    print(f"[hang_keychain] total wall {time.time() - t_wall0:.0f} s")


if __name__ == "__main__":
    wp.init()
    main()
