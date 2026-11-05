"""ContactBench task 25 "Plug in Ethernet" -- the kit's printed Ethernet plug into its 5-port slot, as in the video.

    Fully insert the Ethernet plug held in hand into a socket placed in the workspace.  Success (paper Table C1): the
    Ethernet plug is fully inserted into the socket with its latch engaged, such that the plug can't be withdrawn by a
    light pull; the robot stops, moves away and breaks contact with the task objects.

ASSETS: the kit library (``force_bench.kit``), ``ethernet-plug-and-slot``, built by
``force_bench.kit.mechanisms.ethernet`` (geometry, the latch model, the stiffness estimate and its provenance are
documented there): the black 98 mm shell lies on its side with its five ports facing up, long axis toward the camera,
a grey printed insert in every port, as in the real-world video; the white printed plug (no cable) has its latch tab
on a hinge with a spring drive at the printed flexure's estimated stiffness (0.72 N.m/rad) and catches behind the
inserts' lips by contact.  The plug goes into the port farthest from the camera (port 5; ``--port 1..5``).

START (as the video): the plug held in the FR3 pinch, nose 10 mm above the port's mouth, offset (+0.5, -0.6) mm
from the port axis (``--offset-mm``).  The pads close along the shell's long axis on the plug's rear block (its 11 mm
flanks, tips 2 mm above the shell face once seated), clear of the latch tab.  An unrecorded 0.3 s pre-roll closes
them to the working pinch.

SEEDED STARTS (``--seed N``, ``force-bench run plug_in_ethernet --seed N``): the slot's pose is drawn uniformly in the
paper's 0.15 x 0.15 m square at the board centre and +-20 deg of yaw (``common.sample_start``); the hand starts
above the chosen port wherever it lands.  The FR3 base stands 0.50 m from the board centre (0.42 m in the USB task)
so that every pose of the square keeps >= 0.35 rad of joint margin (at 0.42 m the elbow came within 0.08 rad of its
fold limit at the near edge).  The draw is recorded in the verdict.

PHYSICS MODEL (what the scripted policy meets; nothing positions the plug but the two fingertips and the slot):

* the plug body has 0.5 mm of side play in the port; the tab's wide part rides under the inserts' lips and has to be
  pressed 15.2 deg (its shoulder 2.6 mm down) to pass them, then springs back to 5.0 deg against the roof ramp,
  0.85 mm before the seat (the probe: 14 N of peak insertion force at 5 mm/s, the latch holds 30 N);
* the plug carries a 15 g conditioning mass (print estimate 1.8 g; docs/physics_notes.md pitfall 2.7) and the hinge a
  conditioning inertia (mechanism module); pad mu 1.4 (MuJoCo takes a pair's mu as the MAX); finger mass 0.12 kg;
* SolverMuJoCo, 32 substeps, 20 iterations, impratio 10 (stiff friction rows, as plug_in_usb).

EXPERT (privileged state): xy servo of the TCP on the MEASURED plug against its seat, frozen once the nose is 0.5 mm
into the mouth; insertion on a slew-limited speed profile (12 mm/s in air, 3 mm/s through the mouth, 6 mm/s while the
body slides in, 3 mm/s from the lips' first touch on the tab, 2 mm/s for the last 1.5 mm); the command may lead the
MEASURED HAND by a bounded lead (the arm's in-scene stiffness turns the lead into the pushing force).  The arm's
stiffness varies over the seeded poses, and the plug tips up to ~4 deg in the pinch as the lips press the tab (their
reaction acts on the tab's side of the plug; a pinch resists little torque about its own axis), so once the plug
lags the commanded speed below the lips the lead grows (0.3 mm/s, at most 0.35 mm) while the measured slot reaction is
under 25 N.  Seat = within 0.2 mm of the CAD seat.  Seat hold bled to 4 N; LIGHT PULL with the tab untouched
(force-capped at 10 N; the latch must hold it); back down onto the back wall (the pull leaves the plug on its catch);
pads open, 6 cm retreat, still.

VERDICT (appendix_qa.json): held_in_hand_at_start; fully_inserted (nose on the back wall, within 0.3 mm of the CAD
seat, while held); latch_engaged (the tab pressed past the lips on the way in, >= 12 deg, and back at its rest angle,
< 8 deg, at the seat and at the end); light_pull_held (>= 6 N for >= 0.5 s, rise < 1.5 mm: the latch's play is
0.85 mm); stays_latched_after_release (the plug may back off onto its catch, at most 1.2 mm, tab at rest);
finger_pinch_real; released_and_clear (robot <-> plug/slot < 0.05 N over the final window, hand moved >= 40 mm);
physics QA: finite, velocity spikes, penetration of the plug and its tab into the exact CAD inserts and shell
(0.4 mm), envelope.

Run:  python -m force_bench.appendix_tasks.plug_in_ethernet [--port 5] [--seed N] [--offset-mm DX,DY]
      -> outputs/appendix_tasks/plug_in_ethernet/
"""

from __future__ import annotations

import argparse
import json

import numpy as np
import trimesh
import warp as wp

from force_bench.assets import set_body_mass
from force_bench.kit import add_asset, load_kit, place_on_table, pose_matrix
from force_bench.kit.mechanisms import ethernet as EM
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.nut_bolt_arm import PairProbe
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, ease, check_released_and_clear, save_appendix_verdict, StopRun, Placement,
    sample_start)
from force_bench.appendix_tasks.registry import CRITERIA

NAME = "plug_in_ethernet"
KEY = "ethernet-plug-and-slot"

# ── solver ──────────────────────────────────────────────────────────────────────────────────────────────────────────
SUBSTEPS = 32
ITERS = 20
IMPRATIO = 10.0             # stiff friction rows for a light part in a pinch (plug_in_usb)

# ── FR3 fingertip envelope (measured on the FR3 URDF) ─────────────────────────────────────────────────────────────
TIP_DROP = 0.00993          # m, collision tip below the TCP point
PAD_OFF = 0.000085          # pad face = finger dof - this
FINGER_KE, FINGER_KD = 8000.0, 60.0
PINCH_N = 15.0              # N per pad: Coulomb capacity 2 x 1.4 x 15 = 42 N against the ~14 N insertion peak
FINGER_F_MAX = 25.0
PAD_MU = 1.4
FINGER_MASS = 0.12
TIP_ABOVE_FACE = 0.0020     # pad tips this far above the shell face once the plug is seated (6 mm of rear block in
                            # the pads; the block's front face is 0.35 mm proud of the face)
OPEN_EXTRA = 0.0045         # release: each pad 4.5 mm off the plug

# ── scene ───────────────────────────────────────────────────────────────────────────────────────────────────────────
ROBOT_BASE_Y = 0.50         # m from the board centre (see the docstring: joint margin over the seeded square)
PORT = 5                    # the port farthest from the camera, as in the video
R_UP = np.array([[0.0, 0.0, 1.0],         # kit frame -> world: the shell on its side, ports up (+y_kit -> +z),
                 [1.0, 0.0, 0.0],         # long axis along world y (toward the camera at -y), the latch tab toward
                 [0.0, 1.0, 0.0]])        # +x (+z_kit -> +x)
SLOT_PLACE = Placement((0.0, 0.0), yaw=0.0, dxy=0.075, dyaw=np.radians(20.0), radius=0.06)
# primary camera, relative to the target port's mouth: a three-quarter view from the camera side of the board and the
# tab's side, ~33 deg up, so that the hand's narrow side faces it and the plug stays visible under the hand
CAM_OFF = np.array([0.180, -0.120, 0.140])
CAM_TGT = np.array([0.0, -0.015, -0.005])

# ── trajectory ──────────────────────────────────────────────────────────────────────────────────────────────────────
NOSE_START = 0.010          # m: start with the nose this far above the mouth
SEAT_TO_MOUTH = EM.MOUTH_Y - EM.BACK_WALL_Y      # 15.0 mm: the nose's depth at the seat
RAISE = SEAT_TO_MOUTH + NOSE_START              # plug height above its seat at the start
V_INS = 0.012               # m/s in free air
V_ENTRY = 0.003             # m/s through the mouth
V_BODY = 0.006              # m/s while the body slides in, before the lips touch the tab
V_LATCH = 0.003             # m/s while the lips press the tab
V_SEAT = 0.002              # m/s for the last SEAT_ZONE
ENTRY_D = (SEAT_TO_MOUTH - 0.002, SEAT_TO_MOUTH + 0.003)    # plug height above the seat: the nose in the mouth
LATCH_D = 0.0105            # the lips meet the tab 9.9 mm above the seat (mechanism's latch_kinematics)
SEAT_ZONE = 0.0015
ACC = 0.06                  # m/s^2 slew on the commanded speed
K_ARM = 90000.0             # N/m, the arm's in-scene stiffness (lead -> pushing force)
LEAD_MIN = 0.00025          # m the command may lead the measured hand (free air, body sliding)
LEAD_LATCH = 0.00035        # ... while the lips press the tab (~14 N peak in the probe -> 1.3 x 14 / K_ARM = 0.2 mm)
LEAD_SEAT = 0.00015
OVERTRAVEL = 0.0015         # m the hand command may go past the nominal seat
SEAT_TOL = 0.0002           # seated: the plug body reads ~0.1 mm above the CAD seat on the back wall
# STALL PUSH.  The lead is a force only through the arm's stiffness at this pose, which varies over the seeded square:
# at the far edge the bounded lead pushed ~6 N and the plug, tipped 1.4 deg in the pinch, stopped 0.55 mm short of the
# back wall, or crept in so slowly that the insertion timed out 0.35 mm short.  Below the lips' first touch, while the
# plug moves at less than STALL_FRACTION of the commanded speed, the lead grows by R_EXTRA per frame up to EXTRA_MAX,
# but only while the measured slot reaction is below F_PUSH_MAX.
R_EXTRA = 0.000005          # m/frame (0.3 mm/s)
EXTRA_MAX = 0.00035
F_PUSH_MAX = 25.0           # N (pinch capacity 2 x 1.4 x 15 = 42 N)
STALL_WINDOW, STALL_FRACTION = 0.12, 0.4        # s, share of the commanded travel over the window
T_GIVE_UP = 0.6             # s stalled at the push limits before the policy holds where it is
F_SEAT_HOLD = 4.0           # N the seat press is bled to
R_SEAT = 0.000015           # m/frame of the seat servo (0.9 mm/s)
R_RESEAT = 0.000035         # m/frame (2.1 mm/s) back down to the seat after the pull (lead-bounded like the seat)
F_TRY = 10.0                # N cap of the light pull (the tab untouched)
F_TRY_MIN = 6.0             # N the latch must take, for T_TRY_HOLD
T_TRY_HOLD = 0.5
R_TRY = 0.000025            # m/frame (1.5 mm/s) of command escalation: once the catch takes the load each step is
                            # ~K_ARM x step of pull (0.06 mm/frame overshot a 10 N cap to 16 N)
PULL_LEAD = 0.0025          # m the pull command may stretch past the plug
ALIGN_KP, ALIGN_DB, ALIGN_CLAMP, ALIGN_ALPHA = 0.25, 0.00003, 0.004, 0.3
ALIGN_STEP = 0.00003        # m/frame cap on the align correction
ALIGN_FREEZE_NOSE = -0.0005  # freeze once the nose is 0.5 mm into the mouth
PRE_ROLL = 0.30
T_GRIP0 = 0.5               # recorded: holding the plug above the port
T_ALIGN = 0.6
T_INS_MAX = 14.0
T_HOLD = 0.6
T_TRY = 2.0
T_RESEAT = 1.0
T_OPEN, T_RETREAT, T_STILL = 0.6, 1.4, 1.2
RETREAT = 0.06

# appendix thresholds
SEAT_TOL_APPX = 0.0003      # fully inserted: within 0.3 mm of the CAD seat (nose on the back wall) while held
SNAP_MIN_DEG = 12.0         # the tab must be pressed past the lips on the way in (geometry: 15.2 deg) ...
REST_MAX_DEG = 8.0          # ... and be back at its rest angle (5.0 deg seated, 3.9 deg on the catch)
RISE_MAX = 0.0015           # light pull: the latch's play is 0.85 mm
BACKOFF_MAX = 0.0012        # after release the plug may back off onto its catch (0.85 mm + contact compliance)
PEN_TOL = 0.0004


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, choices=(1, 2, 3, 4, 5), default=PORT,
                    help="target port (1..5 along the shell; 5, the default, is farthest from the camera)")
    ap.add_argument("--seed", type=int, default=None, help="random slot pose (0.15 x 0.15 m square, +-20 deg)")
    ap.add_argument("--offset-mm", type=str, default="0.5,-0.6",
                    help="start xy offset of the plug from the port axis (mm), the misalignment the expert aligns away")
    ap.add_argument("--name", type=str, default=None, help="output folder name")
    ap.add_argument("--check", action="store_true", help="build, settle 0.5 s, print the grasp and exit")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    name = a.name or NAME
    off_xy = np.array([float(v) for v in a.offset_mm.split(",")]) / 1000.0
    start = sample_start({"slot": SLOT_PLACE}, a.seed)["slot"]
    kit = load_kit()

    rig = appendix_rig(name)
    rig.substeps = SUBSTEPS
    rig.add_robot(wp.transform(wp.vec3(0.0, ROBOT_BASE_Y, 0.0), wp.quat_identity()))
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        set_body_mass(rig.builder, bi, FINGER_MASS)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU
    rig.table(center_xy=(0.0, 0.15), half=0.38)

    # ── the slot (shell + five inserts) and the plug above the target port ──────────────────────────────────────────
    yaw = float(start[2])
    Rz = trimesh.transformations.rotation_matrix(yaw, (0.0, 0.0, 1.0))[:3, :3]
    slot_pose = place_on_table(KEY, xy=(start[0], start[1]), R=Rz @ R_UP)
    out_axis = EM.port_axis(slot_pose)                         # +z: the ports face up
    T_seat = slot_pose @ EM.seat_transform(a.port)             # world <- plug CAD frame, fully inserted
    T_start = pose_matrix(xyz=np.r_[off_xy, 0.0] + RAISE * out_axis) @ T_seat
    h = add_asset(rig, KEY, pose=slot_pose, plug="free", plug_pose=T_start, label="ethernet")
    plug_lbl, tab_lbl = "ethernet/plug", "ethernet/plug/tab"
    pb, tb = h.bodies[plug_lbl], h.bodies[tab_lbl]
    plug_sids = h.shapes[f"{plug_lbl}/plug_body"] + h.shapes[f"{plug_lbl}/plug_root"]
    tab_sid = h.shapes[f"{plug_lbl}/plug_tab"][0]
    slot_sids = h.shapes["ethernet_slot_shell"] + [h.shapes[f"insert_port{i}"][0] for i in range(1, 6)]
    pieces = EM._plug_pieces()
    c_body = 0.5 * (pieces["body"].bounds[0] + pieces["body"].bounds[1])
    seat_body = (T_seat @ np.r_[c_body, 1.0])[:3]              # plug body origin at the seat
    mouth = EM.mouth_point(slot_pose, a.port)

    # ── arm start: pads on the rear block's flanks, closing along the shell's long axis ──────────────────────────────
    proud = EM.REAR_Y[0] + EM.SEAT_DY - EM.MOUTH_Y               # rear block's front face above the shell face, seated
    tip_y = EM.REAR_Y[0] + (TIP_ABOVE_FACE - proud)             # pad-tip height on the plug (CAD y)
    z_mid = 0.5 * (EM.REAR_Z[0] + EM.REAR_Z[1])                 # pad centre across the block
    tcp0 = (T_start @ np.r_[EM.PLUG_X, tip_y + TIP_DROP, z_mid, 1.0])[:3]
    q1 = down_grasp_quat(yaw)                                   # fingers close along the shell's long axis
    rig.solve_arm_start(tcp0, q1)
    f_touch = EM.PLUG_HALF_W + PAD_OFF - 0.0001
    f_pinch = f_touch - PINCH_N / FINGER_KE
    f_open = EM.PLUG_HALF_W + PAD_OFF + OPEN_EXTRA
    B = rig.builder
    B.joint_q[7:9] = [f_touch, f_touch]
    B.joint_target_q[7:9] = [f_pinch, f_pinch]
    rig._filter_arm_from(pb, filter_fingers=False)
    rig._filter_arm_from(tb, filter_fingers=False)

    rig.finalize(probes=[plug_lbl], cam_pos=mouth + CAM_OFF, cam_target=mouth + CAM_TGT)
    rig.solver = make_solver(rig.model, iterations=ITERS, impratio=IMPRATIO)
    rig.probes["plug_slot"] = PairProbe(rig.model, plug_lbl, counterpart_shapes=slot_sids)
    rig.probes["tab_slot"] = PairProbe(rig.model, tab_lbl, counterpart_shapes=slot_sids)
    rig.probes["finL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", counterpart_shapes=plug_sids + [tab_sid])
    rig.probes["finR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", counterpart_shapes=plug_sids + [tab_sid])
    shape_lbl = list(rig.model.shape_label)
    rig.probes["robot_object"] = RobotContactProbe(rig.model, [plug_lbl, tab_lbl],
                                                   static_shapes=[shape_lbl[s] for s in slot_sids])
    pi_, ti_ = rig.body_index(plug_lbl), rig.body_index(tab_lbl)
    lf_i = rig.body_index("fr3/fr3_leftfinger")
    rf_i = rig.body_index("fr3/fr3_rightfinger")
    tip_pts = _finger_box_corners(rig.model, (lf_i, rf_i))

    # unrecorded pre-roll: the pads close to the working pinch on the rear block
    rig.drive_arm(tcp0, q1, fingers=f_pinch)
    sim_dt = 1.0 / rig.fps / rig.substeps
    for _ in range(int(PRE_ROLL * rig.fps)):
        rig.s0.clear_forces()
        rig.pipeline.collide(rig.s0, rig.contacts)
        for _ in range(rig.substeps):
            rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
            rig.s0, rig.s1 = rig.s1, rig.s0

    def plug_q() -> np.ndarray:
        return rig.s0.body_q.numpy()[pi_]

    def height(q=None) -> float:
        """Plug height above its seat along the port axis (0 = fully inserted)."""
        return float(np.dot((plug_q() if q is None else q)[:3] - seat_body, out_axis))

    def lateral(q=None) -> np.ndarray:
        """Plug offset from the port axis (world xy)."""
        p = (plug_q() if q is None else q)[:3] - seat_body
        return (p - np.dot(p, out_axis) * out_axis)[:2]

    def tab_deg(bq=None) -> float:
        bq = rig.s0.body_q.numpy() if bq is None else bq
        return float(np.degrees(EM.tab_angle(bq, pi_, ti_)))

    def slot_out() -> float:
        """Slot force on the plug + tab along the port axis (N, + = pushing it out), mean of the last 3 frames."""
        f = 0.0
        for n in ("plug_slot", "tab_slot"):
            hst = rig.probes[n].history[-3:]
            if hst:
                f += float(np.mean([np.dot(r["vec"], out_axis) for r in hst]))
        return f

    def last(n) -> float:
        hst = rig.probes[n].history
        return hst[-1]["force"] if hst else 0.0

    if a.check:
        print(json.dumps({"plug_height_mm": round(height() * 1e3, 3), "lateral_mm": np.round(lateral() * 1e3, 3).tolist(),
                          "tab_deg": round(tab_deg(), 2), "pads_N": [round(last("finL"), 2), round(last("finR"), 2)],
                          "start": {"seed": a.seed, "xy_mm": [round(v * 1e3, 1) for v in start[:2]],
                                    "yaw_deg": round(float(np.degrees(yaw)), 2)}}))

    # ── control ─────────────────────────────────────────────────────────────────────────────────────────────────────
    st = {"phase": "hold0", "t0": 0.0, "z": 0.0, "ramp": 0.0, "align": np.zeros(2), "err": np.zeros(2),
          "frozen": False, "rel_tcp": None, "t_seat": None, "hist": [], "t_release": None, "t_retreat_end": None,
          "off": None, "v": 0.0, "try": {}, "reseated": False, "extra": 0.0, "extra_peak": 0.0, "t_stuck": None}
    frame_dt = 1.0 / rig.fps

    def align_step():
        if st["frozen"]:
            return
        e = -lateral()
        st["err"] = ALIGN_ALPHA * e + (1 - ALIGN_ALPHA) * st["err"]
        step = np.where(np.abs(st["err"]) > ALIGN_DB, ALIGN_KP * st["err"], 0.0)
        step = np.clip(step, -ALIGN_STEP, ALIGN_STEP)
        st["align"] = np.clip(st["align"] + step, -ALIGN_CLAMP, ALIGN_CLAMP)

    def cmd_pos() -> np.ndarray:
        return tcp0 + np.r_[st["align"], 0.0] + st["z"] * out_axis

    def go(phase, t):
        st["phase"], st["t0"] = phase, t
        print(f"[ethernet] t={t:5.2f}s -> {phase}  h={height()*1000:+.3f} mm  slotF={slot_out():.2f} N  "
              f"tab={tab_deg():.2f} deg")

    def hand_cmd() -> float:
        """Measured finger-midpoint height in command coordinates (offset fixed when the insertion starts)."""
        bq = rig.s0.body_q.numpy()
        hz = 0.5 * float(bq[lf_i, 2] + bq[rf_i, 2])
        if st["off"] is None:
            st["off"] = st["z"] - hz
        return hz + st["off"]

    def seat_servo():
        Fs = slot_out()
        if Fs > F_SEAT_HOLD + 1.0:
            st["z"] = min(st["z"] + R_SEAT, -RAISE + 0.004)
        elif Fs < 0.5 * F_SEAT_HOLD:
            st["z"] = max(st["z"] - R_SEAT, -RAISE - OVERTRAVEL)

    def release(t):
        st["rel_tcp"] = cmd_pos()
        st["t_release"] = t
        go("open", t)

    def progress(t) -> float:
        """How far the plug went in over the last STALL_WINDOW (m)."""
        hh = [x for (tt, x) in st["hist"] if tt > t - STALL_WINDOW]
        return hh[0] - hh[-1] if len(hh) >= 6 else 1.0

    def stall_push(t, d, v_cmd) -> bool:
        """Grow the extra lead while the plug lags the commanded speed ``v_cmd`` below the lips' first touch; True =
        stuck at the limits for T_GIVE_UP."""
        F = slot_out()
        stuck = d > SEAT_TOL and progress(t) < STALL_FRACTION * v_cmd * STALL_WINDOW
        if stuck and F < F_PUSH_MAX:
            st["extra"] = min(st["extra"] + R_EXTRA, EXTRA_MAX)
        elif F > F_PUSH_MAX + 3.0:
            st["extra"] = max(st["extra"] - R_EXTRA, 0.0)
        st["extra_peak"] = max(st["extra_peak"], st["extra"])
        at_limit = stuck and (st["extra"] >= EXTRA_MAX or F >= F_PUSH_MAX)
        if not at_limit:
            st["t_stuck"] = None
        elif st["t_stuck"] is None:
            st["t_stuck"] = t
        return at_limit and t - st["t_stuck"] >= T_GIVE_UP

    def tip_z(bq) -> float:
        """Lowest point of the two fingers' collision boxes (world z)."""
        return min(float((_tf(bq[b]) @ c.T)[2].min()) for b, c in tip_pts.items())

    def control(t):
        d = height()
        st["hist"].append((t, d))
        ph = st["phase"]
        nose = d - SEAT_TO_MOUTH                   # nose height above the mouth
        if ph == "hold0":
            align_step()
            if t >= T_GRIP0:
                go("align", t)
        elif ph == "align":
            align_step()
            if (t >= st["t0"] + T_ALIGN and np.all(np.abs(st["err"]) < 0.00008)) or t >= st["t0"] + 2.5 * T_ALIGN:
                go("insert", t)
        elif ph == "insert":
            if not st["frozen"]:
                align_step()
                if nose < ALIGN_FREEZE_NOSE:
                    st["frozen"] = True
                    print(f"[ethernet] t={t:5.2f}s align frozen at {np.round(st['align'] * 1e3, 3).tolist()} mm, "
                          f"plug lateral {np.round(lateral() * 1e3, 3).tolist()} mm")
            if d > ENTRY_D[1]:
                v_des, lead = V_INS, LEAD_MIN
            elif d > ENTRY_D[0]:
                v_des, lead = V_ENTRY, LEAD_MIN
            elif d > LATCH_D:
                v_des, lead = V_BODY, LEAD_MIN
            elif d > SEAT_ZONE:
                v_des, lead = V_LATCH, LEAD_LATCH
            else:
                v_des, lead = V_SEAT, LEAD_SEAT
            brake = 0.5 * st["v"] ** 2 / ACC                 # look ahead so slow zones are entered at their speed
            for edge, v_zone in ((ENTRY_D[1], V_ENTRY), (LATCH_D, V_LATCH), (SEAT_ZONE, V_SEAT)):
                if d > edge and d - brake < edge:
                    v_des = min(v_des, v_zone)
            st["v"] += float(np.clip(v_des - st["v"], -ACC * frame_dt, ACC * frame_dt))
            gave_up = stall_push(t, d, st["v"]) if d < LATCH_D else False
            hc = hand_cmd()
            st["ramp"] = max(st["ramp"] - st["v"] * frame_dt, hc - lead - st["extra"], -(RAISE + OVERTRAVEL))
            st["z"] = st["ramp"]
            if d < SEAT_TOL or gave_up or t >= st["t0"] + T_INS_MAX:
                st["t_seat"] = t
                st["z"] = hand_cmd() - LEAD_MIN * 0.5
                go("hold", t)
        elif ph == "hold":
            seat_servo()
            st["extra"], st["t_stuck"] = 0.0, None
            if t >= st["t0"] + T_HOLD:
                st["try"] = {"off0": st["z"] - d}
                go("try_pull", t)
        elif ph == "try_pull":                      # the light pull, tab untouched
            F = -slot_out()                         # the latch holding the plug in (N)
            z_cap = d + st["try"]["off0"] + PULL_LEAD
            if F < F_TRY:
                rate = R_TRY * float(np.clip((F_TRY - F) / 5.0, 0.2, 1.0))
                st["z"] = max(st["z"], min(st["z"] + rate, z_cap))
            elif F > F_TRY + 1.5:
                st["z"] -= R_TRY
            if t >= st["t0"] + T_TRY:
                go("reseat", t)
        elif ph == "reseat":                        # back down onto the back wall (the pull left it on the catch)
            if not st["reseated"]:
                gave_up = stall_push(t, d, R_RESEAT * rig.fps)
                st["z"] = max(st["z"] - R_RESEAT, hand_cmd() - LEAD_SEAT - st["extra"], -RAISE - OVERTRAVEL)
                if d < SEAT_TOL or gave_up:
                    st["reseated"] = True
            else:
                seat_servo()
            if (t >= st["t0"] + T_RESEAT and st["reseated"]) or t >= st["t0"] + 3.0 * T_RESEAT:
                release(t)
        elif ph == "open":
            u = ease((t - st["t0"]) / T_OPEN)
            rig.drive_arm(st["rel_tcp"], q1, fingers=f_pinch + (f_open - f_pinch) * u)
            if t >= st["t0"] + T_OPEN:
                go("retreat", t)
            return
        elif ph == "retreat":
            u = ease((t - st["t0"]) / T_RETREAT)
            rig.drive_arm(st["rel_tcp"] + RETREAT * u * out_axis, q1, fingers=f_open)
            if t >= st["t0"] + T_RETREAT:
                st["t_retreat_end"] = t
                go("still", t)
            return
        elif ph == "still":
            rig.drive_arm(st["rel_tcp"] + RETREAT * out_axis, q1, fingers=f_open)
            if t >= st["t0"] + T_STILL:
                raise StopRun
            return
        rig.drive_arm(cmd_pos(), q1, fingers=f_pinch)

    log = []
    PHASES = {"hold0": 0, "align": 1, "insert": 2, "hold": 3, "try_pull": 4, "reseat": 5, "open": 6, "retreat": 7,
              "still": 8}

    def extra(state, t):
        bq = state.body_q.numpy()
        d = height(bq[pi_])
        lat = lateral(bq[pi_])
        hp, ht = rig.probes["plug_slot"].history, rig.probes["tab_slot"].history
        fo_p = hp[-1]["vec"] if hp else [0.0, 0.0, 0.0]
        fo_t = ht[-1]["vec"] if ht else [0.0, 0.0, 0.0]
        F_out = float(np.dot(fo_p, out_axis) + np.dot(fo_t, out_axis))
        F_tab = float(np.linalg.norm(fo_t))
        fL, fR = last("finL"), last("finR")
        rc = last("robot_object")
        hz = float(0.5 * (bq[lf_i, 2] + bq[rf_i, 2]))
        tz = tip_z(bq)
        ang = tab_deg(bq)
        x, y, z, w = bq[pi_, 3:7]
        ax = np.array([2 * (x * y - w * z), 1 - 2 * (x * x + z * z), 2 * (y * z + w * x)])   # plug CAD +y in world
        tilt = float(np.degrees(np.arccos(np.clip(np.dot(ax, out_axis), -1, 1))))
        log.append((t, d, lat[0], lat[1], F_out, F_tab, fL, fR, rc, hz, PHASES[st["phase"]], ang, tilt, st["z"],
                    tz))
        return {"plug_height_mm": round(d * 1000, 3), "slot_axial_N": round(F_out, 3), "tab_slot_N": round(F_tab, 3),
                "tab_deg": round(ang, 3), "finL_N": round(fL, 3), "finR_N": round(fR, 3),
                "plug_dx_mm": round(float(lat[0]) * 1000, 4), "plug_dy_mm": round(float(lat[1]) * 1000, 4),
                "plug_tilt_deg": round(tilt, 3), "robot_object_contact_N": round(rc, 4),
                "cmd_dz_mm": round(st["z"] * 1000, 3)}

    budget = (T_GRIP0 + 2.5 * T_ALIGN + T_INS_MAX + T_HOLD + T_TRY + 3.0 * T_RESEAT + T_OPEN + T_RETREAT + T_STILL
              + 0.5)
    if a.check:
        budget = 0.5
    try:
        rec = rig.run(budget, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[ethernet] ending complete -> stopped at t={len(rec.times) / rig.fps:.2f}s")
    paths = rec.save("rollout", force_keys=["slot_axial_N", "tab_slot_N", "finL_N", "finR_N", "tab_deg",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    if a.check:
        Lg = np.array(log)
        print("check: height drift", round((Lg[-1, 1] - Lg[0, 1]) * 1000, 3), "mm; pads",
              round(Lg[:, 6].mean(), 2), round(Lg[:, 7].mean(), 2), "N; tab", round(Lg[-1, 11], 2), "deg")
        return None

    # ── QA ──────────────────────────────────────────────────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    Lg = np.array(log)
    (t_a, d_a, dx_a, dy_a, F_a, Ft_a, gL, gR, rc_a, hz_a, ph_a, ang_a, tilt_a, cmd_a, tip_a) = (
        Lg[:, k] for k in range(Lg.shape[1]))
    dt = 1.0 / rig.fps
    hold0 = ph_a == 0
    drop0 = float(d_a[hold0].max() - d_a[hold0].min()) if hold0.any() else 0.0
    grip0 = float(min(gL[hold0].mean(), gR[hold0].mean())) if hold0.any() else 0.0
    ins, hold, tryp = ph_a == 2, ph_a == 3, ph_a == 4
    held = ins | hold | tryp
    t_rel = st["t_release"] if st["t_release"] is not None else t_a[-1]
    hw = hold & (t_a >= (t_a[hold].max() - 0.25 if hold.any() else 0))
    seat_d = float(np.median(d_a[hw])) if hw.any() else float("nan")
    tab_seat = float(np.median(ang_a[hw])) if hw.any() else float("nan")
    ins_peak = float(np.max(F_a[ins])) if ins.any() else 0.0
    snap = float(np.max(ang_a[ins | hold])) if (ins | hold).any() else 0.0
    tilt_max = float(np.max(tilt_a[ins | hold])) if (ins | hold).any() else 0.0
    grip_ins = {"finL": float(gL[ins].mean()) if ins.any() else 0.0, "finR": float(gR[ins].mean()) if ins.any() else 0.0}
    F_hold = -F_a
    try_peak = float(F_hold[tryp].max()) if tryp.any() else 0.0
    try_s = float((F_hold[tryp] >= F_TRY_MIN).sum() * dt)
    try_rise = float((d_a[tryp] - seat_d).max()) if tryp.any() else float("inf")
    pre_rel = (t_a >= t_rel - 0.2) & (t_a < t_rel)
    d_pre = float(np.mean(d_a[pre_rel])) if pre_rel.any() else float("nan")
    final_d = float(np.median(d_a[-20:]))
    final_tab = float(np.median(ang_a[-20:]))
    t_still = st["t_retreat_end"] if st["t_retreat_end"] is not None else t_a[-1]
    face_z = float(mouth[2])
    crit = {
        "held_in_hand_at_start": {
            "pass": bool(drop0 < 0.001 and grip0 > 3.0),
            "plug_drop_in_hand_mm": round(drop0 * 1000, 3), "min_mean_pad_force_N": round(grip0, 2)},
        "fully_inserted": {
            "pass": bool(abs(seat_d) < SEAT_TOL_APPX),
            "seat_height_from_cad_seat_mm": round(seat_d * 1000, 3), "tolerance_mm": SEAT_TOL_APPX * 1000,
            "insertion_axial_peak_N": round(ins_peak, 2), "plug_tilt_max_deg": round(tilt_max, 2),
            "extra_lead_peak_mm": round(st["extra_peak"] * 1000, 3),
            "pad_tips_above_shell_face_min_mm": round((float(np.min(tip_a[held])) - face_z) * 1000, 2)
            if held.any() else None,
            "note": "seat = the nose on the insert's back wall (the CAD pose, 22.0 mm of travel from the plug's CAD "
                    "assembly pose), measured while the hand holds the plug seated"},
        "latch_engaged": {
            "pass": bool(snap >= SNAP_MIN_DEG and tab_seat < REST_MAX_DEG and final_tab < REST_MAX_DEG),
            "tab_max_deg_on_the_way_in": round(snap, 2), "snap_min_deg": SNAP_MIN_DEG,
            "tab_seated_deg": round(tab_seat, 2), "tab_final_deg": round(final_tab, 2), "rest_max_deg": REST_MAX_DEG,
            "geometry_deg": {"pass_the_lips": round(float(np.degrees(EM.PASS_RAD)), 1),
                             "rest_seated": round(float(np.degrees(EM.SEAT_TAB_RAD)), 2),
                             "stop": round(float(np.degrees(EM.TAB_LIMIT[1])), 2)},
            "note": "the lips press the tab past them on the way in and it springs back behind them; the robot never "
                    "touches it"},
        "light_pull_held": {
            "pass": bool(try_peak >= F_TRY_MIN and try_s >= T_TRY_HOLD and try_rise < RISE_MAX),
            "pull_peak_N": round(try_peak, 2), "pull_cap_N": F_TRY, "time_at_min_pull_s": round(try_s, 2),
            "min_pull_N": F_TRY_MIN, "rise_mm": round(try_rise * 1000, 3), "rise_max_mm": RISE_MAX * 1000,
            "note": "force-capped pull through the pads with the tab untouched; the rise is the latch's 0.85 mm of "
                    "play plus contact compliance"},
        "stays_latched_after_release": {
            "pass": bool(final_d - d_pre < BACKOFF_MAX and final_d < BACKOFF_MAX and final_tab < REST_MAX_DEG),
            "height_before_release_mm": round(d_pre * 1000, 3), "final_height_mm": round(final_d * 1000, 3),
            "backoff_max_mm": BACKOFF_MAX * 1000,
            "final_lateral_mm": [round(float(np.median(dx_a[-20:])) * 1000, 3),
                                 round(float(np.median(dy_a[-20:])) * 1000, 3)],
            "note": "released, the plug may back off onto its catch (the shoulder against the lips: 0.85 mm), never "
                    "further"},
    }
    check_released_and_clear(qa, t_a, rc_a, t_still, extra={
        "hand_rise_mm": round(float(hz_a[-1] - hz_a[t_a < t_rel][-1]) * 1000, 1) if (t_a < t_rel).any() else None})
    rc_entry = qa.report["checks"]["released_and_clear"]
    moved = rc_entry.get("hand_rise_mm") or 0.0
    rc_entry["min_move_mm"] = 40.0
    rc_entry["pass"] = bool(rc_entry["pass"] and moved >= 40.0)
    crit["released_and_clear"] = rc_entry
    crit["finger_pinch_real"] = {
        "pass": bool(min(grip_ins.values()) >= 3.0),
        "mean_pad_force_insert_N": {k: round(v, 2) for k, v in grip_ins.items()}, "pinch_set_N": PINCH_N}
    for k, c in crit.items():
        qa.report["checks"].setdefault(k, c)
    # penetration of the plug body and its tab into the exact CAD insert and shell (body frames: the crops centred on
    # their bodies' origins)
    body_local = pieces["body"].copy()
    body_local.apply_translation(-c_body)
    tab_local = pieces["tab"].copy()
    tab_local.apply_translation(-EM.pivot_cad())
    parts = {p.name: p.mesh() for p in kit[KEY].parts()}
    ins_world = parts["ethernet_slot"].copy()
    ins_world.apply_transform(slot_pose @ pose_matrix(xyz=(EM.PORT_X[a.port - 1], 0.0, 0.0)))
    shell_world = parts["ethernet_slot_shell"].copy()
    shell_world.apply_transform(slot_pose)
    for lbl, local, nm in ((plug_lbl, body_local, "plug"), (tab_lbl, tab_local, "tab")):
        qa.check_penetration_static(lbl, local, ins_world, np.zeros(3), f"{nm}->insert", tol=PEN_TOL, n_samples=48)
        qa.check_penetration_static(lbl, local, shell_world, np.zeros(3), f"{nm}->shell", tol=PEN_TOL, n_samples=48)
    lo = mouth - np.array([0.06, 0.06, 0.04])
    hi = mouth + np.array([0.06, 0.06, 0.08])
    qa.check_envelope([plug_lbl, tab_lbl], lo, hi)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    crit["start"] = {"pass": True, "seed": a.seed, "port": a.port,
                     "slot_xy_mm": [round(float(v) * 1e3, 2) for v in start[:2]],
                     "slot_yaw_deg": round(float(np.degrees(yaw)), 2),
                     "plug_offset_mm": [round(float(v) * 1e3, 2) for v in off_xy],
                     "range": {"xy_mm": round(SLOT_PLACE.dxy * 1e3, 1),
                               "yaw_deg": round(float(np.degrees(SLOT_PLACE.dyaw)), 1)}}
    changes = [
        f"kit '{KEY}' (force_bench.kit; model force_bench.kit.mechanisms.ethernet): the 5-port shell lying on its side "
        "with an insert in every port and the printed plug without a cable, as in the real-world video; the plug goes "
        f"into port {a.port}",
        "starts IN HAND with the nose 10 mm above the mouth, offset "
        f"({off_xy[0] * 1e3:+.1f}, {off_xy[1] * 1e3:+.1f}) mm from the port axis",
        f"printed latch tab = a hinged body on a spring drive at {h.params['ke_N_m_per_rad']:.2f} N.m/rad, an ESTIMATE "
        "(finite-element model of the printed tab, E = 3 GPa; no measurement exists); it catches behind the inserts' "
        "lips by contact",
        "plug mass 15 g and a hinge inertia of 3e-7 kg.m^2, conditioning values (print estimates 1.8 g, 1.6e-8): "
        "MuJoCo's contact stiffness scales with the moving mass",
        "light pull (10 N cap, tab untouched) through the pads, reseat, release at the seat, 6 cm retreat, still",
    ]
    if a.seed is not None:
        changes.append(f"seeded start {a.seed}: slot at ({start[0] * 1e3:+.1f}, {start[1] * 1e3:+.1f}) mm, "
                       f"yaw {np.degrees(yaw):+.1f} deg (uniform in the paper's 0.15 x 0.15 m square, +-20 deg)")
    return save_appendix_verdict(rig, task="Plug in Ethernet", source=f"new task (kit {KEY}, port {a.port})",
                                 spec=CRITERIA[25], criteria=crit, physics_qa=report, changes=changes)


def _tf(row) -> np.ndarray:
    T = trimesh.transformations.quaternion_matrix([row[6], row[3], row[4], row[5]])
    T[:3, 3] = row[:3]
    return T


def _finger_box_corners(model, bodies) -> dict[int, np.ndarray]:
    """Homogeneous corners (body frame, 4 x 8n) of the collision boxes on each finger body."""
    import newton
    sb, st = model.shape_body.numpy(), model.shape_type.numpy()
    sc, tf = model.shape_scale.numpy(), model.shape_transform.numpy()
    out = {}
    for b in bodies:
        pts = []
        for s in np.nonzero((sb == b) & (st == int(newton.GeoType.BOX)))[0]:
            hx, hy, hz = sc[s]
            c = np.array([[x, y, z, 1.0] for x in (-hx, hx) for y in (-hy, hy) for z in (-hz, hz)])
            pts.append((_tf(tf[s]) @ c.T).T)
        out[b] = np.vstack(pts)
    return out


if __name__ == "__main__":
    wp.init()
    main()
