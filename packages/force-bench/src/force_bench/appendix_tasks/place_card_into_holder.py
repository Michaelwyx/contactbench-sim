"""Appendix task "Place card into holder" (Table I, #24) -- NEW TASK, the
suite has no counterpart (it is the reverse of ``tasks/card_pick``).

    Description: "A card holder that already contains other cards is placed
    in the workspace. Place the card held in hand into the card holder."
    Success: "The robot movement stops after placing the card in the card
    holder, with existing cards remaining in the card holder. The robot
    should not remain in contact with the task objects."

SCENE (``_card_place.py``: assets, slot geometry, physics model, why the
deck is 3 cards at 12 deg): the finalized-kit holder on the table with
three cards leaning toward -x on its far bracket posts; a fourth kit card
is PINCHED in the FR3's hand at the start, vertical, 7.5 cm to the free
side of the holder -- the appendix's "card held in hand" (no pre-roll: the
pads start touching its faces and the grip forms in the first frames,
which the verdict checks).

EXPERT (one rollout; the card is moved only by the two fingertip pads; the
expert reads the card's and the deck's poses from the simulator):
  1. hold      0.6 s, robot still: the grip servo settles the pinch (pads
               force-servoed to 2.5-4 N on their probes) and the deck shows
               it is at rest;
  2. carry     to a hover 60 mm up the deck's own plane above the seat,
               turning the card to the deck's MEASURED lean so that it
               comes down parallel to the front card, 0.5 mm off its face;
  3. align     the card's MEASURED position is servoed (droop/slip integral
               on the TCP) until it is within 0.25 mm of its path in y (the
               85 mm card has 0.75 mm of play per end between the slot's
               end walls) and in the face-normal direction;
  4. descend   down the deck plane, 20 mm/s, then 10 mm/s from 40 mm above
               the seat (before the bottom edge reaches the z = 37 bracket
               tops), the y / face-normal servo still running and the
               descent frozen if the card touches the holder or the deck
               with more than 0.5 N;
  5. release   when the deck-side fingertip is 2 mm above the deck's top
               edges.  This is the one place the bare FR3 differs from a
               hand: a face pinch puts one fingertip (15.2 mm thick) BETWEEN
               the card and the deck, so the pads must open while the
               card's lower edge is still ~9 mm above the floor -- already
               inside the slot, below the side-wall tops.  The pads open
               1.5 mm per side and the card slides down the front card's
               face onto the floor (the person in the reference video
               pushes it the last bit down with a finger);
  6. settle, retreat 40 mm up the card plane and 50 mm up, stand still.

VERDICT (appendix_qa.json):
  held_in_hand_at_start   over the hold the card moves < 0.5 mm in the
                          hand's own (link7) frame, both pads loaded,
                          nothing else touches it
  held_through_transport  carry/align/descent: < 1 mm of slip in the hand,
                          both pads loaded on >= 98 % of the frames
  released_inside_slot    at the release the card's whole lower edge is
                          inside the slot walls and below their tops
  card_in_holder          final: all four bottom corners inside the slot
                          walls / end walls, lowest point on the floor
                          (within 0.5 mm), leaning in the stack (lean within
                          3 deg of the front card's, face gap < 0.6 mm), at
                          rest (< 2 mm/s over the final window)
  existing_cards_remain   every pre-loaded card moved < 3 mm and < 3 deg,
                          still inside the slot and on its floor
  released_and_clear      robot <-> cards/holder contact < 0.05 N over the
                          final window, hand moved >= 50 mm from the release
  robot_stopped           arm links < 5 mm/s over the final window
  + physics integrity: finite, flying-object velocity, penetration (dense
    ray-verified audit: every card vs the CAD holder both ways, every card
    pair, 0.8 mm), workspace envelope.

RESULTS (2026-10-16, publication render, outputs/appendix_tasks/
place_card_into_holder/; PASS): the pinch holds 2.6 N per pad with 0.005 mm
of slip over the hold and 0.064 mm through the transport; aligned within
0.10 / 0.06 mm (face normal / y) at the hover; released at t = 7.77 s with
the deck-side tip 1.84 mm over the deck and the card's lower edge 9.4 mm up
(inside the slot, below the 17 mm wall tops); it slides 7.4 mm and lands on
the floor 0.25 s later at 0.16 m/s; final: on the floor (-0.001 mm), all
corners inside the slot (x 4.8..6.1 mm against the +x wall at 7.0), lean
12.74 deg against the front card's 12.12 deg, face gap 0.34 mm, at rest; the
three pre-loaded cards moved <= 0.013 mm / 0.07 deg; hand 89.5 mm away, 0 N.
Worst penetration 0.003 mm (card on the floor), peak card speed 0.16 m/s.
Repeats: identical up to the release on every run; after it the free slide
varies (final lean 12.17-12.74 deg, face gap 0.14-0.34 mm over 4 runs).  A
start 3.5 cm further out and 3 cm higher passes the same way.

Run:  python -m force_bench.appendix_tasks.place_card_into_holder
      (TB_OUT_ROOT=<dir> for iterations; TB_RENDER_FAST=1 TB_MULTIVIEW=0;
       CP_STOP_T=<s> stops early and skips the QA; CP_PREVIEW=<states.npz>
       renders a camera contact sheet instead of simulating)
"""

from __future__ import annotations

import json
import os

import numpy as np
import warp as wp

# GelSight readiness: the tactile layer must be imported before the rig
# (it is the identity unless TB_GELSIGHT=1; finger targets below add
# GS.finger_offset()).
import force_bench.tactile.task as GS
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.assets import set_body_mass
from force_bench.geometry import world_T
from force_bench.robot.franka import down_grasp_quat
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, save_appendix_verdict, check_released_and_clear,
    StopRun)
from force_bench.appendix_tasks import _card_place as C
from force_bench.appendix_tasks._hang_plate import (
    PairProbe, JointTracker, tcp_offset_from_link7, set_camera_fov, preview_cameras,
    parse_cams)

NAME = "place_card_into_holder"
MM = 1000.0


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


# ── the cell (card_pick's: base 0.5 m along the slot axis; IK margin
#    >= 0.73 rad on every key pose, scanned offline) ───────────────────────
ROBOT_BASE = np.array([-0.010, 0.505, 0.0])
TABLE_CENTER, TABLE_HALF = (0.0, 0.02), 0.25

# ── robot drives (MuJoCo-era appendix values: hang_wrench / keychain) ───
ARM_KD = 400.0
FINGER_KE, FINGER_KD = 8000.0, 60.0
FINGER_F_MAX = 15.0
FINGER_MASS = 0.12
PAD_MU = 1.4

# ── the grasp (every finger target here; GelSight adds finger_offset) ───
P_OVER = _env("CP_P_OVER", 0.005)     # pad tips this far below the card's top edge
Q_TOUCH = 0.5 * C.CARD_T + C.PAD_FACE_OFF + GS.finger_offset()
Q_START = Q_TOUCH - 0.00005           # joint_q at t = 0: pads on the faces
Q_PINCH0 = Q_TOUCH - 0.0004           # first pinch target (~3 N at 8 kN/m)
Q_GRIP_MIN, Q_GRIP_MAX = Q_TOUCH - 0.0015, Q_TOUCH + 0.0001
Q_RELEASE = Q_TOUCH + 0.0015          # pads 1.5 mm off the faces at the release
GRIP_LO, GRIP_HI = 2.5, 4.0           # N per pad, force servo band
GRIP_STEP = 0.00002                   # m per frame of finger target

# ── start and path (deck frame of the MEASURED front card) ───────────────
START_CARD = np.array([_env("CP_START_X", 0.075), 0.0, _env("CP_START_Z", 0.110)])
GAP_FACE = _env("CP_GAP", 0.0005)     # card face off the front card's face while descending
S_HOVER = 0.060                        # card centre up the deck plane above its seat
S_SLOW = 0.040                         # slow down from here (bottom edge ~41 mm up)
V_FAST, V_SLOW, A_PATH = 0.020, 0.010, 0.040
TIP_MARGIN = _env("CP_TIP_MARGIN", 0.002)   # deck-side tip above the deck's top edges
ALIGN_TOL = 0.00025
ALIGN_FRAMES = 10
F_GATE = 0.5                           # N card<->holder/deck: freeze the descent
T_GATE_MAX = 0.4                       # s frozen -> release anyway
CARRY_LIFT = 0.015                     # m arc over the carry

# ── card servo (integral of the card's position error onto the TCP) ─────
CORR_K, CORR_STEP, CORR_MAX, CORR_DB = 0.08, 0.0002, 0.008, 0.00005

# ── timings ──────────────────────────────────────────────────────────────
T_HOLD, T_CARRY, T_ALIGN_MAX = 0.6, 2.5, 2.0
T_OPEN, T_SETTLE, T_RETREAT, T_STILL = 0.5, 0.6, 1.6, 1.2
RETREAT_U, RETREAT_Z = 0.040, 0.050
T_BUDGET = 25.0

# ── solver ───────────────────────────────────────────────────────────────
SUBSTEPS = _env("CP_SUBSTEPS", 32)
ITERS = _env("CP_ITERS", 30)
IMPRATIO = _env("CP_IMPRATIO", 10.0)

PHASES = ["hold", "carry", "align", "descend", "open", "settle", "retreat", "still"]


def grasp_quat(theta: float) -> np.ndarray:
    """Pads closing along world x tilted by theta about world y, approach
    down (card_pick's grasp_quat)."""
    base = down_grasp_quat(np.pi / 2)
    tilt = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(theta))
    q = wp.mul(tilt, wp.quat(*base))
    return np.array([q[0], q[1], q[2], q[3]])


def T_of(p, R) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def nominal_tcp(c, R):
    """Nominal TCP for a card pose: tips P_OVER below the top edge, pads on
    its faces, approach down its own plane."""
    u = R[:, 2]
    p = c + (0.5 * C.CARD_H + C.TIP_BELOW_TCP - P_OVER) * u
    R_tcp = R @ C.R_of_q(grasp_quat(0.0))
    return p, R_tcp


def slerp_R(Ra, Rb, s):
    qa, qb = C.quat_of_R(Ra), C.quat_of_R(Rb)
    if np.dot(qa, qb) < 0:
        qb = -qb
    q = wp.quat_slerp(wp.quat(*qa.tolist()), wp.quat(*qb.tolist()), float(s))
    return C.R_of_q([q[0], q[1], q[2], q[3]])


def setup_robot(rig):
    rig.add_robot(wp.transform(wp.vec3(*ROBOT_BASE.tolist()), wp.quat_identity()))
    for dof in range(7):
        rig.builder.joint_target_kd[dof] = ARM_KD
    for dof in (7, 8):
        rig.builder.joint_target_ke[dof] = FINGER_KE
        rig.builder.joint_target_kd[dof] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        set_body_mass(rig.builder, bi, FINGER_MASS)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU
            rig.builder.shape_material_mu_torsional[sid] = C.MU_TORSION
            rig.builder.shape_material_mu_rolling[sid] = C.MU_ROLLING


def build_cell(rig, n_deck=C.N_DECK, lean_deg=C.DECK_LEAN_DEG):
    """Table + holder + deck (the robot must already be added)."""
    sc = C.Scene()
    sc.card_local = C.load_card_local()
    rig.table(center_xy=TABLE_CENTER, half=TABLE_HALF)
    t_sid = rig.shapes["table"][0]
    rig.builder.shape_material_mu_torsional[t_sid] = C.MU_TORSION
    rig.builder.shape_material_mu_rolling[t_sid] = C.MU_ROLLING
    C.build_holder(rig, sc)
    C.build_deck(rig, sc, n_deck, lean_deg)
    return sc


class Log:
    def __init__(self):
        self.rows = []

    def add(self, **kw):
        self.rows.append({k: (float(v) if np.isscalar(v) else v) for k, v in kw.items()})

    def arr(self, key):
        return np.array([r[key] for r in self.rows], float)


class PlaceExpert:
    """Carry -> align -> descend -> release -> retreat, closed-loop on the
    MEASURED card pose (the deck frame is the measured front card's)."""

    def __init__(self, rig, sc, trk, T_off, i7, inew, deck_ids, card_start, fps,
                 t0_phase="hold", hold_s=T_HOLD, t0=0.0, handover=None):
        """handover = dict(T_tc, deck_frame, corr, q_grip) to take over a card
        another controller already holds (the platform variant), starting at
        phase `t0_phase` (normally "align") at time t0."""
        self.rig, self.sc, self.trk, self.T_off, self.i7 = rig, sc, trk, T_off, i7
        self.inew, self.deck_ids, self.fps = inew, deck_ids, fps
        self.c0, self.R0 = card_start
        self.dur = {"hold": hold_s, "carry": T_CARRY, "open": T_OPEN, "settle": T_SETTLE,
                    "retreat": T_RETREAT, "still": T_STILL}
        self.k, self.t0 = t0_phase, float(t0)
        self.t_start = {t0_phase: float(t0)}
        self.finished = False
        self.q_grip = Q_PINCH0
        self.corr = np.zeros(3)
        self.T_tc = None                 # measured in-hand transform (TCP -> card)
        self.deck_frame = None           # (O, n, u, R) of the front card, snapshot
        if handover:
            self.T_tc = handover["T_tc"]
            self.deck_frame = handover["deck_frame"]
            self.corr = np.asarray(handover["corr"], float).copy()
            self.q_grip = float(handover["q_grip"])
        self.s = S_HOVER
        self.v = 0.0
        self.n_ok = 0
        self.gate_t = 0.0
        self.p_cmd = None
        self.R_cmd = None
        self.rel = None
        self.events = {}
        self.q_fingers = Q_PINCH0

    # ── measurement ──────────────────────────────────────────────────────
    def meas(self):
        bq = self.rig.s0.body_q.numpy()
        T7 = world_T(bq[self.i7])
        T_tcp = T7 @ self.T_off
        return bq, T_tcp

    def front(self, bq):
        return C.card_frame(bq[self.deck_ids[-1]])

    def deck_top_b(self, bq, O, u):
        b = -1.0
        for i in self.deck_ids:
            tc = C.card_corners(bq[i], C.TOP_CORNERS)
            b = max(b, float(((tc - O) @ u).max()))
        return b

    def seat_ab(self):
        th = self.sc.lean
        a = C.CARD_T + C.FACE_CLEAR
        return a, -a * np.tan(th)

    def card_target(self, s):
        O, n, u, R = self.deck_frame
        a_s, b_s = self.seat_ab()
        return O + (a_s + GAP_FACE) * n + (b_s + s) * u, R

    def tcp_from_card(self, c, R):
        T = T_of(c, R) @ np.linalg.inv(self.T_tc)
        return T[:3, 3], T[:3, :3]

    # ── phase machine ────────────────────────────────────────────────────
    def go(self, k, t):
        self.k, self.t0 = k, t
        self.t_start[k] = t
        print(f"[{NAME}] t={t:5.2f}s -> {k}")

    def frac(self, t):
        return min(1.0, (t - self.t0) / self.dur[self.k]) if self.k in self.dur else 0.0

    def grip_servo(self, F_L, F_R):
        if min(F_L, F_R) < GRIP_LO:
            self.q_grip -= GRIP_STEP
        elif max(F_L, F_R) > GRIP_HI:
            self.q_grip += GRIP_STEP
        self.q_grip = float(np.clip(self.q_grip, Q_GRIP_MIN, Q_GRIP_MAX))

    def learn(self, e, mode):
        if mode == "none":
            return
        if mode == "perp":
            u = self.deck_frame[2]
            e = e - (e @ u) * u
        ee = np.sign(e) * np.maximum(np.abs(e) - CORR_DB, 0.0)
        self.corr = np.clip(self.corr + np.clip(CORR_K * ee, -CORR_STEP, CORR_STEP),
                            -CORR_MAX, CORR_MAX)

    def step(self, t, F_L, F_R, F_card_env, F_robot_deck):
        bq, T_tcp = self.meas()
        c, R, n, u = C.card_frame(bq[self.inew])
        k = self.k
        if k in self.dur and t - self.t0 >= self.dur[k]:
            nxt = PHASES[PHASES.index(k) + 1] if k != "still" else None
            if nxt is None:
                self.finished = True
                raise StopRun
            self.on_exit(k, t, bq, T_tcp, c, R)
            self.go(nxt, t)
            k = self.k
        if k in ("hold", "carry", "align", "descend"):
            self.grip_servo(F_L, F_R)
            fingers = self.q_grip
        if k == "hold":
            if self.T_tc is None:
                self.T_tc = np.linalg.inv(T_tcp) @ T_of(c, R)
            c_t, R_t = self.c0, self.R0
            self.learn(c_t - c, "full")
            self.T_tc = np.linalg.inv(T_tcp) @ T_of(c, R)    # keep the latest grasp
        elif k == "carry":
            s = C.smooth5(self.frac(t))
            ch, Rh = self.card_target(S_HOVER)
            c_t = self.c0 + (ch - self.c0) * s + np.array([0, 0, CARRY_LIFT * np.sin(np.pi * s)])
            R_t = slerp_R(self.R0, Rh, s)
        elif k == "align":
            c_t, R_t = self.card_target(S_HOVER)
            e = c_t - c
            self.learn(e, "full")
            O, nd, ud, _ = self.deck_frame
            err = np.array([e @ nd, e[1]])
            self.n_ok = self.n_ok + 1 if np.abs(err).max() < ALIGN_TOL else 0
            if self.n_ok >= ALIGN_FRAMES or t - self.t0 > T_ALIGN_MAX:
                print(f"[{NAME}] aligned: err a {err[0]*MM:+.3f} y {err[1]*MM:+.3f} mm, "
                      f"corr {np.round(self.corr*MM, 2)} mm")
                self.events["align_err_mm"] = [round(float(v) * MM, 3) for v in err]
                self.go("descend", t)
        elif k == "descend":
            O, nd, ud, Rd = self.deck_frame
            tip = T_tcp[:3, 3] + C.TIP_BELOW_TCP * T_tcp[:3, 2]
            b_tip = float((tip - O) @ ud)
            b_top = self.deck_top_b(bq, O, ud)
            low = float(C.card_corners(bq[self.inew])[:, 2].min())
            gate = F_card_env > F_GATE or F_robot_deck > 0.05
            self.gate_t = self.gate_t + 1.0 / self.fps if gate else 0.0
            done = (b_tip - b_top <= TIP_MARGIN) or (low <= C.FLOOR_Z + 0.0005) \
                or self.gate_t > T_GATE_MAX
            if done:
                self.events.update({
                    "release_t": round(t, 3), "tip_above_deck_mm": round((b_tip - b_top) * MM, 2),
                    "card_low_z_mm": round(low * MM, 2), "s_mm": round(self.s * MM, 2),
                    "gate_s": round(self.gate_t, 3),
                    "why": ("tip" if b_tip - b_top <= TIP_MARGIN else
                            "floor" if low <= C.FLOOR_Z + 0.0005 else "gate")})
                print(f"[{NAME}] release: {self.events}")
                self.go("open", t)
                self.rel = (self.p_cmd.copy(), self.R_cmd.copy(), self.q_grip)
            else:
                if not gate:
                    v_t = V_FAST if self.s > S_SLOW else V_SLOW
                    self.v = min(v_t, self.v + A_PATH / self.fps)
                    self.s -= self.v / self.fps
                else:
                    self.v = 0.0
                c_t, R_t = self.card_target(self.s)
                self.learn(c_t - c, "perp")
        k = self.k
        if k in ("hold", "carry", "align", "descend"):
            p, Rtcp = self.tcp_from_card(c_t, R_t)
            self.p_cmd, self.R_cmd = p + self.corr, Rtcp
            self.q_fingers = fingers
        elif k == "open":
            p_r, R_r, q0 = self.rel
            self.p_cmd, self.R_cmd = p_r, R_r
            self.q_fingers = q0 + (Q_RELEASE - q0) * C.smooth5(self.frac(t))
        elif k == "settle":
            self.q_fingers = Q_RELEASE
        elif k in ("retreat", "still"):
            p_r, R_r, _ = self.rel
            a = self.frac(t) if k == "retreat" else 1.0
            ud = self.deck_frame[2]
            a1 = C.smooth5(min(1.0, 1.6 * a))
            a2 = C.smooth5(max(0.0, 1.6 * a - 0.6))
            self.p_cmd = p_r + RETREAT_U * a1 * ud + RETREAT_Z * a2 * np.array([0, 0, 1.0])
            self.R_cmd = R_r
            self.q_fingers = Q_RELEASE
        self.trk.step(self.p_cmd, C.quat_of_R(self.R_cmd), self.q_fingers)

    def on_exit(self, k, t, bq, T_tcp, c, R):
        if k == "hold":
            self.T_tc = np.linalg.inv(T_tcp) @ T_of(c, R)
            O, Rf, nf, uf = self.front(bq)
            self.deck_frame = (O, nf, uf, Rf)
            print(f"[{NAME}] grasp: card in TCP frame {np.round(self.T_tc[:3, 3]*MM, 2)} mm; "
                  f"front card lean {C.card_lean_deg(bq[self.deck_ids[-1]]):.2f} deg")
        if k == "carry":
            self.n_ok = 0


def main():
    rig = appendix_rig(NAME)
    rig.collide_every_substep = True      # pitfall 5.3: the safe cadence at build
    rig.substeps = SUBSTEPS
    setup_robot(rig)
    sc = build_cell(rig)

    # ── the new card, pinched in the hand at the start ──────────────────
    R0 = np.eye(3)                         # vertical, face normal +x
    c0 = START_CARD.copy()
    C.add_card(rig, sc, sc.new, c0, R0, sc.card_visual)
    C.filter_statics_and_ground(rig)
    p_tcp0, R_tcp0 = nominal_tcp(c0, R0)
    q_tcp0 = C.quat_of_R(R_tcp0)
    q_start = rig.solve_arm_start(p_tcp0, q_tcp0)
    rig.builder.joint_q[7:9] = [Q_START, Q_START]
    rig.builder.joint_target_q[7:9] = [Q_PINCH0, Q_PINCH0]
    lf = rig.ik.body_pose("fr3/fr3_leftfinger")[:3]
    rf = rig.ik.body_pose("fr3/fr3_rightfinger")[:3]
    deck_side = "left" if (lf - rf)[0] < 0 else "right"
    print(f"arm start TCP err {np.linalg.norm(rig.ik.tcp_pose()[:3]-p_tcp0)*MM:.2f} mm; "
          f"deck-side (-x) finger: {deck_side}")
    T_off = tcp_offset_from_link7(rig)

    # 3/4 view from the slot's -y end: the card edges read as a stack and the
    # new card comes down in front of the deck (the multiview orbit adds the
    # face-on view from +x)
    cam_t = np.array([_env("CP_CAM_TX", 0.010), _env("CP_CAM_TY", 0.0), _env("CP_CAM_TZ", 0.060)])
    cam_p = cam_t + np.array([_env("CP_CAM_OX", 0.08), _env("CP_CAM_OY", -0.30),
                              _env("CP_CAM_OZ", 0.15)])
    if os.environ.get("CP_PREVIEW"):
        rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
        preview_cameras(
            rig, os.environ["CP_PREVIEW"],
            [float(v) for v in os.environ.get("CP_PREVIEW_T", "0.5,3,5,7,9,11").split(",")],
            parse_cams(os.environ.get("CP_PREVIEW_CAMS", f"{','.join(map(str, cam_t))}:"
                                      f"{','.join(map(str, cam_p - cam_t))}")),
            os.environ.get("CP_PREVIEW_OUT", str(rig.out_dir / "camera_preview.png")))
        return
    rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t)
    set_camera_fov(rig, cam_p, cam_t, _env("CP_CAM_FOV", 36.0))
    deck_shapes = [l + "_shape" for l in sc.deck]
    rig.probes["padL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", [sc.new + "_shape"])
    rig.probes["padR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", [sc.new + "_shape"])
    rig.probes["card_holder"] = PairProbe(rig.model, sc.new, sc.proxy_labels)
    rig.probes["card_deck"] = PairProbe(rig.model, sc.new, deck_shapes)
    rig.probes["card_table"] = PairProbe(rig.model, sc.new, ["table"])
    for l in sc.deck:
        rig.probes[f"{l}_holder"] = PairProbe(rig.model, l, sc.proxy_labels)
    rig.probes["robot_object"] = RobotContactProbe(rig.model, [sc.new, *sc.deck],
                                                   static_shapes=["holder"])
    rig.probes["robot_deck"] = RobotContactProbe(rig.model, list(sc.deck),
                                                 static_shapes=["holder"])
    # SensorContact before the pipeline's contacts (card_pick's order)
    GS.make_pipeline(rig, rigid_contact_max=40000, max_triangle_pairs=4_000_000,
                     include_static_kinematic_pairs=False)
    rig.solver = make_solver(rig.model, iterations=ITERS, impratio=IMPRATIO)
    rig.collide_every_substep = True

    inew = rig.body_index(sc.new)
    deck_ids = [rig.body_index(l) for l in sc.deck]
    i7 = rig.body_index("fr3/fr3_link7")
    trk = JointTracker(rig, q_start)
    ex = PlaceExpert(rig, sc, trk, T_off, i7, inew, deck_ids, (c0, R0), rig.fps)
    log = Log()

    def last(name, key="force"):
        h = rig.probes[name].history
        return h[-1][key] if h else 0.0

    def control(t):
        F_env = last("card_holder") + last("card_deck") + last("card_table")
        ex.step(t, last("padL"), last("padR"), F_env, last("robot_deck"))

    def extra(state, t):
        bq = state.body_q.numpy()
        c, R, n, u = C.card_frame(bq[inew])
        low = float(C.card_corners(bq[inew])[:, 2].min())
        T_tcp = world_T(bq[i7]) @ T_off
        fz_h = rig.probes["card_holder"].history[-1]["vec"][2] if rig.probes["card_holder"].history else 0.0
        deck_disp = max(float(np.linalg.norm(bq[i, :3] - p0)) for i, p0 in zip(deck_ids, deck_p0))
        row = dict(t=t, phase=PHASES.index(ex.k), cx=c[0], cy=c[1], cz=c[2], low_z=low,
                   lean=C.card_lean_deg(bq[inew]), padL=last("padL"), padR=last("padR"),
                   holder=last("card_holder"), holder_fz=fz_h, deck=last("card_deck"),
                   table=last("card_table"), robot_obj=last("robot_object"),
                   robot_deck=last("robot_deck"), tcp_x=T_tcp[0, 3], tcp_y=T_tcp[1, 3],
                   tcp_z=T_tcp[2, 3], q_grip=ex.q_fingers, deck_disp=deck_disp, s=ex.s)
        log.add(**row)
        if int(round(t * rig.fps)) % 15 == 0:
            print(f"  [{t:5.2f} {ex.k:8s}] pads {row['padL']:5.2f}/{row['padR']:5.2f} N  card "
                  f"{np.round(c*MM, 2)} low {low*MM:6.2f} lean {row['lean']:5.2f}  env "
                  f"h{row['holder']:.3f} d{row['deck']:.3f} t{row['table']:.3f}  robot "
                  f"{row['robot_obj']:.3f}/{row['robot_deck']:.3f}  deck_disp "
                  f"{deck_disp*MM:.3f} mm  s {ex.s*MM:5.1f}  corr {np.round(ex.corr*MM, 2)}")
        return {"padL_N": round(row["padL"], 3), "padR_N": round(row["padR"], 3),
                "card_holder_N": round(row["holder"], 4), "card_deck_N": round(row["deck"], 4),
                "card_table_N": round(row["table"], 4),
                "robot_object_contact_N": round(row["robot_obj"], 4),
                "robot_deck_N": round(row["robot_deck"], 4),
                "card_low_z_mm": round(low * MM, 3), "card_lean_deg": round(row["lean"], 3),
                "deck_disp_mm": round(deck_disp * MM, 3)}

    deck_p0 = [rig.s0.body_q.numpy()[i, :3].copy() for i in deck_ids]
    stop_t = _env("CP_STOP_T", 0.0)
    try:
        rec = rig.run(stop_t if stop_t > 0 else T_BUDGET, control, extra)
    except StopRun:
        rec = rig.rec
    print(f"[{NAME}] rollout ended at t={len(rec.times) / rig.fps:.2f}s; events {ex.events}")
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "card_holder_N", "card_deck_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    (rig.out_dir / "expert_log.json").write_text(json.dumps(
        {"phase_start_s": ex.t_start, "events": ex.events, "rows": log.rows[::3]}, default=float))
    if stop_t > 0 or not ex.finished:
        print("stopped early: QA skipped")
        return
    verdict(rig, sc, ex, log, paths)


def verdict(rig, sc, ex, log, paths, extra_criteria=None, task_title="Place card into holder",
            changes=None, source="new task (no suite counterpart); reverse of "
                                 "force_bench.tasks.card_pick, finalized-kit holder and cards",
            transport_from=None, extra_pen=None, envelope=None):
    qa = rig.qa(paths)
    d = np.load(paths["states"], allow_pickle=True)
    bq, t = d["body_q"], d["t"]
    labels = list(d["body_labels"])
    fps = rig.fps
    inew = labels.index(sc.new)
    deck_ids = [labels.index(l) for l in sc.deck]
    tph = ex.t_start
    t_end = float(t[-1])
    fin = t >= t_end - T_STILL + 0.2

    def speed_mm_s(i, m):
        cc = np.array([C.card_corners(bq[f, i]) for f in np.flatnonzero(m)])
        if len(cc) < 2:
            return 0.0
        return float(np.linalg.norm(np.diff(cc, axis=0), axis=-1).max() * fps * MM)

    crit = dict(extra_criteria or {})
    i7 = labels.index("fr3/fr3_link7")
    t_rel = ex.events.get("release_t")

    def in_hand(f):
        """Card origin in the link7 frame (the pinch's own frame)."""
        return (np.linalg.inv(world_T(bq[f, i7])) @ world_T(bq[f, inew]))[:3, 3]

    # ── held in hand at the start, and through the transport ─────────────
    th0 = tph.get("hold", 0.0)
    th1 = tph.get("carry", th0 + T_HOLD) if transport_from is None else float(transport_from)
    pL, pR = log.arr("padL"), log.arr("padR")
    lt = log.arr("t")
    lm = (lt >= th0 + 0.1) & (lt < th1)
    hf = np.flatnonzero((t >= th0) & (t < th1))
    ih = np.array([in_hand(f) for f in hf])
    sink = float(np.linalg.norm(ih - in_hand(0), axis=1).max()) if len(ih) else 9.9
    both = float(((pL[lm] > 0.5) & (pR[lm] > 0.5)).mean()) if lm.any() else 0.0
    env = float((log.arr("holder") + log.arr("deck") + log.arr("table"))[lm].max()) if lm.any() else 0.0
    if "hold" in tph:
        crit["held_in_hand_at_start"] = {
            "pass": bool(sink < 0.0005 and both >= 0.95 and env < 0.01),
            "card_slip_in_pinch_during_hold_mm": round(sink * MM, 3),
            "both_pads_loaded_fraction": round(both, 3),
            "pad_mean_N": [round(float(pL[lm].mean()), 2), round(float(pR[lm].mean()), 2)],
            "other_contact_max_N": round(env, 4),
            "window_s": [round(th0, 2), round(th1, 2)],
            "note": "the card starts pinched (pads on its faces at t = 0, no pre-roll); over "
                    "the hold it moves < 0.5 mm in the hand's own frame, both pads carry it "
                    "and nothing else touches it"}
    if t_rel is not None:
        tf = np.flatnonzero((t >= th1) & (t < t_rel))
        tr = np.array([in_hand(f) for f in tf])
        slip_tr = float(np.linalg.norm(tr - tr[0], axis=1).max()) if len(tr) else 9.9
        mt = (lt >= th1) & (lt < t_rel)
        both_tr = float(((pL[mt] > 0.5) & (pR[mt] > 0.5)).mean()) if mt.any() else 0.0
        crit["held_through_transport"] = {
            "pass": bool(slip_tr < 0.001 and both_tr >= 0.98),
            "card_slip_in_pinch_mm": round(slip_tr * MM, 3),
            "both_pads_loaded_fraction": round(both_tr, 3),
            "pad_min_N": [round(float(pL[mt].min()), 2), round(float(pR[mt].min()), 2)],
            "window_s": [round(th1, 2), round(float(t_rel), 2)],
            "note": "carry, align and descent: the card is carried by the two pads only "
                    "(no joint, weld or kinematic attachment exists in this scene) and moves "
                    "< 1 mm in the hand's frame until the release"}
    # ── released inside the slot ────────────────────────────────────────
    if t_rel is not None:
        fr = int(np.argmin(np.abs(t - t_rel)))
        sl = C.in_slot(bq[fr, inew])
        low = float(C.card_corners(bq[fr, inew])[:, 2].min())
        top = float(C.card_corners(bq[fr, inew], C.BOTTOM_CORNERS)[:, 2].max())
        lows = np.array([C.card_corners(bq[f, inew])[:, 2].min() for f in range(fr, len(t))])
        f_land = fr + int(np.argmax(lows <= C.FLOOR_Z + 0.0002)) if (lows <= C.FLOOR_Z + 0.0002).any() else None
        v_land = (float(np.linalg.norm(bq[f_land, inew, :3] - bq[f_land - 1, inew, :3]) * fps)
                  if f_land else None)
        crit["released_inside_slot"] = {
            "pass": bool(sl["bottom_corners_inside_walls_x"] and sl["bottom_corners_inside_end_walls_y"]
                         and top < C.WALL_TOP_Z),
            "release_t_s": round(float(t_rel), 2), **sl,
            "lower_edge_z_mm": [round(low * MM, 2), round(top * MM, 2)],
            "wall_top_z_mm": C.WALL_TOP_Z * MM,
            "fall_to_floor_mm": round((low - C.FLOOR_Z) * MM, 2),
            "landed_on_floor_t_s": round(float(t[f_land]), 3) if f_land else None,
            "landing_speed_m_s": round(v_land, 3) if v_land is not None else None,
            "release_trigger": ex.events.get("why"),
            "deck_side_tip_above_deck_mm": ex.events.get("tip_above_deck_mm"),
            "note": "the pads open with the card's whole lower edge already inside the slot "
                    "(between the side walls and end walls, below the side-wall tops); a "
                    "face pinch keeps one fingertip between the card and the deck, so the "
                    "last few mm are the card sliding down the front card's face"}
    else:
        crit["released_inside_slot"] = {"pass": False, "note": "never released"}
    # ── the placed card ─────────────────────────────────────────────────
    sl = C.in_slot(bq[-1, inew])
    lean_new = C.card_lean_deg(bq[-1, inew])
    lean_front = C.card_lean_deg(bq[-1, deck_ids[-1]])
    cf, Rf, nf, uf = C.card_frame(bq[-1, deck_ids[-1]])
    cn = bq[-1, inew, :3]
    face_gap = float((cn - cf) @ nf) - C.CARD_T
    v_new = speed_mm_s(inew, fin)
    deck_contact = float(log.arr("deck")[log.arr("t") >= t_end - T_STILL + 0.2].mean())
    on_floor = abs(sl["lowest_point_above_floor_mm"]) <= 0.5
    in_walls = sl["bottom_corners_inside_walls_x"] and sl["bottom_corners_inside_end_walls_y"]
    stacked = abs(lean_new - lean_front) < 3.0 and abs(face_gap) < 0.0006
    crit["card_in_holder"] = {
        "pass": bool(in_walls and on_floor and stacked and v_new < 2.0),
        **sl, "on_floor": bool(on_floor),
        "lean_deg": round(lean_new, 2), "front_card_lean_deg": round(lean_front, 2),
        "face_gap_to_front_card_mm": round(face_gap * MM, 3),
        "card_on_deck_contact_mean_N": round(deck_contact, 4),
        "weight_N": round(9.81 * C.card_mass(), 4),
        "final_speed_mm_s": round(v_new, 3),
        "final_centre_mm": [round(float(v) * MM, 2) for v in cn],
        "note": "final window: all four bottom corners inside the slot's side walls and end "
                "walls, the lowest point on the floor (+-0.5 mm), leaning in the stack (lean "
                "within 3 deg of the front card's, face gap < 0.6 mm), at rest (< 2 mm/s)"}
    # ── the existing cards ──────────────────────────────────────────────
    others, ok_o = {}, True
    for l, i in zip(sc.deck, deck_ids):
        disp = float(np.linalg.norm(bq[-1, i, :3] - bq[0, i, :3]))
        dmax = float(np.linalg.norm(bq[:, i, :3] - bq[0, i, :3], axis=1).max())
        n0 = C.card_frame(bq[0, i])[2]
        n1 = C.card_frame(bq[-1, i])[2]
        tilt = float(np.degrees(np.arccos(np.clip(abs(n0 @ n1), -1, 1))))
        s_ = C.in_slot(bq[-1, i])
        v_ = speed_mm_s(i, fin)
        ok = (disp < 0.003 and tilt < 3.0 and s_["bottom_corners_inside_walls_x"]
              and s_["bottom_corners_inside_end_walls_y"]
              and abs(s_["lowest_point_above_floor_mm"]) <= 0.5 and v_ < 2.0)
        others[l] = {"final_disp_mm": round(disp * MM, 3), "max_disp_mm": round(dmax * MM, 3),
                     "tilt_change_deg": round(tilt, 3), "lean_deg_final": round(C.card_lean_deg(bq[-1, i]), 2),
                     "final_speed_mm_s": round(v_, 3), **s_, "pass": bool(ok)}
        ok_o &= ok
    crit["existing_cards_remain"] = {
        "pass": bool(ok_o), "cards": others, "max_disp_mm": 3.0, "max_tilt_deg": 3.0,
        "note": "every pre-loaded card ends within 3 mm / 3 deg of its start, inside the slot, "
                "on its floor and at rest"}
    # ── released and clear, robot stopped ───────────────────────────────
    if t_rel is not None:
        i_rel = int(np.searchsorted(lt, t_rel))
        tcp = np.column_stack([log.arr("tcp_x"), log.arr("tcp_y"), log.arr("tcp_z")])
        moved = float(np.linalg.norm(tcp[-1] - tcp[i_rel]))
    else:
        moved = 0.0
    t_from = tph.get("still", t_end - T_STILL)
    check_released_and_clear(qa, lt, log.arr("robot_obj"), t_from, extra={
        "tcp_moved_from_release_mm": round(moved * MM, 1), "min_move_mm": 50.0})
    rel = qa.report["checks"]["released_and_clear"]
    rel["pass"] = bool(rel["pass"] and moved >= 0.050)
    crit["released_and_clear"] = rel
    arm_ix = [i for i, l in enumerate(labels) if str(l).startswith("fr3/")]
    m = t >= t_end - T_STILL + 0.2
    v_arm = float(np.linalg.norm(np.diff(bq[m][:, arm_ix, :3], axis=0), axis=-1).max()) * fps
    crit["robot_stopped"] = {"pass": bool(v_arm < 0.005),
                             "max_link_speed_final_window_mm_s": round(v_arm * MM, 3),
                             "threshold_mm_s": 5.0}
    # ── physics integrity: penetration + envelope ───────────────────────
    cards = list(sc.deck) + [sc.new]
    n_f = len(t)
    frames = set(np.linspace(0, n_f - 1, 36).astype(int).tolist())
    if t_rel is not None:
        f0 = int(np.searchsorted(t, t_rel))
        frames |= set(range(f0, min(n_f, f0 + int(1.2 * fps)), 3))
    frames = sorted(frames)
    pairs = C.audit_cards(bq, t, labels, cards, sc.holder_tm, sc.holder_raw, frames)
    if extra_pen is not None:
        pairs.update(extra_pen(bq, t, labels, frames))
    for k_, v_ in pairs.items():
        print(f"  audit {k_}: {v_['max_depth_mm']:+.3f} mm at t={v_['at_t']}")
    ok_pen = all(v["pass"] for v in pairs.values())
    qa.report["checks"]["penetration"] = {
        "pass": bool(ok_pen), "tolerance_mm": 0.8, "pairs": pairs, "frames_checked": len(frames),
        "note": "ground truth: card box surfaces (1.5 mm grid) vs the CAD holder mesh with "
                "ray-verified inside sign, CAD holder vertices vs each card box, every card "
                "pair exactly (box vs box points), both directions"}
    qa.report["pass"] &= bool(ok_pen)
    lo_e, hi_e = envelope or ([-0.20, -0.20, -0.002], [0.25, 0.25, 0.30])
    qa.check_envelope(cards, lo_e, hi_e)
    qa.report["checks"]["expert_tracking"] = {
        "pass": True, "info_only": True, "events": ex.events,
        "phase_start_s": {k: round(v, 2) for k, v in tph.items()},
        "peak_joint_cmd_step_rad": round(ex.trk.dq_peak, 4),
        "card_servo_corr_final_mm": np.round(ex.corr * MM, 2).tolist()}
    for k_, c_ in crit.items():
        qa.report["checks"].setdefault(k_, c_)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    return save_appendix_verdict(
        rig, task=task_title, source=source,
        spec="The robot movement stops after placing the card in the card holder, with "
             "existing cards remaining in the card holder. The robot should not remain in "
             "contact with the task objects.",
        criteria=crit, physics_qa=report,
        changes=changes or [
            "new task: finalized-kit holder (assets/bodies_kit/card_holder_kit) with 3 kit "
            "cards leaning at 12 deg on its far bracket posts; a 4th kit card starts pinched "
            "in the FR3's hand, vertical, 7.5 cm to the free side",
            "the card is carried, turned to the deck's measured lean, aligned in the slot "
            "(0.75 mm of end play) and lowered down the deck plane in front of the front card; "
            "the pads open with its lower edge inside the slot ~9 mm above the floor (the "
            "deck-side fingertip has to stay above the deck), and it slides onto the floor",
            "cards are analytic boxes on analytic slot supports matching the CAD faces "
            "(tasks/card_pick's model); the CAD holder collides with the robot and is the "
            "penetration ground truth",
        ])


if __name__ == "__main__":
    wp.init()
    main()
