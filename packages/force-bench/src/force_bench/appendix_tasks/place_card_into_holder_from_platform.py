"""Appendix task "Place card into holder" (Table I, #24), CHAINED after
"Pick up object on platform" (#6): the card does not start in the hand, it
lies flat on a raised platform beside the holder; the FR3 slides it to the
platform's edge, pinches the overhang, lifts it off, turns it upright and
places it into the holder in front of the deck (user request: start by
picking the object up).

    Success (appendix #24): "The robot movement stops after placing the
    card in the card holder, with existing cards remaining in the card
    holder. The robot should not remain in contact with the task objects."
    (#6: "The target object is stably grasped by the robot and lifted off
    the platform.")

SCENE: the cell, holder and 3-card deck of ``place_card_into_holder``
(``_card_place.py``), plus a platform to the holder's free (+x) side: the
analytic 254 x 254 x 3.18 mm board -- the acrylic board of the finalized
kit's platform.step (STEP mesh cache: 254 mm square, 3.18 mm thick, on a
black X-frame, top at 53.2 mm) -- on four 20 mm corner legs.  THE BOARD IS
RAISED TO 120 mm.  A flat card can only
be pinched on its two faces, so the pads must close VERTICALLY with the
approach horizontal; the FR3 hand housing then spans -104 .. +101 mm along
the closing axis (measured off the URDF collision shapes), i.e. it hangs
104 mm below the card, outboard of the board edge.  At 53 mm it would be
50 mm inside the table; at 120 mm it clears it by 16 mm.  (The suite's
slide_off_edge / pick_up_object_on_platform use a 220 mm bench for the
same reason.)  The pick edge faces the robot (+y), and the card lies with
its 85 mm side along that edge, 40 mm in from it.

WHY THIS CELL (offline IK, warm-started along the whole path the way the
joint tracker follows it): only the side pinch with the pads closing along
+z and the insertion grasp with the jaw along -n give one continuous joint
path from the blade press through the side pinch to the holder (worst
joint margin 0.28 rad, at joint 4 in the swing; <= 0.05 rad per step); the
other three sign pairs end on joint 6's stop or lose the IK.  So the card
lies face-down in its own frame (it is a symmetric slab).

EXPERT (fingertip contact only; privileged state for closed loops):
  1. settle     robot still over the card, fingers closed into one blade;
  2. down       blade to 2 mm above the card's top face;
  3. push       blade pressed on the card's trailing half with a FORCE
                SERVO (2-4 N summed pad load, height walks 0.9 mm/s) and
                dragged toward the edge at 17 mm/s while loaded
                (slide_off_edge's push: pad mu 1.4 vs card/board 0.25 ...
                0.15 -- skin stickier than the desk); stops on the MEASURED
                overhang (21.5 mm: the card's centre of mass 6 mm inboard
                of the edge, slide_off_edge's CoM rule);
  4. unload     the press is relieved until the card is still;
  5. up / out   the blade breaks the press straight up, rises to 60 mm over
                the board and moves outboard of the edge;
  6. swing      only there (the hand a clear ball of air) the wrist turns to
                the side grasp and the pads open 10 mm per side;
  7. descend, reach   down to the card's mid-plane outboard of the edge,
                in until the fingertips are 6 mm onto the overhang;
  8. close, lift      pads to the pinch (force servo 2.5-4 N per pad), the
                card is lifted 60 mm off the board;
  9. turn       up 50 mm more, then over the holder while the hand turns
                the card upright to the deck's measured lean, down to the
                hover;
  10. place     ``place_card_into_holder``'s expert from its align phase:
                align in the slot, lower down the deck plane, release with
                the deck-side tip 2 mm over the deck, retreat, stand still.

VERDICT (appendix_qa.json): picked_up_from_platform (rests on the board at
the start, slid to an overhang, lifted >= 30 mm, never touches the board
again, both pads carry it) + every place_card_into_holder criterion
(held_through_transport, released_inside_slot, card_in_holder,
existing_cards_remain, released_and_clear incl. the platform,
robot_stopped) + the physics integrity checks (penetration audit adds the
card vs the board).

RESULTS (2026-10-16, publication render, outputs/appendix_tasks/
place_card_into_holder_from_platform/; PASS): the card rests on the board
carrying its weight (0.0682 N); the force-servoed push (2-3 N, the board
penetrated <= 0.055 mm) slides it 61 mm to a 21.2 mm overhang; the arm sag
learned at the standoff is (0.49, -0.57, 1.03) mm, so the pads close about
the card's mid-plane (card 0.1 mm off the jaw axis; 2.9 / 2.2 N); lifted
59.7 mm clear, never touching the board again (0 N), no robot-board
contact at any time; 0.38 mm of slip in the pinch through the turn (the
grip carries the card flat, then upright); aligned within 0.13 mm at the
hover; released at t = 23.67 s, deck-side tip 1.93 mm over the deck, lower
edge 10.4 mm up (inside the slot), slides 8.4 mm and lands at 0.08 m/s;
final: on the floor, inside the walls, lean 12.69 deg against the front
card's 12.21 deg, face gap 0.30 mm, at rest; pre-loaded cards moved
<= 0.03 mm / 0.16 deg; hand 89.5 mm away, 0 N.  A start 5 mm off in x and
yawed 2 deg passes the same way (pinch 4.9 mm off-centre, absorbed by the
measured grasp: slip 0.40 mm, final lean 12.91 deg, gap 0.35 mm).

Run:  python -m force_bench.appendix_tasks.place_card_into_holder_from_platform
      (CP_STOP_T=<s> stops early and skips the QA; CP_PREVIEW=<states.npz>
       renders a camera sheet; CPP_CARD_DX / CPP_CARD_YAW_DEG perturb the
       start; TB_OUT_ROOT / TB_RENDER_FAST as usual)
"""

from __future__ import annotations

import json
import os

import numpy as np
import warp as wp

import force_bench.tactile.task as GS  # noqa: F401  (before the rig: GelSight readiness)
from force_bench.rig import make_solver
from force_bench.geometry import world_T
from force_bench.robot.franka import down_grasp_quat
from force_bench.appendix_tasks.common import appendix_rig, RobotContactProbe, StopRun
from force_bench.appendix_tasks import _card_place as C
from force_bench.appendix_tasks import place_card_into_holder as P
from force_bench.appendix_tasks._hang_plate import (
    PairProbe, JointTracker, tcp_offset_from_link7, preview_cameras, parse_cams)

NAME = "place_card_into_holder_from_platform"
MM = 1000.0
_env = P._env

# ── the platform (kit board footprint, raised; see the header) ───────────
BOARD_HALF = 0.127
BOARD_T = 0.00318
HP = _env("CPP_HP", 0.120)              # board top above the table
LEG = 0.020
BOARD_MU = 0.15                         # acrylic
EDGE_Y = 0.020                          # the pick edge (toward the robot)
BOARD_C = np.array([0.220, EDGE_Y - BOARD_HALF])
CARD_X0 = 0.220
CARD_INSET = 0.040                      # card's +y edge this far in from the edge
REST_CLEAR = 0.00005
# card lying flat, face-down in its own frame: local x -> -z, y -> -x, z -> +y
R_FLAT = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])

# ── the push (slide_off_edge's card values, MuJoCo era) ─────────────────
F_PRESS_LO, F_PRESS_HI = 2.0, 4.0       # N summed over both pads
PRESS_WIN = 6
PRESS_RATE = 0.000015                   # m/frame
PRESS_MAX, PRESS_MIN = 0.006, -0.004
PRESS_START = -0.002                    # timed descent stops 2 mm high
SLIDE_RATE = 0.00028                    # m/frame (17 mm/s)
OH_STOP = 0.0215                        # overhang: CoM 6 mm inboard of the edge
BLADE_HALF = 0.0152                     # closed pads, half-length along the push
PUSH_BACK = 0.5 * C.CARD_H - BLADE_HALF - 0.0018
COAST_S, COAST_MAX, COAST_WIN = 0.06, 0.002, 10
PUSH_STILL, F_PRESS_OFF = 0.0008, 0.6
UP_BREAK, UP_SPLIT, UP_BACK = 0.020, 0.25, 0.010
SAFE_DZ = 0.060                         # transit height above the board top

# ── the fingers (every finger target in one place; GS adds the offset) ──
BLADE_Q = 0.0 + GS.finger_offset()     # push: pads closed into one blade
FINGER_OPEN = 0.010 + GS.finger_offset()
GRIP = _env("CPP_GRIP", 0.006)          # fingertips this far onto the overhang
STANDOFF = 0.045                        # tips this far outboard of the card at the standoff
LIFT = 0.060
LIFT2 = 0.070
HOVER_UP = 0.050                        # turn ends this far above the hover

# ── timings ──────────────────────────────────────────────────────────────
T_SETTLE, T_DOWN, T_PUSH_MAX, T_UNLOAD = 0.5, 0.8, 7.0, 1.4
T_UP, T_OUT, T_SWING, T_DESC, T_PRE, T_REACH, T_CLOSE, T_LIFT = 1.1, 1.0, 1.6, 1.2, 0.5, 1.2, 0.7, 1.2
T_LIFT2, T_TURN, T_HOVER = 0.8, 2.6, 1.0
T_BUDGET = 45.0
PICK = ["settle", "down", "push", "unload", "up", "out", "swing", "descend", "pre", "reach",
        "close", "lift", "lift2", "turn", "hover"]
DUR = {"settle": T_SETTLE, "down": T_DOWN, "unload": T_UNLOAD, "up": T_UP, "out": T_OUT,
       "swing": T_SWING, "descend": T_DESC, "pre": T_PRE, "reach": T_REACH, "close": T_CLOSE,
       "lift": T_LIFT, "lift2": T_LIFT2, "turn": T_TURN, "hover": T_HOVER}


def side_quat() -> np.ndarray:
    """Approach -y (toward the board, away from the robot), pads closing
    along world +z (the sign the IK path study selected)."""
    z = np.array([0.0, -1.0, 0.0])
    y = np.array([0.0, 0.0, 1.0])
    return C.quat_of_R(np.column_stack([np.cross(y, z), y, z]))


def build_platform(rig):
    """Board + four corner legs, analytic boxes (the board is a box in the
    CAD too); the legs only carry it and are visual + robot colliders."""
    x0, y0 = BOARD_C
    s = rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(float(x0), float(y0), float(HP - 0.5 * BOARD_T)),
                               wp.quat_identity()),
        hx=BOARD_HALF, hy=BOARD_HALF, hz=0.5 * BOARD_T,
        cfg=C._cfg(rig, BOARD_MU, gap=0.0005), label="platform_board")
    rig.shapes["platform_board"] = [s]
    legs = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            cx = x0 + sx * (BOARD_HALF - 0.5 * LEG)
            cy = y0 + sy * (BOARD_HALF - 0.5 * LEG)
            legs.append(rig.builder.add_shape_box(
                -1, xform=wp.transform(wp.vec3(float(cx), float(cy), float(0.5 * (HP - BOARD_T))),
                                       wp.quat_identity()),
                hx=0.5 * LEG, hy=0.5 * LEG, hz=0.5 * (HP - BOARD_T),
                cfg=C._cfg(rig, BOARD_MU, gap=0.0005), label="platform_leg"))
    rig.shapes["platform_leg"] = legs
    return s


class PickController:
    """Phases settle .. hover (slide to the edge, side pinch, lift, turn),
    then hands the card to place_card_into_holder.PlaceExpert at "align"."""

    def __init__(self, rig, sc, trk, T_off, i7, inew, deck_ids, fps, probes):
        self.rig, self.sc, self.trk, self.T_off, self.i7 = rig, sc, trk, T_off, i7
        self.inew, self.deck_ids, self.fps, self.pr = inew, deck_ids, fps, probes
        self.k, self.t0 = "settle", 0.0
        self.t_start = {"settle": 0.0}
        self.place = None
        self.q_blade = down_grasp_quat(0.0)
        self.q_side = side_quat()
        bq = rig.s0.body_q.numpy()
        c, R, _, _ = C.card_frame(bq[inew])
        self.y_cmd = c[1] - PUSH_BACK
        self.touch_z = HP + C.CARD_T + C.TIP_BELOW_TCP
        self.park = np.array([CARD_X0, self.y_cmd, self.touch_z + 0.030])
        self.press = PRESS_START
        self.oh_hist = []
        self.q_f = 0.0
        self.q_grip = P.Q_PINCH0
        self.p_cmd, self.q_cmd = self.park.copy(), self.q_blade.copy()
        self.ev = {}
        self.T_tc = None
        self.deck_frame = None
        self._oh_prev = None
        # arm sag at the side pinch: learned on a STATIC command at the
        # standoff ("pre", README pitfall 8.1/46), applied from there on so the
        # pads close about the card's real mid-plane
        self.droop = np.zeros(3)
        self._p_prev = None

    # ── helpers ──────────────────────────────────────────────────────────
    def go(self, k, t):
        self.k, self.t0 = k, t
        self.t_start[k] = t
        print(f"[{NAME}] t={t:5.2f}s -> {k}")

    def frac(self, t):
        return min(1.0, (t - self.t0) / DUR[self.k]) if self.k in DUR else 0.0

    def pads_sum(self):
        L = self.pr["padL"].history[-PRESS_WIN:]
        R = self.pr["padR"].history[-PRESS_WIN:]
        if not L:
            return 0.0
        return float(np.mean([a["force"] for a in L]) + np.mean([b["force"] for b in R]))

    def card_edge_y(self, bq):
        return float(C.card_corners(bq[self.inew])[:, 1].max())

    def tcp_meas(self, bq):
        return world_T(bq[self.i7]) @ self.T_off

    def grip_servo(self):
        F_L = self.pr["padL"].history[-1]["force"] if self.pr["padL"].history else 0.0
        F_R = self.pr["padR"].history[-1]["force"] if self.pr["padR"].history else 0.0
        if min(F_L, F_R) < P.GRIP_LO:
            self.q_grip -= P.GRIP_STEP
        elif max(F_L, F_R) > P.GRIP_HI:
            self.q_grip += P.GRIP_STEP
        self.q_grip = float(np.clip(self.q_grip, P.Q_GRIP_MIN, P.Q_GRIP_MAX))
        return self.q_grip

    # ── the per-frame command ────────────────────────────────────────────
    def step(self, t, F_env, F_robot_deck):
        if self.place is not None:
            self.place.step(t, *self._pads(), F_env, F_robot_deck)
            return
        bq = self.rig.s0.body_q.numpy()
        k = self.k
        u = self.frac(t)
        if k in DUR and u >= 1.0:
            self.on_exit(k, t, bq)
            if self.place is not None:
                self.place.step(t, *self._pads(), F_env, F_robot_deck)
                return
            k, u = self.k, 0.0
        oh = self.card_edge_y(bq) - EDGE_Y
        if k == "settle":
            p, q, f = self.park, self.q_blade, BLADE_Q
        elif k == "down":
            z = self.park[2] + (self.touch_z - PRESS_START - self.park[2]) * C.smooth5(u)
            p, q, f = np.array([CARD_X0, self.y_cmd, z]), self.q_blade, BLADE_Q
        elif k == "push":
            F = self.pads_sum()
            if F < F_PRESS_LO:
                self.press = min(PRESS_MAX, self.press + PRESS_RATE)
            elif F > F_PRESS_HI:
                self.press = max(PRESS_MIN, self.press - PRESS_RATE)
            self.oh_hist = (self.oh_hist + [oh])[-COAST_WIN:]
            h = self.oh_hist
            v = (h[-1] - h[0]) * self.fps / max(1, len(h) - 1) if len(h) > 1 else 0.0
            coast = float(np.clip(v * COAST_S, 0.0, COAST_MAX))
            if oh + coast >= OH_STOP or t - self.t0 > T_PUSH_MAX:
                self.ev.update(push_end_t=round(t, 3), overhang_mm=round(oh * MM, 2),
                               press_mm=round(self.press * MM, 3), pad_load_N=round(F, 2))
                print(f"[{NAME}] push done: overhang {oh*MM:.1f} mm, press {self.press*MM:+.2f} mm "
                      f"at {F:.2f} N")
                self.go("unload", t)
            elif F >= F_PRESS_LO:
                self.y_cmd += SLIDE_RATE
            p, q, f = np.array([CARD_X0, self.y_cmd, self.touch_z - self.press]), self.q_blade, BLADE_Q
        elif k == "unload":
            vy = abs(oh - self._oh_prev) * self.fps if self._oh_prev is not None else 1.0
            self._oh_prev = oh
            self.press = max(PRESS_MIN, self.press - 4 * PRESS_RATE)
            p, q, f = np.array([CARD_X0, self.y_cmd, self.touch_z - self.press]), self.q_blade, BLADE_Q
            if t - self.t0 > 0.3 and vy < PUSH_STILL and self.pads_sum() < F_PRESS_OFF:
                self.on_exit("unload", t, bq)
                k = self.k
        u = self.frac(t)
        if k == "up":
            z0 = self.touch_z - self.press
            a = min(1.0, u / UP_SPLIT)
            b = C.smooth5(max(0.0, (u - UP_SPLIT) / (1.0 - UP_SPLIT)))
            z = z0 + UP_BREAK * a + (HP + SAFE_DZ - z0 - UP_BREAK) * b
            p, q, f = np.array([CARD_X0, self.y_cmd - UP_BACK * b, z]), self.q_blade, BLADE_Q
        elif k == "out":
            y0 = self.y_cmd - UP_BACK
            p = np.array([CARD_X0, y0 + (self.y_stand - y0) * C.smooth5(u), HP + SAFE_DZ])
            q, f = self.q_blade, BLADE_Q
        elif k == "swing":
            p = np.array([CARD_X0, self.y_stand, HP + SAFE_DZ])
            q = C.quat_of_R(P.slerp_R(C.R_of_q(self.q_blade), C.R_of_q(self.q_side), C.smooth5(u)))
            f = BLADE_Q + (FINGER_OPEN - BLADE_Q) * C.smooth5(min(1.0, 2.0 * u))
        elif k == "descend":
            z = HP + SAFE_DZ + (self.z_mid - HP - SAFE_DZ) * C.smooth5(u)
            p, q, f = np.array([CARD_X0, self.y_stand, z]), self.q_side, FINGER_OPEN
        elif k == "pre":
            p, q, f = np.array([CARD_X0, self.y_stand, self.z_mid]), self.q_side, FINGER_OPEN
        elif k == "reach":
            y = self.y_stand + (self.y_pinch - self.y_stand) * C.smooth5(u)
            p, q, f = np.array([CARD_X0, y, self.z_mid]), self.q_side, FINGER_OPEN
        elif k == "close":
            p, q = np.array([CARD_X0, self.y_pinch, self.z_mid]), self.q_side
            f = FINGER_OPEN + (P.Q_PINCH0 - FINGER_OPEN) * C.smooth5(u)
            if u >= 0.999:
                f = self.grip_servo()
        elif k == "lift":
            p = np.array([CARD_X0, self.y_pinch, self.z_mid + LIFT * C.smooth5(u)])
            q, f = self.q_side, self.grip_servo()
        elif k == "lift2":
            p = np.array([CARD_X0, self.y_pinch, self.z_mid + LIFT + LIFT2 * C.smooth5(u)])
            q, f = self.q_side, self.grip_servo()
        elif k == "turn":
            a = C.smooth5(u)
            p0 = np.array([CARD_X0, self.y_pinch, self.z_mid + LIFT + LIFT2])
            p = p0 + (self.p_hover_up - p0) * a
            q = C.quat_of_R(P.slerp_R(C.R_of_q(self.q_side), self.R_hover, a))
            f = self.grip_servo()
        elif k == "hover":
            p = self.p_hover_up + (self.p_hover - self.p_hover_up) * C.smooth5(u)
            q, f = C.quat_of_R(self.R_hover), self.grip_servo()
        p = np.asarray(p, float)
        if k in ("pre", "reach"):
            # learn per axis only while that axis of the command is still
            # (and only where the correction is applied, or it winds up)
            tcp = self.tcp_meas(bq)[:3, 3]
            if self._p_prev is not None and self.p_cmd is not None:
                still = np.abs(p - self._p_prev) < 5e-5
                e = self.p_cmd - tcp
                step = np.clip(0.3 * e, -0.0004, 0.0004)
                self.droop = np.clip(self.droop + np.where(still, step, 0.0), -0.008, 0.008)
        self._p_prev = p.copy()
        p_out = p + (self.droop if k in ("pre", "reach", "close", "lift", "lift2", "turn", "hover")
                     else 0.0)
        self.p_cmd, self.q_cmd, self.q_f = p, np.asarray(q, float), float(f)
        self.trk.step(p_out, self.q_cmd, self.q_f)

    def _pads(self):
        F_L = self.pr["padL"].history[-1]["force"] if self.pr["padL"].history else 0.0
        F_R = self.pr["padR"].history[-1]["force"] if self.pr["padR"].history else 0.0
        return F_L, F_R

    def on_exit(self, k, t, bq):
        i = PICK.index(k)
        if k == "settle":
            self.ev["card_rest_z_mm"] = round(float(bq[self.inew, 2]) * MM, 3)
        if k == "down":
            self.go("push", t)
            return
        if k == "unload":
            # where the side pinch goes, from the card's MEASURED rest pose
            e = self.card_edge_y(bq)
            c, R, _, _ = C.card_frame(bq[self.inew])
            self.ev.update(card_edge_y_mm=round(e * MM, 2), overhang_rest_mm=round((e - EDGE_Y) * MM, 2))
            self.y_pinch = e - GRIP + C.TIP_BELOW_TCP
            self.y_stand = e + STANDOFF + C.TIP_BELOW_TCP
            self.z_mid = float(c[2])
        if k == "close":
            T_tcp = self.tcp_meas(bq)
            c, R, _, _ = C.card_frame(bq[self.inew])
            self.ev["pinch_t"] = round(t, 3)
            self.ev["droop_mm"] = np.round(self.droop * MM, 3).tolist()
            print(f"[{NAME}] pinch: card origin in the TCP frame "
                  f"{np.round((np.linalg.inv(T_tcp) @ P.T_of(c, R))[:3, 3]*MM, 2)} mm, pads "
                  f"{np.round(self._pads(), 2)} N, droop {np.round(self.droop*MM, 2)} mm")
        if k == "lift":
            # the grasp the turn and the insertion are planned with: measured
            # once the board no longer supports the card
            T_tcp = self.tcp_meas(bq)
            c, R, _, _ = C.card_frame(bq[self.inew])
            self.T_tc = np.linalg.inv(T_tcp) @ P.T_of(c, R)
            print(f"[{NAME}] lifted: card origin in the TCP frame "
                  f"{np.round(self.T_tc[:3, 3]*MM, 2)} mm")
        if k == "lift2":
            # the hover the turn ends at: the insertion expert's own target
            O, Rf, nf, uf = C.card_frame(bq[self.deck_ids[-1]])
            self.deck_frame = (O, nf, uf, Rf)
            tmp = P.PlaceExpert(self.rig, self.sc, self.trk, self.T_off, self.i7, self.inew,
                                self.deck_ids, (None, None), self.fps, t0_phase="align", t0=t,
                                handover=dict(T_tc=self.T_tc, deck_frame=self.deck_frame,
                                              corr=np.zeros(3), q_grip=self.q_grip))
            c_h, R_h = tmp.card_target(P.S_HOVER)
            p_h, Rt_h = tmp.tcp_from_card(c_h, R_h)
            self.p_hover, self.R_hover = p_h, Rt_h
            self.p_hover_up = p_h + np.array([0.0, 0.0, HOVER_UP])
        if k == "hover":
            self.place = P.PlaceExpert(
                self.rig, self.sc, self.trk, self.T_off, self.i7, self.inew, self.deck_ids,
                (None, None), self.fps, t0_phase="align", t0=t,
                handover=dict(T_tc=self.T_tc, deck_frame=self.deck_frame, corr=self.droop,
                              q_grip=self.q_grip))
            print(f"[{NAME}] t={t:5.2f}s handover to the insertion expert")
            return
        if k == "unload" and self.k == "unload":
            self.go("up", t)
            return
        self.go(PICK[i + 1], t)

    @property
    def t_phase(self):
        out = dict(self.t_start)
        if self.place is not None:
            out.update({f"place_{k}": v for k, v in self.place.t_start.items()})
        return out


def main():
    rig = appendix_rig(NAME)
    rig.collide_every_substep = True
    rig.substeps = P.SUBSTEPS
    P.setup_robot(rig)
    sc = P.build_cell(rig)
    build_platform(rig)
    # the card, flat on the board, 40 mm in from the pick edge (CPP_CARD_DX /
    # CPP_CARD_YAW_DEG perturb the start for robustness runs)
    c0 = np.array([CARD_X0 + _env("CPP_CARD_DX", 0.0), EDGE_Y - CARD_INSET - 0.5 * C.CARD_H,
                   HP + 0.5 * C.CARD_T + REST_CLEAR])
    yaw = np.radians(_env("CPP_CARD_YAW_DEG", 0.0))
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0.0], [np.sin(yaw), np.cos(yaw), 0.0], [0, 0, 1.0]])
    C.add_card(rig, sc, sc.new, c0, Rz @ R_FLAT, sc.card_visual)
    C.filter_statics_and_ground(rig)
    pc_y = c0[1] - PUSH_BACK
    park = np.array([CARD_X0, pc_y, HP + C.CARD_T + C.TIP_BELOW_TCP + 0.030])
    q_start = rig.solve_arm_start(park, down_grasp_quat(0.0))
    rig.builder.joint_q[7:9] = [BLADE_Q, BLADE_Q]
    rig.builder.joint_target_q[7:9] = [BLADE_Q, BLADE_Q]
    T_off = tcp_offset_from_link7(rig)

    # primary: the whole cell from the slot's -y end (the platform stands on
    # the deck's free side, so the holder is only unblocked along the slot
    # axis); second view (multiview video): place_card_into_holder's close-up
    cam_t = np.array([_env("CP_CAM_TX", 0.12), _env("CP_CAM_TY", 0.0), _env("CP_CAM_TZ", 0.07)])
    cam_p = cam_t + np.array([_env("CP_CAM_OX", -0.10), _env("CP_CAM_OY", -0.42),
                              _env("CP_CAM_OZ", 0.29)])
    cam2_t = np.array([0.010, 0.0, 0.060])
    cam2_p = cam2_t + np.array([0.08, -0.30, 0.15])
    if os.environ.get("CP_PREVIEW"):
        rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
        preview_cameras(
            rig, os.environ["CP_PREVIEW"],
            [float(v) for v in os.environ.get("CP_PREVIEW_T", "0.3,4,12.6,15,17,19,23.7,27.5").split(",")],
            parse_cams(os.environ.get("CP_PREVIEW_CAMS", f"{','.join(map(str, cam_t))}:"
                                      f"{','.join(map(str, cam_p - cam_t))}")),
            os.environ.get("CP_PREVIEW_OUT", str(rig.out_dir / "camera_preview.png")))
        return
    rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
    from force_bench.recorder import RolloutRecorder
    rig.rec = RolloutRecorder(rig.model, rig.out_dir, cam_pos=cam_p, cam_target=cam_t,
                              cam2_pos=cam2_p, cam2_target=cam2_t, fps=rig.fps,
                              fov_deg=_env("CP_CAM_FOV", 42.0))
    deck_shapes = [l + "_shape" for l in sc.deck]
    board_shapes = ["platform_board"]
    rig.probes["padL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", [sc.new + "_shape"])
    rig.probes["padR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", [sc.new + "_shape"])
    rig.probes["card_holder"] = PairProbe(rig.model, sc.new, sc.proxy_labels)
    rig.probes["card_deck"] = PairProbe(rig.model, sc.new, deck_shapes)
    rig.probes["card_table"] = PairProbe(rig.model, sc.new, ["table"])
    rig.probes["card_board"] = PairProbe(rig.model, sc.new, board_shapes)
    for l in sc.deck:
        rig.probes[f"{l}_holder"] = PairProbe(rig.model, l, sc.proxy_labels)
    rig.probes["robot_object"] = RobotContactProbe(rig.model, [sc.new, *sc.deck],
                                                   static_shapes=["holder", "platform_board"])
    rig.probes["robot_deck"] = RobotContactProbe(rig.model, list(sc.deck), static_shapes=["holder"])
    rig.probes["robot_board"] = RobotContactProbe(rig.model, [], static_shapes=["platform_board"])
    GS.make_pipeline(rig, rigid_contact_max=40000, max_triangle_pairs=4_000_000,
                     include_static_kinematic_pairs=False)
    rig.solver = make_solver(rig.model, iterations=P.ITERS, impratio=P.IMPRATIO)
    rig.collide_every_substep = True

    inew = rig.body_index(sc.new)
    deck_ids = [rig.body_index(l) for l in sc.deck]
    i7 = rig.body_index("fr3/fr3_link7")
    trk = JointTracker(rig, q_start)
    pc = PickController(rig, sc, trk, T_off, i7, inew, deck_ids, rig.fps, rig.probes)
    log = P.Log()
    deck_p0 = [rig.s0.body_q.numpy()[i, :3].copy() for i in deck_ids]

    def last(name, key="force"):
        h = rig.probes[name].history
        return h[-1][key] if h else 0.0

    def control(t):
        F_env = last("card_holder") + last("card_deck") + last("card_table")
        pc.step(t, F_env, last("robot_deck"))
        if pc.place is not None and pc.place.finished:
            raise StopRun

    def extra(state, t):
        bq = state.body_q.numpy()
        c, R, n, u = C.card_frame(bq[inew])
        low = float(C.card_corners(bq[inew])[:, 2].min())
        T_tcp = world_T(bq[i7]) @ T_off
        deck_disp = max(float(np.linalg.norm(bq[i, :3] - p0)) for i, p0 in zip(deck_ids, deck_p0))
        ex = pc.place
        k = pc.k if ex is None else "place_" + ex.k
        row = dict(t=t, cx=c[0], cy=c[1], cz=c[2], low_z=low, lean=C.card_lean_deg(bq[inew]),
                   padL=last("padL"), padR=last("padR"), holder=last("card_holder"),
                   holder_fz=(rig.probes["card_holder"].history[-1]["vec"][2]
                              if rig.probes["card_holder"].history else 0.0),
                   deck=last("card_deck"), table=last("card_table"), board=last("card_board"),
                   board_fz=(rig.probes["card_board"].history[-1]["vec"][2]
                             if rig.probes["card_board"].history else 0.0),
                   robot_obj=last("robot_object"), robot_deck=last("robot_deck"),
                   robot_board=last("robot_board"), tcp_x=T_tcp[0, 3], tcp_y=T_tcp[1, 3],
                   tcp_z=T_tcp[2, 3], q_grip=(pc.q_f if ex is None else ex.q_fingers),
                   deck_disp=deck_disp, s=(ex.s if ex is not None else P.S_HOVER),
                   overhang=pc.card_edge_y(bq) - EDGE_Y, press=pc.press)
        log.add(**row)
        if int(round(t * rig.fps)) % 15 == 0:
            print(f"  [{t:5.2f} {k:14s}] pads {row['padL']:5.2f}/{row['padR']:5.2f} N  card "
                  f"{np.round(c*MM, 1)} low {low*MM:6.2f} lean {row['lean']:6.2f} oh "
                  f"{row['overhang']*MM:6.2f}  board {row['board']:.3f} env h{row['holder']:.3f} "
                  f"d{row['deck']:.3f}  robot {row['robot_obj']:.3f}/deck {row['robot_deck']:.3f}"
                  f"/board {row['robot_board']:.3f}  press {pc.press*MM:+.2f}  deck_disp "
                  f"{deck_disp*MM:.3f}")
        return {"padL_N": round(row["padL"], 3), "padR_N": round(row["padR"], 3),
                "card_board_N": round(row["board"], 4),
                "card_holder_N": round(row["holder"], 4), "card_deck_N": round(row["deck"], 4),
                "robot_object_contact_N": round(row["robot_obj"], 4),
                "robot_board_N": round(row["robot_board"], 4),
                "overhang_mm": round(row["overhang"] * MM, 3),
                "card_low_z_mm": round(low * MM, 3), "deck_disp_mm": round(deck_disp * MM, 3)}

    stop_t = _env("CP_STOP_T", 0.0)
    try:
        rec = rig.run(stop_t if stop_t > 0 else T_BUDGET, control, extra)
    except StopRun:
        rec = rig.rec
    ex = pc.place
    print(f"[{NAME}] rollout ended at t={len(rec.times) / rig.fps:.2f}s; pick events {pc.ev}; "
          f"place events {ex.events if ex else None}")
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "card_board_N", "card_holder_N",
                                            "card_deck_N", "robot_object_contact_N"])
    print("saved:", paths)
    (rig.out_dir / "expert_log.json").write_text(json.dumps(
        {"phase_start_s": pc.t_phase, "pick_events": pc.ev,
         "place_events": ex.events if ex else None, "rows": log.rows[::3]}, default=float))
    if stop_t > 0 or ex is None or not ex.finished:
        print("stopped early: QA skipped")
        return
    # ── picked up from the platform ─────────────────────────────────────
    lt = log.arr("t")
    w = 9.81 * C.card_mass()
    rest = (lt > 0.2) & (lt < T_SETTLE)
    t_lift = pc.t_start["lift"]
    t_lifted = pc.t_start["lift2"]
    after = lt >= t_lift + 0.4
    board_after = float(log.arr("board")[after].max()) if after.any() else 9.9
    low = log.arr("low_z")
    lift_mm = float(low[(lt >= t_lifted) & (lt < pc.t_start["turn"])].min() - HP) * MM
    pL, pR = log.arr("padL"), log.arr("padR")
    m_tr = (lt >= t_lift) & (lt < (ex.events.get("release_t") or lt[-1]))
    both = float(((pL[m_tr] > 0.5) & (pR[m_tr] > 0.5)).mean()) if m_tr.any() else 0.0
    pick_ok = bool(abs(float(log.arr("board_fz")[rest].mean()) - w) < 0.3 * w
                   and pc.ev.get("overhang_rest_mm", 0.0) >= 15.0 and lift_mm >= 30.0
                   and board_after < 0.01 and both >= 0.98)
    crit = {"picked_up_from_platform": {
        "pass": pick_ok,
        "board_support_at_start_N": round(float(log.arr("board_fz")[rest].mean()), 4),
        "weight_N": round(w, 4),
        "start_inset_mm": CARD_INSET * MM, "overhang_after_slide_mm": pc.ev.get("overhang_rest_mm"),
        "lowest_point_above_board_after_lift_mm": round(lift_mm, 1),
        "card_board_contact_after_lift_max_N": round(board_after, 4),
        "both_pads_loaded_fraction_lift_to_release": round(both, 3),
        "platform_height_mm": HP * MM,
        "robot_board_contact_peak_N": round(float(log.arr("robot_board").max()), 3),
        "note": "the card rests on the board at the start, is slid by a force-servoed blade "
                "press to the edge, pinched on its two faces at the overhang (no joint, weld "
                "or kinematic attachment exists), lifted >= 30 mm, never touches the board "
                "again and is carried by both pads to the release"}}
    def board_pen(bq, t, labels, frames):
        """card box vs the board box, exact (box points in box), both ways."""
        ic = labels.index(sc.new)
        pts = C.card_surface_points()
        half_b = np.array([BOARD_HALF, BOARD_HALF, 0.5 * BOARD_T])
        pose_b = np.array([BOARD_C[0], BOARD_C[1], HP - 0.5 * BOARD_T, 0, 0, 0, 1.0])
        gx = np.linspace(-1, 1, 41)
        board_pts = np.array([[a, b, c] for a in gx for b in gx for c in (-1, 1)]) * half_b \
            + pose_b[:3]
        worst, wt = -1.0, None
        for f in sorted(set(frames) | set(range(0, len(t), 4))):
            c_, R_, _, _ = C.card_frame(bq[f, ic])
            d1 = C.point_in_box_depth(pts @ R_.T + c_, pose_b, half_b).max()
            d2 = C.point_in_box_depth(board_pts, bq[f, ic]).max()
            d = float(max(d1, d2))
            if d > worst:
                worst, wt = d, float(t[f])
        return {f"{sc.new}<->platform_board": {"max_depth_mm": round(worst * MM, 3), "at_t": wt,
                                               "pass": bool(worst < 0.0008)}}

    P.verdict(rig, sc, ex, log, paths, extra_criteria=crit, extra_pen=board_pen,
              task_title="Place card into holder (picked up from a platform first)",
              source="new task (no suite counterpart): pick as force_bench.tasks."
                     "slide_off_edge / appendix pick_up_object_on_platform, place as "
                     "appendix place_card_into_holder",
              transport_from=t_lift,
              envelope=([-0.20, -0.30, -0.002], [0.40, 0.25, 0.40]),
              changes=[
                  "chained: the card starts flat on a raised platform (finalized-kit board "
                  "footprint, 254 mm, raised from 53 to 120 mm so the side pinch's hand "
                  "housing clears the table) 40 mm in from its edge, beside the holder",
                  "pick: force-servoed blade press drags it to a 21.5 mm overhang; side pinch "
                  "6 mm onto the overhang (pads closing vertically); lifted 60 + 50 mm",
                  "turn: the hand turns the card upright to the deck's measured lean on the "
                  "way to the holder, then place_card_into_holder's align / descend / release "
                  "/ retreat",
              ])


if __name__ == "__main__":
    wp.init()
    main()
