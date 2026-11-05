"""Appendix task 30 "Turn dial to target position" (paper Table C1) on the three printed dials of the kit.

    Success: the dial is turned to the target position.  For dials that can be turned continuously, the final
    position should be within half a tick interval from the target position.  The robot movement stops after
    terminating the rotation motion, moving away from, and breaking contact with the task objects.

THE DIALS are the finalized printable CAD (``assets/kit/dial*``), built by ``force_bench.kit.add_asset`` with the
mechanism model of ``force_bench.kit.mechanisms.dial`` (its docstring has the model and every parameter's
provenance; ``force-bench kit check dial dial-push-down-to-turn dial-snap-to-position`` measures them):

    --dial push      push-down-to-turn: locked at 0 deg by the cover's tooth and peg, turns only when pressed
                     3.0 mm (~24 N against two accordion springs), stays down off 0 deg (the tooth and peg ride
                     the knob's face), pops up back at 0 deg.  Default target +90 deg (six 15 deg ticks).
    --dial standard  continuous: a six-pad printed friction brake on the knob's round hub; ~0.9 N.m to turn,
                     holds any angle.  Default target +90 deg.
    --dial snap      snap-to-position: the same pads on a rounded-hexagon hub, a detent every 60 deg (one per
                     numeral), 0.2-1.6 N.m.  Default target +120 deg (two positions).
    --knob flat | round   the website's "Knob geometry" variant: a 10 mm wing bar or a 44 mm round grip.

Rows (``force-bench list``): turn_dial_to_target (push, flat), turn_dial_to_target_round, turn_dial_to_target_
standard[_round], turn_dial_to_target_snap[_round].  ``--target-deg`` picks another tick (15 deg grid; for the
snap dial a 60 deg position).  Angles: 0 = the arrow on "0", positive counter-clockwise seen from above.

START.  The dial on the table at 0 deg; the open hand straddles the knob's grip (across the wing bar's two flats,
or across the round grip), fingertips 5 mm above the grip's base.

EXPERT (privileged state; nothing turns, presses or holds the knob but the two fingertips):

  close      the pads close on the grip; the squeeze is the finger force limit (the Franka Hand's grasp force).
  try        (push) with a light squeeze (4 N) the wrist turns 12 deg without pressing: the gate must hold the
             knob (the bar pries the light pinch open instead), then the wrist turns back.
  press      (push) full squeeze; the TCP height is servoed on the MEASURED knob press (joint coordinate) to
             3.25 mm, between the gate clearing (3.0 mm) and the post stop (3.5 mm): the spring load (~26 N) is
             carried by pad friction, and the arm never leans on the stop.
  turn       wrist yaw ramp toward the target at 45 deg/s, paused while the wrist leads the knob by more than
             LAG_MAX (slip), then a trim on the measured knob angle until it is within TRIM_TOL of the target.
  release    the fingers open where they are (push: the springs lift the knob onto the cover's tooth and peg).
  retreat    the hand rises 60 mm, then holds still for 1 s.

VERDICT (appendix_qa.json):
  dial_at_target           |final angle - target| <= 7.5 deg (half the 15 deg tick interval) after the hand has
                           gone; snap dial: in the target detent, |error| <= 3 deg with the six pads back on the
                           hub flats (mean pad deflection within 0.15 mm of the seated preload)
  dial_stays_after_release angle drift over the still phase < 2 deg
  unpressed_try_blocked    (push) the knob turned < 3 deg during the unpressed try
  held_down_off_zero       (push) after release the knob stays pressed >= 2.5 mm: the cover's tooth and peg hold
                           it (real contact; the old emulated hold-down is gone)
  finger_grasp_real        mean pad force > 2 N while turning
  released_and_clear       robot <-> knob / sprung parts / housing contact < 0.05 N over the still window, the
                           hand >= 40 mm away from where it let go
  physics integrity        finite, velocity spikes, knob <-> housing penetration (0.5 mm)

SEEDED STARTS (``--seed N``, ``force-bench run turn_dial_to_target --seed N``): the dial's pose is drawn uniformly in
the paper's 0.15 x 0.15 m square at the board centre and within +-15 deg of yaw (``DIAL_PLACE``,
``common.sample_start``); the target stays relative to the dial's own 0 mark.  The hand starts open around the knob
wherever it lands, turned with the dial, and the arm's start configuration is walked over from the nominal one: the
same IK branch, so joint 7 keeps the room the wrist turn needs (a fresh solve lands on other branches across the
square, some with joint 7 too close to its limit for a 120 deg turn).  Joint 7 sets the yaw range: the wrist turns
through the dial's yaw plus the target, and over the square (IK sweep, robot base where it is) every joint keeps
>= 0.13 rad of margin through the snap dial's 120 deg turn plus 10 deg of trim at +-15 deg (joint 7, at the
robot-side corner on the left; 0.05 rad at +20 deg) and >= 0.19 rad through the 90 deg turns (the elbow, at the
robot-side edge).  The camera keeps its view of the dial.  The draw is recorded in the verdict ("start").

Run:  force-bench run turn_dial_to_target [--target-deg 45] [--seed N]
      python -m force_bench.appendix_tasks.turn_dial_to_target --dial snap --knob round
      --seed N  random dial pose (0.15 x 0.15 m square, +-15 deg); ``force-bench run`` names the folder
                <row>__seed<N>
"""

from __future__ import annotations

import argparse
import json

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.appendix_tasks.common import (Placement, RobotContactProbe, StopRun, appendix_rig,
                                               check_released_and_clear, ease, run_until, sample_start,
                                               save_appendix_verdict)
from force_bench.appendix_tasks.registry import CRITERIA
from force_bench.assets import set_body_mass
from force_bench.kit import add_asset, load_kit, place_on_table
from force_bench.kit.mechanisms import dial as D
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.nut_bolt_arm import PairProbe

MM = 1e-3
DIAL_KEYS = {"push": "dial-push-down-to-turn", "standard": "dial", "snap": "dial-snap-to-position"}
KNOB_LABELS = {"flat": "Flat knob", "round": "Round knob"}
DEFAULT_TARGET_DEG = {"push": 90.0, "standard": 90.0, "snap": 120.0}
TICK_DEG = {"push": 15.0, "standard": 15.0, "snap": 60.0}       # tick interval (snap: one position per numeral)
HALF_TICK_DEG = 7.5

# ── layout ────────────────────────────────────────────────────────────────────────────────────────────────────────
ROBOT_BASE = (0.0, 0.42, 0.0)          # IK-swept: wrist yaw -170..+165 deg reachable at the knob
DIAL_XY = (0.0, 0.0)
DIAL_DYAW_DEG = 15.0                   # --seed: the dial's yaw range about 0 (joint 7's room for the wrist turn)
DIAL_PLACE = Placement(DIAL_XY, yaw=0.0, dxy=0.075, dyaw=np.radians(DIAL_DYAW_DEG), radius=0.05)
GRIP_BASE = 28.0 * MM                  # the knob's grip starts above its hub step (CAD z, all six knobs)
TIP_ABOVE_GRIP = 5.0 * MM              # fingertips this far above the grip's base
TIP_BELOW_TCP = 9.93 * MM              # FR3 collision tips below the TCP (physics_notes pitfall 7.1)
PAD_OFF = 0.000085                     # pad face = finger dof - this
CAM_POS = np.array([0.175, -0.215, 0.135])
CAM_TGT = np.array([0.0, 0.0, 0.040])

# ── solver / contact ──────────────────────────────────────────────────────────────────────────────────────────────
SUBSTEPS = 32
ITERS = 20
IMPRATIO = 10.0                        # stiff friction rows (plug_in_usb, install_light_bulb): the pinch twists
ROBOT_GAP = 0.003                      # robot / ground detection gap (install_light_bulb: triangle-pair census)

# ── fingers ───────────────────────────────────────────────────────────────────────────────────────────────────────
FINGER_KE, FINGER_KD = 20000.0, 80.0   # stiff position drive; the squeeze is set by the force limit below
FINGER_MASS = 0.12
PAD_MU = 1.4                           # elastomer pads (suite value; MuJoCo pair mu = max)
SQUEEZE = {"flat": 70.0, "round": 45.0}   # N per finger.  Flat: the Franka Hand's 70 N continuous grasp force --
                                          # the bar is turned by form closure over the pads' half-width (8.75 mm),
                                          # so the squeeze IS the torque capacity.  Round: a friction grip at r 22 mm.
SQUEEZE_TRY = 4.0                      # N, the light pinch of the unpressed try
OPEN_CLEAR = 6.0 * MM                  # pad clearance per side when open

# ── expert ────────────────────────────────────────────────────────────────────────────────────────────────────────
T_CLOSE = 0.5
XY_GAIN = 0.15                         # per frame: TCP xy servo on the measured finger midpoint (close, try) --
XY_RATE = 0.05 * MM                    # the arm's own sag puts the hand ~mm off the knob axis, and the
XY_MAX = 4.0 * MM                      # mimic-coupled fingers then pinch one-sided (install_light_bulb's servo)
TRY_DEG, T_TRY = 12.0, 1.2
PRESS_GOAL = 3.25 * MM                 # between the gate clearing (3.0) and the post stop (3.5)
PRESS_GAIN = 0.15                      # per frame on the press error
PRESS_RATE = 0.10 * MM                 # m/frame cap on the TCP press command change
T_PRESS = 1.2
OMEGA = np.radians(45.0)               # rad/s wrist ramp
LAG_MAX = np.radians(8.0)              # pause the ramp while the wrist leads the knob by more than this
EXPERT_TRIM_GAIN = 0.20                # per frame on the knob angle error
EXPERT_TRIM_CLAMP = np.radians(15.0)
TRIM_TOL = np.radians(0.5)
T_TRIM_MIN, T_TRIM_MAX = 0.3, 2.5
T_OPEN, T_RETREAT, T_STILL = 0.5, 1.2, 1.0
RETREAT = 0.060

# ── verdict ───────────────────────────────────────────────────────────────────────────────────────────────────────
DRIFT_MAX_DEG = 2.0
TRY_MAX_DEG = 3.0
HELD_DOWN_MIN = 2.5 * MM
SNAP_TOL_DEG = 3.0
SNAP_SEAT_MM = 0.15
PEN_TOL = 0.0005
MOVE_MIN = 0.040


def row_name(dial: str, knob: str) -> str:
    return "turn_dial_to_target" + ("" if dial == "push" else f"_{dial}") + ("_round" if knob == "round" else "")


def _robot_setup(rig, knob: str):
    rig.add_robot(wp.transform(wp.vec3(*ROBOT_BASE), wp.quat_identity()))
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    set_finger_effort_limit(rig.builder, SQUEEZE[knob])
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        set_body_mass(rig.builder, bi, FINGER_MASS)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU
    for i, lbl in enumerate(rig.builder.shape_label):
        b = rig.builder.shape_body[i]
        if (b >= 0 and str(rig.builder.body_label[b]).startswith("fr3/")) or (lbl and "ground" in str(lbl)):
            rig.builder.shape_gap[i] = ROBOT_GAP


def _grip_geometry(asset, choice, knob: str, up: float) -> dict:
    """Grip band the pads close on (CAD measurements): TCP height and the half-width at the pads."""
    roles = D._parts_by_role(asset.parts(choice))
    tm = roles["knob"].mesh()
    z_tip = GRIP_BASE + up + TIP_ABOVE_GRIP
    zs = np.linspace(z_tip, z_tip + 18.0 * MM, 7)
    axis = 3 if knob == "flat" else 2            # flat: bar half-thickness (y); round: grip radius (x)
    half = max(D._section_mm(tm, (z - up) / MM).bounds[axis] for z in zs) * MM
    return {"top": float(tm.bounds[1][2]) + up, "tip_z": z_tip, "tcp_z": z_tip + TIP_BELOW_TCP, "half": float(half)}


class Expert:
    def __init__(self, rig, h, dial: str, knob: str, target: float, g: dict, probes: dict, fingers: list[int],
                 yaw0: float = 0.0):
        self.rig, self.h, self.dial, self.knob = rig, h, dial, knob
        self.yaw0 = float(yaw0)                               # the dial's yaw: wrist yaws are relative to it
        self.target = target
        self.p = probes
        self.fi = fingers
        self.axis = np.asarray(h.pose[:2, 3], float)          # the dial axis in the world (CAD origin)
        self.tcp0 = np.array([self.axis[0], self.axis[1], g["tcp_z"]])
        self.xy = np.zeros(2)                                 # xy servo offset of the TCP command
        self.f_touch = g["half"] + PAD_OFF
        self.f_open = self.f_touch + OPEN_CLEAR
        self.f_grip = max(0.0, self.f_touch - 1.5 * SQUEEZE[knob] / FINGER_KE)
        self.st = {"phase": "close", "t0": 0.0, "phases": [], "yaw_cmd": 0.0, "trim": 0.0, "press_cmd": 0.0,
                   "in_tol": 0.0, "z_release": None, "yaw_release": None, "t_still": None}
        self.log = []
        self._squeeze = None

    def squeeze(self, f):
        """Finger force limit (MuJoCo actuator force range), changed at run time."""
        if self._squeeze == f:
            return
        self._squeeze = f
        lim = self.rig.model.joint_effort_limit.numpy()
        lim[7:9] = f
        self.rig.model.joint_effort_limit.assign(lim)
        self.rig.solver.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)

    def go(self, phase, t):
        self.st["phases"].append((phase, round(t, 3)))
        self.st["phase"], self.st["t0"] = phase, t
        ang, press = D.knob_coords(self.rig, self.h)
        print(f"[dial] t={t:5.2f}s -> {phase}  (knob {np.degrees(ang):+.2f} deg, press {-press / MM:.3f} mm)")

    def drive(self, yaw, z, f):
        p = self.tcp0[:2] + self.xy
        self.rig.drive_arm(np.array([p[0], p[1], z]), down_grasp_quat(self.yaw0 + yaw), fingers=f)

    def hand_offset(self) -> np.ndarray:
        """Measured finger midpoint minus the dial axis (xy, m)."""
        bq = self.rig.s0.body_q.numpy()
        return 0.5 * (bq[self.fi[0], :2] + bq[self.fi[1], :2]) - self.axis

    def center(self):
        """TCP xy servo: put the finger midpoint on the dial axis (only while the hand is not clamped)."""
        step = np.clip(-XY_GAIN * self.hand_offset(), -XY_RATE, XY_RATE)
        self.xy = np.clip(self.xy + step, -XY_MAX, XY_MAX)

    def control(self, t):
        st = self.st
        ang, press = D.knob_coords(self.rig, self.h)
        tau = t - st["t0"]
        ph = st["phase"]
        z0 = self.tcp0[2]
        if ph == "close":
            self.squeeze(SQUEEZE_TRY if self.dial == "push" else SQUEEZE[self.knob])
            if self.dial == "push" or tau < 0.6 * T_CLOSE:
                self.center()
            u = ease(tau / T_CLOSE)
            self.drive(0.0, z0, self.f_open + (self.f_grip - self.f_open) * u)
            if tau >= T_CLOSE + 0.2:
                self.go("try" if self.dial == "push" else "turn", t)
            return
        if ph == "try":
            self.center()
            s = min(1.0, tau / T_TRY)
            self.drive(np.radians(TRY_DEG) * np.sin(np.pi * s), z0, self.f_grip)
            if tau >= T_TRY + 0.3:
                self.squeeze(SQUEEZE[self.knob])
                self.go("press", t)
            return
        if self.dial == "push" and ph in ("press", "turn", "trim"):
            # TCP height servo on the measured press: the command moves at most PRESS_RATE per frame
            err = PRESS_GOAL - (-press)
            st["press_cmd"] = float(np.clip(st["press_cmd"] + np.clip(PRESS_GAIN * err, -PRESS_RATE, PRESS_RATE),
                                            0.0, PRESS_GOAL + 3.0 * MM))
        z = z0 - st["press_cmd"]
        if ph == "press":
            self.drive(0.0, z, self.f_grip)
            if (tau >= T_PRESS and -press >= PRESS_GOAL - 0.15 * MM) or tau >= 3.0 * T_PRESS:
                self.go("turn", t)                 # (a press that never got there fails the verdict)
            return
        if ph == "turn":
            lead = st["yaw_cmd"] - ang
            step = OMEGA / self.rig.fps * np.sign(self.target - st["yaw_cmd"])
            if abs(lead) < LAG_MAX or np.sign(lead) != np.sign(step):
                st["yaw_cmd"] = float(self.target) if abs(self.target - st["yaw_cmd"]) <= abs(step) \
                    else float(st["yaw_cmd"] + step)
            self.drive(st["yaw_cmd"], z, self.f_grip)
            if st["yaw_cmd"] == self.target:
                self.go("trim", t)
            return
        if ph == "trim":
            err = self.target - ang
            st["trim"] = float(np.clip(st["trim"] + EXPERT_TRIM_GAIN * err, -EXPERT_TRIM_CLAMP, EXPERT_TRIM_CLAMP))
            self.drive(self.target + st["trim"], z, self.f_grip)
            st["in_tol"] = st["in_tol"] + 1.0 / self.rig.fps if abs(err) < TRIM_TOL else 0.0
            if (tau >= T_TRIM_MIN and st["in_tol"] >= 0.2) or tau >= T_TRIM_MAX:
                st["z_release"], st["yaw_release"] = z, self.target + st["trim"]
                self.go("open", t)
            return
        yaw, zr = st["yaw_release"], st["z_release"]
        if ph == "open":
            u = ease(tau / T_OPEN)
            self.drive(yaw, zr, self.f_grip + (self.f_open - self.f_grip) * u)
            if tau >= T_OPEN:
                self.go("retreat", t)
            return
        if ph == "retreat":
            self.drive(yaw, zr + RETREAT * ease(tau / T_RETREAT), self.f_open)
            if tau >= T_RETREAT:
                st["t_still"] = t
                self.go("still", t)
            return
        if tau >= T_STILL:
            raise StopRun
        self.drive(yaw, zr + RETREAT, self.f_open)

    def extra(self, state, t):
        ang, press = D.knob_coords(self.rig, self.h, state)
        fl = self.p["finL"].history[-1]["force"] if self.p["finL"].history else 0.0
        fr = self.p["finR"].history[-1]["force"] if self.p["finR"].history else 0.0
        ro = self.p["robot_object"].history[-1]["force"] if self.p["robot_object"].history else 0.0
        bq = state.body_q.numpy()
        tips = 0.5 * (bq[self.fi[0], :3] + bq[self.fi[1], :3])
        pads = []
        if self.dial != "push":
            q = state.joint_q.numpy()
            qs = self.rig.model.joint_q_start.numpy()
            pads = [float(q[qs[self.h.joints[f"pad_{k}"]]]) for k in range(6)]
        off = tips[:2] - self.axis
        self.log.append({"t": t, "phase": self.st["phase"], "ang": ang, "press": press, "yaw_cmd": self.st["yaw_cmd"],
                         "fL": fl, "fR": fr, "robot": ro, "tips": tips.tolist(), "pads": pads,
                         "off_x": float(off[0]), "off_y": float(off[1])})
        return {"knob_deg": round(float(np.degrees(ang)), 2), "press_mm": round(-press / MM, 3),
                "pads_N": round(fl + fr, 2), "robot_object_contact_N": round(ro, 4)}


def build(dial: str, knob: str, name: str, start=(DIAL_XY[0], DIAL_XY[1], 0.0)):
    key = DIAL_KEYS[dial]
    choice = {"Knob geometry": KNOB_LABELS[knob]}
    rig = appendix_rig(name)
    rig.substeps = SUBSTEPS
    rig.collide_every_substep = True
    _robot_setup(rig, knob)
    rig.table(half=0.35)
    pose = place_on_table(key, choice, xy=(start[0], start[1]), yaw=float(start[2]))
    h = add_asset(rig, key, choice, pose=pose)
    # the arm never touches the knob except with its fingertips (Rig.robot_grip's contact policy)
    rig.robot_grip(h.bodies["knob"], filter_fingers=False)
    g = _grip_geometry(load_kit()[key], choice, knob, h.info.get("up_gap_mm", 0.0) * MM)
    return rig, h, g, key, choice


def run(dial: str, knob: str, target_deg: float, name: str | None = None, seed: int | None = None) -> dict:
    name = name or row_name(dial, knob)
    start = sample_start({"dial": DIAL_PLACE}, seed)["dial"]
    yaw0 = float(start[2])
    rig, h, g, key, choice = build(dial, knob, name, start)
    tcp0 = np.array([h.pose[0, 3], h.pose[1, 3], g["tcp_z"]])
    if seed is not None:
        # walk the IK over from the nominal start's solution (its branch leaves joint 7 the room of the wrist turn)
        nominal = place_on_table(key, choice, xy=DIAL_XY)
        p_nom = np.array([nominal[0, 3], nominal[1, 3], g["tcp_z"]])
        rig.ik.solve(p_nom, down_grasp_quat(0.0), iterations=64)
        for u in np.linspace(0.0, 1.0, 16):
            rig.ik.solve(p_nom + u * (tcp0 - p_nom), down_grasp_quat(u * yaw0), iterations=32)
    rig.solve_arm_start(tcp0, down_grasp_quat(yaw0))
    f_open = g["half"] + PAD_OFF + OPEN_CLEAR
    rig.builder.joint_q[7:9] = [f_open, f_open]
    rig.builder.joint_target_q[7:9] = [f_open, f_open]
    shift = np.array([start[0] - DIAL_XY[0], start[1] - DIAL_XY[1], 0.0])     # the camera keeps its view of the dial
    rig.finalize(probes=[f"{h.label}/knob"], cam_pos=CAM_POS + shift, cam_target=CAM_TGT + shift)
    rig.solver = make_solver(rig.model, iterations=ITERS, impratio=IMPRATIO)
    labels = [str(lbl) for lbl in rig.model.shape_label]
    knob_shapes = [lbl for lbl in labels if lbl.startswith(f"kit/{h.label}/") and "knob" in lbl]
    static_labels = [lbl for lbl in labels if lbl.startswith(f"kit/{h.label}/")
                     and any(s in lbl for s in ("_top", "_bottom", "gate_"))]
    moving = [rig.model.body_label[b] for b in h.bodies.values()]
    probes = {"finL": PairProbe(rig.model, "fr3/fr3_leftfinger", counterpart_shapes=knob_shapes),
              "finR": PairProbe(rig.model, "fr3/fr3_rightfinger", counterpart_shapes=knob_shapes),
              "robot_object": RobotContactProbe(rig.model, moving, static_shapes=static_labels)}
    rig.probes.update(probes)
    fingers = [rig.body_index("fr3/fr3_leftfinger"), rig.body_index("fr3/fr3_rightfinger")]
    ex = Expert(rig, h, dial, knob, np.radians(target_deg), g, probes, fingers, yaw0)
    budget = 16.0 + abs(target_deg) / 45.0
    run_until(rig, budget, ex.control, ex.extra, lambda t: False)
    paths = rig.rec.save("rollout", force_keys=["pads_N", "robot_object_contact_N"])
    return verdict(rig, h, g, ex, dial, knob, target_deg, key, choice, paths, seed, start)


def verdict(rig, h, g, ex, dial, knob, target_deg, key, choice, paths, seed=None, start=None) -> dict:
    L = ex.log
    t = np.array([r["t"] for r in L])
    ang = np.degrees(np.array([r["ang"] for r in L]))
    press = -np.array([r["press"] for r in L])
    phases = [r["phase"] for r in L]
    st = ex.st
    final = float(ang[-1])
    err = final - target_deg
    t_still = st["t_still"] if st["t_still"] is not None else float(t[-1])
    still = t >= t_still
    drift = float(np.ptp(ang[still])) if still.any() else float("nan")
    crit = {}
    if dial == "snap":
        pads = np.array([r["pads"] for r in L]) / MM
        seat, seat0 = float(pads[-1].mean()), float(pads[0].mean())
        crit["dial_at_target"] = {
            "pass": bool(abs(err) <= SNAP_TOL_DEG and seat - seat0 <= SNAP_SEAT_MM), "target_deg": target_deg,
            "final_deg": round(final, 2), "error_deg": round(err, 2), "tolerance_deg": SNAP_TOL_DEG,
            "positions_every_deg": TICK_DEG[dial], "pad_deflection_final_mm": round(seat, 3),
            "pad_deflection_seated_mm": round(seat0, 3), "seat_tolerance_mm": SNAP_SEAT_MM,
            "measured_after": "release + 6 cm lift + still",
            "note": "snap-to-position dial: the knob must sit in the target detent (pads back on the hub flats)"}
    else:
        crit["dial_at_target"] = {
            "pass": bool(abs(err) <= HALF_TICK_DEG), "target_deg": target_deg, "final_deg": round(final, 2),
            "error_deg": round(err, 2), "tolerance_deg": HALF_TICK_DEG, "tick_interval_deg": TICK_DEG[dial],
            "measured_after": "release + 6 cm lift + still"}
    crit["dial_stays_after_release"] = {"pass": bool(drift < DRIFT_MAX_DEG), "drift_still_deg": round(drift, 3)}
    if dial == "push":
        tr = np.array([p == "try" for p in phases])
        blocked = float(np.abs(ang[tr]).max()) if tr.any() else float("nan")
        crit["unpressed_try_blocked"] = {"pass": bool(blocked < TRY_MAX_DEG), "rotation_deg": round(blocked, 3),
                                         "wrist_try_deg": TRY_DEG, "squeeze_N": SQUEEZE_TRY,
                                         "gate_play_deg": h.info.get("gate_play_deg")}
        turning = np.array([p in ("turn", "trim") for p in phases])
        crit["held_down_off_zero"] = {
            "pass": bool(press[-1] >= HELD_DOWN_MIN and abs(final) > 15.0),
            "final_press_mm": round(float(press[-1] / MM), 3), "min_mm": round(HELD_DOWN_MIN / MM, 2),
            "press_while_turning_mm": [round(float(press[turning].min() / MM), 3),
                                       round(float(press[turning].max() / MM), 3)] if turning.any() else None,
            "gate_clears_at_mm": h.info.get("gate_engagement_mm"),
            "note": "after release the springs push the knob's face onto the cover's tooth and peg"}
    pads_n = np.array([r["fL"] + r["fR"] for r in L])
    turning = np.array([p in ("turn", "trim") for p in phases])
    crit["finger_grasp_real"] = {"pass": bool(turning.any() and pads_n[turning].mean() > 2.0),
                                 "mean_pad_force_N": round(float(pads_n[turning].mean()), 2) if turning.any() else 0.0,
                                 "squeeze_limit_N": SQUEEZE[knob]}
    # robot <-> objects contact, and the hand's motion away from where it let go
    hist = rig.probes["robot_object"].history
    tc = np.array([r["t"] for r in hist])
    fc = np.array([r["force"] for r in hist])
    tips = np.array([r["tips"] for r in L])
    i_open = next((i for i, p in enumerate(phases) if p == "open"), len(L) - 1)
    moved = float(np.linalg.norm(tips[-1] - tips[i_open]))
    qa = rig.qa(paths)
    check_released_and_clear(qa, tc, fc, t_still, extra={"hand_moved_mm": round(moved * 1000, 1),
                                                         "min_move_mm": MOVE_MIN * 1000})
    crit["released_and_clear"] = dict(qa.report["checks"]["released_and_clear"])
    if moved < MOVE_MIN:
        crit["released_and_clear"]["pass"] = False
    _penetration(qa, rig, h, key, choice)
    report = rig.finish(qa)
    start = (DIAL_XY[0], DIAL_XY[1], 0.0) if start is None else start
    crit["start"] = {"pass": True, "info_only": True, "seed": seed,
                     "dial_xy_mm": [round(float(v) / MM, 2) for v in start[:2]],
                     "dial_yaw_deg": round(float(np.degrees(start[2])), 2),
                     "axis_xy_mm": [round(float(v) / MM, 2) for v in h.pose[:2, 3]],
                     "range": {"xy_mm": round(DIAL_PLACE.dxy / MM, 1), "yaw_deg": DIAL_DYAW_DEG},
                     "note": "the dial's centre on the table and its yaw about the nominal orientation; the target is "
                             "relative to the dial's 0 mark" +
                             ("" if seed is not None else "; nominal start (no seed)")}
    expert = {"phases": st["phases"], "target_deg": target_deg, "grip": g, "squeeze_N": SQUEEZE[knob],
              "params": h.params, "info": h.info,
              "turning_mean_pad_force_N": crit["finger_grasp_real"]["mean_pad_force_N"],
              "expert_log_columns": ["t", "knob_angle_rad", "knob_press_m (negative = pressed)", "yaw_cmd_rad",
                                     "pad_left_N", "pad_right_N", "robot_object_N", "hand_off_x_m", "hand_off_y_m"]}
    (rig.out_dir / "expert_state.json").write_text(json.dumps(expert, indent=1, default=_json_default))
    np.save(rig.out_dir / "expert_log.npy",
            np.array([[r["t"], r["ang"], r["press"], r["yaw_cmd"], r["fL"], r["fR"], r["robot"], r["off_x"], r["off_y"]]
                      for r in L]))
    return save_appendix_verdict(rig, task="Turn dial to target position",
                                 source=f"new: kit asset {key} ({KNOB_LABELS[knob]}), force_bench.kit.mechanisms.dial",
                                 spec=CRITERIA[30], criteria=crit, changes=_changes(dial, seed, start),
                                 physics_qa=report)


def _penetration(qa, rig, h, key, choice):
    """Knob vs the housing bottom and top, geometric (trimesh), over the rollout."""
    roles = D._parts_by_role(load_kit()[key].parts(choice))
    up = h.info.get("up_gap_mm", 0.0) * MM
    kb = h.bodies["knob"]
    q = rig.model.body_q.numpy()[kb]
    T0 = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
    T0[:3, 3] = q[:3]
    raise_up = np.eye(4)
    raise_up[2, 3] = up
    knob_local = roles["knob"].mesh()
    knob_local.apply_transform(np.linalg.inv(T0) @ h.pose @ raise_up)
    for role in ("bottom", "top"):
        tm = roles[role].mesh()
        tm.apply_transform(h.pose)
        qa.check_penetration_static(rig.model.body_label[kb], knob_local, tm, np.zeros(3), f"knob->{role}",
                                    tol=PEN_TOL, n_samples=32)


def _changes(dial, seed=None, start=None) -> list[str]:
    out = [
        "built from the finalized printable CAD (assets/kit) with force_bench.kit; the printed springs are sprung "
        "contact bodies whose stiffness is a beam-theory estimate (kit/mechanisms/dial.py), not a measurement",
        "the knob turns on a revolute joint (push: plus a vertical slide with the post as its stop); the CAD running "
        "fits around the hub, bore and cover are below contact resolution and are carried by the joint "
        "(knob/housing contact filtered)",
        "the expert reads the knob angle and press from the simulator (privileged state), as the suite's experts do",
    ]
    if dial == "push":
        out.append("unpressed try: the wrist turns 12 deg with a light 4 N pinch so that the gate, not the arm, is "
                   "tested (a firm pinch driven by the stiff arm out-muscles any contact gate, pitfall 6.1)")
    if dial == "snap":
        out.append("with PLA-on-PLA friction (mu 0.5) the printed detents do not self-centre (kit probe: a knob let go "
                   "20 deg off a detent stops at ~15 deg), so the expert trims onto the detent itself")
    if seed is not None:
        out.append(f"seeded start {seed}: dial at ({start[0] / MM:+.1f}, {start[1] / MM:+.1f}) mm, yaw "
                   f"{np.degrees(start[2]):+.1f} deg (uniform in the paper's 0.15 x 0.15 m square, "
                   f"+-{DIAL_DYAW_DEG:.0f} deg)")
    return out


def _json_default(o):
    if isinstance(o, np.floating):
        return float(o)
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    raise TypeError(type(o))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Turn dial to target position (kit dials)")
    ap.add_argument("--dial", choices=sorted(DIAL_KEYS), default="push")
    ap.add_argument("--knob", choices=sorted(KNOB_LABELS), default="flat")
    ap.add_argument("--target-deg", type=float, default=None)
    ap.add_argument("--name", default=None, help="output folder name (default: the row name)")
    ap.add_argument("--seed", type=int, default=None,
                    help=f"random dial pose (0.15 x 0.15 m square, +-{DIAL_DYAW_DEG:.0f} deg)")
    a = ap.parse_args(argv)
    target = DEFAULT_TARGET_DEG[a.dial] if a.target_deg is None else a.target_deg
    if target % TICK_DEG[a.dial]:
        raise SystemExit(f"target must be a {TICK_DEG[a.dial]:g} deg tick of the {a.dial} dial")
    return run(a.dial, a.knob, target, a.name, a.seed)


if __name__ == "__main__":
    wp.init()
    main()
