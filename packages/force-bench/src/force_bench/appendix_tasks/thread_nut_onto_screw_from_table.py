"""Appendix task "Screw nut onto screw" (Table I, #21), starting with the nut
lying on the table: the FR3 picks it up, carries it over the kit bolt block,
finds the thread start by a reverse turn, screws it on until the threads
fully engage, lets go and moves away.

    Description (appendix): "Screw the nut held in hand onto the screw head
    placed in the workspace such that their threads fully engage."
    Success: "The robot movement stops after terminating the rotation
    motion, moving away from, and breaking contact with the task objects."

User additions: the nut starts on the table and is picked up first (real
reference videos: website_assets/videos/tasks/pick_up_nut_from_platform.mp4
and thread_nut_onto_screw.mp4).  The appendix's in-hand start is the state
the expert reaches after its carry.  There is no put-back motion: the task's
end state IS the nut on the screw.

SCENE (``_nut_table_scene.py``): the finalized-kit bolt block (yellow block,
bolt axis at the world origin, 23.45 mm of shaft above the block) and its
nut lying hex-face down on the table 60 mm to the -x side, yawed
``NUT_YAW0_DEG``.  Same robot cell as ``nut_bolt_arm`` (base 0.325 m along
+y), so the suite's stroke window and IK-verified wrist bias are reused.

THE THREAD START (measured offline on the kit meshes: coaxial clearance map
C(h, psi) from ray-cast radius maps of the bolt's outer and the nut's inner
surface, 0.05 mm x 1 deg grid; psi = nut yaw in the kit frame).  Lowered
coaxially, the nut comes to rest in one of two states:
  * psi in [0, ~330] deg: its first thread drops into the bolt's first
    groove and rests on a PARTIAL arc of flank -- about a full turn just
    after psi = 0, shrinking to nothing at psi ~330 (nut centre 43.2 ->
    45.3 mm, rising at the lead: it is the helix);
  * psi in (~330, ~355): thread end on thread end, no flank engagement.
Once per revolution, at psi ~355 deg, the nut's thread start passes the
bolt's and the free height drops by ~2 mm -- the "click" a person feels
when backing a nut off before starting it; after it the nut has ~a full turn
of flank.  The sim reproduces this: touch-down at 44.72 mm for psi 235
(map: 44.69), click after 120.5 deg of reverse turn at psi 355.5, drop
1.93 mm.
A short arc is not a support: a nut released on ~100 deg of flank tipped
17 deg the moment the pads opened to a 3 mm cage, and the returning wrist
then dragged it off the bolt (an earlier design regrasped mid-search).  So
the pads never open before the nut has gone >= 180 deg forward from its
search pose.

EXPERT (one rollout, fingertip contact only, nothing welded or filtered):
  1. settle  hover 60 mm above the nut, pads open 4.5 mm clear of each flat;
  2. descend to the pinch pose: fingertips 2 mm above the table (keeps the
     finger probes out of the table's contact margin), pads on two
     opposite flats over the nut's upper 5 mm;
  3. pre: droop compensation -- the arm sags 1.06 mm at this pose, and the
     pads only centre the nut ALONG their closing axis; an off-centre nut
     then orbits the TCP when the wrist turns (first search run: 2.4 mm off
     the bolt axis after 180 deg, climbed the thread end, fell).  The TCP
     command is servoed until the MEASURED pinch centre is on the nut;
  4. close + hold: finger target 0.64 mm/side into the flats, pad force
     held in [3.5, 6.5] N by nut_bolt_arm's admittance loop;
  5. lift the nut 73 mm off the table, carry it over the bolt while the
     wrist turns to the start of the stroke window (the nut-in-hand offset
     measured after the lift is fed forward);
  6. align / lower / land: TCP xy servo on the MEASURED nut centre -> bolt
     axis; descend at 1.5 mm/s from ~3 mm above the expected touch until
     the nut<->bolt force says it rests on the thread;
  7. search: one reverse (CCW, unscrewing) wrist stroke of <= 180 deg at
     0.7 rad/s while a z force servo keeps the nut<->bolt touch at
     0.25-0.9 N (the nut is carried up its own flank) and the nut-centre
     servo keeps it coaxial.  A measured height drop >= 1 mm = the click.
     No click within the stroke = the nut was already engaged at
     touch-down (the click lies beyond the window); either way the stroke
     chains, still pinched, into
  8. forward regrasp strokes with nut_bolt_arm's RegraspDriver (lag-limited
     wrist sweeps, helix-following TCP z, admittance grip, open / return /
     close between strokes, xy servo on the bolt axis) until the nut's top
     face is below the bolt's full-crest thread top -- the whole nut on full
     thread (5-6 strokes);
  9. settle on the lower flank: still pinched, lower at 0.5 mm/s until the
     nut<->bolt force reads >= 0.3 N.  Screwed down by the pads the nut rides
     the UPPER flank, and at every regrasp open it falls through the
     thread's 0.6 mm axial backlash (measured 0.45-0.59 mm);
 10. open the pads, rise 70 mm, hold still.
A fault guard (nut > 2 mm off the axis or > 8 deg tilted while on the bolt)
opens the pads and retreats instead of pressing on a lost nut.

VERDICT (appendix_qa.json): picked_up_from_table, real_finger_grasp,
threads_fully_engaged (whole nut below the full-crest thread top, coaxial
< 0.5 mm, upright < 3 deg, rotation-vs-descent within 10 % of the mesh lead
between the two lower-flank states, no fault), nut_stays_put_after_release
(< 0.3 mm, < 5 deg, still), released_and_clear (< 0.05 N to nut, bolt and
block over the final window, TCP >= 50 mm away), robot_stopped; the search
outcome is recorded (thread_start_search, info); plus the suite integrity
checks (finite, velocity, penetration nut<->bolt incl. a dense bore-vertex
audit at the 16 highest-force frames, nut<->table, envelope).
Because the pads always close on flats and the landing wrist yaw is fixed,
the nut can only land in six thread phases (psi = -125 deg mod 60); all six
were run (NT_NUT_YAW_DEG -> landing phase: outcome, strokes, measured pitch;
lead 2.333 mm/turn), all PASS:
    -100 -> 235: click after 120.5 deg, 5 strokes, 2.363  (default)
     -40 -> 295: click after  61.1 deg, 6 strokes, 2.386
    -160 -> 175: click at the window edge (180.5 deg, caught by the hold),
                 5 strokes, 2.330
      80 ->  55: no click (engaged at touch-down), 6 strokes, 2.326
     140 -> 115: no click, 7 strokes, 2.330
      20 -> 355: no click, 6 strokes, 2.332

GelSight readiness: every finger target is a module constant with
GS.finger_offset() added; the grip is force-servoed on the finger
ForceProbes; corridors: the open pads clear the flats by 4.5 mm at the pick
and the regrasp opening is nut_bolt_arm's 30 mm.

Run:  python -m force_bench.appendix_tasks.thread_nut_onto_screw_from_table
      (TB_OUT_ROOT=<dir> TB_RENDER_FAST=1 TB_MULTIVIEW=0 for iterations;
       NT_STOP_AFTER=<phase> ends the rollout after that phase and skips
       the QA; NT_NUT_YAW_DEG overrides the nut's start yaw)
"""

from __future__ import annotations

import os

import numpy as np
import trimesh
import warp as wp

# GelSight phase (TB_GELSIGHT=1 only; read at force_bench.tactile.rig import):
# the pick puts the fingertips 2 mm above the table, so the sensor case must
# not stick out past the tip.  nut_bolt_arm setdefaults a 6 mm extension at
# its import (its pads ride above a bolt head); pre-empt it here.
os.environ.setdefault("GS_CASE_TIP_EXTEND", "0")
# nut_bolt_arm imports the GelSight opt-in layer (force_bench.tactile.task)
# BEFORE force_bench.rig, as a suite task must; import it first.
from force_bench.tasks import nut_bolt_arm as NA  # noqa: E402
import force_bench.tactile.task as GS
from force_bench.tasks.nut_bolt_arm import PairProbe, RegraspDriver, YawTrack, build_plan
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, StopRun, check_released_and_clear,
    save_appendix_verdict)
from force_bench.appendix_tasks import _nut_table_scene as SC
from force_bench.geometry import world_T
from force_bench.probes import ForceProbe
from force_bench.rig import make_solver
from force_bench.robot.franka import down_grasp_quat

NAME = "thread_nut_onto_screw_from_table"


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


# ── robot cell (nut_bolt_arm's) ─────────────────────────────────────────
BASE_XY = (0.0, 0.325)
YAW_BIAS = NA.YAW_BIAS            # -45 deg: where the stroke window sits
YAW0 = NA.YAW0                    # strokes sweep ctl yaw +-80 deg (+0.4 overshoot)

# ── the nut's start pose ────────────────────────────────────────────────
NUT_YAW0_DEG = _env("NT_NUT_YAW_DEG", -100.0)

# ── grasp geometry (bare FR3; GelSight adds GS.finger_offset()) ────────
TIP_BELOW_TCP = 0.00993           # fingertip collision bottom (pitfall 7.1)
TIP_CLEAR = 0.0020                # tips above the table at the pick
PAD_FACE_OFF = 0.000085           # pad face = dof - 0.085 mm
FINGER_TOUCH = 0.5 * SC.NUT_AF + PAD_FACE_OFF + GS.finger_offset()
FINGER_GRIP = NA.FINGER_GRIP      # 0.64 mm/side into the flats (already
                                  # GS-offset by nut_bolt_arm)
FINGER_OPEN_PICK = FINGER_TOUCH + 0.0045
FINGER_OPEN = NA.FINGER_OPEN      # regrasp / release opening (30 mm)
F_GRIP_OK, F_GRIP_MAX = NA.F_GRIP_OK, NA.F_GRIP_MAX      # 3.5 / 6.5 N
RELAX_RATE = NA.RELAX_RATE
RELAX_OPEN_MAX, RELAX_CLOSE_MAX = NA.RELAX_OPEN_MAX, NA.RELAX_CLOSE_MAX
F_TOUCH_PAD = 0.5                 # N: a pad counts as touching

TCP_PICK_Z = TIP_CLEAR + TIP_BELOW_TCP          # 11.93 mm
HOVER = 0.060                     # above the pick pose
CARRY_Z = 0.085                   # TCP height for the carry (nut bottom ~70 mm)

# ── timing [s] ──────────────────────────────────────────────────────────
T_SETTLE, T_DESC, T_PRE, T_CLOSE, T_HOLD = 0.4, 1.0, 0.45, 0.6, 0.3
T_LIFT, T_CARRY, T_ALIGN, T_LOWER = 1.0, 1.6, 0.6, 1.0
T_CLICK_HOLD, T_CLICK_MAX = 0.3, 1.0
T_RELEASE, T_RETREAT, T_STILL = 0.45, 1.4, 1.2
# Before letting go the nut is lowered, still pinched, onto the LOWER flank of
# its thread.  Screwed down by the pads it rides the UPPER flank (the bolt's
# ridge above pushes it down); the measured free fall at every regrasp open
# of the first full run was 0.45-0.59 mm = the thread's 0.6 mm axial
# backlash.  Settled first, the release is a no-op for the nut.
V_SETTLE = 0.0005                 # m/s
F_SETTLE = 0.30                   # N nut<->bolt: resting on the lower flank
SETTLE_MAX = 0.0009               # m: give up (backlash 0.6 mm + margin)
T_SETTLE_HOLD = 0.15
RETREAT_Z = 0.070
T_BUDGET = 95.0

# ── align / land / search ───────────────────────────────────────────────
XY_GAIN, XY_RATE = 0.30, 0.00010  # nut-centre servo (fraction/frame, m/frame)
# The arm droops ~1 mm at the pick pose (measured: pinch centre 0.9 mm off
# the commanded TCP, perpendicular to the closing axis), and the pads only
# centre the nut ALONG their closing axis.  An off-centre nut then orbits
# the TCP when the wrist turns: the first search run carried it 2.4 mm off
# the bolt axis in 180 deg, it climbed the thread end instead of dropping,
# and fell off when the pads opened.  So: droop-compensate before closing
# (measured TCP -> nut centre, while static, pitfall 8.4), and keep the
# MEASURED nut centre on the bolt axis while turning.
Z_PRELAND = 0.0475                # nut centre before the slow descent
                                  # (touch-down at 43.2-45.3 mm, see header)
V_LAND = 0.0015                   # m/s slow descent
F_TOUCH = 0.15                    # N nut<->bolt: resting on the thread
LAND_MAX_DROP = 0.006             # m below Z_PRELAND without a touch -> fault
OMEGA_SEARCH = 0.7                # rad/s reverse (CCW) turn
F_LO, F_HI, F_PANIC = 0.25, 0.90, 3.0       # N seat-touch band / pause
V_DOWN, V_DOWN_FREE, V_UP = 0.00002, 0.00005, 0.00005   # m/frame
LEAD_MAX = 0.00025                # m: command may lead the (droop-corrected)
                                  # measured TCP downward by at most this
DROP_MM = 1.0                     # height drop that means "thread start found"
PEAK_ARM_DEG = 10.0               # the drop counts once the wrist has turned this far
SEARCH_LIMIT = YAW0 + 0.35        # ctl yaw at which the reverse stroke ends
                                  # (180 deg of reverse turn)
T_SEARCH_HOLD = 0.6               # s at the window end, wrist still, touch
                                  # servo on: a drop already under way (the
                                  # click AT the window edge, measured in the
                                  # landing-phase-175 run: seat force gone
                                  # 0.4 s before the limit, 0.7 of ~2 mm
                                  # fallen) completes before the decision
# fault guard: a nut this far off the bolt axis / this tilted is not on the
# thread any more -- open, retreat, stop (the verdict then fails honestly)
FAULT_XY = 0.0020
FAULT_TILT_DEG = 8.0

# ── forward strokes ─────────────────────────────────────────────────────
N_STROKES_MAX = 9                 # budget; the strokes stop at engagement
                                  # (5 in the default scene; the worst landing
                                  # class, phase 115 with the search having
                                  # lifted the nut 1.17 mm, needed 7)
ENGAGE_MARGIN = 0.0003            # stop once 0.3 mm past full engagement
FLOOR_DROP = 0.0090               # TCP may follow the nut <= 9 mm down

ORDER = ["settle", "descend", "pre", "close", "hold", "lift", "carry", "align",
         "lower", "land", "search", "search_hold", "click", "thread",
         "settle_flank", "release", "retreat", "still"]


def smooth5(u):
    u = float(np.clip(u, 0.0, 1.0))
    return u ** 3 * (u * (6.0 * u - 15.0) + 10.0)


def flat_pinch_yaw(nut_yaw: float, near: float) -> float:
    """Wrist yaw (down_grasp_quat argument) whose closing axis is a flat
    normal of a nut at `nut_yaw`, nearest to `near`.  down_grasp_quat(Y)
    closes along Y + 90 deg; the flat normals repeat every 60 deg."""
    f = SC.NUT_FLAT_PHASE + nut_yaw
    base = f - np.pi / 2
    k = np.round((near - base) / (np.pi / 3))
    return float(base + k * np.pi / 3)


def tilt_deg(q) -> float:
    R = world_T(q)[:3, :3]
    return float(np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0))))


class Expert:
    """Phase machine for pick -> carry -> align -> land -> search -> thread
    -> release.  Everything except the threading strokes is commanded here;
    the strokes are nut_bolt_arm's RegraspDriver, instantiated at the
    measured state once the thread start is found."""

    def __init__(self, rig, y_pick, y_land, nut_xy0):
        self.rig = rig
        self.ni = rig.builder.body_label.index("nut")       # = model index
        self.wi = rig.builder.body_label.index("fr3/fr3_link7")
        self.y_pick, self.y_land = float(y_pick), float(y_land)
        self.grasp = np.array([nut_xy0[0], nut_xy0[1], TCP_PICK_Z])
        self.hover = self.grasp + np.array([0.0, 0.0, HOVER])
        self.p = self.hover.copy()
        self.yaw = self.y_pick
        self.f = FINGER_OPEN_PICK
        self.relax = 0.0
        self.phase = "settle"
        self.t_phase = 0.0
        self.t_start = {"settle": 0.0}
        self.ev: dict = {}
        self.drv = None
        self.t_thr0 = None
        self.done = False
        self.stop_after = os.environ.get("NT_STOP_AFTER", "")
        self.nut_yaw = YawTrack()
        self._last_seg = None
        self.search = {}
        self.T_off = None                      # link7 -> TCP (set after finalize)

    # ── helpers ──────────────────────────────────────────────────────────
    def last(self, name, key="force"):
        h = self.rig.probes[name].history
        return h[-1][key] if h else 0.0

    def seat_F(self, n=3):
        h = self.rig.probes["seat"].history
        return float(np.mean([r["force"] for r in h[-n:]])) if h else 0.0

    def tcp_meas(self, bq):
        return (world_T(bq[self.wi]) @ self.T_off)[:3, 3]

    def goto(self, name, t):
        if self.stop_after and self.phase == self.stop_after:
            print(f"[{NAME}] NT_STOP_AFTER={self.stop_after}: stopping at t={t:.2f}s")
            raise StopRun
        self.phase, self.t_phase = name, t
        self.t_start[name] = t
        print(f"[{NAME}] t={t:6.2f}s -> {name}")

    def admittance(self, active=True):
        if not active:
            self.relax -= float(np.clip(self.relax, -RELAX_RATE, RELAX_RATE))
            return
        F = max(self.last("finL"), self.last("finR"))
        if F > F_GRIP_MAX:
            self.relax = min(self.relax + RELAX_RATE, RELAX_OPEN_MAX)
        elif F < F_GRIP_OK:
            self.relax = max(self.relax - RELAX_RATE, -RELAX_CLOSE_MAX)

    def xy_servo(self, nut_xy):
        err = SC.BOLT_AXIS - nut_xy
        self.p[:2] += np.clip(XY_GAIN * err, -XY_RATE, XY_RATE)

    def on_axis(self, nut) -> bool:
        return bool(np.linalg.norm(nut[:2] - SC.BOLT_AXIS) < FAULT_XY
                    and tilt_deg(nut) < FAULT_TILT_DEG)

    def _fault(self, t, nut, why):
        """Abort: release where the hand is, retreat, stop."""
        self.ev["fault"] = {"t": round(t, 3), "phase": self.phase, "why": why,
                            "nut_xyz_mm": (np.asarray(nut[:3]) * 1e3).round(2).tolist(),
                            "tilt_deg": round(tilt_deg(nut), 2)}
        print(f"[{NAME}] FAULT at t={t:.2f}s in {self.phase}: {why}")
        if self.phase == "thread":
            ctl = self.drv.ctl
            self.p = np.array([self.drv.tcp0[0] + ctl["xy"][0], self.drv.tcp0[1] + ctl["xy"][1],
                               ctl["z"]])
            self.yaw = ctl["yaw"] + YAW_BIAS
            self.f = ctl["fingers"]
            self.relax = self.drv._relax
        self.ev["rel_f0"], self.ev["rel_relax0"] = self.f, self.relax
        self.goto("release", t)

    # ── the phase machine ────────────────────────────────────────────────
    def control(self, t):
        rig = self.rig
        bq = rig.s0.body_q.numpy()
        nut = bq[self.ni]
        nz = float(nut[2])
        nyaw = self.nut_yaw.update(nut)
        if self.phase in ("land", "search", "search_hold", "click", "thread") \
                and not self.on_axis(nut):
            self._fault(t, nut, "nut off the bolt axis or tilted")
        s = t - self.t_phase
        k = self.phase
        if k == "settle":
            if s >= T_SETTLE:
                self.goto("descend", t)
        elif k == "descend":
            self.p = self.hover + (self.grasp - self.hover) * smooth5(s / T_DESC)
            if s >= T_DESC:
                self.goto("pre", t)
        elif k == "pre":
            # droop compensation: put the MEASURED pinch centre on the nut
            err = nut[:2] - self.tcp_meas(bq)[:2]
            self.p[:2] += np.clip(XY_GAIN * err, -XY_RATE, XY_RATE)
            if s >= T_PRE:
                self.ev["pick_xy_err_mm"] = float(np.linalg.norm(err)) * 1e3
                self.ev["pick_droop_comp_mm"] = (
                    (self.p[:2] - self.grasp[:2]) * 1e3).round(3).tolist()
                self.goto("close", t)
        elif k == "close":
            a = smooth5(s / (0.85 * T_CLOSE))
            self.f = FINGER_OPEN_PICK + (FINGER_GRIP - FINGER_OPEN_PICK) * a
            self.admittance(max(self.last("finL"), self.last("finR")) > F_TOUCH_PAD)
            if s >= T_CLOSE:
                self.goto("hold", t)
        elif k == "hold":
            self.f = FINGER_GRIP
            self.admittance()
            if s >= T_HOLD:
                self.ev["z_lift0"] = self.p[2]
                self.goto("lift", t)
        elif k == "lift":
            self.admittance()
            self.p[2] = self.ev["z_lift0"] + (CARRY_Z - self.ev["z_lift0"]) * smooth5(s / T_LIFT)
            if s >= T_LIFT:
                tm = self.tcp_meas(bq)
                d = nut[:2] - tm[:2]
                dy = self.y_land - self.y_pick
                c, sn = np.cos(dy), np.sin(dy)
                self.ev["nut_in_hand_xy"] = d.copy()
                cy, sy = np.cos(-self.yaw), np.sin(-self.yaw)
                self.ev["nut_in_hand_mm"] = [round(float(cy * d[0] - sy * d[1]) * 1e3, 3),
                                             round(float(sy * d[0] + cy * d[1]) * 1e3, 3)]
                self.ev["carry_from"] = self.p.copy()
                self.ev["carry_to"] = np.array([
                    SC.BOLT_AXIS[0] - (c * d[0] - sn * d[1]),
                    SC.BOLT_AXIS[1] - (sn * d[0] + c * d[1]), CARRY_Z])
                self.goto("carry", t)
        elif k == "carry":
            self.admittance()
            a = smooth5(s / T_CARRY)
            self.p = self.ev["carry_from"] + (self.ev["carry_to"] - self.ev["carry_from"]) * a
            self.yaw = self.y_pick + (self.y_land - self.y_pick) * a
            if s >= T_CARRY:
                self.goto("align", t)
        elif k == "align":
            self.admittance()
            self.xy_servo(nut[:2])
            if s >= T_ALIGN:
                self.ev["lower_from"] = self.p[2]
                self.ev["lower_to"] = self.p[2] + (Z_PRELAND - nz)
                self.goto("lower", t)
        elif k == "lower":
            self.admittance()
            self.xy_servo(nut[:2])
            self.p[2] = self.ev["lower_from"] + (self.ev["lower_to"] - self.ev["lower_from"]) \
                * smooth5(s / T_LOWER)
            if s >= T_LOWER:
                self.ev["land_z0"] = nz
                self.ev["droop_z"] = float(self.p[2] - self.tcp_meas(bq)[2])
                self.goto("land", t)
        elif k == "land":
            self.admittance()
            self.xy_servo(nut[:2])
            F = self.seat_F(2)
            if F >= F_TOUCH:
                self.ev["touch_t"] = t
                self.ev["touch_nut_z"] = nz
                self.ev["touch_xy_err_mm"] = float(np.linalg.norm(nut[:2] - SC.BOLT_AXIS)) * 1e3
                self.ev["touch_tilt_deg"] = tilt_deg(nut)
                self.ev["touch_nut_yaw_deg"] = float(np.degrees(np.arctan2(
                    world_T(nut)[1, 0], world_T(nut)[0, 0])))
                self.search = {"peak": nz, "yaw0": self.yaw, "nyaw0": nyaw,
                               "yaw_ref": self.yaw, "found": False}
                self.goto("search", t)
            elif self.ev["land_z0"] - nz > LAND_MAX_DROP:
                self._fault(t, nut, "no touch-down within 6 mm of descent")
            else:
                self.p[2] -= V_LAND / rig.fps
        elif k == "search":
            self.admittance()
            self._seat_servo(bq)
            self.xy_servo(nut[:2])
            F = self.seat_F(2)
            ctl = self.yaw - YAW_BIAS
            if F < F_PANIC and ctl < SEARCH_LIMIT:
                self.yaw += OMEGA_SEARCH / rig.fps
            sr = self.search
            if self.yaw - sr["yaw_ref"] < np.radians(PEAK_ARM_DEG):
                sr["peak"] = nz          # first touch may still settle on a chamfer
            sr["peak"] = max(sr["peak"], nz)
            if sr["peak"] - nz >= DROP_MM * 1e-3:
                self._click(t, nz, nyaw)
            elif ctl >= SEARCH_LIMIT:
                self.goto("search_hold", t)
        elif k == "search_hold":
            self.admittance()
            self._seat_servo(bq)
            self.xy_servo(nut[:2])
            sr = self.search
            if sr["peak"] - nz >= DROP_MM * 1e-3:
                self._click(t, nz, nyaw)
            elif s >= T_SEARCH_HOLD:
                # No click within one reverse stroke: the nut was already
                # engaged at touch-down (a landing in the 0-175 deg part of
                # the thread phase, with >= ~155 deg of flank arc), and the
                # reverse stroke has only unscrewed it by half a turn.  The
                # pads do NOT open here: released on a short arc the nut tips
                # off the bolt (measured: 17 deg the moment the pads opened,
                # then dragged off by the returning wrist).  Turning forward
                # from here, still pinched, re-threads it -- the first stroke
                # (up to 206 deg) restores more arc than the search took.
                self.ev["search_no_click"] = True
                self.ev["reverse_turn_deg"] = float(np.degrees(nyaw - sr["nyaw0"]))
                self.ev["search_peak_rise_mm"] = (sr["peak"] - self.ev["touch_nut_z"]) * 1e3
                self._start_thread(t, bq)
                self.goto("thread", t)
        elif k == "click":
            # let the nut finish its drop onto the next flank (the touch
            # servo brings it down) before the forward strokes take over
            self.admittance()
            self._seat_servo(bq)
            self.xy_servo(nut[:2])
            if (s >= T_CLICK_HOLD and self.seat_F(3) >= F_LO) or s >= T_CLICK_MAX:
                self.ev["click_landed_z_mm"] = nz * 1e3
                self._start_thread(t, bq)
                self.goto("thread", t)
        elif k == "thread":
            self._thread(t, bq)
            return
        elif k == "settle_flank":
            self.admittance()
            F = self.seat_F(3)
            if F >= F_SETTLE or self.ev["settle_z0"] - self.p[2] >= SETTLE_MAX:
                self.ev.setdefault("settle_t", t)
                self.ev.setdefault("settle_lowered_mm", (self.ev["settle_z0"] - self.p[2]) * 1e3)
                self.ev.setdefault("settle_seat_N", F)
                if t - self.ev["settle_t"] >= T_SETTLE_HOLD:
                    self.ev["rel_f0"], self.ev["rel_relax0"] = self.f, self.relax
                    self.goto("release", t)
            else:
                self.p[2] -= V_SETTLE / rig.fps
        elif k == "release":
            a = smooth5(s / (0.85 * T_RELEASE))
            self.f = self.ev["rel_f0"] + (FINGER_OPEN - self.ev["rel_f0"]) * a
            self.relax = self.ev["rel_relax0"] * (1.0 - a)
            if s >= T_RELEASE:
                self.ev["retreat_z0"] = self.p[2]
                self.goto("retreat", t)
        elif k == "retreat":
            self.p[2] = self.ev["retreat_z0"] + RETREAT_Z * smooth5(s / T_RETREAT)
            if s >= T_RETREAT:
                self.goto("still", t)
        elif k == "still":
            if s >= T_STILL:
                self.done = True
                raise StopRun
        if k not in ("thread",):
            rig.drive_arm(self.p, down_grasp_quat(self.yaw),
                          fingers=float(np.clip(self.f + self.relax, 0.0, 0.04)))
        if k == "lift" and "tcp_off" not in self.ev and s >= T_LIFT - 2.0 / rig.fps:
            self.ev["tcp_off"] = float(self.p[2] - nz)

    def _click(self, t, nz, nyaw):
        sr = self.search
        sr["found"] = True
        self.ev["click_t"] = t
        self.ev["click_drop_mm"] = (sr["peak"] - nz) * 1e3
        self.ev["click_peak_z_mm"] = sr["peak"] * 1e3
        self.ev["reverse_turn_deg"] = float(np.degrees(nyaw - sr["nyaw0"]))
        self.ev["reverse_wrist_deg"] = float(np.degrees(self.yaw - sr["yaw0"]))
        self.goto("click", t)

    def _seat_servo(self, bq):
        """Light downward touch on the thread: hold the nut<->bolt force in
        [F_LO, F_HI] by moving the TCP command, with the downward lead over
        the MEASURED TCP bounded (pitfall 6.2: a position lead is a force)."""
        F = self.seat_F(2)
        if F > F_HI:
            self.p[2] += min(V_UP, 0.00003 * (F - F_HI) / F_HI + 0.00001)
        elif F < 0.05:
            self.p[2] -= V_DOWN_FREE
        elif F < F_LO:
            self.p[2] -= V_DOWN
        tz = self.tcp_meas(bq)[2]
        self.p[2] = max(self.p[2], tz + self.ev.get("droop_z", 0.0) - LEAD_MAX)

    def _start_thread(self, t, bq):
        """Hand over to nut_bolt_arm's RegraspDriver at the measured state."""
        rig = self.rig
        nut = bq[self.ni]
        tm = self.tcp_meas(bq)
        plan, t_plan = build_plan([(-1.0, N_STROKES_MAX, False)], t_settle=0.0)
        self.plan, self.t_plan = plan, t_plan
        # the IK model's current solution is the last command: RegraspDriver
        # takes the TCP-in-link7 offset from it and tcp0
        drv = RegraspDriver(rig, plan, self.p.copy(), self.ni, self.wi,
                            part_z0=float(nut[2]), yaw_init=self.yaw - YAW_BIAS,
                            finger_grip=FINGER_GRIP, latch_mm=99.0, floor_drop=FLOOR_DROP,
                            f_seat=NA.F_SEAT, lead_m=SC.LEAD, axis_xy=SC.BOLT_AXIS.copy(),
                            yaw_bias=YAW_BIAS, f_grip_max=F_GRIP_MAX, f_grip_ok=F_GRIP_OK,
                            relax_open_max=RELAX_OPEN_MAX, relax_close_max=RELAX_CLOSE_MAX,
                            relax_rate=RELAX_RATE)
        drv.ctl["fingers"] = self.f
        drv._relax = self.relax
        self.drv = drv
        self.t_thr0 = t
        self.ev["thread_z0_mm"] = float(nut[2]) * 1e3
        self.ev["thread_axis_xy_mm"] = (tm[:2] * 1e3).round(3).tolist()
        print(f"[{NAME}] thread start: nut z {nut[2]*1e3:.2f} mm, plan "
              f"{[(s_['kind'], round(s_['t0'], 2)) for s_ in plan][:6]}... total {t_plan:.1f}s")

    def _thread(self, t, bq):
        drv = self.drv
        tl = t - self.t_thr0
        nz = float(bq[self.ni][2])
        seg = None
        for s_ in self.plan:
            if s_["t0"] <= tl < s_["t1"]:
                seg = s_
                break
        kind = seg["kind"] if seg else "end"
        prev = self._last_seg
        self._last_seg = kind
        engaged = nz <= SC.ENGAGED_Z - ENGAGE_MARGIN and self.on_axis(bq[self.ni])
        if (prev == "stroke" and kind != "stroke" and engaged) or kind == "end" \
                or tl >= self.t_plan:
            # take over at the stroke's end pose: hold the TCP, release
            ctl = drv.ctl
            self.p = np.array([drv.tcp0[0] + ctl["xy"][0] + ctl["aside"][0],
                               drv.tcp0[1] + ctl["xy"][1] + ctl["aside"][1], ctl["z"]])
            self.yaw = ctl["yaw"] + YAW_BIAS
            self.f = ctl["fingers"]
            self.relax = drv._relax
            self.ev["engaged_at_release"] = bool(engaged)
            self.ev["strokes_used"] = len(drv.strokes) + (1 if prev == "stroke" else 0)
            self.ev["settle_z0"] = float(self.p[2])
            # close the stroke bookkeeping of the last stroke
            drv.control(tl)
            self.goto("settle_flank", t)
            self.rig.drive_arm(self.p, down_grasp_quat(self.yaw),
                               fingers=float(np.clip(self.f + self.relax, 0.0, 0.04)))
            return
        drv.control(tl)


def main():
    nut_yaw0 = np.radians(NUT_YAW0_DEG)
    y_land = -YAW0 + YAW_BIAS                      # start of the stroke window
    y_pick = flat_pinch_yaw(nut_yaw0, y_land)
    rig = appendix_rig(NAME)
    base_xf = wp.transform(wp.vec3(BASE_XY[0], BASE_XY[1], 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = NA.FINGER_KE
        rig.builder.joint_target_kd[d] = NA.FINGER_KD
    sc = SC.build_scene(rig, nut_xy=SC.NUT_XY0, nut_yaw=nut_yaw0)

    ex = Expert(rig, y_pick, y_land, SC.NUT_XY0)
    q0 = rig.solve_arm_start(ex.hover, down_grasp_quat(y_pick))
    rig.builder.joint_q[7:9] = [FINGER_OPEN_PICK, FINGER_OPEN_PICK]
    rig.builder.joint_target_q[7:9] = [FINGER_OPEN_PICK, FINGER_OPEN_PICK]
    lo = rig.ik.model.joint_limit_lower.numpy()[:7]
    hi = rig.ik.model.joint_limit_upper.numpy()[:7]
    marg = np.degrees(np.minimum(q0[:7] - lo, hi - q0[:7]))
    print(f"nut yaw {NUT_YAW0_DEG:.1f} deg -> pick wrist yaw {np.degrees(y_pick):.1f}, land "
          f"{np.degrees(y_land):.1f}; start joint margin {marg.min():.1f} deg")
    T_off = np.linalg.inv(world_T(rig.ik.body_pose("fr3/fr3_link7"))) @ world_T(rig.ik.tcp_pose())
    ex.T_off = T_off

    # primary: the whole task (pick spot, carry height, bolt), from the +x/-y
    # side, 0.30 m; second view (multiview video): close-up of the thread,
    # roughly perpendicular to the pinch line at the start of the strokes
    cam_t = np.array([_env("NT_CAM_TX", -0.025), _env("NT_CAM_TY", 0.0),
                      _env("NT_CAM_TZ", 0.040)])
    cam_p = cam_t + np.array([_env("NT_CAM_OX", 0.0730), _env("NT_CAM_OY", -0.2723),
                              _env("NT_CAM_OZ", 0.1026)])
    cam2_t = np.array([0.0, 0.0, 0.040])
    cam2_p = cam2_t + np.array([-0.0618, -0.1071, 0.0402])
    if os.environ.get("NT_PREVIEW"):
        # camera contact sheet from saved states, no physics (HP helper)
        from force_bench.appendix_tasks import _hang_plate as HP
        rig.finalize(probes=[], cam_pos=cam_p, cam_target=cam_t, record=False)
        cams = HP.parse_cams(os.environ.get(
            "NT_PREVIEW_CAMS", f"{','.join(map(str, cam_t))}:{','.join(map(str, cam_p - cam_t))}"))
        HP.preview_cameras(rig, os.environ["NT_PREVIEW"],
                           [float(v) for v in os.environ.get("NT_PREVIEW_T", "1,3,5,8,12,20").split(",")],
                           cams, os.environ.get("NT_PREVIEW_OUT", "camera_preview.png"))
        return
    rig.finalize(probes=["nut"], cam_pos=cam_p, cam_target=cam_t,
                 cam2_pos=cam2_p, cam2_target=cam2_t)
    SC.recolor_kit(rig.model)
    rig.probes = {
        "seat": PairProbe(rig.model, "nut", counterpart_shapes=["bolt"]),
        "table": PairProbe(rig.model, "nut", counterpart_shapes=["table"]),
        "finL": ForceProbe(rig.model, sensing_body="fr3/fr3_leftfinger",
                           counterpart_bodies=["nut"]),
        "finR": ForceProbe(rig.model, sensing_body="fr3/fr3_rightfinger",
                           counterpart_bodies=["nut"]),
        "robot_object": RobotContactProbe(rig.model, ["nut"],
                                          static_shapes=["bolt", "block_top", "block_bottom"]),
    }
    rig.solver = make_solver(rig.model, iterations=20)
    ni, wi = ex.ni, ex.wi
    log = []

    def extra(state, t):
        out = {}
        if ex.phase == "thread" and ex.drv is not None and "_ny" in ex.drv.ctl:
            ex.drv.extra(state, t)        # its own bookkeeping only: the CSV
                                          # header is fixed by the first row
        bq = state.body_q.numpy()
        nut = bq[ni]
        tcp = (world_T(bq[wi]) @ T_off)[:3, 3]
        R = world_T(nut)[:3, :3]
        rec = dict(t=t, phase=ORDER.index(ex.phase) if ex.phase in ORDER else -1,
                   nx=float(nut[0]), ny=float(nut[1]), nz=float(nut[2]),
                   nyaw=ex.nut_yaw.total, tilt=float(np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))),
                   tx=float(tcp[0]), ty=float(tcp[1]), tz=float(tcp[2]),
                   fl=ex.last("finL"), fr=ex.last("finR"), seat=ex.last("seat"),
                   seat_z=ex.last("seat", "vec")[2] if rig.probes["seat"].history else 0.0,
                   table=ex.last("table"), robj=ex.last("robot_object"),
                   cmd_z=float(ex.drv.ctl["z"] if ex.phase == "thread" else ex.p[2]),
                   yaw_cmd=float(ex.drv.ctl["yaw"] + YAW_BIAS if ex.phase == "thread" else ex.yaw),
                   fing=float(rig.control.joint_target_q.numpy()[7]))
        log.append(rec)
        if int(round(t * rig.fps)) % 15 == 0:
            print(f"  [{t:6.2f} {ex.phase:14s}] nut ({nut[0]*1e3:7.2f},{nut[1]*1e3:6.2f},"
                  f"{nut[2]*1e3:6.2f}) yaw {np.degrees(ex.nut_yaw.total):7.1f} tilt {rec['tilt']:4.1f}"
                  f" | pads {rec['fl']:5.2f}/{rec['fr']:5.2f} seat {rec['seat']:5.2f} (z {rec['seat_z']:+.2f})"
                  f" table {rec['table']:5.3f} robj {rec['robj']:5.2f} | tcp z {tcp[2]*1e3:6.2f}"
                  f" cmd {rec['cmd_z']*1e3:6.2f} wrist {np.degrees(rec['yaw_cmd']):7.1f}")
        out.update({"nut_z_mm": round(rec["nz"] * 1e3, 3),
                    "robot_object_contact_N": round(rec["robj"], 4),
                    "table_N": round(rec["table"], 4)})
        return out

    try:
        rec = rig.run(T_BUDGET, ex.control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{NAME}] rollout ended at t={len(rec.times) / rig.fps:.2f}s (phase {ex.phase})")
    paths = rec.save("rollout", force_keys=["seat_N", "finL_N", "finR_N", "table_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    print("events:", {k: (round(v, 3) if isinstance(v, float) else v) for k, v in ex.ev.items()
                      if not isinstance(v, np.ndarray)})
    print("phase starts:", {k: round(v, 2) for k, v in ex.t_start.items()})
    if ex.stop_after:
        print("stopped early (NT_STOP_AFTER): QA skipped")
        return
    if not ex.done:
        print(f"[{NAME}] WARNING: the expert did not finish (phase {ex.phase}); scoring anyway")
    finish_qa(rig, ex, sc, paths, log)


def finish_qa(rig, ex, sc, paths, log):
    qa = rig.qa(paths)
    L = {k: np.array([r[k] for r in log]) for k in log[0]}
    t = L["t"]
    # an incomplete rollout is scored too (as a failure): phases it never
    # reached start at the end of the log
    ts = {k: ex.t_start.get(k, float(t[-1]) + 1.0 / rig.fps) for k in ORDER}
    for k in ("lower", "settle_flank", "release", "retreat", "still"):
        if k not in ex.t_start:
            ts[k] = float(t[-1])
    g = 9.81 * SC.NUT_MASS

    def win(a, b):
        return (t >= a) & (t < b)

    # ── picked up from the table ─────────────────────────────────────────
    w_rest = win(0.1, ts["descend"])
    w_carry = win(ts["lift"] + 0.3, ts["lower"])
    rise = float(L["nz"][win(ts["lift"], ts["lower"])].max() - sc["nut_p0"][2])
    picked = {
        "pass": bool(L["table"][w_rest].mean() > 0.5 * g and rise >= 0.030
                     and L["table"][w_carry].max() < 0.01
                     and min(L["fl"][w_carry].mean(), L["fr"][w_carry].mean()) > 1.0),
        "start_on_table_contact_N": round(float(L["table"][w_rest].mean()), 4),
        "nut_weight_N": round(g, 4),
        "start_tilt_deg": round(float(L["tilt"][w_rest].max()), 2),
        "max_rise_above_start_mm": round(rise * 1e3, 1),
        "table_contact_during_carry_max_N": round(float(L["table"][w_carry].max()), 4),
        "pad_mean_during_carry_N": [round(float(L["fl"][w_carry].mean()), 2),
                                    round(float(L["fr"][w_carry].mean()), 2)],
        "nut_in_hand_xy_offset_mm": np.round(np.asarray(ex.ev.get("nut_in_hand_xy", [0, 0])) * 1e3,
                                             3).tolist(),
    }

    # ── real finger grasp through the strokes ────────────────────────────
    drv = ex.drv
    tight = [s for s in (drv.strokes if drv else []) if s["dirn"] < 0]
    strokes_m = np.zeros_like(t, bool)
    if drv is not None:
        for s in ex.plan:
            if s["kind"] == "stroke":
                strokes_m |= win(ex.t_thr0 + s["t0"] + 0.1,
                                 min(ex.t_thr0 + s["t1"], ts["settle_flank"]))
    grip = {
        "pass": bool(strokes_m.any() and L["fl"][strokes_m].mean() > 1.0
                     and L["fr"][strokes_m].mean() > 1.0
                     and min(L["fl"][w_carry].mean(), L["fr"][w_carry].mean()) > 1.0),
        "mean_pad_N_in_strokes": [round(float(L["fl"][strokes_m].mean()), 2) if strokes_m.any() else 0,
                                  round(float(L["fr"][strokes_m].mean()), 2) if strokes_m.any() else 0],
        "note": "the arm links are collision-filtered against the nut, the fingertips are not: "
                "every force on the nut other than bolt/table contact is fingertip contact; "
                "no wrist->nut joint, spring or kinematic attachment exists",
    }

    # ── thread start (how the expert started the thread; info) ───────────
    # Not a pass/fail gate of its own: whether the reverse stroke meets the
    # click depends on the thread phase the nut happens to land in (a nut
    # landing in the 0-175 deg part of the phase is already engaged and the
    # click lies beyond the 180 deg reverse window).  What has to be TRUE --
    # the nut really threaded, coaxial and upright, at the mesh lead -- is
    # gated in threads_fully_engaged.
    start = {
        "pass": True, "info_only": True,
        "outcome": ("click: thread start found by the reverse turn"
                    if ex.ev.get("click_drop_mm", 0.0) >= DROP_MM else
                    "no click within the 180 deg reverse stroke: already engaged at "
                    "touch-down; forward strokes started from the reversed pose"),
        "touch_down_nut_centre_mm": round(ex.ev.get("touch_nut_z", 0.0) * 1e3, 2),
        "touch_down_xy_from_axis_mm": round(ex.ev.get("touch_xy_err_mm", 0.0), 3),
        "touch_down_tilt_deg": round(ex.ev.get("touch_tilt_deg", 0.0), 2),
        "touch_down_nut_yaw_deg": round(ex.ev.get("touch_nut_yaw_deg", 0.0), 1),
        "reverse_turn_deg": round(ex.ev.get("reverse_turn_deg", 0.0), 1),
        "click_drop_mm": round(ex.ev.get("click_drop_mm", 0.0), 2),
        "note": "reverse (unscrewing) turn under a 0.25-0.9 N force-servoed touch; a measured "
                "height drop >= 1 mm is the nut's thread start passing the bolt's",
    }

    # ── threads fully engaged ────────────────────────────────────────────
    w_end = win(ts["still"], t[-1] + 1)
    z_end = float(L["nz"][w_end].mean())
    xy_end = float(np.hypot(L["nx"][w_end].mean(), L["ny"][w_end].mean()))
    tilt_end = float(L["tilt"][w_end].max())
    # rotation vs descent from the thread start (nut pressed onto its lower
    # flank by the search touch) to the release (lowered onto the lower flank
    # by settle_flank): both ends at the same side of the 0.6 mm backlash
    w_thr = win(ex.t_thr0 if ex.t_thr0 is not None else t[-1] + 1, ts["release"])
    turns = (L["nyaw"][w_thr][-1] - L["nyaw"][w_thr][0]) / (2 * np.pi) if w_thr.any() else 0.0
    dz = (L["nz"][w_thr][-1] - L["nz"][w_thr][0]) if w_thr.any() else 0.0
    pitch = float(dz / turns) * 1e3 if abs(turns) > 0.2 else 0.0
    eng = {
        "pass": bool("fault" not in ex.ev and z_end <= SC.ENGAGED_Z and xy_end < 0.0005
                     and tilt_end < 3.0
                     and abs(pitch - SC.LEAD * 1e3) < 0.10 * SC.LEAD * 1e3),
        "expert_fault": ex.ev.get("fault"),
        "final_nut_top_face_mm": round((z_end + SC.NUT_TOP_ABOVE_C) * 1e3, 2),
        "full_crest_thread_top_mm": round(SC.THREAD_TOP * 1e3, 2),
        "final_nut_centre_mm": round(z_end * 1e3, 2),
        "engaged_when_centre_below_mm": round(SC.ENGAGED_Z * 1e3, 2),
        "engaged_length_mm": round((min(z_end + SC.NUT_TOP_ABOVE_C, SC.THREAD_TOP)
                                    - (z_end - (SC.NUT_H - SC.NUT_TOP_ABOVE_C))) * 1e3, 2),
        "final_axis_offset_mm": round(xy_end * 1e3, 3),
        "final_tilt_deg": round(tilt_end, 2),
        "forward_turns": round(float(turns), 3),
        "forward_descent_mm": round(float(dz) * 1e3, 2),
        "measured_pitch_mm_per_turn": round(pitch, 3),
        "mesh_lead_mm_per_turn": round(SC.LEAD * 1e3, 3),
        "strokes": [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items()}
                    for s in tight],
    }

    # ── stays put after release ──────────────────────────────────────────
    w_rel0 = win(ts["release"] - 0.2, ts["release"])
    z_rel = float(np.median(L["nz"][w_rel0]))
    yaw_rel = float(np.median(L["nyaw"][w_rel0]))
    w_still = win(ts["still"], t[-1] + 1)
    p_still = np.c_[L["nx"][w_still], L["ny"][w_still], L["nz"][w_still]]
    still_move = float(np.linalg.norm(p_still - p_still[0], axis=1).max()) if w_still.any() else 9.9
    stays = {
        "pass": bool(abs(z_end - z_rel) < 0.0003 and abs(np.degrees(L["nyaw"][-1] - yaw_rel)) < 5.0
                     and still_move < 0.00005),
        "z_before_release_mm": round(z_rel * 1e3, 3), "final_z_mm": round(z_end * 1e3, 3),
        "drop_at_release_mm": round((z_rel - z_end) * 1e3, 3),
        "yaw_change_after_release_deg": round(float(np.degrees(L["nyaw"][-1] - yaw_rel)), 2),
        "nut_motion_in_final_still_window_mm": round(still_move * 1e3, 4),
        "pre_release_flank_settle": {k: (round(ex.ev[k], 3) if isinstance(ex.ev.get(k), float) else ex.ev.get(k))
                                     for k in ("settle_lowered_mm", "settle_seat_N")},
    }

    # "moving away from, and breaking contact with the task objects": zero
    # robot<->{nut, bolt, block} contact over the final still window, and the
    # TCP has risen >= 50 mm from where the pads opened
    check_released_and_clear(qa, t, L["robj"], ts["still"])
    rel = qa.report["checks"]["released_and_clear"]
    w_open = win(ts["release"], ts["retreat"])
    rise = float(L["tz"][-1] - L["tz"][w_open][-1]) if w_open.any() else 0.0
    rel["tcp_moved_away_mm"] = round(rise * 1e3, 1)
    rel["min_move_mm"] = 50.0
    rel["pass"] = bool(rel["pass"] and rise >= 0.050)

    w_last = win(t[-1] - 0.5, t[-1] + 1)
    tcp_path = np.c_[L["tx"][w_last], L["ty"][w_last], L["tz"][w_last]]
    move = float(np.linalg.norm(tcp_path - tcp_path[0], axis=1).max())
    stopped = {"pass": bool(move < 0.0005), "tcp_motion_last_0.5s_mm": round(move * 1e3, 3)}

    crit = {"picked_up_from_table": picked, "real_finger_grasp": grip,
            "thread_start_search": start, "threads_fully_engaged": eng,
            "nut_stays_put_after_release": stays, "released_and_clear": rel,
            "robot_stopped": stopped}
    for k_, c_ in crit.items():
        qa.report["checks"][k_] = c_

    # ── physics integrity ────────────────────────────────────────────────
    qa.check_penetration_static("nut", sc["nut_local"], sc["bolt_tm"], sc["bolt_c"],
                                "nut->bolt", tol=0.0015, n_samples=60)
    # ...and densely where it matters: EVERY bore/thread vertex of the nut
    # (r < 6 mm) at the 16 frames with the largest nut<->bolt force (the
    # 800-vertex random subsample above can miss the few touching vertices)
    Vb = sc["nut_local"].vertices
    Vb = Vb[np.hypot(Vb[:, 0], Vb[:, 1]) < 0.0060]
    pq = trimesh.proximity.ProximityQuery(sc["bolt_tm"])
    seat_hist = np.array([r["force"] for r in rig.probes["seat"].history])
    seat_t = np.array([r["t"] for r in rig.probes["seat"].history])
    worst, worst_t = 0.0, None
    ni_ = rig.body_index("nut")
    for i in np.argsort(seat_hist)[-16:]:
        f = int(np.argmin(np.abs(qa.t - seat_t[i])))
        T = world_T(qa.body_q[f, ni_])
        dmax = float(pq.signed_distance((T[:3, :3] @ Vb.T).T + T[:3, 3]).max())
        if dmax > worst:
            worst, worst_t = dmax, float(qa.t[f])
    pen0 = qa.report["checks"]["penetration"]
    pen0["pairs"]["nut->bolt_dense_at_peak_force"] = {
        "max_depth_mm": round(worst * 1e3, 3), "at_t": worst_t, "bore_vertices": int(len(Vb)),
        "frames": 16, "pass": bool(worst < 0.0015)}
    pen0["pass"] = bool(pen0["pass"] and worst < 0.0015)
    low = []
    ni = rig.body_index("nut")
    v = sc["nut_local"].vertices[::7]
    for fi in np.linspace(0, len(qa.t) - 1, 120).astype(int):
        T = world_T(qa.body_q[fi, ni])
        low.append(float(((T[:3, :3] @ v.T).T + T[:3, 3])[:, 2].min()))
    zmin = min(low)
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["nut->table_top"] = {"min_vertex_z_mm": round(zmin * 1e3, 3),
                                     "pass": bool(zmin > -0.0010)}
    pen["pass"] = bool(pen["pass"] and zmin > -0.0010)
    qa.check_envelope(["nut"], [-0.12, -0.08, -0.002], [0.08, 0.08, 0.14])
    qa.report["checks"]["expert_events"] = {
        "pass": True, "info_only": True,
        "events": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in ex.ev.items()
                   if not isinstance(v, np.ndarray)},
        "phase_start_s": {k: round(v, 2) for k, v in ts.items()}}
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    save_appendix_verdict(
        rig, task="Screw nut onto screw (picked up from the table)",
        source="new expert on the finalized-kit bolt block; threading strokes = "
               "force_bench.tasks.nut_bolt_arm.RegraspDriver (as in screw_nut_onto_screw)",
        spec="Screw the nut held in hand onto the screw head placed in the workspace such that "
             "their threads fully engage. The robot movement stops after terminating the "
             "rotation motion, moving away from, and breaking contact with the task objects.",
        criteria=crit, physics_qa=report,
        changes=[
            "starts with the nut lying flat on the table beside the bolt block (appendix: in "
            "hand) -- picked up by a top-down pinch on two flats (user request)",
            "finalized-kit bolt block (assets/bodies_kit/bolt_block): the bolt protrudes "
            "23.45 mm above the yellow block; same thread as the suite fasteners",
            "thread start found by feel: reverse turn under a light force-servoed touch "
            "until the measured nut height drops (the 'click'), then forward strokes",
            "strokes stop once the whole nut is on full-crest thread; open, rise 70 mm, still",
            "nut mass 10 g (the suite's value for this fastener; a PLA print is 1.2 g)",
        ])


if __name__ == "__main__":
    wp.init()
    main()
