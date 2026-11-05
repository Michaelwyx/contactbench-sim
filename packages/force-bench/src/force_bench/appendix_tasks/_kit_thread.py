"""Screw a threaded kit part home with the FR3: "Install light bulb" (task 31) and "Put bottle cap onto
bottle" (task 19) on the finalized printable kit.  Scene, scripted expert and verdict shared by

  install_light_bulb             lightbulb-1 (55 mm, 2 revolutions) held in the pinch above lightbulb-slot
  install_light_bulb_from_table  the same bulb upright on a printed stand; picked up, carried, screwed in
  install_light_bulb_4rev        lightbulb-1 55 mm, 4 revolutions ("more rotation")
  install_light_bulb_2_50mm      lightbulb-2 (pear shape), 50 mm, 2 revolutions
  install_light_bulb_3_70mm      lightbulb-3 (reflector), 70 mm, 2 revolutions
  put_bottle_cap_onto_bottle     bottle-1 cap held in the pinch above its bottle, 2 revolutions (+ variants)

THE PARTS are the kit's published STLs (``assets/kit/<asset>/``, ``force_bench.kit``), nothing generated:
the bulb or the cap is ONE free rigid body, the socket or the bottle is static.  Each pair is exported in
its CAD frame screwed together; measured on every variant (``measure_pair``):

* the export is 0.577 mm PAST the seat (bulb tip 0.577 mm into the slot floor at z -2.423; bottle lip
  0.573-0.577 mm into the cap ceiling at z 10.48), so the seated pose is that far back up the helix;
* one thread form for all of them: a single-start right-hand thread, 1.2 mm deep, 0.30 mm radial
  clearance (bulb root/crest r 12.49 / 13.70 against slot crest/root 12.80 / 14.00; bottle-1 neck
  15.0 / 16.2 against cap 15.3 / 16.5, bottle-2 and -3 the same +5 / +15 mm), ~9.4 mm engaged when
  seated; the revolution count sets the lead: 4.665 / 3.75 / 3.13 / 2.68 / 2.34 mm per turn for
  2 / 2.5 / 3 / 3.5 / 4 revolutions (helical self-consistency fit, ``thread_lead``; bottle-1's
  "3 revolutions" files carry 3.73 mm, the 2.5-revolution lead -- a kit export error);
* no spring tab: the bulb tightens with its flat tip face (r 8.7 mm) on the slot floor, the cap with
  its ceiling on the bottle's lip (an annulus r 13.8-15.0 mm on bottle-1);
* the ridges are narrow (1.05 mm of a 4.665 mm lead), so the 2-revolution pairs have 2.9 mm of AXIAL
  PLAY (224 deg of free turning at a fixed height; 0.6 mm / 92 deg at 4 revolutions).  Two things
  follow, both seen in the rollouts: lowered at an arbitrary phase the part can drop up to ~3 mm onto
  the thread before it bites (at the default start phase it touches 4 mm lower than the thread start),
  and once the seat face touches the part still turns freely through the play (the "take-up") until
  the opposite flanks meet -- then the joint locks within a degree (the seat is stiff: tens of N of
  axial preload per degree).

PHYSICS MODEL (nothing positions the part but the fingertips and the fixture):

* The thread is real geometry: Newton SDF mesh-mesh contact consumed by SolverMuJoCo.  Each part is
  split for collision only: the threaded end gets a fine SDF (~0.08 mm cells against the 0.3 mm
  clearance; sim.yaml's 0.2-0.3 mm cells are for the settle check), the rest a coarse one (the pads and
  the stand only meet it).  The visible shape is the unsplit kit part (print colour, label
  ``kit/<label>/<part>``) and carries the mass distribution; the printed mass is the kit estimate
  (walls + 20 % infill).  PLA/PLA mu from sim.yaml (0.5).
* "TIGHTENED" WITHOUT A TAB is defined from the contact state, measured per contact
  (``ContactTap``): the seat face is in contact (axial seat force, separated from the thread flanks by
  where each contact sits), the thread flanks pull the part onto it (the PRELOAD, -flank z force,
  >= PRELOAD_TIGHT), the part has STALLED (< 1 deg in T_TIGHT_CONFIRM with the wrist held) and the
  twisting torque the pads deliver (moment of the finger contact forces about the part's axis) has
  reached TAU_TIGHT.  Twisting past seat contact preloads the joint: the seat pushes the part up, the
  thread flanks pull it down, and that preload is what the holding probe finds after release (a
  self-locking thread: mu 0.5 >> tan(lead angle) 0.06).  The verdict re-measures all of it.

EXPERT (privileged state):
  align      TCP xy servo putting the part's thread centre on the fixture axis (measured pose).
  descend    fast, then slow; stops on the first thread contact (pair force > F_TOUCH).
  engage     clockwise strokes with a light force-servoed press until the part descends at >= 0.75 x
             the lead per turn of its own measured rotation (it rides the run-in until the thread
             starts meet); the hand regrasps while the run-in holds the part.
  run-down   regrasp strokes: open -> twist back with the pads open -> close -> clockwise twist of up to
             160 deg, lag-limited on the measured part yaw; TCP z = the helix predicted from the measured
             part rotation, trimmed on the measured axial fixture force (light press); xy on the grip
             point (a symmetric pinch).
  tighten    once the seat face touches, the TCP holds the height of that touch (re-anchored at each
             regrasp) with only the light-press trim -- following the measured part instead lifts it into
             the thread play, and a hand that pushes makes the pads slide vertically, which in MuJoCo's
             cone leaves almost no friction about the axis (measured: 1.5-10 mN.m of pad torque while the
             wrist slipped 20 deg past a stalled cap).  The take-up runs at OMEGA_TAKEUP; when the pad
             torque crosses TAU_TIGHT WITH the flanks pulling the part down >= PRELOAD_TIGHT (they have
             locked; the touch alone spikes the torque and, through the hand, the seat force) the wrist
             stops, and the part is tight if it then turns < 1 deg in T_TIGHT_CONFIRM with the preload still
             there (the wound-up pad torque may relax meanwhile: friction creeps in MuJoCo).  A seat touch
             during descend/engage counts as engagement (the play can drop the part onto its seat).
             Seated, the hand presses F_AX_PRESS_SEAT (3 N) and the pinch runs at 8-12 N: a bulb held by
             two point contacts at its equator, 78 mm above the thread, otherwise pivots about the pinch line
             under the one-sided flank force of the lock (measured 0.27 deg, 30 um lifted), locks tilted,
             and loses the whole preload when it settles upright after release; so, seated, the xy servo
             also stands the part upright (grip point over the thread centre: < 0.05 deg at the lock).
  release    hold, open slowly; CHECK with the pads open at the part that the flanks still pull it onto its
             seat (>= PRELOAD_MIN); if not, re-grip and tighten again (at most MAX_RETIGHTEN times); then rise
             RETREAT, hold still, and the PROBE: PROBE_TAU on the part about its axis, loosen-way then
             tighten-way, 1 s each.

GRASPS (the FR3 palm is a flat plate 37.45 mm above the TCP across the whole hand, so a top-down
pinch must keep the part's top below it; pads span TCP -9.45 .. +9.05 mm, 17.5 mm wide, max pad gap
79.6 mm):
  lightbulb-1   TCP at the globe's equator (r 27.5 mm, 27.5 mm below the top: 10 mm palm clearance).
                The 15 mm neck band is out of reach: the 75 mm of globe above it would have to fit
                under the palm.
  lightbulb-2   the equator (z 47) is 60 mm below the top, so the palm limits the TCP to >= 3 mm palm
                clearance; the pads' lower edges bear on the upper globe (50 mm bulb: r 23.6 mm at z 63.5).
  lightbulb-3   the reflector rim (r 34.65 mm at z 73, the widest point); 5.2 mm of opening left per
                side.  The 80 mm reflector (r 39.49) leaves 0.32 mm per side with the fingers fully open
                (max pad half-gap 39.81 mm) and the 90 mm one (r 44.35) is 9 mm too wide for the 79.6 mm
                opening: not graspable top-down (measure_pair rejects both).
  caps          the cylindrical side at mid-height (a 13 mm line contact); the pads reach 3 mm below
                the skirt and clear the bottle's shoulder (it reaches the cap radius 5.8 mm lower).

SEEDED STARTS (in-hand rows, ``--seed N``; ``force-bench run install_light_bulb --seed N``): the fixture's axis (the
socket, or the bottle) is drawn uniformly in the paper's 0.15 x 0.15 m square at the board centre (``AXIS_PLACE``,
``common.sample_start``); no yaw: the socket and the bottles are round, and the part's thread phase has its own
option (``--yaw0-deg``).  The part starts in the pinch above the fixture wherever it lands (the same 4 mm above its
top); the policy aligns on the measured part and the fixture axis.  The robot base stays where it is (0.40 m from
the board centre): over the square the arm reaches every pose of the wrist window (+-80 deg) at the start, seated
and retreat heights, with the elbow closest to its limit at the robot-side edge (>= 0.09 rad).  The camera keeps its
view of the fixture.  The draw is recorded in the verdict ("start").  The from-table row takes no seed.

RESULTS (2026-10-28, default render quality; turns = first thread contact -> tight, lead fitted on the
engaged run-down, peak = pad torque at the lock incl. its overshoot, preload = flank pull after release,
probe rotation loosen-way / tighten-way):
  row                               verdict turns     lead (CAD)  take-up      peak  preload    probe (deg)   wall
  install_light_bulb                PASS     1.95  4.778 (4.665)   232 deg   357 mNm  10.1 N  0.012/0.001 16 min
  install_light_bulb_from_table     PASS     1.95  4.887 (4.665)   231 deg   279 mNm   8.2 N  0.012/0.005  8 min
  install_light_bulb_4rev           PASS     3.83  2.335 (2.340)    49 deg   179 mNm   9.3 N  0.014/0.001 16 min
  install_light_bulb_2_50mm         PASS     1.95  4.957 (4.665)   226 deg   350 mNm   9.3 N  0.012/0.003 10 min
  install_light_bulb_3_70mm         PASS     1.95  4.815 (4.665)   227 deg   220 mNm  10.2 N  0.012/0.002  9 min
  put_bottle_cap_onto_bottle        PASS     1.96  4.554 (4.664)   178 deg   314 mNm   6.1 N  0.065/0.008 17 min
  put_bottle_cap_onto_bottle_2      PASS     1.96  4.338 (4.665)   132 deg   484 mNm   4.9 N  0.033/0.004  9 min
  put_bottle_cap_onto_bottle_3      PASS     1.95  4.536 (4.663)   177 deg   911 mNm   7.5 N  0.006/0.000 11 min
  put_bottle_cap_onto_bottle_4rev   PASS     3.85  2.318 (2.329)    36 deg   422 mNm   5.5 N  0.047/0.002 15 min
  repeat runs (same code, separate output root): install_light_bulb PASS, preload 9.0 N; put_bottle_cap_onto_bottle PASS, preload 6.0 N
  install_light_bulb_from_table: lifted 45 mm off its stand (0 N stand contact through the carry), then as in hand.
  The peak torques are the lock: past the take-up the seat stiffens to tens of N per degree, so the twist
  (0.8 rad/s) overshoots the 150 mN.m stop by up to the pads' slip limit (~0.9 N.m on the 66 mm cap).

Run:  python -m force_bench.appendix_tasks.install_light_bulb [--bulb lightbulb-2 --size 50mm --rev 2] [--seed N]
      python -m force_bench.appendix_tasks.put_bottle_cap_onto_bottle [--bottle bottle-2 --rev 3] [--seed N]
      --seed N (in-hand rows) random fixture position (0.15 x 0.15 m square); ``force-bench run`` names the folder
      <row>__seed<N>;
      env KT_STOP_AFTER=<phase> ends a rollout when <phase> starts (iteration only, no verdict);
      KT_DEBUG_D_MM=<mm> starts the part on its thread that far above the seat (iteration only);
      `kill -USR1 <pid>` ends a running rollout with the expert log dumped
"""

from __future__ import annotations

import argparse
import functools
import json
import os
from dataclasses import dataclass, field

import numpy as np
import trimesh
import warp as wp

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv_processed as subdiv
from force_bench.kit import load_kit
from force_bench.kit.build import add_part_shape, part_mass, pose_matrix
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.nut_bolt_arm import PairProbe, YawTrack
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, check_released_and_clear, save_appendix_verdict, StopRun, Placement, sample_start)

MM = 1.0e-3

# ── layout (m) ──────────────────────────────────────────────────────────────────────────────────────────────────────
ROBOT_BASE = (0.0, 0.40, 0.0)
AXIS_XY = np.array([0.0, 0.0])          # fixture axis on the table (nominal; a seeded in-hand start draws it)
# --seed (in-hand rows): the fixture's axis in the paper's 0.15 x 0.15 m square; no yaw (the socket and bottles are round)
AXIS_PLACE = Placement((float(AXIS_XY[0]), float(AXIS_XY[1])), dxy=0.075, radius=0.05)
STAND_XY = np.array([0.105, 0.030])     # bulb from the table: the stand
START_ABOVE = 0.004                     # in-hand start: part's lowest point this far above the fixture top
YAW0_DEG = 25.0                         # arbitrary start phase of the part's thread

# ── contact model ───────────────────────────────────────────────────────────────────────────────────────────────────
GAP = 0.0003              # contact detection gap of the part and fixture shapes
ROBOT_GAP = 0.003         # the FR3's and the ground's (see run(): the default 0.1 m floods the buffer)
SDF_BAND = 0.0012
SDF_PAD = 0.0005
CELL_FINE = 0.00008       # m: threaded ends (0.3 mm radial clearance)
CELL_COARSE = 0.00035     # m: envelopes (pads, stand)
EDGE_FINE = 0.0010        # max edge after subdivision (physics_notes pitfall 3.2)
EDGE_COARSE = 0.0030
SUBSTEPS = 32
ITERS = 20
IMPRATIO = 10.0           # friction/normal impedance ratio (elliptic cone): grasp and thread stiction

# ── FR3 fingers ─────────────────────────────────────────────────────────────────────────────────────────────────────
FINGER_KE, FINGER_KD = 8000.0, 60.0
FINGER_F_MAX = 25.0       # N per pad (MuJoCo forcerange)
FINGER_MASS = 0.12
PAD_MU = 1.4              # elastomer pads (suite value; MuJoCo pair mu = max)
PAD_OFF = 0.000085        # pad face = finger dof - this (measured on the FR3 URDF)
PAD_LO, PAD_HI = -0.00945, 0.00905      # pad face extent along world z, relative to the TCP (measured)
PALM_ABOVE_TCP = 0.03745                # FR3 palm plate (collision hull) above the TCP (measured)
FINGER_MAX = 0.0399                     # finger dof at the joint limit (0.04) less a hair
GRIP_OVERDRIVE = 0.0010                 # ~8 N per pad
OPEN_CLEAR = 0.0060                     # per side off the grip radius when open (less if the limit binds)
F_LO, F_HI = 5.0, 9.0                   # admittance band (mean pad force)
F_LO_SEAT, F_HI_SEAT = 8.0, 12.0        # ... once seated: more torsional friction at a point-contact pinch
                                        # (the one-sided flank force at the lock tilts a bulb held at its
                                        # equator 78 mm above the thread about the pinch line)
RELAX_RATE = 0.00004
RELAX_OPEN_MAX, RELAX_CLOSE_MAX = 0.0008, 0.0015

# ── expert ──────────────────────────────────────────────────────────────────────────────────────────────────────────
YAW_HI, YAW_LO = np.radians(80.0), np.radians(-80.0)    # wrist window (IK-checked at build)
OMEGA = 1.2               # rad/s wrist twist on the free thread
OMEGA_ENGAGE = 1.0        # rad/s while the thread is being found
OMEGA_TAKEUP = 0.8        # rad/s seated, before the flanks bear: the kit threads have ~2.6 mm of axial play
                          # (a 1.05 mm ridge in a 4.67 mm lead), i.e. up to ~200 deg of turning at the seat
OMEGA_SEAT = 0.25         # rad/s once the pad torque passes TAU_SLOW_FRAC x TAU_TIGHT
TAU_SLOW_FRAC = 0.4
LAG_STOP, LAG_TAPER = 0.35, 0.15       # rad: wrist ahead of the part -> slow/stop
LAG_HOLD_S = 0.6          # s frozen at LAG_STOP ends the stroke (pads slip)
V_FAST, V_SLOW = 0.008, 0.002          # m/s descent
ENGAGED_LEN = 0.0094      # m of thread engaged when seated (measured, every kit pair): the first thread
SLOW_ABOVE = 0.003        # contact is expected that far above the seat; slow down this much before it
F_TOUCH = 0.6             # N thread force = first touch
F_PRESS = 1.5             # N light press while engaging
PRESS_RATE = 0.00002      # m/frame
PRESS_MAX = 0.0030        # m
ENGAGE_MM = 1.0           # descent below the first touch, and ...
ENGAGE_WIN = np.radians(45.0)   # ... over the last 45 deg of turning ...
ENGAGE_RATE = 0.75        # ... >= 0.75 x the lead
ENGAGE_MAX_TURNS = 1.5
XY_GAIN, XY_RATE, XY_MAX = 0.10, 0.00004, 0.006
Z_RATE = 0.0003           # m/frame cap on the z follow
F_AX_PRESS = 0.5          # N: the hand's axial press while twisting on the thread
F_AX_PRESS_SEAT = 3.0     # N: ... once seated: the part's seat face held flat on the seat while the flanks
                          # lock (a bulb tilted or lifted at the lock loses the preload when released)
T_CHECK = 0.4             # s with the pads open at the part after a release: is the preload still there?
MAX_RETIGHTEN = 2         # ... if not, re-grip and tighten again (as a person would check), this often
AX_GAIN, AX_RATE, AX_MAX = 0.00002, 0.000008, 0.0015
FING_PRECLOSE = 0.0010    # m: fast close to 1 mm off the grip radius, then ...
V_CLOSE = 0.006           # m/s per finger for the last mm + the overdrive
T_PREROLL = 0.3
T_HOLD0 = 0.4
T_CLOSE, T_OPEN, T_RET = 0.25, 0.35, 1.3
T_TIGHT_HOLD = 0.4
T_RELEASE, T_RETREAT, T_STILL = 0.6, 1.2, 0.8
RETREAT = 0.070
MAX_STROKES = 16
STALL_DEG = 4.0           # an engaged stroke turning the part less than this ...
STALL_STROKES = 2         # ... this many times in a row = torque limit
# tightening (no tab: seat contact + stall + torque, see the module doc)
F_SEAT_TOUCH = 0.3        # N axial seat force = the seat face touched
TAU_TIGHT = 0.150         # N.m tightening torque the expert stops at (pads, about the part axis) ...
PRELOAD_TIGHT = 5.0       # N ... with the thread flanks pulling the part onto its seat this hard (the
                          # flanks have locked; during the take-up they carry nothing, and the touch
                          # itself spikes the pad torque and, through the hand, the seat force)
T_TIGHT_CONFIRM = 0.15    # s the torque must hold, with the part stalled
STALL_RATE = np.radians(6.0)   # rad/s: "stalled" = turning slower than this (mean over the hold)
STALL_HOLD_DEG = 1.0      # deg the part may turn during the T_TIGHT_CONFIRM hold and still count as stalled
SEAT_DZ = 0.0004          # m: a fixture-side contact this close to the seat plane, inside its radial band
# pick (bulb from the table)
APPROACH_H = 0.050
LIFT_CLEAR = 0.045
T_APPROACH, T_DOWN, T_LIFT, T_CARRY, T_SETTLE = 1.4, 1.2, 1.2, 2.6, 0.4
F_PICK_OK = 4.0
# the holding probe
PROBE_TAU = 0.010         # N.m, the "small force"
T_PROBE = 1.0

# ── verdict thresholds ──────────────────────────────────────────────────────────────────────────────────────────────
STATIONARY_DEG = 1.0
LEAD_TOL = 0.15
MIN_TURNS_FRAC = 0.5      # turns from first thread contact to tight >= this x the kit's revolutions: the
                          # axial play lets the part drop up to ~0.6 turn's worth before the thread bites
SEAT_GAP_MAX = 0.0003     # m: seat face within this of the fixture seat after release
PRELOAD_MIN = 1.0         # N of axial seat force beyond the part's weight after release
TILT_MAX_DEG = 2.0
PEN_TOL = 0.0010


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# measured geometry
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
def _rays_z(tm: trimesh.Trimesh, r: float, up: bool) -> list[float]:
    """z of every surface a vertical ray at radius r meets (mesh in metres), nearest first."""
    z0 = tm.bounds[0][2] - 0.1 if up else tm.bounds[1][2] + 0.1
    o = np.array([[r, 0.31 * MM, z0]])
    d = np.array([[0.0, 0.0, 1.0 if up else -1.0]])
    loc, _, _ = tm.ray.intersects_location(o, d)
    return sorted(loc[:, 2].tolist(), reverse=not up) if len(loc) else []


def outer_profile(tm: trimesh.Trimesh, z0: float, z1: float, dz: float = 0.0005) -> np.ndarray:
    """(z, max radius) of the part's sections between z0 and z1 (metres)."""
    zs = np.arange(z0, z1, dz)
    lines = trimesh.intersections.mesh_multiplane(tm, [0, 0, 0], [0, 0, 1], zs)[0]
    out = []
    for z, seg in zip(zs, lines):
        if len(seg):
            out.append((z, float(np.hypot(seg[..., 0], seg[..., 1]).max())))
    return np.array(out)


def ring_radii(tm: trimesh.Trimesh, z: float) -> tuple[float, float]:
    """(min, max) radius of the outermost section loop at height z: an external thread's root and crest."""
    sec = tm.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    loops = [np.hypot(*np.asarray(e)[:, :2].T) for e in sec.discrete]
    r = max(loops, key=lambda a: a.mean())
    return float(r.min()), float(r.max())


def thread_lead(tm: trimesh.Trimesh, z0: float, z1: float, r_win: tuple[float, float], guess: float) -> float:
    """Lead (m/turn, right hand) of a single-start thread by HELICAL SELF-CONSISTENCY: the thread's vertices,
    rotated half a turn and lifted by lead/2, must land back on the surface; the lead minimising the mean
    mismatch is the thread's (3 um residual on the kit parts).  Slice-based estimates (angle of the crest
    vertex, phase of r(theta)'s first harmonic) scatter by +-1 % on these tessellated knuckle threads."""
    v = np.asarray(tm.vertices)
    r = np.hypot(v[:, 0], v[:, 1])
    sel = v[(v[:, 2] > z0) & (v[:, 2] < z1) & (r > r_win[0]) & (r < r_win[1])]
    sel = sel[np.random.RandomState(0).choice(len(sel), min(250, len(sel)), replace=False)]
    # query only the thread region (unsigned distance to the nearest face): ~10x faster than the full part
    fz = v[tm.faces][:, :, 2]
    fr = np.hypot(v[tm.faces][:, :, 0], v[tm.faces][:, :, 1])
    keep = ((fz > z0 - 1 * MM).all(1) & (fz < z1 + 0.5 * guess + 1 * MM).all(1)
            & (fr > r_win[0] - 1 * MM).all(1) & (fr < r_win[1] + 1 * MM).all(1))
    sub = trimesh.Trimesh(v, tm.faces[keep], process=False)
    pq = trimesh.proximity.ProximityQuery(sub)
    R = np.diag([-1.0, -1.0, 1.0])                          # half a turn about z

    def mismatch(lead):
        return float(pq.on_surface(sel @ R.T + [0.0, 0.0, 0.5 * lead])[1].mean())

    best = guess
    for span, step in ((0.25, 0.02), (0.02, 0.002), (0.002, 0.0002)):
        cands = best * (1.0 + np.arange(-span, span + 1e-12, step))
        best = float(min(cands, key=mismatch))
    return best


@dataclass
class Pair:
    """One threaded kit pair, measured from its STLs (metres, CAD frame of the pair)."""
    kind: str                       # "bulb" | "cap"
    row_variant: str                # human label, e.g. "lightbulb-1 55 mm, 2 revolutions"
    revs: float
    fixture_key: str
    fixture_choice: dict
    fixture_part: str
    free_key: str
    free_choice: dict
    free_part: str
    lead: float = 0.0               # m/turn (free part's thread)
    seat_lift: float = 0.0          # CAD pose -> seated pose, along the helix (m up)
    free_seat_z: float = 0.0        # free part's seat face (bulb tip / cap ceiling), CAD z
    fixture_seat_z: float = 0.0     # fixture's seat face (slot floor / bottle lip)
    seat_r: tuple = (0.0, 0.0)      # radial band of the seat contact
    fixture_low_z: float = 0.0      # stands on the table here
    fixture_top_z: float = 0.0
    free_low_z: float = 0.0
    free_top_z: float = 0.0
    thread_z: tuple = (0.0, 0.0)    # free part's full-depth thread (CAD z)
    root_r: float = 0.0             # free part thread root / crest radii
    crest_r: float = 0.0
    grip_z: float = 0.0             # TCP height on the free part (CAD z)
    grip_r: float = 0.0             # radius the pads close on
    grip_contact_z: float = 0.0     # where the pads bear (CAD z)
    palm_clear: float = 0.0
    fine_cut: float | None = None   # bulb: fine collider below this z (a cap is fine all over)
    coarse_cut: float | None = None # bulb: coarse collider above this z
    fixture_cut: float = 0.0        # fixture: fine collider above this z
    notes: list = field(default_factory=list)


def _kit_part(key: str, choice, name: str):
    a = load_kit()[key]
    for p in a.parts(choice):
        if p.name == name:
            return a, p
    raise KeyError(f"{key}: no part {name} in {choice}")


@functools.lru_cache(maxsize=None)
def measure_pair(kind: str, free_key: str, size: str, revs: str) -> Pair:
    """Build and measure the pair spec of one row (cached per process)."""
    rev_label = f"{revs} revolutions"
    if kind == "bulb":
        fc = {"Lightbulb geometry": f"{size} diameter", "Amount of rotation to tighten": rev_label}
        n = free_key.split("-")[1]
        P = Pair(kind, f"{free_key} {size.replace('mm', ' mm')}, {revs} revolutions", float(revs),
                 "lightbulb-slot", {"Amount of rotation to tighten": rev_label}, f"lightbulb_slot_{revs}_rev",
                 free_key, fc, f"lightbulb_{n}_{size}_{revs}_rev")
    else:
        n = free_key.split("-")[1]
        ch = {"Amount of rotation to tighten": rev_label}
        P = Pair(kind, f"{free_key}, {revs} revolutions", float(revs), free_key, ch, f"bottle_{n}_{revs}_rev",
                 free_key, ch, f"cap_{n}_{revs}_rev")
    _, pf = _kit_part(P.fixture_key, P.fixture_choice, P.fixture_part)
    _, pp = _kit_part(P.free_key, P.free_choice, P.free_part)
    fx, fr = pf.mesh(), pp.mesh()
    P.fixture_low_z, P.fixture_top_z = float(fx.bounds[0][2]), float(fx.bounds[1][2])
    P.free_low_z, P.free_top_z = float(fr.bounds[0][2]), float(fr.bounds[1][2])
    if kind == "bulb":
        P.fixture_seat_z = _rays_z(fx, 4 * MM, up=False)[0]           # slot floor
        P.free_seat_z = _rays_z(fr, 4 * MM, up=True)[0]               # bulb tip face
        prof = outer_profile(fr, -0.5 * MM, 12.5 * MM, 0.0001)
        full = prof[prof[:, 1] > 13.4 * MM]
        P.thread_z = (float(full[0, 0]), float(full[-1, 0]))
        P.root_r, P.crest_r = ring_radii(fr, 0.5 * (P.thread_z[0] + P.thread_z[1]))
        P.seat_r = (0.0, 10.5 * MM)                                    # tip face r <= 8.7 mm
        P.lead = thread_lead(fr, P.thread_z[0] + 0.8 * MM, P.thread_z[1] - 2.8 * MM, (12.6 * MM, 13.65 * MM),
                             9.33 * MM / float(revs))
        P.fine_cut, P.coarse_cut = 17.0 * MM, 14.0 * MM
        P.fixture_cut = -2.95 * MM            # the collar + floor (the flange stays visual only)
        # grasp: widest point the pads can reach with the palm clear of the top
        g = outer_profile(fr, 20 * MM, P.free_top_z, 0.0005)
        z_rmax, r_max = (float(v) for v in g[np.argmax(g[:, 1])])
        tcp_min = P.free_top_z - PALM_ABOVE_TCP + 3.0 * MM               # >= 3 mm palm clearance
        tcp = max(z_rmax, tcp_min)
        lo, hi = tcp + PAD_LO, tcp + PAD_HI
        band = g[(g[:, 0] >= lo) & (g[:, 0] <= hi)]
        k = int(np.argmax(band[:, 1]))
        P.grip_z, P.grip_r, P.grip_contact_z = tcp, float(band[k, 1]), float(band[k, 0])
        if P.grip_r + PAD_OFF + 0.0004 > FINGER_MAX:
            raise ValueError(f"{P.row_variant}: grip radius {P.grip_r * 1e3:.2f} mm does not fit the FR3 "
                             f"opening (max pad half-gap {(FINGER_MAX - PAD_OFF) * 1e3:.2f} mm)")
        P.palm_clear = tcp + PALM_ABOVE_TCP - P.free_top_z
    else:
        P.fixture_seat_z = None
        sec = fx.section(plane_origin=[0, 0, P.fixture_top_z - 1.0 * MM], plane_normal=[0, 0, 1])
        rs = sorted(float(np.hypot(*np.asarray(e)[:, :2].T).min()) for e in sec.discrete)
        bore, root = rs[0], rs[1]
        r_lip = 0.5 * (bore + root)
        P.fixture_seat_z = _rays_z(fx, r_lip, up=False)[0]              # bottle lip
        P.free_seat_z = _rays_z(fr, r_lip, up=True)[0]                  # cap ceiling
        P.seat_r = (bore - 0.3 * MM, root - 0.15 * MM)                # lip top face, not the thread run-out
        P.root_r = root
        prof = outer_profile(fx, P.fixture_top_z - 12 * MM, P.fixture_top_z, 0.0001)
        full = prof[prof[:, 1] > root + 0.9 * MM]
        P.crest_r = float(full[:, 1].max())
        P.thread_z = (float(full[0, 0]), float(full[-1, 0]))           # neck thread (fixture CAD z)
        P.lead = thread_lead(fx, P.thread_z[0] + 0.8 * MM, P.thread_z[1] - 2.8 * MM,
                             (root + 0.1 * MM, root + 1.15 * MM), 9.33 * MM / float(revs))
        P.fixture_cut = P.fixture_top_z - 15.0 * MM                     # neck + shoulder top
        r_out = float(outer_profile(fr, P.free_low_z + 1 * MM, P.free_top_z - 1 * MM, 0.0005)[:, 1].max())
        P.grip_r = r_out
        P.grip_z = 0.5 * (P.free_low_z + P.free_top_z)
        P.grip_contact_z = P.grip_z
        P.palm_clear = P.grip_z + PALM_ABOVE_TCP - P.free_top_z
    P.seat_lift = P.fixture_seat_z - P.free_seat_z
    return P


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# scene
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
def _boundary_loops(tm: trimesh.Trimesh) -> list[np.ndarray]:
    """Closed loops (vertex index arrays) of the edges used by exactly one face."""
    e = tm.edges_sorted
    uniq, cnt = np.unique(e, axis=0, return_counts=True)
    nb: dict[int, list[int]] = {}
    for a, b in uniq[cnt == 1]:
        nb.setdefault(int(a), []).append(int(b))
        nb.setdefault(int(b), []).append(int(a))
    loops, seen = [], set()
    for v0 in nb:
        if v0 in seen:
            continue
        loop, prev, v = [v0], None, v0
        seen.add(v0)
        while True:
            nxt = [w for w in nb[v] if w != prev]
            if not nxt or nxt[0] == v0:
                break
            prev, v = v, nxt[0]
            if v in seen:
                break
            loop.append(v)
            seen.add(v)
        loops.append(np.asarray(loop))
    return loops


def _slice(tm: trimesh.Trimesh, z: float, keep_above: bool) -> trimesh.Trimesh:
    """The part on one side of the plane z, closed with a planar cap.  The cut sections here are star-shaped
    about the part's axis (a disk, or an annulus between two such loops), so the cap is a fan from the axis
    or an angle-ordered zipper between the two loops (no triangulation library needed)."""
    n = [0, 0, 1] if keep_above else [0, 0, -1]
    out = trimesh.intersections.slice_mesh_plane(tm, plane_normal=n, plane_origin=[0, 0, z], cap=False)
    out.merge_vertices()
    V = np.asarray(out.vertices)
    loops = [lp for lp in _boundary_loops(out) if np.allclose(V[lp, 2], z, atol=1e-7)]
    ang = lambda lp: np.arctan2(V[lp, 1], V[lp, 0])                       # noqa: E731
    loops = [lp[np.argsort(ang(lp))] for lp in loops]
    loops.sort(key=lambda lp: float(np.hypot(V[lp, 0], V[lp, 1]).mean()))
    faces, extra = [], []
    if len(loops) == 1:
        c = len(V)
        extra.append([0.0, 0.0, z])
        lp = loops[0]
        faces += [(c, int(lp[k]), int(lp[(k + 1) % len(lp)])) for k in range(len(lp))]
    elif len(loops) == 2:
        inner, outer = loops
        ai, ao = ang(inner), ang(outer)
        i = j = 0
        ni, no = len(inner), len(outer)
        while i < ni or j < no:
            a_i = ai[(i + 1) % ni] + (2 * np.pi if i + 1 >= ni else 0.0)
            a_o = ao[(j + 1) % no] + (2 * np.pi if j + 1 >= no else 0.0)
            if (a_o <= a_i and j < no) or i >= ni:
                faces.append((int(inner[i % ni]), int(outer[j % no]), int(outer[(j + 1) % no])))
                j += 1
            else:
                faces.append((int(inner[i % ni]), int(outer[j % no]), int(inner[(i + 1) % ni])))
                i += 1
    else:
        raise ValueError(f"_slice: {len(loops)} boundary loops at z={z * 1e3:.2f} mm (expected 1 or 2)")
    Vn = np.vstack([V, np.asarray(extra).reshape(-1, 3)])
    Fc = np.asarray(faces)
    nz = np.cross(Vn[Fc[:, 1]] - Vn[Fc[:, 0]], Vn[Fc[:, 2]] - Vn[Fc[:, 0]])[:, 2]
    want = -1.0 if keep_above else 1.0                     # cap normal points away from the kept side
    flip = nz * want < 0
    Fc[flip] = Fc[flip][:, ::-1]
    res = trimesh.Trimesh(Vn, np.vstack([out.faces, Fc]), process=True)
    return res


def _res(tm: trimesh.Trimesh, cell: float) -> int:
    return int(np.ceil(float(tm.extents.max()) / cell))


def _collider(rig, body, tm, label, cell, edge, mu, origin=np.zeros(3)):
    """Invisible SDF collision shape on `body` (-1: static), `tm` in the body (or world) frame."""
    w = subdiv(tm, edge)
    mesh, c = make_newton_mesh(w, sdf=True, sdf_resolution=_res(w, cell), narrow_band=SDF_BAND,
                               margin=SDF_PAD)
    cfg = rig.cfg(mu=mu, gap=GAP, is_visible=False)
    cfg.density = 0.0
    sid = rig.builder.add_shape_mesh(
        body, mesh=mesh, xform=wp.transform(wp.vec3(*(np.asarray(c, float) + origin)), wp.quat_identity()),
        cfg=cfg, label=label)
    rig.shapes.setdefault(label, []).append(sid)
    return sid


def _visual(rig, body, tm, label, color, origin=np.zeros(3)):
    mesh, c = make_newton_mesh(tm, sdf=False)
    cfg = rig.cfg(is_visible=True, has_shape_collision=False)
    cfg.has_particle_collision = False
    cfg.density = 0.0
    sid = rig.builder.add_shape_mesh(
        body, mesh=mesh, xform=wp.transform(wp.vec3(*(np.asarray(c, float) + origin)), wp.quat_identity()),
        cfg=cfg, color=color, label=label)
    rig.shapes.setdefault(label, []).append(sid)
    return sid


def _stand_mesh(P: Pair, rest_z: float) -> trimesh.Trimesh:
    """Printed pick-up stand for a bulb (stand frame: origin at its bottom centre on the table): a ring
    whose 45 deg conical seat matches the bulb's 45 deg neck cone (r = z for z 12.5..14.9 mm on every kit
    bulb), bore 0.5 mm clear of the thread crest."""
    bore = P.crest_r + 0.5 * MM
    # bulb cone in world: r = (z_w - rest_z) + c0, with c0 from the bulb (r ~ z_b on the cone)
    c0 = 0.0
    z_seat0 = rest_z + bore - c0                 # where the cone has radius `bore`
    top = z_seat0 + 1.4 * MM
    r_top = bore + 1.4 * MM
    r_out = 22.0 * MM
    prof = [(bore, 0.0), (r_out, 0.0), (r_out, top), (r_top, top), (bore, z_seat0), (bore, 0.0)]
    return trimesh.creation.revolve(np.asarray(prof), sections=128)


@dataclass
class Scene:
    P: Pair
    part: int                    # free body
    part_shapes: list
    fixture_shapes: list
    fixture_world: np.ndarray    # world_from_cad of the fixture
    free_vis: trimesh.Trimesh    # free part (metres, CAD frame)
    fixture_vis: trimesh.Trimesh
    p0: np.ndarray               # free body start position (world)
    q0: np.ndarray
    mass: float
    statics: list
    static_labels: list
    stand_shapes: list = field(default_factory=list)
    stand_world: np.ndarray | None = None
    rest_z: float = 0.0
    axis_xy: np.ndarray = field(default_factory=lambda: AXIS_XY.copy())   # the fixture's axis (world xy)
    seed: int | None = None                                                 # the seeded start's seed, if any


def build_scene(rig, P: Pair, mode: str, yaw0: float, axis_xy=AXIS_XY) -> Scene:
    B = rig.builder
    axis_xy = np.asarray(axis_xy, float)
    rig.table(center_xy=(0.03, 0.12), half=0.35)
    kit = load_kit()
    fa, fpart = _kit_part(P.fixture_key, P.fixture_choice, P.fixture_part)
    pa, ppart = _kit_part(P.free_key, P.free_choice, P.free_part)
    mu = float(fa.sim.get("mu", kit.sim_defaults.get("mu", 0.5)))
    # ── fixture: static, its lowest point on the table ──────────────────────────────────────────────────────────
    W = pose_matrix(xyz=(axis_xy[0], axis_xy[1], -P.fixture_low_z))
    fx = fpart.mesh()
    add_part_shape(rig, -1, fa, fpart, W, "socket" if P.kind == "bulb" else "bottle", collide=False)
    fine = _slice(fx, P.fixture_cut, keep_above=True)
    flabel = "socket_thread" if P.kind == "bulb" else "bottle_neck"
    sf = _collider(rig, -1, fine, flabel, CELL_FINE, EDGE_FINE, mu, origin=W[:3, 3])
    fixture_shapes = [sf]
    if P.kind == "cap":       # the bottle's body: the pads could reach its shoulder
        body = _slice(fx, P.fixture_cut + 1.0 * MM, keep_above=False)
        fixture_shapes.append(_collider(rig, -1, body, "bottle_body", CELL_COARSE, EDGE_COARSE, mu,
                                        origin=W[:3, 3]))
    # ── free part: one body whose frame is its CAD frame ───────────────────────────────────────────────────────
    q0 = np.array([0.0, 0.0, np.sin(yaw0 / 2), np.cos(yaw0 / 2)])
    stand_shapes, stand_W, rest_z = [], None, 0.0
    if mode == "hand":
        d = (P.fixture_top_z + START_ABOVE) - P.free_low_z           # CAD offset of the free part
        dbg = os.environ.get("KT_DEBUG_D_MM")                         # iteration aid: start ON the thread,
        if dbg:                                                       # this far above the seat, in phase
            d = P.seat_lift + float(dbg) * MM
            yaw0 = 2 * np.pi * d / P.lead
            q0 = np.array([0.0, 0.0, np.sin(yaw0 / 2), np.cos(yaw0 / 2)])
            print(f"[{P.kind}] KT_DEBUG_D_MM: start {float(dbg):.1f} mm above the seat, yaw {np.degrees(yaw0):.1f}")
        p0 = np.array([axis_xy[0], axis_xy[1], W[2, 3] + d])
    else:
        rest_z = 6.0 * MM                                            # bulb CAD origin above the table:
        p0 = np.array([STAND_XY[0], STAND_XY[1], rest_z + 0.00005])  # tip 3 mm clear
    body = B.add_body(xform=wp.transform(wp.vec3(*p0), wp.quat(*q0)), label=P.kind)
    fr = ppart.mesh()
    add_part_shape(rig, body, pa, ppart, np.eye(4), P.kind, collide=False)   # visual + mass distribution
    part_shapes = []
    if P.kind == "bulb":
        part_shapes.append(_collider(rig, body, _slice(fr, P.fine_cut, keep_above=False), "bulb_base",
                                     CELL_FINE, EDGE_FINE, mu))
        part_shapes.append(_collider(rig, body, _slice(fr, P.coarse_cut, keep_above=True), "bulb_envelope",
                                     CELL_COARSE, EDGE_COARSE, mu))
    else:
        part_shapes.append(_collider(rig, body, fr, "cap_shell", CELL_FINE, EDGE_FINE, mu))
    mass = part_mass(pa, ppart)
    set_body_mass(B, body, mass)
    rig.shapes[P.kind] = list(part_shapes)
    rig.meshes[P.kind] = (fr, np.zeros(3))
    statics = list(fixture_shapes) + [rig.shapes["table"][0]]
    statics += [i for i, lbl in enumerate(B.shape_label) if lbl and "ground" in str(lbl)]
    static_labels = [flabel] + (["bottle_body"] if P.kind == "cap" else [])
    if mode == "table":
        stand = _stand_mesh(P, rest_z)
        stand_W = pose_matrix(xyz=(STAND_XY[0], STAND_XY[1], 0.0))
        s_st = _collider(rig, -1, stand, "stand", 0.00015, EDGE_COARSE, mu, origin=stand_W[:3, 3])
        _visual(rig, -1, stand, "stand_visual", (0.93, 0.93, 0.90), origin=stand_W[:3, 3])
        stand_shapes = [s_st]
        statics.append(s_st)
        static_labels.append("stand")
    for i, a in enumerate(statics):                     # static-static: never
        for b in statics[i + 1:]:
            B.add_shape_collision_filter_pair(a, b)
    return Scene(P, body, part_shapes, fixture_shapes, W, fr, fx, p0, q0, mass, statics, static_labels,
                 stand_shapes, stand_W, rest_z, axis_xy.copy())


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# measurement
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
def rotm(q_row) -> np.ndarray:
    x, y, z, w = (float(c) for c in q_row[3:7])
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ContactTap:
    """Per-contact view of the free part's contacts, sampled like a probe (after ``update_contacts``):

    seat     axial force the fixture's SEAT FACE puts on the part (fixture-side contact points within
             SEAT_DZ of the seat plane and inside the seat's radial band), N, + = pushing the part up
    flank    the rest of the fixture's force on the part (the thread), vector
    tau_pads tightening torque the fingers deliver about the part's own axis (+ = clockwise from above =
             tightening a right-hand thread), from each finger contact's force and position
    """

    def __init__(self, model, part_body: int, part_shapes, fixture_shapes, fingers: dict[str, list[int]],
                 seat_z: float, seat_r: tuple, axis_xy):
        n = model.shape_count
        self.role = np.zeros(n, np.int8)
        self.role[list(part_shapes)] = 1
        self.role[list(fixture_shapes)] = 2
        for k, (name, ids) in enumerate(fingers.items()):
            self.role[list(ids)] = 3 + k
        self.shape_body = model.shape_body.numpy()
        self.bi = part_body
        self.seat_z, self.seat_r, self.axis = float(seat_z), seat_r, np.asarray(axis_xy, float)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        nc = int(contacts.rigid_contact_count.numpy()[0])
        rec = {"t": t, "force": 0.0, "friction": 0.0, "seat": 0.0, "n_seat": 0, "flank": [0.0, 0.0, 0.0],
               "tau_pads": 0.0, "pad_N": [0.0, 0.0], "vec": [0.0, 0.0, 0.0]}
        if nc == 0:
            self.history.append(rec)
            return rec
        s0 = contacts.rigid_contact_shape0.numpy()[:nc]
        s1 = contacts.rigid_contact_shape1.numpy()[:nc]
        r0, r1 = self.role[s0], self.role[s1]
        on_part = (r0 == 1) | (r1 == 1)
        if not on_part.any():
            self.history.append(rec)
            return rec
        idx = np.flatnonzero(on_part)
        F = contacts.force.numpy()[idx, :3]
        sgn = np.where(r0[idx] == 1, 1.0, -1.0)[:, None]
        F_part = F * sgn                                          # force ON the part
        other = np.where(r0[idx] == 1, r1[idx], r0[idx])
        bq = state.body_q.numpy()
        row = bq[self.bi]
        R = rotm(row)
        p0l = contacts.rigid_contact_point0.numpy()[idx]
        p1l = contacts.rigid_contact_point1.numpy()[idx]
        pl = np.where((r0[idx] == 1)[:, None], p0l, p1l)          # part-side point, part frame
        pw = pl @ R.T + row[:3]
        # fixture side (static: already world)
        fx = other == 2
        if fx.any():
            pf = np.where((r0[idx] == 2)[:, None], p0l, p1l)[fx]
            rr = np.hypot(pf[:, 0] - self.axis[0], pf[:, 1] - self.axis[1])
            seat = (np.abs(pf[:, 2] - self.seat_z) < SEAT_DZ) & (rr >= self.seat_r[0]) & (rr <= self.seat_r[1])
            Ff = F_part[fx]
            rec["seat"] = float(Ff[seat, 2].sum())
            rec["n_seat"] = int(seat.sum())
            rec["flank"] = Ff[~seat].sum(axis=0).tolist()
            rec["vec"] = Ff.sum(axis=0).tolist()
            rec["force"] = float(np.linalg.norm(Ff.sum(axis=0)))
        a = R[:, 2]
        o = row[:3]
        tau = 0.0
        pads = [0.0, 0.0]
        for k in (3, 4):
            m = other == k
            if m.any():
                tau += float(np.cross(pw[m] - o, F_part[m]).sum(axis=0) @ a)
                pads[k - 3] = float(np.linalg.norm(F_part[m].sum(axis=0)))
        rec["tau_pads"] = -tau                                    # clockwise (tightening) positive
        rec["pad_N"] = pads
        self.history.append(rec)
        return rec


class ProbeTorque:
    """Solver wrapper writing the holding-probe torque to the part's world-z torque dof every substep (a free
    joint's joint_f goes to MuJoCo's xfrc_applied about the COM, on the part's axis): the BulbSeat /
    CapStiction pattern without the stiction element."""

    def __init__(self, solver, dof):
        self._solver = solver
        self.dof = dof
        self.probe = 0.0
        self._written = 0.0

    def step(self, state_in, state_out, control, contacts, dt):
        if self.probe != self._written:
            jf = control.joint_f.numpy()
            jf[self.dof] = self.probe
            control.joint_f.assign(jf)
            self._written = self.probe
        return self._solver.step(state_in, state_out, control, contacts, dt)

    def __getattr__(self, name):
        return getattr(self._solver, name)


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# the expert
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
def _q5(u):
    """Quintic smootherstep (physics_notes pitfall 8.4: no acceleration steps)."""
    u = float(np.clip(u, 0.0, 1.0))
    return u * u * u * (u * (6 * u - 15) + 10)


GRIP_PHASES = ("grip", "lift", "carry", "settle", "hold0", "align", "descend", "engage", "twist", "tight")


class ScrewExpert:
    def __init__(self, rig, S: Scene, mode, tcp0, yaw_start, fingers: dict):
        self.rig, self.S, self.P, self.mode = rig, S, S.P, mode
        self.bi = S.part
        self.wi = rig.body_index("fr3/fr3_link7")
        l7 = rig.ik.body_pose("fr3/fr3_link7")
        self.tcp_in_l7 = rotm(l7).T @ (np.asarray(tcp0) - l7[:3])
        q_tcp = down_grasp_quat(yaw_start)
        self.R_tcp_in_l7 = rotm(l7).T @ rotm(np.r_[0.0, 0.0, 0.0, q_tcp])
        self.tcp0 = np.asarray(tcp0, float)
        self.F = fingers                      # touch / grip / open finger targets
        self.part_yaw = YawTrack()
        self.omega_f = 0.0
        self._by_prev = None
        self.st = {"phase": "hold0" if mode == "hand" else "approach", "t0": 0.0,
                   "yaw": float(yaw_start), "z": float(tcp0[2]), "xy": self.tcp0[:2].copy(),
                   "xy_off": np.zeros(2), "f": fingers["grip"] if mode == "hand" else fingers["open"],
                   "relax": 0.0, "picked": mode == "hand", "t_lift": None,
                   "stroke": 0, "engaged": False, "z_touch": None, "by_touch": None, "t_touch": None,
                   "t_engaged": None, "by_engaged": None, "z_engaged": None, "press": 0.0, "z_press0": 0.0,
                   "eng_hist": [], "ax_trim": 0.0, "ref": None, "helix": None, "strokes": [], "stall_n": 0,
                   "lag_t": None, "t_seat": None, "by_seat": None, "Z_seat": None, "ztcp_seat": None,
                   "hold_since": None, "hold_by": None, "tau_peak": 0.0, "hold_rate": None,
                   "t_tight": None, "by_tight": None, "tight_reason": None, "retighten": 0, "checks": [],
                   "preload_hist": [],
                   "tau_at_stop": None, "rate_at_stop": None, "seat_at_stop": None,
                   "t_release": None, "t_retreat_end": None, "t_probe": None, "probe": 0.0, "from": None,
                   "phases": []}
        self.log = []

    # ── measurements ─────────────────────────────────────────────────────────────────────────────────────────
    def row(self):
        return self.rig.s0.body_q.numpy()[self.bi]

    def point(self, z_cad, row=None):
        row = self.row() if row is None else row
        return row[:3] + rotm(row) @ np.array([0.0, 0.0, z_cad])

    def grip_point(self, row=None):
        return self.point(self.P.grip_z, row)

    def align_point(self, row=None):
        z = 0.5 * (self.P.thread_z[0] + self.P.thread_z[1]) if self.P.kind == "bulb" else self.P.free_low_z
        return self.point(z, row)

    def Z(self, row=None):
        """Part origin height above the fixture's CAD origin (m): the helix coordinate."""
        row = self.row() if row is None else row
        return float(row[2] - self.S.fixture_world[2, 3])

    def seat_gap(self, row=None):
        """Seat face above the fixture's seat face (m); 0 = seated (upright part)."""
        row = self.row() if row is None else row
        return float(self.point(self.P.free_seat_z, row)[2]
                     - (self.S.fixture_world[2, 3] + self.P.fixture_seat_z))

    def low_above_top(self, row=None):
        row = self.row() if row is None else row
        return float(self.point(self.P.free_low_z, row)[2] - (self.S.fixture_world[2, 3] + self.P.fixture_top_z))

    def wrist_yaw_meas(self):
        """Measured TCP yaw (unwrapped against the command): the twist the hand really applied."""
        R = rotm(self.rig.s0.body_q.numpy()[self.wi])
        x = R @ (self.R_tcp_in_l7 @ np.array([1.0, 0.0, 0.0]))
        y = float(np.arctan2(x[1], x[0]))
        c = self.st["yaw"]
        return c + (y - c + np.pi) % (2 * np.pi) - np.pi

    def tcp_meas(self):
        l7 = self.rig.s0.body_q.numpy()[self.wi]
        return l7[:3] + rotm(l7) @ self.tcp_in_l7

    def tap(self):
        h = self.rig.probes["tap"].history
        return h[-1] if h else {"seat": 0.0, "tau_pads": 0.0, "pad_N": [0.0, 0.0], "vec": [0.0, 0.0, 0.0],
                                "force": 0.0, "flank": [0.0, 0.0, 0.0], "n_seat": 0}

    def pads(self):
        return tuple(self.tap()["pad_N"])

    def go(self, phase, t):
        st = self.st
        st["phases"].append((round(t, 3), phase))
        st["phase"], st["t0"] = phase, t
        tp = self.tap()
        print(f"[{self.P.kind}] t={t:6.2f}s -> {phase:13s} Z={self.Z() * 1000:7.2f} mm "
              f"turns={self.part_yaw.total / (2 * np.pi):+6.3f} seat_gap={self.seat_gap() * 1000:6.2f} mm "
              f"pads={[round(v, 1) for v in tp['pad_N']]} N thread={tp['force']:5.2f} N seat={tp['seat']:5.2f} N "
              f"tau={tp['tau_pads'] * 1000:6.1f} mNm wrist={np.degrees(st['yaw']):6.1f}", flush=True)
        stop = os.environ.get("KT_STOP_AFTER")
        if stop and phase == stop:
            raise StopRun

    # ── strokes ──────────────────────────────────────────────────────────────────────────────────────────────
    def _stroke_start(self, t, by, phase):
        st = self.st
        st["ref"] = {"by0": by, "yaw0": st["yaw"], "t0": t, "z0": self.Z()}
        if st["engaged"]:
            st["helix"] = {"by0": by, "ztcp0": st["z"]}
            st["ax_trim"] = 0.0
        if st["t_seat"] is not None:          # re-anchor on the regrasp (pads re-closed on the seated part)
            st["ztcp_seat"] = st["z"]
        st["lag_t"] = None
        st["hold_since"] = None
        self.go(phase, t)

    def _engage_start(self, t, by):
        st = self.st
        st["z_press0"], st["press"] = st["z"], 0.0
        st["eng_hist"] = []
        self._stroke_start(t, by, "engage")

    def _stroke_end(self, t, by):
        st = self.st
        r = st["ref"]
        d_b = np.degrees(by - r["by0"])
        d_w = np.degrees(st["yaw"] - r["yaw0"])
        st["strokes"].append({"idx": st["stroke"], "kind": st["phase"], "t": [round(r["t0"], 2), round(t, 2)],
                              "wrist_deg": round(d_w, 1), "part_deg": round(d_b, 1),
                              "dZ_mm": round((self.Z() - r["z0"]) * 1000, 3),
                              "seat_N": round(self.tap()["seat"], 2), "engaged": bool(st["engaged"])})
        st["stroke"] += 1
        if st["engaged"] and st["phase"] == "twist":
            st["stall_n"] = st["stall_n"] + 1 if abs(d_b) < STALL_DEG else 0

    def _tight(self, t, by, reason):
        st = self.st
        tp = self.tap()
        st["t_tight"], st["by_tight"], st["tight_reason"] = t, by, reason
        held = reason == "seat torque"
        st["tau_at_stop"] = st["tau_peak"] if held else tp["tau_pads"]
        st["rate_at_stop"] = st["hold_rate"] if held else self.omega_f
        st["seat_at_stop"] = tp["seat"]
        self.go("tight", t)

    def _lagged_cw(self, t, by, omega=OMEGA):
        """Advance the clockwise twist, lag-limited; returns (lag, frozen_s)."""
        st, r = self.st, self.st["ref"]
        lag = (by - r["by0"]) - (st["yaw"] - r["yaw0"])     # > 0: wrist ahead
        scale = float(np.clip((LAG_STOP - lag) / LAG_TAPER, 0.0, 1.0))
        st["yaw"] = max(st["yaw"] - omega * scale / self.rig.fps, YAW_LO)
        if scale == 0.0:
            st["lag_t"] = st["lag_t"] if st["lag_t"] is not None else t
        else:
            st["lag_t"] = None
        return lag, (t - st["lag_t"]) if st["lag_t"] is not None else 0.0

    def _press(self):
        st = self.st
        fz = float(self.tap()["vec"][2])
        if fz < F_PRESS:
            st["press"] = min(st["press"] + PRESS_RATE, PRESS_MAX)
        elif fz > 1.5 * F_PRESS:
            st["press"] = max(st["press"] - PRESS_RATE, -PRESS_MAX)
        st["z"] = st["z_press0"] - st["press"]

    def _seat_touch(self, t, by, row, seat):
        st = self.st
        st["t_seat"], st["by_seat"], st["Z_seat"], st["ztcp_seat"] = t, by, self.Z(row), st["z"]
        print(f"[{self.P.kind}] t={t:6.2f}s SEAT contact at Z={self.Z(row) * 1000:.2f} mm "
              f"(seat gap {self.seat_gap(row) * 1000:+.3f} mm), {seat:.2f} N", flush=True)

    def _check_engaged(self, t, by, row):
        st = self.st
        Z = self.Z(row)
        if self.tap()["seat"] > F_SEAT_TOUCH:  # on its seat: the thread is engaged by geometry (the play can
            if st["t_seat"] is None:          # let the part drop onto the seat before the thread bites)
                self._seat_touch(t, by, row, self.tap()["seat"])
            st["engaged"], st["t_engaged"] = True, t
            st["by_engaged"], st["z_engaged"] = by, Z
            st["helix"] = {"by0": by, "ztcp0": st["z"]}
            st["ax_trim"] = 0.0
            return
        hist = st["eng_hist"]
        hist.append((by, Z))
        dz_touch = (st["z_touch"] - Z) * 1000.0
        if dz_touch < ENGAGE_MM:
            return
        old = [hz for hz in hist if hz[0] - by >= ENGAGE_WIN]
        if not old:
            return
        b0, z0 = old[-1]
        rate = (z0 - Z) / ((b0 - by) / (2 * np.pi))
        if rate >= ENGAGE_RATE * self.P.lead:
            st["engaged"], st["t_engaged"] = True, t
            st["by_engaged"], st["z_engaged"] = by, Z
            st["helix"] = {"by0": by, "ztcp0": st["z"]}
            turned = -(by - st["by_touch"])
            print(f"[{self.P.kind}] t={t:6.2f}s ENGAGED after {np.degrees(turned):.0f} deg: {dz_touch:.2f} mm "
                  f"below the first touch, now {rate * 1000:.2f} mm/turn", flush=True)

    # ── control ──────────────────────────────────────────────────────────────────────────────────────────────
    stop_requested = False

    def control(self, t):
        if ScrewExpert.stop_requested:
            raise StopRun
        st, rig, P = self.st, self.rig, self.P
        row = self.row()
        by = self.part_yaw.update(row)
        if self._by_prev is not None:
            self.omega_f = 0.8 * self.omega_f + 0.2 * (by - self._by_prev) * rig.fps
        self._by_prev = by
        ph = st["phase"]
        tau = t - st["t0"]
        tp = self.tap()
        fL, fR = tp["pad_N"]
        servo_xy = None
        F = self.F
        if ph == "approach":                  # (table) open, above the stand
            if st["from"] is None:
                st["from"] = np.array([*st["xy"], st["z"]])
            b = self.grip_point(row)
            p = st["from"] + (np.array([b[0], b[1], b[2] + APPROACH_H]) - st["from"]) * _q5(tau / T_APPROACH)
            st["xy"], st["z"], st["f"] = p[:2], p[2], F["open"]
            if tau >= T_APPROACH:
                st["from"] = None
                self.go("down", t)
        elif ph == "down":
            if st["from"] is None:
                st["from"] = np.array([*st["xy"], st["z"]])
            p = st["from"] + (self.grip_point(row) - st["from"]) * _q5(tau / T_DOWN)
            st["xy"], st["z"] = p[:2], p[2]
            if tau >= T_DOWN:
                st["from"] = None
                self.go("grip", t)
        elif ph == "grip":
            f_pre = F["touch"] + FING_PRECLOSE
            if tau < T_CLOSE:
                st["f"] = F["open"] + (f_pre - F["open"]) * _q5(tau / T_CLOSE)
            else:
                st["f"] = max(F["grip"], f_pre - V_CLOSE * (tau - T_CLOSE))
            st["relax"] = 0.0
            ok = min(fL, fR) >= F_PICK_OK and st["f"] <= F["grip"] + 1e-6
            if ok or tau > T_CLOSE + 1.5:
                if not st["picked"]:
                    st["picked"], st["t_lift"] = True, t
                    st["z_lift0"] = st["z"]
                    self.go("lift", t)
                elif st["engaged"]:
                    self._stroke_start(t, by, "twist")
                else:
                    self._engage_start(t, by)
        elif ph == "lift":
            st["z"] = st["z_lift0"] + LIFT_CLEAR * _q5(tau / T_LIFT)
            if tau >= T_LIFT:
                st["from"] = np.array([*st["xy"], st["z"]])
                self.go("carry", t)
        elif ph == "carry":
            p0 = st["from"]
            d = (P.fixture_top_z + START_ABOVE) - P.free_low_z
            dst = np.array([self.S.axis_xy[0], self.S.axis_xy[1], self.S.fixture_world[2, 3] + d + P.grip_z])
            u = _q5(tau / T_CARRY)
            st["xy"] = p0[:2] + (dst[:2] - p0[:2]) * u
            st["z"] = p0[2] + (dst[2] - p0[2]) * u
            if tau >= T_CARRY:
                st["from"] = None
                self.go("settle", t)
        elif ph == "settle":
            if tau >= T_SETTLE:
                self.go("align", t)
        elif ph == "hold0":
            if tau >= T_HOLD0:
                self.go("align", t)
        elif ph == "align":
            servo_xy = self.S.axis_xy - self.align_point(row)[:2]
            if (np.linalg.norm(servo_xy) < 0.00015 and tau > 0.3) or tau > 1.5:
                self.go("descend", t)
        elif ph == "descend":
            servo_xy = self.S.axis_xy - self.align_point(row)[:2]
            fast = self.Z(row) > P.seat_lift + ENGAGED_LEN + SLOW_ABOVE
            st["z"] -= (V_FAST if fast else V_SLOW) / rig.fps
            if tp["force"] > F_TOUCH:
                st["z_touch"], st["by_touch"], st["t_touch"] = self.Z(row), by, t
                self._engage_start(t, by)
        elif ph == "engage":
            servo_xy = self.S.axis_xy - self.align_point(row)[:2]
            self._press()
            lag, frozen = self._lagged_cw(t, by, OMEGA_ENGAGE)
            self._check_engaged(t, by, row)
            if st["engaged"]:
                st["phase"] = "twist"
            elif st["yaw"] <= YAW_LO + 1e-9 or frozen > LAG_HOLD_S:
                self._stroke_end(t, by)
                if -(by - st["by_touch"]) >= ENGAGE_MAX_TURNS * 2 * np.pi:
                    self._tight(t, by, "no thread engagement")
                else:
                    self.go("open", t)
        elif ph == "twist":
            seat = tp["seat"]
            if st["t_seat"] is None and seat > F_SEAT_TOUCH:
                self._seat_touch(t, by, row, seat)
            torque = tp["tau_pads"]
            done = False
            # tight = the torque crossed TAU_TIGHT with the seat loaded, the wrist then held still, and the
            # part did not turn over T_TIGHT_CONFIRM (stalled).  The wound-up pad torque itself is allowed to
            # relax during the hold: MuJoCo's friction is regularised and creeps under a static load.
            # the thread PRELOAD is the flanks' downward pull on the part (-flank z); a hand pushing the part
            # onto its seat loads the seat face too, but not the flanks
            preloaded = -float(tp["flank"][2]) >= PRELOAD_TIGHT
            if st["t_seat"] is not None and torque >= TAU_TIGHT and preloaded and st["hold_since"] is None:
                st["hold_since"], st["hold_by"], st["tau_peak"] = t, by, torque
            if st["hold_since"] is not None:
                st["tau_peak"] = max(st["tau_peak"], torque)
                if t - st["hold_since"] >= T_TIGHT_CONFIRM:
                    turned = abs(by - st["hold_by"])
                    if turned < np.radians(STALL_HOLD_DEG) and preloaded:
                        st["hold_rate"] = turned / (t - st["hold_since"])
                        self._stroke_end(t, by)
                        self._tight(t, by, "seat torque")
                        done = True
                    else:                              # it still turned: keep tightening
                        st["hold_since"] = None
            if not done:
                if st["hold_since"] is not None:      # frozen while the torque is being confirmed
                    lag, frozen = 0.0, 0.0
                else:
                    if st["t_seat"] is None:
                        om = OMEGA
                    elif torque < TAU_SLOW_FRAC * TAU_TIGHT:
                        om = OMEGA_TAKEUP              # seated: taking up the thread's axial play
                    else:
                        om = OMEGA_SEAT                # the flanks bear: torque rises, go slow
                    lag, frozen = self._lagged_cw(t, by, om)
                f_ax = float(tp["vec"][2])
                press = F_AX_PRESS if st["t_seat"] is None else F_AX_PRESS_SEAT
                err = f_ax - (self.S.mass * 9.81 + press)
                st["ax_trim"] = float(np.clip(st["ax_trim"] + np.clip(AX_GAIN * err, -AX_RATE, AX_RATE),
                                              -AX_MAX, AX_MAX))
                if st["t_seat"] is None:               # on the thread: the helix from the measured rotation
                    hx = st["helix"]
                    z_t = hx["ztcp0"] + P.lead * (by - hx["by0"]) / (2 * np.pi) + st["ax_trim"]
                else:                                  # seated: hold the height of the seat touch, light press
                    z_t = st["ztcp_seat"] + st["ax_trim"]  # only (NOT the measured part: lifting it would raise
                                                           # the command -- runaway into the ~2.9 mm thread play)
                st["z"] = float(np.clip(z_t, st["z"] - Z_RATE, st["z"] + Z_RATE))
                if st["yaw"] <= YAW_LO + 1e-9 or frozen > LAG_HOLD_S:
                    self._stroke_end(t, by)
                    if st["stall_n"] >= STALL_STROKES:
                        self._tight(t, by, "torque limit (part stopped following)")
                    elif st["stroke"] >= MAX_STROKES:
                        self._tight(t, by, "stroke budget")
                    else:
                        self.go("open", t)
        elif ph == "open":
            st["f"] = F["grip"] + (F["open"] - F["grip"]) * _q5(tau / T_OPEN)
            if tau >= T_OPEN:
                st["yaw_from"] = st["yaw"]
                self.go("return", t)
        elif ph == "return":
            st["yaw"] = st["yaw_from"] + (YAW_HI - st["yaw_from"]) * _q5(tau / T_RET)
            st["f"] = F["open"]
            if tau >= T_RET:
                self.go("grip", t)
        elif ph == "tight":
            if tau >= T_TIGHT_HOLD:
                st["t_release"] = t
                self.go("release", t)
        elif ph == "release":
            st["f"] = F["grip"] + (F["open"] - F["grip"]) * _q5(tau / T_RELEASE)
            if tau >= T_RELEASE:
                st["preload_hist"] = []
                self.go("check", t)
        elif ph == "check":                   # pads open at the part: did the preload survive the release?
            st["f"] = F["open"]
            st["preload_hist"].append(-float(tp["flank"][2]))
            if tau >= T_CHECK:
                pre = float(np.mean(st["preload_hist"][len(st["preload_hist"]) // 2:]))
                st["checks"].append({"t": round(t, 2), "preload_N": round(pre, 2)})
                if pre >= PRELOAD_MIN or st["retighten"] >= MAX_RETIGHTEN or st["t_seat"] is None:
                    st["z_rel"] = st["z"]
                    self.go("retreat", t)
                else:                         # it relaxed: re-grip at the seat reference and tighten again
                    st["retighten"] += 1
                    print(f"[{P.kind}] t={t:6.2f}s preload after release {pre:.2f} N < {PRELOAD_MIN} N: "
                          f"re-tighten ({st['retighten']})", flush=True)
                    st["t_release"] = None
                    self.go("grip", t)
        elif ph == "retreat":
            st["z"] = st["z_rel"] + RETREAT * _q5(tau / T_RETREAT)
            st["f"] = F["open"]
            if tau >= T_RETREAT:
                st["t_retreat_end"] = t
                self.go("still", t)
        elif ph == "still":
            if tau >= T_STILL:
                st["t_probe"] = t
                self.go("probe_loosen", t)
        elif ph == "probe_loosen":
            st["probe"] = +PROBE_TAU                 # counter-clockwise = loosen
            if tau >= T_PROBE:
                self.go("probe_tighten", t)
        elif ph == "probe_tighten":
            st["probe"] = -PROBE_TAU
            if tau >= T_PROBE:
                st["probe"] = 0.0
                self.go("end", t)
        elif ph == "end":
            if tau >= 0.3:
                raise StopRun
        ph = st["phase"]
        # ── z on the grip point while regrasping on the thread ─────────────────────────────────────────────
        if ph in ("return", "grip") and st["z_touch"] is not None:
            # seated: the reference height of the seat touch (the measured grip point would follow a light
            # part that the closing pads lift into the thread play)
            z_t = st["ztcp_seat"] if st["t_seat"] is not None else float(self.grip_point(row)[2])
            st["z"] = float(np.clip(z_t, st["z"] - Z_RATE, st["z"] + Z_RATE))
            servo_xy = self.grip_point(row)[:2] - self.tcp_meas()[:2]
        if ph == "twist":
            if st["t_seat"] is None:                  # on the thread: a symmetric pinch on the grip point
                servo_xy = self.grip_point(row)[:2] - self.tcp_meas()[:2]
            else:                                     # seated, the part pivots about its seat face: stand it
                servo_xy = self.align_point(row)[:2] - self.grip_point(row)[:2]   # upright (grip point over
                                                                                    # the thread centre)
        if servo_xy is not None:
            st["xy_off"] = np.clip(st["xy_off"] + np.clip(XY_GAIN * servo_xy, -XY_RATE, XY_RATE), -XY_MAX, XY_MAX)
        # ── admittance grip on the mean pad force ──────────────────────────────────────────────────────────
        gripping = (ph in GRIP_PHASES and ph != "grip") or (ph == "grip" and st["f"] <= F["grip"] + 1e-6)
        if gripping:
            fm = 0.5 * (fL + fR)
            f_lo, f_hi = (F_LO, F_HI) if st["t_seat"] is None else (F_LO_SEAT, F_HI_SEAT)
            if fm > f_hi:
                st["relax"] = min(st["relax"] + RELAX_RATE, RELAX_OPEN_MAX)
            elif fm < f_lo:
                st["relax"] = max(st["relax"] - RELAX_RATE, -RELAX_CLOSE_MAX)
            fing = F["grip"] + st["relax"]
        else:
            fing = st["f"]
            if ph in ("return", "retreat", "still"):
                st["relax"] = 0.0
        xy_cmd = st["xy"] + st["xy_off"]
        rig.drive_arm(np.array([xy_cmd[0], xy_cmd[1], st["z"]]), down_grasp_quat(st["yaw"]),
                      fingers=float(np.clip(fing, 0.0, 0.04)))
        rig.solver.probe = st["probe"]

    def extra(self, state, t):
        st = self.st
        row = state.body_q.numpy()[self.bi]
        tp = self.tap()
        fL, fR = tp["pad_N"]
        rc = float(self.rig.probes["robot_object"].history[-1]["force"]) \
            if self.rig.probes["robot_object"].history else 0.0
        Z = float(row[2] - self.S.fixture_world[2, 3])
        tilt = float(np.degrees(np.arccos(np.clip(rotm(row)[2, 2], -1.0, 1.0))))
        stand = float(self.rig.probes["standpair"].history[-1]["force"]) \
            if "standpair" in self.rig.probes and self.rig.probes["standpair"].history else 0.0
        self.log.append((t, Z, self.part_yaw.total, self.seat_gap(row), fL, fR, tp["force"], float(tp["flank"][2]),
                         rc, tp["seat"], tp["tau_pads"], st["probe"], tilt, float(row[0]), float(row[1]),
                         stand, float(np.linalg.norm(self.align_point(row)[:2] - self.S.axis_xy)),
                         float(self.tcp_meas()[2]), st["ax_trim"], self.omega_f, self.wrist_yaw_meas(),
                         st["yaw"]))
        return {"part_Z_mm": round(Z * 1000, 3), "part_turns": round(self.part_yaw.total / (2 * np.pi), 4),
                "seat_gap_mm": round(self.seat_gap(row) * 1000, 3), "thread_N": round(tp["force"], 3),
                "seat_N": round(tp["seat"], 3), "pad_torque_mNm": round(tp["tau_pads"] * 1000, 2),
                "probe_mNm": round(st["probe"] * 1000, 2), "robot_object_contact_N": round(rc, 4),
                "tilt_deg": round(tilt, 3), "phase": st["phase"]}


LOG_COLS = ("t", "Z", "yaw", "seat_gap", "fL", "fR", "thread", "flank_z", "robot", "seat", "tau_pads", "probe",
            "tilt", "x", "y", "standN", "axis_err", "tcp_z", "ax_trim", "omega", "wrist_meas", "wrist_cmd")


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# run + verdict
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
@dataclass
class Task:
    kind: str           # "bulb" | "cap"
    title: str          # paper task title
    number: int
    base_name: str      # output name of the default row
    defaults: dict      # default part options


TASKS = {
    "bulb": Task("bulb", "Install light bulb", 31, "install_light_bulb",
                 {"part": "lightbulb-1", "size": "55mm", "rev": "2"}),
    "cap": Task("cap", "Put bottle cap onto bottle", 19, "put_bottle_cap_onto_bottle",
                {"part": "bottle-1", "size": "", "rev": "2"}),
}
CAM = {"bulb": {"hand": (np.array([0.235, -0.345, 0.185]), np.array([0.0, 0.0, 0.070])),
                "table": (np.array([0.300, -0.400, 0.210]), np.array([0.050, 0.012, 0.070]))},
       "cap": {"hand": (np.array([0.200, -0.300, 0.175]), np.array([0.0, 0.0, 0.078]))}}
# visual-only shapes, coloured AFTER finalize (the studio palette recolours moving bodies inside finalize)
VIS_COL = {"stand_visual": (0.93, 0.93, 0.90)}


def _rev_str(r: str) -> str:
    return r[:-2] if r.endswith(".0") else r


def row_name(kind: str, mode: str, part: str, size: str, rev: str) -> str:
    T = TASKS[kind]
    name = T.base_name
    if kind == "bulb" and (part != T.defaults["part"] or size != T.defaults["size"]):
        name += f"_{part.split('-')[1]}_{size}"
    if kind == "cap" and part != T.defaults["part"]:
        name += f"_{part.split('-')[1]}"
    if rev != T.defaults["rev"]:
        name += f"_{rev.replace('.', 'p')}rev"
    if mode == "table":
        name += "_from_table"
    return name


def parse(kind: str, mode: str, argv=None):
    T = TASKS[kind]
    ap = argparse.ArgumentParser(description=f"{T.title} on the kit ({mode} start)")
    if kind == "bulb":
        ap.add_argument("--bulb", dest="part", default=T.defaults["part"],
                        choices=["lightbulb-1", "lightbulb-2", "lightbulb-3"])
        ap.add_argument("--size", default=T.defaults["size"], help="globe diameter, e.g. 55mm")
    else:
        ap.add_argument("--bottle", dest="part", default=T.defaults["part"],
                        choices=["bottle-1", "bottle-2", "bottle-3"])
        ap.add_argument("--size", default="", help=argparse.SUPPRESS)
    ap.add_argument("--rev", default=T.defaults["rev"], choices=["2", "2.5", "3", "3.5", "4"],
                    help="the kit's 'amount of rotation to tighten'")
    ap.add_argument("--yaw0-deg", type=float, default=YAW0_DEG, help="start yaw of the part (thread phase)")
    if mode == "hand":
        ap.add_argument("--seed", type=int, default=None,
                        help="random fixture position (the paper's 0.15 x 0.15 m square; the part starts above it)")
    ap.add_argument("--name", type=str, default=None, help="output folder name")
    ap.add_argument("--check", action="store_true", help="build, print the measured pair and IK checks, exit")
    return ap.parse_args(argv)


def _fingers(P: Pair) -> dict:
    touch = P.grip_r + PAD_OFF
    return {"touch": touch, "grip": touch - GRIP_OVERDRIVE, "open": min(touch + OPEN_CLEAR, FINGER_MAX)}


def setup(kind: str, mode: str, a, name: str):
    """Build the rig, scene, solver and probes; returns (rig, S, P, F, tcp0)."""
    P = measure_pair(kind, a.part, a.size, a.rev)
    F = _fingers(P)
    print(f"[{kind}] {name}: {P.row_variant}; lead {P.lead * 1e3:.3f} mm/turn, seat lift {P.seat_lift * 1e3:.3f} mm, "
          f"grip r {P.grip_r * 1e3:.2f} mm at z {P.grip_z * 1e3:.1f} (contact z {P.grip_contact_z * 1e3:.1f}), "
          f"palm clearance {P.palm_clear * 1e3:.1f} mm, fingers touch/grip/open "
          f"{F['touch'] * 1e3:.2f}/{F['grip'] * 1e3:.2f}/{F['open'] * 1e3:.2f} mm", flush=True)
    rig = appendix_rig(name)
    rig.substeps = SUBSTEPS
    rig.collide_every_substep = True        # thread + rotating fingertip contact
    base_xf = wp.transform(wp.vec3(*ROBOT_BASE), wp.quat_identity())
    rig.add_robot(base_xf)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        set_body_mass(rig.builder, bi, FINGER_MASS)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU
    # the imported robot and the ground come with newton's 0.1 m detection gap, which pairs every fingertip box
    # with every triangle of the threaded meshes and overflows the narrow-phase buffer
    for i, lbl in enumerate(rig.builder.shape_label):
        bi_ = rig.builder.shape_body[i]
        if (bi_ >= 0 and str(rig.builder.body_label[bi_]).startswith("fr3/")) or (lbl and "ground" in str(lbl)):
            rig.builder.shape_gap[i] = ROBOT_GAP
    seed = getattr(a, "seed", None)
    axis_xy = np.array(sample_start({"fixture": AXIS_PLACE}, seed)["fixture"][:2])
    S = build_scene(rig, P, mode, np.radians(a.yaw0_deg), axis_xy)
    S.seed = seed
    # ── arm start ──────────────────────────────────────────────────────────────────────────────────────────
    grip0 = S.p0 + rotm(np.r_[S.p0, S.q0]) @ np.array([0.0, 0.0, P.grip_z])
    if mode == "hand":
        tcp0 = grip0.copy()
        f0, f_tgt = F["touch"] - 0.0001, F["grip"]
    else:
        tcp0 = grip0 + np.array([0.0, 0.0, APPROACH_H + 0.030])
        f0 = f_tgt = F["open"]
    # IK check over the wrist window at the start and the seated height
    seat_tcp_z = S.fixture_world[2, 3] + P.seat_lift + P.grip_z
    for zc in (tcp0[2], seat_tcp_z):
        for yw in (YAW_HI, 0.0, YAW_LO):
            q = rig.ik.solve(np.array([S.axis_xy[0], S.axis_xy[1], zc]), down_grasp_quat(yw), iterations=64)
            err = rig.ik.tcp_pose()[:3] - np.array([S.axis_xy[0], S.axis_xy[1], zc]) if hasattr(rig.ik, "tcp_pose") \
                else np.zeros(3)
            print(f"[{kind}] IK z={zc * 1e3:6.1f} mm yaw={np.degrees(yw):+5.0f}: residual "
                  f"{np.linalg.norm(err) * 1e3:.2f} mm, q7 {np.degrees(q[6]):+.0f} deg")
    rig.solve_arm_start(tcp0, down_grasp_quat(YAW_HI))
    rig.builder.joint_q[7:9] = [f0, f0]
    rig.builder.joint_target_q[7:9] = [f_tgt, f_tgt]
    cam_pos, cam_tgt = CAM[kind][mode]
    shift = np.r_[S.axis_xy - AXIS_XY, 0.0]          # a seeded start: the camera keeps its view of the fixture
    cam_pos, cam_tgt = cam_pos + shift, cam_tgt + shift
    # a contact sensor must exist when finalize() creates the contacts, or contacts.force is never allocated
    rig.finalize(probes=[P.kind], cam_pos=cam_pos, cam_target=cam_tgt)
    rig.solver = make_solver(rig.model, iterations=ITERS, impratio=IMPRATIO)
    sc = rig.model.shape_color.numpy()
    for lbl, col in VIS_COL.items():
        for sid in rig.shapes.get(lbl, []):
            sc[sid] = col
    rig.model.shape_color.assign(sc)
    fingers = {k: [int(s) for s in np.flatnonzero(rig.model.shape_body.numpy() == rig.body_index(b))]
               for k, b in (("finL", "fr3/fr3_leftfinger"), ("finR", "fr3/fr3_rightfinger"))}
    seat_z_w = S.fixture_world[2, 3] + P.fixture_seat_z
    rig.probes = {
        "tap": ContactTap(rig.model, S.part, S.part_shapes, S.fixture_shapes, fingers, seat_z_w, P.seat_r,
                          S.axis_xy),
        "robot_object": RobotContactProbe(rig.model, [P.kind], static_shapes=S.static_labels),
    }
    if mode == "table":
        rig.probes["standpair"] = PairProbe(rig.model, P.kind, counterpart_shapes=["stand"])
    cj = int(np.flatnonzero(rig.model.joint_child.numpy() == S.part)[0])
    rig.solver = ProbeTorque(rig.solver, int(rig.model.joint_qd_start.numpy()[cj]) + 5)
    return rig, S, P, F, tcp0


def preroll(rig, S, F, tcp0, kind):
    """Unrecorded pre-roll for the in-hand start: the pads close on the part (plug_in_usb / ethernet pattern)."""
    rig.drive_arm(tcp0, down_grasp_quat(YAW_HI), fingers=F["grip"])
    sim_dt = 1.0 / rig.fps / rig.substeps
    for _ in range(int(T_PREROLL * rig.fps)):
        for _ in range(rig.substeps):
            rig.s0.clear_forces()
            rig.pipeline.collide(rig.s0, rig.contacts)
            rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
            rig.s0, rig.s1 = rig.s1, rig.s0
    rig.solver.update_contacts(rig.contacts, rig.s0)
    for p in rig.probes.values():
        p.sample(rig.contacts, rig.s0, 0.0)
    print(f"[{kind}] pre-roll: pads {rig.probes['tap'].history[-1]['pad_N']} N, part dz "
          f"{(float(rig.s0.body_q.numpy()[S.part][2]) - S.p0[2]) * 1000:+.3f} mm", flush=True)


def run(kind: str, mode: str, argv=None):
    a = parse(kind, mode, argv)
    name = a.name or row_name(kind, mode, a.part, a.size, a.rev)
    rig, S, P, F, tcp0 = setup(kind, mode, a, name)
    if a.check:
        print(json.dumps({k: (v if not isinstance(v, float) else round(v * 1e3, 3)) for k, v in P.__dict__.items()
                          if not isinstance(v, (dict, list))}, indent=1, default=str))
        print("mass", S.mass, "shapes", rig.model.shape_count)
        return
    # ── unrecorded pre-roll (in hand: the pads close on the part) ──────────────────────────────────────────
    if mode == "hand":
        preroll(rig, S, F, tcp0, kind)
    ex = ScrewExpert(rig, S, mode, tcp0, YAW_HI, F)
    import signal                            # iteration aid: `kill -USR1 <pid>` ends the rollout with a dump

    def _usr1(*_):
        ScrewExpert.stop_requested = True
    signal.signal(signal.SIGUSR1, _usr1)
    try:
        rig.run(150.0, ex.control, ex.extra)
    except StopRun:
        print(f"[{kind}] rollout stopped at t={len(rig.rec.times) / rig.fps:.2f}s")
    paths = rig.rec.save("rollout", force_keys=["thread_N", "seat_N", "pad_torque_mNm", "robot_object_contact_N",
                                                "probe_mNm"])
    print("saved:", paths)
    if (os.environ.get("KT_STOP_AFTER") or ScrewExpert.stop_requested) and ex.st["phase"] != "end":
        print(f"[{kind}] KT_STOP_AFTER: iteration run, no verdict")
        _dump(rig, ex)
        return
    verdict(rig, S, ex, mode, paths, TASKS[kind])


def _dump(rig, ex):
    np.save(rig.out_dir / "expert_log.npy", np.array(ex.log))
    (rig.out_dir / "expert_state.json").write_text(json.dumps(
        {"phases": ex.st["phases"], "strokes": ex.st["strokes"], "cols": LOG_COLS,
         "pair": {k: v for k, v in ex.P.__dict__.items() if not isinstance(v, (list,))}}, indent=1, default=str))


def verdict(rig, S: Scene, ex: ScrewExpert, mode, paths, T: Task):
    _dump(rig, ex)
    P, st = S.P, ex.st
    part = "bulb" if P.kind == "bulb" else "cap"
    L = np.array(ex.log)
    C = {c: L[:, i] for i, c in enumerate(LOG_COLS)}
    t = C["t"]
    qa = rig.qa(paths)
    crit = {}
    # ── start ────────────────────────────────────────────────────────────────────────────────────────────
    if mode == "hand":
        g0 = t < 0.5
        z0 = S.p0[2] - S.fixture_world[2, 3]
        crit["held_in_hand_at_start"] = {
            "pass": bool(min(C["fL"][g0].mean(), C["fR"][g0].mean()) > 2.0 and abs(C["Z"][g0].min() - z0) < 0.001),
            "mean_pad_N_first_0.5s": [round(float(C["fL"][g0].mean()), 2), round(float(C["fR"][g0].mean()), 2)],
            f"{part}_drop_mm": round(float(z0 - C["Z"][g0].min()) * 1000, 3),
            "start_low_point_above_fixture_top_mm": START_ABOVE * 1000}
    else:
        tl = st["t_lift"]
        lifted = (t > (tl or 1e9) + T_LIFT) & (t < (tl or 1e9) + T_LIFT + T_CARRY)
        rest = S.rest_z - S.fixture_world[2, 3]
        # lifted >= 30 mm off the stand, then carried clear of it (the carry ends at the in-hand start
        # height, 24 mm above the stand-rest level on the kit slot)
        peak = float((C["Z"][lifted] - rest).max()) if lifted.any() else 0.0
        rise = float((C["Z"][lifted] - rest).min()) if lifted.any() else 0.0
        support = float(C["standN"][lifted].max()) if lifted.any() else 1e9
        crit["picked_up_from_table"] = {
            "pass": bool(tl is not None and peak >= 0.030 and rise >= 0.010 and support < 0.05
                         and min(C["fL"][lifted].mean(), C["fR"][lifted].mean()) > 2.0),
            "lift_above_stand_rest_mm": round(peak * 1000, 1),
            "min_rise_above_stand_rest_during_carry_mm": round(rise * 1000, 1),
            "stand_contact_during_carry_peak_N": round(support, 4),
            "mean_pad_N_during_carry": [round(float(C["fL"][lifted].mean()), 2),
                                        round(float(C["fR"][lifted].mean()), 2)] if lifted.any() else None,
            "note": "the bulb starts upright on a printed stand; lifted by the pinch alone and carried to the socket"}
    # ── threaded in ──────────────────────────────────────────────────────────────────────────────────────
    eng, tt, ts = st["t_engaged"], st["t_tight"], st["t_seat"]
    lead_fit = turns = None
    if eng is not None and tt is not None:
        w = (t >= eng) & (t <= (ts or tt))
        if w.sum() > 20:
            lead_fit = abs(np.polyfit(C["yaw"][w], C["Z"][w], 1)[0]) * 2 * np.pi
        if st["by_touch"] is not None:
            turns = float(-(st["by_tight"] - st["by_touch"])) / (2 * np.pi)
    min_turns = MIN_TURNS_FRAC * P.revs
    travel = None if (st["z_touch"] is None or tt is None) else float(st["z_touch"] - C["Z"][t <= tt][-1])
    crit[f"{part}_threaded_{'in' if part == 'bulb' else 'on'}"] = {
        "pass": bool(eng is not None and lead_fit is not None and abs(lead_fit - P.lead) <= LEAD_TOL * P.lead
                     and turns is not None and turns >= min_turns),
        "first_thread_contact_Z_mm": None if st["z_touch"] is None else round(st["z_touch"] * 1000, 2),
        "engaged_at_t_s": None if eng is None else round(eng, 2),
        "turns_first_contact_to_tight": None if turns is None else round(turns, 3),
        "min_turns": round(min_turns, 2), "kit_revolutions": P.revs,
        "measured_lead_mm_per_turn": None if lead_fit is None else round(lead_fit * 1000, 3),
        "cad_lead_mm_per_turn": round(P.lead * 1000, 3),
        "axial_travel_first_contact_to_tight_mm": None if travel is None else round(travel * 1000, 2),
        "lead_fit_window": "engagement -> seat contact (pure thread travel)",
        "engage_strokes": sum(1 for s_ in st["strokes"] if s_["kind"] == "engage"),
        "strokes": st["strokes"]}
    # ── seated and tightened ─────────────────────────────────────────────────────────────────────────────
    tp = st["t_probe"]
    pre = (t >= (tp or 1e9) - 0.4) & (t < (tp or 1e9))
    seat_rel = float(C["seat"][pre].mean()) if pre.any() else 0.0
    preload_rel = float(-C["flank_z"][pre].mean()) if pre.any() else 0.0
    gap_rel = float(C["seat_gap"][pre].mean()) if pre.any() else 1.0
    tilt_rel = float(C["tilt"][pre].mean()) if pre.any() else 99.0
    weight = S.mass * 9.81
    stall = st["rate_at_stop"]
    crit["seated_and_tightened"] = {
        "pass": bool(st["tight_reason"] == "seat torque" and gap_rel <= SEAT_GAP_MAX
                     and preload_rel >= PRELOAD_MIN and tilt_rel <= TILT_MAX_DEG
                     and stall is not None and abs(stall) < STALL_RATE),
        "stop_reason": st["tight_reason"],
        "definition": ("seat face in contact, the part stalled, the pad torque about its axis >= the tightening "
                       f"torque with the flanks pulling it onto the seat (>= {PRELOAD_TIGHT:.0f} N); after release: "
                       f"still seated and preloaded (>= {PRELOAD_MIN:.0f} N)"),
        "tightening_torque_at_stop_mNm": None if st["tau_at_stop"] is None else round(st["tau_at_stop"] * 1000, 1),
        "tightening_torque_target_mNm": TAU_TIGHT * 1000,
        "part_rate_at_stop_deg_s": None if stall is None else round(float(np.degrees(stall)), 2),
        "seat_force_at_stop_N": None if st["seat_at_stop"] is None else round(st["seat_at_stop"], 2),
        "rotation_seat_contact_to_tight_deg": None if (st["by_seat"] is None or st["by_tight"] is None)
        else round(float(np.degrees(-(st["by_tight"] - st["by_seat"]))), 1),
        "post_release_preload_checks": st["checks"],
        "re_tightened": st["retighten"],
        "seat_force_after_release_N": round(seat_rel, 2),
        "part_weight_N": round(weight, 3),
        "preload_after_release_N": round(preload_rel, 2),
        "preload_definition": "downward pull of the thread flanks on the part (the seat carries it + the weight)",
        "seat_gap_after_release_mm": round(gap_rel * 1000, 3),
        "max_seat_gap_mm": SEAT_GAP_MAX * 1000,
        "tilt_deg": round(tilt_rel, 3)}
    # ── stationary under a small torque ──────────────────────────────────────────────────────────────────
    moves = {}
    for k, nm in ((0, "loosen_way"), (1, "tighten_way")):
        w = (t >= (tp or 1e9) + k * T_PROBE) & (t < (tp or 1e9) + (k + 1) * T_PROBE)
        moves[nm] = {"rotation_deg": round(float(np.degrees(np.ptp(C["yaw"][w]))), 4) if w.any() else None,
                     "z_change_mm": round(float(np.ptp(C["Z"][w])) * 1000, 4) if w.any() else None}
    crit["stationary_under_small_torque"] = {
        "pass": bool(tp is not None and all(v["rotation_deg"] is not None and v["rotation_deg"] < STATIONARY_DEG
                                            for v in moves.values())),
        "probe_torque_mNm": PROBE_TAU * 1000, "probe_s_each_way": T_PROBE, "threshold_deg": STATIONARY_DEG,
        **moves, "note": f"applied to the {part} about its axis after the hand had left"}
    # ── real finger grasp ────────────────────────────────────────────────────────────────────────────────
    tw = np.zeros_like(t, bool)
    for s in st["strokes"]:
        if s["kind"] in ("engage", "twist"):
            tw |= (t >= s["t"][0]) & (t < s["t"][1])
    crit["real_finger_grasp"] = {
        "pass": bool(tw.any() and C["fL"][tw].mean() > 1.0 and C["fR"][tw].mean() > 1.0),
        "mean_pad_N_while_twisting": [round(float(C["fL"][tw].mean()), 2), round(float(C["fR"][tw].mean()), 2)]
        if tw.any() else None,
        "grip_radius_mm": round(P.grip_r * 1000, 2), "palm_clearance_mm": round(P.palm_clear * 1000, 1)}
    # ── released and clear ───────────────────────────────────────────────────────────────────────────────
    t_rel = st["t_release"] if st["t_release"] is not None else t[-1]
    t_clear = st["t_retreat_end"] if st["t_retreat_end"] is not None else t[-1]
    check_released_and_clear(qa, t, C["robot"], t_clear)
    rc = qa.report["checks"]["released_and_clear"]
    z_rel = float(C["tcp_z"][t >= t_rel][0]) if (t >= t_rel).any() else 0.0
    rise = float(C["tcp_z"][-1]) - z_rel
    rc["tcp_rise_after_release_mm"] = round(rise * 1000, 1)
    rc["min_rise_mm"] = 50.0
    q = np.asarray(qa.body_q)
    li = qa.labels.index("fr3/fr3_link7")
    tq = np.asarray(qa.t)
    wq = tq >= t_clear
    v_end = float(np.linalg.norm(np.diff(q[wq, li, :3], axis=0), axis=1).max() * rig.fps) if wq.sum() > 2 else 0.0
    rc["hand_peak_speed_final_window_mm_s"] = round(v_end * 1000, 3)
    rc["max_hand_speed_mm_s"] = 1.0
    rc["pass"] = bool(rc["pass"] and rise >= 0.050 and v_end < 0.001)
    crit["released_and_clear"] = rc
    # ── physics integrity ────────────────────────────────────────────────────────────────────────────────
    for k_, c in crit.items():
        qa.report["checks"].setdefault(k_, c)
    if P.kind == "bulb":
        v = S.free_vis.vertices
        sel = v[(v[:, 2] > P.thread_z[0] - 4 * MM) & (v[:, 2] < P.thread_z[1] + 2 * MM)]
    else:
        sel = S.free_vis.vertices
    fx_w = S.fixture_vis.copy()
    fx_w.apply_transform(S.fixture_world)
    qa.check_penetration_static(P.kind, trimesh.Trimesh(vertices=sel, process=False), fx_w, np.zeros(3),
                                f"{part}->{'socket' if part == 'bulb' else 'bottle'}", tol=PEN_TOL)
    shift = np.r_[S.axis_xy - AXIS_XY, 0.0]
    qa.check_envelope([P.kind], np.array([-0.10, -0.10, -0.002]) + shift, np.array([0.20, 0.15, 0.30]) + shift)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    if mode == "hand":
        crit["start"] = {"pass": True, "info_only": True, "seed": S.seed,
                         "axis_xy_mm": [round(float(v) / MM, 2) for v in S.axis_xy],
                         "range": {"xy_mm": round(AXIS_PLACE.dxy / MM, 1)},
                         "part_yaw0_deg": round(float(np.degrees(2 * np.arctan2(S.q0[2], S.q0[3]))), 2),
                         "note": f"the {'socket' if part == 'bulb' else 'bottle'}'s axis on the table (nominal "
                                 "(0, 0) mm); the part starts in the pinch above it" +
                                 ("" if S.seed is not None else "; nominal start (no seed)")}
    changes = [
        f"kit parts: {P.free_part} + {P.fixture_part} (assets/kit, finalized printable kit); "
        f"{P.row_variant}",
        "thread = real SDF helix contact of the published STLs (fine collider on the threaded ends, coarse on the "
        "rest; the visible part is the unsplit STL)",
        "'tightened' = seat face in contact + stalled + pad torque >= "
        f"{TAU_TIGHT * 1000:.0f} mN.m (the kit has no spring tab); the preload is what holds it after release",
        f"mass = kit print estimate {S.mass * 1000:.1f} g (walls + 20 % infill)",
        ("start: in the pinch, lowest point 4 mm above the fixture top (appendix start)" if mode == "hand" else
         "start: bulb upright on a printed stand 110 mm away (generated part, not in the kit); picked up and "
         "carried (user addition)"),
        f"probe: {PROBE_TAU * 1000:.0f} mN.m each way about the {part} axis after the hand left",
    ]
    if S.seed is not None:
        changes.append(f"seeded start {S.seed}: {'socket' if part == 'bulb' else 'bottle'} axis at "
                       f"({S.axis_xy[0] / MM:+.1f}, {S.axis_xy[1] / MM:+.1f}) mm (uniform in the paper's "
                       "0.15 x 0.15 m square), the part in the pinch above it")
    if P.kind == "bulb":
        changes.append(f"grasp on the globe (r {P.grip_r * 1000:.1f} mm): the FR3 palm cannot clear the globe "
                       "for a grasp on the neck")
    save_appendix_verdict(rig, task=T.title, source="new (kit parts)", spec="", criteria=crit,
                          physics_qa=report, changes=changes)
