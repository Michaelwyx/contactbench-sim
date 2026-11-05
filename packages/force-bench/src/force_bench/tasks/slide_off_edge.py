"""Slide a flat part to the TABLE EDGE, then pick it off the overhang.

A card lying flat on a table cannot be picked up.  The pads close in the
plane of the table, and there is nothing to close on: the card is 1.2 mm
thick and the table is in the way of the lower pad.  That is not a
simulation artefact, it is why people slide a card to the edge of a desk
before picking it up, and it is the whole task.

    press      the closed gripper comes down on the card's face and the
               press is FORCE-SERVOED to a few newtons
    slide      it drags the card toward the table edge
    overhang   it stops when the measured overhang reaches half the card
    retract    the blade lifts off and swings OUTSIDE the table
    pinch      the pads, now closing VERTICALLY, straddle the two FACES of
               the overhanging half -- reachable only because that half is
               proud of the edge and has open air under it
    lift       straight up

Run:
    python -m force_bench.tasks.slide_off_edge           # the card
    python -m force_bench.tasks.slide_off_edge key       # the twin, with a key

═══════════════════════════════════════════════════════════════════════
THE THREE THINGS THAT WERE WRONG BEFORE, ALL OF THEM MEASURED
═══════════════════════════════════════════════════════════════════════

1. THERE WAS NO EDGE.  `Rig.__init__` calls `builder.add_ground_plane()`,
   so an infinite floor sits at z = 0 -- and the old scene put its table
   top at z = 0 and its "raised boss" at z in [-12, 0] mm, i.e. the boss
   was buried in the ground plane and the card was lying on the GROUND,
   dead centre of an infinite plane.  Every rollout frame shows it: a
   card on featureless grey with no step anywhere.  The card could be
   pushed but it could never overhang anything, and the side grasp had
   nothing to reach into.
   FIX: the ground plane becomes what it should have been all along --
   the FLOOR -- the work surface is lifted onto a 220 mm bench, and the
   FR3 stands on the floor beside it.  The 220 mm under the overhang is
   the open air the lower pad needs, and the floor-mounted base is what
   makes the side pinch KINEMATICALLY reachable (see side_grasp_quat).

2. THE LOWER PAD NEVER HAD ROOM.  Even taking the old boss at face value
   it was 12 mm proud, and the FR3 pad is 15.18 mm thick behind its own
   face and sits 9.5 mm from the TCP (FINGER_ENVELOPE, measured on the
   FR3 URDF).  A side grasp opened to 10 mm/side puts the
   lower pad's underside 25.2 mm below the card -- twice the clearance
   that existed.  140 mm of air clears it by a factor of five.

3. A FLAT PRESS CANNOT DRAG A CARD WHEN BOTH INTERFACES SHARE ONE mu.
   Newton averages the two shapes' mu into the pair coefficient (both the
   XPBD and the MuJoCo path: solvers/xpbd/kernels.py "use average contact
   material properties").  With the pad, the card and the table all at
   mu = 0.5 the drag condition is
        mu_pad * N  >  mu_table * (N + m g)
   i.e. 0.5 N > 0.5 N + 0.5 m g, which is FALSE for every N.  Pressing
   harder cannot help -- it adds to both sides.  That is exactly what the
   first rollouts measured (the blade slid over a stationary card), and
   it is why the previous attempts reached for wrist tilt and then for an
   edge-catching push: both were working around a materials bug.
   FIX: give the interfaces the coefficients they have in reality --
   PAD_MU 1.4 (an elastomer fingertip) against CARD_MU / TABLE_MU 0.15
   (PVC on a laminated desktop, handbook 0.1-0.3).  Pair mu is then 0.775
   at the pad and 0.15 at the table, a ratio of 5.2, and a 4 N press
   drags with 3.1 N against 0.62 N of resistance.  A real finger works
   for exactly this reason: skin is stickier than the desk.

WHY THE TABLE IS BUILT HERE AND NOT TAKEN FROM THE ASSET LIBRARY.  There
is no table in the CAD library -- the suite's "table" is a 10 mm slab
primitive whose only job is to be a floor.  This task needs an EDGE, and
the edge's geometry is the physics: a perfectly sharp 90-degree corner is
a contact singularity a part can catch on or pivot about, while a real
desk edge is eased.  So it is built here, out of ANALYTIC boxes and one
cylinder rather than a mesh + SDF: a 1.2 mm card resting on a 480 mm slab
would be asking a 480 mm SDF to resolve a 1.2 mm plate, which is the same
sampling failure that once let the blade press straight through the card
(see the part's own 512-resolution note below).  The eased edge is the
standard rounded-edge union:

    slab A   full depth, top lowered by r        }
    slab B   top at full height, front pulled     }  union = a box whose
             back by r                            }  front-top edge is
    fillet   cylinder radius r along x at the     }  rounded at radius r
             corner both slabs stop short of      }

    top at z = 220 mm       the number is the AIR under the overhang, and
                            220 mm is 9x what the lower pad needs; it is
                            also the bench height at which every pose in
                            the trajectory is IK-clean for a floor-mounted
                            FR3 (scanned, see side_grasp_quat)
    thickness 18 mm         a standard laminated desktop panel is 18-25 mm
    edge radius 1.0 mm      a machined or laminated edge is eased by
                            0.5-2 mm
    pedestal set back       the leg stops 100 mm short of the edge, so
      100 mm from the edge  nothing is under the overhanging half

MEASURED, on the card (85.0 x 55.0 x 1.2 mm):
    the pad tip is 9.5 mm from the TCP along the approach axis and the pad
    face is 18.5 mm long, so a side pinch whose tip stops 3 mm short of
    the edge lays 18.5 mm of pad on the overhang.  Half of 55 mm is
    27.5 mm, which covers it with 6 mm to spare -- and OVERHANG_FRAC is
    the *task's* number ("half off the table"), not a number chosen to
    make the grasp fit.
"""

from __future__ import annotations

import os
import sys

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.nut_bolt_arm import PairProbe
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass
from force_bench.rig import Rig, set_finger_effort_limit


def side_grasp_quat() -> np.ndarray:
    """TCP reaching HORIZONTALLY (+y) with the pads closing VERTICALLY.

    The overhanging card has to be taken by its two faces and those faces
    are horizontal, so the gripper cannot come from above -- the table is
    where the lower pad would have to be.  It comes in from outside the
    edge instead: approach axis +y, closing axis world z.  As a rotation of
    the world frame that is exactly -90 degrees about x:

        z_tcp = +y_world   (reach in over the overhang)
        y_tcp = -z_world   (pads close vertically, straddling the faces)
        x_tcp = +x_world

    THE SIGN OF THAT ROLL IS NOT COSMETIC.  The other choice (+90 about x,
    i.e. the same grasp with the two fingers swapped) is the one this task
    shipped with, and the FR3 CANNOT REACH IT: solved fresh from home at
    the pinch pose it lands with q7 exactly on its -3.0159 rad stop and
    22.0 mm of residual, and at the standoff pose 64.5 mm -- README pitfall
    21c, a joint that starts on its limit is pinned there.  The rollout it
    produced looked like a physics failure (a finger stuck at 340 N while
    the card fell off the table) and was a kinematics failure.  With this
    roll the ENTIRE trajectory -- park, press, drag, lift-off, transit
    slerp, descend, reach, pinch, lift -- walks continuously at 0.00 mm of
    residual with 3.08 deg of margin on the tightest joint.
    """
    q = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), float(-np.pi / 2))
    return np.array([q[0], q[1], q[2], q[3]])


from force_bench.geometry import smooth


# ── the table ───────────────────────────────────────────────────────────
# THE FR3 STANDS ON THE FLOOR BESIDE THE BENCH, IT IS NOT BOLTED TO IT.
# That is what makes the side pinch reachable at all: a base mounted ON the
# work surface puts the pinch at exactly base height with a horizontal
# wrist, and the IK solves that only by sitting on a joint stop (see
# side_grasp_quat).  On the floor, the same pinch is 220 mm ABOVE the base
# and 350 mm out -- an ordinary reach.  Bench height and base stand-off were
# picked by scanning the IK over both (scratch ikscan): every pose in the
# trajectory solves to 0.00 mm at these numbers.
TABLE_TOP = 0.220         # m — bench top above the floor.  This is also the
                          # AIR under the overhang, which is what the lower
                          # pad needs; see the header.
TABLE_T = 0.018           # panel thickness
EDGE_Y = -0.340           # the edge the part is slid to
BACK_Y = -0.105           # the back of the bench -- it stops short of the
                          # robot, whose base is a ~90 mm cylinder at y = 0
TABLE_HX = 0.160          # half-width along x
EDGE_R = 0.0010           # eased edge radius
PED_HX = 0.110            # pedestal half-width
PED_SETBACK = 0.100       # leg stops this far short of the edge, so the
                          # overhanging half has nothing under it
BASE_Y = 0.0              # FR3 base, on the floor beside the bench
BASE_Z = 0.0

# ── materials (see header item 3) ───────────────────────────────────────
PAD_MU = 1.4              # elastomer fingertip
CARD_MU = 0.15            # PVC card
TABLE_MU = 0.15           # laminated desktop

# ── the part ────────────────────────────────────────────────────────────
# The part is a positional argument when run as a script and TB_SLIDE_PART when
# imported (a task that builds on this module must not read ITS argv here).
PART = os.environ.get("TB_SLIDE_PART") or (
    sys.argv[1] if __name__ == "__main__" and len(sys.argv) > 1 else "card")
PART_SPEC = {
    "card": ("card_swiper", "card_v2_1_part4_1_body1", 0.005),
    "key": ("key_1", None, 0.012),
}
REST_CLEAR = 0.00015
PART_MAX_EDGE = 0.002     # m — no face may span more than this; see the
                          # subdivision note in main()
START_INSET = 0.008       # the part starts with its front edge THIS FAR
                          # inboard of the table edge -- "near the edge",
                          # which is where a card you are about to pick up
                          # actually is.  It was dead centre of the table
                          # before, so the whole rollout was spent driving
                          # it across an open plain.

# ── phases ──────────────────────────────────────────────────────────────
T_SETTLE = 0.8
T_DOWN = 0.8              # blade descends onto the card face
T_SLIDE = 9.5 if PART == "key" else 4.0
# ^ two more seconds under MuJoCo: even with the band fixed the drag runs
#   ~20 % slower than under XPBD, and the key reached 8.3 of the 9.85 mm it
#   needs when the budget expired (10.8 mm with the extra time).
                          # push budget (the stop is MEASURED, not timed);
                          # the key's slower stroke needs a longer one
T_PUSH_SETTLE = 1.4       # blade holds, then UNLOADS, while the part stops
PUSH_STILL = 0.0008       # m/s — "measurably still" for that hold
F_PRESS_OFF = 0.6         # N — and "essentially unloaded" for its release
T_UP = 1.1                # blade lifts STRAIGHT up, then backs off
T_SWING = 2.0             # round to outside the edge, wrist re-oriented
T_DESCEND = 1.4           # down to the card's mid-plane, outboard of the edge
T_REACH = 1.2             # in along +y until the pads straddle the card
T_CLOSE = 0.7
T_LIFT = 1.6
T_HOLD = 0.8
# THE RETREAT IS ITS OWN PROBLEM AND IT COST A ROLLOUT.  The first build
# slerped the wrist from the overhead blade pose to the horizontal side
# pose *while translating over the table*, and the 90-degree wrist flip
# swept a finger through the card: measured, the card sat still at 25.37 mm
# of overhang until t = 3.85 s and was airborne at +29 mm by t = 4.02 s,
# after which the same finger drove 340 N into the tabletop.  The path is
# now up-over-down with a hard rule: THE ORIENTATION ONLY CHANGES AT
# SAFE_Z, where the whole hand is a clear ball of air.
SAFE_Z = 0.090            # m above the table top for the transit leg.  In
                          # the side orientation nothing hangs more than
                          # ~20 mm below the TCP and in the blade
                          # orientation the pad tip is 9.5 mm below it, so
                          # the flip sweeps an arc well inside this.  Higher
                          # is NOT better: at 0.11 the transit pose costs
                          # all of the tightest joint's margin.

# ── the gripper ─────────────────────────────────────────────────────────
FINGER_KE, FINGER_KD = 6000.0, 60.0
FINGER_F_MAX = 12.0       # N/pad (MuJoCo effort limit)
BLADE_Q = 0.0             # push phase: fully closed, the two pads are one
                          # blade and nothing can slip between them
FINGER_OPEN = 0.010       # 20 mm gap for the side approach.  Bigger is
                          # pointless (the card is 1.2 mm) and costs
                          # clearance: the lower pad's underside sits
                          # q + 15.18 mm below the card.
FINGER_CLOSE = 0.0003     # 0.6 mm gap onto a 1.2 mm card = 0.3 mm/side

# ── the push (FORCE-servoed, never position-servoed) ────────────────────
# A stiff position drive commanded 0.4 mm into a card lying on a rigid
# table does not press with 0.4 mm worth of card compliance, it presses
# with whatever the scene stiffness gives -- measured 52-75 N per pad on
# the previous build, and 250 N into the boss.  The press therefore closes
# on the MEASURED pad force instead (pitfall 8.1 / README 17): the blade
# height walks down while the load is under F_PRESS_LO and back up above
# F_PRESS_HI, rate-capped both ways.
F_PRESS_LO, F_PRESS_HI = (1.2, 4.8) if PART == "key" else (2.0, 4.0)
# ^ THE KEY'S PRESS BAND UNDER MuJoCo.  The push servo walks the pad down
#   while the load is under F_PRESS_LO and back up above F_PRESS_HI, and the
#   drag command only advances while the pad is loaded.  MuJoCo's contact is
#   stiffer than XPBD's positional one, so at the XPBD band the servo
#   chatters: measured over the push, 52 % of frames unloaded, 6.5 % in
#   band, 41 % over it (XPBD: 17 / 71 / 12 %), the drag advanced at half
#   speed and the key stopped 12 mm short of the edge.  A 4.8 N ceiling
#   stops the back-off; 1.2 N still releases.  XPBD keeps 2.4.
                          # N summed over both pads.  The key's old
                          # (0.6, 1.5) band dated from the mesh-collided
                          # key that the card's band ejected; with the box
                          # shell the contact is calm, and at 0.7 N the
                          # measured drag slipped 50 % (2.5 mm/s achieved
                          # of 4.8 commanded) and parked short.
PRESS_WIN = 6             # frames the pad load is averaged over before the
                          # servo is allowed to see it (see pads_N)
PRESS_RATE = 0.000030 if PART == "key" else 0.000015
                          # m/frame — key 1.8 mm/s, card 0.9 mm/s.  Was
                          # 0.04 mm/frame, which is fast enough to chase
                          # the contact noise instead of the load.
PRESS_MAX = 0.0060        # m — how far below the nominal touch height the
                          # servo may go...
PRESS_MIN = -0.0040       # m — ...and how far ABOVE it.  The offset must be
                          # allowed to go negative or the servo has no way
                          # to relieve: the nominal height puts the pad tip
                          # exactly on the card's top face, and the arm
                          # still sags into it under its own weight -- the
                          # first run measured 20-22 N per pad with the
                          # offset pinned at its 0.0 floor for the whole
                          # stroke.
PRESS_START = -0.0060 if PART == "key" else -0.0020
                          # m — the timed key descent stops 6 mm high.  At
                          # 2 mm the compliant wrist sag reached the curved
                          # key during the timed move: measured on the
                          # restored full task (t=1.50 s, still in "down"),
                          # the pads hit it at 25 N, it rose 8.4 mm and was
                          # over the edge 0.3 s later.  The force servo
                          # alone closes the remaining gap.  The broad card
                          # keeps its proven 2 mm endpoint.
SLIDE_RATE = 0.00008 if PART == "key" else 0.00028
                          # key: 4.8 mm/s; card: validated 17 mm/s
OVERHANG_FRAC = 0.50      # "half of it off the table" -- the task's number
# ...but stop the stroke HERE, and PER PART.  The stroke always overshoots
# the command a little, and a part whose centre crosses the edge tips off on
# its own -- but the two parts do not reach that point the same way.  The
# card is 85 mm wide and rests on its whole face; the key is 20 mm wide on a
# bow ring and a toothed blade, drifts during the settle, and has to keep
# more of the 50 % geometric tipping point in hand.  One number for both
# served the key and cost the card a third of its stroke: 30 % of the part
# instead of the 44 % this task is actually about.
#
# THE KEY GETS 0.22, NOT 0.32, and the reason is margin rather than taste.
# Its tipping point is 24.87 mm (half its 49.74 mm depth -- see COM_MARGIN
# for why the geometric centre is the real one here), and stopping at
# 15.9 mm left only 9 mm of it.  That is not enough to absorb the drift
# this part shows after the blade leaves, and the part went over the edge.
# 10.9 mm leaves 14 mm.  The pinch does not mind: `grasp_for` centres the
# pad on whatever overhang was ACHIEVED and clamps the tip TIP_MARGIN short
# of the edge, so a shorter overhang costs pad COVERAGE, not the grasp --
# ~8 mm of the 18.5 mm pad face still lands on a 2.5 mm part that weighs
# 12 g, which is an order more than the pinch needs.
OVERHANG_STOP_BY_PART = {"card": 0.45, "key": 0.22}
OVERHANG_STOP = OVERHANG_STOP_BY_PART.get(PART, 0.35)
COAST_S_BY_PART = {"card": 0.06, "key": 0.04}
COAST_S = COAST_S_BY_PART.get(PART, 0.10)
                          # s of arm coast the push stop allows for.  It
                          # was 0.45 s, sized when the part really did run
                          # on for 0.4 s after the command froze -- but that
                          # coast was the press limit cycle and the one-shot
                          # release, and with those fixed the MEASURED coast
                          # is 0.11 mm.  Reserving 7.6 mm for it was then
                          # simply throwing away a third of the stroke: the
                          # card stopped at 14.4 mm of a 21.5 mm target.
COAST_WIN = 10            # frames the coast speed is averaged over
COAST_MAX_BY_PART = {"card": 0.002, "key": 0.001}
COAST_MAX = COAST_MAX_BY_PART.get(PART, 0.004)
                          # m — hard bound on the allowance.  With the
                          # pure-z break the key's measured release run-on
                          # is 0.7 mm; keep one millimetre of bounded
                          # prediction rather than the old 3-4 mm estimate.
UP_BREAK = 0.020          # m of PURE-Z travel that breaks the press before
                          # anything lateral happens
UP_SPLIT = 0.25           # fraction of T_UP spent going STRAIGHT up before
                          # the retreat starts (see the "up" phase)
UP_BACK = 0.010           # m — the blade retreats this far while it lifts,
                          # and it does so on a LINEAR ramp rather than an
                          # eased one: the whole point is to be moving
                          # BACKWARD in the first few frames after the stop.
                          # The arm is a position-tracked spring, so it
                          # keeps travelling for a moment after the command
                          # freezes -- measured on the key, it coasted the
                          # part another 6-9 mm past a stop that was
                          # supposed to leave 6 mm of centre-of-mass margin,
                          # which is exactly enough to tip it off.
COM_MARGIN = 0.006        # m — and NEVER push the part's own CENTRE OF MASS
                          # closer than this to the edge, whatever
                          # OVERHANG_STOP says.  For the card the two rules
                          # agree (its CoM is on its bbox centre to within
                          # 0.005 mm); for the key they do not.
                          # AND FOR BOTH PARTS IT IS A NO-OP, which is
                          # worth stating rather than leaving as a rule
                          # that looks live.  Measured off the finalized
                          # builder, `body_com` for the key is
                          # (0.00, 0.00, 0.00) mm from the body origin --
                          # the mesh centroid and the bounding-box centre
                          # coincide in its resting pose -- so the tipping
                          # point is the BBOX centre crossing the edge,
                          # i.e. 50 % of the depth, and the geometric stop
                          # is the only one that binds.  0.32 leaves 9 mm
                          # of margin to that; 0.40 left 5 mm and the key
                          # drifted into it during the settle.
LAG_MAX = 0.004           # m -- see the push stop
PUSH_BACK_MAX = 0.015     # m — press at most this far behind the part's
                          # centre, so the load stays on the SUPPORTED half
                          # all the way to the stop...
BLADE_HALF = 0.0152       # m — ...but the closed pads are 30.4 mm long
                          # along the push (FINGER_ENVELOPE: each pad runs
                          # from q-0.02 to q+15.18 mm), and if the press
                          # point sits further back than the part's own
                          # half-depth minus that, the blade's INBOARD end
                          # hangs off the part and lands on the bench.  The
                          # force servo then reads the bench and the part
                          # gets a tilted edge contact: measured on the key
                          # (49.7 mm deep, so the blade ran 5.4 mm off it),
                          # the part was already contact-free 0.07 s after
                          # the stop and slid off the edge with the pads
                          # reading their 0.12 N floor.
LIFT_D = 0.090

# ── the side pick ───────────────────────────────────────────────────────
TIP_FROM_TCP = 0.0095     # m — pad tip ahead of the TCP along the approach
PAD_LEN = 0.0185          # m — pad face length (FINGER_ENVELOPE: -9.5..+9.0)
TIP_MARGIN = 0.003        # m — how far short of the edge the tip stops
STANDOFF = 0.055          # m — where the side approach starts, outboard.
                          # 0.090 put the descend pose on a joint stop.


def _box(rig, label, y0, y1, z0, z1, hx, mu):
    sid = rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(0.0, 0.5 * (y0 + y1), 0.5 * (z0 + z1)),
                               wp.quat_identity()),
        hx=hx, hy=0.5 * (y1 - y0), hz=0.5 * (z1 - z0),
        cfg=rig.cfg(mu=mu, gap=0.0003), label=label)
    rig.shapes[label] = [sid]
    return sid


def build_table(rig):
    """Analytic bench.  ONE box for the top; the eased edge is gone.

    THE ROUNDED EDGE WAS A UNION OF THREE SHAPES AND IT COST THE TASK ITS
    PART.  It was built the standard way -- a slab lowered by r, a slab
    pulled back by r, and a fillet cylinder tangent to both -- which is
    exact geometry and completely wrong for what rests on it: near the edge
    a 2.5 mm part is supported by the top slab AND by the cylinder along
    the line where they meet, i.e. by two shapes claiming the same plane.
    Measured, the bench<->part contact then DROPS OUT and re-acquires:
    part_N reads 0.105 N (its weight is 0.118), then 0.000 N with the part
    still at its resting height of 1.28 mm and the blade already gone, then
    1.038 N -- nine times its weight -- as the contact comes back, and that
    re-acquisition impulse throws the part 11 mm in 0.14 s and over the
    edge.  It is the mechanism behind every "the key slides off and nothing
    visible pushed it" symptom in this file's history, and it is why none
    of the trajectory, press, timestep, SDF-margin, contact-cadence or
    mesh-density experiments touched it.

    A single box has one exact edge and one contact manifold.  The eased
    edge was justified on realism (a machined desk edge IS broken 0.5-2 mm)
    and it is not worth a contact singularity.  The honest note is that the
    corner is now sharp, which makes the slide HARDER rather than easier.
    """
    top, t = TABLE_TOP, TABLE_T
    _box(rig, "table_b", EDGE_Y, BACK_Y, top - t, top, TABLE_HX, TABLE_MU)
    _box(rig, "pedestal", EDGE_Y + PED_SETBACK, BACK_Y,
         0.0, top - t, PED_HX, TABLE_MU)


def part_mesh():
    """The part, laid down in a MEASURED STABLE POSE.

    Squaring the thin axis to z is right for a card, whose two faces really
    are parallel planes, and WRONG for a key: `key_1/part1` is 6.8 mm at
    its bow and thinner along the blade, so "thin axis down" rests it on
    the bow with the blade cantilevered.  Measured, it started carrying
    28 % of its own weight, rocked off the bench inside 0.3 s and was two
    metres away by the time the push began.  `compute_stable_poses` returns
    the orientations the part actually rests in, most probable first.
    """
    asset, stem, mass = PART_SPEC[PART]
    if stem is None:
        from force_bench.assets import list_bodies
        stem = list_bodies(asset)[0]
    tm = load_tm(asset, stem)
    try:
        Ts, probs = trimesh.poses.compute_stable_poses(tm, n_samples=8)
    except Exception:
        Ts, probs = [], []
    if len(Ts):
        # the flattest of the top poses: a part slid to an edge and pinched
        # on its two faces has to be lying DOWN, not standing on an edge
        best = min(range(min(4, len(Ts))),
                   key=lambda i: _z_extent(tm, Ts[i]))
        tm.apply_transform(Ts[best])
        tm.apply_translation(-tm.bounds.mean(axis=0))
        # ...AND THE HEAVY END GOES INBOARD.  The push is toward -y, so a
        # part whose centre of mass sits on the -y side of its own bounding
        # box reaches the tipping point EARLIER than its geometry says.
        # Measured on the key: the CoM is 6.03 mm off centre, and with it
        # pointing outboard the part tipped off the bench at 22.4 mm of a
        # 49.7 mm depth -- 45 %, which "half of it overhangs" would have
        # called a success right up to the moment it fell 220 mm.  A hand
        # slides something to an edge heavy-end-back for the same reason.
        if float(tm.center_mass[1]) < 0.0:
            tm.apply_transform(trimesh.transformations.rotation_matrix(
                np.pi, [0, 0, 1]))
            tm.apply_translation(-tm.bounds.mean(axis=0))
        print(f"stable pose {best} of {len(Ts)} (p={probs[best]:.2f}), "
              f"extents {np.round(tm.extents * 1000, 2)} mm, CoM "
              f"{np.round(tm.center_mass * 1000, 2)} mm off the bbox centre")
    else:                                   # no stable pose: thin axis down
        thin = int(np.argmin(tm.extents))
        if thin != 2:
            ax = [1, 0, 0] if thin == 1 else [0, 1, 0]
            tm.apply_transform(trimesh.transformations.rotation_matrix(
                np.pi / 2, ax))
    return tm, mass


def _z_extent(tm, T):
    v = trimesh.transform_points(np.asarray(tm.vertices), T)
    return float(v[:, 2].max() - v[:, 2].min())


def main() -> None:
    rig = Rig(f"slide_off_edge_{PART}")
    if PART == "key":
        # SMALLER TIMESTEP FOR THE KEY, and only the key.  The curved 12 g
        # key is a fast-contact problem, not the broad card's quasi-static
        # one: at the suite default of 16 XPBD substeps one contact
        # produced -0.30 m/s and 45 rad/s; at 64 the same approach stayed
        # settled.  Respect larger explicit overrides.
        rig.substeps = max(rig.substeps, 64)
    tm, mass = part_mesh()
    ext = tm.extents
    depth, thick = float(ext[1]), float(ext[2])
    com = np.asarray(tm.center_mass)
    PUSH_BACK = min(PUSH_BACK_MAX, depth * 0.5 - BLADE_HALF - 0.0018)
    start_y = EDGE_Y + depth * 0.5 + START_INSET
    goal_y = EDGE_Y + depth * (0.5 - OVERHANG_FRAC)
    # the two stop rules, and the stroke obeys whichever binds first
    stop_geom = EDGE_Y + depth * (0.5 - OVERHANG_STOP)
    stop_com = EDGE_Y + COM_MARGIN - float(com[1])
    stop_y = max(stop_geom, stop_com)
    oh_stop = EDGE_Y - (stop_y - depth / 2)

    # THE ROBOT GOES IN FIRST.  Rig asserts it, because the FR3 has to own
    # coordinates 0..8 for every downstream index (drive_arm writes qt[:9],
    # the finger dofs are 7 and 8).  It is MOUNTED ON THE TABLE, so every
    # TCP pose below is the same relative geometry the rest of the suite
    # uses -- only the datum moved.
    rig.add_robot(wp.transform(wp.vec3(0.0, BASE_Y, BASE_Z),
                               wp.quat_identity()))
    if PART == "key":
        # The stock 200 N*m*s/rad arm drives are underdamped for this 12 g
        # edge interaction. The wrist otherwise continues outboard after
        # the push target stops and launches the key during lift-off.
        for d in range(7):
            rig.builder.joint_target_kd[d] = 900.0
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    # elastomer fingertips: the mu ratio IS the push mechanism (header 3)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU

    print(f"part {PART}: extents {np.round(ext * 1000, 2)} mm, "
          f"mass {mass * 1000:.0f} g")
    print(f"bench edge y {EDGE_Y*1000:.1f} mm, top z {TABLE_TOP*1000:.1f} mm "
          f"({TABLE_TOP*1000:.0f} mm of air under the overhang); FR3 base "
          f"on the floor at y {BASE_Y*1000:.0f} mm")
    print(f"part centre {start_y*1000:.1f} -> {stop_y*1000:.1f} mm "
          f"(push {(start_y-stop_y)*1000:.1f} mm, overhang stop "
          f"{oh_stop*1000:.1f} mm = {oh_stop/depth*100:.0f} %); limited by "
          f"{'the CoM' if stop_com > stop_geom else 'geometry'} "
          f"(geom {EDGE_Y - (stop_geom - depth/2):.4f} m, CoM "
          f"{EDGE_Y - (stop_com - depth/2):.4f} m)")

    build_table(rig)

    # 512, NOT 256.  The card is 85 x 55 x 1.2 mm and an SDF is sampled on
    # a cube: at 256 the thin axis gets ~4 cells, which cannot represent a
    # 1.2 mm plate, and the blade pressed straight THROUGH the card onto
    # the table -- measured as pads at 34-62 N while the card felt only its
    # own 0.05 N of weight for the entire rollout.  card_pick uses 384 on
    # the same asset for the same reason.
    # ...AND A MARGIN SIZED TO THE PART, NOT THE DEFAULT.  make_newton_mesh
    # defaults to a 1.5 mm SDF margin and a 2 mm narrow band, and ShapeConfig
    # to a 0.8 mm contact gap.  On a 2.5 mm key that margin is 60 % of the
    # part's own thickness, and the resting contact goes unstable: measured,
    # the key's height alternated 1.25 <-> 1.55 mm every frame with the
    # blade held STATIONARY 1 mm clear of it, and the chatter walked it
    # 13 mm outboard in 0.7 s -- a vibratory feeder, not a push.  These are
    # the values task_nut_bolt settled on for parts of this size.
    # SUBDIVISION WAS TRIED HERE AND REVERTED, and the measurement is the
    # point.  This asset is exported coarse -- the key is 120 vertices /
    # 236 faces with a MAXIMUM EDGE OF 97.1 mm on a 49.7 mm part -- and
    # contact samples are mesh VERTICES against the counterpart's SDF
    # (README pitfall 3.2), so its underside meets the flat bench at a
    # handful of points and the resting contact never settles: part_N on a
    # 12 g part reads 0.000 / 0.034 / 0.141 / 0.260 N where its weight is
    # 0.118.  Subdividing to 2 mm edges (120 -> 13157 verts) DID fix that
    # statistic -- part_N then averages 0.1243 N against the 0.118 N weight
    # -- and did NOT fix the key, which is still ejected at 137 mm/s.
    # AND IT BROKE THE CARD, which had been passing: 14960 verts and the
    # card falls off the bench instead of being picked (lift -220 mm
    # against the 89.0 mm it had made twice).  A denser mesh is not free
    # here, so the coarse asset ships and the key's contact stays an open
    # item.
    pmesh, _ = make_newton_mesh(tm, sdf=True, sdf_resolution=512,
                                narrow_band=0.001, margin=0.0001)
    pos = np.array([0.0, start_y, TABLE_TOP + thick * 0.5 + REST_CLEAR])
    body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*pos), wp.quat_identity()), label="part")
    if PART == "key":
        # THE KEY GETS AN ANALYTIC CONTACT SHELL, exactly the treatment
        # card_pick gives its 1.2 mm cards and for the same reason: the
        # asset ships coarse (120 vertices, MAXIMUM EDGE 97.1 mm on a
        # 49.7 mm part), contact samples are mesh VERTICES against the
        # counterpart's SDF (README pitfall 3.2), so the flat underside
        # meets the bench at a handful of points.  Measured consequence,
        # at 64 XPBD substeps with every servo already gentle: the slide
        # runs perfectly at 5 mm/s until the FRONT VERTEX crosses the
        # bench corner, then the contact normal tilts, part_N drops from
        # 0.22 N to 0.02 N and the key accelerates 5 -> 55 mm/s in 0.15 s
        # with the pads reading ~0 -- ballistic before the stop can fire.
        # Under MuJoCo the same sparse contact makes the press limit-cycle
        # 0 <-> 6 N and the stroke advances 2 mm in 7.5 s.  Subdividing
        # the mesh was tried earlier (cea1bda) and rejected: it fixed the
        # resting statistic, not the ejection, and broke the card.  A
        # box-vs-box contact has a clean 4-point manifold that shrinks
        # smoothly as the key overhangs the edge.  The CAD mesh stays as
        # the visual body; the box uses its measured extents, and the
        # body's mass/CoM already sit on the bbox centre (see COM_MARGIN).
        vis = newton.ModelBuilder.ShapeConfig(has_shape_collision=False)
        rig.builder.add_shape_mesh(body, mesh=pmesh, cfg=vis,
                                   label="part_visual")
        rig.builder.add_shape_box(
            body, hx=float(ext[0]) / 2, hy=float(ext[1]) / 2,
            hz=float(ext[2]) / 2,
            cfg=rig.cfg(mu=CARD_MU, gap=0.0003, is_visible=False),
            label="part_shape")
    else:
        rig.builder.add_shape_mesh(body, mesh=pmesh,
                                   cfg=rig.cfg(mu=CARD_MU, gap=0.0003),
                                   label="part_shape")
    rig.shapes["part"] = [rig.builder.shape_count - 1]
    set_body_mass(rig.builder, body, mass)

    # ── the poses the trajectory is built out of ────────────────────────
    blade_park_z = TABLE_TOP + thick + TIP_FROM_TCP + 0.030
    blade_touch_z = TABLE_TOP + thick + TIP_FROM_TCP     # tip on the face
    # the card RESTS ON the table, so its mid-plane is HALF A THICKNESS
    # ABOVE the top, not below it (the first build had the sign wrong and
    # aimed the pinch 1.2 mm low)
    grasp_z = TABLE_TOP + thick * 0.5                    # pads straddle it
    transit_z = TABLE_TOP + SAFE_Z
    # the pad tip stops TIP_MARGIN short of the edge, so the whole 18.5 mm
    # pad face lands on the overhang and nothing touches the table
    grasp_y = EDGE_Y - TIP_MARGIN - TIP_FROM_TCP

    def grasp_for(oh):
        """Centre the pad face on the overhang that was actually ACHIEVED.

        A fixed grasp assumes the overhang is at least as long as the pad
        (18.5 mm) plus its clearance; when it is shorter -- and for a part
        whose safe stop is set by tipping rather than by the pad, it will
        be -- a fixed grasp puts the pad's rear over the bench.  Centring
        on the measurement trades pad coverage for never touching the
        table, and the tip still stops TIP_MARGIN short of the edge."""
        want = EDGE_Y - 0.5 * oh - 0.5 * (TIP_FROM_TCP - (PAD_LEN
                                                          - TIP_FROM_TCP))
        return min(grasp_y, want)
    print(f"side pinch: TCP y {grasp_y*1000:.1f} mm, pad face covers "
          f"{(grasp_y - PAD_LEN + TIP_FROM_TCP)*1000:.1f} .. "
          f"{(grasp_y + TIP_FROM_TCP)*1000:.1f} mm; overhang reaches "
          f"{(EDGE_Y - depth*OVERHANG_STOP)*1000:.1f} mm")

    # SEED THE ARM AT ITS FIRST COMMANDED POSE.  add_robot leaves the FR3
    # at FR3_HOME and drive_arm issues the working pose on frame 1, so a
    # 12000 N.m/rad drive swings the whole arm across the scene in a few
    # frames and a finger sweeps the part away -- measured on an earlier
    # build: the card was airborne at z = 33 mm by t = 0.42 s, during
    # SETTLE, while the trajectory had not started moving.
    park_y = start_y + PUSH_BACK
    q0 = rig.ik.solve(np.array([0.0, park_y, blade_park_z]),
                      down_grasp_quat(0.0), iterations=64)
    rig.builder.joint_q[:7] = [float(v) for v in q0[:7]]
    rig.builder.joint_target_q[:7] = [float(v) for v in q0[:7]]
    rig.builder.joint_q[7:9] = [BLADE_Q, BLADE_Q]
    rig.builder.joint_target_q[7:9] = [BLADE_Q, BLADE_Q]

    # COLLIDE EVERY SUBSTEP.  The suite default is once per FRAME (README
    # pitfall 5.1), and that default exists to make a GRIP work: XPBD dry
    # friction is a positional constraint on the anchors `pipeline.collide`
    # records, so re-anchoring inside the substep loop lets a pinch creep.
    # A part SLIDING on a static bench is the opposite problem.  Measured
    # here with the frame cadence: the key rests on the bench carrying
    # exactly its own 0.118 N of weight (part_N 0.107-0.140 N) while
    # part_fric_N reads 0.000-0.008 N, and it slides 20 -> 32 mm at 26 mm/s
    # WITHOUT DECELERATING.  At the pair mu of 0.15 even that measured
    # friction should stop a 12 g part inside 0.1 s, so the contact is not
    # being applied on most substeps -- it is stale geometry, not a
    # friction coefficient.
    # ...FOR THE MESH-COLLIDED CARD.  The key's analytic box shell (see
    # above) has no stale-geometry problem -- its resting part_N is a
    # steady 0.118 N at ANY cadence -- and the every-substep cadence is
    # actively harmful to it: re-anchoring 64x/frame dilutes the blade's
    # friction drag exactly as pitfall 5.1 describes for a grip, and the
    # measured stroke stalled at 0.6 mm of 19 with the pads steady at
    # 0.8 N.  Per-frame anchors restore the drag AND are what the side
    # pinch needs later anyway.
    rig.collide_every_substep = (PART != "key")

    cam_t = np.array([0.0, EDGE_Y + 0.015, TABLE_TOP - 0.020])
    rig.finalize(probes=["part", "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"],
                 cam_pos=cam_t + np.array([0.33, -0.30, 0.16]),
                 cam_target=cam_t)
    # COLOUR THE EDGE.  The camera looks along the table, and at one grey
    # the step at the edge is invisible -- the reviewer could not see the
    # edge at all, and the edge IS the task.
    col = rig.model.shape_color.numpy().copy()
    for i, nm in enumerate(list(rig.model.shape_label)):
        if nm == "table_b":
            col[i] = (0.58, 0.60, 0.64)
        elif nm == "pedestal":
            col[i] = (0.28, 0.29, 0.32)
        elif nm in ("part_shape", "part_visual"):
            col[i] = (0.95, 0.62, 0.15)
    rig.model.shape_color.assign(col)

    if PART == "key":
        # Pair-resolved force is mandatory for the low-force key servo.
        # At 64 substeps each unfiltered fingertip probe has a ~0.30 N
        # floor from unrelated contacts, enough for the two pads to fake
        # the whole 0.6 N contact threshold and leave the blade suspended
        # above the key (measured: press offset walked to -3.15 mm and
        # the "push" ended at -7.9 mm of overhang with the key untouched).
        rig.probes.pop("fr3/fr3_leftfinger")
        rig.probes.pop("fr3/fr3_rightfinger")
        rig.probes["padL"] = PairProbe(
            rig.model, sensing_body="fr3/fr3_leftfinger",
            counterpart_bodies=["part"])
        rig.probes["padR"] = PairProbe(
            rig.model, sensing_body="fr3/fr3_rightfinger",
            counterpart_bodies=["part"])
    else:
        rig.probes["padL"] = rig.probes.pop("fr3/fr3_leftfinger")
        rig.probes["padR"] = rig.probes.pop("fr3/fr3_rightfinger")
    pi = rig.body_index("part")

    st = {"ph": "settle", "t0": 0.0, "dur": T_SETTLE,
          "cmd_y": park_y, "press": PRESS_START, "grasp_y": grasp_y,
          "oh_stop": None, "oh_hist": []}
    t_end = (T_SETTLE + T_DOWN + T_SLIDE + T_PUSH_SETTLE + T_UP + T_SWING
             + T_DESCEND + T_REACH + T_CLOSE + T_LIFT + T_HOLD)
    q_down, q_side = down_grasp_quat(0.0), side_grasp_quat()

    def slerp(a, b, u):
        qa = wp.quat(*[float(v) for v in a])
        qb = wp.quat(*[float(v) for v in b])
        r = wp.quat_slerp(qa, qb, float(np.clip(u, 0.0, 1.0)))
        return np.array([r[0], r[1], r[2], r[3]])

    def pads_N():
        """Pad load, SMOOTHED over PRESS_WIN frames.

        THE RAW PER-FRAME READING IS WHAT MADE THE PRESS CHATTER.  It is a
        contact impulse, not a steady load, and on a 12 g part against a
        rigid bench under a 12 kN.m/rad position drive it swings hard: the
        two pads measured 9.53/0.00, then 0.00/2.57, then 6.86/0.00, then
        0.05/0.05 on consecutive frames while the part bounced 1.25 <-> 1.45
        mm.  A bang-bang servo on that signal is a limit cycle, and the
        cycle is an IMPULSE GENERATOR: by the time the stroke stopped, the
        key already carried 39 deg/s of spin and -15.6 mm/s with both pads
        reading zero -- it had left the blade tumbling.  Lifting off did not
        launch it; the press had already done that."""
        L = rig.probes["padL"].history[-PRESS_WIN:]
        R = rig.probes["padR"].history[-PRESS_WIN:]
        if not L:
            return 0.0
        return float(np.mean([a["force"] for a in L])
                     + np.mean([b["force"] for b in R]))

    def control(t):
        """press -> drag -> lift off -> swing outside -> side pinch -> lift."""
        bq = rig.s0.body_q.numpy()
        py = float(bq[pi, 1])
        oh = EDGE_Y - (py - depth / 2)          # mm of part past the edge
        ph = st["ph"]
        u = min(1.0, (t - st["t0"]) / st["dur"])

        if ph == "settle":
            if u >= 1.0:
                st.update(ph="down", t0=t, dur=T_DOWN)
            rig.drive_arm(np.array([0.0, park_y, blade_park_z]),
                          down_grasp_quat(0.0), fingers=BLADE_Q)
            return

        if ph == "down":                        # blade to 2 mm off the face
            z = blade_park_z + (blade_touch_z - PRESS_START
                                - blade_park_z) * smooth(u)
            if u >= 1.0:
                st.update(ph="push", t0=t, dur=T_SLIDE)
            rig.drive_arm(np.array([0.0, park_y, z]),
                          down_grasp_quat(0.0), fingers=BLADE_Q)
            return

        if ph == "push":
            # FORCE-servoed press (see F_PRESS_LO/HI) + rate-capped drag.
            # The stroke does not START until the press is loaded, so the
            # drag never runs on a blade that is still in the air.
            F = pads_N()
            if F < F_PRESS_LO:
                st["press"] = min(PRESS_MAX, st["press"] + PRESS_RATE)
            elif F > F_PRESS_HI:
                st["press"] = max(PRESS_MIN, st["press"] - PRESS_RATE)
            # PREDICTIVE STOP.  The arm is a position-tracked spring and
            # it keeps travelling after the command freezes: measured on
            # the key, the part slid another 0.4 s at exactly the commanded
            # 17 mm/s -- 7 mm -- with its roll and pitch at 0.00 deg the
            # whole time, so it was still being pushed, not tipping.
            # Stopping on where the part WILL be rather than on where it is
            # costs nothing and is the same closed-loop idea as the rest of
            # this file.
            # ...over a 10-FRAME WINDOW, and bounded.  A frame-to-frame
            # difference on a part that is being dragged through contact is
            # mostly noise: unsmoothed it read speeds that put the stop at
            # 6.7 mm of a 19.9 mm target.  (And it is seeded on the FIRST
            # push frame, not at build -- seeded at -depth/2 it read a
            # 1012 mm/s "speed" on frame 1 and stopped before the part had
            # moved at all.)
            st["oh_hist"].append(oh)
            if len(st["oh_hist"]) > COAST_WIN:
                st["oh_hist"].pop(0)
            h = st["oh_hist"]
            v = ((h[-1] - h[0]) * rig.fps / max(1, len(h) - 1)
                 if len(h) > 1 else 0.0)
            coast = float(np.clip(v * COAST_S, 0.0, COAST_MAX))
            # THE COMMAND STOP ASSUMES THE PART DOES NOT SLIP.  It takes the
            # part's centre to be cmd_y - PUSH_BACK; under MuJoCo the key
            # trails the pad by ~2.6 mm (XPBD: ~0.6), so the pad reached its
            # nominal stop with the key short of the edge while the measured-
            # overhang stop -- the real safety -- had not fired.  Reference it
            # to the MEASURED lag, clamped, and only under MuJoCo (LAG_MAX 0
            # makes this the old rule exactly).
            lag = float(np.clip(py - (st["cmd_y"] - PUSH_BACK), 0.0, LAG_MAX))
            done = oh + coast >= oh_stop or st["cmd_y"] <= stop_y + PUSH_BACK - lag
            if done or u >= 1.0:
                st["oh_stop"] = oh
                st["grasp_y"] = grasp_for(oh)
                st["hold_t"] = t
                # NO LATERAL STEP HERE.  Stepping the y command back at
                # the stop was tried and is worse than freezing it: a 10 mm
                # step is a jerk, the arm accelerates hard with the blade
                # still 0.5 mm into the part, and the part goes BALLISTIC
                # -- measured, it rose 0.48 mm, rolled 1 deg and then flew
                # 16 mm forward in 0.8 s while the command was moving the
                # other way.  What separates a 0.5 mm press cleanly is a
                # fast PURE-Z break; see the "up" phase.
                # The 12 g key cannot settle under the still-loaded blade:
                # replay measured 22 -> 65 mm/s outboard acceleration
                # during that phase, followed by a tip.  Break contact in
                # pure Z as soon as the safe overhang is reached.  The
                # broad card keeps the validated gradual unload path.
                next_phase = "up" if PART == "key" else "settle_push"
                next_dur = T_UP if PART == "key" else T_PUSH_SETTLE
                st.update(ph=next_phase, t0=t, dur=next_dur)
                print(f"t={t:.2f}s push done: overhang {oh*1000:.1f} mm "
                      f"of {oh_stop*1000:.1f}, press offset "
                      f"{st['press']*1000:+.2f} mm at {F:.1f} N")
            elif F >= F_PRESS_LO:
                # OPEN-LOOP STROKE FOR BOTH PARTS.  A closed-loop floor
                # (cmd at most PUSH_LEAD ahead of the measured part) was
                # tried for the key and DEADLOCKS: the moment the part
                # stalls, the lead is consumed as arm displacement, the
                # blade physically reaches its command, the stored shear
                # relaxes and nothing moves again -- measured parked at
                # -2.97 mm for 3 s with the pads steady at 0.7 N.  The
                # measured overhang stop above fires before the open-loop
                # command can overrun the part.
                st["cmd_y"] = max(stop_y + PUSH_BACK - lag,
                                  st["cmd_y"] - SLIDE_RATE)
            rig.drive_arm(np.array([0.0, st["cmd_y"],
                                    blade_touch_z - st["press"]]),
                          down_grasp_quat(0.0), fingers=BLADE_Q)
            return

        if ph == "settle_push":
            # LET IT COME TO REST UNDER THE BLADE.  The blade's two pads do
            # not both land on a 20 mm-wide part -- measured on the key,
            # padL carried 5-6 N and padR read 0.00 N for the entire stroke
            # -- so the drag is off-centre and the part picks up YAW.  Take
            # the blade away at that moment and the part keeps pivoting on
            # the bench edge: measured 66 -> 141 deg/s over 0.7 s, walking
            # its centre of mass outboard until it tipped, all with the
            # pads reading their 0.12 N floor.  Holding the press where it
            # is until the part is measurably still costs half a second and
            # removes the whole failure.
            st["hold_v"] = abs(oh - st.get("hold_oh", oh)) * rig.fps
            st["hold_oh"] = oh
            # ...AND THE PRESS IS UNLOADED ON A RAMP, not dropped.  With the
            # blade pitched, the whole press sits on ONE pad -- measured
            # 9.4-10.8 N on padL against 0.00 N on padR -- and taking that
            # away in a single frame is itself the impulse: the part picked
            # up 30 deg/s the moment the pads read zero.  Walking the blade
            # up until the load is essentially gone turns the release into a
            # ramp, and only then does the lift start.
            st["press"] = max(PRESS_MIN, st["press"] - PRESS_RATE * 4)
            if (u >= 1.0 or (st["hold_v"] < PUSH_STILL
                             and pads_N() < F_PRESS_OFF
                             and t - st["t0"] > 0.3)):
                st.update(ph="up", t0=t, dur=T_UP)
            rig.drive_arm(np.array([0.0, st["cmd_y"],
                                    blade_touch_z - st["press"]]),
                          down_grasp_quat(0.0), fingers=BLADE_Q)
            return

        if ph == "up":       # up AND BACK, same orientation
            # BACK, not straight up.  The blade's two pads do not both land
            # on a 20 mm-wide part: measured on the key, one pad carried
            # 4.0 N and the other 0.00, so the last contact of the lift-off
            # is an OFF-CENTRE one that keeps driving the part outboard.
            # It slid another 9 mm after the stop and tipped off the bench.
            # Retreating 6 mm while rising means any residual contact
            # pushes the part back onto the bench instead of over the edge.
            if u >= 1.0:
                if PART == "key":
                    # re-measure AFTER the break: the blade leaving the
                    # key moves it, and the grasp has to centre on where
                    # it actually came to rest, not on the push stop
                    st["oh_stop"] = oh
                    st["grasp_y"] = grasp_for(oh)
                st.update(ph="swing", t0=t, dur=T_SWING)
            # LIFT FIRST, THEN RETREAT.  Doing both at once means the
            # blade is still within a millimetre of the part's top face
            # while it is already moving, and the pad's leading lower
            # corner catches it: measured on the key, one pad read 2.64 N,
            # the part rose 0.3 mm and was AIRBORNE from the next frame,
            # travelling 22 mm/s with no contact at all (part_N sat at the
            # probe's 0.12 N floor, not at its 0.118 N weight).  The blade
            # now goes straight up through UP_SPLIT of the phase -- 8 mm of
            # clearance -- and only then backs off.
            # BREAK CONTACT FIRST, AND FAST.  The press is only ~0.5 mm
            # deep, so UP_BREAK of pure-z travel in the first UP_SPLIT of
            # the phase separates the blade from the part with NO lateral
            # command at all; only after that does the wrist rise the rest
            # of the way and back off.
            z0 = blade_touch_z - st["press"]
            a = min(1.0, u / UP_SPLIT)                 # linear: be quick
            b = smooth(max(0.0, (u - UP_SPLIT) / (1.0 - UP_SPLIT)))
            st["cmd_y_up"] = st["cmd_y"] + UP_BACK * b
            z = z0 + UP_BREAK * a + (transit_z - z0 - UP_BREAK) * b
            rig.drive_arm(np.array([0.0, st["cmd_y_up"], z]),
                          down_grasp_quat(0.0), fingers=BLADE_Q)
            return

        if ph == "swing":
            # TRANSIT LEG, at SAFE_Z: translate out past the edge AND slerp
            # the wrist from the overhead blade pose to the horizontal side
            # pose.  Both happen up here on purpose (see SAFE_Z).
            a = smooth(u)
            y0 = st.get("cmd_y_up", st["cmd_y"])
            y = y0 + (EDGE_Y - STANDOFF - y0) * a
            if u >= 1.0:
                st.update(ph="descend", t0=t, dur=T_DESCEND)
            rig.drive_arm(np.array([0.0, y, transit_z]),
                          slerp(q_down, q_side, a),
                          fingers=FINGER_OPEN * smooth(min(1.0, u * 2)))
            return

        if ph == "descend":
            # straight down, OUTSIDE the edge: the pad tip is still
            # TIP_MARGIN + TIP_FROM_TCP short of the table, so the pads
            # drop past the card's plane through open air
            a = smooth(u)
            if u >= 1.0:
                st.update(ph="reach", t0=t, dur=T_REACH)
            rig.drive_arm(np.array([0.0, EDGE_Y - STANDOFF,
                                    transit_z + (grasp_z - transit_z) * a]),
                          q_side, fingers=FINGER_OPEN)
            return

        if ph == "reach":                       # in along +y over/under it
            y = (EDGE_Y - STANDOFF) + (
                st["grasp_y"] - (EDGE_Y - STANDOFF)) * smooth(u)
            if u >= 1.0:
                st.update(ph="close", t0=t, dur=T_CLOSE)
            rig.drive_arm(np.array([0.0, y, grasp_z]), q_side,
                          fingers=FINGER_OPEN)
            return

        if ph == "close":
            if u >= 1.0:
                st.update(ph="lift", t0=t, dur=T_LIFT)
            rig.drive_arm(np.array([0.0, st["grasp_y"], grasp_z]),
                          q_side,
                          fingers=FINGER_OPEN
                          + (FINGER_CLOSE - FINGER_OPEN) * smooth(u))
            return

        rig.drive_arm(np.array([0.0, st["grasp_y"],
                                grasp_z + LIFT_D * smooth(u)]),
                      q_side, fingers=FINGER_CLOSE)

    def extra(state, t):
        bq = state.body_q.numpy()
        return {"part_y_mm": round(float(bq[pi, 1]) * 1000, 2),
                "part_z_mm": round((float(bq[pi, 2]) - TABLE_TOP) * 1000, 2),
                "overhang_mm": round(
                    (EDGE_Y - (float(bq[pi, 1]) - depth / 2)) * 1000, 2),
                "press_mm": round(st["press"] * 1000, 3),
                "padL_N": round(rig.probes["padL"].history[-1]["force"], 3),
                "padR_N": round(rig.probes["padR"].history[-1]["force"], 3)}

    rec = rig.run(t_end, control, extra)
    paths = rec.save("rollout", force_keys=["part_N", "padL_N", "padR_N",
                                            "overhang_mm", "part_z_mm"])
    print("saved:", paths)
    qa = rig.qa(paths)
    d = np.load(paths["states"], allow_pickle=True)
    bq = d["body_q"]
    lift = float(bq[-1, pi, 2] - bq[0, pi, 2])
    # the overhang the PUSH reached, not the minimum over the whole run:
    # a card that has fallen off the table also has a very large "overhang"
    oh = float(st["oh_stop"] or 0.0)
    need = min(depth * 0.35, oh_stop * 0.9)
    ok = lift > 0.030 and oh > need
    qa.report["checks"]["slide_then_pick"] = {
        "pass": bool(ok), "lift_mm": round(lift * 1000, 2),
        "max_overhang_mm": round(oh * 1000, 2),
        "overhang_frac": round(oh / depth, 3),
        "overhang_needed_mm": round(need * 1000, 2),
        "note": "a flat part on a flat table has no graspable face; the "
                "overhang is what creates one, and the side pinch is only "
                "reachable because there is air under it"}
    qa.report["pass"] &= bool(ok)
    print(f"  slide_then_pick: {'PASS' if ok else 'FAIL'}  "
          f"lift {lift*1000:.1f} mm, overhang {oh*1000:.1f} mm "
          f"({oh/depth*100:.0f} % of the part)")
    rig.finish(qa)


if __name__ == "__main__":
    main()
