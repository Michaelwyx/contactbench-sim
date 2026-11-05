"""Task: the FR3 picks a keychain (ring + two FREE keys) off a tabletop pile
and HANGS IT ON A PEG of the hanging_board — a real fingertip pinch on the
ring, keys threaded on the ring by CONTACT ONLY, no reorientation anywhere.

GEOMETRY (measured off the meshes; mm, assembly coords):
  * hanging_board — wall (x -140..-130, top z 181) on an L base, and three
    HOOK PEGS at y = -81, -25, +31.  Each peg is a 6.0 x 6.0 mm bar (y +-3.0
    about its centre, z 152.0..158.0) running out of the wall's root fillet
    to x = -99.0, then an UPTURNED HOOK x -99.0..-86.0 whose crest is
    z 168.22 at x = -90.0 and whose free tip is z 162.86..167.63 at x = -87.
    Measured on 0.5-1 mm x-slices; the three pegs are identical.  The hook's
    inner slope (x -99..-90) is 49 deg, well past atan(mu = 0.55) = 28.8, so
    anything threaded over the tip SLIDES DOWN onto the flat bar; its outer
    slope is 11 deg, so nothing slides back off the tip.
  * unidentified/part1 — a flat WRENCH (7.5 mm thick in x, 14.3 mm wide in y
    below its boss) hanging on the MIDDLE peg, top dome at z = 161.01.
  * keyring/part3 — ring OD 26.0, ID 21.0, centreline radius 11.75, and the
    cross-section is SQUARE, 2.5 x 2.5 mm.  That square section is what
    makes this task possible with bare fingers: the flat pads land on a flat
    2.5 mm face, so the pinch resists ROLL about the tube tangent — a round
    wire would roll between the pads and the ring would flop to vertical the
    moment it left the table.
  * key_1 part1 + part3 — two 50x20 mm keys; each bow has a 12.4 x 6.0 mm
    SLOT through the ~2.5 mm plate.  The ring tube passes through both slots
    — the keys are topologically LINKED to the ring (discrete Gauss linking
    number = -1) and hang/swing on it by contact alone.  NO JOINT of any
    kind connects them, and QA gates that every run (linking number at the
    hang and at the end, plus a free-swing range during the carry).

WHY THE RING IS NEVER STOOD UP — the measurement that shaped this task.
A ring hangs on a horizontal peg with its normal along the peg, i.e.
VERTICAL, and the obvious protocol is to slerp the wrist during the carry
and thread the ring on sideways.  It does not survive contact with this
solver, and the reason is a force-path one rather than a tuning one:

  * at the pile's rest attitude the ring's normal is 27.1 deg off vertical,
    so cos(27.1) = 89 % of the keychain's 0.167 N goes into the pads as a
    NORMAL contact load — the pinch is a shelf, not a clamp;
  * stand the ring vertical and 100 % of it is TANGENTIAL, carried only by
    XPBD's friction, whose per-iteration correction is clamped to
    mu*lambda_n of that iteration — and lambda_n collapses as soon as the
    normal constraint is satisfied, so the effective hold is far below the
    nominal 2*mu*F_n.

Instrumented (the rollout logs slip[tangential/axial/radial], the ring's
displacement from a rigid pad-ring relation in the tool frame; the pad's
window on the ring is radial 5.8-24.3 mm against a tube at 10.5-13.0, and
+-8.75 mm tangential):

  | attempt                              | pad force | outcome              |
  |--------------------------------------|-----------|----------------------|
  | vertical carry, 1.2 N/pad            | 1.35->0.4 | dropped 6.5 s in     |
  | vertical carry, 1.6 N/pad            | 1.8 ->0.5 | dropped 8.2 s in     |
  | + ke 2500->5000 and an ADMITTANCE    | 2.2-2.7   | radial creep 0.15 mm |
  |   grip holding 3 N/pad               |           | over 4 s: FIXED      |
  | ...but the same run then lost it to  | 2.0       | 10.4 mm tangential,  |
  |   a 53 deg swing about the pinch     |           | out sideways         |
  | pinch moved to the ring's exact top  | 2.0-2.2   | still 8.5 mm         |
  |   (no tangential gravity component)  |           | tangential, dropped  |

Survival time scaled as 1/(pad force) and was indifferent to transport
speed, i.e. it is per-FRAME creep, not inertia.  The cure is not a bigger
number, it is not asking friction to do the job: keep the ring at the
attitude it was picked in and hang it on a feature a near-horizontal ring
can be threaded onto.

WHERE THAT LEAVES THE WRENCH (the previous protocol's target).  Threading
the ring DOWN over the wrench's dome is impossible, and by ~3 mm rather
than by a tuning margin.  Free x-span in the middle peg's y-band, wall
fillet to hook, per mm of height (a 26.0 mm ring needs 26.0):

    z mm    161.0  160.0  159.0  158.0  157.0  156.0  155.0  154.0
    span    28.73  27.53  25.70  20.07  15.05  20.07  25.02  29.96

The pocket is wide enough only down to z ~ 159.5 and the wrench's own tip
is at 161.0, so the deepest a 26 mm ring can be captured on the wrench is
about 1 mm below its apex, resting on the peg 0.5 mm from both the fillet
and the hook.  A ring 1 mm onto a 113 mm bar is balanced, not hung.  Below
that the peg bar itself (z 152..158, x -120..-99) blocks it outright.

THE HANG THAT WORKS — thread the HOOK TIP, from above, unrotated.  The
peg's free end curls UP, which is exactly the feature a near-horizontal
ring can be dropped onto.  Scanned over ring-centre x/y/z at the grasp
attitude (344 poses tested against the board mesh), the best is ring
centre (-89.0, -81.5, 164.0): the hook centreline pierces the ring's own
plane 0.9 mm from the ring centre (hole radius 10.5), the ring clears the
board by 0.52 mm, the fingertip boxes clear it by 6.55 mm, and the FR3's
IK residual is 0.000 mm.  The descent corridor to it is clear the whole
way (ring/board 13.4 -> 1.5 mm from z 185 to 165, fingertips never below
6.8 mm), and the OPEN pads at the release clear the board by 7.6 mm.
Because the ring's own tube reaches the board at ring-centre z = 163.5 and
the hook is already through the hole by z = 164.0, the descent is
force-gated and allowed to command past contact: the board reaction stops
it, which is a physical seat rather than a 1 mm geometric window.

FREE-KEY PHYSICS (settle_keys_keychain.py, QA PASS): keys threaded on the
held ring dangle at rest (contact = key weight), swing as pendulums when
kicked (14-130 deg, ~20 zero crossings), stay linked, penetrate < 0.03 mm.
Mesh recipe: subdivide <=1.5 mm, SDF res 448, margin 0.4 mm, gap 0.3 mm —
the slot/tube clearance is ~0.5 mm and must stay real.

START — the tabletop pile: a flat ring with threaded keys is geometrically
impossible (coplanar linked loops), so the pile's rest is ring-on-keys.
The pile sits at the EXACT pose settle_keys_keychain.py "start" validates,
ring centre 3.3 mm inside the table's +y edge, so its +y crescent overhangs
by 8.5 mm and the pinch needs no transport phase.

NO STABILIZER JOINT.  Earlier versions carried the usual weak d6 from link7
to the ring.  With the ring resting on the pile that joint is fatal:
articulation membership (joint_articulation[j] = 0, which loop grasps need
or the gripped body's static contacts die) closes the loop table -> key ->
ring -> d6 -> arm -> world, and XPBD blows the cluster apart at 800 m/s
within ~1 s of full gravity — even with all six gains at zero, purely from
being in the chain.  The square ring section removes the need for the joint
entirely, so the fingertips do 100 % of the holding.

EXPERT PROTOCOL:
  1. ramped-gravity settle of the pile at the table's +y edge; the arm
     waits parked ALREADY in the card-pinch attitude above and behind the
     standoff, so no part of it ever passes over the pile,
  2. EDGE PINCH: hand horizontal, pads above and below the overhanging tube
     crescent — the lower finger (and its 26 mm root) hangs in free air past
     the bench edge; cage, then a stationary-hand close onto the flat 2.5 mm
     face of the square section, ramped to a real preload,
  3. LIFT 128 mm straight up: the keys peel off the table and swing down to
     hang on the tube,
  4. TRANSIT 225 mm to the hover in front of the board, attitude UNCHANGED,
     then DWELL 1.5 s (~4 pendulum periods) so the carried cluster stops
     swinging before anything has to be aimed,
  5. lateral 19 mm onto the thread axis, closed-loop on the MEASURED ring
     centre (EMA + deadband, droop-scale cap),
  6. THREAD: force-gated 25 mm descent; the hook tip passes up through the
     ring's 21 mm hole and the board reaction seats it,
  7. RELEASE: the pads simply open — the ring drops onto the hook and
     slides down its 49 deg inner slope onto the flat peg bar, keys hanging,
  8. RETREAT: rise 35 mm clear of the board FIRST (the open pad's bottom
     face is 9.5 mm below the TCP and the hook crest is above that at the
     release height), then translate +x and hold.  BOTH the retreat window
     and the last 0.8 s (arm fully clear) are QA-gated on the keychain still
     being on the peg with its whole weight carried by the board.

COLLISION CADENCE is armed SAFE at build (README pitfall 5.3: the teleop
gate replaces control() wholesale, so anything set per frame does not exist
under the gate).  The expert relaxes it to once-per-frame only between the
settle and the release — the window that needs pitfall 5.1's anchor
accumulation and in which nothing free-falls.

TRAJECTORY SHAPE IS A GRASP PARAMETER HERE, in two measured ways:
  * every waypoint segment uses a QUINTIC smootherstep, not the usual cubic
    smoothstep.  A cubic is C1 but leaves an ACCELERATION STEP at each
    waypoint, and the carried keychain is a ~0.35 s pendulum on a friction
    pivot: on the cubic profile the transit rang the ring 11 mm ahead of
    its own command and rolled the pinch off the tube.
  * every transport leg is speed-budgeted TWICE — once against the contact
    model (once-per-frame collision anchors the contact at the frame's
    start pose, so the grasp wants << 2 mm/frame) and once against the
    jerk-limited joint tracker.  A 3.9 s transit demanded 0.0189 rad/frame
    of joint step against a DQ_MAX of 0.020, and a second-order tracker
    running at v settles 1.6 deg behind (err = v^2/2a) — which showed up as
    10-11 mm of measured TCP error and shook the pendulum.  5.5 s brings the
    demand to 0.0134 rad/frame and the IK residual to 0.20 mm.

ARM SMOOTHNESS: the TCP path is C1 everywhere, but that is not enough on a
7-dof arm — see drive_smooth() and README pitfall 54.

STATUS (2026-08-14) — TELEOP GATE PASS, GRASP SOLVED, HANG NOT.
The acceptance gate is green: `teleop_gate.py keychain_hang --repeat 6`
returns 6/6 with ZERO integrity violations, `flaky: false`, no
divergence, worst overlap 0.101 mm (ring<->key_b) and 0.00 mm of excess
over the rest baseline, peak body speed 0.618 m/s, guard peak pad force
0.41 N.  Read it with one caveat: this scene's feature clearances are
still undeclared in teleop_gate.TASKS (5 bodies on the 5 mm `_default`,
README pitfall 9.6), so the governor was permissive; the numbers it
should carry are ring/board 0.5 mm and fingertip/board 4.5 mm.  What is fixed and
measured: the pinch itself.  Pair-resolved fingertip<->ring force is
2.6-3.7 N/pad sustained from the close to the release, the ring's
displacement from a rigid pad-ring relation stays under 2 mm through the
lift and transit, the ring's attitude is preserved (|n_x| 0.31 -> 0.34),
and the arm tracks its command to < 3 mm.  The keys stay linked (Gauss
-1) and swing 64-90 deg on the ring.  What is NOT fixed: the keychain is
lost between the top of the transit and the release on roughly 3 runs in
4, always the same way — the ring ROLLS in the pads (|n_x| 0.33 -> 0.9 in
about 0.7 s) with the arm tracking cleanly and zero environment contact,
i.e. purely from the two keys' pendulum torque against the 1.25 mm moment
arm the flat pads offer.  Compressing the carry from 13 s to 7.4 s took it
from always-lost to sometimes-held (one clean arrival at the drop point,
3.6 N/pad, on target to 0.7 mm), and relaxing the squeeze to
FINGER_TOUCH before opening removes the 1.06 m/s ejection the stored pad
energy produces, but neither makes the carry reliable.  A flaky carry is a
failing carry.  The remaining lever is not in the controller — see
docs/asset_defects.md.

Run: python -m force_bench.tasks.keychain_hang
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.probes import ForceProbe
# TB_GELSIGHT=1 -> GelSight fingertips (sets the gel solver defaults at
# import, so this must precede task_rig).  Case tip extension stays 0 --
# the pinch rides the ring's overhanging crescent with the lower fingertip
# hanging past the bench edge, and the FOV question is measured, not
# assumed (CoP lands mid-pad: the tube sits 4.7-7.2 mm inside the pad).
import force_bench.tactile.task as GS
from force_bench.rig import make_solver, JD, Rig
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass

MM = 1000.0

# ── measured geometry (m) ───────────────────────────────────────────────
# hanging_board: three HOOK PEGS at y = -0.081, -0.025, +0.031.  Each is a
# 6.0 x 6.0 mm bar (y +-3.0 mm about its centre, z 0.1520..0.1580) running
# out of the wall root fillet to x = -0.0990, then an upturned hook
# x -0.0990..-0.0860 whose crest is z 0.16822 at x = -0.0900 and whose free
# tip is z 0.16286..0.16763 at x = -0.0870.  Cross-sections measured on
# 1 mm x-slices; the three pegs are identical.
PEG_Y = -0.0810         # HANG TARGET: the keyring's OWN CAD home peg
PEG_TOP = 0.1580
PEG_X0, PEG_X1 = -0.1300, -0.0860   # peg + hook x extent
# the wrench, on the MIDDLE peg: reference geometry only now — it is a
# live collision shape and a probe target, but no longer the hang target
# (see the header's free-span table for why it cannot be threaded)
ROD_AXIS_XY = np.array([-0.1108, -0.0252])
ROD_TIP_Z = 0.16101
R_TUBE = 0.011751       # tube centerline radius
R_ID = 0.0105           # ring inner radius (OD 26.0, square section 2.5)
HANG_Z = PEG_TOP - R_ID   # 0.1475 — ring inner-top resting on the peg top
# THREAD corridor (ring-centre space), measured clearances in the header.
# The ring is threaded onto the peg's UPTURNED HOOK TIP from above, at the
# attitude it was picked in — no wrist reorientation anywhere in the
# mission (see the header for why standing the ring up loses the grasp).
THREAD_XY = np.array([-0.0890, -0.0815])   # over the hook tip
HOVER_Z = 0.1850        # ring-centre height for the lateral approach: the
                        # ring's low arc is 6.6 mm below its centre, i.e.
                        # 178.4 mm, clear of the 168.22 mm hook crest
THREAD_Z = 0.1830       # THE RING IS DROPPED ONTO THE HOOK, not lowered
                        # onto it.  The keys hang from the ring's rim, 7.5
                        # mm to -y and 7.3 mm to -x of its centre, so at any
                        # ring height where the hole can reach the peg a key
                        # is alongside the peg too — the contact is not
                        # avoidable, it is the task.  And the pinch is
                        # ROLL-WEAK by construction: the pads grip a 2.5 mm
                        # wide annular FACE, so the moment arm resisting
                        # roll about the tube tangent is 1.25 mm and the
                        # capacity at 3.5 N/pad is ~8.8 mN.m, against the
                        # ~36 mN.m a 6 g key delivers when 0.9 N of board
                        # reaction lands 40 mm out.  Measured: 0.5 s of key
                        # contact rolled the ring from |n_x| 0.33 to 0.02,
                        # i.e. edge-on to the peg, and the "hang" then slid
                        # straight off.  4x more grip is not available (14
                        # N/pad needs 2.8 mm of interference into a 2.5 mm
                        # tube), so the answer is TIME: arrive and let go.
                        # From 183.0 the ring free-falls the 19.5 mm to
                        # where its tube meets the hook (163.5) in 63 ms —
                        # four frames — and the hook is inside the hole for
                        # the whole last 1.5 mm of that.

# ── tabletop start: the pile's BAKED REST (settle_keys_keychain "start",
#    pile dropped at (0, 0.132) on a full table, 5 s, QA PASS, residual
#    drift < 0.6 mm/s).  Baking the rest instead of re-dropping it every
#    run makes the whole grasp deterministic: the emergent rest is NOT the
#    assembled pose (the ring slides off the leaning keys and ends up
#    LEANING at 27 deg with its top arc 12 mm in the air, which is exactly
#    what makes it graspable).
BAKED = {
    "ring": (np.array([-0.004617, 0.128457, 0.006598]),
             np.array([0.129202, -0.804139, -0.105886, 0.570487])),
    "key_a": (np.array([-0.034355, 0.129189, 0.002443]),
              np.array([-0.502216, 0.427331, -0.577241, 0.481623])),
    "key_b": (np.array([-0.002606, 0.098783, 0.002391]),
              np.array([-0.033013, -0.704984, -0.703372, 0.084711])),
}
# L-shaped bench.  The pinch has to put the LOWER fingertip (15 mm of body
# behind its pad face) under the ring's raised arc, which reaches 5 mm
# below the bench top over x 0.6..19.6 mm, y 134..153 mm.  Nothing in the
# pile rests there (the ring touches down at x -12 y 121, key_a lives at
# x < -9.5, key_b at y < 124), so the bench simply ends at y = 0.130 for
# x > -0.005 and keeps its full y = 0.160 edge everywhere else.
BENCH_Z, BENCH_T = 0.0, 0.005
BENCH_A = (-0.320, -0.005, -0.440, 0.160)   # x0 x1 y0 y1 (full support)
BENCH_B = (-0.005, 0.280, -0.440, 0.130)    # cut-back corner
TABLE_C = (-0.02, -0.14)                    # (kept for the QA note)

RING_MASS, KEY_MASS = 0.005, 0.006
W_TOTAL = (RING_MASS + 2 * KEY_MASS) * 9.81
KC_MU, ROD_MU, BOARD_MU = 0.6, 0.4, 0.5   # XPBD averages the
                        # pair's mu, so pad(0.9)+ring(0.6) = 0.75

TCP_R = 0.0155          # TCP 15.5 mm from the ring centre along the pinch
                        # direction (measured IN the ring's own plane).
                        # The pad runs from ring radius 5.8 (its tip) out
                        # to 24.3, so the 10.5..13.0 tube face is seated
                        # 4.7-7.2 mm INSIDE the pad.  At 19 mm the tube sat
                        # 1.2 mm from the pad tip and a 3.4 mm slip during
                        # the close walked the pads straight off it.
                        # Because the thread attitude pinches the ring's
                        # HIGH side, the pad's inner end still clears the
                        # rod dome by 2.5 mm at the release height.
FINGER_KE, FINGER_KD = 5000.0, 30.0
# The carry is a pure FRICTION hang and it CREEPS, at a rate measured to go
# as 1/(pad force).  Instrumented (slip[t/a/r] in the rollout log): at
# 1.2 N/pad the ring walked radially out of the pads and was dropped 6.5 s
# after the pinch; at 1.6 N/pad, 8.2 s after.  The pad's radial window on
# the ring is 5.8-24.3 mm against a tube at 10.5-13.0, i.e. a 4.7 mm budget
# in the direction gravity pushes, so the carry has to be BOTH shorter and
# harder-gripped than a nominally-30x-Coulomb-margin calculation suggests.
# ke 2500 -> 5000 keeps omega*dt = sqrt(5000/0.005)/(60*24) = 0.69, below
# the 0.93 at which docs/asset_defects.md records nut_bolt_arm diverging.
F_GRIP_TARGET = 4.5     # N/pad held by the ADMITTANCE grip (pitfall 7.6):
F_GRIP_DB = 0.5         # a fixed finger POSITION cannot hold a force that
GRIP_STEP = 6.0e-6      # decays as the part creeps out of the pads
FINGER_MIN = 0.00020    # advance stop: 1.055 mm interference ~ 5.3 N/pad
FINGER_HOME = 0.04      # URDF home half-gap: the t=0 command MUST ramp out
                        # of it (a step slams the fingers shut in one frame)
FINGER_OPEN = 0.0045    # cage half-gap on approach (tube is 2.5 mm)
# The pads first touch the 2.5 mm section at dof = 1.335 mm (measured:
# the ring starts reacting at exactly that gap).  Commanding 0.2 mm meant
# 2.1 mm of interference — on a 5 g ring resting on a slope that is not a
# grasp, it is a launch: XPBD resolved the overlap by firing the ring
# 3.4 mm out from between the pads before the second pad ever loaded.
# Close to a REAL preload instead: 0.29 mm interference ~ 0.7 N/pad, then
# 0.49 mm ~ 1.2 N/pad, against a 0.167 N payload at mu 0.9.
FINGER_TOUCH = 0.001335
FINGER_CLOSE = 0.00110   # 0.235 mm interference ~ 1.2 N/pad at the
                        # CLOSE — unchanged in NEWTONS from the proven
                        # close, the gain change is absorbed here
FINGER_TIGHT = 0.00070   # 0.635 mm ~ 3.2 N/pad for the carry
FINGER_RELEASE = 0.006
FINGER_MU = 0.9         # rubberized Franka fingertip pads
# GelSight variant: every pad-face-calibrated dof gets the sensing-face
# offset (5 mm); the admittance grip then holds the same 4.5 N/pad band.
FINGER_OPEN += GS.finger_offset()
FINGER_TOUCH += GS.finger_offset()
FINGER_CLOSE += GS.finger_offset()
FINGER_TIGHT += GS.finger_offset()
FINGER_MIN += GS.finger_offset()
FINGER_RELEASE += GS.finger_offset()
if GS.finger_offset():
    # SOFT-GEL DEEPENING (measured on the first gel run): the bare-pad
    # ladder is sized against near-rigid rubber; on the 2.5 mm tube the
    # gel's Winkler slope is only ~1.5 N/mm (tiny contact strip), so the
    # bare interference + the 0.5 mm admittance budget delivered
    # 0.55-1.29 N/pad against the 4.5 N band and the ring was dropped in
    # transit (axial slip 157 mm).  1 mm deeper reaches the stiffening
    # regime (p = kh*x*(1+10(x/t)^2)) where the band is attainable; the
    # admittance still owns the setpoint.  CoP sits at +8.8 mm -- the
    # pad-tip zone, outside the imaged FOV: hardware-true for this pinch
    # (a case tip-extend would collide with the hook at the thread).
    # Round 2 (measured): -1.0 mm carried the WHOLE transit at 1.4-2.6 N
    # (was dropped at t=7 before) and arrived on target to 0.7 mm, but the
    # ring still ROLLED in the pinch (|n_x| 0.33 -> 0.61) and the drop
    # missed the rolled hole.  Roll capacity grows with indentation (the
    # gel wraps the square tube's corners -- geometric interlock, not just
    # friction), and the admittance was pinned at FINGER_MIN before ever
    # reaching its 4.5 N band (2.06 N mean).  -2.0 mm gives the servo the
    # room to actually reach the band.
    # Round 3 (measured): -2.0 mm REGRESSED -- the commanded face gap goes
    # 2.8 mm NEGATIVE, the two gel faces close on EACH OTHER around the
    # 2.5 mm tube, and the gel-gel hydro contact steals the closure: the
    # sensor carried 7.5 N/pad but only 2.1 N of it was on the RING
    # (probe/sensor 0.28, crosscheck FAIL) and the carry still dropped.
    # -1.0 mm is the working depth: ring force 2.1 N at crosscheck 0.60,
    # full transit survived; the residual roll-drop at the very release
    # matches the bare task's own documented flakiness (3 of 4 carries).
    FINGER_CLOSE -= 0.0005
    FINGER_TIGHT -= 0.0010
    FINGER_MIN -= 0.0010

SUBDIV_KC = 0.0015
SUBDIV_ROD = 0.003
SDF_KC = dict(sdf=True, sdf_resolution=448, narrow_band=0.0012,
              margin=0.0004)
SDF_STAT = dict(sdf=True, sdf_resolution=384, narrow_band=0.0015,
                margin=0.0008)
GAP_KC = 0.0003

# phase times (s)
T_GRAV = 0.3            # short gravity ramp; the pile starts AT REST so
                        # this only takes the numerical edge off frame 0
T_SETTLE = 1.7          # pile seated; grasp geometry frozen here
T_APPR = 3.4            # hand descends to the edge-pinch standoff (+22 mm)
T_CAGE = 4.4            # open pads slide -y over the crescent
T_CLOSE = 5.1           # stationary-hand close on the tube
T_TIGHT = 5.9
# EVERY TRANSPORT LEG IS SPEED-BUDGETED.  Once-per-frame collision (the
# thing that makes this friction pinch work at all, pitfall 5.1) anchors the
# contact at the frame's START pose, so a leg that moves the grasp far in
# one frame degrades the anchor set.  Measured on this task: a lift at
# 2.7 mm/frame and a transit at 3.3 mm/frame bled the pads from 1.35 N to
# 0.4 N over the carry and DROPPED the keychain at t=11.9 s, while the
# 1.95 mm/frame lift of the previous protocol held 1.3-1.9 N throughout.
# Every leg below is therefore sized for a smoothstep peak of <= 1.5
# mm/frame (90 mm/s), which costs ~5 s of rollout and buys the grasp.
T_PEEL = 7.5            # the keys hang ~62 mm below the ring, so once the
                        # ring passes z ~ 75 mm their tips are off the bench
                        # and nothing is breaking contact any more.  The
                        # grasp rides only ~1.6 s of the slow cadence, and it
                        # is 89 % NORMAL-loaded there.
T_LIFT = 8.8            # 128 mm straight up, peak 1.36 mm/frame — this leg is where THE
                        # KEYS PEEL OFF THE TABLE, a contact-BREAKING
                        # transition under once-per-frame collision, and it
                        # is budgeted for THEM: at 2.35 mm/frame (8x the
                        # 0.3 mm contact gap) key_a was ejected out of the
                        # scene at t=7.3 on 1 run in 2 while the ring itself
                        # carried on and hung perfectly.  The rollout looks
                        # like a success until you ask where the keys are
T_TRANS = 11.8          # 220 mm transit AT THE LIFT HEIGHT, peak 2.29
                        # mm/frame.  Transport speed is not the variable
                        # that decides this grasp — EXPOSURE TIME is.  The
                        # keys are two 6 g pendulums hanging off the ring's
                        # rim and the pinch's roll capacity is 1.25 mm of
                        # moment arm (see THREAD_Z), so the ring rolls out
                        # of the pads eventually no matter how slowly it is
                        # carried: measured 12.4 s and 13.0 s carries lost
                        # it at 3+ N/pad with the arm tracking to 3 mm and
                        # zero environment contact, while SLOWING the
                        # transit from 3.9 s to 6.5 s to 9.0 s did not help
                        # (2 of 3 lost).  This protocol is back to the ~7 s
                        # carry the task's own earlier revision survived.
T_DWELL = 12.2          # brief settle before the climb
T_HOVER0 = 13.8         # climb 50 mm at 0.98 mm/frame (key_b was lost
                        # here at 1.56 mm/frame — same peel problem) in front of the
                        # board (the board's x_max is -86.0, this is -70.0)
T_HOVER = 14.1          # closed-loop re-centre on the measured ring pose
T_THREAD = 14.9         # force-gated 19 mm LATERAL move onto the drop axis
T_RELEASE = 15.4        # fingers open; the ring falls 19.5 mm onto the hook
                        # and slides down its 49 deg inner slope onto the
                        # flat peg bar
T_SWING = 16.6
T_RETREAT = 17.8        # TCP rises clear, then retreats +x
T_FREE = 18.6           # end of the retreat; free-hang QA window
                        # runs T_RETREAT+0.2 .. T_FREE and T_END-0.8 .. T_END
T_END = 19.8
RETREAT_CLEAR = 0.016   # FIRST retreat leg: slide the open pads radially OUT
                        # from under the ring's rim, along the horizontal
                        # pinch direction (+x+y, away from wall, hook and
                        # keys), before rising.  Measured 2026-09-17 on the
                        # GelSight variant, two identical runs: after the
                        # release the rim still rested on the lower pad
                        # (0.1-0.2 N on the right fingertip) and the rise
                        # carried the ring 10 mm back up off the hook tip;
                        # where it landed afterwards was luck (once onto the
                        # bar, once onto the bench).  16 mm takes the pad's
                        # 5.8 mm inner end past the 13.0 mm tube radius.
RETREAT_D = 0.055       # retreat distance along +x (retraces the corridor)
RETREAT_UP = 0.035      # ... but RISE first: the open pad's bottom face
                        # is 9.5 mm below the TCP and the hook crest is
                        # 8 mm above it at the release height
START_OFF = np.array([0.0, 0.030, 0.045])   # arm start: TCP parked above
                        # and behind the pinch standoff, already in the
                        # card-pinch attitude (no reorientation near the pile)

# ── control smoothness (stability agent; keep) ──────────────────────────
DROOP_GAIN, DROOP_STEP, DROOP_MAX = 0.35, 0.0004, 0.008
CAGE_STAND = 0.022      # pads start this far out along the pinch ray
CORR_GAIN, CORR_STEP, CORR_MAX = 0.25, 0.0008, 0.012
CORR_ALPHA = 0.06       # ~0.33 s: long enough to average out the
                        # keys' ~0.35 s pendulum instead of chasing it
CORR_DB = 0.0005
ARM_KD_TASK = 400.0
# Jerk-limited joint-command tracker.  The TCP waypoint path is C1, but the
# 7-dof LM IK is redundant: at a nearly converged residual its damping term
# has decayed and a single step can slide the arm along the NULL SPACE
# (elbow flip) — 43 mm of link3 travel in one frame, 158 m/s^2, while the
# TCP itself never moves.  Rate + acceleration limits on the joint command
# (with the limited pose written BACK as the IK warm start) bound that to
# a few m/s^2 and cost only one frame of lag on the real trajectory.
DQ_MAX = 0.030          # rad/frame (1.8 rad/s), inside the FR3's own
                        # datasheet limits and ~1.6x this mission's peak
DDQ_MAX = 0.0060        # rad/frame^2 (21.6 rad/s^2).  A tracker running
                        # at v settles v^2/2a behind, so a cap set just
                        # under the mission's demand is not a guard, it is
                        # a trajectory: at 0.0025 the transit ran 2 deg
                        # (~10 mm of TCP) behind and rang the payload


from force_bench.geometry import subdiv


from force_bench.geometry import world_T


def quat_of_R(R: np.ndarray) -> np.ndarray:
    """[x,y,z,w] (warp layout) from a rotation matrix."""
    M = np.eye(4)
    M[:3, :3] = R
    q = trimesh.transformations.quaternion_from_matrix(M)
    return np.array([q[1], q[2], q[3], q[0]])


def wp_quat(q):
    return wp.quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def R_of_q(q):
    """Rotation matrix from a warp-layout quaternion [x, y, z, w]."""
    return trimesh.transformations.quaternion_matrix(
        [q[3], q[0], q[1], q[2]])[:3, :3]


def slerp(qa, qb, s):
    qa = qa / np.linalg.norm(qa)
    qb = qb / np.linalg.norm(qb)
    d = float(np.dot(qa, qb))
    if d < 0:
        qb, d = -qb, -d
    if d > 0.9995:
        q = qa + s * (qb - qa)
        return q / np.linalg.norm(q)
    th = np.arccos(np.clip(d, -1, 1))
    return (np.sin((1 - s) * th) * qa + np.sin(s * th) * qb) / np.sin(th)


def smooth(u):
    """Quintic smootherstep: zero VELOCITY *and* zero ACCELERATION at both
    ends.  The cubic 3u^2-2u^3 leaves an acceleration STEP at each waypoint,
    and the carried keychain is a ~0.35 s pendulum hanging off a friction
    pinch: measured on the cubic profile, the transit rang the ring 11 mm
    ahead of its command and rolled the pinch off the tube.  A jerk-free
    profile costs nothing and removes the excitation at source."""
    u = np.clip(u, 0.0, 1.0)
    return u ** 3 * (u * (6.0 * u - 15.0) + 10.0)


def gauss_link(c1, c2):
    """Discrete Gauss linking number of two closed polylines."""
    a0, a1 = c1, np.roll(c1, -1, axis=0)
    b0, b1 = c2, np.roll(c2, -1, axis=0)
    am, da = (a0 + a1) / 2, a1 - a0
    bm, db = (b0 + b1) / 2, b1 - b0
    r = am[:, None, :] - bm[None, :, :]
    rn = np.linalg.norm(r, axis=-1) ** 3
    cr = np.cross(da[:, None, :], db[None, :, :])
    return float(np.einsum("ijk,ijk->ij", cr, r / rn[..., None]).sum()
                 / (4 * np.pi))


def key_geometry(tm: trimesh.Trimesh):
    """Bow-plate normal, hole center + boundary loop (CAD coords)."""
    v = tm.vertices
    bow = v[v[:, 2] > 0.1345]
    bc = bow.mean(axis=0)
    n = np.linalg.svd(bow - bc, full_matrices=False)[2][2]
    if n[1] < 0:
        n = -n
    d = (bow - bc) @ n
    mid = bc + n * (d.max() + d.min()) / 2
    sec = tm.section(plane_origin=mid, plane_normal=n)
    for lp in sec.discrete:
        lo, hi = lp.min(axis=0), lp.max(axis=0)
        if lo[2] > 0.130 and (hi[0] - lo[0]) < 0.015:
            return n, lp.mean(axis=0), lp
    raise RuntimeError("key bow slot loop not found")


class PairProbe:
    """ForceProbe-compatible probe restricted to specific COUNTERPART shapes.

    README pitfall 6.6: SensorContact.total_force sums EVERY contact on the
    sensing body, so a whole-body fingertip probe reads whatever else the
    fingertip is touching.  Measured on this task's own rollout before the
    rework: at the thread the right-hand "grip" probe read 34 N while the
    board probe read 36 N — the number was the fingertip GRINDING THE
    BOARD, not the ring, and the task's own grip check passed on it.
    """

    def __init__(self, model, sensing_body, counterpart_shapes):
        self.sensor = newton.sensors.SensorContact(
            model, sensing_bodies=[sensing_body],
            counterpart_shapes=counterpart_shapes, measure_total=False)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        self.sensor.update(state, contacts)
        F = self.sensor.force_matrix.numpy()[0].sum(axis=0)
        Ff = self.sensor.force_matrix_friction.numpy()[0].sum(axis=0)
        f, fr = float(np.linalg.norm(F)), float(np.linalg.norm(Ff))
        rec = {"t": t, "force": f, "friction": fr, "normal": f - fr,
               "vec": F.tolist()}
        self.history.append(rec)
        return rec


def main():
    rig = GS.make_rig("keychain_hang")
    rig.substeps = 24
    # README pitfall 5.3: arm the SAFE collision cadence at BUILD time so
    # every driver of this scene (the teleop gate replaces control()
    # wholesale) inherits per-substep collision; the expert below RELAXES
    # it to once-per-frame only for the grasp/carry window, which is the
    # phase that needs pitfall 5.1's anchor accumulation.
    rig.collide_every_substep = True

    base_xf = wp.transform(wp.vec3(0.32, 0.0, 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    # smoothness: raise the underdamped arm drives' damping per-task
    for dof in range(7):
        rig.builder.joint_target_kd[dof] = ARM_KD_TASK
    # stiff finger drives for the 2.5 mm tube pinch (utensil pattern)
    for dof in (7, 8):
        rig.builder.joint_target_ke[dof] = FINGER_KE
        rig.builder.joint_target_kd[dof] = FINGER_KD
    # rubberized fingertip pads
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
    # visible mounting pedestal under the arm base (VISUAL ONLY)
    rig.builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(0.32, 0.0, 0.006),
                               wp.quat_identity()),
        hx=0.115, hy=0.115, hz=0.006,
        cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False),
        label="arm_pedestal")

    # ── static scenery: board (raw) + wrench rod (subdivided) ──────────
    def add_static(asset, stem, label, max_edge, mu):
        tm = load_tm(asset, stem)
        if max_edge:
            tm = subdiv(tm, max_edge)
        mesh, c = make_newton_mesh(tm, **SDF_STAT)
        c = np.asarray(c)
        sid = rig.builder.add_shape_mesh(
            -1, mesh=mesh,
            xform=wp.transform(wp.vec3(*c), wp.quat_identity()),
            cfg=rig.cfg(mu=mu), label=label)
        rig.meshes[label] = (tm, c)
        rig.shapes[label] = [sid]
        return c

    add_static("hanging_board", "part1_1_body1", "board", None, BOARD_MU)
    add_static("unidentified", "part1_1_body1", "rod", SUBDIV_ROD, ROD_MU)
    # static-static contact floods the triangle-pair midphase — filter
    statics = list(rig.builder.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            rig.filter_pair(statics[i], statics[j])

    # ── keychain: THREE free bodies at their BAKED REST ────────────────
    cfg_kc = dict(cfg=rig.cfg(mu=KC_MU, gap=GAP_KC))
    ring_raw = load_tm("keyring", "part3_1_body1")
    ring_tm = subdiv(ring_raw, SUBDIV_KC)
    ring_mesh, ring_c = make_newton_mesh(ring_tm, **SDF_KC)
    ring_c = np.asarray(ring_c)
    p_ring0, q_ring0 = BAKED["ring"]
    ring_body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*p_ring0), wp_quat(q_ring0)),
        label="ring")
    rig.shapes["ring"] = [rig.builder.add_shape_mesh(
        ring_body, mesh=ring_mesh, label="ring_shape", **cfg_kc)]
    set_body_mass(rig.builder, ring_body, RING_MASS)
    ring_local = ring_tm.copy()
    ring_local.apply_translation(-ring_c)
    rig.meshes["ring"] = (ring_tm, ring_c)

    key_locals, key_holes = {}, {}
    for name, stem in (("key_a", "part1_1_body1"),
                       ("key_b", "part3_1_body1")):
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
        print(f"{name}: rest start {np.round(pk, 4)}")

    # GelSight pads (no-op unless TB_GELSIGHT=1): mounted with the pile in
    # place; the ring's square 2.5 mm tube is what the pads pinch, so it is
    # the hydroelastic counterpart.  The arm/keychain collision filters
    # below skip finger bodies, so the gel and case stay live vs the pile.
    GS.mount(rig)
    GS.hydroelastic(rig, "ring")

    # ring centerline circle in the ring body frame (CAD normal +x)
    th_c = np.linspace(0, 2 * np.pi, 72, endpoint=False)
    circle_local = np.stack([np.zeros_like(th_c), R_TUBE * np.cos(th_c),
                             R_TUBE * np.sin(th_c)], axis=1)

    # ground plane pairs with every triangle — filter it
    ground_sid = statics[0]
    for b, sids in rig.builder.body_shapes.items():
        for s in sids:
            if s != ground_sid:
                rig.filter_pair(ground_sid, s)

    # The arm works in the air past the bench cut-out and above
    # the board, so table/pedestal pairs are pure triangle-midphase cost —
    # filter them.  board and rod stay LIVE on purpose: a knuckle grinding
    # the wall is the documented failure mode of any grasp in this
    # neighbourhood and it must stay visible in board_N.
    arm_bodies = [b for b, l in enumerate(rig.builder.body_label)
                  if l.startswith("fr3/")]
    keep_live = {rig.shapes["board"][0], rig.shapes["rod"][0]}
    for b in arm_bodies:
        for s in rig.builder.body_shapes.get(b, []):
            for sst in statics:
                if sst not in keep_live:
                    rig.filter_pair(s, sst)

    # ── Franka orientations ────────────────────────────────────────────
    def tcp_quat(p_hat, n_hat):
        z_t = -p_hat
        y_t = n_hat / np.linalg.norm(n_hat)
        x_t = np.cross(y_t, z_t)
        return quat_of_R(np.column_stack([x_t, y_t, z_t]))

    # ── grasp frame, straight off the baked rest ───────────────────────
    # The ring leans 27 deg, so its TOP ARC stands 12 mm clear of the
    # bench.  Pinch THERE, with the closing axis along the ring's own
    # normal: the pads then land on the square section's flat 2.5 mm faces
    # (a round wire would just roll out).  Approach radially outward-and-up
    # from the ring centre, which is the one direction with nothing in it.
    R_ring0 = trimesh.transformations.quaternion_matrix(
        [q_ring0[3], q_ring0[0], q_ring0[1], q_ring0[2]])[:3, :3]
    RING_N = R_ring0 @ np.array([1.0, 0.0, 0.0])   # CAD ring normal is +x
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
    print(f"grasp: ring normal {np.round(RING_N, 3)} "
          f"(tilt {np.degrees(np.arccos(RING_N[2])):.1f} deg), top arc "
          f"{np.round(P_TOP * MM, 2)} mm, pinch dir {np.round(P_HAT_G, 3)}")

    # ── NO REORIENTATION ANYWHERE IN THE MISSION ───────────────────────
    # The ring is carried, threaded and released at the attitude it was
    # picked in.  That is not a convenience, it is the result the physics
    # forced (three instrumented failures, header "WHY THE RING IS NEVER
    # STOOD UP"): at the pile's rest attitude the ring's normal is 27 deg
    # off vertical, so 89 % of the keychain's weight is carried by the pads
    # as a NORMAL contact load; stand the ring vertical and the same pinch
    # has to carry all of it in FRICTION, and it walks out of the pads in
    # about a second no matter how hard it squeezes.
    Q_HANG = Q_GRASP
    P_HAT_H = P_HAT_G
    e2_thr = np.array([1.0, 0.0, 0.0])   # retreat direction: straight +x

    # start parked on the pinch ray, 30 mm further out and 45 mm up
    tcp0 = p_ring0 + (TCP_R + CAGE_STAND + 0.008) * P_HAT_G \
        + np.array([0, 0, 0.045])
    q_start = rig.solve_arm_start(tcp0, Q_GRASP)
    tcp_fk = rig.ik.tcp_pose()
    print(f"arm start TCP err "
          f"{np.linalg.norm(tcp_fk[:3] - tcp0) * MM:.2f} mm")
    # offset from the two fingertip bodies' midpoint to the TCP, in the
    # left finger's frame — lets the rollout measure the ACTUAL TCP (the
    # scene model collapses the hand_tcp frame into link7)
    _lf = rig.ik.body_pose("fr3/fr3_leftfinger")
    _rf = rig.ik.body_pose("fr3/fr3_rightfinger")
    FINGER_TCP_OFF = world_T(_lf)[:3, :3].T @ (
        tcp_fk[:3] - (_lf[:3] + _rf[:3]) / 2)

    # NO stabilizer joint anywhere in this scene — see the header: a d6 from
    # link7 to the ring is what used to eject the pile, and the square-
    # section ring does not need one.
    # arm links never collide with the keychain; the FINGERTIPS DO (the pinch
    # and any incidental key brush are the only real robot-object contacts)
    for kb in (ring_body, *[rig.builder.body_label.index(n)
                            for n in ("key_a", "key_b")]):
        for ts in rig.builder.body_shapes.get(kb, []):
            for bl in arm_bodies:
                if "finger" in rig.builder.body_label[bl]:
                    continue
                for s in rig.builder.body_shapes.get(bl, []):
                    rig.builder.add_shape_collision_filter_pair(s, ts)

    cam_t = np.array([-0.05, 0.05, 0.10])
    rig.finalize(probes=["ring", "key_a", "key_b",
                         "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"],
                 cam_pos=cam_t + np.array([0.30, -0.10, 0.16]),
                 cam_target=cam_t)
    # PAIR-RESOLVED pad forces (README pitfall 6.6).  The whole-body finger
    # probes stay in the CSV as fingertip TOTALS (they are what shows a
    # knuckle grinding the wall), but every control law and every QA gate
    # reads the finger<->RING pair only.
    rig.probes["fingertip_L"] = rig.probes.pop("fr3/fr3_leftfinger")
    rig.probes["fingertip_R"] = rig.probes.pop("fr3/fr3_rightfinger")
    _ring_shapes = rig.shapes["ring"]
    rig.probes["grip_L"] = PairProbe(rig.model, "fr3/fr3_leftfinger",
                                     _ring_shapes)
    rig.probes["grip_R"] = PairProbe(rig.model, "fr3/fr3_rightfinger",
                                     _ring_shapes)
    rod_probe = ForceProbe(rig.model, sensing_shape="rod")
    board_probe = ForceProbe(rig.model, sensing_shape="board")

    GS.make_pipeline(rig, rigid_contact_max=20000,
                     max_triangle_pairs=4_000_000)
    rig.solver = make_solver(rig.model, iterations=15)

    # ── the hang target's own centreline, measured off the board mesh ──
    # The QA test for "the keychain is on the peg" is a LINKAGE test, not a
    # position box: the peg's centreline must pierce the ring's own plane
    # INSIDE the 21 mm hole.  The peg is not straight (it curls up into the
    # hook over its last 13 mm), so the centreline is sampled on 0.5 mm
    # x-slices of the board in the target peg's y-band rather than assumed.
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
    print(f"peg centreline: {len(PEG_CL)} samples, x "
          f"{PEG_CL[0,0]*MM:.1f}..{PEG_CL[-1,0]*MM:.1f} mm, z "
          f"{PEG_CL[:,2].min()*MM:.1f}..{PEG_CL[:,2].max()*MM:.1f} mm")

    ri = rig.body_index("ring")
    ki = {n: rig.body_index(n) for n in ("key_a", "key_b")}
    fli = rig.body_index("fr3/fr3_leftfinger")
    fri = rig.body_index("fr3/fr3_rightfinger")

    st = {"corr": np.zeros(2), "err_f": np.zeros(2),
          "u_ins": 0.0, "u_hold": None, "grasp": None,
          "tcp_frozen": None, "g_done": False,
          "q_cmd": np.asarray(q_start[:7], dtype=float).copy(),
          "dq": np.zeros(7), "dq_peak": 0.0, "tcp_cmd": None,
          "tcp_err_peak": 0.0, "droop": np.zeros(3), "droop_hold": None,
          "c_cmd": None, "R_cmd": None, "slip_peak": np.zeros(3),
          "grip_adv": 0.0}

    def drive_smooth(tcp_p, tcp_q, fingers):
        """Jerk-limited joint command (README pitfall 10.1).

        rig.drive_arm writes the raw LM-IK solution straight into the drive
        targets; on a 7-dof arm that solution can slide along the null space
        without moving the TCP at all, and a one-frame slide is an arm whip.
        Here the command tracks the solution under a velocity cap and an
        acceleration cap (with a sqrt braking profile so it does not
        overshoot), and the LIMITED pose is written back as the IK warm
        start so the two never diverge.
        """
        q_sol = rig.ik.solve(tcp_p, tcp_q, iterations=16)
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

    def control(t):
        bq = rig.s0.body_q.numpy()
        ring_p = bq[ri, :3]

        # COLLISION CADENCE, PHASE-DEPENDENT (README pitfalls 5.1/33/39).
        # Per-substep is the build-time default and is what the THREE
        # CONTACT-BREAKING phases need: the pile settle, the KEYS
        # PEELING OFF THE BENCH in the first half of the lift, and the
        # post-release drop.  The peel had to be measured to be
        # believed: key_a's contact force spikes to 0.168 N (3x its own
        # weight) and then reads 0.0 for the rest of the rollout — an
        # impulsive ejection — and it does that at 1.36 mm/frame just
        # as readily as at 2.35.  Formerly this said 'the two
        # FREE-FALL phases need — the pile settle and the post-release drop
        # of the ring onto the peg.  Once-per-frame is what makes the
        # fingertip friction pinch hold, so the expert relaxes to it only
        # between the settle and the release, where nothing free-falls and
        # everything moves < 0.8 mm/frame.
        rig.collide_every_substep = (t < T_PEEL) or (t >= T_RELEASE)

        # quasi-static pile assembly: ramped gravity
        if not st["g_done"]:
            s = float(np.clip(t / T_GRAV, 0.1, 1.0))
            rig.model.gravity.assign(
                np.array([[0.0, 0.0, -9.81 * s * s]], dtype=np.float32))
            if s >= 1.0:
                st["g_done"] = True

        # ── fingers ────────────────────────────────────────────────────
        if t < T_CAGE:
            # ramp OUT of the URDF home gap; a step command here is a
            # one-frame 40 mm finger slam (the classic accel spike)
            fingers = FINGER_HOME + (FINGER_OPEN - FINGER_HOME) * \
                smooth(t / 1.2)
        elif t < T_CLOSE:
            fingers = FINGER_OPEN + (FINGER_CLOSE - FINGER_OPEN) * \
                smooth((t - T_CAGE) / (T_CLOSE - T_CAGE))
        elif t < T_RELEASE:
            fingers = FINGER_CLOSE + (FINGER_TIGHT - FINGER_CLOSE) * \
                smooth((t - T_CLOSE) / (T_TIGHT - T_CLOSE))
            # ADMITTANCE GRIP (README pitfall 7.6).  A fixed finger position
            # holds a fixed INTERFERENCE, not a fixed force: as the ring
            # creeps radially out of the pads the overlap shrinks, the force
            # decays and the creep accelerates — measured 1.8 -> 0.5 N/pad
            # over one carry, ending in the keychain being dropped.  Advance
            # the command to hold the measured PAIR-RESOLVED pad force in a
            # deadband, capped at FINGER_MIN.
            if t >= T_TIGHT and t < T_THREAD and rig.probes["grip_L"].history:
                f = min(rig.probes["grip_L"].history[-1]["force"],
                        rig.probes["grip_R"].history[-1]["force"])
                if f < F_GRIP_TARGET - F_GRIP_DB:
                    st["grip_adv"] += GRIP_STEP
                elif f > F_GRIP_TARGET + F_GRIP_DB:
                    st["grip_adv"] -= GRIP_STEP
                st["grip_adv"] = float(np.clip(
                    st["grip_adv"], 0.0, FINGER_TIGHT - FINGER_MIN))
            fingers -= st["grip_adv"]
            if t >= T_THREAD:
                # BLEED THE SQUEEZE BEFORE LETTING GO.  1.06 mm of pad
                # interference at 5000 N/m stores 2.8 mJ per pad, and on a
                # 5 g ring that is 1.06 m/s if it comes out as a kick: the
                # release flung the keychain 119 mm sideways in 0.45 s and
                # it never reached the hook.  Relax to FINGER_TOUCH (the
                # measured just-touching gap, zero interference) first, then
                # open — the ring is then simply let go, not ejected.
                fingers += (FINGER_TOUCH - fingers) * smooth(
                    (t - T_THREAD) / (T_RELEASE - T_THREAD))
        else:
            fingers = FINGER_TOUCH + (FINGER_RELEASE - FINGER_TOUCH) * \
                smooth((t - T_RELEASE) / 0.3)

        # ── measured-pose helpers ──────────────────────────────────────
        if st["grasp"] is None and t >= T_SETTLE:
            # freeze the grasp target: the MEASURED ring centre.  The whole
            # trajectory is written in ring-centre space and the pinch
            # offset is carried by TCP_R * p_hat.
            st["grasp"] = ring_p.copy()
            print(f"t={t:.2f}s grasp target: ring centre "
                  f"{np.round(ring_p * MM, 2)} mm "
                  f"(baked {np.round(p_ring0 * MM, 2)})")
        g = st["grasp"]

        # ── TCP trajectory ─────────────────────────────────────────────
        F_env = (rod_probe.history[-1]["force"]
                 + board_probe.history[-1]["force"]) \
            if rod_probe.history else 0.0
        tcp_q = Q_GRASP
        if t < T_APPR:
            # park, then a single smoothstep in along the pinch ray
            # (attitude is already Q_GRASP, so nothing rotates near the pile)
            appr = (g if g is not None else p_ring0) \
                + (TCP_R + CAGE_STAND) * P_HAT_G
            s = smooth((t - T_SETTLE) / (T_APPR - T_SETTLE)) \
                if t >= T_SETTLE else 0.0
            tcp_p = tcp0 + s * (appr - tcp0)
        else:
            # edge pinch onwards: ring-center-space waypoints
            c_pin = g.copy()
            lift = c_pin + [0, 0, 0.135 - c_pin[2]]
            wpts = [
                (T_APPR, c_pin),
                (T_CAGE, c_pin),
                (T_TIGHT, c_pin),
                (T_LIFT, lift),
                (T_TRANS, np.array([-0.0700, THREAD_XY[1], lift[2]])),
                (T_DWELL, np.array([-0.0700, THREAD_XY[1], lift[2]])),
                (T_HOVER0, np.array([-0.0700, THREAD_XY[1], HOVER_Z])),
                (T_HOVER, np.array([-0.0700, THREAD_XY[1], HOVER_Z])),
                (T_THREAD, np.array([*THREAD_XY, THREAD_Z])),
                (T_END, np.array([*THREAD_XY, THREAD_Z])),
            ]

            def c_des(tt):
                if tt <= T_APPR:
                    return wpts[0][1]
                for (ta, pa), (tb, pb) in zip(wpts, wpts[1:]):
                    if tt <= tb:
                        s = 0.0 if tb == ta else np.clip(
                            (tt - ta) / (tb - ta), 0, 1)
                        s = smooth(s)
                        return pa + s * (pb - pa)
                return wpts[-1][1]

            # ── the ONE reorientation: 72 deg, in place, at height ──────
            # Rotating a held ring is the risky manoeuvre in this task (an
            # earlier version fired the cluster out at 14 m/s doing it while
            # translating).  It is done here at a standstill over the bench,
            # C1-smoothstepped, and the ring's centre is HELD so the pads
            # only rotate about the pinch point.
            tcp_q = Q_GRASP
            p_hat = P_HAT_G

            # cage standoff: the pads ride in along the pinch ray, so the
            # waypoints stay TRUE ring-centre targets and the closed loop
            # below never sees the standoff as a tracking error
            stand = CAGE_STAND * (1.0 - smooth(
                (t - T_APPR) / (T_CAGE - T_APPR))) if t < T_CAGE else 0.0

            # closed-loop (y, z) on the measured ring centre (EMA +
            # deadband) — the two axes perpendicular to the -x insertion.
            # Only armed from the hover: the grasp starts from a baked rest
            # whose run-to-run drift measures 0.2 mm, and an integrator on
            # a 2.5 mm pinch target is all downside there.
            des_now = c_des(t)
            err = des_now[:2] - ring_p[:2]
            st["err_f"] = st["err_f"] + CORR_ALPHA * (err - st["err_f"])
            e = st["err_f"]
            e = np.sign(e) * np.maximum(np.abs(e) - CORR_DB, 0.0)
            # armed from the lateral approach: the grasp creeps a
            # sub-millimetre amount over the carry and the hook has to land
            # inside a 21 mm hole, so the command follows the MEASURED part
            # (pitfall 8.1) from the moment the approach starts
            if T_HOVER0 <= t < T_RELEASE:
                st["corr"] += np.clip(CORR_GAIN * e, -CORR_STEP,
                                      CORR_STEP)
                st["corr"] = np.clip(st["corr"], -0.008, 0.008)

            # Force-gated INSERTION (admittance).  The fingers clear the
            # board by >= 4.5 mm everywhere on this corridor, so any board
            # force during the insertion is the RING touching the hook —
            # exactly the signal that should throttle the slide.
            # NO SNAP: the warp time never jumps to the segment end.  The
            # previous version did (`des = wpts[-1]` past T_THREAD) and that
            # single line was a 6.5 mm one-frame step in the commanded ring
            # centre — it drove the fingertip into the board at 37 N.
            if T_HOVER < t and st["u_ins"] < 1.0 and st["u_hold"] is None:
                gate = float(np.clip((2.5 - F_env) / (2.5 - 0.8),
                                     0.15, 1.0))
                st["u_ins"] = min(1.0, st["u_ins"] + gate
                                  / ((T_THREAD - T_HOVER) * rig.fps))
            if t >= T_RELEASE and st["u_hold"] is None:
                st["u_hold"] = st["u_ins"]
                print(f"t={t:.2f}s thread travel {st['u_ins']*100:.1f} % "
                      f"of {(HOVER_Z - THREAD_Z) * MM:.1f} mm")
            u = st["u_hold"] if st["u_hold"] is not None else st["u_ins"]
            t_warp = t if t <= T_HOVER else T_HOVER + u * (T_THREAD - T_HOVER)
            des = c_des(t_warp)

            if t < T_SWING:
                tcp_p = des + (TCP_R + stand) * p_hat \
                    + np.array([*st["corr"], 0.0])
                st["tcp_frozen"] = tcp_p.copy()
            else:
                # TWO-LEG RETREAT, and the order is not cosmetic: at the
                # release height the lower pad's bottom face sits at TCP z
                # -9.5 mm = 160.1 mm, and the hook crest the insertion just
                # threaded is 168.22 mm, so a straight +x retreat drags the
                # pad 8 mm THROUGH the hook.  Rise clear of the board first
                # (board top 181.0), then translate out.
                s1 = np.clip((t - T_SWING) / (T_RETREAT - T_SWING), 0, 1)
                r_hat = np.array([p_hat[0], p_hat[1], 0.0])
                r_hat /= np.linalg.norm(r_hat)
                clear = smooth(float(np.clip(s1 / 0.30, 0.0, 1.0)))
                up = smooth(float(np.clip((s1 - 0.30) / 0.35, 0.0, 1.0)))
                out = smooth(float(np.clip((s1 - 0.65) / 0.35, 0.0, 1.0)))
                tcp_p = st["tcp_frozen"] \
                    + clear * RETREAT_CLEAR * r_hat \
                    + up * RETREAT_UP * np.array([0.0, 0.0, 1.0]) \
                    + out * RETREAT_D * e2_thr
            # GRASP SLIP INSTRUMENT: where the ring actually is relative to
            # where a rigid pad-ring relation would put it, resolved in the
            # tool frame (tangential / axial-along-the-pinch-normal /
            # radial).  A friction pinch on a RING can only really escape
            # RADIALLY -- tangential creep is just the ring spinning between
            # the pads -- so these three numbers say which failure it is.
            st["c_cmd"] = des + np.array([0.0, *st["corr"]])
            st["R_cmd"] = R_of_q(tcp_q)
        # ── droop compensation (README pitfall 8.1) ─────────────────────
        # This arm sags 1.5-2 mm at the reach the bench forces, and the
        # pinch lands on a 2.5 mm face with 3.2 mm of pad clearance: a
        # 1.2 mm AXIAL offset makes the upper pad touch first and shove
        # the ring off the pile (measured: ring pushed 3.6 mm radially,
        # pads then closed on air).  Integrate the measured TCP error into
        # the command, then FREEZE it at the hover so the force-gated
        # thread descent is not fighting an active integrator.
        tcp_act = (bq[fli, :3] + bq[fri, :3]) / 2 \
            + world_T(bq[fli])[:3, :3] @ FINGER_TCP_OFF
        # ...and ONLY while the command is HOLDING STILL.  This integrator
        # measures "commanded minus measured TCP", and during a transport
        # leg that quantity is dominated by the jerk-limited joint tracker's
        # own dynamic lag (a second-order tracker running at v settles
        # v^2/2a behind, here ~2 deg = 10 mm).  Integrating THAT is the
        # lead-clamp runaway of docs/teleop_guard.md bug 1 in a different
        # costume: the command leads, the arm chases, the lag grows, and the
        # measured TCP rang +-10 mm at ~0.7 Hz through the transit and shook
        # the keychain out of the pinch.  Droop is a STATIC quantity; only
        # measure it when the command is static.
        cmd_step = 0.0 if st["tcp_cmd"] is None else float(
            np.linalg.norm(tcp_p - st["tcp_cmd"]))
        if st["tcp_cmd"] is not None and t >= T_SETTLE:
            if t < T_HOVER and cmd_step < 5.0e-5:
                derr = st["tcp_cmd"] - tcp_act
                st["droop"] = np.clip(
                    st["droop"] + np.clip(DROOP_GAIN * derr,
                                          -DROOP_STEP, DROOP_STEP),
                    -DROOP_MAX, DROOP_MAX)
            elif t >= T_HOVER and st["droop_hold"] is None:
                st["droop_hold"] = st["droop"].copy()
                print(f"t={t:.2f}s droop lead frozen at "
                      f"{np.round(st['droop'] * MM, 2)} mm")
        st["tcp_cmd"] = tcp_p.copy()
        drive_smooth(tcp_p + st["droop"], tcp_q, fingers)

    def key_ring_angle(bq, name):
        Tk = world_T(bq[ki[name]])
        Tr = world_T(bq[ri])
        blade_r = Tr[:3, :3].T @ (Tk[:3, :3] @ np.array([0, 0, -1.0]))
        return np.degrees(np.arccos(np.clip(-blade_r[2], -1, 1)))

    def peg_link(bq_row_ring):
        """Closest piercing of the PEG CENTRELINE through the ring's plane.

        "The keychain is on the peg" is a linkage statement: the peg's
        centreline must cross the ring's own plane inside the ring's 21 mm
        hole.  Returns (offset from the ring centre in m, x of the piercing
        point, |n_x| of the ring).  Offset 9.9 means no crossing at all.
        """
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
        gl = rig.probes["grip_L"].history[-1]["force"]
        gr = rig.probes["grip_R"].history[-1]["force"]
        tl = rig.probes["fingertip_L"].history[-1]["force"]
        tr = rig.probes["fingertip_R"].history[-1]["force"]
        off, px, nx = peg_link(bq[ri])
        if st["c_cmd"] is not None:
            dv = st["R_cmd"].T @ (bq[ri, :3] - st["c_cmd"])
            slip = np.array([dv[0], dv[1], -dv[2]])   # tang / axial / radial
            st["slip_peak"] = np.maximum(st["slip_peak"], np.abs(slip))
        else:
            slip = np.zeros(3)
        tcp_act = (bq[fli, :3] + bq[fri, :3]) / 2 + \
            (world_T(bq[fli])[:3, :3] @ FINGER_TCP_OFF)
        if st["tcp_cmd"] is not None:
            st["tcp_err_peak"] = max(st["tcp_err_peak"],
                                     float(np.linalg.norm(
                                         tcp_act - st["tcp_cmd"])))
        if int(round(t * rig.fps)) % 15 == 0:
            de = (tcp_act - st["tcp_cmd"]) * MM \
                if st["tcp_cmd"] is not None else np.zeros(3)
            print(f"  [{t:5.2f}] grip {gl:5.2f}/{gr:5.2f} (tip "
                  f"{tl:5.2f}/{tr:5.2f}) N  ring {np.round(bq[ri, :3]*MM, 1)}"
                  f"  slip[t/a/r] {np.round(slip*MM, 2)}"
                  f"  peg_off {off*MM:6.2f}@x{px*MM:7.1f} nx {nx:.2f}"
                  f"  tcp_err {np.round(de, 2)}  board {b['force']:.2f}")
        return {
            "ncon": int(rig.contacts.rigid_contact_count.numpy()[0]),
            "rod_N": round(r["force"], 4),
            "board_N": round(b["force"], 4),
            "grip_min_N": round(min(gl, gr), 3),
            "tip_max_N": round(max(tl, tr), 3),
            "ring_x_mm": round(bq[ri, 0] * MM, 1),
            "ring_y_mm": round(bq[ri, 1] * MM, 1),
            "ring_z_mm": round(bq[ri, 2] * MM, 1),
            "peg_offset_mm": round(off * MM, 2),
            "slip_radial_mm": round(float(slip[2]) * MM, 3),
            "slip_tang_mm": round(float(slip[0]) * MM, 3),
            "key_a_rel_deg": round(key_ring_angle(bq, "key_a"), 1),
            "key_b_rel_deg": round(key_ring_angle(bq, "key_b"), 1),
        }

    rec = rig.run(T_END, control, extra)
    paths = rec.save("rollout",
                     force_keys=["ring_N", "rod_N", "grip_L_N",
                                 "grip_R_N", "key_a_N", "key_b_N"])
    print("saved:", paths)

    # ── QA ─────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    d = np.load(paths["states"], allow_pickle=True)
    t_arr = d["t"]
    bq_all = d["body_q"]

    # velocity gate recomputed AFTER the pile-assembly settle: the ramped
    # scene-prep phase legitimately pops mm-scale wedges (excluded window)
    dt = np.diff(t_arr)
    dt[dt <= 0] = np.inf
    disp = np.linalg.norm(np.diff(bq_all[..., :3], axis=0), axis=-1)
    fd_vel = disp / dt[:, None]
    m_post = t_arr[:-1] >= T_SETTLE
    peak_fd = float(fd_vel[m_post].max())
    labels = list(d["body_labels"])
    ok_vel = bool(np.isfinite(peak_fd) and peak_fd < 2.5)
    qa.report["checks"]["velocity_spikes"] = {
        "pass": ok_vel,
        "peak_displacement_velocity_m_s": round(peak_fd, 3),
        "worst_body": labels[int(
            fd_vel[m_post].max(axis=0).argmax())],
        "note": "flying-object detector, evaluated after the ramped "
                "pile-assembly settle (t >= T_SETTLE)",
    }
    qa.report["pass"] = bool(all(
        v["pass"] for v in qa.report["checks"].values()))

    def window(t0, t1):
        m = (t_arr >= t0) & (t_arr <= t1)
        fs = np.flatnonzero(m)
        z = float(bq_all[m, ri, 2].mean())
        off = np.array([peg_link(bq_all[f, ri])[0] for f in fs])
        px = np.array([peg_link(bq_all[f, ri])[1] for f in fs])
        nx = np.array([peg_link(bq_all[f, ri])[2] for f in fs])
        rod = float(np.mean([r["force"] for r in rod_probe.history
                             if t0 <= r["t"] <= t1]))
        brd = float(np.mean([r["force"] for r in board_probe.history
                             if t0 <= r["t"] <= t1]))
        return dict(z=z, off=float(off.max()), px_lo=float(px.min()),
                    px_hi=float(px.max()), nx=float(nx.min()),
                    rod=rod, brd=brd)

    # There is no stabilizer to hold anything up: from the moment the pads
    # open, the keychain is carried by the board alone.  Both windows (hand
    # still retreating / hand fully clear at the end) are GATED, on the same
    # criterion — the PEG must pierce the ring's own plane inside the ring's
    # 21 mm hole, at an x inside the peg's own extent, and the board must be
    # carrying at least 80 % of the keychain's weight.
    W_S = window(T_RETREAT + 0.2, T_FREE)
    W_F = window(T_END - 0.8, T_END)

    def hung(w):
        # the peg centreline pierces the ring's plane inside the hole (a
        # 1 mm margin on the 10.5 mm hole radius), at an x that is on the
        # peg, and the board carries >= 80 % of the keychain's weight
        return bool(w["off"] < R_ID - 0.001 and PEG_X0 <= w["px_lo"]
                    and w["px_hi"] <= PEG_X1
                    and (w["rod"] + w["brd"]) >= 0.8 * W_TOTAL)

    ok_hang = hung(W_S)
    still_hung = hung(W_F)

    def link_at(f, name):
        Tk = world_T(bq_all[f, ki[name]])
        Tr = world_T(bq_all[f, ri])
        loop_w = (Tk[:3, :3] @ key_holes[name].T).T + Tk[:3, 3]
        circ_w = (Tr[:3, :3] @ circle_local.T).T + Tr[:3, 3]
        return gauss_link(loop_w, circ_w)

    f_hang = int(np.searchsorted(t_arr, T_FREE - 0.1))
    links = {n: {"start": round(link_at(0, n), 3),
                 "hang": round(link_at(f_hang, n), 3),
                 "end": round(link_at(len(t_arr) - 1, n), 3)}
             for n in ("key_a", "key_b")}
    ok_linked = all(abs(abs(v[k]) - 1.0) < 0.25
                    for v in links.values() for k in ("hang", "end"))

    swings = {}
    ok_swing = True
    for n in ("key_a", "key_b"):
        ang = np.array([r[f"{n}_rel_deg"] for r in rec.force_rows])
        tf = np.array([r["t"] for r in rec.force_rows])
        m = (tf >= T_TIGHT) & (tf <= T_RELEASE)
        rng = float(ang[m].max() - ang[m].min())
        swings[n] = round(rng, 1)
        ok_swing &= rng > 10.0

    tg = np.array([r["t"] for r in rig.probes["grip_L"].history])
    gl = np.array([r["force"] for r in rig.probes["grip_L"].history])
    gr = np.array([r["force"] for r in rig.probes["grip_R"].history])
    # ends at T_THREAD, not T_RELEASE: the last 0.5 s is a DELIBERATE
    # relax to zero interference (see the finger schedule), so taking a
    # min through it would score the controlled let-go as a lost grip
    mg = (tg >= T_TIGHT) & (tg <= T_THREAD)
    grip_min = float(np.minimum(gl, gr)[mg].min())
    grip_mean = float(((gl + gr) / 2)[mg].mean())
    ok_grip = grip_min > 0.5

    ok = bool(ok_hang and still_hung and ok_linked and ok_swing and ok_grip)

    def hang_block(w, note):
        return {
            "pass": bool(hung(w)),
            "ring_z_mm": round(w["z"] * MM, 1),
            "expected_rest_z_mm": round(HANG_Z * MM, 1),
            "peg_pierce_offset_mm": round(w["off"] * MM, 2),
            "ring_hole_radius_mm": round(R_ID * MM, 2),
            "peg_pierce_x_mm": [round(w["px_lo"] * MM, 1),
                                round(w["px_hi"] * MM, 1)],
            "peg_x_extent_mm": [round(PEG_X0 * MM, 1), round(PEG_X1 * MM, 1)],
            "ring_normal_x": round(w["nx"], 3),
            "rod_contact_N": round(w["rod"], 4),
            "board_contact_N": round(w["brd"], 4),
            "note": note,
        }

    qa.report["checks"]["task_success"] = {
        "pass": ok,
        "hang_phase": hang_block(
            W_S, "the peg pierces the ring's own plane INSIDE the 21 mm "
                 "hole, within the peg's x extent, and the board carries "
                 ">= 80 % of the keychain weight"),
        "keys_linked_gauss": {**links, "pass": bool(ok_linked),
                              "note": "|L|=1 <=> the key's bow slot still "
                                      "encircles the ring tube"},
        "keys_free_swing": {
            **swings, "pass": bool(ok_swing),
            "note": "key-vs-ring relative blade angle range during the "
                    "carry must exceed 10 deg (contact-hung, not welded)"},
        "grip": {
            "min_N": round(grip_min, 3), "mean_N": round(grip_mean, 3),
            "pass": bool(ok_grip),
            "admittance_advance_mm": round(float(st["grip_adv"]) * MM, 3),
            "peak_slip_tang_axial_radial_mm": [
                round(float(st["slip_peak"][0]) * MM, 2),
                round(float(st["slip_peak"][1]) * MM, 2),
                round(float(st["slip_peak"][2]) * MM, 2)],
            "note": "min(left,right) PAIR-RESOLVED fingertip<->ring force "
                    "(pitfall 6.6), close->relax; the 2.5 mm square tube "
                    "is pinched between the pads.  peak_slip is the ring's "
                    "displacement from a rigid pad-ring relation, in the "
                    "tool frame: RADIAL is the one that ends in a drop "
                    "(the pad's radial window on the ring is 5.8-24.3 mm "
                    "against a tube at 10.5-13.0)"},
        "final_hang": hang_block(
            W_F, "last 0.8 s, arm fully clear: the keychain must still be "
                 "on the peg with its whole weight carried by the board"),
        "keychain_weight_N": round(W_TOTAL, 4),
    }
    qa.report["pass"] &= ok

    # smoothness gate (stability agent; windows adapted to this protocol)
    dtf = float(np.median(np.diff(t_arr)))
    arm_ix = [i for i, lbl in enumerate(qa.labels)
              if lbl.startswith("fr3/")]
    P = qa.body_q[:, arm_ix, :3]
    V = np.diff(P, axis=0) / dtf
    A = np.linalg.norm(np.diff(V, axis=0) / dtf, axis=-1)
    free_m = np.ones(len(t_arr), bool)
    # excluded: the scene-prep settle and the two CONTACT events (pads
    # biting the tube, pads letting go) — real impulses, not command
    # discontinuities
    free_m[(t_arr < T_SETTLE) | (np.abs(t_arr - T_RELEASE) < 0.2)
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
        return float(np.sqrt(
            (np.linalg.norm(resid, axis=1) ** 2).mean()))

    j_hover = _jitter_rms(T_HOVER0 + 0.1, T_HOVER)
    j_hang = _jitter_rms(T_RETREAT + 0.2, T_FREE)
    smooth_ok = j_hover < 0.0015 and j_hang < 0.0015 and peak_a < 15.0
    qa.report["checks"]["smoothness"] = {
        "pass": bool(smooth_ok),
        "tcp_jitter_hover_rms_mm": round(j_hover * MM, 3),
        "tcp_jitter_hang_hold_rms_mm": round(j_hang * MM, 3),
        "peak_arm_accel_free_m_s2": round(peak_a, 2),
        "peak_arm_accel_all_m_s2": round(float(A.max()), 2),
        "peak_joint_cmd_step_rad": round(st["dq_peak"], 5),
        "note": "link7 0.25s-detrended jitter RMS < 1.5 mm in the "
                "arrive-hover and post-release hold windows; peak fr3-link "
                "accel < 15 m/s^2 outside settle / finger close / finger "
                "release",
    }
    qa.report["pass"] &= smooth_ok

    # penetration: keys vs ring at 0.3 mm (the flagship contact detail),
    # keychain bodies vs rod/board at the standard 1.5 mm
    from trimesh.proximity import ProximityQuery
    pq_ring = ProximityQuery(ring_local)
    frames = np.linspace(0, len(t_arr) - 1, 24).astype(int)
    pen = {}
    ok_pen = True
    for n in ("key_a", "key_b"):
        sub = key_locals[n].vertices[::4]
        worst, worst_t = -9.0, None
        for f in frames:
            Tk = world_T(bq_all[f, ki[n]])
            Tr = world_T(bq_all[f, ri])
            M = np.linalg.inv(Tr) @ Tk
            pts = (M[:3, :3] @ sub.T).T + M[:3, 3]
            mm = ((pts > ring_local.bounds[0] - 0.003)
                  & (pts < ring_local.bounds[1] + 0.003)).all(axis=1)
            if not mm.any():
                continue
            sd = float(pq_ring.signed_distance(pts[mm]).max())
            if sd > worst:
                worst, worst_t = sd, float(t_arr[f])
        entry = {"max_depth_mm": round(worst * MM, 3), "at_t": worst_t,
                 "pass": bool(worst < 0.0003)}
        pen[f"{n}->ring"] = entry
        ok_pen &= entry["pass"]
    qa.report["checks"]["key_ring_penetration"] = {
        "pass": bool(ok_pen), "tolerance_mm": 0.3, "pairs": pen}
    qa.report["pass"] &= ok_pen

    for lbl_b, loc in (("ring", ring_local),
                       ("key_a", key_locals["key_a"]),
                       ("key_b", key_locals["key_b"])):
        qa.check_penetration_static(lbl_b, loc, *rig.meshes["rod"],
                                    f"{lbl_b}->rod", tol=0.0015)
        qa.check_penetration_static(lbl_b, loc, *rig.meshes["board"],
                                    f"{lbl_b}->board", tol=0.0015)
    # tactile stacks + gel gates (no-op unless TB_GELSIGHT=1): the loaded
    # window matches the QA's own grip window (close -> thread; the last
    # 0.5 s is a deliberate relax), cross-checked against the same
    # pair-resolved grip_L/grip_R probes every gate reads
    GS.save_tactile(
        rig, [GS.object_spec(rig, "ring")], qa=qa,
        window=(T_TIGHT, T_THREAD),
        probe_forces={
            "gel_left": [r["force"] for r in rig.probes["grip_L"].history],
            "gel_right": [r["force"] for r in rig.probes["grip_R"].history]})
    rig.finish(qa)


if __name__ == "__main__":
    wp.init()
    main()
