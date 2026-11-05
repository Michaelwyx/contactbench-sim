"""Task: FRANKA picks a card from the card holder.

Current collision cadence: refresh contacts every physics substep, including
after settling and during the slide.  The speed governor described in the
historical XPBD notes below reused stale geometry under MuJoCo, letting a
pressed card rotate into its neighbours before the next collision check.
Cards now use their true CAD thickness with a single set of analytic slot
supports.  Both FR3 fingers receive the same target (the imported mimic
constraint remains enabled), and closure follows the measured pack height.

Asset: card_holder.  The holder is a stand with a 14 mm wide, 86.5 mm long
slot (floor z=2 mm, mid walls to z=17 mm, tall U-brackets at both slot ends
to z=37 mm).  The card_N_body1.stl files are all byte-identical: Fusion
flattened the instance transforms, so each file contains ALL TEN 1.2 mm
card slabs packed with zero gap (12 mm AABB).  A single card is extracted
as the first slab sub-mesh (1.2 x 85 x 55 mm) and instanced explicitly.

Scene: FOUR cards leaning as ONE BLOCK at 13 deg, faces in contact, bottom
edges on the slot floor, the front card's +x face resting on the z=37
slot-end bracket corners — the way a half-full card holder actually stands.
`deck_pose` builds that in closed form; the justification is measured, see
"THE DECK WAS NEVER AT REST" below.

Why the expert must SLIDE before pinching — measured FR3 finger collision
geometry: the fingertip is 18.5 mm tall and 15.2 mm thick behind the pad
face.  A direct pinch on the outermost card of a packed deck is impossible
(2-6 mm of air behind it), and the deck's lean only cantilevers the front
card ~11 mm past its neighbour (the U-bracket columns cap the lean at
~13 deg via their z=37 top corners against the card's +x FACE) — still
less than one fingertip.  So: PRESS one fingertip pad on the front card's
+x face (the other finger swung wide open in free air), then FRICTION-SLIDE
it UP ITS OWN PLANE (pad-card mu 0.62 beats card-card 0.25, and each pack
card is held down by its own weight, so only the pressed card climbs) until
its top edge stands ~13 mm proud of the deck.  Then the second finger
sweeps in OVER the pack tops, the 1.2 mm card is sandwiched between the
pads (near-zero commanded gap), and the card is pulled out along a
diagonal.  Its bottom edge rides up the +x bracket inner faces on the way
out — the 2-3 N bracket-ride force signature of the original task is
preserved, force-gated (admittance) on the card's measured contact force.
The wrist stabilizer d6 carries NO gains (crutch rule).  The other cards
must stay put.

THE GRASP IS TILTED TO THE DECK and every press/slide/lift command is
written in the deck's own (a, b) frame — a along the card's face normal,
b up the card's own plane.  A VERTICAL pad on a 13 deg face touches along
one pad edge (the face moves 18.5*tan13 = 4.3 mm in x across the pad's
height), which makes the "press" a torque about a pad corner.  That torque
is what broke the task; see the root cause below.

═══════════════════════════════════════════════════════════════════════
ROOT CAUSE OF "the cards stick together / pick one and the rest float up"
(2026-08-13, measured on Delta; this is the user-reported defect)
═══════════════════════════════════════════════════════════════════════

THE CARDS ARE NOT STUCK TO EACH OTHER.  Measured directly with the robot
DELETED from the scene (`card_probe_deck.py CP_MODE=settle`, per-counterpart
SensorContact force matrix, 4 cards x 5 counterparts): at rest each card
carries its own 0.147 N of weight straight into the HOLDER and the largest
card-to-card vertical force anywhere in the deck is 0.008 N.  Refuted with
it: contact-buffer saturation (peak 706 contacts of 40000), and mesh
instancing (one shared newton.Mesh vs four separate ones changes nothing —
CP_SHARED=0 settles the same way).

WHAT ACTUALLY HAPPENS IS TUNNELLING.  The picked card is driven THROUGH
the 1.5 mm-thick +x slot wall and left impaled on it:

  * the slot's side walls are 1.5 mm thick (inner faces x = -37.0 / -23.0,
    outer -38.5 / -21.5), measured by containment-sampling the holder mesh;
  * at the LOAD->SLIDE handover the picked card's bottom edge swung +x from
    -29.7 mm to -21.8 mm in 0.1 s — 8.0 mm, i.e. 1.33 mm PER FRAME — while
    the card rotated 19 deg PAST vertical;
  * the contact gap was 0.4 mm/shape (gap_sum 0.8 mm) and the collision
    cadence was once-per-frame, so 1.33 mm/frame crosses the entire wall
    inside ONE collide() interval and no contact is ever generated;
  * from t = 3.5 s to the end of the rollout ~11 % of the card's vertices
    (329/3000 sampled) sit INSIDE the holder, reaching x = -20.58 mm, i.e.
    2.4 mm past the wall's inner face.  The dense audit reads it as
    card_picked<->holder 2.506 mm.

That impalement IS the ">1.3 N restraint" the previous round was hunting.
The card is not held down by friction it can beat; it is nailed to the
holder through a wall, which is why the press-force arithmetic never
predicted the behaviour and why buying more press force never helped.  It
is README pitfall 6.1 in its purest form: the force channel reads "nothing
is holding this" precisely because the contact was skipped.

THE COUPLING IS A CADENCE ARTIFACT, AND HERE IS THE A/B THAT PROVES IT.
`card_probe_deck.py CP_MODE=lift` makes card_picked KINEMATIC and drives it
out of the deck at 20 mm/s -- an infinitely strong hand taking exactly one
card, with the robot, the expert and the grasp all removed from the
question.  Same scene, same speed, ONLY the collision cadence changed:

                        once-per-frame      per-substep
    card_a  rise          +13.72 mm          +0.28 mm
    card_b  rise          +13.18 mm          +0.28 mm
    card_c  rise          +25.39 mm          -0.04 mm
    picked <-> card_c     +0.452 N           +0.0044 N   (signed z force)

Re-measured on the SHIPPED scene (deck at its equilibrium on the -x
bracket, gap 0.8 mm), the same A/B still separates cleanly, and the holder
reaction is the giveaway:

                        once-per-frame      per-substep
    card_a  rise           +4.07 mm          -0.64 mm
    card_b  rise           -2.88 mm          -0.64 mm
    card_c  rise          +23.23 mm          -0.64 mm
    peak card speed      243 mm/s            33 mm/s
    card_c <-> holder     -29.36 N          +0.139 N   (signed z force)

29.4 N on a 15 g card is 200x its own weight.  Nothing in this scene can
apply 29 N; that is the reaction of an overlap the solver never resolved,
which is precisely why the card leaves the holder.  The governor's 0.6 mm
per frame threshold sits an order of magnitude below the 243 mm/s = 4 mm
per frame these events reach, so it catches them.

A card weighs 0.147 N, so at once-per-frame the picked card was dragging its
neighbour with THREE TIMES the neighbour's own weight, and that would need
0.452/0.25 = 1.8 N of face normal force in a stack whose measured face
normals are ~0.03 N.  It is not friction.  Dropping the lift to 5 mm/s
(rate-independent if it were friction) gives -0.25 mm and 0.0011 N, and
running only two cards gives -1.1 mm and 0.0067 N.  That is why the fix is
a SPEED-GOVERNED cadence and not a friction or mass re-tune.

AND IT IS THE SAME DEFECT AS THE USER'S "the rest float up".  A card lodged
in a wall gets a positional correction once per frame along a direction
that does not reduce the overlap, so it RATCHETS.  Measured in an earlier
rollout: card_c rose from z = 31.4 mm to 53.6 mm — 22 mm, out of the
holder, with NOTHING within 40 mm of it — while its reported linear
velocity was exactly 0.000 m/s for the whole climb (the motion is one kick
per frame at the first substep, so the last substep of every frame reads
zero).  It ended with its centre at y = +19 mm, which is 13 mm past the
+y slot end wall: geometrically unreachable without passing through the
holder.  A body that moves at zero velocity is not being pushed, it is
being teleported by unresolved overlap.

THE DECK WAS NEVER AT REST EITHER.  The old scene stood three cards
perfectly VERTICAL on their 1.2 mm bottom edges in a 14 mm slot — a
knife-edge balance.  With the robot removed they topple: peak 87 mm/s
(1.45 mm/frame, 3.6x the old contact gap), +5.4..6.5 mm of deck drift,
coming to rest only at t = 1.5 s in a 13.2/13.2/12.3/12.3 deg leaning
block.  The old T_SETTLE was 0.7 s, so the last 0.8 s of that topple ran
at once-per-frame cadence — a second tunnelling window — and the expert's
press stop X_STOP was keyed to a wall-compacted VERTICAL deck that never
existed, putting its nominal plane ~8 mm inside the card's real face.
That over-travel, delivered through a ~20 N/mm arm by a vertical pad
edge-loading a leaning card, is the torque that flung the bottom edge.

THE FOUR FIXES (all in this file):
  1. `deck_pose` starts the deck AT its measured equilibrium, so there is
     no toppling transient at all and the press geometry is keyed to a
     deck that exists.  Closed form, verified against the holder mesh per
     card, and it reproduces the measured settle to 0.3 mm.
  2. GAP 0.4 -> 0.8 mm per shape.  gap_sum 1.6 mm now EXCEEDS the 1.5 mm
     wall, so a card can never be clear of both wall faces at once.  Safe:
     `gap` is a broad-phase detection distance only — newton's XPBD
     contact is `d = dot(n, p1-p0) - (margin0+margin1)` with an early
     return on `d >= 0`, and `ShapeConfig.margin` is 0.0 here (pitfall 3.1),
     so a wider gap generates contacts EARLIER and force NEVER at
     separation.
  3. The collision cadence is SPEED-governed, not phase-governed.
     Pitfall 33 states the condition ("safe while per-frame travel stays
     well inside the contact gap") and the old code tested the PHASE
     instead.  It is now measured on the free cards every frame.
  4. The push ends on the deck's own face coordinate and is clamped to the
     measured face, so the ram is gone; and the grasp is tilted to the
     deck so the press is a face press instead of an edge torque.

HONEST-GRASP RE-TUNE (2026-08-13, after the friction-anchor fix, README
pitfall 5.1).  This task is the one most transformed by the fix, because its
whole expert IS friction.  Measured, not redesigned:

  1. THE SETTLE NO LONGER TOPPLES THE DECK, and that invalidates a load-
     bearing assumption of the old design.  Once-per-frame collision is
     unsafe for free FALL (pitfall 5.2): a card released from rest drops
     1.36 mm in its first frame, 3.4x the 0.4 mm contact gap.  With the
     cadence made phase-dependent the four cards stand perfectly still
     (measured lean 0.00 deg for the entire rollout) -- *** THAT CLAIM WAS
     FALSE AND THE MEASUREMENT THAT MADE IT WAS BROKEN: `lean_deg` was
     logged through np.clip(theta, TILT_MIN, TILT_MAX) with both bounds
     0.0, so it read 0.00 unconditionally.  The deck topples; see "THE DECK
     WAS NEVER AT REST" above, measured with the robot removed. *** -- but
     the press geometry was keyed to the old behaviour, "four free cards
     standing in a 14 mm slot TOPPLE +x onto the slot-end bracket (~10 deg,
     7 mm of drift at the press height)".  The compacted deck now ends up ~1.9 mm
     +x of where X_STOP assumes, so the press servo's 4.5 mm travel budget
     ran out before the pad ever loaded the card: pad press decayed
     2.00 -> 0.67 N across the slide while the card SANK 1.0 mm.  Budget
     widened to 14 mm (safe: the servo is bounded by FORCE, pitfall 6.2).
  2. THE WRIST STABILIZER WAS UNGATED here too.  Gated (crutch rule).
  3. WHAT THE CADENCE FIX ALONE BOUGHT (one run, everything else stock):
         other cards dragged   143 / 129 / 119 mm  ->  18 / 28 / 29 mm
         grip at the pinch     24.8 / 15.1 N       ->  4.9 / 1.6 N
                               (i.e. the pinch grabbed FOUR cards; now ~one)
         card rise pre-pinch   -2.84 mm (it sank)  ->  +4.36 mm
         dense audit worst     1.83 mm             ->  0.98 mm
     The remaining gap is that the slide still only lifts the card ~4 mm of
     the 13 mm PROUD_GOAL, so the far pad sweeps in too low.
  5. WIDENING THE PRESS BUDGET FIXED THE PRESS, NOT THE SLIDE, AND COST
     PENETRATION -- tried at 14 mm and REVERTED to 4.5 mm (see
     PRESS_ADJ_HI for the side-by-side table).  With 14 mm the pad reaches
     AND HOLDS its target:
         at load    2.00 -> 2.51 N   (target 2.6, servo adj only 0.67 mm)
         at slide   0.67 -> 2.10 N   (i.e. the force is held for 4.2 s)
     and yet the card still ends -1.4 mm proud: over the slide the pad rose
     14.5 mm while the card TOP fell from 60.3 to 56.1 mm.  That is the
     informative result.  Post-friction-fix the pad-card contact holds
     statically, so a pad that slides 14.5 mm relative to the card means the
     Coulomb limit was exceeded, and at 2.10 N of press that limit is
     0.62*2.10 = 1.30 N against a budget of 0.147 N of card weight plus
     0.25*2.10 = 0.53 N of card-card drag -- a NET +0.62 N that should have
     carried it.  So something else is holding the picked card down by more
     than 1.3 N, and the press force is no longer the free variable.  NEXT
     STEP IS THE SLOT, NOT THE SERVO: instrument what the picked card
     touches during the slide (slot floor, the two z=37 slot-end brackets it
     was leaning on at rest, card_c) with a PairProbe per counterpart, and
     look for the bracket -- flattening the leaning card onto the pack is
     what puts its top edge under those bracket corners.
     *** THAT NEXT STEP WAS TAKEN AND THE BRACKET HYPOTHESIS IS REFUTED.
     The PairProbes (pk_holder / pk_deck, logged every frame as
     hold_holder_fz_N / hold_deck_fz_N) show the deck pushing the picked
     card UP by +0.2..0.6 N during the slide and the holder swinging only
     -1.07..+0.82 N -- neither of which is a >1.3 N hold-down.  They
     understate it because a whole-pair vector sum cancels the two halves
     of a WEDGE, and a wedge is what it was: the card had been driven
     bodily through the slot wall (see the root-cause block above).  The
     probes were still worth building -- they are what ruled the deck out
     as the source, which is the user's "the cards are stuck together"
     question. ***  Also note the
     SLIP_CAP interlock fired again (s_up 14.5 mm vs 14.0 mm cap with zero
     rise), so the slide froze itself before its travel ran out; that cap
     needs to be a force/again test, not a displacement test.
     Two other numbers from this run for the record: the pinch closed at
     17.18/11.76 N (vs 4.90/1.58 N when the card had risen +4.4 mm), which
     is the signature of the pads closing on ~3 cards instead of one --
     i.e. the pinch force is a READOUT of how proud the card got, and worth
     gating on.  That is also why the 14 mm budget lost on penetration: the
     dense audit went 0.98 -> 2.63 mm (worst pair card_picked<->holder at
     t=8.43 s, i.e. during the lift), because a pinch that has taken three
     cards drags the deck through the holder on the way out.

  4. A NOTE ON THE OLD LOG FIELD "lean_deg": it was reported through
     np.clip(theta, TILT_MIN, TILT_MAX) with TILT_MIN == TILT_MAX == 0.0,
     so it read 0.00 unconditionally and told you nothing.  It now logs the
     card's REAL lean (`card_lean_deg`), which is what showed the press
     rotating the card 19 deg PAST vertical, alongside the press servo's
     own state (press_adj, s_up, the signed pad-face-to-card-face gap) and
     the tunnelling governor's (card_step_mm, collide_substep).

DELTA BASELINE BEFORE THIS ROUND (job 21128796, the numbers everything
below is measured against; note Delta does not reproduce rocky's numbers —
re-baseline, do not trust the older figures in this header): QA FAIL,
NO PINCH at 0.44/0.44 N, picked rise 0.14 cm / lateral 0.48 cm, card rise
before the pinch 0.63 mm, dense audit worst card_picked<->holder 2.506 mm
(the tunnelled wall), others_disp 2.14 / 1.64 / 1.23 mm.

WHERE THIS STANDS (2026-08-13, end of the tunnelling round)

  FIXED, and measured:
    * penetration / tunnelling.  Dense audit worst 2.506 mm FAIL (and the
      picked card impaled on the slot wall for 8 s) -> 0.610 mm, ALL PAIRS
      PASS, and card_picked<->holder is no longer even the worst pair.
      Static per-card audit worst 1.665 -> 0.042 mm.
    * the deck at rest.  Free-settle drift +5.4..6.5 mm and 1.5 s of
      toppling at 87 mm/s -> +0.15..0.36 mm and at rest by t = 0.27 s.
    * the deck's COUPLING, which was the user-reported bug.  Kinematic
      single-card pull-out: neighbours +13.7/+13.2/+25.4 mm -> -0.64 mm,
      card-to-card vertical coupling 0.452 N -> 0.0044..0.027 N against a
      0.147 N card weight.

  TELEOP GATE: card_pick had NO gate result in its history.  It now has
  one, and it PASSES -- FINAL TALLY 20 sessions, 20 PASS, 0 FAIL / INCON /
  ERROR, jobs 21130246 / 21130247:
      card_pick  PASS  mode=tcp  pen=0.329mm  vel=0.24m/s  feat=5/5  {}
      card_pick  PASS  mode=tcp  pen=0.344mm  vel=0.25m/s  feat=5/5  {}
  i.e. ZERO integrity violations of any kind under the adversarial session,
  0.33 mm of penetration GROWTH over the rest baseline, no flying objects,
  and 5/5 bodies covered by the feature declaration (pitfall 9.6).  Note the
  gate is what makes the build-time `rig.collide_every_substep = True`
  load-bearing (pitfall 5.3): the gate replaces the control callback, so the
  speed governor below does not exist under it and the SAFE cadence has to
  be the constructor default.

  STILL BROKEN: the scripted expert's FRICTION SLIDE, and therefore
  task_success.  The press itself is fixed (0.01 N -> 3.50 N held through
  the slide at a 4.5 N target), but the card does not climb: over a 24 mm
  pad stroke the card rose 0.04 mm, and a STATIC pad pressing 2 N could not
  even stop the card sliding 0.3 mm DOWN over 2 s.  The arithmetic says it
  should work -- pad 0.625*N up against card_c's 0.25*N plus 0.147 N of
  weight, a 2.1x Coulomb margin at 4.5 N -- and the kinematic probe says the
  card is free to move (a driven card leaves the deck against 0.027 N of
  resistance).  So the pad is delivering a small fraction of its Coulomb
  capacity.  `pk_pad` (PairProbe on the pressing pad, logging
  pad_drive_u_N = the friction component ALONG the slide) was added to
  measure exactly that; a press-force sweep (TB_CARD_PRESS_N) is the next
  step, since if the drive scales with N the slide is a force-scale problem
  and if it does not the contact model cannot do this stroke at all --
  which would make this README pitfall 7.7 (budget the slide before
  designing the stroke) with a hard "no".
  Note the slide budget is also tight independent of friction: the pad tip
  starts 20 mm below the card's top edge, so it runs OFF the card after
  20 mm of stroke, against a 13.9 mm rise requirement -- i.e. the whole
  stroke needs a slip ratio better than 1.44:1.

  THE SWEEP RAN (job 21130316) AND THE ANSWER IS NEITHER OF THE TWO
  HYPOTHESES ABOVE.  TB_CARD_PRESS_N in {4.5, 12, 25} N, one run each:
      4.5 N -> card top 27.13 -> 41.46 mm, +13.07 mm proud (CLEARS the goal)
     12.0 N -> 27.03 -> 26.96 mm, -1.43 mm proud (dead), pad delivering 14 N
     25.0 N -> 26.89 -> 26.91 mm, -1.47 mm proud (dead), pad delivering 28 N
  So it is not force-scale-limited and not structurally impossible; more
  press makes it WORSE.  Physically coherent: the pad's driving friction
  scales with press, but so does the reaction at every OTHER contact the
  press pushes the card into (deck face, slot floor, bracket corners), and
  the resisting side wins.  That also retroactively explains why the earlier
  round's "widen the press budget to 14 mm" attempt made everything worse.

  BUT DO NOT TRUST THAT SWEEP AS A TREND -- THE EXPERT IS NON-DETERMINISTIC.
  Five runs at the DEFAULT 4.5 N press give a slide outcome ranging over
  90 mm:
      +13.07 / -1.43 / -1.47 / -0.55 / -77.55 mm proud
  The -77.55 mm run ends with the card 49 mm BELOW datum at 0.00 N pad force
  -- it escaped the holder entirely.  Three single runs at three force
  levels therefore cannot separate a force trend from this variance, and the
  +13.07 mm was a lucky draw, not an operating point.  THE NEXT STEP IS TO
  FIND THE SOURCE OF THE VARIANCE, not to tune the press: repeat the default
  case >= 10x and bisect what differs between a +13 mm run and a -77 mm one.
  (The gate is unaffected by any of this -- it replaces the control callback
  and never runs the expert, which is why it can sit at 20/20 while the
  scripted pick is unreliable.)

Penetration hygiene (README pitfall 3.2): all meshes subdivided to <=2.5 mm
edges, SDF margin 0.5 mm, contact gap 0.8 mm/shape (sized to the 1.5 mm
slot wall, see GAP), 24 substeps / 30 XPBD iterations.  QA: dense
full-rollout audit of every pair (4 cards x holder + all card-card pairs),
both directions, ray-verified trimesh ground truth (tol 0.8 mm).

DIAGNOSTIC: `card_probe_deck.py` builds this scene WITHOUT the robot and
answers "is the deck coupled?" directly — a free settle with a full
per-counterpart force matrix, and a mode that makes one card KINEMATIC and
drives it out of the deck to see whether the others follow.

Run: python -m force_bench.tasks.card_pick
"""

from __future__ import annotations

import gc
import os

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.robot.franka import down_grasp_quat
# TB_GELSIGHT=1 -> GelSight fingertips (sets the gel solver defaults at
# import, so this must precede task_rig).  Case tip extension stays 0: the
# press sweeps the pad in over the pack tops, and a case standing proud of
# the fingertip would shovel the deck.
import force_bench.tactile.task as GS
from force_bench.rig import make_solver, set_finger_effort_limit, JD, Rig
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass

# measured geometry (m)
CARD_T = 0.0012          # true single-card thickness in the STL
CARD_COLLISION_T = CARD_T  # analytic box matches the visible 1.2 mm slab
SLOT_X = (-0.037, -0.023)
SLOT_FLOOR_Z = 0.002
BRACKET_TOP_Z = 0.037    # slot-end bracket tops (the lean rests on these)
CARD_H = 0.055
# (the old PACK_XS / PACK_FACE_X hand-placed the three pack cards standing
#  perfectly vertical on 0.8 mm gaps.  Both are gone: the deck's x layout is
#  now derived from its own lean by deck_pose, because a 1.2 mm slab
#  standing upright on its bottom edge is a knife-edge balance and measurably
#  does not stay there.)
CARD_Y_SHIFT = -0.00075  # center the 85 mm card in the 86.5 mm slot: the
                         # CAD deck butts the +y slot end wall EXACTLY
                         # (0.00 mm clearance, measured)
CARD_MASS = 0.015
CARD_MU = 0.25           # glossy PVC card (pair with pad mu 1.0 -> 0.62)
HOLDER_MU = 0.3
# WHICH WALL THE DECK LEANS ON.  -1 = the -x (near) bracket, which is the
# only side that BACKS a face press; see the mirror block in deck_pose for
# the measurement.  +1 restores the original far-side lean (kept so the A/B
# can be re-run, not because it works).
DECK_LEAN_SIGN = -1

# FR3 fingertip (measured collision shapes)
TIP_BELOW_TCP = 0.0097
PAD_TALL = 0.0185
PAD_FACE_OFF = 0.000085  # pad inner face sits at (dof - this) from center

# expert plan
Q_PRESS = 0.038          # both fingers share FR3's mechanical mimic constraint
Q_OPEN = 0.038           # far-side finger dof (wide open, free air)
Q_PINCH = 0.0            # both dofs at the pinch: the 1.2 mm card holds each
                         # pad ~0.6 mm open -> 0.6 mm drive interference
                         # -> ~3 N/pad at FINGER_KE 5000 (gentle; pitfall 6.1)
# GelSight faces sit finger_offset() (5 mm) proud of the bare tips.
# Offset the grasp gap, keep the open jaws inside their 40 mm stops, and
# use those actual commands plus the proud offset in tcp_at below.
Q_PRESS = min(0.04, Q_PRESS + GS.finger_offset())
Q_OPEN = min(0.04, Q_OPEN + GS.finger_offset())
Q_PINCH += GS.finger_offset()
# NOTE on press control: the arm chain is ~20 N/mm and re-pressing the
# MEASURED card face at 60 Hz pumps the retreating card out of the holder
# (observed: card ejected onto the holder base twice).  The press is
# therefore a quasi-static sweep to a nominal stop (X_STOP) followed by a
# pure FORCE servo on the pressing pad's own contact probe — never a
# face-tracking servo.
# TB_CARD_PRESS_N overrides the press target for a force sweep without
# editing the file (the slide's whole question is whether the pad's 0.625 mu
# beats card_c's 0.25 plus the card's weight, and that is a force sweep).
PRESS_N = float(os.environ.get("TB_CARD_PRESS_N", 4.5))
_PRESS_N_DOC = """target pad press force (N) during press + slide:
    friction budget 0.625*N up vs 0.25*N card-card drag
    + 0.147 N card weight -> 0.53 N of net lift at 1.8.
    RAISED after the force log showed the slide phase
    never held the target: pad force averaged 0.79 N
    (peak 1.87) and the card SANK 0.83 mm — the budget
    at 0.79 N is +0.15 N, inside the slip margin.  The
    servo stalled because its advance hit CRUSH_GUARD
    while the deck was still compacting (the other
    cards moved up to 5.9 mm), so the travel caps go
    up with the target.

"""
K_PRESS = 0.00003        # press admittance (m per N per frame)
PRESS_RATE = 0.00025     # m/frame (15 mm/s) slew of the commanded pad-face
                         # plane toward the card's MEASURED face.  This is
                         # what makes face-tracking safe: the command
                         # follows, never leads (see press_face).
PRESS_ADJ_LO, PRESS_ADJ_HI = -0.0005, 0.008   # INTERFERENCE about the
                         # MEASURED face, not travel about a fixed plane.
                         # WIDENING THIS TO 0.014 WAS TRIED AND REVERTED --
                         # a measured dead end worth keeping.  Rationale for
                         # trying it: the press geometry was keyed to a deck
                         # that TOPPLED during the settle ("~10 deg, 7 mm of
                         # drift"), and with the cadence fixed (pitfall 5.2)
                         # the cards stand perfectly still, so the deck ends
                         # up ~1.9 mm +x of where X_STOP assumes and the
                         # 4.5 mm budget ran out before the pad loaded the
                         # card (press decayed 2.00 -> 0.67 N across the
                         # slide).  At 14 mm the servo DID hold its target
                         # (2.51 N at load, 2.10 N held for the whole 4.2 s
                         # slide) -- and every QA axis got WORSE, because the
                         # card still did not climb and the extra travel just
                         # buried the pad in the deck:
                         #                      4.5 mm      14 mm
                         #   dense audit worst   0.98 mm     2.63 mm
                         #   grip at the pinch   4.9/1.6 N   17.2/11.8 N
                         #                       (~1 card)   (~3 cards)
                         #   card rise pre-pinch +4.36 mm    -1.4 mm
                         # So press FORCE is not the free variable here (see
                         # the arithmetic in the header) and buying more of
                         # it costs penetration.  Back to 4.5 mm.
                         #
                         # *** RE-OPENED AT 12 mm, AND THE OLD VERDICT IS
                         # NOW EXPLAINED.  The 14 mm attempt cost penetration
                         # because the extra travel was delivered by a
                         # VERTICAL pad edge-loading a LEANING card while
                         # the collision cadence could not catch 1.33 mm of
                         # travel per frame -- i.e. it bought tunnelling, not
                         # force.  With the grasp tilted to the deck, the gap
                         # sized to the slot wall and the cadence governed by
                         # measured speed, none of that mechanism is left.
                         # And the travel is NEEDED, measured: at 1.2 mm the
                         # press reached 0.04 N at the load and 0.15 N across
                         # the whole slide, because A LEANING DECK HAS NO
                         # BACKSTOP -- a face press at the pad's height
                         # (~43 mm above the floor pivot) simply STANDS THE
                         # DECK UP, and standing 15 deg up is ~7 mm of travel
                         # at that height before the rear card's bottom
                         # corner even reaches the -x slot wall.  The force
                         # servo cannot bound a force that does not exist.
                         # What bounds the advance instead is a GEOMETRIC
                         # ENVELOPE, not a travel budget: see `a_limit`. ***
CRUSH_GUARD = 0.0003     # the servo freezes once the DECK'S OWN THICKNESS
                         # -- the picked card's centre to card_a's centre
                         # measured along the card normal -- falls this far
                         # below its PACKED value of 3*CARD_T = 3.6 mm.
                         # A force servo with NO force
                         # feedback (a loose deck reads 0 N) otherwise runs
                         # to its stop: 6 mm of blind advance crushed the
                         # whole deck in one run.  This replaces the old
                         # guard, which watched the picked card's absolute
                         # displacement and therefore could not tell a deck
                         # being CRUSHED from a deck being legitimately
                         # STOOD UP -- and standing up is ~9 mm of card
                         # centre travel, four times the old 2.6 mm guard,
                         # so the old guard froze the press before it could
                         # ever reach a backstop.  Deck thickness is
                         # invariant to both standing up and sliding.
OVERLAP_SLIDE = 0.020    # pad tip this far below the pack tops at the press:
                         # low enough that the pack's lever arm about its
                         # z=17 wall prop stays short, high enough that ~22 mm
                         # of slide puts the pad tip ABOVE the pack tops
V_SLIDE = 0.008          # up-plane pad speed (m/s): slower keeps the
                         # friction budget quasi-static (the slip that sank
                         # the card happened at 12 mm/s)
S_UP_MAX = 0.028         # slide travel cap
SLIP_CAP = 0.018         # pad may outrun the card by at most this.  At
                         # 6 mm this DEADLOCKED the task: the state log shows
                         # the pressing pad rising 4.5 mm and then standing
                         # still for the rest of the slide, because the card
                         # had not climbed and the cap froze the command --
                         # so the friction never got a second chance.
PROUD_GOAL = 0.013       # slide until the card top stands this proud of the
                         # pack tops (the pinch band)
TIP_MARGIN = 0.003       # both fingertips must end this far above the pack
                         # tops before the far finger may sweep in
# THE GRASP IS TILTED TO THE DECK'S OWN LEAN (was: VERTICAL).  The deck's
# rest state is a leaning BLOCK at 12.3 deg (deck_pose, measured), so a
# vertical pad pressed on the front card's face touches along ONE pad edge:
# over the 18.5 mm pad height the card face moves 18.5*tan(12.3) = 4.0 mm in
# x, which turns the press into a torque about a pad corner.  That torque is
# what swung the card's bottom edge +x through the slot wall.  With
# grasp_quat(lean_th) the pad normal is the card's face normal exactly, the
# press is a face press, and the slide is colinear with the plane the card
# has to travel along.

LIFT_DZ = 0.115          # lift leg 1: ALONG THE CARD PLANE (u).  A single
LIFT_DX = 0.080          # diagonal (old 0.35:0.937) drove the card into the
LIFT_XS = 0.40           # +x bracket wall -- 18 N of wedging.
                         # LIFT_XS 0.28 -> 0.40: the +x leg must not start
                         # until the card's BOTTOM edge is clear of the
                         # z=37 slot-end brackets.  smooth(0.40) = 0.352 of
                         # 115 mm = 40.5 mm along u, which puts the bottom
                         # edge at z = 41.6 mm, 4.6 mm clear.  At 0.28 it
                         # was still at z = 25 mm, i.e. inside the slot.
                         # LIFT_DX 55 -> 80 mm: leg 1 now travels -x with
                         # the deck's lean (u_x = -0.225), giving back
                         # 25.9 mm, so 55 mm left the QA lateral gate at
                         # 29.3 mm against a 30 mm threshold.
GATE_FULL, GATE_STOP, GATE_MIN = 2.5, 7.0, 0.35
K_XY = 0.25
XY_CORR_MAX = 0.012

# T_SETTLE is MEASURED with the robot removed entirely (card_probe_deck.py
# CP_MODE=settle), not guessed:
#   * from the OLD CAD pose (3 cards vertical + 1 leaning) the deck TOPPLES,
#     peaks at 87 mm/s = 1.45 mm/FRAME and only comes to rest at t = 1.5 s.
#     The old T_SETTLE of 0.7 s therefore handed the last 0.8 s of that
#     topple to the once-per-frame cadence at 3.6x the contact gap.
#   * from the deck_pose equilibrium the deck settles its 0.2 mm of floor
#     clearance and is at rest by t = 0.27 s (peak 37 mm/s, all of it in the
#     first frame; x drift +0.14..0.35 mm instead of +5.4..6.5 mm).
# 0.6 s is that 0.27 s with a 2x margin.  Collision detection continues at
# every substep after this initial settling window.
T_SETTLE, T_DESCEND, T_PUSH, T_LOAD = 0.6, 0.9, 0.9, 0.6
T_SLIDE, T_CLOSE = 4.2, 0.7   # slide window keeps ~34 mm of pad
                              # travel available at the slower V_SLIDE
T_LIFT, LIFT_SLACK, T_HOLD = 2.4, 0.6, 0.4
# THE ABSOLUTE PRESS STOP IS GONE.  It used to be
#     X_FLAT = SLOT_X[0] + 4*CARD_T = -32.2 mm
# i.e. "where the picked card's face sits once the deck is compacted flat
# against the -x slot wall".  That deck does not exist: the free deck stands
# LEANING on the +x brackets with its rear card ~1 mm off the -x wall (see
# deck_pose), so -32.2 mm is ~8 mm INSIDE the real card face and the sweep
# to it was the ram that tunnelled the card through the slot wall.  Every
# press command is now a coordinate in the deck's own (a, b) frame,
# referenced to the MEASURED card face, and CRUSH_GUARD is measured as the
# card centre's displacement along the face normal from its own rest pose.

FINGER_KE = 8000.0       # ~4.8 N/pad at a 0.6 mm finger opening, below 12 N limit
FINGER_KD = 60.0         # damp the pinch as the card clears its neighbours
# THE WRIST->CARD d6 IS A CRUTCH (crutch rule): 1200 N/m needs 0.13 mm of
# stretch to hold the 15 g card, so it can carry the whole extraction on its
# own.  OFF unless TB_STABILIZER=1 -- both ke AND kd, since a damping-only
# XPBD drive is still a velocity weld (XPBD-era pitfall 21b).  The joint itself stays
# for its articulation membership (pitfall 4.6).
STAB_ON = os.environ.get("TB_STABILIZER") == "1"
_STAB_BUILD = STAB_ON
STAB_KE = (1200.0, 450.0) if STAB_ON else (0.0, 0.0)
STAB_KD = (12.0, 2.5) if STAB_ON else (0.0, 0.0)

MAX_EDGE = 0.0025
SDF_MARGIN = 0.0005
# CONTACT GAP IS SIZED TO THE THINNEST WALL IN THE SCENE, NOT TO THE PART.
# `ShapeConfig.gap` is a broad-phase DETECTION distance only -- verified in
# newton's source: the XPBD contact constraint is
# `d = dot(n, p1-p0) - (margin0+margin1)` and returns early on `d >= 0`, and
# `gap` appears only in the candidate threshold `rigid_gap + radii + margins`
# (`collision_core.compute_gjk_mpr_contacts`).  So a bigger gap generates
# contacts EARLIER and never generates force at separation.  The measured
# failure this fixes: the slot's side walls are 1.5 mm thick (inner faces
# x = -37.0 / -23.0, outer -38.5 / -21.5) and the picked card's bottom edge
# was flung +x at 80 mm/s = 1.33 mm PER FRAME, so at the old 0.4 mm gap
# (gap_sum 0.8 mm) and once-per-frame collision it crossed the whole wall
# inside a single collide() interval and NO contact was ever generated -- it
# ended up impaled, 11 % of its vertices inside the wall, reaching 2.4 mm
# past the inner face.  With 0.8 mm per shape the gap_sum is 1.6 mm > the
# 1.5 mm wall, so a card can never be simultaneously clear of BOTH faces:
# some contact always exists to be resolved.
GAP = 0.0008


def subdiv(tm: trimesh.Trimesh, max_edge: float = MAX_EDGE) -> trimesh.Trimesh:
    v, f = trimesh.remesh.subdivide_to_size(tm.vertices, tm.faces,
                                            max_edge=max_edge, max_iter=8)
    return trimesh.Trimesh(v, f, process=False)


def single_card_tm() -> trimesh.Trimesh:
    """Extract slab #1 (x in [-35.0, -33.8] mm) from the 10-card STL.

    Faces on the shared plane between slab 1 and slab 2 belong to both
    cards (coincident, opposite normals) — keep only the +x-normal copy.
    """
    full = load_tm("card_holder", "card_1_body1")
    tc = full.triangles_center[:, 0]
    nrm = full.face_normals
    lo, hi = -0.0350, -0.0338
    eps = 1e-5
    inside = (tc > lo - eps) & (tc < hi + eps)
    at_hi = np.abs(tc - hi) < eps
    keep = inside & ~(at_hi & (nrm[:, 0] < 0.5))
    sub = full.submesh([np.where(keep)[0]], append=True)
    sub.merge_vertices()
    sub.update_faces(sub.nondegenerate_faces())
    assert sub.is_watertight, "single-card slab extraction not watertight"
    ext = sub.extents
    assert abs(ext[0] - CARD_T) < 1e-4, f"card thickness {ext[0]}"
    sub = subdiv(sub)                       # pitfall 3.2: <=2.5 mm edges
    assert sub.is_watertight
    return sub


class RobustDepth:
    """Closest-point depth with normal-sign candidates, each verified by a
    ray-parity containment test on the RAW mesh (the pure face-normal sign
    test false-positives on 1.2 mm slabs near sharp edges)."""

    def __init__(self, mesh_fine: trimesh.Trimesh, mesh_raw: trimesh.Trimesh):
        self.mesh = mesh_fine
        self.pq = trimesh.proximity.ProximityQuery(mesh_fine)
        self.raw = mesh_raw

    def depth(self, pts: np.ndarray, pad: float = 0.004) -> float:
        lo, hi = self.mesh.bounds
        m = ((pts > lo - pad) & (pts < hi + pad)).all(axis=1)
        if not m.any():
            return -pad
        pts = pts[m]
        closest, dist, tri = self.pq.on_surface(pts)
        cand = (np.einsum("ij,ij->i", pts - closest,
                          self.mesh.face_normals[tri]) < 0) & (dist > 1e-6)
        if cand.any():
            ci = np.where(cand)[0]
            if len(ci) > 64:
                ci = ci[np.argsort(dist[ci])[-64:]]
            ins = self.raw.contains(pts[ci])
            if ins.any():
                return float(dist[ci][ins].max())
        out = dist[~cand]
        return float(-out.min()) if len(out) else -pad


class PairProbe:
    """ForceProbe-compatible probe restricted to specific COUNTERPART shapes
    (README pitfall 6.6).  Used here as the diagnostic the slide needed: a
    whole-body probe on the picked card sums the pressing pad, the deck and
    the holder into one number, so "something holds the card down" could not
    be attributed.  These resolve it per counterpart, and log the SIGNED z
    component, because the question is specifically a vertical one.
    """

    def __init__(self, model, sensing_body, counterpart_shapes=None,
                 counterpart_bodies=None):
        kw = ({"counterpart_bodies": counterpart_bodies}
              if counterpart_bodies is not None
              else {"counterpart_shapes": counterpart_shapes})
        self.sensor = newton.sensors.SensorContact(
            model, sensing_bodies=[sensing_body], measure_total=False, **kw)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        self.sensor.update(state, contacts)
        F = self.sensor.force_matrix.numpy()[0].sum(axis=0)
        Ff = self.sensor.force_matrix_friction.numpy()[0].sum(axis=0)
        f, fr = float(np.linalg.norm(F)), float(np.linalg.norm(Ff))
        rec = {"t": t, "force": f, "friction": fr, "normal": f - fr,
               "fz": float(F[2]), "vec": F.tolist()}
        self.history.append(rec)
        return rec


def sample_pts(tm: trimesh.Trimesh, cap: int = 4500, seed: int = 0):
    v = tm.vertices
    if len(v) > cap:
        v = v[np.random.RandomState(seed).choice(len(v), cap, replace=False)]
    return np.asarray(v)


from force_bench.geometry import world_T


from force_bench.geometry import quat_mul


from force_bench.geometry import quat_inv


def stab_zero_stress_targets(q_l7, q_obj, REL):
    X_wp = world_T(q_l7) @ REL
    X_wc = world_T(q_obj)
    M = np.linalg.inv(X_wp) @ X_wc
    rel_p = M[:3, 3]
    qp = trimesh.transformations.quaternion_from_matrix(X_wp)
    qc = trimesh.transformations.quaternion_from_matrix(X_wc)
    qp = np.array([qp[1], qp[2], qp[3], qp[0]])
    qc = np.array([qc[1], qc[2], qc[3], qc[0]])
    if np.dot(qp, qc) < 0:
        qc = -qc
    rel_q = quat_mul(quat_inv(qp), qc)
    qtwist = np.array([rel_q[0], 0.0, 0.0, rel_q[3]])
    qtwist /= np.linalg.norm(qtwist)
    qswing = quat_mul(rel_q, quat_inv(qtwist))
    err0 = 2.0 * np.arcsin(np.clip(qtwist[0], -1.0, 1.0))
    err1, err2 = qswing[1], qswing[2]
    if qswing[3] ** 2 + 1e-4 < 1.0:
        d = np.sqrt(1.0 - qswing[3] ** 2)
        theta = 2.0 * np.arccos(np.clip(qswing[3], -1.0, 1.0))
        err1 *= theta / d
        err2 *= theta / d
    return np.array([*rel_p, err0, err1, err2])


def deck_pose(card_local: trimesh.Trimesh, holder_raw: trimesh.Trimesh,
              card_y: float, n_cards: int = 4):
    """Rest pose for the WHOLE deck: n parallel cards leaning +x at ONE
    angle, faces exactly CARD_T apart, bottom edges on the slot floor, the
    front card's +x face against the z=37 slot-end bracket corners.

    WHY THIS REPLACED THE OLD `lean_pose` (which stood three cards
    perfectly VERTICAL and leaned only the fourth).  MEASURED with the robot
    deleted from the scene entirely (`card_probe_deck.py CP_MODE=settle`):
    a 1.2 mm slab standing upright on its own bottom edge in a 14 mm slot is
    a knife-edge balance, and all three of them TOPPLE.  The deck reaches
    87 mm/s -- 1.45 mm per FRAME, 3.6x the contact gap -- drifts +5.4..6.5 mm
    in x, and only comes to rest at t = 1.5 s, in exactly the configuration
    built here: leans 13.2 / 13.2 / 12.3 / 12.3 deg, faces in contact, and
    every card carrying its own 0.147 N of weight straight into the holder
    (card-to-card vertical force 0.008 N, i.e. NOTHING -- the cards are not
    stuck to each other at rest, they are stacked).  So the CAD pose was
    never the rest pose; it bought a 1.5 s toppling transient at tunnelling
    speed, 6 mm of deck drift charged to the "other cards moved" gate, and
    a press geometry (X_STOP) keyed to a deck that does not exist.

    The construction is closed-form, and it reproduces that measured
    equilibrium to 0.3 mm at 12.3 deg, which is what justifies it:
      * two parallel planes with normal (cos th, 0, -sin th) offset purely
        in x by d are exactly d*cos(th) apart, so the deck's x pitch is
        (CARD_T + FACE_CLEAR)/cos(th) and the faces are parallel by
        construction (no wedge, no point contact);
      * the floor sets z: the lowest rotated vertex sits FLOOR_CLEAR above
        the slot floor;
      * the bracket sets x: the front card's +x face passes BRACKET_CLEAR
        clear of the corner (SLOT_X[1], BRACKET_TOP_Z);
      * the lean is the largest for which the REAR card's -x bottom corner
        still clears the -x slot wall.  Measured feasibility: 12.3 deg
        leaves 1.01 mm, 13.0 deg leaves 0.56 mm, 14.0 deg penetrates.
    Verified against the real holder mesh, per card, and asserted.
    """
    FLOOR_CLEAR, BRACKET_CLEAR, FACE_CLEAR, WALL_CLEAR = \
        0.00002, 0.00002, 0.00001, 0.0004
    v0 = np.asarray(card_local.vertices)
    pq = trimesh.proximity.ProximityQuery(holder_raw)

    def clearance(v):
        closest, dist, tri = pq.on_surface(v)
        cand = (np.einsum("ij,ij->i", v - closest,
                          holder_raw.face_normals[tri]) < 0) & (dist > 1e-6)
        if cand.any() and holder_raw.contains(v[cand]).any():
            return -1.0                 # some vertex is INSIDE the holder
        return float(dist.min())

    best = None
    for th in np.deg2rad(np.linspace(6.0, 14.0, 33)):
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])   # Ry(+th): top -> +x
        vr = v0 @ R.T + np.array([0.0, card_y, 0.0])
        oz = (SLOT_FLOOR_Z + FLOOR_CLEAR) - vr[:, 2].min()
        # front card: +x face through the bracket corner, minus clearance
        s_brk = (BRACKET_TOP_Z - oz + (CARD_COLLISION_T / 2) * s) / c
        ox = (SLOT_X[1] - BRACKET_CLEAR
              - (CARD_COLLISION_T / 2) * c - s_brk * s)
        pitch = (CARD_COLLISION_T + FACE_CLEAR) / c
        rear_corner = (ox - (n_cards - 1) * pitch
                       - (CARD_COLLISION_T / 2) * c - (CARD_H / 2) * s)
        if rear_corner < SLOT_X[0] + WALL_CLEAR:
            continue                    # the deck no longer fits the slot
        best = (th, R, oz, ox, pitch, vr)
    assert best is not None, "no feasible deck lean"
    th, R, oz, ox, pitch, vr = best
    xs = [ox - n * pitch for n in range(n_cards)]

    if DECK_LEAN_SIGN < 0:
        # MIRROR THE WHOLE DECK ONTO THE -x SIDE.  This is the difference
        # between a press that works and one that does not, and it is
        # measured, not aesthetic: leaning on the FAR (+x) bracket the deck
        # has NO BACKSTOP behind the pressed card -- a face press ~43 mm
        # above the floor pivot only has to beat gravity's restoring torque,
        # W*(H/2)*sin(lean) = 0.0042 N.m over a 43 mm arm = 0.10 N, so the
        # deck simply STANDS UP and the pad reached 0.04 N at the load and
        # 0.15 N across the whole 4.2 s slide (measured, job 21129549).  The
        # slide needs 0.625*N > W + 0.25*N, i.e. N > 0.39 N, so the press
        # can never be made to work that way without shoving the deck bodily
        # into the far wall -- which is exactly the ram that used to tunnel
        # the card through the slot wall.
        # Leaning on the NEAR (-x) bracket instead, the deck is wedged
        # between the floor and the z=37 bracket corners from frame one and
        # the pressed card is backed by three cards and a wall.  The holder
        # is mirror-symmetric about the slot centre (walls -38.5..-37 and
        # -23..-21.5, base -59..-1, both symmetric about x = -30.0), so the
        # mirror of a valid placement is a valid placement -- and it is
        # re-verified against the real mesh below, per card, either way.
        xc = 0.5 * (SLOT_X[0] + SLOT_X[1])
        xs = [2.0 * xc - x for x in xs][::-1]   # picked card is +x-most
        th = -th
        c, s = np.cos(th), np.sin(th)
        R = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
        vr = v0 @ R.T + np.array([0.0, card_y, 0.0])
        oz = (SLOT_FLOOR_Z + FLOOR_CLEAR) - vr[:, 2].min()

    cls = [clearance(vr + np.array([x, 0.0, oz])) for x in xs]
    print(f"deck lean {np.degrees(th):.2f} deg, pitch {pitch*1000:.4f} mm, "
          f"origins x {[round(x*1000, 3) for x in xs]} mm, z {oz*1000:.3f} mm")
    print("  per-card clearance to the holder (mm): "
          + " ".join(f"{v*1000:.3f}" for v in cls))
    assert min(cls) >= 0.5 * FLOOR_CLEAR, f"deck placement collides: {cls}"
    return th, R, oz, xs


def main():
    rig = GS.make_rig("card_pick")

    base_xf = wp.transform(wp.vec3(-0.04, 0.51, 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    for dof in (7, 8):
        rig.builder.joint_target_ke[dof] = FINGER_KE
        rig.builder.joint_target_kd[dof] = FINGER_KD

    rig.table(center_xy=(-0.03, 0.005), half=0.25)

    cfg_holder = rig.cfg(mu=HOLDER_MU, gap=GAP)
    cfg_card = rig.cfg(mu=CARD_MU, gap=GAP, is_visible=False)
    cfg_card_visual = newton.ModelBuilder.ShapeConfig(
        has_shape_collision=False, density=0.0)

    # ── static holder ──────────────────────────────────────────────────
    holder_raw = load_tm("card_holder", "holder_1_body1")
    holder_tm = subdiv(holder_raw)
    holder_mesh, holder_c = make_newton_mesh(
        holder_tm, sdf=True, sdf_resolution=320, narrow_band=0.002,
        margin=SDF_MARGIN)
    holder_c = np.asarray(holder_c)
    sid = rig.builder.add_shape_mesh(
        -1, mesh=holder_mesh,
        xform=wp.transform(wp.vec3(*holder_c), wp.quat_identity()),
        cfg=cfg_holder, label="holder")
    rig.meshes["holder"] = (holder_tm, holder_c)
    rig.shapes["holder"] = [sid]
    holder_shape = sid

    # ── cards: shared visual mesh, one analytic collider per card ──────
    card_tm = single_card_tm()
    # Analytic boxes avoid resolving the thin slab through a sampled SDF.
    # Match the visible CAD thickness; an inflated shell creates contacts
    # across visible gaps between cards.
    card_mesh, card_c = make_newton_mesh(card_tm, sdf=False)
    card_c = np.asarray(card_c)
    card_half = 0.5 * np.asarray(card_tm.extents)
    card_half[0] = CARD_COLLISION_T / 2.0
    card_local = card_tm.copy()
    card_local.apply_translation(-card_c)

    def add_card(label, pos, quat=None):
        q = wp.quat_identity() if quat is None else quat
        body = rig.builder.add_body(
            xform=wp.transform(wp.vec3(*pos), q), label=label)
        rig.builder.add_shape_mesh(
            body, mesh=card_mesh, cfg=cfg_card_visual,
            label=label + "_visual")
        s = rig.builder.add_shape_box(
            body, hx=float(card_half[0]), hy=float(card_half[1]),
            hz=float(card_half[2]), cfg=cfg_card, label=label + "_shape")
        # The slot is represented by analytic supports below.  Resolving
        # the coincident CAD mesh as well gave resting cards two competing
        # contact manifolds and persistent motion with no robot contact.
        rig.builder.add_shape_collision_filter_pair(s, holder_shape)
        rig.shapes[label] = [s]
        set_body_mass(rig.builder, body, CARD_MASS)
        rig.meshes[label] = (card_tm, card_c)
        return body

    # THE DECK IS PLACED AT ITS OWN MEASURED EQUILIBRIUM (see deck_pose):
    # four parallel cards leaning on the slot-end brackets, faces in
    # contact, bottom edges on the floor.  card_picked is the FRONT card
    # (most +x); card_c, card_b, card_a stack behind it.
    Y0 = card_c[1] + CARD_Y_SHIFT

    # Non-overlapping analytic supports on the measured slot faces.  The
    # card/holder mesh pair is filtered, so a resting card does not receive
    # competing reactions from both this support and the coincident SDF.
    # The CAD holder remains visible, collides with the robot, and supplies
    # the ground truth for the penetration audit.
    proxy_labels = []

    def holder_proxy(label, center, half):
        sid = rig.builder.add_shape_box(
            -1,
            xform=wp.transform(wp.vec3(*center), wp.quat_identity()),
            hx=float(half[0]), hy=float(half[1]), hz=float(half[2]),
            cfg=rig.cfg(mu=HOLDER_MU, gap=GAP, is_visible=False),
            label=label,
        )
        rig.shapes["holder"].append(sid)
        proxy_labels.append(label)
        # The robot still uses the CAD holder; avoid duplicate supports for
        # it too.  At this point the cards have not been instantiated.
        for body, shapes in rig.builder.body_shapes.items():
            if body >= 0:
                for other in shapes:
                    rig.builder.add_shape_collision_filter_pair(sid, other)

    slot_mid_x = 0.5 * (SLOT_X[0] + SLOT_X[1])
    slot_half_x = 0.5 * (SLOT_X[1] - SLOT_X[0])
    slot_half_y = 0.5 * 0.0865
    holder_proxy(
        "holder_floor_proxy",
        (slot_mid_x, Y0, SLOT_FLOOR_Z - 0.001),
        (slot_half_x, slot_half_y, 0.001),
    )
    wall_half_x = 0.001
    wall_half_z = 0.5 * (0.017 - SLOT_FLOOR_Z)
    wall_z = SLOT_FLOOR_Z + wall_half_z
    holder_proxy(
        "holder_wall_neg_proxy",
        (SLOT_X[0] - wall_half_x, Y0, wall_z),
        (wall_half_x, slot_half_y, wall_half_z),
    )
    holder_proxy(
        "holder_wall_pos_proxy",
        (SLOT_X[1] + wall_half_x, Y0, wall_z),
        (wall_half_x, slot_half_y, wall_half_z),
    )
    post_half_y = 0.0025
    post_half_z = 0.5 * (BRACKET_TOP_Z - 0.017)
    post_z = 0.017 + post_half_z
    for x_name, x in (
        ("neg", SLOT_X[0] - wall_half_x),
        ("pos", SLOT_X[1] + wall_half_x),
    ):
        for y_name, y_sign in (("neg", -1.0), ("pos", 1.0)):
            holder_proxy(
                f"holder_post_{x_name}_{y_name}_proxy",
                (x, Y0 + y_sign * (slot_half_y - post_half_y), post_z),
                (wall_half_x, post_half_y, post_half_z),
            )
    for y_name, y_sign in (("neg", -1.0), ("pos", 1.0)):
        holder_proxy(
            f"holder_end_{y_name}_proxy",
            (slot_mid_x, Y0 + y_sign * (slot_half_y + 0.001),
             0.5 * (SLOT_FLOOR_Z + BRACKET_TOP_Z)),
            (slot_half_x + 2 * wall_half_x, 0.001,
             0.5 * (BRACKET_TOP_Z - SLOT_FLOOR_Z)),
        )

    lean_th, lean_R, deck_z, deck_xs = deck_pose(card_local, holder_raw, Y0)
    Mq = np.eye(4)
    Mq[:3, :3] = lean_R
    qtm = trimesh.transformations.quaternion_from_matrix(Mq)   # [w,x,y,z]
    lean_q = wp.quat(qtm[1], qtm[2], qtm[3], qtm[0])
    # deck_pose worked on the CENTERED card verts, so its x/z ARE the world
    # x/z of the body origin; y keeps the slab's original center
    others = ["card_c", "card_b", "card_a"]        # front-to-back behind it
    picked_body = add_card("card_picked",
                           np.array([deck_xs[0], Y0, deck_z]), lean_q)
    for lbl, x in zip(others, deck_xs[1:]):
        add_card(lbl, np.array([x, Y0, deck_z]), lean_q)
    others = ["card_a", "card_b", "card_c"]        # keep the QA label order
    # GelSight pads (no-op unless TB_GELSIGHT=1), mounted with the deck in
    # place; only the PICKED card is flagged hydroelastic -- the gel feels
    # the face it presses and slides on, the pack stays plain contact.
    GS.mount(rig)
    GS.hydroelastic(rig, "card_picked")
    # The sensors are mounted after the support proxies were built.  Like
    # the bare robot, they use the CAD holder, not a second coincident set
    # of supports.  Keep all sensor/holder CAD collisions enabled.
    if GS.GS_ON:
        # The sensor default (96 cells along the longest axis) gives this
        # 85 mm card fewer than two cells through its 1.2 mm thickness.
        # Resolve the rigid card independently of the much smaller gel:
        # at 512 cells its thickness spans about seven SDF samples.
        for shape in rig.shapes["card_picked"]:
            rig.builder.shape_sdf_max_resolution[shape] = 512
        for label in proxy_labels:
            proxy = rig.builder.shape_label.index(label)
            for body, label_body in enumerate(rig.builder.body_label):
                if str(label_body).startswith("fr3/"):
                    for shape in rig.builder.body_shapes.get(body, []):
                        rig.builder.add_shape_collision_filter_pair(proxy, shape)
    # the deck's own frame: n = card face normal (+x side), u = "up" along
    # the card plane.  Every press/slide/lift command below is written in
    # (n, u) so the pads stay PARALLEL to the card faces -- a vertical pad
    # on a 12.3 deg face touches only along one pad edge, which is a torque,
    # not a press.
    nrm = lean_R @ np.array([1.0, 0.0, 0.0])
    upl = lean_R @ np.array([0.0, 0.0, 1.0])
    O_deck = np.array([deck_xs[0], Y0, deck_z])    # front card body origin

    # ── build-time rest clearance audit ────────────────────────────────
    local, raw_local, pts = {}, {}, {}
    holder_local = holder_tm.copy()
    holder_local.apply_translation(-holder_c)
    holder_raw_local = holder_raw.copy()
    holder_raw_local.apply_translation(-holder_c)
    local["holder"], raw_local["holder"] = holder_local, holder_raw_local
    pts["holder"] = sample_pts(holder_local)
    for lbl in others + ["card_picked"]:
        local[lbl] = card_local
        raw_local[lbl] = card_local
        pts[lbl] = sample_pts(card_local)
    names = others + ["card_picked"]
    pair_list = [(lbl, "holder") for lbl in names]
    pair_list += [(a, b) for i, a in enumerate(names) for b in names[i + 1:]]

    def audit_pairs(get_T, frames):
        out = {}
        for b_lbl in ["holder"] + names:
            fd = RobustDepth(local[b_lbl], raw_local[b_lbl])
            for a, b in pair_list:
                fwd = b == b_lbl
                if not fwd and a != b_lbl:
                    continue
                x, y = (a, b) if fwd else (b, a)
                worst, wt = out.get(f"{a}<->{b}", (-9.9, None))
                for f, tt in frames:
                    Txy = np.linalg.inv(get_T(y, f)) @ get_T(x, f)
                    dd = fd.depth((Txy[:3, :3] @ pts[x].T).T + Txy[:3, 3])
                    if dd > worst:
                        worst, wt = dd, tt
                out[f"{a}<->{b}"] = (worst, wt)
            del fd
            gc.collect()
        return out

    deck_origin = {"card_picked": deck_xs[0], "card_c": deck_xs[1],
                   "card_b": deck_xs[2], "card_a": deck_xs[3]}

    def T_build(lbl, f_):
        T = np.eye(4)
        if lbl == "holder":
            T[:3, 3] = holder_c
        else:
            T[:3, :3] = lean_R
            T[:3, 3] = [deck_origin[lbl], Y0, deck_z]
        return T

    rest_res = audit_pairs(T_build, [(None, 0.0)])
    rest = {k: round(v[0] * 1000, 3) for k, v in rest_res.items()}
    for k, v in rest.items():
        print(f"rest clearance {k}: {-v:+.2f} mm")
    assert all(v < 0.3 for v in rest.values()), f"initial interpenetration: {rest}"
    gc.collect()

    # ── arm start: above the press pose, TILTED TO THE DECK ───────────
    def grasp_quat(theta=0.0):
        base = down_grasp_quat(np.pi / 2)                    # close along x
        tilt = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(theta))
        q = wp.mul(tilt, wp.quat(*base))
        return np.array([q[0], q[1], q[2], q[3]])

    top_local = np.array([0.0, 0.0, CARD_H / 2])             # top edge center

    # ── the DECK FRAME (a, b) ─────────────────────────────────────────
    # a = distance along the card's face NORMAL from the front card's body
    #     origin (a = +CARD_T/2 IS its +x face)
    # b = distance UP the card's own plane   (b = +CARD_H/2 IS its top edge)
    # The whole expert is written here instead of in world x/z because the
    # deck leans 12.3 deg: a vertical pad pressed on a 12.3 deg face only
    # touches along ONE pad edge, and over the 18.5 mm pad height the card's
    # face moves 4.0 mm in x, so a vertical press is a torque about a pad
    # corner rather than a face press.  Tilting the grasp by the deck's own
    # lean makes the pads parallel to the faces and the slide colinear with
    # the plane the card actually has to travel along.
    def ab_of(P):
        d = np.asarray(P) - O_deck
        return float(d @ nrm), float(d @ upl)

    def P_of(a, b):
        return O_deck + a * nrm + b * upl

    deck_top_z = float(deck_z + (lean_R @ card_local.vertices.T).T[:, 2].max())
    A_FACE = CARD_COLLISION_T / 2             # conservative +n contact face
    A_APP = A_FACE + 0.004                    # descend this far off the face
    B_PRESS = CARD_H / 2 - OVERLAP_SLIDE      # pad tip at the press
    # THE PACK TOPS ARE NOT ALL AT b = CARD_H/2.  The deck is offset purely
    # in world x, and world x has a component along BOTH deck axes
    # (u_x = sin(lean)), so each card behind the picked one sits
    # |dx * u_x| = 0.30 mm HIGHER in b than the last.  Measure the real
    # maximum instead of assuming, or the far pad sweeps in at pack-top
    # height and shovels the deck.
    B_PACK_TOP = max(
        float(np.max(((lean_R @ card_local.vertices.T).T
                      + np.array([x - deck_xs[0], 0.0, 0.0])) @ upl))
        for x in deck_xs[1:])
    B_MIN_TIP = B_PACK_TOP + TIP_MARGIN       # tips clear of the pack tops
    print(f"deck top z {deck_top_z*1000:.2f} mm, lean {np.degrees(lean_th):.2f} "
          f"deg, n={np.round(nrm, 4)} u={np.round(upl, 4)}, "
          f"press tip b {B_PRESS*1000:.1f} mm, pack top b "
          f"{B_PACK_TOP*1000:.2f} mm (picked {CARD_H/2*1000:.1f} mm), "
          f"far-pad sweep b {B_MIN_TIP*1000:.2f} mm")

    def tcp_at(a_face, b_tip, q_px=Q_PRESS):
        """TCP putting the +n pad FACE plane at deck coord `a_face` and the
        fingertip bottoms at deck coord `b_tip` (grasp tilted to the deck)."""
        return P_of(a_face - q_px + PAD_FACE_OFF + GS.finger_offset(),
                    b_tip + TIP_BELOW_TCP)

    # ── the press ENVELOPE (README pitfall 4.8, applied to a servo) ──────
    # A press has to be bounded by FORCE (pitfall 6.2), but a force servo has
    # nothing to bound when the thing it pushes has no backstop -- and a
    # leaning deck has none until it has stood up and slid ~1 mm to the -x
    # slot wall.  Measured: at a 1.2 mm travel budget the press reached
    # 0.04 N.  So the advance is bounded GEOMETRICALLY instead: the pad-face
    # plane may never pass the plane a fully WALL-COMPACTED deck's front
    # face would occupy, X_FLAT = SLOT_X[0] + 4*CARD_T.  That is a real
    # fixture limit (four card thicknesses off the far wall), it is derived
    # from the holder rather than tuned, and it cannot be walked past by any
    # integrator, any operator or any policy.
    # ONE card thickness off the far wall: a pad face there has passed
    # every position a real four-card deck could occupy, so nothing
    # legitimate is ever clipped, and no integrator, operator or policy can
    # walk further.  (The pad itself never touches the -x wall: the wall
    # tops out at z = 17 mm and the pad works above z = 36 mm.)
    # The envelope is a CONSTANT in the deck frame: one millimetre past the
    # REAR card's own back face.  A pad there has passed the entire deck, so
    # nothing legitimate is ever clipped, and no integrator, operator or
    # policy can walk further.  Two things it must NOT be:
    #   * a function of the pad's height.  The pad rises ALONG the card
    #     plane, which moves it |u_x| = 0.225 mm of world x per mm of climb,
    #     so a WORLD-X floor re-evaluated at the pad's current height
    #     tightens by 6.3 mm over the 28 mm slide and peels the pad off the
    #     card face mid-press.  Climbing the card is not pressing deeper.
    #   * tight.  Measured: an envelope 2.35 mm past the card face capped
    #     the press at 2.0 N with the servo integrator saturated and still
    #     asking for more -- the press was ENVELOPE-limited, not
    #     FORCE-limited, which defeats the point of a force servo.  The arm
    #     is only ~1.1 N/mm at this pose (1.8 mm of commanded interference
    #     bought 2.0 N), so a few newtons needs a few millimetres.
    # Crushing is caught by CRUSH_GUARD, which is deck-relative and fires
    # long before this; that is the right division of labour.
    rear_face_a = float((np.array([deck_xs[-1], Y0, deck_z]) - O_deck) @ nrm) \
        - CARD_COLLISION_T / 2
    A_MIN = rear_face_a - 0.001

    def a_limit(b_tip=None):
        return A_MIN

    print(f"press envelope: a >= {A_MIN*1000:.2f} mm (card face a = "
          f"{A_FACE*1000:.2f} mm, i.e. {(A_FACE - A_MIN)*1000:.2f} mm of "
          f"usable advance; rear card's back face is at "
          f"{rear_face_a*1000:.2f} mm)")

    tcp_app = tcp_at(A_APP, B_PRESS) + upl * 0.035

    # WHICH WAY DOES THE TILT GO?  Do not derive it -- MEASURE it.  The pad
    # closing axis has to BE the card's face normal or the press is an edge
    # contact and a torque (which is how the card got rammed through the
    # slot wall).  Solve the start pose at both signs, keep the one whose
    # measured jaw axis aligns with the deck normal, and assert.
    best = None
    for sgn in (+1.0, -1.0):
        rig.solve_arm_start(tcp_app, grasp_quat(sgn * lean_th))
        lf = rig.ik.body_pose("fr3/fr3_leftfinger")
        rf = rig.ik.body_pose("fr3/fr3_rightfinger")
        jaw = np.asarray(lf[:3]) - np.asarray(rf[:3])
        jaw = jaw / max(np.linalg.norm(jaw), 1e-9)
        al = float(abs(jaw @ nrm))
        print(f"  grasp tilt {sgn*np.degrees(lean_th):+.2f} deg -> jaw "
              f"{np.round(jaw, 4)} vs deck normal {np.round(nrm, 4)}, "
              f"|cos| {al:.5f} ({np.degrees(np.arccos(min(al, 1.0))):.2f} deg)")
        if best is None or al > best[0]:
            best = (al, sgn, float(jaw @ nrm))
    align, GRASP_SGN, jaw_dot = best
    assert align > 0.995, f"grasp is not parallel to the deck: |cos| {align}"
    GRASP_TH = GRASP_SGN * lean_th
    rig.solve_arm_start(tcp_app, grasp_quat(GRASP_TH))
    tcp_fk = rig.ik.tcp_pose()
    print(f"arm start TCP err {np.linalg.norm(tcp_fk[:3]-tcp_app)*1000:.2f} mm")
    left_is_px = jaw_dot > 0.0
    print(f"grasp tilt {np.degrees(GRASP_TH):+.2f} deg, +n finger: "
          f"{'left(dof7)' if left_is_px else 'right(dof8)'}")
    lf = rig.ik.body_pose("fr3/fr3_leftfinger")
    rf = rig.ik.body_pose("fr3/fr3_rightfinger")

    # ── weak wrist stabilizer (inert until the pinch closes) ───────────
    l7 = rig.ik.body_pose("fr3/fr3_link7")
    T_l7 = wp.transform(wp.vec3(*l7[:3]), wp.quat(*l7[3:7]))
    obj_t = rig.builder.body_q[picked_body]
    T_obj = wp.transform(wp.vec3(obj_t[0], obj_t[1], obj_t[2]),
                         wp.quat(obj_t[3], obj_t[4], obj_t[5], obj_t[6]))
    rel = wp.transform_multiply(wp.transform_inverse(T_l7), T_obj)
    REL = world_T(np.array([*[rel[i] for i in range(3)],
                            *[rel[3 + i] for i in range(4)]]))
    hand_body = rig.builder.body_label.index("fr3/fr3_link7")
    # Built ONLY when the crutch is switched on.  A zero-gain d6 is inert
    # in XPBD but NOT in MuJoCo, which realises every d6 as a hard
    # constraint regardless of its gains -- so leaving one in place there
    # welds the object to the wrist and the fingers stop mattering.
    jg = None if not _STAB_BUILD else rig.builder.add_joint_d6(
        parent=hand_body, child=picked_body,
        linear_axes=tuple(JD(axis=a, target_ke=STAB_KE[0], target_kd=STAB_KD[0])
                          for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
        angular_axes=tuple(JD(axis=a, target_ke=STAB_KE[1], target_kd=STAB_KD[1])
                           for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
        parent_xform=rel, child_xform=wp.transform_identity(),
    )
    if jg is not None:
        rig.builder.joint_articulation[jg] = 0
    set_finger_effort_limit(rig.builder, limit_N=12.0)
    # arm LINKS never collide with the picked card; the FINGERS DO
    for ts in rig.builder.body_shapes.get(picked_body, []):
        for bl in range(0, hand_body + 1):
            for s2 in rig.builder.body_shapes.get(bl, []):
                rig.builder.add_shape_collision_filter_pair(s2, ts)

    cam_t = np.array([-0.02, 0.005, 0.05])
    rig.finalize(probes=["card_picked", "card_c",
                         "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"],
                 cam_pos=cam_t + np.array([0.16, -0.28, 0.14]),
                 cam_target=cam_t)
    rig.probes["grip_L"] = rig.probes.pop("fr3/fr3_leftfinger")
    rig.probes["grip_R"] = rig.probes.pop("fr3/fr3_rightfinger")
    px_probe = "grip_L" if left_is_px else "grip_R"   # the PRESSING pad
    # WHAT IS HOLDING THE CARD DOWN?  Pair-resolved probes on the picked
    # card, constructed BEFORE the pipeline is rebuilt (SensorContact must
    # exist before CollisionPipeline.contacts()).
    rig.probes["pk_holder"] = PairProbe(
        rig.model, "card_picked", ["holder", *proxy_labels])
    rig.probes["pk_deck"] = PairProbe(
        rig.model, "card_picked",
        ["card_a_shape", "card_b_shape", "card_c_shape"])
    # AND WHAT IS THE PAD ACTUALLY DELIVERING?  The whole slide is one
    # question -- does the pad's friction along +u beat card_c's friction
    # plus the card's weight -- and neither a whole-body probe nor
    # pk_holder/pk_deck answers it, because the pad's normal and card_c's
    # normal cancel in any vector sum.  This resolves the pressing pad on
    # its own; extra() logs the component of its friction along the slide
    # direction, which IS the drive.
    px_body = ("fr3/fr3_leftfinger" if left_is_px else "fr3/fr3_rightfinger")
    rig.probes["pk_pad"] = PairProbe(rig.model, "card_picked",
                                     counterpart_bodies=[px_body])
    print(f"pressing-pad body: {px_body}")

    GS.make_pipeline(rig, rigid_contact_max=40000,
                     max_triangle_pairs=4_000_000,
                     include_static_kinematic_pairs=False)
    rig.substeps = 64
    # 15 -> 30 iterations: XPBD friction is ITERATION-LIMITED, and this
    # task is a pure friction task (the card climbs only if the pad's
    # 0.625 mu beats the deck's 0.25 plus the card's weight).  The same
    # lever measurably changed grip strength in task_plug_insert_arm
    # (pad retention rose from 13 N to 15-17 N on 30 -> 60 iterations).
    # 30 -> 60 iterations.  XPBD friction is ITERATION-LIMITED and this task
    # is a pure friction task: the card climbs only if the pad's 0.625 mu
    # beats card_c's 0.25 plus the card's own weight.  MEASURED shortfall at
    # 30: a STATIC pad pressing 1.8-2.2 N could not hold the 0.147 N card,
    # which slid 0.3 mm down over 2 s -- against a Coulomb capacity of
    # 0.625*2.0 = 1.25 N, an 8x margin -- and during the slide the pad
    # outran the card 14.7 mm to 0.6 mm, a 25:1 slip ratio.  The same lever
    # measurably changed grip strength in task_plug_insert_arm (pad
    # retention 13 N -> 15-17 N on 30 -> 60).
    rig.solver = make_solver(rig.model, iterations=60)

    # MuJoCo needs fresh contact geometry for these thin cards.  Set the
    # cadence at construction too: teleop replaces the expert callback.
    rig.collide_every_substep = True

    # stabilizer inert until engaged (absent entirely when the crutch is off)
    if jg is not None:
        qd0 = int(rig.model.joint_qd_start.numpy()[jg])
        ke_arr = rig.model.joint_target_ke.numpy()
        kd_arr = rig.model.joint_target_kd.numpy()
        ke_engaged = ke_arr[qd0:qd0 + 6].copy()
        kd_engaged = kd_arr[qd0:qd0 + 6].copy()
        ke_arr[qd0:qd0 + 6] = 0.0
        kd_arr[qd0:qd0 + 6] = 0.0
        rig.model.joint_target_ke.assign(ke_arr)
        rig.model.joint_target_kd.assign(kd_arr)
        stab_idx = int(rig.model.joint_target_q_start.numpy()[jg])
    else:
        qd0 = stab_idx = None
        ke_arr = kd_arr = ke_engaged = kd_engaged = None

    pi = rig.body_index("card_picked")
    hi = rig.body_index("fr3/fr3_link7")
    tcp_off = tcp_fk[:3] - l7[:3]
    card_p0 = rig.s0.body_q.numpy()[pi, :3].copy()

    t1 = T_SETTLE
    t2 = t1 + T_DESCEND
    t2p = t2 + T_PUSH
    t2l = t2p + T_LOAD              # force-servo the press before sliding
    t3 = t2l + T_SLIDE              # slide may end early on the goal
    t_end = t3 + T_CLOSE + T_LIFT + LIFT_SLACK + T_HOLD

    pack_ids = [rig.body_index(l) for l in others]     # card_a/b/c only
    card_corners = trimesh.bounds.corners(card_local.bounds)
    rear_id = rig.body_index("card_a")                # rearmost card
    DECK_THICK0 = float(
        (rig.s0.body_q.numpy()[pi, :3] - rig.s0.body_q.numpy()[rear_id, :3])
        @ nrm)
    # The floor is SOLID CONTACT, not the as-built spacing.  deck_pose leaves
    # 0.08 mm of clearance per face so the deck settles onto itself instead
    # of starting interpenetrated, so the as-built thickness is 3*0.08 mm
    # ABOVE the packed thickness and closing that gap is the deck doing
    # exactly what it is supposed to do -- measuring the guard from the
    # as-built value fired it on frame one and froze the press integrator at
    # 0.08 mm, which is BELOW the arm's own tracking error, so the pad
    # reached 0.01 N.
    DECK_THICK_MIN = 3 * CARD_COLLISION_T - CRUSH_GUARD
    print(f"deck thickness at rest {DECK_THICK0*1000:.3f} mm, packed "
          f"{3*CARD_COLLISION_T*1000:.3f} mm, crush guard fires below "
          f"{DECK_THICK_MIN*1000:.3f} mm")
    st = {"phase": "settle", "s_up": 0.0, "frozen": None,
          "engaged": None, "u": 0.0, "t_eng": None, "xy_corr": np.zeros(2),
          "t_close": None, "lift_ref": None, "press_adj": 0.0,
          "x_cmd": None, "prev_cards": None, "step_mm": 0.0, "substep": 1,
          "max_step_mm": 0.0, "substep_frames": 0, "a_push": None,
          "frozen_b": 0.0,
          "top_z0": None, "slide_peak_N": 0.0, "grip_at_close": (0.0, 0.0)}

    def smooth(u):
        u = np.clip(u, 0.0, 1.0)
        return 3 * u * u - 2 * u ** 3

    def measured(bq):
        """Picked card's top edge and +n face, in world AND in deck (a, b).

        `theta` is the card's REAL lean now.  The old code returned it
        through np.clip(theta, TILT_MIN, TILT_MAX) with both bounds 0.0, so
        the logged `lean_deg` read 0.00 unconditionally and hid the fact
        that the press was rotating the card 19 deg PAST vertical.
        """
        T = world_T(bq[pi])
        p_top = T[:3, :3] @ top_local + T[:3, 3]
        p_face = T[:3, :3] @ np.array([CARD_T / 2, 0.0, 0.0]) + T[:3, 3]
        nx = T[:3, :3] @ np.array([1.0, 0.0, 0.0])
        theta = float(np.arctan2(-nx[2], nx[0]))
        a_face, _ = ab_of(p_face)
        _, b_top = ab_of(p_top)
        return p_top, p_face, theta, a_face, b_top

    def drive(pos, quat, qL, qR):
        q_sol = rig.ik.solve(pos, quat, iterations=16)
        qt = rig.control.joint_target_q.numpy()
        qt[:7] = q_sol[:7]
        qt[7] = qL
        qt[8] = qR
        rig.control.joint_target_q.assign(qt)

    def fingers_lr(q_px, q_mx):
        return (q_px, q_mx) if left_is_px else (q_mx, q_px)

    def press_face(F_pad, x_card_c, x_face_meas, b_tip=None):
        """Pad-face plane referenced to the CARD'S MEASURED FACE, with a
        bounded force-servoed interference on top and a rate limit.

        WHY IT IS NOT AN ABSOLUTE PLANE ANY MORE (measured with the
        per-counterpart probes, see the header).  The old law integrated a
        force error against the FIXED plane X_STOP.  But the deck MOVES:
        when the pad loads the picked card the pack compacts toward the
        wall, and the card's face retreated 3 mm in 0.16 s (~19 mm/s) while
        K_PRESS x (2.6 N) advances the command at only ~4.5 mm/s.  The
        servo can never catch a reference that runs away six times faster
        than its own slew, so it saturated at PRESS_ADJ_HI and sat there
        with the commanded plane 1.4-3.2 mm OUTSIDE the card face (logged
        as pad_face_gap_mm) and the pad press decaying 2.0 -> 0.19 N.  That
        is why widening the budget to 14 mm "fixed" the force and wrecked
        the penetration: it did not fix the tracking, it just let the
        integrator run until it buried the pad in the deck.

        Referenced to the measured face, the servo only ever needs the
        EQUILIBRIUM INTERFERENCE -- ~2.6 N over a ~20 N/mm chain, i.e.
        ~0.13 mm -- so its bounds shrink from millimetres to tenths.  The
        header's old warning that face-tracking "pumps the retreating card
        out of the holder at 60 Hz" is answered by PRESS_RATE: the command
        FOLLOWS the face at a bounded slew and never leads it.
        """
        step = K_PRESS * (PRESS_N - F_pad)
        if step > 0.0 and x_card_c < DECK_THICK_MIN:
            step = 0.0                  # the deck is being crushed, not loaded
        st["press_adj"] = float(np.clip(st["press_adj"] + step,
                                        PRESS_ADJ_LO, PRESS_ADJ_HI))
        target = float(x_face_meas) - st["press_adj"]
        if st["x_cmd"] is None:
            st["x_cmd"] = target
        st["x_cmd"] += float(np.clip(target - st["x_cmd"],
                                     -PRESS_RATE, PRESS_RATE))
        # the envelope is a HARD stop the integrator cannot walk past
        st["x_cmd"] = max(st["x_cmd"],
                          a_limit(B_PRESS if b_tip is None else b_tip))
        return st["x_cmd"]

    def deck_thickness(bq):
        # Project onto the CURRENT deck normal.  The original normal counts
        # upward sliding along a tilted stack as compression and shuts down
        # the press even when the cards are still a full thickness apart.
        normal = world_T(bq[rear_id])[:3, 0]
        return float((bq[pi, :3] - bq[rear_id, :3]) @ normal)

    def control(t):
        bq = rig.s0.body_q.numpy()
        p_top, p_face, th_m, a_face, b_top = measured(bq)
        # A press can tilt the stack and raise a neighbour's top edge.
        # Closing above the initial deck height then pinches two cards.
        b_pack_top = max(
            float(np.max((card_corners @ world_T(bq[i])[:3, :3].T
                          + bq[i, :3] - O_deck) @ upl))
            for i in pack_ids)
        b_min_tip = b_pack_top + TIP_MARGIN
        # Keep the existing displacement diagnostics: watch the pack for
        # the whole rollout and the picked card while it is in the holder.
        # These measurements no longer select the collision cadence.
        p_bot = (world_T(bq[pi])[:3, :3] @ np.array([0.0, 0.0, -CARD_H / 2])
                 + bq[pi, :3])
        ids = list(pack_ids)
        if float(p_bot[2]) < BRACKET_TOP_Z + 0.002:
            ids.append(pi)
        cp = bq[ids, :3]
        if st["prev_cards"] is None or len(st["prev_cards"]) != len(cp):
            st["step_mm"] = 0.0
        else:
            st["step_mm"] = float(
                np.linalg.norm(cp - st["prev_cards"], axis=1).max()) * 1000
            st["max_step_mm"] = max(st["max_step_mm"], st["step_mm"])
        st["prev_cards"] = cp.copy()
        # MuJoCo needs fresh geometry even at low translational speed:
        # rotation moves a card's edges without moving its centre, and the
        # governor only sees displacement after the preceding frame.  The
        # XPBD anchor workaround must not turn off substep collision here.
        rig.collide_every_substep = True
        st["substep"] = 1
        st["substep_frames"] += 1
        F = rig.probes["card_picked"].history[-1]["force"] \
            if rig.probes["card_picked"].history else 0.0
        F_pad = rig.probes[px_probe].history[-1]["force"] \
            if rig.probes[px_probe].history else 0.0
        gq = grasp_quat(GRASP_TH)
        # DECK THICKNESS, not absolute travel: picked card centre to
        # card_a centre along the card normal.  Invariant to the deck
        # standing up or sliding, so it isolates CRUSH.
        a_deck = deck_thickness(bq)

        if t < t1:                                      # settle
            drive(tcp_at(A_APP, B_PRESS) + upl * 0.035, gq,
                  *fingers_lr(Q_PRESS, Q_OPEN))
            return
        if t < t2:                                      # descend beside card,
            u = smooth((t - t1) / T_DESCEND)            # ALONG the card plane
            drive(tcp_at(A_APP, B_PRESS) + upl * (0.035 * (1.0 - u)), gq,
                  *fingers_lr(Q_PRESS, Q_OPEN))
            return
        if t < t2p:                                     # push onto the face
            u = smooth((t - t2) / T_PUSH)
            a_cmd = A_APP + (A_FACE - PRESS_ADJ_HI - A_APP) * u
            # A PRESS MUST BE BOUNDED BY FORCE, NOT BY TRAVEL (pitfall 6.2).
            # The sweep is open-loop, so once the pad is loaded it FREEZES
            # and hands over to the servo at the same command -- no step at
            # the handover, and no blind advance into a card that is
            # already carrying the target force.
            if F_pad >= PRESS_N and st["a_push"] is None:
                st["a_push"] = a_cmd
            if st["a_push"] is not None:
                a_cmd = st["a_push"]
            # THE OLD PUSH WAS A RAM.  Its stop, X_STOP, was a NOMINAL plane
            # derived from a fully wall-compacted VERTICAL deck
            # (SLOT_X[0] + 4*CARD_T = -32.2 mm); the deck neither stands
            # vertical nor compacts that far, so the sweep ran ~8 mm past the
            # card's real face and the pad, on a ~20 N/mm arm, swung the
            # card's bottom edge +x at 1.33 mm/frame straight THROUGH the
            # 1.5 mm slot wall.  The sweep now ends on the deck's own face
            # coordinate, and is additionally clamped to the same
            # interference bound the force servo uses about the MEASURED
            # face.  The clamp can only ever STOP the command early -- it
            # never advances it toward the card faster than the sweep, so
            # this is not the 60 Hz face-tracking that once pumped the card
            # out of the holder.
            a_cmd = max(a_cmd, a_face - PRESS_ADJ_HI, a_limit(B_PRESS))
            st["x_cmd"] = a_cmd         # seed the servo: no step at handover
            drive(tcp_at(a_cmd, B_PRESS), gq, *fingers_lr(Q_PRESS, Q_OPEN))
            return
        if t < t2l:                                     # LOAD: pure force
            drive(tcp_at(press_face(F_pad, a_deck, a_face, B_PRESS),
                         B_PRESS), gq,
                  *fingers_lr(Q_PRESS, Q_OPEN))         # servo, no slide
            st["top_z0"] = b_top
            st["slide_peak_N"] = max(st["slide_peak_N"], F_pad)
            return
        if st["phase"] == "settle":
            st["phase"] = "slide"
            print(f"t={t:.2f}s load done: card top b {st['top_z0']*1000:.2f} mm, "
                  f"pad press {F_pad:.2f} N, adj {st['press_adj']*1000:.2f} mm")
        if st["phase"] == "slide":
            st["slide_peak_N"] = max(st["slide_peak_N"], F_pad)
            rise = b_top - st["top_z0"]                  # UP THE CARD'S PLANE
            if st["s_up"] - rise < SLIP_CAP and st["s_up"] < S_UP_MAX:
                st["s_up"] += V_SLIDE / rig.fps
            tips = B_PRESS + st["s_up"]
            a_cmd = press_face(F_pad, a_deck, a_face, tips)  # 1 step/frame
            drive(tcp_at(a_cmd, tips), gq, *fingers_lr(Q_PRESS, Q_OPEN))
            proud = b_top - b_pack_top
            if (proud >= PROUD_GOAL and tips >= b_min_tip) \
                    or st["s_up"] >= S_UP_MAX or t >= t3:
                st["phase"] = "close"
                st["t_close"] = t
                st["frozen_b"] = max(tips, b_min_tip)
                st["frozen"] = tcp_at(a_cmd, st["frozen_b"])
                st["close_a"] = a_cmd
                print(f"t={t:.2f}s slide done: card top b "
                      f"{b_top*1000:.2f} mm ({proud*1000:.2f} mm proud), "
                      f"s_up {st['s_up']*1000:.1f} mm, tips b "
                      f"{tips*1000:.1f} mm, pad press {F_pad:.2f} N")
            return
        tcp_p = st["frozen"]
        if st["phase"] == "close":       # far pad sweeps in OVER the pack
            u = (t - st["t_close"]) / T_CLOSE
            if u < 0.6:                  # fast to 5 mm, then slow onto the
                q_mx = Q_OPEN + (0.005 - Q_OPEN) * smooth(u / 0.6)   # card
            else:
                q_mx = 0.005 + (Q_PINCH - 0.005) * smooth((u - 0.6) / 0.4)
            # the pressing pad keeps its force servo running until the far
            # pad has taken the card: releasing early drops it back down
            a_close = (press_face(F_pad, a_deck, a_face, st["frozen_b"])
                       if u < 0.6 else st["close_a"])
            if u < 0.6:
                st["close_a"] = a_close
            # FR3 closes both fingers together.  Translate the TCP by the
            # same amount so the pressing pad stays on the card while the
            # opposite pad approaches; independent jaw targets fight the
            # imported mimic constraint and never establish the press.
            pos = tcp_at(a_close, st["frozen_b"], q_px=q_mx)
            drive(pos, gq, q_mx, q_mx)
            st["frozen"] = tcp_p = pos.copy()
            if u >= 1.0:
                st["phase"] = "lift"
                gl = rig.probes["grip_L"].history[-1]["force"]
                gr = rig.probes["grip_R"].history[-1]["force"]
                st["grip_at_close"] = (round(gl, 2), round(gr, 2))
                lost = (not np.isfinite(p_top).all()) or \
                    np.linalg.norm(p_top - (bq[hi, :3] + tcp_off)) > 0.03 \
                    or min(gl, gr) < 0.5
                st["lift_ref"] = (tcp_p.copy(),
                                  p_top - (bq[hi, :3] + tcp_off))
                if lost:
                    # no pinch -> DO NOT engage the stabilizer: the coupling
                    # would carry the card out with zero grip force and the
                    # rollout would look like a pick that never happened
                    print(f"t={t:.2f}s NO PINCH (grip {gl:.2f}/{gr:.2f} N) "
                          f"- stabilizer left inert")
                    st["t_eng"] = t
                    return
                tgt6 = stab_zero_stress_targets(bq[hi], bq[pi], REL)
                if jg is not None:
                    qt = rig.control.joint_target_q.numpy()
                    qt[stab_idx:stab_idx + 6] = tgt6
                    rig.control.joint_target_q.assign(qt)
                    ke_arr[qd0:qd0 + 6] = ke_engaged
                    kd_arr[qd0:qd0 + 6] = kd_engaged
                    rig.model.joint_target_ke.assign(ke_arr)
                    rig.model.joint_target_kd.assign(kd_arr)
                st["engaged"] = tgt6
                st["t_eng"] = t
                print(f"t={t:.2f}s pinch confirmed (grip {gl:.2f}/{gr:.2f} N)")
            return
        # lift: force-gated.  Leg 1 runs UP THE CARD'S OWN PLANE (along u)
        # until the card's bottom edge is clear of the z=37 slot-end
        # brackets -- a straight-up leg would drag the leaning card's bottom
        # corner against the +x wall for the whole climb.  Leg 2 is +x.
        gate = GATE_MIN + (1.0 - GATE_MIN) * float(np.clip(
            (GATE_STOP - F) / (GATE_STOP - GATE_FULL), 0.0, 1.0))
        if st["u"] < 1.0:
            st["u"] = min(1.0, st["u"] + gate / (T_LIFT * rig.fps))
        s = smooth(st["u"])
        sx = smooth(max(0.0, st["u"] - LIFT_XS) / (1.0 - LIFT_XS))
        tcp0, top_rel0 = st["lift_ref"]
        tcp_meas = bq[hi, :3] + tcp_off
        tgt_top = tcp_meas + top_rel0
        st["xy_corr"] += K_XY * (p_top[:2] - tgt_top[:2] - st["xy_corr"])
        st["xy_corr"] = np.clip(st["xy_corr"], -XY_CORR_MAX, XY_CORR_MAX)
        pos = tcp0 + upl * (LIFT_DZ * s) + np.array([LIFT_DX * sx, 0.0, 0.0]) \
            + np.array([*st["xy_corr"], 0.0])
        drive(pos, gq, *fingers_lr(Q_PINCH, Q_PINCH))

    def extra(state, t):
        bq = state.body_q.numpy()
        gl = rig.probes["grip_L"].history[-1]["force"]
        gr = rig.probes["grip_R"].history[-1]["force"]
        p_top, p_face, th_m, a_face, b_top = measured(bq)
        # WHERE THE PINCH ACTUALLY IS.  Under MuJoCo the pads read 0.1 N at
        # a commanded interference worth ~3 N, and contact compliance only
        # accounts for a third of that -- so log the finger coordinate
        # against its target and stop guessing which of the two is wrong.
        jq = state.joint_q.numpy()
        jt = rig.control.joint_target_q.numpy()
        return {
            "fingers_mm": round(float(jq[7] + jq[8]) * 1000, 3),
            "fingers_tgt_mm": round(float(jt[7] + jt[8]) * 1000, 3),
            "card_dz_mm": round((bq[pi, 2] - card_p0[2]) * 1000, 2),
            "card_top_z_mm": round(p_top[2] * 1000, 1),
            # the card's REAL lean.  The old log field read 0.00 forever
            # because it was clipped to TILT_MIN==TILT_MAX==0.
            "card_lean_deg": round(np.degrees(th_m), 2),
            "card_top_b_mm": round(b_top * 1000, 2),
            # the press servo's own state, so a stalled slide can be read
            # off the force log instead of re-derived: how far the servo has
            # advanced, how far the pad has swept up, and the SIGNED gap
            # between the commanded pad-face plane and the card's measured
            # +x face at the pad's height (negative = the command is inside
            # the card, i.e. the pad should be loaded)
            # signed vertical force on the picked card, per counterpart:
            # this is the measurement that says whether the slot/brackets or
            # the deck is what the friction slide is fighting
            "hold_holder_fz_N": round(
                rig.probes["pk_holder"].history[-1]["fz"], 3)
            if rig.probes["pk_holder"].history else 0.0,
            "hold_deck_fz_N": round(
                rig.probes["pk_deck"].history[-1]["fz"], 3)
            if rig.probes["pk_deck"].history else 0.0,
            # the pressing pad, resolved on its own: total force, friction
            # magnitude, and the friction component ALONG THE SLIDE (+u).
            # That last number is the drive; everything else is bookkeeping.
            "pad_N": round(rig.probes["pk_pad"].history[-1]["force"], 3)
            if rig.probes["pk_pad"].history else 0.0,
            "pad_fric_N": round(rig.probes["pk_pad"].history[-1]["friction"], 3)
            if rig.probes["pk_pad"].history else 0.0,
            "pad_drive_u_N": round(float(
                np.asarray(rig.probes["pk_pad"].history[-1]["vec"]) @ upl), 3)
            if rig.probes["pk_pad"].history else 0.0,
            "deck_drag_u_N": round(float(
                np.asarray(rig.probes["pk_deck"].history[-1]["vec"]) @ upl), 3)
            if rig.probes["pk_deck"].history else 0.0,
            "press_adj_mm": round(st["press_adj"] * 1000, 2),
            "s_up_mm": round(st["s_up"] * 1000, 2),
            # Displacement diagnostics and the current collision cadence.
            "card_step_mm": round(st["step_mm"], 3),
            "collide_substep": st["substep"],
            # the deck's own thickness (picked centre -> card_a centre along
            # the card normal): the crush guard's input, and the direct
            # answer to "is the press squashing the deck?"
            "deck_thick_mm": round(deck_thickness(bq) * 1000, 3),
            # SIGNED gap between the commanded pad-face plane and the card's
            # measured face, both in the deck's normal coordinate (negative
            # = the command is inside the card, i.e. the pad is loaded)
            "pad_face_gap_mm": round(
                ((st["x_cmd"] if st["x_cmd"] is not None else A_APP)
                 - a_face) * 1000, 2),
            "grip_min_N": round(min(gl, gr), 3),
            "phase": st["phase"],
        }

    rec = rig.run(t_end, control, extra)
    paths = rec.save("rollout", force_keys=["card_picked_N",
                                            "card_picked_fric_N",
                                            "grip_L_N", "grip_R_N",
                                            "card_c_N"])
    print("saved:", paths)

    # ── QA ─────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    d = np.load(paths["states"], allow_pickle=True)
    bq, t = d["body_q"], d["t"]
    labels = list(d["body_labels"])

    p = bq[:, pi, :3]
    dz = float(p[-1, 2] - p[0, 2])
    lat = float(np.linalg.norm(p[-1, :2] - p[0, :2]))
    others_disp = {}
    for lbl in others:
        q = bq[:, labels.index(lbl), :3]
        others_disp[lbl] = {
            "final_mm": round(float(np.linalg.norm(q[-1] - q[0])) * 1000, 2),
            "max_mm": round(float(np.linalg.norm(q - q[0], axis=1).max()) * 1000, 2),
        }
    others_ok = all(v["final_mm"] < 15.0 for v in others_disp.values())

    # grip force through the lift
    tl = np.array([r["t"] for r in rig.probes["grip_L"].history])
    fl = np.array([r["force"] for r in rig.probes["grip_L"].history])
    fr = np.array([r["force"] for r in rig.probes["grip_R"].history])
    fc = np.array([r["force"] for r in rig.probes["card_picked"].history])
    lift_w = tl >= (st["t_eng"] + 0.2 if st["t_eng"] else t[-1])
    grip_min = float(np.minimum(fl, fr)[lift_w].min()) if lift_w.any() else 0.0
    grip_mean = float(((fl + fr) / 2)[lift_w].mean()) if lift_w.any() else 0.0
    ride_peak = float(fc[lift_w].max()) if lift_w.any() else 0.0

    ok = bool(dz >= 0.05 and lat >= 0.03 and others_ok and grip_min > 1.0
              and st["engaged"] is not None)
    qa.report["checks"]["task_success"] = {
        "pass": ok,
        "picked_rise_cm": round(dz * 100, 2),
        "picked_lateral_cm": round(lat * 100, 2),
        "grip_min_N_during_lift": round(grip_min, 2),
        "grip_mean_N_during_lift": round(grip_mean, 2),
        "grip_at_pinch_N": list(st["grip_at_close"]),
        "slide_press_peak_N": round(st["slide_peak_N"], 2),
        "card_rise_before_pinch_mm": round(
            (float(bq[int(st["t_eng"] * rig.fps) - 1, pi, 2]) - card_p0[2])
            * 1000, 2) if st["t_eng"] else 0.0,
        "bracket_ride_peak_N": round(ride_peak, 2),
        "others_disp": others_disp,
        "max_card_step_mm": round(st["max_step_mm"], 3),
        "substep_frames": st["substep_frames"],
        "note": "picked card >=5 cm up and >=3 cm lateral; others <1.5 cm; "
                "both finger forces >1 N through the lift (real pinch: the "
                "wrist stabilizer is only engaged if both pads read >0.5 N)",
    }
    qa.report["pass"] &= ok
    qa.report["checks"]["rest_clearance"] = {
        "pass": True, "penetration_mm": rest,
        "note": "build-time geometric clearance, negative = separated",
    }

    # official per-card vs holder checks (tol 0.8 mm)
    np.random.seed(0)
    pts800, _ = trimesh.sample.sample_surface_even(card_local, 800)
    cloud = trimesh.Trimesh(vertices=np.asarray(pts800),
                            faces=np.zeros((0, 3), dtype=int), process=False)
    for lbl in names:
        qa.check_penetration_static(lbl, cloud, holder_raw, holder_c,
                                    f"{lbl}->holder", tol=0.0008)

    # dense full-rollout audit: every pair, both directions, 28 frames
    frame_ids = sorted(set(np.linspace(0, len(t) - 1, 28).astype(int)))
    frames = [(f, float(t[f])) for f in frame_ids]

    def get_T(lbl, f):
        if lbl == "holder":
            T = np.eye(4)
            T[:3, 3] = holder_c
            return T
        return world_T(bq[f, labels.index(lbl)])

    dense_raw = audit_pairs(get_T, frames)
    dense = {}
    for k, (worst, wt) in dense_raw.items():
        dense[k] = {"max_depth_mm": round(worst * 1000, 3),
                    "at_t": wt, "pass": bool(worst < 0.0008)}
        print(f"dense audit {k}: {worst*1000:+.3f} mm at t={wt:.2f}")
    ok_dense = all(v["pass"] for v in dense.values())
    qa.report["checks"]["penetration_dense_audit"] = {
        "pass": ok_dense, "tolerance_mm": 0.8, "pairs": dense,
        "note": "trimesh ground truth, subdivided surfaces, ray-verified "
                "inside sign, 28 frames, both directions per pair",
    }
    qa.report["pass"] &= ok_dense

    # tactile stacks + gel gates (no-op unless TB_GELSIGHT=1): loaded
    # window = pinch engage -> end (press/slide load only ONE pad; the
    # gates need both, so they run on the two-pad phase), cross-checked
    # against the same grip_L/grip_R finger<->card probes the QA uses
    _w0 = (st["t_eng"] + 0.3) if st["t_eng"] else t3
    GS.save_tactile(
        rig, [GS.object_spec(rig, "card_picked",
                             tm=card_local, center=(0.0, 0.0, 0.0))],
        qa=qa, window=(_w0, t_end),
        probe_forces={
            "gel_left": [r["force"] for r in rig.probes["grip_L"].history],
            "gel_right": [r["force"] for r in rig.probes["grip_R"].history]})
    rig.finish(qa)


if __name__ == "__main__":
    wp.init()
    main()
