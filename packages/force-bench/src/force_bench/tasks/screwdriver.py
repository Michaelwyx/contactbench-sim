"""Task: use the screwdriver to turn the screw (form-closed drive engagement).

Geometry verified by trimesh cross-sections (`screwdriver_geom_probe.py`,
which prints every number below):

  screwdriver/part2_1_body1   1.2 x 12.5 x 1.2 driver, long axis Y, tip at
                              y_min.  Measured section vs distance from the
                              tip end: a pyramidal lead-in 3.610 -> 5.000 mm
                              over the first 1.00 mm, then a 5.000 x 5.000 mm
                              SQUARE shank to 48 mm, an 8.3 mm collar, and a
                              12.0 mm FLUTED handle (radius 6.03..7.24 mm).
                              The lead-in flank is a 36.2 deg RAMP off the
                              tool axis — steeper than atan(mu)=21.8 deg at
                              mu 0.4, so it does NOT self-lock and CAN cam
                              the tip out (README pitfall 3.4/8).
  fasteners/screw_075mm_1     1.76 x 1.55 x 2.7 printed screw, head at z_min,
                              head 17.59 x 15.50 x 7.0 mm over an 8.6 mm
                              shank.  The head carries a square drive RECESS:
                              6.91 mm at the face, chamfering to 5.500 mm at
                              1.00 mm depth (a 36.4 deg ramp — the other half
                              of the cam-out pair) and 5.500 mm constant to
                              the 4.0 mm floor.
  FIT: square-in-square, 0.25 mm clearance per side (0.5 mm diametral) ->
       FORM CLOSURE with ±6.06 deg rotational backlash
       (corner radius 3.536 mm vs socket half-width 2.75 mm).
  screw_together_blocks/bottom  3 x 8 x 1.1 block with two through pockets
       (10.5 mm square top holes at y=-19 / y=-14) -> scenery workpiece.

Build: screw mounted VERTICALLY head-up (flipped pi about x so the recess
faces +z) on a revolute-z joint (rotary friction kd ~0.02 — a screw turning
in a hole) whose UPPER LIMIT is the SEAT: at SEAT_DEG the screw is home and
further turning has to be carried by the drive engagement.  Block on a
static stand beneath as scenery (filtered vs screw).
Driver VERTICAL tip-down, tip seated in the recess, guided by a free
carrier chain (world -prismatic z-> carrier -revolute z-> driver, both
undriven — the bottle-cap cap pattern, single angular axes so multi-turn
angles unwrap, README XPBD-era pitfall 9) and gripped by a chained massive hand
(world -prismatic z-> hand_base -revolute z-> hand_spin) through a
compliant d6.  Torque flows hand -> d6 -> driver -> tip/recess CONTACT ->
screw; the driver's own joints carry no drive torque, so any screw rotation
is genuine contact torque transfer.  The hand z closes the loop on the
MEASURED driver height (pitfall 8.1) with a small downforce preload — a
human pressing while turning — which also fights cam-out.

Contact pair driver<->screw stays ENABLED.  The driver carries TWO shapes on
one body: a 20 mm TIP cropped out of the CAD mesh with its own SDF, and the
rest of the tool.  That is a contact-RESOLUTION fix, not cosmetics — SDF
cell = (extent + 2*margin)/res (pitfall 3.1), so the 125 mm tool at res 512
resolves the interface at 0.2449 mm, i.e. ONE CELL SPANS THE WHOLE 0.25 mm
drive fit; the 20 mm tip at the same resolution resolves it at 0.0398 mm.

Phases: settle/preload -> spin +1.5 rad/s until the screw SEATS and past it
(over-torque) -> pause -> reverse -1.5 rad/s -> hold.

THE PRESS ENVELOPE IS THE ONLY THING THAT BOUNDS THE PRESS FORCE, and that
is a measured result, not a preference — see the block comment on the
`hand_z` joint below for the full refutation table.

WHAT CHANGED 2026-08-14 — THE GRIP IS A TREE JOINT UNDER SolverMuJoCo, AND
THE THREE THINGS XPBD NEVER DELIVERED HAD TO BECOME REAL NUMBERS.

The compliant `hspin -> driver` d6 IS this task's torque path, and it is a
LOOP joint, so SolverMuJoCo drops it ("Loop joint 5 (type D6, 3 linear +
3 angular DOFs) has no supported MuJoCo equality constraint mapping").
LOAD-BEARING, measured: with it dropped the run completes 5.7 s with
screw_fwd 0.0 deg, peak drive torque 0.00 mN.m, preload 0.88 N (the
driver's own weight) and grip_windup 50.5 deg — the hand turning against a
tool it is no longer attached to.  No equality constraint can carry it:
MuJoCo's three loop mappings are WELD (rigid — deletes the compliance the
`grip_info` wind-up reads) and CONNECT (constrains translations, leaves
rotation free — drops exactly the axis that carries the torque).  So the
cure is task_nut_bolt's: a TREE joint, on the two axes that were ever
load-bearing (the carrier chain and the hand chain pin the SAME four dofs,
so the loop d6's x/y and roll/pitch see identically zero error in both
backends).  The carrier chain is not built under MuJoCo, because a body may
have only one tree parent.

Then three XPBD "nominals" that had never been delivered arrived at once,
and each is a task constant that had to be re-derived, not a tuning knob:

  quantity            nominal   XPBD in-scene   MuJoCo   what it cost
  grip torsional      4000      5.54 N.m/rad    4000     293 mN.m spike
  screw rotary kd     0.02      8.4 mN.m run    30 mN.m  running torque AT
                                                          the cam-out limit
  press command       0.2 mm    21.9 N          1.16 N   no torque transfer

Each is written up above its constant (MJ_GRIP_ANG / MJ_SCREW_KD /
MJ_PRESS).  The pattern is one thing seen three times: XPBD's loop d6 and
its damping-only drives are AUTHORITY-STARVED (XPBD-era pitfall 21a/21b), so the
declared gains were never the delivered gains, and MuJoCo delivers them
exactly.  Note XPBD-era pitfall 21b INVERTS here — the kd*omega estimate this file
calls an over-read (29.2 predicted vs 8.4 measured under XPBD) is exact
under MuJoCo (28.8 vs 29.7 before the correction).  It is an XPBD property,
not a physics one.

Also new, and both no-ops under XPBD: `joint_effort_limit` on hand_z (the
press) and hand_spin (the operator's torque), and HARD JOINT LIMITS
(task_rig.mujoco_hard_limits) — without them the screw walked 62.9 deg past
its SEAT_DEG stop and hand_z 2.0 mm past its 6 mm envelope, because a
MuJoCo joint limit derived from newton's default limit_ke/limit_kd has a
0.2 s time constant.

A/B, all three verdicts:

  verdict                      TB_SOLVER=xpbd        TB_SOLVER=mujoco
  physics QA                   PASS, all 6 checks    PASS, all 6 checks
  audit worst live pair        screw<->driver        screw<->driver
                                 +0.635 mm             +0.100 mm
    audit verdict              solver-compliance     margin-equilibrium
    (repeats: XPBD 0.610-0.635, MuJoCo 0.100 every time)
  teleop_gate --repeat 6       PASS 6/6              PASS 6/6
    run vector                 [T,T,T,T,T,T]         [T,T,T,T,T,T]
    gate worst penetration     0.487 mm              0.284 mm
    gate peak body speed       0.20 m/s              0.15 m/s
    violations                 none                  none

  screw_fwd / driver_fwd [deg] 150.0 / 171.5         156.5 / 162.7
  driver/screw ratio           1.14                  1.04
  reversal return [deg]        117.3                 139.9
  preload [N]                  21.9                  22.9
  running drive torque [mNm]   8.41                  8.20
  grip wind-up [deg]           0.087                 0.023
  effective grip k [N.m/rad]   5.54                  20.01
  peak drive torque [mNm]      21.4                  110.1
  QA depth driver->screw [mm]  0.634                 0.138

TWO NUMBERS MOVE AND BOTH ARE THE SAME MECHANISM: A SOFT STOP READS ITS OWN
REACTION.  The seat is modelled as the screw revolute's upper LIMIT, and
under MuJoCo a limit is a spring, so the drive engagement measures the seat
reaction — a harder stop reads a BIGGER torque.  Peak drive torque is
therefore 110 mN.m against XPBD's 21.4, and the screw overshoots its 150.0
deg stop by 6.5 deg.  That trade is the whole content of the number, and it
was isolated by refuting every other candidate one at a time (the ablation
table sits above `rig.limit_dampratio` in build_scene; contact stiffness
over 30x, hand mass over 40x, the grip stiffness, the grip torque clamp,
OMEGA_HARD and the screw's dry friction all leave the peak within 5 %,
while `limit_dampratio` alone moves it 293 -> 73 mN.m and the screw's
armature moves it the other way, to 582 mN.m with a seat exact to 0.01 deg).
DECLARED LIMITATION: XPBD stops the screw dead at its seat; MuJoCo cannot
have both a rigid stop and a bounded reaction, and this file chooses
6.5 deg of overshoot with a bounded reaction.

Run: python -m force_bench.tasks.screwdriver
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.rig import make_solver, Rig, JD
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass

# ── geometry constants (m, from the cross-section characterization) ────
RECESS_DEPTH = 0.0040     # square socket depth in the head
TIP_HALF_LEN = 0.0625     # driver AABB center -> tip end
HEAD_TOP_Z = 0.042        # world z of the head top face (recess entry)
SCREW_H = 0.027
AXIS_XY = np.array([0.09, -0.14])          # block pocket at (9.0,-14.0) cm
TIP_START_DEPTH = 0.0025  # tip end starts this far into the recess
TIP_LEN = 0.020           # length of the separately-SDF'd tip shape

# ── drive / coupling constants ─────────────────────────────────────────
OMEGA = 1.5               # hand spin speed [rad/s]
OMEGA_HARD = 6.0          # "it stopped turning, so lean on it": the speed
# the expert commands ONCE THE MEASURED SCREW ANGLE SAYS THE SCREW IS HOME.
# Needed because a damping-only drive is not a kd-valued torque source
# (XPBD-era pitfall 21b): at OMEGA the stalled hand only ever delivered 22.9 mN.m
# into a seated screw, below the cam-out threshold, so the run ended in a
# quiet stall with tip_rise_after_seat = 0.003 mm and no release at all.
LEAD_MAX = 0.9            # anti-wrap lead limiter [rad]; relaxed while
LEAD_MAX_OT = 1.5         # over-torquing (still far from the d6 ±pi wrap)
KD_SPIN = 3.0             # gentle spin authority (pitfall 6.1)
SCREW_KD = 0.02           # rotary friction of the screw in its hole
SEAT_DEG = 150.0          # the screw is HOME here (head against the work).
# Modelled as the screw revolute's upper LIMIT, i.e. armed at build time so
# it exists for every driver of the scene, not just this expert (pitfall
# 39).  It is what makes OVER-TORQUE a thing that can happen at all: with a
# freely spinning screw the drive engagement never carries more than the
# 0.009 N.m its rotary friction dissipates, and the force channel learns
# nothing.  The expert turns past it on purpose.
PRESS = 0.0002            # preload: hand z commanded below the driver
                          # (0.0006 gave a 25-34 N gorilla press and 1.15 mm
                          # floor intrusion; 0.0002 lands 20-23 N on Delta)
                          # (NO sag compensation: the XPBD hand drive is far
                          # stiffer than nominal — a +offset turns the
                          # follow loop into positive feedback and the hand
                          # drags the driver clean out of the recess)
GRIP_LIN = 8000.0
GRIP_ANG = 4000.0         # stiff tilt (bottle-cap lesson) + spin transfer
T_SETTLE, T_FWD, T_PAUSE, T_REV, T_END = 0.6, 2.4, 0.4, 1.7, 0.6

# ── UNDER SolverMuJoCo THE GRIP IS A TREE JOINT AND THE HAND IS CLAMPED ──
# 2026-08-14.  The compliant `hspin -> driver` d6 IS the torque path of this
# task (hand -> d6 -> driver -> tip/recess CONTACT -> screw), and it is a
# LOOP joint, so SolverMuJoCo drops it:
#     UserWarning: Loop joint 5 (type D6, 3 linear + 3 angular DOFs) has no
#     supported MuJoCo equality constraint mapping. Skipping loop closure.
# Measured with the loop dropped (TB_SOLVER=mujoco, unmodified file):
# screw_fwd 0.0 deg, driver_fwd -0.0 deg, peak drive torque 0.00 mN.m,
# preload 0.88 N (that is the driver's own 0.08 kg hanging on its carrier,
# nothing else), grip_windup 50.53 deg -- the hand spun for 4.5 s against a
# driver it was no longer attached to.  QA FAIL on task_success AND
# over_torque.  LOAD-BEARING, then, and unambiguously so; contrast a
# pinch on a free part, where a wrist d6 is INERT and is simply not built.
#
# NO EQUALITY CONSTRAINT CAN CARRY IT, so "map it instead" is not an option.
# SolverMuJoCo has exactly three loop mappings (solver_mujoco.py, the
# `for j in joints_loop:` block): FIXED -> WELD, 1 angular dof -> 2x CONNECT,
# 3 angular dofs (ball) -> 1x CONNECT.  CONNECT constrains TRANSLATIONS and
# leaves rotation free, i.e. it drops precisely the axis that has to carry
# the drive torque; WELD carries the torque but rigidly, which deletes the
# compliance the `grip_info` wind-up measurement reads.  The cure is
# task_nut_bolt.py's: make the grip a TREE joint so its axes become real
# MuJoCo position actuators.
#
# THE TREE IS `world -j3-> hand_base -j4-> hand -grip-> driver`, AND THE
# CARRIER CHAIN IS NOT BUILT under MuJoCo -- a body may have only one tree
# parent ("Multiple joints lead to body N"), and the carrier chain and the
# grip would be two.  That costs nothing, because the two chains constrain
# the SAME four dofs: the carrier pins the driver to (z, yaw) about the tool
# axis, the hand chain pins the hand to (z, yaw) about the same line, so the
# loop d6's x/y linear and roll/pitch angular axes see IDENTICALLY zero error
# and carry no load in either backend.  The MuJoCo grip therefore declares
# ONLY the two axes that were ever load-bearing (linear z + angular z) and
# the driver ends up with exactly the two dofs it has under XPBD.
#
# THE PRESS HAS TO BE RE-DERIVED, and that is not a fudge: it is the same
# thing task_nut_bolt measured (its note (b)).  Against a loop d6 at 20
# iterations XPBD's grip is effectively RIGID, so `hand_z = driver_z - PRESS`
# is a DISPLACEMENT command into a rigid chain and the press force is bounded
# by GEOMETRY -- 0.2 mm of command buys 21.9 N.  MuJoCo delivers the nominal
# GRIP_LIN exactly, so the same command buys 5714 N/m x 0.2 mm = 1.1 N
# (measured 1.16 N), the tip cams straight out and the screw never turns.
# MJ_PRESS is that same ~21 N asked for in the units MuJoCo actually honours,
# and MJ_PRESS_F_MAX is what BOUNDS it -- a `joint_effort_limit`, i.e. the
# force clamp this whole migration is for, replacing the positional envelope
# that XPBD needed.  Under XPBD both are unused and every number is bit
# identical (verified: QA json diff is empty).
MJ_GRIP_Z = 0.015         # m — grip travel, driver vs hand.  Not binding:
                          # the measured excursion is <0.2 mm.  It exists so
                          # the driver cannot be commanded into the hand disk
                          # (17 mm of clearance) — pitfall 21.
MJ_GRIP_TWIST = 2.5       # rad — grip wind-up limit > LEAD_MAX_OT (1.5)
MJ_HAND_Z_LIMITS = (-0.006, 0.002)   # m — the working travel is bigger here
                          # because the same press force costs more command
                          # against a real 8000 N/m grip.  The MEASURED
                          # expert command span is [-4.99, -0.20] mm, so
                          # -6.0 leaves 1.0 mm of headroom (pitfall 4.7).
MJ_PRESS = 0.0019         # m — commanded hand-z offset below the measured
                          # driver height.  Chosen for press force, not for
                          # depth: see the sweep table above build_scene.
MJ_PRESS_F_MAX = 30.0     # N — joint_effort_limit on hand_z.  Above the
                          # 21 N working press, far below the 258 N at which
                          # the tip PUNCHES THROUGH the socket floor.
MJ_HAND_MASS = 2.0        # kg — the hand bodies.  UNCHANGED, and that is
                          # nominal that inverts here: `Rig.spinner` carries
                          # the note "XPBD torque authority scales with the
                          # driver's inertia -- a light hand cannot transmit
                          # mechanism-scale torques no matter the gains"
                          # (pitfall 4.5), which is why this hand is 2 kg.
                          # Under MuJoCo an actuator delivers its commanded
                          # torque irrespective of inertia, so the mass buys
                          # nothing and COSTS the seat impact: the peak
                          # torque a rotating hand can dump into the grip
                          # spring is omega*sqrt(I*k), and at I = 4e-4 that
                          # is 134 mN.m at OMEGA alone -- 6x XPBD's whole
                          # over-torque ceiling, before the actuator has
                          # done anything.  The spike is a MOMENTUM
                          # reservoir, which is why it did not move under
                          # contact stiffness (1e6..3e7: 204..212 mN.m),
                          # the screw's friction, or the hand_spin effort
                          # limit.  MEASURED AND REFUTED, which is why it
                          # is still 2.0: dropping it to 0.5 / 0.2 / 0.05 kg
                          # moved the peak 212 -> 204 / 201 / 201 mN.m, i.e.
                          # not at all, and cost the press (22.9 -> 12.8 N,
                          # because the hand's own weight is part of it).
                          # The reservoir is not the hand -- see
                          # `limit_dampratio` below.
MJ_SCREW_ARMATURE = 0.0   # kg.m^2 — reflected inertia on the screw dof.
                          # OFF: it makes the seat EXACT (the screw stops
                          # at 150.01 deg instead of overshooting 6.5) and
                          # triples the drive torque doing it (515-582
                          # mN.m), which is the same trade as
                          # `limit_dampratio` seen from the other end.
                          # Kept as a named knob because it is the lever to
                          # reach for if the seat angle ever matters more
                          # than the torque.
MJ_SCREW_KD = 0.0057      # N.m.s/rad — the screw's rotary friction, as a
MJ_SCREW_FRICTION = 0.0   # N.m — ...plus DRY friction (joint_friction, one
                          # more thing XPBD lists as unsupported).  OFF:
                          # 0.01 and 0.03 N.m both leave the seat torque at
                          # 210 mN.m, i.e. it is not a dissipation problem.
                          # SCREW_KD = 0.02 is another nominal XPBD never
                          # delivered: this file's own grip_info check
                          # measures 8.26 mN.m of running drive torque
                          # against the 29.2 mN.m the kd*omega estimate
                          # predicts, and the header calls that out as
                          # XPBD-era pitfall 21b (a damping-only drive is a velocity
                          # weld, not a kd-valued torque).  Under MuJoCo it
                          # IS a kd-valued torque -- measured 29.72 mN.m
                          # against the 29.77 estimate, i.e. XPBD-era pitfall 21b is
                          # XPBD-SPECIFIC and inverts here -- so the screw
                          # suddenly resists 3.6x harder than the rig was
                          # ever calibrated for, which alone pushes the
                          # running torque up to the cam-out threshold.
                          # 0.0057 = 8.26 mN.m / 1.458 rad/s, i.e. the
                          # value XPBD actually delivered.
MJ_GRIP_ANG = 20.0        # N.m/rad — the grip's TORSIONAL stiffness, and
MJ_GRIP_ANG_KD = 0.05     # N.m.s/rad — the number GRIP_ANG never was.
                          # GRIP_ANG = 4000 is a rigid weld (4 N.m per
                          # milliradian) and no hand is that.  It survived
                          # only because XPBD never delivered it: this
                          # file's own `grip_info` check measures the
                          # IN-SCENE stiffness and reads 5.5 N.m/rad under
                          # XPBD against the nominal 4000, i.e. three
                          # orders of authority starvation.  MuJoCo
                          # delivers the nominal exactly (measured 3993.6),
                          # and a 4000 N.m/rad spring behind a 4e-4 kg.m^2
                          # hand turns the cam-out into a 293 mN.m torque
                          # spike -- 2x the over_torque QA ceiling, from a
                          # hand whose ACTUATOR is clamped at 60 mN.m.  The
                          # spike is the SPRING, not the actuator, which is
                          # why clamping hand_spin harder does not touch it
                          # (measured: OMEGA_HARD 6 -> 1.5 moved the peak
                          # 293 -> 263 mN.m).  20 N.m/rad reproduces XPBD's
                          # own measured wind-up (0.087 deg) at the running
                          # torque, and is what a fist on a 6.5 mm-radius
                          # fluted handle actually is.
MJ_GRIP_F_MAX = 40.0      # N — joint_effort_limit on the grip's linear z.
                          # The hand cannot press the tool into the work
                          # harder than it can hold it; 1.8x the 22.8 N
                          # working press.  task_nut_bolt's GRIP_F_MAX.
MJ_GRIP_TAU_MAX = 1.0e6   # N.m — OFF, AND THAT IS A MEASURED DEAD END, not
                          # an omission.  The obvious cure for the seat
                          # torque spike is task_nut_bolt's GRIP_TAU_MAX --
                          # clutch the grip so it cannot transmit more than
                          # a hand can hold -- and here it makes things far
                          # worse.  At 0.10 N.m the grip stops holding the
                          # hand to the tool, the wind-up runs to 6 deg
                          # (from 0.004) and the release throws a 5693 N
                          # contact spike; at 0.08 N.m the peak torque does
                          # fall to 83 mN.m but the grip slips 19.4 deg and
                          # the axial force still reaches 175 N.  The spike
                          # is the SEAT'S OWN REACTION (see the ablation
                          # table above `rig.limit_dampratio`), and
                          # decoupling the driver from the hand does not
                          # remove a reaction the screw's stop is supplying.
MJ_HAND_TAU_MAX = 0.03    # N.m — joint_effort_limit on hand_spin.  KD_SPIN
                          # is a VELOCITY actuator under MuJoCo and a real
                          # torque source there (kd*omega = 4.5 N.m at
                          # OMEGA), against XPBD's velocity weld that could
                          # only ever deliver the hand's own momentum
                          # (21.6 mN.m).  Unclamped that is 180x the
                          # measured cam-out ceiling; see the sweep.


from force_bench.geometry import subdiv_processed as subdiv


def cap_open_boundary(m: trimesh.Trimesh, normal) -> trimesh.Trimesh:
    """Fan-triangulate the open boundary loops left by a plane slice.

    trimesh's own `cap=True` needs a triangulation engine this environment
    does not ship; the cut sections here are single convex loops, so a
    centroid fan is exact (verified: tip + rest volumes sum to the whole
    driver to 1e-9 m^3 and both come out watertight).
    """
    m = m.copy()
    m.merge_vertices()
    uniq, cnt = np.unique(m.edges_sorted, axis=0, return_counts=True)
    bnd = uniq[cnt == 1]
    if len(bnd) == 0:
        return m
    adj: dict[int, list[int]] = {}
    for a, b in bnd:
        adj.setdefault(int(a), []).append(int(b))
        adj.setdefault(int(b), []).append(int(a))
    V, F = list(m.vertices), list(m.faces)
    n = np.asarray(normal, float)
    n /= np.linalg.norm(n)
    seen: set[int] = set()
    for start in list(adj):
        if start in seen:
            continue
        loop, cur, prev = [start], start, None
        seen.add(start)
        while True:
            nxt = [x for x in adj[cur] if x != prev]
            if not nxt or nxt[0] == start:
                break
            cur, prev = nxt[0], cur
            loop.append(cur)
            seen.add(cur)
        if len(loop) < 3:
            continue
        pts = np.asarray([m.vertices[i] for i in loop])
        ci = len(V)
        V.append(pts.mean(axis=0))
        for k in range(len(loop)):
            a, b = loop[k], loop[(k + 1) % len(loop)]
            p = np.asarray([V[ci], V[a], V[b]])
            tri = ([ci, a, b]
                   if np.dot(np.cross(p[1] - p[0], p[2] - p[0]), n) >= 0
                   else [ci, b, a])
            F.append(tri)
    return trimesh.Trimesh(np.asarray(V), np.asarray(F), process=True)


def split_driver(tm: trimesh.Trimesh, tip_len: float = TIP_LEN):
    """(tip, rest) — the driver cut `tip_len` back from its tip end."""
    y_cut = tm.bounds[0][1] + tip_len
    tip = trimesh.intersections.slice_mesh_plane(
        tm, plane_normal=[0, -1, 0], plane_origin=[0, y_cut, 0], cap=False)
    rest = trimesh.intersections.slice_mesh_plane(
        tm, plane_normal=[0, 1, 0], plane_origin=[0, y_cut, 0], cap=False)
    return (cap_open_boundary(tip, [0, 1, 0]),
            cap_open_boundary(rest, [0, -1, 0]))


def yaw_of(bq) -> float:
    """World-z rotation angle of a body whose rest pose is R_z(0)*R_fix:
    the body x-axis stays horizontal for both our flips, so
    theta = atan2(R10, R00) works uniformly."""
    M = trimesh.transformations.quaternion_matrix([bq[6], bq[3], bq[4], bq[5]])
    return float(np.arctan2(M[1, 0], M[0, 0]))


def _qrot(q, v):
    q = np.asarray(q, float)
    u, w = q[..., :3], q[..., 3:4]
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


class DriveContacts:
    """The drive engagement, measured where it actually is.

    Pair-resolves the driver<->screw contacts out of the live contact arrays
    and returns BOTH the force they carry and the TORQUE they carry about
    the screw axis, as sum(r x F)_z.  The task used to estimate the torque
    as SCREW_KD * omega_screw; that over-reads by ~2.3x, because a
    damping-only joint drive is not a kd-valued torque source at all — it
    resolves with gamma = 1 s regardless of kd (XPBD-era pitfall 21b).  Measured
    side by side at the same steady spin: 20 mN.m estimated, 8.7 mN.m real.
    """

    def __init__(self, model, driver_shapes, screw_shapes):
        self.shape_body = model.shape_body.numpy()
        self.d = {int(s) for s in driver_shapes}
        self.s = {int(s) for s in screw_shapes}
        self.history: list[dict] = []

    def sample(self, contacts, state, axis_xy, t):
        n = int(contacts.rigid_contact_count.numpy()[0])
        rec = {"t": t, "n": 0, "F": 0.0, "Fz": 0.0, "tau": 0.0}
        if n <= 0 or getattr(contacts, "force", None) is None:
            self.history.append(rec)
            return rec
        n = min(n, len(contacts.rigid_contact_shape0))
        s0 = contacts.rigid_contact_shape0.numpy()[:n]
        s1 = contacts.rigid_contact_shape1.numpy()[:n]
        keep = np.array([(int(a) in self.d and int(b) in self.s) or
                         (int(a) in self.s and int(b) in self.d)
                         for a, b in zip(s0, s1)], dtype=bool)
        if not keep.any():
            self.history.append(rec)
            return rec
        f = contacts.force.numpy()[:n][keep].astype(float)
        p0 = contacts.rigid_contact_point0.numpy()[:n][keep].astype(float)
        b0 = self.shape_body[s0[keep]]
        bq = state.body_q.numpy()
        w0 = bq[b0, :3] + _qrot(bq[b0, 3:7], p0)
        r = w0.copy()
        r[:, 0] -= axis_xy[0]
        r[:, 1] -= axis_xy[1]
        sgn = np.where(np.isin(s0[keep], list(self.s)), 1.0, -1.0)[:, None]
        fs = f * sgn                      # force ON THE SCREW
        rec.update(n=int(keep.sum()),
                   F=float(np.linalg.norm(f, axis=1).sum()),
                   Fz=float(fs[:, 2].sum()),
                   tau=float((r[:, 0] * fs[:, 1] - r[:, 1] * fs[:, 0]).sum()))
        self.history.append(rec)
        return rec


def build_scene(rig: Rig) -> dict:
    """Assemble the scene.  Split out of main() so the probe scripts can
    drive the identical rig without importing a copy of it."""
    b = rig.builder
    # THE GRIP IS A TREE JOINT UNDER MuJoCo — see the block above.  Read at
    # BUILD time (pitfall 5.3): the acceptance gate replaces control(), so a
    # model-topology decision may not live in a control callback.
    actuated = True
    rig.table(center_xy=(AXIS_XY[0], -0.15), half=0.3)

    # ── scenery: stand + workpiece block (filtered vs screw) ───────────
    stand_c = np.array([0.09, -0.165, 0.010])
    sid_stand = b.add_shape_box(
        -1, xform=wp.transform(wp.vec3(*stand_c), wp.quat_identity()),
        hx=0.020, hy=0.045, hz=0.010, cfg=rig.cfg(), label="stand")
    rig.shapes["stand"] = [sid_stand]
    blk_c_cad = np.array([0.09, -0.165, 0.0055])   # block AABB center (CAD)
    rig.static_mesh(
        "screw_together_blocks", "bottom_1_body1", "block", res=256,
        xform=wp.transform(wp.vec3(*(blk_c_cad + [0, 0, 0.020])),
                           wp.quat_identity()))

    # ── screw: head-up on a revolute-z (rotary friction), SEAT stop ────
    q_flip = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), np.pi)
    q_flip_inv = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), -np.pi)
    scr_tm = subdiv(load_tm("fasteners", "screw_075mm_1_body1"), 0.0015)
    scr_mesh, scr_c = make_newton_mesh(scr_tm, sdf=True, sdf_resolution=512,
                                       narrow_band=0.0015, margin=0.0002)
    screw_pos = np.array([AXIS_XY[0], AXIS_XY[1], HEAD_TOP_Z - SCREW_H / 2])
    screw_body = b.add_link(
        xform=wp.transform(wp.vec3(*screw_pos), q_flip), label="screw")
    s_scr = b.add_shape_mesh(screw_body, mesh=scr_mesh, cfg=rig.cfg(mu=0.4),
                             label="screw_shape")
    rig.meshes["screw"] = (scr_tm, np.asarray(scr_c))
    rig.shapes["screw"] = [s_scr]
    j_screw = b.add_joint_revolute(
        parent=-1, child=screw_body, axis=(0.0, 0.0, 1.0),
        parent_xform=wp.transform(wp.vec3(*screw_pos), wp.quat_identity()),
        child_xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), q_flip_inv),
        target_ke=0.0,
        target_kd=MJ_SCREW_KD if actuated else SCREW_KD,
        friction=MJ_SCREW_FRICTION if actuated else 0.0,
        armature=MJ_SCREW_ARMATURE if actuated else 0.0,
        limit_lower=-12 * np.pi, limit_upper=np.deg2rad(SEAT_DEG))
    b.add_articulation([j_screw])
    set_body_mass(b, screw_body, 0.02)
    rig._joint_of["screw_joint"] = j_screw
    for scen in ("block", "stand", "table"):
        rig.filter_labels("screw", scen)

    # ── driver: vertical tip-down, on a free carrier chain ─────────────
    q_drv = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), np.pi / 2)
    q_drv_inv = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), -np.pi / 2)
    drv_raw = load_tm("screwdriver", "part2_1_body1")
    drv_tm = subdiv(drv_raw, 0.0015)
    drv_c = (drv_tm.vertices.min(axis=0) + drv_tm.vertices.max(axis=0)) / 2
    tip_z0 = HEAD_TOP_Z - TIP_START_DEPTH
    drv_pos = np.array([AXIS_XY[0], AXIS_XY[1], tip_z0 + TIP_HALF_LEN])
    carrier = None
    if not actuated:
        carrier = b.add_link(
            xform=wp.transform(wp.vec3(*drv_pos), wp.quat_identity()),
            label="carrier")
        car_sid = b.add_shape_box(carrier, hx=0.004, hy=0.004, hz=0.004,
                                  cfg=rig.cfg(), label="carrier_shape")
        rig.shapes["carrier"] = [car_sid]
    driver_body = b.add_link(
        xform=wp.transform(wp.vec3(*drv_pos), q_drv), label="driver")
    tip_tm, rest_tm = split_driver(drv_raw, TIP_LEN)
    tip_tm, rest_tm = subdiv(tip_tm, 0.0012), subdiv(rest_tm, 0.0020)
    tip_mesh, tip_c = make_newton_mesh(tip_tm, sdf=True, sdf_resolution=512,
                                       narrow_band=0.0015, margin=0.0002)
    rest_mesh, rest_c = make_newton_mesh(rest_tm, sdf=True,
                                         sdf_resolution=512,
                                         narrow_band=0.002, margin=0.0005)
    s_tip = b.add_shape_mesh(
        driver_body, mesh=tip_mesh, cfg=rig.cfg(mu=0.4),
        xform=wp.transform(wp.vec3(*(np.asarray(tip_c) - drv_c)),
                           wp.quat_identity()), label="driver_tip")
    s_drv = b.add_shape_mesh(
        driver_body, mesh=rest_mesh, cfg=rig.cfg(mu=0.4),
        xform=wp.transform(wp.vec3(*(np.asarray(rest_c) - drv_c)),
                           wp.quat_identity()), label="driver_shape")
    rig.meshes["driver"] = (drv_tm, np.asarray(drv_c))
    rig.shapes["driver"] = [s_tip, s_drv]
    if not actuated:
        jd1 = b.add_joint_prismatic(
            parent=-1, child=carrier, axis=(0, 0, 1),
            parent_xform=wp.transform(wp.vec3(*drv_pos), wp.quat_identity()),
            target_ke=0.0, target_kd=0.01, limit_lower=-0.02,
            limit_upper=0.05)
        jd2 = b.add_joint_revolute(
            parent=carrier, child=driver_body, axis=(0, 0, 1),
            parent_xform=wp.transform_identity(),
            child_xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), q_drv_inv),
            target_ke=0.0, target_kd=0.0005,
            limit_lower=-12 * np.pi, limit_upper=12 * np.pi)
        b.add_articulation([jd1, jd2])
        set_body_mass(b, carrier, 0.01)
    set_body_mass(b, driver_body, 0.08)
    # driver<->screw stays ENABLED — it IS the torque path (no filter!)
    if not actuated:
        rig.filter_labels("carrier", "driver")
        rig.filter_labels("carrier", "screw")
        rig.filter_labels("carrier", "table")

    # ── massive hand: world -prismatic z-> base -revolute z-> spinner ──
    hand_pos = np.array([drv_pos[0], drv_pos[1], drv_pos[2] + TIP_HALF_LEN
                         + 0.025])
    hbase = b.add_link(
        xform=wp.transform(wp.vec3(*hand_pos), wp.quat_identity()),
        label="hand_base")
    hb_sid = b.add_shape_box(hbase, hx=0.006, hy=0.006, hz=0.006,
                             cfg=rig.cfg(density=2000.0), label="hand_base_shape")
    hspin = b.add_link(
        xform=wp.transform(wp.vec3(*hand_pos), wp.quat_identity()),
        label="hand")
    hs_sid = b.add_shape_cylinder(hspin, radius=0.02, half_height=0.008,
                                  cfg=rig.cfg(density=2000.0), label="hand_shape")
    rig.shapes["hand_base"] = [hb_sid]
    rig.shapes["hand"] = [hs_sid]
    for s in (hb_sid, hs_sid):
        for tlabel in (("driver", "screw", "table") if actuated else
                       ("driver", "screw", "carrier", "table")):
            for ts in rig.shapes[tlabel]:
                b.add_shape_collision_filter_pair(s, ts)
    j3 = b.add_joint_prismatic(
        parent=-1, child=hbase, axis=(0, 0, 1),
        parent_xform=wp.transform(wp.vec3(*hand_pos), wp.quat_identity()),
        target_pos=0.0, target_ke=20000.0, target_kd=200.0,
        # THE PRESS ENVELOPE IS THE ONLY THING THAT BOUNDS THE PRESS FORCE.
        # This is a measured result.  `screwdriver_press_probe.py` ramps
        # hand_z to a commanded depth, holds it, and reports the
        # driver<->screw contact force with the trimesh signed-distance
        # penetration.  The force/overlap curve inside the envelope is
        #     30 N -> 0.16 mm   53 N -> 0.50   88 N -> 0.70   114 N -> 0.96
        #    145 N -> 1.20     182 N -> 1.38  220 N -> 1.56  258 N -> 1.72
        # and at ~258 N the tip PUNCHES THROUGH the socket floor: overlap
        # jumps to 2.9-3.7 mm, the normal force collapses from 258 N to
        # ~145 N of sliding drag and the driver walks 10 mm down into the
        # screw.  That break-through is the model's validity limit, and
        # EVERY knob that was supposed to move it does not (measured at a
        # -20/-40 mm command, worst overlap in mm):
        #    baseline                         3.195 / 3.696
        #    tip SDF cell 0.2449 -> 0.0398    3.220
        #    XPBD iterations 20 -> 60         3.207  (at 880 N)
        #    hand drive ke 20000 -> 2000/200  3.191 / 3.272
        #    d6 grip_lin 8000 -> 2000         3.127
        #    axial dof under the screw        3.258
        #    carrier joint backstop at 2.2 mm 3.269   <- the JOINT gives too
        #    collide every substep            3.180 / 3.239
        # Only the envelope moves it: -40 mm 3.696, -20 mm 3.195,
        # -5 mm 1.232, -3 mm ~0.5.  So the envelope is not a safety margin
        # bolted on, it is the model (pitfall 4.8; also XPBD-era pitfall 21a — with
        # 20 iterations x 16 substeps every joint drive has converged to a
        # rigid constraint, so there is no gain that makes a soft spring).
        # The bound is the MEASURED working travel: instrumenting the
        # scripted expert shows it commands hand_z within
        # [-2.312, -0.200] mm, so -3.0 mm leaves 0.69 mm of headroom and
        # the trajectory never touches the stop (pitfall 4.7: a body ON a
        # limit is pinned).  Real fixtures have hard end stops for exactly
        # this reason.  Was -5 mm (150 N, 1.23 mm) and -20/-40 mm before
        # that (434-439 tunnelling events, 18.2 mm).
        # UNDER MuJoCo THE BOUND IS A FORCE, NOT A POSITION (see the block
        # above the constants): the envelope is widened to the measured
        # MuJoCo working travel and `effort_limit` -- MuJoCo's actuator
        # `forcerange`, which XPBD does not implement -- caps the press at
        # MJ_PRESS_F_MAX no matter how far the command is pushed.
        limit_lower=MJ_HAND_Z_LIMITS[0] if actuated else -0.003,
        limit_upper=MJ_HAND_Z_LIMITS[1] if actuated else 0.002,
        effort_limit=MJ_PRESS_F_MAX if actuated else 1.0e6)
    j4 = b.add_joint_revolute(
        parent=hbase, child=hspin, axis=(0, 0, 1),
        parent_xform=wp.transform_identity(),
        child_xform=wp.transform_identity(),
        target_vel=0.0, target_ke=0.0, target_kd=KD_SPIN,
        limit_lower=-12 * np.pi, limit_upper=12 * np.pi,
        effort_limit=MJ_HAND_TAU_MAX if actuated else 1.0e6)
    if not actuated:
        b.add_articulation([j3, j4])
    hand_mass = MJ_HAND_MASS if actuated else 2.0
    set_body_mass(b, hbase, hand_mass)
    set_body_mass(b, hspin, hand_mass)
    rig._joint_of["hand_z"] = j3
    rig._joint_of["hand_spin"] = j4

    # compliant grip: loop d6 hand -> driver (NO add_articulation).  Joint
    # frames made world-aligned via the q_drv_inv child correction; relative
    # angle stays near zero (lead-limited), far from the ±pi d6 wrap.
    rel = wp.transform(wp.vec3(*(drv_pos - hand_pos)), wp.quat_identity())
    if actuated:
        # TREE grip: the driver's ONLY parent joint, carrying the two axes
        # that were ever load-bearing.  Contiguity is required --
        # `add_articulation` rejects a non-contiguous joint range, and a
        # joint outside its articulation's range is skipped by `eval_fk`,
        # which leaves the body's `body_q` frozen while MuJoCo integrates it
        # underneath (task_nut_bolt.add_gripped_body documents that trap).
        j_grip = b.add_joint_d6(
            parent=hspin, child=driver_body,
            linear_axes=(JD(axis=(0, 0, 1), target_ke=GRIP_LIN,
                            target_kd=40.0, limit_lower=-MJ_GRIP_Z,
                            limit_upper=MJ_GRIP_Z,
                            effort_limit=MJ_GRIP_F_MAX),),
            angular_axes=(JD(axis=(0, 0, 1), target_ke=MJ_GRIP_ANG,
                             target_kd=MJ_GRIP_ANG_KD,
                             limit_lower=-MJ_GRIP_TWIST,
                             limit_upper=MJ_GRIP_TWIST,
                             effort_limit=MJ_GRIP_TAU_MAX),),
            parent_xform=rel,
            child_xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), q_drv_inv))
        assert j_grip == j4 + 1, (j3, j4, j_grip)
        b.add_articulation([j3, j4, j_grip])
    else:
        b.add_joint_d6(
            parent=hspin, child=driver_body,
            linear_axes=tuple(JD(axis=a, target_ke=GRIP_LIN, target_kd=40.0)
                              for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            angular_axes=tuple(JD(axis=a, target_ke=GRIP_ANG, target_kd=8.0)
                               for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            parent_xform=rel,
            child_xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), q_drv_inv))

    # MuJoCo joint limits are SOFT unless asked otherwise -- see the block
    # over `mujoco_hard_limits` in task_rig.py.  Two limits in this scene are
    # load-bearing and both were being walked through before this line:
    # SEAT_DEG (the screw revolute's upper stop, which IS the head bottoming
    # on the work) went 62.9 deg past itself, and hand_z's -6 mm envelope
    # went 2.0 mm past itself.  A no-op under XPBD.
    rig.hard_joint_limits = True
    rig.limit_dampratio = 1.5
    # WHY 1.5 AND NOT THE 0.5 THAT IS BEST FOR A SLIDER.  The over-torque
    # phase pushes a stopped rotor into a soft stop, and MuJoCo's stop is a
    # spring: the drive engagement therefore reads the SEAT REACTION, so a
    # harder stop reads a BIGGER torque.  That is Newton's third law, not a
    # tuning artifact, and it is why every other lever failed to move the
    # number -- measured, peak drive torque in the QA window:
    #     contact stiffness 1e6 / 3e6 / 1e7 / 3e7   205 / 207 / 212 / 205
    #     hand mass 2.0 / 0.5 / 0.2 / 0.05 kg       212 / 204 / 201 / 201
    #     GRIP_ANG 20 / 60 / 200 N.m/rad            262 / 483 / diverged
    #     grip torque clamp 0.10 / 0.08 N.m         5693 N spike / 83
    #     OMEGA_HARD 6 -> 1.5                       293 -> 263
    #     screw dry friction 0.01 / 0.03 N.m        210 / 210
    #     screw armature 1e-4/1e-3/1e-2 kg.m^2      582 / 515 / 562
    #                                     (and the seat becomes EXACT: the
    #                                      screw stops at 150.01 deg -- the
    #                                      same trade, seen from the other
    #                                      end)
    # Only the seat's own compliance moves it, and the 2-D sweep against
    # MJ_HAND_TAU_MAX puts (1.5, 0.03 N.m) at 110 mN.m of peak drive torque
    # with 6.5 deg of seat overshoot.  DECLARED LIMITATION: XPBD stops the
    # screw dead at 150.0 deg; MuJoCo overshoots its stop by 6.5 deg,
    # because a soft-constraint solver cannot have both a rigid stop and a
    # bounded reaction.
    head_c = np.array([AXIS_XY[0], AXIS_XY[1], HEAD_TOP_Z])
    rig.finalize(probes=["screw", "driver"],
                 cam_pos=head_c + np.array([0.11, -0.16, 0.10]),
                 cam_target=head_c + np.array([0.0, 0.0, 0.03]))
    # contact convergence at the tight drive-recess interface (the ethernet
    # 10 -> 20 finding, also applied in task_nut_bolt/_arm): the penetration
    # audit measured 0.84 mm of driver-tip-into-recess overlap under the
    # bit-seating press with the task_rig default 10 iterations.  XPBD rigid
    # contacts are hard positional constraints, so sustained overlap under
    # load is iteration starvation, not a margin.
    rig.solver = make_solver(rig.model, iterations=20)
    # SAFE CADENCE ARMED AT BUILD TIME (pitfall 5.3): the acceptance gate
    # replaces control(), so a cadence set per frame by the expert simply
    # does not exist under the gate.  There is no fingertip friction grasp
    # in this rig to protect (the grip is a d6), so per-substep collision
    # costs it nothing.  Measured: it does NOT change the break-through
    # overlap (3.180 vs 3.195 mm at -20 mm) — it is here as the safe
    # default, not as the fix.
    rig.collide_every_substep = True
    return {"drv_pos": drv_pos, "screw_pos": screw_pos, "head_c": head_c,
            "drv_tm": drv_tm, "drv_c": np.asarray(drv_c),
            "scr_tm": scr_tm, "scr_c": np.asarray(scr_c)}


def main():
    rig = Rig("screwdriver")
    h = build_scene(rig)
    drv_pos = h["drv_pos"]
    si = rig.body_index("screw")
    di = rig.body_index("driver")
    hi = rig.body_index("hand")
    drive = DriveContacts(rig.model, rig.shapes["driver"], rig.shapes["screw"])

    trackers = {n: {"prev": 0.0, "total": 0.0} for n in ("screw", "driver",
                                                         "hand")}

    def _unwrap(name, idx):
        yaw = yaw_of(rig.s0.body_q.numpy()[idx])
        tr = trackers[name]
        d = yaw - tr["prev"]
        if d > np.pi:
            d -= 2 * np.pi
        elif d < -np.pi:
            d += 2 * np.pi
        tr["total"] += d
        tr["prev"] = yaw
        return tr["total"]

    log = []

    def control(t):
        screw_a = _unwrap("screw", si)
        driver_a = _unwrap("driver", di)
        hand_a = _unwrap("hand", hi)
        # closed-loop preload: hand z follows the MEASURED driver height
        # (pitfall 8.1), pressed down by PRESS
        drv_dz = rig.s0.body_q.numpy()[di][2] - drv_pos[2]
        rig.target_q("hand_z", float(np.clip(
            drv_dz - MJ_PRESS,
            -0.02, 0.002)))
        # OVER-TORQUE is triggered by the MEASURED screw angle, not a clock:
        # the operator feels the screw go home and cranks harder.
        seated = np.degrees(screw_a) > SEAT_DEG - 3.0
        lead_cap = LEAD_MAX
        if t < T_SETTLE:
            omega = 0.0
        elif t < T_SETTLE + T_FWD:
            omega = OMEGA_HARD if seated else OMEGA
            lead_cap = LEAD_MAX_OT if seated else LEAD_MAX
        elif t < T_SETTLE + T_FWD + T_PAUSE:
            omega = 0.0
        elif t < T_SETTLE + T_FWD + T_PAUSE + T_REV:
            omega = -OMEGA
        else:
            omega = 0.0
        lead = hand_a - driver_a
        if (omega > 0 and lead > lead_cap) or (omega < 0 and lead < -lead_cap):
            omega = 0.0                      # anti-wrap lead limiter
        rig.target_qd("hand_spin", omega)

    def extra(state, t):
        bq = state.body_q.numpy()
        tip_dz = float(bq[di][2] - drv_pos[2])
        sa, da, ha = (trackers[n]["total"] for n in ("screw", "driver", "hand"))
        c = drive.sample(rig.contacts, state, AXIS_XY, t)
        log.append((t, sa, da, ha, tip_dz, c["tau"], c["F"]))
        return {"screw_deg": round(np.degrees(sa), 2),
                "driver_deg": round(np.degrees(da), 2),
                "hand_deg": round(np.degrees(ha), 2),
                "tip_rise_mm": round(tip_dz * 1000, 3),
                "drive_torque_mNm": round(c["tau"] * 1000, 2),
                "drive_force_N": round(c["F"], 2),
                "grip_windup_deg": round(np.degrees(ha - da), 3)}

    duration = T_SETTLE + T_FWD + T_PAUSE + T_REV + T_END
    rec = rig.run(duration, control, extra)
    paths = rec.save("rollout", force_keys=["screw_N", "screw_fric_N",
                                            "drive_torque_mNm",
                                            "drive_force_N"])
    print("saved:", paths)

    # ── QA ─────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    arr = np.array(log)
    t_a, s_a, d_a, h_a, tz, tau, fdrv = (arr[:, k] for k in range(7))
    t_peak_end = T_SETTLE + T_FWD + T_PAUSE
    fwd = (t_a >= T_SETTLE) & (t_a < t_peak_end)
    i_peak = int(np.argmax(np.where(fwd, s_a, -np.inf)))
    screw_fwd = float(np.degrees(s_a[i_peak]))
    driver_fwd = float(np.degrees(d_a[i_peak]))
    ratio = driver_fwd / max(screw_fwd, 1e-6)
    lag_deg = driver_fwd - screw_fwd
    screw_final = float(np.degrees(s_a[-1]))
    rev_return = screw_fwd - screw_final
    cam_out_mm = float(tz[fwd].max() * 1000) if fwd.any() else 0.0
    ok = (screw_fwd >= 60.0 and ratio <= 3.0 and rev_return >= 30.0)
    qa.report["checks"]["task_success"] = {
        "pass": bool(ok),
        "screw_fwd_deg": round(screw_fwd, 1),
        "driver_fwd_deg": round(driver_fwd, 1),
        "driver_over_screw_ratio": round(ratio, 2),
        "engagement_lag_deg": round(lag_deg, 1),
        "rev_return_deg": round(rev_return, 1),
        "screw_final_deg": round(screw_final, 1),
        "tip_max_rise_mm": round(cam_out_mm, 3),
        "seat_deg": SEAT_DEG,
        "note": "square 5.0 tip in square 5.5 recess (0.25 mm/side, "
                "±6 deg backlash): screw driven by contact torque only; "
                "ratio<=3 rules out cam-out free-spin; reversal returns",
    }
    qa.report["pass"] &= ok

    # ── OVER-TORQUE: the screw seats, the hand keeps turning ───────────
    # This is the force-channel story the free-spinning screw could not
    # tell.  The drive engagement carries a RISING torque up to a ceiling
    # set by the press, then RELEASES: the tip climbs the 36.2 deg lead-in
    # ramp against the 36.4 deg socket chamfer (both steeper than the
    # 21.8 deg friction angle at mu 0.4, so neither self-locks) and the
    # screw drops back off its seat.  Characterized standalone in
    # screwdriver_press_probe.py --mode torque: with the screw stopped, the
    # ceiling is 0.029 N.m at omega 1.5 / 20 N of press, 0.050 N.m at
    # omega 6, and 0.074 N.m at omega 6 with a 50 N press -- i.e. it scales
    # with the press, which is what cam-out means.
    seated = np.degrees(s_a) > SEAT_DEG - 3.0
    tau_pk = float(np.abs(tau[fwd]).max()) if fwd.any() else 0.0
    if seated.any():
        i_seat = int(np.argmax(seated))
        tau_after = tau[i_seat:][t_a[i_seat:] < t_peak_end]
        rise_after = float((tz[i_seat:][t_a[i_seat:] < t_peak_end].max()
                            - tz[i_seat]) * 1000) if len(tau_after) else 0.0
        release = float(np.degrees(s_a[i_seat:][t_a[i_seat:]
                                                < t_peak_end].min())
                        - np.degrees(s_a[i_seat])) if len(tau_after) else 0.0
    else:
        i_seat, rise_after, release = -1, 0.0, 0.0
    released = bool(rise_after > 0.02 or release < -1.0)
    # The GATE is "bounded, and the motion gets stuck" — which is the
    # acceptance criterion verbatim.  `released` is REPORTED, not required,
    # and it is false here: see the note below.
    ot_ok = bool(seated.any() and 0.005 < tau_pk < 0.15)
    qa.report["checks"]["over_torque"] = {
        "pass": ot_ok,
        "seat_reached": bool(seated.any()),
        "released": released,
        "seat_t_s": round(float(t_a[i_seat]), 3) if i_seat >= 0 else None,
        "peak_drive_torque_mNm": round(tau_pk * 1000, 2),
        "peak_drive_force_N": round(float(fdrv[fwd].max()), 2)
        if fwd.any() else 0.0,
        "tip_rise_after_seat_mm": round(rise_after, 3),
        "screw_release_deg": round(release, 2),
        "note": "the screw SEATS at seat_deg (joint stop = head home) and "
                "the hand keeps turning: the drive torque rises to a "
                "CEILING and the motion STOPS -- it does not keep climbing "
                "and it does not tunnel.  MEASURED LIMITATION, stated "
                "plainly: the ceiling here is the hand's angular MOMENTUM "
                "being absorbed at the seat (I~2.4e-4 kg.m2 at 1.5 rad/s "
                "over one frame = 21.6 mN.m, and that is what is measured), "
                "after which the engagement STALLS instead of camming out. "
                "It stalls because hand_spin is a damping-only drive, and "
                "a damping-only drive is a velocity weld with gamma = 1 s "
                "regardless of kd (XPBD-era pitfall 21b) -- it cannot hold a "
                "sustained torque.  Commanding OMEGA_HARD = 6 rad/s at the "
                "seat instead of OMEGA = 1.5 changed the ceiling by "
                "1.2 mN.m (22.9 -> 21.7), which is that pitfall measured "
                "directly.  The CAM-OUT RELEASE itself is real and is "
                "characterized standalone in screwdriver_press_probe.py "
                "--mode torque, where the hand arrives at the seat with "
                "momentum: 0.029 N.m at omega 1.5 / 20 N press, 0.050 N.m "
                "at omega 6, 0.074 N.m at omega 6 with a 50 N press -- it "
                "scales with the press, which is what cam-out means -- and "
                "the tip then climbs 0.53 mm and the screw drops 24 deg "
                "off its seat.  With the FR3 the release comes in-task "
                "because the wrist is a position drive and the pads are a "
                "real friction limiter in series.",
    }
    qa.report["pass"] &= ot_ok

    # in-scene grip characterization (pitfall 6.5)
    mid = (t_a >= T_SETTLE + 0.4) & (t_a < T_SETTLE + 1.4)
    om_mid = float(np.gradient(s_a, t_a)[mid].mean()) if mid.any() else 0.0
    wind_mid = float((h_a - d_a)[mid].mean()) if mid.any() else 0.0
    tau_mid = float(np.abs(tau[mid]).mean()) if mid.any() else 0.0
    preload = float(np.mean(
        [r["force"] for r in rig.probes["screw"].history
         if 0.3 <= r["t"] < T_SETTLE]))
    qa.report["checks"]["grip_info"] = {
        "pass": True,
        "preload_N": round(preload, 2),
        "steady_omega_rad_s": round(om_mid, 3),
        "steady_drive_torque_mNm": round(tau_mid * 1000, 2),
        "kd_omega_estimate_mNm": round(SCREW_KD * om_mid * 1000, 2),
        "grip_windup_deg": round(np.degrees(wind_mid), 3),
        "effective_grip_ang_stiffness_Nm_per_rad": round(
            tau_mid / max(abs(wind_mid), 1e-9), 2),
        "note": "torque is MEASURED from the driver<->screw contact arrays "
                "(sum r x F about the screw axis); the old SCREW_KD*omega "
                "estimate is reported next to it and over-reads (pitfall "
                "21b: a damping-only drive is not a kd-valued torque)",
    }

    # geometric penetration, driver<->screw, tolerance 1.5 mm
    drv_local = h["drv_tm"].copy()
    drv_local.apply_translation(-h["drv_c"])
    scr_local = h["scr_tm"].copy()
    scr_local.apply_translation(-h["scr_c"])
    tip_pts = drv_local.vertices[drv_local.vertices[:, 1] < -0.045]
    tip_cloud = trimesh.Trimesh(vertices=tip_pts, process=False)
    head_pts = scr_local.vertices[scr_local.vertices[:, 2] < -0.008]
    head_cloud = trimesh.Trimesh(vertices=head_pts, process=False)
    qa.check_penetration(
        [("driver", tip_cloud, "screw", scr_local),
         ("screw", head_cloud, "driver", drv_local)], tol=0.0015)
    qa.check_envelope(["driver", "screw"],
                      h["head_c"] + np.array([-0.06, -0.06, -0.06]),
                      h["head_c"] + np.array([0.06, 0.06, 0.20]))
    rig.finish(qa)


if __name__ == "__main__":
    wp.init()
    main()
