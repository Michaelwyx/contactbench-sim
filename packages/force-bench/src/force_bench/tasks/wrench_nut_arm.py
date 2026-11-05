"""Task: FRANKA tightens a nut onto a bolt WITH AN OPEN-END WRENCH.

The suite already turns this nut with the fingertips (task_nut_bolt_arm).
That task is about a FRICTION grasp on the hex flats.  This one is about a
TOOL: the pads never touch the nut at all, they hold a spanner handle
90 mm away, and every newton-metre that reaches the thread crosses two
contacts that are pure FORM CLOSURE -- jaw flat against nut flat.  The
whole tactile signature is different, and so is the failure mode: a
friction grasp slips, a wrench CAMS OFF.

    engage    the jaw slides radially onto two opposite flats
    swing     the wrist arcs 60 deg about the BOLT axis; the couple in the
              jaw turns the nut and the thread pulls it down its helix
    lift off  the jaw slides radially clear
    index     the wrist arcs back to the next flat pair
    repeat    until the nut is on its seat

60 degrees is not a tuning choice: a hex has six-fold symmetry, so an
open-end wrench can only be re-seated every 60 deg of nut rotation.  That
is why tightening a nut in a confined space is a sequence of short strokes
and re-seats, and it is the motion this task reproduces.

THE JAW MUST BE PARALLEL TO TWO FLATS BEFORE IT GOES ON, EVERY TIME.  An
open-end wrench that arrives even a few degrees off does not slide onto
the flats -- it lands on two CORNERS, cams, and rounds them.  With a
16.154 mm jaw on a 15.50 mm nut the misalignment a slide-on tolerates is
about atan(0.65/17.59) = 2.1 deg, and the nut moves during every stroke by
an amount the wrench does not control (measured follow 0.31-1.00 of the
commanded 60 deg), so where the flats ARE is not predictable and has to be
MEASURED.  So:
  * `hex_flat_phase` measures the flat-normal phase off the nut MESH, by
    sweeping the support width -- a hex is narrowest across its flats, so
    the direction that minimises the projected width IS a flat normal;
  * `engage_angle` puts the jaw's own opening normal on the nearest of the
    six flat normals of the nut's MEASURED yaw, and it is re-evaluated
    continuously through the approach, not once at the start of it, so a
    nut that settles while the jaw is closing is still followed;
  * `align_err_deg` is logged every frame and gated: the jaw has to be
    within ALIGN_TOL of a flat pair at the instant the stroke begins, or
    the run says so instead of quietly rounding the nut.

Run:
    python -m force_bench.tasks.wrench_nut_arm
    TB_STROKES=2 python -m force_bench.tasks.wrench_nut_arm     # smoke test

═══════════════════════════════════════════════════════════════════════
EVERY PART IS REAL CAD, INCLUDING THE TOOL
═══════════════════════════════════════════════════════════════════════
    bolt   fasteners/screw_075mm_1_body1     STATIC, shaft up
    nut    fasteners/nut_075mm_1_body1       FREE, on the thread
    wrench unidentified/part1_1_body1        FREE, held by the fingers

The wrench is the flat open-end spanner that hangs on the middle peg of
the hanging_board in task_keychain_hang -- the same body, taken off its
peg and put to work.  (`hanging_board/part1_1_body1`, the pegboard itself,
is one welded 406 cm^3 solid with the tools fused to it and cannot be
separated; `unidentified/part1_1_body1` is the spanner as its own body,
which is why that is the one used.)

MEASURED OFF THE SPANNER MESH (assembly coords, mm), by sampling the
solid on a 0.05 mm grid through its mid-thickness plane and least-squares
fitting the two inner jaw faces:

    overall                113.09 long x 30.0 wide x 7.50 thick
    volume                 10903.7 mm^3  ->  85.6 g in steel
    jaw parallel band      z 51.25 .. 58.25   (7.0 mm of flat)
    inner faces            y = -44.432 + 0.18217 z   and that +16.416
    jaw axis tilt          10.32 deg off the shank -- this is a real
                           offset open-end spanner, not a symmetric fork
    TRUE across flats      16.150 mm  (the 16.416 mm y-gap is measured
                           across y, not across the jaw's own normal)
    nut seat               x -110.771, y -26.251, z 54.750  = the centre
                           of the parallel band
    shank at the balance   11.84 mm wide, centreline y = -25.14
    balance point          45.0 mm along the jaw axis from the seat

So the tool is a 16 mm spanner and the nut is 15.50 mm across flats:
0.65 mm of clearance, which is a real if slightly loose pairing and is
exactly the kind of fit that makes an open-end wrench cam off a corner.
Nothing here is chosen -- the parts are what the library contains, and the
lost motion per stroke is MEASURED (mean_follow_per_stroke) rather than
designed away.

THE JAW IS ONLY 7.0 mm DEEP against a nut that is 17.59 mm across corners,
so the nut is never fully enclosed: half of it stands outside the mouth
and its far corner reaches into the jaw's closing taper.  That is normal
for an open-end wrench and it is the reason the load path is two SHORT
flats rather than a box.

WHY THE PADS CLOSE ACROSS THE SHANK'S WIDTH, not its faces.  The load the
grasp has to carry is TANGENTIAL: the swing pushes the shank sideways,
perpendicular to its length.  Closing the pads across the width puts that
load on a pad NORMAL (form closure) instead of on pad friction.

WHERE THE PADS TAKE IT, and the measurement that decided it.  A pinch
across an 11.8 mm shank resists PITCH (nose-down rotation about the
closing axis) only through the pressure centre shifting inside the pad's
own 17.5 mm length -- roughly N x 8.75 mm of moment capacity.  Held at the
far end of the shank, an earlier build measured the consequence exactly:
the jaw sank 8.2 mm in 9 s while the FINGERS held their height to a tenth
of a millimetre, and by the second stroke the jaw was passing under a nut
it never touched again (jaw force identically 0.000 N).  The pads
therefore take the spanner at its measured BALANCE POINT, where the pinch
carries no static moment at all -- 45 mm of lever, which is what a 113 mm
spanner has to give.
"""

from __future__ import annotations

import os

import numpy as np
import trimesh
import warp as wp

from force_bench.robot.franka import down_grasp_quat
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass
from force_bench.tasks.nut_bolt import (
    load_subdivided,
    LEAD,
    BOLT_WORLD_C,
    MATE_ZC,
    MATE_YAW_DEG,
    SDF_RES,
    SDF_NB,
    SDF_MARGIN,
    GAP,
)
from force_bench.tasks.nut_bolt_arm import PairProbe, YawTrack
from force_bench.rig import Rig, set_finger_effort_limit


def ease(u):
    u = float(np.clip(u, 0.0, 1.0))
    return 3 * u * u - 2 * u ** 3


# ── the fastener (imported thread constants; see the header) ────────────
Z_START = 0.0135          # m — nut centre, same start pose task_nut_bolt_arm
                          # uses: ~3.0 mm above the head-shoulder seat, i.e.
                          # 1.29 turns of tightening to go
BAKE_YAW_DEG = (MATE_YAW_DEG + (Z_START - MATE_ZC) / LEAD * 360.0) % 360.0
SEAT_ZC = 0.0105          # m — nut centre at the seat (mesh-measured)
NUT_AF = 0.01550          # m — across flats
NUT_AC = 0.01759          # m — across corners
NUT_H = 0.00702           # m
NUT_MU = 0.30
NUT_MASS = 0.010

# ── the wrench: unidentified/part1_1_body1 (see the header) ─────────────
WRENCH_ASSET = ("unidentified", "part1_1_body1")
WRENCH_RHO = 7850.0       # kg/m^3 — steel; 10903.7 mm^3 -> 85.6 g
WRENCH_MU = 0.35          # steel on steel, machined
WRENCH_SUBDIV = 0.0020    # m — pitfall 3.2: no face may span an SDF wall
WRENCH_SDF_RES = 768      # 0.147 mm voxels on a 113 mm part; the jaw
                          # clearance is 0.33 mm per side and an SDF that
                          # cannot resolve it cannot hold the flats

GRIP_DROP = 0.0050        # pad tip below the spanner's mid-thickness plane.
                          # The shank is 7.5 mm thick, so this puts the tip
                          # 1.25 mm under its bottom face and still 3.2 mm
                          # clear of the tabletop.
CLEAR_E = 0.020           # m — radial retreat that takes the jaw fully off
                          # the nut: the arms reach 6.8 mm past the seat
                          # toward the mouth and the nut's corner radius is
                          # 8.80 mm, so 15.6 mm clears with margin
GRIP_R = None             # m — set from the MEASURED balance point
GRIP_LOCAL = None         # m — grasp point in the jaw frame
SHANK_ANG = None          # rad — shank direction in the jaw frame

# ── the stroke ladder ───────────────────────────────────────────────────
STROKE_DEG = 60.0         # a hex indexes every 60 deg — see the header
ALIGN_RATE = np.radians(2.0)   # rad/frame cap on the re-alignment servo
TOOL_LOST_Z = 0.0060      # m — jaw-plane offset that means the spanner is
                          # out of the pinch (the servo's own authority is
                          # 6 mm, so past this it cannot be a tracking lag)
ALIGN_TOL = 1.5           # deg — how square the jaw has to be to the flats
                          # at the moment the stroke starts.  The slide-on
                          # tolerance of this pair is 2.1 deg (see the
                          # header), so the gate is inside it.
N_STROKES = int(os.environ.get("TB_STROKES", "9"))
THETA0_DEG = 90.0         # where the engage angle is chosen NEAREST to.
                          # THE HANDLE POINTS AWAY FROM THE ROBOT.  It was
                          # 300 deg (handle toward the robot) on the theory
                          # that a near grasp is an easy one, and it is the
                          # opposite: that puts the grip point 231 mm from
                          # the FR3's own base at 27 mm of height, inside
                          # the shoulder, and the IK came apart (21.1 mm of
                          # residual, a joint on its stop).  At 90 deg the
                          # same point is 443 mm out and the whole sweep
                          # tracks to 0.00 mm.
YAW_BIAS = np.pi          # rad — added to every commanded wrist yaw.  A
                          # HALF TURN IS A NO-OP FOR THE GRASP (it swaps
                          # which finger is which and leaves the closing
                          # AXIS identical) and it is worth 22.4 deg of
                          # joint-limit margin instead of 0.0: swept over
                          # the whole stroke ladder, bias 0 rides q3
                          # exactly on its limit, bias pi never comes
                          # within 22 deg of any stop.  README pitfall 20.
# EVERYTHING IS SLOWER THAN IT LOOKS IT SHOULD BE, because the tool is
# held in friction: the faster the wrist re-seats, the more the spanner
# flops in the pinch, and a flop of 9 deg is what dropped it (see
# FING_GRIP).  Rate is the cheapest thing this task can spend.
T_SETTLE, T_ENG, T_SWING, T_OFF, T_IDX, T_END = 0.8, 2.2, 3.2, 0.9, 1.8, 1.0

# ── the grasp ───────────────────────────────────────────────────────────
FINGER_KE, FINGER_KD = 8000.0, 60.0
FINGER_F_MAX = 20.0
PAD_MU = 1.4              # elastomer fingertips (Newton averages the pair)
FINGER_MASS = 0.12        # kg — README pitfall 4.5: what a fingertip pinch can
                          # TRANSMIT is set by the driven body's inertia, not
                          # by the drive stiffness.  The URDF fingertips are
                          # too light to hold a tool against a contact
                          # reaction; 0.12 kg is the value the bottle-cap and
                          # ethernet tasks measured and settled on.
FING_OPEN = 0.012
FING_TOUCH = 0.00600      # frame-0 command: pads KISSING the 11.84 mm
                          # shank.  Writing FING_GRIP straight into joint_q
                          # starts the run 1.8 mm/side interpenetrated and
                          # the first frame resolved it as a 295 N spike.
FING_GRIP = 0.00350       # 11.84 mm shank - 2 x 2.50 mm of interference,
                          # i.e. ~15 N/pad at FINGER_KE and still under the
                          # 20 N actuator limit.  1.0 mm/side (7.3 N) let
                          # the wrench slide back along its own handle the
                          # moment the jaw touched the nut: 2 x 7.3 x 0.875
                          # is 12.8 N of friction against a position-driven
                          # engage that will push as hard as it has to.
DZ_RATE = 0.00020         # m/frame — cap on the TCP height servo
# ── TOOL-POSE SERVO (README pitfall 8.1) ─────────────────────────────────
# The spanner is held in friction, so where the JAW ends up is not where
# the wrist was told to put it.  Measured on the first build with the real
# asset: the first engage seated correctly, and every one after it stopped
# 2.8 mm short -- the tool had slipped back through the pinch when the jaw
# first touched the nut -- against a flat band only 6.3 mm deep, so the nut
# was sitting at the mouth instead of in the jaw and the stroke delivered
# nothing (jaw force identically 0.000 N through strokes 1 and 2).  The
# same friction let it sink 2.2 mm in 11 s.
# So the command closes on the MEASURED jaw seat, in both axes, integral
# and hard-bounded so it can only converge.
TOOL_GAIN = 0.10          # fraction of the error per frame
TOOL_RATE = 0.00008       # m/frame
TOOL_MAX_R = 0.007        # m — radial authority
TOOL_MAX_Z = 0.006        # m — vertical authority
SEAT_TOL = 0.0004         # m — "seated" for the purposes of the servo
TIP_DROP = 0.0095         # pad tip below the TCP
GRIP_DROP = 0.0060        # pad tip below the handle's mid-plane


def measure_jaw(tm: trimesh.Trimesh) -> dict:
    """Derive the open-end jaw's own frame from the SPANNER MESH.

    Nothing here is read off a drawing: the solid is sampled on a 0.05 mm
    grid through its mid-thickness plane, the void that opens to the free
    end is found row by row, the two inner faces of the PARALLEL band are
    least-squares fitted with a shared slope, and the frame falls out of
    that fit.  Doing it by measurement is what makes the 10.32 deg jaw
    offset a fact about the part rather than a number somebody typed.

    Returns the jaw's true across-flats, its tilt off the shank, the depth
    of the parallel band, and `T` -- the 4x4 that carries the mesh into the
    jaw frame (origin = the nut seat, +x out of the mouth, +y the opening
    normal, +z the spanner's thickness).
    """
    v = np.asarray(tm.vertices)
    ax = int(np.argmin(tm.extents))                 # thickness axis
    lo, hi = np.delete([0, 1, 2], ax)               # the in-plane axes
    xm = 0.5 * (v[:, ax].min() + v[:, ax].max())
    # the jaw is at the end of the part with the smaller `hi` coordinate
    h0, h1 = v[:, hi].min(), v[:, hi].max()
    span = np.arange(v[:, lo].min() - 0.001, v[:, lo].max() + 0.001, 5e-5)
    rows = []
    for h in np.arange(h0, h0 + 0.20 * (h1 - h0), 2.5e-4):
        P = np.empty((span.size, 3))
        P[:, ax], P[:, lo], P[:, hi] = xm, span, h
        ins = tm.contains(P)
        idx = np.where(ins)[0]
        if len(idx) < 3:
            continue
        gaps, run = [], None
        for i in range(idx[0], idx[-1] + 1):
            if not ins[i]:
                run = i if run is None else run
            elif run is not None:
                gaps.append((run, i - 1))
                run = None
        if not gaps:
            continue
        g = max(gaps, key=lambda a: a[1] - a[0])
        rows.append((h, span[g[0] - 1], span[g[1] + 1]))
    rows = np.array(rows)
    op = rows[:, 2] - rows[:, 1]
    # THE PARALLEL BAND IS THE LONGEST RUN OF CONSTANT OPENING, not the
    # widest row.  "Widest" picks up a single outlier where the sampling
    # catches the mouth chamfer or a taper, and one bad row takes the fit
    # with it: measured, that returned a 0.3 mm "band" with a 13.66 mm
    # across-flats and a 35 deg tilt on a jaw that is really 16.15 mm at
    # 10.3 deg.  Parallel faces are the definition of the working part of
    # the jaw, so find them as such.
    best = (0, 0, 0)
    i = 0
    while i < len(op):
        j = i + 1
        while j < len(op) and abs(op[j] - op[j - 1]) < 5e-5:
            j += 1
        if j - i > best[0]:
            best = (j - i, i, j)
        i = j
    band = rows[best[1]:best[2]]
    n = len(band)
    A = np.stack([np.ones(2 * n), np.concatenate([band[:, 0]] * 2),
                  np.concatenate([np.zeros(n), np.ones(n)])], axis=1)
    a0, slope, sep = np.linalg.lstsq(
        A, np.concatenate([band[:, 1], band[:, 2]]), rcond=None)[0]
    af = float(sep / np.hypot(1.0, slope))          # across the NORMAL
    h_seat = float(0.5 * (band[0, 0] + band[-1, 0]))
    l_seat = float(a0 + slope * h_seat + sep / 2)

    # jaw frame: depth unit d (deeper = increasing h), normal unit nrm
    d = np.array([slope, 1.0]) / np.hypot(1.0, slope)
    nrm = np.array([1.0, -slope]) / np.hypot(1.0, slope)
    R = np.zeros((3, 3))
    R[0, lo], R[0, hi] = -d[0], -d[1]               # +x = out of the mouth
    R[1, lo], R[1, hi] = nrm[0], nrm[1]             # +y = opening normal
    R[2, ax] = 1.0                                  # +z = thickness
    if np.linalg.det(R) < 0:                        # keep it right-handed
        R[1] = -R[1]
    seat = np.zeros(3)
    seat[ax], seat[lo], seat[hi] = xm, l_seat, h_seat
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ seat
    return {"af": af, "tilt": float(np.arctan(slope)),
            "band": float(band[-1, 0] - band[0, 0]), "T": T}


def hex_flat_phase(tm: trimesh.Trimesh) -> float:
    """Angle (rad) of ONE flat NORMAL of a hex prism, in [0, pi/3).

    Found by sweeping the support width: a hex is narrowest across its
    flats, so the direction that minimises the projected width IS a flat
    normal.  Doing it by measurement rather than by assuming the mesh's own
    yaw phase means the jaw stays aligned even if the mating-yaw bake
    changes.
    """
    v = np.asarray(tm.vertices)[:, :2] - np.asarray(tm.centroid)[:2]
    th = np.linspace(0.0, np.pi / 3.0, 721, endpoint=False)
    w = [float(np.ptp(v @ np.array([np.cos(a), np.sin(a)]))) for a in th]
    return float(th[int(np.argmin(w))])


def main() -> None:
    rig = Rig("wrench_nut_arm")
    rig.add_robot(wp.transform(wp.vec3(0.0, 0.325, 0.0), wp.quat_identity()))
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        set_body_mass(rig.builder, bi, FINGER_MASS)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU
    rig.table(center_xy=(0.0, 0.16), half=0.35)

    # ── bolt: STATIC, shaft up at the world origin ──────────────────────
    bolt_tm = load_subdivided("fasteners", "screw_075mm_1_body1")
    bolt_mesh, bolt_c = make_newton_mesh(
        bolt_tm, sdf=True, sdf_resolution=SDF_RES,
        narrow_band=SDF_NB, margin=SDF_MARGIN)
    bolt_sid = rig.builder.add_shape_mesh(
        -1, mesh=bolt_mesh,
        xform=wp.transform(wp.vec3(*BOLT_WORLD_C), wp.quat_identity()),
        cfg=rig.cfg(mu=NUT_MU, gap=GAP), label="bolt")
    rig.shapes["bolt"] = [bolt_sid]
    bolt_world = bolt_tm.copy()
    bolt_world.apply_translation(BOLT_WORLD_C - np.asarray(bolt_c))
    rig.meshes["bolt"] = (bolt_world, BOLT_WORLD_C)

    # ── nut: FREE body, mating yaw baked into the mesh ──────────────────
    nut_tm = load_subdivided("fasteners", "nut_075mm_1_body1")
    nut_tm.apply_translation(-(nut_tm.bounds[0] + nut_tm.bounds[1]) / 2)
    nut_tm.apply_transform(trimesh.transformations.rotation_matrix(
        np.radians(BAKE_YAW_DEG), [0, 0, 1]))
    flat_phase = hex_flat_phase(nut_tm)
    nut_mesh, nut_c = make_newton_mesh(
        nut_tm, sdf=True, sdf_resolution=SDF_RES,
        narrow_band=SDF_NB, margin=SDF_MARGIN)
    nut_pos = np.array([0.0, 0.0, Z_START])
    nut_body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*nut_pos), wp.quat_identity()), label="nut")
    nut_sid = rig.builder.add_shape_mesh(nut_body, mesh=nut_mesh,
                                         cfg=rig.cfg(mu=NUT_MU, gap=GAP),
                                         label="nut_shape")
    rig.shapes["nut"] = [nut_sid]
    set_body_mass(rig.builder, nut_body, NUT_MASS)
    nut_world = nut_tm.copy()
    nut_world.apply_translation(nut_pos - np.asarray(nut_c))
    rig.meshes["nut"] = (nut_world, nut_pos)
    print(f"nut across flats {NUT_AF*1000:.2f} mm, flat-normal phase "
          f"{np.degrees(flat_phase):.2f} deg, bake yaw {BAKE_YAW_DEG:.1f} deg")

    # ── the wrench: the real spanner mesh, re-framed on its own jaw ─────
    # measure_jaw() derives the jaw frame from the MESH (see the header for
    # the numbers it returns), then the body is rebuilt so that
    #   origin  = the nut seat inside the jaw
    #   +x      = out of the jaw mouth (the nut enters along -x)
    #   +y      = the jaw's opening normal
    #   +z      = the spanner's thickness (up, when it lies flat)
    # so every commanded pose below is written straight in that frame.
    w_raw = load_tm(*WRENCH_ASSET)
    jaw = measure_jaw(w_raw)
    print(f"spanner jaw: across flats {jaw['af']*1000:.3f} mm on a "
          f"{NUT_AF*1000:.2f} mm nut ({(jaw['af']-NUT_AF)*1000:.2f} mm "
          f"clearance), axis tilt {np.degrees(jaw['tilt']):.2f} deg, "
          f"flat band {jaw['band']*1000:.1f} mm deep")
    assert abs(jaw["af"] - 0.01615) < 3e-4, "jaw geometry moved"
    assert jaw["af"] > NUT_AF, "the spanner does not fit this nut"

    w_tm = w_raw.copy()
    w_tm.apply_transform(jaw["T"])                  # into the jaw frame
    v, f = trimesh.remesh.subdivide_to_size(
        np.asarray(w_tm.vertices, dtype=np.float64), np.asarray(w_tm.faces),
        max_edge=WRENCH_SUBDIV)
    w_tm = trimesh.Trimesh(v, f)
    # THE BALANCE POINT (see the header): the spanner is one uniform steel
    # part, so its volume centroid IS its centre of mass, and holding it
    # there means the pinch carries no static pitching moment.
    com = np.asarray(w_tm.center_mass)
    global GRIP_R, GRIP_LOCAL, SHANK_ANG
    GRIP_LOCAL = np.array([com[0], com[1], 0.0])
    GRIP_R = float(np.linalg.norm(GRIP_LOCAL[:2]))
    SHANK_ANG = float(np.pi + jaw["tilt"])          # shank dir in the frame
    w_mass = float(w_tm.volume * WRENCH_RHO)
    print(f"spanner: {w_mass*1000:.1f} g, balance point at "
          f"({com[0]*1000:.1f}, {com[1]*1000:.1f}) mm -> lever "
          f"{GRIP_R*1000:.1f} mm, shank at "
          f"{np.degrees(SHANK_ANG):.1f} deg in the jaw frame")

    w_mesh, w_c = make_newton_mesh(w_tm, sdf=True,
                                   sdf_resolution=WRENCH_SDF_RES,
                                   narrow_band=SDF_NB, margin=SDF_MARGIN)
    th0 = np.radians(THETA0_DEG)
    R0 = np.array([[np.cos(th0), -np.sin(th0), 0.0],
                   [np.sin(th0), np.cos(th0), 0.0], [0.0, 0.0, 1.0]])
    # body ORIGIN placed so the JAW SEAT lands `CLEAR_E` back from the nut
    w_seat = np.array([-CLEAR_E * np.cos(th0), -CLEAR_E * np.sin(th0),
                       Z_START])
    w_pos = w_seat + R0 @ np.asarray(w_c)
    w_body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*w_pos),
                           wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0),
                                                   float(th0))),
        label="wrench")
    w_sid = rig.builder.add_shape_mesh(
        w_body, mesh=w_mesh, cfg=rig.cfg(mu=WRENCH_MU, gap=GAP),
        label="wrench_shape")
    rig.shapes["wrench"] = [w_sid]
    set_body_mass(rig.builder, w_body, w_mass)
    w_world = w_tm.copy()
    w_world.apply_translation(-np.asarray(w_c))
    rig.meshes["wrench"] = (w_world, w_pos)
    # the mesh centre, in the JAW frame, is the offset between the body
    # origin and the seat every commanded pose is written about
    seat_off = np.asarray(w_c)

    # the spanner never touches the table or the bolt HEAD; it is a tool in
    # the air, held by the robot, and the only contacts that should carry
    # anything are jaw<->nut and pad<->shank
    rig.filter_labels("wrench", "table")

    # ── arm start: already holding the shank, jaw clear of the nut ──────
    def tcp_for(theta, e, z_head=Z_START):
        """TCP that puts the pads on the shank at wrench angle `theta`
        with the jaw retreated `e` radially and the jaw plane at `z_head`.

        THE HEAD HAS TO FOLLOW THE NUT DOWN.  The jaw is 7.0 mm deep and
        the nut is 7.02 mm tall, so a fixed height loses the flats after a
        couple of millimetres of tightening -- which is the whole travel
        this task has.  A hand does the same thing without thinking."""
        c, sn = np.cos(theta), np.sin(theta)
        R = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]])
        p = R @ (GRIP_LOCAL + np.array([-e, 0.0, 0.0]))
        return np.array([p[0], p[1],
                         z_head - GRIP_DROP + TIP_DROP])

    def wrist_yaw(theta):
        """Wrist yaw that closes the pads ACROSS the shank.

        down_grasp_quat(y) closes along world y+90 deg; the shank lies at
        SHANK_ANG in the jaw frame, i.e. theta + SHANK_ANG in the world, so
        the closing axis has to be theta + SHANK_ANG + 90 -- which makes
        the commanded yaw theta + SHANK_ANG."""
        return theta + SHANK_ANG

    tcp0 = tcp_for(th0, CLEAR_E)
    rig.solve_arm_start(tcp0, down_grasp_quat(wrist_yaw(th0)))
    rig.builder.joint_q[7:9] = [FING_TOUCH, FING_TOUCH]
    rig.builder.joint_target_q[7:9] = [FING_TOUCH, FING_TOUCH]

    cam_t = np.array([0.0, 0.0, Z_START])
    rig.finalize(probes=["nut", "wrench",
                         "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"],
                 cam_pos=cam_t + np.array([0.16, -0.20, 0.14]),
                 cam_target=cam_t)
    rig.probes["padL"] = rig.probes.pop("fr3/fr3_leftfinger")
    rig.probes["padR"] = rig.probes.pop("fr3/fr3_rightfinger")
    # THE MEASUREMENT THIS TASK EXISTS FOR: what crosses the JAW.  A probe
    # on the wrench body sums the pads' own grip too, and the pads carry
    # 8-10 N each — an order above the jaw load — so the tool-vs-work
    # contact has to be pair-resolved or it is invisible.
    rig.probes["jaw"] = PairProbe(rig.model, "nut",
                                  counterpart_shapes=["wrench_shape"])
    rig.probes["thread"] = PairProbe(rig.model, "nut",
                                     counterpart_shapes=["bolt"])
    ni = rig.body_index("nut")
    wi = rig.body_index("wrench")

    nut_yaw = YawTrack()
    wr_yaw = YawTrack()
    st = {"ph": "settle", "t0": 0.0, "dur": T_SETTLE, "n": 0,
          "theta": th0, "e": CLEAR_E, "from": th0, "to": th0,
          "z": Z_START, "br": 0.0, "bz": 0.0, "e_meas": CLEAR_E,
          "z_meas": 0.0, "align": 0.0, "align_at_engage": 0.0,
          "lost": None, "seated": None, "log": []}
    st["align_at_engage"] = None
    t_end = (T_SETTLE + N_STROKES * (T_ENG + T_SWING + T_OFF + T_IDX)
             + T_END)

    def engage_angle(nu, prefer):
        """Wrench angle that lines the jaw up with a flat pair.

        The jaw's opening normal is the wrench frame's own +y, which sits
        at theta + 90 deg in the world; the nut's six flat normals sit at
        flat_phase + nu + k*60 deg.  Pick the k that lands nearest
        `prefer` -- so the sweep stays where the arm can reach when
        `prefer` is the parking angle, and so the servo cannot jump a whole
        flat when `prefer` is the current angle.
        """
        base = flat_phase + nu - np.pi / 2.0
        k = np.round((prefer - base) / (np.pi / 3.0))
        return base + k * np.pi / 3.0

    def align_err(theta, nu):
        """Signed angle from the jaw's normal to the NEAREST flat normal,
        wrapped into +-30 deg.  Zero means the jaw is square to two flats."""
        e = (theta + np.pi / 2.0) - (flat_phase + nu)
        return (e + np.pi / 6.0) % (np.pi / 3.0) - np.pi / 6.0

    def jaw_seat(bq):
        """World position of the jaw's nut seat, from the MEASURED pose."""
        q = bq[wi]
        M = trimesh.transformations.quaternion_matrix(
            [q[6], q[3], q[4], q[5]])[:3, :3]
        return np.asarray(q[:3]) - M @ seat_off, M

    def control(t):
        bq = rig.s0.body_q.numpy()
        nu = nut_yaw.update(bq[ni])
        wr_yaw.update(bq[wi])
        ph, u = st["ph"], min(1.0, (t - st["t0"]) / st["dur"])

        # ── tool-pose servo: where is the JAW, not where was it sent ────
        seat_w, _ = jaw_seat(bq)
        nut_w = np.asarray(bq[ni, :3])
        rad = np.array([np.cos(st["theta"]), np.sin(st["theta"])])
        # measured radial retreat of the seat from the nut axis (+ = out)
        e_meas = float(-(seat_w[:2] - nut_w[:2]) @ rad)
        st["e_meas"], st["z_meas"] = e_meas, float(seat_w[2] - nut_w[2])
        if ph in ("engage", "swing"):
            st["br"] = float(np.clip(
                st["br"] + np.clip(TOOL_GAIN * (e_meas - st["e"]),
                                   -TOOL_RATE, TOOL_RATE), 0.0, TOOL_MAX_R))
            st["bz"] = float(np.clip(
                st["bz"] - np.clip(TOOL_GAIN * st["z_meas"],
                                   -TOOL_RATE, TOOL_RATE),
                -TOOL_MAX_Z, TOOL_MAX_Z))

        st["align"] = float(np.degrees(align_err(st["theta"], nu)))

        if ph == "settle":
            if u >= 1.0:
                st.update(ph="engage", t0=t, dur=T_ENG, ue=0.0)
                st["theta"] = engage_angle(nu, th0)
                st["align_at_engage"] = None
        elif ph == "engage":
            # RE-ALIGN CONTINUOUSLY WHILE THE JAW GOES ON.  The nut settles
            # as the jaw touches it, and a wrench that was square when the
            # approach started can be a degree or two out by the time the
            # flats meet -- which on this pair is most of the 2.1 deg
            # slide-on tolerance.  `prefer` is the CURRENT angle so the
            # servo tracks the same flat pair it started on and can never
            # jump 60 deg mid-approach; the step is rate-capped.
            tgt = engage_angle(nu, st["theta"])
            st["theta"] += float(np.clip(tgt - st["theta"],
                                         -ALIGN_RATE, ALIGN_RATE))
            # THE JAW WAITS UNTIL IT IS SQUARE.  The radial approach only
            # advances while the jaw is inside half the slide-on tolerance
            # of a flat pair; the alignment servo is rate-capped, so an
            # approach that started before the servo had converged used to
            # meet the nut 3 deg off (measured 2026-09-17: first contact at
            # -3.16 deg against a 1.5 deg gate).  The wait is bounded by a
            # second engage duration so the ladder cannot hang.
            st["align"] = float(np.degrees(align_err(st["theta"], nu)))
            ur = (t - st["t0"]) / st["dur"]
            if abs(st["align"]) <= 0.5 * ALIGN_TOL or ur >= 2.0:
                st["ue"] = min(1.0, st.get("ue", 0.0)
                               + 1.0 / (st["dur"] * rig.fps))
            u = st.get("ue", 0.0)
            st["e"] = CLEAR_E * (1.0 - ease(u))
            # LATCH THE ALIGNMENT AT FIRST CONTACT, which is the instant
            # that decides whether the jaw slides onto two flats or lands
            # on two corners.  Reading it at the END of the approach reads
            # something else: by then the jaw has been pushing the nut, and
            # a nut that CAMMED round because it was met off-square shows
            # up as a large error caused by the very event the number is
            # supposed to predict.
            if (st["align_at_engage"] is None
                    and rig.probes["jaw"].history
                    and rig.probes["jaw"].history[-1]["force"] > 0.15):
                st["align_at_engage"] = st["align"]
            if u >= 1.0:
                if st["align_at_engage"] is None:
                    st["align_at_engage"] = st["align"]
                st.update(ph="swing", t0=t, dur=T_SWING, e=0.0)
                st["from"] = st["theta"]
                st["to"] = st["theta"] - np.radians(STROKE_DEG)
                st["nu0"], st["z0"] = nu, float(bq[ni, 2])
        elif ph == "swing":
            st["theta"] = st["from"] + (st["to"] - st["from"]) * ease(u)
            if u >= 1.0:
                d_nu = np.degrees(st["nu0"] - nu)
                d_z = (st["z0"] - float(bq[ni, 2])) * 1000
                st["log"].append({"stroke": st["n"], "nut_deg": round(d_nu, 2),
                                  "descent_mm": round(d_z, 3),
                                  "align_at_engage_deg": round(
                                      st.get("align_at_engage") or 0.0, 2),
                                  "follow": round(d_nu / STROKE_DEG, 3)})
                print(f"t={t:5.2f}s stroke {st['n']}: jaw square to "
                      f"{st.get('align_at_engage') or 0.0:+.2f} deg, wrench "
                      f"{STROKE_DEG:.0f} deg -> nut {d_nu:+.1f} deg, "
                      f"down {d_z:+.3f} mm")
                st.update(ph="off", t0=t, dur=T_OFF, n=st["n"] + 1)
        elif ph == "off":
            st["e"] = CLEAR_E * ease(u)
            # HAS THE TOOL BEEN DROPPED?  An 85 g steel spanner held in a
            # friction pinch is not retained indefinitely against repeated
            # jaw reactions, and when it goes the rollout has nothing left
            # to measure -- the arm just waves an empty hand at a nut for
            # the rest of the clock.  Say so once, stop the ladder, and let
            # the QA report which stroke it happened on.
            if st["lost"] is None and abs(st["z_meas"]) > TOOL_LOST_Z:
                st["lost"] = st["n"]
                print(f"t={t:5.2f}s TOOL LOST after {st['n']} strokes: the "
                      f"jaw seat is {st['z_meas']*1000:+.1f} mm off the "
                      f"nut's plane, i.e. the spanner is no longer in the "
                      f"pinch")
            # STOP WHEN IT IS TIGHT, NOT WHEN THE COUNTER RUNS OUT.  The
            # nut reaches its head-shoulder seat after ~6 strokes and every
            # stroke after that is a re-seat of the jaw for no travel --
            # which is also where the spanner eventually slips out of the
            # pinch.  A person stops when the nut stops going down; so does
            # this, on the MEASURED descent.
            # SEATED WITHIN THE THREAD'S OWN BACKLASH.  The 2026-09-11
            # rollout stopped 0.07 mm short of the mesh-measured seat
            # (2.93 of 3.00 mm) -- inside the +-0.3 mm backlash the nut task
            # measures -- so an exact test never fired, the ladder went on
            # to a sixth engage against a nut that could not turn, the jaw
            # met a corner (align 1.6 -> 23 deg, jaw 110 N) and cammed the
            # spanner out of the pinch.  Seat within SEAT_TOL instead.
            if st["seated"] is None and (Z_START - float(bq[ni, 2])) >= \
                    (Z_START - SEAT_ZC) - SEAT_TOL:
                st["seated"] = st["n"]
                print(f"t={t:5.2f}s SEATED after {st['n']} strokes: the nut "
                      f"has descended {(Z_START - float(bq[ni, 2]))*1000:.2f}"
                      f" mm of the {(Z_START - SEAT_ZC)*1000:.2f} mm to its "
                      f"head-shoulder seat")
            if u >= 1.0:
                if (st["seated"] is not None or st["lost"] is not None
                        or st["n"] >= N_STROKES):
                    st.update(ph="done", t0=t, dur=T_END)
                else:
                    st.update(ph="index", t0=t, dur=T_IDX, e=CLEAR_E)
                    st["from"] = st["theta"]
                    # re-seat on the NEXT flat pair, measured off the nut
                    st["to"] = engage_angle(nu, th0)
        elif ph == "index":
            # the target is RE-MEASURED every frame of the return, so the
            # wrench arrives already square to wherever the nut ended up
            st["to"] = engage_angle(nu, th0)
            st["theta"] = st["from"] + (st["to"] - st["from"]) * ease(u)
            if u >= 1.0:
                # the alignment latch is per stroke: every engage records
                # ITS OWN first-contact squareness (it used to be latched
                # once, at the first engage, and reported for all strokes)
                st.update(ph="engage", t0=t, dur=T_ENG, ue=0.0)
                st["theta"] = engage_angle(nu, th0)
                st["align_at_engage"] = None

        # rate-capped follow of the measured nut height (pitfall 6.1/14)
        z_now = float(bq[ni, 2])
        st["z"] = float(np.clip(z_now, st["z"] - DZ_RATE, st["z"] + DZ_RATE))
        fg = FING_TOUCH + (FING_GRIP - FING_TOUCH) * min(1.0, t / 0.5)
        rig.drive_arm(tcp_for(st["theta"], st["e"] - st["br"],
                              st["z"] + st["bz"]),
                      down_grasp_quat(wrist_yaw(st["theta"])),
                      fingers=fg)

    def extra(state, t):
        bq = state.body_q.numpy()
        return {
            "phase": st["ph"],
            "nut_deg": round(np.degrees(nut_yaw.total), 2),
            "nut_z_mm": round(float(bq[ni, 2]) * 1000, 3),
            "descent_mm": round((Z_START - float(bq[ni, 2])) * 1000, 3),
            "wrench_deg": round(np.degrees(wr_yaw.total), 2),
            "align_err_deg": round(st.get("align", 0.0), 3),
            "seat_err_mm": round(st["e_meas"] * 1000, 3),
            "seat_dz_mm": round(st["z_meas"] * 1000, 3),
            "jaw_N": round(rig.probes["jaw"].history[-1]["force"], 3),
            "thread_N": round(rig.probes["thread"].history[-1]["force"], 3),
            "padL_N": round(rig.probes["padL"].history[-1]["force"], 3),
            "padR_N": round(rig.probes["padR"].history[-1]["force"], 3),
        }

    rec = rig.run(t_end, control, extra)
    paths = rec.save("rollout", force_keys=["jaw_N", "thread_N", "padL_N",
                                            "padR_N", "descent_mm"])
    print("saved:", paths)

    qa = rig.qa(paths)
    d = np.load(paths["states"], allow_pickle=True)
    bq = d["body_q"]
    descent = float((Z_START - bq[:, ni, 2].min()) * 1000)
    turns = descent / 1000.0 / LEAD
    strokes = st["log"]
    follow = float(np.mean([s["follow"] for s in strokes])) if strokes else 0.0
    ok_turn = descent >= 1.5 and follow >= 0.5
    held = st["lost"] is None
    qa.report["checks"]["tool_retained"] = {
        "pass": bool(held),
        "strokes_delivered": len(strokes),
        "strokes_requested": N_STROKES,
        "seated_after_stroke": st["seated"],
        "lost_after_stroke": st["lost"],
        "note": "an 85 g steel spanner is held ONLY in pad friction across "
                "an 11.84 mm shank; if it leaves the pinch the rollout has "
                "nothing left to measure and the ladder stops there",
    }
    qa.report["pass"] &= held

    jaw = np.array([r["force"] for r in rig.probes["jaw"].history])
    pads = np.array([max(a["force"], b["force"]) for a, b
                     in zip(rig.probes["padL"].history,
                            rig.probes["padR"].history)])
    # GATED ON THE 95th PERCENTILE, WITH THE PEAK REPORTED BESIDE IT.  A
    # spanner held in friction slips, and the tool-pose servo's job is to
    # push it back where it belongs; that shows up as short transients at
    # the moment the jaw re-seats.  Gating the instantaneous maximum makes
    # the verdict a function of one frame out of 1332, so the sustained
    # load is what is gated and the transient is what is published.
    ok_tool = (float(np.percentile(jaw, 95)) > 0.5
               and float(np.percentile(pads, 95)) < 45.0)

    qa.report["checks"]["wrench_tightens_nut"] = {
        "pass": bool(ok_turn),
        "strokes": len(strokes),
        "stroke_deg": STROKE_DEG,
        "nut_descent_mm": round(descent, 3),
        "nut_turns": round(turns, 3),
        "lead_mm_per_turn": round(LEAD * 1000, 3),
        "mean_follow_per_stroke": round(follow, 3),
        "descent_needed_to_seat_mm": round((Z_START - SEAT_ZC) * 1000, 2),
        "per_stroke": strokes,
        "note": "the nut is turned ONLY through the jaw: a 60 deg wrench "
                "stroke per hex index, re-seated between strokes, and the "
                "descent is the thread's own helix",
    }
    qa.report["pass"] &= ok_turn

    aligns = [abs(sg["align_at_engage_deg"]) for sg in strokes]
    ok_align = bool(aligns) and max(aligns) <= ALIGN_TOL
    qa.report["checks"]["jaw_square_to_the_flats"] = {
        "pass": ok_align,
        "worst_align_at_engage_deg": round(max(aligns), 3) if aligns else None,
        "tolerance_deg": ALIGN_TOL,
        "slide_on_tolerance_deg": round(
            float(np.degrees(np.arctan((0.01615 - NUT_AF) / NUT_AC))), 2),
        "per_stroke_deg": [sg["align_at_engage_deg"] for sg in strokes],
        "note": "an open-end wrench that arrives off-square lands on two "
                "CORNERS and rounds them; the jaw is re-aligned on the "
                "nut's MEASURED yaw continuously through every approach",
    }
    qa.report["pass"] &= ok_align

    qa.report["checks"]["load_path_is_the_tool"] = {
        "pass": bool(ok_tool),
        "jaw_N_p95": round(float(np.percentile(jaw, 95)), 2),
        "peak_jaw_N": round(float(jaw.max()), 2),
        "mean_jaw_N_in_stroke": round(float(jaw[jaw > 0.1].mean())
                                      if (jaw > 0.1).any() else 0.0, 2),
        "pad_N_p95": round(float(np.percentile(pads, 95)), 2),
        "peak_pad_N": round(float(pads.max()), 2),
        "grip_radius_mm": round(GRIP_R * 1000, 1),
        "note": "torque reaches the thread as a COUPLE in the jaw (flat "
                "against flat, form closure); the pads only have to hold "
                "the handle, and must not be a vise",
    }
    qa.report["pass"] &= ok_tool

    print(f"  wrench_tightens_nut: {'PASS' if ok_turn else 'FAIL'}  "
          f"descent {descent:.2f} mm = {turns:.2f} turns, follow "
          f"{follow:.2f}/stroke")
    print(f"  jaw_square_to_the_flats: {'PASS' if ok_align else 'FAIL'}  "
          f"worst {max(aligns) if aligns else 0:.2f} deg of {ALIGN_TOL}")
    print(f"  load_path_is_the_tool: {'PASS' if ok_tool else 'FAIL'}  "
          f"jaw peak {jaw.max():.2f} N, pad peak {pads.max():.2f} N")
    rig.finish(qa)


if __name__ == "__main__":
    main()
