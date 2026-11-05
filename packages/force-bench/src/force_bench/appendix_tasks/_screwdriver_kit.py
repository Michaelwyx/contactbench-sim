"""Tighten / loosen screw with screwdriver (paper Table C1 tasks 15 and 16), screwdriver in the hand, on the kit's
toy assembly block: the setup of the task videos.

    Tighten. "A screw is placed into the screw slot of an assembly block in the workspace, but the threads are
    unengaged. Use the screwdriver grasped in hand to tighten the screw."
    Loosen.  "A screw is placed into the screw slot of an assembly block in the workspace and is fully tightened.
    Use the screwdriver grasped in hand to loosen the screw."
    Success: registry.CRITERIA[15] / [16] (verbatim in the verdict).

THE SCENE (finalized kit, ``force_bench.kit``; arranged as in the videos)
-------------------------------------------------------------------------
* toy-assembly-block, both halves (80 x 30 mm, 19.5 mm tall on the table: the final bottom half is 10.5 mm), static
  (the real block is held by Velcro), long axis LEFT-RIGHT as seen from the camera, which faces the robot from the
  far side of the board as the video camera faces the person;
* the kit nut, captive in the hex pocket under the right-hand hole (pushed up against the pocket ceiling, the
  position the screw's pull gives it), static;
* the kit screw (6.5 revolutions, or 7.5 = "Longer screw") in the RIGHT-hand hole as seen from the camera, the left
  hole empty: 15 mm round head with a 5.5 mm square recess 4.0 mm deep, right-hand thread, 2.34 mm per turn;
* the kit screwdriver (5.0 mm square tip, 12 mm rounded-square handle) held in the hand.
"THREADS UNENGAGED" is the screw dropped into the hole and resting on the nut's thread.  Measured on the kit meshes
(radius maps of the nut's bore and the screw's thread, the screw lowered without turning at 36 phases): the screw's
1.7 mm unthreaded lead-in (r 3.54 mm) drops into the nut's bore (crest r 4.05 mm), and its first thread (crest
r 4.95 mm) then lands on the nut's thread at a height that is a sawtooth over the phase -- 0 to 2.14 mm above the
seat for the 6.5-revolution screw (mean 1.12 mm), 2.32 to 4.46 mm for the 7.5-revolution one (mean 3.44 mm: one
pitch more).  The screw starts at the MEAN (``RISE``): 172 deg of turning to the seat (6.5) or 529 deg (7.5).  The
tighten video agrees: the head drops 1.1 mm from its first frame to its last (pixel measurement on the head's top
edge, 0.36 mm/px at the block, 35 deg camera; the Allen-key video gives 1.8 mm).  (The from-table rows' 4.0 mm start
puts the screw's tip on the nut's top face, which the narrow lead-in never rests on.)

THE SCREW MECHANISM is ``_screwdriver_table.ThreadModel`` (the from-table rows' model, unchanged): world -prismatic
z-> carrier -revolute z-> screw; a helix spring couples the two dofs (2.34 mm per clockwise turn down), load-
dependent elasto-plastic thread friction, the seat a hard stop, a seat preload from the measured seating torque
(loosen starts seated with 20 mN.m), and the holding probe.  The tool-screw drive is real contact: the tip and the
recess carry their own fine SDFs (0.04-0.05 mm cells against the 0.25 mm drive clearance).

GRASP: a TOP-DOWN pinch on two flats of the handle, the screwdriver hanging tip-down below the fingers -- the
person's top grip in the video.  The TCP is GRASP_D = 95 mm from the tip: the 125 mm tool's butt then ends 30 mm
above the TCP, 7.5 mm under the FR3 palm plate (37.45 mm above the TCP), and the pads (TCP -9.45 .. +9.05 mm) sit on
the handle's flats (70-125 mm from the tip).  The tool's axis is the wrist axis, so a turn of joint 7 turns the tool
about its own axis: a stroke can use joint 7's whole range (+-172.8 deg, less a 0.25 rad margin each side): 227 to
317 deg, depending on how the recess is turned at the insertion.  The tool is a free body held by fingertip friction
only.

EXPERT (privileged state):
  carry    from the start pose to the tool upright TIP_ABOVE over the measured recess, its yaw kept;
  turn     in place, about the tool's own axis, to the recess-congruent yaw (mod 90 deg) that puts joint 7 at the
           end of its range the stroke starts from (tighten = clockwise from above = joint 7 increasing).  Joint 7
           and the tool's yaw move one for one only at a fixed wrist position: predicted before the carry, joint 7
           was off by up to 30 deg across the seed square (measured), hence the turn after the carry;
  align / lower / insert   the from-table rows' integral servo on the measured tool pose, then a force-servoed
           insertion (lands at > 0.6 N, presses 6 N; done at >= 3 mm in the 4 mm recess, tool < 1 deg from vertical);
  strokes  the recorded hand pose turned about the screw axis with a 6 N press, to the joint-7 margin; between
           strokes: press off, unwind 4 deg inside the tip's backlash, lift the tip 12 mm out IN THE GRIP, swing back
           a multiple of 90 deg in the air (flats stay congruent), re-insert.  Tighten creeps onto the seat
           (15 deg/s) and snugs to 40 mN.m at the tip; loosen breaks the preload and backs the screw off LOOSE_GOAL;
  ending   press off, unwind, lift the tool out of the recess and move away with it (the person leaves with the
           tool in every video), hold still; then the probe torque on the screw.
Joint targets go through ``_screwdriver_table.JointTracker`` (velocity/acceleration limits downstream of the IK).

SEEDED START (--seed N): the block's pose is drawn as the paper's protocol draws each object's origin -- uniform in a
0.15 x 0.15 m square at the board centre -- with its yaw within +-30 deg of the video's arrangement (the paper names
no orientation range; this keeps the screw on the camera's right); the hand starts at the same pose for every seed.
The draw is recorded in the verdict ("start_state").  The screw keeps its place in the block (a turned block turns
the recess).  Over the reachable screw positions (x 0.418-0.582 m, y -0.052-0.098 m, the block's yaw included) the
IK reaches joint 7's whole working range with the hand pointing down, at zero pose error (``--check`` prints it).

RESULTS (2026-10-29, draft quality, a GPU shared with other jobs; outputs/work/runs/final in the rw-tools worktree)
  tighten_screw_with_screwdriver         PASS x2  172.3 deg in 1 stroke, seating 48.8 / 46.2 mN.m -> preload
                                                  34.2 / 32.3, probe 0.08 / 0.18 deg; 224 / 378 s
  tighten_..._longer_screw               PASS x2  529.2 deg in 2 strokes (270 deg in-air swing, re-insert first try),
                                                  seating 44.9 / 44.5 mN.m, probe 0.09 / 0.18 deg; 521 / 692 s
  loosen_screw_with_screwdriver          PASS x2  206.9 / 207.0 deg in 1 stroke, breakaway at 26.6 / 26.7 mN.m
                                                  (preload 20), probe 81.7 / 89.3 deg in 1 s; 277 / 394 s
  tighten --seed 1 / 2 / 3               PASS     block (0.502, 0.068) 68.7 deg / (0.464, -0.030) 108.9 / (0.438,
                                                  -0.040) 108.1: seating 40.3 / 45.6 / 43.9 mN.m, probe <= 0.19 deg
  loosen --seed 1 / 2 / 3                PASS     backed off 207.4 / 206.0 / 206.0 deg, breakaway 25.9 / 27.1 / 26.0
  every run: tip in first try (4.03 mm at 5.6-6.0 N, tilt <= 0.04 deg), pads 11-12 N throughout, tool <= 2.0 mm
  of slip in the hand, 0 N robot/tool vs screw/block/nut at the end, hand 108 mm away, links < 2 mm/s, tool-screw
  penetration <= 0.08 mm.

Run: python -m force_bench.appendix_tasks.tighten_screw_with_screwdriver [--screw 7.5] [--seed N] [--name N]
     python -m force_bench.appendix_tasks.loosen_screw_with_screwdriver [--seed N]
     (TB_OUT_ROOT=<dir> TB_RENDER_FAST=1 TB_MULTIVIEW=0 for iterations; SDK_STOP=<phase> stops when it begins;
      --check builds the scene, checks the IK window and holds the start for 0.5 s, unrecorded)
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import world_T
from force_bench.kit import add_asset, load_kit, place_on_table, pose_matrix
from force_bench.kit.build import add_part_shape, part_mass, to_wp
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.screwdriver import split_driver
from force_bench.tasks.screwdriver import subdiv as subdiv_p
from force_bench.appendix_tasks.common import (
    CONTACT_FREE_N, Placement, RobotContactProbe, StopRun, appendix_rig, sample_start,
    save_appendix_verdict)
from force_bench.appendix_tasks._hang_plate import PairProbe, R_of_q, quat_of_R, smooth5, tcp_offset_from_link7
from force_bench.appendix_tasks._screwdriver_table import (
    DriveSensor, JointTracker, ThreadModel, congruent, expmap, rotvec, tool_R_vertical,
    LEAD, LOOSE_FREE_DEG, LOOSE_PRELOAD0, PRELOAD_FRAC, PRELOAD_MAX, PRELOAD_MIN, PROBE_TAU, SCREW_KD,
    SCREW_MASS, SEAT_TOL, T_PROBE)

MM = 1000.0

# ── layout (world, m; table top z = 0, robot base at the origin) ─────────────────────────────────────────────────────
BOARD_XY = (0.50, 0.0)              # board centre = the block's nominal centre
BLOCK_YAW = np.pi / 2               # block CAD +x -> world +y: long axis left-right in the camera, the CAD x = +25 mm
                                    # hole (the screw) on the camera's right
SEED_DXY = 0.075                    # the paper's 0.15 x 0.15 m square
SEED_DYAW = np.radians(30.0)
HOLE_CAD = np.array([0.025, 0.0])   # the screw's hole in the block CAD frame
BLOCK_TOP_CAD = 0.020               # block top face in its CAD frame (bottom face at 0.5 mm)
NUT_CEIL_CAD = 0.007475             # hex pocket ceiling (measured); the nut is pushed up against it
SCREW_HEAD_X = 0.006                # screw CAD: axis = x, head underside at x = 6.0 mm, head top at 13.0 mm
HEAD_H = 0.007
RISE = {"6.5": 0.00112, "7.5": 0.00344}   # threads unengaged: head above the seat (mean rest height, module doc)
START_RECESS_DEG = 25.0             # recess orientation at the start, nominal block (as in the video)

# ── tool (kit screwdriver; body frame: AABB centre, +y toward the butt, tip at -y, flats normal to body x and z) ─────
TOOL_LEN = 0.125
TIP_Y = -TOOL_LEN / 2
GRASP_D = 0.095                     # TCP from the tip, along the tool
HANDLE_HALF = 0.0060                # half across the handle flats
TIP_SPLIT = 0.020                   # the tip's own fine SDF covers the last 20 mm

# ── fingers (as _screwdriver_table) ───────────────────────────────────────────────────────────────────────────────
FINGER_KE, FINGER_KD = 8000.0, 60.0
FINGER_F_MAX = 20.0
FINGER_MASS = 0.12
PAD_MU = 1.0
PAD_FACE_OFF = 0.000085
SQUEEZE = 0.0015
F_TOUCH_W = HANDLE_HALF + PAD_FACE_OFF - 0.0001
F_GRIP = HANDLE_HALF + PAD_FACE_OFF - SQUEEZE
GRIP_BAND = (7.0, 16.0)
GRIP_TRIM_STEP = 0.00001
GRIP_TRIM_MAX = 0.0015
ROBOT_GAP = 0.003                   # the FR3's and the ground's contact gap (newton's 0.1 m floods the buffer)

# ── materials, solver ────────────────────────────────────────────────────────────────────────────────────────────────
TOOL_MU = 0.5
SCREW_MU = 0.4
MU_TORSION = 0.002
IMPRATIO = 10.0
ITERATIONS = 20
ARM_KD_TASK = 400.0

# ── expert ────────────────────────────────────────────────────────────────────────────────────────────────────────────
Q7_LO, Q7_HI = -3.0159, 3.0159      # FR3 joint 7
Q7_MARGIN = 0.25
START_TIP_ABOVE = 0.035             # start: tip this far above the (unengaged) head top, over the nominal screw
TIP_ABOVE = 0.020                   # tip above the recess entry before the descent
F_TOUCH = 0.6
F_PRESS = 6.0
PRESS_GAIN = 0.0006
PRESS_LEAD_MAX = 0.004
CREEP_RATE = np.radians(15.0)
LIFT_OUT = 0.012
UNWIND = np.radians(4.0)
SEAT_SLOW = np.radians(15.0)
SNUG_TAU = 0.040
SNUG_OVER = np.radians(20.0)
MAX_STROKES = 8
LOOSE_GOAL = np.radians(200.0)      # backed off 200 deg: preload gone, still on the thread
EXTRACT = 0.030                     # tip lifted out of the recess
RETREAT = 0.070                     # then up and away with the tool
TIMES = dict(settle=0.3, carry=2.2, align=0.6, lower=1.0, insert_max=4.0, s_turn90=2.0, s_unpress=0.35,
             s_lift=0.5, s_swing90=1.6, snug_hold=0.3, unpress=0.5, extract=1.0, retreat=1.4, still=1.0)


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# scene
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
def Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def screw_choice(screw: str) -> str:
    return f"{screw} revolutions"


def build_scene(rig, mode: str, screw: str, block_xy, block_yaw: float) -> dict:
    """FR3 + table + kit block (static) + kit nut (static) + kit screw on its thread joints + free kit screwdriver."""
    b = rig.builder
    rig.add_robot(wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity()))
    for dof in range(7):
        b.joint_target_kd[dof] = ARM_KD_TASK
    for dof in (7, 8):
        b.joint_target_ke[dof] = FINGER_KE
        b.joint_target_kd[dof] = FINGER_KD
    set_finger_effort_limit(b, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = b.body_label.index(bl)
        set_body_mass(b, bi, FINGER_MASS)
        for sid in b.body_shapes.get(bi, []):
            b.shape_material_mu[sid] = PAD_MU
    for i, lbl in enumerate(b.shape_label):
        bi = b.shape_body[i]
        if (bi >= 0 and str(b.body_label[bi]).startswith("fr3/")) or (lbl and "ground" in str(lbl)):
            b.shape_gap[i] = ROBOT_GAP
    rig.table(center_xy=(BOARD_XY[0], BOARD_XY[1]), half=0.30)
    kit = load_kit()

    # block and nut: static, in the block's CAD frame
    T_blk = place_on_table("toy-assembly-block", xy=block_xy, yaw=block_yaw)
    hb = add_asset(rig, "toy-assembly-block", pose=T_blk, label="block")
    nut_part = kit["nut"].parts()[0]
    nut_top = float(nut_part.mesh().bounds[1][2])
    T_nut = T_blk @ pose_matrix(xyz=(HOLE_CAD[0], HOLE_CAD[1], NUT_CEIL_CAD - nut_top))
    hn = add_asset(rig, "nut", pose=T_nut, label="nut", fixed=True)
    block_shapes = [s for v in hb.shapes.values() for s in v]
    nut_shapes = [s for v in hn.shapes.values() for s in v]

    # screw: world -prismatic z-> carrier -revolute z-> screw.  Joint frame: on the hole axis at the block top
    # (the head's underside when seated), z up, x turned so the recess has START_RECESS_DEG at the start.
    q0 = RISE[screw] / LEAD if mode == "tighten" else 0.0
    phase0 = np.radians(START_RECESS_DEG) - BLOCK_YAW - q0      # the screw's phase in the block: a turned block
                                                                 # turns the recess with it
    S0 = (T_blk @ np.r_[HOLE_CAD, BLOCK_TOP_CAD, 1.0])[:3]
    J = pose_matrix(xyz=S0, R=Rz(block_yaw + phase0))
    q_max = max(q0, LOOSE_GOAL + np.radians(150.0)) + 0.6
    carrier = b.add_link(xform=to_wp(J), label="screw_carrier")
    b.add_shape_box(carrier, hx=0.002, hy=0.002, hz=0.002,
                    cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False, density=1000.0, is_visible=False),
                    label="screw_carrier_shape")
    set_body_mass(b, carrier, 0.010)
    j_z = b.add_joint_prismatic(parent=-1, child=carrier, axis=(0, 0, 1), parent_xform=to_wp(J),
                                target_ke=0.0, target_kd=0.0, limit_lower=-0.00005, limit_upper=LEAD * q_max + 0.0006)
    scr_body = b.add_link(xform=to_wp(J), label="screw")
    sa = kit["screw"]
    sp = sa.parts(screw_choice(screw))[0]
    # screw CAD -> screw body: CAD x (axis, toward the head) -> body z, head underside at z = 0
    T_sc = pose_matrix(xyz=(0.0, 0.0, -SCREW_HEAD_X), R=np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]]))
    scr_tm = sp.mesh()
    scr_tm.apply_transform(T_sc)
    scr_fine = subdiv_p(scr_tm, 0.0010)
    scr_mesh, scr_c = make_newton_mesh(scr_fine, sdf=True, sdf_resolution=512, narrow_band=0.0015, margin=0.0002)
    s_scr = b.add_shape_mesh(scr_body, mesh=scr_mesh, xform=wp.transform(wp.vec3(*np.asarray(scr_c, float)),
                                                                        wp.quat_identity()),
                             cfg=rig.cfg(mu=SCREW_MU, mu_torsional=MU_TORSION), color=sp.rgb,
                             label=f"kit/screw/{sp.name}")
    rig.shapes["screw"] = [s_scr]
    j_q = b.add_joint_revolute(parent=carrier, child=scr_body, axis=(0.0, 0.0, 1.0),
                               parent_xform=wp.transform_identity(), child_xform=wp.transform_identity(),
                               target_ke=0.0, target_kd=SCREW_KD, limit_lower=0.0, limit_upper=q_max)
    b.add_articulation([j_z, j_q])
    set_body_mass(b, scr_body, SCREW_MASS)
    for s in block_shapes + nut_shapes + rig.shapes["table"]:
        b.shape_material_mu_torsional[s] = MU_TORSION
        rig.filter_pair(s_scr, s)                                # the thread model positions the screw
    rig._joint_of["screw_z"] = j_z
    rig._joint_of["screw_q"] = j_q
    return dict(j_z=j_z, j_q=j_q, q0=q0, q_max=q_max, phase0=phase0, block_yaw=block_yaw, T_blk=T_blk, T_nut=T_nut,
                S0=S0, screw_xy=S0[:2].copy(), head_top0=float(S0[2] + HEAD_H + LEAD * q0), scr_tm=scr_tm,
                block_shapes=block_shapes, nut_shapes=nut_shapes, screw_shape=s_scr, screw_part=sp.name)


def add_tool(rig, h: dict, T_tool: np.ndarray) -> None:
    """The kit screwdriver as a FREE body at world pose ``T_tool`` (kit CAD: axis x, tip at +x -> tool body frame:
    axis y, tip at -y, origin at the AABB centre).  The 20 mm tip carries its own fine SDF for the drive clearance;
    the visible kit shape carries the print colour and the mass distribution.  Then the scene-wide filters."""
    b = rig.builder
    kit = load_kit()
    da = kit["screwdriver"]
    dp = da.parts()[0]
    d_cad = dp.mesh()
    c_cad = 0.5 * (d_cad.bounds[0] + d_cad.bounds[1])
    T_dc = pose_matrix(R=Rz(-np.pi / 2)) @ pose_matrix(xyz=-c_cad)
    drv = d_cad.copy()
    drv.apply_transform(T_dc)
    assert abs(float(drv.bounds[0][1]) - TIP_Y) < 1e-4 and abs(float(np.ptp(drv.bounds[:, 1])) - TOOL_LEN) < 1e-4
    tool = b.add_body(xform=to_wp(T_tool), label="screwdriver")
    add_part_shape(rig, tool, da, dp, T_dc, "screwdriver", collide=False)          # visible, print colour, mass
    tip_tm, rest_tm = split_driver(drv, TIP_SPLIT)
    tip_tm, rest_tm = subdiv_p(tip_tm, 0.0012), subdiv_p(rest_tm, 0.0020)
    tool_shapes = []
    for tm, lbl, nb, mg in ((tip_tm, "screwdriver_tip_collider", 0.0015, 0.0002),
                            (rest_tm, "screwdriver_collider", 0.002, 0.0005)):
        mesh, c = make_newton_mesh(tm, sdf=True, sdf_resolution=512, narrow_band=nb, margin=mg)
        cfg = rig.cfg(mu=TOOL_MU, mu_torsional=MU_TORSION, is_visible=False)
        cfg.density = 0.0
        tool_shapes.append(b.add_shape_mesh(tool, mesh=mesh, cfg=cfg, label=lbl,
                                            xform=wp.transform(wp.vec3(*np.asarray(c, float)), wp.quat_identity())))
    set_body_mass(b, tool, part_mass(da, dp))
    rig.shapes["tool"] = tool_shapes
    rig._filter_arm_from(tool, filter_fingers=False)             # fingertips touch the tool, the arm never does
    # static-static pairs and the ground plane carry nothing here (the table is the support)
    statics = list(b.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            rig.filter_pair(statics[i], statics[j])
    ground = [i for i, lbl in enumerate(b.shape_label) if lbl and "ground" in str(lbl)]
    for g in ground:
        for s in range(len(b.shape_label)):
            if s != g and b.shape_body[s] != -1:
                rig.filter_pair(g, s)
    h.update(drv_tm=drv, tool_shapes=tool_shapes, tool_mass=part_mass(da, dp))


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# measurement
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
class Meas:
    """What the expert reads each frame (privileged state)."""

    def __init__(self, rig, h):
        self.rig, self.h = rig, h
        self.i_tool = rig.body_index("screwdriver")
        self.i7 = rig.body_index("fr3/fr3_link7")
        qs = rig.model.joint_q_start.numpy()
        self.cz, self.cq = int(qs[h["j_z"]]), int(qs[h["j_q"]])
        self.T_off = tcp_offset_from_link7(rig)
        v = h["drv_tm"].vertices
        self.v_low = v[np.random.RandomState(0).choice(len(v), min(600, len(v)), replace=False)]

    def update(self, state):
        bq = state.body_q.numpy()
        jq = state.joint_q.numpy()
        T = world_T(bq[self.i_tool])
        self.R, self.p = T[:3, :3], T[:3, 3]
        self.tip = self.p + self.R @ np.array([0.0, TIP_Y, 0.0])
        self.grasp = self.p + self.R @ np.array([0.0, TIP_Y + GRASP_D, 0.0])
        self.axis = self.R[:, 1]
        xh = self.R[:, 0] - self.R[2, 0] * np.array([0.0, 0.0, 1.0])
        self.flats_yaw = float(np.arctan2(xh[1], xh[0]))
        self.tilt = float(np.degrees(np.arccos(np.clip(self.axis[2], -1.0, 1.0))))
        self.low = float((self.v_low @ self.R[2] + self.p[2]).min())
        self.q = float(jq[self.cq])
        self.z = float(jq[self.cz])
        self.recess_yaw = float(self.h["block_yaw"] + self.h["phase0"] + self.q)
        self.T_tcp = world_T(bq[self.i7]) @ self.T_off
        self.tcp = self.T_tcp[:3, 3]
        self.in_tcp = (np.linalg.inv(self.T_tcp) @ np.r_[self.p, 1.0])[:3]     # tool centre in the TCP frame
        return self

    def T_tool(self):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.p
        return T


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# expert
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
PHASES = ["settle", "carry", "turn", "align", "lower", "s_insert", "s_turn", "snug_hold", "s_unpress", "s_lift", "s_swing",
          "end_unpress", "extract", "retreat", "still", "probe", "done"]


class Expert:
    """Phase machine returning (tcp_pos, tcp_quat, finger_dof) every frame.  One grasp for the whole episode."""

    def __init__(self, rig, meas, thread, drive, trk, mode, h, p0, q0):
        self.rig, self.m, self.thread, self.drive, self.trk, self.mode, self.h = rig, meas, thread, drive, trk, mode, h
        self.sgn = -1.0 if mode == "tighten" else 1.0          # tighten = clockwise from above = -yaw
        self.phase, self.t0 = "settle", 0.0
        self.t_start = {"settle": 0.0}
        self.finished = False
        self.p_cmd, self.q_cmd, self.f_cmd = np.asarray(p0, float), np.asarray(q0, float), F_GRIP
        self.trim = 0.0
        self.k: dict = {}
        self.strokes = 0
        self.press = 0.0
        self.z_land = 0.0
        self.corr = np.zeros(3)
        self.k_swing = 0.0
        self.seated = False
        self.tau_seat = 0.0
        self.released = mode == "tighten"
        self.t_probe = None
        self.t_off_screw = None
        self.stroke_log: list[dict] = []
        self.insert_log: list[dict] = []
        self.retries = 0
        self.phi_end = 0.0

    # -- bookkeeping --
    def goto(self, ph, t):
        self.phase, self.t0 = ph, t
        self.t_start.setdefault(ph, t)
        self.k = {}
        print(f"t={t:6.2f}s -> {ph}   q7 {np.degrees(self.trk.q[6]):+7.1f} deg  screw {np.degrees(self.m.q):7.1f} deg",
              flush=True)

    def phase_id(self):
        return PHASES.index(self.phase)

    def s(self, t, dur):
        return min(1.0, (t - self.t0) / dur)

    def Fz(self):
        """Downward push of the tool on the screw (N)."""
        return -self.drive.history[-1]["Fz"] if self.drive.history else 0.0

    def grip_cmd(self, last):
        F = 0.5 * (last("padL") + last("padR"))
        lo, hi = GRIP_BAND
        if F < lo:
            self.trim = max(-GRIP_TRIM_MAX, self.trim - GRIP_TRIM_STEP)
        elif F > hi:
            self.trim = min(GRIP_TRIM_MAX, self.trim + GRIP_TRIM_STEP)
        return F_GRIP + self.trim

    # -- poses --
    def in_hand(self):
        return np.linalg.inv(self.m.T_tcp) @ self.m.T_tool()

    def tcp_for_tool(self, R_des, p_tip_des, H):
        T_des = np.eye(4)
        T_des[:3, :3] = R_des
        T_des[:3, 3] = p_tip_des - R_des @ np.array([0.0, TIP_Y, 0.0])
        T_cmd = T_des @ np.linalg.inv(H)
        return T_cmd[:3, 3], quat_of_R(T_cmd[:3, :3])

    def recess_center(self):
        return np.array([self.h["screw_xy"][0], self.h["screw_xy"][1], self.h["S0"][2] + HEAD_H + self.m.z])

    def insert_psi(self):
        """Tool flats on the measured recess, the 90 deg branch nearest the tool's current flats."""
        return congruent(self.m.recess_yaw, self.m.flats_yaw)

    def stroke_start_psi(self):
        """The recess-congruent tool yaw that puts joint 7 at the start end of its range: tighten turns the tool
        clockwise (joint 7 increasing), so it starts within 90 deg above the lower margin; loosen mirrored.  Joint 7
        and the tool yaw move one for one (the tool is on the wrist axis), measured from the current pose."""
        q7, psi = float(self.trk.q[6]), self.m.flats_yaw
        lo, hi = Q7_LO + Q7_MARGIN, Q7_HI - Q7_MARGIN
        best = None
        for k in range(-6, 7):
            cand = congruent(self.m.recess_yaw, psi) + k * np.pi / 2
            q7c = q7 - (cand - psi)
            if not (lo <= q7c <= hi):
                continue
            key = q7c if self.mode == "tighten" else -q7c
            if best is None or key < best[0]:
                best = (key, cand, q7c)
        return best[1], best[2]

    def tool_servo(self, tip_des, z_free=True, gain_p=0.08, gain_r=0.06, psi=None):
        """Integral servo on the MEASURED tool (``_screwdriver_table`` Expert.tool_servo): a rotation about the
        measured tip toward the upright, recess-congruent attitude, then a rate-limited step on the tip error."""
        m = self.m
        tip_des = np.asarray(tip_des, float)
        prev = self.k.get("tip_des_prev", tip_des)
        self.k["tip_des_prev"] = tip_des.copy()
        p = self.p_cmd + (tip_des - prev)
        R_des = tool_R_vertical(self.insert_psi() if psi is None else psi)
        rv = rotvec(R_des @ m.R.T)
        step = gain_r * rv
        lim = np.radians(0.4)
        if np.linalg.norm(step) > lim:
            step = step * lim / np.linalg.norm(step)
        Rd = expmap(step)
        R_cmd = Rd @ R_of_q(self.q_cmd)
        p = m.tip + Rd @ (p - m.tip)
        e = tip_des - m.tip
        if not z_free:
            e[2] = 0.0
        p = p + np.clip(gain_p * e, -0.0003, 0.0003)
        self.k["tool_err"] = (round(float(np.linalg.norm(e[:2])) * MM, 3), round(float(np.degrees(np.linalg.norm(rv))), 2))
        return p, quat_of_R(R_cmd)

    def drive_pose(self, F_target, phi, gripped=True):
        """The hand pose recorded when the tip seated, turned by phi about the vertical screw axis; force servo on
        the press below the landing height; a slow xy integral keeps the tool's grasp point over the axis."""
        m = self.m
        self.press = float(np.clip(self.press + PRESS_GAIN * (F_target - self.Fz()) / self.rig.fps,
                                   -PRESS_LEAD_MAX, PRESS_LEAD_MAX))
        sxy = self.h["screw_xy"]
        if gripped:
            exy = np.array([sxy[0] - m.grasp[0], sxy[1] - m.grasp[1], 0.0])
            self.corr = np.clip(self.corr + np.clip(0.03 * exy, -0.0001, 0.0001), -0.008, 0.008)
        p0, R0 = self.k_ref
        R = expmap(np.array([0.0, 0.0, phi]))
        c = np.array([sxy[0], sxy[1], p0[2]])
        p = c + R @ (p0 - c)
        p[2] = self.z_land - self.press
        return p + self.corr, quat_of_R(R @ R0)

    # -- the step --
    def step(self, t, last):
        m = self.m
        ph = self.phase
        T = TIMES
        p, q, f = self.p_cmd, self.q_cmd, self.f_cmd
        if ph == "settle":
            f = self.grip_cmd(last)
            if t - self.t0 >= T["settle"]:
                self.goto("carry", t)
        elif ph == "carry":
            # the tool upright over the measured recess, its yaw kept (the turn comes next, in place: joint 7 and
            # the tool's yaw only move one for one about a fixed wrist position)
            f = self.grip_cmd(last)
            if "p1" not in self.k:
                H = self.in_hand()
                tgt = self.recess_center() + np.array([0.0, 0.0, TIP_ABOVE])
                p1, q1 = self.tcp_for_tool(tool_R_vertical(m.flats_yaw), tgt, H)
                self.k.update(p0=self.p_cmd.copy(), R0=R_of_q(self.q_cmd), p1=p1, R1=R_of_q(q1))
            u = smooth5(self.s(t, T["carry"]))
            k = self.k
            p = k["p0"] + (k["p1"] - k["p0"]) * u + np.array([0.0, 0.0, 0.015 * np.sin(np.pi * u)])
            q = quat_of_R(expmap(rotvec(k["R1"] @ k["R0"].T) * u) @ k["R0"])
            if t - self.t0 >= T["carry"]:
                self.goto("turn", t)
        elif ph == "turn":
            # about the tool's own axis to the recess-congruent yaw that starts the stroke at the end of joint 7's
            # range (a slerp would take the short way round: the angle is applied explicitly)
            f = self.grip_cmd(last)
            if "dpsi" not in self.k:
                psi1, q7_1 = self.stroke_start_psi()
                dpsi = psi1 - m.flats_yaw
                self.k.update(p0=self.p_cmd.copy(), R0=R_of_q(self.q_cmd), c=m.tip.copy(), dpsi=dpsi,
                              dur=max(0.4, T["s_swing90"] * abs(dpsi) / (np.pi / 2)))
                print(f"    turn: recess {np.degrees(m.recess_yaw) % 360:.1f} deg, tool flats "
                      f"{np.degrees(m.flats_yaw):.1f} -> {np.degrees(psi1):.1f} deg ({np.degrees(dpsi):+.1f}), "
                      f"joint 7 {np.degrees(self.trk.q[6]):+.1f} -> {np.degrees(q7_1):+.1f} deg", flush=True)
            k = self.k
            u = smooth5(self.s(t, k["dur"]))
            R = expmap(np.array([0.0, 0.0, k["dpsi"] * u]))
            p = k["c"] + R @ (k["p0"] - k["c"])
            q = quat_of_R(R @ k["R0"])
            if t - self.t0 >= k["dur"]:
                self.goto("align", t)
        elif ph == "align":
            f = self.grip_cmd(last)
            p, q = self.tool_servo(self.recess_center() + [0.0, 0.0, TIP_ABOVE])
            if t - self.t0 >= T["align"]:
                self.goto("lower", t)
        elif ph == "lower":
            f = self.grip_cmd(last)
            u = smooth5(self.s(t, T["lower"]))
            p, q = self.tool_servo(self.recess_center() + [0.0, 0.0, TIP_ABOVE + (0.0010 - TIP_ABOVE) * u])
            if t - self.t0 >= T["lower"]:
                self.goto("s_insert", t)
        elif ph == "s_insert":
            f = self.grip_cmd(last)
            k = self.k
            if "z0" not in k:
                k.update(z0=self.p_cmd[2], dz=0.0, landed=False, t_land=None)
            if not k["landed"]:
                above = m.tip[2] - self.recess_center()[2]
                k["dz"] -= (0.010 if above > 0.002 else 0.005) / self.rig.fps
                if self.Fz() > F_TOUCH:
                    k["landed"], k["t_land"] = True, t
                    k["z_land"] = k["z0"] + k["dz"]
            else:
                k["dz"] -= float(np.clip(PRESS_GAIN * (F_PRESS - self.Fz()), -0.003, 0.003)) / self.rig.fps
            k["dz"] = max(k["dz"], -(LIFT_OUT + 0.010))
            p, q = self.tool_servo(self.recess_center(), z_free=False)
            p[2] = k["z0"] + k["dz"]
            depth = self.recess_center()[2] - m.tip[2]
            ok = k["landed"] and t - k["t_land"] > 0.25 and depth > 0.0030 and m.tilt < 1.0
            if (not ok) and t - self.t0 >= T["insert_max"] and self.retries < 2:
                self.retries += 1
                print(f"    insertion not seated (depth {depth*MM:.2f} mm, press {self.Fz():.2f} N): lift and "
                      f"retry {self.retries}", flush=True)
                self.k_swing = 0.0
                self.goto("s_lift", t)
            elif ok or t - self.t0 >= T["insert_max"]:
                self.retries = 0
                self.insert_log.append({"t": round(t, 2), "depth_mm": round(depth * MM, 2), "press_N": round(self.Fz(), 2),
                                        "ok": bool(ok), "tilt_deg": round(m.tilt, 2),
                                        "joint7_deg": round(float(np.degrees(self.trk.q[6])), 1)})
                print(f"    inserted: tip depth {depth*MM:.2f} mm, press {self.Fz():.2f} N, tilt {m.tilt:.2f} deg, "
                      f"ok {ok}", flush=True)
                self.z_land = float(k["z_land"]) if k.get("z_land") is not None else p[2]
                self.press = float(self.z_land - p[2])
                self.corr = np.zeros(3)
                self.k_ref = (np.asarray(p, float).copy(), R_of_q(q))
                self.goto("s_turn", t)
        elif ph in ("s_turn", "snug_hold", "s_unpress", "s_lift", "s_swing"):
            p, q, f = self.stroke_step(t, last)
        elif ph == "end_unpress":
            u = smooth5(self.s(t, T["unpress"]))
            p, q = self.drive_pose(0.2, self.phi_end - self.sgn * UNWIND * u, gripped=False)
            f = self.grip_cmd(last)
            if t - self.t0 >= T["unpress"]:
                self.goto("extract", t)
        elif ph == "extract":
            f = self.grip_cmd(last)
            p0 = self.k.setdefault("p0", self.p_cmd.copy())
            u = smooth5(self.s(t, T["extract"]))
            p = p0 + np.array([0.0, 0.0, EXTRACT * u])
            if t - self.t0 >= T["extract"]:
                self.t_off_screw = t
                self.goto("retreat", t)
        elif ph == "retreat":
            # away with the tool: up, and back toward the robot base
            f = self.grip_cmd(last)
            p0 = self.k.setdefault("p0", self.p_cmd.copy())
            back = -p0[:2] / max(np.linalg.norm(p0[:2]), 1e-6)
            u = smooth5(self.s(t, T["retreat"]))
            p = p0 + np.r_[0.040 * back, RETREAT] * u
            if t - self.t0 >= T["retreat"]:
                self.goto("still", t)
        elif ph == "still":
            f = self.grip_cmd(last)
            if t - self.t0 >= T["still"]:
                self.t_probe = t
                self.goto("probe", t)
        elif ph == "probe":
            f = self.grip_cmd(last)
            kk = (t - self.t0) / T_PROBE
            # +q = loosen-way (counter-clockwise from above), -q = tighten-way
            self.thread.probe = PROBE_TAU if kk < 1.0 else (-PROBE_TAU if kk < 2.0 else 0.0)
            if kk >= 2.0:
                self.thread.probe = 0.0
                self.goto("done", t)
        elif ph == "done":
            self.finished = True
        if (not self.released) and m.q >= np.radians(LOOSE_FREE_DEG):
            self.released = True
            self.thread.set_preload(0.0, m.q)
        self.p_cmd, self.q_cmd, self.f_cmd = np.asarray(p, float), np.asarray(q, float), float(f)
        return self.p_cmd, self.q_cmd, self.f_cmd

    # -- strokes --
    def q7_room(self):
        """Joint-7 travel left in the stroke direction (rad), and back toward the start end."""
        q7 = float(self.trk.q[6])
        lo, hi = Q7_LO + Q7_MARGIN, Q7_HI - Q7_MARGIN
        return (hi - q7, q7 - lo) if self.mode == "tighten" else (q7 - lo, hi - q7)

    def stroke_step(self, t, last):
        m = self.m
        T = TIMES
        ph = self.phase
        sgn = self.sgn
        f = self.grip_cmd(last)
        p, q = self.p_cmd, self.q_cmd
        if ph == "s_turn":
            if "q0" not in self.k:
                span = max(0.0, self.q7_room()[0])
                self.k.update(q0=m.q, span=span, u=0.0, phi=0.0, seat_phi=None, q7_0=float(self.trk.q[6]),
                              dur=max(0.8, T["s_turn90"] * span / (np.pi / 2)))
            kt = self.k
            near = self.mode == "tighten" and m.q < SEAT_SLOW
            if near:
                kt["phi"] = max(-kt["span"], kt["phi"] - CREEP_RATE / self.rig.fps)
            else:
                kt["u"] = min(1.0, kt["u"] + 1.0 / (kt["dur"] * self.rig.fps))
                kt["phi"] = sgn * kt["span"] * smooth5(kt["u"])
            p, q = self.drive_pose(F_PRESS, kt["phi"])
            tau = abs(self.drive.history[-1]["tau"]) if self.drive.history else 0.0
            goal = False
            if self.mode == "tighten":
                if m.q <= SEAT_TOL:
                    self.seated = True
                    if kt["seat_phi"] is None:
                        kt["seat_phi"] = kt["phi"]
                    self.tau_seat = max(self.tau_seat, tau)
                    goal = tau >= SNUG_TAU or abs(kt["phi"] - kt["seat_phi"]) >= SNUG_OVER
            else:
                goal = m.q >= LOOSE_GOAL
            at_end = abs(kt["phi"]) >= kt["span"] - 1e-6
            self.phi_end = kt["phi"]
            if goal or at_end:
                self.strokes += 1
                self.stroke_log.append({"stroke": self.strokes, "screw_from_deg": round(np.degrees(kt["q0"]), 1),
                                        "screw_to_deg": round(np.degrees(m.q), 1),
                                        "wrist_deg": round(np.degrees(kt["phi"]), 1),
                                        "joint7_from_deg": round(np.degrees(kt["q7_0"]), 1),
                                        "joint7_to_deg": round(float(np.degrees(self.trk.q[6])), 1)})
                if goal or self.strokes >= MAX_STROKES:
                    self.goto("snug_hold" if self.mode == "tighten" else "end_unpress", t)
                else:
                    back = self.q7_room()[1] - UNWIND
                    n90 = int(np.floor(max(back, 0.0) / (np.pi / 2)))
                    self.k_swing = -sgn * n90 * np.pi / 2
                    self.goto("s_unpress", t)
        elif ph == "snug_hold":
            p, q = self.p_cmd, self.q_cmd
            if t - self.t0 >= T["snug_hold"]:
                if self.seated:
                    pre = float(np.clip(PRELOAD_FRAC * self.tau_seat, PRELOAD_MIN, PRELOAD_MAX))
                    self.thread.set_preload(pre, m.q)
                    print(f"    seated: seating torque {self.tau_seat*1000:.1f} mN.m -> breakaway preload "
                          f"{pre*1000:.1f} mN.m", flush=True)
                self.goto("end_unpress", t)
        elif ph == "s_unpress":
            u = smooth5(self.s(t, T["s_unpress"]))
            p, q = self.drive_pose(0.2, self.phi_end - sgn * UNWIND * u, gripped=False)
            if t - self.t0 >= T["s_unpress"]:
                self.goto("s_lift", t)
        elif ph == "s_lift":
            p0 = self.k.setdefault("p0", self.p_cmd.copy())
            u = smooth5(self.s(t, T["s_lift"]))
            lift = LIFT_OUT if self.k_swing != 0.0 else 0.004
            p = p0 + np.array([0.0, 0.0, lift * u])
            if t - self.t0 >= T["s_lift"]:
                self.goto("s_swing" if self.k_swing != 0.0 else "s_insert", t)
        elif ph == "s_swing":
            if "p0" not in self.k:
                self.k.update(p0=self.p_cmd.copy(), R0=R_of_q(self.q_cmd), c=m.tip.copy())
            dphi = self.k_swing
            dur = max(0.6, T["s_swing90"] * abs(dphi) / (np.pi / 2))
            u = smooth5(self.s(t, dur))
            R = expmap(np.array([0.0, 0.0, dphi * u]))
            c = self.k["c"]
            p = c + R @ (self.k["p0"] - c)
            q = quat_of_R(R @ self.k["R0"])
            if t - self.t0 >= dur:
                self.goto("s_insert", t)
        return p, q, f


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# run
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
LOG_KEYS = ("t", "q", "z", "tipx", "tipy", "tipz", "gx", "gy", "gz", "px", "py", "pz", "tilt", "flats", "tcpx", "tcpy",
            "tcpz", "padL", "padR", "table", "block", "robot_obj", "tau", "Fd", "Fz", "F_ax", "tau_f", "probe",
            "preload", "low", "q7", "hx", "hy", "hz", "phase")


def row_name(mode: str, screw: str) -> str:
    return f"{mode}_screw_with_screwdriver" + ("_longer_screw" if screw == "7.5" else "")


def parse(mode: str, argv=None):
    ap = argparse.ArgumentParser(description=f"{mode.capitalize()} screw with screwdriver (screwdriver in hand, kit)")
    ap.add_argument("--screw", default="6.5", choices=["6.5", "7.5"],
                    help="kit screw length in revolutions (7.5 = the 'Longer screw' variation)")
    ap.add_argument("--seed", type=int, default=None, help="draw the block's pose (paper protocol, module doc)")
    ap.add_argument("--name", default=None, help="output folder name")
    ap.add_argument("--check", action="store_true", help="build, check the IK window, hold the start 0.5 s, exit")
    return ap.parse_args(argv)


def tool_start(tcp, q_tcp):
    """World pose of the tool held at the start: axis along the TCP z (down), body x along the TCP x (the pads on
    the body-z flats), the TCP GRASP_D above the tip."""
    T_tcp = world_T(np.r_[tcp, q_tcp])
    H = np.eye(4)
    H[:3, :3] = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
    H[:3, 3] = [0.0, 0.0, GRASP_D + TIP_Y]
    return T_tcp @ H


def run(mode: str, argv=None):
    assert mode in ("tighten", "loosen")
    a = parse(mode, argv)
    name = a.name or row_name(mode, a.screw)
    draw = sample_start({"block": Placement(BOARD_XY, BLOCK_YAW, dxy=SEED_DXY, dyaw=SEED_DYAW, radius=0.045)},
                        a.seed)["block"]
    rig = appendix_rig(name)
    rig.collide_every_substep = True
    rig.hard_joint_limits = True
    rig.solver_options["impratio"] = IMPRATIO
    h = build_scene(rig, mode, a.screw, draw[:2], draw[2])
    b = rig.builder

    # arm start: the tool upright over the NOMINAL screw position, the hand yaw leaving joint 7 mid-way into the
    # window the first stroke starts from
    nominal = place_on_table("toy-assembly-block", xy=BOARD_XY, yaw=BLOCK_YAW)
    s_nom = (nominal @ np.r_[HOLE_CAD, BLOCK_TOP_CAD, 1.0])[:3]
    tcp0 = np.array([s_nom[0], s_nom[1], h["head_top0"] + START_TIP_ABOVE + GRASP_D])
    q7_want = (Q7_LO + Q7_MARGIN + np.pi / 4) if mode == "tighten" else (Q7_HI - Q7_MARGIN - np.pi / 4)
    yaw = 0.0
    for _ in range(3):                         # joint 7 and the hand yaw move one for one (opposite signs)
        q_ik = rig.ik.solve(tcp0, down_grasp_quat(yaw), iterations=64)
        yaw += float(q_ik[6]) - q7_want
    q_tcp0 = down_grasp_quat(yaw)
    q_start = rig.solve_arm_start(tcp0, q_tcp0)
    b.joint_q[7:9] = [F_TOUCH_W, F_TOUCH_W]
    b.joint_target_q[7:9] = [F_GRIP, F_GRIP]
    T_tool0 = tool_start(tcp0, q_tcp0)
    add_tool(rig, h, T_tool0)
    # the IK window over the seed square, at the stroke height: the hand turned through joint 7's whole working range
    # in 10 deg steps (warm-started, as the strokes do), at the nominal screw and the four corners (printed)
    z_turn = h["head_top0"] - 0.004 + GRASP_D
    for dx, dy in ((0.0, 0.0), (-SEED_DXY, -SEED_DXY), (SEED_DXY, SEED_DXY), (-SEED_DXY, SEED_DXY),
                   (SEED_DXY, -SEED_DXY)):
        pt = np.array([s_nom[0] + dx, s_nom[1] + dy, z_turn])
        rig.ik.solve(pt, down_grasp_quat(yaw), iterations=64)
        yw = yaw
        for _ in range(40):                    # down to the lower margin
            q_ik = rig.ik.solve(pt, down_grasp_quat(yw), iterations=24)
            if q_ik[6] <= Q7_LO + Q7_MARGIN:
                break
            yw += np.radians(10.0)
        lo7, worst = float(q_ik[6]), 0.0
        while True:                           # then up through the range to the upper margin
            q_ik = rig.ik.solve(pt, down_grasp_quat(yw), iterations=24)
            Tq = world_T(rig.ik.tcp_pose())
            err = np.linalg.norm(Tq[:3, 3] - pt) + 0.05 * np.linalg.norm(rotvec(R_of_q(down_grasp_quat(yw)) @ Tq[:3, :3].T))
            worst = max(worst, float(err))
            if q_ik[6] >= Q7_HI - Q7_MARGIN or yw < yaw - 2 * np.pi:
                break
            yw -= np.radians(10.0)
        print(f"[{name}] IK at ({pt[0]:.3f}, {pt[1]:.3f}): joint 7 {np.degrees(lo7):+.1f} .. {np.degrees(q_ik[6]):+.1f} deg "
              f"by turning the hand, worst pose error {worst*MM:.2f} (mm + 0.05 m x rad)", flush=True)
    rig.ik.solve(tcp0, q_tcp0, iterations=64)                   # warm start back at the start pose

    # the video's camera: in front of the board, opposite the robot (the person works from the far side), ~35 deg up
    cam_t = np.array([BOARD_XY[0], BOARD_XY[1] + 0.012, _env("SDK_CAM_TZ", 0.100)])
    el, az, dist = np.radians(_env("SDK_CAM_EL", 35.0)), np.radians(_env("SDK_CAM_AZ", 6.0)), _env("SDK_CAM_D", 0.60)
    cam_p = cam_t + dist * np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    rig.finalize(probes=["screwdriver"], cam_pos=cam_p, cam_target=cam_t)
    rig.solver = make_solver(rig.model, iterations=ITERATIONS, **rig.solver_options)
    model = rig.model

    qs = model.joint_q_start.numpy()
    cz, cq = int(qs[h["j_z"]]), int(qs[h["j_q"]])
    for state in (rig.s0, rig.s1):
        jq = state.joint_q.numpy()
        jq[cq] = h["q0"]
        jq[cz] = LEAD * h["q0"]
        state.joint_q.assign(jq)
        newton.eval_fk(model, state.joint_q, state.joint_qd, state)
    thread = ThreadModel(rig.solver, model, h["j_z"], h["j_q"], h["q0"], LOOSE_PRELOAD0 if mode == "loosen" else 0.0)
    rig.solver = thread

    tool_shapes = h["tool_shapes"]
    labels = list(model.shape_label)
    blk_lbls = [labels[s] for s in h["block_shapes"]]
    nut_lbls = [labels[s] for s in h["nut_shapes"]]
    rig.probes["padL"] = PairProbe(model, "fr3/fr3_leftfinger", tool_shapes)
    rig.probes["padR"] = PairProbe(model, "fr3/fr3_rightfinger", tool_shapes)
    rig.probes["tool_table"] = PairProbe(model, "screwdriver", ["table"])
    rig.probes["tool_block"] = PairProbe(model, "screwdriver", blk_lbls + nut_lbls)
    rig.probes["robot_obj"] = RobotContactProbe(model, ["screw"], static_shapes=blk_lbls + nut_lbls)
    drive = DriveSensor(model, tool_shapes, rig.shapes["screw"])

    meas = Meas(rig, h)
    trk = JointTracker(rig, q_start)

    # unrecorded pre-roll: the pads close on the handle (the tool is placed in the open-to-touch fingers)
    sim_dt = 1.0 / rig.fps / rig.substeps
    qt = rig.control.joint_target_q.numpy()
    qt[:7] = q_start[:7]
    qt[7:9] = F_GRIP
    rig.control.joint_target_q.assign(qt)
    for _ in range(int(0.3 * rig.fps)):
        for _ in range(rig.substeps):
            rig.s0.clear_forces()
            rig.pipeline.collide(rig.s0, rig.contacts)
            rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
            rig.s0, rig.s1 = rig.s1, rig.s0
    rig.solver.update_contacts(rig.contacts, rig.s0)
    for pr in rig.probes.values():
        pr.sample(rig.contacts, rig.s0, 0.0)
    meas.update(rig.s0)
    slip0 = float(np.linalg.norm(meas.p - T_tool0[:3, 3]))
    d_tcp = (np.linalg.inv(world_T(np.r_[tcp0, q_tcp0])) @ np.r_[meas.p, 1.0])[:3] - (np.linalg.inv(
        world_T(np.r_[tcp0, q_tcp0])) @ np.r_[T_tool0[:3, 3], 1.0])[:3]
    print(f"[{name}] pre-roll: pads {rig.probes['padL'].history[-1]['force']:.2f} / "
          f"{rig.probes['padR'].history[-1]['force']:.2f} N, tool moved {slip0*MM:.3f} mm (TCP frame "
          f"{np.round(d_tcp*MM, 3)}), tilt {meas.tilt:.2f} deg; "
          f"block at ({draw[0]:.4f}, {draw[1]:.4f}) yaw {np.degrees(draw[2]):.1f} deg; screw {a.screw} rev starts "
          f"{np.degrees(h['q0']):.1f} deg / {LEAD*h['q0']*MM:.2f} mm above the seat", flush=True)
    for pr in rig.probes.values():
        pr.history.clear()

    expert = Expert(rig, meas, thread, drive, trk, mode, h, tcp0, q_tcp0)
    stop_at = os.environ.get("SDK_STOP", "")
    log = []

    def last(nm, key="force"):
        hh = rig.probes[nm].history
        return hh[-1][key] if hh else 0.0

    if a.check:
        t_hold = 0.5
        for _ in range(int(t_hold * rig.fps)):
            meas.update(rig.s0)
            trk.step(tcp0, q_tcp0, expert.grip_cmd(last))
            for _ in range(rig.substeps):
                rig.s0.clear_forces()
                rig.pipeline.collide(rig.s0, rig.contacts)
                rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
                rig.s0, rig.s1 = rig.s1, rig.s0
            rig.solver.update_contacts(rig.contacts, rig.s0)
            for pr in rig.probes.values():
                pr.sample(rig.contacts, rig.s0, 0.0)
        meas.update(rig.s0)
        print(f"[{name}] --check: after {t_hold} s holding the start: pads {last('padL'):.2f} / {last('padR'):.2f} N, "
              f"tool moved {np.linalg.norm(meas.p - T_tool0[:3, 3])*MM:.3f} mm from where it was placed, tilt "
              f"{meas.tilt:.2f} deg, screw {np.degrees(meas.q):.2f} deg, tip {np.round(meas.tip*MM, 2)} mm, "
              f"recess {np.round(expert.recess_center()*MM, 2)} mm", flush=True)
        return None

    def control(t):
        meas.update(rig.s0)
        p, q, f = expert.step(t, last)
        if expert.finished or (stop_at and expert.phase == stop_at):
            raise StopRun
        trk.step(p, q, f)

    def extra(state, t):
        thread.refresh()
        mm_ = meas.update(state)
        c = drive.sample(rig.contacts, state, h["screw_xy"], t)
        row = dict(t=t, q=mm_.q, z=mm_.z, tipx=mm_.tip[0], tipy=mm_.tip[1], tipz=mm_.tip[2], gx=mm_.grasp[0],
                   gy=mm_.grasp[1], gz=mm_.grasp[2], px=mm_.p[0], py=mm_.p[1], pz=mm_.p[2], tilt=mm_.tilt,
                   flats=mm_.flats_yaw, tcpx=mm_.tcp[0], tcpy=mm_.tcp[1], tcpz=mm_.tcp[2], padL=last("padL"),
                   padR=last("padR"), table=last("tool_table"), block=last("tool_block"), robot_obj=last("robot_obj"),
                   tau=c["tau"], Fd=c["F"], Fz=c["Fz"], F_ax=thread.F_ax, tau_f=thread.tau_f, probe=thread.probe,
                   preload=thread.preload, low=mm_.low, q7=float(trk.q[6]), hx=mm_.in_tcp[0], hy=mm_.in_tcp[1],
                   hz=mm_.in_tcp[2], phase=expert.phase_id())
        log.append(row)
        k = int(round(t * rig.fps))
        if k % 15 == 0:
            print(f"  [{t:6.2f} {expert.phase:11s}] pads {row['padL']:5.2f}/{row['padR']:5.2f} drv {c['F']:5.2f}N "
                  f"tau {c['tau']*1000:6.2f}mNm q {np.degrees(mm_.q):7.1f} z {mm_.z*MM:5.2f} tilt {mm_.tilt:5.2f} "
                  f"tip {np.round(mm_.tip*MM, 1)} q7 {np.degrees(trk.q[6]):+6.1f} rob {row['robot_obj']:5.2f} "
                  f"blk {row['block']:5.2f} fric {thread.tau_f*1000:5.2f} pre {thread.preload*1000:4.1f} "
                  f"f {expert.f_cmd*MM:5.2f} err {expert.k.get('tool_err', '')}", flush=True)
        return {"padL_N": round(row["padL"], 3), "padR_N": round(row["padR"], 3),
                "tool_table_N": round(row["table"], 4), "tool_block_N": round(row["block"], 4),
                "drive_force_N": round(c["F"], 3), "drive_torque_mNm": round(c["tau"] * 1000, 3),
                "robot_object_contact_N": round(row["robot_obj"], 4), "screw_deg": round(float(np.degrees(mm_.q)), 3),
                "screw_rise_mm": round(mm_.z * MM, 4), "thread_friction_mNm": round(thread.tau_f * 1000, 3),
                "probe_torque_mNm": round(thread.probe * 1000, 3), "seat_preload_mNm": round(thread.preload * 1000, 3),
                "tool_tilt_deg": round(mm_.tilt, 3)}

    try:
        rec = rig.run(_env("SDK_BUDGET", 90.0), control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[{name}] stopped at t={len(rec.times) / rig.fps:.2f}s (phase {expert.phase})", flush=True)
    paths = rec.save("rollout", force_keys=["padL_N", "padR_N", "drive_force_N", "drive_torque_mNm",
                                            "robot_object_contact_N"])
    print("saved:", paths, flush=True)
    if not expert.finished:
        print("expert did not finish: no verdict", flush=True)
        np.save(rig.out_dir / "partial_log.npy", np.array([[r[k] for k in LOG_KEYS] for r in log]))
        return None
    return score(rig, h, expert, thread, log, paths, mode, a, draw, name)


def _env(name, default):
    v = os.environ.get(name, "")
    return type(default)(v) if v != "" else default


# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
# verdict
# ═════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
def score(rig, h, expert, thread, log, paths, mode, a, draw, name):
    from force_bench.appendix_tasks.registry import CRITERIA
    qa = rig.qa(paths)
    L = {k: np.array([r[k] for r in log], float) for k in LOG_KEYS}
    t = L["t"]
    ts = expert.t_start
    t_end = float(t[-1])
    crit = {}

    def win(lo, hi):
        return (t >= lo) & (t <= hi)

    crit["start_state"] = {
        "pass": True, "info_only": True, "seed": a.seed, "screw": f"{a.screw} revolutions",
        "block_xy_m": [round(float(draw[0]), 4), round(float(draw[1]), 4)],
        "block_yaw_deg": round(float(np.degrees(draw[2])), 2),
        "screw_axis_xy_m": [round(float(h["screw_xy"][0]), 4), round(float(h["screw_xy"][1]), 4)],
        "sampling": (f"block centre uniform in +-{SEED_DXY*MM:.0f} mm about ({BOARD_XY[0]}, {BOARD_XY[1]}) (the "
                     f"paper's 0.15 x 0.15 m square), yaw +-{np.degrees(SEED_DYAW):.0f} deg about "
                     f"{np.degrees(BLOCK_YAW):.0f} deg" if a.seed is not None else "nominal (no seed)"),
        "screw_start_deg_from_seat": round(float(np.degrees(L["q"][0])), 1),
        "screw_start_head_rise_mm": round(float(L["z"][0]) * MM, 3),
        "note": "the screwdriver starts in the hand, upright over the nominal screw position, tip "
                f"{START_TIP_ABOVE*MM:.0f} mm above the head"}

    t_probe = expert.t_probe or t_end
    pre = (t >= t_probe - 0.3) & (t < t_probe)
    q_deg = np.degrees(L["q"])
    ang_end = float(q_deg[pre].mean()) if pre.any() else float(q_deg[-1])
    t_end_turn = ts.get("end_unpress", t_end)
    turn_m = win(ts.get("s_turn", t_end_turn), t_end_turn)
    turning = turn_m & (L["phase"] == PHASES.index("s_turn"))
    tau_pk = float(np.abs(L["tau"][turning]).max()) if turning.any() else 0.0
    if mode == "tighten":
        crit["screw_tightened"] = {
            "pass": bool(expert.seated and ang_end < np.degrees(SEAT_TOL) + 0.5
                         and float(q_deg[0] - ang_end) > np.degrees(h["q0"]) - 2.0
                         and abs(float(L["z"][pre].mean())) < 0.0002),
            "start_deg_from_seat": round(float(q_deg[0]), 1),
            "start_head_rise_mm": round(float(L["z"][0]) * MM, 3),
            "turned_deg": round(float(q_deg[0] - ang_end), 1),
            "angle_from_seat_after_release_deg": round(ang_end, 3),
            "head_rise_after_release_mm": round(float(L["z"][pre].mean()) * MM, 4),
            "strokes": expert.strokes, "stroke_log": expert.stroke_log,
            "seating_torque_peak_mNm": round(expert.tau_seat * 1000, 2),
            "tip_torque_peak_incl_snug_hold_mNm": round(float(np.abs(L["tau"][turn_m]).max()) * 1000, 2)
            if turn_m.any() else None,
            "breakaway_preload_mNm": round(float(L["preload"][pre].mean()) * 1000, 2),
            "note": "threads unengaged at the start: the screw resting on the nut's thread, at the mean rest height "
                    "over the thread phase (kit-mesh drop test); seat = hard stop q = 0"}
        moves = {}
        for kk, lbl in ((0, "loosen_way"), (1, "tighten_way")):
            w = win(t_probe + kk * T_PROBE, t_probe + (kk + 1) * T_PROBE)
            moves[lbl] = round(float(np.ptp(q_deg[w])) if w.any() else 0.0, 4)
        crit["stationary_under_small_torque"] = {
            "pass": bool(max(moves.values()) < 1.0), "probe_torque_mNm": PROBE_TAU * 1000, "rotation_deg": moves,
            "threshold_deg": 1.0, "window_s": T_PROBE,
            "note": "applied to the screw dof once the robot had moved away and stopped; loosen-way held by the seat "
                    "preload model, tighten-way by the seat"}
    else:
        backed = float(ang_end - q_deg[0])
        crit["screw_loosened"] = {
            "pass": bool(backed >= 180.0 and expert.released and tau_pk > LOOSE_PRELOAD0),
            "start_deg_from_seat": round(float(q_deg[0]), 2),
            "backed_off_deg_after_release": round(backed, 1),
            "head_rise_after_release_mm": round(float(L["z"][pre].mean()) * MM, 3),
            "breakaway_preload_mNm": LOOSE_PRELOAD0 * 1000,
            "peak_drive_torque_at_tip_mNm": round(tau_pk * 1000, 2),
            "preload_released": bool(expert.released), "strokes": expert.strokes, "stroke_log": expert.stroke_log,
            "threshold_deg": 180.0}
        moves = {}
        for kk, lbl in ((0, "loosen_way"), (1, "tighten_way")):
            w = win(t_probe + kk * T_PROBE, t_probe + (kk + 1) * T_PROBE)
            moves[lbl] = round(float(abs(q_deg[w][-1] - q_deg[w][0])) if w.any() else 0.0, 3)
        crit["rotates_freely_under_small_torque"] = {
            "pass": bool(min(moves.values()) > 5.0), "probe_torque_mNm": PROBE_TAU * 1000,
            "rotation_deg_in_1s": moves, "threshold_deg": 5.0,
            "note": "applied to the screw dof once the robot had moved away and stopped"}

    # the tool did the turning, and stayed in the hand all along
    pads_on = float(((L["padL"][turning] > 0.5) & (L["padR"][turning] > 0.5)).mean()) if turning.any() else 0.0
    crit["turned_by_the_tool_in_hand"] = {
        "pass": bool(tau_pk > 0.003 and pads_on > 0.98),
        "peak_tip_torque_mNm": round(tau_pk * 1000, 2),
        "both_pads_loaded_fraction_while_turning": round(pads_on, 3),
        "strokes": expert.strokes, "insertions": expert.insert_log,
        "note": "the screw only turns through the tool tip in the recess; the tool only moves through the fingertip "
                "pads; between strokes it is lifted out in the grip, swung back in the air and re-inserted"}
    held = float(((L["padL"] > 0.5) & (L["padR"] > 0.5)).mean())
    rel = np.column_stack([L["hx"], L["hy"], L["hz"]])            # tool centre in the TCP frame
    slide = float(np.linalg.norm(rel - rel[0], axis=1).max())
    crit["screwdriver_kept_in_hand"] = {
        "pass": bool(held > 0.98 and float(L["table"].max()) < CONTACT_FREE_N and slide < 0.008),
        "both_pads_loaded_fraction": round(held, 4),
        "tool_table_contact_max_N": round(float(L["table"].max()), 4),
        "tool_centre_to_tcp_drift_max_mm": round(slide * MM, 2),
        "pad_force_mean_N": [round(float(L["padL"].mean()), 2), round(float(L["padR"].mean()), 2)],
        "note": "held by fingertip friction only (a free body, no joint or weld to the hand) from the start to the "
                "end; the person in the video also leaves with the tool"}

    # moved away, contact broken, robot stopped
    t_clear = ts.get("still", t_end - 1.0)
    clear = t >= t_clear
    tcp = np.column_stack([L["tcpx"], L["tcpy"], L["tcpz"]])
    i_off = int(np.searchsorted(t, ts.get("extract", t_end)))
    moved = float(np.linalg.norm(tcp[-1] - tcp[min(i_off, len(t) - 1)]))
    peak_rob = float(L["robot_obj"][clear].max()) if clear.any() else 9.9
    peak_tool = float(np.maximum(L["Fd"][clear], L["block"][clear]).max()) if clear.any() else 9.9
    crit["released_and_clear"] = {
        "pass": bool(peak_rob < CONTACT_FREE_N and peak_tool < CONTACT_FREE_N and moved >= 0.050),
        "window_s": [round(t_clear, 2), round(t_end, 2)],
        "robot_screw_block_nut_contact_peak_N": round(peak_rob, 4),
        "tool_screw_block_contact_peak_N": round(peak_tool, 4), "threshold_N": CONTACT_FREE_N,
        "tcp_moved_from_the_screw_mm": round(moved * MM, 1), "min_move_mm": 50.0,
        "note": "the hand keeps the screwdriver (as in the videos): neither the robot nor the tool touches the "
                "screw, the block or the nut"}
    d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
    labels = list(d["body_labels"])
    fm = d["t"] >= t_clear
    arm_ix = [i for i, lbl in enumerate(labels) if str(lbl).startswith("fr3/")]
    v_arm = float(np.linalg.norm(np.diff(d["body_q"][fm][:, arm_ix, :3], axis=0), axis=-1).max()) * rig.fps \
        if fm.sum() > 1 else 9.9
    crit["robot_stopped"] = {"pass": bool(v_arm < 0.005), "max_link_speed_final_window_mm_s": round(v_arm * MM, 3),
                             "threshold_mm_s": 5.0}
    off = t >= ts.get("retreat", t_end)
    crit["tool_off_screw"] = {
        "pass": bool(L["Fd"][off].max() < CONTACT_FREE_N) if off.any() else False,
        "tool_screw_contact_after_extraction_peak_N": round(float(L["Fd"][off].max()), 4) if off.any() else None,
        "tip_above_head_at_end_mm": round(float(L["tipz"][-1] - (h["S0"][2] + HEAD_H + L["z"][-1])) * MM, 1)}

    for k_, c_ in crit.items():
        qa.report["checks"][k_] = c_
    # penetration: tool tip <-> screw (both ways), tool -> block halves and nut, tool -> table top
    drv_local = h["drv_tm"]
    scr_local = h["scr_tm"].copy()
    tip_pts = drv_local.vertices[drv_local.vertices[:, 1] < TIP_Y + 0.008]
    head_pts = scr_local.vertices[scr_local.vertices[:, 2] > 0.0005]
    qa.check_penetration(
        [("screwdriver", trimesh.Trimesh(vertices=tip_pts, process=False), "screw", scr_local),
         ("screw", trimesh.Trimesh(vertices=head_pts, process=False), "screwdriver", drv_local)], tol=0.0015)
    kit = load_kit()
    for part in kit["toy-assembly-block"].parts():
        tm = part.mesh()
        tm.apply_transform(h["T_blk"])
        qa.check_penetration_static("screwdriver", drv_local, tm, np.zeros(3), f"screwdriver->{part.name}", tol=0.0015)
    qa.check_envelope(["screwdriver", "screw"], np.array([0.25, -0.30, -0.01]), np.array([0.80, 0.30, 0.40]))
    qa.report["checks"]["expert_info"] = {
        "pass": True, "info_only": True, "phase_start_s": {k: round(v, 2) for k, v in ts.items()},
        "insertions": expert.insert_log, "thread_friction_peak_mNm": round(thread.peak_fric * 1000, 2),
        "joint7_range_deg": [round(float(np.degrees(L["q7"].min())), 1), round(float(np.degrees(L["q7"].max())), 1)]}
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    title = "Tighten screw with screwdriver" if mode == "tighten" else "Loosen screw with screwdriver"
    spec = CRITERIA[15 if mode == "tighten" else 16]
    changes = [
        "kit scene (force_bench.kit): toy-assembly-block (both final halves), the kit nut captive under the "
        "right-hand hole, the kit screw " + f"({a.screw} revolutions)" + " in that hole, the kit screwdriver; the "
        "block's long axis left-right in the camera, screw on the camera's right, as in the videos",
        "the screwdriver starts in the hand (top-down pinch on two handle flats, 95 mm above the tip) and stays in "
        "it; the hand leaves with the tool, as the person does",
        ("start: threads unengaged -- the screw resting on the nut's thread (mean rest height over the phase), "
         f"{RISE[a.screw]*MM:.2f} mm ({np.degrees(h['q0']):.0f} deg) above the seat" if mode == "tighten" else
         "start: seated with a 20 mN.m breakaway preload"),
        "thread model: _screwdriver_table.ThreadModel (helix spring between the screw's axial and rotary dofs, "
        "load-dependent elasto-plastic thread friction, hard seat, seat preload from the measured seating torque)",
        "strokes: joint 7's range (up to ~317 deg per stroke); between strokes the tool is lifted out in the grip, "
        "swung back in the air and re-inserted",
        f"probe: +-{PROBE_TAU*1000:.0f} mN.m on the screw once the robot had moved away and stopped",
    ]
    return save_appendix_verdict(rig, task=title + (" (longer screw)" if a.screw == "7.5" else ""),
                                 source="new task (kit assets); thread model of _screwdriver_table",
                                 spec=spec, criteria=crit, physics_qa=report, changes=changes)
