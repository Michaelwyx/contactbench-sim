"""ContactBench task 26 "Plug in USB" -- USB sticks 1-4 into the final printable USB slot, with its three springs.

    Fully insert the USB stick held in hand into the USB slot placed in the workspace.  Success (paper Table C1):
    the USB stick is fully inserted into the slot, such that it does not noticeably move when further pushed toward
    the slot; the robot stops, moves away and breaks contact with the task objects.

ASSETS: the kit library (``force_bench.kit``).  The slot is ``usb-slot`` -- the final printable shell, keyed centre
and one printed spring part -- built by ``force_bench.kit.mechanisms.usb_slot`` (geometry, the jaw model, the
stiffness estimate and its provenance are documented there); ``--spring 0.2|0.4|0.6`` picks the spring variant
(the indent of its jaws; 0.2 mm, the softest, is the default).  The stick is ``usb-stick-N`` (``--usb 1..4``): a
FREE body with the kit's contact settings (0.12 mm SDF cells, 0.1 mm contact gap, PLA mu 0.4) and a 15 g
CONDITIONING mass (the kit's print estimates are 1.5 / 5.6 / 2.8 / 1.8 g; see the physics model).  Sticks 1-3
share one 12 x 5 mm plug on three grip bodies and seat on their shoulder and plug tip; stick 4 is a flat 12 x 5 mm
bar and seats on its tip alone.  The slot stands on the
table with its mouth up, turned so that its jaws close along world y and the gripper pinches the stick's wide faces
along world x (as a person holds it).

PHYSICS MODEL (what the scripted policy meets; nothing positions the stick but the two fingertips and the slot):

* the slot's mouth clears the plug by 0.175 mm per side; the key tongue runs in the plug's channel; the two jaws
  ride prismatic joints with the flexure's estimated stiffness (66 N/mm per jaw) and stand into the plug's path by
  the spring's indent, so a seated stick is clamped by ~12 / 25 / 37 N per jaw (0.2 / 0.4 / 0.6 mm, measured by
  ``force-bench kit check usb-slot``) and slides in against ~10 / 20 / 30 N of friction;
* the jaws are stiff and load the stick through a contact, so that contact must be stiffer still: under MuJoCo a
  contact's stiffness is capped by the timestep and scales with the mass of the body it acts on, and at the
  stick's 1.5 g print mass the stick sank 0.13-0.16 mm into the jaws, which opened only 0.07 of the 0.2 mm (4.6 N
  instead of ~12 N).  The stick therefore carries a 15 g conditioning mass (``STICK_MASS``; its weight, 0.15 N, is
  negligible against the pinch) and the jaw contacts are stiffened (``usb_slot.stiffen_jaw_contacts``); with both,
  the jaws open 0.17-0.19 mm and the lead-in peak is 12 N, as in the kit's joint-guided probe (64 substeps alone
  gave 0.13 mm at twice the cost);
* the pads close across the stick's thin axis on its wide faces, tips 2.5 mm above the shoulder (for stick 4: 2.5
  mm above the slot face once seated); pad mu 1.4 (MuJoCo takes a pair's mu as the MAX, so pad/PLA = 1.4); finger
  mass 0.12 kg;
* SolverMuJoCo, 32 substeps, 20 iterations, impratio 10 (the friction rows must be stiff for a gram-scale stick to
  stay in a pinch: at the default 1.0 it creeps out); contacts found before every substep (a stick that falls or is
  swung moves several mm per frame, more than the slot's 1 mm top wall);
* the PINCH is the force budget of the task: Coulomb capacity 2 x 1.4 x F_pad.  The base pinch is 12 N per pad
  (1.5 mm of overdrive on 8 kN/m fingers), 34 N of capacity; the 0.4 and 0.6 mm springs need more (``PINCH_N``).

EXPERT (privileged state): xy servo of the TCP on the MEASURED stick against its seat (start offset +0.6 / -0.8 mm,
``--offset-mm``), rate-limited and frozen once the plug is 0.5 mm into the mouth; insertion on a slew-limited speed
profile (12 mm/s in air, 3 mm/s from 3 mm above the mouth until the jaw lead-ins are passed, 2 mm/s for the last 3
mm); the command may lead the MEASURED HAND by a bounded lead -- the arm's ~90 kN/m in-scene stiffness turns the
lead into the pushing force, so the lead is sized from the spring's expected sliding force (``leads()``), plus a
wedge allowance while the plug opens the jaws.  Seat = within 0.15 mm of the CAD seat, or stalled within 1.5 mm
of it.  Seat hold bled to 4 N, push test (>= 8 N of slot reaction), release at the pushed pose, 6 cm retreat.

SEEDED STARTS (``--seed N``, ``force-bench run plug_in_usb_1 --seed N``): the slot's pose is drawn uniformly in the
paper's 0.15 x 0.15 m square at the board centre and +-20 deg of yaw about its nominal turn (``SLOT_PLACE``,
``common.sample_start``); the stick starts in the pinch above the slot wherever it lands, with the same offset from
the slot axis (world frame) and the pads closing across the stick's thin axis (the hand turned with the slot).  The
arm's start configuration is walked over from the nominal one (the same IK branch; a fresh solve at the robot-side
corner turned +20 deg lands with joint 7 on its limit).  The robot base stays where it is (0.42 m from the board
centre); over the square the arm keeps >= 0.18 rad of margin on every joint from the start through the insertion
and the retreat (the elbow is closest to its limit, at the robot-side edge).  The draw is recorded in the verdict
("start").

VERDICT (appendix_qa.json): held_in_hand_at_start, fully_inserted (within 0.5 mm of the CAD seat),
no_motion_when_pushed (< 0.3 mm gained under >= 6 N), stays_seated_after_release (< 0.3 mm), finger_pinch_real,
released_and_clear (robot <-> stick/slot < 0.05 N over the final window, hand moved >= 40 mm), start (information
only: the seed and the slot's pose); physics QA: finite, velocity spikes, penetration (0.4 mm) of the stick into the
exact CAD shell / centre / jaws and into the slot body behind its walls (``usb_slot.intrusion``, every frame), envelope.

Run:  python -m force_bench.appendix_tasks.plug_in_usb --usb 1 [--spring 0.6] [--offset-mm DX,DY] [--seed N]
      -> outputs/appendix_tasks/plug_in_usb_<N>[_spring04|_spring06]/
      --seed N  random slot pose (0.15 x 0.15 m square, +-20 deg); ``force-bench run`` names the folder <row>__seed<N>
      --check / --probe  grasp tests (hold; hold + 1 mm lateral move + 10 mm descent in air)
      --pinch-n N, --stick-mass-g G, --substeps N  reproduce the measurements above
"""

from __future__ import annotations

import argparse
import json

import numpy as np
import trimesh
import warp as wp

from force_bench.assets import set_body_mass
from force_bench.kit import add_asset, load_kit, place_on_table, pose_matrix
from force_bench.kit.mechanisms import usb_slot as U
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks.nut_bolt_arm import PairProbe
from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, ease, check_released_and_clear, save_appendix_verdict, StopRun, Placement,
    sample_start)

STICKS = (1, 2, 3, 4)
SPRINGS = ("0.2", "0.4", "0.6")

# ── solver ──────────────────────────────────────────────────────────────────────────────────────────────────────
SUBSTEPS = 32
ITERS = 20
IMPRATIO = 10.0             # stiff friction rows: a gram-scale stick holds in a 12 N pinch
STICK_MASS = 0.015          # kg, CONDITIONING mass of the stick (print estimates 1.5-5.6 g): MuJoCo's contact
                            # stiffness scales with the body's mass at this timestep, and at 1.5 g the stick sank
                            # 0.13-0.16 mm into the jaws, which opened 0.07 of 0.2 mm (4.6 N instead of ~12 N);
                            # at 15 g, with the jaw contacts stiffened (usb_slot.stiffen_jaw_contacts), 0.17-0.19
                            # mm and a 12.1 N lead-in peak, the kit probe's numbers.  Its weight (0.15 N) is
                            # negligible against the pinch.

# ── FR3 fingertip envelope (measured on the FR3 URDF) ─────────────────────────────────────────────────────────
TIP_DROP = 0.00993          # m, collision tip below the TCP point
PAD_OFF = 0.000085          # pad face = finger dof - this
FINGER_KE, FINGER_KD = 8000.0, 60.0
PINCH_N = {"0.2": 12.0, "0.4": 18.0, "0.6": 24.0}   # N per pad (overdrive = PINCH_N / FINGER_KE): Coulomb
                            # capacity 2 x 1.4 x PINCH_N = 34 / 50 / 67 N against insertion peaks of ~12 / 27 / 43 N
FINGER_F_MAX = 25.0         # N per pad (MuJoCo forcerange); the FR3 hand grips with up to 70 N
PAD_MU = 1.4
FINGER_MASS = 0.12
TIP_ABOVE_SHOULDER = 0.0025  # pad tips this far above the stick's shoulder (2.5 mm above the slot face, seated)
OPEN_EXTRA = 0.0045         # release: each pad 4.5 mm off the body

# ── trajectory ──────────────────────────────────────────────────────────────────────────────────────────────────
RAISE = 0.025               # start: stick 25 mm above its seat (plug tip 13 mm above the mouth)
V_INS = 0.012               # m/s in free air and along the jaws' flat
V_ENTRY = 0.003             # m/s through the mouth and the jaw lead-ins
ENTRY_TIP = (-0.0055, 0.0030)  # plug-tip height window rel. the mouth (slow): the lead-ins end ~5.2 mm in
SEAT_ZONE = 0.0030
V_SEAT = 0.002              # m/s for the last SEAT_ZONE
ACC = 0.06                  # m/s^2 slew on the commanded speed (velocity steps jerk the stick through the pinch)
K_ARM = 90000.0             # N/m, the arm's in-scene stiffness at this reach (lead -> pushing force)
LEAD_MIN = 0.00025          # m the command may lead the measured hand (free air, light springs)
LEAD_SEAT_MIN = 0.00015     # ... at the seat (the lead is the seat-impact bound)
LEAD_WEDGE = 0.00035        # extra lead while the plug wedges the jaws open
LEAD_MARGIN = 1.3           # lead >= this x (expected sliding force) / K_ARM
JAW_TIP = (-0.0055, -0.0020)  # tip depth window of the jaw lead-ins
OVERTRAVEL = 0.0020          # m the hand command may go past the nominal seat: under the stiff springs the stick
                            # creeps up in the pinch (measured 0.9-2.2 mm at ~30 N on the 0.6 mm spring) and the hand
                            # must follow; 2 mm still leaves the pad tips 0.5 mm above the slot face
SEAT_TOL = 0.00015
F_SEAT_HOLD = 4.0           # N the seat press is bled to
R_SEAT = 0.000015           # m/frame of the seat servo (0.9 mm/s)
PUSH_F = 8.0                # N of slot reaction the push test must reach
R_PUSH = 0.000012           # m/frame of extra lead (0.7 mm/s)
PUSH_LEAD_MAX = 0.0012
PUSH_HOLD = 0.6             # s held at the push force
ALIGN_KP, ALIGN_DB, ALIGN_CLAMP, ALIGN_ALPHA = 0.25, 0.00003, 0.004, 0.3
ALIGN_STEP = 0.00003        # m/frame cap on the align correction (1.8 mm/s)
ALIGN_FREEZE_TIP = -0.0005  # freeze once the tip is 0.5 mm into the mouth
PRE_ROLL = 0.30
T_GRIP0 = 0.5               # recorded: holding the stick above the slot
T_ALIGN = 0.6
T_INS_MAX = 8.0
T_HOLD = 0.6
T_PUSH_MAX = 2.5
T_OPEN, T_RETREAT, T_STILL = 0.6, 1.4, 1.2
RETREAT = 0.06

# appendix thresholds
SEAT_TOL_APPX = 0.0005
PUSH_MOVE_MAX = 0.0003
STAY_MAX = 0.0003
PEN_TOL = 0.0004

ROBOT_BASE = (0.0, 0.42, 0.0)
SLOT_XY = (0.0, 0.0)
SLOT_YAW = np.pi / 2        # slot x (the jaws' axis) -> world y; slot y (the stick's thin axis) -> world -x
SLOT_PLACE = Placement(SLOT_XY, yaw=SLOT_YAW, dxy=0.075, dyaw=np.radians(20.0), radius=0.03)   # --seed draws
# primary camera (the website clip), relative to the mouth centre: 3/4 view from the robot's front-right
CAM_OFF = np.array([0.060, -0.150, 0.075])
CAM_TGT = np.array([0.0, 0.0, 0.018])


def row_name(usb: int, spring: str) -> str:
    return f"plug_in_usb_{usb}" + ("" if spring == "0.2" else f"_spring{spring.replace('.', '')}")


def leads(f_slide: float) -> dict:
    """Lead budget [m] for an expected sliding force [N]: the push the arm can deliver is ~K_ARM x lead."""
    ins = max(LEAD_MIN, LEAD_MARGIN * f_slide / K_ARM)
    return {"ins": ins, "jaw": ins + LEAD_WEDGE, "seat": max(LEAD_SEAT_MIN, LEAD_MARGIN * f_slide / K_ARM)}


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--usb", type=int, choices=STICKS, default=1)
    ap.add_argument("--spring", type=str, choices=SPRINGS, default="0.2",
                    help="slot spring variant: the jaws' indent in mm (insertion force)")
    ap.add_argument("--offset-mm", type=str, default="0.6,-0.8",
                    help="start xy offset of the stick from the slot axis (mm), the misalignment the expert must "
                         "align away")
    ap.add_argument("--pinch-n", type=float, default=None, help="pad force [N] (default: PINCH_N per spring)")
    ap.add_argument("--seed", type=int, default=None, help="random slot pose (0.15 x 0.15 m square, +-20 deg)")
    ap.add_argument("--name", type=str, default=None, help="output folder name")
    ap.add_argument("--substeps", type=int, default=SUBSTEPS, help="solver substeps per frame (default 32)")
    ap.add_argument("--stick-mass-g", type=float, default=None,
                    help="stick mass [g] instead of the 15 g conditioning mass (e.g. the print estimate, to "
                         "reproduce the contact-compliance measurement in the docstring)")
    ap.add_argument("--check", action="store_true", help="build, settle 0.5 s, print the grasp and exit")
    ap.add_argument("--probe", action="store_true",
                    help="grasp probe: hold, 1 mm lateral move, 10 mm descent in air; prints the slip and exits")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    stick_key = f"usb-stick-{a.usb}"
    spring_label = f"{a.spring}mm indent spring"
    name = a.name or row_name(a.usb, a.spring)
    off_xy = np.array([float(v) for v in a.offset_mm.split(",")]) / 1000.0
    pinch_n = float(a.pinch_n if a.pinch_n is not None else PINCH_N[a.spring])
    start = sample_start({"slot": SLOT_PLACE}, a.seed)["slot"]          # nominal (SLOT_XY, SLOT_YAW) without --seed
    kit = load_kit()
    st_tm = kit[stick_key].parts()[0].mesh()                    # stick CAD frame, m
    x_tip, y_c, z_c = U.plug_reference(st_tm)
    half_t = 0.5 * float(st_tm.extents[2])                      # across the pinch: the stick's thin (CAD z) axis
    c_cad = 0.5 * (st_tm.bounds[0] + st_tm.bounds[1])

    rig = appendix_rig(name)
    rig.substeps = int(a.substeps)
    # collide before every substep (docs/physics_notes.md rules 5.1-5.3), set at build time so it holds whoever drives
    # the scene: a stick that falls or is swung moves several mm per frame, more than the slot's 1 mm top wall, and with
    # contacts found once per frame (0.1 mm contact gap) it could cross the wall between two contact updates
    rig.collide_every_substep = True
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
    rig.table(center_xy=(0.0, 0.16), half=0.35)

    # ── the stick (free) and the slot (mechanism), from the kit library ─────────────────────────────────────────
    slot_pose = place_on_table("usb-slot", spring_label, xy=(start[0], start[1]), yaw=float(start[2]))
    T_seat = U.seat_pose(stick_key, slot_pose)                 # world <- stick CAD frame, fully inserted
    T_start = pose_matrix(xyz=(off_xy[0], off_xy[1], RAISE)) @ T_seat
    hs = add_asset(rig, stick_key, pose=T_start, label="usb_stick")
    hslot = add_asset(rig, "usb-slot", spring_label, pose=slot_pose, label="usb_slot")
    sb = hs.bodies[""]
    stick_sid = hs.shapes[kit[stick_key].parts()[0].name][0]
    shell_sid = hslot.shapes["usb_slot_shell"][0]
    center_sid = hslot.shapes["usb_slot_center"][0]
    jaw_names = ("jaw_right", "jaw_left")
    jaw_sids = [hslot.shapes[n][0] for n in jaw_names]
    stick_mass_print = float(rig.builder.body_mass[sb])
    set_body_mass(rig.builder, sb, STICK_MASS if a.stick_mass_g is None else a.stick_mass_g * 1e-3)
    stick_mass = float(rig.builder.body_mass[sb])
    seat_body = (T_seat @ np.r_[c_cad, 1.0])[:3]               # stick body origin at the seat
    p_shoulder = (T_start @ np.r_[x_tip - U.PLUG_LEN, y_c, z_c, 1.0])[:3]
    jaw_force = float(hslot.params["jaw_force_seated_N"])
    mu = float(hslot.params["mu"])
    f_slide = 2.0 * mu * jaw_force                              # expected sliding friction on the jaws' flats
    L = leads(f_slide)

    # ── collision filters: the hand meets the stick (fingertips) and the shell, never the centre or the jaws ───
    B = rig.builder
    hand_body = B.body_label.index("fr3/fr3_link7")
    robot_shapes = [s for bl in range(0, hand_body + 3) for s in B.body_shapes.get(bl, [])]
    for s in robot_shapes:
        for t in (center_sid, *jaw_sids):
            B.add_shape_collision_filter_pair(s, t)

    # ── arm start: pads on the stick's wide faces, closing along world x ────────────────────────────────────────
    tcp0 = np.array([p_shoulder[0], p_shoulder[1], p_shoulder[2] + TIP_ABOVE_SHOULDER + TIP_DROP])
    q1 = down_grasp_quat(float(start[2]))                       # fingers close along the stick's thin axis
                                                                # (world x at the nominal SLOT_YAW = pi/2)
    if a.seed is not None:
        # walk the IK over from the nominal start's solution: the same arm branch over the whole square (a fresh
        # solve at the robot-side corner turned +20 deg lands with joint 7 on its limit)
        nom_pose = place_on_table("usb-slot", spring_label, xy=SLOT_XY, yaw=SLOT_YAW)
        T_nom = pose_matrix(xyz=(off_xy[0], off_xy[1], RAISE)) @ U.seat_pose(stick_key, nom_pose)
        tcp_nom = (T_nom @ np.r_[x_tip - U.PLUG_LEN, y_c, z_c, 1.0])[:3] + [0.0, 0.0, TIP_ABOVE_SHOULDER + TIP_DROP]
        rig.ik.solve(tcp_nom, down_grasp_quat(SLOT_YAW), iterations=64)
        for u in np.linspace(0.0, 1.0, 16):
            rig.ik.solve(tcp_nom + u * (tcp0 - tcp_nom), down_grasp_quat(SLOT_YAW + u * (start[2] - SLOT_YAW)),
                         iterations=32)
    rig.solve_arm_start(tcp0, q1)
    f_touch = half_t + PAD_OFF - 0.0001
    f_pinch = f_touch - pinch_n / FINGER_KE
    f_open = half_t + PAD_OFF + OPEN_EXTRA
    B.joint_q[7:9] = [f_touch, f_touch]
    B.joint_target_q[7:9] = [f_pinch, f_pinch]
    rig._filter_arm_from(sb, filter_fingers=False)
    rig.hard_joint_limits = True                               # the jaw travel stops

    mouth = slot_pose[:3, 3] + slot_pose[:3, :3] @ np.array([0.0, 0.0, U.MOUTH_Z])
    rig.finalize(probes=["usb_stick"], cam_pos=mouth + CAM_OFF, cam_target=mouth + CAM_TGT)
    rig.solver = make_solver(rig.model, iterations=ITERS, impratio=IMPRATIO)
    U.stiffen_jaw_contacts(rig, hslot)                # after make_solver, which resets every contact stiffness
    jaw_labels = [f"usb_slot/{n}" for n in jaw_names]
    rig.probes["stick_slot"] = PairProbe(rig.model, "usb_stick",
                                         counterpart_shapes=[shell_sid, center_sid, *jaw_sids])
    rig.probes["stick_seat"] = PairProbe(rig.model, "usb_stick", counterpart_shapes=[shell_sid, center_sid])
    for lbl in jaw_labels:
        rig.probes[lbl] = PairProbe(rig.model, lbl, counterpart_shapes=[stick_sid])
    rig.probes["finL"] = PairProbe(rig.model, "fr3/fr3_leftfinger", counterpart_shapes=[stick_sid])
    rig.probes["finR"] = PairProbe(rig.model, "fr3/fr3_rightfinger", counterpart_shapes=[stick_sid])
    shape_lbl = list(rig.model.shape_label)
    rig.probes["robot_object"] = RobotContactProbe(
        rig.model, ["usb_stick", *jaw_labels], static_shapes=[shape_lbl[shell_sid], shape_lbl[center_sid]])
    si = rig.body_index("usb_stick")
    lf_i = rig.body_index("fr3/fr3_leftfinger")
    rf_i = rig.body_index("fr3/fr3_rightfinger")
    q_start = rig.model.joint_q_start.numpy()
    jaw_q = [int(q_start[hslot.joints[n]]) for n in jaw_names]

    # unrecorded pre-roll: the pads close to the working pinch on the body
    rig.drive_arm(tcp0, q1, fingers=f_pinch)
    sim_dt = 1.0 / rig.fps / rig.substeps
    for _ in range(int(PRE_ROLL * rig.fps)):
        rig.s0.clear_forces()
        rig.pipeline.collide(rig.s0, rig.contacts)
        for _ in range(rig.substeps):
            rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
            rig.s0, rig.s1 = rig.s1, rig.s0

    def stick_q() -> np.ndarray:
        return rig.s0.body_q.numpy()[si]

    def depth() -> float:
        """Stick height above its seat (0 = fully inserted)."""
        return float(stick_q()[2] - seat_body[2])

    def last_vec(name) -> np.ndarray:
        h = rig.probes[name].history
        return np.array(h[-1]["vec"]) if h else np.zeros(3)

    def last(name) -> float:
        h = rig.probes[name].history
        return h[-1]["force"] if h else 0.0

    def seat_force() -> float:
        """Upward reaction of the slot on the stick (N), smoothed."""
        h = rig.probes["stick_slot"].history[-3:]
        return float(np.mean([r["vec"][2] for r in h])) if h else 0.0

    def jaw_open(state=None) -> list[float]:
        q = (rig.s0 if state is None else state).joint_q.numpy()
        return [float(q[i]) for i in jaw_q]

    if a.check:
        bq = rig.s0.body_q.numpy()
        print(json.dumps({
            "stick_mass_g": round(stick_mass * 1000, 2),
            "stick_dz_mm": round(depth() * 1000, 3),
            "stick_xy_off_mm": np.round((bq[si, :2] - seat_body[:2]) * 1000, 3).tolist(),
            "pads_N": [round(last("finL"), 2), round(last("finR"), 2)],
            "jaw_open_mm": [round(v * 1000, 3) for v in jaw_open()],
            "leads_mm": {k: round(v * 1e3, 3) for k, v in L.items()}, "f_slide_expected_N": round(f_slide, 2)}))

    # ── control ─────────────────────────────────────────────────────────────────────────────────────────────────
    st = {"phase": "hold0", "t0": 0.0, "z": 0.0, "ramp": 0.0, "align": np.zeros(2), "err": np.zeros(2),
          "frozen": False, "push": {}, "rel_tcp": None, "t_seat": None, "hist": [], "t_release": None,
          "t_retreat_end": None, "t_end": None, "off": None, "v": 0.0}
    frame_dt = 1.0 / rig.fps
    seat_xy = seat_body[:2]

    def align_step():
        if st["frozen"]:
            return
        e = seat_xy - stick_q()[:2].astype(float)
        st["err"] = ALIGN_ALPHA * e + (1 - ALIGN_ALPHA) * st["err"]
        step = np.where(np.abs(st["err"]) > ALIGN_DB, ALIGN_KP * st["err"], 0.0)
        step = np.clip(step, -ALIGN_STEP, ALIGN_STEP)
        st["align"] = np.clip(st["align"] + step, -ALIGN_CLAMP, ALIGN_CLAMP)

    def cmd_pos() -> np.ndarray:
        return tcp0 + np.array([st["align"][0], st["align"][1], st["z"]])

    def go(phase, t):
        st["phase"], st["t0"] = phase, t
        print(f"[usb] t={t:5.2f}s -> {phase}  dz={depth()*1000:+.3f} mm  seatF={seat_force():.2f} N  "
              f"jaws={[round(v*1000,3) for v in jaw_open()]} mm")

    def seat_servo():
        Fs = seat_force()
        if Fs > F_SEAT_HOLD + 1.0:
            st["z"] = min(st["z"] + R_SEAT, -RAISE + 0.004)
        elif Fs < 0.5 * F_SEAT_HOLD:
            st["z"] = max(st["z"] - R_SEAT, -RAISE - OVERTRAVEL)

    def hand_cmd() -> float:
        """Measured finger-midpoint height in command coordinates (offset fixed when the insertion starts)."""
        bq = rig.s0.body_q.numpy()
        hz = 0.5 * float(bq[lf_i, 2] + bq[rf_i, 2])
        if st["off"] is None:
            st["off"] = st["z"] - hz
        return hz + st["off"]

    def control(t):
        dz = depth()
        st["hist"].append((t, dz))
        ph = st["phase"]
        tip = dz - U.PLUG_LEN                  # plug tip height above the mouth
        if ph == "hold0":
            align_step()
            if t >= T_GRIP0:
                go("align", t)
        elif ph == "align":
            align_step()
            if t >= st["t0"] + T_ALIGN and np.all(np.abs(st["err"]) < 0.00008):
                go("insert", t)
            elif t >= st["t0"] + 2.5 * T_ALIGN:
                go("insert", t)
        elif ph == "insert":
            if not st["frozen"]:
                align_step()
                if tip < ALIGN_FREEZE_TIP:
                    st["frozen"] = True
                    print(f"[usb] t={t:5.2f}s align frozen at {np.round(st['align']*1000, 3).tolist()} mm, "
                          f"stick xy err {np.round((seat_xy - stick_q()[:2])*1000, 3).tolist()} mm")
            v_des = V_INS
            if ENTRY_TIP[0] < tip < ENTRY_TIP[1]:
                v_des = V_ENTRY
            lead = L["jaw"] if JAW_TIP[0] < tip < JAW_TIP[1] else L["ins"]
            if dz < SEAT_ZONE:
                v_des, lead = V_SEAT, L["seat"]
            # slew-limited speed; braking is looked ahead so the slow zones are entered at their speed
            brake = 0.5 * st["v"] ** 2 / ACC
            if (tip - brake < ENTRY_TIP[1] and tip > ENTRY_TIP[0]) or dz - brake < SEAT_ZONE:
                v_des = min(v_des, V_ENTRY if dz - brake >= SEAT_ZONE else V_SEAT)
            st["v"] += float(np.clip(v_des - st["v"], -ACC * frame_dt, ACC * frame_dt))
            v = st["v"]
            # LEAD BOUND ON THE MEASURED HAND (not on the stick): the command may lead the hand by at most
            # `lead`, i.e. the arm presses with at most ~K_ARM x lead.  Bounding on the stick instead runs away
            # upward once the stick slips in the pinch.
            hc = hand_cmd()
            st["ramp"] = max(st["ramp"] - v * frame_dt, hc - lead, -(RAISE + OVERTRAVEL))
            st["z"] = st["ramp"]
            seated = dz < SEAT_TOL
            if not seated and dz < 0.0015:
                h = [d for (tt, d) in st["hist"] if tt > t - 0.15]
                seated = len(h) >= 8 and (max(h) - min(h)) < 0.00008
            if seated or t >= st["t0"] + T_INS_MAX:
                st["t_seat"] = t
                st["z"] = hand_cmd() - L["ins"] * 0.5
                go("hold", t)
        elif ph == "hold":
            seat_servo()
            if t >= st["t0"] + T_HOLD:
                st["push"] = {"lead": 0.0, "z0": st["z"], "dz0": dz, "dz_min": dz, "F_peak": 0.0, "t_reached": None}
                go("push", t)
        elif ph == "push":
            P = st["push"]
            F = seat_force()
            P["F_peak"] = max(P["F_peak"], F)
            P["dz_min"] = min(P["dz_min"], dz)
            if F < PUSH_F:
                P["lead"] = min(P["lead"] + R_PUSH, PUSH_LEAD_MAX)
            elif P["t_reached"] is None:
                P["t_reached"] = t
            st["z"] = P["z0"] - P["lead"]
            if (P["t_reached"] is not None and t >= P["t_reached"] + PUSH_HOLD) or t >= st["t0"] + T_PUSH_MAX:
                st["rel_tcp"] = cmd_pos()
                st["t_release"] = t
                go("open", t)
        elif ph == "open":
            u = ease((t - st["t0"]) / T_OPEN)
            rig.drive_arm(st["rel_tcp"], q1, fingers=f_pinch + (f_open - f_pinch) * u)
            if t >= st["t0"] + T_OPEN:
                go("retreat", t)
            return
        elif ph == "retreat":
            u = ease((t - st["t0"]) / T_RETREAT)
            rig.drive_arm(st["rel_tcp"] + np.array([0.0, 0.0, RETREAT * u]), q1, fingers=f_open)
            if t >= st["t0"] + T_RETREAT:
                st["t_retreat_end"] = t
                go("still", t)
            return
        elif ph == "still":
            rig.drive_arm(st["rel_tcp"] + np.array([0.0, 0.0, RETREAT]), q1, fingers=f_open)
            if t >= st["t0"] + T_STILL:
                st["t_end"] = t
                raise StopRun
            return
        rig.drive_arm(cmd_pos(), q1, fingers=f_pinch)

    log = []
    PHASES = {"hold0": 0, "align": 1, "insert": 2, "hold": 3, "push": 4, "open": 5, "retreat": 6, "still": 7}

    def extra(state, t):
        bq = state.body_q.numpy()
        dz = float(bq[si, 2] - seat_body[2])
        xy = (bq[si, :2] - seat_xy)
        sF = last_vec("stick_slot")
        seatF = last_vec("stick_seat")
        fL, fR = last("finL"), last("finR")
        jF = [last(lbl) for lbl in jaw_labels]
        jo = jaw_open(state)
        rc = last("robot_object")
        hz = float(0.5 * (bq[lf_i, 2] + bq[rf_i, 2]))
        tip_gap = float(min(bq[lf_i, 2], bq[rf_i, 2]))
        q = bq[si, 3:7]
        x, y, z, w = q                                     # the plug axis = the stick body's -x (CAD x points down)
        ax = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
        tilt = float(np.degrees(np.arccos(np.clip(-ax[2], -1, 1))))
        log.append((t, dz, xy[0], xy[1], sF[2], seatF[2], fL, fR, jF[0], jF[1], jo[0], jo[1], rc, hz,
                    PHASES[st["phase"]], tilt, st["z"], tip_gap))
        return {"stick_dz_mm": round(dz * 1000, 3),
                "slot_axial_N": round(float(sF[2]), 3), "seat_axial_N": round(float(seatF[2]), 3),
                "finL_N": round(fL, 3), "finR_N": round(fR, 3),
                "jaw_R_N": round(jF[0], 3), "jaw_L_N": round(jF[1], 3),
                "jaw_R_open_mm": round(jo[0] * 1000, 4), "jaw_L_open_mm": round(jo[1] * 1000, 4),
                "stick_dx_mm": round(float(xy[0]) * 1000, 4), "stick_dy_mm": round(float(xy[1]) * 1000, 4),
                "stick_tilt_deg": round(tilt, 3), "robot_object_contact_N": round(rc, 4),
                "cmd_dz_mm": round(st["z"] * 1000, 3)}

    budget = T_GRIP0 + 2.5 * T_ALIGN + T_INS_MAX + T_HOLD + T_PUSH_MAX + T_OPEN + T_RETREAT + T_STILL + 0.5
    if a.check:
        budget = 0.5
    if a.probe:
        def control(t):                         # noqa: F811 (probe only)
            lat = 0.001 * ease((t - 0.5) / 0.6)
            dzc = -0.010 * ease((t - 1.6) / 1.0)
            rig.drive_arm(tcp0 + np.array([-lat, 0.0, dzc]), q1, fingers=f_pinch)
        budget = 3.2
    try:
        rec = rig.run(budget, control, extra)
    except StopRun:
        rec = rig.rec
        print(f"[usb] ending complete -> stopped at t={len(rec.times) / rig.fps:.2f}s")
    paths = rec.save("rollout", force_keys=["slot_axial_N", "finL_N", "finR_N", "jaw_R_N", "jaw_L_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)
    if a.check or a.probe:
        Lg = np.array(log)
        print("check: dz drift", round((Lg[-1, 1] - Lg[0, 1]) * 1000, 3), "mm; pads",
              round(Lg[:, 6].mean(), 2), round(Lg[:, 7].mean(), 2), "N; jaws open",
              np.round(Lg[-1, 10:12] * 1000, 3).tolist())
        slip = ((Lg[:, 1] + seat_body[2]) - Lg[:, 13]) - ((Lg[0, 1] + seat_body[2]) - Lg[0, 13])
        marks = {f"{tt:.1f}s": round(float(slip[min(int(tt * rig.fps), len(slip) - 1)]) * 1000, 3)
                 for tt in (0.5, 1.1, 1.6, 2.6, 3.1)}
        print("probe: stick slip in the pinch (mm, - = down)", json.dumps(marks),
              "max tilt", round(float(Lg[:, 15].max()), 2), "deg")
        return None

    # ── QA ──────────────────────────────────────────────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    Lg = np.array(log)
    (t_a, dz_a, dx_a, dy_a, sF_a, seatF_a, gL, gR, jR, jL, joR, joL, rc_a, hz_a,
     ph_a, tilt_a, cmd_a, tipz_a) = (Lg[:, k] for k in range(Lg.shape[1]))
    hold0 = ph_a == 0
    drop0 = float(dz_a[hold0].max() - dz_a[hold0].min()) if hold0.any() else 0.0
    grip0 = float(min(gL[hold0].mean(), gR[hold0].mean())) if hold0.any() else 0.0
    ins, hold, push = ph_a == 2, ph_a == 3, ph_a == 4
    t_rel = st["t_release"] if st["t_release"] is not None else t_a[-1]
    hw = hold & (t_a >= (t_a[hold].max() - 0.25 if hold.any() else 0))
    seat_dz = float(np.median(dz_a[hw])) if hw.any() else float("nan")
    P = st["push"] or {"dz0": seat_dz, "dz_min": seat_dz, "F_peak": 0.0}
    gained = float(P["dz0"] - P["dz_min"]) if P else float("nan")
    pre_rel = (t_a >= t_rel - 0.2) & (t_a < t_rel)
    dz_pre = float(np.mean(dz_a[pre_rel])) if pre_rel.any() else float("nan")
    final_dz = float(np.median(dz_a[-20:]))
    ins_peak = float(np.max(sF_a[ins])) if ins.any() else 0.0
    grip_ins = {"finL": float(gL[ins].mean()) if ins.any() else 0.0,
                "finR": float(gR[ins].mean()) if ins.any() else 0.0}
    tilt_max = float(np.max(tilt_a[ins | hold | push])) if (ins | hold | push).any() else 0.0
    jaw_seated = [float(np.median(joR[hw]) * 1000), float(np.median(joL[hw]) * 1000)] if hw.any() else [0.0, 0.0]
    jawF_seated = [float(np.median(jR[hw])), float(np.median(jL[hw]))] if hw.any() else [0.0, 0.0]
    final_xy = [float(np.median(dx_a[-20:]) * 1000), float(np.median(dy_a[-20:]) * 1000)]
    t_still = st["t_retreat_end"] if st["t_retreat_end"] is not None else t_a[-1]
    crit = {
        "held_in_hand_at_start": {
            "pass": bool(drop0 < 0.001 and grip0 > 3.0),
            "stick_drop_in_hand_mm": round(drop0 * 1000, 3), "min_mean_pad_force_N": round(grip0, 2)},
        "fully_inserted": {
            "pass": bool(abs(seat_dz) < SEAT_TOL_APPX),
            "seat_depth_from_cad_seat_mm": round(seat_dz * 1000, 3), "tolerance_mm": SEAT_TOL_APPX * 1000,
            "insertion_axial_peak_N": round(ins_peak, 2),
            "jaw_deflection_seated_mm": [round(v, 3) for v in jaw_seated],
            "jaw_force_seated_N": [round(v, 2) for v in jawF_seated],
            "spring_indent_mm": float(a.spring),
            "stick_tilt_max_deg": round(tilt_max, 2),
            "note": "seat = plug tip on the slot's centre block (and, for sticks 1-3, the shoulder on the slot "
                    "face): the CAD pose, 12 mm of plug in the slot"},
        "no_motion_when_pushed": {
            "pass": bool(P["F_peak"] >= 0.75 * PUSH_F and gained < PUSH_MOVE_MAX),
            "push_force_peak_N": round(float(P["F_peak"]), 2), "push_force_target_N": PUSH_F,
            "depth_gained_mm": round(gained * 1000, 3), "threshold_mm": PUSH_MOVE_MAX * 1000},
        "stays_seated_after_release": {
            "pass": bool(abs(final_dz - dz_pre) < STAY_MAX),
            "depth_before_release_mm": round(dz_pre * 1000, 3), "final_depth_mm": round(final_dz * 1000, 3),
            "final_xy_offset_mm": [round(v, 3) for v in final_xy], "threshold_mm": STAY_MAX * 1000},
    }
    check_released_and_clear(qa, t_a, rc_a, t_still, extra={
        "hand_rise_mm": round(float(hz_a[-1] - hz_a[t_a < t_rel][-1]) * 1000, 1) if (t_a < t_rel).any() else None})
    rc_entry = qa.report["checks"]["released_and_clear"]
    moved = rc_entry.get("hand_rise_mm") or 0.0
    rc_entry["min_move_mm"] = 40.0
    rc_entry["pass"] = bool(rc_entry["pass"] and moved >= 40.0)
    crit["released_and_clear"] = rc_entry
    crit["finger_pinch_real"] = {
        "pass": bool(min(grip_ins.values()) >= 3.0),
        "mean_pad_force_insert_N": {k: round(v, 2) for k, v in grip_ins.items()}, "pinch_set_N": pinch_n}
    for k, c in crit.items():
        qa.report["checks"].setdefault(k, c)
    # penetration of the stick into the exact CAD shell / centre / jaws (body frames: the CAD meshes centred on
    # their bodies' origins; check_penetration REPLACES the entry, the static checks merge into it)
    stick_local = st_tm.copy()
    stick_local.apply_translation(-c_cad)
    plug_v = st_tm.vertices[st_tm.vertices[:, 0] > x_tip - U.PLUG_LEN - 0.0003] - c_cad
    plug_local = trimesh.Trimesh(vertices=plug_v, faces=np.zeros((0, 3), int), process=False)
    slot_parts = {p.name: p.mesh() for p in kit["usb-slot"].parts(spring_label)}
    pieces = U.split_spring(slot_parts[f"usb_slot_spring_{a.spring}mm_indent"])
    pairs = []
    for n, lbl in zip(jaw_names, jaw_labels):
        jl = pieces[n].copy()
        jl.apply_translation(-0.5 * (jl.bounds[0] + jl.bounds[1]))
        pairs += [(lbl, jl, "usb_stick", stick_local), ("usb_stick", stick_local, lbl, jl)]
    qa.check_penetration(pairs, tol=PEN_TOL, n_samples=48)
    for part, nm in (("usb_slot_shell", "shell"), ("usb_slot_center", "centre")):
        world = slot_parts[part].copy()
        world.apply_transform(slot_pose)
        qa.check_penetration_static("usb_stick", stick_local, world, np.zeros(3), f"stick->{nm}",
                                    tol=PEN_TOL, n_samples=48)
        qa.check_penetration_static("usb_stick", plug_local, world, np.zeros(3), f"plug(all vertices)->{nm}",
                                    tol=PEN_TOL, n_samples=96)
    # the stick inside the slot body behind its walls (usb_slot.intrusion): a signed distance into a 1 mm wall never
    # exceeds 0.5 mm, so the checks above cannot see a stick that has gone through the wall
    surf, _ = trimesh.sample.sample_surface_even(st_tm, 2000, seed=0)
    box = np.random.default_rng(0).uniform(st_tm.bounds[0], st_tm.bounds[1], size=(8000, 3))
    pts_local = np.vstack([surf, box[st_tm.contains(box)][:2000]]) - c_cad
    slot_inv = np.linalg.inv(slot_pose)
    deep, deep_t = 0.0, None
    for f in range(len(qa.t)):
        q = qa.body_q[f, qa._idx("usb_stick")]
        R = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])[:3, :3]
        d = float(U.intrusion((pts_local @ R.T + q[:3]) @ slot_inv[:3, :3].T + slot_inv[:3, 3]).max())
        if d > deep:
            deep, deep_t = d, float(qa.t[f])
    pen = qa.report["checks"]["penetration"]
    pen["pairs"]["stick->slot body (behind its walls)"] = {"max_depth_mm": round(deep * 1000, 3), "at_t": deep_t,
                                                           "pass": bool(deep < PEN_TOL)}
    pen["pass"] = bool(pen["pass"] and deep < PEN_TOL)
    lo = mouth - np.array([0.06, 0.06, 0.03])
    hi = mouth + np.array([0.06, 0.06, 0.08])
    qa.check_envelope(["usb_stick"], lo, hi)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    report = rig.finish(qa)
    crit["start"] = {"pass": True, "info_only": True, "seed": a.seed,
                     "slot_xy_mm": [round(float(v) * 1e3, 2) for v in start[:2]],
                     "slot_yaw_deg": round(float(np.degrees(start[2])), 2),
                     "stick_offset_mm": [round(float(v) * 1e3, 2) for v in off_xy],
                     "range": {"xy_mm": round(SLOT_PLACE.dxy * 1e3, 1),
                               "yaw_deg": round(float(np.degrees(SLOT_PLACE.dyaw)), 1)},
                     "note": "slot pose: its centre on the table (nominal (0, 0) mm, yaw 90 deg: the jaws close "
                             "along world y)" + ("" if a.seed is not None else "; nominal start (no seed)")}
    changes = [
        f"stick '{stick_key}' and slot 'usb-slot' ({spring_label}) from the final printable kit "
        "(force_bench.kit; the slot model: force_bench.kit.mechanisms.usb_slot)",
        f"starts IN HAND 25 mm above the seat, offset ({off_xy[0]*1000:+.1f}, {off_xy[1]*1000:+.1f}) mm from the "
        "slot axis",
        f"printed spring jaws = prismatic joint drives at {hslot.params['ke_N_per_m'] / 1000:.0f} N/mm per jaw, "
        "an ESTIMATE (finite-element model of the printed flexure, E = 3 GPa; no measurement exists)",
        f"stick mass {stick_mass * 1000:.0f} g, a conditioning value (print estimate {stick_mass_print * 1000:.2f} "
        "g): MuJoCo's contact stiffness scales with the body's mass, and a gram-scale stick sinks into the stiff "
        "jaws instead of opening them; the jaw contacts are stiffened for the same reason",
        "push test, release at the pushed pose, 6 cm retreat, still",
    ]
    if a.seed is not None:
        changes.append(f"seeded start {a.seed}: slot at ({start[0] * 1e3:+.1f}, {start[1] * 1e3:+.1f}) mm, yaw "
                       f"{np.degrees(start[2]):+.1f} deg (uniform in the paper's 0.15 x 0.15 m square, +-20 deg "
                       "about the nominal 90 deg)")
    if pinch_n != PINCH_N["0.2"]:
        changes.append(f"pinch {pinch_n:.0f} N per pad for the stiffer spring (the 0.2 mm rows use 12 N; the FR3 "
                       "hand grips with up to 70 N)")
    return save_appendix_verdict(
        rig, task="Plug in USB", source=f"new task (kit {stick_key} + usb-slot, {spring_label})",
        spec="The USB stick is fully inserted into the slot, such that it does not noticeably move when further "
             "pushed toward the slot. The robot movement stops after terminating the insertion motion, moving away "
             "from, and breaking contact with the task objects.",
        criteria=crit, physics_qa=report, changes=changes)


if __name__ == "__main__":
    wp.init()
    main()
