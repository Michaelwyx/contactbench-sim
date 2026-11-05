"""Appendix task "Turn doorknob to open door" (Table C1, task 28) on the FINAL kit door, lying flat on the table like
the real fixture (``force_bench.appendix_tasks._door_kit``; the door system is the kit library's ``door`` mechanism).

    Description: Turn the doorknob fully to unlock the door, then open the door.  Success: the door is opened by at
    least 5 degrees, such that it is visually obvious that the door has been unlocked.

Mechanism (``force_bench.kit.mechanisms.doorknob_module``): the knob, its pinion and the spring hub are one link
turning about the vertical spindle; the latch is a prismatic link driven by the pinion through a stiff joint equality
(the printed rack and pinion, bilateral); the printed spiral return spring is a spindle spring with the stiffness and
preload fitted to the measured lever push force.  The latch starts 15.24 mm beyond the strike jamb, in the strike
pocket of the frame, and the printed stop lets it retract 15.44 mm.  Variants (``--knob``, ``--diameter``,
``--unlock``, ``--spring``) are the website's configuration options of the doorknob module.

Expert, top-down, every command closed on MEASURED state:
  0  hover 9 cm above the knob, fingers open (a hidden LOCK PROBE pushes the leaf open with ~3x its weight moment and
     the thrown latch holds it shut);
  1  descend so the pads straddle the knob 15 mm below its equator (the underside slope the pads bear on is what
     makes the later lift form-closed; both knob shapes have the same profile there);
  2  squeeze (35 N/pad actuator ceiling, 70 N total = the real hand);
  3  turn clockwise (seen from above) until the MEASURED latch has retracted past the strike jamb (+0.1 mm), with a
     bounded wrist lead over the measured knob angle; the turn freezes there;
  4  lift the leaf on the hinge arc of the MEASURED door angle (TCP clamped to a bounded lead ahead of it), knob held
     turned, to 45 deg;
  5  hold the door open and stop (the reference videos end holding the lifted door; a released flat door falls shut).

Run: python -m force_bench.appendix_tasks.turn_doorknob_open_door_flat [--knob 1|2 --diameter 50|55|60 --unlock
40|48|60|72|80 --spring 5|7.5|10 --name <output>]
"""

from __future__ import annotations

import argparse

import numpy as np
import warp as wp

from force_bench.appendix_tasks._door_kit import (
    CAM_POS, CAM_TARGET, LOCK_PROBE_NM, LOCK_PROBE_S, Hinge, HingeProbe, arm_filter, build_door, door_angle,
    door_penetration, down_quat, drawn_shapes, fingers_filter, joint_reader, lock_probe_check, make_rig, pads_check,
    smooth)
from force_bench.appendix_tasks.common import save_appendix_verdict
from force_bench.kit.mechanisms.doorknob_module import JAMB_CLEAR_MM
from force_bench.rig import make_solver, set_finger_effort_limit

NAME = "turn_doorknob_open_door_flat"
SPEC = ("The door is opened by at least 5 degrees, such that it is visually obvious that the door has been "
        "unlocked.")
CLEAR_MARGIN_MM = 0.1          # turn until the measured latch is this far past the jamb face
TWIST_LEAD = np.deg2rad(25)    # bounded wrist lead ahead of the MEASURED knob
TWIST_RATE = np.deg2rad(32)    # deg/s of wrist turn while the latch is still in the strike
OPEN_TO = np.deg2rad(45)       # door reference
LEAD_MM = 6.0                  # TCP lead on the hinge arc ahead of the MEASURED door angle: this bound IS the
# lifting force (the arm is a stiff position servo), and it brakes the leaf on the way up
YAW0 = np.deg2rad(40)          # hand yaw at the grasp (IK-swept)
GRASP_BELOW_EQUATOR = 0.015    # TCP below the knob's equator: pads span ~TCP-9.7 .. TCP+8.8 mm
EQUATOR_Z_MODULE = 0.060       # knob equator in the module frame (both shapes: r = d/2 at z 58.5..60 mm)
HOVER = 0.09
FINGERS_OPEN, FINGERS_GRIP = 0.040, 0.013
MJ_FINGER_KE, MJ_FINGER_F_MAX = 4000.0, 35.0
IMPRATIO = 10.0                # friction/normal impedance: at MuJoCo's 1.0 the pinch CREEPS under the spring's
# sustained torque (measured: the knob slid back 2 deg in 1 s and the latch re-entered the strike) although its
# Coulomb capacity is ~3x the load (plug_in_usb, hang_wrench, install_light_bulb use the same value)

T_SET, T_DESC, T_CLOSE, T_TW, T_PAUSE, T_OPEN, T_HOLD = 0.7, 1.4, 0.6, 3.0, 0.4, 3.6, 1.6
T = np.cumsum([T_SET, T_DESC, T_CLOSE, T_TW, T_PAUSE, T_OPEN, T_HOLD])


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--knob", choices=["1", "2"], default="1")
    ap.add_argument("--diameter", choices=["50", "55", "60"], default="55")
    ap.add_argument("--unlock", choices=["40", "48", "60", "72", "80"], default="60")
    ap.add_argument("--spring", choices=["5", "7.5", "10"], default="5")
    ap.add_argument("--name", default=None, help="output folder name")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    choice = {"Doorknob geometry": f"Doorknob {a.knob}, {a.diameter}mm diameter",
              "Amount of rotation to unlock": f"{a.unlock}°", "Rotation force": f"{a.spring}mm spring"}
    name = a.name or NAME
    rig = make_rig(name)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = MJ_FINGER_KE
    set_finger_effort_limit(rig.builder, MJ_FINGER_F_MAX)
    h = build_door(rig, knob=choice, lock="90°", lock_params={"hold_retracted": True})   # deadbolt parked
    hk = h._modules["doorknob-module"]
    knob, leaf = h.bodies["knob"], h.bodies["leaf"]
    axis_xy = np.asarray(hk.info["spindle_point_world"])[:2]
    z_eq = float(hk.info["spindle_point_world"][2]) + EQUATOR_Z_MODULE
    g0 = np.array([axis_xy[0], axis_xy[1], z_eq - GRASP_BELOW_EQUATOR])
    clear_m = (JAMB_CLEAR_MM + CLEAR_MARGIN_MM) * 1e-3
    rp = hk.info["pitch_radius_m"]

    rig.solve_arm_start(g0 + [0.0, 0.0, HOVER], down_quat(YAW0))
    rig.builder.joint_q[7:9] = [FINGERS_OPEN, FINGERS_OPEN]
    rig.builder.joint_target_q[7:9] = [FINGERS_OPEN, FINGERS_OPEN]
    arm_filter(rig, knob)                                     # only the fingertips touch the knob ...
    fingers_filter(rig, drawn_shapes(rig, knob, hk.info["grip_hull_shapes"]))   # ... through its hull
    arm_filter(rig, leaf)

    rig.hard_joint_limits = True
    rig.limit_dampratio = 0.5
    rig.finalize(probes=[f"{hk.label}/latch", f"{hk.label}/knob", "door/leaf",
                         "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"], cam_pos=CAM_POS, cam_target=CAM_TARGET)
    rig.solver = make_solver(rig.model, iterations=24, **{**rig.solver_options, "impratio": IMPRATIO})
    probe = HingeProbe(rig, h)
    hinge = Hinge(h)
    jq = joint_reader(rig, h)
    li = leaf
    fl, fr = rig.body_index("fr3/fr3_leftfinger"), rig.body_index("fr3/fr3_rightfinger")
    r_knob = hinge.radius(g0)
    lead_ang = LEAD_MM * 1e-3 / r_knob
    K, tau0 = hk.params["K_Nm_per_rad"], hk.params["tau0_Nm"]
    log, tcp_log = [], []
    st = {"ph": 0.0, "t_clear": None}

    def control(t):
        bq = rig.s0.body_q.numpy()
        q = rig.s0.joint_q.numpy()
        th_m = door_angle(bq[li])
        ph_m = float(q[jq["knob"]])        # spindle angle relative to the leaf (joint coordinate)
        x_m = float(q[jq["latch"]])
        probe.set(-LOCK_PROBE_NM if LOCK_PROBE_S[0] <= t < LOCK_PROBE_S[1] else 0.0)
        z_off, fg, th = 0.0, FINGERS_GRIP, 0.0
        if t < T[0]:
            z_off, fg = HOVER, FINGERS_OPEN
        elif t < T[1]:
            z_off, fg = HOVER * (1.0 - smooth((t - T[0]) / T_DESC)), FINGERS_OPEN
        elif t < T[2]:
            fg = FINGERS_OPEN + (FINGERS_GRIP - FINGERS_OPEN) * smooth((t - T[1]) / T_CLOSE)
        if t >= T[2]:
            # turn on the MEASURED latch: advance while it is still in the strike (also if a pad slip lets it back in
            # during the lift), with a bounded lead over the measured angle; freeze once it has passed the jamb
            if x_m >= clear_m:
                if st["t_clear"] is None:
                    st["t_clear"] = t
            else:
                st["ph"] = float(max(st["ph"] - TWIST_RATE / rig.fps, ph_m - TWIST_LEAD))
        if t >= T[3]:
            if t < T[4]:
                th_r = 0.0
            elif t < T[5]:
                th_r = -OPEN_TO * smooth((t - T[4]) / T_OPEN)
            else:
                th_r = -OPEN_TO
            th = float(np.clip(th_r, th_m - lead_ang, th_m + lead_ang))
        tcp = hinge.arc(g0 + np.array([0.0, 0.0, z_off]), th)
        rig.drive_arm(tcp, down_quat(YAW0 + st["ph"], th), fingers=fg)

    def extra(state, t):
        bq = state.body_q.numpy()
        q = state.joint_q.numpy()
        th = door_angle(bq[li])
        ph = float(q[jq["knob"]])
        x = float(q[jq["latch"]])
        log.append((t, th, ph, x))
        tcp_log.append((t, *(0.5 * (bq[fl, :3] + bq[fr, :3]))))
        return {"door_deg": round(float(np.degrees(-th)), 3), "knob_deg": round(float(np.degrees(-ph)), 2),
                "latch_retract_mm": round(x * 1000, 3),
                "spring_torque_Nm": round(tau0 + K * max(0.0, -float(q[jq["knob"]])), 4)}

    rec = rig.run(T[-1], control, extra)
    paths = rec.save("rollout", force_keys=["door_deg", "knob_deg", "latch_retract_mm", "spring_torque_Nm",
                                            "fr3/fr3_leftfinger_N", "fr3/fr3_rightfinger_N"])
    print("saved:", paths)

    arr = np.array(log)
    t_a, th_a, ph_a, x_a = arr.T
    open_deg = np.degrees(-th_a)
    qa = rig.qa(paths)
    crit = {}
    crit["locked_at_start"] = lock_probe_check(t_a, th_a)
    eng = x_a * 1000.0 > JAMB_CLEAR_MM
    t_unl = float(t_a[np.argmax(eng)]) if eng.any() else float("inf")
    pre = open_deg[t_a < t_unl]
    crit["unlocked_by_knob_then_opened"] = {
        "pass": bool(eng.any() and (pre.max() if len(pre) else 0.0) < 1.0 and open_deg[-1] >= 5.0),
        "t_latch_clear_s": round(t_unl, 3),
        "door_deg_before_latch_clear_max": round(float(pre.max()) if len(pre) else 0.0, 3),
        "peak_latch_retract_mm": round(float(x_a.max() * 1000), 3), "jamb_clear_mm": JAMB_CLEAR_MM,
        "peak_knob_deg": round(float(np.degrees(-ph_a).max()), 1), "knob_stop_deg": hk.params["stop_deg"],
        "final_door_deg": round(float(open_deg[-1]), 2), "peak_door_deg": round(float(open_deg.max()), 2),
        "threshold_deg": 5.0,
        "note": "the leaf stays shut until the MEASURED latch has passed the strike jamb, then is lifted open; final "
                "angle measured at the end of the rollout"}
    crit["finger_grip_sustained"] = pads_check(rig, T[2] + 0.2, T[-1])
    tcp = np.array(tcp_log)
    m = tcp[:, 0] >= T[-1] - 1.0
    drift = float(np.linalg.norm(tcp[m, 1:][-1] - tcp[m, 1:][0]) * 1000)
    dd = float(np.ptp(open_deg[t_a >= T[-1] - 1.0]))
    crit["motion_stopped_door_held_open"] = {
        "pass": bool(drift < 3.0 and dd < 1.0 and open_deg[-1] >= 5.0),
        "tcp_drift_last_1s_mm": round(drift, 2), "door_range_last_1s_deg": round(dd, 3),
        "note": "ends like the reference videos: hand on the knob, door held open, arm still (a flat door released "
                "at 45 deg falls shut)"}
    for k, v in crit.items():
        qa.report["checks"][k] = v
        qa.report["pass"] &= bool(v["pass"])
    door_penetration(qa, rig, h)
    gear_err = float(np.abs(x_a + rp * ph_a).max())     # latch = r_p * (-knob) (the leaf-relative knob angle)
    qa.report["checks"]["gear_coupling"] = {
        "pass": bool(gear_err < 3e-4), "max_error_mm": round(gear_err * 1e3, 4), "pitch_radius_mm": round(rp * 1e3, 3),
        "note": "latch travel vs pitch radius x knob angle over the whole rollout (the rack-and-pinion equality)"}
    qa.report["pass"] &= bool(gear_err < 3e-4)
    report = rig.finish(qa)
    v = hk.params
    save_appendix_verdict(
        rig, task="Turn doorknob to open door", source=__name__, spec=SPEC, criteria=crit, physics_qa=report, changes=[
            "final kit door (door + frame halves + doorknob and lock modules, assembled at the site's poses) lying "
            "FLAT on the table with no stand, as the real fixture",
            f"doorknob module: {choice['Doorknob geometry']}, {choice['Amount of rotation to unlock']} pinion "
            f"(r_p {v['pitch_radius_mm']} mm, stop {v['stop_deg']} deg, latch clears the jamb at "
            f"{v['jamb_clear_deg']} deg), {choice['Rotation force']} (K {v['K_Nm_per_rad']} N.m/rad, preload "
            f"{v['tau0_Nm']} N.m, fitted to measured lever push force)",
            "top-down expert: grasp the knob from above, turn clockwise until the measured latch clears the strike "
            "jamb, lift the leaf about the horizontal hinge to 45 deg",
            "ends holding the door open with the arm stopped, as the reference videos do (a released flat door "
            "falls shut)",
            "hidden lock probe at t < 0.5 s: 1.5 N.m opening torque at the hinge must not move the locked leaf",
            "the lock module's deadbolt is parked withdrawn (scenery), so the latch alone gates the leaf",
            "the fingertips touch an invisible convex hull of the knob's graspable portion (the printed knob mesh "
            "is drawn); the printed rack and pinion is a joint equality, the spiral spring a spindle spring",
            "finger_grip_sustained is gated per 50 ms (3 frames): a single-frame zero reading of one pad (the pinch "
            "see-sawing between the pads within a frame) is reported, not failed",
            f"MuJoCo impratio {IMPRATIO:g} (friction/normal impedance) against the spring's sustained torque",
            *[f"door model: {d}" for d in h.info["deviations"]],
        ])
    return report


if __name__ == "__main__":
    wp.init()
    main()
