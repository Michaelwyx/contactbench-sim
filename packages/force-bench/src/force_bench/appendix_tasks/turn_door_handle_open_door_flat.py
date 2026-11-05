"""Appendix task "Turn door handle to open door" (Table C1, task 29) on the FINAL kit door, lying flat on the table
like the real fixture (``force_bench.appendix_tasks._door_kit``; the door system is the kit library's ``door``
mechanism).

    Description: Turn the door handle fully to unlock the door, then open the door.  Success: the door is opened by
    at least 5 degrees, such that it is visually obvious that the door has been unlocked.

The lever.  On the final CAD the handle's bar points along the module's +y, which on the assembled door is the HINGE
edge: the lever rests pointing at the hinge, as on the real fixture, and is turned clockwise seen from above (the
bar swinging away from the camera), which is the turn that retracts the latch.  Lever, pinion and spring hub are one
link on the vertical spindle with the measured return spring (``force_bench.kit.mechanisms.doorknob_module``).

Grasp: top-down, the pads close ACROSS the bar 85 mm out from the spindle (the radius the springs were measured at),
so turning the lever is carried by a pad NORMAL; lifting the leaf through the bar is carried by pad friction (2 x
35 N x mu against ~2.5 N needed).  The fingertips touch an invisible convex hull of the bar (the drawn printed mesh
does not collide).

Expert, every command closed on MEASURED state:
  0  hover above the bar (hidden LOCK PROBE: the thrown latch holds the leaf shut against ~3x its weight moment);
  1  descend, squeeze across the bar;
  2  sweep the lever clockwise on its own arc (wrist yaw follows), with a bounded lead ahead of the MEASURED lever
     angle, until the MEASURED latch has retracted past the strike jamb (+0.1 mm); the sweep freezes there;
  3  lift the leaf to 45 deg on the hinge arc of the MEASURED door angle, lever held turned;
  4  hold the door open and stop (the reference videos end holding the lifted door; a flat door released at 45 deg
     falls shut).

Run: python -m force_bench.appendix_tasks.turn_door_handle_open_door_flat [--handle 1|2 --unlock 40|48|60|72|80
--spring 5|7.5|10 --name <output>]
"""

from __future__ import annotations

import argparse

import numpy as np
import warp as wp

from force_bench.appendix_tasks._door_kit import (
    CAM_POS, CAM_TARGET, LOCK_PROBE_NM, LOCK_PROBE_S, Hinge, HingeProbe, arm_filter, build_door, door_angle,
    door_penetration, down_quat, drawn_shapes, fingers_filter, joint_reader, lock_probe_check, make_rig, pads_check,
    smooth, spin)
from force_bench.appendix_tasks.common import save_appendix_verdict
from force_bench.kit.mechanisms.doorknob_module import JAMB_CLEAR_MM
from force_bench.rig import make_solver, set_finger_effort_limit

NAME = "turn_door_handle_open_door_flat"
SPEC = ("The door is opened by at least 5 degrees, such that it is visually obvious that the door has been "
        "unlocked.")
CLEAR_MARGIN_MM = 0.1
SWEEP_LEAD = np.deg2rad(6)     # bounded lead of the commanded lever angle over the MEASURED one
SWEEP_RATE = np.deg2rad(30)    # deg/s while the latch is still in the strike
OPEN_TO = np.deg2rad(45)
LEAD_MM = 6.0
YAW0 = 0.0                     # finger gap along world y = across the resting bar
GRIP_R = 0.085                 # along the bar from the spindle (the measured spring fits' push radius)
BAR_Z_MODULE = {"door_handle_1": 0.0725, "door_handle_2": 0.078}   # bar mid-height in the module frame (CAD)
HOVER = 0.08
FINGERS_OPEN, FINGERS_GRIP = 0.030, 0.000   # clamp the 16.5 / 20 mm bar
MJ_FINGER_KE, MJ_FINGER_F_MAX = 4000.0, 35.0
IMPRATIO = 10.0                # friction/normal impedance: at MuJoCo's 1.0 the pinch CREEPS under the spring's
# sustained torque (measured: the knob slid back 2 deg in 1 s and the latch re-entered the strike) although its
# Coulomb capacity is ~3x the load (plug_in_usb, hang_wrench, install_light_bulb use the same value)

T_SET, T_DESC, T_CLOSE, T_SW, T_PAUSE, T_OPEN, T_HOLD = 0.7, 1.4, 0.6, 3.0, 0.4, 3.6, 1.6
T = np.cumsum([T_SET, T_DESC, T_CLOSE, T_SW, T_PAUSE, T_OPEN, T_HOLD])


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--handle", choices=["1", "2"], default="1")
    ap.add_argument("--unlock", choices=["40", "48", "60", "72", "80"], default="60")
    ap.add_argument("--spring", choices=["5", "7.5", "10"], default="5")
    ap.add_argument("--name", default=None, help="output folder name")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    choice = {"Door handle geometry": f"Handle {a.handle}", "Amount of rotation to unlock": f"{a.unlock}°",
              "Rotation force": f"{a.spring}mm spring"}
    name = a.name or NAME
    rig = make_rig(name)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = MJ_FINGER_KE
    set_finger_effort_limit(rig.builder, MJ_FINGER_F_MAX)
    h = build_door(rig, knob=choice, lock="90°", lock_params={"hold_retracted": True})   # deadbolt parked
    hk = h._modules["doorknob-module"]
    lever, leaf = h.bodies["knob"], h.bodies["leaf"]
    sp = np.asarray(hk.info["spindle_point_world"])
    axis_xy = sp[:2]
    bar_dir = np.asarray(hk.pose)[:3, :3] @ np.array([0.0, 1.0, 0.0])   # the bar along the module's +y (world -x)
    g0 = sp + GRIP_R * bar_dir
    g0[2] = sp[2] + BAR_Z_MODULE[hk.info["grip_part"]]
    clear_m = (JAMB_CLEAR_MM + CLEAR_MARGIN_MM) * 1e-3
    rp = hk.info["pitch_radius_m"]

    rig.solve_arm_start(g0 + [0.0, 0.0, HOVER], down_quat(YAW0))
    rig.builder.joint_q[7:9] = [FINGERS_OPEN, FINGERS_OPEN]
    rig.builder.joint_target_q[7:9] = [FINGERS_OPEN, FINGERS_OPEN]
    arm_filter(rig, lever)
    fingers_filter(rig, drawn_shapes(rig, lever, hk.info["grip_hull_shapes"]))
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
    K, tau0 = hk.params["K_Nm_per_rad"], hk.params["tau0_Nm"]
    log, tcp_log = [], []
    st = {"ph": 0.0, "t_clear": None}

    def control(t):
        bq = rig.s0.body_q.numpy()
        q = rig.s0.joint_q.numpy()
        th_m = door_angle(bq[li])
        ph_m = float(q[jq["knob"]])        # lever angle relative to the leaf (joint coordinate)
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
                st["ph"] = float(max(st["ph"] - SWEEP_RATE / rig.fps, ph_m - SWEEP_LEAD))
        if t >= T[3]:
            if t < T[4]:
                th_r = 0.0
            elif t < T[5]:
                th_r = -OPEN_TO * smooth((t - T[4]) / T_OPEN)
            else:
                th_r = -OPEN_TO
            lead = LEAD_MM * 1e-3 / max(1e-3, hinge.radius(spin(g0, st["ph"], axis_xy)))
            th = float(np.clip(th_r, th_m - lead, th_m + lead))
        grasp = spin(g0, st["ph"], axis_xy) + np.array([0.0, 0.0, z_off])
        rig.drive_arm(hinge.arc(grasp, th), down_quat(YAW0 + st["ph"], th), fingers=fg)

    def extra(state, t):
        bq = state.body_q.numpy()
        q = state.joint_q.numpy()
        th = door_angle(bq[li])
        ph = float(q[jq["knob"]])
        x = float(q[jq["latch"]])
        log.append((t, th, ph, x))
        tcp_log.append((t, *(0.5 * (bq[fl, :3] + bq[fr, :3]))))
        return {"door_deg": round(float(np.degrees(-th)), 3), "lever_deg": round(float(np.degrees(-ph)), 2),
                "latch_retract_mm": round(x * 1000, 3),
                "spring_torque_Nm": round(tau0 + K * max(0.0, -float(q[jq["knob"]])), 4)}

    rec = rig.run(T[-1], control, extra)
    paths = rec.save("rollout", force_keys=["door_deg", "lever_deg", "latch_retract_mm", "spring_torque_Nm",
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
    crit["unlocked_by_handle_then_opened"] = {
        "pass": bool(eng.any() and (pre.max() if len(pre) else 0.0) < 1.0 and open_deg[-1] >= 5.0),
        "t_latch_clear_s": round(t_unl, 3),
        "door_deg_before_latch_clear_max": round(float(pre.max()) if len(pre) else 0.0, 3),
        "peak_latch_retract_mm": round(float(x_a.max() * 1000), 3), "jamb_clear_mm": JAMB_CLEAR_MM,
        "peak_lever_deg": round(float(np.degrees(-ph_a).max()), 1), "lever_stop_deg": hk.params["stop_deg"],
        "final_door_deg": round(float(open_deg[-1]), 2), "peak_door_deg": round(float(open_deg.max()), 2),
        "threshold_deg": 5.0,
        "note": "the leaf stays shut until the MEASURED latch has passed the strike jamb, then is lifted open"}
    crit["finger_grip_sustained"] = pads_check(rig, T[2] + 0.2, T[-1])
    tcp = np.array(tcp_log)
    m = tcp[:, 0] >= T[-1] - 1.0
    drift = float(np.linalg.norm(tcp[m, 1:][-1] - tcp[m, 1:][0]) * 1000)
    dd = float(np.ptp(open_deg[t_a >= T[-1] - 1.0]))
    crit["motion_stopped_door_held_open"] = {
        "pass": bool(drift < 3.0 and dd < 1.0 and open_deg[-1] >= 5.0),
        "tcp_drift_last_1s_mm": round(drift, 2), "door_range_last_1s_deg": round(dd, 3),
        "note": "ends like the reference videos: hand on the lever, door held open, arm still (a flat door released "
                "at 45 deg falls shut)"}
    for k, v in crit.items():
        qa.report["checks"][k] = v
        qa.report["pass"] &= bool(v["pass"])
    door_penetration(qa, rig, h)
    gear_err = float(np.abs(x_a + rp * ph_a).max())
    qa.report["checks"]["gear_coupling"] = {
        "pass": bool(gear_err < 3e-4), "max_error_mm": round(gear_err * 1e3, 4), "pitch_radius_mm": round(rp * 1e3, 3),
        "note": "latch travel vs pitch radius x lever angle over the whole rollout (the rack-and-pinion equality)"}
    qa.report["pass"] &= bool(gear_err < 3e-4)
    report = rig.finish(qa)
    v = hk.params
    save_appendix_verdict(
        rig, task="Turn door handle to open door", source=__name__, spec=SPEC, criteria=crit, physics_qa=report,
        changes=[
            "final kit door (door + frame halves + doorknob and lock modules, assembled at the site's poses) lying "
            "FLAT on the table with no stand, as the real fixture",
            f"doorknob module: {choice['Door handle geometry']}, {choice['Amount of rotation to unlock']} pinion "
            f"(r_p {v['pitch_radius_mm']} mm, stop {v['stop_deg']} deg, latch clears the jamb at "
            f"{v['jamb_clear_deg']} deg), {choice['Rotation force']} (K {v['K_Nm_per_rad']} N.m/rad, preload "
            f"{v['tau0_Nm']} N.m, fitted to measured lever push force)",
            "the lever rests pointing at the HINGE edge (the final CAD's own mounting, as on the real fixture) and is "
            "swept clockwise (seen from above) until the measured latch clears the strike jamb",
            "top-down pinch across the bar 85 mm from the spindle, then the leaf is lifted about the horizontal hinge "
            "to 45 deg through the lever",
            "ends holding the door open with the arm stopped, as the reference videos do (a released flat door "
            "falls shut)",
            "hidden lock probe at t < 0.5 s: 1.5 N.m opening torque at the hinge must not move the locked leaf",
            "the lock module's deadbolt is parked withdrawn (scenery), so the latch alone gates the leaf",
            "the fingertips touch an invisible convex hull of the bar (the printed lever mesh is drawn); the printed "
            "rack and pinion is a joint equality, the spiral spring a spindle spring",
            "finger_grip_sustained is gated per 50 ms (3 frames): a single-frame zero reading of one pad (the pinch "
            "see-sawing between the pads within a frame) is reported, not failed",
            f"MuJoCo impratio {IMPRATIO:g} (friction/normal impedance) against the spring's sustained torque",
            *[f"door model: {d}" for d in h.info["deviations"]],
        ])
    return report


if __name__ == "__main__":
    wp.init()
    main()
