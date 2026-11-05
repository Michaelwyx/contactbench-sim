"""Appendix task "Turn key to unlock door" (Table C1, task 27) on the FINAL kit door, lying flat on the table like the
real fixture (``force_bench.appendix_tasks._door_kit``; the door system is the kit library's ``door`` mechanism).

    Description: A locked door module is placed in the workspace with a key already inserted into the lock.  Turn the
    key to unlock the door.  Success: the key is fully turned such that the door can be opened.  The robot movement
    stops after terminating the rotation motion, moving away from, and breaking contact with the task objects.

Mechanism (``force_bench.kit.mechanisms.lock_module``): the lock cylinder (plug + pinion, one printed part) is a
revolute link of the leaf about the vertical lock axis; the deadbolt is a prismatic link driven by the pinion through
a stiff joint equality (the printed rack and pinion), with the friction spring as dry friction on its slide, so it
stays where the key leaves it.  Key 1 is the kit's ``key-1`` seated in the keyway (its blade section fitted into the
keyway polygon, tip on the keyway floor) and, as this task never inserts or withdraws it, welded into the plug link.
The doorknob module is scenery with its latch held back at its stop, so the deadbolt alone gates the leaf.
``--lock`` picks the website's lock variant (90 / 180 / 270 / 360 deg to unlock).

Expert: hover above the key (hidden LOCK PROBE: the thrown deadbolt holds the leaf shut against ~3x its weight
moment), descend so the pads straddle the bow plate's 2.5 mm faces with the fingertips 2.8 mm above the lock cone
(elastomer pads, a pad-balance servo and an admittance grip), turn clockwise on the MEASURED plug angle
with a bounded lead, the TCP carried by the bow's measured pose, until the MEASURED deadbolt has retracted past the
strike jamb (+0.3 mm); then open the fingers, rise 12 cm and stop.  A turn longer than one wrist stroke (the 360 deg
lock: the FR3's last joint runs out after 295 deg from the start pose) REGRASPS after ``STROKE_MAX``: the fingers open,
the open hand swings back 180 deg about the plug axis (the bow plate maps onto itself) and pinches again; the plug
stays put meanwhile (the deadbolt's friction spring).

"Such that the door can be opened" is TESTED after the hand has left: an opening torque at the hinge (the same
1.5 N.m that could not move the locked leaf at the start) must now lift it >= 5 deg.  That probe is the last
``T_PROBE`` seconds of the rollout and is not part of the task, so a website clip ends before it.

Run: python -m force_bench.appendix_tasks.turn_key_unlock_door_flat [--lock 90|180|270|360 --name <output>]
"""

from __future__ import annotations

import argparse

import numpy as np
import trimesh
import warp as wp

from force_bench.appendix_tasks._door_kit import (
    CAM_POS, CAM_TARGET, LOCK_PROBE_NM, LOCK_PROBE_S, HingeProbe, arm_filter, build_door, door_angle,
    door_penetration, down_quat, joint_reader, lock_probe_check, make_rig, smooth)
from force_bench.appendix_tasks.common import CONTACT_FREE_N, RobotContactProbe, save_appendix_verdict
from force_bench.kit.mechanisms.lock_module import JAMB_CLEAR_MM
from force_bench.rig import make_solver, set_finger_effort_limit
from force_bench.tasks.nut_bolt_arm import PairProbe

NAME = "turn_key_unlock_door_flat"
SPEC = ("The key is fully turned such that the door can be opened. The robot movement stops after terminating the "
        "rotation motion, moving away from, and breaking contact with the task objects.")
CLEAR_MARGIN_MM = 0.3            # turn until the measured deadbolt is this far past the jamb face
TURN_LEAD = np.deg2rad(18)
TURN_RATE = np.deg2rad(35)       # deg/s of commanded turn
YAW0 = np.pi / 2                 # finger gap along world x = across the bow plate; the clockwise turn then runs the
# FR3's wrist joint (q7) from -1.78 rad up to its 3.016 rad limit, reached after a 295 deg turn (seeded IK sweep)
STROKE_MAX = np.deg2rad(278)     # longest single stroke (0.2 rad of q7 margin left); a longer turn (the 360 deg lock)
# REGRASPS: release, swing the open hand back 180 deg about the plug axis (the bow plate maps onto itself), pinch
# again, go on turning
T_RG_OPEN, T_RG_SWING, T_RG_CLOSE = 0.4, 1.0, 0.6
HOVER = 0.08
RETREAT = 0.12
CONE_TOP_Z_MODULE = 0.0425       # lock top's cone face, lock module frame (CAD)
TIP_CLEARANCE = 0.0028           # fingertips above the cone face
FINGERTIP_BELOW_TCP = 0.00993    # physics_notes pitfall 7.1
BOW_HALF = 0.00125               # key-1 is 2.5 mm thick
FING_OPEN = 0.020                # 40 mm gap around the bow
FING_TOUCH = BOW_HALF + 0.000085 + 0.00009   # pads kissing the bow faces (pad face at dof - 0.085 mm)
FING_KEY = FING_TOUCH - 0.0010   # ~1 mm/side into the bow, force-capped
GRIP_FORCE_LO, GRIP_FORCE_HI = 6.0, 20.0   # N/pad admittance band
GRIP_RELAX_RATE = 0.00003        # m/frame of finger-command relief
BAL_GAIN, BAL_RATE, BAL_MAX = 2.0e-5, 6.0e-5, 0.0035   # pad-balance servo
FINGER_KE, FINGER_KD, FINGER_F_MAX = 8000.0, 60.0, 20.0
PAD_MU = 1.4                     # elastomer pads
PROBE_STOP_DEG = 8.0             # the open probe switches off at this angle

T_SET, T_DESC, T_CLOSE, T_SETTLE = 0.7, 1.4, 0.5, 0.4
T_OPEN, T_RETREAT, T_STILL, T_PROBE = 0.6, 1.4, 1.0, 1.2


def parse(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--lock", choices=["90", "180", "270", "360"], default="90")
    ap.add_argument("--name", default=None, help="output folder name")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse(argv)
    name = a.name or NAME
    rig = make_rig(name)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = FINGER_KE
        rig.builder.joint_target_kd[d] = FINGER_KD
    set_finger_effort_limit(rig.builder, FINGER_F_MAX)
    for bl in ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger"):
        bi = rig.builder.body_label.index(bl)
        for sid in rig.builder.body_shapes.get(bi, []):
            rig.builder.shape_material_mu[sid] = PAD_MU
    h = build_door(rig, lock=f"{a.lock}°", key_mode="seated", knob_params={"hold_retracted": True})
    hl = h._modules["lock-module"]
    plug, leaf = h.bodies["plug"], h.bodies["leaf"]
    lp = np.asarray(hl.info["plug_point_world"])
    tcp_z = lp[2] + CONE_TOP_Z_MODULE + TIP_CLEARANCE + FINGERTIP_BELOW_TCP
    bow0 = np.array([lp[0], lp[1], tcp_z])     # the bow plate's mid-plane is on the plug axis
    r = hl.info["travel_per_rad_m"]
    clear_m = (JAMB_CLEAR_MM + CLEAR_MARGIN_MM) * 1e-3
    turn_max = min(hl.info["stop_rad"] - np.deg2rad(0.5), clear_m / r + np.deg2rad(2.0))
    n_regrasp = int(np.ceil(turn_max / STROKE_MAX)) - 1
    T_TURN = 2.6 * float(np.degrees(turn_max)) / 90.0 + n_regrasp * (T_RG_OPEN + T_RG_SWING + T_RG_CLOSE + 0.3)
    T = np.cumsum([T_SET, T_DESC, T_CLOSE, T_TURN, T_SETTLE, T_OPEN, T_RETREAT, T_STILL, T_PROBE])
    t_clip_end = float(T[7])

    rig.solve_arm_start(bow0 + [0.0, 0.0, HOVER], down_quat(YAW0))
    rig.builder.joint_q[7:9] = [FING_OPEN, FING_OPEN]
    rig.builder.joint_target_q[7:9] = [FING_OPEN, FING_OPEN]
    arm_filter(rig, plug)
    arm_filter(rig, leaf)     # only the fingertips may ever touch the door

    rig.hard_joint_limits = True
    rig.limit_dampratio = 0.5
    rig.finalize(probes=[f"{hl.label}/deadbolt", f"{hl.label}/plug", "door/leaf",
                         "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"], cam_pos=CAM_POS, cam_target=CAM_TARGET)
    for side in ("left", "right"):   # fingertip <-> key (on the plug link) only
        lbl = f"fr3/fr3_{side}finger"
        rig.probes[lbl] = PairProbe(rig.model, sensing_body=lbl, counterpart_bodies=[f"{hl.label}/plug"])
    rig.probes["robot_object"] = RobotContactProbe(rig.model, [f"{hl.label}/plug", "door/leaf",
                                                               f"{hl.label}/deadbolt"])
    rig.solver = make_solver(rig.model, iterations=24, **rig.solver_options)
    probe = HingeProbe(rig, h)
    jq = joint_reader(rig, h)
    li, pi_ = leaf, plug
    fl, fr = rig.body_index("fr3/fr3_leftfinger"), rig.body_index("fr3/fr3_rightfinger")
    pq0 = rig.s0.body_q.numpy()[pi_].copy()
    R0 = trimesh.transformations.quaternion_matrix([pq0[6], pq0[3], pq0[4], pq0[5]])[:3, :3]
    grasp_local = R0.T @ (bow0 - pq0[:3])   # a point ON the bow, plug frame
    log, tcp_log = [], []
    st = {"tr": 0.0, "probe_on": True, "end": None, "off": np.zeros(3), "fq": FING_OPEN, "t_clear": None,
          "yaw_off": 0.0, "stroke0": 0.0, "rg": None, "regrasps": []}

    def control(t):
        bq = rig.s0.body_q.numpy()
        q = rig.s0.joint_q.numpy()
        th_m = door_angle(bq[li])
        ps_m = float(q[jq["plug"]])       # joint coordinate: the lock module sits at -90 deg to the leaf
        x_m = float(q[jq["deadbolt"]])
        Rp = trimesh.transformations.quaternion_matrix([bq[pi_][6], bq[pi_][3], bq[pi_][4], bq[pi_][5]])[:3, :3]
        bow = np.asarray(bq[pi_][:3]) + Rp @ grasp_local
        tau = 0.0
        if LOCK_PROBE_S[0] <= t < LOCK_PROBE_S[1]:
            tau = -LOCK_PROBE_NM
        if t >= T[7] and st["probe_on"]:
            if np.degrees(-th_m) >= PROBE_STOP_DEG:
                st["probe_on"] = False
            else:
                tau = -LOCK_PROBE_NM
        probe.set(tau)
        # PAD-BALANCE SERVO + ADMITTANCE GRIP: the arm is a spring under position control, so a
        # pinch commanded on geometry ends up lopsided; shift the TCP along the measured closing axis until the two
        # pad forces match, and hold the squeeze inside a force band.
        hL = rig.probes["fr3/fr3_leftfinger"].history
        hR = rig.probes["fr3/fr3_rightfinger"].history
        fL = hL[-1]["force"] if hL else 0.0
        fR = hR[-1]["force"] if hR else 0.0
        if T[1] + 0.3 <= t < T[4] and st["rg"] is None:
            u = np.asarray(bq[fl][:3]) - np.asarray(bq[fr][:3])
            nu_ = float(np.linalg.norm(u))
            if nu_ > 1e-9:
                st["off"] = np.clip(st["off"] + np.clip(BAL_GAIN * (fL - fR), -BAL_RATE, BAL_RATE) * u / nu_,
                                    -BAL_MAX, BAL_MAX)
            if max(fL, fR) > GRIP_FORCE_HI:
                st["fq"] = min(st["fq"] + GRIP_RELAX_RATE, FING_TOUCH)
            elif min(fL, fR) < GRIP_FORCE_LO:
                st["fq"] = max(st["fq"] - GRIP_RELAX_RATE, FING_KEY)
        z_off = 0.0
        if t < T[0]:
            z_off, st["fq"] = HOVER, FING_OPEN
        elif t < T[1]:
            z_off = HOVER * (1.0 - smooth((t - T[0]) / T_DESC))
            st["fq"] = FING_OPEN
        elif t < T[1] + 0.3:     # close to a kiss, then the admittance band
            st["fq"] = FING_OPEN + (FING_TOUCH - FING_OPEN) * smooth((t - T[1]) / 0.3)
        if T[2] <= t < T[4] and st["rg"] is not None:
            # REGRASP: open, swing the open hand back 180 deg about the plug axis, close to a kiss
            dt = t - st["rg"]
            if dt < T_RG_OPEN:
                st["fq"] = st["fq_rg"] + (FING_OPEN - st["fq_rg"]) * smooth(dt / T_RG_OPEN)
            elif dt < T_RG_OPEN + T_RG_SWING:
                st["fq"] = FING_OPEN
                st["yaw_off"] = st["yaw_off0"] + np.pi * smooth((dt - T_RG_OPEN) / T_RG_SWING)
            elif dt < T_RG_OPEN + T_RG_SWING + T_RG_CLOSE:
                st["yaw_off"] = st["yaw_off0"] + np.pi
                st["off"] = np.zeros(3)
                st["fq"] = FING_OPEN + (FING_TOUCH - FING_OPEN) * smooth((dt - T_RG_OPEN - T_RG_SWING) / T_RG_CLOSE)
            else:
                st["rg"], st["stroke0"] = None, st["tr"]
        elif T[2] <= t < T[4]:
            # turn on the MEASURED deadbolt: advance while it is still in the strike, with a bounded lead over the
            # measured plug angle, never past turn_max; freeze once it has passed the jamb
            if x_m >= clear_m:
                if st["t_clear"] is None:
                    st["t_clear"] = t
            elif st["tr"] - st["stroke0"] <= -STROKE_MAX:
                st.update(rg=t, yaw_off0=st["yaw_off"], fq_rg=st["fq"])
                st["regrasps"].append(round(t, 3))
            else:
                tr = max(st["tr"] - TURN_RATE / rig.fps, ps_m - TURN_LEAD, -turn_max)
                st["tr"] = min(tr, st["tr"])
        if t >= T[4]:
            if st["end"] is None:          # freeze the pose the turn ended in
                st["end"] = (bow + st["off"], st["tr"], st["fq"])
            p_end, tr_end, fq_end = st["end"]
            fg = fq_end + (FING_OPEN - fq_end) * smooth((t - T[4]) / T_OPEN)
            rise = RETREAT * smooth((t - T[5]) / T_RETREAT) if t >= T[5] else 0.0
            rig.drive_arm(p_end + np.array([0.0, 0.0, rise]), down_quat(YAW0 + tr_end + st["yaw_off"]), fingers=fg)
        else:
            tgt = (bow + st["off"]) if t >= T[1] else bow0
            rig.drive_arm(tgt + np.array([0.0, 0.0, z_off]), down_quat(YAW0 + st["tr"] + st["yaw_off"]),
                          fingers=st["fq"])

    def extra(state, t):
        bq = state.body_q.numpy()
        q = state.joint_q.numpy()
        th = door_angle(bq[li])
        ps = float(q[jq["plug"]])
        x = float(q[jq["deadbolt"]])
        log.append((t, th, ps, x))
        tcp_log.append((t, *(0.5 * (bq[fl, :3] + bq[fr, :3]))))
        h_ = rig.probes["robot_object"].history
        return {"door_deg": round(float(np.degrees(-th)), 3), "key_deg": round(float(np.degrees(-ps)), 2),
                "bolt_retract_mm": round(x * 1000, 3),
                "robot_object_contact_N": round(h_[-1]["force"], 4) if h_ else 0.0}

    rec = rig.run(T[-1], control, extra)
    paths = rec.save("rollout", force_keys=["key_deg", "bolt_retract_mm", "door_deg",
                                            "fr3/fr3_leftfinger_N", "fr3/fr3_rightfinger_N"])
    print("saved:", paths)

    arr = np.array(log)
    t_a, th_a, ps_a, x_a = arr.T
    qa = rig.qa(paths)
    crit = {}
    crit["start_locked_key_inserted"] = lock_probe_check(t_a, th_a, 1.0)
    crit["start_locked_key_inserted"]["initial_bolt_retract_mm"] = round(float(x_a[0] * 1000), 3)
    before = t_a < T[7]
    fin_ret = float(x_a[before][-1] * 1000)
    peak_key = float(np.degrees(-ps_a).max())
    crit["key_fully_turned"] = {
        "pass": bool(fin_ret > JAMB_CLEAR_MM),
        "peak_key_deg": round(peak_key, 1), "nominal_unlock_deg": hl.params["nominal_unlock_deg"],
        "key_deg_at_jamb_clear_cad": hl.params["jamb_clear_deg"], "stop_deg": hl.params["stop_deg"],
        "final_key_deg": round(float(np.degrees(-ps_a[before][-1])), 1),
        "bolt_retract_after_release_mm": round(fin_ret, 3), "jamb_clear_mm": JAMB_CLEAR_MM,
        "note": "fully turned = the deadbolt is out of the strike (retracted past the jamb face) after the hand has "
                "let go: the plug carries no spring, so the bolt stays where the key left it"}
    h_ = rig.probes["robot_object"].history
    tc = np.array([r_["t"] for r_ in h_])
    fc = np.array([r_["force"] for r_ in h_])
    win = (tc >= T[6]) & (tc < T[7])
    peak_clear = float(fc[win].max()) if win.any() else float("inf")
    tcp = np.array(tcp_log)
    i_cut = np.searchsorted(tcp[:, 0], T[4])
    i_end = np.searchsorted(tcp[:, 0], T[7]) - 1
    moved = float(np.linalg.norm(tcp[i_end, 1:] - tcp[i_cut, 1:]))
    still = tcp[(tcp[:, 0] >= T[6] + 0.3) & (tcp[:, 0] < T[7])]
    drift = float(np.linalg.norm(still[-1, 1:] - still[0, 1:]) * 1000)
    crit["released_moved_away_still"] = {
        "pass": bool(peak_clear < CONTACT_FREE_N and moved >= 0.08 and drift < 2.0),
        "robot_object_contact_peak_N": round(peak_clear, 4),
        "window_s": [round(float(T[6]), 2), round(float(T[7]), 2)],
        "hand_moved_mm": round(moved * 1000, 1), "still_drift_mm": round(drift, 2), "threshold_N": CONTACT_FREE_N}
    pm = t_a >= T[7]
    swing = float(np.degrees(-th_a[pm]).max()) if pm.any() else 0.0
    crit["door_can_be_opened"] = {
        "pass": bool(swing >= 5.0), "probe_torque_Nm": LOCK_PROBE_NM, "probe_switches_off_at_deg": PROBE_STOP_DEG,
        "door_swing_deg": round(swing, 2), "threshold_deg": 5.0,
        "note": "after the hand has left, the same opening torque that could not move the locked leaf at t < 0.5 s "
                "now lifts it"}
    segs, grip = {}, {}
    t_rg = T_RG_OPEN + T_RG_SWING + T_RG_CLOSE + 0.2       # a regrasp opens the hand on purpose

    def pinching(t_):
        return T[2] <= t_ <= T[4] and not any(r0 <= t_ <= r0 + t_rg for r0 in st["regrasps"])
    for side in ("left", "right"):
        segs[side] = np.array([r_["force"] for r_ in rig.probes[f"fr3/fr3_{side}finger"].history
                               if pinching(r_["t"])])
        grip[f"{side}_mean_N"] = round(float(segs[side].mean()), 2)
        grip[f"{side}_p99_N"] = round(float(np.percentile(segs[side], 99)), 2)
        grip[f"{side}_contact_fraction"] = round(float(np.mean(segs[side] > 1.0)), 3)
    grip["pass"] = bool(all(np.mean(segs[s] > 1.0) > 0.85 and np.percentile(segs[s], 99) < 50.0 for s in segs))
    grip["regrasps_at_s"] = list(st["regrasps"])
    grip["note"] = ("fingertip<->key pinch during the turn (regrasps excluded): each pad loaded > 1 N for > 85 % of "
                    "it, p99 < 50 N (20 N/pad actuators)")
    crit["finger_pinch_during_turn"] = grip
    for k, v in crit.items():
        qa.report["checks"][k] = v
        qa.report["pass"] &= bool(v["pass"])
    door_penetration(qa, rig, h)
    gear_err = float(np.abs(x_a + r * ps_a).max())
    qa.report["checks"]["gear_coupling"] = {
        "pass": bool(gear_err < 3e-4), "max_error_mm": round(gear_err * 1e3, 4), "travel_per_rad_mm": round(r * 1e3, 3),
        "note": "deadbolt travel vs pinion radius x plug angle over the whole rollout (the rack-and-pinion equality)"}
    qa.report["pass"] &= bool(gear_err < 3e-4)
    report = rig.finish(qa)
    v = hl.params
    save_appendix_verdict(
        rig, task="Turn key to unlock door", source=__name__, spec=SPEC, criteria=crit, physics_qa=report, changes=[
            "final kit door (door + frame halves + doorknob and lock modules, assembled at the site's poses) lying "
            "FLAT on the table with no stand, starts CLOSED and LOCKED, key 1 fully inserted",
            f"lock module: {v['unlock_label']} variant (pinion {v['travel_per_rad_mm']} mm per rad, stop "
            f"{v['stop_deg']} deg, the deadbolt clears the jamb at {v['jamb_clear_deg']} deg); the friction spring is "
            f"{v['friction_N']} N of dry friction on the deadbolt (estimated, not measured)",
            "top-down pinch on the key bow, clockwise turn until the measured deadbolt clears the strike jamb, "
            "release, 12 cm rise, stop",
            "hidden lock probe at t < 0.5 s (locked leaf must not move) and a door-can-be-opened probe after the hand "
            f"left (same torque must lift it >= 5 deg); a website clip should end at t = {t_clip_end:.2f} s, before "
            "that probe",
            "key welded into the plug (turn-only task); the printed rack and pinion is a joint equality",
            "the doorknob module's latch is held back at its stop (scenery), so the deadbolt alone gates the leaf",
            *[f"door model: {d}" for d in h.info["deviations"]],
        ])
    return report


if __name__ == "__main__":
    wp.init()
    main()
