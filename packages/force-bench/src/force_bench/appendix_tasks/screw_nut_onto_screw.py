"""Appendix task "Screw nut onto screw" (Table I).

    Description: Screw the nut held in hand onto the screw head placed in
    the workspace such that their threads fully engage.  Success: the robot
    movement stops after terminating the rotation motion, moving away from,
    and breaking contact with the task objects.

A fork of ``force_bench.tasks.nut_bolt_arm`` (unchanged on disk): the same
printed M-size bolt (static, shaft up) and free hex nut on the real SDF
thread, and the suite's own regrasp-stroke controller (``RegraspDriver``:
lag-limited wrist sweeps, helix-following TCP z, admittance grip) and
plan builder.  What differs:

- START IN HAND.  The suite nut starts ALREADY threaded 1.3 turns above its
  seat with the fingers open around it, and the job is tighten-then-loosen.
  Here the nut starts in the closed pinch at the top of the bolt with only
  its bottom ~1.5 mm caught on the first thread (0.6 turn) -- the state a
  hand is in right after starting a nut by feel.  The pose is helix-
  consistent (same mate-yaw bake the suite uses), and the wrist starts at
  the yaw that puts the pads on two flats.  (Starting the thread from fully
  off the bolt is not attempted: the 0.23 mm thread clearance needs a
  coaxial search the suite has no controller for.)
- TIGHTEN-WAY STROKES ONLY until the threads are fully engaged -- the
  whole 7.0 mm nut below the bolt's thread top -- no seating press, no
  loosen block.
- ENDING: the plan's own open + 10 mm lift, then a further 6 cm retreat
  and stop.  The nut must stay where it was left (no back-off, no fall)
  and nothing of the robot may touch nut or bolt.

Run: python -m force_bench.appendix_tasks.screw_nut_onto_screw
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

from force_bench.appendix_tasks.common import (
    appendix_rig, RobotContactProbe, ease, check_released_and_clear,
    save_appendix_verdict)
from force_bench.probes import ForceProbe
from force_bench.rig import make_solver
from force_bench.robot.franka import down_grasp_quat
from force_bench.tasks import nut_bolt_arm as NA
from force_bench.tasks.nut_bolt import (
    load_subdivided, LEAD, BOLT_WORLD_C, MATE_ZC, MATE_YAW_DEG, SDF_RES, SDF_NB,
    SDF_MARGIN)
from force_bench.tasks.nut_bolt_arm import PairProbe, RegraspDriver, build_plan
from force_bench.tasks.wrench_nut_arm import hex_flat_phase

NAME = "screw_nut_onto_screw"
Z_START = 0.0290            # nut centre: bottom 1.5 mm onto the first thread
THREAD_TOP = 0.0270         # m, mesh-measured top of the full-radius thread
NUT_H = 0.00702
ENGAGED_Z = THREAD_TOP - NUT_H / 2      # nut centre at full engagement
N_STROKES = 7
FING_TOUCH = 0.00785        # 15.50 mm flats + 0.17 mm pad offset, /2
T_GRIP0 = 0.4
T_RETREAT, T_STILL = 1.2, 1.2
RETREAT = 0.06
FLOOR_DROP = 0.0090         # the TCP follows the nut ~6.5 mm down


def main():
    bake_yaw = (MATE_YAW_DEG + (Z_START - MATE_ZC) / LEAD * 360.0) % 360.0
    rig = appendix_rig(NAME)
    base_xf = wp.transform(wp.vec3(0.0, 0.325, 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    for d in (7, 8):
        rig.builder.joint_target_ke[d] = NA.FINGER_KE
        rig.builder.joint_target_kd[d] = NA.FINGER_KD
    rig.table(center_xy=(0.0, 0.16), half=0.35)

    from force_bench.assets import make_newton_mesh, set_body_mass
    bolt_tm = load_subdivided("fasteners", "screw_075mm_1_body1")
    bolt_mesh, bolt_c = make_newton_mesh(bolt_tm, sdf=True, sdf_resolution=SDF_RES,
                                         narrow_band=SDF_NB, margin=SDF_MARGIN)
    bolt_sid = rig.builder.add_shape_mesh(
        -1, mesh=bolt_mesh, xform=wp.transform(wp.vec3(*BOLT_WORLD_C), wp.quat_identity()),
        cfg=rig.cfg(mu=NA.NUT_MU, gap=NA.GAP), label="bolt")
    rig.shapes["bolt"] = [bolt_sid]
    bolt_world_tm = bolt_tm.copy()
    bolt_world_tm.apply_translation(BOLT_WORLD_C - np.asarray(bolt_c))
    rig.meshes["bolt"] = (bolt_world_tm, BOLT_WORLD_C)

    nut_tm = load_subdivided("fasteners", "nut_075mm_1_body1")
    nut_tm.apply_translation(-(nut_tm.bounds[0] + nut_tm.bounds[1]) / 2)
    nut_tm.apply_transform(trimesh.transformations.rotation_matrix(
        np.radians(bake_yaw), [0, 0, 1]))
    flat = hex_flat_phase(nut_tm)               # a flat normal, world
    nut_mesh, nut_c = make_newton_mesh(nut_tm, sdf=True, sdf_resolution=SDF_RES,
                                       narrow_band=SDF_NB, margin=SDF_MARGIN)
    nut_pos = np.array([0.0, 0.0, Z_START])
    nut_body = rig.builder.add_body(
        xform=wp.transform(wp.vec3(*nut_pos), wp.quat_identity()), label="nut")
    nut_sid = rig.builder.add_shape_mesh(nut_body, mesh=nut_mesh,
                                         cfg=rig.cfg(mu=NA.NUT_MU, gap=NA.GAP),
                                         label="nut_shape")
    rig.shapes["nut"] = [nut_sid]
    set_body_mass(rig.builder, nut_body, 0.01)
    nut_world_tm = nut_tm.copy()
    nut_world_tm.apply_translation(nut_pos - np.asarray(nut_c))
    rig.meshes["nut"] = (nut_world_tm, nut_pos)

    # wrist yaw that puts the pads on two FLATS: down_grasp_quat(y) closes
    # along world y + 90 deg, the flats repeat every 60 deg; pick the one
    # nearest the suite's start yaw so the stroke window is the suite's
    base = flat - np.pi / 2 - NA.YAW_BIAS
    k = np.round((NA.YAW0 - base) / (np.pi / 3))
    yaw_init = float(base + k * np.pi / 3)
    if yaw_init > NA.YAW0 + 0.35:          # keep inside the stroke clip window
        yaw_init -= np.pi / 3
    tcp0 = np.array([0.0, 0.0, Z_START + NA.TCP_OFF])
    rig.solve_arm_start(tcp0, down_grasp_quat(yaw_init + NA.YAW_BIAS))
    rig.builder.joint_q[7:9] = [FING_TOUCH, FING_TOUCH]
    rig.builder.joint_target_q[7:9] = [FING_TOUCH, FING_TOUCH]

    cam_c = np.array([0.0, 0.0, 0.022])
    rig.finalize(probes=["nut"], cam_pos=cam_c + np.array([0.07, -0.10, 0.055]),
                 cam_target=cam_c)
    rig.probes = {
        "seat": PairProbe(rig.model, "nut", counterpart_shapes=["bolt"]),
        "finL": ForceProbe(rig.model, sensing_body="fr3/fr3_leftfinger",
                           counterpart_bodies=["nut"]),
        "finR": ForceProbe(rig.model, sensing_body="fr3/fr3_rightfinger",
                           counterpart_bodies=["nut"]),
        "robot_object": RobotContactProbe(rig.model, ["nut"], static_shapes=["bolt"]),
    }
    rig.solver = make_solver(rig.model, iterations=20)
    ni = rig.body_index("nut")
    wi = rig.body_index("fr3/fr3_link7")

    # no settle (the nut is already in the hand), tighten-way strokes only
    plan, t_plan = build_plan([(-1.0, N_STROKES, False)], t_settle=0.0)
    print("plan:", [(s["kind"], round(s["t0"], 2)) for s in plan], "total", round(t_plan, 2))
    drv = RegraspDriver(rig, plan, tcp0, ni, wi, part_z0=float(nut_pos[2]),
                        yaw_init=yaw_init, finger_grip=NA.FINGER_GRIP,
                        latch_mm=99.0, floor_drop=FLOOR_DROP, f_seat=NA.F_SEAT,
                        lead_m=LEAD, axis_xy=(0.0, 0.0), yaw_bias=NA.YAW_BIAS,
                        f_grip_max=NA.F_GRIP_MAX, f_grip_ok=NA.F_GRIP_OK,
                        relax_open_max=NA.RELAX_OPEN_MAX,
                        relax_close_max=NA.RELAX_CLOSE_MAX, relax_rate=NA.RELAX_RATE)
    drv.ctl["fingers"] = FING_TOUCH              # already closed on the flats
    duration = t_plan + T_RETREAT + T_STILL
    end = {"p": None}

    def control(t):
        if t < t_plan:
            drv.control(t)
            return
        if end["p"] is None:
            tcp = rig.ik.tcp_pose()
            end["p"], end["q"] = tcp[:3].copy(), tcp[3:7].copy()
            end["f"] = float(rig.control.joint_target_q.numpy()[7])
        u = ease((t - t_plan) / T_RETREAT)
        rig.drive_arm(end["p"] + np.array([0.0, 0.0, RETREAT * u]), end["q"],
                      fingers=end["f"])

    log = []

    def extra(state, t):
        out = drv.extra(state, t) if t < t_plan and "_ny" in drv.ctl else {}
        bq = state.body_q.numpy()
        rc = rig.probes["robot_object"].history[-1]["force"] \
            if rig.probes["robot_object"].history else 0.0
        log.append((t, float(bq[ni, 2]), rc,
                    rig.probes["finL"].history[-1]["force"] if rig.probes["finL"].history else 0.0,
                    rig.probes["finR"].history[-1]["force"] if rig.probes["finR"].history else 0.0))
        out.update({"nut_z_mm": round(float(bq[ni, 2]) * 1000, 3),
                    "robot_object_contact_N": round(rc, 4)})
        return out

    rec = rig.run(duration, control, extra)
    paths = rec.save("rollout", force_keys=["seat_N", "finL_N", "finR_N",
                                            "robot_object_contact_N"])
    print("saved:", paths)

    qa = rig.qa(paths)
    L = np.array(log)
    t_a, z_a, rc_a, fl, fr = (L[:, k] for k in range(5))
    turns = (Z_START - float(z_a[t_a < t_plan].min())) / LEAD
    g0 = t_a < T_GRIP0 + 0.3
    strokes_m = np.zeros_like(t_a, bool)
    for s in plan:
        if s["kind"] == "stroke":
            strokes_m |= (t_a >= s["t0"]) & (t_a < s["t1"])
    still = t_a >= t_plan + T_RETREAT
    z_rel = float(np.median(z_a[(t_a >= t_plan - 0.3) & (t_a < t_plan)]))
    crit = {
        "held_in_hand_at_start": {
            "pass": bool(min(fl[g0].max(), fr[g0].max()) > 1.0
                         and abs(z_a[g0].min() - Z_START) < 0.001),
            "pad_peak_N": [round(float(fl[g0].max()), 2), round(float(fr[g0].max()), 2)],
            "start_z_mm": Z_START * 1000, "start_engagement_turns":
                round((THREAD_TOP - (Z_START - NUT_H / 2)) / LEAD, 2)},
        "threads_fully_engaged": {
            "pass": bool(float(z_a[-1]) <= ENGAGED_Z),
            "final_nut_centre_mm": round(float(z_a[-1]) * 1000, 2),
            "engaged_when_centre_below_mm": round(ENGAGED_Z * 1000, 2),
            "turns_screwed": round(turns, 2),
            "per_stroke": [{k: round(v, 3) if isinstance(v, float) else v
                            for k, v in s.items()} for s in drv.strokes]},
        "nut_stays_put_after_release": {
            "pass": bool(abs(float(z_a[-1]) - z_rel) < 0.0003),
            "z_before_release_mm": round(z_rel * 1000, 3),
            "final_z_mm": round(float(z_a[-1]) * 1000, 3)},
        "real_finger_grasp": {
            "pass": bool(fl[strokes_m].mean() > 1.0 and fr[strokes_m].mean() > 1.0),
            "mean_pad_N_in_strokes": [round(float(fl[strokes_m].mean()), 2),
                                      round(float(fr[strokes_m].mean()), 2)]},
    }
    check_released_and_clear(qa, t_a, rc_a, t_plan + T_RETREAT)
    crit["released_and_clear"] = qa.report["checks"]["released_and_clear"]
    for k, c in crit.items():
        qa.report["checks"].setdefault(k, c)
    qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
    nut_local = rig.meshes["nut"][0].copy()
    nut_local.apply_translation(-nut_pos)
    qa.check_penetration_static("nut", nut_local, rig.meshes["bolt"][0],
                                np.zeros(3), "nut->bolt", tol=0.0015)
    report = rig.finish(qa)
    save_appendix_verdict(
        rig, task="Screw nut onto screw",
        source="force_bench.tasks.nut_bolt_arm (scene + RegraspDriver reused)",
        spec="The robot movement stops after terminating the rotation motion, moving "
             "away from, and breaking contact with the task objects (nut screwed on "
             "such that the threads fully engage).",
        criteria=crit, physics_qa=report,
        changes=[
            "nut starts IN THE PINCH at the bolt top, 0.6 turn on the first thread "
            "(suite: already threaded 1.3 turns above the seat, fingers open)",
            f"tighten-way regrasp strokes only ({N_STROKES}), no seating press, no "
            "loosen block",
            "ending: open + 1 cm lift (plan) + 6 cm retreat + still",
            "verdict: whole nut below the thread top, nut stays put, robot clear",
        ])


if __name__ == "__main__":
    wp.init()
    main()
