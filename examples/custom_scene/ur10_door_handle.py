"""A ContactBench kit mechanism, loaded from its USD stage, in a scene of your own: a UR10 from Newton's asset
repository pushes the lever handle of the kit's doorknob module, which turns the spindle against its printed spring
and pulls the latch in.

    force-bench kit export-usd doorknob-module --variant "Handle 1"     # writes the stage this script loads
    python examples/custom_scene/ur10_door_handle.py                    # or --stage <path to a doorknob-module stage>

Nothing here is ContactBench's FR3 rig: a plain ``newton.ModelBuilder``, the robot from ``newton.utils.download_asset``
added with ``add_usd``, Newton's own IK, a plain ``SolverMuJoCo`` with the settings the kit was calibrated with.  The
UR10 carries a push rod on its tool flange and pushes the handle with it in a straight line 85 mm from the spindle, as
in the measurement the spring was fitted to (a UR arm with a wrist F/T sensor, examples/force_response), until the
handle is at its stop, holds, and backs off.  The script prints the handle angle, the latch travel, the force between
rod and handle, and the torque that force puts on the spindle next to the calibrated spring's, tau0 + K * angle.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import warp as wp

import newton
import newton.ik as ik
import newton.utils
from force_bench.kit.usd import add_asset_usd, read_meta

LEVER = 0.085                       # m, the push line's distance from the spindle
SPINDLE = np.array([0.85, 0.0])     # world xy of the spindle; the UR10's base is at the origin
ROD_RADIUS, ROD_LENGTH = 0.006, 0.100   # m, the push rod on the tool flange, along the tool axis
TIP_Z = 0.055                       # m, height of the rod's tip above the table (the handle bar is 67-78 mm)
EE_Z = TIP_Z + ROD_LENGTH           # m, height of the tool flange
SPEED = 0.025                       # m/s along the push line
X_START, X_END = -0.030, 0.065      # m, rod axis along the push direction (module x): from 16 mm clear of the bar
                                    # to past where the handle meets its stop; the push stops at the stop
STOP_MARGIN = np.radians(0.3)       # the push stops this short of the handle's stop (a stiff arm against a hard
SLOW = np.radians(4.0)              # stop only builds force), and slows to a fifth of SPEED this close to it


def rot_z(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0, 0.0], [s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])


def add_ur10(builder: newton.ModelBuilder) -> None:
    path = newton.utils.download_asset("universal_robots_ur10") / "usd" / "ur10_instanceable.usda"
    builder.add_usd(str(path), xform=wp.transform(wp.vec3(0.0, 0.0, 0.001), wp.quat_identity()),
                    collapse_fixed_joints=False, enable_self_collisions=False, hide_collision_shapes=True)


def add_push_rod(builder: newton.ModelBuilder) -> None:
    """A mounting plate and a rod on the UR10's tool flange (``ee_link``, whose +x is the tool axis)."""
    ee = builder.body_label.index("/ur10/ee_link")
    along_x = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), np.pi / 2)      # a cylinder's axis is its local z
    builder.add_shape_cylinder(ee, xform=wp.transform(wp.vec3(0.005, 0.0, 0.0), along_x), radius=0.032,
                               half_height=0.005, color=(0.25, 0.26, 0.28), label="rod_plate")
    builder.add_shape_cylinder(ee, xform=wp.transform(wp.vec3(ROD_LENGTH / 2, 0.0, 0.0), along_x), radius=ROD_RADIUS,
                               half_height=ROD_LENGTH / 2, color=(0.72, 0.74, 0.77), label="push_rod")


def main(stage: str, seconds: float | None = None) -> dict:
    wp.init()
    meta = read_meta(stage)
    cal = meta["calibration"]
    fps, substeps = cal["fps"], cal["substeps"]               # 60 x 32: the timestep the kit was calibrated at
    dt = 1.0 / (fps * substeps)

    # ── the scene: a table, the robot, the kit asset ──────────────────────────────────────────────────────────────
    builder = newton.ModelBuilder()
    newton.solvers.SolverMuJoCo.register_custom_attributes(builder)
    builder.add_shape_box(-1, xform=wp.transform(wp.vec3(0.4, 0.0, -0.01), wp.quat_identity()), hx=0.8, hy=0.6,
                          hz=0.01, label="table")
    add_ur10(builder)
    add_push_rod(builder)
    robot_bodies = list(range(builder.body_count))
    n_robot_dofs = builder.joint_dof_count
    ke = [3.0e4] * 3 + [3.0e3] * 3                           # stiff position servos (N.m/rad)
    kd = [1.0e3] * 3 + [30.0] * 3
    for i in range(n_robot_dofs):
        builder.joint_target_ke[i], builder.joint_target_kd[i] = ke[i], kd[i]
        builder.joint_target_mode[i] = int(newton.JointTargetMode.POSITION)
        builder.joint_armature[i] = 0.1
    # the module lies on the table, spindle up, its lever pointing at the robot (module +y = world -x), so that
    # pushing along module +x (world +y) turns the handle clockwise and retracts the latch
    pose = np.eye(4)
    pose[:3, 3] = [SPINDLE[0], SPINDLE[1], 0.0]
    pose = pose @ rot_z(np.pi / 2)
    h = add_asset_usd(builder, stage, pose=pose)                 # the CAD frame (spindle axis, housing bottom) at pose
    if h.key != "doorknob-module":
        raise SystemExit(f"{stage} is a {h.key} stage; this script pushes a doorknob-module's handle")

    # ── IK for the UR10's flange (a robot-only model): straight line at the handle's height, tool pointing down ──
    robot = newton.ModelBuilder()
    add_ur10(robot)
    ik_model = robot.finalize()
    ee = ik_model.body_label.index("/ur10/ee_link")
    ik_state = ik_model.state()
    q_home = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0], dtype=np.float32)
    ik_model.joint_q.assign(q_home)
    newton.eval_fk(ik_model, ik_model.joint_q, ik_model.joint_qd, ik_state)
    down = wp.transform_get_rotation(wp.transform(*ik_state.body_q.numpy()[ee]))   # flange orientation: tool down
    R = pose[:3, :3]

    def target(x_mod: float) -> np.ndarray:                                 # rod axis at module (x, LEVER)
        p = pose[:3, 3] + R @ np.array([x_mod, LEVER, 0.0])
        return np.array([p[0], p[1], EE_Z])

    pos_obj = ik.IKObjectivePosition(link_index=ee, link_offset=wp.vec3(0.0, 0.0, 0.0),
                                     target_positions=wp.array([wp.vec3(*target(X_START))], dtype=wp.vec3))
    rot_obj = ik.IKObjectiveRotation(link_index=ee, link_offset_rotation=wp.quat_identity(),
                                     target_rotations=wp.array([wp.vec4(down[0], down[1], down[2], down[3])],
                                                               dtype=wp.vec4))
    ik_q = ik_model.joint_q.reshape((1, ik_model.joint_coord_count))
    ik_solver = ik.IKSolver(model=ik_model, n_problems=1, objectives=[pos_obj, rot_obj],
                            lambda_initial=0.1, jacobian_mode=ik.IKJacobianType.ANALYTIC)
    ik_solver.step(ik_q, ik_q, iterations=200)

    def arm_targets(x_mod: float) -> np.ndarray:
        pos_obj.set_target_position(0, wp.vec3(*target(x_mod)))
        ik_solver.step(ik_q, ik_q, iterations=16)
        return ik_q.numpy()[0, :n_robot_dofs]

    builder.joint_q[:n_robot_dofs] = arm_targets(X_START).tolist()     # the arm starts there, at rest
    builder.joint_target_q[:n_robot_dofs] = builder.joint_q[:n_robot_dofs]

    # ── model, solver with the calibration's settings ─────────────────────────────────────────────────────────────
    model = builder.finalize()
    h.prepare_model(model)                                       # contact stiffness, damping, condim of the calibration
    state_0, state_1, control = model.state(), model.state(), model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state_0)
    # contact force on the spindle body from each robot link, and where it acts (force-weighted contact position)
    sensor = newton.sensors.SensorContact(model, sensing_bodies=[h.bodies["knob"]], counterpart_bodies=robot_bodies,
                                          measure_total=True)
    pipeline = newton.CollisionPipeline(model, rigid_contact_max=20000, reduce_contacts=True, broad_phase="sap")
    contacts = pipeline.contacts()
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, **h.solver_options())
    h.after_solver(model, solver)                                # warns where the solver differs from the calibration
    h.check_scene(dt=dt, collide_every_substep=True)

    # ── run: push until the handle is at its stop, hold, back off ─────────────────────────────────────────────────
    qs = model.joint_q_start.numpy()
    i_knob, i_latch = int(qs[h.joints["knob"]]), int(qs[h.joints["latch"]])
    K, tau0, stop = h.params["K_Nm_per_rad"], h.params["tau0_Nm"], np.radians(h.params["stop_deg"])
    spindle = np.asarray(h.info["spindle_point_world"])
    axis = np.asarray(h.info["spindle_axis_world"])
    x, phase, t_hold, total = X_START, "push", None, seconds or 8.0
    rows, t = [], 0.0
    print(f"{h.key} ({Path(stage).name}): K = {K} N.m/rad, tau0 = {tau0} N.m, stop {h.params['stop_deg']} deg, "
          f"{fps} fps x {substeps} substeps")
    print("   t [s]  handle [deg]  latch [mm]  push force [N]  torque on spindle [N.m]  spring tau0 + K a [N.m]")
    for frame in range(int(total * fps)):
        angle = -float(state_0.joint_q.numpy()[i_knob])              # clockwise (unlocking) = positive
        if phase == "push":
            x = min(X_END, x + SPEED / fps * (0.2 if angle > stop - SLOW else 1.0))
            if angle >= stop - STOP_MARGIN or x >= X_END:
                phase, t_hold = "hold", t
        elif phase == "hold" and t >= t_hold + 0.5:
            phase = "back"
        elif phase == "back":
            x = max(X_START, x - SPEED / fps)
            if x <= X_START and t > t_hold + 1.5:
                phase = "rest"
        qt = control.joint_target_q.numpy()
        qt[:n_robot_dofs] = arm_targets(x)
        control.joint_target_q.assign(qt)
        for _ in range(substeps):
            state_0.clear_forces()
            pipeline.collide(state_0, contacts)
            solver.step(state_0, state_1, control, contacts, dt)
            state_0, state_1 = state_1, state_0
        t += 1.0 / fps
        solver.update_contacts(contacts, state_0)
        sensor.update(state_0, contacts)
        f = sensor.force_matrix.numpy()[0]                              # (robot links, 3), on the spindle body
        p = sensor.position_matrix.numpy()[0]
        torque = -float(sum(np.cross(pi - spindle, fi) @ axis for pi, fi in zip(p, f)))     # clockwise = positive
        force = float(np.linalg.norm(f.sum(axis=0)))
        q = state_0.joint_q.numpy()
        angle = -float(q[i_knob])
        rows.append((t, np.degrees(angle), float(q[i_latch]) * 1e3, force, torque, tau0 + K * max(angle, 0.0),
                     phase == "push"))
        if frame % (fps // 4) == 0:
            print(f"  {t:5.2f}  {np.degrees(angle):11.2f}  {q[i_latch] * 1e3:10.2f}  {force:14.2f}  {torque:23.3f}  "
                  f"{tau0 + K * max(angle, 0.0):23.3f}")
        if phase == "rest" and abs(angle) < 1e-3:
            break
    a = np.array(rows)
    turning = a[(a[:, 6] > 0) & (a[:, 1] > 2.0) & (a[:, 1] < np.degrees(stop) - 1.0)]
    res = {"max_handle_deg": round(float(a[:, 1].max()), 2), "max_latch_mm": round(float(a[:, 2].max()), 2),
           "final_handle_deg": round(float(a[-1, 1]), 2), "final_latch_mm": round(float(a[-1, 2]), 2)}
    if len(turning):
        res["spindle_torque_over_spring_median"] = round(float(np.median(turning[:, 4] / turning[:, 5])), 3)
    print(res)
    print("latch retracted past the jamb (15.24 mm)" if res["max_latch_mm"] >= 15.24 else "latch NOT retracted",
          "and returned" if abs(res["final_latch_mm"]) < 0.5 else "and did not return")
    return res


if __name__ == "__main__":
    from force_bench.paths import OUTPUTS_ROOT
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stage", default=str(OUTPUTS_ROOT / "kit_usd" / "doorknob-module" / "Handle_1.usda"))
    ap.add_argument("--seconds", type=float, default=None, help="simulated time (default: push, hold, return)")
    a = ap.parse_args()
    if not Path(a.stage).exists():
        raise SystemExit(f"{a.stage} not found: run  force-bench kit export-usd doorknob-module --variant 'Handle 1'")
    main(a.stage, a.seconds)
