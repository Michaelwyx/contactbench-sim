"""Franka FR3 robot cell for ContactBench: dynamic arm + kinematic IK.

Pattern:
  * the SCENE model contains the FR3 (added FIRST -> coords 0..8) plus the
    assets; arm joints get position drives (target_ke/target_kd)
  * a SEPARATE single-robot kinematic model runs newton.ik each frame; the
    solved arm coordinates become the scene drives' position targets
  * so the arm is fully dynamic — asset contact forces push back through
    the wrist — while teleop/trajectories speak TCP poses

Also supports UR10 (USD import, no gripper) for teleop demos.
"""

from __future__ import annotations

import numpy as np
import warp as wp

import newton
import newton.ik as ik
import newton.utils

import os

FR3_HOME = np.array([0.0, -0.5, 0.0, -2.2, 0.0, 1.75, 0.785, 0.04, 0.04])
# DIAGNOSTIC ONLY -- NOT A FIX.  The FR3 URDF's real effort limits are
# [87, 87, 87, 87, 12, 12, 12] N.m, and XPBD ignores joint_effort_limit
# entirely, so under XPBD this arm has always been infinitely strong.  Under
# MuJoCo the limits bind for the first time, which is physically right and
# which no task was ever calibrated against.  TB_ARM_EFFORT overrides them
# so a single A/B can say whether a tracking deficit IS the effort limit.
# If it is, the honest fix is a slower or lighter task, never this knob.
ARM_EFFORT_OVERRIDE = os.environ.get("TB_ARM_EFFORT")
ARM_KE = 12000.0   # N·m/rad position drive (stiff industrial gains)
ARM_KD = 200.0
FINGER_KE = 1200.0  # N/m
FINGER_KD = 20.0
TCP_BODY = "fr3/fr3_hand_tcp"


def add_franka(builder: newton.ModelBuilder, base_xform: wp.transform) -> None:
    """Add a drive-controlled FR3 to the scene builder (must be added FIRST).

    collapse_fixed_joints merges the massless frame links (fr3_hand_tcp,
    link8, hand) into fr3_link7 — welding payloads to a massless frame body
    makes the dynamics singular (NaN).
    """
    urdf = newton.utils.download_asset("franka_emika_panda") / "urdf/fr3_franka_hand.urdf"
    builder.add_urdf(urdf, xform=base_xform, floating=False, collapse_fixed_joints=True)
    # position drives on all 9 dofs
    for d in range(7):
        builder.joint_target_ke[d] = ARM_KE
        builder.joint_target_kd[d] = ARM_KD
    for d in (7, 8):
        builder.joint_target_ke[d] = FINGER_KE
        builder.joint_target_kd[d] = FINGER_KD
    if ARM_EFFORT_OVERRIDE:
        for d in range(7):
            builder.joint_effort_limit[d] = float(ARM_EFFORT_OVERRIDE)
    builder.joint_q[:9] = FR3_HOME.tolist()
    # drive targets start at home
    builder.joint_target_q[:9] = FR3_HOME.tolist()


class FrankaIK:
    """Kinematic IK on a private FR3 model; returns 9 joint coords."""

    def __init__(self, base_xform: wp.transform):
        b = newton.ModelBuilder()
        urdf = newton.utils.download_asset("franka_emika_panda") / "urdf/fr3_franka_hand.urdf"
        b.add_urdf(urdf, xform=base_xform, floating=False)
        b.joint_q[:9] = FR3_HOME.tolist()
        self.model = b.finalize()
        self.ee_index = list(self.model.body_label).index(TCP_BODY)

        state = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, state)
        tcp0 = state.body_q.numpy()[self.ee_index]

        self.pos_obj = ik.IKObjectivePosition(
            link_index=self.ee_index,
            link_offset=wp.vec3(0.0, 0.0, 0.0),
            target_positions=wp.array([wp.vec3(*tcp0[:3])], dtype=wp.vec3),
        )
        self.rot_obj = ik.IKObjectiveRotation(
            link_index=self.ee_index,
            link_offset_rotation=wp.quat_identity(),
            target_rotations=wp.array([wp.vec4(*tcp0[3:7])], dtype=wp.vec4),
        )
        limits = ik.IKObjectiveJointLimit(
            joint_limit_lower=self.model.joint_limit_lower,
            joint_limit_upper=self.model.joint_limit_upper,
            weight=10.0,
        )
        self.joint_q = self.model.joint_q.reshape((1, self.model.joint_coord_count))
        self.solver = ik.IKSolver(
            model=self.model,
            n_problems=1,
            objectives=[self.pos_obj, self.rot_obj, limits],
            lambda_initial=0.1,
            jacobian_mode=ik.IKJacobianType.ANALYTIC,
        )
        self.home_tcp = tcp0.copy()

    def solve(self, target_pos, target_quat, iterations: int = 16) -> np.ndarray:
        self.pos_obj.set_target_position(0, wp.vec3(*np.asarray(target_pos, dtype=float)))
        q = np.asarray(target_quat, dtype=float)
        self.rot_obj.set_target_rotation(0, wp.vec4(*q))
        self.solver.step(self.joint_q, self.joint_q, iterations=iterations)
        return self.joint_q.numpy()[0].copy()

    def tcp_pose(self) -> np.ndarray:
        state = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, state)
        return state.body_q.numpy()[self.ee_index].copy()

    def body_pose(self, label: str) -> np.ndarray:
        state = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, state)
        return state.body_q.numpy()[list(self.model.body_label).index(label)].copy()


def down_grasp_quat(yaw: float = 0.0) -> np.ndarray:
    """TCP orientation: z_tcp pointing down (fingers toward table), yaw about world z."""
    q_flip = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), np.pi)  # z -> -z
    q_yaw = wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), float(yaw))
    q = wp.mul(q_yaw, q_flip)
    return np.array([q[0], q[1], q[2], q[3]])
