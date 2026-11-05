"""Interactive Franka teleoperation over the ContactBench assets.

A dynamic FR3 tracks a mouse-draggable TCP gizmo (or keyboard jog) through
inverse kinematics, in a scene with the REAL card_swiper and outlet meshes
plus a free card and charger plug to play with. Finger drives give a
physical pinch grasp; contact forces on the fingers print live.

TELEOP GUARD (packages/force-bench/src/force_bench/teleop/guard.py) is wired in and ON by default.  Every
task in this repo is a scripted expert whose trajectory is co-designed with
the contact model — rate-limited approaches, force-servoed presses, gentle
grips — and a human violates all of that in the first few seconds.  The
guard sits between the operator's command and the robot:

  * a CONTACT-AWARE SPEED GOVERNOR clamps the per-frame TCP step and the
    command's lead over the measured TCP to a fraction of the smallest
    feature near the gripper (this scene's card slot is 2.1 mm for a 1.2 mm
    card, so ~0.5 mm/frame there against 4.2 mm/frame in free air);
  * an ADMITTANCE FILTER (EMA + deadband) smooths the input;
  * a FORCE-SERVOED press budget bleeds the lead — and drives it negative —
    when the measured hand contact force exceeds the budget;
  * a FORCE-LIMITED finger drive keeps the squeeze inside the measured
    human grasp window;
  * LIVE QA runs the RolloutQA checks online and prints violations as they
    happen instead of after the session.

Controls (GL viewer):
  drag "tcp_target" gizmo   6-DoF TCP control
  W/S A/D R/F               jog x/y/z          Q/E  yaw
  G / H                     close / open fingers
  T                         toggle the teleop guard (A/B it live)

Run:            force-bench teleop --viewer gl
                force-bench teleop --viewer gl --no-guard
Headless test:  force-bench teleop --viewer null --num-frames 90 --test
Record LfD:     force-bench teleop --viewer gl --collect-dir outputs/datasets/teleop
"""

from __future__ import annotations

import os
import sys

import numpy as np
import warp as wp

import newton
import newton.examples

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))
from force_bench.probes import ForceProbe  # noqa: E402
from force_bench.rig import make_solver  # noqa: E402
from force_bench.robot.franka import FrankaIK, add_franka, down_grasp_quat  # noqa: E402
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass  # noqa: E402
from force_bench.teleop.guard import FeatureMap, GuardCfg, TeleopGuard  # noqa: E402

MOVE_SPEED = 0.15
YAW_SPEED = 1.5
JAW_SPEED = 0.05
CFG = dict(ke=1.0e5, kd=1.0e3, gap=0.0008, mu_torsional=0.0, mu_rolling=0.0)

# Smallest mechanical feature per region, in METRES.  These are CAD facts,
# not tuning knobs -- the governor cannot infer them from the solver:
#   card slot   2.1 mm of slot for a 1.2 mm card plus a 0.82 mm pad blade,
#               i.e. the part has to be held to a ~0.04 mm lateral window
#               (README pitfall 8.2)
#   card        the 1.2 mm card itself: the thinnest thing in the scene, and
#               the one a fast drive can punch a fingertip straight through
#               (README pitfall 6.1)
#   plug        2.5 mm face-plate slots, 0.45 mm of blade play per side
FEATURE_SLOT = 0.0021
FEATURE_CARD = 0.0012
FEATURE_PLUG = 0.0025


class _CollectionRigView:
    """Read-only adapter to the common demonstration observation/action schema."""
    name = "teleop_franka"
    solver_options = {}

    def __init__(self, example):
        self.example = example

    @property
    def s0(self):
        return self.example.state_0

    @property
    def substeps(self):
        return self.example.sim_substeps

    def effective_collide_every_substep(self):
        return True

    def __getattr__(self, name):
        return getattr(self.example, name)


class Example:
    def __init__(self, viewer, args=None):
        self.fps = 60
        self.frame_dt = 1.0 / self.fps
        self.sim_substeps = 16
        self.sim_dt = self.frame_dt / self.sim_substeps
        self.sim_time = 0.0
        self.viewer = viewer

        base_xf = wp.transform(wp.vec3(0.01, -0.50, 0.0), wp.quat_identity())
        self.ik = FrankaIK(base_xf)

        # coordinate-layout targets: franka occupies coords 0..8 either way,
        # but the free bodies (card/plug) make the layouts differ beyond that
        newton.use_coord_layout_targets = True

        builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
        add_franka(builder, base_xf)
        builder.add_ground_plane()

        pla = newton.ModelBuilder.ShapeConfig(mu=0.5, density=1240.0, **CFG)
        builder.add_shape_box(
            -1, xform=wp.transform(wp.vec3(0.05, -0.05, -0.005), wp.quat_identity()),
            hx=0.45, hy=0.40, hz=0.005, cfg=pla, label="table",
        )

        # card swiper (housing static + spring-replaced pad)
        housing_tm = load_tm("card_swiper", "part1_1_body1")
        pad_tm = load_tm("card_swiper", "part2_1_body1")
        housing_mesh, housing_c = make_newton_mesh(housing_tm, sdf=True, sdf_resolution=384)
        pad_mesh, pad_c = make_newton_mesh(pad_tm, sdf=True, sdf_resolution=256)
        builder.add_shape_mesh(
            -1, mesh=housing_mesh,
            xform=wp.transform(wp.vec3(*housing_c), wp.quat_identity()),
            cfg=pla, label="housing",
        )
        pad_xf = wp.transform(wp.vec3(*pad_c), wp.quat_identity())
        pad_body = builder.add_link(xform=pad_xf, label="pad")
        builder.add_shape_mesh(pad_body, mesh=pad_mesh, cfg=pla, label="pad_shape")
        pad_joint = builder.add_joint_prismatic(
            parent=-1, child=pad_body, axis=(0.0, 1.0, 0.0),
            parent_xform=pad_xf, child_xform=wp.transform_identity(),
            target_pos=0.0, target_ke=7000.0, target_kd=8.0,
            limit_lower=0.0, limit_upper=0.004,
        )
        builder.add_articulation([pad_joint])
        set_body_mass(builder, pad_body, 0.03)

        # free card on the table in front of the robot
        card_tm = load_tm("card_swiper", "card_v2_1_part4_1_body1")
        card_mesh, card_c = make_newton_mesh(card_tm, sdf=True, sdf_resolution=384,
                                             narrow_band=0.0015, margin=0.001)
        card_ext = card_tm.extents
        self.card_body = builder.add_body(
            xform=wp.transform(
                wp.vec3(0.15, -0.20, card_ext[1] / 2 + 0.001),
                wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), np.pi / 2),
            ),  # lying flat
            label="card",
        )
        builder.add_shape_mesh(
            self.card_body, mesh=card_mesh,
            cfg=newton.ModelBuilder.ShapeConfig(mu=0.35, density=1400.0, **CFG),
            label="card_shape",
        )

        # free charger plug
        plug_tm = load_tm("charging_block_outlet", "charging_block_3_1_body1")
        plug_mesh, plug_c = make_newton_mesh(plug_tm, sdf=True, sdf_resolution=320)
        self.plug_body = builder.add_body(
            xform=wp.transform(wp.vec3(-0.12, -0.22, plug_tm.extents[2] / 2 + 0.001),
                               wp.quat_identity()),
            label="plug",
        )
        builder.add_shape_mesh(self.plug_body, mesh=plug_mesh, cfg=pla, label="plug_shape")

        self._zones = [
            ("card_slot", np.asarray(housing_c), 0.09, FEATURE_SLOT),
            ("card", np.array([0.15, -0.20, card_ext[1] / 2 + 0.001]), 0.07,
             FEATURE_CARD),
            ("plug", np.array([-0.12, -0.22, plug_tm.extents[2] / 2 + 0.001]),
             0.07, FEATURE_PLUG),
        ]

        self.model = builder.finalize()
        self.solver = make_solver(self.model, iterations=10)
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()

        self.probe_l = ForceProbe(self.model, sensing_body="fr3/fr3_leftfinger")
        self.probe_r = ForceProbe(self.model, sensing_body="fr3/fr3_rightfinger")
        # squeeze force is measured against the GRASPABLE bodies only; the
        # whole-hand probes above are the PRESS channel (a fingertip leaning
        # on the swiper housing is not a grip)
        self.grip_l = ForceProbe(self.model, sensing_body="fr3/fr3_leftfinger",
                                 counterpart_bodies=["card", "plug"])
        self.grip_r = ForceProbe(self.model, sensing_body="fr3/fr3_rightfinger",
                                 counterpart_bodies=["card", "plug"])

        # update_contacts exports MuJoCo's entire allocated contact buffer,
        # even when fewer contacts are active. The automatic scene estimate
        # can be smaller than the solver's configured naconmax.
        self.pipeline = newton.CollisionPipeline(
            self.model, rigid_contact_max=int(self.solver.mjw_data.naconmax))
        self.contacts = self.pipeline.contacts()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)

        # teleop command: [x, y, z, yaw, jaw]
        tcp0 = self.ik.home_tcp
        self.cmd = np.array([tcp0[0], tcp0[1], tcp0[2], 0.0, 0.04])
        self.target_tf = wp.transform(wp.vec3(*tcp0[:3]), wp.quat(*tcp0[3:7]))

        # ── teleop guard ────────────────────────────────────────────────
        self.hand_i = list(self.model.body_label).index("fr3/fr3_link7")
        w0 = self.state_0.body_q.numpy()[self.hand_i]
        self.tcp_off = _qrot(np.array([-w0[3], -w0[4], -w0[5], w0[6]]),
                             np.asarray(tcp0[:3]) - np.asarray(w0[:3]))
        fmap = FeatureMap(free_air_m=0.05)
        for name, c, r, feat in self._zones:
            fmap.add_sphere(name, c, radius=r, feature_m=feat, pad=0.02)
        gcfg = GuardCfg(
            fps=self.fps, substeps=self.sim_substeps,
            qa_track_bodies=["card", "plug"],
            qa_envelope_lo=(-0.9, -0.9, -0.02), qa_envelope_hi=(0.9, 0.9, 1.0),
            qa_stride=3)
        self.guard = TeleopGuard(gcfg, self.model, feature_map=fmap)
        self.guard.enabled = not bool(getattr(args, "no_guard", False))
        self.guard.reset(self.cmd)
        self._guard_key_was_down = False
        print(f"[teleop_guard] {'ON' if self.guard.enabled else 'OFF'} "
              f"({len(fmap)} feature zones; press T to toggle)")

        self.viewer.set_model(self.model)
        self.viewer.set_camera(pos=wp.vec3(0.55, -0.65, 0.45), pitch=-28.0, yaw=130.0)
        self.graph = None
        self.collection = None
        if getattr(args, "collect_dir", None):
            from dataclasses import asdict
            from force_bench.data.collection import RigCollector
            self.collection = RigCollector(_CollectionRigView(self), args.collect_dir,
                                           source="teleoperation", seed=int(getattr(args, "seed", 0) or 0))
            self.collection.writer.metadata["teleoperation"] = {
                "guard_config": asdict(gcfg), "guard_enabled_initial": bool(self.guard.enabled),
                "guard_toggles": [], "viewer": str(getattr(args, "viewer", "unknown")),
                "external_viewer_forces_possible": True,
                "scene": "standalone card-swiper/charger scene, not a registered task"}

    # ── measured TCP (NOT the command echoed back) ──────────────────────
    def measured_tcp(self):
        w = self.state_0.body_q.numpy()[self.hand_i]
        return np.asarray(w[:3]) + _qrot(w[3:7], self.tcp_off)

    def _keyboard(self, dt):
        v = self.viewer
        if not hasattr(v, "is_key_down"):
            return
        mv = MOVE_SPEED * dt
        if v.is_key_down("w"): self.cmd[0] -= mv
        if v.is_key_down("s"): self.cmd[0] += mv
        if v.is_key_down("a"): self.cmd[1] -= mv
        if v.is_key_down("d"): self.cmd[1] += mv
        if v.is_key_down("r"): self.cmd[2] += mv
        if v.is_key_down("f"): self.cmd[2] -= mv
        if v.is_key_down("q"): self.cmd[3] += YAW_SPEED * dt
        if v.is_key_down("e"): self.cmd[3] -= YAW_SPEED * dt
        if v.is_key_down("g"): self.cmd[4] = max(0.0, self.cmd[4] - JAW_SPEED * dt)
        if v.is_key_down("h"): self.cmd[4] = min(0.04, self.cmd[4] + JAW_SPEED * dt)
        down = bool(v.is_key_down("t"))
        if down and not self._guard_key_was_down:
            self.guard.enabled = not self.guard.enabled
            if self.collection is not None:
                self.collection.writer.metadata["teleoperation"]["guard_toggles"].append(
                    {"timestamp": self.sim_time, "enabled": bool(self.guard.enabled)})
            print(f"[teleop_guard] {'ON' if self.guard.enabled else 'OFF'}")
        self._guard_key_was_down = down
        self.cmd[2] = max(0.03, self.cmd[2])

    def step(self):
        self._keyboard(self.frame_dt)
        if getattr(self.viewer, "gizmo_is_using", False):
            p = wp.transform_get_translation(self.target_tf)
            self.cmd[0], self.cmd[1], self.cmd[2] = p[0], p[1], max(0.03, p[2])
        else:
            self.target_tf[:] = wp.transform(
                wp.vec3(*self.cmd[:3]), wp.quat(*down_grasp_quat(self.cmd[3])))

        # ── the guard sits HERE: between the operator and the robot ────
        hl = self.probe_l.history
        hr = self.probe_r.history
        gl = self.grip_l.history
        gr = self.grip_r.history
        pads = (gl[-1]["force"] if gl else 0.0, gr[-1]["force"] if gr else 0.0)
        hand = max(hl[-1]["force"] if hl else 0.0,
                   hr[-1]["force"] if hr else 0.0)
        self.cmd = self.guard.filter(self.cmd, measured_tcp=self.measured_tcp(),
                                     pad_forces=pads, contact_force=hand)

        q_sol = self.ik.solve(self.cmd[:3], down_grasp_quat(self.cmd[3]), iterations=16)
        qt = self.control.joint_target_q.numpy()
        qt[:7] = q_sol[:7]
        qt[7:9] = self.cmd[4]
        self.control.joint_target_q.assign(qt)
        if self.collection is not None:
            self.collection.after_control()

        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            self.pipeline.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control,
                             self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0
        self.sim_time += self.frame_dt
        if self.collection is not None:
            self.collection.after_step(self.sim_time)

        self.solver.update_contacts(self.contacts, self.state_0)
        rl = self.probe_l.sample(self.contacts, self.state_0, self.sim_time)
        rr = self.probe_r.sample(self.contacts, self.state_0, self.sim_time)
        self.grip_l.sample(self.contacts, self.state_0, self.sim_time)
        self.grip_r.sample(self.contacts, self.state_0, self.sim_time)

        # live QA + HUD (the whole point: violations surface DURING the
        # session, not in a post-hoc report nobody reads)
        self.guard.observe(self.state_0, self.contacts, self.sim_time,
                           pad_forces=(self.grip_l.history[-1]["force"],
                                       self.grip_r.history[-1]["force"]))
        for line in self.guard.hud_lines(t=self.sim_time, every_s=0.5):
            print(line)
        if (rl["force"] + rr["force"]) > 0.1 and int(self.sim_time * self.fps) % 12 == 0:
            print(f"grip L={rl['force']:5.2f}N R={rr['force']:5.2f}N")

    def render(self):
        self.viewer.begin_frame(self.sim_time)
        if hasattr(self.viewer, "log_gizmo"):
            self.viewer.log_gizmo("tcp_target", self.target_tf)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()

    def test_final(self):
        bq = self.state_0.body_q.numpy()
        assert np.all(np.isfinite(bq)), "non-finite transform"
        for b, name in ((self.card_body, "card"), (self.plug_body, "plug")):
            assert -0.05 <= bq[b][2] < 0.3, f"{name} left the workspace"
        import json
        print("[teleop_guard] session summary:")
        s = self.guard.summary()
        s.pop("violations", None)
        print(json.dumps(s, indent=2, default=float))

    def finish_collection(self, error=None):
        """Publish completed transitions on exit; no implicit human-success label."""
        if self.collection is None or not self.collection.writer.actions:
            return None
        import json
        self.collection.writer.metadata["teleoperation"]["guard_summary"] = json.loads(
            json.dumps(self.guard.summary(), default=float))
        outcome = {"status": "unvalidated", "success": None, "source": "teleoperation_exit",
                   "reason": "operator/viewer ended session; task completion was not assessed"}
        if error is not None:
            outcome = {"status": "failed", "success": False, "source": "teleoperation_exception",
                       "reason": f"{type(error).__name__}: {error}"}
        path = self.collection.finish(outcome=outcome, discard_pending=error is not None)
        self.collection = None
        return path


def _qrot(q, v):
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    u, w = q[:3], q[3]
    return 2.0 * (u @ v) * u + (w * w - u @ u) * v + 2.0 * w * np.cross(u, v)


if __name__ == "__main__":
    parser = newton.examples.create_parser()
    parser.add_argument("--no-guard", action="store_true",
                        help="bypass the teleop guard's clamps "
                             "(live QA keeps running, so the A/B is measured)")
    parser.add_argument("--collect-dir", help="save an aligned joint-action demonstration on viewer exit")
    viewer, args = newton.examples.init(parser)
    example = Example(viewer, args)
    try:
        newton.examples.run(example, args)
    except BaseException as error:
        example.finish_collection(error)
        raise
    else:
        example.finish_collection()
