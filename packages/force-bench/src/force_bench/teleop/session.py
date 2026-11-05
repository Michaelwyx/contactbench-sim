"""The scene-agnostic teleoperation session behind both UIs.

    operator input (rates)  ->  raw command  [x y z yaw roll jaw]
                            ->  TeleopGuard  (admittance, contact-aware governor,
                                              press servo, force-limited fingers)
                            ->  IK + joint-rate limit  ->  the FR3's drives
                            ->  one physics frame (CUDA graph when capturable)
                            ->  contact readback: per-body / per-pair forces,
                                overlap depth vs the REST baseline, body speeds
                            ->  AlarmMonitor  ->  telemetry + force arrows

Everything the UIs show is read from ``telemetry()`` once per frame; the
GL window and the web page are two views of the same dict.

CONTACT FORCES ARE READ OFF THE CONTACT ARRAYS, NOT OFF SENSORS.  A
``SensorContact`` has to exist before the task calls
``CollisionPipeline.contacts()``, i.e. before the scene is handed over, so
a session that attaches afterwards cannot add probes; one pass over
``contacts.force`` (the force on shape 0, per Newton's sensor convention)
gives every body's load, every pair's load, the overlap depth and the
arrow geometry at once.  The task's own probes are still sampled so its
``extra`` readouts keep working.

PENETRATION IS MEASURED AS GROWTH OVER REST.  Several interfaces overlap
at rest by design (an inserted plug, a rack in its bore), so the session
holds the start pose for ``settle_frames`` first, records the rest depth
of every shape pair, and reports how much deeper any pair has gone since.
3 mm of growth is the gate's tunnelling threshold.
"""

from __future__ import annotations

import collections
import json
import time
from dataclasses import dataclass, field

import numpy as np
import warp as wp

import newton

from force_bench.teleop.alarms import AlarmCfg, AlarmMonitor
from force_bench.teleop.guard import GuardCfg, TeleopGuard
from force_bench.teleop.inputs import AXES, Edge, InputState

FR3_QD_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610])

# The pose command, by index.  PITCH joined it on 2026-09-12 for the
# SpaceMouse: a 6-DoF puck has three rotation axes, and with only yaw and
# roll two of them had nowhere to go -- worse, for a top-down grasp the
# tool's approach axis IS the vertical, so roll and yaw were the same axis
# with opposite signs and the device had ONE useful rotation.
IX, IY, IZ, IYAW, IPITCH, IROLL, IJAW = range(7)
NCMD = 7


# ── quaternion helpers (x, y, z, w) ─────────────────────────────────────
def qrot(q, v):
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    u, w = q[:3], q[3]
    return 2.0 * (u @ v) * u + (w * w - u @ u) * v + 2.0 * w * np.cross(u, v)


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qaxis(axis, ang):
    a = np.asarray(axis, dtype=float)
    a = a / max(np.linalg.norm(a), 1e-12)
    s = np.sin(ang / 2.0)
    return np.array([a[0] * s, a[1] * s, a[2] * s, np.cos(ang / 2.0)])


CONTACT_FORCE_FIELDS = ("pad_L", "pad_R", "hand", "object", "mech")


@dataclass
class SessionCfg:
    speed_m_s: float = 0.15          # full-stick translation rate
    yaw_rate: float = 1.2            # rad/s
    pitch_rate: float = 1.2          # rad/s about the tool's lateral axis
    roll_rate: float = 1.2           # rad/s about the tool axis
    jaw_rate: float = 0.05           # m/s of finger travel
    speed_scale: float = 1.0
    guard: bool = True
    graph: bool = True               # capture the physics frame as a CUDA graph
    settle_frames: int = 30
    readouts: bool = True            # sample the task's own extra() every frame
    history_s: float = 12.0
    z_min: float = 0.003
    alarms: AlarmCfg = field(default_factory=AlarmCfg)
    arrow_min_N: float = 0.05
    arrow_max: int = 48
    realtime: bool = True            # UIs sleep to the sim clock when faster
    translation_rates_m_s: tuple[float, float, float] | None = None
    """Optional forward/left/up gains for task-specific precision control."""


class TeleopSession:
    def __init__(self, scene, cfg: SessionCfg | None = None):
        self.scene = scene
        self.rig = rig = scene.rig
        self.cfg = cfg or SessionCfg()
        m = rig.model
        self.model = m
        self.labels = list(m.body_label)
        self.nb = int(m.body_count)
        self.fps = float(rig.fps)
        self.dt = 1.0 / self.fps
        self.sim_dt = self.dt / rig.substeps
        self.every = bool(rig.effective_collide_every_substep())
        self.shape_body = m.shape_body.numpy()
        try:
            self.shape_label = list(m.shape_label)
        except Exception:
            self.shape_label = [f"shape{i}" for i in range(int(m.shape_count))]

        self.wi = self.labels.index("fr3/fr3_link7")
        self.pad_idx = [i for i, l in enumerate(self.labels)
                        if l.endswith("leftfinger")] + \
                       [i for i, l in enumerate(self.labels) if l.endswith("rightfinger")]
        self.robot_idx = np.array([i for i, l in enumerate(self.labels) if l.startswith("fr3/")])
        self.is_robot = np.zeros(self.nb, dtype=bool)
        self.is_robot[self.robot_idx] = True
        self.is_pad = np.zeros(self.nb, dtype=bool)
        self.is_pad[self.pad_idx] = True
        self.robot_shapes = (self.shape_body >= 0) & self.is_robot[np.clip(self.shape_body, 0, None)]

        # operator frame: forward = from the robot base toward the start TCP
        base = np.array([rig.robot_base[0], rig.robot_base[1], rig.robot_base[2]], dtype=float)
        fwd = np.asarray(scene.tcp0[:2], float) - base[:2]
        if np.linalg.norm(fwd) < 1e-6:
            fwd = np.array([1.0, 0.0])
        fwd = fwd / np.linalg.norm(fwd)
        self.op_frame = np.array([[fwd[0], fwd[1], 0.0],
                                  [-fwd[1], fwd[0], 0.0],
                                  [0.0, 0.0, 1.0]])        # rows: fwd, left, up (world coords)

        # TCP offset in the wrist frame (the IK model's FK is the reference)
        w0 = rig.s0.body_q.numpy()[self.wi]
        self.tcp_off = qrot(qconj(w0[3:7]), np.asarray(scene.tcp0) - np.asarray(w0[:3]))
        self.q0 = np.asarray(scene.q0, dtype=float)
        self.pitch_axis, self.pitch_sign = self._pitch_axis()

        # command state
        self.cmd0 = np.array([*scene.tcp0, 0.0, 0.0, 0.0, scene.grip0], dtype=float)
        self.raw = self.cmd0.copy()
        self.cmd = self.cmd0.copy()
        self.speed_scale = float(self.cfg.speed_scale)
        self.hold = False
        self.paused = False
        self.viewer_paused = False      # the GL viewer's own pause (sidebar checkbox)
        self.diverged = False
        self._frozen = False            # hold / stale: the retreat point is latched ONCE
        self.goal_pending = False       # an absolute set_target goal is being walked to
        self.edge = Edge()
        self.input = InputState()

        gcfg = GuardCfg(fps=self.fps, substeps=rig.substeps,
                        qa_envelope_lo=tuple(scene.env_lo), qa_envelope_hi=tuple(scene.env_hi),
                        qa_stride=6)
        self.guard = TeleopGuard(gcfg, m, feature_map=scene.fmap)
        self.guard.enabled = bool(self.cfg.guard)
        self.guard.reset(self._guard_vec(self.cmd0))

        self.alarms = AlarmMonitor(self.cfg.alarms)
        self.rest_depth: dict[tuple, float] = {}
        self.rest_force: dict[tuple, float] = {}
        self.armed = False
        self.graph = None
        self.graph_error = None
        self.t = 0.0
        self.frame = 0
        self.wall_fps = 0.0
        self._wall_prev = None
        self._prev_p = None
        self.hist_len = int(self.cfg.history_s * self.fps)
        self.hist = {k: collections.deque(maxlen=self.hist_len)
                     for k in ("t", "pad_L", "pad_R", "hand", "object", "mech",
                               "penetration", "speed", "track")}
        self.last: dict = {}
        self.arrow_list: list = []
        self.readout_failures = 0
        self.readouts: dict = {}
        self._capture_rest_state()
        self.events_log: list = []
        self.ik_tol_m = 0.008
        self.ik_err_m = 0.0
        self.ik_detail = ""
        self._last_reachable = None
        self.collector = None           # force_bench.data.collection.RigCollector while recording
        self._collect_args = None
        self.collected: list = []

    def _pitch_axis(self):
        """The axis positive pitch turns about, and its sign.

        Neither can be a constant.  Pitch has to be applied in the TOOL
        frame so that it still means "tilt" after the command has yawed,
        but the tool's own x axis is whatever the scene's grasp
        orientation made it: on a scene whose grasp yaw happens to put
        tool-x along the operator's line of sight, pitching about it rolls
        the gripper sideways instead of tilting it, which is not what any
        operator means by pitch.

        So the axis is the OPERATOR's lateral direction carried into the
        tool frame -- that keeps the tilt in the fore/aft vertical plane
        for every scene -- and the sign is chosen so that positive pitch
        RAISES the approach direction's elevation.  That one rule reads
        correctly both ways round: on a top-down pick it tilts the nose up
        off the vertical, toward the operator, and on a horizontal door
        reach it lifts the nose above the horizon.
        """
        left = np.asarray(self.op_frame[1], dtype=float)
        axis = qrot(qconj(self.q0), left)          # the operator's lateral axis, in the tool
        n = float(np.linalg.norm(axis))
        axis = axis / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
        z = np.array([0.0, 0.0, 1.0])
        a0 = qrot(self.q0, z)
        a1 = qrot(qmul(self.q0, qaxis(axis, 1e-3)), z)
        rise = float(a1[2] - a0[2])
        if abs(rise) < 1e-12:                      # the tool points along the pitch axis
            return axis, 1.0                       #   (never happens in the suite)
        return axis, (1.0 if rise > 0.0 else -1.0)

    # ── state capture / reset ───────────────────────────────────────────
    def _capture_rest_state(self):
        rig = self.rig
        self._q0 = rig.s0.body_q.numpy().copy()
        self._qd0 = rig.s0.body_qd.numpy().copy()
        self._jt0 = rig.control.joint_target_q.numpy().copy()
        self._jq0 = self._jqd0 = None
        if rig.s0.joint_q is not None and rig.model.joint_count:
            jq = wp.zeros_like(rig.s0.joint_q)
            jqd = wp.zeros_like(rig.s0.joint_qd)
            newton.eval_ik(rig.model, rig.s0, jq, jqd)
            self._jq0, self._jqd0 = jq.numpy().copy(), jqd.numpy().copy()
        self._ik_q0 = rig.ik.joint_q.numpy().copy()

    def reset(self):
        rig = self.rig
        recording = self.collector is not None
        if recording:
            self.finish_collection(reason="operator reset the scene")
        for s in (rig.s0, rig.s1):
            s.body_q.assign(self._q0)
            s.body_qd.assign(self._qd0)
            if self._jq0 is not None:
                s.joint_q.assign(self._jq0)
                s.joint_qd.assign(self._jqd0)
        rig.control.joint_target_q.assign(self._jt0)
        rig.ik.joint_q.assign(self._ik_q0)
        # MuJoCo warm-starts every solve from its previous qacc and keeps
        # applied forces / actuator state in mjw_data; after a divergence the
        # restored joint coordinates alone re-diverge on the first substep.
        # flags=0 leaves the joint state written above alone and only clears
        # those buffers.  A task may wrap the solver (a thread model, a reaction
        # torque), so walk down to the SolverMuJoCo underneath.
        solver = rig.solver
        while hasattr(solver, "_solver"):
            solver = solver._solver
        if hasattr(solver, "reset"):
            try:
                solver.reset(rig.s0, flags=0)
            except Exception as exc:
                self._note(f"solver reset skipped: {type(exc).__name__}: {exc}")
        self.last = {}
        self.arrow_list = []
        self.readouts = {}
        self.raw = self.cmd0.copy()
        self.cmd = self.cmd0.copy()
        self.guard.reset(self._guard_vec(self.cmd0))
        self.alarms.reset()
        for h in self.hist.values():
            h.clear()
        self.hold = False
        self.paused = False
        self.diverged = False
        self._frozen = False
        self.goal_pending = False
        self.t = 0.0
        self.frame = 0
        self._prev_p = None
        self.readout_failures = 0
        self._last_reachable = None
        self.events_log.append({"t": 0.0, "channel": "session", "level": "info",
                                "value": 0, "unit": "", "detail": "reset to the start pose"})
        if recording:
            self.start_collection(*self._collect_args)

    # ── demonstration recording ─────────────────────────────────────────
    def start_collection(self, output, seed: int = 0, info: dict | None = None):
        """Record every stepped frame as an aligned episode (force_bench.data episode format):
        FR3 joint-target actions, measured joint state + object poses, and the tactile fields when
        the gel pads are mounted.  A reset closes the episode and opens the next one."""
        from dataclasses import asdict
        from force_bench.data.collection import RigCollector
        self._collect_args = (output, seed, info)
        self.collector = RigCollector(self.rig, output, task=f"teleop/{self.scene.name}",
                                      seed=int(seed), source="teleoperation")
        self.collector.writer.metadata["teleoperation"] = json.loads(json.dumps({
            "scene": self.scene.name, "guard_config": asdict(self.guard.cfg),
            "guard_enabled_initial": bool(self.guard.enabled),
            "contact_forces": {"fields": list(CONTACT_FORCE_FIELDS), "units": "N",
                               "meaning": "total contact force magnitude after each step: left / right finger "
                                          "pad, the rest of the robot, the manipulated object, the scene's "
                                          "mechanism pair (as shown live to the operator)"},
            **(info or {})}, default=float))

    def finish_collection(self, reason: str = "operator ended the session"):
        """Publish the running episode (if it has any transitions); task success is not assessed."""
        if self.collector is None:
            return None
        collector, self.collector = self.collector, None
        if not collector.writer.actions:
            return None
        collector.writer.metadata["teleoperation"]["events"] = json.loads(
            json.dumps(self.events_log, default=float))
        outcome = {"status": "unvalidated", "success": None, "source": "teleoperation", "reason": reason}
        path = collector.finish(outcome=outcome, discard_pending=True)
        self.collected.append(path)
        return path

    # ── settle + graph ──────────────────────────────────────────────────
    def settle(self, frames: int | None = None):
        n = self.cfg.settle_frames if frames is None else int(frames)
        for _ in range(n):
            self._drive(self.cmd[:3], self._quat(self.cmd), self.cmd[IJAW])
            self._physics()
            self._measure(baseline=True)
        self.armed = True
        self._prev_p = None
        if getattr(self.scene, "hold_start_pinch", False):
            self._hold_start_pinch()
        if self.cfg.graph:
            self._try_capture()

    def _hold_start_pinch(self):
        """A task that starts with the object in the fingers (a USB stick, a screwdriver, a bulb) pinches it at its
        own working force, which can be above the guard's human-grip ceiling: the limiter would open the fingers and
        drop it, and the press servo, which reads the pads too, would take the squeeze for a press and back the arm
        off.  Both ceilings and the pad alarms are raised to that pinch, as measured at the end of the settle."""
        pinch = max(self.last.get("pad_L", 0.0), self.last.get("pad_R", 0.0))
        gc = self.guard.cfg
        if 1.25 * pinch <= gc.grip_force_max_N:
            return
        gc.grip_force_max_N = 1.25 * pinch
        gc.press_force_max_N = max(gc.press_force_max_N, 1.25 * pinch)
        self.alarms.cfg.allow_pinch(pinch)
        self._note(f"start pinch {pinch:.1f} N/pad: grip and press ceilings {gc.grip_force_max_N:.1f} N")

    def _try_capture(self):
        rig = self.rig
        if rig.substeps % 2:
            self.graph_error = "odd substep count"
            return
        try:
            with wp.ScopedCapture() as cap:
                self._frame_eager()
            self.graph = cap.graph
        except Exception as exc:
            self.graph = None
            self.graph_error = f"{type(exc).__name__}: {str(exc)[:120]}"
            # the failed capture may have left a half-stepped state; the
            # start pose is restored so the operator starts where the task does
            for s in (rig.s0, rig.s1):
                s.body_q.assign(self._q0)
                s.body_qd.assign(self._qd0)
                if self._jq0 is not None:
                    s.joint_q.assign(self._jq0)
                    s.joint_qd.assign(self._jqd0)

    def _frame_eager(self):
        rig = self.rig
        if not self.every:
            rig.s0.clear_forces()
            rig.pipeline.collide(rig.s0, rig.contacts)
        for _ in range(rig.substeps):
            if self.every:
                rig.s0.clear_forces()
                rig.pipeline.collide(rig.s0, rig.contacts)
            rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, self.sim_dt)
            rig.s0, rig.s1 = rig.s1, rig.s0

    def _physics(self):
        if self.graph is not None:
            wp.capture_launch(self.graph)
        else:
            self._frame_eager()
        self.rig.solver.update_contacts(self.rig.contacts, self.rig.s0)

    # ── commands ────────────────────────────────────────────────────────
    @staticmethod
    def _guard_vec(c):
        """The guard's own 5-vector: [x, y, z, yaw, finger width]."""
        return np.array([c[IX], c[IY], c[IZ], c[IYAW], c[IJAW]], dtype=float)

    def _quat(self, c):
        """world yaw  x  the scene's grasp orientation  x  tool pitch  x  tool roll.

        Yaw is applied in the WORLD (about +z) because that is what an
        operator means by "turn the gripper"; pitch and roll are applied in
        the TOOL frame, which is how a wrist works and what keeps them
        meaningful after a yaw."""
        q = qmul(qaxis((0, 0, 1), c[IYAW]), self.q0)
        q = qmul(q, qaxis(self.pitch_axis, c[IPITCH] * self.pitch_sign))
        return qmul(q, qaxis((0, 0, 1), c[IROLL]))

    def measured_tcp(self) -> np.ndarray:
        w = self.rig.s0.body_q.numpy()[self.wi]
        return np.asarray(w[:3], float) + qrot(w[3:7], self.tcp_off)

    def apply_input(self, inp: InputState):
        """Integrate one frame of operator rates into the RAW command."""
        self.input = inp
        for b in self.edge.rising(inp.buttons):
            if b == "guard":
                self.guard.enabled = not self.guard.enabled
                self._note(f"guard {'ON' if self.guard.enabled else 'OFF'}")
            elif b == "reset":
                self.reset()
            elif b == "pause":
                self.paused = not self.paused
                self._note("paused" if self.paused else "running")
            elif b == "speed_up":
                self.speed_scale = float(min(3.0, self.speed_scale * 1.5))
            elif b == "speed_down":
                self.speed_scale = float(max(0.1, self.speed_scale / 1.5))
        self.hold = bool(inp.buttons.get("hold"))
        if self.hold or inp.stale:
            self.freeze()
            return
        self._frozen = False            # input is live again: the next hold re-latches
        if self.paused or self.viewer_paused:
            return                      # time is stopped: rates do not accumulate
        if np.abs(inp.axes).max() > 0.0:
            self.goal_pending = False   # the sticks took over from any absolute goal
        k = self.speed_scale * (0.25 if inp.buttons.get("fine") else 1.0)
        c = self.cfg
        rates = c.speed_m_s if c.translation_rates_m_s is None else np.asarray(c.translation_rates_m_s)
        v = self.op_frame.T @ (inp.axes[:3] * rates * k)   # fwd/left/up -> world
        self.raw[:3] += v * self.dt
        self.raw[IYAW] += inp.axis("yaw") * c.yaw_rate * k * self.dt
        self.raw[IPITCH] += inp.axis("pitch") * c.pitch_rate * k * self.dt
        self.raw[IROLL] += inp.axis("roll") * c.roll_rate * k * self.dt
        self.raw[IJAW] += inp.axis("jaw") * c.jaw_rate * k * self.dt
        self._clamp_raw()

    def freeze(self):
        """E-stop / lost link: every axis stops where the GUARDED command is.
        On the FIRST frozen frame the translation retreats to the measured
        TCP, so any press lead the governor was holding is released; it is
        done once, never per frame -- the measured TCP sits a gravity droop
        below the command, and re-snapping onto it every frame walks the arm
        down at the droop rate (docs/teleop_guard.md, bug 1).  The jaw is set
        to the value that keeps the EMA + relief loop stationary (the guard's
        state follows the post-relief width, so ``raw = cmd`` would re-add
        the relief every frame and open a held grasp)."""
        self.raw[:IJAW] = self.cmd[:IJAW]        # x y z yaw pitch roll
        if not self._frozen and self.last.get("finite", True):
            self.raw[:3] = self.measured_tcp()
        self._frozen = True
        self.goal_pending = False
        gc = self.guard.cfg
        lo, hi = gc.finger_range
        relief = float(getattr(self.guard.grip, "relief", 0.0) or 0.0)
        self.raw[IJAW] = float(np.clip(self.cmd[IJAW] - relief / max(gc.ema_alpha, 1e-6), lo, hi))
        self._clamp_raw()

    def set_target(self, pos=None, yaw=None, roll=None, jaw=None, pitch=None):
        """Direct target (gizmo drag / web absolute pose).  Non-finite input
        is refused: a NaN would pass every clamp and poison the IK seed."""
        vals = {"pos": pos, "yaw": yaw, "pitch": pitch, "roll": roll, "jaw": jaw}
        for k, v in vals.items():
            if v is not None and not np.isfinite(np.asarray(v, dtype=float)).all():
                self._note(f"set_target refused non-finite {k}")
                return
        if pos is not None:
            self.raw[:3] = np.asarray(pos, float)
        if yaw is not None:
            self.raw[IYAW] = float(yaw)
        if pitch is not None:
            self.raw[IPITCH] = float(pitch)
        if roll is not None:
            self.raw[IROLL] = float(roll)
        if jaw is not None:
            self.raw[IJAW] = float(jaw)
        self._clamp_raw()
        self.goal_pending = True        # walked to by the guard; exempt from the run-ahead bound

    def _clamp_raw(self):
        lo, hi = self.scene.env_lo, self.scene.env_hi
        self.raw[:3] = np.clip(self.raw[:3], lo, hi)
        self.raw[2] = max(self.raw[2], self.cfg.z_min)
        for i in (IYAW, IPITCH, IROLL):
            self.raw[i] = float(np.clip(self.raw[i], -np.pi, np.pi))
        self.raw[IJAW] = float(np.clip(self.raw[IJAW], 0.0, 0.04))

    def _note(self, text):
        self.events_log.append({"t": round(self.t, 3), "channel": "session", "level": "info",
                                "value": 0, "unit": "", "detail": text})

    # ── one frame ───────────────────────────────────────────────────────
    def step(self):
        if self.paused:
            self._wall_tick()
            return
        tcp_m = self.measured_tcp()
        fL, fR = self.last.get("pad_L", 0.0), self.last.get("pad_R", 0.0)
        hand = self.last.get("hand", 0.0)
        g = self.guard.filter(self._guard_vec(self.raw), measured_tcp=tcp_m,
                              pad_forces=(fL, fR), contact_force=max(fL, fR, hand))
        # pitch and roll are not guard axes; give them the governor's own
        # local yaw-rate clamp, so a tilt near a 0.1 mm feature is as slow
        # as a yaw there
        pitch, roll = self.raw[IPITCH], self.raw[IROLL]
        if self.guard.enabled:
            step = self.guard.gov.limits(g[:3]).get("yaw_step_rad", 0.05)
            pitch = float(np.clip(pitch, self.cmd[IPITCH] - step, self.cmd[IPITCH] + step))
            roll = float(np.clip(roll, self.cmd[IROLL] - step, self.cmd[IROLL] + step))
        self.cmd = np.array([g[0], g[1], g[2], g[3], pitch, roll, g[4]], dtype=float)
        if not self.guard.enabled:
            self.raw[:] = self.cmd
        else:
            # the operator integrates at full stick rate while the governor
            # lets the command advance at the local feature rate; bound how
            # far raw may run ahead so releasing the stick stops the arm
            # within a few frames instead of after the backlog plays out.
            # An ABSOLUTE goal (gizmo drag, web target) is exempt until the
            # command reaches it: the guard walks the arm there at its rate.
            gc = self.guard.cfg
            lim = self.guard.gov.limits(self.cmd[:3])
            L = max(4.0 * lim["step_m"], 2.0 * gc.deadband_m)
            if self.goal_pending and np.abs(self.raw[:3] - self.cmd[:3]).max() <= L:
                self.goal_pending = False
            if not self.goal_pending:
                self.raw[:3] = self.cmd[:3] + np.clip(self.raw[:3] - self.cmd[:3], -L, L)
            Ly = max(4.0 * lim["yaw_step_rad"], 2.0 * gc.deadband_rad)
            for i in (IYAW, IPITCH, IROLL):
                self.raw[i] = self.cmd[i] + float(np.clip(self.raw[i] - self.cmd[i], -Ly, Ly))
            Lj = max(4.0 * gc.grip_close_rate / self.fps, 2.0 * gc.finger_deadband_m)
            self.raw[IJAW] = self.cmd[IJAW] + float(
                np.clip(self.raw[IJAW] - self.cmd[IJAW], -Lj, Lj))
        ik_ok = self._drive(self.cmd[:3], self._quat(self.cmd), self.cmd[IJAW])
        if self.collector is not None:
            self.collector.after_control()
        self._physics()
        self.t += self.dt
        self.frame += 1
        if self.cfg.readouts and self.scene.task_extra is not None and self.readout_failures < 3:
            self._sample_readouts()
        self._measure(baseline=False, ik_ok=ik_ok, tcp_m=tcp_m)
        if self.collector is not None:
            self.collector.after_step(self.t, extra={"contact_forces": [
                float(self.last.get(k) or 0.0) for k in CONTACT_FORCE_FIELDS]})
        self._wall_tick()

    def _drive(self, pos, quat, fingers) -> bool:
        """IK + the arm's own joint-rate limit + a REACHABILITY check.

        A damped-least-squares IK seeded from its last answer can only
        report a pose it cannot reach (a joint limit, the workspace edge)
        as a residual; the drives then track a wrong pose and the guard
        sees a permanent 20 mm lead.  If the IK model's own FK misses the
        target by more than ``ik_tol_m`` the target is refused: the raw
        command snaps back to the last reachable one and the ``ik`` flag
        tells the operator why the arm stopped."""
        rig = self.rig
        q_sol = rig.ik.solve(pos, quat, iterations=16)
        qt = rig.control.joint_target_q.numpy()
        ok = bool(np.isfinite(q_sol[:7]).all())
        if ok:
            fk = rig.ik.tcp_pose()
            self.ik_err_m = float(np.linalg.norm(np.asarray(fk[:3]) - np.asarray(pos)))
            if self.ik_err_m > self.ik_tol_m:
                ok = False
                self.ik_detail = f"target unreachable ({self.ik_err_m * 1000:.0f} mm short)"
                if self._last_reachable is not None:
                    self.raw[:] = self._last_reachable
                    self.cmd[:] = self._last_reachable
                    self.guard.reset(self._guard_vec(self.cmd))
                    q_sol = rig.ik.solve(self.cmd[:3], self._quat(self.cmd), iterations=16)
            else:
                self.ik_detail = ""
                self._last_reachable = self.cmd.copy()
            step = FR3_QD_MAX / self.fps
            if np.isfinite(q_sol[:7]).all():
                qt[:7] = np.clip(q_sol[:7], qt[:7] - step, qt[:7] + step)
        else:
            self.ik_detail = "non-finite IK solution"
        qt[7:9] = fingers
        rig.control.joint_target_q.assign(qt)
        return ok

    def _sample_readouts(self):
        rig = self.rig
        try:
            for pr in rig.probes.values():
                pr.sample(rig.contacts, rig.s0, self.t)
            out = self.scene.task_extra(rig.s0, self.t) or {}
            clean = {}
            for k, v in out.items():
                try:
                    fv = float(v)
                except (TypeError, ValueError):
                    clean[k] = str(v)[:24]
                    continue
                clean[k] = round(fv, 4) if np.isfinite(fv) else None
            self.readouts = clean
        except Exception as exc:
            self.readout_failures += 1
            if self.readout_failures >= 3:
                self.readouts = {"_error": f"{type(exc).__name__}: {str(exc)[:60]}"}

    def _wall_tick(self):
        now = time.perf_counter()
        if self._wall_prev is not None:
            dt = max(now - self._wall_prev, 1e-6)
            self.wall_fps = 0.9 * self.wall_fps + 0.1 / dt if self.wall_fps else 1.0 / dt
        self._wall_prev = now

    # ── contact readback ────────────────────────────────────────────────
    def _contacts(self, bq):
        c = self.rig.contacts
        n = int(c.rigid_contact_count.numpy()[0])
        if n <= 0:
            return None
        n = min(n, len(c.rigid_contact_shape0))
        s0 = c.rigid_contact_shape0.numpy()[:n]
        s1 = c.rigid_contact_shape1.numpy()[:n]
        keep = (s0 >= 0) & (s1 >= 0) & (s0 != s1)
        b0 = self.shape_body[np.clip(s0, 0, None)]
        b1 = self.shape_body[np.clip(s1, 0, None)]
        keep &= b0 != b1
        if not keep.any():
            return None
        s0, s1, b0, b1 = s0[keep], s1[keep], b0[keep], b1[keep]
        p0 = c.rigid_contact_point0.numpy()[:n][keep].astype(float)
        p1 = c.rigid_contact_point1.numpy()[:n][keep].astype(float)
        nm = c.rigid_contact_normal.numpy()[:n][keep].astype(float)
        f = c.force.numpy()[:n][keep].astype(float)
        f = f[:, :3] if f.ndim == 2 and f.shape[1] >= 3 else f
        w0, w1 = p0.copy(), p1.copy()
        k = b0 >= 0
        if k.any():
            w0[k] = bq[b0[k], :3] + _qrot_rows(bq[b0[k], 3:7], p0[k])
        k = b1 >= 0
        if k.any():
            w1[k] = bq[b1[k], :3] + _qrot_rows(bq[b1[k], 3:7], p1[k])
        nn = -nm
        d = np.sum(nn * (w0 - w1), axis=1)
        depth = np.maximum(0.0, -d)
        return dict(s0=s0, s1=s1, b0=b0, b1=b1, w0=w0, w1=w1, f=f,
                    fmag=np.linalg.norm(f, axis=1), depth=depth, n=nn)

    def _pair_name(self, a, b):
        def one(i):
            i = int(i)
            body = int(self.shape_body[i])
            owner = self.labels[body] if body >= 0 else "static"
            lbl = self.shape_label[i] if i < len(self.shape_label) else f"shape{i}"
            return owner if (lbl is None or lbl.startswith("shape")) else f"{owner}:{lbl}"
        return f"{one(a)}<->{one(b)}"

    def _measure(self, baseline: bool, ik_ok: bool = True, tcp_m=None):
        rig = self.rig
        bq = rig.s0.body_q.numpy()
        finite = bool(np.isfinite(bq).all())
        if not finite:
            self.diverged = True
            self.paused = True
        cd = self._contacts(bq) if finite else None
        total = np.zeros(self.nb)
        from_robot = np.zeros(self.nb)
        pad_by = {i: {} for i in self.pad_idx}
        pairs_force: dict[tuple, float] = {}
        pairs_depth: dict[tuple, float] = {}
        arrows = []
        if cd is not None:
            b0, b1, fmag, depth = cd["b0"], cd["b1"], cd["fmag"], cd["depth"]
            for arr in (b0, b1):
                k = arr >= 0
                if k.any():
                    np.add.at(total, arr[k], fmag[k])
            r0 = (b0 >= 0) & self.is_robot[np.clip(b0, 0, None)]
            r1 = (b1 >= 0) & self.is_robot[np.clip(b1, 0, None)]
            one_robot = r0 ^ r1
            obj = np.where(r0, b1, b0)
            k = one_robot & (obj >= 0)
            if k.any():
                np.add.at(from_robot, obj[k], fmag[k])
            for i in np.where(one_robot)[0]:
                rb = b0[i] if r0[i] else b1[i]
                if rb in pad_by:
                    ob = obj[i]
                    lbl = self.labels[ob] if ob >= 0 else "static"
                    pad_by[rb][lbl] = pad_by[rb].get(lbl, 0.0) + float(fmag[i])
            # per-pair aggregation (shape pairs: depth baseline; body pairs: force)
            skey = np.stack([np.minimum(cd["s0"], cd["s1"]), np.maximum(cd["s0"], cd["s1"])], axis=1)
            for j, (a, b) in enumerate(map(tuple, skey)):
                if depth[j] > pairs_depth.get((a, b), 0.0):
                    pairs_depth[(a, b)] = float(depth[j])
            bkey = np.stack([np.minimum(b0, b1), np.maximum(b0, b1)], axis=1)
            for j, (a, b) in enumerate(map(tuple, bkey)):
                pairs_force[(a, b)] = pairs_force.get((a, b), 0.0) + float(fmag[j])
            if not baseline:
                arrows = self._arrows(cd, skey)
        if baseline:
            for kk, v in pairs_depth.items():
                if v > self.rest_depth.get(kk, 0.0):
                    self.rest_depth[kk] = v
            for kk, v in pairs_force.items():
                if v > self.rest_force.get(kk, 0.0):
                    self.rest_force[kk] = v
            self.last = {"pad_L": float(total[self.pad_idx[0]]), "pad_R": float(total[self.pad_idx[1]]),
                         "hand": float(total[self.robot_idx].sum() - total[self.pad_idx].sum())}
            return

        pad_L = float(total[self.pad_idx[0]]) if len(self.pad_idx) > 0 else 0.0
        pad_R = float(total[self.pad_idx[1]]) if len(self.pad_idx) > 1 else 0.0
        hand = float(total[self.robot_idx].sum() - pad_L - pad_R)
        obj_i = int(np.argmax(from_robot)) if self.nb else 0
        obj_N = float(from_robot[obj_i])
        obj_lbl = self.labels[obj_i] if obj_N > 0 else ""
        # mechanism load: object-object pairs, growth over rest
        mech_N, mech_lbl = 0.0, ""
        for (a, b), v in pairs_force.items():
            if (a >= 0 and self.is_robot[a]) or (b >= 0 and self.is_robot[b]):
                continue
            if a < 0 and b < 0:
                continue
            ex = v - self.rest_force.get((a, b), 0.0)
            if ex > mech_N:
                mech_N = ex
                mech_lbl = f"{self.labels[a] if a >= 0 else 'static'}<->{self.labels[b] if b >= 0 else 'static'}"
        pen_ex, pen_pair, pen_abs, pen_abs_pair = 0.0, "", 0.0, ""
        for kk, v in pairs_depth.items():
            ex = v - self.rest_depth.get(kk, 0.0)
            if ex > pen_ex:
                pen_ex, pen_pair = ex, self._pair_name(*kk)
            if v > pen_abs:
                pen_abs, pen_abs_pair = v, self._pair_name(*kk)
        # speed (frame-to-frame displacement)
        speed, speed_body = 0.0, ""
        if finite and self._prev_p is not None:
            sp = np.linalg.norm(bq[:, :3] - self._prev_p, axis=1) / self.dt
            j = int(np.argmax(sp))
            speed, speed_body = float(sp[j]), self.labels[j]
        if finite:
            self._prev_p = bq[:, :3].copy()
        # envelope
        env_hit, env_body = False, ""
        if finite:
            p = bq[:, :3]
            out = (p < self.scene.env_lo).any(axis=1) | (p > self.scene.env_hi).any(axis=1)
            if out.any():
                env_hit, env_body = True, self.labels[int(np.argmax(out))]
        if tcp_m is None:
            tcp_m = self.measured_tcp() if finite else np.zeros(3)
        track = float(np.linalg.norm(self.cmd[:3] - tcp_m)) * 1000.0

        new = self.alarms.update(self.t, {
            "pad_L": (pad_L, ", ".join(f"{k} {v:.1f}N" for k, v in pad_by[self.pad_idx[0]].items()) if self.pad_idx else ""),
            "pad_R": (pad_R, ", ".join(f"{k} {v:.1f}N" for k, v in pad_by[self.pad_idx[1]].items()) if len(self.pad_idx) > 1 else ""),
            "hand": (hand, ""),
            "object": (obj_N, obj_lbl),
            "mech": (mech_N, mech_lbl),
            "penetration": (pen_ex * 1000.0, pen_pair),
            "speed": (speed, speed_body),
            "track": (track, ""),
        }, flags={"divergence": (not finite, "reset the scene"),
                  "envelope": (env_hit, env_body),
                  "ik": (not ik_ok, self.ik_detail)})
        self.events_log.extend(new)
        self.last = {"pad_L": pad_L, "pad_R": pad_R, "hand": hand, "object": obj_N,
                     "object_body": obj_lbl, "mech": mech_N, "mech_pair": mech_lbl,
                     "pen_excess_mm": pen_ex * 1000.0, "pen_pair": pen_pair,
                     "pen_abs_mm": pen_abs * 1000.0, "pen_abs_pair": pen_abs_pair,
                     "speed": speed, "speed_body": speed_body, "track_mm": track,
                     "finite": finite, "tcp": tcp_m, "pad_by": pad_by,
                     "per_body": {self.labels[i]: float(total[i]) for i in range(self.nb) if total[i] > 0.02},
                     "from_robot": {self.labels[i]: float(from_robot[i]) for i in range(self.nb) if from_robot[i] > 0.02},
                     "pairs": sorted(((self.labels[a] if a >= 0 else "static",
                                       self.labels[b] if b >= 0 else "static", v)
                                      for (a, b), v in pairs_force.items() if v > 0.05),
                                     key=lambda r: -r[2])[:10],
                     "n_contacts": int(len(cd["fmag"])) if cd is not None else 0}
        self.arrow_list = arrows
        h = self.hist
        h["t"].append(self.t)
        h["pad_L"].append(pad_L); h["pad_R"].append(pad_R); h["hand"].append(hand)
        h["object"].append(obj_N); h["mech"].append(mech_N)
        h["penetration"].append(pen_ex * 1000.0); h["speed"].append(speed); h["track"].append(track)

    def _arrows(self, cd, skey):
        """One arrow per contacting shape pair, pointing the way the force is
        applied to the NON-robot side (the force on shape 0 is what Newton
        stores; the other side gets its negative)."""
        thr = self.alarms.cfg.thresholds
        groups: dict[tuple, list] = {}
        for j, key in enumerate(map(tuple, skey)):
            groups.setdefault(key, []).append(j)
        out = []
        for (a, b), idx in groups.items():
            idx = np.asarray(idx)
            f_on_a = np.where((cd["s0"][idx] == a)[:, None], cd["f"][idx], -cd["f"][idx])
            F = f_on_a.sum(axis=0)
            mag = float(np.linalg.norm(F))
            if mag < self.cfg.arrow_min_N:
                continue
            w = cd["fmag"][idx]
            wsum = float(w.sum())
            pt = (cd["w0"][idx] * w[:, None]).sum(axis=0) / wsum if wsum > 0 else cd["w0"][idx].mean(axis=0)
            ra, rb = bool(self.robot_shapes[a]), bool(self.robot_shapes[b])
            if ra and not rb:
                direction = -F            # force ON b (the object) = -force on a
                chan = "pad_L" if self.shape_body[a] == self.pad_idx[0] else (
                    "pad_R" if self.shape_body[a] == self.pad_idx[1] else "hand")
            elif rb and not ra:
                direction = F             # force on a = the object
                chan = "pad_L" if self.shape_body[b] == self.pad_idx[0] else (
                    "pad_R" if self.shape_body[b] == self.pad_idx[1] else "hand")
            else:
                direction = F
                chan = "mech"
            t = thr[chan]
            level = "critical" if mag >= t.critical else ("warn" if mag >= t.warn else "ok")
            length = min(0.15, 0.015 + 0.03 * np.log1p(mag))
            end = pt + direction / mag * length
            out.append({"s": [round(float(v), 4) for v in pt], "e": [round(float(v), 4) for v in end],
                        "N": round(mag, 3), "level": level, "pair": self._pair_name(a, b),
                        "robot": ra or rb})
        out.sort(key=lambda r: -r["N"])
        return out[: self.cfg.arrow_max]

    # ── views ───────────────────────────────────────────────────────────
    def telemetry(self) -> dict:
        gi = self.guard.last_info or {}
        L = self.last
        tcp = L.get("tcp", self.measured_tcp())
        return {
            "t": round(self.t, 3), "frame": self.frame,
            "scene": self.scene.name, "fps_wall": round(self.wall_fps, 1),
            "realtime": round(self.wall_fps / self.fps, 2) if self.fps else 0.0,
            "graph": self.graph is not None, "paused": self.paused or self.viewer_paused,
            "viewer_paused": self.viewer_paused, "hold": self.hold,
            "diverged": self.diverged, "finite": bool(L.get("finite", True)),
            "guard": {"enabled": self.guard.enabled, "zone": gi.get("zone", ""),
                      "feature_mm": round(float(gi.get("feature_m", 0.0)) * 1000, 2),
                      "vmax_mm_s": round(float(gi.get("speed_m_s", 0.0)) * 1000, 1),
                      "lead_mm": round(float(gi.get("lead_budget_m", gi.get("lead_m", 0.0))) * 1000, 2),
                      "clips": {k: bool(gi.get(k)) for k in
                                ("rate_clipped", "lead_clipped", "yaw_clipped", "press_servo",
                                 "over_limit")},
                      "counts": dict(self.guard.counts)},
            "cmd": {"x": round(float(self.cmd[IX]), 4), "y": round(float(self.cmd[IY]), 4),
                    "z": round(float(self.cmd[IZ]), 4),
                    "yaw_deg": round(float(np.degrees(self.cmd[IYAW])), 1),
                    "pitch_deg": round(float(np.degrees(self.cmd[IPITCH])), 1),
                    "roll_deg": round(float(np.degrees(self.cmd[IROLL])), 1),
                    "jaw_mm": round(float(self.cmd[IJAW]) * 1000, 2),
                    "raw_jaw_mm": round(float(self.raw[IJAW]) * 1000, 2),
                    "speed_scale": round(self.speed_scale, 2)},
            "tcp": {"x": round(float(tcp[0]), 4), "y": round(float(tcp[1]), 4), "z": round(float(tcp[2]), 4)},
            "forces": {"pad_L": round(L.get("pad_L", 0.0), 3), "pad_R": round(L.get("pad_R", 0.0), 3),
                       "hand": round(L.get("hand", 0.0), 3), "object": round(L.get("object", 0.0), 3),
                       "object_body": L.get("object_body", ""), "mech": round(L.get("mech", 0.0), 3),
                       "mech_pair": L.get("mech_pair", ""),
                       "pad_by": {("L" if i == self.pad_idx[0] else "R"): {k: round(v, 3) for k, v in d.items()}
                                  for i, d in L.get("pad_by", {}).items()},
                       "per_body": {k: round(v, 3) for k, v in L.get("per_body", {}).items()},
                       "pairs": [[a, b, round(v, 3)] for a, b, v in L.get("pairs", [])],
                       "n_contacts": L.get("n_contacts", 0)},
            "penetration": {"excess_mm": round(L.get("pen_excess_mm", 0.0), 3), "pair": L.get("pen_pair", ""),
                            "abs_mm": round(L.get("pen_abs_mm", 0.0), 3), "abs_pair": L.get("pen_abs_pair", ""),
                            "rest_max_mm": round(max(self.rest_depth.values(), default=0.0) * 1000, 3)},
            "speed": {"max": round(L.get("speed", 0.0), 3), "body": L.get("speed_body", "")},
            "track_mm": round(L.get("track_mm", 0.0), 2),
            "alarms": self.alarms.snapshot(),
            "headline": self.alarms.headline(),
            "readouts": self.readouts,
            "input": self.input.as_dict(),
            "arrows": self.arrow_list,
            "events": (self.events_log[-12:]),
        }

    def history(self) -> dict:
        return {k: list(v) for k, v in self.hist.items()}

    def report(self) -> dict:
        return {"scene": self.scene.name, "scene_info": self.scene.info(),
                "frames": self.frame, "sim_time_s": round(self.t, 3),
                "wall_fps": round(self.wall_fps, 1), "graph": self.graph is not None,
                "graph_error": self.graph_error,
                "guard": self.guard.summary(),
                "alarms": self.alarms.summary(),
                "rest_penetration_max_mm": round(max(self.rest_depth.values(), default=0.0) * 1000, 3),
                "diverged": self.diverged,
                "readouts": self.readouts,
                "session_events": [e for e in self.events_log if e.get("channel") == "session"]}


def _qrot_rows(q, v):
    """Row-wise quaternion rotation (N,4) x (N,3)."""
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    u, w = q[:, :3], q[:, 3:4]
    uv = np.sum(u * v, axis=1, keepdims=True)
    uu = np.sum(u * u, axis=1, keepdims=True)
    return 2.0 * uv * u + (w * w - uu) * v + 2.0 * w * np.cross(u, v)
