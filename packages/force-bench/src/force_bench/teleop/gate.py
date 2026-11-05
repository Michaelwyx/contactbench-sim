"""Teleop ACCEPTANCE GATE — model integrity under an adversarial operator.

WHY THIS IS A GATE AND NOT A DEMO
---------------------------------
The guard in `force_bench/teleop/guard.py` turns an abusive command stream
into a survivable one.  The gate makes that an ACCEPTANCE CRITERION: a task
must survive a fixed adversarial session with ZERO INTEGRITY VIOLATIONS,
checked the same way every time.

The criterion is deliberately NOT "the task still succeeds".  A teleoperator
who yanks the gripper sideways SHOULD fail to unlock the door — that is the
task being hard, not the model being broken.  What must never happen is the
MODEL losing integrity:

    tunnelling        a part passes through another part (the sampled
                      contact force then reads 0 N — the failure is silent,
                      which is why it needs a gate)
    floating          a free body holds or gains height while nothing is
                      touching it: it is being carried by a constraint that
                      should not exist
    envelope escape   a part leaves the workspace
    flying objects    frame-to-frame displacement speed above the QA limit
    non-finite state  the solver diverged

A task that FAILS this gate has an asset or mechanism defect worth fixing.
It is not a policy problem, and no controller can be trusted to avoid it.

HOW IT DRIVES THE TASK WITHOUT TOUCHING TASK CODE
-------------------------------------------------
Every task funnels its whole rollout through exactly one `Rig.run(...)`
call.  The gate imports the task module, with the task's own arguments
(as `force-bench run` resolves them), and monkeypatches `Rig.run` (plus any
`Rig` subclass in that module that overrides it) so that, at the moment the
scene is fully built and finalized, the task's scripted policy is REPLACED
by the adversarial session and the task's own `main()` is aborted before it
saves or judges anything.  No task file is read at run time and none is
modified.

The adversary is the abusive-operator profile in TCP terms: every phase
transition is a single-frame STEP, which is what a human hand and a policy
argmax produce.  The task's own start TCP and ORIENTATION are the anchor (a
door is reached sideways, a pick from above), so the abuse is expressed in
the frame the task actually works in.  (A scene without the arm would have
every actuator it has slammed between its limits instead; every ContactBench
task has the arm.)

The guard's zones are the teleop UI's (`teleop.scenes.task_featuremap`): the
declared clearances below for the scene a task builds on, the hard-surface
standoff for every other part, the table and the ground.

USAGE
    force-bench gate --list
    force-bench gate plug_in_usb_2
    force-bench gate plug_in_usb_2 --repeat 4
    force-bench gate plug_in_usb_2 --mode unguarded   # diagnostic contrast, not a verdict

Writes `outputs/teleop_gate/<task>/teleop_gate.json`.
"""

from __future__ import annotations

import argparse
import importlib
import json
import sys
import time
from dataclasses import dataclass, field

import numpy as np
import warp as wp

import newton

from force_bench.paths import OUTPUTS_ROOT
from force_bench.rig import Rig
from force_bench.teleop.guard import (
    ContactAudit,
    FeatureMap,
    GuardCfg,
    SpeedGovernor,
    TeleopGuard,
)

FPS = 60
OUTPUT_DIR = OUTPUTS_ROOT / "teleop_gate"


# ── per-task feature declarations ───────────────────────────────────────
#
# Each number is the SMALLEST RELEVANT CAD CLEARANCE in the region of the
# named body, in metres, with its provenance.  The governor turns it into a
# per-frame travel cap and a lead (=press force) cap, so it must be a
# CLEARANCE the motion has to fit through, not a part size and not a
# positioning tolerance.
#
# `_default` covers every body in the scene with no declared feature.  5 mm
# is deliberately loose-but-bounded: it caps free motion near unremarkable
# geometry at 1.25 mm/frame (75 mm/s) instead of the 4.17 mm/frame the guard
# allows in true free air.  A `_default` that never gets specialized is a
# coverage gap, and the gate reports the covered/total ratio so it shows.

@dataclass
class TaskSpec:
    module: str
    features: dict = field(default_factory=dict)
    note: str = ""


# keyed by the scene module a task builds on (its registry ``source``); a task whose scene is not listed, or is
# built from the kit library, gets the hard-surface standoff on every part (teleop.scenes.task_featuremap)
TASKS: dict[str, TaskSpec] = {
    "nut_bolt_arm": TaskSpec(
        module="force_bench.tasks.nut_bolt_arm",
        # constant 0.23 mm helical clearance measured over 5 full turns
        # (tasks/nut_bolt.py header)
        features={"nut": 0.00023, "bolt": 0.00023, "_default": 0.005},
    ),
    "plug_insert_arm": TaskSpec(
        module="force_bench.tasks.plug_insert_arm",
        # blade has 0.45 mm of x-play and 0.10 mm of y-play per side in the
        # face-plate slot (plug_insert_arm.py LATERAL PENETRATION note)
        features={"plug": 0.0001, "shell": 0.0001, "_default": 0.005},
    ),
    "card_pick": TaskSpec(
        module="force_bench.tasks.card_pick",
        # the card deck sits in the holder slot's 1.5 mm of play, i.e.
        # 0.75 mm per side once CARD_Y_SHIFT centres the 85 mm card in the
        # 86.5 mm slot, and the standing cards are packed on 0.8 mm gaps.
        # Features are keyed on exact rig.meshes labels: card_a/b/c,
        # card_picked and holder.
        features={"card_a": 0.00075, "card_b": 0.00075,
                  "card_c": 0.00075, "card_picked": 0.00075,
                  "holder": 0.00075, "_default": 0.005},
    ),
    # measured build-time clearances rather than the 5 mm _default, which
    # would let the governor step 1.25 mm/frame through a scene whose
    # tightest real clearance is 0.5 mm.  ring/board 0.5 mm is the threading
    # corridor; the fingertip clears the board by 4.5 mm at the hang pose.
    "keychain_hang": TaskSpec(module="force_bench.tasks.keychain_hang",
                              features={"ring": 0.0005, "board": 0.0005,
                                        "key_a": 0.0005, "key_b": 0.0005,
                                        "rod": 0.0045, "_default": 0.005}),
}


# ── integrity monitor ───────────────────────────────────────────────────
@dataclass
class GateCfg:
    duration_s: float = 12.0
    pen_tol_m: float = 0.003
    """Tunnelling threshold, deliberately at INTEGRITY scale rather than the
    suite's 0.25 mm visual standard (that stays `rollout_qa.json`'s job).
    3 mm is chosen from the geometry, not tuned: the largest SDF cell in the
    suite is ~0.7 mm, so 3 mm is >4 cells and past anything contact
    compliance explains; and it is below the thinnest STRUCTURAL wall in the
    assets (the 4.6 mm hinge knuckle, the 2.5 mm face plate is thinner still
    but is a plate a blade passes through by design).  So a 3 mm overlap
    means a part is genuinely inside another, not compressed against it.
    The measured worst depth is always reported, threshold or not."""
    vel_max: float = 2.5              # m/s, same as RolloutQA.check_velocity
    float_rise_m: float = 0.008       # rise above rest that needs support
    float_force_N: float = 0.05       # below this = nothing is touching it
    float_frames: int = 8             # consecutive frames before it counts
    envelope_pad_m: float = 0.5
    guard: bool = True


class IntegrityMonitor:
    """The five integrity checks, evaluated every frame."""

    def __init__(self, rig, cfg: GateCfg):
        self.cfg = cfg
        self.rig = rig
        m = rig.model
        self.labels = list(m.body_label)
        self.audit = ContactAudit(m)
        self.nb = m.body_count
        q0 = rig.s0.body_q.numpy()
        self.z0 = q0[:, 2].copy()
        lo = q0[:, :3].min(axis=0) - cfg.envelope_pad_m
        hi = q0[:, :3].max(axis=0) + cfg.envelope_pad_m
        self.env_lo, self.env_hi = lo, hi
        # a body is FREE-FLOATING if its only connection to the world is a
        # single FREE joint to the ground: anything on a revolute/prismatic
        # is legitimately held by a constraint, and robot links are the
        # robot's business
        jt = m.joint_type.numpy()
        jp = m.joint_parent.numpy()
        jc = m.joint_child.numpy()
        njoints = np.zeros(self.nb, dtype=int)
        free = np.zeros(self.nb, dtype=bool)
        for j in range(len(jt)):
            c = int(jc[j])
            if c < 0:
                continue
            njoints[c] += 1
            if int(jt[j]) == int(newton.JointType.FREE) and int(jp[j]) < 0:
                free[c] = True
        self.free_bodies = [i for i in range(self.nb)
                            if free[i] and njoints[i] == 1
                            and not self.labels[i].startswith("fr3/")]
        # every robot link, for the press servo: a jab that rams the WRIST
        # into a door reads 0 N on the fingertip probes, so a force servo
        # fed only by the pads never engages and the arm keeps pushing
        self.robot_bodies = [i for i in range(self.nb)
                             if self.labels[i].startswith("fr3/")]
        self.robot_force = 0.0
        # PAD FORCE IS MEASURED, NOT LOOKED UP.  Tasks rename their probes
        # freely (task_nut_bolt_arm replaces rig.probes with seat/finL/finR),
        # so a gate that fetches rig.probes["fr3/fr3_leftfinger"] silently
        # reads 0 N and the grip limiter never engages -- which is exactly
        # how the fingers crushed 3.2 mm into a static bolt before this.
        self.pad_idx = [i for i in range(self.nb)
                        if self.labels[i].endswith("leftfinger")
                        or self.labels[i].endswith("rightfinger")]
        self.pad_force = (0.0, 0.0)
        self._float_run = {i: 0 for i in self.free_bodies}
        self._prev_p = None
        self._prev_t = None
        self.phase = "?"
        self.diverged = False
        # REST BASELINE: several interfaces in this suite overlap by design
        # (a square drive inserted in its socket, a rack sliding in its
        # bore), and the tasks' own audits carry per-pair tolerances for
        # exactly that reason.  The gate therefore measures DEGRADATION --
        # how much deeper than at rest -- which is the only question with a
        # scene-independent answer.  `armed` is False while the baseline is
        # being taken during the settle phase.
        self.rest = {}
        self.rest_max_mm = 0.0
        self.armed = False
        self.reset_stats()

    def arm(self):
        self.armed = True
        self.rest_max_mm = round(max(self.rest.values(), default=0.0) * 1000, 3)

    def reset_stats(self):
        self.v = []
        self.stats = {"pen_max_mm": 0.0, "pen_pair": None,
                      "excess_max_mm": 0.0, "excess_pair": None,
                      "vel_max_m_s": 0.0, "vel_body": None,
                      "float_max_frames": 0, "float_body": None,
                      "frames": 0}

    def flag(self, kind, t, **kw):
        self.v.append({"t": round(float(t), 3), "kind": kind, **kw})

    def update(self, state, contacts, t):
        c = self.cfg
        self.stats["frames"] += 1
        bq = state.body_q.numpy()
        if not np.isfinite(bq).all():
            # a diverged solver cannot be measured further: record it once,
            # with the phase that produced it, and stop the session
            if not self.diverged:
                self.flag("non_finite_state", t, phase=self.phase)
                self.diverged = True
            return False
        a = self.audit.audit(contacts, state, self.nb)
        d, pair = a["pen_max_m"], a["pen_pair"]
        if d * 1000 > self.stats["pen_max_mm"]:
            self.stats["pen_max_mm"] = round(d * 1000, 3)
            self.stats["pen_pair"] = pair
        if not self.armed:
            for k, v in a["pairs"].items():
                if v > self.rest.get(k, 0.0):
                    self.rest[k] = v
        else:
            for k, v in a["pairs"].items():
                excess = v - self.rest.get(k, 0.0)
                if excess > self.stats["excess_max_mm"] / 1000:
                    self.stats["excess_max_mm"] = round(excess * 1000, 3)
                    self.stats["excess_pair"] = self.audit._pair_name(*k)
                if excess > c.pen_tol_m:
                    self.flag("tunnelling", t,
                              pair=self.audit._pair_name(*k),
                              depth_mm=round(v * 1000, 3),
                              rest_mm=round(self.rest.get(k, 0.0) * 1000, 3),
                              excess_mm=round(excess * 1000, 3))

        p = bq[:, :3]
        if self._prev_p is not None and t > self._prev_t:
            sp = np.linalg.norm(p - self._prev_p, axis=1) / (t - self._prev_t)
            j = int(np.argmax(sp))
            if sp[j] > self.stats["vel_max_m_s"]:
                self.stats["vel_max_m_s"] = round(float(sp[j]), 3)
                self.stats["vel_body"] = self.labels[j]
            if sp[j] > c.vel_max:
                self.flag("flying_object", t, body=self.labels[j],
                          speed_m_s=round(float(sp[j]), 3))
        self._prev_p, self._prev_t = p.copy(), float(t)

        out = (p < self.env_lo).any(axis=1) | (p > self.env_hi).any(axis=1)
        if out.any():
            j = int(np.argmax(out))
            self.flag("envelope_escape", t, body=self.labels[j],
                      pos=[round(float(x), 4) for x in p[j]])

        f = a["force_per_body"]
        self.robot_force = float(f[self.robot_bodies].sum()) \
            if self.robot_bodies else 0.0
        self.pad_force = tuple(float(f[i]) for i in self.pad_idx[:2]) \
            if len(self.pad_idx) >= 2 else (0.0, 0.0)
        for i in self.free_bodies:
            rise = float(bq[i, 2] - self.z0[i])
            if rise > c.float_rise_m and f[i] < c.float_force_N:
                self._float_run[i] += 1
            else:
                self._float_run[i] = 0
            if self._float_run[i] > self.stats["float_max_frames"]:
                self.stats["float_max_frames"] = self._float_run[i]
                self.stats["float_body"] = self.labels[i]
            if self._float_run[i] == c.float_frames:
                self.flag("floating", t, body=self.labels[i],
                          rise_mm=round(rise * 1000, 2),
                          contact_N=round(float(f[i]), 4))
        return True

    def by_kind(self):
        out = {}
        for x in self.v:
            out[x["kind"]] = out.get(x["kind"], 0) + 1
        return out


# ── adversarial driver ──────────────────────────────────────────────────
def _qrot(q, v):
    q = np.asarray(q, dtype=float); v = np.asarray(v, dtype=float)
    u, w = q[:3], q[3]
    return 2.0 * (u @ v) * u + (w * w - u @ u) * v + 2.0 * w * np.cross(u, v)


def _qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
                     w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


# The profile.  Every entry is (duration_s, name); the command inside a
# phase is CONSTANT, so each transition is a single-frame step of up to
# 120 mm — three orders of magnitude past anything the scripted experts do.
TCP_PHASES = [
    (0.6, "settle"),
    (2.4, "jab_down"),      # step 100 mm along -z (into whatever is below)
    (1.6, "over_squeeze"),  # fingers -> 0 in one frame
    (2.4, "jab_in"),        # step 100 mm along the tool approach axis
    (2.0, "yank_up_yaw"),   # step +100 mm and +115 deg of yaw together
    (2.4, "shake"),         # +-30 mm reversal every 3 frames, max rate
    (0.6, "release"),
]


def tcp_command(t, p0, q0, phases, grip_open, grip_shut, approach):
    """Scene-agnostic abusive command in the task's OWN TCP frame."""
    ends = np.cumsum([d for d, _ in phases])
    i = int(np.searchsorted(ends, t, side="right"))
    i = min(i, len(phases) - 1)
    name = phases[i][1]
    p, g, yaw = p0.copy(), grip_open, 0.0
    if name == "jab_down":
        p = p0 + np.array([0.0, 0.0, -0.10])
    elif name == "over_squeeze":
        p = p0 + np.array([0.0, 0.0, -0.10]); g = grip_shut
    elif name == "jab_in":
        p = p0 + approach * 0.10; g = grip_shut
    elif name == "yank_up_yaw":
        p = p0 + np.array([0.0, 0.0, 0.10]); g = grip_shut; yaw = 2.0
    elif name == "shake":
        k = int((t - (ends[i - 1] if i else 0.0)) * FPS) // 3
        p = p0 + np.array([0.03 * (1 if k % 2 else -1), 0.0, 0.0])
        g = grip_shut
    return p, yaw, g, name


class Adversary:
    """Drives the scene as badly as an operator can, in whichever mode the
    scene supports.  Holds no task knowledge beyond the rig itself."""

    def __init__(self, rig, spec: TaskSpec, cfg: GateCfg):
        self.rig, self.spec, self.cfg = rig, spec, cfg
        self.mode = "tcp" if hasattr(rig, "ik") else "drive"
        m = rig.model
        self.labels = list(m.body_label)
        if self.mode == "tcp":
            tcp = rig.ik.tcp_pose()
            self.p0 = np.asarray(tcp[:3], dtype=float)
            self.q0 = np.asarray(tcp[3:7], dtype=float)
            # HAND OVER, don't yank.  The adversary takes the controls in
            # whatever state the task left them: forcing the fingers to a
            # fixed 40 mm at t=0 is a step command the operator never gave,
            # and on nut_bolt_arm (which starts with the pads already on the
            # nut) it diverged the solver inside 13 frames -- a gate artifact
            # that would have been read as an asset defect.
            self.grip_start = float(rig.control.joint_target_q.numpy()[7])
            # the tool's own approach direction, so "jab in" means into the
            # work for a horizontal door reach AND for a vertical pick
            self.approach = _qrot(self.q0, np.array([0.0, 0.0, 1.0]))
        else:
            self.p0 = self.q0 = self.approach = None
            self.joints = list(rig._joint_of)
            starts = m.joint_target_q_start.numpy()
            lo = m.joint_limit_lower.numpy()
            hi = m.joint_limit_upper.numpy()
            ds = m.joint_qd_start.numpy() if hasattr(m, "joint_qd_start") else starts
            jtype = m.joint_type.numpy()
            jc = m.joint_child.numpy()
            bq = rig.s0.body_q.numpy()
            self.drive = []
            for lbl in self.joints:
                j = rig._joint_of[lbl]
                ci, di = int(starts[j]), int(ds[j])
                a, b = float(lo[di]), float(hi[di])
                if not (np.isfinite(a) and np.isfinite(b)) or abs(b - a) > 1e6:
                    a, b = -0.5, 0.5
                child = int(jc[j])
                pos = bq[child, :3] if child >= 0 else np.zeros(3)
                rot = int(jtype[j]) == int(newton.JointType.REVOLUTE)
                self.drive.append({"label": lbl, "coord": ci, "lo": a,
                                   "hi": b, "pos": np.asarray(pos, float),
                                   "revolute": rot,
                                   "cmd": float(
                                       rig.control.joint_target_q.numpy()[ci])})
            # every free body gets shoved too
            jt = m.joint_type.numpy(); jc = m.joint_child.numpy()
            jp = m.joint_parent.numpy()
            self.shove = [int(jc[j]) for j in range(len(jt))
                          if int(jt[j]) == int(newton.JointType.FREE)
                          and int(jp[j]) < 0 and int(jc[j]) >= 0
                          and not self.labels[int(jc[j])].startswith("fr3/")]
        self.phase = "init"

    # -- TCP mode -------------------------------------------------------
    def raw_command(self, t):
        lo, hi = 0.0, self.grip_start
        p, yaw, g, name = tcp_command(t, self.p0, self.q0, TCP_PHASES, hi, lo,
                                      self.approach)
        self.phase = name
        return p, yaw, g

    def quat(self, yaw):
        """Task's own start orientation, turned by `yaw` about world z."""
        qz = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
        return _qmul(qz, self.q0)

    # -- DRIVE mode -----------------------------------------------------
    def drive_targets(self, t, governor=None):
        """Slam every registered drive between its limits.

        The RAW command flips to the far limit every 0.4 s -- a single-frame
        step of the whole travel.  With a governor the per-frame change is
        clamped to the same feature scale the TCP governor uses, evaluated
        at the driven body's own position; that is the drive-space analogue
        of the speed governor and it is what makes the gate a test of the
        MODEL rather than of the solver's tolerance for step inputs.
        """
        if t < 0.6:                       # settle: take the rest baseline
            self.phase = "settle"
            return
        self.phase = "slam_drives"
        qt = self.rig.control.joint_target_q.numpy()
        k = int(t * FPS) // 24            # flip every 0.4 s
        for d in self.drive:
            goal = d["hi"] if k % 2 else d["lo"]
            if governor is None:
                d["cmd"] = goal
            else:
                lim = governor.limits(d["pos"])
                step = lim["yaw_step_rad"] if d["revolute"] else lim["step_m"]
                delta = float(np.clip(goal - d["cmd"], -step, step))
                d["cmd"] += delta
            qt[d["coord"]] = d["cmd"]
        self.rig.control.joint_target_q.assign(qt)

    def shove_forces(self, t):
        """A rotating 60 N wrench on every free body.

        DIAGNOSTIC ONLY (`--mode unguarded`).  An external wrench applied
        straight to a part is not an action any operator has available, and
        every rigid-body model tunnels under one at 60 fps -- including
        healthy ones -- so it cannot be part of an acceptance criterion.
        """
        if not self.shove:
            return None
        w = 2.0 * np.pi * 0.35 * t
        f = 60.0 * np.array([np.cos(w), np.sin(w), 0.35 * np.sin(2 * w)])
        return {b: f for b in self.shove}


# ── joint-space rate limit (downstream of IK) ──────────────────────────
# Franka Emika Panda / FR3 datasheet joint velocity limits [rad/s].  The
# URDF import leaves Model.joint_velocity_limit at 1e6, so they are stated
# here rather than read back.
FR3_QD_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610])


def drive_arm_guarded(rig, pos, quat, fingers, qd_max=FR3_QD_MAX):
    """`Rig.drive_arm` plus the two things a real robot controller does
    between the IK and the drives: reject a non-finite solution, and rate
    limit the joint targets to the arm's own velocity limits.

    THIS IS WHERE MOST GATE DIVERGENCES CAME FROM.  `FrankaIK.solve` is a
    damped-least-squares solver seeded from its own previous answer; drive
    it toward a pose the arm cannot reach (an operator jamming the gripper
    100 mm into a static bolt) and its solution wanders through
    near-singular configurations, stepping the joint targets 1-3.5 deg per
    FRAME -- above the FR3's real velocity limits -- while the Cartesian
    command sits still.  12 kN.m/rad drives chasing that produce a
    non-finite state with a completely benign 23 N of contact force, which
    is why no amount of contact tuning (iterations, substeps, jaw stops,
    contact filtering, a 6x more aggressive press servo) ever fixed it.

    Clamping to the datasheet velocity limits is not a safety margin bolted
    on top: it is the constraint the physical arm already has.
    """
    q_sol = rig.ik.solve(pos, quat, iterations=16)
    qt = rig.control.joint_target_q.numpy()
    if np.isfinite(q_sol[:7]).all():
        step = qd_max / FPS
        qt[:7] = np.clip(q_sol[:7], qt[:7] - step, qt[:7] + step)
    qt[7:9] = fingers
    rig.control.joint_target_q.assign(qt)


# ── stepping (must match the suite's collision cadence) ─────────────────
def step_frame(rig, forces=None):
    """One control frame.  README pitfall 5.1: `Rig.run` collides ONCE PER
    FRAME so XPBD's friction anchors accumulate a whole frame of resistance.
    A gate on a different cadence would measure a different grasp."""
    sim_dt = 1.0 / FPS / rig.substeps
    nb = rig.model.body_count
    if not rig.collide_every_substep:
        rig.s0.clear_forces()
        rig.pipeline.collide(rig.s0, rig.contacts)
    for _ in range(rig.substeps):
        if rig.collide_every_substep:
            rig.s0.clear_forces()
            rig.pipeline.collide(rig.s0, rig.contacts)
        if forces:
            bf = np.zeros((nb, 6), dtype=np.float32)
            for bi, f in forces.items():
                bf[bi, :3] = f
            rig.s0.body_f.assign(bf)
        rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
        rig.s0, rig.s1 = rig.s1, rig.s0
    rig.solver.update_contacts(rig.contacts, rig.s0)


def build_featuremap(rig, spec: TaskSpec):
    """Zones straight off the assembled geometry: one AABB per named body,
    tagged with that body's declared clearance.  The geometry is derived,
    the NUMBERS are declared — which is the only part a human must get right.

    Zones are padded by 60 mm because they are expressed in TCP coordinates
    and the FR3 fingertip assembly reaches ~25 mm behind the pad face: a
    40 mm-open gripper is ~60 mm wide around the TCP, so a clamp that waits
    for the TCP to arrive engages after the knuckles already have.
    """
    fm = FeatureMap(free_air_m=0.05)
    default = spec.features.get("_default", 0.005)
    hit, missed = [], []
    for label, (tm, c) in rig.meshes.items():
        feat = spec.features.get(label)
        (hit if feat is not None else missed).append(label)
        fm.add_mesh_bounds(label, tm, c, feature_m=default if feat is None
                           else feat, pad=0.06)
    stale = sorted(k for k in spec.features
                   if k != "_default" and k not in rig.meshes)
    return fm, {"bodies": len(hit) + len(missed), "declared": len(hit),
                "default_m": default,
                "declared_labels": sorted(hit),
                "undeclared_labels": sorted(missed),
                "stale_declarations": stale}


class _GateDone(Exception):
    """Raised to abort the task's own main() once the gate has run."""
    def __init__(self, report):
        self.report = report


def run_session(rig, task, source: str, cfg: GateCfg):
    from force_bench.teleop.scenes import task_featuremap
    spec = TASKS.get(source) or TaskSpec(module="")
    fm, _, coverage, _, _ = task_featuremap(rig, source)
    adv = Adversary(rig, spec, cfg)
    mon = IntegrityMonitor(rig, cfg)

    guard = None
    gov = SpeedGovernor(GuardCfg(fps=FPS, substeps=rig.substeps), fm) \
        if cfg.guard else None
    if cfg.guard and adv.mode == "tcp":
        gcfg = GuardCfg(fps=FPS, substeps=rig.substeps,
                        qa_envelope_lo=tuple(mon.env_lo),
                        qa_envelope_hi=tuple(mon.env_hi), qa_stride=6)
        guard = TeleopGuard(gcfg, rig.model, feature_map=fm)
        guard.reset(np.array([*adv.p0, 0.0, adv.grip_start]))
        wi = rig.body_index("fr3/fr3_link7")
        w0 = rig.s0.body_q.numpy()[wi]
        tcp_off = _qrot(np.array([-w0[3], -w0[4], -w0[5], w0[6]]),
                        adv.p0 - np.asarray(w0[:3]))

    t = 0.0
    phase_log = {}
    for _ in range(int(cfg.duration_s * FPS)):
        if adv.mode == "tcp":
            p, yaw, g = adv.raw_command(t)
            if guard is not None:
                bq = rig.s0.body_q.numpy()
                tcp_m = np.asarray(bq[wi][:3]) + _qrot(bq[wi][3:7], tcp_off)
                fL, fR = mon.pad_force
                # the YAW goes through the guard too: a 115 deg orientation
                # step is exactly as violent as a 100 mm position step, and
                # routing it around the governor's yaw-rate clamp is how the
                # first version of this gate diverged the solver in one frame
                cmd = guard.filter(np.array([*p, yaw, g]), measured_tcp=tcp_m,
                                   pad_forces=(fL, fR),
                                   contact_force=max(fL, fR,
                                                     mon.robot_force))
                p, yaw, g = cmd[:3], float(cmd[3]), cmd[4]
            drive_arm_guarded(rig, p, adv.quat(yaw), g)
            forces = None
        else:
            adv.drive_targets(t, governor=gov)
            forces = None if cfg.guard else adv.shove_forces(t)
        step_frame(rig, forces)
        t += 1.0 / FPS
        for pr in rig.probes.values():
            pr.sample(rig.contacts, rig.s0, t)
        mon.phase = adv.phase
        if not mon.armed and adv.phase != "settle":
            mon.arm()      # baseline taken; from here overlap must not grow
        alive = mon.update(rig.s0, rig.contacts, t)
        phase_log.setdefault(adv.phase, 0)
        phase_log[adv.phase] += 1
        if not alive:
            break

    kinds = mon.by_kind()
    report = {
        "task": task,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "mode": adv.mode,
        "guard": bool(guard is not None),
        "duration_s": cfg.duration_s,
        "frames": mon.stats["frames"],
        "phases": phase_log,
        "feature_coverage": coverage,
        "violations_by_kind": kinds,
        "violations": mon.v[:40],
        "worst": mon.stats,
        "rest_penetration_mm": mon.rest_max_mm,
        "diverged": bool(mon.diverged),
        "guard_clamps": (guard.summary()["clamps"] if guard else None),
        "guard_peak_pad_force_N": (round(guard.grip.peak_force, 2)
                                   if guard else None),
        "thresholds": {"tunnelling_mm": cfg.pen_tol_m * 1000,
                       "flying_m_s": cfg.vel_max,
                       "floating_rise_mm": cfg.float_rise_m * 1000,
                       "floating_force_N": cfg.float_force_N},
        "actuators": ({"tcp": True} if adv.mode == "tcp" else
                      {"joint_drives": len(adv.drive),
                       "drive_labels": [d["label"] for d in adv.drive],
                       "free_bodies": len(adv.shove)}),
        "pass": bool(not kinds),
        "criterion": "ZERO integrity violations under a fixed adversarial "
                     "session. Task SUCCESS is explicitly NOT required.",
    }
    # A DRIVE-mode scene with no registered joint drive has nothing the gate
    # can take over (its motion is prescribed inside the control code the
    # gate replaces).  Reporting that as a PASS would be a false pass -- the
    # adversary never touched anything.
    if not mon.armed:
        mon.arm()
    if adv.mode == "drive" and not adv.drive:
        report["pass"] = None
        report["inconclusive"] = (
            "no actuator to drive: the scene sat still and the run proves "
            "nothing.")
    if spec.note:
        report["note"] = spec.note
    return report


def _patch(mod, source, task, cfg):
    """Replace Rig.run (and any subclass override in `mod`) with the gate."""
    def gate_run(self, *a, **kw):
        rep = run_session(self, task, source, cfg)
        raise _GateDone(rep)

    originals = [(Rig, Rig.run)]
    Rig.run = gate_run
    for name, obj in list(vars(mod).items()):
        if (isinstance(obj, type) and issubclass(obj, Rig) and obj is not Rig
                and "run" in obj.__dict__):
            originals.append((obj, obj.__dict__["run"]))
            obj.run = gate_run
    return originals


def gate_task(task: str, cfg: GateCfg, repeat: int = 1) -> dict:
    """Run the gate `repeat` times; the verdict is the AND over all runs.

    Contact ordering in the collision reduction is not bit-reproducible on
    GPU, so a scene sitting at the divergence boundary can flip run to run.
    A verdict that flips is not a pass: the gate reports the flaky task as
    FAIL and records every run so the flakiness is visible rather than
    resampled away.
    """
    runs = [_gate_once(task, cfg) for _ in range(max(1, repeat))]
    if len(runs) == 1:
        return runs[0]
    out = dict(runs[-1])
    verdicts = [r.get("pass") for r in runs]
    out["repeat"] = len(runs)
    out["run_verdicts"] = verdicts
    out["flaky"] = bool(len(set(v for v in verdicts if v is not None)) > 1)
    out["pass"] = (None if all(v is None for v in verdicts)
                   else all(v for v in verdicts if v is not None)
                   and not any(v is None for v in verdicts))
    return out


def _gate_once(task: str, cfg: GateCfg) -> dict:
    from force_bench.appendix_tasks.registry import BY_NAME as ROWS
    from force_bench.cli import _run_plan
    row = ROWS.get(task)
    if row is None:
        return {"task": task, "pass": False, "error": f"unknown task {task!r}; see `force-bench list`"}
    mod = importlib.import_module(row.dotted)
    originals = _patch(mod, row.source, task, cfg)
    argv = sys.argv
    sys.argv = [row.dotted, *_run_plan(task, row, [])[0]]
    t0 = time.time()
    try:
        mod.main()
        rep = {"task": task, "pass": False, "error":
               "task main() returned without reaching Rig.run"}
    except _GateDone as done:
        rep = done.report
    except Exception as exc:
        rep = {"task": task, "pass": False,
               "error": f"{type(exc).__name__}: {exc}"}
    finally:
        sys.argv = argv
        for cls, fn in originals:
            cls.run = fn
    rep["seconds"] = round(time.time() - t0, 1)
    return rep


def write_report(task, rep):
    d = OUTPUT_DIR / task
    d.mkdir(parents=True, exist_ok=True)
    p = d / "teleop_gate.json"
    with open(p, "w") as f:
        json.dump(rep, f, indent=2, default=float)
    return p


def summarize(rep) -> str:
    if "error" in rep:
        return f"{rep['task']:<26s} ERROR {rep['error'][:70]}"
    k = rep["violations_by_kind"]
    w = rep["worst"]
    cov = rep["feature_coverage"]
    verdict = ("PASS" if rep["pass"] is True
               else ("INCON" if rep["pass"] is None else "FAIL"))
    return (f"{rep['task']:<26s} {verdict:<5s} "
            f"mode={rep['mode']:<5s} pen={w['pen_max_mm']:6.3f}mm "
            f"vel={w['vel_max_m_s']:5.2f}m/s "
            f"feat={cov['declared']}/{cov['bodies']}+{len(cov.get('model_zones', []))} zones "
            f"{'FLAKY ' if rep.get('flaky') else ''}"
            f"{rep.get('run_verdicts', '')}"
            f"{k if k else ('{}' if rep['pass'] is not False else 'flaky')}"
            if rep['pass'] is not None else "no actuators")


def main():
    ap = argparse.ArgumentParser(prog="force-bench gate")
    ap.add_argument("tasks", nargs="*", help="ContactBench tasks (force-bench list)")
    ap.add_argument("--all", action="store_true", help="every task with a scripted policy")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--mode", choices=["guarded", "unguarded"],
                    default="guarded")
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--repeat", type=int, default=1,
                    help="run each task N times; the verdict is the AND over "
                         "all runs (a flip is a FAIL -- see gate_task)")
    a = ap.parse_args()
    from force_bench.appendix_tasks.registry import APPENDIX
    if a.list:
        for row in APPENDIX:
            decl = "declared clearances" if row.source in TASKS else "hard-surface standoff"
            print(f"{row.name:<44s} {row.source:<16s} {decl}")
        return
    names = a.tasks or ([row.name for row in APPENDIX if row.scripted] if a.all else [])
    if not names:
        ap.error("give task names or --all")
    cfg = GateCfg(duration_s=a.duration, guard=(a.mode == "guarded"))
    wp.init()
    rows = []
    for t in names:
        print(f"\n=== teleop gate: {t} ({a.mode}) ===", flush=True)
        rep = gate_task(t, cfg, repeat=a.repeat)
        print("wrote", write_report(t, rep))
        rows.append(rep)
        print(summarize(rep), flush=True)
    print("\n" + "=" * 96)
    for r in rows:
        print(summarize(r))
    print("=" * 96)


if __name__ == "__main__":
    main()
