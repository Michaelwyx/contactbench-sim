"""Teleoperation safety layer for the ContactBench scenes.

WHY THIS EXISTS
---------------
Every task in this repo is a *scripted expert* whose trajectory is
co-designed with the contact model: rate-limited approaches, force-servoed
presses, gentle grips, commands closed on measured part state.  Those are
not stylistic choices, they are the conditions under which the physics is
valid.  A human teleoperator — or any unscripted policy — violates all of
them within seconds, and the failures are SILENT rather than loud:

  * README pitfall 6.1 — a strong drive punches a small part through a
    compliant contact inside one frame, and the sampled contact force then
    reads 0 N.  The operator sees a part pass through a wall and a tactile
    channel that says "no contact".  Nothing raises an error.
  * README pitfall 6.5/7 — grip force is a scene-dependent function of
    finger overdrive, not of the commanded number.  The measured window on
    the 7.3 mm chopstick is: ~1.8 N/pad slips, ~4.3 N/pad picks, and a
    ~7 N/pad command (28 N at closure) EJECTS the rod like a watermelon
    seed.  A human squeezing "harder to be safe" destroys the grasp.
  * The plug-insert arm task measured wall presses at 10-50 N/mm of
    position-command lead: any fixed multi-mm lead into a blocked feature
    is tens of newtons and mm-deep penetration.
  * QA (penetration / flying objects / envelope) is post-hoc, so a
    30-minute teleop session is only known to be garbage after it ends.

This module is the guard layer that makes the suite usable outside the
scripted experts.  It is deliberately a pure command filter plus a passive
monitor: it never touches the scene, the solver or the assets, so it can
be dropped in front of ANY command source (keyboard, gizmo, SpaceMouse,
VR, a learned policy, `packages/force-bench/src/force_bench/env.py` actions).

    raw operator command  ->  AdmittanceFilter   (b)
                          ->  SpeedGovernor      (a)   contact-aware clamp
                          ->  GripForceLimiter   (c)   force-limited fingers
                          ->  robot
                          ->  LiveQA             (d)   online violations

WHAT EACH PIECE IS AND WHERE THE NUMBERS COME FROM
--------------------------------------------------
(a) SpeedGovernor — the command's per-frame TCP displacement, and its LEAD
    over the measured TCP, are both clamped to a fraction of the smallest
    relevant mechanical feature near the gripper.  Feature sizes are
    declared per scene through a `FeatureMap` (spheres/AABBs carrying the
    smallest clearance in that region), because the smallest feature is a
    property of the CAD, not something the solver can be asked for: the
    card swiper's slot is 2.1 mm for a 1.2 mm card, the plug's blade has
    0.45 mm of x-play in its slot, the latch bolt has 0.25 mm of clearance
    in its strike pocket.  Away from any declared zone the governor falls
    back to free-air limits.  Clamping the DISPLACEMENT bounds tunneling
    (pitfall 6.1); clamping the LEAD bounds press force (10-50 N/mm).

(b) AdmittanceFilter — EMA + deadband + rate limit on the operator input.
    This is the recipe recorded in `packages/force-bench/src/force_bench/tasks/plug_insert_arm.py`'s header,
    which took the arm's limit-cycle shake from 1.34 mm RMS (bang-bang on
    the raw error, 3.5-5 Hz) to 0.12 mm RMS: alpha 0.25 (fc ~2.7 Hz at
    60 fps), a 0.25 mm deadband, and a per-frame step cap.  Human input is
    exactly the same kind of noisy, stepping signal.

(c) GripForceLimiter — a closed loop on the MEASURED per-pad contact force
    that opens the finger command whenever the squeeze exceeds a
    human-plausible grip.  Default ceiling 6 N/pad, from the measured
    grasp window above (2-6 N holds the 7.3 mm rod, ~15 N ejects it).
    This is the only way to bound grip force here: XPBD finger drives are
    position constraints, so the force is set by overdrive against a
    scene-dependent effective stiffness and cannot be commanded directly.

(d) LiveQA — the RolloutQA checks, run ONLINE:
      * penetration proxy: the solver's own contact geometry, evaluated the
        way the contact kernels do it
        (d = dot(-n, p0_world - margin0*(-n) - (p1_world + margin1*(-n))),
        d < 0 = overlap).  Validated against a known 3.000 mm commanded
        overlap: proxy read 3.000 mm.  This is a PROXY, not a trimesh
        signed-distance ground truth — it only sees pairs the broad phase
        generated — but it is cheap enough to run
        every few frames during a live session.
      * displacement-velocity spike: frame-to-frame position speed, the
        flying-object detector (raw solver body_qd shows single-substep
        impulse artifacts — README rule 6).
      * workspace envelope: tracked bodies inside a world AABB.
    Violations are surfaced immediately through `hud_lines()` / the
    `on_violation` callback and accumulated into `summary()`.

USAGE
-----
    from teleop_guard import TeleopGuard, GuardCfg, FeatureMap

    fmap = FeatureMap(free_air_m=0.05)
    fmap.add_aabb("slot", lo, hi, feature_m=0.0021, pad=0.03)
    guard = TeleopGuard(GuardCfg(), model, feature_map=fmap)
    guard.reset(cmd0)                       # cmd = [x, y, z, yaw, finger]

    # every control frame:
    cmd = guard.filter(raw_cmd, measured_tcp)
    ...  drive the robot with cmd, step the physics ...
    guard.observe(state, contacts, t, pad_forces=(fL, fR))
    for line in guard.hud_lines(): print(line)

`TeleopGuard.enabled = False` bypasses every clamp while leaving LiveQA
running, which is exactly the A/B the abusive-operator harness uses.

Self-test:  python -m force_bench.teleop.guard --selftest    (no GPU / no scene needed)
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

import numpy as np

import newton

# ── configuration ───────────────────────────────────────────────────────


@dataclass
class GuardCfg:
    fps: float = 60.0
    substeps: int = 16

    # (a) contact-aware speed governor ---------------------------------
    feature_fraction: float = 0.25
    """Per-frame commanded TCP displacement <= this * (local feature size).

    0.25 of the smallest local feature per FRAME (not per substep): the
    drive delivers most of a step command inside one frame, which is the
    regime pitfall 6.1 describes.  At the card swiper's 2.1 mm slot that is
    0.52 mm/frame = 31 mm/s; in free air the free-air feature (50 mm)
    would allow 12.5 mm/frame, so `v_max_free` is what binds there."""
    lead_fraction: float = 1.0
    """Command may lead the MEASURED TCP by at most this * feature size.

    The lead IS the press force (10-50 N/mm through the arm), so a 2 mm
    feature gets a 2 mm lead, i.e. tens of newtons at most, and a blocked
    command cannot integrate a limitless push."""
    press_force_max_N: float = 15.0
    """Contact-force ceiling for a PRESS (whole hand, any counterpart).

    A position-command lead into a blocked feature is not a displacement,
    it is a force: measured on this arm at 10-50 N/mm in the plug task and
    at ~400 N/mm for a fingertip against a rigid wall at short reach.  So
    the lead limit alone cannot bound the press, and the governor closes a
    slow servo on the MEASURED contact force that shrinks the lead and, if
    the force keeps climbing, drives it NEGATIVE (the command retreats
    behind the measured TCP).  This is the 'adaptive press budget, allowed
    to go negative' the scripted experts use, generalized."""
    press_kp_m: float = 0.00025       # m of lead removed per unit rel. excess
    press_relax_m: float = 0.00004    # m of lead recovered per frame
    press_retreat_max: float = 0.004  # m of NEGATIVE lead allowed
    retreat_step_scale: float = 4.0
    """How much faster than an advance the command may back off.  Retreating
    always reduces contact force, so it may be quicker -- but it must still
    be rate limited, or the force servo becomes a bang-bang oscillator."""
    v_max_free: float = 0.25          # m/s ceiling in free air
    v_min: float = 0.004              # m/s floor (never freeze the operator)
    lead_max_free: float = 0.020      # m
    lead_min: float = 0.003
    """Floor on the lead budget, in metres.

    It must sit ABOVE the arm's own steady gravity droop (~0.5-2 mm at these
    reaches, README pitfall 8.1/18), because droop is not press: a lead
    limit tighter than the droop is measuring the arm's weight, not contact,
    and freezes the operator for a reason that has nothing to do with
    safety.  Fine bounding of press force is the FORCE SERVO's job — it
    reads newtons instead of inferring them from a position error."""
    yaw_rate_free: float = 1.5        # rad/s in free air
    yaw_rate_min: float = 0.05        # rad/s at the tightest feature

    # (b) admittance / smoothing filter --------------------------------
    ema_alpha: float = 0.25           # fc ~2.7 Hz at 60 fps
    deadband_m: float = 0.00025       # 0.25 mm
    deadband_rad: float = 0.002
    finger_deadband_m: float = 0.0002

    # (c) force-limited finger drive ------------------------------------
    grip_force_max_N: float = 6.0
    """Ceiling per pad.  Measured window (a chopstick pick, 7.3 mm
    rod): 1.8 N/pad slips, 4.3 N/pad picks, ~7 N/pad commanded (28 N at
    closure) ejects the rod.  6 N is the top of the human-plausible band."""
    grip_relief_kp: float = 8.0e-5    # m of finger opening per N of excess
    grip_relief_max: float = 0.012    # m
    grip_relief_decay: float = 0.97   # per frame, once back under the limit
    grip_close_rate: float = 0.05     # m/s max finger closing rate
    finger_range: tuple = (0.0, 0.04)
    finger_stop_margin: float = 0.001
    """Keep the finger command this far off its own upper joint stop.  A
    body that starts or sits exactly ON a joint limit is pinned there
    (README pitfall 4.7), and a stiff finger drive pressed into that stop is
    a divergence source."""

    # (d) live QA --------------------------------------------------------
    qa_grip_force_N: float | None = None
    """Live-QA grip-force alarm, N per pad.  None = 1.25 * the limiter's
    ceiling: a guard that is HOLDING the squeeze at its limit is working,
    not violating, so the alarm has to sit above the limiter's own set
    point or every guarded session reads as one long violation."""
    qa_stride: int = 3                # frames between penetration reads
    qa_vel_max: float = 2.5           # m/s frame-to-frame displacement
    qa_pen_max_m: float = 0.0015      # penetration proxy budget
    qa_envelope_lo: tuple | None = None
    qa_envelope_hi: tuple | None = None
    qa_track_bodies: list = field(default_factory=list)
    """Body labels the envelope check applies to.  Empty = all bodies."""
    qa_ignore_shape_pairs: list = field(default_factory=list)
    """(shapeA, shapeB) index pairs excluded from the penetration proxy —
    for interfaces that overlap BY DESIGN (a rack sliding in its bore)."""


# ── feature map ─────────────────────────────────────────────────────────


class FeatureMap:
    """Smallest relevant mechanical feature as a function of TCP position.

    A zone is a region of the workspace tagged with the smallest clearance
    the manipulation there has to respect.  Zones are declared by the task
    because that number lives in the CAD, not in the solver:

        card swiper slot      2.1 mm for a 1.2 mm card + 0.82 mm pad blade
        plug blade in slot    0.45 mm of x-play per side
        latch bolt in strike  0.25 mm of swing clearance
        7.3 mm chopstick      the rod itself

    `pad` grows the zone outward so the governor tightens BEFORE the
    gripper arrives (a clamp that engages on contact is already too late).
    Overlapping zones take the minimum feature.
    """

    def __init__(self, free_air_m: float = 0.05):
        self.free_air_m = float(free_air_m)
        self._zones: list[dict] = []

    # -- declaration ----------------------------------------------------
    def add_sphere(self, name, center, radius, feature_m, pad=0.02):
        self._zones.append(dict(kind="sphere", name=name,
                                c=np.asarray(center, dtype=float),
                                r=float(radius) + float(pad),
                                f=float(feature_m)))
        return self

    def add_aabb(self, name, lo, hi, feature_m, pad=0.02):
        self._zones.append(dict(kind="aabb", name=name,
                                lo=np.asarray(lo, dtype=float) - pad,
                                hi=np.asarray(hi, dtype=float) + pad,
                                f=float(feature_m)))
        return self

    def add_mesh_bounds(self, name, tm, center, feature_m, pad=0.02):
        """Zone from a trimesh's AABB, moved to its placed `center`."""
        b = np.asarray(tm.bounds, dtype=float)
        mid = 0.5 * (b[0] + b[1])
        off = np.asarray(center, dtype=float) - mid
        return self.add_aabb(name, b[0] + off, b[1] + off, feature_m, pad)

    # -- query ----------------------------------------------------------
    def feature_at(self, p) -> tuple[float, str]:
        p = np.asarray(p, dtype=float)
        best, who = self.free_air_m, "free_air"
        for z in self._zones:
            if z["kind"] == "sphere":
                inside = np.linalg.norm(p - z["c"]) <= z["r"]
            else:
                inside = bool(np.all(p >= z["lo"]) and np.all(p <= z["hi"]))
            if inside and z["f"] < best:
                best, who = z["f"], z["name"]
        return best, who

    def __len__(self):
        return len(self._zones)


# ── (b) admittance / smoothing filter ───────────────────────────────────


class AdmittanceFilter:
    """EMA + deadband on a command vector.

    The rate limit is NOT applied here — it belongs to the governor, which
    knows the local feature size.  Order matters: smoothing first (so the
    governor is not fed operator chatter), clamp second.
    """

    def __init__(self, cfg: GuardCfg):
        self.cfg = cfg
        self.state: np.ndarray | None = None

    def reset(self, cmd):
        self.state = np.asarray(cmd, dtype=float).copy()

    def __call__(self, raw) -> np.ndarray:
        raw = np.asarray(raw, dtype=float)
        if self.state is None:
            self.reset(raw)
            return self.state.copy()
        a = self.cfg.ema_alpha
        db = np.array([self.cfg.deadband_m] * 3
                      + [self.cfg.deadband_rad, self.cfg.finger_deadband_m])
        d = raw - self.state
        # deadband: ignore motion smaller than the solver's own noise floor
        d = np.where(np.abs(d) < db, 0.0, d)
        self.state = self.state + a * d
        return self.state.copy()


# ── (a) contact-aware speed governor ────────────────────────────────────


class SpeedGovernor:
    """Clamp commanded TCP motion to the local mechanical feature scale."""

    def __init__(self, cfg: GuardCfg, feature_map: FeatureMap | None = None):
        self.cfg = cfg
        self.fmap = feature_map or FeatureMap()
        self.press_lead = None      # m, the force-servoed lead budget
        self.press_active = 0       # frames the servo has been cutting in

    def reset(self):
        self.press_lead = None
        self.press_active = 0

    def limits(self, p) -> dict:
        c = self.cfg
        f, zone = self.fmap.feature_at(p)
        dt = 1.0 / c.fps
        step = float(np.clip(c.feature_fraction * f, c.v_min * dt,
                             c.v_max_free * dt))
        lead = float(np.clip(c.lead_fraction * f, c.lead_min, c.lead_max_free))
        scale = float(np.clip(f / max(self.fmap.free_air_m, 1e-9), 0.0, 1.0))
        yaw = float(np.clip(c.yaw_rate_free * scale, c.yaw_rate_min,
                            c.yaw_rate_free)) * dt
        return {"feature_m": f, "zone": zone, "step_m": step,
                "lead_m": lead, "yaw_step_rad": yaw,
                "speed_m_s": step * c.fps,
                "substep_travel_m": step / max(c.substeps, 1)}

    def clamp(self, prev_cmd, new_cmd, measured_tcp=None, contact_force=None):
        """prev_cmd/new_cmd = [x,y,z,yaw,finger]; returns (cmd, info)."""
        c = self.cfg
        prev = np.asarray(prev_cmd, dtype=float).copy()
        new = np.asarray(new_cmd, dtype=float).copy()
        lim = self.limits(prev[:3])
        out = new.copy()

        # force servo on the lead budget (may go negative = retreat)
        lead_budget = lim["lead_m"]
        if contact_force is not None:
            if self.press_lead is None:
                self.press_lead = lead_budget
            excess = float(contact_force) / max(c.press_force_max_N, 1e-9) - 1.0
            if excess > 0.0:
                self.press_lead -= c.press_kp_m * excess
                self.press_active += 1
            else:
                self.press_lead += c.press_relax_m
            self.press_lead = float(np.clip(self.press_lead,
                                            -c.press_retreat_max, lead_budget))
            lead_budget = min(lead_budget, self.press_lead)

        # rate limit on the translation
        d = new[:3] - prev[:3]
        n = float(np.linalg.norm(d))
        rate_clipped = n > lim["step_m"]
        if rate_clipped and n > 0:
            d = d * (lim["step_m"] / n)
        out[:3] = prev[:3] + d

        # lead limit against the MEASURED TCP (this is the press force)
        lead_clipped = False
        if measured_tcp is not None:
            m = np.asarray(measured_tcp, dtype=float)[:3]
            e = out[:3] - m
            en = float(np.linalg.norm(e))
            if lead_budget < 0.0 and en > 1e-12:
                # the FORCE SERVO is explicitly commanding a retreat: this is
                # the only case in which the command may be driven BEHIND the
                # measured TCP, and it happens because a measured force says
                # so, not because the arm is lagging.
                #
                # It is RATE LIMITED below for a reason.  Snapping straight to
                # `m + e_hat*lead_budget` recomputes the direction from an
                # error that FLIPS SIGN the moment the command lands behind
                # the arm, so the command ping-pongs +-4 mm every frame and
                # slams 12 kN.m/rad joint drives back and forth: measured on
                # task_nut_bolt_arm as 2-3.5 deg/frame of joint-target chatter
                # ending in a non-finite state.  This is exactly the
                # bang-bang limit cycle task_plug_insert_arm documented
                # (1.34 mm RMS shake) and the reason this module has an
                # admittance filter at all.
                out[:3] = m + (e / en) * lead_budget
                lead_clipped = True
            elif en > lead_budget:
                # HOLD, NEVER RETREAT.  Snapping the command onto the measured
                # TCP looks like the same clamp but is a runaway: the arm's
                # steady gravity droop IS a lead, so the command follows the
                # arm down, which lets it droop further, which drags the
                # command down again.  Measured on task_nut_bolt_arm's start
                # pose (feature 0.23 mm -> 0.5 mm lead): the arm walked ~15 mm
                # into the floor in 0.23 s and the solver went non-finite,
                # while the same pose held rock-steady with no guard at all.
                # Freezing the command instead bounds the press exactly as
                # well (the command cannot integrate into a wall) and cannot
                # move the robot anywhere the operator did not ask for.
                out[:3] = prev[:3]
                lead_clipped = True

        # FINAL RATE LIMIT.  The lead clamp above can move the command a long
        # way in one frame; nothing the guard emits may ever be a step.
        # Retreating is allowed to be faster than advancing (it always
        # reduces force), but it still has to be smooth.
        d2 = out[:3] - prev[:3]
        n2 = float(np.linalg.norm(d2))
        cap = lim["step_m"] * (c.retreat_step_scale if lead_clipped else 1.0)
        if n2 > cap and n2 > 0.0:
            out[:3] = prev[:3] + d2 * (cap / n2)
            rate_clipped = True

        # yaw rate limit
        dy = float(new[3] - prev[3])
        yaw_clipped = abs(dy) > lim["yaw_step_rad"]
        if yaw_clipped:
            dy = np.sign(dy) * lim["yaw_step_rad"]
        out[3] = prev[3] + dy

        info = dict(lim)
        info.update(rate_clipped=bool(rate_clipped),
                    lead_clipped=bool(lead_clipped),
                    yaw_clipped=bool(yaw_clipped),
                    requested_step_m=n,
                    lead_budget_m=float(lead_budget),
                    contact_force_N=(float(contact_force)
                                     if contact_force is not None else 0.0),
                    press_servo=bool(lead_budget < lim["lead_m"] - 1e-12))
        return out, info


# ── (c) force-limited finger drive ──────────────────────────────────────


class GripForceLimiter:
    """Bound per-pad grip force by opening the finger command.

    XPBD finger drives are position CONSTRAINTS: the force is whatever the
    overdrive times a scene-dependent effective stiffness produces (README
    pitfall 6.5), so there is no torque to saturate.  The only actuator the
    guard has is the commanded width, and the loop is a leaky integrator
    on the measured excess — the same shape as the plug task's relief
    integrator, which is deadbanded and decaying so it cannot wind up.
    """

    def __init__(self, cfg: GuardCfg):
        self.cfg = cfg
        self.reset()

    def reset(self, width=None):
        self.relief = 0.0
        self.prev = None if width is None else float(width)
        self.peak_force = 0.0
        self.excess_frames = 0
        self.unrelievable_frames = 0
        self._prev_f = None

    def limit(self, cmd_width, fL, fR):
        c = self.cfg
        f = max(float(fL), float(fR))
        self.peak_force = max(self.peak_force, f)
        excess = f - c.grip_force_max_N
        lo, hi = c.finger_range
        # never command the drive ONTO its own joint stop: a body that sits
        # exactly on a limit is pinned there (README pitfall 4.7), and an
        # 8 kN/m finger drive pressed into that stop is what diverges
        hi_soft = hi - c.finger_stop_margin
        if excess > 0.0:
            self.excess_frames += 1
            # ANTI-WINDUP.  Integrate only while opening can still do
            # something.  A force that does NOT fall as the jaws open is not
            # a grip -- on task_nut_bolt_arm it was the whole hand jammed
            # sideways into a static bolt -- and integrating against it drove
            # the finger command from 30 mm to 35 mm and onto the joint stop,
            # taking the solver non-finite while the pads read a placid 8.8 N.
            saturated = float(cmd_width) + self.relief >= hi_soft - 1e-9
            responding = (self._prev_f is None or f < self._prev_f - 1e-6
                          or self.relief <= 0.0)
            if not saturated and responding:
                self.relief = min(c.grip_relief_max,
                                  self.relief + c.grip_relief_kp * excess)
            self.unrelievable_frames += int(not responding)
        else:
            self.relief *= c.grip_relief_decay
        self._prev_f = f
        w = float(cmd_width) + self.relief
        w = min(w, hi_soft)
        # slew the CLOSING direction only: opening fast is always safe
        if self.prev is not None:
            max_close = c.grip_close_rate / c.fps
            if w < self.prev - max_close:
                w = self.prev - max_close
        w = float(np.clip(w, lo, hi_soft))
        self.prev = w
        return w, {"pad_force_N": f, "relief_m": self.relief,
                   "over_limit": bool(excess > 0.0),
                   "unrelievable": bool(self.unrelievable_frames > 0)}


# ── (d) live QA ─────────────────────────────────────────────────────────


def _qrot(q, v):
    """Rotate v by quaternion q (xyzw), batched."""
    u = q[..., :3]
    w = q[..., 3:4]
    return (2.0 * np.sum(u * v, axis=-1, keepdims=True) * u
            + (w * w - np.sum(u * u, axis=-1, keepdims=True)) * v
            + 2.0 * w * np.cross(u, v))


class PenetrationProxy:
    """Online overlap depth from the solver's own contact geometry.

    Reproduces the contact kernels' gap:
        n  = -contact_normal                       (B-to-A)
        a  = X_wb(body0) * point0 - margin0 * n
        b  = X_wb(body1) * point1 + margin1 * n
        d  = dot(n, a - b)        d < 0  =>  overlap of depth -d

    Validated: a box placed 3.000 mm inside the ground plane reads 3.000 mm.
    It only sees pairs the broad phase generated, so it is a PROXY — the
    ground truth is a trimesh signed distance.

    THE MARGINS COME FROM THE MODEL, NOT FROM THE CONTACT ARRAY (2026-08-14),
    and that is not a micro-optimisation -- it is what makes this proxy mean
    anything under SolverMuJoCo.  `SolverMuJoCo.update_contacts` REBUILDS the
    Newton contact buffer from MuJoCo's own contact list: it overwrites
    count/shape0/shape1/point0/point1/normal/force, and it does NOT touch
    `rigid_contact_margin0/1`.  MuJoCo's contact ordering is its own, so
    slot i ends up holding a DIFFERENT PAIR with the PREVIOUS pair's margin,
    and this proxy -- which subtracts margin0+margin1 from the point gap --
    then reports that margin as penetration.
    MEASURED on an upright-door scene, one settle with no adversary and nothing
    touching anything: XPBD reports no contact at all, MuJoCo reports
    `fr3_leftfinger<->leaf:panel_face` 22.775 mm deep, on a pair whose
    surfaces are 30 mm apart.  22.7 mm is not a coincidence: the knob's
    analytic ball collider has radius 27.3 mm, that is its margin, and it is
    the value left in the slot.  The whole 1126-event "tunnelling" verdict
    on that task, its 22.8 mm REST baseline and its 29.7 mm worst pair were
    that artifact; the numbers moved with the door's mesh margins, not with
    the door.

    THE MARGIN TERM BELONGS TO WHOEVER PRODUCED THE BUFFER, and that is the
    whole rule:
      * Newton's pipeline puts a rounded shape's contact point at its
        CENTRE and the rounding radius in the margin, so the margin must be
        subtracted.  It is a per-shape constant (`radius_eff + margin`,
        geometry/contact_match.py), so it is read off the model here rather
        than out of the (possibly stale) array.
      * `SolverMuJoCo.update_contacts` emits SURFACE points already
        (`pos -+ 0.5*dist*normal`, solvers/mujoco/kernels.py), because a
        MuJoCo sphere geom is a real sphere.  Subtracting a margin there
        double-counts the radius: measured, the same untouched pads read
        14.29 mm of "penetration" against the knob ball whose radius is
        27.3 mm and whose true separation is 13 mm.
    Hence `mujoco_contacts` (default True: SolverMuJoCo is the only backend).
    """

    # geometry whose contact margin is its own rounding radius; everything
    # else (mesh, box, plane, cylinder, cone) contacts on its raw surface
    _ROUNDED = (int(newton.GeoType.SPHERE), int(newton.GeoType.CAPSULE))

    def __init__(self, model, ignore_shape_pairs=(), mujoco_contacts=None):
        self.shape_body = model.shape_body.numpy()
        if mujoco_contacts is None:
            mujoco_contacts = True
        self.mujoco_contacts = bool(mujoco_contacts)
        st = model.shape_type.numpy()
        sc = model.shape_scale.numpy()
        self.shape_margin = np.where(np.isin(st, self._ROUNDED),
                                     sc[:, 0], 0.0).astype(float)
        try:
            self.shape_margin += model.shape_margin.numpy().astype(float)
        except Exception:
            pass
        if self.mujoco_contacts:
            self.shape_margin = np.zeros_like(self.shape_margin)
        try:
            self.shape_label = list(model.shape_label)
        except Exception:
            self.shape_label = None
        try:
            self.body_label = list(model.body_label)
        except Exception:
            self.body_label = None
        self.ignore = {tuple(sorted(p)) for p in ignore_shape_pairs}

    def _parse(self, contacts, state):
        """One pass over the live contact arrays -> (depth, shape0, shape1,
        body0, body1, |force|).  Shared by worst() and ContactAudit."""
        n = int(contacts.rigid_contact_count.numpy()[0])
        if n <= 0:
            return None
        n = min(n, len(contacts.rigid_contact_shape0))
        s0 = contacts.rigid_contact_shape0.numpy()[:n]
        s1 = contacts.rigid_contact_shape1.numpy()[:n]
        # Two shapes on the SAME rigid body cannot move relative to each
        # other, so any overlap between them is a modelling constant, not a
        # manipulation artifact.  (collapse_fixed_joints merges the FR3's
        # hand shells into fr3_link7, and those shells overlap by 22.2 mm
        # in the stock URDF.)  Static-vs-static falls out of the same rule,
        # both being body -1.
        keep = (s0 >= 0) & (s1 >= 0) & (s0 != s1)
        keep &= self.shape_body[np.clip(s0, 0, None)] != \
            self.shape_body[np.clip(s1, 0, None)]
        if not keep.any():
            return None
        p0 = contacts.rigid_contact_point0.numpy()[:n][keep]
        p1 = contacts.rigid_contact_point1.numpy()[:n][keep]
        nm = contacts.rigid_contact_normal.numpy()[:n][keep]
        s0, s1 = s0[keep], s1[keep]
        m0 = self.shape_margin[s0]
        m1 = self.shape_margin[s1]
        bq = state.body_q.numpy()
        b0, b1 = self.shape_body[s0], self.shape_body[s1]
        w0, w1 = p0.astype(float).copy(), p1.astype(float).copy()
        k = b0 >= 0
        if k.any():
            w0[k] = bq[b0[k], :3] + _qrot(bq[b0[k], 3:7], p0[k])
        k = b1 >= 0
        if k.any():
            w1[k] = bq[b1[k], :3] + _qrot(bq[b1[k], 3:7], p1[k])
        nn = -nm.astype(float)
        d = np.sum(nn * ((w0 - m0[:, None] * nn) - (w1 + m1[:, None] * nn)),
                   axis=1)
        fmag = None
        if getattr(contacts, "force", None) is not None:
            fmag = np.linalg.norm(
                contacts.force.numpy()[:n][keep].astype(float), axis=1)
        if self.ignore:
            mask = np.array([tuple(sorted((int(a), int(b)))) not in self.ignore
                             for a, b in zip(s0, s1)])
            if not mask.any():
                return None
            d, s0, s1 = d[mask], s0[mask], s1[mask]
            b0, b1 = b0[mask], b1[mask]
            if fmag is not None:
                fmag = fmag[mask]
        return d, s0, s1, b0, b1, fmag

    def _pair_name(self, a, b):
        """Readable pair.  Most URDF shapes carry no label of their own, so
        the OWNING BODY is what makes a report legible ("fr3/fr3_leftfinger"
        rather than "shape_48")."""
        def one(i):
            i = int(i)
            body = int(self.shape_body[i])
            owner = self.body_label[body] if (self.body_label and body >= 0) \
                else "static"
            lbl = self.shape_label[i] if self.shape_label else f"shape{i}"
            return owner if lbl.startswith("shape_") else f"{owner}:{lbl}"
        return f"{one(a)}<->{one(b)}"

    def worst(self, contacts, state):
        r = self._parse(contacts, state)
        if r is None:
            return 0.0, None
        d, s0, s1, _, _, _ = r
        i = int(np.argmin(d))
        depth = max(0.0, -float(d[i]))
        if depth <= 0.0:
            return 0.0, None
        return depth, self._pair_name(s0[i], s1[i])


class ContactAudit(PenetrationProxy):
    """One contact-array pass giving BOTH the penetration proxy and the
    total contact force carried by each body.

    The per-body force is what makes the FLOATING check possible without
    adding a `SensorContact` per body: sensors must be constructed before
    `CollisionPipeline.contacts()`, i.e. before the task hands the rig over,
    so a gate that attaches later has to read the contact arrays directly.
    Same exclusions as the penetration proxy (same-body pairs).
    """

    def audit(self, contacts, state, nbody, deep_floor=0.0005) -> dict:
        """`deep_floor` selects the contacts worth naming individually.  The
        per-PAIR breakdown is what lets a caller compare against a REST
        baseline instead of an absolute threshold: several interfaces in
        this suite overlap by design (an inserted square drive, a rack in
        its bore), so "how much deeper than at rest" is the only question
        with a scene-independent answer."""
        out = {"pen_max_m": 0.0, "pen_pair": None, "pairs": {},
               "force_per_body": np.zeros(nbody)}
        r = self._parse(contacts, state)
        if r is None:
            return out
        d, s0, s1, b0, b1, fmag = r
        i = int(np.argmin(d))
        depth = max(0.0, -float(d[i]))
        if depth > 0.0:
            out["pen_max_m"] = depth
            out["pen_pair"] = self._pair_name(s0[i], s1[i])
        deep = np.where(d < -deep_floor)[0]
        for k in deep:
            key = (int(s0[k]), int(s1[k]))
            v = -float(d[k])
            if v > out["pairs"].get(key, 0.0):
                out["pairs"][key] = v
        if fmag is not None:
            f = out["force_per_body"]
            for arr in (b0, b1):
                k = arr >= 0
                if k.any():
                    np.add.at(f, arr[k], fmag[k])
        return out


class LiveQA:
    """RolloutQA's checks, online: penetration proxy, displacement-velocity
    spike, workspace envelope, non-finite state."""

    def __init__(self, cfg: GuardCfg, model):
        self.cfg = cfg
        self.labels = list(model.body_label)
        self.pen = PenetrationProxy(model, cfg.qa_ignore_shape_pairs)
        idx = [self.labels.index(l) for l in cfg.qa_track_bodies] \
            if cfg.qa_track_bodies else list(range(len(self.labels)))
        self.track = np.asarray(idx, dtype=int)
        self.reset()

    def reset(self):
        self._prev_p = None
        self._prev_t = None
        self._i = 0
        self.violations: list[dict] = []
        self.stats = {"pen_max_m": 0.0, "pen_worst_pair": None,
                      "vel_max_m_s": 0.0, "vel_worst_body": None,
                      "envelope_violations": 0, "nonfinite_frames": 0,
                      "frames": 0}

    def _flag(self, kind, t, **kw):
        v = {"t": round(float(t), 3), "kind": kind, **kw}
        self.violations.append(v)
        return v

    def update(self, state, contacts, t) -> list[dict]:
        c = self.cfg
        self._i += 1
        self.stats["frames"] += 1
        new: list[dict] = []
        bq = state.body_q.numpy()

        if not np.isfinite(bq).all():
            self.stats["nonfinite_frames"] += 1
            new.append(self._flag("nonfinite", t))
            return new

        p = bq[self.track, :3]
        if self._prev_p is not None and t > self._prev_t:
            v = np.linalg.norm(p - self._prev_p, axis=1) / (t - self._prev_t)
            j = int(np.argmax(v))
            if v[j] > self.stats["vel_max_m_s"]:
                self.stats["vel_max_m_s"] = float(v[j])
                self.stats["vel_worst_body"] = self.labels[int(self.track[j])]
            if v[j] > c.qa_vel_max:
                new.append(self._flag("velocity_spike", t,
                                      body=self.labels[int(self.track[j])],
                                      speed_m_s=round(float(v[j]), 3)))
        self._prev_p, self._prev_t = p.copy(), float(t)

        if c.qa_envelope_lo is not None:
            lo = np.asarray(c.qa_envelope_lo, dtype=float)
            hi = np.asarray(c.qa_envelope_hi, dtype=float)
            out = ((p < lo).any(axis=1) | (p > hi).any(axis=1))
            if out.any():
                self.stats["envelope_violations"] += int(out.sum())
                j = int(np.argmax(out))
                new.append(self._flag("envelope", t,
                                      body=self.labels[int(self.track[j])],
                                      pos=[round(float(x), 4) for x in p[j]]))

        if contacts is not None and self._i % max(c.qa_stride, 1) == 0:
            depth, pair = self.pen.worst(contacts, state)
            if depth > self.stats["pen_max_m"]:
                self.stats["pen_max_m"] = depth
                self.stats["pen_worst_pair"] = pair
            if depth > c.qa_pen_max_m:
                new.append(self._flag("penetration", t, pair=pair,
                                      depth_mm=round(depth * 1000, 3)))
        return new

    def report(self) -> dict:
        c = self.cfg
        s = dict(self.stats)
        s["pen_max_mm"] = round(s.pop("pen_max_m") * 1000, 3)
        s["vel_max_m_s"] = round(s["vel_max_m_s"], 3)
        s["violations"] = len(self.violations)
        s["by_kind"] = {}
        for v in self.violations:
            s["by_kind"][v["kind"]] = s["by_kind"].get(v["kind"], 0) + 1
        s["pass"] = bool(s["pen_max_mm"] <= c.qa_pen_max_m * 1000
                         and s["vel_max_m_s"] <= c.qa_vel_max
                         and s["envelope_violations"] == 0
                         and s["nonfinite_frames"] == 0)
        return s


# ── the guard ───────────────────────────────────────────────────────────


class TeleopGuard:
    """The whole safety layer behind one command filter + one monitor.

    `enabled = False` bypasses every clamp (LiveQA keeps running), which is
    the A/B the abusive-operator harness measures.
    """

    def __init__(self, cfg: GuardCfg | None = None, model=None,
                 feature_map: FeatureMap | None = None,
                 on_violation=None):
        self.cfg = cfg or GuardCfg()
        self.enabled = True
        self.fmap = feature_map or FeatureMap()
        self.filt = AdmittanceFilter(self.cfg)
        self.gov = SpeedGovernor(self.cfg, self.fmap)
        self.grip = GripForceLimiter(self.cfg)
        self.qa = LiveQA(self.cfg, model) if model is not None else None
        self.on_violation = on_violation
        self.cmd = None
        self.last_info: dict = {}
        self.counts = {"rate_clipped": 0, "lead_clipped": 0,
                       "yaw_clipped": 0, "grip_clipped": 0,
                       "press_servo": 0, "frames": 0}
        self._hud_t = 0.0

    # -- command path ---------------------------------------------------
    def reset(self, cmd0):
        self.cmd = np.asarray(cmd0, dtype=float).copy()
        self.filt.reset(self.cmd)
        self.gov.reset()
        self.grip.reset(self.cmd[4] if len(self.cmd) > 4 else None)
        if self.qa is not None:
            self.qa.reset()
        for k in self.counts:
            self.counts[k] = 0
        return self.cmd.copy()

    def filter(self, raw_cmd, measured_tcp=None,
               pad_forces: tuple | None = None,
               contact_force: float | None = None) -> np.ndarray:
        """raw_cmd = [x, y, z, yaw, finger_width] -> guarded command."""
        raw = np.asarray(raw_cmd, dtype=float)
        if self.cmd is None:
            self.reset(raw)
            return self.cmd.copy()
        self.counts["frames"] += 1

        if not self.enabled:
            self.cmd = raw.copy()
            # keep every clamp's anchor on the live command, so re-enabling
            # the guard starts from where the robot is instead of replaying
            # the EMA state, the finger slew and the press lead it had when
            # it was last on (measured: a 12 mm jaw snap and a TCP step back)
            self.filt.state = raw.copy()
            self.grip.prev = float(raw[4]) if len(raw) > 4 else None
            self.grip.relief = 0.0
            self.grip._prev_f = None
            self.gov.press_lead = None
            self.gov.press_active = 0
            # the bypass skips GripForceLimiter.limit: keep its peak /
            # over-limit bookkeeping honest for summary()
            if pad_forces is not None:
                fp = max(float(pad_forces[0]), float(pad_forces[1]))
                self.grip.peak_force = max(self.grip.peak_force, fp)
                if fp > self.cfg.grip_force_max_N:
                    self.grip.excess_frames += 1
            f, zone = self.fmap.feature_at(self.cmd[:3])
            self.last_info = {"feature_m": f, "zone": zone,
                              "requested_step_m": float(np.linalg.norm(
                                  raw[:3] - self.cmd[:3])),
                              "bypassed": True}
            return self.cmd.copy()

        smoothed = self.filt(raw)                       # (b)
        out, info = self.gov.clamp(self.cmd, smoothed, measured_tcp,
                                   contact_force)       # (a)
        if pad_forces is not None:                      # (c)
            w, ginfo = self.grip.limit(out[4], pad_forces[0], pad_forces[1])
            if abs(w - out[4]) > 1e-9:
                self.counts["grip_clipped"] += 1
            out[4] = w
            info.update(ginfo)
        for k in ("rate_clipped", "lead_clipped", "yaw_clipped",
                  "press_servo"):
            if info.get(k):
                self.counts[k] += 1
        # the filter's own state must follow the clamped command, or the
        # EMA keeps integrating an unreachable target and the operator's
        # input stops corresponding to the robot (windup)
        self.filt.state = out.copy()
        self.cmd = out
        self.last_info = info
        return out.copy()

    # -- monitor path ---------------------------------------------------
    def observe(self, state, contacts, t, pad_forces=None):
        new = []
        if self.qa is not None:
            new = self.qa.update(state, contacts, t)
        if pad_forces is not None:
            f = max(float(pad_forces[0]), float(pad_forces[1]))
            self.grip.peak_force = max(self.grip.peak_force, f)
            alarm = self.cfg.qa_grip_force_N
            if alarm is None:
                alarm = 1.25 * self.cfg.grip_force_max_N
            if f > alarm:
                new.append({"t": round(float(t), 3), "kind": "grip_force",
                            "pad_force_N": round(f, 2)})
                if self.qa is not None:
                    self.qa.violations.append(new[-1])
        if self.on_violation:
            for v in new:
                self.on_violation(v)
        return new

    # -- HUD -------------------------------------------------------------
    def hud_lines(self, t=None, every_s: float = 0.5) -> list[str]:
        """Compact live status; rate-limited so it can be called every frame."""
        now = time.time() if t is None else float(t)
        if now - self._hud_t < every_s:
            return []
        self._hud_t = now
        i = self.last_info
        st = "GUARD ON " if self.enabled else "GUARD OFF"
        lines = []
        if i:
            flags = "".join(c for c, k in (("R", "rate_clipped"),
                                           ("L", "lead_clipped"),
                                           ("Y", "yaw_clipped"),
                                           ("P", "press_servo"),
                                           ("G", "over_limit"))
                            if i.get(k)) or "-"
            lines.append(
                f"[{st}] zone={i.get('zone','?'):<12s} "
                f"feat={i.get('feature_m', 0)*1000:6.2f}mm "
                f"vmax={i.get('speed_m_s', 0)*1000:6.1f}mm/s "
                f"lead<={i.get('lead_budget_m', i.get('lead_m', 0))*1000:6.2f}mm "
                f"F={i.get('contact_force_N', 0.0):6.1f}N "
                f"pad={i.get('pad_force_N', 0.0):5.2f}N clip={flags}")
        if self.qa is not None:
            r = self.qa.report()
            lines.append(
                f"          QA pen={r['pen_max_mm']:6.3f}mm "
                f"vel={r['vel_max_m_s']:5.2f}m/s "
                f"viol={r['violations']:4d} {r['by_kind']}")
            for v in self.qa.violations[-2:]:
                if abs(v["t"] - (t or 0)) < every_s:
                    lines.append(f"          !! {v}")
        return lines

    def summary(self) -> dict:
        out = {"guard_enabled": self.enabled,
               "clamps": dict(self.counts),
               "peak_pad_force_N": round(self.grip.peak_force, 3),
               "grip_over_limit_frames": self.grip.excess_frames}
        if self.qa is not None:
            out["live_qa"] = self.qa.report()
            out["violations"] = self.qa.violations
        return out


# ── self-test (no GPU, no scene) ────────────────────────────────────────


def _selftest():
    cfg = GuardCfg()
    fmap = FeatureMap(free_air_m=0.05)
    fmap.add_aabb("slot", (-0.01, -0.01, 0.0), (0.01, 0.01, 0.05),
                  feature_m=0.0021, pad=0.02)
    g = TeleopGuard(cfg, model=None, feature_map=fmap)

    # free air: a 100 mm jab is clamped to the free-air ceiling
    g.reset([0.30, 0.0, 0.10, 0.0, 0.04])
    out = g.filter([0.40, 0.0, 0.10, 0.0, 0.04], measured_tcp=[0.30, 0, 0.10])
    step_free = out[0] - 0.30
    assert step_free <= cfg.v_max_free / cfg.fps + 1e-9, step_free

    # tight zone: the same jab is clamped sub-mm
    g.reset([0.0, 0.0, 0.03, 0.0, 0.04])
    out = g.filter([0.10, 0.0, 0.03, 0.0, 0.04], measured_tcp=[0.0, 0, 0.03])
    step_tight = out[0]
    assert step_tight < 0.001, step_tight
    assert step_tight < step_free

    # lead limit: a command far ahead of a stalled TCP is pulled back
    g.reset([0.0, 0.0, 0.03, 0.0, 0.04])
    for _ in range(200):
        out = g.filter([0.10, 0.0, 0.03, 0.0, 0.04],
                       measured_tcp=[0.0, 0.0, 0.03])
    lead = out[0]
    assert lead <= cfg.lead_max_free + 1e-9, lead
    # the command advances one rate-limited step per frame and FREEZES on
    # the last step that still fits inside the lead budget; it must never be
    # dragged back onto the stalled measured TCP (that is the runaway that
    # walked task_nut_bolt_arm's arm into the floor)
    step_tight = cfg.feature_fraction * 0.0021
    assert cfg.lead_min - step_tight <= lead <= cfg.lead_min + 1e-9, lead

    # grip limiter: a hard squeeze against a 20 N pad reading opens up
    lim = GripForceLimiter(cfg)
    w = 0.002
    for _ in range(200):
        w_out, gi = lim.limit(0.002, 20.0, 20.0)
    assert w_out > 0.002, w_out
    assert lim.relief <= cfg.grip_relief_max + 1e-12

    # admittance filter: deadband kills sub-0.25 mm chatter entirely
    f = AdmittanceFilter(cfg)
    f.reset([0.0, 0.0, 0.0, 0.0, 0.04])
    for i in range(50):
        s = f([0.0001 * (-1) ** i, 0.0, 0.0, 0.0, 0.04])
    assert abs(s[0]) < 1e-12, s[0]

    print("teleop_guard selftest OK")
    print(f"  free-air step  {step_free*1000:.3f} mm/frame "
          f"({step_free*cfg.fps*1000:.0f} mm/s)")
    print(f"  2.1 mm slot    {step_tight*1000:.3f} mm/frame "
          f"({step_tight*cfg.fps*1000:.0f} mm/s)")
    print(f"  lead clamp     {lead*1000:.3f} mm")
    print(f"  grip relief    +{lim.relief*1000:.3f} mm at 20 N/pad "
          f"-> width {w_out*1000:.2f} mm")


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    _selftest()
