"""Excessive-force and integrity alarms for the teleoperation UIs.

One :class:`AlarmMonitor` evaluates a fixed set of CHANNELS every frame.
A numeric channel has a *warn* and a *critical* threshold with hysteresis:
the level only drops once the value falls below ``clear_fraction`` of the
threshold that raised it, and never inside ``hold_s`` of being raised, so
a force that chatters around a limit reads as one alarm rather than sixty
a second.  Boolean channels (solver divergence, workspace escape) are
critical while true.  Every RISE in level becomes an event
``{t, channel, level, value, detail}`` that the UIs list, that the session
report keeps, and that the web client beeps on.

The default numbers are the suite's own measured limits, not guesses:

* ``pad_L`` / ``pad_R`` warn at 6 N -- the top of the human-plausible grip
  window on the 7.3 mm chopstick (``GuardCfg.grip_force_max_N``); 12 N is
  where the pads have crushed past anything a scripted expert commands.
* ``hand`` warns at 15 N -- the guard's press ceiling
  (``GuardCfg.press_force_max_N``); fingertip presses through the arm are
  10-50 N/mm of command lead, so 40 N means millimetres of overdrive.
* ``penetration`` is the growth of a contact pair's overlap beyond its
  REST depth (several interfaces overlap at rest by design); 3 mm is the
  teleop gate's tunnelling threshold, 1 mm is a fair warning.
* ``speed`` mirrors ``RolloutQA.check_velocity`` (2.5 m/s frame-to-frame
  displacement = a flying object).
"""

from __future__ import annotations

from dataclasses import dataclass, field

LEVELS = ("ok", "warn", "critical")
_RANK = {lvl: i for i, lvl in enumerate(LEVELS)}


@dataclass
class Threshold:
    warn: float
    critical: float
    unit: str = "N"
    label: str = ""
    clear_fraction: float = 0.8
    hold_s: float = 0.25
    critical_frames: int = 4
    """Consecutive samples at or above ``critical`` before the level is
    raised to critical.  A position-controlled fingertip meeting a rigid
    part produces one-frame impact spikes (16 ms) that no operator can act
    on and that do no harm; a SUSTAINED excess is what the red banner
    means.  Warnings are immediate, and the spike still shows in the gauge,
    the sparkline and the peak."""

    def level_for(self, value: float) -> str:
        if value >= self.critical:
            return "critical"
        if value >= self.warn:
            return "warn"
        return "ok"


def default_thresholds() -> dict[str, Threshold]:
    return {
        "pad_L": Threshold(6.0, 12.0, "N", "left pad force"),
        "pad_R": Threshold(6.0, 12.0, "N", "right pad force"),
        "hand": Threshold(15.0, 40.0, "N", "arm / hand contact force"),
        "object": Threshold(15.0, 40.0, "N", "force the robot puts on one object"),
        "mech": Threshold(20.0, 60.0, "N", "object-object load above rest"),
        "penetration": Threshold(1.0, 3.0, "mm", "overlap growth over rest", critical_frames=2),
        "speed": Threshold(1.0, 2.5, "m/s", "fastest body", critical_frames=2),
        "track": Threshold(15.0, 40.0, "mm", "command lead over the measured TCP"),
    }


BOOL_CHANNELS = {
    "divergence": ("critical", "solver produced a non-finite state"),
    "envelope": ("critical", "a body left the workspace"),
    "ik": ("warn", "IK returned a non-finite solution"),
}


@dataclass
class AlarmCfg:
    thresholds: dict[str, Threshold] = field(default_factory=default_thresholds)
    max_events: int = 400

    def scale_pads(self, finger_effort_N: float | None) -> None:
        """Pad thresholds from the scene's own finger force clamp: the drive
        cannot legitimately deliver more than its ``joint_effort_limit``, so
        anything above it is an impact or a press through the arm.  Warn at
        half the clamp, critical at the clamp, never below the 6 / 12 N of
        the chopstick window."""
        if not finger_effort_N:
            return
        F = float(finger_effort_N)
        for k in ("pad_L", "pad_R"):
            thr = self.thresholds[k]
            thr.warn = max(6.0, 0.5 * F)
            thr.critical = max(12.0, F)
        # 'object' sums BOTH pads' contact on the pinched body, so a pinch at
        # the pad thresholds reads 2x there; keep it ordered after the pads
        obj = self.thresholds["object"]
        obj.warn = max(15.0, F)
        obj.critical = max(40.0, 2.0 * F)

    def allow_pinch(self, pinch_N: float) -> None:
        """A scene that starts with an object pinched at ``pinch_N`` per pad (the task's own working grip): the pads
        warn above 1.25x that pinch and go critical above 1.5x, so the grip the task needs is not an alarm."""
        P = float(pinch_N)
        for k in ("pad_L", "pad_R"):
            thr = self.thresholds[k]
            thr.warn = max(thr.warn, 1.25 * P)
            thr.critical = max(thr.critical, 1.5 * P)
        obj = self.thresholds["object"]
        obj.warn = max(obj.warn, 2.5 * P)
        obj.critical = max(obj.critical, 3.0 * P)

    def override(self, spec: str) -> None:
        """Apply ``name=warn:critical`` (e.g. ``pad_L=5:10``)."""
        name, _, vals = spec.partition("=")
        name = name.strip()
        if name not in self.thresholds:
            raise ValueError(f"unknown alarm channel {name!r}; "
                             f"one of {sorted(self.thresholds)}")
        w, _, c = vals.partition(":")
        thr = self.thresholds[name]
        if w.strip():
            thr.warn = float(w)
        if c.strip():
            thr.critical = float(c)
        if thr.critical < thr.warn:
            raise ValueError(f"{name}: critical ({thr.critical}) < warn ({thr.warn})")


class _Channel:
    def __init__(self, name: str, thr: Threshold):
        self.name = name
        self.thr = thr
        self.level = "ok"
        self.value = 0.0
        self.detail = ""
        self.peak = 0.0
        self.raised_t = -1e9
        self.frames_over = 0
        self.crit_run = 0

    def update(self, value: float, t: float, detail: str = "") -> str | None:
        """Return the new level if it ROSE, else None."""
        self.value = float(value)
        self.detail = detail
        self.peak = max(self.peak, self.value)
        wanted = self.thr.level_for(self.value)
        if wanted == "critical":
            self.crit_run += 1
            if self.crit_run < max(1, int(self.thr.critical_frames)) and self.level != "critical":
                wanted = "warn"          # not sustained yet
        else:
            self.crit_run = 0
        rose = None
        if _RANK[wanted] > _RANK[self.level]:
            self.level = wanted
            self.raised_t = t
            rose = wanted
        elif _RANK[wanted] < _RANK[self.level]:
            # hysteresis: drop only below clear_fraction of the threshold
            # that holds the current level, and only after hold_s
            hold_thr = (self.thr.critical if self.level == "critical"
                        else self.thr.warn)
            if (self.value < self.thr.clear_fraction * hold_thr
                    and t - self.raised_t >= self.thr.hold_s):
                self.level = wanted
        if self.level != "ok":
            self.frames_over += 1
        return rose

    def reset(self):
        self.level = "ok"
        self.value = 0.0
        self.detail = ""
        self.peak = 0.0
        self.raised_t = -1e9
        self.frames_over = 0
        self.crit_run = 0


class _BoolChannel:
    def __init__(self, name: str, level: str, label: str):
        self.name = name
        self.on_level = level
        self.label = label
        self.level = "ok"
        self.value = False
        self.detail = ""
        self.frames_over = 0

    def update(self, value: bool, t: float, detail: str = "") -> str | None:
        rose = None
        value = bool(value)
        if value and self.level == "ok":
            rose = self.on_level
        self.level = self.on_level if value else "ok"
        self.value = value
        self.detail = detail
        if value:
            self.frames_over += 1
        return rose

    def reset(self):
        self.level = "ok"
        self.value = False
        self.detail = ""
        self.frames_over = 0


class AlarmMonitor:
    """Evaluate all channels for one frame; keep the event log."""

    def __init__(self, cfg: AlarmCfg | None = None):
        self.cfg = cfg or AlarmCfg()
        self.channels: dict[str, _Channel] = {
            k: _Channel(k, thr) for k, thr in self.cfg.thresholds.items()}
        self.flags: dict[str, _BoolChannel] = {
            k: _BoolChannel(k, lvl, lbl) for k, (lvl, lbl) in BOOL_CHANNELS.items()}
        self.events: list[dict] = []
        self.counts = {"warn": 0, "critical": 0}
        self.level = "ok"
        self.t = 0.0

    # -- per frame ------------------------------------------------------
    def update(self, t: float, readings: dict, flags: dict | None = None) -> list[dict]:
        """``readings[name] = value`` or ``(value, detail)``; returns new events."""
        self.t = float(t)
        new = []
        for name, r in readings.items():
            ch = self.channels.get(name)
            if ch is None:
                continue
            value, detail = (r if isinstance(r, tuple) else (r, ""))
            rose = ch.update(value, t, detail)
            if rose:
                new.append(self._event(t, name, rose, value, detail, ch.thr.unit))
        for name, r in (flags or {}).items():
            fl = self.flags.get(name)
            if fl is None:
                continue
            value, detail = (r if isinstance(r, tuple) else (r, ""))
            rose = fl.update(value, t, detail)
            if rose:
                new.append(self._event(t, name, rose, 1.0, detail or fl.label, ""))
        worst = "ok"
        for ch in list(self.channels.values()) + list(self.flags.values()):
            if _RANK[ch.level] > _RANK[worst]:
                worst = ch.level
        self.level = worst
        return new

    def _event(self, t, channel, level, value, detail, unit) -> dict:
        ev = {"t": round(float(t), 3), "channel": channel, "level": level,
              "value": round(float(value), 3), "unit": unit, "detail": str(detail)}
        self.events.append(ev)
        if len(self.events) > self.cfg.max_events:
            del self.events[: len(self.events) - self.cfg.max_events]
        self.counts[level] += 1
        return ev

    # -- views ------------------------------------------------------------
    def snapshot(self) -> dict:
        chans = {}
        for k, ch in self.channels.items():
            chans[k] = {"value": round(ch.value, 3), "level": ch.level,
                        "warn": ch.thr.warn, "critical": ch.thr.critical,
                        "unit": ch.thr.unit, "label": ch.thr.label,
                        "detail": ch.detail, "peak": round(ch.peak, 3)}
        for k, fl in self.flags.items():
            chans[k] = {"value": bool(fl.value), "level": fl.level, "unit": "",
                        "label": fl.label, "detail": fl.detail}
        return {"level": self.level, "channels": chans,
                "counts": dict(self.counts),
                "recent": self.events[-8:]}

    def headline(self) -> str:
        """One line for a banner: the worst channel and its value."""
        worst, rank = None, 0
        for ch in list(self.channels.values()) + list(self.flags.values()):
            if _RANK[ch.level] > rank:
                worst, rank = ch, _RANK[ch.level]
        if worst is None:
            return "nominal"
        if isinstance(worst, _BoolChannel):
            return f"{worst.level.upper()}: {worst.label}" + (
                f" ({worst.detail})" if worst.detail else "")
        thr = worst.thr
        lim = thr.critical if worst.level == "critical" else thr.warn
        s = f"{worst.level.upper()}: {thr.label} {worst.value:.2f} {thr.unit} > {lim:g} {thr.unit}"
        if worst.detail:
            s += f" [{worst.detail}]"
        return s

    def reset(self):
        for ch in self.channels.values():
            ch.reset()
        for fl in self.flags.values():
            fl.reset()
        self.events.clear()
        self.counts = {"warn": 0, "critical": 0}
        self.level = "ok"

    def summary(self) -> dict:
        return {"counts": dict(self.counts),
                "peaks": {k: round(ch.peak, 3) for k, ch in self.channels.items()},
                "frames_over": {k: ch.frames_over for k, ch in
                                list(self.channels.items()) + list(self.flags.items())
                                if ch.frames_over},
                "events": list(self.events)}
