"""Operator input for the teleop UIs: keyboard, gamepad, web, scripted.

Every source produces the same :class:`InputState` -- five RATE axes in
[-1, 1] and a set of momentary buttons -- so the session, the guard and
the recorder never know which device is attached.  The axes live in the
OPERATOR FRAME (forward = away from the robot base toward the work, left,
up, yaw about world z, roll about the tool's own approach axis) and the
jaw axis is a closing/opening rate.

The mapping below is the single source of truth for the browser page too
(``web/index.html`` mirrors it) so a gamepad behaves identically on the
GL window and on the web page:

    keyboard   W/S forward/back   A/D left/right   R/F up/down
               Q/E yaw            C/V pitch        Z/X roll
               G/H close/open jaw
               T guard on/off     SPACE hold (e-stop while held)
               BACKSPACE reset    P pause          [ / ] speed down/up
               SHIFT fine (x0.25)
    spacemouse 6 DoF at once, see teleop/spacemouse.py
    gamepad    left stick  forward/back + left/right
               right stick up/down (vertical) + yaw (horizontal)
               hold R3     right stick vertical controls pitch instead of height
               LT / RT     close / open jaw (analog)
               D-pad       left/right roll, up/down fine z
               A guard     B hold    Y reset    X pause
               LB / RB     speed down / up
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

AXES = ("fwd", "left", "up", "yaw", "pitch", "roll", "jaw")
BUTTONS = ("guard", "hold", "reset", "pause", "speed_up", "speed_down", "fine")
DEADZONE = 0.12


@dataclass
class InputState:
    axes: np.ndarray = field(default_factory=lambda: np.zeros(len(AXES)))
    buttons: dict = field(default_factory=lambda: {b: False for b in BUTTONS})
    source: str = "none"
    stale: bool = False

    def clear(self):
        self.axes[:] = 0.0
        for b in self.buttons:
            self.buttons[b] = False

    def set_axis(self, name: str, value: float):
        v = float(value)
        self.axes[AXES.index(name)] = float(np.clip(v, -1.0, 1.0)) if np.isfinite(v) else 0.0

    def axis(self, name: str) -> float:
        return float(self.axes[AXES.index(name)])

    def as_dict(self) -> dict:
        return {"source": self.source, "stale": self.stale,
                "axes": {a: round(float(v), 3) for a, v in zip(AXES, self.axes)},
                "buttons": {b: bool(v) for b, v in self.buttons.items() if v}}

    @staticmethod
    def from_dict(d) -> "InputState":
        """Total for any JSON value: a bad axis or container leaves zeros."""
        if not isinstance(d, dict):
            return InputState(source="web")
        st = InputState(source=str(d.get("source", "web"))[:24])
        axes = d.get("axes")
        if isinstance(axes, dict):
            for a, v in axes.items():
                if a in AXES:
                    try:
                        st.set_axis(a, float(v))
                    except (TypeError, ValueError):
                        pass
        buttons = d.get("buttons")
        if isinstance(buttons, dict):
            for b, v in buttons.items():
                if b in st.buttons:
                    st.buttons[b] = bool(v)
        return st


def deadzone(v: float, dz: float = DEADZONE) -> float:
    v = float(v)
    if abs(v) < dz:
        return 0.0
    return float(np.sign(v) * (abs(v) - dz) / (1.0 - dz))


class Edge:
    """Rising-edge detector for toggle buttons held across frames."""

    def __init__(self):
        self.prev: dict[str, bool] = {}

    def rising(self, buttons: dict) -> set:
        out = set()
        for k, v in buttons.items():
            if v and not self.prev.get(k, False):
                out.add(k)
        self.prev = dict(buttons)
        return out


# ── keyboard ────────────────────────────────────────────────────────────
KEYMAP = {
    "fwd": ("w", "s"), "left": ("a", "d"), "up": ("r", "f"),
    "yaw": ("q", "e"), "pitch": ("c", "v"), "roll": ("z", "x"),
    "jaw": ("h", "g"),                      # H opens (+), G closes (-)
}
# ViewerGL.is_key_down resolves single letters / digits and a short table of
# names (space, backspace, shift = LEFT shift only); everything else must be
# given as the pyglet key constant, which equals the X11 keysym, so the
# values are written out here rather than imported (importing pyglet.window
# opens a shadow GL window and needs a display).
KEY_BUTTONS = {"guard": ("t",), "hold": ("space",), "reset": ("backspace",), "pause": ("p",),
               "speed_up": (0x05d,),            # pyglet.window.key.BRACKETRIGHT  ]
               "speed_down": (0x05b,),          # BRACKETLEFT  [
               "fine": (0xffe1, 0xffe2)}        # LSHIFT, RSHIFT


class KeyboardInput:
    """Polls a ``is_key_down(name_or_code) -> bool`` callable (the GL viewer's).

    ``capturing`` (optional) returns True while an imgui text field owns the
    keyboard, in which case the arm gets no input from the keys."""

    def __init__(self, is_key_down, capturing=None):
        self.is_key_down = is_key_down
        self.capturing = capturing
        self.state = InputState(source="keyboard")

    def _down(self, key) -> bool:
        try:
            return bool(self.is_key_down(key))
        except Exception:
            return False

    def poll(self) -> InputState:
        st = self.state
        st.clear()
        if self.capturing is not None and self.capturing():
            return st
        for axis, (pos, neg) in KEYMAP.items():
            v = (1.0 if self._down(pos) else 0.0) - (1.0 if self._down(neg) else 0.0)
            st.set_axis(axis, v)
        for b, keys in KEY_BUTTONS.items():
            st.buttons[b] = any(self._down(k) for k in keys)
        return st


# ── gamepad (standard layout) ───────────────────────────────────────────
def gamepad_to_state(g: dict, st: InputState | None = None) -> InputState:
    """Map a standard-layout gamepad reading to an InputState.

    ``g`` keys: leftx lefty rightx righty lefttrigger righttrigger (floats,
    sticks in [-1,1] with +y DOWN as every gamepad API reports, triggers
    in [0,1]) and booleans a b x y leftshoulder rightshoulder dpup dpdown
    dpleft dpright.  The browser page applies the very same table.
    """
    st = st or InputState(source="gamepad")
    st.clear()
    st.source = "gamepad"
    st.set_axis("fwd", -deadzone(g.get("lefty", 0.0)))
    st.set_axis("left", -deadzone(g.get("leftx", 0.0)))
    vertical = -deadzone(g.get("righty", 0.0))
    pitch_mode = bool(g.get("rightstick", False))
    up = 0.0 if pitch_mode else vertical
    if g.get("dpup"):
        up = 0.3
    elif g.get("dpdown"):
        up = -0.3
    st.set_axis("up", up)
    st.set_axis("pitch", vertical if pitch_mode else 0.0)
    st.set_axis("yaw", -deadzone(g.get("rightx", 0.0)))
    roll = 0.0
    if g.get("dpleft"):
        roll = 0.5
    elif g.get("dpright"):
        roll = -0.5
    st.set_axis("roll", roll)
    rt = float(np.clip(g.get("righttrigger", 0.0), 0.0, 1.0))
    lt = float(np.clip(g.get("lefttrigger", 0.0), 0.0, 1.0))
    st.set_axis("jaw", rt - lt)                      # RT opens, LT closes
    st.buttons["guard"] = bool(g.get("a"))
    st.buttons["hold"] = bool(g.get("b"))
    st.buttons["reset"] = bool(g.get("y"))
    st.buttons["pause"] = bool(g.get("x"))
    st.buttons["speed_down"] = bool(g.get("leftshoulder"))
    st.buttons["speed_up"] = bool(g.get("rightshoulder"))
    st.buttons["fine"] = False
    return st


class PygletGamepad:
    """A pyglet ``Controller`` (evdev on Linux, needs a display) as a source."""

    ATTRS = ("leftx", "lefty", "rightx", "righty", "lefttrigger", "righttrigger",
             "a", "b", "x", "y", "leftshoulder", "rightshoulder",
             "rightstick", "dpup", "dpdown", "dpleft", "dpright")

    def __init__(self):
        self.controller = None
        self.name = None
        self.state = InputState(source="gamepad")
        self.error = None
        try:
            import pyglet.input
            ctrls = pyglet.input.get_controllers()
            if ctrls:
                self.controller = ctrls[0]
                self.controller.open()
                self.name = getattr(self.controller, "name", "controller")
        except Exception as exc:                        # no display / no device
            self.error = f"{type(exc).__name__}: {exc}"

    @property
    def connected(self) -> bool:
        return self.controller is not None

    def poll(self) -> InputState | None:
        if self.controller is None:
            return None
        g = {}
        for a in self.ATTRS:
            v = getattr(self.controller, a, 0.0)
            g[a] = float(v) if not isinstance(v, bool) else v
        return gamepad_to_state(g, self.state)


# ── scripted operators (verification + headless demos) ──────────────────
@dataclass
class Segment:
    duration_s: float
    axes: dict = field(default_factory=dict)
    buttons: dict = field(default_factory=dict)
    note: str = ""
    stop_when: object = None
    """Optional ``fn(session) -> bool``: end the segment early (e.g. on contact)."""


def _touching(session, thr_N: float = 0.3) -> bool:
    L = getattr(session, "last", {}) or {}
    return max(L.get("pad_L", 0.0), L.get("pad_R", 0.0), L.get("hand", 0.0)) > thr_N


def _low(session) -> bool:
    return float(session.raw[2]) <= float(session.cfg.z_min) + 0.002


class ScriptedInput:
    """Piecewise-constant operator: segments with DURATIONS and optional
    stop conditions, so the same script adapts to a scene whose start pose
    is 40 cm above the work and one whose pads already straddle it."""

    def __init__(self, segments: list, source: str = "script"):
        self.segments = list(segments)
        self.state = InputState(source=source)
        self.i = 0
        self.seg_t0 = 0.0
        self.duration = float(sum(s.duration_s for s in self.segments))
        self._finished = False

    def poll(self, t: float, session=None) -> InputState:
        st = self.state
        st.clear()
        while self.i < len(self.segments):
            seg = self.segments[self.i]
            elapsed = t - self.seg_t0
            early = seg.stop_when is not None and session is not None and elapsed > 0.05 \
                and bool(seg.stop_when(session))
            if elapsed >= seg.duration_s or early:
                self.i += 1
                self.seg_t0 = t
                continue
            for a, v in seg.axes.items():
                st.set_axis(a, v)
            for b, v in seg.buttons.items():
                st.buttons[b] = bool(v)
            st.note = seg.note
            return st
        self._finished = True
        st.note = "done"
        return st

    def done(self, t: float | None = None) -> bool:
        return self._finished or (t is not None and t >= self.duration)


def script_by_name(name: str) -> ScriptedInput:
    """Canned operators.  Every axis is a RATE, so what a script does in a
    scene depends on the speed scale and the guard -- which is the point:
    the same abusive stream through the guard and without it is the A/B.

    ``gentle``   a careful operator: slow squeeze on what the pads straddle, hold, release.
    ``gentle_held``  the same for a part the task starts with in the pinch: squeeze, hold, no release.
    ``abusive``  the gate's profile in operator terms: full-speed plunge to
                 contact, crush, jab into the work, yank, shake.  Expected to
                 raise alarms; expected NOT to break the model with the guard on.
    ``demo``     a plausible session for recordings: approach to touch, grasp, lift and
                 carry until the contact force says the part is held back, set down, release.
    ``hover``    no input (settling / rendering checks).
    """
    S = Segment
    if name == "hover":
        return ScriptedInput([S(3.0, note="hover")])
    if name == "gentle":
        # a careful operator with the pads already where the task left them:
        # a slow squeeze (10 mm/s), a hold, a slow release.  No blind descent
        # (a position-controlled fingertip landing on a rigid part at any
        # useful speed spikes to tens of newtons at ~400 N/mm) and no pull on
        # whatever is held (a knob on a locked door is a wall).
        return ScriptedInput([
            S(1.0, note="settle"),
            S(3.0, {"jaw": -0.2}, note="close to touch", stop_when=lambda s: _touching(s, 2.0)),
            S(1.0, note="hold"),
            S(2.0, {"jaw": 0.3}, note="open"),
            S(0.5, note="rest"),
        ])
    if name == "gentle_held":
        # the careful operator for a task that starts with the part in the pinch: the same squeeze and hold, and no
        # release (a part let go above its seat falls wherever it lands, which says nothing about the guard)
        return ScriptedInput([
            S(1.0, note="settle"),
            S(3.0, {"jaw": -0.2}, note="close to touch", stop_when=lambda s: _touching(s, 2.0)),
            S(3.0, note="hold"),
            S(0.5, note="rest"),
        ])
    if name == "abusive":
        return ScriptedInput([
            S(0.6, note="settle"),
            S(6.0, {"up": -1.0}, note="plunge", stop_when=lambda s: _touching(s, 1.0) or _low(s)),
            S(1.6, {"up": -1.0}, note="press on"),
            S(1.6, {"jaw": -1.0}, note="crush"),
            S(2.4, {"fwd": 1.0, "jaw": -1.0}, note="jab in"),
            S(2.0, {"up": 1.0, "yaw": 1.0, "jaw": -1.0}, note="yank+yaw"),
            S(1.2, {"left": 1.0, "jaw": -1.0}, note="shake L"),
            S(1.2, {"left": -1.0, "jaw": -1.0}, note="shake R"),
            S(0.6, {"jaw": 1.0}, note="release"),
        ])
    if name == "demo":
        # a plausible teleop session for the recordings: approach until the
        # pads touch, back off a few mm, a slow grasp, lift and carry until
        # the contact force says the part is held back (a fixed knob, a
        # latched plug), a small wiggle, set it down, release, retreat
        def held_back(s):
            thr = s.alarms.cfg.thresholds
            L = s.last
            return (max(L.get("pad_L", 0.0), L.get("pad_R", 0.0)) > 0.8 * thr["pad_L"].warn
                    or L.get("hand", 0.0) > 0.8 * thr["hand"].warn
                    or L.get("mech", 0.0) > 0.8 * thr["mech"].warn)

        def touched(s):
            # the pads met the work, or the fingertips are about to meet the
            # floor (tips reach 9.7 mm below the TCP): stop before the hand
            # rides down onto whatever stands between the open fingers
            if _touching(s, 0.5) or _low(s) or float(s.raw[2]) <= 0.020:
                s._demo_z_touch = float(s.raw[2])
                return True
            return False

        def back_down(s):
            # set the part down where it was picked: 4 mm above the touch height
            z0 = getattr(s, "_demo_z_touch", None)
            return _low(s) or (z0 is not None and float(s.raw[2]) <= z0 + 0.004)
        return ScriptedInput([
            S(1.0, note="settle"),
            S(9.0, {"up": -0.2}, note="approach", stop_when=touched),
            S(0.15, {"up": 0.3}, note="back off"),
            S(3.0, {"jaw": -0.25}, note="grasp"),
            S(0.5, note="hold"),
            S(2.5, {"up": 0.25}, note="lift", stop_when=held_back),
            S(1.5, {"left": 0.3}, note="carry", stop_when=held_back),
            S(0.8, {"yaw": 0.6}, note="wiggle", stop_when=held_back),
            S(0.8, {"yaw": -0.6}, note="wiggle back", stop_when=held_back),
            S(1.5, {"left": -0.3}, note="carry back", stop_when=held_back),
            S(4.0, {"up": -0.25}, note="set down", stop_when=back_down),
            S(1.5, {"jaw": 0.4}, note="release"),
            S(1.2, {"up": 0.3}, note="retreat"),
            S(0.5, note="rest"),
        ])
    if name == "abusive_unguarded":
        s = script_by_name("abusive")
        s.segments[0].buttons = {"guard": True}       # toggles the guard OFF at t=0
        s.segments[0].note = "guard off"
        return s
    raise ValueError(f"unknown script {name!r}; one of hover, gentle, gentle_held, demo, abusive, "
                     "abusive_unguarded")
