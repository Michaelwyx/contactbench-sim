"""3Dconnexion SpaceMouse for the teleop UI.

A SpaceMouse is a 6-DoF force/displacement puck: you push, pull, twist and
tilt the cap and it reports how far it is deflected on six axes.  The
deflection is read as a RATE command (the harder you push, the faster the
TCP moves), which is how the session's other input devices work too.

TWO BACKENDS, both producing the same canonical reading:

  spnav  the ``spacenavd`` daemon through ``libspnav`` (ctypes, no pip
         install).  PREFERRED ON LINUX: if ``spacenavd`` is running it has
         already opened the device, and a second reader on the raw HID node
         gets nothing.  ``systemctl status spacenavd`` to check.
  hid    the raw HID reports through ``libhidapi`` (ctypes) or the ``hid``
         module.  For machines with no daemon.  Needs read access to the
         ``/dev/hidraw*`` node (the ``plugdev`` group, or a udev rule).

THE CANONICAL DEVICE FRAME is the one the operator sees, sitting at the
desk with the cap in front of them and the logo facing them:

    +X   cap pushed RIGHT
    +Y   cap pushed FORWARD, away from the operator
    +Z   cap pulled UP
    +RX  right-handed about +X: the far edge of the cap lifts (nose up)
    +RY  right-handed about +Y: the cap rolls clockwise, seen from behind
    +RZ  right-handed about +Z: the cap twists counter-clockwise, seen
         from above

It is right-handed (X x Y = Z).  Each backend converts its own frame into
this one, so the robot mapping below is written once.

    spnav reports in the X11/OpenGL frame (x right, y UP, z TOWARD the
    operator), so X=x, Y=-z, Z=y and, because that change of basis is a
    proper rotation, the rotation components go the same way:
    RX=rx, RY=-rz, RZ=ry.

THE ROBOT MAPPING (`to_input_state`) is deliberately expressed in the
OPERATOR frame the session already defines -- fwd is away from the robot
base toward the work, left, up -- so it does not depend on where a scene
put its robot:

    cap right    -> TCP right         (the session's `left` axis, negated)
    cap forward  -> TCP away from the operator, into the work
    cap up       -> TCP up
    cap twist    -> TCP yaw about world +z
    cap nose-up  -> the tool's nose pitches UP
    cap roll     -> the tool rolls about its own approach axis

Yaw is unambiguous -- the device twist and the command's yaw are both
right-handed about "up" -- and so is roll about the tool's own approach
axis.  PITCH is the one that cannot be written down as a constant here:
the command applies it in the TOOL frame (so that it still means "tilt"
after a yaw), but which tool axis that should be, and with which sign,
depends on the scene's grasp orientation and on where its robot stands.
`TeleopSession._pitch_axis` resolves both from the geometry, under one
rule that reads correctly for a top-down pick and a horizontal door reach
alike: positive pitch RAISES the approach direction.  Every axis can
still be flipped by hand with `invert`.

    force-bench teleop-ui --spacemouse-test        # what each motion does
    force-bench teleop-ui --scene plug_in_usb_2 --input spacemouse
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
import time
from dataclasses import dataclass, field

import numpy as np

AXES6 = ("x", "y", "z", "rx", "ry", "rz")

# 3Dconnexion USB vendor / product ids, for the HID backend and for
# reporting which device was found.  0x046d is Logitech, who owned
# 3Dconnexion when the older pucks shipped.
VENDORS = (0x046D, 0x256F)
PRODUCTS = {
    0xC603: "SpaceMouse Plus", 0xC605: "CadMan", 0xC606: "SpaceMouse Classic",
    0xC621: "SpaceBall 5000", 0xC623: "SpaceTraveler", 0xC625: "SpacePilot",
    0xC626: "SpaceNavigator", 0xC627: "SpaceExplorer",
    0xC628: "SpaceNavigator for Notebooks", 0xC629: "SpacePilot Pro",
    0xC62B: "SpaceMouse Pro",
    0xC62E: "SpaceMouse Wireless (cabled)", 0xC62F: "SpaceMouse Wireless receiver",
    0xC631: "SpaceMouse Pro Wireless", 0xC632: "SpaceMouse Pro Wireless receiver",
    0xC633: "SpaceMouse Enterprise",
    0xC635: "SpaceMouse Compact", 0xC636: "SpaceMouse Module",
    0xC652: "Universal Receiver",
}


def usb_devices_present() -> list:
    """3Dconnexion devices on the USB bus, as (vid, pid, name).

    Needed because ``spnav_open()`` succeeds whenever the DAEMON is running,
    whether or not a puck is plugged in -- spacenavd waits for hot-plug --
    and libspnav 0.2.3 (Debian/Ubuntu's) has no `spnav_dev_name` to ask.
    Reading sysfs keeps this free of pyusb and of root.
    """
    found = []
    root = "/sys/bus/usb/devices"
    try:
        entries = sorted(os.listdir(root))
    except OSError:
        return found
    for name in entries:
        try:
            with open(os.path.join(root, name, "idVendor")) as f:
                vid = int(f.read().strip(), 16)
            with open(os.path.join(root, name, "idProduct")) as f:
                pid = int(f.read().strip(), 16)
        except (OSError, ValueError):
            continue
        if vid in VENDORS and pid in PRODUCTS:
            found.append((vid, pid, PRODUCTS[pid]))
    return found


@dataclass
class SpaceMouseCfg:
    """Everything about the device that an operator might have to change."""

    full_scale: float = 350.0
    """Raw counts at full deflection.  The classic pucks saturate at 350;
    some report up to 512.  Too small just means the axis reaches 1.0
    early, which the deadzone/expo curve below then flattens."""
    deadzone: float = 0.06
    """Fraction of full scale ignored around centre.  A SpaceMouse rests
    at zero but drifts a count or two, and an un-deadzoned drift is a
    robot that creeps while nobody is touching it."""
    expo: float = 2.0
    """Response curve: out = sign(u) * |u|**expo after the deadzone.  2.0
    is the usual SpaceMouse feel -- a quarter of the deflection is a
    sixteenth of the speed, so fine work near contact is possible without
    dropping the speed scale.  1.0 is linear."""
    hold_timeout_s: float = 0.25
    """Zero the axes if no motion report arrives for this long.  The
    daemon sends a zero event when the cap is released, but a DROPPED zero
    would leave the robot driving itself, so the reading also expires."""
    invert: tuple = ()
    """Canonical axis names to flip, e.g. ``("ry", "rz")``."""
    trans_gain: float = 1.0
    rot_gain: float = 1.0
    jaw_gain: float = 1.0
    button_close: int = 0        # hold to close the jaw
    button_open: int = 1         # hold to open it
    button_guard: int | None = None
    button_reset: int | None = None
    button_hold: int | None = None

    def parse_invert(self, spec: str) -> None:
        """``--spacemouse-invert y,rz`` -> cfg.invert."""
        names = tuple(n.strip().lower() for n in spec.split(",") if n.strip())
        bad = [n for n in names if n not in AXES6]
        if bad:
            raise ValueError(f"unknown SpaceMouse axis {bad}; one of {AXES6}")
        self.invert = names


@dataclass
class Reading:
    """One frame of the device, in the canonical frame, each axis in [-1, 1]."""

    axes: dict = field(default_factory=lambda: dict.fromkeys(AXES6, 0.0))
    buttons: frozenset = frozenset()
    t: float = 0.0
    stale: bool = True

    def vec(self) -> np.ndarray:
        return np.array([self.axes[a] for a in AXES6], dtype=float)


# ── backends ────────────────────────────────────────────────────────────
class SpaceMouseError(RuntimeError):
    pass


class _SpnavBackend:
    """``libspnav`` -> the ``spacenavd`` daemon."""

    name = "spnav"

    class _Motion(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("x", ctypes.c_int), ("y", ctypes.c_int),
                    ("z", ctypes.c_int), ("rx", ctypes.c_int), ("ry", ctypes.c_int),
                    ("rz", ctypes.c_int), ("period", ctypes.c_uint),
                    ("data", ctypes.POINTER(ctypes.c_int))]

    class _Button(ctypes.Structure):
        _fields_ = [("type", ctypes.c_int), ("press", ctypes.c_int), ("bnum", ctypes.c_int)]

    MOTION, BUTTON = 1, 2

    def __init__(self):
        path = ctypes.util.find_library("spnav") or "libspnav.so.0"
        try:
            self.lib = ctypes.CDLL(path)
        except OSError as exc:
            raise SpaceMouseError(f"libspnav not loadable ({exc}); install libspnav0") from exc

        class _Event(ctypes.Union):
            _fields_ = [("type", ctypes.c_int), ("motion", _SpnavBackend._Motion),
                        ("button", _SpnavBackend._Button)]

        self._Event = _Event
        self.lib.spnav_open.restype = ctypes.c_int
        self.lib.spnav_close.restype = ctypes.c_int
        self.lib.spnav_poll_event.argtypes = [ctypes.POINTER(_Event)]
        self.lib.spnav_poll_event.restype = ctypes.c_int
        if self.lib.spnav_open() == -1:
            raise SpaceMouseError(
                "spnav_open() failed: the spacenavd daemon is not reachable. "
                "`systemctl status spacenavd` (start it with `sudo systemctl "
                "start spacenavd`), or use --spacemouse-backend hid.")
        present = usb_devices_present()
        self.device = ("spacenavd -> " + present[0][2]) if present else \
            "spacenavd (no 3Dconnexion device on the USB bus yet)"
        self._buttons: set[int] = set()

    def poll(self) -> tuple[dict | None, frozenset]:
        """Drain the queue; return the newest motion (or None) and the buttons."""
        ev = self._Event()
        latest = None
        while self.lib.spnav_poll_event(ctypes.byref(ev)) != 0:
            if ev.type == self.MOTION:
                m = ev.motion
                # spnav is x right, y UP, z TOWARD the operator
                latest = {"x": float(m.x), "y": float(-m.z), "z": float(m.y),
                          "rx": float(m.rx), "ry": float(-m.rz), "rz": float(m.ry)}
            elif ev.type == self.BUTTON:
                b = ev.button
                (self._buttons.add if b.press else self._buttons.discard)(int(b.bnum))
        return latest, frozenset(self._buttons)

    def close(self):
        try:
            self.lib.spnav_close()
        except Exception:
            pass


class _HidBackend:
    """Raw HID reports, for machines with no spacenavd."""

    name = "hid"

    def __init__(self):
        self.dev = None
        self.device = "?"
        try:
            import hid  # type: ignore
        except Exception as exc:
            raise SpaceMouseError(
                "the `hid` module is not installed (pip install hidapi), and "
                "the raw-HID backend needs it; on Linux prefer "
                "--spacemouse-backend spnav with the spacenavd daemon") from exc
        found = None
        for info in hid.enumerate():
            if info.get("vendor_id") in VENDORS and info.get("product_id") in PRODUCTS:
                found = info
                break
        if found is None:
            raise SpaceMouseError(
                "no 3Dconnexion device on the HID bus (checked vendors "
                f"{[hex(v) for v in VENDORS]}). Plugged in? `lsusb | grep -i 3dconnexion`")
        self.dev = hid.device()
        self.dev.open_path(found["path"])
        self.dev.set_nonblocking(True)
        self.device = PRODUCTS.get(found["product_id"], hex(found["product_id"]))
        self._axes = dict.fromkeys(AXES6, 0.0)
        self._buttons: set[int] = set()

    @staticmethod
    def _i16(lo: int, hi: int) -> float:
        v = (hi << 8) | lo
        return float(v - 65536 if v >= 32768 else v)

    def poll(self) -> tuple[dict | None, frozenset]:
        got = False
        while True:
            data = self.dev.read(13)
            if not data:
                break
            got = True
            rid = data[0]
            # 3Dconnexion raw frame: +x right, +y toward the operator,
            # +z up-is-negative -- so y and z are negated into the
            # canonical frame (cap forward = +Y, cap up = +Z).
            if rid == 1 and len(data) >= 7:
                self._axes["x"] = self._i16(data[1], data[2])
                self._axes["y"] = -self._i16(data[3], data[4])
                self._axes["z"] = -self._i16(data[5], data[6])
                if len(data) >= 13:          # newer pucks pack all six here
                    self._axes["rx"] = self._i16(data[7], data[8])
                    self._axes["ry"] = -self._i16(data[9], data[10])
                    self._axes["rz"] = -self._i16(data[11], data[12])
            elif rid == 2 and len(data) >= 7:
                self._axes["rx"] = self._i16(data[1], data[2])
                self._axes["ry"] = -self._i16(data[3], data[4])
                self._axes["rz"] = -self._i16(data[5], data[6])
            elif rid == 3 and len(data) >= 3:
                mask = data[1] | (data[2] << 8)
                self._buttons = {i for i in range(16) if mask & (1 << i)}
        return (dict(self._axes) if got else None), frozenset(self._buttons)

    def close(self):
        try:
            if self.dev is not None:
                self.dev.close()
        except Exception:
            pass


def open_backend(prefer: str = "auto"):
    """Open the best available backend.  Raises SpaceMouseError with every
    reason when none works, because 'no SpaceMouse' is usually a daemon or
    a permissions problem rather than a missing device."""
    order = {"auto": ("spnav", "hid"), "spnav": ("spnav",), "hid": ("hid",)}[prefer]
    errors = []
    for name in order:
        try:
            return (_SpnavBackend if name == "spnav" else _HidBackend)()
        except SpaceMouseError as exc:
            errors.append(f"{name}: {exc}")
    raise SpaceMouseError("no SpaceMouse backend available.\n  " + "\n  ".join(errors))


# ── the device ──────────────────────────────────────────────────────────
class SpaceMouse:
    """Canonical readings from whichever backend is available."""

    def __init__(self, cfg: SpaceMouseCfg | None = None, prefer: str = "auto"):
        self.cfg = cfg or SpaceMouseCfg()
        self.backend = open_backend(prefer)
        self.reading = Reading()
        self._raw = dict.fromkeys(AXES6, 0.0)
        self._last_motion = -1e9

    @property
    def name(self) -> str:
        return f"{self.backend.device} via {self.backend.name}"

    def _curve(self, raw: float) -> float:
        c = self.cfg
        u = float(np.clip(raw / max(c.full_scale, 1e-9), -1.0, 1.0))
        d = c.deadzone
        if abs(u) <= d:
            return 0.0
        u = (abs(u) - d) / max(1.0 - d, 1e-9)
        return float(np.sign(raw) * u ** max(c.expo, 1e-3))

    def poll(self, now: float | None = None) -> Reading:
        now = time.monotonic() if now is None else now
        motion, buttons = self.backend.poll()
        if motion is not None:
            self._raw = motion
            self._last_motion = now
        stale = (now - self._last_motion) > self.cfg.hold_timeout_s
        if stale:
            self._raw = dict.fromkeys(AXES6, 0.0)
        axes = {}
        for a in AXES6:
            v = self._curve(self._raw[a])
            axes[a] = (-v if a in self.cfg.invert else v) + 0.0   # no signed zero
        self.reading = Reading(axes=axes, buttons=buttons, t=now, stale=stale)
        return self.reading

    def raw(self) -> dict:
        return dict(self._raw)

    def close(self):
        self.backend.close()


# ── the robot mapping ───────────────────────────────────────────────────
#: canonical device axis -> (InputState axis, sign).  The session puts the
#: scene-resolved sign on `pitch` when it builds the orientation, so this
#: table stays a plain statement about the DEVICE.
ROBOT_MAP = {
    "x": ("left", -1.0),      # cap right -> TCP right, and `left` is the axis
    "y": ("fwd", +1.0),       # cap forward -> into the work
    "z": ("up", +1.0),        # cap up -> TCP up
    "rz": ("yaw", +1.0),      # cap twist -> yaw about world +z, both right-handed
    "rx": ("pitch", +1.0),    # cap nose-up -> tool tilts back (sign resolved per scene)
    "ry": ("roll", +1.0),     # cap roll -> tool rolls about its approach axis
}


def to_input_state(reading: Reading, state, cfg: SpaceMouseCfg,
                   pitch_sign: float = 1.0) -> None:
    """Write a canonical reading into an InputState, in place."""
    for dev, (axis, sign) in ROBOT_MAP.items():
        gain = cfg.rot_gain if dev.startswith("r") else cfg.trans_gain
        if axis == "pitch":
            sign = sign * pitch_sign
        state.set_axis(axis, reading.axes[dev] * sign * gain)
    jaw = 0.0
    if cfg.button_close in reading.buttons:
        jaw -= 1.0
    if cfg.button_open in reading.buttons:
        jaw += 1.0
    state.set_axis("jaw", jaw * cfg.jaw_gain)
    for name, b in (("guard", cfg.button_guard), ("reset", cfg.button_reset),
                    ("hold", cfg.button_hold)):
        if b is not None and name in state.buttons:
            state.buttons[name] = b in reading.buttons


def describe_mapping(pitch_sign: float = 1.0) -> str:
    rows = [("push the cap right", "TCP moves right"),
            ("push the cap forward (away)", "TCP moves into the work"),
            ("pull the cap up", "TCP moves up"),
            ("twist the cap", "TCP yaws about vertical"),
            ("tilt the cap nose-up", "the tool's nose pitches UP -- off the vertical and "
                                      "back toward you on a top-down grasp, above the "
                                      "horizon on a horizontal reach"),
            ("roll the cap", "tool rolls about its approach axis"),
            ("button 1 / button 2", "close / open the jaw")]
    w = max(len(a) for a, _ in rows)
    return "\n".join(f"  {a:<{w}s}  ->  {b}" for a, b in rows)


# ── the axis check ──────────────────────────────────────────────────────
def axis_test(seconds: float = 0.0, hz: float = 10.0, prefer: str = "auto",
              cfg: SpaceMouseCfg | None = None) -> int:
    """Print raw counts, the canonical frame and the robot axes, live.

    This is the 10-second way to confirm the mapping on YOUR puck, which
    matters because the sign conventions differ between 3Dconnexion models
    and between the daemon and the raw HID path.  Anything that comes out
    backwards is one `--spacemouse-invert <axis>` away from fixed.
    """
    cfg = cfg or SpaceMouseCfg()
    try:
        sm = SpaceMouse(cfg, prefer=prefer)
    except SpaceMouseError as exc:
        print(f"[spacemouse] {exc}")
        return 1
    from force_bench.teleop.inputs import InputState
    present = usb_devices_present()
    print(f"[spacemouse] {sm.name}")
    if present:
        print("[spacemouse] on the USB bus: " +
              ", ".join(f"{n} ({v:#06x}:{p:#06x})" for v, p, n in present))
    else:
        print("[spacemouse] NO 3Dconnexion device is on the USB bus -- plug the puck in. "
              "(spacenavd accepts clients and waits for hot-plug, so the connection above "
              "succeeds either way.)")
    print(f"[spacemouse] full scale {cfg.full_scale:g} counts, deadzone {cfg.deadzone:.0%}, "
          f"expo {cfg.expo:g}, inverted {list(cfg.invert) or 'none'}")
    print(describe_mapping())
    print("\nPush the cap one way at a time; Ctrl-C to stop.")
    print(f"{'raw x/y/z':>26s} {'raw rx/ry/rz':>26s} | "
          f"{'fwd':>6s}{'left':>7s}{'up':>7s}{'yaw':>7s}{'pitch':>7s}{'roll':>7s}{'jaw':>6s}  buttons")
    st = InputState(source="spacemouse")
    tty = bool(getattr(__import__("sys").stdout, "isatty", lambda: False)())
    t_end = time.monotonic() + seconds if seconds else None
    t0 = time.monotonic()
    seen_motion = False
    try:
        while t_end is None or time.monotonic() < t_end:
            r = sm.poll()
            if any(abs(v) > 0 for v in sm.raw().values()):
                seen_motion = True
            raw = sm.raw()
            to_input_state(r, st, cfg)
            rt = " ".join(f"{raw[a]:7.0f}" for a in ("x", "y", "z"))
            rr = " ".join(f"{raw[a]:7.0f}" for a in ("rx", "ry", "rz"))
            vals = "".join(f"{st.axis(a) + 0.0:7.2f}" for a in
                           ("fwd", "left", "up", "yaw", "pitch", "roll"))
            line = (f"{rt:>26s} {rr:>26s} | {vals}{st.axis('jaw') + 0.0:6.1f}  "
                    f"{str(sorted(r.buttons)) if r.buttons else '-':<12}"
                    f"{'  (stale)' if r.stale else '         '}")
            # one rewritten line on a terminal; one line per sample when the
            # output is piped into a log, where \r would run it all together
            if tty:
                print("\r" + line, end="", flush=True)
            else:
                print(line, flush=True)
            time.sleep(1.0 / max(hz, 1e-3))
    except KeyboardInterrupt:
        pass
    finally:
        sm.close()
        print()
    if not seen_motion and time.monotonic() - t0 > 3.0:
        print("[spacemouse] no motion was reported at all.  Check, in order: the puck is "
              "plugged in (`lsusb | grep -i 3dconnexion`); spacenavd is running "
              "(`systemctl status spacenavd`); and if you are NOT using the daemon, that "
              "you can read /dev/hidraw* (the plugdev group) and are passing "
              "--spacemouse-backend hid.")
    return 0


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="3Dconnexion SpaceMouse axis check")
    ap.add_argument("--backend", default="auto", choices=["auto", "spnav", "hid"])
    ap.add_argument("--seconds", type=float, default=0.0)
    ap.add_argument("--invert", default="")
    ap.add_argument("--expo", type=float, default=None)
    ap.add_argument("--full-scale", type=float, default=None)
    a = ap.parse_args()
    c = SpaceMouseCfg()
    if a.invert:
        c.parse_invert(a.invert)
    if a.expo is not None:
        c.expo = a.expo
    if a.full_scale is not None:
        c.full_scale = a.full_scale
    raise SystemExit(axis_test(a.seconds, prefer=a.backend, cfg=c))
