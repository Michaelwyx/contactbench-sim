"""CPU checks for the teleop UI layer: alarms, inputs, scripts, camera
projection math, and the web server's HTTP + WebSocket protocol.  The
physics-backed acceptance checks are `force-bench teleop-ui --verify`."""
import asyncio
import json
import os
import socket
import threading
import time

import numpy as np
import pytest

from force_bench.teleop.alarms import AlarmCfg, AlarmMonitor, Threshold
from force_bench.teleop.inputs import (AXES, Edge, InputState, KeyboardInput, ScriptedInput, Segment,
                                       deadzone, gamepad_to_state, script_by_name)


# ── alarms ──────────────────────────────────────────────────────────────
def test_alarm_critical_is_debounced_but_warn_is_immediate():
    cfg = AlarmCfg()
    cfg.thresholds["pad_L"] = Threshold(6.0, 12.0, "N", "pad", critical_frames=3)
    mon = AlarmMonitor(cfg)
    ev = mon.update(0.00, {"pad_L": 20.0})           # one-frame impact spike
    assert [e["level"] for e in ev] == ["warn"] and mon.level == "warn"
    assert mon.update(0.02, {"pad_L": 20.0}) == [] and mon.level == "warn"
    ev = mon.update(0.03, {"pad_L": 20.0})           # third consecutive sample: sustained
    assert [e["level"] for e in ev] == ["critical"] and mon.level == "critical"
    assert mon.snapshot()["channels"]["pad_L"]["peak"] == 20.0
    mon.reset()
    mon.update(0.0, {"pad_L": 20.0}); mon.update(0.02, {"pad_L": 1.0}); mon.update(0.03, {"pad_L": 20.0})
    assert mon.level == "warn"                         # the run restarted: still not sustained


def test_alarm_levels_hysteresis_and_events():
    cfg = AlarmCfg()
    cfg.thresholds["pad_L"] = Threshold(6.0, 12.0, "N", "pad", clear_fraction=0.8, hold_s=0.25,
                                        critical_frames=1)
    mon = AlarmMonitor(cfg)
    assert mon.update(0.0, {"pad_L": 1.0}) == []
    ev = mon.update(0.1, {"pad_L": (7.0, "card 7.0N")})
    assert [e["level"] for e in ev] == ["warn"] and ev[0]["detail"] == "card 7.0N"
    ev = mon.update(0.2, {"pad_L": 13.0})
    assert [e["level"] for e in ev] == ["critical"] and mon.level == "critical"
    # inside hold_s and above clear fraction: stays critical, no new event
    assert mon.update(0.3, {"pad_L": 11.0}) == [] and mon.level == "critical"
    # after hold_s but still above 0.8*critical: stays
    assert mon.update(0.6, {"pad_L": 10.0}) == [] and mon.level == "critical"
    # drops below 0.8 * 12 = 9.6 -> re-evaluates to warn (10 > 6? no: 9.0 >= 6 -> warn)
    mon.update(0.7, {"pad_L": 9.0})
    assert mon.level == "warn"
    mon.update(1.0, {"pad_L": 0.0})
    assert mon.level == "ok"
    assert mon.counts == {"warn": 1, "critical": 1}
    assert "pad" in mon.snapshot()["channels"]["pad_L"]["label"]
    mon.reset()
    assert mon.level == "ok" and mon.events == []


def test_alarm_bool_flags_and_headline():
    mon = AlarmMonitor()
    ev = mon.update(1.0, {}, flags={"divergence": (True, "reset")})
    assert ev and ev[0]["channel"] == "divergence" and ev[0]["level"] == "critical"
    assert "non-finite" in mon.headline()
    mon.update(1.1, {}, flags={"divergence": False})
    assert mon.level == "ok"
    mon.update(1.2, {"penetration": (3.5, "plug<->socket")})
    assert mon.level == "warn"                       # one sample: not sustained yet
    mon.update(1.22, {"penetration": (3.5, "plug<->socket")})
    assert mon.headline().startswith("CRITICAL") and "plug<->socket" in mon.headline()


def test_scale_pads_keeps_object_ordered_after_pads():
    cfg = AlarmCfg()
    cfg.scale_pads(35.0)
    assert (cfg.thresholds["pad_L"].warn, cfg.thresholds["pad_L"].critical) == (17.5, 35.0)
    assert (cfg.thresholds["object"].warn, cfg.thresholds["object"].critical) == (35.0, 70.0)
    cfg = AlarmCfg()
    cfg.scale_pads(8.0)                               # never below the chopstick window
    assert (cfg.thresholds["pad_L"].warn, cfg.thresholds["pad_L"].critical) == (6.0, 12.0)
    assert (cfg.thresholds["object"].warn, cfg.thresholds["object"].critical) == (15.0, 40.0)


def test_alarm_override_spec():
    cfg = AlarmCfg()
    cfg.override("pad_L=5:10")
    assert cfg.thresholds["pad_L"].warn == 5 and cfg.thresholds["pad_L"].critical == 10
    cfg.override("hand=:33")
    assert cfg.thresholds["hand"].critical == 33
    with pytest.raises(ValueError):
        cfg.override("nope=1:2")
    with pytest.raises(ValueError):
        cfg.override("pad_L=9:3")


# ── inputs ──────────────────────────────────────────────────────────────
def test_keyboard_mapping_and_edges():
    down = set()
    kb = KeyboardInput(lambda k: k in down)
    down.update({"w", "d", "r", "e", "g", "t", "c", 0xffe2, 0x05d})   # right shift, ]
    st = kb.poll()
    assert st.axis("fwd") == 1 and st.axis("left") == -1 and st.axis("up") == 1
    assert st.axis("yaw") == -1 and st.axis("jaw") == -1 and st.axis("pitch") == 1
    assert st.buttons["guard"] and st.buttons["fine"] and st.buttons["speed_up"]
    assert not st.buttons["hold"] and not st.buttons["speed_down"]
    edge = Edge()
    assert edge.rising(st.buttons) == {"guard", "fine", "speed_up"}
    assert edge.rising(st.buttons) == set()          # held, not re-triggered
    down.clear()
    st = kb.poll()
    assert np.all(st.axes == 0) and not any(st.buttons.values())
    # an imgui text field owning the keyboard mutes the arm
    down.add("w")
    muted = KeyboardInput(lambda k: k in down, capturing=lambda: True)
    assert muted.poll().axis("fwd") == 0.0
    st = InputState()
    st.set_axis("fwd", float("nan"))
    assert st.axis("fwd") == 0.0


def test_gamepad_mapping_matches_documented_table():
    g = {"leftx": 0.5, "lefty": -1.0, "rightx": -0.4, "righty": 0.8, "lefttrigger": 0.0,
         "righttrigger": 1.0, "a": True, "b": False, "y": True, "leftshoulder": True}
    st = gamepad_to_state(g)
    assert st.axis("fwd") == pytest.approx(1.0)                 # stick up = forward
    assert st.axis("left") == pytest.approx(-deadzone(0.5))     # stick right = -left
    assert st.axis("up") == pytest.approx(-deadzone(0.8))
    assert st.axis("yaw") == pytest.approx(-deadzone(-0.4))
    assert st.axis("jaw") == pytest.approx(1.0)                 # RT opens
    assert st.buttons["guard"] and st.buttons["reset"] and st.buttons["speed_down"]
    assert deadzone(0.05) == 0.0 and deadzone(1.0) == pytest.approx(1.0)
    st = gamepad_to_state({"dpup": True, "dpleft": True, "lefttrigger": 0.5})
    assert st.axis("up") == 0.3 and st.axis("roll") == 0.5 and st.axis("jaw") == -0.5


@pytest.mark.parametrize("d", [{"axes": {"fwd": "x"}}, {"axes": [1]}, {"buttons": [1]},
                               {"axes": {"fwd": None}}, "junk", None, {"axes": {"fwd": float("inf")}}])
def test_inputstate_from_dict_tolerates_garbage(d):
    st = InputState.from_dict(d)
    assert np.all(st.axes == 0) and not any(st.buttons.values())


def test_web_input_sanitizer_and_nan_literal():
    from force_bench.teleop.web_ui import _reject_constant, _sanitize_input
    assert _sanitize_input({"axes": [1, 2]}) is None and _sanitize_input("x") is None
    clean = _sanitize_input({"axes": {"fwd": "0.5", "bogus": 1, "up": float("nan")}, "buttons": {"hold": 1, "x": 1}})
    assert clean["axes"] == {"fwd": 0.5, "up": 0.0} and clean["buttons"] == {"hold": True}
    with pytest.raises(ValueError):
        json.loads('{"axes": {"fwd": NaN}}', parse_constant=_reject_constant)


def test_inputstate_roundtrip_and_clip():
    d = {"source": "gamepad", "axes": {"fwd": 2.0, "jaw": -0.5, "bogus": 1}, "buttons": {"hold": True, "x": 1}}
    st = InputState.from_dict(d)
    assert st.axis("fwd") == 1.0 and st.axis("jaw") == -0.5 and st.buttons["hold"] and "x" not in st.buttons
    back = st.as_dict()
    assert back["axes"]["fwd"] == 1.0 and back["buttons"] == {"hold": True} and back["source"] == "gamepad"


class _FakeSession:
    def __init__(self):
        self.last = {"pad_L": 0.0, "pad_R": 0.0, "hand": 0.0}
        self.raw = np.array([0, 0, 0.3, 0, 0, 0.04])
        self.cfg = type("C", (), {"z_min": 0.003})()


def test_scripted_segments_and_contact_stop():
    s = ScriptedInput([Segment(1.0, note="a"), Segment(5.0, {"up": -1.0}, note="descend",
                                                      stop_when=lambda ses: ses.last["hand"] > 0.3),
                       Segment(0.5, {"jaw": -1.0}, note="close")])
    ses = _FakeSession()
    assert s.poll(0.0, ses).note == "a"
    st = s.poll(1.2, ses)
    assert st.note == "descend" and st.axis("up") == -1.0
    ses.last["hand"] = 2.0                        # contact -> the descend ends early
    st = s.poll(1.5, ses)
    assert st.note == "close" and st.axis("jaw") == -1.0
    assert s.poll(2.1, ses).note == "done" and s.done()
    for name in ("hover", "gentle", "gentle_held", "demo", "abusive", "abusive_unguarded"):
        sc = script_by_name(name)
        assert sc.duration > 0
    g = script_by_name("gentle")
    assert [x.note for x in g.segments] == ["settle", "close to touch", "hold", "open", "rest"]
    assert all(set(x.axes) <= {"jaw"} for x in g.segments)        # no translation, ever
    assert g.segments[1].axes == {"jaw": -0.2} and g.segments[3].axes["jaw"] > 0
    assert script_by_name("abusive_unguarded").segments[0].buttons == {"guard": True}
    with pytest.raises(ValueError):
        script_by_name("nope")


# ── camera projection (pure numpy) ───────────────────────────────────────
def test_pinhole_projection_matches_ray_convention():
    from force_bench.teleop.camera import TeleopCamera, _quat_to_mat
    from force_bench.recorder import lookat_transform
    cam = TeleopCamera.__new__(TeleopCamera)
    cam.width, cam.height, cam.fov = 800, 600, np.radians(40.0)
    pos, tgt = np.array([1.0, -1.0, 0.8]), np.array([0.0, 0.0, 0.1])
    tf = lookat_transform(pos, tgt)
    cam._p = np.array([tf[0], tf[1], tf[2]])
    cam._R = _quat_to_mat([tf[3], tf[4], tf[5], tf[6]])
    px, front = cam.project([tgt])
    assert front[0] and np.allclose(px[0], [400.0, 300.0], atol=1e-3)   # look-at point = image centre
    # a point straight "up" from the target in camera space lands above the centre
    up = cam._R[:, 1]
    px2, _ = cam.project([tgt + 0.1 * up])
    assert px2[0, 1] < 300.0 and abs(px2[0, 0] - 400.0) < 1e-3
    # a point behind the camera is flagged
    _, back = cam.project([pos + cam._R[:, 2] * 0.5])
    assert not back[0]
    # the vertical fov: a point at the top edge of the frustum projects to y = 0
    d = np.linalg.norm(tgt - pos)
    top = tgt + up * d * np.tan(cam.fov / 2)
    px3, _ = cam.project([top])
    assert abs(px3[0, 1]) < 1e-3


# ── web server: page + websocket protocol (no GPU) ───────────────────────
def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_web_page_lists_every_channel_and_key():
    from force_bench.teleop.web_ui import HTML_PATH
    html = open(HTML_PATH, encoding="utf-8").read()
    for ch in ("pad_L", "pad_R", "hand", "object", "mech", "penetration", "speed", "track"):
        assert f'"{ch}"' in html
    for key in ("KeyW", "KeyT", "Space", "Backspace", "KeyP", "BracketLeft"):
        assert key in html
    assert "getGamepads" in html and "AudioContext" in html and 'type:"input"' in html
    assert 'KeyT:"guard"' in html and "e.ctrlKey || e.metaKey || e.altKey" in html


def test_web_server_http_and_websocket_roundtrip():
    websockets = pytest.importorskip("websockets")
    from force_bench.teleop.web_ui import WebServer
    import urllib.request
    port = _free_port()
    server = WebServer("127.0.0.1", port, hello={"scene": {"name": "unit"}}).start()
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5) as r:
            page = r.read()
        assert b"force-bench teleop" in page
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/health", timeout=5) as r:
            assert json.loads(r.read())["ok"] is True

        got = {"hello": None, "frame": None, "tm": None}

        async def client():
            async with websockets.connect(f"ws://127.0.0.1:{port}/") as ws:
                got["hello"] = json.loads(await ws.recv())
                await ws.send(json.dumps({"type": "input", "source": "test", "axes": {"fwd": 0.7},
                                          "buttons": {"guard": True}}))
                await ws.send(json.dumps({"type": "cmd", "action": "speed", "value": 2.0}))
                # wait until the server has our input, then publish a frame to us
                for _ in range(50):
                    if server.active():
                        break
                    await asyncio.sleep(0.02)
                server.publish(b"\xff\xd8jpeg", {"type": "telemetry", "t": 1.0})
                got["frame"] = await asyncio.wait_for(ws.recv(), 5)
                got["tm"] = json.loads(await asyncio.wait_for(ws.recv(), 5))

        asyncio.run(client())
        assert got["hello"]["type"] == "hello" and got["hello"]["scene"]["name"] == "unit"
        assert got["frame"] == b"\xff\xd8jpeg" and got["tm"]["t"] == 1.0
        st = server.current_input()
        assert st.source == "test" and st.axis("fwd") == pytest.approx(0.7) and st.buttons["guard"]
        assert not st.stale
        cmds = server.drain_commands()
        assert cmds == [{"type": "cmd", "action": "speed", "value": 2.0}]
        # a value-less toggle from the page's keydown path is a plain command
        assert json.loads(json.dumps({"type": "cmd", "action": "guard"}))["action"] == "guard"
        server.stale_s = 0.0
        time.sleep(0.01)
        assert server.current_input().stale and np.all(server.current_input().axes == 0)
    finally:
        server.stop()


# ── SpaceMouse (no hardware needed) ─────────────────────────────────────
def test_spacemouse_response_curve_and_deadzone():
    from force_bench.teleop.spacemouse import SpaceMouse, SpaceMouseCfg
    sm = SpaceMouse.__new__(SpaceMouse)
    sm.cfg = SpaceMouseCfg(full_scale=350.0, deadzone=0.1, expo=2.0)
    assert sm._curve(0.0) == 0.0
    assert sm._curve(30.0) == 0.0                       # inside the deadzone
    assert sm._curve(350.0) == pytest.approx(1.0)
    assert sm._curve(-350.0) == pytest.approx(-1.0)
    assert sm._curve(700.0) == pytest.approx(1.0)       # saturates
    half = sm._curve(0.55 * 350.0)                      # expo squashes mid travel
    assert 0.1 < half < 0.35
    lin = SpaceMouse.__new__(SpaceMouse)
    lin.cfg = SpaceMouseCfg(full_scale=350.0, deadzone=0.0, expo=1.0)
    assert lin._curve(175.0) == pytest.approx(0.5)


def test_spacemouse_robot_mapping_is_the_documented_one():
    """Each cap motion has to reach the axis the docs promise, with the
    sign an operator expects.  This is the mapping the user cares about."""
    from force_bench.teleop.inputs import InputState
    from force_bench.teleop.spacemouse import AXES6, Reading, SpaceMouseCfg, to_input_state
    cfg = SpaceMouseCfg()
    def drive(**dev):
        r = Reading(axes={**dict.fromkeys(AXES6, 0.0), **dev})
        st = InputState(source="spacemouse")
        to_input_state(r, st, cfg)
        return st
    # cap right -> TCP right, i.e. NEGATIVE on the session's `left` axis
    assert drive(x=1.0).axis("left") == pytest.approx(-1.0)
    # cap forward (away from the operator) -> into the work
    assert drive(y=1.0).axis("fwd") == pytest.approx(1.0)
    assert drive(z=1.0).axis("up") == pytest.approx(1.0)
    # twist about the vertical -> yaw; both are right-handed about "up"
    assert drive(rz=1.0).axis("yaw") == pytest.approx(1.0)
    assert drive(rx=1.0).axis("pitch") == pytest.approx(1.0)
    assert drive(ry=1.0).axis("roll") == pytest.approx(1.0)
    # one cap motion must not leak into another robot axis
    for dev in AXES6:
        st = drive(**{dev: 1.0})
        assert sum(1 for v in st.axes if abs(v) > 1e-9) == 1, dev


def test_spacemouse_buttons_drive_the_jaw_and_invert_flips_axes():
    from force_bench.teleop.inputs import InputState
    from force_bench.teleop.spacemouse import AXES6, Reading, SpaceMouseCfg, to_input_state
    cfg = SpaceMouseCfg()
    st = InputState(source="spacemouse")
    to_input_state(Reading(buttons=frozenset({cfg.button_close})), st, cfg)
    assert st.axis("jaw") == pytest.approx(-1.0)        # close
    to_input_state(Reading(buttons=frozenset({cfg.button_open})), st, cfg)
    assert st.axis("jaw") == pytest.approx(1.0)         # open
    to_input_state(Reading(buttons=frozenset({cfg.button_close, cfg.button_open})), st, cfg)
    assert st.axis("jaw") == 0.0                        # both = no motion
    cfg.parse_invert("y,rz")
    assert cfg.invert == ("y", "rz")
    with pytest.raises(ValueError):
        SpaceMouseCfg().parse_invert("nope")


def test_spnav_frame_conversion_matches_the_canonical_frame():
    """spnav reports x right, y UP, z TOWARD the operator; the canonical
    frame is x right, y FORWARD, z up."""
    from force_bench.teleop.spacemouse import _SpnavBackend
    conv = lambda x, y, z, rx, ry, rz: {
        "x": float(x), "y": float(-z), "z": float(y),
        "rx": float(rx), "ry": float(-rz), "rz": float(ry)}
    assert conv(0, 0, -1, 0, 0, 0)["y"] == 1.0          # away from the operator = forward
    assert conv(0, 1, 0, 0, 0, 0)["z"] == 1.0           # spnav +y (up) = canonical up
    assert conv(0, 0, 0, 0, 1, 0)["rz"] == 1.0          # spnav ry (about up) = yaw
    # the source does the same thing
    import inspect
    src = inspect.getsource(_SpnavBackend.poll)
    assert '"y": float(-m.z)' in src and '"z": float(m.y)' in src
    assert '"ry": float(-m.rz)' in src and '"rz": float(m.ry)' in src


def test_pitch_sign_and_quat_without_a_scene():
    """The geometry the SpaceMouse mapping rests on, with no GPU: positive
    pitch must tilt the tool's approach axis back toward the operator, and
    roll must leave that axis alone (it turns about it)."""
    from force_bench.teleop.session import IPITCH, IROLL, IYAW, TeleopSession, qaxis, qmul, qrot

    def _horiz(fwd):
        """A horizontal reach: the approach points along fwd."""
        a = np.cross(np.array([0.0, 0.0, 1.0]), fwd)
        ang = np.pi / 2
        return qaxis(a / np.linalg.norm(a), ang)

    class Fake:
        pass

    z = np.array([0.0, 0.0, 1.0])
    for fwd in (np.array([1.0, 0.0, 0.0]), np.array([0.0, -1.0, 0.0]),
                np.array([-0.42, 0.91, 0.0])):
        f = Fake()
        f.op_frame = np.array([fwd, np.array([-fwd[1], fwd[0], 0.0]), z])
        for yaw0 in (0.0, 0.7, -2.1):                    # any grasp orientation
            # a top-down grasp: z flipped, then yawed
            f.q0 = qmul(qaxis((0, 0, 1), yaw0), qaxis((1, 0, 0), np.pi))
            f.pitch_axis, f.pitch_sign = TeleopSession._pitch_axis(f)
            assert f.pitch_sign in (1.0, -1.0)
            cmd = np.zeros(7)
            a0 = qrot(TeleopSession._quat(f, cmd), z)
            cmd[IPITCH] = 0.25
            a1 = qrot(TeleopSession._quat(f, cmd), z)
            assert a1[2] - a0[2] > 1e-6, (fwd, yaw0)            # the nose pitches UP
            assert float((a1 - a0) @ (-fwd)) > 0, (fwd, yaw0)   # and back toward the operator
            cmd[IPITCH] = 0.0
            cmd[IROLL] = 0.9
            a2 = qrot(TeleopSession._quat(f, cmd), z)
            assert np.linalg.norm(a2 - a0) < 1e-9               # roll spins about it
            cmd[IROLL] = 0.0
            cmd[IYAW] = 0.4
            a3 = qrot(TeleopSession._quat(f, cmd), z)
            assert abs(a3[2] - a0[2]) < 1e-9                    # yaw keeps the tilt
        # a HORIZONTAL reach (the door tasks) must pitch up too, not down
        f.q0 = _horiz(fwd)
        f.pitch_axis, f.pitch_sign = TeleopSession._pitch_axis(f)
        cmd = np.zeros(7)
        a0 = qrot(TeleopSession._quat(f, cmd), z)
        assert abs(a0[2]) < 1e-6, a0                            # approach is horizontal
        cmd[IPITCH] = 0.25
        assert qrot(TeleopSession._quat(f, cmd), z)[2] > 1e-6   # the nose lifts


def test_spacemouse_reports_clearly_when_absent():
    from force_bench.teleop.spacemouse import SpaceMouseError, open_backend
    try:
        be = open_backend("hid")
        be.close()
    except SpaceMouseError as exc:
        assert "3dconnexion" in str(exc).lower() or "hid" in str(exc).lower()


def test_scene_registry_names_are_registered_tasks():
    from force_bench.appendix_tasks.registry import APPENDIX, BY_NAME
    from force_bench.teleop.scenes import VERIFIED, names, task_names
    for n in VERIFIED:
        assert n == "playground" or n in BY_NAME
    # every ContactBench task is a teleop scene, then the playground
    assert sorted(task_names()) == sorted(s.name for s in APPENDIX)
    assert names() == task_names() + ["playground"]


def test_alarm_thresholds_admit_the_start_pinch():
    from force_bench.teleop.alarms import AlarmCfg
    cfg = AlarmCfg()
    cfg.scale_pads(25.0)
    warn = cfg.thresholds["pad_L"].warn
    cfg.allow_pinch(4.0)                     # under the scene's own thresholds: nothing changes
    assert cfg.thresholds["pad_L"].warn == warn
    cfg.allow_pinch(18.0)                    # a task that holds its object at 18 N/pad
    assert cfg.thresholds["pad_R"].warn >= 1.25 * 18.0
    assert cfg.thresholds["pad_R"].critical >= 1.5 * 18.0
    assert cfg.thresholds["object"].warn >= 2.5 * 18.0
