"""The web teleop UI: one HTTP+WebSocket port, one static page.

    browser  <-- JPEG frames (binary) + telemetry (JSON) --  WebServer  <--  session
    browser  --  input rates / commands (JSON)          -->  WebServer  -->  session

The page (``web/index.html``) renders the offscreen camera's JPEG stream
on a canvas, draws the projected contact-force arrows over it, shows the
force gauges, sparkline plots, the alarm banner (with an audible beep on
a new critical event), the task readouts and the alarm log, and turns the
keyboard and the browser Gamepad API into the same normalized input the
GL window uses.  Camera orbit / zoom / pan are mouse (or touch) drags on
the view.

The server is a thread running ``websockets``' asyncio server; the
simulation stays on the main thread.  Frames are published "latest wins":
a slow client skips frames rather than lagging behind.  Inputs carry a
timestamp and go STALE after ``stale_s`` -- a dropped connection stops
the arm, it does not leave it running the last command.
"""

from __future__ import annotations

import asyncio
import collections
import io
import json
import os
import threading
import time
from http import HTTPStatus

import numpy as np

from force_bench.teleop.inputs import InputState

HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web", "index.html")


def _reject_constant(name):
    """json.loads accepts NaN / Infinity literals; a NaN target passes every
    clamp and poisons the IK seed, so the whole message is dropped."""
    raise ValueError(f"non-finite JSON constant {name}")


class StopLoop(Exception):
    """Raised from a ``run_web`` on_frame callback to end the loop early."""


def _sanitize_input(d):
    """Normalise a raw 'input' message on the socket thread; None if malformed."""
    from force_bench.teleop.inputs import AXES, BUTTONS
    if not isinstance(d, dict):
        return None
    axes = d.get("axes") or {}
    buttons = d.get("buttons") or {}
    if not isinstance(axes, dict) or not isinstance(buttons, dict):
        return None
    return {"type": "input", "source": str(d.get("source", "web"))[:24],
            "axes": {a: _finite(v, 0.0) for a, v in axes.items() if a in AXES},
            "buttons": {b: bool(v) for b, v in buttons.items() if b in BUTTONS}}


def _finite(v, default=None):
    try:
        f = float(v)
    except (TypeError, ValueError):
        return default
    return f if np.isfinite(f) else default


class WebServer:
    def __init__(self, host: str = "127.0.0.1", port: int = 8765, stale_s: float = 0.5,
                 hello: dict | None = None):
        self.host, self.port = host, port
        self.stale_s = stale_s
        self.hello = hello or {}
        self.loop: asyncio.AbstractEventLoop | None = None
        self.thread: threading.Thread | None = None
        self.clients: set = set()
        self._lock = threading.Lock()
        self._input: dict | None = None
        self._input_t = 0.0
        self._commands: collections.deque = collections.deque()
        self._latest = {"jpeg": None, "telemetry": None, "seq": 0}
        self._event: asyncio.Event | None = None
        self._server = None
        self.started = threading.Event()
        self.error: str | None = None
        self.stats = {"frames_sent": 0, "inputs": 0, "bytes": 0}

    # ── lifecycle ─────────────────────────────────────────────────────
    def start(self):
        self.thread = threading.Thread(target=self._run, name="teleop-web", daemon=True)
        self.thread.start()
        if not self.started.wait(10.0):
            raise RuntimeError("web server did not start")
        if self.error:
            raise RuntimeError(self.error)
        return self

    def _run(self):
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.loop.run_until_complete(self._main())
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.started.set()

    async def _main(self):
        from websockets.asyncio.server import serve
        self._event = asyncio.Event()
        try:
            self._server = await serve(self._handler, self.host, self.port,
                                       process_request=self._http, compression=None,
                                       max_size=2 ** 20, ping_interval=20)
        except OSError as exc:
            self.error = f"cannot bind {self.host}:{self.port}: {exc}"
            self.started.set()
            return
        self.started.set()
        await self._broadcast()

    def stop(self):
        if self.loop is not None and self._server is not None:
            self.loop.call_soon_threadsafe(self._server.close)

    @property
    def url(self) -> str:
        host = "localhost" if self.host in ("0.0.0.0", "") else self.host
        return f"http://{host}:{self.port}/"

    # ── HTTP: the page ────────────────────────────────────────────────
    def _http(self, connection, request):
        if request.headers.get("Upgrade", "").lower() == "websocket":
            return None
        path = request.path.split("?")[0]
        if path in ("/", "/index.html"):
            with open(HTML_PATH, "r", encoding="utf-8") as f:
                html = f.read()
            # respond() sizes Content-Length from the text it is given, so the
            # page goes in here rather than being attached afterwards
            resp = connection.respond(HTTPStatus.OK, html)
            del resp.headers["Content-Type"]
            resp.headers["Content-Type"] = "text/html; charset=utf-8"
            resp.headers["Cache-Control"] = "no-store"
            return resp
        if path == "/health":
            resp = connection.respond(HTTPStatus.OK, json.dumps({"ok": True, "clients": len(self.clients)}))
            resp.headers["Content-Type"] = "application/json"
            return resp
        return connection.respond(HTTPStatus.NOT_FOUND, "not found\n")

    # ── WebSocket ─────────────────────────────────────────────────────
    async def _handler(self, ws):
        self.clients.add(ws)
        try:
            await ws.send(json.dumps({"type": "hello", **self.hello}))
            async for msg in ws:
                if isinstance(msg, bytes):
                    continue
                try:
                    d = json.loads(msg, parse_constant=_reject_constant)
                except Exception:
                    continue                 # malformed, or a NaN/Infinity literal
                if not isinstance(d, dict):
                    continue
                kind = d.get("type")
                if kind == "input":
                    clean = _sanitize_input(d)
                    if clean is None:
                        continue
                    with self._lock:
                        self._input = clean
                        self._input_t = time.monotonic()
                    self.stats["inputs"] += 1
                elif kind == "cmd":
                    with self._lock:
                        self._commands.append(d)
                elif kind == "ping":
                    await ws.send(json.dumps({"type": "pong", "t": d.get("t")}))
        except Exception:
            pass
        finally:
            self.clients.discard(ws)

    async def _broadcast(self):
        seq = -1
        while True:
            await self._event.wait()
            self._event.clear()
            with self._lock:
                jpeg, tm, s = self._latest["jpeg"], self._latest["telemetry"], self._latest["seq"]
            if s == seq or not self.clients:
                continue
            seq = s

            async def one(ws):
                try:
                    if jpeg is not None:
                        await asyncio.wait_for(ws.send(jpeg), 0.5)
                        self.stats["bytes"] += len(jpeg)
                    if tm is not None:
                        await asyncio.wait_for(ws.send(tm), 0.5)
                    self.stats["frames_sent"] += 1
                except Exception:
                    # a client that cannot take a frame in 0.5 s is dropped
                    # rather than allowed to stall everyone ("latest wins").
                    # Not `await ws.close()`: the close handshake drains the
                    # paused transport with no deadline and would park the
                    # whole gather; abort() is synchronous and connection_lost
                    # ends the handler, whose finally discards the client.
                    self.clients.discard(ws)
                    try:
                        ws.transport.abort()
                    except Exception:
                        pass
            await asyncio.gather(*(one(ws) for ws in list(self.clients)))

    # ── main-thread API ───────────────────────────────────────────────
    def publish(self, jpeg: bytes | None, telemetry: dict):
        if self.loop is None:
            return
        with self._lock:
            if jpeg is not None:
                self._latest["jpeg"] = jpeg
            self._latest["telemetry"] = json.dumps(telemetry, default=float)
            self._latest["seq"] += 1
        self.loop.call_soon_threadsafe(self._event.set)

    def current_input(self) -> InputState:
        with self._lock:
            d, t = self._input, self._input_t
        st = InputState(source="web")
        if d is None:
            st.stale = True
            return st
        try:
            st = InputState.from_dict(d)
        except Exception:
            st = InputState(source="web")
        st.stale = (time.monotonic() - t) > self.stale_s
        if st.stale:
            st.axes[:] = 0.0
        return st

    def drain_commands(self) -> list:
        with self._lock:
            out = list(self._commands)
            self._commands.clear()
        return out

    def active(self) -> bool:
        with self._lock:
            return self._input is not None and (time.monotonic() - self._input_t) <= self.stale_s


def encode_jpeg(img: np.ndarray, quality: int = 80) -> bytes:
    from PIL import Image
    b = io.BytesIO()
    Image.fromarray(img).save(b, "JPEG", quality=quality)
    return b.getvalue()


class WebBridge:
    """Ties a WebServer to a session + offscreen camera.  Both UI loops call
    ``poll()`` before the step and ``publish()`` after it."""

    def __init__(self, session, camera, server: WebServer, stream_fps: float = 30.0,
                 quality: int = 80):
        self.session = session
        self.camera = camera
        self.server = server
        self.quality = quality
        self.stream_fps = stream_fps
        self._last_render = 0.0
        self.show_arrows = True

    def apply_commands(self):
        ses = self.session
        for c in self.server.drain_commands():
            a = c.get("action")
            try:
                if a == "guard":
                    ses.guard.enabled = bool(c.get("value", not ses.guard.enabled))
                    ses._note(f"guard {'ON' if ses.guard.enabled else 'OFF'} (web)")
                elif a == "reset":
                    ses.reset()
                elif a == "pause":
                    ses.paused = bool(c.get("value", not ses.paused))
                    ses._note("paused (web)" if ses.paused else "running (web)")
                elif a == "speed":
                    v = _finite(c.get("value"))
                    if v is not None:
                        ses.speed_scale = float(np.clip(v, 0.1, 3.0))
                elif a == "speed_up":
                    ses.speed_scale = float(min(3.0, ses.speed_scale * 1.5))
                elif a == "speed_down":
                    ses.speed_scale = float(max(0.1, ses.speed_scale / 1.5))
                elif a == "threshold":
                    thr = ses.alarms.cfg.thresholds.get(str(c.get("channel")))
                    if thr is not None:
                        w = _finite(c.get("warn"))
                        cr = _finite(c.get("critical"))
                        if w is not None:
                            thr.warn = max(0.0, w)
                        if cr is not None:
                            thr.critical = max(0.0, cr)
                        thr.critical = max(thr.critical, thr.warn)
                elif a == "camera":
                    pan = c.get("pan")
                    if pan is not None and not np.isfinite(np.asarray(pan, dtype=float)).all():
                        pan = None
                    self.camera.orbit(_finite(c.get("d_az"), 0.0), _finite(c.get("d_el"), 0.0),
                                      _finite(c.get("zoom"), 1.0), pan=pan)
                elif a == "view_reset":
                    self.camera.reset_view()
                elif a == "arrows":
                    self.show_arrows = bool(c.get("value", True))
                elif a == "target":
                    ses.set_target(pos=c.get("pos"), yaw=c.get("yaw"), roll=c.get("roll"),
                                   jaw=c.get("jaw"))
            except Exception as exc:
                ses._note(f"web command {a} failed: {exc}")

    def poll(self) -> InputState:
        self.apply_commands()
        return self.server.current_input()

    def publish(self, telemetry: dict | None = None):
        ses = self.session
        tm = telemetry if telemetry is not None else ses.telemetry()
        if not self.server.clients:
            return tm                    # nobody watching: no render, no encode, no projection
        now = time.perf_counter()
        jpeg = None
        if now - self._last_render >= 1.0 / self.stream_fps - 1e-4:
            self._last_render = now
            img = self.camera.render(ses.rig.s0)
            jpeg = encode_jpeg(img, self.quality)
        arrows = tm.get("arrows", []) if self.show_arrows else []
        a2d = []
        if arrows:
            pts = np.array([[*a["s"], *a["e"]] for a in arrows], dtype=float).reshape(-1, 3)
            px, front = self.camera.project(pts)
            for i, a in enumerate(arrows):
                if front[2 * i] and front[2 * i + 1]:
                    a2d.append({"s": [round(float(px[2 * i, 0]), 1), round(float(px[2 * i, 1]), 1)],
                                "e": [round(float(px[2 * i + 1, 0]), 1), round(float(px[2 * i + 1, 1]), 1)],
                                "N": a["N"], "level": a["level"], "pair": a["pair"]})
        tcp_px, tcp_front = self.camera.project(np.array([[tm["tcp"]["x"], tm["tcp"]["y"], tm["tcp"]["z"]],
                                                          [tm["cmd"]["x"], tm["cmd"]["y"], tm["cmd"]["z"]]]))
        tm = dict(tm)
        tm["type"] = "telemetry"
        tm["arrows2d"] = a2d
        tm["tcp_px"] = [round(float(v), 1) for v in tcp_px[0]] if tcp_front[0] else None
        tm["cmd_px"] = [round(float(v), 1) for v in tcp_px[1]] if tcp_front[1] else None
        tm["camera"] = self.camera.pose()
        tm["clients"] = len(self.server.clients)
        tm["stream"] = {"w": self.camera.width, "h": self.camera.height, "fps": self.stream_fps}
        tm.pop("arrows", None)
        self.server.publish(jpeg, tm)
        return tm


def run_web(session, camera, server: WebServer, frames: int | None = None, script=None,
            stream_fps: float = 30.0, on_frame=None):
    """Headless loop: web clients (or a script) drive the session."""
    bridge = WebBridge(session, camera, server, stream_fps=stream_fps)
    ses = session
    period = ses.dt
    i = 0
    try:
        while frames is None or i < frames:
            t0 = time.perf_counter()
            if script is not None:
                bridge.apply_commands()
                inp = script.poll(ses.t, ses)
                web = server.current_input()
                if not web.stale and (np.abs(web.axes).max() > 0 or any(web.buttons.values())):
                    inp = web
            else:
                inp = bridge.poll()
            ses.apply_input(inp)
            ses.step()
            tm = bridge.publish()
            if on_frame is not None:
                on_frame(i, tm)            # may raise StopLoop to stop early
            i += 1
            if ses.cfg.realtime:
                el = time.perf_counter() - t0
                if el < period:
                    time.sleep(period - el)
    except StopLoop:
        pass
    return bridge
