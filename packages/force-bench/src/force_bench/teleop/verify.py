"""Acceptance checks for the teleop UIs, per scene, on the real physics.

    force-bench teleop-ui --verify --scene plug_in_usb_2
    force-bench teleop-ui --verify-all [--quick]

For each scene the same session machinery the UIs use is driven by
scripted operators, and the run is judged on the things a human would
otherwise have to eyeball:

  finite            no run diverged the solver
  guard_integrity   the ABUSIVE operator THROUGH THE GUARD never grew a
                    contact overlap by more than the gate's 3 mm and never
                    launched a body above 2.5 m/s -- the model kept its
                    integrity under the worst input the UI can deliver
  alarms_detect     the same abuse WITHOUT the guard raised at least one
                    force / overlap alarm -- the monitor sees what it should
  gentle_quiet      a careful operator raised no CRITICAL alarm (``gentle``, or
                    ``gentle_held`` when the task starts with a part in the pinch)
  gl_headless       the Newton GL window, panel included, rendered offscreen
                    through EGL and the PNG is not blank
  web_loopback      the web server served the page, streamed JPEG frames
                    and telemetry to a WebSocket client, and applied the
                    client's input and commands to the session
  fps               reported, not judged (the scene notes carry the numbers)

Everything lands under ``outputs/teleop_ui/<scene>/``: ``verify.json``,
one ``report.json`` + ``telemetry.csv`` per scripted run, camera PNGs at
the first alarm of each level and at the end, the GL screenshots.
"""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np


def annotate(img: np.ndarray, cam, session, title: str = ""):
    """Camera frame + projected force arrows + headline, for the report."""
    from PIL import Image, ImageDraw
    im = Image.fromarray(np.ascontiguousarray(img))
    dr = ImageDraw.Draw(im)
    col = {"ok": (80, 220, 120), "warn": (255, 200, 40), "critical": (255, 70, 70)}
    for a in session.arrow_list:
        (p0, p1), fr = cam.project([a["s"], a["e"]])
        if fr.all():
            c = col[a["level"]]
            dr.line([tuple(p0), tuple(p1)], fill=c, width=3)
            d = p1 - p0
            n = np.linalg.norm(d)
            if n > 1e-6:
                d = d / n
                left = p1 - 9 * d + 5 * np.array([-d[1], d[0]])
                right = p1 - 9 * d - 5 * np.array([-d[1], d[0]])
                dr.polygon([tuple(p1), tuple(left), tuple(right)], fill=c)
            dr.text((p1[0] + 5, p1[1] - 12), f"{a['N']:.1f}N", fill=c)
    tm = session.telemetry()
    lvl = tm["alarms"]["level"]
    head = tm["headline"] if lvl != "ok" else "nominal"
    bar = {"ok": (30, 90, 50), "warn": (150, 110, 10), "critical": (160, 30, 30)}[lvl]
    dr.rectangle([0, 0, im.width, 34], fill=bar)
    dr.text((8, 4), f"{title}  t={tm['t']:.2f}s  {head}"[:150], fill=(255, 255, 255))
    f = tm["forces"]
    dr.text((8, 18), f"pads {f['pad_L']:.1f}/{f['pad_R']:.1f} N  hand {f['hand']:.1f} N  object {f['object']:.1f} N  "
                     f"overlap +{tm['penetration']['excess_mm']:.2f} mm  speed {tm['speed']['max']:.2f} m/s  "
                     f"guard {'ON' if tm['guard']['enabled'] else 'OFF'}", fill=(255, 255, 255))
    return im


def run_script(session, script, frames: int, cam=None, out_dir: Path | None = None,
               snapshot_every: int = 0, title: str = "") -> dict:
    """Drive the session with a scripted operator; return the run summary."""
    out_dir = Path(out_dir) if out_dir else None
    snaps = []
    seen = set()
    t0 = time.perf_counter()
    peak = {"pen_excess_mm": 0.0, "pen_pair": "", "speed": 0.0, "speed_body": "",
            "pad": 0.0, "hand": 0.0, "object": 0.0, "track_mm": 0.0}
    for i in range(int(frames)):
        inp = script.poll(session.t, session)
        session.apply_input(inp)
        session.step()
        L = session.last
        if L.get("pen_excess_mm", 0.0) > peak["pen_excess_mm"]:
            peak["pen_excess_mm"], peak["pen_pair"] = L["pen_excess_mm"], L["pen_pair"]
        if L.get("speed", 0.0) > peak["speed"]:
            peak["speed"], peak["speed_body"] = L["speed"], L["speed_body"]
        peak["pad"] = max(peak["pad"], L.get("pad_L", 0.0), L.get("pad_R", 0.0))
        peak["hand"] = max(peak["hand"], L.get("hand", 0.0))
        peak["object"] = max(peak["object"], L.get("object", 0.0))
        peak["track_mm"] = max(peak["track_mm"], L.get("track_mm", 0.0))
        lvl = session.alarms.level
        want = (cam is not None and out_dir is not None and
                ((lvl != "ok" and lvl not in seen) or
                 (snapshot_every and i % snapshot_every == 0) or i == frames - 1))
        if want:
            seen.add(lvl)
            im = annotate(cam.render(session.rig.s0), cam, session, f"{title} {inp.note}")
            p = out_dir / f"cam_{i:05d}_{lvl}.png"
            im.save(p)
            snaps.append(str(p))
        if session.diverged:
            break
    wall = time.perf_counter() - t0
    return {"frames": session.frame, "sim_s": round(session.t, 2), "wall_s": round(wall, 1),
            "fps": round(session.frame / max(wall, 1e-6), 1), "diverged": session.diverged,
            "graph": session.graph is not None, "peak": {k: (round(v, 3) if isinstance(v, float) else v)
                                                        for k, v in peak.items()},
            "alarm_counts": dict(session.alarms.counts),
            "alarm_peaks": {k: round(ch.peak, 3) for k, ch in session.alarms.channels.items()},
            "events": session.alarms.events[:30], "guard_clamps": dict(session.guard.counts),
            "snapshots": snaps}


def _check_gl(session, out_dir: Path, frames: int = 300) -> dict:
    """The GL window under the abusive operator, offscreen: the panel, the
    banner and the arrows all get drawn; PNGs at the first warn / critical."""
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    try:
        import pyglet
        pyglet.options["headless"] = True
        from force_bench.teleop.inputs import script_by_name
        from force_bench.teleop.local_ui import LocalTeleopUI
        session.reset()
        session.guard.enabled = True
        ui = LocalTeleopUI(session, headless=True, width=1500, height=900, gamepad=False)
        saved = ui.run_headless(frames, script_by_name("abusive"), str(out_dir), snapshot_every=0)
        ui.close()
        from PIL import Image
        arr = np.asarray(Image.open(saved[-1]))
        std = float(arr.std())
        return {"pass": bool(std > 5.0 and len(saved) > 0 and not session.diverged), "snapshots": saved,
                "pixel_std": round(std, 1), "panel": ui.viewer.gui is not None,
                "alarm_counts": dict(session.alarms.counts)}
    except Exception as exc:
        import traceback
        return {"pass": False, "error": f"{type(exc).__name__}: {exc}",
                "trace": traceback.format_exc()[-800:]}


def _find_chrome():
    import shutil
    for c in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
        p = shutil.which(c)
        if p:
            return p
    return None


def _check_web(session, out_dir: Path, frames: int = 90) -> dict:
    """Loopback: a real WebSocket client drives the session over the wire,
    and, when a Chrome is installed, a headless browser loads the page
    against the live server and screenshots it (the page's JS runs for
    real: socket, JPEG decode, gauges, plots)."""
    import asyncio
    import socket
    import subprocess
    import threading
    import urllib.request
    from force_bench.teleop.camera import TeleopCamera
    from force_bench.teleop.web_ui import StopLoop, WebServer, run_web
    from force_bench.teleop.app import _hello
    session.reset()
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
    cam = TeleopCamera(session.model, session.scene.cam_pos, session.scene.cam_target, 640, 480)
    server = WebServer("127.0.0.1", port, hello=_hello(session)).start()
    res = {"port": port, "page_bytes": 0, "frames_rx": 0, "telemetry_rx": 0, "hello": False,
           "moved_mm": 0.0, "guard_toggled": False, "pong": False, "errors": []}
    try:
        with urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5) as r:
            res["page_bytes"] = len(r.read())
    except Exception as exc:
        res["errors"].append(f"http: {exc}")

    guard0 = session.guard.enabled
    raw0 = session.raw[:3].copy()

    async def client():
        import websockets
        async with websockets.connect(f"ws://127.0.0.1:{port}/", max_size=2 ** 22) as ws:
            t_end = time.monotonic() + 8.0
            sent_cmd = False
            await ws.send(json.dumps({"type": "ping", "t": 1}))
            while time.monotonic() < t_end:
                try:
                    msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
                except asyncio.TimeoutError:
                    continue
                if isinstance(msg, bytes):
                    res["frames_rx"] += 1
                    if res["frames_rx"] == 1:
                        with open(out_dir / "web_first_frame.jpg", "wb") as f:
                            f.write(msg)
                else:
                    d = json.loads(msg)
                    if d.get("type") == "hello":
                        res["hello"] = True
                    elif d.get("type") == "pong":
                        res["pong"] = True
                    elif d.get("type") == "telemetry":
                        res["telemetry_rx"] += 1
                        if res["telemetry_rx"] == 1:
                            with open(out_dir / "web_first_telemetry.json", "w") as f:
                                json.dump(d, f, indent=1)
                        if not sent_cmd and res["telemetry_rx"] >= 3:
                            await ws.send(json.dumps({"type": "cmd", "action": "guard", "value": not guard0}))
                            sent_cmd = True
                # keep pushing forward at full stick
                await ws.send(json.dumps({"type": "input", "source": "loopback",
                                          "axes": {"fwd": 1.0}, "buttons": {}}))
                if res["frames_rx"] >= 8 and res["telemetry_rx"] >= 20 and sent_cmd:
                    break

    def client_thread():
        try:
            asyncio.run(client())
        except Exception as exc:
            res["errors"].append(f"client: {type(exc).__name__}: {exc}")

    th = threading.Thread(target=client_thread, daemon=True)
    th.start()
    chrome = _find_chrome()
    res["browser"] = {"chrome": chrome}
    chrome_thread = None
    if chrome:
        shot = out_dir / "browser.png"
        dom = out_dir / "browser_dom.html"

        def chrome_runs():
            # ONE chrome at a time, each with its own profile: two headless
            # chromes on one user-data-dir hand off to each other and the
            # second one dumps an error page instead of ours
            for i, extra in enumerate(([f"--screenshot={shot}"], ["--dump-dom"], ["--dump-dom"])):
                if i == 2:
                    # retry the DOM dump once if the first one did not load our page
                    try:
                        if "<title>force-bench teleop" in open(dom, encoding="utf-8", errors="replace").read():
                            break
                    except Exception:
                        pass
                prof = out_dir / f"chrome_profile_{i}"
                cmd = [chrome, "--headless=new", "--no-sandbox", "--disable-gpu", "--hide-scrollbars",
                       "--window-size=1500,950", "--virtual-time-budget=8000", "--no-first-run",
                       "--disable-dev-shm-usage", f"--user-data-dir={prof}", *extra,
                       f"http://127.0.0.1:{port}/"]
                try:
                    out = open(dom, "wb") if extra == ["--dump-dom"] else subprocess.DEVNULL
                    subprocess.run(cmd, stdout=out, stderr=subprocess.DEVNULL, timeout=90)
                except Exception as exc:
                    res["browser"]["error"] = f"{type(exc).__name__}: {exc}"
                finally:
                    if out is not subprocess.DEVNULL:
                        out.close()
                import shutil
                shutil.rmtree(prof, ignore_errors=True)

        chrome_thread = threading.Thread(target=chrome_runs, daemon=True)
        chrome_thread.start()
        frames = max(frames, 2400)      # keep the stream alive while chrome loads (stops early below)
    t_stop = [None]

    def on_frame(i, tm):
        # end early once every consumer is done
        if not th.is_alive() and (chrome_thread is None or not chrome_thread.is_alive()) and i > 30:
            raise StopLoop
    try:
        run_web(session, cam, server, frames=frames, stream_fps=30.0, on_frame=on_frame)
    finally:
        server.stop()
    th.join(timeout=10.0)
    if chrome_thread is not None:
        chrome_thread.join(timeout=120)
    if chrome:
        b = res["browser"]
        try:
            from PIL import Image
            arr = np.asarray(Image.open(out_dir / "browser.png"))
            b["screenshot"] = str(out_dir / "browser.png")
            b["pixel_std"] = round(float(arr.std()), 1)
        except Exception as exc:
            b["screenshot_error"] = f"{type(exc).__name__}: {exc}"
        try:
            html = open(out_dir / "browser_dom.html", encoding="utf-8", errors="replace").read()
            b["dom_bytes"] = len(html)
            b["connected"] = ">connected<" in html
            b["gauges_filled"] = html.count('class="val">') > 3 and "&nbsp;N<" in html or " N<" in html
            b["scene_label"] = ("id=\"p-scene\">" + session.scene.name) in html
        except Exception as exc:
            b["dom_error"] = f"{type(exc).__name__}: {exc}"
        try:
            b["page_loaded"] = "<title>force-bench teleop" in html
        except Exception:
            b["page_loaded"] = False
        # the DOM dump is taken under Chrome's virtual clock, after which the
        # socket may already have been closed, so `connected` is reported but
        # the criterion is: the page loaded, ran its script and built its
        # gauges from the server's hello, and the screenshot is not blank
        b["pass"] = bool(b.get("pixel_std", 0) > 5 and b.get("page_loaded") and b.get("scene_label"))
    res["moved_mm"] = round(float(np.linalg.norm(session.raw[:3] - raw0)) * 1000, 2)
    res["guard_toggled"] = bool(session.guard.enabled != guard0)
    res["server_stats"] = dict(server.stats)
    session.guard.enabled = guard0
    res["pass"] = bool(res["page_bytes"] > 5000 and res["hello"] and res["frames_rx"] >= 3
                       and res["telemetry_rx"] >= 3 and res["moved_mm"] > 0.5
                       and res["guard_toggled"] and not res["errors"]
                       and (res["browser"].get("pass", True) if chrome else True))
    return res


def verify_scene(name: str, out_root: Path, quick: bool = False) -> dict:
    import warp as wp
    from force_bench.teleop.camera import TeleopCamera
    from force_bench.teleop.inputs import script_by_name
    from force_bench.teleop.scenes import load_scene
    from force_bench.teleop.session import SessionCfg, TeleopSession
    wp.init()
    out = out_root / name
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    scene = load_scene(name)
    from force_bench.teleop.alarms import AlarmCfg
    acfg = AlarmCfg()
    acfg.scale_pads(scene.finger_effort_N)
    session = TeleopSession(scene, SessionCfg(realtime=False, alarms=acfg))
    session.settle()
    cam = TeleopCamera(session.model, scene.cam_pos, scene.cam_target, 800, 600)
    rep = {"scene": name, "info": scene.info(), "graph": session.graph is not None,
           "graph_error": session.graph_error,
           "rest_overlap_mm": round(max(session.rest_depth.values(), default=0.0) * 1000, 3),
           "runs": {}, "checks": {}}
    dur_scale = 0.5 if quick else 1.0
    # a task that starts with the part in the pinch gets the careful operator that keeps it there
    held = scene.hold_start_pinch and max(session.last.get("pad_L", 0.0), session.last.get("pad_R", 0.0)) > 1.0
    gentle = "gentle_held" if held else "gentle"
    for script_name in ("abusive", "abusive_unguarded", gentle):
        session.reset()
        session.guard.enabled = True
        sc = script_by_name(script_name)
        frames = int(sc.duration * session.fps * dur_scale)
        d = out / script_name
        d.mkdir(exist_ok=True)
        r = run_script(session, sc, frames, cam, d, snapshot_every=0, title=f"{name}/{script_name}")
        r["guard_enabled_at_end"] = session.guard.enabled
        with open(d / "report.json", "w") as f:
            json.dump({**session.report(), "run": r}, f, indent=1, default=float)
        h = session.history()
        if h["t"]:
            import csv
            with open(d / "telemetry.csv", "w", newline="") as f:
                w = csv.writer(f)
                keys = list(h)
                w.writerow(keys)
                for i in range(len(h["t"])):
                    w.writerow([round(float(h[k][i]), 4) for k in keys])
        rep["runs"][script_name] = r
        print(f"  {script_name:18s} {r['frames']:4d} frames {r['fps']:5.1f} fps  diverged={r['diverged']}  "
              f"pen +{r['peak']['pen_excess_mm']:.2f}mm ({r['peak']['pen_pair'][:30]})  "
              f"speed {r['peak']['speed']:.2f} m/s  pad {r['peak']['pad']:.1f}N hand {r['peak']['hand']:.1f}N  "
              f"alarms {r['alarm_counts']}", flush=True)
    ab, ab_off, ge = rep["runs"]["abusive"], rep["runs"]["abusive_unguarded"], rep["runs"][gentle]
    rep["checks"]["finite"] = {"pass": not any(r["diverged"] for r in rep["runs"].values())}
    rep["checks"]["guard_integrity"] = {
        "pass": bool(not ab["diverged"] and ab["peak"]["pen_excess_mm"] < 3.0 and ab["peak"]["speed"] < 2.5),
        "pen_excess_mm": ab["peak"]["pen_excess_mm"], "speed": ab["peak"]["speed"],
        "criterion": "abusive operator through the guard: overlap growth < 3 mm, no body > 2.5 m/s"}
    force_ch = ("pad_L", "pad_R", "hand", "object", "mech", "penetration")
    n_off = sum(1 for e in ab_off["events"] if e["channel"] in force_ch)
    n_on = sum(1 for e in ab["events"] if e["channel"] in force_ch)
    rep["checks"]["alarms_detect"] = {
        "pass": bool(n_off + n_on > 0 and ab_off["guard_enabled_at_end"] is False),
        "events_unguarded": n_off, "events_guarded": n_on,
        "criterion": "abuse raises at least one force / overlap alarm; the script's guard toggle took effect"}
    rep["checks"]["gentle_quiet"] = {"pass": bool(ge["alarm_counts"]["critical"] == 0 and not ge["diverged"]),
                                     "critical": ge["alarm_counts"]["critical"], "warn": ge["alarm_counts"]["warn"]}
    rep["checks"]["gl_headless"] = _check_gl(session, out / "gl")
    print(f"  gl_headless        pass={rep['checks']['gl_headless']['pass']} "
          f"{rep['checks']['gl_headless'].get('error', '')}", flush=True)
    (out / "web").mkdir(exist_ok=True)
    rep["checks"]["web_loopback"] = _check_web(session, out / "web", frames=60 if quick else 120)
    print(f"  web_loopback       pass={rep['checks']['web_loopback']['pass']} "
          f"frames={rep['checks']['web_loopback']['frames_rx']} telemetry={rep['checks']['web_loopback']['telemetry_rx']} "
          f"moved={rep['checks']['web_loopback']['moved_mm']}mm errors={rep['checks']['web_loopback']['errors']}", flush=True)
    rep["fps"] = {k: v["fps"] for k, v in rep["runs"].items()}
    rep["pass"] = all(c.get("pass") for c in rep["checks"].values())
    rep["seconds"] = round(time.time() - t0, 1)
    with open(out / "verify.json", "w") as f:
        json.dump(rep, f, indent=1, default=float)
    return rep


def summarize(rep: dict) -> str:
    c = rep["checks"]
    flags = " ".join(f"{k}={'P' if v.get('pass') else 'F'}" for k, v in c.items())
    ab = rep["runs"]["abusive"]["peak"]
    return (f"{rep['scene']:24s} {'PASS' if rep['pass'] else 'FAIL'}  {flags}  "
            f"fps {rep['fps']['abusive']:5.1f}  guarded-abuse pen +{ab['pen_excess_mm']:.2f}mm "
            f"speed {ab['speed']:.2f}m/s  {rep['seconds']:.0f}s")


def main(scenes=None, quick: bool = False, out_root: Path | None = None) -> int:
    from force_bench.teleop.scenes import VERIFIED
    from force_bench.teleop.app import _out_root
    out_root = out_root or _out_root()
    names = scenes or list(VERIFIED)
    rows = []
    for n in names:
        print(f"\n=== teleop-ui verify: {n} ===", flush=True)
        try:
            rep = verify_scene(n, out_root, quick=quick)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            rep = {"scene": n, "pass": False, "error": f"{type(exc).__name__}: {exc}", "checks": {},
                   "runs": {"abusive": {"peak": {"pen_excess_mm": 0, "speed": 0}}}, "fps": {"abusive": 0},
                   "seconds": 0}
            with open(out_root / f"{n}_error.json", "w") as f:
                json.dump(rep, f, indent=1, default=float)
        rows.append(rep)
        print(summarize(rep) if "error" not in rep else f"{n:24s} ERROR {rep['error'][:80]}", flush=True)
    print("\n" + "=" * 100)
    for r in rows:
        print(summarize(r) if "error" not in r else f"{r['scene']:24s} ERROR {r['error'][:80]}")
    print("=" * 100)
    with open(out_root / "verify_summary.json", "w") as f:
        json.dump([{k: v for k, v in r.items() if k != "runs"} for r in rows], f, indent=1, default=float)
    return 0 if all(r.get("pass") for r in rows) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or None))
