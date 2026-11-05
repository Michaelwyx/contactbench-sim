"""``force-bench teleop-ui`` -- drive any ContactBench task by hand, watch the forces.

    force-bench teleop-ui --list-scenes
    force-bench teleop-ui --scene plug_in_usb_2                # a task of force-bench list: Newton GL window
    force-bench teleop-ui --scene plug_in_usb_2 --seed 3       # ...from a random start, as force-bench run
    force-bench teleop-ui --scene turn_dial_to_target --set "Object geometry=Round knob"
    force-bench teleop-ui --scene retrieve_utensil_from_bin --embodiment finray   # a scene without a scripted policy
    force-bench teleop-ui --scene plug_in_usb_2 --ui web       # browser at http://127.0.0.1:8765/
    force-bench teleop-ui --scene plug_in_usb_2 --ui both      # GL window + browser on the same session
    force-bench teleop-ui --scene plug_in_usb_2 --ui web --host 0.0.0.0 --port 8765   # reachable on the LAN
    force-bench teleop-ui --scene playground --no-guard        # A/B the guard (alarms keep running)
    force-bench teleop-ui --scene plug_in_usb_2 --ui none --script abusive --out outputs/teleop_ui/usb
                                                               # headless scripted session -> report + PNGs
    force-bench teleop-ui --verify --scene plug_in_usb_2       # the whole acceptance check for one scene
    force-bench teleop-ui --verify-all [--quick]               # ...for every verified scene
    force-bench teleop-ui --record --scene plug_in_usb_2       # videos of a demo teleop: GL window + web view

Scenes are the ContactBench tasks, built as ``force-bench run`` builds them
(see ``teleop/scenes.py``), and the playground; the operator takes over at
the task's start pose.  Every knob of the session (speed, thresholds,
guard, graph capture, substeps) is a flag.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path


def _out_root() -> Path:
    from force_bench.paths import OUTPUT_DIR
    return Path(OUTPUT_DIR).parent / "teleop_ui"


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="force-bench teleop-ui", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", default="playground",
                    help="a ContactBench task (force-bench list) or the playground")
    ap.add_argument("--list-scenes", action="store_true")
    ap.add_argument("--seed", type=int, default=None,
                    help="a ContactBench task's random start, as force-bench run --seed")
    ap.add_argument("--set", action="append", default=[], metavar="AXIS=VALUE",
                    help="a ContactBench task's variation, as force-bench run --set (repeatable)")
    ap.add_argument("--embodiment", default=None, help="the fingers, as force-bench run --embodiment")
    ap.add_argument("--ui", choices=["gl", "web", "both", "none"], default="gl")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--stream", default="800x600", help="web stream resolution")
    ap.add_argument("--stream-fps", type=float, default=30.0)
    ap.add_argument("--no-guard", action="store_true", help="start with the guard off (T toggles it)")
    ap.add_argument("--no-graph", action="store_true", help="do not capture the physics frame as a CUDA graph")
    ap.add_argument("--substeps", type=int, default=None)
    ap.add_argument("--collide-substep", type=int, choices=[0, 1], default=None)
    ap.add_argument("--no-readouts", action="store_true", help="skip the task's own extra() readouts")
    ap.add_argument("--no-realtime", action="store_true", help="never sleep to the sim clock")
    ap.add_argument("--speed", type=float, default=0.15, help="full-stick translation rate, m/s")
    ap.add_argument("--speed-scale", type=float, default=1.0)
    ap.add_argument("--alarm", action="append", default=[], metavar="CH=WARN:CRIT",
                    help="override a threshold, e.g. --alarm pad_L=5:10 (repeatable)")
    ap.add_argument("--settle", type=int, default=30, help="frames held at the start pose for the rest baseline")
    ap.add_argument("--headless", action="store_true", help="GL: render offscreen through EGL (PNG snapshots)")
    ap.add_argument("--no-gamepad", action="store_true")
    ap.add_argument("--input", choices=["auto", "keyboard", "gamepad", "spacemouse"],
                    default="auto",
                    help="which device drives the arm in the GL window; auto takes a "
                         "3Dconnexion SpaceMouse when one opens, else the gamepad/keyboard")
    ap.add_argument("--spacemouse-test", action="store_true",
                    help="print the puck's raw counts and the robot axes they map to, live, "
                         "and exit -- the 10-second way to confirm the mapping on your device")
    ap.add_argument("--spacemouse-backend", choices=["auto", "spnav", "hid"], default="auto",
                    help="spnav = the spacenavd daemon (preferred on Linux), hid = raw HID")
    ap.add_argument("--spacemouse-invert", default="",
                    help="flip device axes, e.g. --spacemouse-invert y,rz (x y z rx ry rz)")
    ap.add_argument("--spacemouse-expo", type=float, default=None,
                    help="response curve exponent (default 2.0; 1.0 = linear)")
    ap.add_argument("--spacemouse-deadzone", type=float, default=None)
    ap.add_argument("--spacemouse-full-scale", type=float, default=None,
                    help="raw counts at full deflection (default 350)")
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=950)
    ap.add_argument("--script", default=None, help="hover | gentle | abusive | abusive_unguarded")
    ap.add_argument("--frames", type=int, default=None, help="stop after N frames (scripted / headless runs)")
    ap.add_argument("--out", default=None, help="report + snapshot directory for scripted runs")
    ap.add_argument("--snapshot-every", type=int, default=60)
    ap.add_argument("--verify", action="store_true", help="run the acceptance checks (see teleop/verify.py)")
    ap.add_argument("--verify-all", action="store_true", help="--verify over every verified scene")
    ap.add_argument("--quick", action="store_true", help="--verify with shorter scripts")
    ap.add_argument("--record", action="store_true",
                    help="record the scene under the demo operator: GL-window video, web-view video, key frames")
    ap.add_argument("--record-all", action="store_true", help="--record over every verified scene")
    ap.add_argument("--no-gl-video", action="store_true")
    ap.add_argument("--no-cam-video", action="store_true")
    ap.add_argument("--collect-dir", default=None,
                    help="save the session as aligned demonstration episodes (one per attempt; reset starts "
                         "the next); inspect them with `force-bench dataset <dir>`")
    return ap


def spacemouse_cfg(args):
    from force_bench.teleop.spacemouse import SpaceMouseCfg
    cfg = SpaceMouseCfg()
    if args.spacemouse_invert:
        cfg.parse_invert(args.spacemouse_invert)
    for attr, val in (("expo", args.spacemouse_expo), ("deadzone", args.spacemouse_deadzone),
                      ("full_scale", args.spacemouse_full_scale)):
        if val is not None:
            setattr(cfg, attr, float(val))
    return cfg


def open_spacemouse(args, required: bool):
    """Open the puck; None when there is none and it was not demanded."""
    from force_bench.teleop.spacemouse import SpaceMouse, SpaceMouseError
    cfg = spacemouse_cfg(args)
    try:
        sm = SpaceMouse(cfg, prefer=args.spacemouse_backend)
    except SpaceMouseError as exc:
        if required:
            raise SystemExit(f"[teleop-ui] --input spacemouse: {exc}")
        print(f"[teleop-ui] no SpaceMouse ({str(exc).splitlines()[0]}); "
              "falling back to the gamepad / keyboard")
        return None, cfg
    from force_bench.teleop.spacemouse import usb_devices_present
    print(f"[teleop-ui] SpaceMouse: {sm.name}")
    if not usb_devices_present():
        msg = ("no 3Dconnexion device is on the USB bus; the daemon connection succeeds "
               "anyway and the axes will read zero until the puck is plugged in")
        if required:
            print(f"[teleop-ui] WARNING: {msg}")
        else:
            sm.close()
            print(f"[teleop-ui] {msg} -- staying on the gamepad / keyboard")
            return None, cfg
    return sm, cfg


def task_args(args) -> tuple[str, ...]:
    """The force-bench run options for the scene's task."""
    out = [] if args.seed is None else ["--seed", str(args.seed)]
    for v in args.set:
        out += ["--set", v]
    if args.embodiment:
        out += ["--embodiment", args.embodiment]
    return tuple(out)


def make_session(args):
    import warp as wp
    wp.init()
    from force_bench.teleop.alarms import AlarmCfg
    from force_bench.teleop.scenes import load_scene
    from force_bench.teleop.session import SessionCfg, TeleopSession
    t0 = time.time()
    scene = load_scene(args.scene, substeps=args.substeps, collide_substep=args.collide_substep,
                       args=task_args(args))
    acfg = AlarmCfg()
    acfg.scale_pads(scene.finger_effort_N)
    for spec in args.alarm:
        acfg.override(spec)
    cfg = SessionCfg(speed_m_s=args.speed, speed_scale=args.speed_scale, guard=not args.no_guard,
                     graph=not args.no_graph, settle_frames=args.settle, readouts=not args.no_readouts,
                     realtime=not args.no_realtime, alarms=acfg)
    session = TeleopSession(scene, cfg)
    print(f"[teleop-ui] scene {scene.name}: {scene.note}", flush=True)
    print(f"[teleop-ui] {json.dumps(scene.info())}", flush=True)
    session.settle()
    print(f"[teleop-ui] settled ({time.time() - t0:.1f}s); physics {'CUDA graph' if session.graph else 'eager'}"
          f"{'' if session.graph else ' (' + str(session.graph_error) + ')'}; rest overlap max "
          f"{max(session.rest_depth.values(), default=0.0) * 1000:.2f} mm", flush=True)
    return session


def _hello(session) -> dict:
    thr = {k: {"warn": t.warn, "critical": t.critical, "unit": t.unit, "label": t.label}
           for k, t in session.alarms.cfg.thresholds.items()}
    return {"scene": session.scene.info(), "thresholds": thr,
            "controls": {"axes": ["fwd", "left", "up", "yaw", "roll", "jaw"]}}


def write_report(session, out_dir: Path, extra: dict | None = None) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    rep = session.report()
    if extra:
        rep.update(extra)
    p = out_dir / "report.json"
    with open(p, "w") as f:
        json.dump(rep, f, indent=1, default=float)
    h = session.history()
    if h["t"]:
        import csv
        with open(out_dir / "telemetry.csv", "w", newline="") as f:
            w = csv.writer(f)
            keys = list(h)
            w.writerow(keys)
            for i in range(len(h["t"])):
                w.writerow([round(float(h[k][i]), 4) for k in keys])
    return p


def run_none(args, session):
    """Headless scripted session with camera snapshots."""
    from force_bench.teleop.camera import TeleopCamera
    from force_bench.teleop.inputs import script_by_name
    from force_bench.teleop.verify import annotate, run_script
    script = script_by_name(args.script or "gentle")
    out = Path(args.out) if args.out else _out_root() / args.scene / f"none_{args.script or 'gentle'}"
    out.mkdir(parents=True, exist_ok=True)
    w, h = (int(v) for v in args.stream.split("x"))
    cam = TeleopCamera(session.model, session.scene.cam_pos, session.scene.cam_target, w, h)
    frames = args.frames or int(script.duration * session.fps)
    res = run_script(session, script, frames, cam, out, snapshot_every=args.snapshot_every)
    p = write_report(session, out, {"script": args.script or "gentle", "run": res})
    print(f"[teleop-ui] wrote {p}")
    print(json.dumps({k: v for k, v in res.items() if k != "snapshots"}, indent=1, default=float))


def _attach_spacemouse(args, ui):
    if args.input in ("keyboard", "gamepad") or args.headless:
        return
    sm, cfg = open_spacemouse(args, required=args.input == "spacemouse")
    if sm is not None:
        from force_bench.teleop.spacemouse import describe_mapping
        ui.attach_spacemouse(sm, cfg)
        print("[teleop-ui] SpaceMouse mapping (check it with --spacemouse-test):")
        print(describe_mapping())


def run_gl(args, session, web=None):
    from force_bench.teleop.local_ui import LocalTeleopUI
    ui = LocalTeleopUI(session, headless=args.headless, width=args.width, height=args.height,
                       gamepad=not args.no_gamepad)
    _attach_spacemouse(args, ui)
    if web is not None:
        ui.web = web
    if args.headless or args.script:
        from force_bench.teleop.inputs import script_by_name
        script = script_by_name(args.script or "gentle")
        out = Path(args.out) if args.out else _out_root() / args.scene / f"gl_{args.script or 'gentle'}"
        frames = args.frames or int(script.duration * session.fps)
        saved = ui.run_headless(frames, script, str(out), snapshot_every=args.snapshot_every) \
            if args.headless else ui.run_scripted(frames, script)
        p = write_report(session, out, {"script": args.script, "snapshots": saved})
        print(f"[teleop-ui] wrote {p} ({len(saved) if saved else 0} snapshots)")
        ui.close()
        return
    if ui.gamepad is not None:
        print(f"[teleop-ui] gamepad: {ui.gamepad.name or ('none' + (' (' + ui.gamepad.error + ')' if ui.gamepad.error else ''))}")
    print("[teleop-ui] GL window: W/S A/D R/F move, Q/E yaw, Z/X roll, G/H jaw, T guard, SPACE hold, "
          "BACKSPACE reset, P pause, [ ] speed; drag the tcp_target gizmo", flush=True)
    ui.run()


def run_web_mode(args, session, also_gl: bool = False):
    from force_bench.teleop.camera import TeleopCamera
    from force_bench.teleop.web_ui import WebBridge, WebServer, run_web
    w, h = (int(v) for v in args.stream.split("x"))
    cam = TeleopCamera(session.model, session.scene.cam_pos, session.scene.cam_target, w, h)
    server = WebServer(args.host, args.port, hello=_hello(session)).start()
    print(f"[teleop-ui] web UI at {server.url}  (bound to {args.host}:{args.port})", flush=True)
    if also_gl:
        from force_bench.teleop.local_ui import LocalTeleopUI
        ui = LocalTeleopUI(session, headless=args.headless, width=args.width, height=args.height,
                           gamepad=not args.no_gamepad)
        _attach_spacemouse(args, ui)
        ui.web = WebBridge(session, cam, server, stream_fps=args.stream_fps)
        print("[teleop-ui] GL window + web page share the session; the web input wins while active",
              flush=True)
        ui.run()
        server.stop()
        return
    script = None
    if args.script:
        from force_bench.teleop.inputs import script_by_name
        script = script_by_name(args.script)
    frames = args.frames or (int(script.duration * session.fps) if script else None)
    try:
        run_web(session, cam, server, frames=frames, script=script, stream_fps=args.stream_fps)
    finally:
        server.stop()
    if args.out or script:
        out = Path(args.out) if args.out else _out_root() / args.scene / f"web_{args.script or 'live'}"
        p = write_report(session, out, {"script": args.script, "web": server.stats})
        print(f"[teleop-ui] wrote {p}")


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if args.list_scenes:
        from force_bench.teleop.scenes import VERIFIED, task_note, task_names
        print("ContactBench tasks (force-bench list; --seed / --set / --embodiment as force-bench run):")
        for n in task_names():
            print(f"  {n:42s} {task_note(n, start=False)}")
        print(f"\n  {'playground':42s} {VERIFIED['playground']}")
        return 0
    if args.spacemouse_test:
        from force_bench.teleop.spacemouse import axis_test
        return axis_test(prefer=args.spacemouse_backend, cfg=spacemouse_cfg(args))
    if args.record or args.record_all:
        from force_bench.teleop.record import main as record_main
        from force_bench.teleop.scenes import VERIFIED
        scenes = list(VERIFIED) if args.record_all else [args.scene]
        return record_main(scenes, Path(args.out) if args.out else _out_root(),
                           script=args.script or "demo", gl=not args.no_gl_video,
                           cam=not args.no_cam_video, guard=not args.no_guard)
    if args.verify or args.verify_all:
        from force_bench.teleop.verify import main as verify_main
        scenes = None if args.verify_all else [args.scene]
        return verify_main(scenes, quick=args.quick, out_root=Path(args.out) if args.out else None)
    if args.ui == "gl" and args.headless:
        # EGL + pyglet headless must be configured before newton.viewer is imported
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
        import pyglet
        pyglet.options["headless"] = True
    session = make_session(args)
    if args.collect_dir:
        session.start_collection(args.collect_dir, info={"ui": args.ui, "input": args.input,
                                                         "script": args.script,
                                                         "task_args": list(task_args(args))})
        print(f"[teleop-ui] recording demonstrations to {args.collect_dir}", flush=True)
    try:
        if args.ui == "none":
            run_none(args, session)
        elif args.ui == "gl":
            run_gl(args, session)
        elif args.ui == "web":
            run_web_mode(args, session)
        elif args.ui == "both":
            run_web_mode(args, session, also_gl=True)
    finally:
        session.finish_collection()
        for path in session.collected:
            print(f"[teleop-ui] episode {path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
