"""Record a teleop session: the GL window and the web view, as video.

    force-bench teleop-ui --record --scene plug_in_usb_2     # outputs/teleop_ui/plug_in_usb_2/record/
    force-bench teleop-ui --record-all                        # every verified scene

One pass per scene drives the ``demo`` operator (approach to touch, grasp,
lift and carry until the contact force says the part is held back, set
down, release) through the guard and writes, from the SAME run:

    gl_<script>.mp4        the Newton GL window rendered offscreen: 3D force
                           arrows, the teleop panel (gauges, sparklines, alarm
                           log) and the alarm banner -- what the local operator sees
    camera_<script>.mp4    the web view: the streaming camera with the projected
                           force arrows and a drawn gauge panel -- what the browser
                           operator sees
    frames/*.png           key frames: start, first contact, first warning, first
                           critical, peak pad force, end (both views)
    contact_sheet.png      those key frames on one sheet
    summary.json           peaks, alarm events, frame rate, file list
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

import numpy as np

LEVEL_RGB = {"ok": (79, 208, 122), "warn": (255, 198, 58), "critical": (255, 77, 77)}
CHANNELS = ("pad_L", "pad_R", "hand", "object", "mech", "penetration", "speed", "track")
NAMES = {"pad_L": "pad L", "pad_R": "pad R", "hand": "arm/hand", "object": "robot->object",
         "mech": "mechanism", "penetration": "overlap growth", "speed": "body speed",
         "track": "tracking lead"}


def _font(size):
    from PIL import ImageFont
    for p in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
              "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"):
        if os.path.exists(p):
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


class PanelPainter:
    """The web page's right-hand panel, drawn with PIL next to the camera."""

    def __init__(self, session, width: int = 320, height: int = 600):
        self.session = session
        self.w, self.h = width, height
        self.f11, self.f12, self.f14 = _font(11), _font(12), _font(14)

    def paint(self, tm: dict):
        from PIL import Image, ImageDraw
        im = Image.new("RGB", (self.w, self.h), (28, 31, 39))
        d = ImageDraw.Draw(im)
        dim, fg = (154, 160, 173), (230, 232, 238)
        al = tm["alarms"]
        lvl = al["level"]
        y = 8
        d.text((10, y), f"{tm['scene']}", fill=fg, font=self.f14)
        d.text((10, y + 18), f"t {tm['t']:.1f}s   {tm['fps_wall']:.0f} fps   {'graph' if tm['graph'] else 'eager'}",
               fill=dim, font=self.f11)
        g = "guard ON" if tm["guard"]["enabled"] else "guard OFF"
        gc = LEVEL_RGB["ok"] if tm["guard"]["enabled"] else LEVEL_RGB["critical"]
        d.text((200, y), g, fill=gc, font=self.f12)
        d.text((200, y + 18), f"[{lvl.upper()}]", fill=LEVEL_RGB[lvl], font=self.f12)
        y += 44
        d.text((10, y), "CONTACT FORCES", fill=dim, font=self.f11)
        y += 16
        for k in CHANNELS:
            ch = al["channels"].get(k, {})
            val = float(ch.get("value", 0.0) or 0.0)
            warn, crit = float(ch.get("warn", 1)), float(ch.get("critical", 2))
            span = max(crit * 1.25, 1e-6)
            col = LEVEL_RGB.get(ch.get("level", "ok"), LEVEL_RGB["ok"])
            d.text((10, y), NAMES[k], fill=dim, font=self.f11)
            x0, x1 = 100, 240
            d.rectangle([x0, y + 1, x1, y + 12], fill=(38, 42, 53))
            d.rectangle([x0, y + 1, x0 + (x1 - x0) * min(1.0, val / span), y + 12], fill=col)
            for lim, c in ((warn, LEVEL_RGB["warn"]), (crit, LEVEL_RGB["critical"])):
                xx = x0 + (x1 - x0) * min(1.0, lim / span)
                d.line([xx, y + 1, xx, y + 12], fill=c, width=2)
            d.text((246, y), f"{val:.1f} {ch.get('unit', '')}", fill=col if ch.get("level") != "ok" else fg,
                   font=self.f11)
            y += 17
        pb = tm["forces"].get("pad_by", {})
        for side in ("L", "R"):
            if pb.get(side):
                d.text((10, y), f"pad {side}: " + ", ".join(f"{k} {v:.1f}N" for k, v in list(pb[side].items())[:2]),
                       fill=dim, font=self.f11)
                y += 14
        y += 6
        d.text((10, y), "LAST 10 S", fill=dim, font=self.f11)
        y += 14
        h = self.session.history()
        for key, colr in (("pad_L", (79, 208, 122)), ("pad_R", (143, 224, 168)), ("penetration", (255, 198, 58))):
            vals = list(h[key])[-600:]
            thr = self.session.alarms.cfg.thresholds[key]
            top = max(thr.critical * 1.25, (max(vals) if vals else 0.0) * 1.05, 1e-3)
            d.rectangle([10, y, self.w - 10, y + 36], fill=(26, 29, 37))
            for lim, c in ((thr.warn, (120, 100, 30)), (thr.critical, (120, 40, 40))):
                yy = y + 36 - 36 * min(1.0, lim / top)
                d.line([10, yy, self.w - 10, yy], fill=c)
            if len(vals) > 1:
                pts = [(10 + (self.w - 20) * i / 599, y + 36 - 36 * min(1.0, v / top)) for i, v in enumerate(vals)]
                d.line(pts, fill=colr, width=2)
            d.text((14, y + 1), f"{NAMES[key]} {vals[-1] if vals else 0:.2f} {thr.unit}", fill=colr, font=self.f11)
            y += 40
        y += 4
        c, p = tm["cmd"], tm["tcp"]
        d.text((10, y), f"cmd  {c['x']:+.3f} {c['y']:+.3f} {c['z']:+.3f}  yaw {c['yaw_deg']:+.0f}  jaw {c['jaw_mm']:.1f}mm",
               fill=fg, font=self.f11)
        d.text((10, y + 14), f"tcp  {p['x']:+.3f} {p['y']:+.3f} {p['z']:+.3f}  lead {tm['track_mm']:.1f} mm",
               fill=fg, font=self.f11)
        gz = tm["guard"]
        clips = [k.replace("_clipped", "") for k, v in gz["clips"].items() if v]
        d.text((10, y + 28), f"zone {gz['zone'] or '-'}  feat {gz['feature_mm']:.2f}mm  clips {' '.join(clips) or '-'}",
               fill=dim, font=self.f11)
        y += 48
        d.text((10, y), "ALARM LOG", fill=dim, font=self.f11)
        y += 14
        for e in [e for e in tm.get("events", []) if e.get("level") != "ok"][-6:][::-1]:
            colr = LEVEL_RGB.get(e.get("level", "ok"), dim) if e.get("level") in LEVEL_RGB else dim
            d.text((10, y), f"{e['t']:5.1f}s {e['level'][:4]} {e['channel']:11s} {e['value']:.1f}{e['unit']} {str(e.get('detail', ''))[:14]}",
                   fill=colr, font=self.f11)
            y += 14
            if y > self.h - 16:
                break
        return im


def _writer(path, fps):
    import imageio.v2 as imageio
    return imageio.get_writer(str(path), fps=fps, codec="libx264", macro_block_size=8,
                              pixelformat="yuv420p", ffmpeg_params=["-crf", "18", "-preset", "medium"])


def record_scene(name: str, out_dir: Path, script_name: str = "demo", gl: bool = True, cam: bool = True,
                 video_fps: int = 30, gl_size=(1520, 912), cam_size=(800, 600), guard: bool = True,
                 max_s: float | None = None) -> dict:
    import warp as wp
    from PIL import Image
    from force_bench.teleop.alarms import AlarmCfg
    from force_bench.teleop.camera import TeleopCamera
    from force_bench.teleop.inputs import script_by_name
    from force_bench.teleop.scenes import load_scene
    from force_bench.teleop.session import SessionCfg, TeleopSession
    from force_bench.teleop.verify import annotate
    wp.init()
    out_dir = Path(out_dir)
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    scene = load_scene(name)
    acfg = AlarmCfg()
    acfg.scale_pads(scene.finger_effort_N)
    session = TeleopSession(scene, SessionCfg(realtime=False, guard=guard, alarms=acfg))
    session.settle()
    script = script_by_name(script_name)
    ui = None
    if gl:
        os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
        import pyglet
        pyglet.options["headless"] = True
        from force_bench.teleop.local_ui import LocalTeleopUI
        ui = LocalTeleopUI(session, headless=True, width=gl_size[0], height=gl_size[1], gamepad=False)
        ui.scripted = script
    camera = painter = None
    if cam:
        camera = TeleopCamera(session.model, scene.cam_pos, scene.cam_target, cam_size[0], cam_size[1])
        painter = PanelPainter(session, 320, cam_size[1])
    every = max(1, int(round(session.fps / video_fps)))
    w_gl = _writer(out_dir / f"gl_{script_name}.mp4", session.fps / every) if gl else None
    w_cam = _writer(out_dir / f"camera_{script_name}.mp4", session.fps / every) if cam else None
    key = {}                      # tag -> (frame index, gl image, cam image)
    peak_pad, peak_i = -1.0, -1
    last_imgs = (None, None)
    frames = int((max_s or script.duration + 1.0) * session.fps)
    i = 0
    try:
        while i < frames and not (script.done() and i > 30):
            if ui is not None:
                ui.frame()
            else:
                inp = script.poll(session.t, session)
                session.apply_input(inp)
                session.step()
            tm = ui._telemetry if ui is not None else session.telemetry()
            img_gl = img_cam = None
            if i % every == 0:
                if ui is not None:
                    img_gl = ui.viewer.get_frame(render_ui=True).numpy().copy()
                    w_gl.append_data(img_gl)
                if camera is not None:
                    cam_img = annotate(camera.render(session.rig.s0), camera, session,
                                       f"{name}/{script.state.note}")
                    panel = painter.paint(tm)
                    sheet = Image.new("RGB", (cam_img.width + panel.width, cam_img.height))
                    sheet.paste(cam_img, (0, 0))
                    sheet.paste(panel, (cam_img.width, 0))
                    img_cam = np.asarray(sheet)
                    w_cam.append_data(img_cam)
                last_imgs = (img_gl, img_cam)
                L = session.last
                pad = max(L.get("pad_L", 0.0), L.get("pad_R", 0.0))
                lvl = session.alarms.level
                tags = []
                if i == 0:
                    tags.append("start")
                if pad > 0.5 and "first_contact" not in key:
                    tags.append("first_contact")
                if lvl == "warn" and "first_warn" not in key:
                    tags.append("first_warn")
                if lvl == "critical" and "first_critical" not in key:
                    tags.append("first_critical")
                if pad > peak_pad:
                    peak_pad, peak_i = pad, i
                    key["peak_pad"] = (i, img_gl, img_cam)
                for tg in tags:
                    key[tg] = (i, img_gl, img_cam)
            i += 1
            if session.diverged:
                break
    finally:
        if w_gl is not None:
            w_gl.close()
        if w_cam is not None:
            w_cam.close()
    key["end"] = (i - 1, *last_imgs)
    files = {}
    order = ["start", "first_contact", "first_warn", "first_critical", "peak_pad", "end"]
    for tg in order:
        if tg not in key:
            continue
        fi, g, c = key[tg]
        if g is not None:
            p = out_dir / "frames" / f"gl_{tg}_{fi:05d}.png"
            Image.fromarray(g).save(p)
            files[f"gl_{tg}"] = str(p)
        if c is not None:
            p = out_dir / "frames" / f"cam_{tg}_{fi:05d}.png"
            Image.fromarray(c).save(p)
            files[f"cam_{tg}"] = str(p)
    # contact sheet from the camera view (falls back to the GL frames)
    tiles = [(tg, key[tg][2] if key[tg][2] is not None else key[tg][1]) for tg in order if tg in key]
    tiles = [(tg, im) for tg, im in tiles if im is not None]
    if tiles:
        tw = 560
        th = int(tw * tiles[0][1].shape[0] / tiles[0][1].shape[1])
        cols = 3 if len(tiles) > 4 else 2
        rows = (len(tiles) + cols - 1) // cols
        sheet = Image.new("RGB", (cols * tw, rows * (th + 22)), (20, 22, 27))
        from PIL import ImageDraw
        dr = ImageDraw.Draw(sheet)
        f = _font(14)
        for k, (tg, im) in enumerate(tiles):
            x, y = (k % cols) * tw, (k // cols) * (th + 22)
            sheet.paste(Image.fromarray(im).resize((tw, th)), (x, y + 22))
            dr.text((x + 6, y + 4), f"{tg.replace('_', ' ')}  (frame {key[tg][0]}, t={key[tg][0] / session.fps:.1f}s)",
                    fill=(230, 232, 238), font=f)
        sheet.save(out_dir / "contact_sheet.png")
        files["contact_sheet"] = str(out_dir / "contact_sheet.png")
    if ui is not None:
        files["gl_video"] = str(out_dir / f"gl_{script_name}.mp4")
        ui.close()
    if cam:
        files["camera_video"] = str(out_dir / f"camera_{script_name}.mp4")
    rep = session.report()
    summary = {"scene": name, "script": script_name, "guard": guard, "frames": i,
               "sim_s": round(session.t, 2), "wall_s": round(time.time() - t0, 1),
               "video_fps": session.fps / every, "peak_pad_N": round(peak_pad, 2),
               "peak_pad_frame": peak_i, "alarms": rep["alarms"]["counts"],
               "alarm_peaks": rep["alarms"]["peaks"], "events": rep["alarms"]["events"][:40],
               "guard_clamps": rep["guard"]["clamps"], "diverged": session.diverged,
               "graph": session.graph is not None, "files": files}
    with open(out_dir / "summary.json", "w") as f:
        json.dump(summary, f, indent=1, default=float)
    return summary


def main(scenes, out_root: Path, script: str = "demo", **kw) -> int:
    rows = []
    for n in scenes:
        print(f"\n=== teleop-ui record: {n} ({script}) ===", flush=True)
        try:
            s = record_scene(n, out_root / n / "record", script_name=script, **kw)
            print(f"{n:24s} {s['frames']} frames {s['sim_s']}s sim in {s['wall_s']}s  peak pad {s['peak_pad_N']} N  "
                  f"alarms {s['alarms']}  -> {s['files'].get('gl_video', '')} {s['files'].get('camera_video', '')}",
                  flush=True)
            rows.append(s)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            rows.append({"scene": n, "error": f"{type(exc).__name__}: {exc}"})
    with open(out_root / "record_summary.json", "w") as f:
        json.dump(rows, f, indent=1, default=float)
    return 0 if all("error" not in r for r in rows) else 1
