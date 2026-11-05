"""The Newton-native teleop UI: ``newton.viewer.ViewerGL`` + an imgui panel.

What the window shows on top of the scene:

* a draggable ``tcp_target`` gizmo (mouse) for the position target -- the
  same command path as the keys and the gamepad (yaw and roll stay on
  the keys / pad);
* CONTACT FORCE ARROWS at every contacting pair, coloured by alarm level
  (green / amber / red), drawn the way the force is applied to the object;
* a side panel: scene, guard, speed, the six force channels as bars with
  their warn / critical marks, penetration growth, tracking error, the
  task's own readouts, the alarm log, and the control map;
* a banner across the top of the viewport whenever any channel is above
  its warning level, red and pulsing when critical;
* Newton's own live plots (``log_scalar``) for the pad, hand and object
  forces and the penetration growth.

Runs headless too (``--headless``): the same frame, panel included, is
rendered through EGL and written to PNG, which is how the UI is verified
on a machine without a display.
"""

from __future__ import annotations

import os
import time

import numpy as np
import warp as wp


def _prepare_headless():
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
    import pyglet
    pyglet.options["headless"] = True


LEVEL_RGB = {"ok": (0.35, 0.85, 0.45), "warn": (1.0, 0.78, 0.15), "critical": (1.0, 0.25, 0.25)}
LEVEL_TXT = {"ok": (0.75, 0.9, 0.78, 1.0), "warn": (1.0, 0.82, 0.3, 1.0), "critical": (1.0, 0.4, 0.4, 1.0)}
CHANNEL_ORDER = ("pad_L", "pad_R", "hand", "object", "mech", "penetration", "speed", "track")
CHANNEL_NAMES = {"pad_L": "pad L", "pad_R": "pad R", "hand": "arm/hand", "object": "robot->object",
                 "mech": "mechanism", "penetration": "overlap growth", "speed": "body speed",
                 "track": "tracking lead"}


class LocalTeleopUI:
    def __init__(self, session, headless: bool = False, width: int = 1600, height: int = 950,
                 gamepad: bool = True, viewer=None, newton_stats: bool = False):
        self.session = session
        self.headless = headless
        if headless:
            _prepare_headless()
        import newton.viewer
        if viewer is None:
            viewer = newton.viewer.ViewerGL(width=width, height=height, headless=headless, vsync=False)
        self.viewer = viewer
        if headless and getattr(viewer, "gui", None) is None:
            # ViewerGL skips the GUI when headless; the EGL context can carry it
            from newton._src.viewer.viewer_gui import ViewerGui
            try:
                viewer.gui = ViewerGui(viewer, viewer.renderer.window)
            except Exception as exc:
                print(f"[teleop-ui] headless imgui unavailable: {exc}")
        from force_bench.teleop.inputs import KeyboardInput, PygletGamepad
        gui = getattr(viewer, "gui", None)
        self._claim_keyboard(gui)
        self.keyboard = KeyboardInput(viewer.is_key_down,
                                      capturing=(gui.is_keyboard_capturing if gui is not None else None))
        self.gamepad = PygletGamepad() if (gamepad and not headless) else None
        self.spacemouse = None            # set by attach_spacemouse()
        self._sm_state = None
        self._web_active = False
        self._web_lost_t = -1e9
        self.scripted = None
        self.web = None            # a web_ui.WebBridge when --ui both
        self.viewer.set_model(session.model)
        self._place_camera()
        self.target_tf = wp.transform(wp.vec3(*session.cmd[:3]), wp.quat(*session._quat(session.cmd)))
        self._gizmo_prev = False
        self._banner_t = 0.0
        self.show_arrows = True
        self.show_contacts = False
        self.status = ""
        self._telemetry = {}
        self._arrows_np = (None, None, None)
        self.panel_width = 420.0
        self.newton_stats = newton_stats
        if not newton_stats and getattr(viewer, "gui", None) is not None \
                and hasattr(viewer.gui, "_render_stats_overlay"):
            # Newton's "Performance Stats" overlay is pinned to the top-right
            # corner where the teleop panel lives and draws on top of it; the
            # panel carries the frame rate itself
            viewer.gui._render_stats_overlay = lambda: None
        if hasattr(viewer, "register_ui_callback"):
            viewer.register_ui_callback(self._panel_window, position="free")
            viewer.register_ui_callback(self._banner, position="free")

    def _claim_keyboard(self, gui):
        """Newton's viewer binds W/A/S/D/Q/E (camera fly), SPACE (pause
        toggle), H (hide the UI) and F (frame the model) -- the teleop keys.
        ``ViewerGL.on_key_press`` and ``_update_camera`` look these GUI
        methods up at call time, so replacing them here takes the keys
        over: ESC still closes, the ARROW keys still fly the camera, and
        everything else reaches the arm only."""
        if gui is None:
            return
        try:
            import pyglet
            key = pyglet.window.key
            orig_press = gui.handle_key_press

            def handle_key_press(symbol, close_fn=None):
                if symbol == key.ESCAPE:
                    return orig_press(symbol, close_fn)
                return None

            orig_cam = gui.update_camera_from_keys
            arrows = {key.LEFT, key.RIGHT, key.UP, key.DOWN, key.PAGEUP, key.PAGEDOWN}
            cam_map = {key.LEFT: key.A, key.RIGHT: key.D, key.UP: key.W, key.DOWN: key.S,
                       key.PAGEUP: key.E, key.PAGEDOWN: key.Q}

            def update_camera_from_keys(dt, is_key_down):
                # feed the original the arrow keys in place of the letters
                def arrow_only(k):
                    for a, letter in cam_map.items():
                        if k == letter:
                            return bool(is_key_down(a))
                    return bool(is_key_down(k)) if k in arrows else False
                return orig_cam(dt, arrow_only)

            gui.handle_key_press = handle_key_press
            gui.update_camera_from_keys = update_camera_from_keys
        except Exception as exc:
            print(f"[teleop-ui] could not take over the viewer keys: {exc}")

    def attach_spacemouse(self, device, cfg):
        """Drive the arm from a 3Dconnexion puck (teleop/spacemouse.py).

        The puck owns the six pose axes and the jaw buttons; the KEYBOARD
        keeps the toggles, because a SpaceMouse Compact has two buttons and
        both are worth more as the jaw than as a guard switch.  So the
        e-stop stays on Space, the guard on T and reset on Backspace."""
        from force_bench.teleop.inputs import InputState
        self.spacemouse = device
        self.spacemouse_cfg = cfg
        self._sm_state = InputState(source="spacemouse")

    # ── camera ────────────────────────────────────────────────────────
    def _place_camera(self):
        sc = self.session.scene
        pos = np.asarray(sc.cam_pos, float)
        d = np.asarray(sc.cam_target, float) - pos
        d /= max(np.linalg.norm(d), 1e-9)
        pitch = float(np.degrees(np.arcsin(np.clip(d[2], -1, 1))))
        yaw = float(np.degrees(np.arctan2(d[1], d[0])))
        self.viewer.set_camera(pos=wp.vec3(*pos), pitch=pitch, yaw=yaw)
        # Newton's camera flies at 4 m/s, which is sized for room-scale
        # scenes; these are 0.3-0.6 m of table and the camera crosses the
        # whole workspace in a tenth of a second.  Scale it to the scene:
        # one standoff distance per second.
        try:
            self.viewer.camera_speed = float(np.clip(np.linalg.norm(
                np.asarray(sc.cam_target, float) - pos), 0.15, 1.5))
        except Exception:
            pass

    # ── input ─────────────────────────────────────────────────────────
    def poll_input(self):
        ses = self.session
        if self.scripted is not None:
            return self.scripted.poll(ses.t, ses)
        if self.web is not None:
            w = self.web.poll()                      # also applies web commands
            active = not w.stale
            if self._web_active and not active:
                # the browser link dropped: hand the session one stale frame
                # so the lead is released, then fall back to the local devices
                self._web_active = False
                now = time.monotonic()
                if now - self._web_lost_t > 5.0:      # link jitter: one note per 5 s
                    ses._note("web link lost")
                    self._web_lost_t = now
                return w
            self._web_active = active
            if active and (np.abs(w.axes).max() > 0 or any(w.buttons.values())):
                # the web steers, but the local hold / toggles are always honoured
                gp = self.gamepad.poll() if (self.gamepad is not None and self.gamepad.connected) else None
                for loc in (gp, self.keyboard.poll()):
                    if loc is not None:
                        for b, v in loc.buttons.items():
                            w.buttons[b] = w.buttons[b] or v
                return w
        kb = self.keyboard.poll()
        if self.spacemouse is not None:
            from force_bench.teleop.spacemouse import to_input_state
            st = self._sm_state
            st.clear()
            to_input_state(self.spacemouse.poll(), st, self.spacemouse_cfg)
            for b, v in kb.buttons.items():      # the keys keep the toggles
                st.buttons[b] = st.buttons[b] or v
            # the puck IS the source once attached: a released cap must read
            # as zero, not fall through to whatever the keyboard is doing
            return st
        gp = self.gamepad.poll() if (self.gamepad is not None and self.gamepad.connected) else None
        if gp is not None and (np.abs(gp.axes).max() > 0 or any(gp.buttons.values())):
            # gamepad wins while it is being used; keys still toggle things
            for b, v in kb.buttons.items():
                gp.buttons[b] = gp.buttons[b] or v
            return gp
        return kb

    def _gizmo(self):
        ses = self.session
        v = self.viewer
        using = bool(getattr(v, "gizmo_is_using", False))
        if using:
            p = wp.transform_get_translation(self.target_tf)
            ses.set_target(pos=(float(p[0]), float(p[1]), float(p[2])))
        else:
            self.target_tf = wp.transform(wp.vec3(*ses.raw[:3]), wp.quat(*ses._quat(ses.raw)))
        self._gizmo_prev = using

    # ── frame ─────────────────────────────────────────────────────────
    def frame(self):
        ses = self.session
        inp = self.poll_input()
        # should_step() also consumes a single-step request: call it once
        stepping = self.viewer.should_step() if hasattr(self.viewer, "should_step") else True
        ses.viewer_paused = not stepping
        ses.apply_input(inp)
        self._gizmo()
        if stepping:
            ses.step()
        self._telemetry = ses.telemetry()
        if self.web is not None:
            self.web.publish(self._telemetry)
        self.render()

    def run_scripted(self, frames: int, script) -> list:
        """Visible window driven by a scripted operator (demos)."""
        self.scripted = script
        period = self.session.dt
        for _ in range(int(frames)):
            if not self.viewer.is_running():
                break
            t0 = time.perf_counter()
            self.frame()
            if self.session.cfg.realtime:
                el = time.perf_counter() - t0
                if el < period:
                    time.sleep(period - el)
        self.scripted = None
        return []

    def render(self):
        ses = self.session
        v = self.viewer
        tm = self._telemetry
        v.begin_frame(ses.t)
        if hasattr(v, "log_gizmo"):
            # translation only: yaw / roll stay on the keys and the pad
            v.log_gizmo("tcp_target", self.target_tf, rotate=())
        v.log_state(ses.rig.s0)
        if self.show_contacts:
            v.log_contacts(ses.rig.contacts, ses.rig.s0)
        self._log_arrows(tm.get("arrows", []))
        v.end_frame()

    def _log_arrows(self, arrows):
        v = self.viewer
        if not self.show_arrows or not arrows:
            v.log_arrows("contact_forces", None, None, None)
            return
        s = wp.array([wp.vec3(*a["s"]) for a in arrows], dtype=wp.vec3)
        e = wp.array([wp.vec3(*a["e"]) for a in arrows], dtype=wp.vec3)
        c = wp.array([wp.vec3(*LEVEL_RGB[a["level"]]) for a in arrows], dtype=wp.vec3)
        v.log_arrows("contact_forces", s, e, c)

    # ── imgui ─────────────────────────────────────────────────────────
    def _bar(self, imgui, name, ch):
        val = float(ch.get("value", 0.0) or 0.0)
        crit = float(ch.get("critical", 1.0) or 1.0)
        warn = float(ch.get("warn", 0.5) or 0.5)
        span = max(crit * 1.25, 1e-6)
        frac = float(np.clip(val / span, 0.0, 1.0))
        col = LEVEL_RGB[ch.get("level", "ok")]
        imgui.push_style_color(imgui.Col_.plot_histogram, imgui.ImVec4(col[0], col[1], col[2], 0.9))
        imgui.text_colored(imgui.ImVec4(*LEVEL_TXT[ch.get("level", "ok")]),
                           f"{CHANNEL_NAMES.get(name, name):13s}")
        imgui.same_line()
        pos = imgui.get_cursor_screen_pos()
        width = 150.0
        imgui.progress_bar(frac, imgui.ImVec2(width, 16), f"{val:.2f} {ch.get('unit', '')}")
        dl = imgui.get_window_draw_list()
        for lim, colr in ((warn, (1.0, 0.8, 0.2, 0.9)), (crit, (1.0, 0.3, 0.3, 0.9))):
            x = pos.x + width * float(np.clip(lim / span, 0.0, 1.0))
            dl.add_line(imgui.ImVec2(x, pos.y), imgui.ImVec2(x, pos.y + 16),
                        imgui.color_convert_float4_to_u32(imgui.ImVec4(*colr)), 2.0)
        imgui.pop_style_color()
        det = (ch.get("detail") or "").split(",")[0].strip()
        if det:
            imgui.same_line()
            imgui.text_colored(imgui.ImVec4(0.7, 0.7, 0.75, 1.0), det[:20])

    def _panel_window(self, imgui):
        """The teleop panel as its own window docked at the right edge, so it
        never competes with Newton's left panel for space."""
        vp = imgui.get_main_viewport()
        w = self.panel_width
        imgui.set_next_window_pos(imgui.ImVec2(vp.pos.x + vp.size.x - w - 8, vp.pos.y + 8),
                                  imgui.Cond_.first_use_ever)
        imgui.set_next_window_size(imgui.ImVec2(w, vp.size.y - 16), imgui.Cond_.first_use_ever)
        imgui.push_style_color(imgui.Col_.window_bg, imgui.ImVec4(0.08, 0.09, 0.11, 0.92))
        if imgui.begin("Teleop", None, imgui.WindowFlags_.no_collapse)[0]:
            self._panel(imgui)
        imgui.end()
        imgui.pop_style_color()

    def _sparklines(self, imgui):
        h = self.session.history()
        if len(h["t"]) < 2:
            return
        thr = self.session.alarms.cfg.thresholds
        for key, label in (("pad_L", "pad L N"), ("pad_R", "pad R N"), ("hand", "hand N"),
                           ("object", "object N"), ("penetration", "overlap mm")):
            arr = np.asarray(h[key], dtype=np.float32)
            t = thr[key]
            top = float(max(t.critical * 1.25, arr.max() * 1.05, 1e-3))
            lvl = "critical" if arr[-1] >= t.critical else ("warn" if arr[-1] >= t.warn else "ok")
            c = LEVEL_RGB[lvl]
            imgui.push_style_color(imgui.Col_.plot_lines, imgui.ImVec4(c[0], c[1], c[2], 1.0))
            imgui.plot_lines(f"##{key}", arr, 0, f"{label} {arr[-1]:.2f}  (warn {t.warn:g} / crit {t.critical:g})",
                             0.0, top, imgui.ImVec2(self.panel_width - 30, 42))
            imgui.pop_style_color()

    def _panel(self, imgui):
        ses = self.session
        tm = self._telemetry
        if not tm:
            return
        al = tm["alarms"]
        lvl = al["level"]
        imgui.text_colored(imgui.ImVec4(*LEVEL_TXT[lvl]), f"TELEOP  {tm['scene']}   [{lvl.upper()}]")
        imgui.text(f"t={tm['t']:.1f}s  sim {tm['fps_wall']:.0f} fps (x{tm['realtime']:.2f} realtime)"
                   f"  {'graph' if tm['graph'] else 'eager'}  contacts {tm['forces']['n_contacts']}")
        if tm.get("diverged"):
            imgui.text_colored(imgui.ImVec4(1, 0.3, 0.3, 1), "SOLVER DIVERGED -- press Backspace / Reset")
        elif tm.get("paused"):
            imgui.text_colored(imgui.ImVec4(1, 0.8, 0.3, 1),
                               "PAUSED (viewer)" if tm.get("viewer_paused") else "PAUSED (P)")
        elif tm.get("hold"):
            imgui.text_colored(imgui.ImVec4(1, 0.8, 0.3, 1), "HOLD (space) -- command frozen")
        imgui.separator()
        changed, en = imgui.checkbox("Guard (T)", ses.guard.enabled)
        if changed:
            ses.guard.enabled = bool(en)
        imgui.same_line()
        if imgui.button("Reset (Backspace)"):
            ses.reset()
        imgui.same_line()
        if imgui.button("Pause" if not ses.paused else "Run"):
            ses.paused = not ses.paused
        changed, sp = imgui.slider_float("robot speed x", ses.speed_scale, 0.1, 3.0, "%.2f")
        if changed:
            ses.speed_scale = float(sp)
        cs = float(getattr(self.viewer, "camera_speed", 1.0) or 1.0)
        changed, cs2 = imgui.slider_float("camera m/s", cs, 0.05, 4.0, "%.2f")
        if changed:
            self.viewer.camera_speed = float(cs2)
        _, self.show_arrows = imgui.checkbox("force arrows", self.show_arrows)
        imgui.same_line()
        _, self.show_contacts = imgui.checkbox("contact points", self.show_contacts)
        g = tm["guard"]
        clips = [k.replace("_clipped", "") for k, v in g["clips"].items() if v]
        imgui.text(f"zone {g['zone'] or '-'}  feature {g['feature_mm']:.2f} mm  "
                   f"vmax {g['vmax_mm_s']:.0f} mm/s  lead<= {g['lead_mm']:.1f} mm")
        imgui.text(f"clips: {' '.join(clips) if clips else '-'}")
        c = tm["cmd"]
        p = tm["tcp"]
        imgui.text(f"cmd  {c['x']:+.3f} {c['y']:+.3f} {c['z']:+.3f}  yaw {c['yaw_deg']:+.0f} "
                   f"pitch {c.get('pitch_deg', 0):+.0f} roll {c['roll_deg']:+.0f}")
        imgui.text(f"tcp  {p['x']:+.3f} {p['y']:+.3f} {p['z']:+.3f}  lead {tm['track_mm']:.1f} mm  "
                   f"jaw {c['jaw_mm']:.1f} mm")
        imgui.separator()
        imgui.text("contact forces")
        for name in CHANNEL_ORDER:
            self._bar(imgui, name, al["channels"].get(name, {}))
        pen = tm["penetration"]
        imgui.text_colored(imgui.ImVec4(0.7, 0.7, 0.75, 1.0),
                           f"worst overlap {pen['abs_mm']:.2f} mm (rest {pen['rest_max_mm']:.2f}) {pen['abs_pair'][:34]}")
        pb = tm["forces"].get("pad_by", {})
        for side in ("L", "R"):
            if pb.get(side):
                imgui.text_colored(imgui.ImVec4(0.7, 0.7, 0.75, 1.0),
                                   f"pad {side}: " + ", ".join(f"{k} {v:.1f}N" for k, v in pb[side].items())[:60])
        imgui.separator()
        if imgui.collapsing_header("last 12 s", imgui.TreeNodeFlags_.default_open):
            self._sparklines(imgui)
        if tm["readouts"]:
            imgui.separator()
            if imgui.collapsing_header("task readouts", imgui.TreeNodeFlags_.default_open):
                for k, v in list(tm["readouts"].items())[:14]:
                    imgui.text(f"{k:22s} {v}")
        imgui.separator()
        if imgui.collapsing_header("alarm log", imgui.TreeNodeFlags_.default_open):
            ev = list(tm.get("events", []))[-8:]
            if not ev:
                imgui.text_colored(imgui.ImVec4(0.6, 0.6, 0.65, 1.0), "no events")
            for e in reversed(ev):
                col = LEVEL_TXT.get(e.get("level", "ok"), (0.8, 0.8, 0.8, 1.0))
                imgui.text_colored(imgui.ImVec4(*col),
                                   f"{e['t']:6.2f}s {e['level']:8s} {e['channel']:11s} "
                                   f"{e['value']:.2f}{e['unit']} {str(e.get('detail', ''))[:26]}")
            cnt = al["counts"]
            imgui.text(f"warn {cnt['warn']}   critical {cnt['critical']}   "
                       f"guard clips R{g['counts'].get('rate_clipped', 0)} L{g['counts'].get('lead_clipped', 0)} "
                       f"G{g['counts'].get('grip_clipped', 0)} P{g['counts'].get('press_servo', 0)}")
        if imgui.collapsing_header("controls"):
            src = tm["input"].get("source", "-")
            gp = self.gamepad
            sm = self.spacemouse
            imgui.text(f"input: {src}   gamepad: "
                       f"{(gp.name or 'connected') if (gp and gp.connected) else 'none'}"
                       f"   spacemouse: {sm.name if sm is not None else 'none'}")
            for line in ("W/S fwd/back  A/D left/right  R/F up/down",
                         "Q/E yaw  C/V pitch  Z/X roll  G/H close/open  SHIFT fine",
                         "T guard  SPACE hold  BACKSPACE reset  P pause  [ ] speed",
                         "pad: L stick move, R stick up/yaw, LT/RT jaw,",
                         "     A guard  B hold  Y reset  X pause  LB/RB speed",
                         "mouse: drag the tcp_target gizmo (position)",
                         "camera: arrows = pan, PgUp/PgDn = up/down,",
                         "        shift+middle-drag = pan, middle = orbit,",
                         "        left = look, wheel = zoom, ESC quits"):
                imgui.text(line)

    def _banner(self, imgui):
        tm = self._telemetry
        if not tm:
            return
        lvl = tm["alarms"]["level"]
        if lvl == "ok" and not tm.get("diverged") and not tm.get("hold"):
            return
        vp = imgui.get_main_viewport()
        text = tm["headline"] if lvl != "ok" else ("SOLVER DIVERGED" if tm.get("diverged") else "HOLD")
        if lvl == "critical" or tm.get("diverged"):
            pulse = 0.6 + 0.4 * np.sin(time.time() * 8.0)
            bg = (0.75 * pulse, 0.05, 0.05, 0.92)
        elif lvl == "warn":
            bg = (0.7, 0.5, 0.05, 0.9)
        else:
            bg = (0.2, 0.35, 0.6, 0.9)
        w = min(vp.size.x * 0.7, 900.0)
        imgui.set_next_window_pos(imgui.ImVec2(vp.pos.x + (vp.size.x - w) * 0.5, vp.pos.y + 12))
        imgui.set_next_window_size(imgui.ImVec2(w, 0))
        imgui.push_style_color(imgui.Col_.window_bg, imgui.ImVec4(*bg))
        flags = (imgui.WindowFlags_.no_decoration | imgui.WindowFlags_.no_inputs
                 | imgui.WindowFlags_.no_saved_settings | imgui.WindowFlags_.no_nav
                 | imgui.WindowFlags_.no_move | imgui.WindowFlags_.always_auto_resize)
        if imgui.begin("##teleop_banner", None, flags)[0]:
            imgui.text_colored(imgui.ImVec4(1, 1, 1, 1), text)
            recent = [e for e in tm["alarms"].get("recent", []) if e["level"] != "ok"][-2:]
            for e in recent:
                imgui.text_colored(imgui.ImVec4(1, 0.95, 0.9, 1),
                                   f"{e['t']:.1f}s {e['channel']} {e['value']:.2f}{e['unit']} {e['detail'][:40]}")
        imgui.end()
        imgui.pop_style_color()

    # ── loops ─────────────────────────────────────────────────────────
    def run(self):
        ses = self.session
        v = self.viewer
        period = ses.dt
        while v.is_running():
            t0 = time.perf_counter()
            self.frame()
            if ses.cfg.realtime:
                el = time.perf_counter() - t0
                if el < period:
                    time.sleep(period - el)
        v.close()

    def run_headless(self, frames: int, script, out_dir: str, snapshot_every: int = 60,
                     render_ui: bool = True, snapshot_on_alarm: bool = True) -> list:
        """Scripted operator, PNGs of the full window (panel included) every
        ``snapshot_every`` frames, at the first alarm of each level, and at
        the end."""
        from PIL import Image
        os.makedirs(out_dir, exist_ok=True)
        self.scripted = script
        saved = []
        seen = set()
        for i in range(int(frames)):
            self.frame()
            lvl = self.session.alarms.level
            want = (snapshot_every and i % snapshot_every == 0) or i == frames - 1 or \
                (snapshot_on_alarm and lvl != "ok" and lvl not in seen)
            if want:
                seen.add(lvl)
                img = self.viewer.get_frame(render_ui=render_ui).numpy()
                p = os.path.join(out_dir, f"gl_{i:05d}_{lvl}.png")
                Image.fromarray(img).save(p)
                saved.append(p)
        self.scripted = None
        return saved

    def close(self):
        try:
            self.viewer.close()
        except Exception:
            pass
