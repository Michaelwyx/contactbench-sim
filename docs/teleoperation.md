# Teleoperation

`force-bench teleop-ui` puts a person in control of the simulated FR3 in any ContactBench task, with live
contact-force feedback and excessive-force alarms. It has two front ends that share one session:

| front end | what it is | input |
|---|---|---|
| **Newton GL window** (`--ui gl`, the default) | `newton.viewer.ViewerGL` with an imgui panel, 3D force arrows, a top banner and live plots | keyboard, 3Dconnexion SpaceMouse, gamepad, the mouse `tcp_target` gizmo |
| **browser page** (`--ui web`) | one HTTP + WebSocket port serving a page with a JPEG stream of an offscreen camera, projected force arrows, gauges, sparklines, an alarm banner and beep, the task's readouts and an alarm log | keyboard, the browser Gamepad API, mouse orbit |
| both (`--ui both`) | the GL window and the page on the same session; the browser's input takes over while it is active | |
| none (`--ui none`) | a headless scripted session that writes a report and camera snapshots | |

Between every input device and the robot sits the **teleoperation guard** ([teleop_guard.md](teleop_guard.md)), which
limits speed near contact, filters the input and caps the squeeze. `T` toggles it live.

![The Newton GL teleoperation window](media/teleop/interfaces/newton_gl.png)

The browser page: [screenshot](media/teleop/interfaces/browser.png). Both show the `playground`, a sandbox scene with
a card reader, a free card and a charger plug.

## Running it

```bash
pip install -e "packages/force-bench[teleop]"       # pyglet, imgui-bundle (Newton's viewer) and websockets
force-bench teleop-ui --list-scenes                                  # the ContactBench tasks, then the playground
force-bench teleop-ui --scene plug_in_usb_2                          # GL window
force-bench teleop-ui --scene plug_in_usb_2 --ui web                 # then open http://127.0.0.1:8765/
force-bench teleop-ui --scene plug_in_usb_2 --ui web --host 0.0.0.0  # reachable from another machine
force-bench teleop-ui --scene turn_doorknob_open_door_flat --ui both
force-bench teleop-ui --scene plug_in_usb_2 --input spacemouse       # a 3Dconnexion SpaceMouse
force-bench teleop-ui --scene plug_in_usb_2 --seed 3                 # a random start, as force-bench run --seed
force-bench teleop-ui --scene turn_dial_to_target --set "Object geometry=Round knob"   # a variation, as --set
force-bench teleop-ui --scene retrieve_utensil_from_bin --embodiment finray   # task 2, Fin Ray fingers
force-bench teleop-ui --scene playground --no-guard                  # start with the guard off (alarms keep running)
force-bench teleop-ui --scene plug_in_usb_2 --ui none --script abusive --out outputs/teleop_ui/usb   # headless
force-bench teleop-ui --scene plug_in_usb_2 --collect-dir outputs/datasets/teleop   # record demonstrations (below)
force-bench teleop-ui --verify --scene plug_in_usb_2                 # the acceptance checks (below)
```

The GL window needs a display; the browser page does not. Every artefact goes under `outputs/teleop_ui/`, or under
`TB_OUT_ROOT` when it is set.

### Options

| option | default | effect |
|---|---|---|
| `--scene NAME` | `playground` | a row of `force-bench list`, or `playground` |
| `--list-scenes` | | list the scenes and exit |
| `--seed N`, `--set AXIS=VALUE` (repeatable), `--embodiment E` | | the task's random start, variation and fingers, as in `force-bench run`; not for the playground |
| `--ui gl\|web\|both\|none` | `gl` | the front end |
| `--host`, `--port` | `127.0.0.1`, `8765` | where the browser page is served |
| `--stream WxH`, `--stream-fps` | `800x600`, `30` | the browser stream's resolution and rate (also the camera of `--ui none`) |
| `--width`, `--height` | `1600`, `950` | the GL window's size |
| `--headless` | | GL: render offscreen through EGL, run a scripted operator and write PNG snapshots |
| `--input auto\|keyboard\|gamepad\|spacemouse` | `auto` | the device that drives the arm in the GL window; `auto` takes a SpaceMouse when one opens, otherwise the gamepad and the keyboard |
| `--no-gamepad` | | do not open a gamepad in the GL window |
| `--speed` | `0.15` | full-stick translation rate, m/s |
| `--speed-scale` | `1.0` | initial speed scale (`[` / `]` change it live, 0.1 to 3) |
| `--no-guard` | | start with the guard off; `T` turns it on |
| `--alarm CH=WARN:CRIT` (repeatable) | | override an alarm threshold, e.g. `--alarm pad_L=5:10` ([channels](#what-is-displayed)) |
| `--settle N` | `30` | frames the start pose is held before the operator takes over; the rest overlap of every contact pair is recorded then |
| `--no-graph` | | do not capture the physics frame as a CUDA graph |
| `--substeps N` | the task's | physics substeps per frame (see [Notes and limits](#notes-and-limits)) |
| `--collide-substep 0\|1` | the task's | collide once per frame (0) or before every substep (1) |
| `--no-readouts` | | do not sample the task's own readouts |
| `--no-realtime` | | never wait for the simulation clock; by default the UIs do not run faster than real time |
| `--script NAME` | `gentle`; `demo` for `--record` | a scripted operator instead of a person, in any front end: `hover`, `gentle`, `gentle_held`, `abusive`, `abusive_unguarded` or `demo` |
| `--frames N` | the script's length | stop a scripted or headless run after `N` frames |
| `--out DIR` | `outputs/teleop_ui/<scene>/...` | where a scripted, verification or recording run writes |
| `--snapshot-every N` | `60` | frames between snapshots of a scripted run |
| `--verify`, `--verify-all`, `--quick` | | the acceptance checks ([Verification](#verification)) |
| `--record`, `--record-all`, `--no-gl-video`, `--no-cam-video` | | videos of a scripted session ([Recording](#recording)) |
| `--collect-dir DIR` | | record demonstration episodes ([Recording](#recording)) |
| `--spacemouse-*` | | the SpaceMouse options ([below](#the-spacemouse)) |

The scripted operators: `hover` gives no input; `gentle` closes the jaw slowly until the pads touch, holds and opens;
`gentle_held` is the same for a part that starts in the pinch, without the release; `abusive` plunges to contact at
full speed, crushes, jabs into the work, yanks and shakes; `abusive_unguarded` is `abusive` with the guard switched
off at the start; `demo` approaches until the pads touch, grasps, lifts and carries until the contact force says the
part is held back, sets it down and lets go.

## Tasks

The teleoperation UI drives every **ContactBench task** (`force-bench list`, see [tasks.md](tasks.md)). A task is
captured the way the acceptance gate captures it: its `main()` runs with `Rig.run` intercepted, so the scene is built
exactly as `force-bench run` builds it (assets, mechanisms, drives, collision filters, solver options, start pose,
camera, an object already in the hand) and handed over at the moment the scripted policy would have started. The
operator takes over at the start pose and the scripted policy never runs. Task 2, *Retrieve single utensil from bin*,
has no scripted policy: its bin is filled by physics and handed over with the arm waiting above it.

A task takes the options of `force-bench run`: `--seed N` for a random start, `--set "axis=value"` for a variation
([variations.md](variations.md)) and `--embodiment` for other fingers ([embodiments.md](embodiments.md)). Recorded
episodes keep them in their metadata (`task_args`).

- The task's own readouts (door angle, plug depth, the utensils outside the bin, ...) are sampled every frame and
  shown.
- The guard's speed and lead limits are set region by region from feature clearances: measured clearances where the
  task's scene declares them, 1.5 mm on every other object (the kit's parts have sub-millimetre fits), and the table
  and the floor as hard surfaces ([teleop_guard.md](teleop_guard.md#feature-declarations)).
- A task that starts with the object in the fingers (a USB stick, a screwdriver, a light bulb) keeps the task's own
  pinch: the guard's grip and press ceilings and the pad alarms are raised to the pinch measured at the end of the
  settle.
- Tasks with threads, many contacts or collision every substep run slower than real time; the panel shows the frame
  rate.

`--scene playground` is the sandbox of the screenshots above, the scene of the minimal viewer `force-bench teleop`.

## Controls

Every device produces the same normalized input: six spatial **rate** axes (forward, left, up, yaw, pitch, roll), a
jaw rate, and momentary buttons. Forward points from the robot base toward the work; yaw is about world z, while pitch
and roll are in the tool frame. The guard and the session do not know which device is attached.

| action | keyboard | SpaceMouse | gamepad |
|---|---|---|---|
| forward / back, left / right | `W`/`S`, `A`/`D` | push the cap | left stick |
| up / down | `R`/`F` | pull up / press down | right stick vertical (D-pad up / down: fine) |
| yaw | `Q`/`E` | twist the cap | right stick horizontal |
| pitch the tool's nose up / down | `C`/`V` | tilt the cap | hold right-stick click (R3), then right stick vertical |
| roll about the tool axis (turn a knob) | `Z`/`X` | roll the cap | D-pad left / right |
| close / open the jaw | `G`/`H` | button 1 / button 2 | LT / RT (analog) |
| guard on / off | `T` | (keyboard) | A |
| hold (e-stop while held) | `Space` | (keyboard) | B |
| reset to the start pose | `Backspace` | (keyboard) | Y |
| pause / run | `P` | (keyboard) | X |
| speed scale down / up (0.1–3×) | `[` / `]` | (keyboard) | LB / RB |
| fine (×0.25) | `Shift` | (keyboard) | — |
| position target | drag the `tcp_target` gizmo (GL); yaw and roll stay on the keys and the pad | — | |
| camera | GL: arrow keys pan, PgUp / PgDn move up / down, middle-drag orbits, shift + middle-drag pans, left-drag looks, the wheel zooms; `ESC` quits. Web: drag to orbit, wheel to zoom, shift-drag to pan | | |

In the GL window the teleoperation keys replace Newton's own camera keys (W/A/S/D/Q/E) and its Space / H / F
shortcuts.

How the command behaves:

- **Hold** freezes every axis where the guarded command is. On the first held frame the command retreats to the
  measured TCP, so any press is released; the jaw keeps its grip.
- **Reset** restores the start state, and starts a new episode when recording.
- While **paused**, the sticks integrate nothing, so unpausing does not launch the arm.
- The operator's command never runs more than four guard steps ahead of the guarded command, so releasing a stick
  stops the arm within a few frames. A gizmo drag is walked to at the guard's rate.
- The command is kept inside the scene's workspace and at least 3 mm above the floor. A target the arm cannot reach
  (the inverse kinematics misses it by more than 8 mm) is refused: the command returns to the last reachable pose and
  the `ik` warning says why.
- A browser that sends no input for 0.5 s is treated as disconnected: every axis freezes and the press is released.
  The page sends toggles (guard, reset, pause, speed) once per key press, so a short tap lands exactly once even on a
  slow scene.

## The SpaceMouse

```bash
force-bench teleop-ui --spacemouse-test                          # what each motion does, live
force-bench teleop-ui --scene plug_in_usb_2 --input spacemouse   # drive the arm with it
```

A 3Dconnexion puck is a 6-DoF force sensor: the cap's deflection is read as a **rate**, so the harder you push, the
faster the TCP moves, and the guard governs the result exactly as it does for the keyboard.

| cap motion | robot |
|---|---|
| push right / left | TCP right / left |
| push forward / back | TCP into the work / back toward you |
| pull up / press down | TCP up / down |
| twist | yaw about world +z |
| tilt nose-up / nose-down | the tool's nose pitches up / down |
| roll | the tool rolls about its own approach axis |
| button 1 / button 2 (hold) | close / open the jaw |

Translation, yaw and roll are fixed correspondences. Pitch is applied in the tool frame, so that it still means
"tilt" after a yaw; which tool axis that is, and with which sign, depends on the scene's grasp orientation and robot
placement. The session resolves both from the geometry with one rule: **positive pitch raises the approach
direction**. On a top-down pick that tilts the gripper off the vertical and back toward you; on a horizontal door
reach it lifts the nose above the horizon.

The toggles stay on the keyboard (a SpaceMouse Compact has two buttons, used for the jaw): hold is `Space`, the guard
`T`, reset `Backspace`.

**Backends** (`--spacemouse-backend auto|spnav|hid`). On Linux, prefer `spnav`: the `spacenavd` daemon through
`libspnav` (the `libspnav0` package), with nothing to `pip install`. If the daemon is running it already holds the
device, so a second reader on the raw HID node would get nothing. `hid` reads the HID reports directly, through
`libhidapi` or the `hid` module (`pip install hidapi`), for machines without the daemon; it needs read access to
`/dev/hidraw*` (the `plugdev` group, or a udev rule). Because the daemon connection succeeds whether or not a puck is
plugged in, the UI looks for a 3Dconnexion USB id and says so when none is attached.

**If an axis feels backwards**, `--spacemouse-test` prints the raw counts, the device frame and the robot axes side
by side; flip the wrong one with `--spacemouse-invert` (`x,y,z,rx,ry,rz`, comma separated). Sign conventions differ
between 3Dconnexion models and between the daemon and the raw-HID path, so run this check once per device.

| option | default | effect |
|---|---|---|
| `--spacemouse-invert AXES` | none | flip device axes, e.g. `y,rz` |
| `--spacemouse-expo` | `2.0` | response curve exponent (quadratic; `1.0` is linear) |
| `--spacemouse-deadzone` | `0.06` | fraction of full scale ignored around the centre |
| `--spacemouse-full-scale` | `350` | raw counts that read as full deflection |

The gamepad in the GL window is a pyglet `Controller` (evdev on Linux); on the web page it is the browser's Gamepad
API, with the same mapping table (`teleop/inputs.py` is the source of truth).

## What is displayed

**Force channels.** All are read from the solver's contact arrays after each frame, so they are the forces the
physics resolved. The key is the channel's name for `--alarm` and in `telemetry.csv`.

| channel | key | meaning | warn / critical (default) |
|---|---|---|---|
| `pad L` / `pad R` | `pad_L`, `pad_R` | total contact force on each fingertip, with a breakdown by counterpart body | 6 / 12 N, raised to half / all of the scene's finger force clamp when that is higher (10 / 20 N at the usual 20 N clamp): the drive cannot deliver more than its clamp, so anything above is an impact or a press through the arm; a task that starts with the object in the pinch raises them to 1.25 / 1.5 × that pinch |
| `arm/hand` | `hand` | contact force on every other robot link (the wrist or hand ramming something) | 15 / 40 N; 15 N is the guard's press ceiling |
| `robot→object` | `object` | the largest force the robot puts on one body | 15 / 40 N, raised to the finger clamp / twice the clamp when that is higher; with a start pinch, at least 2.5 / 3 × the pinch |
| `mechanism` | `mech` | the largest object–object pair load **above its rest value** (a plug jammed into its socket, a bolt against its strike) | 20 / 60 N |
| `overlap growth` | `penetration` | the deepest contact-pair overlap **beyond its rest depth** (several interfaces overlap at rest by design) | 1 / 3 mm; 3 mm is the gate's tunnelling threshold |
| `body speed` | `speed` | the fastest frame-to-frame displacement of any body (flying-object detector) | 1 / 2.5 m/s |
| `tracking lead` | `track` | how far the command leads the measured TCP (a blocked or unreachable arm) | 15 / 40 mm |

Boolean channels: solver divergence (critical; the session pauses, and reset also clears MuJoCo's warm-start buffers,
without which a diverged scene diverges again on the first frame), a body leaving the workspace (critical), and an
unreachable or non-finite IK target (`ik`, warning).

Each alarm has hysteresis: it clears below 80% of the threshold that raised it and never within 0.25 s, so a
chattering contact is one event rather than sixty a second. A warning is immediate; the critical level needs the excess
to be **sustained** for 4 consecutive frames (2 for overlap growth and body speed). A position-controlled fingertip
meeting a rigid part produces one-frame impact spikes that no operator can act on; those show in the gauge, the
sparkline and the peak value, while the red banner means a sustained excess. Every rise in level is logged with the
channel, value and counterpart (for example `pad_R critical 15.0 N [static 15.0N]`) and shown in the banner; the web
page beeps on new events. Thresholds are set with `--alarm` and can be edited live on the page.

**Force arrows.** One arrow per contacting shape pair, at the force-weighted contact point, pointing the way the force
acts on the non-robot side, with length ∝ log(1 + F) and coloured by that channel's level. In the GL window they are
3D; on the page they are projected through the streaming camera's pinhole model and drawn over the image.

**The GL panel** shows the scene, the guard's state (zone, local feature size, speed and lead limits, which clamps are
active), the speed scale, every force channel as a bar with its warn and critical marks, the task's readouts, the
alarm log and the control map. A banner across the top of the viewport appears whenever a channel is above its warning
level, red and pulsing when critical, and Newton's live plots show the pad, hand and object forces and the overlap
growth. The browser page shows the same channels as gauges and sparklines, the readouts, the alarm log, a speed slider
and an editor for the thresholds.

## Recording

There are three ways to get data out of a teleoperation session.

- **Demonstrations from a person.** Give `teleop-ui` a directory with `--collect-dir` and every attempt is saved as
  one episode: the FR3 joint targets sent each frame (the actions), the measured joint positions and velocities and
  the poses of the task objects (the observations), the contact forces shown to the operator (left and right finger
  pad, the rest of the robot, the manipulated object, the scene's mechanism pair, in newtons), and the tactile fields
  when the scene runs with `TB_GELSIGHT=1`. Reset (`Backspace`) closes the current episode and starts the next;
  closing the window or stopping the program closes the last one. An attempt without a single step is not saved.

  ```bash
  force-bench teleop-ui --scene plug_in_usb_2 --input spacemouse --collect-dir outputs/datasets/teleop_usb
  force-bench dataset outputs/datasets/teleop_usb             # check what was recorded
  ```

  The episodes have the same format as scripted-policy episodes ([data.md](data.md)), with
  `source: "teleoperation"` and the guard settings, the task options and the session events in their metadata. Task
  success is not judged during teleoperation, so their outcome is `unvalidated`. The minimal viewer records too:
  `force-bench teleop --viewer gl --collect-dir <dir>` saves one episode of the playground scene when the viewer
  closes.
- **Videos of a scripted session.** `force-bench teleop-ui --record --scene plug_in_usb_2` runs the `demo` operator
  through the full UI (`--script` picks another one; the file names follow it). One run writes, under
  `outputs/teleop_ui/<scene>/record/`:

  | file | contents |
  |---|---|
  | `gl_demo.mp4` | the GL window: panel, banner and arrows |
  | `camera_demo.mp4` | the browser view: streaming camera, projected arrows and a gauge panel |
  | `frames/` | key frames of both views: start, first contact, first warning, first critical, peak pad force, end |
  | `contact_sheet.png` | the key frames on one sheet |
  | `summary.json` | peaks, alarm events, frame rate and the file list |

  `--record-all` records the playground and `plug_in_usb_2`. Either command also writes a summary of the scenes it
  recorded to `outputs/teleop_ui/record_summary.json`. `--no-gl-video` and `--no-cam-video` skip one of the videos, and
  `--no-guard` records with the guard off.
- **Session telemetry.** Scripted and verification runs write `report.json` (scene, guard and alarm summaries,
  readouts, session events) and `telemetry.csv` (the force channels above over the last 12 s, one row per frame) to
  the directory given by `--out`, or by default under `outputs/teleop_ui/<scene>/`: `none_<script>/` for `--ui none`,
  `gl_<script>/` for `--headless`, `web_<script>/` for a scripted browser session.

To record the scripted policies of the ContactBench tasks as demonstrations, use `force-bench collect`
([data.md](data.md)).

## Verification

`force-bench teleop-ui --verify --scene X` runs the session under scripted operators and checks what a person would
otherwise judge by eye. `--verify-all` runs it for the playground and `plug_in_usb_2`, and `--quick` halves the
scripts. Results go to `outputs/teleop_ui/<scene>/` (or `--out`): `verify.json`, one `report.json` and `telemetry.csv`
per scripted run, camera PNGs at the first alarm of each level and at the end, and GL screenshots, with a summary of
all verified scenes in `outputs/teleop_ui/verify_summary.json`. The command exits non-zero if a check fails.

| check | criterion |
|---|---|
| `finite` | no run diverged the solver |
| `guard_integrity` | an abusive operator (full-speed plunge to contact, crush, jab, yank, shake) **through the guard** never grew an overlap by more than 3 mm and never launched a body above 2.5 m/s |
| `alarms_detect` | the abuse raised at least one force or overlap alarm, and the script's guard toggle took effect (the unguarded run is the detection control) |
| `gentle_quiet` | a careful operator (closes at 10 mm/s until the pads touch at 2 N, holds, releases; a task that starts with the part in the fingers is held and not released) raised no critical alarm |
| `gl_headless` | the GL window, panel included, rendered offscreen through EGL and is not blank |
| `web_loopback` | the server served the page, streamed images and telemetry to a real WebSocket client, and the client's input moved the command and toggled the guard; if Chrome or Chromium is installed, a headless browser also loads the page against the live server and its screenshot and DOM are checked |

The CPU unit tests (`pytest packages/force-bench/tests/test_teleop_ui.py`) cover alarm hysteresis, the keyboard,
gamepad and SpaceMouse mapping tables, the scripted operators, the projection math and the web protocol.

## Code layout

```
force_bench/teleop/
  app.py              the CLI (force-bench teleop-ui): gl | web | both | none, --verify, --record
  session.py          TeleopSession: guard -> IK + joint-rate limit + reachability -> physics frame
                      (CUDA graph when capturable) -> contact readback -> AlarmMonitor -> telemetry
  scenes.py           task capture (Rig.run interception), guard zones, the playground
  alarms.py           thresholds with hysteresis, the event log, --alarm overrides
  inputs.py           InputState; keyboard / gamepad / scripted sources; the mapping tables
  spacemouse.py       3Dconnexion: spnav and raw-HID backends, the robot mapping, the axis check
  camera.py           offscreen SensorTiledCamera with orbit state and pixel projection
  local_ui.py         the ViewerGL app: imgui panel, banner, arrows, plots, gizmo, headless PNGs
  web_ui.py           WebServer (websockets, HTTP page on the same port), WebBridge
  web/index.html      the page (plain JavaScript, no build step)
  verify.py           the acceptance checks
  record.py           demo recordings: GL-window video, web-view video, key frames
  guard.py            the teleoperation guard (teleop_guard.md)
  gate.py             the acceptance gate and the declared clearances (teleop_guard.md)
  franka_teleop.py    the minimal guarded viewer (force-bench teleop)
```

## Notes and limits

- After the settle, the physics frame is captured as a CUDA graph when the scene allows it (no host reads inside the
  step, an even substep count). The graph replays exactly the eager frame. A scene that cannot be captured runs
  eagerly, and the console says why.
- Speeds: full stick is 0.15 m/s. Near the work the binding limit is the guard's contact-aware governor: 250 mm/s in
  free air and, near a declared clearance, 0.25 × the local feature size per frame. The table and the floor are
  1.5 mm hard-surface zones in every scene (22 mm/s, 3 mm lead), so a fingertip landing on a slab meets it at tens of
  newtons instead of hundreds.
- `--substeps` changes the session's step after the scene has been built; hard joint limits and the mechanisms that
  wrap the solver keep the time step they were built with (the CLI prints a note).
- The finger drives are position servos with a force clamp. The guard's grip limiter holds a steady squeeze at 6 N per
  pad, but a pad closing at 50 mm/s onto a hard part still produces an impact spike, and it is reported as one.
- **The gamepad and SpaceMouse paths are tested with synthetic input only.** The mappings, the response curve and the
  button tables are unit-tested, the browser gamepad path is exercised by the loopback check, and the SpaceMouse
  mapping has been checked in simulation, for a top-down grasp and for a sideways reach, by feeding in each cap
  motion and measuring which way the tool moves. The pyglet controller path and the SpaceMouse readers have not been
  run against physical devices; run `--spacemouse-test` once before relying on a puck.
