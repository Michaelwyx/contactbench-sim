"""Rollout instrumentation: offscreen GPU camera -> mp4, force log -> csv/png,
full state log -> npz for post-hoc QA (see force_bench.qa)."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import numpy as np
import trimesh
import warp as wp

import newton


def lookat_transform(pos, target, up=(0.0, 0.0, 1.0)) -> wp.transformf:
    """CAMERA-TO-WORLD pose for SensorTiledCamera (camera looks along -z)."""
    pos = np.asarray(pos, dtype=np.float64)
    f = np.asarray(target, dtype=np.float64) - pos
    f /= np.linalg.norm(f)
    r = np.cross(f, np.asarray(up, dtype=np.float64))
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    c2w = np.stack([r, u, -f], axis=1)  # columns: right, up, -forward
    q = wp.quat_from_matrix(wp.mat33f(*c2w.flatten().tolist()))
    return wp.transformf(wp.vec3f(*pos.tolist()), q)


class _FrameStore:
    """Indexable frame log that keeps frames JPEG-compressed in RAM.

    A 40 s rollout at 1280x960 is 8+ GB of raw uint8 frames — with several
    task processes running at once that is the machine's memory.  Contact
    sheets only ever read a handful of frames, so they are stored encoded
    (~20x smaller) and decoded on access; the MP4 is written from the raw
    frames as they are captured, so the video itself is not double-encoded.
    """

    def __init__(self, quality: int = 94):
        self._buf: list[bytes] = []
        self.quality = quality

    def append(self, img: np.ndarray):
        import io
        from PIL import Image
        b = io.BytesIO()
        Image.fromarray(img).save(b, "JPEG", quality=self.quality, subsampling=0)
        self._buf.append(b.getvalue())

    def __len__(self):
        return len(self._buf)

    def __getitem__(self, i):
        import io
        from PIL import Image
        if isinstance(i, slice):
            return [self[k] for k in range(*i.indices(len(self._buf)))]
        return np.asarray(Image.open(io.BytesIO(self._buf[i])).convert("RGB"))

    def __iter__(self):
        for i in range(len(self._buf)):
            yield self[i]


# ── studio rendering ────────────────────────────────────────────────────
# The tiled camera renders one hit-shaded sample per pixel with a hard-coded
# background, so image quality comes from three things we control here:
#   * SUPERSAMPLING — render ss x ss samples per output pixel and box-filter
#     them on the GPU (the CPU-side float mean cost 60+ ms/frame; the warp
#     kernel is <1 ms).  This is what removes the stair-stepped silhouettes.
#   * A KEY + FILL light rig instead of the single default light: the fill
#     lifts the shadow side so parts read as solid instead of silhouetting
#     against their own shadow (only the key casts shadows).
#   * A studio BACKDROP composited from the shape-index image (cleared to
#     0xFFFFFFFF where a ray missed everything) — the renderer's own
#     background is a fixed dark slate and ignores clear_color.
# Shapes are recolored by ROLE (see studio_palette): the manipulated bodies
# get saturated accents, the environment stays desaturated, the robot keeps
# its URDF white — so a viewer can see at a glance what is being moved.
BG_TOP = (0.15, 0.17, 0.21)


BG_BOTTOM = (0.42, 0.45, 0.51)


KEY_DIR = (-0.5, 0.6, -0.62)     # light travel direction (renderer negates)


FILL_DIR = (0.72, -0.36, -0.58)


ACCENTS = [
    (0.95, 0.62, 0.16),   # amber
    (0.20, 0.68, 0.72),   # teal
    (0.86, 0.34, 0.42),   # rose
    (0.55, 0.45, 0.80),   # violet
    (0.42, 0.72, 0.33),   # green
    (0.93, 0.82, 0.28),   # yellow
]


NEUTRALS = [
    (0.60, 0.63, 0.68),
    (0.52, 0.56, 0.62),
    (0.67, 0.69, 0.72),
]


@wp.kernel
def _resolve_frame_kernel(
    color: wp.array4d(dtype=wp.uint32),
    sidx: wp.array4d(dtype=wp.uint32),
    out: wp.array3d(dtype=wp.uint8),
    ss: wp.int32,
    src_h: wp.int32,
    bg_top: wp.vec3f,
    bg_bot: wp.vec3f,
    cam: wp.int32,
    x_off: wp.int32,
):
    """Box-filter ss x ss samples and composite the backdrop gradient.

    `cam` selects which tiled-camera slice to read and `x_off` where to write
    it in `out`, so the same kernel resolves a single view (cam=0, x_off=0)
    or tiles several views side by side into one wide frame.
    """
    y, x = wp.tid()
    acc = wp.vec3f(0.0, 0.0, 0.0)
    for dy in range(ss):
        for dx in range(ss):
            sy = y * ss + dy
            sx = x * ss + dx
            if sidx[0, cam, sy, sx] == wp.uint32(0xFFFFFFFF):
                f = wp.float32(sy) / wp.float32(src_h - 1)
                acc = acc + bg_top * (1.0 - f) + bg_bot * f
            else:
                c = color[0, cam, sy, sx]
                acc = acc + wp.vec3f(
                    wp.float32(c & wp.uint32(255)) / 255.0,
                    wp.float32((c >> wp.uint32(8)) & wp.uint32(255)) / 255.0,
                    wp.float32((c >> wp.uint32(16)) & wp.uint32(255)) / 255.0,
                )
    inv = 1.0 / wp.float32(ss * ss)
    for k in range(3):
        v = wp.clamp(acc[k] * inv, 0.0, 1.0) * 255.0 + 0.5
        out[y, x + x_off, k] = wp.uint8(wp.int32(v))


def orbit_camera(cam_pos, cam_target, az_deg: float, lift_frac: float = 0.20):
    """A SECOND camera pose that orbits `cam_pos` about `cam_target`.

    Orbiting about the target is what makes this safe to apply to every task
    unattended: the look-at point is unchanged, so whatever the primary view
    was framed on -- the object being manipulated -- stays centred in the new
    view too.  Only the viewing ANGLE changes.  Radius is preserved so the
    framing scale matches, and the camera is lifted by `lift_frac` of the
    horizontal radius for a 3/4 view that reads depth better than a pure
    side-orbit.

    Degenerate case: a primary camera directly above its target has no
    azimuth to orbit, so fall back to backing off along +x.
    """
    p = np.asarray(cam_pos, dtype=float)
    t = np.asarray(cam_target, dtype=float)
    d = p - t
    r = float(np.hypot(d[0], d[1]))
    if r < 1e-6:                       # top-down primary: nothing to orbit
        r = max(abs(float(d[2])), 1e-3)
        return t + np.array([r, 0.0, float(d[2]) * 0.5])
    a = np.arctan2(d[1], d[0]) + np.radians(az_deg)
    return t + np.array([r * np.cos(a), r * np.sin(a),
                         float(d[2]) + lift_frac * r])


def studio_palette(model: newton.Model) -> None:
    """Recolor shapes by role: accents for moving parts, gray for scenery.

    The robot's URDF colors are left alone (white FR3 reads correctly), the
    ground/table become desaturated neutrals, every non-robot rigid body
    cycles through the accent palette and static asset shapes through muted
    neutrals.  Purely visual — no effect on physics.
    """
    colors = model.shape_color.numpy().copy()
    body_lbl = list(model.body_label)
    shp_lbl = list(model.shape_label)
    sbody = model.shape_body.numpy()
    body_accent: dict[int, tuple] = {}
    for i in range(len(colors)):
        lbl = shp_lbl[i] or ""
        b = int(sbody[i])
        blbl = body_lbl[b] if b >= 0 else ""
        if "fr3" in lbl or "fr3" in blbl or "panda" in blbl:
            continue                                   # robot keeps its skin
        if lbl.startswith("kit/"):
            continue                                   # kit parts keep their print colour (force_bench.kit)
        if "ground" in lbl:
            colors[i] = (0.30, 0.32, 0.36)
        elif "table" in lbl:
            colors[i] = (0.68, 0.64, 0.58)             # warm workbench top
        elif lbl.startswith("cable") or blbl.startswith("cable"):
            colors[i] = (0.13, 0.36, 0.78)             # one blue patch cable,
                                                       # not eight accents
        elif b >= 0:
            if b not in body_accent:
                body_accent[b] = ACCENTS[len(body_accent) % len(ACCENTS)]
            colors[i] = body_accent[b]
        else:
            colors[i] = NEUTRALS[i % len(NEUTRALS)]
    model.shape_color.assign(colors.astype(np.float32))


class RolloutRecorder:
    """Offscreen video + force + state logging for one rollout.

    Video is written incrementally (h264, crf 16) so long rollouts do not
    hold thousands of full-resolution frames in RAM; `frames` still indexes
    every captured frame (JPEG-backed) for contact sheets.
    """

    def __init__(
        self,
        model: newton.Model,
        out_dir: str | Path,
        cam_pos,
        cam_target,
        width: int = 1280,
        height: int = 960,
        fps: int = 60,
        supersample: int | None = None,
        palette: bool = True,
        fov_deg: float = 40.0,
        cam2_pos=None,
        cam2_target=None,
    ):
        from newton.sensors import SensorTiledCamera

        self.model = model
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        # TB_RENDER_FAST=1 renders draft frames (iteration loops); the
        # published rollouts are rendered at the defaults.
        if os.environ.get("TB_RENDER_FAST") == "1":
            width, height, supersample = 800, 600, 1
        self.width, self.height = width, height
        self.ss = int(supersample if supersample is not None else 2)
        rw, rh = width * self.ss, height * self.ss

        # ContactBench kit look (purely visual, keyed by the output folder name): render-only stand-ins are
        # hidden before the camera is built; the kit colours go on at the first captured frame, after any
        # colours a task sets itself (force_bench.kit_look; TB_LOOK=studio turns it off)
        from force_bench import kit_look
        self._look_name = self.out_dir.name
        self._look_pending = True
        kit_look.hide_stand_ins(model, self._look_name)
        self.cam = SensorTiledCamera(model=model)
        self.cam.default_render_config.enable_shadows = True
        self.cam.utils.create_default_light(enable_shadows=True)
        self._light_rig()
        if palette:
            studio_palette(model)
        self.bg_top, self.bg_bottom = BG_TOP, BG_BOTTOM

        # SECOND VIEW.  Derived by orbiting the primary about its own look-at
        # target (see orbit_camera), so it needs no per-task tuning and is
        # guaranteed to still frame whatever the task pointed the camera at.
        # A task can override it explicitly; TB_CAM2_AZ / TB_CAM2_LIFT tune
        # the derived one, and TB_MULTIVIEW=0 turns the whole thing off.
        self.multiview = os.environ.get("TB_MULTIVIEW", "1") != "0"
        if cam2_pos is None and self.multiview:
            cam2_pos = orbit_camera(
                cam_pos, cam2_target if cam2_target is not None else cam_target,
                float(os.environ.get("TB_CAM2_AZ", 72.0)),
                float(os.environ.get("TB_CAM2_LIFT", 0.20)))
        if cam2_target is None:
            cam2_target = cam_target
        self.cam2_pos, self.cam2_target = cam2_pos, cam2_target
        ncam = 2 if self.multiview else 1

        fovs = np.radians([fov_deg] * ncam)
        self.rays = self.cam.utils.compute_camera_rays_pinhole(
            rw, rh, camera_fovs=fovs
        )
        self.color = self.cam.utils.create_color_image_output(rw, rh, ncam)
        self.sidx = self.cam.utils.create_shape_index_image_output(rw, rh, ncam)
        self.out_img = wp.zeros((height, width, 3), dtype=wp.uint8)
        # HOST STAGING BUFFERS, ALLOCATED ONCE.  `out_img.numpy()` on a CUDA
        # array allocates a fresh multi-megabyte host buffer every frame,
        # and with the small long-lived per-frame records (state rows, the
        # JPEG bytes) interleaved between those big transient blocks glibc's
        # heap fragments: the freed frame buffers cannot be trimmed or
        # reused, and RSS grows by about one raw frame per frame (measured
        # 2026-09-11: +2.6 MB/frame at draft res, +7 MB/frame at
        # publication res; a 6735-frame rollout reached 34 GB and was
        # OOM-killed).  Copying into these fixed host arrays
        # and handing the writers VIEWS of them keeps the per-frame heap
        # traffic to the JPEG bytes (~0.1 MB/frame).
        self._img_host = wp.zeros((height, width, 3), dtype=wp.uint8, device="cpu")
        self._bq_host = wp.zeros(model.body_count, dtype=wp.transform, device="cpu")
        self._bqd_host = wp.zeros(model.body_count, dtype=wp.spatial_vector, device="cpu")
        tfs = [lookat_transform(cam_pos, cam_target)]
        if self.multiview:
            tfs.append(lookat_transform(cam2_pos, cam2_target))
            self.out_mv = wp.zeros((height, width * 2, 3), dtype=wp.uint8)
            self._mv_host = wp.zeros((height, width * 2, 3), dtype=wp.uint8, device="cpu")
        # NB shape is (camera_count, world_count), NOT (world_count,
        # camera_count) -- the render context asserts on it. The image
        # outputs use the opposite order, (world, camera, h, w), which is why
        # the resolve kernel indexes [0, cam].  The original single-camera
        # [[tf]] was 1x1 and hid the distinction.
        self.cam_tf = wp.array([[tf] for tf in tfs], dtype=wp.transformf)

        self.frames = _FrameStore()
        self.force_rows: list[dict] = []
        self.state_rows: list[np.ndarray] = []   # body_q per frame
        self.qd_rows: list[np.ndarray] = []      # body_qd per frame
        self.times: list[float] = []
        self._writer = None
        self._writer_mv = None
        self._stream_path = self.out_dir / f"_stream_{os.getpid()}.mp4"
        self._stream_mv_path = self.out_dir / f"_stream_mv_{os.getpid()}.mp4"

    def _light_rig(self):
        """Key (shadow-casting) + fill directional lights."""
        ctx = getattr(self.cam, "_SensorTiledCamera__render_context", None)
        if ctx is None:      # private layout changed — keep the default light
            return
        dev = ctx.device
        ctx.lights_active = wp.array([True, True], dtype=wp.bool, device=dev)
        ctx.lights_type = wp.array([1, 1], dtype=wp.int32, device=dev)
        ctx.lights_cast_shadow = wp.array([True, False], dtype=wp.bool, device=dev)
        ctx.lights_position = wp.array([wp.vec3f(0.0)] * 2, dtype=wp.vec3f, device=dev)
        ctx.lights_orientation = wp.array(
            [wp.vec3f(*KEY_DIR), wp.vec3f(*FILL_DIR)], dtype=wp.vec3f, device=dev)

    def discard_episode(self):
        """Close and discard an unsaved episode before an environment reset."""
        for name in ("_writer", "_writer_mv"):
            writer = getattr(self, name, None)
            if writer is not None:
                writer.close()
                setattr(self, name, None)
        for path in (self._stream_path, self._stream_mv_path):
            path.unlink(missing_ok=True)
        self.frames = _FrameStore()
        self.force_rows.clear()
        self.state_rows.clear()
        self.qd_rows.clear()
        self.times.clear()

    def _mk_writer(self, path):
        import imageio.v2 as imageio
        return imageio.get_writer(
            path, fps=self.fps, codec="libx264",
            macro_block_size=8, pixelformat="yuv420p",
            ffmpeg_params=["-crf", "16", "-preset", "medium"])

    def _video_writer(self):
        if self._writer is None:
            self._writer = self._mk_writer(self._stream_path)
        return self._writer

    def _video_writer_mv(self):
        if self._writer_mv is None:
            self._writer_mv = self._mk_writer(self._stream_mv_path)
        return self._writer_mv

    def capture(self, state, sim_time: float, forces: dict | None = None):
        if self._look_pending:
            from force_bench import kit_look
            self._look_pending = False
            if kit_look.apply_colors(self.model, self._look_name):
                self.bg_top, self.bg_bottom = kit_look.background(self._look_name)
        self.model.bvh_refit_shapes(state)
        self.cam.update(
            state,
            self.cam_tf,
            self.rays,
            color_image=self.color,
            shape_index_image=self.sidx,
            clear_data=type(self.cam).GRAY_CLEAR_DATA,
        )
        wp.launch(
            _resolve_frame_kernel, dim=(self.height, self.width),
            inputs=[self.color, self.sidx, self.out_img, self.ss,
                    self.height * self.ss, wp.vec3f(*self.bg_top), wp.vec3f(*self.bg_bottom),
                    0, 0])
        # through the fixed host staging buffers (see __init__): the writer
        # consumes the view synchronously (it serialises to the ffmpeg pipe)
        # and the frame store JPEG-encodes it on append
        wp.copy(self._img_host, self.out_img)
        img = self._img_host.numpy()
        self._video_writer().append_data(img)
        # frames[] stays the PRIMARY view only: contact sheets and every other
        # downstream reader expect a single-view frame.
        self.frames.append(img)
        if self.multiview:
            for ci in range(2):
                wp.launch(
                    _resolve_frame_kernel, dim=(self.height, self.width),
                    inputs=[self.color, self.sidx, self.out_mv, self.ss,
                            self.height * self.ss, wp.vec3f(*self.bg_top),
                            wp.vec3f(*self.bg_bottom), ci, ci * self.width])
            wp.copy(self._mv_host, self.out_mv)
            self._video_writer_mv().append_data(self._mv_host.numpy())
        self.times.append(sim_time)
        wp.copy(self._bq_host, state.body_q)
        wp.copy(self._bqd_host, state.body_qd)
        self.state_rows.append(self._bq_host.numpy().copy())
        self.qd_rows.append(self._bqd_host.numpy().copy())
        if forces:
            self.force_rows.append({"t": sim_time, **forces})

    def save(self, name: str, force_keys: list[str] | None = None) -> dict:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        paths = {}
        mp4 = self.out_dir / f"{name}.mp4"
        if self._writer is not None:
            self._writer.close()
            self._writer = None
            os.replace(self._stream_path, mp4)
        paths["video"] = str(mp4)

        if self._writer_mv is not None:
            self._writer_mv.close()
            self._writer_mv = None
            mv = self.out_dir / f"{name}_multiview.mp4"
            os.replace(self._stream_mv_path, mv)
            paths["video_multiview"] = str(mv)

        npz = self.out_dir / f"{name}_states.npz"
        np.savez_compressed(
            npz,
            t=np.array(self.times),
            body_q=np.array(self.state_rows),
            body_qd=np.array(self.qd_rows),
            body_labels=np.array(self.model.body_label, dtype=object),
        )
        paths["states"] = str(npz)

        if self.force_rows:
            import csv
            keys = list(self.force_rows[0].keys())
            csv_path = self.out_dir / f"{name}_forces.csv"
            with open(csv_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=keys)
                w.writeheader()
                w.writerows(self.force_rows)
            paths["forces_csv"] = str(csv_path)

            t = [r["t"] for r in self.force_rows]
            plot_keys = force_keys or [k for k in keys if k != "t"]
            fig, ax = plt.subplots(figsize=(9.5, 4.4))
            for k in plot_keys:
                ax.plot(t, [r[k] for r in self.force_rows], label=k, lw=1.6)
            ax.set_xlabel("time [s]")
            ax.set_ylabel("force [N]")
            ax.legend(fontsize=8, framealpha=0.9)
            ax.grid(alpha=0.25, lw=0.6)
            ax.set_title(f"{self.out_dir.name} — {name}")
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            fig.tight_layout()
            png = self.out_dir / f"{name}_forces.png"
            fig.savefig(png, dpi=160)
            plt.close(fig)
            paths["forces_png"] = str(png)
        return paths
