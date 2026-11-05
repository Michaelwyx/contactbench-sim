"""GelSight validation bench: prescribed indenter motions, no solver.

WHY (2026-08-24): the tactile output inside the tasks looked wrong, so this
bench isolates the SENSOR + RENDER stack from all dynamics: a kinematic
indenter follows prescribed press / shear / rotation trajectories against
one static pad, every frame runs collide -> HydroelasticGelSight.sample ->
the GPU tactile renderer, and the video carries the commanded and measured
numbers in a side panel.  Where a closed form exists (sphere: F = pi k R
d^2, flat: F = k A d, both with the stiffening correction integrated
numerically) the analytic force is shown next to the sensor's.

Experiments (``--exp`` selects, default all):
  press_<obj>     depth ramp 0 -> 1.5 mm -> hold -> retract
  steps_<obj>     depth staircase 0.3 / 0.6 / 0.9 / 1.2 mm
  shear_<obj>     press 0.8 mm, slide +x 2.5 mm, back, then +y 2.5 mm
  rotate_<obj>    press 0.8 mm, twist 45 deg about the normal, back
with objects sphere / box / rod / key_bow.  Videos land in
outputs/gelsight/validate/<exp>.mp4, metrics in <exp>.json.

Parameter sweeps (``--sweep``): the SAME shear_sphere trajectory rendered
side by side for 3 values of one parameter, labeled per tile:
  gel_kh, stiffen, skirt (membrane softness SKIRT_SIGMA_MM), blur
  (BLUR_SIGMA), shear_k (shear_stiffness_pa_per_m), marker_gain
  (dilation_gain = shear_gain), lambda_d
-> outputs/gelsight/validate/sweep_<param>.mp4

Run (env tactile-newton, from sim/):
    PYTHONNOUSERSITE=1 hydrogel validate [--exp press_sphere,...]
    PYTHONNOUSERSITE=1 hydrogel validate --sweep marker_gain
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import warp as wp

import newton

import hydrogel.render_kernels as krn
from hydrogel.render import (
    GS_H,
    GS_W,
    PIXMM,
    TaximRenderer,
    draw_markers,
    marker_grid,
    pressure_tile,
)
from hydrogel.markers import HydroShearMarkerConfig, HydroShearMarkers
from hydrogel.sensor import hydroelastic_object_cfg, winning_hydroelastic_config
from hydrogel.sensor import GelSightConfig, HydroelasticGelSight, attach_gelsight_pad
from hydrogel.paths import GELSIGHT_OUTPUT_DIR

import os

OUT = GELSIGHT_OUTPUT_DIR / "validate"
FPS = 60
# GS_VIDEO_SCALE (default 2): integer upscale of the validate videos.  The
# native tiles are 320x240 -- too small to inspect.  The sensor tiles are
# upscaled NEAREST (keeps the raster pixels crisp) and the panel text is
# drawn at the scaled size with a real vector font, so it stays sharp.
VIDEO_SCALE = max(1, int(os.environ.get("GS_VIDEO_SCALE", "2")))
_FONT_CACHE: dict = {}


def _panel_font(s: int):
    if s not in _FONT_CACHE:
        from PIL import ImageFont
        try:
            _FONT_CACHE[s] = ImageFont.truetype("DejaVuSansMono.ttf", 11 * s)
        except OSError:
            try:
                _FONT_CACHE[s] = ImageFont.truetype("DejaVuSans.ttf", 11 * s)
            except OSError:
                _FONT_CACHE[s] = None
    return _FONT_CACHE[s]


def _upscale(frame, s: int):
    if s <= 1:
        return frame
    from PIL import Image
    im = Image.fromarray(np.ascontiguousarray(frame))
    return np.asarray(im.resize((im.width * s, im.height * s), Image.NEAREST))


# ── objects ────────────────────────────────────────────────────────────
def object_trimesh(name: str):
    import trimesh

    if name == "sphere":
        return trimesh.creation.icosphere(subdivisions=4, radius=0.004)
    if name == "box":
        return trimesh.creation.box(extents=(0.012, 0.012, 0.012))
    if name == "rod":
        # 6 mm rod lying along the pad's long axis (gel x = image h)
        tm = trimesh.creation.capsule(radius=0.003, height=0.014, count=(48, 48))
        tm.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2, (0.0, 1.0, 0.0)))
        return tm
    if name == "key_bow":
        from hydrogel.indenters import object_mesh

        tm, _kw = object_mesh("key_bow_b")
        return tm
    raise ValueError(name)


class Bench:
    """One static pad + one kinematic indenter; collide + sample + render."""

    def __init__(self, obj: str, cfg: GelSightConfig,
                 marker_cfg: HydroShearMarkerConfig | None = None,
                 skirt_sigma_mm: float = 1.2, blur_sigma: float = 1.1):
        import trimesh
        from hydrogel.mesh import make_newton_mesh

        self.cfg = cfg
        self.obj_name = obj
        tm = object_trimesh(obj)
        b = newton.ModelBuilder()
        self.z0 = 0.05
        body = b.add_body(xform=wp.transform(wp.vec3(0.0, 0.0, self.z0),
                                             wp.quat_identity()),
                          is_kinematic=True, label="indenter")
        mesh, center = make_newton_mesh(tm, sdf=True, sdf_resolution=256,
                                        narrow_band=0.002, margin=0.0005)
        b.add_shape_mesh(body, mesh=mesh,
                         cfg=hydroelastic_object_cfg(cfg, density=1000.0,
                                                     mesh=True),
                         label="obj")
        # pad LAST: the gel must be shape B of the pair (see gelsight_sensor)
        t = cfg.gel_thickness_m
        gel = attach_gelsight_pad(b, -1, cfg, xform=wp.transform_identity(),
                                  label="gel", physics_thickness_m=2.0 * t)
        self.model = b.finalize()
        self.state = self.model.state()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd,
                       self.state)
        self.pipeline = newton.CollisionPipeline(
            self.model, rigid_contact_max=20_000, broad_phase="explicit",
            sdf_hydroelastic_config=winning_hydroelastic_config(
                self.model, cfg, [gel]))
        self.contacts = self.pipeline.contacts()
        self.sensor = HydroelasticGelSight(self.model, self.pipeline, gel, cfg)
        self.body = body
        # lowest object point over the pad footprint -> touch height
        v = tm.vertices - np.asarray(center)
        over = v[(np.abs(v[:, 0]) < 0.5 * cfg.width_m)
                 & (np.abs(v[:, 1]) < 0.5 * cfg.height_m)]
        self.zmin = float((over if len(over) else v)[:, 2].min())
        self.z_touch = t - self.zmin
        self.center = np.asarray(center, dtype=np.float64)
        # renderer state
        self.renderer = TaximRenderer()
        self.renderer_blur = blur_sigma
        self.skirt_px = skirt_sigma_mm / PIXMM
        self.grid = marker_grid()
        self.markers = HydroShearMarkers(
            self.grid, (GS_H, GS_W), (cfg.width_m, cfg.height_m),
            cfg=marker_cfg or HydroShearMarkerConfig())
        self.wp_mesh = wp.Mesh(
            points=wp.array(np.asarray(tm.vertices - center, dtype=np.float32),
                            dtype=wp.vec3),
            indices=wp.array(np.asarray(tm.faces, dtype=np.int32).flatten(),
                             dtype=wp.int32))
        self._h = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        self._t1 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        self._t2 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        self._mech = wp.zeros((cfg.rows, cfg.cols), dtype=wp.float32)

    def step(self, pos, quat, lin_v, ang_v, dt=1.0 / FPS):
        """Place the indenter (body origin at the mesh AABB centre), collide,
        sample.  Velocities feed the sensor's shear inference."""

        bq = self.state.body_q.numpy()
        bq[self.body] = (*pos, *quat)
        self.state.body_q.assign(bq)
        qd = self.state.body_qd.numpy()
        qd[self.body, :3] = lin_v
        qd[self.body, 3:] = ang_v
        self.state.body_qd.assign(qd)
        self.pipeline.collide(self.state, self.contacts)
        return self.sensor.sample(self.state, dt)

    # ── height map: raycast in the gel frame (identity here) + mech max ──
    def height_device(self, frame):
        self._h.zero_()
        bq = self.state.body_q.numpy()[self.body]
        # X_og: gel frame -> object frame (object at bq, gel at identity)
        import trimesh.transformations as tt

        M = tt.quaternion_matrix([bq[6], bq[3], bq[4], bq[5]])
        M[:3, 3] = bq[:3]
        Mi = np.linalg.inv(M)
        q = tt.quaternion_from_matrix(Mi)          # (w, x, y, z)
        X_og = wp.transform(wp.vec3(*Mi[:3, 3].astype(np.float32)),
                            wp.quat(float(q[1]), float(q[2]), float(q[3]),
                                    float(q[0])))
        from hydrogel.render import CAM_PERSP, CAM_STANDOFF_M
        if CAM_PERSP:
            wp.launch(krn.raycast_height_persp, dim=(GS_H, GS_W),
                      inputs=[self.wp_mesh.id, X_og,
                              float(self.cfg.width_m), float(self.cfg.height_m),
                              float(self.cfg.gel_thickness_m), CAM_STANDOFF_M,
                              0.02, float(self.cfg.gel_thickness_m)],
                      outputs=[self._h])
        else:
            wp.launch(krn.raycast_height, dim=(GS_H, GS_W),
                      inputs=[self.wp_mesh.id, X_og,
                              float(self.cfg.width_m), float(self.cfg.height_m),
                              float(self.cfg.gel_thickness_m), 0.02,
                              float(self.cfg.gel_thickness_m)],
                      outputs=[self._h])
        sig = float(self.skirt_px)
        rad = int(4.0 * sig + 0.5)
        wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[self._h, rad, sig],
                  outputs=[self._t1])
        wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[self._t1, rad, sig],
                  outputs=[self._t2])
        wp.launch(krn.max_into, dim=(GS_H, GS_W), inputs=[self._t2],
                  outputs=[self._h])
        self._mech.assign(np.ascontiguousarray(frame.normal_displacement_m,
                                               dtype=np.float32))
        wp.launch(krn.upsample_raster_max, dim=(GS_H, GS_W),
                  inputs=[self._mech], outputs=[self._h])
        from hydrogel.render import BULGE_K, BULGE_SIGMA_MM
        k_bulge = getattr(self, "_bulge_k", BULGE_K)
        if k_bulge > 0.0:
            if not hasattr(self, "_b1"):
                self._b1 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
                self._b2 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
                self._b3 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
            sig = BULGE_SIGMA_MM / PIXMM
            rad = int(4.0 * sig + 0.5)
            wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[self._h, rad, float(sig)], outputs=[self._b1])
            wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[self._b1, rad, float(sig)], outputs=[self._b2])
            wp.launch(krn.bulge_ring, dim=(GS_H, GS_W), inputs=[self._h, self._b2, float(k_bulge)], outputs=[self._b3])
            return self._b3
        return self._h

    def render_tile(self, frame, vmax_pa: float):
        hdev = self.height_device(frame)
        d = self.renderer._device_tables()
        sig = float(self.renderer_blur)
        rad = int(2.0 * sig + 0.5)
        wp.launch(krn.scale_into, dim=(GS_H, GS_W),
                  inputs=[hdev, float(1000.0 / PIXMM)], outputs=[d["px"]])
        wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[d["px"], rad, sig],
                  outputs=[d["tmp"]])
        wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[d["tmp"], rad, sig],
                  outputs=[d["px"]])
        wp.launch(krn.taxim_shade, dim=(GS_H, GS_W),
                  inputs=[d["px"], d["grad_r"], d["grad_g"], d["grad_b"],
                          float(self.renderer.x_binr), float(self.renderer.y_binr),
                          float(getattr(self.renderer, "gain", 1.0)),
                          d["flat"], d["bg"]],
                  outputs=[d["rgb"]])
        rgb = d["rgb"].numpy()
        from hydrogel.render import MARKER_MODEL, project_markers
        if MARKER_MODEL == "hydroshear":
            centers = self.markers.displace_device(hdev, frame.shear_displacement_m)
        else:
            from hydrogel.markers import field_markers
            centers = field_markers(self.grid, frame.shear_displacement_m,
                                    (GS_H, GS_W))
        h_np = hdev.numpy()
        centers = project_markers(centers, h_np)
        from hydrogel.render import MARKER_RENDER, warp_marker_layer
        if MARKER_RENDER == "warp":
            # texture-warp: dots deform non-rigidly across the contact rim
            tile_rgb = warp_marker_layer(rgb, h_np,
                                         frame.shear_displacement_m)
        else:
            tile_rgb = draw_markers(rgb, centers)
        from hydrogel.render import draw_marker_arrows
        tile_rgb = draw_marker_arrows(tile_rgb, self.grid, centers)
        tile_p = pressure_tile(frame.pressure_pa, vmax_pa)
        return np.concatenate([tile_rgb, tile_p], axis=1), centers


# ── trajectories: t -> (pos, quat, lin_v, ang_v, label dict) ───────────
def make_traj(kind: str, bench: Bench):
    import trimesh.transformations as tt

    def base(depth, x=0.0, y=0.0, yaw=0.0):
        # bench.z_touch is the body height at which the object's lowest
        # point sits exactly on the sensing face; pressing = going below it
        q = tt.quaternion_about_axis(yaw, (0, 0, 1))
        return (np.array([x, y, bench.z_touch - depth]),
                np.array([q[1], q[2], q[3], q[0]]))

    if kind == "press":
        T = 4.5

        def f(t):
            if t < 2.0:
                d = 1.5e-3 * t / 2.0
            elif t < 3.0:
                d = 1.5e-3
            else:
                d = 1.5e-3 * max(0.0, (4.0 - t)) / 1.0
            pos, quat = base(d)
            return pos, quat, np.array([0.0, 0.0, -0.75e-3 if t < 2.0 else 0.0]), np.zeros(3), dict(depth_mm=d * 1e3)
        return T, f
    if kind == "steps":
        T = 4.8
        steps = [0.3e-3, 0.6e-3, 0.9e-3, 1.2e-3]

        def f(t):
            d = steps[min(int(t // 1.2), 3)]
            pos, quat = base(d)
            return pos, quat, np.zeros(3), np.zeros(3), dict(depth_mm=d * 1e3)
        return T, f
    if kind == "shear":
        T = 7.0
        D = 0.8e-3
        A = 2.5e-3
        V = A / 1.5

        def f(t):
            x = y = 0.0
            vx = vy = 0.0
            if t < 1.0:
                d = D * t
            else:
                d = D
                if t < 2.5:
                    x = A * (t - 1.0) / 1.5
                    vx = V
                elif t < 4.0:
                    x = A * (4.0 - t) / 1.5
                    vx = -V
                elif t < 5.5:
                    y = A * (t - 4.0) / 1.5
                    vy = V
                else:
                    y = A * (7.0 - t) / 1.5
                    vy = -V
            pos, quat = base(d, x, y)
            return (pos, quat, np.array([vx, vy, 0.0]), np.zeros(3),
                    dict(depth_mm=d * 1e3, slide_x_mm=x * 1e3, slide_y_mm=y * 1e3))
        return T, f
    if kind == "rotate":
        T = 5.0
        D = 0.8e-3
        W = np.deg2rad(45.0) / 1.5

        def f(t):
            yaw = 0.0
            wz = 0.0
            d = D * min(t, 1.0)
            if 1.0 <= t < 2.5:
                yaw = W * (t - 1.0)
                wz = W
            elif 2.5 <= t < 4.0:
                yaw = W * (4.0 - t)
                wz = -W
            pos, quat = base(d, yaw=yaw)
            return (pos, quat, np.zeros(3), np.array([0.0, 0.0, wz]),
                    dict(depth_mm=d * 1e3, yaw_deg=np.degrees(yaw)))
        return T, f
    raise ValueError(kind)


# ── analytic references ────────────────────────────────────────────────
def analytic_force(obj: str, depth: float, cfg: GelSightConfig) -> float | None:
    from hydrogel.pressure import pressure_np

    k, t, a = cfg.gel_kh_pa_per_m, cfg.gel_thickness_m, cfg.gel_stiffening
    if depth <= 0.0:
        return 0.0
    if obj == "sphere":
        R = 0.004
        r = np.linspace(0.0, np.sqrt(max(2.0 * R * depth - depth ** 2, 0.0)), 200)
        x = depth - (R - np.sqrt(np.maximum(R * R - r * r, 0.0)))
        p = pressure_np(np.maximum(x, 0.0), k, t, a)
        return float(np.trapezoid(p * 2.0 * np.pi * r, r))
    if obj == "box":
        A = 0.012 * 0.012
        return float(pressure_np(np.array(depth), k, t, a) * A)
    return None


# ── overlay ────────────────────────────────────────────────────────────
def overlay_panel(labels: dict, h: int, w: int = 190,
                  s: int = 1) -> np.ndarray:
    from PIL import Image, ImageDraw

    img = Image.new("RGB", (w * s, h), (18, 22, 24))
    dr = ImageDraw.Draw(img)
    y = 14 * s
    for k, v in labels.items():
        dr.text((10 * s, y), f"{k:<13s}{v}", fill=(230, 228, 220),
                font=_panel_font(s))
        y += 22 * s
    return np.asarray(img)


def run_experiment(exp: str, cfg: GelSightConfig | None = None,
                   marker_cfg=None, skirt=1.2, blur=1.1,
                   out_dir: Path = OUT, tag: str = "") -> dict:
    kind, obj = exp.split("_", 1)
    cfg = cfg or GelSightConfig(rows=96, cols=128, gel_kh_pa_per_m=4.0e7,
                                gel_stiffening=10.0)
    bench = Bench(obj, cfg, marker_cfg=marker_cfg, skirt_sigma_mm=skirt,
                  blur_sigma=blur)
    T, traj = make_traj(kind, bench)
    import imageio.v2 as iio

    out_dir.mkdir(parents=True, exist_ok=True)
    name = exp + (f"_{tag}" if tag else "")
    vpath = out_dir / f"{name}.mp4"
    writer = iio.get_writer(vpath, fps=FPS, quality=8, macro_block_size=None)
    metrics = []
    vmax = None
    grid0 = bench.grid
    for i in range(int(T * FPS)):
        t = i / FPS
        pos, quat, lin, ang, lab = traj(t)
        frame = bench.step(pos, quat, lin, ang)
        if vmax is None:
            # scale the pressure colormap from the law at full depth
            vmax = max(float(analytic_force(obj, 1.5e-3, cfg) or 5.0), 1.0) \
                / (0.25 * cfg.width_m * cfg.height_m)
        tile, centers = bench.render_tile(frame, vmax)
        marker_disp = float(np.linalg.norm(centers - grid0, axis=1).max())
        ana = analytic_force(obj, lab.get("depth_mm", 0.0) * 1e-3, cfg)
        row = dict(t=round(t, 3), **{k: round(v, 3) for k, v in lab.items()},
                   F_sensor_N=round(frame.total_normal_force_n, 3),
                   F_analytic_N=None if ana is None else round(ana, 3),
                   p_max_kPa=round(float(frame.pressure_pa.max()) * 1e-3, 1),
                   shear_max_um=round(float(np.linalg.norm(
                       frame.shear_displacement_m, axis=0).max()) * 1e6, 1),
                   marker_max_px=round(marker_disp, 2))
        metrics.append(row)
        labels = {k: (f"{v:8.2f}" if isinstance(v, float) else f"{v}")
                  for k, v in row.items() if v is not None}
        tile = _upscale(tile, VIDEO_SCALE)
        panel = overlay_panel(labels, tile.shape[0], s=VIDEO_SCALE)
        writer.append_data(np.concatenate([tile, panel], axis=1))
    writer.close()
    with open(out_dir / f"{name}.json", "w") as fh:
        json.dump(dict(exp=exp, cfg=dict(gel_kh=cfg.gel_kh_pa_per_m,
                                         stiffen=cfg.gel_stiffening,
                                         shear_k=cfg.shear_stiffness_pa_per_m),
                       skirt=skirt, blur=blur, frames=metrics), fh)
    last_press = [m for m in metrics if m.get("depth_mm", 0) >= 1.0][:1]
    print(f"[validate] {name}: wrote {vpath.name}; "
          + (f"F at 1.0 mm sensor {last_press[0]['F_sensor_N']} vs analytic "
             f"{last_press[0]['F_analytic_N']}" if last_press else ""))
    return dict(video=str(vpath), metrics=metrics)


SWEEPS = {
    "gel_kh": [1.0e7, 4.0e7, 1.6e8],
    "stiffen": [0.0, 10.0, 40.0],
    "skirt": [0.4, 1.2, 3.0],
    "blur": [0.5, 1.1, 3.0],
    "shear_k": [2.0e7, 8.0e7, 3.2e8],
    "marker_gain": [1.0, 4.0, 16.0],
    "lambda_d": [5000.0, 20000.0, 80000.0],
    "contrast": [1.0, 2.0, 4.0],
    "bulge": [0.0, 0.8, 2.0],
}


def run_sweep(param: str, exp: str = "shear_sphere", out_dir: Path = OUT) -> None:
    values = SWEEPS[param]
    benches = []
    for v in values:
        cfg = dict(rows=96, cols=128, gel_kh_pa_per_m=4.0e7, gel_stiffening=10.0)
        mk = dict()
        skirt, blur = 1.2, 1.1
        if param == "gel_kh":
            cfg["gel_kh_pa_per_m"] = v
        elif param == "stiffen":
            cfg["gel_stiffening"] = v
        elif param == "shear_k":
            cfg["shear_stiffness_pa_per_m"] = v
        elif param == "skirt":
            skirt = v
        elif param == "blur":
            blur = v
        elif param == "contrast":
            pass                     # applied to the bench renderer below
        elif param == "bulge":
            pass                     # applied via env below
        elif param == "marker_gain":
            mk = dict(dilation_gain=v, shear_gain=v)
        elif param == "lambda_d":
            mk = dict(lambda_d=v)
        kind, obj = exp.split("_", 1)
        b = Bench(obj, GelSightConfig(**cfg),
                  marker_cfg=HydroShearMarkerConfig(**mk) if mk else None,
                  skirt_sigma_mm=skirt, blur_sigma=blur)
        if param == "contrast":
            b.renderer.gain = v
        if param == "bulge":
            b._bulge_k = v
        benches.append((v, b))
    T, _ = make_traj(exp.split("_", 1)[0], benches[0][1])
    import imageio.v2 as iio
    from PIL import Image, ImageDraw

    out_dir.mkdir(parents=True, exist_ok=True)
    vpath = out_dir / f"sweep_{param}.mp4"
    writer = iio.get_writer(vpath, fps=FPS, quality=8, macro_block_size=None)
    vmax = 4.0e7 * 1.0e-3 * 3.0
    for i in range(int(T * FPS)):
        t = i / FPS
        tiles = []
        for v, b in benches:
            _T, traj = make_traj(exp.split("_", 1)[0], b)
            pos, quat, lin, ang, lab = traj(t)
            frame = b.step(pos, quat, lin, ang)
            tile, _ = b.render_tile(frame, vmax)
            img = Image.fromarray(np.ascontiguousarray(
                _upscale(tile, VIDEO_SCALE)))
            dr = ImageDraw.Draw(img)
            dr.text((8 * VIDEO_SCALE, 6 * VIDEO_SCALE),
                    f"{param} = {v:g}   F {frame.total_normal_force_n:5.2f} N",
                    fill=(255, 240, 200), font=_panel_font(VIDEO_SCALE))
            tiles.append(np.asarray(img))
        row = {k: f"{x:8.2f}" for k, x in lab.items()}
        panel = overlay_panel(row, tiles[0].shape[0], 150, s=VIDEO_SCALE)
        writer.append_data(np.concatenate(tiles + [panel], axis=1))
    writer.close()
    print(f"[validate] sweep {param}: wrote {vpath.name}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp", default="")
    ap.add_argument("--sweep", default="")
    args = ap.parse_args()
    wp.init()
    if args.sweep:
        params = list(SWEEPS) if args.sweep == "all" else args.sweep.split(",")
        for q in params:
            run_sweep(q)
        return 0
    exps = (args.exp.split(",") if args.exp else
            [f"{k}_{o}" for k in ("press", "steps", "shear", "rotate")
             for o in ("sphere", "box", "rod", "key_bow")])
    for e in exps:
        run_experiment(e)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
