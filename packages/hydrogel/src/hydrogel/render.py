"""GelSight Mini tactile RGB rendering from solver-derived gel deformation.

TacEx/UniVTAC-style optical pipeline on the Newton hydroelastic contact
model: height map -> Taxim polynomial-lookup RGB -> marker composite.  The
height map is NOT a camera render -- it is the sensor's own
``normal_displacement_m`` channel (pressure / gel_kh), so the tactile image
derives from the same validated mechanics the QA gates check.

The Taxim evaluation replicates HydroShear's ``gelsight_render.py``
(NVIDIA TacSL's tensorized Taxim) in NumPy: height map in pixel units ->
np.gradient -> (arctan-squashed magnitude, direction) bin lookup into a
per-bin 6-coefficient polynomial of the PIXEL COORDINATES -> per-channel
delta added to a real GelSight Mini background image.  Calibration assets
are vendored under assets/gelsight/gs_mini/ (MIT, see PROVENANCE.md).

Markers: a 9x7 grid advected by the sensor's ``shear_displacement_m``
field (bilinearly sampled at each marker, metres -> pixels via pixmm),
drawn as anti-aliased dark dots -- HydroShear's actual policy observation
is this marker displacement field, so the dots are data, not decoration.

Usage:
    python -m hydrogel.render --selftest
    python -m hydrogel.render --task-dir ../outputs/gelsight/gelsight_grip [--panel]

As a library: ``render_task_dir(task_dir)`` reads ``tactile_left.npz`` /
``tactile_right.npz`` (written by task_gelsight_grip.py) and writes
``tactile_strip.mp4`` (L RGB+markers | R RGB+markers | L pressure |
R pressure), plus ``rollout_panel_tactile.mp4`` (multiview video with the
strip composited underneath) with ``panel=True``.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import warp as wp
from scipy import ndimage

import os

from hydrogel.markers import HydroShearMarkers

# GS_MARKERS=hydroshear (default): HydroShear dilation + kernel-spread shear
# from the geometric height map and the shear state; local: bilinear sample
# of the shear raster only (no dilation, no spreading)
MARKER_MODEL = os.environ.get("GS_MARKERS", "field")
# GS_MARKER_RENDER=warp (default): markers are rendered by INVERSE-WARPING
# an analytic rest-dot texture through the per-pixel displacement field
# (membrane-blurred shear + per-pixel pinhole shift), so a dot straddling
# the contact edge deforms NON-RIGIDLY -- half of it rides the pressed
# surface, half stays on the plane, and the crease/kink a real sensor
# shows at indentation borders emerges from the displacement gradient
# inside the dot.  "stamp" restores the rigid-disk translation (each dot
# moved whole by the displacement at its centre).
MARKER_RENDER = os.environ.get("GS_MARKER_RENDER", "warp")

from hydrogel.paths import GELSIGHT_OUTPUT_DIR, GS_MINI_DIR

ASSET_DIR = GS_MINI_DIR

# GelSight Mini optical constants (HydroShear conf_gs_mini; PROVENANCE.md)
GS_H = 320            # image rows, along the gel's LONG axis (sensor x/cols)
GS_W = 240            # image cols, along the gel's short axis (sensor y/rows)
NUM_BINS = 120
PIXMM = 0.065         # mm per pixel
# cv2.GaussianBlur((5,5), sigma=0) equivalent: sigma = 0.3*((5-1)*0.5-1)+0.8
BLUR_SIGMA = 1.1
# contrast gain on the calibrated intensity response (rgb = bg + gain*(poly -
# flat)); the vendored GS Mini table's own contrast reads soft next to real
# captures -- GS_RENDER_GAIN (or TaximRenderer.gain) scales it
# default 2.0 (2026-08-24): the validation bench showed gain 2-4 matches the
# vivid look of real captures; 1.0 is the vendored table's own contrast
RENDER_GAIN = float(os.environ.get("GS_RENDER_GAIN", "2.0"))

MARKER_GRID = (9, 7)  # (along h, along w) = 63 markers
MARKER_MARGIN_PX = 24
MARKER_RADIUS_PX = 6.0
MARKER_DARKEN = 0.55  # dot intensity multiplier at the center

# Gel membrane draping: the skirt the gel surface forms around a contact.
# Applied to the GEOMETRIC height map (max of the sharp penetration map and
# its wide Gaussian), so edges stay sharp while the surroundings dome.
SKIRT_SIGMA_MM = 1.2
# GS Mini camera pinhole standoff behind the gel surface (datasheet/TacEx:
# camera 24 mm behind the pad, clipping 24-34 mm).  GS_CAM_PERSP=0 restores
# the orthographic height map and un-projected markers.
CAM_STANDOFF_M = 0.024
# membrane bulge: fraction of the ring field (positive part of a wide blur
# minus the indentation) SUBTRACTED from the height map, i.e. raised rim
# around every contact -- the volume-conserving skin a Winkler law lacks.
# GS_BULGE=0 disables; sigma is the rim's spread.
BULGE_K = float(os.environ.get("GS_BULGE", "0"))   # off by default (2026-08-24 user call); 0.8 was the demo value
BULGE_SIGMA_MM = float(os.environ.get("GS_BULGE_SIGMA", "2.5"))
CAM_PERSP = os.environ.get("GS_CAM_PERSP", "1") != "0" 
MAX_PEN_M = 0.0035    # penetration clamp: just under the 4 mm gel thickness


def _validate_calib(data) -> None:
    for key, shape in (("grad_r", 3), ("grad_g", 3), ("grad_b", 3)):
        if key not in data.files:
            raise KeyError(f"{key} missing from polycalib.npz "
                           f"(has {data.files}); see PROVENANCE.md")
        if data[key].ndim != shape or data[key].shape[-1] != 6:
            raise ValueError(f"{key} has shape {data[key].shape}, "
                             "expected (bins, bins, 6)")


class TaximRenderer:
    """NumPy port of the HydroShear/TacSL Taxim polynomial renderer."""

    def __init__(self, calib_path: Path | None = None,
                 background_path: Path | None = None) -> None:
        calib_path = calib_path or ASSET_DIR / "polycalib.npz"
        background_path = background_path or ASSET_DIR / "bg.png"
        data = np.load(calib_path)
        _validate_calib(data)
        self.grad_r = data["grad_r"]
        self.grad_g = data["grad_g"]
        self.grad_b = data["grad_b"]

        import imageio.v2 as iio

        self.background = np.asarray(iio.imread(background_path), dtype=np.float64)
        if self.background.shape[:2] != (GS_H, GS_W):
            raise ValueError(f"background is {self.background.shape[:2]}, "
                             f"expected {(GS_H, GS_W)}")

        xx, yy = np.meshgrid(range(GS_W), range(GS_H))
        xf, yf = xx.flatten(), yy.flatten()
        self.A = np.stack(
            [xf * xf, yf * yf, xf * yf, xf, yf, np.ones(GS_H * GS_W)],
            axis=-1).reshape(GS_H, GS_W, 6)

        binm = NUM_BINS - 1
        self.x_binr = 0.5 * np.pi / binm       # magnitude bins over [0, pi/2]
        self.y_binr = 2.0 * np.pi / binm       # direction bins over [-pi, pi]

        # Flat-field correction: the calibrated table's zero-gradient bin
        # does NOT evaluate to zero against the vendored background (it
        # encodes up to +-100 intensity of position-dependent offset,
        # presumably the difference to the calibration-time background).
        # Subtracting the flat-bin map makes an untouched gel render the
        # background EXACTLY while contact response stays the calibrated
        # difference between bins.
        flat_dir_bin = int(np.floor(np.pi / self.y_binr))
        self.flat_delta = np.stack(
            [np.sum(self.A * tab[0, flat_dir_bin, :], axis=-1)
             for tab in (self.grad_r, self.grad_g, self.grad_b)], axis=-1)

    def _device_tables(self):
        if getattr(self, "_dev", None) is None:
            f32 = lambda a: wp.array(np.ascontiguousarray(a, dtype=np.float32))
            self._dev = dict(
                grad_r=wp.array(np.ascontiguousarray(self.grad_r, dtype=np.float32), dtype=wp.float32),
                grad_g=wp.array(np.ascontiguousarray(self.grad_g, dtype=np.float32), dtype=wp.float32),
                grad_b=wp.array(np.ascontiguousarray(self.grad_b, dtype=np.float32), dtype=wp.float32),
                flat=wp.array(np.ascontiguousarray(self.flat_delta, dtype=np.float32), dtype=wp.float32),
                bg=wp.array(np.ascontiguousarray(self.background[:, :, :3], dtype=np.float32), dtype=wp.float32),
                px=wp.zeros((GS_H, GS_W), dtype=wp.float32),
                tmp=wp.zeros((GS_H, GS_W), dtype=wp.float32),
                rgb=wp.zeros((GS_H, GS_W, 3), dtype=wp.uint8),
            )
        return self._dev

    def render_device(self, height_m_dev) -> np.ndarray:
        """Same shading as ``render`` with the height map already on the
        device (wp.array2d float32, metres); returns uint8 RGB (host)."""

        import hydrogel.render_kernels as krn
        d = self._device_tables()
        sig = float(BLUR_SIGMA)
        rad = int(2.0 * sig + 0.5)               # scipy: int(truncate*sigma + 0.5), truncate=2.0
        wp.launch(krn.scale_into, dim=(GS_H, GS_W),
                  inputs=[height_m_dev, float(1000.0 / PIXMM)], outputs=[d["px"]])
        wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[d["px"], rad, sig], outputs=[d["tmp"]])
        wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[d["tmp"], rad, sig], outputs=[d["px"]])
        wp.launch(krn.taxim_shade, dim=(GS_H, GS_W),
                  inputs=[d["px"], d["grad_r"], d["grad_g"], d["grad_b"],
                          float(self.x_binr), float(self.y_binr),
                          float(getattr(self, "gain", RENDER_GAIN)),
                          d["flat"], d["bg"]],
                  outputs=[d["rgb"]])
        return d["rgb"].numpy()

    def render(self, indent_m: np.ndarray) -> np.ndarray:
        """Tactile RGB (uint8, GS_H x GS_W x 3) from an indentation map.

        ``indent_m``: (GS_H, GS_W) gel indentation in metres, >= 0 where
        pressed (the sensor's normal_displacement_m resampled to image
        resolution -- see ``raster_to_image``).
        """

        if indent_m.shape != (GS_H, GS_W):
            raise ValueError(f"indent map is {indent_m.shape}, "
                             f"expected {(GS_H, GS_W)}")
        height_px = indent_m * 1000.0 / PIXMM
        height_px = ndimage.gaussian_filter(
            height_px.astype(np.float64), sigma=BLUR_SIGMA, truncate=2.0)

        dzdx, dzdy = np.gradient(height_px)
        grad_mag_orig = np.sqrt(dzdx ** 2 + dzdy ** 2)
        grad_mag = np.arctan(grad_mag_orig)
        grad_dir = np.arctan2(dzdx, dzdy)
        grad_dir[grad_mag_orig == 0] = 0.0
        # replicate the reference's boundary handling: interior values,
        # symmetric-padded back to full size
        grad_mag = np.pad(grad_mag[1:-1, 1:-1], 1, mode="symmetric")
        grad_dir = np.pad(grad_dir[1:-1, 1:-1], 1, mode="symmetric")

        idx_x = np.clip(np.floor(grad_mag / self.x_binr).astype(int),
                        0, self.grad_r.shape[0] - 1)
        idx_y = np.clip(np.floor((grad_dir + np.pi) / self.y_binr).astype(int),
                        0, self.grad_r.shape[1] - 1)

        out = np.empty((GS_H, GS_W, 3), dtype=np.float64)
        out[:, :, 0] = np.sum(self.A * self.grad_r[idx_x, idx_y, :], axis=-1)
        out[:, :, 1] = np.sum(self.A * self.grad_g[idx_x, idx_y, :], axis=-1)
        out[:, :, 2] = np.sum(self.A * self.grad_b[idx_x, idx_y, :], axis=-1)
        out = (out - self.flat_delta) * getattr(self, "gain", RENDER_GAIN)
        out += self.background[:, :, :3]
        return np.clip(out, 0, 255).astype(np.uint8)


def raster_to_image(raster: np.ndarray) -> np.ndarray:
    """Resample a sensor raster (rows=gel y, cols=gel x) to image layout.

    The image h axis runs along the gel's long axis (sensor cols/x) and the
    w axis along the short axis (sensor rows/y), matching the GS Mini
    calibration orientation.  Cubic interpolation; e.g. a 96x128 raster
    maps to 320x240 with an exact 2.5x zoom.
    """

    a = np.ascontiguousarray(raster.T)          # (cols, rows) = (x, y)
    zoom = (GS_H / a.shape[0], GS_W / a.shape[1])
    out = ndimage.zoom(a, zoom, order=3, mode="nearest")
    return np.clip(out, 0.0, None) if raster.min() >= 0 else out


# ── geometry-derived height map ────────────────────────────────────────
# The hydroelastic solver's equilibrium already fixes HOW DEEP the object
# sits in the gel (body poses are the realized pressure-field balance); the
# raycast recovers WHAT SHAPE that penetration has at full image
# resolution, and the membrane filter adds the gel's draping skirt.  The
# mechanics rasters stay the ground-truth observation; this only feeds the
# optical path.

_wp = None


def _warp():
    global _wp
    if _wp is None:
        import warp as wp

        import hydrogel.render_kernels as krn

        _wp = (wp, krn)
    return _wp


def _quat_to_matrix(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(q_xyzw, dtype=np.float64)
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _compose(a7: np.ndarray, b7: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(R, p) of transform a7 ∘ b7 (each 7-vec: pos + quat xyzw)."""

    Ra, Rb = _quat_to_matrix(a7[3:]), _quat_to_matrix(b7[3:])
    return Ra @ Rb, a7[:3] + Ra @ b7[:3]


def _object_trimesh(spec: dict, task_dir: Path | None = None):
    """Object mesh for a renderer spec.  Mesh paths are stored absolute by
    the task; when a folder has been moved (outputs/ reorganization
    2026-08-23) fall back to the same basename next to the scene file."""

    import trimesh

    if spec["type"] == "mesh":
        path = Path(spec["params"][0])
        if not path.exists() and task_dir is not None \
                and (Path(task_dir) / path.name).exists():
            path = Path(task_dir) / path.name
        return trimesh.load(str(path), force="mesh")
    if spec["type"] == "box":
        hx, hy, hz = spec["params"]
        return trimesh.creation.box(extents=(2 * hx, 2 * hy, 2 * hz))
    if spec["type"] == "sphere":
        return trimesh.creation.icosphere(subdivisions=4, radius=spec["params"][0])
    raise ValueError(f"unknown object spec {spec}")


class GeometricHeightmap:
    """Per-frame raycast height map of scene objects against a gel face."""

    def __init__(self, scene_path: Path, states_path: Path) -> None:
        wp, self._krn = _warp()
        self._wp_mod = wp
        scene = np.load(scene_path, allow_pickle=True)
        self.pads = [str(p) for p in scene["pads"]]
        self.finger_labels = [str(p) for p in scene["finger_labels"]]
        self.gel_xforms = np.asarray(scene["gel_xforms"], dtype=np.float64)
        pad_size = [float(v) for v in scene["pad_size"]]
        self.pad_w, self.pad_h, self.pad_t = pad_size[:3]
        # sensing-face height in the gel shape frame (thick physics slabs
        # put it at the full gel thickness; legacy scenes at half)
        self.face_z = pad_size[3] if len(pad_size) > 3 else 0.5 * self.pad_t
        objects = json.loads(str(scene["objects_json"]))

        d = np.load(states_path, allow_pickle=True)
        self.body_q = np.asarray(d["body_q"], dtype=np.float64)
        labels = [str(x) for x in d["body_labels"]]
        self.finger_idx = {p: labels.index(f)
                           for p, f in zip(self.pads, self.finger_labels)}
        self.obj_meshes = []
        for spec in objects:
            tm = _object_trimesh(spec, Path(scene_path).parent)
            mesh = wp.Mesh(
                points=wp.array(np.asarray(tm.vertices, dtype=np.float32),
                                dtype=wp.vec3),
                indices=wp.array(np.asarray(tm.faces, dtype=np.int32).flatten(),
                                 dtype=wp.int32))
            self.obj_meshes.append((mesh, labels.index(spec["body"])))
        self._out = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        self._skirt_px = SKIRT_SIGMA_MM / PIXMM

    def n_frames(self) -> int:
        return len(self.body_q)

    def height(self, index: int, pad: str) -> np.ndarray:
        """(GS_H, GS_W) gel indentation [m] at frame ``index``."""

        wp, krn = self._wp_mod, self._krn
        pad_i = self.pads.index(pad)
        finger7 = self.body_q[index, self.finger_idx[pad]]
        R_wg, p_wg = _compose(finger7, self.gel_xforms[pad_i])
        self._out.zero_()
        for mesh, obj_body in self.obj_meshes:
            obj7 = self.body_q[index, obj_body]
            R_wo, p_wo = _quat_to_matrix(obj7[3:]), obj7[:3]
            # gel -> object: X_og = inv(X_wo) ∘ X_wg
            R_og = R_wo.T @ R_wg
            p_og = R_wo.T @ (p_wg - p_wo)
            q_og = _matrix_to_quat(R_og)
            X_og = wp.transform(wp.vec3(*p_og.astype(np.float32)),
                                wp.quat(*q_og.astype(np.float32)))
            if CAM_PERSP:
                wp.launch(krn.raycast_height_persp, dim=(GS_H, GS_W),
                          inputs=[mesh.id, X_og, self.pad_w, self.pad_h,
                                  self.face_z, CAM_STANDOFF_M, 0.05, MAX_PEN_M],
                          outputs=[self._out])
            else:
                wp.launch(krn.raycast_height, dim=(GS_H, GS_W),
                          inputs=[mesh.id, X_og, self.pad_w, self.pad_h,
                                  self.face_z, 0.05, MAX_PEN_M],
                          outputs=[self._out])
        return self._finish_height()

    def height_device(self, index: int, pad: str):
        """Raycast + membrane skirt on the device; returns (height_m, scratch)
        wp.arrays (GS_H, GS_W) float32 -- the caller keeps them on the GPU."""
        """(GS_H, GS_W) gel indentation [m] at frame ``index``."""

        wp, krn = self._wp_mod, self._krn
        pad_i = self.pads.index(pad)
        finger7 = self.body_q[index, self.finger_idx[pad]]
        R_wg, p_wg = _compose(finger7, self.gel_xforms[pad_i])
        self._out.zero_()
        for mesh, obj_body in self.obj_meshes:
            obj7 = self.body_q[index, obj_body]
            R_wo, p_wo = _quat_to_matrix(obj7[3:]), obj7[:3]
            # gel -> object: X_og = inv(X_wo) ∘ X_wg
            R_og = R_wo.T @ R_wg
            p_og = R_wo.T @ (p_wg - p_wo)
            q_og = _matrix_to_quat(R_og)
            X_og = wp.transform(wp.vec3(*p_og.astype(np.float32)),
                                wp.quat(*q_og.astype(np.float32)))
            if CAM_PERSP:
                wp.launch(krn.raycast_height_persp, dim=(GS_H, GS_W),
                          inputs=[mesh.id, X_og, self.pad_w, self.pad_h,
                                  self.face_z, CAM_STANDOFF_M, 0.05, MAX_PEN_M],
                          outputs=[self._out])
            else:
                wp.launch(krn.raycast_height, dim=(GS_H, GS_W),
                          inputs=[mesh.id, X_og, self.pad_w, self.pad_h,
                                  self.face_z, 0.05, MAX_PEN_M],
                          outputs=[self._out])
        import hydrogel.render_kernels as krn
        if not hasattr(self, "_tmp"):
            self._tmp = wp.zeros((GS_H, GS_W), dtype=wp.float32)
            self._tmp2 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        sig = float(self._skirt_px)
        rad = int(4.0 * sig + 0.5)               # scipy default truncate=4.0
        wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[self._out, rad, sig], outputs=[self._tmp])
        wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[self._tmp, rad, sig], outputs=[self._tmp2])
        wp.launch(krn.max_into, dim=(GS_H, GS_W), inputs=[self._tmp2], outputs=[self._out])
        return self._out


    def _finish_height(self):
        d = self._out.numpy().astype(np.float64)
        # gel membrane: sharp penetration + wide Gaussian skirt
        skirt = ndimage.gaussian_filter(d, sigma=self._skirt_px)
        return np.maximum(d, skirt)


def _matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> quaternion (xyzw)."""

    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(max(1.0 + R[i, i] - R[j, j] - R[k, k], 1e-12)) * 2
    q = np.empty(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def draw_marker_arrows(rgb: np.ndarray, rest_px: np.ndarray,
                       centers_px: np.ndarray, amp: float = 2.0,
                       min_px: float = 0.3) -> np.ndarray:
    """Overlay amplified displacement arrows (rest -> current) on the tile.
    Marker motion is often a near-uniform shift or a slow vortex that dots
    alone hide; every video draws these by default (GS_MARKER_ARROWS=0
    disables, GS_MARKER_AMP changes the amplification)."""

    if os.environ.get("GS_MARKER_ARROWS", "1") == "0":
        return rgb
    amp = float(os.environ.get("GS_MARKER_AMP", str(amp)))
    from PIL import Image, ImageDraw

    img = Image.fromarray(np.ascontiguousarray(rgb))
    dr = ImageDraw.Draw(img)
    for (r0, c0), (r1, c1) in zip(rest_px, centers_px):
        dvr, dvc = (r1 - r0) * amp, (c1 - c0) * amp
        if dvr * dvr + dvc * dvc < min_px * min_px:
            continue
        dr.line([(c0, r0), (c0 + dvc, r0 + dvr)], fill=(255, 70, 40), width=2)
    return np.asarray(img)


def project_markers(centers_px: np.ndarray, height_img: np.ndarray) -> np.ndarray:
    """Pinhole projection of the markers (2026-08-24): a marker riding a
    surface pressed toward the camera by h appears shifted radially OUT from
    the image centre by the factor d/(d - h) (d = CAM_STANDOFF_M) -- the
    dominant "apparent dilation" a real GelSight shows under pure pressing,
    on top of the HydroShear mechanical displacement."""

    if not CAM_PERSP:
        return centers_px
    r = np.clip(centers_px[:, 0].astype(int), 0, height_img.shape[0] - 1)
    c = np.clip(centers_px[:, 1].astype(int), 0, height_img.shape[1] - 1)
    h = np.maximum(height_img[r, c], 0.0)
    scale = CAM_STANDOFF_M / np.maximum(CAM_STANDOFF_M - h, 1.0e-4)
    ctr = np.array([(height_img.shape[0] - 1) / 2.0,
                    (height_img.shape[1] - 1) / 2.0])
    return ctr + (centers_px - ctr) * scale[:, None]


def marker_grid() -> np.ndarray:
    """Rest marker centers, shape (N, 2) as (row_h, col_w) image pixels."""

    hs = np.linspace(MARKER_MARGIN_PX, GS_H - MARKER_MARGIN_PX, MARKER_GRID[0])
    ws = np.linspace(MARKER_MARGIN_PX, GS_W - MARKER_MARGIN_PX, MARKER_GRID[1])
    return np.stack(np.meshgrid(hs, ws, indexing="ij"), axis=-1).reshape(-1, 2)


def advect_markers(shear_m: np.ndarray, grid_px: np.ndarray) -> np.ndarray:
    """Displace markers by the sensor shear field.

    ``shear_m``: (2, rows, cols) shear displacement [m], components along
    gel x and gel y.  Marker motion: gel x -> image h, gel y -> image w.
    """

    rows, cols = shear_m.shape[1], shear_m.shape[2]
    # marker position in sensor raster coordinates
    r_idx = grid_px[:, 1] / (GS_W - 1) * (rows - 1)   # image w -> gel y/rows
    c_idx = grid_px[:, 0] / (GS_H - 1) * (cols - 1)   # image h -> gel x/cols
    coords = np.stack([r_idx, c_idx])
    dx = ndimage.map_coordinates(shear_m[0], coords, order=1)   # gel x [m]
    dy = ndimage.map_coordinates(shear_m[1], coords, order=1)   # gel y [m]
    disp_px = np.stack([dx, dy], axis=-1) * 1000.0 / PIXMM
    return grid_px + disp_px


def warp_marker_layer(rgb: np.ndarray, height_img: np.ndarray,
                      shear_m: np.ndarray,
                      membrane_sigma_mm: float = 2.0) -> np.ndarray:
    """Texture-warp marker rendering (2026-08-23).

    For every output pixel p, the source point on the REST marker texture
    is q = p - u(p), with u the local in-plane displacement in pixels:
    the membrane-blurred stick/slip shear field (identical physics to
    gelsight_markers.field_markers, evaluated per PIXEL instead of per
    marker centre) plus the pinhole term -- a surface point pressed to
    height h appears at ctr + (p_rest - ctr) * d/(d-h), whose inverse is
    q = p - (p - ctr) * h/d with h taken at p (small-shift approximation,
    the same order as the whole marker model).  The dot shade is computed
    ANALYTICALLY from q's distance to its nearest rest-grid centre, so a
    dot whose pixels carry different u bends, stretches and creases --
    the half-on-the-bump / half-on-the-plane kink a real GelSight shows.
    """

    rows, cols = shear_m.shape[1], shear_m.shape[2]
    sig_cells = membrane_sigma_mm / PIXMM * rows / float(GS_H)
    ux = ndimage.gaussian_filter(shear_m[0].astype(np.float64), sig_cells)
    uy = ndimage.gaussian_filter(shear_m[1].astype(np.float64), sig_cells)
    # raster (rows = gel y = image w, cols = gel x = image h) -> image
    z = (GS_H / cols, GS_W / rows)
    dpr = ndimage.zoom(np.ascontiguousarray(ux.T), z, order=1,
                       mode="nearest") * (1000.0 / PIXMM)   # image rows
    dpc = ndimage.zoom(np.ascontiguousarray(uy.T), z, order=1,
                       mode="nearest") * (1000.0 / PIXMM)   # image cols
    rr, cc = np.meshgrid(np.arange(GS_H, dtype=np.float64),
                         np.arange(GS_W, dtype=np.float64), indexing="ij")
    qr, qc = rr - dpr, cc - dpc
    if CAM_PERSP:
        f = np.maximum(height_img, 0.0) / CAM_STANDOFF_M
        qr -= (rr - (GS_H - 1) / 2.0) * f
        qc -= (cc - (GS_W - 1) / 2.0) * f
    # nearest rest-grid dot, analytically (the rest grid is regular)
    n_h, n_w = MARKER_GRID
    step_h = (GS_H - 2.0 * MARKER_MARGIN_PX) / (n_h - 1)
    step_w = (GS_W - 2.0 * MARKER_MARGIN_PX) / (n_w - 1)
    ih = np.clip(np.round((qr - MARKER_MARGIN_PX) / step_h), 0, n_h - 1)
    iw = np.clip(np.round((qc - MARKER_MARGIN_PX) / step_w), 0, n_w - 1)
    dist = np.hypot(qr - (MARKER_MARGIN_PX + ih * step_h),
                    qc - (MARKER_MARGIN_PX + iw * step_w))
    alpha = np.clip(MARKER_RADIUS_PX + 0.5 - dist, 0.0, 1.0)
    shade = 1.0 - (1.0 - MARKER_DARKEN) * alpha
    out = rgb.astype(np.float64) * shade[:, :, None]
    return np.clip(out, 0, 255).astype(np.uint8)


def draw_markers(img: np.ndarray, centers: np.ndarray) -> np.ndarray:
    """Anti-aliased dark dots (in place on a float copy; returns uint8)."""

    out = img.astype(np.float64)
    rad = MARKER_RADIUS_PX
    for ch, cw in centers:
        h0, h1 = int(max(0, ch - rad - 2)), int(min(GS_H, ch + rad + 3))
        w0, w1 = int(max(0, cw - rad - 2)), int(min(GS_W, cw + rad + 3))
        if h0 >= h1 or w0 >= w1:
            continue
        hh, ww = np.meshgrid(np.arange(h0, h1), np.arange(w0, w1),
                             indexing="ij")
        dist = np.sqrt((hh - ch) ** 2 + (ww - cw) ** 2)
        alpha = np.clip(rad + 0.5 - dist, 0.0, 1.0)          # AA edge
        shade = 1.0 - (1.0 - MARKER_DARKEN) * alpha
        out[h0:h1, w0:w1] *= shade[:, :, None]
    return np.clip(out, 0, 255).astype(np.uint8)


_INFERNO_LUT = None


def pressure_tile(pressure_pa: np.ndarray, vmax_pa: float) -> np.ndarray:
    """Colormapped pressure raster at image resolution (uint8 HxWx3)."""

    global _INFERNO_LUT
    if _INFERNO_LUT is None:
        from matplotlib import cm
        _INFERNO_LUT = (cm.inferno(np.linspace(0.0, 1.0, 256))[:, :3] * 255).astype(np.uint8)
    # linear resample + 256-entry LUT: 29 ms -> ~2 ms per tile (measured);
    # cubic + per-pixel colormap evaluation was 60 % of a strip frame
    a = np.ascontiguousarray(pressure_pa.T)
    img = ndimage.zoom(a, (GS_H / a.shape[0], GS_W / a.shape[1]), order=1, mode="nearest")
    idx = np.clip(img / max(vmax_pa, 1.0) * 255.0, 0.0, 255.0).astype(np.uint8)
    return _INFERNO_LUT[idx]


class TactileTileWriter:
    """Render per-frame tactile tiles and write the strip video.

    ``geometry``: optional GeometricHeightmap.  When present, the tactile
    RGB is rendered from the raycast (geometry-derived) height map at full
    image resolution; otherwise it falls back to upsampling the sensor's
    ``normal_displacement_m`` raster.
    """

    def __init__(self, renderer: TaximRenderer | None = None,
                 geometry: "GeometricHeightmap | None" = None,
                 pad_size_m: tuple[float, float] | None = None) -> None:
        self.renderer = renderer or TaximRenderer()
        self.geometry = geometry
        self.grid = marker_grid()
        if pad_size_m is None:
            pad_size_m = ((geometry.pad_w, geometry.pad_h) if geometry is not None
                          else (0.0186, 0.0143))
        self.markers = HydroShearMarkers(self.grid, (GS_H, GS_W), pad_size_m)

    def marker_positions(self, height_m: np.ndarray, shear_m: np.ndarray) -> np.ndarray:
        if MARKER_MODEL == "local":
            return project_markers(advect_markers(shear_m, self.grid), height_m)
        if MARKER_MODEL == "hydroshear":
            pos, _dil, _shr = self.markers.displace(height_m, shear_m)
            return project_markers(pos, height_m)
        from hydrogel.markers import field_markers
        return project_markers(
            field_markers(self.grid, shear_m, (GS_H, GS_W)), height_m)

    def _bulge_np(self, h: np.ndarray) -> np.ndarray:
        if BULGE_K <= 0.0:
            return h
        ring = np.maximum(ndimage.gaussian_filter(h, BULGE_SIGMA_MM / PIXMM) - h, 0.0)
        return h - BULGE_K * ring

    def _height(self, pad_data: dict, pad_name: str, index: int) -> np.ndarray:
        # SOLVER-CONSISTENT HEIGHT (2026-08-23): the sensor's normal
        # displacement is the pressure law's inverse of the SOLVER-scaled
        # pressure (gelsight_sensor: the raster's integral is the force the
        # solver actually applied), so where the solver carries load through
        # a geometric contact that is shallower than the law predicts -- a
        # bow edge on a gel corner, a bottomed ridge -- the image shows the
        # indentation that load implies.  The geometric raycast keeps the
        # sharp object shape wherever the two agree; the per-pixel max
        # keeps both.  GS_RENDER_HEIGHT=geom restores the raycast alone.
        mech = raster_to_image(pad_data["normal_displacement_m"][index])
        if self.geometry is not None and index < self.geometry.n_frames():
            geom = self.geometry.height(index, pad_name)
            if os.environ.get("GS_RENDER_HEIGHT", "max") == "geom":
                return self._bulge_np(geom)
            return self._bulge_np(np.maximum(geom, mech))
        return self._bulge_np(mech)

    def _height_and_rgb_gpu(self, pad_data: dict, pad_name: str, index: int):
        """Device path: raycast (+skirt) and the mechanics raster are max'ed
        on the GPU, Taxim shades there too; height comes back only for the
        marker model.  Falls back to the CPU path when there is no geometry
        and GS_RENDER_HEIGHT=geom semantics are kept."""

        import hydrogel.render_kernels as krn
        if not hasattr(self, "_mech_dev"):
            self._mech_dev = None
            self._hdev = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        mech = np.ascontiguousarray(pad_data["normal_displacement_m"][index], dtype=np.float32)
        if self._mech_dev is None or self._mech_dev.shape != mech.shape:
            self._mech_dev = wp.array(mech, dtype=wp.float32)
        else:
            self._mech_dev.assign(mech)
        use_geom = self.geometry is not None and index < self.geometry.n_frames()
        if use_geom:
            hdev = self.geometry.height_device(index, pad_name)   # geometry's own buffer
            if os.environ.get("GS_RENDER_HEIGHT", "max") != "geom":
                wp.launch(krn.upsample_raster_max, dim=(GS_H, GS_W),
                          inputs=[self._mech_dev], outputs=[hdev])
        else:
            hdev = self._hdev
            hdev.zero_()
            wp.launch(krn.upsample_raster_max, dim=(GS_H, GS_W),
                      inputs=[self._mech_dev], outputs=[hdev])
        hdev = self._apply_bulge(hdev)
        rgb = self.renderer.render_device(hdev)
        self._last_hdev = hdev
        return hdev.numpy().astype(np.float64), rgb

    def _apply_bulge(self, hdev):
        if BULGE_K <= 0.0:
            return hdev
        import hydrogel.render_kernels as krn
        if not hasattr(self, "_bulge_tmp"):
            self._bulge_tmp = wp.zeros((GS_H, GS_W), dtype=wp.float32)
            self._bulge_tmp2 = wp.zeros((GS_H, GS_W), dtype=wp.float32)
            self._bulge_out = wp.zeros((GS_H, GS_W), dtype=wp.float32)
        sig = BULGE_SIGMA_MM / PIXMM
        rad = int(4.0 * sig + 0.5)
        wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[hdev, rad, float(sig)], outputs=[self._bulge_tmp])
        wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[self._bulge_tmp, rad, float(sig)], outputs=[self._bulge_tmp2])
        wp.launch(krn.bulge_ring, dim=(GS_H, GS_W), inputs=[hdev, self._bulge_tmp2, float(BULGE_K)], outputs=[self._bulge_out])
        return self._bulge_out

    def marker_positions_gpu(self, shear_m: np.ndarray) -> np.ndarray:
        h = self._last_hdev.numpy()
        if MARKER_MODEL == "local":
            return project_markers(advect_markers(shear_m, self.grid), h)
        if MARKER_MODEL == "hydroshear":
            return project_markers(
                self.markers.displace_device(self._last_hdev, shear_m), h)
        from hydrogel.markers import field_markers
        return project_markers(field_markers(self.grid, shear_m, (GS_H, GS_W)), h)

    def frame_tiles(self, left: dict, right: dict, vmax_pa: float,
                    index: int) -> np.ndarray:
        tiles = []
        gpu = os.environ.get("GS_RENDER_GPU", "1") != "0"
        for pad_data, pad_name in ((left, "gel_left"), (right, "gel_right")):
            if gpu:
                height, rgb = self._height_and_rgb_gpu(pad_data, pad_name, index)
                centers = self.marker_positions_gpu(pad_data["shear_displacement_m"][index])
            else:
                height = self._height(pad_data, pad_name, index)
                rgb = self.renderer.render(height)
                centers = self.marker_positions(
                    height, pad_data["shear_displacement_m"][index])
            shear = pad_data["shear_displacement_m"][index]
            if MARKER_RENDER == "warp":
                tile = warp_marker_layer(rgb, height, shear)
            else:
                tile = draw_markers(rgb, centers)
            tile = draw_marker_arrows(tile, self.grid, centers)
            tiles.append(tile)
        for pad_data in (left, right):
            tiles.append(pressure_tile(pad_data["pressure_pa"][index], vmax_pa))
        return np.concatenate(tiles, axis=1)      # (320, 4*240, 3)

    def write_strip(self, left: dict, right: dict, out_path: Path,
                    fps: int = 60) -> Path:
        import imageio.v2 as iio

        n = min(len(left["pressure_pa"]), len(right["pressure_pa"]))
        # p99.5 colormap scale: a single saturated frame (bottomed pad at
        # p_sat) used to set vmax for the whole video and crush every
        # normal frame to near-black
        import numpy as _np
        vmax = max(float(_np.percentile(left["pressure_pa"], 99.5)),
                   float(_np.percentile(right["pressure_pa"], 99.5)), 1.0)
        writer = iio.get_writer(out_path, fps=fps, quality=8,
                                macro_block_size=None)
        try:
            for i in range(n):
                writer.append_data(self.frame_tiles(left, right, vmax, i))
        finally:
            writer.close()
        return out_path


def compose_panel(task_dir: Path, strip_path: Path,
                  out_name: str = "rollout_panel_tactile.mp4") -> Path | None:
    """Stack the multiview rollout video over the tactile strip."""

    import imageio.v2 as iio

    video_path = task_dir / "rollout_multiview.mp4"
    if not video_path.exists():
        video_path = task_dir / "rollout.mp4"
    if not video_path.exists():
        print(f"[gelsight-render] no rollout video in {task_dir}, "
              "skipping panel")
        return None
    vid = iio.get_reader(video_path)
    strip = iio.get_reader(strip_path)
    meta = vid.get_meta_data()
    out_path = task_dir / out_name
    writer = iio.get_writer(out_path, fps=meta.get("fps", 60), quality=8,
                            macro_block_size=None)
    try:
        for frame, tactile in zip(vid, strip):
            # PIL bilinear resize: 2.6 ms vs 32 ms for ndimage.zoom per frame
            from PIL import Image
            scale = frame.shape[1] / tactile.shape[1]
            resized = np.asarray(Image.fromarray(np.ascontiguousarray(tactile)).resize(
                (frame.shape[1], int(round(tactile.shape[0] * scale))),
                Image.BILINEAR))[:, :frame.shape[1]]
            combined = np.concatenate(
                [frame, resized.astype(frame.dtype)], axis=0)
            # libx264 requires even dimensions
            combined = combined[:combined.shape[0] // 2 * 2,
                                :combined.shape[1] // 2 * 2]
            writer.append_data(combined)
    finally:
        writer.close()
        vid.close()
        strip.close()
    return out_path


def render_task_dir(task_dir: str | Path, fps: int = 60,
                    panel: bool = False) -> Path:
    task_dir = Path(task_dir)
    left = dict(np.load(task_dir / "tactile_left.npz"))
    right = dict(np.load(task_dir / "tactile_right.npz"))
    geometry = None
    scene_path = task_dir / "tactile_scene.npz"
    states_path = task_dir / "rollout_states.npz"
    if scene_path.exists() and states_path.exists():
        geometry = GeometricHeightmap(scene_path, states_path)
        print("[gelsight-render] geometry-derived height maps "
              f"({len(geometry.obj_meshes)} object(s))")
    strip = TactileTileWriter(geometry=geometry).write_strip(
        left, right, task_dir / "tactile_strip.mp4", fps=fps)
    print(f"[gelsight-render] wrote {strip}")
    if panel:
        p = compose_panel(task_dir, strip)
        if p:
            print(f"[gelsight-render] wrote {p}")
    return strip


def _selftest(out_dir: Path) -> int:
    """Hemisphere press: rendered image must differ from the background
    inside the contact and match it outside; markers must shift with an
    imposed shear field."""

    import imageio.v2 as iio

    renderer = TaximRenderer()
    hh, ww = np.meshgrid(np.arange(GS_H), np.arange(GS_W), indexing="ij")
    r_px = 4.0 / PIXMM * 0.5                       # 4 mm sphere -> ~30 px cap
    d2 = (hh - GS_H / 2) ** 2 + (ww - GS_W / 2) ** 2
    indent = np.where(d2 < r_px ** 2,
                      np.sqrt(np.maximum(r_px ** 2 - d2, 0.0)) * PIXMM / 1000.0,
                      0.0) * 0.35                  # peak ~0.35 mm
    rgb = renderer.render(indent)

    contact = d2 < (0.8 * r_px) ** 2
    outside = d2 > (3.0 * r_px) ** 2
    diff = np.abs(rgb.astype(float) - renderer.background[:, :, :3])
    inside_delta = float(diff[contact].mean())
    outside_delta = float(diff[outside].mean())

    shear = np.zeros((2, 96, 128), dtype=np.float32)
    shear[0] = 0.0005                              # 0.5 mm of +x shear
    grid = marker_grid()
    moved = advect_markers(shear, grid)
    shift_px = float(np.abs(moved - grid).max())
    # HydroShear terms: dilation must push markers radially OUT of the
    # hemisphere contact; kernel-spread shear must move markers along +x
    hs = HydroShearMarkers(grid, (GS_H, GS_W), (0.0186, 0.0143))
    _pos, dil, shr = hs.displace(indent, shear)
    radial = (grid - np.array([GS_H / 2, GS_W / 2]))
    radial /= np.maximum(np.linalg.norm(radial, axis=1, keepdims=True), 1e-9)
    dil_radial = float(np.mean(np.sum(dil * radial, axis=1)))
    shr_x = float(np.mean(shr[:, 0]))
    rgb_m = draw_markers(rgb, moved)

    out_dir.mkdir(parents=True, exist_ok=True)
    iio.imwrite(out_dir / "selftest_hemisphere.png", rgb_m)
    ok = (inside_delta > 5.0 and outside_delta < 2.0 and 7.0 < shift_px < 8.5
          and dil_radial > 0.05 and shr_x > 0.0)
    print(f"[gelsight-render] selftest: contact delta {inside_delta:.1f}, "
          f"background delta {outside_delta:.2f}, marker shift "
          f"{shift_px:.2f} px (expect ~{0.0005 * 1000 / PIXMM:.2f}), "
          f"HydroShear dilation {dil_radial:+.2f} px radial (max "
          f"{np.linalg.norm(dil, axis=1).max():.2f}), kernel shear "
          f"{shr_x:+.2f} px along +x -> {'PASS' if ok else 'FAIL'}")
    print(f"[gelsight-render] wrote {out_dir / 'selftest_hemisphere.png'}")
    return 0 if ok else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-dir", default=None,
                        help="task output dir with tactile_{left,right}.npz")
    parser.add_argument("--panel", action="store_true",
                        help="also compose rollout_panel_tactile.mp4")
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--selftest", action="store_true")
    args = parser.parse_args()
    if args.selftest:
        return _selftest(GELSIGHT_OUTPUT_DIR / "gelsight_render")
    if not args.task_dir:
        parser.error("--task-dir or --selftest required")
    render_task_dir(args.task_dir, fps=args.fps, panel=args.panel)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
