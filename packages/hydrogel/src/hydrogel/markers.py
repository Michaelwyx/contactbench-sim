"""HydroShear-style marker displacement field for the GelSight renderer.

HydroShear (Dang et al., RSS 2026; github.com/MMintLab/hydroshear,
``rl/hydroshear_sensors/hydroshear_sensors.py``) models marker motion as two
Gaussian-kernel superpositions over taxel / contact points, fitted to a real
GelSight Mini:

    dilation  M_d[i] = s_d * SUM_j h_j (p_i - p_j) exp(-lambda_d |p_i - p_j|^2)
    shear     M_s[i] = s_s * SUM_j h_j f_j  exp(-lambda_s |p_i - a_j|^2),
              a_j = p_j + f_j                        ("affected" position)

with h_j the normal "force" and f_j the accumulated tangential "force" of
contact point j.  HydroShear lumps E = K = A = 1, so h_j IS the accumulated
penetration depth [m] and f_j IS the accumulated, Coulomb-clamped tangential
displacement [m] -- which are exactly this sensor's geometric height map and
its per-cell shear-displacement state.  The mapping is therefore literal:

    h_j  <- geometric height map (raycast penetration) at taxel j
    f_j  <- HydroelasticGelSight shear_displacement_m at taxel j
    p_i  <- marker rest positions, p_j <- taxel grid, all in gel metres
    output in pixels via s_d = s_s = 1000 / 0.065 (GS Mini pixmm)

Constants are HydroShear's fitted values (lambda_d = 20000, lambda_s = 10800,
mu 0.5-0.75 lives in the sensor).  One thing theirs leaves implicit: the
sums are over a discrete point set, so their magnitude scales with point
density.  We evaluate on a taxel grid at our own resolution and rescale each
term by (taxel area / HydroShear's reference taxel area), i.e. the result is
what their formula gives at THEIR density (MeshSDFSensor default
num_divs=[20,30] on a 20.75 x 25.25 mm elastomer = 0.87 mm^2 per taxel).
``dilation_gain`` / ``shear_gain`` are the knobs to refit against your own
marker data, exactly as HydroShear refit lambda and scale to their unit.

Sign: our shear state integrates (object - gel) tangential velocity, so it
points along the object's motion and markers follow it with +s_s (HydroShear
writes -s_s because their f_t is the reaction on the indenter).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage


@dataclass(frozen=True)
class HydroShearMarkerConfig:
    lambda_d: float = 20000.0          # 1/m^2, dilation kernel (HydroShear)
    lambda_s: float = 10800.0          # 1/m^2, shear kernel (HydroShear)
    pixmm: float = 0.065               # GS Mini mm per pixel
    dilation_gain: float = 1.0
    shear_gain: float = 1.0
    taxel_area_ref_m2: float = 0.87e-6  # HydroShear [20,30] grid on its pad
    taxel_stride_px: int = 8            # taxel every N image pixels


class HydroShearMarkers:
    """Marker displacements from the geometric height map + shear state.

    ``marker_grid_px``: (N, 2) rest positions as (row_h, col_w) image pixels.
    Image h runs along gel x (pad width), image w along gel y (pad height).
    """

    def __init__(self, marker_grid_px: np.ndarray, image_hw: tuple[int, int],
                 pad_size_m: tuple[float, float],
                 cfg: HydroShearMarkerConfig | None = None) -> None:
        self.cfg = cfg or HydroShearMarkerConfig()
        self.H, self.W = image_hw
        self.pad_w, self.pad_h = pad_size_m
        self.grid_px = np.asarray(marker_grid_px, dtype=np.float64)
        self.p_i = self._px_to_gel(self.grid_px)           # (N, 2) gel metres
        s = self.cfg.taxel_stride_px
        hh = np.arange(s // 2, self.H, s)
        ww = np.arange(s // 2, self.W, s)
        th, tw = np.meshgrid(hh, ww, indexing="ij")
        self.taxel_px = np.stack([th.ravel(), tw.ravel()], axis=-1).astype(np.float64)
        self.p_j = self._px_to_gel(self.taxel_px)          # (M, 2)
        self.taxel_area = (s * self.pad_w / self.H) * (s * self.pad_h / self.W)
        self.density_scale = self.taxel_area / self.cfg.taxel_area_ref_m2
        self.scale_px_per_m = 1000.0 / self.cfg.pixmm

    def _px_to_gel(self, px: np.ndarray) -> np.ndarray:
        x = (px[:, 0] + 0.5) / self.H * self.pad_w - 0.5 * self.pad_w
        y = (px[:, 1] + 0.5) / self.W * self.pad_h - 0.5 * self.pad_h
        return np.stack([x, y], axis=-1)

    def _sample_shear(self, shear_m: np.ndarray) -> np.ndarray:
        """Sensor shear raster (2, rows, cols) -> (M, 2) gel-frame metres."""

        rows, cols = shear_m.shape[1], shear_m.shape[2]
        r_idx = self.taxel_px[:, 1] / (self.W - 1) * (rows - 1)   # image w -> gel y/rows
        c_idx = self.taxel_px[:, 0] / (self.H - 1) * (cols - 1)   # image h -> gel x/cols
        coords = np.stack([r_idx, c_idx])
        ux = ndimage.map_coordinates(shear_m[0], coords, order=1)
        uy = ndimage.map_coordinates(shear_m[1], coords, order=1)
        return np.stack([ux, uy], axis=-1)

    def displace_device(self, height_dev, shear_m: np.ndarray | None) -> np.ndarray:
        """GPU version of ``displace`` (marker positions only): ``height_dev``
        is the (H, W) float32 wp.array the renderer already holds; the
        shear raster is uploaded (2 x rows x cols).  Returns marker_px (N,2)."""

        import warp as wp
        import hydrogel.render_kernels as krn

        cfg = self.cfg
        if getattr(self, "_dev", None) is None:
            self._dev = dict(
                taxel=wp.array(self.taxel_px.astype(np.int32), dtype=wp.vec2i),
                p_j=wp.array(self.p_j.astype(np.float32), dtype=wp.vec2),
                p_i=wp.array(self.p_i.astype(np.float32), dtype=wp.vec2),
                out=wp.zeros(len(self.p_i), dtype=wp.vec2),
                shear=None)
        d = self._dev
        if shear_m is None:
            shear_np = np.zeros((2, 2, 2), dtype=np.float32)
        else:
            shear_np = np.ascontiguousarray(shear_m, dtype=np.float32)
        if d["shear"] is None or d["shear"].shape != shear_np.shape:
            d["shear"] = wp.array(shear_np, dtype=wp.float32)
        else:
            d["shear"].assign(shear_np)
        k = self.density_scale * self.scale_px_per_m
        wp.launch(krn.hydroshear_markers, dim=len(self.p_i),
                  inputs=[height_dev, d["shear"], d["taxel"], d["p_j"], d["p_i"],
                          float(cfg.lambda_d), float(cfg.lambda_s),
                          float(cfg.dilation_gain * k), float(cfg.shear_gain * k),
                          int(self.H), int(self.W)],
                  outputs=[d["out"]])
        return self.grid_px + d["out"].numpy().astype(np.float64)

    def displace(self, height_m: np.ndarray, shear_m: np.ndarray | None):
        """Return (marker_px (N,2), dilation_px (N,2), shear_px (N,2)).

        ``height_m``: (H, W) gel indentation [m] at image resolution (the
        geometric height map); ``shear_m``: sensor shear raster or None.
        """

        cfg = self.cfg
        h_j = height_m[self.taxel_px[:, 0].astype(int), self.taxel_px[:, 1].astype(int)]
        h_j = np.maximum(h_j.astype(np.float64), 0.0)
        active = h_j > 0.0
        dil = np.zeros_like(self.p_i)
        shr = np.zeros_like(self.p_i)
        if active.any():
            pj, hj = self.p_j[active], h_j[active]
            d = self.p_i[:, None, :] - pj[None, :, :]                 # (N, M, 2)
            w = hj[None, :] * np.exp(-cfg.lambda_d * np.sum(d * d, axis=-1))
            dil = (cfg.dilation_gain * self.density_scale * self.scale_px_per_m
                   * np.sum(w[:, :, None] * d, axis=1))
            if shear_m is not None:
                f_j = self._sample_shear(shear_m)[active]             # (M, 2) metres
                a_j = pj + f_j
                da = self.p_i[:, None, :] - a_j[None, :, :]
                ws = hj[None, :] * np.exp(-cfg.lambda_s * np.sum(da * da, axis=-1))
                shr = (cfg.shear_gain * self.density_scale * self.scale_px_per_m
                       * np.sum(ws[:, :, None] * f_j[None, :, :], axis=1))
        return self.grid_px + dil + shr, dil, shr

def field_markers(grid_px: np.ndarray, shear_m: np.ndarray,
                  image_hw: tuple[int, int], pixmm: float = 0.065,
                  membrane_sigma_mm: float = 2.0) -> np.ndarray:
    """STICK-FOLLOWING marker model (2026-08-24, the default).

    Inside the contact the gel surface sticks to the object up to the
    Coulomb bound, so a marker moves by the LOCAL stored shear displacement
    -- exactly the sensor's per-cell ``shear_displacement_m`` field (which
    already contains the stick/slip clamp and the rotation vortex).  The
    HydroShear kernel model smeared this into a near-uniform shift and
    under-predicted in-contact motion by ~10x.  Outside the contact the
    membrane drags neighbours along with a Gaussian falloff
    (``membrane_sigma_mm``).  Returns marker positions in image px.
    """

    H, W = image_hw
    sig_cells = membrane_sigma_mm / pixmm * shear_m.shape[1] / float(H)
    ux = ndimage.gaussian_filter(shear_m[0].astype(np.float64), sig_cells)
    uy = ndimage.gaussian_filter(shear_m[1].astype(np.float64), sig_cells)
    rows, cols = shear_m.shape[1], shear_m.shape[2]
    r_idx = grid_px[:, 1] / (W - 1) * (rows - 1)     # image w -> raster rows (gel y)
    c_idx = grid_px[:, 0] / (H - 1) * (cols - 1)     # image h -> raster cols (gel x)
    coords = np.stack([r_idx, c_idx])
    mx = ndimage.map_coordinates(ux, coords, order=1)   # gel x -> image h
    my = ndimage.map_coordinates(uy, coords, order=1)   # gel y -> image w
    disp_px = np.stack([mx, my], axis=-1) * (1000.0 / pixmm)
    return grid_px + disp_px
