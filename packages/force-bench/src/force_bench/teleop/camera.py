"""Offscreen camera for the web teleop view (and headless snapshots).

Wraps ``newton.sensors.SensorTiledCamera`` the way ``RolloutRecorder``
does -- same look-at convention, same key/fill light rig, same backdrop
compositing kernel -- but renders one small view into a fixed host
buffer with no video writer, and exposes an ORBIT state (target, distance,
azimuth, elevation) the web client drives, plus :meth:`project` so contact
force arrows can be drawn over the JPEG in pixel space.
"""

from __future__ import annotations

import numpy as np
import warp as wp

import newton

from force_bench.recorder import (
    BG_BOTTOM, BG_TOP, FILL_DIR, KEY_DIR, _resolve_frame_kernel, lookat_transform,
)


def _quat_to_mat(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TeleopCamera:
    def __init__(self, model: newton.Model, cam_pos, cam_target, width: int = 800,
                 height: int = 600, fov_deg: float = 40.0, supersample: int = 1):
        from newton.sensors import SensorTiledCamera
        self.model = model
        self.width, self.height = int(width), int(height)
        self.ss = int(supersample)
        self.fov = float(np.radians(fov_deg))
        rw, rh = self.width * self.ss, self.height * self.ss
        self.cam = SensorTiledCamera(model=model)
        self.cam.default_render_config.enable_shadows = True
        self.cam.utils.create_default_light(enable_shadows=True)
        self._light_rig()
        self.rays = self.cam.utils.compute_camera_rays_pinhole(
            rw, rh, camera_fovs=np.array([self.fov]))
        self.color = self.cam.utils.create_color_image_output(rw, rh, 1)
        self.sidx = self.cam.utils.create_shape_index_image_output(rw, rh, 1)
        self.out_img = wp.zeros((self.height, self.width, 3), dtype=wp.uint8)
        self._img_host = wp.zeros((self.height, self.width, 3), dtype=wp.uint8, device="cpu")
        # orbit state
        self.target = np.asarray(cam_target, dtype=float).copy()
        self.home = (np.asarray(cam_pos, dtype=float).copy(), self.target.copy())
        self._set_from_pos(np.asarray(cam_pos, dtype=float))

    def _light_rig(self):
        ctx = getattr(self.cam, "_SensorTiledCamera__render_context", None)
        if ctx is None:
            return
        dev = ctx.device
        ctx.lights_active = wp.array([True, True], dtype=wp.bool, device=dev)
        ctx.lights_type = wp.array([1, 1], dtype=wp.int32, device=dev)
        ctx.lights_cast_shadow = wp.array([True, False], dtype=wp.bool, device=dev)
        ctx.lights_position = wp.array([wp.vec3f(0.0)] * 2, dtype=wp.vec3f, device=dev)
        ctx.lights_orientation = wp.array(
            [wp.vec3f(*KEY_DIR), wp.vec3f(*FILL_DIR)], dtype=wp.vec3f, device=dev)

    # ── orbit ─────────────────────────────────────────────────────────
    def _set_from_pos(self, pos):
        d = pos - self.target
        self.dist = float(max(np.linalg.norm(d), 0.05))
        self.az = float(np.arctan2(d[1], d[0]))
        self.el = float(np.arcsin(np.clip(d[2] / self.dist, -1.0, 1.0)))
        self._update_tf()

    @property
    def pos(self) -> np.ndarray:
        ce = np.cos(self.el)
        return self.target + self.dist * np.array([ce * np.cos(self.az), ce * np.sin(self.az), np.sin(self.el)])

    def _update_tf(self):
        tf = lookat_transform(self.pos, self.target)
        self.cam_tf = wp.array([[tf]], dtype=wp.transformf)
        p = np.array([tf[0], tf[1], tf[2]], dtype=float)
        q = np.array([tf[3], tf[4], tf[5], tf[6]], dtype=float)
        self._R = _quat_to_mat(q)           # columns: right, up, -forward
        self._p = p

    def orbit(self, d_az_deg: float = 0.0, d_el_deg: float = 0.0, zoom: float = 1.0,
              pan=None):
        self.az += np.radians(d_az_deg)
        self.el = float(np.clip(self.el + np.radians(d_el_deg), np.radians(-85), np.radians(85)))
        self.dist = float(np.clip(self.dist * zoom, 0.05, 5.0))
        if pan is not None:
            # pan in the camera's right/up plane, metres
            self.target = self.target + self._R[:, 0] * pan[0] + self._R[:, 1] * pan[1]
        self._update_tf()

    def reset_view(self):
        self.target = self.home[1].copy()
        self._set_from_pos(self.home[0])

    def look_at(self, pos, target):
        self.target = np.asarray(target, dtype=float).copy()
        self._set_from_pos(np.asarray(pos, dtype=float))

    def pose(self) -> dict:
        return {"pos": [round(float(v), 4) for v in self.pos],
                "target": [round(float(v), 4) for v in self.target],
                "az_deg": round(float(np.degrees(self.az)), 2),
                "el_deg": round(float(np.degrees(self.el)), 2),
                "dist": round(self.dist, 4), "fov_deg": round(float(np.degrees(self.fov)), 2),
                "width": self.width, "height": self.height}

    # ── render ────────────────────────────────────────────────────────
    def render(self, state: newton.State) -> np.ndarray:
        """RGB uint8 (H, W, 3) on the host (a view of a fixed staging buffer)."""
        self.model.bvh_refit_shapes(state)
        self.cam.update(state, self.cam_tf, self.rays, color_image=self.color,
                        shape_index_image=self.sidx,
                        clear_data=type(self.cam).GRAY_CLEAR_DATA)
        wp.launch(_resolve_frame_kernel, dim=(self.height, self.width),
                  inputs=[self.color, self.sidx, self.out_img, self.ss,
                          self.height * self.ss, wp.vec3f(*BG_TOP), wp.vec3f(*BG_BOTTOM), 0, 0])
        wp.copy(self._img_host, self.out_img)
        return self._img_host.numpy()

    # ── projection (pinhole, vertical fov, matches compute_camera_rays_pinhole) ──
    def project(self, points) -> tuple[np.ndarray, np.ndarray]:
        """World points (N,3) -> pixel (N,2) + in-front mask."""
        P = np.atleast_2d(np.asarray(points, dtype=float))
        pc = (P - self._p) @ self._R                  # R^T (p - cam)
        z = -pc[:, 2]
        front = z > 1e-4
        zs = np.where(front, z, 1.0)
        h = np.tan(self.fov / 2.0)
        aspect = self.width / self.height
        u = (pc[:, 0] / zs) / (2.0 * h * aspect)
        v = (-pc[:, 1] / zs) / (2.0 * h)
        px = np.stack([(u + 0.5) * self.width, (v + 0.5) * self.height], axis=1)
        return px, front
