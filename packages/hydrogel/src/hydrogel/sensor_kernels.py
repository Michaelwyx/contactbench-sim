"""Warp kernels for the GelSight rasterizer (GPU-side contact-surface splat).

The NumPy rasterizer in gelsight_sensor.py copies every contact-surface
face (3 vertices, depth, shape pair) to the CPU each frame and works on
them with array ops: measured 100 ms/frame at 80k faces in a door-key
pinch (30 % of the frame, more than the MuJoCo step).
This module does the same arithmetic on the GPU and copies back only the
rows x cols force grid and a handful of sums:

  * per face: area, centroid, unit normal, pressure from the law of the
    pair's SECOND shape (linear, or the stiffening law when that shape is a
    gel), force = p * A * |n . z_gel|
  * area splat: every face is split into s^2 equal-area sub-triangles and
    each sub-centroid is bilinearly splatted into the gel-local raster
    (atomic adds) with the same out-of-bounds weight renormalization as
    the CPU path (``subsample_faces`` is the CPU twin)
  * force-weighted sums for the centre of pressure and for the relative
    tangential velocity of the object against the gel (shear history)

``HydroelasticGelSight.sample`` uses it when ``GS_SENSOR_GPU`` != "0";
the CPU path stays as the reference (the unit tests run both).
"""

from __future__ import annotations

import numpy as np
import warp as wp


@wp.func
def _pressure_law(depth: wp.float32, kh: wp.float32, gel_t: wp.float32,
                  stiffen: wp.float32) -> wp.float32:
    x = -depth
    if x <= 0.0:
        return 0.0
    if gel_t > 0.0 and stiffen > 0.0:
        r = x / gel_t
        return kh * x * (1.0 + stiffen * r * r)
    return kh * x


# AREA SPLAT (2026-08-25).  Newton's marching cubes tessellates the contact
# surface on the coarser SDF's voxel lattice (sdf_hydroelastic.py:158), so
# with ~1 face per raster cell a centroid-only splat alternates between 1
# and 2 centroids per cell and prints a fixed ±15-50 % plaid (beat of the
# 0.2 mm voxel lattice against the 0.145 mm cell: period 3.5 cells, measured
# in keychain_hang gel-on-gel and in press_key_bow).  Every face is now
# split into s^2 equal-area sub-triangles (s from its longest edge in cell
# units) and each sub-centroid carries f/s^2: the limit is the exact
# area-weighted integral, force / CoP / velocity sums are unchanged, and
# the raster becomes a continuous field.  GS_SPLAT_MAX_LEVEL caps s.
SPLAT_MAX_LEVEL = 6
SPLAT_EDGE_PER_LEVEL = 2.0    # sub-samples per cell along the longest edge


def splat_max_level() -> int:
    """Sub-triangle cap; GS_SPLAT_MAX_LEVEL=1 restores the centroid splat."""
    import os
    return max(1, int(os.environ.get("GS_SPLAT_MAX_LEVEL", SPLAT_MAX_LEVEL)))


def subsample_faces(vertices_local: np.ndarray, face_forces: np.ndarray,
                    cell_w: float, cell_h: float, max_level: int | None = None):
    """CPU twin of the kernel's sub-triangle splat.

    ``vertices_local``: (n, 3, 3) face vertices in the gel frame.  Returns
    (points (m, 3), forces (m,)) -- the s^2 equal-area sub-centroids of
    every face, each carrying f / s^2, so the raster is the area-weighted
    integral of the face pressure rather than a lattice of centroids.
    """
    if max_level is None:
        max_level = splat_max_level()
    v = np.asarray(vertices_local, dtype=np.float64)
    f = np.asarray(face_forces, dtype=np.float64)
    if len(f) == 0:
        return v.mean(axis=1), f
    d = np.stack([v[:, 1] - v[:, 0], v[:, 2] - v[:, 1], v[:, 0] - v[:, 2]])
    e_px = np.maximum(np.abs(d[..., 0]) / cell_w, np.abs(d[..., 1]) / cell_h).max(axis=0)
    level = np.clip(np.ceil(e_px * SPLAT_EDGE_PER_LEVEL), 1, max_level).astype(np.int64)
    pts, frc = [], []
    for s in np.unique(level):
        sel = level == s
        p0, e1, e2 = v[sel, 0], v[sel, 1] - v[sel, 0], v[sel, 2] - v[sel, 0]
        if s == 1:
            pts.append(p0 + (e1 + e2) / 3.0)
            frc.append(f[sel])
            continue
        i, j = np.meshgrid(np.arange(s), np.arange(s), indexing="ij")
        up = (i + j <= s - 1)
        dn = (i + j <= s - 2)
        a = np.concatenate([(i[up] + 1.0 / 3.0), (i[dn] + 2.0 / 3.0)]) / s
        b = np.concatenate([(j[up] + 1.0 / 3.0), (j[dn] + 2.0 / 3.0)]) / s
        assert len(a) == s * s
        pts.append((p0[:, None, :] + a[None, :, None] * e1[:, None, :]
                    + b[None, :, None] * e2[:, None, :]).reshape(-1, 3))
        frc.append(np.repeat(f[sel] / (s * s), s * s))
    return np.concatenate(pts), np.concatenate(frc)


@wp.func
def _splat_level(e_px: wp.float32, max_level: wp.int32) -> wp.int32:
    s = wp.int32(wp.ceil(e_px * SPLAT_EDGE_PER_LEVEL))
    if s < 1:
        s = 1
    if s > max_level:
        s = max_level
    return s


@wp.func
def _splat_point(
    c_l: wp.vec3,                 # sample point, gel frame
    fw: wp.float32,               # force carried by this sample
    v_gel: wp.vec3,               # gel-frame tangential velocity at the sample
    width: wp.float32,
    height: wp.float32,
    rows: wp.int32,
    cols: wp.int32,
    grid: wp.array2d(dtype=wp.float32),
    vgrid: wp.array3d(dtype=wp.float32),
):
    u = (c_l[0] / width + 0.5) * wp.float32(cols) - 0.5
    v = (c_l[1] / height + 0.5) * wp.float32(rows) - 0.5
    # samples OUTSIDE the imaged FOV must not be folded into the boundary
    # cells (the physical pad is larger than the FOV: off-image load stays
    # off-image, exactly like the hardware camera) -- they still count in
    # the force/CoP/velocity sums
    in_fov = (u >= -0.5 and u <= wp.float32(cols) - 0.5
              and v >= -0.5 and v <= wp.float32(rows) - 0.5)
    if not in_fov:
        return
    u0 = wp.int32(wp.floor(u))
    v0 = wp.int32(wp.floor(v))
    du = u - wp.float32(u0)
    dv = v - wp.float32(v0)
    w00 = (1.0 - dv) * (1.0 - du)
    w01 = (1.0 - dv) * du
    w10 = dv * (1.0 - du)
    w11 = dv * du
    ok00 = v0 >= 0 and v0 < rows and u0 >= 0 and u0 < cols
    ok01 = v0 >= 0 and v0 < rows and u0 + 1 >= 0 and u0 + 1 < cols
    ok10 = v0 + 1 >= 0 and v0 + 1 < rows and u0 >= 0 and u0 < cols
    ok11 = v0 + 1 >= 0 and v0 + 1 < rows and u0 + 1 >= 0 and u0 + 1 < cols
    wsum = float(0.0)
    if ok00:
        wsum += w00
    if ok01:
        wsum += w01
    if ok10:
        wsum += w10
    if ok11:
        wsum += w11
    if wsum <= 0.0:
        return
    if ok00:
        w = fw * w00 / wsum
        wp.atomic_add(grid, v0, u0, w)
        wp.atomic_add(vgrid, 0, v0, u0, w * v_gel[0])
        wp.atomic_add(vgrid, 1, v0, u0, w * v_gel[1])
    if ok01:
        w = fw * w01 / wsum
        wp.atomic_add(grid, v0, u0 + 1, w)
        wp.atomic_add(vgrid, 0, v0, u0 + 1, w * v_gel[0])
        wp.atomic_add(vgrid, 1, v0, u0 + 1, w * v_gel[1])
    if ok10:
        w = fw * w10 / wsum
        wp.atomic_add(grid, v0 + 1, u0, w)
        wp.atomic_add(vgrid, 0, v0 + 1, u0, w * v_gel[0])
        wp.atomic_add(vgrid, 1, v0 + 1, u0, w * v_gel[1])
    if ok11:
        w = fw * w11 / wsum
        wp.atomic_add(grid, v0 + 1, u0 + 1, w)
        wp.atomic_add(vgrid, 0, v0 + 1, u0 + 1, w * v_gel[0])
        wp.atomic_add(vgrid, 1, v0 + 1, u0 + 1, w * v_gel[1])


@wp.func
def _rel_velocity(
    x_w: wp.vec3,
    ob: wp.int32,
    gel_body: wp.int32,
    body_q: wp.array(dtype=wp.transform),
    body_qd: wp.array(dtype=wp.spatial_vector),
) -> wp.vec3:
    """Object-minus-gel velocity of world point x_w (rigid bodies)."""
    v_rel = wp.vec3(0.0, 0.0, 0.0)
    if ob >= 0:
        qd = body_qd[ob]
        lin = wp.spatial_top(qd)
        ang = wp.spatial_bottom(qd)
        v_rel = lin + wp.cross(ang, x_w - wp.transform_get_translation(body_q[ob]))
    if gel_body >= 0:
        qd = body_qd[gel_body]
        lin = wp.spatial_top(qd)
        ang = wp.spatial_bottom(qd)
        v_rel = v_rel - (lin + wp.cross(ang, x_w - wp.transform_get_translation(body_q[gel_body])))
    return v_rel


@wp.kernel
def splat_faces(
    face_count: wp.array(dtype=wp.int32),
    pairs: wp.array(dtype=wp.vec2i),
    depths: wp.array(dtype=wp.float32),
    points: wp.array(dtype=wp.vec3),            # 3 per face
    gel_shape: wp.int32,
    gel_pos: wp.vec3,
    gel_rot_inv: wp.mat33,                      # world -> gel
    gel_z: wp.vec3,                             # gel +z in world
    width: wp.float32,
    height: wp.float32,
    rows: wp.int32,
    cols: wp.int32,
    shape_kh: wp.array(dtype=wp.float32),
    shape_gel_t: wp.array(dtype=wp.float32),
    stiffen: wp.float32,
    shape_body: wp.array(dtype=wp.int32),
    body_q: wp.array(dtype=wp.transform),
    body_qd: wp.array(dtype=wp.spatial_vector),
    gel_body: wp.int32,
    gel_rot: wp.mat33,                          # gel -> world (columns = gel axes)
    max_level: wp.int32,                        # sub-triangle splat cap (SPLAT_MAX_LEVEL)
    # outputs
    grid: wp.array2d(dtype=wp.float32),
    vgrid: wp.array3d(dtype=wp.float32),        # (2, rows, cols): f-weighted gel-frame tangential velocity
    sums: wp.array(dtype=wp.float32),           # [F, F*x, F*y, F*vx, F*vy, F*vz, n_faces]
):
    tid = wp.tid()
    if tid >= face_count[0]:
        return
    pr = pairs[tid]
    if pr[0] != gel_shape and pr[1] != gel_shape:
        return
    d = depths[tid]
    if d >= 0.0:
        return
    p0 = points[3 * tid]
    p1 = points[3 * tid + 1]
    p2 = points[3 * tid + 2]
    n = wp.cross(p1 - p0, p2 - p0)
    area2 = wp.length(n)
    if area2 <= 0.0:
        return
    area = 0.5 * area2
    nrm = n / area2
    sb = pr[1]
    p = _pressure_law(d, shape_kh[sb], shape_gel_t[sb], stiffen)
    cos_n = wp.abs(wp.dot(nrm, gel_z))
    f = p * area * cos_n
    if f <= 0.0:
        return
    c_w = (p0 + p1 + p2) / 3.0
    c_l = gel_rot_inv * (c_w - gel_pos)
    other = pr[0]
    if other == gel_shape:
        other = pr[1]
    ob = shape_body[other]
    # relative velocity at the face centroid (object - gel): feeds the
    # pad-wide sums; the per-cell shear field below is sampled per point
    v_rel = _rel_velocity(c_w, ob, gel_body, body_q, body_qd)
    gx = wp.vec3(gel_rot[0, 0], gel_rot[1, 0], gel_rot[2, 0])
    gy = wp.vec3(gel_rot[0, 1], gel_rot[1, 1], gel_rot[2, 1])
    # sub-triangle level from the longest edge in raster-cell units
    l0 = gel_rot_inv * (p0 - gel_pos)
    l1 = gel_rot_inv * (p1 - gel_pos)
    l2 = gel_rot_inv * (p2 - gel_pos)
    cw = width / wp.float32(cols)
    ch = height / wp.float32(rows)
    e_px = float(0.0)
    for k in range(3):
        qa = l0
        qb = l1
        if k == 1:
            qa = l1
            qb = l2
        if k == 2:
            qa = l2
            qb = l0
        e = wp.max(wp.abs(qa[0] - qb[0]) / cw, wp.abs(qa[1] - qb[1]) / ch)
        e_px = wp.max(e_px, e)
    s = _splat_level(e_px, max_level)
    if s == 1:
        # PER-CELL SHEAR (2026-08-24): the shear state used to integrate ONE
        # force-weighted mean velocity across the whole pad, so rotation
        # about the normal and any non-uniform slip produced almost no
        # signal (the antisymmetric field averaged to zero) -- the markers
        # barely moved in the tasks.  Splat the gel-frame tangential
        # velocity per cell instead, with the same bilinear weights as the
        # force.
        v_gel = wp.vec3(wp.dot(gx, v_rel), wp.dot(gy, v_rel), 0.0)
        _splat_point(c_l, f, v_gel, width, height, rows, cols, grid, vgrid)
    else:
        # s^2 equal-area sub-triangles: "up" triangles at barycentric
        # ((i+1/3)/s, (j+1/3)/s) and "down" triangles at ((i+2/3)/s,
        # (j+2/3)/s), i + j <= s - 1 resp. s - 2
        fs = f / wp.float32(s * s)
        inv = 1.0 / wp.float32(s)
        e1 = p1 - p0
        e2 = p2 - p0
        for i in range(s):
            for j in range(s - i):
                a = (wp.float32(i) + 1.0 / 3.0) * inv
                b = (wp.float32(j) + 1.0 / 3.0) * inv
                x_w = p0 + a * e1 + b * e2
                vr = _rel_velocity(x_w, ob, gel_body, body_q, body_qd)
                v_gel = wp.vec3(wp.dot(gx, vr), wp.dot(gy, vr), 0.0)
                _splat_point(gel_rot_inv * (x_w - gel_pos), fs, v_gel,
                             width, height, rows, cols, grid, vgrid)
                if j < s - 1 - i:
                    a = (wp.float32(i) + 2.0 / 3.0) * inv
                    b = (wp.float32(j) + 2.0 / 3.0) * inv
                    x_w = p0 + a * e1 + b * e2
                    vr = _rel_velocity(x_w, ob, gel_body, body_q, body_qd)
                    v_gel = wp.vec3(wp.dot(gx, vr), wp.dot(gy, vr), 0.0)
                    _splat_point(gel_rot_inv * (x_w - gel_pos), fs, v_gel,
                                 width, height, rows, cols, grid, vgrid)
    # force-weighted sums: CoP and relative tangential velocity (object - gel)
    wp.atomic_add(sums, 0, f)
    wp.atomic_add(sums, 1, f * c_l[0])
    wp.atomic_add(sums, 2, f * c_l[1])
    wp.atomic_add(sums, 3, f * v_rel[0])
    wp.atomic_add(sums, 4, f * v_rel[1])
    wp.atomic_add(sums, 5, f * v_rel[2])
    wp.atomic_add(sums, 6, 1.0)


class GpuRasterizer:
    """Persistent device buffers for one sensor; call ``run`` per sample."""

    def __init__(self, model, rows: int, cols: int, gel_shape: int,
                 gel_thickness: float, stiffen: float, gel_shapes=()):
        self.device = model.device
        self.rows, self.cols = rows, cols
        self.gel_shape = int(gel_shape)
        self.stiffen = float(stiffen)
        self.grid = wp.zeros((rows, cols), dtype=wp.float32, device=self.device)
        self.vgrid = wp.zeros((2, rows, cols), dtype=wp.float32, device=self.device)
        self.sums = wp.zeros(8, dtype=wp.float32, device=self.device)
        t = np.zeros(model.shape_count, dtype=np.float32)
        for s in (gel_shapes or [gel_shape]):
            t[int(s)] = float(gel_thickness)
        self.shape_gel_t = wp.array(t, dtype=wp.float32, device=self.device)
        self.shape_kh = model.shape_material_kh
        self.shape_body = model.shape_body
        self.gel_body = int(model.shape_body.numpy()[gel_shape])

    def run(self, surface, state, gel_pos_w, gel_rot_w, width, height):
        """Returns (force_grid[rows, cols] float64, sums[7] float64)."""

        self.grid.zero_()
        self.vgrid.zero_()
        self.sums.zero_()
        n_max = surface.contact_surface_depth.shape[0]
        R = np.asarray(gel_rot_w, dtype=np.float32)
        rot_inv = wp.mat33(*R.T.flatten().tolist())
        z = wp.vec3(*R[:, 2].tolist())
        wp.launch(
            splat_faces, dim=n_max,
            inputs=[surface.face_contact_count, surface.contact_surface_shape_pair,
                    surface.contact_surface_depth, surface.contact_surface_point,
                    self.gel_shape, wp.vec3(*np.asarray(gel_pos_w, dtype=np.float32).tolist()),
                    rot_inv, z, float(width), float(height), self.rows, self.cols,
                    self.shape_kh, self.shape_gel_t, self.stiffen,
                    self.shape_body, state.body_q, state.body_qd, self.gel_body,
                    wp.mat33(*R.flatten().tolist()), splat_max_level()],
            outputs=[self.grid, self.vgrid, self.sums], device=self.device)
        return (self.grid.numpy().astype(np.float64),
                self.vgrid.numpy().astype(np.float64),
                self.sums.numpy().astype(np.float64))
