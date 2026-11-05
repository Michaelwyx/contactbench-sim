"""Warp kernels for gelsight_render's geometry-derived height map."""

from __future__ import annotations

import warp as wp


@wp.kernel
def raycast_height(
    mesh: wp.uint64,
    X_og: wp.transform,     # gel-frame -> object-frame transform
    width_m: float,         # gel extent along image h (gel x)
    height_m: float,        # gel extent along image w (gel y)
    z_top: float,           # sensing face height in the gel shape frame
    far: float,             # ray start distance below the face
    max_pen: float,
    out: wp.array2d(dtype=wp.float32),
):
    """One ray per image pixel, cast from inside the gel toward the object.

    The ray starts ``far`` below the sensing face and travels along gel +z;
    the first surface it hits is the object face pressing into the gel, and
    the penetration is how far below the sensing face that hit lies.  The
    object pose is the SOLVER's hydroelastic equilibrium, so the recovered
    depth is the physically realized indentation.
    """

    i, j = wp.tid()
    h = out.shape[0]
    w = out.shape[1]
    x = (float(i) + 0.5) / float(h) * width_m - 0.5 * width_m
    y = (float(j) + 0.5) / float(w) * height_m - 0.5 * height_m
    origin = wp.transform_point(X_og, wp.vec3(x, y, z_top - far))
    direction = wp.transform_vector(X_og, wp.vec3(0.0, 0.0, 1.0))
    q = wp.mesh_query_ray(mesh, origin, direction, far + 0.02)
    if q.result:
        pen = far - q.t
        if pen > 0.0:
            out[i, j] = wp.max(out[i, j], wp.min(pen, max_pen))


# ── GPU shading path (2026-08-23): height map stays on the device from the
# raycast through the Taxim shading; only the RGB tile (and the height, for
# the marker model) come back.  Measured per pad: CPU path 5.8 (height) +
# 6.7 (Taxim) ms -> GPU ~1 ms.
@wp.kernel
def blur_h(src: wp.array2d(dtype=wp.float32), radius: int, sigma: float,
           dst: wp.array2d(dtype=wp.float32)):
    i, j = wp.tid()
    h = src.shape[0]
    w = src.shape[1]
    acc = float(0.0)
    wsum = float(0.0)
    for k in range(-radius, radius + 1):
        jj = j + k
        if jj < 0:
            jj = -jj - 1             # reflect (scipy 'reflect' default)
        if jj >= w:
            jj = 2 * w - jj - 1
        g = wp.exp(-0.5 * float(k * k) / (sigma * sigma))
        acc += g * src[i, jj]
        wsum += g
    dst[i, j] = acc / wsum


@wp.kernel
def blur_v(src: wp.array2d(dtype=wp.float32), radius: int, sigma: float,
           dst: wp.array2d(dtype=wp.float32)):
    i, j = wp.tid()
    h = src.shape[0]
    acc = float(0.0)
    wsum = float(0.0)
    for k in range(-radius, radius + 1):
        ii = i + k
        if ii < 0:
            ii = -ii - 1
        if ii >= h:
            ii = 2 * h - ii - 1
        g = wp.exp(-0.5 * float(k * k) / (sigma * sigma))
        acc += g * src[ii, j]
        wsum += g
    dst[i, j] = acc / wsum


@wp.kernel
def upsample_raster_max(raster: wp.array2d(dtype=wp.float32),   # (rows=gel y, cols=gel x)
                        out: wp.array2d(dtype=wp.float32)):     # image (h along gel x, w along gel y)
    """Bilinear resample of a sensor raster into image layout, max'ed into out."""

    i, j = wp.tid()
    h = out.shape[0]
    w = out.shape[1]
    rows = raster.shape[0]
    cols = raster.shape[1]
    # image h axis <-> raster cols (x), image w axis <-> raster rows (y)
    u = (float(i) + 0.5) / float(h) * float(cols) - 0.5
    v = (float(j) + 0.5) / float(w) * float(rows) - 0.5
    u0 = wp.int32(wp.floor(u))
    v0 = wp.int32(wp.floor(v))
    du = u - float(u0)
    dv = v - float(v0)
    u1 = wp.min(wp.max(u0 + 1, 0), cols - 1)
    v1 = wp.min(wp.max(v0 + 1, 0), rows - 1)
    u0 = wp.min(wp.max(u0, 0), cols - 1)
    v0 = wp.min(wp.max(v0, 0), rows - 1)
    val = ((1.0 - dv) * ((1.0 - du) * raster[v0, u0] + du * raster[v0, u1])
           + dv * ((1.0 - du) * raster[v1, u0] + du * raster[v1, u1]))
    out[i, j] = wp.max(out[i, j], wp.max(val, 0.0))


@wp.kernel
def max_into(src: wp.array2d(dtype=wp.float32), dst: wp.array2d(dtype=wp.float32)):
    i, j = wp.tid()
    dst[i, j] = wp.max(dst[i, j], src[i, j])


@wp.kernel
def taxim_shade(height_px: wp.array2d(dtype=wp.float32),      # blurred, in pixels
                grad_r: wp.array3d(dtype=wp.float32),         # (nbins, nbins, 6)
                grad_g: wp.array3d(dtype=wp.float32),
                grad_b: wp.array3d(dtype=wp.float32),
                x_binr: float, y_binr: float,
                gain: float,                                  # contrast on the calibrated response
                flat_delta: wp.array3d(dtype=wp.float32),     # (h, w, 3)
                background: wp.array3d(dtype=wp.float32),     # (h, w, 3)
                rgb: wp.array3d(dtype=wp.uint8)):
    """Taxim polynomial shading: gradient -> (magnitude, direction) bin ->
    per-pixel polynomial in (x, y) -- identical arithmetic to
    TaximRenderer.render, including np.gradient's central differences and the
    symmetric border fill."""

    i, j = wp.tid()
    h = height_px.shape[0]
    w = height_px.shape[1]
    ii = wp.min(wp.max(i, 1), h - 2)
    jj = wp.min(wp.max(j, 1), w - 2)
    dzdx = 0.5 * (height_px[ii + 1, jj] - height_px[ii - 1, jj])
    dzdy = 0.5 * (height_px[ii, jj + 1] - height_px[ii, jj - 1])
    mag0 = wp.sqrt(dzdx * dzdx + dzdy * dzdy)
    mag = wp.atan(mag0)
    direction = float(0.0)
    if mag0 > 0.0:
        direction = wp.atan2(dzdx, dzdy)
    nb = grad_r.shape[0]
    ix = wp.min(wp.max(wp.int32(wp.floor(mag / x_binr)), 0), nb - 1)
    iy = wp.min(wp.max(wp.int32(wp.floor((direction + 3.14159265358979) / y_binr)), 0), grad_r.shape[1] - 1)
    xf = float(j)          # A = [x*x, y*y, x*y, x, y, 1] with x = column, y = row
    yf = float(i)
    a0 = xf * xf
    a1 = yf * yf
    a2 = xf * yf
    r = (a0 * grad_r[ix, iy, 0] + a1 * grad_r[ix, iy, 1] + a2 * grad_r[ix, iy, 2]
         + xf * grad_r[ix, iy, 3] + yf * grad_r[ix, iy, 4] + grad_r[ix, iy, 5])
    g = (a0 * grad_g[ix, iy, 0] + a1 * grad_g[ix, iy, 1] + a2 * grad_g[ix, iy, 2]
         + xf * grad_g[ix, iy, 3] + yf * grad_g[ix, iy, 4] + grad_g[ix, iy, 5])
    b = (a0 * grad_b[ix, iy, 0] + a1 * grad_b[ix, iy, 1] + a2 * grad_b[ix, iy, 2]
         + xf * grad_b[ix, iy, 3] + yf * grad_b[ix, iy, 4] + grad_b[ix, iy, 5])
    r = gain * (r - flat_delta[i, j, 0]) + background[i, j, 0]
    g = gain * (g - flat_delta[i, j, 1]) + background[i, j, 1]
    b = gain * (b - flat_delta[i, j, 2]) + background[i, j, 2]
    rgb[i, j, 0] = wp.uint8(wp.min(wp.max(r, 0.0), 255.0))
    rgb[i, j, 1] = wp.uint8(wp.min(wp.max(g, 0.0), 255.0))
    rgb[i, j, 2] = wp.uint8(wp.min(wp.max(b, 0.0), 255.0))


@wp.kernel
def scale_into(src: wp.array2d(dtype=wp.float32), k: float, dst: wp.array2d(dtype=wp.float32)):
    i, j = wp.tid()
    dst[i, j] = src[i, j] * k


@wp.kernel
def hydroshear_markers(
    height_m: wp.array2d(dtype=wp.float32),       # (H, W) image-res height [m]
    shear: wp.array3d(dtype=wp.float32),          # (2, rows, cols) sensor shear raster [m]
    taxel_px: wp.array(dtype=wp.vec2i),           # (M,) taxel image pixels (h, w)
    p_j: wp.array(dtype=wp.vec2),                 # (M,) taxel gel coords [m]
    p_i: wp.array(dtype=wp.vec2),                 # (N,) marker gel coords [m]
    lambda_d: float, lambda_s: float,
    k_dil: float, k_shr: float,                   # gain * density_scale * px_per_m
    img_h: int, img_w: int,
    out: wp.array(dtype=wp.vec2),                 # (N,) displacement [px] (h, w)
):
    """HydroShear dilation + kernel-spread shear, same sums as
    HydroShearMarkers.displace (vectorized NumPy), one thread per marker."""

    n = wp.tid()
    pi = p_i[n]
    dil = wp.vec2(0.0, 0.0)
    shr = wp.vec2(0.0, 0.0)
    rows = shear.shape[1]
    cols = shear.shape[2]
    for m in range(p_j.shape[0]):
        tp = taxel_px[m]
        h = height_m[tp[0], tp[1]]
        if h <= 0.0:
            continue
        pj = p_j[m]
        d = pi - pj
        dil += h * wp.exp(-lambda_d * wp.dot(d, d)) * d
        # bilinear sample of the shear raster at the taxel (image w -> rows, h -> cols)
        r = float(tp[1]) / float(img_w - 1) * float(rows - 1)
        c = float(tp[0]) / float(img_h - 1) * float(cols - 1)
        r0 = wp.int32(wp.floor(r))
        c0 = wp.int32(wp.floor(c))
        r1 = wp.min(r0 + 1, rows - 1)
        c1 = wp.min(c0 + 1, cols - 1)
        fr = r - float(r0)
        fc = c - float(c0)
        fx = ((1.0 - fr) * ((1.0 - fc) * shear[0, r0, c0] + fc * shear[0, r0, c1])
              + fr * ((1.0 - fc) * shear[0, r1, c0] + fc * shear[0, r1, c1]))
        fy = ((1.0 - fr) * ((1.0 - fc) * shear[1, r0, c0] + fc * shear[1, r0, c1])
              + fr * ((1.0 - fc) * shear[1, r1, c0] + fc * shear[1, r1, c1]))
        fj = wp.vec2(fx, fy)
        da = pi - (pj + fj)
        shr += h * wp.exp(-lambda_s * wp.dot(da, da)) * fj
    out[n] = k_dil * dil + k_shr * shr


@wp.kernel
def raycast_height_persp(
    mesh: wp.uint64,
    X_og: wp.transform,     # gel-frame -> object-frame transform
    width_m: float,
    height_m: float,
    z_top: float,           # sensing face height in the gel shape frame
    cam_standoff: float,    # camera pinhole this far BEHIND the sensing face
    far: float,
    max_pen: float,
    out: wp.array2d(dtype=wp.float32),
):
    """Pinhole variant of ``raycast_height`` (2026-08-24): the real GS Mini
    camera sits ~24 mm behind the gel surface, so pixels map to rays from a
    pinhole through the pixel's point on the UNDEFORMED face plane.  For a
    flat gel this equals the orthographic map; pressed regions are seen
    obliquely toward the FOV edges (foreshortening), like the hardware."""

    i, j = wp.tid()
    h = out.shape[0]
    w = out.shape[1]
    x = (float(i) + 0.5) / float(h) * width_m - 0.5 * width_m
    y = (float(j) + 0.5) / float(w) * height_m - 0.5 * height_m
    cam = wp.vec3(0.0, 0.0, z_top - cam_standoff)
    pix = wp.vec3(x, y, z_top)
    dir_g = wp.normalize(pix - cam)
    # start the ray far enough back along the ray to catch deep presses
    origin_g = pix - dir_g * (far / dir_g[2])
    origin = wp.transform_point(X_og, origin_g)
    direction = wp.transform_vector(X_og, dir_g)
    q = wp.mesh_query_ray(mesh, origin, direction, 2.0 * (far + 0.02))
    if q.result:
        hit_g = origin_g + dir_g * q.t
        pen = z_top - hit_g[2]
        if pen > 0.0:
            out[i, j] = wp.max(out[i, j], wp.min(pen, max_pen))


@wp.kernel
def bulge_ring(h: wp.array2d(dtype=wp.float32),        # indentation [m], >= 0
               blurred: wp.array2d(dtype=wp.float32),  # wide gaussian of h
               k_bulge: float,
               out: wp.array2d(dtype=wp.float32)):
    """Volume-conserving membrane bulge (2026-08-24): a near-incompressible
    gel pushed down inside the contact must rise AROUND it.  The Winkler
    law has no lateral coupling, so the height map showed a hole with no
    rim -- real GelSight images always show the bright raised ring.  The
    ring is modelled as the positive part of (blur - h): zero inside the
    contact (blur < h), a smooth annulus outside; k_bulge scales it so the
    displaced volume reappears in the rim (sweepable, GS_BULGE)."""

    i, j = wp.tid()
    ring = blurred[i, j] - h[i, j]
    if ring < 0.0:
        ring = 0.0
    out[i, j] = h[i, j] - k_bulge * ring
