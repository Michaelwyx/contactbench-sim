"""GPU-parallel throughput benchmark for the GelSight sensor + render stages.

The physics substep chain is sequential by nature (each step depends on the
last), but the SENSOR SPLAT and the RENDER pipeline are pure data-parallel
kernel sequences.  This bench measures both stages two ways:

  * LATENCY mode  -- wp.synchronize() after every frame: what one frame
    costs when the host waits for it (the number quoted in the docs).
  * THROUGHPUT mode -- N independent buffer sets, M frames launched
    back-to-back, ONE final sync: what the GPU can pipeline when frames
    are independent (the multi-env / batched-collection regime).

Synthetic inputs (a sphere-press height map; a random triangle soup with
plausible depths) so the bench needs no scene and runs in seconds.

Run: python -m hydrogel.bench_parallel            (quiet GPU recommended)
"""

from __future__ import annotations

import time

import numpy as np
import warp as wp

from hydrogel.render import TaximRenderer, GS_H, GS_W, PIXMM
import hydrogel.render_kernels as krn
from hydrogel.sensor_kernels import splat_faces

REPORT = []


def _time(label, mode, n_buf, frames, fn):
    wp.synchronize()
    t0 = time.perf_counter()
    fn()
    wp.synchronize()
    dt = time.perf_counter() - t0
    ms = dt / frames * 1e3
    REPORT.append((label, mode, n_buf, round(ms, 3), round(1.0 / (dt / frames), 1)))
    print(f"{label:8s} {mode:10s} N={n_buf:3d}  {ms:7.3f} ms/frame  {1.0/(dt/frames):8.1f} fps")


# ── render stage ────────────────────────────────────────────────────────
def bench_render(frames=600):
    r = TaximRenderer()
    d = r._device_tables()
    # unique height maps (sphere presses at different centres/depths)
    hh, ww = np.meshgrid(np.arange(GS_H), np.arange(GS_W), indexing="ij")

    def sphere(cy, cx, depth_mm):
        r_px = 8.0 / PIXMM * 0.5
        d2 = (hh - cy) ** 2 + (ww - cx) ** 2
        return (np.where(d2 < r_px ** 2,
                         np.sqrt(np.maximum(r_px ** 2 - d2, 0.0)) * PIXMM / 1e3,
                         0.0) * depth_mm / 4.0).astype(np.float32)

    def make_bufs(n):
        hs = [wp.array(sphere(GS_H / 2 + 10 * (i % 5), GS_W / 2 + 8 * (i % 7),
                              1.0 + 0.1 * i), dtype=wp.float32) for i in range(n)]
        outs = [dict(px=wp.zeros((GS_H, GS_W), dtype=wp.float32),
                     tmp=wp.zeros((GS_H, GS_W), dtype=wp.float32),
                     rgb=wp.zeros((GS_H, GS_W, 3), dtype=wp.uint8)) for _ in range(n)]
        return hs, outs

    sig = 1.1
    rad = int(2.0 * sig + 0.5)
    gain = float(getattr(r, "gain", 1.0))

    def one(h, o):
        wp.launch(krn.scale_into, dim=(GS_H, GS_W), inputs=[h, float(1000.0 / PIXMM)], outputs=[o["px"]])
        wp.launch(krn.blur_h, dim=(GS_H, GS_W), inputs=[o["px"], rad, sig], outputs=[o["tmp"]])
        wp.launch(krn.blur_v, dim=(GS_H, GS_W), inputs=[o["tmp"], rad, sig], outputs=[o["px"]])
        wp.launch(krn.taxim_shade, dim=(GS_H, GS_W),
                  inputs=[o["px"], d["grad_r"], d["grad_g"], d["grad_b"],
                          float(r.x_binr), float(r.y_binr), gain, d["flat"], d["bg"]],
                  outputs=[o["rgb"]])

    hs, outs = make_bufs(1)
    for _ in range(20):
        one(hs[0], outs[0])          # warm-up / compile
    wp.synchronize()

    def latency():
        for i in range(frames):
            one(hs[0], outs[0])
            wp.synchronize()
    _time("render", "latency", 1, frames, latency)

    for n in (8, 32, 64):
        hs, outs = make_bufs(n)
        def burst():
            for i in range(frames):
                one(hs[i % n], outs[i % n])
        _time("render", "throughput", n, frames, burst)


# ── sensor splat stage ──────────────────────────────────────────────────
def bench_splat(frames=600, n_faces=20000):
    rng = np.random.default_rng(0)
    W, H = 0.0186, 0.0143
    pts = np.zeros((3 * n_faces, 3), dtype=np.float32)
    c = np.stack([rng.uniform(-W / 2, W / 2, n_faces),
                  rng.uniform(-H / 2, H / 2, n_faces),
                  np.zeros(n_faces)], axis=1)
    e = 2.0e-4
    pts[0::3] = c + [-e, -e, 0]
    pts[1::3] = c + [e, -e, 0]
    pts[2::3] = c + [0, e, 0]
    depths = -rng.uniform(1e-4, 1.5e-3, n_faces).astype(np.float32)
    pairs = np.zeros((n_faces, 2), dtype=np.int32)
    pairs[:, 0] = 0
    pairs[:, 1] = 1                      # gel is shape B = 1

    count = wp.array(np.array([n_faces], dtype=np.int32), dtype=wp.int32)
    pairs_d = wp.array(pairs, dtype=wp.vec2i)
    depths_d = wp.array(depths, dtype=wp.float32)
    pts_d = wp.array(pts, dtype=wp.vec3)
    shape_kh = wp.array(np.array([1e10, 4e7], dtype=np.float32), dtype=wp.float32)
    shape_gel_t = wp.array(np.array([0.0, 0.004], dtype=np.float32), dtype=wp.float32)
    shape_body = wp.array(np.array([0, 1], dtype=np.int32), dtype=wp.int32)
    body_q = wp.array(np.tile(np.array([0, 0, 0, 0, 0, 0, 1], dtype=np.float32), (2, 1)),
                      dtype=wp.transform)
    body_qd = wp.zeros(2, dtype=wp.spatial_vector)
    rot = wp.mat33(*np.eye(3, dtype=np.float32).flatten().tolist())
    rows, cols = 96, 128

    def make_bufs(n):
        return [dict(grid=wp.zeros((rows, cols), dtype=wp.float32),
                     vgrid=wp.zeros((2, rows, cols), dtype=wp.float32),
                     sums=wp.zeros(8, dtype=wp.float32)) for _ in range(n)]

    def one(o):
        o["grid"].zero_(); o["vgrid"].zero_(); o["sums"].zero_()
        wp.launch(splat_faces, dim=n_faces,
                  inputs=[count, pairs_d, depths_d, pts_d, 1,
                          wp.vec3(0.0, 0.0, 0.0), rot, wp.vec3(0.0, 0.0, 1.0),
                          W, H, rows, cols, shape_kh, shape_gel_t, 10.0,
                          shape_body, body_q, body_qd, 1, rot],
                  outputs=[o["grid"], o["vgrid"], o["sums"]])

    bufs = make_bufs(1)
    for _ in range(20):
        one(bufs[0])
    wp.synchronize()

    def latency():
        for i in range(frames):
            one(bufs[0])
            wp.synchronize()
    _time("splat", "latency", 1, frames, latency)

    for n in (8, 32, 64):
        bufs = make_bufs(n)
        def burst():
            for i in range(frames):
                one(bufs[i % n])
        _time("splat", "throughput", n, frames, burst)


if __name__ == "__main__":
    wp.init()
    print(f"device: {wp.get_device()}  |  {GS_H}x{GS_W} render, 96x128 splat")
    bench_splat()
    bench_render()
    print("\nlabel    mode       N    ms/frame   fps")
    for r_ in REPORT:
        print(f"{r_[0]:8s} {r_[1]:10s} {r_[2]:3d}  {r_[3]:9.3f}  {r_[4]:8.1f}")
