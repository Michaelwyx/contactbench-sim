"""Stiffening gel pressure law for Newton's hydroelastic contact.

WHY (2026-08-22, measured with packages/force-bench/src/force_bench/tactile/press_probe.py): the default
linear law ``p = kh * depth`` makes the pad a Winkler foundation with no
stiffening, so a narrow feature -- the door key's embossed bow, a thread
crest -- pressed at a suite-typical 8-10 N/pad gets only ~0.1 N of
resistance from the whole gel thickness and SINKS THROUGH to the rigid
backing box, which then carries the grasp as a plain, unsensed contact
(a key-bow pinch: finger probes 10 N, gel integral < 1 N).  A
real gel is a thin confined layer whose compression modulus rises steeply
toward full compression; a sharp feature at 10 N sinks 1-3 mm, not 4.

The law here keeps the calibrated small-strain stiffness and adds a cubic
term that grows with the squared strain x/t (x = indentation, t = gel
thickness):

    p(x) = kh * x * (1 + a * (x / t)^2)          x = -signed_depth > 0
    p(d) = kh * d                                d >= 0 (linear extrapolation)

``a`` = ``GelSightConfig.gel_stiffening`` (0 -> the original linear law,
byte-identical).  At a = 10 the gel is 11x stiffer at full compression;
the box/sphere/nut presses of the probe stay within a few % of the linear
law below 0.5 mm and the key bow sinks ~2.5 mm at 10 N instead of
bottoming out.  Rigid objects keep the linear law (their kh is 1e10, the
iso-surface sits at their surface), so the sensor's exported-pressure
identity ``p = kh_rigid * depth_rigid`` is untouched when the gel is
shape A; when the gel is shape B the sensor evaluates this same law.

Newton contract (HydroelasticSDF.Config.pressure_func): finite and
monotone non-increasing in signed_depth over the whole real line, no
clipping of the non-penetrating side -- satisfied (both branches are
monotone and meet at 0 with slope kh).
"""

from __future__ import annotations

import numpy as np
import warp as wp


@wp.struct
class StiffeningPressureData:
    shape_kh: wp.array[wp.float32]
    shape_gel_t: wp.array[wp.float32]   # gel thickness for gel shapes, 0 else
    stiffening: wp.float32


@wp.func
def stiffening_pressure(signed_depth: wp.float32, shape_idx: wp.int32,
                        data: StiffeningPressureData) -> wp.float32:
    kh = data.shape_kh[shape_idx]
    t = data.shape_gel_t[shape_idx]
    if signed_depth >= 0.0 or t <= 0.0:
        return -kh * signed_depth
    x = -signed_depth
    r = x / t
    return kh * x * (1.0 + data.stiffening * r * r)


def make_pressure_data(model, gel_shapes, gel_thickness_m: float,
                       stiffening: float) -> StiffeningPressureData:
    """Build the struct from a FINALIZED model (shape indexing is final)."""

    t = np.zeros(model.shape_count, dtype=np.float32)
    for s in gel_shapes:
        t[int(s)] = float(gel_thickness_m)
    d = StiffeningPressureData()
    d.shape_kh = model.shape_material_kh
    d.shape_gel_t = wp.array(t, dtype=wp.float32, device=model.device)
    d.stiffening = float(stiffening)
    return d


def pressure_np(indent_m: np.ndarray, kh: float, t: float, a: float) -> np.ndarray:
    """NumPy mirror of ``stiffening_pressure`` for indentation x >= 0."""

    x = np.maximum(np.asarray(indent_m, dtype=np.float64), 0.0)
    if a <= 0.0 or t <= 0.0:
        return kh * x
    return kh * x * (1.0 + a * (x / t) ** 2)


def indent_from_pressure_np(p: np.ndarray, kh: float, t: float, a: float,
                            iters: int = 12) -> np.ndarray:
    """Inverse of ``pressure_np`` (monotone cubic; fixed-point + Newton)."""

    p = np.maximum(np.asarray(p, dtype=np.float64), 0.0)
    x = p / kh
    if a <= 0.0 or t <= 0.0:
        return x
    for _ in range(iters):
        r2 = (x / t) ** 2
        f = kh * x * (1.0 + a * r2) - p
        df = kh * (1.0 + 3.0 * a * r2)
        x = np.maximum(x - f / df, 0.0)
    return x
