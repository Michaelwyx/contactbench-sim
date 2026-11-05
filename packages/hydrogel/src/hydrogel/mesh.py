"""Mesh -> newton.Mesh (+SDF) helpers.

A copy of ``force_bench.assets.make_newton_mesh`` / ``set_body_mass`` so the
sensor core has no dependency on the task suite; the two must stay identical
(the SDF cache directory is shared on purpose).
"""

from __future__ import annotations

import numpy as np
import trimesh

import newton

from hydrogel.paths import SDF_CACHE


def make_newton_mesh(
    tm: trimesh.Trimesh,
    *,
    sdf: bool = False,
    sdf_resolution: int = 256,
    narrow_band: float = 0.002,
    margin: float = 0.0015,
) -> tuple[newton.Mesh, np.ndarray]:
    """Build a newton.Mesh centered at its AABB center; returns (mesh, center).

    Center the vertices so the body origin is the AABB center — callers place
    the body at `center` in the world to preserve assembly coordinates.
    """
    vertices = np.array(tm.vertices, dtype=np.float32)
    center = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
    vertices = vertices - center
    indices = np.array(tm.faces.flatten(), dtype=np.int32)
    mesh = newton.Mesh(vertices, indices)
    if sdf:
        mesh.build_sdf(
            max_resolution=sdf_resolution,
            narrow_band_range=(-narrow_band, narrow_band),
            margin=margin,
            cache_dir=SDF_CACHE,
        )
    return mesh, center


def set_body_mass(builder: newton.ModelBuilder, body: int, mass: float):
    """Override a body's mass CONSISTENTLY (inv mass + scaled inertia).

    builder.body_mass alone leaves body_inv_mass/inertia stale, which
    destabilizes the solver — always use this helper instead.
    """
    old = builder.body_mass[body]
    if old <= 0.0:
        raise ValueError(f"body {body} has zero mass; add shapes first")
    f = mass / old
    builder.body_mass[body] = mass
    builder.body_inv_mass[body] = 1.0 / mass
    builder.body_inertia[body] = builder.body_inertia[body] * f
    builder.body_inv_inertia[body] = builder.body_inv_inertia[body] * (1.0 / f)
