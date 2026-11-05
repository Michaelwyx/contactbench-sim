"""Asset library for force-bench.

Loads the named per-body STLs exported from Fusion (assets/bodies/, units
cm) and builds Newton collision geometry (SDF for tight-fit parts).  Body
files matching *spring* are excluded from import and reported: printed PLA
springs are replaced by joint drives (see assets/manifest.yaml).
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.paths import BODIES_DIR, SDF_CACHE


CM = 0.01  # exported bodies are in centimeters


SPRING_RE = re.compile(r"spring", re.IGNORECASE)


def body_path(asset: str, stem: str) -> Path:
    p = BODIES_DIR / asset / f"{stem}.stl"
    if not p.exists():
        raise FileNotFoundError(p)
    return p


def list_bodies(asset: str) -> list[str]:
    return sorted(p.stem for p in (BODIES_DIR / asset).glob("*.stl"))


def springs_of(asset: str) -> list[str]:
    return [s for s in list_bodies(asset) if SPRING_RE.search(s)]


def load_tm(asset: str, stem: str) -> trimesh.Trimesh:
    """Load a body as trimesh in METERS, keeping assembly coordinates."""
    m = trimesh.load(body_path(asset, stem), force="mesh")
    m.apply_scale(CM)
    return m


#: Attribute of a newton.Mesh built by make_newton_mesh(sdf=True): the build_sdf arguments it was (or, deferred,
#: would be) baked with.  force_bench.kit.usd reads it to write the same SDF settings into a USD stage.
SDF_PARAMS_ATTR = "force_bench_sdf_params"
_DEFER_SDF = [False]


class defer_sdf_builds:
    """Context manager: make_newton_mesh records its SDF settings on the mesh but does not bake the SDF.

    For code that only inspects a scene as built (the USD exporter): no CUDA needed, and no SDF bake time.  A mesh
    built inside it has no SDF, so a model finalized from it would collide through the triangle mesh instead."""

    def __enter__(self):
        self._prev = _DEFER_SDF[0]
        _DEFER_SDF[0] = True
        return self

    def __exit__(self, *exc):
        _DEFER_SDF[0] = self._prev
        return False


def sdf_builds_deferred() -> bool:
    """True inside ``defer_sdf_builds``.  A cache of built meshes keys on it, so that meshes built there (no SDF)
    never reach a model that is simulated."""
    return _DEFER_SDF[0]


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
        params = dict(max_resolution=int(sdf_resolution), narrow_band_range=(-float(narrow_band), float(narrow_band)),
                      margin=float(margin))
        setattr(mesh, SDF_PARAMS_ATTR, params)
        if not _DEFER_SDF[0]:
            mesh.build_sdf(**params, cache_dir=SDF_CACHE)
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
