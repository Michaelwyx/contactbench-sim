"""Indenter meshes for the validation bench (hydrogel.validate).

``key_bow`` is the bow of the benchmark's key_1 asset (vendored as
``data/indenters/key_bow.stl``, cm units, same file as
``assets/bodies/key_1/part3_1_body1.stl``) so the sensor bench runs without
the task suite.  The pose code is shared with
``force_bench.tactile.press_probe.object_mesh`` and must stay identical.
"""

from __future__ import annotations

import numpy as np
import trimesh

from hydrogel.paths import INDENTER_DIR

CM = 0.01


def load_indenter(name: str) -> trimesh.Trimesh:
    p = INDENTER_DIR / f"{name}.stl"
    if not p.exists():
        raise FileNotFoundError(p)
    tm = trimesh.load(p, force="mesh")
    tm.apply_scale(CM)
    return tm


def object_mesh(name: str):
    """(trimesh in metres, posed with the contact face toward -z; SDF kwargs)."""

    if name in ("key_bow", "key_bow_b", "key_bow_nb2"):
        # RAW STL frame: length along z (49.8 mm), bow width along x (20 mm),
        # thickness along y (4.65 mm incl. the blade's bitting).  Find the
        # bow as the wide end, lay the key along the pad's y with the bow
        # centred, and turn one bow face (+y for key_bow, -y for key_bow_b)
        # to face the gel (-z).
        tm = load_indenter("key_bow")
        v = tm.vertices
        zs = np.linspace(v[:, 2].min(), v[:, 2].max(), 50)
        width = [np.ptp(v[(v[:, 2] >= a) & (v[:, 2] < b), 0]) if
                 ((v[:, 2] >= a) & (v[:, 2] < b)).sum() > 2 else 0.0
                 for a, b in zip(zs[:-1], zs[1:])]
        bow_at_max = bool(np.mean(width[-10:]) > np.mean(width[:10]))
        z_bow = (v[:, 2].max() - 0.010) if bow_at_max else (v[:, 2].min() + 0.010)
        bow = v[np.abs(v[:, 2] - z_bow) < 0.010]
        tm.apply_translation((-bow[:, 0].mean(), -bow[:, 1].mean(), -z_bow))
        # old y -> new -z (face +y down) via R_x(-90): (x, y, z) -> (x, z, -y)
        sgn = -1.0 if name == "key_bow_b" else 1.0
        tm.apply_transform(trimesh.transformations.rotation_matrix(
            sgn * -np.pi / 2, (1.0, 0.0, 0.0)))
        kw = dict(sdf_resolution=512, narrow_band=0.001, margin=0.0004)
        if name == "key_bow_nb2":
            kw = dict(sdf_resolution=512, narrow_band=0.002, margin=0.0005)
        return tm, kw
    raise ValueError(name)
