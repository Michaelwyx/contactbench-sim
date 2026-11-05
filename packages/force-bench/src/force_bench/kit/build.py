"""Put kit assets into a scene.

    from force_bench.kit import add_asset, place_on_table
    rig = Rig("my_scene"); rig.table()
    h = add_asset(rig, "outlet", "0.7mm", pose=place_on_table("outlet", xy=(0.45, 0.0)))
    h.bodies, h.joints, h.shapes            # what was created, by role
    ...
    rig.finalize(...)

``add_asset`` builds what ``sim.yaml`` says the asset is: a ``free`` object (one dynamic body), a ``fixture`` (static
shapes), or a ``mechanism`` (bodies and joints from ``force_bench.kit.mechanisms``).  Every shape keeps its print
colour and is labelled ``kit/<label>/<part>``; every body is labelled ``<label>`` or ``<label>/<role>``.

Poses are 4x4 world transforms of the asset's CAD frame (the frame its STL files share).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import trimesh
import warp as wp

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.kit.catalog import Asset, Part, load_kit, print_mass, sdf_resolution


@dataclass
class AssetHandle:
    """What add_asset created."""
    key: str
    label: str
    choice: dict
    pose: np.ndarray
    bodies: dict[str, int] = field(default_factory=dict)      # role -> body id ("" for a single free body)
    joints: dict[str, int] = field(default_factory=dict)      # role -> joint id
    shapes: dict[str, list[int]] = field(default_factory=dict)  # part name -> shape ids
    params: dict = field(default_factory=dict)                 # the variant's physical parameters, as built
    info: dict = field(default_factory=dict)                   # measured geometry the mechanism was built from


# ── poses ─────────────────────────────────────────────────────────────────────────────────────────────────────────
def pose_matrix(xyz=(0.0, 0.0, 0.0), yaw: float = 0.0, R: np.ndarray | None = None) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R if R is not None else trimesh.transformations.rotation_matrix(yaw, (0, 0, 1))[:3, :3]
    T[:3, 3] = xyz
    return T


def place_on_table(key: str, choice=None, xy=(0.0, 0.0), yaw: float = 0.0, z0: float = 0.0,
                   R: np.ndarray | None = None, clearance: float = 0.0) -> np.ndarray:
    """Pose that puts the asset's lowest point ``clearance`` above the plane z = ``z0`` at ``xy`` (centred)."""
    parts = load_kit()[key].parts(choice)
    tm = trimesh.util.concatenate([p.mesh() for p in parts])
    T = pose_matrix(yaw=yaw, R=R)
    tm.apply_transform(T)
    lo, hi = tm.bounds
    c = 0.5 * (lo + hi)
    T[:3, 3] += np.array([xy[0] - c[0], xy[1] - c[1], z0 + clearance - lo[2]])
    return T


def to_wp(T: np.ndarray) -> wp.transform:
    q = trimesh.transformations.quaternion_from_matrix(T)        # w, x, y, z
    return wp.transform(wp.vec3(*T[:3, 3]), wp.quat(float(q[1]), float(q[2]), float(q[3]), float(q[0])))


# ── settings from sim.yaml ────────────────────────────────────────────────────────────────────────────────────────
def _setting(asset: Asset, part: str | None, key: str, default=None):
    """sim.yaml lookup: asset.parts.<part>.<key>, then asset.<key>, then defaults.<key>."""
    kit = load_kit()
    if part:
        for pattern, over in (asset.sim.get("parts") or {}).items():
            if pattern == part or (pattern.endswith("*") and part.startswith(pattern[:-1])):
                if key in over:
                    return over[key]
    if key in asset.sim:
        return asset.sim[key]
    return kit.sim_defaults.get(key, default)


def part_mass(asset: Asset, part: Part) -> float:
    grams = _setting(asset, part.name, "mass_g")
    if grams is not None:
        return float(grams) * 1e-3
    pr = load_kit().sim_defaults.get("print", {})
    return print_mass(part.mesh(), density=float(pr.get("density", 1240.0)),
                      infill=float(pr.get("infill", 0.20)), wall=float(pr.get("wall_mm", 0.8)) * 1e-3)


# ── shapes ────────────────────────────────────────────────────────────────────────────────────────────────────────
def add_part_shape(rig, body: int, asset: Asset, part: Part, body_from_cad: np.ndarray, label: str,
                   mesh: trimesh.Trimesh | None = None, collide: bool = True, mu: float | None = None,
                   sdf_res: int | None = None) -> int:
    """One part as a mesh shape on ``body`` (-1: static), placed by ``body_from_cad`` (body frame <- CAD frame)."""
    tm = mesh if mesh is not None else part.mesh()
    use_sdf = collide and _setting(asset, part.name, "collision", "sdf") == "sdf"
    res = sdf_res or _setting(asset, part.name, "sdf_res") or sdf_resolution(
        tm, cell=float(_setting(asset, part.name, "sdf_cell_mm", 0.4)) * 1e-3,
        lo=int(_setting(asset, part.name, "sdf_min_res", 64)), hi=int(_setting(asset, part.name, "sdf_max_res", 384)))
    nm, center = make_newton_mesh(tm, sdf=use_sdf, sdf_resolution=int(res),
                                  narrow_band=float(_setting(asset, part.name, "narrow_band_mm", 2.0)) * 1e-3,
                                  margin=float(_setting(asset, part.name, "margin_mm", 1.2)) * 1e-3)
    T = body_from_cad @ pose_matrix(xyz=center)
    friction = mu if mu is not None else float(_setting(asset, part.name, "mu", 0.5))
    cfg = rig.cfg(mu=friction)
    gap = _setting(asset, part.name, "gap_mm")
    if gap is not None:
        cfg.gap = float(gap) * 1e-3
    if not collide:
        cfg.has_shape_collision = False
        cfg.has_particle_collision = False
    sid = rig.builder.add_shape_mesh(body, xform=to_wp(T), mesh=nm, cfg=cfg, color=part.rgb,
                                     label=f"kit/{label}/{part.name}")
    rig.shapes.setdefault(label, []).append(sid)
    return sid


def add_extra_boxes(rig, asset: Asset, body: int, body_from_cad: np.ndarray, label: str) -> dict[str, list[int]]:
    """Non-printed parts declared in sim.yaml (``extra_boxes``: name, size_mm, center_mm in the CAD frame,
    color 0-255, mu, mass_g), e.g. the platform's acrylic sheet."""
    out = {}
    for ex in asset.sim.get("extra_boxes") or []:
        half = np.asarray(ex["size_mm"], float) * 0.5e-3
        T = body_from_cad @ pose_matrix(xyz=np.asarray(ex["center_mm"], float) * 1e-3)
        cfg = rig.cfg(mu=float(ex.get("mu", 0.5)))
        rgb = tuple(float(c) / 255.0 for c in ex.get("color", (200, 200, 200)))
        sid = rig.builder.add_shape_box(body, xform=to_wp(T), hx=float(half[0]), hy=float(half[1]), hz=float(half[2]),
                                        cfg=cfg, color=rgb, label=f"kit/{label}/{ex['name']}")
        rig.shapes.setdefault(label, []).append(sid)
        out[ex["name"]] = [sid]
    return out


def add_rigid(rig, asset: Asset, parts: list[Part], pose: np.ndarray, label: str, fixed: bool,
              mass: float | None = None) -> tuple[int, dict[str, list[int]]]:
    """All ``parts`` welded into one body (``fixed``: static shapes, body -1).  Returns (body, shapes)."""
    tms = [p.mesh() for p in parts]
    lo = np.min([m.bounds[0] for m in tms], axis=0)
    hi = np.max([m.bounds[1] for m in tms], axis=0)
    c_cad = 0.5 * (lo + hi)
    shapes: dict[str, list[int]] = {}
    if fixed:
        body, world_from_body = -1, np.eye(4)
    else:
        world_from_body = pose @ pose_matrix(xyz=c_cad)
        body = rig.builder.add_body(xform=to_wp(world_from_body), label=label)
    body_from_cad = np.linalg.inv(world_from_body) @ pose
    for p, tm in zip(parts, tms):
        shapes[p.name] = [add_part_shape(rig, body, asset, p, body_from_cad, label, mesh=tm)]
    shapes.update(add_extra_boxes(rig, asset, body, body_from_cad, label))
    if not fixed:
        total = mass if mass is not None else sum(part_mass(asset, p) for p in parts) + sum(
            float(ex.get("mass_g", 0.0)) * 1e-3 for ex in asset.sim.get("extra_boxes") or [])
        set_body_mass(rig.builder, body, total)
    return body, shapes


def add_asset(rig, key: str, choice=None, pose: np.ndarray | None = None, label: str | None = None,
              fixed: bool | None = None, **params) -> AssetHandle:
    """Add kit asset ``key`` (variant ``choice``) to ``rig`` at ``pose``.  See the module docstring.

    ``fixed`` overrides sim.yaml's kind for a single rigid asset (a free part bolted down, or a fixture let loose);
    ``params`` override a mechanism's parameters (spring rates, preloads, ...)."""
    kit = load_kit()
    asset = kit[key]
    pose = np.eye(4) if pose is None else np.asarray(pose, dtype=float)
    label = label or key.replace("-", "_")
    kind = asset.sim.get("kind", "free")
    picked = asset.resolve(choice)
    h = AssetHandle(key, label, {k: (v.label if v else None) for k, v in picked.items()}, pose)
    if kind == "mechanism":
        from force_bench.kit.mechanisms import module_for
        module_for(asset).build(rig, asset, h, **params)
        return h
    parts = asset.parts(choice)
    if kind == "assembly":
        roles = asset.sim.get("roles") or {}
        for p in parts:
            role = next((r for pat, r in roles.items()
                         if pat == p.name or (pat.endswith("*") and p.name.startswith(pat[:-1]))), "free")
            part_label = f"{label}/{p.name}"
            body, shapes = add_rigid(rig, asset, [p], pose, part_label, role == "fixture")
            if body >= 0:
                h.bodies[p.name] = body
            h.shapes.update(shapes)
        return h
    is_fixed = (kind == "fixture") if fixed is None else bool(fixed)
    body, shapes = add_rigid(rig, asset, parts, pose, label, is_fixed)
    if body >= 0:
        h.bodies[""] = body
    h.shapes.update(shapes)
    return h
