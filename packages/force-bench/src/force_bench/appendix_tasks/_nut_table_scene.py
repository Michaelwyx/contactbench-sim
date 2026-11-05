"""Scene of ``thread_nut_onto_screw_from_table``: the finalized-kit bolt block
and its nut, lying flat on the table beside it.

ASSETS (``assets/bodies_kit/bolt_block``, metres, kit STEP frame, see its
``provenance.json``): ``bolt.stl`` (hex head captive in the block bottom,
threaded shaft standing up), ``block_bottom.stl`` / ``block_top.stl`` (the
yellow 80 x 30 x 20 mm toy block), ``nut.stl`` (15.5 mm across flats,
7.02 mm thick).  They are the same fastener solids as the suite's
``assets/bodies/fasteners/{screw,nut}_075mm_1_body1`` (same 2.333 mm/turn
right-hand thread, same nut), except that the kit bolt's shaft is 16 mm
longer, so the suite's mesh-measured mate constants do not carry over and
everything below is re-measured on the kit meshes
(:func:`measure_geometry`, ray-cast radius maps; the scratch analysis that
produced the landing table is summarised in the task module's header).

GEOMETRY (kit frame, z up, mm):
  bolt   axis at (x, y) = (-25.0, 0.0) (the hex head's AABB centre); hex
         head z 0.5..8.0 inside the block bottom; plain shank r 4.95 to
         z 19.05; thread (minor r 3.74, major r 4.95, single-start,
         right-hand, lead 2.333 mm/turn) from z 19.05 up to the full-crest
         top at z 42.20, then a tip chamfer to z 43.45.
  block  80 x 30 mm, top face z 20.0 (the bolt protrudes 23.45 mm).
  nut    AABB z -7.010..0.013, centre z -3.498; flats every 60 deg with a
         flat NORMAL at 30 deg (corners on +-x); internal crest r 4.05,
         root r 5.25, full internal thread z -5.95..-1.05 (both faces
         countersunk).

WORLD = kit + (25, 0, 0) mm: the bolt axis is the world origin (the suite's
nut tasks put their bolt there too, so the robot base and the stroke window
of ``nut_bolt_arm`` are reused unchanged).  The block sits on the table
(z = 0) and extends toward +x; the nut lies hex-face down on the table at
``NUT_XY0`` (the -x side, away from the block).

COLLISION: the bolt and the nut carry the suite's thread contact settings
(SDF narrow band 1 mm, 0.1 mm margin, gap 0.3 mm, mu 0.30); the bolt's SDF
voxel matches the suite's 27 mm bolt at max_resolution 512 (0.053 mm) on
this 43 mm part.  The block halves collide through coarse SDFs (nothing is
meant to touch them; a dropped nut would land on them, not fall through).
Static-static and ground-plane pairs are filtered (the table box carries
every rest contact).
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import set_body_mass
from force_bench.geometry import subdiv
from force_bench.paths import ASSETS_ROOT, SDF_CACHE

KIT_DIR = ASSETS_ROOT / "bodies_kit" / "bolt_block"

# ── kit -> world ────────────────────────────────────────────────────────
BOLT_AXIS_KIT = np.array([-0.0250, 0.0])
KIT_TO_WORLD = np.array([0.0250, 0.0, 0.0])
BOLT_AXIS = np.array([0.0, 0.0])            # world xy of the bolt axis

# ── measured geometry (world z, m; measure_geometry() recomputes) ──────
LEAD = 0.002333             # m/turn, right-hand: z rises with CCW yaw
THREAD_TOP = 0.04220        # last z with crest radius >= 4.90 mm
BOLT_TIP = 0.04345
THREAD_BOTTOM = 0.01905
BLOCK_TOP = 0.0200
BLOCK_HALF = (0.040, 0.015)
NUT_ZMIN_KIT, NUT_ZMAX_KIT = -0.0070095, 0.0000134
NUT_H = NUT_ZMAX_KIT - NUT_ZMIN_KIT          # 7.023 mm
NUT_C_KIT = 0.5 * (NUT_ZMIN_KIT + NUT_ZMAX_KIT)
NUT_TOP_ABOVE_C = NUT_ZMAX_KIT - NUT_C_KIT   # 3.511 mm
NUT_AF = 0.015506           # across flats
NUT_AC = 0.017588           # across corners
NUT_FLAT_PHASE = np.radians(30.0)            # a flat normal, kit frame
# The nut is fully engaged when its whole height is on the full-crest
# thread: top face at or below THREAD_TOP, i.e. body (AABB) centre at
ENGAGED_Z = THREAD_TOP - NUT_TOP_ABOVE_C     # 38.69 mm

# ── materials / contact (the suite's thread values, tasks/nut_bolt.py) ─
NUT_MU = 0.30
NUT_MASS = 0.010            # kg -- the suite's value for this fastener pair
                            # (nut_bolt / nut_bolt_arm / screw_nut_onto_screw).
                            # A solid PLA print would be 1.15 g; the stroke
                            # controller's admittance and lag gates were tuned
                            # at 10 g and are reused unchanged.
GAP = 0.0003
SDF_NB = 0.001
SDF_MARGIN = 0.0001
NUT_SDF_RES = 512           # 17.6 mm extent -> 0.034 mm voxels (suite)
BOLT_SDF_VOXEL = 0.000053   # the suite's 27 mm bolt at res 512
MAX_EDGE = 0.002            # pitfall 3.2: no face spans an SDF wall

# ── where the nut starts ────────────────────────────────────────────────
NUT_XY0 = np.array([-0.060, 0.0])
NUT_REST_GAP = 0.00005


def load_kit(stem: str) -> trimesh.Trimesh:
    return trimesh.load(KIT_DIR / f"{stem}.stl", force="mesh")


def kit_world(stem: str) -> trimesh.Trimesh:
    tm = load_kit(stem)
    tm.apply_translation(KIT_TO_WORLD)
    return tm


def sdf_mesh(tm: trimesh.Trimesh, *, res: int | None = None, voxel: float | None = None,
             nb: float = SDF_NB, margin: float = SDF_MARGIN):
    """newton.Mesh centred at its AABB centre (make_newton_mesh's
    convention) with a sparse SDF given by resolution or voxel size."""
    v = np.asarray(tm.vertices, dtype=np.float64)
    c = 0.5 * (v.min(0) + v.max(0))
    mesh = newton.Mesh((v - c).astype(np.float32), np.asarray(tm.faces, np.int32).ravel())
    kw = dict(narrow_band_range=(-nb, nb), margin=margin, cache_dir=SDF_CACHE)
    if voxel is not None:
        ext = float((v.max(0) - v.min(0)).max() + 2 * margin)
        r = int(np.ceil(ext / voxel / 8.0) * 8)
        kw["max_resolution"] = r
    else:
        kw["max_resolution"] = int(res)
    mesh.build_sdf(**kw)
    return mesh, c


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, np.sin(0.5 * yaw), np.cos(0.5 * yaw)])


def build_scene(rig, nut_xy=NUT_XY0, nut_yaw: float = 0.0) -> dict:
    """Table, block, bolt (static) and the free nut on the table.

    ``nut_yaw`` is the nut body's yaw about world z relative to the kit
    frame (the bolt is unrotated, so it is also the thread phase the
    landing analysis is written in)."""
    b = rig.builder
    rig.table(center_xy=(0.0, 0.16), half=0.35)

    # block halves: static, coarse SDF, scenery
    for stem in ("block_bottom", "block_top"):
        tm = kit_world(stem)
        mesh, c = sdf_mesh(subdiv(tm, 0.004), res=256, nb=0.0015, margin=0.0008)
        sid = b.add_shape_mesh(-1, mesh=mesh, xform=wp.transform(wp.vec3(*c), wp.quat_identity()),
                               cfg=rig.cfg(mu=0.5, gap=GAP), label=stem)
        rig.shapes[stem] = [sid]
        rig.meshes[stem] = (tm, c)

    # bolt: static thread
    bolt_tm = subdiv(kit_world("bolt"), MAX_EDGE)
    bolt_mesh, bolt_c = sdf_mesh(bolt_tm, voxel=BOLT_SDF_VOXEL)
    sid = b.add_shape_mesh(-1, mesh=bolt_mesh,
                           xform=wp.transform(wp.vec3(*bolt_c), wp.quat_identity()),
                           cfg=rig.cfg(mu=NUT_MU, gap=GAP), label="bolt")
    rig.shapes["bolt"] = [sid]
    rig.meshes["bolt"] = (bolt_tm, bolt_c)

    # nut: free body, kit orientation, AABB-centred, lying flat on the table
    nut_raw = load_kit("nut")
    nut_tm = subdiv(nut_raw, MAX_EDGE)
    nut_mesh, nut_c = sdf_mesh(nut_tm, res=NUT_SDF_RES)
    z0 = (NUT_C_KIT - NUT_ZMIN_KIT) + NUT_REST_GAP
    p0 = np.array([float(nut_xy[0]), float(nut_xy[1]), z0])
    q0 = yaw_quat(nut_yaw)
    body = b.add_body(xform=wp.transform(wp.vec3(*p0), wp.quat(*q0)), label="nut")
    nsid = b.add_shape_mesh(body, mesh=nut_mesh, cfg=rig.cfg(mu=NUT_MU, gap=GAP),
                            label="nut_shape")
    rig.shapes["nut"] = [nsid]
    set_body_mass(b, body, NUT_MASS)
    nut_local = nut_tm.copy()
    nut_local.apply_translation(-nut_c)
    world0 = nut_local.copy()
    world0.apply_transform(trimesh.transformations.quaternion_matrix([q0[3], q0[0], q0[1], q0[2]]))
    world0.apply_translation(p0)
    rig.meshes["nut"] = (world0, p0)

    # the arm never touches the nut except through the fingertips
    rig.robot_grip(body, filter_fingers=False)
    # static-static pairs and the ground plane carry nothing here
    statics = list(b.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            b.add_shape_collision_filter_pair(statics[i], statics[j])
    ground = statics[0]
    for bb, sids in b.body_shapes.items():
        for s in sids:
            if s != ground:
                b.add_shape_collision_filter_pair(ground, s)
    return dict(nut_body=body, nut_local=nut_local, nut_p0=p0, nut_q0=q0,
                bolt_tm=bolt_tm, bolt_c=np.asarray(bolt_c))


def recolor_kit(model) -> None:
    """Finalized-kit colours after the recorder's role palette: yellow
    block, dark-grey bolt, mid-grey nut (purely visual)."""
    labels = list(model.shape_label)
    col = model.shape_color.numpy().copy()
    for i, lbl in enumerate(labels):
        lbl = lbl or ""
        if lbl.startswith("block_"):
            col[i] = (0.93, 0.83, 0.10)
        elif lbl == "bolt":
            col[i] = (0.30, 0.30, 0.32)
        elif lbl == "nut_shape":
            col[i] = (0.55, 0.56, 0.60)
    model.shape_color.assign(col.astype(np.float32))


# ── offline measurement (documentation; not used at run time) ──────────
def measure_geometry(dz: float = 0.00005) -> dict:
    """Ray-cast radius maps of the kit bolt and nut -> the constants above.
    CPU only (trimesh), ~1 min."""
    bolt = load_kit("bolt")
    nut = load_kit("nut")
    phis = np.radians(np.arange(0.0, 360.0, 2.0))

    def rmap(tm, axis, zs, inward):
        ray = trimesh.ray.ray_triangle.RayMeshIntersector(tm)
        out = np.full((len(zs), len(phis)), np.nan)
        d = np.stack([np.cos(phis), np.sin(phis), np.zeros_like(phis)], 1)
        for k, z in enumerate(zs):
            o = np.c_[np.tile(axis, (len(phis), 1)) + (0.012 * d[:, :2] if inward else 0.0),
                      np.full(len(phis), z)]
            loc, idx, _ = ray.intersects_location(o, -d if inward else d, multiple_hits=False)
            if len(idx) == 0:
                continue
            loc = np.asarray(loc).reshape(-1, 3)
            out[k, idx] = np.linalg.norm(loc[:, :2] - axis, axis=1)
        return out
    zb = np.arange(0.0190, 0.0440, 4 * dz)
    rb = rmap(bolt, BOLT_AXIS_KIT, zb, True)
    rmax = np.nanmax(np.nan_to_num(rb, nan=-1.0), axis=1)
    thread_top = float(zb[np.flatnonzero(rmax >= 0.00490).max()])
    zn = np.arange(-0.0068, -0.0002, 4 * dz)
    rn = rmap(nut, np.zeros(2), zn, False)
    return dict(thread_top_kit_z=thread_top, bolt_rmax_mm=float(np.nanmax(rb)) * 1000,
                nut_crest_r_mm=float(np.nanmin(rn)) * 1000, nut_root_r_mm=float(np.nanmax(rn)) * 1000)
