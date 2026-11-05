"""The USB stick and USB slot of the finalized STEP kit (appendix task 27).

The simulated suite had no USB asset.  The finalized kit
(``STEP-20261015T040134Z-1-001.zip``, tessellated by
``paper-writing/figures/asset-overviews/final_asset_table/source/convert_step.py``
into ``mesh_cache/``: metres, Z-up, the STEP frame) has three sticks and one
slot.  :func:`export_kit` copies them, untouched, to

    assets/bodies_kit/usb_slot/{shell,center,spring_left,spring_right}.stl
    assets/bodies_kit/usb_stick/{usb_1,usb_head_2,usb_head_3}.stl

in METRES (not the centimetres of ``assets/bodies``), each folder with a
``PROVENANCE.json`` (source hashes, units, frame).  The task builds its
colliders from these files at run time (crops, subdivision, SDFs), so the
files themselves stay the CAD.

GEOMETRY (STEP frame, mm, measured on the meshes by ray casts and sections;
:func:`measure` recomputes the numbers):

  stick    all three share ONE plug ("tongue"): x -12..0, a 12.00 x 5.00 mm
           rectangular tube whose trapezoidal channel (open at the tip, top
           at z +1.9, floor at z -0.3) is the keyway.  The tip is square (no
           chamfer).  The grip body starts at x = 0:
             usb_1       16 x 10 rounded box, 8 mm long, 1.5 mm shoulder chamfer
             usb_head_2  20 x 13 rounded box, 40 mm long
             usb_head_3  20 x 10 ELLIPTICAL section, 25 mm long
           solid PLA (1240 kg/m^3): 2.08 / 13.27 / 5.42 g.
  slot     52 x 28 x 22 mm housing ('shell', bolted along z), front face at
           x = 0.  The STEP files share one assembly frame: a stick at the
           identity pose IS fully inserted (shoulder on the front face, tip
           on the back stop at x = -12, both at zero gap).
           - mouth: a 1.15 mm front wall with a 12.35 x 5.35 opening
             (0.175 mm clearance per side) and a ~0.2 mm entry round;
           - behind it a cavity whose z walls (+-2.675) run the full 12 mm
             (the same 0.175 mm), open in y up to +-16 for the springs;
           - 'center': the keyed TONGUE (a trapezoid, z 0..1.5, x -12..-1.3)
             that runs inside the stick's channel (~0.3 mm clearance) -- an
             upside-down stick cannot enter;
           - 'spring left/right': printed flexures.  Each carries a JAW
             block (x -11.5..-1.5, |y| 5.5..8.0, z +-2.25) with 45 deg lead
             chamfers on its inner face, hung on two S-shaped 0.6 mm PLA
             beams from a base plate bolted at |y| 16..26.  The jaw face
             sits at |y| = 5.5 against the stick's 6.0: 0.5 mm of
             interference per side, i.e. the slot CLAMPS the stick.

FRAME.  The real slot stands on its back face, opening up, and the stick goes
in from above (the real-world video and thumbnail).  Task frame =
``R_TASK @ step + T_TASK`` with x_step -> +z (insertion runs down), z_step
(the stick's thin axis, the bolt axis) -> +x, y_step (the 12 mm axis) ->
-y; the back face rests on the table (z = 0) and the opening is at
z = 22 mm.  A gripper closing along world x pinches the stick's wide faces,
as the person in the video does.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import shapely
import trimesh
from scipy.spatial import cKDTree
from shapely.geometry import Polygon
from trimesh import intersections

from force_bench.paths import REPO_ROOT, ASSETS_ROOT

MESH_CACHE = (REPO_ROOT / "paper-writing" / "figures" / "asset-overviews"
              / "final_asset_table" / "mesh_cache")
KIT_DIR = ASSETS_ROOT / "bodies_kit"
SLOT_DIR = KIT_DIR / "usb_slot"
STICK_DIR = KIT_DIR / "usb_stick"

# --usb N -> STEP file stem (website variants "USB 1/2/3")
VARIANTS = {1: "usb_1", 2: "usb_head_2", 3: "usb_head_3"}
SLOT_PARTS = {"shell": "shell", "center": "center",
              "spring left": "spring_left", "spring right": "spring_right"}

# ── task frame ────────────────────────────────────────────────────────────
R_TASK = np.array([[0.0, 0.0, 1.0],
                   [0.0, -1.0, 0.0],
                   [1.0, 0.0, 0.0]])
SLOT_DEPTH = 0.022                      # back face (x=-22) on the table
T_TASK = np.array([0.0, 0.0, SLOT_DEPTH])
MOUTH_Z = SLOT_DEPTH                    # task frame: the slot's top face

# ── measured geometry (STEP frame, m) ─────────────────────────────────────
PLUG_LEN = 0.012                        # tongue: x -12..0
PLUG_HALF_W = 0.0060                    # y half-width (the 12 mm axis)
PLUG_HALF_T = 0.0025                    # z half-thickness
OPEN_HALF_W = 0.006175                  # mouth half-width (y)
OPEN_HALF_T = 0.002675                  # mouth / cavity half-height (z)
FRONT_WALL = 0.00115                    # front wall thickness
JAW_FACE = 0.0055                       # jaw inner face |y| at rest
JAW_X = (-0.0115, -0.0015)              # jaw block x extent
JAW_CROP_Y = 0.0084                     # jaw collider: |y| <= this (block +
                                        # 0.4 mm of the flexure stubs)
# body half-thickness across the pinch (STEP z) and length, per stem
BODY = {"usb_1": dict(half_t=0.0050, half_w=0.0080, length=0.008),
        "usb_head_2": dict(half_t=0.0065, half_w=0.0100, length=0.040),
        "usb_head_3": dict(half_t=0.0050, half_w=0.0100, length=0.025)}
PLA_DENSITY = 1240.0


def to_task(p_step) -> np.ndarray:
    return np.asarray(p_step, float) @ R_TASK.T + T_TASK


def posed(tm: trimesh.Trimesh, offset=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    """STEP-frame trimesh (m) -> task frame, plus a world offset."""
    M = np.eye(4)
    M[:3, :3] = R_TASK
    M[:3, 3] = T_TASK + np.asarray(offset, float)
    out = tm.copy()
    out.apply_transform(M)
    return out


# ── asset export ──────────────────────────────────────────────────────────
def _sha256(p: Path) -> str:
    return hashlib.sha256(p.read_bytes()).hexdigest()


def _cache_parts(stem: str) -> tuple[dict[str, trimesh.Trimesh], dict]:
    d = np.load(MESH_CACHE / f"{stem}.npz")
    meta = json.loads((MESH_CACHE / f"{stem}.json").read_text())
    parts = {p["name"]: trimesh.Trimesh(d[f"v{p['key']}"].astype(np.float64),
                                        d[f"f{p['key']}"], process=True)
             for p in meta["parts"]}
    return parts, meta


def export_kit(force: bool = False) -> list[Path]:
    """Write the USB stick/slot STLs (metres, STEP frame) + PROVENANCE.json."""
    written = []
    jobs = [(SLOT_DIR, "usb_slot", SLOT_PARTS)]
    jobs += [(STICK_DIR, stem, None) for stem in VARIANTS.values()]
    prov: dict[Path, dict] = {}
    for folder, stem, names in jobs:
        folder.mkdir(parents=True, exist_ok=True)
        parts, meta = _cache_parts(stem)
        entry = prov.setdefault(folder, {
            "units": "m", "frame": "STEP (Z-up), assembly coordinates kept",
            "source_zip": "STEP-20261015T040134Z-1-001.zip (repo root, sha256 "
                          "998af1e66c067927175c358ccfb482253b6ec1921fa239c80d5395142babac72)",
            "tessellation": "cascadio 0.1.1 via final_asset_table/source/convert_step.py "
                            "(0.01 mm chordal, 0.5 rad angular)",
            "mesh_cache": str(MESH_CACHE.relative_to(REPO_ROOT)),
            "used_by": "force_bench.appendix_tasks.plug_in_usb (appendix task 27, Plug in USB)",
            "files": {}})
        for name, tm in parts.items():
            fname = (names[name] if names else stem) + ".stl"
            out = folder / fname
            if force or not out.exists():
                tm.export(out)
                written.append(out)
            entry["files"][fname] = {
                "step_file": meta["source"], "step_sha256": meta["source_sha256"],
                "part": name, "fusion_color_srgb": next(
                    p["color"] for p in meta["parts"] if p["name"] == name),
                "n_tris": int(len(tm.faces)), "watertight": bool(tm.is_watertight),
                "bounds_mm": np.round(tm.bounds * 1000, 3).tolist(),
                "stl_sha256": _sha256(out)}
    for folder, entry in prov.items():
        (folder / "PROVENANCE.json").write_text(json.dumps(entry, indent=1))
    return written


def load_slot() -> dict[str, trimesh.Trimesh]:
    """STEP-frame slot parts keyed shell/center/spring_left/spring_right."""
    if not all((SLOT_DIR / f"{v}.stl").exists() for v in SLOT_PARTS.values()):
        export_kit()
    return {v: trimesh.load(SLOT_DIR / f"{v}.stl", force="mesh")
            for v in SLOT_PARTS.values()}


def load_stick(stem: str) -> trimesh.Trimesh:
    p = STICK_DIR / f"{stem}.stl"
    if not p.exists():
        export_kit()
    return trimesh.load(p, force="mesh")


# ── watertight crops (no triangulation engine is installed in the env) ────
def slice_capped(m: trimesh.Trimesh, origin, normal) -> trimesh.Trimesh:
    """Keep the half-space (p - origin) . normal >= 0 of a watertight mesh and
    close the cut with a constrained-Delaunay cap (shapely) built on the kept
    mesh's OWN boundary vertices, so no T-junction opens the result."""
    origin = np.asarray(origin, float)
    n = np.asarray(normal, float) / np.linalg.norm(normal)
    kept = intersections.slice_mesh_plane(m, n, origin, cap=False)
    kept.merge_vertices(digits_vertex=10)
    V = kept.vertices
    uniq, cnt = np.unique(kept.edges_sorted, axis=0, return_counts=True)
    bnd = uniq[cnt == 1]
    if len(bnd) == 0:
        return kept
    on = np.abs((V - origin) @ n) < 1e-7
    bnd = bnd[on[bnd[:, 0]] & on[bnd[:, 1]]]
    adj: dict[int, list[int]] = {}
    for a, b in bnd:
        adj.setdefault(int(a), []).append(int(b))
        adj.setdefault(int(b), []).append(int(a))
    assert all(len(v) == 2 for v in adj.values()), "non-manifold cut boundary"
    seen, loops = set(), []
    for s in adj:
        if s in seen:
            continue
        loop, prev, cur = [s], None, s
        seen.add(s)
        while True:
            nb = adj[cur]
            nxt = nb[0] if nb[0] != prev else nb[1]
            if nxt == s:
                break
            loop.append(nxt)
            seen.add(nxt)
            prev, cur = cur, nxt
        loops.append(loop)
    u = np.cross(n, [1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.cross(n, [0.0, 1.0, 0.0])
    u /= np.linalg.norm(u)
    w = np.cross(n, u)
    P2 = np.column_stack([(V - origin) @ u, (V - origin) @ w])
    polys = [Polygon(P2[lp]) for lp in loops]
    depth = [sum(1 for j, q in enumerate(polys) if j != i and q.contains(p))
             for i, p in enumerate(polys)]
    idx = np.unique(np.concatenate(loops))
    tree = cKDTree(P2[idx])
    faces = []
    for si, d in enumerate(depth):
        if d % 2:
            continue
        holes = [polys[j].exterior.coords for j, dj in enumerate(depth)
                 if dj == d + 1 and polys[si].contains(polys[j])]
        for g in shapely.constrained_delaunay_triangles(
                Polygon(polys[si].exterior.coords, holes)).geoms:
            dist, k = tree.query(np.array(g.exterior.coords)[:3])
            assert dist.max() < 1e-9, "cap vertex off the cut boundary"
            faces.append(idx[k])
    faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
    fn = np.cross(V[faces[:, 1]] - V[faces[:, 0]], V[faces[:, 2]] - V[faces[:, 0]])
    flip = (fn @ n) > 0                       # the cap faces -n (outward)
    faces[flip] = faces[flip][:, ::-1]
    out = trimesh.Trimesh(V, np.vstack([kept.faces, faces]), process=False)
    out.remove_unreferenced_vertices()
    return out


def crop_box(m: trimesh.Trimesh, lo, hi) -> trimesh.Trimesh:
    """Watertight intersection of a watertight mesh with an axis box."""
    out = m
    for ax in range(3):
        for val, sgn in ((lo[ax], 1.0), (hi[ax], -1.0)):
            if val is None:
                continue
            if (sgn > 0 and out.bounds[0, ax] >= val) or (sgn < 0 and out.bounds[1, ax] <= val):
                continue
            nrm = np.zeros(3)
            nrm[ax] = sgn
            o = np.zeros(3)
            o[ax] = val
            out = slice_capped(out, o, nrm)
    assert out.is_watertight, "crop is not watertight"
    return out


# ── measurement (the numbers in the module docstring) ─────────────────────
def _ray(m, o, d):
    loc, _, _ = m.ray.intersects_location([o], [d])
    if len(loc) == 0:
        return None
    return float(np.min(np.linalg.norm(loc - np.asarray(o), axis=1)))


def measure() -> dict:
    """Clearances and interference from ray casts on the slot and sticks."""
    slot = load_slot()
    sh, sr = slot["shell"], slot["spring_right"]
    out = {
        "mouth_half_w_mm": _ray(sh, [-0.0006, 0, 0], [0, 1, 0]) * 1000,
        "mouth_half_t_mm": _ray(sh, [-0.0006, 0, 0], [0, 0, 1]) * 1000,
        "cavity_half_t_mm": _ray(sh, [-0.006, 0, 0], [0, 0, 1]) * 1000,
        "jaw_face_mm": _ray(sr, [-0.006, 0, 0], [0, 1, 0]) * 1000,
    }
    for stem in VARIANTS.values():
        st = load_stick(stem)
        out[stem] = {
            "tongue_half_w_mm": _ray(st, [-0.006, 0.010, 0.0022], [0, -1, 0]) and
            (0.010 - _ray(st, [-0.006, 0.010, 0.0022], [0, -1, 0])) * 1000,
            "mass_g": st.volume * PLA_DENSITY * 1000,
            "bounds_mm": np.round(st.bounds * 1000, 3).tolist()}
    out["clearance_per_side_mm"] = {
        "width": out["mouth_half_w_mm"] - PLUG_HALF_W * 1000,
        "thickness": out["mouth_half_t_mm"] - PLUG_HALF_T * 1000,
        "cavity": out["cavity_half_t_mm"] - PLUG_HALF_T * 1000}
    out["jaw_interference_per_side_mm"] = PLUG_HALF_W * 1000 - out["jaw_face_mm"]
    return out


if __name__ == "__main__":
    import sys
    print("exported:", export_kit(force="--force" in sys.argv))
    print(json.dumps(measure(), indent=1, default=float))
