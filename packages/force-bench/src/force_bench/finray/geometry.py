"""Measured geometry of the Fin Ray finger and of its FR3 adapter.

Frames
------
STEP frame (mm), the finger as exported (``assets/finray/finray_finger.step``):
  x  depth: the straight contact flank is the face x = 0, the back flank
     reaches x = 40 at the base
  y  extrusion, -25.8 .. 0
  z  length: base top z = 0, tip z = -123.36
FR3 finger frame (m), ``fr3_leftfinger`` / ``fr3_rightfinger`` of
``fr3_franka_hand.urdf``:
  x  across the finger, y outward (the stock pad face is the plane y = 0,
     the pad gap is 2 q), z along the approach axis away from the hand;
     the frame origin is 58.4 mm from ``fr3_hand``
Mount (the adapter, ``assets/finray/adapter_finray_fr3.py``):
  x_f = y_s + 12.9,   y_f = x_s,   z_f = Z_BASE - z_s          (mm)
so the contact face sits exactly where the stock pad face is: the 0-80 mm
stroke is kept and two closed fingers touch.

Sources of the adapter numbers (see ``assets/finray/README.md``):
  official drawing   Franka Hand product manual R50010 v1.2, 5.2.3 / 5.2.4:
                     tab 6.00 thick x 21 wide, protrudes 11.40 from the
                     housing, 2 x M4 12 mm apart, 7 mm from the tab end
                     (15 - 8), tab inner faces 25 mm apart when closed
  official files     FR3 description (franka_description meshes, the ones
                     Newton downloads): housing underside 66.0 mm below
                     fr3_hand = 7.6 mm into the finger frame; finger.dae
                     holes at z = 12.0, x = +-5.8
  derived            tab end z = 7.6 + 11.4 = 19.0, holes z = 19.0 - 7.0 =
                     12.0 (agrees with finger.dae), tab y = 12.5 .. 18.5
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path

import numpy as np

from force_bench.paths import ASSETS_ROOT

FINRAY_DIR = ASSETS_ROOT / "finray"
FINGER_STL = FINRAY_DIR / "finray_finger.stl"          # mm, watertight
PROFILE_JSON = FINRAY_DIR / "finray_profile.json"      # cache of profile()

# ── the finger (STEP frame, mm) ─────────────────────────────────────────
WIDTH = 25.8              # extrusion
DEPTH = 40.0              # base block, along the closing axis
LENGTH = 123.36           # base top to tip
BASE_H = 10.0             # base block height (z 0 .. -10)
BASE_HOLES = ((5.0, -5.0), (35.0, -5.0))     # Ø4.0 through y
LUG_HOLE = (35.0, -40.0)                      # Ø4.0, on the back-flank lug
HOLE_D = 4.0
VOLUME_CM3 = 47.07        # measured on the STL
TPU_DENSITY = 1210.0      # kg/m^3, TPU 95A (filament data sheets 1.20-1.23)

# ── the FR3 carriage tab (finger frame, mm) ─────────────────────────────
TAB_THICK = 6.0           # official (Fig. 5.5)
TAB_WIDTH = 21.0          # official
TAB_PROTRUSION = 11.4     # official, from the housing underside
TAB_HOLE_SPACING = 12.0   # official, M4
TAB_HOLE_FROM_END = 7.0   # official (15 - 8)
TAB_GAP_CLOSED = 25.0     # official (Fig. 5.4): tab inner faces, fingers closed
HOUSING_Z = 7.6           # FR3 description: hand.dae underside 66.0 - 58.4
TAB_INNER_Y = TAB_GAP_CLOSED / 2.0                 # 12.5
TAB_END_Z = HOUSING_Z + TAB_PROTRUSION             # 19.0
TAB_HOLE_Z = TAB_END_Z - TAB_HOLE_FROM_END         # 12.0

# ── the adapter (finger frame, mm) -- keep in step with adapter_finray_fr3.py
ADAPTER_TOP_Z = 8.4       # 0.8 mm under the housing
POCKET_CLEAR = 0.15       # per side, around the 6 mm tab
POCKET_BOTTOM_Z = TAB_END_Z + 0.4
BRIDGE = 4.0              # floor between the tab pocket and the finger
Z_BASE = POCKET_BOTTOM_Z + BRIDGE                  # 23.4: finger base top
ADAPTER_INNER_Y = 1.0     # adapter stays 1 mm behind the contact face
CHEEK = 5.0               # clevis cheeks either side of the base block
CHEEK_CLEAR = 0.1
OUTER_CHEEK = 5.0         # behind the tab, carries the two M4 x 12 heads
ADAPTER_MASS = 0.0178     # kg, PETG at 100 %, the CAD volume (13.98 cm^3); the sim uses its boxes

TIP_Z = Z_BASE + LENGTH                            # 146.76 mm
HAND_TO_FINGER_Z = 58.4   # URDF fr3_finger_joint origin
HAND_TO_TCP_Z = 103.4     # URDF fr3_hand_tcp
TIP_BELOW_TCP = (HAND_TO_FINGER_Z + TIP_Z - HAND_TO_TCP_Z) / 1000.0  # 0.10176 m


def step_to_finger(p_mm) -> np.ndarray:
    """STEP-frame points (mm) -> FR3 finger-frame points (m)."""
    p = np.atleast_2d(np.asarray(p_mm, float))
    out = np.stack([p[:, 1] + WIDTH / 2.0, p[:, 0], Z_BASE - p[:, 2]], axis=1)
    return out / 1000.0


# rotation part of step_to_finger (acts on STEP vectors)
R_STEP_TO_FINGER = np.array([[0.0, 1.0, 0.0],
                             [1.0, 0.0, 0.0],
                             [0.0, 0.0, -1.0]])


def finger_mesh_vertices_m() -> tuple[np.ndarray, np.ndarray]:
    """The finger STL in the FR3 finger frame (m): (vertices, faces)."""
    import trimesh
    m = trimesh.load(FINGER_STL, force="mesh")
    return step_to_finger(np.asarray(m.vertices)), np.asarray(m.faces)


# ── the 2-D profile: walls and ribs, measured off the STL ───────────────
def _section_polygon():
    import trimesh
    from shapely.geometry import Polygon
    m = trimesh.load(FINGER_STL, force="mesh")
    sec = m.section(plane_origin=[0.0, -WIDTH / 2.0, 0.0], plane_normal=[0.0, 1.0, 0.0])
    loops = [sec.vertices[e.points][:, [0, 2]] for e in sec.entities]
    polys = sorted((Polygon(p) for p in loops), key=lambda p: -p.area)
    return polys[0], polys[1:]


def _outer_edge(c: np.ndarray, x_wall: float) -> np.ndarray:
    """The curved (back-flank) edge of a rib cavity, top to bottom."""
    n = len(c)
    keep = []
    for k in range(n):
        if abs(c[k, 0] - x_wall) < 1e-3:
            continue
        a, b = c[k - 1], c[(k + 1) % n]
        if np.linalg.norm(c[k] - 0.5 * (a + b)) < 1e-2 and np.linalg.norm(b - a) > 5.0:
            continue                       # mid-vertex of a straight rib face
        keep.append(c[k])
    e = np.asarray(keep)
    return e[np.argsort(-e[:, 1])]


def _polyline_intersect(p0, d, poly: np.ndarray):
    """First intersection of the ray p0 + s d (s > 0) with a polyline."""
    best = None
    for a, b in zip(poly[:-1], poly[1:]):
        m = np.array([d, a - b]).T
        if abs(np.linalg.det(m)) < 1e-12:
            continue
        s, u = np.linalg.solve(m, a - p0)
        if s > 0 and -1e-9 <= u <= 1 + 1e-9 and (best is None or s < best[0]):
            best = (s, a + u * (b - a), u, (a, b))
    return best


def extract_profile() -> dict:
    """Walls, ribs and the back-flank midline of the finger (STEP mm)."""
    outer, holes = _section_polygon()
    cav = [h for h in holes if h.area > 30.0]
    cav.sort(key=lambda h: -h.centroid.y)
    x_wall = min(float(np.asarray(h.exterior.coords)[:, 0].min()) for h in cav)   # 1.8
    t_contact = x_wall
    # back-flank inner surface: the cavities' curved edges, top to bottom
    inner = np.concatenate([_outer_edge(np.asarray(h.exterior.coords)[:-1], x_wall)
                            for h in cav])
    # back-flank outer surface: the outer contour's back side, the lug
    # bump (around LUG_HOLE) replaced by a straight chord
    oc = np.asarray(outer.exterior.coords)
    back = oc[(oc[:, 0] > x_wall + 0.5) & (oc[:, 1] < -0.5)]
    back = back[np.argsort(-back[:, 1])]
    lug = np.hypot(back[:, 0] - LUG_HOLE[0], back[:, 1] - LUG_HOLE[1]) < 7.5
    back = back[~lug]
    # back-flank midline: each inner-surface point paired with its nearest
    # outer-surface point
    from shapely.geometry import LineString, Point
    outer_line = LineString(back)
    mid, thick = [], []
    for p in inner:
        q = outer_line.interpolate(outer_line.project(Point(p)))
        q = np.array([q.x, q.y])
        mid.append(0.5 * (p + q))
        thick.append(float(np.linalg.norm(q - p)))
    mid, thick = np.asarray(mid), np.asarray(thick)
    # the base end of the midline at the base block's underside
    mid = np.vstack([[mid[0, 0], -BASE_H], mid])
    thick = np.concatenate([[thick[0]], thick])
    # ribs: between consecutive cavities
    ribs = []
    for i in range(len(cav) - 1):
        c0 = np.asarray(cav[i].exterior.coords)[:-1]
        c1 = np.asarray(cav[i + 1].exterior.coords)[:-1]
        in0 = c0[np.isclose(c0[:, 0], x_wall)]
        in1 = c1[np.isclose(c1[:, 0], x_wall)]
        zb0, zt1 = in0[:, 1].min(), in1[:, 1].max()
        oe0, oe1 = _outer_edge(c0, x_wall), _outer_edge(c1, x_wall)
        out0, out1 = oe0[-1], oe1[0]          # bottom of cavity i, top of i+1
        a_in = np.array([x_wall, 0.5 * (zb0 + zt1)])
        a_out = 0.5 * (out0 + out1)
        d = (a_out - a_in) / np.linalg.norm(a_out - a_in)
        nrm = np.array([-d[1], d[0]])
        t = float(abs((np.array([x_wall, zb0]) - np.array([x_wall, zt1])) @ nrm))
        # extend the rib centreline to both wall midlines
        p_in = a_in + d * ((t_contact / 2.0 - a_in[0]) / d[0])
        hit = _polyline_intersect(a_in, d, mid)
        p_out = hit[1]
        k = int(np.argmin(np.linalg.norm(mid - p_out, axis=1)))
        ribs.append({"inner": p_in.tolist(), "outer": p_out.tolist(), "t": round(t, 3),
                     "t_back": float(thick[k]),
                     "angle_deg": float(np.degrees(np.arctan2(d[1], d[0])))})
    # the tip: where the two walls meet
    tip = np.array([t_contact / 2.0, float(oc[:, 1].min()) + 1.5])
    zmin = float(oc[:, 1].min())
    return {
        "units": "mm, STEP frame (x depth, z length)",
        "contact_wall_t": t_contact, "contact_mid_x": t_contact / 2.0,
        "base_z": -BASE_H, "tip": tip.tolist(), "tip_z": zmin,
        "back_mid": mid.tolist(), "back_t": thick.tolist(),
        "ribs": ribs,
        "area_mm2": float(outer.area - sum(h.area for h in holes)),
        "n_cavities": len(cav),
    }


@lru_cache(maxsize=1)
def profile() -> dict:
    """The cached profile (``assets/finray/finray_profile.json``)."""
    if PROFILE_JSON.exists():
        return json.loads(PROFILE_JSON.read_text())
    p = extract_profile()
    PROFILE_JSON.write_text(json.dumps(p, indent=1))
    return p


def back_thickness_at(z: float) -> float:
    """Back-flank wall thickness (mm) at STEP height z, from the midline."""
    p = profile()
    mid = np.asarray(p["back_mid"])
    t = np.asarray(p["back_t"])
    return float(np.interp(-z, -mid[:, 1], t))


@lru_cache(maxsize=1)
def _back_face_table():
    """(h above the tip, back-face x) of the outer contour, mm, every 0.25 mm."""
    from shapely.geometry import LineString
    outer, _ = _section_polygon()
    zmin = float(np.asarray(outer.exterior.coords)[:, 1].min())
    hs = np.arange(0.0, LENGTH - BASE_H + 0.25, 0.25)
    xs = []
    for h in hs:
        z = zmin + max(h, 0.05)
        seg = outer.intersection(LineString([(-1.0, z), (DEPTH + 5.0, z)]))
        pts = [c[0] for g in getattr(seg, "geoms", [seg]) for c in g.coords]
        xs.append(max(pts) if pts else 0.0)
    return hs, np.maximum.accumulate(np.asarray(xs))


def outer_x_at(z: float) -> float:
    """Back-face x (mm) of the finger at STEP height z: the finger's thickness
    there, measured on the outer contour (no lug; monotone toward the base)."""
    hs, xs = _back_face_table()
    return float(np.interp(z + LENGTH, hs, xs))


if __name__ == "__main__":
    p = extract_profile()
    PROFILE_JSON.write_text(json.dumps(p, indent=1))
    print(f"wrote {PROFILE_JSON}: {len(p['ribs'])} ribs, contact wall "
          f"{p['contact_wall_t']:.2f} mm, area {p['area_mm2']:.1f} mm^2 "
          f"(x {WIDTH} = {p['area_mm2'] * WIDTH / 1000:.2f} cm^3)")
    for i, r in enumerate(p["ribs"]):
        print(f"  rib {i + 1:2d}: inner z {r['inner'][1]:7.2f}  outer ({r['outer'][0]:5.2f},"
              f"{r['outer'][1]:7.2f})  t {r['t']:.2f}  angle {r['angle_deg']:6.1f}  "
              f"back wall {r['t_back']:.2f}")
    mid = np.asarray(p["back_mid"])
    print("back midline z:", np.round(mid[::6, 1], 1).tolist())
    print("back thickness:", np.round(np.asarray(p["back_t"])[::6], 2).tolist())
    print(f"TIP_BELOW_TCP {TIP_BELOW_TCP * 1000:.2f} mm, TIP_Z {TIP_Z:.2f} mm, Z_BASE {Z_BASE:.2f} mm")
