"""Scene, geometry and audits for the "Place card into holder" appendix tasks
(Table I, task 24): ``place_card_into_holder`` (card starts in the hand) and
``place_card_into_holder_from_platform`` (card first picked off a platform).

    Description (appendix): "A card holder that already contains other cards
    is placed in the workspace. Place the card held in hand into the card
    holder."  Success: "The robot movement stops after placing the card in
    the card holder, with existing cards remaining in the card holder. The
    robot should not remain in contact with the task objects."

ASSETS: the finalized kit (``assets/bodies_kit/card_holder_kit``, metres,
STEP frame = world frame here): ``holder.stl`` and ``card_N.stl`` (one
1.2 x 85 x 55 mm card each).  It is the same holder family as the suite's
``assets/bodies/card_holder`` (identical slot; the kit's base feet are
50 mm wide instead of 58 mm).

HOLDER GEOMETRY (measured off holder.stl by ray scans, kit frame, mm):
  slot      x -7.0 .. +7.0 (14 mm), y -43.25 .. +43.25 (86.5 mm), floor z 2.0
  side walls 2 mm thick (|x| 7..9), top z 17.0 (inner top edge ~1 mm fillet)
  end walls  2 mm thick (|y| 43.25..45.25) over the full slot width, z to 37
  U-bracket posts on both side walls at both ends: |x| 7..9, |y| 33.25..43.25
            (inside the slot; a gusset below z 20), top z 37.0
  feet      |x| <= 25 at both ends, z 0..5
A card (85 mm) has 0.75 mm of play per end inside the 86.5 mm slot, so the
hand has to put it within +-0.75 mm in y before its bottom edge passes the
z = 37 bracket tops.

PHYSICS MODEL (tasks/card_pick.py's, which passes its QA, carried over):
* every card is an ANALYTIC BOX collider (1.2 x 85 x 55 mm, the true slab)
  with the kit card mesh as a non-colliding visual; card mass = solid PLA
  volume (5.6 cm^3 x 1240 kg/m^3 = 6.9 g; the kit cards are printed);
* the cards meet ANALYTIC SLOT SUPPORTS (floor, side walls, end walls,
  bracket posts) placed exactly on the measured CAD faces; the CAD holder
  mesh is filtered against the cards (a coincident second contact manifold
  made resting cards jitter in card_pick) but stays live for the ROBOT and
  is the ground truth of the penetration audit;
* gap 0.8 mm per shape (card_pick: the sum exceeds the 2 mm wall, so a card
  can never be clear of both wall faces at once), collision every substep,
  mu card 0.25 / holder 0.30 / pads 1.4 (MuJoCo takes the pair's MAX);
* a small non-zero torsional/rolling friction on every scene shape: condim 4
  with mu_torsional 0 is a degenerate MuJoCo row (the NaN warning card_pick
  prints for every card).

THE PRE-LOADED DECK.  N cards leaning toward -x as one block: the rear
card's -x face on the -x bracket posts' top corners (x = -7, z = 37), every
bottom edge on the floor, faces parallel and touching, in closed form
(``deck_layout``; card_pick's deck_pose construction, mirrored onto the kit
frame).  card_pick leans its 4-card deck at the LARGEST angle the slot
allows (13.75 deg), i.e. with the bottoms against the far wall, and that is
the one deck nothing can be added to: a fifth card's bottom edge wedges
between the front card and the +x wall 3.8 mm above the floor (12 deg
wedge, inside the 14 deg friction cone: it self-locks).  So the holder here
holds N = 3 cards at 12 deg, which leaves the room the real holder in the
reference video has in front of its deck: 2.9 mm of floor between the front
card and the +x wall, 1.66 mm once the new card is in.  A deck at less than
its maximum lean stands on floor friction: it needs mu >= 0.77 * sin(2 th)
/ 2 = 0.16 at 12 deg against the 0.30 it has, and the rollout's first
0.6 s (robot still) is the measured proof that it is at rest.
"""

from __future__ import annotations

import gc

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv, world_T
from force_bench.paths import ASSETS_ROOT

MM = 1000.0
KIT_DIR = ASSETS_ROOT / "bodies_kit" / "card_holder_kit"

# ── cards ───────────────────────────────────────────────────────────────
CARD_T, CARD_L, CARD_H = 0.0012, 0.085, 0.055
PLA_DENSITY = 1240.0
CARD_MU = 0.25
HOLDER_MU = 0.30
MU_TORSION = 0.002            # condim-4 rows need a non-zero coefficient
MU_ROLLING = 0.0001
GAP = 0.0008                  # contact detection gap per shape (card_pick)

# ── holder (kit frame, measured; see the header) ─────────────────────────
SLOT_X = (-0.007, 0.007)
SLOT_Y = (-0.04325, 0.04325)
FLOOR_Z = 0.002
WALL_T = 0.002
WALL_TOP_Z = 0.017
POST_TOP_Z = 0.037
POST_Y_IN = 0.03325           # posts span |y| POST_Y_IN .. SLOT_Y[1] inside
HOLDER_SDF = dict(sdf=True, sdf_resolution=320, narrow_band=0.002, margin=0.0005)
HOLDER_MAX_EDGE = 0.0025      # pitfall 3.2 (robot contacts sample its vertices)

# ── the deck ─────────────────────────────────────────────────────────────
N_DECK = 3
DECK_LEAN_DEG = 12.0
FLOOR_CLEAR = 0.00002         # as built: bottoms 0.02 mm above the floor ...
POST_CLEAR = 0.00002          # ... rear face 0.02 mm off the post corner ...
FACE_CLEAR = 0.00001          # ... and 0.01 mm between faces (card_pick)


def deck_names(n_cards: int) -> list[str]:
    """Rear (on the -x posts) ... front (the free side)."""
    if n_cards == 1:
        return ["card_front"]
    return ["card_rear"] + [f"card_mid{k}" if n_cards > 3 else "card_mid"
                            for k in range(1, n_cards - 1)] + ["card_front"]

# ── FR3 fingertip envelope (measured collision shapes; card_pick / hang) ─
TIP_BELOW_TCP = 0.0097        # fingertip bottoms, along the approach axis
PAD_FACE_OFF = 0.000085       # pad face = dof - this
FINGER_BACK = 0.0152          # fingertip thickness behind the pad face


def Ry(phi: float) -> np.ndarray:
    c, s = np.cos(phi), np.sin(phi)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_of_R(R: np.ndarray) -> np.ndarray:
    """xyzw quaternion of a rotation matrix."""
    M = np.eye(4)
    M[:3, :3] = R
    q = trimesh.transformations.quaternion_from_matrix(M)       # w x y z
    return np.array([q[1], q[2], q[3], q[0]])


def R_of_q(q) -> np.ndarray:
    return trimesh.transformations.quaternion_matrix([q[3], q[0], q[1], q[2]])[:3, :3]


def smooth5(u: float) -> float:
    """Quintic smootherstep (zero velocity and acceleration at both ends)."""
    u = float(np.clip(u, 0.0, 1.0))
    return u ** 3 * (u * (6.0 * u - 15.0) + 10.0)


def lean_R(lean: float) -> np.ndarray:
    """Card orientation leaning toward -x by `lean` (rad): the card's local
    +x (face normal) maps to n = (cos, 0, sin), local +z (up its own plane)
    to u = (-sin, 0, cos)."""
    return Ry(-float(lean))


def card_mass() -> float:
    return CARD_T * CARD_L * CARD_H * PLA_DENSITY


# ── deck layout (closed form) ───────────────────────────────────────────
def deck_layout(n_cards: int = N_DECK, lean_deg: float = DECK_LEAN_DEG):
    """Body origins + rotation of an n-card deck leaning toward -x.

    Card k = 0 is the REAR card (its -x face on the -x post corners), k =
    n-1 the FRONT card (the free side).  Returns (R, z, xs, pitch)."""
    th = np.radians(lean_deg)
    R = lean_R(th)
    z = FLOOR_Z + FLOOR_CLEAR + 0.5 * CARD_T * np.sin(th) + 0.5 * CARD_H * np.cos(th)
    # rear -x face plane through the post corner (x = SLOT_X[0], z = POST_TOP_Z)
    x0 = SLOT_X[0] + (POST_TOP_Z - z) * np.tan(th) + (0.5 * CARD_T + POST_CLEAR) / np.cos(th)
    pitch = (CARD_T + FACE_CLEAR) / np.cos(th)
    return R, float(z), [float(x0 + k * pitch) for k in range(n_cards)], float(pitch)


def seat_pose(n_cards: int = N_DECK, lean_deg: float = DECK_LEAN_DEG):
    """Where the NEW card would sit: one more card in front of the deck,
    bottom on the floor, face on the front card's face."""
    R, z, xs, pitch = deck_layout(n_cards + 1, lean_deg)
    return R, np.array([xs[-1], 0.0, z])


def bottom_corner_gap(n_cards: int, lean_deg: float) -> float:
    """Floor room between the front card's +x bottom corner and the +x wall."""
    th = np.radians(lean_deg)
    _, _, xs, _ = deck_layout(n_cards, lean_deg)
    return SLOT_X[1] - (xs[-1] + 0.5 * CARD_T * np.cos(th) + 0.5 * CARD_H * np.sin(th))


# ── box geometry of a card ──────────────────────────────────────────────
HALF = np.array([0.5 * CARD_T, 0.5 * CARD_L, 0.5 * CARD_H])
CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                    for sz in (-1, 1)]) * HALF
BOTTOM_CORNERS = CORNERS[CORNERS[:, 2] < 0]
TOP_CORNERS = CORNERS[CORNERS[:, 2] > 0]


def card_frame(bq_row):
    """(centre, R, n, u) of a card body_q row (n = face normal toward the
    card's local +x, u = up its own plane)."""
    T = world_T(bq_row)
    R = T[:3, :3]
    return T[:3, 3].copy(), R, R[:, 0].copy(), R[:, 2].copy()


def card_corners(bq_row, local=CORNERS) -> np.ndarray:
    c, R, _, _ = card_frame(bq_row)
    return c + local @ R.T


def card_lean_deg(bq_row) -> float:
    """Lean of the card plane from vertical toward -x (deg, signed)."""
    _, _, n, _ = card_frame(bq_row)
    if n[0] < 0:
        n = -n
    return float(np.degrees(np.arctan2(n[2], n[0])))


def point_in_box_depth(pts, bq_row, half=HALF) -> np.ndarray:
    """Per point: depth inside the card box (> 0 inside, = distance to the
    nearest face), negative outside (crude: max of the axis excesses)."""
    c, R, _, _ = card_frame(bq_row)
    loc = (np.asarray(pts) - c) @ R
    d = half - np.abs(loc)
    inside = (d > 0).all(axis=1)
    out = np.where(inside, d.min(axis=1), -np.linalg.norm(np.maximum(-d, 0.0), axis=1))
    return out


# ── meshes ──────────────────────────────────────────────────────────────
def load_holder() -> trimesh.Trimesh:
    return trimesh.load(KIT_DIR / "holder.stl", force="mesh")


def load_card_local() -> trimesh.Trimesh:
    """Kit card_1.stl centred on its AABB (1.2 x 85 x 55, thickness on x)."""
    tm = trimesh.load(KIT_DIR / "card_1.stl", force="mesh")
    tm.apply_translation(-tm.bounds.mean(axis=0))
    ext = tm.extents
    assert np.allclose(ext, [CARD_T, CARD_L, CARD_H], atol=2e-5), ext
    return tm


def clearance_to_mesh(tm: trimesh.Trimesh, pts: np.ndarray) -> float:
    """Smallest distance of pts to tm, -1 if any point is inside (ray
    verified, the face-normal sign alone false-positives near edges)."""
    pq = trimesh.proximity.ProximityQuery(tm)
    closest, dist, tri = pq.on_surface(pts)
    cand = (np.einsum("ij,ij->i", pts - closest, tm.face_normals[tri]) < 0) & (dist > 1e-7)
    if cand.any() and tm.contains(pts[cand]).any():
        return -1.0
    return float(dist.min())


# ── scene construction ──────────────────────────────────────────────────
class Scene:
    """What build_scene made: labels, body indices (after finalize), meshes."""

    def __init__(self):
        self.deck = []                 # deck card labels, rear -> front
        self.new = "card_new"
        self.proxy_labels = []
        self.holder_shape = None
        self.holder_tm = None          # subdivided CAD (robot collider, audit)
        self.holder_raw = None
        self.card_local = None
        self.card_visual = None
        self.R_deck = None
        self.deck_z = None
        self.deck_xs = None
        self.pitch = None
        self.lean = None


def _cfg(rig, mu, **kw):
    kw.setdefault("gap", GAP)
    kw.setdefault("mu_torsional", MU_TORSION)
    kw.setdefault("mu_rolling", MU_ROLLING)
    return rig.cfg(mu=mu, **kw)


def build_holder(rig, sc: Scene, offset=(0.0, 0.0, 0.0)):
    """Static CAD holder (robot collider, visual) + analytic card supports."""
    off = np.asarray(offset, float)
    raw = load_holder()
    raw.apply_translation(off)
    tm = subdiv(raw, HOLDER_MAX_EDGE)
    mesh, c = make_newton_mesh(tm, **HOLDER_SDF)
    c = np.asarray(c, float)
    sid = rig.builder.add_shape_mesh(
        -1, mesh=mesh, xform=wp.transform(wp.vec3(*c), wp.quat_identity()),
        cfg=_cfg(rig, HOLDER_MU), label="holder")
    rig.meshes["holder"] = (tm, c)
    rig.shapes["holder"] = [sid]
    sc.holder_shape, sc.holder_tm, sc.holder_raw = sid, tm, raw

    robot_shapes = [s for b, lbl in enumerate(rig.builder.body_label)
                    if str(lbl).startswith("fr3/") for s in rig.builder.body_shapes.get(b, [])]

    def proxy(label, lo, hi):
        lo, hi = np.asarray(lo, float) + off, np.asarray(hi, float) + off
        ctr, half = 0.5 * (lo + hi), 0.5 * (hi - lo)
        s = rig.builder.add_shape_box(
            -1, xform=wp.transform(wp.vec3(*ctr), wp.quat_identity()),
            hx=float(half[0]), hy=float(half[1]), hz=float(half[2]),
            cfg=_cfg(rig, HOLDER_MU, is_visible=False), label=label)
        rig.shapes.setdefault("holder_proxies", []).append(s)
        sc.proxy_labels.append(label)
        for rs in robot_shapes:       # the robot meets the CAD holder instead
            rig.builder.add_shape_collision_filter_pair(s, rs)
        return s

    x0, x1 = SLOT_X
    y0, y1 = SLOT_Y
    proxy("holder_floor_proxy", (x0, y0, FLOOR_Z - 0.002), (x1, y1, FLOOR_Z))
    proxy("holder_wall_neg_proxy", (x0 - WALL_T, y0, FLOOR_Z), (x0, y1, WALL_TOP_Z))
    proxy("holder_wall_pos_proxy", (x1, y0, FLOOR_Z), (x1 + WALL_T, y1, WALL_TOP_Z))
    for yn, ys in (("neg", -1.0), ("pos", 1.0)):
        ya, yb = sorted((ys * POST_Y_IN, ys * y1))
        proxy(f"holder_post_neg_{yn}_proxy", (x0 - WALL_T, ya, WALL_TOP_Z), (x0, yb, POST_TOP_Z))
        proxy(f"holder_post_pos_{yn}_proxy", (x1, ya, WALL_TOP_Z), (x1 + WALL_T, yb, POST_TOP_Z))
        ea, eb = sorted((ys * y1, ys * (y1 + WALL_T)))
        proxy(f"holder_end_{yn}_proxy", (x0 - WALL_T, ea, FLOOR_Z), (x1 + WALL_T, eb, POST_TOP_Z))
    return sid


def add_card(rig, sc: Scene, label: str, pos, R, visual_mesh) -> int:
    """A free card: analytic box collider + kit mesh visual."""
    q = quat_of_R(R)
    body = rig.builder.add_body(xform=wp.transform(wp.vec3(*np.asarray(pos, float)),
                                                   wp.quat(*q.tolist())), label=label)
    rig.builder.add_shape_mesh(
        body, mesh=visual_mesh,
        cfg=newton.ModelBuilder.ShapeConfig(has_shape_collision=False, density=0.0),
        label=label + "_visual")
    s = rig.builder.add_shape_box(
        body, hx=float(HALF[0]), hy=float(HALF[1]), hz=float(HALF[2]),
        cfg=_cfg(rig, CARD_MU, is_visible=False), label=label + "_shape")
    rig.builder.add_shape_collision_filter_pair(s, sc.holder_shape)   # proxies instead
    rig.shapes[label] = [s]
    set_body_mass(rig.builder, body, card_mass())
    rig.meshes[label] = (sc.card_local, np.zeros(3))
    return body


def build_deck(rig, sc: Scene, n_cards=N_DECK, lean_deg=DECK_LEAN_DEG, offset=(0, 0, 0)):
    off = np.asarray(offset, float)
    R, z, xs, pitch = deck_layout(n_cards, lean_deg)
    sc.R_deck, sc.deck_z, sc.pitch, sc.lean = R, z + off[2], pitch, np.radians(lean_deg)
    sc.deck_xs = [x + off[0] for x in xs]
    vis, _ = make_newton_mesh(sc.card_local, sdf=False)
    sc.card_visual = vis
    names = deck_names(n_cards)
    for lbl, x in zip(names, sc.deck_xs):
        add_card(rig, sc, lbl, (x, off[1], sc.deck_z), R, vis)
        sc.deck.append(lbl)
    # build-time audit: every deck card clear of the CAD holder
    cl = [clearance_to_mesh(sc.holder_raw, sc.card_local.vertices @ R.T
                            + np.array([x, off[1], sc.deck_z])) for x in sc.deck_xs]
    print(f"[card_place] deck: {n_cards} cards, lean {lean_deg:.2f} deg, pitch "
          f"{pitch*MM:.4f} mm, x {np.round(np.array(sc.deck_xs)*MM, 3)} mm, z "
          f"{sc.deck_z*MM:.3f} mm; CAD clearance {np.round(np.array(cl)*MM, 3)} mm; "
          f"floor room in front {bottom_corner_gap(n_cards, lean_deg)*MM:.2f} mm")
    assert min(cl) >= 0.5 * FLOOR_CLEAR, f"deck placement collides with the holder: {cl}"
    return names


def filter_statics_and_ground(rig):
    """Static-static pairs carry nothing; the ground plane coincides with the
    table top and would double every table contact."""
    statics = list(rig.builder.body_shapes.get(-1, []))
    for i in range(len(statics)):
        for j in range(i + 1, len(statics)):
            rig.filter_pair(statics[i], statics[j])
    ground = statics[0]
    for b, sids in rig.builder.body_shapes.items():
        for s in sids:
            if s != ground:
                rig.filter_pair(ground, s)


# ── penetration audit (ground truth: CAD holder, ray-verified sign) ─────
class RobustDepth:
    """card_pick's audit primitive: closest-point depth with normal-sign
    candidates verified by ray-parity containment on the RAW mesh (the face-
    normal sign alone false-positives on 1.2 mm slabs near sharp edges)."""

    def __init__(self, mesh_fine: trimesh.Trimesh, mesh_raw: trimesh.Trimesh):
        self.mesh = mesh_fine
        self.pq = trimesh.proximity.ProximityQuery(mesh_fine)
        self.raw = mesh_raw

    def depth(self, pts: np.ndarray, pad: float = 0.004) -> float:
        lo, hi = self.mesh.bounds
        m = ((pts > lo - pad) & (pts < hi + pad)).all(axis=1)
        if not m.any():
            return -pad
        pts = pts[m]
        closest, dist, tri = self.pq.on_surface(pts)
        cand = (np.einsum("ij,ij->i", pts - closest, self.mesh.face_normals[tri]) < 0) \
            & (dist > 1e-6)
        if cand.any():
            ci = np.where(cand)[0]
            if len(ci) > 64:
                ci = ci[np.argsort(dist[ci])[-64:]]
            ins = self.raw.contains(pts[ci])
            if ins.any():
                return float(dist[ci][ins].max())
        out = dist[~cand]
        return float(-out.min()) if len(out) else -pad


def card_surface_points(spacing=0.0015) -> np.ndarray:
    """Points on the six faces of a card box (local frame)."""
    hx, hy, hz = HALF
    ys = np.arange(-hy, hy + 1e-9, spacing)
    zs = np.arange(-hz, hz + 1e-9, spacing)
    xs = np.array([-hx, 0.0, hx])
    Y, Z = np.meshgrid(ys, zs)
    faces = [np.column_stack([np.full(Y.size, sx), Y.ravel(), Z.ravel()]) for sx in (-hx, hx)]
    for sy in (-hy, hy):
        X, Zb = np.meshgrid(xs, zs)
        faces.append(np.column_stack([X.ravel(), np.full(X.size, sy), Zb.ravel()]))
    for sz in (-hz, hz):
        X, Yb = np.meshgrid(xs, ys)
        faces.append(np.column_stack([X.ravel(), Yb.ravel(), np.full(X.size, sz)]))
    return np.vstack(faces)


def audit_cards(bq, t, labels, card_labels, holder_tm, holder_raw, frames,
                tol=0.0008):
    """Worst interpenetration per pair over `frames`:
       card -> holder   card surface points inside the CAD holder (RobustDepth)
       holder -> card   CAD holder vertices near the slot inside the card box
       card <-> card    surface points of one inside the other's box (exact)
    Returns {pair: {max_depth_mm, at_t, pass}}."""
    pts_local = card_surface_points()
    # closest points on the RAW CAD surface: subdivision does not move the
    # surface, but trimesh's closest-point search on the process=False
    # subdivided mesh returned points 0.15-0.2 mm away on the same floor
    # plane (measured: a card corner 2 um below the z = 2 mm floor read as
    # 0.145 mm deep), so the subdivided mesh only supplies holder vertices
    rd = RobustDepth(holder_raw, holder_raw)
    hv = np.asarray(holder_tm.vertices)
    lo, hi = np.array([SLOT_X[0] - 0.004, SLOT_Y[0] - 0.004, 0.0]), \
        np.array([SLOT_X[1] + 0.004, SLOT_Y[1] + 0.004, POST_TOP_Z + 0.001])
    hv = hv[((hv > lo) & (hv < hi)).all(axis=1)]
    idx = {l: labels.index(l) for l in card_labels}
    out = {}

    def upd(k, d, tt):
        w = out.get(k, (-1.0, None))
        if d > w[0]:
            out[k] = (d, tt)

    for f in frames:
        tt = float(t[f])
        world = {}
        for l in card_labels:
            c, R, _, _ = card_frame(bq[f, idx[l]])
            world[l] = pts_local @ R.T + c
            upd(f"{l}->holder", rd.depth(world[l]), tt)
            dh = point_in_box_depth(hv, bq[f, idx[l]])
            upd(f"holder->{l}", float(dh.max()) if len(dh) else -1.0, tt)
        for i, a in enumerate(card_labels):
            for b in card_labels[i + 1:]:
                da = point_in_box_depth(world[a], bq[f, idx[b]]).max()
                db = point_in_box_depth(world[b], bq[f, idx[a]]).max()
                upd(f"{a}<->{b}", float(max(da, db)), tt)
        gc.collect()
    return {k: {"max_depth_mm": round(v[0] * MM, 3), "at_t": v[1],
                "pass": bool(v[0] < tol)} for k, v in out.items()}


def in_slot(bq_row, tol=0.0003) -> dict:
    """Where a card's four bottom corners are relative to the slot."""
    bc = card_corners(bq_row, BOTTOM_CORNERS)
    x_ok = bool(((bc[:, 0] > SLOT_X[0] - tol) & (bc[:, 0] < SLOT_X[1] + tol)).all())
    y_ok = bool(((bc[:, 1] > SLOT_Y[0] - tol) & (bc[:, 1] < SLOT_Y[1] + tol)).all())
    low = float(card_corners(bq_row)[:, 2].min())
    return {"bottom_corners_inside_walls_x": x_ok, "bottom_corners_inside_end_walls_y": y_ok,
            "lowest_point_above_floor_mm": round((low - FLOOR_Z) * MM, 3),
            "bottom_x_range_mm": [round(float(bc[:, 0].min()) * MM, 2),
                                  round(float(bc[:, 0].max()) * MM, 2)],
            "bottom_y_range_mm": [round(float(bc[:, 1].min()) * MM, 2),
                                  round(float(bc[:, 1].max()) * MM, 2)]}
