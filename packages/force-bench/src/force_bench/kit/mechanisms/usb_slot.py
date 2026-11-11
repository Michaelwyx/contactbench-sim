"""USB slot: the printed shell, the keyed centre and one printed spring part whose two jaws clamp the stick.

GEOMETRY of the final printable CAD (``assets/kit/usb-slot``: mm, Z up, the assembly frame; :func:`measure`
re-derives every number below by ray casts):

  shell    52 x 28 x 22 mm, standing on the table with its MOUTH ON TOP: a 1 mm top wall (z 21..22) with a
           12.35 x 5.35 mm opening centred on the origin and a 0.2 mm entry round, i.e. 0.175 mm of play per side
           around the 12.0 x 5.0 mm plug.  Below it an open cavity holds the other two parts.  The stick goes in
           straight down (-z).
  centre   the keyed TONGUE (x +-4.72, y -1.535..-0.025, z 10..20.5) that runs in the plug's channel (0.33-0.37 mm
           of play), standing on a block whose top (z = 10) is the stop for the plug tip.  An upside-down stick
           lands its solid wall on the tongue 1.5 mm into the mouth.
  spring   one part; per side a JAW block (x .. 8.0, y +-5, z 10.5..20.5) whose inner face stands at |x| = 5.8 /
           5.6 / 5.4 mm for the 0.2 / 0.4 / 0.6 mm indent variants, against the plug's half-width of 6.0 mm: each
           jaw is pushed out by the indent.  The face has a curved lead-in at the top (it reaches |x| = 6.0 at
           z = 18.9 / 19.4 / 19.2) and a chamfer at the bottom.  Each jaw hangs on two serpentine flexures (upper
           and lower, mirror images about z ~ 15.5) of 0.6 mm PLA beams, 10 mm deep, from a frame at |x| >= 16.
  sticks   ``usb-stick-1..4``: the same 12 x 5 mm plug (a tube with the key channel, 0.5 mm rounds on the tip's
           narrow edges) on four grip bodies; sticks 1-3 have a shoulder 12 mm behind the tip, stick 4 is a flat
           12 x 5 mm bar with a key-ring loop and no shoulder.

SEAT.  In the assembly frame a stick whose plug tip rests on the centre block (z = 10) has its shoulder on the
shell's top face (z = 22), both at zero gap (sticks 1-3; stick 4 seats on its tip alone).  :func:`seat_pose` gives
that pose for any stick, upright or upside down.

MODEL.  Shell and centre are static and collide (fine SDF, sim.yaml).  The printed spring is not simulated as a
deforming body (docs/physics_notes.md rule 1): each jaw block -- the spring part cropped to 5 <= |x| <= 8.4 mm,
z >= 10.25 mm, i.e. the block with its real lead-in and 0.4 mm of its flexure stubs -- rides a prismatic joint along
x with the flexure's stiffness as drive and its CAD pose as target, and collides only with what enters the slot
(rule 3: it is filtered against the shell, the centre and the other jaw).  The flexures and the frame are drawn
and do not collide.  Joint travel -0.2 / +1.2 mm (inward / outward; rule 21c: not starting on a limit).

PARAMETERS per variant (``build(..., ke=, kd=)`` overrides), and what the probe measures (stick 1, 5 mm/s):

    indent   jaw face |x|   interference   ke [N/mm/jaw]   source           probe: jaw opening, force   peak / sliding
    0.2 mm   5.8 mm         0.2 mm/jaw     66              estimate (FEM)   0.195 mm, 12.9 N/jaw        12.4 / 10.4 N
    0.4 mm   5.6 mm         0.4 mm/jaw     66              estimate (FEM)   0.391 mm, 25.8 N/jaw        27.6 / 20.6 N
    0.6 mm   5.4 mm         0.6 mm/jaw     66              estimate (FEM)   0.586 mm, 38.7 N/jaw        43.6 / 30.6 N

(peak: the plug wedging the jaws open over their lead-ins, 3.5-5 mm in; sliding: along the jaws' flats, = 2 x mu
0.4 x the jaw force; the upside-down stick stops 1.50 mm in on the key at the 15 N cap.)

ke is an ESTIMATE; no force measurement of this slot exists.  The three variants share the flexure (they differ
only in the jaw face), so they share ke.  It comes from a plane-stress finite-element model of the flexure's
section (:func:`fem_flexure_stiffness`: the y = 0 section of the spring part, Q4 elements on a 0.0125 mm grid,
mesh-converged to 1 %, jaw block moved rigidly along x, frame clamped, 10 mm deep) with E = 3 GPa and nu = 0.35:
66 N/mm per jaw.  Plane strain (the beams are 17x wider than thick) gives 75 N/mm; E = 2 / 3.5 GPa (the range for
printed PLA) gives 44 / 77 N/mm, and a +-0.05 mm error in the printed beam thickness moves it by about +-25 %.
A beam-theory reading of the flexure as three ~8.4 mm beams in series (3-4 N/mm) does not match the CAD: the
straight segments are about 3 mm long, and the upper and lower serpentines act in parallel.  For scale, the
seated jaw forces above put the insertion friction of the three variants at roughly 8-11 / 16-21 / 24-32 N
(mu 0.3-0.4), the range of a real USB-A receptacle (<= 35 N).

The jaw mass (30 g; the printed block is 0.3 g) is a conditioning value: it keeps the drive's natural frequency
at omega*dt ~ 0.8 at 32 substeps.  The jaws are quasi-static here (the stick moves at a few mm/s).

CONTACT STIFFNESS.  The jaws load the stick through a contact, and that contact must be much stiffer than the
66 N/mm flexure or it, not the flexure, takes the indent.  Under SolverMuJoCo a contact's stiffness is capped by
the timestep (its solref time constant cannot go below 2 dt) and, under that cap, scales with the effective mass of
the bodies it acts on, so a gram-scale FREE stick sinks into the jaws instead of opening them.  Measured in
``appendix_tasks.plug_in_usb`` (stick 1, 0.2 mm spring, 32 substeps): at the stick's 1.5 g print estimate the jaws
opened 0.07 of the 0.2 mm (4.6 N instead of ~12 N) and the stick sank 0.13-0.16 mm into them; 64 substeps gave
0.13 mm of opening at twice the cost; a 15 g conditioning mass for the stick 0.16 mm; that mass plus
:func:`stiffen_jaw_contacts` (contact stiffness 1e9 instead of the scene's 1e7 N/m on the jaw shapes only)
0.17-0.19 mm and a 12.1 N lead-in peak -- the probe's numbers below.  A task calls :func:`stiffen_jaw_contacts`
after it builds its solver (``make_solver`` resets every shape's contact stiffness).

PROBE (``force-bench kit check usb-slot``): stick 1 on a probe joint (translation along the slot axis and
rotation about it, both driven), centred on the slot axis.  It is first turned UPSIDE DOWN and pushed into the
mouth with a force cap: it must stop on the key tongue (tip ~1.5 mm in).  Then it is withdrawn, turned upright,
and driven in at 5 mm/s to the seat; force against depth is written to ``probe_force.csv`` and the check passes
when the stick reaches the CAD seat (within 0.3 mm), the upside-down stick was blocked, and the peak force is
finite and below 150 N.  The jaw openings and drive forces at the seat are reported against the indent.
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.kit.build import _setting, pose_matrix, to_wp
from force_bench.kit.catalog import sdf_resolution

MM = 1e-3

# ── the model ─────────────────────────────────────────────────────────────────────────────────────────────────────
KE_JAW = 66000.0            # N/m per jaw: plane-stress FEM of the flexure, E = 3 GPa (estimate, see docstring)
KE_SOURCE = "estimate: plane-stress FEM of the printed flexure (E = 3 GPa, nu = 0.35); not measured"
KD_JAW = 60.0               # N.s/m, ~0.7 of critical for the conditioning mass
JAW_MASS = 0.030            # kg, conditioning value (printed block: 0.3 g)
JAW_TRAVEL = (-0.0002, 0.0012)   # m, inward / outward joint limits around the CAD pose
JAW_CROP_X = (0.0050, 0.0084)    # |x| range of the jaw collider: the block and 0.4 mm of its flexure stubs
JAW_CROP_Z = 0.01025        # jaw collider: z >= this (the base plate below is static scenery)
JAW_CONTACT_KE = 1.0e9      # N/m, contact stiffness of the jaw shapes (scene default 1e7; see CONTACT STIFFNESS)
#: contact stiffness per shape role once the solver is built (stiffen_jaw_contacts); the USD export authors it
SOLVER_SHAPE_KE = {"jaw_right": JAW_CONTACT_KE, "jaw_left": JAW_CONTACT_KE}

# ── geometry (assembly frame, m; :func:`measure` re-derives them) ─────────────────────────────────────────────────
MOUTH_Z = 0.022             # shell top face, the stick's shoulder seat
SEAT_TIP_Z = 0.010          # centre block top, the plug-tip stop
KEY_TOP_Z = 0.0205          # top of the key tongue
PLUG_LEN = 0.012            # tip to shoulder (sticks 1-3)
PLUG_HALF_W = 0.0060        # plug half-width (slot x)
# stick CAD frame -> slot assembly frame: plug (+x in the stick files) points DOWN, the stick's 12 mm axis (y)
# along the slot's x, its 5 mm axis (z, the channel on the +z side) along slot -y, i.e. over the key tongue
R_STICK = np.array([[0.0, 1.0, 0.0],
                    [0.0, 0.0, -1.0],
                    [-1.0, 0.0, 0.0]])
R_FLIP = np.diag([-1.0, -1.0, 1.0])          # upside down: half a turn about the slot axis

# ── the slot body, for QA (assembly frame, m) ─────────────────────────────────────────────────────────────────────
MOUTH_HALF = (0.006175, 0.002675)           # the mouth's half extents (x, y): 0.175 mm of play around the plug
SHELL_BOX = ((-0.026, -0.014, 0.0), (0.026, 0.014, MOUTH_Z))     # the shell's outside
SHELL_ROUND = 0.0006                        # its 2 mm edge rounds leave the box by up to 0.59 mm


def intrusion(p: np.ndarray) -> np.ndarray:
    """Depth [m] of points (N x 3, assembly frame) inside the slot body: inside the shell's outside box (deeper than
    its edge rounds) and outside the plug's channel (the mouth extruded down to the centre block), the distance a
    point must travel to leave the box or to reach the channel; 0 elsewhere.  A signed distance into a 1 mm wall stops
    growing at 0.5 mm; this keeps growing once a body is through the wall, into the cavity behind it."""
    p = np.asarray(p, float)
    lo, hi = np.asarray(SHELL_BOX[0]), np.asarray(SHELL_BOX[1])
    d_out = np.minimum(p - lo, hi - p).min(axis=1)
    c_lo = np.array([-MOUTH_HALF[0], -MOUTH_HALF[1], SEAT_TIP_Z])
    c_hi = np.array([MOUTH_HALF[0], MOUTH_HALF[1], MOUTH_Z + 0.001])
    in_ch = np.all((p >= c_lo) & (p <= c_hi), axis=1)
    d_ch = np.linalg.norm(np.maximum(np.maximum(c_lo - p, p - c_hi), 0.0), axis=1)
    return np.where((d_out > SHELL_ROUND) & ~in_ch, np.minimum(d_out, d_ch), 0.0)


def indent_mm(label: str | None) -> float:
    return float(label.split("mm")[0]) if label else 0.2


def spring_params(indent: float) -> dict:
    """The jaw model of a spring variant (indent in mm): ke [N/m], interference [m], seated jaw force [N]."""
    return {"indent_mm": indent, "ke_N_per_m": KE_JAW, "interference_mm": indent,
            "jaw_force_seated_N": round(KE_JAW * indent * MM, 2), "source": KE_SOURCE}


# ── poses of a stick in the slot ──────────────────────────────────────────────────────────────────────────────────
def plug_reference(stick_mesh: trimesh.Trimesh) -> tuple[float, float, float]:
    """(x_tip, y_c, z_c) in the stick's CAD frame (m): the plug tip and the centre of the plug's outer section."""
    x_tip = float(stick_mesh.bounds[1][0])
    sec = stick_mesh.section(plane_origin=[x_tip - 0.003, 0.0, 0.0], plane_normal=[1.0, 0.0, 0.0])
    v = np.asarray(sec.vertices)
    return x_tip, 0.5 * float(v[:, 1].min() + v[:, 1].max()), 0.5 * float(v[:, 2].min() + v[:, 2].max())


def seat_pose(stick_key: str, slot_pose: np.ndarray | None = None, flipped: bool = False,
              lift: float = 0.0) -> np.ndarray:
    """World pose (4x4) of a stick's CAD frame, fully inserted (plug tip on the centre block, plug centred in the
    mouth, channel over the key), raised by ``lift`` along the slot axis.  ``flipped``: half a turn about the
    slot axis (the channel away from the key; such a stick cannot go in)."""
    from force_bench.kit import load_kit
    st = load_kit()[stick_key].parts()[0].mesh()
    x_tip, y_c, z_c = plug_reference(st)
    T = np.eye(4)
    T[:3, :3] = R_STICK
    T[:3, 3] = -R_STICK @ np.array([x_tip, y_c, z_c]) + np.array([0.0, 0.0, SEAT_TIP_Z + lift])
    if flipped:
        F = np.eye(4)
        F[:3, :3] = R_FLIP
        T = F @ T
    return (np.eye(4) if slot_pose is None else np.asarray(slot_pose, float)) @ T


# ── watertight crops (no triangulation engine for trimesh's cap is installed; shapely's CDT closes the cut) ────────
def _slice_capped(m: trimesh.Trimesh, origin, normal) -> trimesh.Trimesh:
    """Keep the half-space (p - origin).normal >= 0 of a watertight mesh and close the cut with a constrained
    Delaunay cap on the kept mesh's own boundary vertices (no T-junctions)."""
    import shapely
    from scipy.spatial import cKDTree
    from shapely.geometry import Polygon
    from trimesh import intersections
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
    depth = [sum(1 for j, q in enumerate(polys) if j != i and q.contains(p)) for i, p in enumerate(polys)]
    idx = np.unique(np.concatenate(loops))
    tree = cKDTree(P2[idx])
    faces = []
    for si, d in enumerate(depth):
        if d % 2:
            continue
        holes = [polys[j].exterior.coords for j, dj in enumerate(depth)
                 if dj == d + 1 and polys[si].contains(polys[j])]
        for g in shapely.constrained_delaunay_triangles(Polygon(polys[si].exterior.coords, holes)).geoms:
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


def _crop_box(m: trimesh.Trimesh, lo, hi) -> trimesh.Trimesh:
    """Watertight intersection of a watertight mesh with an axis box (None = unbounded)."""
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
            out = _slice_capped(out, o, nrm)
    assert out.is_watertight, "crop is not watertight"
    return out


def _open_slice(m: trimesh.Trimesh, keep) -> trimesh.Trimesh:
    """Uncapped slices (for drawing only): keep = [(origin, normal), ...] half-spaces."""
    from trimesh import intersections
    out = m
    for o, n in keep:
        out = intersections.slice_mesh_plane(out, np.asarray(n, float), np.asarray(o, float), cap=False)
    return out


def split_spring(spring: trimesh.Trimesh) -> dict[str, trimesh.Trimesh]:
    """The spring part (assembly frame, m) as {jaw_right, jaw_left (watertight colliders), frame (drawn)}."""
    x0, x1 = JAW_CROP_X
    jaw_r = _crop_box(spring, (x0, None, JAW_CROP_Z), (x1, None, None))
    jaw_l = _crop_box(spring, (-x1, None, JAW_CROP_Z), (-x0, None, None))
    right = _open_slice(spring, [((x1, 0, 0), (1, 0, 0))])
    left = _open_slice(spring, [((-x1, 0, 0), (-1, 0, 0))])
    base = _open_slice(spring, [((-x1, 0, 0), (1, 0, 0)), ((x1, 0, 0), (-1, 0, 0)), ((0, 0, JAW_CROP_Z), (0, 0, -1))])
    return {"jaw_right": jaw_r, "jaw_left": jaw_l, "frame": trimesh.util.concatenate([right, left, base])}


# ── build ─────────────────────────────────────────────────────────────────────────────────────────────────────────
def _shape(rig, body: int, asset, part_name: str, tm: trimesh.Trimesh, body_from_cad: np.ndarray, label: str,
           rgb, mu: float, collide: bool = True) -> int:
    """A mesh shape with the asset's (part's) contact settings from sim.yaml, labelled ``label``."""
    cell = float(_setting(asset, part_name, "sdf_cell_mm", 0.4)) * MM
    res = _setting(asset, part_name, "sdf_res") or sdf_resolution(
        tm, cell=cell, lo=int(_setting(asset, part_name, "sdf_min_res", 64)),
        hi=int(_setting(asset, part_name, "sdf_max_res", 384)))
    nm, c = make_newton_mesh(tm, sdf=collide, sdf_resolution=int(res),
                             narrow_band=float(_setting(asset, part_name, "narrow_band_mm", 2.0)) * MM,
                             margin=float(_setting(asset, part_name, "margin_mm", 1.2)) * MM)
    cfg = rig.cfg(mu=mu)
    gap = _setting(asset, part_name, "gap_mm")
    if gap is not None:
        cfg.gap = float(gap) * MM
    if not collide:
        cfg.has_shape_collision = False
        cfg.has_particle_collision = False
        cfg.density = 0.0
    sid = rig.builder.add_shape_mesh(body, xform=to_wp(body_from_cad @ pose_matrix(xyz=c)), mesh=nm, cfg=cfg,
                                     color=rgb, label=label)
    return sid


def build(rig, asset, h, ke: float | None = None, kd: float = KD_JAW, mu: float | None = None) -> None:
    from force_bench.kit.build import add_part_shape
    indent = indent_mm(h.choice.get("Insertion force"))
    p = spring_params(indent)
    if ke is not None:
        p["ke_N_per_m"], p["source"] = float(ke), "override"
        p["jaw_force_seated_N"] = round(float(ke) * indent * MM, 2)
    ke = float(p["ke_N_per_m"])
    mu = float(_setting(asset, None, "mu", 0.5)) if mu is None else float(mu)
    p.update({"kd": kd, "mu": mu, "jaw_mass_kg": JAW_MASS, "jaw_travel_mm": [v * 1e3 for v in JAW_TRAVEL]})
    h.params = p
    pose = np.asarray(h.pose, float)
    statics = []
    for name in ("usb_slot_shell", "usb_slot_center"):
        part = asset.part(name, h.choice)
        sid = add_part_shape(rig, -1, asset, part, pose, h.label, mu=mu)
        h.shapes[name] = [sid]
        statics.append(sid)
    spring = next(pt for pt in asset.parts(h.choice) if pt.name.startswith("usb_slot_spring"))
    pieces = split_spring(spring.mesh())
    sid = _shape(rig, -1, asset, spring.name, pieces["frame"], pose, f"kit/{h.label}/spring_flexures",
                 spring.rgb, mu, collide=False)
    h.shapes["spring_flexures"] = [sid]
    rig.shapes.setdefault(h.label, []).append(sid)
    jaw_sids = []
    for name, side in (("jaw_right", 1.0), ("jaw_left", -1.0)):
        tm = pieces[name]
        c = 0.5 * (tm.bounds[0] + tm.bounds[1])
        world_from_body = pose @ pose_matrix(xyz=c)
        body = rig.builder.add_link(xform=to_wp(world_from_body), label=f"{h.label}/{name}")
        s = _shape(rig, body, asset, spring.name, tm, pose_matrix(xyz=-c), f"kit/{h.label}/{name}", spring.rgb, mu)
        rig.shapes.setdefault(h.label, []).append(s)
        # axis in the joint's anchor frame (= the body frame at rest, rotated with the asset): outward along x
        j = rig.builder.add_joint_prismatic(parent=-1, child=body, axis=(side, 0.0, 0.0),
                                            parent_xform=to_wp(world_from_body), child_xform=wp.transform_identity(),
                                            target_pos=0.0, target_ke=ke, target_kd=float(kd),
                                            limit_lower=JAW_TRAVEL[0], limit_upper=JAW_TRAVEL[1],
                                            label=f"{h.label}/{name}")
        rig.builder.add_articulation([j], label=f"{h.label}/{name}")
        set_body_mass(rig.builder, body, JAW_MASS)
        h.bodies[name], h.joints[name], h.shapes[name] = body, j, [s]
        jaw_sids.append(s)
    for s in jaw_sids:                      # the joint is the flexure's guide: jaws meet only what enters the slot
        for t in statics:
            rig.builder.add_shape_collision_filter_pair(s, t)
    rig.builder.add_shape_collision_filter_pair(jaw_sids[0], jaw_sids[1])
    jr = pieces["jaw_right"]
    h.info = {"jaw_face_x_mm": round(_jaw_face(jr) * 1e3, 3),
              "jaw_crop_mm": {"x": [v * 1e3 for v in JAW_CROP_X], "z_min": JAW_CROP_Z * 1e3},
              "jaw_mass_cad_g": round(float(jr.volume) * 1240.0 * 1e3, 3),
              "mouth_z_mm": MOUTH_Z * 1e3, "seat_tip_z_mm": SEAT_TIP_Z * 1e3, "key_top_z_mm": KEY_TOP_Z * 1e3}


def stiffen_jaw_contacts(rig, h, solver=None, ke: float = JAW_CONTACT_KE) -> None:
    """Make the jaws' contacts stiffer than the flexure they load (module docstring, CONTACT STIFFNESS).  Call it
    after the solver is built -- ``make_solver`` resets every shape's contact stiffness -- with that solver
    (default ``rig.solver``)."""
    sids = [h.shapes[n][0] for n in ("jaw_right", "jaw_left")]
    ke_arr = rig.model.shape_material_ke.numpy()
    ke_arr[sids] = float(ke)
    rig.model.shape_material_ke.assign(ke_arr)
    (rig.solver if solver is None else solver).notify_model_changed(newton.ModelFlags.SHAPE_PROPERTIES)
    h.params["jaw_contact_ke_N_per_m"] = float(ke)


def _ray(m: trimesh.Trimesh, o, d) -> float | None:
    loc, _, _ = m.ray.intersects_location([o], [d])
    if len(loc) == 0:
        return None
    return float(np.min(np.linalg.norm(loc - np.asarray(o, float), axis=1)))


def _jaw_face(jaw_right: trimesh.Trimesh) -> float:
    """|x| of the right jaw's flat face at mid-height (m)."""
    return float(_ray(jaw_right, [0.0, 0.0, 0.015], [1.0, 0.0, 0.0]))


def measure() -> dict:
    """The numbers of the module docstring, from ray casts on the final CAD (mm)."""
    from force_bench.kit import load_kit
    kit = load_kit()
    a = kit["usb-slot"]
    sh = a.part("usb_slot_shell").mesh()
    ce = a.part("usb_slot_center").mesh()
    out = {"mouth_half_x_mm": _ray(sh, [0, 0, 0.0215], [1, 0, 0]) * 1e3,
           "mouth_half_y_mm": _ray(sh, [0, 0, 0.0215], [0, 1, 0]) * 1e3,
           "top_wall_underside_z_mm": (0.015 + _ray(sh, [0.007, 0, 0.015], [0, 0, 1])) * 1e3,
           "key_top_z_mm": (0.025 - _ray(ce, [0, -0.0008, 0.025], [0, 0, -1])) * 1e3,
           "tip_stop_z_mm": (0.025 - _ray(ce, [0, 0.002, 0.025], [0, 0, -1])) * 1e3,
           "jaws": {}, "sticks": {}}
    for v in a.groups[0].variants:
        spring = v.parts[0].mesh()
        face = _ray(spring, [0, 0, 0.015], [1, 0, 0])
        lead = next(z for z in np.arange(0.0205, 0.015, -0.0001) if _ray(spring, [0, 0, z], [1, 0, 0]) <= 0.0060)
        out["jaws"][v.label] = {"face_x_mm": round(face * 1e3, 3),
                                "interference_mm": round((PLUG_HALF_W - face) * 1e3, 3),
                                "lead_in_reaches_plug_at_z_mm": round(float(lead) * 1e3, 2)}
    for k in ("usb-stick-1", "usb-stick-2", "usb-stick-3", "usb-stick-4"):
        st = kit[k].parts()[0].mesh()
        x_tip, y_c, z_c = plug_reference(st)
        out["sticks"][k] = {"plug_tip_x_mm": round(x_tip * 1e3, 3), "plug_center_yz_mm": [round(y_c * 1e3, 3),
                                                                                         round(z_c * 1e3, 3)],
                            "extent_mm": [round(float(e) * 1e3, 2) for e in st.extents]}
    out["mouth_play_per_side_mm"] = round(out["mouth_half_x_mm"] - PLUG_HALF_W * 1e3, 3)
    return out


# ── FEM estimate of the flexure stiffness ─────────────────────────────────────────────────────────────────────────
def fem_flexure_stiffness(indent: str = "0.4", h_mm: float = 0.025, E: float = 3.0e9, nu: float = 0.35,
                          plane: str = "stress", depth_mm: float = 10.0) -> float:
    """Stiffness [N/m] of one jaw's flexure: 2D finite elements (bilinear quads on an h_mm raster) of the spring
    part's y = 0 section on the right side (x 5..16.6 mm, z 10.3..20.7 mm), the jaw block (x <= 8 mm) moved
    rigidly along x, the frame (x >= 16 mm) clamped, times the 10 mm extrusion depth.  The section is the same
    over the whole depth (checked at |y| <= 4.5 mm)."""
    import scipy.sparse as sp
    import scipy.sparse.linalg as spla
    import shapely
    from shapely.geometry import box
    from shapely.ops import unary_union
    from force_bench.kit import load_kit
    a = load_kit()["usb-slot"]
    m = a.part(f"usb_slot_spring_{indent}mm_indent", f"{indent}mm").mesh()
    m.apply_scale(1e3)                                   # mm
    sec = m.section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
    T = np.eye(4)
    T[:3, :3] = np.array([[1.0, 0, 0], [0, 0, 1.0], [0, 1.0, 0]])     # (x, z) -> plane
    p2, _ = sec.to_2D(to_2D=T)
    x0, z0, x1, z1 = 5.0, 10.3, 16.6, 20.7
    reg = unary_union(list(p2.polygons_full)).intersection(box(x0, z0, x1, z1))
    h = float(h_mm)
    nx, nz = int(round((x1 - x0) / h)), int(round((z1 - z0) / h))
    XC, ZC = np.meshgrid(x0 + (np.arange(nx) + 0.5) * h, z0 + (np.arange(nz) + 0.5) * h, indexing="ij")
    inside = shapely.contains_xy(reg, XC.ravel(), ZC.ravel()).reshape(nx, nz)
    Emm = E * 1e-6                                       # N/mm^2
    if plane == "stress":
        D = Emm / (1 - nu ** 2) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
    else:
        Ep, nup = Emm / (1 - nu ** 2), nu / (1 - nu)
        D = Ep / (1 - nup ** 2) * np.array([[1, nup, 0], [nup, 1, 0], [0, 0, (1 - nup) / 2]])
    g = 1 / np.sqrt(3)
    Ke = np.zeros((8, 8))
    for xi in (-g, g):
        for eta in (-g, g):
            dx = 0.25 * np.array([-(1 - eta), (1 - eta), (1 + eta), -(1 + eta)]) / (h / 2)
            dz = 0.25 * np.array([-(1 - xi), -(1 + xi), (1 + xi), (1 - xi)]) / (h / 2)
            B = np.zeros((3, 8))
            B[0, 0::2], B[1, 1::2], B[2, 0::2], B[2, 1::2] = dx, dz, dz, dx
            Ke += B.T @ D @ B * (h / 2) ** 2
    ei, ej = np.nonzero(inside)
    nid = lambda i, j: i * (nz + 1) + j                  # noqa: E731
    conn = np.stack([nid(ei, ej), nid(ei + 1, ej), nid(ei + 1, ej + 1), nid(ei, ej + 1)], 1)
    used = np.unique(conn)
    remap = -np.ones((nx + 1) * (nz + 1), int)
    remap[used] = np.arange(len(used))
    c = remap[conn]
    dofs = np.stack([2 * c, 2 * c + 1], 2).reshape(-1, 8)
    K = sp.coo_matrix((np.tile(Ke.ravel(), len(dofs)), (np.repeat(dofs, 8, 1).ravel(), np.tile(dofs, (1, 8)).ravel())),
                      shape=(2 * len(used), 2 * len(used))).tocsr()
    X = x0 + (used // (nz + 1)) * h
    jaw, frame = X <= 8.0 + 1e-9, X >= 16.0 - 1e-9
    delta = 0.01
    u = np.zeros(2 * len(used))
    fixed = np.zeros(2 * len(used), bool)
    for sel, dxv in ((jaw, delta), (frame, 0.0)):
        i = np.nonzero(sel)[0]
        fixed[2 * i] = fixed[2 * i + 1] = True
        u[2 * i] = dxv
    free = ~fixed
    u[free] = spla.spsolve(K[free][:, free].tocsc(), -K[free][:, fixed] @ u[fixed])
    F = (K @ u)[2 * np.nonzero(jaw)[0]].sum() * depth_mm             # N
    return float(F / delta) * 1e3                                        # N/m


# ── probe: stick 1, upside down against the key, then upright to the seat ───────────────────────────────────────
PROBE_STICK = "usb-stick-1"
PROBE_MASS = 0.06           # kg (the outlet probe's value)
PROBE_KE, PROBE_KD = 2.0e5, 300.0           # N/m, N.s/m: 40 N of insertion force lags the target by 0.2 mm
PROBE_EFFORT = 80.0         # N, drive clamp (bounds the seat push)
PROBE_ANG_KE, PROBE_ANG_KD = 2.0, 0.004     # N.m/rad, N.m.s/rad
START_GAP = 0.002           # m, plug tip above the mouth at the start
SPEED = 0.005               # m/s, insertion speed
OVERTRAVEL = 0.0005         # m, target past the seat
F_BLOCK = 15.0              # N, force cap of the upside-down push
T_TURN = 0.5                # s, half a turn
SEAT_TOL = 0.0003           # m, seated = within this of the CAD seat
PEAK_MAX = 150.0            # N


def add_probe(rig, h) -> None:
    from force_bench.kit import load_kit
    from force_bench.kit.build import add_part_shape
    kit = load_kit()
    sa = kit[PROBE_STICK]
    part = sa.parts()[0]
    st = part.mesh()
    lift = (MOUTH_Z - SEAT_TIP_Z) + START_GAP          # tip START_GAP above the mouth
    T_cad = seat_pose(PROBE_STICK, h.pose, lift=lift)  # world <- stick CAD frame, upright
    c_cad = 0.5 * (st.bounds[0] + st.bounds[1])
    world_from_body = T_cad @ pose_matrix(xyz=c_cad)
    body = rig.builder.add_link(xform=to_wp(world_from_body), label="probe_stick")
    add_part_shape(rig, body, sa, part, np.linalg.inv(pose_matrix(xyz=c_cad)), "probe_stick")
    set_body_mass(rig.builder, body, PROBE_MASS)
    # the joint anchor: on the slot axis at the body's height, axes along the slot axis (world-aligned anchor)
    axis_w = h.pose[:3, :3] @ np.array([0.0, 0.0, -1.0])
    p_body = world_from_body[:3, 3]
    p_axis = h.pose[:3, 3] + h.pose[:3, :3] @ np.array([0.0, 0.0, (np.linalg.inv(h.pose) @ np.r_[p_body, 1.0])[2]])
    anchor = pose_matrix(xyz=p_axis)
    child = np.linalg.inv(world_from_body) @ anchor
    seat_s = lift                                      # joint travel from the start to the seat
    JD = newton.ModelBuilder.JointDofConfig
    j = rig.builder.add_joint_d6(
        parent=-1, child=body,
        linear_axes=[JD(axis=tuple(axis_w), target_ke=PROBE_KE, target_kd=PROBE_KD, effort_limit=PROBE_EFFORT,
                        limit_lower=-0.005, limit_upper=seat_s + 0.003)],
        angular_axes=[JD(axis=tuple(axis_w), target_ke=PROBE_ANG_KE, target_kd=PROBE_ANG_KD,
                         limit_lower=-0.5, limit_upper=np.pi + 0.5)],
        parent_xform=to_wp(anchor), child_xform=to_wp(child), label="probe_stick")
    rig.builder.add_articulation([j], label="probe_stick")
    h.bodies["probe_stick"] = body
    h.info["probe"] = {"stick": PROBE_STICK, "joint": j, "body": body, "seat_s_mm": round(seat_s * 1e3, 3),
                       "start_gap_mm": START_GAP * 1e3}
    h.info["probe_bodies"] = ["probe_stick"]


def run_probe(rig, h) -> dict:
    from force_bench.rig import make_solver
    pr = h.info["probe"]
    j, seat_s = pr["joint"], pr["seat_s_mm"] * MM
    probe = rig.probes["probe_stick"]
    t_start = rig.model.joint_target_q_start.numpy()
    q_start = rig.model.joint_q_start.numpy()
    i_lin, i_ang = int(t_start[j]), int(t_start[j]) + 1
    q_lin, q_ang = int(q_start[j]), int(q_start[j]) + 1
    jaw_q = [int(q_start[h.joints[n]]) for n in ("jaw_right", "jaw_left")]
    jaw_ke = float(h.params["ke_N_per_m"])
    up = h.pose[:3, 2]
    inner = make_solver(rig.model, iterations=24)
    stiffen_jaw_contacts(rig, h, inner)
    st = {"phase": "turn", "t0": 0.0, "s_tgt": 0.0, "a_tgt": 0.0, "F": 0.0, "s": 0.0, "block": None,
          "seat_t": None}

    class Ramp:                      # targets move every SUBSTEP (a per-frame step kicks the drive)
        def __init__(self):
            self.t = 0.0

        def step(self, s_in, s_out, control, contacts, dt):
            self.t += dt
            t = self.t
            ph = st["phase"]
            if ph == "turn":                                   # half a turn above the mouth
                u = min(1.0, t / T_TURN)
                st["a_tgt"] = np.pi * (3 * u * u - 2 * u ** 3)
            elif ph == "flip_in":                              # upside down: push until the force cap
                if st["F"] < F_BLOCK:
                    st["s_tgt"] = min(st["s_tgt"] + SPEED * dt, START_GAP + 0.004)
            elif ph == "back":                                 # withdraw, then turn upright
                st["s_tgt"] = max(0.0, st["s_tgt"] - 2 * SPEED * dt)
                if st["s_tgt"] <= 0.0:
                    st["a_tgt"] = max(0.0, st["a_tgt"] - np.pi * dt / T_TURN)
            elif ph == "insert":                               # upright, constant speed to the seat
                st["s_tgt"] = min(st["s_tgt"] + SPEED * dt, seat_s + OVERTRAVEL)
            qt = control.joint_target_q.numpy()
            qt[i_lin], qt[i_ang] = st["s_tgt"], st["a_tgt"]
            control.joint_target_q.assign(qt)
            return inner.step(s_in, s_out, control, contacts, dt)

        def __getattr__(self, n):
            return getattr(inner, n)

    rows = []

    def extra(state, t):
        r = probe.sample(rig.contacts, state, t)
        q = state.joint_q.numpy()
        s = float(q[q_lin])
        F = float(np.dot(r["vec"], up))                       # resistance along the slot axis (up = +)
        st["F"], st["s"] = F, s
        jo = [float(q[i]) for i in jaw_q]
        ph = st["phase"]
        rows.append((t, ["turn", "flip_in", "flip_hold", "back", "insert", "hold"].index(ph),
                     (s - START_GAP) * 1e3, F, float(q[q_ang]), jo[0] * 1e3, jo[1] * 1e3))
        if ph == "turn" and t >= T_TURN + 0.1:
            st["phase"], st["t0"] = "flip_in", t
        elif ph == "flip_in" and (F >= F_BLOCK or st["s_tgt"] >= START_GAP + 0.004 - 1e-9):
            st["phase"], st["t0"], st["block"] = "flip_hold", t, (s, F)
        elif ph == "flip_hold" and t >= st["t0"] + 0.2:
            st["block"] = (max(st["block"][0], s), max(st["block"][1], F))
            st["phase"], st["t0"] = "back", t
        elif ph == "back" and st["s_tgt"] <= 0.0 and st["a_tgt"] <= 0.0 and abs(float(q[q_ang])) < 0.01:
            st["phase"], st["t0"] = "insert", t
        elif ph == "insert" and st["s_tgt"] >= seat_s + OVERTRAVEL - 1e-9:
            st["phase"], st["t0"] = "hold", t
        return {}

    saved = rig.solver
    rig.solver = Ramp()
    budget = T_TURN + 0.1 + (0.004 + START_GAP) / SPEED + 0.2 + (0.004 + START_GAP) / (2 * SPEED) + T_TURN + 0.2 \
        + (seat_s + OVERTRAVEL) / SPEED + 0.5
    rig.run(budget, lambda t: None, extra, print_every=10 ** 9)
    rig.solver = saved
    stiffen_jaw_contacts(rig, h, saved)            # the check's settle runs on the scene's own solver
    a = np.array(rows)
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "probe_force.csv", a, delimiter=",", comments="",
               header="t,phase(0 turn 1 flip_in 2 flip_hold 3 back 4 insert 5 hold),tip_depth_mm,force_N,"
                      "turn_rad,jaw_right_open_mm,jaw_left_open_mm")
    seat_depth = (seat_s - START_GAP) * 1e3                    # tip depth below the mouth at the CAD seat
    ins = a[a[:, 1] == 4]
    hold = a[a[:, 1] == 5]
    end = hold[-max(1, len(hold) // 3):] if len(hold) else a[-5:]
    depth_end = float(np.median(end[:, 2]))
    jaw_open = [float(np.median(end[:, 5])), float(np.median(end[:, 6]))]
    blocked_depth = (st["block"][0] - START_GAP) * 1e3 if st["block"] else float("nan")
    blocked_F = st["block"][1] if st["block"] else 0.0
    key_depth = (MOUTH_Z - KEY_TOP_Z) * 1e3
    pre_seat = ins[ins[:, 2] < seat_depth - 0.5] if len(ins) else ins
    peak = float(np.max(pre_seat[:, 3])) if len(pre_seat) else float("nan")
    flat = ins[(ins[:, 2] > 6.0) & (ins[:, 2] < seat_depth - 1.0)] if len(ins) else ins
    seated = abs(depth_end - seat_depth) <= SEAT_TOL * 1e3
    blocked = bool(blocked_F >= 0.9 * F_BLOCK and blocked_depth <= key_depth + 0.5)
    ok = bool(np.isfinite(a).all() and seated and blocked and peak < PEAK_MAX)
    indent = float(h.params["indent_mm"])
    res = {"pass": ok, "stick": pr["stick"],
           "seat_tip_depth_mm": round(seat_depth, 3), "tip_depth_end_mm": round(depth_end, 3),
           "seated": bool(seated), "seat_tolerance_mm": SEAT_TOL * 1e3,
           "peak_insertion_force_N": round(peak, 2),
           "sliding_force_N": round(float(np.mean(flat[:, 3])), 2) if len(flat) else None,
           "force_at_depth_N": {f"{d:g}mm": round(float(np.interp(d, ins[:, 2], ins[:, 3])), 2)
                                for d in (1, 2, 4, 6, 8, 10, 11) if len(ins) and d < ins[:, 2].max()},
           "jaw_open_seated_mm": [round(v, 3) for v in jaw_open], "indent_mm": indent,
           "jaw_drive_force_seated_N": [round(jaw_ke * v * MM, 2) for v in jaw_open],
           "upside_down": {"blocked": blocked, "tip_depth_mm": round(blocked_depth, 3),
                           "key_top_depth_mm": round(key_depth, 3), "force_N": round(blocked_F, 2),
                           "force_cap_N": F_BLOCK},
           "speed_mm_s": SPEED * 1e3}
    return res
