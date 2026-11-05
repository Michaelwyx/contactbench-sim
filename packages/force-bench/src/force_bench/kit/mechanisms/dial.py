"""Dials: the three printed dials of the kit, their printed springs replaced by sprung contact bodies.

All three share the housing (``*_bottom``, ``*_top``, the inlaid ``*_grid``), a knob (flat wing bar or round grip,
the "Knob geometry" variant; the mechanism does not depend on it) and an arrow inlay on the knob.  The housing is
static and the knob turns about the housing axis.  Angles: 0 at the CAD pose (the arrow on numeral "0"), positive
counter-clockwise seen from above (numerals 1, 2, 3 ... lie clockwise, 60 deg apart; the standard and push-down
grids add minor ticks every 15 deg).  What differs is the printed spring under the knob, read off the CAD:

**Standard continuous dial** (``dial``).  ``dial_spring`` is a ring clamped between bottom and top carrying six
rigid pads around the knob's ROUND hub (r 16.00 mm), each pad hung on two folded serpentine flexures (0.8 mm beams,
10 mm deep).  The pad faces form a hexagon of inscribed radius 15.25 mm, so the hub pushes every pad out 0.75 mm:
a six-shoe FRICTION BRAKE that holds the knob at any angle.  Model: the knob on a revolute joint; each pad a rigid
prism (its CAD section) on a radial prismatic joint whose spring drive rests at the pad's CAD position, so the
hub's interference sets the normal load and contact friction sets the torque.

**Snap-to-position dial** (``dial-snap-to-position``).  The same spring on a ROUNDED-HEXAGON hub (flats 15.50 mm,
corners 16.43 mm, corner radius 9.5 mm), flats on the pads at the CAD pose: seated, each pad is out 0.25 mm; a
30 deg turn lifts all six to 1.18 mm, so the torque peaks and falls every 60 deg -- six DETENTS, one per numeral.
Same model; the detents come from the CAD hub against the CAD pads through contact.  With PLA-on-PLA friction the
detents do not self-centre: a pad's normal force has a 0-3.5 mm lever about the axis, its friction a 15.5-16.4 mm
one, so friction wins wherever mu > ~0.2 (the probe below measures it).

**Push-down-to-turn dial** (``dial-push-down-to-turn``).  Two D-shaped accordion springs (``dial_push_spring_1/2``:
base plate, three 0.8 mm leaves joined by walls, top plate with a half-ring bump r 18.1-21.9 mm) push the knob UP;
at the CAD pose the bump overlaps the knob 0.5 mm and the knob's face sits 0.5 mm under the cover, i.e. the knob
rests against the cover with no preload.  The cover carries a key tooth (180 deg; r 27.9-32.8 mm, +-10.0 deg) and a
peg (0 deg; r 22.8-27.4 mm, +-10.0 deg) reaching 3.0 mm below the knob's face; the knob has the matching rim notch
(+-10.9 deg, floor r 27.4 mm) and window (r 22.5-27.9 mm, +-11.1 deg).  Up, the knob is LOCKED (0.9 deg of play);
pressed 3.0 mm it turns (0.5 mm of clearance at the 3.5 mm stop, where the knob's bore bottoms on the centre post);
let go away from 0 deg, the springs hold the knob's face against the tooth and peg, so it STAYS DOWN at 3.0 mm and
keeps its angle by friction; turned back to 0 deg it POPS UP and locks.  Model: the knob on a joint with a vertical
slide (0 = up against the cover, stop at -3.5 mm) and a free rotation; each spring's top plate a body on a vertical
prismatic joint (spring drive resting at the free height), carrying the bump as an annular contact sector; tooth
and peg are static annular sectors with the CAD's radii, angles and depth, meeting the knob's real mesh (which has
the notch, the window and the face they ride on).  The rest of each spring is drawn and does not collide; the top
plate's drawing moves with its body.

Contacts below contact resolution are filtered and carried by the joints instead: the knob against the housing (CAD
running fits of 0-0.3 mm at the hub, the bore, the rim and the cover lip), pads and plates against the housing.  The
knob touches only the pads (standard, snap), or the plates, tooth and peg (push), and the robot.

Parameters (``build(..., **params)`` overrides each; ``handle.params`` records what was built; ``handle.info`` the
CAD measurements it was built from):

    parameter         value              provenance
    k_pad             29.0 kN/m per pad  ESTIMATE from the CAD and PLA's modulus (E 3.0 GPa, nu 0.35; E 2.5-3.5 GPa
                                         gives 24-34 kN/m).  ``beam_estimates()``: a 2D Euler-Bernoulli frame of the
                                         serpentine centrelines (stem, 3.0 / 8.0 / 3.0 mm beams 0.8 mm wide and 10 mm
                                         deep, two 1 mm U-bends, stem; pad guided by its mirror serpentine; plate
                                         bending E' = E / (1 - nu^2)) gives 30.2; ``pad_fem()``, a plane-strain FEM
                                         of the CAD section itself (Q4 pixels, 0.1 / 0.05 mm), gives 28.7 / 29.0, the
                                         value used.  Peak bending stress (frame) 40 MPa at the standard dial's
                                         0.75 mm and 62 MPa at the snap dial's 1.18 mm, against ~50-60 MPa for
                                         printed PLA: a real print relaxes, and will hold and snap less.
    k_push            3.94 kN/m per half ESTIMATE, ``beam_estimates()``: a frame of one accordion half in its y-z
                                         plane (three 0.8 mm leaves and their walls, each leaf a plate strip as wide
                                         as the D-shape chord sqrt(32^2 - y^2) - 1 mm), same E and nu; 3.3-4.6 kN/m
                                         for E 2.5-3.5 GPa.  Zero preload at rest (see above).
    mu                0.5                sim.yaml (PLA on PLA) on the knob, pads, plates' bumps, tooth and peg.
    pad_mass          10 g               conditioning (the real pad is ~0.5 g; quasi-static here), as the outlet's
                                         and the USB slot's sprung pads.
    plate_mass        10 g               conditioning (real top plate ~3 g).
    pad_kd, plate_kd  15, 6 N.s/m        ~0.4 of critical on the conditioning masses.
    knob mass         print estimate     sim.yaml print profile (catalog.print_mass), knob + arrow.
    knob_armature     2.5e-3 kg.m^2      push dial only: armature on the knob's ROTATION, numerical conditioning
                                         (the knob itself is 5.2e-6 / 7.1e-6 kg.m^2 flat / round; the standard and
                                         snap knobs keep 2e-6).  See "The push knob's armature" below.

``build`` also asks the scene for hard MuJoCo joint limits (the pad, plate and knob stops are mechanism stops) and,
unless the scene set one, impratio 10 (stiff friction rows, the suite's grasp setting and what the dial tasks run:
at MuJoCo's default 1 the standard dial's brake creeps 1 deg at 0.58 N.m against a 1.04 N.m breakaway).  The dial's
contact shapes get a 1 mm torsional friction length (the rig's 0 makes MuJoCo's condim-4 cone degenerate).

**The push knob's armature** (2026-10-29).  Without it the flat knob went non-finite at random whenever it sat up:
kit check 2 of 6 runs (both in the check's final settle), the probe's first settle stretched to 10 s 6 of 6 (after
0.5-8.4 s); round knob 0 of 6 checks.  Cause, measured: up, the knob rests on the two bumps with its weight alone
(0.15 N over ~85 coplanar contacts: the CAD's zero preload), and that contact cannot settle.  The depth Newton's
mesh-SDF narrow phase reports for it is a staircase in the knob's height (the deepest contacts hold -3.15 um over
3.4 um of travel, then jump to +0.87 um, out of contact: the SDF is read through the texture unit's 8-bit
interpolation weights) against an equilibrium depth of ~0.03 um at that load, so the knob chatters (1-5 mm/s; the
bump contacts drop out and come back every ~5 ms) and the coplanar group keeps crossing the apex of MuJoCo's
elliptic friction cone together (normal residual ~1e-7, tangential ~1e-6) while the knob does not turn.  There
MuJoCo Warp's cone Hessian is not positive semidefinite: it floors t^3 at mjMINVAL = 1e-15 but not the t of mu/t,
so with t < 1e-5 and a positive normal residual N a contact adds -dm mu (N/t)(1 - t^3/1e-15) (u/t . mu_t J_t)^2 to
the knob's rotation (J_t: its friction rows' lever arms), at most D/(1 + mu^2) (mu_t r)^2 = 0.164/1.025 x
(0.5 x 21.6 mm)^2 = 1.9e-5 kg.m^2 per contact.  In the captured failure 39 such contacts (-3.6e-4 kg.m^2)
outweighed the knob's 7.2e-6 kg.m^2, the Newton Hessian went indefinite, its Cholesky (no pivot guard) returned NaN
and the step made every dof NaN (a float64 rebuild with the floor reproduces the GPU's Hessian; with the exact t^3
it is positive definite).  The armature exceeds the bound for all bump contacts at once (~92 x 1.9e-5 = 1.7e-3
kg.m^2), so the Hessian stays positive definite whatever the chatter does (smallest eigenvalue >= 2.1e-3 over 31k
Newton iterations, largest floor error seen 6.1e-4; 10 s rests 0 of 6, kit checks 0 of 6, flat and round).
It is not free: the pressed turning torque is a soft-friction artifact that depends on the knob's rotational
inertia, and it rises from 0.09 / 0.18 to 0.19 / 0.19 N.m flat / round (1e-3 kg.m^2: 0.23; 2e-4: 0.27 and still
non-finite; Coulomb 0.28), and the plates now and then lift off the knob's face while it turns pressed (the knob
then sits on its post stop).  Press forces, gate, stays-down and pop-up are unchanged; the task rows
turn_dial_to_target[_round] pass with the same angles (inertial torque <= 0.04 N.m at the wrist's spin-up and stop).

Probe (``force-bench kit check dial dial-push-down-to-turn dial-snap-to-position``), driven through the knob's own
joint (a torque ramp, then a compliant position drive; no robot):

* standard: breakaway (holding) torque, the turning torque over a 90 deg sweep and its ripple, drift after release;
* snap: breakaway torque at a detent, the torque and pad lift over a 150 deg sweep (detent spacing, peak torque),
  where a knob let go 20 deg off a detent ends up;
* push: an unpressed turn against the gate, the press force against depth, the turning torque while pressed, the
  knob after release at 90 deg (stays down, keeps its angle), and after turning back to 0 deg (pops up).

MEASURED (``force-bench kit check``, default quality, impratio 10; flat / round knob):

    standard  pads 0.68-0.70 mm out at rest (0.75 geometric; ~20 N each) and 0.65-0.71 mm all through the sweep
              (round hub: no detents); breakaway 1.04 / 1.04 N.m; turning (90 deg at 45 deg/s, median) 0.72 /
              0.95 N.m with stick-slip (0.2-1.0 N.m, the brake's static friction re-catching); drift after release
              0.00 deg.  (Coulomb estimate 6 x 0.5 x 20 N x 16 mm = 0.96 N.m.)
    snap      detents at 0 / 59.4 / 119.3 and 59.2 / 119.2 deg (spacing 59.7 / 60.0 deg), peaks at 31.7 / 89.7 /
              145.6 deg; pads 0.24 mm out seated, 1.11 mm at a peak; peak turning torque 1.65 / 1.66 N.m, breakaway
              out of a detent 0.58 N.m; a knob let go 20 deg off a detent stays at 21.5 deg (no self-centring).
    push      unpressed: turns 0.95 / 0.93 deg under ~1 N.m, then the gate holds (0.93 deg play); press force
              3.1-3.8 / 7.1-7.8 / 13.8-15.6 / 23.3 / 27.0 (round 26.8) N at 0.5 / 1 / 2 / 3 (gate clears) / 3.45 mm
              (= 7.9 kN/m, no preload, no parasitic friction; 6 runs, read at the first frame past each depth of the
              press ramp); turned 90 deg pressed (89.3 deg reached), let go: stays down at 3.00 mm and keeps its
              angle; turned back to 0: pops up to 0.05 mm.  Turning torque while pressed 0.19 / 0.19 N.m (6 runs:
              0.191-0.192 / 0.195; 0.09 / 0.18 before the armature, see above), 0.7 of the Coulomb estimate
              0.5 x 27.6 N x 20 mm = 0.28 N.m: MuJoCo's soft friction on the flat bump ring under-delivers while it
              slides, so a print turns stiffer and holds its off-zero angle harder than the model.
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv_processed as subdiv
from force_bench.kit.build import add_part_shape, part_mass, pose_matrix, to_wp

MM = 1e-3
JD = newton.ModelBuilder.JointDofConfig

TYPE_OF_KEY = {"dial": "standard", "dial-push-down-to-turn": "push", "dial-snap-to-position": "snap"}

# ── printed spring estimates (see the module docstring and _beam_estimates()) ─────────────────────────────────────
E_PLA = 3.0e9
NU_PLA = 0.35
K_PAD = 29.0e3            # N/m per pad (two serpentines): plane-strain FEM of the CAD section at E_PLA
                          # (beam-theory frame of the centrelines: 30.2e3)
K_PUSH_HALF = 3.94e3      # N/m per accordion half, beam theory at E_PLA
PAD_MASS = 0.010          # kg, conditioning value
PAD_KD = 15.0             # N.s/m
PAD_TRAVEL_OUT = 2.0e-3   # m, outward stop (the slits; the hub needs <= 1.2 mm)
PAD_TRAVEL_IN = 0.3e-3    # m, inward stop
PLATE_MASS = 0.010        # kg, conditioning value
PLATE_KD = 6.0            # N.s/m
PLATE_TRAVEL = 4.5e-3     # m, downward stop of the spring plate (the knob stop is 3.5 mm)
IMPRATIO = 10.0           # MuJoCo friction/normal impedance ratio requested for the scene (setdefault)
KNOB_ARMATURE = 2.0e-6    # kg.m^2 on the knob dofs, conditioning (the knob's own inertia is ~1e-5)
KNOB_ARMATURE_PUSH = 2.5e-3   # kg.m^2 on the push knob's ROTATION: keeps the Newton solver's Hessian positive
                              # definite through MuJoCo Warp's cone-Hessian clamp error (module docstring)
KNOB_ANG_DAMPING = 2.0e-4  # N.m.s/rad, light bearing damping on the knob rotation
SUB_EDGE = 1.0e-3         # m, contact-surface subdivision of the knob and the sprung parts
MU_TORSION = 1.0e-3       # m, torsional friction length of the dial's contact shapes (~ the contact patch radius).
                          # The rig's shapes carry 0, which makes MuJoCo's condim-4 elliptic cone degenerate
                          # (mujoco_warp: "friction[1] < MJ_MINMU with condim=4 may cause NaN").

# section heights in the CAD frame (mm): pads and hub overlap over z 10..18.5
Z_PAD = (8.5, 18.5)
Z_HUB_PROBE = 14.0
Z_GATE_PROBE = 23.0


# ── geometry helpers ──────────────────────────────────────────────────────────────────────────────────────────────
def _section_mm(tm: trimesh.Trimesh, z_mm: float):
    """Planar section of a CAD-frame mesh (metres) at height z, as a shapely geometry in millimetres."""
    from shapely.affinity import scale
    from shapely.ops import unary_union
    s = tm.section(plane_origin=(0.0, 0.0, z_mm * MM), plane_normal=(0.0, 0.0, 1.0))
    if s is None:
        return None
    to_plane = np.eye(4)
    to_plane[2, 3] = -z_mm * MM                         # keep x, y: the section in CAD-frame coordinates
    p2, _ = s.to_2D(to_2D=to_plane)
    return scale(unary_union(p2.polygons_full), 1.0 / MM, 1.0 / MM, origin=(0, 0))


def _support(geom, ang: float) -> float:
    """Support function (mm) of a planar geometry along direction ang (rad)."""
    pts = []
    for g in getattr(geom, "geoms", [geom]):
        pts.append(np.asarray(g.exterior.coords))
    p = np.vstack(pts)
    return float((p @ np.array([np.cos(ang), np.sin(ang)])).max())


def _ray_first(geom, ang: float, r0=0.0, r1=60.0) -> float:
    """First boundary radius (mm) of `geom` along a ray from the origin at angle ang (rad)."""
    from shapely.geometry import LineString
    d = np.array([np.cos(ang), np.sin(ang)])
    g = geom.intersection(LineString([tuple(d * r0), tuple(d * r1)]))
    if g.is_empty:
        return float("nan")
    segs = [g] if g.geom_type == "LineString" else list(g.geoms)
    return float(min(min(np.hypot(*np.asarray(s.coords).T)) for s in segs))


def _ray_segments(geom, ang: float, r1=60.0) -> list[tuple[float, float]]:
    """(r_min, r_max) of every piece of `geom` (mm) along the ray from the origin at angle ang, sorted."""
    from shapely.geometry import LineString
    d = np.array([np.cos(ang), np.sin(ang)])
    g = geom.intersection(LineString([(0.0, 0.0), tuple(d * r1)]))
    if g.is_empty:
        return []
    segs = [g] if g.geom_type == "LineString" else [s for s in g.geoms if s.geom_type == "LineString"]
    return sorted((float(np.hypot(*np.asarray(s.coords).T).min()), float(np.hypot(*np.asarray(s.coords).T).max()))
                  for s in segs)


def _prism(poly_xy_mm: np.ndarray, z0_mm: float, z1_mm: float) -> trimesh.Trimesh:
    """Convex prism (metres) over a planar point set (mm)."""
    p = np.asarray(poly_xy_mm, float)
    v = np.vstack([np.c_[p, np.full(len(p), z0_mm)], np.c_[p, np.full(len(p), z1_mm)]]) * MM
    return trimesh.convex.convex_hull(v)


def _annular_sector(r_in, r_out, a0, a1, z0, z1, n=48) -> trimesh.Trimesh:
    """Watertight (non-convex) annular sector prism (metres): radii r_in..r_out (mm), angles a0..a1 (rad)."""
    th = np.linspace(a0, a1, n)
    V = []
    for t in th:
        c, s = np.cos(t), np.sin(t)
        V += [(r_in * c, r_in * s, z0), (r_out * c, r_out * s, z0), (r_out * c, r_out * s, z1), (r_in * c, r_in * s, z1)]
    V = np.array(V) * MM
    F = []
    for i in range(n - 1):
        a, b = 4 * i, 4 * (i + 1)
        for k in range(4):
            k1 = (k + 1) % 4
            F += [(a + k, b + k, b + k1), (a + k, b + k1, a + k1)]
    F += [(0, 2, 1), (0, 3, 2)]
    e = 4 * (n - 1)
    F += [(e, e + 1, e + 2), (e, e + 2, e + 3)]
    tm = trimesh.Trimesh(V, np.array(F), process=True)
    trimesh.repair.fix_normals(tm)
    return tm


def _rot_z(a: float) -> np.ndarray:
    return trimesh.transformations.rotation_matrix(a, (0, 0, 1))


# ── shape helpers ─────────────────────────────────────────────────────────────────────────────────────────────────
def _shape_cfg(rig, mu: float, collide: bool = True, visible: bool = True, density: float = 1240.0):
    cfg = rig.cfg(mu=mu, density=density)
    if not collide:
        cfg.has_shape_collision = False
        cfg.has_particle_collision = False
    cfg.is_visible = visible
    return cfg


def _add_mesh(rig, body: int, tm: trimesh.Trimesh, body_from_mesh: np.ndarray, label: str, rgb, mu: float,
              collide=True, visible=True, sdf_res: int = 128, density: float = 1240.0) -> int:
    """A mesh shape on `body`; `tm` in its own frame (metres), placed by body_from_mesh."""
    nm, c = make_newton_mesh(tm, sdf=collide, sdf_resolution=sdf_res, narrow_band=0.002, margin=0.0006)
    T = body_from_mesh @ pose_matrix(xyz=c)
    sid = rig.builder.add_shape_mesh(body, xform=to_wp(T), mesh=nm,
                                     cfg=_shape_cfg(rig, mu, collide, visible, density), color=rgb, label=label)
    return sid


def _torsion(rig, sids):
    for s in sids:
        rig.builder.shape_material_mu_torsional[s] = MU_TORSION


def _parts_by_role(parts):
    roles = {"bottom": None, "top": None, "grid": None, "knob": None, "arrow": None, "spring": []}
    for p in parts:
        n = p.name
        if "spring" in n:
            roles["spring"].append(p)
        elif "knob" in n:
            roles["knob"] = p
        elif n.endswith("arrow"):
            roles["arrow"] = p
        elif n.endswith("bottom"):
            roles["bottom"] = p
        elif n.endswith("_top"):
            roles["top"] = p
        elif n.endswith("grid"):
            roles["grid"] = p
    missing = [k for k, v in roles.items() if not v]
    if missing:
        raise KeyError(f"dial: parts not found for roles {missing}")
    return roles


def dial_type(asset) -> str:
    return asset.sim.get("type") or TYPE_OF_KEY[asset.key]


# ── build ─────────────────────────────────────────────────────────────────────────────────────────────────────────
def build(rig, asset, h, **params) -> None:
    kind = dial_type(asset)
    mu = float(params.pop("mu", asset.sim.get("mu", 0.5)))
    roles = _parts_by_role(asset.parts(h.choice))
    pose = np.asarray(h.pose, float)
    h.info = {"type": kind, "angle_convention": "0 at the CAD pose (arrow on '0'), +ccw seen from above"}
    # hard MuJoCo joint limits: the pad / plate / knob stops are mechanism stops (rig docstring)
    rig.hard_joint_limits = True
    # stiff friction rows, the suite's grasp setting (and what the dial tasks run): the dials hold by friction, and at
    # MuJoCo's default impratio 1 the standard dial's brake creeps 1 deg at 0.58 N.m against a 1.04 N.m breakaway
    rig.solver_options.setdefault("impratio", IMPRATIO)

    # static housing: bottom and top collide (with the robot), the inlaid grid is drawn only
    statics = []
    for role in ("bottom", "top", "grid"):
        p = roles[role]
        sid = add_part_shape(rig, -1, asset, p, pose, h.label, collide=(role != "grid"))
        h.shapes[p.name] = [sid]
        statics.append(sid)
    h.info["static_shapes"] = statics

    # the knob (+ arrow): one body on the dial axis
    knob_p, arrow_p = roles["knob"], roles["arrow"]
    ktm = knob_p.mesh()
    z_ref = float(0.5 * (ktm.bounds[0][2] + ktm.bounds[1][2]))
    up = 0.0
    if kind == "push":
        g = _push_geometry(asset, h, roles)
        up = g["up_gap_mm"] * MM                         # the knob rests against the cover ceiling
    world_from_knob = pose @ pose_matrix(xyz=(0.0, 0.0, z_ref + up))
    knob = rig.builder.add_link(xform=to_wp(world_from_knob), label=f"{h.label}/knob")
    knob_from_cad = np.linalg.inv(world_from_knob) @ pose @ pose_matrix(xyz=(0.0, 0.0, up))   # raised by `up`
    sid_k = add_part_shape(rig, knob, asset, knob_p, knob_from_cad, h.label, mesh=subdiv(ktm, 2.0 * MM), mu=mu)
    sid_a = add_part_shape(rig, knob, asset, arrow_p, knob_from_cad, h.label, collide=False)
    h.shapes[knob_p.name] = [sid_k]
    h.shapes[arrow_p.name] = [sid_a]
    _torsion(rig, [sid_k])
    m_knob = part_mass(asset, knob_p) + part_mass(asset, arrow_p)
    h.bodies["knob"] = knob
    for s in statics:
        rig.builder.add_shape_collision_filter_pair(sid_k, s)

    if kind == "push":
        _build_push(rig, asset, h, roles, g, knob, sid_k, world_from_knob, m_knob, mu, params)
    else:
        _build_pads(rig, asset, h, roles, kind, knob, sid_k, world_from_knob, m_knob, mu, params)
    if params:
        raise TypeError(f"dial.build: unknown parameters {sorted(params)}")
    h.info["probe_bodies"] = [f"{h.label}/knob"]


def _knob_joint_revolute(rig, h, knob, world_from_knob, m_knob):
    # POSITION actuator mode with zero stiffness: a light bearing damper now, and a drive whose gains a probe
    # (or a test) can raise at run time (MuJoCo builds an actuator only for dofs that have a target mode)
    j = rig.builder.add_joint_revolute(
        parent=-1, child=knob, axis=(0.0, 0.0, 1.0), parent_xform=to_wp(world_from_knob),
        child_xform=wp.transform_identity(), target_pos=0.0, target_ke=0.0, target_kd=KNOB_ANG_DAMPING,
        armature=KNOB_ARMATURE, actuator_mode=newton.JointTargetMode.POSITION, label=f"{h.label}/knob")
    rig.builder.add_articulation([j])
    set_body_mass(rig.builder, knob, m_knob)
    h.joints["knob"] = j
    return j


# ── standard / snap: six sprung pads around the hub ──────────────────────────────────────────────────────────────
def _pad_geometry(asset, roles):
    """Pad prisms and hub profile measured on the CAD (mm, CAD frame)."""
    spring = roles["spring"][0].mesh()
    knob = roles["knob"].mesh()
    zc = 0.5 * (Z_PAD[0] + Z_PAD[1]) + 0.5
    S = _section_mm(spring, zc)
    H = _section_mm(knob, Z_HUB_PROBE)
    # pad centres: the directions where the spring's inner boundary is CLOSEST (the flat centres);
    # slits (gaps between pads) read the outer ring instead
    angs = np.radians(np.arange(0.0, 360.0, 0.5))
    first = np.array([_ray_first(S, a) for a in angs])
    r_face = float(np.nanmin(first))
    cand = angs[first < r_face + 0.02]
    # cluster the minima into six pad directions
    centres = []
    for a in np.sort(cand):
        if centres and abs(np.angle(np.exp(1j * (a - centres[-1][-1])))) < np.radians(5):
            centres[-1].append(a)
        else:
            centres.append([a])
    if len(centres) > 1 and abs(np.angle(np.exp(1j * (centres[0][0] - centres[-1][-1])))) < np.radians(5):
        centres[0] = centres.pop() + centres[0]
    phis = sorted(float(np.angle(np.mean(np.exp(1j * np.array(c))))) for c in centres)
    if len(phis) != 6:
        raise RuntimeError(f"dial: expected 6 spring pads, found {len(phis)} at {np.degrees(phis)}")
    # each pad: the spring section within its +-29.5 deg sector, radially between the face and the pad's back
    # edge (where the serpentine stems start), as a convex prism section in pad-local coordinates
    from shapely.affinity import rotate
    from shapely.geometry import LineString, Polygon
    pads = []
    for phi in phis:
        u = np.array([np.cos(phi), np.sin(phi)])
        line = S.intersection(LineString([tuple(u * r_face * 0.99), tuple(u * 40.0)]))
        segs = [line] if line.geom_type == "LineString" else list(line.geoms)
        r_back = min(max(np.hypot(*np.asarray(s.coords).T)) for s in segs)
        local = rotate(S, -phi, origin=(0, 0), use_radians=True)       # pad axis along +x
        sector = Polygon([(0, 0)] + [(60 * np.cos(t), 60 * np.sin(t)) for t in np.radians(np.linspace(-29.5, 29.5, 31))])
        band = Polygon([(r_face - 1.0, -30), (r_back - 0.03, -30), (r_back - 0.03, 30), (r_face - 1.0, 30)])
        piece = local.intersection(sector).intersection(band)
        geoms = [g for g in getattr(piece, "geoms", [piece]) if g.geom_type == "Polygon" and g.area > 1.0]
        poly = max(geoms, key=lambda g: g.area).convex_hull
        pads.append({"phi": phi, "local_xy": np.asarray(poly.exterior.coords)[:-1], "r_back": float(r_back)})
    hub_support = {round(float(np.degrees(p["phi"])), 2): round(_support(H, p["phi"]), 3) for p in pads}
    hub_r = np.array([_support(H, a) for a in np.radians(np.arange(0.0, 60.0, 0.25))])
    return {"r_face": r_face, "pads": pads, "hub_support": hub_support,
            "hub_r_min": float(hub_r.min()), "hub_r_max": float(hub_r.max())}


def _build_pads(rig, asset, h, roles, kind, knob, sid_k, world_from_knob, m_knob, mu, params):
    k_pad = float(params.pop("k_pad", K_PAD))
    kd = float(params.pop("pad_kd", PAD_KD))
    pad_mass = float(params.pop("pad_mass", PAD_MASS))
    pose = h.pose
    _knob_joint_revolute(rig, h, knob, world_from_knob, m_knob)
    g = _pad_geometry(asset, roles)
    spring_p = roles["spring"][0]
    # the printed spring: drawn only (its pads are the bodies below)
    h.shapes[spring_p.name] = [add_part_shape(rig, -1, asset, spring_p, pose, h.label, collide=False)]
    pad_ids = []
    preloads = []
    for k, pad in enumerate(g["pads"]):
        phi = pad["phi"]
        # pad frame: origin on the face at mid height, x radial out
        T_cad = _rot_z(phi) @ pose_matrix(xyz=(g["r_face"] * MM, 0.0, 0.5 * (Z_PAD[0] + Z_PAD[1]) * MM))
        T_w = pose @ T_cad
        xy = pad["local_xy"] - np.array([g["r_face"], 0.0])
        tm = _prism(xy, Z_PAD[0] - 0.5 * (Z_PAD[0] + Z_PAD[1]), Z_PAD[1] - 0.5 * (Z_PAD[0] + Z_PAD[1]))
        tm = subdiv(tm, SUB_EDGE)
        q0 = max(0.0, g["hub_support"][round(float(np.degrees(phi)), 2)] - g["r_face"]) * MM
        body = rig.builder.add_link(xform=to_wp(T_w @ pose_matrix(xyz=(q0, 0.0, 0.0))), label=f"{h.label}/pad_{k}")
        sid = _add_mesh(rig, body, tm, np.eye(4), f"kit/{h.label}/pad_{k}", spring_p.rgb, mu, sdf_res=96)
        _torsion(rig, [sid])
        j = rig.builder.add_joint_prismatic(
            parent=-1, child=body, axis=(1.0, 0.0, 0.0), parent_xform=to_wp(T_w),
            child_xform=wp.transform_identity(), target_pos=0.0, target_ke=k_pad, target_kd=kd,
            limit_lower=-PAD_TRAVEL_IN, limit_upper=PAD_TRAVEL_OUT, label=f"{h.label}/pad_{k}")
        rig.builder.add_articulation([j])
        set_body_mass(rig.builder, body, pad_mass)
        rig.builder.joint_q[rig.builder.joint_q_start[j]] = q0
        for s in h.info["static_shapes"]:
            rig.builder.add_shape_collision_filter_pair(sid, s)
        for s in pad_ids:
            rig.builder.add_shape_collision_filter_pair(sid, s)
        pad_ids.append(sid)
        h.bodies[f"pad_{k}"], h.joints[f"pad_{k}"], h.shapes[f"pad_{k}"] = body, j, [sid]
        preloads.append(q0)
    pre = np.array(preloads)
    h.params = {"type": kind, "k_pad_N_per_m": round(k_pad, 1), "pad_kd": kd, "pad_mass_kg": pad_mass, "mu": mu,
                "knob_mass_kg": round(m_knob, 5), "k_pad_source": "beam-theory estimate (module docstring)"}
    h.info.update({
        "pad_face_radius_mm": round(g["r_face"], 3),
        "pad_angles_deg": [round(float(np.degrees(p["phi"])), 2) for p in g["pads"]],
        "pad_back_radius_mm": round(float(np.mean([p["r_back"] for p in g["pads"]])), 3),
        "hub_radius_min_max_mm": [round(g["hub_r_min"], 3), round(g["hub_r_max"], 3)],
        "pad_preload_mm": [round(float(x) / MM, 3) for x in pre],
        "pad_peak_mm": round(g["hub_r_max"] - g["r_face"], 3),
        "pad_preload_force_N": round(float(k_pad * pre.mean()), 2),
    })


# ── push-down: accordion springs, gate ───────────────────────────────────────────────────────────────────────────
def _push_geometry(asset, h, roles) -> dict:
    """Measured on the CAD (mm): the cover ceiling gap, the post stop, tooth/peg/notch/window, the bump ring."""
    top = roles["top"].mesh()
    knob = roles["knob"].mesh()
    bottom = roles["bottom"].mesh()
    springs = [p.mesh() for p in roles["spring"]]

    def zs(tm, x, y):
        loc, _, _ = tm.ray.intersects_location([[x * MM, y * MM, 0.2]], [[0.0, 0.0, -1.0]])
        return np.sort(loc[:, 2]) / MM if len(loc) else np.array([])

    T = _section_mm(top, Z_GATE_PROBE)
    K = _section_mm(knob, Z_GATE_PROBE - 1.0)
    # the gate features sit at 180 deg (tooth in the rim notch) and 0 deg (peg in the window)
    tooth_in = _ray_first(T, np.pi, r0=0.0, r1=40.0)
    peg = T.intersection(_ring_polygon(15.0, 28.0))
    peg = max([g for g in getattr(peg, "geoms", [peg]) if g.geom_type == "Polygon"], key=lambda g: g.area)
    peg_r = [float(np.hypot(*np.asarray(peg.exterior.coords).T).min()),
             float(np.hypot(*np.asarray(peg.exterior.coords).T).max())]

    def half_angle(geom, r, ang):
        from shapely.geometry import LineString
        c = np.array([np.cos(ang), np.sin(ang)])
        n = np.array([-np.sin(ang), np.cos(ang)])
        seg = geom.intersection(LineString([tuple(c * r - n * 12), tuple(c * r + n * 12)]))
        segs = [seg] if seg.geom_type == "LineString" else list(getattr(seg, "geoms", []))
        best = None
        for s in segs:
            p = np.asarray(s.coords)
            t = (p - c * r) @ n
            if t.min() <= 0.0 <= t.max():
                best = (t.min(), t.max())
        return best

    def gap_half_angle(geom, r, ang):
        """Half angle (rad) of the GAP in geom around direction ang at radius r."""
        from shapely.geometry import LineString
        c = np.array([np.cos(ang), np.sin(ang)])
        n = np.array([-np.sin(ang), np.cos(ang)])
        seg = geom.intersection(LineString([tuple(c * r - n * 12), tuple(c * r + n * 12)]))
        segs = [seg] if seg.geom_type == "LineString" else list(getattr(seg, "geoms", []))
        lo, hi = -12.0, 12.0
        for s in segs:
            t = (np.asarray(s.coords) - c * r) @ n
            if t.max() < 0:
                lo = max(lo, t.max())
            elif t.min() > 0:
                hi = min(hi, t.min())
        return float(np.arctan2(0.5 * (hi - lo), r)), 0.5 * (hi + lo)

    r_t = 0.5 * (tooth_in + 32.5)
    tooth_half = half_angle(T, r_t, np.pi)
    tooth_ha = float(np.arctan2(0.5 * (tooth_half[1] - tooth_half[0]), r_t))
    r_p = 0.5 * (peg_r[0] + peg_r[1])
    peg_half = half_angle(T, r_p, 0.0)
    peg_ha = float(np.arctan2(0.5 * (peg_half[1] - peg_half[0]), r_p))
    notch_ha, _ = gap_half_angle(K, r_t, np.pi)
    window_ha, _ = gap_half_angle(K, r_p, 0.0)
    # radial extent of the window and of the notch floor (knob): the peg must clear the window's inner edge
    ends = _ray_segments(K, 0.0)
    window_r = next(((a[1], b[0]) for a, b in zip(ends[:-1], ends[1:]) if a[1] < r_p < b[0]), (peg_r[0], peg_r[1]))
    notch_floor = max(b for a, b in _ray_segments(K, np.pi) if a < 10.0)
    peg_cad_r_in = peg_r[0]
    peg_r = [max(peg_r[0], window_r[0] + 0.25), min(peg_r[1], window_r[1] - 0.25)]
    z_tooth = zs(top, -r_t, 0.0)
    z_peg = zs(top, r_p, 0.0)
    z_knob_disk = zs(knob, -0.5 * (tooth_in + 22.0), 12.0)     # the disk face beside the notch
    z_ceiling = zs(top, -27.0, 15.0)
    z_post = zs(bottom, 0.0, 0.0)
    z_bore = zs(knob, 0.0, 0.0)
    # bump ring on each spring half: its top and radial extent
    bump = []
    for sp in springs:
        cx = 1.0 if sp.bounds[0][0] > -1e-4 else -1.0
        zz = zs(sp, cx * 20.0, 0.0)
        S = _section_mm(sp, float(zz.max()) - 0.6)
        rr = np.hypot(*np.asarray(S.exterior.coords).T) if S.geom_type == "Polygon" else \
            np.hypot(*np.vstack([np.asarray(g.exterior.coords) for g in S.geoms]).T)
        bump.append({"side": cx, "z_top": float(zz.max()), "r_in": float(rr.min()), "r_out": float(rr.max())})
    up_gap = float(z_ceiling.min() - z_knob_disk.max())
    engage = float(z_knob_disk.max() - z_tooth.min())             # at the CAD pose
    post_gap = float(z_bore[z_bore > z_post.max() - 1e-6].min() - z_post.max()) if len(z_bore) else float("nan")
    knob_bottom = float(knob.bounds[0][2] / MM)
    return {
        "up_gap_mm": up_gap,                             # knob face to cover ceiling at the CAD pose
        "engage_mm": engage + up_gap,                    # gate engagement at the up rest
        "stroke_mm": post_gap + up_gap,                  # bore bottoms on the post (from the up rest)
        "tooth": {"r_in": tooth_in, "r_out": 32.6, "half_ang": tooth_ha, "z0": float(z_tooth.min()),
                  "z1": float(z_ceiling.min()) - 0.1},
        "peg": {"r_in": peg_r[0], "r_out": peg_r[1], "half_ang": peg_ha, "z0": float(z_peg.min()),
                "z1": float(z_ceiling.min()) - 0.1},
        "notch_half_ang": notch_ha, "window_half_ang": window_ha,
        "window_r": [float(window_r[0]), float(window_r[1])], "notch_floor_r": float(notch_floor),
        "peg_cad_r_in": float(peg_cad_r_in),
        "bump": bump, "knob_bottom_mm": knob_bottom,
    }


def _ring_polygon(r0, r1, n=180):
    from shapely.geometry import Polygon
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return Polygon([(r1 * np.cos(t), r1 * np.sin(t)) for t in th],
                   [[(r0 * np.cos(t), r0 * np.sin(t)) for t in th]])


def _build_push(rig, asset, h, roles, g, knob, sid_k, world_from_knob, m_knob, mu, params):
    k_half = float(params.pop("k_push", K_PUSH_HALF))
    kd = float(params.pop("plate_kd", PLATE_KD))
    plate_mass = float(params.pop("plate_mass", PLATE_MASS))
    knob_arm = float(params.pop("knob_armature", KNOB_ARMATURE_PUSH))
    pose = h.pose
    stroke = g["stroke_mm"] * MM
    # knob joint: vertical slide (0 = up rest against the ceiling, down to the post stop) + free rotation
    j = rig.builder.add_joint_d6(
        parent=-1, child=knob,
        linear_axes=[JD(axis=(0.0, 0.0, 1.0), limit_lower=-stroke, limit_upper=0.0, target_ke=0.0, target_kd=0.0,
                        armature=KNOB_ARMATURE, actuator_mode=newton.JointTargetMode.POSITION)],
        angular_axes=[JD(axis=(0.0, 0.0, 1.0), target_ke=0.0, target_kd=KNOB_ANG_DAMPING, armature=knob_arm,
                         actuator_mode=newton.JointTargetMode.POSITION)],
        parent_xform=to_wp(world_from_knob), child_xform=wp.transform_identity(), label=f"{h.label}/knob")
    rig.builder.add_articulation([j])
    set_body_mass(rig.builder, knob, m_knob)
    h.joints["knob"] = j
    # gate: tooth (180 deg) and peg (0 deg), static wedges from the CAD measurements
    gate = []
    for name, f, ang in (("tooth", g["tooth"], np.pi), ("peg", g["peg"], 0.0)):
        # a true annular sector: the CAD tooth and peg have arc faces concentric with the dial (a convex hull would
        # replace the inner arc by its chord, 0.35 mm proud at the middle -- into the knob's window edge)
        tm = subdiv(_annular_sector(f["r_in"], f["r_out"], -f["half_ang"], f["half_ang"], f["z0"], f["z1"], n=24),
                    SUB_EDGE)
        sid = _add_mesh(rig, -1, tm, pose @ _rot_z(ang), f"kit/{h.label}/gate_{name}", roles["top"].rgb, mu,
                        visible=False, sdf_res=96)
        gate.append(sid)
        h.shapes[f"gate_{name}"] = [sid]
        _torsion(rig, [sid])
    # springs: the lower part drawn static, the top plate (with the bump) drawn on a sprung body
    plates = []
    up = g["up_gap_mm"]
    for sp_p, b in zip(roles["spring"], sorted(g["bump"], key=lambda b: b["side"])):
        sp = sp_p.mesh()
        z_cut = _plate_cut_z(sp)
        lower = sp.slice_plane((0, 0, z_cut * MM), (0, 0, -1), cap=False)
        upper = sp.slice_plane((0, 0, z_cut * MM), (0, 0, 1), cap=False)
        h.shapes[sp_p.name] = [_add_mesh(rig, -1, lower, pose, f"kit/{h.label}/{sp_p.name}", sp_p.rgb, mu,
                                         collide=False)]
        side = b["side"]
        # plate body frame: the bump's top centre at the free height (= the knob face at the up rest)
        T_w = pose @ pose_matrix(xyz=(0.0, 0.0, b["z_top"] * MM))
        body = rig.builder.add_link(xform=to_wp(T_w), label=f"{h.label}/plate_{'+x' if side > 0 else '-x'}")
        vis = _add_mesh(rig, body, upper, pose_matrix(xyz=(0.0, 0.0, -b["z_top"] * MM)),
                        f"kit/{h.label}/{sp_p.name}_plate", sp_p.rgb, mu, collide=False)
        a0, a1 = (-np.pi / 2 + 0.03, np.pi / 2 - 0.03) if side > 0 else (np.pi / 2 + 0.03, 3 * np.pi / 2 - 0.03)
        bump_tm = subdiv(_annular_sector(b["r_in"] + 0.3, b["r_out"] - 0.3, a0, a1, -1.5, 0.0), SUB_EDGE)
        sid = _add_mesh(rig, body, bump_tm, np.eye(4), f"kit/{h.label}/{sp_p.name}_bump", sp_p.rgb, mu,
                        visible=False, sdf_res=128)
        _torsion(rig, [sid])
        # free height: the bump top touches the knob face at the up rest (CAD: 0.5 mm interference, 0.5 mm
        # gap to the ceiling -> zero preload)
        free = (b["z_top"] - (g["knob_bottom_mm"] + up))
        jp = rig.builder.add_joint_prismatic(
            parent=-1, child=body, axis=(0.0, 0.0, 1.0), parent_xform=to_wp(T_w),
            child_xform=wp.transform_identity(), target_pos=-free * MM, target_ke=k_half, target_kd=kd,
            limit_lower=-PLATE_TRAVEL, limit_upper=0.2 * MM, label=f"{h.label}/plate_{'+x' if side > 0 else '-x'}")
        rig.builder.add_articulation([jp])
        set_body_mass(rig.builder, body, plate_mass)
        for s in h.info["static_shapes"] + gate:
            rig.builder.add_shape_collision_filter_pair(sid, s)
        for s in plates:
            rig.builder.add_shape_collision_filter_pair(sid, s)
        plates.append(sid)
        key = f"plate_{'+x' if side > 0 else '-x'}"
        h.bodies[key], h.joints[key], h.shapes[key] = body, jp, [sid, vis]
    h.params = {"type": "push", "k_push_half_N_per_m": round(k_half, 1), "plate_kd": kd,
                "plate_mass_kg": plate_mass, "mu": mu, "knob_mass_kg": round(m_knob, 5),
                "knob_armature_kg_m2": knob_arm,
                "k_push_source": "beam-theory estimate (module docstring)"}
    deg = np.degrees
    h.info.update({
        "up_gap_mm": round(g["up_gap_mm"], 3),
        "gate_engagement_mm": round(g["engage_mm"], 3),
        "press_stroke_mm": round(g["stroke_mm"], 3),
        "gate_clearance_at_stop_mm": round(g["stroke_mm"] - g["engage_mm"], 3),
        "tooth": {"r_mm": [round(g["tooth"]["r_in"], 2), round(g["tooth"]["r_out"], 2)],
                  "half_angle_deg": round(float(deg(g["tooth"]["half_ang"])), 2),
                  "z_mm": [round(g["tooth"]["z0"], 2), round(g["tooth"]["z1"], 2)]},
        "peg": {"r_mm": [round(g["peg"]["r_in"], 2), round(g["peg"]["r_out"], 2)],
                "half_angle_deg": round(float(deg(g["peg"]["half_ang"])), 2),
                "z_mm": [round(g["peg"]["z0"], 2), round(g["peg"]["z1"], 2)]},
        "notch_half_angle_deg": round(float(deg(g["notch_half_ang"])), 2),
        "window_half_angle_deg": round(float(deg(g["window_half_ang"])), 2),
        "gate_play_deg": round(float(deg(min(g["notch_half_ang"] - g["tooth"]["half_ang"],
                                             g["window_half_ang"] - g["peg"]["half_ang"]))), 2),
        "bump_ring_mm": [[round(b["r_in"], 2), round(b["r_out"], 2), round(b["z_top"], 2)] for b in g["bump"]],
        "spring_force_at_gate_clear_N": round(2 * k_half * g["engage_mm"] * MM, 2),
        "spring_force_at_stop_N": round(2 * k_half * stroke, 2),
    })


def _plate_cut_z(sp: trimesh.Trimesh) -> float:
    """Height (mm) just below the spring's top plate: the lowest z of the top plate's underside."""
    zs = []
    for y in (-26.0, -14.0, 0.0, 14.0, 26.0):
        x = 15.0 if sp.bounds[0][0] > -1e-4 else -15.0
        loc, _, _ = sp.ray.intersects_location([[x * MM, y * MM, 0.2]], [[0.0, 0.0, -1.0]])
        if len(loc) >= 2:
            z = np.sort(loc[:, 2]) / MM
            zs.append(z[-2])                               # the plate's underside
    return float(min(zs)) - 0.05


# ── measuring helpers for tasks and probes ────────────────────────────────────────────────────────────────────────
def knob_coords(rig, h, state=None) -> tuple[float, float]:
    """(angle [rad], press [m], negative = pressed) of the knob, from the joint coordinates."""
    st = state or rig.s0
    q = st.joint_q.numpy()
    i = int(rig.model.joint_q_start.numpy()[h.joints["knob"]])
    if h.info["type"] == "push":
        return float(q[i + 1]), float(q[i])
    return float(q[i]), 0.0


def _knob_dofs(rig, h) -> dict:
    j = h.joints["knob"]
    qs = int(rig.model.joint_q_start.numpy()[j])
    ds = int(rig.model.joint_qd_start.numpy()[j])
    ts = int(rig.model.joint_target_q_start.numpy()[j]) if hasattr(rig.model, "joint_target_q_start") else qs
    if h.info["type"] == "push":
        return {"ang": (qs + 1, ds + 1, ts + 1), "lin": (qs, ds, ts)}
    return {"ang": (qs, ds, ts)}


# ── probe ─────────────────────────────────────────────────────────────────────────────────────────────────────────
PROBE_KE_ANG = 20.0       # N.m/rad, the sweep drive (1 N.m of resistance lags it 2.9 deg)
PROBE_KD_ANG = 0.3        # N.m.s/rad
PROBE_KD_LIN = 20.0       # N.s/m on the knob slide while the probe presses it
PRESS_F_MAX = 40.0        # N, the probe's press force (above the springs' 27.6 N at the stop)
PRESS_F_RATE = 15.0       # N/s
SWEEP_RATE = np.radians(45.0)     # rad/s
TAU_RAMP = 2.5            # N.m/s, breakaway ramp rate
BREAK_DEG = 1.0           # rotation that counts as breakaway


def add_probe(rig, h) -> None:
    """Nothing to add: the probe drives the knob's own joint."""


class _Driver:
    """Sets the knob joint's drive gains/targets and its applied torque, frame by frame."""

    def __init__(self, rig, h):
        self.rig, self.h = rig, h
        self.d = _knob_dofs(rig, h)
        self.ke0 = rig.model.joint_target_ke.numpy().copy()
        self.kd0 = rig.model.joint_target_kd.numpy().copy()

    def gains(self, ang=None, lin=None):
        key = (ang, lin)
        if getattr(self, "_last", None) == key:
            return
        self._last = key
        ke = self.rig.model.joint_target_ke.numpy()
        kd = self.rig.model.joint_target_kd.numpy()
        for key, g in (("ang", ang), ("lin", lin)):
            if key not in self.d:
                continue
            dof = self.d[key][1]
            ke[dof], kd[dof] = (self.ke0[dof], self.kd0[dof]) if g is None else g
        self.rig.model.joint_target_ke.assign(ke)
        self.rig.model.joint_target_kd.assign(kd)
        self._notify()

    def _notify(self):
        s = self.rig.solver
        if hasattr(s, "notify_model_changed"):
            s.notify_model_changed(newton.ModelFlags.JOINT_DOF_PROPERTIES)

    def target(self, ang=None, lin=None):
        qt = self.rig.control.joint_target_q.numpy()
        if ang is not None:
            qt[self.d["ang"][2]] = ang
        if lin is not None and "lin" in self.d:
            qt[self.d["lin"][2]] = lin
        self.rig.control.joint_target_q.assign(qt)

    def torque(self, tau=0.0, force=0.0):
        f = self.rig.control.joint_f.numpy()
        f[self.d["ang"][1]] = tau
        if "lin" in self.d:
            f[self.d["lin"][1]] = force
        self.rig.control.joint_f.assign(f)

    def state(self):
        q = self.rig.s0.joint_q.numpy()
        qd = self.rig.s0.joint_qd.numpy()
        out = {"ang": float(q[self.d["ang"][0]]), "w": float(qd[self.d["ang"][1]])}
        if "lin" in self.d:
            out["z"] = float(q[self.d["lin"][0]])
            out["vz"] = float(qd[self.d["lin"][1]])
        return out


def _segments(rig, h, plan):
    """Run a probe plan: list of (name, duration, fn(drv, tau) -> None). Returns rows of measurements."""
    drv = _Driver(rig, h)
    rows = []
    for name, dur, fn in plan:
        t0 = [None]

        def ctl(t, fn=fn, t0=t0):
            if t0[0] is None:
                t0[0] = t
            fn(drv, t - t0[0])

        def extra(state, t, name=name):
            s = drv.state()
            qt = rig.control.joint_target_q.numpy()
            ke = rig.model.joint_target_ke.numpy()
            kd = rig.model.joint_target_kd.numpy()
            f = rig.control.joint_f.numpy()
            da = drv.d["ang"]
            tau = ke[da[1]] * (qt[da[2]] - s["ang"]) - kd[da[1]] * s["w"] + f[da[1]]
            row = {"phase": name, "t": t, "ang": s["ang"], "tau": float(tau)}
            pr = rig.probes.get(f"{h.label}/knob")
            if pr is not None and pr.history:
                row["contact_N"] = float(pr.history[-1]["force"])
                row["contact_fric_N"] = float(pr.history[-1]["friction"])
            if "lin" in drv.d:
                dl = drv.d["lin"]
                row["z"] = s["z"]
                row["fz"] = float(ke[dl[1]] * (qt[dl[2]] - s["z"]) - kd[dl[1]] * s["vz"] + f[dl[1]])
            if h.info["type"] != "push":
                q = state.joint_q.numpy()
                qs = rig.model.joint_q_start.numpy()
                row["pads_mm"] = [float(q[qs[h.joints[f"pad_{k}"]]]) / MM for k in range(6)]
            rows.append(row)
            return {}

        rig.run(dur, ctl, extra, print_every=10 ** 9)
    drv.gains()
    drv.torque(0.0, 0.0)
    return rows


def run_probe(rig, h) -> dict:
    kind = h.info["type"]
    out = {"type": kind}
    if kind == "push":
        res = _probe_push(rig, h)
    else:
        res = _probe_hub(rig, h, kind)
    out.update(res)
    import json
    out_dir = rig.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = out.pop("_rows", [])
    if rows:
        keys = [k for k in rows[0] if k not in ("phase", "pads_mm")]
        with open(out_dir / "probe_trace.csv", "w") as f:
            f.write("phase," + ",".join(keys) + (",pads_mm" if "pads_mm" in rows[0] else "") + "\n")
            for r in rows:
                f.write(r["phase"] + "," + ",".join(f"{r[k]:.6g}" for k in keys)
                        + ("," + " ".join(f"{x:.4f}" for x in r["pads_mm"]) if "pads_mm" in r else "") + "\n")
    json.dumps(out)        # must be serialisable
    return out


def _probe_hub(rig, h, kind) -> dict:
    """Standard / snap: breakaway torque, sweep torque profile, release behaviour."""
    sweep = np.radians(90.0 if kind == "standard" else 150.0)
    t_sweep = sweep / SWEEP_RATE
    tau_max = 4.0
    st = {"a0": None, "broke": None, "tau_break": float("nan"), "a_start": None, "park0": None}

    def settle(drv, tau):
        drv.gains(ang=(0.0, KNOB_ANG_DAMPING))
        drv.torque(0.0)

    def ramp(drv, tau):               # torque ramp until the knob slips BREAK_DEG, then hold it there
        s = drv.state()
        if st["a0"] is None:
            st["a0"] = s["ang"]
        if st["broke"] is None and abs(s["ang"] - st["a0"]) > np.radians(BREAK_DEG):
            st["broke"], st["tau_break"] = s["ang"], min(tau_max, TAU_RAMP * tau)
        if st["broke"] is not None:
            drv.torque(0.0)
            drv.gains(ang=(PROBE_KE_ANG, PROBE_KD_ANG))
            drv.target(ang=st["broke"])
        else:
            drv.gains(ang=(0.0, KNOB_ANG_DAMPING))
            drv.torque(min(tau_max, TAU_RAMP * tau))

    def sweep_fn(drv, tau):
        if st["a_start"] is None:
            st["a_start"] = drv.state()["ang"]
        drv.torque(0.0)
        drv.gains(ang=(PROBE_KE_ANG, PROBE_KD_ANG))
        drv.target(ang=st["a_start"] + min(sweep, SWEEP_RATE * tau))

    def release(drv, tau):
        drv.gains(ang=(0.0, KNOB_ANG_DAMPING))
        drv.torque(0.0)

    def park(drv, tau):               # ramp to 20 deg off the 0 deg detent (a third of the way to the next)
        if st["park0"] is None:
            st["park0"] = drv.state()["ang"]
        goal = np.radians(20.0)
        a = st["park0"] + np.clip(goal - st["park0"], -SWEEP_RATE * tau, SWEEP_RATE * tau)
        drv.gains(ang=(PROBE_KE_ANG, PROBE_KD_ANG))
        drv.target(ang=a)

    plan = [("settle", 0.3, settle), ("breakaway", tau_max / TAU_RAMP + 0.5, ramp),
            ("sweep", t_sweep + 0.3, sweep_fn), ("release", 0.8, release)]
    if kind == "snap":
        plan += [("park", np.radians(150.0) / SWEEP_RATE + 0.6, park), ("let_go", 1.0, release)]
    rows = _segments(rig, h, plan)
    sw = [r for r in rows if r["phase"] == "sweep"]
    ang = np.array([r["ang"] for r in sw])
    tau = np.array([r["tau"] for r in sw])
    a0 = ang[0]
    moving = ((ang - a0) > np.radians(5.0)) & ((ang - a0) < sweep - np.radians(2.0))
    rel = [r for r in rows if r["phase"] == "release"]
    drift = float(np.degrees(abs(rel[-1]["ang"] - rel[0]["ang"])))
    pads = np.array([r["pads_mm"] for r in sw])
    tau_break = st["tau_break"]
    res = {"breakaway_torque_Nm": round(float(tau_break), 4),
           "sweep_deg": round(float(np.degrees(ang[-1] - a0)), 2),
           "turning_torque_mean_Nm": round(float(tau[moving].mean()), 4) if moving.any() else None,
           "turning_torque_min_max_Nm": [round(float(tau[moving].min()), 4), round(float(tau[moving].max()), 4)]
           if moving.any() else None,
           "drift_after_release_deg": round(drift, 3),
           "pad_deflection_mm_at_rest": [round(x, 3) for x in rows[0]["pads_mm"]],
           "pad_deflection_mm_sweep_min_max": [round(float(pads.min()), 3), round(float(pads.max()), 3)]}
    if kind == "standard":
        # ripple: 5-95 % spread over the median (single-frame contact re-finds excluded)
        tm_ = tau[moving]
        ripple = float((np.percentile(tm_, 95) - np.percentile(tm_, 5)) / max(abs(np.median(tm_)), 1e-6)) \
            if moving.any() else float("nan")
        res["turning_torque_median_Nm"] = round(float(np.median(tm_)), 4) if moving.any() else None
        res["turning_torque_ripple"] = round(ripple, 3)
        ok = (np.isfinite(tau_break) and 0.05 < tau_break < 3.0 and res["sweep_deg"] > 85.0 and drift < 1.0)
    else:
        # detents: minima of the mean pad deflection along the sweep, peaks between them
        pm = pads.mean(axis=1)
        adeg = np.degrees(ang - a0)
        lo, hi = pm.min(), pm.max()
        mins = [float(adeg[i]) for i in range(1, len(pm) - 1)
                if pm[i] <= pm[i - 1] and pm[i] <= pm[i + 1] and pm[i] < lo + 0.15 * (hi - lo)]
        maxs = [float(adeg[i]) for i in range(1, len(pm) - 1)
                if pm[i] >= pm[i - 1] and pm[i] >= pm[i + 1] and pm[i] > hi - 0.15 * (hi - lo)]
        res["detents_deg"] = [round(m, 1) for m in _cluster(mins)]
        res["detent_peaks_deg"] = [round(m, 1) for m in _cluster(maxs)]
        sp = np.diff(res["detents_deg"]) if len(res["detents_deg"]) > 1 else []
        res["detent_spacing_deg"] = round(float(np.mean(sp)), 2) if len(sp) else None
        res["peak_turning_torque_Nm"] = round(float(np.abs(tau[moving]).max()), 4) if moving.any() else None
        lg = [r for r in rows if r["phase"] == "let_go"]
        res["released_20deg_off_detent_ends_at_deg"] = round(float(np.degrees(lg[-1]["ang"])), 2)
        ok = (np.isfinite(tau_break) and tau_break > 0.02 and res["detent_spacing_deg"] is not None
              and abs(res["detent_spacing_deg"] - 60.0) < 4.0 and res["sweep_deg"] > 140.0)
    res["pass"] = bool(ok)
    res["_rows"] = rows
    return res


def _cluster(xs, tol=8.0):
    out = []
    for x in sorted(xs):
        if out and x - out[-1][-1] < tol:
            out[-1].append(x)
        else:
            out.append([x])
    return [float(np.mean(c)) for c in out]


def _probe_push(rig, h) -> dict:
    """Push-down: blocked unpressed turn, press force curve, pressed turn, stays down, pops up at 0."""
    stroke = h.info["press_stroke_mm"] * MM
    engage = h.info["gate_engagement_mm"] * MM
    try_ang = np.radians(15.0)
    turn = np.radians(90.0)

    def settle(drv, tau):
        drv.gains(ang=(0.0, KNOB_ANG_DAMPING), lin=(0.0, 0.0))
        drv.torque(0.0, 0.0)

    def try_turn(drv, tau):           # compliant drive toward 15 deg without pressing: the gate must block
        drv.gains(ang=(4.0, 0.05), lin=(0.0, 0.0))
        drv.target(ang=min(try_ang, try_ang * tau / 0.5))

    def back(drv, tau):
        drv.gains(ang=(4.0, 0.05), lin=(0.0, 0.0))
        drv.target(ang=0.0)

    def press(drv, tau):              # press force ramp (quasi-static): the spring curve, then the stop
        drv.gains(ang=(4.0, 0.05), lin=(0.0, PROBE_KD_LIN))
        drv.target(ang=0.0)
        drv.torque(0.0, -min(PRESS_F_MAX, PRESS_F_RATE * tau))

    def turn_fn(drv, tau):            # hold the press force, turn to 90 deg
        drv.gains(ang=(PROBE_KE_ANG, PROBE_KD_ANG), lin=(0.0, PROBE_KD_LIN))
        drv.target(ang=min(turn, SWEEP_RATE * tau))
        drv.torque(0.0, -PRESS_F_MAX)

    def let_go(drv, tau):             # release both: the springs push the knob up against tooth and peg
        drv.gains(ang=(0.0, KNOB_ANG_DAMPING), lin=(0.0, 0.0))
        drv.torque(0.0, 0.0)

    def return_fn(drv, tau):          # turn back to 0 without pressing: pops up at the gate
        drv.gains(ang=(PROBE_KE_ANG, PROBE_KD_ANG), lin=(0.0, 0.0))
        drv.target(ang=max(0.0, turn - SWEEP_RATE * tau))

    plan = [("settle", 0.3, settle), ("try", 1.0, try_turn), ("back", 0.4, back),
            ("press", PRESS_F_MAX / PRESS_F_RATE + 0.4, press), ("turn", turn / SWEEP_RATE + 0.3, turn_fn),
            ("let_go", 1.0, let_go), ("return", turn / SWEEP_RATE + 0.6, return_fn), ("rest", 0.5, settle)]
    rows = _segments(rig, h, plan)
    ph = lambda n: [r for r in rows if r["phase"] == n]     # noqa: E731
    tr = ph("try")
    blocked = float(np.degrees(max(abs(r["ang"]) for r in tr)))
    tau_try = float(max(abs(r["tau"]) for r in tr))
    pr = ph("press")
    z = np.array([r["z"] for r in pr])
    fz = np.array([-r["fz"] for r in pr])                     # downward press force applied to the knob

    def force_at(d):                                          # first time the press reaches depth d
        i = np.flatnonzero(-z >= d)
        return round(float(fz[i[0]]), 2) if len(i) else None
    curve = {f"{d / MM:.2f}mm": force_at(d) for d in (0.5 * MM, 1.0 * MM, 2.0 * MM, engage, stroke - 0.05 * MM)}
    tn = ph("turn")
    turned = float(np.degrees(tn[-1]["ang"]))
    tau_turn = [abs(r["tau"]) for r in tn if np.radians(10) < r["ang"] < turn - np.radians(5)]
    lg = ph("let_go")
    held_z = float(lg[-1]["z"])
    held_ang = float(np.degrees(lg[-1]["ang"]))
    rest = ph("rest")
    z_end = float(rest[-1]["z"])
    ang_end = float(np.degrees(rest[-1]["ang"]))
    res = {"unpressed_try": {"drive_target_deg": 15.0, "rotation_deg": round(blocked, 2),
                             "torque_applied_Nm": round(tau_try, 3)},
           "press_force_N": curve,
           "press_depth_reached_mm": round(float(-z.min() / MM), 3),
           "pressed_turn_deg": round(turned, 2),
           "pressed_turning_torque_Nm": [round(float(np.mean(tau_turn)), 4), round(float(np.max(tau_turn)), 4)]
           if tau_turn else None,
           "released_at_90": {"press_mm": round(-held_z / MM, 3), "angle_deg": round(held_ang, 2)},
           "returned_to_0": {"press_mm": round(-z_end / MM, 3), "angle_deg": round(ang_end, 2)}}
    ok = (blocked < 3.0 and turned > 85.0 and -held_z > engage - 0.6 * MM and abs(held_ang - 90.0) < 3.0
          and -z_end < 0.3 * MM and abs(ang_end) < 3.0)
    res["pass"] = bool(ok)
    res["_rows"] = rows
    return res


# ── provenance: the printed-spring stiffness estimates (not used at build time) ──────────────────────────────────────
def _frame_solve(nodes, elems, bc):
    """2D Euler-Bernoulli frame: nodes (n, 2) m; elems (i, j, A, I, E); bc {dof: value}.  Returns (u, K u)."""
    n = len(nodes)
    K = np.zeros((3 * n, 3 * n))
    for i, j, A, I, Em in elems:
        d = nodes[j] - nodes[i]
        L = float(np.hypot(*d))
        c, s = d / L
        a, b, e = 12 * Em * I / L ** 3, 6 * Em * I / L ** 2, Em * A / L
        k = np.array([[e, 0, 0, -e, 0, 0], [0, a, b, 0, -a, b], [0, b, 4 * Em * I / L, 0, -b, 2 * Em * I / L],
                      [-e, 0, 0, e, 0, 0], [0, -a, -b, 0, a, -b], [0, b, 2 * Em * I / L, 0, -b, 4 * Em * I / L]])
        R = np.zeros((6, 6))
        R[:3, :3] = R[3:, 3:] = [[c, s, 0], [-s, c, 0], [0, 0, 1]]
        idx = [3 * i, 3 * i + 1, 3 * i + 2, 3 * j, 3 * j + 1, 3 * j + 2]
        K[np.ix_(idx, idx)] += R.T @ k @ R
    u = np.zeros(3 * n)
    fixed = np.array(sorted(bc))
    free = np.setdiff1d(np.arange(3 * n), fixed)
    u[fixed] = [bc[d] for d in fixed]
    u[free] = np.linalg.solve(K[np.ix_(free, free)], -K[np.ix_(free, fixed)] @ u[fixed])
    return u, K @ u


def _polyline(pts, per_mm=4):
    out = [np.asarray(pts[0], float)]
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        m = max(2, int(np.ceil(np.hypot(*(b - a)) / MM * per_mm)))
        out += [a + (b - a) * k / m for k in range(1, m + 1)]
    return out


def beam_estimates(E: float = E_PLA, nu: float = NU_PLA) -> dict:
    """Spring rates of the printed dial springs from beam theory on the CAD centrelines (N/m).

    k_pad: one standard/snap pad, two mirror serpentines (centrelines measured on the dial_spring section at
    z 14 mm; beams 0.8 mm wide in plane, 10 mm deep), the pad guided (no rotation) by symmetry, the ring clamped.
    k_push_half: one accordion half of the push-down spring in its y-z plane (leaves 0.8 mm thick, each a plate
    strip as wide as the D-shape chord sqrt(32^2 - y^2) - 1 mm at its y; walls at y 10 / 20 / 30 mm with their
    measured widths), both +-y sides, top plate guided.  Plate bending: E' = E / (1 - nu^2)."""
    Ep = E / (1.0 - nu ** 2)
    t, b = 0.8 * MM, 10.0 * MM
    pts = _polyline([(5.2 * MM, 17.98 * MM), (5.2 * MM, 19.99 * MM), (8.2 * MM, 19.99 * MM)])
    th = np.linspace(-np.pi / 2, np.pi / 2, 24)[1:]
    pts += [np.array([8.2 * MM + MM * np.cos(a), 20.99 * MM + MM * np.sin(a)]) for a in th]          # U-bend 1
    pts += _polyline([(8.2 * MM, 21.99 * MM), (2.2 * MM, 21.99 * MM)])[1:]
    th = np.linspace(-np.pi / 2, -3 * np.pi / 2, 24)[1:]
    pts += [np.array([2.2 * MM + 0.995 * MM * np.cos(a), 22.985 * MM + 0.995 * MM * np.sin(a)]) for a in th]
    pts += _polyline([(2.2 * MM, 23.98 * MM), (5.2 * MM, 23.98 * MM), (5.2 * MM, 25.98 * MM)])[1:]
    nodes = np.array(pts)
    elems = [(i, i + 1, b * t, b * t ** 3 / 12, Ep) for i in range(len(nodes) - 1)]
    last = len(nodes) - 1
    d = 1e-4
    _, f = _frame_solve(nodes, elems, {0: 0.0, 1: d, 2: 0.0, 3 * last: 0.0, 3 * last + 1: 0.0, 3 * last + 2: 0.0})
    k_pad = 2.0 * f[1] / d

    def chord(y):
        return max(np.sqrt((32 * MM) ** 2 - y ** 2) - MM, 1e-4)

    segs = [((20, 5), (20, 8), 24.0), ((20, 8), (30, 8), None), ((30, 8), (30, 11), 11.0), ((30, 11), (10, 11), None),
            ((10, 11), (10, 14), 29.4), ((10, 14), (20, 14), None), ((20, 14), (20, 17), 24.0)]
    nodes, elems = [], []
    for a, c, width in segs:
        p = _polyline([(a[0] * MM, a[1] * MM), (c[0] * MM, c[1] * MM)], per_mm=6)
        if not nodes:
            nodes.append(p[0])
        for q in p[1:]:
            i = len(nodes) - 1
            nodes.append(q)
            w = width * MM if width else chord(0.5 * (nodes[i][0] + q[0]))
            elems.append((i, i + 1, w * t, w * t ** 3 / 12, Ep))
    nodes = np.array(nodes)
    last = len(nodes) - 1
    _, f = _frame_solve(nodes, elems, {0: 0.0, 1: 0.0, 2: 0.0, 3 * last: 0.0, 3 * last + 1: -d, 3 * last + 2: 0.0})
    return {"k_pad_N_per_m": float(k_pad), "k_push_half_N_per_m": float(2.0 * -f[3 * last + 1] / d)}


def pad_fem(h_mm: float = 0.05, E: float = E_PLA, nu: float = NU_PLA) -> float:
    """Radial stiffness (N/m) of one standard-dial pad from a plane-strain FEM of the CAD section itself.

    Bilinear (Q4) pixel elements of size h_mm over the spring's section at z 14 mm, restricted to the top pad and
    its two serpentines; the pad face is pushed out radially (tangential motion held by symmetry), the ring is
    clamped beyond r = 26.3 mm; depth 10 mm.  0.1 / 0.05 mm pixels give 28.7 / 29.0 kN/m at E = 3 GPa."""
    import scipy.sparse as sp
    import scipy.sparse.linalg as spla
    from matplotlib.path import Path

    from force_bench.kit import load_kit
    S = _section_mm(load_kit()["dial"].part("dial_spring").mesh(), 14.0)
    poly = max(getattr(S, "geoms", [S]), key=lambda g: g.area)
    h = float(h_mm)
    x0, y0 = -11.0, 15.0
    nx, ny = int(round(22.0 / h)), int(round(11.8 / h))
    X, Y = np.meshgrid(x0 + (np.arange(nx) + 0.5) * h, y0 + (np.arange(ny) + 0.5) * h, indexing="ij")
    pts = np.c_[X.ravel(), Y.ravel()]
    inside = Path(np.asarray(poly.exterior.coords)).contains_points(pts)
    for ring in poly.interiors:
        inside &= ~Path(np.asarray(ring.coords)).contains_points(pts)
    inside = inside.reshape(nx, ny) & ((np.abs(np.degrees(np.arctan2(Y, X)) - 90.0) <= 29.5) | (Y > 25.5))
    el = np.argwhere(inside)
    conn = np.stack([el[:, 0] * (ny + 1) + el[:, 1], (el[:, 0] + 1) * (ny + 1) + el[:, 1],
                     (el[:, 0] + 1) * (ny + 1) + el[:, 1] + 1, el[:, 0] * (ny + 1) + el[:, 1] + 1], axis=1)
    used = np.unique(conn)
    remap = -np.ones((nx + 1) * (ny + 1), int)
    remap[used] = np.arange(len(used))
    conn = remap[conn]
    xy = np.c_[x0 + (used // (ny + 1)) * h, y0 + (used % (ny + 1)) * h]
    E_mpa = E * 1e-6
    lam, mu = E_mpa * nu / ((1 + nu) * (1 - 2 * nu)), E_mpa / (2 * (1 + nu))
    Dm = np.array([[lam + 2 * mu, lam, 0], [lam, lam + 2 * mu, 0], [0, 0, mu]])
    ke = np.zeros((8, 8))
    xi_n, eta_n = np.array([-1, 1, 1, -1]), np.array([-1, -1, 1, 1])
    for xi in (-3 ** -0.5, 3 ** -0.5):
        for eta in (-3 ** -0.5, 3 ** -0.5):
            dx, dy = 0.5 * xi_n * (1 + eta * eta_n) / h, 0.5 * eta_n * (1 + xi * xi_n) / h
            B = np.zeros((3, 8))
            B[0, 0::2], B[1, 1::2], B[2, 0::2], B[2, 1::2] = dx, dy, dy, dx
            ke += B.T @ Dm @ B * (h / 2) ** 2 * 10.0                        # 10 mm deep; N/mm units
    dofs = np.stack([2 * conn, 2 * conn + 1], axis=2).reshape(len(conn), 8)
    K = sp.csr_matrix((np.tile(ke.ravel(), len(conn)), (np.repeat(dofs, 8, axis=1).ravel(),
                                                       np.tile(dofs, (1, 8)).ravel())), shape=(2 * len(used),) * 2)
    face = xy[:, 1] < 15.25 + 0.6 * h
    clamp = xy[:, 1] > 26.3
    u = np.zeros(2 * len(used))
    fixed = np.r_[2 * np.flatnonzero(face), 2 * np.flatnonzero(face) + 1, 2 * np.flatnonzero(clamp),
                  2 * np.flatnonzero(clamp) + 1]
    u[2 * np.flatnonzero(face) + 1] = 0.1                                   # mm
    free = np.setdiff1d(np.arange(2 * len(used)), fixed)
    u[free] = spla.spsolve(K[free][:, free].tocsc(), -K[free][:, fixed] @ u[fixed])
    return float((K @ u)[2 * np.flatnonzero(face) + 1].sum() / 0.1 * 1e3)
