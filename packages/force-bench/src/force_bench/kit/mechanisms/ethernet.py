"""Ethernet plug and slot: the printed 5-port shell with a slot insert in every port, and the printed plug whose latch
tab is a hinged body on a spring drive at the printed flexure's estimated stiffness.

GEOMETRY of the final printable CAD (``assets/kit/ethernet-plug-and-slot``: mm, the assembly frame of the site's
``ethernet_plug_and_slot.obj``; :func:`measure` and :func:`latch_kinematics` re-derive every number below):

  shell    98 x 32.5 x 32.5 mm, lying on its 98 x 32.5 face, long axis x, five ports facing +y (mouths in the front
           face, y = 16.25).  Each port is a 12.0 x 17.9 mm pocket (x +-6.0 around x = -30, -15, 0, 15, 30;
           z 8.1..26.0) running back to y = -8.75.  The kit ships ONE insert STL (the x = 0 one); the site's OBJ and
           the real-world video have an insert in every port, so the model places five copies.
  insert   11.6 x 25 x 17.5 mm (0.2 mm of play in its pocket), a C on its side: FLOOR (top z = 11.6, a 0.5 mm round
           at the mouth), BACK WALL (y = 1.25: the plug's stop) and ROOF.  The roof is flat at z = 20.3 for
           y <= 6.65, then a 15 deg ramp up to z = 22.86 at the mouth.  Over |x| >= 3.5 it carries the two LIPS
           (|x| 3.5..5.8, y 14.4..16.25, bottom z = 20.3): their inner faces lean 15 deg (y 14.41 at z 20.4 ->
           13.81 at z 22.2) and are the catch.  The channel between the lips (|x| < 3.5) has no lip.  The pocket's
           walls (x = +-6.0) are the cavity's side walls.
  plug     11.0 wide (x +-5.5 about x = -15 in the CAD pose).  Nose block z 11.6..18.6 from y 23.25 (the nose) to
           38.6; a deeper rear block z 10.1..18.6 over y 38.6..46.25 (the grasp); a 7 x 3 mm tail to y 56.25.
           The LATCH TAB springs from the nose's top edge: a 1.00 mm thick plate (1.064 mm measured along z) at
           20.0 deg to the plug axis, its underside leaving the body's top face (z = 18.6) at y = 26.17 (the
           notch).  It is 11.0 mm wide up to the SHOULDER (its end face at y = 34.99, top corner at z = 22.87),
           then 6.0 mm wide (x +-3.0) to its free end at y = 42.95 (top z = 25.65).

CLEARANCES AND CATCH.  The plug body has 0.5 mm of play per side in x and 1.7 mm above the nose (where the tab
runs).  It is fully inserted after 22.0 mm of travel from its CAD pose: the nose on the back wall, 15.0 mm inside
the mouth; the rear block's front face is then 0.35 mm proud of the mouth.  The tab's wide part runs under the lips
(x overlap 3.5..5.5, 2 mm per side), its narrow part through the channel.  Going in, the lips meet the tab 9.9 mm
before the seat; their bottom corners ride up the tab's sloped top face and press it toward the body.  The shoulder
passes under them once the tab is pressed 15.2 deg (its top corner 2.6 mm down), 3 to 2 mm before the seat; then it
rides down the lips' leaning inner faces (which pull the plug in) and 0.85 mm before the seat springs back behind
them.  Seated, the tab rests at 5.0 deg against the roof ramp.  Pulled, the plug backs off those 0.85 mm (the tab at
3.9 deg) and the shoulder's rounded end face bears on the lips' faces 0.9 mm above their bottom edge (the hook).
The bearing force's line passes 0.3 mm from the hinge, so a 30 N pull presses the tab with only 0.01 N.m against
the spring's 0.05 N.m, and friction on the faces adds to the hold.  Releasing needs the tab pressed past 15.2 deg
again; it lies flat on the body at 19.8 deg (its stop).

MODEL.  The shell and the five inserts are static, with the kit's fine SDF contact (sim.yaml).  The plug is split
at the tab's root section (the plane through the notch normal to the tab): the BODY (everything below the body's
top face, plus the fused root corner in front of that plane) is one rigid body; the TAB beyond it is a second body
on a revolute joint about x through the root section's mid-thickness (y 26.00, z 19.07), positive = pressing the
tab toward the body.  The printed flexure is not simulated as a deforming body (docs/assets.md rule 2, "printed
springs become joint drives"): the joint's drive is the flexure's stiffness, its target the CAD (relaxed) pose, and
its upper limit the stop (hard MuJoCo limits: ``build`` sets ``rig.hard_joint_limits``).  The tab collides with
everything except the body it is moulded to (filtered).

PARAMETERS (``build(..., ke=, kd=)`` overrides), with their provenance:

    hinge stiffness   0.72 N.m/rad   ESTIMATE: plane-stress finite elements of the tab's side section, E = 3 GPa
    hinge damping     5e-4 N.m.s/rad about 0.5 of critical with the conditioning inertia
    hinge armature    3e-7 kg.m^2    conditioning (the printed tab: 1.6e-8); omega*dt = 0.8 at 32 substeps
    stop              19.8 deg       the tab's underside flat on the body (CAD), a hard joint limit
    plug mass         15 g           conditioning (print estimate 1.8 g; docs/physics_notes.md pitfall 2.7)
    mu                0.4            sim.yaml (printed PLA on PLA, as the other printed connectors)

The stiffness comes from :func:`fem_tab_stiffness`: the x = centre section of the plug (the tab plus 1 mm of body,
clamped there), Q4 elements on a 0.025 mm raster, each element as deep as the tab is wide at that point (11 / 6 mm),
a force normal to the tab on its top face just in front of the shoulder (y 34.6): 8.3 N/mm there, mesh-converged to
0.5 %.  With the hinge at the root that is K = k r^2 = 0.72 N.m/rad (r = 9.33 mm, the force's lever about the
hinge).  Plane strain (the tab is 11x wider than thick) gives 0.82; E = 2 / 3.5 GPa (the range for printed PLA)
0.48 / 0.84, and +-0.05 mm of printed thickness about +-15 %.  Euler-Bernoulli theory for the 11 x 1.0 mm cantilever
clamped at its root section gives 10.2 N/mm at the same point: it takes the root as rigid; the FEM's root compliance
is worth 0.7 mm of extra length.  NO force measurement of this plug exists.
A rigid tab on one hinge reproduces the force at the shoulder, where the catch and the largest deflection are; it
is 44 % stiffer than the printed tab at the mouth (y 38.25) and 85 % stiffer at the free end, where the narrow part
bends too.  Two consequences of the estimate are worth stating: at the 15.2 deg needed to pass the lips, linear
elasticity puts ~110 MPa in the printed tab's root, above printed PLA's flexural strength (60-100 MPa), so the real
tab likely yields, or the real insertion force is lower than this model's; and by the same linear model, pressing the
printed tab at its free end bottoms it on the body when the shoulder has moved 1.9 of the 2.5 mm it needs (pressing
at the mouth: 2.4 mm), so the printed latch is released by a firm press right at the mouth.  The model's rigid tab
releases wherever it is pressed.

PROBE (``force-bench kit check ethernet-plug-and-slot``): the default build seats a free plug in port 3 (it must
come to rest).  The probe adds a second plug on a prismatic probe joint in front of port 1, nose 2 mm out of the
mouth: it is driven in at 5 mm/s to the seat (force against depth -> ``probe_force.csv``), pulled with 10 N (the
light pull: it must rise less than 1.5 mm), pulled up to 30 N (the hold, reported), let back to the seat, its tab
pressed to the stop by a hinge torque (0.32 N.m), pulled with 10 N again (it must come out by 5 mm), un-pressed
gradually and driven back in (it must latch again).  Measured: 0.7 N while the body slides in, a 14 N peak once the
lips press the tab, down to -19 N as the shoulder rides down the lips' faces, the snap 0.86 mm before the seat; the
tab at 14.8 deg on the way in and 4.95 deg seated; 10 N held at 0.93 mm of rise, 30 N at 0.97 mm; the pressed plug
out against 2.8 N; re-latched.  The free plug in port 3 settles on its catch, 0.77 mm out, tab 3.5 deg.
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, sdf_builds_deferred, set_body_mass
from force_bench.kit.build import _setting, part_mass, pose_matrix, to_wp
from force_bench.kit.catalog import sdf_resolution

MM = 1e-3

# ── geometry (kit CAD frame, m; :func:`measure` re-derives it) ──────────────────────────────────────────────────────
PORT_X = (-0.030, -0.015, 0.0, 0.015, 0.030)    # ports 1..5 (the OBJ's five inserts; the kit's STL is the x = 0 one)
CAD_PORT = 2                                     # the plug's CAD pose stands in front of port 2
PLUG_X = -0.015                                  # plug centre (x) in its CAD pose
SEAT_DY = -0.022                                 # CAD pose -> fully inserted: the nose on the insert's back wall
MOUTH_Y = 0.01625                                # shell front face = insert front = the ports' mouths
BACK_WALL_Y = 0.00125
NOSE_Y = 0.02325                                 # plug nose, CAD pose
FLOOR_Z = 0.0116                                 # insert floor top = plug nose bottom
BODY_TOP_Z = 0.0186                              # plug body top face (the tab's stop)
REAR_Y = (0.0386, 0.04625)                       # the rear (grasp) block, z 10.1..18.6
REAR_Z = (0.0101, 0.0186)
PLUG_HALF_W = 0.0055
POCKET_HALF_W = 0.0060
TAB_ALPHA = float(np.radians(20.0))              # the tab's faces rise 20.0 deg from the plug axis
TAB_SLOPE = float(np.tan(TAB_ALPHA))
TAB_T = 1.064e-3 * np.cos(TAB_ALPHA)             # 1.000 mm normal to the tab (1.064 mm measured along z)
TAB_NOTCH = (0.026174, BODY_TOP_Z)               # (y, z): the tab's underside leaves the body's top face
PIVOT_YZ = (TAB_NOTCH[0] - 0.5 * TAB_T * np.sin(TAB_ALPHA),     # the root section's mid-thickness point: the hinge
            TAB_NOTCH[1] + 0.5 * TAB_T * np.cos(TAB_ALPHA))
SHOULDER_Y = 0.03499                             # end face of the tab's 11 mm wide part (the catch)
CUT_Z = BODY_TOP_Z + 1e-5                        # body | tab split plane (just above the top face)

# ── the model ───────────────────────────────────────────────────────────────────────────────────────────────────────
KE_TAB = 0.72               # N.m/rad: plane-stress FEM of the printed tab, E = 3 GPa (estimate, see docstring)
KE_SOURCE = "estimate: plane-stress FEM of the printed tab (E = 3 GPa, nu = 0.35); not measured"
KD_TAB = 5.0e-4             # N.m.s/rad, ~0.5 of critical with the conditioning inertia
ARMATURE = 3.0e-7           # kg.m^2 on the hinge dof (conditioning: omega*dt ~ 0.8 at 32 substeps)
TAB_LIMIT = (-0.10, 0.3459)  # rad: outward / the stop (19.82 deg: the tab's underside flat on the body top)
TAB_EFFORT = 1.0            # N.m drive clamp (the spring at the stop is 0.25 N.m)
PLUG_MASS = 0.015           # kg, conditioning mass of the plug body (print estimate ~1.8 g; pitfall 2.7)
SEAT_TAB_RAD = np.radians(4.99)  # the tab's rest angle in the seat (the roof ramp presses it), :func:`latch_kinematics`
PASS_RAD = np.radians(15.3)      # the deflection that takes the shoulder under the lips (3D check: 15.2-15.4 deg)
PLUG_EDGE = 0.0012          # m, subdivision of the plug body (contact samples; physics_notes pitfall 3.2)
TAB_EDGE = 0.0005


def port_index(port: int) -> int:
    if not 1 <= int(port) <= len(PORT_X):
        raise ValueError(f"ethernet slot: port {port} (ports are 1..{len(PORT_X)})")
    return int(port) - 1


def seat_transform(port: int, lift: float = 0.0) -> np.ndarray:
    """Kit frame <- plug CAD frame for the plug in ``port`` (1..5), fully inserted, then withdrawn by ``lift`` (m)
    along the port axis."""
    return pose_matrix(xyz=(PORT_X[port_index(port)] - PLUG_X, SEAT_DY + lift, 0.0))


def port_axis(pose: np.ndarray) -> np.ndarray:
    """World direction OUT of the ports (the kit frame's +y)."""
    return np.asarray(pose, float)[:3, :3] @ np.array([0.0, 1.0, 0.0])


def mouth_point(pose: np.ndarray, port: int) -> np.ndarray:
    """World point at the centre of ``port``'s mouth (plug centre line, on the front face)."""
    p = np.array([PORT_X[port_index(port)], MOUTH_Y, 0.5 * (FLOOR_Z + BODY_TOP_Z)])
    return (np.asarray(pose, float) @ np.r_[p, 1.0])[:3]


# ── the plug, split at the tab's root ───────────────────────────────────────────────────────────────────────────────
def split_plug(plug: trimesh.Trimesh) -> dict[str, trimesh.Trimesh]:
    """The plug (CAD frame, m) as {body, root, tab}: watertight crops.  ``body`` is everything below the top face,
    ``root`` the corner of the nose the tab is moulded into (rigid with the body), ``tab`` the latch beyond the
    root section (the plane through the notch normal to the tab)."""
    from force_bench.kit.mechanisms.usb_slot import _slice_capped
    e_tab = np.array([0.0, np.cos(TAB_ALPHA), np.sin(TAB_ALPHA)])
    notch = np.array([0.0, TAB_NOTCH[0], TAB_NOTCH[1]])
    upper = _slice_capped(plug, [0.0, 0.0, CUT_Z], [0.0, 0.0, 1.0])
    out = {"body": _slice_capped(plug, [0.0, 0.0, CUT_Z], [0.0, 0.0, -1.0]),
           "root": _slice_capped(upper, notch, -e_tab),
           "tab": _slice_capped(upper, notch, e_tab)}
    for k, m in out.items():
        assert m.is_watertight, f"ethernet plug crop {k} is not watertight"
    return out


def pivot_cad() -> np.ndarray:
    return np.array([PLUG_X, PIVOT_YZ[0], PIVOT_YZ[1]])


def tab_angle(body_q: np.ndarray, plug_i: int, tab_i: int) -> float:
    """Hinge angle (rad) from the two bodies' poses, positive = pressed toward the body (reads the same whatever the
    joint bookkeeping)."""
    qp, qt = body_q[plug_i], body_q[tab_i]
    qa = wp.quat(float(qp[3]), float(qp[4]), float(qp[5]), float(qp[6]))
    qb = wp.quat(float(qt[3]), float(qt[4]), float(qt[5]), float(qt[6]))
    rel = wp.mul(wp.quat_inverse(qa), qb)
    return 2.0 * float(np.arctan2(-rel[0], rel[3]))      # rotation about -x of the plug frame


# ── build ───────────────────────────────────────────────────────────────────────────────────────────────────────────
def _mesh_shape(rig, body: int, asset, part_name: str, tm: trimesh.Trimesh, body_from_cad: np.ndarray, label: str,
                rgb, mu: float, collide: bool = True, mesh=None) -> int:
    """A mesh shape with the asset's (part's) contact settings from sim.yaml, labelled ``label``; ``mesh`` reuses a
    built newton.Mesh (with its SDF) whose vertices are ``tm`` centred on its AABB centre."""
    c = 0.5 * (tm.bounds[0] + tm.bounds[1])
    if mesh is None:
        cell = float(_setting(asset, part_name, "sdf_cell_mm", 0.4)) * MM
        res = _setting(asset, part_name, "sdf_res") or sdf_resolution(
            tm, cell=cell, lo=int(_setting(asset, part_name, "sdf_min_res", 64)),
            hi=int(_setting(asset, part_name, "sdf_max_res", 384)))
        mesh, c = make_newton_mesh(tm, sdf=collide, sdf_resolution=int(res),
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
    sid = rig.builder.add_shape_mesh(body, xform=to_wp(body_from_cad @ pose_matrix(xyz=c)), mesh=mesh, cfg=cfg,
                                     color=rgb, label=label)
    rig.shapes.setdefault(label.split("/")[1] if label.startswith("kit/") else label, []).append(sid)
    return sid


def add_plug(rig, asset, h, world_from_cad: np.ndarray, label: str, root: str = "free", tab_q: float = 0.0,
             ke: float = KE_TAB, kd: float = KD_TAB, mu: float = 0.4, mass: float = PLUG_MASS,
             probe_drive: dict | None = None, probe_axis: np.ndarray | None = None) -> dict:
    """One plug (body + hinged tab) whose CAD frame sits at ``world_from_cad``.

    root: "free" (a free joint) or "probe" (a prismatic joint along ``probe_axis``, the world insertion direction,
    with ``probe_drive`` = dict(ke, kd, effort, lower, upper)).  The tab starts at hinge angle ``tab_q`` (rad, +
    pressed).  Bodies ``<label>`` and ``<label>/tab``; returns the ids."""
    p = asset.part("ethernet_plug")
    pieces = _plug_pieces()
    B = rig.builder
    c_body = 0.5 * (pieces["body"].bounds[0] + pieces["body"].bounds[1])
    world_from_body = world_from_cad @ pose_matrix(xyz=c_body)
    body = B.add_link(xform=to_wp(world_from_body), label=label)
    body_from_cad = pose_matrix(xyz=-c_body)
    s_body = _mesh_shape(rig, body, asset, p.name, pieces["body_sub"], body_from_cad, f"kit/{label}/plug_body",
                         p.rgb, mu, mesh=pieces["body_mesh"])
    s_root = _mesh_shape(rig, body, asset, p.name, pieces["root"], body_from_cad, f"kit/{label}/plug_root",
                         p.rgb, mu, mesh=pieces["root_mesh"])
    # the tab's frame: at the hinge, rotated by tab_q about -x (the CAD frame's orientation at tab_q = 0)
    piv = pivot_cad()
    R_tab = trimesh.transformations.rotation_matrix(-float(tab_q), (1.0, 0.0, 0.0))
    world_from_pivot = world_from_cad @ pose_matrix(xyz=piv)
    world_from_tab = world_from_pivot @ R_tab
    tab = B.add_link(xform=to_wp(world_from_tab), label=f"{label}/tab")
    tab_from_cad = pose_matrix(xyz=-piv)
    s_tab = _mesh_shape(rig, tab, asset, p.name, pieces["tab_sub"], tab_from_cad, f"kit/{label}/plug_tab", p.rgb, mu,
                        mesh=pieces["tab_mesh"])
    for s in (s_body, s_root):
        B.add_shape_collision_filter_pair(s, s_tab)             # moulded together: the hinge is the joint
    if root == "free":
        j_root = B.add_joint_free(body)
    elif root == "probe":
        d = dict(ke=2.0e5, kd=300.0, effort=80.0, lower=-0.005, upper=0.03)
        d.update(probe_drive or {})
        ax = np.asarray(probe_axis, float)
        # anchor frame: world-aligned at the body's start position, axis given in world coordinates
        j_root = B.add_joint_prismatic(parent=-1, child=body, axis=tuple(float(v) for v in ax),
                                       parent_xform=wp.transform(wp.vec3(*world_from_body[:3, 3]), wp.quat_identity()),
                                       child_xform=to_wp(np.linalg.inv(world_from_body) @
                                                         pose_matrix(xyz=world_from_body[:3, 3])),
                                       target_pos=0.0, target_ke=float(d["ke"]), target_kd=float(d["kd"]),
                                       limit_lower=float(d["lower"]), limit_upper=float(d["upper"]), label=label)
        B.joint_effort_limit[B.joint_qd_start[j_root]] = float(d["effort"])
    else:
        raise ValueError(f"ethernet plug root {root!r}")
    # hinge: parent frame at the pivot in the body frame, CAD orientation; child frame = the tab's origin rotated
    # back by tab_q, so q = tab_q at the build pose.  Axis -x: positive q presses the tab toward the body.
    body_from_pivot = np.linalg.inv(world_from_body) @ world_from_pivot
    q_start = len(B.joint_q)
    j_tab = B.add_joint_revolute(parent=body, child=tab, axis=(-1.0, 0.0, 0.0),
                                 parent_xform=to_wp(body_from_pivot), child_xform=wp.transform_identity(),
                                 target_pos=0.0, target_ke=float(ke), target_kd=float(kd),
                                 limit_lower=TAB_LIMIT[0], limit_upper=TAB_LIMIT[1], label=f"{label}/tab")
    B.joint_q[q_start] = float(tab_q)                       # eval_fk starts from joint_q (not the body poses)
    dof = B.joint_qd_start[j_tab]
    B.joint_armature[dof] = ARMATURE
    B.joint_effort_limit[dof] = TAB_EFFORT
    B.add_articulation([j_root, j_tab], label=label)
    set_body_mass(B, body, float(mass))
    set_body_mass(B, tab, float(pieces["tab"].volume) * 1240.0)     # solid: thinner than two perimeters
    h.bodies[label], h.bodies[f"{label}/tab"] = body, tab
    h.joints[label], h.joints[f"{label}/tab"] = j_root, j_tab
    h.shapes[f"{label}/plug_body"], h.shapes[f"{label}/plug_root"] = [s_body], [s_root]
    h.shapes[f"{label}/plug_tab"] = [s_tab]
    return {"body": body, "tab": tab, "j_root": j_root, "j_tab": j_tab, "shapes": [s_body, s_root, s_tab],
            "world_from_body": world_from_body, "c_body": c_body}


_PIECES: dict = {}     # SDF bakes deferred (bool) -> the pieces


def _plug_pieces() -> dict:
    """The plug crops (CAD frame), subdivided copies for contact, and their newton meshes (built once; separately
    inside ``defer_sdf_builds``, whose meshes have no SDF, as the USD exporter builds the asset)."""
    deferred = sdf_builds_deferred()
    if deferred in _PIECES:
        return _PIECES[deferred]
    from force_bench.geometry import subdiv_processed as subdiv
    from force_bench.kit import load_kit
    asset = load_kit()["ethernet-plug-and-slot"]
    pieces = split_plug(asset.part("ethernet_plug").mesh())
    out = dict(pieces)
    out["body_sub"] = subdiv(pieces["body"], PLUG_EDGE)
    out["tab_sub"] = subdiv(pieces["tab"], TAB_EDGE)
    for k, tm in (("body_mesh", out["body_sub"]), ("root_mesh", pieces["root"]), ("tab_mesh", out["tab_sub"])):
        cell = float(_setting(asset, "ethernet_plug", "sdf_cell_mm", 0.4)) * MM
        res = sdf_resolution(tm, cell=cell, lo=int(_setting(asset, "ethernet_plug", "sdf_min_res", 64)),
                             hi=int(_setting(asset, "ethernet_plug", "sdf_max_res", 384)))
        out[k], _ = make_newton_mesh(tm, sdf=True, sdf_resolution=int(res),
                                     narrow_band=float(_setting(asset, "ethernet_plug", "narrow_band_mm", 2.0)) * MM,
                                     margin=float(_setting(asset, "ethernet_plug", "margin_mm", 1.2)) * MM)
    _PIECES[deferred] = out
    return out


def build(rig, asset, h, plug: str | None = "seated", port: int = 3, lift: float = 0.0,
          plug_pose: np.ndarray | None = None, tab_q: float | None = None, ke: float | None = None,
          kd: float = KD_TAB, mu: float | None = None, plug_mass: float = PLUG_MASS, ports: int = 5) -> None:
    """The shell, an insert in each of the first ``ports`` ports, and (``plug``):

    "seated"  a free plug fully inserted and latched in ``port`` (the default; tab at its seated rest angle),
    "free"    a free plug at ``plug_pose`` (world pose of its CAD frame), or withdrawn ``lift`` (m) from its seat
              in ``port`` along the port axis, tab relaxed,
    None      no plug.
    """
    from force_bench.kit.build import add_part_shape
    ke = KE_TAB if ke is None else float(ke)
    mu = float(_setting(asset, None, "mu", 0.4)) if mu is None else float(mu)
    pose = np.asarray(h.pose, float)
    h.params = {"ke_N_m_per_rad": ke, "ke_source": KE_SOURCE if ke == KE_TAB else "override", "kd_N_m_s_per_rad": kd,
                "armature_kg_m2": ARMATURE, "tab_limit_deg": [round(float(np.degrees(v)), 2) for v in TAB_LIMIT],
                "plug_mass_kg": plug_mass, "mu": mu}
    # shell + inserts (static)
    shell_p = asset.part("ethernet_slot_shell")
    h.shapes["ethernet_slot_shell"] = [add_part_shape(rig, -1, asset, shell_p, pose, h.label, mu=mu)]
    ins_p = asset.part("ethernet_slot")
    ins_tm = ins_p.mesh()
    ins_mesh = None
    for i in range(int(ports)):
        T_ins = pose @ pose_matrix(xyz=(PORT_X[i] - 0.0, 0.0, 0.0))      # the STL is the x = 0 insert
        if ins_mesh is None:
            cell = float(_setting(asset, ins_p.name, "sdf_cell_mm", 0.4)) * MM
            res = sdf_resolution(ins_tm, cell=cell, lo=int(_setting(asset, ins_p.name, "sdf_min_res", 64)),
                                 hi=int(_setting(asset, ins_p.name, "sdf_max_res", 384)))
            ins_mesh, _ = make_newton_mesh(ins_tm, sdf=True, sdf_resolution=int(res),
                                           narrow_band=float(_setting(asset, ins_p.name, "narrow_band_mm", 2.0)) * MM,
                                           margin=float(_setting(asset, ins_p.name, "margin_mm", 1.2)) * MM)
        sid = _mesh_shape(rig, -1, asset, ins_p.name, ins_tm, T_ins, f"kit/{h.label}/ethernet_slot_port{i + 1}",
                          ins_p.rgb, mu, mesh=ins_mesh)
        h.shapes[f"insert_port{i + 1}"] = [sid]
    h.info = {"ports_x_mm": [round(v * 1e3, 2) for v in PORT_X[:ports]], "seat_travel_mm": -SEAT_DY * 1e3,
              "pivot_yz_mm": [round(v * 1e3, 3) for v in PIVOT_YZ], "seat_tab_deg": round(np.degrees(SEAT_TAB_RAD), 2),
              "pass_tab_deg": round(np.degrees(PASS_RAD), 2), "plug_mass_print_g":
              round(part_mass(asset, asset.part("ethernet_plug")) * 1e3, 2)}
    rig.hard_joint_limits = True                      # the tab's stop is a joint limit: make MuJoCo honour it
    if plug is None:
        return
    if plug == "seated":
        T = pose @ seat_transform(port)
        q0 = SEAT_TAB_RAD if tab_q is None else tab_q
    elif plug == "free":
        T = np.asarray(plug_pose, float) if plug_pose is not None else pose @ seat_transform(port, lift)
        q0 = 0.0 if tab_q is None else tab_q
    else:
        raise ValueError(f"ethernet-plug-and-slot: plug={plug!r} (seated | free | None)")
    add_plug(rig, asset, h, T, f"{h.label}/plug", root="free", tab_q=q0, ke=ke, kd=kd, mu=mu, mass=plug_mass)
    h.info.update({"plug": plug, "port": int(port), "lift_mm": round(float(lift) * 1e3, 3)})


# ── measured geometry ───────────────────────────────────────────────────────────────────────────────────────────────
def _ray(m: trimesh.Trimesh, o, d) -> list[float]:
    """Hit coordinates along the ray's dominant axis, sorted (m)."""
    loc, _, _ = m.ray.intersects_location([o], [d])
    k = int(np.argmax(np.abs(d)))
    return sorted(float(v) for v in loc[:, k]) if len(loc) else []


def _section(m: trimesh.Trimesh, x: float):
    """YZ section of ``m`` at ``x`` as a shapely geometry in (y, z) (m)."""
    from shapely.ops import unary_union
    s = m.section(plane_origin=[x, 0.0, 0.0], plane_normal=[1.0, 0.0, 0.0])
    if s is None:
        from shapely.geometry import Polygon
        return Polygon()
    T = np.eye(4)
    T[:3, :3] = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]])
    p2, _ = s.to_2D(to_2D=T)
    return unary_union(list(p2.polygons_full))


def latch_kinematics(step: float = 0.05e-3) -> dict:
    """The tab's travel along the insertion, in the two YZ profiles that make the problem (the slot is prismatic in x
    over each): the channel (|x| < 3.5 mm: the whole tab against the ramp roof) and the ears (|x| 3.5..5.5: the wide
    part against a lip).  For each insertion depth, the least hinge angle at which the rigid tab clears the slot
    (shapely, exact on the sections).  Returns the profile and its key numbers (degrees, mm)."""
    from shapely import affinity
    from shapely.ops import unary_union
    from force_bench.kit import load_kit
    a = load_kit()["ethernet-plug-and-slot"]
    plug, ins, shell = (a.part(n).mesh() for n in ("ethernet_plug", "ethernet_slot", "ethernet_slot_shell"))
    tab = split_plug(plug)["tab"]
    profiles = []
    for dx in (0.0010, 0.0045):                     # channel, ear (every port has the same section; port 3 is x = 0)
        sl = unary_union([_section(ins, dx), _section(shell, dx)])
        profiles.append((_section(tab, PLUG_X + dx), sl))
    piv = PIVOT_YZ

    def overlap(theta, dy):
        worst = 0.0
        for tb, sl in profiles:
            g = affinity.translate(affinity.rotate(tb, -theta, origin=piv, use_radians=True), dy, 0.0)
            worst = max(worst, g.intersection(sl).area)
        return worst

    def theta_min(dy, tol=1e-12):
        if overlap(0.0, dy) < tol:
            return 0.0
        lo, hi = 0.0, 0.40
        if overlap(hi, dy) >= tol:
            return float("nan")
        for _ in range(22):
            m = 0.5 * (lo + hi)
            lo, hi = (lo, m) if overlap(m, dy) < tol else (m, hi)
        return hi

    # depths: travel from the CAD pose (0) to the seat (SEAT_DY), where the tab first meets the lips onward
    dys = np.arange(-0.010, SEAT_DY - 1e-9, -step)
    dys = np.append(dys, SEAT_DY)
    th = np.array([theta_min(d) for d in dys])
    i_max = int(np.nanargmax(th))
    after = np.nonzero((np.arange(len(th)) > i_max) & (th < 0.5 * th[i_max]))[0]
    i_snap = int(after[0]) if len(after) else len(th) - 1
    first = np.nonzero(th > 1e-4)[0]
    # the stop: the tab's underside meets the body's top face (points > 2 mm from the hinge)
    v = tab.vertices[:, 1:] - np.array(piv)
    far = np.linalg.norm(v, axis=1) > 0.002
    dz_top = BODY_TOP_Z - piv[1]
    stops = []
    for dy_, dz_ in v[far]:
        # pressed by theta a point (dy, dz) about the hinge sits at z = -dy sin(theta) + dz cos(theta)
        #                                                         = r cos(theta + psi), psi = atan2(dy, dz)
        r, psi = np.hypot(dy_, dz_), np.arctan2(dy_, dz_)
        if abs(dz_top / r) <= 1.0:
            a0 = np.arccos(dz_top / r)
            stops += [t for t in (a0 - psi, -a0 - psi, a0 - psi + 2 * np.pi) if 0.0 < t < 0.6]
    stop = float(min(stops)) if stops else float("nan")
    # withdrawal play: out from the seat, the tab follows the rising roof until its shoulder meets the lips
    ws = np.arange(0.0, 0.0016, 0.01e-3)
    tw = np.array([theta_min(SEAT_DY + w) for w in ws])
    rising = np.nonzero(np.diff(tw) > np.radians(0.2))[0]
    play = float(ws[rising[0]]) if len(rising) else float("nan")
    return {"depth_mm": np.round((dys - SEAT_DY) * 1e3, 3), "theta_deg": np.degrees(th),
            "first_contact_mm_from_seat": round(float((dys[first[0]] - SEAT_DY) * 1e3), 2) if len(first) else None,
            "max_deg": round(float(np.degrees(th[i_max])), 2),
            "max_at_mm_from_seat": round(float((dys[i_max] - SEAT_DY) * 1e3), 2),
            "snap_at_mm_from_seat": round(float((dys[i_snap] - SEAT_DY) * 1e3), 2),
            "seat_deg": round(float(np.degrees(th[-1])), 2), "stop_deg": round(float(np.degrees(stop)), 2),
            "withdrawal_play_mm": round(play * 1e3, 2),
            "tab_deg_at_play": round(float(np.degrees(tw[rising[0]])), 2) if len(rising) else None}


def measure() -> dict:
    """The numbers of the module docstring, from ray casts and sections of the final CAD (mm, degrees)."""
    from force_bench.kit import load_kit
    a = load_kit()["ethernet-plug-and-slot"]
    plug, ins, shell = (a.part(n).mesh() for n in ("ethernet_plug", "ethernet_slot", "ethernet_slot_shell"))
    xc = PLUG_X
    out = {}
    walls = _ray(shell, [-0.06, 0.010, 0.015], [1.0, 0.0, 0.0])
    out["pockets_x_mm"] = [[round(walls[i] * 1e3, 3), round(walls[i + 1] * 1e3, 3)] for i in range(1, len(walls) - 1, 2)]
    out["pocket_z_mm"] = [round(v * 1e3, 3) for v in _ray(shell, [0.0, 0.010, -0.005], [0, 0, 1.0])[1:3]]
    out["mouth_y_mm"] = round(_ray(shell, [0.0075, 0.05, 0.015], [0, -1.0, 0])[-1] * 1e3, 3)   # wall 3 | 4
    out["insert"] = {
        "floor_z_mm": round(_ray(ins, [0.0, 0.005, 0.015], [0, 0, -1.0])[-1] * 1e3, 3),
        "back_wall_y_mm": round(_ray(ins, [0.0, 0.03, 0.015], [0, -1.0, 0])[-1] * 1e3, 3),
        "roof_z_mm_at_y": {f"{y:g}": round(_ray(ins, [0.0, y * MM, 0.015], [0, 0, 1.0])[0] * 1e3, 3)
                           for y in (3.0, 6.65, 10.0, 13.0, 16.2)},
        "lip_x_mm": [round(v * 1e3, 3) for v in _ray(ins, [-0.01, 0.0155, 0.021], [1.0, 0, 0])],
        "lip_bottom_z_mm": round(_ray(ins, [0.0045, 0.0155, 0.015], [0, 0, 1.0])[0] * 1e3, 3),
        "lip_inner_face_y_mm_at_z": {f"{z:g}": round(_ray(ins, [0.0045, 0.03, z * MM], [0, -1.0, 0])[-2] * 1e3, 3)
                                     for z in (20.4, 21.0, 21.8, 22.2)},
    }
    out["plug"] = {
        "width_x_mm": [round(v * 1e3, 3) for v in _ray(plug, [-0.03, 0.030, 0.015], [1.0, 0, 0])],
        "nose_z_mm": [round(v * 1e3, 3) for v in _ray(plug, [xc, 0.030, 0.0], [0, 0, 1.0])[:2]],
        "rear_z_mm": [round(v * 1e3, 3) for v in _ray(plug, [xc + 0.0045, 0.044, 0.0], [0, 0, 1.0])],
        "nose_y_mm": round(_ray(plug, [xc, 0.0, 0.015], [0, 1.0, 0])[0] * 1e3, 3),
        "rear_front_y_mm": round(_ray(plug, [xc, 0.0, 0.011], [0, 1.0, 0])[0] * 1e3, 3),
        "tail_end_y_mm": round(_ray(plug, [xc, 0.0, 0.015], [0, 1.0, 0])[-1] * 1e3, 3),
    }
    zt = _ray(plug, [xc, 0.030, 0.0], [0, 0, 1.0])
    zt2 = _ray(plug, [xc, 0.033, 0.0], [0, 0, 1.0])
    slope = (zt2[-1] - zt[-1]) / 0.003
    out["tab"] = {
        "slope_deg": round(float(np.degrees(np.arctan(slope))), 3),
        "thickness_vertical_mm": round((zt[-1] - zt[-2]) * 1e3, 3),
        "thickness_normal_mm": round((zt[-1] - zt[-2]) * np.cos(np.arctan(slope)) * 1e3, 3),
        "wide_x_mm": [round(v * 1e3, 3) for v in _ray(plug, [-0.03, 0.033, zt2[-1] - 0.0005], [1.0, 0, 0])],
        "shoulder_y_mm": round(_ray(plug, [xc + 0.0045, 0.06, 0.022], [0, -1.0, 0])[-1] * 1e3, 3),
        "narrow_x_mm": [round(v * 1e3, 3) for v in _ray(plug, [-0.03, 0.040, 0.0242], [1.0, 0, 0])],
        "free_end_y_mm": round(_ray(plug, [xc, 0.06, 0.0250], [0, -1.0, 0])[-1] * 1e3, 3),
        "notch_y_mm": round(TAB_NOTCH[0] * 1e3, 3), "pivot_yz_mm": [round(v * 1e3, 3) for v in PIVOT_YZ],
    }
    out["clearance_x_per_side_mm"] = round((POCKET_HALF_W - PLUG_HALF_W) * 1e3, 3)
    # seat: the body's travel from the CAD pose until it meets the slot (sections at two x)
    from shapely import affinity
    body = split_plug(plug)["body"]
    seat = []
    for dx in (0.0010, 0.0045):
        from shapely.ops import unary_union
        sl = unary_union([_section(ins, dx), _section(shell, dx)])
        bd = _section(body, xc + dx)
        lo, hi = 0.0, -0.030
        for _ in range(50):
            m = 0.5 * (lo + hi)
            lo, hi = (m, hi) if affinity.translate(bd, m, 0.0).intersection(sl).area < 1e-12 else (lo, m)
        seat.append(lo)
    out["seat_travel_mm"] = round(-max(seat) * 1e3, 3)
    out["rear_face_proud_of_mouth_mm"] = round((out["plug"]["rear_front_y_mm"] + max(seat) * 1e3)
                                               - out["mouth_y_mm"], 3)
    lk = latch_kinematics()
    out["latch"] = {k: v for k, v in lk.items() if not isinstance(v, np.ndarray)}
    return out


# ── FEM estimate of the tab stiffness ───────────────────────────────────────────────────────────────────────────────
def fem_tab_stiffness(h_mm: float = 0.025, E: float = 3.0e9, nu: float = 0.35, plane: str = "stress",
                      load_y_mm: float = 34.6) -> dict:
    """The printed tab under a unit force normal to it on its top face at ``load_y_mm`` (CAD y), 2D finite elements
    (bilinear quads on an ``h_mm`` raster) of the plug's x = centre section: the tab and 1 mm of body below its top
    face (clamped at z = 17.6 mm); every element as deep (x) as the tab is wide there (11 mm in front of the
    shoulder, 6 mm behind it).  Returns the stiffness at the load point [N/m], its lever about the hinge [m] and the
    equivalent hinge stiffness K = k r^2 [N.m/rad]."""
    import scipy.sparse as sp
    import scipy.sparse.linalg as spla
    import shapely
    from shapely.geometry import box
    from force_bench.kit import load_kit
    m = load_kit()["ethernet-plug-and-slot"].part("ethernet_plug").mesh()
    m.apply_scale(1e3)                                         # mm
    sec = _section(m, PLUG_X * 1e3)
    y0, y1, z0, z1 = 23.0, 43.2, 17.6, 25.8
    reg = sec.intersection(box(y0, z0, y1, z1))
    h = float(h_mm)
    ny, nz = int(round((y1 - y0) / h)), int(round((z1 - z0) / h))
    YC, ZC = np.meshgrid(y0 + (np.arange(ny) + 0.5) * h, z0 + (np.arange(nz) + 0.5) * h, indexing="ij")
    inside = shapely.contains_xy(reg, YC.ravel(), ZC.ravel()).reshape(ny, nz)
    Emm = E * 1e-6                                             # N/mm^2
    if plane == "stress":
        D = Emm / (1 - nu ** 2) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
    else:
        Ep, nup = Emm / (1 - nu ** 2), nu / (1 - nu)
        D = Ep / (1 - nup ** 2) * np.array([[1, nup, 0], [nup, 1, 0], [0, 0, (1 - nup) / 2]])
    g = 1 / np.sqrt(3)
    Ke = np.zeros((8, 8))
    for xi in (-g, g):
        for eta in (-g, g):
            dy = 0.25 * np.array([-(1 - eta), (1 - eta), (1 + eta), -(1 + eta)]) / (h / 2)
            dz = 0.25 * np.array([-(1 - xi), -(1 + xi), (1 + xi), (1 - xi)]) / (h / 2)
            Bm = np.zeros((3, 8))
            Bm[0, 0::2], Bm[1, 1::2], Bm[2, 0::2], Bm[2, 1::2] = dy, dz, dz, dy
            Ke += Bm.T @ D @ Bm * (h / 2) ** 2
    ei, ej = np.nonzero(inside)
    depth = np.where(YC[ei, ej] < SHOULDER_Y * 1e3, 2 * PLUG_HALF_W * 1e3, 6.0)
    nid = lambda i, j: i * (nz + 1) + j                       # noqa: E731
    conn = np.stack([nid(ei, ej), nid(ei + 1, ej), nid(ei + 1, ej + 1), nid(ei, ej + 1)], 1)
    used = np.unique(conn)
    remap = -np.ones((ny + 1) * (nz + 1), int)
    remap[used] = np.arange(len(used))
    c = remap[conn]
    dofs = np.stack([2 * c, 2 * c + 1], 2).reshape(-1, 8)
    vals = (Ke.ravel()[None, :] * depth[:, None]).ravel()
    K = sp.coo_matrix((vals, (np.repeat(dofs, 8, 1).ravel(), np.tile(dofs, (1, 8)).ravel())),
                      shape=(2 * len(used), 2 * len(used))).tocsr()
    Y, Z = y0 + (used // (nz + 1)) * h, z0 + (used % (nz + 1)) * h
    fixed = np.zeros(2 * len(used), bool)
    cl = np.nonzero(Z <= z0 + 1e-9)[0]
    fixed[2 * cl] = fixed[2 * cl + 1] = True
    z_top = (BODY_TOP_Z * 1e3 + TAB_T * 1e3 / np.cos(TAB_ALPHA)) + TAB_SLOPE * (Y - TAB_NOTCH[0] * 1e3)
    sel = np.nonzero((np.abs(Y - load_y_mm) <= 0.15 + 1e-9) & (np.abs(Z - z_top) <= h * 1.01))[0]
    n_dir = np.array([np.sin(TAB_ALPHA), -np.cos(TAB_ALPHA)])  # pressing, normal to the tab
    f = np.zeros(2 * len(used))
    f[2 * sel] += n_dir[0] / len(sel)
    f[2 * sel + 1] += n_dir[1] / len(sel)
    free = ~fixed
    u = np.zeros(2 * len(used))
    u[free] = spla.spsolve(K[free][:, free].tocsc(), f[free])
    defl = float(np.mean(u[2 * sel] * n_dir[0] + u[2 * sel + 1] * n_dir[1]))      # mm per N
    k = 1.0 / defl * 1e3                                                         # N/m
    p_load = np.array([load_y_mm, float(np.mean(Z[sel]))]) * 1e-3
    lever = float(np.dot(p_load - np.array(PIVOT_YZ), [np.cos(TAB_ALPHA), np.sin(TAB_ALPHA)]))
    return {"k_N_per_m": k, "lever_m": lever, "K_N_m_per_rad": k * lever ** 2, "elements": int(len(ei)),
            "h_mm": h, "E_Pa": E, "plane": plane}


# ── probe ───────────────────────────────────────────────────────────────────────────────────────────────────────────
PROBE_PORT = 1
PROBE_START_GAP = 0.002     # m, nose out of the mouth at the start
SPEED = 0.005               # m/s, insertion
OVERTRAVEL = 0.0004         # m, insertion target past the seat (the drive's clamp bounds the push)
F_LIGHT = 10.0              # N, the light pull
F_STRONG = 30.0             # N, the pull ramp's top (reported)
LIGHT_RISE_MAX = 0.0015     # m
PRESS_TORQUE = 0.32         # N.m on the hinge: past the spring at the stop (KE_TAB x 0.346 = 0.25 N.m)
PRESS_LEVER = 0.0137        # m, hinge -> the tab's top face at the mouth when seated (for N <-> N.m)
EXTRACT_MIN = 0.005         # m the pressed plug must come out
SEAT_TOL = 0.0003
PEAK_MAX = 150.0
PROBE_PHASES = ("insert", "hold", "light_pull", "strong_pull", "reseat", "press", "pressed_pull", "unpress",
                "reinsert", "done")
T_UNPRESS = 0.3             # s: the press is let off gradually (an instant release flings the tab ~11 deg in one
                            # substep, faster than the contact can catch it, and it tunnels through the lip)


def add_probe(rig, h) -> None:
    from force_bench.kit import load_kit
    asset = load_kit()["ethernet-plug-and-slot"]
    lift = (MOUTH_Y - BACK_WALL_Y) + PROBE_START_GAP          # seat -> nose PROBE_START_GAP out of the mouth
    T = h.pose @ seat_transform(PROBE_PORT, lift)
    axis_in = -port_axis(h.pose)
    pl = add_plug(rig, asset, h, T, "probe_plug", root="probe", tab_q=0.0, mu=float(h.params["mu"]),
                  ke=float(h.params["ke_N_m_per_rad"]), kd=float(h.params["kd_N_m_s_per_rad"]),
                  probe_drive=dict(lower=-0.005, upper=lift + 0.003), probe_axis=axis_in)
    h.info["probe"] = {"port": PROBE_PORT, "joint": pl["j_root"], "hinge": pl["j_tab"], "seat_s_mm": round(lift * 1e3, 3),
                       "start_gap_mm": PROBE_START_GAP * 1e3, "body": pl["body"], "tab": pl["tab"]}
    h.info["probe_bodies"] = ["probe_plug", "probe_plug/tab"]


def run_probe(rig, h) -> dict:
    from force_bench.rig import make_solver
    pr = h.info["probe"]
    seat_s = pr["seat_s_mm"] * MM
    j, jt = pr["joint"], pr["hinge"]
    t_start = rig.model.joint_target_q_start.numpy()
    q_start = rig.model.joint_q_start.numpy()
    qd_start = rig.model.joint_qd_start.numpy()
    i_tgt, q_lin, q_tab = int(t_start[j]), int(q_start[j]), int(q_start[jt])
    f_tab = int(qd_start[jt])
    ke_drive = float(rig.model.joint_target_ke.numpy()[int(qd_start[j])])
    out_axis = port_axis(h.pose)
    probes = [rig.probes["probe_plug"], rig.probes["probe_plug/tab"]]
    inner = make_solver(rig.model, iterations=24)
    st = {"phase": "insert", "t0": 0.0, "s_tgt": 0.0, "press": 0.0, "pull": 0.0, "q": 0.0}

    class Ramp:                      # targets move every SUBSTEP (a per-frame step kicks the drive)
        def __init__(self):
            self.t = 0.0

        def step(self, s_in, s_out, control, contacts, dt):
            self.t += dt
            ph = st["phase"]
            q = float(s_in.joint_q.numpy()[q_lin])
            if ph == "insert":
                st["s_tgt"] = min(st["s_tgt"] + SPEED * dt, seat_s + OVERTRAVEL)
            elif ph in ("hold", "reseat"):
                st["s_tgt"] = seat_s + OVERTRAVEL * 0.5
            elif ph in ("light_pull", "strong_pull", "pressed_pull"):
                st["s_tgt"] = q - st["pull"] / ke_drive          # a constant-force pull through the position drive
            elif ph == "press":
                st["s_tgt"] = seat_s + OVERTRAVEL * 0.5
            elif ph == "unpress":
                st["s_tgt"] = st["s_hold"]
            elif ph == "reinsert":
                st["s_tgt"] = min(st["s_tgt"] + SPEED * dt, seat_s + OVERTRAVEL)
            elif ph == "done":
                st["s_tgt"] = seat_s + OVERTRAVEL * 0.5
            qt = control.joint_target_q.numpy()
            qt[i_tgt] = st["s_tgt"]
            control.joint_target_q.assign(qt)
            jf = control.joint_f.numpy()
            jf[f_tab] = st["press"]
            control.joint_f.assign(jf)
            return inner.step(s_in, s_out, control, contacts, dt)

        def __getattr__(self, n):
            return getattr(inner, n)

    rows = []

    def extra(state, t):
        F = 0.0
        for p in probes:
            r = p.sample(rig.contacts, state, t)
            F += float(np.dot(r["vec"], out_axis))               # the slot's push along +out (resists insertion)
        q = state.joint_q.numpy()
        s, ang = float(q[q_lin]), float(q[q_tab])
        ph = st["phase"]
        rows.append((t, PROBE_PHASES.index(ph), (s - (seat_s - (MOUTH_Y - BACK_WALL_Y))) * 1e3, F,
                     float(np.degrees(ang)), st["pull"], st["press"]))
        el = t - st["t0"]
        if ph == "insert" and st["s_tgt"] >= seat_s + OVERTRAVEL - 1e-9 and el > 0.1:
            st.update(phase="hold", t0=t)
        elif ph == "hold" and el >= 0.4:
            st.update(phase="light_pull", t0=t, seat_s=s)
        elif ph == "light_pull":
            st["pull"] = F_LIGHT * min(1.0, el / 0.2)
            if el >= 0.7:
                st.update(phase="strong_pull", t0=t)
        elif ph == "strong_pull":
            st["pull"] = F_LIGHT + (F_STRONG - F_LIGHT) * min(1.0, el / 0.6)
            if el >= 0.8 or s < st["seat_s"] - 0.004:
                st.update(phase="reseat", t0=t, pull=0.0)
        elif ph == "reseat" and el >= 0.5:
            st.update(phase="press", t0=t)
        elif ph == "press":
            st["press"] = PRESS_TORQUE * min(1.0, el / 0.4)
            if el >= 0.6:
                st.update(phase="pressed_pull", t0=t)
        elif ph == "pressed_pull":
            st["pull"] = F_LIGHT * min(1.0, el / 0.2)
            if el >= 1.2 or s < st["seat_s"] - 0.008:
                st.update(phase="unpress", t0=t, pull=0.0, s_hold=s)
        elif ph == "unpress":
            st["press"] = PRESS_TORQUE * max(0.0, 1.0 - el / T_UNPRESS)
            if el >= T_UNPRESS + 0.1:
                st.update(phase="reinsert", t0=t, press=0.0, s_tgt=s)
        elif ph == "reinsert" and st["s_tgt"] >= seat_s + OVERTRAVEL - 1e-9 and el > 0.1:
            st.update(phase="done", t0=t)
        return {}

    saved = rig.solver
    rig.solver = Ramp()
    budget = 0.1 + (seat_s + OVERTRAVEL) / SPEED + 0.4 + 0.7 + 0.8 + 0.5 + 0.6 + 1.2 + T_UNPRESS + 0.1 + \
        0.2 + 0.010 / SPEED + 0.4
    rig.run(budget, lambda t: None, extra, print_every=10 ** 9)
    rig.solver = saved
    a = np.array(rows)
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "probe_force.csv", a, delimiter=",", comments="",
               header="t,phase(" + " ".join(f"{i} {p}" for i, p in enumerate(PROBE_PHASES)) + "),"
                      "nose_depth_mm,slot_force_out_N,tab_deg,pull_cmd_N,press_cmd_N_m")
    seat_depth = (MOUTH_Y - BACK_WALL_Y) * 1e3                    # nose depth below the mouth at the seat (15.0)

    def ph(name):
        return a[a[:, 1] == PROBE_PHASES.index(name)]
    ins, hold, light, strong, press, ppull = (ph(n) for n in ("insert", "hold", "light_pull", "strong_pull", "press",
                                                               "pressed_pull"))
    d_seat = float(np.median(hold[-max(1, len(hold) // 3):, 2])) if len(hold) else float("nan")
    seated = abs(d_seat - seat_depth) <= SEAT_TOL * 1e3
    pre = ins[ins[:, 2] < seat_depth - 0.3] if len(ins) else ins
    peak = float(np.max(pre[:, 3])) if len(pre) else float("nan")
    snap = float(np.max(ins[:, 4])) if len(ins) else 0.0
    tab_seat = float(np.median(hold[-max(1, len(hold) // 3):, 4])) if len(hold) else float("nan")
    lw = light[light[:, 5] >= 0.95 * F_LIGHT] if len(light) else light
    light_rise = float(d_seat - np.min(light[:, 2])) if len(light) else float("nan")
    light_held = bool(len(lw) and light_rise < LIGHT_RISE_MAX * 1e3 and np.min(lw[:, 2]) > d_seat - 1.5)
    strong_rise = float(d_seat - np.min(strong[:, 2])) if len(strong) else float("nan")
    held_max = float(np.max(strong[strong[:, 2] > d_seat - 1.5, 5])) if len(strong) else 0.0
    press_deg = float(np.max(press[:, 4])) if len(press) else 0.0
    extracted = float(d_seat - np.min(ppull[:, 2])) if len(ppull) else 0.0
    pp = ppull[(ppull[:, 2] < d_seat - 0.3) & (ppull[:, 2] > d_seat - 3.0)] if len(ppull) else ppull
    resist = float(np.max(-pp[:, 3])) if len(pp) else 0.0
    released = extracted >= EXTRACT_MIN * 1e3
    snapped = snap >= 12.0 and tab_seat < 8.0
    rein, done = ph("reinsert"), ph("done")
    relatched = bool(len(done) and abs(float(np.median(done[:, 2])) - seat_depth) <= SEAT_TOL * 1e3
                     and len(rein) and float(np.max(rein[:, 4])) >= 12.0 and float(np.median(done[:, 4])) < 8.0)
    ok = bool(np.isfinite(a).all() and seated and snapped and light_held and released and relatched
              and peak < PEAK_MAX)
    # the asset's own (free) plug, seated at build: where it sits now
    seated_plug = None
    lbl = f"{h.label}/plug"
    if lbl in h.bodies and h.info.get("plug") == "seated":
        bq = rig.s0.body_q.numpy()
        c_body = 0.5 * (_plug_pieces()["body"].bounds[0] + _plug_pieces()["body"].bounds[1])
        p_seat = (h.pose @ seat_transform(h.info["port"]) @ np.r_[c_body, 1.0])[:3]
        seated_plug = {"port": h.info["port"],
                       "backed_off_mm": round(float(np.dot(bq[h.bodies[lbl], :3] - p_seat, out_axis)) * 1e3, 3),
                       "tab_deg": round(float(np.degrees(tab_angle(bq, h.bodies[lbl], h.bodies[lbl + "/tab"]))), 2)}
    # force against depth on the way in
    fd = {}
    for d in (2, 4, 6, 8, 10, 11, 12, 13, 14, 14.5):
        if len(ins) and d < ins[:, 2].max():
            fd[f"{d:g}mm"] = round(float(np.interp(d, ins[:, 2], ins[:, 3])), 2)
    i_pk = int(np.argmax(pre[:, 3])) if len(pre) else 0
    return {"pass": ok, "port": pr["port"], "speed_mm_s": SPEED * 1e3,
            "seat_depth_mm": round(seat_depth, 3), "depth_seated_mm": round(d_seat, 3), "seated": bool(seated),
            "peak_insertion_force_N": round(peak, 2),
            "peak_at_depth_mm": round(float(pre[i_pk, 2]), 2) if len(pre) else None,
            "force_at_depth_N": fd,
            "tab": {"max_deg_on_the_way_in": round(snap, 2), "seated_deg": round(tab_seat, 2),
                    "snapped_in": bool(snapped)},
            "light_pull": {"force_N": F_LIGHT, "rise_mm": round(light_rise, 3), "held": light_held,
                           "rise_max_mm": LIGHT_RISE_MAX * 1e3},
            "strong_pull": {"top_N": F_STRONG, "held_up_to_N": round(held_max, 1), "rise_mm": round(strong_rise, 3)},
            "press": {"torque_N_m": PRESS_TORQUE, "force_at_mouth_N": round(PRESS_TORQUE / PRESS_LEVER, 1),
                      "tab_deg": round(press_deg, 2), "stop_deg": round(float(np.degrees(TAB_LIMIT[1])), 2)},
            "pressed_pull": {"force_N": F_LIGHT, "extracted_mm": round(extracted, 2), "released": bool(released),
                             "resistance_peak_N": round(resist, 2)},
            "reinsert": {"tab_max_deg": round(float(np.max(rein[:, 4])), 2) if len(rein) else None,
                         "depth_mm": round(float(np.median(done[:, 2])), 3) if len(done) else None,
                         "tab_deg": round(float(np.median(done[:, 4])), 2) if len(done) else None,
                         "relatched": relatched},
            "seated_plug": seated_plug}
