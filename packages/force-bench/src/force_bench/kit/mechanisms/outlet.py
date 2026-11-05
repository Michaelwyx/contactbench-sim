"""Outlet: the printed shell with a printed contact spring, as a static shell with one sprung pad per blade slot.

The printed spring is one part behind both blade slots; its flexures stand into the slots by the variant's
*indent* (0.4, 0.7, 0.85 or 1 mm) and grip the charger's blades.  It is not simulated as a deforming body.  Each slot
gets a pad on a prismatic joint normal to the blade face (the model of ``force_bench.tasks.plug_insert_arm``,
whose shell has the same slots): the pad's face stands ``protrusion`` into the slot and is pushed back against a
spring of rate ``ke``.  Its entry is a curved ramp tangent to the grip face, so the blade wedges it open without
a corner.  The printed spring itself is drawn but does not collide.

Parameters per variant (see examples/force_response/README.md for the fits):

    indent   protrusion   ke          source
    0.4 mm   0.80 mm      28 kN/m     fitted to measured insertion force (UR arm, wrist F/T, 10 trials)
    0.7 mm   1.10 mm      36.7 kN/m   interpolated between the two measured springs
    0.85 mm  1.25 mm      41 kN/m     fitted to measured insertion force
    1 mm     1.40 mm      45.3 kN/m   extrapolated (not measured)

protrusion = indent + 0.4 mm, because the first 0.4 mm of pad travel only takes up the blade's play in its slot.
``build(..., protrusion=, ke=)`` overrides both.  Measured force profiles and the fit are in
``examples/force_response``.

Probe (``force-bench kit check outlet``): charging block 3 is driven straight into the outlet at 10 mm/s; the
check records force against depth and passes when the block reaches its seat and the peak force is finite and
below 150 N.
"""

from __future__ import annotations

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.assets import make_newton_mesh, set_body_mass
from force_bench.geometry import subdiv_processed as subdiv
from force_bench.kit.build import pose_matrix, to_wp
from force_bench.tasks.plug_insert import find_slots

MEASURED = {0.4: (0.0008, 28000.0), 0.85: (0.00125, 41000.0)}     # indent mm -> (protrusion m, ke N/m)
CLEARANCE = 0.0004
PAD_KD = 15.0
PAD_TRAVEL = 0.003
PAD_MASS = 0.010
PAD_HX = 0.0055
SLOT_HALFWIDTH = 0.00125
CREST_DROP = 0.002          # pad crest below the face plate
PLATE_BOT = 0.0055          # pad bottom: below the seated blade tip
FLARE_DEG = 30.0
CFG = dict(ke=1.0e5, kd=1.0e3, gap=0.0001, mu_torsional=0.0, mu_rolling=0.0)
SDF_MARGIN = 0.00015


def spring_params(indent_mm: float) -> tuple[float, float, str]:
    """(protrusion [m], ke [N/m], provenance) for a printed spring of the given indent."""
    (i0, (_, k0)), (i1, (_, k1)) = sorted(MEASURED.items())
    ke = k0 + (k1 - k0) * (indent_mm - i0) / (i1 - i0)
    src = ("measured" if indent_mm in MEASURED else
           "interpolated" if i0 < indent_mm < i1 else "extrapolated")
    return indent_mm * 1e-3 + CLEARANCE, ke, src


def pad_profile_tm(hx: float, side: int, prot: float, h_face: float, h_bevel: float,
                   back: float = 0.002, n_arc: int = 24) -> trimesh.Trimesh:
    """Pad prism: vertical grip face (0..h_face), then a curved entry tangent to it that recedes by ``prot``
    over ``h_bevel``.  Local frame: grip face plane at y = 0, bottom at z = 0, extruded along x."""
    s = float(side)
    pts = [(0.0, 0.0), (0.0, h_face)]
    for u in np.linspace(0.0, 1.0, n_arc)[1:]:
        pts.append((s * prot * (1.0 - np.cos(0.5 * np.pi * u)), h_face + u * h_bevel))
    pts += [(s * (prot + back), h_face + h_bevel), (s * (prot + back), 0.0)]
    v = np.array([[x, y, z] for x in (-hx, hx) for (y, z) in pts])
    return subdiv(trimesh.convex.convex_hull(v), 0.0015)


def _indent_mm(label: str | None) -> float:
    return float(label.split("mm")[0]) if label else 0.85


def build(rig, asset, h, protrusion: float | None = None, ke: float | None = None, kd: float = PAD_KD,
          mu: float | None = None) -> None:
    mu = float(asset.sim.get("mu", 0.4)) if mu is None else mu
    indent = _indent_mm(h.choice.get("Insertion force"))
    prot0, ke0, src = spring_params(indent)
    if protrusion is not None or ke is not None:
        src = "override"
    prot = prot0 if protrusion is None else protrusion
    ke = ke0 if ke is None else ke
    h.params = {"indent_mm": indent, "protrusion_mm": round(prot * 1e3, 3), "ke_N_per_m": round(ke, 1),
                "kd": kd, "mu": mu, "source": src}
    pose = h.pose
    shell_p = asset.part("outlet_shell", h.choice)
    shell_tm = shell_p.mesh()
    slots = find_slots(shell_tm)
    assert len(slots) == 2, f"outlet: expected 2 blade slots, found {len(slots)}"
    face_z = float(shell_tm.bounds[1][2])
    h.info = {"face_z_cad": face_z, "slots": [{"center_mm": (s["center"] * 1e3).round(3).tolist(),
                                              "size_mm": s["size_mm"].round(3).tolist()} for s in slots]}
    # shell: static, fine SDF with tight surfaces (the blades have 0.25 mm of play per side)
    mesh, c = make_newton_mesh(subdiv(shell_tm, 0.003), sdf=True, sdf_resolution=int(asset.sim.get("sdf_res", 512)),
                               margin=SDF_MARGIN)
    cfg = newton.ModelBuilder.ShapeConfig(mu=mu, density=1240.0, **CFG)
    shell_sid = rig.builder.add_shape_mesh(-1, xform=to_wp(pose @ pose_matrix(xyz=c)), mesh=mesh, cfg=cfg,
                                           color=shell_p.rgb, label=f"kit/{h.label}/outlet_shell")
    h.shapes["outlet_shell"] = [shell_sid]
    # the printed spring: drawn only
    spring_p = [p for p in asset.parts(h.choice) if p.name.startswith("outlet_spring")][0]
    sm, sc = make_newton_mesh(spring_p.mesh(), sdf=False)
    vis = newton.ModelBuilder.ShapeConfig(density=0.0, has_shape_collision=False, has_particle_collision=False)
    h.shapes[spring_p.name] = [rig.builder.add_shape_mesh(-1, xform=to_wp(pose @ pose_matrix(xyz=sc)), mesh=sm,
                                                          cfg=vis, color=spring_p.rgb,
                                                          label=f"kit/{h.label}/{spring_p.name}")]
    # one sprung pad per slot, on opposite walls as the printed flexures are
    flare = np.deg2rad(FLARE_DEG)
    z_top = face_z - CREST_DROP
    h_bevel = prot / np.tan(flare)
    h_face = (z_top - h_bevel) - PLATE_BOT
    for slot, side, name in ((slots[0], -1, "pad_wide"), (slots[1], +1, "pad_narrow")):
        y_grip = float(slot["center"][1]) + side * (SLOT_HALFWIDTH - prot)
        T_cad = pose_matrix(xyz=(float(slot["center"][0]), y_grip, PLATE_BOT))
        T_w = pose @ T_cad
        body = rig.builder.add_link(xform=to_wp(T_w), label=f"{h.label}/{name}")
        tm_pad = pad_profile_tm(PAD_HX, side, prot, float(h_face), float(h_bevel))
        pm, pc = make_newton_mesh(tm_pad, sdf=True, sdf_resolution=128, margin=SDF_MARGIN)
        pad_cfg = newton.ModelBuilder.ShapeConfig(mu=mu, density=4000.0, **CFG)
        sid = rig.builder.add_shape_mesh(body, xform=wp.transform(wp.vec3(*pc), wp.quat_identity()), mesh=pm,
                                         cfg=pad_cfg, color=spring_p.rgb, label=f"kit/{h.label}/{name}")
        rig.builder.add_shape_collision_filter_pair(sid, shell_sid)
        # the axis is in the joint's parent frame, T_w, which already carries the outlet's rotation
        j = rig.builder.add_joint_prismatic(parent=-1, child=body, axis=(0.0, float(side), 0.0),
                                            parent_xform=to_wp(T_w), child_xform=wp.transform_identity(),
                                            target_pos=0.0, target_ke=ke, target_kd=kd,
                                            limit_lower=0.0, limit_upper=PAD_TRAVEL, label=f"{h.label}/{name}")
        rig.builder.add_articulation([j])
        set_body_mass(rig.builder, body, PAD_MASS)
        h.bodies[name], h.joints[name], h.shapes[name] = body, j, [sid]
    h.info["pad_h_face_mm"] = round(float(h_face) * 1e3, 3)
    h.info["pad_h_bevel_mm"] = round(float(h_bevel) * 1e3, 3)


# ── probe: push charging block 3 straight in ─────────────────────────────────────────────────────────────────────
SPEED = 0.010
PLUG = "charging-block-3"


def add_probe(rig, h) -> None:
    from force_bench.kit import load_kit
    from force_bench.tasks.plug_insert import find_prongs
    from force_bench.tasks.plug_insert_arm import pose_plug, center_in_slots, measure_seat, APPROACH
    kit = load_kit()
    shell_tm = kit["outlet"].part("outlet_shell").mesh()
    slots = find_slots(shell_tm)
    plug_p = kit[PLUG].parts()[0]
    plug_tm = plug_p.mesh()
    posed = pose_plug(plug_tm, shell_tm, slots, find_prongs(plug_tm))
    center_in_slots(posed, slots)
    seat = measure_seat(posed, shell_tm)
    posed.apply_transform(h.pose)
    pm, pc = make_newton_mesh(subdiv(posed, 0.003), sdf=True, sdf_resolution=448, margin=SDF_MARGIN)
    pc = np.asarray(pc)
    body = rig.builder.add_link(xform=wp.transform(wp.vec3(*pc), wp.quat_identity()), label="probe_plug")
    cfg = newton.ModelBuilder.ShapeConfig(mu=float(h.params["mu"]), density=1240.0, **CFG)
    rig.builder.add_shape_mesh(body, mesh=pm, cfg=cfg, color=plug_p.rgb, label="probe_plug")
    set_body_mass(rig.builder, body, 0.06)
    stroke = seat["dz_seat"] - 0.0006
    down = h.pose[:3, :3] @ np.array([0.0, 0.0, -1.0])
    ke = 6.0e4
    j = rig.builder.add_joint_prismatic(parent=-1, child=body, axis=tuple(float(a) for a in down),
                                        parent_xform=wp.transform(wp.vec3(*pc), wp.quat_identity()),
                                        child_xform=wp.transform_identity(), target_pos=0.0, target_ke=ke,
                                        target_kd=2.0 * np.sqrt(ke * 0.06) * 1.5,
                                        limit_lower=-0.005, limit_upper=stroke + 0.002, label="probe_plug")
    rig.builder.add_articulation([j])
    h.info["probe"] = {"plug": PLUG, "stroke_mm": round(stroke * 1e3, 3), "approach_mm": APPROACH * 1e3,
                       "joint": j, "body": body, "seat_dz_mm": round(seat["dz_seat"] * 1e3, 3)}
    h.bodies["probe_plug"] = body
    h.info["probe_bodies"] = ["probe_plug"]


def run_probe(rig, h) -> dict:
    from force_bench.rig import make_solver
    pr = h.info["probe"]
    j, stroke = pr["joint"], pr["stroke_mm"] * 1e-3
    probe = rig.probes["probe_plug"]
    tgt_idx = int(rig.model.joint_target_q_start.numpy()[j])
    q_idx = int(rig.model.joint_q_start.numpy()[j])
    inner = make_solver(rig.model, iterations=24)

    class Ramp:                      # move the push target every SUBSTEP (a per-frame step kicks the drive)
        def __init__(self):
            self.t = 0.0

        def step(self, s_in, s_out, control, contacts, dt):
            self.t += dt
            qt = control.joint_target_q.numpy()
            qt[tgt_idx] = min(stroke, SPEED * max(0.0, self.t - 0.2))
            control.joint_target_q.assign(qt)
            return inner.step(s_in, s_out, control, contacts, dt)

        def __getattr__(self, n):
            return getattr(inner, n)

    saved = rig.solver
    rig.solver = Ramp()
    rows = []

    def extra(state, t):
        r = probe.sample(rig.contacts, state, t)
        depth = float(state.joint_q.numpy()[q_idx])
        rows.append((t, depth * 1e3, float(np.dot(r["vec"], h.pose[:3, 2]))))
        return {}

    rig.run(0.2 + stroke / SPEED + 0.4, lambda t: None, extra, print_every=10 ** 9)
    rig.solver = saved
    a = np.array(rows)
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    np.savetxt(out / "probe_force.csv", a, delimiter=",", header="t,depth_mm,force_N", comments="")
    depth_end = float(a[-1, 1])
    peak = float(np.max(np.abs(a[:, 2]))) if len(a) else float("nan")
    seated = depth_end >= pr["stroke_mm"] - 0.6
    ok = bool(np.isfinite(a).all() and seated and peak < 150.0)
    return {"pass": ok, "peak_force_N": round(peak, 2), "depth_reached_mm": round(depth_end, 3),
            "stroke_mm": pr["stroke_mm"], "plug": pr["plug"],
            "force_at_depth_N": {f"{d:g}mm": round(float(np.interp(d, a[:, 1], np.abs(a[:, 2]))), 2)
                                 for d in (2, 5, 10, 15) if d < depth_end}}
