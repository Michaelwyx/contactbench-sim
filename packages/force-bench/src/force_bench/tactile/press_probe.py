"""Instrument: press suite objects into one gel pad and audit sensor vs solver.

Why this exists (2026-08-22): integrating the pads into the suite tasks
(TB_GELSIGHT=1) showed that on the door key's BOW the finger probes carry
~10 N per pad while the gel's pressure integral reads < 1 N, whereas the
flat cube of task_gelsight_grip agrees to ~10 %.  This probe isolates the
contact model from the arm and MuJoCo dynamics: a kinematic indenter
carrying one object is parked at a known geometric indentation into a
static pad (the same 2x physics slab the FR3 mount uses, winning
hydroelastic config), and for each depth we report

  * F_sensor    the gel sensor's pressure integral (kh * depth * area, normal-
                projected) -- the tactile observation
  * F_hydro     sum over the REDUCED solver contacts of
                c_stiffness * (-contact_distance) -- the force the hydroelastic
                pipeline asks the solver to apply (Newton's own identity
                F = area * pressure per face)
  * faces / contacts / margin counts and the pressure range

so the question "is the sensor honest to the solver" is answered per object
with no dynamics in the loop.  Objects: box (flat 16 mm), sphere, nut (hex
flat), key_bow (a door key's embossed bow, as a pinch grips it), chop (a
chopstick, rod).

Run (env tactile-newton, from sim/):
    PYTHONNOUSERSITE=1 python -m force_bench.tactile.press_probe [--objects key_bow,box] [--depths 0.1,0.3,0.6,1.0]
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import trimesh
import warp as wp

import newton

from hydrogel.sensor import (
    GelSightConfig,
    HydroelasticGelSight,
    attach_gelsight_pad,
    make_gel_shape_config,
)
from hydrogel.sensor import hydroelastic_object_cfg, winning_hydroelastic_config
from force_bench.assets import load_tm, make_newton_mesh


def object_mesh(name: str):
    """Return (trimesh in a frame whose -z face is what touches the gel,
    sdf kwargs) for the suite objects."""

    if name in ("key_bow", "key_bow_b", "key_bow_nb2"):
        # RAW STL frame: length along z (49.8 mm), bow width along x (20 mm),
        # thickness along y (4.65 mm incl. the blade's bitting).  Find the
        # bow as the wide end, lay the key along the pad's y with the bow
        # centred, and turn one bow face (+y for key_bow, -y for key_bow_b)
        # to face the gel (-z).
        tm = load_tm("key_1", "part3_1_body1")
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
    if name == "nut":
        tm = load_tm("fasteners", "nut_075mm_1_body1")
        # a hex flat down: the flats are along +-y (width 15.5 across flats);
        # rotate so a flat faces -z
        tm.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2, (1.0, 0.0, 0.0)))
        return tm, dict(sdf_resolution=160, narrow_band=0.002, margin=0.0005)
    if name == "chop":
        tm = load_tm("utensil_chopstick", "part2_1_body1")
        # rod axis along local z (199 mm); lay it along the pad's x
        tm.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2, (0.0, 1.0, 0.0)))
        # centre the rod on the pad (mid-rod section, ~5.5 mm dia)
        tm.apply_translation(-(tm.bounds[0] + tm.bounds[1]) / 2.0)
        return tm, dict(sdf_resolution=384, narrow_band=0.002, margin=0.0005)
    raise ValueError(name)


def build(name: str, cfg: GelSightConfig, density: float = 2700.0):
    b = newton.ModelBuilder()
    t = cfg.gel_thickness_m
    as_mesh = os.environ.get("GS_GEL_MESH", "1") != "0"
    body = b.add_body(xform=wp.transform(wp.vec3(0.0, 0.0, 0.1),
                                         wp.quat_identity()),
                      is_kinematic=True, label="indenter")
    spec = {}
    if name == "box":
        h = 0.008
        b.add_shape_box(body, hx=h, hy=h, hz=h,
                        cfg=hydroelastic_object_cfg(cfg, density=density),
                        label="obj")
        spec["zmin"] = -h
    elif name == "sphere":
        r = 0.008
        b.add_shape_sphere(body, radius=r,
                           cfg=hydroelastic_object_cfg(cfg, density=density),
                           label="obj")
        spec["zmin"] = -r
    else:
        tm, sdf_kw = object_mesh(name)
        mesh, center = make_newton_mesh(tm, sdf=True, **sdf_kw)
        b.add_shape_mesh(body, mesh=mesh,
                         cfg=hydroelastic_object_cfg(cfg, density=density,
                                                     mesh=True),
                         label="obj")
        # lowest point OVER THE PAD FOOTPRINT (a tapered rod's lowest point
        # is off the pad): the indentation is defined at the pad
        v = tm.vertices
        over = v[(np.abs(v[:, 0]) < 0.5 * cfg.width_m) & (np.abs(v[:, 1]) < 0.5 * cfg.height_m)]
        zl = float(over[:, 2].min()) if len(over) else float(v[:, 2].min())
        spec["zmin"] = zl - float(center[2])
        spec["center"] = np.asarray(center)
    # static pad, sensing face at z = +t, physics slab 2t thick (centre 0);
    # added AFTER the object so a mesh gel is shape B of the pair
    gel = attach_gelsight_pad(b, -1, cfg, xform=wp.transform_identity(),
                              label="gel", physics_thickness_m=2.0 * t,
                              as_mesh=as_mesh)
    model = b.finalize()
    state = model.state()
    pipeline = newton.CollisionPipeline(
        model, rigid_contact_max=20_000,
        broad_phase=os.environ.get("GS_BROAD", "explicit"),
        sdf_hydroelastic_config=winning_hydroelastic_config(model, cfg, [gel]))
    contacts = pipeline.contacts()
    sensor = HydroelasticGelSight(model, pipeline, gel, cfg)
    return model, state, pipeline, contacts, sensor, body, gel, spec


def press(name: str, depths_mm, cfg: GelSightConfig, verbose=True):
    model, state, pipeline, contacts, sensor, body, gel, spec = build(name, cfg)
    face_z = cfg.gel_thickness_m
    obj_shape = [i for i, l in enumerate(model.shape_label) if str(l) == "obj"][0]
    rows = []
    for d_mm in depths_mm:
        d = d_mm * 1e-3
        z = face_z - d - spec["zmin"]
        bq = state.body_q.numpy()
        bq[body] = (0.0, 0.0, z, 0.0, 0.0, 0.0, 1.0)
        state.body_q.assign(bq)
        # settle the sensor's shear memory: two collides, read the second
        pipeline.collide(state, contacts)
        sensor.reset()
        frame = sensor.sample(state, 1.0 / 60.0)
        surf = pipeline.hydroelastic_sdf.get_contact_surface()
        fc = int(surf.face_contact_count.numpy()[0])
        sd = surf.contact_surface_depth.numpy()[:fc]
        n = int(contacts.rigid_contact_count.numpy()[0])
        s0 = contacts.rigid_contact_shape0.numpy()[:n]
        s1 = contacts.rigid_contact_shape1.numpy()[:n]
        sel = ((s0 == gel) & (s1 == obj_shape)) | ((s1 == gel) & (s0 == obj_shape))
        k = contacts.rigid_contact_stiffness.numpy()[:n][sel]
        p0 = contacts.rigid_contact_point0.numpy()[:n][sel]
        p1 = contacts.rigid_contact_point1.numpy()[:n][sel]
        nrm = contacts.rigid_contact_normal.numpy()[:n][sel]
        sb = model.shape_body.numpy()
        # points are body-local; world them
        def world(p, s):
            bi = sb[s]
            if bi < 0:
                return p
            T = wp.transform(wp.vec3(*bq[bi][:3]), wp.quat(*bq[bi][3:]))
            return np.array(wp.transform_point(T, wp.vec3(*p)))
        dist = np.array([np.dot(world(b1, b) - world(a1, a), nn)
                         for a1, b1, a, b, nn in zip(p0, p1, s0[sel], s1[sel], nrm)]
                        ) if sel.any() else np.zeros(0)
        f_hydro = float(np.sum(k * np.maximum(-dist, 0.0))) if sel.any() else 0.0
        n_pen = int(np.sum(dist < 0.0)); n_mar = int(np.sum(dist >= 0.0))
        row = dict(
            object=name, depth_mm=d_mm, F_sensor=frame.total_normal_force_n,
            F_hydro=f_hydro, faces=fc, contacts=int(sel.sum()), pen=n_pen,
            margin=n_mar,
            k_sum=float(k.sum()) if sel.any() else 0.0,
            k_margin=float(k[dist >= 0].sum()) if sel.any() else 0.0,
            p_max_kPa=float(frame.pressure_pa.max()) * 1e-3,
            indent_max_mm=float(frame.normal_displacement_m.max()) * 1e3,
            surf_depth_um=(float(-sd.min()) * 1e6 if fc else 0.0),
            cells=int(frame.contact_mask.sum()),
        )
        rows.append(row)
        if verbose:
            print(f"{name:11s} d={d_mm:4.2f} mm | F_sensor {row['F_sensor']:7.3f} N  "
                  f"F_hydro {row['F_hydro']:7.3f} N  ratio {row['F_hydro']/max(row['F_sensor'],1e-9):6.2f} | "
                  f"faces {fc:6d} contacts {row['contacts']:3d} (pen {n_pen}, margin {n_mar}, "
                  f"k_margin {row['k_margin']:.2e}) | p_max {row['p_max_kPa']:6.1f} kPa "
                  f"indent_max {row['indent_max_mm']:.3f} mm cells {row['cells']}")
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--objects", default="box,sphere,nut,key_bow,key_bow_nb2,chop")
    ap.add_argument("--depths", default="0.05,0.1,0.2,0.4,0.8,1.2")
    ap.add_argument("--gel-kh", type=float, default=4.0e7)
    ap.add_argument("--stiffen", type=float, default=0.0,
                    help="gel_stiffening a (0 = linear law)")
    args = ap.parse_args()
    cfg = GelSightConfig(rows=96, cols=128, gel_kh_pa_per_m=args.gel_kh,
                         gel_stiffening=args.stiffen)
    print(f"gel_kh {args.gel_kh:.2e} stiffening a={args.stiffen:g}")
    depths = [float(x) for x in args.depths.split(",")]
    for name in args.objects.split(","):
        press(name, depths, cfg)
    return 0


if __name__ == "__main__":
    wp.init()
    raise SystemExit(main())


# ── solver mode: what indentation does SolverMuJoCo REALIZE for a pushed pad? ──
def solver_press(name: str, forces_n, cfg: GelSightConfig, ke: float = 8000.0,
                 substeps: int = 32, settle_s: float = 1.0, impratio: float = 10.0,
                 verbose: bool = True):
    newton.use_coord_layout_targets = True
    from hydrogel.mujoco_force_space import install as _fs_install
    _fs_install()
    """A prismatic-driven indenter (drive ke, overdrive sized for each force)
    pushed into the static pad under the suite's MuJoCo configuration
    (Newton contacts, impratio 10, elliptic cone, shape ke 1e7/kd 1e4, 32
    substeps).  Reports the REALIZED geometric indentation, the drive force
    ke*(target-q), the sensor integral and Newton's intended hydro force.
    This is the number the task sees; ``press`` reports the contact model's
    intent.  The indenter is a 0.1 kg link on a world prismatic along -z
    (SolverMuJoCo ignores contacts on kinematic world roots)."""

    b = newton.ModelBuilder()
    t = cfg.gel_thickness_m
    # THE PAD MUST BE A DYNAMIC BODY.  Measured 2026-08-22: a hydroelastic
    # contact between a driven box and a gel attached to the WORLD (body -1)
    # is not enforced at all by SolverMuJoCo with Newton contacts -- the box
    # goes exactly to its commanded indentation with zero drive force while
    # the sensor integrates 8.5 N (the plain-contact fallback pushes back
    # fine in the same scene).  So the pad here is a 1 kg link on a world
    # prismatic held by a 1e6 N/m drive (sags < 0.02 mm at 20 N), which is
    # what the FR3 mount is in the tasks: a gel on a driven finger link.
    as_mesh = os.environ.get("GS_GEL_MESH", "1") != "0"
    pad_body = b.add_link(xform=wp.transform_identity(), label="pad")
    # rigid backing 3.5 mm behind the face, like the FR3 mount (17.5x18.5)
    b.add_shape_box(pad_body, xform=wp.transform(wp.vec3(0.0, 0.0, t - 0.0035 - 0.0076),
                                                 wp.quat_identity()),
                    hx=0.00875, hy=0.00925, hz=0.0076,
                    cfg=newton.ModelBuilder.ShapeConfig(mu=0.8, gap=0.0008),
                    label="backing")
    jp = b.add_joint_prismatic(parent=-1, child=pad_body, axis=(0.0, 0.0, 1.0),
                               parent_xform=wp.transform_identity(),
                               target_pos=0.0, target_ke=1.0e6, target_kd=2000.0,
                               limit_lower=-0.05, limit_upper=0.05)
    b.add_articulation([jp])
    z0 = 0.05
    body = b.add_link(xform=wp.transform(wp.vec3(0.0, 0.0, z0), wp.quat_identity()),
                      label="indenter")
    if name == "box":
        h = 0.008
        b.add_shape_box(body, hx=h, hy=h, hz=h,
                        cfg=hydroelastic_object_cfg(cfg, density=200.0), label="obj")
        zmin = -h
    elif name == "sphere":
        r = 0.008
        b.add_shape_sphere(body, radius=r,
                           cfg=hydroelastic_object_cfg(cfg, density=200.0), label="obj")
        zmin = -r
    else:
        tm, sdf_kw = object_mesh(name)
        mesh, center = make_newton_mesh(tm, sdf=True, **sdf_kw)
        b.add_shape_mesh(body, mesh=mesh,
                         cfg=hydroelastic_object_cfg(cfg, density=200.0, mesh=True),
                         label="obj")
        v = tm.vertices
        over = v[(np.abs(v[:, 0]) < 0.5 * cfg.width_m) & (np.abs(v[:, 1]) < 0.5 * cfg.height_m)]
        zmin = float(over[:, 2].min() - center[2])
    j = b.add_joint_prismatic(parent=-1, child=body, axis=(0.0, 0.0, 1.0),
                              parent_xform=wp.transform(wp.vec3(0.0, 0.0, z0), wp.quat_identity()),
                              target_pos=0.0, target_ke=ke, target_kd=40.0,
                              limit_lower=-0.2, limit_upper=0.2)
    b.add_articulation([j])
    # the gel LAST so it is shape B of the hydroelastic pair (see attach_gelsight_pad)
    gel = attach_gelsight_pad(b, pad_body, cfg, xform=wp.transform_identity(),
                              label="gel", physics_thickness_m=2.0 * t,
                              as_mesh=as_mesh)
    if as_mesh:
        for s_ in b.body_shapes.get(pad_body, []):
            if s_ != gel:
                b.add_shape_collision_filter_pair(s_, gel)
    from force_bench.assets import set_body_mass
    set_body_mass(b, pad_body, 1.0)
    set_body_mass(b, body, 0.1)
    b.joint_armature[1] = 0.1      # like the FR3 finger dofs under the suite
    b.shape_material_mu_torsional = [max(v, 0.005) for v in b.shape_material_mu_torsional]
    b.shape_material_mu_rolling = [max(v, 1.0e-4) for v in b.shape_material_mu_rolling]
    newton.solvers.SolverMuJoCo.register_custom_attributes(b)
    model = b.finalize()
    model.shape_material_ke.fill_(1.0e7)
    model.shape_material_kd.fill_(1.0e4)
    mj = getattr(model, "mujoco", None)
    if mj is not None and hasattr(mj, "condim"):
        c_ = mj.condim.numpy(); c_[:] = 4; mj.condim.assign(c_)
    solver = newton.solvers.SolverMuJoCo(
        model, iterations=20, use_mujoco_contacts=False, nconmax=16384, njmax=65536,
        solver="newton", integrator="implicitfast", cone="elliptic",
        impratio=impratio, ls_iterations=100)
    pipeline = newton.CollisionPipeline(
        model, rigid_contact_max=20_000,
        broad_phase=os.environ.get("GS_BROAD", "explicit"),
        sdf_hydroelastic_config=winning_hydroelastic_config(model, cfg, [gel]))
    contacts = pipeline.contacts()
    sensor = HydroelasticGelSight(model, pipeline, gel, cfg)
    s0, s1 = model.state(), model.state()
    control = model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)
    # contact-kiss coordinate: object lowest point at the gel face
    q_touch = (t - zmin) - z0
    fps, dt = 60, 1.0 / 60.0
    sim_dt = dt / substeps
    rows = []
    for F in forces_n:
        q_target = q_touch - F / ke      # overdrive sized for F at zero indent
        for fi in range(int(settle_s * fps)):
            qt = control.joint_target_q.numpy(); qt[1] = q_target; control.joint_target_q.assign(qt)
            for si in range(substeps):
                s0.clear_forces(); pipeline.collide(s0, contacts)
                solver.step(s0, s1, control, contacts, sim_dt)
                s0, s1 = s1, s0
        pipeline.collide(s0, contacts)
        sensor.reset(); fr = sensor.sample(s0, dt)
        q = float(s0.joint_q.numpy()[1])
        zb = float(s0.body_q.numpy()[body][2])
        q_body = zb - z0                             # joint coord from the BODY pose
        indent = (q_touch - q_body)                  # geometric indentation
        f_drive = ke * (q_body - q_target)           # the drive's push (N)
        solver.update_contacts(contacts, s0)
        fc = contacts.rigid_contact_force.numpy()[:int(contacts.rigid_contact_count.numpy()[0])] if contacts.rigid_contact_force is not None else None
        # intended hydro force
        n = int(contacts.rigid_contact_count.numpy()[0])
        s0s = contacts.rigid_contact_shape0.numpy()[:n]; s1s = contacts.rigid_contact_shape1.numpy()[:n]
        obj_shape = [i for i, l in enumerate(model.shape_label) if str(l) == "obj"][0]
        sel = ((s0s == gel) & (s1s == obj_shape)) | ((s1s == gel) & (s0s == obj_shape))
        back = [i for i, l in enumerate(model.shape_label) if str(l) == "backing"][0]
        selb = ((s0s == back) & (s1s == obj_shape)) | ((s1s == back) & (s0s == obj_shape))
        f_solver = float(np.linalg.norm(fc[sel].sum(axis=0))) if (fc is not None and sel.any()) else float('nan')
        f_back = float(np.linalg.norm(fc[selb].sum(axis=0))) if (fc is not None and selb.any()) else 0.0
        row = dict(object=name, F_cmd=F, indent_mm=indent * 1e3, F_drive=f_drive,
                   F_sensor=fr.total_normal_force_n, n_gel_contacts=int(sel.sum()),
                   n_backing_contacts=int(selb.sum()),
                   p_max_kPa=float(fr.pressure_pa.max()) * 1e-3,
                   cells=int(fr.contact_mask.sum()))
        rows.append(row)
        if verbose:
            print(f"SOLVER {name:10s} a={cfg.gel_stiffening:g} F_cmd {F:5.1f} N | realized indent "
                  f"{row['indent_mm']:6.3f} mm (q_joint {q*1e3:.2f} q_body {q_body*1e3:.2f} mm) | drive {f_drive:6.2f} N | solver contact {f_solver:6.2f} N (+backing {f_back:.2f}) | sensor {row['F_sensor']:6.2f} N "
                  f"(ratio drive/sensor {f_drive/max(row['F_sensor'],1e-6):5.2f}) | gel contacts {row['n_gel_contacts']} "
                  f"backing contacts {row['n_backing_contacts']} | p_max {row['p_max_kPa']:.0f} kPa cells {row['cells']}")
    return rows
