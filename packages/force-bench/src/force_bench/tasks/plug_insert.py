"""Expert rollout: insert + extract the charger plug into the REAL outlet.

Bodies (assets/bodies/charging_block_outlet, cm -> m):
  shell            -> outlet face plate, static, SDF. Two polarized blade
                      slots (9 x 2.5 and 6.9 x 2.5 mm) + ground hole.
  spring_8mm       -> printed retention leaf inside the shell. REMOVED and
                      replaced by a pad on a prismatic drive = the spring.
  charging_block_3 -> charger brick; prongs found geometrically and rotated
                      to point down into the slots.

Trajectory: descend (insert) -> hold -> ascend (extract), driven by a
velocity rail. Outputs in outputs/plug_insert/: rollout.mp4, force csv/png
(axial insertion/extraction force = the signature force profile), states
npz, QA json (penetration, velocity spikes, retention check).

Run: python -m force_bench.tasks.plug_insert
"""

from __future__ import annotations

import json

import numpy as np
import trimesh
import warp as wp

import newton
JD = newton.ModelBuilder.JointDofConfig

from force_bench.rig import make_solver

from force_bench.probes import ForceProbe
from force_bench.paths import OUTPUT_DIR
from force_bench.qa import RolloutQA
from force_bench.recorder import RolloutRecorder
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass

# spring replacement (assets/manifest.yaml: charging_block_outlet)
PAD_KE = 8000.0
PAD_KD = 25.0
PAD_TRAVEL = 0.003
PAD_MASS = 0.010          # kg — keeps the stiff drive well-damped
PAD_PROTRUSION = 0.0015   # grip-face reach into the 2.5 mm blade channel
FLARE_DEG = 30.0          # entry flare angle — MUST exceed atan(mu) or the
                          # wedge self-locks (4 deg + mu 0.4 jams forever)

INSERT_SPEED = 0.04   # m/s
RAIL_KE = 20000.0     # N/m -- stiff enough that a 25 N push is 1.25 mm of
                      # lag, so the commanded ramp is tracked closely
RAIL_KD = 300.0
LAT_KE = 4000.0       # N/m -- lateral centring: soft enough that a 1 mm
                      # excursion costs 4 N, which a 20 N insertion can
                      # easily buy, and stiff enough to hold the plug on
                      # station while nothing is pushing it
LAT_KD = 40.0
ANG_KE = 8.0          # N.m/rad -- the same idea about the three rotations
ANG_KD = 0.2
RAIL_F_MAX = 30.0     # N -- the effort limit IS the physics here: a human
                      # pushes a plug home with tens of newtons, and the
                      # arm task's own insert peaked at 22.06 N
APPROACH = 0.004      # prong tips start this high above the face
DEPTH = 0.0160        # to the TRUE full seat: APPROACH + DEPTH = 20.0 mm.
                      # The old 8.5 mm came from "stop ~1 mm before the
                      # prongs bottom out", and the prongs never bottom out
                      # at all -- task_plug_insert_arm measured the slots as
                      # THROUGH-HOLES in a 2.5 mm face plate with an open
                      # cavity below, and the real stop is the plug's raised
                      # prong-base plateau landing flush on the outlet face
                      # at dz = 20.0 mm.  That task reaches 20.19 mm and
                      # passes; this rig was asking for less than a third of
                      # the stroke and then reporting it as a failure.
FPS = 60
SUBSTEPS = 16
MU = 0.40

CFG = dict(ke=1.0e5, kd=1.0e3, gap=0.0015, mu_torsional=0.0, mu_rolling=0.0)


def find_slots(shell: trimesh.Trimesh) -> list[dict]:
    """World-space blade-slot rectangles in the shell top face."""
    ztop = shell.bounds[1][2]
    sec = shell.section(plane_origin=[0, 0, ztop - 0.0015], plane_normal=[0, 0, 1])
    p2, T = sec.to_2D()
    slots = []
    for poly in p2.polygons_full:
        for hole in poly.interiors:
            pts2 = np.array(hole.coords)
            pts3 = trimesh.transformations.transform_points(
                np.column_stack([pts2, np.zeros(len(pts2))]), T)
            lo, hi = pts3.min(0), pts3.max(0)
            size = (hi - lo) * 1000
            if 1.5 < size[1] < 4.0 and size[0] > 5.0:   # blade slots: thin in y
                slots.append({"center": (lo + hi) / 2, "size_mm": size})
    slots.sort(key=lambda s: -s["size_mm"][0])  # widest first
    return slots


def find_prongs(plug: trimesh.Trimesh) -> dict:
    """Locate prongs as the thin protrusion range along x."""
    lo, hi = plug.bounds
    xs = np.linspace(lo[0] + 1e-4, hi[0] - 1e-4, 60)
    areas = []
    for x in xs:
        sec = plug.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
        if sec is None:
            areas.append(0.0)
            continue
        p2, _ = sec.to_2D()
        areas.append(sum(pp.area for pp in p2.polygons_full))
    areas = np.array(areas)
    body_area = np.percentile(areas[areas > 0], 80)
    thin = areas < 0.25 * body_area
    # prongs at whichever end is thin
    if thin[:10].sum() > thin[-10:].sum():
        prong_dir = -1
        prong_x = xs[thin & (xs < xs.mean())]
    else:
        prong_dir = 1
        prong_x = xs[thin & (xs > xs.mean())]
    prong_len = float(prong_x.max() - prong_x.min()) if len(prong_x) else 0.016
    tip_x = lo[0] if prong_dir < 0 else hi[0]
    # prong y positions: slice near the tip
    sec = plug.section(plane_origin=[tip_x - prong_dir * 0.002, 0, 0], plane_normal=[1, 0, 0])
    p2, T = sec.to_2D()
    prong_centers = []
    for pp in p2.polygons_full:
        b = np.array(pp.bounds)
        c2 = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2, 0.0]])
        c3 = trimesh.transformations.transform_points(c2, T)[0]
        prong_centers.append({"center": c3, "w_mm": (b[2] - b[0]) * 1000, "h_mm": (b[3] - b[1]) * 1000})
    prong_centers.sort(key=lambda p: -max(p["w_mm"], p["h_mm"]))
    return {"dir": prong_dir, "len": prong_len, "tip_x": tip_x, "prongs": prong_centers}


def main():
    out_dir = OUTPUT_DIR / "plug_insert"

    shell_tm = load_tm("charging_block_outlet", "shell_1_body1")
    plug_tm = load_tm("charging_block_outlet", "charging_block_3_1_body1")
    spring_tm = load_tm("charging_block_outlet", "spring_8mm_1_body1")

    slots = find_slots(shell_tm)
    pr = find_prongs(plug_tm)
    print(f"slots: {[np.round(s['size_mm'],1).tolist() for s in slots]}")
    print(f"prongs: dir={pr['dir']} len={pr['len']*1000:.1f}mm n={len(pr['prongs'])}")
    assert len(slots) >= 2 and len(pr["prongs"]) >= 2, "slot/prong detection failed"

    # ── pose the plug: prong axis (x*dir) -> -z (downward) ─────────────
    # R_y(+90deg) maps +x -> -z; for dir=-1 use -90deg
    ang = np.pi / 2 * pr["dir"]
    plug_posed = plug_tm.copy()
    plug_posed.apply_transform(trimesh.transformations.rotation_matrix(ang, [0, 1, 0]))

    def prong_footprints(mesh):
        """World-space xy rectangles of the downward prongs near the tips."""
        tipz = mesh.bounds[0][2]
        sec = mesh.section(plane_origin=[0, 0, tipz + 0.002], plane_normal=[0, 0, 1])
        p2, T = sec.to_2D()
        rects = []
        for pp in p2.polygons_full:
            if pp.area > 60e-6:   # prong cross-sections are tiny
                continue
            pts2 = np.array(pp.exterior.coords)
            pts3 = trimesh.transformations.transform_points(
                np.column_stack([pts2, np.zeros(len(pts2))]), T)
            lo, hi = pts3.min(0), pts3.max(0)
            rects.append({"center": (lo + hi) / 2, "ext": hi - lo})
        rects.sort(key=lambda r: -max(r["ext"][0], r["ext"][1]))
        return rects, tipz

    rects, tipz = prong_footprints(plug_posed)
    assert len(rects) >= 2, f"prong footprint detection failed ({len(rects)})"
    # blades' long axis must match the slots' long axis (x)
    wide = rects[0]
    if wide["ext"][1] > wide["ext"][0]:
        plug_posed.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi / 2, [0, 0, 1], point=wide["center"]))
        rects, tipz = prong_footprints(plug_posed)
        wide = rects[0]
    narrow = rects[1]
    print(f"prong footprints: wide {np.round(wide['ext']*1000,1)}mm at "
          f"{np.round(wide['center'][:2],4)}, narrow {np.round(narrow['ext']*1000,1)}mm")

    face_z = shell_tm.bounds[1][2]
    t_wide = slots[0]["center"]     # widest slot
    delta = np.array([t_wide[0] - wide["center"][0],
                      t_wide[1] - wide["center"][1],
                      face_z + APPROACH - tipz])
    plug_posed.apply_translation(delta)
    n_c = narrow["center"] + delta
    err = np.linalg.norm(n_c[:2] - slots[1]["center"][:2])
    print(f"narrow prong -> narrow slot error: {err*1000:.2f} mm")
    if err > 0.004:
        plug_posed.apply_transform(trimesh.transformations.rotation_matrix(
            np.pi, [0, 0, 1], point=t_wide))
        rects, _ = prong_footprints(plug_posed)
        err2 = np.linalg.norm(rects[1]["center"][:2] - slots[1]["center"][:2])
        print(f"flipped 180deg; new error: {err2*1000:.2f} mm")
        assert err2 < 0.004, "prong-slot alignment failed"

    # ── build model ─────────────────────────────────────────────────────
    shell_mesh, shell_c = make_newton_mesh(shell_tm, sdf=True, sdf_resolution=320)
    plug_mesh, plug_c = make_newton_mesh(plug_posed, sdf=True, sdf_resolution=320)

    builder = newton.ModelBuilder(gravity=(0.0, 0.0, -9.81))
    builder.add_ground_plane()
    pla = newton.ModelBuilder.ShapeConfig(mu=MU, density=1240.0, **CFG)

    # visible table slab (the infinite ground plane doesn't render in the
    # tiled camera): top surface flush with z=0
    builder.add_shape_box(
        -1, xform=wp.transform(wp.vec3(0.14, 0.10, -0.005), wp.quat_identity()),
        hx=0.25, hy=0.25, hz=0.005, cfg=pla, label="table",
    )

    shell_shape = builder.add_shape_mesh(
        -1, mesh=shell_mesh,
        xform=wp.transform(wp.vec3(*shell_c), wp.quat_identity()),
        cfg=pla, label="shell",
    )

    # ── retention contacts replacing spring_8mm ─────────────────────────
    # One angled pad per blade channel, pressing the blade's wide face
    # sideways (±y) inside the shell cavity — the joint drive is the spring.
    # The pad is tilted about x so the blade meets a ramp on entry.
    slot_halfwidth = 0.00125          # 2.5 mm channel
    pad_hx, pad_hy = 0.004, 0.0006    # blade-width coverage, plate half-thickness
    grip_hz = 0.0045
    flare = np.deg2rad(FLARE_DEG)
    flare_drop = PAD_PROTRUSION / np.tan(flare)   # vertical extent of the flare
    flare_hl = PAD_PROTRUSION / np.sin(flare) / 2  # half-length of flare plate
    pad_bodies, pad_names = [], []
    face_z_w = shell_tm.bounds[1][2]
    for slot, side, name in ((slots[0], -1, "pad_wide"), (slots[1], +1, "pad_narrow")):
        sy = slot["center"][1]
        y_edge = sy + side * slot_halfwidth              # channel edge (pad side)
        y_grip = sy + side * (slot_halfwidth - PAD_PROTRUSION)  # grip face plane
        z_top = face_z_w - 0.0005                        # flare crest at the face
        z_mid = z_top - flare_drop                       # flare -> grip transition
        # body frame: center of the grip plate
        body_pos = wp.vec3(slot["center"][0], y_grip + side * pad_hy, z_mid - grip_hz)
        xf = wp.transform(body_pos, wp.quat_identity())
        body = builder.add_link(xform=xf, label=name)
        cfg_pad = newton.ModelBuilder.ShapeConfig(mu=MU, density=4000.0, **CFG)
        shapes = []
        # grip plate: vertical, constant protrusion
        shapes.append(builder.add_shape_box(
            body, hx=pad_hx, hy=pad_hy, hz=grip_hz, cfg=cfg_pad,
            label=name + "_grip",
        ))
        # entry flare: tilted plate from the channel edge down to the grip
        # face; local +z of the plate points up-and-outward (R_x(-side*ang))
        fl_cy = (y_edge + y_grip) / 2 - body_pos[1]
        fl_cz = (z_top + z_mid) / 2 - body_pos[2]
        q_fl = wp.quat_from_axis_angle(
            wp.vec3(1.0, 0.0, 0.0), float(-side * flare))
        shapes.append(builder.add_shape_box(
            body, xform=wp.transform(wp.vec3(0.0, float(fl_cy), float(fl_cz)), q_fl),
            hx=pad_hx, hy=pad_hy, hz=float(flare_hl), cfg=cfg_pad,
            label=name + "_flare",
        ))
        # the replacement spring interacts ONLY with the plug blades — the
        # printed leaf lived inside the shell wall it would otherwise fight
        for s in shapes:
            builder.add_shape_collision_filter_pair(s, shell_shape)
        j = builder.add_joint_prismatic(
            parent=-1, child=body, axis=(0.0, float(side), 0.0),  # +q = retract
            parent_xform=xf, child_xform=wp.transform_identity(),
            target_pos=0.0, target_ke=PAD_KE, target_kd=PAD_KD,
            limit_lower=0.0, limit_upper=PAD_TRAVEL,
        )
        builder.add_articulation([j])
        set_body_mass(builder, body, PAD_MASS)
        pad_bodies.append(body)
        pad_names.append(name)

    plug_xf = wp.transform(wp.vec3(*plug_c), wp.quat_identity())
    plug_body = builder.add_link(xform=plug_xf, label="plug")
    builder.add_shape_mesh(plug_body, mesh=plug_mesh, cfg=pla, label="plug_shape")
    # A COMPLIANT MOUNT, NOT A RAIL.  The prismatic version pinned the plug
    # to one line -- measured 0.000 mm of lateral drift and 0.000 deg of
    # tilt for the whole rollout -- so it could not self-align and stopped
    # at 6-8 mm of a 20 mm seat however hard it was pushed.
    # task_plug_insert_arm reaches 20.19 mm at a LOWER peak force (22.06 N)
    # because the fingers let the plug shift 0.22 mm in x and 0.71 mm in y
    # on the way in.  That shift is not a fixed offset to copy: pre-aligning
    # the rail by the measured delta made it WORSE (7.9 -> 4.5 mm), because
    # the alignment is continuous, not a constant.
    # So the axis gets a real drive and the other five DOFs get soft
    # centring springs -- which is what a remote-centre-compliance device is
    # for, and what every industrial insertion fixture actually has.
    rail = builder.add_joint_d6(
        parent=-1, child=plug_body,
        linear_axes=(JD(axis=(1.0, 0.0, 0.0), target_ke=LAT_KE,
                        target_kd=LAT_KD),
                     JD(axis=(0.0, 1.0, 0.0), target_ke=LAT_KE,
                        target_kd=LAT_KD),
                     JD(axis=(0.0, 0.0, -1.0), target_ke=RAIL_KE,
                        target_kd=RAIL_KD)),
        angular_axes=tuple(JD(axis=a, target_ke=ANG_KE, target_kd=ANG_KD)
                           for a in ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                                     (0.0, 0.0, 1.0))),
        parent_xform=plug_xf, child_xform=wp.transform_identity(),
        # A POSITION DRIVE, not a velocity one.  target_ke=0/kd=600 has no
        # position stiffness, so its force is capped at kd*v_cmd = 600*0.04
        # = 24 N NO MATTER WHAT RESISTS IT -- and the plug parked at 6.0 mm
        # of a 20 mm seat.  task_plug_insert_arm says the same thing about
        # this rig in its own header ("the rail version stopped at 9.6 mm
        # and rebounded ~3 mm during hold because its velocity drive has no
        # position stiffness"), and it reaches 20.19 mm on ARM POSITION
        # drives.  A position drive's force grows with the lag until the
        # effort limit stops it, which is what a hand does.
    )
    builder.add_articulation([rail])
    builder.joint_effort_limit[builder.joint_qd_start[rail]] = RAIL_F_MAX
    set_body_mass(builder, plug_body, 0.06)

    model = builder.finalize()
    solver = make_solver(model, iterations=10)
    s0, s1 = model.state(), model.state()
    control = model.control()

    plug_probe = ForceProbe(model, sensing_body="plug")
    padw_probe = ForceProbe(model, sensing_body="pad_wide")
    padn_probe = ForceProbe(model, sensing_body="pad_narrow")

    # rigid_contact_max must cover the solver's nconmax or SolverMuJoCo
    # refuses to run ("naconmax (8000) exceeds rigid_contact_max (1000)").
    # These standalone rollouts build their own pipeline instead of going
    # through task_rig, which is why they alone crashed under MuJoCo.
    pipeline = newton.CollisionPipeline(model, rigid_contact_max=20000,
                                        reduce_contacts=True,
                                        broad_phase="sap")
    contacts = pipeline.contacts()
    newton.eval_fk(model, model.joint_q, model.joint_qd, s0)

    center = np.array(shell_c)
    rec = RolloutRecorder(
        model, out_dir,
        cam_pos=center + np.array([0.16, -0.20, 0.15]),
        cam_target=center + np.array([0.0, 0.0, 0.03]),
        fps=FPS,
    )

    frame_dt = 1.0 / FPS
    sim_dt = frame_dt / SUBSTEPS
    # PHASES END ON MEASURED DEPTH, NOT ON THE CLOCK.  t_insert was
    # distance/speed, which is only right if the velocity drive TRACKS its
    # command -- true under XPBD, where a damping-only drive is a
    # kinematic weld, and false under MuJoCo, where it is a force source
    # worth kd*(v_cmd - v) and tops out at 600*0.04 = 24 N (which is
    # exactly the 24.2 N spike in the log).  The plug averaged 13 mm/s
    # against a commanded 40, so the phase expired at 4.1 mm of a 12.5 mm
    # stroke and the "extraction" that followed measured 0.0 N because
    # there was nothing left to extract.  A person pushes a plug until it
    # is seated, not for 0.31 seconds.
    t_insert = (APPROACH + DEPTH) / INSERT_SPEED       # now a CEILING
    t_hold = 0.4
    t_extract = t_insert
    total_t = 2.5 * t_insert + t_hold + t_extract + 0.3
    seat_depth = APPROACH + DEPTH - 0.0005   # 0.5 mm short of the stop
    n_frames = int(total_t * FPS)
    # the d6 mount contributes SIX dofs; the insertion axis is its third
    # linear one, so the index is no longer "the dof after the pads"
    rail_dof = 2 + 2   # pads = dofs 0,1; d6 linear x,y,z -> z is dof 4
    states = {"a": s0, "b": s1}
    sim_time = 0.0
    max_insert, max_extract = 0.0, 0.0

    print(f"rollout: {n_frames} frames")
    ph = {"name": "insert", "t_end": None}
    rail_cmd = 0.0
    for fi in range(n_frames):
        depth = float(s0.joint_q.numpy()[rail_dof]) if hasattr(
            s0, "joint_q") else 0.0
        if ph["name"] == "insert":
            # seated, or out of patience: 2.5x the ideal-tracking time
            if depth >= seat_depth or sim_time > 2.5 * t_insert:
                ph["name"], ph["t_end"] = "hold", sim_time + t_hold
                print(f"t={sim_time:.2f}s seated at {depth*1000:.2f} mm "
                      f"of {seat_depth*1000:.2f}")
        elif ph["name"] == "hold" and sim_time >= ph["t_end"]:
            ph["name"] = "extract"
        phase = ph["name"]
        vel = (INSERT_SPEED if phase == "insert"
               else 0.0 if phase == "hold" else -INSERT_SPEED)
        # ramp the POSITION target at the commanded speed; the velocity
        # target rides along as feed-forward
        rail_cmd = float(np.clip(rail_cmd + vel * frame_dt,
                                 -0.01, APPROACH + DEPTH))
        qt = control.joint_target_q.numpy()
        qt[rail_dof] = rail_cmd
        control.joint_target_q.assign(qt)
        qd = control.joint_target_qd.numpy()
        qd[rail_dof] = vel
        control.joint_target_qd.assign(qd)

        for _ in range(SUBSTEPS):
            sa, sb = states["a"], states["b"]
            sa.clear_forces()
            pipeline.collide(sa, contacts)
            solver.step(sa, sb, control, contacts, sim_dt)
            states["a"], states["b"] = sb, sa
        sim_time += frame_dt

        sa = states["a"]
        solver.update_contacts(contacts, sa)
        rp = plug_probe.sample(contacts, sa, sim_time)
        rw = padw_probe.sample(contacts, sa, sim_time)
        rn = padn_probe.sample(contacts, sa, sim_time)
        pad_total = rw["force"] + rn["force"]
        axial = abs(rp["vec"][2])
        if phase == "insert":
            max_insert = max(max_insert, axial)
        elif phase == "extract":
            max_extract = max(max_extract, axial)
        rec.capture(sa, sim_time, forces={
            "plug_force_N": round(rp["force"], 4),
            "plug_axial_N": round(axial, 4),
            "plug_friction_N": round(rp["friction"], 4),
            "pad_load_N": round(pad_total, 4),
        })
        if fi % 15 == 0:
            z = sa.body_q.numpy()[plug_body][2] - plug_c[2]
            print(f"t={sim_time:5.2f}s [{phase:7s}] dz={z*1000:+6.1f}mm "
                  f"|F|={rp['force']:6.2f}N axial={axial:6.2f}N pads={pad_total:5.2f}N")

    paths = rec.save("rollout", force_keys=["plug_axial_N", "plug_friction_N", "pad_load_N"])
    print("saved:", paths)
    print(f"max insertion force {max_insert:.2f} N | max extraction force {max_extract:.2f} N")

    qa = RolloutQA(paths["states"])
    qa.check_finite()
    qa.check_velocity(vmax=1.0)
    lo = center - np.array([0.15, 0.15, 0.05])
    hi = center + np.array([0.15, 0.15, 0.25])
    qa.check_envelope(["plug", "pad_wide", "pad_narrow"], lo, hi)

    shell_local = shell_tm.copy(); shell_local.apply_translation(-shell_c)
    plug_local = plug_posed.copy(); plug_local.apply_translation(-plug_c)
    d = np.load(paths["states"], allow_pickle=True)
    pq = trimesh.proximity.ProximityQuery(shell_local)
    frames = np.linspace(0, len(d["t"]) - 1, 24).astype(int)
    pi = list(d["body_labels"]).index("plug")
    va = plug_local.vertices
    va = va[np.random.RandomState(0).choice(len(va), min(900, len(va)), replace=False)]
    worst, worst_t = 0.0, None
    for f in frames:
        q = d["body_q"][f, pi]
        T = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
        T[:3, 3] = q[:3] - shell_c
        pts = (T @ np.hstack([va, np.ones((len(va), 1))]).T).T[:, :3]
        sd = pq.signed_distance(pts)
        depth = float(sd.max()) if len(sd) else 0.0
        if depth > worst:
            worst, worst_t = depth, float(d["t"][f])
    qa.report["checks"]["penetration"] = {
        "pass": bool(worst < 0.0015), "tolerance_mm": 1.5,
        "pairs": {"plug->shell(static)": {
            "max_depth_mm": round(worst * 1000, 3), "at_t": worst_t,
            "pass": bool(worst < 0.0015)}},
    }
    qa.report["pass"] &= worst < 0.0015

    # task: reached depth during hold, and extraction met resistance
    zmax = float((plug_c[2] - d["body_q"][:, pi, 2]).max())
    qa.report["checks"]["task_success"] = {
        "pass": bool(zmax > DEPTH * 0.7 and max_extract > 0.5),
        "max_depth_mm": round(zmax * 1000, 1),
        "max_insert_N": round(max_insert, 2),
        "max_extract_N": round(max_extract, 2),
        "target_bands_N": {"insert": [8, 15], "extract": [6, 12]},
    }
    qa.report["pass"] &= zmax > DEPTH * 0.7 and max_extract > 0.5

    qa.save(out_dir / "rollout_qa.json")
    report = json.load(open(out_dir / "rollout_qa.json"))
    print(json.dumps(report, indent=1))
    print("QA PASS" if report["pass"] else "QA FAIL")


if __name__ == "__main__":
    wp.init()
    main()
