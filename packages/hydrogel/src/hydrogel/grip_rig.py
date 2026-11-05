"""GelSight-on-gripper regression rig + grasp-creep diagnosis matrix.

Port of ``integrations/isaaclab/gelsight_grip_rollout.py`` to newton 1.5 (env
``tactile-newton``), importing the canonical sensor from ``gelsight_sensor``.
Same scene idea, same phase script, same five asserted checks: one gel pad on
EACH jaw of a parallel gripper, a free 2.6 g sphere between them, grasp ->
lift -> shake -> hold -> release, exit code 1 while any check fails.

Deliberate changes from the original:

* The jaws are PRISMATIC-DRIVEN links in one articulation (carrier z for
  lift, carrier y for shake, one x slide per pad), not teleported kinematic
  bodies.  SolverMuJoCo converts kinematic bodies outside an articulation
  as "standalone world roots" whose contacts are never enforced
  (probe_grip_slip.py), and driven joints are also what the FR3 really has.
* The sensors are sampled right after the frame-start ``collide()`` --
  same data one frame earlier, half the hydroelastic cost, and the exact
  cadence the Rig integration uses.
* Solver and hydroelastic knobs are env flags so the historical "the sphere
  creeps out of the grasp" failure can be bisected.  The documented
  "hydroelastic exports zero friction" diagnosis was wrong: XPBD reads
  ``model.shape_material_mu`` and ignores the per-contact friction scale.
  The measured mechanism (probe_static_pinch, 2026-08-20) is XPBD friction
  CONVERGENCE: the per-iteration Coulomb budget is proportional to the
  incremental normal correction, so at 10 iterations a static pinch creeps
  ~1.6 mm/s while 100 iterations hold it exactly; SolverMuJoCo instead
  consumes the hydroelastic per-contact stiffness and solves an implicit
  friction cone.

Flags (defaults = the MEASURED WINNER, 5/5 checks, 2026-08-20):

    (solver)          SolverMuJoCo with use_mujoco_contacts=False (Newton
                      contacts, per-contact hydroelastic stiffness ->
                      solref) per force_bench.rig.make_solver's notes.  The
                      XPBD path, which reproduced the historical creep
                      failure, was removed on 2026-09-11.
    GS_REDUCE=1|0     solver-side hydroelastic contact reduction
                      (pre_prune_contacts stays False: the sensor needs the
                      dense surface, which reduction does NOT decimate).
                      OFF floods MuJoCo with ~5k compliant contacts, which
                      oscillate and eject the sphere during the shake.
    GS_ANCHOR=1|0     anchor_contact (center-of-pressure anchoring; only
                      meaningful with GS_REDUCE=1)
    GS_MOMENT=0|1     moment_matching (adjusts the per-contact friction
                      scale; MuJoCo multiplies mu by it)
    GS_ITERS=10       solver iterations (mujoco floors at 15)
    GS_SUBSTEPS=4     solver substeps per frame (frame-cadence collide keeps
                      the friction anchors, so substeps are convergence-only)
    GS_CADENCE=frame|substep   collide once per frame (docs/physics_notes.md pitfall 5.1)
                      or once per substep
    GS_OBJKH=1e10     object hydroelastic modulus [Pa/m]
    GS_IMPRATIO=10    mujoco impratio: friction-vs-normal impedance ratio.
                      1.0 leaves ~2 mm/s of regularization creep; 10 cuts
                      it to ~0.06 mm/s.  REQUIRES the elliptic cone --
                      impratio 10 on the default pyramidal cone explodes.
    GS_CONE=elliptic  mujoco friction cone (""|pyramidal|elliptic)

Baseline reproduction of the historical failure needed GS_SOLVER=xpbd,
which went with the bench's other XPBD paths on 2026-09-11.

Run (CUDA required):

    PYTHONNOUSERSITE=1 python -m hydrogel.grip_rig

Writes ``outputs/gelsight_grip_rig/grip_rollout_panel.mp4`` plus a
final-hold npz per pad, and prints one greppable ``[gs-matrix]`` line.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

import numpy as np
import warp as wp

import newton
from newton.geometry import HydroelasticSDF

from hydrogel.sensor import (
    GELSIGHT_OUTPUT_DIR,
    GelSightConfig,
    HydroelasticGelSight,
    TactileFrame,
    attach_gelsight_pad,
    make_object_shape_config,
    _save_frame,
)

SPHERE_R = 0.007          # m — the grasped object
SPHERE_DENSITY = 1800.0   # kg/m^3 -> 2.6 g; weight 0.025 N
INDENT = 0.0006           # m of commanded gel indentation per pad
GRASP_DZ = 0.0015         # m above the sphere centre: a gravity-restoring
                          # pendulum grasp rather than a neutral one
LIFT = 0.030              # m
SHAKE = 0.003             # m of in-plane (world y) wiggle
T_SETTLE, T_CLOSE, T_LIFT, T_SHAKE, T_HOLD, T_OPEN, T_DROP = (
    0.3, 0.5, 0.8, 0.8, 0.4, 0.3, 0.4)
FPS = 60

# Drive gains.  The carrier is deliberately heavy and stiff (XPBD drive
# authority scales with the driver body's inertia); the pad slides deliver
# grip force as ke * overdrive, so the close target overdrives past the
# commanded indent and the checks gate on MEASURED pad force.
CARRIER_MASS = 2.0
PAD_MASS = 0.1
CARRIER_KE, CARRIER_KD = 1.0e5, 1.0e3
PAD_KE, PAD_KD = 2.0e4, 1.0e2

# ── diagnosis-matrix flags ─────────────────────────────────────────────
GS_SOLVER = "mujoco"          # the only backend (XPBD removed 2026-09-11)
GS_REDUCE = os.environ.get("GS_REDUCE", "1") == "1"
GS_ANCHOR = os.environ.get("GS_ANCHOR", "1") == "1"
GS_MOMENT = os.environ.get("GS_MOMENT", "0") == "1"
GS_ITERS = int(os.environ.get("GS_ITERS", "10"))
GS_SUBSTEPS = int(os.environ.get("GS_SUBSTEPS", "4"))
GS_CADENCE = os.environ.get("GS_CADENCE", "frame")
assert GS_CADENCE in ("frame", "substep"), GS_CADENCE
GS_OBJKH = float(os.environ.get("GS_OBJKH", "1e10"))
GS_IMPRATIO = float(os.environ.get("GS_IMPRATIO", "10.0"))  # mujoco only
GS_CONE = os.environ.get("GS_CONE", "elliptic")            # ""|pyramidal|elliptic

SUBSTEPS = GS_SUBSTEPS


def make_matrix_pipeline(model: newton.Model) -> newton.CollisionPipeline:
    """The sensor pipeline with the experiment knobs applied."""

    hydro_config = HydroelasticSDF.Config(
        output_contact_surface=True,
        reduce_contacts=GS_REDUCE,
        pre_prune_contacts=False,
        anchor_contact=GS_ANCHOR,
        moment_matching=GS_MOMENT,
        buffer_fraction=1.0,
        buffer_mult_iso=2,
        buffer_mult_contact=2,
        mc_edge_clamp_min=0.0,
    )
    return newton.CollisionPipeline(
        model,
        rigid_contact_max=20_000,
        broad_phase="explicit",
        sdf_hydroelastic_config=hydro_config,
    )


def make_rig_solver(model: newton.Model):
    # Migration notes in task_rig.make_solver: use_mujoco_contacts=False
    # is what makes MuJoCo consume the contacts Newton computed (incl.
    # the hydroelastic per-contact stiffness -> solref); nconmax must
    # cover the dense hydroelastic contact count; XPBD-era ke/kd
    # defaults (1e5/1e3) are untuned, 1e7/1e4 is the measured setting
    # for the plain (pedestal) contacts.
    model.shape_material_ke.fill_(1.0e7)
    model.shape_material_kd.fill_(1.0e4)
    kw = {}
    if GS_CONE:
        kw["cone"] = GS_CONE
    return newton.solvers.SolverMuJoCo(
        model,
        iterations=max(GS_ITERS, 15),
        use_mujoco_contacts=False,
        nconmax=16384,
        njmax=65536,
        impratio=GS_IMPRATIO,
        **kw,
    )


def smooth(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return 3.0 * u * u - 2.0 * u ** 3


OPEN_D = SPHERE_R + 0.004  # pad-face distance from grasp axis when open


def build_scene(config: GelSightConfig, device: str):
    builder = newton.ModelBuilder()

    # pedestal the object starts on and falls back to (plain contact)
    builder.add_shape_box(
        body=-1,
        xform=wp.transform(wp.vec3(0.0, 0.0, -0.005), wp.quat_identity()),
        hx=0.04, hy=0.04, hz=0.005,
        cfg=newton.ModelBuilder.ShapeConfig(mu=0.5),
        label="pedestal",
    )

    obj = builder.add_body(
        xform=wp.transform(wp.vec3(0.0, 0.0, SPHERE_R), wp.quat_identity()),
        label="object",
    )
    obj_cfg = make_object_shape_config(config)
    obj_cfg.density = SPHERE_DENSITY
    obj_cfg.kh = GS_OBJKH
    builder.add_shape_sphere(body=obj, radius=SPHERE_R, cfg=obj_cfg,
                             label="object_sphere")

    # Gripper articulation: world -z-> carrier_z -y-> carrier_y, then one x
    # slide per pad.  q = 0 everywhere is the open pose at grasp height.
    # Each pad's gel local +z (its sensing face) is rotated onto the world
    # grasp axis, so the left image's +x column axis maps to world -z and
    # the right image mirrors it.
    half_pi = float(np.pi / 2)
    qL = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), half_pi)   # +z -> +x
    qR = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), -half_pi)  # +z -> -x
    hz = 0.5 * config.gel_thickness_m
    z0 = SPHERE_R + GRASP_DZ

    xf_z = wp.transform(wp.vec3(0.0, 0.0, z0 + 0.06), wp.quat_identity())
    carrier_z = builder.add_link(xform=xf_z, mass=CARRIER_MASS,
                                 label="carrier_z")
    j_z = builder.add_joint_prismatic(
        parent=-1, child=carrier_z, axis=(0.0, 0.0, 1.0), parent_xform=xf_z,
        target_ke=CARRIER_KE, target_kd=CARRIER_KD)

    xf_y = wp.transform(wp.vec3(0.0, 0.0, 0.0), wp.quat_identity())
    carrier_y = builder.add_link(xform=xf_z, mass=CARRIER_MASS,
                                 label="carrier_y")
    j_y = builder.add_joint_prismatic(
        parent=carrier_z, child=carrier_y, axis=(0.0, 1.0, 0.0),
        parent_xform=xf_y, target_ke=CARRIER_KE, target_kd=CARRIER_KD)

    pads, pad_joints, gels = {}, {}, {}
    for name, sign, quat in (("padL", -1.0, qL), ("padR", +1.0, qR)):
        # pad bodies stay unrotated; the gel's local mount transform carries
        # the sensing-face rotation (the same pattern the FR3 mount uses)
        xf_pad_w = wp.transform(
            wp.vec3(sign * (OPEN_D + hz), 0.0, z0), wp.quat_identity())
        pad = builder.add_link(xform=xf_pad_w, mass=PAD_MASS, label=name)
        # joint axis points TOWARD the sphere for both pads so q >= 0 closes
        xf_parent = wp.transform(
            wp.vec3(sign * (OPEN_D + hz), 0.0, -0.06), wp.quat_identity())
        j = builder.add_joint_prismatic(
            parent=carrier_y, child=pad, axis=(-sign, 0.0, 0.0),
            parent_xform=xf_parent,
            target_ke=PAD_KE, target_kd=PAD_KD)
        gels[name] = attach_gelsight_pad(
            builder, pad, config,
            xform=wp.transform(wp.vec3(0.0, 0.0, 0.0), quat),
            label=name.replace("pad", "gel"))
        pads[name] = pad
        pad_joints[name] = j

    gelL, gelR = gels["padL"], gels["padR"]
    joints = {"z": j_z, "y": j_y, "padL": pad_joints["padL"],
              "padR": pad_joints["padR"]}
    builder.add_articulation(list(joints.values()), label="gripper")

    newton.use_coord_layout_targets = True
    model = builder.finalize(device=device)
    state0, state1 = model.state(), model.state()
    control = model.control()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state0)
    pipeline = make_matrix_pipeline(model)
    contacts = pipeline.contacts()
    solver = make_rig_solver(model)
    sensL = HydroelasticGelSight(model, pipeline, gelL, config)
    sensR = HydroelasticGelSight(model, pipeline, gelR, config)
    body_index = {"object": obj, "padL": pads["padL"], "padR": pads["padR"]}
    target_starts = {
        name: int(model.joint_target_q_start.numpy()[j])
        for name, j in joints.items()}
    return (model, state0, state1, control, pipeline, contacts, solver,
            sensL, sensR, body_index, target_starts)


def finger_targets(t: float):
    """(pad-face distance from the grasp axis, z, y) at time ``t``."""
    closed_d = SPHERE_R - INDENT
    z, y = SPHERE_R + GRASP_DZ, 0.0
    t1 = T_SETTLE
    t2 = t1 + T_CLOSE
    t3 = t2 + T_LIFT
    t4 = t3 + T_SHAKE
    t5 = t4 + T_HOLD
    if t < t1:
        d = OPEN_D
    elif t < t2:
        d = OPEN_D + (closed_d - OPEN_D) * smooth((t - t1) / T_CLOSE)
    else:
        d = closed_d
    if t >= t2:
        z += LIFT * smooth(min(1.0, (t - t2) / T_LIFT))
    if t3 <= t < t4:
        y = SHAKE * np.sin(2.0 * np.pi * 2.0 * (t - t3) / T_SHAKE)
    if t >= t5:
        d = closed_d + (OPEN_D - closed_d) * smooth((t - t5) / T_OPEN)
    return d, z, y


def phase_creep_mm_s(tt: np.ndarray, rel_z: np.ndarray, t_from: float,
                     t_to: float) -> float:
    """Slope of the object's grip-frame z over one phase, in mm/s."""

    sel = (tt >= t_from) & (tt < t_to)
    if sel.sum() < 2:
        return 0.0
    slope = np.polyfit(tt[sel], rel_z[sel], 1)[0]
    return float(slope * 1.0e3)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--rows", type=int, default=32)
    parser.add_argument("--cols", type=int, default=42)
    parser.add_argument(
        "--out", default=str(GELSIGHT_OUTPUT_DIR / "gelsight_grip_rig" / "grip_rollout"))
    parser.add_argument("--video", default=None,
                        help="panel video path ('' disables); default "
                             "<out>_panel.mp4")
    args = parser.parse_args()

    config = GelSightConfig(rows=args.rows, cols=args.cols)
    (model, s0, s1, control, pipeline, contacts, solver,
     sensL, sensR, bi, tstart) = build_scene(config, args.device)

    t_end = T_SETTLE + T_CLOSE + T_LIFT + T_SHAKE + T_HOLD + T_OPEN + T_DROP
    frame_dt = 1.0 / FPS
    sim_dt = frame_dt / SUBSTEPS

    frames_L: list[TactileFrame] = []
    frames_R: list[TactileFrame] = []
    obj_z: list[float] = []
    rel_z: list[float] = []      # object z relative to the commanded grip z
    contact_counts: list[int] = []
    n = int(round(t_end * FPS))
    for k in range(n):
        t = k * frame_dt
        d, z, y = finger_targets(t)
        qt = control.joint_target_q.numpy()
        qt[tstart["z"]] = z - (SPHERE_R + GRASP_DZ)
        qt[tstart["y"]] = y
        # overdrive past the commanded indent: grip force = ke * overdrive
        qt[tstart["padL"]] = OPEN_D - d
        qt[tstart["padR"]] = OPEN_D - d
        control.joint_target_q.assign(qt)

        # Collide once per FRAME by default (docs/physics_notes.md pitfall
        # 22): XPBD dry friction is a positional constraint on the anchors
        # collide() records, so re-anchoring inside the substep loop lets
        # the pinch creep.  GS_CADENCE=substep measures that regression for
        # hydroelastic contacts.  The sensors sample straight off this
        # collide -- no second per-frame collide.
        if GS_CADENCE == "frame":
            pipeline.collide(s0, contacts)
            surface = pipeline.hydroelastic_sdf.get_contact_surface()
            frames_L.append(sensL.sample(s0, frame_dt, surface=surface))
            frames_R.append(sensR.sample(s0, frame_dt, surface=surface))
            contact_counts.append(int(contacts.rigid_contact_count.numpy()[0]))
        for si in range(SUBSTEPS):
            if GS_CADENCE == "substep":
                pipeline.collide(s0, contacts)
                if si == 0:
                    surface = pipeline.hydroelastic_sdf.get_contact_surface()
                    frames_L.append(
                        sensL.sample(s0, frame_dt, surface=surface))
                    frames_R.append(
                        sensR.sample(s0, frame_dt, surface=surface))
                    contact_counts.append(
                        int(contacts.rigid_contact_count.numpy()[0]))
            solver.step(s0, s1, control, contacts, sim_dt)
            s0, s1 = s1, s0

        z_now = float(s0.body_q.numpy()[bi["object"], 2])
        obj_z.append(z_now)
        rel_z.append(z_now - z)

    # ── validation ─────────────────────────────────────────────────────
    tt = np.arange(n) * frame_dt
    t1 = T_SETTLE
    t2 = t1 + T_CLOSE
    t3 = t2 + T_LIFT
    t4 = t3 + T_SHAKE
    hold0 = t4
    hold = (tt >= hold0) & (tt < hold0 + T_HOLD)
    end = tt >= t_end - 0.5 * T_DROP
    fL = np.array([f.total_normal_force_n for f in frames_L])
    fR = np.array([f.total_normal_force_n for f in frames_R])
    rise_hold = float(np.median(np.array(obj_z)[hold]) - SPHERE_R)
    coulomb_ok = True
    for f in frames_L + frames_R:
        shear = np.linalg.norm(f.shear_traction_pa, axis=0)
        bound = config.friction_coefficient * f.pressure_pa + 1.0
        if (shear > bound * 1.05 + 50.0).any():
            coulomb_ok = False
    shear_hold = max(
        float(np.linalg.norm(f.shear_displacement_m, axis=0).max())
        for f, h in zip(frames_L, hold) if h)
    checks = {
        "lifted_by_gel_friction": rise_hold > 0.8 * LIFT,
        "both_pads_loaded_through_hold": bool(
            (fL[hold] > 0.2).all() and (fR[hold] > 0.2).all()),
        "shear_carries_the_weight": shear_hold > 1.0e-5,
        "coulomb_bound_respected": coulomb_ok,
        "released_cleanly": bool(
            fL[end].max() < 0.05 and fR[end].max() < 0.05
            and np.array(obj_z)[end][-1] < SPHERE_R + 0.005),
    }
    rel = np.array(rel_z)
    creep = {
        "lift": phase_creep_mm_s(tt, rel, t2, t3),
        "shake": phase_creep_mm_s(tt, rel, t3, t4),
        "hold": phase_creep_mm_s(tt, rel, t4, t4 + T_HOLD),
    }
    print(f"[gelsight-grip] rise during hold {rise_hold * 1e3:.1f} mm "
          f"of {LIFT * 1e3:.0f} commanded; hold forces "
          f"L {fL[hold].mean():.2f} N / R {fR[hold].mean():.2f} N; "
          f"peak hold shear {shear_hold * 1e6:.0f} um")
    for name, ok in checks.items():
        print(f"[gelsight-grip] {name}: {'PASS' if ok else 'FAIL'}")
    grip_frames = (tt >= t2) & (tt < t4 + T_HOLD)
    print(
        "[gs-matrix] "
        f"solver={GS_SOLVER} reduce={int(GS_REDUCE)} anchor={int(GS_ANCHOR)} "
        f"moment={int(GS_MOMENT)} iters={GS_ITERS} sub={SUBSTEPS} "
        f"cadence={GS_CADENCE} objkh={GS_OBJKH:.0e} "
        f"impratio={GS_IMPRATIO:g} cone={GS_CONE or 'default'} | "
        f"contacts~{int(np.median(np.array(contact_counts)[grip_frames]))} | "
        f"creep lift={creep['lift']:+.2f} shake={creep['shake']:+.2f} "
        f"hold={creep['hold']:+.2f} mm/s | "
        f"rise={rise_hold * 1e3:.1f}mm | "
        f"checks={sum(checks.values())}/5"
    )

    out = Path(args.out)
    _save_frame(frames_L[int(np.flatnonzero(hold)[-1])], out.with_name(
        out.name + "_left.npz"))
    _save_frame(frames_R[int(np.flatnonzero(hold)[-1])], out.with_name(
        out.name + "_right.npz"))
    video = args.video if args.video is not None else str(out) + "_panel.mp4"
    if video:
        _save_grip_video(frames_L, frames_R, obj_z, config, Path(video), FPS)
        print(f"[gelsight-grip] wrote {video}")
    return 0 if all(checks.values()) else 1


def _save_grip_video(frames_L, frames_R, obj_z, config, output: Path,
                     fps: int) -> None:
    """Two sensor rows (left/right pad) x three mechanics channels."""

    import imageio.v2 as iio
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.parent.mkdir(parents=True, exist_ok=True)
    both = frames_L + frames_R
    p_scale = max(1e-6, max(float(f.pressure_pa.max()) for f in both) * 1e-3)
    n_scale = max(1e-6, max(float(f.normal_displacement_m.max())
                            for f in both) * 1e3)
    s_scale = max(1e-6, max(float(np.linalg.norm(f.shear_traction_pa,
                                                 axis=0).max())
                            for f in both) * 1e-3)
    extent = (-0.5 * config.width_m * 1e3, 0.5 * config.width_m * 1e3,
              -0.5 * config.height_m * 1e3, 0.5 * config.height_m * 1e3)
    channels = (("pressure", "kPa", "inferno", p_scale),
                ("normal deformation", "mm", "viridis", n_scale),
                ("shear traction", "kPa", "magma", s_scale))

    fig, axes = plt.subplots(2, 3, figsize=(12, 8.6), dpi=110,
                             constrained_layout=True)
    fig.patch.set_facecolor("#101417")
    images = []
    for r, pad in enumerate(("left pad", "right pad")):
        for c, (title, unit, cmap, scale) in enumerate(channels):
            ax = axes[r][c]
            ax.set_facecolor("#101417")
            im = ax.imshow(np.zeros((config.rows, config.cols)),
                           origin="lower", extent=extent, cmap=cmap,
                           vmin=0.0, vmax=scale, interpolation="bilinear")
            images.append(im)
            ax.set_title(f"{pad}: {title}", color="#e6edf3", fontsize=11)
            ax.tick_params(colors="#a8b3bd", labelsize=7)
            for spine in ax.spines.values():
                spine.set_color("#3a454d")
            cb = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
            cb.set_label(unit, color="#a8b3bd", fontsize=8)
            cb.ax.tick_params(colors="#a8b3bd", labelsize=7)
    status = fig.suptitle("", color="#f0f4f7", fontsize=13, y=0.995)

    writer = iio.get_writer(output, fps=fps, quality=8, macro_block_size=None)
    try:
        for i, (fl, fr) in enumerate(zip(frames_L, frames_R, strict=True)):
            for j, f in enumerate((fl, fr)):
                images[3 * j + 0].set_data(f.pressure_pa * 1e-3)
                images[3 * j + 1].set_data(f.normal_displacement_m * 1e3)
                images[3 * j + 2].set_data(
                    np.linalg.norm(f.shear_traction_pa, axis=0) * 1e-3)
            status.set_text(
                "GelSight on gripper  |  "
                f"t {i / fps:5.2f} s  |  object z {obj_z[i] * 1e3:5.1f} mm  |"
                f"  pad force L {fl.total_normal_force_n:4.2f} / "
                f"R {fr.total_normal_force_n:4.2f} N")
            fig.canvas.draw()
            writer.append_data(np.asarray(fig.canvas.buffer_rgba())[:, :, :3])
    finally:
        writer.close()
        plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
