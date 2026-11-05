"""Task: FR3 GelSight grip probe -- grasp, lift, shake, hold, release a cube.

The first task with hydroelastic GelSight pads on the REAL FR3 fingers
(gelsight_rig.mount_gelsight_on_fr3: the URDF rubber-tip boxes are disabled
and one gel volume per fingertip replaces them).  A 16 mm, 30 g cube starts
on the table; the expert descends with open fingers straddling it, closes
to a commanded gel indent, lifts 80 mm, shakes +-5 mm along world x (in the
pad plane, i.e. pure shear for the gels), holds, then opens and retreats.

Purpose: validate the sensor on solved FR3 dynamics -- every tactile frame
is a consequence of the arm's own tracking, drive compliance, and the
MuJoCo-solved gel contacts, not of a scripted pose.  QA gates (exit code):

    * both pads loaded through the hold (>= 0.5 N each, sensor integral)
    * the cube rides the gripper: >= 80% of the lift held, total
      grip-frame excursion closure->hold under 6 mm (liftoff weight
      take-up ~1.5 mm + motion creep), and the STATIC HOLD creep rate
      under 0.5 mm/s
    * shear channel active during lift/shake, Coulomb bound respected
    * ForceProbe (solver contacts) vs gel sensor (surface integral) agree
      within a factor of 2 during the hold -- the honesty cross-check
      (measured 0.9-1.0 with the calibrated gel_kh = 4e7 Pa/m)
    * released cleanly (pads unload, cube leaves the gripper)

Solver: TB_MJ_IMPRATIO=10 is set as a default HERE
(before task_rig import) per the measured winner in gelsight_grip_rig.py;
override via the environment for A/B runs.

Run:  PYTHONNOUSERSITE=1 python -m force_bench.tactile.grip_task
Outputs: outputs/gelsight_grip/ (rollout mp4s, forces csv/png, states npz,
tactile_{left,right}.npz frame stacks, rollout_qa.json)
"""

from __future__ import annotations

import os

os.environ.setdefault("TB_MJ_IMPRATIO", "10")
# MuJoCo has no XPBD-style friction anchors, so once-per-frame collide only
# hands it 32 substeps of stale contact geometry (task_rig make_solver
# notes).  Measured here: frame cadence loses the cube during the lift;
# per-substep holds it (slip -2.5 mm over the whole lift+shake+hold).
os.environ.setdefault("TB_COLLIDE_SUBSTEP", "1")

import numpy as np
import warp as wp

import newton

from force_bench.robot.franka import down_grasp_quat
from force_bench.tactile.rig import GelSightRig
from hydrogel.sensor import hydroelastic_object_cfg
from hydrogel.sensor import GelSightConfig

# ── scene constants ────────────────────────────────────────────────────
CUBE_HALF = 0.008          # 16 mm object width (cube edge / sphere diameter)
CUBE_MASS = 0.030          # kg -> density below
CUBE_MU = 0.8
# GS_OBJ selects the grasped object (see build_object):
#   box     16 mm sharp cube -- the QA workhorse; flat-on-flat renders as
#           edge lines + uniform interior, which is what a real sensor shows
#   sphere  16 mm ball -- the classic dome imprint
#   rcube   16 mm cube with 1 mm edge radii -- what real cubes look like
#   nut     the suite's hex nut (15.5 mm across flats, 7 mm thick) on a post
#   screw   the suite's screw standing head-down; the 10 mm threaded shank
#           (1.3 mm deep threads) is pinched -- the textured imprint
GS_OBJ = os.environ.get("GS_OBJ", "box")
assert GS_OBJ in ("box", "sphere", "rcube", "nut", "screw"), GS_OBJ
CUBE_XY = (0.0, 0.0)
ROBOT_BASE = (-0.45, 0.0, 0.0)
STEEL = 7800.0

TCP_GRASP_Z = 0.011        # default: gel centers ~10.5 mm up a 16 mm object;
                           # bare tips 1.3 mm above the table (they reach
                           # 9.7 mm below TCP).  Objects override (see
                           # build_object).
APPROACH_DZ = 0.080        # descend start height above the grasp TCP
LIFT = 0.080               # m
SHAKE_AMP = 0.005          # m, world x (in the pad plane -> pure shear)
SHAKE_HZ = 2.0
# GRIP IS FORCE-SERVOED, NOT POSITION-COMMANDED.  Fixed-indent commands
# sawtooth here: the position drive keeps conquering the contact compliance
# (pad force ramps 2.6 -> 5.8 N over ~0.3 s with nothing moving), then the
# over-squeezed cube pops a fraction of a millimetre out of the pinch
# (README pitfall 6.1, "gentle holds, hard ejects") and the cycle repeats --
# the pop during the shake costs 3-4 mm of slip and the outcome is
# run-to-run bimodal.  The admittance pattern from the utensil tasks
# instead: close at a fixed rate until the MEASURED pad force enters a
# deadband, back off if it overshoots, and keep the servo live through the
# hold so a pop is answered by re-closing.
# Deadband measured across ~10 runs: runs that settled near 2.5 N/pad slip
# -4..-6 mm over the lift+shake (regularization creep scales down with
# load margin) while runs that overshot to ~8 N/pad slip under -1.5 mm.
GRIP_F_LO = float(os.environ.get("GS_FORCE_LO", "7.0"))   # N/pad (solver
GRIP_F_HI = float(os.environ.get("GS_FORCE_HI", "10.0"))  # force) deadband
# extreme-force runs also need GS_FINGER_KE / GS_FINGER_EFFORT (gelsight_rig)
CLOSE_RATE = 0.015         # m/s finger-dof rate while seeking contact
SERVO_RATE = 0.008         # m/s while below the band (7 N needs ~5.8 mm of
                           # overdrive at ke=1200 -- the band must be
                           # reached BEFORE liftoff or the realized force
                           # varies with how far the ramp happened to get)
BACKOFF_RATE = 0.003       # m/s while above the band
MAX_INDENT = 0.010         # m: hard servo floor (10 N needs ~8.3 mm
                           # of drive overdrive at ke=1200)
OPEN_DOF = 0.018           # open-finger target while approaching (pad-face
                           # gap 28.8 mm around the 16 mm cube)

T_SETTLE, T_DESCEND, T_CLOSE, T_LIFT, T_SHAKE, T_HOLD, T_OPEN, T_RETREAT = (
    0.4, 1.0, 1.2, 1.4, 1.0, 0.5, 0.4, 0.5)   # 1.2 s close: the force servo
                                              # must reach its band before
                                              # liftoff; 1.4 s lift: most of
                                              # the slip budget is the lift
                                              # transient


def smooth(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return 3.0 * u * u - 2.0 * u ** 3


def _rounded_cube(half: float, radius: float):
    """Minkowski sum of a box and a sphere, as the convex hull of support
    points sampled over an icosphere of directions."""

    import trimesh

    dirs = trimesh.creation.icosphere(subdivisions=4, radius=1.0).vertices
    inner = half - radius
    pts = np.clip(dirs * 10.0, -inner, inner) + radius * dirs
    return trimesh.PointCloud(pts).convex_hull


def build_object(rig, gel_cfg):
    """Add the GS_OBJ object (+ any pedestal) and return its grasp geometry.

    Returns dict(body, width, tcp_z, spec) where ``spec`` is the renderer's
    object description (type/params/body) and ``width`` the extent along the
    finger-closing axis (world y at yaw 0).
    """

    import json

    b = rig.builder
    if GS_OBJ in ("box", "sphere", "rcube"):
        body = b.add_body(
            xform=wp.transform(wp.vec3(CUBE_XY[0], CUBE_XY[1], CUBE_HALF),
                               wp.quat_identity()), label="cube")
        if GS_OBJ == "box":
            dens = CUBE_MASS / (2.0 * CUBE_HALF) ** 3
            b.add_shape_box(body=body, hx=CUBE_HALF, hy=CUBE_HALF, hz=CUBE_HALF,
                            cfg=hydroelastic_object_cfg(gel_cfg, density=dens,
                                                        mu=CUBE_MU),
                            label="cube_box")
            spec = {"type": "box", "params": [CUBE_HALF] * 3, "body": "cube"}
        elif GS_OBJ == "sphere":
            dens = CUBE_MASS / (4.0 / 3.0 * np.pi * CUBE_HALF ** 3)
            b.add_shape_sphere(body=body, radius=CUBE_HALF,
                               cfg=hydroelastic_object_cfg(gel_cfg, density=dens,
                                                           mu=CUBE_MU),
                               label="cube_box")
            spec = {"type": "sphere", "params": [CUBE_HALF], "body": "cube"}
        else:
            from force_bench.assets import make_newton_mesh

            tm = _rounded_cube(CUBE_HALF, 0.001)
            dens = float(CUBE_MASS / tm.volume)
            mesh, _c = make_newton_mesh(tm, sdf=True, sdf_resolution=128,
                                        narrow_band=0.002, margin=0.0005)
            b.add_shape_mesh(body, mesh=mesh,
                             cfg=hydroelastic_object_cfg(gel_cfg, density=dens,
                                                         mu=CUBE_MU, mesh=True),
                             label="cube_box")
            path = rig.out_dir / "object_mesh.obj"
            rig.out_dir.mkdir(parents=True, exist_ok=True)
            tm.export(path)
            spec = {"type": "mesh", "params": [str(path)], "body": "cube"}
        return dict(body=body, width=2.0 * CUBE_HALF, tcp_z=TCP_GRASP_Z, spec=spec)

    # ── suite fastener meshes (centimetre STLs -> metres, AABB-centred) ──
    from force_bench.assets import load_tm, make_newton_mesh

    stem = {"nut": "nut_075mm_1_body1", "screw": "screw_075mm_1_body1"}[GS_OBJ]
    tm = load_tm("fasteners", stem)
    mesh, center = make_newton_mesh(tm, sdf=True, sdf_resolution=160,
                                    narrow_band=0.002, margin=0.0005)
    tm_c = tm.copy()
    tm_c.vertices = tm_c.vertices - np.asarray(center)
    zmin = float(tm_c.bounds[0][2])
    ext = tm_c.bounds[1] - tm_c.bounds[0]
    if GS_OBJ == "nut":
        # a 13 mm post centres the 7 mm nut on the pad (the pads close to
        # 15.5 mm, so the post must stay narrower than the grasp).  8 mm
        # tall, not 7: at 7 the bare fingertips sat 0.8 mm above the table,
        # inside the contact margin, and a phantom ~46 N per finger made the
        # force servo back the fingers OPEN through the whole close phase.
        ped_top = 0.008
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(0.0, 0.0, 0.5 * ped_top),
                                               wp.quat_identity()),
                        hx=0.0065, hy=0.0065, hz=0.5 * ped_top,
                        cfg=rig.cfg(), label="post")
        z0 = float(ped_top - zmin)
        width = float(ext[1])                 # across flats along world y
        tcp_z = ped_top + 0.5 * float(ext[2])
    else:
        z0 = float(-zmin)                     # head-down on the table
        width = 0.01004                       # thread crest diameter
        tcp_z = 0.018                         # pad bottom clears the head
    body = b.add_body(xform=wp.transform(wp.vec3(CUBE_XY[0], CUBE_XY[1], z0),
                                         wp.quat_identity()), label="cube")
    b.add_shape_mesh(body, mesh=mesh,
                     cfg=hydroelastic_object_cfg(gel_cfg, density=STEEL,
                                                 mu=CUBE_MU, mesh=True),
                     label="cube_box")
    path = rig.out_dir / "object_mesh.obj"
    rig.out_dir.mkdir(parents=True, exist_ok=True)
    tm_c.export(path)
    spec = {"type": "mesh", "params": [str(path)], "body": "cube"}
    print(f"[gelsight-grip-task] object {GS_OBJ}: extents "
          f"{np.round(ext * 1e3, 2)} mm, mass {tm.volume * STEEL * 1e3:.1f} g, "
          f"grasp width {width * 1e3:.2f} mm, tcp_z {tcp_z * 1e3:.1f} mm")
    return dict(body=body, width=width, tcp_z=tcp_z, spec=spec)


def main() -> int:
    # 96x128 raster: same physical channels as the default 32x42 (QA gates
    # are integrals/maxima, resolution-independent), but the tactile RGB
    # path (gelsight_render) upsamples this exactly 2.5x to the GS Mini's
    # 320x240 image
    # gel_kh 4e7 Pa/m: calibrated DOWN from the 2e8 placeholder to the
    # solver-consistent stiffness (7 N solver force at the ~0.8 mm realized
    # indentation over the patch implies ~4e7), which is also the physical
    # ballpark of a real GS Mini gel (E/t ~ 0.2 MPa / 4 mm = 5e7).  This
    # aligns the sensor's pressure integral with the solver's contact
    # forces instead of leaving a 5x systematic gap.
    gel_cfg = GelSightConfig(
        rows=96, cols=128, gel_kh_pa_per_m=4.0e7,
        # GS_GEL_STIFFEN>0 opts into the stiffening law (gelsight_pressure)
        gel_stiffening=float(os.environ.get("GS_GEL_STIFFEN", "0")))
    task_name = "gelsight_grip" if GS_OBJ == "box" else f"gelsight_grip_{GS_OBJ}"
    if os.environ.get("GS_FORCE_LO"):
        task_name += f"_f{int(float(os.environ['GS_FORCE_LO']))}"
    rig = GelSightRig(task_name, gel_config=gel_cfg)
    rig.add_robot(wp.transform(wp.vec3(*ROBOT_BASE), wp.quat_identity()))
    rig.table()

    obj = build_object(rig, gel_cfg)
    cube = obj["body"]
    obj_width = obj["width"]
    tcp_z = obj["tcp_z"]

    rig.mount_gelsight_on_fr3()

    # grasp geometry: yaw=0 closes the fingers along world +-y (measured
    # down_grasp_quat convention), across the cube's +-y faces
    grasp_q = down_grasp_quat(0.0)
    tcp_grasp = np.array([CUBE_XY[0], CUBE_XY[1], tcp_z])
    tcp_app = tcp_grasp + np.array([0.0, 0.0, APPROACH_DZ])
    rig.solve_arm_start(tcp_app, grasp_q)
    rig.robot_grip(cube, filter_fingers=False)   # arm-vs-cube filters only;
    # stabilizer stays OFF (crutch rule) and under MuJoCo no joint is built

    touch_dof = rig.grip_dof(obj_width, 0.0)
    floor_dof = rig.grip_dof(obj_width, MAX_INDENT)
    print(f"[gelsight-grip-task] touch_dof={touch_dof * 1e3:.3f} mm, "
          f"servo floor {floor_dof * 1e3:.3f} mm, force deadband "
          f"[{GRIP_F_LO}, {GRIP_F_HI}] N/pad")

    rig.finalize(
        probes=["cube", "fr3/fr3_leftfinger", "fr3/fr3_rightfinger"],
        cam_pos=(0.45, -0.45, 0.30), cam_target=(0.0, 0.0, 0.08))

    # ── phase script ───────────────────────────────────────────────────
    t1 = T_SETTLE
    t2 = t1 + T_DESCEND
    t3 = t2 + T_CLOSE
    t4 = t3 + T_LIFT
    t5 = t4 + T_SHAKE
    t6 = t5 + T_HOLD
    t7 = t6 + T_OPEN
    t_end = t7 + T_RETREAT

    def tcp_target(t: float):
        pos = tcp_app.copy()
        if t >= t1:
            pos[2] = tcp_app[2] - APPROACH_DZ * smooth((t - t1) / T_DESCEND)
        if t >= t3:
            pos[2] = tcp_grasp[2] + LIFT * smooth((t - t3) / T_LIFT)
        if t4 <= t < t5:
            pos[0] += SHAKE_AMP * np.sin(
                2.0 * np.pi * SHAKE_HZ * (t - t4))
        if t >= t7:
            pos[2] = tcp_grasp[2] + LIFT + APPROACH_DZ * smooth(
                (t - t7) / T_RETREAT)
        return pos

    frame_dt = 1.0 / rig.fps
    grip = {"dof": OPEN_DOF, "release_from": None}

    def grip_servo(t: float) -> float:
        if t < t2:
            return OPEN_DOF
        if t >= t6:                      # open: ramp from wherever we are
            if grip["release_from"] is None:
                grip["release_from"] = grip["dof"]
            grip["dof"] = grip["release_from"] + (
                OPEN_DOF - grip["release_from"]) * smooth((t - t6) / T_OPEN)
            return grip["dof"]
        # servo on the SOLVER's contact force (ForceProbe), not the gel
        # sensor: the sensor's kh*penetration integral tracks true force
        # only up to the dt-bound solver compliance (ratio 0.5-4.5 across
        # runs), so regulating on it lets the delivered force wander
        f_meas = min(
            rig.last_forces.get("fr3/fr3_leftfinger_N", 0.0),
            rig.last_forces.get("fr3/fr3_rightfinger_N", 0.0),
        )
        if f_meas < GRIP_F_LO:
            rate = CLOSE_RATE if f_meas <= 0.05 else SERVO_RATE
            grip["dof"] = max(grip["dof"] - rate * frame_dt, floor_dof)
        elif f_meas > GRIP_F_HI:
            grip["dof"] = min(grip["dof"] + BACKOFF_RATE * frame_dt, OPEN_DOF)
        return grip["dof"]

    def control(t: float):
        rig.drive_arm(tcp_target(t), grasp_q, fingers=grip_servo(t))

    rec = rig.run(t_end, control)
    paths = rec.save("rollout", force_keys=[
        "gel_left_N", "gel_right_N",
        "fr3/fr3_leftfinger_N", "fr3/fr3_rightfinger_N", "cube_N"])
    print("saved:", paths)

    # ── tactile scene spec (for the geometry-derived height map) ──────
    import json

    shape_xf = rig.model.shape_transform.numpy()
    np.savez(
        rig.out_dir / "tactile_scene.npz",
        pads=np.array(["gel_left", "gel_right"]),
        finger_labels=np.array(["fr3/fr3_leftfinger", "fr3/fr3_rightfinger"]),
        gel_xforms=np.stack([shape_xf[rig.gel_shapes["gel_left"]],
                             shape_xf[rig.gel_shapes["gel_right"]]]),
        pad_size=np.array([gel_cfg.width_m, gel_cfg.height_m,
                           gel_cfg.gel_thickness_m, rig.gel_face_z]),
        objects_json=json.dumps([obj["spec"]]),
    )

    # ── tactile frame stacks ───────────────────────────────────────────
    for pad in ("gel_left", "gel_right"):
        frames = rig.tactile_frames[pad]
        np.savez_compressed(
            rig.out_dir / f"tactile_{pad.split('_')[1]}.npz",
            pressure_pa=np.stack([f.pressure_pa for f in frames]),
            normal_displacement_m=np.stack(
                [f.normal_displacement_m for f in frames]),
            shear_displacement_m=np.stack(
                [f.shear_displacement_m for f in frames]),
            shear_traction_pa=np.stack(
                [f.shear_traction_pa for f in frames]),
            contact_mask=np.stack([f.contact_mask for f in frames]),
            total_normal_force_n=np.array(
                [f.total_normal_force_n for f in frames], dtype=np.float32),
            center_of_pressure_m=np.stack(
                [f.center_of_pressure_m for f in frames]),
        )

    # ── QA ─────────────────────────────────────────────────────────────
    qa = rig.qa(paths)
    d = np.load(paths["states"], allow_pickle=True)
    bq, tt = d["body_q"], d["t"]
    fps = rig.fps
    cube_i = rig.body_index("cube")
    n = len(tt)

    fL = np.array([f.total_normal_force_n
                   for f in rig.tactile_frames["gel_left"]])[:n]
    fR = np.array([f.total_normal_force_n
                   for f in rig.tactile_frames["gel_right"]])[:n]
    hold = (tt >= t5) & (tt < t6)
    end = tt >= t_end - 0.5 * T_RETREAT
    grip = (tt >= t3) & (tt < t6)

    cube_z = bq[:, cube_i, 2]
    rise_hold = float(np.median(cube_z[hold]) - CUBE_HALF)
    # grip-frame slip: cube z relative to the ACTUAL finger body, not the
    # commanded TCP -- the commanded frame mixes in arm droop (the FR3 sags
    # differently at lift height than at the table), which is the arm's
    # honesty problem, not the gel's
    finger_i = rig.body_index("fr3/fr3_leftfinger")
    rel = cube_z - bq[:, finger_i, 2]
    i_close = int(t3 * fps) + 1
    slip = float(rel[hold].mean() - rel[i_close]) if hold.any() else np.nan
    # Steady-state creep during the static hold -- the real "does the grasp
    # hold" number.  The TOTAL excursion additionally carries the one-time
    # liftoff weight take-up (~1.5 mm of tangential-compliance sag as the
    # cube leaves the table) and finite regularization creep while the arm
    # MOVES (~0.7 mm/s at impratio=10; the jaw rig measured the same
    # characteristic), so it gets a wider budget.
    hold_rate = (float(np.polyfit(tt[hold], rel[hold], 1)[0])
                 if hold.sum() > 2 else np.nan)

    cfg = rig.gel_config
    coulomb_ok = True
    for pad in ("gel_left", "gel_right"):
        for f in rig.tactile_frames[pad]:
            shear = np.linalg.norm(f.shear_traction_pa, axis=0)
            bound = cfg.friction_coefficient * f.pressure_pa + 1.0
            if (shear > bound * 1.05 + 50.0).any():
                coulomb_ok = False
    shear_grip = max(
        float(np.linalg.norm(f.shear_displacement_m, axis=0).max())
        for f, g in zip(rig.tactile_frames["gel_left"], grip) if g)

    # honesty cross-check: solver-contact force on each finger body vs the
    # sensor's surface integral, during the hold.  These measure DIFFERENT
    # things and are NOT expected to be equal: the probe reads MuJoCo's
    # constraint forces (which match the finger-drive statics, ke *
    # overdrive), while the sensor reads kh * geometric penetration -- and
    # MuJoCo's contact stiffness is TIMESTEP-BOUND (task_rig notes), so the
    # solver penetrates deeper than the pressure law implies for the same
    # force.  Measured at 32 substeps: ratio ~0.33 (sensor overreads ~3x);
    # at 64 substeps it flips to ~3 while the GRASP degrades (slip -2.5 ->
    # -10 mm), so dt cannot reconcile them without losing the task.  The
    # bracket is therefore wide -- this gate exists to catch DECOUPLED
    # readings (sensor seeing air, probe dead), and true sensor/solver
    # stiffness reconciliation is the M4 calibration work.
    with open(paths["forces_csv"]) as fh:
        header = fh.readline().strip().split(",")
    forces_data = np.loadtxt(paths["forces_csv"], delimiter=",", skiprows=1)
    ratios = []
    probe_hold_ok = True
    for probe_key, gel in (("fr3/fr3_leftfinger_N", fL),
                           ("fr3/fr3_rightfinger_N", fR)):
        probe = forces_data[:, header.index(probe_key)][:n]
        probe_hold_ok &= bool(hold.any() and (probe[hold] > 0.5).all())
        sel = hold & (gel > 0.1)
        if sel.any():
            ratios.append(float((probe[sel] / gel[sel]).mean()))
    xcheck_ok = bool(ratios) and all(0.5 <= r <= 2.0 for r in ratios)

    checks = {
        # continuity is judged on the SOLVER force (the sensor's geometric
        # reading can dip for single frames as the realized penetration
        # shifts); the sensor must still SEE the load on average
        "both_pads_loaded_through_hold": bool(
            probe_hold_ok and hold.any()
            and fL[hold].mean() > 0.5 and fR[hold].mean() > 0.5),
        "cube_rides_the_gripper": bool(
            rise_hold > 0.8 * LIFT and abs(slip) < 0.006
            and abs(hold_rate) < 0.5e-3),
        "shear_active_and_coulomb_bounded": bool(
            shear_grip > 1.0e-5 and coulomb_ok),
        "probe_sensor_crosscheck": xcheck_ok,
        "released_cleanly": bool(
            end.any() and fL[end].max() < 0.05 and fR[end].max() < 0.05
            and cube_z[end][-1] < 0.05),
    }
    peak_indent = max(
        float(f.normal_displacement_m.max())
        for pad in ("gel_left", "gel_right")
        for f, h in zip(rig.tactile_frames[pad], hold) if h)
    peak_p = max(
        float(f.pressure_pa.max())
        for pad in ("gel_left", "gel_right")
        for f, h in zip(rig.tactile_frames[pad], hold) if h)
    print(f"[gelsight-grip-task] hold peak indent {peak_indent * 1e3:.2f} mm, "
          f"peak pressure {peak_p * 1e-3:.0f} kPa")
    print(f"[gelsight-grip-task] rise {rise_hold * 1e3:.1f} mm of "
          f"{LIFT * 1e3:.0f}; slip {slip * 1e3:+.2f} mm "
          f"(hold rate {hold_rate * 1e3:+.2f} mm/s); hold forces "
          f"L {fL[hold].mean():.2f} / R {fR[hold].mean():.2f} N; "
          f"probe/sensor ratios {[f'{r:.2f}' for r in ratios]}; "
          f"peak grip shear {shear_grip * 1e6:.0f} um")
    for name, ok in checks.items():
        print(f"[gelsight-grip-task] {name}: {'PASS' if ok else 'FAIL'}")
        qa.report["checks"][name] = {"pass": bool(ok)}
        qa.report["pass"] &= bool(ok)

    rig.finish(qa)

    # tactile RGB strip + panel (offline post-pass over the saved frame
    # stacks; TB_GELSIGHT_RENDER=0 skips for fast mechanics-only runs)
    if os.environ.get("TB_GELSIGHT_RENDER", "1") != "0":
        from hydrogel.render import render_task_dir
        render_task_dir(rig.out_dir, fps=rig.fps, panel=True)

    return 0 if all(checks.values()) and qa.report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
