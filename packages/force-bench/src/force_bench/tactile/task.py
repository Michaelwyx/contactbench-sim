"""Opt-in GelSight fingertips for the FR3 tasks.

``TB_GELSIGHT=1`` turns a task into its tactile variant: the FR3
fingertips carry hydroelastic GelSight pads (gelsight_rig.GelSightRig), the
grasped object is flagged hydroelastic so the gel feels it, and the task
writes ``tactile_{left,right}.npz`` + ``tactile_scene.npz`` next to its
rollout (rendered to ``tactile_strip.mp4`` / ``rollout_panel_tactile.mp4``
by gelsight_render unless ``TB_GELSIGHT_RENDER=0``).  A scene run on its
own lands in ``outputs/task_gelsight/<scene>/``; a ContactBench task
derived from it (appendix_tasks/derive.py) keeps its own folder,
``outputs/appendix_tasks/<task>/``.

With the variable unset every helper here is the identity: ``make_rig`` is
``Rig``, ``finger_offset`` is 0.0, ``hydroelastic`` / ``mount`` /
``save_tactile`` are no-ops and ``make_pipeline`` builds the plain pipeline
the task always built: byte-identical behaviour unless the knob is
turned, so the integration can sit inside the task files without
changing the bare results.

GelSightRig carries its own solver and collision-cadence defaults:
SolverMuJoCo consuming Newton contacts, impratio 10, per-substep collide.
Importing this helper does not change a bare Rig's configuration.
Explicit TB_MJ_IMPRATIO / TB_COLLIDE_SUBSTEP overrides still take precedence.

Geometry the tasks must honour: the sensing face sits ``finger_offset()``
(= gel thickness - embed = 3.5 mm) in front of the bare URDF fingertip
face, so every finger joint target sized for bare pads gets that added
(``FING + finger_offset()``) -- the commanded pad-face gap is then the one
the task was tuned for.  Force-servoed grips then regulate exactly as
before; the probe on the finger body sums the gel's contacts too.
"""

from __future__ import annotations

import json
import os

GS_ON = os.environ.get("TB_GELSIGHT", "0") == "1"

import numpy as np  # noqa: E402
import warp as wp  # noqa: E402

import newton  # noqa: E402

from force_bench.rig import Rig  # noqa: E402

if GS_ON:
    from force_bench.tactile.rig import GelSightRig
    from hydrogel.sensor import winning_hydroelastic_config
    from hydrogel.sensor import GelSightConfig

    # the calibrated task configuration from task_gelsight_grip.py: 96x128
    # raster (2.5x -> GS Mini 320x240), gel_kh 4e7 Pa/m (solver-consistent,
    # real-gel ballpark)
    # gel_stiffening: the stiffening pressure law (gelsight_pressure.py),
    # GS_GEL_STIFFEN=0 restores Newton's linear law for an A/B
    GEL_CFG = GelSightConfig(
        rows=96, cols=128, gel_kh_pa_per_m=4.0e7,
        gel_stiffening=float(os.environ.get("GS_GEL_STIFFEN", "10")),
        # real GelSight elastomer is tacky: TacSL calibrates gel-object
        # friction at 2.0 (GS_GEL_MU overrides; the old value was 0.8)
        friction_coefficient=float(os.environ.get("GS_GEL_MU", "2.0")))
else:
    GEL_CFG = None

EMBED = 0.0005                      # gel embed into the fingertip (m)
SUFFIX = ""          # the variant is told apart by its folder, outputs/task_gelsight/<task>
PADS = ("gel_left", "gel_right")
FINGERS = ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger")


def finger_offset() -> float:
    """Finger-dof offset that keeps the commanded pad-face gap unchanged."""

    if not GS_ON:
        return 0.0
    from force_bench.tactile.rig import pad_proud_m
    return pad_proud_m()


def task_name(name: str) -> str:
    return name + SUFFIX if GS_ON else name


def make_rig(name: str, **kw):
    """``GelSightRig(name + "_gs")`` under TB_GELSIGHT=1, else ``Rig(name)``."""

    if GS_ON:
        from force_bench.paths import TASK_GELSIGHT_OUTPUT_DIR
        rig = GelSightRig(task_name(name), gel_config=GEL_CFG, **kw)
        rig.out_dir = TASK_GELSIGHT_OUTPUT_DIR / name
        return rig
    return Rig(name, **kw)


def mount(rig) -> dict:
    """Mount the pads (after add_robot, before finalize).  No-op when off."""

    if not GS_ON:
        return {}
    shapes = rig.mount_gelsight_on_fr3(embed=EMBED)
    print(f"[gelsight] pads mounted on the FR3 fingers: {shapes}; "
          f"sensing face {finger_offset() * 1e3:.1f} mm proud of the bare tip")
    return shapes


def hydroelastic(rig, label: str, kh: float | None = None) -> list[int]:
    """Flag every shape under ``rig.shapes[label]`` hydroelastic (rigid side).

    Newton only builds a hydroelastic surface when BOTH shapes opt in, so
    the grasped object must be flagged or the gel sees nothing (and the
    pair falls back to the plain SDF contact the task always had).  Mesh
    shapes must already carry an SDF (every Rig mesh path builds one);
    primitive shapes get the gel's SDF resolution.  Flags are edited on the
    builder, so call this before finalize.  No-op when off.
    """

    if not GS_ON:
        return []
    b = rig.builder
    kh = GEL_CFG.rigid_kh_pa_per_m if kh is None else float(kh)
    done = []
    for s in rig.shapes[label]:
        src = b.shape_source[s]
        if b.shape_type[s] in (newton.GeoType.MESH, newton.GeoType.CONVEX_MESH):
            if getattr(src, "sdf", None) is None:
                raise ValueError(
                    f"{label}: mesh shape {s} has no SDF; build it with "
                    "make_newton_mesh(sdf=True) before flagging hydroelastic")
        elif b.shape_sdf_max_resolution[s] is None \
                and b.shape_sdf_target_voxel_size[s] is None:
            b.shape_sdf_max_resolution[s] = GEL_CFG.sdf_max_resolution
            b.shape_sdf_narrow_band_range[s] = (
                -GEL_CFG.sdf_narrow_band_m, GEL_CFG.sdf_narrow_band_m)
        b.shape_flags[s] |= int(newton.ShapeFlags.HYDROELASTIC)
        b.shape_material_kh[s] = kh
        done.append(s)
    print(f"[gelsight] {label}: shapes {done} hydroelastic, kh={kh:.1e} Pa/m")
    return done


def make_pipeline(rig, **kw):
    """The pipeline a task rebuilds after finalize, gel-aware.

    Tasks that need ``max_triangle_pairs`` (subdivided bins/trays) replace
    ``rig.pipeline`` after finalize; under the gel that rebuild must keep
    the hydroelastic surface output and the explicit broad phase (sap drops
    the surface for single frames mid-grasp, docs/gelsight_method.md).
    Assigns ``rig.pipeline`` / ``rig.contacts`` and returns the pipeline.
    """

    kw.setdefault("rigid_contact_max", 20000)
    if GS_ON:
        kw.pop("reduce_contacts", None)
        kw.setdefault("broad_phase", os.environ.get("GS_BROAD", "explicit"))
        kw.setdefault("sdf_hydroelastic_config", winning_hydroelastic_config(
            rig.model, rig.gel_config, rig.gel_shapes.values()))
        kw.setdefault("verify_buffers", False)
    else:
        kw.setdefault("reduce_contacts", True)
        kw.setdefault("broad_phase", "sap")
    rig.pipeline = newton.CollisionPipeline(rig.model, **kw)
    rig.contacts = rig.pipeline.contacts()
    return rig.pipeline


def object_spec(rig, label: str, tm=None, center=None) -> dict:
    """Renderer object description for a mesh body: exports the body-local
    mesh to ``<out_dir>/<label>_mesh.obj``.  Defaults to ``rig.meshes``."""

    if tm is None:
        tm, center = rig.meshes[label]
    tm_c = tm.copy()
    tm_c.vertices = tm_c.vertices - np.asarray(center, dtype=np.float64)
    rig.out_dir.mkdir(parents=True, exist_ok=True)
    path = rig.out_dir / f"{label}_mesh.obj"
    tm_c.export(path)
    return {"type": "mesh", "params": [str(path)], "body": label}


def save_tactile(rig, objects: list[dict], qa=None, window=None,
                 probe_forces: dict | None = None, min_force_n: float = 0.5,
                 render: bool | None = None) -> dict:
    """Write the tactile stacks + scene, add the gel QA gates, render.

    ``objects``: renderer specs (see ``object_spec``).  ``window`` = (t0, t1)
    in seconds over which the grasp is supposed to be loaded; with ``qa`` the
    following checks are appended to ``qa.report``:

      * ``gel_pads_loaded``        both sensor integrals average above
                                   ``min_force_n`` in the window and are
                                   non-zero on >= 85 % of its frames
      * ``gel_coulomb_bounded``    |shear traction| <= mu * pressure on
                                   every cell of every frame (sensor
                                   self-consistency)
      * ``gel_probe_crosscheck``   (only with ``probe_forces``: {pad: array
                                   per frame of the SOLVER force on that
                                   finger}) mean probe/sensor ratio within
                                   [0.5, 2.0] in the window -- the honesty
                                   gate from task_gelsight_grip

    Returns the summary dict (also printed).  No-op when off.
    """

    if not GS_ON:
        return {}
    out = rig.out_dir
    out.mkdir(parents=True, exist_ok=True)
    shape_xf = rig.model.shape_transform.numpy()
    np.savez(
        out / "tactile_scene.npz",
        pads=np.array(PADS),
        finger_labels=np.array(FINGERS),
        gel_xforms=np.stack([shape_xf[rig.gel_shapes[p]] for p in PADS]),
        pad_size=np.array([GEL_CFG.width_m, GEL_CFG.height_m,
                           GEL_CFG.gel_thickness_m, rig.gel_face_z]),
        objects_json=json.dumps(list(objects)),
    )
    n = min(len(rig.tactile_frames[p]) for p in PADS)
    for pad in PADS:
        frames = rig.tactile_frames[pad][:n]
        np.savez_compressed(
            out / f"tactile_{pad.split('_')[1]}.npz",
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

    # ── summary + gates ────────────────────────────────────────────────
    tt = np.arange(n) / rig.fps + 1.0 / rig.fps
    if window is None:
        window = (0.0, tt[-1])
    win = (tt >= window[0]) & (tt <= window[1])
    cfg = rig.gel_config
    summary = {"window_s": [float(window[0]), float(window[1])]}
    loaded_ok, coulomb_ok, ratios = True, True, {}
    for pad in PADS:
        frames = rig.tactile_frames[pad][:n]
        f = np.array([fr.total_normal_force_n for fr in frames])
        flaw = np.array([fr.pressure_law_force_n for fr in frames])
        fw = f[win] if win.any() else f
        peak_p = max(float(fr.pressure_pa.max()) for fr in frames)
        peak_d = max(float(fr.normal_displacement_m.max()) for fr in frames)
        peak_s = max(float(np.linalg.norm(fr.shear_displacement_m, axis=0).max())
                     for fr in frames)
        for fr in frames:
            shear = np.linalg.norm(fr.shear_traction_pa, axis=0)
            bound = cfg.friction_coefficient * fr.pressure_pa + 1.0
            if (shear > bound * 1.05 + 50.0).any():
                coulomb_ok = False
        s = {
            "mean_N": round(float(fw.mean()), 3),
            "min_N": round(float(fw.min()), 3),
            "max_N": round(float(fw.max()), 3),
            "contact_fraction": round(float((fw > 0.1).mean()), 3),
            "peak_pressure_kPa": round(peak_p * 1e-3, 1),
            "peak_indent_mm": round(peak_d * 1e3, 3),
            "peak_shear_um": round(peak_s * 1e6, 1),
            # law-vs-solver compliance diagnostic: the raw pressure-law
            # integral over the window vs the solver-consistent force
            "law_mean_N": round(float(flaw[win].mean()), 3),
            "solver_over_law": round(float(fw.mean() / max(flaw[win].mean(), 1e-9)), 2),
        }
        if probe_forces is not None and pad in probe_forces:
            pf = np.asarray(probe_forces[pad], dtype=np.float64)[:n]
            sel = win & (f > 0.1) & (pf > 0.1)
            if sel.any():
                r = float((pf[sel] / f[sel]).mean())
                ratios[pad] = r
                s["probe_over_sensor"] = round(r, 3)
            s["probe_mean_N"] = round(float(pf[win].mean()), 3)
        loaded_ok &= bool(fw.mean() > min_force_n and (fw > 0.1).mean() >= 0.85)
        summary[pad] = s
    summary["coulomb_bounded"] = coulomb_ok
    print("[gelsight] tactile summary:", json.dumps(summary))

    if qa is not None:
        qa.report["checks"]["gel_pads_loaded"] = {
            "pass": bool(loaded_ok), "min_mean_force_N": min_force_n,
            **{p: summary[p] for p in PADS},
            "note": "both gel sensor integrals carry the grasp through the "
                    "task window (mean above threshold, >=85% of frames "
                    "in contact)"}
        qa.report["pass"] &= bool(loaded_ok)
        qa.report["checks"]["gel_coulomb_bounded"] = {
            "pass": bool(coulomb_ok), "mu": cfg.friction_coefficient,
            "note": "|shear traction| <= mu * pressure on every cell"}
        qa.report["pass"] &= bool(coulomb_ok)
        if probe_forces is not None:
            xok = bool(ratios) and all(0.5 <= r <= 2.0 for r in ratios.values())
            qa.report["checks"]["gel_probe_crosscheck"] = {
                "pass": xok, "bracket": [0.5, 2.0],
                "ratios": {k: round(v, 3) for k, v in ratios.items()},
                "note": "solver contact force on each finger vs the gel's "
                        "pressure integral (decoupled-reading detector)"}
            qa.report["pass"] &= xok

    if render is None:
        render = os.environ.get("TB_GELSIGHT_RENDER", "1") != "0"
    if render:
        from hydrogel.render import render_task_dir
        render_task_dir(out, fps=rig.fps, panel=True)
    return summary


def gel_force_arrays(rig) -> dict[str, np.ndarray]:
    """Per-frame sensor force per pad (for a task's own plots/QA)."""

    if not GS_ON:
        return {}
    return {p: np.array([f.total_normal_force_n for f in rig.tactile_frames[p]])
            for p in PADS}
