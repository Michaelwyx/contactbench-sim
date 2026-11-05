"""Calibrate the gel's hydroelastic stiffness (gel_kh) against ball presses.

The hydroelastic contact model is a LOCAL (Winkler / elastic-foundation)
law: pressure = k_eff * local penetration, with k_eff the series stiffness
of the two hydroelastic moduli (1/k_eff = 1/gel_kh + 1/object_kh).  For a
sphere of radius R pressed a depth d into a flat gel this integrates in
closed form:

    F(d) = pi * k_eff * R * d^2         (paraboloid cap, d << R)

so one real-sensor ball press -- (force, depth) pairs at known R -- fixes
gel_kh directly:  gel_kh = F / (pi * R * d^2)  (object kh >> gel kh).

Two modes:

  --sweep           press the simulated gel with the sensor's own demo scene
                    across indentation depths and report F(d), the implied
                    k_eff, and the ratio to the analytic law (the
                    discretization factor of the SDF/raster pipeline)
  --fit real.csv    fit gel_kh from a CSV of real presses with columns
                    force_n, depth_m [, radius_m]; reports the least-squares
                    gel_kh and, if a sweep factor is given (--sim-factor),
                    the value to put in GelSightConfig so the SIMULATED
                    sensor reproduces the real curve.

What this does NOT calibrate: the gel's lateral (membrane) coupling.  The
hydroelastic model has none -- pressure depends only on local penetration
-- so the draping skirt in gelsight_render.py (SKIRT_SIGMA_MM) is a
render-time parameter; fit it from the radial intensity falloff around a
real ball-press image (procedure in docs/gelsight_method.md).

Usage (env tactile-newton, run from sim/):
    PYTHONNOUSERSITE=1 python -m hydrogel.calibrate --sweep
    PYTHONNOUSERSITE=1 python -m hydrogel.calibrate --fit presses.csv [--sim-factor 0.93]
"""

from __future__ import annotations

import argparse
import sys

import numpy as np


def analytic_force(k_eff: float, radius: float, depth: np.ndarray) -> np.ndarray:
    return np.pi * k_eff * radius * np.asarray(depth) ** 2


def series(k_a: float, k_b: float) -> float:
    return 1.0 / (1.0 / k_a + 1.0 / k_b)


def sweep(gel_kh: float, object_kh: float, radius: float, depths_mm, rows: int,
          cols: int, sdf_res: int) -> float:
    from hydrogel.sensor import GelSightConfig, build_demo_scene, set_demo_indenter_pose

    cfg = GelSightConfig(rows=rows, cols=cols, gel_kh_pa_per_m=gel_kh,
                         rigid_kh_pa_per_m=object_kh, sdf_max_resolution=sdf_res)
    scene = build_demo_scene(cfg)
    # the demo scene's indenter radius is fixed at 6 mm; rebuild if asked
    if abs(scene.indenter_radius_m - radius) > 1e-9:
        print(f"[calibrate] note: demo indenter radius is {scene.indenter_radius_m*1e3:.1f} mm; "
              f"--radius is used only for the analytic law")
        radius = scene.indenter_radius_m
    k_eff = series(gel_kh, object_kh)
    print(f"[calibrate] gel_kh={gel_kh:.3g} object_kh={object_kh:.3g} -> k_eff={k_eff:.3g} Pa/m, "
          f"R={radius*1e3:.1f} mm, raster {rows}x{cols}, sdf_res {sdf_res}")
    print(f"{'depth_mm':>9} {'F_sim_N':>9} {'F_analytic':>11} {'ratio':>6} {'k_implied':>10} {'faces':>6}")
    ratios = []
    for d_mm in depths_mm:
        d = d_mm * 1e-3
        scene.sensor.reset()
        set_demo_indenter_pose(scene, indentation_m=d)
        scene.pipeline.collide(scene.state, scene.contacts)
        fr = scene.sensor.sample(scene.state, 0.0, tangential_velocity_local_mps=(0.0, 0.0))
        f_an = float(analytic_force(k_eff, radius, d))
        ratio = fr.total_normal_force_n / f_an if f_an > 0 else np.nan
        k_impl = fr.total_normal_force_n / (np.pi * radius * d * d) if d > 0 else np.nan
        ratios.append(ratio)
        print(f"{d_mm:9.3f} {fr.total_normal_force_n:9.4f} {f_an:11.4f} {ratio:6.3f} "
              f"{k_impl:10.3g} {fr.contact_face_count:6d}")
    factor = float(np.nanmedian(ratios))
    print(f"[calibrate] sim/analytic factor (median) = {factor:.3f} -- pass as --sim-factor "
          f"when fitting real data so the simulated sensor reproduces the real curve")
    return factor


def fit(csv_path: str, radius_default: float, object_kh: float, sim_factor: float) -> float:
    data = np.genfromtxt(csv_path, delimiter=",", names=True)
    force = np.asarray(data["force_n"], dtype=float)
    depth = np.asarray(data["depth_m"], dtype=float)
    radius = (np.asarray(data["radius_m"], dtype=float)
              if "radius_m" in data.dtype.names else np.full_like(force, radius_default))
    x = np.pi * radius * depth ** 2
    k_eff = float(np.sum(x * force) / np.sum(x * x))       # least squares through origin
    resid = force - k_eff * x
    rel_rms = float(np.sqrt(np.mean(resid ** 2)) / max(np.mean(force), 1e-9))
    # undo the series with the (stiff) object and the simulation's own factor
    gel_kh = 1.0 / (1.0 / k_eff - 1.0 / object_kh)
    gel_kh_sim = gel_kh / sim_factor
    print(f"[calibrate] {len(force)} presses: k_eff = {k_eff:.4g} Pa/m "
          f"(relative RMS residual {rel_rms*100:.1f}%)")
    print(f"[calibrate] physical gel_kh = {gel_kh:.4g} Pa/m (object_kh={object_kh:.3g})")
    print(f"[calibrate] GelSightConfig(gel_kh_pa_per_m={gel_kh_sim:.4g}) reproduces the "
          f"real curve in simulation (sim factor {sim_factor:.3f})")
    return gel_kh_sim


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--fit", metavar="CSV")
    ap.add_argument("--gel-kh", type=float, default=4.0e7)
    ap.add_argument("--object-kh", type=float, default=1.0e10)
    ap.add_argument("--radius", type=float, default=0.006, help="ball radius [m]")
    ap.add_argument("--depths-mm", default="0.1,0.2,0.4,0.6,0.8,1.0,1.5")
    ap.add_argument("--rows", type=int, default=96)
    ap.add_argument("--cols", type=int, default=128)
    ap.add_argument("--sdf-res", type=int, default=96)
    ap.add_argument("--sim-factor", type=float, default=1.0)
    args = ap.parse_args()
    if not args.sweep and not args.fit:
        ap.error("--sweep and/or --fit required")
    depths = [float(v) for v in args.depths_mm.split(",")]
    if args.sweep:
        sweep(args.gel_kh, args.object_kh, args.radius, depths, args.rows,
              args.cols, args.sdf_res)
    if args.fit:
        fit(args.fit, args.radius, args.object_kh, args.sim_factor)
    return 0


if __name__ == "__main__":
    sys.exit(main())
