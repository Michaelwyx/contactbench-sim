"""Cylinder press probe of the simulated Fin Ray finger.

The finger hangs tip-down from a fixed carriage (the printed adapter and the
base block, as on the FR3); a rigid Ø25 mm cylinder whose axis runs along the
finger width is pushed into the contact face at a chosen height, at 5 mm/s,
on a stiff position-driven slide.  Logged per frame: the slide's travel past
first touch (indentation), the contact force on the cylinder, and from the
linkage's link poses the contact length and arc on the cylinder and the
tip's motion (toward the object < 0).  The same press is solved
quasi-statically for the PRB linkage on the CPU (``prb_cpu``) and for the
2 mm frame reference (``frame_nl``); ``report`` plots all three.

Run:  python -m force_bench.finray.probe [--z -60] [--depth 20] [--model prb]
Output: outputs/finray/probe_<model>_z<z>/ (rollout.mp4, rollout_states.npz, probe.json); the figure:
``python -m force_bench.finray.report``
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np
import warp as wp

import newton

from force_bench.finray import geometry as G
from force_bench.finray.model import (FinRayParams, add_adapter_and_base, add_prb_finger,
                                      add_loops, add_rigid_finger, filter_finger, tune_solver,
                                      _rigid_mass_props, _set_mass)
from force_bench.paths import OUTPUTS_ROOT
from force_bench.rig import Rig

H = 0.25                     # m, carriage height above the table
R_CYL = 0.0125
V_PRESS = 0.005              # m/s
T_SETTLE = 0.4
SLIDE_KE, SLIDE_KD = 2.0e5, 2.0e3
GAP0 = 0.001                 # m of air before first touch


def finger_to_world(p_f) -> np.ndarray:
    """Finger frame -> world for the probe's carriage (R_y(pi) at height H)."""
    p = np.atleast_2d(p_f)
    return np.stack([-p[:, 0], p[:, 1], H - p[:, 2]], axis=1)


def build(params: FinRayParams, z_step: float, out_name: str):
    rig = Rig(out_name)
    rig.out_dir = OUTPUTS_ROOT / "finray" / out_name
    rig.substeps = params.substeps
    b = rig.builder
    q_base = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(np.pi))
    X_base = wp.transform(wp.vec3(0.0, 0.0, H), q_base)
    base = b.add_link(xform=X_base, label="fr3/finray_P_carriage")
    j0 = b.add_joint_fixed(-1, base, parent_xform=X_base, label="probe_mount")
    add_adapter_and_base(b, base, params, "fr3/finray_P", base_block=(params.model == "prb"))
    if params.model == "rigid":
        add_rigid_finger(b, base, params, "fr3/finray_P")
        m, com, I = _rigid_mass_props(params, with_finger=True)
        fin = None
    else:
        m, com, I = _rigid_mass_props(params, with_finger=False)
        fin = add_prb_finger(b, base, params, "fr3/finray_P", "P")
    _set_mass(b, base, m, com, I)
    b.add_articulation(list(range(j0, b.joint_count)))
    if fin is not None:
        add_loops(b, fin, "fr3/finray_P")
        filter_finger(b, fin, [])
    # the cylinder on a slide along the closing axis (world +y pushes in)
    zc = H - (G.Z_BASE - z_step) / 1000.0
    y0 = -(R_CYL + GAP0)
    X_cyl = wp.transform(wp.vec3(0.0, y0, zc), wp.quat_identity())
    cyl = b.add_link(xform=X_cyl, label="probe_cylinder")
    b.add_shape_cylinder(cyl, xform=wp.transform(wp.vec3(0.0, 0.0, 0.0),
                                                 wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(np.pi / 2))),
                         radius=R_CYL, half_height=0.02,
                         cfg=rig.cfg(mu=0.5, density=2000.0), label="probe_cylinder_shape")
    js = b.add_joint_prismatic(-1, cyl, axis=(0.0, 1.0, 0.0), parent_xform=X_cyl,
                               target_pos=0.0, target_ke=SLIDE_KE, target_kd=SLIDE_KD,
                               label="probe_slide")
    b.add_articulation([js])
    rig.collide_every_substep = True
    rig.finalize(probes=["probe_cylinder"],
                 cam_pos=np.array([0.32, -0.05, zc + 0.02]), cam_target=np.array([0.0, 0.012, zc]))
    rig.solver_tuned = tune_solver(rig.solver, params)
    rig.slide_joint = js
    return rig, fin, zc, y0


def contact_metrics(rig, fin, bq, zc, yc):
    """Contact length and arc of the contact wall on the cylinder, and the
    tip position (finger frame, mm)."""
    if fin is None:
        return {}
    c = np.array([yc, zc])                                      # world (y, z)
    segs = []
    for name, l in fin.links.items():
        if l.kind != "contact":
            continue
        T = bq[fin.bodies[name]]
        p0 = T[:3]
        R = _R(T[3:7])
        d = np.array([0.0, 0.0, 1.0])                            # link axis in body frame
        p1_f = np.array([0.0, (l.p1[0] - l.p0[0]) / 1000.0, -(l.p1[1] - l.p0[1]) / 1000.0])
        # the outer face: offset by -t/2 along the body's y (finger y)
        off = R @ np.array([0.0, -l.t / 2000.0, 0.0])
        a = p0 + off
        bb = p0 + R @ p1_f + off
        segs.append((a[1:], bb[1:]))
    touch, ang, length = [], [], 0.0
    for a, b in segs:
        ts = np.linspace(0.0, 1.0, 9)
        pts = a[None] + ts[:, None] * (b - a)[None]
        d = np.linalg.norm(pts - c, axis=1)
        on = d < R_CYL + 0.0003
        if on.any():
            length += float(np.linalg.norm(b - a) * on.mean())
            ang += list(np.degrees(np.arctan2(pts[on, 1] - c[1], pts[on, 0] - c[0])))
    span = float(max(ang) - min(ang)) if len(ang) > 1 else 0.0
    last = max((n for n, l in fin.links.items() if l.kind == "contact"))
    T = bq[fin.bodies[last]]
    l = fin.links[last]
    tip_w = T[:3] + _R(T[3:7]) @ np.array([0.0, (l.p1[0] - l.p0[0]) / 1000.0, -(l.p1[1] - l.p0[1]) / 1000.0])
    tip_f = np.array([tip_w[1], H - tip_w[2]])
    return {"contact_len_mm": round(length * 1000, 2), "wrap_deg": round(span, 1),
            "tip_y_mm": float(tip_f[0] * 1000), "tip_z_mm": float(tip_f[1] * 1000)}


def _R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def run(params: FinRayParams, z_step=-60.0, depth=0.020, out_name=None) -> dict:
    out_name = out_name or f"probe_{params.model}_z{int(-z_step)}"
    rig, fin, zc, y0 = build(params, z_step, out_name)
    ci = rig.body_index("probe_cylinder")
    qs = int(rig.model.joint_q_start.numpy()[rig.slide_joint])
    T_press = (depth + GAP0) / V_PRESS
    dur = T_SETTLE + T_press + 0.5
    log = []
    tip0 = {}

    def control(t):
        s = 0.0 if t < T_SETTLE else min(depth + GAP0, V_PRESS * (t - T_SETTLE))
        qt = rig.control.joint_target_q.numpy()
        qt[qs] = s
        rig.control.joint_target_q.assign(qt)

    def extra(state, t):
        bq = state.body_q.numpy()
        yc = float(bq[ci, 1])
        f = rig.probes["probe_cylinder"].history[-1]
        m = contact_metrics(rig, fin, bq, zc, yc)
        if not tip0 and m:
            tip0.update(m)
        row = {"t": round(t, 4), "indent_mm": round((yc + R_CYL) * 1000, 3),
               "force_N": round(abs(f["vec"][1]), 4), "force_total_N": round(f["force"], 4)}
        if m:
            row.update({"contact_len_mm": m["contact_len_mm"], "wrap_deg": m["wrap_deg"],
                        "tip_dx_mm": round(m["tip_y_mm"] - tip0["tip_y_mm"], 3),
                        "tip_dz_mm": round(-(m["tip_z_mm"] - tip0["tip_z_mm"]), 3)})
        log.append(row)
        return {k: v for k, v in row.items() if k in ("indent_mm", "force_N", "tip_dx_mm")}

    import time
    t0 = time.time()
    rec = rig.run(dur, control, extra, print_every=60)
    wall = time.time() - t0
    rec.save("rollout")
    res = {"params": params.__dict__, "z_step_mm": z_step, "R_mm": R_CYL * 1000,
           "solver_welds_tuned": rig.solver_tuned, "wall_s": round(wall, 1),
           "sim_s": dur, "bodies": int(rig.model.body_count), "log": log}
    (rig.out_dir / "probe.json").write_text(json.dumps(res, indent=1, default=float))
    print(f"probe done: {rig.out_dir / 'probe.json'}  ({wall:.0f} s wall for {dur:.1f} s)")
    return res


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--z", type=float, default=-60.0, help="STEP height of the press (mm)")
    ap.add_argument("--depth", type=float, default=20.0, help="mm past first touch")
    ap.add_argument("--model", default="prb")
    ap.add_argument("--substeps", type=int, default=32)
    ap.add_argument("--omega-dt", type=float, default=0.35)
    ap.add_argument("--zeta", type=float, default=0.5)
    ap.add_argument("--tag", default="")
    a = ap.parse_args(argv)
    p = FinRayParams(model=a.model, substeps=a.substeps, omega_dt=a.omega_dt, zeta=a.zeta)
    run(p, z_step=a.z, depth=a.depth / 1000.0,
        out_name=f"probe_{a.model}_z{int(-a.z)}{a.tag}")


if __name__ == "__main__":
    wp.init()
    main()
