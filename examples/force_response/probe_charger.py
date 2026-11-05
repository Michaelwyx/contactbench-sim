"""Force response of the simulated charger outlet.

It mirrors a real bench test: a UR arm drives the charging block straight down into the printed
outlet, 40 mm at 10 mm/s, logging the force (variants: the outlet's printed
contact spring with a 0.4 mm or a 0.85 mm indent, i.e. how far the spring
stands into the blade slot).

Sim: the SAME outlet model plug_insert_arm uses — the outlet shell mesh and
its two retention pads (the spring_8mm replacement: a wedge on a sprung
slider in each blade slot) — with the plug (charging block 3) on a stiff
prismatic drive straight down at 10 mm/s from 4 mm above the face to just
short of its seat.  The force is the plug's net contact force along the push.
No robot, no gravity.

Knobs:
    --prot  pad protrusion into the slot [mm]   (the real spring's "indent")
    --ke    pad spring rate [N/m]               (spring stiffness)
    --mu    pad/plug friction coefficient
    --flare pad entry-bevel angle [deg]          (how smoothly the spring is
                                                  wedged open on entry)
    --kd    pad damping [N.s/m]
"""

from __future__ import annotations

import argparse
import os
import tempfile

os.environ.setdefault("TB_OUT_ROOT", tempfile.mkdtemp(prefix="probe_charger_"))

import numpy as np
import warp as wp
import newton

from force_bench.rig import Rig, make_solver
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass
import force_bench.tasks.plug_insert_arm as P

SPEED = 0.010


def pad_smooth_tm(hx, side, prot, h_face, h_bevel, back=0.002, n_arc=24):
    """Retention pad with a CURVED entry: the bevel is the convex arc
    y = prot * (1 - cos(pi/2 * u)), tangent to the grip face, so the blade is
    wedged open without the corner the plug_insert_arm wedge has between its
    flat bevel and its face (the corner is what spikes the entry force).
    Same local frame as plug_insert_arm.pad_wedge_tm."""
    import trimesh
    s = float(side)
    pts = [(0.0, 0.0), (0.0, h_face)]
    for u in np.linspace(0.0, 1.0, n_arc)[1:]:
        pts.append((s * prot * (1.0 - np.cos(0.5 * np.pi * u)), h_face + u * h_bevel))
    pts += [(s * (prot + back), h_face + h_bevel), (s * (prot + back), 0.0)]
    v = np.array([[x, y, z] for x in (-hx, hx) for (y, z) in pts])
    tm = trimesh.convex.convex_hull(v)
    return P.subdiv(tm, 0.0015)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prot", type=float, default=P.PAD_PROTRUSION * 1000)
    ap.add_argument("--ke", type=float, default=P.PAD_KE)
    ap.add_argument("--mu", type=float, default=P.MU)
    ap.add_argument("--flare", type=float, default=P.FLARE_DEG)
    ap.add_argument("--kd", type=float, default=P.PAD_KD)
    ap.add_argument("--drive-ke", type=float, default=6.0e4)
    ap.add_argument("--frame-steps", dest="substep_ramp", action="store_false",
                    help="A/B only: move the push target once per 60 Hz frame instead "
                         "of every physics substep (a 0.17 mm step = a ~10 N drive kick "
                         "every frame -> a frame-locked sawtooth)")
    ap.add_argument("--profile", choices=["wedge", "smooth"], default="wedge")
    ap.add_argument("--bevel-mm", type=float, default=None,
                    help="smooth profile: entry length [mm] (default prot/tan(flare))")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    prot = a.prot / 1000.0

    rig = Rig("probe_charger", gravity=0.0)
    shell_tm = load_tm("charging_block_outlet", "shell_1_body1")
    plug_tm = load_tm("charging_block_outlet", "charging_block_3_1_body1")
    slots = P.find_slots(shell_tm)
    pr = P.find_prongs(plug_tm)
    plug_posed = P.pose_plug(plug_tm, shell_tm, slots, pr)
    P.center_in_slots(plug_posed, slots)
    seat = P.measure_seat(plug_posed, shell_tm)
    face_z = float(shell_tm.bounds[1][2])

    shell_sub = P.subdiv(shell_tm, 0.003)
    shell_mesh, shell_c = make_newton_mesh(shell_sub, sdf=True, sdf_resolution=P.SHELL_RES,
                                           margin=P.SDF_MARGIN)
    cfg_shell = newton.ModelBuilder.ShapeConfig(mu=a.mu, density=1240.0, **P.CFG)
    shell_shape = rig.builder.add_shape_mesh(
        -1, mesh=shell_mesh, xform=wp.transform(wp.vec3(*shell_c), wp.quat_identity()),
        cfg=cfg_shell, label="shell")

    slot_halfwidth = 0.00125
    pad_hx = 0.0055
    flare = np.deg2rad(a.flare)
    z_top = face_z - P.PAD_CREST_DROP
    h_bevel = (a.bevel_mm / 1000.0 if a.bevel_mm else prot / np.tan(flare))
    z_mid = z_top - h_bevel
    h_face = z_mid - P.PAD_PLATE_BOT
    for slot, side, name in ((slots[0], -1, "pad_wide"), (slots[1], +1, "pad_narrow")):
        sy = slot["center"][1]
        y_grip = sy + side * (slot_halfwidth - prot)
        xf = wp.transform(wp.vec3(slot["center"][0], y_grip, P.PAD_PLATE_BOT), wp.quat_identity())
        body = rig.builder.add_link(xform=xf, label=name)
        cfg_pad = newton.ModelBuilder.ShapeConfig(mu=a.mu, density=4000.0, **P.CFG)
        tm_pad = (pad_smooth_tm if a.profile == "smooth" else P.pad_wedge_tm)(
            pad_hx, side, prot, float(h_face), float(h_bevel))
        mesh_pad, c_pad = make_newton_mesh(tm_pad, sdf=True, sdf_resolution=128,
                                           margin=P.SDF_MARGIN)
        s_pad = rig.builder.add_shape_mesh(
            body, xform=wp.transform(wp.vec3(*c_pad), wp.quat_identity()),
            mesh=mesh_pad, cfg=cfg_pad, label=name + "_wedge")
        rig.builder.add_shape_collision_filter_pair(s_pad, shell_shape)
        j = rig.builder.add_joint_prismatic(
            parent=-1, child=body, axis=(0.0, float(side), 0.0),
            parent_xform=xf, child_xform=wp.transform_identity(),
            target_pos=0.0, target_ke=a.ke, target_kd=a.kd,
            limit_lower=0.0, limit_upper=P.PAD_TRAVEL)
        rig.builder.add_articulation([j])
        set_body_mass(rig.builder, body, P.PAD_MASS)

    plug_sub = P.subdiv(plug_posed, 0.003)
    plug_mesh, plug_c = make_newton_mesh(plug_sub, sdf=True, sdf_resolution=P.PLUG_RES,
                                         margin=P.SDF_MARGIN)
    plug_c = np.asarray(plug_c)
    plug = rig.builder.add_link(xform=wp.transform(wp.vec3(*plug_c), wp.quat_identity()),
                                label="plug")
    cfg_plug = newton.ModelBuilder.ShapeConfig(mu=a.mu, density=1240.0, **P.CFG)
    rig.builder.add_shape_mesh(plug, mesh=plug_mesh, cfg=cfg_plug, label="plug_shape")
    set_body_mass(rig.builder, plug, P.PLUG_MASS)
    stroke = seat["dz_seat"] - 0.0006            # stop just short of the seat
    jp = rig.builder.add_joint_prismatic(
        parent=-1, child=plug, axis=(0.0, 0.0, -1.0),
        parent_xform=wp.transform(wp.vec3(*plug_c), wp.quat_identity()),
        child_xform=wp.transform_identity(),
        target_pos=0.0, target_ke=a.drive_ke, target_kd=2.0 * np.sqrt(a.drive_ke * 0.06) * 1.5,
        limit_lower=-0.005, limit_upper=stroke + 0.002)
    rig._joint_of["plug"] = jp
    rig.builder.add_articulation([jp])

    rig.collide_every_substep = True
    rig.finalize(probes=["plug"], cam_pos=np.zeros(3), cam_target=np.ones(3), record=False)
    rig.solver = make_solver(rig.model, iterations=24)
    rows = []
    if a.substep_ramp:
        tgt_idx = int(rig.model.joint_target_q_start.numpy()[jp])
        dt = 1.0 / rig.fps / rig.substeps

        class RampSolver:
            def __init__(self, inner):
                self._inner, self.t = inner, 0.0

            def step(self, s_in, s_out, control, contacts, dt_):
                self.t += dt_
                qt = control.joint_target_q.numpy()
                qt[tgt_idx] = min(stroke, SPEED * max(0.0, self.t - 0.2))
                control.joint_target_q.assign(qt)
                return self._inner.step(s_in, s_out, control, contacts, dt_)

            def __getattr__(self, n):
                return getattr(self._inner, n)

        rig.solver = RampSolver(rig.solver)

    def control(t):
        if not a.substep_ramp:
            rig.target_q("plug", min(stroke, SPEED * max(0.0, t - 0.2)))

    def extra(state, t):
        h = rig.probes["plug"].history[-1]
        rows.append((t, min(stroke, SPEED * max(0.0, t - 0.2)) * 1000, h["vec"][2]))
        return {}

    rig.run(0.2 + stroke / SPEED + 0.3, control, extra, print_every=10 ** 9)
    np.savetxt(a.out, np.array(rows), delimiter=",", header="t,push_mm,force_N",
               comments="")
    print("wrote", a.out, len(rows), "stroke_mm", round(stroke * 1000, 2),
          "approach_mm", P.APPROACH * 1000)


if __name__ == "__main__":
    wp.init()
    main()
