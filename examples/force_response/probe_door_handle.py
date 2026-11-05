"""Force response of the simulated door handle (paper Fig. 7).

It mirrors a real bench test: a UR arm pushes the printed lever with a rod in a straight line,
60 mm at 10 mm/s, while a wrist F/T sensor logs the force (variants: 5 /
7.5 / 10 mm printed return springs).

Sim: the kit's doorknob module with handle 1 (force_bench.kit, "doorknob-module"), lying alone with its spindle
vertical, the return spring on the spindle joint, pushed by a 5 mm sphere on a stiff prismatic drive along a straight
line 85 mm out from the spindle.  The sphere touches the lever's grip hull (what the fingers touch in the door
tasks).  The force is the pusher's contact force along the push.  No robot, no gravity.  The module's 5 / 7.5 /
10 mm springs (force_bench.kit.mechanisms.doorknob_module.SPRINGS) carry the values these fits give.

Spring model (the knobs):
    tau = TAU0 - KE * theta              KE  [N.m/rad]   stiffness (slope)
                                         TAU0 [N.m]      preload (the step at
                                         contact), a constant joint torque
                                         holding the lever on its rest stop
    + joint dry friction FRIC [N.m]      (MuJoCo frictionloss)

    python examples/force_response/probe_door_handle.py --ke 0.48 --tau0 0.15 --out door_k048_t015.csv
"""

from __future__ import annotations

import argparse
import os
import tempfile

os.environ.setdefault("TB_OUT_ROOT", tempfile.mkdtemp(prefix="probe_door_"))

import numpy as np
import warp as wp

from force_bench.assets import set_body_mass
from force_bench.kit import add_asset, load_kit
from force_bench.kit.mechanisms._door_parts import SubstepTargets
from force_bench.rig import Rig, make_solver

PUSH_R = 0.085          # m, push line distance from the spindle (best fit
                        # of the real curves among 60/70/85/100 mm)
PUSH_LEN = 0.062        # m (real: 60 mm)
SPEED = 0.010           # m/s (real: 10 mm/s)
SPHERE_R = 0.005
GAP0 = 0.001            # m, pusher starts this far from the bar


def bar_face(module, choice="Handle 1"):
    """Where the push starts, in the module's frame (spindle along +z through the origin): the lever bar points
    along +y at rest and opens toward +x (a clockwise turn seen from above), so the pusher sits on the bar's -x
    face, PUSH_R out, at the bar's mid-height."""
    grip = next(p for p in load_kit()[module].parts(choice) if p.name.startswith("door_handle"))
    v = grip.mesh().vertices
    bar = v[(v[:, 1] >= 0.010) & (v[:, 2] >= 0.0674)]           # the bar's grip hull (doorknob_module.GRIPS)
    at = bar[np.abs(bar[:, 1] - PUSH_R) < 0.003]
    return float(at[:, 0].min()), float(0.5 * (at[:, 2].min() + at[:, 2].max()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ke", type=float, default=0.30)
    ap.add_argument("--tau0", type=float, default=0.0)
    ap.add_argument("--fric", type=float, default=0.0)
    ap.add_argument("--kd", type=float, default=0.01)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    rig = Rig("probe_door_handle", gravity=0.0)
    h = add_asset(rig, "doorknob-module", "Handle 1", pose=np.eye(4), K=a.ke, tau0=a.tau0, kd=a.kd)
    j = h.joints["knob"]
    rig.builder.joint_friction[rig.builder.joint_qd_start[j]] = a.fric

    # pusher: a +x push at y = PUSH_R turns the lever clockwise seen from above (the unlocking direction)
    x_face, z_mid = bar_face("doorknob-module")
    p0 = np.array([x_face - SPHERE_R - GAP0, PUSH_R, z_mid])
    pusher = rig.builder.add_link(xform=wp.transform(wp.vec3(*p0), wp.quat_identity()), label="pusher")
    sid = rig.builder.add_shape_sphere(pusher, radius=SPHERE_R, cfg=rig.cfg(mu=0.3), label="pusher_tip")
    rig.shapes["pusher"] = [sid]
    for s in h.info["housing_shapes"]:        # the pusher touches the lever's grip hull, nothing else
        rig.filter_pair(sid, s)
    set_body_mass(rig.builder, pusher, 0.05)
    jp = rig.builder.add_joint_prismatic(
        parent=-1, child=pusher, axis=(1.0, 0.0, 0.0),
        parent_xform=wp.transform(wp.vec3(*p0), wp.quat_identity()),
        child_xform=wp.transform_identity(),
        target_pos=0.0, target_ke=4.0e4, target_kd=60.0,
        limit_lower=-0.01, limit_upper=0.10)
    rig.builder.add_articulation([jp])

    rig.hard_joint_limits = True
    rig.collide_every_substep = True
    rig.finalize(probes=["pusher"], cam_pos=np.zeros(3), cam_target=np.ones(3), record=False)
    # the push target moves every physics SUBSTEP: updated once per 60 Hz frame it is a 0.17 mm step, i.e. a
    # several-newton drive kick every frame (a frame-locked sawtooth in the force)
    tgt = int(rig.model.joint_target_q_start.numpy()[jp])
    rig.solver = SubstepTargets(make_solver(rig.model, iterations=24),
                                lambda t: {tgt: min(PUSH_LEN, SPEED * max(0.0, t - 0.2))})
    kq = int(rig.model.joint_q_start.numpy()[j])
    rows = []

    def extra(state, t):
        f = rig.probes["pusher"].history[-1]
        disp = SPEED * max(0.0, t - 0.2)
        rows.append((t, min(disp, PUSH_LEN) * 1000, -f["vec"][0], np.degrees(-state.joint_q.numpy()[kq])))
        return {}

    rig.run(0.2 + PUSH_LEN / SPEED + 0.3, lambda t: None, extra, print_every=10 ** 9)
    np.savetxt(a.out, np.array(rows), delimiter=",", header="t,push_mm,force_N,lever_deg", comments="")
    print("wrote", a.out, len(rows))


if __name__ == "__main__":
    wp.init()
    main()
