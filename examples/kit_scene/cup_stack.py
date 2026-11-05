"""A scene built from the kit library: three kit cups dropped into each other, and how they nest.

    python examples/kit_scene/cup_stack.py          # -> outputs/task/cup_stack/ (rollout video, final still)

The cups are the kit's published cup (68.6 mm rim, 1 mm wall): free bodies whose thin walls collide through
0.2 mm signed-distance fields with small contact margins (assets/kit/sim.yaml), with the mass of a PLA print
(15.2 g).  Each upper cup starts 10 mm above its nested height and a little off the stack's axis; they settle
centred, at a 4.68 mm step.  No robot.
"""

from __future__ import annotations

import numpy as np
import warp as wp

from force_bench.kit import add_asset, place_on_table
from force_bench.rig import Rig

STEP = 0.00468          # measured nesting step of the kit cup [m]
DROP = 0.010            # start height of each upper cup above its nested height [m]


def main() -> dict:
    wp.init()
    rig = Rig("cup_stack")
    rig.table(half=0.3)
    rig.collide_every_substep = True                 # thin walls: re-detect contact every substep
    base = place_on_table("cup", xy=(0.0, 0.0))     # lowest point on the table, centred at xy
    cups = [add_asset(rig, "cup", pose=base, label="cup_a")]
    for i, label in enumerate(("cup_b", "cup_c"), start=1):
        pose = base.copy()
        pose[:3, 3] += [0.0015 * i, -0.0010, i * (STEP + DROP)]
        cups.append(add_asset(rig, "cup", pose=pose, label=label))
    rig.finalize(probes=[], cam_pos=np.array([0.25, -0.30, 0.25]), cam_target=np.array([0.0, 0.0, 0.05]))
    rig.run(2.0, lambda t: None, print_every=10 ** 9)

    q = rig.s0.body_q.numpy()
    xyz = np.array([q[h.bodies[""], :3] for h in cups], dtype=float)   # body origins: bounding-box centres
    steps = np.diff(xyz[:, 2]) * 1e3
    offsets = np.linalg.norm(xyz[1:, :2] - xyz[0, :2], axis=1) * 1e3
    print(f"nesting steps {np.round(steps, 2).tolist()} mm (measured {STEP * 1e3:.2f}), "
          f"off-axis {np.round(offsets, 2).tolist()} mm")
    if rig.rec is not None and len(rig.rec.frames):
        from PIL import Image
        Image.fromarray(rig.rec.frames[len(rig.rec.frames) - 1]).save(rig.out_dir / "final.png")
        rig.rec.save("rollout")
    return {"steps_mm": steps.tolist(), "off_axis_mm": offsets.tolist()}


if __name__ == "__main__":
    main()
