"""Appendix task "Pick up object on platform" (Table I).

    Description: Pick up the thin and/or small object placed on the flat
    platform.  Success: the target object is stably grasped by the robot
    and lifted off the platform.

The appendix photo: a yellow card lying in the MIDDLE of a white board that
stands on four black corner legs; the hand slides it to the edge and
pinches the overhang.  Derived from ``force_bench.tasks.slide_off_edge``
(unchanged on disk), which already uses that strategy (blade-press slide to
the edge, side pinch on the overhang, lift) but on a bench whose card starts
8 mm from the edge.  Changes, all set from outside the module:

- START ON THE PLATFORM, NOT AT ITS EDGE.  ``START_INSET`` 8 -> 40 mm (the
  card lies fully on the board with 4 cm of board in front of it; the photo
  has it dead centre, which this cell cannot reach -- see BASE_Y below) and
  the base 2 cm further back; the slide budget ``T_SLIDE`` 4 -> 7 s for the
  longer stroke (the stop is still the suite's MEASURED overhang).
- THE PLATFORM.  The suite bench is a board on one pedestal set back 100 mm
  from the edge; here it is a board on four corner legs (the photo's black
  brackets), same top height, same top, same free overhang under the edge.
- VERDICT.  Lifted >= 30 mm clear of the platform AND stably grasped: both
  pads loaded through a 1.5 s hold at the top (>= 95 % of frames > 0.5 N)
  with < 2 mm of part motion relative to the fingers.

Variants: ``card`` (default, the photo's object) and ``key``.

Run: python -m force_bench.appendix_tasks.pick_up_object_on_platform [card|key]
"""

from __future__ import annotations

import os
import sys

import numpy as np
import warp as wp

from force_bench.appendix_tasks.derive import Variant, run_variant

LEG = 0.020            # m square legs
INSET_MID = 0.040      # m; the card's front edge this far inboard of the edge
BASE_Y = 0.020         # m; FR3 base 2 cm further from the bench than the suite's
# WHY NOT THE PHOTO'S DEAD CENTRE (85 mm).  The FR3 stands on the floor
# BESIDE the bench and the blade comes down on the card from above; the
# further inboard the card starts, the closer that blade pose is to the
# robot's own base, and the elbow folds onto its limit.  Scanned with the
# IK (joint 4, limit -3.042 rad): inset 8 mm (suite) -> 0.13 rad of margin,
# 40 mm -> 0.04, 55-85 mm -> 0.00 -- and the first appendix run at 85 mm
# measured exactly that: the blade arrived 5 mm high, the press servo spent
# 6 s finding the card, overshot to 287 N and the slide ran out of budget
# 30 mm short.  Moving the base back buys blade margin but costs it at the
# side pinch on the far edge (base +2 cm: blade 0.09 / pinch-standoff 0.10;
# +4 cm: 0.12 / 0.03).  40 mm + 2 cm is the deepest start this cell reaches
# with >= 0.09 rad everywhere; a centred start needs a re-laid-out cell.
LIFT_MIN = 0.030
HOLD_S = 1.5


def build_platform(mod):
    def build_table(rig):
        top, t = mod.TABLE_TOP, mod.TABLE_T
        mod._box(rig, "table_b", mod.EDGE_Y, mod.BACK_Y, top - t, top,
                 mod.TABLE_HX, mod.TABLE_MU)
        for sx in (-1, 1):
            for y0 in (mod.EDGE_Y, mod.BACK_Y - LEG):
                x = sx * (mod.TABLE_HX - 0.5 * LEG)
                sid = rig.builder.add_shape_box(
                    -1, xform=wp.transform(
                        wp.vec3(x, y0 + 0.5 * LEG, 0.5 * (top - t)), wp.quat_identity()),
                    hx=0.5 * LEG, hy=0.5 * LEG, hz=0.5 * (top - t),
                    cfg=rig.cfg(mu=mod.TABLE_MU, gap=0.0003), label="platform_leg")
                rig.shapes.setdefault("platform_leg", []).append(sid)
    return build_table


class PlatformVariant(Variant):
    part: str = "card"

    def configure(self, mod):
        self._orig_build = mod.build_table
        mod.build_table = build_platform(mod)
        mod.START_INSET = INSET_MID
        mod.BASE_Y = BASE_Y
        mod.T_SLIDE = 7.0 if mod.PART == "card" else 14.0

    def cleanup(self, mod):
        mod.build_table = self._orig_build

    def extra_duration(self):
        return HOLD_S

    def ending(self, rig, t, tau):
        pass                    # the expert keeps holding the part up

    def criteria(self, rig, qa):
        mod = self.mod
        d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
        bq, t = d["body_q"], d["t"]
        pi = list(d["body_labels"]).index("part")
        li = list(d["body_labels"]).index("fr3/fr3_leftfinger")
        ri = list(d["body_labels"]).index("fr3/fr3_rightfinger")
        lift = float(bq[-1, pi, 2] - bq[0, pi, 2])
        start_from_edge = float(bq[0, pi, 1] - mod.EDGE_Y)
        hold = t >= t[-1] - HOLD_S
        mid = 0.5 * (bq[:, li, :3] + bq[:, ri, :3])
        rel = bq[:, pi, :3] - mid
        slip = float(np.linalg.norm(rel[hold] - rel[hold][0], axis=1).max())
        fl = np.array([r["force"] for r in rig.probes["padL"].history])
        fr = np.array([r["force"] for r in rig.probes["padR"].history])
        th = np.array([r["t"] for r in rig.probes["padL"].history])
        hm = th >= th[-1] - HOLD_S
        both = float(((fl[hm] > 0.5) & (fr[hm] > 0.5)).mean())
        return {
            "starts_mid_platform": {
                "pass": bool(start_from_edge > 0.05),
                "front_edge_inset_mm": INSET_MID * 1000, "suite_inset_mm": 8.0,
                "part_centre_from_edge_mm": round(start_from_edge * 1000, 1),
                "platform_depth_mm": round((mod.BACK_Y - mod.EDGE_Y) * 1000, 1)},
            "lifted_off_platform": {
                "pass": bool(lift >= LIFT_MIN), "lift_mm": round(lift * 1000, 1),
                "min_mm": LIFT_MIN * 1000},
            "stably_grasped": {
                "pass": bool(both >= 0.95 and slip < 0.002),
                "hold_s": HOLD_S, "both_pads_loaded_fraction": round(both, 3),
                "pad_mean_N": [round(float(fl[hm].mean()), 2), round(float(fr[hm].mean()), 2)],
                "part_slip_in_hand_mm": round(slip * 1000, 3)},
        }


def make_variant(part: str = "card") -> PlatformVariant:
    os.environ["TB_SLIDE_PART"] = part        # read at the module's import
    v = PlatformVariant(
        name="pick_up_object_on_platform" + ("" if part == "card" else f"_{part}"),
        source="force_bench.tasks.slide_off_edge",
        appendix_task="Pick up object on platform",
        spec="The target object is stably grasped by the robot and lifted off the "
             "platform.",
        objects=("part",),
        static_shapes=("table_b",),
        changes=[
            "part starts on the platform, front edge 40 mm from the edge (suite 8 mm; "
            "dead centre is past FR3 joint-4's limit in this cell), base 2 cm back, "
            "slide budget 4 -> 7 s (card) / 9.5 -> 14 s (key)",
            "platform = board on four corner legs (suite: board on a set-back pedestal)",
            "1.5 s hold at the top; verdict adds 'stably grasped' (both pads loaded, "
            "< 2 mm slip in hand)",
        ],
        argv=(part,),
    )
    v.part = part
    return v


VARIANT = make_variant("card")


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    part = argv[0] if argv else "card"
    return run_variant(make_variant(part))


if __name__ == "__main__":
    wp.init()
    main()
