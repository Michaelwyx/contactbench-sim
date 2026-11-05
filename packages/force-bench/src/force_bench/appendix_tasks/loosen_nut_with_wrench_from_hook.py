"""Appendix task "Loosen nut with wrench" (Table I, #18), with the wrench
taken from the hook shelf and hung back (user request, 2026-10-16).

    Description: A nut is threaded onto a screw head in the workspace and is
    fully tightened.  Use the wrench grasped in hand to loosen the screw
    [nut].  Success: The nut is loosened and can rotate freely when a small
    force is applied in an attempt to rotate the nut.  The robot movement
    stops after terminating the loosening motion, moving away from, and
    breaking contact with the nut.

Scene, mechanism model and expert: ``_wrench_hook.py``.  Start state: the
kit wrench hangs on the middle hook; the kit nut sits on its seat (the bolt
block's top face) with a 20 mN.m seat preload (elasto-plastic stiction on
its yaw, what a ~30 mN.m tightening leaves at the tighten task's 0.7 ratio).

EXPERT: the tighten task's (take the wrench off the hook, carry it flat to
the nut, square the jaw to the MEASURED flats, slide on), but the strokes
swing 60 deg COUNTER-clockwise: the first one has to break the preload
through the jaw (the reaction is logged), the preload model releases once
the nut has backed off 15 deg, and the ladder stops once the nut is backed
off >= 100 deg.  Then jaw off, lift, carry back, hang the wrench on the hook,
hold still, and probe the nut with the same torque-limited drag (<= 5 mN.m,
60 deg/s) -- a loose nut follows it.

Run:  python -m force_bench.appendix_tasks.loosen_nut_with_wrench_from_hook
      (TB_OUT_ROOT=<dir> TB_RENDER_FAST=1 TB_MULTIVIEW=0 for iterations;
       WH_STOP_PHASE=<phase> / WH_STOP_T=<s> stop early and skip the QA)
"""

from __future__ import annotations

import warp as wp

from force_bench.appendix_tasks._wrench_hook import (
    Mode, run, LOOSE_PRELOAD, LOOSE_FREE_DEG, LOOSEN_DONE_DEG)

MODE = Mode(
    name="loosen_nut_with_wrench_from_hook",
    title="Loosen nut with wrench (wrench from the hook shelf)",
    spec="The nut is loosened and can rotate freely when a small force is applied in "
         "an attempt to rotate the nut. The robot movement stops after terminating the "
         "loosening motion, moving away from, and breaking contact with the nut.",
    tighten=False,
    z_above_seat=0.0,
    theta_pref_handle_deg=160.0,
    changes=[
        "user request: the wrench starts HANGING on the middle hook of the hanging board "
        "(appendix: already in hand) -- taken off along the hook centre-line and hung "
        "back at the end",
        "finalized-kit parts: wrench (PLA 13.5 g), bolt block (bolt shaft through the "
        "block top = the seat), nut seated on the block top",
        f"fully tightened = seated + seat preload (elasto-plastic stiction, breakaway "
        f"{LOOSE_PRELOAD*1000:.0f} mN.m, released after {LOOSE_FREE_DEG:.0f} deg of "
        f"backing off; documented mechanism model)",
        f"loosened = backed off >= {LOOSEN_DONE_DEG:.0f} deg through the jaw",
        "probe after the robot left: torque-limited drag (<= 5 mN.m, 60 deg/s) 1 s each way",
    ],
)


def main():
    return run(MODE)


if __name__ == "__main__":
    wp.init()
    main()
