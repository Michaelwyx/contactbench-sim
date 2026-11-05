"""Appendix task "Tighten nut with wrench" (Table I, #17), with the wrench
taken from the hook shelf and hung back (user request, 2026-10-16).

    Description: A nut is threaded onto a screw head in the workspace but is
    left untightened.  Use the wrench grasped in hand to tighten the nut.
    Success: The nut is tightened and remains stationary when a small force
    is applied in an attempt to rotate the nut.  The robot movement stops
    after terminating the tightening motion, moving away from, and breaking
    contact with the nut.

Scene, mechanism model and expert: ``_wrench_hook.py``.  Start state: the
kit wrench hangs on the middle hook of the hanging board; the kit nut is
threaded on the kit bolt block's shaft, finger-loose 0.6 mm above its seat
on the block top (it settles 0.29 mm onto its thread flank at t = 0; with
the thread's 0.6 mm axial backlash that is ~140 deg of free run-down before
the clamp starts to build).

EXPERT (one continuous rollout, fingertip contact only, nothing welded):
  1. take the wrench off the hook: approach the hanging wrench along its
     plate normal, pinch the handle across its width at the balance point
     (pads over the whole 7.5 mm edge), lift the hole off the bar
     (force-gated on the board reaction), and carry it back out along the
     hook centre-line -- bar, 60 deg bend (tilt 86 -> 30 deg), stub, past
     the tip -- closed-loop on the MEASURED hole centre (hang_wrench's path,
     run backwards); then straight out 60 mm from the board before the
     wrench is turned flat (flattening beside the board swings the hole end
     down onto the hook);
  2. carry it flat to the nut, jaw 20 mm back from it; re-measure the
     in-hand pose on the static wrench; lower the jaw plane to 1 mm above
     the block top (closed-loop on the measured jaw seat, squared to the
     MEASURED flats);
  3. stroke ladder: hold, re-measure the in-hand pose, slide the jaw on --
     fast to 11 mm, then slowly, advancing only while the measured jaw is
     square to a flat pair (< 0.75 deg) and centred on the measured nut
     (< 0.15 mm), backing off on any touch (a square jaw slides on without
     touching the free-spinning nut) -- swing 60 deg clockwise about the
     bolt, jaw off, index back to the next flats, repeat.  On the seat the
     nut first turns ~90 deg free (thread backlash, swing capped at
     25 deg/s); once the measured jaw torque builds (> 6 mN.m) the swing
     slows to 8 deg/s and stops when it stalls at 30 mN.m: the nut is
     tight, and the seat-preload model takes 0.7 x the peak;
  4. jaw off, lift, re-measure the in-hand pose, carry back via the same
     stand-off point and hang the wrench on the hook (hang_wrench's expert:
     thread the stub, follow the bend, seat, open, retreat), hold still;
  5. probe: a torque-limited drag on the nut (<= 5 mN.m), 1 s each way.

Run:  python -m force_bench.appendix_tasks.tighten_nut_with_wrench_from_hook
      (TB_OUT_ROOT=<dir> TB_RENDER_FAST=1 TB_MULTIVIEW=0 for iterations;
       WH_STOP_PHASE=<phase> / WH_STOP_T=<s> stop early and skip the QA)
"""

from __future__ import annotations

import warp as wp

from force_bench.appendix_tasks._wrench_hook import Mode, run, TAU_TIGHT, PRELOAD_FRAC

MODE = Mode(
    name="tighten_nut_with_wrench_from_hook",
    title="Tighten nut with wrench (wrench from the hook shelf)",
    spec="The nut is tightened and remains stationary when a small force is applied "
         "in an attempt to rotate the nut. The robot movement stops after terminating "
         "the tightening motion, moving away from, and breaking contact with the nut.",
    tighten=True,
    z_above_seat=0.0006,
    theta_pref_handle_deg=200.0,
    changes=[
        "user request: the wrench starts HANGING on the middle hook of the hanging board "
        "(appendix: already in hand) -- taken off along the hook centre-line and hung "
        "back at the end",
        "finalized-kit parts: wrench (PLA 13.5 g), bolt block (bolt shaft through the "
        "block top = the seat), nut threaded 0.6 mm above the seat",
        f"tightened = torque-up against the seat until the measured jaw torque stalls at "
        f"{TAU_TIGHT*1000:.0f} mN.m; seat preload = elasto-plastic stiction with "
        f"breakaway {PRELOAD_FRAC} x the peak (documented mechanism model)",
        "probe after the robot left: torque-limited drag (<= 5 mN.m, 60 deg/s) 1 s each way",
    ],
)


def main():
    return run(MODE)


if __name__ == "__main__":
    wp.init()
    main()
