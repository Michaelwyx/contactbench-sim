"""Appendix task "Tighten screw with screwdriver" (paper Table C1 task 15), screwdriver in the hand.

The kit's toy assembly block with the kit nut and the kit screw in its right-hand hole, threads unengaged; the FR3
holds the kit screwdriver tip-down in a top-down pinch, inserts it into the screw's square recess, tightens the
screw in joint-7 strokes until it seats and snugs, lifts the tool out and moves away with it; a small torque then
tries to turn the screw.  Scene, thread model, expert and verdict: ``force_bench.appendix_tasks._screwdriver_kit``.

Run: python -m force_bench.appendix_tasks.tighten_screw_with_screwdriver [--screw 7.5] [--seed N]
     -> outputs/appendix_tasks/tighten_screw_with_screwdriver/ (--screw 7.5: ..._longer_screw)
"""

from force_bench.appendix_tasks._screwdriver_kit import run


def main(argv=None):
    return run("tighten", argv)


if __name__ == "__main__":
    import warp as wp
    wp.init()
    main()
