"""Appendix task "Loosen screw with screwdriver" (paper Table C1 task 16), screwdriver in the hand.

The kit's toy assembly block with the kit nut and the kit screw seated in its right-hand hole (20 mN.m breakaway);
the FR3 holds the kit screwdriver tip-down in a top-down pinch, inserts it into the screw's square recess, breaks the
screw loose and backs it off 200 deg in one joint-7 stroke, lifts the tool out and moves away with it; a small torque
then turns the screw.  Scene, thread model, expert and verdict: ``force_bench.appendix_tasks._screwdriver_kit``.

Run: python -m force_bench.appendix_tasks.loosen_screw_with_screwdriver [--seed N]
     -> outputs/appendix_tasks/loosen_screw_with_screwdriver/
"""

from force_bench.appendix_tasks._screwdriver_kit import run


def main(argv=None):
    return run("loosen", argv)


if __name__ == "__main__":
    import warp as wp
    wp.init()
    main()
