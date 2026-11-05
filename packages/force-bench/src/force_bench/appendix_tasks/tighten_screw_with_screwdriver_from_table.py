"""Appendix task "Tighten screw with screwdriver" (Table I), with the tool
picked up from the table and put back (user addition).

The screwdriver starts lying on the table beside the finalized-kit toy
block; the screw sits in the block with its threads unengaged (head 4 mm
up, 615 deg short of the seat).  The FR3 picks the tool up with a
top-down pinch, pitches it tip-down into a side grasp, inserts it into the
square recess and -- keeping that one grasp throughout -- tightens the screw
in ~90 deg strokes (lift out, swing back in the air, re-insert between
them) until it seats and snugs, takes the tool out, lays it back on the
table where it was, lets go and moves away; a small torque then tries to
turn the screw.  Scene, thread model, grasp plan and measurements:
``force_bench.appendix_tasks._screwdriver_table``.

Run: python -m force_bench.appendix_tasks.tighten_screw_with_screwdriver_from_table
     (TB_OUT_ROOT=<dir> TB_RENDER_FAST=1 TB_MULTIVIEW=0 for iterations;
      SDT_STOP=<phase> stops when that phase begins)
"""

from force_bench.appendix_tasks._screwdriver_table import run


def main():
    return run("tighten")


if __name__ == "__main__":
    import warp as wp
    wp.init()
    main()
