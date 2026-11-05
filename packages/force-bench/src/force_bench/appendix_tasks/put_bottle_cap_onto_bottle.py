"""Appendix task "Put bottle cap onto bottle" (task 19): a kit bottle without its cap stands in the workspace;
the cap starts held in the pinch above it, is screwed on with regrasp strokes and tightened.  Scene, expert,
physics model and verdict: ``_kit_thread.py`` (the kit parts).

Run: python -m force_bench.appendix_tasks.put_bottle_cap_onto_bottle [--bottle bottle-1 --rev 2] [--seed N]
     -> outputs/appendix_tasks/put_bottle_cap_onto_bottle/  (variants: ..._<n> for bottle-n, ..._<rev>rev)
     --seed N  random bottle position in the paper's 0.15 x 0.15 m square (no yaw: the bottle is round); the cap
               starts in the pinch above it; ``force-bench run`` names the folder <row>__seed<N>
"""

from __future__ import annotations

import warp as wp

from force_bench.appendix_tasks._kit_thread import run


def main(argv=None):
    run("cap", "hand", argv)


if __name__ == "__main__":
    wp.init()
    main()
