"""Appendix task "Install light bulb" (task 31), the appendix start: the kit bulb is held in the pinch above
the kit's lightbulb slot.  Scene, expert, physics model and verdict: ``_kit_thread.py`` (the kit parts).

Run: python -m force_bench.appendix_tasks.install_light_bulb [--bulb lightbulb-1 --size 55mm --rev 2] [--seed N]
     -> outputs/appendix_tasks/install_light_bulb/  (other variants: install_light_bulb_<n>_<size>,
        install_light_bulb_<rev>rev; see _kit_thread.row_name)
     --seed N  random socket position in the paper's 0.15 x 0.15 m square (no yaw: the socket is round); the bulb
               starts in the pinch above it; ``force-bench run`` names the folder <row>__seed<N>
"""

from __future__ import annotations

import warp as wp

from force_bench.appendix_tasks._kit_thread import run


def main(argv=None):
    run("bulb", "hand", argv)


if __name__ == "__main__":
    wp.init()
    main()
