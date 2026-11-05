"""Appendix task "Install light bulb" (task 31), the pick-up variant: the kit bulb starts upright on a printed
stand on the table, is picked up, carried over the kit's lightbulb slot and screwed in.  Scene, expert,
physics model and verdict: ``_kit_thread.py`` (the kit parts).

Run: python -m force_bench.appendix_tasks.install_light_bulb_from_table
     -> outputs/appendix_tasks/install_light_bulb_from_table/
"""

from __future__ import annotations

import warp as wp

from force_bench.appendix_tasks._kit_thread import run


def main(argv=None):
    run("bulb", "table", argv)


if __name__ == "__main__":
    wp.init()
    main()
