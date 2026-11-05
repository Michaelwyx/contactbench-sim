"""Appendix Table I #13 "Tighten screw with Allen key" -- the key is picked
up from the table, tightens the screw by orbiting strokes with re-insertion,
and is put back on the table (bare FR3 gripper).

Description (appendix): "A screw is placed into the screw slot of an assembly
block in the workspace, but the threads are unengaged.  Use the Allen key
grasped in hand to tighten the screw."
Success: "The screw is tightened and remains stationary when a small force is
applied in an attempt to rotate the screw.  The robot movement stops after
terminating the tightening motion, moving away from, and breaking contact
with the screw."

Scene, mechanism, expert and verdict: ``_allen_key.py``.

Run:  python -m force_bench.appendix_tasks.tighten_screw_with_allen_key_from_table
"""

from __future__ import annotations

import warp as wp

from force_bench.appendix_tasks._allen_key import run


def main():
    return run("tighten")


if __name__ == "__main__":
    wp.init()
    main()
