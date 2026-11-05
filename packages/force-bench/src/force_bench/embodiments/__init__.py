"""Embodiments of the FR3 hand: which fingers the Franka Hand's two carriages carry.

An embodiment is every property of the fingers that a task's scripted policy and its verdict depend on: how the
fingers are built and attached to the FR3 in the scene, where they are relative to the FR3's TCP
(``fr3_hand_tcp``), how far they open, their contact and solver settings, and the checks they add to a verdict.
Each one is defined once, here (:data:`EMBODIMENTS`); its simulation code is a :class:`~force_bench.embodiments.
fingers.Fingers` subclass, imported only when a run uses it (:func:`load`).

    force-bench embodiments                                 the list, and the tasks that support each
    force-bench run <task> --embodiment finray [--seed N]   -> outputs/appendix_tasks/<task>__finray[_seed<N>]/

``franka`` (the stock fingers) is the default and every task supports it.  A task supports another embodiment
only when it opts in: its module is listed in :data:`TASKS` and its ``main()`` takes ``--embodiment``
(:func:`add_argument`); any other combination is refused before the scene is built (:func:`check`).

This module imports nothing heavy (the CLI reads it to list and to check); docs/embodiments.md explains the data
flow and how to add an embodiment.
"""

from __future__ import annotations

import importlib
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Embodiment:
    """One finger set for the FR3's Franka Hand.  Lengths in mm (for listings and docs; the implementation class
    holds the same numbers in metres, and the tests check that they agree)."""
    name: str                          # CLI value of --embodiment, and the output-folder suffix
    title: str
    summary: str
    impl: str                          # "module:Class" of its Fingers implementation
    finger_width_mm: float             # across the closing axis (along the object in a pinch)
    tip_below_tcp_mm: float            # fingertip bottom beyond fr3_hand_tcp, along the approach axis
    tip_thickness_mm: float            # fingertip thickness along the closing axis (the room it needs beside a part)
    opening_mm: tuple[float, float]    # gap between the two contact faces, fully closed .. fully open
    compliance: str                    # "rigid", or the compliance model in a few words
    contact: str                       # contact material and friction
    solver: dict = field(default_factory=dict)   # solver options it needs (merged into rig.solver_options)
    collide_every_substep: bool = False          # needs collision every substep (rig.collide_every_substep)
    checks: tuple[str, ...] = ()       # the verdict checks it adds
    bodies_per_finger: int = 1
    doc: str = ""                      # its reference page


DEFAULT = "franka"

EMBODIMENTS: dict[str, Embodiment] = {e.name: e for e in (
    Embodiment(
        name="franka", title="Franka Hand, stock fingers",
        summary="the FR3's own fingers: rigid fingertip blocks with elastomer pads",
        impl="force_bench.embodiments.fingers:Fingers",
        finger_width_mm=17.5, tip_below_tcp_mm=9.5, tip_thickness_mm=15.18, opening_mm=(0.0, 80.0),
        compliance="rigid", contact="elastomer pads, friction set by each task"),
    Embodiment(
        name="finray", title="Fin Ray TPU fingers on a printed adapter",
        summary="a 3D-printed Fin Ray finger (TPU 95A, 123.4 mm long) on each carriage through a printed adapter",
        impl="force_bench.finray.embodiment:FinRayFingers",
        finger_width_mm=25.8, tip_below_tcp_mm=100.8, tip_thickness_mm=5.3, opening_mm=(0.0, 80.0),
        compliance="pseudo-rigid-body linkage of its walls and ribs, 71 bodies per finger, E = 26 MPa (a data-sheet "
                   "estimate)",
        contact="TPU, mu 1.0", solver={"impratio": 10.0}, collide_every_substep=True,
        checks=("finray_deformation",), bodies_per_finger=71, doc="docs/finray.md"),
)}

# The tasks that opted in (appendix_tasks module -> the embodiments its policy supports).  Every task supports the
# default; a task listed here takes --embodiment and its policy is adapted to each embodiment it names.
TASKS: dict[str, tuple[str, ...]] = {
    "retrieve_utensil_from_bin": ("franka", "finray"),
}


def names() -> list[str]:
    return list(EMBODIMENTS)


def get(name: str) -> Embodiment:
    try:
        return EMBODIMENTS[name]
    except KeyError:
        raise KeyError(f"unknown embodiment {name!r}; one of {', '.join(EMBODIMENTS)} "
                       f"(force-bench embodiments)") from None


def supported(module: str) -> tuple[str, ...]:
    """The embodiments a task module supports (the default always)."""
    return TASKS.get(module, (DEFAULT,))


def check(module: str, name: str) -> Embodiment:
    """The embodiment ``name`` if the task module supports it; ValueError (with the reason) if not."""
    emb = get(name)
    ok = supported(module)
    if name not in ok:
        raise ValueError(f"this task does not support the {name!r} embodiment ({emb.title}); it supports "
                         f"{', '.join(ok)} (see force-bench embodiments)")
    return emb


def tasks_supporting(name: str) -> list[str] | None:
    """Task modules that support ``name`` (None: every task, i.e. the default)."""
    get(name)
    if name == DEFAULT:
        return None
    return [m for m, ok in TASKS.items() if name in ok]


def load(name: str = DEFAULT):
    """The Fingers implementation of an embodiment (imports its simulation code)."""
    emb = get(name)
    mod, cls = emb.impl.split(":")
    return getattr(importlib.import_module(mod), cls)(emb)


def add_argument(parser, module: str) -> None:
    """``--embodiment`` for a task's own argument parser, with the embodiments the module supports."""
    parser.add_argument("--embodiment", choices=supported(module), default=DEFAULT,
                        help="the fingers on the FR3 hand (force-bench embodiments; default: %(default)s)")


def run_name(row: str, name: str) -> str:
    """The output folder of a row run with ``name``: the row's own for the default, ``<row>__<name>`` otherwise (so
    a run with other fingers never overwrites the row's outputs)."""
    return row if name == DEFAULT else f"{row}__{name}"
