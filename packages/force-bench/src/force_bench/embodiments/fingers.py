"""The simulation side of an embodiment: the hooks a task calls on the FR3's fingers.

:class:`Fingers` is the stock Franka Hand (the default): every hook leaves the scene exactly as the task builds it.
Another embodiment subclasses it and overrides what differs (``force_bench.finray.embodiment.FinRayFingers``).

A task that supports embodiments calls, in this order::

    fingers = embodiments.load(name)
    HP.setup_robot(rig)              # the FR3 first, with the task's finger drives
    fingers.mount(rig)               # right after the robot: replace or adjust the fingers
    ...                              # the task's scene, filtering arm links with fingers.is_finger(label)
    rig.finalize(...)
    fingers.finalize(rig)            # solver-side settings that need the built solver
    rig.probes["padL"] = fingers.pad_probe(rig.model, "L", target_shapes)
    ...                              # the episode, planned with the geometry (tip_below_tcp, underside, boxes)
    criteria.update(fingers.checks(states_npz, windows))   # the embodiment's own verdict checks

Frames: the TCP frame is ``fr3_hand_tcp`` (z = approach, y = closing axis); a finger coordinate q is the drive
target of one carriage (the two contact faces are 2 (q - face_offset) apart).
"""

from __future__ import annotations

import numpy as np

FINGER_LINKS = {"L": "fr3/fr3_leftfinger", "R": "fr3/fr3_rightfinger"}


class Fingers:
    """The stock Franka Hand fingers (rubber pads on rigid fingertips)."""

    # geometry in the TCP frame, metres (the FR3 description meshes; pad face measured on mesh 43)
    tip_below_tcp = 0.0095          # fingertip block bottom, beyond the TCP along the approach
    face_offset = 0.00008           # contact face = finger coordinate - this
    tip_half_width = 0.00875        # fingertip half-width across the closing axis
    tip_thickness = 0.01518         # fingertip block along the closing axis
    tip_top = 0.0090                # tip block top, behind the TCP along the approach
    face_bottom_above_tip = 0.0     # the flat contact face reaches the fingertip's lowest point

    def __init__(self, info):
        self.info = info
        self.name = info.name

    # ── building ──────────────────────────────────────────────────────────────────────────────────────────────────
    def mount(self, rig) -> None:
        """Attach the fingers to the FR3 just added to ``rig`` (call right after the robot, before any other body
        or joint).  The stock fingers are already there."""

    def finalize(self, rig) -> None:
        """Settings that need the built solver (call right after ``rig.finalize``)."""

    def is_finger(self, label: str) -> bool:
        """Whether a body label belongs to the fingers (tasks keep finger contact live and filter the arm)."""
        return "finger" in str(label)

    def finger_bodies(self, model, side: str) -> list[int]:
        """Body indices of one finger ('L' or 'R'): what a pad force probe senses."""
        return [list(model.body_label).index(FINGER_LINKS[side])]

    def pad_probe(self, model, side: str, counterpart_shapes):
        """Contact force on one finger from the given shapes (``force``, ``normal``, ``friction`` per frame)."""
        from force_bench.appendix_tasks._hang_plate import PairProbe
        return PairProbe(model, FINGER_LINKS[side], counterpart_shapes)

    # ── planning ──────────────────────────────────────────────────────────────────────────────────────────────────
    def boxes(self, q: float) -> list[tuple[np.ndarray, np.ndarray]]:
        """Axis-aligned boxes (lo, hi) in the TCP frame enclosing both fingers opened to q: the fingertip block, and
        the flare and mount as one generous box."""
        out = []
        for s in (+1.0, -1.0):
            y0, y1 = sorted([s * q, s * (q + self.tip_thickness)])
            out.append((np.array([-self.tip_half_width, y0, -self.tip_top]),
                        np.array([self.tip_half_width, y1, self.tip_below_tcp])))
            y0, y1 = sorted([s * (q + 0.004), s * (q + 0.026)])
            out.append((np.array([-0.011, y0, -0.047]), np.array([0.011, y1, -self.tip_top])))
        return out

    def underside(self, y):
        """Height (m) of the finger's underside above its lowest point, at ``y`` (m) outward from the contact face,
        at rest; inf where the finger has no material (planning a fingertip over a part lying under it).  The stock
        fingertip block is flat underneath; beyond it, the flare starts behind the TCP."""
        y = np.asarray(y, float)
        h = np.where(y <= 0.026, self.tip_below_tcp + self.tip_top, np.inf)
        return np.where((y >= 0.0) & (y <= self.tip_thickness), 0.0, np.where(y < 0.0, np.inf, h))

    # ── verdict ───────────────────────────────────────────────────────────────────────────────────────────────────
    def checks(self, states_npz, windows: dict[str, list[tuple[float, float]]]) -> dict:
        """The embodiment's own verdict checks, from the saved states (``{name: {"pass": bool, ...}}``); ``windows``
        maps "grasp" (fingers closed on the part) and "free" (fingers open) to lists of (t0, t1) intervals."""
        return {}
