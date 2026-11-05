"""Shared pieces of the ContactBench tasks.

The paper's Appendix C specifies every task by an initial condition, a
description and a success criterion.  Most of its success criteria end the
same way:

    "The robot movement stops after terminating the <motion>, moving away
     from, and breaking contact with the task objects."

and the fastening ones add a holding test:

    "... tightened and remains stationary when a small force is applied in
     an attempt to rotate ..."

Some tasks build on a scene module under ``force_bench.tasks`` (a mechanism
and its contact model, developed and calibrated on its own): the scene
builder, geometry and mechanism are imported from it, untouched, and the
task sets its own start state, policy ending and verdict.  The others build
their scene here, most of them from the kit library.  Everything a task
writes goes to ``outputs/appendix_tasks/<name>/``.

This module holds what every task needs:

- :func:`appendix_rig` -- a ``Rig`` whose outputs land in the appendix tree;
- :class:`RobotContactProbe` -- total contact force between ANY robot link
  and a set of task-object bodies, logged per frame like the other probes;
- :func:`retreat_pose` -- the stand-off pose along the tool's approach axis;
- :func:`check_released_and_clear` -- the "breaking contact" criterion;
- :func:`save_appendix_verdict` -- ``appendix_qa.json``: the appendix's own
  success criterion next to the scene's physics QA.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import newton

from force_bench.paths import OUTPUTS_ROOT
from force_bench.rig import Rig

APPENDIX_OUT = OUTPUTS_ROOT / "appendix_tasks"

# Contact below this total force (N) counts as "not in contact": MuJoCo
# reports margin contacts at exactly zero and resting grazes at ~1e-3 N.
CONTACT_FREE_N = 0.05


class StopRun(Exception):
    """Raised from a control callback to end a rollout early (the recorder
    keeps every frame captured so far)."""


def run_until(rig, duration, control, extra, done):
    """``rig.run`` that stops as soon as ``done(t)`` is true, so a variant
    whose ending starts at a state-dependent time does not idle out a
    worst-case budget on video."""
    def ctl(t):
        if done(t):
            raise StopRun
        control(t)
    try:
        return rig.run(duration, ctl, extra)
    except StopRun:
        print(f"[appendix] ending complete -> rollout stopped at "
              f"t={len(rig.rec.times) / rig.fps:.2f}s")
        return rig.rec


def run_name(name: str) -> str:
    """The output folder of a run: ``name``, or ``TB_RUN_NAME`` when it is set (``force-bench run`` sets it for a run
    with ``--set`` or ``--name``, so a variation never overwrites the row's own outputs)."""
    return os.environ.get("TB_RUN_NAME") or name


def appendix_rig(name: str, rig_cls=Rig, **kw) -> Rig:
    """A Rig named `name` that writes to outputs/appendix_tasks/<name>/ (or ``TB_RUN_NAME``, see run_name).

    The recorder is created in ``finalize()`` from ``rig.out_dir``, so
    redirecting it here (before finalize) moves every artefact."""
    rig = rig_cls(name, **kw)
    rig.out_dir = APPENDIX_OUT / run_name(name)
    return rig


@dataclass(frozen=True)
class Placement:
    """Where one task object starts: nominal ``xy`` [m] and ``yaw`` [rad], how far a seeded start may move it
    (uniform in +-``dxy`` per axis about ``centre``, and +-``dyaw`` about the nominal yaw), and the radius of the disc
    it occupies on the table.  ``centre`` defaults to the nominal ``xy``; objects scattered over one square (several
    cups) all give the square's centre, so that each is drawn anywhere in it."""
    xy: tuple[float, float]
    yaw: float = 0.0
    dxy: float = 0.0
    dyaw: float = 0.0
    radius: float = 0.0
    centre: tuple[float, float] | None = None


def sample_start(placements: dict[str, Placement], seed: int | None, gap: float = 0.005,
                 tries: int = 1000) -> dict[str, tuple[float, float, float]]:
    """The start pose (x, y, yaw) of every object: the nominal one without a seed, otherwise a reproducible draw
    in which no two objects' discs come closer than ``gap`` (``force-bench run <row> --seed N``).  A task records
    the draw in its verdict and keeps its ranges inside what its scripted policy was checked on."""
    if seed is None:
        return {k: (p.xy[0], p.xy[1], p.yaw) for k, p in placements.items()}
    rng = np.random.default_rng(seed)
    centre = {k: p.xy if p.centre is None else p.centre for k, p in placements.items()}
    for _ in range(tries):
        out = {k: (centre[k][0] + rng.uniform(-p.dxy, p.dxy), centre[k][1] + rng.uniform(-p.dxy, p.dxy),
                   p.yaw + rng.uniform(-p.dyaw, p.dyaw)) for k, p in placements.items()}
        keys = list(out)
        if all(np.hypot(out[a][0] - out[b][0], out[a][1] - out[b][1])
               >= placements[a].radius + placements[b].radius + gap
               for i, a in enumerate(keys) for b in keys[i + 1:]):
            return out
    raise RuntimeError(f"no collision-free start in {tries} draws (seed {seed}); the ranges overlap")


def robot_body_indices(model) -> list[int]:
    return [i for i, lbl in enumerate(model.body_label) if str(lbl).startswith("fr3/")]


class RobotContactProbe:
    """ForceProbe-compatible: summed contact force magnitude between every
    robot link and the listed task-object bodies.

    Magnitudes are summed (not the vector sum PairProbe takes) so two pads
    squeezing in opposite directions read as contact, not as zero."""

    def __init__(self, model, object_bodies: list[str] | list[int],
                 static_shapes: list[str] = ()):
        """object_bodies: body labels/indices; static_shapes: labels of
        world-attached shapes (a socket, a bin) that have no body."""
        bodies = {model.body_label.index(b) if isinstance(b, str) else int(b)
                  for b in object_bodies}
        shape_body = model.shape_body.numpy()
        shapes = [int(s) for s in np.flatnonzero(np.isin(shape_body, list(bodies)))]
        labels = list(model.shape_label)
        shapes += [labels.index(s) for s in static_shapes]
        self.sensor = newton.sensors.SensorContact(
            model, sensing_bodies=robot_body_indices(model),
            counterpart_shapes=sorted(set(shapes)), measure_total=False)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        self.sensor.update(state, contacts)
        fm = self.sensor.force_matrix.numpy()          # (n_robot, n_obj, 3)
        ff = self.sensor.force_matrix_friction.numpy()
        f = float(np.linalg.norm(fm, axis=-1).sum())
        fr = float(np.linalg.norm(ff, axis=-1).sum())
        rec = {"t": t, "force": f, "friction": fr, "normal": f - fr,
               "vec": fm.sum(axis=(0, 1)).tolist()}
        self.history.append(rec)
        return rec


def quat_rotate(q, v) -> np.ndarray:
    """Rotate v by the xyzw quaternion q."""
    x, y, z, w = (float(c) for c in q)
    u = np.array([x, y, z])
    v = np.asarray(v, float)
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def approach_axis(q_tcp) -> np.ndarray:
    """World direction the fingers point along (the TCP's +z)."""
    a = quat_rotate(q_tcp, (0.0, 0.0, 1.0))
    return a / np.linalg.norm(a)


def retreat_pose(tcp, q_tcp, distance: float, world_up_bias: float = 0.0) -> np.ndarray:
    """TCP position `distance` back along the approach axis (away from the
    object), optionally blended toward world +z (for side approaches near a
    table)."""
    d = -approach_axis(q_tcp)
    if world_up_bias:
        d = d + world_up_bias * np.array([0.0, 0.0, 1.0])
        d /= np.linalg.norm(d)
    return np.asarray(tcp, float) + distance * d


def ease(u: float) -> float:
    u = float(np.clip(u, 0.0, 1.0))
    return 3 * u * u - 2 * u ** 3


def check_released_and_clear(qa, t, contact_N, t_from: float, *, name="released_and_clear",
                             tcp_z=None, tcp_rise_min=None, extra: dict | None = None) -> bool:
    """The appendix's "moving away from, and breaking contact with the task
    objects": over [t_from, end] the summed robot<->object contact force
    stays below CONTACT_FREE_N."""
    t = np.asarray(t, float)
    f = np.asarray(contact_N, float)
    m = t >= t_from
    peak = float(f[m].max()) if m.any() else float("inf")
    ok = bool(m.any() and peak < CONTACT_FREE_N)
    entry = {"pass": ok, "window_s": [round(float(t_from), 2), round(float(t[-1]), 2)],
             "robot_object_contact_peak_N": round(peak, 4),
             "threshold_N": CONTACT_FREE_N}
    if tcp_z is not None and tcp_rise_min is not None:
        rise = float(np.asarray(tcp_z)[-1] - np.asarray(tcp_z)[m][0]) if m.any() else 0.0
        entry["tcp_moved_away_mm"] = round(rise * 1000, 1)
        entry["min_move_mm"] = round(tcp_rise_min * 1000, 1)
        ok = ok and rise >= tcp_rise_min
        entry["pass"] = bool(ok)
    if extra:
        entry.update(extra)
    qa.report["checks"][name] = entry
    qa.report["pass"] &= ok
    return ok


def save_appendix_verdict(rig, *, task: str, source: str, spec: str,
                          criteria: dict[str, dict], changes: list[str],
                          physics_qa: dict | None = None) -> dict:
    """Write appendix_qa.json: the appendix criterion, item by item.

    criteria: {name: {"pass": bool, ...measurements}}; the verdict is the
    AND of the criteria and of the physics QA's integrity checks (finite,
    velocity, penetration, envelope, and with GelSight pads the three gel
    checks) when given."""
    # the published criterion (paper Table C1, verbatim) replaces the module's own wording of it
    from force_bench.appendix_tasks.registry import BY_NAME, CRITERIA
    row = BY_NAME.get(Path(rig.out_dir).name)
    if row is not None and row.number in CRITERIA:
        spec = CRITERIA[row.number]
    integrity = {}
    if physics_qa is not None:
        for k in ("finite", "velocity_spikes", "penetration", "workspace_envelope",
                  "gel_pads_loaded", "gel_coulomb_bounded", "gel_probe_crosscheck"):
            if k in physics_qa.get("checks", {}):
                integrity[k] = bool(physics_qa["checks"][k]["pass"])
    ok = all(bool(c.get("pass")) for c in criteria.values()) and all(integrity.values())
    out = {
        "appendix_task": task,
        "derived_from": source,
        "appendix_success_criterion": spec,
        "changes_vs_suite_task": changes,
        "criteria": criteria,
        "physics_integrity": integrity,
        "pass": bool(ok),
    }
    path = Path(rig.out_dir) / "appendix_qa.json"

    def _default(o):
        if isinstance(o, (np.bool_,)):
            return bool(o)
        if isinstance(o, np.integer):
            return int(o)
        if isinstance(o, np.floating):
            return float(o)
        raise TypeError(type(o))

    path.write_text(json.dumps(out, indent=1, default=_default))
    print(f"[appendix] {task}: {'PASS' if ok else 'FAIL'} -> {path}")
    return out
