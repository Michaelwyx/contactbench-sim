"""Derive an appendix-aligned variant from a suite task WITHOUT editing it.

A suite task is one big ``main()``: it builds its scene, calls
``Rig.run(duration, control, extra)`` once, computes its QA and calls
``Rig.finish(qa)``.  :func:`run_variant` runs that ``main()`` with four
class-level patches on :class:`force_bench.rig.Rig`, active only for the
duration of the call:

``__init__``  renames the rig to the variant's name and points its
              ``out_dir`` at ``outputs/appendix_tasks/<name>/``;
``drive_arm`` remembers the last commanded TCP pose and finger width, so an
              appended phase can start exactly where the expert left off;
``run``       adds a robot<->task-object contact probe, lets the variant
              wrap the expert's control callback (cut it, override it, or
              append phases after it) and extend the duration;
``finish``    lets the variant add its appendix checks to the suite's QA
              report, then writes ``appendix_qa.json`` beside it.

Under ``TB_GELSIGHT=1`` the scene builds a ``GelSightRig`` (``GS.make_rig``),
whose run loop is its own fork of ``Rig.run``: ``run`` is patched on it too,
wrapping that loop, and ``GS.make_rig`` (which points the rig at
``outputs/task_gelsight/<scene>/``) is wrapped to keep the variant's outputs,
tactile stacks included, in ``outputs/appendix_tasks/<name>/``.

Module-level constants the variant wants changed are set on the imported
task module by :meth:`Variant.configure` before ``main()`` runs and
restored afterwards, so a variant never leaks into another.
"""

from __future__ import annotations

import importlib
import inspect
import sys
from dataclasses import dataclass, field

import numpy as np

import force_bench.tactile.task as GS
from force_bench.rig import Rig
from force_bench.appendix_tasks.common import (
    APPENDIX_OUT, RobotContactProbe, ease, retreat_pose, approach_axis, run_name,
    check_released_and_clear, save_appendix_verdict, CONTACT_FREE_N, StopRun)

FINGER_BODIES = ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger")


@dataclass
class Variant:
    """One appendix task derived from one suite task."""

    name: str                       # appendix task id = output folder
    source: str                     # dotted module of the suite task
    appendix_task: str              # the appendix's task title
    spec: str                       # the appendix's success criterion, verbatim
    objects: tuple[str, ...]        # task-object body labels (contact check)
    static_shapes: tuple[str, ...] = ()
    argv: tuple[str, ...] = ()
    changes: list[str] = field(default_factory=list)
    overrides: dict = field(default_factory=dict)

    # ── the appended ending (release -> retreat -> still) ───────────────
    t_cut: float | None = None      # None = after the expert's own duration
    call_original_after_cut: bool = True
    open_width: float = 0.040       # finger dof to open to (m)
    t_open: float = 0.6
    retreat_m: float = 0.08
    t_retreat: float = 1.4
    retreat_up_bias: float = 0.0
    t_still: float = 1.0

    # collision cadence of the derived run (docs/physics_notes.md rules 5.1-5.3): a subclass sets True to collide
    # before every substep from the moment the rig is built, whoever drives it; None keeps the suite task's cadence.
    # (A plain class attribute, not a dataclass field, so a subclass can override it.)
    collide_every_substep = None

    # ── filled in at run time ───────────────────────────────────────────
    rig: object = None
    ns: dict = field(default_factory=dict)
    mod: object = None

    # hooks a subclass overrides --------------------------------------------
    def configure(self, mod) -> None:
        """Set module constants before main() (self.overrides by default)."""

    def cleanup(self, mod) -> None:
        """Undo anything configure() patched outside `mod` (always runs)."""

    def cut_time(self, duration: float) -> float | None:
        """Fixed cut time, or None for a state-dependent cut decided frame by
        frame by :meth:`should_cut` (then the run lasts dynamic_total)."""
        return duration if self.t_cut is None else self.t_cut

    def should_cut(self, rig, t: float) -> bool:
        return False

    def dynamic_total(self, duration: float) -> float:
        return duration

    def extra_duration(self) -> float:
        return self.t_open + self.t_retreat + self.t_still

    def ending(self, rig, t: float, tau: float) -> None:
        """Command the appended phases; tau = time since the cut."""
        p0, q0, f0 = self.start_cmd
        if tau < self.t_open:
            u = ease(tau / self.t_open)
            rig.drive_arm(p0, q0, fingers=f0 + (self.open_width - f0) * u)
            return
        p1 = retreat_pose(p0, q0, self.retreat_m, self.retreat_up_bias)
        u = ease((tau - self.t_open) / self.t_retreat)
        rig.drive_arm(p0 + (p1 - p0) * u, q0, fingers=self.open_width)

    def extra(self, rig, state, t) -> dict:
        return {}

    def criteria(self, rig, qa) -> dict[str, dict]:
        """Appendix criteria computed after the run (see common.py)."""
        return {}

    def drop_checks(self) -> tuple[str, ...]:
        """Suite checks that test phases the appendix task does not have."""
        return ()

    # helpers ----------------------------------------------------------------
    @property
    def t_release_end(self) -> float:
        return self.t_cut_eff + self.t_open

    def contact_history(self):
        h = self.rig.probes["robot_object"].history
        return np.array([r["t"] for r in h]), np.array([r["force"] for r in h])

    def released_and_clear(self, qa, t_from: float | None = None, min_move=None):
        t, f = self.contact_history()
        t0 = self.t_cut_eff + self.t_open + self.t_retreat if t_from is None else t_from
        tcp = np.array(self.tcp_log)
        ok = check_released_and_clear(
            qa, t, f, t0, tcp_z=None, extra={
                "tcp_moved_mm": round(float(np.linalg.norm(tcp[-1, 1:] - self.cut_tcp)) * 1000, 1)
                if len(tcp) and self.cut_tcp is not None else None})
        entry = qa.report["checks"]["released_and_clear"]
        if min_move is not None and len(tcp) and self.cut_tcp is not None:
            moved = float(np.linalg.norm(tcp[-1, 1:] - self.cut_tcp))
            entry["min_move_mm"] = round(min_move * 1000, 1)
            ok = ok and moved >= min_move
            entry["pass"] = bool(ok)
            qa.report["pass"] = all(v.get("pass", True) for v in qa.report["checks"].values())
        return entry


def _closure_ns(fn) -> dict:
    fn = getattr(fn, "__func__", fn)
    try:
        cv = inspect.getclosurevars(fn)
        return dict(cv.nonlocals)
    except (TypeError, ValueError):
        return {}


def run_variant(v: Variant) -> dict | None:
    mod = importlib.import_module(v.source)
    v.mod = mod
    saved = {k: getattr(mod, k) for k in v.overrides if hasattr(mod, k)}
    for k, val in v.overrides.items():
        setattr(mod, k, val)
    v.configure(mod)

    orig = {k: Rig.__dict__[k] for k in ("__init__", "run", "finish", "drive_arm")}
    # every class with its own run loop, mapped to that loop
    loops = {Rig: orig["run"]}
    if GS.GS_ON:
        from force_bench.tactile.rig import GelSightRig
        loops[GelSightRig] = GelSightRig.__dict__["run"]
    make_rig = GS.make_rig
    v.tcp_log, v.cut_tcp, v.start_cmd, v.t_cut_eff = [], None, None, None
    v.done_at = None
    result = {}

    def __init__(self, name, *a, **kw):
        orig["__init__"](self, v.name, *a, **kw)
        if v.collide_every_substep is not None:
            self.collide_every_substep = bool(v.collide_every_substep)
        self.out_dir = APPENDIX_OUT / run_name(v.name)
        self.appendix_source_name = name
        v.rig = self

    def drive_arm(self, tcp_pos, tcp_quat, fingers=0.04):
        self._appendix_cmd = (np.array(tcp_pos, float), np.array(tcp_quat, float),
                              float(fingers))
        return orig["drive_arm"](self, tcp_pos, tcp_quat, fingers)

    def run(self, duration, control_cb, extra_forces_cb=None, print_every=20):
        frame = inspect.currentframe().f_back
        ns = dict(frame.f_locals)
        del frame
        ns.update(_closure_ns(control_cb))
        v.ns = ns
        self.probes["robot_object"] = RobotContactProbe(
            self.model, list(v.objects), list(v.static_shapes))
        v.t_cut_eff = v.cut_time(duration)
        total = (v.dynamic_total(duration) if v.t_cut_eff is None
                 else v.t_cut_eff + v.extra_duration())
        fi = [FINGER_BODIES[0] in self.labels and self.labels.index(FINGER_BODIES[0]),
              FINGER_BODIES[1] in self.labels and self.labels.index(FINGER_BODIES[1])]
        print(f"[appendix] {v.name}: expert until "
              f"{'a state-dependent cut' if v.t_cut_eff is None else f't={v.t_cut_eff:.2f}s'}, "
              f"appended ending -> {total:.2f}s")

        def control(t):
            if v.done_at is not None and t >= v.done_at:
                raise StopRun
            if v.t_cut_eff is None and v.should_cut(self, t):
                v.t_cut_eff = t
                print(f"[appendix] {v.name}: cut at t={t:.2f}s")
            if v.t_cut_eff is None or t < v.t_cut_eff:
                self._appendix_cmd = None
                control_cb(t)
                if self._appendix_cmd is None:
                    # the expert drives the arm through its own IK call (not
                    # Rig.drive_arm): its last command is the IK solution
                    tcp = self.ik.tcp_pose()
                    qt = self.control.joint_target_q.numpy()
                    self._appendix_cmd = (tcp[:3].copy(), tcp[3:7].copy(),
                                          float(0.5 * (qt[7] + qt[8])))
                return
            if v.call_original_after_cut:
                control_cb(t)
            if v.start_cmd is None:
                p, q, f = self._appendix_cmd
                v.start_cmd = (p.copy(), q.copy(), f)
                bq = self.s0.body_q.numpy()
                v.cut_tcp = 0.5 * (bq[fi[0], :3] + bq[fi[1], :3])
            v.ending(self, t, t - v.t_cut_eff)

        def extra(state, t):
            out = dict(extra_forces_cb(state, t)) if extra_forces_cb else {}
            bq = state.body_q.numpy()
            mid = 0.5 * (bq[fi[0], :3] + bq[fi[1], :3])
            v.tcp_log.append((t, *mid))
            out["robot_object_contact_N"] = round(
                self.probes["robot_object"].history[-1]["force"], 4) \
                if self.probes["robot_object"].history else 0.0
            out.update(v.extra(self, state, t))
            return out

        loop = next(loops[c] for c in type(self).__mro__ if c in loops)
        try:
            return loop(self, total, control, extra, print_every)
        except StopRun:
            print(f"[appendix] {v.name}: ending complete, rollout stopped at "
                  f"t={len(self.rec.times) / self.fps:.2f}s")
            return self.rec

    def finish(self, qa):
        crit = v.criteria(self, qa)
        for k in v.drop_checks():
            if k in qa.report["checks"]:
                qa.report["checks"][k] = {**qa.report["checks"][k],
                                          "pass": True, "dropped_for_appendix": True,
                                          "pass_in_suite_run": qa.report["checks"][k].get("pass")}
        qa.report["pass"] = all(c.get("pass", True) for c in qa.report["checks"].values())
        report = orig["finish"](self, qa)
        result["verdict"] = save_appendix_verdict(
            self, task=v.appendix_task, source=v.source + (" " + " ".join(v.argv) if v.argv else ""),
            spec=v.spec, criteria=crit, changes=v.changes, physics_qa=report)
        return report

    def gs_make_rig(name, **kw):
        rig = make_rig(name, **kw)
        rig.out_dir = APPENDIX_OUT / run_name(v.name)
        return rig

    Rig.__init__, Rig.run, Rig.finish, Rig.drive_arm = __init__, run, finish, drive_arm
    for cls in loops:
        cls.run = run
    if GS.GS_ON:
        GS.make_rig = gs_make_rig
    argv = sys.argv
    sys.argv = [v.source, *v.argv]
    try:
        mod.main()
    finally:
        sys.argv = argv
        for k, fn in orig.items():
            setattr(Rig, k, fn)
        for cls, fn in loops.items():
            cls.run = fn
        GS.make_rig = make_rig
        for k, val in saved.items():
            setattr(mod, k, val)
        v.cleanup(mod)
    return result.get("verdict")


__all__ = ["Variant", "run_variant", "approach_axis", "CONTACT_FREE_N"]
