"""``force-bench`` command line for the ContactBench 1.0 simulation.

Benchmark tasks (ContactBench, paper Appendix C):
    force-bench list                       the benchmark tasks available in simulation
    force-bench run <task> [args]          one task -> outputs/appendix_tasks/<task>/ (a task marked "scene only"
                                           has no scripted policy: the run records its scene, the arm holding still)
    force-bench run-all [tasks] [-j N]     several tasks in parallel + outputs/appendix_tasks/summary.json
    force-bench info <task>                a task's definition, start state, variations and documented deviations
    force-bench variations [task]          the website's variation axes each task supports, and their values
    force-bench run <task> --set "axis=value" [--set ...] [--name N]
                                           a variation of a task (e.g. --set "Insertion force=0.85mm")
    force-bench embodiments                the finger sets the FR3 hand can carry, and the tasks supporting each
    force-bench run <task> --embodiment E  the task with other fingers -> outputs/appendix_tasks/<task>__E/
                                           (e.g. --embodiment finray; combines with --seed and --set)

The printable kit (every asset of the project website, sim-ready; docs/assets.md):
    force-bench kit list | info <key> | check [keys] | import | export-usd [keys]

Teleoperation and data:
    force-bench teleop-ui [--scene T] [--ui gl|web|both|none] [--input keyboard|gamepad|spacemouse]
                                           drive a task by hand with live contact forces (docs/teleoperation.md)
    force-bench teleop [--no-guard]        the minimal guarded FR3 teleop viewer
    force-bench gate <tasks> [--repeat N]  adversarial teleop acceptance gate (docs/teleop_guard.md)
    force-bench guard-selftest             the teleop guard's self-test
    force-bench collect [...]              record scripted rollouts as aligned demonstrations
    force-bench dataset [...]              inspect recorded episodes

Tactile: ``TB_GELSIGHT=1`` mounts simulated GelSight Mini pads on the tasks that support them (``force-bench list``
marks them; docs/tactile.md).  ``TB_OUT_ROOT`` redirects every artefact; ``TB_LOOK=studio`` switches the kit
colours off.
"""

from __future__ import annotations

import sys


def _run_module(modname: str, argv: list[str], prog: str, pass_argv: bool):
    import importlib
    import runpy
    sys.argv = [prog, *argv]
    mod = importlib.import_module(modname)
    if hasattr(mod, "main"):
        rc = mod.main(argv) if pass_argv else mod.main()
        raise SystemExit(0 if rc is None else rc)
    runpy.run_module(modname, run_name="__main__", alter_sys=True)


def _run_options(args: list[str]) -> tuple[list[str], str | None, int | None, list[str]]:
    """Split ``force-bench run`` arguments into --set values, --name, --seed and the module's own arguments."""
    sets, name, seed, extra = [], None, None, []
    i = 0
    while i < len(args):
        a = args[i]
        key, eq, val = a.partition("=")
        if key in ("--set", "--name", "--seed"):
            if not eq:
                if i + 1 >= len(args):
                    raise SystemExit(f"force-bench run: {key} needs a value")
                val = args[i + 1]
                i += 1
            if key == "--set":
                sets.append(val)
            elif key == "--name":
                name = val
            else:
                seed = int(val)
        else:
            extra.append(a)
        i += 1
    return sets, name, seed, extra


def _embodiment_option(args: list[str]) -> tuple[str | None, list[str]]:
    """Take ``--embodiment E`` (or ``--embodiment=E``) out of ``force-bench run`` arguments."""
    name, rest = None, []
    i = 0
    while i < len(args):
        key, eq, val = args[i].partition("=")
        if key == "--embodiment":
            if not eq:
                if i + 1 >= len(args):
                    raise SystemExit("force-bench run: --embodiment needs a value (force-bench embodiments)")
                val = args[i + 1]
                i += 1
            name = val
        else:
            rest.append(args[i])
        i += 1
    return name, rest


def _list_embodiments() -> None:
    import textwrap
    from force_bench import embodiments as E
    print(f"{'name':8s} {'fingers':42s} {'width':>6s} {'tip below TCP':>14s}  tasks")
    for e in E.EMBODIMENTS.values():
        tasks = E.tasks_supporting(e.name)
        print(f"{e.name + ('*' if e.name == E.DEFAULT else ''):8s} {e.title:42s} {e.finger_width_mm:5.1f}mm "
              f"{e.tip_below_tcp_mm:12.1f}mm  {'every task' if tasks is None else ', '.join(tasks)}")
    print("\n(* = the default)  force-bench run <task> --embodiment <name> [--seed N] [--set ...]")
    for e in E.EMBODIMENTS.values():
        extra = []
        if e.solver:
            extra.append("solver " + ", ".join(f"{k} {v:g}" for k, v in e.solver.items()))
        if e.collide_every_substep:
            extra.append("collision every substep")
        if e.checks:
            extra.append("adds the check " + ", ".join(e.checks))
        text = (f"{e.name}: {e.summary}. Compliance: {e.compliance}. Contact: {e.contact}."
                + (" " + "; ".join(extra).capitalize() + "." if extra else ""))
        print(textwrap.fill(text + (f" ({e.doc})" if e.doc else ""), 116, initial_indent="  ",
                            subsequent_indent="    "))


def _list_tasks() -> None:
    from force_bench.appendix_tasks.registry import APPENDIX
    last = None
    print(f"{'#':>3}  {'task':34s} {'name (force-bench run <name>)':42s} variant")
    for s in sorted(APPENDIX, key=lambda s: (s.number, s.name)):
        title = s.appendix_task if s.appendix_task != last else ""
        last = s.appendix_task
        flag = ("  [scene only: no scripted policy]" if not s.scripted else "") + \
            ("  [TB_GELSIGHT=1]" if s.gelsight else "")
        print(f"{s.number:>3}  {title:34s} {s.name:42s} {s.variant}{flag}")
    print("\nNot in simulation: tasks 1, 3, 7, 8, 10, 11, 12 and 21 (docs/tasks.md#tasks-not-included)")


def _info(name: str) -> None:
    import textwrap
    from force_bench.appendix_tasks.registry import BY_NAME, CRITERIA
    s = BY_NAME.get(name)
    if s is None:
        raise SystemExit(f"unknown task {name!r}; see `force-bench list`")
    rows = [("task", f"{s.number}. {s.appendix_task}" + (f" ({s.variant})" if s.variant else "")),
            ("skills", ", ".join(s.skills)), ("success", CRITERIA.get(s.number, "")), ("start", s.start),
            ("module", (s.dotted + " " + " ".join(s.argv)).strip()),
            ("built on", f"the scene builder of force_bench.tasks.{s.source}" if s.source != "new"
             else "a scene of its own"),
            ("policy", "scripted (the run is judged: appendix_qa.json)" if s.scripted else
             "none: a scene for teleoperation (force-bench teleop-ui --scene " + s.name + ") or a policy of "
             "your own; force-bench run records the scene with the arm holding still"),
            ("deviations", s.notes or "none documented"),
            ("tactile", "validated with TB_GELSIGHT=1" if s.gelsight else "bare gripper only"),
            ("embodiment", _embodiments_of(s)),
            ("outputs", f"outputs/appendix_tasks/{s.name}/ " + ("(verdict: appendix_qa.json)" if s.scripted
                                                                else "(scene.json: the fill and the criterion's state)"))]
    for k, v in rows:
        print(textwrap.fill(v, 108, initial_indent=f"{k:11s} ", subsequent_indent=" " * 12))
    from force_bench.appendix_tasks.variations import SEEDED, describe
    lines = describe(s.module, s.argv)
    for i, line in enumerate(lines or ["none in simulation yet"]):
        print(textwrap.fill(line, 108, initial_indent=f"{'variations' if i == 0 else '':11s} ",
                            subsequent_indent=" " * 14))
    if lines:
        print(" " * 12 + f'(* = this row; run another with: force-bench run {s.name} --set "<axis>=<value>")')
    seeded = (f"force-bench run {s.name} --seed N draws the start within the task's documented ranges"
              if s.module in SEEDED else "the nominal start only (no --seed yet)")
    print(textwrap.fill(seeded, 108, initial_indent=f"{'random':11s} ", subsequent_indent=" " * 12))


def _embodiments_of(spec) -> str:
    from force_bench import embodiments as E
    ok = E.supported(spec.module)
    if len(ok) == 1:
        return f"{E.DEFAULT} (the stock fingers) only"
    other = [n for n in ok if n != E.DEFAULT]
    return (f"{E.DEFAULT} (the default); " + "; ".join(f"{n} ({E.get(n).title})" for n in other)
            + f": force-bench run {spec.name} --embodiment {other[0]}")


def _variations(name: str | None) -> None:
    """Per paper task: the variation axes its website page lists, and those each simulated module supports."""
    import textwrap
    from force_bench.appendix_tasks.registry import APPENDIX, BY_NAME
    from force_bench.appendix_tasks.variations import SEEDED, axes_for, describe, website_task
    if name and name not in BY_NAME:
        raise SystemExit(f"unknown task {name!r}; see `force-bench list`")
    rows = [BY_NAME[name]] if name else APPENDIX
    fill = lambda text, first, rest: textwrap.fill(text, 116, initial_indent=first, subsequent_indent=rest)  # noqa
    numbers = sorted({s.number for s in rows} | (set() if name else set(range(1, 32))))
    for number in numbers:
        specs = [s for s in rows if s.number == number]
        page = website_task(number)
        if not specs:
            print(f"{number:>2}. {page.get('name', '')}  (not simulated)")
            continue
        print(f"{number:>2}. {specs[0].appendix_task}")
        site_axes = page.get("variations", [])
        if site_axes:
            text = "; ".join(f"{v['axis']} ({', '.join(i['label'] for i in v['instances']) or 'any number'})"
                             for v in site_axes)
            print(fill(text, "      website:  ", " " * 16))
        modules = list(dict.fromkeys(s.module for s in specs))
        sim_names = {a.name.lower() for m in modules for a in axes_for(m)}
        for m in modules:
            argv = BY_NAME[name].argv if name else ()
            for i, line in enumerate(describe(m, argv) or ["no variations"]):
                print(fill(line, f"      {m + ':' if i == 0 else '':s}", " " * 10).replace(
                    f"      {m}:", f"      sim ({m}): ", 1) if i == 0 else fill(line, " " * 10, " " * 12))
            if m in SEEDED:
                print(" " * 10 + "random start: --seed N")
        missing = [v["axis"] for v in site_axes if v["axis"].lower() not in sim_names]
        if missing:
            print(f"      not in simulation yet: {', '.join(dict.fromkeys(missing))}")
    if not name:
        print("\n(* = the module's default; force-bench info <row> marks the row's own choices)")


def _run_plan(name: str, spec, args: list[str]) -> tuple[list[str], str | None]:
    """The module arguments and the output folder (``TB_RUN_NAME``) of ``force-bench run <name> args``: --set, --seed
    and --embodiment applied to the row's own arguments, the rest passed through.  A run with --set, --seed or a
    non-default embodiment writes to ``<row>__<embodiment>_<settings>_seed<N>`` (the parts it uses), never to the
    row's own folder; --name chooses the folder."""
    argv = tuple(spec.argv)
    embodiment, args = _embodiment_option(args)
    sets, run_name, seed, extra = _run_options(args)
    from force_bench import embodiments
    from force_bench.appendix_tasks.variations import SEEDED, apply
    slugs = []
    if embodiment is not None:                # checked first: refused before anything is built
        try:
            embodiments.check(spec.module, embodiment)
        except (KeyError, ValueError) as e:
            raise SystemExit(f"force-bench run {name}: {e.args[0]}") from None
    if sets:
        try:
            argv, slugs = apply(spec.module, argv, sets)
        except KeyError as e:
            raise SystemExit(f"force-bench run {name}: {e.args[0]}") from None
    if seed is not None:
        if spec.module not in SEEDED:
            raise SystemExit(f"force-bench run {name}: this task has no randomized start yet (--seed)")
        argv += ("--seed", str(seed))
        slugs = [*slugs, f"seed{seed}"]
    if embodiment is not None and embodiment != embodiments.DEFAULT:
        argv += ("--embodiment", embodiment)
        slugs = [embodiment, *slugs]
    if slugs:
        run_name = run_name or f"{name}__{'_'.join(slugs)}"
    return [*argv, *extra], run_name


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return
    cmd, rest = argv[0], argv[1:]
    if cmd == "list":
        _list_tasks()
        return
    if cmd == "info":
        if not rest:
            raise SystemExit("force-bench info <task>")
        _info(rest[0])
        return
    if cmd == "variations":
        _variations(rest[0] if rest else None)
        return
    if cmd == "embodiments":
        _list_embodiments()
        return
    if cmd == "kit":
        from force_bench.kit.cli import main as kit_main
        raise SystemExit(kit_main(rest))
    if cmd == "run":
        if not rest:
            raise SystemExit("force-bench run <task> [args]")
        from force_bench.appendix_tasks.registry import BY_NAME as TASKS
        name, args = rest[0], rest[1:]
        spec = TASKS.get(name)
        if spec is None:
            raise SystemExit(f"unknown task {name!r}; see `force-bench list`")
        import os
        import runpy
        argv, run_name = _run_plan(name, spec, args)
        if run_name:
            os.environ["TB_RUN_NAME"] = run_name
        sys.argv = [spec.dotted, *argv]
        import warp as wp
        wp.init()
        runpy.run_module(spec.dotted, run_name="__main__", alter_sys=True)
        return
    table = {
        "run-all": ("force_bench.appendix_tasks.run_all", True),
        "gate": ("force_bench.teleop.gate", False),
        "teleop": ("force_bench.teleop.franka_teleop", False),
        "teleop-ui": ("force_bench.teleop.app", True),
        "guard-selftest": ("force_bench.teleop.guard", False),
        "collect": ("force_bench.data.collect", True),
        "dataset": ("force_bench.data.inspect", True),
    }
    if cmd not in table:
        raise SystemExit(f"force-bench: unknown command {cmd!r}; one of list, info, variations, embodiments, run, "
                         f"{', '.join(table)}")
    modname, pass_argv = table[cmd]
    if cmd == "guard-selftest":
        rest = ["--selftest", *rest]
    if cmd == "teleop":
        # franka_teleop builds its viewer under __main__ only
        import runpy
        sys.argv = [modname, *rest]
        runpy.run_module(modname, run_name="__main__", alter_sys=True)
        return
    _run_module(modname, rest, f"force-bench {cmd}", pass_argv)


if __name__ == "__main__":
    main()
