"""``hydrogel`` command line: the sensor's own benches and demos.

    hydrogel validate [...]      the solver-free sensor bench -> outputs/gelsight/validate/
    hydrogel demo [...]          press an indenter into a pad, save frame + video
    hydrogel render-selftest     Taxim renderer self-test
    hydrogel calibrate [...]     --sweep / --fit gel_kh against real ball presses
    hydrogel grip-rig [...]      two-jaw regression rig (5 asserted checks)
    hydrogel bench               kernel latency/throughput bench
    hydrogel housing [...]       install GelSight Inc.'s GS Mini housing CAD as the render mesh (visual only)

Every subcommand forwards its remaining arguments to the module's own parser;
``python -m hydrogel.<module>`` is equivalent.
"""

from __future__ import annotations

import sys


def _forward(modname: str, argv: list[str], prog: str):
    import importlib
    import runpy
    sys.argv = [prog, *argv]
    mod = importlib.import_module(modname)
    if hasattr(mod, "main"):
        rc = mod.main()
        raise SystemExit(0 if rc is None else rc)
    runpy.run_module(modname, run_name="__main__", alter_sys=True)


COMMANDS = {
    "validate": "hydrogel.validate",
    "demo": "hydrogel.sensor",
    "render-selftest": "hydrogel.render",
    "calibrate": "hydrogel.calibrate",
    "grip-rig": "hydrogel.grip_rig",
    "bench": "hydrogel.bench_parallel",
    "housing": "hydrogel.housing",
}


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return
    cmd, rest = argv[0], argv[1:]
    if cmd not in COMMANDS:
        raise SystemExit(f"hydrogel: unknown command {cmd!r}; one of {', '.join(COMMANDS)}")
    if cmd == "demo" and "--demo" not in rest:
        rest = ["--demo", *rest]
    _forward(COMMANDS[cmd], rest, f"hydrogel {cmd}")


if __name__ == "__main__":
    main()
