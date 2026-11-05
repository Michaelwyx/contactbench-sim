"""Export kit assets as USD stages for NVIDIA Newton (and, as plain UsdPhysics, for other simulators).

    force-bench kit export-usd                        # all variants -> outputs/kit_usd/<key>/<variant>.usda
    force-bench kit export-usd outlet door --out <dir>
    force-bench kit export-usd doorknob-module --variant "Handle 1" --param K=0.6 --param tau0=0.2
    force-bench kit export-usd cup --format usdc      # binary crate files (smaller, faster to load)

Each variant becomes one self-contained stage: the asset as ``force_bench.kit.add_asset`` builds it, with its bodies,
masses and inertias, joints, limits, spring drives, gear couplings, joint friction and armature, collision filters,
signed-distance-field and contact settings, friction and contact stiffness, standing on z = 0 and centred on the
origin (metres, kilograms, Z up).  ``--param`` overrides a mechanism parameter as ``add_asset(..., **params)`` does,
so a re-tuned asset can be saved and shared like any other.  What USD cannot say is stored under ``contactbench:``
and applied by ``force_bench.kit.usd.add_asset_usd`` (the format and what round-trips: ``force_bench.kit.usd``;
how to use a stage in your own scene: docs/custom_scenes.md).  Needs ``usd-core`` (the ``usd`` extra); adding a
stage to a Newton scene also needs ``newton-usd-schemas`` (``pip install "newton[importers]"``).
"""

from __future__ import annotations

import argparse
import ast
from pathlib import Path

from force_bench.paths import OUTPUTS_ROOT


def _value(text: str):
    try:
        return ast.literal_eval(text)
    except (ValueError, SyntaxError):
        return text


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="force-bench kit export-usd", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("keys", nargs="*", help="assets (default: all)")
    ap.add_argument("--out", default=None, help="output folder (default outputs/kit_usd)")
    ap.add_argument("--variant", default=None, help="one variant label (or a unique part of one), not all of them")
    ap.add_argument("--param", action="append", default=[], metavar="NAME=VALUE",
                    help="mechanism parameter override, as add_asset(..., NAME=VALUE)")
    ap.add_argument("--format", choices=("usda", "usdc"), default="usda")
    a = ap.parse_args(argv)
    from force_bench.kit.catalog import load_kit
    from force_bench.kit.check import slug
    from force_bench.kit.usd import export_variant
    kit = load_kit()
    out = Path(a.out) if a.out else OUTPUTS_ROOT / "kit_usd"
    params = {}
    for p in a.param:
        name, _, val = p.partition("=")
        params[name.strip()] = _value(val.strip())
    unknown = [k for k in a.keys if k not in kit]
    if unknown:
        raise SystemExit(f"unknown assets {unknown}; see `force-bench kit list`")
    n = 0
    for key in a.keys or list(kit):
        choices = kit[key].variant_choices()
        if a.variant:
            picked = kit[key].resolve(a.variant)
            choices = [{g: v.label for g, v in picked.items()
                        if v is not None and a.variant.lower() in v.label.lower()}]
        for choice in choices:
            path = export_variant(key, choice, out / key / f"{slug(choice)}.{a.format}", **params)
            n += 1
            print(path)
    print(f"[kit export-usd] {n} stages -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
