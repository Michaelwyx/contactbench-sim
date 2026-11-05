"""``force-bench kit``: the printable kit, sim-ready.

    force-bench kit list                  every asset: kind, variant groups, number of variants
    force-bench kit info <key>            its parts (size, estimated mass, colour), variants, physical model
    force-bench kit check [keys] [-j N]   build, settle and probe every variant -> outputs/kit/
    force-bench kit check --from-usd --compare [keys]
                                          the same checks on the assets imported from USD, compared with the
                                          Python-built ones -> outputs/usd_roundtrip/report.md
    force-bench kit import [--site ...]   re-import assets/kit/ from the project website
    force-bench kit export-usd [keys]     one self-contained USD stage per variant, joints and springs included,
                                          -> outputs/kit_usd/ (needs the usd extra; docs/custom_scenes.md)
"""

from __future__ import annotations

import sys


def _list() -> None:
    from force_bench.kit import load_kit
    kit = load_kit()
    print(f"{'key':26s} {'kind':10s} {'parts':>5s} {'variants':>8s}  groups")
    for key, a in kit.items():
        n = len(a.variant_choices()) if (a.groups or a.combinations) else 1
        groups = "; ".join(f"{g.variation} ({len(g.variants)})" for g in a.groups)
        print(f"{key:26s} {a.sim.get('kind', 'free'):10s} {len(a.parts()):5d} {n:8d}  {groups}")


def _info(key: str) -> None:
    from force_bench.kit import load_kit
    from force_bench.kit.build import part_mass
    kit = load_kit()
    if key not in kit:
        raise SystemExit(f"unknown asset {key!r}; see `force-bench kit list`")
    a = kit[key]
    print(f"{a.name} ({key})  kind={a.sim.get('kind', 'free')}"
          + (f" model={a.sim['model']}" if a.sim.get("model") else ""))
    if a.note:
        print(f"note: {a.note}")
    print(f"site: https://jingyi-xiang.github.io/contactbench/assets.html#{key}")
    print("parts (default variant):")
    for p in a.parts():
        m = p.mesh()
        e = " x ".join(f"{x * 1e3:.1f}" for x in m.extents)
        rgb = tuple(round(c * 255) for c in p.rgb)
        print(f"  {p.name:36s} {e:>24s} mm  {part_mass(a, p) * 1e3:7.2f} g  rgb{rgb}")
    for g in a.groups:
        slot = f"  [slot {g.slot}]" if g.slot else ""
        labels = ", ".join(v.label + (" *" if v.default else "") for v in g.variants)
        print(f"variation '{g.variation}'{slot}: {labels}")
    if a.combinations:
        print(f"{len(a.combinations)} combinations of the groups above, one file each")
    if a.sim:
        print(f"sim.yaml: {a.sim}")


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if not argv or argv[0] in ("-h", "--help"):
        print(__doc__)
        return 0
    cmd, rest = argv[0], argv[1:]
    if cmd == "list":
        _list()
        return 0
    if cmd == "info":
        if not rest:
            raise SystemExit("force-bench kit info <key>")
        _info(rest[0])
        return 0
    if cmd == "check":
        from force_bench.kit.check import main as check_main
        return check_main(rest)
    if cmd == "import":
        from force_bench.kit.importer import main as import_main
        return import_main(rest)
    if cmd == "export-usd":
        from force_bench.kit.export import main as export_main
        return export_main(rest)
    raise SystemExit(f"force-bench kit: unknown command {cmd!r}; one of list, info, check, import, export-usd")


if __name__ == "__main__":
    raise SystemExit(main())
