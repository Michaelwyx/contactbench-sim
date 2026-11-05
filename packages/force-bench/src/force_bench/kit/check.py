"""Check that every kit asset and variant is sim-ready.

    force-bench kit check                      # every asset, every variant -> outputs/kit/summary.json
    force-bench kit check outlet usb-slot -j 2
    force-bench kit check --summary            # only re-collect
    force-bench kit check --from-usd outlet    # the same checks on the asset imported from its USD stage
    force-bench kit check --from-usd --compare # ... and on the Python-built asset, then compare the two

``--from-usd --structure`` compares the models only, field by field (CPU, no simulation; structure.json).
``--from-usd`` exports every variant (``force-bench kit export-usd``, to ``--usd-dir``, default outputs/kit_usd;
``--no-export`` uses the stages already there), adds it with ``force_bench.kit.usd.add_asset_usd`` instead of
``add_asset`` and runs the same checks and probes; reports go to outputs/usd_roundtrip/usd/.  ``--compare`` also runs
the Python-built asset into outputs/usd_roundtrip/python/ and writes outputs/usd_roundtrip/report.json and
report.md: every probe's numbers side by side, within the tolerances of ``force_bench.kit.usd_check``.

For each variant the asset is built on a table and left alone (or, for a mechanism, run through its probe):

* ``settle``  free and assembled parts are dropped from 1 mm and must come to rest: finite state, no tunnelling
  through the table, speeds at the end below 5 mm/s and 0.5 rad/s;
* ``overlap`` parts of an assembly (a cap on its bottle) must not end up interpenetrating by more than 0.5 mm;
* ``probe``   a mechanism's own test (spring force against displacement, rotation to unlock, ...), see
  ``force_bench.kit.mechanisms``.

Each variant writes ``outputs/kit/<key>/<variant>/report.json`` and ``still.png`` (the scene at the end, in the
kit's colours).  Each asset runs in its own process.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from force_bench.paths import OUTPUTS_ROOT

KIT_OUT = OUTPUTS_ROOT / "kit"
SETTLE_S = 1.5
V_REST, W_REST = 0.005, 0.5           # m/s, rad/s at the end of the settle
TUNNEL_MM = 0.5                       # deepest allowed point below the table top
OVERLAP_MM = 0.5


def slug(choice: dict) -> str:
    if not choice:
        return "default"
    s = "__".join(v for v in choice.values() if v)
    return re.sub(r"[^A-Za-z0-9.]+", "_", s).strip("_") or "default"


def _camera(lo, hi):
    c = 0.5 * (lo + hi)
    r = max(float(np.linalg.norm(hi - lo)) * 0.5, 0.03)
    d = np.array([0.75, -0.95, 0.85])
    d /= np.linalg.norm(d)
    return c + d * r * 3.0, c


def usd_stage_path(key: str, choice: dict, usd_dir: Path | None = None) -> Path:
    return Path(usd_dir or OUTPUTS_ROOT / "kit_usd") / key / f"{slug(choice)}.usda"


def check_choice(key: str, choice: dict, out_root: Path = KIT_OUT, render: bool = True, source: str = "python",
                 usd_dir: Path | None = None) -> dict:
    """``source="usd"``: the asset comes from its USD stage (``usd_stage_path``) through ``add_asset_usd``."""
    import trimesh
    import warp as wp
    from force_bench.kit import add_asset, load_kit, place_on_table
    from force_bench.kit.build import part_mass
    from force_bench.rig import Rig

    kit = load_kit()
    asset = kit[key]
    kind = asset.sim.get("kind", "free")
    out = out_root / key / slug(choice)
    t0 = time.time()
    rig = Rig(f"kit_check_{key}")
    rig.out_dir = out
    rig.table(half=0.4)
    rig.collide_every_substep = True
    clearance = 0.0 if kind in ("fixture", "mechanism") else 0.001
    pose = place_on_table(key, choice, clearance=clearance)
    if source == "usd":
        from force_bench.kit.usd import add_asset_usd
        stage = usd_stage_path(key, choice, usd_dir)
        h = add_asset_usd(rig, stage, pose=pose)
    else:
        h = add_asset(rig, key, choice, pose=pose)
    mech = None
    if kind == "mechanism":
        from force_bench.kit.mechanisms import module_for
        mech = module_for(asset)
        if hasattr(mech, "add_probe"):
            mech.add_probe(rig, h)
    parts = asset.parts(choice)
    world = trimesh.util.concatenate([p.mesh().apply_transform(pose) for p in parts])
    if not h.bodies:
        # a fixture alone has no joint (MuJoCo needs one) and nothing touching it: drop a 10 mm PLA cube onto
        # the middle of its top, which also exercises its collision surface
        lo, hi = world.bounds
        c = 0.5 * (lo + hi)
        cube = rig.builder.add_body(xform=wp.transform(wp.vec3(c[0], c[1], hi[2] + 0.010), wp.quat_identity()),
                                    label="check_cube")
        rig.builder.add_shape_box(cube, hx=0.005, hy=0.005, hz=0.005, cfg=rig.cfg(mu=0.5), label="check_cube")
        h.bodies["check_cube"] = cube
    cam_pos, cam_target = _camera(*world.bounds)
    rig.finalize(probes=list(h.info.get("probe_bodies", [])), cam_pos=cam_pos, cam_target=cam_target,
                 record=render)
    report = {"key": key, "name": asset.name, "kind": kind, "choice": h.choice, "parts": [], "checks": {},
              "source": source}
    if source == "usd":
        report["stage"] = str(stage)
        report["usd_import"] = dict(h.import_report)
    for p in parts:
        m = p.mesh()
        report["parts"].append({"name": p.name, "file": str(p.file.relative_to(p.file.parents[1])),
                                "extent_mm": [round(float(x) * 1e3, 2) for x in m.extents],
                                "volume_cm3": round(float(m.volume) * 1e6, 3),
                                "mass_g": round(part_mass(asset, p) * 1e3, 2),
                                "watertight": bool(m.is_watertight)})
    if mech is not None and hasattr(mech, "run_probe"):
        report["probe"] = mech.run_probe(rig, h)
        report["params"] = h.params
        report["info"] = h.info
    q0 = rig.s0.body_q.numpy().copy()
    rig.run(SETTLE_S, lambda t: None, print_every=10 ** 9)
    q1, qd1 = rig.s0.body_q.numpy(), rig.s0.body_qd.numpy()
    labels = list(rig.model.body_label)
    finite = bool(np.isfinite(q1).all() and np.isfinite(qd1).all())
    moving = {}
    for role, b in h.bodies.items():
        v, w = float(np.linalg.norm(qd1[b][:3])), float(np.linalg.norm(qd1[b][3:]))
        dz = float(q1[b][2] - q0[b][2])
        moving[labels[b]] = {"v_mm_s": round(v * 1e3, 2), "w_rad_s": round(w, 3), "dz_mm": round(dz * 1e3, 2),
                             "dxy_mm": round(float(np.linalg.norm(q1[b][:2] - q0[b][:2])) * 1e3, 2)}
    # deepest point of every part below the table, in the final pose
    shape_body = rig.model.shape_body.numpy()
    shape_tf = rig.model.shape_transform.numpy()
    shape_lbl = list(rig.model.shape_label)
    lowest_mm, finals = 1e9, {}
    for sid, lbl in enumerate(shape_lbl):
        if not (lbl or "").startswith("kit/"):
            continue
        name = lbl.split("/")[-1]
        part = next((p for p in parts if p.name == name), None)
        if part is None:
            continue
        b = int(shape_body[sid])
        Tb = np.eye(4)
        if b >= 0:
            Tb = _tf(q1[b])
        Ts = _tf(shape_tf[sid])
        m = part.mesh()
        lo, hi = m.bounds
        m.apply_translation(-0.5 * (lo + hi))               # make_newton_mesh centres at the AABB centre
        m.apply_transform(Tb @ Ts)
        finals[name] = m
        lowest_mm = min(lowest_mm, float(m.vertices[:, 2].min()) * 1e3)
    report["checks"]["finite"] = {"pass": finite}
    report["checks"]["no_tunnelling"] = {"pass": bool(lowest_mm > -TUNNEL_MM), "lowest_point_mm": round(lowest_mm, 3)}
    if moving:
        still = all(m["v_mm_s"] < V_REST * 1e3 and m["w_rad_s"] < W_REST for m in moving.values())
        report["checks"]["comes_to_rest"] = {"pass": bool(still and finite), "bodies": moving}
    if kind == "assembly" and len(finals) > 1:
        worst = 0.0
        names = list(finals)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                pts = finals[b].sample(4000)
                sd = trimesh.proximity.signed_distance(finals[a], pts)      # > 0 inside a
                worst = max(worst, float(sd.max()) * 1e3)
        report["checks"]["no_overlap"] = {"pass": bool(worst < OVERLAP_MM), "deepest_mm": round(worst, 3)}
    if "probe" in report:
        report["checks"]["probe"] = {"pass": bool(report["probe"].get("pass", False))}
    report["pass"] = all(c["pass"] for c in report["checks"].values())
    report["wall_s"] = round(time.time() - t0, 1)
    out.mkdir(parents=True, exist_ok=True)
    if render and rig.rec is not None and len(rig.rec.frames):
        from PIL import Image
        Image.fromarray(rig.rec.frames[len(rig.rec.frames) - 1]).save(out / "still.png")
        for f in out.glob("*.mp4"):
            f.unlink()
    (out / "report.json").write_text(json.dumps(report, indent=1))
    print(f"[kit check] {key} {slug(choice)}: {'PASS' if report['pass'] else 'FAIL'} "
          f"{[k for k, c in report['checks'].items() if not c['pass']]}", flush=True)
    wp.synchronize()
    return report


def _tf(row) -> np.ndarray:
    import trimesh
    T = trimesh.transformations.quaternion_matrix([row[6], row[3], row[4], row[5]])
    T[:3, 3] = row[:3]
    return T


def check_asset(key: str, render: bool = True, source: str = "python", out_root: Path = KIT_OUT,
                usd_dir: Path | None = None, export: bool = True) -> list[dict]:
    from force_bench.kit import load_kit
    asset = load_kit()[key]
    if source == "usd" and export:
        from force_bench.kit.usd import export_variant
        for ch in asset.variant_choices():
            export_variant(key, ch, usd_stage_path(key, ch, usd_dir))
    return [check_choice(key, ch, out_root=out_root, render=render, source=source, usd_dir=usd_dir)
            for ch in asset.variant_choices()]


def collect(out_root: Path = KIT_OUT) -> dict:
    rows = {}
    for p in sorted(out_root.glob("*/*/report.json")):
        r = json.loads(p.read_text())
        rows.setdefault(r["key"], {})[p.parent.name] = {
            "pass": r["pass"], "failed": [k for k, c in r["checks"].items() if not c["pass"]]}
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / "summary.json").write_text(json.dumps(rows, indent=1))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="force-bench kit check", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("keys", nargs="*")
    ap.add_argument("-j", "--jobs", type=int, default=3)
    ap.add_argument("--summary", action="store_true")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--from-usd", action="store_true", help="import each variant from its USD stage")
    ap.add_argument("--compare", action="store_true",
                    help="with --from-usd: also check the Python-built asset and write the comparison report")
    ap.add_argument("--usd-dir", default=None, help="stages (default outputs/kit_usd)")
    ap.add_argument("--no-export", action="store_true", help="use the stages in --usd-dir as they are")
    ap.add_argument("--structure", action="store_true",
                    help="with --from-usd: compare the models field by field (CPU, no simulation) and stop")
    ap.add_argument("--out", default=None, help="report folder (default outputs/kit, outputs/usd_roundtrip/usd); with "
                    "--compare the round-trip folder that gets usd/, python/ and the report (default outputs/usd_roundtrip)")
    ap.add_argument("--_one", help=argparse.SUPPRESS)
    ap.add_argument("--_source", default="python", help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    from force_bench.kit.usd_check import ROUNDTRIP_OUT
    usd_dir = Path(a.usd_dir) if a.usd_dir else None
    if a._one:
        import warp as wp
        wp.init()
        out_root = Path(a.out) if a.out else (ROUNDTRIP_OUT / "usd" if a._source == "usd" else KIT_OUT)
        check_asset(a._one, render=not a.no_render, source=a._source, out_root=out_root, usd_dir=usd_dir,
                    export=not a.no_export)
        return 0
    if a.from_usd and a.structure:
        from force_bench.kit.usd_check import structure
        st = structure(a.keys or None, usd_dir=usd_dir, export=not a.no_export)
        print(f"[kit check --from-usd --structure] {st['identical_with_helper']}/{st['variants']} variants identical "
              f"after add_asset_usd, {st['identical_with_add_usd_alone']} after Newton's add_usd alone "
              f"-> {ROUNDTRIP_OUT / 'structure.json'}")
        return 0 if st["identical_with_helper"] == st["variants"] else 1
    rt = Path(a.out) if a.out else ROUNDTRIP_OUT
    if a.from_usd and a.compare:
        runs = [("usd", rt / "usd"), ("python", rt / "python")]
    else:
        runs = [("usd" if a.from_usd else "python",
                 Path(a.out) if a.out else (ROUNDTRIP_OUT / "usd" if a.from_usd else KIT_OUT))]
    if not a.summary:
        from force_bench.kit import load_kit
        todo = [(k, src, root) for k in (a.keys or list(load_kit().keys())) for src, root in runs]
        running = []
        while todo or running:
            while todo and len(running) < a.jobs:
                k, src, root = todo.pop(0)
                logs = root / "_logs"
                logs.mkdir(parents=True, exist_ok=True)
                f = open(logs / f"{k}.log", "w")
                cmd = [sys.executable, "-u", "-m", "force_bench.kit.check", "--_one", k, "--_source", src,
                       "--out", str(root)]
                if a.no_render:
                    cmd.append("--no-render")
                if usd_dir:
                    cmd += ["--usd-dir", str(usd_dir)]
                if a.no_export:
                    cmd.append("--no-export")
                running.append((subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT), k, src, time.time()))
            for job in list(running):
                p, k, src, t0 = job
                if p.poll() is not None:
                    running.remove(job)
                    print(f"finished {k} [{src}] (rc {p.returncode}, {time.time() - t0:.0f} s)", flush=True)
            time.sleep(1)
    if a.from_usd and a.compare:
        from force_bench.kit.usd_check import compare
        rep = compare(a.keys or None, root=rt)
        print(f"[kit check --from-usd] {rep['summary']['variants_pass']}/{rep['summary']['variants']} variants match "
              f"-> {rt / 'report.md'}")
    rows = collect(runs[0][1])
    n = sum(len(v) for v in rows.values())
    bad = [(k, s, r["failed"]) for k, v in rows.items() for s, r in v.items() if not r["pass"]]
    print(f"[kit check] {n - len(bad)}/{n} variants pass")
    for k, s, f in bad:
        print(f"  FAIL {k} {s}: {f}")
    return 0 if not bad else 1


if __name__ == "__main__":
    raise SystemExit(main())
