"""Compare ``force-bench kit check`` on the Python-built kit with the same check on the assets imported from USD.

    force-bench kit check --from-usd --compare          # run both, then compare
    force-bench kit check --from-usd --compare --summary  # compare existing reports only
    force-bench kit check --from-usd --structure        # the models only (CPU, no simulation): structure.json

Reads outputs/usd_roundtrip/python/<key>/<variant>/report.json (``add_asset``) and outputs/usd_roundtrip/usd/...
(``add_asset_usd``), writes outputs/usd_roundtrip/report.json and report.md.  Repeated runs, if any, are read too:
outputs/usd_roundtrip/python_repeat*/ and usd_repeat*/ (``kit check [--from-usd --no-export] --out <that folder>``).
The probes run on a GPU whose contact generation is not bitwise reproducible, so two runs of the same model differ
where contact sticks and slips; the repeats measure that spread.

A mechanism matches when both probes pass and

* every CALIBRATION number -- a spring rate, a preload, a friction torque, a sliding or plateau force averaged over
  a depth window, a stop, a gear ratio, a detent -- agrees within

      |mean(USD runs) - mean(Python runs)| <= max(REL[unit] x |mean(Python runs)|, FLOOR[unit])

  Forces, torques and spring rates get 5 %; gear ratios 2 %; angles and travel 1 %.  The floors are the
  resolution at which a number means something for its probe: 0.5 N for forces (the outlet and USB slot probes
  push with 10-150 N), 0.005 N.m for torques (the lock's dry friction is 0.005-0.02 N.m, the doorknob spring
  0.15-0.8 N.m), 0.05 mm for travel, 0.5 deg for angles (the door's locked-leaf check allows 0.6 deg);
* every TRANSIENT number -- a peak or a force sampled at one depth on one frame (60 Hz) of a 1920 Hz simulation,
  e.g. the force spike when a charger blade wedges the outlet's pad open -- agrees in the mean within the same
  tolerance or within three standard errors of the difference of the means, whichever is larger, the standard
  error coming from the run-to-run spread of both (pooled within-group standard deviation; a two-sample test that
  holds for identical models about 99 times in 100).  With a single run of each, the tolerance alone.

A free, fixture or assembly variant matches when every settle check passes on both and the end state agrees (lowest
point within 0.1 mm, speeds within 1 mm/s).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from force_bench.paths import OUTPUTS_ROOT

ROUNDTRIP_OUT = OUTPUTS_ROOT / "usd_roundtrip"
REL = {"N": 0.05, "Nm": 0.05, "Nm/rad": 0.05, "mm/rad": 0.02, "mm": 0.01, "deg": 0.01}
FLOOR = {"N": 0.5, "Nm": 0.005, "Nm/rad": 0.005, "mm/rad": 0.05, "mm": 0.05, "deg": 0.5}
CAL, TRANSIENT = "calibration", "transient"

METRICS = {   # probe -> [(path in report["probe"] ("a/b") or "trace:<name>", unit, class)]
    "outlet": [("trace:force_6_8mm", "N", CAL), ("trace:force_10_18mm", "N", CAL), ("depth_reached_mm", "mm", CAL),
               ("peak_force_N", "N", TRANSIENT), ("force_at_depth_N/10mm", "N", TRANSIENT),
               ("force_at_depth_N/15mm", "N", TRANSIENT)],
    "usb_slot": [("sliding_force_N", "N", CAL), ("jaw_drive_force_seated_N/0", "N", CAL),
                 ("jaw_drive_force_seated_N/1", "N", CAL), ("tip_depth_end_mm", "mm", CAL),
                 ("upside_down/force_N", "N", CAL), ("upside_down/tip_depth_mm", "mm", CAL),
                 ("peak_insertion_force_N", "N", TRANSIENT), ("force_at_depth_N/4mm", "N", TRANSIENT),
                 ("force_at_depth_N/8mm", "N", TRANSIENT)],
    "dial/standard": [("breakaway_torque_Nm", "Nm", CAL), ("turning_torque_median_Nm", "Nm", CAL),
                      ("turning_torque_mean_Nm", "Nm", CAL), ("sweep_deg", "deg", CAL),
                      ("drift_after_release_deg", "deg", CAL)],
    "dial/snap": [("breakaway_torque_Nm", "Nm", CAL), ("peak_turning_torque_Nm", "Nm", CAL),
                  ("turning_torque_mean_Nm", "Nm", CAL), ("detent_spacing_deg", "deg", CAL),
                  ("released_20deg_off_detent_ends_at_deg", "deg", CAL), ("sweep_deg", "deg", CAL)],
    # (the press force 1 mm down, a single sample early in the press, came out 7.23 and 6.46 N in two runs of one
    # model; 2 and 3 mm down varied by at most 0.32 N over four runs)
    "dial/push": [("press_force_N/1.00mm", "N", TRANSIENT), ("press_force_N/2.00mm", "N", CAL),
                  ("press_force_N/3.00mm", "N", CAL),
                  ("pressed_turning_torque_Nm/0", "Nm", CAL), ("unpressed_try/rotation_deg", "deg", CAL),
                  ("pressed_turn_deg", "deg", CAL), ("released_at_90/press_mm", "mm", CAL),
                  ("released_at_90/angle_deg", "deg", CAL), ("returned_to_0/press_mm", "mm", CAL)],
    "doorknob_module": [("K_fit_Nm_per_rad", "Nm/rad", CAL), ("tau0_fit_Nm", "Nm", CAL),
                        ("torque_at_stop_Nm", "Nm", CAL), ("stop_deg", "deg", CAL),
                        ("latch_mm_per_rad", "mm/rad", CAL), ("latch_clears_jamb_at_deg", "deg", CAL)],
    "lock_module": [("friction_torque_Nm", "Nm", CAL), ("torque_opening_Nm", "Nm", CAL),
                    ("torque_closing_Nm", "Nm", CAL), ("stop_deg", "deg", CAL), ("deadbolt_mm_per_rad", "mm/rad", CAL),
                    ("deadbolt_clears_jamb_at_deg", "deg", CAL)],
    "door": [("locked_both_bolts_deg", "deg", CAL), ("locked_deadbolt_only_deg", "deg", CAL),
             ("opened_unlocked_deg", "deg", CAL), ("shut_after_release_deg", "deg", CAL),
             ("relocked_deg", "deg", CAL), ("latch_retracted_at_open_mm", "mm", CAL),
             ("deadbolt_retracted_at_open_mm", "mm", CAL)],
    "ethernet": [("trace:insert_force_6_12mm", "N", CAL), ("trace:hold_force", "N", CAL),
                 ("depth_seated_mm", "mm", CAL), ("tab/max_deg_on_the_way_in", "deg", CAL),
                 ("tab/seated_deg", "deg", CAL), ("light_pull/rise_mm", "mm", CAL),
                 ("strong_pull/held_up_to_N", "N", CAL), ("strong_pull/rise_mm", "mm", CAL),
                 ("press/tab_deg", "deg", CAL), ("reinsert/tab_max_deg", "deg", CAL), ("reinsert/depth_mm", "mm", CAL),
                 ("reinsert/tab_deg", "deg", CAL), ("seated_plug/backed_off_mm", "mm", CAL),
                 ("seated_plug/tab_deg", "deg", CAL),
                 ("peak_insertion_force_N", "N", TRANSIENT), ("force_at_depth_N/8mm", "N", TRANSIENT),
                 ("force_at_depth_N/12mm", "N", TRANSIENT), ("pressed_pull/resistance_peak_N", "N", TRANSIENT)],
}
TRACES = {    # probe -> (csv file, column of the main force/torque reading)
    "outlet": ("probe_force.csv", "force_N"), "usb_slot": ("probe_force.csv", "force_N"),
    "dial/standard": ("probe_trace.csv", "tau"), "dial/snap": ("probe_trace.csv", "tau"),
    "dial/push": ("probe_trace.csv", "fz"), "doorknob_module": ("probe_spindle.csv", "torque_Nm"),
    "lock_module": ("probe_plug.csv", "torque_Nm"), "door": ("probe_door.csv", "leaf_deg"),
    "ethernet": ("probe_force.csv", "slot_force_out_N"),
}


def _probe_kind(rep: dict) -> str:
    from force_bench.kit import load_kit
    model = load_kit()[rep["key"]].sim.get("model")
    if model == "dial":
        return f"dial/{rep['probe'].get('type')}"
    return model


def _get(d, path):
    """A number in a nested report: ``path`` is ``key/key/...`` (keys may contain dots, as ``1.00mm``)."""
    cur = d
    for part in path.split("/"):
        if isinstance(cur, list):
            cur = cur[int(part)]
        elif isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return None
    return float(cur) if isinstance(cur, (int, float)) and not isinstance(cur, bool) else None


def _tol(p, unit):
    return max(REL[unit] * abs(p), FLOOR[unit])


def _read_csv(f: Path) -> dict | None:
    import csv
    if not f.exists():
        return None
    with open(f) as fh:
        rows = list(csv.reader(fh))
    hdr = [h.split("(")[0] for h in rows[0]]
    cols = {}
    for i, h in enumerate(hdr):
        try:
            cols[h] = np.array([float(r[i]) for r in rows[1:] if len(r) > i])
        except ValueError:
            pass
    return cols


def _trace_metrics(kind: str, run_dir: Path) -> dict:
    """Window averages of a probe's force trace (robust where single samples are not)."""
    out = {}
    if kind == "outlet":
        c = _read_csv(run_dir / "probe_force.csv")
        if c:
            d, f = c["depth_mm"], np.abs(c["force_N"])
            for name, lo, hi in (("force_6_8mm", 6.0, 8.0), ("force_10_18mm", 10.0, 18.0)):
                m = (d >= lo) & (d < hi)
                out[name] = float(f[m].mean()) if m.any() else None
    elif kind == "ethernet":
        c = _read_csv(run_dir / "probe_force.csv")
        if c:
            ph, d, f = c["phase"], c["nose_depth_mm"], c["slot_force_out_N"]
            m = (ph == 0) & (d >= 6.0) & (d < 12.0)       # inserting: the latch tab rides up the insert's lip
            out["insert_force_6_12mm"] = float(f[m].mean()) if m.any() else None
            m = ph == 1                                   # seated, the drive holding the plug against the back wall
            out["hold_force"] = float(f[m].mean()) if m.any() else None
    return out


def _value(rep: dict, run_dir: Path, kind: str, path: str):
    if path.startswith("trace:"):
        return _trace_metrics(kind, run_dir).get(path[6:])
    return _get(rep["probe"], path)


def _trace_nrmse(dir_a: Path, dir_b: Path, fname: str, col: str):
    """RMS difference of a probe trace (same time base) over the range of the first trace."""
    ca, cb = _read_csv(dir_a / fname), _read_csv(dir_b / fname)
    if not ca or not cb or col not in ca or col not in cb or not len(ca[col]) or not len(cb[col]):
        return None
    a, b = ca[col], cb[col]
    n = min(len(a), len(b))
    rng = float(np.ptp(a[:n])) or 1.0
    return round(float(np.sqrt(np.mean((a[:n] - b[:n]) ** 2)) / rng), 4)


def _compare_mech(kind, prs, urs):
    """prs / urs: [(report, run dir)] of the Python and the USD runs."""
    rows = []
    ok = all(bool(r["probe"].get("pass")) for r, _ in prs + urs)
    for path, unit, cls in METRICS.get(kind, []):
        pv = [v for v in (_value(r, d, kind, path) for r, d in prs) if v is not None]
        uv = [v for v in (_value(r, d, kind, path) for r, d in urs) if v is not None]
        if not pv and not uv:
            continue
        good = bool(pv and uv)
        pm, um = (float(np.mean(pv)) if pv else None), (float(np.mean(uv)) if uv else None)
        tol = _tol(pm, unit) if pv else None
        noise = None
        if good and cls == TRANSIENT and len(pv) + len(uv) > 2:
            ss = sum((v - pm) ** 2 for v in pv) + sum((v - um) ** 2 for v in uv)
            sp = float(np.sqrt(ss / (len(pv) + len(uv) - 2)))
            noise = 3.0 * sp * float(np.sqrt(1.0 / len(pv) + 1.0 / len(uv)))
        if good:
            good = abs(um - pm) <= max(tol, noise or 0.0)
        ok &= good
        rows.append({"metric": path, "unit": unit, "class": cls, "python": pv, "usd": uv,
                     "diff": None if pm is None or um is None else round(um - pm, 6),
                     "rel": None if pm is None or um is None or pm == 0 else round((um - pm) / abs(pm), 4),
                     "tol": None if tol is None else round(tol, 6),
                     "noise_3se": None if noise is None else round(noise, 6),
                     "python_spread": round(max(pv) - min(pv), 6) if len(pv) > 1 else None,
                     "usd_spread": round(max(uv) - min(uv), 6) if len(uv) > 1 else None, "match": bool(good)})
    return ok, rows


def _compare_settle(p, u):
    rows, ok = [], p["pass"] and u["pass"]
    for name in sorted(set(p["checks"]) | set(u["checks"])):
        cp, cu = p["checks"].get(name, {}), u["checks"].get(name, {})
        rows.append({"check": name, "python": cp.get("pass"), "usd": cu.get("pass")})
        ok &= bool(cp.get("pass")) == bool(cu.get("pass"))
    lp = p["checks"].get("no_tunnelling", {}).get("lowest_point_mm")
    lu = u["checks"].get("no_tunnelling", {}).get("lowest_point_mm")
    if lp is not None and lu is not None:
        good = abs(lu - lp) <= 0.1
        ok &= good
        rows.append({"metric": "lowest_point_mm", "python": lp, "usd": lu, "match": good})
    bp = p["checks"].get("comes_to_rest", {}).get("bodies", {})
    bu = u["checks"].get("comes_to_rest", {}).get("bodies", {})
    for body in sorted(set(bp) & set(bu)):
        good = abs(bu[body]["v_mm_s"] - bp[body]["v_mm_s"]) <= 1.0
        ok &= good
        rows.append({"metric": f"{body} end speed mm/s", "python": bp[body]["v_mm_s"], "usd": bu[body]["v_mm_s"],
                     "match": good})
    return ok, rows


def _runs(root: Path, prefix: str) -> list[Path]:
    return sorted(d for d in root.iterdir()
                  if d.is_dir() and (d.name == prefix or d.name.startswith(prefix + "_repeat")))


def compare(keys=None, root: Path = ROUNDTRIP_OUT) -> dict:
    py_runs, us_runs = _runs(root, "python"), _runs(root, "usd")
    out = {"tolerance": {"rel": REL, "floor": FLOOR}, "runs": {"python": [d.name for d in py_runs],
                                                              "usd": [d.name for d in us_runs]},
           "variants": {}, "summary": {}}
    n = n_ok = 0
    by_kind = {}
    for f in sorted((root / "usd").glob("*/*/report.json")):
        key, var = f.parent.parent.name, f.parent.name
        if keys and key not in keys:
            continue
        prs = [(json.loads((d / key / var / "report.json").read_text()), d / key / var) for d in py_runs
               if (d / key / var / "report.json").exists()]
        urs = [(json.loads((d / key / var / "report.json").read_text()), d / key / var) for d in us_runs
               if (d / key / var / "report.json").exists()]
        if not prs:
            continue
        p, u = prs[0][0], urs[0][0]
        entry = {"key": key, "variant": var, "kind": p["kind"], "python_pass": all(r["pass"] for r, _ in prs),
                 "usd_pass": all(r["pass"] for r, _ in urs), "runs": {"python": len(prs), "usd": len(urs)},
                 "usd_import": u.get("usd_import", {}),
                 "failed_checks": {side: sorted({c for r, _ in runs for c, v in r["checks"].items() if not v["pass"]})
                                   for side, runs in (("python", prs), ("usd", urs))},
                 "wall_s": {"python": p.get("wall_s"), "usd": u.get("wall_s")}}
        if "probe" in p and "probe" in u:
            kind = _probe_kind(p)
            ok, rows = _compare_mech(kind, prs, urs)
            entry.update(probe=kind, metrics=rows)
            tr = TRACES.get(kind)
            if tr:
                entry["trace_nrmse"] = _trace_nrmse(prs[0][1], urs[0][1], *tr)
                if len(prs) > 1:
                    entry["trace_nrmse_python_repeat"] = _trace_nrmse(prs[0][1], prs[1][1], *tr)
        else:
            ok, rows = _compare_settle(p, u)
            entry.update(checks=rows)
        entry["match"] = bool(ok)
        out["variants"][f"{key}/{var}"] = entry
        n += 1
        n_ok += bool(ok)
        k = entry.get("probe") or p["kind"]
        s = by_kind.setdefault(k, [0, 0])
        s[0] += 1
        s[1] += bool(ok)
    out["summary"] = {"variants": n, "variants_pass": n_ok, "by_kind": {k: {"variants": v[0], "match": v[1]}
                                                                        for k, v in by_kind.items()}}
    if (root / "structure.json").exists():
        st = json.loads((root / "structure.json").read_text())
        out["structure"] = {k: v for k, v in st.items() if k != "rows"}
    root.mkdir(parents=True, exist_ok=True)
    (root / "report.json").write_text(json.dumps(out, indent=1))
    (root / "report.md").write_text(markdown(out))
    return out


def _fmt(v, nd=4):
    if v is None:
        return "-"
    if isinstance(v, bool):
        return "yes" if v else "NO"
    return f"{v:.{nd}g}"


def markdown(rep: dict) -> str:
    s = rep["summary"]
    lines = ["# USD round trip of the ContactBench kit", "",
             f"{s['variants_pass']} of {s['variants']} variants match: the asset imported from its USD stage "
             f"(`force_bench.kit.usd.add_asset_usd`) passes the same `force-bench kit check` as the Python-built "
             f"asset (`add_asset`), with the same numbers within max(a fraction of the Python value, a floor): "
             + ", ".join(f"{k}: {REL[k]:.0%} or {v}" for k, v in FLOOR.items()) + ".", "",
             "| kind | variants | match |", "|---|---|---|"]
    for k, v in sorted(s["by_kind"].items()):
        lines.append(f"| {k} | {v['variants']} | {v['match']} |")
    bad = [(n, e) for n, e in rep["variants"].items() if not e["match"]]
    if bad:
        lines += ["", "Not matching:", ""]
        for n, e in bad:
            fp, fu = e.get("failed_checks", {}).get("python", []), e.get("failed_checks", {}).get("usd", [])
            same = " -- both builds fail the same checks" if fp and fp == fu else ""
            lines.append(f"- `{n}`: Python {'pass' if e['python_pass'] else 'FAIL ' + str(fp)}, "
                         f"USD {'pass' if e['usd_pass'] else 'FAIL ' + str(fu)}{same}")
    st = rep.get("structure")
    if st:
        lines += ["", "## Structure: the model the stage gives", "",
                  f"Field by field against the builder `add_asset` fills, {st['variants']} variants: identical "
                  f"after `add_asset_usd` in {st['identical_with_helper']}, after Newton's `add_usd` alone in "
                  f"{st['identical_with_add_usd_alone']}. Variants where `add_usd` alone differs, by field:", "",
                  "| field | add_usd alone | add_asset_usd |", "|---|---|---|"]
        tp, th = st["variants_with_difference"]["plain"], st["variants_with_difference"]["helper"]
        for k in sorted(set(tp) | set(th)):
            lines.append(f"| {k} | {tp.get(k, 0)} | {th.get(k, 0)} |")
    lines += ["", "## Mechanisms: probe numbers, Python-built vs USD-imported", "",
              f"Runs: Python {', '.join(rep['runs']['python'])}; USD {', '.join(rep['runs']['usd'])}. "
              "`cal` numbers must agree in the mean within `tol`; `trans` numbers (peaks, single-frame samples) "
              "within `tol` or three standard errors of the difference of the means (`3 se`, from the pooled "
              "run-to-run spread), whichever is larger. `trace`: RMS difference of the probe's main force/torque "
              "trace over its range, Python vs USD (and Python vs Python).", "",
              "| asset / variant | metric | class | Python | USD | diff | tol | 3 se | ok |",
              "|---|---|---|---|---|---|---|---|---|"]
    for name, e in rep["variants"].items():
        if "metrics" not in e:
            continue
        first = True
        for m in e["metrics"]:
            pv = " / ".join(_fmt(v) for v in m["python"]) or "-"
            uv = " / ".join(_fmt(v) for v in m["usd"]) or "-"
            lines.append(f"| {name if first else ''} | {m['metric']} [{m['unit']}] | {m['class'][:5]} | {pv} | {uv} | "
                         f"{_fmt(m['diff'], 3)} | {_fmt(m['tol'], 3)} | {_fmt(m.get('noise_3se'), 3)} | "
                         f"{'yes' if m['match'] else 'NO'} |")
            first = False
        lines.append(f"|  | probes pass (Python / USD); trace (Python-USD / Python-Python) |  | "
                     f"{_fmt(e['python_pass'])} | {_fmt(e['usd_pass'])} | {_fmt(e.get('trace_nrmse'), 3)} | "
                     f"{_fmt(e.get('trace_nrmse_python_repeat'), 3)} |  | {'yes' if e['match'] else 'NO'} |")
    lines += ["", "## Free, fixture and assembly assets: settle check", "",
              "| asset / variant | kind | Python | USD | lowest point mm (Python / USD) | match |",
              "|---|---|---|---|---|---|"]
    for name, e in rep["variants"].items():
        if "checks" not in e:
            continue
        low = next((c for c in e["checks"] if c.get("metric") == "lowest_point_mm"), None)
        lp = f"{_fmt(low['python'])} / {_fmt(low['usd'])}" if low else "-"
        lines.append(f"| {name} | {e['kind']} | {'pass' if e['python_pass'] else 'FAIL'} | "
                     f"{'pass' if e['usd_pass'] else 'FAIL'} | {lp} | {'yes' if e['match'] else 'NO'} |")
    return "\n".join(lines) + "\n"


# ── structure: the builder add_asset fills against the one the stage gives ────────────────────────────────────────
STRUCTURE_FIELDS = {   # what is compared, and how
    "body": ("body_mass", "body_inv_mass", "body_com", "body_inertia", "body_inv_inertia", "body_q"),
    "dof": ("joint_target_ke", "joint_target_kd", "joint_target_mode", "joint_target_qd", "joint_limit_lower",
            "joint_limit_upper", "joint_limit_ke", "joint_limit_kd", "joint_armature", "joint_friction",
            "joint_damping", "joint_effort_limit", "joint_velocity_limit", "joint_f"),
    "shape": ("shape_material_mu", "shape_material_restitution", "shape_material_mu_torsional",
              "shape_material_mu_rolling", "shape_margin", "shape_gap", "shape_collision_group"),
}


def _close(a, b, rtol=2e-6, atol=1e-9) -> bool:
    a, b = np.asarray(a, float), np.asarray(b, float)
    return a.shape == b.shape and bool(np.allclose(a, b, rtol=rtol, atol=atol))


def diff_builders(A, na, B, nb) -> dict[str, list]:
    """Differences between two builders' contents after the counts ``na`` / ``nb`` = (bodies, shapes, joints),
    matched by label (bodies, joints, shapes; a free joint by its body's), articulations by their members, in world
    frame where frames may legitimately differ (a joint's frame is rotated when its axis is not +X/+Y/+Z, UsdPhysics
    axes being tokens).  Drawn-only shapes' contact properties are not compared (never used)."""
    import newton
    from force_bench.kit.usd import _mat
    out: dict[str, list] = {}

    def d(kind, what):
        out.setdefault(kind, []).append(what)

    def by_label(labels, start, n):
        m = {}
        for i in range(start, n):
            m.setdefault(labels[i], []).append(i)
        return m

    ba, bb = by_label(A.body_label, na[0], A.body_count), by_label(B.body_label, nb[0], B.body_count)
    if set(ba) != set(bb):
        d("body_labels", sorted(set(ba) ^ set(bb)))
    for lbl, ia in ba.items():
        for i, j in zip(ia, bb.get(lbl, [])):
            for f in STRUCTURE_FIELDS["body"]:
                va, vb = np.asarray(getattr(A, f)[i], float), np.asarray(getattr(B, f)[j], float)
                if f == "body_q":
                    good = _close(_mat(va), _mat(vb), rtol=0.0, atol=1e-7)
                elif f == "body_com":
                    good = _close(va, vb, rtol=0.0, atol=1e-9)
                else:                                  # relative to the largest entry (inertia: 1e-9 .. 1e-3 kg m^2)
                    good = _close(va, vb, rtol=0.0, atol=2e-6 * max(float(np.abs(va).max()), 1e-30))
                if not good:
                    d(f, lbl)
    free = int(newton.JointType.FREE)

    def jkey(Bld, jj):
        """A joint's name for matching: its label, or for a free joint (an importer names it by index) its body's."""
        return f"free:{Bld.body_label[Bld.joint_child[jj]]}" if int(Bld.joint_type[jj]) == free \
            else Bld.joint_label[jj]

    ja = {jkey(A, i): i for i in range(na[2], A.joint_count)}
    jb = {jkey(B, i): i for i in range(nb[2], B.joint_count)}
    if set(ja) != set(jb):
        d("joint_labels", sorted(set(ja) ^ set(jb)))

    def frame(Bld, jj):
        p = Bld.joint_parent[jj]
        return (_mat(Bld.body_q[p]) if p >= 0 else np.eye(4)) @ _mat(Bld.joint_X_p[jj])

    def art_members(Bld, jj):
        a = Bld.joint_articulation[jj]
        return None if a < 0 else sorted(jkey(Bld, x) for x in range(Bld.joint_count)
                                         if Bld.joint_articulation[x] == a)

    for lbl, i in ja.items():
        j = jb.get(lbl)
        if j is None:
            continue
        if int(A.joint_type[i]) != int(B.joint_type[j]) or tuple(A.joint_dof_dim[i]) != tuple(B.joint_dof_dim[j]):
            d("joint_type", lbl)
            continue
        is_free = int(A.joint_type[i]) == free
        pa, pb = A.joint_parent[i], B.joint_parent[j]
        if (A.body_label[pa] if pa >= 0 else None) != (B.body_label[pb] if pb >= 0 else None):
            d("joint_parent", lbl)
        Fa, Fb = frame(A, i), frame(B, j)
        if not _close(Fa[:3, 3], Fb[:3, 3], atol=1e-7):
            d("joint_frame", lbl)
        if art_members(A, i) != art_members(B, j):
            d("articulation", lbl)
        da, db = A.joint_qd_start[i], B.joint_qd_start[j]
        qa, qb = A.joint_q_start[i], B.joint_q_start[j]
        if is_free:
            # the pose its coordinates give the body (a quaternion's sign is free)
            Ta = Fa @ _mat(A.joint_q[qa:qa + 7]) @ np.linalg.inv(_mat(A.joint_X_c[i]))
            Tb = Fb @ _mat(B.joint_q[qb:qb + 7]) @ np.linalg.inv(_mat(B.joint_X_c[j]))
            if not _close(Ta, Tb, rtol=0.0, atol=1e-7):
                d("joint_q", lbl)
        for k in range(sum(A.joint_dof_dim[i])):
            if not _close(Fa[:3, :3] @ np.asarray(A.joint_axis[da + k], float),
                          Fb[:3, :3] @ np.asarray(B.joint_axis[db + k], float), atol=1e-6):
                d("joint_axis", lbl)
            for f in STRUCTURE_FIELDS["dof"]:
                if not _close(getattr(A, f)[da + k], getattr(B, f)[db + k], atol=1e-12):
                    d(f, f"{lbl}[{k}]")
            for f in ("joint_q", "joint_target_q"):
                if not is_free and not _close(getattr(A, f)[qa + k], getattr(B, f)[qb + k], atol=1e-9):
                    d(f, f"{lbl}[{k}]")
            for key, attr in A.custom_attributes.items():
                if attr.namespace != "mujoco" or attr.dtype is str or not isinstance(attr.values, dict) \
                        or attr.frequency != newton.Model.AttributeFrequency.JOINT_DOF:
                    continue
                other = B.custom_attributes.get(key)
                va = attr.values.get(da + k, attr.default)
                vb = other.values.get(db + k, other.default) if other is not None else None
                if (va is None) != (vb is None) or (va is not None and not _close(va, vb)):
                    d("joint_" + key.split(":")[-1], f"{lbl}[{k}]")
    sa, sb = by_label(A.shape_label, na[1], A.shape_count), by_label(B.shape_label, nb[1], B.shape_count)
    if set(sa) != set(sb) or any(len(v) != len(sb.get(k, [])) for k, v in sa.items()):
        d("shape_labels", sorted(set(sa) ^ set(sb)))
    col = int(newton.ShapeFlags.COLLIDE_SHAPES)
    vis = int(newton.ShapeFlags.VISIBLE)
    for lbl, ia in sa.items():
        for i, j in zip(ia, sb.get(lbl, [])):
            if int(A.shape_type[i]) != int(B.shape_type[j]):
                d("shape_type", lbl)
            if (int(A.shape_flags[i]) & (col | vis)) != (int(B.shape_flags[j]) & (col | vis)):
                d("shape_flags", lbl)
            if int(A.shape_flags[i]) & col:
                for f in STRUCTURE_FIELDS["shape"]:
                    if not _close(getattr(A, f)[i], getattr(B, f)[j], atol=1e-12):
                        d(f, lbl)
            pa, pb = A.shape_body[i], B.shape_body[j]
            Ta = (_mat(A.body_q[pa]) if pa >= 0 else np.eye(4)) @ _mat(A.shape_transform[i])
            Tb = (_mat(B.body_q[pb]) if pb >= 0 else np.eye(4)) @ _mat(B.shape_transform[j])
            if not _close(Ta, Tb, rtol=0.0, atol=1e-6):
                d("shape_pose", lbl)
            if not _close(A.shape_scale[i], B.shape_scale[j]):
                d("shape_scale", lbl)
            ma, mb = A.shape_source[i], B.shape_source[j]
            if (ma is None) != (mb is None) or (ma is not None and not (
                    np.array_equal(np.asarray(ma.vertices), np.asarray(mb.vertices))
                    and np.array_equal(np.asarray(ma.indices).reshape(-1), np.asarray(mb.indices).reshape(-1)))):
                d("shape_mesh", lbl)
            sdf = getattr(ma, "force_bench_sdf_params", None) if ma is not None else None
            if sdf is not None:
                if B.shape_sdf_max_resolution[j] != sdf["max_resolution"] or \
                        not _close(B.shape_sdf_narrow_band_range[j], sdf["narrow_band_range"]) or \
                        B.shape_sdf_padding[j] is None or not _close(B.shape_sdf_padding[j], sdf["margin"]):
                    d("shape_sdf", lbl)
            elif (int(A.shape_flags[i]) & col) and B.shape_sdf_max_resolution[j] is not None:
                d("shape_sdf", lbl)
            ca, cb = A.shape_color[i], B.shape_color[j]
            if ca is not None and (cb is None or not _close(ca, cb, atol=1e-5)):
                d("shape_color", lbl)

    def filters(Bld, n_s):
        return {tuple(sorted((Bld.shape_label[x], Bld.shape_label[y]))) for x, y in Bld.shape_collision_filter_pairs
                if x >= n_s and y >= n_s and int(Bld.shape_flags[x]) & col and int(Bld.shape_flags[y]) & col}

    if filters(A, na[1]) != filters(B, nb[1]):
        d("collision_filters", len(filters(A, na[1]) ^ filters(B, nb[1])))

    def mimics(Bld, n_j):
        return sorted((Bld.joint_label[x], Bld.joint_label[y], float(np.float32(c0)), float(np.float32(c1)))
                      for x, y, c0, c1 in zip(Bld.constraint_mimic_joint0, Bld.constraint_mimic_joint1,
                                              Bld.constraint_mimic_coef0, Bld.constraint_mimic_coef1) if x >= n_j)

    if mimics(A, na[2]) != mimics(B, nb[2]):
        d("gear_coupling", len(mimics(A, na[2])))
    for key in ("mujoco:eq_solref", "mujoco:eq_solimp"):
        va, vb = A.custom_attributes.get(key), B.custom_attributes.get(key)
        la = [np.asarray(v, np.float32).tolist() for v in (va.values if va else [])]
        lb = [np.asarray(v, np.float32).tolist() for v in (vb.values if vb else [])]
        if la != lb:
            d("gear_" + key.split(":")[-1].replace("eq_", ""), len(la))
    return out


def structure(keys=None, root: Path = ROUNDTRIP_OUT, usd_dir: Path | None = None, export: bool = True) -> dict:
    """For every variant: the builder ``add_asset`` fills against Newton's ``add_usd`` alone and against
    ``add_asset_usd``.  CPU only (signed-distance fields are not baked).  Writes ``root``/structure.json."""
    import warnings
    import newton
    import warp as wp
    from force_bench.assets import defer_sdf_builds
    from force_bench.kit import add_asset, load_kit, place_on_table
    from force_bench.kit.check import usd_stage_path
    from force_bench.kit.usd import add_asset_usd, export_variant, read_meta
    from force_bench.rig import Rig
    kit = load_kit()
    rows, totals = {}, {"plain": {}, "helper": {}}
    with wp.ScopedDevice("cpu"):
        for key in keys or list(kit):
            for choice in kit[key].variant_choices():
                path = usd_stage_path(key, choice, usd_dir)
                if export or not path.exists():
                    export_variant(key, choice, path)
                a = Rig("py")
                na = (a.builder.body_count, a.builder.shape_count, a.builder.joint_count)
                with defer_sdf_builds():
                    add_asset(a, key, choice, pose=place_on_table(key, choice))
                res = {}
                for mode in ("plain", "helper"):
                    b = Rig("usd")
                    nb = (b.builder.body_count, b.builder.shape_count, b.builder.joint_count)
                    with warnings.catch_warnings():
                        warnings.simplefilter("ignore")
                        if mode == "plain":                   # Newton's importer alone; labels mapped to compare
                            newton.solvers.SolverMuJoCo.register_custom_attributes(b.builder)
                            m = b.builder.add_usd(str(path))
                            rec = read_meta(path)["record"]
                            for kind, mp, arr in (("bodies", "path_body_map", "body_label"),
                                                  ("joints", "path_joint_map", "joint_label"),
                                                  ("shapes", "path_shape_map", "shape_label")):
                                for p, dd in rec[kind].items():
                                    if p in m[mp]:
                                        getattr(b.builder, arr)[m[mp][p]] = dd["label"]
                        else:
                            add_asset_usd(b, path)
                    diffs = diff_builders(a.builder, na, b.builder, nb)
                    if mode == "plain" and (a.hard_joint_limits or a.solver_options):
                        diffs["scene_settings"] = [str(a.solver_options)]
                    if mode == "helper" and (a.hard_joint_limits != b.hard_joint_limits
                                             or a.solver_options != b.solver_options):
                        diffs["scene_settings"] = [str(b.solver_options)]
                    res[mode] = {k: len(v) for k, v in diffs.items()}
                    for k in diffs:
                        totals[mode][k] = totals[mode].get(k, 0) + 1
                rows[f"{key}/{path.stem}"] = res
                print(f"[structure] {key}/{path.stem}: add_usd {res['plain'] or 'identical'}; "
                      f"add_asset_usd {res['helper'] or 'identical'}", flush=True)
    out = {"variants": len(rows), "identical_with_helper": sum(1 for r in rows.values() if not r["helper"]),
           "identical_with_add_usd_alone": sum(1 for r in rows.values() if not r["plain"]),
           "variants_with_difference": totals, "rows": rows}
    root.mkdir(parents=True, exist_ok=True)
    (root / "structure.json").write_text(json.dumps(out, indent=1))
    return out
