"""Probe figure: the simulated finger against its two quasi-static references.

    for z in 45 60 75 90 105; do python -m force_bench.finray.probe --z -$z; done
    python -m force_bench.finray.report [--probe-root outputs/finray] [--out outputs/finray/probe.png]

The probe root holds the folders written by ``probe.py`` (probe_prb_z45 ... probe_prb_z105).  The frame
reference (``reference()``, the 2 mm frame model, about 2 min on a CPU) is written to
<probe root>/nl_reference.json the first time and reused.  Prints the table of docs/finray.md.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

HEIGHTS = (45, 60, 75, 90, 105)
SIM, CPU, REF = "#2a78d6", "#eb6834", "#52514e"       # reference palette slots 1, 2 + secondary ink


def reference(out: Path, heights=HEIGHTS, depth=20.0, R=12.5) -> dict:
    """The fine large-deformation frame reference (``frame_nl.press``)."""
    from force_bench.finray.frame_nl import press
    res = {}
    for z in heights:
        _, rows = press(z_center=-float(z), R=R, depth=depth, steps=int(depth))
        res[f"z{z}_R{R}"] = rows
    Path(out).write_text(json.dumps(res, indent=1))
    return res


def figure(probe_root: Path, ref_json: Path, out: Path) -> dict:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from force_bench.finray import prb
    from force_bench.finray.prb_cpu import press as cpu_press
    ref = json.loads(Path(ref_json).read_text())
    L = prb.layout()
    fig, axes = plt.subplots(3, len(HEIGHTS), figsize=(3.0 * len(HEIGHTS), 7.6), sharex=True)
    summary = {}
    for j, z in enumerate(HEIGHTS):
        d = json.loads((Path(probe_root) / f"probe_prb_z{z}" / "probe.json").read_text())
        log = [r for r in d["log"] if r["indent_mm"] >= 0.0]
        ind = np.array([r["indent_mm"] for r in log])
        F = np.array([r["force_N"] for r in log])
        tdx = np.array([r["tip_dx_mm"] for r in log])
        wr = np.array([r["wrap_deg"] for r in log])
        rr = ref[f"z{z}_R12.5"]
        ri = np.array([r["indent_mm"] for r in rr])
        _, cpu = cpu_press(L, z_center=-float(z), depth=20.0, steps=10)
        ci = np.array([r["indent_mm"] for r in cpu])
        a = axes[0, j]
        a.plot(ri, [r["force_N"] for r in rr], c=REF, lw=1.4, ls="--", label="frame model, 2 mm beams")
        a.plot(ci, [r["force_N"] for r in cpu], c=CPU, lw=1.4, ls=":", label="PRB linkage, static (CPU)")
        a.plot(ind, F, c=SIM, lw=1.6, label="PRB linkage in the sim (MuJoCo)")
        a.set_title(f"press at {z} mm below the base", fontsize=9)
        a.set_ylim(0, 45)
        b = axes[1, j]
        b.axhline(0.0, c="0.8", lw=0.8)
        b.plot(ri, [r["tip_dx_mm"] for r in rr], c=REF, lw=1.4, ls="--")
        b.plot(ci, [r["tip_dx_mm"] for r in cpu], c=CPU, lw=1.4, ls=":")
        b.plot(ind, tdx, c=SIM, lw=1.6)
        b.set_ylim(-4, 14)
        c = axes[2, j]
        c.axhline(25.3, c="0.75", lw=0.8, ls="--")
        c.plot(ind, wr, c=SIM, lw=1.6)
        c.set_ylim(0, 100)
        c.set_xlabel("indentation (mm)", fontsize=8)
        for ax in (a, b, c):
            ax.grid(True, lw=0.3, c="0.85")
            ax.tick_params(labelsize=8)
        k10 = int(np.argmin(np.abs(ind - 10.0)))
        k20 = int(np.argmin(np.abs(ind - min(19.8, ind.max()))))
        summary[z] = {"F10_sim": round(float(F[k10]), 2), "F10_ref": rr[10]["force_N"],
                      "F20_sim": round(float(F[k20]), 2), "F20_ref": rr[20]["force_N"],
                      "tip10_sim": round(float(tdx[k10]), 2), "tip10_ref": rr[10]["tip_dx_mm"],
                      "tip20_sim": round(float(tdx[k20]), 2), "tip20_ref": rr[20]["tip_dx_mm"],
                      "wrap10_deg": float(wr[k10]), "wrap20_deg": float(wr[k20])}
        a.annotate(f"{F[k20]:.0f} N", (ind[k20], F[k20]), (-4, 6), textcoords="offset points",
                   fontsize=8, color="0.2", ha="right")
    axes[0, 0].set_ylabel("force on the cylinder (N)", fontsize=8)
    axes[1, 0].set_ylabel("tip motion (mm)\n< 0: toward the object", fontsize=8)
    axes[2, 0].set_ylabel("contact arc on the\ncylinder (deg)", fontsize=8)
    axes[2, 0].text(9.0, 17.0, "flat, rigid face", fontsize=7, color="0.45")
    axes[0, 4].legend(fontsize=7, loc="upper left", frameon=False)
    fig.suptitle("Fin Ray finger pressed by a Ø25 mm cylinder (TPU 95A, E = 26 MPa estimated)", fontsize=10)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    return summary


def main(argv=None):
    from force_bench.paths import OUTPUTS_ROOT
    ap = argparse.ArgumentParser(description="Fin Ray probe figure and table")
    ap.add_argument("--probe-root", type=Path, default=OUTPUTS_ROOT / "finray")
    ap.add_argument("--reference", type=Path, default=None, help="default: <probe root>/nl_reference.json")
    ap.add_argument("--out", type=Path, default=None, help="default: <probe root>/probe.png")
    a = ap.parse_args(argv)
    ref = a.reference or a.probe_root / "nl_reference.json"
    if not ref.exists():
        print(f"computing the frame reference -> {ref}")
        reference(ref)
    s = figure(a.probe_root, ref, a.out or a.probe_root / "probe.png")
    print(json.dumps(s, indent=1))


if __name__ == "__main__":
    main()
