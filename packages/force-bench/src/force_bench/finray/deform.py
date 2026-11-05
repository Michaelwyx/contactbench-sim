"""How much the simulated Fin Ray fingers deform, from recorded body poses.

Per finger and per frame:
  hinge_deg       the largest rotation of any linkage joint from its printed
                  shape (wall and rib joints, about the finger's width axis)
  wall_deg        the same over the contact-wall joints only (the face's bending)
  tip_mm          how far the finger tip has moved from its printed position,
                  in the carriage's own frame (so carriage motion does not count)
  face_tilt_deg   the angle between the contact face's lowest link and the
                  carriage's approach axis (the face's tilt at the tip)

``summarize`` reduces them over time windows (e.g. the phases of an episode).

    python -m force_bench.finray.deform <rollout_states.npz> [t0,t1 ...]
"""

from __future__ import annotations

import json
import sys

import numpy as np

from force_bench.finray import geometry as G
from force_bench.finray import prb

SIDES = (("L", "fr3/fr3_leftfinger"), ("R", "fr3/fr3_rightfinger"))


def _R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _xangle(R) -> float:
    """Rotation of a (near pure x-axis) relative rotation, degrees."""
    return float(np.degrees(np.arctan2(R[2, 1], R[1, 1])))


class Deformation:
    def __init__(self, body_labels, layout: prb.Layout | None = None):
        self.L = layout or prb.layout()
        lab = list(body_labels)
        self.idx = {}
        for side, car in SIDES:
            if car not in lab or f"fr3/finray_{side}_c00" not in lab:
                continue
            d = {"carriage": lab.index(car)}
            for l in self.L.links:
                d[l.name] = lab.index(f"fr3/finray_{side}_{l.name}")
            self.idx[side] = d
        contact = [l for l in self.L.links if l.kind == "contact"]
        self.last = contact[-1]
        self.contact_names = {l.name for l in contact}
        p = self.last
        self.tip_local = np.array([0.0, (p.p1[0] - p.p0[0]) / 1000.0, -(p.p1[1] - p.p0[1]) / 1000.0])
        tip = p.p1
        self.tip_rest = np.array([0.0, tip[0] / 1000.0, (G.Z_BASE - tip[1]) / 1000.0])

    @property
    def ok(self) -> bool:
        return bool(self.idx)

    def frame(self, bq) -> dict:
        out = {}
        for side, d in self.idx.items():
            Rc = _R(bq[d["carriage"], 3:7])
            pc = bq[d["carriage"], :3]
            hinge, wall = 0.0, 0.0
            for l in self.L.links:
                Rl = _R(bq[d[l.name], 3:7])
                Rp = Rc if l.parent is None else _R(bq[d[l.parent], 3:7])
                a = abs(_xangle(Rp.T @ Rl))
                hinge = max(hinge, a)
                if l.kind == "contact":
                    wall = max(wall, a)
            b = bq[d[self.last.name]]
            Rb = _R(b[3:7])
            tip = Rc.T @ (b[:3] + Rb @ self.tip_local - pc)
            tilt = abs(_xangle(Rc.T @ Rb))
            out[side] = {"hinge_deg": hinge, "wall_deg": wall,
                         "tip_mm": float(np.linalg.norm(tip - self.tip_rest) * 1000.0),
                         "face_tilt_deg": tilt}
        return out

    def series(self, body_q, stride: int = 1) -> dict:
        keys = ("hinge_deg", "wall_deg", "tip_mm", "face_tilt_deg")
        s = {side: {k: [] for k in keys} for side in self.idx}
        for f in range(0, len(body_q), stride):
            r = self.frame(body_q[f])
            for side in r:
                for k in keys:
                    s[side][k].append(r[side][k])
        return {side: {k: np.asarray(v) for k, v in d.items()} for side, d in s.items()}


def summarize(states_npz, windows: dict | None = None, stride: int = 1) -> dict:
    """Max of every quantity per finger (and both) over each window {name: (t0, t1)}."""
    d = np.load(states_npz, allow_pickle=True)
    t = d["t"][::stride]
    D = Deformation(d["body_labels"])
    if not D.ok:
        return {}
    s = D.series(d["body_q"], stride)
    windows = windows or {"episode": (float(t[0]), float(t[-1]) + 1.0)}
    out = {}
    for name, (t0, t1) in windows.items():
        m = (t >= t0) & (t <= t1)
        if not m.any():
            continue
        row = {}
        for k in ("hinge_deg", "wall_deg", "tip_mm", "face_tilt_deg"):
            v = max(float(s[side][k][m].max()) for side in s)
            row[k] = round(v, 2)
        out[name] = row
    return out


if __name__ == "__main__":
    path = sys.argv[1]
    win = None
    if len(sys.argv) > 2:
        cuts = [float(x) for x in sys.argv[2].split(",")]
        win = {f"{a:.1f}-{b:.1f}s": (a, b) for a, b in zip(cuts[:-1], cuts[1:])}
    print(json.dumps(summarize(path, win), indent=1))
