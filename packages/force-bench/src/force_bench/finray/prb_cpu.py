"""The PRB linkage solved quasi-statically on the CPU, for calibration.

Same nodes, links, springs and loops as the Newton model (``prb.layout``),
same static-equilibrium solver as the fine frame reference (``frame_nl``):
rigid links (stiff axial residuals), revolute springs, pins where the loops
close, and a penalty cylinder pressed into the contact wall's outer face
(segment-wise, so a contact between two joints counts).  ``calibrate()``
fits ``K_SCALE`` and ``C_RIB`` to the fine reference.
"""

from __future__ import annotations

import json

import numpy as np
from scipy.optimize import least_squares

from force_bench.finray import prb
from force_bench.finray.frame_nl import _angle

K_PEN = 2000.0    # N/mm


class PRBStatic:
    def __init__(self, L: prb.Layout):
        self.L = L
        pts, key = [], {}

        def node(p):
            k = (round(float(p[0]), 4), round(float(p[1]), 4))
            if k not in key:
                key[k] = len(pts)
                pts.append(np.asarray(p, float))
            return key[k]

        by = L.by_name()
        self.edges = []
        for l in L.links:
            self.edges.append((node(l.p0), node(l.p1)))
        # loop points coincide with link ends already (pins = shared nodes)
        ghosts = []
        trips, ks = [], []
        for l in L.links:
            a, b = node(l.p0), node(l.p1)
            if l.parent is None:
                d = (l.p1 - l.p0) / l.length
                g = len(pts) + len(ghosts)
                ghosts.append(l.p0 - d)          # clamped root: a fixed ghost
                trips.append((g, a, b))
            elif l.kind == "rib":
                trips.append((node(by[l.parent].p1), a, b))   # rib end vs its wall link
            else:
                trips.append((node(by[l.parent].p0), a, b))
            ks.append(l.k * 1000.0)              # N.m/rad -> N.mm/rad
        for la, lb, pt in L.loops:               # welds: the relative angle is frozen
            A, B = by[la], by[lb]
            trips.append((node(A.p0), node(pt), node(B.p0)))
            ks.append(1.0e5)
        X0 = np.vstack([np.asarray(pts), np.asarray(ghosts)])
        self.X0 = X0
        self.n = len(X0)
        self.trip = np.array(trips)
        self.sb = np.sqrt(np.array(ks))
        V = lambda i: X0[self.trip[:, i]]
        self.phi0 = _angle(V(1) - V(0), V(2) - V(1))
        self.ea = np.array([e[0] for e in self.edges])
        self.eb = np.array([e[1] for e in self.edges])
        self.L0 = np.linalg.norm(X0[self.eb] - X0[self.ea], axis=1)
        self.sk = np.sqrt(1e5 / self.L0)          # rigid links: stiff axial springs
        roots = [node(l.p0) for l in L.links if l.parent is None]
        fixed = set(roots) | set(range(len(pts), self.n))
        self.free = np.array([i for i in range(self.n) if i not in fixed])
        self.cseg = [(node(l.p0), node(l.p1)) for l in L.links if l.kind == "contact"]
        self.half_t = 0.5 * L.links[0].t
        self.tip = node(L.loops[-1][2])
        self.X = X0.copy()

    def _full(self, u):
        X = self.X0.copy()
        X[self.free] = u.reshape(-1, 2)
        return X

    def _faces(self, X):
        a = X[[s[0] for s in self.cseg]]
        b = X[[s[1] for s in self.cseg]]
        d = (b - a) / np.linalg.norm(b - a, axis=1)[:, None]
        n = np.stack([d[:, 1], -d[:, 0]], axis=1)      # (0,-1) -> (-1, 0): toward the object
        return a + self.half_t * n, b + self.half_t * n

    def _seg_dist(self, X, c):
        a, b = self._faces(X)
        ab = b - a
        t = np.clip(((c - a) * ab).sum(1) / (ab * ab).sum(1), 0.0, 1.0)
        q = a + t[:, None] * ab
        return np.linalg.norm(q - c, axis=1), q, t

    def residual(self, u, cyl):
        X = self._full(u)
        e = X[self.eb] - X[self.ea]
        r_s = self.sk * (np.linalg.norm(e, axis=1) - self.L0)
        a, c, b = X[self.trip[:, 0]], X[self.trip[:, 1]], X[self.trip[:, 2]]
        r_b = self.sb * (_angle(c - a, b - c) - self.phi0)
        out = [r_s, r_b]
        if cyl is not None:
            (cx, cz), R = cyl
            d, _, _ = self._seg_dist(X, np.array([cx, cz]))
            out.append(np.sqrt(K_PEN) * np.maximum(R - d, 0.0))
        return np.concatenate(out)

    def solve(self, cyl):
        sol = least_squares(self.residual, self.X[self.free].ravel(), args=(cyl,),
                            method="trf", xtol=1e-10, ftol=1e-12, gtol=1e-10, max_nfev=600)
        self.X = self._full(sol.x)
        return sol

    def force(self, cyl):
        (cx, cz), R = cyl
        c = np.array([cx, cz])
        d, q, _ = self._seg_dist(self.X, c)
        pen = np.maximum(R - d, 0.0)
        v = (q - c) / np.maximum(d, 1e-9)[:, None]
        return float((K_PEN * pen * v[:, 0]).sum()), pen


def press(L: prb.Layout, z_center=-60.0, R=12.5, depth=20.0, steps=20):
    m = PRBStatic(L)
    tip0 = m.X0[m.tip].copy()
    rows = []
    for k in range(steps + 1):
        dx = depth * k / steps
        cyl = ((-R + dx, z_center), R)
        m.solve(cyl)
        F, pen = m.force(cyl)
        tip = m.X[m.tip] - tip0
        rows.append({"indent_mm": round(dx, 2), "force_N": round(F, 3),
                     "n_contact_links": int((pen > 1e-4).sum()),
                     "tip_dx_mm": round(float(tip[0]), 3), "tip_dz_mm": round(float(tip[1]), 3)})
    return m, rows


REF_JSON = "nl_reference.json"


def calibrate(ref: dict, heights=(-45, -60, -75, -90), k_grid=(0.6, 0.8, 1.0, 1.2, 1.4),
              c_grid=(6.0, 12.0, 24.0), depth=20.0, steps=10):
    """Grid fit of (K_SCALE, C_RIB) to the fine reference: force at 5, 10,
    20 mm and tip motion at 10, 20 mm, all heights.  Returns the table."""
    out = []
    for c in c_grid:
        for ks in k_grid:
            L = prb.layout(k_scale=ks, c_rib=c)
            err_f, err_t = [], []
            for z in heights:
                _, rows = press(L, z_center=float(z), depth=depth, steps=steps)
                rr = ref[f"z{-z}_R12.5"]
                for d in (4.0, 10.0, 20.0):
                    a = next(r for r in rows if abs(r["indent_mm"] - d) < 1e-6)
                    b = next(r for r in rr if abs(r["indent_mm"] - d) < 1e-6)
                    err_f.append(np.log(max(a["force_N"], 1e-3) / max(b["force_N"], 1e-3)))
                for d in (10.0, 20.0):
                    a = next(r for r in rows if abs(r["indent_mm"] - d) < 1e-6)
                    b = next(r for r in rr if abs(r["indent_mm"] - d) < 1e-6)
                    err_t.append(np.hypot(a["tip_dx_mm"] - b["tip_dx_mm"], a["tip_dz_mm"] - b["tip_dz_mm"]))
            row = {"k_scale": ks, "c_rib": c, "force_rms_log": float(np.sqrt(np.mean(np.square(err_f)))),
                   "tip_rms_mm": float(np.sqrt(np.mean(np.square(err_t))))}
            print(json.dumps(row), flush=True)
            out.append(row)
    return out


if __name__ == "__main__":
    import sys
    from pathlib import Path
    ref = json.loads(Path(sys.argv[1]).read_text())
    calibrate(ref)
