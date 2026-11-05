"""Large-deformation reference for the Fin Ray profile, with a cylinder press.

The linear frame model (``frame_fe``) cannot show the Fin Ray effect: the
wrap comes from the thin contact wall bending around the object, and the
shortening of its chord that this bending causes is second order in the
deflection, i.e. zero in linear theory.  Measured with the linear model:
a point load at mid-length moves the tip 0.56 mm AWAY from the load per mm
of indentation, a cantilever.

This model keeps the same walls, thicknesses and modulus (``frame_fe.Frame``
mesh, ~2 mm edges) but is geometrically exact: every node has a position,
every edge an axial spring E'A/L, every interior node of a wall a bending
spring E'I/l on its turning angle, every rib-wall junction a bending spring
on the rib's angle to the wall (the junctions are printed solid), and the
wall roots are clamped.  A rigid cylinder of radius R (axis along the
finger width) is pushed into the contact wall's face (x = 0) step by step;
contact is a stiff penalty on the wall's outer surface (centreline - t/2).
Each step is a static equilibrium solved by sparse nonlinear least squares.

It is a quasi-static reference built from the same estimated modulus as the
simulation model, not a measurement of a printed finger.
"""

from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix

from force_bench.finray.frame_fe import Frame, E_TPU95A

K_PEN = 2000.0            # N/mm, cylinder-wall penalty (stiff vs the wall)


def _angle(e1, e2):
    return np.arctan2(e1[..., 0] * e2[..., 1] - e1[..., 1] * e2[..., 0],
                      (e1 * e2).sum(-1))


class FrameNL:
    def __init__(self, E: float = E_TPU95A, seg: float = 2.0, fixed_extra=None, prof=None):
        fr = Frame(E=E, seg=seg, prof=prof)
        self.fr = fr
        X0 = np.asarray(fr.nodes)
        self.X0 = X0
        n = len(X0)
        Ep, w = fr.Ep, fr.w
        # edges (stretch)
        ea, eb, ks = [], [], []
        for a, b, t, _ in fr.elems:
            L = np.linalg.norm(X0[b] - X0[a])
            ea.append(a), eb.append(b), ks.append(Ep * w * t / L)
        self.ea, self.eb = np.array(ea), np.array(eb)
        self.L0 = np.linalg.norm(X0[self.eb] - X0[self.ea], axis=1)
        self.sk = np.sqrt(np.array(ks))
        # bending triples (p, c, q): angle between edge p->c and c->q
        trip, kb = [], []
        for ids, ts, kind in fr.members:
            for k in range(1, len(ids) - 1):
                a, c, b = ids[k - 1], ids[k], ids[k + 1]
                l1 = np.linalg.norm(X0[c] - X0[a])
                l2 = np.linalg.norm(X0[b] - X0[c])
                t = 0.5 * (ts[k - 1] + ts[k])
                trip.append((a, c, b)), kb.append(Ep * w * t ** 3 / 12.0 / (0.5 * (l1 + l2)))
        # junctions: rib end edge vs the wall edges at that node (both sides)
        walls = {}
        for ids, ts, kind in fr.members:
            if kind == "rib":
                continue
            for k, i in enumerate(ids):
                walls.setdefault(i, []).append((ids, ts, k))
        for ids, ts, kind in fr.members:
            if kind != "rib":
                continue
            for end, nb, te in ((ids[0], ids[1], ts[0]), (ids[-1], ids[-2], ts[-1])):
                l = np.linalg.norm(X0[nb] - X0[end])
                kj = Ep * w * te ** 3 / 12.0 / l
                for wids, wts, k in walls.get(end, []):
                    for kk in (k - 1, k + 1):
                        if 0 <= kk < len(wids):
                            # angle at `end` between wall neighbour and rib neighbour
                            trip.append((wids[kk], end, nb)), kb.append(kj)
        # the tip: last contact edge vs last back edge
        c_ids = fr.contact_ids
        b_ids = fr.back_ids
        trip.append((c_ids[-2], c_ids[-1], b_ids[-2]))
        kb.append(Ep * w * fr.p["contact_wall_t"] ** 3 / 12.0 / 2.0)
        self.trip = np.array(trip)
        V = lambda i: X0[self.trip[:, i]]
        self.phi0 = _angle(V(1) - V(0), V(2) - V(1))
        self.sb = np.sqrt(np.array(kb))
        # fixed nodes: both wall roots and their first neighbours (clamp)
        fixed = {c_ids[0], c_ids[1], b_ids[0], b_ids[1]}
        if fixed_extra == "lug":
            from force_bench.finray.geometry import LUG_HOLE
            k = int(np.argmin([np.linalg.norm(X0[i] - np.asarray(LUG_HOLE)) for i in b_ids]))
            fixed |= {b_ids[k]}
        self.free = np.array([i for i in range(n) if i not in fixed])
        self.n = n
        # contact candidates: contact-wall nodes, offset to the outer face
        self.cnodes = np.array(c_ids)
        self.half_t = 0.5 * fr.p["contact_wall_t"]
        self.X = X0.copy()
        self._sparsity = None

    # ── residuals ────────────────────────────────────────────────────────
    def _full(self, u):
        X = self.X0.copy()
        X[self.free] = u.reshape(-1, 2)
        return X

    def residual(self, u, cyl):
        X = self._full(u)
        e = X[self.eb] - X[self.ea]
        r_s = self.sk * (np.linalg.norm(e, axis=1) - self.L0)
        a, c, b = X[self.trip[:, 0]], X[self.trip[:, 1]], X[self.trip[:, 2]]
        r_b = self.sb * (_angle(c - a, b - c) - self.phi0)
        out = [r_s, r_b]
        if cyl is not None:
            (cx, cz), R = cyl
            p = X[self.cnodes] - np.array([self.half_t, 0.0])   # outer face
            d = np.hypot(p[:, 0] - cx, p[:, 1] - cz)
            out.append(np.sqrt(K_PEN) * np.maximum(R - d, 0.0))
        return np.concatenate(out)

    def sparsity(self, with_cyl: bool):
        nf = len(self.free)
        col = -np.ones(self.n, int)
        col[self.free] = np.arange(nf)
        rows = len(self.ea) + len(self.trip) + (len(self.cnodes) if with_cyl else 0)
        S = lil_matrix((rows, 2 * nf), dtype=int)
        r = 0
        for a, b in zip(self.ea, self.eb):
            for i in (a, b):
                if col[i] >= 0:
                    S[r, 2 * col[i]] = S[r, 2 * col[i] + 1] = 1
            r += 1
        for tr in self.trip:
            for i in tr:
                if col[i] >= 0:
                    S[r, 2 * col[i]] = S[r, 2 * col[i] + 1] = 1
            r += 1
        if with_cyl:
            for i in self.cnodes:
                if col[i] >= 0:
                    S[r, 2 * col[i]] = S[r, 2 * col[i] + 1] = 1
                r += 1
        return S.tocsr()

    def solve(self, cyl, u0=None):
        u0 = self.X[self.free].ravel() if u0 is None else u0
        S = self.sparsity(cyl is not None)
        sol = least_squares(self.residual, u0, args=(cyl,), jac_sparsity=S,
                            method="trf", x_scale=1.0, xtol=1e-10, ftol=1e-12, gtol=1e-10,
                            max_nfev=400)
        self.X = self._full(sol.x)
        return sol

    def contact_force(self, cyl) -> tuple[float, np.ndarray]:
        (cx, cz), R = cyl
        p = self.X[self.cnodes] - np.array([self.half_t, 0.0])
        v = p - np.array([cx, cz])
        d = np.linalg.norm(v, axis=1)
        pen = np.maximum(R - d, 0.0)
        f = (K_PEN * pen)[:, None] * v / np.maximum(d, 1e-9)[:, None]   # on the wall
        return float(f[:, 0].sum()), pen


def press(z_center: float = -60.0, R: float = 12.5, depth: float = 10.0, steps: int = 20,
          E: float = E_TPU95A, fixed_extra=None, contact_tol: float | None = None, prof: dict | None = None):
    """Push a cylinder (axis along the width) into the contact face at STEP
    height z_center (mm), from first touch to `depth` mm.  Rows: indentation
    (mm), force (N), wrap angle (deg of the cylinder's circumference the
    face touches), contact length (mm), tip x motion (mm, < 0 = toward the
    object), tip z motion (mm)."""
    m = FrameNL(E=E, fixed_extra=fixed_extra, prof=prof)
    tip0 = m.X0[m.fr.tip_id].copy()
    rows = []
    for k in range(steps + 1):
        dx = depth * k / steps
        cyl = ((-R + dx, z_center), R)
        m.solve(cyl)
        F, pen = m.contact_force(cyl)
        p = m.X[m.cnodes] - np.array([m.half_t, 0.0])
        d = np.hypot(p[:, 0] - cyl[0][0], p[:, 1] - cyl[0][1])
        touch = pen > 1e-4 if contact_tol is None else d < R + contact_tol
        ang = np.degrees(np.arctan2(p[touch, 1] - cyl[0][1], p[touch, 0] - cyl[0][0])) if touch.any() else np.zeros(0)
        span = float(ang.max() - ang.min()) if len(ang) > 1 else 0.0
        clen = float(np.sum(np.linalg.norm(np.diff(p[touch], axis=0), axis=1))) if touch.sum() > 1 else 0.0
        tip = m.X[m.fr.tip_id] - tip0
        rows.append({"indent_mm": round(dx, 2), "force_N": round(F, 3), "wrap_deg": round(span, 1),
                     "contact_len_mm": round(clen, 1), "tip_dx_mm": round(float(tip[0]), 3),
                     "tip_dz_mm": round(float(tip[1]), 3)})
    return m, rows


if __name__ == "__main__":
    import json, sys
    z = float(sys.argv[1]) if len(sys.argv) > 1 else -60.0
    m, rows = press(z_center=z)
    for r in rows:
        print(json.dumps(r))
