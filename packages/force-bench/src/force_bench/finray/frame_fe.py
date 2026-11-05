"""Linear 2-D frame model of the Fin Ray profile: the stiffness ground truth.

Every wall of the measured profile (``geometry.profile()``) is an
Euler-Bernoulli beam of its own measured thickness: the 1.8 mm contact wall
(centreline x = 0.9 mm), the fifteen ribs (2.0 / 1.2 mm), and the back wall
(6.0 mm at the base tapering to 2.5 mm at the tip) along its measured
midline.  The ribs are rigidly joined to both walls (the part is printed in
one piece), the two walls meet at the tip and are clamped to the base block
at z = -10 mm.  Plane strain across the 25.8 mm width: E' = E / (1 - nu^2).

It is linear (small strain AND small rotation), so it is valid for the
first millimetres of indentation; the pseudo-rigid-body model in
``model.py`` is checked against it there and takes over beyond.

The modulus is an ESTIMATE: TPU 95A filament data sheets give a tensile
modulus of ~26 MPa (Ultimaker TPU 95A, NinjaTek Cheetah); 85A grades are
~12 MPa.  A printed part is softer than the bulk value where layers bond
imperfectly, so treat every stiffness here as +-50 %.
"""

from __future__ import annotations

import numpy as np

from force_bench.finray.geometry import WIDTH, profile

E_TPU95A = 26.0           # MPa, tensile modulus, data-sheet value (estimate)
NU_TPU = 0.45
SEG = 2.0                 # mm, target element length


class Frame:
    def __init__(self, E: float = E_TPU95A, nu: float = NU_TPU, seg: float = SEG,
                 width: float = WIDTH, prof: dict | None = None):
        self.Ep = E / (1.0 - nu * nu)
        self.w = width
        self.p = prof or profile()
        self.nodes: list[np.ndarray] = []
        self.elems: list[tuple[int, int, float, str]] = []   # (a, b, t, kind)
        self.members: list[tuple[list[int], list[float], str]] = []   # polyline ids, edge t, kind
        self._build(seg)

    # ── mesh ─────────────────────────────────────────────────────────────
    def _node(self, xz) -> int:
        xz = np.asarray(xz, float)
        for i, n in enumerate(self.nodes):
            if np.linalg.norm(n - xz) < 1e-6:
                return i
        self.nodes.append(xz)
        return len(self.nodes) - 1

    def _line(self, a, b, t, kind, seg):
        a, b = np.asarray(a, float), np.asarray(b, float)
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / seg)))
        ids = [self._node(a + (b - a) * k / n) for k in range(n + 1)]
        for i, j in zip(ids[:-1], ids[1:]):
            self.elems.append((i, j, t, kind))
        return ids

    def _member(self, ids: list[int], kind: str):
        ts = []
        for a, b in zip(ids[:-1], ids[1:]):
            ts.append(next(t for (i, j, t, _) in self.elems if (i, j) == (a, b)))
        self.members.append((ids, ts, kind))

    def _build(self, seg):
        p = self.p
        xc = p["contact_mid_x"]
        tc = p["contact_wall_t"]
        ribs = p["ribs"]
        tip = np.asarray(p["tip"])
        # contact wall: base -> rib inner nodes -> tip
        zc = [p["base_z"]] + [r["inner"][1] for r in ribs] + [tip[1]]
        self.contact_ids = []
        for z0, z1 in zip(zc[:-1], zc[1:]):
            ids = self._line([xc, z0], [xc, z1], tc, "contact", seg)
            self.contact_ids += ids if not self.contact_ids else ids[1:]
        self.base_c = self.contact_ids[0]
        self.tip_id = self.contact_ids[-1]
        self._member(self.contact_ids, "contact")
        # back wall: along the midline, with the rib outer ends inserted
        mid = np.asarray(p["back_mid"])
        bt = np.asarray(p["back_t"])
        s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(mid, axis=0), axis=1))]
        knots = [0.0]
        for r in ribs:
            o = np.asarray(r["outer"])
            k = int(np.argmin(np.linalg.norm(mid - o, axis=1)))
            knots.append(s[k] + float(np.dot(o - mid[k], (mid[min(k + 1, len(mid) - 1)] - mid[k]) /
                                              max(1e-9, np.linalg.norm(mid[min(k + 1, len(mid) - 1)] - mid[k])))))
        knots.append(s[-1])
        pts = [mid[0]] + [np.asarray(r["outer"]) for r in ribs] + [mid[-1]]
        self.back_ids = []
        for (k0, k1), a, b in zip(zip(knots[:-1], knots[1:]), pts[:-1], pts[1:]):
            t = float(np.interp(0.5 * (k0 + k1), s, bt))
            ids = self._line(a, b, t, "back", seg)
            self.back_ids += ids if not self.back_ids else ids[1:]
        self.base_b = self.back_ids[0]
        # the back wall closes onto the contact wall at the tip
        self.back_ids += self._line(self.nodes[self.back_ids[-1]], tip, float(bt[-1]), "back", seg)[1:]
        self._member(self.back_ids, "back")
        # ribs
        self.rib_ids = []
        for r in ribs:
            self.rib_ids.append(self._line(r["inner"], r["outer"], r["t"], "rib", seg))
            self._member(self.rib_ids[-1], "rib")
        self.rib_inner = [ids[0] for ids in self.rib_ids]

    # ── solve ────────────────────────────────────────────────────────────
    def stiffness(self) -> np.ndarray:
        n = len(self.nodes)
        K = np.zeros((3 * n, 3 * n))
        for a, b, t, _ in self.elems:
            pa, pb = self.nodes[a], self.nodes[b]
            d = pb - pa
            L = float(np.linalg.norm(d))
            c, s = d / L
            A = self.w * t
            I = self.w * t ** 3 / 12.0
            EA, EI = self.Ep * A, self.Ep * I
            k = np.array([
                [EA / L, 0, 0, -EA / L, 0, 0],
                [0, 12 * EI / L ** 3, 6 * EI / L ** 2, 0, -12 * EI / L ** 3, 6 * EI / L ** 2],
                [0, 6 * EI / L ** 2, 4 * EI / L, 0, -6 * EI / L ** 2, 2 * EI / L],
                [-EA / L, 0, 0, EA / L, 0, 0],
                [0, -12 * EI / L ** 3, -6 * EI / L ** 2, 0, 12 * EI / L ** 3, -6 * EI / L ** 2],
                [0, 6 * EI / L ** 2, 2 * EI / L, 0, -6 * EI / L ** 2, 4 * EI / L]])
            T = np.zeros((6, 6))
            r = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])
            T[:3, :3] = r
            T[3:, 3:] = r
            kg = T.T @ k @ T
            idx = [3 * a, 3 * a + 1, 3 * a + 2, 3 * b, 3 * b + 1, 3 * b + 2]
            K[np.ix_(idx, idx)] += kg
        return K

    def solve(self, loads: dict[int, np.ndarray], fixed=None) -> np.ndarray:
        """loads: node -> (Fx, Fz, M) in N, N.mm.  Returns (n, 3) displacements
        (mm, mm, rad).  The two wall roots are clamped."""
        K = self.stiffness()
        n = len(self.nodes)
        f = np.zeros(3 * n)
        for i, v in loads.items():
            f[3 * i:3 * i + 3] += v
        fixed = set(fixed or ()) | {self.base_c, self.base_b}
        free = np.array([d for d in range(3 * n) if d // 3 not in fixed])
        u = np.zeros(3 * n)
        u[free] = np.linalg.solve(K[np.ix_(free, free)], f[free])
        return u.reshape(n, 3)

    def contact_node_at(self, z: float) -> int:
        zs = np.array([self.nodes[i][1] for i in self.contact_ids])
        return self.contact_ids[int(np.argmin(np.abs(zs - z)))]


def press_table(E: float = E_TPU95A, heights=None, F: float = 1.0, fixed_extra=None):
    """Normal point load F (N) into the contact wall at STEP heights z (mm).
    Returns rows: z, stiffness (N/mm), tip motion toward the load per mm of
    indentation (> 0 = the tip wraps toward the object: the Fin Ray effect)."""
    fr = Frame(E=E)
    fixed = None
    if fixed_extra == "lug":
        # the optional lug pin: the back-wall node nearest the lug hole
        from force_bench.finray.geometry import LUG_HOLE
        b = fr.back_ids
        k = int(np.argmin([np.linalg.norm(fr.nodes[i] - np.asarray(LUG_HOLE)) for i in b]))
        fixed = [b[k]]
    heights = heights or [-30.0, -45.0, -60.0, -75.0, -90.0, -105.0, -115.0]
    rows = []
    for z in heights:
        i = fr.contact_node_at(z)
        u = fr.solve({i: np.array([F, 0.0, 0.0])}, fixed=fixed)
        k = F / u[i, 0]
        wrap = -u[fr.tip_id, 0] / u[i, 0]
        rows.append({"z_mm": round(float(fr.nodes[i][1]), 2), "k_N_per_mm": round(float(k), 4),
                     "tip_wrap_per_mm": round(float(wrap), 3),
                     "tip_dz_per_mm": round(float(u[fr.tip_id, 1] / u[i, 0]), 3)})
    return fr, rows


if __name__ == "__main__":
    import json
    for E in (12.0, E_TPU95A):
        fr, rows = press_table(E)
        print(f"E = {E} MPa (plane strain {fr.Ep:.1f}), {len(fr.nodes)} nodes, {len(fr.elems)} elements")
        for r in rows:
            print("  ", json.dumps(r))
    fr, rows = press_table(E_TPU95A, fixed_extra="lug")
    print("with the lug pinned:")
    for r in rows:
        print("  ", json.dumps(r))
