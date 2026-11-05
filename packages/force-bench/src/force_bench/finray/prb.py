"""The pseudo-rigid-body (PRB) linkage of the Fin Ray finger.

The finger is a closed frame: a straight 1.8 mm contact wall, a back wall
(6.0 mm at the base, 2.7 mm near the tip), fifteen ribs (2.0 / 1.2 mm) and
a tip where the two walls meet.  The PRB keeps that topology with rigid
links and revolute joints about the finger's width axis:

  contact wall   C_0 .. C_n   a chain from the base block to the tip, a
                              joint at every rib (and in the middle of the
                              long segments near the tip)
  back wall      B_0 .. B_15  a chain from the base block to the tip, a
                              joint at every rib's outer end
  ribs           R_k = Ra_k + Rb_k   two halves: Ra hinged to the contact wall
                              at the rib root, Rb hinged to the back wall
                              at the rib's outer end, each with a spring
  loops          Ra_k == Rb_k   welded at mid-rib (a loop-closing FIXED
                              joint: one MuJoCo WELD equality under
                              SolverMuJoCo), so the rib is rigid between
                              its two compliant ends
                 C_n == B_m   the tip, welded (the tip is a solid wedge)
A pin (CONNECT) cannot carry a spring and a pinned rib end let the last
bays fold at 10 mm of indentation (668 N against the reference's 23 N at
the tip); welding the halves keeps a spring at every member end.

Spring rates (N.m/rad) come from the measured wall thicknesses and the TPU
modulus (an estimate, ``frame_fe.E_TPU95A``):
  wall joints     k = E' I / l     (I = w t^3 / 12 of that wall, l = the mean
                                    of the two adjacent link lengths; the
                                    discrete-rod bending rate; a root joint
                                    uses l = half its link: the wall is
                                    clamped in the base block)
  rib ends        k = C_RIB E' I_rib / L_rib at each end (6: a rib in the
                  bays' shear mode bends with its inflection at mid-length)
and the whole set is scaled by K_SCALE.  The theory values (K_SCALE = 1,
C_RIB = 6) already match the large-deformation frame reference (``frame_nl``,
2 mm beam elements of the same walls) on the cylinder press to ~10 % over the
upper three quarters of the finger (docs/finray.md); ``prb_cpu.calibrate()``
refits them if the geometry changes.

This module is numpy only; ``model.py`` turns the layout into Newton bodies.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from force_bench.finray.frame_fe import E_TPU95A, NU_TPU
from force_bench.finray.geometry import WIDTH, profile

MAX_CONTACT_LINK = 8.0    # mm: split longer contact-wall links (near the tip)
MAX_BACK_LINK = 8.0       # mm: same for the back wall (its last link is 13 mm)
C_RIB = 6.0               # rib end rate = C_RIB E'I/L at each end
K_SCALE = 1.0             # global factor, fitted in calibrate()


@dataclass
class Link:
    name: str
    kind: str                     # contact | back | rib
    p0: np.ndarray                # proximal joint (STEP mm, x-z)
    p1: np.ndarray                # distal end
    t: float                      # wall thickness (mm)
    parent: str | None            # None = the finger body (base block)
    k: float = 0.0                # spring rate of the joint at p0 (N.m/rad)

    @property
    def length(self) -> float:
        return float(np.linalg.norm(self.p1 - self.p0))


@dataclass
class Layout:
    links: list[Link] = field(default_factory=list)
    loops: list[tuple[str, str, np.ndarray]] = field(default_factory=list)  # (a, b, point): welds

    def by_name(self) -> dict[str, Link]:
        return {l.name: l for l in self.links}


def _ei(t_mm: float, E: float) -> float:
    """E'I of a wall of thickness t across the finger width, N.m^2."""
    Ep = E * 1e6 / (1.0 - NU_TPU ** 2)
    return Ep * (WIDTH * 1e-3) * (t_mm * 1e-3) ** 3 / 12.0


def layout(E: float = E_TPU95A, k_scale: float = K_SCALE, c_rib: float = C_RIB,
           rib_stride: int = 1, max_contact_link: float = MAX_CONTACT_LINK,
           max_back_link: float = MAX_BACK_LINK) -> Layout:
    p = profile()
    tc = p["contact_wall_t"]
    xc = p["contact_mid_x"]
    tip = np.asarray(p["tip"], float)
    ribs = p["ribs"][rib_stride - 1::rib_stride]
    mid = np.asarray(p["back_mid"])
    bt = np.asarray(p["back_t"])
    L = Layout()
    # ── contact wall: nodes at the rib roots, long links split ──────────
    zs = [p["base_z"]] + [r["inner"][1] for r in ribs] + [tip[1]]
    nodes, rib_node = [], {}
    for i, (z0, z1) in enumerate(zip(zs[:-1], zs[1:])):
        n = max(1, int(np.ceil(abs(z1 - z0) / max_contact_link)))
        for k in range(n):
            nodes.append(z0 + (z1 - z0) * k / n)
        if i < len(ribs):
            rib_node[i] = len(nodes)          # the next node is rib i's root
    nodes.append(zs[-1])
    cpts = [np.array([xc, z]) for z in nodes]
    for j in range(len(cpts) - 1):
        L.links.append(Link(f"c{j:02d}", "contact", cpts[j], cpts[j + 1], tc,
                            None if j == 0 else f"c{j - 1:02d}"))
    # ── back wall: nodes at the rib outer ends, long links split ─────────
    bnodes = [mid[0]] + [np.asarray(r["outer"], float) for r in ribs] + [tip]
    bpts, rib_bnode = [], {}
    for i, (a, b) in enumerate(zip(bnodes[:-1], bnodes[1:])):
        n = max(1, int(np.ceil(np.linalg.norm(b - a) / max_back_link)))
        for k in range(n):
            bpts.append(a + (b - a) * k / n)
        if i < len(ribs):
            rib_bnode[i] = len(bpts)           # the next node is rib i's outer end
    bpts.append(bnodes[-1])

    def t_back(a, b):
        z = 0.5 * (a[1] + b[1])
        return float(np.interp(-z, -mid[:, 1], bt))

    for k in range(len(bpts) - 1):
        L.links.append(Link(f"b{k:02d}", "back", bpts[k], bpts[k + 1], t_back(bpts[k], bpts[k + 1]),
                            None if k == 0 else f"b{k - 1:02d}"))
    # ── ribs: two halves, hinged at both walls, welded at mid-rib ────────
    for i, r in enumerate(ribs):
        root = cpts[rib_node[i]]
        out = bpts[rib_bnode[i]]
        midp = 0.5 * (root + out)
        L.links.append(Link(f"ra{i + 1:02d}", "rib", root, midp, r["t"], f"c{rib_node[i]:02d}"))
        L.links.append(Link(f"rb{i + 1:02d}", "rib", out, midp, r["t"], f"b{rib_bnode[i]:02d}"))
        L.loops.append((f"ra{i + 1:02d}", f"rb{i + 1:02d}", midp))
    L.loops.append((f"c{len(cpts) - 2:02d}", f"b{len(bpts) - 2:02d}", tip))
    # ── spring rates ──────────────────────────────────────────────────────
    by = L.by_name()
    for l in L.links:
        if l.kind == "rib":
            l.k = k_scale * c_rib * _ei(l.t, E) / (2.0 * l.length * 1e-3)   # rib length = 2 halves
            continue
        if l.parent is None:
            lm = 0.5 * l.length
            t = l.t
        else:
            pl = by[l.parent]
            lm = 0.5 * (pl.length + l.length)
            t = 0.5 * (pl.t + l.t)
        l.k = k_scale * _ei(t, E) / (lm * 1e-3)
    return L


def link_masses(L: Layout, density: float = 1210.0) -> dict[str, float]:
    """kg, from wall thickness x length x width (the profile area is 1824 mm^2,
    the links' sum ~1700 mm^2: the base block carries the rest)."""
    return {l.name: density * (l.t * l.length * WIDTH) * 1e-9 for l in L.links}


if __name__ == "__main__":
    L = layout()
    m = link_masses(L)
    print(f"{len(L.links)} links, {len(L.loops)} loops, total link mass {sum(m.values()) * 1000:.1f} g")
    for l in L.links:
        print(f"  {l.name} {l.kind:7s} parent {str(l.parent):5s} len {l.length:6.2f} t {l.t:4.2f} "
              f"k {l.k:8.4f} N.m/rad  m {m[l.name] * 1000:5.2f} g")
