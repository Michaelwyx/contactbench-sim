"""The ``finray`` embodiment: the Fin Ray finger on both carriages of the FR3's Franka Hand.

``FinRayFingers`` implements the hooks of :class:`force_bench.embodiments.fingers.Fingers` with the models of this
package: ``mount`` switches the stock finger shapes off and builds the printed adapter, the finger's base block and
the pseudo-rigid-body linkage on each carriage (``model.mount_on_fr3``), and sets the solver needs (MuJoCo
``impratio`` 10, collision every substep); ``finalize`` stiffens the linkage's loop welds in the built solver;
``pad_probe`` senses every body of a finger; ``boxes`` is the finger's envelope at rest, from the same link boxes
the simulation collides with; ``checks`` adds the deformation gate (``finray_deformation``).

Geometry (TCP frame = ``fr3_hand_tcp``): the contact face is the stock pad plane (``face_offset`` 0: the faces are
2 q apart), the finger is 25.8 mm wide across the closing axis, and the linkage's lowest point (the corner of its
last back-wall link, at the tip) is ``tip_below_tcp`` = 100.80 mm beyond the TCP.  The printed finger's tip is
0.96 mm lower (the linkage's last links end at the walls' centre-line junction; the solid tip below it is not
modelled).
"""

from __future__ import annotations

from functools import lru_cache

import numpy as np

from force_bench.embodiments.fingers import FINGER_LINKS, Fingers
from force_bench.finray import geometry as G
from force_bench.finray import prb
from force_bench.finray.model import ADAPTER_BOXES_MM, CHEEK_BOXES_MM, FinRayParams

FINGER_TO_TCP = (G.HAND_TO_TCP_Z - G.HAND_TO_FINGER_Z) / 1000.0     # finger frame z - this = TCP frame z (m)

# The deformation gate (verdict check ``finray_deformation``), over the linkage's joints and its tip (the tip's
# displacement from its printed position in the carriage frame):
#   free   fingers open, not holding anything: the finger is back to its printed shape.  Past 3 mm at the tip (6-12 N
#          on the tip's 2-4 N/mm) or 8 deg at one joint (a 1 mm dent across 7 mm) it is being driven into
#          something that does not give
#   grasp  closed on the part: it is meant to bend, but not past 5 mm at the tip (~9 N at 1.8 N/mm)
FREE_TIP_MM, FREE_HINGE_DEG, GRASP_TIP_MM = 3.0, 8.0, 5.0


def _box_corners(c, half, ang) -> np.ndarray:
    """Corners (finger frame, m) of a box centred at c, half extents ``half``, turned ``ang`` about x."""
    s = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * half
    ca, sa = np.cos(ang), np.sin(ang)
    R = np.array([[1.0, 0.0, 0.0], [0.0, ca, -sa], [0.0, sa, ca]])
    return np.asarray(c) + s @ R.T


@lru_cache(maxsize=None)
def rest_points() -> np.ndarray:
    """Corners of every colliding box of one finger at rest (adapter, base block, contact and back walls; the ribs
    do not collide), finger frame, m."""
    pts = []
    for xh, y0, y1, z0, z1 in ADAPTER_BOXES_MM:
        pts.append(_box_corners([0.0, 0.5 * (y0 + y1) / 1e3, 0.5 * (z0 + z1) / 1e3],
                                np.array([xh, 0.5 * (y1 - y0), 0.5 * (z1 - z0)]) / 1e3, 0.0))
    for s, y0, y1, z0, z1 in CHEEK_BOXES_MM:
        xc = s * (G.WIDTH / 2 + G.CHEEK_CLEAR + G.CHEEK / 2)
        pts.append(_box_corners([xc / 1e3, 0.5 * (y0 + y1) / 1e3, 0.5 * (z0 + z1) / 1e3],
                                np.array([G.CHEEK / 2, 0.5 * (y1 - y0), 0.5 * (z1 - z0)]) / 1e3, 0.0))
    pts.append(_box_corners([0.0, G.DEPTH / 2e3, (G.Z_BASE + G.BASE_H / 2) / 1e3],
                            np.array([G.WIDTH / 2, G.DEPTH / 2, G.BASE_H / 2]) / 1e3, 0.0))
    for link in prb.layout().links:
        if link.kind == "rib":
            continue
        p0 = np.array([link.p0[0], G.Z_BASE - link.p0[1]]) / 1e3          # (y, z) finger frame
        p1 = np.array([link.p1[0], G.Z_BASE - link.p1[1]]) / 1e3
        d = p1 - p0
        pts.append(_box_corners([0.0, *(0.5 * (p0 + p1))], np.array([G.WIDTH / 2e3, link.t / 2e3, link.length / 2e3]),
                                float(np.arctan2(-d[0], d[1]))))
    return np.vstack(pts)


UNDERSIDE_STEP = 0.00005          # m, the underside table's resolution across the finger


@lru_cache(maxsize=None)
def underside_table() -> tuple[np.ndarray, float]:
    """The finger's rest underside: for y = 0, 0.05, ... mm outward from the contact face, the height (m) of its
    lowest material above the fingertip's lowest point (inf where there is none), from the same boxes the simulation
    collides with (contact and back walls, base block); and the height of the flat contact face's lower end above the
    tip (the last contact-wall link ends where the two walls meet, above the back wall's lowest corner)."""
    pts = []
    rects = []
    for link in prb.layout().links:
        if link.kind == "rib":
            continue
        rects.append((np.array([link.p0[0], G.Z_BASE - link.p0[1]]), np.array([link.p1[0], G.Z_BASE - link.p1[1]]),
                      link.t))
    rects.append((np.array([G.DEPTH / 2, G.Z_BASE]), np.array([G.DEPTH / 2, G.Z_BASE + G.BASE_H]), G.DEPTH))
    for p0, p1, t in rects:                                    # (y, z) mm, sampled on a 0.05 mm grid
        d = p1 - p0
        length = float(np.linalg.norm(d))
        u = d / length
        n = np.array([-u[1], u[0]])
        a, b = np.meshgrid(np.linspace(0.0, length, max(2, int(length / 0.05) + 1)),
                           np.linspace(-t / 2, t / 2, max(2, int(t / 0.05) + 1)))
        pts.append(p0 + a.ravel()[:, None] * u + b.ravel()[:, None] * n)
    P = np.vstack(pts) / 1000.0
    tip = float(P[:, 1].max())
    n = int(round(G.DEPTH / 1000.0 / UNDERSIDE_STEP)) + 1
    low = np.full(n, -np.inf)
    k = np.round(P[:, 0] / UNDERSIDE_STEP).astype(int)
    ok = (k >= 0) & (k < n)
    np.maximum.at(low, k[ok], P[ok, 1])
    face = float(P[P[:, 0] < UNDERSIDE_STEP / 2, 1].max())
    return np.where(np.isfinite(low), tip - low, np.inf), tip - face


class FinRayFingers(Fingers):
    """Fin Ray fingers (PRB linkage) on the FR3's carriages."""

    face_offset = 0.0
    tip_half_width = G.WIDTH / 2000.0

    def __init__(self, info, params: FinRayParams | None = None):
        super().__init__(info)
        self.params = params or FinRayParams(model="prb")
        P = rest_points()
        self.tip_below_tcp = float(P[:, 2].max()) - FINGER_TO_TCP
        self.tip_top = FINGER_TO_TCP - float(P[:, 2].min())          # the adapter's top, behind the TCP
        # thickness along the closing axis of the finger's lowest 6 mm (the room it needs beside a part)
        low = P[P[:, 2] > P[:, 2].max() - 0.006]
        self.tip_thickness = float(low[:, 1].max())
        self.face_bottom_above_tip = underside_table()[1]
        self.fingers = None

    # ── building ──────────────────────────────────────────────────────────────────────────────────────────────────
    def mount(self, rig) -> None:
        from force_bench.finray.model import mount_on_fr3
        self.params.substeps, self.params.fps = int(rig.substeps), int(rig.fps)
        self.fingers = mount_on_fr3(rig.builder, self.params)
        rig.solver_options.update(self.info.solver)
        if self.info.collide_every_substep:
            rig.collide_every_substep = True

    def finalize(self, rig) -> None:
        from force_bench.finray.model import tune_solver
        rig.model._finray_fingers = self.fingers
        n = tune_solver(rig.solver, self.params)
        expected = sum(len(f.loop_joints) for f in self.fingers)
        if n != expected:
            raise RuntimeError(f"Fin Ray: {n} of {expected} loop welds found in the solver (the linkage would be "
                               f"held together by MuJoCo's default 20 ms equality springs)")

    def is_finger(self, label: str) -> bool:
        return "finger" in str(label) or "finray" in str(label)

    def finger_bodies(self, model, side: str) -> list[int]:
        labels = list(model.body_label)
        return [labels.index(FINGER_LINKS[side])] + [i for i, lbl in enumerate(labels)
                                                     if str(lbl).startswith(f"fr3/finray_{side}_")]

    def pad_probe(self, model, side: str, counterpart_shapes):
        return FingerProbe(model, self.finger_bodies(model, side), counterpart_shapes)

    # ── planning ──────────────────────────────────────────────────────────────────────────────────────────────────
    def boxes(self, q: float) -> list[tuple[np.ndarray, np.ndarray]]:
        """The rest envelope of both fingers opened to q, in slabs along the approach (the back flank widens from
        1.3 mm at the tip to 40 mm at the base)."""
        P = rest_points()
        z = P[:, 2] - FINGER_TO_TCP
        cuts = self.tip_below_tcp - np.array([0.0, 0.004, 0.010, 0.020, 0.035, 0.060, 0.090])
        cuts = np.r_[cuts, z.min()]
        out = []
        for s in (+1.0, -1.0):
            for z_hi, z_lo in zip(cuts[:-1], cuts[1:]):
                # the slab's extent: the finger outline is piecewise linear between box corners, so take every
                # corner within the slab and the next ones beyond it on both sides
                m = (z >= z_lo - 0.008) & (z <= z_hi + 0.008)
                y_out = float(P[m, 1].max())
                xh = float(np.abs(P[m, 0]).max())
                y0, y1 = sorted([s * q, s * (q + y_out)])
                out.append((np.array([-xh, y0, z_lo]), np.array([xh, y1, z_hi])))
        return out

    def underside(self, y):
        h, _ = underside_table()
        k = np.round(np.asarray(y, float) / UNDERSIDE_STEP).astype(int)
        return np.where((k >= 0) & (k < len(h)), h[np.clip(k, 0, len(h) - 1)], np.inf)

    # ── verdict ───────────────────────────────────────────────────────────────────────────────────────────────────
    def checks(self, states_npz, windows) -> dict:
        from force_bench.finray.deform import Deformation
        d = np.load(states_npz, allow_pickle=True)
        D = Deformation(d["body_labels"])
        s = D.series(d["body_q"])
        t = np.asarray(d["t"])

        def peak(key, spans):
            m = np.zeros(len(t), bool)
            for t0, t1 in spans:
                m |= (t >= t0) & (t <= t1)
            if not m.any():
                return 0.0
            return round(max(float(s[side][key][m].max()) for side in s), 2)

        free, grasp = windows.get("free", []), windows.get("grasp", [])
        row = {"free_tip_mm": peak("tip_mm", free), "free_hinge_deg": peak("hinge_deg", free),
               "grasp_tip_mm": peak("tip_mm", grasp), "grasp_hinge_deg": peak("hinge_deg", grasp),
               "grasp_wall_deg": peak("wall_deg", grasp), "grasp_face_tilt_deg": peak("face_tilt_deg", grasp)}
        ok = (row["free_tip_mm"] <= FREE_TIP_MM and row["free_hinge_deg"] <= FREE_HINGE_DEG
              and row["grasp_tip_mm"] <= GRASP_TIP_MM)
        return {"finray_deformation": {
            "pass": bool(ok), **row,
            "limits": {"free_tip_mm": FREE_TIP_MM, "free_hinge_deg": FREE_HINGE_DEG, "grasp_tip_mm": GRASP_TIP_MM},
            "windows_s": {k: [[round(a, 2), round(b, 2)] for a, b in v] for k, v in windows.items()},
            "note": "largest over both fingers: tip = the tip's displacement from its printed position in the "
                    "carriage frame; hinge = the largest linkage-joint rotation; wall = the contact wall's; face "
                    "tilt = the lowest contact link's tilt"}}


class FingerProbe:
    """Pad probe of a multi-body finger: the vector sum of the contact forces on every body of the finger from the
    counterpart shapes (the stock probe's quantity, summed over the linkage)."""

    def __init__(self, model, bodies: list[int], counterpart_shapes):
        import newton
        labels = list(model.shape_label)
        shapes = [labels.index(s) if isinstance(s, str) else int(s) for s in counterpart_shapes]
        self.sensor = newton.sensors.SensorContact(model, sensing_bodies=list(bodies), counterpart_shapes=shapes,
                                                   measure_total=False)
        self.history: list[dict] = []

    def sample(self, contacts, state, t):
        self.sensor.update(state, contacts)
        F = self.sensor.force_matrix.numpy().sum(axis=(0, 1))
        Ff = self.sensor.force_matrix_friction.numpy().sum(axis=(0, 1))
        rec = {"t": t, "force": float(np.linalg.norm(F)), "friction": float(np.linalg.norm(Ff)),
               "normal": float(np.linalg.norm(F - Ff)), "vec": F.tolist()}
        self.history.append(rec)
        return rec
