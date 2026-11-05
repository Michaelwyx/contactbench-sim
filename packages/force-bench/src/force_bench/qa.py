"""Post-hoc rollout QA: NaN/velocity/envelope checks + geometric penetration
measurement with trimesh (solver-independent ground truth)."""

from __future__ import annotations

import json
import os
import re
from pathlib import Path

import numpy as np
import trimesh
import warp as wp

import newton


class RolloutQA:
    """Post-hoc checks on a recorded rollout.

    Penetration is measured geometrically with trimesh (independent of the
    solver): for each named body pair, sample frames and compute the largest
    interpenetration depth of vertices of A inside B (and vice versa).
    """

    def __init__(self, states_npz: str | Path):
        d = np.load(states_npz, allow_pickle=True)
        self.t = d["t"]
        self.body_q = d["body_q"]      # (T, nbody, 7)
        self.body_qd = d["body_qd"]    # (T, nbody, 6)
        self.labels = list(d["body_labels"])
        self.report: dict = {"checks": {}, "pass": True}

    def _idx(self, label: str) -> int:
        return self.labels.index(label)

    def check_finite(self):
        ok = bool(np.isfinite(self.body_q).all() and np.isfinite(self.body_qd).all())
        self.report["checks"]["finite"] = {"pass": ok}
        self.report["pass"] &= ok

    def check_velocity(self, vmax: float = 3.0):
        """Flying-object detector on frame-to-frame DISPLACEMENT velocity.

        Raw solver body_qd can show single-substep impulse spikes on impact
        that never move anything (positions stay bounded) — so the pass/fail
        criterion uses finite differences of body positions; the raw solver
        peak is reported as info only.
        """
        dt = np.diff(self.t)
        dt[dt <= 0] = np.inf
        disp = np.linalg.norm(np.diff(self.body_q[..., :3], axis=0), axis=-1)
        fd_vel = disp / dt[:, None]
        peak_fd = float(fd_vel.max()) if fd_vel.size else 0.0
        peak_body = self.labels[int(fd_vel.max(axis=0).argmax())] if fd_vel.size else None
        raw_peak = float(np.abs(self.body_qd).max())
        ok = bool(np.isfinite(peak_fd) and peak_fd < vmax)
        self.report["checks"]["velocity_spikes"] = {
            "pass": ok,
            "peak_displacement_velocity_m_s": round(peak_fd, 3),
            "worst_body": peak_body,
            "raw_solver_peak_info": round(raw_peak, 2),
            "note": "flying-object detector: frame-to-frame displacement speed",
        }
        self.report["pass"] &= ok

    def check_envelope(self, labels: list[str], bounds_lo, bounds_hi):
        """All listed bodies stay inside a world-space AABB the whole rollout."""
        lo, hi = np.asarray(bounds_lo), np.asarray(bounds_hi)
        bad = {}
        for lbl in labels:
            p = self.body_q[:, self._idx(lbl), :3]
            out = (p < lo).any() | (p > hi).any()
            if out.any():
                bad[lbl] = int(out.sum())
        ok = not bad
        self.report["checks"]["workspace_envelope"] = {"pass": ok, "violations": bad}
        self.report["pass"] &= ok

    def check_penetration(
        self,
        pairs: list[tuple[str, trimesh.Trimesh, str, trimesh.Trimesh]],
        tol: float = 0.0015,
        n_samples: int = 24,
    ):
        """pairs: (labelA, meshA_local, labelB, meshB_local) — meshes in body frame.

        Measures max depth of A's vertices inside B (via signed distance) on
        sampled frames. tol = allowed depth (solver contact slop), meters.
        """
        results = {}
        frames = np.linspace(0, len(self.t) - 1, n_samples).astype(int)
        worst_all = 0.0
        for la, ma, lb, mb in pairs:
            ia, ib = self._idx(la), self._idx(lb)
            worst = 0.0
            worst_t = None
            pq_b = trimesh.proximity.ProximityQuery(mb)
            va = ma.vertices
            if len(va) > 800:
                va = va[np.random.RandomState(0).choice(len(va), 800, replace=False)]
            for f in frames:
                qa_ = self.body_q[f, ia]
                qb_ = self.body_q[f, ib]
                Ta = trimesh.transformations.quaternion_matrix(
                    [qa_[6], qa_[3], qa_[4], qa_[5]])
                Ta[:3, 3] = qa_[:3]
                Tb = trimesh.transformations.quaternion_matrix(
                    [qb_[6], qb_[3], qb_[4], qb_[5]])
                Tb[:3, 3] = qb_[:3]
                # A's vertices in B's frame
                pts = (np.linalg.inv(Tb) @ Ta @ np.hstack(
                    [va, np.ones((len(va), 1))]).T).T[:, :3]
                sd = pq_b.signed_distance(pts)  # positive = inside
                depth = float(sd.max()) if len(sd) else 0.0
                if depth > worst:
                    worst, worst_t = depth, float(self.t[f])
            results[f"{la}->{lb}"] = {
                "max_depth_mm": round(worst * 1000, 3), "at_t": worst_t,
                "pass": bool(worst < tol),
            }
            worst_all = max(worst_all, worst)
        ok = all(r["pass"] for r in results.values())
        self.report["checks"]["penetration"] = {
            "pass": ok, "tolerance_mm": tol * 1000, "pairs": results,
        }
        self.report["pass"] &= ok

    def check_penetration_static(self, body_label: str, body_local: trimesh.Trimesh,
                                 static_tm: trimesh.Trimesh, static_center,
                                 pair_name: str, tol: float = 0.0015,
                                 n_samples: int = 24):
        """Max depth of a tracked body's vertices inside a STATIC mesh."""
        static_local = static_tm.copy()
        static_local.apply_translation(-np.asarray(static_center))
        pq = trimesh.proximity.ProximityQuery(static_local)
        bi = self._idx(body_label)
        va = body_local.vertices
        if len(va) > 800:
            va = va[np.random.RandomState(0).choice(len(va), 800, replace=False)]
        va_h = np.hstack([va, np.ones((len(va), 1))])
        frames = np.linspace(0, len(self.t) - 1, n_samples).astype(int)
        worst, worst_t = 0.0, None
        for f in frames:
            q = self.body_q[f, bi]
            T = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
            T[:3, 3] = q[:3] - np.asarray(static_center)
            pts = (T @ va_h.T).T[:, :3]
            sd = pq.signed_distance(pts)
            depth = float(sd.max()) if len(sd) else 0.0
            if depth > worst:
                worst, worst_t = depth, float(self.t[f])
        entry = {"max_depth_mm": round(worst * 1000, 3), "at_t": worst_t,
                 "pass": bool(worst < tol)}
        pen = self.report["checks"].setdefault(
            "penetration", {"pass": True, "tolerance_mm": tol * 1000, "pairs": {}})
        pen["pairs"][pair_name] = entry
        pen["pass"] = bool(pen["pass"] and entry["pass"])
        self.report["pass"] &= entry["pass"]

    def save(self, path: str | Path) -> dict:
        def _default(o):
            if isinstance(o, (np.bool_,)):
                return bool(o)
            if isinstance(o, (np.integer,)):
                return int(o)
            if isinstance(o, (np.floating,)):
                return float(o)
            raise TypeError(f"not serializable: {type(o)}")

        with open(path, "w") as f:
            json.dump(self.report, f, indent=1, default=_default)
        return self.report
