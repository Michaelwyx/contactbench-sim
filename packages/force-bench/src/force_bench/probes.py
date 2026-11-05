"""ForceProbe: a rolling force log around newton.sensors.SensorContact, the
same probe the rollout QA gates and the task force profiles read."""

from __future__ import annotations

import numpy as np

import newton


class ForceProbe:
    """Rolling force log for one sensing body or shape."""

    def __init__(self, model: newton.Model, sensing_body: str | None = None,
                 sensing_shape: str | None = None, counterpart_bodies=None):
        kwargs = {}
        if sensing_shape is not None:
            kwargs["sensing_shapes"] = [sensing_shape]
        else:
            kwargs["sensing_bodies"] = [sensing_body]
        self.sensor = newton.sensors.SensorContact(
            model,
            counterpart_bodies=counterpart_bodies,
            measure_total=True,
            **kwargs,
        )
        self.history: list[dict] = []

    def sample(self, contacts, state, sim_time: float) -> dict:
        self.sensor.update(state, contacts)
        total = self.sensor.total_force.numpy()[0]
        friction = self.sensor.total_force_friction.numpy()[0]
        rec = {
            "t": sim_time,
            "force": np.linalg.norm(total),
            "friction": np.linalg.norm(friction),
            # Decompose vectors before taking their magnitudes. This is
            # the normal resultant, not the sum of per-contact pressures.
            "normal": float(np.linalg.norm(total - friction)),
            "vec": total.tolist(),
        }
        self.history.append(rec)
        return rec

    def summary(self, t_start: float = 0.0) -> dict:
        recs = [r for r in self.history if r["t"] >= t_start]
        if not recs:
            return {}
        f = np.array([r["force"] for r in recs])
        fr = np.array([r["friction"] for r in recs])
        return {
            "samples": len(recs),
            "force_mean": float(f.mean()),
            "force_max": float(f.max()),
            "friction_mean": float(fr.mean()),
            "friction_max": float(fr.max()),
        }
