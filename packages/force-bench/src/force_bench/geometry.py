"""Small geometry helpers that several tasks used to carry as identical copies.

Only AST-identical copies were folded in here; the tasks whose ``subdiv`` /
``smooth`` / ``PairProbe`` differ keep their own, because those differences are
task-tuned (see the ``README`` note on duplicated helpers).
"""

from __future__ import annotations

import numpy as np
import trimesh


def smooth(u):
    u = float(np.clip(u, 0.0, 1.0))
    return 3 * u * u - 2 * u ** 3


def subdiv(tm: trimesh.Trimesh, max_edge: float) -> trimesh.Trimesh:
    """Subdivide to `max_edge` (<= 8 iterations), no trimesh post-processing."""
    v, f = trimesh.remesh.subdivide_to_size(tm.vertices, tm.faces,
                                            max_edge=max_edge, max_iter=8)
    return trimesh.Trimesh(v, f, process=False)



def subdiv_processed(tm: trimesh.Trimesh, max_edge: float) -> trimesh.Trimesh:
    """Subdivide to `max_edge` with no iteration cap and trimesh processing on.

    Contact sampling uses mesh VERTICES vs the counterpart SDF; large
    faces tunnel through thin walls (README pitfall 3.2)."""
    v, f = trimesh.remesh.subdivide_to_size(tm.vertices, tm.faces,
                                            max_edge=max_edge)
    return trimesh.Trimesh(v, f)


def world_T(q7):
    T = trimesh.transformations.quaternion_matrix([q7[6], q7[3], q7[4], q7[5]])
    T[:3, 3] = q7[:3]
    return T



def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_inv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])
