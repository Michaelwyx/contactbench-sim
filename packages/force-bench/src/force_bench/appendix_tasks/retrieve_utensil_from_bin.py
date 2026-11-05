"""ContactBench task 2 "Retrieve single utensil from bin" (paper Table C1): the scene, without a scripted policy.

    Description: A utensil storage bin is placed in the workspace.  The target utensil is placed in the bin
    alongside multiple other utensils.  Pick up a single copy of the target utensil from the cluttered storage bin
    and place it on the table without removing other objects from the bin.

Built from the kit library (``force_bench.kit``) in the website's three scene arrangements, with the counts of the
real-world videos.  The bin is filled by physics before the episode (not recorded): the utensils go in one at a time,
in a random order and at random places, and settle on the floor and on each other.  No utensil is placed for a grasp,
so the copies of the target lie anywhere in the pile, under or between the others.

  horizontal-chopsticks  "Chopsticks in horizontal bin": the 220 mm bin, the chopsticks lowered in lying along it,
                         each turned up to +-4 deg and let go at the lowest height where it keeps 1 mm from the floor,
                         the walls and the chopsticks already in; default 12.
  vertical-chopsticks    "Chopsticks in vertical bin": the 92 mm cup, the chopsticks dropped in handle first from just
                         above the rim where the column above the floor is most open, leaning as a bundle away from
                         the robot; default 14.
  vertical-mixed         "Mixed utensils in vertical bin": the cup head-up with knives, forks, spoons and chopsticks
                         (6 / 5 / 4 / 5 for the default 20, the video's; fewer with ``--n-objects``, same proportion),
                         dropped in the same way, wide heads first, the tall knives and chopsticks in its back half;
                         the default target is a spoon (the video's).

A utensil that misses the bin is put in again; one that still misses after three tries is taken out of the scene
and reported.

There is no scripted policy: the task is left to teleoperation (``force-bench teleop-ui --scene
retrieve_utensil_from_bin``) and to policies of your own.  ``force-bench run`` builds and fills the scene and records
it with the arm holding its start pose (``--hold`` seconds): ``rollout.mp4``, the states and contact forces, and
``scene.json`` with the arrangement, the composition, the fill, the start and the criterion's state at the end.

THE CRITERION'S STATE (``Task.criterion_state``; every frame as readouts, at the end in scene.json): the utensils
outside the bin (lowest point outside its inner footprint or more than 10 mm above the rim), and whether exactly one
utensil is outside and it is a copy of the target, at rest on the table and clear of the bin's footprint.  The
paper's criterion also asks that the robot has stopped and let go; the probe ``robot_object`` logs the robot's
contact force on the utensils and the bin.

EMBODIMENTS  --embodiment finray mounts the printed Fin Ray finger on both carriages (force_bench.finray,
docs/finray.md), in every arrangement.

OPTIONS  --arrangement, --target, --n-objects (the website's axes, see variations.AXES), --seed N (the bin's pose
uniform in a 0.15 x 0.15 m square and +-15 deg, and the fill; recorded in scene.json), --hold S, --name,
--embodiment.

Run: force-bench run retrieve_utensil_from_bin [--set "Scene arrangement=..."] [--seed N] [--embodiment finray]
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench import embodiments
from force_bench.appendix_tasks import _hang_plate as HP
from force_bench.appendix_tasks._hang_plate import JointTracker, quat_of_R
from force_bench.appendix_tasks.common import Placement, RobotContactProbe, appendix_rig, sample_start
from force_bench.appendix_tasks.stack_cups import Z0, build_bench
from force_bench.geometry import world_T
from force_bench.kit import add_asset, load_kit, pose_matrix

NAME = "retrieve_utensil_from_bin"
MM = 1000.0
TYPES = ("chopstick", "fork", "knife", "spoon")


@dataclass(frozen=True)
class Arrangement:
    key: str
    label: str                  # the task page's value of "Scene arrangement"
    bin: str                    # kit key
    n: int                      # default number of utensils in the bin (the video's count)
    n_min: int
    n_max: int
    target: str                 # default target type


ARRANGEMENTS = {
    "horizontal-chopsticks": Arrangement("horizontal-chopsticks", "Chopsticks in horizontal bin",
                                         "utensil-bin-horizontal", 12, 10, 14, "chopstick"),
    "vertical-chopsticks": Arrangement("vertical-chopsticks", "Chopsticks in vertical bin",
                                       "utensil-bin-vertical", 14, 2, 16, "chopstick"),
    "vertical-mixed": Arrangement("vertical-mixed", "Mixed utensils in vertical bin",
                                  "utensil-bin-vertical", 20, 4, 22, "spoon"),
}
# the video's mixed bin: at least 4 forks, 3 spoons and 6 knives, chopsticks behind (about 20)
MIX = (("knife", 6), ("fork", 5), ("spoon", 4), ("chopstick", 5))

# ── the cell ──────────────────────────────────────────────────────────────────────────────────────────────────────
BIN_XY = {"horizontal-chopsticks": np.array([0.000, 0.000]), "vertical-chopsticks": np.array([-0.030, 0.000]),
          "vertical-mixed": np.array([-0.030, 0.000])}
SEED_DXY = 0.075                 # paper protocol: uniform in a 0.15 x 0.15 m square at the board centre
SEED_DYAW = np.radians(15.0)     # and the bin turned up to +-15 deg from its nominal orientation
FILL_SEED = 20261029             # the fill's order and places without --seed (the nominal scene)
IN_BIN_ABOVE_RIM = 0.010         # "in the bin": lowest point inside the inner footprint, at most this above the rim
ON_TABLE_MAX = 0.002             # "on the table": its lowest point at most this above the table ...
AT_REST = 0.005                  # ... and slower than this (m/s)
STILL_WATCH_S = 0.5              # the fill has settled once nothing moved > 1 mm/s or turned > 0.5 deg/s over this long
HOLD_S = 3.0                     # recorded after the fill, the arm at its start pose

# what each arrangement changes against its real-world video (scene.json "changes_vs_real_task")
CHANGES = {
    "horizontal-chopsticks": [
        "kit horizontal bin; the chopsticks are lowered in one at a time at random places across it, each turned up to "
        "4 deg, and settle under gravity before the episode (physics, not recorded); in the video they lie packed in "
        "layers along the bin",
    ],
    "vertical-chopsticks": [
        "kit vertical bin; the chopsticks are dropped in one at a time before the episode (physics, not recorded) and "
        "lean as a bundle away from the robot",
    ],
    "vertical-mixed": [
        "kit vertical bin; knives, forks, spoons and chopsticks are dropped in head up, one at a time, before the "
        "episode (physics, not recorded); the clutter leans away from the robot",
    ],
}


def rot(axis, ang) -> np.ndarray:
    return trimesh.transformations.rotation_matrix(float(ang), np.asarray(axis, float))[:3, :3]


def unit(v) -> np.ndarray:
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def frame(z_axis, y_hint) -> np.ndarray:
    """Rotation whose z is ``z_axis`` and whose y is ``y_hint`` made orthogonal to it (a TCP frame: z = approach,
    y = closing axis)."""
    z = unit(z_axis)
    y = unit(np.asarray(y_hint, float) - z * float(np.dot(y_hint, z)))
    return np.column_stack([np.cross(y, z), y, z])


class Bin:
    """The kit bin: its pose and the inner footprint measured on the CAD."""

    def __init__(self, key: str, xy, yaw: float):
        tm = load_kit()[key].parts()[0].mesh()
        self.key, self.tm, self.yaw = key, tm, float(yaw)
        self.P = pose_matrix(xyz=(float(xy[0]), float(xy[1]), Z0), yaw=yaw)
        self.rim = float(tm.bounds[1][2])
        hits, _, _ = tm.ray.intersects_location([[0.0, 0.0, self.rim + 0.05]], [[0.0, 0.0, -1.0]])
        self.floor = float(hits[:, 2].max())
        self.outer = 0.5 * np.asarray(tm.extents[:2])
        self.inner_floor = np.array([self._inner(0, 0.003), self._inner(1, 0.003)])
        self.inner_rim = np.array([self._inner(0, self.rim - 0.001), self._inner(1, self.rim - 0.001)])

    def _inner(self, ax, z) -> float:
        d = np.zeros(3)
        d[ax] = 1.0
        loc, _, _ = self.tm.ray.intersects_location([[0.0, 0.0, z]], [d])
        return float(loc[:, ax].min())

    def half_width(self, ax: int, z: float) -> float:
        """Inner half-width along CAD axis ``ax`` at height z above the bin's base (linear in the draft)."""
        u = np.clip((z - 0.003) / (self.rim - 0.004), 0.0, 1.0)
        return float(self.inner_floor[ax] + u * (self.inner_rim[ax] - self.inner_floor[ax]))

    def to_bin(self, pts) -> np.ndarray:
        P = np.linalg.inv(self.P)
        return np.atleast_2d(pts) @ P[:3, :3].T + P[:3, 3]

    def to_world(self, pts) -> np.ndarray:
        return np.atleast_2d(pts) @ self.P[:3, :3].T + self.P[:3, 3]

    def contains_foot(self, p_world, tol=0.001) -> bool:
        """A utensil whose lowest point is this is in the bin: inside the inner footprint and at most IN_BIN_ABOVE_RIM
        above the rim (in a full cup a utensil may rest on the others with its end just above the rim)."""
        b = self.to_bin(p_world)[0]
        z = min(b[2], self.rim)
        return bool(abs(b[0]) < self.half_width(0, z) + tol and abs(b[1]) < self.half_width(1, z) + tol
                    and b[2] < self.rim + IN_BIN_ABOVE_RIM)

    def over_footprint(self, pts_world, grow=0.002) -> bool:
        b = self.to_bin(pts_world)
        return bool(np.any((np.abs(b[:, 0]) < self.outer[0] + grow) & (np.abs(b[:, 1]) < self.outer[1] + grow)))


class Utensil:
    """One utensil in the scene: kit key, label, its CAD mesh and the body frame (bbox centre) it was built with."""

    def __init__(self, key: str, label: str, P_cad: np.ndarray):
        self.key, self.label = key, label
        self.tm = load_kit()[key].parts()[0].mesh()
        lo, hi = self.tm.bounds
        self.c_cad = 0.5 * (lo + hi)
        self.P0 = np.asarray(P_cad, float)
        self.v = np.asarray(self.tm.vertices)[::max(1, len(self.tm.vertices) // 1200)]   # subsampled, CAD frame
        self.body = -1

    def body_pose(self, P_cad) -> np.ndarray:
        return P_cad @ pose_matrix(xyz=self.c_cad)

    def cad_pose(self, body_q_row) -> np.ndarray:
        return world_T(body_q_row) @ pose_matrix(xyz=-self.c_cad)

    def world(self, body_q_row, full=False) -> np.ndarray:
        T = self.cad_pose(body_q_row)
        V = np.asarray(self.tm.vertices) if full else self.v
        return V @ T[:3, :3].T + T[:3, 3]

    def section(self, x: float, axis: str):
        """Cross-section of the CAD mesh at x: centre (CAD) and half-width along ``axis`` ('y' or 'z')."""
        s = self.tm.section(plane_origin=[x, 0.0, 0.0], plane_normal=[1.0, 0.0, 0.0])
        V = np.vstack(s.discrete)
        k = "xyz".index(axis)
        return 0.5 * (V.min(0) + V.max(0)), 0.5 * float(V[:, k].max() - V[:, k].min())


# ── scene composition ──────────────────────────────────────────────────────────────────────────────────────────────
def composition(arr: Arrangement, n: int, target: str, rng) -> list[str]:
    """Kit keys of the n utensils, in the order they go into the bin; at least one is of the target's type.  Chopstick
    arrangements: n - 1 chopsticks and the target type (one chopstick more, or the one other utensil), in a random
    order.  Mixed bin: the video's proportions, the wide heads first (a spoon dropped onto heads already above the rim
    perches on them), the slim knives and chopsticks last, into the gaps."""
    if arr.key != "vertical-mixed":
        keys = ["chopstick"] * (n - 1) + [target]
        return [keys[i] for i in rng.permutation(n)]
    total = sum(c for _, c in MIX)
    raw = {k: c * n / total for k, c in MIX}
    cnt = {k: int(np.floor(v)) for k, v in raw.items()}
    for k in sorted(raw, key=lambda k: raw[k] - cnt[k], reverse=True)[:n - sum(cnt.values())]:
        cnt[k] += 1
    if cnt[target] == 0:                       # the target's type is always there: take one from the largest group
        cnt[max(cnt, key=cnt.get)] -= 1
        cnt[target] += 1
    return [k for k in ("spoon", "fork", "knife", "chopstick") for _ in range(cnt[k])]


def flat_rotation(key: str, flip: bool) -> np.ndarray:
    """CAD -> bin rotation of a utensil lying flat along the bin's x axis (``flip``: head toward -x)."""
    R = rot((1, 0, 0), -np.pi / 2) if key == "knife" else np.eye(3)   # the knife on its flat side, the others CAD z up
    return (rot((0, 0, 1), np.pi) @ R) if flip else R


def standing_rotation(lean_dir, tilt: float, spin: float) -> np.ndarray:
    """CAD -> bin rotation of a utensil standing head up (CAD +x up), turned ``spin`` about its axis, then tilted
    ``tilt`` toward the horizontal direction ``lean_dir``."""
    R_up = rot((0, 1, 0), -np.pi / 2)                          # CAD x -> +z, y -> y, z -> -x
    d = unit([lean_dir[0], lean_dir[1], 0.0])
    return rot(np.cross([0.0, 0.0, 1.0], d), tilt) @ rot((0, 0, 1), spin) @ R_up


def drop_pose(b: Bin, key: str, foot_xy, lean_dir, tilt: float, spin: float, above_rim: float = 0.004,
              above_floor: float | None = None) -> np.ndarray:
    """CAD pose (bin frame) of a utensil about to be dropped in: standing, its lowest point ``above_rim`` above the
    rim (or ``above_floor`` above the floor), over ``foot_xy``."""
    R = standing_rotation(lean_dir, tilt, spin)
    V = load_kit()[key].parts()[0].mesh().vertices @ R.T
    low = V[np.argmin(V[:, 2])]
    z = b.floor + above_floor if above_floor is not None else b.rim + above_rim
    return pose_matrix(xyz=(foot_xy[0] - low[0], foot_xy[1] - low[1], z - low[2]), R=R)


def tangential_spin(lean_dir) -> float:
    """Spin that puts the utensil's CAD y horizontal and across the lean (the end pinch closes along it)."""
    k = np.cross([0.0, 0.0, 1.0], unit([lean_dir[0], lean_dir[1], 0.0]))
    return float(np.arctan2(-k[0], k[1]))


def vertical_plan(arr: Arrangement, b: Bin, keys: list[str], rng) -> list[dict]:
    """Drop plan of a vertical bin (bin frame: +y toward the robot).  The chopsticks' feet go in the back half and they
    lean back (+-35 deg); in the mixed bin the tall knives and chopsticks stand in the back half (as in the video), the
    forks and spoons anywhere on the floor, and each leans outward and back from wherever its foot lands.  Each foot is
    placed where the column above it is most open when it is dropped (Task.dropper)."""
    hx, hy = b.inner_floor - 0.004
    mixed = arr.key == "vertical-mixed"
    if mixed:
        wide, tall = (-hx, hx, -hy, hy, []), (-hx, hx, -hy, 0.0, [])
    else:
        wide = tall = (-0.85 * hx, 0.85 * hx, -hy, 0.10 * hy, [])
    plan = []
    for key in keys:
        u = rng.uniform(-1.0, 1.0, 4)
        region = tall if key in ("knife", "chopstick") else wide
        foot = (region[0] + (region[1] - region[0]) * 0.5 * (u[0] + 1.0),
                region[2] + (region[3] - region[2]) * 0.5 * (u[1] + 1.0))
        ang = np.radians(20.0 if mixed else 35.0) * u[2]
        d = None if mixed else np.array([np.sin(ang), -np.cos(ang)])
        plan.append(dict(key=key, foot=foot, dir=d, jitter=ang, tilt=np.radians(5.0), region=region,
                         spin=(np.pi * u[3]) if key == "chopstick" else None, spin_off=np.radians(25.0) * u[3],
                         outward=hx))
    return plan


def drop_direction(p: dict, foot) -> tuple[np.ndarray, float]:
    """Lean direction and spin of a planned drop once its foot is known: outward and back from the foot unless the
    plan fixed them."""
    d = p["dir"]
    if d is None:
        base = unit([0.9 * foot[0] / p["outward"], -1.0])
        d = rot((0, 0, 1), p["jitter"])[:2, :2] @ base
    spin = p["spin"] if p.get("spin") is not None else tangential_spin(d) + p.get("spin_off", 0.0)
    return d, spin


class Task:
    def __init__(self, arrangement: str, target: str | None, n: int | None, seed: int | None, name: str | None,
                 embodiment: str = embodiments.DEFAULT):
        self.arr = ARRANGEMENTS[arrangement]
        embodiments.check(NAME, embodiment)
        self.embodiment = embodiment
        self.fingers = embodiments.load(embodiment)
        self.target = target or self.arr.target
        self.n = int(n or self.arr.n)
        if not self.arr.n_min <= self.n <= self.arr.n_max:
            raise SystemExit(f"--n-objects for {self.arr.label!r} is {self.arr.n_min}..{self.arr.n_max} "
                             f"(the bin's capacity in this model), not {self.n}")
        self.seed = seed
        self.name = name or NAME
        self.vertical = self.arr.bin == "utensil-bin-vertical"
        self.rng = np.random.default_rng(seed if seed is not None else FILL_SEED)

    # ── scene ──────────────────────────────────────────────────────────────────────────────────────────────────────
    def build(self):
        rig = appendix_rig(self.name)
        rig.collide_every_substep = True
        rig.solver_options["impratio"] = 10.0
        HP.setup_robot(rig)
        self.fingers.mount(rig)                 # the embodiment's fingers (the stock ones: nothing to do)
        build_bench(rig)
        self.rig = rig
        nominal = BIN_XY[self.arr.key]
        r0 = unit(HP.ROBOT_BASE[:2] - nominal)
        # vertical bin: its long inner axis (y) toward the robot; horizontal bin: its length across the robot's view
        yaw0 = np.arctan2(r0[1], r0[0]) + (-np.pi / 2 if self.vertical else np.pi / 2)
        start = sample_start({"bin": Placement(tuple(nominal), float(yaw0), SEED_DXY, SEED_DYAW)}, self.seed)
        bx, by, byaw = start["bin"]
        self.bin = Bin(self.arr.bin, (bx, by), byaw)
        self.r_hat = unit(HP.ROBOT_BASE[:2] - np.array([bx, by]))
        self.start = {"bin_xy_mm": [round(bx * MM, 1), round(by * MM, 1)], "bin_yaw_deg": round(np.degrees(byaw), 1),
                      "sampled": self.seed is not None}
        add_asset(rig, self.arr.bin, pose=self.bin.P, label="bin")
        keys = composition(self.arr, self.n, self.target, self.rng)
        self.items, seen = [], {}
        for k, key in enumerate(keys):                  # parked far away (free fall) until it goes into the bin
            self.items.append(Utensil(key, f"{key}_{seen.get(key, 0)}", pose_matrix(xyz=(3.0 + 0.4 * k, 3.0, 1.0))))
            seen[key] = seen.get(key, 0) + 1
        if self.vertical:
            self.drops = vertical_plan(self.arr, self.bin, keys, self.rng)
        for it in self.items:
            add_asset(rig, it.key, pose=it.P0, label=it.label)
        HP.filter_statics_and_ground(rig)
        for it in self.items:                   # arm links pass through the utensils; the hand and fingers do not
            bi = rig.builder.body_label.index(it.label)
            for b, lbl in enumerate(rig.builder.body_label):
                if str(lbl).startswith("fr3/") and not self.fingers.is_finger(lbl) and "link7" not in lbl:
                    for s in rig.builder.body_shapes.get(b, []):
                        for ts in rig.builder.body_shapes.get(bi, []):
                            rig.filter_pair(s, ts)
        self.active, self.left_out = list(self.items), []
        # the arm waits above the robot side of the bin, fingers down
        self.R_park = frame([0, 0, -1.0], [*self.r_hat, 0.0])
        self.p_park = np.array([bx + 0.17 * self.r_hat[0], by + 0.17 * self.r_hat[1], Z0 + 0.33])
        self.q_start = rig.solve_arm_start(self.p_park, quat_of_R(self.R_park))
        rig.builder.joint_q[7:9] = [0.03, 0.03]
        rig.builder.joint_target_q[7:9] = [0.03, 0.03]
        self.cam_t, self.cam_p = self.camera()
        # a contact sensor must exist when finalize() creates the contacts
        rig.finalize(probes=[self.items[0].label], cam_pos=self.cam_p, cam_target=self.cam_t)
        self.fingers.finalize(rig)
        HP.set_camera_fov(rig, self.cam_p, self.cam_t, 40.0 if self.vertical else 36.0)
        lbls = list(rig.model.shape_label)
        ushapes = [lbls[s] for it in self.items for s in rig.shapes[it.label]]
        rig.probes["padL"] = self.fingers.pad_probe(rig.model, "L", ushapes)
        rig.probes["padR"] = self.fingers.pad_probe(rig.model, "R", ushapes)
        rig.probes["robot_object"] = RobotContactProbe(rig.model, [it.label for it in self.items],
                                                       static_shapes=[lbls[s] for s in rig.shapes["bin"]])
        m = rig.model
        jc = m.joint_child.numpy()
        self.jq = {}
        for it in self.items:
            it.body = rig.body_index(it.label)
            j = int(np.flatnonzero(jc == it.body)[0])
            self.jq[it.label] = (int(m.joint_q_start.numpy()[j]), int(m.joint_qd_start.numpy()[j]))

    def camera(self):
        """The real videos' view: from across the board (opposite the robot), about 35 deg up."""
        c = np.array([*self.bin.P[:2, 3], Z0])
        side = np.cross([0.0, 0.0, 1.0], np.r_[self.r_hat, 0.0])
        if self.vertical:
            tgt = c + np.array([0.0, 0.0, 0.080]) + 0.05 * side
            dist, elev = 0.66, np.radians(30.0)
        else:
            tgt = c + np.array([*(-0.040 * self.r_hat), 0.015])
            dist, elev = 0.56, np.radians(52.0)          # steep enough to see into the 41.5 mm deep bin
        back = unit(-np.r_[self.r_hat, 0.0] + 0.25 * side)
        return tgt, tgt + dist * (np.cos(elev) * back + np.sin(elev) * np.array([0.0, 0.0, 1.0]))

    # ── state helpers ──────────────────────────────────────────────────────────────────────────────────────────────
    def bq(self):
        return self.rig.s0.body_q.numpy()

    def teleport(self, it: Utensil, P_cad_world: np.ndarray):
        s = self.rig.s0
        q, qd = s.joint_q.numpy(), s.joint_qd.numpy()
        qs, ds = self.jq[it.label]
        T = it.body_pose(P_cad_world)
        w = trimesh.transformations.quaternion_from_matrix(T)          # w, x, y, z
        q[qs:qs + 7] = [*T[:3, 3], w[1], w[2], w[3], w[0]]
        qd[ds:ds + 6] = 0.0
        s.joint_q.assign(q)
        s.joint_qd.assign(qd)
        newton.eval_fk(self.rig.model, s.joint_q, s.joint_qd, s)

    def lowest(self, it: Utensil, bq) -> np.ndarray:
        W = it.world(bq[it.body])
        return W[np.argmin(W[:, 2])]

    def motion(self, it: Utensil, bq0, bq1) -> tuple[float, float]:
        """Speed of its lowest point (m/s) and turn rate of its axis (rad/s) between two states STILL_WATCH_S apart."""
        v = float(np.linalg.norm(self.lowest(it, bq1) - self.lowest(it, bq0))) / STILL_WATCH_S
        a0, a1 = (it.cad_pose(q[it.body])[:3, 0] for q in (bq0, bq1))
        return v, float(np.arccos(np.clip(abs(a0 @ a1), -1.0, 1.0))) / STILL_WATCH_S

    # ── filling the bin (physics, before the episode, not recorded) ───────────────────────────────────────────────
    def preroll(self, schedule, settle: float, trk):
        """Physics with nothing recorded: teleport each (t, utensil, CAD pose or a function returning one) at its time,
        then settle.  It steps the solver itself (Rig.run's frame loop without the recorder and the probes), so the
        recording, and a teleoperation session, which takes Rig.run over, begin with the bin filled."""
        rig = self.rig
        todo = sorted(schedule, key=lambda s: s[0])
        newton.use_coord_layout_targets = True
        frame_dt = 1.0 / rig.fps
        sim_dt = frame_dt / rig.substeps
        every = rig.effective_collide_every_substep()
        hook = getattr(rig, "substep_hook", None)
        q_park = quat_of_R(self.R_park)
        t = 0.0
        for _ in range(int(((todo[-1][0] if todo else 0.0) + settle) * rig.fps)):
            while todo and todo[0][0] <= t:
                _, it, P = todo.pop(0)
                self.teleport(it, P() if callable(P) else P)
            trk.step(self.p_park, q_park, 0.03)
            if not every:
                rig.s0.clear_forces()
                rig.pipeline.collide(rig.s0, rig.contacts)
            for _ in range(rig.substeps):
                if every:
                    rig.s0.clear_forces()
                    rig.pipeline.collide(rig.s0, rig.contacts)
                rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, sim_dt)
                rig.s0, rig.s1 = rig.s1, rig.s0
                if hook is not None:
                    hook()
            t += frame_dt
        rig.solver.update_contacts(rig.contacts, rig.s0)

    def free_spots(self, planned, region) -> list[tuple[float, float]]:
        """Candidate spots (bin frame) for the next drop in ``region`` (x0, x1, y0, y1, keep-out boxes), best first:
        the vertical column above each, floor to 210 mm above the rim, farthest from the utensils already in the
        bin, with a pull toward the planned foot."""
        b = self.bin
        bq = self.bq()
        x0, x1, y0, y1, keep_out = region
        C = np.array([(x, y) for x in np.arange(x0, x1 + 1e-9, 0.0025) for y in np.arange(y0, y1 + 1e-9, 0.0025)
                      if not any(a <= x <= c and e <= y <= f for a, c, e, f in keep_out)])
        done = [o for o in self.items if o.label in self.dropped]
        P = np.vstack([b.to_bin(o.world(bq[o.body])) for o in done]) if done else np.zeros((0, 3))
        P = P[(P[:, 2] > b.floor) & (P[:, 2] < b.rim + 0.21), :2]
        d = np.sqrt(((C[:, None, :] - P[None, :, :]) ** 2).sum(-1)).min(1) if len(P) else np.full(len(C), 1.0)
        score = np.minimum(d, 0.012) - 0.2 * np.linalg.norm(C - np.asarray(planned), axis=1)
        return [tuple(c) for c in C[np.argsort(-score)]]

    def dropper(self, it: Utensil, p: dict):
        """The drop pose of ``it``, decided at the moment it is dropped (``Task.preroll`` calls it): the best spot
        where the whole utensil, as dropped, keeps 1 mm from everything already in the bin (a wide head teleported
        into another one would be thrown out of the scene); higher above the rim if no spot is free."""
        from scipy.spatial import cKDTree

        def pose():
            b = self.bin
            bq = self.bq()
            done = [o for o in self.items if o.label in self.dropped]
            tree = cKDTree(np.vstack([b.to_bin(o.world(bq[o.body])) for o in done])) if done else None
            spots = self.free_spots(p["foot"], p["region"])
            best = None
            for above in (0.004, 0.030, 0.060):
                for foot in spots[:40]:
                    d, spin = drop_direction(p, foot)
                    P = drop_pose(b, it.key, foot, d, p["tilt"], spin, above_rim=above)
                    if tree is None or tree.query(it.v @ P[:3, :3].T + P[:3, 3])[0].min() > 0.001:
                        best = P
                        break
                if best is not None:
                    break
            if best is None:                          # nothing free: the best column, as before
                d, spin = drop_direction(p, spots[0])
                best = drop_pose(b, it.key, spots[0], d, p["tilt"], spin)
            self.dropped.add(it.label)
            return b.P @ best
        return pose

    def lowered(self, it: Utensil):
        """The pose of ``it`` let go into the horizontal bin, decided when it goes in (``Task.preroll`` calls it):
        lying along the bin, flipped at random, turned up to +-4 deg, at a random place across and along it, its
        lowest point 2 mm above the floor or above the highest point of the utensils already in, whichever is higher,
        so that it starts clear of everything and falls onto the pile."""

        def pose():
            b, rng = self.bin, self.rng
            bq = self.bq()
            done = [o for o in self.items if o.label in self.dropped]
            top = max([float(b.to_bin(o.world(bq[o.body]))[:, 2].max()) for o in done], default=b.floor)
            for _ in range(400):
                R = rot((0, 0, 1), np.radians(4.0) * rng.uniform(-1.0, 1.0)) @ flat_rotation(it.key,
                                                                                         bool(rng.integers(2)))
                V = it.v @ R.T
                lo, hi = V.min(0), V.max(0)
                room = b.inner_floor - 0.5 * (hi - lo)[:2] - 0.001     # inside the walls down to the floor
                if np.any(room < 0.0):
                    continue
                xy = rng.uniform(-room, room) - 0.5 * (lo + hi)[:2]
                self.dropped.add(it.label)
                return b.P @ pose_matrix(xyz=(xy[0], xy[1], top + 0.002 - lo[2]), R=R)
            raise RuntimeError(f"[{self.name}] {it.label} does not fit along the bin")
        return pose

    def fill(self, trk) -> dict:
        b = self.bin
        self.dropped = set()
        mixed = self.arr.key == "vertical-mixed"
        if self.vertical:
            sched, t = [], 0.05
            for it, p in zip(self.items, self.drops):
                t += 0.45 if mixed else 0.24
                sched.append((t, it, self.dropper(it, p)))
        else:
            sched = [(0.05 + 0.30 * k, it, self.lowered(it)) for k, it in enumerate(self.items)]
        print(f"[{self.name}] filling the {b.key} with {len(self.items)} utensils")
        self.preroll(sched, 0.9, trk)
        report = {"order": [it.label for it in self.items], "redropped": []}
        for _ in range(3):                                        # anything that did not land in the bin: again
            bq = self.bq()
            out = [it for it in self.items if not b.contains_foot(self.lowest(it, bq))]
            if not out:
                break
            sched = []
            for k, it in enumerate(out):
                self.dropped.discard(it.label)
                if self.vertical:
                    p = dict(self.drops[self.items.index(it)])
                    u = self.rng.uniform(-1.0, 1.0, 2)
                    # the most open column anywhere on the floor
                    p.update(dir=None if mixed else unit([0.3 * u[0], -1.0]), tilt=np.radians(4.0),
                             foot=(0.5 * p["region"][0] * u[1], 0.0), jitter=0.0)
                    sched.append((0.05 + 0.3 * k, it, self.dropper(it, p)))
                else:
                    sched.append((0.05 + 0.3 * k, it, self.lowered(it)))
                report["redropped"].append(it.label)
            print(f"[{self.name}] putting {[it.label for it in out]} in again")
            self.preroll(sched, 0.8, trk)
        for _ in range(8):                                        # until nothing moves
            bq0 = self.bq()
            self.preroll([], STILL_WATCH_S, trk)
            bq = self.bq()
            speed, turn = np.max([self.motion(it, bq0, bq) for it in self.items], axis=0)
            if speed < 0.001 and turn < np.radians(0.5):
                break
        report.update(settled=bool(speed < 0.001 and turn < np.radians(0.5)),
                      max_speed_mm_s=round(float(speed) * MM, 2), max_turn_deg_s=round(float(np.degrees(turn)), 2))
        # a utensil that would not land in the bin after three tries is taken out of the scene (held still far away,
        # left out of the criterion's state) and reported
        self.left_out = [it for it in self.items if not b.contains_foot(self.lowest(it, bq))]
        for it in self.left_out:
            self.teleport(it, it.P0)
        self.active = [it for it in self.items if it not in self.left_out]
        report["left_out"] = [it.label for it in self.left_out]
        report["in_bin_after_fill"] = len(self.active)
        return report

    # ── the criterion's state ─────────────────────────────────────────────────────────────────────────────────────
    def criterion_state(self, state) -> dict:
        """The part of the paper's criterion the scene's state decides: which utensils are outside the bin, and whether
        exactly one is, a copy of the target, at rest on the table and clear of the bin's footprint."""
        b = self.bin
        bq, bqd = state.body_q.numpy(), state.body_qd.numpy()
        out = [it for it in self.active if not b.contains_foot(self.lowest(it, bq))]
        st = {"outside_bin": [it.label for it in out], "single_target_on_table": False}
        if len(out) == 1:
            it = out[0]
            W = it.world(bq[it.body], full=True)
            low, speed = float(W[:, 2].min() - Z0), float(np.linalg.norm(bqd[it.body][:3]))
            st.update(lowest_above_table_mm=round(low * MM, 2), speed_mm_s=round(speed * MM, 2),
                      over_bin_footprint=b.over_footprint(W))
            st["single_target_on_table"] = bool(it.key == self.target and low < ON_TABLE_MAX and speed < AT_REST
                                                and not st["over_bin_footprint"])
        return st

    # ── the episode: the arm holds its start pose ─────────────────────────────────────────────────────────────────
    def setup(self):
        """Build the scene and fill the bin; returns the rig, ready for a policy (the arm at its start pose, the
        joint tracker that holds it in ``self.trk``, the fill's record in ``self.fill_report``)."""
        self.build()
        self.trk = JointTracker(self.rig, self.q_start)
        self.fill_report = self.fill(self.trk)
        print(f"[{self.name}] fill: {self.fill_report}")
        return self.rig

    def run(self, hold: float = HOLD_S):
        rig, trk, fill = self.setup(), self.trk, self.fill_report
        q_park = quat_of_R(self.R_park)

        def control(t):
            trk.step(self.p_park, q_park, 0.03)
            for it in self.left_out:                                     # out of the scene, held still
                self.teleport(it, it.P0)

        def extra(state, t):
            st = self.criterion_state(state)
            return {"outside_bin": len(st["outside_bin"]), "single_target_on_table": int(st["single_target_on_table"])}

        rec = rig.run(hold, control, extra)
        rec.save("rollout", force_keys=["padL_N", "padR_N", "robot_object_N"])
        from force_bench.appendix_tasks.registry import CRITERIA
        bq = rig.s0.body_q.numpy()
        info = {"task": "Retrieve single utensil from bin", "number": 2, "scripted_policy": False,
                "success_criterion": CRITERIA[2], "arrangement": self.arr.label, "target": self.target,
                "n_objects": self.n, "embodiment": self.embodiment,
                "composition": {k: sum(1 for it in self.active if it.key == k) for k in TYPES},
                "seed": self.seed, "start": self.start, "fill": fill, "changes_vs_real_task": CHANGES[self.arr.key],
                "finite": bool(np.isfinite(bq).all()), "criterion_state_at_end": self.criterion_state(rig.s0)}
        path = rig.out_dir / "scene.json"
        path.write_text(json.dumps(info, indent=1))
        print(f"[{self.name}] scene -> {path}")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Retrieve single utensil from bin (paper task 2): the scene, filled by "
                                             "physics, without a scripted policy")
    ap.add_argument("--arrangement", choices=sorted(ARRANGEMENTS), default="horizontal-chopsticks")
    ap.add_argument("--target", choices=TYPES, default=None)
    ap.add_argument("--n-objects", type=int, default=None, help="utensils in the bin, the target's copies included")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--hold", type=float, default=HOLD_S, help="seconds recorded after the fill (default %(default)s)")
    ap.add_argument("--name", default=None, help="output folder name (default: the row name, with __<embodiment> "
                                                  "for a non-default embodiment)")
    embodiments.add_argument(ap, "retrieve_utensil_from_bin")
    a = ap.parse_args(argv)
    name = a.name or embodiments.run_name(NAME, a.embodiment)
    Task(a.arrangement, a.target, a.n_objects, a.seed, name, a.embodiment).run(a.hold)


if __name__ == "__main__":
    wp.init()
    main()
