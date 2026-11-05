"""Scenes for the teleop UIs: any ContactBench task (``force-bench list``) or a playground.

A task is captured the way the acceptance gate captures it: its ``main()``
is run with ``Rig.run`` patched, so the task builds and finalizes its scene
exactly as published (assets, drives, filters, start pose, solver options,
camera, an object already in hand) and hands the rig over the moment it
would have started its scripted policy.  The policy never runs; the
operator takes the controls at the task's own start pose.  A task takes the
arguments of ``force-bench run`` (``--seed``, ``--set``, ``--embodiment``)
and resolves them the same way.  No task file is read or modified.

Mechanisms a task adds to the physics (thread models, socket torques) wrap
the solver, so they step with the operator's session as well.

What the operator gets from the task for free:

* the task's ``extra`` readouts (door angle, plug depth, the utensils
  outside the bin, ...) sampled every frame and shown in the UI;
* guard zones on every task object, sized by the feature clearances in
  ``force_bench.teleop.gate.TASKS`` where the task's scene declares them
  and by the hard-surface standoff for the kit's parts, which set the
  guard's speed / lead limits region by region.

The ``playground`` is the ``force_bench.teleop.franka_teleop`` scene (a card
reader, a free card and a charger plug) rebuilt on the Rig.
"""

from __future__ import annotations

import importlib
import time
from dataclasses import dataclass

import numpy as np
import warp as wp

from force_bench.rig import Rig
from force_bench.teleop import gate as G
from force_bench.teleop.guard import FeatureMap

# the scenes ``--verify-all`` and ``--record-all`` run: name -> what it is (results in
# outputs/teleop_ui/<scene>/verify.json)
VERIFIED: dict[str, str] = {
    "playground": "card reader, free card and charger plug, the sandbox scene (CUDA graph)",
    "plug_in_usb_2": "task 26: USB stick 2 in the pinch above the kit's USB slot (CUDA graph)",
}


@dataclass
class Scene:
    name: str
    rig: Rig
    fmap: FeatureMap
    features: dict
    coverage: dict
    tcp0: np.ndarray
    q0: np.ndarray
    grip0: float
    cam_pos: np.ndarray
    cam_target: np.ndarray
    env_lo: np.ndarray
    env_hi: np.ndarray
    task_extra: object = None
    finger_effort_N: float | None = None
    note: str = ""
    build_s: float = 0.0
    verified: bool = False
    args: tuple = ()                # force-bench run options the task was built with (--seed, --set, ...)
    hold_start_pinch: bool = False  # the guard keeps the pinch the task starts with (TeleopSession.settle)

    def info(self) -> dict:
        return {"name": self.name, "note": self.note, "verified": self.verified,
                "bodies": int(self.rig.model.body_count),
                "shapes": int(self.rig.model.shape_count),
                "substeps": int(self.rig.substeps),
                "collide_every_substep": bool(self.rig.effective_collide_every_substep()),
                "features": {k: v for k, v in self.features.items()},
                "coverage": self.coverage,
                "readouts": self.task_extra is not None,
                "finger_effort_N": self.finger_effort_N,
                "task_args": list(self.args),
                "build_s": round(self.build_s, 1)}


def task_names() -> list[str]:
    """The ContactBench tasks, in ``force-bench list`` order."""
    from force_bench.appendix_tasks.registry import APPENDIX
    return [s.name for s in sorted(APPENDIX, key=lambda s: (s.number, s.name))]


def names() -> list[str]:
    return task_names() + ["playground"]


def task_note(name: str, start: bool = True) -> str:
    """A ContactBench task's number, title, variant and (``start``) start state."""
    from force_bench.appendix_tasks.registry import BY_NAME
    s = BY_NAME[name]
    note = f"task {s.number}: {s.appendix_task}" + (f" ({s.variant})" if s.variant else "")
    return note + (f"; {s.start}" if start and s.start else "")


# ── capturing a registered task ─────────────────────────────────────────
class _Captured(Exception):
    def __init__(self, rig, control_cb, extra_cb):
        self.rig, self.control_cb, self.extra_cb = rig, control_cb, extra_cb


def _capture_task(task: str, args: tuple[str, ...] = ()):
    from force_bench.appendix_tasks.registry import BY_NAME as TASKS
    from force_bench.cli import _run_plan
    spec = TASKS.get(task)
    if spec is None:
        raise SystemExit(f"unknown task {task!r}; `force-bench teleop-ui --list-scenes`")
    # the module arguments `force-bench run <task> <args>` would use (--seed, --set, --embodiment)
    task_argv, _ = _run_plan(task, spec, list(args))
    mod = importlib.import_module(spec.dotted)

    def patched(self, duration, control_cb, extra_forces_cb=None, print_every=20):
        raise _Captured(self, control_cb, extra_forces_cb)

    originals = [(Rig, Rig.run)]
    Rig.run = patched
    for obj in list(vars(mod).values()):
        if (isinstance(obj, type) and issubclass(obj, Rig) and obj is not Rig
                and "run" in obj.__dict__):
            originals.append((obj, obj.__dict__["run"]))
            obj.run = patched
    import sys
    argv = sys.argv
    sys.argv = [spec.dotted, *task_argv]
    try:
        mod.main()
    except _Captured as c:
        return c
    finally:
        sys.argv = argv
        for cls, fn in originals:
            cls.run = fn
    raise RuntimeError(f"{task}: main() returned without calling Rig.run")


FEATURE_HARD_SURFACE = 0.0015
"""A wall or table has no clearance; the "feature" is the standoff a press
may consume (docs/teleop_guard.md).  1.5 mm caps the approach at ~22 mm/s
and the lead at the governor's 3 mm floor, which keeps a fingertip landing
on a rigid slab to tens of newtons instead of the hundreds a 250 mm/s
free-air approach produces at ~400 N/mm."""


def add_table_zone(rig, fmap: FeatureMap, above_m: float = 0.05) -> bool:
    """Declare the scene's table (``Rig.table``'s box) as a hard surface: the
    zone spans the slab's footprint from below it to ``above_m`` above it."""
    m = rig.model
    sids = rig.shapes.get("table")
    if not sids:
        try:
            sids = [i for i, l in enumerate(list(m.shape_label)) if l == "table"]
        except Exception:
            sids = []
    if not sids:
        return False
    xf = m.shape_transform.numpy()[sids[0]]
    sc = m.shape_scale.numpy()[sids[0]]
    c = np.asarray(xf[:3], dtype=float)
    h = np.asarray(sc[:3], dtype=float)
    fmap.add_aabb("table", c - h - np.array([0, 0, 0.02]), c + h + np.array([0, 0, above_m]),
                  FEATURE_HARD_SURFACE, pad=0.0)
    return True


def add_ground_zone(rig, fmap: FeatureMap, lo, hi, above_m: float = 0.05) -> None:
    """Every Rig has a ground plane at z = 0; declare it as a hard surface
    across the scene's footprint so an approach to the floor is governed
    like an approach to the table."""
    fmap.add_aabb("ground", (lo[0], lo[1], -0.03), (hi[0], hi[1], above_m),
                  FEATURE_HARD_SURFACE, pad=0.0)


def add_body_zones(rig, fmap: FeatureMap, features: dict) -> list[str]:
    """Zones off the model's collision shapes for the bodies ``rig.meshes`` does not list (the kit library adds its
    assets to the builder directly): one AABB per body and one per labelled static shape, tagged with the declared
    feature for that label or ``_default`` and padded like ``gate.build_featuremap``'s.  Robot links, the table and
    the ground plane get none (the table and the ground have zones of their own)."""
    import newton
    from scipy.spatial.transform import Rotation
    m = rig.model
    body_lbl, shape_lbl = list(m.body_label), list(m.shape_label)
    sbody, stype, sflags = m.shape_body.numpy(), m.shape_type.numpy(), m.shape_flags.numpy()
    sscale, sxf, srad = m.shape_scale.numpy(), m.shape_transform.numpy(), m.shape_collision_radius.numpy()
    bq = rig.s0.body_q.numpy()
    skip = set(rig.meshes) | {"table"}
    table = set(rig.shapes.get("table", []))
    collide = int(newton.ShapeFlags.COLLIDE_SHAPES)
    planes = (int(newton.GeoType.PLANE), int(newton.GeoType.HFIELD))
    boxes: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for i in range(m.shape_count):
        if not int(sflags[i]) & collide or int(stype[i]) in planes or i in table:
            continue
        b = int(sbody[i])
        name = str(body_lbl[b]) if b >= 0 else str(shape_lbl[i])
        if name.startswith("fr3/") or name in skip:
            continue
        rot, pos = Rotation.from_quat(sxf[i][3:7]), np.asarray(sxf[i][:3], float)
        if b >= 0:      # shape frame in the world at the start pose
            rb = Rotation.from_quat(bq[b][3:7])
            rot, pos = rb * rot, np.asarray(bq[b][:3], float) + rb.apply(pos)
        verts = getattr(m.shape_source[i], "vertices", None)
        if verts is not None and len(verts):
            pts = pos + rot.apply(np.asarray(verts, float) * np.asarray(sscale[i], float))
            lo, hi = pts.min(axis=0), pts.max(axis=0)
        else:           # a primitive, centred on its frame
            r = float(srad[i])
            if not np.isfinite(r) or r > 1.0:
                continue
            lo, hi = pos - r, pos + r
        if name in boxes:
            lo, hi = np.minimum(lo, boxes[name][0]), np.maximum(hi, boxes[name][1])
        boxes[name] = (lo, hi)
    default = features.get("_default", 0.005)
    for name, (lo, hi) in boxes.items():
        fmap.add_aabb(name, lo, hi, features.get(name, default), pad=0.06)
    return sorted(boxes)


def task_featuremap(rig, source: str):
    """The guard's zones for a task's scene (the teleop UI's and the acceptance gate's): the clearances declared in
    ``gate.TASKS`` for the scene module it builds on (``source``) on the meshes that scene registers, every other
    body and labelled static shape at the hard-surface standoff, the table and the ground.  Returns (feature map,
    features, coverage, envelope low, envelope high)."""
    spec = G.TASKS.get(source)
    features = dict(spec.features) if spec else {"_default": 0.005}
    fmap, coverage = G.build_featuremap(rig, G.TaskSpec(module="", features=features))
    # no measured clearances for the kit's parts: their zones take the hard-surface standoff (the parts are
    # printed with sub-millimetre fits, which the 5 mm default would let the governor drive straight into)
    kit = {**features, "_default": min(features.get("_default", 0.005), FEATURE_HARD_SURFACE)}
    coverage["model_zones"] = add_body_zones(rig, fmap, kit)
    lo, hi = _envelope(rig)
    if add_table_zone(rig, fmap):
        features = {**features, "table": FEATURE_HARD_SURFACE}
    add_ground_zone(rig, fmap, lo, hi)
    features = {**features, "ground": FEATURE_HARD_SURFACE}
    return fmap, features, coverage, lo, hi


def _finger_effort(rig) -> float | None:
    try:
        lim = float(rig.model.joint_effort_limit.numpy()[7])
    except Exception:
        return None
    return lim if np.isfinite(lim) and 0.0 < lim < 1e4 else None


def _envelope(rig, pad=0.5):
    q0 = rig.s0.body_q.numpy()
    return q0[:, :3].min(axis=0) - pad, q0[:, :3].max(axis=0) + pad


def _camera_of(rig, tcp0):
    rec = getattr(rig, "rec", None)
    if rec is not None:
        tf = rec.cam_tf.numpy()[0][0]
        pos = np.array([tf[0], tf[1], tf[2]], dtype=float)
        tgt = np.asarray(rec.cam2_target, dtype=float)
        return pos, tgt
    tgt = np.asarray(tcp0[:3], dtype=float)
    return tgt + np.array([0.45, -0.45, 0.35]), tgt


def load_task_scene(task: str, substeps: int | None = None,
                    collide_substep: bool | None = None, args: tuple[str, ...] = ()) -> Scene:
    from force_bench.appendix_tasks.registry import BY_NAME as TASKS
    t0 = time.time()
    cap = _capture_task(task, tuple(args))
    rig = cap.rig
    if not hasattr(rig, "ik"):
        raise SystemExit(f"{task}: scene has no FR3; the teleop UI drives the arm")
    if substeps and int(substeps) != int(rig.substeps):
        # the task has already finalized: hard joint limits and any gear
        # transmission were sized for its own substep count and keep it
        print(f"[teleop-ui] {task}: substeps {rig.substeps} -> {substeps} after finalize; "
              "build-time dt consumers (hard limits, transmissions) keep the task's value")
        rig.substeps = int(substeps)
    if collide_substep is not None:
        rig.collide_substep_override = bool(collide_substep)
    fmap, features, coverage, lo, hi = task_featuremap(rig, TASKS[task].source)
    # the IK's warm start = the arm's commanded joints: a task that drives the joints itself (a joint-space
    # tracker, an in-hand pre-roll) leaves the IK at its last solve, which need not be where the arm stands
    jq = rig.ik.joint_q.numpy()
    jq[0, :7] = rig.control.joint_target_q.numpy()[:7]
    rig.ik.joint_q.assign(jq)
    tcp = rig.ik.tcp_pose()
    tcp0 = np.asarray(tcp[:3], dtype=float)
    q0 = np.asarray(tcp[3:7], dtype=float)
    grip0 = float(rig.control.joint_target_q.numpy()[7])
    cam_pos, cam_target = _camera_of(rig, tcp0)
    return Scene(name=task, rig=rig, fmap=fmap, features=features, coverage=coverage,
                 tcp0=tcp0, q0=q0, grip0=grip0, cam_pos=cam_pos, cam_target=cam_target,
                 env_lo=lo, env_hi=hi, task_extra=cap.extra_cb,
                 finger_effort_N=_finger_effort(rig),
                 note=VERIFIED.get(task) or task_note(task), build_s=time.time() - t0, verified=task in VERIFIED,
                 args=tuple(args), hold_start_pinch=True)


# ── the playground ──────────────────────────────────────────────────────
FEATURE_SLOT = 0.0021     # 2.1 mm card slot for a 1.2 mm card (README pitfall 8.2)
FEATURE_CARD = 0.0012     # the card itself: what a fast drive punches through (pitfall 6.1)
FEATURE_PLUG = 0.0025     # 2.5 mm face-plate slots, 0.45 mm of blade play per side


def build_playground(substeps: int | None = None, collide_substep: bool | None = None) -> Scene:
    from force_bench.assets import load_tm
    from force_bench.recorder import studio_palette
    from force_bench.rig import set_finger_effort_limit
    t0 = time.time()
    rig = Rig("teleop_playground")
    if substeps:
        rig.substeps = int(substeps)
    if collide_substep is not None:
        rig.collide_substep_override = bool(collide_substep)
    base_xf = wp.transform(wp.vec3(0.01, -0.50, 0.0), wp.quat_identity())
    rig.add_robot(base_xf)
    set_finger_effort_limit(rig.builder, 20.0)
    rig.table(center_xy=(0.05, -0.05), half=0.45)

    housing_c = rig.static_mesh("card_swiper", "part1_1_body1", "housing", res=384)
    pad_body, pad_c = rig.link_meshes("card_swiper", ["part2_1_body1"], "pad", res=256)
    rig.prismatic_part(pad_body, pad_c, (0.0, 1.0, 0.0), "pad_joint", spring_ke=7000.0,
                       spring_kd=8.0, limits=(0.0, 0.004), mass=0.03)

    card_tm = load_tm("card_swiper", "card_v2_1_part4_1_body1")
    card_ext = card_tm.extents
    card_pos = np.array([0.15, -0.20, card_ext[1] / 2 + 0.001])
    card_body, _ = rig.free_mesh(
        "card_swiper", "card_v2_1_part4_1_body1", "card", res=384, nb=0.0015, margin=0.001,
        mu=0.35, mass=0.006,
        xform=wp.transform(wp.vec3(*card_pos),
                           wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), np.pi / 2)))
    plug_tm = load_tm("charging_block_outlet", "charging_block_3_1_body1")
    plug_pos = np.array([-0.12, -0.22, plug_tm.extents[2] / 2 + 0.001])
    plug_body, _ = rig.free_mesh(
        "charging_block_outlet", "charging_block_3_1_body1", "plug", res=320, mass=0.06,
        xform=wp.transform(wp.vec3(*plug_pos), wp.quat_identity()))

    # start above the charger plug (the one graspable part: the card lies
    # flat), fingers down, 13 cm up: the original scene started at the FR3
    # home pose, which puts the gripper over the table edge behind the objects
    from force_bench.robot.franka import down_grasp_quat
    tcp_start = np.array([plug_pos[0], plug_pos[1], 0.13])
    rig.solve_arm_start(tcp_start, down_grasp_quat(0.0))
    rig.builder.joint_q[7:9] = [0.04, 0.04]
    rig.builder.joint_target_q[7:9] = [0.04, 0.04]
    # camera from the far side of the table looking back at the work and the
    # robot behind it (the robot base is at y = -0.5)
    cam_pos = np.array([0.42, 0.30, 0.42])
    cam_target = np.array([0.05, -0.20, 0.05])
    rig.finalize(probes=["fr3/fr3_leftfinger", "fr3/fr3_rightfinger", "card", "plug"],
                 cam_pos=cam_pos, cam_target=cam_target, record=False)
    studio_palette(rig.model)

    fmap = FeatureMap(free_air_m=0.05)
    fmap.add_sphere("card_slot", np.asarray(housing_c), radius=0.09, feature_m=FEATURE_SLOT, pad=0.02)
    fmap.add_sphere("card", card_pos, radius=0.07, feature_m=FEATURE_CARD, pad=0.02)
    fmap.add_sphere("plug", plug_pos, radius=0.07, feature_m=FEATURE_PLUG, pad=0.02)
    add_table_zone(rig, fmap)
    lo, hi = _envelope(rig)
    add_ground_zone(rig, fmap, lo, hi)
    features = {"card_slot": FEATURE_SLOT, "card": FEATURE_CARD, "plug": FEATURE_PLUG,
                "table": FEATURE_HARD_SURFACE, "ground": FEATURE_HARD_SURFACE}
    tcp = rig.ik.tcp_pose()
    return Scene(name="playground", rig=rig, fmap=fmap, features=features,
                 coverage={"bodies": 5, "declared": 5, "declared_labels": sorted(features),
                           "undeclared_labels": [], "stale_declarations": []},
                 tcp0=np.asarray(tcp[:3], float), q0=np.asarray(tcp[3:7], float),
                 grip0=0.04, cam_pos=cam_pos, cam_target=cam_target, env_lo=lo, env_hi=hi,
                 finger_effort_N=_finger_effort(rig),
                 note=VERIFIED["playground"], build_s=time.time() - t0, verified=True)


def load_scene(name: str, substeps: int | None = None,
               collide_substep: bool | None = None, args: tuple[str, ...] = ()) -> Scene:
    if name == "playground":
        if args:
            raise SystemExit("teleop-ui: --seed, --set and --embodiment apply to the tasks, not the playground")
        return build_playground(substeps, collide_substep)
    return load_task_scene(name, substeps, collide_substep, args)
