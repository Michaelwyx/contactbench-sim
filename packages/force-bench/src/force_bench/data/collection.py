"""Opt-in hooks shared by scripted rigs and closed-loop policy environments.

These helpers read device state; they never apply a command or modify physics.
"""
from __future__ import annotations

import hashlib
import importlib.metadata
from dataclasses import asdict, is_dataclass
import os
from pathlib import Path
import subprocess

import numpy as np

from .episode import EpisodeWriter

ROBOT_DOF = 9
PAD_ORDER = ("gel_left", "gel_right")


def tactile_metadata(size=16):
    if not isinstance(size, int) or size < 1:
        raise ValueError("tactile_size must be a positive integer")
    return {"quantity": "pressure", "units": "Pa", "pad_order": list(PAD_ORDER),
            "shape": [2, size, size], "resampling": "area_average",
            "initial_missing": "zero filled; tactile_valid marks unmeasured frames"}

# Explicit simulation/controller knobs in this repository. Render settings,
# output paths, cache directories and unrelated process environment are not
# part of the physics compatibility contract.
_PHYSICAL_ENVIRONMENT_KEYS = {
    "TB_GELSIGHT", "TB_SUBSTEPS", "TB_COLLIDE_SUBSTEP", "TB_MU_TORSIONAL",
    "TB_ARM_EFFORT", "TB_STABILIZER", "TB_CARD_PRESS_N", "TB_STROKES",
    "TB_PIC_CADENCE", "TB_PIC_LANE", "TB_PIC_NCON", "TB_PIC_NOGROUND",
    "TB_PIC_RESTCLEAR", "TB_PIC_SETTLE_DUR", "TB_GRIP_EXT", "TB_ITERS",
    "TB_PAD_KE", "TB_SD_GRIP", "TB_SD_NOGRIP", "TB_SLIDE_PART", "TB_CADENCE_T", "TB_GAP",
    "GS_BROAD", "GS_GEL_MESH", "GS_CASE_TIP_EXTEND", "GS_COLLIDE_EVERY",
    "GS_FINGER_EFFORT", "GS_FINGER_KE", "GS_GRAPH", "GS_MOUNT", "GS_PAD_LONG_AXIS",
    "GS_GEL_MU", "GS_GEL_STIFFEN", "GS_FORCE_HI", "GS_FORCE_LO", "GS_OBJ",
}


def physics_environment():
    """Semantic explicit knobs; unset GelSight is the same as explicit zero."""
    values = {key: value for key, value in os.environ.items()
              if key in _PHYSICAL_ENVIRONMENT_KEYS or key.startswith("TB_MJ_")}
    values["TB_GELSIGHT"] = os.environ.get("TB_GELSIGHT", "0")
    return values


def physics_metadata(rig):
    """Snapshot this built rig, including options of the *actual* solver.

    The requested rig defaults are not evidence that a replacement solver
    used them. A solver without the factory's effective-options tag records
    None explicitly rather than fabricating a configuration.
    """
    effective = getattr(rig.solver, "_force_bench_effective_options", None)
    gel_config = getattr(rig, "gel_config", None)
    return {"configuration_version": 1, "solver": type(rig.solver).__name__,
            "substeps": int(rig.substeps),
            "solver_options": dict(getattr(rig, "solver_options", {})),
            "effective_solver_options": None if effective is None else dict(effective),
            "collide_every_substep_at_start": bool(rig.effective_collide_every_substep()),
            "collide_every_n_substeps": int(getattr(rig, "_collide_every", 1)),
            "gel_config": asdict(gel_config) if is_dataclass(gel_config) else None,
            "environment": physics_environment()}


def _numpy(value):
    return value.numpy() if hasattr(value, "numpy") else np.asarray(value)


def action_metadata(rig):
    labels = [str(value) for value in rig.model.joint_label][:ROBOT_DOF]
    if len(labels) != ROBOT_DOF or not all(value.startswith("fr3/") for value in labels):
        raise ValueError("collection requires Rig.add_robot(): first nine coordinates must be FR3 joints")
    return {"action_type": "joint_position_target", "action_dim": ROBOT_DOF,
            "action_names": labels, "action_indices": list(range(ROBOT_DOF)),
            "action_units": ["rad"] * 7 + ["m"] * 2}


def robot_action(rig):
    return np.asarray(_numpy(rig.control.joint_target_q)[:ROBOT_DOF], dtype=np.float32).copy()


class RigObservation:
    """Measured robot q/qd followed by nonrobot world body poses (privileged).

    Model joint_q holds initial configuration in Newton; use *state* joint_q.
    Field ordering is stable within a scene and stored in the episode schema.
    """
    def __init__(self, rig):
        self.rig = rig
        action = action_metadata(rig)
        labels = [str(value) for value in rig.model.body_label]
        self.body_indices = [i for i, label in enumerate(labels) if not label.startswith("fr3/")]
        self.body_labels = [labels[i] for i in self.body_indices]
        fields = [{"name": "robot_joint_position", "start": 0, "stop": 9,
                   "units": action["action_units"], "privileged": False},
                  {"name": "robot_joint_velocity", "start": 9, "stop": 18,
                   "units": ["rad/s"] * 7 + ["m/s"] * 2, "privileged": False}]
        names = [f"joint_position/{label}" for label in action["action_names"]]
        names += [f"joint_velocity/{label}" for label in action["action_names"]]
        for i, label in enumerate(self.body_labels):
            start = 18 + 7 * i
            fields.append({"name": f"body_pose/{label}", "start": start, "stop": start + 7,
                           "units": ["m"] * 3 + ["unitless"] * 4,
                           "frame": "world", "quaternion_order": "xyzw", "privileged": True})
            names.extend(f"body_pose/{label}/{axis}" for axis in ("x", "y", "z", "qx", "qy", "qz", "qw"))
        self.metadata = {"observation_dim": len(names), "observation_names": names,
                         "observation_fields": fields, "task_body_labels": self.body_labels,
                         "observation_source": "measured_simulator_state_with_privileged_object_poses"}

    def sample(self):
        state = self.rig.s0
        q = _numpy(state.joint_q)[:ROBOT_DOF]
        qd = _numpy(state.joint_qd)[:ROBOT_DOF]
        bodies = _numpy(state.body_q)[self.body_indices].reshape(-1)
        return np.concatenate((q, qd, bodies)).astype(np.float32)


def _area_resize(image, size):
    """Area averaging preserves the integral of the uniformly spaced raster."""
    image = np.asarray(image, dtype=np.float32)
    def weights(source):
        edges = np.linspace(0, source, size + 1)
        lo = np.maximum(edges[:-1, None], np.arange(source)[None, :])
        hi = np.minimum(edges[1:, None], np.arange(1, source + 1)[None, :])
        return (np.maximum(0.0, hi - lo) / (source / size)).astype(np.float32)
    return weights(image.shape[0]) @ image @ weights(image.shape[1]).T


def read_tactile(rig, size=16):
    values = np.zeros((2, size, size), dtype=np.float32)
    valid = np.zeros(2, dtype=bool)
    for i, pad in enumerate(PAD_ORDER):
        frames = getattr(rig, "tactile_frames", {}).get(pad, [])
        if frames:
            values[i] = _area_resize(frames[-1].pressure_pa, size)
            valid[i] = True
    return values, valid


def source_manifest(repo_root=None):
    """Hash actual package sources, including untracked modules, without content."""
    if repo_root is None:
        here = Path(__file__).resolve()
        repo_root = next((parent for parent in here.parents
                          if (parent / "packages" / "force-bench" / "src" / "force_bench").is_dir()), None)
    if repo_root is None:
        return {"files_sha256": {}, "unavailable_reason": "source checkout not found"}
    root = Path(repo_root)
    files = []
    for source in (root / "packages/force-bench/src/force_bench", root / "packages/hydrogel/src/hydrogel"):
        files.extend(source.rglob("*.py"))
    manifest = root / "assets/manifest.yaml"
    if manifest.is_file():
        files.append(manifest)
    return {"files_sha256": {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
                             for path in sorted(files)},
            "scope": "force_bench and hydrogel Python source plus assets/manifest.yaml; CAD binaries not hashed"}


def provenance():
    """Capture source and relevant explicit physics knobs, without shell expansion."""
    versions = {}
    for package in ("force-bench", "hydrogel", "newton", "warp-lang", "mujoco-warp", "numpy"):
        try:
            versions[package] = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            versions[package] = None
    git = {}
    directory = Path(__file__).resolve().parent
    for key, args in (("commit", ["rev-parse", "HEAD"]), ("status", ["status", "--porcelain"])):
        result = subprocess.run(["git", "-C", str(directory), *args], capture_output=True, text=True, check=False)
        git[key] = result.stdout.strip() if result.returncode == 0 else None
    # Git pathspecs are relative to -C unless explicitly rooted. This
    # function runs inside force_bench/data, not at the checkout root.
    diff = subprocess.run(["git", "-C", str(directory), "diff", "HEAD", "--",
                           ":(top)packages", ":(top)integrations"],
                          capture_output=True, check=False)
    git["tracked_source_diff_sha256"] = hashlib.sha256(diff.stdout).hexdigest() if diff.returncode == 0 else None
    git["tracked_source_diff_scope"] = [":(top)packages", ":(top)integrations"]
    # Only simulation knobs are relevant; paths and unrelated environment values
    # (e.g. credentials) are deliberately not copied to a shareable dataset.
    return {"git": git, "versions": versions, "physics_environment": physics_environment(),
            "source_manifest": source_manifest()}


class RigCollector:
    def __init__(self, rig, output, *, task=None, seed=0, source="scripted_expert", tactile_size=16):
        self.rig = rig
        self.observation = RigObservation(rig)
        self.has_tactile = bool(getattr(rig, "gel_sensors", {}))
        self.tactile_size = int(tactile_size)
        if self.tactile_size < 1:
            raise ValueError("tactile_size must be positive")
        metadata = {"task": task or rig.name, "seed": int(seed), "source": source,
                    **action_metadata(rig), **self.observation.metadata,
                    "fps": float(rig.fps), "dt": 1.0 / rig.fps,
                    "timebase": "simulation_seconds; obs[0] before first control, obs[t+1] after action[t]",
                    "reward_semantics": "unavailable; stored zeros are placeholders, not measured reward",
                    "terminal_semantics": "script horizon is truncated; offline QA does not create an MDP terminal",
                    "physics": physics_metadata(rig),
                    "provenance": provenance(),
                    "replay_compatible": False,
                    "replay_limitations": ["Generic scripted callbacks can modify auxiliary mechanisms; use a task-specific replay adapter.",
                                           "Nonzero robot velocity/feedforward commands require corresponding action support."],
                    "extra_action_fields": {"action_target_velocity": "robot joint target velocity rad/s (7), m/s (2)",
                                            "action_feedforward": "robot joint torque N m (7), force N (2)"}}
        if self.has_tactile:
            metadata["tactile"] = tactile_metadata(self.tactile_size)
            initial, validity = read_tactile(rig, self.tactile_size)
        else:
            initial, validity = None, None
        self.writer = EpisodeWriter(output, metadata, self.observation.sample(), tactile=initial,
                                    tactile_valid=validity)
        self.pending = None

    def after_control(self):
        if self.pending is not None:
            raise RuntimeError("previous control has no post-step observation")
        extra = {}
        for output, field in (("action_target_velocity", "joint_target_qd"), ("action_feedforward", "joint_f")):
            values = getattr(self.rig.control, field, None)
            extra[output] = np.zeros(ROBOT_DOF, dtype=np.float32) if values is None else _numpy(values)[:ROBOT_DOF].copy()
        self.pending = robot_action(self.rig), extra

    def after_step(self, timestamp, extra=None):
        """``extra``: further per-step arrays measured after the step (for example contact forces)."""
        if self.pending is None:
            raise RuntimeError("post-step observation has no recorded control")
        tactile, valid = read_tactile(self.rig, self.tactile_size) if self.has_tactile else (None, None)
        action, extra_cmd = self.pending
        extra = {**extra_cmd, **{k: np.asarray(v, dtype=np.float32) for k, v in (extra or {}).items()}}
        self.writer.append(action, self.observation.sample(), timestamp=timestamp, tactile=tactile,
                           tactile_valid=valid, extra=extra)
        self.pending = None

    def finish(self, *, outcome=None, discard_pending=False):
        if self.pending is not None:
            if not discard_pending:
                raise RuntimeError("cannot finish before the pending control is stepped")
            self.writer.metadata["discarded_incomplete_transition"] = True
            self.pending = None
        path = self.writer.finish(truncated=True, outcome=outcome)
        print(f"[collection] wrote {path} ({len(self.writer.actions)} transitions; QA pending)")
        return path


def collector_from_env(rig):
    return RigCollector(rig, os.environ["TB_COLLECT_DIR"],
                        task=os.environ.get("TB_COLLECT_TASK", rig.name),
                        seed=int(os.environ.get("TB_COLLECT_SEED", "0")),
                        source=os.environ.get("TB_COLLECT_SOURCE", "scripted_expert"),
                        tactile_size=int(os.environ.get("TB_COLLECT_TACTILE_SIZE", "16")))
