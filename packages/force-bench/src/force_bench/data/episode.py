"""Versioned, aligned demonstration episodes; NumPy is the only dependency.

An action at index t is the command applied between observations t and t+1.
A directory becomes discoverable only after both arrays and metadata exist.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid

import numpy as np

SCHEMA_VERSION = 1


def atomic_json(path: Path, value: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=".metadata-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _float(value, name):
    value = np.asarray(value, dtype=np.float32)
    if not np.isfinite(value).all():
        raise ValueError(f"{name} contains non-finite values")
    return value.copy()


@dataclass
class Episode:
    path: Path
    metadata: dict
    arrays: dict[str, np.ndarray]

    def __getitem__(self, name):
        return self.arrays[name]


def validate_episode(arrays: dict, metadata: dict) -> None:
    if metadata.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("unsupported episode schema_version")
    for name in ("observations", "actions", "timestamps", "rewards", "terminated", "truncated"):
        if name not in arrays:
            raise ValueError(f"episode missing {name}")
    obs, actions = arrays["observations"], arrays["actions"]
    if actions.ndim != 2 or obs.ndim != 2 or len(obs) != len(actions) + 1 or len(actions) == 0:
        raise ValueError("expected nonempty actions[T,A] and observations[T+1,D]")
    n = len(actions)
    if metadata.get("action_dim") != actions.shape[1] or metadata.get("observation_dim") != obs.shape[1]:
        raise ValueError("array dimensions disagree with metadata")
    if obs.dtype != np.float32 or actions.dtype != np.float32 or arrays["rewards"].dtype != np.float32:
        raise ValueError("observations, actions, and rewards must be float32")
    if arrays["timestamps"].dtype != np.float64:
        raise ValueError("timestamps must be float64")
    for key in ("rewards", "terminated", "truncated"):
        if arrays[key].shape != (n,):
            raise ValueError(f"{key} must have shape [T]")
    if arrays["timestamps"].shape != (n + 1,) or not (np.diff(arrays["timestamps"]) > 0).all():
        raise ValueError("timestamps must increase strictly and align with observations")
    if arrays["terminated"].dtype != np.bool_ or arrays["truncated"].dtype != np.bool_:
        raise ValueError("terminal flags must be bool")
    if np.any(arrays["terminated"][:-1] | arrays["truncated"][:-1]):
        raise ValueError("episode contains transitions after a terminal flag")
    if "tactile" in arrays:
        tactile = arrays["tactile"]
        if tactile.ndim != 4 or tactile.shape[:2] != (n + 1, 2):
            raise ValueError("tactile must have shape [T+1,2,H,W]")
        if "tactile_valid" not in arrays or arrays["tactile_valid"].shape != (n + 1, 2):
            raise ValueError("tactile_valid must have shape [T+1,2]")
    for name, value in arrays.items():
        if value.dtype.kind == "O":
            raise ValueError(f"object array {name} is forbidden")
        if value.dtype.kind in "fc" and not np.isfinite(value).all():
            raise ValueError(f"{name} contains non-finite values")


class EpisodeWriter:
    """Reusable writer for expert, teleoperation, or policy transitions.

    The caller owns action semantics and measures the initial observation
    BEFORE applying the first action. Failed/unfinished runs are not published.
    """
    def __init__(self, output, metadata, initial_observation, *, timestamp=0.0,
                 tactile=None, tactile_valid=None, episode_id=None):
        self.output = Path(output)
        self.metadata = dict(metadata)
        self.episode_id = episode_id or (
            datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ_") + uuid.uuid4().hex[:10])
        if Path(self.episode_id).name != self.episode_id or self.episode_id.startswith("."):
            raise ValueError("episode_id must be a visible directory basename")
        initial = _float(initial_observation, "initial observation")
        if initial.ndim != 1:
            raise ValueError("initial observation must be a vector")
        self.observations = [initial]
        self.actions, self.timestamps, self.rewards = [], [float(timestamp)], []
        self.terminated, self.truncated = [], []
        self.tactile = None if tactile is None else [_float(tactile, "tactile")]
        self.tactile_valid = None if tactile is None else [np.asarray(tactile_valid, dtype=bool).copy()]
        self.extra = {}
        self._closed = False

    def append(self, action, observation, *, timestamp, reward=0.0, terminated=False,
               truncated=False, tactile=None, tactile_valid=None, extra=None):
        if self._closed or (self.terminated and (self.terminated[-1] or self.truncated[-1])):
            raise ValueError("cannot append to a closed/terminal episode")
        action = _float(action, "action")
        observation = _float(observation, "observation")
        if action.ndim != 1 or action.shape != (self.metadata["action_dim"],):
            raise ValueError("action shape disagrees with metadata")
        if observation.shape != self.observations[0].shape:
            raise ValueError("observation shape changed within episode")
        if not np.isfinite(timestamp) or timestamp <= self.timestamps[-1] or not np.isfinite(reward):
            raise ValueError("timestamp must increase; timestamp/reward must be finite")
        extras = {k: _float(v, k) for k, v in (extra or {}).items()}
        reserved = {"observations", "actions", "timestamps", "rewards", "terminated", "truncated", "tactile", "tactile_valid"}
        if set(extras) & reserved:
            raise ValueError("extra fields cannot overwrite core episode arrays")
        if self.actions and set(extras) != set(self.extra):
            raise ValueError("extra command fields changed within episode")
        for key, value in extras.items():
            if key in self.extra and value.shape != self.extra[key][0].shape:
                raise ValueError(f"extra field {key} shape changed")
        if (tactile is None) != (self.tactile is None):
            raise ValueError("tactile availability changed within episode")
        if self.tactile is not None:
            tactile = _float(tactile, "tactile")
            tactile_valid = np.asarray(tactile_valid, dtype=bool)
            if tactile.shape != self.tactile[0].shape or tactile_valid.shape != (2,):
                raise ValueError("tactile shape changed or validity missing")
            self.tactile.append(tactile)
            self.tactile_valid.append(tactile_valid.copy())
        self.actions.append(action)
        self.observations.append(observation)
        self.timestamps.append(float(timestamp))
        self.rewards.append(float(reward))
        self.terminated.append(bool(terminated))
        self.truncated.append(bool(truncated))
        for key, value in extras.items():
            self.extra.setdefault(key, []).append(value)

    def finish(self, *, truncated=True, outcome=None):
        if self._closed or not self.actions:
            raise ValueError("cannot publish an empty or already closed episode")
        self.truncated[-1] = bool(truncated and not self.terminated[-1])
        arrays = {
            "observations": np.asarray(self.observations, dtype=np.float32),
            "actions": np.asarray(self.actions, dtype=np.float32),
            "timestamps": np.asarray(self.timestamps, dtype=np.float64),
            "rewards": np.asarray(self.rewards, dtype=np.float32),
            "terminated": np.asarray(self.terminated, dtype=bool),
            "truncated": np.asarray(self.truncated, dtype=bool),
            **{key: np.asarray(rows, dtype=np.float32) for key, rows in self.extra.items()},
        }
        if self.tactile is not None:
            arrays["tactile"] = np.asarray(self.tactile, dtype=np.float32)
            arrays["tactile_valid"] = np.asarray(self.tactile_valid, dtype=bool)
        metadata = {**self.metadata, "schema_version": SCHEMA_VERSION,
                    "episode_id": self.episode_id, "num_transitions": len(self.actions),
                    "observation_dim": arrays["observations"].shape[1],
                    "created_utc": datetime.now(timezone.utc).isoformat(),
                    "outcome": outcome or {"status": "unvalidated", "success": None,
                                           "source": "unavailable"}}
        validate_episode(arrays, metadata)
        self.output.mkdir(parents=True, exist_ok=True)
        destination = self.output / self.episode_id
        if destination.exists():
            raise FileExistsError(destination)
        temporary = Path(tempfile.mkdtemp(prefix=".episode-", dir=self.output))
        try:
            data_path = temporary / "episode.npz"
            np.savez_compressed(data_path, **arrays)
            metadata["arrays_sha256"] = hashlib.sha256(data_path.read_bytes()).hexdigest()
            atomic_json(temporary / "metadata.json", metadata)
            os.rename(temporary, destination)
        except BaseException:
            shutil.rmtree(temporary, ignore_errors=True)
            raise
        self._closed = True
        return destination


def load_episode(path, *, verify_hash=True):
    path = Path(path)
    metadata = json.loads((path / "metadata.json").read_text())
    data_path = path / "episode.npz"
    if verify_hash and metadata.get("arrays_sha256"):
        if hashlib.sha256(data_path.read_bytes()).hexdigest() != metadata["arrays_sha256"]:
            raise ValueError(f"episode checksum mismatch: {path}")
    with np.load(data_path, allow_pickle=False) as archive:
        arrays = {key: archive[key] for key in archive.files}
    validate_episode(arrays, metadata)
    return Episode(path, metadata, arrays)


def discover_episodes(root, *, success_only=False):
    """Return complete visible episodes; incomplete staging is never training data."""
    root = Path(root)
    candidates = [root] if (root / "metadata.json").exists() else [p.parent for p in root.rglob("metadata.json")]
    found = []
    for directory in sorted(candidates):
        relative = directory.relative_to(root) if directory != root else Path()
        if any(part.startswith(".") for part in relative.parts) or not (directory / "episode.npz").exists():
            continue
        metadata = json.loads((directory / "metadata.json").read_text())
        if success_only and metadata.get("outcome", {}).get("success") is not True:
            continue
        found.append(directory)
    return found
