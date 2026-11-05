"""Validate dataset checksums/alignment and report collection coverage."""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np

from .episode import discover_episodes, load_episode


def inspect_dataset(path, *, success_only=False):
    paths = discover_episodes(path, success_only=success_only)
    outcomes, tasks, dimensions, rates = Counter(), Counter(), Counter(), Counter()
    transitions = 0
    tactile_episodes = 0
    tactile_valid, tactile_total = 0, 0
    errors = []
    for episode_path in paths:
        try:
            episode = load_episode(episode_path)
            metadata = episode.metadata
            outcomes[metadata.get("outcome", {}).get("status", "unvalidated")] += 1
            tasks[metadata.get("task", "unknown")] += 1
            dimensions[f"observation={episode['observations'].shape[1]}, action={episode['actions'].shape[1]}"] += 1
            rates[str(metadata.get("fps", "unknown"))] += 1
            transitions += len(episode["actions"])
            if "tactile" in episode.arrays:
                tactile_episodes += 1
                validity = episode["tactile_valid"]
                tactile_valid += int(np.count_nonzero(validity))
                tactile_total += int(validity.size)
        except (ValueError, OSError, KeyError) as error:
            errors.append({"episode": str(episode_path), "error": str(error)})
    return {"path": str(Path(path).expanduser().resolve()), "success_only": bool(success_only),
            "episodes": len(paths), "validated_episodes": len(paths) - len(errors),
            "transitions": transitions, "outcomes": dict(outcomes), "tasks": dict(tasks),
            "dimensions": dict(dimensions), "fps": dict(rates),
            "tactile": {"episodes": tactile_episodes, "valid_pad_frames": tactile_valid,
                         "total_pad_frames": tactile_total,
                         "valid_fraction": tactile_valid / tactile_total if tactile_total else None},
            "errors": errors, "pass": bool(paths) and not errors}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path")
    parser.add_argument("--success-only", action="store_true")
    args = parser.parse_args(argv)
    try:
        summary = inspect_dataset(args.path, success_only=args.success_only)
    except (ValueError, OSError) as error:
        print(json.dumps({"pass": False, "errors": [str(error)]}, indent=2))
        return 1
    print(json.dumps(summary, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
