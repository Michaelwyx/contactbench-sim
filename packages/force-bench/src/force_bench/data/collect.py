"""Collect registered scripted experts in isolated fresh processes.

python -m force_bench.data.collect --task plug_in_usb_2 --episodes 2 --output outputs/datasets/usb

``--task`` takes a ContactBench task with a scripted policy (``force-bench list``; a task marked "scene only" is
recorded by teleoperation instead, ``force-bench teleop-ui --collect-dir``).  An episode's outcome is the task's own
verdict, ``appendix_qa.json``: its success criterion plus the physics-integrity checks.

Episode ``i`` runs with seed ``--seed + i``.  A ContactBench task that takes ``--seed`` (``variations.SEEDED``) gets
it too, so each episode starts from its own random start (object poses drawn within the task's ranges);
``--nominal-start`` keeps every episode at the task's nominal start.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import random
import runpy
import shutil
import subprocess
import sys
import tempfile
import uuid

import numpy as np

from .episode import atomic_json, discover_episodes, load_episode


def _spec(name):
    from force_bench.appendix_tasks.registry import BY_NAME as TASKS
    spec = TASKS.get(name)
    if spec is None:
        raise ValueError(f"unknown task {name!r}; see `force-bench list`")
    if not spec.scripted:
        raise ValueError(f"{name} has no scripted policy (a scene only); record it by teleoperation: "
                         f"force-bench teleop-ui --scene {name} --collect-dir <dir>")
    return spec


def task_arguments(spec, seed, random_start=True):
    """The task module's arguments for one episode: the row's own, plus ``--seed`` where the task draws its start."""
    from force_bench.appendix_tasks.variations import SEEDED
    seeded = random_start and getattr(spec, "module", None) in SEEDED
    return [*spec.argv, *(["--seed", str(seed)] if seeded else [])]


def _worker(task, seed, random_start=True):
    spec = _spec(task)
    random.seed(seed)
    np.random.seed(seed)
    # Task imports happen only after the parent has set all simulation knobs.
    sys.argv = [spec.dotted, *task_arguments(spec, seed, random_start)]
    runpy.run_module(spec.dotted, run_name="__main__", alter_sys=True)


def qa_outcome(artifact_root, *, returncode):
    """Read only this fresh subprocess's isolated artifact tree, never old PASS."""
    # the task's verdict: appendix_qa.json (a scene-level rollout_qa.json beside it is not the task's criterion)
    reports = sorted(Path(artifact_root).rglob("appendix_qa.json"))
    if returncode != 0:
        return {"status": "failed", "success": False, "source": "process_exit",
                "returncode": returncode, "reason": "task subprocess did not finish successfully"}
    if len(reports) != 1:
        return {"status": "unvalidated", "success": None, "source": "unavailable",
                "reason": f"expected one fresh appendix_qa.json; found {len(reports)}"}
    try:
        report = json.loads(reports[0].read_text())
    except (OSError, ValueError) as error:
        return {"status": "unvalidated", "success": None, "source": "invalid_qa", "reason": str(error)}
    passed = report.get("pass")
    if not isinstance(passed, bool):
        return {"status": "unvalidated", "success": None, "source": "invalid_qa",
                "reason": "QA report has no boolean pass value"}
    return {"status": "passed" if passed else "failed", "success": passed,
            "qa_pass": passed, "source": "task_qa",
            "criterion": "ContactBench success criterion and physics-integrity checks",
            "qa_path": str(Path("artifacts") / reports[0].relative_to(artifact_root))}


def relocate_artifact_references(artifact_root, old_root, new_root):
    """Keep task-generated scene references valid when staging is published.

    Legacy task renders store absolute mesh paths. Only paths under this run's
    artifact tree are rewritten; asset references elsewhere retain their source.
    """
    old_root, new_root = Path(old_root), Path(new_root)

    def replace(value):
        if isinstance(value, str):
            try:
                relative = Path(value).relative_to(old_root)
            except ValueError:
                return value
            return str(new_root / relative)
        if isinstance(value, list):
            return [replace(item) for item in value]
        if isinstance(value, dict):
            return {key: replace(item) for key, item in value.items()}
        return value

    for scene in Path(artifact_root).rglob("tactile_scene.npz"):
        with np.load(scene, allow_pickle=False) as archive:
            arrays = {key: archive[key] for key in archive.files}
        if "objects_json" in arrays:
            objects = json.loads(str(arrays["objects_json"].item()))
            arrays["objects_json"] = np.asarray(json.dumps(replace(objects)))
            np.savez(scene, **arrays)
    for report in Path(artifact_root).rglob("*.json"):
        try:
            content = json.loads(report.read_text())
        except (OSError, ValueError):
            continue
        updated = replace(content)
        if updated != content:
            atomic_json(report, updated)


def collect_episode(task, output, *, seed=0, gelsight=False, random_start=True):
    spec = _spec(task)
    if gelsight and not spec.gelsight:
        raise ValueError(f"{task} is not registered as GelSight-capable")
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    token = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ_") + uuid.uuid4().hex[:8]
    staging = Path(tempfile.mkdtemp(prefix=".collecting-", dir=output))
    artifact_root = staging / "artifacts"
    env = os.environ.copy()
    env.update(TB_COLLECT_DIR=str(staging / "episodes"), TB_COLLECT_TASK=task,
               TB_COLLECT_SEED=str(seed), TB_COLLECT_SOURCE="scripted_expert",
               TB_OUT_ROOT=str(artifact_root), TB_GELSIGHT="1" if gelsight else "0",
               PYTHONHASHSEED=str(seed))
    command = [sys.executable, "-m", "force_bench.data.collect", "--_worker", "--task", task, "--seed", str(seed),
               *([] if random_start else ["--nominal-start"])]
    task_args = task_arguments(spec, seed, random_start)
    print(f"[collect] task={task} seed={seed} gelsight={gelsight}; staging={staging}", flush=True)
    log_path = staging / "worker.log"
    with log_path.open("w") as log:
        process = subprocess.run(command, env=env, stdout=log, stderr=subprocess.STDOUT, check=False)
    outcome = qa_outcome(artifact_root, returncode=process.returncode)
    episodes = discover_episodes(staging / "episodes")
    if not episodes:
        failure = output / f"failed_{task}_{token}"
        atomic_json(staging / "collection_failure.json", {"task": task, "seed": seed, "outcome": outcome,
                    "reason": "no aligned Rig episode produced; standalone/mechanism rigs are not supported"})
        os.rename(staging, failure)
        raise RuntimeError(f"No episode produced for {task}; inspect {failure / 'worker.log'}")
    published = []
    # Normal tasks call Rig.run once. If a task has multiple run segments,
    # preserve every segment but do not attribute the overall QA to each one.
    for index, episode_path in enumerate(episodes):
        episode = load_episode(episode_path)
        metadata = episode.metadata
        metadata["outcome"] = outcome if len(episodes) == 1 else {
            "status": "unvalidated", "success": None, "source": "multiple_segments",
            "reason": "task produced multiple Rig.run segments; overall QA is not segment success", "task_outcome": outcome}
        metadata["collection"] = {"command": command, "gelsight": gelsight, "returncode": process.returncode,
                                   "seed_scope": "Python random, NumPy global generator, PYTHONHASHSEED; task-local fixed RNGs may override",
                                   "task_arguments": task_args, "segment_index": index,
                                   "start": "random (--seed)" if "--seed" in task_args else "nominal",
                                   "segment_count": len(episodes)}
        name = f"{task}_{token}" + (f"_segment{index}" if len(episodes) > 1 else "")
        metadata["episode_id"] = name
        destination = output / name
        if len(episodes) == 1:
            if artifact_root.exists():
                relocate_artifact_references(artifact_root, artifact_root, destination / "artifacts")
                shutil.move(str(artifact_root), str(episode_path / "artifacts"))
            shutil.copy2(log_path, episode_path / "worker.log")
        else:
            metadata["collection"]["shared_artifacts"] = f"collection_{task}_{token}"
        atomic_json(episode_path / "metadata.json", metadata)
        os.rename(episode_path, destination)
        published.append(destination)
        print(f"[collect] {destination}: {metadata['outcome']['status']}", flush=True)
    if len(episodes) > 1:
        shared = output / f"collection_{task}_{token}"
        relocate_artifact_references(artifact_root, artifact_root, shared / "artifacts")
        os.rename(staging, shared)
    else:
        shutil.rmtree(staging)
    return published


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task", required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--output", default="outputs/datasets")
    parser.add_argument("--gelsight", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--nominal-start", action="store_true",
                        help="every episode at the task's nominal start, also for tasks that take --seed")
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.episodes < 1 or args.seed < 0 or args.seed + args.episodes > 2**32:
        parser.error("episodes must be positive and seeds must be in [0, 2**32)")
    if args._worker:
        _worker(args.task, args.seed, random_start=not args.nominal_start)
        return 0
    for index in range(args.episodes):
        collect_episode(args.task, args.output, seed=args.seed + index, gelsight=args.gelsight,
                        random_start=not args.nominal_start)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
