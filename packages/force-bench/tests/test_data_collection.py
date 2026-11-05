"""CPU tests for demonstration alignment, measured state and fail-closed publication."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from force_bench.data import EpisodeWriter, discover_episodes, load_episode
from force_bench.data.collect import qa_outcome
from force_bench.data.collection import RigCollector, RigObservation, read_tactile


def metadata():
    return {"task": "example", "action_dim": 2, "action_type": "joint_position_target"}


def writer(output, **kwargs):
    return EpisodeWriter(output, metadata(), [10, 20, 30], **kwargs)


def test_actions_pair_with_before_and_after_observations(tmp_path):
    w = writer(tmp_path)
    action = np.array([1, 2], dtype=float)
    next_state = np.array([11, 21, 31], dtype=float)
    w.append(action, next_state, timestamp=0.1)
    action[:] = -100  # Caller reuses its device/host command buffers.
    next_state[:] = -100
    w.append([3, 4], [12, 22, 32], timestamp=0.2)
    episode = load_episode(w.finish())
    np.testing.assert_array_equal(episode["observations"], [[10, 20, 30], [11, 21, 31], [12, 22, 32]])
    np.testing.assert_array_equal(episode["actions"], [[1, 2], [3, 4]])
    np.testing.assert_array_equal(episode["truncated"], [False, True])
    assert not episode["terminated"].any()
    assert episode["observations"].dtype == np.float32
    assert episode["timestamps"].dtype == np.float64
    assert episode.metadata["outcome"]["success"] is None


def test_failed_save_never_publishes_a_training_episode(tmp_path, monkeypatch):
    w = writer(tmp_path)
    w.append([0, 0], [1, 1, 1], timestamp=0.1)
    def fail(*args, **kwargs):
        raise OSError("disk full")
    monkeypatch.setattr(np, "savez_compressed", fail)
    with pytest.raises(OSError, match="disk full"):
        w.finish()
    assert not discover_episodes(tmp_path)
    assert not list(tmp_path.iterdir())


def test_only_explicit_pass_is_success_and_staging_is_hidden(tmp_path):
    for i, outcome in enumerate((None, {"status": "failed", "success": False}, {"status": "passed", "success": True})):
        w = writer(tmp_path, episode_id=f"episode_{i}")
        w.append([0, 0], [1, 1, 1], timestamp=0.1)
        w.finish(outcome=outcome)
    hidden = tmp_path / ".collecting-aborted"
    w = writer(hidden)
    w.append([0, 0], [1, 1, 1], timestamp=0.1)
    w.finish(outcome={"status": "passed", "success": True})
    assert len(discover_episodes(tmp_path)) == 3
    assert [p.name for p in discover_episodes(tmp_path, success_only=True)] == ["episode_2"]


def test_checksum_detects_mixed_or_replaced_arrays(tmp_path):
    w = writer(tmp_path)
    w.append([0, 0], [1, 1, 1], timestamp=0.1)
    path = w.finish()
    with (path / "episode.npz").open("ab") as stream:
        stream.write(b"changed")
    with pytest.raises(ValueError, match="checksum"):
        load_episode(path)


def test_timestamp_nan_and_terminal_contract(tmp_path):
    w = writer(tmp_path)
    with pytest.raises(ValueError, match="non-finite"):
        w.append([np.nan, 0], [1, 1, 1], timestamp=0.1)
    with pytest.raises(ValueError, match="timestamp"):
        w.append([0, 0], [1, 1, 1], timestamp=0.0)
    w.append([0, 0], [1, 1, 1], timestamp=0.1, terminated=True)
    with pytest.raises(ValueError, match="terminal"):
        w.append([0, 0], [2, 2, 2], timestamp=0.2)
    ep = load_episode(w.finish())
    assert ep["terminated"][-1] and not ep["truncated"][-1]


def fake_rig():
    model = SimpleNamespace(joint_label=[f"fr3/joint{i}" for i in range(9)],
                            body_label=["fr3/base", "target"], joint_q=np.full(9, -999.0))
    return SimpleNamespace(name="fake", model=model, fps=60, substeps=32,
                           solver=SimpleNamespace(), solver_options={},
                           s0=SimpleNamespace(joint_q=np.arange(9, dtype=float), joint_qd=np.arange(9, dtype=float) + 1,
                                              body_q=np.array([[0, 0, 0, 0, 0, 0, 1], [1, 2, 3, 0, 0, 0, 1]], dtype=float)),
                           control=SimpleNamespace(joint_target_q=np.arange(9, dtype=float) + 2,
                                                   joint_target_qd=np.zeros(9), joint_f=np.zeros(9)),
                           effective_collide_every_substep=lambda: True)


def test_measured_robot_observation_never_uses_model_initial_q():
    rig = fake_rig()
    obs = RigObservation(rig)
    measured = obs.sample()
    np.testing.assert_array_equal(measured[:9], rig.s0.joint_q)
    np.testing.assert_array_equal(measured[9:18], rig.s0.joint_qd)
    np.testing.assert_array_equal(measured[18:], rig.s0.body_q[1])
    assert len(obs.metadata["observation_names"]) == 25
    assert obs.metadata["task_body_labels"] == ["target"]


def test_collector_captures_command_before_solver_and_actual_next_state(tmp_path, monkeypatch):
    monkeypatch.setattr("force_bench.data.collection.provenance", lambda: {})
    rig = fake_rig()
    c = RigCollector(rig, tmp_path)
    initial = rig.s0.joint_q.copy()
    action = rig.control.joint_target_q.copy()
    c.after_control()
    rig.control.joint_target_q[:] = 123  # A solver wrapper can mutate targets later.
    rig.s0.joint_q[:] += 0.25
    c.after_step(1 / 60)
    episode = load_episode(c.finish())
    np.testing.assert_array_equal(episode["actions"][0], action)
    np.testing.assert_array_equal(episode["observations"][0, :9], initial)
    np.testing.assert_array_equal(episode["observations"][1, :9], initial + 0.25)
    assert episode["action_target_velocity"].shape == (1, 9)
    assert episode["action_feedforward"].shape == (1, 9)


def test_tactile_initial_invalid_and_area_average_preserves_mean():
    rig = fake_rig()
    rig.tactile_frames = {"gel_left": [], "gel_right": []}
    pressure, valid = read_tactile(rig)
    assert pressure.shape == (2, 16, 16)
    assert not valid.any()
    raster = np.arange(35, dtype=np.float32).reshape(5, 7)
    rig.tactile_frames["gel_left"].append(SimpleNamespace(pressure_pa=raster))
    pressure, valid = read_tactile(rig, size=3)
    assert valid.tolist() == [True, False]
    assert pressure[0].mean() == pytest.approx(raster.mean(), rel=1e-6)
    assert pressure[1].sum() == 0


def test_process_failure_beats_even_fresh_qa_pass(tmp_path):
    (tmp_path / "appendix_qa.json").write_text(json.dumps({"pass": True}))
    outcome = qa_outcome(tmp_path, returncode=1)
    assert outcome["status"] == "failed" and outcome["success"] is False
    assert outcome["source"] == "process_exit"


def test_missing_or_malformed_qa_is_never_success(tmp_path):
    assert qa_outcome(tmp_path, returncode=0)["success"] is None
    report = tmp_path / "appendix_qa.json"
    report.write_text('{"pass":"true"}')
    assert qa_outcome(tmp_path, returncode=0)["success"] is None
    report.write_text('{"pass":false}')
    assert qa_outcome(tmp_path, returncode=0)["success"] is False
    report.write_text('{"pass":true}')
    assert qa_outcome(tmp_path, returncode=0)["success"] is True


def test_published_tactile_scene_references_moved_mesh(tmp_path):
    from force_bench.data.collect import relocate_artifact_references
    old = tmp_path / ".staging" / "artifacts"
    old.mkdir(parents=True)
    (old / "target.obj").write_text("mesh")
    new = tmp_path / "published" / "artifacts"
    scene = old / "tactile_scene.npz"
    spec = [{"type": "mesh", "params": [str(old / "target.obj")], "body": "target"},
            {"type": "mesh", "params": ["/external/assets/fixture.obj"], "body": "fixture"}]
    np.savez(scene, objects_json=json.dumps(spec), pads=np.array(["left", "right"]))
    relocate_artifact_references(old, old, new)
    new.parent.mkdir()
    old.rename(new)
    with np.load(new / "tactile_scene.npz", allow_pickle=False) as archive:
        objects = json.loads(str(archive["objects_json"].item()))
    from pathlib import Path
    assert Path(objects[0]["params"][0]).is_file()
    assert objects[1]["params"][0] == "/external/assets/fixture.obj"


def test_invalid_extra_does_not_partially_append(tmp_path):
    w = writer(tmp_path, tactile=np.zeros((2, 2, 2)), tactile_valid=np.zeros(2, dtype=bool))
    with pytest.raises(ValueError, match="overwrite"):
        w.append([0, 0], [1, 1, 1], timestamp=0.1, tactile=np.ones((2, 2, 2)),
                 tactile_valid=np.ones(2, dtype=bool), extra={"actions": [1]})
    assert len(w.observations) == len(w.tactile) == 1
    assert len(w.actions) == 0
    w.append([0, 0], [1, 1, 1], timestamp=0.1, tactile=np.ones((2, 2, 2)),
             tactile_valid=np.ones(2, dtype=bool))
    assert load_episode(w.finish())["tactile"].shape == (2, 2, 2, 2)


def test_dataset_inspector_reports_invalid_checksum(tmp_path):
    from force_bench.data.inspect import inspect_dataset
    w = writer(tmp_path)
    w.append([0, 0], [1, 1, 1], timestamp=0.1)
    path = w.finish()
    assert inspect_dataset(tmp_path)["pass"] is True
    with (path / "episode.npz").open("ab") as stream:
        stream.write(b"corrupt")
    report = inspect_dataset(tmp_path)
    assert not report["pass"]
    assert report["validated_episodes"] == 0
    assert "checksum" in report["errors"][0]["error"]


def test_teleop_exit_is_unvalidated_and_preserves_guard_metadata(tmp_path, monkeypatch):
    from force_bench.teleop.franka_teleop import Example, _CollectionRigView
    monkeypatch.setattr("force_bench.data.collection.provenance", lambda: {})
    rig = fake_rig()
    example = Example.__new__(Example)
    example.model, example.solver, example.control = rig.model, rig.solver, rig.control
    example.state_0, example.sim_substeps, example.fps = rig.s0, 16, 60
    example.guard = SimpleNamespace(summary=lambda: {"guard_enabled": True, "violations": []})
    example.collection = RigCollector(_CollectionRigView(example), tmp_path, source="teleoperation")
    example.collection.writer.metadata["teleoperation"] = {"guard_toggles": []}
    example.collection.after_control()
    example.state_0.joint_q += 0.5
    example.collection.after_step(1 / 60)
    path = example.finish_collection()
    episode = load_episode(path)
    assert episode.metadata["task"] == "teleop_franka"
    assert episode.metadata["source"] == "teleoperation"
    assert episode.metadata["outcome"]["success"] is None
    assert episode.metadata["teleoperation"]["guard_summary"]["guard_enabled"] is True
    assert episode["truncated"][-1]
    assert example.collection is None


def test_teleop_error_preserves_only_completed_transitions_as_failed(tmp_path, monkeypatch):
    from force_bench.teleop.franka_teleop import Example, _CollectionRigView
    monkeypatch.setattr("force_bench.data.collection.provenance", lambda: {})
    rig = fake_rig()
    example = Example.__new__(Example)
    example.model, example.solver, example.control = rig.model, rig.solver, rig.control
    example.state_0, example.sim_substeps, example.fps = rig.s0, 16, 60
    example.guard = SimpleNamespace(summary=lambda: {"violations": []})
    example.collection = RigCollector(_CollectionRigView(example), tmp_path, source="teleoperation")
    example.collection.writer.metadata["teleoperation"] = {}
    example.collection.after_control()
    example.collection.after_step(1 / 60)
    example.collection.after_control()  # Solver fails before the next post-state.
    path = example.finish_collection(RuntimeError("solver failed"))
    episode = load_episode(path)
    assert len(episode["actions"]) == 1
    assert episode.metadata["outcome"]["status"] == "failed"
    assert episode.metadata["outcome"]["success"] is False
    assert episode.metadata["discarded_incomplete_transition"] is True


def test_physics_metadata_uses_actual_solver_and_ignores_render_settings(monkeypatch):
    from dataclasses import dataclass
    from force_bench.data.collection import physics_metadata
    monkeypatch.delenv("TB_GELSIGHT", raising=False)
    monkeypatch.setenv("TB_GELSIGHT_RENDER", "0")
    monkeypatch.setenv("GS_VIDEO", "large.mp4")
    monkeypatch.setenv("GS_PROFILE", "1")
    monkeypatch.setenv("TB_OUT_ROOT", "/output/does/not/change/physics")
    monkeypatch.setenv("TB_ARM_EFFORT", "50")
    monkeypatch.setenv("TB_STABILIZER", "1")
    monkeypatch.setenv("GS_MOUNT", "slab")
    @dataclass
    class GelConfig:
        friction_coefficient: float = 2.0
    rig = fake_rig()
    rig.solver_options = {"impratio": 10.0}
    rig.solver._force_bench_effective_options = {"impratio": 7.0, "iterations": 60}
    rig.gel_config = GelConfig()
    rig._collide_every = 2
    result = physics_metadata(rig)
    assert result["solver_options"]["impratio"] == 10.0
    assert result["effective_solver_options"]["impratio"] == 7.0
    assert result["gel_config"]["friction_coefficient"] == 2.0
    assert result["collide_every_n_substeps"] == 2
    environment = result["environment"]
    assert environment["TB_GELSIGHT"] == "0"
    assert environment["TB_ARM_EFFORT"] == "50"
    assert environment["TB_STABILIZER"] == "1"
    assert environment["GS_MOUNT"] == "slab"
    for irrelevant in ("TB_GELSIGHT_RENDER", "GS_VIDEO", "GS_PROFILE", "TB_OUT_ROOT"):
        assert irrelevant not in environment


def test_unknown_effective_solver_options_are_not_invented():
    from force_bench.data.collection import physics_metadata
    assert physics_metadata(fake_rig())["effective_solver_options"] is None


def test_source_manifest_hashes_untracked_python_and_asset_manifest(tmp_path):
    import hashlib
    from force_bench.data.collection import source_manifest
    source = tmp_path / "packages/force-bench/src/force_bench/data/new.py"
    source.parent.mkdir(parents=True)
    source.write_text("UNTRACKED = True\n")
    assets = tmp_path / "assets/manifest.yaml"
    assets.parent.mkdir()
    assets.write_text("asset: test\n")
    files = source_manifest(tmp_path)["files_sha256"]
    assert files[str(source.relative_to(tmp_path))] == hashlib.sha256(source.read_bytes()).hexdigest()
    assert files["assets/manifest.yaml"] == hashlib.sha256(assets.read_bytes()).hexdigest()
    source.write_text("UNTRACKED = False\n")
    assert source_manifest(tmp_path)["files_sha256"][str(source.relative_to(tmp_path))] != files[str(source.relative_to(tmp_path))]


def test_nested_provenance_hash_tracks_root_package_and_integration_edits(tmp_path, monkeypatch):
    import hashlib
    import subprocess
    from force_bench.data import collection

    def git(*args):
        return subprocess.run(["git", "-C", str(tmp_path), *args], check=True,
                              capture_output=True, text=True)

    nested = tmp_path / "packages/force-bench/src/force_bench/data"
    nested.mkdir(parents=True)
    package = nested.parent / "example.py"
    package.write_text("VALUE = 1\n")
    integration = tmp_path / "integrations/example.py"
    integration.parent.mkdir()
    integration.write_text("VALUE = 1\n")
    readme = tmp_path / "README.md"
    readme.write_text("Initial documentation\n")
    git("init", "-q")
    git("add", "packages", "integrations", "README.md")
    git("-c", "user.name=Provenance Test", "-c", "user.email=provenance@example.invalid",
        "commit", "-qm", "baseline")
    monkeypatch.setattr(collection, "__file__", str(nested / "collection.py"))

    clean = collection.provenance()["git"]["tracked_source_diff_sha256"]
    assert clean == hashlib.sha256(b"").hexdigest()
    package.write_text("VALUE = 2\n")
    changed = collection.provenance()["git"]["tracked_source_diff_sha256"]
    assert changed != clean, "nested -C must still hash tracked edits at repository packages/"
    git("add", "packages")
    assert collection.provenance()["git"]["tracked_source_diff_sha256"] == changed
    readme.write_text("Documentation-only changes are outside this source scope\n")
    assert collection.provenance()["git"]["tracked_source_diff_sha256"] == changed
    integration.write_text("VALUE = 2\n")
    assert collection.provenance()["git"]["tracked_source_diff_sha256"] != changed


def test_collection_draws_a_random_start_per_episode_for_seeded_tasks():
    from force_bench.data.collect import _spec, task_arguments
    usb, card = _spec("plug_in_usb_2"), _spec("retrieve_card_from_deck")
    assert task_arguments(usb, 7)[-2:] == ["--seed", "7"] and task_arguments(usb, 7)[:-2] == list(usb.argv)
    assert task_arguments(usb, 7, random_start=False) == list(usb.argv)
    assert task_arguments(card, 7) == list(card.argv)          # a task without a random start keeps its own
