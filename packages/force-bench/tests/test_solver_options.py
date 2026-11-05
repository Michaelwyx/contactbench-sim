"""CPU regression for per-model GelSight solver defaults across task rebuilds."""
from types import SimpleNamespace

import pytest

from force_bench import rig as core


@pytest.fixture
def solver_factory(monkeypatch):
    def build(model, **options):
        return SimpleNamespace(model=model, options=options)
    monkeypatch.setattr(core.newton.solvers, "SolverMuJoCo", build)
    monkeypatch.setattr(core, "MJ_IMPRATIO", None)
    monkeypatch.setattr(core, "MJ_CONDIM", None)
    monkeypatch.setattr(core, "MU_TORSIONAL", None)
    monkeypatch.setattr(core, "mujoco_contact_tuning", lambda model: {})
    return core.make_solver


def test_bare_solver_default_is_not_changed_by_another_model(solver_factory):
    gel = SimpleNamespace(_force_bench_solver_options={"impratio": 10.0})
    bare = SimpleNamespace()
    assert solver_factory(gel).options["impratio"] == 10.0
    assert solver_factory(bare).options["impratio"] == 1.0


def test_sensor_solver_rebuild_keeps_model_defaults_and_records_actual_options(solver_factory):
    model = SimpleNamespace(_force_bench_solver_options={"impratio": 10.0})
    rebuilt = solver_factory(model, iterations=60)
    assert rebuilt.options["impratio"] == 10.0
    assert rebuilt._force_bench_effective_options == rebuilt.options
    assert rebuilt._force_bench_effective_options["iterations"] == 60


def test_explicit_task_override_wins_over_model_default(solver_factory):
    model = SimpleNamespace(_force_bench_solver_options={"impratio": 10.0})
    solver = solver_factory(model, impratio=7.0)
    assert solver.options["impratio"] == 7.0
    assert solver._force_bench_effective_options["impratio"] == 7.0


def test_explicit_run_override_still_wins(solver_factory, monkeypatch):
    model = SimpleNamespace(_force_bench_solver_options={"impratio": 10.0})
    monkeypatch.setattr(core, "MJ_IMPRATIO", 3.0)
    assert solver_factory(model, impratio=7.0).options["impratio"] == 3.0
