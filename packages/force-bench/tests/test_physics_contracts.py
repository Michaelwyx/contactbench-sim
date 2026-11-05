"""CPU checks for force decomposition and task-local tactile configuration."""
import json
import os
import subprocess
import sys
from types import SimpleNamespace

import numpy as np
import pytest


def test_probe_normal_is_vector_difference():
    from force_bench.probes import ForceProbe
    probe = ForceProbe.__new__(ForceProbe)
    probe.history = []
    probe.sensor = SimpleNamespace(
        update=lambda *args: None,
        total_force=SimpleNamespace(numpy=lambda: np.array([[3., 0., 4.]])),
        total_force_friction=SimpleNamespace(numpy=lambda: np.array([[3., 0., 0.]])))
    sample = probe.sample(None, None, 0.0)
    assert sample["force"] == pytest.approx(5)
    assert sample["normal"] == pytest.approx(4)
    assert sample["friction"] == pytest.approx(3)


@pytest.mark.parametrize("first", ["force_bench.rig", "force_bench.tactile.task"])
def test_tactile_defaults_are_local_and_import_order_independent(first):
    source = f"""
import {first}
from force_bench.rig import Rig, MJ_IMPRATIO, COLLIDE_SUBSTEP_FORCE
from force_bench.tactile.task import make_rig
import json
bare = Rig('bare')
gel = make_rig('gel')
print(json.dumps(dict(bare=bare.solver_options,
                     bare_cadence=bare.effective_collide_every_substep(),
                     gel=gel.solver_options,
                     gel_cadence=gel.effective_collide_every_substep(),
                     global_impratio=MJ_IMPRATIO, global_cadence=COLLIDE_SUBSTEP_FORCE)))
"""
    env = {**os.environ, "TB_GELSIGHT": "1"}
    env.pop("TB_MJ_IMPRATIO", None)
    env.pop("TB_COLLIDE_SUBSTEP", None)
    result = subprocess.run([sys.executable, "-c", source], env=env,
                            capture_output=True, text=True, check=True)
    data = json.loads(result.stdout.strip().splitlines()[-1])
    assert data == {"bare": {}, "bare_cadence": False,
                    "gel": {"impratio": 10.0}, "gel_cadence": True,
                    "global_impratio": None, "global_cadence": None}
