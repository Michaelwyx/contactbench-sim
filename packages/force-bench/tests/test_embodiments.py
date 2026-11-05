"""The embodiment registry and its selection (CPU only, no simulation).

    python -m pytest packages/force-bench/tests/test_embodiments.py
"""

import re
from pathlib import Path

import numpy as np
import pytest

from force_bench import embodiments as E
from force_bench.appendix_tasks.registry import APPENDIX, BY_NAME
from force_bench.cli import _embodiment_option, _run_plan

TASK_DIR = Path(__file__).resolve().parents[1] / "src" / "force_bench" / "appendix_tasks"


def test_the_default_is_the_stock_fingers_and_every_row_supports_it():
    assert E.DEFAULT == "franka" and E.names()[0] == "franka"
    for spec in APPENDIX:
        assert E.DEFAULT in E.supported(spec.module)
    assert E.supported("stack_cups") == ("franka",)
    assert E.tasks_supporting("franka") is None                     # every task
    assert E.tasks_supporting("finray") == ["retrieve_utensil_from_bin"]


def test_an_unknown_name_is_an_error_that_lists_the_names():
    with pytest.raises(KeyError, match="one of franka, finray"):
        E.get("soft")
    with pytest.raises(SystemExit, match="unknown embodiment 'soft'"):
        _run_plan("retrieve_utensil_from_bin", BY_NAME["retrieve_utensil_from_bin"],
                  ["--embodiment", "soft"])


def test_a_task_that_did_not_opt_in_refuses():
    with pytest.raises(ValueError, match="does not support the 'finray' embodiment"):
        E.check("stack_cups", "finray")
    with pytest.raises(SystemExit, match="stack_cups: this task does not support the 'finray'"):
        _run_plan("stack_cups", BY_NAME["stack_cups"], ["--embodiment", "finray"])


def test_the_default_run_is_unchanged():
    spec = BY_NAME["retrieve_utensil_from_bin"]
    assert _run_plan(spec.name, spec, []) == ([], None)
    assert _run_plan(spec.name, spec, ["--embodiment", "franka"]) == ([], None)   # explicit default: same
    assert _run_plan(spec.name, spec, ["--seed", "1"]) == (["--seed", "1"], f"{spec.name}__seed1")
    assert E.run_name(spec.name, "franka") == spec.name


def test_embodiment_combines_with_seed_and_set_into_its_own_folder():
    spec = BY_NAME["retrieve_utensil_from_bin"]
    argv, run = _run_plan(spec.name, spec, ["--embodiment=finray", "--seed", "2"])
    assert argv == ["--seed", "2", "--embodiment", "finray"] and run == f"{spec.name}__finray_seed2"
    argv, run = _run_plan(spec.name, spec, ["--set", "Number of objects=11", "--embodiment", "finray",
                                            "--hold", "5"])
    assert argv[-2:] == ["--hold", "5"] and argv[argv.index("--embodiment") + 1] == "finray"
    assert argv[argv.index("--n-objects") + 1] == "11"
    assert run.startswith(f"{spec.name}__finray_") and run != spec.name
    assert E.run_name(spec.name, "finray") == f"{spec.name}__finray"
    assert _embodiment_option(["--seed", "1", "--embodiment", "finray"]) == ("finray", ["--seed", "1"])


def test_every_opted_in_task_is_a_row_and_takes_the_option():
    modules = {s.module for s in APPENDIX}
    for module, names in E.TASKS.items():
        assert module in modules and E.DEFAULT in names and set(names) <= set(E.names())
        src = (TASK_DIR / f"{module}.py").read_text()
        assert re.search(rf'embodiments\.add_argument\(ap, "{module}"\)', src), module


def test_registry_numbers_agree_with_the_implementations():
    for name in E.names():
        info, fingers = E.get(name), E.load(name)
        assert fingers.name == name
        assert fingers.tip_below_tcp * 1000 == pytest.approx(info.tip_below_tcp_mm, abs=0.05)
        assert 2 * fingers.tip_half_width * 1000 == pytest.approx(info.finger_width_mm, abs=0.05)
        assert fingers.tip_thickness * 1000 == pytest.approx(info.tip_thickness_mm, abs=0.05)


def test_the_stock_fingers_are_the_task_constants():
    """The default embodiment must reproduce the fingertip geometry the tasks are written with."""
    from force_bench.appendix_tasks import _hang_plate as HP
    f = E.load("franka")
    assert f.tip_below_tcp == HP.TIP_BELOW_TCP and f.face_offset == HP.PAD_FACE_OFF
    assert (f.tip_half_width, f.tip_thickness, f.tip_top) == (0.00875, 0.01518, 0.0090)
    assert f.is_finger("fr3/fr3_leftfinger") and not f.is_finger("fr3/fr3_link6")
    # the envelope: the fingertip block and the finger behind it, both sides
    q = 0.012
    boxes = f.boxes(q)
    assert len(boxes) >= 4
    lo, hi = boxes[0]
    assert np.array_equal(lo, [-f.tip_half_width, q, -f.tip_top])
    assert np.array_equal(hi, [f.tip_half_width, q + f.tip_thickness, f.tip_below_tcp])


def test_the_task_2_scene_takes_every_arrangement_with_either_embodiment():
    from force_bench.appendix_tasks import retrieve_utensil_from_bin as U
    for arrangement in U.ARRANGEMENTS:
        for embodiment in E.supported("retrieve_utensil_from_bin"):
            task = U.Task(arrangement, None, None, None, None, embodiment)
            assert task.embodiment == embodiment and task.n == U.ARRANGEMENTS[arrangement].n
    with pytest.raises(SystemExit, match="--n-objects"):
        U.Task("horizontal-chopsticks", None, 30, None, None)
    # the scene has no scripted policy: no verdict, and run-all and collect leave it out
    assert not BY_NAME["retrieve_utensil_from_bin"].scripted
    assert all(s.scripted for s in APPENDIX if s.name != "retrieve_utensil_from_bin")
