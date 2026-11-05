"""Task variations: the website's axes mapped onto task options, and the run command's handling of them.

    python -m pytest packages/force-bench/tests/test_variations.py

CPU only; no simulation runs.
"""

import pytest

from force_bench.appendix_tasks.registry import APPENDIX
from force_bench.appendix_tasks.variations import AXES, apply, axes_for, describe, find_axis, website_task
from force_bench.cli import _run_options


def test_set_replaces_the_rows_own_option():
    argv, slugs = apply("plug_in_charger_blocks", ("--block", "1", "--spring", "0.85"),
                        ["Insertion force=0.4mm", "Object geometry=block 3"])
    assert argv == ("--spring", "0.4", "--block", "3")
    assert slugs == ["force-0.4mm-indent-spring", "geometry-charging-block-3"]


def test_values_match_by_label_part_or_website_name():
    force = find_axis("plug_in_usb", "insertion force")
    assert force.find("0.6").argv == ("--spring", "0.6")
    assert force.find("More force").argv == ("--spring", "0.4")
    with pytest.raises(KeyError):
        force.find("mm")                                    # matches all three springs
    with pytest.raises(KeyError):
        find_axis("plug_in_usb", "Number of objects")       # not an axis of this task
    target = find_axis("turn_dial_to_target", "Target position")
    assert target.find("45").argv == ("--target-deg", "45")
    with pytest.raises(KeyError):
        target.find("ninety")


def test_every_row_option_is_a_value_of_its_axes():
    for spec in APPENDIX:
        opts = dict(zip(spec.argv[::2], spec.argv[1::2]))
        for axis in axes_for(spec.module):
            if axis.numeric or not (axis.flags() & opts.keys()):
                continue
            assert axis.current(spec.argv) != "default", (spec.name, axis.name)


# the variation axes the paper's Table I and the task pages use
TABLE_I = {"Required rotation amount", "Rotation force", "Insertion force", "Object geometry", "Object type",
           "Number of objects", "Scene arrangement"}


def test_axis_names_follow_the_website():
    """Every axis uses the website's vocabulary: an axis its task page lists, or one of Table I's (the asset page
    offers e.g. a bulb's rotation amounts that the bulb's task page does not list), or it is a task parameter."""
    for spec in APPENDIX:
        site = {v["axis"] for v in website_task(spec.number).get("variations", [])}
        for axis in axes_for(spec.module):
            if axis.kind != "task parameter":
                assert axis.name in site | TABLE_I, (spec.name, axis.name, site)


def test_info_marks_the_rows_choice():
    lines = describe("turn_doorknob_open_door_flat", ("--unlock", "80", "--name", "x"))
    assert any("*80°" in line for line in lines)
    assert all(m in AXES for m in ("plug_in_charger_blocks", "plug_in_usb", "turn_dial_to_target"))


def test_run_options_are_split_from_the_modules_own():
    sets, name, seed, extra = _run_options(["--set", "Insertion force=0.85mm", "--set=Object geometry=block 2",
                                            "--name", "mine", "--seed", "3", "--offset-mm", "1,0"])
    assert sets == ["Insertion force=0.85mm", "Object geometry=block 2"]
    assert (name, seed, extra) == ("mine", 3, ["--offset-mm", "1,0"])


def test_seeded_start_is_reproducible_and_collision_free():
    from force_bench.appendix_tasks.common import Placement, sample_start
    places = {"a": Placement((0.0, 0.0), dxy=0.02, dyaw=0.5, radius=0.03),
              "b": Placement((0.08, 0.0), dxy=0.02, radius=0.03)}
    assert sample_start(places, None) == {"a": (0.0, 0.0, 0.0), "b": (0.08, 0.0, 0.0)}
    one, two = sample_start(places, 7), sample_start(places, 7)
    assert one == two and one != sample_start(places, 8)
    (ax, ay, _), (bx, by, _) = one["a"], one["b"]
    assert ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5 >= 0.065
    with pytest.raises(RuntimeError):
        sample_start({"a": Placement((0, 0), radius=0.05), "b": Placement((0.01, 0), radius=0.05)}, 1, tries=10)


def test_scattered_objects_are_drawn_in_one_square():
    from force_bench.appendix_tasks.common import Placement, sample_start
    places = {k: Placement(xy, dxy=0.075, radius=0.03, centre=(0.0, 0.0)) for k, xy in (("a", (0.3, 0.3)),
                                                                                          ("b", (-0.3, 0.3)))}
    assert sample_start(places, None) == {"a": (0.3, 0.3, 0.0), "b": (-0.3, 0.3, 0.0)}
    for seed in range(10):
        draw = sample_start(places, seed)
        assert all(abs(x) <= 0.075 and abs(y) <= 0.075 for x, y, _ in draw.values())


def test_multi_object_seeded_starts_keep_the_nominal_layout_and_stay_apart():
    from itertools import combinations
    from force_bench.appendix_tasks import stack_cups as SC
    assert [tuple(v) for v in SC.draw_cups(None)] == [tuple(v) for v in SC.CUP_XY]
    for seed in (1, 2, 3, 11):
        cups = SC.draw_cups(seed)
        assert all(((a - b) ** 2).sum() ** 0.5 >= 2 * SC.CUP_R for a, b in combinations(cups, 2))
        assert all(abs(c[i] - SC.SQUARE_C[i]) <= SC.SEED_DXY for c in cups for i in (0, 1))


def test_seeded_task_modules_draw_reproducible_starts_inside_their_ranges():
    """The in-hand insertion, dial and threaded tasks: no seed = the nominal pose; a seed = the same draw every
    time, off the nominal pose, inside the 0.15 x 0.15 m square (and the yaw range); the modules take --seed."""
    from force_bench.appendix_tasks import _kit_thread, plug_in_usb, turn_dial_to_target
    from force_bench.appendix_tasks.common import sample_start
    from force_bench.appendix_tasks.variations import SEEDED
    places = {"plug_in_usb": plug_in_usb.SLOT_PLACE, "turn_dial_to_target": turn_dial_to_target.DIAL_PLACE,
              "install_light_bulb": _kit_thread.AXIS_PLACE, "put_bottle_cap_onto_bottle": _kit_thread.AXIS_PLACE}
    assert plug_in_usb.SLOT_PLACE.xy == plug_in_usb.SLOT_XY and plug_in_usb.SLOT_PLACE.yaw == plug_in_usb.SLOT_YAW
    assert turn_dial_to_target.DIAL_PLACE.xy == turn_dial_to_target.DIAL_XY
    assert list(_kit_thread.AXIS_PLACE.xy) == list(_kit_thread.AXIS_XY)
    for module, p in places.items():
        assert module in SEEDED
        assert sample_start({"o": p}, None)["o"] == (p.xy[0], p.xy[1], p.yaw)
        assert p.dxy <= 0.075
        for seed in (1, 2, 3):
            x, y, yaw = sample_start({"o": p}, seed)["o"]
            assert (x, y, yaw) == sample_start({"o": p}, seed)["o"]
            assert (x, y) != p.xy
            assert abs(x - p.xy[0]) <= p.dxy and abs(y - p.xy[1]) <= p.dxy and abs(yaw - p.yaw) <= p.dyaw
    assert _kit_thread.AXIS_PLACE.dyaw == 0.0                       # round fixtures: position only
    assert "install_light_bulb_from_table" not in SEEDED
    assert plug_in_usb.parse(["--seed", "2"]).seed == 2 and plug_in_usb.parse([]).seed is None
    assert _kit_thread.parse("cap", "hand", ["--seed", "3"]).seed == 3
    with pytest.raises(SystemExit):
        _kit_thread.parse("bulb", "table", ["--seed", "1"])        # the from-table start has no seeded start
