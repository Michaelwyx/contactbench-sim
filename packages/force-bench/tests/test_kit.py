"""The kit library: catalog, variant choice, print-mass model, and the website-config parser.

Run explicitly with:
    python -m pytest packages/force-bench/tests/test_kit.py

Everything but the last test runs on the CPU; the last one settles one asset through Newton and needs CUDA.
"""

import hashlib

import pytest
import trimesh
import yaml

from force_bench.kit import load_kit
from force_bench.kit.catalog import KIT_DIR, print_mass, sdf_resolution
from force_bench.kit.importer import js_object_to_json, parse_asset_page
from force_bench.kit.mechanisms import module_for


@pytest.fixture(scope="module")
def kit():
    return load_kit()


def test_every_variant_resolves_to_files_matching_the_catalog(kit):
    assert len(kit) == 49
    files = {}
    for key, a in kit.items():
        for choice in a.variant_choices():
            parts = a.parts(choice)
            assert parts, (key, choice)
            files.update({p.file: p.sha256 for p in parts})
    for path, sha in files.items():
        assert hashlib.sha256(path.read_bytes()).hexdigest() == sha, path


def test_sim_yaml_covers_the_catalog_and_names_real_models(kit):
    sim = yaml.safe_load((KIT_DIR / "sim.yaml").read_text())
    assert set(sim["assets"]) == set(kit)
    for a in kit.values():
        assert a.sim.get("kind") in ("free", "fixture", "assembly", "mechanism"), a.key
        if a.sim["kind"] == "mechanism":
            assert callable(module_for(a).build), a.key


def test_variant_choice_by_label_by_fragment_and_by_slot(kit):
    outlet = kit["outlet"]
    exact = outlet.parts({"Insertion force": "0.7mm indent spring"})
    assert [p.name for p in outlet.parts("0.7mm")] == [p.name for p in exact]
    assert len(outlet.variant_choices()) == 4

    bulb = kit["lightbulb-1"]                 # one file per (diameter, turns) combination
    assert len(bulb.variant_choices()) == 15
    assert [p.name for p in bulb.parts({"Lightbulb geometry": "55mm diameter",
                                        "Amount of rotation to tighten": "2 revolutions"})] == ["lightbulb_1_55mm_2_rev"]

    module = kit["doorknob-module"]           # a handle and a doorknob share the grip slot
    picked = module.resolve({"Doorknob geometry": "Doorknob 1, 55mm diameter"})
    assert picked["Door handle geometry"] is None
    assert picked["Doorknob geometry"].label == "Doorknob 1, 55mm diameter"
    as_built = {k: (v.label if v else None) for k, v in picked.items()}      # an AssetHandle's choice
    assert [p.name for p in module.parts(as_built)] == [p.name for p in module.parts("Doorknob 1, 55mm")]
    with pytest.raises(KeyError):
        outlet.parts("no such spring")


def test_print_mass_walls_plus_infill():
    cube = trimesh.creation.box((0.01, 0.01, 0.01))          # 10 mm cube
    shell = 6e-4 * 0.0008                                      # area x 0.8 mm
    assert print_mass(cube) == pytest.approx(1240.0 * (shell + 0.2 * (1e-6 - shell)))
    thin = trimesh.creation.box((0.05, 0.03, 0.001))           # thinner than two walls: solid
    assert print_mass(thin) == pytest.approx(1240.0 * thin.volume)
    assert sdf_resolution(cube, cell=0.0004) == 64                 # 25 cells, raised to the minimum
    assert sdf_resolution(trimesh.creation.box((0.3, 0.1, 0.1)), cell=0.0004) == 384


def test_kit_masses_are_plausible(kit):
    from force_bench.kit.build import part_mass
    card, cup = kit["card"], kit["cup"]
    assert part_mass(card, card.parts()[0]) == pytest.approx(0.0069, abs=0.0005)
    assert part_mass(cup, cup.parts()[0]) == pytest.approx(0.0152, abs=0.0010)


def test_js_object_literal_parser():
    js = """defineAssetPage({
      // a comment
      dir: 'website_assets/benchmark_assets/',   /* another */
      colors: { white: [246, 246, 243], },
      assets: [
        { key: "card", name: 'Card', note: 'it\\'s thin', default: true, stl: null, },
      ],
    });"""
    page = parse_asset_page(js)
    assert page["dir"] == "website_assets/benchmark_assets/"
    assert page["colors"]["white"] == [246, 246, 243]
    assert page["assets"][0] == {"key": "card", "name": "Card", "note": "it's thin", "default": True, "stl": None}
    with pytest.raises(ValueError):
        js_object_to_json("{ a: someVariable }")


def test_one_asset_settles_on_the_table(tmp_path):
    wp = pytest.importorskip("warp")
    if not wp.is_cuda_available():
        pytest.skip("signed-distance collision requires CUDA")
    from force_bench.kit.check import check_choice
    report = check_choice("card", {}, out_root=tmp_path, render=False)
    assert report["pass"], report["checks"]
