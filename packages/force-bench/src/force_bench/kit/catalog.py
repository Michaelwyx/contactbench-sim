"""The printable ContactBench kit as data: assets, parts, variants, and the physical properties of each part.

Two files describe the kit (see docs/assets.md):

* ``assets/kit/catalog.json``: the website's asset page, transcribed by ``force-bench kit import``.  Every asset
  with its parts, its variant groups (an outlet's springs, a lock's rotation amounts, a door handle or a doorknob),
  and their colours.  Part files are the published STLs: millimetres, Z up, in the asset's CAD assembly frame.
* ``assets/kit/sim.yaml``: how each asset is simulated (hand-written): free object, fixture, or mechanism; contact
  friction; collision resolution; a measured mass where one exists; the mechanism model and its per-variant
  parameters.

Choosing a variant::

    kit = load_kit()
    a = kit["outlet"]
    a.groups                                   # [Group('Insertion force', ['0.4mm indent spring', ...])]
    parts = a.parts({"Insertion force": "0.7mm indent spring"})
    parts = a.parts("0.7mm")                   # the same: a label (or a unique part of one) picks its group

Groups that share a slot (a door handle or a doorknob on the same shaft) are alternatives: choosing a variant in
one of them leaves the others empty.
"""

from __future__ import annotations

import functools
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh
import yaml

from force_bench.paths import ASSETS_ROOT

KIT_DIR = ASSETS_ROOT / "kit"
MM = 1e-3


@dataclass(frozen=True)
class Part:
    """One printable part file."""
    name: str
    file: Path
    rgb: tuple[float, float, float]     # 0..1
    sha256: str

    def mesh(self) -> trimesh.Trimesh:
        """The part in METRES, in its asset's CAD assembly frame (a copy: safe to transform)."""
        return _load_mesh(str(self.file)).copy()


@functools.lru_cache(maxsize=None)
def _load_mesh(path: str) -> trimesh.Trimesh:
    m = trimesh.load(path, force="mesh", process=True)
    m.apply_scale(MM)
    return m


@dataclass
class Variant:
    label: str
    parts: list[Part]
    default: bool = False


@dataclass
class Group:
    variation: str
    variants: list[Variant]
    slot: str | None = None

    def labels(self) -> list[str]:
        return [v.label for v in self.variants]

    def default(self) -> Variant:
        return next((v for v in self.variants if v.default), self.variants[0])

    def find(self, label: str) -> Variant | None:
        exact = [v for v in self.variants if v.label == label]
        if exact:
            return exact[0]
        loose = [v for v in self.variants if label.lower() in v.label.lower()]
        return loose[0] if len(loose) == 1 else None


@dataclass
class Asset:
    key: str
    name: str
    note: str
    base: list[Part]
    groups: list[Group]
    combinations: list[tuple[tuple[str, ...], list[Part]]]
    sim: dict = field(default_factory=dict)

    # ── choosing variants ────────────────────────────────────────────────────────────────────────────────────
    def resolve(self, choice: dict[str, str] | str | None = None) -> dict[str, Variant | None]:
        """{group variation: chosen Variant (None for an unused alternative in a shared slot)}.

        A label of None leaves its group empty, so the ``choice`` of an AssetHandle can be passed back as is."""
        if isinstance(choice, str):
            choice = {"": choice}
        choice = dict(choice or {})
        picked: dict[str, Variant | None] = {}
        for key, label in list(choice.items()):
            if label is None:
                if not any(g.variation == key for g in self.groups):
                    raise KeyError(f"{self.key}: no variation {key!r} in {[g.variation for g in self.groups]}")
                picked[key] = None
                continue
            groups = [g for g in self.groups if g.variation == key] if key else \
                [g for g in self.groups if g.find(label)]
            if len(groups) != 1:
                raise KeyError(f"{self.key}: cannot place {label!r} in one group of "
                               f"{[g.variation for g in self.groups]}")
            v = groups[0].find(label)
            if v is None:
                raise KeyError(f"{self.key}: no variant {label!r} in {groups[0].variation!r}; "
                               f"one of {groups[0].labels()}")
            picked[groups[0].variation] = v
        slots_taken = {g.slot for g in self.groups if picked.get(g.variation) is not None and g.slot}
        for g in self.groups:
            if g.variation in picked:
                continue
            if g.slot and g.slot in slots_taken:
                picked[g.variation] = None
                continue
            picked[g.variation] = g.default()
            if g.slot:
                slots_taken.add(g.slot)
        return picked

    def parts(self, choice: dict[str, str] | str | None = None) -> list[Part]:
        """Base parts plus the parts of the chosen variants (defaults where not given)."""
        picked = self.resolve(choice)
        out = list(self.base)
        if self.combinations:
            labels = tuple(picked[g.variation].label for g in self.groups)
            for combo, parts in self.combinations:
                if tuple(combo) == labels:
                    return out + parts
            raise KeyError(f"{self.key}: no combination {labels}")
        for g in self.groups:
            v = picked[g.variation]
            if v is not None:
                out += v.parts
        return out

    def variant_choices(self) -> list[dict[str, str]]:
        """Every distinct choice worth checking: each variant of each group once, others at their defaults
        (all combinations for an asset defined by combinations)."""
        if self.combinations:
            return [dict(zip([g.variation for g in self.groups], combo)) for combo, _ in self.combinations]
        if not self.groups:
            return [{}]
        out = []
        for g in self.groups:
            for v in g.variants:
                out.append({g.variation: v.label})
        return out

    def part(self, name: str, choice=None) -> Part:
        for p in self.parts(choice):
            if p.name == name:
                return p
        raise KeyError(f"{self.key}: no part {name!r}")


class Kit(dict):
    """{asset key: Asset}, plus the defaults of sim.yaml."""

    def __init__(self, assets: dict[str, Asset], sim_defaults: dict, catalog: dict):
        super().__init__(assets)
        self.sim_defaults = sim_defaults
        self.catalog = catalog


def _rgb(c, colors) -> tuple[float, float, float]:
    if isinstance(c, str):
        c = colors.get(c)
    if not c:
        c = [200, 200, 200]
    return tuple(float(x) / 255.0 for x in c[:3])


@functools.lru_cache(maxsize=None)
def load_kit(kit_dir: str | None = None) -> Kit:
    root = Path(kit_dir) if kit_dir else KIT_DIR
    cat_path = root / "catalog.json"
    if not cat_path.exists():
        raise FileNotFoundError(f"{cat_path} not found: run `force-bench kit import`")
    cat = json.loads(cat_path.read_text())
    sim_path = root / "sim.yaml"
    sim = yaml.safe_load(sim_path.read_text()) if sim_path.exists() else {}
    defaults = sim.get("defaults", {})
    colors = cat["colors"]

    def parts_of(lst):
        return [Part(p["name"], root / p["file"], _rgb(p.get("color_name") or p.get("color"), colors), p["sha256"])
                for p in lst or []]

    assets = {}
    for key, a in cat["assets"].items():
        groups = [Group(g["variation"], [Variant(v["label"], parts_of(v.get("parts")), v.get("default", False))
                                         for v in g["variants"]], g.get("slot"))
                  for g in a.get("groups", [])]
        combos = [(tuple(c["variants"]), parts_of(c["parts"])) for c in a.get("combinations", [])]
        assets[key] = Asset(key, a["name"], a.get("note", ""), parts_of(a.get("parts")), groups, combos,
                            (sim.get("assets") or {}).get(key, {}))
    return Kit(assets, defaults, cat)


# ── physical properties of a printed part ─────────────────────────────────────────────────────────────────────────
def print_mass(mesh: trimesh.Trimesh, *, density: float = 1240.0, infill: float = 0.20,
               wall: float = 0.0008) -> float:
    """Mass [kg] of an FDM print of ``mesh`` (metres): solid walls of thickness ``wall`` plus sparse infill.

    The wall volume is approximated by surface area x wall thickness (capped at the part volume, so thin parts
    such as cards and keys come out solid).  PLA density 1240 kg/m^3; the defaults match the site's print profile
    (0.4 mm nozzle, two perimeters, 20 % infill).  A measured mass in sim.yaml overrides this estimate."""
    volume = float(abs(mesh.volume))
    shell = min(volume, float(mesh.area) * wall)
    return density * (shell + infill * (volume - shell))


def sdf_resolution(mesh: trimesh.Trimesh, cell: float = 0.0004, lo: int = 64, hi: int = 384) -> int:
    """Signed-distance-field resolution (cells along the longest side) giving roughly ``cell`` metres per cell."""
    n = int(np.ceil(float(mesh.extents.max()) / cell))
    return int(min(hi, max(lo, n)))
