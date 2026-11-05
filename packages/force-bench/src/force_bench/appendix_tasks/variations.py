"""Task variations: the website's variation axes as parameters of the simulated tasks.

The ContactBench task pages list, for every task, the variations the printed kit supports (paper Table I): an axis
such as "Insertion force", "Required rotation amount" or "Object geometry", and its values.  Each simulated task
module declares here the axes it supports, under the website's names and with the asset page's variant labels as
values, and maps every value to the module's own command-line options.  The registry rows are presets of these.

    force-bench variations                                     # every task's axes
    force-bench info plug_in_charger_block1                    # one row's axes, values and current choices
    force-bench run plug_in_charger_block1 --set "Insertion force=0.85mm" --set "Object geometry=block 3"

A value is matched like a kit variant: its exact label, or a unique part of it ("0.85mm", "block 3").  A numeric axis
("Target position") takes a number.  Combinations other than the registry rows are not validated: run them and read
the verdict.  A run with ``--set`` writes to ``outputs/appendix_tasks/<row>__<choices>/`` unless ``--name`` is given.
"""

from __future__ import annotations

import re
from dataclasses import dataclass


@dataclass(frozen=True)
class Value:
    label: str                       # the asset page's variant label where there is one
    argv: tuple[str, ...]            # the module options this value sets
    website: str = ""                # the task page's instance it realises, when it names one ("More force")
    note: str = ""


@dataclass(frozen=True)
class Axis:
    name: str                        # the task page's axis name
    kind: str                        # "behavioral" / "visual+behavioral" (as on the task page) or "task parameter"
    values: tuple[Value, ...] = ()
    flag: str = ""                   # numeric axis: the option that takes the number
    unit: str = ""
    note: str = ""

    @property
    def numeric(self) -> bool:
        return bool(self.flag)

    def flags(self) -> set[str]:
        if self.numeric:
            return {self.flag}
        return {a for v in self.values for a in v.argv[::2]}

    def find(self, text: str) -> Value:
        """The value ``text`` names: an exact label, or a unique case-insensitive part of one."""
        if self.numeric:
            try:
                float(text)
            except ValueError:
                raise KeyError(f"{self.name!r} takes a number ({self.unit}), not {text!r}") from None
            return Value(f"{text} {self.unit}".strip(), (self.flag, text))
        exact = [v for v in self.values if v.label == text]
        if exact:
            return exact[0]
        t = text.lower()
        loose = [v for v in self.values if t in v.label.lower() or (v.website and t in v.website.lower())]
        if len(loose) == 1:
            return loose[0]
        labels = [v.label for v in self.values]
        raise KeyError(f"{self.name!r}: {text!r} matches {len(loose)} values; one of {labels}")

    def current(self, argv: tuple[str, ...]) -> str:
        """The label of the value ``argv`` selects (the module default when it sets none)."""
        opts = dict(zip(argv[::2], argv[1::2]))
        if self.numeric:
            return f"{opts[self.flag]} {self.unit}".strip() if self.flag in opts else "default"
        for v in self.values:
            if all(opts.get(k) == val for k, val in zip(v.argv[::2], v.argv[1::2])):
                return v.label
        return next((v.label for v in self.values if "default" in v.note), "default")


def _springs(labels: list[str], flag: str, values: list[str], more_force: str, default: str) -> tuple[Value, ...]:
    return tuple(Value(lbl, (flag, val), website="More force" if val == more_force else "",
                       note="default" if val == default else "") for lbl, val in zip(labels, values))


_DEGREES = ("40", "48", "60", "72", "80")
_REVS = ("2", "2.5", "3", "3.5", "4")
_ROTATION_REVS = Axis(
    "Required rotation amount", "behavioral",
    tuple(Value(f"{r} revolutions", ("--rev", r), website="More rotation" if r == "4" else "",
                note="default" if r == "2" else "") for r in _REVS),
    note="the thread's lead: the same 9.4 mm of engagement in 2 to 4 turns")
_DOOR_ROTATION = Axis(
    "Required rotation amount", "behavioral",
    tuple(Value(f"{d}°", ("--unlock", d), website=f"{d} degree rotation", note="default" if d == "60" else "")
          for d in _DEGREES),
    note="the doorknob module's pinion size")
_DOOR_FORCE = Axis(
    "Rotation force", "behavioral",
    tuple(Value(f"{s}mm spring", ("--spring", s), note="default" if s == "5" else "") for s in ("5", "7.5", "10")),
    note="the printed return spring, with its measured stiffness and preload; the task page names the springs "
         "2.5 / 5 / 7.5 mm, the asset page's files 5 / 7.5 / 10 mm")

# module -> the axes it supports, in the order they are listed
AXES: dict[str, tuple[Axis, ...]] = {
    "retrieve_utensil_from_bin": (
        Axis("Scene arrangement", "visual+behavioral",
             (Value("Chopsticks in horizontal bin", ("--arrangement", "horizontal-chopsticks"), note="default"),
              Value("Chopsticks in vertical bin", ("--arrangement", "vertical-chopsticks")),
              Value("Mixed utensils in vertical bin", ("--arrangement", "vertical-mixed"))),
             note="every arrangement is filled by physics, the utensils going in one at a time in a random order and "
                  "at random places (task 2 is a scene without a scripted policy)"),
        Axis("Object type", "visual+behavioral",
             tuple(Value(t.capitalize(), ("--target", t), note="default" if t == "chopstick" else "")
                   for t in ("chopstick", "fork", "knife", "spoon")),
             note="the target utensil (default Chopstick; Spoon for mixed utensils); in the chopstick arrangements a "
                  "fork, knife or spoon target is the one other utensil in the bin"),
        Axis("Number of objects", "visual+behavioral", flag="--n-objects", unit="utensils",
             note="utensils in the bin, the target included; default 12 / 14 / 20 (the videos), 10-14 / 2-16 / 4-22 "
                  "by arrangement"),
    ),
    "plug_in_charger_blocks": (
        Axis("Object geometry", "visual+behavioral",
             tuple(Value(f"Charging block {i}", ("--block", str(i)), website=f"Charging block {i}")
                   for i in (1, 2, 3, 4))),
        Axis("Insertion force", "behavioral",
             _springs(["0.4mm indent spring", "0.7mm indent spring", "0.85mm indent spring", "1mm indent spring"],
                      "--spring", ["0.4", "0.7", "0.85", "1"], more_force="0.85", default="0.4"),
             note="the outlet's printed spring; 0.4 and 0.85 mm are fitted to measurements, 0.7 and 1 mm "
                  "interpolated and extrapolated"),
    ),
    "plug_in_usb": (
        Axis("Object geometry", "visual+behavioral",
             tuple(Value(f"USB {i}", ("--usb", str(i)), website=f"USB {i}" if i < 4 else "",
                         note="default" if i == 1 else "") for i in (1, 2, 3, 4))),
        Axis("Insertion force", "behavioral",
             _springs(["0.2mm indent spring", "0.4mm indent spring", "0.6mm indent spring"],
                      "--spring", ["0.2", "0.4", "0.6"], more_force="0.4", default="0.2"),
             note="the slot's printed spring jaws (stiffness estimated, not measured)"),
    ),
    "turn_dial_to_target": (
        Axis("Object type", "behavioral",
             (Value("Continuous dial that requires pressing down to turn", ("--dial", "push"), note="default"),
              Value("Standard continuous dial", ("--dial", "standard")),
              Value("Snap-to-position dial", ("--dial", "snap")))),
        Axis("Object geometry", "visual+behavioral",
             (Value("Flat knob", ("--knob", "flat"), note="default"), Value("Round knob", ("--knob", "round")))),
        Axis("Target position", "task parameter", flag="--target-deg", unit="deg",
             note="a 15° grid tick (60° positions on the snap dial); default 90 (snap: 120)"),
    ),
    "turn_key_unlock_door_flat": (
        Axis("Required rotation amount", "behavioral",
             tuple(Value(f"{d}°", ("--lock", d), website=f"{d} degree rotation" if d in ("90", "180") else "",
                         note="default" if d == "90" else "") for d in ("90", "180", "270", "360")),
             note="the lock module's pinion; 360° needs one regrasp"),
    ),
    "turn_doorknob_open_door_flat": (
        _DOOR_ROTATION, _DOOR_FORCE,
        Axis("Object geometry", "visual+behavioral",
             tuple(Value(f"Doorknob {k}, {d}mm diameter", ("--knob", k, "--diameter", d),
                         website=f"Doorknob {k} ({d}mm)", note="default" if (k, d) == ("1", "55") else "")
                   for k in ("1", "2") for d in ("50", "55", "60"))),
    ),
    "turn_door_handle_open_door_flat": (
        _DOOR_ROTATION, _DOOR_FORCE,
        Axis("Object geometry", "visual+behavioral",
             (Value("Handle 1", ("--handle", "1"), website="Door handle 1", note="default"),
              Value("Handle 2", ("--handle", "2"), website="Door handle 2"))),
    ),
    "install_light_bulb": (
        Axis("Object geometry", "visual+behavioral",
             tuple(Value(f"Lightbulb {b}, {s}mm diameter", ("--bulb", f"lightbulb-{b}", "--size", f"{s}mm"),
                         note="default" if (b, s) == ("1", "55") else "")
                   for b, sizes in (("1", ("55", "60", "65")), ("2", ("45", "50", "55")), ("3", ("70",)))
                   for s in sizes),
             note="light bulb 3 at 80 and 90 mm is wider than the FR3 hand opens"),
        _ROTATION_REVS,
    ),
    "put_bottle_cap_onto_bottle": (
        Axis("Object geometry", "visual+behavioral",
             tuple(Value(f"Bottle {b}", ("--bottle", f"bottle-{b}"), note="default" if b == "1" else "")
                   for b in ("1", "2", "3"))),
        Axis(_ROTATION_REVS.name, _ROTATION_REVS.kind, _ROTATION_REVS.values,
             note="the kit's bottle-1 '3 revolutions' files carry the 2.5-revolution lead (an export error)"),
    ),
    "tighten_screw_with_screwdriver": (
        Axis("Required rotation amount", "behavioral",
             (Value("6.5 revolutions", ("--screw", "6.5"), note="default"),
              Value("7.5 revolutions", ("--screw", "7.5"), website="Longer screw")),
             note="the kit screw's length: the 7.5-revolution screw rests on the nut one pitch (2.34 mm) higher and "
                  "needs 529 deg to seat against 172 deg"),
    ),
}
AXES["install_light_bulb_from_table"] = AXES["install_light_bulb"]

# modules that take --seed N: a reproducible random start (object poses sampled within the task's checked ranges)
SEEDED: set[str] = {
    "tighten_screw_with_screwdriver",   # the block in the paper's 0.15 x 0.15 m square, yaw within +-30 deg
    "loosen_screw_with_screwdriver",
    "plug_in_ethernet",                 # the slot in the paper's 0.15 x 0.15 m square, yaw within +-20 deg
    "retrieve_utensil_from_bin",        # the bin in the paper's 0.15 x 0.15 m square, yaw +-15 deg, and the fill
    "retrieve_top_cup_from_stack",      # the stack within +-50 mm of its nominal place (the fixed camera's view)
    "stack_cups",                       # the three cups each in the 0.15 x 0.15 m square, no overlap
    "plug_in_usb",                      # the slot in the paper's 0.15 x 0.15 m square, yaw within +-20 deg
    "turn_dial_to_target",              # the dial in the paper's 0.15 x 0.15 m square, yaw within +-15 deg
    "install_light_bulb",               # the socket in the paper's 0.15 x 0.15 m square (round: no yaw)
    "put_bottle_cap_onto_bottle",       # the bottle in the paper's 0.15 x 0.15 m square (round: no yaw)
}


def website_task(number: int | str) -> dict:
    """Paper task ``number`` as its page on the project website describes it (``assets/kit/tasks.json``, written by
    ``force-bench kit import``): name, assets, real-world videos and the variation axes with their instances."""
    import json
    from force_bench.paths import ASSETS_ROOT
    path = ASSETS_ROOT / "kit" / "tasks.json"
    if not path.exists():
        return {}
    return json.loads(path.read_text())["tasks"].get(str(number), {})


def axes_for(module: str) -> tuple[Axis, ...]:
    return AXES.get(module, ())


def find_axis(module: str, name: str) -> Axis:
    axes = axes_for(module)
    exact = [a for a in axes if a.name.lower() == name.lower()]
    loose = exact or [a for a in axes if name.lower() in a.name.lower()]
    if len(loose) != 1:
        have = [a.name for a in axes] or "none"
        raise KeyError(f"{module}: no single variation axis {name!r}; this task has {have}")
    return loose[0]


def apply(module: str, base_argv: tuple[str, ...], settings: list[str]) -> tuple[tuple[str, ...], list[str]]:
    """``base_argv`` with each ``"axis=value"`` in ``settings`` applied: the axis's options replace any the base
    sets.  Returns (argv, a short slug per setting for the output folder name)."""
    argv = list(base_argv)
    slugs = []
    for s in settings:
        if "=" not in s:
            raise KeyError(f"--set takes 'axis=value', not {s!r}")
        name, text = (x.strip() for x in s.split("=", 1))
        axis = find_axis(module, name)
        value = axis.find(text)
        drop = axis.flags()
        kept = []
        i = 0
        while i < len(argv):
            if argv[i] in drop:
                i += 2
                continue
            kept.append(argv[i])
            i += 1
        argv = kept + list(value.argv)
        slugs.append(re.sub(r"[^a-z0-9.]+", "-", f"{axis.name.split()[-1]}-{value.label}".lower()).strip("-"))
    return tuple(argv), slugs


def describe(module: str, argv: tuple[str, ...] = ()) -> list[str]:
    """Lines for ``force-bench info``: every axis with its values, the row's choice marked with *."""
    lines = []
    for a in axes_for(module):
        head = f"{a.name} ({a.kind})"
        if a.numeric:
            lines.append(f"{head}: {a.flag} <{a.unit}>, now {a.current(argv)}. {a.note}".rstrip())
            continue
        cur = a.current(argv)
        vals = ", ".join(("*" if v.label == cur else "") + v.label +
                         (f" [{v.website}]" if v.website and v.website != v.label else "") for v in a.values)
        lines.append(f"{head}: {vals}" + (f". {a.note}" if a.note else ""))
    return lines
