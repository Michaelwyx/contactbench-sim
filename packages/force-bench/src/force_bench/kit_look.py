"""The ContactBench kit look: every benchmark task rendered in the colours of the printed kit.

The printed ContactBench objects come in a few filament colours: yellow boards, bins, blocks and cards; white
door, outlet, platform and slots; black dials, holders and plugs; grey tools and USB sticks; orange keys.  The
benchmark tasks (``force_bench.appendix_tasks``) are rendered in those colours on a tan ground with a light-grey
work surface and a light backdrop.  This is purely visual: nothing here touches collision, mass or contact.

How it is applied (``RolloutRecorder.__init__``, keyed by the name of the recorder's output folder):

* before the camera is built, render-only stand-ins (collision proxies that duplicate a visible mesh) are hidden;
* at the first captured frame (after the studio palette and after any colours a task sets itself), each shape is
  recoloured by the task's rules (below) and the background gradient is switched to the light backdrop.

Rules are ``(pattern, colour)`` pairs, later rules win.  ``pattern`` matches a substring of the shape label or of
its body's label; ``=label`` matches a shape label exactly and ``@label`` a body label exactly.  The robot keeps
its own colours.  Tasks without an entry keep the studio palette.  ``TB_LOOK=studio`` switches the kit look off.
"""

from __future__ import annotations

import os

import numpy as np

PALETTE = {
    "yellow": (0.90, 0.94, 0.14),
    "dark": (0.33, 0.34, 0.33),
    "gray": (0.55, 0.56, 0.55),
    "white": (0.93, 0.93, 0.91),
    "black": (0.13, 0.13, 0.14),
    "orange": (0.92, 0.50, 0.28),
    "metal": (0.70, 0.71, 0.72),
    "pedestal": (0.60, 0.63, 0.68),
    "gray90": (0.42, 0.42, 0.42),        # STEP colour (90, 90, 90): wrench, USB sticks
    "dark48": (0.24, 0.24, 0.24),        # STEP colour (48, 48, 48): ruler, keyring
    "olive": (0.61, 0.61, 0.01),         # STEP colour (156, 156, 2): the cards already in the holder
}
GROUND = (0.70, 0.56, 0.40)              # tan floor
WORKTOP = (0.71, 0.72, 0.71)             # light-grey work surface (every scene table)
BG_TOP, BG_BOTTOM = (0.88, 0.88, 0.87), (0.94, 0.94, 0.93)

_HANG = [("arm_pedestal", "pedestal"), ("board", "yellow")]
_CHARGER = [("arm_pedestal", "pedestal"), ("shell", "white"), ("pad_", "white")]
_SCREWDRIVER_TABLE = [("block", "yellow"), ("@screw", "dark"), ("carrier", "dark"), ("@tool", "black"),
                      ("tool_tip", "metal")]
_ALLEN = [("block", "yellow"), ("=screw_visual", "dark"), ("@allen_key", "black")]
_WRENCH_HOOK = _HANG + [("block", "yellow"), ("bolt", "gray"), ("@nut", "dark"), ("@wrench", "gray90")]
_CARD_HOLDER = [("holder", "black"), ("card_rear", "olive"), ("card_mid", "olive"), ("card_front", "olive"),
                ("card_new", "yellow"), ("platform_board", "white"), ("platform_leg", "black")]
_BULB = [("socket", "black"), ("stand", "white")]

# task output name (outputs/appendix_tasks/<name>) -> dict(colors=[...], hide=[...])
LOOKS: dict[str, dict] = {
    "retrieve_card_from_deck": dict(colors=[("holder", "black"), ("card", "yellow")], hide=["_proxy"]),
    "pick_up_object_on_platform": dict(colors=[("platform", "black"), ("=table_b", "white"), ("@part", "yellow")]),
    "hang_keychain": dict(colors=_HANG + [("rod", "gray90"), ("@ring", "dark48"), ("@key_a", "orange"),
                                          ("@key_b", "orange")]),
    "hang_wrench": dict(colors=_HANG + [("@wrench", "gray90")]),
    "hang_ruler": dict(colors=_HANG + [("@ruler", "dark48")], hide=["=ruler_slab"]),
    "screw_nut_onto_screw": dict(colors=[("bolt", "gray"), ("@nut", "dark")]),
    # the four flat-door rows (and their variant rows) are built from the kit library: kit parts keep their print
    # colours and every collision stand-in is invisible, so they need no rules (kit_look's kit fallback)
    "plug_in_charger": dict(colors=_CHARGER + [("@plug", "black")]),
    "plug_in_charger_block1": dict(colors=_CHARGER + [("@plug", "white")]),
    "plug_in_charger_block2": dict(colors=_CHARGER + [("@plug", "black")]),
    "plug_in_charger_block3": dict(colors=_CHARGER + [("@plug", "black")]),
    "plug_in_charger_block4": dict(colors=_CHARGER + [("@plug", "black")]),
    "pick_up_object_on_platform_key": dict(colors=[("platform", "black"), ("=table_b", "white"),
                                                   ("@part", "orange")]),
    "tighten_screw_with_allen_key_from_table": dict(colors=_ALLEN),
    "loosen_screw_with_allen_key_from_table": dict(colors=_ALLEN),
    "tighten_screw_with_screwdriver_from_table": dict(colors=_SCREWDRIVER_TABLE),
    "loosen_screw_with_screwdriver_from_table": dict(colors=_SCREWDRIVER_TABLE),
    "tighten_nut_with_wrench_from_hook": dict(colors=_WRENCH_HOOK),
    "loosen_nut_with_wrench_from_hook": dict(colors=_WRENCH_HOOK),
    "thread_nut_onto_screw_from_table": dict(colors=[("block", "yellow"), ("bolt", "gray"), ("@nut", "dark")]),
    "place_card_into_holder": dict(colors=_CARD_HOLDER),
    "place_card_into_holder_from_platform": dict(colors=_CARD_HOLDER),
    "install_light_bulb": dict(colors=_BULB),
    "install_light_bulb_from_table": dict(colors=_BULB),
}


def enabled() -> bool:
    return os.environ.get("TB_LOOK", "kit") != "studio"


def look_for(name: str) -> dict | None:
    """The look of output ``name``; a variant folder (``<row>_<suffix>``) falls back to its row's look."""
    if not enabled():
        return None
    if name in LOOKS:
        return LOOKS[name]
    for key in sorted(LOOKS, key=len, reverse=True):
        if name.startswith(key + "_"):
            return LOOKS[key]
    return None


def _match(pattern: str, label: str, body: str) -> bool:
    if pattern.startswith("="):
        return label == pattern[1:]
    if pattern.startswith("@"):
        return body == pattern[1:]
    return pattern == "*" or pattern in label or bool(body and pattern in body)


def hide_stand_ins(model, name: str) -> None:
    """Clear the VISIBLE flag of render-only stand-ins (call before any camera is built)."""
    look = look_for(name)
    if not look or not look.get("hide"):
        return
    import newton
    flags = model.shape_flags.numpy().copy()
    sb = model.shape_body.numpy()
    bl = list(model.body_label)
    vis = int(newton.ShapeFlags.VISIBLE)
    for i, lbl in enumerate(model.shape_label):
        lbl = lbl or ""
        body = bl[int(sb[i])] if sb[i] >= 0 else ""
        if any(_match(h, lbl, "") if h.startswith("=") else (h in lbl) for h in look["hide"]):
            flags[i] &= ~vis
    model.shape_flags.assign(flags)


def _has_kit_parts(model) -> bool:
    return any((lbl or "").startswith("kit/") for lbl in model.shape_label)


def apply_colors(model, name: str) -> bool:
    """Recolour the scene for task ``name``; returns False when the task has no kit look.

    Shapes built from the kit library (``force_bench.kit``, labelled ``kit/...``) already carry their print colour
    and are left alone; a scene made of them gets the kit ground, work surface and backdrop even without rules."""
    look = look_for(name)
    if not look and enabled() and _has_kit_parts(model):
        look = {"colors": []}
    if not look:
        return False
    cols = model.shape_color.numpy().copy()
    sb = model.shape_body.numpy()
    bl = list(model.body_label)
    rules = [(p, PALETTE.get(c, c)) for p, c in look.get("colors", [])]
    for i, lbl in enumerate(model.shape_label):
        lbl = lbl or ""
        body = bl[int(sb[i])] if sb[i] >= 0 else ""
        if "fr3" in lbl or "fr3" in body or "gel" in lbl or "gs_case" in lbl or lbl.startswith("kit/"):
            continue                                     # the robot, the sensors and kit parts keep their colours
        if "ground" in lbl:
            cols[i] = GROUND
            continue
        hit = None
        for pat, c in rules:
            if _match(pat, lbl, body):
                hit = c
        if hit is None and "table" in lbl:
            hit = WORKTOP
        if hit is not None:
            cols[i] = hit
    model.shape_color.assign(np.asarray(cols, dtype=np.float32))
    return True


def background(name: str):
    """(top, bottom) gradient for the recorder: light for kit-look scenes."""
    return BG_TOP, BG_BOTTOM
