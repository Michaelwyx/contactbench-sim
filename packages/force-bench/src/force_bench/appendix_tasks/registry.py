"""The ContactBench 1.0 tasks available in simulation, one row per runnable task (or variant).

Task titles, numbers and skills follow the paper's Appendix C (Table C1, base tasks).  ``name`` is the CLI name
and the output folder (``outputs/appendix_tasks/<name>/``); ``module`` lives under
``force_bench.appendix_tasks`` and receives ``argv``.  ``source`` is the scene module in ``force_bench.tasks`` whose
builder the task imports (unchanged by it), or "new".  ``start`` states how the episode begins, ``notes`` lists
every documented difference from the real-world task definition, ``gelsight`` marks rows validated with GelSight
pads (``TB_GELSIGHT=1``), and ``scripted`` is False for a task that is a scene only: it has no scripted policy and
no verdict, and is driven by teleoperation or a policy of your own.
"""

from __future__ import annotations

from dataclasses import dataclass

ARTICULATED = "articulated object manipulation"
INSERTION = "insertion and alignment"
EXTRINSIC = "extrinsic contact control"
PLACING = "placing under environmental constraints"
THIN = "grasping thin or small objects"
CLUTTER = "grasping from clutter"


@dataclass(frozen=True)
class AppendixSpec:
    name: str                       # CLI name and outputs/appendix_tasks/<name>
    module: str                     # under force_bench.appendix_tasks
    appendix_task: str              # ContactBench task title (paper Appendix C)
    number: int                     # task number in Table C1
    source: str                     # scene module the task builds on (force_bench.tasks.<source>) or "new"
    argv: tuple[str, ...] = ()
    variant: str = ""               # object variant, e.g. "USB 2", "key"
    skills: tuple[str, ...] = ()
    start: str = ""                 # how the episode starts
    notes: str = ""                 # documented differences from the real-world definition
    gelsight: bool = False          # validated with TB_GELSIGHT=1
    scripted: bool = True           # has a scripted policy and a verdict (False: a scene only)

    @property
    def dotted(self) -> str:
        return f"force_bench.appendix_tasks.{self.module}"


_DIAL_NOTES = ("the printed springs are sprung contact bodies with stiffnesses estimated from the CAD (finite-element "
               "and beam models, not measured); the knob rides a joint that carries the CAD's sub-0.3 mm running fits")

# Plug in USB: the slot's spring jaws are joint drives at an ESTIMATED flexure stiffness (force_bench.kit.mechanisms.
# usb_slot); the "more force" rows use the stiffer-indent springs where the scripted policy passes
_USB_NOTES = ("the slot's printed spring jaws are prismatic joint drives at an estimated 66 N/mm per jaw (finite-element "
              "model of the printed flexure; no force measurement exists); the stick carries a 15 g conditioning mass "
              "(print estimate 1.5-5.6 g) and the jaw contacts are stiffened, so that MuJoCo's contact is stiffer "
              "than the jaws")
_USB_MORE_FORCE = tuple((i, s) for s in ("0.4", "0.6") for i in ("1", "2", "3", "4"))
_USB_PINCH = {"0.4": 18, "0.6": 24}         # plug_in_usb.PINCH_N

def _kit_door_variants(module: str, title: str, number: int, start: str, notes: str,
                       rows: list[tuple[str, tuple[str, ...], str]]) -> list[AppendixSpec]:
    """Variant rows of a kit-door task: (name suffix, module options, variant label).  The options pick the
    website's module variant (force_bench.kit.mechanisms.doorknob_module / lock_module); ``--name`` sends the
    outputs to the row's folder."""
    return [AppendixSpec(f"{module}_{suffix}", module, title, number, "new", argv=(*argv, "--name", f"{module}_{suffix}"),
                         variant=label, skills=(ARTICULATED,), start=start, notes=notes)
            for suffix, argv, label in rows]

# tasks 15 / 16 (screwdriver)
_SCREWDRIVER_NOTES = ("the thread is a helix spring between the screw's axial and rotary dofs with load-dependent friction, "
                      "a hard seat and a seat-preload stiction element (the tool-recess drive is real contact); the "
                      "screw carries 20 g (printed: 1.6 g) for the explicit stiction; the hand keeps the screwdriver at "
                      "the end, as the person does")
_SCREWDRIVER_TABLE_NOTES = ("an earlier export of the kit block (bottom half 11 mm, final kit 10.5 mm), the same thread model "
                            "as the in-hand rows; the tool is driven from a side grasp and laid back on the table")
_THREAD_NOTES = ("kit parts with their real threads (SDF contact); 'tightened' is measured from contact: seat face "
                 "in contact, part stalled, pad torque >= 150 mN.m, axial preload still there after release (the kit "
                 "has no spring tab); probe 10 mN.m each way")
_BULB_NOTES = _THREAD_NOTES + "; pinched on the globe (the palm cannot clear the globe for a grasp on the neck)"
_CAP_NOTES = _THREAD_NOTES + "; pinched on the cap's side"

_UTENSIL_BIN_NOTES = ("a scene without a scripted policy (teleoperation or your own policy); the bin is filled by "
                      "physics, the utensils going in one at a time in a random order and at random places, so no "
                      "copy of the target is placed for a grasp; the vertical-bin arrangements (14 chopsticks; 20 "
                      "mixed utensils, the video counts) are --set settings")

APPENDIX: list[AppendixSpec] = [
    AppendixSpec("retrieve_utensil_from_bin", "retrieve_utensil_from_bin", "Retrieve single utensil from bin", 2,
                 "new", variant="chopsticks in horizontal bin", skills=(CLUTTER, THIN),
                 start="12 kit chopsticks put into the horizontal bin at random by physics; the arm waits above it",
                 notes=_UTENSIL_BIN_NOTES, scripted=False),
    AppendixSpec("retrieve_card_from_deck", "retrieve_card_from_deck", "Retrieve single card from deck", 5,
                 "card_pick", skills=(CLUTTER, THIN), start="four cards leaning in the card holder", gelsight=True),
    AppendixSpec("pick_up_object_on_platform", "pick_up_object_on_platform", "Pick up object from platform", 6,
                 "slide_off_edge", argv=("card",), variant="card", skills=(THIN,),
                 start="card lying flat on the raised platform",
                 notes="the card starts 40 mm in from the platform edge (the platform centre is beyond the FR3's "
                       "reach in this cell); it is slid to the edge, then pinched"),
    AppendixSpec("pick_up_object_on_platform_key", "pick_up_object_on_platform", "Pick up object from platform", 6,
                 "slide_off_edge", argv=("key",), variant="key", skills=(THIN,),
                 start="key lying flat on the raised platform", notes="as the card variant"),
    AppendixSpec("retrieve_top_cup_from_stack", "retrieve_top_cup_from_stack", "Retrieve top cup from stack", 4,
                 "new", skills=(CLUTTER,), start="three kit cups nested on the bench",
                 notes="inside grasp: the fingertips go into the top cup's mouth and open onto its inner wall"),
    AppendixSpec("hang_keychain", "hang_keychain", "Hang object", 9, "keychain_hang", variant="keychain",
                 skills=(PLACING,), start="keychain on the bench, picked up and hung on an empty hook"),
    AppendixSpec("hang_wrench", "hang_wrench", "Hang object", 9, "new", variant="wrench", skills=(PLACING,),
                 start="wrench on the table, hung by its hole on the hanging board"),
    AppendixSpec("hang_ruler", "hang_ruler", "Hang object", 9, "new", variant="ruler", skills=(PLACING,),
                 start="ruler on the table, hung by its hole on the hanging board"),
    AppendixSpec("tighten_screw_with_allen_key_from_table", "tighten_screw_with_allen_key_from_table",
                 "Tighten screw with Allen key", 13, "new", skills=(EXTRINSIC, INSERTION),
                 start="Allen key on the table; picked up, used, put back"),
    AppendixSpec("loosen_screw_with_allen_key_from_table", "loosen_screw_with_allen_key_from_table",
                 "Loosen screw with Allen key", 14, "new", skills=(EXTRINSIC, INSERTION),
                 start="Allen key on the table; picked up, used, put back"),
    # tasks 15 / 16 on the kit block (appendix_tasks/_screwdriver_kit.py); the from-table rows are a start-state variant
    AppendixSpec("tighten_screw_with_screwdriver", "tighten_screw_with_screwdriver",
                 "Tighten screw with screwdriver", 15, "new", skills=(EXTRINSIC, INSERTION),
                 start="kit screwdriver held tip-down in a top-down pinch over the kit toy assembly block; the kit screw "
                       "(6.5 revolutions) in its right-hand hole with the threads unengaged, 172 deg above the seat",
                 notes=_SCREWDRIVER_NOTES),
    AppendixSpec("tighten_screw_with_screwdriver_longer_screw", "tighten_screw_with_screwdriver",
                 "Tighten screw with screwdriver", 15, "new", argv=("--screw", "7.5"),
                 variant="longer screw (7.5 revolutions)", skills=(EXTRINSIC, INSERTION),
                 start="as the base row with the 7.5-revolution kit screw, 529 deg above the seat",
                 notes=_SCREWDRIVER_NOTES),
    AppendixSpec("tighten_screw_with_screwdriver_from_table", "tighten_screw_with_screwdriver_from_table",
                 "Tighten screw with screwdriver", 15, "new", variant="screwdriver from the table",
                 skills=(EXTRINSIC, INSERTION), start="screwdriver on the table; picked up, used, put back",
                 notes=_SCREWDRIVER_TABLE_NOTES + "; the screw starts 4.0 mm (615 deg) above the seat"),
    AppendixSpec("loosen_screw_with_screwdriver", "loosen_screw_with_screwdriver",
                 "Loosen screw with screwdriver", 16, "new", skills=(EXTRINSIC, INSERTION),
                 start="kit screwdriver held tip-down in a top-down pinch over the kit toy assembly block; the kit screw "
                       "seated in its right-hand hole with a 20 mN.m breakaway preload",
                 notes=_SCREWDRIVER_NOTES),
    AppendixSpec("loosen_screw_with_screwdriver_from_table", "loosen_screw_with_screwdriver_from_table",
                 "Loosen screw with screwdriver", 16, "new", variant="screwdriver from the table",
                 skills=(EXTRINSIC, INSERTION), start="screwdriver on the table; picked up, used, put back",
                 notes=_SCREWDRIVER_TABLE_NOTES),
    AppendixSpec("tighten_nut_with_wrench_from_hook", "tighten_nut_with_wrench_from_hook", "Tighten nut with wrench",
                 17, "wrench_nut_arm", skills=(EXTRINSIC, INSERTION),
                 start="wrench on the hanging board; taken off the hook, used, hung back"),
    AppendixSpec("loosen_nut_with_wrench_from_hook", "loosen_nut_with_wrench_from_hook", "Loosen nut with wrench",
                 18, "wrench_nut_arm", skills=(EXTRINSIC, INSERTION),
                 start="wrench on the hanging board; taken off the hook, used, hung back"),
    AppendixSpec("put_bottle_cap_onto_bottle", "put_bottle_cap_onto_bottle", "Put bottle cap onto bottle", 19, "new",
                 variant="bottle 1 (2 revolutions)", skills=(INSERTION, ARTICULATED),
                 start="kit cap held in the pinch 4 mm above its open bottle", notes=_CAP_NOTES),
    AppendixSpec("put_bottle_cap_onto_bottle_2", "put_bottle_cap_onto_bottle", "Put bottle cap onto bottle", 19, "new",
                 argv=("--bottle", "bottle-2"), variant="bottle 2 (2 revolutions)", skills=(INSERTION, ARTICULATED),
                 start="kit cap held in the pinch 4 mm above its open bottle", notes=_CAP_NOTES),
    AppendixSpec("put_bottle_cap_onto_bottle_3", "put_bottle_cap_onto_bottle", "Put bottle cap onto bottle", 19, "new",
                 argv=("--bottle", "bottle-3"), variant="bottle 3 (2 revolutions)", skills=(INSERTION, ARTICULATED),
                 start="kit cap held in the pinch 4 mm above its open bottle", notes=_CAP_NOTES),
    AppendixSpec("put_bottle_cap_onto_bottle_4rev", "put_bottle_cap_onto_bottle", "Put bottle cap onto bottle", 19,
                 "new", argv=("--rev", "4"), variant="bottle 1, 4 revolutions", skills=(INSERTION, ARTICULATED),
                 start="kit cap held in the pinch 4 mm above its open bottle", notes=_CAP_NOTES),
    AppendixSpec("thread_nut_onto_screw_from_table", "thread_nut_onto_screw_from_table", "Thread nut onto screw", 20,
                 "nut_bolt_arm", skills=(INSERTION, ARTICULATED), start="nut on the table, picked up and threaded"),
    AppendixSpec("screw_nut_onto_screw", "screw_nut_onto_screw", "Thread nut onto screw", 20, "nut_bolt_arm",
                 variant="nut in hand", skills=(INSERTION, ARTICULATED),
                 notes="starts in the pinch 0.6 turn onto the thread, not fully off the bolt"),
    AppendixSpec("plug_in_charger", "plug_in_charger", "Plug in charger", 22, "plug_insert_arm",
                 skills=(INSERTION,), start="charger held above the outlet"),
    *[AppendixSpec(f"plug_in_charger_block{i}", "plug_in_charger_blocks", "Plug in charger", 22, "plug_insert_arm",
                   argv=("--block", str(i)), variant=f"charging block {i}", skills=(INSERTION,),
                   start="charging block held above the outlet") for i in (1, 2, 3, 4)],
    AppendixSpec("plug_in_charger_block1_spring_0.85mm", "plug_in_charger_blocks", "Plug in charger", 22,
                 "plug_insert_arm", argv=("--block", "1", "--spring", "0.85"),
                 variant="charging block 1, 0.85 mm outlet spring", skills=(INSERTION,),
                 start="charging block held above the outlet",
                 notes="the stiffer printed spring (insertion-force variation): the pinch stalls part-way, so the "
                       "policy re-grasps and presses the block home from its top"),
    AppendixSpec("place_card_into_holder", "place_card_into_holder", "Place card into holder", 23, "new",
                 skills=(PLACING,), start="card held above a holder that already contains cards"),
    AppendixSpec("place_card_into_holder_from_platform", "place_card_into_holder_from_platform",
                 "Place card into holder", 23, "new", variant="card from platform", skills=(PLACING, THIN),
                 start="card on the raised platform; picked up and placed into the holder"),
    AppendixSpec("stack_cups", "stack_cups", "Stack cups", 24, "new", skills=(PLACING,),
                 start="three kit cups upright on the bench",
                 notes="inside grasp (fingertips in the cup's mouth); each cup is lowered onto the stack's axis and "
                       "released within 0.8 mm of its nested height, where it settles under its own weight"),
    AppendixSpec("plug_in_ethernet", "plug_in_ethernet", "Plug in Ethernet", 25, "new", skills=(INSERTION,),
                 start="kit Ethernet plug held 10 mm above port 5 of the kit's 5-port slot (shell on its side, ports "
                       "up, an insert in every port)",
                 notes=("the printed latch tab is a hinged body on a spring drive at an estimated 0.72 N.m/rad "
                        "(finite-element model of the printed tab; no force measurement exists) and catches behind "
                        "the inserts' lips by contact; the plug carries a 15 g conditioning mass (print estimate "
                        "1.8 g); the light pull is 10 N through the pads")),
    *[AppendixSpec(f"plug_in_usb_{i}", "plug_in_usb", "Plug in USB", 26, "new", argv=("--usb", str(i)),
                   variant=f"USB {i}", skills=(INSERTION,),
                   start="USB stick held above the slot (0.2 mm indent spring)", notes=_USB_NOTES)
      for i in (1, 2, 3, 4)],
    *[AppendixSpec(f"plug_in_usb_{i}_spring{s.replace('.', '')}", "plug_in_usb", "Plug in USB", 26, "new",
                   argv=("--usb", str(i), "--spring", s), variant=f"USB {i}, {s} mm indent spring (more force)",
                   skills=(INSERTION,), start=f"USB stick held above the slot ({s} mm indent spring)",
                   notes=_USB_NOTES + f"; pinch {_USB_PINCH[s]} N per pad (12 N on the 0.2 mm spring)")
      for i, s in _USB_MORE_FORCE],
    AppendixSpec("turn_key_unlock_door_flat", "turn_key_unlock_door_flat", "Turn key to unlock door", 27, "new",
                 variant="lock 90°", skills=(ARTICULATED,),
                 start="final kit door lying flat, closed and locked, key 1 fully inserted",
                 notes="the doorknob module's latch is held back (scenery), so the deadbolt alone gates the leaf"),
    *_kit_door_variants("turn_key_unlock_door_flat", "Turn key to unlock door", 27,
                        "final kit door lying flat, closed and locked, key 1 fully inserted",
                        "the doorknob module's latch is held back (scenery), so the deadbolt alone gates the leaf",
                        [("lock180", ("--lock", "180"), "lock 180°"),
                         ("lock270", ("--lock", "270"), "lock 270°"),
                         ("lock360", ("--lock", "360"), "lock 360° (one regrasp)")]),
    AppendixSpec("turn_doorknob_open_door_flat", "turn_doorknob_open_door_flat", "Turn doorknob to open door", 28,
                 "new", variant="doorknob 1 55 mm, 60°, 5 mm spring", skills=(ARTICULATED,),
                 start="final kit door lying flat, closed and latched",
                 notes="the episode ends holding the door open (a released flat door falls shut); the lock module's "
                       "deadbolt is parked withdrawn"),
    *_kit_door_variants("turn_doorknob_open_door_flat", "Turn doorknob to open door", 28,
                        "final kit door lying flat, closed and latched",
                        "the episode ends holding the door open (a released flat door falls shut); the lock module's "
                        "deadbolt is parked withdrawn; doorknob module otherwise as the base row (doorknob 1 55 mm, "
                        "60°, 5 mm spring)",
                        [("unlock40", ("--unlock", "40"), "unlock 40°"),
                         ("unlock80", ("--unlock", "80"), "unlock 80°"),
                         ("spring7_5mm", ("--spring", "7.5"), "7.5 mm spring"),
                         ("spring10mm", ("--spring", "10"), "10 mm spring"),
                         ("knob2", ("--knob", "2"), "doorknob 2 55 mm"),
                         ("knob1_50mm", ("--diameter", "50"), "doorknob 1 50 mm"),
                         ("knob1_60mm", ("--diameter", "60"), "doorknob 1 60 mm")]),
    AppendixSpec("turn_door_handle_open_door_flat", "turn_door_handle_open_door_flat",
                 "Turn door handle to open door", 29, "new", variant="handle 1, 60°, 5 mm spring",
                 skills=(ARTICULATED,), start="final kit door lying flat, closed and latched",
                 notes="the episode ends holding the door open (a released flat door falls shut); the lock module's "
                       "deadbolt is parked withdrawn"),
    *_kit_door_variants("turn_door_handle_open_door_flat", "Turn door handle to open door", 29,
                        "final kit door lying flat, closed and latched",
                        "the episode ends holding the door open (a released flat door falls shut); the lock module's "
                        "deadbolt is parked withdrawn; doorknob module otherwise as the base row (handle 1, 60°, "
                        "5 mm spring)",
                        [("handle2", ("--handle", "2"), "handle 2"),
                         ("unlock40", ("--unlock", "40"), "unlock 40°"),
                         ("unlock80", ("--unlock", "80"), "unlock 80°"),
                         ("spring7_5mm", ("--spring", "7.5"), "7.5 mm spring"),
                         ("spring10mm", ("--spring", "10"), "10 mm spring")]),
    # task 30 on the finalized printable dials (force_bench.kit, mechanism model kit/mechanisms/dial.py)
    AppendixSpec("turn_dial_to_target", "turn_dial_to_target", "Turn dial to target position", 30, "new",
                 variant="push-down dial, flat knob", skills=(ARTICULATED,),
                 start="push-down-to-turn dial at 0 deg (locked up), open hand around the knob's wing bar",
                 notes=_DIAL_NOTES + "; the unpressed try uses a light 4 N pinch so the gate, not the arm, is tested"),
    AppendixSpec("turn_dial_to_target_round", "turn_dial_to_target", "Turn dial to target position", 30, "new",
                 argv=("--knob", "round"), variant="push-down dial, round knob", skills=(ARTICULATED,),
                 start="push-down-to-turn dial at 0 deg (locked up), open hand around the round knob",
                 notes=_DIAL_NOTES + "; the unpressed try uses a light 4 N pinch so the gate, not the arm, is tested"),
    AppendixSpec("turn_dial_to_target_standard", "turn_dial_to_target", "Turn dial to target position", 30, "new",
                 argv=("--dial", "standard"), variant="continuous dial, flat knob", skills=(ARTICULATED,),
                 start="standard continuous dial at 0 deg, open hand around the knob's wing bar",
                 notes=_DIAL_NOTES),
    AppendixSpec("turn_dial_to_target_standard_round", "turn_dial_to_target", "Turn dial to target position", 30,
                 "new", argv=("--dial", "standard", "--knob", "round"), variant="continuous dial, round knob",
                 skills=(ARTICULATED,), start="standard continuous dial at 0 deg, open hand around the round knob",
                 notes=_DIAL_NOTES),
    AppendixSpec("turn_dial_to_target_snap", "turn_dial_to_target", "Turn dial to target position", 30, "new",
                 argv=("--dial", "snap"), variant="snap-to-position dial, flat knob", skills=(ARTICULATED,),
                 start="snap-to-position dial at position 0, open hand around the knob's wing bar",
                 notes=_DIAL_NOTES + "; the printed detents do not self-centre (PLA friction), so the policy trims "
                                     "onto the target detent"),
    AppendixSpec("turn_dial_to_target_snap_round", "turn_dial_to_target", "Turn dial to target position", 30, "new",
                 argv=("--dial", "snap", "--knob", "round"), variant="snap-to-position dial, round knob",
                 skills=(ARTICULATED,), start="snap-to-position dial at position 0, open hand around the round knob",
                 notes=_DIAL_NOTES + "; the printed detents do not self-centre (PLA friction), so the policy trims "
                                     "onto the target detent"),
    AppendixSpec("install_light_bulb_from_table", "install_light_bulb_from_table", "Install light bulb", 31, "new",
                 variant="lightbulb 1 (55 mm, 2 revolutions) from the table", skills=(INSERTION, ARTICULATED),
                 start="kit bulb upright on a printed stand on the table; picked up, screwed into the kit's "
                       "lightbulb slot",
                 notes=_BULB_NOTES + "; the stand is a generated printable part (not in the kit); the paper starts "
                       "with the bulb in the hand"),
    AppendixSpec("install_light_bulb", "install_light_bulb", "Install light bulb", 31, "new",
                 variant="lightbulb 1 (55 mm, 2 revolutions), bulb in hand", skills=(INSERTION, ARTICULATED),
                 start="kit bulb held in the pinch 4 mm above the kit's lightbulb slot", notes=_BULB_NOTES),
    AppendixSpec("install_light_bulb_4rev", "install_light_bulb", "Install light bulb", 31, "new",
                 argv=("--rev", "4"), variant="lightbulb 1 (55 mm), 4 revolutions", skills=(INSERTION, ARTICULATED),
                 start="kit bulb held in the pinch above the 4-revolution slot", notes=_BULB_NOTES),
    AppendixSpec("install_light_bulb_2_50mm", "install_light_bulb", "Install light bulb", 31, "new",
                 argv=("--bulb", "lightbulb-2", "--size", "50mm"), variant="lightbulb 2 (50 mm, 2 revolutions)",
                 skills=(INSERTION, ARTICULATED), start="kit bulb held in the pinch above the lightbulb slot",
                 notes=_BULB_NOTES + "; the FR3 palm keeps the pads above this bulb's equator: they bear on the "
                       "upper globe with their lower edges"),
    AppendixSpec("install_light_bulb_3_70mm", "install_light_bulb", "Install light bulb", 31, "new",
                 argv=("--bulb", "lightbulb-3", "--size", "70mm"), variant="lightbulb 3 (70 mm, 2 revolutions)",
                 skills=(INSERTION, ARTICULATED), start="kit bulb held in the pinch above the lightbulb slot",
                 notes=_BULB_NOTES + "; pinched at the reflector rim; the 80 and 90 mm reflectors do not fit the FR3 "
                       "hand (0.3 mm of opening left per side at full opening, and 9 mm too wide)"),
]

BY_NAME = {s.name: s for s in APPENDIX}

# Table C1 success criteria of the paper, verbatim (shown by `force-bench info`)
CRITERIA: dict[int, str] = {
    4: ("The robot movement stops after separating the top cup from the stack and placing it on the "
        "table. The remaining cups should remain in a neat stack. The robot should not remain in contact "
        "with the task objects."),
    24: ("The robot movement stops after all cups form a tight, upright stack (not knocked over). The "
         "robot should not remain in contact with the task objects."),
    2: ("The robot movement stops after separating a single copy of the target utensil and placing it on "
        "the table. Other utensils should remain in the storage bin. The robot should not remain in "
        "contact with the task objects."),
    5: ("The robot movement stops after separating a single card from the deck and placing it on the "
        "table. The remaining cards should remain a neat deck in the cardholder. The robot should not "
        "remain in contact with the task objects."),
    6: ("The target object is stably grasped by the robot and lifted off the platform."),
    9: ("The robot movement stops after placing the target object onto a hanging hook. The robot should "
        "not remain in contact with the task objects."),
    13: ("The screw is tightened and remains stationary when a small force is applied in an attempt to "
        "rotate the screw. The robot movement stops after terminating the tightening motion, moving away "
        "from, and breaking contact with the screw."),
    14: ("The screw is loosened and can rotate freely when a small force is applied in an attempt to "
        "rotate the screw. The robot movement stops after terminating the loosening motion, moving away "
        "from, and breaking contact with the screw."),
    15: ("The screw is tightened and remains stationary when a small force is applied in an attempt to "
        "rotate the screw. The robot movement stops after terminating the tightening motion, moving away "
        "from, and breaking contact with the screw."),
    16: ("The screw is loosened and can rotate freely when a small force is applied in an attempt to "
        "rotate the screw. The robot movement stops after terminating the loosening motion, moving away "
        "from, and breaking contact with the screw."),
    17: ("The nut is tightened and remains stationary when a small force is applied in an attempt to "
        "rotate the nut. The robot movement stops after terminating the tightening motion, moving away "
        "from, and breaking contact with the nut."),
    18: ("The nut is loosened and can rotate freely when a small force is applied in an attempt to rotate "
        "the nut. The robot movement stops after terminating the loosening motion, moving away from, and "
        "breaking contact with the nut."),
    19: ("The bottle cap is tightened and remains stationary when a small force is applied in an attempt to "
         "rotate the cap. The robot movement stops after terminating the tightening motion, moving away from, and "
         "breaking contact with the task objects."),
    20: ("The nut and screw threads engage. The robot movement stops after terminating the rotation "
        "motion, moving away from, and breaking contact with the task objects."),
    22: ("The charger is fully inserted into the outlet, such that it does not noticeably move when "
        "further pushed toward the outlet. The robot movement stops after terminating the insertion "
        "motion, moving away from, and breaking contact with the task objects."),
    23: ("The robot movement stops after placing the card in the card holder, with existing cards "
        "remaining in the card holder. The robot should not remain in contact with the task objects."),
    25: ("The Ethernet plug is fully inserted into the socket with its latch engaged, such that the plug "
        "can't be withdrawn by a light pull. The robot movement stops after terminating the insertion "
        "motion, moving away from, and breaking contact with the task objects."),
    26: ("The USB stick is fully inserted into the slot, such that it does not noticeably move when "
        "further pushed toward the slot. The robot movement stops after terminating the insertion motion, "
        "moving away from, and breaking contact with the task objects."),
    27: ("The key is fully turned such that the door can be opened. The robot movement stops after "
        "terminating the rotation motion, moving away from, and breaking contact with the task objects."),
    28: ("The door is opened by at least 5 degrees, such that it is visually obvious that the door has "
        "been unlocked."),
    29: ("The door is opened by at least 5 degrees, such that it is visually obvious that the door has "
        "been unlocked."),
    30: ("The dial is turned to the target position. For dials that can be turned continuously, the final "
        "position should be within half a tick interval from the target position. The robot movement "
        "stops after terminating the rotation motion, moving away from, and breaking contact with the "
        "task objects."),
    31: ("The light bulb is tightened and remains stationary when a small force is applied in an attempt "
        "to rotate it. The robot movement stops after terminating the tightening motion, moving away "
        "from, and breaking contact with the task objects."),
}
