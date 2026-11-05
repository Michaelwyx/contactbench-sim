# Tasks

ContactBench 1.0 defines 31 base tasks and 5 longer-horizon tasks (paper Appendix C, Tables C1 and C2). This release
simulates 22 of the base tasks with a scripted policy and an automatic check of the paper's success criterion, in
73 configurations, and provides task 2 as a scene without a scripted policy. The tasks that are not included, and
why, are listed at the end ([Tasks not included](#tasks-not-included)).

Each configuration is a row of `force-bench list` and is run by its name. A task can have several rows: the object
variants of the kit (USB sticks, charging blocks, dials, door modules, bulbs, bottles), its springs and other
variation values, or a second start state (the tool already in the hand, as the paper specifies, or lying on the
table, to be picked up first and put back at the end).

```bash
force-bench list                 # every row, by paper task number
force-bench info <name>          # what the row does, how it starts, how it differs from the real task
force-bench run <name>           # run it -> outputs/appendix_tasks/<name>/
force-bench run-all -j 3         # every row with a scripted policy -> outputs/appendix_tasks/summary.json
force-bench variations           # the variation axes of each task
```

## How a run is judged

A run sets up the task's start state, runs the scripted policy with the Franka FR3 arm and Franka Hand, and writes
its verdict to `appendix_qa.json`:

- `appendix_success_criterion`: the paper's success criterion (Table C1), verbatim;
- `criteria`: the criterion split into individual checks, each with `pass` and the values it was decided on;
- `physics_integrity`: checks on the simulation itself (finite values, no velocity spikes, no penetration beyond
  tolerance, nothing leaving the workspace), so that a success produced by a simulation artefact does not count;
- `changes_vs_suite_task`: the documented differences between this simulated task and the real one;
- `pass`: true only if every check passes.

Many of the paper's criteria end with "the robot movement stops after ..., moving away from, and breaking contact
with the task objects". This is checked literally: `released_and_clear` measures the contact force between the robot
and the task objects at the end, and `robot_stopped` the arm's motion. For the fastening tasks, the paper's holding
test ("remains stationary when a small force is applied") is a small torque applied to the screw, nut, cap or bulb in
both directions after the robot has left.

The scripted policies react to the simulated state (object poses, joint angles, contact forces), for example tightening
until the torque stalls. They show that a task can be solved in simulation and provide reference demonstrations; they
are not a baseline to beat.

## Task list

Skills, as named in the paper: grasping thin or small objects (thin), grasping from clutter (clutter), placing under
environmental constraints (placing), extrinsic contact control (extrinsic), articulated object manipulation
(articulated), insertion and alignment (insertion).

| # | task | skills | rows |
|---|---|---|---|
| 2 | Retrieve single utensil from bin | clutter, thin | `retrieve_utensil_from_bin` (scene only, no scripted policy) |
| 4 | Retrieve top cup from stack | clutter | `retrieve_top_cup_from_stack` |
| 5 | Retrieve single card from deck | clutter, thin | `retrieve_card_from_deck` |
| 6 | Pick up object from platform | thin | `pick_up_object_on_platform` (card), `pick_up_object_on_platform_key` |
| 9 | Hang object | placing | `hang_keychain`, `hang_wrench`, `hang_ruler` |
| 13, 14 | Tighten / loosen screw with Allen key | extrinsic, insertion | `tighten_screw_with_allen_key_from_table`, `loosen_screw_with_allen_key_from_table` |
| 15, 16 | Tighten / loosen screw with screwdriver | extrinsic, insertion | `tighten_screw_with_screwdriver`, `tighten_screw_with_screwdriver_longer_screw`, `tighten_screw_with_screwdriver_from_table`, `loosen_screw_with_screwdriver`, `loosen_screw_with_screwdriver_from_table` |
| 17, 18 | Tighten / loosen nut with wrench | extrinsic, insertion | `tighten_nut_with_wrench_from_hook`, `loosen_nut_with_wrench_from_hook` |
| 19 | Put bottle cap onto bottle | insertion, articulated | `put_bottle_cap_onto_bottle`, `_2`, `_3`, `_4rev` |
| 20 | Thread nut onto screw | insertion, articulated | `screw_nut_onto_screw`, `thread_nut_onto_screw_from_table` |
| 22 | Plug in charger | insertion | `plug_in_charger`, `plug_in_charger_block1` ... `4`, `plug_in_charger_block1_spring_0.85mm` |
| 23 | Place card into holder | placing | `place_card_into_holder`, `place_card_into_holder_from_platform` |
| 24 | Stack cups | placing | `stack_cups` |
| 25 | Plug in Ethernet | insertion | `plug_in_ethernet` |
| 26 | Plug in USB | insertion | `plug_in_usb_1` ... `4`, and `plug_in_usb_N_spring04`, `_spring06` with stiffer springs |
| 27 | Turn key to unlock door | articulated | `turn_key_unlock_door_flat`, `_lock180`, `_lock270`, `_lock360` |
| 28 | Turn doorknob to open door | articulated | `turn_doorknob_open_door_flat`, `_unlock40`, `_unlock80`, `_spring7_5mm`, `_spring10mm`, `_knob2`, `_knob1_50mm`, `_knob1_60mm` |
| 29 | Turn door handle to open door | articulated | `turn_door_handle_open_door_flat`, `_handle2`, `_unlock40`, `_unlock80`, `_spring7_5mm`, `_spring10mm` |
| 30 | Turn dial to target position | articulated | `turn_dial_to_target`, `_round`, `turn_dial_to_target_standard`, `_standard_round`, `turn_dial_to_target_snap`, `_snap_round` |
| 31 | Install light bulb | insertion, articulated | `install_light_bulb`, `install_light_bulb_from_table`, `_4rev`, `_2_50mm`, `_3_70mm` |

Each task below lists its setup, the checks of its verdict, its options and its differences from the real task.
`force-bench info <name>` prints the same for a single row.

---

### 2. Retrieve single utensil from bin

*Pick up a single copy of the target utensil from a cluttered bin and place it on the table, leaving the others in
the bin.*

**Scene only.** This task has no scripted policy and no verdict: with the FR3's fingertips, a scripted pinch only
works if the target is placed where the fingers fit, which removes the clutter the task is about. The scene is
provided for teleoperation and for policies of your own.

**Setup.** The website's three arrangements, with the counts of the real-world videos: 12 chopsticks in the horizontal
bin (the row `retrieve_utensil_from_bin`), 14 chopsticks in the vertical bin, or 20 mixed utensils (6 knives, 5 forks,
4 spoons, 5 chopsticks) in the vertical bin, where the target is a spoon. Before the episode the bin is filled by
physics, in a random order and at random places, so that no utensil is placed for a grasp. The arm starts above the
bin.

**Use.**

```bash
force-bench teleop-ui --scene retrieve_utensil_from_bin                          # drive it by hand
force-bench teleop-ui --scene retrieve_utensil_from_bin --collect-dir outputs/datasets/utensils   # and record it
force-bench run retrieve_utensil_from_bin                                        # record the filled scene
```

`force-bench run` writes the video of the filled scene and `scene.json`: the arrangement, the utensils in the bin,
the start, and the state of the criterion at the end (which utensils are outside the bin, and whether exactly one is,
a copy of the target, at rest on the table). The same state is shown live as readouts in the teleoperation UI. From
Python, `Task(arrangement, target, n_objects, seed, name).setup()` (in
`force_bench.appendix_tasks.retrieve_utensil_from_bin`) builds the filled scene for a policy of your own, and
`Task.criterion_state(state)` reports the criterion's state.

**Options.**

| axis (`--set`) | values |
|---|---|
| Scene arrangement | Chopsticks in horizontal bin (default), Chopsticks in vertical bin, Mixed utensils in vertical bin |
| Object type | the target: Chopstick (default), Fork, Knife, Spoon (default in the mixed bin) |
| Number of objects | utensils in the bin: 10 to 14, 2 to 16 or 4 to 22 by arrangement (default 12, 14, 20) |

`--seed N` places the bin anywhere in the paper's 0.15 × 0.15 m square, turned by up to ±15°, and draws the fill.
`--embodiment finray` mounts the Fin Ray fingers ([finray.md](finray.md)).

**Differences.** In the horizontal-bin video the chopsticks are packed in layers; here they fall onto a pile.

### 4. Retrieve top cup from stack

*Separate the top cup from a stack and place it on the table, leaving the other cups in a neat stack.*

**Setup.** Three kit cups nested on the bench. The robot lifts the top cup out with an inside grasp (fingertips in the
cup's mouth, opened against its inner wall), sets it upright on the table and moves away.

**Checks.** `top_cup_on_table` (upright, at least 80 mm from the stack), `stack_remains_neat` (the other two still
nested and upright), `real_inside_grasp`, `released_and_clear`, `robot_stopped`.

**Options.** `--seed N` places the stack within ±50 mm of its nominal position.

**Differences.** The random start covers 0.10 × 0.10 m rather than the paper's 0.15 × 0.15 m square, to keep the cups
in the fixed camera's view.

### 5. Retrieve single card from deck

*Separate a single card from a deck in a card holder and place it on the table, leaving the deck neat.*

**Setup.** Four cards lean in the card holder. The robot pinches the front card, slides it up out of the slot, sets it
down on the table and moves away.

**Checks.** `single_card_on_table`, `deck_remains_neat` (the other cards move less than 5 mm and 3° and stay in the
slot), `released_and_clear`.

**Options.** `TB_GELSIGHT=1 force-bench run retrieve_card_from_deck` runs the task with simulated GelSight Mini pads
and records both pads' tactile fields ([tactile.md](tactile.md#3-gelsight-on-the-tasks-tb_gelsight1)).

### 6. Pick up object from platform

*Grasp a thin object lying on a platform and lift it off.*

**Setup.** A card (`pick_up_object_on_platform`) or a key (`pick_up_object_on_platform_key`) lies flat on a raised
platform. It is too thin to pinch from above, so the robot presses on it with a fingertip, slides it until it
overhangs the platform's edge, pinches the overhang and lifts it.

**Checks.** `starts_mid_platform`, `lifted_off_platform`, `stably_grasped` (both pads loaded, less than 2 mm of slip
during the hold).

**Differences.** The object starts 40 mm in from the platform's edge rather than at its centre, which is beyond the
FR3's reach in this layout.

### 9. Hang object

*Hang the target object on a hook.*

**Setup.** The hanging board with three hooks. A keychain (key ring with two keys), the kit wrench or the ruler lies on
the table; the robot picks it up, threads it onto an empty hook, lets go and moves away. The ruler is first slid over
the table's edge to be pinched, as in task 6.

**Checks.** `keychain_on_hook` or `hung_on_hook`, `picked_up_from_bench` (or `real_fingertip_grasp` and
`keys_still_linked` for the keychain), `released_and_clear`, `robot_stopped` or `robot_still`.

**Differences.** The paper starts with the object in the hand; here it is picked up from the table first.

### 13, 14. Tighten / loosen screw with Allen key

*Tighten (or loosen) a hex-socket screw with an Allen key, so that it stays put (or turns freely) under a small
torque.*

**Setup.** The kit's toy assembly block with a hex-socket screw, and the Allen key on the table. The robot picks up the
key, inserts its end into the screw's socket, turns the screw in strokes, lifting out and re-inserting between strokes,
and puts the key back on the table. Tighten starts with the threads not yet engaged (615° to go); loosen starts with
the screw seated and preloaded.

**Checks.** `picked_up_from_table`, `turned_by_the_tool_in_hand`, `no_cam_out`, `screw_tightened` and
`stationary_under_small_torque` (tighten) or `screw_loosened` and `rotates_freely_under_small_torque` (loosen),
`key_off_screw`, `key_returned_to_table`, `released_and_clear`, `robot_stopped`.

**Differences.**
- The paper starts with the key in the hand.
- The screw's thread is a reduced model: an axial and a rotary motion coupled at the thread's lead, with friction,
  a seat and a seated holding torque. The key turns the screw through real contact in the socket.

### 15, 16. Tighten / loosen screw with screwdriver

*Tighten (or loosen) a screw with a screwdriver, so that it stays put (or turns freely) under a small torque.*

**Setup.** The scene of the task videos: the kit's toy assembly block with its captive nut, and the kit's round-head
screw (square recess) in the right-hand hole. In `tighten_screw_with_screwdriver` and `loosen_screw_with_screwdriver`
the screwdriver starts in the hand, held from above by its handle, tip down. The robot lines the tip up with the
recess, inserts it, turns the screw by turning the hand while pressing, and moves away. Tighten starts with the screw
resting in the hole, its thread not yet engaged; loosen starts with it seated and preloaded.

- `tighten_screw_with_screwdriver_longer_screw`: the website's longer screw (7.5 instead of 6.5 revolutions); it takes
  two or three strokes, the tip lifted out and turned back between them.
- `tighten_screw_with_screwdriver_from_table`, `loosen_screw_with_screwdriver_from_table`: the screwdriver lies on the
  table; it is picked up, used from a side grasp, and laid back.

**Checks.** `screw_tightened` and `stationary_under_small_torque` (tighten) or `screw_loosened` and
`rotates_freely_under_small_torque` (loosen), `turned_by_the_tool_in_hand`, `screwdriver_kept_in_hand` (or
`picked_up_from_table` and `tool_returned_to_table`), `tool_off_screw`, `released_and_clear`, `robot_stopped`.

**Options.** `--seed N` (the in-hand rows) places the block anywhere in the paper's 0.15 × 0.15 m square, turned by up
to ±30°.

**Differences.**
- A person twists the handle with the fingers and regrasps without letting go. A parallel gripper cannot: the robot
  keeps its grasp and lifts the tip out to turn back between strokes.
- The robot lines the tip up with the measured recess; a person finds it by feel.
- The thread is the reduced model of tasks 13 and 14; the screw is heavier than the print (20 g instead of 1.6 g) to
  keep that model stable.
- The from-table rows use an earlier version of the block (0.5 mm taller) and start the screw higher above its seat.

### 17, 18. Tighten / loosen nut with wrench

*Tighten (or loosen) a nut with an open-end wrench, so that it stays put (or turns freely) under a small torque.*

**Setup.** The kit wrench hangs on the hanging board; the kit's bolt block with its nut stands on the table. The robot
takes the wrench off its hook, engages the jaw on the nut's flats, turns the nut in strokes, and hangs the wrench back.
Tighten starts with the nut just above its seat; loosen starts with it seated and preloaded.

**Checks.** `wrench_taken_from_hook`, `wrench_held_and_clear_of_the_block`, `nut_tightened`,
`nut_unmoved_after_wrench_left` and `stationary_under_small_torque` (tighten) or `started_seated_and_tight`,
`nut_loosened` and `rotates_freely_under_small_torque` (loosen), `wrench_off_the_nut`, `wrench_returned_to_hook`,
`released_and_clear`, `robot_stopped`.

**Differences.** The paper starts with the wrench in the hand. The nut's seated holding torque uses the same model as
the screw tasks.

### 19. Put bottle cap onto bottle

*Screw a bottle cap onto a bottle until it is tight.*

**Setup.** The kit's bottles and caps: bottle 1 with the 2-revolution thread (`put_bottle_cap_onto_bottle`), bottles 2
and 3 (`_2`, `_3`) and bottle 1 with the 4-revolution thread (`_4rev`). The cap starts in the hand just above the
bottle. The thread is the printed geometry, and only the fingertips hold the cap. The robot screws the cap on in
strokes until it stalls on the bottle's lip, lets go and moves away.

**Checks.** `held_in_hand_at_start`, `seated_and_tightened`, `stationary_under_small_torque`, `real_finger_grasp`,
`released_and_clear`. The kit has no spring tab, so "tightened" is judged from contact: the cap seated on the lip,
pulled down by the thread, and still preloaded after release.

**Options.** `--seed N` places the bottle anywhere in the paper's 0.15 × 0.15 m square.

**Differences.** The cap is pinched on its side and turned in strokes rather than in one continuous twist. Bottle 1's
"3 revolutions" files carry the 2.5-revolution thread; this variant has no row.

### 20. Thread nut onto screw

*Thread a nut onto a screw until the threads engage.*

**Setup.** The kit's bolt block. The thread is real geometry, and the nut is carried down the bolt by thread contact.
In `screw_nut_onto_screw` the nut starts in the hand, just started on the thread; in
`thread_nut_onto_screw_from_table` it lies on the table: the robot picks it up, finds the thread's start by turning it
backwards under a light touch until it drops onto the thread, then turns it on until the whole nut is on full thread.

**Checks.** `threads_fully_engaged`, `nut_stays_put_after_release`, `real_finger_grasp`, `released_and_clear`, and
`held_in_hand_at_start` or `picked_up_from_table` and `thread_start_search`.

**Differences.** The in-hand row starts 0.6 turn onto the thread rather than fully off the bolt. The nut is heavier
than the print (10 g instead of about 1.2 g).

### 22. Plug in charger

*Fully insert a charger into an outlet, so that it does not move further when pushed.*

**Setup.** The charger starts in the hand with its prongs just above the outlet. `plug_in_charger` uses the original
charger; `plug_in_charger_block1` ... `4` use the kit's four charging blocks with the kit outlet's softest printed
spring (0.4 mm indent). `plug_in_charger_block1_spring_0.85mm` uses the stiffer 0.85 mm spring, the paper's
insertion-force variation: the grasp slips part-way in, so the robot presses the block home from its top, as a person
would with a thumb. After seating, the robot pushes again to check that the charger does not move, lets go and moves
away.

**Checks.** `fully_inserted`, `no_motion_when_pushed`, `stays_seated_after_release`, `released_and_clear`.

**Options.** `--spring 0.4 | 0.7 | 0.85 | 1` selects the outlet's printed spring.

**Differences.** The printed contact springs are modelled as sprung pads in the blade slots, fitted to the measured
insertion force of the 0.4 and 0.85 mm springs; the 0.7 and 1 mm springs are interpolated and extrapolated from
them. The blocks' masses are estimates from their printed volume.

### 23. Place card into holder

*Place a card into a card holder that already holds cards, leaving those in place.*

**Setup.** The kit card holder with three cards. In `place_card_into_holder` a fourth card starts in the hand; the
robot tilts it to the deck's lean, aligns it with the slot and slides it down in front of the other cards. In
`place_card_into_holder_from_platform` the card starts flat on a raised platform and is picked up first, as in task 6.

**Checks.** `held_in_hand_at_start` or `picked_up_from_platform`, `held_through_transport`, `released_inside_slot`,
`card_in_holder`, `existing_cards_remain`, `released_and_clear`, `robot_stopped`.

### 24. Stack cups

*Stack the cups into a tight, upright stack.*

**Setup.** Three kit cups stand upright on the bench. The robot lifts the second cup with an inside grasp (as in task
4), lowers it onto the first and lets go, so that it settles into the stack under its own weight; then the third.

**Checks.** `cups_stacked` (centred, nested, upright), `stack_not_knocked`, `real_inside_grasp`, `released_and_clear`,
`robot_stopped`.

**Options.** `--seed N` scatters the three cups in the paper's 0.15 × 0.15 m square, at least 6.4 mm apart.

### 25. Plug in Ethernet

*Fully insert an Ethernet plug into its socket with the latch engaged, so that a light pull does not withdraw it.*

**Setup.** As in the real-world video: the kit's black 5-port shell lies on its side with its ports facing up, a grey
printed insert in every port, and the white printed plug, without a cable, starts in the hand above the port farthest
from the camera. The plug's printed latch tab springs down as it passes the insert's lips and catches behind them. The
robot inserts the plug, pulls it lightly without touching the tab (the latch must hold it), lets go and moves away.

**Checks.** `held_in_hand_at_start`, `fully_inserted`, `latch_engaged`, `light_pull_held` (at least 6 N),
`stays_latched_after_release`, `finger_pinch_real`, `released_and_clear`.

**Options.** `--seed N` places the shell anywhere in the paper's 0.15 × 0.15 m square, turned by up to ±20°;
`--port 1..5` picks another port.

**Differences.**
- The latch's stiffness is an estimate from a model of the printed tab, not a measurement; the real tab may be
  softer.
- The plug is heavier than the print (15 g instead of 1.8 g) to keep its contact stable in the solver.
- The person in the video pushes the plug in by hand; the robot holds it by its rear block, so the simulated insertion
  force is somewhat higher.

### 26. Plug in USB

*Fully insert a USB stick into its slot, so that it does not move further when pushed.*

**Setup.** The kit's four USB sticks and its USB slot, standing with its mouth up. The slot's printed spring jaws
grip the inserted stick; `plug_in_usb_1` ... `4` use the softest spring (0.2 mm indent), the `_spring04` and
`_spring06` rows the stiffer 0.4 and 0.6 mm springs. The stick starts in the hand above the slot, slightly off its
axis. The robot aligns it, inserts it, pushes again to check that it does not move, lets go and moves away.

**Checks.** `held_in_hand_at_start`, `fully_inserted`, `no_motion_when_pushed`, `stays_seated_after_release`,
`finger_pinch_real`, `released_and_clear`.

**Options.** `--seed N` places the slot anywhere in the paper's 0.15 × 0.15 m square, turned by up to ±20°;
`--offset-mm X,Y` sets the stick's start offset from the slot's axis.

**Differences.** The jaws' stiffness is an estimate from a model of the printed flexure, not a measurement. The sticks
are heavier than the prints (15 g instead of 1.5 to 5.6 g) to keep their contact stable in the solver.

### 27. Turn key to unlock door

*Turn the key until the door can be opened.*

**Setup.** The final kit door lies flat, closed and locked, with key 1 fully inserted in the lock module. The lock
variants of the website need 90° (the default row), 180°, 270° or 360° of rotation. The robot pinches the key's bow,
turns it until the deadbolt clears the strike (the 360° lock needs one regrasp), lets go and moves away. A hidden
torque at the hinge checks that the door is locked at the start and can be opened at the end.

**Checks.** `start_locked_key_inserted`, `key_fully_turned`, `finger_pinch_during_turn`, `released_moved_away_still`,
`door_can_be_opened`.

**Differences.** The key starts inserted and turns with the lock cylinder (inserting it is not modelled, see task 21
under [Tasks not included](#tasks-not-included)). The doorknob module's latch is held back, so only the deadbolt keeps
the door shut. The deadbolt's friction spring is an estimate.

### 28. Turn doorknob to open door

*Turn the doorknob and open the door by at least 5°.*

**Setup.** The final kit door lies flat, closed and latched. The default row uses doorknob 1 (55 mm), 60° to unlock and
the 5 mm return spring; the other rows vary the unlock angle (40°, 80°), the spring (7.5, 10 mm) and the knob (doorknob
2, doorknob 1 at 50 or 60 mm). The robot grasps the knob from above, turns it until the latch clears the strike, and
lifts the door about its hinge through the knob.

**Checks.** `locked_at_start`, `unlocked_by_knob_then_opened`, `finger_grip_sustained`, `motion_stopped_door_held_open`.

**Differences.** The episode ends with the door held open and the arm stopped, as in the reference videos: a released
flat door falls shut. The return springs' stiffness and preload are fitted to push forces measured on printed handles
([examples/force_response](../examples/force_response/README.md)).

### 29. Turn door handle to open door

*Turn the door handle and open the door by at least 5°.*

**Setup.** The same door with a lever handle (handle 1 by default, handle 2 in `_handle2`). The robot pinches the lever
bar, pushes it down until the latch clears, and lifts the door through the lever. The other rows vary the unlock angle
and the spring as for task 28.

**Checks.** `locked_at_start`, `unlocked_by_handle_then_opened`, `finger_grip_sustained`,
`motion_stopped_door_held_open`.

**Differences.** As for task 28.

### 30. Turn dial to target position

*Turn the dial to the target position (for continuous dials, within half a tick).*

**Setup.** The kit's three dials, each with the flat (wing) knob or the round knob:

- push-down dial (`turn_dial_to_target`, `_round`): locked at 0° until it is pressed down about 3 mm; target 90°;
- standard dial (`_standard`, `_standard_round`): a friction brake holds any angle; target 90°;
- snap-to-position dial (`_snap`, `_snap_round`): a detent every 60°; target 120°.

The open hand closes on the knob. On the push-down dial the robot first tries to turn it without pressing (the lock
must hold), then presses, turns to the target, lets go and moves away.

**Checks.** `dial_at_target` (within 7.5°; the snap dial seated in the target detent), `dial_stays_after_release`,
`unpressed_try_blocked` and `held_down_off_zero` (push-down dial), `finger_grasp_real`, `released_and_clear`.

**Options.** `--set "Target position=<degrees>"` sets another target (any 15° tick; the snap dial's 60° positions);
`--seed N` places the dial anywhere in the paper's 0.15 × 0.15 m square, turned by up to ±15°.

**Differences.** The dials' printed springs and brake pads are modelled with stiffnesses estimated from the CAD and the
printed material, not measured. The snap dial's detents do not centre the knob by themselves, so the robot trims onto
the target detent. The pressed push-down knob turns with somewhat less torque than estimated (about 0.19 N·m instead
of 0.28 N·m).

### 31. Install light bulb

*Screw a light bulb into its socket until it is tight.*

**Setup.** The kit's bulbs and bulb socket, with the printed thread geometry as in task 19. `install_light_bulb` starts
with light bulb 1 (55 mm) in the hand above the socket; `install_light_bulb_from_table` starts with it on a printed
stand on the table; `_4rev` uses the 4-revolution thread, `_2_50mm` and `_3_70mm` light bulbs 2 and 3. The robot holds
the bulb by its globe, screws it in in strokes until it stalls, lets go and moves away.

**Checks.** `held_in_hand_at_start` or `picked_up_from_table`, `seated_and_tightened`, `stationary_under_small_torque`,
`real_finger_grasp`, `released_and_clear`.

**Options.** `--seed N` (the in-hand rows) places the socket anywhere in the paper's 0.15 × 0.15 m square.

**Differences.** The bulb is held by its globe: the FR3's palm cannot reach a grasp on its neck. The stand of the
from-table row is a printable part made for this task, not part of the kit. Light bulb 3 at 80 and 90 mm is wider
than the FR3 hand opens and has no row.

---

## Task options

Every task's variation axes, random starts and other options are listed by `force-bench variations` and
`force-bench info <name>`, and described in [variations.md](variations.md):

```bash
force-bench run plug_in_charger_block1 --set "Insertion force=0.85mm"   # a variation value, by its website label
force-bench run turn_dial_to_target --set "Target position=45"           # another target
force-bench run plug_in_usb_1 --seed 2                                   # a random start
force-bench run retrieve_utensil_from_bin --embodiment finray            # other fingers (embodiments.md)
force-bench run plug_in_ethernet --port 3                                # an option of the task itself
```

A run with `--set`, `--seed` or `--embodiment` writes to its own folder, `outputs/appendix_tasks/<name>__<choices>/`,
so it never overwrites the row's default run. `--name <folder>` or `TB_OUT_ROOT=<dir>` chooses another place.

## Tasks not included

Nine of the 31 base tasks have no scripted policy in this release; task 2 is included as a scene without one (its
section above), the others are not included. A task is left out rather than changed: a setup simplified until a
scripted FR3 policy can solve it would no longer measure what the task is about.

| # | task | why it is not included |
|---|---|---|
| 1 | Grasp target key from keychain | Singling out one key means separating thin keys that hang against each other on a ring. A scripted policy would need the keys spread apart, and contact among many thin, loosely coupled parts is slow to simulate. |
| 2 | Retrieve single utensil from bin | Included as a scene only. The FR3's fingertips (15.2 mm thick) need free space on both sides of the target, which a packed or randomly filled bin does not leave; a scripted policy would need the target placed for the fingers. |
| 3 | Retrieve single fastener from bin | The kit's fastener bin (about 52 × 37 mm inside, 45 mm deep) is too narrow for the FR3 hand to reach a part on its floor: the fingers flare to 25 mm each above their tips. |
| 7 | Collect utensils | Meant for hands that hold several objects at once. A parallel-jaw gripper holds one at a time, so the task would become a series of single picks. |
| 8 | Collect fasteners | As task 7, with screws and nuts. |
| 10 | Place utensil into bin | Finding an open pocket needs a bin full of utensils. With the few stored utensils a scripted placement can handle, the task is too easy; a full bin is slow to simulate and beyond a scripted placement. |
| 11 | In-hand repose | In-hand manipulation (turning a key, a card or a utensil over in the fingers) relies on controlled sliding and rolling inside the grasp, which depends on pad friction and compliance that are not calibrated in this simulation. |
| 12 | In-hand grasp transition | As task 11 (changing the grasp on an Allen key, a screwdriver or a wrench). |
| 21 | Insert key into lock | The keyway's clearance (0.01 to 0.13 mm) is below what the contact model resolves at a usable time step, so the key could only enter the lock on a joint, which would remove the alignment and search the task is about. Task 27 starts with the key inserted. |
| C2 | the five longer-horizon tasks (assemble toy block, add keys to keyring, unlock and open door, sort utensils, sort fasteners) | not implemented as chained tasks; several of their steps exist as base tasks above. |

Contributions of further tasks are welcome; [developing.md](developing.md) explains how a task is put
together.

## Run times

The simulation runs slower than real time: contact-critical parts are simulated finely, and every frame is rendered.
On an RTX 5090 shared by two to four runs, the insertion, door and dial tasks take 2 to 6 minutes each, the in-hand
screwdriver tasks 4 to 12, the cup tasks 6 to 20, the threaded bulb and cap tasks 8 to 17, and the tasks that pick a
tool up, use it and put it back 30 to 60 minutes. A run alone on the GPU is faster. `TB_RENDER_FAST=1` and
`TB_MULTIVIEW=0` save rendering time. Contact on the GPU is not bitwise reproducible, so measured forces differ slightly
from run to run; the checks have margins for this.
