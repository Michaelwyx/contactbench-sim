# Task variations and random starts

Every task page on the [project website](https://jingyi-xiang.github.io/contactbench/) lists the variations the
printable kit supports: an axis such as *Insertion force*, *Required rotation amount*, *Object geometry* or *Number
of objects*, and the kit parts that realise each value (a stiffer printed spring, a longer screw, another charging
block). In the simulation the same axes are options of the task, under the website's names and with the asset page's
variant labels as values. The rows of `force-bench list` are presets of these options.

## See what a task can vary

```bash
force-bench variations                       # every paper task: the website's axes, the simulated ones, the gaps
force-bench variations plug_in_usb_2         # one row, with the row's own choices marked *
force-bench info plug_in_charger_block1      # one row: its axes, their values, and the row's choice (marked *)
```

`force-bench variations` prints, for each of the paper's 31 tasks, the axes its website page lists, the axes the
simulated task supports with their values, and the website's axes that are not in simulation. In summary:

| # | task | axes in simulation: values | on the website, not in simulation |
|---|---|---|---|
| 2 | Retrieve single utensil from bin | **Scene arrangement**: Chopsticks in horizontal bin (default), Chopsticks in vertical bin, Mixed utensils in vertical bin. **Object type** (the target): Chopstick (default; Spoon in the mixed bin), Fork, Knife, Spoon. **Number of objects**: utensils in the bin, by default 12 / 14 / 20 by arrangement (the videos), within 10–14 / 2–16 / 4–22. Task 2 is a scene without a scripted policy ([tasks.md](tasks.md#2-retrieve-single-utensil-from-bin)) | |
| 4 | Retrieve top cup from stack | none | Number of objects |
| 5 | Retrieve single card from deck | none | Number of objects |
| 6 | Pick up object from platform | none (a card row and a key row) | Object type |
| 9 | Hang object | none (one row per object: keychain, wrench, ruler) | Object type |
| 13 | Tighten screw with Allen key | none | Required rotation amount (Longer screw) |
| 14 | Loosen screw with Allen key | none | |
| 15 | Tighten screw with screwdriver | in-hand rows: **Required rotation amount**: 6.5 revolutions (default), 7.5 revolutions (*Longer screw*) | |
| 16 | Loosen screw with screwdriver | none | |
| 17, 18 | Tighten / loosen nut with wrench | none | |
| 19 | Put bottle cap onto bottle | **Object geometry**: Bottle 1 (default), Bottle 2, Bottle 3. **Required rotation amount**: 2 (default), 2.5, 3, 3.5, 4 revolutions (4 = *More rotation*) | |
| 20 | Thread nut onto screw | none | |
| 22 | Plug in charger | `plug_in_charger_block*` rows: **Object geometry**: Charging block 1 to 4. **Insertion force**: 0.4mm (default), 0.7mm, 0.85mm (*More force*), 1mm indent spring | |
| 23 | Place card into holder | none | Number of objects |
| 24 | Stack cups | none | Number of objects |
| 25 | Plug in Ethernet | none | |
| 26 | Plug in USB | **Object geometry**: USB 1 (default) to USB 4. **Insertion force**: 0.2mm (default), 0.4mm (*More force*), 0.6mm indent spring | |
| 27 | Turn key to unlock door | **Required rotation amount**: 90° (default), 180°, 270°, 360° | |
| 28 | Turn doorknob to open door | **Required rotation amount**: 40°, 48°, 60° (default), 72°, 80°. **Rotation force**: 5mm (default), 7.5mm, 10mm spring. **Object geometry**: Doorknob 1 or 2 at 50, 55 or 60 mm diameter (default Doorknob 1, 55 mm) | |
| 29 | Turn door handle to open door | **Required rotation amount** and **Rotation force** as for 28. **Object geometry**: Handle 1 (default), Handle 2 | |
| 30 | Turn dial to target position | **Object type**: continuous dial that requires pressing down to turn (default), standard continuous dial, snap-to-position dial. **Object geometry**: Flat knob (default), Round knob. **Target position**: a number in degrees (default 90, snap dial 120) | |
| 31 | Install light bulb | **Object geometry**: Lightbulb 1 at 55 (default), 60 or 65 mm, Lightbulb 2 at 45, 50 or 55 mm, Lightbulb 3 at 70 mm. **Required rotation amount**: 2 (default), 2.5, 3, 3.5, 4 revolutions (4 = *More rotation*) | |

"Default" is the task's default; a row may choose another value (`force-bench info <row>` marks the row's own).

Tasks 1, 3, 7, 8, 10, 11, 12 and 21 are not included ([tasks.md](tasks.md#tasks-not-included)).
`force-bench variations` also prints notes on several axes: the door tasks' task pages name the return springs
2.5 / 5 / 7.5 mm, the asset page's files 5 / 7.5 / 10 mm; the 0.4 and 0.85 mm outlet springs are fitted to
measurements, the 0.7 and 1 mm ones interpolated and extrapolated; the USB slot's spring stiffness is an estimate;
light bulb 3 at 80 and 90 mm is wider than the FR3 hand opens, so it is not offered; the kit's bottle 1
"3 revolutions" files carry the 2.5-revolution lead; a *Target position* is a 15° grid tick (a 60° position on the
snap dial).

## Run a variation

```bash
force-bench run plug_in_charger_block1 --set "Insertion force=0.85mm"
force-bench run plug_in_charger_block1 --set "Insertion force=0.85mm" --set "Object geometry=block 3"
force-bench run turn_dial_to_target --set "Target position=120"
force-bench run plug_in_usb_1 --set "Object geometry=USB 3" --name usb3_stiff --set "Insertion force=0.6"
```

- `--set "axis=value"` starts from the row's own settings and replaces the options of that axis. It can be given
  several times, once per axis.
- An axis is named by its website name or a unique, case-insensitive part of it (`force`, `geometry`). A value is
  named as on the asset page, by its exact label or a unique, case-insensitive part of it (`0.85mm`, `block 3`), or
  by the task page's name for it where it has one (`More force`). A numeric axis (*Target position*) takes a number
  in the unit shown. An ambiguous or unknown name stops the run with the list of choices.
- A run with `--set` writes to `outputs/appendix_tasks/<row>__<choices>/` (the first command above:
  `plug_in_charger_block1__force-0.85mm-indent-spring/`), so it does not overwrite the row's default run;
  `--name <folder>` picks the folder name.
- An axis value can change more than a part. Task 2's *Scene arrangement* switches the bin, its contents and how
  the bin is filled.
- A default can depend on another axis: task 2's target is a chopstick except in the mixed bin, where it is a spoon,
  and its default *Number of objects* is 12, 14 or 20 by arrangement. `force-bench variations` marks the task's
  default with `*`; `force-bench info <row>` and `force-bench variations <row>` mark the row's own choices.
- A numeric axis can have a valid range per setting (task 2: 10–14, 2–16 or 4–22 utensils by arrangement). Outside
  it the run stops before the scene is built and prints the range; `force-bench info` and `force-bench variations`
  print the ranges.
- The rows of `force-bench list` are the combinations the scripted policies are tuned for. Any other combination
  runs with the same scripted policy and the same checks; read its verdict, since the policy is not tuned for it.

The same in Python, for a sweep:

```python
import subprocess
for force in ("0.4mm", "0.7mm", "0.85mm", "1mm"):
    for block in (1, 2, 3, 4):
        subprocess.run(["force-bench", "run", "plug_in_charger_block1",
                        "--set", f"Insertion force={force}", "--set", f"Object geometry=block {block}"], check=False)
```

`force_bench.appendix_tasks.variations.apply` turns settings into the task module's own command-line options:
`apply("plug_in_charger_blocks", ("--block", "1"), ["Insertion force=0.85mm"])` gives `--block 1 --spring 0.85`,
which `python -m force_bench.appendix_tasks.plug_in_charger_blocks` also takes directly (`force-bench info <row>`
prints a row's module and arguments).

## Random starts

The paper's protocol samples the initial object poses uniformly within a 0.15 m × 0.15 m square at the centre of the
board. Tasks that support it take `--seed N`:

```bash
force-bench run tighten_screw_with_screwdriver --seed 3
force-bench run plug_in_usb_3_spring06 --seed 3
force-bench run stack_cups --seed 2
force-bench run tighten_screw_with_screwdriver --seed 2 --set "rotation=7.5"   # combines with --set
```

- Without `--seed` a task starts from its nominal, documented pose.
- With `--seed N` the task draws its objects' positions (and, where the task allows it, their yaw) from a random
  generator seeded with `N`, within the ranges its section in [tasks.md](tasks.md) gives. The draw rejects poses
  where two objects would overlap. The same seed gives the same start on every machine.
- The drawn start is written to the verdict (`appendix_qa.json`, under `criteria`) and the run goes to
  `outputs/appendix_tasks/<row>__seed<N>/` (with `--set` as well: `<row>__<choices>_seed<N>/`).
- `--set` and `--seed` also combine with `--embodiment` on a task that supports other fingers
  (`force-bench run retrieve_utensil_from_bin --embodiment finray --seed 2`, into `<row>__finray_seed2/`;
  [embodiments.md](embodiments.md)).
- The contact computation on the GPU is not bitwise reproducible, so two runs from the same start can differ
  slightly in their forces; the start itself is identical. A scene that the physics fills before the episode (task
  2's bin, filled by letting the utensils fall in) settles slightly differently from run to run even with the same
  seed: the same seed gives the same bin pose and fill plan, and `scene.json` records the scene each run produced.
- A task without a random start stops with "this task has no randomized start yet".

| rows | what is drawn | range | in the verdict |
|---|---|---|---|
| `retrieve_utensil_from_bin`, with any `--set` arrangement | the bin's position and yaw, and the fill: the order the utensils go in and where | the 0.15 × 0.15 m square around the bin's nominal position, yaw within ±15° | `scene.json` (`start`, `fill`) |
| `retrieve_top_cup_from_stack` | the stack's position; the cup is set down at its nominal offset from the stack | within ±50 mm of the stack's nominal position (the fixed camera's view); no yaw (the cups are round). The hand starts at the same pose for every seed. | `start` |
| `tighten_screw_with_screwdriver`, `tighten_screw_with_screwdriver_longer_screw` | the toy assembly block's position and yaw; the screw keeps its place in the block | the 0.15 × 0.15 m square at the board centre; yaw within ±30° of the video's arrangement. The hand starts at the same pose for every seed. | `start_state` |
| `loosen_screw_with_screwdriver` | as for tighten | as for tighten | `start_state` |
| `put_bottle_cap_onto_bottle`, `_2`, `_3`, `_4rev` | the bottle's position; the cap starts in the pinch above it | the 0.15 × 0.15 m square at the board centre; no yaw (the bottle is round; `--yaw0-deg` sets the cap's thread phase) | `start` |
| `stack_cups` | the positions of the three cups, drawn jointly | each in the 0.15 × 0.15 m square centred on the nominal layout, rims at least 6.4 mm apart; no yaw (the cups are round). The hand starts at the same pose for every seed. | `start` |
| `plug_in_ethernet` | the 5-port slot's position and yaw | the 0.15 × 0.15 m square at the board centre, yaw within ±20°. The hand starts above the chosen port wherever the slot lands. | `start` |
| `plug_in_usb_1` … `plug_in_usb_4` and their `_spring04` / `_spring06` rows | the USB slot's position and yaw; the stick starts in the pinch above the slot, the hand turned with it | the 0.15 × 0.15 m square at the board centre, yaw within ±20° of the nominal orientation. The robot base stays where it is. | `start` |
| `turn_dial_to_target`, `_round`, `_standard`, `_standard_round`, `_snap`, `_snap_round` | the dial's position and yaw; the target stays relative to the dial's 0 mark, and the hand starts around the knob, turned with the dial | the 0.15 × 0.15 m square at the board centre, yaw within ±15° (the FR3's last wrist joint turns through the dial's yaw plus the target). The robot base stays where it is. | `start` |
| `install_light_bulb`, `_4rev`, `_2_50mm`, `_3_70mm` | the bulb socket's position; the bulb starts in the pinch above it | the 0.15 × 0.15 m square at the board centre; no yaw (the socket is round; `--yaw0-deg` sets the bulb's thread phase) | `start` |

Each entry under "in the verdict" is a criterion of `appendix_qa.json` (for task 2, an entry of `scene.json`) that
holds the seed and the drawn poses; it is information only. The rows that start with the tool or part on the table
(`tighten_screw_with_screwdriver_from_table`, `loosen_screw_with_screwdriver_from_table` and
`install_light_bulb_from_table`) do not take `--seed`, nor do the tasks not listed here.

A task of your own draws a random start the same way: describe its objects with `Placement` (nominal position and
yaw, the range to sample in, a footprint radius, and optionally the centre of a square that several objects are
scattered over) and draw them with `sample_start(placements, seed)`, both from `force_bench.appendix_tasks.common`;
see [developing.md](developing.md#adding-a-task).
