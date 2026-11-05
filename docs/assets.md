# Assets

Every object in the simulation is made from the CAD of the 3D-printable ContactBench kit. The printable files, a 3D
view and print notes for each object are on the project's asset page,
<https://jingyi-xiang.github.io/contactbench/assets.html>. This page describes how the same objects are simulated:
the kit library and its commands, how each printed mechanism is modelled, where its parameters are set, the checks,
the USD export, and the colours of the rendered scenes.

## The kit library

All 49 assets of the asset page are in the simulation, with every variant the page offers (spring stiffnesses,
rotation amounts, geometries), 137 variants in all. `assets/kit/` holds the kit as the asset page publishes it, and
`force_bench.kit` builds simulation objects from it.

| file | contents |
|---|---|
| `assets/kit/<asset>/*.stl` | every printable part, as published: millimetres, Z up, each asset's parts in place in its CAD assembly frame (a cap on its bottle, a knob on its dial base). The page's multi-body visual models (`.obj`, in centimetres) are kept where an assembly needs them, e.g. the door with both modules mounted. |
| `assets/kit/catalog.json` | the asset page transcribed: every asset with its parts, its variant groups and its print colours, and a SHA-256 hash per file. Written by `force-bench kit import`; not edited by hand. |
| `assets/kit/tasks.json` | the website's task pages: each task's description, success criterion, assets and variation axes. Written by `force-bench kit import`; not edited by hand. |
| `assets/kit/sim.yaml` | how each asset is simulated (hand-written): kind, friction, collision resolution and contact margins, masses where they are given, the mechanism model, and non-printed parts such as the platform's acrylic sheet. |

```bash
force-bench kit list                      # every asset: kind, parts, number of variants, variant groups
force-bench kit info outlet               # parts with size, estimated mass, colour; variants; sim.yaml settings
force-bench kit check                     # build, settle and probe every variant -> outputs/kit/
force-bench kit export-usd outlet         # one USD stage per variant, joints and springs included (usd extra)
force-bench kit check --from-usd outlet   # the same checks on the asset imported from its stage
force-bench kit import                    # re-import assets/kit/ from the project website (--site: a local mirror)
```

```python
from force_bench.kit import load_kit, add_asset, place_on_table

kit = load_kit()
kit["outlet"].groups                      # [Group('Insertion force', ['0.4mm indent spring', ...])]
h = add_asset(rig, "outlet", "0.7mm", pose=place_on_table("outlet", xy=(0.45, 0.0)))
h.bodies, h.joints, h.params              # what was built, and the variant's physical parameters
```

A variant is chosen by its label, by a unique part of a label (`"0.7mm"`), or as `{variation: label}` for assets with
several groups. Groups that share a slot are alternatives (a door handle or a doorknob on the doorknob module); assets
whose files are combinations of two groups (a light bulb's diameter and its number of turns) take one label from each.
`place_on_table` returns the pose that puts the asset's lowest point on the table at `xy`. Tasks 2, 4, 19, 24 to 31
and the in-hand rows of 15 and 16 build their scenes this way ([tasks.md](tasks.md)); building a scene of your own
is described in [custom_scenes.md](custom_scenes.md).

## How an asset is simulated

`sim.yaml` gives every asset one of four kinds.

- **free** (75 variants): one dynamic body: cards, keys, key fob, keyring, ruler, utensils, tools, fasteners,
  charging blocks, USB sticks, cup, light bulbs.
- **fixture** (12 variants): static geometry: card holder, bins, hanging board, toy assembly block, light-bulb
  socket, platform. `add_asset(..., fixed=True)` bolts a free object down; `fixed=False` lets a fixture move.
- **assembly** (15 variants): several bodies without joints, held together by contact only: a bottle and its cap,
  whose thread is the printed geometry.
- **mechanism** (35 variants): bodies and joints built by `force_bench.kit.mechanisms.<model>` with the variant's
  parameters: the outlet, the USB slot, the Ethernet plug and slot, the three dials, the door, the doorknob module
  and the lock module ([below](#from-printed-part-to-simulated-mechanism)).

**Collision.** Every part collides through a signed-distance field of its own mesh. The default cell size is 0.4 mm;
parts with tight fits or thin walls get finer fields and smaller contact margins in `sim.yaml`: 0.12 mm cells and a
0.1 mm contact gap for the charging blocks, whose blades have 0.25 mm of play per side in the outlet, and for the USB
sticks; 0.2 mm cells for the cup's 1 mm wall.

**Mass.** Unless `sim.yaml` gives a mass (`mass_g`), a part's mass is estimated from its mesh for an FDM print with
the asset page's print profile (PLA, 0.4 mm nozzle, 20 % infill): solid walls of 0.8 mm (surface area × thickness,
capped at the volume, so thin parts such as cards and keys come out solid) plus 20 % of the remaining volume, at
1240 kg/m³. A kit card comes out at 6.9 g, a key at 1.4 g, a cup at 15.2 g. `force-bench kit info <asset>` prints
the estimate of every part.

**Non-printed parts.** The platform needs a 10 × 10 inch acrylic sheet; it is a 254 × 254 × 3.18 mm box resting on
the two printed halves, placed as in the asset page's model.

## From printed part to simulated mechanism

Every mechanism follows the same rules.

1. **Rigid parts and joints from the CAD.** Each printed body is a rigid body with the CAD's geometry. Joints use the
   axes, travel limits and fixed supports of the assembly: the door hinge, the latch slide, the dial axis, the lock
   cylinder.
2. **Printed springs become joint drives.** A printed flexure or spring is not simulated as a deforming body. It is a
   prismatic or revolute spring–damper drive on the joint it acts on, or a sprung contact body where it acts by
   contact (the outlet, the USB slot, the dials). Its stiffness and preload are explicit parameters, measured where a
   measurement exists and otherwise estimated from beam theory or a finite-element model of the CAD section, and
   marked as such. The robot feels the printed part's resistance (the force to press a latch, turn a handle or seat a
   plug); the spring's deformation itself is not simulated.
3. **Fine contact where it matters.** Contact-critical geometry (threads, keyways, insertion passages, card slots,
   latch hooks) collides through signed-distance fields of the meshes, at a resolution chosen per part. Where
   detailed geometry is not needed, analytic primitives (boxes, capsules, spheres) are used, which are faster and
   give cleaner contact normals.
4. **Features below contact resolution are approximated.** A feature smaller than what the contact model resolves
   (the 0.01 to 0.13 mm keyway clearance, the dial knob's running fits under 0.3 mm) is carried by a joint or a force
   model instead. Every such choice is listed in the task's `changes_vs_suite_task` and in [tasks.md](tasks.md).
5. **Masses** come from the print estimate above unless given. A few parts are heavier for solver stability; the
   task's differences in [tasks.md](tasks.md) state each one.

| mechanism | printed part | simulated as |
|---|---|---|
| charger outlet (kit) | a printed indent spring in each blade slot (0.4, 0.7, 0.85, 1 mm indent) | a sprung retention pad in each slot (prismatic spring–damper) with a curved entry that the blades wedge open; protrusion and rate per spring, fitted to measured insertion forces for 0.4 and 0.85 mm, interpolated for 0.7 mm and extrapolated for 1 mm |
| USB slot (kit) | one printed spring part: two jaws on serpentine flexures (0.2, 0.4, 0.6 mm indent) | the two jaw blocks, cut from the CAD with their lead-ins, on prismatic joints at an estimated 66 N/mm each (finite-element model of the flexure), standing into the stick's path by the spring's indent |
| Ethernet latch (kit) | the plug's latch tab, a 1.0 mm printed cantilever that catches behind the slot inserts' lips | the tab cut from the CAD at its root, on a revolute joint with a spring drive at an estimated 0.72 N·m/rad (finite-element model of the printed tab) and a hard stop where it lies on the plug; the catch is contact with the inserts' lips ([details](#the-ethernet-plug-and-slot)) |
| dials (kit) | standard: six flexure-hung brake pads on the knob's hub; snap-to-position: the same pads on a hexagonal hub, which makes a detent every 60°; push-down: two accordion springs under a gated cover | radial prismatic pads at an estimated 29 kN/m each against the CAD hub; two vertical sprung plates at an estimated 3.9 kN/m, and the cover's tooth and peg as contact sectors. The push-down knob's rotation carries a 2.5 × 10⁻³ kg·m² armature for solver stability; pressed, it turns at 0.19 N·m with either knob (Coulomb estimate 0.28 N·m) |
| door (kit) | the leaf in its frame on a pinned hinge; the latch and deadbolt pockets in the strike jamb | the leaf on a revolute joint on the CAD hinge axis, with the doorknob and lock modules at their assembled poses; the pockets' walls are boxes on the CAD's free volumes. The 30° entry ramp above the latch pocket, which cams a closing latch in, is not modelled |
| doorknob module (kit): latch, doorknob, lever | a spiral return spring (5, 7.5, 10 mm); rack and pinion to the latch, the pinion's size setting the unlock angle (40° to 80°) | grip (doorknob or handle), pinion and spring hub as one link on the spindle; a spindle spring with the measured stiffness and preload of each printed spring; the rack and pinion as a stiff joint coupling at the CAD pinion's pitch radius |
| lock module and key (kit) | the key turns the plug, whose pinion throws the deadbolt (90° to 360° to unlock); a friction spring holds the bolt | key seated in the plug (inserting it is not modelled); plug → pinion → deadbolt as a stiff joint coupling; dry friction on the bolt (1.25 N, an estimate) |
| bottle caps and light bulbs (kit) | threads | the published thread (single start, 1.2 mm deep, 0.3 mm radial clearance) colliding through fine signed-distance fields; the cap or bulb is a free rigid body that only the fingertips hold |
| nut on bolt (task 20) | threads | real helical geometry colliding through signed-distance fields |
| screws in the toy assembly block (tasks 13 to 16) | threads | an axial and a rotary degree of freedom coupled by a helix spring, with load-dependent friction, a hard seat and a seated holding torque; the tool–screw drive is real contact |

Each mechanism module in `force_bench/kit/mechanisms/` documents its model, its parameters and their sources in its
docstring. Measured force profiles of printed handles and outlets, and the parameters that reproduce them, are in
[examples/force_response](../examples/force_response/README.md). [physics_notes.md](physics_notes.md) lists the
modelling rules behind these choices (units, spring placement, articulation membership, contact buffers,
signed-distance fields of thin parts).

### The Ethernet plug and slot

`ethernet-plug-and-slot` is simulated as the real-world task uses it: the black 98 × 32.5 × 32.5 mm shell, a grey
printed insert in each of its five ports (the kit ships one insert file; the asset page's assembled model and the
task video have one in every port), and the white printed plug, without a cable. The shell and the inserts are
static. Each port is a 12.0 mm wide pocket whose insert gives it a floor, a back wall and a roof; over the outer
2.3 mm of each side the roof carries a lip (its bottom 20.3 mm above the shell's base) whose inner face, leaning 15°,
is the catch. The plug's body is 11.0 mm wide (0.5 mm of play per side) and is fully inserted when its nose reaches
the insert's back wall, 15.0 mm inside the mouth.

The latch tab is a printed cantilever: a 1.0 mm plate rising at 20° from the plug's nose, 11.0 mm wide up to its
shoulder and 6.0 mm wide beyond it. The plug is cut at the tab's root, and the tab is a second rigid body on a
revolute joint through the middle of the root section. A spring drive holds it in its printed (relaxed) pose, and a
hard joint limit stops it where it would lie flat on the plug. The tab collides with everything except the plug it is
printed on. Going in, the lips press the tab down; its shoulder passes under them once the tab is pressed 15.2°
(2.6 mm at the shoulder), and 0.85 mm before the seat it springs back behind them. Seated, it rests at 5.0° against
the ramped roof.

| parameter | value | source |
|---|---|---|
| hinge stiffness | 0.72 N·m/rad (8.3 N/mm at the shoulder) | **estimate**, not measured: plane-stress finite-element model of the tab's side section, E = 3 GPa. E = 2 to 3.5 GPa gives 0.48 to 0.84 N·m/rad, plane strain 0.82; ±0.05 mm of printed thickness changes it by ±15 %. Beam theory for the 11 × 1.0 mm section on a rigid root gives a 23 % stiffer tab (it leaves out the root's own compliance). |
| hinge stop | 19.8° | CAD: the tab's underside flat on the plug |
| hinge damping | 5 × 10⁻⁴ N·m·s/rad | about half of critical |
| hinge inertia | 3 × 10⁻⁷ kg·m² | conditioning value for the solver's time step (the printed tab: 1.6 × 10⁻⁸ kg·m²) |
| plug mass | 15 g | conditioning value for contact stability (print estimate 1.8 g) |
| friction | 0.4 | printed PLA on PLA, as for the other printed connectors |

`force-bench kit check ethernet-plug-and-slot` seats a free plug in port 3 and drives a second plug into port 1 at
5 mm/s. The insertion force is 0.7 N while the body slides in, rises once the lips meet the tab (9.9 mm before the
seat) to a peak of about 14 N, and turns negative as the tab's shoulder rides down the lips' leaning faces and pulls
the plug in; the tab snaps back 0.86 mm before the seat. The latched plug holds a 30 N pull. With the tab pressed to
its stop (0.32 N·m, about 23 N at the mouth), a 10 N pull extracts it against 2.8 N, and the plug latches again when
re-inserted. A latched plug that nothing pushes in rests on its catch, about 0.8 mm out of its seat.

Limits of the model:

- A single hinge reproduces the printed tab's stiffness at its shoulder, where the catch and the largest deflection
  are. Further out it is stiffer than the printed tab (by 44 % at the mouth and 85 % at the free end), where the
  narrow part bends as well.
- At the 15.2° needed to pass the lips, linear elasticity puts about 110 MPa at the root of the printed tab, above
  printed PLA's flexural strength. The real tab may yield, and the real insertion force may be lower than the
  simulated one.
- By the same linear model, a finger pressing the printed tab's free end bottoms it on the plug before the shoulder
  clears the lips, so the printed latch is released by pressing near the mouth. The simulated rigid tab releases
  wherever it is pressed.

## Changing a parameter

- **Friction, collision and mass**: the asset's entry in `assets/kit/sim.yaml` (`mu`, `sdf_cell_mm` or `sdf_res`,
  `margin_mm`, `gap_mm`, `mass_g`, per asset or per part under `parts:`). The file's header lists every key.
- **A mechanism's springs, preloads, friction and masses**: the named constants at the top of its module in
  `force_bench/kit/mechanisms/`, each with its unit and source, or keyword arguments of `add_asset`, which passes
  them to the mechanism's builder: `add_asset(rig, "outlet", "0.7mm", ke=45000.0)`,
  `add_asset(rig, "doorknob-module", "Handle 1", K=0.6, tau0=0.2)`. Each module's docstring and `build` function
  list the parameters it takes.
- **A re-tuned variant as a USD stage**: `force-bench kit export-usd <asset> --variant <label> --param NAME=VALUE`
  ([custom_scenes.md](custom_scenes.md#export)).

After editing `sim.yaml` or a module constant, `force-bench kit check <asset>` shows the effect on the asset's
settle check or probe, and re-running a task shows it on the task. Changing the parameters of a task is described in
[getting_started.md](getting_started.md#change-a-physical-parameter).

## Checks

`force-bench kit check` builds every variant of every asset on a table and checks it:

- **settle**: free and assembled parts are dropped from 1 mm and left for 1.5 s; the state must stay finite, nothing
  may tunnel through the table, and speeds at the end must be below 5 mm/s and 0.5 rad/s;
- **overlap**: the parts of an assembly (a cap on its bottle) must not end up interpenetrating by more than 0.5 mm;
- a fixture gets a 10 mm cube dropped onto it, which exercises its collision surface;
- **probe**: a mechanism runs its own test, for example spring force against displacement or rotation to unlock.
  For the outlet, charging block 3 is driven in at 10 mm/s and the force is recorded against depth; for the Ethernet
  slot, the sequence [above](#the-ethernet-plug-and-slot).

```bash
force-bench kit check                     # every asset and variant -> outputs/kit/summary.json
force-bench kit check outlet usb-slot -j 2   # some assets, two processes in parallel
force-bench kit check --summary           # only re-collect the existing reports
force-bench kit check --no-render         # without the still images
```

Each variant writes `outputs/kit/<asset>/<variant>/report.json` and `still.png` (the scene at the end, in the kit's
colours); `--out <dir>` writes elsewhere. Each asset runs in its own process.

## USD stages

`force-bench kit export-usd` writes every variant as a self-contained USD stage for Newton
(`outputs/kit_usd/<asset>/<variant>.usda`; metres, kilograms, Z up): the bodies with their masses and inertias, every
shape with its mesh, print colour, friction, contact and signed-distance-field settings, and the mechanisms' joints,
limits, spring drives, gear couplings and collision filters, in UsdPhysics with Newton's and MuJoCo's schema
attributes on top. `force_bench.kit.usd.add_asset_usd` adds a stage to any Newton `ModelBuilder`, next to a robot of
your own, and gives the same model as `add_asset`. [custom_scenes.md](custom_scenes.md) describes both routes, the
solver settings the calibration assumes, and how to compare a stage with the Python-built asset
(`force-bench kit check --from-usd --compare`).

The springs and contacts are calibrated against MuJoCo Warp with Newton's contacts. The stages are standard
UsdPhysics underneath but are not tested in Isaac Sim or Isaac Lab: PhysX ignores the Newton- and MuJoCo-specific
attributes (signed-distance collision, the gear couplings, the doorknob's spring preload, the hard joint stops), and
the springs and contacts would need re-tuning there ([custom_scenes.md](custom_scenes.md#other-simulators)).

## Other asset files

The tasks that build on the scene modules of `force_bench.tasks` load part of their geometry from other folders:

| path | contents |
|---|---|
| `assets/bodies/<asset>/*.stl` | one mesh per rigid part, exported from the Fusion 360 archive of the benchmark objects, in **centimetres** in the assembly frame of each design (`force_bench.assets` converts to metres). `inventory.json` lists every file with its face count, extent and whether it is watertight. |
| `assets/bodies_kit/<part>/*.stl` | kit parts that differ from or are missing in the archive, tessellated from a STEP export of the kit, in **metres**, each folder with a `provenance.json`. |
| `assets/manifest.yaml` | default physical parameters (PLA density and friction, contact offsets, materials) and, per articulated asset, the intended function of its joints. |

Where these parts differ from the final kit in a way that matters, the tasks load `assets/kit/` instead. The other
parts agree with the final kit in volume and extent (the four charging blocks, for example, to 0.03 % in volume); a
remaining difference is listed under the task's differences in [tasks.md](tasks.md). `assets/finray/` holds the
Fin Ray finger and its FR3 adapter ([finray.md](finray.md)).

The Franka FR3 description is downloaded from Newton's asset repository on first use
([installation.md](installation.md#the-robot-description-downloaded-on-first-use)).

## Colours

Scenes are rendered in the filament colours of the printed kit, on a tan ground with a light-grey work surface and a
light backdrop. Parts built from the kit library carry their print colour from the asset page (white, yellow, grey or
black); other shapes are coloured by substring rules on shape and body labels in `force_bench/kit_look.py`, kept per
task. The robot and the GelSight sensors keep their own colours. The colours affect only the rendered images:
collision, mass and contact are unchanged. `TB_LOOK=studio` switches to the generic studio palette (accent colours
per moving body on a dark backdrop), which makes the moving parts of a scene easier to tell apart.
