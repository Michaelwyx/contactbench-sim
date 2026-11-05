# Fin Ray finger on the Franka Hand (FR3 / Panda): adapter and finger

A printed adapter that replaces each stock Franka Hand finger with the 3D-printed Fin Ray finger
(`finray_finger.step`, single solid, 40 x 25.8 x 123.4 mm, 47.1 cm³). Two adapters, two fingers, the fingers'
straight contact faces facing each other.

![assembly](assembly_fr3_finray.png)

## Files

| file | what |
|---|---|
| `adapter_finray_fr3.step` / `.stl` | the adapter, mm, in the finger frame below. **Print two** (left and right are the same part). |
| `adapter_finray_fr3_lug.step` / `.stl` | optional variant whose cheeks also pin the finger's lug hole (see [The lug hole](#the-lug-hole)) |
| `adapter_finray_fr3.py` | parametric CadQuery source of both; every dimension is a named parameter at the top |
| `adapter_spec.json` | the parameters, the derived positions, volume and mass, written by the script |
| `finray_finger.step` | the finger as supplied (Fusion export) |
| `finray_finger.stl` | the same, mm, watertight (converted with cascadio), the print file |
| `fr3_carriage_tab.stl` | the Franka carriage tab with its two M4 holes, for checks and renders |
| `finray_profile.json` | the finger's measured walls and ribs (used by the simulation, `force_bench.finray`) |
| `render_assembly.py` | makes `assembly_fr3_finray.png` (hand mesh, tabs, adapters, fingers; dimensioned section) |

To regenerate the adapter files and the figure, install CadQuery in a separate environment (for example
`conda create -n cad python=3.11 && pip install cadquery`), then run `python adapter_finray_fr3.py --lug` (`--lug`
also writes the lug variant) and `python render_assembly.py`.

## Frame

The frame of the URDF finger links (`fr3_leftfinger` / `fr3_rightfinger` in `fr3_franka_hand.urdf`): x across the
finger, y outward from the gripper centre (the stock pad face is the plane y = 0), z along the approach axis away
from the hand, origin 58.4 mm below `fr3_hand`. Both fingers use it (the right one is turned 180° about z).

The finger is mounted with its straight contact flank in the plane y = 0, **exactly where the stock pad face is**:
the finger coordinate keeps its meaning (face gap = 2 q), the full 0-80 mm stroke is kept, and closed fingers
touch. Its length runs along +z; its tip is 146.8 mm below the carriage (stock fingertip 54.5 mm), i.e.
**101.8 mm below the stock TCP** (`fr3_hand_tcp`, 103.4 mm from the hand). Set your TCP accordingly.

## The Franka interface, and where each number comes from

| dimension | value | source |
|---|---|---|
| carriage tab thickness (y) | 6.00 mm | **official drawing**: Franka Hand product manual R50010 v1.2, §5.2.4, Fig. 5.5 |
| tab width (x) | 21 mm | official drawing, Fig. 5.5 |
| tab protrusion below the housing | 11.40 mm | official drawing, Fig. 5.5 |
| tab screws | 2 x M4 per tab, 12 mm apart (x = ±6), axes along y, 7 mm from the tab's free end (15 - 8) | official drawing, Fig. 5.5 |
| recommended screws | 4 x M4x12 (two per finger), 2.25 N·m | manual §5.2.4 (for the stock metal finger) |
| tab inner faces, gripper closed / open | 25 / 105 mm apart | official drawing, Fig. 5.4 (travel range) |
| design finger length | 54 mm | manual §5.2.4 ("designed and tested for a finger length of 54 mm") |
| housing underside below the finger joint origin | 7.6 mm (66.0 - 58.4) | **official files**: FR3 description meshes (`franka_description`, `hand.dae`; the ones Newton downloads) |
| tab position in the finger frame | y 12.5 .. 18.5, end at z 19.0, M4 axes at z 12.0 | **derived**: 25 / 2; 7.6 + 11.4; 19.0 - 7.0. Cross-check: the stock `finger.dae` has its two screw holes at z 12.0, x ±5.8 |
| finger base holes | Ø4.0, along x, at y 5 and 35, 5 mm below the base top | measured from `finray_finger.step` |
| finger lug hole | Ø4.0 at y 35, 40 mm below the base top | measured from the STEP |

Franka publishes no STEP file of the finger interface, so the tab geometry is the manual's drawing plus the
description meshes.

### Check these with calipers before printing

1. **Tab thickness and width** (6.00 x 21.0 mm). The pocket is 6.30 x 21.50 mm (`pocket_clear`, `pocket_clear_x`).
2. **M4 hole spacing (12.0) and the hole axis's distance from the tab's free end (7.0).**
3. **Tab protrusion below the housing (11.4).** The adapter's top face stays 0.8 mm below the housing
   (`housing_clear`). A shorter tab is fine (the pocket floor is 0.4 mm past the tab end); a tab longer than
   11.8 mm bottoms out.
4. **Gap between the two tabs' inner faces with the gripper closed: 25.0 mm expected.** It sets where the two
   contact faces meet. If you measure 25 + 2e, the closed faces stop e mm short of each other (e < 0: they
   interfere); change `tab_gap_closed` and re-export. This is the number the drawings support least directly
   (the description's visual mesh puts the tab about 1 mm further in).
5. The housing underside is flat over the adapter's footprint (x ±18, y 1..40 mm from the pad plane) along the
   whole stroke.
6. The printed finger's Ø4 holes: TPU usually prints them undersize, so an M4 shank fits snugly (ream with a
   4 mm drill if it does not pass).

## Print

* **Adapter**: PETG (or PLA+, ASA, PA-CF), 0.2 mm layers, 100 % infill (or ≥ 5 walls + 60 % gyroid). Lay it on
  its **inner face** (the flat y = 1 face, the one facing the other finger): every feature is then prismatic along
  the print direction except the two Ø4.5 finger-bolt holes (they bridge fine) and the roof over the tab pocket
  (a 21.5 mm bridge). No supports. 13.98 cm³, about 18 g at 100 % PETG.
* **Finger**: TPU 95A. The simulation assumes E ≈ 26 MPa, the data-sheet modulus of Ultimaker TPU 95A / NinjaTek
  Cheetah; an 85A grade is about half as stiff, so halve every force of the probe results
  ([docs/finray.md](../../docs/finray.md#validation)). Lay it on one of its 25.8 mm faces so walls and ribs are
  extrusions; 100 % infill (the walls are 1.2-6 mm, perimeters only), 20-30 mm/s, no supports. About 57 g.

## Hardware (for two fingers)

| part | qty | use |
|---|---|---|
| M4x12 socket head cap screw (ISO 4762) + M4 washer (ISO 7089) | 4 + 4 | adapter onto the carriage tab, into the tab's threads (full 6 mm engagement) |
| M4x40 socket head cap screw (ISO 4762) + M4 washer | 4 + 4 | finger into the adapter, through the finger's two base holes |
| M4 hex nut (ISO 4032, 7 mm AF) | 4 | in the hex traps of the +x cheek |

**M4, not M3, for the finger**: the finger's holes are Ø4.0, so an M4 shank fills them and the soft base cannot
shift on its bolts; M3 would leave 0.5 mm of radial play in TPU and rely on clamping friction alone.

## Assembly

1. Remove the stock fingers (two M4 screws each, from the outside).
2. Push an adapter up onto each carriage tab (the tab enters the 6.3 mm pocket, open side up), align the holes,
   and drive two M4x12 + washers in from the outside (the side away from the other finger). Tighten to
   **about 1 N·m**, not the manual's 2.25 N·m: that value is for the metal stock finger and crushes PETG under the
   heads.
3. Drop an M4 nut into each hex trap on the +x cheek.
4. Slide the finger's base block up between the cheeks until it touches the bridge (contact face toward the
   other finger, curved back flank outward), align the holes and fit two M4x40 + washers from the -x cheek.
   Tighten until the TPU is just compressed (about 0.5 N·m).
5. Close the gripper: the two contact faces should meet along their whole length (check 4 above if not).
6. Keep the grasp force moderate (about 30-40 N or less, rather than the hand's 70 N maximum): the finger reaches
   147 mm below the carriage against the 54 mm the hand is rated for, so the same force puts about 2.7 times the
   tilting moment on the carriage guides.

## The lug hole

The finger has a third Ø4 hole on a lug of its back flank, 40 mm below the base. `adapter_finray_fr3_lug.*`
extends both cheeks down to pin it (one more M4x40 + nut). Pinning the lug stiffens the finger most near its root,
where it matters least for wrapping: in a linear frame model of the finger (`force_bench.finray.frame_fe`), the press
stiffness at 45 / 60 / 75 / 90 mm below the base rises from 1.84 / 1.08 / 0.69 / 0.54 N/mm to
6.98 / 2.05 / 0.96 / 0.67 N/mm. **Default: base holes only**; use the lug variant if the finger is too soft near its
root for a task.

## In simulation

`force_bench.finray` models the finger on the FR3 (the adapter as boxes on the carriage, the finger as a
pseudo-rigid-body linkage of its measured walls and ribs) and provides it as the `finray` embodiment:
`force-bench run retrieve_utensil_from_bin --embodiment finray`. See [docs/finray.md](../../docs/finray.md) and
[docs/embodiments.md](../../docs/embodiments.md).
