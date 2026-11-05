# Fin Ray fingers

A 3D-printed Fin Ray finger (TPU) mounted on each carriage of the FR3's Franka Hand through a printed adapter, and
its simulation model. In the simulator it is the `finray` embodiment ([embodiments.md](embodiments.md)):

```bash
force-bench run retrieve_utensil_from_bin --embodiment finray
```

The printable parts, where each dimension comes from and how to assemble them are in
[`assets/finray/README.md`](../assets/finray/README.md); the code is `force_bench.finray` ([Code](#code)).

## The physical finger and adapter

**The finger** (`assets/finray/finray_finger.step`, one solid): 123.4 mm long from the top of its base block to the
tip, 25.8 mm wide, with a 40 × 25.8 × 10 mm base block, a straight 1.8 mm contact wall, a back wall 6.0 mm thick at
the base tapering to 2.5 mm near the tip, and fifteen ribs (2.0 mm; the lowest five 1.2 mm) sloping from 0° at the
base to 58° toward the tip. 47.1 cm³, about 57 g in TPU 95A. Two Ø4 mm holes through the base block hold it; a third
on a lug of the back wall, 40 mm below the base, is optional.

**The adapter** (`assets/finray/adapter_finray_fr3.step` / `.stl`, CadQuery source `adapter_finray_fr3.py`, print
two): a PETG block that clamps the Franka Hand's carriage tab with two M4 × 12 screws and holds the finger's base
block between two cheeks with two M4 × 40 screws and nuts. The tab dimensions come from the Franka Hand manual's
drawing and the FR3 description meshes; the asset README lists the dimensions to check with calipers before
printing. The finger's contact face sits exactly where the stock pad face is, so the 0 to 80 mm stroke is kept and
closed fingers touch. The tip is 146.8 mm below the carriage (the stock fingertip: 54.5 mm), **101.8 mm beyond the
FR3's TCP**: grasp poses need that offset. The hand is rated for 54 mm fingers; keep the grasp force at about 30 to
40 N or less. A variant (`adapter_finray_fr3_lug.*`) also pins the lug hole.

## The model

**Material.** TPU 95A with E ≈ 26 MPa, from two TPU 95A filament data sheets; **it is an estimate, not fitted to a
printed finger**. The walls bend in plane strain across the 25.8 mm width (E' = E / (1 − ν²), ν = 0.45). Every force
below scales with E; the shapes do not. A printed part can be softer than the bulk value: read the forces as ±50 %.

**Pseudo-rigid-body linkage.** The walls and ribs are rigid links on revolute joints about the finger's width axis,
and the profile's closed loops stay closed:

- contact wall: 21 links, a joint at every rib root (the 22 mm span below the last rib split in three);
- back wall: 20 links, a joint at every rib's outer end;
- ribs: two halves each, hinged at both walls and **welded** at mid-rib; the two walls are welded at the tip. A weld
  is a loop-closing FIXED joint, which MuJoCo turns into a WELD equality. Welding the halves, rather than pinning
  them, keeps a bending spring at both ends of every rib; with pinned rib ends the last bays fold under a 10 mm
  press.
- springs: wall joints `E'I / l` (the discrete-rod bending rate of the wall, `l` the mean length of the two adjacent
  links, the root joints clamped), rib ends `6 E'I / L` each (a rib bending with its inflection at mid-length). These
  are the theory values, with no fitted factor.
- masses from the TPU density and the link volumes, scaled to the printed 57 g.

Per finger: 71 bodies, 71 revolute joints and 16 welds; the two fingers add 142 bodies to a scene.

**Solver settings**, which do not change the statics: every joint gets armature `a = k (dt / 0.35)²`, so each
spring's own ω·dt is 0.35 whatever its rate (the links weigh tenths of a gram on springs up to 7 N·m/rad; without
armature ω·dt ≈ 8 and the explicit spring diverges); the damping ratio is 0.5; and the welds' `solref` is set to
(2 dt, 1) after the solver is built (MuJoCo's default is a 20 ms spring, far softer than the walls). The finger's
shapes are boxes along each wall link (the ribs do not collide): TPU against the task's parts with μ = 1.0 and a
0.4 mm contact gap.

**On the FR3.** The stock finger shapes are switched off; each carriage carries the adapter (boxes) and the finger's
base block, and the linkage hangs below it. The carriage keeps the task's finger mass and drive (in task 2: an
8 kN/m position drive limited to 15 N). The contact face is the stock pad plane, so a finger command `q` means what
it does for the stock fingers (faces `2 q` apart). The linkage's lowest point is **100.8 mm beyond the TCP**; the
printed tip is 1.0 mm lower (the solid wedge below the junction of the two walls is not modelled). Body labels start
with `fr3/finray_`, so every robot-contact probe of a task counts the finger. The scene runs with MuJoCo `impratio`
10 and collision every substep: the contact links weigh 0.2 to 0.4 g, and MuJoCo's contact rows scale with the mass
of the bodies they act on ([physics_notes.md](physics_notes.md), rule 2.7), which makes the light links' friction
soft unless the friction rows are weighted up.

## Validation

The probe hangs one finger tip-down from a fixed carriage and pushes a Ø25 mm cylinder (its axis along the finger's
width) into the contact face at 5 mm/s, at a chosen height below the base. The same press is solved for a reference
of 2 mm beam elements of the same walls, geometrically exact (`frame_nl`), and for the linkage quasi-statically on the
CPU (`prb_cpu`):

```bash
for z in 45 60 75 90 105; do python -m force_bench.finray.probe --z -$z; done    # -> outputs/finray/probe_prb_z<z>/
python -m force_bench.finray.report                     # -> outputs/finray/probe.png, and prints the table below
```

`probe` also takes `--depth` (mm past first touch, default 20). Each press writes `rollout.mp4`,
`rollout_states.npz` and `probe.json`. It runs at the tasks' 32 substeps per 60 Hz frame and takes about 1.5 min on
an RTX 5090. `report` computes the beam reference the first time (about 2 min on a CPU) and reuses it from
`outputs/finray/nl_reference.json`.

![probe](media/finray/probe.png)

| press height (below the base) | force at 10 / 20 mm, sim | reference | tip motion at 10 / 20 mm, sim | reference | contact arc at 10 / 20 mm, sim |
|---|---|---|---|---|---|
| 45 mm | 19.4 / 37.1 N | 20.0 / 37.6 N | +3.1 / −2.5 mm | +3.5 / −2.6 mm | 38° / 66° |
| 60 mm | 11.7 / 24.4 N | 11.9 / 23.2 N | +2.9 / −0.6 mm | +3.4 / −0.9 mm | 32° / 45° |
| 75 mm | 7.3 / 17.6 N | 6.8 / 16.3 N | +1.9 / −2.5 mm | +2.4 / −2.0 mm | 36° / 62° |
| 90 mm | 6.4 / 32.9 N | 6.0 / 18.0 N | +1.5 / −0.1 mm | +2.7 / +3.6 mm | 32° / 83° |
| 105 mm | 15.0 / 69.1 N | 8.2 / 22.7 N | +3.4 / +12.0 mm | +4.8 / +11.8 mm | 49° / 90° |

Tip motion is along the closing axis, **negative = toward the object** (the Fin Ray effect). Contact arc = the arc of
the cylinder within 0.3 mm of the contact wall; a flat rigid face reads 25° at the same tolerance.

1. **The linkage reproduces the reference over the upper three quarters of the finger.** From 45 to 75 mm below the
   base its force stays within 10 % of the reference and its tip motion within 0.6 mm over the whole 20 mm press,
   including where the tip reverses. In the last 30 mm it agrees up to 15 mm of indentation at 90 mm and up to 6 mm
   at 105 mm (force within 20 %), and beyond that it is 1.8 to 3 times stiffer. There the last cavities are only 5 to
   10 mm wide: a real finger's walls would touch, which neither model represents.
2. **This finger curls toward the object only under large indentation.** Pressed at mid-length (60 mm), its tip
   first moves *away* from the object (+2.9 mm at 10 mm of indentation and 12 N) and crosses back only past 19 mm and
   23 N. The 6 mm back wall and fifteen closely spaced ribs make it behave like a tapered beam; the curl needs the
   thin contact wall's chord to shorten as it bends round the object, which is second order in the deflection (a
   linear model shows no curl at all: the tip moves 0.56 mm away per mm of indentation). Pressed near the tip (90 to
   105 mm), it does not curl within the 20 mm press. The contact still spreads (the arc grows from 25° to 45° and
   more), so the finger conforms, but it does not wrap round a part at grasp-scale forces.
3. **Stiffness** against the Ø25 mm cylinder, force over indentation at 10 mm (the reference): 2.0, 1.2, 0.7, 0.6
   and 0.8 N/mm at 45, 60, 75, 90 and 105 mm below the base (E = 26 MPa). Pinning the lug hole stiffens the upper half
   2 to 4 times ([the lug hole](../assets/finray/README.md#the-lug-hole)).

## Retrieve single utensil from bin (task 2)

Task 2 is a scene without a scripted policy ([tasks.md](tasks.md#2-retrieve-single-utensil-from-bin)): its bin is
filled by physics and handed over with the arm waiting above it. `--embodiment finray` mounts the Fin Ray fingers on
it, in every arrangement:

```bash
force-bench teleop-ui --scene retrieve_utensil_from_bin --embodiment finray            # drive it by hand
force-bench run retrieve_utensil_from_bin --embodiment finray --seed 1                 # record the filled scene
```

A policy of your own gets the scene from `Task(..., embodiment="finray").setup()` (in
`force_bench.appendix_tasks.retrieve_utensil_from_bin`). The fingertips are 100.8 mm beyond the TCP, so the hand
stays about 100 mm above the utensils while the fingers reach into the bin; the finger's flat contact face ends
0.53 mm above its lowest point, so a pinch holds best with that end below the part's axis. The scene's probes
`padL` and `padR` read the contact force on each finger's links, and the task's fingers object, `Task.fingers`,
evaluates the deformation gate below on a recorded episode with `checks(states, windows)`.

## The deformation gate

`force_bench.finray.deform` measures, per finger and per frame: the largest rotation of any linkage joint from the
printed shape (hinge), the same over the contact wall's joints (wall), the tip's displacement from its printed
position in the carriage frame (tip), and the tilt of the lowest contact link (face tilt). The verdict check
`finray_deformation` takes their maxima over both fingers in two windows:

- **free** (fingers open: before the closing, and from the retreat on): tip ≤ 3 mm and no joint beyond 8°. With the
  fingers open, only the hand's own motion bends the finger (about 0.9 mm and 2.4° while the hand accelerates), so a
  larger deflection means the finger is being pushed into something (3 mm at the tip takes 6 N or more). 8° at one
  joint is a 1 mm dent across 7 mm, a local kink the tip may not show.
- **grasp** (closed on the part, from the end of the closing to the release): tip ≤ 5 mm. The finger is meant to
  bend round the part; 5 mm at the tip takes about 9 to 11 N, below the 15 N limit of the finger drive.

The same measures for any recorded run, as maxima per time window:

```bash
python -m force_bench.finray.deform outputs/appendix_tasks/<run>/rollout_states.npz 0,2,5   # windows 0-2 s, 2-5 s
```

## Code

| module of `force_bench.finray` | what |
|---|---|
| `geometry` | the finger's measured profile (walls, ribs; cached in `assets/finray/finray_profile.json`) and the adapter's frames, shared with the CAD |
| `frame_fe` | a linear 2-D frame (beam) model of the profile |
| `frame_nl` | the same walls, geometrically exact, pressed by a cylinder: the stiffness reference |
| `prb`, `prb_cpu` | the pseudo-rigid-body linkage, and its quasi-static solution on the CPU |
| `model` | the Newton models (the linkage, or the finger as one rigid mesh) on the FR3's carriages, and on a fixed mount for the probe; `FinRayParams` holds the material and solver parameters |
| `embodiment` | the `finray` embodiment's fingers class: mount, probes, envelope, deformation gate ([embodiments.md](embodiments.md#how-an-embodiment-is-integrated)) |
| `deform` | how much the simulated fingers bend, from recorded body poses |
| `probe`, `report` | the cylinder press in the simulator, and its figure against the references |

## Limits

- The modulus is a data-sheet estimate, not fitted to a printed finger. The probe is the place to calibrate it:
  press a printed finger with the same Ø25 mm cylinder and scale `FinRayParams.E_mpa` (`force_bench.finray.model`).
- The linkage bends only in the finger's profile plane. It is rigid across the 25.8 mm width and in torsion, and its
  faces are rigid (no compliance of the TPU skin), so it cannot conform to an edge or a corner locally.
- The walls cannot touch each other (no self-contact). In the last 30 mm the cavities are only 5 to 10 mm wide, so a
  real finger's walls would meet there under a deep press; the model has neither that contact nor the softness
  before it.
- The linkage ends 1.0 mm above the printed tip, and its flat contact face 0.53 mm above its own lowest point.
- The links are light, so MuJoCo needs `impratio` 10 for the finger's friction; a task that supports the finger runs
  its whole scene with it.
- One task supports the finger: task 2's scene, which has no scripted policy. Adapting a task's scripted policy to the
  finger is described in [embodiments.md](embodiments.md#adding-an-embodiment).
- The finger costs run time: its 142 bodies, 142 joints and 32 welds make a scene about 1.5 times as slow as with the
  stock fingers.
- MuJoCo warns that the joint order is not depth-first: the linkages' joints come after the hand's two finger
  joints. The simulated press agrees with the quasi-static CPU solution of the same linkage (the figure above).
