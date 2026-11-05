# GelSight simulation: method and parameters

This document describes the simulated GelSight Mini (`packages/hydrogel`, mounted on the tasks by
`force_bench.tactile`): how the contact is modelled, how the sensor fields are computed and rendered, what each step
costs, and the physical meaning of every tunable parameter. How to run the sensor and what it measures on the tasks
is in [tactile.md](tactile.md); the validation experiments and videos are produced by `hydrogel validate` under
`outputs/gelsight/validate/`. Timings are for an NVIDIA RTX 5090.

## 0. Architecture

```
Newton physics (SolverMuJoCo + hydroelastic SDF contacts)
   │  per substep: CollisionPipeline.collide → iso-pressure contact surface
   │               (triangles + depths) + reduced solver contacts
   │  per substep: solver.step  (per-contact secant stiffness → force-space
   │               solref at impedance 0.5)
   ▼
Sensor (hydrogel.sensor, once per frame, GPU)
   │  contact-surface triangles → pressure / normal-displacement / shear /
   │  contact-mask rasters (96×128, FOV only)
   │  integral scaled to the solver's realized normal force
   │  (solver-consistent), pressure saturation spread
   ▼
Render (hydrogel.render, GPU)
   │  pinhole-perspective raycast height map → membrane skirt → bulge ring
   │  → Taxim polynomial shading → markers → displacement arrows
   ▼
tactile_{left,right}.npz  +  tactile_strip.mp4 / rollout_panel_tactile.mp4
```

Three principles run through the design:

1. **The solver is ground truth**: the sensor's readings agree with the force the solver actually applied, not with an
   analytic model.
2. **Every effect is independently verifiable**: `hydrogel validate` has commanded-against-measured experiments, with
   analytic references, for each channel.
3. **Every approximation has a parameter and a sweep video**, so the visual consequence of each parameter can be seen
   directly.

## 1. Contact modelling (normal)

**Model.** Newton's hydroelastic SDF contact (the pressure-field model of the Elandt / Drake lineage). Both shapes
carry an SDF and a pressure field p = f(signed depth); the contact surface is the iso-surface where the two fields
are equal (extracted by GPU marching cubes), each triangle carrying its area and depth; force = Σ p·A.

**Gel pressure law** (`hydrogel.pressure`, a Newton `pressure_func` callback):

    p(x) = gel_kh · x · (1 + a·(x/t)²)      x = indentation, t = gel thickness

The linear term is the Winkler (elastic-foundation) law p = (E_eff/t)·x; the cubic stiffening term a expresses the
steep rise of a thin, nearly incompressible confined layer approaching full compression. Rigid objects stay linear at
kh = 1e10, so the iso-surface hugs the object's surface and the exported depth is the gel's true indentation.

**Two conditions for the solver to follow this law:**

- **Pair ordering.** Newton exports the depth of a pair's second shape, and pairs are ordered by type (box < mesh).
  The gel must be shape B, so it is built as a mesh and mounted after the objects; otherwise the solver sees only the
  rigid side's micrometre depth and the gel behaves as a wall.
- **MuJoCo impedance.** Newton's default per-contact `solref` has no inverse-weight scaling and an impedance of 0.95,
  so the constraint is 95 % rigid and the realized compliance is set by mass and iterations, independent of kh.
  `hydrogel.mujoco_force_space` replaces the contact-conversion kernel at run time with
  k = c·invw·(1−d)/d, tc = 1/√(d·k), d = 0.5, so the rest solution is exactly F = c·r (c = the law's secant
  stiffness). A driven indenter's realized indentation-force curve then follows the law to within a factor of 1.16 to
  1.30, independent of the indenter's inertia.

**Analytic reference** (`hydrogel validate`): sphere F = ∫p·2πr dr (stiffening integrated numerically), flat face
F = p·A. Sphere and box agree within 3.5 % over the whole depth range.

## 2. Sensor computation (rasterization)

Once per frame (at the frame's end, from the last substep's contact surface), a GPU kernel
(`hydrogel.sensor_kernels.splat_faces`) processes every contact-surface triangle:

1. pressure p = law(depth, side-B shape); force = p·A·|n·ẑ| (the normal projection discards the lateral load of
   side-wall faces);
2. the face is transformed to the gel frame and splatted bilinearly into the rows × cols force grid; **faces whose
   centroid lies outside the FOV are not splatted**: the physical pad (25.25 × 20.75 mm) is larger than the imaged FOV
   (18.6 × 14.3 mm), and off-image contact carries load but is not seen, as on the hardware;
3. the force-weighted per-cell tangential velocity of the object relative to the gel (§3) and the totals (force,
   centre of pressure, face count) are accumulated in the same pass.

**Area splat.** Newton's marching cubes tessellates the contact surface on the coarser SDF's voxel lattice (0.2 mm on
the gel, against 0.145 mm raster cells). Splatting each face's force at its centroid alone would make cells alternate
between one and two centroids, a fixed plaid of ±7 to 17 % on flat presses. `splat_faces` therefore splits every face
into s² equal-area sub-triangles (s from the longest edge in cell units, at most `GS_SPLAT_MAX_LEVEL` = 6) and splats
each sub-centroid with f/s². The raster is then the area integral of the face pressure; force, centre of pressure and
velocity sums are unchanged, and the cost is the same. The ripple that remains on the key-bow press (3.1 % std) is
surface detail: it is unchanged at 4× denser sampling.

**Solver-consistent scaling.** The pressure raster's integral is scaled to the solver's realized normal force on that
pad (`contacts.force` after `update_contacts`, projected on the gel normal, summed). The spatial shape comes from the
pressure field and the magnitude from the solver, so the image shows the force the robot feels. The unscaled law
integral is kept as a diagnostic channel (`gel_*_law_N`, `solver_over_law`).

**Saturation spread.** Scaling can push a small patch beyond physical pressure. Any cell above the full-compression
pressure p_sat = law(t) is clamped and the excess force pushed into the neighbouring ring, iteratively: a bottoming
contact grows its imprint area, as a real gel does against its backing.

## 3. Shear, stick, slip and rotation

**State.** A 2-D surface shear displacement u(x, y) per cell (gel frame, metres), integrated over time:

    in contact:   u += v_tang · dt     (v_tang = force-weighted PER-CELL
                                        object-minus-gel tangential velocity)
    then clamped: |u| ≤ min( μ·p / k_shear , shear_cap )   ← Coulomb stick bound
    off contact:  u ← u · exp(−dt/τ_release)               ← gel-memory relaxation

Physical reading: k_shear = G/t is the gel layer's shear foundation stiffness. A surface stuck to the object and
displaced by u produces the shear traction τ = k_shear·u; when τ reaches μ·p the surface slips and u stops growing.
This yields **Cattaneo–Mindlin partial slip**: the low-pressure rim slips first, the high-pressure centre stays stuck.
The shear-traction channel is k_shear·u, and a verdict check (`gel_coulomb_bounded`) confirms it stays ≤ μ·p
everywhere.

**Rotation** needs no separate model: the per-cell velocity field of a twist about the normal is the vortex
v = ω×r, which integrates into a rotational shear field; only the rim beyond the stick bound slips. This is why the
state is kept per cell: integrating one pad-mean velocity would average the antisymmetric rotation field to zero, and
rotation would read as pure slip.

**Simplification.** After slipping, u stays clamped at the μ_s·p/k level (effectively μ_s = μ_k); real kinetic
friction is lower.

## 4. Marker modelling

Marker displacement is the sum of **a mechanical term and a camera-perspective term**, with distinct physical
origins:

1. **Field-following** (`GS_MARKERS=field`, the default): inside the contact a marker sticks to the surface, so its
   displacement is the local shear field u, sampled bilinearly at the marker; outside the contact a 2 mm Gaussian
   membrane blur drags neighbours along, decaying with distance. A twist produces a marker vortex; a sliding contact
   moves only the markers under it. Alternatives: `hydroshear` (the HydroShear paper's two-Gaussian kernel sums,
   which give an order of magnitude less marker motion; kept for comparison) and `local` (plain bilinear advection).
2. **Pinhole projection**: the camera sits 24 mm behind the gel; a surface pressed toward the camera by h shifts a
   marker's projection radially outward by a factor d/(d−h). This is the *apparent dilation* a real sensor shows under
   a pure press.

Every video overlays **displacement arrows** (rest → current, amplified by `GS_MARKER_AMP` = 2×) by default, because
uniform shifts and slow vortices are invisible as dots alone.

## 5. Rendering

**Marker texture warping** (`GS_MARKER_RENDER=warp`, the default). Markers are not rigid disks translated by the
displacement sampled at their centres: the rest dot lattice is inverse-warped per pixel, M(x) = M0(x − u(x)), with u
the membrane-blurred shear field (the same physics as the field marker model, evaluated per pixel) plus the per-pixel
pinhole term (q = p − (p − ctr)·h/d). A dot straddling the indentation border therefore bends and stretches with the
displacement gradient inside it, and the crease a real sensor shows (half the dot on the indented surface, half on the
flat) emerges by itself. `stamp` draws rigid disks instead; the arrow overlay reads the centre displacements in both
cases (the data is the same).

1. **Height map**: pinhole raycast (one ray per pixel from the camera through the undeformed face plane, against the
   object mesh at the solver's realized pose) → max(geometry, law⁻¹(scaled pressure)); the second term covers load
   the geometry misses.
2. **Membrane skirt**: max(h, blur_σ_skirt(h)), the draped skirt around a contact.
3. **Bulge ring** (off by default; enabled with `GS_BULGE` > 0): h −= k_bulge·max(blur_σ_bulge(h) − h, 0), the
   volume-conserving raised rim. The Winkler law has no lateral coupling, so this membrane effect is added back in the
   rendered height only, with no effect on the dynamics.
4. **Taxim shading**: h → image-space gradients → (magnitude, direction) bins into the real GS Mini calibration
   polynomial table → RGB, with a flat-field correction (the zero-gradient bin subtracted) so an untouched gel renders
   the background exactly; `gain` scales the calibrated response's contrast.
5. **Markers and arrows** (§4). The pressure tile uses an inferno colour map scaled at the video's 99.5th percentile,
   so one saturated frame does not wash out the whole strip.

## 6. Cost

| stage | cost | how |
|---|---|---|
| sensor sampling | 2 ms per frame | Warp splat kernel; the contact surface stays on the GPU |
| tactile strip, 2 pads | 6 ms per frame (164 fps) | height map, shading and marker positions on the GPU; panel resized with PIL |
| strip and panel of a 444-frame rollout | 7.4 s (60 fps end to end) | including mp4 encoding |
| physics frame, 32 substeps | about 170 ms | `GS_GRAPH`, `GS_COLLIDE_EVERY` and `TB_SUBSTEPS` make it up to 2.2× faster, with the side effects listed under [Performance](#performance) |

NumPy reference paths exist for both GPU stages (`GS_SENSOR_GPU=0`, `GS_RENDER_GPU=0`); the unit tests check that the
sensor's two paths agree (pressure raster to 0.1 %, total force to 0.01 %).

## 7. Sensor asset and mounting (`GS_MOUNT=case`, the default)

Following the three-body sensor structure of TacEx and UniVTAC and the official datasheet: a rigid case of
32 × 28 × 24 mm (collides; adds 40 g to the finger; top edge flush with the fingertip); the gel-cartridge dome stands
4 mm proud of the case rim; the physical gel pad is 25.25 × 20.75 × 4.25 mm (larger than the FOV). The case rim is
recessed 4 mm behind the sensing face, so normal grasps touch only the gel and a bottoming contact lands on the case.
`pad_proud_m()` (5 mm) is the single source of every finger-joint offset; tasks take it through `GS.finger_offset()`.

**Orientation.** The sensor's long edge runs along the finger (the 32 mm case edge and the gel's 18.6 mm image axis
point toward the wrist) and its short edge across it, so the case is 28 mm across × 32 mm along the finger with its
top edge flush with the fingertip, and the FOV centre sits 16.5 mm from the tip. `GS_PAD_LONG_AXIS=across` turns the
whole sensor 90° about the grasp normal (32 mm across the finger) for comparison; the gel frame is unchanged (gel x =
image columns = long edge).

**Appearance** (optional). Renders can use GelSight Inc.'s official housing CAD (`gsmini_shell.stp`, converted with
cascadio to `packages/hydrogel/src/hydrogel/data/gs_mini/gsmini_shell.obj`, 32.0 × 28.25 × 28.89 mm). It is
GPL-3.0 licensed and therefore not shipped; without it the parametric case box is rendered. `hydrogel housing
--download` installs it locally ([tactile.md](tactile.md#the-sensor-housing-gelsights-cad-installed-locally)). The
CAD has its elastomer at its +y end, so it is turned 180° about its own centre before mounting (housing face toward
the grasp), and its elastomer is dropped from the visual: the physics gel slab, which has the same footprint, takes
its place. The housing is a visual-only shape: collision and mass stay on the parametric case box
(`is_visible=False`). `GS_SHELL_VISUAL=0` restores the box look.

**Field of view and grasp point.** The FOV centre sits well behind the fingertip, so a grasp with the finger's last
millimetres images little of the contact (measured from the centre of pressure in gel coordinates).
`GS_CASE_TIP_EXTEND=<mm>` lets the case pass the fingertip by that much, which moves the FOV toward the contact at the
cost of clearance: the housing tip can then ride on the part, as real hardware would. Where the housing cannot extend
(a key pinched next to a door face), the grasp point has to move onto a part of the object that the FOV covers.

## 8. Validation methodology

- `hydrogel validate`: {press ramp, depth staircase, two-direction shear, rotation} × {sphere, box, rod, key bow},
  videos with commanded and measured values in a side panel; `--sweep <param>` renders the same trajectory side by
  side at three parameter values.
- `hydrogel calibrate`: `--sweep` presses the simulated gel at a range of depths and reports F(d), the implied
  stiffness and its ratio to the analytic law (the discretization factor of the SDF and raster); `--fit presses.csv`
  fits `gel_kh` to real ball presses (columns `force_n`, `depth_m`, optional `radius_m`), and with `--sim-factor`
  from a sweep gives the value that makes the simulated sensor reproduce the real curve.
- `force_bench.tactile.press_probe`: contact-model audit (the sensor's force against the force Newton's hydroelastic
  contact asks for, per object, with no dynamics) and, in `solver_press`, a driven-indenter audit (the solver's
  realized compliance against the law).
- `hydrogel grip-rig`: a gel pad on each jaw of a parallel gripper grasps, lifts, shakes, holds and releases a free
  sphere; five asserted checks.
- `force_bench.tactile.grip_task`: the same on the FR3 with a cube: both pads loaded through the hold, the cube rides
  the gripper, shear active and within the Coulomb bound, the solver's finger force and the sensor integral within a
  factor of 2 during the hold, a clean release.
- `packages/hydrogel/tests/test_sensor.py`: unit tests of the sensor, including the agreement of its CPU and GPU
  paths.
- Verdict checks in the tasks: `gel_pads_loaded`, `gel_coulomb_bounded`, `gel_probe_crosscheck` (the solver's
  finger force against the sensor integral, ratio in [0.5, 2]: a detector of a sensor decoupled from the contact)
  ([tactile.md](tactile.md#4-what-the-tactile-variants-measure)).

## 9. Known limits

- The CUDA-graph capture of the substep loop (`GS_GRAPH=1`, 2.2× faster) kicks the object at release, so it is off by
  default.
- Colliding less often than every substep (`GS_COLLIDE_EVERY` = 2 or 4) is faster, but free-fall landings produce
  force spikes; it is off by default.
- Kinetic friction is not separated from static friction (μ_s = μ_k).
- The gel bottoms out on hard, small features (a key bow, nut flats): the indentation reaches the gel thickness and
  the rigid backing takes over. The forces stay correct; the gel law no longer describes the contact there.

---

# Parameter table

Bold = default. "Validation tool" points at the sweep video or experiment that shows the parameter's effect.

## Contact & material (`GelSightConfig` / task layer)

The values are those the tasks use: the configuration in `force_bench/tactile/task.py`, built from
`hydrogel.sensor.GelSightConfig`. The class's own defaults differ for `rows` / `cols` (32 × 42),
`gel_kh_pa_per_m` (2e8), `friction_coefficient` (0.8) and `gel_stiffening` (0).

| parameter | default | physical meaning | effect of raising / lowering | validation tool |
|---|---|---|---|---|
| `gel_kh_pa_per_m` | **4e7** | gel normal foundation stiffness E_eff/t (Pa/m); E ≈ 0.16 MPa over 4 mm, consistent with the solver's 7 N ↔ 0.8 mm equilibrium | higher → shallower indentation at the same force, smaller imprint; lower → deeper, bottoms out sooner | `--sweep gel_kh`; sphere F = πkRd² analytic; `hydrogel calibrate --fit` against real ball presses |
| `gel_stiffening` (`GS_GEL_STIFFEN`) | **10** (task layer) | stiffening a in p = kh·x(1+a(x/t)²): the steep rise of a thin confined layer near full compression | higher → force climbs steeply at depth, resists bottoming; 0 = pure Winkler | `--sweep stiffen` |
| `rigid_kh_pa_per_m` | **1e10** | rigid-side pressure-law stiffness; keeps the iso-surface on the object's skin | rarely changed | — |
| `friction_coefficient` (`GS_GEL_MU`) | **2.0** (task layer) | gel–object friction (TacSL's calibrated value; real gel is tacky) | higher → higher stick bound, firmer grasp, more storable marker shear; lower → slips | shear/rotation saturation = μp/k |
| `shear_stiffness_pa_per_m` | **3.0e7** | shear foundation stiffness G/t; G ≈ 0.12 MPa over 4 mm; sets how much displacement builds the Coulomb traction | higher → small marker motion and early slip; lower → larger motion | `--sweep shear_k`; rotation vortex magnitude |
| `shear_cap_m` | **2 mm** | hard cap on stored surface shear (shear strain ≈ 0.5); guards against unphysical stick bounds at saturated pressure | lower → truncates large twists | rotate experiments |
| `shear_release_time_s` | **25 ms** | relaxation of the shear memory after contact loss (viscoelasticity) | higher → longer after-image | shear return stroke |
| `gel_thickness_m` | **4 mm** | gel thickness (official 4.25 ± 0.2 mm); enters the law, saturation and shear strain | — | — |
| `width_m` / `height_m` | **18.6 / 14.3 mm** | imaged FOV (what the camera sees), smaller than the physical pad | — | — |
| `rows` / `cols` | **96 × 128** | sensor raster resolution (the render upsamples 2.5× to 320×240) | higher → finer texture, slightly slower | — |
| `sdf_max_resolution` | **96** | primitive-object SDF resolution; mesh objects build theirs in `make_newton_mesh` | must resolve the smallest tactile feature (task 5's card uses 512) | — |
| `contact_gap_m` | **0.3 mm** | contact detection margin | — | — |

## Solver (making the physics follow the law)

| parameter | default | meaning | note |
|---|---|---|---|
| `GS_FORCE_SPACE` | **1** | per-contact secant stiffness through a force-space `solref` into MuJoCo (§1); 0 = Newton's default, whose compliance follows inertia and ignores kh | required for the gel law to hold |
| `GS_HYDRO_IMP` | **0.5** | MuJoCo impedance d for hydroelastic contacts; the rest solution F = c·r holds for any d, d sets settling speed and regularization | 0.95 behaves as a rigid wall |
| `GS_GEL_MESH` | **1** | gel built as a mesh shape, which guarantees it is the pair's side B, so the true indentation is exported (§1) | 0 = a box gel, which acts as a wall |
| `TB_MJ_IMPRATIO` / `TB_COLLIDE_SUBSTEP` | 10 / 1 | the scene-level solver settings (friction-cone regularization, collision every substep) | set automatically by `TB_GELSIGHT=1`; an explicit value takes precedence |

## Mounting (GS Mini asset)

| parameter | default | meaning |
|---|---|---|
| `GS_MOUNT` | **case** | case + gel asset mount; any other value (e.g. `slab`) mounts a bare gel slab without the case |
| `CASE_*` (constants in `force_bench/tactile/rig.py`) | 32×28×24 mm, 40 g | aluminium case dimensions and mass (official datasheet) |
| `GEL_PAD_W` / `GEL_PAD_H` | 25.25 / 20.75 mm | physical gel pad (larger than the FOV) |
| `GEL_PROUD_OF_CASE` | 4 mm | cartridge dome proud of the case rim (the rim is the bottoming backstop) |
| `pad_proud_m()` | 5 mm | sensing face proud of the bare fingertip: the single source of the finger-joint offsets |
| `GS_PAD_LONG_AXIS` | **finger** | the sensor's long edge along the finger; `across` turns it 90° about the grasp normal |
| `GS_MARKER_RENDER` | **warp** | marker rendering: `warp` = texture inverse-warp (edge creases emerge); `stamp` = rigid disks |
| `GS_CASE_TIP_EXTEND` | **0** mm | how far the case may pass the fingertip: gains imaging of the tip zone, costs clearance to the table and obstacles |
| `GS_SHELL_VISUAL` | **1** | render the official GS Mini housing CAD if it has been installed locally (visual only; 0 or a missing file = the parametric box) |
| `GS_GEL_SDF_RES` | **128** | gel mesh SDF resolution (sets the contact-surface tessellation; the area splat keeps the raster independent of it) |
| `GS_SPLAT_MAX_LEVEL` | **6** | area splat: each contact face is split into at most s² equal-area sub-triangles before rasterization (§2); 1 = centroid splat, which shows the plaid |

## Rendering & markers

| parameter | default | physical meaning | effect | validation tool |
|---|---|---|---|---|
| `GS_RENDER_GAIN` | **2.0** | contrast of the calibrated response (scaled about the background); the vendored table reads soft | higher → vivid imprints, amplified flat-field residue | `--sweep contrast` |
| `BLUR_SIGMA` | **1.1 px** | pre-shading height blur (surface optical smoothing of the Taxim reference) | higher → mushy imprint | `--sweep blur` |
| `SKIRT_SIGMA_MM` | **1.2 mm** | membrane skirt width (draping around contact), a render-side membrane parameter; fit it to the radial intensity falloff around a real ball-press image | higher → wider soft halo | `--sweep skirt` |
| `GS_BULGE` / `GS_BULGE_SIGMA` | **0 (off) / 2.5 mm** | amplitude and width of the volume-conserving bulge rim (incompressible membrane effect) | higher → brighter raised ring | `--sweep bulge` |
| `GS_CAM_PERSP` (`CAM_STANDOFF_M`) | **1** (24 mm) | pinhole camera model: perspective height map + apparent marker dilation d/(d−h) | off → orthographic, no marker motion under a pure press | press experiments |
| `GS_MARKERS` | **field** | marker mechanics: `field` (stick physics) / `hydroshear` kernels / `local` advection | field → rotation vortex, local following | rotate/shear experiments |
| membrane σ (inside the field model) | 2 mm | decay length over which the membrane drags out-of-contact markers | — | — |
| `dilation_gain` / `shear_gain` / `λ_d` / `λ_s` | 1 / 1 / 20000 / 10800 | HydroShear kernel parameters (only with `GS_MARKERS=hydroshear`) | — | `--sweep marker_gain`, `--sweep lambda_d` |
| `GS_MARKER_ARROWS` / `GS_MARKER_AMP` | **1 / 2×** | displacement-arrow overlay (visualization, not physics) | — | every video |
| `GS_RENDER_HEIGHT` | **max** | height = max(geometric raycast, law⁻¹(pressure)); `geom` = raycast only | — | — |
| `PIXMM` | 0.065 mm/px | GS Mini pixel pitch (calibration constant) | — | — |

## Performance

| parameter | default | meaning |
|---|---|---|
| `GS_SENSOR_GPU` / `GS_RENDER_GPU` | **1 / 1** | GPU splat / render; 0 = NumPy reference paths |
| `GS_COLLIDE_EVERY` | **1** | collide every K substeps (2 or 4 is faster, but free-fall landings spike) |
| `GS_GRAPH` | **0** | capture the substep loop in a CUDA graph (2.2× faster, but kicks the object at release) |
| `TB_SUBSTEPS` | 32 | solver substeps per 60 Hz frame (every task's default) |
| `GS_PROFILE` | 0 | print per-stage timings every 60 frames |
| `TB_GELSIGHT_RENDER` | 1 | render the tactile strip and panel after the task |
| `GS_VIDEO_SCALE` | 2 | integer upscale of the `hydrogel validate` videos |
