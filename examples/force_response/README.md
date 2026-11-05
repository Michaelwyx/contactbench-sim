# Force response of the simulated mechanisms

The printed springs of the kit are not simulated as deforming bodies. Each one is a joint drive or a sprung contact
body whose stiffness and preload are explicit parameters (see [docs/assets.md](../../docs/assets.md)). These two
scripts push a mechanism in a straight line at constant speed, the way a robot with a wrist force/torque sensor
pushes the printed part on a bench, and log force against displacement to a CSV file. Use them to see what each
parameter does, or to fit the parameters to measurements of your own prints.

Both run without a robot and without gravity, and take about a minute each on a recent GPU (the first run also
compiles kernels).

## Door handle (paper Fig. 7)

`probe_door_handle.py` uses the kit's doorknob module with handle 1, lying alone with its spindle vertical; the
return spring acts on the spindle joint. A 5 mm sphere pushes the lever along a straight line 85 mm from the
spindle, 62 mm at 10 mm/s. The return spring is modelled as

    tau(theta) = tau0 - K * theta        (+ optional joint dry friction)

| option | meaning | default |
|---|---|---|
| `--ke` | spring stiffness K [N·m/rad] | 0.30 |
| `--tau0` | preload torque tau0 [N·m], holding the lever on its rest stop | 0 |
| `--fric` | joint dry friction [N·m] | 0 |
| `--kd` | joint damping [N·m·s/rad] | 0.01 |
| `--out` | output CSV: `t, push_mm, force_N, lever_deg` | required |

```bash
python examples/force_response/probe_door_handle.py --ke 0.48 --tau0 0.15 --out door_5mm.csv
```

Handles printed on a Bambu Lab P1S with 5, 7.5 and 10 mm return springs were pushed with a UR arm and a wrist F/T
sensor in the same way. These values reproduce the measured force profiles:

| printed spring | K [N·m/rad] | tau0 [N·m] |
|---|---|---|
| 5 mm | 0.48 | 0.15 |
| 7.5 mm | 0.67 | 0.25 |
| 10 mm | 0.78 | 0.31 |

The kit's doorknob module uses these values for its three spring variants
(`force_bench.kit.mechanisms.doorknob_module.SPRINGS`), so the door tasks 28 and 29 run with the measured springs;
`force-bench kit check doorknob-module` probes them on the module.

Quasi-statically the push force is linear in the two parameters, F(d) = tau0·A(d) + K·B(d), so two or three probe
runs are enough to fit them to a new measured curve by least squares. The preload is a constant joint torque
(`Control.joint_f`), not a drive target beyond the joint stop: a target beyond the stop flings the lever when contact
begins.

## Charger outlet

`probe_charger.py` uses the outlet of task 22's `plug_in_charger` row (`force_bench.tasks.plug_insert_arm`): the
outlet shell and a sprung retention pad in each blade slot, which replaces the printed contact spring. Charging
block 3 is driven straight down into it at 10 mm/s, from 4 mm above the outlet's face to just short of its seat.

| option | meaning | default |
|---|---|---|
| `--prot` | pad protrusion into the slot [mm]; the first ~0.4 mm only takes up the blade's clearance, so the interference with the blade is `prot - 0.4` | 1.8 |
| `--ke` | pad spring rate [N/m] | 3000 |
| `--mu` | pad/plug friction coefficient | 0.4 |
| `--flare` | pad entry bevel angle [deg] | 30 |
| `--profile wedge\|smooth` | flat wedge entry (as in `plug_in_charger`) or a curved entry tangent to the grip face | `wedge` |
| `--bevel-mm` | length of the curved entry [mm] | `prot / tan(flare)` |
| `--kd` | pad damping [N·s/m] | 15 |
| `--drive-ke` | stiffness of the drive that pushes the block [N/m] | 60000 |
| `--frame-steps` | move the push target once per 60 Hz frame instead of every substep ([below](#a-note-on-probing)) | off |
| `--out` | output CSV: `t, push_mm, force_N` | required |

The kit outlet's four printed springs (`force_bench.kit.mechanisms.outlet`, used by the `plug_in_charger_block*`
rows) are this pad with the curved entry. Printed springs with a 0.4 mm and a 0.85 mm indent were matched to their
measured insertion force (UR arm, wrist F/T sensor, 10 trials) by an interference equal to the indent; the other two
are derived from them:

| printed spring | `--prot` [mm] | `--ke` [N/m] | source |
|---|---|---|---|
| 0.4 mm indent | 0.8 | 28000 | fitted to measurement |
| 0.7 mm indent | 1.10 | 36700 | interpolated |
| 0.85 mm indent | 1.25 | 41000 | fitted to measurement |
| 1 mm indent | 1.40 | 45300 | extrapolated |

```bash
python examples/force_response/probe_charger.py --prot 0.8 --ke 28000 --profile smooth --out outlet_0.4mm.csv
```

With the flat wedge, the force spikes at the corner between bevel and grip face (2 to 3 times the plateau); the
curved entry ramps up like the printed spring but overshoots on entry.

## A note on probing

Both scripts move the push target every physics substep rather than once per rendered frame. A target that jumps
0.17 mm once per 60 Hz frame kicks a stiff drive by several newtons every frame, which shows up as a saw-tooth in the
force and biases the mean force low. `probe_charger.py --frame-steps` shows the effect.
