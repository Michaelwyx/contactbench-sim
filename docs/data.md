# Recording demonstrations

Demonstrations come from two sources and are stored in one format.

- **Scripted policies.** `force-bench collect` runs a task's scripted policy and stores every control step.
- **Teleoperation.** `force-bench teleop-ui --collect-dir <dir>` stores what a person does with a keyboard,
  gamepad or SpaceMouse ([teleoperation.md](teleoperation.md#recording)).

Each episode is a directory with an `episode.npz` (NumPy arrays) and a `metadata.json`. Reading one needs
only NumPy.

## Collect from the scripted policies

```bash
force-bench collect --task plug_in_usb_2 --episodes 5 --output outputs/datasets/usb
force-bench collect --task retrieve_card_from_deck --episodes 2 --gelsight --output outputs/datasets/card_tactile
force-bench dataset outputs/datasets/usb                     # check every episode, summarise
force-bench dataset outputs/datasets/usb --success-only
```

`--task` takes a ContactBench task with a scripted policy (`force-bench list`; task 2, a scene without one, is recorded
by teleoperation). `--gelsight` mounts the GelSight pads and records the tactile fields; it is only accepted for the
tasks validated with the sensor (`[TB_GELSIGHT=1]` in `force-bench list`). Episode `i` uses seed `--seed + i` (default
0) for Python's and NumPy's global random generators. A task that takes `--seed`
([variations.md](variations.md#random-starts)) gets it too, so each episode starts from its own random start, drawn
within the task's ranges; `--nominal-start` keeps every episode at the task's nominal start. The episode's metadata
records the start (`collection.start`, `collection.task_arguments`).

Every episode runs in its own fresh process with its outputs in a private directory, so episodes never share state or
overwrite each other. When the process ends, the task's own verdict becomes the episode's outcome: `appendix_qa.json`
(the paper's success criterion plus the physics-integrity checks, see
[tasks.md](tasks.md#how-a-run-is-judged)). A run that crashes leaves a `failed_<task>_<id>/` directory with its
log instead of an episode.

The scripted policies are deterministic up to GPU contact ordering, so episodes that start from the same state differ
only slightly. They are reference demonstrations, not a diverse dataset. For variety, collect a task that takes
`--seed`, so that every episode starts from its own random start; collect several rows of a task (its variants in
`force-bench list`); or record teleoperation, which also takes `--set` and `--seed`. A task's physical parameters are
constants in its modules ([getting_started.md](getting_started.md#change-a-physical-parameter)); `force-bench collect`
runs the modules as installed.

## Episode format

`episode.npz`, with `T` control steps at the task's frame rate (60 Hz):

| array | shape | contents |
|---|---|---|
| `observations` | `(T+1, D)` | measured robot joint positions (9: seven arm joints in rad, two finger joints in m), joint velocities (9), then the pose of every task object (`x, y, z, qx, qy, qz, qw`, world frame, metres) |
| `actions` | `(T, 9)` | the FR3 joint position targets applied between observation `t` and `t+1` (rad for the arm, m for the fingers) |
| `action_target_velocity` | `(T, 9)` | joint target velocities sent with the targets (usually zero) |
| `action_feedforward` | `(T, 9)` | feed-forward joint torques / finger forces sent with the targets (usually zero) |
| `timestamps` | `(T+1,)` | simulation time of each observation, seconds |
| `rewards`, `terminated`, `truncated` | `(T,)` | placeholders for RL tooling: rewards are zero, episodes end by truncation |
| `contact_forces` | `(T, 5)` | teleoperation episodes only: contact force magnitudes after each step (left pad, right pad, rest of the robot, manipulated object, mechanism pair), N |
| `tactile` | `(T+1, 2, S, S)` | with GelSight pads only: the pressure field of the left and right pad, area-averaged to `S × S` (16 by default), Pa |
| `tactile_valid` | `(T+1, 2)` | with GelSight pads only: whether that pad had a measurement at that step |

`metadata.json` describes everything needed to interpret and reproduce the episode:

- `observation_names`, `observation_fields` and `action_names`: the meaning and unit of every column (object
  poses are flagged `privileged`, since a real robot would have to estimate them);
- `task`, `seed`, `source` (`scripted_expert` or `teleoperation`), `fps`;
- `outcome`: `status` (`passed`, `failed` or `unvalidated`), `success`, and where the verdict came from;
- `physics`: the solver, substeps, solver options and the explicit simulation settings in effect;
- `provenance`: package versions, the git commit and whether the working tree was modified, and SHA-256
  hashes of the package sources and `assets/manifest.yaml`;
- `collection` (scripted) or `teleoperation` (teleop): how the episode was produced.

The full rollout of a scripted episode (video, `rollout_forces.csv` with every logged contact force at the
same 60 Hz frames, `rollout_states.npz`, the verdict files) is kept next to the episode under
`artifacts/`.

```python
from force_bench.data.episode import discover_episodes, load_episode

for path in discover_episodes("outputs/datasets/usb", success_only=True):
    ep = load_episode(path)                      # verifies the stored hashes
    obs, act = ep["observations"], ep["actions"]
    print(path.name, ep.metadata["outcome"]["status"], obs.shape, act.shape)
```

`force-bench dataset <dir>` loads every episode the same way, checks its arrays and hashes, and prints a
summary (episode count, outcomes, tasks, dimensions, frame rates, tactile coverage). It exits non-zero if any
episode fails validation.

## What is not recorded

- Rendered camera images are not stored per step; the episode keeps the rollout video under `artifacts/`
  (scripted) and the object poses, from which any view can be re-rendered.
- Rewards are not defined; the verdict is a per-episode outcome, not a per-step signal.
- Replaying an episode's actions open-loop does not in general reproduce it exactly: some scripted policies also
  act on the scene beyond the recorded joint targets (for example the test torque the door tasks apply at the hinge),
  and GPU contact ordering is not deterministic. The metadata says so (`replay_compatible: false`).
