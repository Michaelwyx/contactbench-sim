# force_bench.appendix_tasks

The ContactBench 1.0 tasks in simulation: one module per task (or per object variant), listed in `registry.py`. Each
builds its scene, runs a scripted policy with the FR3 arm and checks the paper's success criterion; the verdict is
written to `appendix_qa.json`. Task 2 (`retrieve_utensil_from_bin`) is a scene without a scripted policy: its run
records the filled bin with the arm holding still (`scene.json`).

```bash
force-bench list                 # the rows, by paper task number
force-bench info <row>           # skills, start state, the scene it builds on, documented differences
force-bench run <row>            # one row -> outputs/appendix_tasks/<row>/
force-bench run-all -j 3         # several rows + outputs/appendix_tasks/summary.json
```

Every task with its simulated setup, its checks and its differences from the real task is described in
[docs/tasks.md](../../../../../docs/tasks.md); how a task module is put together, and how to add one, in
[docs/developing.md](../../../../../docs/developing.md).

Files:

- `registry.py`: the rows (paper title and number, skills, start state, notes, arguments) and the paper's success
  criteria, verbatim.
- `common.py`: `appendix_rig`, `RobotContactProbe`, retreat geometry, `check_released_and_clear`, the verdict writer.
- `derive.py`: the `Variant` harness that builds a task on a scene module of `force_bench.tasks` without editing
  it (cut, override or extend its control, add checks).
- `run_all.py`: runs rows in separate processes and collects their verdicts.
- `_*.py`: pieces shared by several rows (the hanging-board cell and its finger model, the kit door, the Allen key
  and screwdriver tools, the threads of the kit's bottles and bulbs, ...).
- the other modules: one per task; tasks built on the final kit take their objects from `force_bench.kit`.
