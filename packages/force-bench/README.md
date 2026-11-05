# force-bench

The simulation package of ContactBench 1.0 (import name `force_bench`): the benchmark tasks, the simulated
3D-printable assets, scripted policies with automatic success checks, rendering, teleoperation and
demonstration recording, on NVIDIA Newton with the MuJoCo Warp solver.

It is developed and documented as part of the repository it lives in; install it from a checkout so that it
finds the asset meshes at the repository root:

```bash
pip install -e packages/hydrogel -e "packages/force-bench[hydrogel,teleop]"
force-bench list
force-bench run plug_in_usb_2
```

See the repository's [README](../../README.md) and [docs/](../../docs/): installation, getting started, the task
catalogue, the assets, tactile sensing, teleoperation, data recording and the developer guide.
