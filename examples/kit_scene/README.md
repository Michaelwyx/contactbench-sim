# A scene from the kit library

`cup_stack.py` is the smallest useful scene built from the kit library (`force_bench.kit`, see
[docs/assets.md](../../docs/assets.md)): three kit cups dropped into each other on a table, without a robot. It prints
how the cups nest, next to the nesting step measured on printed cups:

```bash
python examples/kit_scene/cup_stack.py
# nesting steps [4.68, 4.68] mm (measured 4.68), off-axis [0.0, 0.0] mm
```

It writes `outputs/task/cup_stack/rollout.mp4` and `final.png`, and takes well under a minute on a recent GPU after
the first run (which compiles kernels).

The script shows the calls a scene needs:

```python
rig = Rig("cup_stack")                              # a Newton model with the MuJoCo Warp solver, camera, recorder
rig.table(half=0.3)
pose = place_on_table("cup", xy=(0.0, 0.0))         # 4x4 pose: the asset's lowest point on the table at xy
h = add_asset(rig, "cup", pose=pose, label="cup_a") # bodies, shapes and joints as sim.yaml describes the asset
rig.finalize(probes=[], cam_pos=..., cam_target=...)
rig.run(2.0, lambda t: None)                        # control callback: here nothing to control
rig.rec.save("rollout")                             # the video, state and force logs -> outputs/task/cup_stack/
```

Any other asset works the same way, with a variant where it has them:

```python
add_asset(rig, "outlet", "0.85mm", pose=place_on_table("outlet"))
add_asset(rig, "lightbulb-2", {"Lightbulb geometry": "50mm diameter", "Amount of rotation to tighten": "3 revolutions"})
```

`force-bench kit list` names every asset and `force-bench kit info <asset>` its variants. To add a robot, see
[docs/custom_scenes.md](../../docs/custom_scenes.md) or a task built on the kit, e.g.
`force_bench/appendix_tasks/stack_cups.py`.
