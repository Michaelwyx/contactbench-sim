"""Mechanism builders for kit assets whose printed springs, gears and latches are modelled as joints.

One module per model named in sim.yaml (``model: outlet`` -> ``force_bench.kit.mechanisms.outlet``), with

``build(rig, asset, handle, **params)``
    add the asset's bodies, shapes and joints at ``handle.pose`` for the variant ``handle.choice``, and fill
    ``handle.bodies / joints / shapes / params / info``;
``add_probe(rig, handle)``  (optional)
    add what the mechanism's check needs before the model is finalised (a pusher, a torque source);
``run_probe(rig, handle) -> dict``  (optional)
    after finalisation: exercise the mechanism and return its measurements with a ``pass`` verdict.

The model of each mechanism, and what its variants change, is documented in its module.
"""

from __future__ import annotations

import importlib


def module_for(asset):
    return importlib.import_module(f"force_bench.kit.mechanisms.{asset.sim['model']}")
