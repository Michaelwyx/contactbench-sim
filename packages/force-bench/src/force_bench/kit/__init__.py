"""The ContactBench kit, sim-ready: every printable asset of the project website, loadable into a scene.

    from force_bench.kit import load_kit, add_asset, place_on_table

    kit = load_kit()                           # {key: Asset}; kit["outlet"].groups, .parts(choice)
    h = add_asset(rig, "outlet", "0.7mm indent spring", pose=place_on_table("outlet", xy=(0.45, 0.0)))

Command line: ``force-bench kit list | info <key> | check [keys] | import | export-usd [keys]``.
Data: ``assets/kit/catalog.json`` (transcribed from the website) and ``assets/kit/sim.yaml`` (the physical model);
see docs/assets.md.

As USD, for scenes of your own (docs/custom_scenes.md)::

    from force_bench.kit import export_variant, add_asset_usd
    export_variant("outlet", "0.7mm", "outlet.usda")           # = force-bench kit export-usd outlet
    h = add_asset_usd(builder, "outlet.usda", xform=...)       # any newton.ModelBuilder (or a Rig)
"""

from force_bench.kit.catalog import Asset, Group, Kit, Part, Variant, load_kit, print_mass  # noqa: F401
from force_bench.kit.build import (AssetHandle, add_asset, add_rigid, place_on_table, pose_matrix,  # noqa: F401
                                   to_wp)
from force_bench.kit.usd import UsdAssetHandle, add_asset_usd, export_variant  # noqa: F401
