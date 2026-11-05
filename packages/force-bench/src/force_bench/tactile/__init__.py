"""force_bench.tactile: the optional GelSight bridge (``pip install force-bench[hydrogel]``).

``force_bench.tactile.task`` is the ``TB_GELSIGHT=1`` opt-in layer every wired
task imports; it is the identity when the flag is off, so this subpackage is
safe to import without hydrogel installed until a gel is actually requested.
"""
