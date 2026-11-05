"""force-bench: force-realistic manipulation assets and tasks in Newton physics.

Submodules are imported explicitly (``force_bench.rig``, ``force_bench.tasks.<name>``);
this package deliberately imports nothing eagerly because several modules read
``TB_*`` environment variables at import time (see ``force_bench.tactile.task``).
"""

__version__ = "0.1.0"
