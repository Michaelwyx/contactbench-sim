"""Package data and output locations for hydrogel.

Package data (the vendored GelSight Mini calibration, CAD shell and the
validation indenters) ships inside the wheel under ``hydrogel/data``.
Outputs default to ``./outputs/gelsight`` and follow ``TB_OUT_ROOT`` so a
force-bench checkout and the sensor share one output tree.  The generated
MuJoCo kernel module goes to a user cache (``HYDROGEL_CACHE``), never into
the source tree.
"""

from __future__ import annotations

import os
from pathlib import Path

DATA_DIR = Path(__file__).resolve().parent / "data"
GS_MINI_DIR = DATA_DIR / "gs_mini"
INDENTER_DIR = DATA_DIR / "indenters"


def _outputs_root() -> Path:
    env = os.environ.get("TB_OUT_ROOT")
    if env:
        return Path(env).expanduser()
    # a source checkout of the repository: share its outputs/ tree
    here = Path(__file__).resolve()
    for p in here.parents:
        if (p / "assets" / "manifest.yaml").exists():
            return p / "outputs"
    return Path.cwd() / "outputs"


OUTPUTS_ROOT = _outputs_root()
GELSIGHT_OUTPUT_DIR = OUTPUTS_ROOT / "gelsight"
CACHE_DIR = Path(os.environ.get("HYDROGEL_CACHE")
                 or os.path.join(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache"), "hydrogel"))
SDF_CACHE = Path(os.environ.get("TMPDIR", "/tmp")) / "tactile_bench_sdf_cache"
