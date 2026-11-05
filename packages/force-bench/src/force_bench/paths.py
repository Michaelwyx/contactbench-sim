"""Where the benchmark's data lives, and where its runs go.

The assets (``assets/bodies``, ``assets/manifest.yaml``) are NOT package data:
they are 100+ MB of STL and live at the repository root.  They are found, in
order, from ``TB_ASSETS_ROOT``, by walking up from this package to a directory
holding ``assets/manifest.yaml`` (a source checkout), and finally from the
current working directory.

``TB_OUT_ROOT`` redirects every artefact a task writes.  Repeat runs and
ablations otherwise overwrite the PUBLISHED rollout, panel and QA of the task
they are studying -- measuring a task's reproducibility would destroy the
very outputs being characterised.

Layout:  outputs/task/<task>            the benchmark tasks (bare-handed)
         outputs/gelsight/<study>       the GelSight simulation's own evaluation
         outputs/task_gelsight/<task>   the tasks with the tactile sensor integrated
"""

from __future__ import annotations

import os
from pathlib import Path


def _find_repo_root() -> Path:
    env = os.environ.get("TB_ASSETS_ROOT")
    if env:
        return Path(env).expanduser().resolve().parent if Path(env).name == "assets" else Path(env).expanduser().resolve()
    here = Path(__file__).resolve()
    for p in [here, *here.parents]:
        if (p / "assets" / "manifest.yaml").exists():
            return p
    cwd = Path.cwd().resolve()
    for p in [cwd, *cwd.parents]:
        if (p / "assets" / "manifest.yaml").exists():
            return p
    raise FileNotFoundError(
        "force_bench cannot find the benchmark assets: set TB_ASSETS_ROOT to the "
        "directory that contains assets/manifest.yaml (a checkout of this repository)")


REPO_ROOT = _find_repo_root()
ASSETS_ROOT = REPO_ROOT / "assets"
BODIES_DIR = ASSETS_ROOT / "bodies"
MANIFEST_PATH = ASSETS_ROOT / "manifest.yaml"

OUTPUTS_ROOT = Path(os.environ.get("TB_OUT_ROOT") or (REPO_ROOT / "outputs"))
OUTPUT_DIR = OUTPUTS_ROOT / "task"
GELSIGHT_OUTPUT_DIR = OUTPUTS_ROOT / "gelsight"
TASK_GELSIGHT_OUTPUT_DIR = OUTPUTS_ROOT / "task_gelsight"
SDF_CACHE = Path(os.environ.get("TMPDIR", "/tmp")) / "tactile_bench_sdf_cache"
