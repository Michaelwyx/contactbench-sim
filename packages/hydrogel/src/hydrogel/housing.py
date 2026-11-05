"""Install GelSight Inc.'s GS Mini housing CAD as the sensor's render mesh (visual only).

    hydrogel housing --download               # fetch gsmini_shell.zip from gelsightinc/gsrobotics, convert, install
    hydrogel housing path/to/gsmini_shell.zip # the same from a file you downloaded (.zip or .stp)

The housing comes from the gelsightinc/gsrobotics repository (``docs/CAD/gsmini_shell.zip``), which is licensed under
GPL-3.0, so it is not distributed with this package. This command converts the STEP file to a triangle mesh in
metres (``trimesh`` with ``cascadio``: ``pip install cascadio``) and saves it as ``gsmini_shell.obj`` in
``hydrogel/data/gs_mini/``, where ``force_bench.tactile`` picks it up: renders then show the real housing instead of
the parametric case box. Collision and mass stay on the case box either way. Keep the converted file local.
"""

from __future__ import annotations

import argparse
import io
import tempfile
import urllib.request
import zipfile
from pathlib import Path

URL = "https://github.com/gelsightinc/gsrobotics/raw/main/docs/CAD/gsmini_shell.zip"
EXTENTS_MM = (32.0, 28.25, 28.89)   # the CAD's own frame: elastomer at +y, mounting plate at -y
TOL_MM = 0.3


def _step_bytes(source: Path | None) -> bytes:
    if source is None:
        print(f"[housing] downloading {URL} (GPL-3.0, GelSight Inc.)")
        with urllib.request.urlopen(URL, timeout=120) as r:
            data, name = r.read(), "gsmini_shell.zip"
    else:
        data, name = source.read_bytes(), source.name
    if name.lower().endswith(".zip"):
        with zipfile.ZipFile(io.BytesIO(data)) as z:
            steps = [n for n in z.namelist() if n.lower().endswith((".stp", ".step"))]
            if len(steps) != 1:
                raise SystemExit(f"hydrogel housing: expected one STEP file in {name}, found {steps}")
            return z.read(steps[0])
    return data


def convert(step: bytes):
    """The housing as one trimesh.Trimesh in metres, in the CAD's frame."""
    try:
        import cascadio  # noqa: F401  (trimesh's STEP loader)
    except ImportError:
        raise SystemExit("hydrogel housing: reading STEP needs cascadio: pip install cascadio") from None
    import trimesh
    with tempfile.TemporaryDirectory() as d:
        p = Path(d) / "gsmini_shell.stp"
        p.write_bytes(step)
        scene = trimesh.load(str(p))
    mesh = scene.to_geometry() if hasattr(scene, "to_geometry") else scene.dump(concatenate=True)
    got = [round(float(e) * 1000.0, 2) for e in mesh.extents]
    if any(abs(g - e) > TOL_MM for g, e in zip(got, EXTENTS_MM)):
        raise SystemExit(f"hydrogel housing: extents {got} mm, expected {list(EXTENTS_MM)} mm: not the GS Mini shell")
    return mesh


def main(argv: list[str] | None = None) -> int:
    from hydrogel.paths import GS_MINI_DIR
    ap = argparse.ArgumentParser(prog="hydrogel housing", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", nargs="?", type=Path, help="gsmini_shell.zip or gsmini_shell.stp")
    ap.add_argument("--download", action="store_true", help=f"fetch the file from {URL}")
    ap.add_argument("--out", type=Path, default=GS_MINI_DIR / "gsmini_shell.obj")
    a = ap.parse_args(argv)
    if (a.source is None) == (not a.download):
        ap.error("give the downloaded file, or --download")
    mesh = convert(_step_bytes(a.source))
    a.out.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(str(a.out))
    print(f"[housing] {len(mesh.faces)} faces, {[round(float(e) * 1000, 2) for e in mesh.extents]} mm -> {a.out}")
    print("[housing] GPL-3.0 (GelSight Inc., gelsightinc/gsrobotics): keep it local; GS_SHELL_VISUAL=0 hides it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
