"""Import the printable ContactBench kit from the project website into ``assets/kit/``.

    force-bench kit import                       # from https://jingyi-xiang.github.io/contactbench/
    force-bench kit import --site <dir-or-url>   # from a local mirror of the site

The website's asset page (``assets.html``) is generated from ``config/asset-page.js``: one entry per printable
asset, with its parts, its variants (springs, rotation amounts, knob shapes, ...) and their colours.  This module
reads that config, downloads every file it names, unpacks the zips, and writes

    assets/kit/<asset-key>/<part>.stl       every printable part, unchanged (millimetres, Z up, in the asset's
                                            CAD assembly frame, so the parts of one asset are already in place)
    assets/kit/<asset-key>/*.obj, *.mtl     the site's multi-body visual models, kept where an assembly needs them
                                            (the door with its modules mounted)
    assets/kit/catalog.json                 the transcription: assets, parts, variants, colours, SHA-256 hashes
    assets/kit/tasks.json                   the task pages (``config/tasks/*.js``): every task's description,
                                            success criterion, assets, real-world videos and variation axes, under
                                            the paper's numbering (the site skips id 20: site 21-32 = paper 20-31)

    force-bench kit import --tasks-only          # refresh tasks.json alone (no part downloads)

``catalog.json`` and ``tasks.json`` are never edited by hand; re-run the import when the site changes.  The
physical model of each asset (roles, joints, springs, masses, collision settings) lives in ``assets/kit/sim.yaml``
(hand-written) and ``force_bench.kit``.
"""

from __future__ import annotations

import argparse
import datetime
import hashlib
import io
import json
import re
import shutil
import urllib.request
import zipfile
from pathlib import Path

SITE = "https://jingyi-xiang.github.io/contactbench/"
CONFIG = "config/asset-page.js"
TASK_INDEX = "config/tasks/index.js"
TASK_VIDEOS = "website_assets/videos/tasks/"


# ── the site's config is a JavaScript object literal: turn it into JSON ──────────────────────────────────────────
def js_object_to_json(src: str) -> str:
    """Convert the object literal passed to ``defineAssetPage(...)`` to JSON.

    Handles what the config uses: // and /* */ comments, single- and double-quoted strings, bare identifier keys,
    trailing commas.  Expressions (concatenation, calls, variables) are not supported and raise."""
    out, i, n = [], 0, len(src)
    while i < n:
        c = src[i]
        if src.startswith("//", i):
            i = src.find("\n", i)
            i = n if i < 0 else i
            continue
        if src.startswith("/*", i):
            i = src.index("*/", i) + 2
            continue
        if c in "'\"":
            j, buf = i + 1, []
            while src[j] != c:
                if src[j] == "\\":
                    buf.append(src[j:j + 2])
                    j += 2
                    continue
                buf.append(src[j])
                j += 1
            s = "".join(buf).replace("\\'", "'")
            out.append(json.dumps(json.loads('"' + s.replace('"', '\\"') + '"')))
            i = j + 1
            continue
        m = re.match(r"[A-Za-z_$][\w$]*", src[i:])
        if m:
            word = m.group(0)
            rest = src[i + len(word):].lstrip()
            if rest.startswith(":"):
                out.append(json.dumps(word))
            elif word in ("true", "false", "null"):
                out.append(word)
            else:
                raise ValueError(f"unsupported JavaScript expression near {src[i:i + 40]!r}")
            i += len(word)
            continue
        out.append(c)
        i += 1
    text = "".join(out)
    return re.sub(r",(\s*[}\]])", r"\1", text)


def _call_argument(js: str, fn: str) -> str:
    start = js.index(fn + "(") + len(fn) + 1
    return js[start:js.rindex(")")]


def parse_asset_page(js: str) -> dict:
    return json.loads(js_object_to_json(_call_argument(js, "defineAssetPage")))


def paper_number(site_id: str) -> str:
    """The paper's task number for a site task id: the site skips 20, so its 21-32 are the paper's 20-31;
    challenge tasks (C01-C05) keep their ids."""
    if not site_id.isdigit():
        return site_id.upper()
    n = int(site_id)
    return str(n - 1 if n >= 21 else n)


# ── fetching ────────────────────────────────────────────────────────────────────────────────────────────────────
class Site:
    def __init__(self, root: str):
        self.root = root if root.endswith("/") else root + "/"
        self.local = not self.root.startswith(("http://", "https://"))

    def read(self, rel: str) -> bytes:
        if self.local:
            return (Path(self.root) / rel).read_bytes()
        with urllib.request.urlopen(self.root + rel, timeout=120) as r:
            return r.read()


def _sha256(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def _as_list(v):
    if v is None or v == "":
        return []
    return list(v) if isinstance(v, list) else [v]


# ── the import ──────────────────────────────────────────────────────────────────────────────────────────────────
class _Importer:
    def __init__(self, site: Site, page: dict, out: Path):
        self.site, self.page, self.out = site, page, out
        self.dir = page["dir"]
        self.colors = page["colors"]
        self.cache: dict[str, bytes] = {}

    def fetch(self, rel: str) -> bytes:
        if rel not in self.cache:
            self.cache[rel] = self.site.read(self.dir + rel)
        return self.cache[rel]

    def stls_of(self, download: str) -> dict[str, bytes]:
        """The printable STL files behind one download entry (an .stl, or a .zip of them)."""
        if not download:
            return {}
        data = self.fetch(download)
        if download.endswith(".stl"):
            return {Path(download).name: data}
        if download.endswith(".zip"):
            out = {}
            with zipfile.ZipFile(io.BytesIO(data)) as zf:
                for name in zf.namelist():
                    if name.lower().endswith(".stl"):
                        out[Path(name).name] = zf.read(name)
            return out
        raise ValueError(f"unexpected download type: {download}")

    def obj_colors(self, model: str) -> list[tuple[str, list[int], tuple]]:
        """(body name, colour 0-255, (min, max) bounds in mm) of each body of a visual .obj model."""
        import numpy as np
        obj = self.fetch(model).decode("utf-8", "replace").splitlines()
        mtl_name = model[:-4] + ".mtl"
        kd = {}
        try:
            cur = None
            for line in self.fetch(mtl_name).decode().splitlines():
                if line.startswith("newmtl "):
                    cur = line.split(None, 1)[1].strip()
                elif line.startswith("Kd ") and cur:
                    kd[cur] = [round(float(x) * 255) for x in line.split()[1:4]]
        except Exception:
            pass
        verts, bodies, cur_body, cur_mtl = [], [], None, None
        for line in obj:
            if line.startswith("v "):
                verts.append([float(x) for x in line.split()[1:4]])
            elif line.startswith("g "):
                cur_body = {"name": line.split(None, 1)[1].strip(), "mtl": cur_mtl, "idx": set()}
                bodies.append(cur_body)
            elif line.startswith("usemtl "):
                cur_mtl = line.split(None, 1)[1].strip()
                if cur_body is not None:
                    cur_body["mtl"] = cur_mtl
            elif line.startswith("f ") and cur_body is not None:
                for tok in line.split()[1:]:
                    k = int(tok.split("/")[0])
                    cur_body["idx"].add(k - 1 if k > 0 else len(verts) + k)
        V = np.asarray(verts)
        out = []
        for b in bodies:
            if not b["idx"]:
                continue
            p = V[sorted(b["idx"])]
            out.append((b["name"], kd.get(b["mtl"]), (p.min(0), p.max(0))))
        return out

    def color_from_obj(self, stl: bytes, objs) -> list[int] | None:
        """Colour of an STL part from the visual .obj bodies that share its CAD frame.

        The site's .obj files are Fusion exports in CENTIMETRES (the STLs are millimetres), and one printed part
        can be several .obj bodies (a dial's grid marks).  The part takes the colour of the .obj body with the same
        bounds, else of the largest body inside its bounds."""
        import numpy as np
        import trimesh
        m = trimesh.load(io.BytesIO(stl), file_type="stl", force="mesh")
        lo, hi = m.bounds
        best = None
        for scale in (10.0, 1.0):
            for _, rgb, (blo, bhi) in objs:
                e = float(np.abs(blo * scale - lo).max() + np.abs(bhi * scale - hi).max())
                if e < 0.5 and (best is None or e < best[0]):
                    best = (e, rgb)
        if best:
            return best[1]
        inside = []
        for scale in (10.0, 1.0):
            for _, rgb, (blo, bhi) in objs:
                if np.all(blo * scale >= lo - 0.3) and np.all(bhi * scale <= hi + 0.3):
                    inside.append((float(np.prod(np.maximum(bhi - blo, 1e-6) * scale)), rgb))
        return max(inside, key=lambda t: t[0])[1] if inside else None

    def part_entries(self, key, download, model, color, objs_from=None, fallback_color=None) -> list[dict]:
        """Write one download's STL parts into assets/kit/<key>/ and describe them."""
        stls = self.stls_of(download)
        models = _as_list(model)
        colors = _as_list(color)
        objs = []
        for m in models + _as_list(objs_from):
            if m.endswith(".obj"):
                objs += self.obj_colors(m)
        entries = []
        folder = self.out / key
        folder.mkdir(parents=True, exist_ok=True)
        for name, data in sorted(stls.items()):
            c = None
            stl_models = [Path(m).name for m in models if m.endswith(".stl")]
            if name in stl_models and colors:
                c = colors[stl_models.index(name)] if len(colors) > 1 else colors[0]
            if c is None and objs:
                rgb = self.color_from_obj(data, objs)
                c = rgb if rgb is not None else None
            if c is None:
                c = colors[0] if len(colors) == 1 else fallback_color
            (folder / name).write_bytes(data)
            entries.append({"name": Path(name).stem, "file": f"{key}/{name}", "sha256": _sha256(data),
                            "color": self.colors.get(c, c) if isinstance(c, str) else c,
                            "color_name": c if isinstance(c, str) else None})
        return entries

    def run(self) -> dict:
        assets = {}
        for a in self.page["assets"]:
            key = a["key"]
            color = a.get("color")
            entry = {"name": a["name"], "note": a.get("note", ""), "site": f"assets.html#{key}"}
            if a.get("stl"):
                entry["parts"] = self.part_entries(key, a["stl"], a.get("model"), color)
                if a.get("model", "").endswith(".obj") if isinstance(a.get("model"), str) else False:
                    self._keep_visual(key, a["model"])
            base = a.get("base")
            if base:
                entry["parts"] = self.part_entries(key, base.get("stl"), base.get("model"),
                                                   base.get("color", color), fallback_color=color)
            groups = []
            if a.get("variants"):
                groups.append({"variation": a.get("variation", ""), "slot": None,
                               "variants": a["variants"], "color": color, "offset": None})
            for g in a.get("groups") or []:
                groups.append({"variation": g.get("variation", ""), "slot": g.get("slot"),
                               "variants": g["variants"], "color": g.get("color", color), "offset": g.get("offset")})
            if groups:
                entry["groups"] = []
                for g in groups:
                    vs = []
                    for v in g["variants"]:
                        ve = {"label": v["label"], "default": bool(v.get("default"))}
                        if v.get("stl"):
                            ve["parts"] = self.part_entries(key, v["stl"], v.get("model"),
                                                            v.get("color", g["color"]), fallback_color=color,
                                                            objs_from=(base or {}).get("model"))
                        vs.append(ve)
                    entry["groups"].append({"variation": g["variation"], "slot": g["slot"], "variants": vs,
                                            "display_offset_mm": g["offset"]})
            if a.get("combinations"):
                entry["combinations"] = [
                    {"variants": c["variants"], "parts": self.part_entries(key, c["stl"], c.get("model"),
                                                                           c.get("color", color))}
                    for c in a["combinations"]]
            assets[key] = entry
        return assets

    def _keep_visual(self, key, model):
        """Keep a multi-body .obj (+ .mtl) next to the parts: it records how an assembly is put together."""
        folder = self.out / key
        for rel in (model, model[:-4] + ".mtl"):
            try:
                (folder / Path(rel).name).write_bytes(self.fetch(rel))
            except Exception:
                pass


def import_tasks(site: str = SITE) -> dict:
    """The site's task pages as data: {paper number: task}, each with the site id, name, skills, description,
    success criterion, kit assets, real-world video URLs and variation axes (name, type, instances)."""
    s = Site(site)
    names = json.loads(js_object_to_json(_call_argument(s.read(TASK_INDEX).decode("utf-8"), "defineTaskList")))
    tasks = {}
    for stem in names:
        t = json.loads(js_object_to_json(_call_argument(s.read(f"config/tasks/{stem}.js").decode("utf-8"),
                                                        "defineTask")))
        video = lambda v: s.root + TASK_VIDEOS + v if v else ""  # noqa: E731
        tasks[paper_number(t["id"])] = {
            "site_id": t["id"],
            "name": t["name"],
            "skills": t.get("skills", []),
            "description": t.get("description", ""),
            "success": t.get("success", ""),
            "assets": t.get("assetCards", []),
            "video": video(t.get("video", "")),
            "variations": [{"axis": v["axis"], "type": v.get("type", ""),
                            "instances": [{"label": i.get("label", ""), "assets": i.get("assets", []),
                                           "video": video(i.get("video", ""))} for i in v.get("instances", [])]}
                           for v in t.get("variations", [])],
        }
    return {"source": s.root, "config": TASK_INDEX, "imported": datetime.date.today().isoformat(),
            "numbering": "paper Table C1 (the site's ids 21-32 are the paper's 20-31)", "tasks": tasks}


def import_kit(site: str = SITE, out: Path | None = None) -> dict:
    from force_bench.paths import ASSETS_ROOT
    out = Path(out) if out else ASSETS_ROOT / "kit"
    s = Site(site)
    js = s.read(CONFIG).decode("utf-8")
    page = parse_asset_page(js)
    tmp = out.with_name(out.name + ".importing")
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    keep = out / "sim.yaml"
    assets = _Importer(s, page, tmp).run()
    catalog = {
        "source": s.root + "assets.html",
        "config": CONFIG,
        "config_sha256": _sha256(js.encode()),
        "imported": datetime.date.today().isoformat(),
        "units": "mm",
        "up": "+z",
        "colors": page["colors"],
        "assets": assets,
    }
    (tmp / "catalog.json").write_text(json.dumps(catalog, indent=1) + "\n")
    (tmp / "tasks.json").write_text(json.dumps(import_tasks(site), indent=1, ensure_ascii=False) + "\n")
    if keep.exists():
        shutil.copy2(keep, tmp / "sim.yaml")
    if out.exists():
        shutil.rmtree(out)
    tmp.rename(out)
    return catalog


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="force-bench kit import", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--site", default=SITE, help="site root URL or a local mirror directory")
    ap.add_argument("--out", default=None, help="output folder (default assets/kit)")
    ap.add_argument("--tasks-only", action="store_true", help="refresh tasks.json only")
    a = ap.parse_args(argv)
    if a.tasks_only:
        from force_bench.paths import ASSETS_ROOT
        out = Path(a.out) if a.out else ASSETS_ROOT / "kit"
        tasks = import_tasks(a.site)
        (out / "tasks.json").write_text(json.dumps(tasks, indent=1, ensure_ascii=False) + "\n")
        print(f"[kit import] {len(tasks['tasks'])} task pages -> {out / 'tasks.json'}")
        return 0
    cat = import_kit(a.site, a.out)
    n_parts = sum(len(x.get("parts", [])) + sum(len(v.get("parts", [])) for g in x.get("groups", [])
                                                 for v in g["variants"])
                  + sum(len(c["parts"]) for c in x.get("combinations", [])) for x in cat["assets"].values())
    print(f"[kit import] {len(cat['assets'])} assets, {n_parts} part files -> {a.out or 'assets/kit'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
