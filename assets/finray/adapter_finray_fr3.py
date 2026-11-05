"""Parametric FR3 / Franka Hand -> Fin Ray finger adapter (CadQuery).

Run (needs cadquery, e.g. `conda create -n cad python=3.11 && pip install cadquery`):

    python adapter_finray_fr3.py            # writes the files next to this script
    python adapter_finray_fr3.py --lug      # also the variant that pins the lug hole

Outputs (mm, in the FR3 finger frame described below):
    adapter_finray_fr3.step / .stl          the part (print two, identical)
    adapter_finray_fr3_lug.step / .stl      optional: cheeks extended to the lug hole
    fr3_carriage_tab.stl                    the Franka carriage tab, for renders/checks
    adapter_spec.json                       every dimension below + derived ones

Frame (identical for the left and the right finger; it is the URDF frame of
fr3_leftfinger / fr3_rightfinger): x across the finger, y outward from the
gripper centre (the stock pad face is the plane y = 0), z along the approach
axis away from the hand, origin 58.4 mm below fr3_hand.

The Franka side (Franka Hand product manual R50010 v1.2, 5.2.3 Fig. 5.4 and
5.2.4 Fig. 5.5; see README.md for which numbers to check with calipers):
    the carriage ends in a flat tab, 6.00 thick (y) x 21 wide (x), protruding
    11.40 below the housing, with 2 x M4 threads 12 apart (x = +-6) whose
    axes run along y, 7 mm from the tab's free end; the two tabs' inner faces
    are 25 mm apart when the gripper is closed (105 open)
The Fin Ray side (finray_finger.step): 40 x 25.8 x 10 mm base block, two
    Ø4.0 holes along the finger width at 5 and 35 mm from the contact face,
    5 mm below the base top; the contact face lands on the stock pad plane.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cadquery as cq

HERE = Path(__file__).resolve().parent

P = dict(
    # ── Franka carriage tab (official drawing) ──────────────────────────
    tab_thick=6.0, tab_width=21.0, tab_protrusion=11.4, tab_hole_spacing=12.0,
    tab_hole_from_end=7.0, tab_gap_closed=25.0,
    # ── FR3 description meshes (franka_description, as Newton downloads it)
    housing_z=7.6,               # hand.dae underside: 66.0 - 58.4 mm
    # ── Fin Ray base block (finray_finger.step) ─────────────────────────
    finger_width=25.8, finger_depth=40.0, base_h=10.0,
    finger_holes=((5.0, 5.0), (35.0, 5.0)),   # (y from contact face, z below base top)
    lug_hole=(35.0, 40.0),
    # ── the adapter's own choices ───────────────────────────────────────
    housing_clear=0.8,           # top face stays this far below the housing
    pocket_clear=0.15,           # per side around the 6 mm tab (y)
    pocket_clear_x=0.25,         # per side around the 21 mm tab (x)
    pocket_end_clear=0.4,        # below the tab's end
    bridge=4.0,                  # floor between the tab pocket and the finger
    inner_y=1.0,                 # adapter stays 1 mm behind the contact face
    outer_cheek=5.0,             # behind the tab, carries the M4 x 12 heads
    clamp_half_x=13.0,
    cheek=5.0, cheek_clear=0.1,  # clevis either side of the 25.8 mm base block
    cheek_below=1.0,             # cheeks run 1 mm past the base block's underside
    m4_clear=4.5, m4_nut_af=7.3, m4_nut_depth=3.4,
    tab_screw_relief_to_y=6.0,   # the M4 x 12 tips may pass the tab's inner face
    edge_fillet=0.8, pocket_chamfer=0.6,
)


def derived(p: dict) -> dict:
    d = {}
    d["tab_inner_y"] = p["tab_gap_closed"] / 2.0
    d["tab_end_z"] = p["housing_z"] + p["tab_protrusion"]
    d["tab_hole_z"] = d["tab_end_z"] - p["tab_hole_from_end"]
    d["top_z"] = p["housing_z"] + p["housing_clear"]
    d["pocket_y0"] = d["tab_inner_y"] - p["pocket_clear"]
    d["pocket_y1"] = d["tab_inner_y"] + p["tab_thick"] + p["pocket_clear"]
    d["pocket_bottom_z"] = d["tab_end_z"] + p["pocket_end_clear"]
    d["z_base"] = d["pocket_bottom_z"] + p["bridge"]           # finger base top
    d["outer_y"] = d["pocket_y1"] + p["outer_cheek"]
    d["cheek_x0"] = p["finger_width"] / 2.0 + p["cheek_clear"]
    d["cheek_x1"] = d["cheek_x0"] + p["cheek"]
    d["cheek_z1"] = d["z_base"] + p["base_h"] + p["cheek_below"]
    d["finger_tip_z"] = d["z_base"] + 123.36
    d["finger_hole_z"] = [d["z_base"] + h[1] for h in p["finger_holes"]]
    return d


def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def adapter(p: dict, lug: bool = False) -> cq.Workplane:
    d = derived(p)
    cx = p["clamp_half_x"]
    body = box(-cx, cx, p["inner_y"], d["outer_y"], d["top_z"], d["pocket_bottom_z"])
    body = body.union(box(-d["cheek_x1"], d["cheek_x1"], p["inner_y"], p["finger_depth"],
                          d["pocket_bottom_z"], d["z_base"]))
    for s in (-1, 1):
        x0, x1 = sorted((s * d["cheek_x0"], s * d["cheek_x1"]))
        body = body.union(box(x0, x1, p["inner_y"], p["finger_depth"], d["z_base"], d["cheek_z1"]))
        if lug:
            ly, lz = p["lug_hole"]
            body = body.union(box(x0, x1, ly - 5.0, p["finger_depth"], d["cheek_z1"] - 1.0,
                                  d["z_base"] + lz + 5.0))
    # outside edges first (a fillet after the cuts would chase the holes)
    try:
        body = body.edges("|Y").fillet(p["edge_fillet"])
    except Exception:
        pass
    # the tab pocket, open toward the hand, with a lead-in chamfer
    tx = p["tab_width"] / 2.0 + p["pocket_clear_x"]
    pocket = box(-tx, tx, d["pocket_y0"], d["pocket_y1"], d["top_z"] - 1.0, d["pocket_bottom_z"])
    body = body.cut(pocket)
    try:
        body = body.faces(f"<Z").edges(cq.selectors.BoxSelector(
            (-tx - 0.01, d["pocket_y0"] - 0.01, d["top_z"] - 0.01),
            (tx + 0.01, d["pocket_y1"] + 0.01, d["top_z"] + 0.01))).chamfer(p["pocket_chamfer"])
    except Exception:
        pass
    # the two M4 x 12 screws into the tab: clearance through the outer cheek
    for sx in (-1, 1):
        x = sx * p["tab_hole_spacing"] / 2.0
        hole = (cq.Workplane("XZ").center(x, d["tab_hole_z"]).circle(p["m4_clear"] / 2.0)
                .extrude(-(d["outer_y"] - p["tab_screw_relief_to_y"]) - 1.0)
                .translate((0, p["tab_screw_relief_to_y"], 0)))
        body = body.cut(hole)
    # the finger bolts: Ø4.5 through both cheeks, an M4 nut trap in the +x cheek
    holes = list(p["finger_holes"])
    if lug:
        holes.append((p["lug_hole"][0], p["lug_hole"][1]))
    for hy, hz in holes:
        z = d["z_base"] + hz
        bolt = (cq.Workplane("YZ").center(hy, z).circle(p["m4_clear"] / 2.0)
                .extrude(2 * d["cheek_x1"] + 2.0).translate((-d["cheek_x1"] - 1.0, 0, 0)))
        body = body.cut(bolt)
        nut = (cq.Workplane("YZ").center(hy, z).polygon(6, p["m4_nut_af"] / 0.8660254, circumscribed=False)
               .extrude(p["m4_nut_depth"] + 1.0).translate((d["cheek_x1"] - p["m4_nut_depth"], 0, 0)))
        body = body.cut(nut.rotate((d["cheek_x1"], hy, z), (d["cheek_x1"] + 1, hy, z), 30.0))
    return body


def carriage_tab(p: dict) -> cq.Workplane:
    """The Franka carriage tab (with its M4 holes) and a slab of housing."""
    d = derived(p)
    t = box(-p["tab_width"] / 2.0, p["tab_width"] / 2.0, d["tab_inner_y"], d["tab_inner_y"] + p["tab_thick"],
            p["housing_z"] - 3.0, d["tab_end_z"])
    for sx in (-1, 1):
        t = t.cut(cq.Workplane("XZ").center(sx * p["tab_hole_spacing"] / 2.0, d["tab_hole_z"])
                  .circle(1.65).extrude(-20).translate((0, 0, 0)).translate((0, 0, 0))
                  .translate((0, d["tab_inner_y"] - 5.0, 0)))
    return t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--lug", action="store_true", help="also write the lug-pin variant")
    ap.add_argument("--out", default=str(HERE))
    a = ap.parse_args()
    out = Path(a.out)
    variants = [("adapter_finray_fr3", False)] + ([("adapter_finray_fr3_lug", True)] if a.lug else [])
    spec = {"parameters": P, "derived": derived(P), "units": "mm", "frame": "FR3 finger frame"}
    for name, lug in variants:
        part = adapter(P, lug=lug)
        cq.exporters.export(part, str(out / f"{name}.step"))
        cq.exporters.export(part, str(out / f"{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        bb = part.val().BoundingBox()
        spec[name] = {"volume_mm3": round(part.val().Volume(), 1),
                      "mass_g_petg_100pct": round(part.val().Volume() * 1.27e-3, 1),
                      "bbox_mm": [[round(bb.xmin, 2), round(bb.ymin, 2), round(bb.zmin, 2)],
                                  [round(bb.xmax, 2), round(bb.ymax, 2), round(bb.zmax, 2)]]}
        print(f"wrote {name}.step/.stl  volume {spec[name]['volume_mm3']} mm^3, bbox {spec[name]['bbox_mm']}")
    cq.exporters.export(carriage_tab(P), str(out / "fr3_carriage_tab.stl"), tolerance=0.02)
    (out / "adapter_spec.json").write_text(json.dumps(spec, indent=1))
    print("wrote adapter_spec.json")


if __name__ == "__main__":
    main()
