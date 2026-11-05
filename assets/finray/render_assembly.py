"""Assembly render of the Fin Ray adapter on the Franka Hand (matplotlib, no GPU).

    python render_assembly.py [--q 20] [--out assembly_fr3_finray.png]

Shows the FR3 hand (franka_description's visual mesh, as Newton downloads it;
--hand PATH to point at another copy), the two carriage tabs, the two
adapters and the two Fin Ray fingers at a finger opening q (mm per finger), plus
a dimensioned side view in the finger frame with the stock finger's outline.
"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import numpy as np
import trimesh
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

HERE = Path(__file__).resolve().parent


def finger_T(side: str, q_mm: float) -> np.ndarray:
    """Finger frame -> hand frame (mm)."""
    T = np.eye(4)
    T[2, 3] = 58.4
    if side == "L":
        T[1, 3] = q_mm
    else:
        T[:3, :3] = np.diag([-1.0, -1.0, 1.0])
        T[1, 3] = -q_mm
    return T


def step_to_finger(spec) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = [[0, 1, 0], [1, 0, 0], [0, 0, -1]]
    T[:3, 3] = [spec["parameters"]["finger_width"] / 2.0, 0.0, spec["derived"]["z_base"]]
    return T


def shade(mesh: trimesh.Trimesh, color, light=np.array([0.4, -0.6, 0.7])):
    n = mesh.face_normals
    lam = 0.35 + 0.65 * np.clip(n @ (light / np.linalg.norm(light)), 0, 1)
    c = np.asarray(color)[None, :] * lam[:, None]
    return np.clip(c, 0, 1)


def add(ax, mesh, color, alpha=1.0, lw=0.0):
    tris = mesh.vertices[mesh.faces]
    pc = Poly3DCollection(tris, facecolors=shade(mesh, color), linewidths=lw, alpha=alpha)
    ax.add_collection3d(pc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--q", type=float, default=22.0)
    ap.add_argument("--hand", default=None)
    ap.add_argument("--out", default=str(HERE / "assembly_fr3_finray.png"))
    a = ap.parse_args()
    spec = json.loads((HERE / "adapter_spec.json").read_text())
    adapter = trimesh.load(HERE / "adapter_finray_fr3.stl")
    finger = trimesh.load(HERE / "finray_finger.stl")
    tab = trimesh.load(HERE / "fr3_carriage_tab.stl")
    finger.apply_transform(step_to_finger(spec))
    hand_path = a.hand or (glob.glob(str(Path.home() / ".cache/newton/newton-assets_franka_emika_panda_*/"
                                         "franka_emika_panda/meshes/robot_ee/franka_hand_white/visual/hand.dae")) or [None])[0]
    hand = None
    if hand_path:
        hs = trimesh.load(hand_path, force="scene")
        hand = hs.dump(concatenate=True) if hasattr(hs, "dump") else hs
        hand.apply_scale(1000.0)
    fig = plt.figure(figsize=(15, 7.6))
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    for side in "LR":
        T = finger_T(side, a.q)
        for m, col in ((adapter, (0.25, 0.26, 0.30)), (finger, (0.96, 0.52, 0.10)), (tab, (0.70, 0.72, 0.75))):
            mm = m.copy()
            mm.apply_transform(T)
            add(ax, mm, col)
    if hand is not None:
        h = hand.copy()
        keep = h.vertices[h.faces][:, :, 2].min(axis=1) > -20
        h.update_faces(keep)
        add(ax, h, (0.93, 0.93, 0.93), alpha=0.35)
    ax.set_xlim(-90, 90), ax.set_ylim(-90, 90), ax.set_zlim(220, 40)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=18, azim=-58)
    ax.set_xlabel("x (mm)"), ax.set_ylabel("y (mm)"), ax.set_zlabel("z below fr3_hand (mm)")
    ax.set_title(f"FR3 hand + 2 x adapter + 2 x Fin Ray, q = {a.q:.0f} mm per finger")

    # ── side view in the finger frame (y-z), dimensioned ────────────────
    bx = fig.add_subplot(1, 2, 2)
    d = spec["derived"]
    p = spec["parameters"]
    sec = adapter.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    for e in sec.entities:
        v = sec.vertices[e.points]
        bx.plot(v[:, 1], v[:, 2], c="0.2", lw=1.2)
    sec = adapter.section(plane_origin=[15.5, 0, 0], plane_normal=[1, 0, 0])
    for e in sec.entities:
        v = sec.vertices[e.points]
        bx.plot(v[:, 1], v[:, 2], c="0.5", lw=0.8, ls="--")
    sec = finger.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    for e in sec.entities:
        v = sec.vertices[e.points]
        bx.fill(v[:, 1], v[:, 2], fc=(0.96, 0.52, 0.10, 0.25), ec=(0.85, 0.40, 0.05), lw=0.8)
    sec = tab.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    for e in sec.entities:
        v = sec.vertices[e.points]
        bx.fill(v[:, 1], v[:, 2], fc="0.75", ec="0.3", lw=0.8)
    # stock finger collision boxes (URDF), for scale
    for c, s in (((18.5, 11.0), (15.0, 20.0)), ((6.8, 2.2), (8.8, 3.8)), ((7.58, 45.25), (15.2, 18.5))):
        bx.add_patch(plt.Rectangle((c[0] - s[0] / 2, c[1] - s[1] / 2), s[0], s[1], fc="none", ec="tab:blue",
                                   ls=":", lw=1))
    bx.axhline(p["housing_z"], c="k", lw=0.6)
    bx.text(-4, p["housing_z"] - 1.5, "housing underside (z 7.6)", fontsize=7)
    bx.axvline(0.0, c="tab:red", lw=0.6, ls="--")
    bx.text(0.8, 157, "contact face = stock pad plane (y 0)", fontsize=7, color="tab:red")
    ann = [(f"tab 6.0 x 21, M4 at z {d['tab_hole_z']:.1f}", (d["tab_inner_y"] + 3, d["tab_hole_z"])),
           (f"pocket bottom z {d['pocket_bottom_z']:.1f}", (2, d["pocket_bottom_z"])),
           (f"finger base top z {d['z_base']:.1f}", (2, d["z_base"])),
           (f"finger tip z {d['finger_tip_z']:.1f} (stock tip 54.5)", (2, d["finger_tip_z"] - 4)),
           ("2 x M4 x 40 + nut, y 5 / 35", (36, d["finger_hole_z"][0]))]
    for txt, (yy, zz) in ann:
        bx.annotate(txt, (yy, zz), (45, zz + 3), fontsize=7, arrowprops=dict(arrowstyle="-", lw=0.5))
    bx.plot([], [], c="tab:blue", ls=":", label="stock finger (URDF collision boxes)")
    bx.plot([], [], c="0.5", ls="--", label="adapter cheek (x = 15.5)")
    bx.legend(loc="center right", fontsize=7)
    bx.set_xlim(-10, 100), bx.set_ylim(160, 0)
    bx.set_aspect("equal")
    bx.set_xlabel("y, outward from the gripper centre (mm)"), bx.set_ylabel("z, away from the hand (mm)")
    bx.set_title("Section x = 0, finger frame (fr3_leftfinger)")
    bx.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
