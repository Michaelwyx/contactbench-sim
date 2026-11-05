"""Appendix task 23 "Plug in charger" -- one variant per finalized-kit block.

    Description: An outlet is placed in the workspace.  Fully insert the
    charging block held in hand into the outlet.  Success: the charging
    block is fully inserted into the outlet, such that it does not
    noticeably move when further pushed toward the outlet.  The robot
    movement stops after terminating the insertion motion, moving away
    from, and breaking contact with the task objects.

The website lists four variants, "Charging block 1-4": the four chargers of
the finalized STEP kit (``STEP-20261015T040134Z-1-001.zip``, tessellated in
``paper-writing/figures/asset-overviews/final_asset_table/mesh_cache``).
``plug_in_charger.py`` runs the suite's own plug; this module runs each kit
block through the same outlet, mechanism, expert and appendix ending.

What the kit geometry says (measured from the STEP tessellation; kit frame =
Z up, standing on the prongs, blade midpoint at the origin):

  block  body above the prongs (mm)          prongs (16 mm blades, 2 mm thick,
                                             13.0 mm apart)
  1      white box 22.5 x 35 x 40           two 6 mm blades (unpolarized)
  2      25 x 28 box, 14 tall, tapering to   two 6 mm blades + a 4 mm round
         a 10 mm cord boot (64 mm overall)   ground pin, 19.0 mm long, 11 mm
                                             off the blade line
  3      55 x 35 brick on a 14 x 24 x 1.5    one blade widened to 8 mm near
         prong plateau = the suite's plug    its tip (polarized), one 6 mm
         (charging_block_3_1_body1, same
         surface to 0.15 mm)
  4      18 x 25 x 2.75 flange, 15 x 22 body polarized like 3, mirrored
         to 14 mm, taper + 5 x 8 stem

The kit outlet IS the suite's outlet (``shell_1_body1`` rotated 90 deg
about z): the face plate, the polarized 9 / 6.9 mm blade slots and the
D-shaped ground hole match exactly; only the inner spring pocket differs
(<= 0.5 mm), and the printed spring is replaced by the suite's retention
pads either way.  All four blocks fit it: every blade is 2 mm thick at the
13.0 mm spacing the pads were built for, the 8 mm blades of 3 and 4 only fit
the 9 mm slot (so the block is yawed to put them there), and block 2's
ground pin lands in the D hole (4 mm pin in a 5 mm round, 0.5 mm play).
One real interference: the D hole is BLIND, 18.5 mm deep, and block 2's
ground pin is 19.0 mm long, so block 2 seats on its pin tip with its face
0.5 mm proud of the face plate (the kit outlet's hole has the same 18.5 mm
depth).  "Fully inserted" is measured against that geometric stop.

Everything else is the suite task ``force_bench.tasks.plug_insert_arm`` run
through the ``plug_in_charger`` ending (in-hand start with the prong tips
4 mm above the face, closed-loop insertion on the measured depth to the
geometric seat, 1 s seated hold, force-ramped push test, release, 6 cm
retreat, still), with per-block patches applied for the run only:

  * pose: kit mesh yawed (polarization / ground pin), blades centred in
    their slots (max-min clearance), lowest tip 4 mm above the face;
  * seat: the vertical drop distance to first contact (signed-distance
    bisection, both directions), so plateau, body face or pin tip;
  * grasp: across the body's world-y width (35 / 28 / 35 / 22 mm), finger
    targets keep the suite's 0.5 mm touch / 2.5 mm squeeze; pad height
    chosen on each body's straight walls above the face (FR3 pad = 18.5 mm,
    tips 9.5 mm below the TCP, nothing else within 2.4 mm of the pad face);
  * mass: the suite plug's 60 g scaled by printed volume;
  * camera: a low front-left view of the face plate so the prongs, slots
    and the landing are not hidden by the hand (same for all four).

and three task-level changes, the same for all four blocks (see ENTRY_TRIM,
IMPRATIO and ChargerBlockVariant.should_cut for the measurements):

  * entry trim: a fixed (-0.27, -0.85) mm TCP offset on every drive command
    that cancels the arm's measured settle drift, so the prongs enter
    centred instead of riding the slot edges;
  * MuJoCo impratio 10 (the suite runs the default 1), which stops the
    pinch creeping through the stroke.  With the suite's settings blocks 2
    and 4 crept 1.2-1.3 mm through the fingers and the suite expert, whose
    command may only lead the MEASURED plug depth by 0.25-0.8 mm, froze
    2.9 mm (block 2) and 6.3 mm (block 4) short of the seat; the trim alone
    still left block 4 0.95 mm of creep and 4.4 mm short;
  * seat-latch press: the expert's "gentle ~10 N" seated press is corrected
    for the pre-roll settle its depth reference contains (it landed at
    40-44 N, pressing block 2's ground-pin tip 0.25 mm into the hole floor).

Meshes: ``assets/bodies_kit/charging_block_N/`` (``--export-meshes`` writes
them from the STEP tessellation; METRES, kit frame, provenance.json).

Run:  python -m force_bench.appendix_tasks.plug_in_charger_blocks --block N
      python -m force_bench.appendix_tasks.plug_in_charger_blocks --export-meshes
Outputs: outputs/appendix_tasks/plug_in_charger_block<N>/
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import trimesh

from force_bench.paths import ASSETS_ROOT, REPO_ROOT
from force_bench.rig import Rig
from force_bench.appendix_tasks.derive import run_variant
from force_bench.appendix_tasks.common import ease
from force_bench.appendix_tasks.plug_in_charger import OPEN_TO, ChargerVariant, VARIANT as SUITE_PLUG_VARIANT

KIT_DIR = ASSETS_ROOT / "bodies_kit"
KIT_CACHE = REPO_ROOT / "paper-writing/figures/asset-overviews/final_asset_table/mesh_cache"
KIT_ZIP = "STEP-20261015T040134Z-1-001.zip"

SUITE_PLUG_MASS = 0.060      # kg, plug_insert_arm.PLUG_MASS (= kit block 3)
SUITE_PLUG_VOLUME = 43.273e-6  # m^3, kit block 3
TIP_BELOW_TCP = 0.0095       # FR3 rubber-tip box bottom below the TCP point
APPROACH = 0.004             # lowest prong tip above the face at the start


@dataclass(frozen=True)
class BlockSpec:
    n: int
    yaw_deg: float              # kit frame -> world, about the blade midpoint
    grip_width: float           # body width across the fingers (world y), m
    tips_above_body: float      # fingertip height above the body's bottom face, m
    body_face_z: float          # kit z of the gripped body's bottom face, m
    entry_zone: tuple[float, float]   # slow-entry dz window (plug_insert_arm.ENTRY_ZONE)
    body: str
    prongs: str
    why_yaw: str
    why_grasp: str


BLOCKS = {
    1: BlockSpec(1, 0.0, 0.035, 0.014, 0.0, (0.0015, 0.0100),
                 "white box 22.5 x 35 x 40 mm",
                 "two 6 x 2 mm blades, 16 mm (unpolarized)",
                 "unpolarized and no ground pin: as exported",
                 "pads across the 35 mm faces, tips 14 mm above the body face "
                 "(pads cover 14-32.5 mm of the 40 mm walls)"),
    2: BlockSpec(2, 180.0, 0.028, 0.004, 0.0, (0.0015, 0.0130),
                 "25 x 28 mm box, straight walls to 14 mm, tapering to a 10 mm cord boot (45 mm)",
                 "two 6 x 2 mm blades, 16 mm + round 4 mm ground pin, 19.0 mm",
                 "180 deg: the ground pin goes to the D-shaped ground hole",
                 "pads across the 28 mm faces, tips 4 mm above the body face so the "
                 "contact stays on the 13 mm of straight wall below the taper"),
    3: BlockSpec(3, 180.0, 0.035, 0.0095, 0.0015, (0.0015, 0.0100),
                 "55 x 35 x 25 mm brick on a 14 x 24 x 1.5 mm prong plateau",
                 "one blade widened to 8 mm near the tip (polarized) + one 6 mm blade",
                 "180 deg: the 8 mm blade goes to the 9 mm slot",
                 "the suite's grasp: pads across the 35 mm faces over the blade axis, "
                 "tips 9.5 mm above the brick face"),
    4: BlockSpec(4, 0.0, 0.022, 0.00425, 0.0, (0.0015, 0.0100),
                 "18 x 25 x 2.75 mm flange, 15 x 22 mm body to 14 mm, taper, 5 x 8 mm stem (36 mm)",
                 "polarized like block 3, mirrored (8 mm blade on the -y side)",
                 "as exported: the 8 mm blade already faces the 9 mm slot",
                 "pads across the 22 mm body, tips 1.5 mm above the 25 mm flange "
                 "so the contact stays on the body's straight walls"),
}


# ── kit meshes ───────────────────────────────────────────────────────────
def _main_body(tm: trimesh.Trimesh) -> trimesh.Trimesh:
    """Largest connected body (block 2's tessellation carries 3 stray
    single triangles beside its closed main body)."""
    parts = tm.split(only_watertight=False)
    return max(parts, key=lambda p: len(p.faces)) if len(parts) > 1 else tm


def export_kit_meshes() -> list[Path]:
    """Write assets/bodies_kit/charging_block_N/charging_block_N.stl (METRES,
    kit frame) + provenance.json from the STEP tessellation cache."""
    written = []
    for n in BLOCKS:
        name = f"charging_block_{n}"
        meta = json.loads((KIT_CACHE / f"{name}.json").read_text())
        npz = np.load(KIT_CACHE / f"{name}.npz")
        parts = [trimesh.Trimesh(npz[f"v{p['key']}"].astype(float), npz[f"f{p['key']}"],
                                 process=True) for p in meta["parts"]]
        raw = trimesh.util.concatenate(parts)
        tm = _main_body(raw)
        assert tm.is_watertight, f"{name}: main body not watertight"
        out = KIT_DIR / name
        out.mkdir(parents=True, exist_ok=True)
        stl = out / f"{name}.stl"
        tm.export(stl)
        vol = float(tm.volume)
        prov = {
            "object": name,
            "units": "m",
            "frame": "kit STEP frame: Z up, standing on the prongs, blade midpoint at the "
                     "origin, prong base plane z = 0",
            "source": f"{KIT_ZIP} -> {meta['source']} (sha256 {meta['source_sha256']})",
            "tessellation": meta["tessellation"] | {"tool": "cascadio step_to_glb "
                                                    "(final_asset_table/source/convert_step.py)"},
            "kept": "largest connected body" + (
                f" (dropped {len(raw.faces) - len(tm.faces)} stray degenerate triangles)"
                if len(raw.faces) != len(tm.faces) else ""),
            "watertight": bool(tm.is_watertight),
            "faces": int(len(tm.faces)),
            "bounds_m": tm.bounds.round(6).tolist(),
            "volume_cm3": round(vol * 1e6, 3),
            "center_of_mass_m": np.round(tm.center_mass, 6).tolist(),
            "sim_mass_kg": round(SUITE_PLUG_MASS * vol / SUITE_PLUG_VOLUME, 4),
            "sim_mass_rule": "suite plug's 60 g (kit block 3) scaled by printed volume",
            "stl_sha256": hashlib.sha256(stl.read_bytes()).hexdigest(),
            "used_by": "force_bench.appendix_tasks.plug_in_charger_blocks",
        }
        (out / "provenance.json").write_text(json.dumps(prov, indent=1))
        written.append(stl)
        print(f"{name}: {len(tm.faces)} faces, {vol*1e6:.2f} cm3 -> {stl}")
    return written


def load_block(n: int) -> trimesh.Trimesh:
    stl = KIT_DIR / f"charging_block_{n}" / f"charging_block_{n}.stl"
    if not stl.exists():
        raise FileNotFoundError(f"{stl} missing: run with --export-meshes first")
    return trimesh.load(stl, force="mesh")


def block_mass(tm: trimesh.Trimesh) -> float:
    return SUITE_PLUG_MASS * float(tm.volume) / SUITE_PLUG_VOLUME


# ── geometry: pose, centre, seat ─────────────────────────────────────────
def blade_footprints(posed: trimesh.Trimesh, base_z: float, mid: np.ndarray):
    """xy bounds of each prong over its whole length (posed frame).

    Returns (blades sorted by y, pin or None); a prong is everything below
    the prong base plane, grouped by its offset from the blade midpoint."""
    v = np.asarray(posed.vertices)
    v = v[v[:, 2] < base_z - 0.0002]
    dy = v[:, 1] - mid[1]
    groups = {"lo": v[dy < -0.003], "hi": v[dy > 0.003], "pin": v[np.abs(dy) <= 0.003]}
    out = {}
    for k, g in groups.items():
        if len(g):
            lo, hi = g[:, :2].min(0), g[:, :2].max(0)
            out[k] = {"lo": lo, "hi": hi, "center": (lo + hi) / 2, "ext": hi - lo,
                      "tip_z": float(g[:, 2].min())}
    blades = [out["lo"], out["hi"]]
    return blades, out.get("pin")


def ground_hole(shell_tm: trimesh.Trimesh) -> dict:
    """The D-shaped ground hole in the face plate (~5 x 5 mm)."""
    ztop = shell_tm.bounds[1][2]
    sec = shell_tm.section(plane_origin=[0, 0, ztop - 0.0015], plane_normal=[0, 0, 1])
    p2, T = sec.to_2D()
    best = None
    for poly in p2.polygons_full:
        for hole in poly.interiors:
            pts = trimesh.transformations.transform_points(
                np.column_stack([np.array(hole.coords), np.zeros(len(hole.coords))]), T)
            lo, hi = pts.min(0), pts.max(0)
            ext = (hi - lo) * 1000
            if 4.0 < ext[0] < 6.0 and 4.0 < ext[1] < 6.0:
                best = {"center": (lo + hi) / 2, "size_mm": ext}
    assert best is not None, "ground hole not found"
    return best


def pose_block(tm: trimesh.Trimesh, shell_tm: trimesh.Trimesh, slots: list[dict],
               spec: BlockSpec) -> tuple[trimesh.Trimesh, dict]:
    """Kit block -> world: yaw about the blade midpoint, blade midpoint onto
    the slot midpoint, blades centred in their slots, lowest tip APPROACH
    above the face.  Returns the posed mesh and a geometry report."""
    posed = tm.copy()
    posed.apply_transform(trimesh.transformations.rotation_matrix(
        np.radians(spec.yaw_deg), [0, 0, 1]))
    face_z = float(shell_tm.bounds[1][2])
    s_lo = min(slots[:2], key=lambda s: s["center"][1])
    s_hi = max(slots[:2], key=lambda s: s["center"][1])
    slot_mid = ((np.asarray(s_lo["center"]) + np.asarray(s_hi["center"])) / 2)[:2]
    tip_z = float(posed.bounds[0][2])
    posed.apply_translation([slot_mid[0], slot_mid[1], face_z + APPROACH - tip_z])
    base_z = face_z + APPROACH - tip_z          # kit z = 0 (prong base plane)
    blades, pin = blade_footprints(posed, base_z, slot_mid)
    # polarization: each blade must fit its slot's width (x) with play
    for b, s in zip(blades, (s_lo, s_hi)):
        w_slot = float(s["size_mm"][0]) / 1000
        assert b["ext"][0] < w_slot - 0.0002, (
            f"block {spec.n}: {b['ext'][0]*1000:.2f} mm blade in a {w_slot*1000:.2f} mm "
            f"slot -- wrong yaw")
    # max-min clearance over both blades (as plug_insert_arm.center_in_slots)
    d = np.zeros(2)
    for ax in (0, 1):
        lo_allow, hi_allow = -np.inf, np.inf
        for b, s in zip(blades, (s_lo, s_hi)):
            half = float(s["size_mm"][ax]) / 2000.0
            lo_allow = max(lo_allow, (s["center"][ax] - half) - b["lo"][ax])
            hi_allow = min(hi_allow, (s["center"][ax] + half) - b["hi"][ax])
        assert lo_allow <= hi_allow, f"block {spec.n}: blades do not fit the slots (axis {ax})"
        d[ax] = (lo_allow + hi_allow) / 2
    posed.apply_translation([d[0], d[1], 0.0])
    blades, pin = blade_footprints(posed, base_z, slot_mid + d)
    rep = {"yaw_deg": spec.yaw_deg, "centring_shift_mm": (d * 1000).round(3).tolist(),
           "blade_play_mm": []}
    for b, s in zip(blades, (s_lo, s_hi)):
        half = np.asarray(s["size_mm"][:2]) / 2000.0
        play = np.minimum(b["lo"] - (np.asarray(s["center"][:2]) - half),
                          (np.asarray(s["center"][:2]) + half) - b["hi"])
        rep["blade_play_mm"].append((play * 1000).round(3).tolist())
        assert play.min() > 0.05e-3, f"block {spec.n}: blade play {play*1000} mm"
    if pin is not None:
        gh = ground_hole(shell_tm)
        off = float(np.linalg.norm(pin["center"] - gh["center"][:2]))
        r_hole = min(gh["size_mm"][:2]) / 2000.0
        r_pin = max(pin["ext"]) / 2.0
        rep["ground_pin"] = {"pin_dia_mm": round(2 * r_pin * 1000, 2),
                             "hole_mm": np.round(gh["size_mm"][:2], 2).tolist(),
                             "centre_offset_mm": round(off * 1000, 3),
                             "radial_play_mm": round((r_hole - r_pin - off) * 1000, 3)}
        assert r_hole - r_pin - off > 0.1e-3, f"block {spec.n}: ground pin misses its hole"
    rep["base_z"] = base_z
    rep["slot_mid"] = (slot_mid + d).tolist()
    rep["blade_tip_gap_mm"] = round((min(b["tip_z"] for b in blades) - face_z) * 1000, 3)
    return posed, rep


def drop_to_contact(plug: trimesh.Trimesh, shell: trimesh.Trimesh,
                    lo: float = 0.0, hi: float = 0.045):
    """Largest straight-down travel before the plug touches the shell:
    bisection on max signed depth, BOTH directions (plug samples into the
    shell and shell samples into the plug), on 1 mm-subdivided surfaces.

    Returns (travel, signed depth at that travel (<= 0), the start-pose
    plug points that touch 0.05 mm further down).  Cached on disk (keyed
    by both meshes' vertices) next to the SDF cache: it takes 1-3 min."""
    from force_bench.paths import SDF_CACHE
    h = hashlib.sha256()
    for m in (plug, shell):
        h.update(np.ascontiguousarray(m.vertices, dtype=np.float64).tobytes())
        h.update(np.ascontiguousarray(m.faces, dtype=np.int64).tobytes())
    h.update(np.array([lo, hi]).tobytes())
    cache = Path(SDF_CACHE) / f"charger_drop_{h.hexdigest()[:24]}.npz"
    if cache.exists():
        d = np.load(cache)
        return float(d["dz"]), float(d["sd"]), d["first"]
    res = _drop_to_contact(plug, shell, lo, hi)
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez(cache, dz=res[0], sd=res[1], first=res[2])
    return res


def _drop_to_contact(plug, shell, lo, hi):
    from force_bench.geometry import subdiv_processed as subdiv
    rs = np.random.RandomState(0)
    pv = np.asarray(subdiv(plug, 0.001).vertices)
    pv = pv[pv[:, 2] < plug.bounds[0][2] + 0.030]
    if len(pv) > 12000:
        pv = pv[rs.choice(len(pv), 12000, replace=False)]
    sv = np.asarray(subdiv(shell, 0.001).vertices)
    sv = sv[sv[:, 2] > shell.bounds[1][2] - 0.025]
    if len(sv) > 12000:
        sv = sv[rs.choice(len(sv), 12000, replace=False)]
    pq_s = trimesh.proximity.ProximityQuery(shell)
    pq_p = trimesh.proximity.ProximityQuery(plug)

    def depth(dz, where=False):
        a = pq_s.signed_distance(pv - [0, 0, dz])
        b = pq_p.signed_distance(sv + [0, 0, dz])
        if not where:
            return float(max(a.max(), b.max()))
        # world points (at the START pose of the plug) that touch first
        return np.vstack([pv[a > 0], sv[b > 0] + [0, 0, dz]])

    assert depth(lo) < 0.0, "plug already intersects the outlet at the start pose"
    assert depth(hi) > 0.0, "no contact within the search range"
    for _ in range(22):
        mid = 0.5 * (lo + hi)
        if depth(mid) > 0.0:
            hi = mid
        else:
            lo = mid
    first = depth(lo + 0.00005, where=True)
    return lo, depth(lo), first


# ── the variant ──────────────────────────────────────────────────────────
# Camera (all four blocks): low front-left, looking along the brick so the
# prongs, the slots and the landing are in view under the hand -- chosen from
# rendered previews of the start and seated poses of blocks 1 (tallest), 3
# and 4 (smallest); the suite's front-right view hid the plug behind the hand.
CAM_AZ_DEG = -150.0        # camera azimuth about the slot midpoint (0 = +x)
CAM_EL_DEG = 15.0
CAM_DIST = 0.25
CAM_TARGET_DZ = 0.015      # look-at height above the face plate

# ENTRY TRIM.  The suite commands the TCP straight over the slots, and the
# 0.35 s grip pre-roll then lets the arm settle under gravity: measured on
# blocks 2/3/4 alike, the gripped plug ends (+0.23..0.31, +0.84..0.86) mm off
# its slot-centred pose, so its 2 mm blades (0.10 mm play per side in y) land
# on the slot edges and the slot chamfer shoves the plug 0.85 mm sideways
# through the pinch at first contact.  The suite plug shrugs that off; on the
# lighter kit blocks it cost up to 0.66 mm of grasp slip in one frame.  The
# trim is added to every DRIVE command (the pre-roll's, the expert's and the
# ending's; not the IK start pose, so the fingers still close centred on the
# plug), so the settled plug starts centred; the expert's xy servo is
# referenced to the post-pre-roll wrist and is unaffected.
ENTRY_TRIM = (-0.00027, -0.00085)   # m (world x, y)
# FRICTION IMPEDANCE.  At MuJoCo's default impratio 1 the friction constraint
# is far softer than the normal one and a pinch creeps under steady load
# (force_bench.rig, "IMPRATIO IS THE GRASP KNOB"); the suite plug crept
# 0.25 mm over its stroke, kit block 2 crept 1.2 mm, and the suite expert's
# insertion command may lead the MEASURED plug depth by only 0.25 mm in its
# slow zone, so the creep froze the command 2.9 mm short of the seat.  10 is
# the value the other appendix grasp tasks (plug_in_usb, hang_keychain,
# hang_wrench) and the GelSight tasks use.
IMPRATIO = 10.0
_env_trim = os.environ.get("TB_CHARGER_TRIM")          # "0" disables (A/B only)
if _env_trim is not None:
    ENTRY_TRIM = (0.0, 0.0) if _env_trim == "0" else tuple(
        float(v) / 1000 for v in _env_trim.split(","))
IMPRATIO = float(os.environ.get("TB_CHARGER_IMPRATIO", IMPRATIO))   # A/B only


class ChargerBlockVariant(ChargerVariant):
    """plug_in_charger's ending on a kit block (see the module docstring)."""

    block: BlockSpec = None
    spring: float | None = None     # printed outlet spring indent [mm]; None = the suite's retention pads

    def configure(self, mod):
        names = ("pose_plug", "center_in_slots", "measure_seat", "TCP_ABOVE_BRICK_FACE",
                 "GRIP_START", "GRIP_TARGET", "GRIP_EXTRACT", "GRIP_FREE", "GRIP_RELEASE",
                 "PLUG_MASS", "T_INS_MAX", "ENTRY_ZONE", "APPROACH", "T_EXT_MAX",
                 "PRESS_LEAD", "PAD_PROTRUSION", "PAD_KE", "pad_wedge_tm")
        self._saved = {k: getattr(mod, k) for k in names}
        super().configure(mod)
        spec = self.block
        if self.spring is not None:
            # the kit outlet's printed spring (force_bench.kit.mechanisms.outlet): pad protrusion and rate of
            # the chosen spring, curved entry tangent to the grip face
            from force_bench.kit.mechanisms import outlet as kit_outlet
            prot, ke, src = kit_outlet.spring_params(self.spring)
            mod.PAD_PROTRUSION, mod.PAD_KE, mod.pad_wedge_tm = prot, ke, kit_outlet.pad_profile_tm
        tm = load_block(spec.n)
        self.geom = {}
        geom = self.geom
        half = spec.grip_width / 2

        def pose_plug(plug_tm, shell_tm, slots, pr):
            posed, rep = pose_block(tm, shell_tm, slots, spec)
            geom.update(rep)
            return posed

        def center_in_slots(posed, slots):
            return np.asarray(geom["centring_shift_mm"]) / 1000.0   # done in pose_block

        def measure_seat(plug_posed, shell_tm):
            dz, sd_seat, first = drop_to_contact(plug_posed, shell_tm)
            face_z = float(shell_tm.bounds[1][2])
            base_z = geom["base_z"]
            geom.update(
                dz_seat_mm=round(dz * 1000, 3),
                prong_base_above_plate_at_seat_mm=round((base_z - dz - face_z) * 1000, 3),
                first_contact_mm={"n": int(len(first)),
                                  "xy_rel_slot_mid": ((first[:, :2].mean(0) - np.asarray(geom["slot_mid"]))
                                                      * 1000).round(2).tolist(),
                                  "z_rel_plate": round(float((first[:, 2].mean() - dz - face_z)
                                                             * 1000), 2)})
            return {
                "dz_seat": dz,
                "z_plateau": base_z,                      # prong base plane
                "plateau_proud_mm": spec.body_face_z * 1000,
                "blade_engage_mm": dz * 1000 - geom["blade_tip_gap_mm"],
                "sd_at_seat_mm": sd_seat * 1000,
            }

        mod.pose_plug = pose_plug
        mod.center_in_slots = center_in_slots
        mod.measure_seat = measure_seat
        mod.TCP_ABOVE_BRICK_FACE = spec.tips_above_body + TIP_BELOW_TCP
        mod.GRIP_START = half - 0.0005
        mod.GRIP_TARGET = half - 0.0025
        mod.GRIP_EXTRACT = half - 0.0040
        mod.GRIP_FREE = half - 0.0015
        mod.GRIP_RELEASE = half + 0.0025
        mod.PLUG_MASS = block_mass(tm)
        mod.T_INS_MAX = 3.0
        mod.ENTRY_ZONE = spec.entry_zone
        mod.APPROACH = APPROACH
        geom["mass_kg"] = round(mod.PLUG_MASS, 4)
        if self.spring is not None:
            geom["outlet_spring"] = {"indent_mm": self.spring, "pad_protrusion_mm": round(mod.PAD_PROTRUSION * 1e3, 3),
                                     "pad_ke_N_per_m": round(mod.PAD_KE, 1), "source": src}
        geom["finger_targets_mm"] = {"start": round(mod.GRIP_START * 1000, 2),
                                     "squeeze": round(mod.GRIP_TARGET * 1000, 2)}
        # camera: same framing for every block, looking at the slot midpoint
        self._orig_finalize = Rig.__dict__["finalize"]
        orig_finalize = self._orig_finalize

        def finalize(rig, probes, cam_pos, cam_target, cam2_pos=None, cam2_target=None,
                     record=True):
            if IMPRATIO > 0:
                # plug_insert_arm rebuilds its solver after finalize with
                # make_solver(model, iterations=30); make_solver picks the
                # model's solver options up from here
                rig.solver_options["impratio"] = IMPRATIO
            geom["impratio"] = IMPRATIO
            geom["entry_trim_mm"] = [round(v * 1000, 3) for v in ENTRY_TRIM]
            center = np.asarray(cam_target, float) - np.array([0.0, 0.0, 0.05])
            target = np.array([center[0] - 0.005, center[1],
                               float(rig.meshes["shell"][0].bounds[1][2]) + CAM_TARGET_DZ])
            az, el = np.radians(CAM_AZ_DEG), np.radians(CAM_EL_DEG)
            pos = target + CAM_DIST * np.array([np.cos(el) * np.cos(az),
                                                np.cos(el) * np.sin(az), np.sin(el)])
            geom["camera"] = {"pos": pos.round(4).tolist(), "target": target.round(4).tolist()}
            return orig_finalize(rig, probes, pos, target, None, None, record)

        Rig.finalize = finalize
        # entry trim on every DRIVE command (pre-roll, expert, ending), not on
        # the start pose: the arm is built at the IK pose with the fingers
        # centred on the plug, and the pre-roll's trimmed command then
        # cancels the settle drift instead of the plug riding it
        self._orig_rig = {"drive_arm": Rig.__dict__["drive_arm"]}
        trim = np.array([ENTRY_TRIM[0], ENTRY_TRIM[1], 0.0])
        orig_drive = self._orig_rig["drive_arm"]

        def drive_arm(rig, tcp_pos, tcp_quat, fingers=0.04):
            return orig_drive(rig, np.asarray(tcp_pos, float) + trim, tcp_quat, fingers)

        Rig.drive_arm = drive_arm

    def cleanup(self, mod):
        for k, val in getattr(self, "_saved", {}).items():
            setattr(mod, k, val)
        if getattr(self, "_orig_finalize", None) is not None:
            Rig.finalize = self._orig_finalize
        for k, fn in getattr(self, "_orig_rig", {}).items():
            setattr(Rig, k, fn)

    def should_cut(self, rig, t):
        if "press_lead_mm" not in self.geom:
            # SEAT LATCH FRAME FIX.  On seating the suite expert sets its hand
            # command to `dz + PRESS_LEAD` ("back off to a gentle ~10 N
            # press"), but dz is the plug depth from its BUILT pose and so
            # already contains the pre-roll settle (0.67-0.82 mm here), while
            # the hand command is zero at the settled pose.  The latch thus
            # jumps the command ~0.5 mm INTO the seat: a 40-44 N landing spike
            # (suite plug: 24.7-27.2 N on its broad plateau, 0.05 mm overlap;
            # block 2: 44 N on its ground-pin tip, 0.252 mm into the blind
            # hole's floor).  Subtracting the settle measured at the first
            # frame gives the press the suite intended.
            ns = self.ns
            dz0 = float(ns["z0"]) - float(rig.s0.body_q.numpy()[ns["plug_i"], 2])
            self.mod.PRESS_LEAD = self._saved["PRESS_LEAD"] - dz0
            self.geom["preroll_settle_mm"] = round(dz0 * 1000, 3)
            self.geom["press_lead_mm"] = round(self.mod.PRESS_LEAD * 1000, 3)
        return super().should_cut(rig, t)

    def dynamic_total(self, duration):
        # a stiff spring may add the press-from-top phases (~10 s); the run stops when the ending is done
        return duration + (14.0 if self.spring is not None else 2.0)

    # ── stiff outlet springs: press the block home from its top ──────────────────────────────────────────────
    # The pinch cannot transmit the insertion force of the stiffer printed springs (0.7 mm and up: the block
    # stalls ~6 mm in, where the blades meet the pads).  A person then lets go and pushes the charger in by its
    # back; so does this: open, rise, close the fingers, come down onto the block's top and press with a slowly
    # growing command lead (the arm drives turn lead into force) until the block seats or the outlet pushes back
    # PRESS_F_MAX.  Then the usual push test, release and retreat follow from the pressing pose.
    PRESS_T = {"open": 0.4, "rise": 0.6, "close": 0.4}
    PRESS_RISE = 0.025
    PRESS_DESCEND_V = 0.015          # m/s down onto the block's top
    PRESS_TOUCH_N = 1.0              # robot<->block contact that ends the descent
    PRESS_RATE = 0.004               # m/s of extra lead while pressing
    PRESS_F_MAX = 80.0               # N of outlet reaction
    PRESS_HOLD = 0.4                 # s held at the seat before the push test

    def _press(self, rig, t, tau):
        """One frame of the press-from-top phases; returns True while they run."""
        P = self._pr
        log = self.ns["log"]
        dz = float(log[-1][1]) if log else 0.0
        env_z = float(log[-1][3]) if log else 0.0
        p0, q0, f0 = P["cut"]
        up = np.array([0.0, 0.0, 1.0])
        T = self.PRESS_T
        if P["stage"] == "open":
            u = ease(min(1.0, tau / T["open"]))
            rig.drive_arm(p0, q0, fingers=f0 + (OPEN_TO - f0) * u)
            if tau >= T["open"]:
                P.update(stage="rise", t0=tau)
            return True
        if P["stage"] == "rise":
            u = ease(min(1.0, (tau - P["t0"]) / T["rise"]))
            rig.drive_arm(p0 + up * self.PRESS_RISE * u, q0, fingers=OPEN_TO)
            if tau - P["t0"] >= T["rise"]:
                P.update(stage="close", t0=tau)
            return True
        if P["stage"] == "close":
            u = ease(min(1.0, (tau - P["t0"]) / T["close"]))
            rig.drive_arm(p0 + up * self.PRESS_RISE, q0, fingers=OPEN_TO + (0.0005 - OPEN_TO) * u)
            if tau - P["t0"] >= T["close"]:
                P.update(stage="descend", t0=tau, z=float(p0[2] + self.PRESS_RISE))
            return True
        touch = rig.probes["robot_object"].history[-1]["force"] if rig.probes["robot_object"].history else 0.0
        if P["stage"] == "descend":
            P["z"] -= self.PRESS_DESCEND_V / 60.0
            rig.drive_arm(np.array([p0[0], p0[1], P["z"]]), q0, fingers=0.0005)
            if touch > self.PRESS_TOUCH_N or tau - P["t0"] > 3.0:
                P.update(stage="press", t0=tau, z_touch=P["z"], lead=0.0, f_peak=0.0)
                self.geom["press_from_top"] = {"touch_at_depth_mm": round(dz * 1000, 3)}
            return True
        if P["stage"] == "press":
            seated = dz >= self._seat_dz - 0.0002
            P["f_peak"] = max(P["f_peak"], env_z)
            if not seated and env_z < self.PRESS_F_MAX:
                P["lead"] += self.PRESS_RATE / 60.0
            rig.drive_arm(np.array([p0[0], p0[1], P["z_touch"] - P["lead"]]), q0, fingers=0.0005)
            if seated or env_z >= self.PRESS_F_MAX or tau - P["t0"] > 6.0:
                P.update(stage="hold", t0=tau)
                self.geom["press_from_top"].update(seated=bool(seated), depth_mm=round(dz * 1000, 3),
                                                   outlet_reaction_peak_N=round(P["f_peak"], 2),
                                                   lead_mm=round(P["lead"] * 1000, 3))
            return True
        if P["stage"] == "hold":
            rig.drive_arm(np.array([p0[0], p0[1], P["z_touch"] - P["lead"]]), q0, fingers=0.0005)
            if tau - P["t0"] >= self.PRESS_HOLD:
                # the push test, release and retreat start from the pressing pose
                self.start_cmd = (np.array([p0[0], p0[1], P["z_touch"] - P["lead"]]), q0, 0.0005)
                P.update(stage="done", tau_end=tau)
            return True
        return False

    def ending(self, rig, t, tau):
        if getattr(self, "_pr", None) is None:
            log = self.ns["log"]
            dz = float(log[-1][1]) if log else 0.0
            self._seat_dz = float(self.geom.get("dz_seat_mm", 0.0)) / 1000.0
            need = self.spring is not None and dz < self._seat_dz - 0.001
            self._pr = {"stage": "open" if need else "done", "cut": self.start_cmd, "tau_end": 0.0}
            if need:
                print(f"[appendix] {self.name}: stalled at {dz*1000:.2f} of {self._seat_dz*1000:.2f} mm "
                      "-> pressing the block home from its top")
        if self._pr["stage"] != "done" and self._press(rig, t, tau):
            return
        tau -= self._pr["tau_end"]
        if tau >= self.t_push + self.t_open + self.t_retreat + self.t_still and self.done_at is None:
            self.done_at = t + 1.0 / 60.0
        super().ending(rig, t, tau)

    def released_window_start(self):
        return super().released_window_start() + (self._pr["tau_end"] if getattr(self, "_pr", None) else 0.0)

    def push_start(self):
        return self.t_cut_eff + (self._pr["tau_end"] if getattr(self, "_pr", None) else 0.0)

    def criteria(self, rig, qa):
        crit = super().criteria(rig, qa)
        g = self.geom
        fi = crit["fully_inserted"]
        gap = g.get("prong_base_above_plate_at_seat_mm", 0.0)
        fi["stop_feature"] = ("ground-pin tip on the floor of the 18.5 mm blind ground hole"
                              if gap > 0.05 else
                              ("prong plateau on the face plate" if self.block.body_face_z > 0
                               else "body face on the face plate"))
        fi["body_face_above_plate_at_seat_mm"] = round(gap + self.block.body_face_z * 1000, 3)
        return crit


DEFAULT_SPRING = 0.4     # mm indent: the softest printed outlet spring (measured); "more force" rows use stiffer ones


def make_variant(n: int, spring: float | None = DEFAULT_SPRING) -> ChargerBlockVariant:
    spec = BLOCKS[n]
    v = ChargerBlockVariant(
        name=f"plug_in_charger_block{n}" + (f"_spring_{spring:g}mm" if spring not in (None, DEFAULT_SPRING) else ""),
        source="force_bench.tasks.plug_insert_arm",
        appendix_task=f"Plug in charger (Charging block {n})",
        spec=SUITE_PLUG_VARIANT.spec,
        objects=SUITE_PLUG_VARIANT.objects,
        static_shapes=SUITE_PLUG_VARIANT.static_shapes,
        changes=[
            f"plug = finalized-kit charging_block_{n} ({spec.body}; {spec.prongs}), "
            f"assets/bodies_kit/charging_block_{n}/, instead of the suite plug",
            f"pose: {spec.why_yaw}; blades centred in their slots; lowest prong tip "
            f"{APPROACH*1000:.0f} mm above the face plate (in hand at the start)",
            f"grasp: {spec.why_grasp}",
            "seat = vertical drop distance to first contact (signed-distance bisection, "
            "both directions)",
            "mass = suite plug's 60 g scaled by printed volume",
            f"entry trim ({ENTRY_TRIM[0]*1000:+.2f}, {ENTRY_TRIM[1]*1000:+.2f}) mm on every "
            "TCP drive command: cancels the arm's measured settle drift so the prongs "
            "enter centred",
            f"MuJoCo impratio {IMPRATIO:g} (suite default 1): stops the pinch creeping "
            "through the stroke, which froze the suite expert short of the seat on the "
            "lighter kit blocks",
            "seat-latch press = suite PRESS_LEAD minus the measured pre-roll settle (the "
            "expert's latch adds the plug depth, which contains the settle, to the hand "
            "command and landed at 40-44 N instead of the intended ~10 N)",
            "camera: low front-left view of the face plate (same for all four blocks)",
            *SUITE_PLUG_VARIANT.changes,
        ],
        call_original_after_cut=False,
        retreat_m=0.06, t_open=0.6, t_retreat=1.4, t_still=1.0,
    )
    v.block = spec
    if spring is not None:
        from force_bench.kit.mechanisms import outlet as kit_outlet
        prot, ke, src = kit_outlet.spring_params(spring)
        v.spring = spring
        v.changes.insert(0, f"outlet spring = the kit's {spring:g} mm indent spring: retention pads "
                            f"{prot*1e3:.2f} mm proud at {ke/1e3:.1f} kN/m with a curved entry ({src}; "
                            "force_bench.kit.mechanisms.outlet), instead of the suite's 1.8 mm / 3 kN/m wedge")
    return v


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--block", type=int, choices=sorted(BLOCKS))
    ap.add_argument("--export-meshes", action="store_true")
    ap.add_argument("--spring", type=float, default=DEFAULT_SPRING,
                    help="outlet spring indent in mm: 0.4 (default), 0.7, 0.85 or 1")
    a = ap.parse_args(argv)
    if a.export_meshes:
        export_kit_meshes()
        if a.block is None:
            return None
    if a.block is None:
        ap.error("--block N is required")
    v = make_variant(a.block, a.spring)
    verdict = run_variant(v)
    print(json.dumps({"block": a.block, "geometry": v.geom}, indent=1, default=str))
    if verdict is not None:
        out = Path(v.rig.out_dir) / "block_geometry.json"
        out.write_text(json.dumps(v.geom, indent=1, default=str))
    return verdict


if __name__ == "__main__":
    import warp as wp
    wp.init()
    main()
