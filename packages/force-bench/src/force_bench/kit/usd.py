"""Kit assets as self-contained USD stages for NVIDIA Newton, and the helper that adds them to any scene.

    force-bench kit export-usd outlet          # every variant -> outputs/kit_usd/outlet/<variant>.usda
    force-bench kit check --from-usd outlet    # import each stage and run kit check's probe on it

    from force_bench.kit.usd import add_asset_usd
    h = add_asset_usd(builder, "outputs/kit_usd/outlet/0.4mm_indent_spring.usda",
                      xform=wp.transform((0.45, 0.0, 0.0), wp.quat_identity()))
    model = builder.finalize()
    h.prepare_model(model)          # the calibration's contact stiffness, damping and condim, on every shape
    solver = newton.solvers.SolverMuJoCo(model, use_mujoco_contacts=False, **h.solver_options())
    h.after_solver(model, solver)   # re-applies the asset's contact settings; warns where the solver differs

EXPORT.  ``export_variant`` builds the asset exactly as ``force_bench.kit.add_asset`` does (in a scratch scene, its
signed-distance fields deferred), then writes what the builder holds: every body (pose, mass, centre of mass and full
inertia), every shape (mesh, box or convex hull; friction, contact stiffness, contact margin and gap, SDF settings,
colour), every joint (frames, axis, limits, drive, armature, dry friction, MuJoCo limit and friction impedance),
the gear couplings (MuJoCo joint equalities, Newton mimic constraints), articulation roots and collision filters.
A free joint has no prim (a UsdPhysics body without a joint to its parent is free; the record keys the joint by its
body, ``free:<body path>``); a floating base with joints below it (the Ethernet plug and its hinged latch tab) has
PhysicsArticulationRootAPI on its body, from which Newton's importer makes the free joint and the joints below it one
articulation, as ``add_asset`` does.
The stage stands on z = 0, centred on the origin, metres and kilograms, Z up, no PhysicsScene (it is meant to be
added to someone else's scene).  Standard UsdPhysics underneath; Newton's own schema attributes (``newton:*``,
NewtonSDFCollisionAPI, NewtonMimicAPI ...) and MuJoCo's (``mjc:*``) on top.

What USD has no attribute for is written under ``contactbench:`` and applied by ``add_asset_usd``:

* ``contactbench:jointForce``    constant generalised force on a joint (the doorknob's spring preload);
* ``contactbench:actuatorMode``  Newton's JointTargetMode where the USD drive gains imply another one (a POSITION
                                  drive with zero stiffness reads back as VELOCITY, see ``JointTargetMode.from_gains``);
* ``contactbench:sdfMaxResolution``  the exact SDF resolution (Newton's importer only takes multiples of 8; the
                                  standard attribute holds the next multiple of 8);
* ``contactbench:solverContactStiffness``  a contact stiffness the mechanism sets after the solver is built
                                  (the USB slot's jaws, ``usb_slot.stiffen_jaw_contacts``);
* the root prim's ``customData["contactbench"]``: asset, variant, the CAD pose, the mechanism's parameters and
  measured geometry (``handle``), the solver settings the calibration assumes (``calibration``), and an exact record
  of every value the builder held (``record``), which the helper uses to verify the import and to restore what
  Newton's importer rounds (degrees and per-degree gains in 32-bit floats) or drops (articulations of a single
  world-anchored joint, which UsdPhysics does not list).

IMPORT.  ``add_asset_usd(builder_or_rig, path, xform=..., pose=..., label=...)`` registers SolverMuJoCo's custom
attributes, calls ``builder.add_usd``, applies the ``contactbench:`` extras and returns a ``UsdAssetHandle`` (an
``AssetHandle`` with the same roles, parameters and info as ``add_asset`` gives, indices remapped).  Given a
``force_bench.rig.Rig`` it also makes the scene-level settings ``add_asset`` makes (hard joint limits and impratio
for the dials).  ``handle.prepare_model(model)`` gives every shape of the finalized scene the contact stiffness,
damping and condim the calibration ran with (as ``force_bench.rig.make_solver`` does), ``handle.after_solver(model,
solver)`` re-applies the asset's own after a solver that overwrote them and warns where the scene's solver differs
from the calibration, ``handle.check_scene(...)`` checks the timestep and collision cadence, and
``handle.apply_preload(control)`` restores constant joint forces into a ``Control`` whose joint_f you zeroed.

Other simulators read the UsdPhysics part only: PhysX (Isaac Sim / Isaac Lab) ignores every ``newton:``, ``mjc:``
and ``contactbench:`` attribute, has no signed-distance collision unless asked for, and has no gear coupling, so the
forces of the mechanisms differ there; the calibration was done against MuJoCo Warp with Newton's contacts
(docs/custom_scenes.md).
"""

from __future__ import annotations

import json
import math
import re
import warnings
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import trimesh

FORMAT = "contactbench-kit-usd/1"
DEG = 180.0 / math.pi                 # degrees per radian (UsdPhysics angles, per-degree gains)
UNLIMITED = 1.0e10                    # Newton's "no limit" (JointDofConfig default)
F32 = 2.0e-6                          # relative tolerance of a value that went through a 32-bit float
MIMIC_KIND = 2                        # newton MjcEqualityTargetKind.MIMIC
FREE_REF = "free:"                    # record key of a body's free joint (no prim): "free:<body prim path>"


# ── small transforms ──────────────────────────────────────────────────────────────────────────────────────────────
def _mat(t) -> np.ndarray:
    """4x4 from a warp transform / 7-sequence (px py pz qx qy qz qw) or a 4x4."""
    a = np.asarray(t, dtype=float)
    if a.shape == (4, 4):
        return a.copy()
    a = a.reshape(-1)
    M = trimesh.transformations.quaternion_matrix([a[6], a[3], a[4], a[5]])
    M[:3, 3] = a[:3]
    return M


def _seven(M: np.ndarray) -> list[float]:
    q = trimesh.transformations.quaternion_from_matrix(M)      # w x y z
    if q[0] < 0:
        q = -q
    return [float(M[0, 3]), float(M[1, 3]), float(M[2, 3]), float(q[1]), float(q[2]), float(q[3]), float(q[0])]


def _wp(M: np.ndarray):
    import warp as wp
    s = _seven(M)
    return wp.transform(wp.vec3(*s[:3]), wp.quat(*s[3:]))


def _srgb_to_linear(c: float) -> float:
    c = float(c)
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _list(x) -> list[float]:
    return [float(v) for v in np.asarray(x, dtype=float).reshape(-1)]


# the 24 proper rotations that permute the coordinate axes (identity first)
def _axis_rotations() -> list[np.ndarray]:
    import itertools
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1.0, -1.0), repeat=3):
            R = np.zeros((3, 3))
            for i, (p, s) in enumerate(zip(perm, signs)):
                R[i, p] = s
            if np.linalg.det(R) > 0:
                out.append(R)
    out.sort(key=lambda R: (not np.allclose(R, np.eye(3)), -np.trace(R)))
    return out


_AXIS_ROTS = _axis_rotations()


def _joint_frame(axes: list[np.ndarray]) -> tuple[np.ndarray, list[str]]:
    """A rotation Rf of the joint frame such that every dof axis is +X, +Y or +Z of the rotated frame (UsdPhysics
    axes are tokens).  Returns (Rf, tokens).  A single axis along -X/-Y/-Z keeps its letter (half a turn about the
    next frame axis)."""
    if len(axes) == 1:
        a = np.asarray(axes[0], float) / np.linalg.norm(axes[0])
        k = int(np.argmax(np.abs(a)))
        if abs(a[k] + 1.0) < 1e-6:
            n = np.zeros(3)
            n[(k + 1) % 3] = 1.0
            return trimesh.transformations.rotation_matrix(math.pi, n)[:3, :3].round(12), ["XYZ"[k]]
    for R in _AXIS_ROTS:
        toks = []
        for a in axes:
            v = R.T @ (np.asarray(a, float) / np.linalg.norm(a))
            k = int(np.argmax(np.abs(v)))
            if abs(v[k] - 1.0) > 1e-6:
                break
            toks.append("XYZ"[k])
        else:
            return R, toks
    if len(axes) == 1:                       # an oblique single axis: shortest arc from +X
        a = np.asarray(axes[0], float) / np.linalg.norm(axes[0])
        R = trimesh.geometry.align_vectors([1.0, 0.0, 0.0], a)[:3, :3]
        return R, ["X"]
    raise NotImplementedError(f"joint axes {axes} are not a permutation of the frame axes")


def _prim_name(s: str) -> str:
    s = s.replace("+", "pos").replace("-", "neg") if re.fullmatch(r".*[+-][xyz]", s or "") else s
    s = re.sub(r"[^A-Za-z0-9_]", "_", s or "x")
    return s if re.match(r"[A-Za-z_]", s) else "_" + s


def _unique(name: str, taken: set) -> str:
    n, i = name, 1
    while n in taken:
        n, i = f"{name}_{i}", i + 1
    taken.add(n)
    return n


# ── the calibration the stage assumes ─────────────────────────────────────────────────────────────────────────────
PROBE_COLLISION = {   # kit check's collision cadence during each mechanism's probe (the settle collides every substep)
    "outlet": "every substep", "usb_slot": "every substep", "dial": "every substep",
    "doorknob_module": "once per frame", "lock_module": "once per frame",
    "door": "every substep for the first 0.25 s, then once per frame", "ethernet": "every substep",
}


def calibration_settings(rig, model_name: str | None = None) -> dict:
    """The solver and scene settings the kit's probes and tasks run with (force_bench.rig: Rig + make_solver)."""
    from force_bench import rig as R
    opts = dict(R._MJ_OPTS)
    opts.update(rig.solver_options)
    if R.MJ_IMPRATIO is not None:
        opts["impratio"] = R.MJ_IMPRATIO
    g = [float(v) for v in rig.builder.gravity] if hasattr(rig.builder.gravity, "__len__") else \
        [0.0, 0.0, float(rig.builder.gravity)]
    out = {
        "solver": "newton.solvers.SolverMuJoCo (MuJoCo Warp)", "use_mujoco_contacts": False,
        "contacts": "Newton CollisionPipeline (SDF and mesh contacts), broad_phase sap, reduce_contacts, "
                    "rigid_contact_max 20000",
        "timestep": 1.0 / (rig.fps * rig.substeps), "fps": int(rig.fps), "substeps": int(rig.substeps),
        "iterations": 15, "probe_iterations": 24, "ls_iterations": int(opts.get("ls_iterations", 100)),
        "mj_solver": str(opts.get("solver")), "integrator": str(opts.get("integrator")), "cone": str(opts.get("cone")),
        "impratio": float(opts.get("impratio", 1.0)), "nconmax": int(R.MJ_NCONMAX), "njmax": int(R.MJ_NJMAX),
        "contact_stiffness": float(R.MJ_CONTACT_KE), "contact_damping": float(R.MJ_CONTACT_KD),
        "condim": int(R.MJ_CONDIM) if R.MJ_CONDIM is not None else 3,
        "hard_joint_limits": bool(rig.hard_joint_limits), "limit_dampratio": float(rig.limit_dampratio),
        "collide_every_substep": True, "gravity": g,
    }
    if model_name:
        out["probe_collision"] = PROBE_COLLISION.get(model_name, "every substep")
    return out


# ── export ────────────────────────────────────────────────────────────────────────────────────────────────────────
def _custom(b, key):
    a = b.custom_attributes.get(key)
    return a


def _eq_rows_for_mimic(b) -> dict[int, int]:
    """mimic index -> MuJoCo equality row (the row mjc_add_equality_mimic wrote for it)."""
    kind = _custom(b, "mujoco:equality_constraint_target_kind")
    tgt = _custom(b, "mujoco:equality_constraint_target")
    if kind is None or tgt is None or not kind.values:
        return {}
    out = {}
    for r, (k, t) in enumerate(zip(kind.values, tgt.values)):
        if int(k) == MIMIC_KIND:
            out[int(t)] = r
    return out


def _snapshot(rig) -> dict:
    b = rig.builder
    return dict(body=b.body_count, shape=b.shape_count, joint=b.joint_count, art=b.articulation_count,
                mimic=len(b.constraint_mimic_joint0), filters=set(b.shape_collision_filter_pairs),
                rig_shapes={k: list(v) for k, v in rig.shapes.items()}, joint_of=dict(rig._joint_of),
                solver_options=dict(rig.solver_options), hard=bool(rig.hard_joint_limits),
                joint_f=[float(v) for v in b.joint_f])


def build_for_export(key: str, choice=None, pose=None, label: str | None = None, **params):
    """Build kit asset ``key`` as ``add_asset`` does, in a scratch Rig, SDF bakes deferred (no CUDA needed).
    Returns (rig, handle, snapshot taken before the build)."""
    from force_bench.assets import defer_sdf_builds
    from force_bench.kit import add_asset, place_on_table
    from force_bench.rig import Rig
    rig = Rig("kit_usd_export")
    snap = _snapshot(rig)
    if pose is None:
        pose = place_on_table(key, choice)
    with defer_sdf_builds():
        h = add_asset(rig, key, choice, pose=pose, label=label, **params)
    return rig, h, snap


def _info_encode(obj, key, refs):
    """JSON-safe copy of handle.info: shape/body/joint indices become prim references, world-frame vectors are
    tagged (stage frame; the helper moves them with the asset)."""
    if isinstance(obj, dict):
        return {k: _info_encode(v, k, refs) for k, v in obj.items()}
    if key.endswith("_shapes") or key == "pocket_shapes":
        if isinstance(obj, (list, tuple)):
            return [{"$ref": "shape", "path": refs["shape"][int(s)]} for s in obj]
    if key == "joints_in_order":
        return [{"$ref": "joint", "path": refs["joint"][int(j)]} for j in obj]
    if key == "parent" and isinstance(obj, (int, np.integer)):
        return {"$ref": "body", "path": refs["body"].get(int(obj))}
    if key.endswith("_world") and isinstance(obj, (list, tuple, np.ndarray)) and len(obj) == 3:
        kind = "dir" if ("axis" in key or "dir" in key) else "point"
        return {"$vec": kind, "v": _list(obj)}
    if isinstance(obj, (list, tuple)):
        return [_info_encode(v, key, refs) for v in obj]
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (np.floating,)):
        return float(obj)
    if isinstance(obj, (np.integer,)):
        return int(obj)
    return obj


def _handle_encode(h, refs) -> dict:
    out = {"key": h.key, "label": h.label, "choice": h.choice, "cad_pose": np.asarray(h.pose, float).tolist(),
           "bodies": {r: refs["body"][int(b)] for r, b in h.bodies.items()},
           "joints": {r: refs["joint"][int(j)] for r, j in h.joints.items()},
           "shapes": {n: [refs["shape"][int(s)] for s in sids] for n, sids in h.shapes.items()},
           "params": json.loads(json.dumps(h.params, default=_json_default)),
           "info": _info_encode(h.info, "", refs)}
    mods = getattr(h, "_modules", None)
    if mods:
        out["modules"] = {k: _handle_encode(m, refs) for k, m in mods.items()}
    return out


def _json_default(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating,)):
        return float(o)
    if isinstance(o, (np.integer,)):
        return int(o)
    return str(o)


def collect(rig, h, snap) -> dict:
    """Everything the asset put into ``rig.builder`` (indices relative to the snapshot), with prim paths."""
    import newton
    from force_bench.assets import SDF_PARAMS_ATTR
    from force_bench.kit import load_kit
    b = rig.builder
    JT, GT = newton.JointType, newton.GeoType
    asset = load_kit()[h.key]
    root = "/" + _prim_name(h.label)
    bodies = list(range(snap["body"], b.body_count))
    shapes = list(range(snap["shape"], b.shape_count))
    joints = list(range(snap["joint"], b.joint_count))
    # ── prim paths ──
    taken_root = {"Static", "Joints", "PhysicsMaterials"}
    body_path, joint_path, shape_path = {}, {}, {}
    role_of_body = {int(v): k for k, v in h.bodies.items()}
    for bi in bodies:
        role = role_of_body.get(bi)
        name = role if role else (h.label if role == "" else b.body_label[bi].split("/")[-1])
        body_path[bi] = f"{root}/{_unique(_prim_name(name or h.label), taken_root)}"
    role_of_joint = {int(v): k for k, v in h.joints.items()}
    taken_j = set()
    for ji in joints:
        if int(b.joint_type[ji]) == int(JT.FREE):
            # a free joint has no prim: UsdPhysics makes a body without a joint to its parent free (a floating base:
            # ArticulationRootAPI on the body), Newton's importer gives it a free joint; the record keys it by body
            c = int(b.joint_child[ji])
            if int(b.joint_parent[ji]) != -1 or c not in body_path:
                raise NotImplementedError(f"free joint {b.joint_label[ji]}: only a free joint to the world is exported")
            joint_path[ji] = FREE_REF + body_path[c]
            continue
        name = role_of_joint.get(ji) or (b.joint_label[ji] or f"joint_{ji}").split("/")[-1]
        joint_path[ji] = f"{root}/Joints/{_unique(_prim_name(name), taken_j)}"
    taken_s: dict[str, set] = {}
    for si in shapes:
        bi = int(b.shape_body[si])
        parent = body_path[bi] if bi >= 0 else f"{root}/Static"
        lbl = b.shape_label[si] or f"shape_{si}"
        shape_path[si] = f"{parent}/{_unique(_prim_name(lbl.split('/')[-1]), taken_s.setdefault(parent, set()))}"
    refs = {"body": {**body_path, -1: None}, "joint": joint_path, "shape": shape_path}

    # ── bodies at zero joint coordinates (Newton's importer anchors a world joint at its body's authored pose) ──
    pose0 = {-1: np.eye(4)}
    moved = set()
    for bi in bodies:
        pose0[bi] = _mat(b.body_q[bi])
    for ji in joints:
        if int(b.joint_type[ji]) == int(JT.FREE):
            continue
        p, c = int(b.joint_parent[ji]), int(b.joint_child[ji])
        q0 = b.joint_q_start[ji]
        nq = sum(b.joint_dof_dim[ji])
        if any(abs(float(x)) > 0.0 for x in b.joint_q[q0:q0 + nq]) or p in moved:
            pose0[c] = pose0[p] @ _mat(b.joint_X_p[ji]) @ np.linalg.inv(_mat(b.joint_X_c[ji]))
            moved.add(c)

    rec = {"root": root, "bodies": {}, "joints": {}, "free_joints": {}, "shapes": {}, "mimic": [], "filters": [],
           "articulations": [], "zero_pose_bodies": [body_path[c] for c in sorted(moved)]}
    for bi in bodies:
        inertia = np.array(b.body_inertia[bi], dtype=float).reshape(3, 3)
        rec["bodies"][body_path[bi]] = {
            "label": b.body_label[bi], "q": _list(b.body_q[bi]), "q_usd": _seven(pose0[bi]),
            "mass": float(b.body_mass[bi]), "com": _list(b.body_com[bi]), "inertia": inertia.reshape(-1).tolist(),
            "inv_mass": float(b.body_inv_mass[bi]),
            "inv_inertia": np.array(b.body_inv_inertia[bi], dtype=float).reshape(-1).tolist()}

    # ── shapes ──
    overrides = {}
    if asset.sim.get("kind") == "mechanism":
        from force_bench.kit.mechanisms import module_for
        for role, ke in getattr(module_for(asset), "SOLVER_SHAPE_KE", {}).items():
            for s in h.shapes.get(role, [])[:1]:
                overrides[int(s)] = float(ke)
    from force_bench import rig as R
    cal_ke, cal_kd = float(R.MJ_CONTACT_KE), float(R.MJ_CONTACT_KD)
    custom_shape = [a for a in b.custom_attributes.values()
                    if a.frequency == newton.Model.AttributeFrequency.SHAPE and a.namespace == "mujoco"]
    for si in shapes:
        flags = int(b.shape_flags[si])
        collide = bool(flags & int(newton.ShapeFlags.COLLIDE_SHAPES))
        st = int(b.shape_type[si])
        src = b.shape_source[si]
        d = {"label": b.shape_label[si], "body": refs["body"][int(b.shape_body[si])], "type": st,
             "X": _list(b.shape_transform[si]), "scale": _list(b.shape_scale[si]), "flags": flags,
             "collide": collide, "visible": bool(flags & int(newton.ShapeFlags.VISIBLE)),
             "mu": float(b.shape_material_mu[si]), "restitution": float(b.shape_material_restitution[si]),
             "mu_torsional": float(b.shape_material_mu_torsional[si]),
             "mu_rolling": float(b.shape_material_mu_rolling[si]),
             "ke": float(b.shape_material_ke[si]), "kd": float(b.shape_material_kd[si]),
             "kf": float(b.shape_material_kf[si]), "ka": float(b.shape_material_ka[si]),
             "ke_calibrated": overrides.get(si, cal_ke), "kd_calibrated": cal_kd,
             "solver_ke": overrides.get(si), "margin": float(b.shape_margin[si]), "gap": float(b.shape_gap[si]),
             "group": int(b.shape_collision_group[si]), "color": _list(b.shape_color[si])
             if b.shape_color[si] is not None else None,
             "sdf": dict(getattr(src, SDF_PARAMS_ATTR)) if src is not None and hasattr(src, SDF_PARAMS_ATTR) else None,
             "custom": {}}
        if d["sdf"]:
            d["sdf"]["narrow_band_range"] = list(d["sdf"]["narrow_band_range"])
        for a in custom_shape:
            if isinstance(a.values, dict) and si in a.values and a.usd_attribute_name not in (None, "*"):
                d["custom"][a.key] = {"usd": a.usd_attribute_name, "value": _list(a.values[si])
                                      if hasattr(a.values[si], "__len__") else float(a.values[si])}
        if st in (int(GT.MESH), int(GT.CONVEX_MESH)):
            d["mesh"] = {"vertices": np.asarray(src.vertices, dtype=np.float32),
                         "indices": np.asarray(src.indices, dtype=np.int32).reshape(-1)}
        elif st != int(GT.BOX):
            raise NotImplementedError(f"shape {b.shape_label[si]}: geometry type {st} is not exported")
        rec["shapes"][shape_path[si]] = d

    # ── joints ──
    custom_joint = [a for a in b.custom_attributes.values()
                    if a.frequency in (newton.Model.AttributeFrequency.JOINT_DOF, newton.Model.AttributeFrequency.JOINT)
                    and a.namespace == "mujoco"]
    for ji in joints:
        jt = int(b.joint_type[ji])
        free = jt == int(JT.FREE)
        if jt not in (int(JT.PRISMATIC), int(JT.REVOLUTE), int(JT.D6), int(JT.FIXED), int(JT.FREE)):
            raise NotImplementedError(f"joint {b.joint_label[ji]}: type {jt} is not exported")
        d0, q0 = b.joint_qd_start[ji], b.joint_q_start[ji]
        nl, na = b.joint_dof_dim[ji]
        dofs = []
        for k in range(nl + na):
            d = d0 + k
            dofs.append({"kind": "linear" if k < nl else "angular", "axis": _list(b.joint_axis[d]),
                         "mode": int(b.joint_target_mode[d]), "ke": float(b.joint_target_ke[d]),
                         "kd": float(b.joint_target_kd[d]), "target_qd": float(b.joint_target_qd[d]),
                         "lower": float(b.joint_limit_lower[d]),
                         "upper": float(b.joint_limit_upper[d]), "limit_ke": float(b.joint_limit_ke[d]),
                         "limit_kd": float(b.joint_limit_kd[d]), "armature": float(b.joint_armature[d]),
                         "friction": float(b.joint_friction[d]), "damping": float(b.joint_damping[d]),
                         "effort_limit": float(b.joint_effort_limit[d]),
                         "velocity_limit": float(b.joint_velocity_limit[d]),
                         "f": float(b.joint_f[d]) - (snap["joint_f"][d] if d < len(snap["joint_f"]) else 0.0),
                         "qd": float(b.joint_qd[d])})
            if not free:                   # one coordinate per dof (a free joint has 7: position, quaternion xyzw)
                dofs[-1].update(target_q=float(b.joint_target_q[q0 + k]), q=float(b.joint_q[q0 + k]))
        custom = {}
        for a in custom_joint:
            if not isinstance(a.values, dict):
                continue
            idx = [d0 + k for k in range(nl + na)] if a.frequency == newton.Model.AttributeFrequency.JOINT_DOF \
                else [ji]
            vals = [a.values.get(i) for i in idx]
            if all(v is None for v in vals):
                continue
            vals = [a.default if v is None else v for v in vals]
            custom[a.key] = {"usd": a.usd_attribute_name, "values": [
                _list(v) if hasattr(v, "__len__") else float(v) for v in vals]}
        p = int(b.joint_parent[ji])
        jr = {"label": b.joint_label[ji], "type": jt, "parent": refs["body"][p],
              "child": body_path[int(b.joint_child[ji])],
              "X_p": _list(b.joint_X_p[ji]), "X_c": _list(b.joint_X_c[ji]), "dofs": dofs, "custom": custom,
              "collision_filter_parent": bool(b.joint_collision_filter_parent[ji]),
              "articulation": int(b.joint_articulation[ji]) - snap["art"] if b.joint_articulation[ji] >= 0 else None,
              "enabled": bool(b.joint_enabled[ji])}
        if free:
            jr.update(q=[float(v) for v in b.joint_q[q0:q0 + 7]],
                      target_q=[float(v) for v in b.joint_target_q[q0:q0 + 7]])
            rec["free_joints"][joint_path[ji]] = jr
        else:
            rec["joints"][joint_path[ji]] = jr
    arts = {}
    for ji in joints:
        a = int(b.joint_articulation[ji])
        if a >= snap["art"] and ji in joint_path:
            arts.setdefault(a, []).append(joint_path[ji])
    rec["articulations"] = [arts[a] for a in sorted(arts)]

    # ── gear couplings ──
    eq_of = _eq_rows_for_mimic(b)
    for m in range(snap["mimic"], len(b.constraint_mimic_joint0)):
        j0, j1 = int(b.constraint_mimic_joint0[m]), int(b.constraint_mimic_joint1[m])
        if j0 not in joint_path or j1 not in joint_path:
            raise NotImplementedError(f"mimic {b.constraint_mimic_label[m]} couples a joint outside the asset")
        if joint_path[j0].startswith(FREE_REF) or joint_path[j1].startswith(FREE_REF):
            raise NotImplementedError(f"mimic {b.constraint_mimic_label[m]} couples a free joint")
        row = eq_of.get(m)
        eqc = {}
        if row is not None:
            for k in ("mujoco:eq_solref", "mujoco:eq_solimp"):
                a = _custom(b, k)
                if a is not None and a.values is not None and row < len(a.values):
                    eqc[k] = _list(a.values[row])
        rec["mimic"].append({"follower": joint_path[j0], "leader": joint_path[j1],
                             "coef0": float(b.constraint_mimic_coef0[m]), "coef1": float(b.constraint_mimic_coef1[m]),
                             "enabled": bool(b.constraint_mimic_enabled[m]), "label": b.constraint_mimic_label[m],
                             "custom": eqc})

    # ── collision filters between two colliding shapes of the asset ──
    for a, c in sorted(set(b.shape_collision_filter_pairs) - snap["filters"]):
        if b.shape_body[a] == b.shape_body[c]:
            continue                      # shapes of one body (or two static shapes): the builder filters them itself
        if a < snap["shape"] or c < snap["shape"]:
            raise NotImplementedError(f"filter pair ({a}, {c}) reaches outside the asset")
        if rec["shapes"][shape_path[a]]["collide"] and rec["shapes"][shape_path[c]]["collide"]:
            rec["filters"].append([shape_path[a], shape_path[c]])

    # ── what add_asset changed on the Rig ──
    rig_shapes = {}
    for k, v in rig.shapes.items():
        new = [s for s in v[len(snap["rig_shapes"].get(k, [])):] if s in shape_path]
        if new:
            rig_shapes[k] = [shape_path[s] for s in new]
    joint_of = {k: joint_path[v] for k, v in rig._joint_of.items() if k not in snap["joint_of"] and v in joint_path}
    solver_options = {k: v for k, v in rig.solver_options.items() if snap["solver_options"].get(k) != v}
    rec["rig"] = {"shapes": rig_shapes, "joint_of": joint_of, "solver_options": solver_options,
                  "hard_joint_limits": bool(rig.hard_joint_limits and not snap["hard"])}
    rec["handle"] = _handle_encode(h, refs)
    return rec


def _record_json(rec) -> str:
    slim = {k: v for k, v in rec.items() if k not in ("shapes", "handle")}
    slim["shapes"] = {p: {k: v for k, v in d.items() if k != "mesh"} for p, d in rec["shapes"].items()}
    return json.dumps(slim, default=_json_default, separators=(",", ":"))


def write_stage(rec: dict, out_path, meta: dict) -> Path:
    """Write the collected asset ``rec`` as a USD stage (``.usda`` text or ``.usdc`` binary by extension)."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
    import newton
    JT, GT = newton.JointType, newton.GeoType
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    if out_path.exists():
        out_path.unlink()
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, rec["root"])
    stage.SetDefaultPrim(root.GetPrim())
    Usd.ModelAPI(root.GetPrim()).SetKind("component")
    stage.GetRootLayer().documentation = (
        f"ContactBench kit asset {meta['asset']} ({meta['name']}), variant {meta['variant']}: exported by "
        f"force_bench.kit.usd for NVIDIA Newton (add with force_bench.kit.usd.add_asset_usd, or builder.add_usd). "
        f"Metres, kilograms, Z up, standing on z = 0.")

    def xform_ops(xf, T7, scale=None):
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*T7[:3]))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(T7[6], T7[3], T7[4], T7[5]))
        if scale is not None:
            xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))

    def attr(prim, name, vtype, value, custom=True):
        a = prim.CreateAttribute(name, vtype, custom)
        a.Set(value)
        return a

    def label(prim, text):
        if text:
            attr(prim, "contactbench:label", Sdf.ValueTypeNames.String, str(text))

    # physics materials: one per distinct contact property set
    UsdGeom.Scope.Define(stage, rec["root"] + "/PhysicsMaterials")
    mats = {}
    UsdGeom.Scope.Define(stage, rec["root"] + "/Looks")
    looks = {}

    def look(rgb):
        """A UsdPreviewSurface in the print colour (linear values, as USD expects; the kit's colours are sRGB)."""
        key = tuple(round(float(c), 6) for c in rgb)
        if key in looks:
            return looks[key]
        lin = [_srgb_to_linear(c) for c in key]
        m = UsdShade.Material.Define(stage, f"{rec['root']}/Looks/print_{len(looks)}")
        sh = UsdShade.Shader.Define(stage, f"{rec['root']}/Looks/print_{len(looks)}/surface")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*lin))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        looks[key] = m
        return m

    def material(d):
        key = (d["mu"], d["restitution"], d["mu_torsional"], d["mu_rolling"], d["ke_calibrated"], d["kd_calibrated"],
               d["kf"], d["ka"])
        if key in mats:
            return mats[key]
        m = UsdShade.Material.Define(stage, f"{rec['root']}/PhysicsMaterials/contact_{len(mats)}")
        p = m.GetPrim()
        api = UsdPhysics.MaterialAPI.Apply(p)
        api.CreateStaticFrictionAttr(float(d["mu"]))
        api.CreateDynamicFrictionAttr(float(d["mu"]))
        api.CreateRestitutionAttr(float(d["restitution"]))
        p.AddAppliedSchema("NewtonMaterialAPI")
        attr(p, "newton:torsionalFriction", Sdf.ValueTypeNames.Float, float(d["mu_torsional"]), False)
        attr(p, "newton:rollingFriction", Sdf.ValueTypeNames.Float, float(d["mu_rolling"]), False)
        attr(p, "newton:contactStiffness", Sdf.ValueTypeNames.Float, float(d["ke_calibrated"]), False)
        attr(p, "newton:contactDamping", Sdf.ValueTypeNames.Float, float(d["kd_calibrated"]), False)
        attr(p, "newton:contactFrictionGain", Sdf.ValueTypeNames.Float, float(d["kf"]), False)
        attr(p, "newton:contactAdhesion", Sdf.ValueTypeNames.Float, float(d["ka"]), False)
        mats[key] = m
        return m

    # bodies
    UsdGeom.Xform.Define(stage, rec["root"] + "/Static")
    for path, d in rec["bodies"].items():
        xf = UsdGeom.Xform.Define(stage, path)
        xform_ops(xf, d["q_usd"])
        p = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(p)
        mass = UsdPhysics.MassAPI.Apply(p)
        inertia = np.asarray(d["inertia"], float).reshape(3, 3)
        w, V = np.linalg.eigh(0.5 * (inertia + inertia.T))
        if np.linalg.det(V) < 0:
            V[:, 2] *= -1
        q = trimesh.transformations.quaternion_from_matrix(np.block([[V, np.zeros((3, 1))], [np.zeros((1, 3)), 1]]))
        mass.CreateMassAttr(float(d["mass"]))
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*d["com"]))
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*[max(float(v), 0.0) for v in w]))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        p.AddAppliedSchema("NewtonMassAPI")
        attr(p, "newton:inertia", Sdf.ValueTypeNames.DoubleArray,
             Vt.DoubleArray([inertia[0, 0], inertia[1, 1], inertia[2, 2], inertia[0, 1], inertia[0, 2],
                             inertia[1, 2]]), False)
        label(p, d["label"])

    # shapes
    for path, d in rec["shapes"].items():
        st = d["type"]
        T7 = d["X"]
        if st == int(GT.BOX):
            g = UsdGeom.Cube.Define(stage, path)
            g.CreateSizeAttr(2.0)
            xform_ops(g, T7, d["scale"])
        else:
            if not np.allclose(d["scale"], 1.0):
                raise NotImplementedError(f"{path}: scaled mesh shapes are not exported")
            g = UsdGeom.Mesh.Define(stage, path)
            xform_ops(g, T7)
            v = np.ascontiguousarray(d["mesh"]["vertices"], dtype=np.float32)
            idx = np.ascontiguousarray(d["mesh"]["indices"], dtype=np.int32)
            pts = Vt.Vec3fArray.FromNumpy(v)
            g.CreatePointsAttr(pts)
            g.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(idx) // 3, 3, dtype=np.int32)))
            g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx))
            g.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            g.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(pts))
        p = g.GetPrim()
        if d["color"] is not None:
            g.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(*[_srgb_to_linear(c) for c in d["color"]])]))
            UsdShade.MaterialBindingAPI.Apply(p).Bind(look(d["color"]))
        label(p, d["label"])
        if not d["collide"]:
            continue                                          # drawn only: plain geometry, no physics
        if not d["visible"]:
            g.CreatePurposeAttr(UsdGeom.Tokens.guide)          # a collider nobody should see
        UsdPhysics.CollisionAPI.Apply(p)
        UsdShade.MaterialBindingAPI.Apply(p).Bind(material(d), UsdShade.Tokens.weakerThanDescendants, "physics")
        p.AddAppliedSchema("NewtonCollisionAPI")
        attr(p, "newton:contactMargin", Sdf.ValueTypeNames.Float, float(d["margin"]), False)
        attr(p, "newton:contactGap", Sdf.ValueTypeNames.Float, float(d["gap"]), False)
        if st != int(GT.BOX):
            mc = UsdPhysics.MeshCollisionAPI.Apply(p)
            if st == int(GT.CONVEX_MESH):
                mc.CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
                p.AddAppliedSchema("NewtonMeshCollisionAPI")
                attr(p, "newton:maxHullVertices", Sdf.ValueTypeNames.Int, int(len(d["mesh"]["vertices"])), False)
            else:
                mc.CreateApproximationAttr(UsdPhysics.Tokens.none)
        sdf = d["sdf"]
        if sdf:
            res = int(sdf["max_resolution"])
            p.AddAppliedSchema("NewtonSDFCollisionAPI")
            attr(p, "newton:sdfMaxResolution", Sdf.ValueTypeNames.Int, int(8 * math.ceil(res / 8)), False)
            attr(p, "newton:sdfNarrowBandInner", Sdf.ValueTypeNames.Float, float(sdf["narrow_band_range"][0]), False)
            attr(p, "newton:sdfNarrowBandOuter", Sdf.ValueTypeNames.Float, float(sdf["narrow_band_range"][1]), False)
            attr(p, "newton:sdfPadding", Sdf.ValueTypeNames.Float, float(sdf["margin"]), False)
            attr(p, "newton:sdfTextureFormat", Sdf.ValueTypeNames.Token, "uint16", False)
            attr(p, "newton:hydroelasticEnabled", Sdf.ValueTypeNames.Bool, False, False)
            if res % 8:
                attr(p, "contactbench:sdfMaxResolution", Sdf.ValueTypeNames.Int, res)
        for key, c in d["custom"].items():
            v = c["value"]
            if isinstance(v, list):
                attr(p, c["usd"], Sdf.ValueTypeNames.DoubleArray, Vt.DoubleArray(v))
            elif key.endswith(("condim", "contype", "conaffinity", "group", "priority")):
                attr(p, c["usd"], Sdf.ValueTypeNames.Int, int(v))
            else:
                attr(p, c["usd"], Sdf.ValueTypeNames.Double, float(v))
        if "mujoco:condim" not in d["custom"]:
            attr(p, "mjc:condim", Sdf.ValueTypeNames.Int, int(meta["calibration"]["condim"]))
        if d["solver_ke"] is not None:
            attr(p, "contactbench:solverContactStiffness", Sdf.ValueTypeNames.Double, float(d["solver_ke"]))

    # joints
    UsdGeom.Scope.Define(stage, rec["root"] + "/Joints")
    art_roots = set()
    free_joints = rec.get("free_joints", {})
    for art in rec["articulations"]:
        bases = [j for j in art if j in free_joints]
        if bases:
            # a floating base: the root on its body (UsdPhysics' floating articulation; Newton's importer gives the
            # body a free joint and puts it in one articulation with the joints below), unless the body is alone
            if len(art) > 1:
                UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(free_joints[bases[0]]["child"]))
            continue
        roots = [j for j in art if rec["joints"][j]["parent"] is None]
        if roots:
            art_roots.add(roots[0])
    hard = meta["calibration"]["hard_joint_limits"]
    dt = meta["calibration"]["timestep"]
    for path, d in rec["joints"].items():
        jt = d["type"]
        dofs = d["dofs"]
        Rf, toks = _joint_frame([np.asarray(x["axis"]) for x in dofs]) if dofs else (np.eye(3), [])
        RfM = np.eye(4)
        RfM[:3, :3] = Rf
        Xp = _seven(_mat(d["X_p"]) @ RfM)
        Xc = _seven(_mat(d["X_c"]) @ RfM)
        if jt == int(JT.PRISMATIC):
            j = UsdPhysics.PrismaticJoint.Define(stage, path)
            j.CreateAxisAttr(toks[0])
        elif jt == int(JT.REVOLUTE):
            j = UsdPhysics.RevoluteJoint.Define(stage, path)
            j.CreateAxisAttr(toks[0])
        elif jt == int(JT.FIXED):
            j = UsdPhysics.FixedJoint.Define(stage, path)
        else:
            j = UsdPhysics.Joint.Define(stage, path)
        p = j.GetPrim()
        if d["parent"]:
            j.CreateBody0Rel().SetTargets([d["parent"]])
        j.CreateBody1Rel().SetTargets([d["child"]])
        j.CreateLocalPos0Attr(Gf.Vec3f(*Xp[:3]))
        j.CreateLocalRot0Attr(Gf.Quatf(Xp[6], Xp[3], Xp[4], Xp[5]))
        j.CreateLocalPos1Attr(Gf.Vec3f(*Xc[:3]))
        j.CreateLocalRot1Attr(Gf.Quatf(Xc[6], Xc[3], Xc[4], Xc[5]))
        j.CreateCollisionEnabledAttr(not d["collision_filter_parent"])
        j.CreateJointEnabledAttr(bool(d["enabled"]))
        label(p, d["label"])
        if path in art_roots:
            UsdPhysics.ArticulationRootAPI.Apply(p)
        p.AddAppliedSchema("NewtonJointAPI")
        single = jt in (int(JT.PRISMATIC), int(JT.REVOLUTE))
        modes, forces = [], []
        for x, tok in zip(dofs, toks):
            ang = x["kind"] == "angular"
            s = DEG if ang else 1.0                     # radians -> degrees for positions, per-degree for gains
            inst = ("angular" if ang else "linear") if single else (("rot" if ang else "trans") + tok)
            lo, hi = x["lower"], x["upper"]
            if single:
                if lo > -UNLIMITED * 0.999:
                    j.CreateLowerLimitAttr(float(lo * s))
                if hi < UNLIMITED * 0.999:
                    j.CreateUpperLimitAttr(float(hi * s))
            else:
                lim = UsdPhysics.LimitAPI.Apply(p, inst)
                lim.CreateLowAttr(float(lo * s) if lo > -UNLIMITED * 0.999 else -math.inf)
                lim.CreateHighAttr(float(hi * s) if hi < UNLIMITED * 0.999 else math.inf)
            if x["mode"] != 0:
                dr = UsdPhysics.DriveAPI.Apply(p, inst)
                dr.CreateTypeAttr("force")
                dr.CreateStiffnessAttr(float(x["ke"] / s))
                dr.CreateDampingAttr(float(x["kd"] / s))
                dr.CreateTargetPositionAttr(float(x["target_q"] * s))
                dr.CreateTargetVelocityAttr(float(x["target_qd"] * s))
                dr.CreateMaxForceAttr(float(x["effort_limit"]))
            modes.append(x["mode"])
            forces.append(x["f"])
            if single:
                attr(p, "newton:armature", Sdf.ValueTypeNames.Float, float(x["armature"]), False)
                attr(p, "newton:friction", Sdf.ValueTypeNames.Float, float(x["friction"]), False)
                attr(p, "newton:damping", Sdf.ValueTypeNames.Float, float(x["damping"] / s), False)
                attr(p, "newton:velocityLimit", Sdf.ValueTypeNames.Float, float(x["velocity_limit"] * s), False)
                attr(p, "newton:limitStiffness", Sdf.ValueTypeNames.Float, float(x["limit_ke"] / s), False)
                attr(p, "newton:limitDamping", Sdf.ValueTypeNames.Float, float(x["limit_kd"] / s), False)
                if x["q"] != 0.0:
                    kind = "angular" if ang else "linear"
                    attr(p, f"newton:{kind}:position", Sdf.ValueTypeNames.Float, float(x["q"] * s))
                    p.AddAppliedSchema(f"PhysicsJointStateAPI:{kind}")
                    attr(p, f"state:{kind}:physics:position", Sdf.ValueTypeNames.Float, float(x["q"] * s))
        if jt == int(JT.D6):
            used = {(("rot" if x["kind"] == "angular" else "trans") + t) for x, t in zip(dofs, toks)}
            for inst in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
                if inst not in used:                         # UsdPhysics: low > high locks the axis
                    lim = UsdPhysics.LimitAPI.Apply(p, inst)
                    lim.CreateLowAttr(1.0)
                    lim.CreateHighAttr(-1.0)
        if not single and dofs:
            # NewtonJointAPI holds one value per joint; a D6 whose dofs agree gets it, the rest come from the record
            for name, key, scale in (("newton:armature", "armature", None), ("newton:friction", "friction", None)):
                vals = {x[key] for x in dofs}
                if len(vals) == 1:
                    attr(p, name, Sdf.ValueTypeNames.Float, float(vals.pop()), False)
        if any(m != (1 if x["ke"] != 0.0 else 2 if x["kd"] != 0.0 else 4) for m, x in zip(modes, dofs) if m != 0):
            attr(p, "contactbench:actuatorMode", Sdf.ValueTypeNames.IntArray, Vt.IntArray([int(m) for m in modes]))
        if any(f != 0.0 for f in forces):
            attr(p, "contactbench:jointForce", Sdf.ValueTypeNames.DoubleArray, Vt.DoubleArray(forces))
        # MuJoCo per-dof attributes (one value per joint prim)
        custom = dict(d["custom"])
        if hard and dofs and "mujoco:solreflimit" not in custom:
            custom["mujoco:solreflimit"] = {"usd": "mjc:solreflimit",
                                           "values": [[2.0 * dt, meta["calibration"]["limit_dampratio"]]] * len(dofs)}
        for key, c in custom.items():
            if key == "mujoco:solreflimit_mode" or not c["usd"] or c["usd"] == "*":
                continue
            vals = c["values"]
            if any(v != vals[0] for v in vals):
                continue                                     # not one value per joint: the record carries it
            v = vals[0]
            if isinstance(v, list):
                attr(p, c["usd"], Sdf.ValueTypeNames.DoubleArray, Vt.DoubleArray(v))
            else:
                attr(p, c["usd"], Sdf.ValueTypeNames.Double, float(v))

    # gear couplings: MuJoCo joint equality on the follower (Newton imports it as a mimic constraint + equality row)
    for m in rec["mimic"]:
        p = stage.GetPrimAtPath(m["follower"])
        p.AddAppliedSchema("NewtonMimicAPI")
        p.AddAppliedSchema("MjcEqualityJointAPI")
        p.CreateRelationship("newton:mimicJoint", False).SetTargets([m["leader"]])
        s = DEG if rec["joints"][m["follower"]]["type"] == int(JT.REVOLUTE) else 1.0
        attr(p, "newton:mimicCoef0", Sdf.ValueTypeNames.Float, float(m["coef0"] * s), False)
        attr(p, "newton:mimicCoef1", Sdf.ValueTypeNames.Float, float(m["coef1"]), False)
        attr(p, "newton:mimicEnabled", Sdf.ValueTypeNames.Bool, bool(m["enabled"]), False)
        for key, usd in (("mujoco:eq_solref", "mjc:solref"), ("mujoco:eq_solimp", "mjc:solimp")):
            if key in m["custom"]:
                attr(p, usd, Sdf.ValueTypeNames.DoubleArray, Vt.DoubleArray(m["custom"][key]))

    # collision filters
    by_first: dict[str, list[str]] = {}
    for a, c in rec["filters"]:
        by_first.setdefault(a, []).append(c)
    for a, cs in by_first.items():
        UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(a)).CreateFilteredPairsRel().SetTargets(cs)

    root.GetPrim().SetCustomDataByKey("contactbench", {
        "format": FORMAT, "asset": meta["asset"], "name": meta["name"], "kind": meta["kind"],
        "model": meta.get("model") or "", "variant": meta["variant"], "label": rec["handle"]["label"],
        "choice": json.dumps(rec["handle"]["choice"]), "cadPose": Gf.Matrix4d(np.asarray(meta["cad_pose"]).T.tolist()),
        "calibration": json.dumps(meta["calibration"]), "handle": json.dumps(rec["handle"], default=_json_default),
        "rig": json.dumps(rec["rig"]), "record": _record_json(rec)})
    stage.GetRootLayer().Save()
    return out_path


def variant_slug(choice: dict) -> str:
    from force_bench.kit.check import slug
    return slug(choice)


def export_variant(key: str, choice=None, out_path=None, label: str | None = None, **params) -> Path:
    """Build kit asset ``key`` (variant ``choice``; mechanism ``params`` as for add_asset) and write it as a USD
    stage at ``out_path`` (default outputs/kit_usd/<key>/<variant>.usda)."""
    from force_bench.kit import load_kit
    from force_bench.paths import OUTPUTS_ROOT
    kit = load_kit()
    asset = kit[key]
    choice = {} if choice is None else (choice if isinstance(choice, dict) else {"": choice})
    variant = variant_slug(choice)
    if out_path is None:
        out_path = OUTPUTS_ROOT / "kit_usd" / key / f"{variant}.usda"
    rig, h, snap = build_for_export(key, choice, label=label, **params)
    rec = collect(rig, h, snap)
    meta = {"asset": key, "name": asset.name, "kind": asset.sim.get("kind", "free"), "model": asset.sim.get("model"),
            "variant": variant,
            "cad_pose": np.asarray(h.pose, float), "calibration": calibration_settings(rig, asset.sim.get("model"))}
    if params:
        meta["calibration"]["build_overrides"] = {k: v for k, v in params.items()
                                                  if isinstance(v, (int, float, str, bool, type(None)))}
    return write_stage(rec, out_path, meta)


# ── import ────────────────────────────────────────────────────────────────────────────────────────────────────────
def read_meta(path) -> dict:
    """The ``customData["contactbench"]`` of a stage, JSON fields decoded."""
    from pxr import Usd
    stage = Usd.Stage.Open(str(path))
    root = stage.GetDefaultPrim()
    cd = dict(root.GetCustomDataByKey("contactbench") or {})
    if not cd or not str(cd.get("format", "")).startswith("contactbench-kit-usd/"):
        raise ValueError(f"{path}: not a ContactBench kit stage (no customData['contactbench'])")
    out = dict(cd)
    for k in ("choice", "calibration", "handle", "rig", "record"):
        out[k] = json.loads(cd[k])
    out["cadPose"] = np.asarray(cd["cadPose"], dtype=float).T
    out["root"] = str(root.GetPath())
    return out


def _ensure_mujoco_attributes(builder) -> None:
    import newton
    if "mujoco:condim" not in builder.custom_attributes:
        newton.solvers.SolverMuJoCo.register_custom_attributes(builder)


def _relabel(text, old, new):
    if not text or old == new:
        return text
    for pre in ("kit/", ""):
        if text == pre + old:
            return pre + new
        if text.startswith(pre + old + "/"):
            return pre + new + text[len(pre + old):]
    return text


@dataclass
class UsdAssetHandle:
    """What add_asset_usd created: the fields of ``force_bench.kit.AssetHandle`` (same roles, parameters and info;
    indices of the scene it was added to) plus the stage's extras and hooks."""
    key: str
    label: str
    choice: dict
    pose: np.ndarray
    bodies: dict = field(default_factory=dict)
    joints: dict = field(default_factory=dict)
    shapes: dict = field(default_factory=dict)
    params: dict = field(default_factory=dict)
    info: dict = field(default_factory=dict)
    path: str = ""
    calibration: dict = field(default_factory=dict)
    preload: dict = field(default_factory=dict)          # dof index -> constant generalised force
    solver_shape_ke: dict = field(default_factory=dict)  # shape index -> contact stiffness after the solver is built
    shape_ke: dict = field(default_factory=dict)         # shape index -> (ke, kd) the calibration ran with
    import_report: dict = field(default_factory=dict)    # what add_usd alone did not reproduce: counts per field
    import_details: dict = field(default_factory=dict)   # ... and where

    # ── hooks ──
    def solver_options(self) -> dict:
        """SolverMuJoCo keyword arguments of the calibration (pass to SolverMuJoCo or force_bench.rig.make_solver)."""
        c = self.calibration
        return {"solver": c["mj_solver"], "integrator": c["integrator"], "cone": c["cone"],
                "impratio": c["impratio"], "iterations": c["iterations"], "ls_iterations": c["ls_iterations"],
                "nconmax": c["nconmax"], "njmax": c["njmax"]}

    def prepare_model(self, model, scene_contacts: bool = True) -> None:
        """Before the solver is built: give every shape of the scene (your robot's too) the contact stiffness,
        damping and MuJoCo condim the calibration ran with -- what ``force_bench.rig.make_solver`` does to a
        ContactBench scene -- then this asset's own values (``scene_contacts=False``: only those)."""
        if scene_contacts:
            model.shape_material_ke.fill_(float(self.calibration["contact_stiffness"]))
            model.shape_material_kd.fill_(float(self.calibration["contact_damping"]))
            mj = getattr(model, "mujoco", None)
            if mj is not None and hasattr(mj, "condim"):
                mj.condim.fill_(int(self.calibration["condim"]))
        self.after_solver(model, None, warn=False)

    def after_solver(self, model, solver=None, warn: bool = True) -> None:
        """Re-apply the contact stiffness and damping the calibration ran with to this asset's shapes (and the USB
        jaws' stiffer contacts) after the solver exists -- ``force_bench.rig.make_solver`` resets every shape's --
        and warn where the solver differs from the calibration.  ``model`` may be a Rig (then its solver is used)."""
        import newton
        if hasattr(model, "model") and hasattr(model, "solver"):
            solver = solver or model.solver
            model = model.model
        if self.shape_ke or self.solver_shape_ke:
            ke = model.shape_material_ke.numpy()
            kd = model.shape_material_kd.numpy()
            for s, (k, d) in self.shape_ke.items():
                ke[s], kd[s] = k, d
            for s, k in self.solver_shape_ke.items():
                ke[s] = k
            model.shape_material_ke.assign(ke)
            model.shape_material_kd.assign(kd)
            if solver is not None and hasattr(solver, "notify_model_changed"):
                solver.notify_model_changed(newton.ModelFlags.SHAPE_PROPERTIES)
        if warn and solver is not None:
            self.check_scene(solver=solver)

    def apply_preload(self, control) -> None:
        """Add the asset's constant joint forces (the doorknob's spring preload) to ``control.joint_f``.  They are
        already in ``model.control()`` (builder.joint_f); call this after zeroing joint_f yourself."""
        if not self.preload:
            return
        f = control.joint_f.numpy()
        for d, v in self.preload.items():
            f[d] += v
        control.joint_f.assign(f)

    def check_scene(self, rig=None, solver=None, dt: float | None = None, collide_every_substep=None) -> list[str]:
        """Differences between a scene and the settings the calibration assumes (also issued as warnings)."""
        c = self.calibration
        out = []
        if rig is not None:
            dt = dt or 1.0 / (rig.fps * rig.substeps)
            if collide_every_substep is None:
                collide_every_substep = rig.effective_collide_every_substep()
            solver = solver or getattr(rig, "solver", None)
        if dt is not None and abs(dt - c["timestep"]) > 1e-3 * c["timestep"]:
            out.append(f"timestep {dt:.6g} s, calibrated at {c['timestep']:.6g} s ({c['fps']} fps x {c['substeps']} "
                       f"substeps): MuJoCo's contact and limit stiffness are bound to the timestep")
        if collide_every_substep is not None and bool(collide_every_substep) != bool(c["collide_every_substep"]):
            out.append(f"collision every substep = {collide_every_substep}, calibrated with "
                       f"{c['collide_every_substep']} ({c.get('probe_collision', '')})")
        if solver is not None:
            mj = getattr(solver, "mjw_model", None) or getattr(solver, "mj_model", None)
            opt = getattr(mj, "opt", None)
            if opt is not None:
                imp = _first(getattr(opt, "impratio", None))
                if imp is not None and abs(imp - c["impratio"]) > 1e-6 * max(1.0, c["impratio"]):
                    out.append(f"impratio {imp:g}, calibrated with {c['impratio']:g}")
                it = _first(getattr(opt, "iterations", None))
                if it is not None and it < c["iterations"]:
                    out.append(f"{it} solver iterations, calibrated with {c['iterations']} "
                               f"(probes {c['probe_iterations']})")
                if bool(_first(getattr(opt, "run_collision_detection", None)) or False):
                    out.append("SolverMuJoCo(use_mujoco_contacts=True): MuJoCo collides meshes as convex hulls; the "
                               "calibration used Newton's contacts (use_mujoco_contacts=False)")
            else:
                out.append(f"solver {type(solver).__name__}: the calibration used SolverMuJoCo")
        for m in out:
            warnings.warn(f"{self.key} ({self.path}): {m}", stacklevel=2)
        return out


def _first(v):
    if v is None:
        return None
    try:
        a = np.asarray(v.numpy() if hasattr(v, "numpy") else v).reshape(-1)
        return float(a[0]) if a.size else None
    except Exception:  # noqa: BLE001
        return None


def add_asset_usd(target, path, xform=None, *, pose=None, label: str | None = None, exact: bool = True,
                  register_mujoco: bool = True, **usd_kwargs) -> UsdAssetHandle:
    """Add a kit stage written by ``export_variant`` to ``target`` (a ``newton.ModelBuilder`` or a
    ``force_bench.rig.Rig``) and return its ``UsdAssetHandle``.

    ``xform``: where the stage goes (a warp transform or 4x4; the stage stands on z = 0, centred), or ``pose``: the
    world pose of the asset's CAD frame, as ``add_asset`` takes it.  ``label``: rename the asset (default: the
    exported label).  ``exact``: restore the values Newton's importer rounds or drops from the stage's record (see
    the module docstring); False leaves the builder as ``add_usd`` made it, plus the ``contactbench:`` extras.
    ``usd_kwargs`` go to ``builder.add_usd``."""
    import newton
    rig = target if hasattr(target, "builder") and hasattr(target, "solver_options") else None
    builder = rig.builder if rig is not None else target
    if builder.up_axis != newton.Axis.Z:
        raise ValueError("add_asset_usd: the kit's stages and their record are Z-up; build the scene with a Z-up "
                         f"ModelBuilder (Newton's default), not {builder.up_axis}")
    meta = read_meta(path)
    rec, hd = meta["record"], meta["handle"]
    old_label = hd["label"]
    label = label or old_label
    cad = meta["cadPose"]
    if pose is not None:
        X = np.asarray(pose, float) @ np.linalg.inv(cad)
    elif xform is None:
        X = np.eye(4)
    else:
        X = _mat(xform)
    if register_mujoco:
        _ensure_mujoco_attributes(builder)
        if rig is not None:
            rig._kit_mjc_attrs = True
    with warnings.catch_warnings():
        # the initial position of the dial's sprung pads is newton:linear:position, a non-schema attribute that
        # Newton 1.5 reads with a request to file an issue
        warnings.filterwarnings("ignore", message=r".*non-schema attribute.*")
        j_lo = builder.joint_count
        res = builder.add_usd(str(path), xform=_wp(X), **usd_kwargs)
    bmap, jmap, smap = res["path_body_map"], dict(res["path_joint_map"]), res["path_shape_map"]
    report: dict[str, list] = {}

    def note(kind, what):
        report.setdefault(kind, []).append(what)

    # ── the free joints Newton's importer made (no prim: found by their body) ──
    free = rec.get("free_joints", {})
    prim_joints = set(jmap.values())
    for p, d in free.items():
        b = bmap.get(d["child"])
        js = [j for j in range(j_lo, builder.joint_count) if builder.joint_child[j] == b and j not in prim_joints]
        if b is not None and len(js) == 1:
            jmap[p] = js[0]                         # (another type if add_usd got floating=False or base_joint=...)

    # ── labels ──
    for p, d in rec["bodies"].items():
        if p in bmap:
            builder.body_label[bmap[p]] = _relabel(d["label"], old_label, label)
    for p, d in [*rec["joints"].items(), *free.items()]:
        if p in jmap and not (p in free and re.fullmatch(r"joint_\d+", d["label"] or "")):
            builder.joint_label[jmap[p]] = _relabel(d["label"], old_label, label)   # (not a default label: index)
    for p, d in rec["shapes"].items():
        if p not in smap:
            note("missing_shapes", p)
            continue
        builder.shape_label[smap[p]] = _relabel(d["label"], old_label, label)

    # ── contactbench extras (always) ──
    preload = {}
    for p, d in [*rec["joints"].items(), *free.items()]:
        j = jmap.get(p)
        if j is None:
            note("missing_joints", p)
            continue
        if int(builder.joint_type[j]) != d["type"]:
            note("joint_type", p)                   # left as the importer made it (see add_usd's floating/base_joint)
            continue
        d0 = builder.joint_qd_start[j]
        for k, x in enumerate(d["dofs"]):
            if int(builder.joint_target_mode[d0 + k]) != x["mode"]:
                builder.joint_target_mode[d0 + k] = x["mode"]
                note("actuator_mode", f"{p}[{k}]")
            if x["f"] != 0.0:
                builder.joint_f[d0 + k] += x["f"]
                preload[d0 + k] = x["f"]
                note("joint_force", f"{p}[{k}]")
    for p, d in rec["shapes"].items():
        s = smap.get(p)
        if s is None or not d["sdf"]:
            continue
        res8 = builder.shape_sdf_max_resolution[s]
        if res8 != d["sdf"]["max_resolution"]:
            builder.shape_sdf_max_resolution[s] = int(d["sdf"]["max_resolution"])
            note("sdf_resolution", p)

    # ── exact restore and verification ──
    if exact:
        _restore(builder, rec, X, bmap, jmap, smap, note, path)

    # ── articulations Newton's importer did not make (a single world-anchored joint: UsdPhysics lists no body) ──
    # added in joint order (Newton wants articulation starts increasing; the importer orders joints by type)
    missing = []
    for art in rec["articulations"]:
        js = sorted(jmap[p] for p in art if p in jmap)
        owners = {builder.joint_articulation[j] for j in js}
        if js and owners == {-1}:
            missing.append((js, art))
        elif len(owners) > 1:
            note("articulation_split", art)
            warnings.warn(f"{path}: joints {art} are one articulation in the asset, Newton's importer split them "
                          f"(a floating base needs PhysicsArticulationRootAPI on its root body)", stacklevel=2)
    for js, art in sorted(missing, key=lambda m: m[0][0]):
        last = builder.articulation_start[-1] if builder.articulation_count else -1
        if js != list(range(js[0], js[0] + len(js))) or js[0] < last:
            note("articulation_not_added", art)
            warnings.warn(f"{path}: joints {art} could not be made an articulation (not contiguous, or before the "
                          f"builder's last articulation); SolverMuJoCo still simulates them", stacklevel=2)
            continue
        first = rec["joints"].get(art[0]) or free.get(art[0])
        builder.add_articulation(js, label=_relabel(first["label"], old_label, label))
        note("articulation", art[0])

    # ── the handle ──
    h = _decode_handle(hd, X, bmap, jmap, smap, old_label, label)
    h.path = str(path)
    h.calibration = meta["calibration"]
    h.preload = preload
    for p, d in rec["shapes"].items():
        s = smap.get(p)
        if s is None or not d["collide"]:
            continue
        h.shape_ke[s] = (d["ke_calibrated"], d["kd_calibrated"])
        if d["solver_ke"] is not None:
            h.solver_shape_ke[s] = d["solver_ke"]
    h.import_report = {k: len(v) for k, v in report.items()}
    h.import_details = report

    # ── a Rig gets the scene settings add_asset gives it ──
    if rig is not None:
        rs = meta["rig"]
        if rs.get("hard_joint_limits"):
            rig.hard_joint_limits = True
        for k, v in rs.get("solver_options", {}).items():
            rig.solver_options.setdefault(k, v)
        for k, paths in rs.get("shapes", {}).items():
            rig.shapes.setdefault(_relabel(k, old_label, label), []).extend(smap[p] for p in paths if p in smap)
        for k, p in rs.get("joint_of", {}).items():
            if p in jmap:
                rig._joint_of[_relabel(k, old_label, label)] = jmap[p]
    return h


def _restore(builder, rec, X, bmap, jmap, smap, note, path=None) -> None:
    """Overwrite what the importer rounded or approximated with the record's exact values; note what differed."""
    def close(a, b, tol=F32):
        a, b = np.asarray(a, float), np.asarray(b, float)
        return bool(np.all(np.abs(a - b) <= tol * np.maximum(1.0, np.maximum(np.abs(a), np.abs(b)))))

    def place(T7):
        return _seven(X @ _mat(T7))

    for p, d in rec["bodies"].items():
        b = bmap.get(p)
        if b is None:
            note("missing_bodies", p)
            continue
        if not close(builder.body_mass[b], d["mass"]):
            note("body_mass", p)
        builder.body_mass[b] = d["mass"]
        builder.body_inv_mass[b] = d.get("inv_mass", 1.0 / d["mass"] if d["mass"] > 0 else 0.0)
        inertia = np.asarray(d["inertia"], float).reshape(3, 3)
        if not close(np.asarray(builder.body_inertia[b], float).reshape(3, 3), inertia, 1e-5):
            note("body_inertia", p)
        import warp as wp
        builder.body_inertia[b] = wp.mat33(*inertia.reshape(-1).tolist())
        # the inverse as the builder held it (Newton keeps it in 32-bit floats as shapes add mass)
        inv = d.get("inv_inertia") or (np.linalg.inv(inertia).reshape(-1).tolist() if np.linalg.det(inertia) > 0
                                       else [0.0] * 9)
        builder.body_inv_inertia[b] = wp.mat33(*inv)
        if not close(builder.body_com[b], d["com"], 1e-6):
            note("body_com", p)
        builder.body_com[b] = wp.vec3(*d["com"])
        q = place(d["q"])
        if p in rec["zero_pose_bodies"]:
            note("body_initial_pose", p)
        builder.body_q[b] = wp.transform(wp.vec3(*q[:3]), wp.quat(*q[3:]))

    import warp as wp
    free = rec.get("free_joints", {})
    moved = not np.allclose(X, np.eye(4), rtol=0.0, atol=1e-12)       # (pose @ inv(cadPose) at the export pose)
    for p, d in [*rec["joints"].items(), *free.items()]:
        j = jmap.get(p)
        if j is None or int(builder.joint_type[j]) != d["type"]:
            continue
        is_free = p in free
        # a free joint keeps its anchor (the world origin, as add_joint_free makes it) and its coordinates carry
        # the body's pose; any other joint to the world moves its anchor with the asset
        Xp = d["X_p"] if d["parent"] or is_free else place(d["X_p"])
        if not close(_seven(_mat(builder.joint_X_p[j])), _seven(_mat(Xp)), 1e-5):
            note("joint_frame", p)
        builder.joint_X_p[j] = wp.transform(wp.vec3(*Xp[:3]), wp.quat(*Xp[3:]))
        builder.joint_X_c[j] = wp.transform(wp.vec3(*d["X_c"][:3]), wp.quat(*d["X_c"][3:]))
        d0, q0 = builder.joint_qd_start[j], builder.joint_q_start[j]
        for k, x in enumerate(d["dofs"]):
            i = d0 + k
            if not close(builder.joint_axis[i], x["axis"], 1e-6):
                builder.joint_axis[i] = wp.vec3(*x["axis"])
            fields = (("joint_target_ke", "ke"), ("joint_target_kd", "kd"),
                      ("joint_target_qd", "target_qd"), ("joint_limit_lower", "lower"), ("joint_limit_upper", "upper"),
                      ("joint_limit_ke", "limit_ke"), ("joint_limit_kd", "limit_kd"), ("joint_armature", "armature"),
                      ("joint_friction", "friction"), ("joint_damping", "damping"),
                      ("joint_effort_limit", "effort_limit"), ("joint_velocity_limit", "velocity_limit"),
                      ("joint_qd", "qd"))
            for arr, key in fields:
                cur = getattr(builder, arr)[i]
                if not close(cur, x[key]):
                    note(arr.replace("joint_", "dof_"), f"{p}[{k}]")
                getattr(builder, arr)[i] = x[key]
            if is_free:
                continue
            # joint_target_q and joint_q are per coordinate (q0 + k), everything else per dof (d0 + k); for the
            # revolute, prismatic and D6 joints they coincide within the joint (a free joint's are below)
            if not close(builder.joint_target_q[q0 + k], x["target_q"]):
                note("dof_joint_target_q", f"{p}[{k}]")
            builder.joint_target_q[q0 + k] = x["target_q"]
            if not close(builder.joint_q[q0 + k], x["q"], 1e-6):
                note("dof_initial_position", f"{p}[{k}]")
            builder.joint_q[q0 + k] = x["q"]
        if is_free:
            # coordinates: the body's pose relative to the anchor, moved with the asset; velocities rotate with it
            A = _mat(Xp)
            q = list(d["q"])
            if moved:
                q = _seven(np.linalg.inv(A) @ X @ A @ _mat(d["q"]))
                if np.dot(q[3:], d["q"][3:]) < 0.0:
                    q[3:] = [-v for v in q[3:]]          # the quaternion's sign nearest the recorded one
            if not close(_mat(builder.joint_q[q0:q0 + 7]), _mat(q), 1e-6):
                note("free_joint_position", p)
            builder.joint_q[q0:q0 + 7] = q
            builder.joint_target_q[q0:q0 + 7] = d["target_q"]
            if moved:
                Rm = (np.linalg.inv(A) @ X @ A)[:3, :3]
                qd = np.asarray([x["qd"] for x in d["dofs"]], float)
                builder.joint_qd[d0:d0 + 6] = [float(v) for v in (*(Rm @ qd[:3]), *(Rm @ qd[3:]))]
        for key, c in d["custom"].items():
            a = builder.custom_attributes.get(key)
            if a is None:
                note("custom_attribute_unregistered", f"{p}:{key}")
                continue
            idx = [d0 + k for k in range(len(d["dofs"]))] if len(c["values"]) == len(d["dofs"]) and d["dofs"] \
                else [j]
            for i, v in zip(idx, c["values"]):
                old = a.values.get(i) if isinstance(a.values, dict) else None
                if old is None or not close(_list(old) if hasattr(old, "__len__") else float(old), v):
                    note("joint_" + key.split(":")[-1], f"{p}")
                a.values[i] = a.dtype(*v) if isinstance(v, list) else a.dtype(v)

    for p, d in rec["shapes"].items():
        s = smap.get(p)
        if s is None:
            continue
        for arr, key in (("shape_material_mu", "mu"), ("shape_material_restitution", "restitution"),
                         ("shape_material_mu_torsional", "mu_torsional"), ("shape_material_mu_rolling", "mu_rolling"),
                         ("shape_margin", "margin"), ("shape_gap", "gap")):
            if d["collide"] and not close(getattr(builder, arr)[s], d[key]):
                note(arr, p)                         # (a drawn-only shape carries no contact material)
            getattr(builder, arr)[s] = d[key]
        if d["collide"] and not close([builder.shape_material_ke[s], builder.shape_material_kd[s]],
                                      [d["ke_calibrated"], d["kd_calibrated"]]):
            note("shape_contact_stiffness", p)
        if d["sdf"]:
            nb = tuple(float(v) for v in d["sdf"]["narrow_band_range"])
            if not close(builder.shape_sdf_narrow_band_range[s], nb) or \
                    not close(builder.shape_sdf_padding[s] or 0.0, d["sdf"]["margin"]):
                note("shape_sdf_settings", p)
            builder.shape_sdf_narrow_band_range[s] = nb
            builder.shape_sdf_padding[s] = float(d["sdf"]["margin"])
            builder.shape_sdf_max_resolution[s] = int(d["sdf"]["max_resolution"])
        elif builder.shape_sdf_max_resolution[s] is not None and d["collide"]:
            note("shape_sdf_unexpected", p)
        T = d["X"] if d["body"] else place(d["X"])
        if not close(_seven(_mat(builder.shape_transform[s])), _seven(_mat(T)), 1e-5):
            note("shape_transform", p)
        builder.shape_transform[s] = wp.transform(wp.vec3(*T[:3]), wp.quat(*T[3:]))
        import newton
        want_vis = d["visible"]
        has_vis = bool(builder.shape_flags[s] & int(newton.ShapeFlags.VISIBLE))
        if want_vis != has_vis:
            note("shape_visible", p)
            if want_vis:
                builder.shape_flags[s] |= int(newton.ShapeFlags.VISIBLE)
            else:
                builder.shape_flags[s] &= ~int(newton.ShapeFlags.VISIBLE)
        if d["collide"] != bool(builder.shape_flags[s] & int(newton.ShapeFlags.COLLIDE_SHAPES)):
            note("shape_collide", p)

    have = {tuple(sorted(pr)) for pr in builder.shape_collision_filter_pairs}
    for a, c in rec["filters"]:
        if a in smap and c in smap:
            pair = tuple(sorted((smap[a], smap[c])))
            if pair not in have:
                builder.add_shape_collision_filter_pair(*pair)
                note("collision_filter", f"{a} | {c}")

    # gear couplings: exact coefficients (newton:mimicCoef* are 32-bit) in the mimic and in its MuJoCo equality row
    poly = builder.custom_attributes.get("mujoco:equality_constraint_polycoef")
    eq_of = _eq_rows_for_mimic(builder)
    for m in rec["mimic"]:
        f, ld = jmap.get(m["follower"]), jmap.get(m["leader"])
        hits = [i for i in range(len(builder.constraint_mimic_joint0))
                if builder.constraint_mimic_joint0[i] == f and builder.constraint_mimic_joint1[i] == ld]
        if not hits:
            note("mimic_missing", m["follower"])
            continue
        i = hits[-1]
        if not close([builder.constraint_mimic_coef0[i], builder.constraint_mimic_coef1[i]], [m["coef0"], m["coef1"]]):
            note("mimic_coefficients", m["follower"])
        builder.constraint_mimic_coef0[i] = m["coef0"]
        builder.constraint_mimic_coef1[i] = m["coef1"]
        r = eq_of.get(i)
        if poly is not None and r is not None and r < len(poly.values):
            pc = _list(poly.values[r])
            pc[0], pc[1] = m["coef0"], m["coef1"]
            poly.values[r] = pc                      # a list of five floats per row (see newton's equality.py)
        for key, v in m["custom"].items():
            a = builder.custom_attributes.get(key)
            if a is not None and r is not None and r < len(a.values):
                if not close(_list(a.values[r]), v):
                    note("mimic_" + key.split(":")[-1], m["follower"])
                a.values[r] = a.dtype(*v)

    # convex hulls: Newton's importer re-hulls the authored points (physics:approximation = convexHull) and may drop
    # nearly coplanar ones; the stage holds the exact hull, as add_shape_convex_hull got it
    hulls = [(p, smap[p]) for p, d in rec["shapes"].items() if d["type"] == 10 and p in smap]
    if hulls and path is not None:
        import newton
        from pxr import Usd, UsdGeom
        from newton._src.geometry.utils import compute_shape_radius
        stage = Usd.Stage.Open(str(path))
        for p, s in hulls:
            g = UsdGeom.Mesh(stage.GetPrimAtPath(p))
            v = np.asarray(g.GetPointsAttr().Get(), dtype=np.float32)
            idx = np.asarray(g.GetFaceVertexIndicesAttr().Get(), dtype=np.int32)
            src = builder.shape_source[s]
            same = src is not None and np.array_equal(np.asarray(src.vertices), v) and \
                np.array_equal(np.asarray(src.indices).reshape(-1), idx)
            if not same:
                note("convex_hull", p)
                mesh = newton.Mesh(v, idx)
                builder.shape_source[s] = mesh
                builder.shape_type[s] = newton.GeoType.CONVEX_MESH
                builder.shape_collision_radius[s] = compute_shape_radius(newton.GeoType.CONVEX_MESH,
                                                                         builder.shape_scale[s], mesh)


def _decode_handle(hd, X, bmap, jmap, smap, old_label, label) -> UsdAssetHandle:
    def dec(obj):
        if isinstance(obj, dict):
            if "$ref" in obj:
                m = {"shape": smap, "joint": jmap, "body": bmap}[obj["$ref"]]
                return -1 if obj["path"] is None else m.get(obj["path"], -1)
            if "$vec" in obj:
                v = np.asarray(obj["v"], float)
                return (X[:3, :3] @ v if obj["$vec"] == "dir" else X[:3, :3] @ v + X[:3, 3]).tolist()
            return {k: dec(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [dec(v) for v in obj]
        if isinstance(obj, str):
            return _relabel(obj, old_label, label)
        return obj

    lbl = label if hd["label"] == old_label else hd["label"]
    h = UsdAssetHandle(hd["key"], lbl, dict(hd["choice"]), X @ np.asarray(hd["cad_pose"], float),
                       bodies={r: bmap[p] for r, p in hd["bodies"].items() if p in bmap},
                       joints={r: jmap[p] for r, p in hd["joints"].items() if p in jmap},
                       shapes={n: [smap[p] for p in ps if p in smap] for n, ps in hd["shapes"].items()},
                       params=hd["params"], info=dec(hd["info"]))
    if "modules" in hd:
        h._modules = {k: _decode_handle(m, X, bmap, jmap, smap, old_label, label) for k, m in hd["modules"].items()}
    return h
