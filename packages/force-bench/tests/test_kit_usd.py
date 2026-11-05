"""The kit's USD export (force_bench.kit.usd): the structure of the stages, and the round trip into a ModelBuilder.

Run with:
    python -m pytest packages/force-bench/tests/test_kit_usd.py

Everything but the last test runs on the CPU (signed-distance fields are not baked: the exporter only reads their
settings); the last one settles a card from its stage through Newton and needs CUDA.  Needs usd-core; the round-trip
tests also need newton-usd-schemas (the ``usd`` extra), which Newton's USD importer requires.
"""

import json
import math

import numpy as np
import pytest

pxr = pytest.importorskip("pxr")
wp = pytest.importorskip("warp")

from pxr import Usd, UsdPhysics  # noqa: E402

from force_bench.kit import add_asset, place_on_table  # noqa: E402
from force_bench.kit import usd as kusd  # noqa: E402


@pytest.fixture(scope="module")
def stages(tmp_path_factory):
    """Export a doorknob module (revolute + prismatic, spring preload, gear coupling, convex hulls), the push-down
    dial (D6 joint, zero-stiffness position drives, hard limits, impratio), the standard dial (pads that start
    displaced), the Ethernet plug and slot (a floating base: the plug on a free joint, its latch tab hinged to it)
    and a cup (a free body)."""
    out = tmp_path_factory.mktemp("kit_usd")
    paths = {}
    with wp.ScopedDevice("cpu"):
        paths["doorknob"] = kusd.export_variant("doorknob-module", {"Door handle geometry": "Handle 1"},
                                                out / "doorknob.usda")
        paths["push"] = kusd.export_variant("dial-push-down-to-turn", {}, out / "push.usda")
        paths["dial"] = kusd.export_variant("dial", {}, out / "dial.usda")
        paths["ethernet"] = kusd.export_variant("ethernet-plug-and-slot", {}, out / "ethernet.usda")
        paths["cup"] = kusd.export_variant("cup", {}, out / "cup.usdc")
    return paths


def _api(prim) -> set[str]:
    """Applied API schemas as authored (GetAppliedSchemas lists only schemas the registry knows; MuJoCo's are
    codeless and may not be installed)."""
    lo = prim.GetMetadata("apiSchemas")
    return set(lo.GetAddedOrExplicitItems()) if lo else set()


def _prims(stage, type_name=None, api=None):
    out = []
    for p in stage.Traverse():
        if type_name and p.GetTypeName() != type_name:
            continue
        if api and api not in _api(p):
            continue
        out.append(p)
    return out


def test_joint_frame_maps_any_axis_onto_a_frame_axis():
    R, toks = kusd._joint_frame([np.array([0.0, -1.0, 0.0])])
    assert toks == ["Y"] and np.allclose(R @ [0.0, 1.0, 0.0], [0.0, -1.0, 0.0])
    R, toks = kusd._joint_frame([np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0])])
    assert toks == ["Z", "Z"] and np.allclose(R, np.eye(3))
    R, toks = kusd._joint_frame([np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)])
    assert toks == ["X"] and np.allclose(R @ [1.0, 0.0, 0.0], np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0))


def test_doorknob_stage_has_joints_drives_gear_preload_and_contact_settings(stages):
    st = Usd.Stage.Open(str(stages["doorknob"]))
    root = st.GetDefaultPrim()
    meta = kusd.read_meta(stages["doorknob"])
    assert meta["asset"] == "doorknob-module" and meta["kind"] == "mechanism"
    assert UsdPhysics.GetStageKilogramsPerUnit(st) == 1.0 and not _prims(st, "PhysicsScene")
    knob = st.GetPrimAtPath(f"{root.GetPath()}/Joints/knob")
    latch = st.GetPrimAtPath(f"{root.GetPath()}/Joints/latch")
    assert knob.IsA(UsdPhysics.RevoluteJoint) and latch.IsA(UsdPhysics.PrismaticJoint)
    # the spindle spring: drive stiffness per degree, limits in degrees, the preload as a contactbench attribute
    K, tau0 = meta["handle"]["params"]["K_Nm_per_rad"], meta["handle"]["params"]["tau0_Nm"]
    drive = UsdPhysics.DriveAPI(knob, "angular")
    assert drive.GetStiffnessAttr().Get() == pytest.approx(K * math.pi / 180.0, rel=1e-6)
    stop = meta["handle"]["info"]["stop_rad"]
    assert UsdPhysics.RevoluteJoint(knob).GetLowerLimitAttr().Get() == pytest.approx(-math.degrees(stop), rel=1e-6)
    assert list(knob.GetAttribute("contactbench:jointForce").Get()) == pytest.approx([tau0])
    assert knob.GetAttribute("newton:armature").Get() == pytest.approx(1.0e-3)
    assert list(knob.GetAttribute("mjc:solreflimit").Get())[1] == pytest.approx(0.5)
    # the rack and pinion: a MuJoCo joint equality on the latch, leader the knob, ratio -r_p
    assert {"NewtonMimicAPI", "MjcEqualityJointAPI"} <= _api(latch)
    assert latch.GetRelationship("newton:mimicJoint").GetTargets() == [knob.GetPath()]
    rp = meta["handle"]["info"]["pitch_radius_m"]
    assert latch.GetAttribute("newton:mimicCoef1").Get() == pytest.approx(-rp, rel=1e-6)
    assert len(latch.GetAttribute("mjc:solref").Get()) == 2 and len(latch.GetAttribute("mjc:solimp").Get()) == 5
    # each single world-anchored joint is its own articulation root
    assert knob.HasAPI(UsdPhysics.ArticulationRootAPI) and latch.HasAPI(UsdPhysics.ArticulationRootAPI)
    # colliders: SDF settings, contact gap, condim, a physics material with Newton's contact stiffness
    housing = st.GetPrimAtPath(f"{root.GetPath()}/Static/doorknob_module_bottom")
    assert "NewtonSDFCollisionAPI" in _api(housing)
    assert housing.GetAttribute("newton:sdfMaxResolution").Get() % 8 == 0
    assert housing.GetAttribute("newton:contactGap").Get() == pytest.approx(0.0008)
    assert housing.GetAttribute("mjc:condim").Get() == 4
    mat = pxr.UsdShade.MaterialBindingAPI(housing).ComputeBoundMaterial("physics")[0]
    assert mat.GetPrim().GetAttribute("newton:contactStiffness").Get() == pytest.approx(1.0e7)
    hull = [p for p in _prims(st, "Mesh") if p.GetName().startswith("grip_hull")]
    assert hull and all(UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() == "convexHull" for p in hull)
    assert all(p.GetAttribute("purpose").Get() == "guide" for p in hull)
    # collision filters between the moving parts and their housing
    assert any(UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets() for p in st.Traverse()
               if p.HasAPI(UsdPhysics.FilteredPairsAPI))
    # bodies: mass, centre of mass, the full inertia tensor
    body = st.GetPrimAtPath(f"{root.GetPath()}/knob")
    assert UsdPhysics.MassAPI(body).GetMassAttr().Get() > 0 and len(body.GetAttribute("newton:inertia").Get()) == 6


def test_push_dial_stage_has_a_d6_knob_and_the_calibration(stages):
    st = Usd.Stage.Open(str(stages["push"]))
    meta = kusd.read_meta(stages["push"])
    knob = st.GetPrimAtPath(f"{meta['root']}/Joints/knob")
    assert knob.GetTypeName() == "PhysicsJoint"
    limits = {s.split(":")[-1]: UsdPhysics.LimitAPI(knob, s.split(":")[-1]) for s in _api(knob)
              if s.startswith("PhysicsLimitAPI:")}
    assert set(limits) == {"transX", "transY", "transZ", "rotX", "rotY", "rotZ"}
    for locked in ("transX", "transY", "rotX", "rotY"):
        assert limits[locked].GetLowAttr().Get() > limits[locked].GetHighAttr().Get()
    assert limits["transZ"].GetLowAttr().Get() == pytest.approx(-meta["handle"]["info"]["press_stroke_mm"] * 1e-3,
                                                                rel=1e-5)
    assert math.isinf(limits["rotZ"].GetHighAttr().Get())
    # POSITION drives with zero stiffness: the mode is kept for the helper
    assert list(knob.GetAttribute("contactbench:actuatorMode").Get()) == [1, 1]
    cal = meta["calibration"]
    assert cal["impratio"] == 10.0 and cal["hard_joint_limits"] and cal["substeps"] == 32
    assert meta["rig"]["hard_joint_limits"] and meta["rig"]["solver_options"] == {"impratio": 10.0}
    plate = st.GetPrimAtPath(f"{meta['root']}/Joints/plate_posx")
    assert list(plate.GetAttribute("mjc:solreflimit").Get()) == pytest.approx([2.0 * cal["timestep"], 0.5])


def test_dial_pads_are_authored_at_zero_coordinates_with_their_initial_position(stages):
    st = Usd.Stage.Open(str(stages["dial"]))
    meta = kusd.read_meta(stages["dial"])
    pad = st.GetPrimAtPath(f"{meta['root']}/Joints/pad_0")
    q0 = pad.GetAttribute("newton:linear:position").Get()
    assert 0.0005 < q0 < 0.001
    assert pad.GetAttribute("state:linear:physics:position").Get() == pytest.approx(q0)
    assert f"{meta['root']}/pad_0" in meta["record"]["zero_pose_bodies"]


def test_free_body_stage(stages):
    st = Usd.Stage.Open(str(stages["cup"]))
    meta = kusd.read_meta(stages["cup"])
    bodies = [p for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert len(bodies) == 1 and not _prims(st, "PhysicsRevoluteJoint")
    mass = UsdPhysics.MassAPI(bodies[0]).GetMassAttr().Get()
    assert mass == pytest.approx(meta["record"]["bodies"][str(bodies[0].GetPath())]["mass"], rel=1e-6)
    assert json.loads(st.GetDefaultPrim().GetCustomDataByKey("contactbench")["handle"])["key"] == "cup"
    assert not _prims(st, api="PhysicsArticulationRootAPI")          # a lone free body: no articulation root


def test_floating_base_stage_roots_the_articulation_on_its_body(stages):
    """The Ethernet plug: no prim for its free joint, the articulation root on the plug body, the latch tab's
    revolute joint (spring drive, hard stop, armature) to the plug, and the handle's free-joint role kept."""
    st = Usd.Stage.Open(str(stages["ethernet"]))
    meta = kusd.read_meta(stages["ethernet"])
    hd, rec = meta["handle"], meta["record"]
    role = f"{hd['label']}/plug"
    plug, hinge = hd["bodies"][role], hd["joints"][f"{role}/tab"]
    assert hd["joints"][role] == kusd.FREE_REF + plug and list(rec["free_joints"]) == [kusd.FREE_REF + plug]
    assert [str(p.GetPath()) for p in _prims(st, api="PhysicsArticulationRootAPI")] == [plug]
    assert all(UsdPhysics.Joint(p).GetBody0Rel().GetTargets() for p in st.Traverse() if p.IsA(UsdPhysics.Joint))
    tab = st.GetPrimAtPath(hinge)
    assert tab.IsA(UsdPhysics.RevoluteJoint)
    assert [str(t) for t in UsdPhysics.Joint(tab).GetBody0Rel().GetTargets()] == [plug]
    assert UsdPhysics.RevoluteJoint(tab).GetUpperLimitAttr().Get() == pytest.approx(hd["params"]["tab_limit_deg"][1],
                                                                                    abs=0.01)
    assert UsdPhysics.DriveAPI(tab, "angular").GetStiffnessAttr().Get() == pytest.approx(
        hd["params"]["ke_N_m_per_rad"] * math.pi / 180.0, rel=1e-6)
    assert tab.GetAttribute("newton:armature").Get() == pytest.approx(hd["params"]["armature_kg_m2"])
    assert [kusd.FREE_REF + plug, hinge] in rec["articulations"] and meta["rig"]["hard_joint_limits"]


def test_floating_base_moves_with_the_asset(stages):
    """Added elsewhere (turned 90 deg, moved), the plug's free joint carries the plug's new pose, the tab follows,
    and the free joint and the hinge stay one articulation."""
    pytest.importorskip("newton_usd_schemas")
    import newton
    meta = kusd.read_meta(stages["ethernet"])
    rec, hd = meta["record"], meta["handle"]
    role = f"{hd['label']}/plug"
    c, s = math.cos(math.pi / 2), math.sin(math.pi / 2)
    X = np.array([[c, -s, 0.0, 0.4], [s, c, 0.0, -0.2], [0.0, 0.0, 1.0, 0.75], [0.0, 0.0, 0.0, 1.0]])
    with wp.ScopedDevice("cpu"):
        builder = newton.ModelBuilder()
        h = kusd.add_asset_usd(builder, stages["ethernet"], xform=wp.transform(wp.vec3(*X[:3, 3]),
                                                                              wp.quat(*kusd._seven(X)[3:])))
    jf, jt = h.joints[role], h.joints[f"{role}/tab"]
    assert builder.joint_type[jf] == newton.JointType.FREE and builder.joint_parent[jf] == -1
    assert builder.joint_articulation[jf] == builder.joint_articulation[jt] >= 0
    for r in (role, f"{role}/tab"):
        want = X @ kusd._mat(rec["bodies"][hd["bodies"][r]]["q"])
        assert np.allclose(kusd._mat(builder.body_q[h.bodies[r]]), want, atol=1e-6), r
    q0 = builder.joint_q_start[jf]
    assert np.allclose(kusd._mat(builder.joint_q[q0:q0 + 7]), kusd._mat(builder.body_q[h.bodies[role]]), atol=1e-6)
    assert np.allclose(h.pose, X @ np.asarray(hd["cad_pose"]), atol=1e-12)


def test_an_export_leaves_no_sdf_less_meshes_behind():
    """The exporter builds the asset with SDF bakes deferred; the Ethernet plug's mesh cache keeps those meshes apart
    from the ones a simulated build gets (with SDFs)."""
    from force_bench.assets import defer_sdf_builds
    from force_bench.kit.mechanisms import ethernet as E
    with wp.ScopedDevice("cpu"), defer_sdf_builds():
        assert E._plug_pieces()["body_mesh"].sdf is None
    if not wp.is_cuda_available():
        pytest.skip("SDF bakes need CUDA")
    assert all(E._plug_pieces()[k].sdf is not None for k in ("body_mesh", "root_mesh", "tab_mesh"))


def _free_box_first(rig):
    """A free body ahead of the asset: its joint has 7 coordinates and 6 dofs, so coordinate and dof indices of
    the asset's joints differ by one (joint_target_q is per coordinate, the gains per dof)."""
    b = rig.builder.add_body(xform=wp.transform(wp.vec3(0.5, 0.5, 0.05), wp.quat_identity()), label="box")
    rig.builder.add_shape_box(b, hx=0.01, hy=0.01, hz=0.01)


def _python_builder(key, choice):
    from force_bench.assets import defer_sdf_builds
    from force_bench.rig import Rig
    rig = Rig("py")
    _free_box_first(rig)
    n0 = (rig.builder.body_count, rig.builder.joint_count)
    with defer_sdf_builds():
        h = add_asset(rig, key, choice, pose=place_on_table(key, choice))
    return rig, h, n0


@pytest.mark.parametrize("name,key,choice", [("doorknob", "doorknob-module", {"Door handle geometry": "Handle 1"}),
                                             ("push", "dial-push-down-to-turn", {}), ("dial", "dial", {}),
                                             ("ethernet", "ethernet-plug-and-slot", {})])
def test_round_trip_into_a_builder_matches_add_asset(stages, name, key, choice):
    pytest.importorskip("newton_usd_schemas")
    import newton
    from force_bench.rig import Rig
    with wp.ScopedDevice("cpu"):
        a, ha, _ = _python_builder(key, choice)
        rig = Rig("usd")
        _free_box_first(rig)
        h = kusd.add_asset_usd(rig, stages[name], pose=place_on_table(key, choice))
    A, B = a.builder, rig.builder
    assert set(h.bodies) == set(ha.bodies) and set(h.joints) == set(ha.joints)
    assert h.params == json.loads(json.dumps(ha.params, default=kusd._json_default))
    for role, ja in ha.joints.items():
        jb = h.joints[role]
        assert A.joint_type[ja] == B.joint_type[jb]
        if A.joint_type[ja] == newton.JointType.FREE:     # (named by index; the floating base's coordinates: below)
            assert A.body_label[A.joint_child[ja]] == B.body_label[B.joint_child[jb]] and B.joint_parent[jb] == -1
            q0a, q0b = A.joint_q_start[ja], B.joint_q_start[jb]
            assert B.joint_q[q0b:q0b + 7] == pytest.approx(A.joint_q[q0a:q0a + 7], abs=1e-9), role
        else:
            assert A.joint_label[ja] == B.joint_label[jb]
        da, db = A.joint_qd_start[ja], B.joint_qd_start[jb]
        for k in range(sum(A.joint_dof_dim[ja])):
            for f in ("joint_target_ke", "joint_target_kd", "joint_target_mode", "joint_limit_lower",
                      "joint_limit_upper", "joint_armature", "joint_friction", "joint_f"):
                assert float(getattr(B, f)[db + k]) == pytest.approx(float(getattr(A, f)[da + k]), rel=1e-6), (role, f)
            qa, qb = A.joint_q_start[ja] + k, B.joint_q_start[jb] + k
            assert B.joint_target_q[qb] == pytest.approx(A.joint_target_q[qa], abs=1e-12), role
            assert B.joint_q[qb] == pytest.approx(A.joint_q[qa], abs=1e-9), role
        assert B.joint_articulation[jb] >= 0
    # the same joints share an articulation (the floating base: the plug's free joint and the tab's hinge)
    assert {(r, s) for r, i in ha.joints.items() for s, j in ha.joints.items()
            if A.joint_articulation[i] == A.joint_articulation[j]} == \
        {(r, s) for r, i in h.joints.items() for s, j in h.joints.items()
         if B.joint_articulation[i] == B.joint_articulation[j]}
    for role, ba in ha.bodies.items():
        bb = h.bodies[role]
        assert B.body_mass[bb] == pytest.approx(A.body_mass[ba], rel=1e-6)
        assert np.allclose(np.asarray(B.body_inertia[bb]), np.asarray(A.body_inertia[ba]), rtol=1e-5, atol=1e-12)
        assert np.allclose(kusd._mat(B.body_q[bb]), kusd._mat(A.body_q[ba]), atol=1e-6)
    assert len(B.constraint_mimic_joint0) == len(A.constraint_mimic_joint0)
    assert list(B.articulation_start) == sorted(B.articulation_start)       # what ModelBuilder.finalize requires
    assert rig.hard_joint_limits == a.hard_joint_limits and rig.solver_options == a.solver_options
    # the stage alone (Newton's importer without the helper) drops only what the helper is for (dof_armature: a D6
    # whose dofs differ, the push dial's knob, NewtonJointAPI holds one armature per joint)
    assert set(h.import_report) <= {"articulation", "actuator_mode", "joint_force", "sdf_resolution", "joint_frame",
                                    "body_initial_pose", "dof_limit_lower", "dof_limit_upper", "convex_hull",
                                    "shape_transform", "dof_armature"}



def test_stages_are_valid_usdphysics(stages):
    """Plain UsdPhysics (what Isaac Sim and other tools read) parses every body, shape, joint and material."""
    import warnings
    counts, arts = {}, {}
    for name, path in stages.items():
        st = Usd.Stage.Open(str(path))
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", DeprecationWarning)      # renamed in recent usd-core, same parser
            res = UsdPhysics.LoadUsdPhysicsFromRange(st, ["/"])
        for kind, (paths, descs) in res.items():
            assert all(d.isValid for d in descs), (name, str(kind))
            counts.setdefault(name, {})[str(kind)] = len(paths)
            if kind == UsdPhysics.ObjectType.Articulation:
                arts[name] = [({str(b) for b in d.articulatedBodies}, {str(j) for j in d.articulatedJoints})
                              for d in descs]
    assert counts["doorknob"]["RigidBody"] == 2 and counts["doorknob"]["RevoluteJoint"] == 1
    assert counts["doorknob"]["PrismaticJoint"] == 1 and counts["push"]["D6Joint"] == 1
    assert counts["cup"]["RigidBody"] == 1 and counts["cup"]["MeshShape"] == 1
    # the floating base: one articulation of the plug, its tab and the hinge between them
    hd = kusd.read_meta(stages["ethernet"])["handle"]
    role = f"{hd['label']}/plug"
    assert arts["ethernet"] == [({hd["bodies"][role], hd["bodies"][f"{role}/tab"]}, {hd["joints"][f"{role}/tab"]})]


def test_one_stage_settles_on_the_table(tmp_path):
    """kit check --from-usd on one free asset: the stage's SDF is built at finalize and the card comes to rest."""
    pytest.importorskip("newton_usd_schemas")
    if not wp.is_cuda_available():
        pytest.skip("signed-distance collision requires CUDA")
    from force_bench.kit.check import check_choice, usd_stage_path
    kusd.export_variant("card", {}, usd_stage_path("card", {}, tmp_path / "stages"))
    report = check_choice("card", {}, out_root=tmp_path, render=False, source="usd", usd_dir=tmp_path / "stages")
    assert report["pass"], report["checks"]
