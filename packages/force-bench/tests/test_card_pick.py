"""Regression for thin-card contacts in the production card_pick scene.

Run explicitly with:
    python -m pytest packages/force-bench/tests/test_card_pick.py

Requires the simulation environment and CUDA for the holder SDF.
"""

import numpy as np
import pytest
import warp as wp
import newton

from force_bench import rig as rig_module
from force_bench.tasks import card_pick


@pytest.fixture
def scene(monkeypatch):
    if not wp.is_cuda_available():
        pytest.skip("card_pick's holder SDF requires CUDA")

    class SceneReady(Exception):
        pass

    scene = {}

    def capture(rig, duration, control_cb, extra_forces_cb=None, **kwargs):
        scene.update(rig=rig, control=control_cb)
        raise SceneReady

    # Build the real task, including its controller and collision filters.
    # Suppress only rendering and stop before the long expert rollout/QA.
    monkeypatch.setattr(rig_module, "RolloutRecorder", lambda *args, **kwargs: None)
    monkeypatch.setattr(rig_module.Rig, "run", capture)
    if card_pick.GS.GS_ON:
        monkeypatch.setattr(card_pick.GS.GelSightRig, "run", capture)
    with pytest.raises(SceneReady):
        card_pick.main()
    return scene["rig"], scene["control"]


def test_contacts_remain_fresh_after_settling(scene):
    rig, control = scene
    assert rig.collide_every_substep  # also the default inherited by teleop

    # A motionless deck used to disable collision refresh after T_SETTLE,
    # leaving all 64 MuJoCo steps with the same contact geometry.
    for t in (0.0, card_pick.T_SETTLE, card_pick.T_SETTLE + 1.0 / rig.fps):
        control(t)
        assert rig.collide_every_substep, f"stale contacts enabled at t={t}"

    # Centre displacement also misses a rotation moving the thin slab's
    # edge farther than its contact gap.  Exercise the actual callback.
    q = rig.s0.body_q.numpy()
    picked = rig.body_index("card_picked")
    p0 = q[picked, :3].copy()
    rotation = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), 0.04)
    old_q = wp.quat(*q[picked, 3:])
    new_q = wp.mul(rotation, old_q)
    tip = wp.vec3(0.0, 0.0, card_pick.CARD_H / 2.0)
    edge_motion = np.linalg.norm(
        np.asarray(wp.quat_rotate(new_q, tip)) - np.asarray(wp.quat_rotate(old_q, tip)))
    assert edge_motion > card_pick.GAP
    q[picked, 3:] = np.asarray(new_q)
    rig.s0.body_q.assign(q)
    control(card_pick.T_SETTLE + 2.0 / rig.fps)
    np.testing.assert_array_equal(rig.s0.body_q.numpy()[picked, :3], p0)
    assert rig.collide_every_substep


def test_card_geometry_and_symmetric_jaw_commands(scene):
    rig, control = scene
    scale = rig.model.shape_scale.numpy()
    flags = rig.model.shape_flags.numpy()
    labels = list(rig.model.shape_label)
    cad_thickness = card_pick.single_card_tm().extents[0]
    for name in ("card_a", "card_b", "card_c", "card_picked"):
        sid = rig.shapes[name][0]
        assert 2 * scale[sid, 0] == pytest.approx(cad_thickness, abs=1e-8)
        assert not flags[sid] & int(newton.ShapeFlags.VISIBLE)
        visual = labels.index(name + "_visual")
        assert not flags[visual] & int(newton.ShapeFlags.COLLIDE_SHAPES)

    # Keep the robot's mechanical coupling, and exercise the actual expert
    # through approach, press, slide and the start of closure.
    assert rig.model.constraint_mimic_enabled.numpy().any()
    for t in (0.0, 0.6, 1.5, 2.4, 3.0, 7.2, 7.5):
        control(t)
        target = rig.control.joint_target_q.numpy()
        assert target[7] == pytest.approx(target[8]), f"conflicting jaw targets at {t}"


@pytest.mark.skipif(card_pick.GS.GS_ON, reason="bare-card settling regression")
def test_unloaded_cards_settle_without_persistent_motion(scene):
    rig, control = scene
    ids = [rig.body_index(n) for n in ("card_a", "card_b", "card_c", "card_picked")]
    initial = rig.s0.body_q.numpy()[ids, :3]
    dt = 1.0 / rig.fps / rig.substeps

    def frame():
        for _ in range(rig.substeps):
            rig.s0.clear_forces()
            rig.pipeline.collide(rig.s0, rig.contacts)
            rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, dt)
            rig.s0, rig.s1 = rig.s1, rig.s0

    control(0.0)  # hold the gripper above the cards for the entire test
    frame()       # warm kernels before capturing a production physics frame
    with wp.ScopedCapture() as capture:
        frame()
    positions = [initial, rig.s0.body_q.numpy()[ids, :3]]
    for _ in range(rig.fps - 1):
        wp.capture_launch(capture.graph)
        rig.solver.update_contacts(rig.contacts, rig.s0)
        positions.append(rig.s0.body_q.numpy()[ids, :3])
    positions = np.asarray(positions)
    assert np.linalg.norm(positions - initial, axis=-1).max() < 0.00015
    settled = positions[int(card_pick.T_SETTLE * rig.fps):]
    assert np.linalg.norm(settled - settled[0], axis=-1).max() < 0.00005


@pytest.mark.skipif(not card_pick.GS.GS_ON, reason="requires TB_GELSIGHT=1")
def test_tactile_approach_uses_the_sensing_face_and_reachable_jaw_targets(scene):
    rig, control = scene
    # The end of descent must leave 4 mm of clearance to the REAL gel
    # face, including its proud offset, with both jaws within their stops.
    control(card_pick.T_SETTLE + card_pick.T_DESCEND)
    targets = rig.control.joint_target_q.numpy()
    assert 0 <= targets[7] <= .04
    assert targets[7] == pytest.approx(targets[8])
    shape = rig.shapes["card_picked"][0]
    resolution = rig.builder.shape_sdf_max_resolution[shape]
    assert resolution * card_pick.CARD_T / card_pick.single_card_tm().extents.max() >= 6
    q = rig.model.joint_q.numpy()
    q[:9] = targets[:9]
    joints = wp.array(q, dtype=wp.float32, device=rig.model.device)
    state = rig.model.state()
    newton.eval_fk(rig.model, joints, rig.model.joint_qd, state)
    bq = state.body_q.numpy()
    card_pose = bq[rig.body_index("card_picked")]
    card = wp.transform(wp.vec3(*card_pose[:3]), wp.quat(*card_pose[3:]))
    normal = np.asarray(wp.quat_rotate(wp.transform_get_rotation(card), wp.vec3(1, 0, 0)))
    front = np.asarray(wp.transform_point(card, wp.vec3(card_pick.CARD_T / 2, 0, 0)))
    gel = rig.gel_shapes["gel_left"]
    body = rig.body_index("fr3/fr3_leftfinger")
    local = rig.model.shape_transform.numpy()[gel]
    world = wp.transform_multiply(
        wp.transform(wp.vec3(*bq[body, :3]), wp.quat(*bq[body, 3:])),
        wp.transform(wp.vec3(*local[:3]), wp.quat(*local[3:])))
    face = np.asarray(wp.transform_point(world, wp.vec3(0, 0, rig.gel_face_z)))
    # The task's bare-pad offset approximates the URDF face by 85 um;
    # this tolerance allows that while catching the missing 5 mm sensor.
    assert float((face - front) @ normal) == pytest.approx(.004, abs=.00015)

    # Sensor cases keep CAD-holder collision and use no duplicate proxies.
    filters = {tuple(sorted(pair)) for pair in rig.builder.shape_collision_filter_pairs}
    holder = rig.shapes["holder"][0]
    for case in rig.case_shapes.values():
        assert tuple(sorted((holder, case))) not in filters
        for proxy in rig.shapes["holder"][1:]:
            assert tuple(sorted((proxy, case))) in filters


@pytest.mark.skipif(not card_pick.GS.GS_ON, reason="requires TB_GELSIGHT=1")
def test_tactile_64_substep_frame_can_be_captured(scene):
    rig, control = scene
    control(0.)
    dt = 1. / rig.fps / rig.substeps
    for _ in range(rig.substeps):
        rig.s0.clear_forces()
        rig.pipeline.collide(rig.s0, rig.contacts)
        rig.solver.step(rig.s0, rig.s1, rig.control, rig.contacts, dt)
        rig.s0, rig.s1 = rig.s1, rig.s0
    hydro = rig.pipeline.narrow_phase.hydroelastic_sdf
    interval = hydro._host_warning_poll_interval
    # Warmup plus capture crosses Newton's 120-call host polling interval.
    assert hydro._launch_counter < interval <= hydro._launch_counter + rig.substeps
    graph = rig._capture_graph(dt)
    assert hydro._host_warning_poll_interval == interval
    wp.capture_launch(graph)
    wp.synchronize()
    assert np.isfinite(rig.s0.body_q.numpy()).all()
