"""The USB slot model (force_bench.kit.mechanisms.usb_slot): geometry it is built from, the seat pose, the jaw
crop, and the flexure-stiffness estimate.  CPU only (no simulation)."""

import numpy as np
import pytest
import trimesh

from force_bench.kit import load_kit
from force_bench.kit.mechanisms import usb_slot as U

STICKS = ("usb-stick-1", "usb-stick-2", "usb-stick-3", "usb-stick-4")


def _slot_parts():
    a = load_kit()["usb-slot"]
    return a.part("usb_slot_shell").mesh(), a.part("usb_slot_center").mesh()


def test_measured_geometry():
    m = U.measure()
    assert m["mouth_half_x_mm"] == pytest.approx(6.175, abs=1e-3)
    assert m["mouth_half_y_mm"] == pytest.approx(2.675, abs=1e-3)
    assert m["key_top_z_mm"] == pytest.approx(U.KEY_TOP_Z * 1e3, abs=1e-3)
    assert m["tip_stop_z_mm"] == pytest.approx(U.SEAT_TIP_Z * 1e3, abs=1e-3)
    for label, j in m["jaws"].items():
        assert j["interference_mm"] == pytest.approx(U.indent_mm(label), abs=1e-3)


@pytest.mark.parametrize("key", STICKS)
def test_seat_pose_touches_without_overlap(key):
    """Seated: no overlap with the shell or centre (sticks rest on the stops at zero gap); the jaws overlap the plug
    by the indent."""
    shell, center = _slot_parts()
    st = load_kit()[key].parts()[0].mesh()
    st.apply_transform(U.seat_pose(key))
    pts = np.vstack([st.vertices, st.sample(3000, seed=0)])
    for m in (shell, center):
        lo, hi = m.bounds
        sel = np.all((pts > lo - 1e-4) & (pts < hi + 1e-4), axis=1)
        assert trimesh.proximity.signed_distance(m, pts[sel]).max() < 2e-5
    jaw = U.split_spring(load_kit()["usb-slot"].part("usb_slot_spring_0.4mm_indent", "0.4mm").mesh())["jaw_right"]
    assert trimesh.proximity.signed_distance(jaw, pts).max() == pytest.approx(0.0004, abs=3e-5)
    # the plug tip on the stop, the stick centred on the slot axis
    assert st.bounds[0][2] == pytest.approx(U.SEAT_TIP_Z, abs=1e-6)
    assert np.abs(0.5 * (st.bounds[0][:2] + st.bounds[1][:2])).max() < 2e-5


@pytest.mark.parametrize("key", STICKS)
def test_intrusion_sees_a_stick_through_the_wall(key):
    """The QA's slot-body depth is zero for a stick on its way in or seated, and large for one pushed through the top
    wall, where a signed distance into the 1 mm wall cannot exceed 0.5 mm."""
    st = load_kit()[key].parts()[0].mesh()
    pts = np.vstack([st.vertices, st.sample(3000, seed=0)])
    for lift in (0.0, 0.004, 0.012, 0.03):
        T = U.seat_pose(key, lift=lift)
        assert U.intrusion(pts @ T[:3, :3].T + T[:3, 3]).max() < 1e-6
    T = U.seat_pose(key)
    T[:3, 3] += [0.010, 0.0, 0.0]                 # 10 mm off the mouth: through the top wall
    assert U.intrusion(pts @ T[:3, :3].T + T[:3, 3]).max() > 0.003


def test_flipped_stick_lands_on_the_key():
    _, center = _slot_parts()
    st = load_kit()["usb-stick-1"].parts()[0].mesh()
    st.apply_transform(U.seat_pose("usb-stick-1", flipped=True, lift=U.KEY_TOP_Z - U.SEAT_TIP_Z + 1e-4))
    pts = st.vertices
    assert trimesh.proximity.signed_distance(center, pts).max() < 0.0           # just above the key: free
    st.apply_translation([0.0, 0.0, -0.0003])
    pts = np.vstack([st.vertices, st.sample(3000, seed=0)])
    assert trimesh.proximity.signed_distance(center, pts).max() > 0.0001       # 0.2 mm lower: into the key


def test_jaw_crop_is_watertight_and_faces_the_plug():
    a = load_kit()["usb-slot"]
    for v in a.groups[0].variants:
        pieces = U.split_spring(v.parts[0].mesh())
        for n in ("jaw_right", "jaw_left"):
            assert pieces[n].is_watertight
        assert U._jaw_face(pieces["jaw_right"]) == pytest.approx(U.PLUG_HALF_W - U.indent_mm(v.label) * 1e-3,
                                                                 abs=1e-6)
        assert pieces["jaw_right"].bounds[1][0] == pytest.approx(U.JAW_CROP_X[1], abs=1e-9)


def test_flexure_stiffness_estimate():
    k = U.fem_flexure_stiffness(h_mm=0.05)          # coarse grid: within 5 % of the converged 66 N/mm
    assert k == pytest.approx(U.KE_JAW, rel=0.05)
