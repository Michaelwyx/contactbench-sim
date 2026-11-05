"""The Ethernet plug-and-slot model (force_bench.kit.mechanisms.ethernet): the geometry it is built from, the plug's
split at the tab root, the seat pose, the latch kinematics and the tab-stiffness estimate.  CPU only (no simulation).

    python -m pytest packages/force-bench/tests/test_kit_ethernet.py
"""

import numpy as np
import pytest
import trimesh

from force_bench.kit import load_kit
from force_bench.kit.mechanisms import ethernet as E


@pytest.fixture(scope="module")
def parts():
    a = load_kit()["ethernet-plug-and-slot"]
    return {p.name: p.mesh() for p in a.parts()}


def test_measured_geometry():
    m = E.measure()
    assert [c for pair in m["pockets_x_mm"] for c in pair] == pytest.approx(
        [-36, -24, -21, -9, -6, 6, 9, 21, 24, 36], abs=1e-3)       # five 12 mm ports, 15 mm pitch
    assert m["mouth_y_mm"] == pytest.approx(E.MOUTH_Y * 1e3, abs=1e-3)
    assert m["insert"]["back_wall_y_mm"] == pytest.approx(E.BACK_WALL_Y * 1e3, abs=1e-3)
    assert m["insert"]["floor_z_mm"] == pytest.approx(E.FLOOR_Z * 1e3, abs=1e-3)
    assert m["insert"]["lip_bottom_z_mm"] == pytest.approx(20.3, abs=1e-3)
    assert m["insert"]["lip_x_mm"] == pytest.approx([-5.8, -3.5, 3.5, 5.8], abs=1e-3)
    assert m["tab"]["slope_deg"] == pytest.approx(np.degrees(E.TAB_ALPHA), abs=0.01)
    assert m["tab"]["thickness_normal_mm"] == pytest.approx(E.TAB_T * 1e3, abs=0.005)
    assert m["tab"]["wide_x_mm"] == pytest.approx([-20.5, -9.5], abs=1e-3)
    assert m["tab"]["narrow_x_mm"] == pytest.approx([-18.0, -12.0], abs=1e-3)
    assert m["tab"]["shoulder_y_mm"] == pytest.approx(E.SHOULDER_Y * 1e3, abs=0.01)
    assert m["clearance_x_per_side_mm"] == pytest.approx(0.5)
    assert m["seat_travel_mm"] == pytest.approx(-E.SEAT_DY * 1e3, abs=0.01)


def test_latch_kinematics_match_the_model_constants():
    k = E.latch_kinematics(step=0.1e-3)
    assert k["seat_deg"] == pytest.approx(np.degrees(E.SEAT_TAB_RAD), abs=0.05)
    assert k["max_deg"] == pytest.approx(15.2, abs=0.1)          # the shoulder under the lips
    assert k["max_deg"] < np.degrees(E.PASS_RAD) < np.degrees(E.TAB_LIMIT[1])
    assert k["stop_deg"] == pytest.approx(np.degrees(E.TAB_LIMIT[1]), abs=0.05)
    assert 9.5 < k["first_contact_mm_from_seat"] < 10.5
    assert 0.6 < k["withdrawal_play_mm"] < 1.1


def test_plug_split_is_watertight_and_complete(parts):
    pieces = E.split_plug(parts["ethernet_plug"])
    assert all(m.is_watertight for m in pieces.values())
    total = sum(m.volume for m in pieces.values())
    assert total == pytest.approx(parts["ethernet_plug"].volume, rel=1e-6)
    tab = pieces["tab"]
    assert tab.bounds[0][2] >= E.BODY_TOP_Z                      # the tab lies above the body's top face
    assert tab.bounds[1][1] == pytest.approx(0.04295, abs=2e-5)    # ... out to its free end


def test_seat_touches_the_back_wall_without_overlap(parts):
    body = E.split_plug(parts["ethernet_plug"])["body"]
    pts = np.vstack([body.vertices, body.sample(4000, seed=0)])
    for port in (1, 3, 5):
        T = E.seat_transform(port)
        p = trimesh.transform_points(pts, T)
        ins = parts["ethernet_slot"].copy()
        ins.apply_translation([E.PORT_X[port - 1], 0.0, 0.0])
        for m in (ins, parts["ethernet_slot_shell"]):
            assert trimesh.proximity.signed_distance(m, p).max() < 2e-5
        # 0.1 mm deeper is into the back wall
        p2 = trimesh.transform_points(pts, E.seat_transform(port, lift=-1e-4))
        assert trimesh.proximity.signed_distance(ins, p2).max() > 5e-5


def test_seated_tab_rests_on_the_roof(parts):
    """At the seated rest angle the tab touches the roof ramp; 0.8 deg less is into it."""
    tab = E.split_plug(parts["ethernet_plug"])["tab"]
    pts = np.vstack([tab.vertices, tab.sample(6000, seed=1)])
    ins = parts["ethernet_slot"]
    piv = E.pivot_cad()

    def depth(theta):
        R = trimesh.transformations.rotation_matrix(-theta, (1.0, 0.0, 0.0), point=piv)
        p = trimesh.transform_points(trimesh.transform_points(pts, R), E.seat_transform(3))
        return trimesh.proximity.signed_distance(ins, p).max()
    assert depth(E.SEAT_TAB_RAD + np.radians(0.1)) < 1e-5
    assert depth(E.SEAT_TAB_RAD - np.radians(0.8)) > 5e-5


def test_tab_stiffness_estimate():
    r = E.fem_tab_stiffness(h_mm=0.05)          # coarse grid: within 2 % of the converged 0.72 N.m/rad
    assert r["K_N_m_per_rad"] == pytest.approx(E.KE_TAB, rel=0.02)
    assert r["k_N_per_m"] == pytest.approx(8270.0, rel=0.02)
