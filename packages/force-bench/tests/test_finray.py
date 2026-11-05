"""The Fin Ray finger: measured profile, adapter frames, stiffness references, PRB layout, deformation, envelope.

CPU only:  python -m pytest packages/force-bench/tests/test_finray.py
(the GPU probe is ``python -m force_bench.finray.probe``).
"""

import json

import numpy as np
import pytest

from force_bench.finray import geometry as G
from force_bench.finray import prb
from force_bench.finray.frame_fe import press_table


def test_profile_matches_the_step():
    p = G.extract_profile()
    assert len(p["ribs"]) == 15 and p["n_cavities"] == 16
    assert p["contact_wall_t"] == pytest.approx(1.8, abs=0.01)
    # the profile x the 25.8 mm width is the STL's volume
    assert p["area_mm2"] * G.WIDTH / 1000.0 == pytest.approx(G.VOLUME_CM3, rel=1e-3)
    t = np.asarray(p["back_t"])
    assert 5.9 < t[0] < 6.1 and 2.0 < t[-1] < 3.0          # back wall 6 -> ~2.5 mm
    angles = [r["angle_deg"] for r in p["ribs"]]
    assert angles[0] == pytest.approx(0.0, abs=0.5) and angles[-1] < -50.0


def test_cached_profile_is_current():
    cached = json.loads(G.PROFILE_JSON.read_text())
    fresh = G.extract_profile()
    assert np.allclose([r["inner"] for r in cached["ribs"]], [r["inner"] for r in fresh["ribs"]])
    assert np.allclose([r["outer"] for r in cached["ribs"]], [r["outer"] for r in fresh["ribs"]])


def test_adapter_spec_matches_the_simulation_frames():
    spec = json.loads((G.FINRAY_DIR / "adapter_spec.json").read_text())
    d, p = spec["derived"], spec["parameters"]
    assert d["z_base"] == pytest.approx(G.Z_BASE)
    assert d["tab_hole_z"] == pytest.approx(G.TAB_HOLE_Z) == pytest.approx(12.0)
    assert d["tab_inner_y"] == pytest.approx(G.TAB_INNER_Y)
    assert p["inner_y"] == pytest.approx(G.ADAPTER_INNER_Y)
    assert d["cheek_x0"] == pytest.approx(G.WIDTH / 2 + G.CHEEK_CLEAR)
    # the finger tip is 101.76 mm below the stock TCP
    assert G.TIP_BELOW_TCP == pytest.approx(0.10176, abs=1e-5)


def test_mount_puts_the_contact_face_on_the_pad_plane():
    v, _ = G.finger_mesh_vertices_m()
    assert v[:, 1].min() == pytest.approx(0.0, abs=1e-6)           # contact face y = 0
    assert v[:, 0].min() == pytest.approx(-0.0129, abs=1e-5)       # centred across the finger
    assert v[:, 2].max() == pytest.approx(G.TIP_Z / 1000.0, abs=1e-5)


def test_frame_model_stiffness_and_the_lug():
    _, rows = press_table(heights=[-60.0, -90.0])
    k60, k90 = rows[0]["k_N_per_mm"], rows[1]["k_N_per_mm"]
    assert 0.9 < k60 < 1.3 and 0.4 < k90 < 0.7                     # N/mm at E = 26 MPa
    # linear theory: the tip moves AWAY from a mid-length load (no Fin Ray curl)
    assert rows[0]["tip_wrap_per_mm"] < 0.0
    _, lug = press_table(heights=[-60.0], fixed_extra="lug")
    assert lug[0]["k_N_per_mm"] > 1.5 * k60


def test_prb_layout_is_a_closed_linkage():
    L = prb.layout()
    by = L.by_name()
    kinds = [l.kind for l in L.links]
    assert kinds.count("rib") == 30 and len(L.loops) == 16          # 15 ribs in halves + the tip
    for a, b, pt in L.loops:                                        # each weld joins two link ends
        assert np.linalg.norm(by[a].p1 - pt) < 1e-6 or np.linalg.norm(by[a].p0 - pt) < 1e-6
        assert np.linalg.norm(by[b].p1 - pt) < 1e-6 or np.linalg.norm(by[b].p0 - pt) < 1e-6
    for l in L.links:                                               # every child starts where it hangs
        assert l.k > 0.0
        if l.parent is not None and l.kind != "rib":
            assert np.linalg.norm(by[l.parent].p1 - l.p0) < 1e-6
    m = prb.link_masses(L)
    assert 0.040 < sum(m.values()) < 0.055                          # kg; + ~12 g base block = the 57 g print


def test_deformation_reads_zero_at_rest_and_a_turned_tip_link():
    from force_bench.finray.deform import Deformation
    L = prb.layout()
    labels = ["fr3/fr3_leftfinger"] + [f"fr3/finray_L_{l.name}" for l in L.links]
    bq = np.zeros((len(labels), 7))
    bq[:, 6] = 1.0                                                  # identity quaternions (x, y, z, w)
    for i, l in enumerate(L.links, start=1):                        # link frames sit at p0, finger-aligned
        bq[i, 1:3] = np.array([l.p0[0], G.Z_BASE - l.p0[1]]) / 1000.0
    D = Deformation(labels, L)
    r = D.frame(bq)["L"]
    assert r["hinge_deg"] == pytest.approx(0.0, abs=1e-9)
    assert r["tip_mm"] == pytest.approx(0.0, abs=1e-9)
    # the last contact-wall link turned 5 deg about the width axis: the tip moves on an arc of its length
    last = [l for l in L.links if l.kind == "contact"][-1]
    a = np.radians(5.0)
    bq[labels.index(f"fr3/finray_L_{last.name}"), 3:7] = [np.sin(a / 2), 0.0, 0.0, np.cos(a / 2)]
    r = D.frame(bq)["L"]
    assert r["hinge_deg"] == pytest.approx(5.0, abs=1e-6) and r["wall_deg"] == pytest.approx(5.0, abs=1e-6)


def test_rest_envelope_holds_the_finger_and_puts_its_face_on_the_pad_plane():
    from force_bench.finray.embodiment import FINGER_TO_TCP, FinRayFingers, rest_points
    from force_bench import embodiments as E
    f = E.load("finray")
    assert isinstance(f, FinRayFingers) and f.face_offset == 0.0
    P = rest_points()
    assert P[:, 1].min() == pytest.approx(0.0, abs=1e-9)           # nothing in front of the contact face
    # the linkage ends ~1 mm above the printed tip (the solid wedge below the walls' junction is not modelled)
    assert 0.0 < G.TIP_BELOW_TCP - f.tip_below_tcp < 0.0015
    q = 0.006
    boxes = f.boxes(q)
    z = P[:, 2] - FINGER_TO_TCP
    for s in (+1.0, -1.0):                                          # both fingers, every colliding corner inside
        pts = np.column_stack([P[:, 0], s * (q + P[:, 1]), z])
        inside = np.zeros(len(pts), bool)
        for lo, hi in boxes:
            inside |= np.all((pts >= lo - 1e-9) & (pts <= hi + 1e-9), axis=1)
        assert inside.all()
    assert max(hi[2] for _, hi in boxes) == pytest.approx(f.tip_below_tcp)


def test_underside_and_the_flat_face_end():
    from force_bench import embodiments as E
    f = E.load("finray")
    # the flat face ends ~0.5 mm above the lowest point (the back wall's last corner)
    assert 0.0003 < f.face_bottom_above_tip < 0.0008
    assert f.underside(0.0) == pytest.approx(f.face_bottom_above_tip, abs=1e-4)
    h = f.underside(np.arange(0.0, 0.004, 0.0001))
    assert h.min() == pytest.approx(0.0, abs=1e-4)                  # the lowest point, 1.5-2 mm behind the face
    assert np.all(np.isfinite(h)) and f.underside(0.05) == np.inf and f.underside(-0.001) == np.inf
    assert f.underside(0.030) > 0.040                               # the back flank rises away from the tip

