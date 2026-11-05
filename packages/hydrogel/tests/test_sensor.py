"""Behavior tests for the Newton-native hydroelastic GelSight adapter (newton 1.5)."""

from __future__ import annotations

import unittest

import numpy as np
import warp as wp

from hydrogel.sensor import GelSightConfig, build_demo_scene, set_demo_indenter_pose


@unittest.skipUnless(wp.is_cuda_available(), "Newton hydroelastic SDFs require CUDA")
class TestHydroelasticGelSight(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.config = GelSightConfig(rows=24, cols=32, sdf_max_resolution=64)
        cls.scene = build_demo_scene(cls.config)

    @staticmethod
    def sample_scene(scene, indentation_m: float, x_m: float = 0.0, velocity_x: float = 0.0):
        scene.sensor.reset()
        set_demo_indenter_pose(scene, x_m=x_m, indentation_m=indentation_m)
        scene.pipeline.collide(scene.state, scene.contacts)
        return scene.sensor.sample(
            scene.state,
            1.0 / 120.0,
            tangential_velocity_local_mps=(velocity_x, 0.0),
        )

    def sample(self, indentation_m: float, x_m: float = 0.0, velocity_x: float = 0.0):
        return self.sample_scene(self.scene, indentation_m, x_m=x_m, velocity_x=velocity_x)

    def test_separated_surface_is_zero(self) -> None:
        frame = self.sample(-0.001)
        self.assertEqual(frame.contact_face_count, 0)
        self.assertEqual(frame.total_normal_force_n, 0.0)
        self.assertFalse(frame.contact_mask.any())

    def test_deeper_indent_increases_force(self) -> None:
        shallow = self.sample(0.00045)
        deep = self.sample(0.00090)
        self.assertGreater(shallow.contact_face_count, 0)
        self.assertGreater(deep.total_normal_force_n, shallow.total_normal_force_n * 1.2)
        self.assertTrue(np.isfinite(deep.pressure_pa).all())
        self.assertGreater(float(deep.normal_displacement_m.max()), 0.0)
        cell_area = self.config.width_m * self.config.height_m / (self.config.rows * self.config.cols)
        mapped_force = float(deep.pressure_pa.sum() * cell_area)
        self.assertAlmostEqual(mapped_force, deep.total_normal_force_n, delta=1.0e-5)
        self.assertEqual(deep.mechanics_observation().shape, (3, self.config.rows, self.config.cols))

    def test_solver_override_zero_is_not_missing(self) -> None:
        sensor = self.scene.sensor
        previous = sensor._use_gpu
        try:
            for gpu in (False, True):
                sensor._use_gpu = gpu
                self.sample(0.0008)
                for force in (None, 0.0, 2.0):
                    frame = sensor.sample(self.scene.state, 1 / 120, solver_force_n=force)
                    self.assertGreater(frame.pressure_law_force_n, 0.0)
                    if force is None:
                        self.assertGreater(frame.total_normal_force_n, 0.0)
                        self.assertEqual(frame.solver_scale, 1.0)
                    else:
                        self.assertAlmostEqual(frame.total_normal_force_n, force, places=5)
                        self.assertAlmostEqual(frame.solver_scale,
                                               force / frame.pressure_law_force_n, places=5)
                    if force == 0.0:
                        self.assertFalse(frame.pressure_pa.any())
                        self.assertFalse(frame.contact_mask.any())
                        self.assertFalse(frame.shear_traction_pa.any())
                        self.assertTrue(np.isnan(frame.center_of_pressure_m).all())
                    else:
                        self.assertTrue(np.isfinite(frame.center_of_pressure_m).all())
        finally:
            sensor._use_gpu = previous

    def test_invalid_solver_force_is_rejected(self) -> None:
        for force in (-1.0, np.nan, np.inf):
            with self.assertRaises(ValueError):
                self.scene.sensor.sample(self.scene.state, 1 / 120, solver_force_n=force)

    def test_center_of_pressure_tracks_lateral_contact(self) -> None:
        centered = self.sample(0.0008, x_m=0.0)
        shifted = self.sample(0.0008, x_m=0.003)
        self.assertAlmostEqual(float(centered.center_of_pressure_m[0]), 0.0, delta=0.0008)
        self.assertGreater(float(shifted.center_of_pressure_m[0]), 0.0015)
        self.assertLess(float(shifted.center_of_pressure_m[0]), 0.0045)

    def test_shear_history_obeys_coulomb_limit(self) -> None:
        frame = self.sample(0.0008, velocity_x=0.02)
        active = frame.contact_mask
        self.assertTrue(active.any())
        self.assertGreater(float(np.abs(frame.shear_displacement_m[0, active]).max()), 0.0)
        traction = np.linalg.norm(frame.shear_traction_pa, axis=0)
        limit = self.config.friction_coefficient * frame.pressure_pa
        self.assertTrue(np.all(traction <= limit + 1.0))

    def test_pair_order_robustness(self) -> None:
        # The pressure formula indexes kh by pairs[:, 1]; guard the convention
        # by building the same scene with the indenter added BEFORE the gel so
        # the gel can land on the other side of the exported shape pairs.
        reversed_scene = build_demo_scene(self.config, gel_first=False)
        canonical = self.sample(0.0008)
        swapped = self.sample_scene(reversed_scene, 0.0008)
        self.assertGreater(swapped.contact_face_count, 0)
        self.assertAlmostEqual(
            swapped.total_normal_force_n,
            canonical.total_normal_force_n,
            delta=0.1 * canonical.total_normal_force_n,
        )

    def test_surface_survives_solver_contact_reduction(self) -> None:
        # M1 linchpin: reduce_contacts=True must shrink what the SOLVER sees
        # while (with pre_prune_contacts=False) the sensor's contact surface
        # stays dense and integrates to the same force.
        reduced_scene = build_demo_scene(self.config, reduce_contacts=True)
        dense = self.sample(0.0008)
        reduced = self.sample_scene(reduced_scene, 0.0008)

        self.assertGreater(dense.contact_face_count, 0)
        self.assertAlmostEqual(
            reduced.contact_face_count,
            dense.contact_face_count,
            delta=max(2, 0.05 * dense.contact_face_count),
        )
        self.assertAlmostEqual(
            reduced.total_normal_force_n,
            dense.total_normal_force_n,
            delta=0.05 * dense.total_normal_force_n,
        )

        dense_solver_contacts = int(self.scene.contacts.rigid_contact_count.numpy()[0])
        reduced_solver_contacts = int(reduced_scene.contacts.rigid_contact_count.numpy()[0])
        self.assertGreater(dense_solver_contacts, 0)
        self.assertGreater(reduced_solver_contacts, 0)
        self.assertLess(reduced_solver_contacts * 10, dense_solver_contacts)

    def test_area_splat_matches_between_cpu_and_gpu(self) -> None:
        # AREA SPLAT (2026-08-25): the sub-triangle raster must agree between
        # the NumPy reference and the warp kernel and still integrate to the
        # face-force total.
        import os
        sensor = self.scene.sensor
        os.environ["GS_SENSOR_GPU"] = "1"
        sensor._use_gpu = True
        gpu = self.sample(0.0008)
        sensor._use_gpu = False
        cpu = self.sample(0.0008)
        sensor._use_gpu = True
        self.assertGreater(gpu.contact_face_count, 0)
        np.testing.assert_allclose(gpu.pressure_pa, cpu.pressure_pa,
                                   rtol=1.0e-3, atol=1.0)
        self.assertAlmostEqual(gpu.total_normal_force_n, cpu.total_normal_force_n,
                               delta=1.0e-4 * cpu.total_normal_force_n)


class TestSubsampleFaces(unittest.TestCase):
    def test_subtriangles_conserve_force_and_stay_inside(self) -> None:
        from hydrogel.sensor_kernels import subsample_faces
        rng = np.random.default_rng(0)
        v = rng.normal(size=(50, 3, 3)) * 0.0005
        f = rng.uniform(0.1, 1.0, size=50)
        pts, frc = subsample_faces(v, f, 0.000145, 0.000149, max_level=6)
        self.assertAlmostEqual(float(frc.sum()), float(f.sum()), places=12)
        # weighted centroid of all samples == force-weighted centroid of faces
        c_faces = (v.mean(axis=1) * f[:, None]).sum(axis=0) / f.sum()
        c_pts = (pts * frc[:, None]).sum(axis=0) / frc.sum()
        np.testing.assert_allclose(c_pts, c_faces, atol=1.0e-9)
        self.assertGreater(len(pts), 50)          # 0.5 mm faces at 0.145 mm cells subdivide

    def test_small_faces_keep_centroid_splat(self) -> None:
        from hydrogel.sensor_kernels import subsample_faces
        v = np.array([[[0, 0, 0], [1e-5, 0, 0], [0, 1e-5, 0]]], dtype=float)
        pts, frc = subsample_faces(v, np.array([2.0]), 0.000145, 0.000149)
        self.assertEqual(len(pts), 1)
        np.testing.assert_allclose(pts[0], v[0].mean(axis=0))
        self.assertEqual(frc[0], 2.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
