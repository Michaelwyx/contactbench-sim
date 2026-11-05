"""Dense GelSight-like observations from Newton hydroelastic contact surfaces.

Canonical copy for the sim/ suite (newton 1.5 / warp 1.16, env
``tactile-newton``).  Ported from ``integrations/isaaclab/hydroelastic_gelsight.py``
(newton 1.2.1 era); that copy stays frozen until the M4 backport.  Run from
``sim/``:

    python -m hydrogel.sensor

The sensor is intentionally Newton-native.  Both the gel and every object that
can touch it must use ``ShapeConfig(is_hydroelastic=True)`` and the collision
pipeline must export its hydroelastic contact surface.  The adapter rasterizes
that surface into pressure, normal-displacement, and stateful shear maps.

The output is a mechanics observation, not a photorealistic GelSight camera
render.  A renderer can consume the displacement maps without changing the
contact model used by the policy or reward.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

import os

import numpy as np
import warp as wp

import newton
from newton.geometry import HydroelasticSDF

from hydrogel.paths import GELSIGHT_OUTPUT_DIR

OUTPUT_DIR = GELSIGHT_OUTPUT_DIR


@dataclass(frozen=True)
class GelSightConfig:
    """Mechanical and raster configuration for one GelSight Mini-sized pad."""

    width_m: float = 0.0186
    height_m: float = 0.0143
    gel_thickness_m: float = 0.004
    rows: int = 32
    cols: int = 42
    gel_kh_pa_per_m: float = 2.0e8
    rigid_kh_pa_per_m: float = 1.0e10
    # G/t of a real GS Mini gel: E ~ 0.1-0.2 MPa (UniVTAC uses 0.1) ->
    # G ~ 0.03-0.07 MPa over t = 4 mm ~ 1.5e7 Pa/m.  The old 8e7 was ~10x
    # stiff, so the Coulomb stick bound mu*p/k allowed only ~0.4 mm of
    # stored surface shear and a pressed-and-rotated object read as
    # almost pure slip (2026-08-24).
    # 2026-08-24: 1.5e7 -> 3.0e7 (G 60 -> 120 kPa on the 4 mm layer) and the
    # cap 4 -> 2 mm (shear strain ~0.5): task videos (nut regrasp strokes)
    # showed marker sweeps pinned at the old 4 mm cap -- visually far more
    # surface travel than a real pad.  Stiffer shear halves the stored
    # displacement at the same traction; the Coulomb bound u = mu*p/k_s
    # stays exact (validated by the shear experiments' saturation value).
    shear_stiffness_pa_per_m: float = 3.0e7
    friction_coefficient: float = 0.8
    shear_release_time_s: float = 0.025
    min_pressure_pa: float = 100.0
    sdf_max_resolution: int = 96
    sdf_narrow_band_m: float = 0.0025
    contact_gap_m: float = 0.0003
    # stiffening coefficient of the gel pressure law (gelsight_pressure.py):
    # p = kh*x*(1 + a*(x/t)^2); 0 = Newton's linear law, byte-identical
    gel_stiffening: float = 0.0
    # hard cap on the tangential surface displacement the gel can store
    # [m]: the Coulomb limit mu*p/k_shear is the STICK bound, but at
    # saturated pressures (bottomed pad, 1.76 MPa) it evaluates to 17 mm
    # -- no gel surface shears by 4x its thickness before slipping.
    shear_cap_m: float = 2.0e-3   # half the gel thickness (shear strain ~0.5)

    def __post_init__(self) -> None:
        if self.width_m <= 0.0 or self.height_m <= 0.0 or self.gel_thickness_m <= 0.0:
            raise ValueError("GelSight dimensions must be positive")
        if self.rows < 2 or self.cols < 2:
            raise ValueError("GelSight raster must be at least 2x2")
        if self.gel_kh_pa_per_m <= 0.0 or self.rigid_kh_pa_per_m <= 0.0:
            raise ValueError("Hydroelastic moduli must be positive")
        if self.shear_stiffness_pa_per_m <= 0.0:
            raise ValueError("Shear stiffness must be positive")


@dataclass(frozen=True)
class TactileFrame:
    """One sensor sample, with SI units on every mechanics channel."""

    pressure_pa: np.ndarray
    normal_displacement_m: np.ndarray
    shear_displacement_m: np.ndarray
    shear_traction_pa: np.ndarray
    contact_mask: np.ndarray
    total_normal_force_n: float
    center_of_pressure_m: np.ndarray
    contact_face_count: int
    # the raw pressure-LAW integral (kh * depth * area) before any
    # solver-consistency scaling; equals total_normal_force_n when no
    # solver force was supplied (diagnostic: law vs solver compliance)
    pressure_law_force_n: float = 0.0
    solver_scale: float = 1.0

    def mechanics_observation(self) -> np.ndarray:
        """Return ``[normal, shear_x, shear_y]`` with shape ``(3, H, W)``."""

        return np.concatenate((self.normal_displacement_m[None], self.shear_displacement_m), axis=0)


def make_gel_shape_config(config: GelSightConfig) -> newton.ModelBuilder.ShapeConfig:
    """Return the required Newton hydroelastic material for a compliant gel."""

    return newton.ModelBuilder.ShapeConfig(
        density=1050.0,
        mu=config.friction_coefficient,
        gap=config.contact_gap_m,
        is_hydroelastic=True,
        kh=config.gel_kh_pa_per_m,
        sdf_max_resolution=config.sdf_max_resolution,
        sdf_narrow_band_range=(-config.sdf_narrow_band_m, config.sdf_narrow_band_m),
    )


def make_object_shape_config(config: GelSightConfig) -> newton.ModelBuilder.ShapeConfig:
    """Return the hydroelastic material required on objects seen by the gel."""

    return newton.ModelBuilder.ShapeConfig(
        density=1000.0,
        mu=config.friction_coefficient,
        gap=config.contact_gap_m,
        is_hydroelastic=True,
        kh=config.rigid_kh_pa_per_m,
        sdf_max_resolution=config.sdf_max_resolution,
        sdf_narrow_band_range=(-config.sdf_narrow_band_m, config.sdf_narrow_band_m),
    )


def attach_gelsight_pad(
    builder: newton.ModelBuilder,
    finger_body: int,
    config: GelSightConfig,
    *,
    xform: wp.transform | None = None,
    label: str = "gelsight_gel",
    physics_thickness_m: float | None = None,
    as_mesh: bool = False,
    sdf_resolution: int | None = None,   # default GS_GEL_SDF_RES or 192
    pad_extents_m: tuple[float, float] | None = None,
    color: tuple[float, float, float] | None = None,
) -> int:
    """Attach one volumetric gel collider to a fingertip body.

    The box's local +z face is the sensing surface.  ``xform`` therefore
    controls both pad placement and tactile image orientation on the gripper.

    ``physics_thickness_m``: Newton's box pressure field is a TENT peaking at
    the slab's medial axis, so with a box exactly ``gel_thickness`` thick any
    indentation deeper than half the gel reads LESS pressure at its deepest
    point (measured: a sphere at 25 N/pad, 3 mm deep into a 4 mm gel, gave a
    ring-shaped pressure patch with a dark center).  A real rigid-backed gel
    pushes back hardest there.  Passing ``2 * gel_thickness`` makes the
    pressure field monotonic through the whole sensing depth; the caller
    must then shift the box center so the sensing face stays put (the back
    half sits inside the finger).  Default keeps the old one-thickness box.
    """

    t_phys = physics_thickness_m or config.gel_thickness_m
    # the PHYSICAL slab can be larger than the imaged FOV (a real GS Mini
    # gel is 25.25 x 20.75 mm; the camera reads the central 18.6 x 14.3):
    # contact outside the FOV still carries load on the solver side, and
    # the raster (config.width/height_m) keeps reading only the FOV
    pad_w, pad_h = pad_extents_m or (config.width_m, config.height_m)
    if sdf_resolution is None:
        # measured 2026-08-24: raising this does NOT reduce the flat-press
        # plaid moire (128 -> 16.7 %, 256 -> 20.2 % ripple) -- the pattern
        # comes from the pair's iso-surface tessellation, not gel voxels
        sdf_resolution = int(os.environ.get("GS_GEL_SDF_RES", "128"))
    if as_mesh:
        # THE GEL MUST BE SHAPE B OF EVERY HYDROELASTIC PAIR (2026-08-23).
        # Newton exports one depth per contact-surface face -- the SDF depth
        # of the pair's SECOND shape -- and pairs are ordered by shape TYPE
        # (box 7 before mesh 8), then index.  A box gel against a mesh object
        # is shape A, so the solver is handed the rigid object's depth (the
        # iso-surface sits p / 1e10 ~ micrometres inside it) and, seeing no
        # penetration, acts as a wall: the grasp is rigid, the gel never
        # indents and the tactile image is empty however hard the pinch.
        # The 16 mm cube of task_gelsight_grip (a box with a LOWER index than
        # the gels) was the one case where the gel was shape B, which is
        # why that task alone behaved like a gel (0.8 mm at 7 N).  Built as
        # a MESH with a higher index than the object (mount AFTER adding
        # the objects), the gel is B for mesh objects by index and for box
        # /sphere objects by type, so the solver sees the real indentation.
        import trimesh
        from hydrogel.mesh import make_newton_mesh

        tm = trimesh.creation.box(extents=(pad_w, pad_h, t_phys))
        # the SDF must stay valid through the WHOLE slab (iso-surface depths
        # reach the gel thickness): narrow band > half the physics slab
        mesh, _c = make_newton_mesh(
            tm, sdf=True, sdf_resolution=sdf_resolution,
            narrow_band=0.5 * t_phys + 0.001, margin=0.0005)
        cfg = newton.ModelBuilder.ShapeConfig(
            density=1050.0, mu=config.friction_coefficient,
            gap=config.contact_gap_m, is_hydroelastic=True,
            kh=config.gel_kh_pa_per_m)
        return builder.add_shape_mesh(body=finger_body, xform=xform, mesh=mesh,
                                      cfg=cfg, color=color, label=label)
    return builder.add_shape_box(
        body=finger_body,
        xform=xform,
        hx=0.5 * pad_w,
        hy=0.5 * pad_h,
        hz=0.5 * t_phys,
        cfg=make_gel_shape_config(config),
        color=color,
        label=label,
    )


def make_hydroelastic_pipeline(
    model: newton.Model,
    *,
    rigid_contact_max: int = 20_000,
    reduce_contacts: bool = False,
    broad_phase: str = "explicit",
) -> newton.CollisionPipeline:
    """Create a collision pipeline that retains the dense contact surface.

    ``reduce_contacts`` only reduces what the SOLVER sees; with
    ``pre_prune_contacts=False`` every iso face is still exported to the
    contact-surface buffer before reduction runs, so the sensor keeps its
    dense raster either way (verified against newton 1.5
    ``sdf_hydroelastic.py:572, 1781-1798``).
    """

    hydro_config = HydroelasticSDF.Config(
        output_contact_surface=True,
        reduce_contacts=reduce_contacts,
        pre_prune_contacts=False,
        buffer_fraction=1.0,
        buffer_mult_iso=2,
        buffer_mult_contact=2,
        mc_edge_clamp_min=0.0,
    )
    return newton.CollisionPipeline(
        model,
        rigid_contact_max=rigid_contact_max,
        broad_phase=broad_phase,
        sdf_hydroelastic_config=hydro_config,
    )


def _quat_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm == 0.0:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _compose_pose(parent: np.ndarray, child: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    parent_rotation = _quat_to_matrix(parent[3:])
    child_rotation = _quat_to_matrix(child[3:])
    position = parent[:3] + parent_rotation @ child[:3]
    return position, parent_rotation @ child_rotation


class HydroelasticGelSight:
    """Rasterize one Newton hydroelastic gel shape into tactile observations.

    The local gel frame uses x across image columns, y across rows, and +z out
    through the sensing surface.  ``sample`` performs a GPU-to-CPU transfer;
    batched training should replace the NumPy rasterizer with an equivalent
    Warp kernel after the sensor contract is frozen.
    """

    def __init__(
        self,
        model: newton.Model,
        collision_pipeline: newton.CollisionPipeline,
        gel_shape_index: int,
        config: GelSightConfig | None = None,
    ) -> None:
        self.model = model
        self.pipeline = collision_pipeline
        self.gel_shape_index = int(gel_shape_index)
        self.config = config or GelSightConfig()
        if not 0 <= self.gel_shape_index < model.shape_count:
            raise ValueError(f"Invalid gel shape index {self.gel_shape_index}")
        if collision_pipeline.hydroelastic_sdf is None:
            raise ValueError("Collision pipeline does not have hydroelastic SDF contacts enabled")
        if not collision_pipeline.hydroelastic_sdf.config.output_contact_surface:
            raise ValueError("Hydroelastic pipeline must set output_contact_surface=True")

        self._shape_bodies = model.shape_body.numpy().astype(np.int32)
        self._shape_local_poses = model.shape_transform.numpy().astype(np.float64)
        self._shape_kh = model.shape_material_kh.numpy().astype(np.float64)
        self._shear_displacement = np.zeros((2, self.config.rows, self.config.cols), dtype=np.float32)
        # GPU rasterizer (gelsight_sensor_kernels): built lazily; GS_SENSOR_GPU=0
        # keeps the NumPy reference path
        self._gpu = None
        self._use_gpu = os.environ.get("GS_SENSOR_GPU", "1") != "0"

    def reset(self) -> None:
        self._shear_displacement.fill(0.0)

    def _gel_world_pose(self, state: newton.State) -> tuple[np.ndarray, np.ndarray]:
        local_pose = self._shape_local_poses[self.gel_shape_index]
        body = self._shape_bodies[self.gel_shape_index]
        if body < 0:
            return local_pose[:3], _quat_to_matrix(local_pose[3:])
        return _compose_pose(state.body_q.numpy()[body].astype(np.float64), local_pose)

    @staticmethod
    def _point_velocity(
        body_poses: np.ndarray,
        body_velocities: np.ndarray | None,
        body: int,
        point_w: np.ndarray,
    ) -> np.ndarray:
        if body < 0 or body_velocities is None:
            return np.zeros(3, dtype=np.float64)
        body_pose = body_poses[body]
        body_velocity = body_velocities[body]
        linear_velocity = body_velocity[:3]
        angular_velocity = body_velocity[3:]
        return linear_velocity + np.cross(angular_velocity, point_w - body_pose[:3])

    def _infer_tangential_velocity(
        self,
        state: newton.State,
        pairs: np.ndarray,
        centroids_w: np.ndarray,
        force_weights: np.ndarray,
        gel_rotation_w: np.ndarray,
    ) -> np.ndarray:
        gel_body = int(self._shape_bodies[self.gel_shape_index])
        relative_velocity = np.zeros(3, dtype=np.float64)
        weight_sum = float(force_weights.sum())
        if weight_sum <= 0.0:
            return relative_velocity[:2]

        body_poses = state.body_q.numpy().astype(np.float64)
        if state.body_qd is None:
            return relative_velocity[:2]
        body_velocities = state.body_qd.numpy().astype(np.float64)
        # vectorized over faces (a Python loop here cost ~160 ms at 20k
        # faces -- it dominated the whole sensor sample)
        other_shape = np.where(pairs[:, 0] == self.gel_shape_index, pairs[:, 1], pairs[:, 0])
        other_body = self._shape_bodies[other_shape]

        def point_velocities(bodies: np.ndarray) -> np.ndarray:
            v = np.zeros((len(bodies), 3), dtype=np.float64)
            ok = bodies >= 0
            if np.any(ok):
                b = bodies[ok]
                lin = body_velocities[b, :3]
                ang = body_velocities[b, 3:]
                v[ok] = lin + np.cross(ang, centroids_w[ok] - body_poses[b, :3])
            return v

        obj_v = point_velocities(other_body)
        gel_v = point_velocities(np.full(len(pairs), gel_body, dtype=np.int64))
        relative_velocity = (force_weights[:, None] * (obj_v - gel_v)).sum(axis=0)
        return (gel_rotation_w.T @ (relative_velocity / weight_sum))[:2]

    def _rasterize_force(self, points_local: np.ndarray, face_forces: np.ndarray) -> np.ndarray:
        cfg = self.config
        u = (points_local[:, 0] / cfg.width_m + 0.5) * cfg.cols - 0.5
        v = (points_local[:, 1] / cfg.height_m + 0.5) * cfg.rows - 0.5
        # centroids outside the imaged FOV stay off-image (the physical pad
        # is larger than the FOV); their force still counts in the totals
        in_fov = ((u >= -0.5) & (u <= cfg.cols - 0.5)
                  & (v >= -0.5) & (v <= cfg.rows - 0.5))
        u0 = np.floor(u).astype(np.int32)
        v0 = np.floor(v).astype(np.int32)
        du = u - u0
        dv = v - v0

        candidates = (
            (v0, u0, (1.0 - dv) * (1.0 - du)),
            (v0, u0 + 1, (1.0 - dv) * du),
            (v0 + 1, u0, dv * (1.0 - du)),
            (v0 + 1, u0 + 1, dv * du),
        )
        valid_candidates = []
        weight_sum = np.zeros(len(face_forces), dtype=np.float64)
        for rows, cols, weights in candidates:
            valid = ((rows >= 0) & (rows < cfg.rows) & (cols >= 0)
                     & (cols < cfg.cols) & in_fov)
            valid_candidates.append((rows, cols, weights, valid))
            weight_sum[valid] += weights[valid]

        force_grid = np.zeros((cfg.rows, cfg.cols), dtype=np.float64)
        for rows, cols, weights, valid in valid_candidates:
            valid &= weight_sum > 0.0
            normalized = weights[valid] / weight_sum[valid]
            np.add.at(force_grid, (rows[valid], cols[valid]), face_forces[valid] * normalized)
        return force_grid

    def _finish_frame(self, dt, pressure, face_total, cop_local, velocity_xy,
                      law_total, scale, n_faces):
        """Shared tail of the CPU and GPU paths: displacement, shear, frame."""

        contact_mask = pressure >= self.config.min_pressure_pa
        if self.config.gel_stiffening > 0.0:
            from hydrogel.pressure import indent_from_pressure_np
            normal_displacement = indent_from_pressure_np(
                pressure, self.config.gel_kh_pa_per_m, self.config.gel_thickness_m,
                self.config.gel_stiffening).astype(np.float32)
        else:
            normal_displacement = (pressure / self.config.gel_kh_pa_per_m).astype(np.float32)
        if dt > 0.0:
            release = np.exp(-dt / self.config.shear_release_time_s)
            self._shear_displacement[:, ~contact_mask] *= release
            vel = np.asarray(velocity_xy, dtype=np.float64)
            if vel.ndim == 3:            # per-cell (2, rows, cols) field
                self._shear_displacement[:, contact_mask] += (vel * dt)[:, contact_mask]
            else:                        # uniform (2,) velocity (legacy/demo)
                self._shear_displacement[:, contact_mask] += (vel * dt)[:, None]
        displacement_limit = np.minimum(
            self.config.friction_coefficient * pressure / self.config.shear_stiffness_pa_per_m,
            self.config.shear_cap_m,
        )
        shear_norm = np.linalg.norm(self._shear_displacement, axis=0)
        scale_s = np.minimum(1.0, displacement_limit / np.maximum(shear_norm, 1.0e-12))
        self._shear_displacement *= scale_s[None]
        shear_traction = self._shear_displacement * self.config.shear_stiffness_pa_per_m
        return TactileFrame(
            pressure_pa=pressure.astype(np.float32),
            normal_displacement_m=normal_displacement,
            shear_displacement_m=self._shear_displacement.copy(),
            shear_traction_pa=shear_traction.astype(np.float32),
            contact_mask=contact_mask,
            total_normal_force_n=float(face_total),
            center_of_pressure_m=np.asarray(cop_local, dtype=np.float32),
            contact_face_count=int(n_faces),
            pressure_law_force_n=float(law_total),
            solver_scale=float(scale),
        )

    def _sample_gpu(self, state, dt, surface, tangential_velocity_local_mps,
                    solver_force_n):
        from hydrogel.sensor_kernels import GpuRasterizer

        cfg = self.config
        if self._gpu is None:
            self._gpu = GpuRasterizer(self.model, cfg.rows, cfg.cols,
                                      self.gel_shape_index, cfg.gel_thickness_m,
                                      cfg.gel_stiffening)
        gel_position_w, gel_rotation_w = self._gel_world_pose(state)
        grid, vgrid, sums = self._gpu.run(surface, state, gel_position_w,
                                          gel_rotation_w, cfg.width_m, cfg.height_m)
        law_total = float(sums[0])
        n_faces = int(sums[6])
        if n_faces == 0 or law_total <= 0.0:
            return self._empty_frame(dt)
        scale = 1.0
        if solver_force_n == 0.0:
            scale = 0.0
        elif solver_force_n is not None and law_total > 1.0e-9:
            scale = float(solver_force_n) / law_total
        cell_area = cfg.width_m * cfg.height_m / (cfg.rows * cfg.cols)
        pressure = (grid * scale / cell_area).astype(np.float32)
        if scale > 1.0:
            pressure = self._spread_to_saturation(pressure, cell_area)
        cop = sums[1:3] / law_total if scale > 0.0 else np.full(2, np.nan)
        if tangential_velocity_local_mps is None:
            # PER-CELL shear velocity: f-weighted splat / force grid (zero
            # where no force), so rotation about the normal and non-uniform
            # slip reach the shear state and the markers
            with np.errstate(invalid="ignore", divide="ignore"):
                vcell = np.where(grid[None] > 1e-12, vgrid / np.maximum(grid[None], 1e-12), 0.0)
            return self._finish_frame(dt, pressure, law_total * scale, cop, vcell,
                                      law_total, scale, n_faces)
        velocity_xy = np.asarray(tangential_velocity_local_mps, dtype=np.float64)
        return self._finish_frame(dt, pressure, law_total * scale, cop, velocity_xy,
                                  law_total, scale, n_faces)

    def saturation_pressure_pa(self) -> float:
        """Pressure of the gel at full compression under its law."""

        c = self.config
        if c.gel_stiffening > 0.0:
            from hydrogel.pressure import pressure_np
            return float(pressure_np(np.array(c.gel_thickness_m), c.gel_kh_pa_per_m,
                                     c.gel_thickness_m, c.gel_stiffening))
        return float(c.gel_kh_pa_per_m * c.gel_thickness_m)

    def _spread_to_saturation(self, pressure: np.ndarray, cell_area: float,
                              max_iter: int = 64) -> np.ndarray:
        from scipy.ndimage import binary_dilation

        p_sat = self.saturation_pressure_pa()
        p = pressure.astype(np.float64)
        for _ in range(max_iter):
            over = p > p_sat
            if not over.any():
                break
            excess = float(((p[over] - p_sat) * cell_area).sum())
            p[over] = p_sat
            fp = p > 0.0
            ring = binary_dilation(fp) & ~fp
            if not ring.any():
                break                      # the whole pad is saturated
            p[ring] += excess / (float(ring.sum()) * cell_area)
        return p.astype(np.float32)

    def _empty_frame(self, dt: float) -> TactileFrame:
        cfg = self.config
        if dt > 0.0:
            self._shear_displacement *= np.exp(-dt / cfg.shear_release_time_s)
        zeros = np.zeros((cfg.rows, cfg.cols), dtype=np.float32)
        shear = self._shear_displacement.copy()
        return TactileFrame(
            pressure_pa=zeros,
            normal_displacement_m=zeros.copy(),
            shear_displacement_m=shear,
            # The decaying shear state is gel MEMORY, not load: with no
            # contact there is no surface traction (the in-contact path
            # likewise clamps traction to mu * pressure = 0 where pressure
            # vanishes).  Reporting the relaxing displacement but zero
            # traction keeps the Coulomb bound exact through release.
            shear_traction_pa=np.zeros((2, cfg.rows, cfg.cols), dtype=np.float32),
            contact_mask=np.zeros_like(zeros, dtype=bool),
            total_normal_force_n=0.0,
            center_of_pressure_m=np.array([np.nan, np.nan], dtype=np.float32),
            contact_face_count=0,
        )

    def sample(
        self,
        state: newton.State,
        dt: float,
        *,
        tangential_velocity_local_mps: np.ndarray | tuple[float, float] | None = None,
        surface=None,
        solver_force_n: float | None = None,
    ) -> TactileFrame:
        """Sample the latest collision result.

        ``solver_force_n``: the SOLVER's realized normal force on this gel
        (sum of its hydroelastic contact forces after
        ``solver.update_contacts``).  When given, the pressure raster keeps
        the pressure field's spatial shape but is scaled so its integral
        equals that force -- SOLVER-CONSISTENT tactile output.  Measured
        2026-08-22 (packages/force-bench/src/force_bench/tactile/press_probe.py --solver): SolverMuJoCo
        realizes a hydroelastic contact compliance that does not follow
        the pressure law and depends on the bodies involved (9x stiffer
        than the law for a driven 16 mm box, ~1x for the 15 g chopstick,
        ~20-50x for the door key bow / the hex nut), so the raw law
        integral can be off by that factor from what the robot feels.
        The raw integral is kept in ``pressure_law_force_n``.

        Call ``pipeline.collide(state, contacts)`` before this method.  An
        explicit local tangential velocity is useful for kinematic objects;
        otherwise rigid-body velocities are inferred from ``state.body_qd``.
        A caller sharing one collide across several sensors can pass the
        already-fetched ``surface`` (``ContactSurfaceData``) to avoid
        re-fetching it per sensor.
        """

        if dt < 0.0:
            raise ValueError("dt must be non-negative")
        if solver_force_n is not None:
            solver_force_n = float(solver_force_n)
            if not np.isfinite(solver_force_n) or solver_force_n < 0.0:
                raise ValueError("solver_force_n must be finite and non-negative, or None")
        if surface is None:
            surface = self.pipeline.hydroelastic_sdf.get_contact_surface()
        if self._use_gpu:
            return self._sample_gpu(state, dt, surface, tangential_velocity_local_mps,
                                    solver_force_n)
        face_count = int(surface.face_contact_count.numpy()[0])
        if face_count == 0:
            return self._empty_frame(dt)

        pairs = surface.contact_surface_shape_pair.numpy()[:face_count].astype(np.int32)
        depths = surface.contact_surface_depth.numpy()[:face_count].astype(np.float64)
        vertices = (
            surface.contact_surface_point.numpy()[: face_count * 3]
            .astype(np.float64)
            .reshape(face_count, 3, 3)
        )
        selected = ((pairs[:, 0] == self.gel_shape_index) | (pairs[:, 1] == self.gel_shape_index)) & (
            depths < 0.0
        )
        if not np.any(selected):
            return self._empty_frame(dt)

        pairs = pairs[selected]
        depths = depths[selected]
        vertices = vertices[selected]
        edges_a = vertices[:, 1] - vertices[:, 0]
        edges_b = vertices[:, 2] - vertices[:, 0]
        areas = 0.5 * np.linalg.norm(np.cross(edges_a, edges_b), axis=1)
        valid = np.isfinite(areas) & (areas > 0.0)
        if not np.any(valid):
            return self._empty_frame(dt)
        pairs = pairs[valid]
        depths = depths[valid]
        vertices = vertices[valid]
        areas = areas[valid]

        # Newton 1.5 convention (verified in sdf_hydroelastic.py:147-157,
        # 1754, 1795): iso_vertex_depth is the SDF-B signed depth at the face
        # centroid, iso_vertex_shape_pair = (shape_a, shape_b), and the
        # default linear_pressure is p = -kh[shape_b] * signed_depth.  On the
        # equal-pressure isosurface this equals k_eff * total deformation for
        # the two materials in series, so kh[pairs[:, 1]] * (-depth) is the
        # exported face pressure regardless of which side is the gel.
        pressure_faces = self._shape_kh[pairs[:, 1]] * (-depths)
        if self.config.gel_stiffening > 0.0:
            # gel on the B side: the exported depth is the GEL's and the
            # pressure follows the stiffening law (gelsight_pressure.py)
            from hydrogel.pressure import pressure_np
            gb = pairs[:, 1] == self.gel_shape_index
            if gb.any():
                pressure_faces[gb] = pressure_np(
                    -depths[gb], self._shape_kh[self.gel_shape_index],
                    self.config.gel_thickness_m, self.config.gel_stiffening)
        centroids_w = vertices.mean(axis=1)

        gel_position_w, gel_rotation_w = self._gel_world_pose(state)
        centroids_local = (centroids_w - gel_position_w) @ gel_rotation_w
        # The raster is the NORMAL traction on the sensing plane, so each
        # iso-surface face contributes p * A * |n . z_gel|: faces wrapping
        # an object's side walls (an edge dug into the gel, a pinched
        # shank) carry lateral load, not pressure on the pad.  Measured
        # before this projection: a sharp cube at 25 N/pad read 962 kPa and
        # a 24 mm "indentation" from side-wall faces splatted into one row.
        face_normals_w = np.cross(edges_a, edges_b)[valid]
        face_normals_w /= np.maximum(
            np.linalg.norm(face_normals_w, axis=1, keepdims=True), 1.0e-18)
        cos_n = np.abs(face_normals_w @ gel_rotation_w[:, 2])
        face_forces = pressure_faces * areas * cos_n
        keep = face_forces > 0.0
        if not np.any(keep):
            return self._empty_frame(dt)
        pairs, centroids_w, centroids_local, face_forces = (
            pairs[keep], centroids_w[keep], centroids_local[keep],
            face_forces[keep])
        law_total = float(face_forces.sum())
        scale = 1.0
        if solver_force_n == 0.0:
            scale = 0.0
            face_forces = face_forces * scale
        elif solver_force_n is not None and law_total > 1.0e-9:
            scale = float(solver_force_n) / law_total
            face_forces = face_forces * scale
        # AREA SPLAT (2026-08-25, see sensor_kernels.splat_faces): sub-divide
        # every face so the raster is the area integral of the face
        # pressure, not a lattice of centroids beating against the cells
        from hydrogel.sensor_kernels import subsample_faces
        vertices_local = (vertices[keep] - gel_position_w) @ gel_rotation_w
        sub_points, sub_forces = subsample_faces(
            vertices_local, face_forces,
            self.config.width_m / self.config.cols,
            self.config.height_m / self.config.rows)
        force_grid = self._rasterize_force(sub_points, sub_forces)
        cell_area = self.config.width_m * self.config.height_m / (self.config.rows * self.config.cols)
        pressure = (force_grid / cell_area).astype(np.float32)
        if scale > 1.0:
            # SATURATION SPREAD: the solver can carry far more through a
            # tiny geometric patch than the law allows (a bow edge at
            # 20 N on a handful of cells), and scaling alone would print
            # MPa cells.  A gel cannot exceed its full-compression
            # pressure p_sat = law(thickness); clamp there and push the
            # excess into the neighbouring ring, repeatedly, so the
            # imprint grows the way a bottoming gel's does while the
            # integral stays the solver's force.
            pressure = self._spread_to_saturation(pressure, cell_area)
        if self.config.gel_stiffening > 0.0:
            from hydrogel.pressure import indent_from_pressure_np
            normal_displacement = indent_from_pressure_np(
                pressure, self.config.gel_kh_pa_per_m,
                self.config.gel_thickness_m,
                self.config.gel_stiffening).astype(np.float32)
        else:
            normal_displacement = (pressure / self.config.gel_kh_pa_per_m).astype(np.float32)
        contact_mask = pressure >= self.config.min_pressure_pa

        if tangential_velocity_local_mps is None:
            velocity_xy = self._infer_tangential_velocity(
                state, pairs, centroids_w, face_forces, gel_rotation_w
            )
        else:
            velocity_xy = np.asarray(tangential_velocity_local_mps, dtype=np.float64)
            if velocity_xy.shape != (2,):
                raise ValueError("tangential_velocity_local_mps must have shape (2,)")

        if dt > 0.0:
            release = np.exp(-dt / self.config.shear_release_time_s)
            self._shear_displacement[:, ~contact_mask] *= release
            self._shear_displacement[:, contact_mask] += (velocity_xy * dt)[:, None]

        # HydroShear-inspired stick/slip history: store tangential displacement
        # while sticking, then clamp traction to the local Coulomb limit.
        displacement_limit = np.minimum(
            self.config.friction_coefficient * pressure / self.config.shear_stiffness_pa_per_m,
            self.config.shear_cap_m,
        )
        shear_norm = np.linalg.norm(self._shear_displacement, axis=0)
        scale_s = np.minimum(1.0, displacement_limit / np.maximum(shear_norm, 1.0e-12))
        self._shear_displacement *= scale_s[None]
        shear_traction = self._shear_displacement * self.config.shear_stiffness_pa_per_m

        total_force = float(face_forces.sum())
        cop = (np.sum(centroids_local[:, :2] * face_forces[:, None], axis=0) / total_force
               if total_force > 0.0 else np.full(2, np.nan))
        return TactileFrame(
            pressure_pa=pressure,
            normal_displacement_m=normal_displacement,
            shear_displacement_m=self._shear_displacement.copy(),
            shear_traction_pa=shear_traction.astype(np.float32),
            contact_mask=contact_mask,
            total_normal_force_n=total_force,
            center_of_pressure_m=cop.astype(np.float32),
            contact_face_count=len(face_forces),
            pressure_law_force_n=law_total,
            solver_scale=scale,
        )


@dataclass
class DemoScene:
    model: newton.Model
    state: newton.State
    pipeline: newton.CollisionPipeline
    contacts: newton.Contacts
    sensor: HydroelasticGelSight
    indenter_body: int
    indenter_radius_m: float


def build_demo_scene(
    config: GelSightConfig,
    device: str = "cuda:0",
    *,
    gel_first: bool = True,
    reduce_contacts: bool = False,
    broad_phase: str = "explicit",
) -> DemoScene:
    """Build a deterministic kinematic sphere pressing a static gel volume.

    ``gel_first=False`` adds the indenter shape before the gel so the gel can
    land on either side of the exported shape pairs (pair-order regression).
    """

    builder = newton.ModelBuilder()
    radius = 0.006

    def add_indenter() -> int:
        body = builder.add_body(
            xform=wp.transform(wp.vec3(0.0, 0.0, 0.02), wp.quat_identity()),
            is_kinematic=True,
            label="kinematic_indenter",
        )
        builder.add_shape_sphere(
            body=body,
            radius=radius,
            cfg=make_object_shape_config(config),
            label="hydroelastic_indenter",
        )
        return body

    if gel_first:
        gel_shape = attach_gelsight_pad(builder, -1, config)
        body = add_indenter()
    else:
        body = add_indenter()
        gel_shape = attach_gelsight_pad(builder, -1, config)

    model = builder.finalize(device=device)
    state = model.state()
    newton.eval_fk(model, model.joint_q, model.joint_qd, state)
    pipeline = make_hydroelastic_pipeline(
        model, reduce_contacts=reduce_contacts, broad_phase=broad_phase
    )
    contacts = pipeline.contacts()
    sensor = HydroelasticGelSight(model, pipeline, gel_shape, config)
    return DemoScene(model, state, pipeline, contacts, sensor, body, radius)


def set_demo_indenter_pose(
    scene: DemoScene,
    *,
    x_m: float = 0.0,
    y_m: float = 0.0,
    indentation_m: float = 0.0008,
) -> None:
    gel_top = 0.5 * scene.sensor.config.gel_thickness_m
    center_z = gel_top + scene.indenter_radius_m - indentation_m
    body_q = scene.state.body_q.numpy()
    body_q[scene.indenter_body] = (x_m, y_m, center_z, 0.0, 0.0, 0.0, 1.0)
    scene.state.body_q.assign(body_q)


def _save_frame(frame: TactileFrame, output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output,
        pressure_pa=frame.pressure_pa,
        normal_displacement_m=frame.normal_displacement_m,
        shear_displacement_m=frame.shear_displacement_m,
        shear_traction_pa=frame.shear_traction_pa,
        contact_mask=frame.contact_mask,
        total_normal_force_n=np.float32(frame.total_normal_force_n),
        center_of_pressure_m=frame.center_of_pressure_m,
    )

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    figure, axes = plt.subplots(1, 3, figsize=(10, 3), constrained_layout=True)
    channels = (
        (frame.pressure_pa * 1.0e-3, "pressure [kPa]", "inferno"),
        (frame.normal_displacement_m * 1.0e3, "normal displacement [mm]", "viridis"),
        (np.linalg.norm(frame.shear_displacement_m, axis=0) * 1.0e3, "shear displacement [mm]", "magma"),
    )
    for axis, (image, title, cmap) in zip(axes, channels, strict=True):
        plot = axis.imshow(image, origin="lower", cmap=cmap)
        axis.set_title(title)
        axis.set_axis_off()
        figure.colorbar(plot, ax=axis, fraction=0.046)
    figure.savefig(output.with_suffix(".png"), dpi=160)
    plt.close(figure)


def _save_video(
    frames: list[TactileFrame],
    config: GelSightConfig,
    output: Path,
    fps: int,
) -> None:
    """Render solver-derived tactile channels with fixed sequence-wide scales."""

    if not frames:
        raise ValueError("at least one tactile frame is required")

    import imageio.v2 as iio
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.parent.mkdir(parents=True, exist_ok=True)
    pressure_scale = max(float(frame.pressure_pa.max()) for frame in frames) * 1.0e-3
    normal_scale = max(float(frame.normal_displacement_m.max()) for frame in frames) * 1.0e3
    shear_scale = max(
        float(np.linalg.norm(frame.shear_traction_pa, axis=0).max()) for frame in frames
    ) * 1.0e-3
    scales = [max(value, 1.0e-6) for value in (pressure_scale, normal_scale, shear_scale)]
    extent_mm = (
        -0.5 * config.width_m * 1.0e3,
        0.5 * config.width_m * 1.0e3,
        -0.5 * config.height_m * 1.0e3,
        0.5 * config.height_m * 1.0e3,
    )

    figure, axes = plt.subplots(1, 3, figsize=(12, 4.8), dpi=120, constrained_layout=True)
    figure.patch.set_facecolor("#101417")
    channels = (
        ("Hydroelastic pressure", "kPa", "inferno"),
        ("Normal gel deformation", "mm", "viridis"),
        ("Shear traction magnitude", "kPa", "magma"),
    )
    images = []
    for axis, (title, unit, cmap), scale in zip(axes, channels, scales, strict=True):
        axis.set_facecolor("#101417")
        image = axis.imshow(
            np.zeros((config.rows, config.cols)),
            origin="lower",
            extent=extent_mm,
            cmap=cmap,
            vmin=0.0,
            vmax=scale,
            interpolation="bilinear",
        )
        images.append(image)
        axis.set_title(title, color="#e6edf3", fontsize=12, pad=10)
        axis.set_xlabel("gel x [mm]", color="#a8b3bd")
        axis.set_ylabel("gel y [mm]", color="#a8b3bd")
        axis.tick_params(colors="#a8b3bd", labelsize=8)
        for spine in axis.spines.values():
            spine.set_color("#3a454d")
        colorbar = figure.colorbar(image, ax=axis, fraction=0.046, pad=0.04)
        colorbar.set_label(unit, color="#a8b3bd")
        colorbar.ax.tick_params(colors="#a8b3bd", labelsize=8)

    cop_marker, = axes[0].plot(
        [], [], marker="+", linestyle="none", color="#62e6ff", markersize=11, markeredgewidth=2
    )
    status = figure.suptitle("", color="#f0f4f7", fontsize=13, y=0.98)
    writer = iio.get_writer(output, fps=fps, quality=8, macro_block_size=None)
    try:
        for index, frame in enumerate(frames):
            images[0].set_data(frame.pressure_pa * 1.0e-3)
            images[1].set_data(frame.normal_displacement_m * 1.0e3)
            images[2].set_data(np.linalg.norm(frame.shear_traction_pa, axis=0) * 1.0e-3)
            if np.isfinite(frame.center_of_pressure_m).all():
                cop_marker.set_data(
                    [frame.center_of_pressure_m[0] * 1.0e3],
                    [frame.center_of_pressure_m[1] * 1.0e3],
                )
            else:
                cop_marker.set_data([], [])
            status.set_text(
                "Newton hydroelastic GelSight mechanics  |  "
                f"frame {index + 1:03d}/{len(frames):03d}  |  "
                f"normal force {frame.total_normal_force_n:5.2f} N  |  "
                f"contact faces {frame.contact_face_count}"
            )
            figure.canvas.draw()
            writer.append_data(np.asarray(figure.canvas.buffer_rgba())[:, :, :3])
    finally:
        writer.close()
        plt.close(figure)


def run_demo(args: argparse.Namespace) -> TactileFrame:
    config = GelSightConfig(rows=args.rows, cols=args.cols)
    scene = build_demo_scene(config, args.device)
    dt = 1.0 / 120.0
    x_values = np.linspace(-args.travel_mm, args.travel_mm, args.frames) * 1.0e-3
    previous_x = float(x_values[0])
    frame = scene.sensor._empty_frame(0.0)
    frames = []
    for x_m in x_values:
        set_demo_indenter_pose(
            scene,
            x_m=float(x_m),
            indentation_m=args.indentation_mm * 1.0e-3,
        )
        scene.pipeline.collide(scene.state, scene.contacts)
        velocity = ((float(x_m) - previous_x) / dt, 0.0)
        frame = scene.sensor.sample(scene.state, dt, tangential_velocity_local_mps=velocity)
        frames.append(frame)
        previous_x = float(x_m)

    output = Path(args.output)
    _save_frame(frame, output)
    if args.video:
        _save_video(frames, config, Path(args.video), args.fps)
    print(
        f"[hydro-gelsight] faces={frame.contact_face_count} "
        f"force={frame.total_normal_force_n:.3f} N "
        f"peak={frame.pressure_pa.max() * 1.0e-3:.1f} kPa "
        f"cop=({frame.center_of_pressure_m[0] * 1.0e3:.2f}, "
        f"{frame.center_of_pressure_m[1] * 1.0e3:.2f}) mm"
    )
    print(f"[hydro-gelsight] wrote {output} and optional PNG preview")
    if args.video:
        print(f"[hydro-gelsight] wrote {args.video}")
    return frame


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--frames", type=int, default=90)
    parser.add_argument("--rows", type=int, default=32)
    parser.add_argument("--cols", type=int, default=42)
    parser.add_argument("--indentation-mm", type=float, default=0.8)
    parser.add_argument("--travel-mm", type=float, default=2.0)
    parser.add_argument("--output", default=str(OUTPUT_DIR / "gelsight_sensor" / "demo_frame.npz"))
    parser.add_argument("--video", default=str(OUTPUT_DIR / "gelsight_sensor" / "rollout_panel.mp4"))
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    if args.frames < 1:
        parser.error("--frames must be positive")
    if args.fps < 1:
        parser.error("--fps must be positive")
    return args




# ── hydroelastic pipeline configuration ─────────────────────────────────
def winning_hydroelastic_config(model=None, gel_config: GelSightConfig | None = None,
                                gel_shapes=()) -> HydroelasticSDF.Config:
    """The measured 5/5 configuration from hydrogel.grip_rig.

    With a FINALIZED ``model`` and a ``gel_config`` whose ``gel_stiffening``
    is > 0, the gel shapes get the stiffening pressure law
    (hydrogel.pressure); otherwise Newton's linear law.
    """

    extra = {}
    if model is not None and gel_config is not None \
            and gel_config.gel_stiffening > 0.0:
        from hydrogel.pressure import make_pressure_data, stiffening_pressure
        extra = dict(
            pressure_func=stiffening_pressure,
            pressure_data=make_pressure_data(
                model, gel_shapes, gel_config.gel_thickness_m,
                gel_config.gel_stiffening))
    return HydroelasticSDF.Config(
        output_contact_surface=True,
        reduce_contacts=True,
        pre_prune_contacts=False,
        anchor_contact=True,
        buffer_fraction=1.0,
        buffer_mult_iso=2,
        buffer_mult_contact=2,
        mc_edge_clamp_min=0.0,
        **extra,
    )


def hydroelastic_object_cfg(gel_config: GelSightConfig, *, density: float,
                            mu: float | None = None,
                            kh: float | None = None,
                            mesh: bool = False):
    """ShapeConfig for a task object the gels must feel.

    ``mesh=True`` for mesh-backed shapes: newton refuses ``cfg.sdf_*`` on
    those -- the SDF must be built on the newton.Mesh itself
    (``make_newton_mesh(sdf=True, sdf_resolution=...)``), and the
    hydroelastic pipeline then uses that SDF.
    """

    if mesh:
        cfg = newton.ModelBuilder.ShapeConfig(
            density=density,
            mu=mu if mu is not None else gel_config.friction_coefficient,
            gap=gel_config.contact_gap_m,
            is_hydroelastic=True,
            kh=kh if kh is not None else gel_config.rigid_kh_pa_per_m,
        )
        return cfg
    cfg = make_gel_shape_config(gel_config)
    cfg.density = density
    cfg.kh = kh if kh is not None else gel_config.rigid_kh_pa_per_m
    if mu is not None:
        cfg.mu = mu
    return cfg


if __name__ == "__main__":
    run_demo(parse_args())
