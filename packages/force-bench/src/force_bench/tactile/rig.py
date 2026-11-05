"""GelSightRig: force_bench.rig.Rig + hydroelastic GelSight pads on the FR3 fingers.

The subclass rebuilds the collision pipeline with the hydroelastic contact
surface enabled and forks ``run()`` to sample the gel sensors right after
the once-per-frame ``collide()``.

Solver: the validated configuration is SolverMuJoCo consuming Newton
contacts (see packages/hydrogel/src/hydrogel/grip_rig.py and docs/gelsight_method.md).  A gel task should run
with::

    TB_MJ_IMPRATIO=10

which task_gelsight_grip.py sets as its own defaults.  ``_MJ_OPTS`` already
pins the elliptic cone, and Rig then picks 32 substeps.

Mounting: ``mount_gelsight_on_fr3`` must be called BEFORE ``finalize``.  It
attaches one gel box per finger in front of the URDF "rubber tip" collision
box, which stays live as the gel's rigid backing (its face is 3.5 mm behind
the sensing face, so it only engages when the gel bottoms out):
gel local +z (the sensing face) maps to finger -y (the grasp direction on
both fingers -- finger joint 2 carries the 180 deg z-rotation); gel x =
image columns = the LONG edge runs along the finger's length and gel y =
image rows across the finger (GS_PAD_LONG_AXIS, default ``finger``).  The pad face sits ``gel_thickness - embed`` proud of the old pad
face; ``grip_dof()`` converts object width + indent to a finger target.
"""

from __future__ import annotations

import os

import numpy as np
import warp as wp

import newton
from newton.geometry import HydroelasticSDF

from force_bench.rig import Rig
from hydrogel.sensor import (
    GelSightConfig,
    HydroelasticGelSight,
    attach_gelsight_pad,
    make_gel_shape_config,
)
from hydrogel.sensor import (  # re-exported for the tasks
    hydroelastic_object_cfg,
    winning_hydroelastic_config,
)

# Measured FR3 URDF fingertip facts: the grasping "rubber tip" box per
# finger (shape scale + center height in the finger frame).
RUBBER_TIP_SCALE = (0.00875, 0.0076, 0.00925)
RUBBER_TIP_Z = 0.04525
# pad-face gap between bare URDF fingertips is 2*dof - 0.17 mm
FINGER_FACE_OFFSET = -0.00017

# ── GelSight Mini case (GS_MOUNT=case, the default) ────────────────────
# The sensor is a real object on the finger, not a bare gel slab: a rigid
# housing that collides (what a case corner hits, the case hits), carries
# the sensor's mass, and backs the gel.  Constants are parametric; see
# docs/gelsight_method.md for their provenance.
# Official GS Mini datasheet + TacEx/UniVTAC assets (2026-08-24 research):
# aluminum body 32 x 28(t) x 24(d) mm with the ~4 mm gel-cartridge dome
# standing proud of its face; gel slab 25.25 x 20.75 x 4.25 mm, LARGER than
# the imaged FOV (18.6 x 14.3 -- the sensor reads only the centre); mass
# ~45 g; TacEx mounts case+gel+camera as separate prims, all colliding
# except gel<->case.
# ORIENTATION (2026-08-25): the real mount puts the sensor's LONG edge along
# the finger (toward the wrist) and its SHORT edge across the finger, so the
# 32 mm case edge and the gel's 18.6 mm image axis run along finger z.  The
# gel frame itself is unchanged (gel x = image columns = long edge); the
# whole sensor is turned 90 deg about the grasp normal.  GS_PAD_LONG_AXIS=
# across restores the earlier (wrong) mount for A/B against old runs.
PAD_LONG_AXIS = os.environ.get("GS_PAD_LONG_AXIS", "finger")
if PAD_LONG_AXIS not in ("finger", "across"):
    raise ValueError("GS_PAD_LONG_AXIS must be 'finger' or 'across'")
LONG_ALONG_FINGER = PAD_LONG_AXIS == "finger"
# case face extents along finger x (across the finger) / finger z (length)
CASE_FACE_W, CASE_FACE_H = ((0.028, 0.032) if LONG_ALONG_FINGER
                            else (0.032, 0.028))
CASE_DEPTH = 0.024         # aluminum body depth behind its face
CASE_PROUD = 0.001         # case FACE in front of the bare finger face [m]
GEL_PROUD_OF_CASE = 0.004  # the gel-cartridge dome proud of the case rim
CASE_MASS = 0.040          # kg case+adapter; the gel slab adds ~5 g
GEL_PAD_W = 0.02525        # physical gel slab extents (> the imaged FOV)
GEL_PAD_H = 0.02075
# the case must NOT reach past the fingertip (the bare tips end at finger
# z = 54.5 mm; a case centred on the pad would stick out 4.7 mm and hit
# the table before the fingers in any top-down grasp): its TOP edge is
# flush with the fingertip, the gel stays centred on the old pad height
FINGERTIP_Z_END = 0.0545
# GS_CASE_TIP_EXTEND [mm]: how far the case may stick out past the old
# fingertip.  0 (default) keeps every task's table clearance but places the
# FOV centre 14.5 mm behind the tip -- a grasp made with the last few mm of
# the finger (task_nut_bolt_arm's pad-above-bolt-head choice) then lands on
# the pad's UN-IMAGED edge: full force, no image, exactly what the hardware
# would do.  Extending images the tip zone at the cost of tip clearance.
CASE_TIP_EXTEND = float(os.environ.get("GS_CASE_TIP_EXTEND", "0")) * 1e-3
CASE_Z = FINGERTIP_Z_END + CASE_TIP_EXTEND - 0.5 * CASE_FACE_H


def pad_proud_m(mount: str | None = None) -> float:
    """How far the SENSING FACE sits in front of the bare fingertip face.
    The single source of truth for every finger-dof offset
    (gelsight_task.finger_offset, grip_dof)."""

    mount = mount or os.environ.get("GS_MOUNT", "case")
    if mount == "case":
        return CASE_PROUD + GEL_PROUD_OF_CASE
    return 0.0035              # bare slab: gel_thickness - embed (legacy)

FINGERS = ("fr3/fr3_leftfinger", "fr3/fr3_rightfinger")

# ── real GS Mini housing, visual only (GS_SHELL_VISUAL=0 disables) ─────
# Official GelSight Inc. CAD (assets/gelsight/gs_mini/gsmini_shell.obj,
# see PROVENANCE.md): extents 32.0 x 28.3 x 28.9 mm.  The CAD is authored
# with the ELASTOMER at its +y end (a 25.25 x 20.75 mm block, y = 19.0 ..
# 23.25 mm, domed top) and the mounting plate (two 1.6 mm pin holes, two
# 3.5 mm squares) at -y, so it is turned 180 deg about its own centre
# before mounting: the housing face then lands at finger y = -CASE_PROUD
# and the plate at the case back.  The CAD elastomer is dropped from the
# visual -- the physics gel slab has the same footprint and stands
# GEL_PROUD_OF_CASE proud in that window (2026-09-18; until then the
# shell went on with an identity rotation and the videos showed the gel
# dome on the OUTSIDE of the finger while the physics gel faced inward).
# Collision and mass STAY on the parametric case box (the shell envelopes
# it); this mesh only replaces the box in every render, so the videos
# show the sensor people actually recognise.
_SHELL_VISUAL = None


def _shell_visual_mesh():
    """(newton.Mesh, aabb_center) for the housing CAD, or None."""

    global _SHELL_VISUAL
    if os.environ.get("GS_SHELL_VISUAL", "1") == "0":
        return None
    if _SHELL_VISUAL is None:
        from hydrogel.paths import GS_MINI_DIR
        obj = GS_MINI_DIR / "gsmini_shell.obj"
        if not obj.exists():
            print(f"[gelsight] shell visual missing ({obj}); box look kept")
            _SHELL_VISUAL = False
        else:
            import trimesh
            from force_bench.assets import make_newton_mesh
            tm = trimesh.load(str(obj), force="mesh")
            # gel end (+y in the CAD) -> -y, about the CAD's own centre so
            # the extents stay put (housing face -> -CASE_PROUD)
            tm.apply_transform(trimesh.transformations.rotation_matrix(
                np.pi, (1.0, 0.0, 0.0), point=tm.bounds.mean(axis=0)))
            # drop the CAD elastomer (now y < -0.7 mm, centred at
            # x = -0.25, z = +0.5 mm): the physics gel renders there
            v = np.asarray(tm.vertices)
            in_gel = ((v[:, 1] < -0.0007)
                      & (np.abs(v[:, 0] + 0.00025) < 0.01275)
                      & (np.abs(v[:, 2] - 0.0005) < 0.01055))
            tm.update_faces(~in_gel[tm.faces].all(axis=1))
            tm.remove_unreferenced_vertices()
            if LONG_ALONG_FINGER:
                # CAD is authored with its 32 mm edge along x; turn it about
                # the grasp normal so that edge runs along the finger
                tm.apply_transform(trimesh.transformations.rotation_matrix(
                    np.pi / 2, (0.0, 1.0, 0.0)))
            _SHELL_VISUAL = make_newton_mesh(tm, sdf=False)
    return _SHELL_VISUAL or None




class GelSightRig(Rig):
    """Rig with gel pads whose TactileFrames ride the standard run loop."""

    def __init__(self, name: str, gel_config: GelSightConfig | None = None,
                 **kw) -> None:
        super().__init__(name, **kw)
        # Sensor requirements belong to this rig, not process-global import
        # side effects that can silently change a later bare-hand task.
        self.solver_options["impratio"] = float(os.environ.get("TB_MJ_IMPRATIO") or "10")
        self.collide_substep_override = True
        # GelSight studies live under outputs/gelsight/ (the task layer
        # re-points this to outputs/task_gelsight/<task>)
        from force_bench.paths import GELSIGHT_OUTPUT_DIR
        self.out_dir = GELSIGHT_OUTPUT_DIR / name
        self.gel_config = gel_config or GelSightConfig()
        self.gel_shapes: dict[str, int] = {}       # pad name -> shape index
        self.gel_sensors: dict[str, HydroelasticGelSight] = {}
        self.tactile_frames: dict[str, list] = {}
        # last frame's force dict (probes + gel sensors), for closed-loop
        # control callbacks (e.g. a grip force servo)
        self.last_forces: dict = {}
        self._collide_every = max(1, int(os.environ.get("GS_COLLIDE_EVERY", "1")))

    # ── mounting (before finalize) ──────────────────────────────────────
    def mount_gelsight_on_fr3(self, embed: float = 0.0005,
                              as_mesh: bool | None = None) -> dict[str, int]:
        """Replace both fingertip rubber-tip boxes with gel pads.

        ``as_mesh`` (default ``GS_GEL_MESH`` != "0"): build the gel as a mesh
        shape so it is shape B of every hydroelastic pair -- see
        gelsight_sensor.attach_gelsight_pad; MOUNT AFTER ADDING THE OBJECTS.
        """

        if as_mesh is None:
            as_mesh = os.environ.get("GS_GEL_MESH", "1") != "0"
        self.gel_as_mesh = as_mesh
        self.mount_style = os.environ.get("GS_MOUNT", "case")
        cfg = self.gel_config
        half_pi = float(np.pi / 2)
        # gel +z (sensing face) -> finger -y; gel x (image columns, the long
        # edge) -> finger x for the across mount, finger z for the along-
        # finger mount (pre-rotate 90 deg about the gel normal)
        q_gel = wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), half_pi)
        if LONG_ALONG_FINGER:
            q_gel = wp.mul(q_gel, wp.quat_from_axis_angle(
                wp.vec3(0.0, 0.0, 1.0), half_pi))
        for name in FINGERS:
            body = self.builder.body_label.index(name)
            tip = self._find_rubber_tip(body)
            if self.mount_style == "case":
                self._mount_case(body, name, cfg, q_gel, as_mesh)
                continue
            # The URDF rubber-tip box stays live as the gel's RIGID BACKING:
            # its face sits at finger y=0, 3.5 mm behind the gel's sensing
            # face, so it carries nothing at normal indentations but hard-
            # stops an object that bottoms the gel out (measured without it:
            # a nut at 25 N/pad sank 5.9 mm into a 4 mm gel).  The 2x-thick
            # physics slab (below) keeps the pressure field monotonic; the
            # backing box is what a real gel's glass/PCB substrate does.
            self._backing_shape = tip
            pad = "gel_left" if "left" in name else "gel_right"
            # physics slab = 2x the gel thickness so the hydroelastic
            # pressure field rises monotonically through the whole sensing
            # depth (see attach_gelsight_pad); the sensing face stays at
            # finger y = -(t - embed), so the box center moves to y = +embed
            t = cfg.gel_thickness_m
            xf = wp.transform(wp.vec3(0.0, embed, RUBBER_TIP_Z), q_gel)
            self.gel_shapes[pad] = attach_gelsight_pad(
                self.builder, body, cfg, xform=xf, label=pad,
                physics_thickness_m=2.0 * t, as_mesh=as_mesh)
            if as_mesh:
                for s in self.builder.body_shapes.get(body, []):
                    # a mesh gel is not filtered against its own finger's
                    # shapes by default the way the URDF boxes are; never
                    # let the slab's back half (inside the finger) collide
                    if s != self.gel_shapes[pad]:
                        self.builder.add_shape_collision_filter_pair(
                            s, self.gel_shapes[pad])
            self.shapes[pad] = [self.gel_shapes[pad]]
            self.tactile_frames[pad] = []
        self._gel_embed = embed
        # sensing-face height in the gel SHAPE frame (box half-thickness)
        self.gel_face_z = cfg.gel_thickness_m
        return dict(self.gel_shapes)

    def _mount_case(self, body, name, cfg, q_gel, as_mesh):
        """GS Mini as an ASSET: rigid housing (collides, has mass) + gel on
        its face.  The case face sits CASE_PROUD in front of the bare
        fingertip face (finger y = 0 at the rubber-tip face), the sensing
        face GEL_PROUD_OF_CASE beyond the case rim; the case is the gel's
        rigid backing (bottoming happens on the case, not the URDF tip)."""

        from force_bench.assets import set_body_mass

        b = self.builder
        pad = "gel_left" if "left" in name else "gel_right"
        t = cfg.gel_thickness_m
        # housing: centre CASE_DEPTH/2 behind its face; face at y = -CASE_PROUD
        case_cfg = newton.ModelBuilder.ShapeConfig(
            density=CASE_MASS / (CASE_FACE_W * CASE_FACE_H * CASE_DEPTH),
            mu=0.4, gap=0.0008, mu_torsional=0.005, mu_rolling=1.0e-4,
            is_visible=_shell_visual_mesh() is None)
        case = b.add_shape_box(
            body, xform=wp.transform(
                wp.vec3(0.0, -CASE_PROUD + 0.5 * CASE_DEPTH, CASE_Z),
                wp.quat_identity()),
            hx=0.5 * CASE_FACE_W, hy=0.5 * CASE_DEPTH, hz=0.5 * CASE_FACE_H,
            cfg=case_cfg, color=(0.13, 0.14, 0.16), label=pad + "_case")
        sv = _shell_visual_mesh()
        if sv is not None:
            mesh_v, c = sv
            vis_cfg = newton.ModelBuilder.ShapeConfig(
                density=0.0, has_shape_collision=False,
                has_particle_collision=False)
            b.add_shape_mesh(
                body, mesh=mesh_v,
                xform=wp.transform(
                    wp.vec3(float(c[0]), float(c[1]), CASE_Z + float(c[2])),
                    wp.quat_identity()),
                cfg=vis_cfg, color=(0.13, 0.14, 0.16),
                label=pad + "_shell")
        # gel: physics slab 2t thick, sensing face at
        # y = -(CASE_PROUD + GEL_PROUD_OF_CASE); slab centre t behind it
        face_y = -(CASE_PROUD + GEL_PROUD_OF_CASE)
        xf = wp.transform(wp.vec3(0.0, face_y + t, CASE_Z), q_gel)
        self.gel_shapes[pad] = attach_gelsight_pad(
            b, body, cfg, xform=xf, label=pad,
            physics_thickness_m=2.0 * t, as_mesh=as_mesh,
            pad_extents_m=(GEL_PAD_W, GEL_PAD_H),
            color=(0.17, 0.18, 0.20))
        for s in b.body_shapes.get(body, []):
            if s != self.gel_shapes[pad]:
                b.add_shape_collision_filter_pair(s, self.gel_shapes[pad])
        self._backing_shape = case
        self.case_shapes = getattr(self, "case_shapes", {})
        self.case_shapes[pad] = case
        self.shapes[pad] = [self.gel_shapes[pad]]
        self.shapes[pad + "_case"] = [case]
        self.tactile_frames[pad] = []
        self._gel_embed = t - pad_proud_m("case") + 0.0   # informational
        self.gel_face_z = t

    def _find_rubber_tip(self, body: int) -> int:
        for s in self.builder.body_shapes.get(body, []):
            scale = np.asarray(self.builder.shape_scale[s], dtype=np.float64)
            if np.allclose(scale, RUBBER_TIP_SCALE, atol=1.0e-4):
                return s
        raise ValueError(f"rubber-tip collision box not found on body {body}")

    def grip_dof(self, object_width_m: float, indent_m: float) -> float:
        """Finger joint target that indents the gels ``indent_m`` per side."""

        pad_proud = pad_proud_m(getattr(self, "mount_style", None))
        # bare-finger face gap = 2*dof + FINGER_FACE_OFFSET; each gel face
        # sits pad_proud in front of the bare face
        return 0.5 * (object_width_m - 2.0 * indent_m
                      - FINGER_FACE_OFFSET) + pad_proud

    # ── finalize: rebuild the pipeline with surface output ──────────────
    def finalize(self, probes, cam_pos, cam_target, **kw):
        # Under the suite's MuJoCo defaults (condim=4: torsional row), any
        # shape with mu_torsional=0.0 is a degenerate constraint --
        # mujoco_warp warns "friction[1] < MJ_MINMU ... may cause NaN" and
        # the contact solve intermittently drops out for single frames
        # (measured: pad forces sawtooth to zero mid-grasp and the grip
        # slips in quantized pops).  Explicit ShapeConfig instances (gel,
        # hydroelastic objects) can carry zeros; clamp the floors on the
        # BUILDER, before make_solver snapshots the model into mjwarp.
        self.builder.shape_material_mu_torsional = [
            max(v, 0.005) for v in self.builder.shape_material_mu_torsional]
        self.builder.shape_material_mu_rolling = [
            max(v, 1.0e-4) for v in self.builder.shape_material_mu_rolling]
        # Extreme-force experiments: the FR3 finger drive (ke=1200 N/m) can
        # deliver at most ~12 N/pad before the servo floor, so GS_FINGER_KE
        # raises the drive gain and GS_FINGER_EFFORT lifts the MuJoCo
        # forcerange clamp (finger dofs 7 and 8).
        fke = os.environ.get("GS_FINGER_KE")
        feff = os.environ.get("GS_FINGER_EFFORT")
        for d in (7, 8):
            if fke:
                self.builder.joint_target_ke[d] = float(fke)
            if feff:
                self.builder.joint_effort_limit[d] = float(feff)
        # the solver must HONOUR the hydroelastic law: rebind newton's
        # MuJoCo contact conversion so the per-contact secant stiffness is
        # a force-space law at a moderate impedance (GS_FORCE_SPACE=0 keeps
        # newton's conversion; see newton_mujoco_force_space.py)
        from hydrogel.mujoco_force_space import install as _fs_install
        _fs_install()
        model = super().finalize(probes, cam_pos, cam_target, **kw)
        # Rig.finalize hardcodes a plain CollisionPipeline; rebuild it with
        # the hydroelastic surface enabled.  Probes were constructed before
        # either pipeline, so the SensorContact-before-CollisionPipeline
        # ordering rule still holds.
        self.pipeline = newton.CollisionPipeline(
            model,
            rigid_contact_max=20_000,
            broad_phase=os.environ.get("GS_BROAD", "explicit"),
            sdf_hydroelastic_config=winning_hydroelastic_config(
                model, self.gel_config, self.gel_shapes.values()),
            verify_buffers=os.environ.get("GS_VERIFY_BUFFERS", "1") != "0",
        )
        self.contacts = self.pipeline.contacts()
        if self.gel_config.gel_stiffening > 0.0:
            print(f"[gelsight] stiffening gel law a={self.gel_config.gel_stiffening:g}")
        for pad, shape in self.gel_shapes.items():
            self.gel_sensors[pad] = HydroelasticGelSight(
                model, self.pipeline, shape, self.gel_config)
        return model

    def _gel_solver_forces(self, state) -> dict[str, float]:
        """Per-pad SOLVER normal force: sum of the gel's contact forces
        (``contacts.force`` after ``solver.update_contacts``) projected on
        the gel's world normal.  Hydroelastic pairs only touch the gel
        shape itself, so this is the load the gel carries (the rubber
        backing's plain contacts are NOT included -- a bottomed-out pad
        shows only what the gel still carries)."""
        c = self.contacts
        if c.force is None:
            return {}
        n = int(c.rigid_contact_count.numpy()[0])
        if n == 0:
            return {pad: 0.0 for pad in self.gel_shapes}
        s0 = c.rigid_contact_shape0.numpy()[:n]
        s1 = c.rigid_contact_shape1.numpy()[:n]
        f = c.force.numpy()[:n]
        f = f[:, :3] if f.ndim == 2 and f.shape[1] >= 3 else np.asarray(f).reshape(n, -1)[:, :3]
        out = {}
        for pad, shape in self.gel_shapes.items():
            sel0 = s0 == shape
            sel1 = s1 == shape
            if not (sel0.any() or sel1.any()):
                out[pad] = 0.0
                continue
            gp, gR = self.gel_sensors[pad]._gel_world_pose(state)
            z = gR[:, 2]
            tot = float((f[sel0] @ z).sum() - (f[sel1] @ z).sum())
            out[pad] = abs(tot)
        return out

    def _sample_gels(self, state, frame_dt: float, forces: dict,
                     solver_forces: dict | None = None) -> None:
        surface = self.pipeline.hydroelastic_sdf.get_contact_surface()
        for pad, sensor in self.gel_sensors.items():
            fs = None if solver_forces is None else solver_forces.get(pad)
            frame = sensor.sample(state, frame_dt, surface=surface,
                                  solver_force_n=fs)
            self.tactile_frames[pad].append(frame)
            forces[f"{pad}_N"] = round(frame.total_normal_force_n, 4)
            forces[f"{pad}_law_N"] = round(frame.pressure_law_force_n, 4)

    # ── run loop: forked from Rig.run (task_rig.py, pinned 2026-08-20) ──
    # One insertion: gel sensors sample straight off the frame-start
    # collide().  Everything else matches the parent implementation so the
    # collision-cadence semantics (README pitfall 5.1) are unchanged.
    def _capture_graph(self, sim_dt: float):
        """CUDA-graph the substep loop (collide every K + MuJoCo step), the way
        newton's own hydro examples do (example_nut_bolt_hydro.capture).
        Ping-pong states are fixed inside the graph; with an even substep
        count the frame ends back in ``self.s0``.  Control targets are
        device arrays updated in place by the task's control callback, so
        they are read fresh by each launch."""

        assert self.substeps % 2 == 0, "graph capture needs an even substep count"
        s0, s1 = self.s0, self.s1
        # Newton 1.5 polls a hydroelastic overflow counter on the host every
        # 120 collisions. A 64-substep card frame, after 64 warmup steps,
        # hits that device-to-host copy INSIDE capture and can segfault.
        # Defer only that host poll; GPU overflow checks remain active.
        hydro = getattr(self.pipeline.narrow_phase, "hydroelastic_sdf", None)
        poll_interval = getattr(hydro, "_host_warning_poll_interval", None)
        if poll_interval is not None:
            hydro._host_warning_poll_interval = max(
                poll_interval, hydro._launch_counter + self.substeps + 1)
        try:
            with wp.ScopedCapture() as cap:
                for si in range(self.substeps):
                    if si % self._collide_every == 0:
                        s0.clear_forces()
                        self.pipeline.collide(s0, self.contacts)
                    self.solver.step(s0, s1, self.control, self.contacts, sim_dt)
                    s0, s1 = s1, s0
        finally:
            if poll_interval is not None:
                hydro._host_warning_poll_interval = poll_interval
        return cap.graph

    # ── run loop: forked from Rig.run (task_rig.py, pinned 2026-08-20) ──
    # Two insertions: gel sensors sample at frame end with the solver's
    # forces, and (GS_GRAPH=1) the substep loop is CUDA-graph captured.
    # Everything else matches the parent implementation so the collision-
    # cadence semantics (README pitfall 5.1) are unchanged.
    def run(self, duration, control_cb, extra_forces_cb=None, print_every=20):
        import time as _time
        newton.use_coord_layout_targets = True
        frame_dt = 1.0 / self.fps
        sim_dt = frame_dt / self.substeps
        n = int(duration * self.fps)
        t = 0.0
        graph_requested = os.environ.get("GS_GRAPH", "0") == "1"
        graph = None
        profile = os.environ.get("GS_PROFILE", "0") == "1"
        prof = {"sim": 0.0, "gels": 0.0, "rest": 0.0}
        collection = None
        if os.environ.get("TB_COLLECT_DIR"):
            from force_bench.data.collection import collector_from_env
            collection = collector_from_env(self)
        for fi in range(n):
            t_a = _time.perf_counter()
            control_cb(t)
            if collection is not None:
                collection.after_control()
            every_substep = self.effective_collide_every_substep()
            use_graph = graph_requested and every_substep and self.substeps % 2 == 0
            gel_forces: dict = {}
            if use_graph:
                if graph is None:
                    # the first frame runs eagerly (kernel compilation), the
                    # graph is captured from the second frame on
                    for si in range(self.substeps):
                        if si % self._collide_every == 0:
                            self.s0.clear_forces()
                            self.pipeline.collide(self.s0, self.contacts)
                        self.solver.step(self.s0, self.s1, self.control,
                                         self.contacts, sim_dt)
                        self.s0, self.s1 = self.s1, self.s0
                    graph = self._capture_graph(sim_dt)
                    print("[gelsight] substep loop CUDA-graph captured "
                          f"({self.substeps} substeps, collide every "
                          f"{self._collide_every})")
                else:
                    wp.capture_launch(graph)
            else:
                if not every_substep:
                    self.s0.clear_forces()
                    self.pipeline.collide(self.s0, self.contacts)
                for si in range(self.substeps):
                    # GS_COLLIDE_EVERY=K re-collides every K substeps (newton's own
                    # hydro examples use 2 with SolverMuJoCo); 1 = every substep
                    if every_substep and si % self._collide_every == 0:
                        self.s0.clear_forces()
                        self.pipeline.collide(self.s0, self.contacts)
                    self.solver.step(self.s0, self.s1, self.control,
                                     self.contacts, sim_dt)
                    self.s0, self.s1 = self.s1, self.s0
            t += frame_dt
            self.solver.update_contacts(self.contacts, self.s0)
            if profile:
                wp.synchronize(); t_b = _time.perf_counter(); prof["sim"] += t_b - t_a
            # SOLVER-CONSISTENT SAMPLING (2026-08-22): the gels are sampled
            # at frame END, off the LAST collide's contact surface (still in
            # the hydroelastic buffer -- update_contacts rewrites only the
            # reduced rigid contacts) with the solver's realized normal
            # force per pad, so the raster's integral is what the robot
            # feels and its shape is the pressure field's.
            self._sample_gels(self.s0, frame_dt, gel_forces,
                              self._gel_solver_forces(self.s0))
            if profile:
                wp.synchronize(); t_c = _time.perf_counter(); prof["gels"] += t_c - t_b
            forces = {}
            for pname, probe in self.probes.items():
                r = probe.sample(self.contacts, self.s0, t)
                forces[f"{pname}_N"] = round(r["force"], 4)
                forces[f"{pname}_fric_N"] = round(r["friction"], 4)
            forces.update(gel_forces)
            if extra_forces_cb:
                forces.update(extra_forces_cb(self.s0, t))
            self.last_forces = forces
            if collection is not None:
                collection.after_step(t)
            if self.rec is not None:
                self.rec.capture(self.s0, t, forces=forces)
            if profile:
                wp.synchronize(); prof["rest"] += _time.perf_counter() - t_c
                if (fi + 1) % 60 == 0:
                    tot = sum(prof.values())
                    print(f"[gelsight-profile] frames {fi-58}-{fi+1}: {tot/60*1e3:.0f} ms/frame "
                          f"(sim {prof['sim']/60*1e3:.0f}, gels {prof['gels']/60*1e3:.1f}, "
                          f"probes+capture {prof['rest']/60*1e3:.1f})")
                    prof = {"sim": 0.0, "gels": 0.0, "rest": 0.0}
            if fi % print_every == 0:
                msg = " ".join(f"{k}={v}" for k, v in list(forces.items())[:5])
                print(f"t={t:5.2f}s {msg}")
        if collection is not None:
            collection.finish()
        return self.rec
