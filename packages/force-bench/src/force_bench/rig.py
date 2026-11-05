"""Shared rig for the ContactBench task rollouts.

Wraps the boilerplate proven in the first three rollouts (solver loop,
recorder, QA save) plus reusable manipulation drivers:

  * Rig.static_mesh / free_mesh / link_mesh  — asset loading with SDF
  * Rig.revolute_part / prismatic_part       — asset dof with spring drive
                                               (the printed-spring replacement)
  * Rig.spinner / Rig.slider                 — driven "hand" bodies coupled to
                                               a part via a compliant loop d6
                                               (grasp abstraction; loop joints
                                               need NO add_articulation)
  * Rig.run(...)                             — phase loop, force sampling,
                                               video/state capture, QA

Conventions: meters, z-up, SolverMuJoCo consuming Newton's SDF contacts,
32 substeps @ 60 fps.  The modelling rules of docs/physics_notes.md are
honoured here.
"""

from __future__ import annotations

import json
import os

from pathlib import Path

import numpy as np
import trimesh
import warp as wp

import newton

from force_bench.probes import ForceProbe
from force_bench.robot.franka import FrankaIK, add_franka, down_grasp_quat
from force_bench.paths import OUTPUT_DIR
from force_bench.qa import RolloutQA
from force_bench.recorder import RolloutRecorder
from force_bench.assets import load_tm, make_newton_mesh, set_body_mass

JD = newton.ModelBuilder.JointDofConfig
DEFAULT_CFG = dict(ke=1.0e5, kd=1.0e3, gap=0.0008, mu_torsional=0.0, mu_rolling=0.0)


def quat_np(q) -> wp.quat:
    return wp.quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def body_yaw(body_q_row, axis=2) -> float:
    """Rotation angle [rad] of a body about a world axis (for logging)."""
    q = body_q_row
    M = trimesh.transformations.quaternion_matrix([q[6], q[3], q[4], q[5]])
    eul = trimesh.transformations.euler_from_matrix(M)
    return float(eul[axis])



# ── the solver ──────────────────────────────────────────────────────────
# SolverMuJoCo is the only backend: SolverXPBD was retired on 2026-09-11,
# nothing in the bench used it any more.  It won the migration because
# XPBD does not support joint_effort_limit (its own docstring lists it as
# unsupported), so a force-controlled Franka hand can only be modelled as a
# stiff position spring -- grip force = stiffness x overdrive -- which puts
# omega*dt at 0.93 on the lightest gripped parts and diverges.  The MuJoCo
# backend maps joint_effort_limit onto MuJoCo's actuator `forcerange`, the
# same clamp Isaac Lab and Drake use.
#
# Crucially it can consume NEWTON-SUPPLIED contacts, so the SDF non-convex
# meshes, the per-pair collision filters and the SensorContact force probes
# all survive the swap -- only dynamics and actuation move.  (MuJoCo's own
# mesh path is convex-hull only, which would stop threads threading and stop
# a key ring linking a key at all.)

# Solver options matching newton's own examples/contacts/
# example_nut_bolt_sdf.py, the reference for feeding Newton SDF contacts to
# MuJoCo.  Every value below is overridable per task via **kw, or per run
# via the TB_MJ_* environment variables.
_MJ_OPTS = dict(
    solver="newton",
    integrator="implicitfast",
    cone="elliptic",
    impratio=1.0,
    ls_iterations=100,
)
MJ_NCONMAX = int(os.environ.get("TB_MJ_NCON", "8000"))
MJ_NJMAX = int(os.environ.get("TB_MJ_NJMAX", str(4 * MJ_NCONMAX)))
MJ_CONTACT_KE = float(os.environ.get("TB_MJ_KE", "1e7"))
MJ_CONTACT_KD = float(os.environ.get("TB_MJ_KD", "1e4"))
MJ_ARM_ARMATURE = float(os.environ.get("TB_MJ_ARMATURE", "0.1"))
# IMPRATIO IS THE GRASP KNOB NOBODY MENTIONS.  MuJoCo solves friction and
# normal constraints in the same system, and `impratio` is the ratio of
# frictional to normal constraint impedance -- at its default of 1.0 the
# friction constraint is far softer than the normal one, so a pinch creeps
# tangentially even when its Coulomb capacity is many times the load.  It
# is not a fudge factor for weak friction: mu is transferred correctly
# (rigid_contact_friction -> mjw_data.contact.friction), the CAPACITY is
# right and the STIFFNESS of the constraint that enforces it is not.
# Empty string leaves it at MuJoCo's default so an A/B can isolate it.
# COLLISION CADENCE IS SOLVER-SPECIFIC AND THE SUITE'S RULE WAS DERIVED
# UNDER XPBD.  Colliding once per frame is what makes XPBD's friction
# anchors accumulate a whole frame of resistance; MuJoCo has no anchors, so
# the same choice just hands it 8 substeps of STALE contact geometry, which
# a rotating grasp can pivot through.  Override to compare per-frame against
# per-substep on any task without editing it.
_cs = os.environ.get("TB_COLLIDE_SUBSTEP", "")
COLLIDE_SUBSTEP_FORCE = None if _cs == "" else (_cs == "1")

# TORSIONAL FRICTION IS OFF BY DEFAULT AND TWO TASKS FAIL IN TORSION.
# MuJoCo's `condim` defaults to 3 -- normal plus two tangential, and NO
# torsion about the contact normal -- and newton's shape_material_mu_torsional
# defaults to 0.005, which is negligible even where condim allows it.
# A fork pinched out of a tray (it rotated 100+ deg out of a 6 N/pad pinch
# under a 14 mN.m gravity couple) and a bottle cap turned by the wrist (the
# wrist swept 0.444 turns, the cap followed 0.009, with 20 N/pad and 66x the
# needed torque capacity) both failed by TWISTING, not by sliding, which is
# the one mode a condim-3 contact cannot resist except through the spread of
# its patch.  DEFAULT 4: normal, two tangential, AND torsion.  Measured on
# that fork, twisting out of a healthy two-sided 6 N/pad pinch: condim 3
# gives 53.05 deg of rotation and FAILS, condim 4 at the UNCHANGED
# mu_torsional gives 20.25 deg and PASSES.
# The coefficient is deliberately NOT raised.  MuJoCo's torsional friction
# has units of LENGTH -- it is approximately the contact patch radius --
# so newton's 0.005 default is 5 mm against a 17.5 mm pad, which is
# defensible, while the 0.5 that drives the rotation to 0.21 deg would be
# a half-METRE patch and is not.  A number that passes the gate by being
# unphysical is worse than one that fails honestly.
_cd = os.environ.get("TB_MJ_CONDIM", "4")
MJ_CONDIM = int(_cd) if _cd else None
_mut = os.environ.get("TB_MU_TORSIONAL", "")
MU_TORSIONAL = float(_mut) if _mut else None

_imp = os.environ.get("TB_MJ_IMPRATIO", "")
MJ_IMPRATIO = float(_imp) if _imp else None


def _apply_torsion(model) -> None:
    """Turn on torsional friction: condim 4+ AND a non-trivial coefficient.

    Both are needed.  condim 3 has no torsional row at all, so raising mu
    alone does nothing; condim 4 with newton's default mu_torsional of
    0.005 has a row worth almost nothing.
    """
    if MU_TORSIONAL is not None:
        a = model.shape_material_mu_torsional.numpy()
        a[:] = MU_TORSIONAL
        model.shape_material_mu_torsional.assign(a)
    if MJ_CONDIM is not None:
        mj = getattr(model, "mujoco", None)
        if mj is not None and hasattr(mj, "condim"):
            c = mj.condim.numpy()
            c[:] = MJ_CONDIM
            mj.condim.assign(c)


def mujoco_contact_tuning(model) -> dict:
    """Re-point the two model fields XPBD ignores and MuJoCo does not.

    See make_solver's docstring for the measurements.  Returns what it
    changed so a task can log it.  Safe to call on any model: the armature
    is only written when the first nine joints really are the FR3's, which
    `Rig.add_robot` guarantees by asserting the robot is added FIRST.
    """
    changed = {}
    if MJ_CONTACT_KE > 0:
        model.shape_material_ke.fill_(MJ_CONTACT_KE)
        changed["shape_material_ke"] = MJ_CONTACT_KE
    if MJ_CONTACT_KD > 0:
        model.shape_material_kd.fill_(MJ_CONTACT_KD)
        changed["shape_material_kd"] = MJ_CONTACT_KD
    labels = list(model.joint_label)[:9]
    if MJ_ARM_ARMATURE > 0 and len(labels) == 9 \
            and all(str(l).startswith("fr3/") for l in labels):
        a = model.joint_armature.numpy()
        a[:9] = MJ_ARM_ARMATURE
        model.joint_armature.assign(a)
        changed["joint_armature[0:9]"] = MJ_ARM_ARMATURE
    return changed


def make_solver(model, iterations: int = 10, **kw):
    """Build the configured solver.  Keep call sites solver-agnostic.

    `use_mujoco_contacts=False` IS THE WHOLE MIGRATION AND IT IS NOT THE
    DEFAULT.  SolverMuJoCo ships with `use_mujoco_contacts=True`, which sets
    `mjw_model.opt.run_collision_detection` and makes MuJoCo run its OWN
    collision -- and MuJoCo compiles every mesh geom through its convex-hull
    path.  Only when the flag is False does `step()` reach
    `_convert_contacts_to_mjwarp(...)` and consume the contacts Newton
    computed (solver_mujoco.py: `if not
    self.mjw_model.opt.run_collision_detection:`).

    Measured on the default (i.e. convex hulls, WRONG): the nut_bolt rig
    turned 0.308 of the 6.767 turns it does under XPBD and nut_bolt_arm
    reported pitch_mm = 0.0 with mean_slip 0.9996 -- the thread had become
    two smooth cylinders.  That is this flag, not a limit of the solver.

    THREE MORE THINGS ARE ALSO NOT THE DEFAULT, and each of them fails
    SILENTLY -- as a warning on stdout, or as nothing at all:

    1. `nconmax` / `njmax`.  MuJoCo pre-allocates its contact and constraint
       arrays and estimates the size from the INITIAL state.  On the FR3 +
       nut_bolt scene that estimate is 256 against the 950-1500 contacts
       Newton's pipeline actually produces, and the overflow only prints
       "Number of Newton contacts (914) exceeded MJWarp limit (256)".  The
       arm then runs on a quarter of its contacts and walks 209 mm through
       the table.  With the budget right the same session tunnels 3.2 mm.
    2. Contact stiffness.  `ShapeConfig.ke/kd` default to 1e5/1e3 here.
       XPBD IGNORES BOTH (its contacts are positional constraints -- README
       pitfall 3.1), so those numbers were never tuned against anything;
       MuJoCo turns them into the per-contact solref.  newton's own
       examples/contacts/example_nut_bolt_sdf.py uses 1e7/1e4 and labels it
       "Contact stiffness for MuJoCo solver".  Measured on the gate's
       unguarded over-squeeze: 1e5 -> 209.0 mm of tunnelling, 1e7 -> 3.2 mm,
       1e8 -> 33.2 mm (too stiff for the substep).  1e7 is a floor AND a
       ceiling, not a direction.
    3. `joint_armature` on the arm.  It is 0.0 out of the URDF, and a 12000
       N.m/rad position servo with no rotor inertia is what is left of the
       conditioning problem under MuJoCo.  0.1 (the value MuJoCo's own
       Franka model carries) takes the unguarded gate from FLAKY 4/6 to
       PASS 6/6.  It is applied to the FR3's 9 dofs ONLY: joint_armature
       spans every dof, and the gripped parts sit on FREE joints, where 0.1
       would swamp a 10 g nut's ~2e-7 kg.m^2 inertia by six orders.
    """
    # A task may rebuild its solver after finalize to change iterations.
    # Keep sensor requirements attached to that model across the rebuild.
    for key, value in getattr(model, "_force_bench_solver_options", {}).items():
        kw.setdefault(key, value)
    kw.setdefault("use_mujoco_contacts", False)
    for k, v in _MJ_OPTS.items():
        kw.setdefault(k, v)
    kw.setdefault("nconmax", MJ_NCONMAX)
    kw.setdefault("njmax", MJ_NJMAX)
    # _MJ_OPTS already PINS impratio=1.0, so this has to overwrite, not
    # setdefault.  It was a setdefault first, and three tray experiments
    # at "impratio 5 / 20 / 20+substep" all silently ran at 1.0 and
    # agreed with each other -- which read as "impratio does not matter
    # here" when the knob had simply never turned.  A row of settings
    # that agree to two decimals is a broken knob until proven otherwise.
    if MJ_IMPRATIO is not None:
        kw["impratio"] = MJ_IMPRATIO
    if MJ_CONDIM is not None or MU_TORSIONAL is not None:
        _apply_torsion(model)
    mujoco_contact_tuning(model)
    effective_iterations = max(int(iterations), 15)
    solver = newton.solvers.SolverMuJoCo(model, iterations=effective_iterations, **kw)
    solver._force_bench_effective_options = {"iterations": effective_iterations, **kw}
    return solver


# ── MuJoCo JOINT LIMITS ARE NOT LIMITS UNLESS YOU SAY SO ────────────────
# 2026-08-14, and it is a MIGRATION-WIDE defect, not a task bug.  XPBD
# resolves a joint limit as a positional constraint, so `limit_lower/upper`
# is a wall and README pitfall 4.8 ("the envelope IS the joint limits") holds
# exactly.  Under SolverMuJoCo a limit is a soft constraint whose hardness
# comes from `jnt_solref`, and newton DERIVES that from
# `joint_limit_ke`/`joint_limit_kd` (kernels.py `convert_solref` /
# `update_jnt_solref_from_invweight0_kernel`):
#     timeconst = 2 / (kd * invweight0 * (1 - dmax))
#     dampratio = kd/2 * sqrt(d_r / (ke * invweight0 * (1 - dmax)))
# The builder defaults are ke = 1e4, kd = 10, and on a 0.05 kg part that is
# timeconst 0.2 s / dampratio 0.05 -- a limit that takes a fifth of a second
# to develop force.  Against a position drive that delivers what it is asked
# for, that is not a wall, it is a suggestion.
#
# MEASURED on a one-joint rig (0.05 kg body, the door rack's own 20000 N/m
# position drive, commanded +500 mm past a 30 mm slot; "excursion" = how far
# past the slot it ends up; sim/../scratch limit_probe):
#
#   limit_ke/kd    solimplimit dmax   effort_limit    excursion past slot
#   1e4/10 DEFAULT      0.95           none              425.6 mm
#   1e4/10              0.95            200 N            161.6 mm
#   1e4/10              0.95             50 N             79.2 mm
#   1e6/1e3             0.95           none               13.2 mm
#   raw (2dt, 0.5)      0.95           none               13.2 mm
#   raw (2dt, 0.5)      0.95            200 N              1.09 mm
#   raw (2dt, 0.5)      0.95             50 N              0.26 mm
#   raw (2dt, 1.0)      0.95             50 N              1.35 mm
#   1e4/10             0.9999          none              469.9 mm  <- WORSE
#
# THREE THINGS THAT TABLE SAYS, ALL OF WHICH COST A RUN TO LEARN:
#  1. 425.6 mm is a door scene's "latch rack driven 422 mm out of its 32 mm
#     slot" reproduced in a rig with no meshes, no arm and no contact.  The
#     door did not have a door bug.
#  2. `solimplimit` is a TRAP.  Raising dmax is the standard MuJoCo way to
#     harden a constraint, and here it makes things WORSE, because newton
#     divides by (1 - dmax) when it converts the gains -- dmax 0.9999 sends
#     timeconst to 100 s.  Never touch dmax while the gains are in
#     force-space mode.
#  3. NEITHER LEVER IS SUFFICIENT ALONE.  A hard solref still yields 13 mm
#     under an unbounded drive; an effort limit alone still yields 79 mm at
#     50 N.  The pair is what works: a position actuator PLUS a force
#     clamp.
#
# The raw form is used rather than raising limit_ke/limit_kd because the
# force-space conversion is scaled by the DOF's inverse inertia: the same
# 1e6/1e3 that lands on (2dt, 0.5) for this 0.05 kg slider lands on
# dampratio 50 -- i.e. still soft -- for a light revolute.  A raw solref is
# mass-independent and says what it means.
#
# OPT-IN, on purpose: the thread scenes (`nut_bolt`) are already A/B-verified
# under MuJoCo and their numbers must not move underneath them.  A task asks
# for this by setting `rig.hard_joint_limits = True` BEFORE `rig.finalize()`.
MJ_LIMIT_DAMPRATIO = float(os.environ.get("TB_MJ_LIMIT_ZETA", "0.5"))


def mujoco_hard_limits(model, sim_dt: float,
                       dampratio: float = MJ_LIMIT_DAMPRATIO) -> dict:
    """Author a RAW MuJoCo `solreflimit` on every dof.  MuJoCo path only.

    `timeconst = 2 * sim_dt` is the hardest value MuJoCo will honour -- its
    `refsafe` clamp raises anything smaller to twice the timestep, which is
    exactly how the default gains end up soft (their timeconst is 2 us, so
    the clamp costs five orders of stiffness).  Needs
    `SolverMuJoCo.register_custom_attributes(builder)` to have run before
    `finalize`, which `Rig.finalize` does when `hard_joint_limits` is set.
    """
    from newton._src.solvers.mujoco.constants import SOLREF_MODE_RAW
    attrs = getattr(model, "mujoco", None)
    if attrs is None or not hasattr(attrs, "solreflimit"):
        raise RuntimeError(
            "mujoco_hard_limits needs SolverMuJoCo.register_custom_attributes"
            " on the BUILDER before finalize (set rig.hard_joint_limits)")
    tc = 2.0 * float(sim_dt)
    sr = attrs.solreflimit.numpy()
    sr[:] = (tc, float(dampratio))
    attrs.solreflimit.assign(sr)
    md = attrs.solreflimit_mode.numpy()
    md[:] = SOLREF_MODE_RAW
    attrs.solreflimit_mode.assign(md)
    return {"solreflimit": (tc, float(dampratio)), "dofs": int(len(sr))}


def set_finger_effort_limit(builder, limit_N: float = 20.0, dofs=(7, 8)):
    """Clamp the finger drives to a physically real grasp force.

    MuJoCo maps it onto the actuator's forcerange, which is what lets a task
    lower FINGER_KE instead of buying grip force with stiffness.  20 N per
    finger is what newton's own Franka example uses.
    """
    for d in dofs:
        builder.joint_effort_limit[d] = limit_N


class Rig:
    def __init__(self, name: str, gravity: float = -9.81):
        self.name = name
        self.solver_options: dict = {}
        self.out_dir = OUTPUT_DIR / name
        self.builder = newton.ModelBuilder(gravity=(0.0, 0.0, gravity))
        self.builder.add_ground_plane()
        self.meshes: dict[str, tuple[trimesh.Trimesh, np.ndarray]] = {}
        self.shapes: dict[str, list[int]] = {}
        self._joint_of: dict[str, int] = {}   # label -> joint id (1-dof parts)
        self._drive_ke: dict[str, float] = {}
        # MuJoCo's contact compliance is TIMESTEP-BOUND: its solref floor is
        # a couple of substeps, so the stiffest contact it can represent --
        # and therefore the penetration a given load produces -- scales with
        # dt.  Measured: ~0.17 mm of sink at 9.6 N (probe_grip_slip), and
        # a door's deadbolt sank 1.30 mm into its strike pocket under a
        # 116 N transmitted load, which is the same ~60 kN/m.
        # Contact stiffness cannot be bought with shape_material_ke (inert,
        # see mujoco_contact_tuning); it can only be bought with dt.
        # 32 UNDER MuJoCo, 16 UNDER XPBD.  16 was chosen under XPBD and is
        # below the threshold where a loaded contact resolves under MuJoCo
        # at all: that deadbolt sank 1.296 mm into a 0.6 mm tolerance at 16
        # and 0.046 mm at 32, and a fork-in-tray pick's dense penetration
        # audit went from FAIL to PASS across the same step.
        self.substeps = int(os.environ.get(
            "TB_SUBSTEPS", "32"))
        self.fps = 60
        self.collide_every_substep = False   # see run(): grasp friction
        # called after every substep's solver step (None: nothing); DexRig
        # averages the hand's contact forces over the frame with it
        self.substep_hook = None
        # opt into hard MuJoCo joint limits (see mujoco_hard_limits above).
        # Set BEFORE finalize().
        self.hard_joint_limits = False
        # ...and how hard.  dampratio is a REAL trade-off, not a tolerance:
        # a limit that reacts faster also reacts HARDER, so the same stop
        # that holds a slider inside 0.3 mm makes a stopped rotor ring.
        # Measured on task_screwdriver's seat (the screw revolute's upper
        # stop), sweeping this value alone:
        #     zeta   seat overshoot   peak drive torque
        #     0.5       2.7 deg          293 mN.m
        #     1.0       4.5 deg          172
        #     1.5       9.3 deg          157
        #     2.0      11.1 deg          130
        #     5.0      37.6 deg           73
        # so a task picks the point on that curve its own mechanism needs.
        self.limit_dampratio = MJ_LIMIT_DAMPRATIO

    # ── shapes ──────────────────────────────────────────────────────────
    def cfg(self, mu=0.5, density=1240.0, **over):
        base = dict(DEFAULT_CFG)
        base.update(over)
        return newton.ModelBuilder.ShapeConfig(mu=mu, density=density, **base)

    def table(self, center_xy=(0.0, 0.0), half=0.3):
        sid = self.builder.add_shape_box(
            -1, xform=wp.transform(wp.vec3(center_xy[0], center_xy[1], -0.005),
                                   wp.quat_identity()),
            hx=half, hy=half, hz=0.005, cfg=self.cfg(), label="table")
        self.shapes["table"] = [sid]

    def _mesh(self, asset, stem, sdf, res, nb, margin):
        tm = load_tm(asset, stem)
        mesh, center = make_newton_mesh(tm, sdf=sdf, sdf_resolution=res,
                                        narrow_band=nb, margin=margin)
        return tm, mesh, np.asarray(center)

    def static_mesh(self, asset, stem, label, sdf=True, res=320, nb=0.002,
                    margin=0.0012, mu=0.5, xform=None):
        tm, mesh, c = self._mesh(asset, stem, sdf, res, nb, margin)
        xf = xform or wp.transform(wp.vec3(*c), wp.quat_identity())
        sid = self.builder.add_shape_mesh(-1, mesh=mesh, xform=xf,
                                          cfg=self.cfg(mu=mu), label=label)
        self.meshes[label] = (tm, c)
        self.shapes[label] = [sid]
        return c

    def free_mesh(self, asset, stem, label, sdf=True, res=320, nb=0.002,
                  margin=0.0012, mu=0.5, mass=None, xform=None):
        tm, mesh, c = self._mesh(asset, stem, sdf, res, nb, margin)
        xf = xform or wp.transform(wp.vec3(*c), wp.quat_identity())
        body = self.builder.add_body(xform=xf, label=label)
        sid = self.builder.add_shape_mesh(body, mesh=mesh, cfg=self.cfg(mu=mu),
                                          label=label + "_shape")
        self.shapes[label] = [sid]
        if mass:
            set_body_mass(self.builder, body, mass)
        self.meshes[label] = (tm, c)
        return body, c

    def link_meshes(self, asset, stems, label, sdf=True, res=320, nb=0.002,
                    margin=0.0012, mu=0.5, xform=None):
        """One link carrying several welded body meshes (e.g. knob + gears)."""
        tms, meshes, centers = [], [], []
        for s in stems:
            tm, mesh, c = self._mesh(asset, s, sdf, res, nb, margin)
            tms.append(tm), meshes.append(mesh), centers.append(c)
        c0 = centers[0]
        xf = xform or wp.transform(wp.vec3(*c0), wp.quat_identity())
        body = self.builder.add_link(xform=xf, label=label)
        self.shapes[label] = []
        for mesh, c in zip(meshes, centers):
            off = wp.transform(wp.vec3(*(np.asarray(c) - c0)), wp.quat_identity())
            self.shapes[label].append(self.builder.add_shape_mesh(
                body, xform=off, mesh=mesh, cfg=self.cfg(mu=mu), label=None))
        merged = trimesh.util.concatenate(tms)
        self.meshes[label] = (merged, c0)
        return body, c0

    # ── articulated asset parts (spring -> drive) ──────────────────────
    def revolute_part(self, body, pivot, axis, label, spring_ke=0.0,
                      spring_kd=0.01, limits=(-np.pi, np.pi), mass=None):
        j = self.builder.add_joint_revolute(
            parent=-1, child=body, axis=axis,
            parent_xform=wp.transform(wp.vec3(*pivot), wp.quat_identity()),
            child_xform=wp.transform(
                wp.vec3(*(np.asarray(pivot) - self._body_pos(body))),
                wp.quat_identity()),
            target_pos=0.0, target_ke=spring_ke, target_kd=spring_kd,
            limit_lower=limits[0], limit_upper=limits[1],
        )
        self.builder.add_articulation([j])
        if mass:
            set_body_mass(self.builder, body, mass)
        self._joint_of[label] = j
        self._drive_ke[label] = spring_ke
        return j

    def prismatic_part(self, body, origin, axis, label, spring_ke=0.0,
                       spring_kd=1.0, limits=(0.0, 0.01), mass=None):
        j = self.builder.add_joint_prismatic(
            parent=-1, child=body, axis=axis,
            parent_xform=wp.transform(wp.vec3(*origin), wp.quat_identity()),
            child_xform=wp.transform(
                wp.vec3(*(np.asarray(origin) - self._body_pos(body))),
                wp.quat_identity()),
            target_pos=0.0, target_ke=spring_ke, target_kd=spring_kd,
            limit_lower=limits[0], limit_upper=limits[1],
        )
        self.builder.add_articulation([j])
        if mass:
            set_body_mass(self.builder, body, mass)
        self._joint_of[label] = j
        self._drive_ke[label] = spring_ke
        return j

    def d6_part(self, body, origin, label, linear_axes=(), angular_axes=(), mass=None):
        j = self.builder.add_joint_d6(
            parent=-1, child=body,
            linear_axes=linear_axes or None, angular_axes=angular_axes or None,
            parent_xform=wp.transform(wp.vec3(*origin), wp.quat_identity()),
            child_xform=wp.transform(
                wp.vec3(*(np.asarray(origin) - self._body_pos(body))),
                wp.quat_identity()),
        )
        self.builder.add_articulation([j])
        if mass:
            set_body_mass(self.builder, body, mass)
        self._joint_of[label] = j
        return j

    def _body_pos(self, body) -> np.ndarray:
        t = self.builder.body_q[body]
        return np.array([t[0], t[1], t[2]])

    # ── manipulation drivers (grasp abstraction via loop d6) ───────────
    def spinner(self, target_body, pivot, axis, label, drive_ke=60.0,
                drive_kd=3.0, grip_lin=6000.0, grip_ang=30.0, offset=0.03,
                velocity_mode=False, hand_mass=0.05):
        """A driven disk above `pivot` coupled to target_body; rotating the
        disk twists the part through the compliant grip.

        velocity_mode: joint driven by target velocity (much higher effective
        authority in XPBD than position drives — use for stiff mechanisms,
        close the position loop in the control callback via target_qd)."""
        ax = np.asarray(axis, dtype=float)
        pos = np.asarray(pivot) + ax * offset
        body = self.builder.add_link(
            xform=wp.transform(wp.vec3(*pos), wp.quat_identity()), label=label)
        hand_sid = self.builder.add_shape_cylinder(
            body, radius=0.018, half_height=0.008,
            cfg=self.cfg(density=2000.0), label=label + "_shape")
        self.shapes[label] = [hand_sid]
        # the compliant coupling IS the grip — the hand prop must never
        # contact the part it holds (form-lock jams the mechanism)
        for ts in self.builder.body_shapes.get(target_body, []):
            self.builder.add_shape_collision_filter_pair(hand_sid, ts)
        if velocity_mode:
            j = self.builder.add_joint_revolute(
                parent=-1, child=body, axis=tuple(ax),
                parent_xform=wp.transform(wp.vec3(*pos), wp.quat_identity()),
                target_vel=0.0, target_ke=0.0, target_kd=drive_kd,
            )
        else:
            j = self.builder.add_joint_revolute(
                parent=-1, child=body, axis=tuple(ax),
                parent_xform=wp.transform(wp.vec3(*pos), wp.quat_identity()),
                target_pos=0.0, target_ke=drive_ke, target_kd=drive_kd,
            )
        self.builder.add_articulation([j])
        # XPBD torque authority scales with the driver's inertia — a light
        # hand cannot transmit mechanism-scale torques no matter the gains
        set_body_mass(self.builder, body, hand_mass)
        # loop coupling (NO add_articulation — loop joints ride along)
        rel = wp.transform(
            wp.vec3(*(self._body_pos(target_body) - pos)), wp.quat_identity())
        self.builder.add_joint_d6(
            parent=body, child=target_body,
            linear_axes=tuple(JD(axis=a, target_ke=grip_lin, target_kd=25.0)
                              for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            angular_axes=tuple(JD(axis=a, target_ke=grip_ang, target_kd=0.8)
                               for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            parent_xform=rel, child_xform=wp.transform_identity(),
        )
        self._joint_of[label] = j
        return body, j

    def slider(self, target_body, origin, axis, label, drive_ke=0.0,
               drive_kd=300.0, grip_lin=6000.0, grip_ang=30.0,
               limits=(-0.2, 0.2)):
        """A driven carriage coupled to target_body for linear moves."""
        pos = np.asarray(origin, dtype=float)
        body = self.builder.add_link(
            xform=wp.transform(wp.vec3(*pos), wp.quat_identity()), label=label)
        hand_sid = self.builder.add_shape_box(body, hx=0.012, hy=0.012, hz=0.012,
                                              cfg=self.cfg(density=2000.0),
                                              label=label + "_shape")
        self.shapes[label] = [hand_sid]
        for ts in self.builder.body_shapes.get(target_body, []):
            self.builder.add_shape_collision_filter_pair(hand_sid, ts)
        j = self.builder.add_joint_prismatic(
            parent=-1, child=body, axis=tuple(axis),
            parent_xform=wp.transform(wp.vec3(*pos), wp.quat_identity()),
            target_vel=0.0, target_ke=drive_ke, target_kd=drive_kd,
            limit_lower=limits[0], limit_upper=limits[1],
        )
        self.builder.add_articulation([j])
        set_body_mass(self.builder, body, 0.05)
        rel = wp.transform(
            wp.vec3(*(self._body_pos(target_body) - pos)), wp.quat_identity())
        self.builder.add_joint_d6(
            parent=body, child=target_body,
            linear_axes=tuple(JD(axis=a, target_ke=grip_lin, target_kd=25.0)
                              for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            angular_axes=tuple(JD(axis=a, target_ke=grip_ang, target_kd=0.8)
                               for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1))),
            parent_xform=rel, child_xform=wp.transform_identity(),
        )
        self._joint_of[label] = j
        return body, j

    # ── Franka manipulation (robot operates the object) ────────────────
    def add_robot(self, base_xform):
        """FR3 added FIRST (coords 0..8) + kinematic IK. Call before assets."""
        assert self.builder.joint_count == 0, "add_robot must come first"
        add_franka(self.builder, base_xform)
        self.ik = FrankaIK(base_xform)
        self.robot_base = base_xform

    def robot_grip(self, target_body, filter_fingers=True, **_legacy):
        """The finger contact policy for a gripped body: never collide the
        ARM with it and, with ``filter_fingers=False``, let the FINGERTIPS
        touch it so the grasp is real fingertip contact.

        THE WRIST->OBJECT COUPLING IS GONE.  This used to build a d6 spring
        from link7 to the object (``TB_STABILIZER=1``) that could carry a
        part its fingers never gripped (measured in a chopstick pick:
        0.24 N of pad force while the stick rose 13.7 cm).  Under
        SolverMuJoCo that joint is a LOOP the backend refuses ("Multiple
        joints lead to body N"), so once XPBD was retired (2026-09-11) it
        could not exist at all.  The legacy gain arguments (grip_lin,
        grip_ang, tilt_ang, stabilizer) are accepted and ignored so the
        call sites read as they did; the FINGERS do the work.
        """
        self._filter_arm_from(target_body, filter_fingers)

    def _filter_arm_from(self, target_body, filter_fingers):
        """Never collide the arm with the gripped object -- except, when
        `filter_fingers=False`, the FINGERTIPS: a real grasp needs real
        finger contact (squeeze via finger joint targets)."""
        hand_body = self.builder.body_label.index("fr3/fr3_link7")
        for ts in self.builder.body_shapes.get(target_body, []):
            for bl in range(0, hand_body + 3):
                if not filter_fingers and "finger" in self.builder.body_label[bl]:
                    continue
                for s in self.builder.body_shapes.get(bl, []):
                    self.builder.add_shape_collision_filter_pair(s, ts)

    def solve_arm_start(self, tcp_pos, tcp_quat):
        q0 = self.ik.solve(tcp_pos, tcp_quat, iterations=64)
        self.builder.joint_q[:9] = q0.tolist()
        self.builder.joint_target_q[:9] = q0.tolist()
        return q0

    def drive_arm(self, tcp_pos, tcp_quat, fingers=0.04):
        q_sol = self.ik.solve(tcp_pos, tcp_quat, iterations=16)
        qt = self.control.joint_target_q.numpy()
        qt[:7] = q_sol[:7]
        qt[7:9] = fingers
        self.control.joint_target_q.assign(qt)

    def filter_pair(self, shape_a, shape_b):
        self.builder.add_shape_collision_filter_pair(shape_a, shape_b)

    def filter_labels(self, label_a: str, label_b: str):
        for sa in self.shapes[label_a]:
            for sb in self.shapes[label_b]:
                self.builder.add_shape_collision_filter_pair(sa, sb)

    # ── run loop ────────────────────────────────────────────────────────
    def finalize(self, probes: list[str], cam_pos, cam_target,
                 cam2_pos=None, cam2_target=None, record=True):
        if self.hard_joint_limits or MJ_CONDIM is not None:
            # condim is a mujoco CUSTOM attribute, so it does not exist
            # until this has run -- registering it for the torsion knob too
            newton.solvers.SolverMuJoCo.register_custom_attributes(self.builder)
        self.model = self.builder.finalize()
        self.model._force_bench_solver_options = dict(self.solver_options)
        if self.hard_joint_limits:
            self.limit_tuning = mujoco_hard_limits(
                self.model, 1.0 / self.fps / self.substeps,
                dampratio=self.limit_dampratio)
        self.solver = make_solver(self.model, iterations=10, **self.solver_options)
        self.s0, self.s1 = self.model.state(), self.model.state()
        self.control = self.model.control()
        self.probes = {p: ForceProbe(self.model, sensing_body=p) for p in probes}
        # arm meshes + asset SDFs flood the default contact buffer; small
        # gate features get silently dropped when it saturates
        self.model.rigid_contact_max = 20000
        self.pipeline = newton.CollisionPipeline(
            self.model, rigid_contact_max=20000, reduce_contacts=True,
            broad_phase="sap")
        self.contacts = self.pipeline.contacts()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.s0)
        self.rec = RolloutRecorder(self.model, self.out_dir,
                                   cam_pos=cam_pos, cam_target=cam_target,
                                   cam2_pos=cam2_pos, cam2_target=cam2_target,
                                   fps=self.fps) if record else None
        self.labels = list(self.model.body_label)
        return self.model

    def body_index(self, label):
        return self.labels.index(label)

    def target_q(self, joint_label, value):
        starts = self.model.joint_target_q_start.numpy()
        idx = int(starts[self._joint_of[joint_label]])
        qt = self.control.joint_target_q.numpy()
        qt[idx] = value
        self.control.joint_target_q.assign(qt)

    def target_qd(self, joint_label, value):
        starts = self.model.joint_target_q_start.numpy()
        idx = int(starts[self._joint_of[joint_label]])
        qd = self.control.joint_target_qd.numpy()
        qd[idx] = value
        self.control.joint_target_qd.assign(qd)

    def effective_collide_every_substep(self):
        """Resolve explicit run override, sensor requirement, then task cadence."""
        if COLLIDE_SUBSTEP_FORCE is not None:
            return COLLIDE_SUBSTEP_FORCE
        override = getattr(self, "collide_substep_override", None)
        return self.collide_every_substep if override is None else override

    def run(self, duration, control_cb, extra_forces_cb=None, print_every=20):
        newton.use_coord_layout_targets = True
        frame_dt = 1.0 / self.fps
        sim_dt = frame_dt / self.substeps
        n = int(duration * self.fps)
        t = 0.0
        collection = None
        substep_hook = getattr(self, "substep_hook", None)
        if os.environ.get("TB_COLLECT_DIR"):
            from force_bench.data.collection import collector_from_env
            collection = collector_from_env(self)
        try:
            for fi in range(n):
                control_cb(t)
                if collection is not None:
                    collection.after_control()
                # COLLISION CADENCE IS A GRASP PARAMETER.  XPBD dry friction is a
                # positional constraint on the contact ANCHORS recorded by
                # collide(): the tangential correction is clamped to mu*lambda_n
                # and measured from the anchor pair.  Re-colliding every substep
                # re-anchors the contact at the CURRENT (already slipped) pose,
                # so a grasp can only ever oppose ONE substep of slip and creeps
                # under any steady tangential load no matter how hard it squeezes
                # (measured: a connector slid 4.7 mm in 0.28 s out of a 19.6 N/pad
                # pinch whose Coulomb capacity was 32 N).  Colliding ONCE PER
                # FRAME lets the anchors accumulate the full frame of resistance,
                # which is what makes friction-only fingertip grasping work.
                # Set collide_every_substep=True for scenes with fast free-flight
                # contact where missing new contacts mid-frame matters more.
                every_substep = self.effective_collide_every_substep()
                if not every_substep:
                    self.s0.clear_forces()
                    self.pipeline.collide(self.s0, self.contacts)
                for _ in range(self.substeps):
                    if every_substep:
                        self.s0.clear_forces()
                        self.pipeline.collide(self.s0, self.contacts)
                    self.solver.step(self.s0, self.s1, self.control,
                                     self.contacts, sim_dt)
                    self.s0, self.s1 = self.s1, self.s0
                    if substep_hook is not None:
                        substep_hook()
                t += frame_dt
                self.solver.update_contacts(self.contacts, self.s0)
                forces = {}
                for pname, probe in self.probes.items():
                    r = probe.sample(self.contacts, self.s0, t)
                    forces[f"{pname}_N"] = round(r["force"], 4)
                    forces[f"{pname}_fric_N"] = round(r["friction"], 4)
                if extra_forces_cb:
                    forces.update(extra_forces_cb(self.s0, t))
                if collection is not None:
                    collection.after_step(t)
                if self.rec is not None:
                    self.rec.capture(self.s0, t, forces=forces)
                if fi % print_every == 0:
                    msg = " ".join(f"{k}={v}" for k, v in list(forces.items())[:5])
                    print(f"t={t:5.2f}s {msg}")
        finally:
            # also when a task ends its rollout early by raising from a callback
            # (appendix_tasks.common.StopRun): the frames stepped so far are the episode
            if collection is not None and collection.writer.actions:
                collection.finish(discard_pending=True)
        return self.rec

    def qa(self, paths) -> RolloutQA:
        q = RolloutQA(paths["states"])
        q.check_finite()
        q.check_velocity(vmax=2.5)
        return q

    def finish(self, qa_obj) -> dict:
        qa_obj.save(self.out_dir / "rollout_qa.json")
        report = json.load(open(self.out_dir / "rollout_qa.json"))
        print("QA PASS" if report["pass"] else "QA FAIL")
        return report
