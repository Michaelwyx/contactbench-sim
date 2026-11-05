"""Runtime patch: make SolverMuJoCo honour Newton's per-contact stiffness in FORCE space.

WHY (2026-08-23, packages/force-bench/src/force_bench/tactile/press_probe.py ``solver_press``): Newton's
hydroelastic pipeline hands SolverMuJoCo a per-contact secant stiffness
``c`` (N/m) so that ``F = c * (-contact_distance)`` reproduces the
pressure-field force (Elandt et al.).  newton 1.5's
``convert_newton_contacts_to_mjwarp_kernel`` converts it to a solref with
``timeconst = sqrt(1 / (c * (1 - dmax)))`` and NO inverse-weight scaling,
while MuJoCo realizes ``k_eff = k / (invweight * (1 - dmax))`` (see
solver_mujoco.py, "Shape-material contact stiffness and damping").  The
realized force-space stiffness is therefore ``c / invweight`` -- it scales
with the effective mass of the two bodies, so the same gel is 9x stiffer
than its law under a driven 16 mm box, ~1x under a 15 g chopstick, and
rigid under the door key (a link in a 1.2 kg door articulation) or the
captive nut.  Newton's shape-material FORCE_SPACE mode already pre-scales
``ke * (invw_a + invw_b) * (1 - dmax)``; this module applies the same factor
in the per-contact branch (a per-contact solref exists there, so the
static-geom mixing caveat of the shape-material path does not arise).

HOW: newton's kernels.py is read from the installed package, the
per-contact branch is rewritten, the result is written as an importable
module under ``sim/_generated/`` (Warp kernels cannot be defined from an
exec'd string) and ``solver_mujoco.convert_newton_contacts_to_mjwarp_kernel``
-- a module global looked up at every launch -- is rebound to the patched
kernel.  Nothing in site-packages is modified.  ``install()`` is idempotent
and a no-op when ``GS_FORCE_SPACE=0``.  Only contacts carrying a per-contact
stiffness (the hydroelastic ones) are affected; plain contacts keep the
shape-material path.
"""

from __future__ import annotations

import importlib.util
import os
import pathlib
import re
import sys

from hydrogel.paths import CACHE_DIR

_GEN_DIR = CACHE_DIR / "generated"
_MOD_NAME = "newton_mujoco_kernels_force_space"
_installed = {"state": None}
# impedance for hydroelastic (per-contact-stiffness) contacts; GS_HYDRO_IMP overrides
HYDRO_IMP = float(os.environ.get("GS_HYDRO_IMP", "0.5"))

OLD = """            if contact_ke > 0.0:
                imp = solimp[1]
                solimp = vec5(imp, imp, 0.001, 1.0, 0.5)
                contact_ke = contact_ke * (1.0 - imp)
                kd = rigid_contact_damping[tid]
                if kd > 0.0:
                    timeconst = 2.0 / kd
                    dampratio = wp.sqrt(1.0 / (timeconst * timeconst * contact_ke))
                else:
                    timeconst = wp.sqrt(1.0 / contact_ke)
                    dampratio = 1.0
                solref = wp.vec2(timeconst, dampratio)

            friction_scale = rigid_contact_friction[tid]
"""
NEW = """            if contact_ke > 0.0:
                # hydrogel (packages/hydrogel/src/hydrogel/mujoco_force_space.py): make the
                # per-contact stiffness a FORCE-SPACE law the solver honours.
                # MuJoCo's rest solution for a soft contact is
                #     F = d/(1-d) * m_eff * k * r        (k = d/(dmax^2 tc^2 dr^2))
                # so for F = c * r (c = the hydroelastic secant, N/m) we need
                #     k = c * invweight * (1-d)/d,   tc = 1/sqrt(d*k),  dr = 1
                # at a MODERATE impedance d: newton's default (geom solimp
                # dmax = 0.95) makes the constraint 95 % rigid through its
                # regularization, and the realized penetration is then a
                # mass- and settle-time-dependent creep that ignores c
                # (measured: the same gel 2-5x stiffer than its law for a
                # 0.2 kg driven box, 9x for 0.1 kg of armature, rigid for an
                # articulated key).  d = HYDRO_IMP keeps the rest solution
                # exact and lets the body reach it within ~tc.
                imp = wp.static(HYDRO_IMP)
                solimp = vec5(imp, imp, 0.001, 1.0, 0.5)
                invw_a_c = float(0.0)
                invw_b_c = float(0.0)
                if body_a >= 0:
                    invw_a_c = body_invweight0[worldid, mj_body_a][0]
                if body_b >= 0:
                    invw_b_c = body_invweight0[worldid, mj_body_b][0]
                m_inv_c = invw_a_c + invw_b_c
                if m_inv_c <= 0.0:
                    m_inv_c = 1.0
                k_needed = contact_ke * m_inv_c * (1.0 - imp) / imp
                timeconst = 1.0 / wp.sqrt(imp * k_needed)
                dampratio = 1.0
                kd = rigid_contact_damping[tid]
                if kd > 0.0:
                    dampratio = kd * m_inv_c / (2.0 * wp.sqrt(k_needed))
                solref = wp.vec2(timeconst, dampratio)

            friction_scale = rigid_contact_friction[tid]
"""


def enabled() -> bool:
    return os.environ.get("GS_FORCE_SPACE", "1") != "0"


def _generate_module() -> pathlib.Path:
    import newton

    src_path = pathlib.Path(newton.__file__).parent / "_src" / "solvers" / "mujoco" / "kernels.py"
    src = src_path.read_text()
    if src.count(OLD) != 1:
        raise RuntimeError(
            "newton_mujoco_force_space: per-contact stiffness branch not found once in "
            f"{src_path}; newton version differs, refusing to patch")
    src = src.replace(OLD, NEW)
    src = src.replace("import warp as wp\n", f"import warp as wp\nHYDRO_IMP = {HYDRO_IMP!r}\n", 1)
    # relative imports -> absolute (the generated file lives outside the package)
    src = re.sub(r"^from \.\.\.(\S*)", r"from newton._src.\1", src, flags=re.M)
    src = re.sub(r"^from \.\.(\S*)", r"from newton._src.solvers.\1", src, flags=re.M)
    src = re.sub(r"^from \.(\S*)", r"from newton._src.solvers.mujoco.\1", src, flags=re.M)
    # `from newton._src.` (three dots resolved) may read `from newton._src.core...` fine;
    # a bare `from newton._src.solvers.` from `from ..` also fine.
    _GEN_DIR.mkdir(parents=True, exist_ok=True)
    out = _GEN_DIR / f"{_MOD_NAME}.py"
    header = ("# GENERATED by packages/hydrogel/src/hydrogel/mujoco_force_space.py from\n"
              f"# {src_path}\n# Do not edit; regenerated at import.\n")
    if not out.exists() or out.read_text() != header + src:
        out.write_text(header + src)
    return out


def install(verbose: bool = True) -> bool:
    """Rebind the solver's contact-conversion kernel.  Returns True if active."""

    if not enabled():
        if verbose:
            print("[force-space] GS_FORCE_SPACE=0: newton's per-contact solref kept")
        return False
    if _installed["state"] is not None:
        return _installed["state"]
    import newton._src.solvers.mujoco.solver_mujoco as sm

    path = _generate_module()
    spec = importlib.util.spec_from_file_location(_MOD_NAME, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[_MOD_NAME] = mod
    spec.loader.exec_module(mod)
    sm.convert_newton_contacts_to_mjwarp_kernel = mod.convert_newton_contacts_to_mjwarp_kernel
    _installed["state"] = True
    if verbose:
        print("[force-space] SolverMuJoCo per-contact stiffness -> force space "
              f"(patched kernel {path.name})")
    return True
