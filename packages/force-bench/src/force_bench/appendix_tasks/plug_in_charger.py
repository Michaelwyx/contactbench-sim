"""Appendix task "Plug in charger" (Table I).

    Description: An outlet is placed in the workspace.  Fully insert the
    charging block held in hand into the outlet.  Success: the charging
    block is fully inserted into the outlet, such that it does not
    noticeably move when further pushed toward the outlet.  The robot
    movement stops after terminating the insertion motion, moving away
    from, and breaking contact with the task objects.

Derived from ``force_bench.tasks.plug_insert_arm`` (unchanged on disk).  The
suite task already starts with the charger in the hand, prongs aligned
4 mm above the slots, and inserts it on a lead-bounded, measured-depth
ramp to the prong-base plateau (20.09 mm).  It then squeezes harder and
PULLS THE CHARGER BACK OUT (multi-bite extraction) and ends holding it in
the air.  The appendix task stops at insertion, so the variant takes over
the moment the suite's seated hold ends (state-dependent cut on the
expert's own phase flag):

1. PUSH TEST -- "does not noticeably move when further pushed": the press
   lead is ramped until the outlet reacts >= 8 N axially (or 1.2 mm of
   lead) and held; the depth gained under that push must be < 0.3 mm;
2. RELEASE -- the fingers open past the 17.5 mm brick width at the pushed
   pose (wrist frozen so nothing drags the brick sideways or lifts it);
3. RETREAT -- 6 cm straight back along the approach axis, then still.

Verdict: seated within 0.5 mm of the plateau, push test, still seated
after the hand left (< 0.5 mm change), robot out of contact.

Run: python -m force_bench.appendix_tasks.plug_in_charger
"""

from __future__ import annotations

import numpy as np

from force_bench.appendix_tasks.derive import Variant, run_variant
from force_bench.appendix_tasks.common import ease, retreat_pose

PUSH_F = 8.0          # N of outlet reaction the push test must reach
PUSH_LEAD_MAX = 0.0012
PUSH_RATE = 0.0010    # m/s of extra lead
PUSH_MOVE_MAX = 0.0003
SEAT_GAP_MAX = 0.0005
STAY_MAX = 0.0005
OPEN_TO = 0.024       # finger dof: brick blocks at 0.0175


class ChargerVariant(Variant):
    t_push: float = 1.6
    # collide before every substep (the suite task collides once per frame): a block that is dropped, swung or pressed
    # in fast moves more per frame than the outlet's 2.2 mm face wall.  The pad fit
    # (examples/force_response/probe_charger.py) collides every substep as well.
    collide_every_substep = True

    def configure(self, mod):
        # the extraction budget is never used; keep the run short
        mod.T_EXT_MAX = 4.0

    def cut_time(self, duration):
        return None

    def should_cut(self, rig, t):
        st = self.ns.get("st")
        return st is not None and st["phase"] in ("precrush", "extract")

    def extra_duration(self):
        return self.t_push + self.t_open + self.t_retreat + self.t_still

    def ending(self, rig, t, tau):
        st = self.ns["st"]
        if st["phase"] != "done":
            # the suite QA keys phases off this flag; "done" keeps its
            # windows (hold / extract) free of the appended phases
            st["phase"] = "done"
            self._push = {"lead": 0.0, "peak_F": 0.0, "t_hold": None,
                          "dz0": None, "dz_max": None}
        p0, q0, f0 = self.start_cmd
        log = self.ns["log"]
        dz = float(log[-1][1]) if log else 0.0
        env_z = float(log[-1][3]) if log else 0.0
        P = self._push
        a = -np.array([0.0, 0.0, 1.0])         # insertion direction (down)
        if tau < self.t_push:
            if P["dz0"] is None:
                P["dz0"] = dz
            P["peak_F"] = max(P["peak_F"], env_z)
            P["dz_max"] = dz if P["dz_max"] is None else max(P["dz_max"], dz)
            # ramp the press until the outlet pushes back PUSH_F, then hold
            # it.  No back-off afterwards: the pads re-anchor lower on the
            # brick under the push, so returning the hand to the pre-push
            # height LIFTS the plug (measured 0.35 mm on the first run) --
            # a person pushes and then simply lets go, and so does this.
            if env_z < PUSH_F:
                P["lead"] = min(P["lead"] + PUSH_RATE / 60.0, PUSH_LEAD_MAX)
            rig.drive_arm(p0 + a * P["lead"], q0, fingers=f0)
            return
        tau -= self.t_push
        pp = p0 + a * P["lead"]
        if tau < self.t_open:
            u = ease(tau / self.t_open)
            rig.drive_arm(pp, q0, fingers=f0 + (OPEN_TO - f0) * u)
            return
        p1 = retreat_pose(pp, q0, self.retreat_m)
        u = ease((tau - self.t_open) / self.t_retreat)
        rig.drive_arm(pp + (p1 - pp) * u, q0, fingers=OPEN_TO)

    def released_window_start(self):
        return self.t_cut_eff + self.t_push + self.t_open + self.t_retreat

    def drop_checks(self):
        # extraction is not part of the appendix task
        return ("task_success", "multi_bite_extraction")

    def push_start(self) -> float:
        """Time the push test starts (the cut, unless a variant inserts phases before it)."""
        return self.t_cut_eff

    def criteria(self, rig, qa):
        mod = self.mod
        L = np.array(self.ns["log"])
        t_a, dz_a = L[:, 0], L[:, 1]
        t0 = self.push_start()
        pre = (t_a < t0) & (t_a >= t0 - 0.3)
        dz_seat = float(np.median(dz_a[pre]))
        rel0 = t0 + self.t_push
        before_release = float(np.median(dz_a[(t_a >= rel0 - 0.2) & (t_a < rel0)]))
        final = float(np.median(dz_a[-15:]))
        P = self._push
        d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
        pq = d["body_q"][:, list(d["body_labels"]).index("plug"), 3:7]
        i_rel = int(np.searchsorted(d["t"], rel0))

        def zaxis(q):
            x, y, z, w = q
            return np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        tilt = float(np.degrees(np.arccos(np.clip(zaxis(pq[-1]) @ zaxis(pq[i_rel]), -1, 1))))
        crit = {
            "fully_inserted": {
                "pass": bool(self.ns["DZ_SEAT"] - dz_seat <= SEAT_GAP_MAX),
                "seat_depth_mm": round(dz_seat * 1000, 3),
                "plateau_depth_mm": round(self.ns["DZ_SEAT"] * 1000, 3),
                "gap_mm": round((self.ns["DZ_SEAT"] - dz_seat) * 1000, 3),
                "tolerance_mm": SEAT_GAP_MAX * 1000},
            "no_motion_when_pushed": {
                "pass": bool(P["peak_F"] >= 0.75 * PUSH_F
                             and (P["dz_max"] - P["dz0"]) < PUSH_MOVE_MAX),
                "push_force_peak_N": round(P["peak_F"], 2),
                "push_force_target_N": PUSH_F,
                "extra_lead_peak_mm": round(PUSH_LEAD_MAX * 1000, 2),
                "depth_gained_mm": round((P["dz_max"] - P["dz0"]) * 1000, 3),
                "threshold_mm": PUSH_MOVE_MAX * 1000},
            "stays_seated_after_release": {
                "pass": bool(abs(final - before_release) < STAY_MAX),
                "depth_before_release_mm": round(before_release * 1000, 3),
                "final_depth_mm": round(final * 1000, 3),
                "threshold_mm": STAY_MAX * 1000,
                "tilt_after_release_deg": round(tilt, 2),
                "note": "depth = plug body origin; the brick's CoM sits ~15 mm off "
                        "the blade axis, so once unheld it leans on the plateau and "
                        "the origin reads a little deeper"},
        }
        crit["released_and_clear"] = self.released_and_clear(
            qa, t_from=self.released_window_start(), min_move=0.04)
        return crit


VARIANT = ChargerVariant(
    name="plug_in_charger",
    source="force_bench.tasks.plug_insert_arm",
    appendix_task="Plug in charger",
    spec="The charging block is fully inserted into the outlet, such that it does "
         "not noticeably move when further pushed toward the outlet. The robot "
         "movement stops after terminating the insertion motion, moving away from, "
         "and breaking contact with the task objects.",
    objects=("plug", "pad_wide", "pad_narrow"),
    static_shapes=("shell",),
    changes=[
        "no extraction: the expert is cut when its seated hold ends",
        "push test: force-ramped extra press (>= 8 N outlet reaction), depth gain "
        "must stay < 0.3 mm; the fingers then open at the pushed pose",
        "release (fingers open past the brick) + 6 cm retreat + 1 s still",
        "suite task_success / multi_bite_extraction (extraction gates) dropped",
        "contacts found before every substep (the suite task: once per frame), so a block moving several mm per "
        "frame cannot cross the outlet's face wall between two contact updates",
    ],
    call_original_after_cut=False,
    retreat_m=0.06, t_open=0.6, t_retreat=1.4, t_still=1.0,
)


def main():
    return run_variant(VARIANT)


if __name__ == "__main__":
    import warp as wp
    wp.init()
    main()
