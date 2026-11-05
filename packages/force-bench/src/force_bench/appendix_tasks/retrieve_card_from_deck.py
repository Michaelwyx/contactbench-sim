"""Appendix task "Retrieve single card from deck" (Table I).

    Description: A neatly organized deck of cards is placed in a cardholder
    in the workspace.  Pick up a single card and place it on the table
    without disrupting the rest of the card deck.  Success: the robot
    movement stops after separating a single card from the deck and placing
    it on the table.  The remaining cards should remain a neat deck in the
    cardholder.  The robot should not remain in contact with the task
    objects.

Derived from ``force_bench.tasks.card_pick`` (unchanged on disk): a 4-card
deck leaning in the card_holder; the expert presses the front card's face,
friction-slides it up out of the pack, pinches its protruding top edge and
lifts it 115 mm up its own plane and 80 mm aside.  The suite task ENDS
THERE, card in the air.  The appendix adds the place:

1. CARRY the card at height to open table in front of the holder
   (8 cm out in +x, 9 cm toward the camera; the holder ends at x = -1 mm);
2. LOWER at 25 mm/s until the card's measured lowest point is 3 mm above
   the table (a card hanging from its top edge cannot be laid flat by a
   parallel pinch, so it is set down on its edge ...);
3. OPEN the fingers -- the card tips over and lies flat on the table;
4. RETREAT 8 cm straight up and stop.

Verdict: the picked card lies on the table outside the holder; each of the
three other cards is within 5 mm and 3 deg of where it started and still
in the slot; the robot touches neither cards nor holder at the end.

Run: python -m force_bench.appendix_tasks.retrieve_card_from_deck
"""

from __future__ import annotations

import numpy as np

from force_bench.appendix_tasks.derive import Variant, run_variant
from force_bench.appendix_tasks.common import ease

PLACE_XY = (0.08, -0.09)  # m, open table in front of the holder (footprint
                          # ends at x = -1 mm) and inside the suite camera's
                          # view (the first run set it down at (0.13, 0) --
                          # clear, but at the very edge of the frame)
T_CARRY = 1.6
V_LOWER = 0.025          # m/s
T_LOWER_MAX = 3.0
SET_DOWN_Z = 0.003       # m, card's lowest point above the table at release
OPEN_W = 0.030
OTHERS = ("card_a", "card_b", "card_c")
OTHER_DISP_MAX = 0.005
OTHER_TILT_MAX = 3.0


def _R(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class CardVariant(Variant):
    def extra_duration(self):
        return T_CARRY + T_LOWER_MAX + self.t_open + self.t_retreat + self.t_still

    def card_low_z(self, rig) -> float:
        mod = self.mod
        q = rig.s0.body_q.numpy()[rig.body_index("card_picked")]
        half = np.array([mod.CARD_T / 2, 0.0425, mod.CARD_H / 2])
        R = _R(q[3:7])
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                            for sz in (-1, 1)]) * half
        return float((q[:3] + corners @ R.T)[:, 2].min())

    def ending(self, rig, t, tau):
        p0, q0, f0 = self.start_cmd
        if not hasattr(self, "_pl"):
            self._pl = {"z": None, "t_down": None}
        P = self._pl
        above = np.array([PLACE_XY[0], PLACE_XY[1], p0[2]])
        if tau < T_CARRY:
            u = ease(tau / T_CARRY)
            rig.drive_arm(p0 + (above - p0) * u, q0, fingers=f0)
            return
        tau -= T_CARRY
        if P["t_down"] is None:
            if P["z"] is None:
                P["z"] = above[2]
            low = self.card_low_z(rig)
            if low > SET_DOWN_Z and tau < T_LOWER_MAX:
                P["z"] -= V_LOWER / rig.fps
            else:
                P["t_down"] = tau
            rig.drive_arm(np.array([above[0], above[1], P["z"]]), q0, fingers=f0)
            return
        tau -= P["t_down"]
        pd = np.array([above[0], above[1], P["z"]])
        if tau < self.t_open:
            u = ease(tau / self.t_open)
            rig.drive_arm(pd, q0, fingers=f0 + (OPEN_W - f0) * u)
            return
        u = ease((tau - self.t_open) / self.t_retreat)
        rig.drive_arm(pd + np.array([0.0, 0.0, self.retreat_m * u]), q0, fingers=OPEN_W)

    def drop_checks(self):
        # the suite's task_success scores the card IN THE AIR at the end
        return ("task_success",)

    def criteria(self, rig, qa):
        mod = self.mod
        d = np.load(rig.out_dir / "rollout_states.npz", allow_pickle=True)
        labels = list(d["body_labels"])
        bq = d["body_q"]
        pk = bq[:, labels.index("card_picked")]
        R_end = _R(pk[-1, 3:7])
        face_n = R_end[:, 0]                       # card thin axis
        flat = abs(float(face_n[2])) > 0.95        # lying on a face
        on_table = float(pk[-1, 2]) < 0.004
        outside = float(pk[-1, 0]) > 0.0
        others = {}
        ok_others = True
        for lbl in OTHERS:
            c = bq[:, labels.index(lbl)]
            disp = float(np.linalg.norm(c[-1, :3] - c[0, :3]))
            n0 = _R(c[0, 3:7])[:, 0]
            n1 = _R(c[-1, 3:7])[:, 0]
            tilt = float(np.degrees(np.arccos(np.clip(abs(n0 @ n1), -1, 1))))
            in_slot = mod.SLOT_X[0] - 0.01 < float(c[-1, 0]) < mod.SLOT_X[1] + 0.01
            others[lbl] = {"final_disp_mm": round(disp * 1000, 2),
                           "tilt_change_deg": round(tilt, 2), "in_slot": bool(in_slot)}
            ok_others &= disp < OTHER_DISP_MAX and tilt < OTHER_TILT_MAX and in_slot
        t_low = (self.t_cut_eff + T_CARRY + (self._pl["t_down"] or 0.0))
        crit = {
            "single_card_on_table": {
                "pass": bool(flat and on_table and outside),
                "final_card_centre_mm": [round(float(v) * 1000, 1) for v in pk[-1, :3]],
                "lying_flat": bool(flat), "on_table": bool(on_table),
                "outside_holder_footprint": bool(outside),
                "set_down_at_s": round(t_low, 2)},
            "deck_remains_neat": {
                "pass": bool(ok_others), "others": others,
                "max_disp_mm": OTHER_DISP_MAX * 1000, "max_tilt_deg": OTHER_TILT_MAX},
        }
        crit["released_and_clear"] = self.released_and_clear(
            qa, t_from=t_low + self.t_open + self.t_retreat, min_move=0.05)
        return crit


VARIANT = CardVariant(
    name="retrieve_card_from_deck",
    source="force_bench.tasks.card_pick",
    appendix_task="Retrieve single card from deck",
    spec="The robot movement stops after separating a single card from the deck "
         "and placing it on the table. The remaining cards should remain a neat "
         "deck in the cardholder. The robot should not remain in contact with the "
         "task objects.",
    objects=("card_picked", "card_a", "card_b", "card_c"),
    static_shapes=("holder",),
    changes=[
        "appended place: carry to open table in front of the holder, lower "
        "until the card's edge is 3 mm above the table, open (card tips flat), "
        "8 cm retreat, still",
        "verdict: card flat on the table outside the holder; other cards < 5 mm / "
        "< 3 deg from start and still in the slot; robot out of contact",
        "suite task_success (card held in the air) dropped",
    ],
    call_original_after_cut=False,
    t_open=0.5, retreat_m=0.08, t_retreat=1.2, t_still=1.2,
)


def main():
    return run_variant(VARIANT)


if __name__ == "__main__":
    import warp as wp
    wp.init()
    main()
