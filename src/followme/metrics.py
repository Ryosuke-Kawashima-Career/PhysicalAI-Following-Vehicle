"""Acceptance metrics for a run (plan section 1: R2, R3, R4)."""
import math

import numpy as np

from . import config as C
from .world import Circle, clearance


class Metrics:
    def __init__(self, world, ped_names):
        self.world = world
        self.t, self.dist, self.behind, self.in_band, self.clear, self.clear_with = [], [], [], [], [], []
        self.walking = []  # traveller moving this tick
        self._trav_last = None
        self.crossed_between = set()
        self.path_len = {n: 0.0 for n in ped_names}
        self._last = {}

    def record(self, t, cart_box, trav_pos, trav_heading, peds):
        trav = np.asarray(trav_pos, dtype=float)
        me = np.array([cart_box.x, cart_box.y])
        d = float(np.linalg.norm(trav - me))
        fwd = np.array([math.cos(trav_heading), math.sin(trav_heading)])
        behind = float((me - trav) @ fwd) <= 0.3  # cart not ahead of the traveller
        people = [Circle(float(trav[0]), float(trav[1]), 0.3, "traveller")] + [p.circle() for p in peds]
        cands = [(clearance(cart_box, o), o.name) for o in self.world.static + people]
        cl, who = min(cands)
        self.t.append(t)
        self.dist.append(d)
        self.behind.append(behind)
        self.in_band.append(behind and C.BAND[0] <= d <= C.BAND[1])
        self.walking.append(self._trav_last is not None and float(np.linalg.norm(trav - self._trav_last)) > 1e-3)
        self._trav_last = trav.copy()
        self.clear.append(cl)
        self.clear_with.append(who)
        # R2: a pedestrian cuts the segment traveller -> cart
        seg = trav - me
        L2 = float(seg @ seg)
        for p in peds:
            if L2 > 1e-6:
                u = float((p.p - me) @ seg) / L2
                if 0.1 < u < 0.9:
                    r = p.p - me
                    off = abs(float(seg[0] * r[1] - seg[1] * r[0])) / math.sqrt(L2)
                    if off < 0.45:
                        self.crossed_between.add(p.name)
            if p.name in self._last:
                self.path_len[p.name] += float(np.linalg.norm(p.p - self._last[p.name]))
            self._last[p.name] = p.p.copy()

    def summary(self):
        clear = np.array(self.clear)
        i_min = int(np.argmin(clear))
        coll = clear < C.MIN_CLEARANCE
        moving = sorted(n for n, L in self.path_len.items() if L > 2.0)
        s = {
            "ticks": len(self.t),
            "band_fraction": float(np.mean(self.in_band)),
            "band_fraction_walking": float(np.mean(np.array(self.in_band)[np.array(self.walking)])) if any(self.walking) else None,
            "max_distance": float(np.max(self.dist)),
            "mean_distance": float(np.mean(self.dist)),
            "min_clearance": float(clear[i_min]),
            "min_clearance_with": self.clear_with[i_min],
            "min_clearance_t": float(self.t[i_min]),
            "collision_ticks": int(np.sum(coll)),
            "first_collision_t": float(self.t[int(np.argmax(coll))]) if coll.any() else None,
            "moving_pedestrians": moving,
            "crossed_between": sorted(self.crossed_between),
        }
        s["pass"] = {
            "R2_moving_and_crossing": len(moving) >= 2 and len(self.crossed_between) >= 1,
            "R3_band": s["band_fraction"] >= C.BAND_MIN_FRACTION,
            "R3_max_distance": s["max_distance"] < C.MAX_DISTANCE,
            "R4_no_collision": s["collision_ticks"] == 0,
        }
        s["all_pass"] = all(s["pass"].values())
        return s
