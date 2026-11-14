"""Follow-me controller for a unicycle cart (plan Step 3).

1. Breadcrumbs: the traveller's position is recorded every `crumb_spacing` metres it moves, so the cart reuses the
   traveller's own (obstacle-free) path instead of cutting corners.
2. Target: the cart keeps `follow_distance` metres of *path length* behind the traveller. Speed = traveller speed
   + k_gap * (gap error).
3. Pure pursuit: steer towards a lookahead point on the breadcrumb path (lookahead grows with speed).
4. Avoidance: obstacles and pedestrians closer than `avoid_radius` push the desired heading away from them.
5. Safety:
   - cone stop: anything closer than `cone_stop_clearance` inside +-30 deg ahead -> stop;
   - hold: traveller standing and closer than `hold_distance` -> stay put;
   - predictive filter: every candidate speed is rolled out (this tick at that speed, then hardest braking) against
     static obstacles and people predicted at constant velocity; the fastest candidate that keeps `safety_margin`
     wins, otherwise the one with the most clearance (normally: brake hard).
"""
import math
from dataclasses import dataclass

import numpy as np

from .world import Box, Circle, away_vector, clearance


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class CartState:
    x: float
    y: float
    yaw: float
    v: float = 0.0
    w: float = 0.0

    def box(self, length, width, x=None, y=None, yaw=None):
        return Box(self.x if x is None else x, self.y if y is None else y, length / 2, width / 2,
                   self.yaw if yaw is None else yaw, "follow_cart")


class FollowController:
    def __init__(self, cfg, world, cart: CartState, traveller_pos):
        self.c = cfg.cart
        self.dt = cfg.dt
        self.world = world
        self.length, self.width = world.actors["follow_cart"]["size"][:2]
        # seed the trail with a straight line from the cart to the traveller
        p0, p1 = np.array([cart.x, cart.y]), np.asarray(traveller_pos, dtype=float)
        n = max(1, int(np.linalg.norm(p1 - p0) / self.c.crumb_spacing))
        self.crumbs = [p0 + (p1 - p0) * i / n for i in range(n + 1)]
        self.seg = 0  # index of the trail segment the cart is on
        self.debug = {}

    def length_width(self):
        return self.length, self.width

    # --- breadcrumbs -------------------------------------------------------------------------------------------
    def _update_crumbs(self, trav):
        last = self.crumbs[-1]
        d = np.linalg.norm(trav - last)
        while d >= self.c.crumb_spacing:
            last = last + (trav - last) * (self.c.crumb_spacing / d)
            self.crumbs.append(last)
            d = np.linalg.norm(trav - last)

    def _path(self, trav):
        pts = np.array(self.crumbs + [trav])
        cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        return pts, cum

    def _project(self, pts, cum, p):
        """Arc length of the cart's projection onto the trail (searching forward from the last segment)."""
        best = (math.inf, self.seg, 0.0)
        for i in range(max(0, self.seg - 2), min(len(pts) - 1, self.seg + 40)):
            a, b = pts[i], pts[i + 1]
            ab = b - a
            L2 = float(ab @ ab)
            t = 0.0 if L2 < 1e-12 else min(1.0, max(0.0, float((p - a) @ ab) / L2))
            d = float(np.linalg.norm(p - (a + t * ab)))
            if d < best[0] - 1e-9:
                best = (d, i, t)
        _, i, t = best
        self.seg = i
        return cum[i] + t * (cum[i + 1] - cum[i])

    def _point_at(self, pts, cum, s):
        s = min(max(s, 0.0), cum[-1])
        i = int(np.clip(np.searchsorted(cum, s, side="right") - 1, 0, len(pts) - 2))
        seg = cum[i + 1] - cum[i]
        return pts[i] + (0.0 if seg < 1e-12 else (s - cum[i]) / seg) * (pts[i + 1] - pts[i])

    # --- main --------------------------------------------------------------------------------------------------
    def command(self, cart: CartState, trav_pos, trav_vel, people):
        """people: list of (Circle, velocity ndarray) - pedestrians only (the traveller is passed separately)."""
        c = self.c
        trav = np.asarray(trav_pos, dtype=float)
        self._update_crumbs(trav)
        pts, cum = self._path(trav)
        me = np.array([cart.x, cart.y])
        s_cart = self._project(pts, cum, me)
        s_trav = cum[-1]
        gap = s_trav - s_cart
        v_trav = float(np.linalg.norm(trav_vel))
        dist = float(np.linalg.norm(trav - me))

        # 2. speed from the path-length gap
        v_des = min(max(v_trav + c.k_gap * (gap - c.follow_distance), 0.0), c.v_max)

        # 3. pure pursuit towards a lookahead point on the trail (never closer than ~1 m to the traveller)
        ld = c.lookahead_min + c.lookahead_gain * cart.v
        s_la = min(s_cart + ld, max(s_cart + 0.05, s_trav - 0.9))
        la = self._point_at(pts, cum, s_la)
        if np.linalg.norm(la - me) < 0.05:
            la = trav
        heading = np.array([math.cos(cart.yaw), math.sin(cart.yaw)])
        to_la = la - me
        dvec = to_la / max(np.linalg.norm(to_la), 1e-9)

        # 4. avoidance: push the heading away from nearby obstacles / pedestrians in front of the cart
        box = cart.box(self.length, self.width)
        obstacles = [(o, np.zeros(2)) for o in self.world.static] + list(people)
        push = np.zeros(2)
        for o, vel in obstacles:
            if np.any(vel):  # moving person: avoid where they will be, so the cart aims behind a crosser
                o = Circle(o.x + vel[0] * c.avoid_predict, o.y + vel[1] * c.avoid_predict, o.r, o.name)
            cl = clearance(box, o)
            if cl < c.avoid_radius:
                n = np.array(away_vector(o, cart.x, cart.y))
                if n @ heading < 0.2:  # only things ahead / beside count
                    push += c.avoid_gain * (1.0 / max(cl, 0.05) - 1.0 / c.avoid_radius) * n
        dvec = dvec + push
        alpha = wrap(math.atan2(dvec[1], dvec[0]) - cart.yaw)
        v_des *= max(0.0, math.cos(alpha))

        # 5a. hold position while the traveller stands close by
        holding = v_trav < 0.05 and dist < c.hold_distance
        if holding:
            v_des = 0.0
        # 5b. cone stop
        cone_hit = None
        trav_circle = Circle(float(trav[0]), float(trav[1]), 0.3, "traveller")
        for o, _ in obstacles + [(trav_circle, np.zeros(2))]:
            cl = clearance(box, o)
            if cl < c.cone_stop_clearance:
                ox, oy = (o.x, o.y)
                if isinstance(o, Box):  # use the closest point of long boxes (walls), not their centre
                    n = away_vector(o, cart.x, cart.y)
                    ox, oy = cart.x - n[0], cart.y - n[1]
                bearing = abs(wrap(math.atan2(oy - cart.y, ox - cart.x) - cart.yaw))
                if bearing < c.cone_half_angle:
                    cone_hit = o.name
                    v_des = 0.0
                    break

        # steering: pure-pursuit curvature, with a floor speed so the cart can still turn while slow
        v_ref = max(v_des, 0.3)
        w_des = 2.0 * v_ref * math.sin(alpha) / max(ld, 1e-6)
        if (holding or v_des == 0.0) and abs(alpha) < 0.5:
            w_des = 0.0
        w_des = min(max(w_des, -c.w_max), c.w_max)

        v_cmd, w_cmd, margin = self._safety_filter(cart, v_des, w_des, obstacles, trav_circle, trav_vel)
        self.debug = dict(s_cart=s_cart, gap=gap, v_des=v_des, w_des=w_des, la=la, holding=holding,
                          cone=cone_hit, margin=margin, alpha=alpha)
        return v_cmd, w_cmd

    def _safety_filter(self, cart, v_des, w_des, obstacles, trav_circle, trav_vel):
        c, dt = self.c, self.dt
        v_lo = max(0.0, cart.v - c.a_brake * dt)
        v_hi = min(c.v_max, cart.v + c.a_acc * dt, max(v_des, v_lo))
        w_lo, w_hi = cart.w - c.alpha_max * dt, cart.w + c.alpha_max * dt
        w_t = min(max(w_des, w_lo), w_hi)
        w_t = min(max(w_t, -c.w_max), c.w_max)
        movers = [(o, np.asarray(vel, dtype=float)) for o, vel in obstacles if np.any(vel)]
        movers.append((trav_circle, np.asarray(trav_vel, dtype=float)))
        statics = [o for o, vel in obstacles if not np.any(vel)]
        reach = c.v_max * c.safety_horizon + 1.5
        near_static = [o for o in statics if clearance(Circle(cart.x, cart.y, 0.6), o) < reach]
        cands = [(v, w_t) for v in np.linspace(v_hi, v_lo, 6)]
        cands.append((v_lo, min(max(0.0, w_lo), w_hi)))  # brake and stop turning
        # Already inside the margin (someone stands next to us)? Then a move is still fine if it does not get any
        # closer than we are now - otherwise cart and pedestrian can freeze each other forever.
        box = cart.box(self.length, self.width)
        now = min([clearance(box, o) for o in near_static] + [clearance(box, o) for o, _ in movers] + [math.inf])
        need = min(c.safety_margin, max(now - 1e-3, c.hard_clearance))
        best, best_score = None, -math.inf
        for v0, w0 in cands:
            score = self._rollout(cart, v0, w0, near_static, movers)
            if score >= need:
                return v0, w0, score
            if score > best_score + 1e-9:
                best, best_score = (v0, w0), score
        return best[0], best[1], best_score

    def _rollout(self, cart, v0, w0, statics, movers):
        """Min clearance if the cart drives (v0, w0) for one tick, then brakes as hard as it can."""
        c, dt = self.c, self.dt
        x, y, yaw, v, w = cart.x, cart.y, cart.yaw, v0, w0
        worst = math.inf
        steps = max(1, int(round(c.safety_horizon / dt)))
        for k in range(1, steps + 1):
            yaw_m = yaw + 0.5 * w * dt
            x += v * math.cos(yaw_m) * dt
            y += v * math.sin(yaw_m) * dt
            yaw += w * dt
            box = Box(x, y, self.length / 2, self.width / 2, yaw)
            for o in statics:
                worst = min(worst, clearance(box, o))
            t = k * dt
            for o, vel in movers:
                if k == 1:  # where they are now, as well as where they are predicted to be
                    worst = min(worst, clearance(box, o))
                if v > 1e-3 or k == 1:  # once stopped, people walking into us are not the cart's choice
                    # constant-velocity prediction, with a footprint that grows with time (people speed up / turn)
                    p = Circle(o.x + vel[0] * t, o.y + vel[1] * t, o.r + c.predict_uncertainty * t)
                    worst = min(worst, clearance(box, p))
            if v <= 1e-3:
                break
            v = max(0.0, v - c.a_brake * dt)
            w = max(0.0, abs(w) - c.alpha_max * dt) * (1 if w >= 0 else -1)
        return worst
