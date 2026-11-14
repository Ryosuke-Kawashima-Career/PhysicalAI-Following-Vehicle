"""People: the scripted traveller (ground truth) and the pedestrians (social-force agents).

- traveller: walks the waypoint route at a constant speed, with one timed stop (check-in), then stays at the end.
  It does not react to anyone (it is the target; pedestrians avoid it).
- crossers (ped_1, ped_2): wait at their start, then walk straight across the route so that they reach the crossing
  line just after the traveller has passed it - i.e. they cut between the traveller and the cart (requirement R2).
- walkers (ped_3, ped_4): seeded random walk between random goals in their half of the concourse, with pauses.

Pedestrians see the cart and keep out of its way (they never step closer than `cart_personal_space`). The cart's own
safety does not rely on this: it never moves closer than its margin either (see controller.py).
"""
import math

import numpy as np

from .world import Box, Circle, away_vector, clearance, polyline_point


class Traveller:
    def __init__(self, route, speed, stop_at, stop_time):
        self.points = np.asarray(route, dtype=float)
        seg = np.linalg.norm(np.diff(self.points, axis=0), axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(seg)])
        self.speed = speed
        self.stop_s = self.cum[stop_at]
        self.stop_t0 = self.stop_s / speed  # time the stop starts
        self.stop_time = stop_time
        self.radius = 0.3
        self.name = "traveller"

    def arc_length(self, t):
        if t <= self.stop_t0:
            return t * self.speed
        if t <= self.stop_t0 + self.stop_time:
            return self.stop_s
        return min(self.cum[-1], self.stop_s + (t - self.stop_t0 - self.stop_time) * self.speed)

    def state(self, t, dt=0.1):
        """(position, velocity, heading) at time t. Heading is the walking direction (kept while standing)."""
        s = self.arc_length(t)
        p = polyline_point(self.points, self.cum, s)
        s_prev = self.arc_length(max(0.0, t - dt))
        v = (p - polyline_point(self.points, self.cum, s_prev)) / dt if t > 0 else np.zeros(2)
        i = int(np.clip(np.searchsorted(self.cum, max(s - 1e-6, 0.0), side="right") - 1, 0, len(self.points) - 2))
        d = self.points[i + 1] - self.points[i]
        return p, v, math.atan2(d[1], d[0])

    def time_at_x(self, x, dt=0.01):
        """First time the traveller's x coordinate reaches x (used to time the crossers)."""
        t, end = 0.0, self.stop_t0 + self.stop_time + self.cum[-1] / self.speed
        while t <= end:
            if self.state(t)[0][0] >= x:
                return t
            t += dt
        return None

    def done(self, t):
        return self.arc_length(t) >= self.cum[-1] - 1e-9


class Pedestrian:
    def __init__(self, name, start, radius, speed, mode, goal=None, start_time=0.0, region=None):
        self.name = name
        self.p = np.asarray(start, dtype=float)
        self.v = np.zeros(2)
        self.radius = radius
        self.speed = speed
        self.mode = mode  # "cross" | "walk"
        self.goal = None if goal is None else np.asarray(goal, dtype=float)
        self.start_time = start_time
        self.region = region  # (xmin, xmax, ymin, ymax) for random walkers
        self.pause_until = 0.0
        self.done = False
        self.anchor = None  # (position, time) - for detecting being blocked
        self.detour = None  # temporary waypoint around the cart after being blocked

    @property
    def heading(self):
        return math.atan2(self.v[1], self.v[0]) if np.hypot(*self.v) > 0.05 else None

    def circle(self, p=None):
        p = self.p if p is None else p
        return Circle(float(p[0]), float(p[1]), self.radius, self.name)


def make_pedestrians(world, traveller, cfg, rng):
    pc = cfg.people
    peds = []
    for name, goal, x_line, delay in pc.crossers:
        start = np.asarray(world.actors[name]["center"], dtype=float)
        # where the crosser meets the traveller's route, and when the traveller passes that point
        t_pass = traveller.time_at_x(x_line)
        cross_pt = traveller.state(t_pass)[0]
        walk = np.linalg.norm(cross_pt - start)
        t0 = max(0.0, t_pass + delay - walk / pc.crosser_speed - pc.tau)
        peds.append(Pedestrian(name, start, pc.radius, pc.crosser_speed, "cross", goal, t0))
    xmin, xmax, ymin, ymax = world.bounds
    halves = {"ped_3": (-2.0, xmax - 1.0, 0.5, ymax - 0.8), "ped_4": (-2.0, xmax - 1.0, ymin + 0.8, -0.5)}
    for name in pc.walkers:
        start = world.actors[name]["center"]
        region = halves.get(name, (xmin + 1, xmax - 1, ymin + 0.8, ymax - 0.8))
        p = Pedestrian(name, start, pc.radius, float(rng.uniform(*pc.walker_speed)), "walk", region=region)
        p.goal = _random_goal(world, region, rng, pc.radius)
        peds.append(p)
    return peds


def _random_goal(world, region, rng, radius):
    for _ in range(200):
        g = np.array([rng.uniform(region[0], region[1]), rng.uniform(region[2], region[3])])
        if world.static_clearance(Circle(g[0], g[1], radius))[0] > 0.5:
            return g
    return np.array([(region[0] + region[1]) / 2, (region[2] + region[3]) / 2])


def _detour_point(ped, cart_box, world):
    """A side-step waypoint ~1.3 m beside the cart, on the side of the person away from the cart's centre."""
    to_goal = ped.goal - ped.p
    d = to_goal / max(np.linalg.norm(to_goal), 1e-9)
    lat = np.array([-d[1], d[0]])
    away = ped.p - np.array([cart_box.x, cart_box.y])
    side = lat if lat @ away >= 0 else -lat
    for k in (1.3, 1.0, 0.7):
        cand = ped.p + k * side + 0.4 * d
        if world.static_clearance(Circle(cand[0], cand[1], ped.radius))[0] > 0.2:
            return cand
    return None


def step_pedestrians(peds, traveller_pos, cart_box: Box, world, cfg, t, dt, rng, envelope=()):
    """Advance all pedestrians by dt (social force + hard personal-space limits).

    cart_box is the cart's pose after its move this tick; envelope = extra boxes along its braking path
    (people don't step into the space a moving vehicle needs to stop).
    """
    pc = cfg.people
    others = [(p.name, p.p, p.radius) for p in peds] + [("traveller", np.asarray(traveller_pos), 0.3)]
    new_v = {}
    for ped in peds:
        desired = np.zeros(2)
        if ped.mode == "cross":
            active = t >= ped.start_time and not ped.done
        else:
            active = t >= ped.pause_until
        if ped.mode == "walk" and ped.goal is not None and clearance(Circle(*ped.goal, ped.radius), cart_box) < 0.8:
            ped.goal = _random_goal(world, ped.region, rng, ped.radius)  # nobody walks to where the cart stands
        if active and ped.detour is not None:  # walking around the cart first
            if np.linalg.norm(ped.detour - ped.p) < 0.3:
                ped.detour = None
        if active and ped.goal is not None:
            target = ped.goal if ped.detour is None else ped.detour
            to_goal = target - ped.p
            dist = np.linalg.norm(to_goal)
            if dist < 0.3 and ped.detour is None:
                if ped.mode == "cross":
                    ped.done = True
                else:
                    ped.pause_until = t + float(rng.uniform(*pc.walker_pause))
                    ped.goal = _random_goal(world, ped.region, rng, ped.radius)
                    ped.speed = float(rng.uniform(*pc.walker_speed))
            elif dist >= 0.3:
                desired = ped.speed * to_goal / dist
        # blocked (wants to walk, but covered < 0.3 m in `blocked_patience` s - jittering counts as blocked)
        # -> go around instead of standing against the cart
        if not np.any(desired):
            ped.anchor = None
        elif ped.anchor is None or np.linalg.norm(ped.p - ped.anchor[0]) > 0.3:
            ped.anchor = (ped.p.copy(), t)
        if ped.anchor is not None and t - ped.anchor[1] > pc.blocked_patience:
            ped.anchor = None
            ped.detour = _detour_point(ped, cart_box, world)
            if ped.detour is None and ped.mode == "walk":
                ped.goal = _random_goal(world, ped.region, rng, ped.radius)
        force = (desired - ped.v) / pc.tau
        me = ped.circle()
        for name, q, r in others:  # people
            if name == ped.name:
                continue
            c = math.hypot(*(ped.p - q)) - ped.radius - r
            if c < 2.0:
                n = (ped.p - q) / max(np.linalg.norm(ped.p - q), 1e-9)
                force += pc.a_people * math.exp(-max(c, 0.0) / pc.b_people) * n
        for o in world.static:  # walls, pillars, benches
            c = clearance(me, o)
            if c < 1.0:
                force += pc.a_static * math.exp(-max(c, 0.0) / pc.b_static) * np.array(away_vector(o, *ped.p))
        c_cart = clearance(me, cart_box)  # the cart
        if c_cart < 2.0:
            n = np.array(away_vector(cart_box, *ped.p))
            fwd = np.array([math.cos(cart_box.yaw), math.sin(cart_box.yaw)])
            if n @ fwd > 0.5 and np.any(desired):
                # in front of the cart while walking: step sideways out of its path (the side we are heading to),
                # rather than being pushed ahead of it
                lat = np.array([-fwd[1], fwd[0]])
                n = lat if lat @ desired >= 0 else -lat
            force += pc.a_cart * math.exp(-max(c_cart, 0.0) / pc.b_cart) * n
        v = ped.v + force * dt
        vmax = 1.3 * max(ped.speed, 0.5)
        sp = np.linalg.norm(v)
        if sp > vmax:
            v *= vmax / sp
        if not active and np.linalg.norm(v) < 0.05:
            v = np.zeros(2)
        new_v[ped.name] = _limit_step(ped, v, [cart_box, *envelope], world, pc, dt)
    for ped in peds:
        ped.v = new_v[ped.name]
        ped.p = ped.p + ped.v * dt


def _limit_step(ped, v, cart_boxes, world, pc, dt):
    """Hard limits: never step closer than personal space to the cart (or its braking envelope), never step into a
    static obstacle. Moving away / along is always allowed."""
    for _ in range(4):
        nxt = ped.circle(ped.p + v * dt)
        bad = None
        for cb in cart_boxes:
            c_now, c_next = clearance(ped.circle(), cb), clearance(nxt, cb)
            if c_next < pc.cart_personal_space and c_next < c_now:
                bad = cb
                break
        if bad is None:
            for o in world.static:
                if clearance(nxt, o) < 0.02 and clearance(nxt, o) < clearance(ped.circle(), o):
                    bad = o
                    break
        if bad is None:
            return v
        n = np.array(away_vector(bad, *ped.p))
        toward = float(v @ n)
        if toward < 0:
            v = v - toward * n  # drop the component towards the obstacle, keep sliding along it
        else:
            v = v * 0.5
    return np.zeros(2)
