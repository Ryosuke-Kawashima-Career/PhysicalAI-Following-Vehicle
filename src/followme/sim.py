"""The offline simulation loop: world -> people -> controller -> metrics, one `dt` tick at a time (deterministic)."""
import math

import numpy as np

from .config import Config
from .controller import CartState, FollowController
from .metrics import Metrics
from .people import Traveller, make_pedestrians, step_pedestrians
from .world import load_world


def simulate(cfg: Config = None, world=None, scene_path=None, on_tick=None):
    """Run one episode. Returns dict(summary=..., ticks=[{t, actors: [{id, x, y, yaw}], ...}], world=World).

    on_tick(t, actors) is called after every tick (Step 4 uses it to stream poses to Cyberwave).
    """
    cfg = cfg or Config()
    world = world or (load_world(scene_path) if scene_path else load_world())
    rng = np.random.default_rng(cfg.seed)
    pc = cfg.people
    trav = Traveller(world.route, pc.traveller_speed, pc.traveller_stop_at, pc.traveller_stop_time)
    peds = make_pedestrians(world, trav, cfg, rng)
    a = world.actors["follow_cart"]
    cart = CartState(a["center"][0], a["center"][1], a["yaw"])
    t = 0.0
    p_t, v_t, h_t = trav.state(t, cfg.dt)
    ctrl = FollowController(cfg, world, cart, p_t)
    metrics = Metrics(world, [p.name for p in peds])
    ticks = []

    def snapshot(t, p_t, h_t):
        actors = [{"id": "follow_cart", "x": cart.x, "y": cart.y, "yaw": cart.yaw},
                  {"id": "traveller", "x": float(p_t[0]), "y": float(p_t[1]), "yaw": h_t}]
        for p in peds:
            h = p.heading
            if h is None:  # standing: keep the last heading (or the scene yaw)
                h = getattr(p, "_last_h", world.actors[p.name]["yaw"])
            p._last_h = h
            actors.append({"id": p.name, "x": float(p.p[0]), "y": float(p.p[1]), "yaw": h})
        for x in actors:
            x.update(x=round(x["x"], 4), y=round(x["y"], 4), yaw=round(x["yaw"], 4))
        return actors

    n = int(round(cfg.duration / cfg.dt))
    ticks.append({"t": 0.0, "actors": snapshot(0.0, p_t, h_t), "v": 0.0, "w": 0.0})
    metrics.record(0.0, cart.box(*ctrl.length_width()), p_t, h_t, peds)
    for k in range(1, n + 1):
        people = [(p.circle(), p.v.copy()) for p in peds]
        v, w = ctrl.command(cart, p_t, v_t, people)
        # unicycle integration (midpoint heading)
        yaw_m = cart.yaw + 0.5 * w * cfg.dt
        cart.x += v * math.cos(yaw_m) * cfg.dt
        cart.y += v * math.sin(yaw_m) * cfg.dt
        cart.yaw = (cart.yaw + w * cfg.dt + math.pi) % (2 * math.pi) - math.pi
        cart.v, cart.w = v, w
        # pedestrians react to where the cart is now, and keep out of the space it needs to brake
        d_stop = v * cfg.dt + v * v / (2 * cfg.cart.a_brake)
        envelope = [cart.box(*ctrl.length_width(), x=cart.x + f * d_stop * math.cos(cart.yaw),
                             y=cart.y + f * d_stop * math.sin(cart.yaw)) for f in (0.5, 1.0) if d_stop > 0.05]
        step_pedestrians(peds, p_t, cart.box(*ctrl.length_width()), world, cfg, t, cfg.dt, rng, envelope)
        t = k * cfg.dt
        p_t, v_t, h_t = trav.state(t, cfg.dt)
        actors = snapshot(t, p_t, h_t)
        d = ctrl.debug
        ticks.append({"t": round(t, 3), "actors": actors, "v": round(v, 4), "w": round(w, 4),
                      "gap": round(float(d["gap"]), 4), "holding": d["holding"], "cone": d["cone"],
                      "margin": round(float(d["margin"]), 4)})
        metrics.record(t, cart.box(*ctrl.length_width()), p_t, h_t, peds)
        if on_tick:
            on_tick(t, actors)
    return {"summary": metrics.summary(), "ticks": ticks, "world": world, "metrics": metrics, "config": cfg}
