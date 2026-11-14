"""Step 3 exit check (plan): seed 42, 60 s -> 0 collisions, >= 85 % in band, max distance < 4 m.

Plus: geometry sanity, determinism, actuator limits, and safety (R4) over several other seeds.
Run: .venv/Scripts/python.exe -m pytest -q
"""
import math
from dataclasses import replace

import numpy as np
import pytest

from followme import config as C
from followme.config import Config
from followme.sim import simulate
from followme.world import DEFAULT_SCENE, Box, Circle, clearance, load_world

pytestmark = pytest.mark.skipif(not DEFAULT_SCENE.exists(), reason="out/scene.json missing - run scripts/export_scene.py")


@pytest.fixture(scope="module")
def run42():
    return simulate(replace(Config(), seed=42, duration=60.0))


# --- the plan's exit criteria -------------------------------------------------------------------------------------
def test_exit_no_collisions(run42):
    s = run42["summary"]
    assert s["collision_ticks"] == 0, f"first collision at t={s['first_collision_t']}"
    assert s["min_clearance"] >= C.MIN_CLEARANCE


def test_exit_band(run42):
    assert run42["summary"]["band_fraction"] >= C.BAND_MIN_FRACTION


def test_exit_max_distance(run42):
    assert run42["summary"]["max_distance"] < C.MAX_DISTANCE


def test_r2_pedestrians_move_and_cut_between(run42):
    s = run42["summary"]
    assert len(s["moving_pedestrians"]) >= 2
    assert {"ped_1", "ped_2"} <= set(s["crossed_between"])


def test_run_covers_60_s(run42):
    assert run42["summary"]["ticks"] == 601
    assert run42["ticks"][-1]["t"] == pytest.approx(60.0)


# --- safety must hold for every seed, not only 42 ----------------------------------------------------------------
@pytest.mark.parametrize("seed", [0, 3, 7, 11, 16])
def test_no_collisions_other_seeds(seed):
    s = simulate(replace(Config(), seed=seed))["summary"]
    assert s["collision_ticks"] == 0, f"seed {seed}: min clearance {s['min_clearance']:.3f} with {s['min_clearance_with']}"
    assert s["max_distance"] < C.MAX_DISTANCE + 1.0  # never runs away / gets stuck far behind


# --- behaviour ---------------------------------------------------------------------------------------------------
def test_deterministic():
    a = simulate(replace(Config(), seed=5, duration=15.0))["ticks"]
    b = simulate(replace(Config(), seed=5, duration=15.0))["ticks"]
    assert a == b


def test_actuator_limits(run42):
    cc = Config().cart
    t = run42["ticks"]
    v = np.array([x["v"] for x in t])
    w = np.array([x["w"] for x in t])
    dt = Config().dt
    assert v.min() >= 0.0 and v.max() <= cc.v_max + 1e-6
    assert np.abs(w).max() <= cc.w_max + 1e-6
    dv = np.diff(v) / dt
    assert dv.max() <= cc.a_acc + 1e-3 and dv.min() >= -cc.a_brake - 1e-3


def test_cart_holds_while_traveller_waits(run42):
    """At the gate (end of the route) the traveller stands; the cart must stop behind them, inside the band."""
    t = run42["ticks"]
    end = [x for x in t if x["t"] >= 45.0]
    assert all(x["v"] == 0.0 for x in end)
    d = run42["metrics"].dist[-1]
    assert C.BAND[0] <= d <= C.BAND[1]


def test_cart_stays_on_floor(run42):
    w = run42["world"]
    xs = [x["actors"][0]["x"] for x in run42["ticks"]]
    ys = [x["actors"][0]["y"] for x in run42["ticks"]]
    assert w.bounds[0] < min(xs) and max(xs) < w.bounds[1]
    assert w.bounds[2] < min(ys) and max(ys) < w.bounds[3]


# --- world / geometry --------------------------------------------------------------------------------------------
def test_world_loads_scene():
    w = load_world()
    names = {o.name for o in w.static}
    assert {"wall_north", "wall_south", "wall_east", "pillar_1", "pillar_2", "pillar_3",
            "bench_1", "bench_2", "luggage_pile"} <= names
    assert {"follow_cart", "traveller", "ped_1", "ped_2", "ped_3", "ped_4"} <= set(w.actors)
    assert len(w.route) == 8


def test_clearance_geometry():
    assert clearance(Circle(0, 0, 1), Circle(3, 0, 1)) == pytest.approx(1.0)
    box = Box(0, 0, 1, 0.5)
    assert clearance(box, Circle(2.5, 0, 0.5)) == pytest.approx(1.0)
    assert clearance(box, Circle(0, 0, 0.1)) < 0  # inside
    assert clearance(box, Box(3, 0, 1, 0.5)) == pytest.approx(1.0)
    assert clearance(box, Box(1.5, 0, 1, 0.5)) < 0  # overlap
    rot = Box(0, 0, 1, 0.5, math.pi / 2)  # rotated: now 1.0 wide in x
    assert clearance(rot, Circle(1.5, 0, 0.5)) == pytest.approx(0.5)
    # corner-to-corner distance between two separated boxes
    assert clearance(Box(0, 0, 0.5, 0.5), Box(2, 2, 0.5, 0.5)) == pytest.approx(math.hypot(1, 1))
