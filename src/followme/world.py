"""2-D world loaded from out/scene.json: static obstacles as circles / oriented boxes, plus clearance geometry.

The cart is an oriented box (its real 0.93 x 0.62 m footprint), people and pillars are circles, walls / benches / the
luggage pile are boxes. `clearance(a, b)` is the gap between two footprints (negative when they overlap).
"""
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENE = ROOT / "out" / "scene.json"


@dataclass(frozen=True)
class Circle:
    x: float
    y: float
    r: float
    name: str = ""


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    hx: float  # half length along the box's own x axis
    hy: float  # half width
    yaw: float = 0.0  # rad
    name: str = ""

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = []
        for dx, dy in ((self.hx, self.hy), (-self.hx, self.hy), (-self.hx, -self.hy), (self.hx, -self.hy)):
            out.append((self.x + c * dx - s * dy, self.y + s * dx + c * dy))
        return out


# --- geometry ---------------------------------------------------------------------------------------------------
def _point_box(px, py, b: Box):
    """(distance, closest point) from a point to a box (distance 0 and the point itself if inside)."""
    c, s = math.cos(b.yaw), math.sin(b.yaw)
    lx, ly = c * (px - b.x) + s * (py - b.y), -s * (px - b.x) + c * (py - b.y)
    qx, qy = min(max(lx, -b.hx), b.hx), min(max(ly, -b.hy), b.hy)
    wx, wy = b.x + c * qx - s * qy, b.y + s * qx + c * qy
    inside = abs(lx) <= b.hx and abs(ly) <= b.hy
    return (0.0 if inside else math.hypot(px - wx, py - wy)), (wx, wy), inside


def _seg_seg(p1, p2, q1, q2):
    """Distance between two segments (they are known not to cross when this is used)."""
    def pt_seg(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        t = 0.0 if ax == ay == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay)))
        return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def _sat_overlap(a: Box, b: Box):
    """Penetration depth along the separating axes (> 0 means the boxes overlap)."""
    ca, cb = a.corners(), b.corners()
    depth = math.inf
    for box in (a, b):
        for ang in (box.yaw, box.yaw + math.pi / 2):
            ax, ay = math.cos(ang), math.sin(ang)
            pa = [x * ax + y * ay for x, y in ca]
            pb = [x * ax + y * ay for x, y in cb]
            d = min(max(pa), max(pb)) - max(min(pa), min(pb))
            if d <= 0:
                return 0.0
            depth = min(depth, d)
    return depth


def clearance(a, b) -> float:
    """Gap between two footprints in metres (negative = overlap; boxes report -penetration depth)."""
    if isinstance(a, Circle) and isinstance(b, Circle):
        return math.hypot(a.x - b.x, a.y - b.y) - a.r - b.r
    if isinstance(a, Circle):
        a, b = b, a
    if isinstance(b, Circle):  # box vs circle
        d, (wx, wy), inside = _point_box(b.x, b.y, a)
        return (-b.r if inside else d - b.r)
    depth = _sat_overlap(a, b)
    if depth > 0:
        return -depth
    ca, cb = a.corners(), b.corners()
    return min(_seg_seg(ca[i], ca[(i + 1) % 4], cb[j], cb[(j + 1) % 4]) for i in range(4) for j in range(4))


def away_vector(shape, px, py):
    """Unit vector pointing from `shape` towards the point (px, py) (used for repulsion)."""
    if isinstance(shape, Circle):
        dx, dy = px - shape.x, py - shape.y
    else:
        d, (wx, wy), inside = _point_box(px, py, shape)
        dx, dy = (px - shape.x, py - shape.y) if inside or d < 1e-9 else (px - wx, py - wy)
    n = math.hypot(dx, dy)
    return (dx / n, dy / n) if n > 1e-9 else (1.0, 0.0)


# --- scene ------------------------------------------------------------------------------------------------------
@dataclass
class World:
    static: list  # Circle / Box obstacles (walls, pillars, benches, luggage)
    actors: dict  # name -> dict(center=(x, y), yaw=rad, size=(sx, sy, sz), shape=...)
    route: list  # [(x, y)] traveller waypoints
    bounds: tuple  # (xmin, xmax, ymin, ymax) of the floor
    source: str = ""

    def static_clearance(self, shape):
        return min((clearance(shape, o), o.name) for o in self.static)


def load_world(path=DEFAULT_SCENE) -> World:
    scene = json.loads(Path(path).read_text(encoding="utf-8"))
    static, actors, bounds = [], {}, None
    for o in scene["objects"]:
        (cx, cy, _), (sx, sy, sz) = o["center"], o["size"]
        yaw = math.radians(o["yaw_deg"])
        if o["name"] == "floor":
            bounds = (cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2)
        elif o["static"]:
            static.append(Circle(cx, cy, max(sx, sy) / 2, o["name"]) if o["shape"] == "circle"
                          else Box(cx, cy, sx / 2, sy / 2, yaw, o["name"]))
        else:
            actors[o["name"]] = dict(center=(cx, cy), yaw=yaw, size=(sx, sy, sz), shape=o["shape"])
    route = [(w["x"], w["y"]) for w in scene["waypoints"]]
    if bounds is None or not route or "follow_cart" not in actors or "traveller" not in actors:
        raise ValueError(f"{path}: needs a floor, waypoints, follow_cart and traveller")
    return World(static, actors, route, bounds, str(path))


def polyline_point(points: np.ndarray, cum: np.ndarray, s: float):
    """Point at arc length s along a polyline (clamped to its ends)."""
    s = min(max(s, 0.0), cum[-1])
    i = int(np.searchsorted(cum, s, side="right")) - 1
    i = min(max(i, 0), len(points) - 2)
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg <= 1e-12 else (s - cum[i]) / seg
    return points[i] + t * (points[i + 1] - points[i])
