"""Step 2.4: export the environment's follow-me layout to out/scene.json (what the local world loads).

Reads the twins back from Cyberwave (not from scene_spec.py), undoes the measured primitive offsets, and writes real
centres / sizes. Only twins that carry `metadata.followme` are exported (other twins, e.g. `playground_tello`, are ignored).

--check also runs the layout checks that stand in for the MCP's cw_analyze_environment_layout:
  spec match, objects inside the concourse, nothing floating or sunk, no footprint overlaps, route clearance.

Usage: .venv/Scripts/python.exe scripts/export_scene.py [--check] [--out out/scene.json]
"""
import datetime as dt
import json
import math
import os
import sys

from cw_common import ENV_SLUG, ENV_UUID, connect
from scene_spec import ASSETS, GEOMETRY, OBJECTS, PRIMITIVES, WAYPOINT_COLLECTION, WAYPOINTS

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT = sys.argv[sys.argv.index("--out") + 1] if "--out" in sys.argv else os.path.join(ROOT, "out", "scene.json")
ASSET_KEY = {v: k for k, v in ASSETS.items()}
CONCOURSE = (15.0, 6.0)  # half extents of the floor
CART_RADIUS = 0.56  # half-diagonal of the 0.93 x 0.62 cart footprint
TOL = 1e-3


def yaw_of(t):
    return math.degrees(2 * math.atan2(t.rotation_z, t.rotation_w))


def real_geometry(t, key, meta):
    """Twin (position, scale) -> real (centre, size). Inverse of cw_build_scene.twin_transform."""
    g = GEOMETRY[key]
    yaw = math.radians(yaw_of(t))
    if key in PRIMITIVES:
        scale = (t.scale_x, t.scale_y, t.scale_z)
        size = [s * n for s, n in zip(scale, g["native"])]
        off = [c * k for c, k in zip(g["center"], scale)]
        ox, oy = off[0] * math.cos(yaw) - off[1] * math.sin(yaw), off[0] * math.sin(yaw) + off[1] * math.cos(yaw)
        centre = [t.position_x + ox, t.position_y + oy, t.position_z + off[2]]
    else:
        size = list(meta["size"])
        cx, cy = g.get("center_xy", (0.0, 0.0))
        ox, oy = cx * math.cos(yaw) - cy * math.sin(yaw), cx * math.sin(yaw) + cy * math.cos(yaw)
        centre = [t.position_x + ox, t.position_y + oy, t.position_z - g["origin_z"] + size[2] / 2]
    return [round(v, 4) for v in centre], [round(v, 4) for v in size]


def export(cw):
    objs = []
    for t in cw.twins.list(environment_id=ENV_UUID):
        meta = (t.metadata or {}).get("followme")
        if not meta:
            continue
        key = ASSET_KEY.get(t.asset_uuid, t.asset_uuid)
        centre, size = real_geometry(t, key, meta)
        objs.append(dict(name=t.name, uuid=t.uuid, asset=key, group=meta["group"], shape=meta["shape"],
                         static=meta["static"], center=centre, size=size, yaw_deg=round(yaw_of(t), 3)))
    order = {o["name"]: i for i, o in enumerate(OBJECTS)}
    objs.sort(key=lambda o: order.get(o["name"], 999))
    wps = sorted((w for w in cw.environments.get_waypoints(ENV_UUID) if getattr(w, "collection", None) == WAYPOINT_COLLECTION),
                 key=lambda w: (w.metadata or {}).get("followme", {}).get("order", 0))
    return {
        "environment": {"slug": ENV_SLUG, "uuid": ENV_UUID},
        "exported_at": dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds"),
        "frame": "Z-up, metres; center = geometric centre, size = full extent (x, y, z); floor top at z = 0",
        "objects": objs,
        "waypoints": [{"name": w.name, "x": w.position.x, "y": w.position.y} for w in wps],
    }


# ---- checks -----------------------------------------------------------------------------------------------------
def footprint(o):
    """Axis-aligned 2-D footprint. Boxes are rotated by yaw (as their AABB); circles use radius = max(size_x, size_y)/2."""
    cx, cy = o["center"][:2]
    if o["shape"] == "circle":
        r = max(o["size"][0], o["size"][1]) / 2
        return ("circle", cx, cy, r)
    a = math.radians(o["yaw_deg"])
    hx = abs(math.cos(a)) * o["size"][0] / 2 + abs(math.sin(a)) * o["size"][1] / 2
    hy = abs(math.sin(a)) * o["size"][0] / 2 + abs(math.cos(a)) * o["size"][1] / 2
    return ("box", cx, cy, hx, hy)


def point_dist(fp, px, py):
    """Distance from a point to a footprint's boundary (0 if inside)."""
    if fp[0] == "circle":
        return max(0.0, math.hypot(px - fp[1], py - fp[2]) - fp[3])
    dx = max(abs(px - fp[1]) - fp[3], 0.0)
    dy = max(abs(py - fp[2]) - fp[4], 0.0)
    return math.hypot(dx, dy)


def gap(a, b):
    """Clearance between two footprints (negative = overlap; approximate for circle-box)."""
    if a[0] == "circle" and b[0] == "circle":
        return math.hypot(a[1] - b[1], a[2] - b[2]) - a[3] - b[3]
    if a[0] == "box" and b[0] == "box":
        dx = abs(a[1] - b[1]) - a[3] - b[3]
        dy = abs(a[2] - b[2]) - a[4] - b[4]
        return max(dx, dy) if dx < 0 or dy < 0 else math.hypot(dx, dy)
    c, bx = (a, b) if a[0] == "circle" else (b, a)
    return point_dist(bx, c[1], c[2]) - c[3]


def seg_dist(fp, p, q, n=200):
    return min(point_dist(fp, p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n) for i in range(n + 1))


def check(scene):
    errs, warns, info = [], [], []
    by = {o["name"]: o for o in scene["objects"]}
    # 1. spec match
    for s in OBJECTS:
        o = by.get(s["name"])
        if o is None:
            errs.append(f"missing in environment: {s['name']}")
            continue
        want_c = [s["pos"][0], s["pos"][1], s["pos"][2] if s["asset"] in PRIMITIVES else s["size"][2] / 2]
        dc = max(abs(a - b) for a, b in zip(o["center"], want_c))
        ds = max(abs(a - b) for a, b in zip(o["size"], s["size"]))
        dy = abs((o["yaw_deg"] - s["yaw"] + 180) % 360 - 180)
        if dc > TOL or ds > TOL or dy > 0.01:
            errs.append(f"{s['name']}: differs from spec (centre {dc:.4f} m, size {ds:.4f} m, yaw {dy:.3f} deg)")
    extra = set(by) - {s["name"] for s in OBJECTS}
    if extra:
        warns.append(f"followme twins not in spec: {sorted(extra)}")
    if [(w["x"], w["y"]) for w in scene["waypoints"]] != [tuple(map(float, w)) for w in WAYPOINTS]:
        errs.append(f"waypoints differ from spec: {scene['waypoints']}")
    # 2. bounds + floating / sunk (floor top is z = 0)
    for o in scene["objects"]:
        if o["name"] == "floor":
            top = o["center"][2] + o["size"][2] / 2
            if abs(top) > TOL:
                errs.append(f"floor top at z = {top:.3f}, expected 0")
            continue
        bottom = o["center"][2] - o["size"][2] / 2
        if abs(bottom) > 0.01:
            errs.append(f"{o['name']}: bottom at z = {bottom:.3f} ({'floating' if bottom > 0 else 'sunk'})")
        if o["group"] != "concourse":
            fp = footprint(o)
            ext = (fp[3], fp[3]) if fp[0] == "circle" else (fp[3], fp[4])
            if abs(fp[1]) + ext[0] > CONCOURSE[0] or abs(fp[2]) + ext[1] > CONCOURSE[1]:
                errs.append(f"{o['name']}: outside the concourse floor")
    # 3. overlaps between obstacles / actors (and against walls)
    solid = [o for o in scene["objects"] if o["name"] != "floor"]
    for i, a in enumerate(solid):
        for b in solid[i + 1:]:
            if a["group"] == b["group"] == "concourse":
                continue  # walls touch at the corners by design
            g = gap(footprint(a), footprint(b))
            if g < 0:
                errs.append(f"overlap: {a['name']} / {b['name']} ({g:.2f} m)")
            elif g < 0.5 and "actors" in (a["group"], b["group"]):
                warns.append(f"tight: {a['name']} / {b['name']} only {g:.2f} m apart")
    # 4. route clearance for the cart (pedestrians excluded: they move)
    route = [(w["x"], w["y"]) for w in scene["waypoints"]]
    statics = [o for o in solid if o["static"]]
    worst = min(((seg_dist(footprint(o), p, q), o["name"]) for o in statics for p, q in zip(route, route[1:])))
    msg = f"route min clearance to static geometry: {worst[0]:.2f} m ({worst[1]}); cart needs > {CART_RADIUS:.2f} m"
    (info if worst[0] > CART_RADIUS + 0.3 else errs if worst[0] <= CART_RADIUS else warns).append(msg)
    return errs, warns, info


if __name__ == "__main__":
    scene = export(connect())
    os.makedirs(os.path.dirname(os.path.abspath(OUT)), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(scene, f, indent=2)
    print(f"wrote {os.path.abspath(OUT)}: {len(scene['objects'])} objects, {len(scene['waypoints'])} waypoints")
    if "--check" in sys.argv:
        errs, warns, info = check(scene)
        for m in info:
            print("  ok   ", m)
        for m in warns:
            print("  WARN ", m)
        for m in errs:
            print("  ERROR", m)
        print("layout check:", "PASS" if not errs else f"FAIL ({len(errs)} errors)")
        sys.exit(1 if errs else 0)
