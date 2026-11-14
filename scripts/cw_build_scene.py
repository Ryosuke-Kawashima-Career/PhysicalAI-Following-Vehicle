"""Step 2: build / re-apply the airport concourse in the Cyberwave environment from scene_spec.py.

Idempotent: twins are matched by name. Missing twins are created; existing ones get pose, scale and metadata updates.
The traveller route is written as environment waypoints (collection `traveller_route`), replacing any old ones.

Usage: .venv/Scripts/python.exe scripts/cw_build_scene.py [--dry-run] [--only name1,name2] [--no-waypoints]
"""
import math
import sys

from cw_common import ENV_UUID, connect
from scene_spec import ADOPT, ASSETS, GEOMETRY, OBJECTS, PRIMITIVES, WAYPOINT_COLLECTION, WAYPOINTS

DRY = "--dry-run" in sys.argv
ONLY = None
if "--only" in sys.argv:
    ONLY = set(sys.argv[sys.argv.index("--only") + 1].split(","))


def quat_z(yaw_deg):
    h = math.radians(yaw_deg) / 2
    return {"rotation_x": 0.0, "rotation_y": 0.0, "rotation_z": math.sin(h), "rotation_w": math.cos(h)}


def twin_transform(o):
    """Spec (real centre + size) -> twin (position, scale), using the measured GEOMETRY."""
    g = GEOMETRY[o["asset"]]
    if o["asset"] in PRIMITIVES:
        scale = tuple(s / n for s, n in zip(o["size"], g["native"]))
        off = [c * k for c, k in zip(g["center"], scale)]  # mesh centre offset in the twin's frame
        yaw = math.radians(o["yaw"])
        ox, oy = off[0] * math.cos(yaw) - off[1] * math.sin(yaw), off[0] * math.sin(yaw) + off[1] * math.cos(yaw)
        pos = (o["pos"][0] - ox, o["pos"][1] - oy, o["pos"][2] - off[2])
    else:
        scale = (1.0, 1.0, 1.0)
        cx, cy = g.get("center_xy", (0.0, 0.0))
        yaw = math.radians(o["yaw"])
        ox, oy = cx * math.cos(yaw) - cy * math.sin(yaw), cx * math.sin(yaw) + cy * math.cos(yaw)
        pos = (o["pos"][0] - ox, o["pos"][1] - oy, o["pos"][2] + g["origin_z"])
    return pos, scale


def fields(o):
    pos, scale = twin_transform(o)
    pose = {"position_x": pos[0], "position_y": pos[1], "position_z": pos[2], **quat_z(o["yaw"])}
    meta = {"followme": {k: o[k] for k in ("group", "shape", "size", "static")}}
    meta["followme"]["size"] = list(o["size"])
    return pose, dict(scale_x=scale[0], scale_y=scale[1], scale_z=scale[2]), meta


cw = connect()
twins = {t.name: t for t in cw.twins.list(environment_id=ENV_UUID)}

for old, new in ADOPT.items():
    if old in twins and new not in twins and (ONLY is None or new in ONLY):
        print(f"adopt  {old} -> {new}")
        if not DRY:
            twins[new] = cw.twins.update(twins.pop(old).uuid, name=new)

for o in OBJECTS:
    if ONLY is not None and o["name"] not in ONLY:
        continue
    pose, scale, meta = fields(o)
    t = twins.get(o["name"])
    if t is None:
        print(f"create {o['name']:<13} {o['asset']:<8} pos={o['pos']} size={o['size']} yaw={o['yaw']}")
        if not DRY:
            t = cw.twins.create(asset_id=ASSETS[o["asset"]], environment_id=ENV_UUID, name=o["name"], **pose, **scale)
            # create() ignores metadata / fixed_base in SDK 0.7.5 (seen 2026-11-14); update() applies them.
            cw.twins.update(t.uuid, metadata=dict(t.metadata or {}, **meta), fixed_base=o["static"], **scale)
        continue
    if t.asset_uuid != ASSETS[o["asset"]]:
        print(f"WARN   {o['name']}: existing twin uses asset {t.asset_uuid}, spec wants {o['asset']}. Delete it to rebuild.")
    cur_scale = (t.scale_x, t.scale_y, t.scale_z)
    need_meta = (t.metadata or {}).get("followme") != meta["followme"] or any(
        abs(a - b) > 1e-6 for a, b in zip(cur_scale, scale.values()))
    print(f"update {o['name']:<13} pose{' + scale/meta' if need_meta else ''}")
    if not DRY:
        if need_meta:
            md = dict(t.metadata or {}, **meta)
            cw.twins.update(t.uuid, metadata=md, fixed_base=o["static"], **scale)
        cw.twins.update_state(t.uuid, pose)

if "--no-waypoints" not in sys.argv and ONLY is None:
    old = [w for w in cw.environments.get_waypoints(ENV_UUID) if getattr(w, "collection", None) == WAYPOINT_COLLECTION]
    print(f"waypoints: replace {len(old)} old with {len(WAYPOINTS)} in collection '{WAYPOINT_COLLECTION}'")
    if not DRY:
        for w in old:
            cw.environments.delete_waypoint(ENV_UUID, w.id)
        for i, (x, y) in enumerate(WAYPOINTS, 1):
            cw.environments.create_waypoint(ENV_UUID, position={"x": x, "y": y, "z": 0.0}, name=f"wp_{i}",
                                            collection=WAYPOINT_COLLECTION, metadata={"followme": {"order": i}})
print("dry run, nothing changed" if DRY else "done")
