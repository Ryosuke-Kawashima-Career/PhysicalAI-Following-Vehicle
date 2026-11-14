"""Step 1 exit check: get-or-create a test twin and move it to 3 poses via REST (edit_position/edit_rotation).

Usage: .venv/Scripts/python.exe scripts/cw_move_test.py [pause_seconds] [--single]
  --single  set position + yaw in ONE REST request via cw.twins.update_state (instead of edit_position + edit_rotation)
"""
import math
import sys
import time

from cw_common import ENV_SLUG, connect

NAME = "step1_test_cylinder"
POSES = [(0.0, 0.0, 0.0), (2.0, 1.0, 90.0), (-2.0, -1.0, 180.0)]  # x, y (m), yaw (deg)
ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
SINGLE = "--single" in sys.argv
PAUSE = float(ARGS[0]) if ARGS else 3.0

cw = connect()
env = cw.environments.get_by_slug(ENV_SLUG)

# cw.twin("cyberwave/cylinder") fails in SDK 0.7.5 (registry lookup returns nothing for this asset),
# so resolve the primitive from list_primitives() and create by asset UUID.
cyl = next(a for a in cw.assets.list_primitives() if a.registry_id == "cyberwave/cylinder")
existing = [t for t in cw.twins.list(environment_id=env.uuid) if t.name == NAME]
data = existing[0] if existing else cw.twins.create(asset_id=cyl.uuid, environment_id=env.uuid, name=NAME)
twin = cw.twin(twin_id=data.uuid)
print("twin:", twin.uuid, NAME, "created" if not existing else "reused", "in env", env.uuid)

for x, y, yaw in POSES:
    t0 = time.perf_counter()
    if SINGLE:
        h = math.radians(yaw) / 2
        cw.twins.update_state(twin.uuid, {
            "position_x": x, "position_y": y, "position_z": 0.5,
            "rotation_x": 0.0, "rotation_y": 0.0, "rotation_z": math.sin(h), "rotation_w": math.cos(h),
        })
    else:
        twin.edit_position(x=x, y=y, z=0.5)
        twin.edit_rotation(yaw=yaw)
    dt = time.perf_counter() - t0
    raw = cw.twins.get_raw(twin.uuid)
    yaw_back = math.degrees(2 * math.atan2(raw.rotation_z, raw.rotation_w))
    print(
        f"set ({x:+.1f},{y:+.1f}, yaw {yaw:5.1f}) -> server ({raw.position_x:+.2f},{raw.position_y:+.2f},"
        f"{raw.position_z:.2f}, yaw {yaw_back:6.1f})  {'1 REST call' if SINGLE else '2 REST calls'}: {dt * 1000:.0f} ms",
        flush=True,
    )
    time.sleep(PAUSE)
