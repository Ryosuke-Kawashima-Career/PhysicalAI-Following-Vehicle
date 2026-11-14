"""Step 1 extra: stream poses to the test twin over MQTT at 10 Hz (the path Step 4 needs; REST is ~0.6 s/call).

Drives `step1_test_cylinder` around a 2 m circle for DURATION seconds, then reads the REST state back to see
whether the backend persisted the MQTT updates.
Usage: .venv/Scripts/python.exe scripts/cw_mqtt_test.py [duration_s]
"""
import math
import sys
import time

from cw_common import ENV_UUID, connect

NAME = "step1_test_cylinder"
HZ = 10.0
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
R = 2.0

cw = connect()
twin_data = next(t for t in cw.twins.list(environment_id=ENV_UUID) if t.name == NAME)
uuid = twin_data.uuid
before = cw.twins.get_raw(uuid)
print(f"before (REST): ({before.position_x:+.2f},{before.position_y:+.2f})")

mqtt = cw.mqtt
mqtt.connect()
t_end = time.time() + 5
while not mqtt.connected and time.time() < t_end:
    time.sleep(0.1)
print("mqtt connected:", mqtt.connected, "| source_type:", cw.config.source_type)

n = int(DURATION * HZ)
t0 = time.perf_counter()
for i in range(n + 1):
    th = 2 * math.pi * i / n
    x, y, yaw = R * math.cos(th), R * math.sin(th), th + math.pi / 2
    mqtt.update_twin_position(uuid, {"x": x, "y": y, "z": 0.5})
    mqtt.update_twin_rotation(uuid, {"x": 0.0, "y": 0.0, "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)})
    time.sleep(max(0.0, t0 + (i + 1) / HZ - time.perf_counter()))
print(f"published {n + 1} pose pairs in {time.perf_counter() - t0:.1f} s (last pose: ({x:+.2f},{y:+.2f}))")

time.sleep(2.0)
after = cw.twins.get_raw(uuid)
print(f"after  (REST): ({after.position_x:+.2f},{after.position_y:+.2f})")
cw.disconnect()
