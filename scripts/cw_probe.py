"""Step 1 probe: connect with .env and list workspace / environments / twins (read-only)."""
import os, json
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from dotenv import load_dotenv
load_dotenv()
from cw_common import WORKSPACE_UUID
from cyberwave import Cyberwave

cw = Cyberwave(workspace_id=WORKSPACE_UUID)
print("base_url:", cw.config.base_url)
for ws in cw.workspaces.list():
    print("WS", getattr(ws, "slug", None), ws.uuid, ws.name)
envs = cw.environments.list()
for e in envs:
    print("ENV", getattr(e, "slug", None), e.uuid, e.name)
target = cw.environments.get_by_slug("ryosukes-workspace/envs/default-environment")
print("TARGET", target and (target.uuid, target.name, getattr(target, "slug", None)))
if target:
    for t in cw.twins.list(environment_id=target.uuid):
        d = t.to_dict() if hasattr(t, "to_dict") else t.__dict__
        keep = {k: d.get(k) for k in ("uuid", "slug", "name", "asset_uuid", "position_x", "position_y", "position_z",
                                        "rotation_x", "rotation_y", "rotation_z", "rotation_w")}
        print("TWIN", json.dumps(keep, default=str))
