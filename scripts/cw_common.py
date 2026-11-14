"""Shared Cyberwave connection for the step scripts.

The SDK appends ``?workspace_uuid=<CYBERWAVE_WORKSPACE_ID>`` to every REST call. A wrong value there
once made twin creation return 404 and ``environments.list()`` come back empty (fixed in ``.env`` on 2026-11-14).
We still pass the resolved workspace UUID explicitly, as a guard.
"""
from dotenv import load_dotenv

load_dotenv()

from cyberwave import Cyberwave  # noqa: E402

WORKSPACE_SLUG = "ryosukes-workspace"
WORKSPACE_UUID = "309a6e78-f697-4571-afa2-c92c2aa8aa2a"
ENV_SLUG = "ryosukes-workspace/envs/default-environment"
ENV_UUID = "fe5a3155-fb05-4c52-8eb4-549dcbdfac28"


def connect() -> Cyberwave:
    return Cyberwave(workspace_id=WORKSPACE_UUID)
