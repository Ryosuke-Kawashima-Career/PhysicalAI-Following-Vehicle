"""Render the environment server-side (isometric PNG) and save it. Stand-in for the MCP's cw_render_environment_preview.

Usage: .venv/Scripts/python.exe scripts/cw_preview.py [out_path]   (default: out/preview.png)
"""
import os
import sys
import urllib.request

from cw_common import ENV_UUID, connect

out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "out", "preview.png")
os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
p = connect().environments.create_preview(ENV_UUID)
with urllib.request.urlopen(p.file_url, timeout=60) as r, open(out, "wb") as f:
    f.write(r.read())
print("preview:", os.path.abspath(out), p.metadata.get("view") if isinstance(p.metadata, dict) else "")
