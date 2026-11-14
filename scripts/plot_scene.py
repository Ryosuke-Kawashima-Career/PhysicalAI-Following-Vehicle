"""Top-down map of out/scene.json (the exported environment), with footprints, the traveller route and clearances.

Usage: .venv/Scripts/python.exe scripts/plot_scene.py [scene.json] [out.png]   (default: out/scene.json -> out/scene_top.png)
"""
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "out", "scene.json")
dst = sys.argv[2] if len(sys.argv) > 2 else os.path.join(ROOT, "out", "scene_top.png")
scene = json.load(open(src, encoding="utf-8"))

COLORS = {"concourse": "#8a8f98", "obstacles": "#c9822b", "actors": "#2b6cc9"}
fig, ax = plt.subplots(figsize=(13, 6))
for o in scene["objects"]:
    (cx, cy, _), (sx, sy, _) = o["center"], o["size"]
    if o["name"] == "floor":
        ax.add_patch(mp.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, fc="#f4f4f2", ec="#cfcfcf", zorder=0))
        continue
    col = "#d64545" if o["name"] == "follow_cart" else ("#2e9e5b" if o["name"] == "traveller" else COLORS[o["group"]])
    if o["shape"] == "circle":
        ax.add_patch(mp.Circle((cx, cy), max(sx, sy) / 2, fc=col, ec="k", lw=0.6, zorder=3))
        a = math.radians(o["yaw_deg"])
        if not o["static"]:
            ax.arrow(cx, cy, 0.6 * math.cos(a), 0.6 * math.sin(a), head_width=0.18, color="k", lw=0.8, zorder=4)
    else:
        r = mp.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy, fc=col, ec="k", lw=0.6, zorder=2)
        r.set_transform(matplotlib.transforms.Affine2D().rotate_deg_around(cx, cy, o["yaw_deg"]) + ax.transData)
        ax.add_patch(r)
    if o["group"] != "concourse":
        ax.annotate(o["name"], (cx, cy), xytext=(0, 9 + max(sx, sy) * 8), textcoords="offset points",
                    ha="center", fontsize=7.5, zorder=5)
wx = [w["x"] for w in scene["waypoints"]]
wy = [w["y"] for w in scene["waypoints"]]
ax.plot(wx, wy, "--", color="#2e9e5b", lw=1.2, zorder=1, label="traveller route")
ax.scatter(wx, wy, s=14, color="#2e9e5b", zorder=1)
for w in scene["waypoints"]:
    ax.annotate(w["name"], (w["x"], w["y"]), xytext=(3, -10), textcoords="offset points", fontsize=6.5, color="#2e9e5b")
ax.set_xlim(-16, 16)
ax.set_ylim(-7, 7)
ax.set_aspect("equal")
ax.set_xticks(range(-15, 16, 3))
ax.grid(alpha=0.25)
ax.set_xlabel("x (m)  - open end at x = -15")
ax.set_ylabel("y (m)")
ax.set_title(f"{scene['environment']['slug']}  (exported {scene['exported_at']})", fontsize=9)
ax.legend(loc="lower right", fontsize=8)
fig.tight_layout()
fig.savefig(dst, dpi=110)
print("wrote", os.path.abspath(dst))
