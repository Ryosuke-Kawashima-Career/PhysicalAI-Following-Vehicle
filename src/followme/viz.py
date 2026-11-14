"""Plots for a run: a summary figure (paths + distance / clearance over time) and a top-down GIF animation."""
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

from . import config as C
from .world import Box, Circle

COL = {"follow_cart": "#d64545", "traveller": "#2e9e5b", "ped": "#2b6cc9"}


def _draw_static(ax, world):
    x0, x1, y0, y1 = world.bounds
    ax.add_patch(mp.Rectangle((x0, y0), x1 - x0, y1 - y0, fc="#f4f4f2", ec="#cfcfcf", zorder=0))
    for o in world.static:
        if isinstance(o, Circle):
            ax.add_patch(mp.Circle((o.x, o.y), o.r, fc="#c9822b", ec="k", lw=0.5, zorder=2))
        else:
            ax.add_patch(mp.Polygon(o.corners(), closed=True, fc="#8a8f98" if "wall" in o.name else "#c9822b",
                                    ec="k", lw=0.5, zorder=2))
    rx, ry = zip(*world.route)
    ax.plot(rx, ry, ":", color=COL["traveller"], lw=1, zorder=1)
    ax.set_xlim(x0 - 1, x1 + 1)
    ax.set_ylim(y0 - 1, y1 + 1)
    ax.set_aspect("equal")
    ax.grid(alpha=0.2)


def _series(res, name):
    xs, ys = [], []
    for tk in res["ticks"]:
        a = next(a for a in tk["actors"] if a["id"] == name)
        xs.append(a["x"])
        ys.append(a["y"])
    return xs, ys


def summary_plot(res, path):
    m, s, world = res["metrics"], res["summary"], res["world"]
    fig = plt.figure(figsize=(13, 8))
    ax = fig.add_subplot(2, 1, 1)
    _draw_static(ax, world)
    names = [a["id"] for a in res["ticks"][0]["actors"]]
    for n in names:
        xs, ys = _series(res, n)
        col = COL.get(n, COL["ped"])
        ax.plot(xs, ys, "-", color=col, lw=1.6 if n in ("follow_cart", "traveller") else 0.9, alpha=0.9, label=n)
        ax.plot(xs[-1], ys[-1], "o", color=col, ms=4)
    ax.legend(loc="lower right", fontsize=7, ncol=3)
    flags = "  ".join(f"{k}: {'PASS' if v else 'FAIL'}" for k, v in s["pass"].items())
    ax.set_title(f"seed {res['config'].seed}, {res['config'].duration:.0f} s - {flags}", fontsize=9)

    ax2 = fig.add_subplot(2, 2, 3)
    ax2.plot(m.t, m.dist, color=COL["follow_cart"], lw=1)
    ax2.axhspan(*C.BAND, color=COL["traveller"], alpha=0.12, label="band 1.0-2.5 m")
    ax2.axhline(C.MAX_DISTANCE, color="k", ls="--", lw=0.8, label="max 4 m")
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("cart <-> traveller (m)")
    ax2.set_title(f"in band {100 * s['band_fraction']:.1f} % (need >= 85 %), max {s['max_distance']:.2f} m", fontsize=9)
    ax2.legend(fontsize=7)
    ax3 = fig.add_subplot(2, 2, 4)
    ax3.plot(m.t, m.clear, color="#444", lw=1)
    ax3.axhline(C.MIN_CLEARANCE, color="#d64545", ls="--", lw=0.8, label="R4 limit 0.3 m")
    ax3.set_ylim(0, 2.0)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel("min clearance (m)")
    ax3.set_title(f"min {s['min_clearance']:.2f} m ({s['min_clearance_with']} at {s['min_clearance_t']:.1f} s)",
                  fontsize=9)
    ax3.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return Path(path)


def _cart_poly(a, world):
    sx, sy = world.actors["follow_cart"]["size"][:2]
    return Box(a["x"], a["y"], sx / 2, sy / 2, a["yaw"]).corners()


def animate(res, path, stride=2, frames_png=None):
    """GIF at 10/stride fps of sim time x1 (stride=2 -> every 0.2 s). frames_png: list of times to also save as PNG."""
    world, ticks = res["world"], res["ticks"]
    fig, ax = plt.subplots(figsize=(11, 5))
    _draw_static(ax, world)
    cart = mp.Polygon(_cart_poly(ticks[0]["actors"][0], world), closed=True, fc=COL["follow_cart"], ec="k", zorder=5)
    ax.add_patch(cart)
    people = {}
    for a in ticks[0]["actors"][1:]:
        col = COL["traveller"] if a["id"] == "traveller" else COL["ped"]
        circ = mp.Circle((a["x"], a["y"]), 0.3, fc=col, ec="k", lw=0.5, zorder=4)
        ax.add_patch(circ)
        arrow, = ax.plot([], [], "k-", lw=1, zorder=6)
        label = ax.text(a["x"], a["y"] + 0.45, a["id"], fontsize=6.5, ha="center", zorder=7)
        people[a["id"]] = (circ, arrow, label)
    trail, = ax.plot([], [], "-", color=COL["follow_cart"], lw=0.8, alpha=0.6, zorder=3)
    link, = ax.plot([], [], "--", color="#555", lw=0.7, zorder=3)
    info = ax.text(0.01, 0.98, "", transform=ax.transAxes, va="top", fontsize=8, family="monospace")
    m = res["metrics"]
    idx = list(range(0, len(ticks), stride))
    tx, ty = [], []

    def draw(k):
        tk = ticks[k]
        c = tk["actors"][0]
        cart.set_xy(_cart_poly(c, world))
        tx.append(c["x"])
        ty.append(c["y"])
        trail.set_data(tx, ty)
        for a in tk["actors"][1:]:
            circ, arrow, label = people[a["id"]]
            circ.center = (a["x"], a["y"])
            arrow.set_data([a["x"], a["x"] + 0.45 * math.cos(a["yaw"])], [a["y"], a["y"] + 0.45 * math.sin(a["yaw"])])
            label.set_position((a["x"], a["y"] + 0.45))
        tr = tk["actors"][1]
        link.set_data([c["x"], tr["x"]], [c["y"], tr["y"]])
        state = "HOLD" if tk.get("holding") else ("STOP:" + tk["cone"] if tk.get("cone") else "FOLLOW")
        info.set_text(f"t={tk['t']:5.1f}s  v={tk['v']:.2f} m/s  d={m.dist[k]:.2f} m  "
                      f"clear={m.clear[k]:.2f} m ({m.clear_with[k]})  {state}")
        return [cart, trail, link, info]

    if frames_png:
        for t in frames_png:
            k = min(range(len(ticks)), key=lambda i: abs(ticks[i]["t"] - t))
            tx.clear(), ty.clear()
            tx.extend(_series(res, "follow_cart")[0][:k + 1])
            ty.extend(_series(res, "follow_cart")[1][:k + 1])
            tx.pop(), ty.pop()
            draw(k)
            fig.savefig(Path(path).with_name(f"{Path(path).stem}_t{t:04.1f}.png"), dpi=90)
        tx.clear(), ty.clear()
    anim = FuncAnimation(fig, draw, frames=idx, blit=False)
    anim.save(path, writer=PillowWriter(fps=int(10 / stride)), dpi=70)
    plt.close(fig)
    return Path(path)
