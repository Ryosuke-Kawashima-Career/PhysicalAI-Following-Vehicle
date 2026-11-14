"""Run the follow-me demo.

Usage (from the repo root):
  .venv/Scripts/python.exe -m followme.main --offline [--seed 42] [--duration 60] [--plot] [--no-save]
  (with PYTHONPATH=src, or after `uv pip install -e .`)

--offline     local kinematic loop only (default)
--cyberwave   also stream the poses to the Cyberwave twins (plan Step 4 - not implemented yet)
--plot        write out/run_<ts>_summary.png and out/run_<ts>.gif
"""
import argparse
import datetime as dt
import json
import sys
from dataclasses import replace
from pathlib import Path

from .config import Config
from .sim import simulate
from .world import ROOT


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--offline", action="store_true", help="local loop only (default)")
    mode.add_argument("--cyberwave", action="store_true", help="stream poses to Cyberwave (Step 4)")
    ap.add_argument("--seed", type=int, default=Config.seed)
    ap.add_argument("--duration", type=float, default=Config.duration)
    ap.add_argument("--scene", default=str(ROOT / "out" / "scene.json"))
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--no-save", action="store_true", help="don't write out/run_<ts>.json")
    args = ap.parse_args(argv)
    if args.cyberwave:
        sys.exit("--cyberwave is plan Step 4 (cw_bridge.py) and is not implemented yet. Use --offline.")

    cfg = replace(Config(), seed=args.seed, duration=args.duration)
    res = simulate(cfg, scene_path=args.scene)
    s = res["summary"]
    print(json.dumps({k: v for k, v in s.items() if k != "pass"}, indent=2))
    for k, ok in s["pass"].items():
        print(f"  {'PASS' if ok else 'FAIL'}  {k}")

    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    out = ROOT / "out"
    out.mkdir(exist_ok=True)
    if not args.no_save:
        run = {"scene": args.scene, "seed": cfg.seed, "dt": cfg.dt, "duration": cfg.duration, "mode": "offline",
               "summary": s, "ticks": res["ticks"]}
        path = out / f"run_{ts}.json"
        path.write_text(json.dumps(run), encoding="utf-8")
        print("wrote", path)
    if args.plot:
        from .viz import animate, summary_plot
        print("wrote", summary_plot(res, out / f"run_{ts}_summary.png"))
        print("wrote", animate(res, out / f"run_{ts}.gif"))
    return 0 if s["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
