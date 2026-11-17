"""Render a top-down multi-car trajectory from Webots telemetry.

Vehicle rectangles show position and heading over time; the optional world
background reconstructs roads from the Webots file, and optional contact logs
add collision evidence.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.analyze_telemetry import DEFAULT_TELEMETRY, _split_runs
from scripts.track_geometry import parse_world

_BASE_COLORS = [
    "#E53935", "#1E88E5", "#43A047", "#FB8C00", "#8E24AA",
    "#00ACC1", "#6D4C41", "#C0CA33", "#5E35B1", "#546E7A",
]


def _load_all_cars(path: Path) -> list[dict]:
    """Stream telemetry into per-frame vehicle dictionaries."""

    frames: list[dict] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = rec.get("t")
            if t is None:
                continue
            cars: dict[str, dict] = {}
            for car in rec.get("cars") or []:
                tid = car.get("team_id")
                x, y = car.get("x"), car.get("y")
                if tid is None or x is None or y is None:
                    continue
                if any(math.isnan(float(v)) for v in (x, y)):
                    continue
                cars[tid] = {
                    "x": float(x),
                    "y": float(y),
                    "heading": float(car.get("heading", 0.0)),
                    "speed": float(car.get("speed", 0.0)),
                    "status": car.get("status", "normal"),
                }
            if cars:
                frames.append({"t": float(t), "cars": cars})
    return frames


def _car_polygon(x: float, y: float, heading: float, length: float, width: float) -> np.ndarray:
    """Return the four corners of a vehicle rectangle at a world pose."""

    cos_h, sin_h = math.cos(heading), math.sin(heading)
    half_l, half_w = length * 0.5, width * 0.5
    local = [(half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w)]
    return np.array([(x + lx * cos_h - ly * sin_h, y + lx * sin_h + ly * cos_h) for lx, ly in local])


def _load_contact_xy(path: Path, team_id: str | None, t0: float, t1: float) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []
    with path.open(encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if team_id is not None and row.get("team_id") != team_id:
                continue
            t = float(row.get("t", -1.0))
            if t0 <= t <= t1 and row.get("x") is not None and row.get("y") is not None:
                pts.append((float(row["x"]), float(row["y"])))
    return pts


def plot_topdown(args: argparse.Namespace) -> int:
    frames = _load_all_cars(args.telemetry)
    if not frames:
        print(f"[error] no usable frames: {args.telemetry}")
        return 1
    runs = _split_runs(frames)
    run = runs[args.run] if -len(runs) <= args.run < len(runs) else runs[-1]

    t_lo, t_hi = run[0]["t"], run[-1]["t"]
    if args.window:
        w0, w1 = (float(tok) for tok in args.window.split(","))
        t_lo, t_hi = max(t_lo, w0), min(t_hi, w1)
    win = [fr for fr in run if t_lo <= fr["t"] <= t_hi]
    if not win:
        print(f"[error] no frames in time window: {args.window}")
        return 1

    all_ids = sorted({tid for fr in win for tid in fr["cars"]})
    selected = [c.strip() for c in args.cars.split(",")] if args.cars else all_ids
    selected = [c for c in selected if c in all_ids]
    if not selected:
        print(f"[error] selected cars are absent; available IDs: {all_ids}")
        return 1
    color_of = {tid: _BASE_COLORS[i % len(_BASE_COLORS)] for i, tid in enumerate(all_ids)}

    fig, ax = plt.subplots(figsize=(9, 9))
    span = max(t_hi - t_lo, 1e-6)
    xs_all, ys_all = [], []

    track = None
    if args.world:
        try:
            track = parse_world(args.world)
        except Exception as exc:

            print(f"[warn] world parsing failed; using a plain background: {exc}")
    if track:
        for cx, cy, sx, sy, rgb in track["ground"]:
            ax.add_patch(plt.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                       facecolor=rgb, edgecolor="none", zorder=0))
        for poly in track["roads"]:
            ax.add_patch(Polygon(poly, closed=True, facecolor="#333333",
                                 edgecolor="#dddddd", linewidth=0.4, zorder=1))

    for tid in selected:
        is_focus = (tid == args.focus)
        color = color_of[tid]
        path_xy = [(fr["cars"][tid]["x"], fr["cars"][tid]["y"]) for fr in win if tid in fr["cars"]]
        if not path_xy:
            continue
        px, py = zip(*path_xy)
        xs_all.extend(px)
        ys_all.extend(py)
        ax.plot(px, py, color=color, lw=1.2 if is_focus else 0.7, alpha=0.35, zorder=2)

        next_sample = t_lo
        for fr in win:
            if tid not in fr["cars"]:
                continue
            if fr["t"] + 1e-9 < next_sample:
                continue
            next_sample = fr["t"] + args.sample_dt
            car = fr["cars"][tid]
            frac = (fr["t"] - t_lo) / span
            alpha = 0.22 + 0.73 * frac
            poly = _car_polygon(car["x"], car["y"], car["heading"] + args.heading_offset,
                                args.car_length, args.car_width)
            ax.add_patch(Polygon(poly, closed=True, facecolor=color, edgecolor="black",
                                 linewidth=1.1 if is_focus else 0.5, alpha=alpha, zorder=4 if is_focus else 3))
            if is_focus and args.annotate_speed:
                ax.annotate(f"{car['speed']:.1f}", (car["x"], car["y"]), fontsize=6,
                            color="black", ha="center", va="center", zorder=6)

        ax.scatter([px[0]], [py[0]], c=color, s=90, marker="o", edgecolors="white",
                   linewidth=1.0, zorder=5)
        ax.scatter([px[-1]], [py[-1]], c=color, s=150, marker="*", edgecolors="white",
                   linewidth=1.0, zorder=5)

    if args.contact_log and args.contact_log.is_file():
        cpts = _load_contact_xy(args.contact_log, args.focus, t_lo, t_hi)
        if cpts:
            cx, cy = zip(*cpts)
            ax.scatter(cx, cy, c="crimson", s=70, marker="X", edgecolors="white",
                       linewidth=0.6, zorder=7, label="contact")

    if args.full_track and track and track["extent"]:
        xmin, xmax, ymin, ymax = track["extent"]
        ax.set_xlim(xmin - args.pad, xmax + args.pad)
        ax.set_ylim(ymin - args.pad, ymax + args.pad)
    elif xs_all and ys_all:
        pad = args.pad
        ax.set_xlim(min(xs_all) - pad, max(xs_all) + pad)
        ax.set_ylim(min(ys_all) - pad, max(ys_all) + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (world)")
    ax.set_ylabel("y (world)")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"{args.title}  (t={t_lo:.1f}–{t_hi:.1f}s)")

    handles = [plt.Line2D([0], [0], marker="s", color="w", markerfacecolor=color_of[t],
                          markersize=9, label=(t + " (focus)" if t == args.focus else t))
               for t in selected]
    handles.append(plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="gray",
                              markersize=8, label="start o / end *  (alpha = time order)"))
    ax.legend(handles=handles, loc="best", fontsize=8)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"saved: {args.out}  cars={selected}  frames={len(win)}")
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Render top-down multi-car telemetry.")
    p.add_argument("--telemetry", type=Path, default=DEFAULT_TELEMETRY,
                   help=f"Telemetry path (default: {DEFAULT_TELEMETRY})")
    p.add_argument("--run", type=int, default=-1, help="Run index; -1 selects the latest")
    p.add_argument("--window", default=None, help="Time window t0,t1 in seconds")
    p.add_argument("--cars", default=None, help="Comma-separated team IDs; defaults to all")
    p.add_argument("--focus", default=None, help="Team ID to emphasize")
    p.add_argument("--sample-dt", type=float, default=0.5, help="Time between vehicle rectangles")
    p.add_argument("--car-length", type=float, default=2.5, help="Rectangle length in world units")
    p.add_argument("--car-width", type=float, default=1.2, help="Rectangle width in world units")
    p.add_argument("--heading-offset", type=float, default=0.0, help="Heading correction in radians")
    p.add_argument("--annotate-speed", action="store_true", help="Annotate focus-vehicle speed")
    p.add_argument("--contact-log", type=Path, default=None, help="Optional contact log")
    p.add_argument("--world", default=None, help="basic, complex, or a .wbt path")
    p.add_argument("--full-track", action="store_true", help="Frame the full track")
    p.add_argument("--pad", type=float, default=8.0, help="View padding in world units")
    p.add_argument("--out", type=Path, required=True, help="Output PNG")
    p.add_argument("--title", default="multi-car top-down", help="Plot title")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    args.telemetry = args.telemetry.expanduser().resolve()
    if not args.telemetry.is_file():
        print(f"[error] telemetry file not found: {args.telemetry}")
        return 1
    return plot_topdown(args)


if __name__ == "__main__":
    raise SystemExit(main())
