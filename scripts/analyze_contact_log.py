#!/usr/bin/env python3
"""Summarize structured contact logs emitted by the SDK supervisor.

Frame-level contacts are merged into episodes with time, position, peak point
count, and maximum contact height. Wheel-ground contacts are excluded upstream.
"""

import argparse
import json


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def episodes(rows, gap=0.3):
    if not rows:
        return []
    out, cur = [], [rows[0]]
    for a, b in zip(rows, rows[1:]):
        if b["t"] - a["t"] > gap:
            out.append(cur)
            cur = [b]
        else:
            cur.append(b)
    out.append(cur)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="Path to contact_<world>.jsonl")
    ap.add_argument("--window", nargs=2, type=float, metavar=("START", "END"),
                    help="Restrict analysis to this time window")
    ap.add_argument("--gap", type=float, default=0.3, help="Maximum gap within one episode")
    ap.add_argument("--car-slot", default="car_1",
                    help="Car slot to inspect; use 'all' for every vehicle")
    args = ap.parse_args()

    rows = load(args.path)
    if args.car_slot != "all":
        rows = [r for r in rows if r.get("car_slot", args.car_slot) == args.car_slot]
    if args.window:
        lo, hi = args.window
        rows = [r for r in rows if lo <= r["t"] <= hi]

    print("================ CONTACT SUMMARY ================")
    print(f"source         : {args.path}")
    print(f"contact frames : {len(rows)}")
    if not rows:
        print("(no body or barrier contacts recorded in this window)")
        return

    eps = episodes(rows, args.gap)
    print(f"episode  : {len(eps)}")
    print(f"{'t_start':>8} {'t_end':>7} {'frames':>6} {'peak':>5} {'type':>14}  {'pos(x,y)':>16}  {'zmax':>5}")
    for e in eps:
        zmax = max((p[2] for r in e for p in r.get("points", []) if len(p) >= 3), default=0.0)
        peak = max((r.get("count", 0) for r in e), default=0)
        kinds = {r.get("kind") for r in e}
        kind = "car_car" if "car_car" in kinds else "static_geometry"
        mid = e[len(e) // 2]
        print(f"{e[0]['t']:>8.1f} {e[-1]['t']:>7.1f} {len(e):>4} {peak:>5} {kind:>14}  "
              f"({mid['x']:>6.1f},{mid['y']:>6.1f})  {zmax:>5.2f}")
    print("=" * 46)
    print("Interpretation: sustained contacts with at least three points and zmax > 0.6 "
          "are stronger evidence than isolated one- or two-point startup contacts.")


if __name__ == "__main__":
    main()
