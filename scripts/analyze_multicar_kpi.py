#!/usr/bin/env python3
"""Summarize multi-car Webots telemetry as race-relevant KPIs.

The report covers completion, rank, points, collision budget, overtakes,
start-line crowding, stalls, and speed. Official metadata is preferred; older
runs without checkpoint progress fall back to cumulative distance and are
explicitly labelled as proxy rankings.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path

POINTS = {1: 10, 2: 7, 3: 5, 4: 3}


def points_for_rank(rank: int | None) -> int:
    if rank is None:
        return 1
    return POINTS.get(rank, 1)


def load_runs(path: Path) -> list[list[dict]]:
    """Read telemetry and split appended runs at timestamp resets."""

    runs: list[list[dict]] = []
    cur: list[dict] = []
    last_t = None
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                continue
            t = frame.get("t", 0.0)
            if last_t is not None and t < last_t - 0.5:
                runs.append(cur)
                cur = []
            cur.append(frame)
            last_t = t
    if cur:
        runs.append(cur)
    return runs


def pick_team_index(team_ids: list[str], requested: str | None) -> int:
    """Choose the controlled vehicle index from the recorded team IDs."""

    if requested:
        for i, tid in enumerate(team_ids):
            if tid == requested:
                return i
        for i, tid in enumerate(team_ids):
            if requested.lower() in tid.lower():
                return i
    for i, tid in enumerate(team_ids):
        if "our" in tid.lower():
            return i
    return 0


def cumulative_distance(run: list[dict], ci: int) -> list[float]:
    """Compute cumulative distance as a fallback progress proxy."""

    out = [0.0]
    for a, b in zip(run, run[1:]):
        ca, cb = a["cars"][ci], b["cars"][ci]
        out.append(out[-1] + math.hypot(cb["x"] - ca["x"], cb["y"] - ca["y"]))
    return out


def progress_series(run: list[dict], ncars: int) -> tuple[list[list[float]], bool]:
    """Return per-car progress and whether checkpoint progress was available."""

    max_lp = 0.0
    for fr in run:
        for c in fr["cars"]:
            max_lp = max(max_lp, c.get("lap", 0) + c.get("lap_progress", 0.0))
    if max_lp > 1e-6:
        prog = [[fr["cars"][ci].get("lap", 0) + fr["cars"][ci].get("lap_progress", 0.0)
                 for fr in run] for ci in range(ncars)]
        return prog, True
    prog = [cumulative_distance(run, ci) for ci in range(ncars)]
    return prog, False


def rank_at(progress: list[list[float]], frame_idx: int) -> list[int]:
    """Return each car's one-based rank at a frame."""

    order = sorted(range(len(progress)), key=lambda ci: -progress[ci][frame_idx])
    rank = [0] * len(progress)
    for pos, ci in enumerate(order):
        rank[ci] = pos + 1
    return rank


def analyze(run: list[dict], team_index: int, args) -> dict:
    team_ids = [c["team_id"] for c in run[-1]["cars"]]
    ncars = len(team_ids)
    me = team_index
    t_end = run[-1]["t"]

    speeds = [fr["cars"][me]["speed"] for fr in run]
    speeds = [v for v in speeds if v is not None and math.isfinite(v)]
    speed_mean = statistics.fmean(speeds) if speeds else 0.0
    speed_median = statistics.median(speeds) if speeds else 0.0

    stall_events = 0
    stall_total_s = 0.0
    run_start_t = run[0]["t"]
    seg_start = None
    prev_t = run_start_t
    for fr in run:
        v = fr["cars"][me]["speed"]
        t = fr["t"]
        if v < args.stall_speed:
            if seg_start is None:
                seg_start = t
        else:
            if seg_start is not None and t - seg_start >= args.stall_seconds:
                stall_events += 1
                stall_total_s += t - seg_start
            seg_start = None
        prev_t = t
    if seg_start is not None and prev_t - seg_start >= args.stall_seconds:
        stall_events += 1
        stall_total_s += prev_t - seg_start

    progress, used_cp = progress_series(run, ncars)
    start_rank = rank_at(progress, 0)[me]
    end_rank = rank_at(progress, len(run) - 1)[me]
    overtakes_made = 0
    overtakes_conceded = 0
    prev_rank = start_rank
    for fi in range(1, len(run)):
        r = rank_at(progress, fi)[me]
        if r < prev_rank:
            overtakes_made += prev_rank - r
        elif r > prev_rank:
            overtakes_conceded += r - prev_rank
        prev_rank = r

    sev_major = 0
    sev_minor = 0
    contact_starts = 0
    finished = False
    dq = False
    for fr in run:
        for e in fr.get("events", []):
            et = e.get("type")
            tids = e.get("team_ids") or ([e.get("team_id")] if e.get("team_id") else [])
            involves_me = team_ids[me] in tids
            if et == "collision" and involves_me:
                if e.get("severity") == "major":
                    sev_major += 1
                else:
                    sev_minor += 1
            elif et == "contact_start" and (involves_me or e.get("team_id") is None):
                contact_starts += 1
            elif et == "car_finished" and involves_me:
                finished = True
            elif et in ("disqualified", "car_disqualified") and involves_me:
                dq = True

    squeeze_min_dist = math.inf
    squeeze_collisions = 0
    for fr in run:
        if fr["t"] - run_start_t > args.start_window:
            break
        mx, my = fr["cars"][me]["x"], fr["cars"][me]["y"]
        for ci in range(ncars):
            if ci == me:
                continue
            ox, oy = fr["cars"][ci]["x"], fr["cars"][ci]["y"]
            squeeze_min_dist = min(squeeze_min_dist, math.hypot(ox - mx, oy - my))
        for e in fr.get("events", []):
            if e.get("type") == "collision" and team_ids[me] in (e.get("team_ids") or []):
                squeeze_collisions += 1
    squeezed = squeeze_min_dist < args.squeeze_dist or squeeze_collisions > 0

    return {
        "team_ids": team_ids,
        "me": team_ids[me],
        "ncars": ncars,
        "t_end": t_end,
        "speed_mean": speed_mean,
        "speed_median": speed_median,
        "stall_events": stall_events,
        "stall_total_s": stall_total_s,
        "used_cp": used_cp,
        "max_progress": max(progress[me]) if used_cp else None,
        "start_rank": start_rank,
        "end_rank": end_rank,
        "overtakes_made": overtakes_made,
        "overtakes_conceded": overtakes_conceded,
        "sev_major": sev_major,
        "sev_minor": sev_minor,
        "contact_starts": contact_starts,
        "finished": finished,
        "dq": dq,
        "squeeze_min_dist": squeeze_min_dist,
        "squeeze_collisions": squeeze_collisions,
        "squeezed": squeezed,
    }


def load_metadata(path: Path | None, my_team: str) -> dict | None:
    """Read final ranking and collision fields from supervisor metadata."""

    if path is None or not path.exists():
        return None
    try:
        meta = json.loads(path.read_text(encoding="utf-8"))
    except (json.JSONDecodeError, OSError):
        return None
    rankings = meta.get("final_rankings") or []
    mine = next((r for r in rankings if r.get("team_id") == my_team), None)
    return {
        "finish_reason": meta.get("finish_reason"),
        "n_ranked": len(rankings),
        "my_rank": mine.get("rank") if mine else None,
        "my_laps": mine.get("laps") if mine else None,
        "my_best_lap": mine.get("best_lap") if mine else None,
        "my_total_time": mine.get("total_time") if mine else None,
        "my_major": mine.get("collision_major_count") if mine else None,
        "my_status": mine.get("status") if mine else None,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize multi-car race KPIs")
    parser.add_argument("telemetry", type=Path, help="Path to telemetry.jsonl")
    parser.add_argument("--metadata", type=Path, default=None,
                        help="Metadata path; defaults to the telemetry directory")
    parser.add_argument("--team", default=None, help="Controlled team ID")
    parser.add_argument("--run", default="last", help="Run index or 'last'")
    parser.add_argument("--stall-speed", type=float, default=0.5, help="Stall speed threshold in m/s")
    parser.add_argument("--stall-seconds", type=float, default=3.0, help="Minimum stall duration")
    parser.add_argument("--start-window", type=float, default=5.0, help="Start-line crowding window")
    parser.add_argument("--squeeze-dist", type=float, default=3.0, help="Crowding distance threshold")
    args = parser.parse_args()

    runs = load_runs(args.telemetry)
    if not runs:
        print("No usable telemetry frames.")
        return
    if args.run == "last":
        ridx = len(runs) - 1
    else:
        ridx = int(args.run)
        if ridx < 0:
            ridx += len(runs)
    if not (0 <= ridx < len(runs)):
        print(f"Run index out of range; {len(runs)} runs are available.")
        return
    run = runs[ridx]

    team_ids = [c["team_id"] for c in run[-1]["cars"]]
    me = pick_team_index(team_ids, args.team)
    k = analyze(run, me, args)

    meta_path = args.metadata
    if meta_path is None:
        meta_path = args.telemetry.with_name("metadata.json")
    meta = load_metadata(meta_path, k["me"])

    if meta and meta["my_rank"] is not None:
        rank = meta["my_rank"]
        rank_src = "metadata(official)"
    else:
        rank = k["end_rank"]
        rank_src = "telemetry-progress[proxy]" if not k["used_cp"] else "telemetry-progress"
    pts = points_for_rank(rank)
    sev_major = meta["my_major"] if (meta and meta["my_major"] is not None) else k["sev_major"]

    print("=" * 60)
    print(f"MULTI-CAR KPI  run#{ridx}/{len(runs)-1}  team={k['me']}  cars={k['ncars']}  t_end={k['t_end']:.0f}s")
    print(f"  opponents: {[t for t in team_ids if t != k['me']]}")
    print("-" * 60)
    print(f"[rank/points] rank={rank}  points={pts}   ({rank_src})")
    if meta:
        print(f"             finish_reason={meta['finish_reason']}  ranked={meta['n_ranked']}  "
              f"laps={meta['my_laps']}  best_lap={meta['my_best_lap']}  total_time={meta['my_total_time']}")
    finished = (meta and meta["my_status"] == "finished") or k["finished"]
    print(f"[finish]      finished={finished}  DQ={k['dq']}   (three major collisions cause disqualification)")
    print(f"[collisions]  major={sev_major}   minor={k['sev_minor']}   body-contact episodes={k['contact_starts']}")
    if k["used_cp"]:
        print(f"[progress]    max_progress={k['max_progress']:.2f} laps")
    else:
        print("[progress]    checkpoint progress unavailable; rank uses distance proxy")
    print(f"[overtakes]   made={k['overtakes_made']}  conceded={k['overtakes_conceded']}  "
          f"net={k['overtakes_made'] - k['overtakes_conceded']}   (start_rank={k['start_rank']}→end={k['end_rank']})")
    sd = k["squeeze_min_dist"]
    print(f"[start crowd] squeezed={k['squeezed']}  nearest_opponent={sd:.2f}m  collisions={k['squeeze_collisions']}  "
          f"(window {args.start_window:.0f}s)")
    print(f"[stalls]      episodes={k['stall_events']}  total={k['stall_total_s']:.1f}s  (<{args.stall_speed} m/s for {args.stall_seconds:.0f}s)")
    print(f"[speed]       mean={k['speed_mean']:.2f}  median={k['speed_median']:.2f} m/s")
    print("=" * 60)


if __name__ == "__main__":
    main()
