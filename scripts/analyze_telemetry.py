"""Summarize large supervisor telemetry files without loading them into memory.

The report covers position, speed, low-speed episodes, progress, and events for
one vehicle. It can also archive the raw run under the repository's ignored
``.tmp/recordings`` directory.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SDK_DEFAULT = Path(os.environ.get("AIRACER_SDK", "pkudsa.airacer/sdk")) / ".local" / "recordings"
DEFAULT_TELEMETRY = SDK_DEFAULT / "telemetry.jsonl"
ARCHIVE_ROOT = ROOT / ".tmp" / "recordings"


def _pct(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round(q * (len(ordered) - 1)))))
    return ordered[idx]


def _load_car_frames(path: Path, team_id: str | None) -> tuple[list[dict], list[tuple[float, dict]]]:
    """Stream telemetry and return frames for one car plus all recorded events."""

    frames: list[dict] = []
    events: list[tuple[float, dict]] = []
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
            cars = rec.get("cars") or []
            for ev in rec.get("events") or []:
                events.append((t, ev))
            car = None
            if team_id is None:
                car = cars[0] if cars else None
            else:
                for candidate in cars:
                    if candidate.get("team_id") == team_id:
                        car = candidate
                        break
            if car is None:
                continue
            x, y, speed = car.get("x"), car.get("y"), car.get("speed")
            if x is None or y is None or speed is None:
                continue
            if any(math.isnan(v) for v in (float(x), float(y), float(speed))):
                continue
            frames.append({
                "t": float(t),
                "x": float(x),
                "y": float(y),
                "speed": float(speed),
                "lap": car.get("lap", 0),
                "lap_progress": float(car.get("lap_progress", 0.0)),
                "status": car.get("status", "normal"),
            })
    return frames, events


def _split_runs(frames: list[dict]) -> list[list[dict]]:
    """Split an appended telemetry file whenever its timestamp moves backward."""

    segments: list[list[dict]] = []
    current: list[dict] = []
    last_t = None
    for fr in frames:
        if last_t is not None and fr["t"] < last_t - 1e-6:
            if current:
                segments.append(current)
            current = []
        current.append(fr)
        last_t = fr["t"]
    if current:
        segments.append(current)
    return segments


def _read_metadata_frames(telemetry_path: Path) -> int | None:
    """Read ``metadata.json`` for a frame-count integrity check when available."""

    meta = telemetry_path.parent / "metadata.json"
    if not meta.is_file():
        return None
    try:
        return int(json.loads(meta.read_text(encoding="utf-8")).get("total_frames"))
    except (ValueError, TypeError, json.JSONDecodeError):
        return None


def _longest_low_speed(frames: list[dict], thresh: float) -> dict | None:
    """Return the longest contiguous episode below ``thresh``."""

    best = None
    run_start = None
    for i, fr in enumerate(frames):
        if fr["speed"] < thresh:
            if run_start is None:
                run_start = i
        else:
            if run_start is not None:
                best = _take_longer(best, frames, run_start, i - 1)
                run_start = None
    if run_start is not None:
        best = _take_longer(best, frames, run_start, len(frames) - 1)
    return best


def _take_longer(best, frames, i0, i1):
    dur = frames[i1]["t"] - frames[i0]["t"]
    if best is None or dur > best["dur"]:
        return {
            "dur": dur,
            "t0": frames[i0]["t"], "t1": frames[i1]["t"],
            "x0": frames[i0]["x"], "y0": frames[i0]["y"],
            "x1": frames[i1]["x"], "y1": frames[i1]["y"],
        }
    return best


def _archive(src_dir: Path, label: str) -> Path:
    dest = ARCHIVE_ROOT / label
    dest.mkdir(parents=True, exist_ok=True)
    for name in ("telemetry.jsonl", "metadata.json", "live_view.jpg"):
        src = src_dir / name
        if src.exists():
            shutil.copy2(src, dest / name)
    return dest


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize supervisor telemetry.jsonl.")
    parser.add_argument("--telemetry", type=Path, default=DEFAULT_TELEMETRY,
                        help=f"Telemetry path (default: {DEFAULT_TELEMETRY})")
    parser.add_argument("--team-id", default=None, help="Target team ID; defaults to the first car")
    parser.add_argument("--archive", default=None,
                        help="Archive label under .tmp/recordings/<label>")
    parser.add_argument("--no-archive", action="store_true", help="Do not archive the raw run")
    args = parser.parse_args()

    path = args.telemetry
    if not path.is_file():
        print(f"[error] telemetry file not found: {path}")
        return 1

    all_frames, events = _load_car_frames(path, args.team_id)
    if not all_frames:
        print(f"[error] no vehicle frames parsed (team_id={args.team_id})")
        return 1

    runs = _split_runs(all_frames)
    frames = runs[-1]
    meta_frames = _read_metadata_frames(path)
    integrity = "clean"
    if len(runs) > 1:
        integrity = f"interleaved ({len(runs)} runs detected; using the latest)"
    elif meta_frames is not None and abs(len(frames) - meta_frames) > 2:
        integrity = f"suspect ({len(frames)} frames vs metadata.total_frames={meta_frames})"

    speeds = [f["speed"] for f in frames]
    moving = [s for s in speeds if s > 0.05]
    dist = sum(
        math.hypot(frames[i]["x"] - frames[i - 1]["x"], frames[i]["y"] - frames[i - 1]["y"])
        for i in range(1, len(frames))
    )
    t0, t1 = frames[0]["t"], frames[-1]["t"]
    span = t1 - t0
    n = len(frames)
    frac_lt = lambda th: sum(1 for s in speeds if s < th) / n

    bins = 10
    profile = []
    for b in range(bins):
        lo = t0 + span * b / bins
        hi = t0 + span * (b + 1) / bins
        seg = [f["speed"] for f in frames if lo <= f["t"] < hi] or [0.0]
        profile.append(sum(seg) / len(seg))

    end = frames[-1]
    max_progress = max(f["lap_progress"] for f in frames)
    max_lap = max(f["lap"] for f in frames)

    archive_dir = None
    if not args.no_archive:
        label = args.archive or datetime.now().strftime("run_%Y%m%d_%H%M%S")
        archive_dir = _archive(path.parent, label)

    print("================ TELEMETRY SUMMARY ================")
    print(f"source       : {path}")
    if archive_dir:
        print(f"archive      : {archive_dir}")
    print(f"integrity    : {integrity}"
          + (f"  (metadata.total_frames={meta_frames})" if meta_frames is not None else ""))
    print(f"frames/time  : {n} frames, t={t0:.2f}→{t1:.2f} ({span:.1f}s)")
    print(f"distance     : {dist:.1f} world units")
    print(f"final state  : x={end['x']:.2f}, y={end['y']:.2f}, speed={end['speed']:.3f}, status={end['status']}")
    print(f"lap/progress : lap={max_lap}, max_lap_progress={max_progress:.3f}")
    p95_speed = _pct(speeds, 0.95)
    rel_thresh = 0.15 * p95_speed
    rel_frac = sum(1 for s in speeds if s < rel_thresh) / n
    print("---- SPEED (supervisor units, not the normalized command) ----")
    print(f"  mean={sum(speeds)/n:.3f}  median={_pct(speeds,0.5):.3f}  "
          f"p95={p95_speed:.3f}  max={max(speeds):.3f}")
    print(f"  moving mean (>0.05)={sum(moving)/len(moving) if moving else 0:.3f}")
    print(f"  near-stop share: <0.3={frac_lt(0.3):.2f}  <0.05={frac_lt(0.05):.2f}   "
          f"relative crawl (<15% of p95={rel_thresh:.2f})={rel_frac:.2f}")
    print(f"  ten-bin mean-speed profile: {' '.join(f'{v:.2f}' for v in profile)}")
    crawl = _longest_low_speed(frames, 0.3)
    if crawl and crawl["dur"] > 1.0:
        print(f"  longest crawl (<0.3): {crawl['dur']:.1f}s  "
              f"t={crawl['t0']:.1f}→{crawl['t1']:.1f}  "
              f"x={crawl['x0']:.1f},y={crawl['y0']:.1f} → x={crawl['x1']:.1f},y={crawl['y1']:.1f}")
    print("---- EVENTS ----")
    if events:
        for t, ev in events[:20]:
            print(f"  t={t}: {ev}")
        if len(events) > 20:
            print(f"  ... {len(events)} events total")
    else:
        print("  (no checkpoint, collision, or finish events)")
    print("================================================")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
