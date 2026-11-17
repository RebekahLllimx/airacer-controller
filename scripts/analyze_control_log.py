"""Summarize frame-level controller logs into steering and speed diagnostics.

The report covers steering reversals, lateral bias, driving modes, perception
loss, and the relationship between steering demand and commanded speed.
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

DEBUG_FLAG_BITS = [
    (1, "too few valid scan lines"),
    (2, "edge fallback used"),
    (4, "extreme mask fill ratio"),
    (8, "left/right near-centre mismatch"),
    (16, "similar left/right confidence"),
    (32, "red environment"),
]


def _load(path: Path) -> list[dict]:
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _pct(values: list[float], q: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    idx = min(len(ordered) - 1, max(0, int(round(q * (len(ordered) - 1)))))
    return ordered[idx]


def _split_runs(rows: list[dict]) -> list[list[dict]]:
    """Split an appended log whenever its timestamp moves backward."""

    segments: list[list[dict]] = []
    current: list[dict] = []
    last_t = None
    for row in rows:
        try:
            t = float(row.get("t", 0.0))
        except (TypeError, ValueError):
            t = 0.0
        if last_t is not None and t < last_t - 1e-6:
            if current:
                segments.append(current)
            current = []
        current.append(row)
        last_t = t
    if current:
        segments.append(current)
    return segments


def decode_debug_flags(flags: int) -> list[str]:
    """Decode perception and estimator diagnostic flags."""

    labels = [label for bit, label in DEBUG_FLAG_BITS if flags & bit]
    known_mask = 0
    for bit, _label in DEBUG_FLAG_BITS:
        known_mask |= bit
    unknown = flags & ~known_mask
    bit = 1
    while unknown:
        if unknown & bit:
            labels.append(f"unknown bit {bit}")
            unknown &= ~bit
        bit <<= 1
    return labels


def _numeric(rows: list[dict], field: str, absolute: bool = False) -> list[float]:
    values = []
    for row in rows:
        try:
            value = float(row.get(field, 0.0))
        except (TypeError, ValueError):
            value = 0.0
        values.append(abs(value) if absolute else value)
    return values


def _value_stats(rows: list[dict], field: str) -> dict[str, float]:
    values = _numeric(rows, field)
    return {
        "mean": _mean(values),
        "median": _pct(values, 0.5),
        "p10": _pct(values, 0.1),
        "p90": _pct(values, 0.9),
    }


def _lost_segments(rows: list[dict]) -> list[int]:
    """Return the frame count of each contiguous perception-loss episode."""

    segments: list[int] = []
    current = 0
    for row in rows:
        if row.get("lost"):
            current += 1
        elif current:
            segments.append(current)
            current = 0
    if current:
        segments.append(current)
    return segments


def collect_lost_diagnostics(rows: list[dict]) -> dict:
    """Aggregate perception-loss duration, geometry, and diagnostic flags."""

    n = len(rows)
    lost_rows = [row for row in rows if row.get("lost")]
    non_lost_rows = [row for row in rows if not row.get("lost")]
    segments = _lost_segments(rows)

    flag_counter: Counter[int] = Counter()
    for row in lost_rows:
        flags = int(row.get("debug_flags", 0) or 0)
        for bit, _label in DEBUG_FLAG_BITS:
            if flags & bit:
                flag_counter[bit] += 1
        unknown = flags & ~sum(bit for bit, _label in DEBUG_FLAG_BITS)
        bit = 1
        while unknown:
            if unknown & bit:
                flag_counter[bit] += 1
                unknown &= ~bit
            bit <<= 1

    entry_prev_modes: Counter[str] = Counter()
    for index, row in enumerate(rows):
        if not row.get("lost"):
            continue
        if index > 0 and rows[index - 1].get("lost"):
            continue
        prev_mode = "<start>" if index == 0 else str(rows[index - 1].get("mode", "?"))
        entry_prev_modes[prev_mode] += 1

    return {
        "frames": n,
        "lost_frames": len(lost_rows),
        "lost_frac": len(lost_rows) / n if n else 0.0,
        "segments": segments,
        "segment_stats": {
            "count": len(segments),
            "max": max(segments) if segments else 0,
            "mean": _mean([float(v) for v in segments]),
            "median": _pct([float(v) for v in segments], 0.5),
            "p90": _pct([float(v) for v in segments], 0.9),
        },
        "lost_value_stats": {
            field: _value_stats(lost_rows, field)
            for field in ("track_conf", "obs_conf", "obs_points", "road_width")
        },
        "flag_counter": flag_counter,
        "entry_prev_modes": entry_prev_modes,
        "curvature_abs_mean": {
            "lost": _mean(_numeric(lost_rows, "curvature", absolute=True)),
            "non_lost": _mean(_numeric(non_lost_rows, "curvature", absolute=True)),
        },
        "heading_abs_mean": {
            "lost": _mean(_numeric(lost_rows, "heading", absolute=True)),
            "non_lost": _mean(_numeric(non_lost_rows, "heading", absolute=True)),
        },
    }


def print_lost_diagnostics(rows: list[dict]) -> None:
    """Print a compact perception-loss section."""

    diag = collect_lost_diagnostics(rows)
    lost_n = diag["lost_frames"]
    seg = diag["segment_stats"]
    print("---- PERCEPTION LOSS ----")
    print(f"  lost frames={lost_n}/{diag['frames']} ({diag['lost_frac']:.2f})  "
          f"episodes={seg['count']}  longest={seg['max']} frames  "
          f"length median={seg['median']:.0f} p90={seg['p90']:.0f} mean={seg['mean']:.1f}")
    if not lost_n:
        print("  (no lost frames)")
        return
    for field, label in (
        ("track_conf", "track_conf"),
        ("obs_conf", "obs_conf"),
        ("obs_points", "obs_points"),
        ("road_width", "road_width"),
    ):
        stats = diag["lost_value_stats"][field]
        print(f"  lost {label}: mean={stats['mean']:.3f} median={stats['median']:.3f} "
              f"p10={stats['p10']:.3f} p90={stats['p90']:.3f}")
    print("  debug_flags: " + "  ".join(
        f"{decode_debug_flags(bit)[0]}={count/lost_n:.2f}({count})"
        for bit, count in diag["flag_counter"].most_common()
    ))
    print("  mode before loss: " + "  ".join(
        f"{mode}={count/seg['count']:.2f}({count})"
        for mode, count in diag["entry_prev_modes"].most_common()
    ))
    print(f"  |curvature| mean: lost={diag['curvature_abs_mean']['lost']:.3f}  "
          f"non_lost={diag['curvature_abs_mean']['non_lost']:.3f}")
    print(f"  |heading|   mean: lost={diag['heading_abs_mean']['lost']:.3f}  "
          f"non_lost={diag['heading_abs_mean']['non_lost']:.3f}")


def _print_summary(path: Path, rows: list[dict], integrity: str | None = None) -> None:
    """Print the summary for one monotonic run segment."""

    n = len(rows)
    t0, t1 = rows[0].get("t", 0.0), rows[-1].get("t", 0.0)
    span = max(t1 - t0, 1e-6)

    steering = [float(r.get("steering", 0.0)) for r in rows]
    speed = [float(r.get("speed", 0.0)) for r in rows]
    lateral = [float(r.get("lateral", 0.0)) for r in rows]
    heading = [float(r.get("heading", 0.0)) for r in rows]
    curvature = [float(r.get("curvature", 0.0)) for r in rows]

    sign_changes = 0
    abs_dsteer_sum = 0.0
    for i in range(1, n):
        a, b = steering[i - 1], steering[i]
        if a * b < 0 and abs(a) > 0.05 and abs(b) > 0.05:
            sign_changes += 1
        abs_dsteer_sum += abs(b - a)
    osc_per_sec = sign_changes / span
    dsteer_per_sec = abs_dsteer_sum / span

    frac = lambda vals, pred: sum(1 for v in vals if pred(v)) / n
    lost_frac = sum(1 for r in rows if r.get("lost")) / n
    modes = Counter(r.get("mode", "?") for r in rows)

    sp_low_steer = [speed[i] for i in range(n) if abs(steering[i]) < 0.1]
    sp_high_steer = [speed[i] for i in range(n) if abs(steering[i]) > 0.3]
    sp_low_lat = [speed[i] for i in range(n) if abs(lateral[i]) < 0.1]
    sp_high_lat = [speed[i] for i in range(n) if abs(lateral[i]) > 0.3]

    print("================ CONTROL LOG SUMMARY ================")
    print(f"source       : {path}")
    if integrity:
        print(f"integrity    : {integrity}")
    print(f"frames/time  : {n} frames, t={t0:.2f}→{t1:.2f} ({span:.1f}s)")
    print("---- STEERING ----")
    print(f"  mean(signed)={_mean(steering):+.3f}  mean|steer|={_mean([abs(s) for s in steering]):.3f}  "
          f"p95|steer|={_pct([abs(s) for s in steering],0.95):.3f}")
    print(f"  reversals={osc_per_sec:.2f}/s   cumulative |Δsteer|={dsteer_per_sec:.2f}/s")
    print("---- SPEED ----")
    print(f"  mean={_mean(speed):.3f}  median={_pct(speed,0.5):.3f}  p95={_pct(speed,0.95):.3f}  "
          f"max={max(speed):.3f}")
    print(f"  low-speed share: <0.5={frac(speed, lambda v: v<0.5):.2f}  <0.3={frac(speed, lambda v: v<0.3):.2f}")
    print("---- LATERAL BIAS ----")
    print(f"  signed mean={_mean(lateral):+.3f}  (left negative, right positive)")
    print(f"  mean|lat|={_mean([abs(v) for v in lateral]):.3f}  "
          f"share |lat|>0.3={frac(lateral, lambda v: abs(v)>0.3):.2f}")
    print(f"  mean|heading|={_mean([abs(v) for v in heading]):.3f}  "
          f"mean|curv|={_mean([abs(v) for v in curvature]):.3f}")
    print("---- STATE ----")
    print(f"  lost share={lost_frac:.2f}")
    print("  mode share: " + "  ".join(f"{m}={c/n:.2f}" for m, c in modes.most_common()))
    print_lost_diagnostics(rows)
    print("---- SPEED / STEERING RELATIONSHIP ----")
    print(f"  mean speed at |steer|<0.1={_mean(sp_low_steer):.3f}   |steer|>0.3={_mean(sp_high_steer):.3f}")
    print(f"  mean speed at |lat|<0.1={_mean(sp_low_lat):.3f}   |lat|>0.3={_mean(sp_high_lat):.3f}")
    print("=================================================")


def _expand_inputs(paths: list[Path]) -> tuple[list[Path], list[str]]:
    """Expand command-line paths into an ordered list of JSONL logs."""

    files: list[Path] = []
    errors: list[str] = []
    for path in paths:
        if path.is_dir():
            found = sorted(p for p in path.glob("control*.jsonl") if p.is_file())
            if not found:
                found = sorted(p for p in path.glob("*.jsonl") if p.is_file())
            if not found:
                errors.append(f"[error] directory contains no JSONL logs: {path}")
            files.extend(found)
        elif path.is_file():
            files.append(path)
        else:
            errors.append(f"[error] log not found: {path}")
    return files, errors


def analyze_file(path: Path) -> int:
    """Read and summarize one control log."""

    rows = _load(path)
    if not rows:
        print(f"[error] log is empty or cannot be parsed: {path}")
        return 1

    runs = _split_runs(rows)
    active_rows = runs[-1]
    integrity = None
    if len(runs) > 1:
        integrity = f"interleaved ({len(runs)} runs detected; using the latest)"
    _print_summary(path, active_rows, integrity=integrity)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize controller debug logs.")
    parser.add_argument("logs", nargs="+", type=Path, help="One or more control JSONL files or directories")
    args = parser.parse_args()

    files, errors = _expand_inputs(args.logs)
    for error in errors:
        print(error)
    if not files:
        return 1

    status = 0
    for index, path in enumerate(files):
        if index:
            print()
        status = max(status, analyze_file(path))
    return status


if __name__ == "__main__":
    raise SystemExit(main())
