"""Replay recorded stereo frames through the controller in open loop.

The output follows the controller debug-log schema, allowing perception and
estimation revisions to be compared on identical images. Open-loop replay does
not measure driving performance because control commands cannot change later
frames.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.common import clamp_cmd
from controller.estimator import estimate_track, reset_estimator_state
from controller.params import get_profile
from controller.perception import extract_observation
from controller.policy import decide_control, reset_policy_state
import controller.policy as policy_state


def _parse_frame_timestamp(left_path: Path) -> float:
    """Decode a timestamp such as 12.345 from a saved left-frame filename."""

    stem = left_path.stem
    if not stem.startswith("frame_") or not stem.endswith("_left"):
        raise ValueError(f"cannot parse left-frame filename: {left_path.name}")
    token = stem[len("frame_"):-len("_left")]
    if token.startswith("m"):
        token = "-" + token[1:]
    if "_" in token:
        head, tail = token.split("_", 1)
        token = f"{head}.{tail}"
    return float(token)


def iter_frame_pairs(frame_dir: Path) -> list[tuple[float, Path, Path]]:
    """Return complete left/right frame pairs sorted by timestamp."""

    pairs: list[tuple[float, Path, Path]] = []
    for left_path in sorted(frame_dir.glob("frame_*_left.png")):
        right_path = left_path.with_name(left_path.name.replace("_left.png", "_right.png"))
        if not right_path.is_file():
            continue
        pairs.append((_parse_frame_timestamp(left_path), left_path, right_path))
    pairs.sort(key=lambda item: item[0])
    return pairs


def replay_frames(frame_dir: Path, out_path: Path, mode: str = "no_other_cars", limit: int | None = None) -> int:
    """Replay frames and write JSONL compatible with ``analyze_control_log.py``."""

    pairs = iter_frame_pairs(frame_dir)
    if limit is not None:
        pairs = pairs[:max(limit, 0)]
    out_path.parent.mkdir(parents=True, exist_ok=True)
    reset_estimator_state()
    reset_policy_state()
    profile = get_profile(mode)


    written = 0
    with out_path.open("w", encoding="utf-8") as handle:
        for timestamp, left_path, right_path in pairs:
            try:
                left_img = cv2.imread(str(left_path))
                right_img = cv2.imread(str(right_path))
                obs = extract_observation(left_img, right_img, timestamp, profile=profile)
                track = estimate_track(obs, timestamp)
                cmd = decide_control(track, timestamp, mode=mode)
                steering, speed = clamp_cmd(cmd)
                row = {
                    "t": float(timestamp),
                    "steering": round(float(steering), 4),
                    "speed": round(float(speed), 4),
                    "lateral": round(float(track.lateral_error), 4),
                    "heading": round(float(track.heading_error), 4),
                    "curvature": round(float(track.curvature), 4),
                    "lookahead": round(float(track.lookahead_error), 4),
                    "track_conf": round(float(track.confidence), 4),
                    "lost": bool(track.lost),
                    "red_env": bool(track.red_environment),
                    "mode": policy_state._LAST_MODE,
                    "mode_reason": policy_state._LAST_MODE_REASON,
                    "target_steering": round(float(policy_state._LAST_TARGET_STEERING), 4),
                    "target_speed": round(float(policy_state._LAST_TARGET_SPEED), 4),
                    "curve_risk": round(float(policy_state._LAST_SIGNALS.get("curve_risk", 0.0)), 4),
                    "offset_risk": round(float(policy_state._LAST_SIGNALS.get("offset_risk", 0.0)), 4),
                    "margin_risk": round(float(policy_state._LAST_SIGNALS.get("margin_risk", 0.0)), 4),
                    "straight_memory": bool(policy_state._LAST_STRAIGHT_MEMORY_ACTIVE),
                    "obs_conf": round(float(obs.confidence), 4),
                    "obs_points": int(len(obs.center_points)),
                    "road_width": round(float(obs.road_width_est), 2),
                    "debug_flags": int(obs.debug_flags),
                    "line_offset": round(float(track.line_offset), 4),
                    "line_heading": round(float(track.line_heading), 4),
                    "line_conf": round(float(track.line_confidence), 4),
                    "left_margin": round(float(track.left_margin_near), 4),
                    "right_margin": round(float(track.right_margin_near), 4),
                    "near_obstacle": bool(track.near_obstacle),
                    "obstacle_x": round(float(track.obstacle_x), 4),
                    "obstacle_size": round(float(track.obstacle_size), 5),
                }
            except Exception:
                row = {
                    "t": float(timestamp),
                    "steering": 0.0,
                    "speed": 0.0,
                    "lateral": 0.0,
                    "heading": 0.0,
                    "curvature": 0.0,
                    "lookahead": 0.0,
                    "track_conf": 0.0,
                    "lost": True,
                    "red_env": False,
                    "mode": "exception",
                    "mode_reason": "exception",
                    "target_steering": 0.0,
                    "target_speed": 0.0,
                    "curve_risk": 0.0,
                    "offset_risk": 0.0,
                    "margin_risk": 0.0,
                    "straight_memory": False,
                    "obs_conf": 0.0,
                    "obs_points": 0,
                    "road_width": 0.0,
                    "debug_flags": 0,
                    "line_offset": 0.0,
                    "line_heading": 0.0,
                    "line_conf": 0.0,
                    "left_margin": 1.0,
                    "right_margin": 1.0,
                    "near_obstacle": False,
                    "obstacle_x": 0.0,
                    "obstacle_size": 0.0,
                }
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
            written += 1
    return written


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(description="Open-loop replay of saved stereo camera frames.")
    parser.add_argument("frames", type=Path, help="Directory containing frame_<t>_left/right.png")
    parser.add_argument("--out", type=Path, required=True, help="Output control-schema JSONL")
    parser.add_argument("--mode", choices=("no_other_cars", "with_other_cars"), default="no_other_cars")
    parser.add_argument("--limit", type=int, default=None, help="Maximum frames to replay")
    return parser.parse_args()


def main() -> int:
    """Run the command-line replay."""

    args = parse_args()
    if not args.frames.is_dir():
        print(f"[error] frame directory not found: {args.frames}")
        return 1
    written = replay_frames(args.frames, args.out, mode=args.mode, limit=args.limit)
    if written == 0:
        print(f"[error] no complete stereo pairs found: {args.frames}")
        return 1
    print(f"replayed {written} frames; output: {args.out}")
    print("Open-loop replay compares perception and estimation only; it does not measure lap performance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
