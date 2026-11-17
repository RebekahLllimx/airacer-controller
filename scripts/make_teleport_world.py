"""Create a temporary Webots world with a car moved to a recorded pose."""

from __future__ import annotations

import argparse
import json
import os
import re
from pathlib import Path


SDK_DIR = Path(os.environ.get("AIRACER_SDK", "pkudsa.airacer/sdk"))
DEFAULT_TELEMETRY = SDK_DIR / ".local" / "recordings" / "telemetry.jsonl"
DEFAULT_WORLDS = {
    "basic": SDK_DIR / "webots" / "worlds" / "track_basic.wbt",
    "complex": SDK_DIR / "webots" / "worlds" / "track_complex.wbt",
}


def _load_pose(telemetry: Path, target_time: float, team_id: str | None = None) -> dict:
    """Return the recorded vehicle pose nearest ``target_time``."""

    best: dict | None = None
    best_gap = float("inf")
    with telemetry.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            cars = row.get("cars") or []
            for car in cars:
                if team_id and car.get("team_id") != team_id:
                    continue
                try:
                    sim_time = float(row["t"])
                    pose = {
                        "t": sim_time,
                        "x": float(car["x"]),
                        "y": float(car["y"]),
                        "heading": float(car["heading"]),
                        "speed": float(car.get("speed", 0.0)),
                        "status": car.get("status", ""),
                    }
                except (KeyError, TypeError, ValueError):
                    continue
                gap = abs(sim_time - target_time)
                if gap < best_gap:
                    best = pose
                    best_gap = gap
                break
    if best is None:
        raise ValueError(f"no vehicle pose found in telemetry: {telemetry}")
    return best


def _replace_field(block: str, field: str, value: str) -> str:
    """Replace or insert a single-line field in a Webots node block."""

    pattern = re.compile(rf"(?m)^(\s*){re.escape(field)}\s+.+$")
    match = pattern.search(block)
    if match:
        indent = match.group(1)
        return pattern.sub(f"{indent}{field} {value}", block, count=1)
    open_brace = block.find("{")
    if open_brace < 0:
        raise ValueError("vehicle node has no opening brace")
    insert_at = block.find("\n", open_brace)
    if insert_at < 0:
        raise ValueError("vehicle node is not multiline")
    return block[: insert_at + 1] + f"  {field} {value}\n" + block[insert_at + 1 :]


def _find_def_block(source: str, def_name: str) -> tuple[int, int, str]:
    """Locate a named DEF node and its balanced brace block."""

    match = re.search(rf"\bDEF\s+{re.escape(def_name)}\s+\w+\s*\{{", source)
    if not match:
        raise ValueError(f"DEF {def_name} not found in world")
    start = match.start()
    index = source.find("{", match.start())
    depth = 0
    for pos in range(index, len(source)):
        char = source[pos]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return start, pos + 1, source[start : pos + 1]
    raise ValueError(f"DEF {def_name} has unbalanced braces")


def build_teleport_world(source_world: Path, output_world: Path, car_slot: str, pose: dict, z: float) -> None:
    """Write a temporary world with one vehicle translated and rotated."""

    source = source_world.read_text(encoding="utf-8")
    start, end, block = _find_def_block(source, car_slot)
    heading = float(pose["heading"])
    block = _replace_field(block, "translation", f"{pose['x']:.6f} {pose['y']:.6f} {z:.6f}")
    block = _replace_field(block, "rotation", f"0 0 1 {-heading:.9f}")
    output_world.parent.mkdir(parents=True, exist_ok=True)
    output_world.write_text(source[:start] + block + source[end:], encoding="utf-8")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""

    parser = argparse.ArgumentParser(
        description="Create a temporary Webots world from telemetry or an explicit pose.")
    parser.add_argument("--world", default="complex", help="basic, complex, or a .wbt path")
    parser.add_argument("--telemetry", type=Path, default=DEFAULT_TELEMETRY)
    parser.add_argument("--time", type=float, default=None, help="Simulation time to retrieve from telemetry")
    parser.add_argument("--team-id", default=None, help="Optional telemetry team ID")
    parser.add_argument("--x", type=float, default=None, help="Explicit world x; requires y and heading")
    parser.add_argument("--y", type=float, default=None, help="Explicit world y")
    parser.add_argument("--heading", type=float, default=None, help="Explicit telemetry-frame heading in radians")
    parser.add_argument("--car-slot", default="car_1")
    parser.add_argument("--z", type=float, default=0.4)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--pose-out", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    """Create the requested temporary world."""

    args = parse_args()
    source_world = DEFAULT_WORLDS.get(args.world, Path(args.world).expanduser())
    source_world = source_world.resolve()
    output_world = args.out.expanduser().resolve()

    explicit = args.x is not None or args.y is not None or args.heading is not None
    if explicit:
        if None in (args.x, args.y, args.heading):
            print("[error] explicit pose requires --x, --y, and --heading")
            return 2
        pose = {"t": 0.0, "x": float(args.x), "y": float(args.y),
                "heading": float(args.heading), "speed": 0.0, "status": "manual"}
    elif args.time is not None:
        telemetry = args.telemetry.expanduser().resolve()
        pose = _load_pose(telemetry, args.time, team_id=args.team_id)
    else:
        print("[error] provide --time or an explicit --x/--y/--heading pose")
        return 2

    build_teleport_world(source_world, output_world, args.car_slot, pose, args.z)
    if args.pose_out:
        args.pose_out.parent.mkdir(parents=True, exist_ok=True)
        args.pose_out.write_text(json.dumps(pose, ensure_ascii=False, indent=2), encoding="utf-8")
    print(
        f"teleport world: {output_world} "
        f"(source={source_world.name}, t={pose['t']:.3f}, x={pose['x']:.3f}, "
        f"y={pose['y']:.3f}, heading={pose['heading']:.4f}, speed={pose['speed']:.2f})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
