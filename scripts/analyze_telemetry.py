"""Telemetry 分析工具。

功能概述：从 Webots 仿真产生的 telemetry.jsonl 提取赛道表现数据。
输入输出：读取 telemetry.jsonl 和可选的 metadata.json，输出圈速、速度统计和轨迹摘要。
处理流程：自动检测起跑线位置，通过轨迹穿线检测识别圈数完成。

用法：
    python scripts/analyze_telemetry.py [--dir recordings_path] [--json]
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from collections import defaultdict
from typing import Optional

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_RECORDINGS = ROOT / "pkudsa.airacer" / "sdk" / ".local" / "recordings"


def load_telemetry(rec_dir: pathlib.Path) -> list[dict]:
    """加载 telemetry.jsonl 行。"""
    path = rec_dir / "telemetry.jsonl"
    if not path.is_file():
        raise FileNotFoundError(f"telemetry.jsonl not found: {path}")
    frames = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                frames.append(json.loads(line))
    return frames


def load_metadata(rec_dir: pathlib.Path) -> dict:
    """加载 metadata.json。"""
    path = rec_dir / "metadata.json"
    if path.is_file():
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    return {}


def detect_laps_from_trajectory(frames: list[dict], car_index: int = 0) -> dict:
    """从位置轨迹自动检测每圈完成。

    算法：找到车辆 spawn 位置，在其 y 坐标附近建立穿线检测区域。
    当车辆从下方穿越 spawn_y 线（由下向上）时计为一圈完成。

    返回包含 lap_times、estimated_laps、speed_stats 的 dict。
    """
    if not frames:
        return {"lap_times": [], "estimated_laps": 0, "speed_stats": {}}

    # 提取轨迹
    xs, ys, speeds, times = [], [], [], []
    for frame in frames:
        cars = frame.get("cars", [])
        if car_index < len(cars):
            c = cars[car_index]
            xs.append(c["x"])
            ys.append(c["y"])
            speeds.append(c["speed"])
            times.append(frame["t"])

    if len(times) < 10:
        return {"lap_times": [], "estimated_laps": 0, "speed_stats": {}}

    # 自动检测 spawn 位置和起跑线
    spawn_x, spawn_y = xs[0], ys[0]

    # 找到车辆前进主方向（前 20 帧的平均位移方向）
    n_preview = min(20, len(xs) - 1)
    dy_sum = sum(ys[i + 1] - ys[i] for i in range(n_preview))
    forward_sign = 1 if dy_sum > 0 else -1  # +1 = 向北出发, -1 = 向南出发

    # 在 spawn_y 附近建立穿线检测区域
    # line_y 设为 spawn_y + 偏移（确保初始位置在线的"后方"）
    line_margin = 2.0
    line_y = spawn_y + forward_sign * line_margin

    # 穿线检测状态机
    lap_times = []
    lap_start_time = None
    crossed_above = forward_sign > 0  # 需要先穿到线的另一侧

    # 如果向北出发 (forward_sign > 0):
    #   - 初始在 spawn_y（线下方）
    #   - 需要先去线上方 (y > line_y)，再回来穿线 (y < line_y)
    # 如果向南出发 (forward_sign < 0):
    #   - 初始在 spawn_y（线上方）
    #   - 需要先去线下方 (y < line_y)，再回来穿线 (y > line_y)

    # 简化：检测 y 从 line_y 的一侧穿到另一侧
    was_far_side = False  # 是否已经到达过线的远侧
    last_side = None

    for i in range(len(times)):
        current_side = "far" if (ys[i] - line_y) * forward_sign > 0 else "near"

        if last_side is not None:
            # 检测穿线：从 far 侧穿回 near 侧
            if last_side == "far" and current_side == "near" and was_far_side:
                if lap_start_time is not None:
                    lap_time = times[i] - lap_start_time
                    if lap_time > 5.0:  # 过滤假穿线（至少 5 秒一圈）
                        lap_times.append(lap_time)
                    lap_start_time = times[i]
                else:
                    # 第一次穿线 = 比赛开始
                    lap_start_time = times[i]

            if current_side == "far":
                was_far_side = True

        # 如果 lap_start_time 仍未设置（尚未穿线），用第一次远侧作为起点
        if lap_start_time is None and current_side == "far":
            lap_start_time = times[i]
            was_far_side = True

        last_side = current_side

    # 速度统计
    if speeds:
        speeds_sorted = sorted(speeds)
        speed_stats = {
            "max_m_s": round(max(speeds), 2),
            "mean_m_s": round(sum(speeds) / len(speeds), 2),
            "median_m_s": round(speeds_sorted[len(speeds_sorted) // 2], 2),
            "p90_m_s": round(speeds_sorted[int(len(speeds_sorted) * 0.9)], 2),
        }
    else:
        speed_stats = {}

    return {
        "lap_times": [round(t, 3) for t in lap_times],
        "estimated_laps": len(lap_times),
        "speed_stats": speed_stats,
        "total_sim_time": round(times[-1], 1) if times else 0,
        "spawn": (round(spawn_x, 2), round(spawn_y, 2)),
        "detection_line_y": round(line_y, 2),
    }


def analyze_telemetry(rec_dir: pathlib.Path) -> dict:
    """分析一次仿真的完整遥测数据。"""
    frames = load_telemetry(rec_dir)
    metadata = load_metadata(rec_dir)

    n_cars = len(frames[0]["cars"]) if frames else 0
    cars_analysis = []
    for i in range(n_cars):
        team_id = frames[0]["cars"][i].get("team_id", f"car_{i}") if frames else f"car_{i}"
        result = detect_laps_from_trajectory(frames, car_index=i)
        result["team_id"] = team_id

        # 尝试从官方事件中补充数据
        official_laps = 0
        official_best = None
        for frame in frames:
            for event in frame.get("events", []):
                if event.get("type") == "lap_complete" and event.get("team_id") == team_id:
                    official_laps += 1
                    lt = event.get("lap_time")
                    if lt is not None and (official_best is None or lt < official_best):
                        official_best = lt
        result["official_laps"] = official_laps
        result["official_best_lap"] = official_best
        cars_analysis.append(result)

    # 碰撞统计
    collisions = []
    for frame in frames:
        for event in frame.get("events", []):
            if event.get("type") == "collision":
                collisions.append(event)

    return {
        "session_id": metadata.get("session_id", ""),
        "session_type": metadata.get("session_type", ""),
        "total_laps_config": metadata.get("total_laps", 0),
        "finish_reason": metadata.get("finish_reason", ""),
        "duration_sim": metadata.get("duration_sim", 0),
        "cars": cars_analysis,
        "collision_summary": {
            "total": len(collisions),
            "major": sum(1 for c in collisions if c.get("severity") == "major"),
            "minor": sum(1 for c in collisions if c.get("severity") == "minor"),
        },
        "official_rankings": metadata.get("final_rankings", []),
    }


def print_report(analysis: dict):
    """打印人类可读的分析报告。"""
    print(f"\n{'='*60}")
    print(f"遥测分析报告")
    print(f"{'='*60}")
    print(f"Session:     {analysis['session_id']}")
    print(f"类型:        {analysis['session_type']}")
    print(f"配置圈数:    {analysis['total_laps_config']}")
    print(f"仿真时长:    {analysis['duration_sim']:.1f}s")
    print(f"结束原因:    {analysis['finish_reason']}")

    print(f"\n── 碰撞 ──")
    cs = analysis["collision_summary"]
    print(f"  总计: {cs['total']} (严重: {cs['major']}, 轻微: {cs['minor']})")

    for car in analysis["cars"]:
        print(f"\n── {car['team_id']} ──")
        print(f"  检测圈数 (轨迹):  {car['estimated_laps']}")
        print(f"  官方圈数 (事件):  {car['official_laps']}")
        if car["lap_times"]:
            print(f"  圈速:             {car['lap_times']}")
            print(f"  最快圈:           {min(car['lap_times']):.1f}s")
        if car.get("official_best_lap"):
            print(f"  官方最快圈:       {car['official_best_lap']:.1f}s")
        ss = car.get("speed_stats", {})
        if ss:
            print(f"  速度 max/mean/median/p90: {ss.get('max_m_s')}/{ss.get('mean_m_s')}/{ss.get('median_m_s')}/{ss.get('p90_m_s')} m/s")
        print(f"  生成位置:         {car.get('spawn')}")
        print(f"  检测线 y:         {car.get('detection_line_y')}")

    if analysis["official_rankings"]:
        print(f"\n── 官方排名 ──")
        for r in analysis["official_rankings"]:
            print(f"  #{r['rank']} {r['team_id']}: laps={r['laps']} best={r['best_lap']} total={r['total_time']}")

    print()


def parse_args():
    parser = argparse.ArgumentParser(description="Analyze AI Racer telemetry data")
    parser.add_argument("--dir", type=pathlib.Path, default=None,
                        help="recordings 目录路径（默认 SDK .local/recordings）")
    parser.add_argument("--json", action="store_true",
                        help="以 JSON 格式输出")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rec_dir = args.dir or DEFAULT_RECORDINGS

    try:
        analysis = analyze_telemetry(rec_dir)
    except FileNotFoundError as e:
        print(f"错误: {e}", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(analysis, ensure_ascii=False, indent=2))
    else:
        print_report(analysis)
    return 0


if __name__ == "__main__":
    sys.exit(main())
