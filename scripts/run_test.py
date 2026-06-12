"""自动化测试与结果记录脚本。

功能概述：构建 submissions、运行校验、收集性能指标并写入实验记录。
输入输出：接受模式选择和输出格式参数，向 experiments/runs.csv 追加结构化结果。
处理流程：构建 → 本地校验 → SDK 校验 → 性能采集 → 可选 Webots 仿真 → 记录 CSV。

用法:
    # 快速测试（仅校验和性能）
    python scripts/run_test.py --mode fastest

    # 完整测试（含 Webots 仿真，需要 Webots 已安装）
    python scripts/run_test.py --mode fastest --track basic --sim

    # 只测试 safe 模式
    python scripts/run_test.py --mode safe

    # 同时测试两种模式并记录对比
    python scripts/run_test.py --mode both

    # 列出可用赛道
    python scripts/run_test.py --list-tracks
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from typing import Optional

ROOT = pathlib.Path(__file__).resolve().parents[1]
SDK_DIR = ROOT / "pkudsa.airacer" / "sdk"
SUBMISSIONS_DIR = ROOT / "submissions"
EXPERIMENTS_CSV = ROOT / "experiments" / "runs.csv"
BUILD_SCRIPT = ROOT / "scripts" / "build_submission.py"
VALIDATE_SCRIPT = ROOT / "scripts" / "validate_submission.py"


# ── 工具函数 ────────────────────────────────────────────────────────────

def current_commit() -> str:
    """获取当前 git commit 短哈希。"""
    try:
        result = subprocess.run(
            ["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, check=True,
        )
        return result.stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unknown"


def now_iso() -> str:
    """当前 UTC 时间的 ISO 格式字符串。"""
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def run_cmd(cmd: list[str], timeout: int = 120) -> tuple[int, str, str]:
    """运行命令，返回 (退出码, stdout, stderr)。"""
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, cwd=str(ROOT))
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return -1, "", "timeout"
    except FileNotFoundError:
        return -1, "", "command not found"


# ── 校验步骤 ────────────────────────────────────────────────────────────

def step_build(mode: str) -> pathlib.Path:
    """构建 submission 并返回生成文件路径。"""
    out_path = SUBMISSIONS_DIR / mode / "team_controller.py"
    rc, stdout, stderr = run_cmd([sys.executable, str(BUILD_SCRIPT), "--mode", mode, "--out", str(out_path)])
    if rc != 0:
        print(f"  [FAIL] 构建失败: {stderr}")
        raise RuntimeError(f"Build failed for {mode}")
    print(f"  [OK] 构建完成: {out_path}")
    return out_path


def step_local_validate(path: pathlib.Path, mode: str) -> bool:
    """运行本地校验。"""
    print(f"\n── 本地校验 ({mode}) ──")
    rc, stdout, stderr = run_cmd([sys.executable, str(VALIDATE_SCRIPT), str(path)])
    passed = rc == 0
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] 本地校验: {stdout.strip()}")
    if stderr.strip():
        print(f"  stderr: {stderr.strip()}")
    return passed


def step_sdk_validate(path: pathlib.Path, mode: str) -> tuple[bool, dict]:
    """运行 SDK 校验器并返回性能数据。"""
    print(f"\n── SDK 校验 ({mode}) ──")
    sdk_validator = SDK_DIR / "validate_controller.py"
    rules = SDK_DIR / "rules.yaml"

    if not sdk_validator.is_file():
        print("  [SKIP] SDK validator 不可用")
        return True, {}

    cmd = [
        sys.executable, str(sdk_validator),
        "--code-path", str(path),
        "--json",
    ]
    if rules.is_file():
        cmd += ["--rules", str(rules)]

    rc, stdout, stderr = run_cmd(cmd)
    meta = {}
    try:
        report = json.loads(stdout)
        meta = report.get("meta", {})
        if report.get("passed"):
            print(f"  [PASS] SDK 校验通过")
        else:
            print(f"  [WARN] SDK 校验有警告/错误")
            for err in report.get("errors", []):
                print(f"    Error: {err.get('code', '?')}: {err.get('message', '?')}")
        perf = []
        if meta.get("avg_call_ms") is not None:
            perf.append(f"avg={meta['avg_call_ms']}ms")
        if meta.get("p95_call_ms") is not None:
            perf.append(f"p95={meta['p95_call_ms']}ms")
        if perf:
            print(f"  性能: {', '.join(perf)}")
    except json.JSONDecodeError:
        print(f"  [WARN] SDK 输出解析失败")
        if stdout:
            print(f"  stdout: {stdout[:200]}")

    return rc == 0, meta


def step_pytest() -> bool:
    """运行项目测试套件。"""
    print(f"\n── pytest ──")
    rc, stdout, stderr = run_cmd([sys.executable, "-m", "pytest", str(ROOT / "tests"), "-q"])
    passed = rc == 0
    status = "PASS" if passed else "FAIL"
    print(f"  [{status}] pytest")
    if not passed:
        # 显示最后几行输出
        for line in stdout.strip().splitlines()[-5:]:
            print(f"  {line}")
    return passed


# ── Webots 仿真（可选） ────────────────────────────────────────────────

RECORDINGS_DIR = SDK_DIR / ".local" / "recordings"
TELEMETRY_PATH = RECORDINGS_DIR / "telemetry.jsonl"
STOP_PATH = RECORDINGS_DIR / "STOP"
META_PATH = RECORDINGS_DIR / "metadata.json"


def find_webots() -> Optional[str]:
    """查找 Webots 可执行文件路径。"""
    env = os.environ.get("WEBOTS_HOME")
    if env:
        candidates = [
            pathlib.Path(env) / "Contents" / "MacOS" / "webots",
            pathlib.Path(env) / "webots",
        ]
        for c in candidates:
            if c.is_file():
                return str(c)

    import shutil
    found = shutil.which("webots")
    if found:
        return found

    default = "/Applications/Webots.app/Contents/MacOS/webots"
    if pathlib.Path(default).is_file():
        return default

    return None


def _clean_recordings():
    """清理上一次仿真残留的遥测文件。"""
    for path in (TELEMETRY_PATH, META_PATH, STOP_PATH):
        try:
            path.unlink(missing_ok=True)
        except Exception:
            pass


def _kill_webots():
    """强制关闭所有 Webots 进程。"""
    try:
        subprocess.run(["pkill", "-f", "webots"], capture_output=True, timeout=5)
    except Exception:
        pass


def _read_telemetry_summary() -> dict:
    """读取当前 telemetry 的关键指标，用于实时监控。"""
    if not TELEMETRY_PATH.is_file():
        return {"frames": 0, "t": 0, "speed": 0, "events": [], "collisions": 0}
    try:
        lines = TELEMETRY_PATH.read_text(encoding="utf-8").strip().splitlines()
        if not lines:
            return {"frames": 0, "t": 0, "speed": 0, "events": [], "collisions": 0}
        events = []
        collisions = 0
        for line in lines:
            try:
                frame = json.loads(line)
                for e in frame.get("events", []):
                    events.append(e)
                    if e.get("type") == "collision":
                        collisions += 1
            except json.JSONDecodeError:
                continue
        last = json.loads(lines[-1])
        car = last["cars"][0]
        return {
            "frames": len(lines),
            "t": last["t"],
            "speed": car.get("speed", 0),
            "events": events,
            "collisions": collisions,
        }
    except Exception:
        return {"frames": len(lines) if lines else 0, "t": 0, "speed": 0, "events": [], "collisions": 0}


def step_webots_sim(code_path: pathlib.Path, track: str, mode: str,
                    sim_timeout: int = 300, fast: bool = True) -> dict:
    """运行 Webots 仿真，超时自动关闭，返回结构化结果。

    参数:
        code_path: 控制器文件路径
        track: 赛道名 (basic/complex)
        mode: fastest/safe
        sim_timeout: 最大等待秒数（真实时间），超时后写 STOP 文件并强制结束
        fast: 是否使用 --mode=fast（无渲染加速）

    返回:
        dict 包含 laps_completed, best_lap, total_time, collisions_major,
        finish_reason, avg_speed, max_speed, events_count
    """
    print(f"\n── Webots 仿真 ({mode} @ {track}, 超时={sim_timeout}s) ──")
    webots = find_webots()
    if webots is None:
        print("  [SKIP] Webots 未安装")
        return {}

    sdk_runner = SDK_DIR / "run_local.py"
    if not sdk_runner.is_file():
        print("  [SKIP] SDK run_local.py 不可用")
        return {}

    _clean_recordings()
    _kill_webots()
    time.sleep(1)

    # 启动 Webots（后台运行）
    cmd = [
        sys.executable, str(sdk_runner),
        "--code-path", str(code_path),
        "--world", track,
        "--car-slot", "car_1",
        "--skip-validate",
    ]
    if fast:
        cmd.append("--fast")
    cmd.append("--minimize")

    print(f"  启动: {' '.join(cmd)}")
    webots_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    t_start = time.time()
    last_t = 0
    stall_count = 0
    result = {}

    try:
        while time.time() - t_start < sim_timeout:
            time.sleep(5)
            summary = _read_telemetry_summary()

            if summary["frames"] < 2:
                continue

            # 检测仿真是否在推进
            sim_t = summary["t"]
            if sim_t <= last_t + 0.5 and last_t > 5:
                stall_count += 1
                if stall_count >= 12:  # 60 秒无进展 → 卡住
                    print(f"  [WARN] 仿真停滞 (t={sim_t:.1f}s)，结束")
                    break
            else:
                stall_count = 0
            last_t = sim_t

            # 检查是否已完成比赛
            race_end = [e for e in summary["events"] if e.get("type") == "race_end"]
            if race_end:
                print(f"  [OK] 比赛结束: {race_end[0].get('reason', '?')}")
                break

            # 每 30 秒打印进度
            elapsed = int(time.time() - t_start)
            if elapsed % 30 < 5:
                print(f"  [{elapsed}s] sim_t={sim_t:.1f}s  speed={summary['speed']:.1f}m/s  "
                      f"events={len(summary['events'])}  collisions={summary['collisions']}  "
                      f"frames={summary['frames']}")

        else:
            # 超时 → 写 STOP 文件优雅结束
            print(f"  [INFO] 超时 {sim_timeout}s，写入 STOP 信号...")
            STOP_PATH.write_text("")
            time.sleep(5)  # 等待 supervisor 处理 STOP
    finally:
        # 确保 Webots 被关闭
        _kill_webots()
        webots_proc.terminate()
        try:
            webots_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            webots_proc.kill()

    elapsed = time.time() - t_start
    print(f"  仿真耗时: {elapsed:.1f}s（真实时间）")

    # ── 收集最终结果 ──
    summary = _read_telemetry_summary()

    # 从 metadata 读取官方记录
    official = {}
    if META_PATH.is_file():
        try:
            meta = json.loads(META_PATH.read_text(encoding="utf-8"))
            official["finish_reason"] = meta.get("finish_reason", "")
            official["duration_sim"] = meta.get("duration_sim", 0)
            rankings = meta.get("final_rankings", [])
            if rankings:
                r = rankings[0]
                official["laps"] = r.get("laps", 0)
                official["best_lap"] = r.get("best_lap")
                official["total_time"] = r.get("total_time")
                official["collisions_major"] = r.get("collision_major_count", 0)
        except (json.JSONDecodeError, KeyError):
            pass

    # 速度统计
    speeds = _collect_speeds()
    result = {
        "laps_completed": official.get("laps", 0),
        "best_lap": official.get("best_lap"),
        "total_time": official.get("total_time") or official.get("duration_sim", 0),
        "collisions_major": official.get("collisions_major",
                                          sum(1 for e in summary["events"]
                                              if e.get("severity") == "major")),
        "finish_reason": official.get("finish_reason", "timeout"),
        "avg_speed": speeds.get("avg", 0),
        "max_speed": speeds.get("max", 0),
        "sim_time": summary["t"],
        "events_count": len(summary["events"]),
    }

    print(f"  结果: laps={result['laps_completed']} best={result['best_lap']} "
          f"v_avg={result['avg_speed']:.1f}m/s v_max={result['max_speed']:.1f}m/s "
          f"collisions_major={result['collisions_major']} finish={result['finish_reason']}")

    return result


def _collect_speeds() -> dict:
    """从 telemetry 收集速度统计。"""
    if not TELEMETRY_PATH.is_file():
        return {}
    try:
        speeds = []
        with open(TELEMETRY_PATH, encoding="utf-8") as f:
            for line in f:
                try:
                    frame = json.loads(line)
                    for car in frame.get("cars", []):
                        speeds.append(car.get("speed", 0))
                except json.JSONDecodeError:
                    continue
        if not speeds:
            return {}
        return {
            "avg": round(sum(speeds) / len(speeds), 2),
            "max": round(max(speeds), 2),
        }
    except Exception:
        return {}


# ── CSV 记录 ────────────────────────────────────────────────────────────

CSV_HEADER = [
    "date", "commit", "mode", "track", "laps_completed", "best_lap",
    "total_time", "collisions_major", "finish_reason",
    "avg_speed_ms", "max_speed_ms", "sim_time_s",
    "avg_call_ms", "p95_call_ms", "build_ok", "local_validate_ok",
    "sdk_validate_ok", "pytest_ok", "notes",
]


def ensure_csv_header():
    """如果 CSV 文件不存在，写入表头。"""
    if not EXPERIMENTS_CSV.exists():
        EXPERIMENTS_CSV.parent.mkdir(parents=True, exist_ok=True)
        with open(EXPERIMENTS_CSV, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)


def append_csv_row(row: dict):
    """追加一行到实验 CSV。"""
    ensure_csv_header()
    with open(EXPERIMENTS_CSV, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([row.get(col, "") for col in CSV_HEADER])


# ── 主流程 ──────────────────────────────────────────────────────────────

def test_mode(mode: str, track: str = "basic", run_sim: bool = False,
              sim_timeout: int = 300, notes: str = "") -> dict:
    """对单个模式执行完整测试流程。"""
    print(f"\n{'='*60}")
    print(f"测试模式: {mode}")
    print(f"{'='*60}")

    row = {
        "date": now_iso(),
        "commit": current_commit(),
        "mode": mode,
        "track": track if run_sim else "validator_only",
        "laps_completed": "",
        "best_lap": "",
        "total_time": "",
        "collisions_major": "",
        "finish_reason": "",
        "avg_speed_ms": "",
        "max_speed_ms": "",
        "sim_time_s": "",
        "avg_call_ms": "",
        "p95_call_ms": "",
        "build_ok": "1",
        "local_validate_ok": "1",
        "sdk_validate_ok": "1",
        "pytest_ok": "1",
        "notes": notes,
    }

    # Step 1: 构建
    try:
        code_path = step_build(mode)
    except RuntimeError:
        row["build_ok"] = "0"
        append_csv_row(row)
        return row

    # Step 2: 本地校验
    if not step_local_validate(code_path, mode):
        row["local_validate_ok"] = "0"

    # Step 3: SDK 校验 + 性能
    sdk_ok, meta = step_sdk_validate(code_path, mode)
    if not sdk_ok:
        row["sdk_validate_ok"] = "0"
    if meta.get("avg_call_ms") is not None:
        row["avg_call_ms"] = str(meta["avg_call_ms"])
    if meta.get("p95_call_ms") is not None:
        row["p95_call_ms"] = str(meta["p95_call_ms"])

    # Step 4: pytest
    if not step_pytest():
        row["pytest_ok"] = "0"

    # Step 5: Webots 仿真 (可选)
    if run_sim:
        sim_result = step_webots_sim(code_path, track, mode, sim_timeout=sim_timeout)
        if sim_result:
            row["laps_completed"] = str(sim_result.get("laps_completed", ""))
            row["best_lap"] = str(sim_result.get("best_lap", "")) if sim_result.get("best_lap") else ""
            row["total_time"] = str(sim_result.get("total_time", ""))
            row["collisions_major"] = str(sim_result.get("collisions_major", ""))
            row["finish_reason"] = str(sim_result.get("finish_reason", ""))
            row["avg_speed_ms"] = str(sim_result.get("avg_speed", ""))
            row["max_speed_ms"] = str(sim_result.get("max_speed", ""))
            row["sim_time_s"] = str(sim_result.get("sim_time", ""))

    append_csv_row(row)
    return row


def list_tracks():
    """列出 SDK 中可用的赛道。"""
    sdk_runner = SDK_DIR / "run_local.py"
    if sdk_runner.is_file():
        run_cmd([sys.executable, str(sdk_runner), "--list-worlds"])
    else:
        print("SDK run_local.py 不可用")


def parse_args():
    parser = argparse.ArgumentParser(
        description="AI Racer 自动化测试与记录",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--mode", choices=["fastest", "safe", "both"], default="fastest",
                        help="测试模式（默认 fastest）")
    parser.add_argument("--track", default="basic",
                        help="Webots 赛道名（默认 basic，仅在 --sim 时有效）")
    parser.add_argument("--sim", action="store_true",
                        help="启动 Webots 仿真（需要 Webots 已安装）")
    parser.add_argument("--list-tracks", action="store_true",
                        help="列出可用赛道后退出")
    parser.add_argument("--notes", default="",
                        help="附加实验备注")
    parser.add_argument("--no-sim", dest="sim", action="store_false",
                        help="跳过仿真，只做校验（默认）")
    parser.add_argument("--sim-timeout", type=int, default=300,
                        help="Webots 仿真最大等待秒数（默认 300）")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list_tracks:
        list_tracks()
        return 0

    if args.mode == "both":
        for m in ("fastest", "safe"):
            test_mode(m, track=args.track, run_sim=args.sim,
                      sim_timeout=args.sim_timeout, notes=args.notes)
    else:
        test_mode(args.mode, track=args.track, run_sim=args.sim,
                  sim_timeout=args.sim_timeout, notes=args.notes)

    print(f"\n结果已记录到: {EXPERIMENTS_CSV}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
