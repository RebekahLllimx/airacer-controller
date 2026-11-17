"""Build a self-contained ``team_controller.py`` submission artifact.

The builder combines the modular controller sources in dependency order,
removes imports supplied by the generated header, and injects the selected
runtime profile.
"""

from __future__ import annotations

import argparse
import ast
import io
import tokenize
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER_DIR = ROOT / "controller"
DEFAULT_OUTPUTS = {
    "no_other_cars": ROOT / "submissions" / "no_other_cars" / "team_controller.py",
    "with_other_cars": ROOT / "submissions" / "with_other_cars" / "team_controller.py",
}
MODULE_ORDER = [
    "common.py",
    "params.py",
    "opponent.py",
    "perception.py",
    "estimator.py",
    "policy.py",
    "team_controller_local.py",
]


def strip_local_imports(source: str) -> str:
    """Remove imports supplied by the generated single-file header.

    Import nodes are removed by source span rather than string matching. This
    keeps the build stable when a formatter wraps an import across lines.
    """

    tree = ast.parse(source)
    removable_ranges: list[tuple[int, int]] = []
    bundled_imports = {("cv2", None), ("math", None), ("numpy", "np")}

    for node in tree.body:
        remove = False
        if isinstance(node, ast.ImportFrom):
            module = node.module or ""
            remove = (
                module == "__future__"
                or node.level > 0
                or module.startswith("controller.")
                or module == "dataclasses"
            )
        elif isinstance(node, ast.Import):
            remove = all(
                (alias.name, alias.asname) in bundled_imports
                for alias in node.names
            )
        if remove:
            removable_ranges.append((node.lineno, node.end_lineno or node.lineno))

    lines = source.splitlines()
    removed_lines = {
        line_number
        for start, end in removable_ranges
        for line_number in range(start, end + 1)
    }
    return "\n".join(
        line for line_number, line in enumerate(lines, start=1)
        if line_number not in removed_lines
    ).strip()


def strip_submission_text(source: str) -> str:
    """Remove comments and standalone docstrings from bundled source.

    Tokenization avoids treating ``#`` characters inside string literals as
    comments. Executable statements and string values remain unchanged.
    """

    kept_tokens = []
    previous_type = tokenize.INDENT
    for token in tokenize.generate_tokens(io.StringIO(source).readline):
        token_type, token_text, start, end, line = token
        if token_type == tokenize.COMMENT and not token_text.startswith("# ----"):
            continue
        if token_type == tokenize.STRING and previous_type in {
            tokenize.INDENT,
            tokenize.NEWLINE,
            tokenize.NL,
        }:
            previous_type = token_type
            continue
        kept_tokens.append((token_type, token_text, start, end, line))
        if token_type not in {tokenize.NL, tokenize.COMMENT}:
            previous_type = token_type
    stripped = tokenize.untokenize(kept_tokens)
    return "\n".join(
        line.rstrip()
        for line in stripped.splitlines()
        if line.strip() and line.strip() != "\\"
    )


class _SubmissionMinifier(ast.NodeTransformer):
    """Remove non-runtime metadata while preserving executable behavior.

    Dataclass field annotations are retained, while function annotations and
    standalone docstrings are removed to stay below the platform size limit.
    """

    @staticmethod
    def _strip_docstring(body: list[ast.stmt]) -> list[ast.stmt]:
        return [
            stmt for stmt in body
            if not (
                isinstance(stmt, ast.Expr)
                and isinstance(stmt.value, ast.Constant)
                and isinstance(stmt.value.value, str)
            )
        ]

    def visit_Module(self, node: ast.Module) -> ast.AST:
        self.generic_visit(node)
        node.body = self._strip_docstring(node.body)
        return node

    def visit_ClassDef(self, node: ast.ClassDef) -> ast.AST:
        self.generic_visit(node)
        node.body = self._strip_docstring(node.body)
        return node

    def visit_FunctionDef(self, node: ast.FunctionDef) -> ast.AST:
        self.generic_visit(node)
        node.body = self._strip_docstring(node.body)
        node.returns = None
        args = list(node.args.posonlyargs) + list(node.args.args) + list(node.args.kwonlyargs)
        for arg in args:
            arg.annotation = None
        if node.args.vararg is not None:
            node.args.vararg.annotation = None
        if node.args.kwarg is not None:
            node.args.kwarg.annotation = None
        return node

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> ast.AST:
        self.generic_visit(node)
        node.body = self._strip_docstring(node.body)
        node.returns = None
        args = list(node.args.posonlyargs) + list(node.args.args) + list(node.args.kwonlyargs)
        for arg in args:
            arg.annotation = None
        if node.args.vararg is not None:
            node.args.vararg.annotation = None
        if node.args.kwarg is not None:
            node.args.kwarg.annotation = None
        return node


def minify_submission_ast(source: str) -> str:
    """Apply AST-level minification after token-level text removal."""

    tree = ast.parse(source)
    tree = _SubmissionMinifier().visit(tree)
    ast.fix_missing_locations(tree)
    return ast.unparse(tree)


_DEBUG_CONTROL_BLOCK = '''    try:
        profile = get_profile(PROFILE)
        obs = extract_observation(left_img, right_img, timestamp, profile=profile)
        track = estimate_track(obs, timestamp)
        cmd = decide_control(track, timestamp, mode=PROFILE)
        steering, speed = clamp_cmd(cmd)
        return steering, speed
    except Exception:
        return 0.0, 0.0'''


def _debug_console_tee_header() -> list[str]:
    """Return optional stdout/stderr tee code for local debug builds."""

    return [
        "import os as _dbg_os",
        "import pathlib as _dbg_pathlib",
        "import sys as _dbg_sys",
        "class _DbgConsoleTee:",
        "    def __init__(self, primary, extra):",
        "        self._primary = primary",
        "        self._extra = extra",
        "    def write(self, data):",
        "        self._primary.write(data)",
        "        self._extra.write(data)",
        "        self.flush()",
        "    def flush(self):",
        "        self._primary.flush()",
        "        self._extra.flush()",
        "_DBG_CONSOLE_HANDLE = None",
        "try:",
        "    _DBG_CONSOLE_DIR = _dbg_os.environ.get(\"AIRACER_CONTROLLER_CONSOLE_LOG_DIR\")",
        "    if _DBG_CONSOLE_DIR:",
        "        _dbg_console_dir = _dbg_pathlib.Path(_DBG_CONSOLE_DIR)",
        "        _dbg_console_dir.mkdir(parents=True, exist_ok=True)",
        "        _dbg_console_path = _dbg_console_dir / (\"team_controller_%s.log\" % _dbg_os.getpid())",
        "        _DBG_CONSOLE_HANDLE = _dbg_console_path.open(\"a\", encoding=\"utf-8\", buffering=1)",
        "        _dbg_sys.stdout = _DbgConsoleTee(_dbg_sys.stdout, _DBG_CONSOLE_HANDLE)",
        "        _dbg_sys.stderr = _DbgConsoleTee(_dbg_sys.stderr, _DBG_CONSOLE_HANDLE)",
        "        print(\"[team_controller] console tee enabled: %s\" % _dbg_console_path)",
        "except Exception as _dbg_console_exc:",
        "    print(\"[team_controller][warn] failed to enable console tee: %s\" % _dbg_console_exc)",
    ]


def _debug_control_block(
    log_path: str | None = None,
    dump_frames: str | None = None,
    dump_frame_stride: int = 10,
    dump_frame_start: float | None = None,
    dump_frame_end: float | None = None,
) -> str:
    """Return an instrumented ``control`` function for local runs only."""

    return '''    try:
        global _DBG_FRAME_INDEX
        if _DBG_FRAME_DIR is not None:
            try:
                _dbg_timestamp = float(timestamp)
                _dbg_in_window = _DBG_FRAME_START <= _dbg_timestamp <= _DBG_FRAME_END
                if _dbg_in_window:
                    _DBG_FRAME_INDEX += 1
                if _dbg_in_window and _DBG_FRAME_INDEX % _DBG_FRAME_STRIDE == 0:
                    _dbg_t = ("%010.3f" % float(timestamp)).replace("-", "m").replace(".", "_")
                    cv2.imwrite(_DBG_FRAME_DIR + "frame_" + _dbg_t + "_left.png", left_img)
                    cv2.imwrite(_DBG_FRAME_DIR + "frame_" + _dbg_t + "_right.png", right_img)
            except Exception:
                pass
        profile = get_profile(PROFILE)
        obs = extract_observation(left_img, right_img, timestamp, profile=profile)
        track = estimate_track(obs, timestamp)
        cmd = decide_control(track, timestamp, mode=PROFILE)
        steering, speed = clamp_cmd(cmd)
        if _DBG_FH is not None:
            try:
                _DBG_FH.write(_dbg_json.dumps({
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
                    "mode": _LAST_MODE,
                    "mode_reason": _LAST_MODE_REASON,
                    "target_steering": round(float(_LAST_TARGET_STEERING), 4),
                    "target_speed": round(float(_LAST_TARGET_SPEED), 4),
                    "curve_risk": round(float(_LAST_SIGNALS.get("curve_risk", 0.0)), 4),
                    "offset_risk": round(float(_LAST_SIGNALS.get("offset_risk", 0.0)), 4),
                    "margin_risk": round(float(_LAST_SIGNALS.get("margin_risk", 0.0)), 4),
                    "straight_memory": bool(_LAST_STRAIGHT_MEMORY_ACTIVE),
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
                    "frame_motion": round(float(track.frame_motion), 3),
                }) + "\\n")
                _DBG_FH.flush()
            except Exception:
                pass
        return steering, speed
    except Exception:
        return 0.0, 0.0'''


def read_module(
    name: str,
    mode: str,
    debug_log: str | None = None,
    dump_frames: str | None = None,
    dump_frame_stride: int = 10,
    dump_frame_start: float | None = None,
    dump_frame_end: float | None = None,
) -> str:
    """Read one controller module and prepare it for bundling."""

    source = (CONTROLLER_DIR / name).read_text(encoding="utf-8")
    source = strip_local_imports(source)
    if name == "team_controller_local.py":
        # Inject the selected runtime profile into the generated entry point.
        source = source.replace(
            'PROFILE = "no_other_cars"', f'PROFILE = {mode!r}'
        )
        if debug_log or dump_frames:
            header_lines = _debug_console_tee_header()
            if debug_log:
                header_lines.extend([
                    "import json as _dbg_json",
                    f"try:\n    _DBG_FH = open({repr(str(debug_log))}, \"w\", encoding=\"utf-8\")\n"
                    "except Exception:\n    _DBG_FH = None",
                ])
            else:
                header_lines.append("_DBG_FH = None")
            if dump_frames:
                frame_start_expr = "float('-inf')" if dump_frame_start is None else repr(float(dump_frame_start))
                frame_end_expr = "float('inf')" if dump_frame_end is None else repr(float(dump_frame_end))
                header_lines.extend([
                    f"_DBG_FRAME_DIR = {repr(str(dump_frames).rstrip('/') + '/')}",
                    f"_DBG_FRAME_STRIDE = {max(int(dump_frame_stride), 1)}",
                    f"_DBG_FRAME_START = {frame_start_expr}",
                    f"_DBG_FRAME_END = {frame_end_expr}",
                    "_DBG_FRAME_INDEX = 0",
                ])
            else:
                header_lines.extend([
                    "_DBG_FRAME_DIR = None",
                    "_DBG_FRAME_STRIDE = 1",
                    "_DBG_FRAME_START = float('-inf')",
                    "_DBG_FRAME_END = float('inf')",
                    "_DBG_FRAME_INDEX = 0",
                ])
            header = "\n".join(header_lines) + "\n"
            source = header + "\n" + source
            source = source.replace(
                _DEBUG_CONTROL_BLOCK,
                _debug_control_block(
                    log_path=debug_log,
                    dump_frames=dump_frames,
                    dump_frame_stride=dump_frame_stride,
                    dump_frame_start=dump_frame_start,
                    dump_frame_end=dump_frame_end,
                ),
            )
    return source


def build_source(
    mode: str,
    debug_log: str | None = None,
    dump_frames: str | None = None,
    dump_frame_stride: int = 10,
    dump_frame_start: float | None = None,
    dump_frame_end: float | None = None,
) -> str:
    """Build and return a self-contained controller source file."""

    parts = [
        "# Generated by scripts/build_submission.py. Edit controller/ instead.\n"
        "from dataclasses import dataclass, field\n"
        "import math\n\n"
        "import cv2\n"
        "import numpy as np\n"
    ]
    for module in MODULE_ORDER:
        parts.append(f"\n# ---- {module} ----\n")
        parts.append(read_module(
            module,
            mode,
            debug_log=debug_log,
            dump_frames=dump_frames,
            dump_frame_stride=dump_frame_stride,
            dump_frame_start=dump_frame_start,
            dump_frame_end=dump_frame_end,
        ))
        parts.append("\n")
    source = "\n".join(parts).strip() + "\n"
    if not (debug_log or dump_frames):
        source = strip_submission_text(source)
        source = minify_submission_ast(source)
    return source.strip() + "\n"


def parse_args():
    """Parse the controller profile, output path, and debug options."""

    parser = argparse.ArgumentParser(description="Build single-file AI Racer submission.")
    parser.add_argument("--mode", choices=sorted(DEFAULT_OUTPUTS), default="no_other_cars",
                        help="Controller profile: solo time trial or multi-car traffic")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--debug-log", type=Path, default=None,
                        help="Local debug build: write per-frame state and commands to JSONL")
    parser.add_argument("--dump-frames", type=Path, default=None,
                        help="Local debug build: save left/right BGR frames as lossless PNG files")
    parser.add_argument("--dump-frame-stride", type=int, default=10,
                        help="Frame interval for --dump-frames; use 1 for every frame")
    parser.add_argument("--dump-frame-start", type=float, default=None,
                        help="Save frames at or after this simulation timestamp")
    parser.add_argument("--dump-frame-end", type=float, default=None,
                        help="Save frames at or before this simulation timestamp")
    return parser.parse_args()


def main() -> int:
    """Build the requested standalone controller and print its location."""

    args = parse_args()
    output = args.out or DEFAULT_OUTPUTS[args.mode]
    output = output if output.is_absolute() else ROOT / output
    output.parent.mkdir(parents=True, exist_ok=True)
    debug_log = None
    if args.debug_log is not None:
        debug_log = args.debug_log if args.debug_log.is_absolute() else ROOT / args.debug_log
        debug_log.parent.mkdir(parents=True, exist_ok=True)
        debug_log = str(debug_log)
    dump_frames = None
    if args.dump_frames is not None:
        dump_frames_path = args.dump_frames if args.dump_frames.is_absolute() else ROOT / args.dump_frames
        dump_frames_path.mkdir(parents=True, exist_ok=True)
        dump_frames = str(dump_frames_path)
    output.write_text(
        build_source(
            args.mode,
            debug_log=debug_log,
            dump_frames=dump_frames,
            dump_frame_stride=args.dump_frame_stride,
            dump_frame_start=args.dump_frame_start,
            dump_frame_end=args.dump_frame_end,
        ),
        encoding="utf-8",
    )
    suffix_parts = []
    if debug_log:
        suffix_parts.append(f"debug log: {debug_log}")
    if dump_frames:
        window = ""
        if args.dump_frame_start is not None or args.dump_frame_end is not None:
            start = args.dump_frame_start if args.dump_frame_start is not None else "-inf"
            end = args.dump_frame_end if args.dump_frame_end is not None else "inf"
            window = f", window={start}..{end}"
        suffix_parts.append(f"frame directory: {dump_frames} (stride={max(int(args.dump_frame_stride), 1)}{window})")
    suffix = f"  ({'; '.join(suffix_parts)})" if suffix_parts else ""
    print(f"Built {args.mode}: {output}{suffix}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
