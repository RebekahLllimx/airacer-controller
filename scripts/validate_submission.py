"""Validate a generated ``team_controller.py`` before platform use.

The validator checks the source for prohibited capabilities, imports the
single-file controller, and calls ``control`` with mock camera frames.
"""

from __future__ import annotations

import argparse
import ast
import importlib.util
from pathlib import Path

import numpy as np

BANNED_MODULES = {
    "os",
    "sys",
    "socket",
    "subprocess",
    "multiprocessing",
    "threading",
    "time",
    "datetime",
    "io",
    "builtins",
    "ctypes",
    "shutil",
    "tempfile",
    "requests",
    "urllib",
    "http",
    "ftplib",
    "smtplib",
    "signal",
    "gc",
    "inspect",
    "importlib",
    "pickle",
    "glob",
    "fnmatch",
}
BANNED_NAMES = {
    "open",
    "eval",
    "exec",
    "compile",
    "globals",
    "locals",
    "input",
    "breakpoint",
    "__import__",
    "vars",
}


def check_ast(source, path):
    """Return prohibited imports and names found through AST inspection."""

    errors = []
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as exc:
        return [f"Syntax error: {exc}"]

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                root_name = alias.name.split(".")[0]
                if root_name in BANNED_MODULES:
                    errors.append(f"Prohibited module import: {alias.name}")
        elif isinstance(node, ast.ImportFrom):
            root_name = (node.module or "").split(".")[0]
            if root_name in BANNED_MODULES:
                errors.append(f"Prohibited import from module: {node.module}")
        elif isinstance(node, ast.Name) and node.id in BANNED_NAMES:
            errors.append(f"Prohibited name: {node.id}")
    return errors


def load_submission(path):
    """Import a standalone controller from ``path``."""

    spec = importlib.util.spec_from_file_location("team_controller_candidate", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load file: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_runtime(module):
    """Call ``control`` with mock frames and validate its output contract."""

    if not hasattr(module, "control") or not callable(module.control):
        return ["Missing callable control(left_img, right_img, timestamp)"]

    left_img = np.zeros((480, 640, 3), dtype=np.uint8)
    right_img = np.zeros((480, 640, 3), dtype=np.uint8)
    result = module.control(left_img, right_img, 0.0)

    if not isinstance(result, tuple) or len(result) != 2:
        return ["control() must return a two-element tuple"]

    steering, speed = result
    errors = []
    if not isinstance(steering, (int, float, np.floating)):
        errors.append("steering must be numeric")
    elif not -1.0 <= float(steering) <= 1.0:
        errors.append(f"steering is outside [-1, 1]: {steering}")

    if not isinstance(speed, (int, float, np.floating)):
        errors.append("speed must be numeric")
    elif not 0.0 <= float(speed) <= 1.0:
        errors.append(f"speed is outside [0, 1]: {speed}")

    return errors


def parse_args():
    """Parse the submission path."""

    parser = argparse.ArgumentParser(description="Validate an AI Racer team_controller.py file.")
    parser.add_argument("path", type=Path)
    return parser.parse_args()


def main():
    """Run static and runtime checks and return a process exit code."""

    args = parse_args()
    path = args.path.resolve()
    if not path.exists():
        print(f"File not found: {path}")
        return 1

    source = path.read_text(encoding="utf-8")
    errors = check_ast(source, path)
    if not errors:
        try:
            errors.extend(check_runtime(load_submission(path)))
        except Exception as exc:
            errors.append(f"Runtime error: {exc}")

    if errors:
        print("Validation failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print(f"Validation passed: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
