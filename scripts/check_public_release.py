#!/usr/bin/env python3
"""Reject private paths or non-English prose in the public project surface."""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PUBLIC_TARGETS = (
    ROOT / "README.md",
    ROOT / "TASK.md",
    ROOT / "controller",
    ROOT / "docs",
    ROOT / "experiments",
    ROOT / "submissions",
)
TEXT_SUFFIXES = {".csv", ".json", ".jsonl", ".md", ".py", ".rst", ".txt", ".yaml", ".yml"}
HAN = re.compile(r"[\u3400-\u9fff]")
PRIVATE_PATH = re.compile(r"(?:/Users/[^/\s]+|[A-Za-z]:\\Users\\[^\\\s]+)")


def iter_text_files() -> list[Path]:
    files: list[Path] = []
    for target in PUBLIC_TARGETS:
        if target.is_file():
            files.append(target)
        elif target.is_dir():
            files.extend(
                path
                for path in target.rglob("*")
                if path.is_file() and path.suffix.lower() in TEXT_SUFFIXES
            )
    return sorted(set(files))


def main() -> int:
    findings: list[str] = []
    for path in iter_text_files():
        text = path.read_text(encoding="utf-8", errors="replace")
        relative = path.relative_to(ROOT)
        if HAN.search(text):
            findings.append(f"non-English prose: {relative}")
        if PRIVATE_PATH.search(text):
            findings.append(f"private absolute path: {relative}")

    if findings:
        print("Public-release scan failed:")
        for finding in findings:
            print(f"- {finding}")
        return 1

    print(f"Public-release scan passed ({len(iter_text_files())} text files).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
