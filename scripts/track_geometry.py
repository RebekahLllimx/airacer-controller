"""Parse the Webots ground and road network needed for top-down plots.

The parser supports ``Road`` polylines and ``CurvedRoadSegment`` arcs, applies
their z-axis transforms, and returns polygons in the telemetry world frame. It
only reads fields required for rendering and does not depend on Webots.
"""

from __future__ import annotations

import math
import os
import re
from pathlib import Path

DEFAULT_WORLDS = {
    "basic": str(Path(os.environ.get("AIRACER_SDK", "pkudsa.airacer/sdk")) / "webots/worlds/track_basic.wbt"),
    "complex": str(Path(os.environ.get("AIRACER_SDK", "pkudsa.airacer/sdk")) / "webots/worlds/track_complex.wbt"),
}

_NODE_RE = re.compile(r"^(Road|CurvedRoadSegment|Solid)\s*\{", re.MULTILINE)


def _node_blocks(text: str):
    """Yield top-level node blocks by matching braces."""

    for m in _NODE_RE.finditer(text):
        node_type = m.group(1)
        depth = 0
        i = m.end() - 1

        start = i
        while i < len(text):
            ch = text[i]
            if ch == "{":
                depth += 1
            elif ch == "}":
                depth -= 1
                if depth == 0:
                    yield node_type, text[start:i + 1]
                    break
            i += 1


def _scalar(block: str, field: str, default: float | None = None) -> float | None:
    m = re.search(rf"\b{field}\s+(-?\d+\.?\d*(?:e-?\d+)?)", block)
    return float(m.group(1)) if m else default


def _translation(block: str) -> tuple[float, float]:
    m = re.search(r"\btranslation\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)", block)
    return (float(m.group(1)), float(m.group(2))) if m else (0.0, 0.0)


def _rotation_z(block: str) -> float:
    """Return the z-axis rotation in radians, or zero for unsupported axes."""

    m = re.search(r"\brotation\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)", block)
    if not m:
        return 0.0
    ax, ay, az, ang = (float(m.group(i)) for i in range(1, 5))
    return ang if abs(az) > 0.5 else 0.0


def _waypoints(block: str) -> list[tuple[float, float]]:
    m = re.search(r"\bwayPoints\s*\[(.*?)\]", block, re.DOTALL)
    if not m:
        return []
    nums = [float(tok) for tok in re.findall(r"-?\d+\.?\d*", m.group(1))]
    pts = []
    for i in range(0, len(nums) - 2, 3):

        pts.append((nums[i], nums[i + 1]))
    return pts


def _apply(pt: tuple[float, float], angle: float, tx: float, ty: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    x, y = pt
    return (x * c - y * s + tx, x * s + y * c + ty)


def _curve_centerline(radius: float, total_angle: float, n: int = 24) -> list[tuple[float, float]]:
    """Construct a local arc centreline using the Webots segment formula.

        wayPoint_i = (radius·sin(θ), radius·cos(θ)),  θ = i·totalAngle/subdivision

    The curvature centre is the local origin, from ``(0, R)`` to ``(R, 0)``.
    """

    return [(radius * math.sin(total_angle * k / n), radius * math.cos(total_angle * k / n))
            for k in range(n + 1)]


def _offset_polygon(centerline: list[tuple[float, float]], width: float) -> list[tuple[float, float]]:
    """Offset a centreline on both sides to form a closed road polygon."""

    half = width * 0.5
    n = len(centerline)
    if n < 2:
        return []
    left, right = [], []
    for i in range(n):
        if i == 0:
            dx = centerline[1][0] - centerline[0][0]
            dy = centerline[1][1] - centerline[0][1]
        elif i == n - 1:
            dx = centerline[-1][0] - centerline[-2][0]
            dy = centerline[-1][1] - centerline[-2][1]
        else:
            dx = centerline[i + 1][0] - centerline[i - 1][0]
            dy = centerline[i + 1][1] - centerline[i - 1][1]
        norm = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / norm, dx / norm

        px, py = centerline[i]
        left.append((px + nx * half, py + ny * half))
        right.append((px - nx * half, py - ny * half))
    return left + right[::-1]


def parse_world(world: str | Path):
    """Return ground boxes, closed road polygons, and the world extent."""

    path = Path(DEFAULT_WORLDS.get(str(world), str(world))).expanduser()
    text = path.read_text(encoding="utf-8")

    ground = []
    roads = []
    xs, ys = [], []
    for node_type, block in _node_blocks(text):
        if node_type == "Solid":
            if '"ground"' not in block and "name \"ground\"" not in block:
                continue
            size = re.search(r"\bsize\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)", block)
            if not size:
                continue
            sx, sy = float(size.group(1)), float(size.group(2))
            tx, ty = _translation(block)
            color = re.search(r"\bbaseColor\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)\s+(-?\d+\.?\d*)", block)
            rgb = (float(color.group(1)), float(color.group(2)), float(color.group(3))) if color else (0.75, 0.18, 0.14)
            ground.append((tx, ty, sx, sy, rgb))
            continue

        width = _scalar(block, "width", 8.0) or 8.0
        angle = _rotation_z(block)
        tx, ty = _translation(block)
        if node_type == "Road":
            local = _waypoints(block)
        else:  # CurvedRoadSegment
            radius = _scalar(block, "curvatureRadius", 10.0) or 10.0
            total = _scalar(block, "totalAngle", math.pi / 2) or (math.pi / 2)
            local = _curve_centerline(radius, total)
        if len(local) < 2:
            continue
        centerline = [_apply(p, angle, tx, ty) for p in local]
        poly = _offset_polygon(centerline, width)
        if poly:
            roads.append(poly)
            xs.extend(px for px, _ in poly)
            ys.extend(py for _, py in poly)

    extent = (min(xs), max(xs), min(ys), max(ys)) if xs else None
    return {"ground": ground, "roads": roads, "extent": extent}
