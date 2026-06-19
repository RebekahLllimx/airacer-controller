"""从 Webots `.wbt` 解析赛道几何，供顶视图还原成"人在 Webots 里看到的"样子。

功能概述：读取 track_*.wbt，抽出红色地面（Solid name "ground" 的 Box）和路面网络
    （Road 直道的 wayPoints 折线 + CurvedRoadSegment 弧线），换算到世界坐标，
    返回可直接用 matplotlib 画的多边形/折线。只解析渲染需要的字段，不依赖 Webots。

坐标约定：世界系 x/y（与 telemetry 同一套），节点 translation + rotation(绕 z) 把局部
    几何摆到世界里。CurvedRoadSegment 默认 totalAngle=π/2、起点在局部原点、初始朝 +x、
    向左（CCW）转，曲率中心在 (0, R)。
"""

from __future__ import annotations

import math
import re
from pathlib import Path

DEFAULT_WORLDS = {
    "basic": "/Users/day/Desktop/Github/pkudsa.airacer/sdk/webots/worlds/track_basic.wbt",
    "complex": "/Users/day/Desktop/Github/pkudsa.airacer/sdk/webots/worlds/track_complex.wbt",
}

_NODE_RE = re.compile(r"^(Road|CurvedRoadSegment|Solid)\s*\{", re.MULTILINE)


def _node_blocks(text: str):
    """逐个产出顶层节点块 (node_type, block_text)，按花括号配对切。"""

    for m in _NODE_RE.finditer(text):
        node_type = m.group(1)
        depth = 0
        i = m.end() - 1  # 指向 '{'
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
    """返回绕 z 轴的旋转角（弧度）；非 z 轴或缺省按 0。"""

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
    for i in range(0, len(nums) - 2, 3):  # 每个点是 x y z
        pts.append((nums[i], nums[i + 1]))
    return pts


def _apply(pt: tuple[float, float], angle: float, tx: float, ty: float) -> tuple[float, float]:
    c, s = math.cos(angle), math.sin(angle)
    x, y = pt
    return (x * c - y * s + tx, x * s + y * c + ty)


def _curve_centerline(radius: float, total_angle: float, n: int = 24) -> list[tuple[float, float]]:
    """局部弧线中心线，按 Webots CurvedRoadSegment 原始公式：

        wayPoint_i = (radius·sin(θ), radius·cos(θ)),  θ = i·totalAngle/subdivision

    即曲率中心在局部原点，起点 (0, R)、终点 (R, 0)。
    """

    return [(radius * math.sin(total_angle * k / n), radius * math.cos(total_angle * k / n))
            for k in range(n + 1)]


def _offset_polygon(centerline: list[tuple[float, float]], width: float) -> list[tuple[float, float]]:
    """把中心线折线按 width 向两侧偏移成闭合路面多边形。"""

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
        nx, ny = -dy / norm, dx / norm  # 左法线
        px, py = centerline[i]
        left.append((px + nx * half, py + ny * half))
        right.append((px - nx * half, py - ny * half))
    return left + right[::-1]


def parse_world(world: str | Path):
    """解析赛道，返回 {"ground": [...], "roads": [polygon,...], "extent": (xmin,xmax,ymin,ymax)}。

    ground: [(cx, cy, sx, sy, (r,g,b))]；roads: 每条是闭合路面多边形点列表。
    """

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
