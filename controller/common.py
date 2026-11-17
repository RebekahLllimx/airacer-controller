"""Shared data contracts and output-bound helpers for the control pipeline."""

from dataclasses import dataclass

import numpy as np


@dataclass
class PerceptionObs:
    """Pixel-derived road, lane, obstacle, and confidence observation."""

    center_points: np.ndarray
    left_edge_points: np.ndarray
    right_edge_points: np.ndarray
    road_width_est: float
    confidence: float
    debug_flags: int = 0
    line_offset: float = 0.0
    line_heading: float = 0.0
    line_confidence: float = 0.0
    left_margin_near: float = 1.0
    right_margin_near: float = 1.0
    near_obstacle: bool = False
    # Nearby opponent position: -1 left, +1 right, 0 centered or absent.
    obstacle_x: float = 0.0
    # Largest connected vehicle-body component as a fraction of the search ROI.
    obstacle_size: float = 0.0
    # Downsampled grayscale frame-to-frame MAD; low values can indicate blockage.
    # The high default keeps directly constructed test fixtures in motion.
    frame_motion: float = 100.0


@dataclass
class TrackState:
    """Control-ready geometry, margin, lane, obstacle, and loss state."""

    lateral_error: float
    heading_error: float
    curvature: float
    lookahead_error: float
    confidence: float
    lost: bool
    red_environment: bool = False
    line_offset: float = 0.0
    line_heading: float = 0.0
    line_confidence: float = 0.0
    left_margin_near: float = 1.0
    right_margin_near: float = 1.0
    near_obstacle: bool = False
    obstacle_x: float = 0.0
    obstacle_size: float = 0.0
    # Low frame motion under a forward command can indicate physical blockage.
    frame_motion: float = 100.0


@dataclass
class ControlCmd:
    """Normalized steering and speed command returned by the policy."""

    steering: float
    speed: float


def clamp(value: float, low: float, high: float) -> float:
    """Clamp a numeric value to an inclusive interval."""

    return max(low, min(high, float(value)))


def clamp_cmd(cmd: ControlCmd) -> tuple[float, float]:
    """Clamp the platform tuple while retaining local reverse recovery support."""

    return clamp(cmd.steering, -1.0, 1.0), clamp(cmd.speed, -1.0, 1.0)
