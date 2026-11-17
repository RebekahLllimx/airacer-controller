"""Convert perception output into a stable geometric track state."""

import math
from dataclasses import dataclass

import numpy as np

from controller.common import PerceptionObs, TrackState, clamp
from controller.params import ESTIMATOR_PROFILE

_RED_ENVIRONMENT_FLAG = 32
_RED_ENVIRONMENT_LATCH_FRAMES = 3


@dataclass
class EstimatorState:
    """Cross-frame geometry and line memory for one controller session."""

    last_track: TrackState | None = None
    last_timestamp: float | None = None
    last_red_environment: bool = False
    red_environment_streak: int = 0
    line_memory_frames: int = 0
    line_memory_offset: float = 0.0
    line_memory_heading: float = 0.0
    line_memory_confidence: float = 0.0

    def __post_init__(self) -> None:
        if self.last_track is None:
            self.last_track = TrackState(0.0, 0.0, 0.0, 0.0, 0.0, True)


@dataclass(frozen=True)
class GeometryEstimate:
    """Unsmooth geometry and confidence measured from one perception frame."""

    lateral_error: float
    heading_error: float
    curvature: float
    lookahead_error: float
    confidence: float
    left_margin_near: float
    right_margin_near: float


_STATE = EstimatorState()


def _lost_track(
    confidence: float,
    red_environment: bool | None = None,
    obs: PerceptionObs | None = None,
) -> TrackState:
    """Decay prior geometry while preserving current lane and obstacle evidence."""

    if red_environment is None:
        red_environment = _STATE.last_track.red_environment
    if obs is not None:
        line_offset = clamp(float(obs.line_offset), -1.0, 1.0)
        line_heading = clamp(float(obs.line_heading), -1.0, 1.0)
        line_confidence = clamp(float(obs.line_confidence), 0.0, 1.0)
        near_obstacle = bool(obs.near_obstacle)
        obstacle_x = clamp(float(obs.obstacle_x), -1.0, 1.0)
        obstacle_size = max(float(obs.obstacle_size), 0.0)
    else:
        line_offset = _STATE.last_track.line_offset
        line_heading = _STATE.last_track.line_heading
        line_confidence = 0.0
        near_obstacle = _STATE.last_track.near_obstacle
        obstacle_x = _STATE.last_track.obstacle_x
        obstacle_size = _STATE.last_track.obstacle_size
    return TrackState(
        _STATE.last_track.lateral_error * ESTIMATOR_PROFILE["lost_lateral_decay"],
        _STATE.last_track.heading_error * ESTIMATOR_PROFILE["lost_heading_decay"],
        _STATE.last_track.curvature * ESTIMATOR_PROFILE["lost_curvature_decay"],
        _STATE.last_track.lookahead_error * ESTIMATOR_PROFILE["lost_lookahead_decay"],
        clamp(confidence, 0.0, ESTIMATOR_PROFILE["lost_confidence"]),
        True,
        red_environment,
        line_offset,
        line_heading,
        line_confidence,
        _STATE.last_track.left_margin_near,
        _STATE.last_track.right_margin_near,
        near_obstacle,
        obstacle_x,
        obstacle_size,
        obs.frame_motion,
    )


def _clean_points(points) -> np.ndarray:
    """Convert center points to a finite float32 array with shape ``N x 2``."""

    try:
        array = np.asarray(points, dtype=np.float32)
    except (TypeError, ValueError):
        return np.empty((0, 2), dtype=np.float32)
    if array.ndim != 2 or array.shape[1] != 2:
        return np.empty((0, 2), dtype=np.float32)
    return array[np.isfinite(array).all(axis=1)]


def _normalize_points(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
    """Normalize image x to lateral error and y to near-to-far progress."""

    y = points[:, 1].astype(np.float32)
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    y_span = max(y_max - y_min, 0.0)
    progress = (y_max - y) / max(y_span, 1.0)

    x = points[:, 0].astype(np.float32)
    x_norm = (x - ESTIMATOR_PROFILE["image_center_x"]) / ESTIMATOR_PROFILE["x_scale"]
    x_norm = np.clip(x_norm, -1.0, 1.0)
    return x_norm.astype(np.float32), progress.astype(np.float32), y_span


def _near_edge_margin(edge_points, side: str) -> float:
    """Estimate normalized near-field clearance from the image center."""

    points = _clean_points(edge_points)
    if len(points) == 0:
        return 1.0
    y = points[:, 1].astype(np.float32)
    y_min = float(np.min(y))
    y_max = float(np.max(y))
    y_span = max(y_max - y_min, 1.0)
    near_mask = y >= y_min + y_span * 0.62
    if not np.any(near_mask):
        near_mask = np.ones(len(points), dtype=bool)
    x = float(np.median(points[near_mask, 0]))
    center = ESTIMATOR_PROFILE["image_center_x"]
    scale = ESTIMATOR_PROFILE["x_scale"]
    if side == "left":
        return clamp((center - x) / scale, 0.0, 1.0)
    return clamp((x - center) / scale, 0.0, 1.0)


def _line_weight(line_confidence: float) -> float:
    """Convert lane confidence into a bounded geometry-fusion weight."""

    min_confidence = ESTIMATOR_PROFILE["line_target_min_confidence"]
    if line_confidence < min_confidence:
        return 0.0
    usable = (line_confidence - min_confidence) / max(1.0 - min_confidence, 1e-6)
    return clamp(usable, 0.0, 1.0)


def _line_target_trust(
    obs: PerceptionObs, timestamp: float, red_environment: bool
) -> float:
    """Gate lane evidence by confidence, geometry, scene, and road-mask quality."""

    base = _line_weight(obs.line_confidence)
    if base <= 0.0:
        return 0.0

    startup_valid = (
        red_environment
        and timestamp <= ESTIMATOR_PROFILE["line_startup_until"]
        and ESTIMATOR_PROFILE["line_startup_offset_min"]
        <= obs.line_offset
        <= ESTIMATOR_PROFILE["line_startup_offset_max"]
        and ESTIMATOR_PROFILE["line_startup_heading_min"]
        <= obs.line_heading
        <= ESTIMATOR_PROFILE["line_startup_heading_max"]
    )
    normal_valid = (
        abs(obs.line_offset) <= ESTIMATOR_PROFILE["line_target_normal_offset_max"]
    )
    if not startup_valid and not normal_valid:
        return 0.0

    if startup_valid:
        scale = ESTIMATOR_PROFILE["line_target_startup_scale"]
    elif (
        obs.confidence <= ESTIMATOR_PROFILE["line_target_road_confidence_max"]
        or obs.debug_flags & 4
    ):
        scale = ESTIMATOR_PROFILE["line_target_unreliable_road_scale"]
    else:
        scale = ESTIMATOR_PROFILE["line_target_normal_scale"]
    return clamp(base * scale, 0.0, 1.0)


def _update_line_memory(
    obs: PerceptionObs, line_trust: float, timestamp: float, red_environment: bool
) -> float:
    """Bridge brief startup lane gaps with a decaying trusted-line memory."""

    in_startup = (
        red_environment and timestamp <= ESTIMATOR_PROFILE["line_startup_until"]
    )
    startup_line = (
        ESTIMATOR_PROFILE["line_startup_offset_min"]
        <= obs.line_offset
        <= ESTIMATOR_PROFILE["line_startup_offset_max"]
        and ESTIMATOR_PROFILE["line_startup_heading_min"]
        <= obs.line_heading
        <= ESTIMATOR_PROFILE["line_startup_heading_max"]
    )
    if line_trust > 0.0 and in_startup and startup_line:
        _STATE.line_memory_frames = int(ESTIMATOR_PROFILE["line_startup_memory_frames"])
        _STATE.line_memory_offset = clamp(float(obs.line_offset), -1.0, 1.0)
        _STATE.line_memory_heading = clamp(float(obs.line_heading), -1.0, 1.0)
        _STATE.line_memory_confidence = clamp(float(obs.line_confidence), 0.0, 1.0)
        return line_trust

    if not in_startup or _STATE.line_memory_frames <= 0:
        if not in_startup:
            _STATE.line_memory_frames = 0
        return line_trust

    _STATE.line_memory_frames -= 1
    _STATE.line_memory_offset *= ESTIMATOR_PROFILE["line_startup_memory_value_decay"]
    _STATE.line_memory_heading *= ESTIMATOR_PROFILE["line_startup_memory_value_decay"]
    _STATE.line_memory_confidence *= ESTIMATOR_PROFILE[
        "line_startup_memory_value_decay"
    ]
    obs.line_offset = _STATE.line_memory_offset
    obs.line_heading = _STATE.line_memory_heading
    obs.line_confidence = _STATE.line_memory_confidence
    remembered_trust = _line_target_trust(obs, timestamp, red_environment)
    return clamp(
        remembered_trust * ESTIMATOR_PROFILE["line_startup_memory_trust_scale"],
        0.0,
        1.0,
    )


def _line_guidance_targets(obs: PerceptionObs) -> tuple[float, float]:
    """Prefer centering over line slope when offset and heading disagree."""

    line_offset = clamp(float(obs.line_offset), -1.0, 1.0)
    line_heading = clamp(float(obs.line_heading), -1.0, 1.0)
    projected = (
        line_offset + line_heading * ESTIMATOR_PROFILE["line_lookahead_projection"]
    )
    conflict = (
        abs(line_offset) >= ESTIMATOR_PROFILE["line_offset_priority_min"]
        and line_offset * line_heading < 0.0
    )
    if conflict:
        line_heading *= ESTIMATOR_PROFILE["line_conflict_heading_scale"]
        min_projected = (
            abs(line_offset) * ESTIMATOR_PROFILE["line_conflict_projected_scale"]
        )
        projected = math.copysign(max(abs(projected), min_projected), line_offset)
    return clamp(line_heading, -1.0, 1.0), clamp(projected, -1.0, 1.0)


def _apply_line_target(
    lateral_error: float,
    heading_error: float,
    lookahead_error: float,
    obs: PerceptionObs,
    line_trust: float,
) -> tuple[float, float, float]:
    """Blend trusted lane targets into road-derived geometry."""

    if line_trust <= 0.0:
        return lateral_error, heading_error, lookahead_error
    lateral_weight = line_trust * ESTIMATOR_PROFILE["line_lateral_weight"]
    heading_weight = line_trust * ESTIMATOR_PROFILE["line_heading_weight"]
    lookahead_weight = line_trust * ESTIMATOR_PROFILE["line_lookahead_weight"]
    line_heading, projected_lookahead = _line_guidance_targets(obs)
    return (
        clamp(
            lateral_error * (1.0 - lateral_weight) + obs.line_offset * lateral_weight,
            -1.0,
            1.0,
        ),
        clamp(
            heading_error * (1.0 - heading_weight) + line_heading * heading_weight,
            -1.0,
            1.0,
        ),
        clamp(
            lookahead_error * (1.0 - lookahead_weight)
            + projected_lookahead * lookahead_weight,
            -1.0,
            1.0,
        ),
    )


def _line_only_track(
    obs: PerceptionObs, timestamp: float, red_environment: bool, line_trust: float
) -> TrackState:
    """Construct a conservative track state when lane evidence outlives the road mask."""

    del timestamp
    confidence = clamp(
        max(
            ESTIMATOR_PROFILE["lost_confidence"] + 0.04,
            obs.line_confidence * ESTIMATOR_PROFILE["line_target_confidence_scale"],
        ),
        0.0,
        1.0,
    )
    line_heading, projected_lookahead = _line_guidance_targets(obs)
    lateral_error = clamp(obs.line_offset * line_trust, -1.0, 1.0)
    heading_error = clamp(line_heading * line_trust, -1.0, 1.0)
    lookahead_error = clamp(projected_lookahead * line_trust, -1.0, 1.0)
    alpha = _smooth_alpha(confidence)
    track = TrackState(
        _smooth_limited(
            _STATE.last_track.lateral_error,
            lateral_error,
            alpha,
            ESTIMATOR_PROFILE["max_error_delta"],
        ),
        _smooth_limited(
            _STATE.last_track.heading_error,
            heading_error,
            alpha,
            ESTIMATOR_PROFILE["max_heading_delta"],
        ),
        _smooth_limited(
            _STATE.last_track.curvature,
            0.0,
            ESTIMATOR_PROFILE["curve_smooth_alpha"],
            ESTIMATOR_PROFILE["max_curvature_delta"],
        ),
        _smooth_limited(
            _STATE.last_track.lookahead_error,
            lookahead_error,
            alpha,
            ESTIMATOR_PROFILE["max_error_delta"],
        ),
        confidence,
        False,
        red_environment,
        clamp(float(obs.line_offset), -1.0, 1.0),
        clamp(float(obs.line_heading), -1.0, 1.0),
        clamp(float(obs.line_confidence), 0.0, 1.0),
        _STATE.last_track.left_margin_near,
        _STATE.last_track.right_margin_near,
        bool(obs.near_obstacle),
        clamp(float(obs.obstacle_x), -1.0, 1.0),
        max(float(obs.obstacle_size), 0.0),
        obs.frame_motion,
    )
    return track


def _fit_centerline(progress: np.ndarray, x_norm: np.ndarray) -> tuple[np.ndarray, int]:
    """Fit a quadratic centerline when supported, otherwise a line."""

    degree = 2 if len(progress) >= ESTIMATOR_PROFILE["poly2_min_points"] else 1
    coeffs = np.polyfit(progress, x_norm, deg=degree)
    return coeffs.astype(np.float32), degree


def _value_from_band(
    progress: np.ndarray,
    x_norm: np.ndarray,
    coeffs: np.ndarray,
    mask: np.ndarray,
    fallback_progress: float,
) -> float:
    """Estimate lateral error from a progress band or fitted fallback point."""

    if np.any(mask):
        return float(np.median(x_norm[mask]))
    return float(np.polyval(coeffs, fallback_progress))


def _heading_from_fit(coeffs: np.ndarray, degree: int) -> float:
    """Estimate heading error from the centerline derivative."""

    eval_progress = ESTIMATOR_PROFILE["heading_eval_progress"]
    if degree == 2:
        derivative = float(2.0 * coeffs[0] * eval_progress + coeffs[1])
    else:
        derivative = float(coeffs[0])
    return clamp(derivative * ESTIMATOR_PROFILE["heading_gain"], -1.0, 1.0)


def _curvature_trust(n_points: int, y_span: float, fit_score: float) -> float:
    """Down-weight curvature when points, vertical span, or fit quality are weak."""

    span = max(
        ESTIMATOR_PROFILE["curvature_full_points"]
        - ESTIMATOR_PROFILE["poly2_min_points"],
        1.0,
    )
    n_score = clamp(
        (float(n_points) - ESTIMATOR_PROFILE["poly2_min_points"]) / span, 0.0, 1.0
    )
    span_score = clamp(y_span / ESTIMATOR_PROFILE["curvature_min_y_span"], 0.0, 1.0)
    return clamp(n_score * span_score * clamp(fit_score, 0.0, 1.0), 0.0, 1.0)


def _curvature_from_fit(
    coeffs: np.ndarray,
    degree: int,
    lateral_error: float,
    lookahead_error: float,
    trust: float,
) -> float:
    """Estimate signed curvature and shrink unreliable fits toward zero."""

    if degree == 2:
        value = float(coeffs[0]) * ESTIMATOR_PROFILE["curvature_gain"]
    else:
        value = (lookahead_error - lateral_error) * ESTIMATOR_PROFILE[
            "fallback_curvature_gain"
        ]
    return clamp(value * trust, -1.0, 1.0)


def _fit_error_score(
    progress: np.ndarray, x_norm: np.ndarray, coeffs: np.ndarray
) -> float:
    """Convert centerline residual error into a confidence score."""

    fitted = np.polyval(coeffs, progress)
    rmse = float(np.sqrt(np.mean((x_norm - fitted) ** 2)))
    return clamp(1.0 - rmse / ESTIMATOR_PROFILE["max_fit_error"], 0.0, 1.0)


def _geometry_confidence(
    obs: PerceptionObs, points: np.ndarray, y_span: float, fit_score: float
) -> float:
    """Combine observation confidence with point, span, fit, and width quality."""

    obs_score = clamp(obs.confidence, 0.0, 1.0)
    point_score = clamp(
        len(points) / float(ESTIMATOR_PROFILE["min_good_points"]), 0.0, 1.0
    )
    span_score = clamp(y_span / ESTIMATOR_PROFILE["min_y_span_good"], 0.0, 1.0)
    width_score = clamp(
        obs.road_width_est / ESTIMATOR_PROFILE["min_road_width_for_conf"], 0.0, 1.0
    )
    quality = (
        0.30
        + point_score * 0.20
        + span_score * 0.20
        + fit_score * 0.25
        + width_score * 0.05
    )
    confidence = obs_score * quality
    return clamp(min(confidence, obs_score + 0.20), 0.0, 1.0)


def _smooth_alpha(confidence: float) -> float:
    """Choose smoothing strength from geometry confidence."""

    low_conf = 1.0 - clamp(confidence, 0.0, 1.0)
    alpha = (
        ESTIMATOR_PROFILE["min_smooth_alpha"]
        + (
            ESTIMATOR_PROFILE["max_smooth_alpha"]
            - ESTIMATOR_PROFILE["min_smooth_alpha"]
        )
        * low_conf
        + ESTIMATOR_PROFILE["low_conf_extra_smoothing"] * low_conf
    )
    return clamp(
        alpha,
        ESTIMATOR_PROFILE["min_smooth_alpha"],
        ESTIMATOR_PROFILE["max_smooth_alpha"],
    )


def _smooth_limited(
    previous: float, current: float, alpha: float, max_delta: float
) -> float:
    """Smooth one state value and limit its frame-to-frame change."""

    smoothed = previous * alpha + current * (1.0 - alpha)
    delta = clamp(smoothed - previous, -max_delta, max_delta)
    return clamp(previous + delta, -1.0, 1.0)


def reset_estimator_state() -> None:
    """Replace the session state with a clean instance."""

    global _STATE
    _STATE = EstimatorState()


def _maybe_reset_estimator_by_timestamp(timestamp: float) -> None:
    """Reset estimator memory after a timestamp rollback or long input gap."""

    if _STATE.last_timestamp is None:
        return
    elapsed = float(timestamp) - float(_STATE.last_timestamp)
    if elapsed < 0.0 or elapsed > ESTIMATOR_PROFILE["timestamp_reset_gap"]:
        reset_estimator_state()


def _red_environment(obs: PerceptionObs) -> bool:
    """Latch the complex-track environment after consecutive visual detections."""

    observed = bool(obs.debug_flags & _RED_ENVIRONMENT_FLAG)
    if observed:
        _STATE.red_environment_streak += 1
    elif not _STATE.last_red_environment:
        _STATE.red_environment_streak = 0
    if _STATE.red_environment_streak >= _RED_ENVIRONMENT_LATCH_FRAMES:
        _STATE.last_red_environment = True
    return observed or _STATE.last_red_environment


def _measure_geometry(
    obs: PerceptionObs, line_trust: float, red_environment: bool
) -> tuple[GeometryEstimate | None, bool]:
    """Fit road geometry from a usable centerline point cloud."""

    points = _clean_points(obs.center_points)
    if (
        len(points) < ESTIMATOR_PROFILE["min_center_points"]
        or obs.confidence < ESTIMATOR_PROFILE["lost_confidence"]
    ):
        return None, True
    x_norm, progress, y_span = _normalize_points(points)
    if y_span < ESTIMATOR_PROFILE["min_y_span"]:
        return None, False

    coeffs, degree = _fit_centerline(progress, x_norm)
    lateral_error = clamp(
        _value_from_band(
            progress,
            x_norm,
            coeffs,
            progress <= ESTIMATOR_PROFILE["near_progress_max"],
            ESTIMATOR_PROFILE["near_eval_progress"],
        ),
        -1.0,
        1.0,
    )
    lookahead_error = clamp(
        _value_from_band(
            progress,
            x_norm,
            coeffs,
            progress >= ESTIMATOR_PROFILE["far_progress_min"],
            ESTIMATOR_PROFILE["far_eval_progress"],
        ),
        -1.0,
        1.0,
    )
    heading_error = _heading_from_fit(coeffs, degree)
    lateral_error, heading_error, lookahead_error = _apply_line_target(
        lateral_error,
        heading_error,
        lookahead_error,
        obs,
        line_trust,
    )
    fit_score = _fit_error_score(progress, x_norm, coeffs)
    curvature = _curvature_from_fit(
        coeffs,
        degree,
        lateral_error,
        lookahead_error,
        _curvature_trust(len(points), y_span, fit_score),
    )
    confidence = _geometry_confidence(obs, points, y_span, fit_score)
    if line_trust > 0.0:
        confidence = max(
            confidence,
            obs.line_confidence * ESTIMATOR_PROFILE["line_target_confidence_scale"],
        )
    if red_environment and confidence > ESTIMATOR_PROFILE["lost_confidence"] * 0.6:
        confidence = min(confidence + 0.08, 1.0)
    return (
        GeometryEstimate(
            lateral_error,
            heading_error,
            curvature,
            lookahead_error,
            confidence,
            _near_edge_margin(obs.left_edge_points, "left"),
            _near_edge_margin(obs.right_edge_points, "right"),
        ),
        False,
    )


def _smoothed_track(
    obs: PerceptionObs, geometry: GeometryEstimate, red_environment: bool
) -> TrackState:
    """Combine a raw geometry estimate with cross-frame smoothing and metadata."""

    alpha = _smooth_alpha(geometry.confidence)
    curve_alpha = clamp(
        ESTIMATOR_PROFILE["curve_smooth_alpha"]
        + ESTIMATOR_PROFILE["low_conf_extra_smoothing"] * (1.0 - geometry.confidence),
        ESTIMATOR_PROFILE["min_smooth_alpha"],
        ESTIMATOR_PROFILE["max_smooth_alpha"],
    )
    return TrackState(
        _smooth_limited(
            _STATE.last_track.lateral_error,
            geometry.lateral_error,
            alpha,
            ESTIMATOR_PROFILE["max_error_delta"],
        ),
        _smooth_limited(
            _STATE.last_track.heading_error,
            geometry.heading_error,
            alpha,
            ESTIMATOR_PROFILE["max_heading_delta"],
        ),
        _smooth_limited(
            _STATE.last_track.curvature,
            geometry.curvature,
            curve_alpha,
            ESTIMATOR_PROFILE["max_curvature_delta"],
        ),
        _smooth_limited(
            _STATE.last_track.lookahead_error,
            geometry.lookahead_error,
            alpha,
            ESTIMATOR_PROFILE["max_error_delta"],
        ),
        geometry.confidence,
        False,
        red_environment,
        clamp(float(obs.line_offset), -1.0, 1.0),
        clamp(float(obs.line_heading), -1.0, 1.0),
        clamp(float(obs.line_confidence), 0.0, 1.0),
        geometry.left_margin_near,
        geometry.right_margin_near,
        bool(obs.near_obstacle),
        clamp(float(obs.obstacle_x), -1.0, 1.0),
        max(float(obs.obstacle_size), 0.0),
        obs.frame_motion,
    )


def estimate_track(obs: PerceptionObs, timestamp: float) -> TrackState:
    """Convert a perception observation into a stable control-ready track state."""

    timestamp = float(timestamp)
    _maybe_reset_estimator_by_timestamp(timestamp)

    red_environment = _red_environment(obs)
    line_trust = _line_target_trust(obs, timestamp, red_environment)
    line_trust = _update_line_memory(obs, line_trust, timestamp, red_environment)
    geometry, line_only_allowed = _measure_geometry(obs, line_trust, red_environment)
    if geometry is None:
        track = (
            _line_only_track(obs, timestamp, red_environment, line_trust)
            if line_only_allowed and line_trust > 0.0
            else _lost_track(obs.confidence, red_environment, obs)
        )
        _STATE.last_track = track
        _STATE.last_timestamp = timestamp
        return track
    if geometry.confidence < ESTIMATOR_PROFILE["lost_confidence"]:
        track = _lost_track(geometry.confidence, red_environment, obs)
        _STATE.last_track = track
        _STATE.last_timestamp = timestamp
        return track
    track = _smoothed_track(obs, geometry, red_environment)
    _STATE.last_track = track
    _STATE.last_timestamp = timestamp
    return track
