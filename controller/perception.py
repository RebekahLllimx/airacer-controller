"""Road, lane-marking, and nearby-vehicle perception from stereo frames."""

from dataclasses import dataclass

import cv2
import numpy as np

from controller.common import PerceptionObs, clamp
from controller.opponent import detect_near_vehicle_obstacle_state
from controller.params import (
    COLOR_PROFILE,
    LINE_FOLLOW_PROFILE,
    OPPONENT_PROFILE,
    VISION_PROFILE,
)

_RED_ENVIRONMENT_FLAG = 32


@dataclass
class PerceptionState:
    """Temporal scan anchors and frame-motion memory for one session."""

    last_frame_centers: list[tuple[float, float]] | None = None
    last_frame_timestamp: float | None = None
    previous_motion_gray: np.ndarray | None = None
    previous_motion_timestamp: float | None = None


_PERCEPTION_STATE = PerceptionState()


def _save_frame_centers(centers: list[tuple[float, float]]) -> None:
    """Store current scan centers as temporal anchors for the next frame."""
    if not centers:
        _PERCEPTION_STATE.last_frame_centers = None
    else:
        _PERCEPTION_STATE.last_frame_centers = list(centers)


def _maybe_reset_perception_by_timestamp(timestamp: float | None) -> None:
    """Clear temporal anchors after timestamp rollback or a long gap."""
    if timestamp is None:
        _PERCEPTION_STATE.last_frame_centers = None
        _PERCEPTION_STATE.last_frame_timestamp = None
        return
    if _PERCEPTION_STATE.last_frame_timestamp is not None:
        elapsed = float(timestamp) - float(_PERCEPTION_STATE.last_frame_timestamp)
        if elapsed < 0.0 or elapsed > 2.0:
            _PERCEPTION_STATE.last_frame_centers = None
    _PERCEPTION_STATE.last_frame_timestamp = float(timestamp)


@dataclass
class _CameraScan:
    """Stable road, boundary, obstacle, and confidence output from one camera."""

    center_points: np.ndarray
    left_edge_points: np.ndarray
    right_edge_points: np.ndarray
    road_width_est: float
    confidence: float
    debug_flags: int = 0
    near_obstacle: bool = False
    obstacle_x: float = 0.0
    obstacle_size: float = 0.0
    red_environment: bool = False


@dataclass
class _RoadSamples:
    """Horizontal scanline samples collected from one camera frame."""

    centers: list[tuple[float, float]]
    left_edges: list[tuple[float, float]]
    right_edges: list[tuple[float, float]]
    widths: list[float]
    fallback_count: int = 0


def _empty_points() -> np.ndarray:
    return np.empty((0, 2), dtype=np.float32)


def _empty_scan(debug_flags: int = 1) -> _CameraScan:
    points = _empty_points()
    return _CameraScan(points, points, points, 0.0, 0.0, debug_flags=debug_flags)


def _empty_obs(debug_flags: int = 1) -> PerceptionObs:
    points = _empty_points()
    return PerceptionObs(points, points, points, 0.0, 0.0, debug_flags=debug_flags)


def _valid_image(image) -> bool:
    """Return whether input is a three-channel BGR image."""

    return (
        image is not None
        and hasattr(image, "shape")
        and len(image.shape) == 3
        and image.shape[2] == 3
    )


def _is_synthetic_grayscale_frame(image: np.ndarray) -> bool:
    """Detect low-chroma mock frames that contain no useful road colour."""

    sample = image[::32, ::32, :]
    if sample.size == 0:
        return False
    chroma = sample.max(axis=2) - sample.min(axis=2)
    return int(np.max(chroma)) <= 2


def _is_red_environment(image: np.ndarray) -> bool:
    """Detect the red-ground complex-track environment."""

    height = image.shape[0]
    roi = image[int(height * 0.30) : int(height * 0.88), :]
    if roi.size == 0:
        return False
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    red = ((hue < 12) | (hue > 168)) & (saturation > 80) & (value > 50)
    red_ratio = float(np.count_nonzero(red)) / max(float(red.size), 1.0)
    return red_ratio >= VISION_PROFILE["red_world_min_ratio"]


def _hue_delta(hue: np.ndarray, center: float) -> np.ndarray:
    """Return circular distance in OpenCV's hue range."""

    delta = np.abs(hue.astype(np.float32) - float(center))
    return np.minimum(delta, 180.0 - delta)


def _sampled_color_mask(
    hsv: np.ndarray, lab: np.ndarray, color_name: str
) -> np.ndarray:
    """Build a calibrated boolean mask from HSV and Lab tolerances."""

    profile = COLOR_PROFILE[color_name]
    hsv_center = profile["hsv_median"]
    hsv_tol = profile["hsv_tolerance"]
    lab_center = profile["lab_median"]
    lab_tol = profile["lab_tolerance"]
    hsv_match = (
        (_hue_delta(hsv[:, :, 0], hsv_center[0]) <= hsv_tol[0])
        & (np.abs(hsv[:, :, 1].astype(np.float32) - hsv_center[1]) <= hsv_tol[1])
        & (np.abs(hsv[:, :, 2].astype(np.float32) - hsv_center[2]) <= hsv_tol[2])
    )
    lab_match = (
        (np.abs(lab[:, :, 0] - lab_center[0]) <= lab_tol[0])
        & (np.abs(lab[:, :, 1] - lab_center[1]) <= lab_tol[1])
        & (np.abs(lab[:, :, 2] - lab_center[2]) <= lab_tol[2])
    )
    return hsv_match & lab_match


def _road_surface_mask(
    image_roi: np.ndarray, hsv_roi: np.ndarray, lab_roi: np.ndarray
) -> np.ndarray:
    """Segment dark asphalt while excluding calibrated terrain and shoulders."""

    gray_roi = cv2.cvtColor(image_roi, cv2.COLOR_BGR2GRAY)
    bgr_roi = image_roi.astype(np.float32)
    road_color = COLOR_PROFILE["road_asphalt_dark_gray"]
    b_minus_g = bgr_roi[:, :, 0] - bgr_roi[:, :, 1]
    g_minus_r = bgr_roi[:, :, 1] - bgr_roi[:, :, 2]
    road_core = (
        _sampled_color_mask(hsv_roi, lab_roi, "road_asphalt_dark_gray")
        & (gray_roi >= VISION_PROFILE["road_gray_min"])
        & (gray_roi <= VISION_PROFILE["road_gray_max"])
        & (b_minus_g >= road_color["b_minus_g_min"])
        & (b_minus_g <= road_color["b_minus_g_max"])
        & (g_minus_r >= road_color["g_minus_r_min"])
        & (g_minus_r <= road_color["g_minus_r_max"])
    )
    non_road = (
        _sampled_color_mask(hsv_roi, lab_roi, "green_grass")
        | _sampled_color_mask(hsv_roi, lab_roi, "red_ground")
        | _sampled_color_mask(hsv_roi, lab_roi, "blue_sky")
    )
    shoulder = _sampled_color_mask(hsv_roi, lab_roi, "curb_shoulder_light_gray") & (
        lab_roi[:, :, 0]
        > COLOR_PROFILE["road_asphalt_dark_gray"]["lab_median"][0] + 10.0
    )
    return (road_core & ~non_road & ~shoulder).astype(np.uint8) * 255


def _bridge_checkpoint(road_roi: np.ndarray, hsv_roi: np.ndarray) -> None:
    """Bridge wide translucent checkpoint rows without absorbing side rails."""

    candidate = (
        (hsv_roi[:, :, 0] >= VISION_PROFILE["barrier_hue_min"])
        & (hsv_roi[:, :, 0] <= VISION_PROFILE["barrier_hue_max"])
        & (hsv_roi[:, :, 1] >= VISION_PROFILE["barrier_sat_min"])
        & (hsv_roi[:, :, 2] >= VISION_PROFILE["barrier_value_min"])
        & (hsv_roi[:, :, 2] <= VISION_PROFILE["barrier_value_max"])
    )
    row_ratio = np.count_nonzero(candidate, axis=1) / max(
        float(candidate.shape[1]), 1.0
    )
    barrier_rows = row_ratio >= VISION_PROFILE["barrier_bridge_row_ratio"]
    road_roi[candidate & barrier_rows[:, None]] = 255


def _opponent_state(
    image: np.ndarray, timestamp, enabled: bool
) -> tuple[bool, float, float]:
    """Run optional near-vehicle detection after its startup guard interval."""

    if not enabled or not OPPONENT_PROFILE["enable_opponent_avoidance"]:
        return False, 0.0, 0.0
    try:
        current_time = float(timestamp)
    except (TypeError, ValueError):
        current_time = 0.0
    if current_time < OPPONENT_PROFILE["near_obstacle_min_timestamp"]:
        return False, 0.0, 0.0
    return detect_near_vehicle_obstacle_state(image, OPPONENT_PROFILE)


def _build_masks(
    image: np.ndarray,
    timestamp=None,
    enable_opp: bool = True,
) -> tuple[np.ndarray, np.ndarray, float, float, bool, float, float]:
    """Build full-frame road and edge masks plus optional opponent state."""

    height = image.shape[0]
    top = int(height * VISION_PROFILE["roi_top_ratio"])
    roi = image[top:, :, :]

    gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lab_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB).astype(np.float32)
    road_roi = _road_surface_mask(roi, hsv_roi, lab_roi)
    _bridge_checkpoint(road_roi, hsv_roi)

    kernel = np.ones((5, 5), dtype=np.uint8)
    road_roi = cv2.morphologyEx(road_roi, cv2.MORPH_OPEN, kernel, iterations=1)
    road_roi = cv2.morphologyEx(road_roi, cv2.MORPH_CLOSE, kernel, iterations=2)

    blurred = cv2.GaussianBlur(gray_roi, (5, 5), 0)
    edge_roi = cv2.Canny(blurred, 45, 120)

    road_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    edge_mask = np.zeros(image.shape[:2], dtype=np.uint8)
    road_mask[top:, :] = road_roi
    edge_mask[top:, :] = edge_roi

    texture_score = clamp(
        float(np.std(gray_roi)) / VISION_PROFILE["texture_gray_std_scale"], 0.0, 1.0
    )
    mask_fill_ratio = float(np.count_nonzero(road_roi)) / max(float(road_roi.size), 1.0)
    near_obstacle, obstacle_x, obstacle_size = _opponent_state(
        image, timestamp, enable_opp
    )
    return (
        road_mask,
        edge_mask,
        texture_score,
        mask_fill_ratio,
        near_obstacle,
        obstacle_x,
        obstacle_size,
    )


def _segments_from_active(active: np.ndarray) -> list[tuple[int, int]]:
    """Convert a one-dimensional boolean scan into inclusive segments."""

    if active.size == 0:
        return []
    padded = np.concatenate(([False], active.astype(bool), [False]))
    changes = np.flatnonzero(padded[1:] != padded[:-1])
    return [
        (int(changes[i]), int(changes[i + 1] - 1)) for i in range(0, len(changes), 2)
    ]


def _neighbors_are_road(
    road_cols: np.ndarray, left: int, right: int, profile: dict
) -> bool:
    """Require dark asphalt context on both sides of a lane-marking segment."""

    gap = int(profile["context_gap"])
    window = int(profile["context_window"])
    min_ratio = float(profile["context_min_ratio"])
    width = int(road_cols.shape[0])

    def _side_ratio(start: int, stop: int) -> float:
        start = max(int(start), 0)
        stop = min(int(stop), width)
        if stop - start < 1:
            return 0.0
        return float(np.count_nonzero(road_cols[start:stop])) / float(stop - start)

    left_ratio = _side_ratio(left - gap - window, left - gap)
    right_ratio = _side_ratio(right + gap + 1, right + gap + 1 + window)
    return left_ratio >= min_ratio and right_ratio >= min_ratio


def _camera_line_state(
    image: np.ndarray, profile: dict
) -> tuple[float, float, float] | None:
    """Estimate lane offset, heading, and confidence from one camera."""

    if not _valid_image(image):
        return None
    height, width = image.shape[:2]
    lower = int(profile["white_min"])
    chroma_max = float(profile["white_chroma_max"])
    road_dark_min = float(profile["road_dark_min"])
    road_dark_max = float(profile["road_dark_max"])
    road_dark_chroma_max = float(profile["road_dark_chroma_max"])
    y_top = int(height * profile["scan_top_ratio"])
    y_bottom = int(height * profile["scan_bottom_ratio"])
    rows = np.linspace(y_bottom, y_top, int(profile["scan_count"]), dtype=np.int32)
    row_band = int(profile["row_band"])
    min_width = float(profile["min_segment_width"])
    max_width = float(profile["max_segment_width"])
    initial_limit = float(width) * profile["initial_center_max_offset"]
    jump_limit = float(width) * profile["max_center_jump_ratio"]
    image_center = float(width) * 0.5

    points = []
    previous_center = image_center
    has_previous = False
    for y in rows:
        y0 = max(int(y) - row_band, 0)
        y1 = min(int(y) + row_band + 1, height)
        band = image[y0:y1, :, :].astype(np.int16)
        band_min = band.min(axis=2)
        band_max = band.max(axis=2)
        band_chroma = band_max - band_min
        # Markings are bright and near-neutral; blue-gray guardrails have more chroma.
        white = (band_min >= lower) & (band_chroma <= chroma_max)
        active = np.count_nonzero(white, axis=0) >= 2
        # Dark low-chroma asphalt must surround the candidate marking.
        road_dark = (
            (band_min >= road_dark_min)
            & (band_max <= road_dark_max)
            & (band_chroma <= road_dark_chroma_max)
        )
        road_cols = np.count_nonzero(road_dark, axis=0) >= 1
        candidates = []
        for left, right in _segments_from_active(active):
            segment_width = float(right - left + 1)
            if not (min_width <= segment_width <= max_width):
                continue
            if not _neighbors_are_road(road_cols, left, right, profile):
                continue
            center = (float(left) + float(right)) * 0.5
            if not has_previous and abs(center - image_center) <= initial_limit:
                candidates.append(center)
            elif has_previous and abs(center - previous_center) <= jump_limit:
                candidates.append(center)
        if not candidates:
            continue
        center = min(candidates, key=lambda value: abs(value - previous_center))
        points.append((center, float(y)))
        previous_center = center
        has_previous = True

    if len(points) < int(profile["min_points_per_camera"]):
        return None
    point_arr = np.array(points, dtype=np.float32)
    point_y = point_arr[:, 1]
    point_x = point_arr[:, 0]
    y_span = float(np.max(point_y) - np.min(point_y))
    if y_span < float(profile["min_y_span"]):
        return None
    # Medians of measured near and far thirds avoid extrapolation error in corners.
    order = np.argsort(-point_y)  # Larger y values are nearer to the vehicle.
    k = max(1, int(round(len(points) * profile["offset_near_fraction"])))
    near_x = float(np.median(point_x[order[:k]]))
    far_x = float(np.median(point_x[order[-k:]]))
    offset = (near_x - image_center) / max(image_center, 1.0)
    heading = (far_x - near_x) / max(image_center, 1.0)
    # Confidence remains independent of configured scan density.
    confidence = clamp(len(points) / float(profile["confidence_full_points"]), 0.0, 1.0)
    return clamp(offset, -1.0, 1.0), clamp(heading, -1.0, 1.0), confidence


def _startup_single_line_candidate(
    line: tuple[float, float, float] | None,
    profile: dict,
    timestamp,
) -> tuple[float, float, float] | None:
    """Accept a geometrically plausible monocular lane during startup only."""

    if line is None:
        return None
    try:
        current_time = float(timestamp)
    except (TypeError, ValueError):
        return None
    offset, heading, confidence = line
    if current_time > profile["startup_acquire_until"]:
        return None
    if confidence < profile["min_confidence"]:
        return None
    if not (
        profile["startup_offset_min"] <= offset <= profile["startup_offset_trust_max"]
    ):
        return None
    if not (
        profile["startup_heading_min"] <= heading <= profile["startup_heading_max"]
    ):
        return None
    return line


def _red_environment_single_line_candidate(
    line: tuple[float, float, float] | None,
    profile: dict,
    red_environment: bool,
) -> tuple[float, float, float] | None:
    """Provide a discounted monocular fallback in the red complex environment."""

    if line is None or not red_environment or not profile["single_camera_enable"]:
        return None
    offset, heading, confidence = line
    if confidence < profile["single_camera_min_confidence"]:
        return None
    if abs(offset) > profile["single_camera_offset_max"]:
        return None
    scaled_confidence = confidence * profile["single_camera_confidence_scale"]
    return offset, heading, clamp(scaled_confidence, 0.0, 1.0)


def _stereo_line_state(
    left_img,
    right_img,
    profile: dict,
    timestamp=None,
    red_environment: bool | None = None,
) -> tuple[float, float, float]:
    """Fuse stereo lane observations with restricted monocular fallbacks."""

    if not profile["enable"]:
        return 0.0, 0.0, 0.0
    left = _camera_line_state(left_img, profile)
    right = _camera_line_state(right_img, profile)
    if left is None or right is None:
        single = _startup_single_line_candidate(left or right, profile, timestamp)
        if single is not None:
            return single
        if red_environment is None:
            red_environment = _is_red_environment(left_img) or _is_red_environment(
                right_img
            )
        single = _red_environment_single_line_candidate(
            left or right, profile, bool(red_environment)
        )
        if single is not None:
            return single
        return 0.0, 0.0, 0.0
    confidence = min(left[2], right[2])
    if confidence < profile["min_confidence"]:
        return 0.0, 0.0, confidence
    offset = (left[0] + right[0]) * 0.5
    heading = (left[1] + right[1]) * 0.5
    return clamp(offset, -1.0, 1.0), clamp(heading, -1.0, 1.0), confidence


def _merge_close_segments(
    segments: list[tuple[int, int]], max_gap: int | None = None
) -> list[tuple[int, int]]:
    """Merge road segments separated by small marking or rail gaps."""

    if not segments:
        return []
    if max_gap is None:
        max_gap = int(VISION_PROFILE["max_segment_gap"])
    merged = [segments[0]]
    for left, right in segments[1:]:
        prev_left, prev_right = merged[-1]
        if left - prev_right - 1 <= max_gap:
            merged[-1] = (prev_left, right)
        else:
            merged.append((left, right))
    return merged


def _road_segments(
    mask: np.ndarray, y: int, row_band: int, max_gap: int | None = None
) -> list[tuple[int, int]]:
    """Extract continuous road segments from a horizontal mask band."""

    y0 = max(int(y) - row_band, 0)
    y1 = min(int(y) + row_band + 1, mask.shape[0])
    band = mask[y0:y1, :]
    min_hits = max(1, int(np.ceil(band.shape[0] * 0.45)))
    active = np.count_nonzero(band, axis=0) >= min_hits
    return _merge_close_segments(_segments_from_active(active), max_gap=max_gap)


def _edge_fallback_segments(
    edge_mask: np.ndarray, y: int, row_band: int
) -> list[tuple[int, int]]:
    """Infer fallback corridors between adjacent Canny edge segments."""

    y0 = max(int(y) - row_band, 0)
    y1 = min(int(y) + row_band + 1, edge_mask.shape[0])
    band = edge_mask[y0:y1, :]
    active = np.count_nonzero(band, axis=0) > 0
    edge_segments = _segments_from_active(active)
    if len(edge_segments) < 2:
        return []

    edge_centers = [int((left + right) * 0.5) for left, right in edge_segments]
    return [
        (edge_centers[index], edge_centers[index + 1])
        for index in range(len(edge_centers) - 1)
    ]


def _filter_segments(
    segments: list[tuple[int, int]],
    width: int,
    max_width_ratio: float | None = None,
) -> list[tuple[int, int]]:
    """Filter candidate road segments by plausible width."""

    min_width = float(VISION_PROFILE["min_segment_width"])
    if max_width_ratio is None:
        max_width_ratio = VISION_PROFILE["max_segment_width_ratio"]
    max_width = float(width) * float(max_width_ratio)
    filtered = []
    for left, right in segments:
        segment_width = float(right - left + 1)
        if min_width <= segment_width <= max_width:
            filtered.append((left, right))
    return filtered


def _filter_fallback_segments(
    segments: list[tuple[int, int]],
    width: int,
    previous_center: float,
    has_previous: bool,
) -> list[tuple[int, int]]:
    """Reject narrow edge corridors unless they continue a credible center."""

    if not segments:
        return []

    min_width = float(VISION_PROFILE["fallback_min_segment_width"])
    initial_center_limit = (
        float(width) * VISION_PROFILE["fallback_initial_center_max_offset"]
    )
    narrow_jump_limit = float(width) * VISION_PROFILE["fallback_narrow_jump_max_ratio"]
    image_center = float(width) * 0.5

    filtered = []
    for left, right in segments:
        segment_width = float(right - left + 1)
        center = (float(left) + float(right)) * 0.5
        if segment_width >= min_width:
            filtered.append((left, right))
        elif not has_previous and abs(center - image_center) <= initial_center_limit:
            filtered.append((left, right))
        elif has_previous and abs(center - previous_center) <= narrow_jump_limit:
            filtered.append((left, right))
    return filtered


def _localize_wide_segment(
    segment: tuple[int, int],
    previous_center: float,
    width: int,
    enabled: bool = True,
) -> tuple[int, int]:
    """Crop an over-wide mask region around the preceding scan center."""

    left, right = segment
    segment_width = float(right - left + 1)
    if not enabled:
        return segment
    if segment_width <= float(width) * VISION_PROFILE["wide_segment_localize_ratio"]:
        return segment

    target_width = max(
        VISION_PROFILE["min_segment_width"],
        float(width) * VISION_PROFILE["wide_segment_window_ratio"],
    )
    half = target_width * 0.5
    center = previous_center
    center = clamp(center, float(left) + half, float(right) - half)
    localized_left = max(left, int(round(center - half)))
    localized_right = min(right, int(round(center + half)))
    if localized_right <= localized_left:
        return segment
    return localized_left, localized_right


def _pick_segment(
    road_mask: np.ndarray,
    edge_mask: np.ndarray,
    y: int,
    previous_center: float,
    has_previous: bool,
    near_obstacle: bool,
    wide_localize_enabled: bool,
) -> tuple[tuple[int, int] | None, bool]:
    """Choose the most coherent road corridor, using edges only as fallback."""

    width = road_mask.shape[1]
    row_band = int(VISION_PROFILE["row_band"])
    max_width_ratio = (
        VISION_PROFILE["max_segment_width_ratio"]
        if wide_localize_enabled
        else VISION_PROFILE["early_max_segment_width_ratio"]
    )
    max_gap = None
    if near_obstacle and y >= int(
        road_mask.shape[0] * OPPONENT_PROFILE["near_obstacle_scan_y_ratio"]
    ):
        max_gap = int(OPPONENT_PROFILE["near_obstacle_segment_gap"])
    candidates = _filter_segments(
        _road_segments(road_mask, y, row_band, max_gap=max_gap),
        width,
        max_width_ratio=max_width_ratio,
    )
    used_fallback = False
    if not candidates:
        candidates = _filter_segments(
            _edge_fallback_segments(edge_mask, y, row_band),
            width,
            max_width_ratio=max_width_ratio,
        )
        candidates = _filter_fallback_segments(
            candidates, width, previous_center, has_previous
        )
        used_fallback = bool(candidates)
    if not candidates:
        return None, used_fallback

    best = min(
        candidates, key=lambda item: abs(((item[0] + item[1]) * 0.5) - previous_center)
    )

    # Temporal anchors stabilize the choice when an over-wide mask contains branches.
    temporal_anchor: float | None = None
    if wide_localize_enabled and _PERCEPTION_STATE.last_frame_centers:
        best_y_dist = float("inf")
        for cx, cy in _PERCEPTION_STATE.last_frame_centers:
            dist = abs(cy - float(y))
            if dist < best_y_dist:
                best_y_dist = dist
                temporal_anchor = cx
        best_candidate_width = max(c[1] - c[0] for c in candidates)
        temporal_window = float(width) * VISION_PROFILE.get(
            "temporal_anchor_window_ratio", 0.38
        )
        if (
            best_candidate_width
            > float(width) * VISION_PROFILE["wide_segment_localize_ratio"] * 0.7
        ):
            if (
                temporal_anchor is not None
                and abs(temporal_anchor - previous_center) > temporal_window * 0.5
            ):
                previous_center = previous_center * 0.35 + temporal_anchor * 0.65
                # Re-evaluate candidates after replacing the local center anchor.
                best = min(
                    candidates,
                    key=lambda item: abs(((item[0] + item[1]) * 0.5) - previous_center),
                )

    best = _localize_wide_segment(
        best, previous_center, width, enabled=wide_localize_enabled
    )
    center = (best[0] + best[1]) * 0.5
    max_jump = float(width) * VISION_PROFILE["max_center_jump_ratio"]
    if has_previous and abs(center - previous_center) > max_jump:
        return None, used_fallback
    return best, used_fallback


def _score_scan(
    centers: list[tuple[float, float]],
    widths: list[float],
    texture_score: float,
    mask_fill_ratio: float,
    fallback_count: int,
    red_environment: bool = False,
) -> tuple[float, int]:
    """Score one camera scan from coverage, stability, texture, and fallback use."""

    debug_flags = 0
    scan_count = float(VISION_PROFILE["scan_count"])
    valid_count = len(centers)
    valid_ratio = valid_count / max(scan_count, 1.0)

    width_arr = np.array(widths, dtype=np.float32)
    width_median = max(float(np.median(width_arr)), 1.0)
    width_stability = clamp(1.0 - float(np.std(width_arr)) / width_median, 0.0, 1.0)

    center_arr = np.array([point[0] for point in centers], dtype=np.float32)
    if center_arr.size >= 2:
        center_jump = np.abs(np.diff(center_arr))
        center_stability = clamp(
            1.0
            - float(np.mean(center_jump))
            / (640.0 * VISION_PROFILE["max_center_jump_ratio"]),
            0.0,
            1.0,
        )
    else:
        center_stability = 0.0

    confidence = (
        valid_ratio * 0.38
        + width_stability * 0.22
        + center_stability * 0.22
        + texture_score * 0.18
    )

    min_valid = int(VISION_PROFILE["min_valid_scans"])
    if valid_count < min_valid:
        confidence *= valid_count / max(float(min_valid), 1.0)
        debug_flags |= 1
    if mask_fill_ratio < 0.015:
        confidence *= VISION_PROFILE["empty_mask_confidence_scale"]
        debug_flags |= 4
    elif mask_fill_ratio > 0.92:
        confidence *= VISION_PROFILE["saturated_mask_confidence_scale"]
        debug_flags |= 4
    elif red_environment and mask_fill_ratio > VISION_PROFILE["red_mask_fill_warning"]:
        confidence *= VISION_PROFILE["red_mask_fill_confidence_scale"]
        debug_flags |= 4
    if fallback_count:
        confidence *= max(0.55, 1.0 - 0.06 * fallback_count)
        debug_flags |= 2

    return clamp(confidence, 0.0, 1.0), debug_flags


def _grayscale_placeholder_scan(
    image: np.ndarray, timestamp, enable_opp: bool
) -> _CameraScan:
    """Return a minimal observation for synthetic grayscale placeholder frames."""

    _save_frame_centers([])
    scan = _empty_scan(debug_flags=4)
    scan.near_obstacle, scan.obstacle_x, scan.obstacle_size = _opponent_state(
        image, timestamp, enable_opp
    )
    return scan


def _collect_road_samples(
    road_mask: np.ndarray,
    edge_mask: np.ndarray,
    near_obstacle: bool,
    wide_localize_enabled: bool,
) -> _RoadSamples:
    """Trace a coherent road segment from near to far scanlines."""

    height, width = road_mask.shape
    rows = np.linspace(
        int(height * VISION_PROFILE["scan_bottom_ratio"]),
        int(height * VISION_PROFILE["scan_top_ratio"]),
        int(VISION_PROFILE["scan_count"]),
        dtype=np.int32,
    )
    samples = _RoadSamples([], [], [], [])
    previous_center = width * 0.5
    has_previous = False
    for y in rows:
        segment, used_fallback = _pick_segment(
            road_mask,
            edge_mask,
            int(y),
            previous_center,
            has_previous,
            near_obstacle,
            wide_localize_enabled,
        )
        if segment is None:
            continue
        left, right = segment
        center = (left + right) * 0.5
        samples.centers.append((center, float(y)))
        samples.left_edges.append((float(left), float(y)))
        samples.right_edges.append((float(right), float(y)))
        samples.widths.append(float(right - left + 1))
        previous_center = center
        has_previous = True
        samples.fallback_count += int(used_fallback)
    return samples


def _completed_scan(
    samples: _RoadSamples,
    texture_score: float,
    mask_fill_ratio: float,
    near_obstacle: bool,
    obstacle_x: float,
    obstacle_size: float,
    red_environment: bool,
) -> _CameraScan:
    """Convert collected scanlines into a scored camera observation."""

    if not samples.centers:
        _save_frame_centers([])
        debug_flags = 4 if mask_fill_ratio < 0.015 or mask_fill_ratio > 0.92 else 1
        scan = _empty_scan(debug_flags=debug_flags)
        scan.near_obstacle = bool(near_obstacle)
        scan.obstacle_x = float(obstacle_x)
        scan.obstacle_size = float(obstacle_size)
        scan.red_environment = bool(red_environment)
        return scan

    confidence, debug_flags = _score_scan(
        samples.centers,
        samples.widths,
        texture_score,
        mask_fill_ratio,
        samples.fallback_count,
        red_environment=red_environment,
    )
    if red_environment:
        debug_flags |= _RED_ENVIRONMENT_FLAG
    _save_frame_centers(samples.centers)
    return _CameraScan(
        np.array(samples.centers, dtype=np.float32),
        np.array(samples.left_edges, dtype=np.float32),
        np.array(samples.right_edges, dtype=np.float32),
        float(np.median(np.array(samples.widths, dtype=np.float32))),
        confidence,
        debug_flags=debug_flags,
        near_obstacle=bool(near_obstacle),
        obstacle_x=float(obstacle_x),
        obstacle_size=float(obstacle_size),
        red_environment=bool(red_environment),
    )


def _scan_image(
    image: np.ndarray, timestamp=None, enable_opp: bool = True
) -> _CameraScan:
    """Track one camera's road corridor from near to far scanlines."""

    if not _valid_image(image):
        _save_frame_centers([])
        return _empty_scan()

    _maybe_reset_perception_by_timestamp(timestamp)
    if _is_synthetic_grayscale_frame(image):
        return _grayscale_placeholder_scan(image, timestamp, enable_opp)
    (
        road_mask,
        edge_mask,
        texture_score,
        mask_fill_ratio,
        near_obstacle,
        obstacle_x,
        obstacle_size,
    ) = _build_masks(
        image,
        timestamp,
        enable_opp,
    )
    red_environment = _is_red_environment(image)
    samples = _collect_road_samples(
        road_mask,
        edge_mask,
        near_obstacle,
        wide_localize_enabled=red_environment,
    )
    return _completed_scan(
        samples,
        texture_score,
        mask_fill_ratio,
        near_obstacle,
        obstacle_x,
        obstacle_size,
        red_environment,
    )


def _usable(scan: _CameraScan) -> bool:
    """Return whether one camera scan is reliable enough for fusion."""

    return (
        scan.center_points.size > 0
        and scan.confidence >= VISION_PROFILE["min_camera_confidence"]
    )


def _near_center(scan: _CameraScan) -> float:
    """Return the scan center nearest to the vehicle."""

    if scan.center_points.size == 0:
        return 320.0
    return float(scan.center_points[0, 0])


def _to_obs(scan: _CameraScan, debug_flags: int | None = None) -> PerceptionObs:
    """Convert one camera scan into the public perception contract."""

    flags = scan.debug_flags if debug_flags is None else debug_flags
    return PerceptionObs(
        scan.center_points,
        scan.left_edge_points,
        scan.right_edge_points,
        scan.road_width_est,
        clamp(scan.confidence, 0.0, 1.0),
        debug_flags=flags,
        near_obstacle=bool(scan.near_obstacle),
        obstacle_x=float(scan.obstacle_x),
        obstacle_size=float(scan.obstacle_size),
    )


def _fused_obstacle(
    left_scan: _CameraScan, right_scan: _CameraScan
) -> tuple[bool, float, float]:
    """Fuse stereo opponent position using component size as confidence."""

    candidates = [
        scan
        for scan in (left_scan, right_scan)
        if scan.near_obstacle and scan.obstacle_size > 0.0
    ]
    if not candidates:
        return bool(left_scan.near_obstacle or right_scan.near_obstacle), 0.0, 0.0
    total = sum(max(float(scan.obstacle_size), 1e-6) for scan in candidates)
    x = (
        sum(
            float(scan.obstacle_x) * max(float(scan.obstacle_size), 1e-6)
            for scan in candidates
        )
        / total
    )
    size = max(float(scan.obstacle_size) for scan in candidates)
    return True, clamp(x, -1.0, 1.0), size


def _with_fused_obstacle(
    obs: PerceptionObs, left_scan: _CameraScan, right_scan: _CameraScan
) -> PerceptionObs:
    """Attach a confidence-weighted stereo obstacle observation."""

    obs.near_obstacle, obs.obstacle_x, obs.obstacle_size = _fused_obstacle(
        left_scan, right_scan
    )
    return obs


def _merge_compatible_scans(
    left_scan: _CameraScan, right_scan: _CameraScan
) -> PerceptionObs:
    """Merge two geometrically compatible camera scans."""

    center_points = np.concatenate(
        [left_scan.center_points, right_scan.center_points], axis=0
    )
    left_edge_points = np.concatenate(
        [left_scan.left_edge_points, right_scan.left_edge_points], axis=0
    )
    right_edge_points = np.concatenate(
        [left_scan.right_edge_points, right_scan.right_edge_points], axis=0
    )
    weights = np.array(
        [len(left_scan.center_points), len(right_scan.center_points)], dtype=np.float32
    )
    confidence = float(
        np.average(
            np.array([left_scan.confidence, right_scan.confidence], dtype=np.float32),
            weights=weights,
        )
    )
    road_width = float(
        np.average(
            np.array(
                [left_scan.road_width_est, right_scan.road_width_est],
                dtype=np.float32,
            ),
            weights=weights,
        )
    )
    obs = PerceptionObs(
        center_points,
        left_edge_points,
        right_edge_points,
        road_width,
        clamp(confidence, 0.0, 1.0),
        debug_flags=left_scan.debug_flags | right_scan.debug_flags,
    )
    return _with_fused_obstacle(obs, left_scan, right_scan)


def _fuse_scans(left_scan: _CameraScan, right_scan: _CameraScan) -> PerceptionObs:
    """Select or merge stereo road scans according to confidence and agreement."""

    left_ok = _usable(left_scan)
    right_ok = _usable(right_scan)
    if left_ok and not right_ok:
        return _with_fused_obstacle(_to_obs(left_scan), left_scan, right_scan)
    if right_ok and not left_ok:
        return _with_fused_obstacle(_to_obs(right_scan), left_scan, right_scan)
    if not left_ok and not right_ok:
        obs = _empty_obs(debug_flags=left_scan.debug_flags | right_scan.debug_flags | 1)
        return _with_fused_obstacle(obs, left_scan, right_scan)

    center_gap = abs(_near_center(left_scan) - _near_center(right_scan))
    merge_gap = 640.0 * VISION_PROFILE["fusion_merge_gap"]
    merge_confidence = VISION_PROFILE["fusion_merge_min_confidence"]
    confidence_gap = abs(left_scan.confidence - right_scan.confidence)
    if (
        center_gap >= merge_gap
        or left_scan.confidence < merge_confidence
        or right_scan.confidence < merge_confidence
    ):
        chosen = (
            left_scan if left_scan.confidence >= right_scan.confidence else right_scan
        )
        flags = chosen.debug_flags
        if center_gap > 640.0 * VISION_PROFILE["fusion_max_offset_gap"]:
            flags |= 8
        if confidence_gap < VISION_PROFILE["fusion_confidence_margin"]:
            flags |= 16
        obs = _to_obs(chosen, debug_flags=flags)
        return _with_fused_obstacle(obs, left_scan, right_scan)
    return _merge_compatible_scans(left_scan, right_scan)


def _with_line_state(
    obs: PerceptionObs,
    left_img,
    right_img,
    timestamp=None,
    red_environment: bool | None = None,
) -> PerceptionObs:
    """Attach stereo lane state independently of road-mask confidence."""

    if (
        _valid_image(left_img)
        and _valid_image(right_img)
        and _is_synthetic_grayscale_frame(left_img)
        and _is_synthetic_grayscale_frame(right_img)
    ):
        line_offset, line_heading, line_confidence = 0.0, 0.0, 0.0
    else:
        line_offset, line_heading, line_confidence = _stereo_line_state(
            left_img,
            right_img,
            LINE_FOLLOW_PROFILE,
            timestamp,
            red_environment=red_environment,
        )
    obs.line_offset = line_offset
    obs.line_heading = line_heading
    obs.line_confidence = line_confidence
    return obs


def reset_perception_state() -> None:
    """Replace all perception memory with a clean session state."""

    global _PERCEPTION_STATE
    _PERCEPTION_STATE = PerceptionState()


def extract_observation(
    left_img, right_img, timestamp=None, profile=None
) -> PerceptionObs:
    """Extract a fused track observation from the stereo camera frames.

    ``profile`` controls whether opponent detection and frame-motion blockage
    estimation execute. Passing ``None`` retains the legacy global switch used
    by direct tests and offline replay. Each camera is scanned independently;
    one valid side can stand alone, consistent sides are fused, and conflicting
    sides are resolved in favour of the higher-confidence observation. The solo
    profile bypasses every traffic-specific perception path.
    """

    if profile is None:
        enable_opp = bool(OPPONENT_PROFILE["enable_opponent_avoidance"])
    else:
        enable_opp = bool(profile.get("enable_opponent", False))

    left_scan = (
        _scan_image(left_img, timestamp, enable_opp)
        if _valid_image(left_img)
        else _empty_scan()
    )
    right_scan = (
        _scan_image(right_img, timestamp, enable_opp)
        if _valid_image(right_img)
        else _empty_scan()
    )
    obs = _fuse_scans(left_scan, right_scan)
    obs = _with_line_state(
        obs,
        left_img,
        right_img,
        timestamp,
        red_environment=bool(left_scan.red_environment or right_scan.red_environment),
    )
    # Frame-motion blockage detection belongs only to the multi-car profile.
    if enable_opp:
        obs.frame_motion = _update_frame_motion(left_img, timestamp)
    return obs


def _update_frame_motion(image, timestamp=None) -> float:
    """Estimate downsampled grayscale frame motion as mean absolute difference."""

    if not _valid_image(image):
        return 100.0
    small = cv2.resize(
        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (64, 48), interpolation=cv2.INTER_AREA
    )
    if (
        _PERCEPTION_STATE.previous_motion_timestamp is not None
        and timestamp is not None
        and float(timestamp) < _PERCEPTION_STATE.previous_motion_timestamp - 1e-6
    ):
        _PERCEPTION_STATE.previous_motion_gray = None  # Clear history after simulator restart.
    prev = _PERCEPTION_STATE.previous_motion_gray
    _PERCEPTION_STATE.previous_motion_gray = small
    _PERCEPTION_STATE.previous_motion_timestamp = None if timestamp is None else float(timestamp)
    if prev is None:
        return 100.0
    return float(np.mean(np.abs(small.astype(np.int16) - prev.astype(np.int16))))


def reset_frame_motion_state() -> None:
    """Clear only frame-motion memory while retaining road anchors."""

    _PERCEPTION_STATE.previous_motion_gray = None
    _PERCEPTION_STATE.previous_motion_timestamp = None
