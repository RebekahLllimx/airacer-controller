"""Nearby-vehicle perception used by the multi-car controller profile."""

import cv2
import numpy as np

from controller.params import COLOR_PROFILE, OPPONENT_PROFILE


def _hue_delta(hue, center):
    """Return circular distance in OpenCV's hue range."""

    delta = np.abs(hue.astype(np.float32) - float(center))
    return np.minimum(delta, 180.0 - delta)


def _vehicle_color_mask(hsv, hsv_center, hsv_tol):
    """Match one calibrated vehicle colour in HSV space."""

    return (
        (_hue_delta(hsv[:, :, 0], hsv_center[0]) <= hsv_tol[0])
        & (np.abs(hsv[:, :, 1].astype(np.float32) - hsv_center[1]) <= hsv_tol[1])
        & (np.abs(hsv[:, :, 2].astype(np.float32) - hsv_center[2]) <= hsv_tol[2])
    )


def vehicle_body_mask(image, profile=None):
    """Build a lower-frame candidate mask for calibrated vehicle bodies."""

    params = OPPONENT_PROFILE if profile is None else profile
    height, width = image.shape[:2]
    y0 = int(height * params["near_obstacle_roi_top_ratio"])
    y1 = int(height * params["near_obstacle_roi_bottom_ratio"])
    x_margin = int(width * params["near_obstacle_roi_x_margin_ratio"])
    mask = np.zeros((height, width), dtype=np.uint8)
    roi = image[y0:y1, x_margin : width - x_margin]
    if roi.size == 0:
        return mask

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    bright_body = (gray > params["near_obstacle_white_gray_min"]) & (
        saturation < params["near_obstacle_white_sat_max"]
    )
    dark_body = (gray < params["near_obstacle_black_gray_max"]) & (
        value < params["near_obstacle_black_value_max"]
    )
    color_body = np.zeros(roi.shape[:2], dtype=bool)
    for _bgr, hsv_center, hsv_tol in COLOR_PROFILE["car_body_colors"]:
        # White and black bodies are handled by luminance; low-saturation HSV
        # tolerance would otherwise absorb dark asphalt.
        if hsv_center[1] < 80:
            continue
        color_body |= _vehicle_color_mask(hsv, hsv_center, hsv_tol)

    candidate = (bright_body | dark_body | color_body).astype(np.uint8) * 255
    kernel = np.ones((5, 5), dtype=np.uint8)
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, kernel, iterations=1)
    mask[y0:y1, x_margin : width - x_margin] = candidate
    return mask


def detect_near_vehicle_obstacle_state(
    image: np.ndarray, profile: dict | None = None
) -> tuple[bool, float, float]:
    """Return nearby-vehicle presence, horizontal position, and relative size."""

    params = OPPONENT_PROFILE if profile is None else profile
    candidate = vehicle_body_mask(image, params)
    if not np.any(candidate):
        return False, 0.0, 0.0

    count, _, stats, _ = cv2.connectedComponentsWithStats(candidate, 8)
    min_area = float(params["near_obstacle_min_area"])
    min_width = float(params["near_obstacle_min_width"])
    min_height = float(params["near_obstacle_min_height"])
    best = None
    for index in range(1, count):
        left, top, comp_width, comp_height, area = stats[index]
        if area >= min_area and comp_width >= min_width and comp_height >= min_height:
            if best is None or area > best[-1]:
                best = (left, top, comp_width, comp_height, area)
    if best is None:
        return False, 0.0, 0.0

    left, _top, comp_width, _comp_height, area = best
    height, width = candidate.shape[:2]
    roi_top = int(height * params["near_obstacle_roi_top_ratio"])
    roi_bottom = int(height * params["near_obstacle_roi_bottom_ratio"])
    x_margin = int(width * params["near_obstacle_roi_x_margin_ratio"])
    roi_area = max((roi_bottom - roi_top) * max(width - 2 * x_margin, 1), 1)
    center_x = left + comp_width * 0.5
    x_norm = (center_x / max(width - 1, 1) - 0.5) * 2.0
    size = float(area) / float(roi_area)
    return True, float(np.clip(x_norm, -1.0, 1.0)), float(size)


def detect_near_vehicle_obstacle(
    image: np.ndarray, profile: dict | None = None
) -> bool:
    """Compatibility wrapper returning only nearby-vehicle presence."""

    near, _x, _size = detect_near_vehicle_obstacle_state(image, profile)
    return near
