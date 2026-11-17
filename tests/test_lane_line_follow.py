import sys
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.params import LINE_FOLLOW_PROFILE
from controller.perception import _camera_line_state, _stereo_line_state

_ASPHALT = (82, 74, 64)

_RED_GROUND = (75, 74, 168)



def _line_image(x_values):
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = (60, 60, 60)
    rows = np.linspace(410, 240, len(x_values), dtype=np.int32)
    for x, y in zip(x_values, rows):
        cv2.rectangle(image, (int(x) - 5, int(y) - 10), (int(x) + 5, int(y) + 10), (220, 220, 220), -1)
    return image


def _vertical_bar_image(x_center, color, *, red_from=None):
    """Draw a bright barrier-like post over asphalt, optionally beside red ground."""

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = _ASPHALT
    if red_from is not None:
        image[:, int(red_from):] = _RED_GROUND
    cv2.rectangle(image, (int(x_center) - 7, 225), (int(x_center) + 7, 420), color, -1)
    return image


def _curved_centerline_image(near_x, far_x):
    """Draw a continuous near-white centre line curving through asphalt."""

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = _ASPHALT
    for y in range(230, 414):
        frac = (412 - y) / (412 - 230)
        x = int(near_x + (far_x - near_x) * frac)
        cv2.rectangle(image, (x - 4, y), (x + 4, y + 1), (235, 235, 235), -1)
    return image


def _sparse_dashed_curve_image(points):
    """Draw sparse lane dashes resembling the complex world's first left turn."""

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = _ASPHALT
    for x, y in points:
        cv2.rectangle(image, (int(x) - 5, int(y) - 8), (int(x) + 5, int(y) + 8), (235, 235, 235), -1)
    return image


def _red_scene_sparse_dashed_curve_image(points):
    """Draw sparse lane dashes on asphalt against the world's red background."""

    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = _RED_GROUND
    image[220:430, :] = _ASPHALT
    for x, y in points:
        cv2.rectangle(image, (int(x) - 5, int(y) - 8), (int(x) + 5, int(y) + 8), (235, 235, 235), -1)
    return image


def test_symmetric_stereo_line_has_near_zero_correction():
    left = _line_image([350, 348, 346, 344, 342, 340])
    right = _line_image([290, 292, 294, 296, 298, 300])

    offset, _heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE)

    assert confidence >= LINE_FOLLOW_PROFILE["min_confidence"]
    assert abs(offset) < 0.03


def test_stereo_line_right_of_car_steers_right():
    left = _line_image([390, 388, 386, 384, 382, 380])
    right = _line_image([330, 332, 334, 336, 338, 340])

    offset, heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE)

    assert confidence >= LINE_FOLLOW_PROFILE["min_confidence"]
    assert offset * LINE_FOLLOW_PROFILE["offset_gain"] + heading * LINE_FOLLOW_PROFILE["heading_gain"] > 0.04


def test_startup_single_camera_right_line_is_accepted():
    left = _line_image([440, 450, 460, 470, 480, 490])
    right = np.zeros((480, 640, 3), dtype=np.uint8)

    offset, heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE, timestamp=1.5)

    assert confidence >= LINE_FOLLOW_PROFILE["min_confidence"]
    assert LINE_FOLLOW_PROFILE["startup_offset_min"] <= offset <= LINE_FOLLOW_PROFILE["startup_offset_trust_max"]
    assert LINE_FOLLOW_PROFILE["startup_heading_min"] <= heading <= LINE_FOLLOW_PROFILE["startup_heading_max"]


def test_startup_single_camera_left_line_is_rejected():
    left = _line_image([180, 190, 200, 210, 220, 230])
    right = np.zeros((480, 640, 3), dtype=np.uint8)

    offset, heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE, timestamp=1.5)

    assert offset == 0.0
    assert heading == 0.0
    assert confidence == 0.0


def test_single_camera_line_after_startup_is_rejected_outside_red_environment():
    left = _line_image([420, 398, 360, 310])
    right = np.zeros((480, 640, 3), dtype=np.uint8)

    offset, heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE, timestamp=32.0)

    assert offset == 0.0
    assert heading == 0.0
    assert confidence == 0.0


def test_red_environment_single_camera_curve_line_is_low_confidence_fallback():
    left = _red_scene_sparse_dashed_curve_image([
        (420, 399),
        (365, 357),
        (280, 315),
    ])
    right = np.zeros((480, 640, 3), dtype=np.uint8)
    right[:, :] = _RED_GROUND

    offset, heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE, timestamp=32.0)

    assert confidence >= LINE_FOLLOW_PROFILE["min_confidence"]
    assert confidence < 0.80
    assert offset > 0.10
    assert heading < 0.0


def test_bright_white_post_at_road_red_edge_is_rejected():
    """Reject a bright inner barrier post when its far side is red ground."""

    image = _vertical_bar_image(500, (245, 245, 245), red_from=486)

    assert _camera_line_state(image, LINE_FOLLOW_PROFILE) is None


def test_same_bright_bar_with_road_on_both_sides_is_detected():
    """Accept the same bright stripe when road context exists on both sides."""

    image = _vertical_bar_image(500, (245, 245, 245))


    result = _camera_line_state(image, LINE_FOLLOW_PROFILE)

    assert result is not None


def test_bluish_bright_bar_rejected_by_chroma():
    """Reject a bright blue-grey stripe whose chroma is inconsistent with a white line."""

    image = _vertical_bar_image(320, (255, 180, 150))

    assert _camera_line_state(image, LINE_FOLLOW_PROFILE) is None


def test_curved_centerline_in_road_is_recalled_without_inflated_offset():
    """Recover a right-curving centre line without inflating its near-field offset."""

    left = _curved_centerline_image(near_x=320, far_x=372)
    right = _curved_centerline_image(near_x=320, far_x=372)

    offset, heading, confidence = _stereo_line_state(left, right, LINE_FOLLOW_PROFILE, timestamp=50.0)

    assert confidence >= LINE_FOLLOW_PROFILE["min_confidence"]
    assert abs(offset) < 0.15

    assert heading > 0.0



def test_dense_scan_recalls_sparse_dashes_that_legacy_rows_miss():
    """Recover sparse curved dashes with the denser twelve-row scan pattern."""

    image = _sparse_dashed_curve_image([
        (460, 420),
        (430, 378),
        (398, 315),
        (366, 252),
    ])
    legacy = dict(LINE_FOLLOW_PROFILE)
    legacy["scan_top_ratio"] = 0.50
    legacy["scan_bottom_ratio"] = 0.92
    legacy["scan_count"] = 5

    assert _camera_line_state(image, legacy) is None
    result = _camera_line_state(image, LINE_FOLLOW_PROFILE)

    assert result is not None
    offset, heading, confidence = result
    assert 0.35 <= offset <= LINE_FOLLOW_PROFILE["offset_trust_max"]
    assert heading < 0.0
    assert confidence >= LINE_FOLLOW_PROFILE["min_confidence"]
