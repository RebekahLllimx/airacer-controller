import sys
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.params import VISION_PROFILE
from controller.perception import (
    _build_masks,
    _filter_fallback_segments,
    _score_scan,
)


def _three_stable_scan_lines():
    centers = [(320.0, 440.0), (321.0, 400.0), (322.0, 360.0)]
    widths = [220.0, 218.0, 216.0]
    return centers, widths


def test_empty_and_saturated_masks_penalized_equally():
    centers, widths = _three_stable_scan_lines()

    empty_conf, empty_flags = _score_scan(centers, widths, texture_score=0.5, mask_fill_ratio=0.0, fallback_count=0)
    saturated_conf, saturated_flags = _score_scan(
        centers, widths, texture_score=0.5, mask_fill_ratio=0.95, fallback_count=0
    )

    assert empty_flags & 4
    assert saturated_flags & 4
    assert empty_conf < VISION_PROFILE["min_camera_confidence"]
    assert saturated_conf < VISION_PROFILE["min_camera_confidence"]
    assert abs(empty_conf - saturated_conf) < 1e-9


def test_red_world_wide_mask_is_low_confidence():
    centers, widths = _three_stable_scan_lines()

    normal_conf, normal_flags = _score_scan(
        centers,
        widths,
        texture_score=0.8,
        mask_fill_ratio=0.80,
        fallback_count=0,
        red_environment=False,
    )
    red_conf, red_flags = _score_scan(
        centers,
        widths,
        texture_score=0.8,
        mask_fill_ratio=0.80,
        fallback_count=0,
        red_environment=True,
    )

    assert red_flags & 4
    assert red_conf < normal_conf * 0.5


def _grass_with_center_strip(height=480, width=640, strip=(280, 360)):
    """Create saturated grass with one vertical strip of grey asphalt."""

    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:, :] = (0, 200, 0)

    image[:, strip[0]:strip[1]] = (82, 74, 64)

    return image


def test_grass_is_excluded_from_road_mask():
    image = _grass_with_center_strip()
    road_mask, _edge, _tex, mask_fill_ratio, _near, _x, _size = _build_masks(image)

    top = int(image.shape[0] * VISION_PROFILE["roi_top_ratio"])
    grass_cols = road_mask[top:, 0:200]
    assert grass_cols.mean() < 5.0
    strip_cols = road_mask[top:, 290:350]
    assert strip_cols.mean() > 50.0


def test_blue_checkpoint_barrier_is_bridged_as_road():
    image = _grass_with_center_strip()
    barrier_bgr = cv2.cvtColor(np.uint8([[[102, 111, 149]]]), cv2.COLOR_HSV2BGR)[0, 0]
    image[250:300, :] = barrier_bgr


    road_mask, _edge, _tex, _fill, _near, _x, _size = _build_masks(image)
    bridged = road_mask[255:295, 295:345]
    assert bridged.mean() > 50.0


def test_side_blue_guardrail_is_not_bridged_as_road():
    image = _grass_with_center_strip()
    barrier_bgr = cv2.cvtColor(np.uint8([[[102, 111, 149]]]), cv2.COLOR_HSV2BGR)[0, 0]
    image[:, 0:80] = barrier_bgr

    road_mask, _edge, _tex, _fill, _near, _x, _size = _build_masks(image)

    top = int(image.shape[0] * VISION_PROFILE["roi_top_ratio"])
    assert road_mask[top:, 0:80].mean() < 5.0


def test_full_grass_view_collapses_mask():
    image = np.zeros((480, 640, 3), dtype=np.uint8)
    image[:, :] = (0, 200, 0)
    road_mask, _edge, _tex, mask_fill_ratio, _near, _x, _size = _build_masks(image)
    assert mask_fill_ratio < 0.015


def test_narrow_far_fallback_segment_is_rejected_without_history():
    assert _filter_fallback_segments([(512, 541)], 640, 320.0, has_previous=False) == []


def test_narrow_fallback_segment_must_continue_previous_center():
    assert _filter_fallback_segments([(205, 278)], 640, 411.0, has_previous=True) == []
    assert _filter_fallback_segments([(382, 430)], 640, 411.0, has_previous=True) == [(382, 430)]
