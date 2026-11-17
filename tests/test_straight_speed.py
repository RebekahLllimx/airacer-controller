import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.common import TrackState
from controller.params import get_profile
from controller.policy import _control_signals, _target_speed


def test_straight_line_boost_overrides_low_confidence_recovering_cap():
    profile = get_profile("no_other_cars")
    track = TrackState(
        lateral_error=0.0, heading_error=0.0, curvature=0.0, lookahead_error=0.0,
        confidence=0.15, lost=False, red_environment=True,
    )
    signals = _control_signals(track, profile)
    speed = _target_speed(track, signals, "recovering", 0.0, 5.0, profile)
    assert speed >= profile["straight_speed"] - 1e-6


def test_noisy_heading_does_not_block_straight_boost():
    profile = get_profile("no_other_cars")
    track = TrackState(
        lateral_error=0.02, heading_error=0.18, curvature=0.02, lookahead_error=0.04,
        confidence=0.70, lost=False, red_environment=True,
    )
    signals = _control_signals(track, profile)
    speed = _target_speed(track, signals, "cruise", 0.0, 5.0, profile)
    assert speed >= profile["straight_speed"] - 1e-6


def test_straight_boost_not_applied_in_corner():
    profile = get_profile("no_other_cars")
    track = TrackState(
        lateral_error=0.05, heading_error=0.2, curvature=0.6, lookahead_error=0.4,
        confidence=0.7, lost=False, red_environment=True,
    )
    signals = _control_signals(track, profile)
    speed = _target_speed(track, signals, "hard_turn", 0.4, 5.0, profile)
    assert speed < profile["straight_speed"]


def test_lost_without_straight_memory_still_slow():
    profile = get_profile("no_other_cars")
    track = TrackState(0.0, 0.0, 0.0, 0.0, 0.05, True, True)
    signals = _control_signals(track, profile)
    speed = _target_speed(track, signals, "lost", 0.0, 5.0, profile)
    assert abs(speed - profile["lost_speed"]) < 1e-6


def test_recent_straight_lost_frame_keeps_fast_coast():
    profile = get_profile("no_other_cars")
    track = TrackState(0.0, 0.0, 0.0, 0.0, 0.05, True, True)
    signals = _control_signals(track, profile)
    speed = _target_speed(
        track,
        signals,
        "lost",
        0.0,
        5.0,
        profile,
        straight_memory_active=True,
    )
    assert abs(speed - profile["straight_lost_speed"]) < 1e-6
