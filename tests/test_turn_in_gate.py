import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.common import TrackState
from controller.params import get_profile
from controller.policy import _control_signals, _target_steering, reset_policy_state


def _steer(track, profile):
    reset_policy_state()
    signals = _control_signals(track, profile)
    return _target_steering(track, signals, "hard_turn", profile)


def test_turn_in_gate_reduces_steering_when_centered():
    track = TrackState(
        lateral_error=0.0, heading_error=0.0, curvature=0.6, lookahead_error=0.3,
        confidence=0.7, lost=False, red_environment=False,
    )
    profile = get_profile("no_other_cars")

    gated = _steer(track, profile)
    assert abs(gated) < 0.02


def test_turn_in_gate_fully_opens_when_corner_arrived():
    track = TrackState(
        lateral_error=0.40, heading_error=0.30, curvature=0.6, lookahead_error=0.3,
        confidence=0.7, lost=False, red_environment=False,
    )
    profile = get_profile("no_other_cars")

    arrived = _steer(track, profile)
    centered = _steer(
        TrackState(
            lateral_error=0.0, heading_error=0.0, curvature=0.6, lookahead_error=0.3,
            confidence=0.7, lost=False, red_environment=False,
        ),
        profile,
    )
    assert abs(arrived) > abs(centered) * 10.0
