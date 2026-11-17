"""Platform-compatible entry point for the modular local controller."""

from controller.common import clamp_cmd
from controller.estimator import estimate_track
from controller.policy import decide_control
from controller.perception import extract_observation
from controller.params import get_profile

# ``build_submission.py`` injects the requested solo or traffic profile. Both
# profiles share the driving core; traffic perception and recovery are gated.
PROFILE = "no_other_cars"


def control(left_img, right_img, timestamp):
    """Run perception, estimation, policy, and final output clamping.

    The active profile is passed through the full pipeline so the solo profile
    does not execute opponent detection, avoidance, reverse recovery, or visual
    motion blockage logic.
    """

    try:
        profile = get_profile(PROFILE)
        obs = extract_observation(left_img, right_img, timestamp, profile=profile)
        track = estimate_track(obs, timestamp)
        cmd = decide_control(track, timestamp, mode=PROFILE)
        steering, speed = clamp_cmd(cmd)
        return steering, speed
    except Exception:
        return 0.0, 0.0
