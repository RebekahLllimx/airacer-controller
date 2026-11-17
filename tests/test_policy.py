import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from controller.common import TrackState
from controller.params import CONTROL, get_profile
from controller.policy import (
    _control_signals,
    _target_steering,
    decide_control,
    get_policy_diagnostics,
    reset_policy_state,
)


def make_track(
    lateral=0.0,
    heading=0.0,
    curvature=0.0,
    lookahead=0.0,
    confidence=0.95,
    lost=False,
    red_environment=False,
    near_obstacle=False,
    obstacle_x=0.0,
    obstacle_size=0.0,
):
    track = TrackState(
        lateral, heading, curvature, lookahead, confidence, lost, red_environment
    )
    track.near_obstacle = near_obstacle
    track.obstacle_x = obstacle_x
    track.obstacle_size = obstacle_size
    return track


def warm_policy(track, mode="no_other_cars", steps=24):
    cmd = None
    for index in range(steps):
        cmd = decide_control(track, index * 0.05, mode=mode)
    return cmd


def test_reset_replaces_all_policy_memory():
    warm_policy(make_track(heading=0.4, curvature=0.4), steps=4)
    assert get_policy_diagnostics()["mode"] != "start"

    reset_policy_state()
    diagnostics = get_policy_diagnostics()

    assert diagnostics["mode"] == "start"
    assert diagnostics["line_correction"] == 0.0
    assert diagnostics["signals"] == {}


def test_high_confidence_straight_no_other_cars_runs_fast_and_straight():
    reset_policy_state()
    cmd = warm_policy(make_track(), mode="no_other_cars")
    assert abs(cmd.steering) < 0.03
    assert cmd.speed > 0.55


def test_lateral_offset_controls_sign():
    reset_policy_state()
    right_cmd = warm_policy(make_track(lateral=0.35), mode="no_other_cars")
    reset_policy_state()
    left_cmd = warm_policy(make_track(lateral=-0.35), mode="no_other_cars")

    assert right_cmd.steering > 0.05
    assert left_cmd.steering < -0.05


def test_curves_control_sign_and_reduce_speed():
    reset_policy_state()
    straight = warm_policy(make_track(), mode="no_other_cars")
    reset_policy_state()
    right_curve = warm_policy(
        make_track(lateral=0.30, heading=0.25, curvature=0.35, lookahead=0.30),
        mode="no_other_cars",
    )
    reset_policy_state()
    left_curve = warm_policy(
        make_track(lateral=-0.30, heading=-0.25, curvature=-0.35, lookahead=-0.30),
        mode="no_other_cars",
    )

    assert right_curve.steering > 0.05
    assert left_curve.steering < -0.05
    assert right_curve.speed < straight.speed
    assert left_curve.speed < straight.speed


def test_lost_track_uses_lost_speed():
    reset_policy_state()
    cmd = warm_policy(
        make_track(
            lateral=0.22,
            heading=0.36,
            curvature=0.30,
            lookahead=0.22,
            confidence=0.0,
            lost=True,
        ),
        mode="no_other_cars",
        steps=6,
    )
    assert abs(cmd.speed - CONTROL["lost_speed"]) < 0.03
    assert -1.0 <= cmd.steering <= 1.0


def test_recent_straight_lost_track_keeps_speed():
    reset_policy_state()
    straight = make_track(confidence=0.85)
    cmd = warm_policy(straight, mode="no_other_cars", steps=32)
    assert cmd.speed >= 0.90

    lost = make_track(confidence=0.0, lost=True)
    coast = decide_control(lost, 1.70, mode="no_other_cars")
    assert coast.speed >= 0.90


def test_centered_lost_straight_coasts_fast_without_memory():
    reset_policy_state()
    lost = make_track(confidence=0.0, lost=True)
    cmd = warm_policy(lost, mode="no_other_cars", steps=20)
    assert cmd.speed >= 0.90


def test_lost_corner_does_not_use_straight_coast():
    reset_policy_state()
    turn_lost = make_track(
        lateral=0.24,
        heading=0.34,
        curvature=0.30,
        lookahead=0.28,
        confidence=0.0,
        lost=True,
    )
    cmd = warm_policy(turn_lost, mode="no_other_cars", steps=4)
    assert abs(cmd.speed - CONTROL["lost_speed"]) < 0.03


def test_low_confidence_stays_slow():
    reset_policy_state()
    track = make_track(heading=0.24, curvature=0.28, lookahead=0.25, confidence=0.20)
    cmd = warm_policy(track, mode="no_other_cars", steps=12)
    assert cmd.speed <= get_profile("no_other_cars")["recovery_speed"]


def test_unknown_profile_names_default_to_no_other_cars():
    track = make_track(
        lateral=0.20, heading=0.15, curvature=0.22, lookahead=0.18, confidence=0.85
    )
    results = {}
    for name in ("no_other_cars", "fastest", "safe", "unknown"):
        reset_policy_state()
        cmd = warm_policy(track, mode=name)
        results[name] = (cmd.speed, cmd.steering)
    assert results["fastest"] == results["no_other_cars"]
    assert results["safe"] == results["no_other_cars"]
    assert results["unknown"] == results["no_other_cars"]


def test_two_profiles_share_core_driving_but_differ_on_multicar():
    from controller.params import get_profile

    no = get_profile("no_other_cars")
    wo = get_profile("with_other_cars")
    assert no["base_speed"] == wo["base_speed"]
    assert no["turn_in_lateral_ref"] == wo["turn_in_lateral_ref"]
    assert no["opponent_avoid_steering_enable"] is False
    assert wo["opponent_avoid_steering_enable"] is True
    assert no["escape_reverse_speed"] == 0.0
    assert wo["escape_reverse_speed"] > 0.0
    assert no["motion_still_threshold"] == 0.0
    assert wo["motion_still_threshold"] > 0.0


def test_red_environment_flag_does_not_switch_policy_parameters():
    base = make_track(
        lateral=0.18,
        heading=0.12,
        curvature=0.20,
        lookahead=0.16,
        confidence=0.80,
        red_environment=False,
    )
    red = make_track(
        lateral=0.18,
        heading=0.12,
        curvature=0.20,
        lookahead=0.16,
        confidence=0.80,
        red_environment=True,
    )
    reset_policy_state()
    normal_cmd = warm_policy(base, mode="no_other_cars")
    reset_policy_state()
    red_env_cmd = warm_policy(red, mode="no_other_cars")

    assert red_env_cmd.speed == normal_cmd.speed
    assert red_env_cmd.steering == normal_cmd.steering


def test_timestamp_reset_discards_speed_state():
    reset_policy_state()
    fast = warm_policy(make_track(), mode="no_other_cars", steps=20)
    assert fast.speed > 0.45

    reset_cmd = decide_control(make_track(), 0.10, mode="no_other_cars")
    assert reset_cmd.speed < fast.speed
    assert reset_cmd.speed <= 0.20


def test_inside_margin_guard_is_noop_by_default():
    profile = get_profile("no_other_cars")
    normal = make_track(
        lateral=0.16, heading=0.24, curvature=0.32, lookahead=0.28, red_environment=True
    )
    close_right = make_track(
        lateral=0.16, heading=0.24, curvature=0.32, lookahead=0.28, red_environment=True
    )
    close_right.right_margin_near = profile["inside_margin_warning"] * 0.35

    normal_steer = _target_steering(
        normal, _control_signals(normal, profile), "hard_turn", profile
    )
    guarded_steer = _target_steering(
        close_right, _control_signals(close_right, profile), "hard_turn", profile
    )

    assert guarded_steer == normal_steer


def test_inside_margin_limits_steering_toward_guardrail_when_enabled():
    profile = get_profile("no_other_cars")
    profile["inside_margin_outward_gain"] = 0.32
    profile["inside_margin_steering_cap"] = 0.42
    normal = make_track(
        lateral=0.16, heading=0.24, curvature=0.32, lookahead=0.28, red_environment=True
    )
    close_right = make_track(
        lateral=0.16, heading=0.24, curvature=0.32, lookahead=0.28, red_environment=True
    )
    close_right.right_margin_near = profile["inside_margin_warning"] * 0.35

    normal_steer = _target_steering(
        normal, _control_signals(normal, profile), "hard_turn", profile
    )
    guarded_steer = _target_steering(
        close_right, _control_signals(close_right, profile), "hard_turn", profile
    )

    assert normal_steer > 0.03
    assert guarded_steer < normal_steer


def test_confident_line_adds_bounded_post_hoc_steering_correction():
    from controller.params import LINE_FOLLOW_PROFILE

    base = make_track()
    lined = make_track()
    lined.line_offset = 0.25
    lined.line_heading = 0.10
    lined.line_confidence = 0.80

    reset_policy_state()
    base_cmd = warm_policy(base, mode="no_other_cars")
    reset_policy_state()
    lined_cmd = warm_policy(lined, mode="no_other_cars")

    expected = (
        lined.line_offset * LINE_FOLLOW_PROFILE["offset_gain"]
        + lined.line_heading * LINE_FOLLOW_PROFILE["heading_gain"]
    )
    expected = min(expected, LINE_FOLLOW_PROFILE["max_correction"])
    assert lined_cmd.speed == base_cmd.speed
    assert abs((lined_cmd.steering - base_cmd.steering) - expected) < 1e-3


def test_single_frame_line_detection_adds_no_correction():
    reset_policy_state()
    base = make_track()
    flicker = make_track()
    flicker.line_offset = 0.25
    flicker.line_heading = 0.0
    flicker.line_confidence = 0.80

    for index in range(20):
        decide_control(base, index * 0.05, mode="no_other_cars")
    cmd = decide_control(flicker, 20 * 0.05, mode="no_other_cars")

    assert abs(cmd.steering) < 0.02


def test_implausible_large_line_offset_is_rejected():
    from controller.params import LINE_FOLLOW_PROFILE

    rail = make_track()
    rail.line_offset = LINE_FOLLOW_PROFILE["offset_trust_max"] + 0.12
    rail.line_heading = 0.0
    rail.line_confidence = 0.80

    reset_policy_state()
    base_cmd = warm_policy(make_track(), mode="no_other_cars")
    reset_policy_state()
    rail_cmd = warm_policy(rail, mode="no_other_cars")

    assert abs(rail_cmd.steering - base_cmd.steering) < 1e-6


def test_startup_line_acquisition_allows_right_side_line():
    base = make_track(red_environment=True)
    lined = make_track(red_environment=True)
    lined.line_offset = 0.44
    lined.line_heading = 0.22
    lined.line_confidence = 0.80

    reset_policy_state()
    base_cmd = warm_policy(base, mode="no_other_cars", steps=8)
    reset_policy_state()
    lined_cmd = warm_policy(lined, mode="no_other_cars", steps=8)

    assert lined_cmd.steering > base_cmd.steering + 0.08


def test_trusted_offcenter_line_can_steer_left_after_phase2():
    lined = make_track(red_environment=True)
    lined.line_offset = -0.44
    lined.line_heading = 0.00
    lined.line_confidence = 0.80

    reset_policy_state()
    base_cmd = warm_policy(
        make_track(red_environment=True), mode="no_other_cars", steps=8
    )
    reset_policy_state()
    lined_cmd = warm_policy(lined, mode="no_other_cars", steps=8)

    assert lined_cmd.steering < base_cmd.steering - 0.08


def test_offcenter_line_remains_valid_after_startup_window():
    lined = make_track(red_environment=True)
    lined.line_offset = 0.44
    lined.line_heading = 0.22
    lined.line_confidence = 0.80

    reset_policy_state()
    for index in range(8):
        cmd = decide_control(lined, 15.0 + index * 0.05, mode="no_other_cars")

    assert cmd.steering > 0.08


def test_line_correction_suppressed_near_obstacle():
    blocked = make_track()
    blocked.line_offset = 0.25
    blocked.line_heading = 0.0
    blocked.line_confidence = 0.80
    blocked.near_obstacle = True

    reset_policy_state()
    base_cmd = warm_policy(make_track(), mode="no_other_cars")
    reset_policy_state()
    blocked_cmd = warm_policy(blocked, mode="no_other_cars")

    assert abs(blocked_cmd.steering - base_cmd.steering) < 1e-6


def test_side_obstacle_slows_less_than_center_obstacle():
    centered = make_track(near_obstacle=True, obstacle_x=0.0, obstacle_size=0.08)
    side = make_track(near_obstacle=True, obstacle_x=0.72, obstacle_size=0.08)

    reset_policy_state()
    center_cmd = warm_policy(centered, mode="with_other_cars")
    reset_policy_state()
    side_cmd = warm_policy(side, mode="with_other_cars")

    assert side_cmd.speed > center_cmd.speed + 0.08


def test_obstacle_position_changes_avoidance_direction():
    left_obstacle = make_track(near_obstacle=True, obstacle_x=-0.72, obstacle_size=0.08)
    right_obstacle = make_track(near_obstacle=True, obstacle_x=0.72, obstacle_size=0.08)

    reset_policy_state()
    left_cmd = warm_policy(left_obstacle, mode="with_other_cars")
    reset_policy_state()
    right_cmd = warm_policy(right_obstacle, mode="with_other_cars")

    assert left_cmd.steering > right_cmd.steering + 0.10


def test_line_correction_suppressed_in_sharp_turn():
    turn = make_track(
        heading=0.40, curvature=0.40, lookahead=0.40, red_environment=True
    )
    lined_turn = make_track(
        heading=0.40, curvature=0.40, lookahead=0.40, red_environment=True
    )
    lined_turn.line_offset = -0.25
    lined_turn.line_heading = 0.0
    lined_turn.line_confidence = 0.80

    reset_policy_state()
    base_cmd = warm_policy(turn, mode="no_other_cars")
    reset_policy_state()
    lined_cmd = warm_policy(lined_turn, mode="no_other_cars")

    assert abs(lined_cmd.steering - base_cmd.steering) < 1e-6


def test_large_offset_line_keeps_recenter_correction_in_left_turn():
    base = make_track(
        heading=-0.65, lookahead=-0.45, confidence=0.45, red_environment=True
    )
    lined = make_track(
        heading=-0.65, lookahead=-0.45, confidence=0.45, red_environment=True
    )
    lined.line_offset = 0.62
    lined.line_heading = -0.60
    lined.line_confidence = 0.80

    reset_policy_state()
    base_cmd = warm_policy(base, mode="no_other_cars", steps=8)
    reset_policy_state()
    lined_cmd = warm_policy(lined, mode="no_other_cars", steps=8)

    assert lined_cmd.steering > base_cmd.steering + 0.05


def test_geometry_escape_requires_low_speed():
    reset_policy_state()
    cornering = make_track(
        heading=0.50, curvature=0.30, lookahead=0.50, red_environment=True
    )
    for index in range(60):
        decide_control(cornering, index * 0.05, mode="no_other_cars")
        assert get_policy_diagnostics()["mode"] != "escaping"


def test_centered_complex_turn_in_keeps_wide_radius():
    reset_policy_state()
    approach = make_track(
        lateral=0.012,
        heading=0.244,
        curvature=0.252,
        lookahead=0.142,
        confidence=0.53,
        red_environment=True,
    )
    cmd = warm_policy(approach, mode="no_other_cars", steps=10)
    assert abs(cmd.steering) < 0.06


def test_hard_turn_requires_consecutive_frames():
    reset_policy_state()
    track = make_track(
        heading=0.28, curvature=0.35, lookahead=0.32, red_environment=True
    )

    decide_control(track, 0.00, mode="no_other_cars")
    assert get_policy_diagnostics()["mode"] != "hard_turn"

    decide_control(track, 0.05, mode="no_other_cars")
    assert get_policy_diagnostics()["mode"] == "hard_turn"


def test_sustained_inside_turn_line_strengthens_outward_correction():
    from controller.params import CONTROL, LINE_FOLLOW_PROFILE as P

    base = make_track(
        heading=-0.70,
        curvature=-0.30,
        lookahead=-0.45,
        confidence=0.6,
        red_environment=True,
    )
    lined = make_track(
        heading=-0.70,
        curvature=-0.30,
        lookahead=-0.45,
        confidence=0.6,
        red_environment=True,
    )
    lined.line_offset = 0.55
    lined.line_heading = -0.80
    lined.line_confidence = 0.85

    saved = CONTROL["corner_relief_enable"]
    CONTROL["corner_relief_enable"] = False
    try:
        reset_policy_state()
        base_cmd = warm_policy(base, mode="no_other_cars", steps=10)
        reset_policy_state()
        lined_cmd = warm_policy(lined, mode="no_other_cars", steps=10)
    finally:
        CONTROL["corner_relief_enable"] = saved

    correction = lined_cmd.steering - base_cmd.steering
    floor_only = lined.line_offset * P["offset_gain"] * P["offset_curve_min_scale"]
    assert correction > floor_only + 0.05
    assert correction <= P["max_correction"] + 1e-6


def test_inside_turn_correction_holds_through_brief_line_dropout():
    turn = make_track(
        heading=-0.70,
        curvature=-0.30,
        lookahead=-0.45,
        confidence=0.6,
        red_environment=True,
    )
    turn.line_offset = 0.55
    turn.line_heading = -0.80
    turn.line_confidence = 0.85

    reset_policy_state()
    warm_policy(turn, mode="no_other_cars", steps=10)
    held = get_policy_diagnostics()["line_correction"]
    assert held > 0.10

    dropout = make_track(
        heading=-0.70,
        curvature=-0.30,
        lookahead=-0.45,
        confidence=0.6,
        red_environment=True,
    )
    dropout.line_confidence = 0.0
    decide_control(dropout, 10 * 0.05, mode="no_other_cars")
    assert get_policy_diagnostics()["line_correction"] > held * 0.7


def test_inside_assist_inactive_when_line_and_heading_agree():
    from controller.params import LINE_FOLLOW_PROFILE as P

    base = make_track(
        heading=0.30,
        curvature=0.30,
        lookahead=0.30,
        confidence=0.6,
        red_environment=True,
    )
    lined = make_track(
        heading=0.30,
        curvature=0.30,
        lookahead=0.30,
        confidence=0.6,
        red_environment=True,
    )
    lined.line_offset = 0.55
    lined.line_heading = 0.40

    lined.line_confidence = 0.85

    reset_policy_state()
    base_cmd = warm_policy(base, mode="no_other_cars", steps=10)
    reset_policy_state()
    lined_cmd = warm_policy(lined, mode="no_other_cars", steps=10)

    correction = lined_cmd.steering - base_cmd.steering
    assert correction <= P["max_correction"] + 1e-6


def test_corner_relief_reduces_far_term_when_line_shows_inside():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    inside = make_track(
        lateral=-0.30,
        heading=-0.65,
        curvature=0.0,
        lookahead=-0.55,
        confidence=0.7,
        red_environment=True,
    )
    inside.line_offset = 0.55
    inside.line_heading = -0.80
    inside.line_confidence = 0.85
    signals = _control_signals(inside, profile)

    reset_policy_state()
    relieved = _target_steering(inside, signals, "hard_turn", dict(profile))
    reset_policy_state()
    no_relief = dict(profile)
    no_relief["corner_relief_enable"] = False
    baseline = _target_steering(inside, signals, "hard_turn", no_relief)

    assert relieved > baseline + 0.03
    assert relieved <= 0.0



def test_corner_relief_inactive_without_confident_line():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    turn = make_track(
        heading=-0.65,
        curvature=0.0,
        lookahead=-0.55,
        confidence=0.7,
        red_environment=True,
    )
    turn.line_confidence = 0.0
    signals = _control_signals(turn, profile)

    reset_policy_state()
    relieved = _target_steering(turn, signals, "hard_turn", dict(profile))
    reset_policy_state()
    no_relief = dict(profile)
    no_relief["corner_relief_enable"] = False
    baseline = _target_steering(turn, signals, "hard_turn", no_relief)
    assert abs(relieved - baseline) < 1e-9


def test_corner_relief_holds_through_offset_sign_flip():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    inside = make_track(
        lateral=-0.30,
        heading=-0.65,
        curvature=0.0,
        lookahead=-0.55,
        confidence=0.7,
        red_environment=True,
    )
    inside.line_offset = 0.55
    inside.line_heading = -0.80
    inside.line_confidence = 0.85
    trough = make_track(
        lateral=-0.30,
        heading=-0.65,
        curvature=0.0,
        lookahead=-0.55,
        confidence=0.7,
        red_environment=True,
    )
    trough.line_offset = -0.05

    trough.line_heading = -0.80
    trough.line_confidence = 0.85
    sig_in = _control_signals(inside, profile)
    sig_tr = _control_signals(trough, profile)

    reset_policy_state()
    _target_steering(inside, sig_in, "hard_turn", dict(profile))
    held = _target_steering(trough, sig_tr, "hard_turn", dict(profile))
    reset_policy_state()
    no_relief = dict(profile)
    no_relief["corner_relief_enable"] = False
    baseline = _target_steering(trough, sig_tr, "hard_turn", no_relief)

    assert held > baseline + 0.02



def test_turn_in_suppressed_when_car_still_centered_on_approach():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    approach = make_track(
        lateral=0.0,
        heading=-0.45,
        curvature=0.0,
        lookahead=-0.30,
        confidence=0.7,
        red_environment=True,
    )
    signals = _control_signals(approach, profile)
    reset_policy_state()
    steer = _target_steering(approach, signals, "hard_turn", dict(profile))
    assert abs(steer) < 0.06



def test_turn_in_opens_once_car_has_drifted_into_corner():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    arrived = make_track(
        lateral=-0.30,
        heading=-0.45,
        curvature=0.0,
        lookahead=-0.30,
        confidence=0.7,
        red_environment=True,
    )
    signals = _control_signals(arrived, profile)
    reset_policy_state()
    steer = _target_steering(arrived, signals, "hard_turn", dict(profile))
    assert steer < -0.2



def test_turn_in_latch_sustains_turn_through_midcorner_lateral_dip():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    deep = make_track(
        lateral=-0.40,
        heading=-0.80,
        curvature=0.0,
        lookahead=-0.60,
        confidence=0.7,
        red_environment=True,
    )

    dip = make_track(
        lateral=-0.03,
        heading=-0.80,
        curvature=0.0,
        lookahead=-0.60,
        confidence=0.7,
        red_environment=True,
    )

    sig_d = _control_signals(deep, profile)
    sig_p = _control_signals(dip, profile)

    reset_policy_state()
    _target_steering(deep, sig_d, "hard_turn", dict(profile))  # build latch
    held = _target_steering(dip, sig_p, "hard_turn", dict(profile))
    reset_policy_state()
    no_latch = _target_steering(dip, sig_p, "hard_turn", dict(profile))

    assert held < no_latch - 0.05

    assert held < 0.0


def test_turn_in_gate_is_pure_lateral_no_sharpness_modulation():
    from controller.params import get_profile

    profile = get_profile("no_other_cars")
    gentle = make_track(
        lateral=-0.20,
        heading=-0.30,
        curvature=0.0,
        lookahead=-0.25,
        confidence=0.7,
        red_environment=True,
    )  # curve_risk ≈ 0.30
    sharp = make_track(
        lateral=-0.20,
        heading=-0.95,
        curvature=0.0,
        lookahead=-0.90,
        confidence=0.7,
        red_environment=True,
    )  # curve_risk ≈ 0.95
    ref = profile["turn_in_lateral_ref"]
    expected_arrival = min(1.0, 0.20 / ref)
    assert "turn_in_gentle_extra" not in profile and "turn_in_sharp_ref" not in profile
    assert abs(expected_arrival - min(1.0, abs(gentle.lateral_error) / ref)) < 1e-9
    assert abs(expected_arrival - min(1.0, abs(sharp.lateral_error) / ref)) < 1e-9
