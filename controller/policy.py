"""Stateful steering, speed planning, and recovery policy."""

import math
from dataclasses import dataclass

from controller.common import ControlCmd, TrackState, clamp
from controller.params import LINE_FOLLOW_PROFILE, get_profile


@dataclass
class PolicyState:
    """All cross-frame policy memory owned by one controller session."""

    last_steering: float = 0.0
    last_speed: float = 0.0
    last_timestamp: float | None = None
    lost_frames: int = 0
    recovery_frames: int = 0
    last_good_bias: float = 0.0
    last_mode: str = "start"
    stall_frames: int = 0
    escape_frames: int = 0
    escape_steering_sign: float = 1.0
    escape_steering_magnitude: float = 0.0
    escape_speed: float = 0.0
    escape_total_frames: int = 0
    escape_reverse_frames: int = 0
    last_track_signature: tuple[float, ...] | None = None
    straight_memory_frames: int = 0
    hard_turn_candidate_frames: int = 0
    recovery_candidate_frames: int = 0
    last_mode_reason: str = "start"
    last_target_steering: float = 0.0
    last_target_speed: float = 0.0
    last_signals: dict | None = None
    last_straight_memory_active: bool = False
    line_streak: int = 0
    line_last_offset: float = 0.0
    line_correction: float = 0.0
    line_hold_frames: int = 0
    corner_relief: float = 0.0
    turn_in_latch: float = 0.0
    lost_streak: int = 0
    not_stuck_frames: int = 0
    force_escape_active: bool = False
    force_escape_frames: int = 0
    force_escape_speed: float = 0.0
    force_escape_steering: float = 0.0
    force_escape_sign: float = 1.0
    force_escape_total_frames: int = 0
    force_escape_reverse_frames: int = 0
    zero_speed_streak: int = 0
    motion_stall_streak: int = 0

    def __post_init__(self) -> None:
        if self.last_signals is None:
            self.last_signals = {}


_STATE = PolicyState()


def reset_policy_state() -> None:
    """Replace the session state with a clean instance."""

    global _STATE
    _STATE = PolicyState()


def get_policy_diagnostics() -> dict:
    """Return a read-only snapshot for tests, telemetry, and debugging."""

    return {
        "mode": _STATE.last_mode,
        "mode_reason": _STATE.last_mode_reason,
        "line_correction": _STATE.line_correction,
        "target_steering": _STATE.last_target_steering,
        "target_speed": _STATE.last_target_speed,
        "signals": dict(_STATE.last_signals or {}),
        "force_escape_active": _STATE.force_escape_active,
    }


def _maybe_reset_policy_by_timestamp(timestamp: float, profile: dict) -> None:
    """根据时间戳判断是否重置策略状态。"""

    if _STATE.last_timestamp is None:
        return
    elapsed = float(timestamp) - float(_STATE.last_timestamp)
    if elapsed < 0.0 or elapsed > profile["timestamp_reset_gap"]:
        reset_policy_state()


def _dt(timestamp: float, profile: dict) -> float:
    """计算本帧控制间隔，首帧使用 nominal_dt。"""

    if _STATE.last_timestamp is None:
        return profile["nominal_dt"]
    return max(float(timestamp) - float(_STATE.last_timestamp), profile["nominal_dt"])


def _signed_power(value: float, power: float) -> float:
    """保留符号的幂函数。"""

    return math.copysign(abs(value) ** power, value)


def _road_direction_sign(track: TrackState) -> float:
    """估计应朝哪一侧打轮才能回到路面（左负右正）。

    功能：为脱困提供"路在哪边"的方向，而不是写死方向或盲目反打上一帧。
    参数：`track` 是当前赛道状态。
    返回：`+1.0`（向右回到路面）或 `-1.0`（向左回到路面）。
    逻辑：优先用当前横向误差；贴墙丢线时退回最近一次可信偏置；都不可用时反打上一帧舵角脱离。
    """

    reference = track.lateral_error
    if abs(reference) <= 0.05:
        reference = _STATE.last_good_bias
    if abs(reference) <= 1e-3:
        reference = -_STATE.last_steering
    if abs(reference) <= 1e-6:
        return 1.0
    return math.copysign(1.0, reference)


def _margin_escape_sign(track: TrackState, fallback: float) -> float:
    """根据单侧边界余量选择脱困方向。

    功能：在近障碍或贴栏卡死时，优先往余量更大的方向打轮。
    参数：`track` 提供左右近处余量，`fallback` 是几何偏置推断出的方向。
    返回：`+1.0` 表示右打，`-1.0` 表示左打。
    逻辑：只有左右余量差足够明显时才覆盖几何方向，避免噪声边界点造成反复换向。
    """

    margin_gap = abs(track.left_margin_near - track.right_margin_near)
    if margin_gap <= 0.08:
        return fallback
    if track.left_margin_near < track.right_margin_near:
        return 1.0
    return -1.0


def _contact_escape_sign(track: TrackState, fallback: float, profile: dict) -> float:
    """在推断贴边时选择远离栏杆的脱困方向。

    功能：把碰撞/贴边推断和常规循线分开，只有单侧余量极小时才按边界反向。
    参数：`track` 提供左右近处余量，`fallback` 是道路几何推断方向，`profile` 是控制参数。
    返回：`+1.0` 表示右打，`-1.0` 表示左打。
    逻辑：余量不够明确时保留几何方向；左侧近乎贴住就右打，右侧近乎贴住就左打。
    """

    min_margin = min(track.left_margin_near, track.right_margin_near)
    margin_gap = abs(track.left_margin_near - track.right_margin_near)
    if min_margin > profile["escape_boundary_margin_max"] or margin_gap <= 0.08:
        return fallback
    return _margin_escape_sign(track, fallback)


def _control_signals(track: TrackState, profile: dict) -> dict:
    """计算策略使用的风险分量。

    功能：拆分弯道、偏移、置信度和丢线风险。
    参数：`track` 是赛道状态，`profile` 是控制参数。
    返回：包含各类风险和综合风险的字典。
    逻辑：速度和模式选择共享这些风险，避免重复估算。
    """

    curve_risk = clamp(
        max(abs(track.curvature), abs(track.heading_error), abs(track.lookahead_error)),
        0.0,
        1.0,
    )
    offset_risk = clamp(abs(track.lateral_error), 0.0, 1.0)
    confidence_risk = clamp(1.0 - track.confidence, 0.0, 1.0)
    lost_risk = 1.0 if track.lost else 0.0
    min_margin = min(
        clamp(track.left_margin_near, 0.0, 1.0),
        clamp(track.right_margin_near, 0.0, 1.0),
    )
    margin_risk = clamp(
        (profile["inside_margin_warning"] - min_margin)
        / profile["inside_margin_warning"],
        0.0,
        1.0,
    )
    turn_demand = clamp(curve_risk * 0.55 + offset_risk * 0.45, 0.0, 1.0)
    risk = clamp(
        curve_risk * profile["risk_curve_weight"]
        + offset_risk * profile["risk_offset_weight"]
        + confidence_risk * profile["risk_confidence_weight"]
        + lost_risk * profile["risk_lost_weight"],
        0.0,
        1.0,
    )
    return {
        "curve_risk": curve_risk,
        "offset_risk": offset_risk,
        "confidence_risk": confidence_risk,
        "lost_risk": lost_risk,
        "margin_risk": margin_risk,
        "turn_demand": turn_demand,
        "risk": risk,
    }


def _is_straight_candidate(track: TrackState, signals: dict, profile: dict) -> bool:
    """判断当前几何是否足够像直道。

    功能：给直道提速和 lost 惯性滑行提供稳定判据。
    参数：`track` 是赛道状态，`signals` 是风险分量，`profile` 是控制参数。
    返回：当前非丢线帧是否可以视为直道。
    逻辑：主要看曲率、前瞻和横向偏移；heading 只做宽松兜底，避免噪声把直道判坏。
    """

    if track.lost or track.confidence < profile["lost_confidence"]:
        return False
    stable_curve = (
        max(abs(track.curvature), abs(track.lookahead_error))
        <= profile["straight_curve_max"]
    )
    centered = signals["offset_risk"] <= profile["straight_offset_max"]
    heading_ok = abs(track.heading_error) <= profile["straight_heading_max"]
    return stable_curve and centered and heading_ok


def _is_lost_straight_coast_candidate(
    track: TrackState, signals: dict, profile: dict
) -> bool:
    """判断丢线帧是否仍可按直道惯性滑行。

    功能：处理蓝门/天空造成的无观测帧，避免直道速度掉回 0.24。
    参数：`track` 是已衰减的丢线状态，`signals` 是风险分量，`profile` 是控制参数。
    返回：当前 lost 帧是否可安全维持直道速度。
    逻辑：只接受居中、曲率/前瞻低、heading 不大且上一帧舵角很小的 lost 帧。
    """

    if not track.lost:
        return False
    stable_curve = (
        max(abs(track.curvature), abs(track.lookahead_error))
        <= profile["straight_curve_max"]
    )
    centered = signals["offset_risk"] <= profile["straight_offset_max"]
    heading_ok = abs(track.heading_error) <= profile["straight_heading_max"]
    steering_ok = abs(_STATE.last_steering) <= profile["straight_lost_steering_max"]
    return stable_curve and centered and heading_ok and steering_ok


def _update_straight_memory(
    track: TrackState, signals: dict, mode: str, profile: dict
) -> bool:
    """更新直道记忆并返回本帧是否允许直道滑行。

    功能：让蓝门/天空造成的短暂 lost 不再立刻砸到 `lost_speed`。
    参数：当前赛道状态、风险信号、驾驶状态和参数表。
    返回：是否仍处在最近确认过的直道窗口内。
    逻辑：非 lost 直道帧刷新记忆；lost 帧可消耗记忆，或在几何和上一帧舵角都很直时直接滑行。
    """

    if _is_straight_candidate(track, signals, profile):
        _STATE.straight_memory_frames = int(profile["straight_memory_frames"])
        return True
    if _STATE.straight_memory_frames > 0 and mode == "lost":
        _STATE.straight_memory_frames -= 1
        return True
    if _is_lost_straight_coast_candidate(track, signals, profile):
        return True
    _STATE.straight_memory_frames = 0
    return False


def _select_mode(
    track: TrackState, signals: dict, timestamp: float, profile: dict
) -> str:
    """选择内部驾驶状态。

    功能：在 lost、recovering、hard_turn、correcting、cruise 之间切换。
    参数：`track` 是赛道状态，`signals` 是风险分量，`timestamp` 是平台时间。
    返回：内部状态名。
    逻辑：丢线优先，其次恢复缓冲、急弯、回中和巡航。
    """

    del timestamp
    if track.lost or track.confidence < profile["lost_confidence"]:
        _STATE.hard_turn_candidate_frames = 0
        _STATE.recovery_candidate_frames = 0
        _STATE.last_mode_reason = "lost_or_low_confidence"
        return "lost"
    if _STATE.recovery_frames > 0 or track.confidence < profile["recovery_confidence"]:
        _STATE.recovery_candidate_frames += 1
        _STATE.hard_turn_candidate_frames = 0
        if _STATE.recovery_frames > 0 or _STATE.recovery_candidate_frames >= int(
            profile["recovery_enter_frames"]
        ):
            _STATE.last_mode_reason = "recovery_buffer_or_confidence"
            return "recovering"
    else:
        _STATE.recovery_candidate_frames = 0

    hard_turn_candidate = (
        signals["curve_risk"] > profile["hard_turn_threshold"]
        or signals["turn_demand"] > profile["hard_turn_threshold"]
    )
    hard_turn_hold = _STATE.last_mode == "hard_turn" and (
        signals["curve_risk"] > profile["hard_turn_exit_threshold"]
        or signals["turn_demand"] > profile["hard_turn_exit_threshold"]
    )
    if hard_turn_candidate or hard_turn_hold:
        _STATE.hard_turn_candidate_frames += 1
    else:
        _STATE.hard_turn_candidate_frames = 0
    if hard_turn_hold or _STATE.hard_turn_candidate_frames >= int(
        profile["hard_turn_enter_frames"]
    ):
        _STATE.last_mode_reason = "curve_or_turn_demand"
        return "hard_turn"
    if abs(track.lateral_error) > profile["correction_error"]:
        _STATE.last_mode_reason = "lateral_error"
        return "correcting"
    _STATE.last_mode_reason = "nominal"
    return "cruise"


def _apply_inside_margin_guard(raw: float, track: TrackState, profile: dict) -> float:
    """根据近处边界余量限制继续向内打轮。

    功能：弯中贴近内侧护栏时，把目标舵角往外侧推。
    参数：`raw` 是未限幅目标舵角，`track` 提供左右边界余量。
    返回：保护后的目标舵角。
    逻辑：只在舵角方向指向低余量一侧时生效，避免直道无端左右摆。
    """

    warning = max(float(profile["inside_margin_warning"]), 1e-6)
    if raw > 0.0:
        pressure = clamp((warning - track.right_margin_near) / warning, 0.0, 1.0)
        if pressure <= 0.0:
            return raw
        capped = min(raw, profile["inside_margin_steering_cap"])
        return capped - pressure * profile["inside_margin_outward_gain"]
    if raw < 0.0:
        pressure = clamp((warning - track.left_margin_near) / warning, 0.0, 1.0)
        if pressure <= 0.0:
            return raw
        capped = max(raw, -profile["inside_margin_steering_cap"])
        return capped + pressure * profile["inside_margin_outward_gain"]
    return raw


def _target_steering(
    track: TrackState, signals: dict, mode: str, profile: dict
) -> float:
    """计算目标转向。

    功能：把回中项和前瞻项分开组合，并按状态修正。
    参数：`track` 是赛道状态，`signals` 是风险分量，`mode` 是内部驾驶状态。
    返回：裁剪到 `[-1, 1]` 的目标转向。
    逻辑：偏移大时更看近处，弯道明显时更看前方；两者冲突时优先回中。
    """

    center_term = track.lateral_error * profile["gain_lateral"]
    lookahead_term = (
        track.lookahead_error * profile["gain_lookahead"]
        + track.heading_error * profile["gain_heading"]
        + track.curvature * profile["gain_curve"]
    )
    # 入弯时机门控（R042 重做）：远处 road-mask 预瞄项是"切内线"的来源——它在直道接近段就因
    # 远处的路已弯而变大，把车提前打进弯里。旧门控用 |heading| 当"弯到了没"的判据，但 heading
    # 正是远处弯量、在接近段就涨起来，于是门提前全开、毫无迟滞（实测 t224 撞栏弯：heading=-0.39
    # 时门已 0.94，而车还居中、还骑在白线上）。
    # 正确判据是"车是否已经物理到达弯"——只看近处 lateral 漂移：直道接近段它≈0，只有当车跟着
    # 直线开到弯口、真正开始偏离时才长起来。R043 删除 floor 后直接用 arrival 缩放远处预瞄项：
    # 车未到弯口时可把预瞄项压到 0，沿线开进弯口、略外移（out-in-out），再触发"晚而狠"的转向。
    # 不用 |line_offset| 当 arrival——它分不清"外移到弯口"和"已经切到内侧"，后者会把门开更大、越切越深。
    # R046：删除 R044 的"弯有多急(curve_risk)"调制。它在入弯初期把弯误判成缓弯——接近段 curve_risk
    # 还低（远处弯量没在视野里发育起来），sharpness 小 → arrival_ref 被放大 1.5x+ → 过度迟滞；等
    # curve_risk 涨上来时车已深入弯里，才急打轮、半径反而很大、还冲到外侧、速度也掉。根因是
    # "入弯瞬间没有信号能区分缓/急弯"，所以这种基于瞬时 curve_risk 的调制原理上就修不好，直接删。
    # 回到纯近处 lateral 漂移驱动的门控。
    # R047 速度耦合：过弯越快、半径越大——高速进弯时车在"门还没开"的入口段就冲出很远（实测
    # line_offset 入口冲到 −0.59）。把入弯参考随速度收小（faster → ref 小 → arrival 早开 → 早转），
    # 让"入弯提前量"随速度成比例，弥补高速多走的距离。speed_norm=0 时退化为纯 lateral 门控。
    speed_norm = clamp(_STATE.last_speed / max(profile["max_speed"], 1e-6), 0.0, 1.0)
    arrival_ref = max(
        profile["turn_in_lateral_ref"]
        * (1.0 - profile["turn_in_speed_comp"] * speed_norm),
        1e-3,
    )
    instant_arrival = clamp(abs(track.lateral_error) / arrival_ref, 0.0, 1.0)
    # R048 弯中保持 + 出弯迟滞：入弯门控是 lookahead_term 上的连续乘子，但 `lateral`（road-mask 近处
    # 漂移）在弯中会反复回落到≈0（mask 重新对正路面），门控就把远处预瞄项收掉 → 车转一半忽然收轮、
    # 转不到位、半径变大、还得事后找回中线（费速度）；出弯时远处看到直路、门控也跟着提前收。
    # 用 latch 保持最近的 arrival 峰值并按 hold_decay 缓慢衰减：弯中 lateral 短暂回落不收门（持续转），
    # 出弯远处项自然回落时再迟滞收轮（用户要的"出弯 lag"）。直道上远处项≈0，latch 高也不会乱打。
    if mode == "hard_turn":
        # 已进入弯（committed）：把门 ratchet 到最近峰值并保持，不让它在弯中随 lateral 回落而泄掉
        # （否则长 hairpin 里门一直≈0.15、车整段欠转、半径大）。入弯延迟仍在——hard_turn 早段 lateral
        # 还没长起来时 instant_arrival 小，latch 从小值起步、随 lateral 长大才 ratchet 上去。
        _STATE.turn_in_latch = max(instant_arrival, _STATE.turn_in_latch)
    else:
        # 出弯/直道：按 hold_decay 迟滞收门（出弯 lag），同时仍跟随 instant（正常入弯延迟）。
        _STATE.turn_in_latch = max(
            instant_arrival, _STATE.turn_in_latch * profile["turn_in_hold_decay"]
        )
    corner_arrival = _STATE.turn_in_latch
    lookahead_term *= corner_arrival
    # R040（2026-06-12，接触日志直接定位 t≈228.6 撞内栏）：弯中最大的舵角来自远处 road-mask
    # 预瞄项（lookahead+heading 可达 -0.99），它把车拉到接近满左锁；可信白线却显示车已切到
    # 内侧（offset 与远处项反号）。事后 ±0.34 有界修正顶不动 -0.76 的预瞄，只会和它打成
    # 来回甩的极限环，最终过冲撞内栏。这里直接在源头按"白线证明车已多内侧 × 置信度"成比例
    # 削弱远处项——既放大半径又消除那个来回甩。只缩远处转向项，不碰 risk/mode/速度/入弯门控，
    # 保持 R013/R014 的"白线不污染主链路风险与速度"边界。**走线改动，需人上车终判。**
    instant_relief = 0.0
    if (
        profile.get("corner_relief_enable")
        and track.red_environment
        and mode == "hard_turn"
        and track.line_confidence >= profile["corner_relief_conf_min"]
        and abs(track.line_offset) >= profile["corner_relief_offset_min"]
        and track.line_offset * lookahead_term < 0.0
    ):
        instant_relief = clamp(
            (abs(track.line_offset) - profile["corner_relief_offset_min"])
            * profile["corner_relief_gain"],
            0.0,
            profile["corner_relief_max"],
        ) * clamp(track.line_confidence, 0.0, 1.0)
    # R041 保持/迟滞：relief 瞬时触发后按 hold_decay 衰减保持，使 line_offset 在弯中来回穿 0 时
    # 远处项仍被压住，不再被 road-mask 猛拉回 -0.76 → 打破撞内栏的极限环。离开 hard_turn 即清零。
    if (
        profile.get("corner_relief_enable")
        and track.red_environment
        and mode == "hard_turn"
    ):
        _STATE.corner_relief = max(
            instant_relief, _STATE.corner_relief * profile["corner_relief_hold_decay"]
        )
    else:
        _STATE.corner_relief = 0.0
    if _STATE.corner_relief > 0.0:
        lookahead_term *= 1.0 - _STATE.corner_relief
    near_weight = (
        profile["near_weight_base"]
        + signals["offset_risk"] * profile["near_weight_offset_boost"]
    )
    far_weight = (
        profile["far_weight_base"]
        + signals["curve_risk"] * profile["far_weight_curve_boost"]
    )
    if center_term * lookahead_term < 0.0:
        conflict_offset = max(
            0.0, signals["offset_risk"] - profile["far_conflict_offset_start"]
        )
        far_weight *= max(
            profile["far_conflict_min_scale"],
            1.0 - conflict_offset * profile["far_conflict_offset_scale"],
        )

    raw = near_weight * center_term + far_weight * lookahead_term
    raw += profile["gain_lateral_nonlinear"] * _signed_power(track.lateral_error, 1.7)
    raw += profile["gain_curve_nonlinear"] * _signed_power(track.curvature, 1.5)

    if mode == "lost":
        # 不再死保上一帧舵角（贴墙时那正是把车怼进墙的舵），向最近可信道路方向
        # 偏置靠拢并整体衰减，让车松开栏杆等感知恢复，真正的反向由脱困状态机负责。
        raw = 0.50 * _STATE.last_steering + 0.20 * _STATE.last_good_bias
    elif mode == "recovering":
        raw *= 0.70
    elif mode == "correcting":
        raw += track.lateral_error * 0.25
    elif mode == "hard_turn":
        raw *= profile["hard_turn_steering_scale"]

    if (
        track.red_environment
        and track.lateral_error > profile["inside_left_lateral_min"]
        and track.heading_error < profile["inside_left_heading_max"]
        and track.curvature < profile["inside_left_curvature_max"]
    ):
        raw = max(raw, profile["inside_left_steering_limit"])

    raw = _apply_inside_margin_guard(raw, track, profile)

    if abs(raw) < profile["steering_deadzone"]:
        raw = 0.0
    max_abs = profile["max_abs_steering"]
    return clamp(raw, -max_abs, max_abs)


def _steering_smoothing_for_mode(mode: str, profile: dict) -> float:
    """读取当前驾驶状态对应的转向平滑系数。"""

    if mode == "cruise":
        return profile["steering_smoothing_cruise"]
    if mode == "hard_turn":
        return profile["steering_smoothing_turn"]
    if mode == "correcting":
        return profile["steering_smoothing_correction"]
    return profile["steering_smoothing_recovery"]


def _smooth_steering(
    target: float, mode: str, timestamp: float, profile: dict
) -> float:
    """平滑转向并限制变化率。

    功能：按驾驶状态和速度限制方向盘跳变。
    参数：`target` 是目标转向，`mode` 是内部状态，`timestamp` 是平台时间。
    返回：平滑后的转向值。
    逻辑：高速时限制更强，恢复状态更平滑。
    """

    alpha = _steering_smoothing_for_mode(mode, profile)
    smoothed = _STATE.last_steering * alpha + target * (1.0 - alpha)
    dt_factor = max(1.0, _dt(timestamp, profile) / profile["nominal_dt"])
    speed_norm = clamp(_STATE.last_speed / max(profile["max_speed"], 1e-6), 0.0, 1.0)
    speed_factor = 1.0 - 0.35 * speed_norm
    max_delta = profile["max_steering_delta"] * dt_factor * speed_factor
    delta = clamp(smoothed - _STATE.last_steering, -max_delta, max_delta)
    # 速度相关的转向幅值上限：高速时收舵，避免同样舵角在高速下半径过小切内线。
    max_abs = profile["max_abs_steering"] * (
        1.0 - profile["steering_speed_cap_scale"] * speed_norm
    )
    return clamp(_STATE.last_steering + delta, -max_abs, max_abs)


def _target_speed(
    track: TrackState,
    signals: dict,
    mode: str,
    steering: float,
    timestamp: float,
    profile: dict,
    straight_memory_active: bool = False,
) -> float:
    """计算目标速度。

    功能：用乘法降速组合弯道、偏移、置信度和转向风险。
    参数：`track` 是赛道状态，`signals` 是风险分量，`mode` 是内部状态，`steering` 是当前转向；
        `straight_memory_active` 表示最近刚确认过直道。
    返回：目标速度比例。
    逻辑：模式只限制速度上限，正常速度由风险因子相乘得到。
    """

    if mode == "lost":
        if straight_memory_active:
            return profile["straight_lost_speed"]
        return profile["lost_speed"]

    curve_factor = 1.0 - profile["curve_slowdown"] * (
        signals["curve_risk"] ** profile["curve_power"]
    )
    offset_factor = 1.0 - profile["offset_slowdown"] * (
        signals["offset_risk"] ** profile["offset_power"]
    )
    confidence_factor = (
        profile["min_confidence_factor"]
        + (1.0 - profile["min_confidence_factor"]) * track.confidence
    )
    steering_factor = 1.0 - profile["steering_slowdown"] * (
        abs(steering) ** profile["steering_power"]
    )
    margin_factor = 1.0 - profile["inside_margin_slowdown"] * signals["margin_risk"]
    target = (
        profile["base_speed"]
        * curve_factor
        * offset_factor
        * confidence_factor
        * steering_factor
        * margin_factor
    )

    if mode == "recovering":
        target = min(target, profile["recovery_speed"])
    elif mode == "hard_turn":
        centered_bonus = (
            profile["hard_turn_center_speed_bonus"]
            * (1.0 - signals["offset_risk"])
            * track.confidence
        )
        target = min(target, profile["hard_turn_speed"] + centered_bonus)
    elif mode == "correcting":
        target = min(target, profile["correction_speed"])
    # 直道提速：几何明确为直时，速度不该被偏低的 mask 置信度或 recovering 限速压住。
    # 判断主要看 curvature/lookahead/offset，heading 只做宽松兜底，避免噪声挡住直道加速。
    straight = straight_memory_active or _is_straight_candidate(track, signals, profile)
    if straight:
        target = max(target, profile["straight_speed"])
    if timestamp < profile["start_caution_seconds"]:
        target = min(target, profile["start_speed"])
    # 多车：正前方挡车要让速，偏侧车更像并行/被超场景，只轻微让速，避免一被超过就彻底放弃。
    if profile.get("enable_opponent", True) and track.near_obstacle:
        center_x = max(float(profile.get("opponent_center_x", 0.28)), 1e-6)
        centered = clamp(1.0 - abs(float(track.obstacle_x)) / center_x, 0.0, 1.0)
        front_factor = float(profile.get("opponent_speed_factor", 0.72))
        side_factor = float(profile.get("opponent_side_speed_factor", front_factor))
        target *= side_factor + (front_factor - side_factor) * centered
        # 弯道+正前方对手车：更激进减速，防止多车弯道碰撞卡死；偏侧车不叠这么重。
        if centered >= 0.35 and signals["curve_risk"] >= profile.get(
            "opponent_corner_curve_threshold", 0.25
        ):
            target *= profile.get("opponent_corner_speed_factor", 0.55)
    return clamp(target, profile["min_speed"], profile["max_speed"])


def _smooth_speed(target: float, timestamp: float, profile: dict) -> float:
    """平滑速度并限制加减速。

    功能：让加速慢、减速快，减少速度跳变。
    参数：`target` 是目标速度，`timestamp` 是平台时间，`profile` 是控制参数。
    返回：平滑后的速度比例。
    逻辑：上升和下降使用不同变化率，入弯能更快降速。
    """

    dt = _dt(timestamp, profile)
    delta = target - _STATE.last_speed
    if delta >= 0.0:
        delta = min(delta, profile["max_speed_increase_per_sec"] * dt)
    else:
        delta = max(delta, -profile["max_speed_decrease_per_sec"] * dt)
    return clamp(
        _STATE.last_speed + delta,
        min(profile["min_speed"], target),
        profile["max_speed"],
    )


def _track_signature(track: TrackState) -> tuple[float, float, float, float, float]:
    """提取用于判断画面是否停滞的几何签名。"""

    return (
        track.lateral_error,
        track.heading_error,
        track.curvature,
        track.lookahead_error,
        track.confidence,
    )


def _escape_if_stalled(
    track: TrackState,
    signals: dict,
    steering: float,
    speed: float,
    mode: str,
    profile: dict,
    allow_geometry_escape: bool,
) -> tuple[float, float, str]:
    """在顶住边界时短暂脱困。

    功能：检测急弯大转向卡边，或长时间低速且几何签名几乎不变的状态。
    参数：当前赛道状态、风险信号、已平滑的转向和速度、内部模式、参数表，
        `allow_geometry_escape` 控制是否启用依赖可靠几何的急弯/大偏移脱困（仅 complex）。
    返回：可能被覆盖后的 `(steering, speed, mode)`。
    逻辑：脱困方向统一朝感知到的路面一侧（远离顶住的栏杆），不再写死方向或盲目反打；
        低速贴墙本就是低置信状态，所以低速脱困放宽置信度门槛，basic 也启用。
    """

    signature = _track_signature(track)
    if _STATE.last_track_signature is None:
        signature_delta = 1.0
    else:
        signature_delta = sum(
            abs(a - b) for a, b in zip(signature, _STATE.last_track_signature)
        )
    _STATE.last_track_signature = signature

    if _STATE.escape_frames > 0:
        _STATE.escape_frames -= 1
        # 脱困期间交替打轮（左右摆动），打破物理卡死的静摩擦。
        # 每 10 帧切换一次方向，叠加在已设置的基准舵角上。
        wiggle = 1.0 if (_STATE.escape_frames // 10) % 2 == 0 else -1.0
        wiggle_amplitude = float(profile.get("escape_wiggle_amplitude", 0.20))
        max_abs = profile["max_abs_steering"]
        # 倒车相位：脱困的领头若干帧先后退（负速度）拉开与卡点的距离。仅卡死类脱困
        # 设了 reverse_frames>0。三点掉头式：自行车模型 dθ/dt=(v/L)tanδ，倒车 v<0 时
        # 要让车头转向开阔侧，方向盘需反向打（与前冲相位相反），否则车头会越倒越扎进夹角。
        elapsed = _STATE.escape_total_frames - _STATE.escape_frames
        if elapsed <= _STATE.escape_reverse_frames:
            phase_sign = -_STATE.escape_steering_sign
            speed_out = -float(profile.get("escape_reverse_speed", 0.40))
        else:
            phase_sign = _STATE.escape_steering_sign
            speed_out = max(speed, _STATE.escape_speed)
        escape_steering = phase_sign * (
            _STATE.escape_steering_magnitude + wiggle * wiggle_amplitude
        )
        return (
            clamp(escape_steering, -max_abs, max_abs),
            speed_out,
            "escaping",
        )

    high_turn = (
        signals["curve_risk"] >= profile["escape_curve_threshold"]
        or abs(steering) >= profile["escape_steering_threshold"]
    )
    aligned_offset = (
        signals["offset_risk"] >= profile["escape_offset_threshold"]
        and track.lateral_error * track.lookahead_error
        >= profile["escape_offset_lookahead_alignment"]
    )
    large_offset_stall = (
        allow_geometry_escape
        and mode in {"hard_turn", "correcting"}
        and signals["offset_risk"] >= profile["escape_offset_threshold"]
        and speed <= profile["escape_offset_speed_threshold"]
        and abs(track.heading_error) <= profile["escape_offset_heading_abs_max"]
        and abs(track.curvature) <= profile["escape_offset_curve_abs_max"]
        and track.lateral_error * track.lookahead_error
        >= profile["escape_offset_lookahead_alignment"]
    )
    low_speed_stall = speed <= profile["escape_low_speed_threshold"]
    # 顶住栏杆但速度还没塌到低速阈值（落在 low_speed 覆盖空档）：几何冻结 + 大偏移 + 大反向打轮。
    pinned_stall = (
        abs(track.lateral_error) >= profile["escape_pinned_lateral_min"]
        and abs(steering) >= profile["escape_pinned_steering_min"]
        and speed <= profile["escape_pinned_speed_max"]
    )
    boundary_obstacle_stall = (
        allow_geometry_escape
        and track.near_obstacle
        and mode in {"hard_turn", "correcting"}
        and signals["margin_risk"] >= profile["escape_boundary_margin_risk"]
        and min(track.left_margin_near, track.right_margin_near)
        <= profile["escape_boundary_margin_max"]
        and speed <= profile["escape_boundary_speed_max"]
        and not track.lost
        and track.confidence >= profile["escape_boundary_min_confidence"]
    )
    stable_view = signature_delta <= profile["escape_signature_delta"]

    # 统一脱困方向：先按几何判断路在哪边；若推断已贴住单侧边界，则强制远离低余量一侧。
    escape_sign = _road_direction_sign(track)

    should_count_stall = False
    require_confidence = True
    trigger_frames = int(profile["escape_trigger_frames"])
    escape_frames = int(profile["escape_turn_frames"])
    escape_steering = profile["escape_turn_steering"]
    escape_speed = profile["escape_turn_speed"]
    # 倒车领头帧：默认 0（漂移/巡弯类脱困不倒车）；卡死类（pinned/low_speed/boundary）
    # 才先倒车拉开距离。
    escape_reverse_frames = 0
    if large_offset_stall:
        should_count_stall = True
        trigger_frames = int(profile["escape_offset_trigger_frames"])
        escape_frames = int(profile["escape_offset_frames"])
        escape_steering = profile["escape_offset_steering"]
        escape_speed = profile["escape_offset_speed"]
    elif boundary_obstacle_stall:
        # complex 后段 R017：近处静态车和右侧边界同时进入视野，车物理近停但策略速度仍在
        # 0.22-0.23，既没进 low_speed，也达不到 pinned 的大横向偏移。单侧余量为 0 时应更早脱困。
        should_count_stall = True
        require_confidence = False
        trigger_frames = int(profile["escape_boundary_trigger_frames"])
        escape_frames = int(profile["escape_boundary_frames"])
        escape_steering = profile["escape_boundary_steering"]
        escape_speed = profile["escape_boundary_speed"]
        escape_reverse_frames = int(profile.get("escape_boundary_reverse_frames", 0))
        escape_sign = _contact_escape_sign(track, escape_sign, profile)
    elif (
        allow_geometry_escape
        and mode == "hard_turn"
        and high_turn
        and not aligned_offset
        and speed <= profile["escape_turn_speed_max"]
    ):
        # 平稳巡弯同样满足"高弯量+签名稳定+高置信"，必须加低速门槛才算卡边
        # （complex 实跑在正常左弯中误触发过本路径，escape 强行打轮导致撞栏）。
        should_count_stall = True
    elif pinned_stall:
        # basic/complex 都启用：几何冻结地顶住栏杆、speed 又没低到触发 low_speed 时的兜底。
        should_count_stall = True
        trigger_frames = int(profile["escape_pinned_trigger_frames"])
        escape_frames = int(profile["escape_pinned_frames"])
        escape_steering = profile["escape_pinned_steering"]
        escape_speed = profile["escape_pinned_speed"]
        escape_reverse_frames = int(profile.get("escape_pinned_reverse_frames", 0))
        escape_sign = _contact_escape_sign(track, escape_sign, profile)
    elif low_speed_stall:
        should_count_stall = True
        # 贴墙被卡本就是低置信/丢线状态，放宽门槛，否则脱困永远进不来。
        require_confidence = False
        trigger_frames = int(profile["escape_low_speed_trigger_frames"])
        escape_frames = int(profile["escape_low_speed_frames"])
        escape_steering = profile["escape_low_speed_steering"]
        escape_speed = profile["escape_low_speed_speed"]
        escape_reverse_frames = int(profile.get("escape_low_speed_reverse_frames", 0))
        escape_sign = _contact_escape_sign(track, escape_sign, profile)

    confidence_ok = (not require_confidence) or (
        not track.lost and track.confidence >= profile["escape_min_confidence"]
    )
    if should_count_stall and stable_view and confidence_ok:
        _STATE.stall_frames += 1
    else:
        _STATE.stall_frames = 0

    if _STATE.stall_frames >= trigger_frames:
        _STATE.stall_frames = 0
        _STATE.escape_steering_sign = escape_sign
        _STATE.escape_steering_magnitude = escape_steering
        _STATE.escape_speed = escape_speed
        _STATE.escape_frames = escape_frames
        _STATE.escape_total_frames = escape_frames
        # 倒车帧数不超过总帧数的一半，保证后半段一定有前冲把车开出去。
        _STATE.escape_reverse_frames = min(escape_reverse_frames, escape_frames // 2)

    return steering, speed, mode


def _lane_line_correction(
    track: TrackState,
    signals: dict,
    mode: str,
    profile: dict,
    timestamp: float,
) -> float:
    """白线可信时计算后置舵角修正（R011 验证的形态 + 信任门控）。

    功能：让车身中线追向白色虚线，作为最终舵角上的有界微调。
    参数：`track` 携带感知层的双目白线状态，`signals` 提供弯道风险，`mode` 是本帧驾驶状态，
    `timestamp` 用于限制发车捕获窗口。
    返回：平滑后的舵角修正；白线不可信时向 0 衰减。
    逻辑：只修正最终输出，不进入 risk/mode/速度/入弯门控——R013/R014 证明
    白线一旦改写控制目标，会抬高 offset_risk、骗开入弯门控并压低直道速度。
    信任门控针对 2026-06-11 实跑取证的三类误锁：白色护栏（|offset| 大）、
    白车/斑马线（near_obstacle）、弯中错误线段（单帧出现、帧间突变、高弯量时直线拟合失真）。
    """

    normal_valid = (
        profile["enable"]
        and track.line_confidence >= profile["min_confidence"]
        and abs(track.line_offset) <= profile["offset_trust_max"]
        and not track.near_obstacle
        and mode != "escaping"
    )
    startup_valid = (
        profile["enable"]
        and track.red_environment
        and timestamp <= profile["startup_acquire_until"]
        and track.line_confidence >= profile["min_confidence"]
        and profile["startup_offset_min"]
        <= track.line_offset
        <= profile["startup_offset_trust_max"]
        and profile["startup_heading_min"]
        <= track.line_heading
        <= profile["startup_heading_max"]
        and not track.near_obstacle
        and mode != "escaping"
    )
    valid = normal_valid or startup_valid
    if (
        valid
        and _STATE.line_streak > 0
        and abs(track.line_offset - _STATE.line_last_offset)
        > profile["offset_jump_max"]
    ):
        valid = False
        _STATE.line_streak = 0
    if valid:
        _STATE.line_streak += 1
        _STATE.line_last_offset = track.line_offset
    else:
        _STATE.line_streak = 0

    target = 0.0
    confirm_frames = int(
        profile["startup_confirm_frames"]
        if startup_valid and not normal_valid
        else profile["confirm_frames"]
    )
    active = valid and _STATE.line_streak >= confirm_frames
    if active:
        max_correction = (
            profile["startup_max_correction"]
            if startup_valid and not normal_valid
            else profile["max_correction"]
        )
        curve_gate = (
            profile["startup_curve_gate"]
            if startup_valid and not normal_valid
            else profile["curve_gate"]
        )
        offset_target = track.line_offset * profile["offset_gain"]
        mixed_target = offset_target + track.line_heading * profile["heading_gain"]
        curve_scale = 1.0 - clamp(signals["curve_risk"] / curve_gate, 0.0, 1.0)
        target = clamp(mixed_target, -max_correction, max_correction) * curve_scale
        if (
            abs(track.line_offset) >= profile["offset_priority_min"]
            and track.line_offset * track.line_heading < 0.0
        ):
            # R027：弯中 line_heading 可与 offset 回中方向相反。弯很急时仍至少保留一部分
            # 纯 offset 修正，让车先回到白线附近，再相信斜率。
            offset_floor = clamp(
                offset_target * profile["offset_curve_min_scale"],
                -max_correction,
                max_correction,
            )
            if offset_floor * target <= 0.0 or abs(offset_floor) > abs(target):
                target = offset_floor
        # R039：弯中可信白线连续多帧显示车确在内侧（|offset| 大、与 heading 反号）时，curve_scale
        # 会把上面的回中修正压得很弱，不足以抵消 road-mask 的弯道向内预判。叠加一个有界“向外辅助”，
        # 方向恒为把车推回白线一侧（远离内栏），叠加后仍受 max_correction 钳制。
        if (
            profile["inside_assist_enable"]
            and track.red_environment
            and mode in {"hard_turn", "correcting"}
            and _STATE.line_streak >= int(profile["inside_assist_streak_min"])
            and abs(track.line_offset) >= profile["inside_assist_offset_min"]
            and track.line_offset * track.line_heading < 0.0
        ):
            excess = abs(track.line_offset) - profile["inside_assist_offset_min"]
            assist = math.copysign(
                clamp(
                    excess * profile["inside_assist_gain"],
                    0.0,
                    profile["inside_assist_max"],
                ),
                track.line_offset,
            )
            if assist * target >= 0.0:
                target = clamp(target + assist, -max_correction, max_correction)

    # R039：弯中白线短暂丢置信（虚线间隙）时，保持上一段向外修正并衰减，避免空档里 road-mask
    # 弯道预判继续把车往内切。只在 hard_turn/correcting 且上一段可信白线确显示车已偏到内侧时启用。
    turn_mode = mode in {"hard_turn", "correcting"}
    if active:
        _STATE.line_hold_frames = (
            int(profile["hold_frames"])
            if turn_mode and abs(_STATE.line_last_offset) >= profile["hold_offset_min"]
            else 0
        )
    elif _STATE.line_hold_frames > 0 and turn_mode:
        _STATE.line_hold_frames -= 1
        _STATE.line_correction *= profile["hold_decay"]
        return clamp(
            _STATE.line_correction,
            -profile["max_correction"],
            profile["max_correction"],
        )
    else:
        _STATE.line_hold_frames = 0

    if startup_valid and not normal_valid:
        alpha = profile["startup_smoothing"]
    elif (
        not valid
        and timestamp <= profile["startup_acquire_until"]
        and _STATE.line_correction > 0.0
    ):
        alpha = profile["startup_decay"]
    else:
        alpha = profile["smoothing"]
    _STATE.line_correction = _STATE.line_correction * alpha + target * (1.0 - alpha)
    return clamp(
        _STATE.line_correction, -profile["max_correction"], profile["max_correction"]
    )


def _update_policy_state(
    track: TrackState,
    steering: float,
    speed: float,
    mode: str,
    timestamp: float,
    profile: dict,
) -> None:
    """写回策略跨帧状态。

    功能：维护丢线计数、恢复缓冲、最近可信方向和上一帧控制量。
    参数：`track` 是赛道状态，`steering` 和 `speed` 是本帧输出。
    返回：无。
    逻辑：刚从丢线恢复时保守若干帧；高置信时更新恢复方向偏置。
    """

    if track.lost:
        _STATE.lost_frames += 1
    else:
        if _STATE.lost_frames > 0:
            _STATE.recovery_frames = int(profile["recovery_frames"])
        _STATE.lost_frames = 0
    if _STATE.recovery_frames > 0 and not track.lost:
        _STATE.recovery_frames -= 1

    if not track.lost and track.confidence >= profile["recovery_confidence"]:
        _STATE.last_good_bias = clamp(
            track.lateral_error * 0.55
            + track.lookahead_error * 0.25
            + track.curvature * 0.20,
            -1.0,
            1.0,
        )

    _STATE.last_steering = steering
    _STATE.last_speed = speed
    _STATE.last_timestamp = timestamp
    _STATE.last_mode = mode


def _continue_forced_escape(
    track: TrackState,
    timestamp: float,
    profile: dict,
) -> ControlCmd | None:
    """Advance an active multi-car recovery sequence, if one exists."""

    if not _STATE.force_escape_active:
        return None
    if _STATE.force_escape_frames <= 0:
        _STATE.force_escape_active = False
        _STATE.lost_streak = 0
        return None

    _STATE.force_escape_frames -= 1
    elapsed = _STATE.force_escape_total_frames - _STATE.force_escape_frames
    steering = clamp(
        _STATE.force_escape_sign * _STATE.force_escape_steering,
        -1.0,
        1.0,
    )
    if elapsed <= _STATE.force_escape_reverse_frames:
        speed = -float(profile.get("force_reverse_back_speed", 0.42))
    else:
        speed = _STATE.force_escape_speed
    _update_policy_state(track, steering, speed, "escaping", timestamp, profile)
    return ControlCmd(steering, speed)


def _blockage_triggered(
    track: TrackState,
    speed: float,
    drive_mode: str,
    profile: dict,
) -> bool:
    """Update independent blockage counters and report whether one fired."""

    if track.lost:
        _STATE.lost_streak += 1
        _STATE.not_stuck_frames = 0
    else:
        _STATE.not_stuck_frames += 1
        if _STATE.not_stuck_frames > 90:
            _STATE.lost_streak = 0
            _STATE.not_stuck_frames = 0

    zero_speed_frames = int(profile.get("force_reverse_zero_speed_frames", 120))
    zero_speed_threshold = float(
        profile.get("force_reverse_zero_speed_threshold", 0.05)
    )
    if speed <= zero_speed_threshold and drive_mode != "escaping":
        _STATE.zero_speed_streak += 1
    else:
        _STATE.zero_speed_streak = 0

    motion_still_threshold = float(profile.get("motion_still_threshold", 2.5))
    motion_still_frames = int(profile.get("motion_still_frames", 40))
    motion_min_cmd_speed = float(profile.get("motion_still_min_cmd_speed", 0.35))
    if (
        track.frame_motion < motion_still_threshold
        and speed >= motion_min_cmd_speed
        and drive_mode != "escaping"
    ):
        _STATE.motion_stall_streak += 1
    else:
        _STATE.motion_stall_streak = 0

    return (
        _STATE.lost_streak >= int(profile.get("force_reverse_lost_streak", 45))
        or _STATE.zero_speed_streak >= zero_speed_frames
        or _STATE.motion_stall_streak >= motion_still_frames
    )


def _start_forced_escape(track: TrackState, profile: dict) -> None:
    """Initialize a timed reverse/forward recovery toward the open side."""

    _STATE.force_escape_active = True
    _STATE.force_escape_frames = int(profile.get("force_reverse_lost_frames", 120))
    _STATE.force_escape_total_frames = _STATE.force_escape_frames
    _STATE.force_escape_reverse_frames = min(
        int(profile.get("force_reverse_back_frames", 65)),
        int(_STATE.force_escape_frames * 0.6),
    )
    _STATE.force_escape_speed = float(profile.get("force_reverse_lost_speed", 0.35))
    _STATE.force_escape_steering = float(
        profile.get("force_reverse_lost_steering", 0.88)
    )
    _STATE.force_escape_sign = _margin_escape_sign(track, _road_direction_sign(track))
    _STATE.lost_streak = 0
    _STATE.not_stuck_frames = 0
    _STATE.zero_speed_streak = 0
    _STATE.motion_stall_streak = 0


def _apply_opponent_avoidance(
    steering: float,
    track: TrackState,
    profile: dict,
) -> float:
    """Bias steering away from a nearby vehicle within available road space."""

    if not (
        profile.get("enable_opponent", True)
        and profile.get("opponent_avoid_steering_enable", True)
        and track.near_obstacle
        and not track.lost
        and track.confidence >= 0.30
    ):
        return steering

    avoid_gain = float(profile.get("opponent_avoid_steering_gain", 0.40))
    avoid_max = float(profile.get("opponent_avoid_steering_max", 0.18))
    margin_diff = track.right_margin_near - track.left_margin_near
    margin_bias = clamp(margin_diff * avoid_gain, -avoid_max, avoid_max)

    obstacle_x = clamp(float(track.obstacle_x), -1.0, 1.0)
    direction_bias = 0.0
    if abs(obstacle_x) > float(profile.get("opponent_direction_deadzone", 0.16)):
        direction_gain = float(profile.get("opponent_direction_steering_gain", 0.20))
        direction_bias = clamp(-obstacle_x * direction_gain, -avoid_max, avoid_max)

    avoid_bias = clamp(margin_bias + direction_bias, -avoid_max, avoid_max)
    return clamp(steering + avoid_bias, -1.0, 1.0)


def decide_control(
    track: TrackState, timestamp: float, mode: str = "no_other_cars"
) -> ControlCmd:
    """计算最终控制命令。

    功能：按唯一策略生成转向和速度。
    参数：`track` 是赛道状态，`timestamp` 是平台时间，`mode` 仅为接口兼容字段。
    返回：`ControlCmd`，包含 `steering` 和 `speed`。
    逻辑：所有 mode 和赛道都使用同一套参数；内部用状态机协同转向、速度和恢复策略。
    """

    profile = get_profile(mode)
    timestamp = float(timestamp)
    _maybe_reset_policy_by_timestamp(timestamp, profile)

    forced_escape = _continue_forced_escape(track, timestamp, profile)
    if forced_escape is not None:
        return forced_escape

    signals = _control_signals(track, profile)
    drive_mode = _select_mode(track, signals, timestamp, profile)
    straight_memory_active = _update_straight_memory(
        track, signals, drive_mode, profile
    )
    target_steering = _target_steering(track, signals, drive_mode, profile)
    _STATE.last_target_steering = target_steering
    steering = _smooth_steering(target_steering, drive_mode, timestamp, profile)
    target_speed = _target_speed(
        track,
        signals,
        drive_mode,
        steering,
        timestamp,
        profile,
        straight_memory_active=straight_memory_active,
    )
    _STATE.last_target_speed = target_speed
    _STATE.last_signals = dict(signals)
    _STATE.last_straight_memory_active = bool(straight_memory_active)
    speed = _smooth_speed(target_speed, timestamp, profile)
    # 低速贴墙脱困两条赛道都启用；依赖可靠几何的急弯/大偏移脱困仍只在 complex(red)。
    steering, speed, drive_mode = _escape_if_stalled(
        track, signals, steering, speed, drive_mode, profile, track.red_environment
    )
    # 跨帧状态记录修正前的舵角：白线修正只作用于最终输出，平滑、限速和脱困
    # 判据都不感知它（与 R011 在入口层后置修正时的动力学一致）。
    _update_policy_state(track, steering, speed, drive_mode, timestamp, profile)
    line_correction = _lane_line_correction(
        track, signals, drive_mode, LINE_FOLLOW_PROFILE, timestamp
    )
    final_steering = clamp(steering + line_correction, -1.0, 1.0)

    blockage_triggered = _blockage_triggered(track, speed, drive_mode, profile)
    if (
        profile.get("enable_opponent", True)
        and blockage_triggered
        and not _STATE.force_escape_active
        and timestamp > 3.0
    ):
        _start_forced_escape(track, profile)

    final_steering = _apply_opponent_avoidance(final_steering, track, profile)

    return ControlCmd(final_steering, speed)
