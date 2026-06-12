"""策略参数配置。

功能概述：集中保存视觉、估计和控制策略参数。
输入输出：输入 profile 名称（fastest/safe），输出对应控制参数。
处理流程：fastest 激进追求圈速，safe 保守保证完赛。
"""

VISION_PROFILE = {
    "roi_top_ratio": 0.42,
    "scan_top_ratio": 0.50,
    "scan_bottom_ratio": 0.92,
    "scan_count": 12,
    "row_band": 2,
    "road_lab_threshold": 34.0,
    "road_gray_min": 35.0,
    "road_gray_max": 105.0,
    "road_sat_max": 80.0,
    "dark_mask_min_fill": 0.04,
    "texture_gray_std_scale": 35.0,
    "min_segment_width": 24.0,
    "max_segment_gap": 90.0,
    "max_segment_width_ratio": 0.995,
    "max_center_jump_ratio": 0.35,
    "min_valid_scans": 4,
    "min_camera_confidence": 0.12,
    "fusion_max_offset_gap": 0.18,
    "fusion_confidence_margin": 0.18,
    "fusion_merge_gap": 0.12,
    "fusion_merge_min_confidence": 0.35,
}

ESTIMATOR_PROFILE = {
    "image_center_x": 320.0,
    "x_scale": 320.0,
    "lost_confidence": 0.05,
    "min_center_points": 3,
    "min_good_points": 8,
    "min_y_span": 30.0,
    "min_y_span_good": 220.0,
    "min_road_width_for_conf": 20.0,
    "near_progress_max": 0.35,
    "far_progress_min": 0.60,
    "near_eval_progress": 0.15,
    "far_eval_progress": 0.75,
    "heading_eval_progress": 0.45,
    "poly2_min_points": 5,
    "heading_gain": 1.25,
    "curvature_gain": 1.45,
    "fallback_curvature_gain": 0.70,
    "max_fit_error": 0.22,
    "smooth_alpha": 0.28,
    "low_conf_extra_smoothing": 0.30,
    "min_smooth_alpha": 0.18,
    "max_smooth_alpha": 0.70,
    "curve_smooth_alpha": 0.46,
    "max_error_delta": 0.22,
    "max_heading_delta": 0.20,
    "max_curvature_delta": 0.18,
    "lost_lateral_decay": 0.85,
    "lost_heading_decay": 0.78,
    "lost_curvature_decay": 0.76,
    "lost_lookahead_decay": 0.82,
    "timestamp_reset_gap": 2.0,
}

# ── Fastest：激进圈速策略 ──────────────────────────────────────────────
# 目标：最短单圈时间。直道极速、弯道少减速、出弯快加速、赛车线走线。

CONTROL_FASTEST = {
    # ── 速度基础 ──
    "base_speed": 1.00,
    "max_speed": 1.00,
    "min_speed": 0.18,
    "start_caution_seconds": 0.4,
    "start_speed": 0.55,
    # ── 丢线与恢复 ──
    "lost_confidence": 0.06,
    "recovery_confidence": 0.15,
    "lost_speed": 0.34,
    "recovery_speed": 0.62,
    "recovery_frames": 2,
    # ── 弯道速度（渐进式） ──
    "hard_turn_speed": 0.40,
    "hard_turn_center_speed_bonus": 0.35,
    "correction_speed": 0.58,
    "hard_turn_threshold": 0.22,
    "correction_error": 0.22,
    "progressive_turn": True,
    # ── 风险权重 ──
    "risk_curve_weight": 0.44,
    "risk_offset_weight": 0.26,
    "risk_confidence_weight": 0.22,
    "risk_lost_weight": 0.80,
    # ── 转向近/远项权重 ──
    "near_weight_base": 0.95,
    "near_weight_offset_boost": 0.55,
    "far_weight_base": 0.70,
    "far_weight_curve_boost": 0.45,
    "far_conflict_offset_scale": 3.40,
    "far_conflict_min_scale": 0.04,
    # ── 转向增益 ──
    "gain_lateral": 0.72,
    "gain_lookahead": 1.00,
    "gain_heading": 1.05,
    "gain_curve": 0.28,
    "gain_lateral_nonlinear": 0.22,
    "gain_curve_nonlinear": 0.05,
    "steering_deadzone": 0.012,
    # ── 赛车线 ──
    "racing_line_gain": 0.08,
    "racing_line_lookahead_gain": 0.05,
    # ── 速度降幅因子 ──
    "curve_slowdown": 0.55,
    "curve_power": 1.35,
    "offset_slowdown": 0.30,
    "offset_power": 1.25,
    "min_confidence_factor": 0.58,
    "steering_slowdown": 0.24,
    "steering_power": 1.15,
    # ── 转向平滑（更快响应但更稳定） ──
    "steering_smoothing_cruise": 0.12,
    "steering_smoothing_turn": 0.12,
    "steering_smoothing_correction": 0.12,
    "steering_smoothing_recovery": 0.26,
    "max_steering_delta": 0.50,
    # ── 加减速变化率 ──
    "max_speed_increase_per_sec": 2.0,
    "max_speed_decrease_per_sec": 3.8,
    "nominal_dt": 0.032,
    "timestamp_reset_gap": 2.0,
    # ── 预判式速度 ──
    "predictive_brake_gain": 0.30,
    "predictive_accel_gain": 0.15,
}

# ── Safe：稳健多车策略 ─────────────────────────────────────────────────
# 目标：保证完赛、减少碰撞。弯道保守、丢失时迅速降速、不追求极限走线。

CONTROL_SAFE = {
    # ── 速度基础 ──
    "base_speed": 0.88,
    "max_speed": 0.92,
    "min_speed": 0.14,
    "start_caution_seconds": 1.0,
    "start_speed": 0.32,
    # ── 丢线与恢复 ──
    "lost_confidence": 0.08,
    "recovery_confidence": 0.24,
    "lost_speed": 0.20,
    "recovery_speed": 0.34,
    "recovery_frames": 6,
    # ── 弯道速度（硬上限） ──
    "hard_turn_speed": 0.26,
    "hard_turn_center_speed_bonus": 0.20,
    "correction_speed": 0.42,
    "hard_turn_threshold": 0.18,
    "correction_error": 0.28,
    "progressive_turn": False,
    # ── 风险权重（置信度权重更高） ──
    "risk_curve_weight": 0.38,
    "risk_offset_weight": 0.26,
    "risk_confidence_weight": 0.30,
    "risk_lost_weight": 0.90,
    # ── 转向近/远项权重（更偏近处） ──
    "near_weight_base": 1.00,
    "near_weight_offset_boost": 0.60,
    "far_weight_base": 0.60,
    "far_weight_curve_boost": 0.35,
    "far_conflict_offset_scale": 3.80,
    "far_conflict_min_scale": 0.03,
    # ── 转向增益 ──
    "gain_lateral": 0.60,
    "gain_lookahead": 0.80,
    "gain_heading": 0.88,
    "gain_curve": 0.20,
    "gain_lateral_nonlinear": 0.14,
    "gain_curve_nonlinear": 0.03,
    "steering_deadzone": 0.018,
    # ── 赛车线（关闭） ──
    "racing_line_gain": 0.0,
    "racing_line_lookahead_gain": 0.0,
    # ── 速度降幅因子（更保守） ──
    "curve_slowdown": 0.72,
    "curve_power": 1.40,
    "offset_slowdown": 0.42,
    "offset_power": 1.30,
    "min_confidence_factor": 0.50,
    "steering_slowdown": 0.32,
    "steering_power": 1.20,
    # ── 转向平滑（更稳定） ──
    "steering_smoothing_cruise": 0.20,
    "steering_smoothing_turn": 0.18,
    "steering_smoothing_correction": 0.18,
    "steering_smoothing_recovery": 0.32,
    "max_steering_delta": 0.38,
    # ── 加减速变化率（更平缓） ──
    "max_speed_increase_per_sec": 1.2,
    "max_speed_decrease_per_sec": 2.5,
    "nominal_dt": 0.032,
    "timestamp_reset_gap": 2.0,
    # ── 预判式速度（关闭） ──
    "predictive_brake_gain": 0.0,
    "predictive_accel_gain": 0.0,
}


def get_profile(name: str) -> dict:
    """读取控制 profile。

    功能：为顶层控制器提供 fastest 或 safe 对应的控制参数。
    参数：`name` 为 "fastest" 或 "safe"。
    返回：对应参数字典的浅拷贝。
    逻辑：非法模式回退 fastest。
    """

    if name == "safe":
        return dict(CONTROL_SAFE)
    return dict(CONTROL_FASTEST)
