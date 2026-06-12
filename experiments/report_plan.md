# 实验报告规划文档

> 本文档记录报告各章节的关键信息，供最终写作时参考。
> 报告要求：图文并茂、内容丰富、结构清晰、写作规范、逻辑性强。

---

## 一、算法思想阐述

### 1.1 系统架构

控制流水线（Pipeline）：`摄像头图像 → 感知(Perception) → 估计(Estimator) → 策略(Policy) → 控制命令`

```
left_img, right_img (480×640 BGR)
        │
        ▼
  [Perception]  道路表面分割 + 走廊跟踪
        │
        ▼
  PerceptionObs  (center_points, edges, confidence, road_width)
        │
        ▼
  [Estimator]   中心线拟合 + 几何状态估计
        │
        ▼
  TrackState     (lateral_error, heading_error, curvature, lookahead_error, confidence)
        │
        ▼
  [Policy]      状态机 + 转向/速度决策
        │
        ▼
  ControlCmd     (steering ∈ [-1,1], speed ∈ [0,1])
```

### 1.2 感知模块（Perception）

核心技术点：
- **道路颜色估计**：从图像底部中心取 patch，用中位数抑制车道线和阴影干扰
- **暗灰低饱和分割**：gray ∈ [35, 105]，saturation ≤ 80，优先匹配沥青路面
- **颜色距离兜底**：当暗灰 mask 命中率不足时，用 BGR 距离替代
- **形态学处理**：小核腐蚀+膨胀清理噪声，再膨胀补齐路面细缝
- **走廊跟踪**：从近到远 12 条扫描线，每条选离上一条最近的连续道路段
- **置信度评分**：综合有效扫描比例(0.38)、宽度稳定性(0.22)、中心稳定性(0.22)、纹理分数(0.18)
- **融合策略**：左右摄像头根据置信度和近处一致性选择/合并

关键参数（VISION_PROFILE）：
- scan_count=12, min_valid_scans=4
- road_gray_min=35, road_gray_max=105, road_sat_max=80
- min_segment_width=24px, max_segment_gap=90px

### 1.3 估计模块（Estimator）

核心技术点：
- **坐标归一化**：x 映射到 [-1,1]（以图像中心 320 为基准），y 转成 [0,1] progress
- **多项式拟合**：≥5 点时用二次曲线，否则用直线
- **几何量提取**：
  - lateral_error：近处（progress ≤ 0.35）横向偏差中位数
  - lookahead_error：远处（progress ≥ 0.60）横向偏差中位数
  - heading_error：中心线在 progress=0.45 处的一阶导数
  - curvature：二次项系数（主）或远近偏差差值（兜底）
- **自适应平滑**：低置信时更多平滑（alpha 从 0.18 到 0.70）
- **丢线衰减**：各几何量独立衰减系数（0.76-0.85），保留方向记忆

### 1.4 策略模块（Policy）——核心创新

#### 1.4.1 状态机设计
```
  lost ──→ recovering ──→ hard_turn ──→ correcting ──→ cruise
   ↑                         │              │              │
   └─────────────────────────┴──────────────┴──────────────┘
                     (从任意状态回到 lost)
```

- **lost**：track.confidence < lost_confidence 或 track.lost=True，速度降至 lost_speed
- **recovering**：刚从 lost 恢复后缓冲 N 帧，或 confidence < recovery_confidence，速度上限 recovery_speed
- **hard_turn**：curve_risk > hard_turn_threshold，弯道速度限制
- **correcting**：|lateral_error| > correction_error，有限中速回中
- **cruise**：最佳状态，全速巡航

#### 1.4.2 三项策略创新

**（1）渐进式弯道速度（Progressive Corner Speed）**
- 问题：传统硬上限在缓弯中过于保守
- 方案：`turn_severity = (curve_risk - threshold) / (1.0 - threshold)`
- 效果：缓弯接近 base_speed，急弯才降到 hard_turn_speed
- 居中和低偏移时额外奖励 speed_bonus

**（2）赛车线偏置（Racing Line Bias）**
- 问题：车总是沿道路中心行驶，入弯角度不够宽
- 方案：`racing_bias = -curvature * gain - lookahead_error * lookahead_gain`
- 效果：右弯前偏左（外线入弯），左弯前偏右
- 仅当 curve_risk 明显时生效（bias_weight 加权）

**（3）预判式加减速（Predictive Speed）**
- 问题：入弯后才减速、出弯后才加速，响应滞后
- 方案：`predictive_curve = max(lookahead_error, curvature*0.7, heading_error*0.5)`
- 减速：`speed *= (1 - brake_gain * predictive_curve)`（全模式生效）
- 加速：仅在 cruise/correcting 模式下提前恢复速度

#### 1.4.3 Fastest vs Safe 双参数配置

| 参数 | Fastest | Safe | 说明 |
|------|---------|------|------|
| base_speed | 1.00 | 0.88 | 全速巡航基准 |
| recovery_confidence | 0.15 | 0.24 | 恢复模式阈值 |
| recovery_speed | 0.62 | 0.34 | 恢复模式限速 |
| hard_turn_speed | 0.40 | 0.26 | 弯道限速 |
| progressive_turn | True | False | 渐进式/硬上限 |
| racing_line_gain | 0.08 | 0.0 | 赛车线偏置 |
| predictive_brake_gain | 0.30 | 0.0 | 预判刹车 |
| steering_smoothing_cruise | 0.12 | 0.20 | 转向响应速度 |
| max_speed_increase_per_sec | 2.0 | 1.2 | 加速灵敏度 |

---

## 二、程序代码说明

### 2.1 项目结构
```
airacer-controller/
├── controller/           # 核心控制模块
│   ├── common.py         # 数据结构和工具函数
│   ├── params.py         # 参数配置（VISION/ESTIMATOR/CONTROL_FASTEST/CONTROL_SAFE）
│   ├── perception.py     # 道路视觉感知（numpy 实现）
│   ├── estimator.py      # 赛道几何估计
│   ├── policy.py         # 驾驶策略（状态机+转向+速度）
│   └── team_controller_local.py  # 本地入口
├── submissions/          # 单文件提交
│   ├── fastest/team_controller.py
│   ├── safe/team_controller.py
│   └── final/team_controller.py
├── tests/                # 24 个单元测试
├── scripts/
│   ├── build_submission.py   # 构建单文件提交
│   ├── run_test.py           # 自动化测试与记录
│   ├── analyze_telemetry.py  # 遥测数据分析
│   └── validate_submission.py
├── experiments/
│   ├── notes.md          # 实验记录
│   ├── report_plan.md    # 报告规划（本文档）
│   └── runs.csv          # 结构化实验数据
└── pkudsa.airacer/sdk/   # 官方 SDK（通过 .local 子目录使用）
```

### 2.2 关键数据结构
- `PerceptionObs`: center_points, left/right_edge_points, road_width_est, confidence, debug_flags
- `TrackState`: lateral_error, heading_error, curvature, lookahead_error, confidence, lost
- `ControlCmd`: steering ∈ [-1,1], speed ∈ [0,1]

### 2.3 代码行数统计（可后续补充）

---

## 三、测试过程报告

### 3.1 测试体系
- **24 个单元测试**（pytest）：覆盖 contracts、estimator、policy、output_range、interface、submission_static
- **本地校验**：`validate_submission.py` 校验接口契约
- **SDK 校验**：`validate_controller.py` 校验性能 + 规则合规
- **Webots 仿真**：本地图形化/无头仿真验证赛道表现
- **遥测分析**：`analyze_telemetry.py` 从 telemetry 提取圈速和速度统计

### 3.2 实验迭代

| 实验 | 参数变化 | 平均速度 | 最高速度 | 备注 |
|------|----------|----------|----------|------|
| 基线 | recovery_confidence=0.28, recovery_speed=0.55 | 2.57 m/s | 5.84 m/s | 车长期处于 recovering |
| 第1轮 | recovery_confidence=0.18, recovery_speed=0.62 | 2.98 m/s | 4.79 m/s | +16% |
| 第2轮 | recovery_confidence=0.10, recovery_speed=0.72 | 3.11 m/s | 4.80 m/s | +21%，但撞护栏 |
| 第3轮 | 平衡参数 + 降低赛车线 + 提高稳定性 | TBD | TBD | 目标：安全快速 |

### 3.3 多车测试（待完成）
- 测试配置：fastest (car_1) + safe (car_2) on basic/complex
- 关注点：碰撞检测、碰撞后恢复、2s stop penalty、DQ 逻辑

### 3.4 线上测试策略（待完成）
- dev slot → fastest（验证圈速）
- main slot → safe（保证完赛）

---

## 四、小组分工和实验过程总结

### 4.1 分工（待填写）

### 4.2 时间线
- 2026-06-10：项目理解、本地调试、checkpoint 修复
- 2026-06-12：策略优化（参数分离、渐进式弯道、赛车线、预判加减速）、实验记录体系搭建

### 4.3 关键决策记录
1. 用 numpy 代替 OpenCV 实现视觉模块，消除了外部依赖
2. 将 fastest 和 safe 拆分为独立参数配置，便于针对性优化和测试
3. 在本地 supervisor 中修复 checkpoint 坐标，替代遥测轨迹分析法
4. 感知置信度评分对控制策略的影响是整个系统的核心瓶颈

---

## 五、待补充内容

- [ ] 赛道示意图
- [ ] 控制流程图
- [ ] 速度分布对比图（实验前后）
- [ ] 圈速对比表
- [ ] 多车场景测试截图
- [ ] 最终参数表
- [ ] 代码行数/复杂度统计
