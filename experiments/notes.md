# 实验记录

把每次平台测试或本地 Webots 结果写到 `runs.csv`。较长观察放在这里，重点记录改动、现象和下一步。

## 2026-06-12 策略优化实验（optimize-policy 分支）

### 已完成的改动

#### 1. 参数配置分离（params.py）
- `CONTROL_FASTEST`：激进圈速策略，base_speed=1.00，渐进式弯道限速、赛车线偏置、预判加减速
- `CONTROL_SAFE`：稳健多车策略，base_speed=0.88，硬上限弯道限速、无赛车线、无预判加减速
- `get_profile(name)` 根据模式名返回对应参数字典

#### 2. 策略逻辑增强（policy.py）
三项新增机制：
- **渐进式弯道速度**（fastest 专属）：`turn_severity` 在 hard_turn_threshold 到 1.0 之间线性缩放，缓弯接近 base_speed，急弯才降到 hard_turn_speed
- **赛车线偏置**（fastest 专属）：根据曲率和前瞻误差向弯道外侧偏移（outside-inside-outside），仅在 curve_risk 明显时生效
- **预判式加减速**（fastest 专属）：利用 lookahead_error 和 curvature 提前感知前方弯道，提前减速/加速
- **Bug 修复**：预判加速从不安全的 `mode != "lost"` 改为严格的 `mode in ("cruise", "correcting")`，防止绕过 recovering 的限速

#### 3. 感知置信度优化（perception.py）
numpy 实现的感知评分过于严苛：
- `mask_fill_ratio` 处罚因子从 0.25 提升到 0.50
- `fallback` 处罚下限从 0.55 提升到 0.70，处罚斜率从 0.06 降到 0.04

#### 4. 本地测试机制修复（supervisor.py）
- CHECKPOINTS 坐标从旧的 airacer.wbt 更新为 track_basic.wbt 实际坐标
  - CP0=(-13.5, 130), CP1=(40, 279), CP2=(100, 125), CP3=(40, -30)
- FINISH_LINES half_w 从 0.5 扩宽到 12.0（修复 x≈-20 处无法检测到穿线的问题）

#### 5. 实验工具链
- `scripts/run_test.py`：构建→校验→性能采集→可选 Webots 仿真→CSV 记录
- `scripts/analyze_telemetry.py`：从 telemetry 轨迹自动检测圈速、速度统计

### 实验记录

#### 实验 1：原始参数基线（commit 587999b）
| 指标 | 值 |
|------|-----|
| 平均速度 | 2.57 m/s |
| 最高速度 | 5.84 m/s |
| 2-3 m/s 占比 | 63.4% |
| >5 m/s 占比 | 0.1% |
| 完成圈数 | 0（CP1-CP3 检测到，但 finish line 未触发） |

**分析**：车辆长期处于 recovering 模式（recovery_confidence=0.28），速度被限制在 recovery_speed=0.55。感知置信度（numpy 实现）输出偏低，estimator 进一步压缩，导致 track.confidence 极少超过 0.28。

#### 实验 2：第一轮阈值调优（recovery_confidence=0.18, recovery_speed=0.62）
| 指标 | 值 | vs 基线 |
|------|-----|---------|
| 平均速度 | 2.98 m/s | +16% |
| 最高速度 | 4.79 m/s | -18% |
| 2-3 m/s 占比 | 35.3% | -28pp |
| 3-4 m/s 占比 | 51.4% | +31pp |
| CP 检测 | CP1✓ CP2✓ | |

**分析**：速度分布从 2-3 m/s 明显右移到 3-4 m/s。最高速度下降是因为模拟提前结束（车还在弯道中）。

#### 实验 3：激进阈值（recovery_confidence=0.10, recovery_speed=0.72）
| 指标 | 值 | vs 基线 |
|------|-----|---------|
| 平均速度 | 3.11 m/s | +21% |
| 3-4 m/s 占比 | ~51% | +31pp |
| CP 检测 | CP1✓ CP2✓ | |

**问题**：用户观察转弯时撞栏杆。原因是 recovery_confidence=0.10 几乎禁用了 recovering 限速，车以高速入弯，且感知低置信时 curvature/heading 信号偏弱，hard_turn 模式未能及时触发。

### 下一步

1. **恢复平衡参数**：recovery_confidence 回到 0.15，降低赛车线增益、提高转向稳定性
2. **改进 run_test.py**：集成 analyze_telemetry.py 自动获取圈速和速度统计
3. **多车测试**：fastest (car_1) + safe (car_2) on basic/complex
4. **线上测试配置**：fastest→dev slot, safe→main slot

---

## 2026-06-10 Webots basic 调试

- 修复了直道上有效扫描点纵向跨度不足导致的假丢线：`min_y_span` 从 60 降到 30 后，6 到 8 个远处扫描点不再直接进入 lost。
- 右上角固定卡点的主要原因是车贴右侧护栏时，远处弯道项抵消了回中项。现在 `curve_risk` 可直接触发 hard_turn，并在回中项和远处项方向冲突时削弱远处项。
- 当前最好结果：非 debug 单文件控制器在 `basic` 上物理完成一圈。telemetry 显示 `t=288.187s` 从左侧穿过起点区域 `x=-19.498,y=122.6`，300 秒结束在 `x=-19.741,y=155.472`。
- 本地 metadata 仍显示 `timeout/laps=0`，原因是 SDK supervisor 的 checkpoint 坐标和 `track_basic.wbt` 实际赛道不一致。这里以 telemetry 轨迹作为实跑是否穿过赛道的证据。
- 尝试把全宽道路段放行到 `max_segment_width_ratio=1.0` 后，直道假丢线减少，但中心线过度居中，右上角再次贴外侧卡住，已回退。
- 最后采用的提速方式是：保留右上角/右下角的回中冲突抑制，只提高 lost/recovery 阶段速度，并给居中、高置信 hard_turn 小幅速度奖励。
