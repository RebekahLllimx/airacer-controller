# avoidance_webots — 多车避让的 Webots 取证图

这组图来自 **AI 直接驱动 Webots 跑的确定性摆位场景**（2026-06-19，with_other_cars profile），
用来在报告里讲清"对手挡在前面时本车具体怎么做"。六车同策略混跑名次只反映发车位置、且车与车
很少真正贴近，无法稳定复现"对手正前方挡路"，所以改用 teleport 把对手固定到主直道上的指定位姿，
本车从发车格正常起步驶入。

## 复现方式

```bash
SDK=/Users/day/Desktop/Github/pkudsa.airacer/sdk
STAGE="$SDK/webots/worlds/.codex_stage_complex.wbt"

# (1) 摆对手位姿（贴上栏=可通行；居中斜停=堵死）
python scripts/make_teleport_world.py --world complex --car-slot car_2 \
  --x 50 --y -25.3 --heading 0.0 --out "$STAGE"          # 干净超车场景
# python ... --x 48 --y -28 --heading 0.6 ...            # 堵死场景

# (2) 本车 with_other_cars(抽帧+控制日志)，对手=停车桩 experiments/scenarios/parked_controller.py
python scripts/build_submission.py --mode with_other_cars \
  --debug-log .tmp/avoid_scenario/control.jsonl \
  --dump-frames .tmp/avoid_scenario/frames --dump-frame-stride 4 --dump-frame-end 22 \
  --out .tmp/avoid_scenario/ours.py

# (3) 直接跑 Webots（headless）
python "$SDK/run_local.py" --world "$STAGE" \
  --car "$PWD/.tmp/avoid_scenario/ours.py:car_1:ours" \
  --car "$PWD/experiments/scenarios/parked_controller.py:car_2:opp" \
  --skip-validate --fast --minimize --batch

# (4) 出图：摄像头检测叠加 + Webots 式顶视
python scripts/analyze_perception_dump.py .tmp/avoid_scenario/frames \
  --control-log .tmp/avoid_scenario/control.jsonl --mode with_other_cars \
  --overlay-dir .tmp/avoid_scenario/overlays --at 4.5
python scripts/plot_topdown.py --telemetry <telemetry.jsonl> --world complex \
  --cars ours,opp --focus ours --window 2,9 --annotate-speed --out <out.png>
```

## 文件

| 文件 | 场景 | 内容 |
|---|---|---|
| `pass_camera_detect.png` | 对手贴上栏（可通行） | 左相机叠加：对手车身红框 + obstacle_x≈−0.7 品红竖线 |
| `pass_topdown.png` | 同上 | Webots 式顶视：本车贴下侧开阔处干净超车，速度全程未掉零 |
| `blocked_camera_angled.png` | 对手斜停堵死车道 | 左相机：斜过来的对手车身占近半画面 + 检测框 |
| `blocked_topdown.png` | 同上 | 顶视：本车减速朝开阔侧打舵，横向净空不足被楔住 → 触发脱困 |

顶视背景（红色场地 + 灰色路面）由 `scripts/track_geometry.py` 从 `track_complex.wbt` 还原，
与 telemetry 同坐标系；车体按 heading 朝向绘制，浓度随时间从淡到浓。
