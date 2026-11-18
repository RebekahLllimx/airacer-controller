# AI Racer Controller Technical Manual

The controller combines color-based road perception, geometric track estimation, and a stateful steering and speed policy. Parameter sweeps and simulator measurements are recorded in the [experiment report](../experiments/report.md).

## 1. System overview

The platform calls `control(left_img, right_img, timestamp)` once per frame. The implementation is split into four stages:

```text
left and right BGR images
        |
        v
perception.extract_observation() -> PerceptionObs
        |
        v
estimator.estimate_track()       -> TrackState
        |
        v
policy.decide_control()          -> ControlCmd
        |
        v
clamp_cmd()                      -> steering, speed
```

The dataclasses in `controller/common.py` are contracts between the stages. Image processing remains in perception, geometric interpretation remains in estimation, and policy never reads raw pixels.

Each stateful stage owns one explicit session object:

- `PerceptionState` stores temporal road anchors and frame-motion history.
- `EstimatorState` stores the last geometric estimate and lane-memory latch.
- `PolicyState` stores mode hysteresis, smoothing, line correction, and recovery progress.

The runtime uses one state object per stage and assumes one vehicle per controller instance. Reset functions replace the complete object. Separate vehicles must run separate instances; sharing one imported module across simultaneous vehicles would mix their frame histories.

## 2. Perception

### 2.1 Road masks

Each camera is processed independently. The controller crops the lower image region, then classifies pixels using calibrated HSV and Lab colour profiles for asphalt, shoulder, guardrail, grass, red ground, sky, and vehicle colours. The asphalt mask is cleaned with morphology and excludes bright shoulder and non-road regions. A Canny edge mask is retained only as a fallback when colour segmentation fails.

The implementation keeps this pipeline explicit: road-surface segmentation, translucent-checkpoint bridging, and optional opponent detection are separate functions. This prevents scene-specific logic from being folded into one opaque mask builder.

The perception output also records texture, mask fill ratio, and debug flags. These quantities help distinguish a narrow but valid road region from an empty or overfilled mask.

### 2.2 Scan-line corridor tracking

Horizontal scan lines are evaluated from near to far. Continuous road segments are merged across small gaps, filtered by width, and scored against the preceding valid centre. The selected segment contributes:

- road-centre points;
- left and right boundary points;
- local road width;
- scan confidence.

Previous-frame centres are stored as coordinate tuples and matched to the nearest scan height. This temporal anchor reduces candidate switching when a wide or fragmented mask produces multiple plausible segments. Timestamp rollback and explicit resets clear the stored state so it cannot leak across runs.

Collection and scoring are separate stages. `_collect_road_samples()` traces the corridor; `_completed_scan()` converts those samples into a confidence-scored camera contract. Stereo fusion then either selects the reliable camera or merges compatible scans, while obstacle fusion remains independent of road geometry.

### 2.3 Lane-marking detection

The centre marking is detected separately from the road mask. Candidate pixels must be bright, close to neutral white, narrow, and surrounded by dark road pixels. This rejects many guardrail and shoulder highlights. Near and far marking points produce lane offset, heading, and confidence. Stereo agreement receives more trust; a restricted monocular fallback is available during startup and specific scenes.

### 2.4 Opponent observation

The multi-car profile inspects a lower central region for bright, dark, and saturated vehicle-body components. Connected components are filtered by area and shape, then summarized as:

- `near_obstacle`;
- normalized horizontal position `obstacle_x`;
- approximate relative size `obstacle_size`.

The same profile computes low-resolution frame motion. Low visual motion while a forward command is active provides a proxy for blockage when direct wheel-speed feedback is unavailable. The solo profile bypasses these operations.

## 3. Track-state estimation

Image coordinates are normalized around the camera centre. A line or quadratic curve is fitted to the selected road centres depending on point count and span. The estimator derives:

| State | Interpretation |
| --- | --- |
| `lateral_error` | Near-field displacement from the road centre |
| `lookahead_error` | Far-field displacement used for anticipation |
| `heading_error` | Direction of the fitted road centreline |
| `curvature` | Trusted quadratic bending term |
| `left_margin_near`, `right_margin_near` | Near-field clearance to road boundaries |
| `geometry_confidence` | Combined reliability of the observation and fit |

Curvature trust falls when points are sparse, cover too little vertical range, or fit poorly. Low-confidence frames decay the previous estimate rather than forcing every state to zero. Lane-marking evidence can pull the road-centre estimate toward a more reliable centre reference when its confidence and shape checks pass.

`_measure_geometry()` performs the stateless fit and returns an immutable `GeometryEstimate`. `_smoothed_track()` is the only stage that combines that measurement with prior-frame state. Keeping measurement separate from temporal smoothing makes both failure modes and tests easier to interpret.

## 4. Driving policy

### 4.1 Risk and modes

The policy converts curvature, offset, confidence, loss state, and boundary margins into bounded risk components. A state machine selects among `cruise`, `correcting`, `hard_turn`, `lost`, and `recovering`, with hysteresis to avoid rapid switching.

### 4.2 Steering

The target steering combines near-field centring with far-field anticipation:

```text
centre term    = lateral error * lateral gain
look-ahead term = look-ahead error * look-ahead gain
                + heading error * heading gain
                + curvature * curvature gain
```

The far-field term is gated by physical corner arrival. A distant bend can be visible before the vehicle reaches the corner; using it at full strength too early cuts inside. The gate therefore grows primarily from near-field lateral displacement, opens slightly earlier at higher speed, remains latched through the corner, and decays after exit. This produces a more stable outside-in-out trajectory.

When a trusted lane marking indicates that the vehicle is already inside the intended path, a bounded corner-relief term reduces further inward anticipation. The final command then receives smoothing, rate limiting, speed-dependent steering reduction, a dead zone, and output clipping.

Corner arrival, inside-corner relief, near/far term blending, and drive-mode adjustments are separate policy functions. Recovery selection returns an immutable `EscapePlan` before policy state changes, so plan selection and execution can be tested independently.

### 4.3 Speed

Target speed begins from a profile base speed and is reduced multiplicatively by curvature, offset, confidence, steering demand, and boundary risk. Hard-turn and recovery modes impose additional limits. Acceleration is smoothed more slowly than deceleration so the controller can react quickly to risk without oscillating after it clears.

## 5. Multi-car behaviour

Traffic behavior is an extension of the shared driving policy:

1. A centred, nearby vehicle lowers target speed.
2. Relative free space on the left and right creates a bounded avoidance bias.
3. Repeated line loss or low frame motion accumulates blockage state.
4. A timed recovery phase steers toward open road and may request reverse speed in the local simulator.
5. Recovery priority and cooldown rules prevent ordinary lane tracking from immediately cancelling the maneuver.

All traffic-specific paths are guarded by the active profile. This protects solo driving from false opponent detections and unnecessary recovery behavior.

## 6. Output and failure handling

`clamp_cmd()` bounds both values to `[-1, 1]`. Ordinary forward speed is nonnegative; negative speed is reserved for traffic recovery and requires SDK reverse-motion support. The stage functions reset temporal state on timestamp rollback. The entry callback catches an escaping stage exception and returns `(0.0, 0.0)`. Debug logging and frame capture are available in local test builds; formal submissions exclude this file I/O.

## 7. Build architecture

Modular source is convenient for development, but the platform requires a single file. `scripts/build_submission.py` concatenates modules in dependency order, removes bundled imports by parsing their AST source spans, selects a profile, and writes `team_controller.py`. The import pass is independent of formatter line wrapping. Repository and SDK validators inspect the generated artifact rather than assuming the modular source is submission-compatible.

## 8. Validation strategy

- Unit tests verify deterministic functions and state transitions.
- Contract tests verify data fields, output bounds, timestamp reset, and profile isolation.
- Build tests verify both generated profiles and forbidden-import rules.
- SDK checks verify platform compatibility and mock execution time.
- Webots tests verify closed-loop driving, collision behavior, and recovery.

The layers answer different questions. Passing unit tests cannot establish lap completion, while a successful lap does not replace interface and regression tests.
