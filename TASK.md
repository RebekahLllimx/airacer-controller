# AI Racer Control Task and Submission Contract

## Objective

The controller receives synchronized left and right BGR camera frames from a simulated vehicle and returns steering and speed for every frame. The same interface is used for solo time trials and multi-car races:

```python
def control(left_img, right_img, timestamp):
    return steering, speed
```

- `left_img` and `right_img` are NumPy image arrays supplied by the platform.
- `timestamp` is simulation time in seconds and may move backwards when a run restarts.
- `steering` must remain in `[-1, 1]`.
- Forward speed normally remains in `[0, 1]`; the local multi-car recovery profile may request a short negative speed when the simulator supports reverse motion.

The controller has no direct access to vehicle pose, wheel speed, map coordinates, or opponent telemetry. Road geometry, motion, and nearby vehicles must be inferred from the camera stream and retained controller state.

## Decision loop

Every call follows the same sequence:

```text
stereo camera frames
    -> road and lane perception
    -> geometric state estimation
    -> driving-mode and risk selection
    -> steering and speed policy
    -> output bounds
```

State may be carried across frames, but it must be reset when the timestamp rolls back or a new run begins.

## Submission profiles

| Profile | Scenario | Additional behaviour |
| --- | --- | --- |
| `no_other_cars` | Solo time trial | Core road following, speed control, and conservative recovery |
| `with_other_cars` | Traffic and race scenarios | Opponent detection, directional avoidance, blockage detection, and reverse recovery |

The platform does not pass a scenario flag to `control()`. The build process therefore generates a separate self-contained file for each profile:

```bash
python scripts/build_submission.py --mode no_other_cars
python scripts/build_submission.py --mode with_other_cars
```

Both profiles share the driving core. Traffic-specific perception and policy paths are explicitly gated so they do not execute in the solo build.

## File and runtime constraints

The uploaded artifact is one Python file containing `control()`. It must be importable by the platform, stay within the configured size and timing limits, avoid file or network I/O, and use only permitted modules. The local modular source is assembled in this order:

```text
common -> params -> opponent -> perception -> estimator -> policy -> team_controller_local
```

The maintained source of truth is `controller/`; generated files under `submissions/` should not be edited by hand.

## Validation layers

1. **Unit and contract tests** check image handling, state reset, profile isolation, geometry, policy bounds, and build behavior.
2. **Repository validation** checks the generated file for imports, forbidden calls, interface shape, and return bounds.
3. **Official SDK validation** applies the platform rules and mock timing checks.
4. **Webots simulation** tests closed-loop behavior, including track completion, collisions, multi-car interaction, and recovery.

Offline tests establish software correctness, not driving quality. A policy change should be evaluated in Webots before its performance is reported.

## Race objectives

Solo events optimize completion and lap time without major collisions. Multi-car events additionally reward finishing position and penalize serious collisions or disqualification. The exact scoring and race schedule are platform rules and may change; this repository treats them as evaluation configuration rather than hard-coded controller logic.

## Repository implementation

- [`controller/`](controller/) contains the modular implementation.
- [`scripts/build_submission.py`](scripts/build_submission.py) produces the platform artifact.
- [`tests/`](tests/) covers deterministic contracts and regressions.
- [`docs/technical_manual.md`](docs/technical_manual.md) explains the controller design.
- [`docs/official_testing.md`](docs/official_testing.md) explains SDK and Webots validation.
- [`experiments/`](experiments/) retains structured historical evidence and selected figures.
