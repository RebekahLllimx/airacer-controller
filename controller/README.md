# Controller Module Guide

`controller/` contains the maintained, modular implementation. The generated files in `submissions/` should be rebuilt with `scripts/build_submission.py` rather than edited by hand.

## Data flow

```text
left_img / right_img
    -> perception.extract_observation()   -> PerceptionObs
    -> estimator.estimate_track()         -> TrackState
    -> policy.decide_control()             -> ControlCmd
    -> clamp_cmd()                         -> steering, speed
```

## Module boundaries

| Module | Responsibility |
| --- | --- |
| `common.py` | Dataclasses and output-bound helpers shared across stages. Field names form the module contracts. |
| `params.py` | Central source for perception and control parameters, including solo and multi-car profiles. |
| `perception.py` | Road masks, scan-line road centres, lane markings, confidence, and obstacle observations from camera frames. |
| `estimator.py` | Convert perception output into lateral, heading, look-ahead, curvature, margin, and loss state; own cross-frame geometry memory. |
| `policy.py` | Map `TrackState` to steering, target speed, driving mode, avoidance, and recovery behaviour; own policy hysteresis and recovery state. |
| `opponent.py` | Detect nearby vehicles and estimate their horizontal position and apparent size for the multi-car profile. |
| `team_controller_local.py` | Connect the stages, reset state when required, handle exceptions, and clamp the final command. |

The separation is intentional: perception does not issue control commands, estimation does not read raw images, and policy does not perform image processing. This keeps failures attributable to a stage and allows contract-level tests.

## Build and validate

```bash
python scripts/build_submission.py --mode no_other_cars
python scripts/build_submission.py --mode with_other_cars

python scripts/validate_submission.py submissions/no_other_cars/team_controller.py
python scripts/validate_submission.py submissions/with_other_cars/team_controller.py
python -m pytest -q
```

The build script concatenates the modules in dependency order and injects the selected profile into a self-contained `team_controller.py`. See the [technical manual](../docs/technical_manual.md) for algorithm details and the [testing guide](../docs/README.md) for SDK and Webots validation.
