# AI Racer Controller: Design and Experimental Findings

## Summary

The project implements a vision-only racing controller that converts stereo camera frames into steering and speed without map, pose, wheel-speed, or opponent telemetry. Its central engineering result is a modular perception-to-control pipeline supported by reproducible builds, contract tests, Webots runs, structured contact logs, and frame-level diagnosis.

The solo controller completed the official complex-world evaluation with a recorded total time of approximately `252.9 s` and no major collision in the matching metadata. In the recorded six-vehicle complex-world tests, the traffic profile finished second with no disqualification or serious collision. These values belong to the documented SDK and world revisions and should not be generalized to a different simulator configuration.

## Architecture

The controller separates road perception, geometric estimation, driving policy, and output bounds. Scan-line corridor tracking and lane-marking detection produce a compact observation; a trusted geometric fit converts pixels into lateral, heading, look-ahead, curvature, and boundary state; the policy applies mode-dependent steering, speed, and recovery behavior.

Two generated profiles share the driving core. `no_other_cars` bypasses opponent processing for solo timing. `with_other_cars` adds nearby-vehicle detection, directional avoidance, visual-motion blockage detection, and bounded reverse recovery. Profile-isolation tests prevent traffic behavior from leaking into solo driving.

## Corner-radius investigation

Early versions steered toward distant curvature before the vehicle physically reached a corner. This produced systematic inside cutting: a far-field heading signal became large during the approach even while the near-field road centre remained straight. The final design gates far-field steering with near-field lateral arrival, compensates the gate for speed, latches it through the corner, and decays it after exit.

This change addressed the mechanism rather than adding a fixed steering correction. Later speed tuning raised medium-corner speed while preserving the sharp-corner limit. The R049 baseline records the resulting solo-driving behavior and remains the principal rollback reference.

## Multi-car investigation

Traffic tests exposed three distinct failure modes: entering a stationary vehicle cluster too quickly, remaining pinned while the policy still believed it was moving, and treating low-saturation asphalt as a dark vehicle. The maintained traffic profile addresses them with earlier directional avoidance, frame-motion blockage detection, bounded recovery phases, and tighter vehicle-colour gating.

Multi-car contact logs must be filtered by vehicle slot before a collision is attributed. This correction matters because an opponent contacting a barrier can otherwise look like an ego-vehicle event in a shared supervisor log.

## Validation workflow

The repository uses several evidence layers:

1. unit and contract tests for deterministic logic;
2. generated-file validation for platform constraints;
3. official SDK validation for interface and timing checks;
4. Webots runs for closed-loop behavior;
5. telemetry, contact episodes, control logs, and selected frames for mechanism analysis.

The workflow separates software correctness from driving evidence. A passing test suite does not establish lap completion, and a successful lap does not prove that interface, isolation, or state-reset contracts remain correct.

## Limitations

- The perception system uses calibrated colour and geometric rules rather than a learned model; lighting or asset changes may require recalibration.
- Camera-only motion is an indirect proxy for blockage.
- Reverse recovery depends on simulator support and may degrade to a stopped command on a platform that clamps negative speed.
- Multi-car strategy remains reactive and does not estimate relative velocity or plan a race-level defensive line.
- Recorded performance is specific to the SDK, world, starting layout, and controller revision listed in the experiment evidence.
