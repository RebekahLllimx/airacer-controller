# Documentation Guide

These documents cover the controller architecture, local and official validation, Webots execution, and post-run diagnosis. Start with the document that matches the question you are trying to answer.

| Document | Purpose |
| --- | --- |
| [`technical_manual.md`](technical_manual.md) | Explain how camera frames become road geometry, steering, speed, avoidance, and recovery commands. |
| [`official_testing.md`](official_testing.md) | Configure the official SDK, run its validator, and launch supported Webots worlds. |
| [`human_webots_testing.md`](human_webots_testing.md) | Run solo or multi-car Webots tests and collect the expected output artifacts. |
| [`multicar_extreme_tests.md`](multicar_extreme_tests.md) | Reproduce blocked-road, collision, and recovery scenarios for the traffic profile. |
| [`r052_contribution_scope.md`](r052_contribution_scope.md) | Record the reviewed contribution scope within the five-person team project. |

## Suggested reading paths

### Understand the controller

1. Read the root [`README.md`](../README.md) for the project boundary and build workflow.
2. Read [`technical_manual.md`](technical_manual.md) for the perception-to-control design.
3. Use the [`controller/` module guide](../controller/README.md) while reading the source.

### Reproduce the checks

1. Run the offline test and build commands in the root README.
2. Follow [`official_testing.md`](official_testing.md) for SDK validation.
3. Follow [`human_webots_testing.md`](human_webots_testing.md) for closed-loop simulation.
4. Use the analysis commands in [`human_webots_testing.md`](human_webots_testing.md) to interpret the recorded run.

### Inspect development evidence

The [`experiments/`](../experiments/) directory contains structured historical runs, a consolidated report, regression cases, and selected figures.
