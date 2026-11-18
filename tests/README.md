# Test Suite

Run these commands from the repository root after installing `requirements.txt`. The suite covers the controller contract, generated submissions, perception and estimation, policy state, profile isolation, and diagnostic tools.

```bash
python -m pytest -q
python -m pytest tests/test_lane_line_follow.py
python -m pytest tests/test_policy.py::test_lateral_offset_controls_sign
```

| Test group | Coverage |
| --- | --- |
| Interface and contracts | `control()` inputs, return shape, bounds, resets, and shared dataclasses |
| Submission build | Concatenation, profile injection, forbidden imports and built-ins, and debug-build separation |
| Perception and estimation | Road and lane observations, dropout behavior, geometry, and confidence |
| Policy | Steering, speed, corner-arrival gating, straight-line behavior, recovery, and opponent response |
| Tooling | Telemetry, contact and control-log analysis, plotting, replay, and temporary-world generation |

Add narrow deterministic tests for module-level behavior. Use Webots evidence for closed-loop driving claims rather than encoding simulator outcomes as unit tests.
