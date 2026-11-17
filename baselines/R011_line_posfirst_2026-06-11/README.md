# R011 Lane-Position Baseline

- Source commit: `16ae3f3`.
- Controller source: recoverable from the listed commit and the repository history.
- Main change: bounded post-policy lane-position correction with nearby-vehicle detection.
- Basic-world evidence: the vehicle continued normally until the run was stopped at approximately 260 seconds, maintained about 6.1 m/s late in the run, and passed the stationary vehicle group without repeating the earlier collision.
- Complex-world evidence: the controller passed the first-left-turn failure point but later approached a prolonged stop near `x=-10, y=-27`.

This note is retained as an early speed and lane-following comparison, not as the current controller.
