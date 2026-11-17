# Validated Controller Baselines

`baselines/` stores concise evidence notes for controller milestones that were exercised in Webots and remain useful for comparison. A baseline is not created for every experiment.

| Baseline | Purpose |
| --- | --- |
| `R049_turn_in_speed_best_2026-06-13/` | Strong solo-driving reference after corner-arrival gating and speed improvements |
| `R038_phase22_best_human_2026-06-12/` | Earlier conservative reference with residual tight-radius behavior |
| `R011_line_posfirst_2026-06-11/` | Early lane-position reference for basic-track comparisons |
| `webots_basic_physical_finish_2026-06-10/` | Initial physical completion snapshot on the basic world |

Each directory contains an evidence note, and some contain the parameter record available for that run. Generated single-file controller snapshots are intentionally omitted because they duplicate thousands of lines of source and preserve obsolete implementation details. Git history is the authoritative source for reconstructing the corresponding modular implementation.
