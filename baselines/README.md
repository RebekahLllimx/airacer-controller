# Validated Controller Baselines

`baselines/` stores generated controllers that were exercised in Webots and remain useful for comparison or rollback. A baseline is not created for every experiment.

| Baseline | Purpose |
| --- | --- |
| `R049_turn_in_speed_best_2026-06-13/` | Strong solo-driving reference after corner-arrival gating and speed improvements |
| `R038_phase22_best_human_2026-06-12/` | Earlier conservative reference with residual tight-radius behavior |
| `R011_line_posfirst_2026-06-11/` | Early lane-position reference for basic-track comparisons |
| `webots_basic_physical_finish_2026-06-10/` | Initial physical completion snapshot on the basic world |

Each directory contains a self-contained controller and a short evidence note. Git history remains the authoritative source for the modular implementation.
