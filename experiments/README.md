# Experiment Archive

This directory stores structured Webots and platform evidence used to evaluate controller changes.

| Path | Content |
| --- | --- |
| `runs.csv` | One row per recorded run: revision, profile, world, completion, timing, collisions, and notes |
| `report.md` | Consolidated design and experiment findings |
| `cases/` | Minimal failure windows retained for regression analysis |
| `figures/` | Selected trajectory, speed, perception, and comparison figures |

Full camera dumps, videos, raw telemetry copies, and generated debug controllers remain in `.tmp/` and are not committed. A case is retained only when it can reproduce or explain a specific failure; a figure is retained only when it supports a durable result or comparison.

`runs.csv`, the consolidated report, source history, and curated evidence provide the public experiment record.
