# Script Reference

Run examples from the repository root with its Python environment active. Webots launch scripts additionally require the [SDK setup](../docs/official_testing.md).

## Build and validation

| Script | Purpose |
| --- | --- |
| `build_submission.py` | Assemble modular controller source into a self-contained solo or traffic submission |
| `validate_submission.py` | Check interface, output bounds, forbidden imports, file size, and mock execution |

```bash
python scripts/build_submission.py --mode no_other_cars
python scripts/build_submission.py --mode with_other_cars
python scripts/validate_submission.py submissions/no_other_cars/team_controller.py
python -m pytest -q
```

Debug builds may add local control logs and frame capture, but they must be written under `.tmp/` and must not be uploaded.

## Webots execution

| Script | Purpose |
| --- | --- |
| `webots_run.sh` | Solo run with automatic build, frame capture, control log, contact log, and archive rotation |
| `webots_jump_run.sh` | Approximate restart from a historical telemetry pose |
| `make_teleport_world.py` | Generate the temporary world used by a jump run |
| `webots_multicar_run.sh` | Two-vehicle interaction run |
| `webots_day_multicar.sh` | Six-vehicle traffic run |
| `webots_auto_multicar.sh` | Unattended multi-car run with timeout and log-stall watchdog |

## Analysis

| Script | Question answered |
| --- | --- |
| `analyze_telemetry.py` | Where did the vehicle slow, stop, collide, or finish? |
| `analyze_control_log.py` | Which state and policy terms produced the command? |
| `analyze_contact_log.py` | When and where did static-geometry contact occur? |
| `analyze_escape_episodes.py` | Did recovery trigger, reverse, and return to forward driving? |
| `analyze_multicar_kpi.py` | What were finish, rank, collision, blockage, and overtaking outcomes? |
| `plot_run.py` | What do the trajectory, speed, events, and contacts look like together? |
| `analyze_perception_dump.py` | What road, lane, boundary, and obstacle evidence was extracted from selected frames? |
| `replay_offline.py` | How does a new controller revision respond to the same recorded images? |

See the [Webots runbook](../docs/human_webots_testing.md) for complete examples and evidence-handling rules.
