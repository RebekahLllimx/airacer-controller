# Generated Submissions

`submissions/` contains the self-contained controller files produced by `scripts/build_submission.py`. Edit the modular source under `controller/`, then rebuild these artifacts. The commands below run from the repository root, not from this directory, and use the environment described in the root README.

| Directory | Profile | Scenario |
| --- | --- | --- |
| `no_other_cars/` | `no_other_cars` | Solo time trial; opponent processing and traffic recovery disabled |
| `with_other_cars/` | `with_other_cars` | Traffic scenario; opponent avoidance and blockage recovery enabled |

```bash
python scripts/build_submission.py --mode no_other_cars
python scripts/build_submission.py --mode with_other_cars

python scripts/validate_submission.py submissions/no_other_cars/team_controller.py
python scripts/validate_submission.py submissions/with_other_cars/team_controller.py
python -m pytest -q
```

Run the official SDK validator described in `docs/official_testing.md` before formal use. Debug builds containing file I/O belong under `.tmp/`, not in this directory.
