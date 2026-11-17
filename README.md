# AI Racer Controller

A camera-based controller for the PKU DSA AI Racer course challenge. It reads left and right camera frames, estimates the road, and returns steering and speed for each frame. Development happens in modular Python code; the competition artifact is a generated, self-contained `team_controller.py` file.

![Perception-to-control pipeline](experiments/figures/run3_analysis/fig1_pipeline.png)

## How it works

```text
camera frames -> perception -> track estimate -> driving policy -> steering and speed
```

The public controller interface is:

```python
def control(left_img, right_img, timestamp):
    return steering, speed
```

Steering is bounded to `[-1, 1]`. Normal forward speed is in `[0, 1]`; the local multi-car recovery policy can briefly request negative speed to reverse out of a blockage.

| Profile | Purpose | Generated artifact |
| --- | --- | --- |
| `no_other_cars` | Solo time trials; opponent detection and multi-car recovery are disabled. | [Solo submission](submissions/no_other_cars/team_controller.py) |
| `with_other_cars` | Traffic scenarios; adds opponent avoidance and recovery behavior. | [Multi-car submission](submissions/with_other_cars/team_controller.py) |

[`controller/`](controller/) contains perception, track estimation, opponent detection, policy, parameters, and the shared interface. [`scripts/build_submission.py`](scripts/build_submission.py) assembles a single-file submission for either profile. The profiles share driving code, with explicit gates that keep traffic-specific processing out of the solo path. See the [technical manual](docs/technical_manual.md) for the design.

## Run the offline checks

From the repository root, with Python and the packages in `requirements.txt`:

```bash
python -m pip install -r requirements.txt
python -m pytest -q

python scripts/build_submission.py --mode no_other_cars
python scripts/build_submission.py --mode with_other_cars

python scripts/validate_submission.py submissions/no_other_cars/team_controller.py
python scripts/validate_submission.py submissions/with_other_cars/team_controller.py
```

The build commands regenerate the tracked submission files. The checks cover contracts and local submission rules; they do not measure driving performance. For the separate official SDK validator and a Webots run, follow [SDK setup and testing](docs/official_testing.md). That workflow uses `AIRACER_SDK` to point to an external SDK checkout.

## Evidence and project history

[`experiments/runs.csv`](experiments/runs.csv), the [experiment notes](experiments/notes.md), and the [report](experiments/report.md) record course runs, including multi-car scenarios. These are historical results. A new build should be driven and logged again before making a current performance claim. The [experiment archive guide](experiments/README.md) explains how runs, failure cases, and figures are organized.

This was a **five-person team project**. My work focused on multi-car avoidance and recovery in the shared [perception](controller/perception.py) and [policy](controller/policy.py) pipeline, using prior road-center state to stabilize ambiguous segmentation, resolving a controller-simulator interface mismatch, and building [multi-car launch](scripts/webots_multicar_run.sh) and blocked-road test procedures. I also integrated modules through shared contracts and single-file builds. The commit history and experiment records retain the team's contributions; [the R052 scope note](docs/r052_contribution_scope.md) separates that reviewed work from later team development.

## Where to go next

- [Engineering handoff](experiments/STATUS.md): the team's last recorded status, open questions, and next steps (last updated June 2026).
- [Controller module guide](controller/README.md): source boundaries and data flow.
- [Testing documentation](docs/README.md): official SDK, Webots, and review procedures.
- [Script index](scripts/README.md): build, validation, replay, and analysis commands.
- [Submission guide](submissions/README.md): generated artifacts and submission rules.
- [Experiment archive](experiments/README.md): recorded runs, cases, and figures.
