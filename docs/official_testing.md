# Official SDK and Webots Testing

Run the commands from the controller repository root, with the Python 3.10 environment described in the root README active. Offline checks need the repository dependencies; simulator runs additionally need Webots and a separate SDK checkout. The recorded R071 compatibility run used Webots R2025a and SDK revision `cfdeedb`, including the local asset/checkpoint adjustments described below.

The official SDK is kept in a separate checkout. Point `AIRACER_SDK` to its `sdk/` directory:

```bash
git clone https://github.com/pkulab409/pkudsa.airacer.git ../pkudsa.airacer
export AIRACER_SDK="$(cd ../pkudsa.airacer/sdk && pwd)"
python "$AIRACER_SDK/check_env.py"
```

If the SDK is already installed, set the variable to that checkout's `sdk/` directory. It must contain `run_local.py`, `validate_controller.py`, and `rules.yaml`. Record its Git revision with each run; a fresh default-branch clone is not a pinned reproduction of R071.

## 1. Install repository dependencies

```bash
python -m pip install -r requirements.txt
python -m pytest -q
```

## 2. Build both submission profiles

```bash
python scripts/build_submission.py --mode no_other_cars
python scripts/build_submission.py --mode with_other_cars
```

## 3. Run repository validation

```bash
python scripts/validate_submission.py submissions/no_other_cars/team_controller.py
python scripts/validate_submission.py submissions/with_other_cars/team_controller.py
```

These checks cover syntax, the `control()` interface, output bounds, file size, forbidden imports and built-ins, and local mock calls.

## 4. Run the official validator

```bash
python "$AIRACER_SDK/validate_controller.py" \
  --code-path submissions/no_other_cars/team_controller.py \
  --rules "$AIRACER_SDK/rules.yaml"

python "$AIRACER_SDK/validate_controller.py" \
  --code-path submissions/with_other_cars/team_controller.py \
  --rules "$AIRACER_SDK/rules.yaml"
```

The validator does not launch Webots. It confirms that the generated artifact satisfies the platform contract.

## 5. Validate through `run_local`

```bash
python "$AIRACER_SDK/run_local.py" \
  --code-path "$PWD/submissions/no_other_cars/team_controller.py" \
  --validate-only
```

## 6. Run Webots

Before launching a world, set **Webots > Settings > General > Python command** to the same interpreter used to install `numpy` and `opencv-python`. A generic `python3` entry on macOS may resolve to the system interpreter even when the SDK launcher runs from a virtual environment. In that case the Webots car controller exits before loading the submission, typically with `ModuleNotFoundError: cv2`.

Keep the Webots and SDK revisions together in the run record. A world that opens with PROTO field errors or missing SDK assets is not a valid controller result; resolve the SDK/Webots compatibility issue before reporting lap or collision metrics.

The SDK revision used for the November 2026 compatibility run referenced Webots assets that were no longer available through its bundled paths and shipped basic-world checkpoint coordinates that did not match the included oval. The run therefore used an isolated SDK worktree, exact Cyberbotics R2025a assets, absolute local asset paths, and checkpoint coordinates measured from `track_basic.wbt`. Keep these compatibility changes outside the controller repository and record them with the result; do not silently treat a modified local world as an unchanged platform environment.

List the worlds and slots supported by the installed SDK:

```bash
python "$AIRACER_SDK/run_local.py" --list-worlds
```

Solo examples:

```bash
python "$AIRACER_SDK/run_local.py" \
  --code-path "$PWD/submissions/no_other_cars/team_controller.py" \
  --world basic --car-slot car_1

python "$AIRACER_SDK/run_local.py" \
  --code-path "$PWD/submissions/no_other_cars/team_controller.py" \
  --world complex --car-slot car_1
```

Multi-car example:

```bash
python "$AIRACER_SDK/run_local.py" \
  --world complex \
  --car "$PWD/submissions/with_other_cars/team_controller.py:car_1:ours" \
  --car "$PWD/submissions/with_other_cars/team_controller.py:car_2:opponent"
```

Unspecified slots remain stationary and therefore behave as obstacles. `--car` and `--code-path` are alternative interfaces and should not be used together.

## 7. Interpret results

Record the generated controller revision, profile, SDK revision, world, slot configuration, lap status, timing, and collision events. Webots results are configuration-specific; do not combine runs from different worlds or traffic layouts into one performance claim.

For automated logs, frame capture, and contact analysis, continue with the [Webots runbook](human_webots_testing.md).
