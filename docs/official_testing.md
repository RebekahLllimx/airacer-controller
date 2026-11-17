# Official SDK and Webots Testing

The official SDK is kept in a separate checkout. Point `AIRACER_SDK` to its `sdk/` directory:

```bash
git clone https://github.com/pkudsa-ai-racer/pkudsa.airacer.git ../pkudsa.airacer
export AIRACER_SDK="$(cd ../pkudsa.airacer/sdk && pwd)"
python "$AIRACER_SDK/check_env.py"
```

If the SDK is already installed, only the environment variable is required. Keep the SDK revision with any recorded run because worlds, validators, and competition rules can change.

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
