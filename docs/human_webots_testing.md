# Webots Runbook

Before running these commands, install the repository dependencies, set `AIRACER_SDK`, and configure Webots' Python interpreter as described in [SDK setup](official_testing.md). Run from the repository root. These launch scripts rebuild local debug controllers and rotate previous outputs under `.tmp/`; archive any run you need to retain before starting another.

## Solo runs

```bash
bash scripts/webots_run.sh basic
bash scripts/webots_run.sh complex

# Save every frame around a precise failure window.
bash scripts/webots_run.sh complex --frames 1

# Save frames only in a time window.
bash scripts/webots_run.sh complex --frame-window 410 430

# Disable frame capture when visual review is unnecessary.
bash scripts/webots_run.sh basic --no-frames
```

The script archives the preceding run, builds a debug controller, launches Webots, and writes outputs under `.tmp/run/`. Camera frames are enabled by default so a failure can be inspected without rerunning the entire course.

Typical outputs:

| Artifact | Purpose |
| --- | --- |
| `control_<world>.jsonl` | Per-frame perception, state, mode, and command fields |
| `contact_<world>.jsonl` | Vehicle-to-static-geometry contact episodes |
| `frames_<world>/` | Left and right camera frames |
| `webots_console/*.log` | Controller process stdout and stderr |
| `webots_launch.log` | Outer launcher output |
| SDK `telemetry.jsonl` | Position, speed, lap, and race events |

The launch log is not guaranteed to contain every Webots GUI physics message. Use the structured contact log rather than console text as the primary static-collision signal:

```bash
python scripts/analyze_contact_log.py .tmp/run/contact_complex.jsonl
```

Contact point count, height, duration, and position distinguish sustained barrier contact from brief chassis contact near the start.

## Multi-car runs

Build the traffic profile and assign one generated controller to each active slot:

```bash
python scripts/build_submission.py --mode with_other_cars \
  --out .tmp/multicar/team_controller_ours.py

python "$AIRACER_SDK/run_local.py" \
  --world complex \
  --car "$PWD/.tmp/multicar/team_controller_ours.py:car_1:ours" \
  --car "$PWD/.tmp/multicar/team_controller_ours.py:car_2:opponent" \
  --skip-validate
```

Convenience scripts:

| Script | Scenario |
| --- | --- |
| `webots_day_multicar.sh` | Six vehicles using the current traffic profile |
| `webots_auto_multicar.sh` | Unattended run with timeout and log-stall watchdog |
| `webots_multicar_run.sh` | Two-vehicle interaction test |

`webots_auto_multicar.sh` accepts `<world> <maximum-seconds> <log-stall-seconds> <car-count> [opponent-speed-scale]`. A speed scale below one creates a controlled overtaking scenario.

Multi-car contact logs contain all vehicles. Filter by slot before attributing a collision:

```bash
python scripts/analyze_contact_log.py \
  .tmp/multicar/contact_complex_car1.jsonl --car-slot car_1
```

## What to record

- controller revision and profile;
- SDK revision, world, slot, and opponent configuration;
- whether the lap or race completed;
- lap time, finishing position, and collision count where available;
- the location and time of lane departure, barrier contact, blockage, or recovery;
- representative frames and the matching control-log interval.

## Review sequence

1. Summarize telemetry and contact episodes.
2. Inspect the corresponding control-log interval.
3. Review a small set of frames before, during, and after the event.
4. Generate overlays only for the windows needed to explain perception or geometry.
5. Record the conclusion in structured run evidence before clearing `.tmp`.

Useful commands:

```bash
python scripts/analyze_telemetry.py --no-archive
python scripts/analyze_control_log.py .tmp/run/control_complex.jsonl
python scripts/plot_run.py \
  --telemetry "$AIRACER_SDK/.local/recordings/telemetry.jsonl" \
  --contact-log .tmp/run/contact_complex.jsonl \
  --out .tmp/run/trajectory.png
```

Debug builds may contain local file I/O and must not be uploaded. Rebuild the clean submission files and rerun both repository and SDK validation before formal use.
