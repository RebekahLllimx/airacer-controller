#!/usr/bin/env bash
# Launch a two-vehicle Webots scenario with isolated logs and optional frames.
#   bash scripts/webots_multicar_run.sh basic
#
#   bash scripts/webots_multicar_run.sh complex
#
#   bash scripts/webots_multicar_run.sh basic --no-frames
#
#   bash scripts/webots_multicar_run.sh basic \
#     --slot1 submissions/with_other_cars/team_controller.py:car_1:ours \
#     --slot2 submissions/with_other_cars/team_controller.py:car_2:opp
#
#   .tmp/multicar/control_<world>_car1.jsonl
#   .tmp/multicar/control_<world>_car2.jsonl
#   .tmp/multicar/contact_<world>_car1.jsonl
#   .tmp/multicar/frames_<world>_car1/
#   .tmp/multicar/webots_console/
#
#   bash scripts/webots_multicar_run.sh basic --scenario blocked_front
#   bash scripts/webots_multicar_run.sh basic --scenario rear_rammed
#   bash scripts/webots_multicar_run.sh basic --scenario railing_stuck
#
set -euo pipefail

SDK="${AIRACER_SDK:?Set AIRACER_SDK to the official SDK directory}"
WORLD=${1:?Usage: scripts/webots_multicar_run.sh <basic|complex> [--scenario S] [--no-frames] [--slot1 ...] [--slot2 ...]}
shift || true

DUMP_FRAMES=1
STRIDE=10
CONTACT_LOG=1
SCENARIO=""
SLOT1=""
SLOT2=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-frames)
      DUMP_FRAMES=0
      shift
      ;;
    --no-contact)
      CONTACT_LOG=0
      shift
      ;;
    --scenario)
      SCENARIO="${2:?--scenario requires a name}"
      shift 2
      ;;
    --slot1)
      SLOT1="${2:?--slot1 requires controller_path:slot:team}"
      shift 2
      ;;
    --slot2)
      SLOT2="${2:?--slot2 requires controller_path:slot:team}"
      shift 2
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

pkill -f webots 2>/dev/null || true
pkill -f run_local 2>/dev/null || true
sleep 1

rm -rf .tmp/multicar
mkdir -p .tmp/multicar/frames_${WORLD}_car1
mkdir -p .tmp/multicar/webots_console
echo "team_controller stdout/stderr tee → $PWD/.tmp/multicar/webots_console/"

FRAMES_ARGS=()
if [[ "$DUMP_FRAMES" -eq 1 ]]; then
  FRAMES_ARGS+=(--dump-frames ".tmp/multicar/frames_${WORLD}_car1" --dump-frame-stride "$STRIDE")
fi

python scripts/build_submission.py --mode with_other_cars \
  --debug-log ".tmp/multicar/control_${WORLD}_car1.jsonl" \
  ${FRAMES_ARGS[@]+"${FRAMES_ARGS[@]}"} \
  --out .tmp/multicar/team_controller_car1_debug.py

python scripts/build_submission.py --mode with_other_cars \
  --debug-log ".tmp/multicar/control_${WORLD}_car2.jsonl" \
  --out .tmp/multicar/team_controller_car2_debug.py

if [[ -z "$SLOT1" ]]; then
  SLOT1="$PWD/.tmp/multicar/team_controller_car1_debug.py:car_1:ours"
fi
if [[ -z "$SLOT2" ]]; then
  SLOT2="$PWD/.tmp/multicar/team_controller_car2_debug.py:car_2:opp"
fi

export AIRACER_CONTROLLER_CONSOLE_LOG_DIR="$PWD/.tmp/multicar/webots_console"

if [[ "$CONTACT_LOG" -eq 1 ]]; then
  export AIRACER_CONTACT_LOG=1
  export AIRACER_CONTACT_LOG_PATH="$PWD/.tmp/multicar/contact_${WORLD}_car1.jsonl"
  echo "contact (rail) log → $PWD/.tmp/multicar/contact_${WORLD}_car1.jsonl"
fi

echo "run_local/Webots launch log → $PWD/.tmp/multicar/webots_launch.log"
echo "Scenario: ${SCENARIO:-default}"
echo "car_1: $SLOT1"
echo "car_2: $SLOT2"

python "$SDK/run_local.py" \
  --world "$WORLD" \
  --car "$SLOT1" \
  --car "$SLOT2" \
  --skip-validate 2>&1 | tee .tmp/multicar/webots_launch.log
