#!/usr/bin/env bash
# Launch a six-vehicle traffic run for avoidance and recovery validation.
#   bash scripts/webots_day_multicar.sh complex
#   bash scripts/webots_day_multicar.sh basic
#
#
#   python scripts/analyze_control_log.py .tmp/multicar/control_complex_car1.jsonl
#   python scripts/analyze_contact_log.py .tmp/multicar/contact_complex_car1.jsonl
set -euo pipefail

SDK="${AIRACER_SDK:?Set AIRACER_SDK to the official SDK directory}"
WORLD=${1:-complex}
shift || true

DUMP_FRAMES=1
STRIDE=10
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-frames) DUMP_FRAMES=0; shift ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done

pkill -f webots 2>/dev/null || true
pkill -f run_local 2>/dev/null || true
sleep 1

rm -rf .tmp/multicar
mkdir -p .tmp/multicar/frames_${WORLD}_car1
mkdir -p .tmp/multicar/webots_console

FRAMES_ARGS=()
if [[ "$DUMP_FRAMES" -eq 1 ]]; then
  FRAMES_ARGS+=(--dump-frames ".tmp/multicar/frames_${WORLD}_car1" --dump-frame-stride "$STRIDE")
fi
python scripts/build_submission.py --mode with_other_cars \
  --debug-log ".tmp/multicar/control_${WORLD}_car1.jsonl" \
  ${FRAMES_ARGS[@]+"${FRAMES_ARGS[@]}"} \
  --out .tmp/multicar/team_controller_car1_debug.py

python scripts/build_submission.py --mode with_other_cars \
  --out .tmp/multicar/team_controller_opp.py

export AIRACER_CONTROLLER_CONSOLE_LOG_DIR="$PWD/.tmp/multicar/webots_console"
export AIRACER_CONTACT_LOG=1
export AIRACER_CONTACT_LOG_PATH="$PWD/.tmp/multicar/contact_${WORLD}_car1.jsonl"

CAR1="$PWD/.tmp/multicar/team_controller_car1_debug.py"
OPP="$PWD/.tmp/multicar/team_controller_opp.py"

echo "world=$WORLD  6-car  frames=$DUMP_FRAMES"
echo "control log → .tmp/multicar/control_${WORLD}_car1.jsonl"
echo "contact log → .tmp/multicar/contact_${WORLD}_car1.jsonl"

python "$SDK/run_local.py" \
  --world "$WORLD" \
  --car "${CAR1}:car_1:ours" \
  --car "${OPP}:car_2:opp" \
  --car "${OPP}:car_3:opp" \
  --car "${OPP}:car_4:opp" \
  --car "${OPP}:car_5:opp" \
  --car "${OPP}:car_6:opp" \
  --skip-validate 2>&1 | tee .tmp/multicar/webots_launch.log
