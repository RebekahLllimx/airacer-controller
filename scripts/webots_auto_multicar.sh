#!/usr/bin/env bash
# Run a bounded unattended multi-car session with log-stall monitoring.
#   bash scripts/webots_auto_multicar.sh <world> <max_sec> [idle_sec] [ncars]
#   bash scripts/webots_auto_multicar.sh complex 300
#
set -uo pipefail

SDK="${AIRACER_SDK:?Set AIRACER_SDK to the official SDK directory}"
WORLD=${1:-complex}
MAX=${2:-300}
IDLE=${3:-25}
NCARS=${4:-6}
OPP_SPEED_SCALE=${5:-1.0}


pkill -f webots 2>/dev/null || true
pkill -f run_local 2>/dev/null || true
sleep 1

rm -rf .tmp/multicar
mkdir -p .tmp/multicar/frames_${WORLD}_car1 .tmp/multicar/webots_console

python scripts/build_submission.py --mode with_other_cars \
  --debug-log ".tmp/multicar/control_${WORLD}_car1.jsonl" \
  --dump-frames ".tmp/multicar/frames_${WORLD}_car1" --dump-frame-stride 10 \
  --out .tmp/multicar/team_controller_car1_debug.py || exit 1
python scripts/build_submission.py --mode with_other_cars \
  --out .tmp/multicar/team_controller_opp.py || exit 1
if [[ "$OPP_SPEED_SCALE" != "1.0" ]]; then
  cat >> .tmp/multicar/team_controller_opp.py <<PYEOF

_orig_control = control
def control(left_img, right_img, timestamp):
    s, v = _orig_control(left_img, right_img, timestamp)
    return s, v * ${OPP_SPEED_SCALE}
PYEOF
fi

export AIRACER_CONTROLLER_CONSOLE_LOG_DIR="$PWD/.tmp/multicar/webots_console"
export AIRACER_CONTACT_LOG=1
export AIRACER_CONTACT_LOG_PATH="$PWD/.tmp/multicar/contact_${WORLD}_car1.jsonl"

REC="$SDK/.local/recordings"
rm -f "$REC/telemetry.jsonl" "$REC/metadata.json" 2>/dev/null || true

CAR1="$PWD/.tmp/multicar/team_controller_car1_debug.py"
OPP="$PWD/.tmp/multicar/team_controller_opp.py"

CAR_ARGS=(--car "${CAR1}:car_1:ours")
for slot in 2 3 4 5 6; do
  [[ "$slot" -le "$NCARS" ]] && CAR_ARGS+=(--car "${OPP}:car_${slot}:opp")
done

echo "[auto] world=$WORLD ncars=$NCARS max=${MAX}s idle=${IDLE}s opp_speed_scale=${OPP_SPEED_SCALE}"
python "$SDK/run_local.py" --world "$WORLD" "${CAR_ARGS[@]}" --skip-validate \
  > .tmp/multicar/webots_launch.log 2>&1 &
RL_PID=$!

LOG=".tmp/multicar/control_${WORLD}_car1.jsonl"
start=$(date +%s); last_size=-1; last_change=$start
while kill -0 "$RL_PID" 2>/dev/null; do
  now=$(date +%s)
  if (( now - start >= MAX )); then echo "[auto] hard cap ${MAX}s reached"; break; fi
  size=$(stat -f%z "$LOG" 2>/dev/null || echo 0)
  if [[ "$size" != "$last_size" ]]; then last_size=$size; last_change=$now; fi
  if (( size > 0 && now - last_change >= IDLE && now - start > 45 )); then
    echo "[auto] control log idle ${IDLE}s (race likely ended)"; break
  fi
  sleep 3
done

pkill -f webots 2>/dev/null || true
pkill -f run_local 2>/dev/null || true
wait "$RL_PID" 2>/dev/null
elapsed=$(( $(date +%s) - start ))
fsize=$(stat -f%z "$LOG" 2>/dev/null || echo 0)
frames=$(wc -l < "$LOG" 2>/dev/null || echo 0)
echo "[auto] DONE world=$WORLD elapsed=${elapsed}s control_log_bytes=$fsize control_frames=$frames"

cp -f "$REC/telemetry.jsonl" ".tmp/multicar/telemetry_${WORLD}.jsonl" 2>/dev/null || true
cp -f "$REC/metadata.json" ".tmp/multicar/metadata_${WORLD}.json" 2>/dev/null || true
if [[ -f ".tmp/multicar/telemetry_${WORLD}.jsonl" ]]; then
  echo "[auto] ----- multi-car KPI -----"
  python scripts/analyze_multicar_kpi.py ".tmp/multicar/telemetry_${WORLD}.jsonl" \
    --metadata ".tmp/multicar/metadata_${WORLD}.json" --team ours || true
fi
