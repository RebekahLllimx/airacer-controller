#!/usr/bin/env bash
# Build a debug controller, rotate prior artifacts, and launch a solo Webots run.
# Camera frames and structured logs remain under the ignored .tmp directory.
set -euo pipefail

SDK="${AIRACER_SDK:?Set AIRACER_SDK to the official SDK directory}"
ARCHIVE_KEEP=10
WORLD=${1:?Usage: scripts/webots_run.sh <basic|complex> [--frames N] [--frame-window S E] [--no-frames]}
shift || true

DUMP_FRAMES=1
STRIDE=10
CONTACT_LOG=1

WINDOW_ARGS=()
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
    --frames)
      STRIDE=${2:?--frames requires a stride, for example --frames 1}
      DUMP_FRAMES=1
      shift 2
      ;;
    --frame-window)
      START=${2:?--frame-window requires a start time}
      END=${3:?--frame-window requires an end time}
      WINDOW_ARGS+=(--dump-frame-start "$START" --dump-frame-end "$END")
      DUMP_FRAMES=1
      shift 3
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

archive_path() {
  local src="$1"
  local prefix="$2"
  local ext="${3:-}"
  if [[ ! -e "$src" ]]; then
    return
  fi
  if should_discard_archive_source "$src" "$prefix"; then
    rm -rf "$src"
    echo "discarded empty $prefix archive source → $src"
    return
  fi
  mkdir -p .tmp/run.archive
  local ts
  ts=$(date +%Y%m%d_%H%M%S)
  local dest=".tmp/run.archive/${prefix}_${ts}${ext}"
  local idx=1
  while [[ -e "$dest" ]]; do
    dest=".tmp/run.archive/${prefix}_${ts}_${idx}${ext}"
    idx=$((idx + 1))
  done
  mv "$src" "$dest"
  echo "archived previous $prefix → $PWD/$dest"
  prune_archives ".tmp/run.archive/${prefix}_*"
}

should_discard_archive_source() {
  local src="$1"
  local prefix="$2"
  if [[ -d "$src" ]]; then
    case "$prefix" in
      run_*)
        if find "$src" -type f \( -name 'control_*.jsonl' -o -name 'contact_*.jsonl' -o -name '*.png' \) -size +0 -print -quit | grep -q .; then
          return 1
        fi
        return 0
        ;;
    esac
    return 1
  fi
  if [[ ! -s "$src" ]]; then
    return 0
  fi
  return 1
}

prune_archives() {
  local pattern="$1"
  local archived=()
  local item
  while IFS= read -r item; do
    archived+=("$item")
  done < <(compgen -G "$pattern" | sort -r)
  if (( ${#archived[@]} <= ARCHIVE_KEEP )); then
    return
  fi
  for item in "${archived[@]:ARCHIVE_KEEP}"; do
    rm -rf "$item"
    echo "deleted old archive → $PWD/$item"
  done
}

FRAMES_ARGS=()
if [[ "$DUMP_FRAMES" -eq 1 ]]; then
  FRAMES_ARGS+=(--dump-frames ".tmp/run/frames_${WORLD}" --dump-frame-stride "$STRIDE")
  FRAMES_ARGS+=(${WINDOW_ARGS[@]+"${WINDOW_ARGS[@]}"})
fi

pkill -f webots 2>/dev/null || true
pkill -f run_local 2>/dev/null || true
sleep 1
archive_path "$SDK/.local/recordings/telemetry.jsonl" "telemetry_${WORLD}" ".jsonl"

archive_path .tmp/run "run_${WORLD}"
mkdir -p .tmp/run
mkdir -p .tmp/run/webots_console
echo "team_controller stdout/stderr tee → $PWD/.tmp/run/webots_console/*.log"

python scripts/build_submission.py --mode no_other_cars \
  --debug-log ".tmp/run/control_${WORLD}.jsonl" \
  ${FRAMES_ARGS[@]+"${FRAMES_ARGS[@]}"} \
  --out .tmp/run/team_controller_debug.py

export AIRACER_CONTROLLER_CONSOLE_LOG_DIR="$PWD/.tmp/run/webots_console"
if [[ "$CONTACT_LOG" -eq 1 ]]; then
  export AIRACER_CONTACT_LOG=1
  export AIRACER_CONTACT_LOG_PATH="$PWD/.tmp/run/contact_${WORLD}.jsonl"
  echo "contact (rail) log → $PWD/.tmp/run/contact_${WORLD}.jsonl"
else
  unset AIRACER_CONTACT_LOG
fi
echo "run_local/Webots launch log → $PWD/.tmp/run/webots_launch.log"
python "$SDK/run_local.py" \
  --code-path "$PWD/.tmp/run/team_controller_debug.py" \
  --world "$WORLD" --car-slot car_1 --skip-validate 2>&1 | tee .tmp/run/webots_launch.log
