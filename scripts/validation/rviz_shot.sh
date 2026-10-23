#!/usr/bin/env bash
set -euo pipefail

OUT="${1:?output png}"
LOG="${2:?rviz log}"
CONFIG="${3:?rviz config}"
mkdir -p "$(dirname "$OUT")" "$(dirname "$LOG")"

xvfb-run -a -s "-screen 0 1600x900x24 +extension GLX +render -noreset" \
  bash /ws/brightdrive/scripts/validation/rviz_shot_inner.sh "$OUT" "$LOG" "$CONFIG"
