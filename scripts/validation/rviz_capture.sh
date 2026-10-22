#!/usr/bin/env bash
set -euo pipefail

OUT="${1:-/ws/evidence/rviz_map_trajectory.png}"
LOG="${2:-/ws/evidence/logs/rviz.log}"
mkdir -p "$(dirname "$OUT")" "$(dirname "$LOG")"

xvfb-run -a -s "-screen 0 1600x900x24 +extension GLX +render -noreset" \
  bash /ws/brightdrive/scripts/validation/rviz_capture_inner.sh "$OUT" "$LOG"
