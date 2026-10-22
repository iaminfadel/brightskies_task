#!/usr/bin/env bash
set -eo pipefail

OUT="${1:?output png}"
LOG="${2:?rviz log}"
source /opt/brightdrive/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb

ros2 run rviz2 rviz2 -d /ws/brightdrive/config/validation/kiss_icp.rviz >"$LOG" 2>&1 &
RV=$!
sleep 25
import -window root "$OUT"
kill "$RV" 2>/dev/null || true
wait "$RV" 2>/dev/null || true
