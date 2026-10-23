#!/usr/bin/env bash
set -eo pipefail

OUT="${1:?output png}"
LOG="${2:?rviz log}"
CONFIG="${3:?rviz config}"
source /opt/brightdrive/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb

ros2 run rviz2 rviz2 -d "$CONFIG" >"$LOG" 2>&1 &
RV=$!
sleep "${RV_WAIT:-28}"
import -window root "$OUT"
kill "$RV" 2>/dev/null || true
wait "$RV" 2>/dev/null || true
echo "captured $OUT"
