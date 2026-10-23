#!/usr/bin/env bash
set -eo pipefail

OUT="${1:?output png}"
LOG="${2:?rviz log}"
CONFIG="${3:?rviz config}"
source /opt/brightdrive/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_QPA_PLATFORM=xcb

SIM_ARGS=()
if [ "${RV_SIM_TIME:-0}" = "1" ]; then
  SIM_ARGS=(--ros-args -p use_sim_time:=true)
fi
ros2 run rviz2 rviz2 -f -d "$CONFIG" "${SIM_ARGS[@]}" >"$LOG" 2>&1 &
RV=$!
sleep "${RV_WAIT:-28}"
import -window root "$OUT"
kill "$RV" 2>/dev/null || true
wait "$RV" 2>/dev/null || true
echo "captured $OUT"
