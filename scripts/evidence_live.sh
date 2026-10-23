#!/usr/bin/env bash
# Replay the mapping bag against the saved map and capture RViz evidence.
set -euo pipefail

OUT="${1:-/ws/evidence/live}"
RATE="${2:-1.2}"
mkdir -p "$OUT"
source /opt/brightdrive/setup.bash
source /ws/install/setup.bash

pkill -f 'localization_node|kiss_icp_node|ekf_node|robot_state_publisher' 2>/dev/null || true
sleep 2

ros2 launch brightdrive_bringup localization.launch.py >"$OUT/localization.log" 2>&1 &
LPID=$!
python3 /ws/brightdrive/scripts/publish_map_cloud.py \
  --map-dir /ws/maps/current/town05-v2 >"$OUT/map_cloud.log" 2>&1 &
CPID=$!
trap 'kill $LPID $CPID ${BPID:-} 2>/dev/null || true' EXIT INT TERM

sleep 18
ros2 bag play /ws/bags/map_full2 --clock --rate "$RATE" >"$OUT/playback.log" 2>&1 &
BPID=$!

sleep 45
timeout 8 ros2 topic echo --once /localization/status >"$OUT/status_overview.txt" 2>&1 || true
timeout 8 ros2 topic echo --once /localization/scan_quality >"$OUT/scan_quality.txt" 2>&1 || true
bash /ws/brightdrive/scripts/validation/rviz_shot.sh \
  "$OUT/map_overview.png" "$OUT/rviz_overview.log" \
  /ws/brightdrive/config/evidence/map_overview.rviz
sleep 3
RV_WAIT=30 bash /ws/brightdrive/scripts/validation/rviz_shot.sh \
  "$OUT/map_local.png" "$OUT/rviz_local.log" \
  /ws/brightdrive/config/evidence/map_local.rviz
timeout 8 ros2 topic echo --once /localization/status >"$OUT/status_local.txt" 2>&1 || true
ros2 topic list >"$OUT/topics.txt" 2>&1 || true
echo "EVIDENCE_LIVE_DONE $(date -u +%FT%TZ)"
