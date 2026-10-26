#!/usr/bin/env bash
set -eo pipefail
source /opt/brightdrive/setup.bash
source /ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
RATE="${1:-0.5}"
OUT=/ws/evidence/probe
mkdir -p "$OUT"
pkill -f 'localization_node|kiss_icp_node|ekf_node|clock_relay|map_cloud' 2>/dev/null || true
sleep 1
ros2 launch brightdrive_bringup localization.launch.py >"$OUT/launch.log" 2>&1 &
LP=$!
python3 /ws/brightdrive/scripts/reliable_clock_relay.py >"$OUT/relay.log" 2>&1 &
RP=$!
python3 /ws/brightdrive/scripts/publish_map_cloud.py --map-dir /ws/maps/current/town05-v14-pg >"$OUT/cloud.log" 2>&1 &
CP=$!
trap 'kill $LP $RP $CP ${BP:-} 2>/dev/null || true' EXIT INT TERM
sleep 10
ros2 bag play /ws/bags/map_full2 --rate "$RATE" >"$OUT/play.log" 2>&1 &
BP=$!
for i in $(seq 1 14); do
  sleep 7
  {
    echo "--- wall $(date -u +%H:%M:%S) #$i"
    timeout 4 ros2 topic echo --once /localization/status 2>/dev/null || echo no-status
  } >>"$OUT/status.log"
done
grep -E "INIT_FAILED|LOCKED|HEALTH|SCAN_REJECT" "$OUT/launch.log" | tail -20
