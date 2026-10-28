#!/usr/bin/env bash
# CARLA-free estimator replay: play a recorded sensor bag through the
# estimator route (localization-replay + map-server) and record what it
# publishes. The bag-fed variant of compose_smoke.sh -- same estimator bag
# topic list, no bridge, no CARLA, no GPU.
#
# Only the sensor surface is played back (lidar, GNSS, IMU, clock): the
# recorded /tf is not part of the estimator input contract and would leak
# the original drive's frame tree into the replay.
set -euo pipefail

BAG="${REPLAY_BAG:?REPLAY_BAG must name a recorded sensor bag}"
EVIDENCE="${EVIDENCE:-/ws/evidence/replay}"
LOGS="$EVIDENCE/logs"
mkdir -p "$LOGS"

meta="$BAG/metadata.yaml"
[ -f "$meta" ] || { echo "no metadata.yaml under $BAG" >&2; exit 1; }

# Fail fast on a bag missing the sensor surface instead of running silent.
for topic in /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /clock; do
  grep -q "  $topic:" "$meta" || { echo "bag lacks $topic" >&2; exit 1; }
done
duration=$(python3 -c "
import sys, yaml
info = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
print(int(info['duration']['nanoseconds'] / 1e9) + 1)
" "$meta")

rm -rf "$EVIDENCE/bags/estimator"
echo "==> recording estimator surfaces for ${duration}s of bag time"
ros2 bag record -o "$EVIDENCE/bags/estimator" \
  /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /clock /tf /tf_static \
  /kiss/odometry /odometry/filtered /localization/status \
  /localization/scan_pose /localization/gnss_pose /localization/imu \
  >"$LOGS/bag_estimator.log" 2>&1 &
rec_pid=$!

echo "==> playing $BAG"
timeout --signal=INT "$((duration + 120))" \
  ros2 bag play "$BAG" --clock 100 \
  --topics /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /clock \
  >"$LOGS/bag_play.log" 2>&1 || echo "bag play ended early" >&2

sleep 10  # let the estimator flush its tail into the recording
kill -INT "$rec_pid" 2>/dev/null || true
wait "$rec_pid" 2>/dev/null || true

# A replay exists to exercise the estimator: the recording must carry the
# fused output and every yaw source, not just the played-back sensors.
ros2 bag info "$EVIDENCE/bags/estimator" >"$LOGS/bag_info.txt" 2>&1 || true
for topic in /odometry/filtered /localization/scan_pose /kiss/odometry \
             /localization/gnss_pose /localization/imu; do
  grep -q "$topic" "$LOGS/bag_info.txt" || {
    echo "REPLAY_FAILED: estimator published no $topic" \
      | tee "$EVIDENCE/validation_status.txt" >&2
    exit 1
  }
done
echo "REPLAY_OK (${duration}s of bag time); evidence: $EVIDENCE" \
  | tee "$EVIDENCE/validation_status.txt"
