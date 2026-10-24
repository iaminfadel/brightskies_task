#!/bin/bash
# LeGO-LOAM offline runner (inside the legoloam:mm image).
#
# Plays the full-sweep bag into the patched port, records everything, and
# stops cleanly.  usage:
#   lego_offline.sh <sweeps_bag> <out_dir> [imu_damping] [ground_z_hold]
# No `set -u`: ROS setup scripts reference unbound vars.
set -o pipefail
BAG=${1:?sweeps bag}
OUT=${2:?out dir}
IMU_DAMP=${3:-true}
Z_HOLD=${4:-false}

source /opt/lego/setup.bash
mkdir -p "$OUT"

ros2 run lego_loam_sr lego_loam_sr --ros-args \
  --params-file /opt/lego/loam_carla.yaml \
  -p imu_damping:="$IMU_DAMP" \
  -p ground_z_hold:="$Z_HOLD" \
  -r /rslidar_points:=/carla/hero/lidar \
  > "$OUT/lego.log" 2>&1 &
LEGO_PID=$!
sleep 3

if ! kill -0 "$LEGO_PID" 2>/dev/null; then
  echo "LEGO_RUN_FAILED: lego_loam_sr died at startup; see $OUT/lego.log"
  cat "$OUT/lego.log"
  exit 1
fi

ros2 bag record -a -o "$OUT/lego_out" > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 2

ros2 bag play "$BAG" --clock --rate 2 > "$OUT/play.log" 2>&1
PLAY_RC=$?
sleep 8  # let the mapping thread flush the tail

kill -INT "$REC_PID" "$LEGO_PID" 2>/dev/null
sleep 3
kill "$REC_PID" "$LEGO_PID" 2>/dev/null
wait "$REC_PID" 2>/dev/null
wait "$LEGO_PID" 2>/dev/null

echo "play_rc=$PLAY_RC" > "$OUT/status.txt"
echo "LEGO_RUN_DONE play_rc=$PLAY_RC out=$OUT"
