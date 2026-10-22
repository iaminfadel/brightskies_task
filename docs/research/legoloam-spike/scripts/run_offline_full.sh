#!/bin/bash
set +u
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
OUT=${1:-/data/run_full}
PARAMS=${2:-/spike/loam_carla_full.yaml}
mkdir -p "$OUT"
ros2 run lego_loam_sr lego_loam_sr --ros-args \
  --params-file "$PARAMS" \
  -r /rslidar_points:=/carla/hero/lidar > "$OUT/lego.log" 2>&1 &
LEGO_PID=$!
python3 /spike/map_saver.py "$OUT" > "$OUT/saver.log" 2>&1 &
SAVER_PID=$!
sleep 4
ros2 bag play /data/bag --rate 2 --clock --start-offset 1 --topics /carla/hero/lidar /clock > "$OUT/play.log" 2>&1
sleep 5
kill -INT "$SAVER_PID" 2>/dev/null
sleep 8
kill -INT "$LEGO_PID" 2>/dev/null
sleep 5
kill -9 "$SAVER_PID" "$LEGO_PID" 2>/dev/null
echo done > "$OUT/done"
