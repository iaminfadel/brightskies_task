#!/bin/bash
# Empty-town Town05 mapping run: teleport, record sensor-only bag, drive one loop.
set -u
BAG=${1:-/ws/bags/map_full}
DRIVE_SECONDS=${2:-600}
LOG=/ws/map_run.log
exec > >(tee -a "$LOG") 2>&1
source /opt/brightdrive/setup.bash
source /ws/install/setup.bash
echo "MAP_RUN_START bag=$BAG $(date -u +%H:%M:%S)"
/ws/clean_stack.sh || true
rm -rf "$BAG"
python3 /ws/brightdrive/scripts/carla_manual_drive.py --teleport --throttle 0 --seconds 1 > /ws/map_teleport.log 2>&1
ros2 bag record -o "$BAG" /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /tf /tf_static /clock > /ws/map_record.log 2>&1 &
REC=$!
sleep 4
python3 -u /ws/brightdrive/scripts/carla_route_drive.py --no-teleport --ignore-lights \
  --speed-kmh 30 --seconds "$DRIVE_SECONDS" > /ws/map_drive.log 2>&1
DRC=$?
sleep 2
kill -INT "$REC" 2>/dev/null
wait "$REC" 2>/dev/null
{
  echo "drive_rc=$DRC"
  du -sh "$BAG"
  ls "$BAG"
} > /ws/map_run.status
echo "MAP_RUN_DONE $(date -u +%H:%M:%S)"
