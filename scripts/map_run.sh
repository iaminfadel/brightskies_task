#!/bin/bash
# Empty-town Town05 mapping run: respawn hero with the mapping sensor set
# (adds pseudo GT odometry), teleport, record the sensor-only estimator bag
# plus a separate GT eval bag, drive one loop.
set +u
BAG=${1:-/ws/bags/map_full}
DRIVE_SECONDS=${2:-600}
EVAL_BAG=${3:-${BAG}_eval}
LOG=/ws/map_run.log
exec > >(tee -a "$LOG") 2>&1
source /opt/brightdrive/setup.bash
source /ws/install/setup.bash

# Detached runs outlive the SSH session; keep the idle-stop guard satisfied.
heartbeat_loop() {
  while :; do
    sudo mkdir -p /var/lib/brightskies 2>/dev/null || true
    sudo touch /var/lib/brightskies/heartbeat 2>/dev/null || true
    sleep 120
  done
}
heartbeat_loop &
HEARTBEAT_PID=$!

echo "MAP_RUN_START bag=$BAG eval_bag=$EVAL_BAG $(date -u +%H:%M:%S)"
/ws/clean_stack.sh || true
rm -rf "$BAG" "$EVAL_BAG"

# Destroy any existing hero so the respawn can attach the GT odometry sensor.
python3 - <<'PYEOF' > /ws/map_destroy.log 2>&1
import carla
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(10.0)
world = client.get_world()
destroyed = []
for _ in range(10):
    for actor in world.get_actors():
        if actor.attributes.get("role_name") == "hero":
            actor.destroy()
            destroyed.append(actor.id)
    if destroyed:
        break
    world.wait_for_tick(2.0)
print("destroyed:", destroyed)
PYEOF

# Respawn with the mapping sensor set; /carla/hero/odometry is ground truth
# (evaluation path only, recorded into the eval bag, never the estimator bag).
nohup ros2 launch carla_spawn_objects carla_spawn_objects.launch.py \
  objects_definition_file:=/ws/brightdrive/config/validation/objects_map.json \
  > /ws/map_spawn.log 2>&1 &
SPAWN_PID=$!
for _ in $(seq 1 60); do
  ros2 topic list 2>/dev/null | grep -q "^/carla/hero/odometry$" && break
  sleep 2
done
ros2 topic list 2>/dev/null | grep -q "^/carla/hero/odometry$" || {
  echo "MAP_RUN_FAILED: no /carla/hero/odometry after spawn; see /ws/map_spawn.log"
  kill "$SPAWN_PID" "$HEARTBEAT_PID" 2>/dev/null
  exit 1
}

python3 /ws/brightdrive/scripts/carla_manual_drive.py --teleport --throttle 0 --seconds 1 > /ws/map_teleport.log 2>&1
ros2 bag record -o "$BAG" /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /tf /tf_static /clock > /ws/map_record.log 2>&1 &
REC=$!
ros2 bag record -o "$EVAL_BAG" /carla/hero/odometry /clock > /ws/map_record_eval.log 2>&1 &
REC_EVAL=$!
sleep 4
python3 -u /ws/brightdrive/scripts/carla_route_drive.py --no-teleport --ignore-lights \
  --speed-kmh 30 --seconds "$DRIVE_SECONDS" > /ws/map_drive.log 2>&1
DRC=$?
sleep 2
kill -INT "$REC" "$REC_EVAL" 2>/dev/null
wait "$REC" 2>/dev/null
wait "$REC_EVAL" 2>/dev/null
kill "$SPAWN_PID" "$HEARTBEAT_PID" 2>/dev/null
{
  echo "drive_rc=$DRC"
  du -sh "$BAG" "$EVAL_BAG"
  ls "$BAG"
} > /ws/map_run.status 2>/dev/null || true
echo "MAP_RUN_DONE $(date -u +%H:%M:%S)"
exit "$DRC"
