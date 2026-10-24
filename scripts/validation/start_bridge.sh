#!/usr/bin/env bash
# Start the carla_ros_bridge against the running carla-server (Town05 by
# default). Run on the VM inside the brightdrive-deps container, detached:
#   bash scripts/validation/start_bridge.sh [Town05]
set -eo pipefail
TOWN=${1:-Town05}
LOG=${BRIDGE_LOG:-/ws/bridge_${TOWN}.log}

source /opt/brightdrive/setup.bash
# Synchronous, fixed_delta 0.1 s = exactly one 10 Hz lidar rotation per tick:
# every lidar message is a COMPLETE sweep (KISS-ICP's input model), and all
# sensor stamps share the world-tick timeline. The bridge ticks as fast as it
# can (fast-forward); that is fine for mapping runs, where only bag recorders
# consume the topics. Live localization runs need a rate-limited ticker
# instead (separate concern).
nohup setsid ros2 run carla_ros_bridge bridge --ros-args \
  -p host:=127.0.0.1 -p port:=2000 -p timeout:=60 \
  -p town:=Carla/Maps/"$TOWN" \
  -p passive:=false -p synchronous_mode:=true \
  -p synchronous_mode_wait_for_vehicle_control_command:=false \
  -p fixed_delta_seconds:=0.1 -p register_all_sensors:=true \
  -p ego_vehicle_role_name:=hero > "$LOG" 2>&1 &
echo "bridge starting (pid $!, log $LOG); waiting for /clock"

source /ws/install/setup.bash 2>/dev/null || true
for _ in $(seq 1 60); do
  if timeout 5 ros2 topic echo --no-daemon --once /clock >/dev/null 2>&1; then
    echo "BRIDGE_UP town=$TOWN"
    exit 0
  fi
  sleep 2
done
echo "BRIDGE_FAILED: no /clock after 120s; see $LOG" >&2
exit 1
