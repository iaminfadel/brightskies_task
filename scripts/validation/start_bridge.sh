#!/usr/bin/env bash
# Start the carla_ros_bridge against the running carla-server (Town05 by
# default). Run on the VM inside the brightdrive-deps container, detached:
#   bash scripts/validation/start_bridge.sh [Town05]
set -eo pipefail
TOWN=${1:-Town05}
LOG=${BRIDGE_LOG:-/ws/bridge_${TOWN}.log}

source /opt/brightdrive/setup.bash
# Async bridge + the server engine capped at -benchmark -fps=10 (see
# up_carla.sh): the engine paces itself at real-time 10 fps, sim time
# advances 0.1 s per frame, and the 10 Hz lidar rotation completes exactly one
# revolution per frame -- every lidar message is a COMPLETE sweep at a
# deterministic 10 Hz. The bridge never calls world.tick() in async mode, so
# the synchronous-mode tick deadlock cannot occur, and a real-time factor of
# 1 keeps the live localization stack from sim-time starvation later.
nohup setsid ros2 run carla_ros_bridge bridge --ros-args \
  -p host:=127.0.0.1 -p port:=2000 -p timeout:=60 \
  -p town:=Carla/Maps/"$TOWN" \
  -p passive:=false -p synchronous_mode:=false \
  -p register_all_sensors:=true \
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
