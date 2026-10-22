#!/usr/bin/env bash
set -o pipefail

source /opt/brightdrive/setup.bash

EVIDENCE="${EVIDENCE:-/ws/evidence}"
ROOT="${ROOT:-/ws/brightdrive}"
TOWN="${CARLA_TOWN:-Town01}"
LOGS="$EVIDENCE/logs"
PIDS="$EVIDENCE/pids"
mkdir -p "$LOGS" "$PIDS" "$EVIDENCE/bags"

started_epoch=$(date +%s)
fail() {
  echo "VALIDATION_FAILED: $*" >&2
  echo "VALIDATION_FAILED: $*" >>"$EVIDENCE/validation_status.txt"
  exit 1
}

wait_topic() {
  python3 "$ROOT/scripts/validation/wait_graph.py" --topic "$1" --timeout "${2:-60}"
}

wait_publisher() {
  python3 "$ROOT/scripts/validation/wait_graph.py" --topic "$1" --publisher --timeout "${2:-60}"
}

wait_message() {
  local topic="$1" timeout="${2:-60}"
  timeout "$timeout" ros2 topic echo --no-daemon --once "$topic" >/dev/null 2>&1
}

start_bg() {
  local name="$1"; shift
  nohup setsid "$@" >"$LOGS/$name.log" 2>&1 &
  echo $! >"$PIDS/$name.pid"
}

started_bridge=0
started_estimator=0
carla_ready=""

# CARLA RPC readiness.
echo "==> waiting for CARLA RPC"
python3 - "$EVIDENCE" <<'PY'
import json
import sys
import time

import carla

out = sys.argv[1]
started = time.time()
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2.0)
deadline = started + 300
while time.time() < deadline:
    try:
        world = client.get_world()
        break
    except Exception:
        time.sleep(1)
else:
    raise SystemExit("CARLA RPC did not become ready")
elapsed = time.time() - started
with open(f"{out}/carla_rpc_ready.json", "w", encoding="utf-8") as handle:
    json.dump(
        {
            "elapsed_s": round(elapsed, 3),
            "map": world.get_map().name,
            "client_version": client.get_client_version(),
            "server_version": client.get_server_version(),
        },
        handle,
        indent=2,
    )
    handle.write("\n")
print(f"CARLA RPC ready in {elapsed:.1f}s on {world.get_map().name}")
PY
carla_ready=$(date +%s)

# Python API smoke: one RGB frame + one LiDAR scan from the same run.
if [ "${SKIP_SMOKE:-0}" != "1" ]; then
  echo "==> CARLA Python API smoke"
  python3 "$ROOT/scripts/validation/carla_smoke.py" --out "$EVIDENCE/smoke" \
    --town "$TOWN" --timeout 180 >"$LOGS/carla_smoke.log" 2>&1 \
    || fail "CARLA Python API smoke"
fi

# Bridge.
if [ "${SKIP_BRIDGE:-0}" != "1" ]; then
  echo "==> starting bridge"
  start_bg bridge ros2 run carla_ros_bridge bridge --ros-args \
    -p host:=127.0.0.1 -p port:=2000 -p timeout:=60 \
    -p town:=Carla/Maps/"$TOWN" \
    -p passive:=false -p synchronous_mode:=true \
    -p synchronous_mode_wait_for_vehicle_control_command:=false \
    -p fixed_delta_seconds:=0.1 -p register_all_sensors:=true \
    -p ego_vehicle_role_name:=hero \
    || fail "bridge launch"
  wait_publisher /carla/world_info 300 || fail "bridge did not publish /carla/world_info"
  wait_message /clock 60 || fail "bridge did not publish /clock"
else
  echo "==> reusing bridge"
  wait_publisher /carla/world_info 60 || fail "existing bridge not publishing"
fi
started_bridge=$(date +%s)

# Spawn the estimator + visualization rig.
if [ "${SKIP_SPAWN:-0}" != "1" ]; then
  echo "==> spawning hero"
  start_bg spawn ros2 launch carla_spawn_objects carla_spawn_objects.launch.py \
    objects_definition_file:="$ROOT/config/validation/objects.json" \
    || fail "spawn launch"
else
  echo "==> reusing spawn"
fi
for topic in /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /carla/hero/camera_front/image; do
  wait_topic "$topic" 300 || fail "missing $topic"
done
wait_message /carla/hero/lidar 60 || fail "no LiDAR sample"
wait_message /carla/hero/gnss 60 || fail "no GNSS sample"
wait_message /carla/hero/imu 60 || fail "no IMU sample"
wait_message /carla/hero/camera_front/image 60 || fail "no camera sample"

# Estimator stack: KISS-ICP + robot_localization + status stub.
echo "==> starting estimator stack"
if [ "${SKIP_ESTIMATOR:-0}" != "1" ]; then
  start_bg kiss ros2 run kiss_icp kiss_icp_node --ros-args \
    --remap pointcloud_topic:=/carla/hero/lidar \
    -p use_sim_time:=true \
    -p base_frame:=hero/lidar \
    -p lidar_odom_frame:=odom_lidar \
    -p publish_odom_tf:=true \
    -p invert_odom_tf:=false \
    -p publish_debug_clouds:=true \
    --params-file "$ROOT/config/validation/kiss_icp.yaml" \
    || fail "KISS-ICP launch"
  start_bg ekf ros2 run robot_localization ekf_node --ros-args \
    -r __node:=ekf_filter_node \
    -p use_sim_time:=true \
    --params-file "$ROOT/config/validation/ekf.yaml" \
    || fail "EKF launch"
  start_bg status python3 "$ROOT/scripts/validation/localization_status.py" \
    || fail "status launch"
else
  echo "==> reusing estimator"
fi
started_estimator=$(date +%s)
for topic in /kiss/odometry /odometry/filtered /localization/status; do
  wait_topic "$topic" 180 || fail "estimator missing $topic"
done

# Foxglove bridge.
if [ "${SKIP_FOXGLOVE:-0}" != "1" ]; then
  start_bg foxglove ros2 run foxglove_bridge foxglove_bridge --ros-args \
    -p port:=8765 -p address:=127.0.0.1 -p use_sim_time:=true \
    || fail "foxglove launch"
else
  echo "==> reusing foxglove"
fi
foxglove_listening() {
  python3 - <<'PY'
import socket
sock = socket.socket()
sock.settimeout(1.0)
try:
    sock.connect(("127.0.0.1", 8765))
finally:
    sock.close()
PY
}
for _ in $(seq 1 30); do
  if foxglove_listening 2>/dev/null; then break; fi
  sleep 1
done
foxglove_listening || fail "foxglove_bridge did not listen on 8765"

# Move the hero so the odometry and RViz trajectory are non-trivial.
python3 "$ROOT/scripts/validation/drive_hero.py" >"$LOGS/drive_hero.log" 2>&1 \
  || echo "autopilot setup failed; continuing with stationary evidence" >&2

sleep 5

# Topic rates.
for pair in "lidar:/carla/hero/lidar" "gnss:/carla/hero/gnss" "imu:/carla/hero/imu" "clock:/clock"; do
  name="${pair%%:*}"
  topic="${pair#*:}"
  timeout 8 ros2 topic hz "$topic" >"$LOGS/hz_$name.txt" 2>&1 || true
done

# Isolation audit: estimator nodes must not subscribe to the visualization camera.
for node in /kiss_icp_node /ekf_filter_node /localization_status; do
  safe_name="${node#/}"
  ros2 node info --no-daemon "$node" >"$LOGS/node_info_$safe_name.txt" 2>&1 || true
done
if grep -R "/carla/hero/camera_front" "$LOGS"/node_info_*.txt; then
  fail "an estimator node subscribes to the visualization camera"
fi
ros2 topic info --no-daemon -v /carla/hero/camera_front/image >"$LOGS/camera_topic_info.txt" 2>&1 || true

# Bags: estimator bag without camera; visualization bag with camera only.
echo "==> recording bags"
timeout --signal=INT 20 ros2 bag record -o "$EVIDENCE/bags/estimator" \
  /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /clock /tf \
  /kiss/odometry /odometry/filtered /localization/status \
  >"$LOGS/bag_estimator.log" 2>&1 &
estimator_bag_pid=$!
timeout --signal=INT 20 ros2 bag record -o "$EVIDENCE/bags/visualization" \
  /carla/hero/camera_front/image /carla/hero/camera_front/camera_info /clock \
  >"$LOGS/bag_visualization.log" 2>&1 &
visualization_bag_pid=$!
wait "$estimator_bag_pid" 2>/dev/null || true
wait "$visualization_bag_pid" 2>/dev/null || true

# Save one ROS LiDAR scan too.
python3 "$ROOT/scripts/validation/capture_scan.py" \
  --out "$EVIDENCE/lidar_scan_ros.pcd" >"$LOGS/capture_scan.log" 2>&1 \
  || echo "ROS scan capture failed" >&2

# Headless RViz screenshot (compliance fallback path).
echo "==> capturing headless RViz"
bash "$ROOT/scripts/validation/rviz_capture.sh" \
  "$EVIDENCE/rviz_map_trajectory.png" "$LOGS/rviz.log" \
  || echo "RViz capture failed; see logs/rviz.log" >&2

# Summary.
ros2 topic list --no-daemon >"$EVIDENCE/topics.txt" 2>&1 || true
ros2 node list --no-daemon >"$EVIDENCE/nodes.txt" 2>&1 || true
ros2 topic hz /carla/hero/lidar >"$EVIDENCE/hz_lidar_final.txt" 2>&1 &
hz_pid=$!
sleep 6
kill "$hz_pid" 2>/dev/null || true
wait "$hz_pid" 2>/dev/null || true

finished_epoch=$(date +%s)
python3 - "$EVIDENCE" "$started_epoch" "$carla_ready" "$started_bridge" "$started_estimator" "$finished_epoch" <<'PY'
import json
import sys

evidence, started, carla_ready, bridge, estimator, finished = sys.argv[1:]
with open(f"{evidence}/timings.json", "w", encoding="utf-8") as handle:
    json.dump(
        {
            "validation_start_epoch": int(started),
            "carla_rpc_ready_epoch": int(carla_ready),
            "bridge_start_epoch": int(bridge),
            "estimator_start_epoch": int(estimator),
            "validation_end_epoch": int(finished),
            "carla_rpc_from_validation_start_s": int(carla_ready) - int(started),
            "bridge_launch_to_estimator_start_s": int(estimator) - int(bridge),
            "total_validation_s": int(finished) - int(started),
        },
        handle,
        indent=2,
    )
    handle.write("\n")
PY

du -sh "$EVIDENCE"/bags/* >"$EVIDENCE/bag_sizes.txt" 2>&1 || true
date -Is >"$EVIDENCE/READY_FOR_FOXGLOVE"
echo "==> validation ready; stack left running for Foxglove"
