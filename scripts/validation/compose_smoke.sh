#!/usr/bin/env bash
# One-shot smoke driver over the compose runtime route (#26). The compose
# file brings carla/bridge/spawn/localization/status/foxglove up as
# services; this script only verifies, drives, records, and audits:
# readiness waits, one autopilot lap, topic rates, the runtime side of the
# GT-isolation audit, bags, timings. Evidence lands in $EVIDENCE.
set -o pipefail

source /opt/brightdrive/setup.bash
source /ws/install/setup.bash 2>/dev/null || true

EVIDENCE="${EVIDENCE:-/ws/evidence/compose-smoke}"
ROOT="${ROOT:-/ws/brightdrive}"
LOGS="$EVIDENCE/logs"
mkdir -p "$LOGS" "$EVIDENCE/bags"

fail() {
  echo "SMOKE_FAILED: $*" >&2
  echo "SMOKE_FAILED: $*" >>"$EVIDENCE/validation_status.txt"
  exit 1
}

wait_topic() {
  python3 "$ROOT/scripts/validation/wait_graph.py" --topic "$1" --timeout "${2:-60}"
}

wait_message() {
  topic="$1"
  timeout "${2:-60}" ros2 topic echo --no-daemon --once "$topic" >/dev/null 2>&1
}

started_epoch=$(date +%s)
carla_ready=""

# CARLA RPC readiness (real client probe, not just the TCP healthcheck).
echo "==> waiting for CARLA RPC"
python3 - "$EVIDENCE" <<'PY' || fail "CARLA RPC did not become ready"
import json
import sys
import time

import carla

out = sys.argv[1]
started = time.time()
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2.0)
deadline = started + 1800
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

# Sensor surface. Data-level asserts (wait_message) first and camera last:
# wait_topic passes on subscriber-only topics too, and a camera failure must
# not mask whether sensor DATA actually crosses containers.
for topic in /carla/hero/lidar /carla/hero/gnss /carla/hero/imu; do
  wait_topic "$topic" 300 || fail "missing $topic"
done
wait_message /carla/hero/lidar 60 || fail "no LiDAR sample"
wait_message /carla/hero/gnss 60 || fail "no GNSS sample"
wait_message /carla/hero/imu 60 || fail "no IMU sample"
wait_topic /carla/hero/camera_front/image 300 || fail "missing /carla/hero/camera_front/image"
wait_message /carla/hero/camera_front/image 60 || fail "no camera sample"

# Estimator surface.
for topic in /kiss/odometry /odometry/filtered /localization/status; do
  wait_topic "$topic" 180 || fail "estimator missing $topic"
done

# Move the hero so odometry and bags are non-trivial.
python3 "$ROOT/scripts/validation/drive_hero.py" >"$LOGS/drive_hero.log" 2>&1 \
  || echo "autopilot setup failed; continuing with stationary evidence" >&2

sleep 5

# Topic rates.
for pair in "lidar:/carla/hero/lidar" "gnss:/carla/hero/gnss" "imu:/carla/hero/imu" "clock:/clock"; do
  name="${pair%%:*}"
  topic="${pair#*:}"
  timeout 8 ros2 topic hz "$topic" >"$LOGS/hz_$name.txt" 2>&1 || true
done

# Runtime GT-isolation audit: estimator nodes must not subscribe to the
# visualization camera (the source-side audit runs in the evaluator).
for node in /kiss_icp_node /ekf_filter_node /localization_core /localization_status; do
  safe_name="${node#/}"
  ros2 node info --no-daemon "$node" >"$LOGS/node_info_$safe_name.txt" 2>&1 || true
done
if grep -R "/carla/hero/camera_front" "$LOGS"/node_info_*.txt; then
  fail "an estimator node subscribes to the visualization camera"
fi

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

ros2 topic list --no-daemon >"$EVIDENCE/topics.txt" 2>&1 || true
ros2 node list --no-daemon >"$EVIDENCE/nodes.txt" 2>&1 || true

finished_epoch=$(date +%s)
python3 - "$EVIDENCE" "$started_epoch" "$carla_ready" "$finished_epoch" <<'PY'
import json
import sys

evidence, started, carla_ready_at, finished = sys.argv[1:]
with open(f"{evidence}/timings.json", "w", encoding="utf-8") as handle:
    json.dump(
        {
            "smoke_start_epoch": int(started),
            "carla_rpc_ready_epoch": int(carla_ready_at),
            "smoke_end_epoch": int(finished),
            "carla_rpc_from_smoke_start_s": int(carla_ready_at) - int(started),
            "total_smoke_s": int(finished) - int(started),
        },
        handle,
        indent=2,
    )
    handle.write("\n")
PY

du -sh "$EVIDENCE"/bags/* >"$EVIDENCE/bag_sizes.txt" 2>&1 || true
date -Is >"$EVIDENCE/READY_FOR_FOXGLOVE"
echo "SMOKE_OK in $((finished_epoch - started_epoch))s; evidence: $EVIDENCE"
echo "stack left running for Foxglove; make down tears it down"
