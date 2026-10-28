#!/usr/bin/env bash
# One-shot smoke driver over the compose runtime route (#26). The compose
# file brings carla/bridge/spawn/localization/foxglove up as services; this
# script only verifies, drives, records, and audits:
# readiness waits, one autopilot lap, topic rates, the runtime side of the
# GT-isolation audit, bags, timings. Evidence lands in $EVIDENCE.
set -o pipefail

source /opt/brightdrive/setup.bash
source /ws/install/setup.bash 2>/dev/null || true

# A short-lived `ros2 --no-daemon` participant lists an incomplete graph
# (it returned 4 of 12 nodes right after the bags, and missed
# /localization_core for the audit). The daemon keeps discovering, so the
# evidence and the node-info audit use it; message probes still use
# --no-daemon (they only need their own subscription).
ros2 daemon start >/dev/null 2>&1 || true

EVIDENCE="${EVIDENCE:-/ws/evidence/compose-smoke}"
ROOT="${ROOT:-/ws/brightdrive}"
LOGS="$EVIDENCE/logs"
mkdir -p "$LOGS" "$EVIDENCE/bags"
rm -f "$EVIDENCE/validation_status.txt"
# ros2 bag record refuses to overwrite: a re-run must not fail on the
# previous run's evidence.
rm -rf "$EVIDENCE/bags/estimator" "$EVIDENCE/bags/visualization"

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
# One fresh short-lived process per attempt, hard-bounded by `timeout`: a
# long-lived client that survives a CARLA crash/restart can deadlock in the
# C++ network thread (observed as a futex hang) and leaks a streaming
# session that the render/stream ports then miss.
echo "==> waiting for CARLA RPC"
rpc_tmp="$EVIDENCE/carla_rpc_ready.json"
rpc_deadline=$(( $(date +%s) + 1800 ))
while :; do
  if timeout 20 python3 - "$EVIDENCE" >"$LOGS/carla_rpc.log" 2>&1 <<'PY'
import json
import sys

import carla

out = sys.argv[1]
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2.0)
world = client.get_world()
with open(f"{out}/.carla_rpc_ready.json", "w", encoding="utf-8") as handle:
    json.dump(
        {
            "map": world.get_map().name,
            "client_version": client.get_client_version(),
            "server_version": client.get_server_version(),
        },
        handle,
        indent=2,
    )
    handle.write("\n")
PY
  then
    cat "$LOGS/carla_rpc.log" 2>/dev/null || true
    break
  fi
  if [ "$(date +%s)" -ge "$rpc_deadline" ]; then
    fail "CARLA RPC did not become ready: $(tail -1 "$LOGS/carla_rpc.log" 2>/dev/null)"
  fi
  sleep 2
done
carla_ready=$(date +%s)
python3 - "$rpc_tmp" "$carla_ready" "$started_epoch" <<'PY'
import json
import sys

path, ready, started = sys.argv[1:]
with open(path, encoding="utf-8") as handle:
    data = json.load(handle)
data["elapsed_s"] = int(ready) - int(started)
with open(path, "w", encoding="utf-8") as handle:
    json.dump(data, handle, indent=2)
    handle.write("\n")
PY
echo "CARLA RPC ready on $(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["map"])' "$rpc_tmp")"

# Sensor surface. Data-level asserts (wait_message) first and camera last:
# wait_topic passes on subscriber-only topics too, and a camera failure must
# not mask whether sensor DATA actually crosses containers. Timeouts are
# generous because the bridge self-heals (exits) when its first camera
# stream comes up empty after a CARLA startup crash; one bridge restart is
# normal on a cold start, and sample probes must outlive it.
for topic in /carla/hero/lidar /carla/hero/gnss /carla/hero/imu; do
  wait_topic "$topic" 300 || fail "missing $topic"
done
sleep 20
wait_message /carla/hero/lidar 300 || fail "no LiDAR sample"
wait_message /carla/hero/gnss 300 || fail "no GNSS sample"
wait_message /carla/hero/imu 300 || fail "no IMU sample"
wait_topic /carla/hero/camera_front/image 300 || fail "missing /carla/hero/camera_front/image"
wait_message /carla/hero/camera_front/image 300 || fail "no camera sample"

# Estimator surface.
for topic in /kiss/odometry /odometry/filtered /localization/status; do
  wait_topic "$topic" 300 || fail "estimator missing $topic"
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
# visualization camera (the source-side audit runs in the evaluator). A
# node that cannot be inspected makes the audit vacuous, so that is a
# failure, not a warning.
for node in /kiss_icp_node /ekf_filter_node /localization_core; do
  safe_name="${node#/}"
  info="$LOGS/node_info_$safe_name.txt"
  inspected=0
  for _ in $(seq 1 10); do
    ros2 node info "$node" >"$info" 2>&1 && ! grep -q "Unable to find node" "$info" && {
      inspected=1
      break
    }
    sleep 2
  done
  [ "$inspected" = 1 ] || fail "GT-isolation audit could not inspect $node"
done
if grep -R "/carla/hero/camera" "$LOGS"/node_info_*.txt; then
  fail "an estimator node subscribes to the visualization camera"
fi

# Bags: estimator bag without camera; visualization bag with camera only.
# The estimator bag also carries every yaw source the EKF fuses (scan_pose,
# kiss odometry, contract IMU, GNSS bootstrap pose) so a 180-deg yaw flip
# observed in /odometry/filtered can be attributed to its source offline.
# SMOKE_BAG_S widens the window for diagnostic replays; CI keeps the 20 s.
echo "==> recording bags"
timeout --signal=INT "${SMOKE_BAG_S:-20}" ros2 bag record -o "$EVIDENCE/bags/estimator" \
  /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /clock /tf /tf_static \
  /kiss/odometry /odometry/filtered /localization/status \
  /localization/scan_pose /localization/gnss_pose /localization/imu \
  >"$LOGS/bag_estimator.log" 2>&1 &
estimator_bag_pid=$!
timeout --signal=INT 20 ros2 bag record -o "$EVIDENCE/bags/visualization" \
  /carla/hero/camera_front/image /carla/hero/camera_front/camera_info /clock \
  >"$LOGS/bag_visualization.log" 2>&1 &
visualization_bag_pid=$!
wait "$estimator_bag_pid" 2>/dev/null || true
wait "$visualization_bag_pid" 2>/dev/null || true

ros2 topic list >"$EVIDENCE/topics.txt" 2>&1 || true
ros2 node list >"$EVIDENCE/nodes.txt" 2>&1 || true

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
echo "SMOKE_OK in $((finished_epoch - started_epoch))s" >>"$EVIDENCE/validation_status.txt"
echo "SMOKE_OK in $((finished_epoch - started_epoch))s; evidence: $EVIDENCE"
echo "stack left running for Foxglove; make down tears it down"
