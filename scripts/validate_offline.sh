#!/usr/bin/env bash
# CARLA-free validation: replay a recorded sensor bag through the real runtime
# (map_server + localization launch, same nodes and parameters as `make up`)
# and score the result with the evaluation path.
#
# Use this when the CARLA VM is unavailable. It exercises the estimator, the
# map serving contract, sim-time handling and the KPI evaluator; it does not
# exercise the CARLA bridge or ScenarioRunner.
#
#   scripts/validate_offline.sh <bag> <gt_bag> <map_dir> <out_dir> [speed]
#
# `speed` is sim-time speed: 1.0 replays as if the simulator had run in real
# time. Recordings made faster than real time (the CARLA bridge free-runs when
# uncontrolled: the pinned bag holds 627 s of sim time in 268 s of bag time)
# are automatically slowed so the estimator sees the sensor load it saw live.
set -o pipefail

BAG=${1:?sensor bag}
GT_BAG=${2:?ground-truth bag}
MAP_DIR=${3:?map bundle directory}
OUT=${4:?evidence directory}
SPEED=${5:-1.0}

# ROS setup scripts are not `set -u` clean.
set +u
source /opt/brightdrive/setup.bash
source /ws/install/setup.bash 2>/dev/null || true
set -u

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# The pinned deps image predates the rosbags/scipy line in the Dockerfile.
bash "$HERE/ensure_py_deps.sh"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"

# Bag playback is paced by bag time, but the stack runs on the bag's /clock.
# Convert the requested sim-time speed into a --rate that makes sim time
# advance at that speed: rate = speed * bag_duration / sim_duration.
read -r PLAY_RATE SIM_S WALL_S < <(python3 - "$BAG" "$SPEED" <<'PY'
import sys

import yaml
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

bag, speed = sys.argv[1], float(sys.argv[2])
typestore = get_typestore(Stores.ROS2_HUMBLE)
try:
    with Reader(bag) as reader:
        wall = reader.duration / 1e9
        clocks = [c for c in reader.connections if c.topic == "/clock"]
        if not clocks:
            print(speed, 0.0, wall)
            raise SystemExit
        first = last = None
        for connection, _, raw in reader.messages(connections=clocks):
            stamp = typestore.deserialize_cdr(raw, connection.msgtype).clock
            now = stamp.sec + stamp.nanosec * 1e-9
            first = now if first is None else first
            last = now
        sim = (last - first) if first is not None else 0.0
        if sim <= 0:
            print(speed, 0.0, wall)
        else:
            print(round(speed * wall / sim, 3), round(sim, 1), round(wall, 1))
except Exception:  # yaml/reader/metadata trouble: play at the requested speed
    print(speed, 0.0, 0.0)
PY
)

PLAY_RATE=${PLAY_RATE:-$SPEED}
SIM_S=${SIM_S:-0.0}
WALL_S=${WALL_S:-0.0}

if [ "$SIM_S" != "0.0" ]; then
  echo "==> bag holds ${SIM_S}s of sim time in ${WALL_S}s of bag time; playing at ${PLAY_RATE}x for ${SPEED}x sim speed"
else
  echo "==> no usable /clock in the bag; playing at ${PLAY_RATE}x"
fi

LOGS="$OUT/logs"
EST_BAG="$OUT/bags/estimator"
mkdir -p "$LOGS" "$OUT/bags"
rm -rf "$EST_BAG"

pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill "$pid" 2>/dev/null; done
  wait 2>/dev/null
}
trap cleanup EXIT

fail() {
  echo "OFFLINE_VALIDATION_FAILED: $*" >&2
  exit 1
}

echo "==> map server ($MAP_DIR)"
ros2 run brightdrive_map_server map_server --ros-args \
  --params-file /ws/install/brightdrive_bringup/share/brightdrive_bringup/config/map_server.yaml \
  -p use_sim_time:=true \
  -p serving.mode:=full \
  -p serving.map_dir:="$MAP_DIR" \
  >"$LOGS/map_server.log" 2>&1 &
pids+=($!)

echo "==> localization launch"
ros2 launch brightdrive_bringup localization.launch.py \
  >"$LOGS/localization.log" 2>&1 &
pids+=($!)

# The estimator fails fast if the map never arrives; wait for the handshake
# before replaying so the check is about the stack, not startup ordering.
echo "==> waiting for /localization/map_cloud"
timeout 120 python3 "$HERE/validation/wait_graph.py" \
  --topic /localization/map_cloud --publisher --timeout 110 \
  || fail "map_server never published /localization/map_cloud"

echo "==> recording estimator topics"
ros2 bag record -o "$EST_BAG" \
  /clock /tf /tf_static /carla/hero/gnss \
  /odometry/filtered /localization/status /localization/pose \
  >"$LOGS/record.log" 2>&1 &
rec_pid=$!
pids+=("$rec_pid")
sleep 3

echo "==> replaying $BAG at ${PLAY_RATE}x (${SPEED}x sim time)"
# No --clock: the recorder already stored the bridge's /clock (sim time), and
# `--clock` would publish bag (wall) time instead -- two /clock sources make
# every use_sim_time node jump between clocks, which shows up as wall-stamped
# poses and bogus message ages downstream.
ros2 bag play "$BAG" -r "$PLAY_RATE" >"$LOGS/play.log" 2>&1 \
  || fail "bag playback failed (see $LOGS/play.log)"
sleep 3

kill -INT "$rec_pid" 2>/dev/null
wait "$rec_pid" 2>/dev/null
pids=("${pids[@]/$rec_pid}")

test -f "$EST_BAG/metadata.yaml" || fail "no estimator bag was written"

cd /ws/brightdrive || fail "cannot cd /ws/brightdrive"

echo "==> map_check"
status=0
if ros2 run brightdrive_localization map_check "$MAP_DIR" \
     >"$LOGS/map_check.log" 2>&1; then
  tail -3 "$LOGS/map_check.log" || true
else
  echo "map_check FAILED" >&2
  tail -5 "$LOGS/map_check.log" >&2 || true
  status=1
fi

# pipefail turns a failing evaluator in any of these pipelines into status.
echo "==> eval_ate"
python3 scripts/eval_ate.py "$MAP_DIR" "$BAG" "$GT_BAG" 2>&1 | tail -6 \
  || { echo "eval_ate FAILED" >&2; status=1; }

echo "==> eval_kpi"
python3 scripts/eval_kpi.py "$EST_BAG" "$GT_BAG" "$MAP_DIR" --out-dir "$OUT" 2>&1 | tail -6 \
  || { echo "eval_kpi FAILED" >&2; status=1; }

echo "==> kpi_gate"
python3 scripts/kpi_gate.py "$OUT/metrics.json" 2>&1 | tail -8 \
  || { echo "kpi_gate FAILED" >&2; status=1; }

if [ "$status" -ne 0 ]; then
  fail "one or more gates failed; evidence in $OUT"
fi
echo "OFFLINE_VALIDATION_OK evidence=$OUT"
