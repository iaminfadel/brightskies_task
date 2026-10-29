#!/usr/bin/env bash
# Full S1-S4 KPI matrix on the VM. Builds the workspace, runs each scenario
# in order (recording estimator + GT bags), scores and gates each run, and
# stops on the first gate failure. Evidence lands in $EVIDENCE.
set -euo pipefail

ROOT="${ROOT:-/ws/brightdrive}"
EVIDENCE="${EVIDENCE:-/ws/evidence}"
export EVIDENCE
cd "$ROOT"

heartbeat_loop() {
  while :; do
    sudo -S -p '' mkdir -p /var/lib/brightskies 2>/dev/null || true
    sudo -S -p '' touch /var/lib/brightskies/heartbeat 2>/dev/null || true
    sleep 120
  done
}
heartbeat_loop &
heartbeat_pid=$!
trap 'kill "$heartbeat_pid" 2>/dev/null || true' EXIT

rm -rf "$EVIDENCE"
mkdir -p "$EVIDENCE/logs"

for _ in $(seq 1 60); do
  docker info >/dev/null 2>&1 && break
  sleep 5
done
docker info >/dev/null 2>&1 || { echo "docker daemon not reachable" >&2; exit 1; }

. docker/deps.lock
for image in "$CARLA_IMAGE" "$DEPS_IMAGE"; do
  if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "==> pulling $image"
    docker pull "$image" || { echo "missing image: $image" >&2; exit 1; }
  fi
done

{
  date -Is; uname -a; nvidia-smi -q 2>/dev/null || true
  echo "--- docker version ---"; docker version
  echo "--- disk ---"; df -h
} >"$EVIDENCE/host_info.txt" 2>&1 || true

echo "==> building the workspace"
bash infra/ros_exec.sh "colcon build --base-paths /ws/brightdrive/src \
  --build-base /ws/build --install-base /ws/install"

run_scenario() {
  local S="$1"
  local profile="nominal"
  [ "$S" = "S2" ] && profile="noisy"
  echo "==> scenario $S (profile=$profile)"
  docker compose --profile runtime --profile scenario down --remove-orphans --timeout 10 || true
  GNSS_PROFILE="$profile" SCENARIO="$S" SCENARIO_SEED="${SCENARIO_SEED:-7}" \
    SCENARIO_SEED_RUNS="${SCENARIO_SEED_RUNS:-1}" \
    SCENARIO_EVIDENCE="${SCENARIO_EVIDENCE:-/ws/evidence/scenarios}" \
    OBJECTS_FILE="${OBJECTS_FILE:-/ws/brightdrive/config/scenario/objects_scenario.json}" \
    docker compose --profile runtime --profile scenario run --rm scenario
}

run_eval() {
  local S="$1"
  local seed="${SCENARIO_SEED:-7}"
  local est="/ws/evidence/scenarios/$S/seed_$seed/bags/estimator"
  local gt="/ws/evidence/scenarios/$S/seed_$seed/bags/gt"
  local out="/ws/evidence/scenarios/$S"
  echo "==> evaluating $S"
  SCENARIO="$S" ESTIMATOR_BAG="$est" GT_BAG="$gt" \
    MAP_DIR="${MAP_DIR:-/ws/maps/builds/town05-v14-pg}" \
    KPI_EVIDENCE="$out" METRICS_FILE="$out/metrics.json" \
    docker compose --profile eval run --rm evaluator
}

for S in S1 S2 S3 S4; do
  run_scenario "$S"
  run_eval "$S"
done

# collect service logs once at the end
for service in carla bridge localization map-server foxglove; do
  docker compose --profile runtime logs --no-color "$service" \
    >"$EVIDENCE/logs/$service.log" 2>&1 || true
done
docker compose --profile runtime --profile scenario --profile eval down --remove-orphans --timeout 30 || true

echo "FULL_MATRIX_OK; evidence: $EVIDENCE"
