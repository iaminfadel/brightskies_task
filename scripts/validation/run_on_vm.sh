#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/ws/brightdrive}"
EVIDENCE="${EVIDENCE:-/ws/evidence}"

DEPS_LOCK="${DEPS_LOCK:-$(cd "$(dirname "$0")/../.." && pwd)/docker/deps.lock}"
if [ -f "$DEPS_LOCK" ]; then
  # shellcheck source=/dev/null
  . "$DEPS_LOCK"
fi
CARLA_IMAGE="${CARLA_IMAGE:-carlasim/carla:0.9.15@sha256:1af47c314443f4a2d0869ac399c5b5eabde138a5bc39dd31dcbb1540e22f5588}"
DEPS_IMAGE="${DEPS_IMAGE:-ghcr.io/iaminfadel/brightdrive-deps:validation}"

mkdir -p "$EVIDENCE/logs"

docker image inspect "$CARLA_IMAGE" >/dev/null 2>&1 || {
  echo "missing CARLA image: $CARLA_IMAGE" >&2
  exit 1
}
docker image inspect "$DEPS_IMAGE" >/dev/null 2>&1 || {
  echo "missing deps image: $DEPS_IMAGE" >&2
  exit 1
}

{
  date -Is
  uname -a
  nvidia-smi -q
  echo "--- vulkaninfo ---"
  vulkaninfo
  echo "--- docker version ---"
  docker version
  echo "--- disk ---"
  df -h
  echo "--- memory ---"
  free -h
} >"$EVIDENCE/host_info.txt" 2>&1 || true

docker rm -f brightdrive-deps >/dev/null 2>&1 || true

echo "==> starting CARLA server"
if [ "${KEEP_CARLA:-0}" = "1" ] && docker inspect carla-server >/dev/null 2>&1; then
  docker start carla-server >/dev/null
else
  docker rm -f carla-server >/dev/null 2>&1 || true
  docker run -d --name carla-server --gpus all --network host --shm-size=4g \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    "$CARLA_IMAGE" /bin/bash -lc \
    'exec ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Low -ResX=1280 -ResY=720 -carla-server -world-port=2000' \
    >/dev/null
fi

echo "==> starting deps container"
docker run -d --name brightdrive-deps --gpus all --network host --shm-size=4g \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e EVIDENCE="$EVIDENCE" \
  -e ROOT="$ROOT" \
  -v "$ROOT:$ROOT" \
  -v "$EVIDENCE:$EVIDENCE" \
  "$DEPS_IMAGE" sleep infinity >/dev/null

docker exec brightdrive-deps bash -lc \
  'source /opt/brightdrive/setup.bash && ros2 pkg list >/dev/null' \
  || {
    echo "deps image is not usable" >&2
    docker logs brightdrive-deps >&2 || true
    exit 1
  }

if [ "${START_ONLY:-0}" = "1" ]; then
  echo "stack started (start-only)"
  exit 0
fi

echo "==> running validation inside deps container"
set +e
docker exec brightdrive-deps bash -lc \
  "source /opt/brightdrive/setup.bash && $ROOT/scripts/validation/run_validation.sh"
rc=$?
set -e

docker logs carla-server >"$EVIDENCE/logs/carla_server.log" 2>&1 || true
if [ "$rc" -ne 0 ]; then
  echo "validation failed (rc=$rc); tailing logs" >&2
  for log in bridge spawn kiss ekf foxglove; do
    echo "--- $log ---" >&2
    tail -20 "$EVIDENCE/logs/$log.log" >&2 || true
  done
  exit "$rc"
fi

cat <<EOF
Validation is running. To view live data from the laptop:
  gcloud compute ssh brightskies-vm --zone=europe-west4-c --tunnel-through-iap -- -N -L 8765:127.0.0.1:8765
Then open Foxglove Studio and connect to ws://localhost:8765.
Evidence: $EVIDENCE
EOF
