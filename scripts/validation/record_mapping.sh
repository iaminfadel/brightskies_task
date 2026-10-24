#!/usr/bin/env bash
# Record a mapping run with automatic CARLA-crash retries. Runs on the VM host
# (needs docker): each attempt brings the runtime up (up_carla.sh), then runs
# one recording pass inside the deps container. carla-server 0.9.15 headless
# SIGSEGVs under load occasionally; a fresh server usually clears it.
#   record_mapping.sh <bag> <drive_seconds> <eval_bag> [attempts]
set -o pipefail
BAG=${1:?bag}
SECONDS_ARG=${2:?drive_seconds}
EVAL_BAG=${3:?eval_bag}
ATTEMPTS=${4:-3}
ROOT=/ws/brightdrive

for attempt in $(seq 1 "$ATTEMPTS"); do
  echo "=== RECORD_ATTEMPT $attempt/$ATTEMPTS $(date -u +%H:%M:%S)"
  if ! bash "$ROOT/scripts/validation/up_carla.sh" Town05; then
    echo "runtime bring-up failed; retrying"
    docker rm -f carla-server >/dev/null 2>&1 || true
    continue
  fi
  if docker exec brightdrive-deps bash -lc \
    "source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; \
     bash $ROOT/scripts/map_run.sh $BAG $SECONDS_ARG $EVAL_BAG"; then
    echo "RECORD_OK attempt=$attempt"
    exit 0
  fi
  echo "recording attempt $attempt failed; recreating carla-server"
  docker rm -f carla-server >/dev/null 2>&1 || true
done
echo "RECORD_FAILED after $ATTEMPTS attempts" >&2
exit 1
