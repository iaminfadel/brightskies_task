#!/usr/bin/env bash
# Resumable verification chain for the boiled-down runtime:
#   stage 1  pose-graph map build from the existing map_full4 bag (CPU only)
#   stage 2  GNSS noise sweep (per-level resumable; brings the compose
#            runtime up itself)
#   stage 3  fresh legacy recording of map_full5 (GPU)
# Sentinels skip completed stages, so the chain survives Spot preemptions:
# relaunch it unchanged after every VM restart. Stage 3 tears the compose
# stack down first — the legacy carla-server and the compose carla share
# host port 2000 and must not run at once.
set -uo pipefail
ROOT=/ws/brightdrive
MAP_ID=town05-v15-verify
BUILD_DIR=/ws/maps/builds/$MAP_ID
SPAWN=-164.732162,-95.141876,0.300000,-179.913696
PY_ENV="source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; cd /ws/brightdrive"

docker start brightdrive-deps >/dev/null 2>&1 || true
sleep 5

if [ -s "$BUILD_DIR/manifest.json" ]; then
  echo "==> stage 1: map build already done"
else
  echo "==> stage 1: pose-graph build from map_full4"
  docker exec brightdrive-deps bash -lc \
    "$PY_ENV && python3 scripts/build_map.py /ws/bags/map_full4 $BUILD_DIR \
     --map-id $MAP_ID --spawn=$SPAWN --pose-graph" || exit 1
fi
python3 "$ROOT/scripts/map_verify.py" "$BUILD_DIR" || exit 1

echo "==> stage 2: GNSS sweep"
bash "$ROOT/scripts/validation/stress_gnss.sh"

echo "==> stage 3: fresh recording of map_full5"
cd "$ROOT" && docker compose --profile runtime --profile test --profile eval \
  --profile mapping down --remove-orphans --timeout 20 || true
bash "$ROOT/scripts/validation/record_mapping.sh" /ws/bags/map_full5 900 /ws/bags/map_full5_eval 3
