#!/usr/bin/env bash
# Bring the CARLA runtime up from any state (VM reboot, crash, preemption):
# containers up, carla-server (re)created directly on the target town -- a
# fresh server avoids the 0.9.15 segfault-after-hot-load_world -- then the
# bridge. Run on the VM; safe to re-run.
#   bash scripts/validation/up_carla.sh [Town05]
set -eo pipefail
TOWN=${1:-Town05}
CARLA_IMAGE=carlasim/carla:0.9.15@sha256:1af47c314443f4a2d0869ac399c5b5eabde138a5bc39dd31dcbb1540e22f5588

# Restart stopped containers; deps shares /ws and must be up for the loader.
for name in carla-server brightdrive-deps; do
  if ! docker ps --format '{{.Names}}' | grep -qx "$name"; then
    docker start "$name" >/dev/null 2>&1 || true
  fi
done

# Recreate carla-server if it is not running the target town as its initial
# map (cmd ends with the map arg) or if it exited with a crash.
running_town() {
  docker ps --format '{{.Names}}' | grep -qx carla-server || return 1
  docker inspect carla-server --format '{{join .Config.Cmd " "}}' | grep -q "/Game/Carla/Maps/$TOWN"
}
exit_code() { docker inspect carla-server --format '{{.State.ExitCode}}' 2>/dev/null; }
if ! running_town || [ "$(exit_code)" = "139" ]; then
  docker rm -f carla-server >/dev/null 2>&1 || true
  docker run -d --name carla-server --gpus all --network host \
    "$CARLA_IMAGE" /bin/bash -lc \
    "exec ./CarlaUE4.sh -RenderOffScreen -nosound -quality-level=Low -ResX=1280 -ResY=720 -benchmark -fps=10 -carla-server -world-port=2000 /Game/Carla/Maps/$TOWN" >/dev/null
  echo "carla-server recreated on $TOWN"
fi

# Wait for RPC, confirm town, kill any stale bridge, start a fresh one.
docker exec brightdrive-deps bash -lc \
  "python3 /ws/brightdrive/scripts/validation/load_carla_town.py --town $TOWN --timeout 900"
docker exec brightdrive-deps bash -lc \
  "pkill -f '[b]ridge --ros-args' 2>/dev/null; sleep 2; bash /ws/brightdrive/scripts/validation/start_bridge.sh $TOWN"
echo "CARLA_RUNTIME_UP town=$TOWN"
