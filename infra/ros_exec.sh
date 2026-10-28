#!/usr/bin/env bash
# Run one command in a throwaway deps container against the shared workspace.
#
# This is the one-off path (colcon build, build_map, eval_ate,
# eval_kpi, map-install, ...). It needs no running stack -- the runtime, scenarios and
# smoke all go through docker compose instead.
#
#   infra/ros_exec.sh 'python3 scripts/eval_ate.py /ws/maps/builds/x'
#
# Runs on the VM; from the laptop reach it through infra/vm-run.sh.
set -euo pipefail

if [ $# -ne 1 ]; then
  echo "usage: infra/ros_exec.sh '<command>'" >&2
  exit 2
fi

WS="${WS_DIR:-/ws}"
ROOT="$WS/brightdrive"
cd "$ROOT"
# shellcheck source=/dev/null
. docker/deps.lock

exec docker run --rm --network host -v "$WS:$WS" "$DEPS_IMAGE" \
  bash -lc "source /opt/brightdrive/setup.bash; source $WS/install/setup.bash 2>/dev/null || true; \
    bash $ROOT/infra/ensure_py_deps.sh; cd $ROOT && $1"
