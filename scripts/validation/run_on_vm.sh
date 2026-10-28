#!/usr/bin/env bash
# CARLA smoke on the VM, driven by the compose route (the only runtime).
#
# vm_job.py deploys the committed HEAD and calls this; it then collects
# $EVIDENCE and stops the VM. Steps: build the workspace from scratch, bring
# the runtime + test profiles up, run the one-shot smoke, keep every service
# log, tear the stack down.
set -euo pipefail

ROOT="${ROOT:-/ws/brightdrive}"
EVIDENCE="${EVIDENCE:-/ws/evidence}"
export EVIDENCE
cd "$ROOT"

# A detached run must keep the idle-stop guard from stopping the VM under it.
heartbeat_loop() {
  while :; do
    sudo mkdir -p /var/lib/brightskies 2>/dev/null || true
    sudo touch /var/lib/brightskies/heartbeat 2>/dev/null || true
    sleep 120
  done
}
heartbeat_loop &
heartbeat_pid=$!
trap 'kill "$heartbeat_pid" 2>/dev/null || true' EXIT

# /ws/evidence persists on the VM disk; stale logs from the previous run
# otherwise end up mixed into this run's evidence.
rm -rf "$EVIDENCE"
mkdir -p "$EVIDENCE/logs"

# After a cold boot the job can SSH in before dockerd accepts commands.
for _ in $(seq 1 60); do
  docker info >/dev/null 2>&1 && break
  sleep 5
done
docker info >/dev/null 2>&1 || { echo "docker daemon not reachable" >&2; exit 1; }

# shellcheck source=/dev/null
. docker/deps.lock
for image in "$CARLA_IMAGE" "$DEPS_IMAGE"; do
  # A fresh VM disk has no images; pulling beats failing (CARLA is ~10 GB, so
  # this only happens cold).
  if ! docker image inspect "$image" >/dev/null 2>&1; then
    echo "==> pulling $image"
    docker pull "$image" || {
      echo "missing image: $image" >&2
      exit 1
    }
  fi
done

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

echo "==> building the workspace"
bash infra/ros_exec.sh "colcon build --base-paths /ws/brightdrive/src \
  --build-base /ws/build --install-base /ws/install"

echo "==> running the compose smoke"
docker compose --profile runtime --profile test down --remove-orphans --timeout 10 || true
rc=0
docker compose --profile runtime --profile test run --rm smoke || rc=$?

# Collect logs before teardown: a failed smoke is only debuggable with them.
for service in carla bridge localization map-server foxglove; do
  docker compose --profile runtime logs --no-color "$service" \
    >"$EVIDENCE/logs/$service.log" 2>&1 || true
done
docker compose --profile runtime --profile test down --remove-orphans --timeout 30 || true

if [ "$rc" -ne 0 ]; then
  echo "smoke failed (rc=$rc); evidence: $EVIDENCE" >&2
  exit "$rc"
fi

cat <<EOF
Smoke passed. Evidence: $EVIDENCE
To view live data from the laptop:
  make vm-tunnel   # or: infra/vm-ssh.sh -N -L 8765:127.0.0.1:8765
Then open Foxglove Studio and connect to ws://localhost:8765.
EOF
