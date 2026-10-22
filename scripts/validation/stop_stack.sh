#!/usr/bin/env bash
set -euo pipefail
docker rm -f brightdrive-deps carla-server >/dev/null 2>&1 || true
echo "stopped validation containers"
