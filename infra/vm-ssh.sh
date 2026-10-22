#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

exec gcloud compute ssh "$VM_NAME" \
  --zone="$ZONE" \
  --tunnel-through-iap \
  --strict-host-key-checking=no \
  -- "$@"
