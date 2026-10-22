#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

state="$(instance_state)"
if [[ "$state" == "MISSING" ]]; then
  echo "VM $VM_NAME does not exist"
  exit 0
fi
if [[ "$state" == "TERMINATED" || "$state" == "STOPPED" ]]; then
  echo "VM $VM_NAME is already $state"
  exit 0
fi

echo "==> Stopping $VM_NAME in $ZONE"
gcloud compute instances stop "$VM_NAME" --zone="$ZONE" --quiet
echo "==> Stopped; persistent disk and snapshots survive"
