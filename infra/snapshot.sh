#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

name="${1:-brightskies-baseline-$(date -u +%Y%m%d-%H%M%S)}"
echo "==> Creating snapshot $name from $VM_NAME"
gcloud compute snapshots create "$name" \
  --source-disk="$VM_NAME" \
  --source-disk-zone="$ZONE" \
  --storage-location="$REGION" \
  --description="BrightDrive baseline after runtime validation"
echo "==> Snapshot $name created"
