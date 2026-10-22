#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

state="$(instance_state)"
case "$state" in
  MISSING)
    echo "VM $VM_NAME does not exist; run ./infra/provision.sh first" >&2
    exit 1
    ;;
  RUNNING)
    ;;
  TERMINATED|STOPPED|STOPPING|SUSPENDED)
    echo "==> Starting $VM_NAME in $ZONE"
    gcloud compute instances start "$VM_NAME" --zone="$ZONE"
    ;;
  *)
    echo "VM is in state $state; waiting for it to settle"
    ;;
esac

echo "==> Waiting for IAP SSH"
deadline=$((SECONDS + 600))
until gcloud compute ssh "$VM_NAME" \
  --zone="$ZONE" \
  --tunnel-through-iap \
  --strict-host-key-checking=no \
  --quiet \
  --command='echo ready' >/dev/null 2>&1; do
  if (( SECONDS >= deadline )); then
    echo "timed out waiting for IAP SSH to $VM_NAME" >&2
    exit 1
  fi
  sleep 10
done

echo "==> $VM_NAME is up; SSH via IAP is ready"
