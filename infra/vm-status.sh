#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

if ! exists_instance; then
  echo "VM $VM_NAME does not exist in $ZONE"
  exit 0
fi

gcloud compute instances describe "$VM_NAME" --zone="$ZONE" \
  --format='yaml(name,status,machineType.basename(),scheduling.provisioningModel,scheduling.instanceTerminationAction,scheduling.maxRunDuration,scheduling.preemptionNoticeDuration.seconds,disks[0].diskSizeGb,disks[0].type,networkInterfaces[0].accessConfigs[0].natIP,serviceAccounts[0].email,lastStartTimestamp,lastStopTimestamp)'
