#!/usr/bin/env bash
set -euo pipefail

INFRA_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$INFRA_DIR/config.env"

if ! command -v gcloud >/dev/null 2>&1 && [[ -x "$HOME/google-cloud-sdk/bin/gcloud" ]]; then
  export PATH="$HOME/google-cloud-sdk/bin:$PATH"
fi

command -v gcloud >/dev/null 2>&1 || {
  echo "gcloud not found; install the Google Cloud CLI first" >&2
  exit 1
}

gcloud config set project "$PROJECT_ID" >/dev/null

exists_instance() {
  gcloud compute instances describe "$VM_NAME" --zone="$ZONE" >/dev/null 2>&1
}

instance_state() {
  gcloud compute instances describe "$VM_NAME" --zone="$ZONE" \
    --format='value(status)' 2>/dev/null || echo "MISSING"
}
