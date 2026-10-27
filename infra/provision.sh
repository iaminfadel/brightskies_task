#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/common.sh"

echo "==> Enabling APIs"
gcloud services enable \
  compute.googleapis.com \
  iap.googleapis.com \
  iam.googleapis.com \
  cloudresourcemanager.googleapis.com \
  storage.googleapis.com \
  logging.googleapis.com \
  monitoring.googleapis.com \
  cloudbilling.googleapis.com \
  billingbudgets.googleapis.com \
  sts.googleapis.com \
  iamcredentials.googleapis.com \
  --project="$PROJECT_ID" >/dev/null

echo "==> Network"
if ! gcloud compute networks describe "$NETWORK" >/dev/null 2>&1; then
  gcloud compute networks create "$NETWORK" \
    --subnet-mode=custom \
    --description="BrightDrive isolated VPC"
fi

if ! gcloud compute networks subnets describe "$SUBNET" --region="$REGION" >/dev/null 2>&1; then
  gcloud compute networks subnets create "$SUBNET" \
    --network="$NETWORK" \
    --region="$REGION" \
    --range="$SUBNET_RANGE" \
    --description="BrightDrive runtime subnet"
fi

if ! gcloud compute firewall-rules describe brightskies-allow-iap-ssh >/dev/null 2>&1; then
  gcloud compute firewall-rules create brightskies-allow-iap-ssh \
    --network="$NETWORK" \
    --direction=INGRESS \
    --action=ALLOW \
    --rules=tcp:22 \
    --source-ranges=35.235.240.0/20 \
    --description="SSH only through IAP; no public SSH"
fi

echo "==> Service accounts"
for sa in brightskies-ci brightskies-vm; do
  if ! gcloud iam service-accounts describe "$sa@$PROJECT_ID.iam.gserviceaccount.com" >/dev/null 2>&1; then
    gcloud iam service-accounts create "$sa" --display-name="BrightDrive $sa"
  fi
done

echo "==> Artifact bucket"
if ! gcloud storage buckets describe "gs://$BUCKET" >/dev/null 2>&1; then
  gcloud storage buckets create "gs://$BUCKET" \
    --location="$REGION" \
    --uniform-bucket-level-access \
    --public-access-prevention
fi
gcloud storage buckets update "gs://$BUCKET" \
  --lifecycle-file="$INFRA_DIR/bucket-lifecycle.json" >/dev/null

echo "==> IAM"
for role in roles/compute.instanceAdmin.v1 roles/iap.tunnelResourceAccessor; do
  gcloud projects add-iam-policy-binding "$PROJECT_ID" \
    --member="serviceAccount:$CI_SA" --role="$role" --condition=None >/dev/null
done
# gcloud compute ssh on the VM (instance metadata path) requires acting as the
# VM's service account.
gcloud iam service-accounts add-iam-policy-binding "$VM_SA" \
  --member="serviceAccount:$CI_SA" --role=roles/iam.serviceAccountUser >/dev/null
for role in roles/logging.logWriter roles/monitoring.metricWriter; do
  gcloud projects add-iam-policy-binding "$PROJECT_ID" \
    --member="serviceAccount:$VM_SA" --role="$role" --condition=None >/dev/null
done
for member in "serviceAccount:$CI_SA" "serviceAccount:$VM_SA"; do
  gcloud storage buckets add-iam-policy-binding "gs://$BUCKET" \
    --member="$member" --role=roles/storage.objectAdmin >/dev/null
done

echo "==> Workload Identity Federation"
if ! gcloud iam workload-identity-pools describe "$WIF_POOL" --location=global >/dev/null 2>&1; then
  gcloud iam workload-identity-pools create "$WIF_POOL" \
    --location=global --display-name="GitHub Actions"
fi
if ! gcloud iam workload-identity-pools providers describe "$WIF_PROVIDER" \
  --workload-identity-pool="$WIF_POOL" --location=global >/dev/null 2>&1; then
  gcloud iam workload-identity-pools providers create-oidc "$WIF_PROVIDER" \
    --location=global \
    --workload-identity-pool="$WIF_POOL" \
    --display-name="GitHub OIDC" \
    --issuer-uri=https://token.actions.githubusercontent.com \
    --attribute-mapping="google.subject=assertion.sub,attribute.repository=assertion.repository,attribute.ref=assertion.ref" \
    --attribute-condition="assertion.repository == '${REPO}' && (assertion.ref == 'refs/heads/main' || assertion.ref == 'refs/heads/staging')"
fi
gcloud iam service-accounts add-iam-policy-binding "$CI_SA" \
  --member="principalSet://iam.googleapis.com/projects/$PROJECT_NUMBER/locations/global/workloadIdentityPools/$WIF_POOL/attribute.repository/$REPO" \
  --role=roles/iam.workloadIdentityUser >/dev/null

echo "==> Budget alerts (billing account currency: CAD)"
budget_names="$(gcloud billing budgets list --billing-account="$BILLING_ACCOUNT" --format='value(displayName)' 2>/dev/null || true)"
if ! grep -qx "BrightDrive ~25 USD alert (${BUDGET_ALERT_CAD} CAD)" <<<"$budget_names"; then
  gcloud billing budgets create --billing-account="$BILLING_ACCOUNT" \
    --display-name="BrightDrive ~25 USD alert (${BUDGET_ALERT_CAD} CAD)" \
    --budget-amount="$BUDGET_ALERT_CAD" \
    --threshold-rule=percent=1.0 \
    --calendar-period=month
fi
if ! grep -qx "BrightDrive ~100 USD cap alert (${BUDGET_CAP_CAD} CAD)" <<<"$budget_names"; then
  gcloud billing budgets create --billing-account="$BILLING_ACCOUNT" \
    --display-name="BrightDrive ~100 USD cap alert (${BUDGET_CAP_CAD} CAD)" \
    --budget-amount="$BUDGET_CAP_CAD" \
    --threshold-rule=percent=1.0 \
    --calendar-period=month
fi

echo "==> Runtime VM"
if exists_instance; then
  echo "VM $VM_NAME already exists in $ZONE"
else
  gcloud compute instances create "$VM_NAME" \
    --zone="$ZONE" \
    --machine-type="$MACHINE_TYPE" \
    --accelerator="type=$GPU_TYPE,count=1" \
    --maintenance-policy=TERMINATE \
    --provisioning-model=STANDARD \
    --image-family=ubuntu-2204-lts \
    --image-project=ubuntu-os-cloud \
    --boot-disk-size="$BOOT_DISK_SIZE" \
    --boot-disk-type=pd-balanced \
    --network="$NETWORK" \
    --subnet="$SUBNET" \
    --service-account="$VM_SA" \
    --scopes=cloud-platform \
    --tags=brightskies-vm \
    --labels=task=brightdrive,role=runtime \
    --metadata=enable-oslogin=FALSE \
    --metadata-from-file=startup-script="$INFRA_DIR/startup.sh"
fi

echo "==> Done"
echo "Run 'make vm-up' to start the VM and wait for IAP SSH."
