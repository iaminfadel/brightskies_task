#!/bin/bash
set -euo pipefail

STATE_DIR=/var/lib/brightskies
LOG=/var/log/brightskies-startup.log
mkdir -p "$STATE_DIR"
exec > >(tee -a "$LOG") 2>&1
rm -f "$STATE_DIR/bootstrap.done"

echo "=== BrightDrive bootstrap started $(date -Is) ==="

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg lsb-release jq unzip rsync git make tmux htop \
  ffmpeg vulkan-tools libvulkan1 docker.io build-essential \
  "linux-headers-$(uname -r)"

if ! command -v nvidia-smi >/dev/null 2>&1; then
  echo "=== Installing NVIDIA driver ==="
  apt-get install -y --no-install-recommends nvidia-driver-535
fi

if ! command -v nvidia-ctk >/dev/null 2>&1; then
  echo "=== Installing NVIDIA container toolkit ==="
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update
  apt-get install -y --no-install-recommends nvidia-container-toolkit
  nvidia-ctk runtime configure --runtime=docker
fi

if ! docker compose version >/dev/null 2>&1; then
  echo "=== Installing docker compose v2 plugin ==="
  # Ubuntu jammy's docker.io ships no compose plugin; pin the static binary
  # (the compose runtime of #26 depends on it).
  COMPOSE_VERSION=v2.34.0
  mkdir -p /usr/local/lib/docker/cli-plugins
  curl -fsSL \
    "https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-linux-x86_64" \
    -o /usr/local/lib/docker/cli-plugins/docker-compose
  chmod 0755 /usr/local/lib/docker/cli-plugins/docker-compose
  docker compose version
fi

systemctl enable --now docker
systemctl restart docker

for user in $(awk -F: '$3 >= 1000 && $3 < 65534 {print $1}' /etc/passwd); do
  usermod -aG docker "$user" || true
done

if ! command -v gcloud >/dev/null 2>&1; then
  echo "=== Installing Google Cloud CLI (for artifact uploads) ==="
  curl -fsSL https://packages.cloud.google.com/apt/doc/apt-key.gpg \
    | gpg --dearmor -o /usr/share/keyrings/cloud.google.gpg
  echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" \
    > /etc/apt/sources.list.d/google-cloud-sdk.list
  apt-get update
  apt-get install -y --no-install-recommends google-cloud-cli
fi

echo "=== Installing idle-stop guard ==="
cat > /usr/local/sbin/brightskies-idle-stop <<'EOS'
#!/bin/bash
set -uo pipefail
STATE_DIR=/var/lib/brightskies
HEARTBEAT="$STATE_DIR/heartbeat"
LAST="$STATE_DIR/last-activity"
IDLE_SECONDS=$(( ${IDLE_STOP_MINUTES:-30} * 60 ))

[[ -f "$STATE_DIR/bootstrap.done" ]] || exit 0
mkdir -p "$STATE_DIR"
now=$(date +%s)

if [[ ! -f "$LAST" ]]; then
  echo "$now" > "$LAST"
  exit 0
fi

active=0
if [[ -f "$HEARTBEAT" ]]; then
  hb=$(stat -c %Y "$HEARTBEAT")
  (( now - hb < 300 )) && active=1
fi
if ss -Htn state established '( sport = :22 )' 2>/dev/null | grep -q .; then
  active=1
fi
if command -v nvidia-smi >/dev/null 2>&1; then
  util=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits 2>/dev/null | head -1 | tr -d ' ')
  if [[ "$util" =~ ^[0-9]+$ ]] && (( util > 10 )); then
    active=1
  fi
fi

if (( active )); then
  echo "$now" > "$LAST"
  exit 0
fi

last=$(cat "$LAST" 2>/dev/null || echo "$now")
if (( now - last >= IDLE_SECONDS )); then
  logger -t brightskies-idle "no activity for $((now-last))s; stopping VM"
  /sbin/shutdown -h now
fi
EOS
chmod 0755 /usr/local/sbin/brightskies-idle-stop

cat > /etc/systemd/system/brightskies-idle-stop.service <<'EOS'
[Unit]
Description=BrightDrive idle stop check

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/brightskies-idle-stop
EOS

cat > /etc/systemd/system/brightskies-idle-stop.timer <<'EOS'
[Unit]
Description=BrightDrive idle stop timer

[Timer]
OnBootSec=5min
OnUnitActiveSec=5min

[Install]
WantedBy=timers.target
EOS

systemctl daemon-reload
systemctl enable --now brightskies-idle-stop.timer

touch "$STATE_DIR/heartbeat"
echo "$(date +%s)" > "$STATE_DIR/last-activity"
touch "$STATE_DIR/bootstrap.done"
echo "=== BrightDrive bootstrap finished $(date -Is) ==="
