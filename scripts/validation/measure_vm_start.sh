#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
# shellcheck disable=SC1091
source infra/config.env
export PATH="$HOME/google-cloud-sdk/bin:$PATH"

now_ms() { date +%s%3N; }

echo "==> stopping VM"
gcloud compute instances stop "$VM_NAME" --zone="$ZONE" --quiet
sleep 2

start_ms=$(now_ms)
echo "==> starting VM"
gcloud compute instances start "$VM_NAME" --zone="$ZONE" --quiet

echo "==> waiting for SSH"
until timeout 15 ./infra/vm-ssh.sh true >/dev/null 2>&1; do
  sleep 1
done
ssh_ms=$(now_ms)

echo "==> starting CARLA/deps containers"
./infra/vm-ssh.sh 'START_ONLY=1 /ws/brightdrive/scripts/validation/run_on_vm.sh'
stack_ms=$(now_ms)

echo "==> waiting for CARLA RPC"
./infra/vm-ssh.sh 'docker exec brightdrive-deps bash -lc '"'"'source /opt/brightdrive/setup.bash && python3 - <<PY
import carla, time
client = carla.Client("127.0.0.1", 2000)
client.set_timeout(2.0)
deadline = time.time() + 1500
while time.time() < deadline:
    try:
        world = client.get_world()
        print(world.get_map().name)
        break
    except Exception:
        time.sleep(1)
else:
    raise SystemExit("CARLA RPC not ready")
PY'"'"''
carla_ms=$(now_ms)

python3 - "$start_ms" "$ssh_ms" "$stack_ms" "$carla_ms" <<'PY' >/tmp/vm_start_timings.json
import json
import sys
import time

start, ssh, stack, carla = (int(value) for value in sys.argv[1:])
print(
    json.dumps(
        {
            "measured_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "vm_start_epoch_ms": start,
            "ssh_ready_epoch_ms": ssh,
            "stack_started_epoch_ms": stack,
            "carla_ready_epoch_ms": carla,
            "vm_start_to_ssh_s": round((ssh - start) / 1000, 3),
            "ssh_to_stack_start_s": round((stack - ssh) / 1000, 3),
            "stack_start_to_carla_ready_s": round((carla - stack) / 1000, 3),
            "vm_start_to_carla_ready_s": round((carla - start) / 1000, 3),
        },
        indent=2,
    )
    + "\n"
)
PY

cat /tmp/vm_start_timings.json
cat /tmp/vm_start_timings.json | ./infra/vm-ssh.sh \
  'cat > /ws/evidence/vm_start_timings.json && echo saved'
