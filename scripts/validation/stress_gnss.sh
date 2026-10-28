#!/usr/bin/env bash
# GNSS noise sweep: find where the localization KPI gate breaks.
#
# Per level: patch a COPY of config/validation/objects.json with CARLA GNSS
# blueprint noise (noise_*_stddev are degrees; the runtime bridge spawns
# sensors from OBJECTS_FILE), reset the CARLA world (load_carla_town clears
# every actor so the bridge respawn is a fresh rig instead of a duplicate),
# recreate the bridge with OBJECTS_FILE pointed at the copy, enable
# autopilot, record estimator + GT bags for DRIVE_S of wall time, then run
# eval_kpi + kpi_gate over the pinned map. CARLA and the rest of the runtime
# stay up across levels; only the bridge is recreated.
#
# A level with an existing metrics.json is skipped, so the sweep resumes
# where it stopped: rerun with a finer LEVELS list to bisect the break.
#
# Runs on the VM via scripts/vm-run.sh (detached, with the heartbeat
# wrapper the map target uses):
#   scripts/vm-run.sh 'LEVELS="0 1 5 10 20 50" bash /ws/brightdrive/scripts/validation/stress_gnss.sh'
set -uo pipefail  # no -e: one level's failing gate must not stop the sweep

ROOT=/ws/brightdrive
OUT="${OUT:-/ws/evidence/gnss-sweep}"
STRESS=/ws/stress
MAP_ID="${MAP_ID:-town05-v14-pg}"
MAP_DIR="${MAP_DIR:-/ws/maps/builds/$MAP_ID}"
LEVELS="${LEVELS:-0 1 5 10 20 50}"
DRIVE_S="${DRIVE_S:-180}"
SETTLE_S="${SETTLE_S:-30}"
RECORD_S=$((DRIVE_S + SETTLE_S + 60))
COMPOSE="docker compose -f $ROOT/docker-compose.yml --profile runtime"
# Offline eval in a one-shot eval-profile container with an overridden
# entrypoint (the legacy brightdrive-deps container is not part of the
# compose route).
EVAL_RUN="docker compose -f $ROOT/docker-compose.yml --profile eval run --rm --entrypoint bash evaluator"
PY_ENV="source /opt/brightdrive/setup.bash; source /ws/install/setup.bash; cd /ws/brightdrive"

mkdir -p "$STRESS" "$OUT"
echo -e "level\tstatus\tmedian_rmse_m\taccepted_fraction\theading_p95_deg\tlatency_p95_ms" > "$OUT/summary.tsv"

echo "==> bringing the runtime route up (idempotent)"
$COMPOSE up -d

# sigma_m -> degrees (1 deg latitude = 111,320 m; Town05 georef is near lat 0)
patch_objects() {
  local sigma_m=$1 file=$2
  python3 - "$sigma_m" "$file" <<'PY'
import json
import sys

sigma_m, path = float(sys.argv[1]), sys.argv[2]
deg = sigma_m / 111320.0
doc = json.load(open("/ws/brightdrive/config/validation/objects.json"))
for obj in doc["objects"]:
    for sensor in obj.get("sensors", []):
        if sensor.get("id") == "gnss":
            sensor["noise_lat_stddev"] = deg
            sensor["noise_lon_stddev"] = deg
            sensor["noise_alt_stddev"] = deg * 2.0
json.dump(doc, open(path, "w"), indent=1)
print(f"patched {path}: gnss sigma={sigma_m} m")
PY
}

record() {
  local bag=$1
  shift
  $COMPOSE exec -d bridge bash -lc "
    source /opt/brightdrive/setup.bash; source /ws/install/setup.bash 2>/dev/null || true
    timeout -s INT $RECORD_S ros2 bag record -o $bag $*"
}

for sigma in $LEVELS; do
  tag="sigma${sigma}m"
  dir="$OUT/$tag"
  if [ -f "$dir/metrics.json" ]; then
    echo "==> $tag already done, skipping"
    continue
  fi
  echo "==> $tag"

  patch_objects "$sigma" "$STRESS/objects_$tag.json"

  # Fresh world (clears hero + sensors) then a fresh bridge spawning this
  # level's rig. CARLA stays up; --no-deps keeps the recreate scoped.
  $COMPOSE exec -T bridge python3 "$ROOT/scripts/validation/load_carla_town.py" \
    --town "${CARLA_TOWN:-Town05}" --timeout 900 \
    || { echo "$tag	world-reset-failed	FAIL	-	-	-	-" >> "$OUT/summary.tsv"; continue; }
  OBJECTS_FILE="$STRESS/objects_$tag.json" $COMPOSE up -d --no-deps --force-recreate bridge

  # Estimator ready = the whole chain (bridge -> sensors -> kiss -> ekf) flows.
  if ! $COMPOSE exec -T bridge bash -lc "
    source /opt/brightdrive/setup.bash
    for i in \$(seq 1 120); do
      timeout 5 ros2 topic echo --no-daemon --once /odometry/filtered >/dev/null 2>&1 && exit 0
      sleep 5
    done
    exit 1"; then
    echo -e "$tag\testimator-never-ready\tFAIL\t-\t-\t-\t-" >> "$OUT/summary.tsv"
    continue
  fi

  mkdir -p "$dir/bags"
  record "$dir/bags/estimator" \
    /carla/hero/lidar /carla/hero/gnss /carla/hero/imu /clock /tf \
    /kiss/odometry /odometry/filtered /localization/status
  record "$dir/bags/gt" /carla/hero/odometry /clock
  sleep "$SETTLE_S"
  $COMPOSE exec -T bridge python3 "$ROOT/scripts/validation/drive_hero.py" \
    || echo "autopilot setup failed; stationary run" >&2
  sleep "$DRIVE_S"
  # bags self-terminate via timeout; give SIGINT handlers time to finalize
  sleep 25

  if ! $EVAL_RUN -lc "$PY_ENV && python3 scripts/eval_kpi.py \
      $dir/bags/estimator $dir/bags/gt $MAP_DIR --out-dir $dir"; then
    echo -e "$tag\teval-failed\tFAIL\t-\t-\t-\t-" >> "$OUT/summary.tsv"
    continue
  fi
  if $EVAL_RUN -lc "$PY_ENV && python3 scripts/kpi_gate.py $dir/metrics.json"; then
    status=PASS
  else
    status=FAIL
  fi
  python3 - "$dir/metrics.json" >> "$OUT/summary.tsv" <<PY
import json

m = json.load(open("$dir/metrics.json"))
row = ["$tag", "$status"]
for key in ("median_rmse_m", "accepted_fraction", "heading_p95_deg", "latency_p95_ms"):
    v = m.get(key)
    row.append("-" if v is None else f"{v:.4g}")
print("\t".join(row))
PY
done

echo "==> sweep summary"
cat "$OUT/summary.tsv"
