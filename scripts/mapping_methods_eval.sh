#!/bin/bash
# Mapping-methods research driver: build and evaluate every candidate mapper
# on one mapping bag + GT eval bag, on the VM.
#
# Runs ON THE VM from /ws/brightdrive (deploy via scripts/vm-run.sh):
#   scripts/mapping_methods_eval.sh <mapping_bag> <eval_bag> <spawn> [stage...]
#
# Stages: sweeps baseline gicp lego lego_zhold eval
# Every stage is idempotent (skipped when its output exists); delete the
# output or set FORCE=1 to rerun.  All bag reads use --sweep-merge so scan
# counts match across candidates (the user's azimuth-span merge).
set -uo pipefail

BAG=${1:?mapping bag}
EVAL_BAG=${2:?GT eval bag}
SPAWN=${3:?spawn x,y,z,yaw}
shift 3 || true
STAGES=${@:-sweeps baseline gicp lego lego_zhold eval}

DEPS_IMAGE=ghcr.io/iaminfadel/brightdrive-deps:0.9.15-r1
LEGO_IMAGE=legoloam:mm
ROOT=/ws/research/mapping-methods
OUT=$ROOT/out
REPO=/ws/brightdrive
LOG() { echo "[$(date -u +%H:%M:%S)] $*"; }

mkdir -p "$OUT"

# Idle-stop guard: refresh the heartbeat for the whole driver lifetime.
(
  while :; do
    sudo mkdir -p /var/lib/brightskies 2>/dev/null || true
    sudo touch /var/lib/brightskies/heartbeat 2>/dev/null || true
    sleep 120
  done
) &
HEARTBEAT_PID=$!
trap 'kill "$HEARTBEAT_PID" 2>/dev/null' EXIT

py() {  # python inside the deps image
  docker run --rm -v /ws:/ws "$DEPS_IMAGE" bash -c \
    "source /opt/brightdrive/setup.bash && cd $REPO && $*"
}

run_stage() {
  local name=$1
  shift
  if [ "${FORCE:-0}" != "1" ] && [ -f "$OUT/.done-$name" ]; then
    LOG "stage $name: already done"
    return 0
  fi
  LOG "stage $name: start"
  local t0=$(date +%s)
  "$@"
  local rc=$?
  local t1=$(date +%s)
  LOG "stage $name: rc=$rc in $((t1 - t0))s"
  [ $rc -eq 0 ] && touch "$OUT/.done-$name"
  return $rc
}

stage_sweeps() {
  py "python3 scripts/sweep_bag.py $BAG $OUT/sweeps"
}

stage_baseline() {
  py "python3 scripts/build_map.py $BAG $OUT/baseline-kiss --map-id mm-kiss \
      --spawn $SPAWN --sweep-merge --kiss-only"
  py "python3 scripts/build_map.py $BAG $OUT/baseline-kiss --map-id mm-kiss \
      --spawn $SPAWN --sweep-merge"
  py "python3 scripts/build_map.py $BAG $OUT/baseline-kiss-pg --map-id mm-kiss-pg \
      --spawn $SPAWN --sweep-merge --pose-graph --imu-delta"
}

stage_gicp() {
  py "python3 scripts/gicp_odom.py $BAG $OUT/gicp.tum"
  py "python3 scripts/build_map.py $BAG $OUT/gicp-pg --map-id mm-gicp-pg \
      --spawn $SPAWN --sweep-merge --poses-tum $OUT/gicp.tum --pose-graph"
}

stage_lego() {
  if ! docker image inspect "$LEGO_IMAGE" >/dev/null 2>&1; then
    LOG "building $LEGO_IMAGE"
    docker build -f "$REPO/docker/legoloam.Dockerfile" -t "$LEGO_IMAGE" "$REPO" \
      > "$OUT/lego_build.log" 2>&1 \
      || { tail -30 "$OUT/lego_build.log"; return 1; }
  fi
  docker run --rm -v /ws:/ws "$LEGO_IMAGE" \
    bash /ws/brightdrive/scripts/lego_offline.sh "$OUT/sweeps" "$OUT/lego-run" true false
  py "python3 scripts/lego_poses_to_map.py $OUT/lego-run/lego_out $BAG $OUT/lego.tum"
  py "python3 scripts/build_map.py $BAG $OUT/lego --map-id mm-lego \
      --spawn $SPAWN --sweep-merge --poses-tum $OUT/lego.tum"
}

stage_lego_zhold() {
  docker run --rm -v /ws:/ws "$LEGO_IMAGE" \
    bash /ws/brightdrive/scripts/lego_offline.sh "$OUT/sweeps" "$OUT/lego-zhold-run" true true
  py "python3 scripts/lego_poses_to_map.py $OUT/lego-zhold-run/lego_out $BAG $OUT/lego-zhold.tum"
  py "python3 scripts/build_map.py $BAG $OUT/lego-zhold --map-id mm-lego-zhold \
      --spawn $SPAWN --sweep-merge --poses-tum $OUT/lego-zhold.tum"
}

stage_eval() {
  for bundle in baseline-kiss baseline-kiss-pg gicp-pg lego lego-zhold; do
    local dir=$OUT/$bundle
    [ -f "$dir/map.pcd" ] || { LOG "eval: $bundle missing, skipped"; continue; }
    LOG "eval: $bundle"
    py "python3 scripts/eval_ate.py $dir $BAG $EVAL_BAG" \
      || LOG "eval_ate $bundle: FAILED (numbers still written)"
    py "python3 scripts/eval_loops.py $BAG $dir --sweep-merge" \
      || LOG "eval_loops $bundle: FAILED (numbers still written)"
  done
  py "python3 scripts/summarize_bundles.py $OUT"
}

# A failing stage is logged and skipped (its outputs stay missing; eval
# handles missing bundles) so later candidates still run.
for stage in $STAGES; do
  case $stage in
    sweeps) run_stage sweeps stage_sweeps ;;
    baseline) run_stage baseline stage_baseline ;;
    gicp) run_stage gicp stage_gicp ;;
    lego) run_stage lego stage_lego ;;
    lego_zhold) run_stage lego_zhold stage_lego_zhold ;;
    eval) run_stage eval stage_eval ;;
    *) LOG "unknown stage $stage"; exit 2 ;;
  esac
done
LOG "DRIVER_DONE stages=$STAGES"
