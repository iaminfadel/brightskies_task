#!/bin/bash
# Recover the mapping-methods run after the Spot preemption: both LeGO runs
# recorded (play_rc=0); finish extraction, bundles, and eval.  Idempotent.
set -o pipefail
OUT=/ws/research/mapping-methods/out
REPO=/ws/brightdrive-mm
BAG=/ws/bags/map_full4
EVAL_BAG=/ws/bags/map_full4_eval
SPAWN=-164.732162,-95.141876,0.300000,-179.913696
LOG() { echo "[$(date -u +%H:%M:%S)] $*"; }

# Standalone idle-stop heartbeat for the whole batch.
(
  while :; do
    sudo mkdir -p /var/lib/brightskies 2>/dev/null || true
    sudo touch /var/lib/brightskies/heartbeat 2>/dev/null || true
    sleep 120
  done
) &
HPID=$!
trap 'kill $HPID 2>/dev/null' EXIT

py() {
  docker run --rm -v /ws:/ws mm-deps:mm bash -c \
    "source /opt/brightdrive/setup.bash && cd $REPO && $*"
}

LOG "extract lego trajectories"
py "python3 scripts/lego_poses_to_map.py $OUT/lego-run/lego_out $BAG $OUT/lego.tum" \
  || { LOG "lego extraction FAILED"; exit 1; }
py "python3 scripts/lego_poses_to_map.py $OUT/lego-zhold-run/lego_out $BAG $OUT/lego-zhold.tum" \
  || { LOG "zhold extraction FAILED"; exit 1; }

LOG "build gicp-pg bundle (existing gicp.tum)"
rm -rf "$OUT/gicp-pg"
py "python3 scripts/build_map.py $BAG $OUT/gicp-pg --map-id mm-gicp-pg \
    --spawn=$SPAWN --sweep-merge --poses-tum $OUT/gicp.tum --pose-graph" \
  || LOG "gicp-pg build FAILED"

LOG "build lego bundles"
rm -rf "$OUT/lego" "$OUT/lego-zhold"
py "python3 scripts/build_map.py $BAG $OUT/lego --map-id mm-lego \
    --spawn=$SPAWN --sweep-merge --poses-tum $OUT/lego.tum" \
  || LOG "lego build FAILED"
py "python3 scripts/build_map.py $BAG $OUT/lego-zhold --map-id mm-lego-zhold \
    --spawn=$SPAWN --sweep-merge --poses-tum $OUT/lego-zhold.tum" \
  || LOG "lego-zhold build FAILED"

LOG "eval all bundles"
for bundle in baseline-kiss baseline-kiss-pg gicp-pg lego lego-zhold; do
  [ -f "$OUT/$bundle/map.pcd" ] || { LOG "eval: $bundle missing, skipped"; continue; }
  py "python3 scripts/eval_ate.py $OUT/$bundle $BAG $EVAL_BAG" \
    || LOG "eval_ate $bundle FAILED"
  py "python3 scripts/eval_loops.py $BAG $OUT/$bundle --sweep-merge" \
    || LOG "eval_loops $bundle FAILED"
done
py "python3 scripts/summarize_bundles.py $OUT"
LOG "RECOVERY_DONE"
