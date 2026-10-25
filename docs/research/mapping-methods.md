# Mapping methods comparison: KISS-ICP vs small-gicp odometry vs patched LeGO-LOAM

Date: 2026-10-25 (runs on 10-24/25)
Branch: `research/mapping-methods` (worktree; VM tree `/ws/brightdrive-mm`)
Basis: `main` @ f745128 (sync 10 Hz runtime, azimuth/coverage sweep merge)
Status: complete — **LeGO-LOAM: confirmed no-go even with the spike's patch
applied. small-gicp scan-to-submap odometry + the existing pose graph: best
candidate, adopt.**

## Question

KISS-ICP has no loop closures and drifts on the Town05 mapping bag. LeGO-LOAM
(the team has prior experience with it) was spiked on 10-22 and declared
no-go because the fishros Humble port drops the original's IMU roll/pitch
damping — diagnosed then as "the ground-plane correction is missing", which
turned out to be wrong about the original (the original's `transformUpdate()`
damps roll/pitch toward the IMU; there is no ground-plane fit in the original
either). This research reruns that decision with the patch actually applied,
and adds a modern scan-to-submap frontend as a third candidate.

## Method

One bag (`/ws/bags/map_full4` + GT eval bag, user's sync-runtime recording:
6274 full-azimuth sweeps, 3.6 km loop), one downstream pipeline: every
candidate produces lidar poses in the slam frame as TUM, and
`build_map.py --poses-tum` (new in this branch) feeds them through the
identical anchor, accumulation, held-out split, and evals (`eval_ate.py`
against the GT bag, `eval_loops.py` inter-visit consistency).

| Candidate | Frontend | Global opt |
| --- | --- | --- |
| baseline-kiss | KISS-ICP point-to-point | none |
| baseline-kiss-pg | KISS-ICP | pose graph (GNSS factors + ICP loops) |
| gicp-pg | small_gicp scan-to-submap GICP (new `gicp_odom.py`) | same pose graph |
| lego | patched LeGO-LOAM port (own SLAM + loop closure) | own graph |
| lego-zhold | lego + flat-ground z hold (research extra) | own graph |

The LeGO patch (`docker/legoloam/patch_lego.py`) restores the original IMU
damping verbatim (port dropped the IMU subscription entirely) plus an
optional soft z hold; verified against the real source with anchored edits.

## Results (map_full4, GT = `/ws/bags/map_full4_eval`)

| bundle | ATE rmse m | ATE p95 m | drift rmse m | yaw rmse deg | loop med m | loop p95 m | z range m | closure m | xy path m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| baseline-kiss | 23.927 | 53.214 | 20.868 | 7.7 | 3.077 | 12.433 | 21.0 | 57.6 | 3456 |
| baseline-kiss-pg | 2.287 | 3.703 | 1.740 | 31.9 | 0.878 | 2.446 | 1.3 | 4.4 | 4331 |
| **gicp-pg** | **2.150** | **3.607** | **1.594** | 21.8 | **0.874** | 2.490 | **0.1** | 4.4 | 4052 |
| lego | 194.5 | 387.2 | 123.7 | 81.8 | 4.720 | 14.129 | 11.2 | 221.6 | 3161 |
| lego-zhold | 195.0 | 393.0 | 121.7 | 80.2 | 3.676 | 18.155 | 12.8 | 212.5 | 3161 |

Images: `docs/research/mapping-methods-results/<bundle>/validation/{overlay,ate,loop_check}.png`.

## Findings

1. **The LeGO patch works as designed — and that is not enough.** The
   IMU-damping patch eliminates the spike's failure mode: z stays within
   11-13 m over 3.2 km (spike: +139 m on 133 m of road), the maps are planar
   and locally crisp (5.1 M points; see lego overlay). But the frontend
   still wanders: 12 % path deficit, a mid-run excursion that cuts across
   blocks (visible in the overlay), loop-closure median correction 3.7-4.7 m
   (its naive ICP with a 0.3 fitness gate never recovers the excursion).
   ATE 194 m. The original author's "designed for real lidar" reservation
   holds on CARLA's sparse 16-beam, low-texture data. **Final: no-go, with
   evidence, at both the port-defect level (patched) and the algorithm
   level.**
2. **small-gicp scan-to-submap odometry is a strict upgrade over KISS-ICP
   here.** Raw odometry drift: loop closure 4.7 m vs KISS 57.6 m over the
   same loop (12x), path ratio 1.000 vs 0.963, z range 0.12 m vs 21 m.
   With the same pose graph it wins every ATE metric (2.15 m vs 2.29 m,
   aligned 1.59 m vs 1.74 m) and halves the yaw error. 156/6274 alignment
   failures (2.5 %, held poses). Cost: ~6 min for 6274 sweeps on 8 vCPU.
3. **The frontend is no longer the bottleneck.** Every pose-graph candidate
   fails the 0.30 m ATE gate by ~7x. The binding constraints are the graph
   (loop edges built from drifted KISS/GICP poses, GNSS factor noise) and a
   systematic yaw anchor error (yaw RMSE 21-32 deg across ALL pose-graph
   candidates including the frontend-free paths — investigate the IMU-yaw
   anchor convention of the sync bridge before touching the frontend again).
4. **Tooling defects found and fixed on this branch** (worth cherry-picking
   to main):
   - `wrap_angle()` used `math.atan2` and crashes on arrays — the 48c36c6
     azimuth merge hits this on any fragment with >1 point (fix: numpy).
   - The fishros port declares params from **uninitialized members**, so
     with any `laser.*` param missing the projection silently drops every
     point (empty clouds, identity odometry). `loam_carla.yaml` now sets the
     full spike parameter set; a run with garbage defaults produces
     "5 messages, 0 points" — check `/segmented_cloud` width first.
   - The port publishes `/integrated_to_init` (not `_map`).
   - `ros2 bag record -o` fails on existing dirs (batch now cleans first),
     and rosbag2 play wants the metadata `files:` block.
   - The registry deps image ships an empty `dist-packages` (pip stack only
     exists inside the long-lived `brightdrive-deps` container);
     `docker/mm-deps.Dockerfile` makes python steps self-contained.
   - The idle-stop guard shuts the VM down ~5 min after a preemption boot
     (stale `last-activity`), before IAP SSH is reachable; a persistent
     heartbeat keeper on first connection is required to ride through.

## Recommendation

1. Adopt `gicp_odom.py` + `--poses-tum --pose-graph` as the mapping
   frontend path (keep KISS as fallback; it is one flag away).
2. Fix the yaw anchor systematic first — it caps every candidate.
3. Do not invest further in LeGO-LOAM for this rig.

## Reproduce

```bash
# on the VM (research tree /ws/brightdrive-mm, branch research/mapping-methods)
sudo bash /ws/brightdrive-mm/scripts/mapping_methods_eval.sh \
  /ws/bags/map_full4 /ws/bags/map_full4_eval \
  -164.732162,-95.141876,0.300000,-179.913696          # full pipeline
sudo bash /ws/brightdrive-mm/scripts/mm_lego_rerun.sh  # lego variants + eval
```

Artifacts on the VM: `/ws/research/mapping-methods/out/` (bundles, eval JSON,
`comparison.{md,json}`); images mirrored under
`docs/research/mapping-methods-results/` in this branch.
