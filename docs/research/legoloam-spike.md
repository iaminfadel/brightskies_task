# LeGO-LOAM spike: Town10HD

Date: 2026-10-22
Ticket: [#14](https://github.com/iaminfadel/brightskies_task/issues/14) (`wayfinder:prototype`)
Status: complete — **no-go for the LeGO-LOAM ROS 2 port; go for KISS-ICP**

## Scope

Test whether the `fishros/LeGO-LOAM-ROS2` Humble port produces a usable map from
CARLA 0.9.15 Town10HD lidar, with KISS-ICP as the fallback baseline. This is a
mapping-frontend spike only; GICP, the EKF, and the saved-map contract are out of
scope (see #9 and #15).

The input is the bridge's `/carla/hero/lidar`: XYZI, 16 channels, 1,800 columns,
-15..+15 deg, 10 Hz, 50 m range, no `ring` and no per-point `time`. The port
projects by vertical angle and azimuth, so no ring/time adapter is needed. The
spike uses `scan_period: 0.1` (pass 1) and `scan_period: 0.0` (pass 2), the
research note's recommendation.

## Result

All numbers are against the 51.3 s moving segment of `gt_full.csv`
(t = 29.35–80.65 s, 133–135 m, mean 31.5 km/h, max 70.5 km/h). The evaluator
uses x/y only; z is reported separately.

| Run | `scan_period` | Input frames | Map raw / 0.1 m | Trajectory poses | x/y path | GT path | x/y path error | ATE RMSE | ATE max | ATE final |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| LeGO pass 1 | 0.1 | 672 | 64,830 / 14,539 | 130 | 10.16 m | 132.93 m | -92.36% | 36.80 m | 107.99 m | 107.99 m |
| LeGO pass 2 | 0.0 | 724 | 255,011 / 78,555 | 143 | 8.25 m | 131.69 m | -93.73% | 37.30 m | 104.69 m | 104.69 m |
| KISS-ICP | n/a | 2,112 | 241,955 | 2,112 | 133.32 m | 134.83 m | -1.12% | 1.33 m | 6.91 m | 5.90 m |

KISS-ICP passes the path-length gate. Its x/y ATE is 1.33 m (0.99% of the GT
path). Its z drifts from 0 to 4.39 m against a GT z of ~0.15 m — a 4.2 m (3.1%)
vertical error that the x/y evaluator does not capture. The map z range is
-0.19..21.48 m (buildings), so the map is planar, not a vertical smear. The z
error matters for the #11 3D KPI gate and must be checked in #15.

LeGO-LOAM fails both passes. The x/y path collapses because the forward motion
appears in z: the final poses are z = 139.71 m (pass 1) and 104.69 m (pass 2),
while the GT z is 0.15 m. The map z ranges are -47.6..53.6 m (pass 1) and
-49.6..181.8 m (pass 2). The maps are vertical smears; increasing density in
pass 2 (255k points) does not make them usable.

## Diagnosis

The port's `MapOptimization::transformUpdate()` at
`LeGO-LOAM/src/mapOptmization.cpp:378` only copies `transformSum` into
`transformTobeMapped`. The original LeGO-LOAM `transformUpdate()` fits the ground
plane from `groundCloud` and corrects z, roll, and pitch. The port's
`imageProjection` still publishes `/ground_cloud`, but `mapOptimization` never
subscribes to it. The z/roll/pitch are therefore unconstrained by the ground, and
the odometry z absorbs the forward motion. This is a port defect, not a CARLA
input-contract problem.

Evidence:

- The raw scan is upright: z range -2.38..12.94 m, ground at -2.38 m (lidar at
  2.4 m). The cloud is not rotated.
- Ground removal works: mean input 21,668 points/frame, segmented 10,425, ground
  10,988 (sum 21,413, 98.8% retained). `/ground_cloud` is present for 652 frames.
  The `projected` width of 28,800 is the full 16 x 1,800 range-image grid, not a
  point count.
- The z drift starts immediately and grows monotonically; pass 1 is less bad than
  pass 2. `scan_period: 0.0` (the research note's recommendation) is worse in this
  port, not better. Use `0.1` if the port is retried.
- A patch that restores the original ground-plane correction is feasible. It was
  not attempted; the decision below does not depend on it.

## Point retention, runtime, and drive quality

- Point retention is not the failure. The port retains 98.8% of input points
  across segmented + ground clouds. The failure is the pose z drift.
- LeGO runtime: pass 1 672 frames in ~145 s wall; pass 2 724 frames in ~165 s
  wall. Both ran the bag at `--rate 2`, CPU-only, with the LeGO node at ~14% CPU.
  Build time was 16.4 min (research note).
- KISS-ICP runtime: 2,112 frames + map save in 177 s, CPU-only. Settings:
  1.3.0, `max_range=50`, `min_range=0`, `voxel_size=0.5`, `deskew=False` (the
  CARLA cloud has no per-point time), zero timestamps passed to `register_frame`.
- Drive: the hero (id 109) was teleported to spawn 12 in Town10HD. Autopilot from
  standstill did not move the car; a manual throttle kickstart (~2.4 s at 0.6
  throttle) followed by `set_autopilot(True, 8001)` produced 51.3 s of motion.
  The car was stationary for the remaining ~129 s of the 180 s drive. The segment
  is 133–135 m with a 25.7 deg yaw change, not the requested 90 deg turn. The
  2–3 min route requirement is only partially met. The KISS-ICP result is still
  valid evidence for the 135 m moving segment.
- GT is from CARLA hero pose (offline CSV), never from the bag; the bag contains
  only `/carla/hero/lidar` plus the clock needed by the player. The bag is
  633.3 MiB, 851.76 s wall, 2,112 lidar messages.

## Coverage gaps

- **Town05:** #10 requires the spike map to cover the full Town05 loop (S1/S2
  urban, S3 turns/straight, S4 signalized intersection, >=20 m lateral coverage).
  This spike ran Town10HD. The Town10HD map covers ~100 m x 85 m and the 135 m
  segment with ample lateral coverage, but it does not satisfy the #10 contract.
  The LeGO no-go decision is independent of the town; the KISS-ICP baseline is
  Town10HD evidence only. #15 must produce and validate the Town05 loop.
- **Held-out consistency:** the #11 gate (20 held-out mapping scans, GICP median
  RMSE <= 0.20 m, >= 80% accepted) was not run. Hand to #15.
- **3D/z:** the x/y evaluator passes KISS-ICP; the 4.2 m z drift is a gap for the
  #11 3D gate. Check in #15.
- **Pipeline:** KISS-ICP here is odometry-only; the #9 `small_gicp` scan-to-map
  and `robot_localization` EKF were not exercised. #15 defines that contract.

## Decision

- **LeGO-LOAM port: no-go.** It fails the >5% path-length drift gate after two
  tuning passes. The z drift makes the map unusable despite high point retention.
- **KISS-ICP: go as the mapping frontend.** It already is the #9 choice; this
  spike confirms the fallback from #8. The 135 m segment passes the x/y drift
  gate at -1.12% path error and 1.33 m ATE RMSE.
- **Town05 and the held-out/3D gates: hand to #15.** #14 is not a compliant
  Town05 map; it is a frontend decision.

## Artifacts

On the throwaway `research/legoloam-spike` branch, under
`docs/research/legoloam-spike/`:

- `legoloam-pass1/`: `map_raw.pcd`, `map_0p1.pcd`, `trajectory.csv`, `eval.json`,
  `point_counts.csv`, `saver.log`, `lego.log`, `play.log`
- `legoloam-pass2/`: same files for `scan_period: 0.0`
- `kiss-icp/`: `kiss_poses.csv`, `kiss_poses_map.pcd`, `eval_kiss.json`, `kiss.log`
- `gt/`: `gt_full.csv`, `drive_meta_full.json`
- `scripts/`: `colab_drive.py`, `debug_spawn12.py`, `spike_objects.json`,
  `inspect_scan.py`, `kiss_icp_run.py`, `map_saver.py`, `eval_traj.py`,
  `run_offline_full.sh`, `run_offline_00.sh`, `loam_carla_full.yaml`,
  `loam_carla_full_00.yaml`

The 633 MB bag is not committed; it lived at
`/var/tmp/legoloam-spike/data/bag` on the host. The Colab session and CARLA server
were ephemeral.

## Reproduce

1. Run LeGO pass 2: mount the bag at `/data/bag`, the scripts at `/spike`, then
   `bash /spike/run_offline_00.sh /data/run_full_00 /spike/loam_carla_full_00.yaml`.
   The player uses `--clock --start-offset 1` to skip the bag's leading 1e9
   sentinel timestamp; `run_offline_00.sh` uses `timeout --signal=INT 140` to stop
   after the moving segment.
2. Run KISS-ICP: `PYTHONPATH=/spike/kisslib python3 /spike/kiss_icp_run.py
   /data/bag /data/run_full/kiss_poses.csv`. KISS-ICP 1.3.0 must be installed;
   `deskew=False` is required because the CARLA cloud has no per-point time.
3. Evaluate: `python3 /spike/eval_traj.py /data/gt_full.csv
   /data/run_full/kiss_poses.csv /data/run_full/eval_kiss.json`. The evaluator
   interpolates at estimated stamps and scores x/y only.

## References

- Research note: `docs/research/legoloam-ros2.md` on `research/legoloam-ros2`
- Port: <https://github.com/fishros/LeGO-LOAM-ROS2>
- Original `transformUpdate` with the ground correction:
  <https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/896a7a95a8bc510b76819d4cc48707e344bad621/LeGO-LOAM/src/mapOptmization.cpp#L730-L755>
- KISS-ICP: <https://github.com/PRBonn/kiss-icp>
- #8 (pipeline choice), #9 (localization contract), #10 (scenario/Town05),
  #11 (KPIs/held-out gate), #15 (saved-map contract)
