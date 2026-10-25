# BrightDrive: CARLA localization, mapping, and CI/CD

BrightDrive builds a reusable point-cloud map of Town05 in CARLA 0.9.15, localizes a
vehicle against that map on ROS 2 Humble using LiDAR + GNSS + IMU, and exposes the pose,
covariance, and health state. The laptop is the control plane; a spot GCP VM is the
execution plane (CARLA server + Dockerized ROS workspace). There is no public SSH and no
Colab dependency.

## Status

| Area | State |
|---|---|
| CARLA 0.9.15 + ROS 2 bridge on GCP, IAP-only | working (validated in #18) |
| Town05 full-loop mapping bag (6274 scans, 3.6 km loop) | working |
| Saved map `town05-v14-pg` — lidar pose-graph SLAM (2,283,500 points; ATE 0.037 m RMSE / 0.69 deg yaw; held-out 94 % / 0.120 m; loops 0.113 m median) | **all three gates PASS** (ADR 0002) |
| Localization stack (KISS-ICP + GICP + EKF + health) | working; E2E locked on `town05-v2` |
| OpenSCENARIO S1-S4 harness + evaluator + KPI gates | pending (#10, #11) |
| CI (`lint`, `build-test`, `carla-smoke`, `map-gate`, WIF) | `lint`+`build-test` live on PRs; smoke advisory on `staging`; map-gate advisory (dispatch); full matrix pending (#10, #11) |

Until the scenario harness and evaluator land, this README documents the mapping and
localization path that is complete and reproducible today.

## Architecture

```
laptop (control plane)                  GCP VM europe-west4-c (Spot)
┌────────────────────┐   IAP SSH   ┌───────────────────────────────────────────┐
│ make vm-up/down    │ ──────────> │ carla-server (CARLA 0.9.15, Town05)       │
│ scripts/vm-run.sh  │             │   └─ carla_ros_bridge (sync, 10 Hz)       │
│ git archive HEAD   │             │ brightdrive-deps (ROS 2 Humble)           │
└────────────────────┘             │   /carla/hero/{lidar,gnss,imu}            │
                                   │   KISS-ICP  -> /kiss/odometry              │
                                   │   robot_localization -> /odometry/filtered │
                                   │   localization_core -> map->odom, pose     │
                                   └───────────────────────────────────────────┘
```

Only committed `HEAD` is deployed. `scripts/vm-run.sh` ships `git archive` to
`/ws/brightdrive` and runs commands inside `brightdrive-deps`; long jobs run detached
under `/ws/jobs/<job-id>/` and are followed with `--logs`.

## Prerequisites

- `gcloud` authenticated to project `project-e038f785-ff95-4654-ba6` with the IAM roles in
  `infra/provision.sh` (`compute.instanceAdmin.v1`, `iap.tunnelResourceAccessor`).
- Docker with NVIDIA Container Toolkit on the VM (provisioned by `infra/startup.sh`).
- Images pinned in `docker/deps.lock`:
  `carlasim/carla:0.9.15@sha256:1af47c31…` and
  `ghcr.io/iaminfadel/brightdrive-deps@sha256:263332dd…`.

## Quick start

```bash
make vm-up                       # start the VM, wait for IAP SSH
make vm-status                   # heartbeat, containers, GPU
make map-pull                    # download town05-v2 from GCS + verify hashes
make map-install                 # runtime install into /ws/maps/current/town05-v2
```

Start (or verify) the live stack and localization inside the deps container:

```bash
./scripts/vm-run.sh 'docker exec -d brightdrive-deps bash -lc "
  source /opt/brightdrive/setup.bash; source /ws/install/setup.bash;
  ros2 launch brightdrive_bringup localization.launch.py"'

./scripts/vm-run.sh --logs <job-id>
```

The map loader logs `MAP_LOAD_COMPLETE map_id=town05-v2 load_ms=...` and the estimator
logs `LOCKED ... rmse=... inliers=...`. Health is on `/localization/status`
(OK=0, DEGRADED=1, FAILED=2).

`make vm-down` stops the VM; the idle-stop guard stops it after 30 minutes without an SSH
session or heartbeat.

## Mapping pipeline (lidar SLAM, canonical)

The saved map is built by offline pose-graph SLAM over a sensor-only bag
(`/carla/hero/{lidar,gnss,imu}`, no ground truth):

- **Odometry edges**: KISS-ICP on full 10 Hz sweeps (the sync bridge makes every
  lidar message one complete rotation; KISS diverges on per-tick fragments).
- **GNSS factors**: antenna ENU positions with the antenna→lidar lever arm
  modeled in-graph, plus course-over-ground yaw where the vehicle is moving.
- **Loop closures**: ICP scan-to-submap registrations between same-heading
  revisits, each **vetoed against the GNSS-implied relative pose** before it
  enters the graph (mis-locks on repeated geometry otherwise drag the solution
  and hide their own residual).
- Solved by sparse LM/Gauss-Newton (`scripts/pose_graph.py`, SE(2)+z, Huber).

```bash
make map-record               # record the mapping bag + separate eval bag
make map                      # pose-graph build (detached, heartbeated; ~1 h)
make map-eval                 # all three gates: held-out GICP, ATE vs GT, loops
```

`make map-gnss-imu` builds a GNSS+IMU-only map (no lidar). It is a diagnostic
baseline: it passes the gates only because the CARLA rig's GNSS/IMU are
noise-free, so it is never the shipping path. For preemption-resilient long
builds, `build_map.py --pg-dump/--pg-load/--pg-optimized` checkpoints the
graph stages (the dump stores antenna positions; lever and yaw factors are
rebuilt from the bag on load).

Gates (map bundle must pass all three): held-out GICP ≥80% accepted and median
RMSE ≤0.20 m; ATE abs RMSE ≤0.30 m, p95 ≤0.60 m, yaw RMSE ≤2.0°; loop
consistency ≥10 pairs, median correction ≤0.15 m, p95 ≤0.40 m.

## Saved map `town05-v2`

Town05, single full loop, 3635 m trajectory, 0.25 m voxels, 50 m sensor range. The map
frame is the local ENU frame anchored at the first GNSS fix. Poses use GNSS absolute
positions and GNSS course yaw (`pose_source: gnss_positions_course_yaw`); KISS-ICP and
IMU provide the fallbacks.

| Gate | Result |
|---|---|
| Loop closure | **1.840 m** (z 0.001 m) |
| Held-out GICP (20 scans, excluded from the map) | **18/20 accepted (90 %)**, median RMSE **0.1175 m**, p95 0.1296 m |
| T_map_carla held-out alignment | 0.1084 m (threshold 0.5 m) |
| Map load | 5.7 s (gate 60 s) |

Artifacts:

- `maps/town05-v2/{map.yaml,manifest.json}`: committed runtime metadata and provenance.
- `docs/results/map/town05-v2/`: overlay, held-out selection, consistency JSON, coverage.
- GCS: `gs://brightskies-artifacts-330586358676/maps/town05-v2/` (full bundle, 35 MB).

Bundle layout (contract #15): `map.pcd`, `map.yaml`, `map_eval.yaml`, `trajectory.tum`,
`manifest.json`, `validation/heldout/` (scans + poses), `validation/heldout.yaml`,
`validation/coverage.json`, `validation/map_consistency.json`, `validation/overlay.png`.
The runtime install deliberately contains only `map.pcd`, `map.yaml`, and
`trajectory.tum`; `map_eval.yaml` and `validation/` never reach the estimator.

### Rebuild the map

```bash
# 1. record a sensor-only bag while driving one loop (hero and sensors must exist)
./scripts/vm-run.sh 'docker exec -d brightdrive-deps bash -lc "
  source /opt/brightdrive/setup.bash; source /ws/install/setup.bash;
  /ws/brightdrive/scripts/map_run.sh /ws/bags/map_full2 900"'

# 2. build, check held-out scans, verify the bundle
make map                         # build_map.py --gnss-pose --gnss-yaw --imu-delta
make map-check                   # production GICP, 20 held-out scans
make map-verify                  # hashes + contract metadata
make map-install                 # copy the runtime subset to /ws/maps/current
make map-upload                  # push the bundle to GCS
```

`make map` also honours `MAP_ID`, `MAP_BAG`, `MAP_SPAWN`, `MAP_BUILDS`, and `MAP_BUCKET`.

## Localization

- `/kiss/odometry` (KISS-ICP, 10 Hz) is the only `odom -> base_link` source.
- `/localization/gnss_pose` publishes the GNSS/ENU pose at GNSS rate.
- `/localization/scan_pose` publishes accepted GICP scan-to-map poses at ~5 Hz.
- `/odometry/filtered` and `map -> odom` come from stock `robot_localization`.
- `/localization/status` and `/localization/scan_quality` expose health, RMSE, inliers,
  Hessian condition, covariance trace, innovation d², and latency.
- Scans older than 300 ms are dropped; cold start tries 7 yaw starts (±45°, 15° steps)
  and relocalization tries 7 starts (±30°, 10° steps) around the last good yaw.
- Covariance is `C = sigma^2 H^-1`; DEGRADED inflates it.

Estimator inputs are the four allowed sensors only. `/carla/hero/odometry`,
`/carla/hero/vehicle_status`, `/carla/hero/objects`, and GT transforms are evaluation-only
and never subscribed.

## GCP runtime

- VM: `brightskies-vm`, `n1-standard-8` + Tesla T4 Spot, `europe-west4-c`, 100 GB disk.
  Spot with a 120 s preemption notice; the validation/preemption handlers refresh
  `/var/lib/brightskies/heartbeat`.
- Networking: no public SSH; IAP TCP forwarding only. Foxglove:
  `gcloud compute ssh brightskies-vm --zone europe-west4-c --tunnel-through-iap -- -L 8765:localhost:8765`
  then open `ws://localhost:8765` with `config/foxglove/validation_layout.json`.
- Cost controls: budget alerts at 35 CAD and a 140 CAD cap, `--max-run-duration=4h` on
  CI-started VMs, and the 30-minute idle stop. Bags in
  `gs://brightskies-artifacts-330586358676/bags/` expire after 14 days; maps, runs, and
  evidence are kept.
- No fallback: if the GCP/CARLA path fails, the run stops and escalates; there is no
  Colab or substitute execution plane.

## CI

GitHub Actions authenticates with Workload Identity Federation
(`.github/workflows/wif-smoke.yml`); no service-account JSON keys exist in the repo or
secrets. The full branch model, gate tiers, and the advisory→required ratchet live in
`docs/ci.md` (decision record: `docs/adr/0001-branch-model-and-ci-gates.md`). In short:

- `ci.yml` on every PR: `lint` (pre-commit: clang-format 14, ruff, shellcheck) and
  `build-test` (pinned deps image: `colcon build`, gtest + interface launch test,
  evaluator self-tests, isolation audit; evidence uploaded on failure too).
- `carla-smoke.yml` on `staging` pushes + dispatch: merged staging state on the VM
  through `vm_job.py` (advisory until proven, per the ratchet).
- `carla-full.yml` dispatch-only: S1-S4 matrix, KPI gates, evidence bundle, one
  infrastructure-only retry after `PREEMPTED`.

Bootstrap: `make ci-bootstrap` (squash-merge policy, `staging` branch, branch
protection with the required checks; needs the repo to be public or GitHub
Pro).

## Repository layout

```
src/brightdrive_localization/   map loader, GICP matcher, health, geodesy, map_check
src/brightdrive_bringup/        launch, config, URDF, IMU relay
scripts/build_map.py            offline mapping (KISS-ICP, GNSS poses, bundle writer)
scripts/map_verify.py           #15 metadata and hash verification
scripts/map_run.sh              teleport, record a sensor-only bag, drive one loop
scripts/carla_route_drive.py    pure-pursuit driver with a 1 m precision stop
scripts/vm-run.sh               deploy HEAD and run commands over IAP
infra/                          provision, VM lifecycle, idle stop, IAP SSH
docs/results/map/town05-v2/     mapping evidence
```

## Known limitations

- The bridge fast-forwards faster than real time when uncontrolled; under heavy CPU load
  the estimator can starve its sim-time freshness gates. KPI runs tick the world at a
  controlled rate.
- The live stack publishes IMU at ~10 Hz in synchronous mode (contract asks for 100 Hz).
- Held-out selection is arc-length stratified (`same_bag_arc_length`), not yet grouped by
  the S1-S4 scenario routes.
- The scenario harness, KPI evaluator, forced-fail control, and live CI workflows are the
  remaining implementation work.
