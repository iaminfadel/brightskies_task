# BrightDrive: CARLA localization, mapping, and CI/CD

BrightDrive builds a reusable point-cloud map of Town05 in CARLA 0.9.15, localizes a
vehicle against that map on ROS 2 Humble using LiDAR + GNSS + IMU, and exposes the pose,
covariance, and health state. The laptop is the control plane; a spot GCP VM is the
execution plane (CARLA server + a Dockerized ROS workspace). There is no public SSH and no
Colab dependency.

## Status

| Area | State |
|---|---|
| CARLA 0.9.15 + ROS 2 bridge on GCP, IAP-only | working (validated in #18) |
| Town05 full-loop mapping bag (6274 scans, 3.6 km loop) | working |
| Saved map `town05-v14-pg` — lidar pose-graph SLAM (2,283,500 points; ATE 0.037 m RMSE / 0.69 deg yaw; held-out 94 %) | **gates PASS** (ADR 0002) |
| Localization stack (KISS-ICP + GICP + EKF + health) | working; E2E locked on `town05-v14-pg` |
| Tiled map serving with full-map fallback | working (ADR 0003) |
| OpenSCENARIO S1-S4 harness, evaluator, KPI gates | working (#37); full matrix pending (#38) |
| CI (`lint`, `build-test`, `carla-smoke`, WIF) | `lint`+`build-test` required on PRs; `carla-smoke` advisory on `staging` |

## Architecture

```
laptop (control plane)                  GCP VM europe-west4 (Spot)
┌────────────────────┐   IAP SSH   ┌───────────────────────────────────────────┐
│ make vm-up/down    │ ──────────> │ compose profile "runtime":                │
│ scripts/vm-run.sh  │             │   carla (Town05, 0.9.15)                  │
│ git archive HEAD   │             │   bridge (sync, 10 Hz, spawns the hero)   │
└────────────────────┘             │   /carla/hero/{lidar,gnss,imu}            │
                                   │   KISS-ICP -> /kiss/odometry              │
                                   │   robot_localization -> /odometry/filtered│
                                   │   localization_core -> map->odom, pose    │
                                   │   map_server -> /localization/map_cloud   │
                                   │   foxglove_bridge (loopback 8765)         │
                                   └───────────────────────────────────────────┘
```

One runtime, `docker-compose.yml`, selected by profiles: `runtime` (`make up`), `test`,
`scenario`, `eval`, `mapping`. Only committed `HEAD` is deployed: `scripts/vm-run.sh`
ships `git archive` to `/ws/brightdrive`; long jobs run detached under
`/ws/jobs/<job-id>/` and are followed with `--logs`.

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
make build                       # colcon build into /ws/install (pinned deps image)
make map-pull                    # download town05-v14-pg from GCS
make map-install                 # install the runtime subset into /ws/maps/current
make up                          # carla + bridge + localization + map-server + foxglove
make vm-tunnel                   # forward ws://localhost:8765 to the laptop
```

Open Foxglove Studio on `ws://localhost:8765` with `config/foxglove/validation_layout.json`.
The map server logs the tile grid and the served window; the estimator logs
`LOCKED ... rmse=... inliers=...`. Health is on `/localization/status` (OK=0, DEGRADED=1,
FAILED=2).

`make test` adds the one-shot smoke over the running stack (sensor surface, topic rates,
GT-isolation audit, bags, timings) and writes evidence to `$(EVIDENCE)`.
`make down` stops the stack; the idle-stop guard stops the VM after 30 minutes without an
SSH session or heartbeat.

Verification is the evaluator path, not a binary in the ROS packages: `make map-eval-ate`
scores a recorded trajectory against ground truth, and `make eval-kpi` writes and gates the
run KPIs (`eval_ate.py`, `eval_kpi.py`, `kpi_gate.py`). Both consume bags, so they run
unchanged on a scenario run or a recording.

## Mapping pipeline

The saved map is built by offline pose-graph SLAM over a sensor-only bag
(`/carla/hero/{lidar,gnss,imu}`, no ground truth):

- **Odometry edges**: KISS-ICP on full 10 Hz sweeps (the sync bridge makes every lidar
  message one complete rotation; KISS diverges on per-tick fragments).
- **GNSS factors**: antenna ENU positions with the antenna→lidar lever arm modeled
  in-graph, plus course-over-ground yaw where the vehicle is moving.
- **Loop closures**: ICP scan-to-submap registrations between same-heading revisits, each
  **vetoed against the GNSS-implied relative pose** before it enters the graph.
- Solved by sparse LM/Gauss-Newton (`scripts/pose_graph.py`, SE(2)+z, Huber).

```bash
make map                         # recorder (one lap) -> map-builder -> map + tiles
make map-build                   # builder only, from an existing bag
make map-check                   # held-out GICP consistency gate (C++ matcher)
make map-eval-ate                # ATE vs ground truth
```

`build_map.py --pg-dump/--pg-load/--pg-optimized` checkpoints the graph stages for
preemption-resilient rebuilds (the dump stores antenna positions; lever and yaw factors are
rebuilt from the bag on load).

Gates on the bundle: held-out GICP ≥80 % accepted and median RMSE ≤0.20 m; ATE abs RMSE
≤0.30 m, p95 ≤0.60 m, yaw RMSE ≤2.0°. Loop-consistency evidence for the pinned bundle is
archived in `docs/results/map/town05-v14-pg/`.

## Saved map `town05-v14-pg`

Town05, single full loop, 3731 m trajectory, 0.25 m voxels, 50 m sensor range, 2,283,500
points. The map frame is the local ENU frame anchored at the first GNSS fix.

| Gate | Result |
|---|---|
| ATE vs ground truth | **0.037 m RMSE** (p95 0.080 m), yaw **0.69 deg RMSE** |
| Held-out GICP (100 scans, excluded from the map) | **94/100 accepted**, median RMSE **0.120 m** |
| Loop consistency (52 revisit pairs, archived) | median residual **0.113 m** |
| T_map_carla held-out alignment | 0.115 m (threshold 0.5 m) |

Artifacts live in `docs/results/map/town05-v14-pg/` and in
`gs://brightskies-artifacts-330586358676/maps/town05-v14-pg/`.

Bundle layout: `map.pcd`, `map.yaml`, `map_eval.yaml`, `trajectory.tum`, `manifest.json`,
`tiles/` + `map_index.yaml`, `validation/heldout/`, `validation/coverage.json`,
`validation/map_consistency.json`. The runtime install deliberately contains only
`map.pcd`, `map.yaml`, and `trajectory.tum` — the map server tiles `map.pcd` itself.

## Localization

- `/kiss/odometry` (KISS-ICP, 10 Hz) is the only `odom -> base_link` source.
- `/localization/gnss_pose` publishes the GNSS/ENU pose at GNSS rate.
- `/localization/scan_pose` publishes accepted GICP scan-to-map poses at ~5 Hz.
- `/odometry/filtered` and `map -> odom` come from stock `robot_localization`.
- `/localization/status` and `/localization/scan_quality` expose health, RMSE, inliers,
  Hessian condition, covariance trace, innovation d², and latency.
- Scans older than 300 ms are dropped; cold start tries 7 yaw starts (±45°), relocalization
  tries 7 starts (±30°) around the last good yaw.
- Covariance is `C = sigma^2 H^-1`; DEGRADED inflates it.

Estimator inputs are the four allowed sensors only. `/carla/hero/odometry`,
`/carla/hero/vehicle_status`, `/carla/hero/objects`, and GT transforms are evaluation-only
and never subscribed (`scripts/isolation_audit.py` enforces the source side).

## Scenarios and KPIs

```bash
make scenario S=S2               # locked OpenSCENARIO case + estimator/GT bags
make eval                        # GT-isolation audit + KPI gate over the artifacts
make eval-kpi                    # heading / output-age / latency-drift metrics + gate
```

`scripts/eval_kpi.py` scores heading error, `/odometry/filtered` message age, and latency
drift from the estimator + GT bags; `scripts/kpi_gate.py` applies the thresholds and
`scripts/forced_fail_control.py` proves the gate can fail.

## GCP runtime

- VM: `brightdrive-vm`, `n1-standard-8` + Tesla T4 Spot, 100 GB disk, 120 s preemption
  notice. Long jobs refresh `/var/lib/brightskies/heartbeat`; `scripts/vm_job.py` retries
  once on exit 44.
- Networking: no public SSH; IAP TCP forwarding only. Foxglove is loopback-only.
- Cost controls: budget alerts, `--max-run-duration=4h` on CI-started VMs, 30-minute idle
  stop. Bags in GCS expire after 14 days; maps, runs, and evidence are kept.

## CI

GitHub Actions authenticates with Workload Identity Federation; no service-account JSON
keys exist in the repo or secrets. Branch model, gate tiers, and the advisory→required
ratchet live in `docs/ci.md` (decision record: `docs/adr/0001-branch-model-and-ci-gates.md`).

- `ci.yml` on every PR: `lint` (pre-commit: clang-format 14, ruff, shellcheck) and
  `build-test` (pinned deps image: `colcon build` for all four packages, gtest + launch
  tests, evaluator self-tests, scenario harness tests, isolation audit; evidence uploaded
  on failure too).
- `carla-smoke.yml` on `staging` pushes + dispatch: `run_on_vm.sh` builds the workspace,
  runs the compose `test` route on the VM and collects `/ws/evidence` (advisory until
  proven, per the ratchet).
- `carla-full.yml` dispatch-only: the S1-S4 matrix and KPI gates.

## Repository layout

```
src/brightdrive_localization/   estimator: GICP matcher, health, geodesy, SE(3)
src/brightdrive_map_server/     map bundle PCD I/O, tiled serving with full-map fallback
src/brightdrive_bringup/        launch, config, URDF, GNSS/IMU relays
src/brightdrive_interfaces/     LocalizationStatus message
scripts/build_map.py            offline mapping (KISS-ICP, GNSS poses, bundle writer)
scripts/pose_graph.py           SE(2)+z pose-graph solver + ICP loop closures
scripts/eval_ate.py             ATE ground-truth evaluator
scripts/eval_kpi.py             heading / output-age / latency-drift evaluator
scripts/kpi_gate.py             KPI thresholds and gate
scripts/scenario/               ScenarioRunner harness, route coverage, GNSS profiles
scripts/ros_exec.sh             one-off ROS command in a deps container
scripts/ensure_py_deps.sh       top up rosbags/scipy on the pinned deps image
scripts/vm-run.sh               deploy HEAD and run commands over IAP
infra/                          provision, VM lifecycle, idle stop, IAP SSH
```

## Known limitations

- The bridge fast-forwards faster than real time when uncontrolled; under heavy CPU load
  the estimator can starve its sim-time freshness gates. KPI runs tick the world at a
  controlled rate.
- The live stack publishes IMU at ~10 Hz in synchronous mode (contract asks for 100 Hz).
- KPI numbers are only meaningful on the VM. GICP costs ~1.07 s/scan on a laptop against the
  ~200 ms the 5 Hz pipeline (`scan_stride: 2`) is sized for, so a laptop replay starves the
  single-threaded executor in `localization_core`: `/localization/status` drops well below
  10 Hz, `gnss_age` grows, and health walks OK -> DEGRADED -> FAILED on CPU starvation rather
  than on localization quality.
- The GNSS bootstrap can accept a 180-deg-flipped yaw for stretches and the matcher keeps
  that lock instead of rejecting it. This is a robustness question for the VM runs.
- Held-out selection is arc-length stratified (`same_bag_arc_length`), not yet grouped by
  the S1-S4 scenario routes.
- The full S1-S4 acceptance matrix is not yet a required gate (#38).
