# GCP runtime validation (ticket #18)

Date: 2026-10-22. VM: `brightskies-vm` (`n1-standard-8` + Tesla T4, `europe-west4-c`,
Spot). Result: **PASS** — the go/no-go criteria are met.

## Environment

- NVIDIA driver 580.178.04, CUDA 13.0, Tesla T4 (Turing, 16 GB).
- `vulkaninfo` OK: Vulkan API 1.4.312 (`apiVersion = 4211000`).
- Docker 29.1.3, NVIDIA Container Toolkit; CARLA image
  `carlasim/carla:0.9.15@sha256:1af47c31…`; deps image
  `ghcr.io/iaminfadel/brightdrive-deps@sha256:263332dd…` (`docker/deps.lock`).

## Validated

- CARLA 0.9.15 headless (`-RenderOffScreen -quality-level=Low -ResX=1280 -ResY=720`).
- ROS 2 Humble bridge (`3ce0be8d…`) driving `Carla/Maps/Town01`; LiDAR ~15 Hz,
  GNSS, IMU, TF, `/clock`.
- KISS-ICP (`/kiss/odometry`, ~10–13 Hz) + `robot_localization` EKF
  (`/odometry/filtered`) + `/localization/status`.
- Visualization camera (`camera_front`, 1280×720) is not subscribed by any
  estimator node (audit: `logs/node_info_*.txt`); it is recorded in a separate
  visualization bag, never in the estimator bag.
- Foxglove over IAP: `ws://localhost:8765` through
  `gcloud compute ssh … --tunnel-through-iap`; camera, point cloud (KISS
  keypoints), TF, plots, and status panels.
- Headless RViz screenshot under `xvfb-run` (`rviz_map_trajectory.png`).

## Evidence (VM `/ws/evidence`, mirrored where noted)

| Artifact | Path |
|---|---|
| RGB frame + LiDAR scan (same run) | `smoke/rgb_frame.png`, `smoke/lidar_scan.pcd` |
| Independent Python-API pair | `pair/rgb_frame.png` (1280×720), `pair/lidar_scan.pcd` (10,714 pts) |
| ROS LiDAR scan (same run) | `lidar_scan_ros.pcd` |
| Bags (est/visual split) | `bags/estimator` 67 MB, `bags/visualization` 1.4 GB |
| RViz screenshot | `rviz_map_trajectory.png` |
| Timing summary | `timings.json` (validation total 104 s warm) |
| Isolation audit | `logs/node_info_*.txt`, `logs/camera_topic_info.txt` |
| Preemption result | `preempt/preempt_result.json` |
| Preemption retry | `retry_observation.json` |
| Host info (vulkan/nvidia) | `host_info.txt` → `gs://…/validation/host_info.txt` |
| Foxglove layout | `config/foxglove/validation_layout.json` |

## Preemption drill

- Simulated 120 s notice on the running VM: partial evidence (3.2 MB, bags
  excluded) packaged, uploaded to
  `gs://brightskies-artifacts-330586358676/bags/20261022T190804Z/partial-evidence.tar.gz`,
  exit 44; total handler time 12.7 s.
- Retry on a fresh boot: `RETRY_OK` (new `boot_id`, both pinned images present).
- Bug fixed during this drill: the partial-evidence packager matched
  `/bags/` with a leading slash, so top-level `bags/` was not excluded and the
  tarball would have been ~1.5 GB.

## Cold-start timing (finding)

- Warm path (`KEEP_CARLA=1`): CARLA RPC ready in 0.1 s; full validation 104 s.
- Cold path (VM start → CARLA RPC ready) observed across three boots:
  ready by ~6.5 min, not ready at 600 s (ready by ~10.5 min), and not ready at
  1500 s (ready by ~25.5 min). The **#12 budget of ≤90 s is not met** for a cold
  CARLA boot on this VM; cold starts are also highly variable.
- Recommendation for #12: keep the CARLA server warm across steps (containers
  preserved between jobs or a sidecar that preloads), use a ≥30 min first-run
  timeout, or revisit the 90 s budget with this data.

## Other findings

- The IAP tunnel throughput improves materially once NumPy is available to the
  GCP SDK Python; the default 2.7 MB/s raw camera stream overran it
  (`outbox full`). A 320×240 `/carla/hero/camera_live/image` (~0.6 MB/s) is used
  for the live Foxglove view.
- A fresh CARLA client can see an empty `get_actors()` list for a few seconds
  after connecting; all actor lookups now retry (`wait_for_tick`).
- Sensors attached to the hero are namespaced `/carla/hero/<role>`; an
  unattached camera with the same role appears under `/carla/<role>`.
- T4 showed `SW Power Cap` throttling at ~47 % utilization, 77 °C during long
  renders.
- `measure_vm_start.sh` originally aborted without writing timings when CARLA
  missed its deadline; the deadline is now 1500 s.
- Laptop `gcloud`: a user-site `protobuf 4.25.9` (`~/.local/lib/python3.14`)
  shadows the SDK's vendored protobuf (the SDK bundled Python is also 3.14) and
  crashes proto commands with `Metaclasses with custom tp_new are not
  supported`. `infra/common.sh` now exports `PYTHONNOUSERSITE=1`; the IAP
  tunnel itself was unaffected.
- The idle-stop guard powered the VM down at 20:17:22Z while the validation
  stack was meant to be left running for Foxglove (`journalctl` shows the
  idle-stop service run followed by "System is powering down"; no API stop
  event). All three activity signals were quiet by then: the heartbeat is only
  touched at boot, the IAP tunnel's SSH sessions had timed out, and CARLA's
  GPU utilization reads low when nothing drives the world. The guard behaved
  as designed on an effectively idle VM, but "leave the stack running" is not
  safe for unattended runs: `run_validation.sh` now refreshes
  `/var/lib/brightskies/heartbeat` every 2 minutes during a run, and a connected
  Foxglove viewer holds an established port-22 session that keeps the VM alive
  while watched. The #19 orchestrator must heartbeat for the whole job.

## Go/no-go

- RGB frame + LiDAR scan + Foxglove over IAP + preemption handler drill: **all
  pass**. No L4 retry and no `-nullrhi` fallback needed.

## Closure

- Golden snapshot `brightskies-golden-20261022`: **READY**, source disk
  `brightskies-vm`, 100 GB, `europe-west4-c` (taken after validation, with the
  CARLA and deps images already pulled).
- Repo: branch `prototype/gcp-runtime-validation`; evidence on the VM under
  `/ws/evidence`, partial uploads under
  `gs://brightskies-artifacts-330586358676/validation/`.
- The final cold-start timing number is documented qualitatively above; a
  fourth instrumented cold boot can be run on request (VM restart required).
