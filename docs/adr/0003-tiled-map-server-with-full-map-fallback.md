# ADR 0003: Tiled map server with full-map fallback

- Status: accepted (2026-10-26)
- Context: BrightDrive task §1 (mapping/localization), wayfinder map #1, ticket "Design the tiled map_server with a full-map fallback" (#27); builder half landed in #32

## Decision

`brightdrive_map_server` is a C++ ROS 2 node (ament_cmake) that owns the bundle PCD reader (`brightdrive_map_server::read_pcd_xyz`) and serves the saved map on `/localization/map_cloud` (`sensor_msgs/PointCloud2`, transient-local QoS, depth 1). The builder's output is unchanged: `map.pcd` + `map.yaml` for the full map, plus a `tiles/` directory of `tile_{ix}_{iy}.pcd` and a `map_index.yaml` describing a regular 2D X/Y grid (full Z range, voxel-downsampled). Tile lookup is O(1) floor division against the index origin and tile size.

Serving modes, selected by the `serving.mode` parameter (compose-overridable):

- `full`: publish the entire `map.pcd` once, latched. This is the fallback and the default for degraded/unknown pose.
- `windowed`: keep the set of tiles whose centres are within `R_load = lidar_range_m + margin_m` of the current pose; load in a background thread; merge and publish only when the set changes; evict tiles beyond `R_evict = R_load + hysteresis_m`.
- `auto` (default): seed the pose from `initial_pose` (scenario spawn / GNSS), publish the full map immediately as a transient-local fallback, then switch to the windowed set once a pose source is available.

The pose source is `/localization/pose` when present, otherwise the seeded pose; the initial seed breaks the map_server↔localization startup loop. Localization no longer loads `map.dir` in process: it subscribes to `/localization/map_cloud` and builds its matcher from the first cloud, failing if none arrives within `map.cloud_timeout_s`. All serving knobs (margins, hysteresis, downsample voxel, publish rate, mode, timeouts) are YAML parameters with compose env overrides. The grid itself is not a serving knob: `build_map.py` fixes it with `--tile-size` (default 25 m, #32) and the server reads origin and size from `map_index.yaml`.

## Consequences

- Compose's `map-server` service keeps its name and `/localization/map_cloud` contract; only its command changes from `publish_map_cloud.py` to `ros2 run brightdrive_map_server map_server`.
- Windowed serving is demoable in Foxglove by driving and watching the cloud swap; full-map fallback keeps localization working if tiling stalls.
- A bundle installed without `tiles/` + `map_index.yaml` (the runtime install only guarantees `map.pcd`, `map.yaml`, `trajectory.tum`) logs a warning and degrades to `full` instead of failing at startup. Rebuild with `make map-build` (tiles are on by default) to get windowed serving.
- #28 (image publish) and #30 (docs) can proceed once this contract is locked; no new tickets surfaced.

## Amendment (2026-10-28)

`.PCD` I/O originally lived in `brightdrive_localization` because the estimator
loaded the map in process. Once the estimator subscribed instead, the map
server became the only reader of `map.pcd`, and the reuse above turned into a
backwards package dependency (the server linked the whole estimator core for a
100-line reader). `pcd_io` now lives in `brightdrive_map_server` and that
package no longer depends on `brightdrive_localization`.

The held-out consistency checker (`map_check`) is still part of verification,
but it is not part of the ROS packages: it lives in `tools/map_check` as a
plain CMake executable that compiles the production matcher and PCD reader
directly and installs nothing. Map quality is gated by
`make map-check` (held-out GICP) alongside `make map-eval-ate`, and the
evaluator path over recorded bags (`eval_ate.py`, `eval_kpi.py`,
`kpi_gate.py`) scores the running system. Keeping the check out of `src/`
keeps colcon packages to what the runtime actually launches.
