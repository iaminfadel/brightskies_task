# ADR 0003: Tiled map server with full-map fallback

- Status: accepted (2026-10-26)
- Context: BrightDrive task §1 (mapping/localization), wayfinder map #1, ticket "Design the tiled map_server with a full-map fallback" (#27)

## Decision

`brightdrive_map_server` is a C++ ROS 2 node (ament_cmake) that reuses `brightdrive_localization::read_pcd_xyz` for PCD I/O and serves the saved map on `/localization/map_cloud` (`sensor_msgs/PointCloud2`, transient-local QoS, depth 1). The builder's output is unchanged: `map.pcd` + `map.yaml` for the full map, plus a `tiles/` directory of `tile_{ix}_{iy}.pcd` and a `map_index.yaml` describing a regular 2D X/Y grid (full Z range, voxel-downsampled). Tile lookup is O(1) floor division against the index origin and tile size.

Serving modes, selected by the `serving.mode` parameter (compose-overridable):

- `full`: publish the entire `map.pcd` once, latched. This is the fallback and the default for degraded/unknown pose.
- `windowed`: keep the set of tiles whose centres are within `R_load = lidar_range_m + margin_m` of the current pose; load in a background thread; merge and publish only when the set changes; evict tiles beyond `R_evict = R_load + hysteresis_m`.
- `auto` (default): seed the pose from `initial_pose` (scenario spawn / GNSS), publish the full map immediately as a transient-local fallback, then switch to the windowed set once a pose source is available.

The pose source is `/localization/pose` when present, otherwise the seeded pose; the initial seed breaks the map_server↔localization startup loop. Localization no longer loads `map.dir` in process: it subscribes to `/localization/map_cloud` and builds its matcher from the first cloud, failing if none arrives within `map.cloud_timeout_s`. All knobs (tile size, margins, hysteresis, downsample voxel, publish rate, mode, timeouts) are YAML parameters with compose env overrides; mapper scripts are untouched.

## Consequences

- Compose's `map-server` service keeps its name and `/localization/map_cloud` contract; only its command changes from `publish_map_cloud.py` to `ros2 run brightdrive_map_server map_server`.
- Windowed serving is demoable in Foxglove by driving and watching the cloud swap; full-map fallback keeps localization working if tiling stalls.
- #28 (image publish) and #30 (docs) can proceed once this contract is locked; no new tickets surfaced.
