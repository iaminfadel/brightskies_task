# LeGO-LOAM on ROS 2 Humble with CARLA 0.9.15

## Verdict

**Conditionally viable, medium risk.** The Humble port builds and runs with CARLA's XYZI cloud; the unresolved risk is map quality on CARLA's flat, dynamic towns. A one-day spike is worth funding as a bounded go/no-go, not as a commitment to the 7-day plan.

Scope: mapping only, not localization. CARLA 0.9.15, ROS 2 Humble, one saved map from a scripted route.

## Evidence snapshot

| Project | Stars / forks | Last commit | ROS 2 status | Notes |
| --- | ---: | --- | --- | --- |
| `fishros/LeGO-LOAM-ROS2` | 99 / 18 | 2024-03-30 | Humble port | 8 open issues/PRs; no CI; fork of `eperdices/LeGO-LOAM-SR` |
| `eperdices/LeGO-LOAM-SR` | 60 / 18 | 2020-09-23 | Dashing | Source of the Humble port; evenly distributed channels only |
| `RobustFieldAutonomyLab/LeGO-LOAM` | 2,759 / 1,164 | 2020-07-02 | ROS 1 | Original; 40 open issues; designed for VLP-16 |
| `PRBonn/kiss-icp` | 2,355 / 460 | 2026-05-04 | Humble, Jazzy, Kilted, Rolling CI | Odometry only; no loop closure |
| `rsasaki0109/lidar_slam_ros2` | 863 / 174 | 2026-10-19 | Humble, Jazzy CI | Graph SLAM, map bundle; default workflow is LiDAR-inertial |

The Humble port is a migration, not a maintained SLAM project. Its README says it only changes Dashing syntax, does not improve the algorithm, and removes the IMU ([README](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/README.md)). It has no GitHub Actions runs. Open PR #10 (since 2026-01-01) fixes the launch remap ([PR #10](https://github.com/fishros/LeGO-LOAM-ROS2/pull/10)); issue #2 reports the missing `/registered_cloud` topic ([issue #2](https://github.com/fishros/LeGO-LOAM-ROS2/issues/2)); issue #7 reports a compile failure ([issue #7](https://github.com/fishros/LeGO-LOAM-ROS2/issues/7)).

## Build and dependency viability

| Check | Result |
| --- | --- |
| Humble build | Passed in `ros:humble-ros-base` with apt `ros-humble-gtsam` 4.2.0 |
| Dependency layer | ~11.6 min on 8 vCPU |
| `colcon build` | 983.5 s (~16.4 min) for `lego_loam_sr` |
| Node startup | Four nodes start and stay up |
| CARLA-like XYZI test | 32 x 500 cloud at 20 Hz: `/full_cloud_projected` retained 16,000/16,000 points; `/segmented_cloud` 4,200; no crash |
| Docker image | 4.08 GB; build is practical |
| Colab | Not practical for CARLA: 12 h free cap, variable resources, no GPU/display guarantee; usable only for offline bag processing ([FAQ](https://research.google.com/colaboratory/faq.html)) |

Test command:

```bash
docker build -t legoloam-gtsam42 -f docker-legoloam/Dockerfile.gtsam42 .
ros2 run lego_loam_sr lego_loam_sr \
  --ros-args --params-file /ws/install/lego_loam_sr/share/lego_loam_sr/config/loam_config.yaml
```

The README asks for GTSAM 4.0.0-alpha2. That source build did not finish within a 20-minute cap (59% at 943 s), but the apt 4.2 package built and ran. Treat 4.2 as the tested path and alpha2 as unverified, not as a blocker.

Build is therefore not the risk. The remaining work is configuration, map output, and map quality.

## CARLA input contract and adapter

CARLA 0.9.15's native ROS 2 publisher writes only `x`, `y`, `z`, `intensity` ([CarlaLidarPublisher.cpp](https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/ros2/publishers/CarlaLidarPublisher.cpp)). The ROS bridge also publishes XYZI ([lidar.py](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/src/carla_ros_bridge/lidar.py)). There is no `ring` and no per-point `time`.

That matches this port's actual contract. It projects by vertical angle and azimuth, then overwrites intensity ([imageProjection.cpp](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/src/imageProjection.cpp#L185-L216)). The original `useCloudRing=true` path is absent. So no ring/time adapter is needed, but the beam and image parameters must match.

Observed mismatch: the default config is 16 channels, 1,800 columns, `-15..+15`, `ground_scan_index: 7`, `scan_period: 0.1` ([config](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/config/loam_config.yaml)). A common CARLA ROS bridge sample is 32 channels, 320,000 points/s, 20 Hz, `-26.8..+2.0`, range 50 m ([objects.json](https://github.com/carla-simulator/ros-bridge/blob/master/carla_spawn_objects/config/objects.json)). That is 500 points/channel/frame and 0.72 deg horizontal resolution, not 1,800 columns.

| Setting | Bridge-default 32ch | VLP-16 mimic (safer) |
| --- | ---: | ---: |
| CARLA channels | 32 | 16 |
| CARLA FOV | -26.8 / +2.0 | -15 / +15 |
| CARLA points/s | 320,000 | 288,000 |
| CARLA rotation | 20 Hz | 10 Hz |
| `num_vertical_scans` | 32 | 16 |
| `num_horizontal_scans` | 500 | 1,800 |
| `ground_scan_index` | 28 | 7 |
| `vertical_angle_bottom/top` | -26.8 / 2.0 | -15 / 15 |
| `scan_period` | 0.0 | 0.0 |

Notes:

- `scan_period: 0.0` disables the port's deskew interpolation. CARLA measurements are static pictures, so there is no intra-scan motion distortion. The code only multiplies by `_scan_period` when writing the intensity fraction ([featureAssociation.cpp](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/src/featureAssociation.cpp#L205-L207)); verify 0.0 in the spike.
- The port subscribes to `/rslidar_points`, but the launch remaps `/lidar_points` to `/velodyne_points` ([imageProjection.cpp](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/src/imageProjection.cpp#L45-L46), [run.launch.py](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/launch/run.launch.py#L47-L53)). Remap at the CLI: `-r /rslidar_points:=/carla/hero/lidar`.
- CARLA writes points channel-major ([RayCastLidar.cpp](https://github.com/carla-simulator/carla/blob/0.9.15/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/RayCastLidar.cpp)). The port derives sweep phase from azimuth rather than point order, so this is tolerable, but full-sweep endpoints must be checked. If endpoints look wrong, sort by azimuth before publishing.
- Ensure each message is a complete 360 deg sweep. Partial sweeps break the full-scan projection assumption; check CARLA's `rotation_frequency`/FPS setting.
- Set CARLA lidar noise and dropoff rates to zero where supported to avoid random missing points.

## Evidence for and against CARLA

For:

- Schema fits: XYZI only, no ring/time.
- Ground is present in CARLA towns; the algorithm assumes a ground plane.
- The port builds and runs on Humble, and the synthetic 32 x 500 XYZI test retained all input points.
- One user reports using this repo with CARLA and says it was “flawlessly” ([issue #6](https://github.com/fishros/LeGO-LOAM-ROS2/issues/6)); no config or artifacts were provided.
- LeGO-LOAM is lightweight and has known behavior from the team's ROS 1 use.

Against:

- No public CARLA benchmark, demo bag, or trajectory for this port was found.
- The original author's response to a CARLA drift report was: “LeGO-LOAM is only designed for real lidar” ([issue #18](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/18)).
- The port is stale, has no CI, and has unresolved build/topic issues.
- CARLA's flat roads and repetitive buildings are a weak case for a ground-optimized, ICP-loop-closure pipeline.
- No dynamic-object filtering: moving cars and pedestrians become features and map ghosts.
- No map save in the port; `/registered_cloud` is absent. A patch or external accumulation is required.

## Failure modes to watch

1. **Sparse projection:** wrong `num_vertical_scans`, `num_horizontal_scans`, FOV, or `ground_scan_index` silently drops points. Watch `/full_cloud_projected` and `/segmented_cloud` counts.
2. **Feature starvation:** flat/open roads leave mostly ground points; the ground-removal step keeps only a sparse subset, so scan matching can drift or fail.
3. **Dynamic objects:** no filtering; cars create edges and loop-closure targets, causing ghosting and false alignments.
4. **High speed / sharp turns:** no IMU, constant-velocity prediction, and a two-step LM optimizer; at 10 Hz and 50 km/h the vehicle moves ~1.4 m per scan. Prefer 20 Hz and moderate speeds.
5. **Loop closure:** naive ICP with 7 m history radius and fitness 0.3; repetitive places can produce false positives, and `mapOptmization.cpp` requires 30 s between loop candidates ([mapOptmization.cpp](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/src/mapOptmization.cpp#L620)).
6. **Map output:** `/laser_cloud_surround` is only published when subscribed ([mapOptmization.cpp](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/LeGO-LOAM/src/mapOptmization.cpp#L555-L558)); the original save block was removed. Either restore the original `savePCDFile` block ([original](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/blob/896a7a95a8bc510b76819d4cc48707e344bad621/LeGO-LOAM/src/mapOptmization.cpp#L730-L755)) or subscribe and accumulate.

## Comparison

| Pipeline | Humble evidence | Loop closure | Saved map | CARLA fit | Integration risk |
| --- | --- | --- | --- | --- | --- |
| LeGO-LOAM port | Built and launched here | Naive ICP | Needs patch | XYZI fits; behavior unproven | Medium-high |
| KISS-ICP | CI on Humble/Jazzy/Kilted/Rolling | No | No file output; local map topic only | Best schema fit; no tuning | Low, but odometry only |
| lidarslam_ros2 | CI on Humble/Jazzy | Graph-based | Map bundle + PCD | Default IMU; LiDAR-only preset exists | Medium-high |

KISS-ICP's ROS 2 node publishes `kiss/odometry` and `kiss/local_map`, and disables deskew when timestamps are empty ([Preprocessing.cpp](https://github.com/PRBonn/kiss-icp/blob/1ffa7d7512f10bfc8b1185095011fa31184019e3/cpp/kiss_icp/core/Preprocessing.cpp#L58-L59)). It has no loop closure or global PCD save.

`lidarslam_ros2` has a LiDAR-only preset (`use_imu: false`), NDT scan matching, graph-based loop closure, and a map bundle writer ([lidarslam_lo.yaml](https://github.com/rsasaki0109/lidar_slam_ros2/blob/07a74d4f67a9de128bfba0a9ebc8522a64922bf5/lidarslam/param/lidarslam_lo.yaml), [graph_slam_ros_adapter.cpp](https://github.com/rsasaki0109/lidar_slam_ros2/blob/07a74d4f67a9de128bfba0a9ebc8522a64922bf5/graph_based_slam/src/graph_slam_ros_adapter.cpp#L1235-L1238)). It is the maintained option but a larger dependency and integration surface.

## One-day spike

Hard stop at 8 hours. Use one CARLA town, one route, one bag, and one machine.

| Time | Activity | Output |
| --- | --- | --- |
| 0-1 h | Build, remap topic, launch; verify `/segmented_cloud` exists | Running stack |
| 1-3 h | Apply Adapter A or B; verify projection retention and ground cloud | Parameter set |
| 3-5 h | 2-3 min loop at moderate speed; record odometry and save PCD | Trajectory + first map |
| 5-6 h | High speed, 90 deg turns, traffic; inspect drift/ghosting | Stress notes |
| 6-7 h | Run KISS-ICP on the same bag; compare trajectory and effort | Baseline |
| 7-8 h | Go/no-go review | Decision |

Kill criteria, any one stops the spike:

- Cannot build and launch on Humble with GTSAM 4.2 within 90 min.
- 60 s clean-town bag diverges or drifts more than 5% of path length after one tuning pass.
- Projection retains less than 70% of points, or ground cloud is absent after beam/FOV matching.
- At 50 km/h or a 90 deg turn, error exceeds 10% or the run fails and is not fixed within 1 h.
- Dynamic objects cause persistent ghosting after two simple filter attempts.
- No full-route saved PCD by the end of the day.
- Loop closure visibly distorts the map; disabling it does not recover acceptable quality.

Success: one full-route PCD and trajectory; return-to-start under 2 m on a ~200 m loop; at least 90% of points projected; real-time or at least 5 Hz; no crashes.

## Recommendation

Fund the spike if the team values LeGO-LOAM's low compute cost and existing familiarity enough to spend one day testing it. Do not commit the 7-day plan to it yet. If any kill criterion fires, stop and use the same bag to evaluate a maintained alternative. The spike should answer one question: can this port produce a usable CARLA town map with a small, documented patch, or does its ground/loop-closure design fail on simulated data?

## Sources

- [fishros/LeGO-LOAM-ROS2](https://github.com/fishros/LeGO-LOAM-ROS2)
- [Humble port README](https://github.com/fishros/LeGO-LOAM-ROS2/blob/088856332b71e4bc4cc0f08cd8282ac370ca444a/README.md)
- [PR #10](https://github.com/fishros/LeGO-LOAM-ROS2/pull/10), [issue #6](https://github.com/fishros/LeGO-LOAM-ROS2/issues/6), [issue #7](https://github.com/fishros/LeGO-LOAM-ROS2/issues/7), [issue #2](https://github.com/fishros/LeGO-LOAM-ROS2/issues/2)
- [LeGO-LOAM-SR](https://github.com/eperdices/LeGO-LOAM-SR), [original LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM), [original issue #18](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM/issues/18)
- [CARLA 0.9.15 sensor docs](https://github.com/carla-simulator/carla/blob/0.9.15/Docs/ref_sensors.md), [native LiDAR publisher](https://github.com/carla-simulator/carla/blob/0.9.15/LibCarla/source/carla/ros2/publishers/CarlaLidarPublisher.cpp), [raycast LiDAR](https://github.com/carla-simulator/carla/blob/0.9.15/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/RayCastLidar.cpp)
- [CARLA ROS bridge lidar.py](https://github.com/carla-simulator/ros-bridge/blob/master/carla_ros_bridge/src/carla_ros_bridge/lidar.py), [objects.json](https://github.com/carla-simulator/ros-bridge/blob/master/carla_spawn_objects/config/objects.json)
- [KISS-ICP](https://github.com/PRBonn/kiss-icp), [ROS 2 README](https://github.com/PRBonn/kiss-icp/blob/main/ros/README.md)
- [lidarslam_ros2](https://github.com/rsasaki0109/lidar_slam_ros2), [LiDAR-only preset](https://github.com/rsasaki0109/lidar_slam_ros2/blob/07a74d4f67a9de128bfba0a9ebc8522a64922bf5/lidarslam/param/lidarslam_lo.yaml), [LiDAR-only workflow](https://github.com/rsasaki0109/lidar_slam_ros2/blob/07a74d4f67a9de128bfba0a9ebc8522a64922bf5/docs/workflows.md#L270)
- [Colab FAQ](https://research.google.com/colaboratory/faq.html)
