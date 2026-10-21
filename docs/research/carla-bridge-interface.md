# CARLA 0.9.15 Humble bridge interface

**Issue:** #4 · **Date:** 2026-10-21 · **Scope:** `ttgamage/carla-ros-bridge` @ `3ce0be8`, cross-checked against `AlexanderRex/carla-ros-bridge` @ `c596934`; CARLA 0.9.15; ROS 2 Humble.

## Executive answer

`ttgamage/carla-ros-bridge` is a 2023 Humble port of the dormant upstream `carla-simulator/ros-bridge`, not a new interface. Its LiDAR/GNSS/IMU/odometry/TF code is byte-identical to upstream master (last changed 2022-07-22); only `camera.py` and `CARLA_VERSION` differ. The fork pins CARLA 0.9.15, renames demo topics to `hero`, fixes ScenarioRunner environment handling, and makes Humble/CMake fixes. Use `ttgamage`; `AlexanderRex` is a stale fork with only a README difference and issues disabled. It can supply LiDAR, GNSS, IMU, TF, and odometry, but it is not a complete localization interface:

- LiDAR is `sensor_msgs/PointCloud2` with **XYZI only**; there is no `ring` or per-point `time` (`lidar.py:72-87`).
- GNSS, IMU, and odometry have **no populated covariances**; GNSS has no explicit fix status (`gnss.py:68-73`, `imu.py:67-88`, `odom_sensor.py:64-68`).
- It publishes **ground-truth ego state** via `/carla/hero/odometry`, `/carla/hero/vehicle_status`, and `map -> hero` TF; object and marker topics expose GT actors. Exclude these from estimator inputs.
- `map` is the CARLA world origin, not Earth/ENU; no georeference transform is published. `hero` is the CARLA actor origin at the bottom of the vehicle, not REP-105 `base_link`.
- If downstream requires XYZIRT, `ring` is recoverable exactly by patching `lidar.py`; per-point `time` is not exposed and is at best an approximation.

## Install and Python wheel constraints

The README targets Ubuntu 22.04, ROS 2 Humble, Python 3.10, CARLA 0.9.15, and ScenarioRunner 0.9.15 (`README.md:7-12`). Clone with submodules, then use rosdep and colcon (`README.md:18-33`):

```sh
git clone --recurse-submodules https://github.com/ttgamage/carla-ros-bridge.git
mv carla-ros-bridge src
source /opt/ros/humble/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -r
colcon build --symlink-install
```

The stock CARLA 0.9.15 PythonAPI supports Python 2.7/3.7 (3.8 by extension per README), not Humble's 3.10. This fork relies on third-party CPython 3.10 builds from `gezp/carla_ros`; the wheel is `carla-0.9.15-cp310-cp310-linux_x86_64.whl` (24,520,927 bytes), SHA-256 `d8766616df2511a58ac152ccc53e3307a84ccef9e7fb7108006667188d5766f8` (computed 2026-10-21). Its metadata says `Name: carla`, `Version: 0.9.15`. Vendor and pin it; it is not an official CARLA release.

Two traps: `bridge.py:414-419` requires the installed `carla` distribution to equal exactly `0.9.15` (`CARLA_VERSION`), so a 0.9.16 wheel exits at startup; `install_dependencies.sh:46` installs `qt5-default`, unavailable on Ubuntu 22.04, so use rosdep instead of that helper.

## Topics and frames

The convention is `/carla/<parent role>/<sensor role>` (`pseudo_actor.py:84-102`); the demo parent role is `hero` (`objects.json:24-27`).

### Estimator-visible topics

| Topic | ROS 2 type | Frame | Content and limits |
|---|---|---|---|
| `/carla/hero/lidar` | `sensor_msgs/msg/PointCloud2` | `hero/lidar` | XYZI, 16-byte points; no ring/time |
| `/carla/hero/gnss` | `sensor_msgs/msg/NavSatFix` | `hero/gnss` | latitude, longitude, altitude only |
| `/carla/hero/imu` | `sensor_msgs/msg/Imu` | `hero/imu` | orientation, gyro, accelerometer; no covariance |
| `/carla/world_info` | `carla_msgs/msg/CarlaWorldInfo` | none | map name and OpenDRIVE, transient-local, always published |
| `/carla/map` | `std_msgs/msg/String` | none | OpenDRIVE, transient-local; only if `sensor.pseudo.opendrive_map` is spawned |
| `/clock` | `rosgraph_msgs/msg/Clock` | none | CARLA simulation time |
| `/tf` | `tf2_msgs/msg/TFMessage` | `map`/`hero`/sensor | contains GT ego TF if `sensor.pseudo.tf` is spawned |

### Privileged / evaluation-only topics

| Topic | ROS 2 type | Frame | Why privileged |
|---|---|---|---|
| `/carla/hero/odometry` | `nav_msgs/msg/Odometry` | `map -> hero` | simulator pose and body-frame twist; GT-equivalent |
| `/carla/hero/vehicle_status` | `carla_msgs/msg/CarlaEgoVehicleStatus` | `map` | GT orientation, speed, acceleration, control |
| `/carla/hero/objects`, `/carla/objects` | `derived_object_msgs/msg/ObjectArray` | `map` | GT pose, twist, acceleration, box, class |
| `/carla/markers` (and `/static`) | `visualization_msgs/msg/MarkerArray` | `map` | GT actor/environment boxes |
| `map -> hero` TF | `tf2_msgs/msg/TFMessage` | `map -> hero` | GT ego pose |

Frame semantics:

- `map` is the CARLA world frame (metres, ROS axis convention after conversion); there is no Earth/ENU frame and no georeference transform.
- `hero` is the CARLA actor transform, whose origin is at the bottom of the vehicle (`actor.py:57-65`). `vehicle.py:74-85` moves the pivot to the bounding-box centre for markers, but TF and odometry do not apply that offset. Do not equate `hero` with a rear-axle `base_link`.
- Sensor frames are `hero/lidar`, `hero/gnss`, `hero/imu`. In synchronous mode (default), each sensor publishes `hero -> hero/<sensor>` from its fixed `relative_spawn_pose` (`sensor.py:109-119`). In asynchronous mode it publishes `map -> hero/<sensor>` from the measured pose (`sensor.py:121-123`), leaking GT ego pose. There is no `/tf_static`; sensor transforms are republished with measurements.
- `sensor.pseudo.tf` publishes `map -> hero` (`tf_sensor.py:63-81`). Omitting it removes ego TF, but sensors still publish parent-to-sensor TF: the only two `sendTransform` call sites are `tf_sensor.py` and `sensor.py`. There is no global `publish_tf` parameter.

## LiDAR fields

`lidar.py:72-87` builds the cloud manually; `create_cloud` sets `height=1`, `width=N`, `is_dense=False`, `is_bigendian=False`, `point_step=16` (`sensor.py:304-312`). Y is negated for ROS (`lidar.py:83-85`).

| Field | Type | Offset | Present | Notes |
|---|---|---:|---|---|
| `x` | `FLOAT32` | 0 | yes | metres, sensor frame |
| `y` | `FLOAT32` | 4 | yes | sign flipped from CARLA |
| `z` | `FLOAT32` | 8 | yes | metres, sensor frame |
| `intensity` | `FLOAT32` | 12 | yes | 0.0–1.0 atmospheric loss |
| `ring` | `UINT16` | — | **no** | not emitted by bridge |
| `time` | `FLOAT32` | — | **no** | no per-point timestamp |

CARLA has enough metadata for an exact `ring`: the raw header is `horizontal_angle`, channel count, then per-channel point counts, and points are stored as `X,Y,Z,I` in channel order (`LibCarla/source/carla/sensor/data/LidarData.h:30-50`; `LidarMeasurement.h:40-56`; `PythonAPI/docs/sensor_data.yml:238-243`). The bridge discards the header: `lidar.py` reads only `raw_data` and never calls `get_point_count()`. Upstream issue #416 tracks this; PRs #431 and #432 proposed a `ring` field but were closed unmerged. The maintainer's position is that `ring` is a bridge-side change, while exact `time` requires CARLA API changes.

Default demo profile (`objects.json:52-63`): 32 channels, 320,000 points/s, 20 Hz rotation, 50 m range, +2.0° to −26.8° vertical FOV, zero noise. At 20 Hz that is 16,000 points/scan before dropoff (500/channel by `Docs/ref_sensors.md:229`). The demo does **not** override CARLA's dropoff defaults, so the blueprint's 0.45 general dropoff and intensity-based dropoff remain active (`LidarDescription.h:54-64`; `ActorBlueprintFunctionLibrary.cpp:963-998`); expect materially fewer than 16,000 points.

## Ground-truth exposure and isolation

Ground truth is opt-in through the spawn object list, not a global switch. The demo `objects.json` includes global `sensor.pseudo.objects` (`objects.json:8-11`) and an attached copy under `hero` (`objects.json:143-146`), plus `sensor.pseudo.odom`, `sensor.pseudo.tf`, `sensor.pseudo.markers`, and `sensor.pseudo.actor_list`. The attached object sensor excludes its parent; the global one includes all vehicles and walkers (`object_sensor.py:74-84`). Each object carries pose, twist, acceleration, box dimensions, and classification (`traffic_participant.py:65-96`). Markers are also GT (`marker_sensor.py:144-155`).

`/carla/hero/vehicle_status` is not opt-in: `EgoVehicle.update()` always publishes GT orientation, speed, acceleration, and control (`ego_vehicle.py:112-130,180-189`). Odometry is opt-in but, when spawned, publishes simulator pose/twist (`odom_sensor.py:60-74`). TF `map -> hero` is opt-in via `sensor.pseudo.tf`; sensor TF is not.

A workable isolation recipe:

1. Use an estimator `objects.json` with only `sensor.lidar.ray_cast`, `sensor.other.gnss`, `sensor.other.imu`, and optionally `sensor.pseudo.opendrive_map`. Remove `sensor.pseudo.objects`, `actor_list`, `markers`, `tf`, `odom`, and `speedometer`.
2. Run with `synchronous_mode:=True` (the launch default, `carla_ros_bridge.launch.py:27-30,66-69`). Without `sensor.pseudo.tf`, `/tf` then contains only `hero -> hero/<sensor>` extrinsics, not GT ego pose.
3. Do not use asynchronous mode for fair evaluation: sensor TF becomes `map -> sensor` and encodes GT pose.
4. Filter `/carla/hero/vehicle_status`, `/carla/hero/objects`, `/carla/objects`, `/carla/markers`, and `/carla/markers/static` at the estimator. `vehicle_status` cannot be disabled in configuration.
5. Give the evaluator a separate node/namespace and allowlist those GT topics. Do not run `carla_ad_agent` in the estimator: it subscribes to `/carla/<role>/odometry` and `/carla/<role>/objects` (`docs/carla_ad_agent.md:24-27`).

## GNSS, IMU, and odometry details

- GNSS: `NavSatFix` fills latitude, longitude, and altitude only; `status` and covariance are not populated (`gnss.py:68-73`). No georeference is published; to fuse into the CARLA map frame, parse the OpenDRIVE `geoReference` or supply the projection externally.
- IMU: orientation comes from the CARLA transform, not integration; gyro `x`/`z` and accelerometer `y` are sign-flipped for ROS (`imu.py:70-86`). Covariances are zero; demo noise is zero, so measurements are ideal unless modified.

## ScenarioRunner integration

The ROS package wraps an external ScenarioRunner v0.9.15 checkout. Set `SCENARIO_RUNNER_ROOT` before launching; the launch file reads it at import time (`carla_ros_scenario_runner.launch.py:5-6,24-25`).

```sh
ros2 launch carla_ros_scenario_runner carla_ros_scenario_runner.launch.py
ros2 service call /scenario_runner/execute_scenario \
  carla_ros_scenario_runner_types/srv/ExecuteScenario \
  "{scenario: {scenario_file: /abs/path/to/scenario.xosc}}"
```

The node runs `/usr/bin/python3 <root>/scenario_runner.py --openscenario ... --timeout 1000000 --host ... --port ... [--waitForEgo]` (`scenario_runner_runner.py:32-47`). `--waitForEgo` is on by default (`launch.py:27-30`), so the ego must already exist with the role name in the `.xosc`; ScenarioRunner matches it (`scenario_runner.py:320-327`). The ROS node's `role_name` parameter is read but unused (`carla_ros_scenario_runner_node.py:62`), so put `hero` in the scenario, not just in launch args.

The child uses `/usr/bin/python3`, not the colcon interpreter, so the `carla` wheel and CARLA's `PythonAPI/carla` (for `agents.navigation.local_planner`) must be importable by that interpreter (checked at startup, `carla_ros_scenario_runner_node.py:32-46`, with `PYTHONPATH` inherited). The ROS runner does not pass `--sync` or `--reloadWorld`, relying on the running bridge/world. ScenarioRunner 0.9.15 requires CARLA >=0.9.15 (`scenario_runner.py:95-97`); add external supervision because its timeout is 1,000,000 s.

## Minimum contract

**Estimator may rely on:**

- `/carla/hero/lidar`: `PointCloud2`, `hero/lidar`, XYZI, 16-byte points, CARLA simulation timestamps.
- `/carla/hero/gnss`: `NavSatFix`, `hero/gnss`, lat/lon/alt; no covariance or georeference.
- `/carla/hero/imu`: `Imu`, `hero/imu`, orientation/gyro/accel; no covariance.
- `/clock`; `hero -> hero/<sensor>` extrinsics from sync-mode TF or the known spawn pose.
- Static map in the CARLA world frame from `/carla/world_info` (always) or `/carla/map` (demo). Apply extrinsics explicitly; `hero` is not `base_link`.

**Evaluator may additionally use:** `/carla/hero/odometry`, `/carla/hero/vehicle_status`, `map -> hero` TF, `/carla/hero/objects` or `/carla/objects`, and markers, in a separate node with a topic allowlist.

**Not available:** `ring`, `time`, per-point timestamps, populated covariances, GNSS fix status, `earth -> map` georeference, `base_link`, or a blanket TF disable switch.

## Adapter work

1. **`ring` (exact, small patch).** Add `PointField('ring', 16, UINT16, 1)` and fill it with `get_point_count(channel)` per channel, as PR #432 proposed. Points are already sorted by channel, so this is exact; adjust `point_step` and the downstream PCL layout (18 bytes packed, or pad for alignment).
2. **`time` (approximate, higher risk).** The bridge exposes neither `horizontal_angle` nor per-point azimuth. Patch `lidar.py` to use `horizontal_angle`, derive azimuth from `atan2(y, x)` before the Y flip, and assume constant rotation; this needs phase/direction calibration and validation against motion distortion. Exact per-point timing is not in 0.9.15, consistent with #416. If that is unacceptable, choose an XYZI-only front end.
3. **Covariances.** Synthesize them from the configured sensor noise model; zero covariance does not mean certainty.
4. **Georeference.** Parse the OpenDRIVE `geoReference` or supply the map projection externally.
5. **`base_link`.** Add a static transform from the CARLA actor origin (bottom) to the rear-axle `base_link`; do not rename `hero` in place.

For a GNSS/IMU/LiDAR estimator that accepts XYZI and supplies its own covariance and georeference handling, this fork is an adequate sensor source. It is not a ready XYZIRT or REP-105 interface, and it cannot provide exact per-point time without deeper CARLA changes.

## Sources

1. `ttgamage/carla-ros-bridge` @ `3ce0be8` — https://github.com/ttgamage/carla-ros-bridge/tree/3ce0be8daf2ae405f7fe47e101028c16fbff1af1
2. `AlexanderRex/carla-ros-bridge` @ `c596934` — https://github.com/AlexanderRex/carla-ros-bridge/tree/c596934b430173a5713bc1ac191ff23ae8df9686
3. Upstream LiDAR ring/time issue #416 — https://github.com/carla-simulator/ros-bridge/issues/416; PRs #431 and #432 (closed unmerged)
4. CARLA 0.9.15 LiDAR source/docs — `LibCarla/source/carla/sensor/data/LidarData.h`, `LidarMeasurement.h`, `PythonAPI/docs/sensor_data.yml`, `Docs/ref_sensors.md` — https://github.com/carla-simulator/carla/tree/0.9.15
5. Python 3.10 wheel — https://github.com/gezp/carla_ros/releases/tag/carla-0.9.15-ubuntu-22.04
6. ScenarioRunner v0.9.15 — https://github.com/carla-simulator/scenario_runner/tree/v0.9.15
