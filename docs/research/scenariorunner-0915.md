# ScenarioRunner 0.9.15 mechanics for the four CARLA scenarios

Scope: CARLA 0.9.15, ScenarioRunner 0.9.15, ROS 2 Humble, and the `ttgamage` bridge fork. This note establishes the authoring/runtime contract; the final scenario matrix remains issue #10.

## 1. Version and install path

- Use the ScenarioRunner **v0.9.15 tag** (`d12d8bb788201ebf6f03e55b9c5169f9dcb2ea87`), not a release string: `scenario_runner.py:47` still prints `VERSION = '0.9.13'`.
- Use the Humble fork `ttgamage/carla-ros-bridge` (`3ce0be8daf2ae405f7fe47e101028c16fbff1af1`); official `carla-simulator/ros-bridge` master is 0.9.13. The fork targets CARLA/SR 0.9.15, Ubuntu 22.04, Humble, and Python 3.10.
- Install a CARLA **Python distribution** for 0.9.15, not just a `PYTHONPATH` entry: `scenario_runner.py:95-97` calls `pkg_resources.get_distribution("carla")` and rejects anything below 0.9.15.
- ScenarioRunner's pins are hostile to Python 3.10: `numpy==1.18.4`, `Shapely==1.7.1`, and `opencv-python==4.2.0.32` lack cp310 wheels. Use a tested relaxed set or a Python 3.8 container; do not assume the pins install on Humble's Python. `antlr4-python3-runtime==4.10` is only for OSC2.
- The ROS wrapper hardcodes `/usr/bin/python3` and starts `scenario_runner.py` from there (`scenario_runner_runner.py:36-47`). Install SR dependencies and the CARLA wheel into system Python, or patch the wrapper to use a venv. Setting `SCENARIO_RUNNER_ROOT` is mandatory for the wrapper and for `--record`.

## 2. Execution modes and tick ownership

| Mode | Ego | World ticker | Repeats | Duration limit |
|---|---|---|---|---|
| Route + agent | SR spawns `hero` | SR (`--agent` forces `--sync`) | `--repetitions` works | `RouteTimeoutBehavior`: min 300 s, extended by progress |
| OSC + `--waitForEgo` | Bridge spawns `hero`; SR snaps it to OSC init | Bridge | OSC ignores `--repetitions` | `timeout=100000` s plus OSC stop trigger |

- `--route` forces `reloadWorld=True` (`scenario_runner.py:627-628`); `--agent` forces `sync=True` (`630-631`); route cannot be combined with `--scenario`/`--openscenario`; agents are route-only (`609-622`).
- Do not run route mode with the bridge attached. The reload destroys bridge actors, and SR+agent is the ticker. Do not run OSC full-stack with SR `--sync`: the bridge and SR would both tick. Full-stack is bridge ticker + SR `--waitForEgo` without `--sync`.
- `--waitForEgo` finds an existing actor by `role_name`, teleports it to the scenario init transform, zeroes velocity, applies neutral control, and registers it (`scenario_runner.py:212-242`). The OSC ego `role_name` must match the bridge spawner (default `hero`).
- `ScenarioManager.run_scenario` busy-polls `world.get_snapshot()` (`scenario_manager.py:128-136`); `_tick_scenario` advances only when the snapshot timestamp increases (`150-159`). This burns CPU between bridge ticks but does not double-tick. Budget accordingly.
- `--timeout` is both CARLA client timeout (`scenario_runner.py:87-94`) and the manager watchdog timeout (`107`; `scenario_manager.py:124`). The watchdog is reset on every tick (`159`), so it detects stalls, not total runtime. The ROS wrapper passes `1000000` (`scenario_runner_runner.py:42`), which disables useful protection. Always add an external CI `timeout`.
- `_estimate_route_timeout` returns `0.4 * route_length` (`route_scenario.py:51,131-143`) for reports/child scenarios; the actual `RouteTimeoutBehavior` starts at 300 s and extends at 10% of road speed limit (`timer.py:162-239`). OSC's `100000` s is effectively unbounded; use an OSC `StopTrigger` plus an external timeout.
- Criteria failure does not normally produce a non-zero exit: `_load_and_run_scenario` returns True after a completed run (`scenario_runner.py:438`), and `main` returns `not result` (`645`). Only load/setup failures are reliable non-zero exits. Parse JSON/JUnit.
- `--json` writes `{scenario, success, criteria:[...], Duration}` (`result_writer.py:137-204`); JUnit writes XML (`206-288`). If the scenario has **no criteria**, `analyze_scenario` returns before writing either file (`scenario_manager.py:212-215`). At least one `criteria_*` is mandatory for an OSC CI report.
- `--outputDir` and `--record` directories must exist. JSON/JUnit names are `<config.name><timestamp>.json|.xml`; `--record` writes the server recorder log under `SCENARIO_RUNNER_ROOT/<record>/` plus raw criteria JSON (`scenario_runner.py:420-436`).
- The wrapper exposes `/scenario_runner/{execute_scenario,status}` but forwards no JSON/JUnit/recorder options; it is not a CI pass/fail source. Run SR directly with `--waitForEgo` or patch `ScenarioRunnerRunner`.

## 3. Route authoring

Minimal shape (`route_parser.py:47-123`):

```xml
<routes>
  <route id="town05_turn" town="Town05">
    <waypoints>
      <position x="45.0" y="209.0" z="0.0"/>
      <!-- anchors; turns are interpolated -->
    </waypoints>
    <scenarios/>                 <!-- required even when empty -->
    <weathers>                   <!-- optional; default 70 deg sun, 50% cloud -->
      <weather route_percentage="0" sun_altitude_angle="45" cloudiness="20"/>
    </weathers>
  </route>
</routes>
```

- `<scenarios/>` is required: `route_parser.py:78` calls `route.find('scenarios').iter(...)` and throws if absent.
- Route XML has **no speed field**. `RouteScenario` spawns `vehicle.lincoln.mkz_2017` as `hero` (`route_scenario.py:70,120-127`); speed comes from the agent/controller. `NpcAgent` uses `BasicAgent(..., 30)` (30 km/h; `npc_agent.py:74`). A 50 km/h challenge requires a custom agent or controller.
- Route turns are implicit: `interpolate_trajectory` (`route_manipulation.py:133-160`) uses `GlobalRoutePlanner`; waypoints are coarse anchors, not a full sampled trajectory.
- `NpcAgent` includes a left RGB camera (`npc_agent.py:52-57`), unusable under `-nullrhi`. `DummyAgent` is not no-sensor either: it declares cameras, LiDAR, GNSS, CAN, and HD-map (`dummy_agent.py:28-59`). Write an `AutonomousAgent` subclass whose `sensors()` returns `[]` for motion smoke.
- Route `AgentWrapper` supports only camera/LiDAR/GNSS (`agent_wrapper.py:51-75`); an IMU spec fails. Use bridge/OSC for a full GNSS+IMU+LiDAR rig; route mode is motion smoke.
- Route mode always adds `BackgroundBehavior` (`route_scenario.py:286-287`); there is no CLI to disable it. It spawns dynamic traffic using the TM and `CarlaDataProvider` RNG (`background_activity.py:165+`). Budget CPU and use `--trafficManagerSeed`.
- Route criteria: completion (10 m/99%; `atomic_criteria.py:1526-1527`), lanes, collision, lights, stop, min speed, off-route, blocked; a stuck route can consume 300 s.
- `--route` without `--route-id` executes every route in the file. Example:
  `python3 scenario_runner.py --route route.xml --route-id town05_turn --agent headless_agent.py --sync --host 127.0.0.1 --port 2000 --trafficManagerSeed 0 --json --junit --outputDir results --record record`

## 4. OpenSCENARIO authoring and full-stack

- Use OSC 1.0 (`--openscenario`); `--openscenario2` is a separate parser. Put `CARLA:` at the start of `FileHeader description` to use CARLA coordinates (`Docs/openscenario_support.md:31`).
- `LogicFile filepath="Town05"` must match the bridge/server map. Without `--reloadWorld` (the wrapper never reloads), SR checks the live map and exits on mismatch (`scenario_runner.py:351-355`).
- Ego is the `ScenarioObject` with `<Property name="type" value="ego_vehicle"/>`; its `name` becomes `role_name`. The bridge default `hero` matches `carla_ad_demo/config/FollowLeadingVehicle.xosc`.
- `ControllerAction` in `Init` is supported; `AssignRouteAction`/`FollowTrajectoryAction` are **Story-only**; `RoutePosition` is unsupported (use `WorldPosition`); `ActivateControllerAction` is Story-only; `EnvironmentAction` is Init-only (`Docs/openscenario_support.md:97-109`). `AssignRouteAction` maps to `ChangeActorWaypoints`, which requires a controller in `ActorsWithController` (`openscenario_parser.py:1499-1505`; `atomic_behaviors.py:730+`).
- The working full-stack example uses `ControllerAction` with `module="carla_ros_scenario_runner.ros_vehicle_control"`, `launch="carla_ad_agent.launch"`, `launch-package="carla_ad_agent"` (`carla_ad_demo/config/FollowLeadingVehicle.xosc:76-94`). It also uses `SpeedAction` 25 m/s and `Town05` (lines 9, 96-105).
- OSC has **no automatic background traffic**: only declared entities plus whatever the bridge/spawner creates. Route mode is the only mode with built-in `BackgroundBehavior`.
- OSC weather is Init-only. Route weather supports multiple percentages; OSC uses one environment action.
- Evaluation criteria are `StopTrigger` conditions named `criteria_<ClassName>`, using `ByValueCondition/ParameterCondition` (`open_scenario.py:566-584`; `openscenario_parser.py:1092-1116`). The example includes all seven. Use `criteria_CollisionTest` at minimum.
- Docs bug: `Docs/openscenario_support.md:166` says `criteria_OnSideWalkTest`, but the class is `OnSidewalkTest`; use `criteria_OnSidewalkTest`. The parser looks up `condition_name[9:]` in `globals()` and throws on the docs spelling.
- `RosVehicleControl` publishes `/carla/<role>/waypoints` and `/carla/<role>/target_speed` but never sets `_reached_goal` (`basic_control.py:38-48,93-100`; `ros_vehicle_control.py:110-124`). `ChangeActorWaypoints` can therefore stay RUNNING; end route-action scenarios with an explicit `StopTrigger` (traveled distance/time), not route-action completion.
- Reference full-stack is **GT-driven**: `carla_ad_agent` uses `/carla/<role>/odometry` for pose/speed; the planner converts odometry m/s to km/h (`local_planner.py:101-110`), while SR's `SpeedAction` target is m/s. That is acceptable for open-loop localization evaluation if the estimator is passive, but it is not estimator-closed-loop control. For closed-loop, use a custom controller or remap odometry to estimator output; keep separate GT for evaluation.
- Recommended full-stack command:
  `python3 scenario_runner.py --openscenario FollowLeadingVehicle.xosc --waitForEgo --host 127.0.0.1 --port 2000 --timeout 1000000 --json --junit --outputDir results --record record`
  with `results/` and `record/` pre-created and the bridge ticking. Do not add `--sync` or `--reloadWorld`.

## 5. Noise, degradation, determinism, headless

- Native GNSS: `noise_lat_stddev`, `noise_lon_stddev`, `noise_alt_stddev`, `noise_lat_bias`, `noise_lon_bias`, `noise_alt_bias`, `noise_seed`, `sensor_tick` (`Docs/ref_sensors.md:121-133`). The error is Gaussian + constant bias (`GnssSensor.cpp:50-58`). **Latitude/longitude values and their sigmas are in degrees, not metres**; convert metre-level specs (roughly `1e-5 deg/m`). Altitude is metres.
- Native IMU: `noise_accel_stddev_x/y/z`, `noise_gyro_stddev_x/y/z`, `noise_gyro_bias_x/y/z`, `noise_seed`, `sensor_tick` (`Docs/ref_sensors.md:159-174`). Accelerometer noise is zero-mean Gaussian with **no acceleration bias**; gyro noise also has a constant bias (`InertialMeasurementUnit.cpp:73-99`). The accelerometer includes `+9.81 m/s^2` gravity in world Z before rotation into the sensor frame (`InertialMeasurementUnit.cpp:102-144`); gyro is rad/s.
- No native GNSS/IMU dropout; use a ROS 2 relay or custom sensor actor. Native bias/noise covers constant-bias and Gaussian cases.
- `noise_seed` defaults to `0` on standard blueprints, so normal spawning is deterministic; set it explicitly. Random fallback applies only when the attribute is absent (`ActorBlueprintFunctionLibrary.cpp:1684-1692,1713-1720`).
- Sensor rig cannot be declared in route XML or OSC `Entities`; attach via the bridge/`carla_spawn_objects` or a custom agent. Replace the stock camera/radar-laden `objects.json` with LiDAR/GNSS/IMU plus controller pseudo sensors.
- Determinism: sync + fixed `fixed_delta_seconds` before load/reload (`Docs/adv_synchrony_timestep.md:205-275`); SR 20 Hz/0.05 s (`scenario_runner.py:67,339`); bridge 0.05 s. Set `--trafficManagerSeed`; `tm.set_random_device_seed` runs per scenario (`379-383`), but reset TM sync after reload (`Docs/adv_traffic_manager.md:372-397`).
- No `world.set_pseudo_random_seed` exists in 0.9.15. `CarlaDataProvider` uses a fixed 2000 seed (`carla_data_provider.py:67-68`); `--randomize` has no global seed. Use a fresh server/world per run and explicit seeds. Do not assume two runs are identical.
- Weather: route defaults to sun 70 deg / cloud 50 (`route_parser.py:106-123`); route can interpolate multiple weathers. OSC `EnvironmentAction` is Init-only.
- Headless: `-nullrhi` removes rendering; cameras/GPU sensors are unusable (empty or crash), RViz must be disabled. LiDAR is CPU ray-casting and works; GNSS/IMU work. `no_rendering_mode` also empties camera data; off-screen rendering needs a GPU. Lower LiDAR channels/points for CI.
- Ground truth: `--record` writes the server recorder with additional data (`scenario_runner.py:420-423`): per-frame transforms/velocities/controls; query `show_all=True`. It excludes ROS sensor messages; JSON/JUnit are criteria only. Bridge odometry/TF are GT-like, evaluator-only; `carla_ad_agent` consumes them.

## 6. Scenario-authoring contract

- One scenario directory containing the OSC/route file, the spawn/sensor `objects.json`, explicit weather/seed/timeout metadata, and expected criteria.
- Full-stack scenarios: OSC + bridge ticker + `--waitForEgo`; no SR `--sync`, no `--reloadWorld`; `LogicFile` equals the bridge town; `hero` role matches.
- Route-only scenarios: route XML + custom no-sensor agent (or GNSS/LiDAR-only agent), `--sync`; do not attach the bridge; accept mandatory background traffic and the 300 s minimum timeout.
- Every scenario declares at least one `criteria_*` condition (OSC) or relies on route criteria; otherwise no JSON/JUnit is produced.
- Sensors: LiDAR/GNSS/IMU only; no cameras/RViz; explicit `noise_seed`; GNSS sigmas converted from metres to degrees; IMU gravity convention documented.
- Determinism: sync, 0.05 s, TM seed, fresh server, explicit sensor seeds/weather; route `--repetitions` only; OSC repetitions driven by an external process.
- Evaluation: JSON/JUnit are the pass/fail gate; recorder is the raw per-frame GT; GT topics are evaluator-only. Full-stack control may use GT (open-loop) or estimator output (closed-loop), but the two must be labelled.
- External CI timeout is mandatory: OSC has a 100000 s internal timeout; route has a 300 s minimum; the `--timeout` watchdog only detects stalls.

**Smoke-test minimum:** in `-nullrhi` on Town05, start CARLA, bridge, and a minimal LiDAR/GNSS/IMU spawn; run one OSC full-stack scenario with `--waitForEgo --json --junit`, at least `criteria_CollisionTest`, and an explicit OSC `StopTrigger`; assert JSON `success`, at least one LiDAR/GNSS/IMU message, TF/odometry present, no camera topics, and no scenario run beyond a 120 s external timeout. If route mode is part of the matrix, add one 100 m route with the custom no-sensor agent and assert route completion plus JSON success. This proves both tick-ownership paths; it does not decide the four-scenario matrix or the final noise/controller choices (issue #10).
