# BrightDrive localization and mapping

The domain of the seven-day BrightDrive technical task: one CARLA 0.9.15 + ROS 2 Humble system that builds a saved point-cloud map, localizes a vehicle against it, proves the result with repeatable scenarios, and gates it in CI.

## Language

**BrightDrive task**:
The seven-day localization, mapping, and CI/CD technical evaluation defined in `Amin_Fadel_Localization_Mapping_CICD_Task.pdf`.
_Avoid_: Project, assignment

**Estimator path**:
The runtime path from the allowed CARLA sensors to the published localization pose, covariance, and health state.
_Avoid_: Online pipeline, production path

**Evaluation path**:
The offline path that may consume CARLA ground truth to score the estimator.
_Avoid_: Scoring node, metrics pipeline

**Ground truth (GT)**:
Simulator-derived pose or state for the ego or other actors; usable only by the evaluation path.
_Avoid_: Reference truth, perfect pose

**Privileged topic**:
A topic that carries ground truth or a ground-truth-derived transform and is therefore never an estimator input.
_Avoid_: Forbidden topic, GT feed

**Saved map**:
The single reusable 3D point-cloud map plus its metadata, in the `map` frame, that online localization matches against.
_Avoid_: World model, SLAM output

**Map frame (`map`)**:
The local ENU frame whose origin is the mapping run's first GNSS fix.
_Avoid_: World frame, CARLA frame

**Map coverage**:
The part of the saved map within 20 m of the mapping trajectory; every scenario route must stay inside it.
_Avoid_: Map extent, map bounds

**`odom`**:
The continuous LiDAR-odometry frame that may drift and is never jumped.
_Avoid_: Local frame, odometry world

**`base_link`**:
The vehicle frame at the rear axle, on the ground plane, x forward, y left, z up.
_Avoid_: Ego frame, vehicle origin

**`hero`**:
The CARLA actor origin at the bottom of the vehicle; a sensor parent, not `base_link`.
_Avoid_: Vehicle frame, ego base

**Scan-to-map matching**:
Registering one live LiDAR scan against the saved map to obtain an absolute pose.
_Avoid_: Localization scan, registration step

**Localization health state**:
The `OK` / `DEGRADED` / `FAILED` assessment of whether the published pose is backed by a valid absolute update.
_Avoid_: Status flag, confidence
