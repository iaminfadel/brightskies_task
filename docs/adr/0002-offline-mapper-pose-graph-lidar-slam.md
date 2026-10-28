# ADR 0002: Offline mapper is lidar pose-graph SLAM, GNSS-anchored

- Status: accepted (2026-10-25)
- Context: BrightDrive task §1 (mapping), `CONTEXT.md`; gates: `docs/results/map/town05-v14-pg/`

## Context

The mapper must produce the saved map from the allowed sensors (LiDAR, GNSS,
IMU — never CARLA ground truth) and pass three independent gates: held-out
GICP, ATE vs ground truth, loop consistency. Two candidate architectures
existed after KISS-ICP was fixed to run on full 10 Hz sweeps (sync bridge):
(a) pose every scan directly from GNSS antenna positions + IMU yaw
(`--gnss-imu`), and (b) pose-graph SLAM: KISS-ICP odometry edges + GNSS
factors + ICP loop closures. In CARLA the GNSS/IMU are noise-free, so (a)
passes the gates with near-ground-truth poses — but it is sensor stitching,
not mapping: it has no lidar, cannot close loops, and does not transfer to a
real vehicle whose GNSS drifts meters and whose IMU yaw bias grows.

## Decision

The canonical `make map` path is (b), the pose-graph lidar SLAM:

- **Odometry**: KISS-ICP on complete sweeps (per-scan accuracy measured at
  ~0.01 deg yaw / ~1 cm translation on this rig).
- **GNSS factors**: antenna ENU positions with the antenna→lidar lever arm
  modeled in-graph (residual rotates with state yaw), plus course-over-ground
  yaw factors only where the vehicle moves.
- **Loop closures**: same-heading revisit pairs, ICP scan-to-submap edges,
  each **vetoed against the GNSS-implied relative pose** (0.3 m / 10 deg)
  before entering the graph, and weighted at GNSS parity (σ≈0.2 m xy, tight
  yaw) — mis-locks on repeated geometry otherwise drag the solution and hide
  their own residual, and loop σ tighter than GNSS σ lets the loop scaffold
  dominate an exact-GNSS rig.
The loop-consistency gate scores the search-free inter-visit residual (each
visit's scan held at its recorded pose vs the other visit's submap), not a
walked ICP correction: on corridor geometry the ICP basin is shallow enough
that centimetre pose differences flip sub-metre locks into multi-metre
slides. Inter-visit consistency cannot see a globally-pulled-but-self-
consistent map; that failure class is owned by the ATE gate.

## Consequences

- The shipped map (town05-v14-pg) is produced by lidar SLAM and passes all
  three gates: held-out 94 % / 0.120 m, ATE 0.037 m RMSE / 0.080 m p95 /
  0.69 deg yaw, loops 0.113 m median / 0.238 m p95 — within 5 mm of the
  truth-posed baseline's loop residual.
- On a real mapping vehicle the same graph degrades gracefully: when GNSS
  degrades (urban canyons, multipath), odometry + vetoed loop closures carry
  the trajectory — exactly the weight structure this rig cannot exercise,
  since its GNSS never degrades. The veto threshold (0.3 m) assumes
  revisits within GNSS relative accuracy; with real GNSS it loosens to a
  Mahalanobis test on the combined covariances.
- Map bundles are pinned artifacts in GCS, gated from the repository by
  `make map-check` (held-out GICP) and `make map-eval-ate` (ATE). The
  localizer consumes only `map.yaml` + `map.pcd`; evaluator files never reach
  the runtime install.
- Known ceiling (`ponytail:`): SE(2)+z states and planar loop measurements —
  upgrade to SE(3) if elevation-heavy towns show residual z error.

## Update (2026-10-28)

The GNSS+IMU-only diagnostic baseline (`--gnss-imu`) and the dispatch-only
`map-gate` workflow were removed when the tree was cut to the shipped demo;
the loop-consistency gate survives as archived evidence under
`docs/results/map/town05-v14-pg/`. The decision above (pose-graph lidar SLAM)
is unchanged.
