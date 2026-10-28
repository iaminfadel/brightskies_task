# map_check

Held-out map consistency check: the offline map is scored by matching scans that
were **excluded** from it, using the exact GICP matcher the estimator runs.

This is a development/validation tool, not part of the runtime. It is a plain
CMake executable with no ROS dependency and no colcon package, so it cannot be
mistaken for a node and it does not land in the runtime install:

- it compiles the production sources directly (`gicp_matcher.cpp`, `se3.cpp`,
  `pcd_io.cpp`), so the Hessian condition, convergence and rejection semantics
  are the same as `localization_core`;
- it reads only `map.yaml` + `map.pcd` + `validation/heldout/`, so it works on a
  map bundle straight out of `scripts/build_map.py`.

## Run

`make map-check` builds it in the pinned deps image and runs it against
`$(MAP_BUILDS)/$(MAP_ID)` on the VM. By hand:

```bash
cmake -S tools/map_check -B /ws/build/map_check -DCMAKE_BUILD_TYPE=Release
cmake --build /ws/build/map_check -j"$(nproc)"
/ws/build/map_check/map_check /ws/maps/builds/town05-v14-pg
```

Output: one `MAP_CHECK` line per scan, a `MAP_CONSISTENCY` summary line, and
`validation/map_consistency.json` inside the bundle (override with a second
argument). Exit code 0 when the gates pass: accepted fraction >= 0.80 and
median RMSE <= 0.20 m.
