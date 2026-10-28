# ADR 0004: Product code vs infrastructure in the repository layout

- Status: accepted (2026-10-28)
- Context: the strip-down review of `mvp/strip`. `scripts/` held the deliverable
  (`build_map.py`, the evaluators, the scenario harness) next to helpers that
  exist only because the execution plane is a Spot VM on GCP (`vm-run.sh`,
  `vm_job.py`, `ci_bootstrap.sh`) or because one-offs run in a pinned container
  (`ros_exec.sh`, `ensure_py_deps.sh`). Swapping the provider or the container
  runner would delete files that product code sat beside, and a held-out map
  check lived inside a ROS package where it was built and shipped as if it were
  part of the runtime.

## Decision

Four trees, each with one job:

- `src/` — ROS 2 packages the runtime launches. `colcon build` is the whole
  contract; nothing in here is a development utility.
- `scripts/` — product entry points that run inside the workspace and make no
  assumption about the host: mapping (`build_map.py`, `pose_graph.py`,
  `map_run.sh`, the route drivers), evaluation (`eval_ate.py`, `eval_kpi.py`,
  `kpi_gate.py`), scenarios (`scenario/`) and validation harnesses
  (`validation/`).
- `tools/` — development-only checks that support the product but are not part
  of it. `tools/map_check` is a plain CMake executable that compiles the
  production GICP matcher and PCD reader; it has no ROS package, no node, and
  installs nothing.
- `infra/` — everything tied to *where* the system runs: GCP provisioning, VM
  lifecycle, IAP access, `git archive` deploys (`vm-run.sh`), the preemption-
  aware CI job runner (`vm_job.py`), the one-off container runner
  (`ros_exec.sh`), the deps-image top-up (`ensure_py_deps.sh`) and the CI
  bootstrap (`ci_bootstrap.sh`). Replacing the provider is a rewrite of this
  tree and nothing else.

Rule of thumb: if a file stops working when the environment changes, it belongs
in `infra/`; if it checks the product without being part of the runtime, it
belongs in `tools/`; everything else the task delivers belongs in `src/` or
`scripts/`.

## Consequences

- `Makefile`, CI workflows, `AGENTS.md` and the README point at `infra/` for
  deployment verbs; product targets keep the same names (`make map`,
  `make eval-kpi`, `make map-check`).
- `make map-check` builds `tools/map_check` in the pinned deps image and runs it
  on the bundle, so `docs/results/map/town05-v14-pg/map_consistency.json` is
  reproducible (100 scans, 94 accepted, 0.120240782 m median) without adding a
  binary to the runtime install.
- `ci/build-test` compiles `tools/map_check`, so the check cannot rot while
  staying out of `colcon test`.
- The map bundle reader stays with the map server (ADR 0003); `map_check` reads
  `map.yaml`/`map.pcd` directly rather than reinstating a loader in the
  estimator package.
