# CI/CD design and gate ratchet

The branch model, gate tiers, and the rules that govern when a gate becomes
required. This is the operating document for the pipeline; the branch-model
rationale is in `docs/adr/0001-branch-model-and-ci-gates.md`.

## Branch model

```
 FEATURE BRANCH  (one per developer, unprotected, branch from main ONLY)
  │   push anything, any time (backup, WIP). Open the PR early: CI on each
  │   push to the PR branch is the advisory per-push signal.
  ▼
 PR ──► STAGING   (integration branch, protected, ALWAYS GREEN)
 │        required: ci/lint, ci/build-test (hosted runners, minutes)
 │        on merge: carla-smoke runs the merged staging state on the VM
 │                 (advisory until proven, see ratchet)
 ▼
 PR ──► MAIN      (shippable branch, protected)
          required today: ci/lint, ci/build-test
          required later : full S1-S4 KPI matrix + regression vs baseline
 ▼
 SHIP from main. Tags are optional rollback/release-note anchors.
```

Invariants:

1. Feature branches are cut from `main`, never from `staging`. Main is always
   green, so nobody inherits a broken base.
2. Staging is always green or being-fixed-right-now. A red staging has no
   integration signal and must not accumulate merges.
3. Promote staging -> main in small batches (every one or two merges). Small
   promotions keep failure attribution trivial; a failed promotion gate with
   one suspect beats a bisection across ten.
4. One variable per promotion: code changes are gated against the pinned map;
   map changes are gated with the current code (`make map-check` and
   `make map-eval-ate` against the pin, after `make map-pull`).

## Gate tiers

| Tier | Check | Venue | Trigger | State |
|---|---|---|---|---|
| 1 | pre-commit hooks (clang-format 14, ruff, shellcheck, hygiene) | laptop | every commit | required (mirrored by CI) |
| 1 | `ci/lint`: pre-commit over the whole tree | hosted | every PR | **required** |
| 2 | `ci/build-test`: colcon build (4 packages) + gtest + launch tests + self-tests (`build_map.py`, `eval_ate.py`, `eval_kpi.py`) + scenario harness tests + isolation audit, in pinned deps image | hosted | every PR | **required** |
| 2 | build/test evidence artifact (14-day retention) | GH | every run, on failure too | live |
| 3 | `carla-smoke`: merged staging state on the GCP VM; MAP_LOAD, LOCKED, TF tree, health, tripwire KPIs, runtime isolation check; evidence to GCS | VM | push to staging | advisory (ratchet) |
| 3 | `make validate-offline`: recorded sensor bag replayed through the real `map_server` + `localization.launch.py`, scored by `map_check`, `eval_ate` and `eval_kpi` + `kpi_gate` | any host with Docker (no GPU, no CARLA) | manual | map + ATE green, KPI red off-VM (see below) |
| 4 | full S1-S4 KPI matrix + regression vs baseline | VM | staging->main PR | pending #10/#11 |
| 4 | forced-fail control (prove gates can fail) | VM | periodic dispatch | script exists, wiring pending |

## The ratchet

A gate cannot be required before the capability it tests exists, and no gate
becomes required until it has been proven. The rule is: **advisory -> proven
-> required.** A gate is promoted to required only after it (a) is green on
consecutive real runs and (b) has been demonstrated to fail (forced-fail
control, `scripts/forced_fail_control.py`).

History:

| Date | Gate | Transition | Evidence |
|---|---|---|---|
| 2026-10-24 | ci/lint, ci/build-test on PRs | required, proven green | PR #21 (feature→staging) and #22 (promotion→main), all checks SUCCESS; v0.1 tagged at fa1f765 |
| 2026-10-26 | carla-smoke on staging | advisory; WIF blocker cleared: provider recreated as `github-oidc-v2` accepting `refs/heads/main`, `refs/heads/staging`, and `workflow_dispatch` (#42); first green run end-to-end (auth → SSH → deploy → CARLA → validation → evidence), then green from a fully cold VM after sensor-wait + stockout-retry hardening | runs 40408962888, 40530646416; ratchet continues (needs consecutive greens + forced-fail) |
| pending | full KPI matrix on promotion | requires scenario harness (#10) + evaluator (#11) | promotion run + forced-fail run |

## Workflows

- `.github/workflows/ci.yml` - tiers 1+2. Also `workflow_call` for reuse.
  Pulls the deps image from GHCR (public; the login step only lifts the
  anonymous rate limit).
- `.github/workflows/carla-smoke.yml` - tier 3, push to `staging` + manual
  dispatch. Dispatches via `scripts/vm_job.py`: start on-demand VM, deploy
  committed staging HEAD (`git archive`, never uncommitted state), then
  `scripts/validation/run_on_vm.sh` builds the workspace and runs the compose
  `test` route (`make test`), collecting `/ws/evidence`, and stops the VM.
  Remote exit 44 (PREEMPTED) retries once on a fresh VM; test failures never
  auto-retry.
- `.github/workflows/carla-full.yml` - tier 4 (dispatch-only until #10/#11).
- `.github/workflows/wif-smoke.yml` - WIF auth smoke, no keys anywhere.

Local equivalents: `make ci-lint`, `make ci-vm-build-test` (same colcon
command as CI, on the VM deps container - the laptop never builds ROS).

## CARLA-free validation

The CARLA VM is the flakiest part of the system, so the runtime tier can also
be exercised without it. `make validate-offline` runs
`scripts/validate_offline.sh`, which starts the same nodes as `make up`
(`map_server` + `localization.launch.py`) in a deps container, replays a
recorded sensor bag, records the estimator topics, and scores
the result with `map_check` (scan-to-map consistency), `eval_ate.py` (bag vs
bag, no CARLA) and `eval_kpi.py` + `kpi_gate.py`. It does not cover the CARLA
bridge or ScenarioRunner; those stay in the tier 3 smoke. Evidence lands in
`$(OFFLINE_EVIDENCE)`.

On the pinned bundle (`town05-v14-pg`, bag `map_full4`) it reproduces the
committed evidence: `MAP_CONSISTENCY scans=100 accepted=94 fraction=0.94
median_rmse_m=0.120`.

Replay without `--clock`. The recorder already stored the bridge's `/clock`
(sim time), and `ros2 bag play --clock` publishes *bag* time instead - which
is wall-clock recording time, not simulator time. Two `/clock` sources make
every `use_sim_time` node jump between clocks: poses come out wall-stamped,
message ages become nonsense and `kpi_gate` fails on latency. `validate_offline.sh`
therefore plays the bag as recorded and lets the stored `/clock` drive the
stack.

Playback is also slowed to match sim time. The bridge free-runs when it is not
throttled, so the pinned bag packs 627 s of sim time into 268 s of bag time;
playing it at `-r 1.0` would feed the estimator 2.3x real time and it would
never lock. The script derives the rate from the bag's own `/clock`
(`rate = speed * bag_duration / sim_duration`), so `OFFLINE_SPEED=1.0` always
means "the pace the stack saw live".

**Current state off-VM (2026-10-28).** On the pinned bundle the replay
reproduces the committed map evidence exactly - `MAP_CONSISTENCY scans=100
accepted=94 fraction=0.94 median_rmse_m=0.120` and `ATE_SUMMARY poses=6270
abs_rmse_m=0.037 yaw_rmse_deg=0.69 passed=True` - so the replay is faithful to
the recording. The KPI tier fails on the laptop, and the numbers say why:
`heading_rmse` 99.5 deg, `heading_p95` 180 deg, `latency_max` 599.6 ms (with
`latency_p95` 115 ms and `latency_drift` -87 ms passing). GICP costs ~1.07 s
per scan on this host (`map_check`: 107 s for 100 scans) against the ~200 ms
the 5 Hz pipeline is sized for, so `localization_core`'s single-threaded
executor falls behind: `/localization/status` was published 6 times in 627 s
instead of at 10 Hz, `gnss_age` reached 135 s, health went OK -> DEGRADED
(`gnss_timeout`) -> FAILED (`sources_stale`), and the pose held a 180-deg yaw
flip from t=376 s. Treat the KPI tier as unproven off-VM rather than as a
regression: it needs the VM's cores and a fresh live run before the thresholds
mean anything.

**Deps image pin.** `docker/deps.lock` pins a digest built 2026-10-22, before
`docker/deps.Dockerfile` started pip-installing `rosbags`/`scipy`; the image
therefore lacks them, and the `:validation` tag was never published. Until the
image is rebuilt and re-pinned (needs a GHCR push), every entry point calls
`scripts/ensure_py_deps.sh`, which imports what the scripts need and installs
only the missing modules. It is a no-op on a correctly pinned image, so
re-pinning later is a one-line change with no code to revert.

## Artifacts and evidence

Every VM evidence bundle carries a provenance block: git SHA (`.deployed-sha`),
map id + hash, deps + CARLA image digests (from `docker/deps.lock`), scenario
seed. Hosted artifacts: junit XML, colcon logs, uploaded `if: always()` so
failures keep their evidence. Big evidence (bags, maps, runs) lives in GCS
under the existing lifecycle rules; small reports on GitHub.

Run KPIs (`scripts/eval_kpi.py`) consume the estimator bag plus the separate
GT bag and write `metrics.json`, `metrics.csv`, `metrics_summary.md`, and
`kpi_heading_latency.png` into the evidence bundle. `scripts/kpi_gate.py`
owns the #11 thresholds and prints one `KPI CHECK` line per metric before a
single `KPI PASS`/`KPI FAIL`; the same gate is exercised by
`scripts/forced_fail_control.py` so a broken heading or latency threshold
cannot pass silently. `eval_kpi.py --self-test` also round-trips a synthetic
bag pair, so the bag-reading path (CDR, the `base_link` <-> `hero` lever,
`T_map_carla`, `/clock` -> sim time) is covered in CI; the build-test job
installs `rosbags` for it.

## Failure runbook

- **PR red (lint/build-test)**: your change; fix forward. Evidence is in the
  run's artifact.
- **Staging smoke red**: integration broke. Nobody branches from staging, so
  there are no bystanders; fix or revert the offending merge *in staging*,
  then re-push (smoke re-runs on the new staging head).
- **Promotion red (later)**: diff per-merge smoke metrics stored with each
  staging merge to narrow suspects; revert in staging; re-promote.
- **Preempted (exit 44)**: infra failure; vm_job retries once on a fresh VM.
  Never treat a preemption as a test result.

## v0.1 bootstrap

Bootstrap (executed 2026-10-24): repo is public (required checks need GitHub
Pro on private repos), squash-merge-only policy, `staging` created from
`main`, branch protection live on both branches with `ci / lint` +
`ci / build-test` required (PRs only, no force push, up-to-date branches,
conversation resolution). First PR #21 → staging and promotion PR #22 → main
both green; `v0.1` tagged at fa1f765. Re-run anytime with `make ci-bootstrap`
(`scripts/ci_bootstrap.sh` is idempotent).
