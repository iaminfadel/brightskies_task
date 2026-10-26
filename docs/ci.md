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
   map changes are gated with the current code (see "Mapping CI", below).

## Gate tiers

| Tier | Check | Venue | Trigger | State |
|---|---|---|---|---|
| 1 | pre-commit hooks (clang-format 14, ruff, shellcheck, hygiene) | laptop | every commit | required (mirrored by CI) |
| 1 | `ci/lint`: pre-commit over the whole tree | hosted | every PR | **required** |
| 2 | `ci/build-test`: colcon build + gtest + interface launch test + self-tests (`build_map.py`, `eval_ate.py`, `eval_loops.py`, `eval_kpi.py`) + isolation audit, in pinned deps image | hosted | every PR | **required** |
| 2 | build/test evidence artifact (14-day retention) | GH | every run, on failure too | live |
| 3 | `carla-smoke`: merged staging state on the GCP VM; MAP_LOAD, LOCKED, TF tree, health, tripwire KPIs, runtime isolation check; evidence to GCS | VM | push to staging | advisory (ratchet) |
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
| pending | carla-smoke on staging | advisory; BLOCKED on WIF: the GCP workload-identity provider's attribute condition only accepts refs/heads/main, so staging pushes fail auth (`unauthorized_client`). Recreate the provider with a condition accepting staging (and dispatch) before this gate can be proven | run 40088184551 |
| pending | full KPI matrix on promotion | requires scenario harness (#10) + evaluator (#11) | promotion run + forced-fail run |

## Workflows

- `.github/workflows/ci.yml` - tiers 1+2. Also `workflow_call` for reuse.
  Pulls the deps image from GHCR (public; the login step only lifts the
  anonymous rate limit).
- `.github/workflows/carla-smoke.yml` - tier 3, push to `staging` + manual
  dispatch. Dispatches via `scripts/vm_job.py`: start on-demand VM, deploy
  committed staging HEAD (`git archive`, never uncommitted state), run
  `scripts/validation/run_on_vm.sh`, collect `/ws/evidence`, stop VM. Remote
  exit 44 (PREEMPTED) retries once on a fresh VM; test failures never
  auto-retry.
- `.github/workflows/carla-full.yml` - tier 4 (dispatch-only until #10/#11).
- `.github/workflows/wif-smoke.yml` - WIF auth smoke, no keys anywhere.

Local equivalents: `make ci-lint`, `make ci-vm-build-test` (same colcon
command as CI, on the VM deps container - the laptop never builds ROS).

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

## Mapping CI (map-gate workflow)

The map is a data artifact, not code state: mapping is a periodic data
pipeline, localization is the software product that consumes a pinned map.

- CI always tests against a **named, immutable, hash-verified map** - never
  "latest".
- New maps are promoted through a handshake: map self-gates (closure,
  held-out, hash) -> cross-validation (current code + new map + fresh bag +
  GT KPIs - the only non-circular map gate) -> publish to the registry ->
  pin-bump PR through the normal promotion gates.
- One variable per promotion: code gated against the old map, maps gated
  with the current code.

Implemented (2026-10-25): `.github/workflows/map-gate.yml` (workflow_dispatch,
advisory per the ratchet) pulls the pinned bundle from GCS, runs
`scripts/map_verify.py` plus the three gates on the VM (`map_check`,
`eval_ate.py`, `eval_loops.py` — all exit non-zero on failure and write
JSON/PNG evidence, collected as the workflow artifact). First pinned map:
`town05-v14-pg` (lidar pose-graph SLAM, ADR 0002). Promotion = upload with
`make map-upload MAP_ID=<id>`, then dispatch map-gate on the pin, then pin
bump in the localizer config via PR.
