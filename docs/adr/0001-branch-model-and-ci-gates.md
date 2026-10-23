# ADR 0001: Branch model and CI gate placement

- Status: accepted (2026-10-24)
- Context: BrightDrive task, `CONTEXT.md`; pipeline operating doc: `docs/ci.md`

## Context

The system has two validation cost classes. Cheap checks (colcon build, gtest,
interface tests, evaluator self-tests, isolation audit) run in a pinned Docker
image on hosted runners in minutes. Expensive checks (CARLA smoke, S1-S4 KPI
matrix) need the GPU VM and take tens of minutes to hours, serialized on a
single on-demand instance. The pipeline must gate a team of developers working
on independent features without blaming an auxiliary-feature PR for an
integration failure, and `main` must be shippable at every commit.

## Decision

Two protected branches with escalating gates:

- **staging** = integration branch. Feature PRs (one per developer, cut from
  main only) merge here after the cheap required checks. The merged staging
  state is smoke-verified on the VM on every push to staging.
- **main** = shippable branch. staging -> main promotions are PRs gated by the
  full KPI matrix (plus regression vs the previous baseline once the evaluator
  lands, #10/#11).

Gate cadence is inverse to gate cost: cheap checks run per PR, smoke runs per
staging merge, the full matrix runs per promotion. Gates are introduced by a
ratchet (advisory -> proven green + proven able to fail -> required).

## Alternatives considered

- **Merge queue (batched pre-merge integration).** Mechanically superior
  attribution (the failing batch names itself), but batches queue on the one
  serialized expensive resource; queue latency scales with batch-gate cost.
  Rejected for this execution plane; correct once validation is fast and
  parallel (GPU farm).
- **GitFlow-style shared `develop`.** A shared integration branch that accepts
  WIP pushes has no integration signal; it degenerates into a backup server.
  WIP backup belongs to personal feature branches, which this model keeps.
- **Trunk-based with nightly full matrix.** Cheapest and simplest, but `main`
  is only smoke-verified between nightlies; a KPI regression can dwell on
  main for hours. Rejected because shippable-at-every-commit was a hard
  requirement.

## Consequences

- Failure attribution is discipline-based, not mechanical: promote in small
  batches; each staging merge stores its smoke metrics so a failed promotion
  can be narrowed by diffing stored metrics.
- Staging must be kept green; a red staging blocks integration for everyone
  and must be fixed or reverted immediately.
- Reverting in staging and re-promoting is the standard recovery; nobody
  branches from staging, so there are no bystander victims.
- Map promotion enters through a pin-bump PR (see `docs/ci.md`, "Mapping CI"),
  so code and data never change in the same promotion.
