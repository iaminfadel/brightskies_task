# brightskies_task

## Agent skills

### Issue tracker

Issues live in this repo's GitHub Issues, managed with the `gh` CLI. See `docs/agents/issue-tracker.md`.

### Triage labels

Defaults: `needs-triage`, `needs-info`, `ready-for-agent`, `ready-for-human`, `wontfix`. See `docs/agents/triage-labels.md`.

### Domain docs

Single-context: `CONTEXT.md` and `docs/adr/` at the repo root. See `docs/agents/domain.md`.

## Runtime workflow

The laptop is the control plane; the GCP VM is the execution plane. No agent runs inside the instance.

- Deploy and run with `infra/vm-run.sh`. It deploys the committed `HEAD` (`git archive`) to `/ws/brightdrive` and runs the command there over IAP; uncommitted changes are never deployed.
- Long jobs run detached under `/ws/jobs/<job-id>/` (`run.sh`, `console.log`, `exit_code`, `pid`); poll with `infra/vm-run.sh --status` and follow with `--logs <job-id>`. Short diagnostics use `--fg`.
- All VM access goes through `infra/vm-ssh.sh` (IAP only). `infra/common.sh` exports `PYTHONNOUSERSITE=1` so gcloud's proto commands keep working.
- Jobs that outlive the SSH session must refresh `/var/lib/brightskies/heartbeat` and handle the 120 s Spot preemption notice. The validation harness does both; the #19 orchestrator must.
- Commit and push before deploying; the repo is the single source of truth. Never edit files under `/ws/brightdrive` on the VM.
