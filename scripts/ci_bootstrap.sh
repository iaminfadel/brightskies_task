#!/usr/bin/env bash
# One-time CI bootstrap (docs/ci.md): squash-merge policy, the staging
# integration branch, and branch protection matching the current ratchet
# state. Idempotent; safe to re-run. Requires gh authenticated as a repo
# admin and git push access.
set -euo pipefail

REPO="${REPO:-iaminfadel/brightskies_task}"
# Required checks must match job names in .github/workflows/ci.yml exactly.
CONTEXTS='["ci / lint", "ci / build-test"]'

gh auth status >/dev/null

echo "==> repo merge policy: squash only, auto-delete merged branches, auto-merge on"
gh api -X PATCH "repos/$REPO" \
  -f allow_squash_merge=true \
  -f allow_merge_commit=false \
  -f allow_rebase_merge=false \
  -f allow_auto_merge=true \
  -f delete_branch_on_merge=true >/dev/null

echo "==> staging branch (integration, protected below)"
if [ -z "$(git ls-remote --heads origin staging)" ]; then
  git fetch origin main
  git push origin origin/main:refs/heads/staging
  echo "    created staging from main"
else
  echo "    staging already exists"
fi

for branch in main staging; do
  echo "==> protect ${branch}: PRs only, no force push, required checks, up-to-date branches"
  gh api -X PUT "repos/$REPO/branches/${branch}/protection" --input - <<EOF
{
  "required_status_checks": {
    "strict": true,
    "contexts": ${CONTEXTS}
  },
  "enforce_admins": false,
  "required_pull_request_reviews": null,
  "restrictions": null,
  "allow_force_pushes": false,
  "allow_deletions": false,
  "required_conversation_resolution": true
}
EOF
done

cat <<'NOTE'

Done. Current ratchet state (see docs/ci.md):
  required now:   ci / lint, ci / build-test (on PRs to staging and main)
  advisory now:   carla-smoke on staging pushes
  pending gates:  full KPI matrix on staging->main promotion (needs #10/#11)

Note: required checks on a private repo require GitHub Pro; with the repo
public they work on the free plan.
NOTE
