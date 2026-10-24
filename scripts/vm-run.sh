#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_DIR/infra/common.sh"

ROOT=/ws/brightdrive
JOBS=/ws/jobs
SSH="$REPO_DIR/infra/vm-ssh.sh"

usage() {
  cat <<'EOF'
usage: scripts/vm-run.sh [--no-sync] 'command'
       scripts/vm-run.sh --sync-only
       scripts/vm-run.sh --fg [--no-sync] 'command'
       scripts/vm-run.sh --status
       scripts/vm-run.sh --logs <job-id|/path/to/log>

Deploys the committed git HEAD to the VM and runs commands there over IAP.
Long commands run detached in /ws/jobs/<job-id>/ with console.log, exit_code,
pid, and run.sh; the wrapper prints the job id immediately.
EOF
}

mode="detach"
do_sync=1
log_target=""

while [ $# -gt 0 ]; do
  case "$1" in
    --sync-only) mode="sync"; shift ;;
    --fg) mode="fg"; shift ;;
    --status) mode="status"; shift ;;
    --logs)
      if [ $# -lt 2 ]; then usage; exit 2; fi
      mode="logs"; log_target="$2"; shift 2
      ;;
    --no-sync) do_sync=0; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done

ensure_vm() {
  local state
  state=$(instance_state)
  if [ "$state" = RUNNING ]; then
    return 0
  fi
  if [ "$state" != TERMINATED ] && [ "$state" != SUSPENDED ] && [ "$state" != MISSING ]; then
    echo "==> vm is $state; waiting for it to settle" >&2
    for _ in $(seq 1 60); do
      state=$(instance_state)
      if [ "$state" = RUNNING ] || [ "$state" = TERMINATED ]; then
        break
      fi
      sleep 5
    done
  fi
  if [ "$state" != RUNNING ]; then
    "$REPO_DIR/infra/vm-start.sh" >&2
  fi
}

sync_repo() {
  local sha
  if ! git -C "$REPO_DIR" diff --quiet || ! git -C "$REPO_DIR" diff --cached --quiet; then
    echo "==> warning: uncommitted changes are not deployed" >&2
  fi
  sha=$(git -C "$REPO_DIR" rev-parse HEAD)
  echo "==> deploying $sha" >&2
  git -C "$REPO_DIR" archive --format=tar "$sha" \
    | "$SSH" "sudo rm -rf '$ROOT' && mkdir -p '$ROOT' && tar -xf - -C '$ROOT' && printf '%s\n' '$sha' > '$ROOT/.deployed-sha'"
}

make_slug() {
  printf '%s' "$1" | head -c 40 | tr -cs 'a-zA-Z0-9' '-' | sed 's/^-*//; s/-*$//'
}

run_detached() {
  local cmd="$1" stamp slug job_id job_dir tmp
  stamp=$(date -u +%Y%m%dT%H%M%SZ)
  slug=$(make_slug "$cmd")
  [ -n "$slug" ] || slug=cmd
  job_id="$stamp-$slug"
  job_dir="$JOBS/$job_id"
  tmp=$(mktemp)
  {
    printf 'echo $$ > %s\n' "$job_dir/pid"
    printf '(\n'
    printf 'set -o pipefail\n'
    printf '%s\n' "$cmd"
    printf ')\n'
    printf 'rc=$?\n'
    printf 'echo "$rc" > %s\n' "$job_dir/exit_code"
  } >"$tmp"
  "$SSH" "mkdir -p '$job_dir'"
  "$SSH" "cat > '$job_dir/run.sh'" <"$tmp"
  rm -f "$tmp"
  "$SSH" "cd '$ROOT' && nohup setsid bash '$job_dir/run.sh' > '$job_dir/console.log' 2>&1 < /dev/null & echo \$! > '$job_dir/launcher-pid'"
  echo "$job_id"
  echo "==> follow: scripts/vm-run.sh --logs $job_id" >&2
}

run_foreground() {
  local cmd="$1" tmp script
  tmp=$(mktemp)
  script="/tmp/vm-run-$RANDOM.sh"
  printf 'set -o pipefail\n%s\n' "$cmd" >"$tmp"
  "$SSH" "cat > '$script'" <"$tmp"
  rm -f "$tmp"
  "$SSH" "cd '$ROOT' && bash '$script'; rc=\$?; rm -f '$script'; exit \$rc"
}

show_status() {
  "$SSH" 'for d in /ws/jobs/*/; do
    [ -d "$d" ] || continue
    n=${d%/}; n=${n##*/}
    rc=$(cat "$d/exit_code" 2>/dev/null || true)
    p=$(cat "$d/pid" 2>/dev/null || true)
    if [ -n "$p" ] && kill -0 "$p" 2>/dev/null; then s=RUNNING
    elif [ -n "$rc" ]; then s="rc=$rc"
    else s=UNKNOWN; fi
    printf "%-30s %-10s %s\n" "$n" "$s" "$(head -1 "$d/run.sh" 2>/dev/null)"
  done'
}

show_logs() {
  local target="$1"
  case "$target" in
    /*) : ;;
    *) target="$JOBS/$target/console.log" ;;
  esac
  "$SSH" "tail -n +1 -f '$target'"
}

cmd="$*"
ensure_vm

case "$mode" in
  sync) sync_repo ;;
  logs)
    [ -n "$log_target" ] || { usage; exit 2; }
    show_logs "$log_target"
    ;;
  status) show_status ;;
  fg)
    [ -n "$cmd" ] || { usage; exit 2; }
    [ "$do_sync" = 1 ] && sync_repo
    run_foreground "$cmd"
    ;;
  detach)
    [ -n "$cmd" ] || { usage; exit 2; }
    [ "$do_sync" = 1 ] && sync_repo
    run_detached "$cmd"
    ;;
esac
