#!/usr/bin/env bash
# Make sure the Python modules the evaluation scripts import exist in the
# deps image, installing only what is missing.
#
# The pinned deps image is older than the Dockerfile line that adds
# rosbags/scipy (docker/deps.Dockerfile, "pip-install rosbags and scipy used
# by build_map/eval scripts"), so it ships without them. Re-pinning needs a
# GHCR push; until then every entry point tops the image up in place. This is
# a no-op on a correctly pinned image, and it is safe to call from anywhere
# inside the container.
#
#   bash infra/ensure_py_deps.sh
set -euo pipefail

# module name == distribution name for all four.
MODULES="${PY_MODULES:-numpy scipy rosbags pytest}"

missing=()
for module in $MODULES; do
  python3 -c "import $module" >/dev/null 2>&1 || missing+=("$module")
done

if [ ${#missing[@]} -eq 0 ]; then
  exit 0
fi

echo "ensure_py_deps: installing ${missing[*]}" >&2
if ! python3 -m pip install --no-cache-dir --quiet --disable-pip-version-check "${missing[@]}"; then
  # Never worse than before the helper existed: let the command run and fail
  # with its own ModuleNotFoundError if it actually needs the module.
  echo "ensure_py_deps: install failed (offline?); continuing without ${missing[*]}" >&2
fi
exit 0
