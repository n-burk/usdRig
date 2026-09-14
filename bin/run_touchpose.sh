#!/usr/bin/env bash
# Runs one of the plugin/touchPose scripts in the RigExec plugin environment.
# The POSIX twin of run_touchpose.bat; see that file for why it is needed.
#
# Usage: run_touchpose.sh <script> [args...]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_env.sh
. "$here/_env.sh"

if [ $# -eq 0 ]; then
    echo "Usage: run_touchpose.sh <script> [args...]" >&2
    echo "       scripts available in $RIG/plugin/touchPose:" >&2
    for f in "$RIG"/plugin/touchPose/*.py "$RIG"/plugin/touchPose/spikes/*.py; do
        [ -f "$f" ] && echo "         $(basename "$f" .py)" >&2
    done
    exit 2
fi

script="$1"; shift
for candidate in \
    "$RIG/plugin/touchPose/$script" \
    "$RIG/plugin/touchPose/$script.py" \
    "$RIG/plugin/touchPose/spikes/$script" \
    "$RIG/plugin/touchPose/spikes/$script.py"; do
    if [ -f "$candidate" ]; then target="$candidate"; break; fi
done
if [ -z "${target:-}" ]; then
    echo "ERROR: no such script: $script (looked in plugin/touchPose and its spikes)" >&2
    exit 1
fi
exec "$PY" "$target" "$@"
