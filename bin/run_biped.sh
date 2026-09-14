#!/usr/bin/env bash
# Runs one of the tools/biped scripts in the RigExec plugin environment.
# The POSIX twin of run_biped.bat; see that file for why it is needed.
#
# Usage: run_biped.sh <script> [args...]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_env.sh
. "$here/_env.sh"

if [ $# -eq 0 ]; then
    echo "Usage: run_biped.sh <script> [args...]" >&2
    echo "       scripts available in $RIG/tools/biped:" >&2
    for f in "$RIG"/tools/biped/*.py; do
        echo "         $(basename "$f" .py)" >&2
    done
    exit 2
fi

script="$1"; shift
target="$RIG/tools/biped/$script"
[ -f "$target" ] || target="$RIG/tools/biped/$script.py"
if [ ! -f "$target" ]; then
    echo "ERROR: no such script: $script (looked in $RIG/tools/biped)" >&2
    exit 1
fi
exec "$PY" "$target" "$@"
