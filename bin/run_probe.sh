#!/bin/bash
# bin/run_probe.sh -- run a python probe script in the RigExec plugin
# environment. The POSIX twin of run_probe.bat.
#
# Usage: bin/run_probe.sh <script.py> [args...]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
if [ $# -eq 0 ]; then
    echo "Usage: bin/run_probe.sh <script.py> [args...]" >&2
    exit 2
fi
exec "$PY" "$@"
