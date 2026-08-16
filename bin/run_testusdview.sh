#!/bin/bash
# bin/run_testusdview.sh -- headless verification of the live RigExec Hydra integration
# The POSIX twin of run_testusdview.bat.
#
# Usage: bin/run_testusdview.sh [rendererDisplayName]   (e.g. Embree)
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/ArmShotAnim.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the .bat
# helpers. Prepending to the positional parameters rather than collecting an
# array keeps this working under bash 3.2, where expanding an empty array with
# set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewRigExec.py" \
     "$@" "$STAGE"
