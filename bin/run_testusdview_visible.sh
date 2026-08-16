#!/bin/bash
# bin/run_testusdview_visible.sh -- curvenet visible verification against the puppetA
# character. The POSIX twin of run_testusdview_visible.bat.
#
# Usage: bin/run_testusdview_visible.sh [stage.usda]
#   stage defaults to the puppetA curvenet beside the repository, which is not
#   part of this checkout -- pass a stage explicitly if you do not have it.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"

STAGE="${1:-$(cd "$RIG/.." && pwd)/chars/puppetA/puppetA_curvenet.usda}"
rigexec_require_stage "$STAGE"

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewCurvenetVisible.py" "$STAGE"
