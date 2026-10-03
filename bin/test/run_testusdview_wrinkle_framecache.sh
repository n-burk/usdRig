#!/bin/bash
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"
rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
unset RIGEXEC_EVALUATION_MODE RIGEXEC_DYNAMIC_RUNS_PROGRAM
export RIGEXEC_FRAME_CACHE=on RIGEXEC_FRAME_CACHE_VERIFY=0
export RIGEXEC_ENABLE_PARALLEL_EVAL=1
exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewWrinkleFrameCache.py" "$RIG/docs/examples/wrinkle_mover.usda"
