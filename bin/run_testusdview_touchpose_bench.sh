#!/bin/bash
# bin/run_testusdview_touchpose_bench.sh -- TouchPose latency benchmark
# (tests/testUsdviewTouchPoseBench.py) in a real usdview. Not pass/fail:
# prints hover, click, external-selection and pose-change costs, each as
# the call alone, the call plus one synchronous frame, and (for hover) the
# call plus the Qt event loop. See the script's header for what each
# number means.
#
# Usage: bin/run_testusdview_touchpose_bench.sh [stage.usda]
#   stage defaults to examples/biped/Biped_all.usda.
# TOUCHPOSE_BENCH_REPEATS sets the samples per measurement (default 30).
#
# No build step, for the same reason as run_testusdview_touchpose.sh.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"

export PXR_PLUGINPATH_NAME="${PXR_PLUGINPATH_NAME:-}:$RIG/plugin/touchPose"
export PYTHONPATH="$RIG/plugin/touchPose:${PYTHONPATH:-}"
export TOUCHPOSE_PLUGIN_DIR="$RIG/plugin/touchPose"

STAGE="$RIG/examples/biped/Biped_all.usda"
if [ $# -gt 0 ] && [ -f "$1" ]; then
    STAGE="$1"
    shift
fi
rigexec_require_stage "$STAGE"

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewTouchPoseBench.py" "$@" "$STAGE"
