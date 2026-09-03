#!/bin/bash
# bin/run_testusdview_graph.sh -- headless end-to-end test of the usdview
# graph editor (tests/testUsdviewGraphEditor.py) on examples/ArmShotAnim.usda.
#
# Drives synthetic mouse and key events at the pixels the graph canvas
# itself reports for its keys and tangent handles, and asserts what
# landed on the stage: the curve set for a prim and for a property
# selection, a key drag written into the session layer, the Ctrl+Z round
# trip, insert and delete, the Maya tangent types, a tangent handle drag
# weighted and not, break/unify, the infinity mapping, the ruler scrub
# and marquee selection. Prints RIGEXEC_GRAPH_OK.
#
# Usage: bin/run_testusdview_graph.sh [rendererDisplayName]   (e.g. Embree)
# Set RIGEXEC_GRAPH_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/ArmShotAnim.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewGraphEditor.py" \
     "$@" "$STAGE"
