#!/bin/bash
# bin/run_testusdview_arcs.sh -- headless end-to-end test of the guided
# composition-arc flows (tests/testUsdviewCompositionArcs.py) on
# examples/ArmRig.usda.
#
# Asserts the Layer Opinions panel's right-click menu offers every arc, that
# the dialog builds its form from the arc's fields and re-previews as they
# change, that an illegal request disables Author and says why, and that
# authoring reaches the stage and lands on the panel's undo stack.
#
# Then the same for the arcs already there: that each is listed as a row of
# its own with the flow, the removal and the two moves on its menu, that
# reopening one prefills the dialog with no layer or position field, and
# that applying it replaces the arc where it sits.
# Prints RIGEXEC_COMPOSITION_ARCS_OK.
#
# Usage: bin/run_testusdview_arcs.sh [rendererDisplayName]   (e.g. Embree)
# Set RIGEXEC_ARCS_SHOT=/path.png to keep a grab of the reference flow.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/ArmRig.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewCompositionArcs.py" \
     "$@" "$STAGE"
