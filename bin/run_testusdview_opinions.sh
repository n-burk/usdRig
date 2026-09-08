#!/bin/bash
# bin/run_testusdview_opinions.sh -- headless end-to-end test of the Layer
# Opinions panel (tests/testUsdviewLayerOpinions.py) on examples/ArmRig.usda.
#
# Asserts the RigExec menu item is registered, that the panel follows
# usdview's prim selection, and that an edit, a refused parse and a delete
# each land on the layer the clicked row names -- with undo restoring it.
# Prints RIGEXEC_LAYER_OPINIONS_OK.
#
# Usage: bin/run_testusdview_opinions.sh [rendererDisplayName]   (e.g. Embree)
# Set RIGEXEC_OPINIONS_SHOT=/path.png to keep a window grab.
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

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewLayerOpinions.py" \
     "$@" "$STAGE"
