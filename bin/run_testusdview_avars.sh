#!/bin/bash
# bin/run_testusdview_avars.sh -- headless end-to-end test of the Avar
# Editor panel (tests/testUsdviewAvarEditor.py) on biped_full2.usda.
#
# Asserts the RigExec menu item is registered, that the panel follows
# usdview's prim selection, and that driving a row's spin box or slider
# through the panel's own Qt signals moves the EVALUATED rig by the amount
# the maths says (the elbow's chord under a 30 degree FK rotate, the IK/FK
# blend under the custom ikfk dial), republishes to Hydra, keeps a
# file-animated channel's other keys, and lands as one entry per edit on
# the shared undo stack. Prints RIGEXEC_AVAR_EDITOR_OK.
#
# Usage: bin/run_testusdview_avars.sh [rendererDisplayName]   (e.g. Embree)
# Set RIGEXEC_AVARS_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/biped_full2.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewAvarEditor.py" \
     "$@" "$STAGE"
