#!/bin/bash
# bin/run_testusdview_viewcube.sh -- headless end-to-end test of the
# Maya-style view cube (tests/testUsdviewViewCube.py) on
# examples/ArmShotAnim.usda.
#
# Drives synthetic mouse events through the cube's own projected region
# points and asserts what landed on the stage: the free camera's view
# direction and up vector after face/edge/corner clicks, hover,
# the animated orbit, drag-to-tumble, Alt-press propagation, the camera
# prim round trip, Home and the menu toggle. Prints RIGEXEC_VIEWCUBE_OK.
#
# Usage: bin/run_testusdview_viewcube.sh [rendererDisplayName]   (e.g. Embree)
# Set RIGEXEC_VIEWCUBE_SHOT=/path.png to keep a window grab.
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

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewViewCube.py" \
     "$@" "$STAGE"
