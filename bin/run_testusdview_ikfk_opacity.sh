#!/bin/bash
# bin/run_testusdview_ikfk_opacity.sh -- headless end-to-end test that the
# IK/FK switch fades the inactive control set
# (tests/testUsdviewIkFkOpacity.py) on biped_rig_v3.usda.
#
# Why it exists: `guide:displayOpacity.connect = <dial>` compiled and
# changed nothing, because UsdAttribute::Get never follows a connection.
# The imaging bridge now does, and this asserts the DRAWN opacity from the
# terminal Hydra scene index -- never the authored attribute -- as
# tools/biped/params.py wires each limb's controls (into the session
# layer; the file is never written) and the limb's avars:ikfk is driven on
# the param node, a different prim from every guide it fades. The IK
# guides come up with the dial and the FK guides go down against it,
# floored at guide:displayOpacityMin. Prints RIGEXEC_IKFK_OPACITY_OK.
#
# Usage: bin/run_testusdview_ikfk_opacity.sh [rendererDisplayName]  (e.g. Embree)
# Set RIGEXEC_IKFK_OPACITY_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/biped_rig_v3.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewIkFkOpacity.py" \
     "$@" "$STAGE"
