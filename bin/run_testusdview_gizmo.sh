#!/bin/bash
# bin/run_testusdview_gizmo.sh -- headless end-to-end test of the viewport
# gizmo toolbar (tests/testUsdviewGizmo.py) on examples/ArmShotAnim.usda.
#
# Drives synthetic mouse and key events through the gizmo's own projected
# handle positions and asserts what landed on the stage: the avars, the
# undo/redo round trip, Default vs Animation, Pivot vs Pose, a plain
# xform's op stack, and the Maya parity behaviours (planar handles,
# middle-drag repeat, step snap, view ring, gimbal, free rotate, the
# scale ratio rule, Preserve Children, hotkeys). Prints RIGEXEC_GIZMO_OK.
#
# Usage: bin/run_testusdview_gizmo.sh [rendererDisplayName]   (e.g. Embree)
# Set RIGEXEC_GIZMO_SHOT=/path.png to keep a window grab.
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

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewGizmo.py" \
     "$@" "$STAGE"
