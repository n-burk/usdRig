#!/bin/bash
# bin/run_testusdview_touchpose.sh -- headless end-to-end test of the
# TouchPose pick loop (tests/testUsdviewTouchPose.py) on
# examples/biped/Biped_touch.usda. The POSIX twin of
# run_testusdview_touchpose.bat, which carries the full rationale.
#
# In short, three of TouchPose's claims cannot be checked from a script:
# the HIGHLIGHT has to be seen (a displayColor on this mesh is perfectly
# correct at the terminal scene index and moves zero pixels, because
# body_geo is bound to UsdPreviewSurface -- spike R2), the PICK has to
# follow the DEFORMED points that live in Hydra rather than on the stage,
# and the EVENT FILTER has to be reached the way Qt reaches it.
#
# Asserts TouchPose is registered and merged into the existing RigExec
# menu; every region binds a real drivable prim; a known face's centroid
# projected to screen and clicked selects that region's control and NOT
# the mesh; shift adds; the highlight moves a measured share of the
# region's own screen area; hips +12 cm moves the region and the pick
# follows; a click that misses the character is not consumed; a click on
# unpainted skin IS consumed and selects nothing; and turning the mode
# off restores usdview's own picking. Prints RIGEXEC_TOUCHPOSE_OK.
#
# Usage: bin/run_testusdview_touchpose.sh [stage.usda] [rendererDisplayName]
# Set TOUCHPOSE_SHOT=/path.png to keep a frame grab of the highlight.
#
# Deliberately no build step: an open usdview holds a lock on the imaging
# library that would fail the link. TouchPose's native half lives in
# rigExecImaging (touchPose*.cpp) -- build first when that has changed.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"

# APPENDED, never replaced: _env.sh is the canonical environment and
# points PXR_PLUGINPATH_NAME at the GENERATED schema resources.
export PXR_PLUGINPATH_NAME="${PXR_PLUGINPATH_NAME:-}:$RIG/plugin/touchPose"
export PYTHONPATH="$RIG/plugin/touchPose:${PYTHONPATH:-}"

# testusdview `exec`s the script, so it has no __file__ to locate the
# package from. Handed over explicitly rather than guessed at.
export TOUCHPOSE_PLUGIN_DIR="$RIG/plugin/touchPose"

STAGE="$RIG/examples/biped/Biped_touch.usda"
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ] && [ -f "$1" ]; then
    STAGE="$1"
    shift
fi
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewTouchPose.py" "$@" "$STAGE"
