#!/bin/bash
# bin/run_usdview_touchpose.sh -- INTERACTIVE usdview with TouchPose.
# The POSIX twin of run_usdview_touchpose.bat.
#
# Usage: bin/run_usdview_touchpose.sh [stage.usda] [renderer | usdview flags...]
#   stage defaults to examples/biped/Biped_all.usda: the touch
#   regions stacked over the LAYERED rig, which is also the stage the
#   Control Picker resolves against, so both panels work on it.
#   Biped_touch.usda is the same regions over the FLAT rig, and
#   Biped_touch_regions.usda is the touch data alone -- that one shows
#   nothing opened by itself, being `over`s with no geometry under them.
#
# Then: RigExec -> Animation Editors -> TouchPose, tick the box, and hover the character. The
# region under the cursor lights up; click it and the control that owns
# it becomes usdview's selection, so the Avar Editor and the viewport
# gizmo follow. While the box is ticked the MESH is not selectable -- a
# click on unpainted skin selects nothing rather than picking body_geo.
# Untick it (or close the window) and usdview's own picking is back
# exactly as it was. Alt still drives the camera, untouched.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$USDVIEW"

# APPENDED, never replaced: _env.sh is the canonical environment.
export PXR_PLUGINPATH_NAME="${PXR_PLUGINPATH_NAME:-}:$RIG/plugin/touchPose"
export PYTHONPATH="$RIG/plugin/touchPose:${PYTHONPATH:-}"

STAGE="$RIG/examples/biped/Biped_all.usda"
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ] && [ -f "$1" ]; then
    STAGE="$1"
    shift
fi
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$USDVIEW" "$@" "$STAGE"
