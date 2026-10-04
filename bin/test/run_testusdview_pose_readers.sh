#!/usr/bin/env bash
# testusdview: the Shape Editor's pose-reader overlay (falloff cones,
# twist fans, translation spheres) draws over the viewport, live.
#
# Usage: run_testusdview_pose_readers.sh [renderer]
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$here/../_env.sh"
if [ -f "$RIG/build/CMakeCache.txt" ]; then
    cmake --build "$RIG/build" >/dev/null
fi
export PXR_PLUGINPATH_NAME="$PXR_PLUGINPATH_NAME:$RIG/plugin/shapeEditor"
export PYTHONPATH="$RIG/plugin/shapeEditor:${PYTHONPATH:-}"
renderer=()
if [ $# -gt 0 ]; then renderer=(--renderer "$1"); fi
"$PY" "$USD/bin/testusdview" --testScript \
    "$RIG/tests/testUsdviewPoseReaderViz.py" "${renderer[@]}" \
    "$RIG/examples/biped/Biped_stack.usda"
