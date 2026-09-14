#!/bin/bash
# bin/run_testusdview_noodles_touch.sh -- headless end-to-end test of
# TOUCHPOSE IN THE NODE GRAPH (tests/testUsdviewNoodlesTouch.py) on
# examples/biped/Biped_all.usda. The POSIX twin of
# run_testusdview_noodles_touch.bat, which carries the full rationale.
#
# In short: usdNoodles builds its graph from the prims a load path hands
# it, and TouchPose was invisible to the one usdview uses when it opens a
# stage -- measured, NodeGraphStage.load on Biped_all.usda produced one
# node (/Biped) and zero links, because it reads root children and the 98
# regions hang off /Biped/TouchPose.
#
# Asserts every authored region becomes a node, every touchpose:control
# is drawn as a relationship link onto a RigExec prim that is itself a
# node in the graph, the regions are laid out (none at the origin, none
# overlapping), the face array costs a pin row and not a rendered value,
# and selecting the group expands to the regions and their controls.
# Prints RIGEXEC_NOODLES_TOUCH_OK.
#
# Usage: bin/run_testusdview_noodles_touch.sh [stage.usda] [rendererDisplayName]
# Set RIGEXEC_NOODLES_SHOT=/path.png to keep a window grab.
#
# Deliberately no build step and no Noodles panel: no C++ of its own, an
# open usdview holds a lock on the imaging library, and a headless runner
# must not load an extra panel into the app it asserts against.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"

STAGE="$RIG/examples/biped/Biped_all.usda"
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ] && [ -f "$1" ]; then
    STAGE="$1"
    shift
fi
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewNoodlesTouch.py" "$@" "$STAGE"
