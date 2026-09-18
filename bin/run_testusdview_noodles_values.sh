#!/bin/bash
# bin/run_testusdview_noodles_values.sh -- headless end-to-end test of
# READING AND EDITING ATTRIBUTE VALUES ON THE NODE ROWS
# (tests/testUsdviewNoodlesValues.py) on tests/fixtures/noodles_values.usda.
# The POSIX twin of run_testusdview_noodles_values.bat, which carries the
# full rationale.
#
# In short: the value cells need a laid-out node with real row geometry, a
# real Qt event loop and a real undo stack, so none of what they do is
# visible to the unit tests in plugin/usdNoodles/testenv. This registers the
# in-repo editor as the launchers do, then drives clicks, drags, typing and
# the allowedTokens popup with real Qt events -- including the one
# regression that matters most: the outer tenth of each row must STILL start
# a connection drag. Prints RIGEXEC_NOODLES_VALUES_OK.
#
# Usage: bin/run_testusdview_noodles_values.sh [rendererDisplayName]
# Set RIGEXEC_NOODLES_SHOT=/path.png to keep grabs of the editor.
#
# No build step: build first so build/python/UsdNoodles is current.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"

if [ ! -f "$RIG/build/python/UsdNoodles/plugInfo.json" ]; then
    echo "ERROR: build/python/UsdNoodles is not staged; run bin/build_rigexec.sh" >&2
    exit 1
fi
rigexec_register_usdnoodles

STAGE="$RIG/tests/fixtures/noodles_values.usda"
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewNoodlesValues.py" "$@" "$STAGE"
