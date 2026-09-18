#!/bin/bash
# bin/run_testusdview_noodles_icons.sh -- headless end-to-end test of NODE
# TITLE-BAR ICONS (tests/testUsdviewNoodlesIcons.py) on
# tests/fixtures/noodles_icons.usda. The POSIX twin of
# run_testusdview_noodles_icons.bat, which carries the full rationale.
#
# In short: every docs example authors
# `uniform asset ui:nodegraph:node:icon = @../../icons/<node>.png@`, and the
# editor drew the built-in box on every node instead. The node factory set a
# Python-side attribute AFTER the last sync to the C++ NodeData, so
# titleIconPath -- the only field the C++ icon producer reads -- stayed empty.
#
# Asserts the authored icon reaches titleIconPath, that a relative asset path
# anchors to the LAYER and not the working directory, that the resolved file
# loads as an image, that a missing file anchors and then falls back silently,
# that a prim with no opinion keeps the default, and -- the end-to-end part --
# that the title area's pixels actually change when the icon is cleared.
# Prints RIGEXEC_NOODLES_ICONS_OK.
#
# Usage: bin/run_testusdview_noodles_icons.sh [rendererDisplayName]
# Set RIGEXEC_NOODLES_SHOT=/path.png to keep a grab of the open editor.
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

STAGE="$RIG/tests/fixtures/noodles_icons.usda"
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewNoodlesIcons.py" "$@" "$STAGE"
