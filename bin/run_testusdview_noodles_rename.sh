#!/bin/bash
# bin/run_testusdview_noodles_rename.sh -- headless end-to-end test of
# RENAMING A PRIM FROM THE NODE GRAPH (tests/testUsdviewNoodlesRename.py) on
# tests/fixtures/noodles_rename.usda. The POSIX twin of
# run_testusdview_noodles_rename.bat, which carries the full rationale.
#
# In short: double-clicking a node did nothing because usdview ran the
# older pxr.UsdNoodles from the USD install; and once the in-repo editor
# ran, the prim browser did not follow the rename. This registers the
# in-repo editor as the launchers do, drives double-click, typing and
# Return with real Qt events, and checks the prim, the graph and usdview's
# selection. Prints RIGEXEC_NOODLES_RENAME_OK.
#
# Usage: bin/run_testusdview_noodles_rename.sh [rendererDisplayName]
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

STAGE="$RIG/tests/fixtures/noodles_rename.usda"
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" \
     --testScript "$RIG/tests/testUsdviewNoodlesRename.py" "$@" "$STAGE"
