#!/bin/bash
# bin/test/run_testusdview_hover_picker.sh -- headless end-to-end test of
# the hover picker (tests/testUsdviewHoverPicker.py) on
# examples/biped/Biped_stack.usda.
#
# Sends real mouse events to the viewport and asserts the picker's tabs
# select, move, scale, collapse and fade as they should, that everything
# else stays the viewport's, and that the layout is saved per user. Puts
# the user's own saved layout back. Prints HOVER_PICKER_OK.
#
# Usage: bin/test/run_testusdview_hover_picker.sh [rendererDisplayName]
# Set RIGEXEC_HOVER_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/biped/Biped_stack.usda"
rigexec_require_stage "$STAGE"

# A bare (non-flag) argument is the renderer display name, matching the
# other runners. Prepending to the positional parameters rather than
# collecting an array keeps this working under bash 3.2, where expanding
# an empty array with set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewHoverPicker.py" \
     "$@" "$STAGE"
