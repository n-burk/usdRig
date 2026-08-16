#!/bin/bash
# bin/test_muse.sh — verify the Muse assistant, headless then inside usdview.
#
#   1. tests/testMuseAgent.py    message shaping, tool loop, PNG camera metadata
#                                (no Qt, no display, no network, no API key)
#   2. tests/testUsdviewMuse.py  the panel driving a REAL stage in a real
#                                usdview: the model's edits must land, the
#                                viewport capture must be a real image, and an
#                                attached screenshot's camera must reach it
#
# Neither test talks to a model — both script the SDK — so no API key is used
# or needed. Passing prints MUSE_AGENT_OK and MUSE_USDVIEW_OK.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
RIG="$(cd "$SCRIPT_DIR/.." && pwd)"
USD="${USD:-/Users/burkard/work/usd-install}"
VENV="${VENV:-/Users/burkard/work/usd-pr4156-venv}"
PY="$VENV/bin/python"
STAGE="${1:-$RIG/examples/ArmShotAnim.usda}"

if [ ! -x "$PY" ]; then
  echo "ERROR: venv python not found at $PY (set VENV=...)" >&2
  exit 1
fi

export DYLD_LIBRARY_PATH="$USD/lib:$RIG/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export PYTHONPATH="$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant:$USD/lib/python3.11/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PXR_PLUGINPATH_NAME="$RIG/build/usd/rigExecSchema/resources:$RIG/build/usd/rigExecImaging/resources:$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant"

echo "== headless: agent core =="
"$PY" "$RIG/tests/testMuseAgent.py"

echo ""
echo "== in usdview: panel against a live stage =="
"$PY" "$USD/bin/testusdview" --testScript "$RIG/tests/testUsdviewMuse.py" "$STAGE"

# Opt-in: the only test that proves asking in English changes the stage.
# Makes real API calls with the configured key, so it is not run by default.
if [ "${MUSE_LIVE:-0}" = "1" ]; then
  echo ""
  echo "== LIVE: real model, real stage (MUSE_LIVE=1) =="
  "$PY" "$USD/bin/testusdview" --testScript "$RIG/tests/testUsdviewMuseLive.py" "$STAGE"
else
  echo ""
  echo "(set MUSE_LIVE=1 to also run the live end-to-end test against the real model)"
fi
