#!/bin/bash
# Verify hairline control picking, screen tolerance and mesh occlusion.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/../_env.sh"
rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build
exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewControlPicking.py" \
    "$@" "$RIG/tests/fixtures/controlPicking.usda"
