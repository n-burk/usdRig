#!/bin/bash
# bin/run_python_tests.sh -- headless Qt-free tests for the usdview plugin
# modules (undo stack, gizmo math, gizmo screen helpers).
#
# Usage: bin/run_python_tests.sh [test_name ...]   (default: all)
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"
rigexec_require_python

# The native evaluator binding is optional (built in build-python, not
# build); test_gizmo_math compares against it when importable.
export PYTHONPATH="$RIG/build-python/python:$PYTHONPATH"
SCHEMA="$RIG/build/usd/rigExecSchema/resources"

TESTS=("$@")
if [ ${#TESTS[@]} -eq 0 ]; then
    TESTS=(test_rigexec_undo test_gizmo_math test_gizmo_screen
           test_gizmo_settings test_gizmo_drag)
fi

# Only these two read argv[1], to Plug-register the generated schema.
# The other three never touch sys.argv -- they need neither a schema nor
# a build -- so handing them $SCHEMA would tell a reader otherwise.
SCHEMA_TESTS=" test_rigexec_undo test_gizmo_math "
for t in "${TESTS[@]}"; do
    echo "== $t"
    case "$SCHEMA_TESTS" in
        *" $t "*) "$PY" "$RIG/tests/python/$t.py" "$SCHEMA" ;;
        *)        "$PY" "$RIG/tests/python/$t.py" ;;
    esac
done
