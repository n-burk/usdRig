#!/bin/bash
# Print the CTest targets a source change affects.
#
# Dry-run by default. This is not part of CI. --run builds the direct
# executables and runs the direct test regex; that needs a configured
# build tree and an OpenUSD prefix, which the dry-run does not.
#
# Usage:
#   bin/test_changed.sh libs/rigExecMath/solvers.cpp
#   bin/test_changed.sh --diff origin/rigexec-format-v4
#   bin/test_changed.sh --run libs/rigExecBinary/format.cpp
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUN=0
ARGS=()
for arg in "$@"; do
    if [ "$arg" = "--run" ]; then
        RUN=1
    else
        ARGS+=("$arg")
    fi
done

if [ "${#ARGS[@]}" -eq 0 ]; then
    echo "usage: bin/test_changed.sh [--run] [--diff REV] [paths...]" >&2
    exit 2
fi

python3 "$ROOT/tools/select_affected_tests.py" "${ARGS[@]}"

if [ "$RUN" -eq 0 ]; then
    exit 0
fi

# The selector prints the direct build and ctest lines. Re-run it and
# execute those two, from the repo root, against ./build.
mapfile -t lines < <(python3 "$ROOT/tools/select_affected_tests.py" "${ARGS[@]}")
build_line=""
ctest_line=""
for line in "${lines[@]}"; do
    case "$line" in
        "  cmake --build "*) build_line="${line#  }" ;;
        "  ctest "*) ctest_line="${line#  }" ;;
    esac
done

if [ -z "$build_line" ] || [ -z "$ctest_line" ]; then
    echo "test_changed: selector produced no build/ctest command" >&2
    exit 1
fi

cd "$ROOT"
# shellcheck disable=SC2086
eval "$build_line"
# shellcheck disable=SC2086
eval "$ctest_line"
