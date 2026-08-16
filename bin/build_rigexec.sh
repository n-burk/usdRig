#!/bin/bash
# bin/build_rigexec.sh -- configure, build, and test RigExec against the
# installed OpenUSD. The POSIX twin of build_rigexec.bat.
#
# Usage: bin/build_rigexec.sh [--no-test]
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python

# CMAKE_PREFIX_PATH is not optional: without it pxrConfig's
# find_dependency(OpenSubdiv 3.6.1) can resolve against an older OpenSubdiv
# elsewhere on the machine and the configure fails with a version mismatch.
cmake -S "$RIG" -B "$RIG/build" -G Ninja \
      -DCMAKE_BUILD_TYPE=Release \
      -DUSD_INSTALL_DIR="$USD" \
      -DCMAKE_PREFIX_PATH="$USD"
cmake --build "$RIG/build"

if [ "${1:-}" = "--no-test" ]; then
    exit 0
fi
ctest --test-dir "$RIG/build" --output-on-failure
