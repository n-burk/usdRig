#!/bin/bash
# bin/build_rigexec.sh -- configure, build, and test RigExec against the
# installed OpenUSD. The POSIX twin of build_rigexec.bat.
#
# Usage: bin/build_rigexec.sh [--no-test]
#
# JOBS caps parallel compile jobs (default 8): the rigExec translation
# units each eat ~1GB, so Ninja's CPU-count default OOMs the compiler on
# smaller machines. build_rigexec.bat honors the same variable.
set -euo pipefail
: "${JOBS:=8}"
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python

# CMAKE_PREFIX_PATH is not optional: without it pxrConfig's
# find_dependency(OpenSubdiv 3.6.1) can resolve against an older OpenSubdiv
# elsewhere on the machine and the configure fails with a version mismatch.
#
# Branched rather than passed through an array: expanding an empty array
# with set -u is an "unbound variable" error under bash 3.2 (macOS
# /bin/bash); see bin/usdview.sh.
if [ -n "${RIGEXEC_MOVER_PLUGIN_DIRS:-}" ]; then
    cmake -S "$RIG" -B "$RIG/build" -G Ninja \
          -DCMAKE_BUILD_TYPE=Release \
          -DUSD_INSTALL_DIR="$USD" \
          -DCMAKE_PREFIX_PATH="$USD" \
          "-DRIGEXEC_MOVER_PLUGIN_DIRS=$RIGEXEC_MOVER_PLUGIN_DIRS"
else
    cmake -S "$RIG" -B "$RIG/build" -G Ninja \
          -DCMAKE_BUILD_TYPE=Release \
          -DUSD_INSTALL_DIR="$USD" \
          -DCMAKE_PREFIX_PATH="$USD"
fi
cmake --build "$RIG/build" -j "$JOBS"

if [ "${1:-}" = "--no-test" ]; then
    exit 0
fi
# CTEST_PARALLEL_LEVEL is the knob ctest itself reads; when set, it wins.
# CTEST_JOBS is the fallback. A local quick tier is `ctest -LE slow`;
# this script still runs every test, including the slow label.
if [ -n "${CTEST_PARALLEL_LEVEL:-}" ]; then
    CTEST_JOBS="$CTEST_PARALLEL_LEVEL"
elif [ -z "${CTEST_JOBS:-}" ]; then
    CTEST_JOBS="$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)"
fi
ctest --test-dir "$RIG/build" -j "$CTEST_JOBS" --output-on-failure
