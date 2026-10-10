#!/bin/bash
# Compile the runtime, the op scheduler, and the binary format with no
# OpenUSD include path. Fails if a translation unit needs a pxr header,
# or if an undefined symbol is an OpenUSD type.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

CXXFLAGS=(
    -std=c++17 -O2 -fno-fast-math -ffp-contract=off -pthread
    -I "$ROOT/libs"
    -I "$ROOT/thirdparty/flatbuffers/include"
    -I "$ROOT/third_party/lzma/C"
)
CFLAGS=(
    -std=c11 -O2 -DZ7_ST -DRIGEXEC_LZMA_PORTABLE_SCALAR
    -I "$ROOT/third_party/lzma/C"
)

objs=()
compile_one() {
    local src="$1"
    local obj="$WORK/$(basename "${src%.*}").o"
    if [[ "$src" == *.c ]]; then
        gcc "${CFLAGS[@]}" -c "$src" -o "$obj"
    else
        g++ "${CXXFLAGS[@]}" -c "$src" -o "$obj"
    fi
    objs+=("$obj")
}

sources=(
    "$ROOT/libs/rigExecRuntime/open.cpp"
    "$ROOT/libs/rigExecRuntime/exec.cpp"
    "$ROOT/libs/rigExecRuntime/closure.cpp"
    "$ROOT/libs/rigExecRuntime/publish.cpp"
    "$ROOT/libs/rigExecRuntime/kernels.cpp"
    "$ROOT/libs/rigExecRuntime/pose.cpp"
    "$ROOT/libs/rigExecRuntime/poseConstraints.cpp"
    "$ROOT/libs/rigExecRuntime/poseInterpolation.cpp"
    "$ROOT/libs/rigExecRuntime/poseMath.cpp"
    "$ROOT/libs/rigExecRuntime/poseSolvers.cpp"
    "$ROOT/libs/rigExecRuntime/poseSteps.cpp"
    "$ROOT/libs/rigExecRuntime/geometry.cpp"
    "$ROOT/libs/rigExecRuntime/weights.cpp"
    "$ROOT/libs/rigExecRuntime/inputs.cpp"
    "$ROOT/libs/rigExecRuntime/labels.cpp"
    "$ROOT/libs/rigExecRuntime/properties.cpp"
    "$ROOT/libs/rigExecRuntime/spaces.cpp"
    "$ROOT/libs/rigExecGraph/opGraph.cpp"
    "$ROOT/libs/rigExecBinary/format.cpp"
    "$ROOT/libs/rigExecBinary/transport.cpp"
    "$ROOT/third_party/lzma/C/LzmaEnc.c"
    "$ROOT/third_party/lzma/C/LzmaDec.c"
    "$ROOT/third_party/lzma/C/LzFind.c"
    "$ROOT/third_party/lzma/C/CpuArch.c"
)

for src in "${sources[@]}"; do
    compile_one "$src"
done

if nm -uC "${objs[@]}" | grep -E 'pxr::|[^A-Za-z0-9_](Gf|Usd|Sdf|Vt|Tf|Hd|Exec)[A-Za-z0-9_]*' ; then
    echo "runtime link line names an OpenUSD symbol" >&2
    exit 1
fi

echo "runtime compiled with no pxr include path (${#objs[@]} objects)"
