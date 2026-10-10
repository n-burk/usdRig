#!/bin/bash
# bin/gen_flatbuffers.sh -- regenerate the C++ headers of the .rigexec
# FlatBuffers schemas. The POSIX twin of gen_flatbuffers.bat.
#
# Writes into libs/rigExecBinary/generated, which is CHECKED IN: run it only
# after editing libs/rigExecBinary/rigexec.fbs or presentation.fbs, bump
# RigExecFormatVersion (libs/rigExecBinary/format.h) with any rigexec.fbs
# change, and review the diff.
#
# flatc is not vendored. Point FLATC at a flatc 25.12.19 binary, or put one
# on PATH; any other version is refused, because the generated headers
# static_assert the runtime headers in thirdparty/flatbuffers.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

_required="flatc version 25.12.19"

FLATC="${FLATC:-$(command -v flatc 2>/dev/null || true)}"
if [ -z "$FLATC" ] || ! command -v "$FLATC" >/dev/null 2>&1; then
    echo "ERROR: flatc not found. Set FLATC=/path/to/flatc ($_required)" >&2
    echo "       or put it on PATH, and retry." >&2
    exit 1
fi
# The run below happens from $RIG: a relative path must not depend on it.
case "$FLATC" in
    */*) FLATC="$(cd "$(dirname "$FLATC")" && pwd)/$(basename "$FLATC")" ;;
esac
_version="$("$FLATC" --version 2>/dev/null | tr -d '\r' || true)"
if [ "$_version" != "$_required" ]; then
    echo "ERROR: $FLATC reports '$_version'; $_required is required." >&2
    exit 1
fi

cd "$RIG"

# A float or double scalar in a table loses -0.0: the builder omits a value
# equal to its default. rigexec.fbs keeps those in F64/F32 structs instead.
# Comments are stripped, then every table body is split into fields.
_lint="$( { sed -e 's://.*$::' libs/rigExecBinary/rigexec.fbs |
    tr '\n\r' '  ' |
    grep -oE '(^|[^A-Za-z0-9_])table[[:space:]]+[A-Za-z_][A-Za-z0-9_]*[^{]*\{[^}]*\}' |
    sed -e 's/^[^A-Za-z0-9_]\{0,1\}table[[:space:]]*\([A-Za-z_][A-Za-z0-9_]*\)[^{]*{\(.*\)}$/\1|\2/' |
    while IFS='|' read -r _table _body; do
        printf '%s\n' "$_body" | tr ';' '\n' |
            grep -E '^[[:space:]]*[A-Za-z_][A-Za-z0-9_]*[[:space:]]*:[[:space:]]*(float|double|float32|float64)[[:space:]]*($|=|\()' |
            sed -e "s/^[[:space:]]*/$_table./"
    done; } || true)"
if [ -n "$_lint" ]; then
    echo "ERROR: float/double scalar fields in rigexec.fbs tables (use F64/F32):" >&2
    printf '%s\n' "$_lint" | sed -e 's/^/  /' >&2
    exit 1
fi

# --reflect-types emits the type tables RigExecFormatOpen bounds a buffer
# with before the verifier reads it.
"$FLATC" --cpp --cpp-std c++17 --scoped-enums --gen-object-api \
    --object-prefix RigExecWire --object-suffix "" \
    --cpp-field-case-style lower --warnings-as-errors --reflect-types \
    --include-prefix rigExecBinary/generated/ --keep-prefix \
    -o libs/rigExecBinary/generated \
    libs/rigExecBinary/rigexec.fbs libs/rigExecBinary/presentation.fbs

echo "regenerated libs/rigExecBinary/generated -- review the diff"
