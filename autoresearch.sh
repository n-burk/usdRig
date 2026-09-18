#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

STAGE="examples/biped/Biped_all.usda"
FRAMES="1001,1024,1048"
REPEAT=5
BIN="./build/rigExecPose"
REF="autoresearch/fixtures"
mkdir -p "$REF"

# --- Incremental build ---
cmake --build build --target rigExecPose -j"$(nproc)" > /tmp/arp_build.log 2>&1 \
    || { echo "BUILD FAILED" >&2; tail -30 /tmp/arp_build.log >&2; exit 1; }

run_and_parse() {
    local mode="$1"
    local log="/tmp/arp_${mode}.log"
    local pose="/tmp/arp_${mode}_pose.txt"

    "$BIN" "$STAGE" \
        --frames "$FRAMES" \
        --repeat "$REPEAT" \
        --mode "$mode" \
        --pose-out "$pose" \
        --profile "/tmp/arp_${mode}.trace" \
        > "$log" 2>&1

    local compile_ms eval_us
    compile_ms=$(grep '\[compile\] Compile$' "$log" | head -1 | awk '{print $1}' || true)
    eval_us=$(grep 'repeat:' "$log" | head -1 | sed 's/.*(\([0-9.]*\)us\/frame.*/\1/' || true)

    if [[ -z "$compile_ms" || -z "$eval_us" ]]; then
        echo "HARNESS ERROR: failed to parse ${mode} output" >&2
        tail -30 "$log" >&2
        exit 1
    fi

    # Correctness gate: pose output must be byte-identical to reference
    local sha ref_file
    sha=$(sha256sum "$pose" | awk '{print $1}')
    ref_file="$REF/${mode}.sha256"
    if [[ -f "$ref_file" ]]; then
        local expected
        expected=$(cat "$ref_file")
        if [[ "$sha" != "$expected" ]]; then
            echo "HARNESS FAIL: ${mode} pose output differs from reference" >&2
            echo "  expected: $expected" >&2
            echo "  got:      $sha" >&2
            exit 1
        fi
    else
        echo "$sha" > "$ref_file"
        echo "INFO: created reference ${ref_file}" >&2
    fi

    COMPILE_MS="$compile_ms"
    EVAL_US="$eval_us"
}

run_and_parse dynamic
dyn_compile="$COMPILE_MS"
dyn_eval="$EVAL_US"

run_and_parse baked
bk_compile="$COMPILE_MS"
bk_eval="$EVAL_US"

echo "METRIC compile_ms=${dyn_compile}"
echo "METRIC eval_us=${dyn_eval}"
echo "METRIC baked_compile_ms=${bk_compile}"
echo "METRIC baked_eval_us=${bk_eval}"
