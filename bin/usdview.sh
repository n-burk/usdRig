#!/bin/bash
# bin/usdview.sh -- interactive usdview on a RigExec stage with the live Hydra
# integration. The POSIX twin of launch_usdview.bat.
#
# Usage: bin/usdview.sh [stage.usda] [rendererDisplayName | usdview flags...]
#   stage defaults to an empty stage carrying a single World Xform, so that
#     opening the app to build something is the no-argument case; pass
#     examples/ArmShotAnim.usda for the rig that deforms on the timeline.
#   a bare second argument is the renderer (e.g. Embree); anything starting
#   with - is passed to usdview untouched.
#
# For the Muse assistant and its readiness banner use bin/launch.sh instead.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

# Register the Muse assistant's plugin container. _env.sh puts museAssistant on
# PYTHONPATH for every helper, but registering it is deliberately left to the
# interactive launchers: the headless testusdview runners share that env and
# must not load an extra panel into the app they are asserting against.
#
# Hosted providers need MUSE_API_KEY (or ANTHROPIC_API_KEY). Local Apple FM and
# Ollama do not. bin/launch.sh reports provider-specific readiness before
# launching; this lower-level helper stays quiet.
export PXR_PLUGINPATH_NAME="$PXR_PLUGINPATH_NAME:$RIG/plugin/museAssistant"

rigexec_require_python
rigexec_require_usd "$USDVIEW"
rigexec_build
rigexec_register_usdnoodles

# Rewritten every run rather than kept, so an edited or truncated leftover
# cannot turn into a confusing "blank" stage on the next launch.
BLANK="${TMPDIR:-/tmp}/rigexec-blank.usda"
_write_blank() {
    cat > "$BLANK" <<'USD'
#usda 1.0
def Xform "World" {
}
USD
}

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    STAGE="$1"; shift
    rigexec_require_stage "$STAGE"
else
    _write_blank
    STAGE="$BLANK"
fi

# A bare (non-flag) argument is the renderer display name, matching the .bat
# helpers. Prepending to the positional parameters rather than collecting an
# array keeps this working under bash 3.2, where expanding an empty array with
# set -u is an "unbound variable" error.
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$USDVIEW" "$@" "$STAGE"
