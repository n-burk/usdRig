# Shared environment for the RigExec helper scripts in this directory.
#
# Sourced, never executed:  . "$(dirname "$0")/_env.sh"
#
# Every path resolves from this file's own location, so the checkout can live
# anywhere and the helpers work from any working directory. usd-install and the
# python venv are expected as siblings of the repository (see README). Override
# any of RIG / USD / VENV in the environment to point elsewhere.

_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

RIG="${RIG:-$(cd "$_ENV_DIR/.." && pwd)}"
_SIBLINGS="$(cd "$RIG/.." && pwd)"
USD="${USD:-$_SIBLINGS/usd-install}"
VENV="${VENV:-$_SIBLINGS/usd-pr4156-venv}"
PY="$VENV/bin/python"
USDVIEW="$USD/bin/usdview"
TESTUSDVIEW="$USD/bin/testusdview"

# The python modules are under lib/python3.11/site-packages on this build, NOT
# the lib/python that the Windows tree uses.
PY_SITE="$USD/lib/python3.11/site-packages"

export PATH="$RIG/build:$USD/bin:$USD/lib:$PATH"
export DYLD_LIBRARY_PATH="$USD/lib:$RIG/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
export PYTHONPATH="$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant:$PY_SITE${PYTHONPATH:+:$PYTHONPATH}"

# The schema resources MUST be the GENERATED directory: only the generated
# plugInfo carries the LibraryPath and implementsComputeExtent that let Plug
# load the compute-extent registration. The checked-in copy under plugin/ is a
# data-only fallback and will not give RigExec prims their bounds.
export PXR_PLUGINPATH_NAME="$RIG/build/usd/rigExecSchema/resources:$RIG/build/usd/rigExecImaging/resources:$RIG/plugin/rigExecUsdview${PXR_PLUGINPATH_NAME:+:$PXR_PLUGINPATH_NAME}"

# RIGEXEC_IMAGING_DLL is deliberately NOT set here. rigExecUsdview
# .ImagingLibraryPath() resolves the platform's library name and the
# installed-vs-build layout on its own, and both testusdview scripts go
# through it. Set the variable yourself only to point at a library outside
# either layout.

# Fail early and legibly rather than deep inside python.
rigexec_require_python() {
    if [ ! -x "$PY" ]; then
        echo "ERROR: venv python not found at $PY" >&2
        echo "       set VENV=/path/to/venv and retry." >&2
        exit 1
    fi
}

rigexec_require_usd() {
    if [ ! -f "$1" ]; then
        echo "ERROR: $1 not found (USD=$USD)" >&2
        echo "       set USD=/path/to/usd-install and retry." >&2
        exit 1
    fi
}

rigexec_require_stage() {
    if [ ! -f "$1" ]; then
        echo "ERROR: stage not found: $1" >&2
        exit 1
    fi
}

# Build if the tree is already configured; a no-op when up to date. Silent
# unless it fails, because these scripts are run to see their own output.
rigexec_build() {
    if [ -f "$RIG/build/CMakeCache.txt" ]; then
        if ! cmake --build "$RIG/build" >/dev/null 2>&1; then
            echo "ERROR: build failed; run bin/build_rigexec.sh to see why." >&2
            exit 1
        fi
    fi
}
