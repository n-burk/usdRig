# Shared environment for the RigExec helper scripts in this directory.
#
# Sourced, never executed:  . "$(dirname "$0")/_env.sh"
#
# Every path resolves from this file's own location, so the checkout can live
# anywhere and the helpers work from any working directory. usd-install and the
# python environment are expected as siblings of the repository (see README).
# Override any of RIG / USD / VENV / PY in the environment to point elsewhere.
#
# macOS, Linux, and Windows-under-bash (Git Bash, MSYS2, WSL) all reach this
# file. Nothing below assumes a platform: the interpreter, the site-packages
# directory, and the loader variable are each DISCOVERED, because the three
# differ in all three places and a hardcoded answer is right on one machine.

_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"

RIG="${RIG:-$(cd "$_ENV_DIR/.." && pwd)}"
_SIBLINGS="$(cd "$RIG/.." && pwd)"
USD="${USD:-$_SIBLINGS/usd-install}"

# The python that has to import pxr. A venv beside the checkout is the layout
# the README describes, but it is a default and not a requirement: an explicit
# PY wins, then VENV in either of its two layouts (bin/ on POSIX, Scripts/ on
# Windows), then the interpreter on PATH -- which is the right answer when the
# USD build's own python is already the active one.
_rigexec_first_file() {
    for _candidate in "$@"; do
        if [ -n "$_candidate" ] && [ -f "$_candidate" ]; then
            printf '%s\n' "$_candidate"
            return 0
        fi
    done
    return 1
}

if [ -z "${VENV:-}" ]; then
    # Any of these, whichever exists: the checked-in default is the venv this
    # project was developed against, and nobody else has that name.
    for _venv in "$_SIBLINGS/usd-pr4156-venv" "$_SIBLINGS/usd-venv" \
                 "$_SIBLINGS/venv" "$RIG/.venv"; do
        if [ -d "$_venv" ]; then
            VENV="$_venv"
            break
        fi
    done
fi

# On POSIX, python3 is the real interpreter and python may be missing or be
# python2. Under a Windows shell it is the other way round: python3 is usually
# the Microsoft Store alias, which cannot load this USD build's extension
# modules, while python is the install that can. So the order is swapped
# rather than guessed at.
case "$(uname -s 2>/dev/null || echo unknown)" in
    MINGW*|MSYS*|CYGWIN*) _rigexec_path_python="python python3" ;;
    *)                    _rigexec_path_python="python3 python" ;;
esac

if [ -z "${PY:-}" ]; then
    _rigexec_path_first=""
    _rigexec_path_second=""
    for _name in $_rigexec_path_python; do
        _found="$(command -v "$_name" 2>/dev/null || true)"
        if [ -z "$_rigexec_path_first" ]; then
            _rigexec_path_first="$_found"
        else
            _rigexec_path_second="$_found"
        fi
    done
    # Each candidate is passed as its own quoted argument rather than as one
    # split string: a venv under a directory with a space in it is ordinary,
    # and word splitting would turn it into two paths that do not exist.
    PY="$(_rigexec_first_file         "${VENV:+$VENV/bin/python}"         "${VENV:+$VENV/bin/python3}"         "${VENV:+$VENV/Scripts/python.exe}"         "$_rigexec_path_first"         "$_rigexec_path_second" || true)"
fi

# usdview and testusdview are extensionless scripts on POSIX and .cmd wrappers
# on a Windows USD install; under bash the .cmd is what is executable.
USDVIEW="$(_rigexec_first_file "$USD/bin/usdview" "$USD/bin/usdview.cmd" \
    || echo "$USD/bin/usdview")"
TESTUSDVIEW="$(_rigexec_first_file "$USD/bin/testusdview" \
    "$USD/bin/testusdview.cmd" || echo "$USD/bin/testusdview")"

# Where THIS USD build keeps its python modules. The version is part of the
# path on POSIX (lib/python3.11, lib/python3.12, ...) and is not on Windows
# (Lib/site-packages), so it is globbed rather than written down: a helper
# pinned to one minor version stops working when USD is rebuilt against the
# next one.
PY_SITE=""
for _site in "$USD"/lib/python*/site-packages "$USD"/Lib/site-packages \
             "$USD"/lib/python; do
    if [ -d "$_site" ]; then
        PY_SITE="$_site"
        break
    fi
done

export PATH="$RIG/build:$USD/bin:$USD/lib:$PATH"

# The loader variable is not the same one on macOS and Linux, and setting only
# the other platform's is indistinguishable from setting none.
case "$(uname -s 2>/dev/null || echo unknown)" in
    Darwin)
        export DYLD_LIBRARY_PATH="$USD/lib:$RIG/build${DYLD_LIBRARY_PATH:+:$DYLD_LIBRARY_PATH}"
        ;;
    *)
        export LD_LIBRARY_PATH="$USD/lib:$RIG/build${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        ;;
esac

export PYTHONPATH="$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant${PY_SITE:+:$PY_SITE}${PYTHONPATH:+:$PYTHONPATH}"

# The schema resources MUST be the GENERATED directory: only the generated
# plugInfo carries the LibraryPath and implementsComputeExtent that let Plug
# load the compute-extent registration. The checked-in copy under plugin/ is a
# data-only fallback and will not give RigExec prims their bounds.
export PXR_PLUGINPATH_NAME="$RIG/build/usd/rigExecSchema/resources:$RIG/build/usd/rigExecImaging/resources:$RIG/plugin/rigExecUsdview${PXR_PLUGINPATH_NAME:+:$PXR_PLUGINPATH_NAME}"

# The python tests resolve pxr themselves through this, rather than inheriting
# a PYTHONPATH that may or may not carry it (see tests/python/
# rigexec_test_env.py). Exported here so a helper-run test and a ctest-run
# test read the same install.
export RIGEXEC_USD_INSTALL="$USD"

# RIGEXEC_IMAGING_DLL is deliberately NOT set here. rigExecUsdview
# .ImagingLibraryPath() resolves the platform's library name and the
# installed-vs-build layout on its own, and both testusdview scripts go
# through it. Set the variable yourself only to point at a library outside
# either layout.

# Fail early and legibly rather than deep inside python.
rigexec_require_python() {
    if [ -z "${PY:-}" ] || [ ! -f "$PY" ]; then
        echo "ERROR: no python found." >&2
        echo "       Looked for a venv beside the checkout and for python3" >&2
        echo "       on PATH. Set VENV=/path/to/venv or PY=/path/to/python" >&2
        echo "       and retry." >&2
        exit 1
    fi
    # `from pxr import Usd` and not `import pxr`: the latter is a namespace
    # package that imports on any interpreter, including one whose version
    # cannot load the build's extension modules at all. Loading a real
    # module is the only check that tells the two apart.
    if ! "$PY" -c "from pxr import Usd" >/dev/null 2>&1; then
        echo "ERROR: $PY cannot load pxr." >&2
        echo "       USD=$USD" >&2
        if [ -z "$PY_SITE" ]; then
            echo "       No python modules found under that install; set" >&2
            echo "       USD=/path/to/usd-install and retry." >&2
        else
            echo "       Found modules at $PY_SITE, so this is a version or" >&2
            echo "       ABI mismatch -- it imported pxr and then failed to" >&2
            echo "       load it. Use the python the USD build was compiled" >&2
            echo "       against: PY=/path/to/python." >&2
        fi
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
