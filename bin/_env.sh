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

# macOS only: fall back to an older SDK when the default one cannot link.
#
# The selected toolchain and the default SDK drift independently: a Command
# Line Tools update can install a MacOSX27 SDK whose .tbd files name an
# architecture the Xcode-selected linker's TAPI cannot parse, and every
# link then fails with "unknown architecture" / tapi errors. When that is
# the state of this machine, point SDKROOT at the newest SDK the toolchain
# CAN link against, so the helpers build instead of failing. An explicit
# SDKROOT is always respected and never overridden.
#
# This costs one trivial link on a healthy machine and does nothing at all
# when the default links: the fallback search runs only after the default
# has already failed, and when no installed SDK links the environment is
# left alone so the real build reports the real error.
if [ -z "${SDKROOT:-}" ]; then
    case "$(uname -s 2>/dev/null || echo unknown)" in
        Darwin)
            _rigexec_sdk_cxx="${CXX:-c++}"
            if command -v "$_rigexec_sdk_cxx" >/dev/null 2>&1; then
                _rigexec_sdk_probe="$(mktemp -d "${TMPDIR:-/tmp}/rigexec-sdkprobe.XXXXXX" 2>/dev/null || echo "")"
                if [ -n "$_rigexec_sdk_probe" ] && [ -d "$_rigexec_sdk_probe" ]; then
                    printf 'int main(){return 0;}\n' > "$_rigexec_sdk_probe/t.cpp"
                    if ! "$_rigexec_sdk_cxx" -o "$_rigexec_sdk_probe/t" "$_rigexec_sdk_probe/t.cpp" >/dev/null 2>&1; then
                        : > "$_rigexec_sdk_probe/cands"
                        _rigexec_sdk_seen=""
                        for _rigexec_sdk_root in /Library/Developer/CommandLineTools/SDKs \
                                "/Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs"; do
                            [ -d "$_rigexec_sdk_root" ] || continue
                            for _rigexec_sdk in "$_rigexec_sdk_root"/MacOSX*.sdk; do
                                [ -d "$_rigexec_sdk" ] || continue
                                _rigexec_sdk_resolved="$(cd "$_rigexec_sdk" 2>/dev/null && pwd -P 2>/dev/null || echo "")"
                                [ -n "$_rigexec_sdk_resolved" ] || continue
                                case " $_rigexec_sdk_seen " in
                                    *" $_rigexec_sdk_resolved "*) continue ;;
                                esac
                                _rigexec_sdk_seen="${_rigexec_sdk_seen:+$_rigexec_sdk_seen }$_rigexec_sdk_resolved"
                                _rigexec_sdk_ver="$(basename "$_rigexec_sdk_resolved" .sdk)"
                                _rigexec_sdk_ver="${_rigexec_sdk_ver#MacOSX}"
                                _rigexec_sdk_major="${_rigexec_sdk_ver%%.*}"
                                _rigexec_sdk_minor="0"
                                case "$_rigexec_sdk_ver" in
                                    *.*) _rigexec_sdk_minor="${_rigexec_sdk_ver#*.}"
                                         _rigexec_sdk_minor="${_rigexec_sdk_minor%%.*}" ;;
                                esac
                                case "$_rigexec_sdk_major" in ''|*[!0-9]*) continue ;; esac
                                case "$_rigexec_sdk_minor" in ''|*[!0-9]*) _rigexec_sdk_minor=0 ;; esac
                                printf '%05d%05d %s\n' "$_rigexec_sdk_major" "$_rigexec_sdk_minor" \
                                    "$_rigexec_sdk_resolved" >> "$_rigexec_sdk_probe/cands"
                            done
                        done
                        _rigexec_sdk_pick=""
                        while read -r _rigexec_sdk_key _rigexec_sdk_try; do
                            [ -n "${_rigexec_sdk_try:-}" ] || continue
                            if SDKROOT="$_rigexec_sdk_try" "$_rigexec_sdk_cxx" -o "$_rigexec_sdk_probe/t" \
                                    "$_rigexec_sdk_probe/t.cpp" >/dev/null 2>&1; then
                                _rigexec_sdk_pick="$_rigexec_sdk_try"
                                break
                            fi
                        done <<_RIGEXEC_SDK_EOF
$(sort -rn "$_rigexec_sdk_probe/cands" 2>/dev/null)
_RIGEXEC_SDK_EOF
                        if [ -n "$_rigexec_sdk_pick" ]; then
                            export SDKROOT="$_rigexec_sdk_pick"
                            echo "NOTE: the default macOS SDK cannot link with this toolchain;" >&2
                            echo "      using $SDKROOT instead (SDKROOT). To choose another," >&2
                            echo "      export SDKROOT yourself; to fix the toolchain, update" >&2
                            echo "      Xcode or run: sudo xcode-select -s /Library/Developer/CommandLineTools" >&2
                        fi
                    fi
                    rm -rf "$_rigexec_sdk_probe"
                fi
            fi
            ;;
    esac
fi

# build/python is where the build stages the UsdNoodles package beside its
# native module; see rigexec_register_usdnoodles below.
export PYTHONPATH="$RIG/plugin/rigExecUsdview:$RIG/plugin/museAssistant:$RIG/build/python${PY_SITE:+:$PY_SITE}${PYTHONPATH:+:$PYTHONPATH}"

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

# Register the usdNoodles node-graph editor the build staged under
# build/python/UsdNoodles. For the interactive launchers only, like the Muse
# container: the headless runners must not load an extra panel into the app
# they are asserting against. Call it after building, so a first build has
# already produced the plugInfo.json it looks for.
#
# Skipped when this USD install ships its own pxr.UsdNoodles -- an OpenUSD
# built from PR #4156 with noodles, which is where this copy came from. The
# two register the same usdview command names, and usdview answers a
# duplicate name by loading no plugins at all, RigExec's own included.
rigexec_register_usdnoodles() {
    if [ -n "$PY_SITE" ] && [ -d "$PY_SITE/pxr/UsdNoodles" ]; then
        echo "usdNoodles: $PY_SITE/pxr/UsdNoodles is loaded instead of the" >&2
        echo "            in-repo copy; registering both stops usdview loading" >&2
        echo "            any plugin. Build OpenUSD without noodles to use" >&2
        echo "            plugin/usdNoodles." >&2
        return 0
    fi
    if [ -f "$RIG/build/python/UsdNoodles/plugInfo.json" ]; then
        export PXR_PLUGINPATH_NAME="$PXR_PLUGINPATH_NAME:$RIG/build/python/UsdNoodles"
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
