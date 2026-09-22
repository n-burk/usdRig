"""
Shared sys.path bootstrap for the headless usdview-plugin tests.

Import and call SetupPluginTest() BEFORE the first `from pxr import ...`
in any tests/python test that exercises a module under
plugin/rigExecUsdview.

Why this exists:

ctest's ENVIRONMENT property cannot carry ';' in a value on Windows -- it
splits the property on every semicolon -- so a multi-entry PYTHONPATH
arrives mangled. CMakeLists.txt therefore gives these tests a
single-entry PYTHONPATH (the build-tree python directory) and nothing
else, which leaves no site-packages at all for pxr. tests/python/
test_rigexec_python.py:_setup_environment solves the same problem the
same way; this module is that logic, shared, so the plugin tests do not
each carry a copy.

pxr must resolve from THIS USD install and no other. An unrelated pxr is
routinely importable -- a pip usd-core in the venv, or another USD tree
on the ambient PYTHONPATH -- and its binary modules are not loadable
against this build, so the install's site-packages goes to the FRONT of
sys.path unconditionally rather than only when pxr looks missing.
"""
import os
import pathlib
import sys

# Keep os.add_dll_directory handles alive for the lifetime of the
# process: on Windows, CPython drops a DLL search directory when its
# handle is garbage-collected.
_DLL_DIR_HANDLES = []


def _Prepend(path):
    """Put `path` at the front of sys.path if it is a directory."""
    text = str(path)
    if os.path.isdir(text) and text not in sys.path:
        sys.path.insert(0, text)


def ScrubForeignPxrFinders(usdInstall):
    """Drop meta-path finders that claim `pxr` from outside `usdInstall`.

    sys.path order cannot beat a meta-path finder: finders run before
    PathFinder, so an unrelated editable install's redirect hook (on one
    machine, NanoUSD's) claims top-level `pxr` for another tree no
    matter what sys.path says. A finder whose `pxr` spec points
    outside the install is removed; one pointing inside -- or not
    claiming `pxr` at all -- is kept, as is a finder whose probe
    raises. As a backstop, a `pxr` already in sys.modules from
    outside the install (with its submodules) is evicted so the next
    import re-resolves. Call before the first `pxr` import.
    """
    home = os.path.normcase(os.path.normpath(str(usdInstall)))

    def _is_outside(spec):
        locations = getattr(spec, "submodule_search_locations", None) or []
        claimed = os.path.normcase((spec.origin or "") + "\n".join(locations))
        return home not in claimed

    for finder in list(sys.meta_path):
        find_spec = getattr(finder, "find_spec", None)
        if find_spec is None:
            continue
        try:
            spec = finder.find_spec("pxr", None)
        except Exception:  # noqa: BLE001 -- a raising finder keeps its place
            continue
        if spec is not None and _is_outside(spec):
            sys.meta_path.remove(finder)
    module = sys.modules.get("pxr")
    spec = getattr(module, "__spec__", None)
    if (module is not None and spec is not None and
            _is_outside(spec)):
        for name in [n for n in sys.modules
                     if n == "pxr" or n.startswith("pxr.")]:
            del sys.modules[name]


def SetupPluginTest(buildRoot=None):
    """
    Make pxr and the plugin/rigExecUsdview modules importable.

    Idempotent: every step is a no-op when the ambient environment
    already carries the entry.

    `buildRoot` overrides the unittest-unfriendly argv derivation of the
    build directory (the noodle suite's sitecustomize passes its staged
    build explicitly).
    """
    # This file lives at <siblings>/usdRig/tests/python/, and the USD
    # install and the venv are siblings of the checkout (see bin/_env.sh).
    testDir = pathlib.Path(__file__).resolve().parent
    repoRoot = testDir.parents[1]
    siblings = testDir.parents[2]
    usdInstall = pathlib.Path(
        os.environ.get("RIGEXEC_USD_INSTALL") or (siblings / "usd-install"))

    # The layout differs by platform: Windows installs to
    # <usd>/Lib/site-packages, POSIX under <usd>/lib/python*/site-packages.
    siteCandidates = [usdInstall / "Lib" / "site-packages"]
    siteCandidates.extend(
        sorted(usdInstall.glob("lib/python*/site-packages")))
    for sitePackages in siteCandidates:
        _Prepend(sitePackages)

    pluginPath = usdInstall / "lib" / "usd"
    if pluginPath.is_dir() and not os.environ.get("PXR_PLUGINPATH_NAME"):
        os.environ["PXR_PLUGINPATH_NAME"] = str(pluginPath)

    # Python 3.8+ on Windows does not search PATH for extension-module
    # dependencies; register the native directories explicitly and retain
    # the handles (releasing one drops the directory from the search path).
    if buildRoot is None:
        if len(sys.argv) > 1:
            buildRoot = pathlib.Path(sys.argv[1]).parents[2]
        else:
            buildRoot = repoRoot / "build"
    else:
        buildRoot = pathlib.Path(buildRoot)
    if hasattr(os, "add_dll_directory"):
        dllDirs = [usdInstall / "lib", usdInstall / "bin", buildRoot]
        # Multi-config generators (Visual Studio) place the native libs
        # in a per-configuration subdirectory of the build root.
        if buildRoot.is_dir():
            for sub in sorted(buildRoot.iterdir()):
                if sub.is_dir() and (sub / "rigExec.dll").is_file():
                    dllDirs.append(sub)
        for d in dllDirs:
            try:
                if pathlib.Path(d).is_dir():
                    _DLL_DIR_HANDLES.append(os.add_dll_directory(str(d)))
            except OSError:
                pass  # already registered or unsupported; harmless

    # The modules under test are plain files, not an installed package.
    _Prepend(repoRoot / "plugin" / "rigExecUsdview")

    # Meta-path finders outrank the sys.path order above, so scrub the
    # ones that would claim pxr from anywhere but this install.
    ScrubForeignPxrFinders(usdInstall)
