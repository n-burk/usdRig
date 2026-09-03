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


def SetupPluginTest():
    """
    Make pxr and the plugin/rigExecUsdview modules importable.

    Idempotent: every step is a no-op when the ambient environment
    already carries the entry.
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
    if hasattr(os, "add_dll_directory"):
        buildRoot = (
            pathlib.Path(sys.argv[1]).parents[2] if len(sys.argv) > 1
            else repoRoot / "build")
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
