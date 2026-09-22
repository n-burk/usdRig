"""Launcher-environment bootstrap, auto-imported by helper interpreters.

Python imports a `sitecustomize` module found on sys.path at startup,
before anything else runs. The helper scripts (bin/_env.bat, bin/_env.sh
and everything that calls them) put plugin/rigExecUsdview first on
PYTHONPATH, so this file is that hook for every helper-launched
interpreter: it points pxr at this repo's USD install, drops meta-path
finders that would claim pxr from anywhere else, and registers the
native DLL directories -- the same
`rigexec_test_env.SetupPluginTest()` the tests/python suite calls
explicitly.

The scrub is the point. A meta-path finder installed by an unrelated
package -- on one machine, NanoUSD's editable-install hook, which
claims top-level `pxr` for a pure-python shim -- outranks sys.path
order entirely, and usdview imports pxr.Usdviewq before any repo code
runs, so without this hook the shim's stub Launcher (which returns
None) breaks the launch with no repo frame on the stack to blame.

Everything is best-effort and silent: this also runs for any stray
interpreter started with the helpers' PYTHONPATH, which must never
break because of it.
"""

import os as _os
import pathlib as _pathlib
import sys as _sys


def _main():
    here = _pathlib.Path(__file__).resolve()
    repo_root = here.parents[2]
    if not (repo_root / "CMakeLists.txt").is_file():
        return
    usd_install = _os.environ.get("RIGEXEC_USD_INSTALL")
    if not usd_install:
        usd_install = str(repo_root.parent / "usd-install")
    if not _pathlib.Path(usd_install).is_dir():
        return
    _os.environ["RIGEXEC_USD_INSTALL"] = usd_install
    tests_dir = str(repo_root / "tests" / "python")
    if _pathlib.Path(tests_dir).is_dir() and tests_dir not in _sys.path:
        _sys.path.insert(0, tests_dir)
    import rigexec_test_env
    # An explicit build root: the argv derivation is for the test
    # runner's own command line, which a helper process does not have.
    rigexec_test_env.SetupPluginTest(buildRoot=repo_root / "build")


try:
    _main()
except Exception:
    pass
del _main
