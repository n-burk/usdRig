"""Test-environment bootstrap, auto-imported by every test interpreter.

Python imports a `sitecustomize` module found on sys.path at startup,
before anything else runs. The noodle suite starts its main interpreter
(and its `sys.executable -c` child probes) with this directory as the
working directory, so this file is that hook: it points pxr at this
repo's USD install, drops meta-path finders that would claim pxr from
anywhere else, and registers the native DLL directories -- the same
`rigexec_test_env.SetupPluginTest()` the tests/python suite calls
explicitly, which cannot run here because unittest discovery imports
test modules directly with no shared entry point.

Everything is best-effort and silent: this also runs for any stray
interpreter started in this directory, which must never break because
of it.
"""

import os as _os
import pathlib as _pathlib
import sys as _sys


def _main():
    here = _pathlib.Path(__file__).resolve()
    anchor = here.parents[3]
    if (anchor / "CMakeLists.txt").is_file():
        # Source tree (repo/plugin/usdNoodles/testenv): the build is the
        # conventional sibling directory the helper scripts use.
        repo_root = anchor
        build_root = repo_root / "build"
    else:
        # Staged tree (build/python/UsdNoodles/testenv).
        build_root = anchor
        repo_root = build_root.parent
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
    rigexec_test_env.SetupPluginTest(buildRoot=build_root)


try:
    _main()
except Exception:
    pass
del _main
