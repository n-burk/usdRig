"""`import pxr` in this suite must resolve to the repo's USD install.

sys.path order is not enough to guarantee that: a meta-path finder --
an unrelated editable install's redirect hook, such as the NanoUSD one
that once hijacked this machine's interpreter -- runs before PathFinder
and can claim top-level `pxr` for another tree (there, a pure-python
shim without the API the suite needs). The shared bootstraps scrub
such finders before the first pxr import; this test pins that:

  1. in the live interpreter, no finder may claim `pxr` from outside
     the install once the bootstrap ran, and the import itself must
     land inside the install;
  2. synthetically, a hostile claim is dropped while a friendly one
     from inside the install is kept;
  3. the helper launch path: the helper scripts put
     plugin/rigExecUsdview first on PYTHONPATH, so a sitecustomize
     there is the startup scrub for every helper-launched
     interpreter. usdview needs it -- it imports pxr.Usdviewq before
     any repo code runs, so nothing later can rescue it, and a
     NanoUSD-style hook claiming top-level pxr turns that import
     into a stub whose Launcher() returns None. The hook file must
     exist and must drop a hostile claim when executed (3a), and a
     child interpreter started with the launcher environment must
     resolve pxr -- and pxr.Usdviewq where the install ships it --
     from inside the install (3b).

Usage: test_pxr_import_hygiene.py [<generated schema resources dir>]
"""
import os
import pathlib
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import rigexec_test_env  # noqa: E402


def _usd_install():
    here = pathlib.Path(__file__).resolve().parent
    text = os.environ.get("RIGEXEC_USD_INSTALL") or str(
        here.parents[2] / "usd-install")
    return os.path.normcase(os.path.normpath(text))


def _claim_locations(spec):
    locations = getattr(spec, "submodule_search_locations", None) or []
    return os.path.normcase((spec.origin or "") + "\n".join(locations))


class _FakeFinder:
    """A minimal meta-path finder claiming top-level pxr from `origin`."""

    def __init__(self, origin):
        self._origin = origin

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "pxr":
            return None
        import importlib.machinery
        return importlib.machinery.ModuleSpec(
            "pxr", loader=None, origin=self._origin)


def _launcher_site_packages(usd_install):
    """USD site-packages dirs carrying pxr, in the bootstrap's layouts."""
    candidates = [pathlib.Path(usd_install) / "Lib" / "site-packages"]
    candidates.extend(sorted(
        pathlib.Path(usd_install).glob("lib/python*/site-packages")))
    return [c for c in candidates if (c / "pxr").is_dir()]


# The child in 3b must NOT import the test bootstrap: it proves the
# launcher environment alone -- PYTHONPATH plus the startup hook --
# resolves pxr to the install.
_LAUNCHER_PROBE = "; ".join([
    "import importlib.util",
    "import pxr",
    "print('PXR: ' + str((list(getattr(pxr, '__path__', [])) or [None])[0]))",
    "_spec = importlib.util.find_spec('pxr.Usdviewq')",
    "print('VIEW: ' + str(_spec.origin if _spec is not None else None))",
    "print('RIGEXEC_LAUNCHER_PROBE_OK')",
])


def _run_launcher_probe(usd_install, repo_root):
    site_dirs = _launcher_site_packages(usd_install)
    assert site_dirs, "no pxr-bearing site-packages under %s" % usd_install
    entries = [str(repo_root / "plugin" / "rigExecUsdview")]
    entries.extend(str(d) for d in site_dirs)
    env = dict(os.environ)
    # Fresh, not inherited: the test's own single-entry PYTHONPATH must
    # not leak into the simulated launcher environment.
    env["PYTHONPATH"] = os.pathsep.join(entries)
    env["RIGEXEC_USD_INSTALL"] = usd_install
    completed = subprocess.run(
        [sys.executable, "-c", _LAUNCHER_PROBE],
        env=env, capture_output=True, text=True, timeout=180)
    assert completed.returncode == 0, (
        "launcher-env probe exited %s:\n%s"
        % (completed.returncode, completed.stderr[-2000:]))
    assert "RIGEXEC_LAUNCHER_PROBE_OK" in completed.stdout.splitlines(), (
        "launcher-env probe did not finish:\n%s" % completed.stdout[-2000:])
    lines = {}
    for line in completed.stdout.splitlines():
        if ": " in line:
            key, _, value = line.partition(": ")
            lines[key] = value
    home = usd_install + os.sep
    pxr_path = os.path.normcase(os.path.normpath(lines.get("PXR", "")))
    assert pxr_path.startswith(home), (
        "launcher env resolved pxr to %s, not the install at %s"
        % (lines.get("PXR"), usd_install))
    if any((d / "pxr" / "Usdviewq").is_dir() for d in site_dirs):
        view_origin = os.path.normcase(
            os.path.normpath(lines.get("VIEW", "")))
        assert view_origin.startswith(home), (
            "launcher env resolved pxr.Usdviewq to %s, not the install "
            "at %s" % (lines.get("VIEW"), usd_install))


def main():
    usd_install = _usd_install()

    # 1. The live interpreter: scrub, then nobody claims outside.
    rigexec_test_env.SetupPluginTest()
    for finder in sys.meta_path:
        find_spec = getattr(finder, "find_spec", None)
        if find_spec is None:
            continue
        try:
            spec = finder.find_spec("pxr", None)
        except Exception:  # noqa: BLE001 -- a raising finder keeps its place
            continue
        if spec is None:
            continue
        assert usd_install in _claim_locations(spec), (
            "%r claims pxr from outside %s" % (finder, usd_install))
    import pxr  # noqa: E402
    paths = list(getattr(pxr, "__path__", []))
    assert paths, "pxr imported with an empty __path__"
    first = os.path.normcase(os.path.normpath(paths[0]))
    assert first.startswith(usd_install + os.sep), (
        "pxr resolved to %s, not the install at %s" % (paths[0], usd_install))

    # 2. Synthetic: hostile dropped, friendly kept.
    hostile = _FakeFinder(os.path.join("C:", "elsewhere", "pxr", "__init__.py"))
    friendly = _FakeFinder(os.path.join(
        usd_install, "Lib", "site-packages", "pxr", "__init__.py"))
    sys.meta_path.insert(0, hostile)
    sys.meta_path.insert(0, friendly)
    rigexec_test_env.ScrubForeignPxrFinders(usd_install)
    assert hostile not in sys.meta_path, "a hostile pxr claim survived"
    assert friendly in sys.meta_path, "an install-local pxr claim was dropped"
    sys.meta_path.remove(friendly)

    repo_root = pathlib.Path(__file__).resolve().parents[2]

    # 3a. The launcher hook file itself, executed against a hostile
    # claim. Exec, not import: the hook is silent by design (it must
    # never break a stray interpreter), so only the claim's absence
    # afterwards proves it ran the scrub.
    hook = repo_root / "plugin" / "rigExecUsdview" / "sitecustomize.py"
    assert hook.is_file(), "launcher sitecustomize missing: %s" % hook
    import importlib.util
    hook_spec = importlib.util.spec_from_file_location(
        "rigexec_launcher_hook_probe", str(hook))
    hook_module = importlib.util.module_from_spec(hook_spec)
    hostile_hook = _FakeFinder(
        os.path.join("C:", "elsewhere", "pxr", "__init__.py"))
    sys.meta_path.insert(0, hostile_hook)
    hook_spec.loader.exec_module(hook_module)
    assert hostile_hook not in sys.meta_path, (
        "the launcher hook left a hostile pxr claim in place")

    # 3b. End to end: a fresh interpreter with the launcher
    # environment resolves pxr to the install.
    _run_launcher_probe(usd_install, repo_root)

    print("RIGEXEC_PXR_HYGIENE_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
