"""Headless verification that Houdini loads the RigExec plugins.

Runs under hython via ``launch_houdini_rigexec.bat check`` (the wrapper sets
PATH, HOUDINI_USD_DSO_PATH, and PYTHONPATH first). Optional argv[1] is the
RigExec install prefix; otherwise it is resolved from this script's location
(``bin/`` next to ``build-houdini/install``).

Checks, in order:
  0. The interpreter is Houdini's Python 3.13 with no hostile
     PATH/PYTHONPATH/PXR_PLUGINPATH_NAME entry (a stock cp310 pxr package,
     python310.dll, a stock-build DLL dir, or anything MoonRay)
     surviving the wrapper scrub.
  1. The install's USD plugin directory is on PXR_PLUGINPATH_NAME.
  2. The ``rigExecSchema`` and ``rigExecImaging`` plugins are registered and
     load (loading pulls in rigExecImaging.dll and its USD linkage).
  3. The ``RigExecRoot`` schema type is known.
  4. The ``rigexec`` Python bindings import.
  5. A Builder-authored rig compiles and evaluates end to end.
  6. The ``usdMayaRig`` file-format plugin opens a ``.ma`` fixture.

With RIGEXEC_CHECK_MINIMAL=1 (the combined launcher, which carries no
Python bindings), checks 4-5 are replaced by an assertion that ``rigexec``
is neither on PYTHONPATH nor importable.

Exits 0 with an OK summary, or 1 with the first failure.
"""

import glob
import os
import sys


def _fail(message):
    print("CHECK FAILED: %s" % message)
    return 1


def main(argv):
    here = os.path.dirname(os.path.abspath(__file__))
    if len(argv) > 1:
        install = os.path.abspath(argv[1])
    else:
        install = os.path.normpath(
            os.path.join(here, "..", "build-houdini", "install"))
    usd_dir = os.path.join(install, "lib", "usd")
    if not os.path.isfile(os.path.join(
            usd_dir, "rigExecSchema", "resources", "plugInfo.json")):
        return _fail("no rigExecSchema plugInfo under %s" % usd_dir)
    if os.environ.get("MAYAINSTALL"):
        maya_install = os.path.abspath(os.environ["MAYAINSTALL"])
    else:
        maya_install = os.path.normpath(os.path.join(
            here, "..", "..", "usdMayaRig", "build-houdini", "install"))
    maya_usd = os.path.join(maya_install, "lib", "usd")
    if not os.path.isfile(os.path.join(
            maya_usd, "usdMayaRig", "resources", "plugInfo.json")):
        return _fail("no usdMayaRig plugInfo under %s" % maya_usd)

    # 0. The interpreter is Houdini's own 3.13 and no hostile entry
    # survived the wrapper's scrub: a stock (cp310) pxr on PYTHONPATH
    # fails with "DLL load failed while importing _tf", so fail fast
    # here if any such segment is still present. The install's own
    # directories (prepended after the scrub) and Houdini's bin on PATH
    # (ships python313.dll) are exempt.
    minimal = os.environ.get("RIGEXEC_CHECK_MINIMAL") == "1"
    install_norm = os.path.normcase(os.path.abspath(install))
    hfs_raw = (os.environ.get("HOUDINI_ROOT") or os.environ.get("HFS") or "")
    if not hfs_raw and "hython" in os.path.basename(sys.executable).lower():
        hfs_raw = os.path.dirname(
            os.path.dirname(os.path.abspath(sys.executable)))
    hfs = os.path.normcase(os.path.abspath(hfs_raw)) if hfs_raw else ""
    # The combined launcher prepends a second install's lib dir too; its
    # prefix arrives via the environment (batch `set` exports to children).
    extra_roots = [os.path.normcase(os.path.abspath(os.environ[v]))
                   for v in ("INSTALL", "RIGINSTALL", "GENINSTALL",
                             "MAYAINSTALL")
                   if os.environ.get(v)]

    def _under(path, root):
        return bool(root) and (path == root or
                               path.startswith(root + os.sep))

    if sys.version_info[:2] != (3, 13):
        return _fail("expected Houdini Python 3.13, got %s"
                      % sys.version.split()[0])
    # hython sets PYTHONHOME to its own python313 at startup; only a
    # foreign value (e.g. a CPython 3.10 home) is hostile.
    home = os.environ.get("PYTHONHOME")
    if home and not _under(os.path.normcase(os.path.abspath(home)), hfs):
        return _fail("PYTHONHOME points outside Houdini (%r); the wrapper "
                      "must clear it" % home)

    for var in ("PATH", "PYTHONPATH", "PXR_PLUGINPATH_NAME"):
        for entry in os.environ.get(var, "").split(";"):
            entry = entry.strip().strip('"')
            if not entry:
                continue
            seg = os.path.normcase(os.path.abspath(entry))
            if _under(seg, install_norm):
                continue
            if any(_under(seg, root) for root in extra_roots):
                continue
            if var == "PATH" and _under(seg, hfs):
                continue
            if glob.glob(os.path.join(entry, "python3*.dll")):
                return _fail("%s still carries %s (ships python3*.dll); "
                             "the wrapper scrub missed it" % (var, entry))
            if os.path.isfile(os.path.join(entry, "pxr", "__init__.py")):
                return _fail("%s still carries %s (ships a pxr package); "
                             "the wrapper scrub missed it" % (var, entry))
            for leaf in ("usd_usd.dll", "rigExec.dll", "usdGen*.dll",
                         "usdMayaRig.dll", "hdMoonray*.dll"):
                if glob.glob(os.path.join(entry, leaf)):
                    return _fail("%s still carries %s (ships %s); "
                                 "the wrapper scrub missed it"
                                 % (var, entry, leaf))
            if "moonray" in seg:
                return _fail("%s still carries %s (MoonRay); "
                             "the wrapper scrub missed it" % (var, entry))
    if minimal:
        for entry in os.environ.get("PYTHONPATH", "").split(";"):
            entry = entry.strip().strip('"')
            if (entry and os.path.isdir(os.path.join(entry, "rigexec"))):
                return _fail("minimal composition must not carry the "
                             "Python bindings (PYTHONPATH has %s)" % entry)
        print("OK: minimal composition carries no rigexec on PYTHONPATH")
    print("OK: interpreter is 3.13 with no hostile environment entry")

    # 1. The wrapper's plugin path survived into this process. Houdini
    # discovers USD plugins through HOUDINI_USD_DSO_PATH, not
    # PXR_PLUGINPATH_NAME, so that is what is checked.
    plugin_path = os.environ.get("HOUDINI_USD_DSO_PATH", "")
    entries = [os.path.normcase(os.path.abspath(e)) for e in
               plugin_path.split(";") if e and e != "&"]
    want = [os.path.normcase(os.path.abspath(os.path.join(
                usd_dir, name, "resources")))
            for name in ("rigExecSchema", "rigExecImaging")]
    want.append(os.path.normcase(os.path.abspath(os.path.join(
        maya_usd, "usdMayaRig", "resources"))))
    if not all(w in entries for w in want):
        return _fail("HOUDINI_USD_DSO_PATH lacks %s (got %r)"
                     % (usd_dir, plugin_path))
    print("OK: HOUDINI_USD_DSO_PATH carries the install")

    # 2. All three USD plugins register and load.
    from pxr import Plug, Sdf, Tf, Usd, UsdGeom
    if hfs and not _under(os.path.normcase(os.path.abspath(Tf.__file__)),
                          hfs):
        return _fail("pxr resolves to %s, outside HOUDINI_ROOT" % Tf.__file__)
    print("OK: pxr resolves inside the Houdini install")
    registry = Plug.Registry()
    for name in ("rigExecSchema", "rigExecImaging", "usdMayaRig"):
        plugin = registry.GetPluginWithName(name)
        if not plugin:
            return _fail("plugin %r is not registered" % name)
        if not plugin.Load():
            return _fail("plugin %r is registered but would not load" % name)
        print("OK: plugin %s loads from %s" % (name, plugin.path))

    # 3. The schema types resolve.
    root_type = Tf.Type.FindByName("RigExecRoot")
    if not root_type:
        return _fail("TfType RigExecRoot is unknown after loading plugins")
    print("OK: TfType RigExecRoot resolves")

    if minimal:
        # 4-5. The minimal composition carries no Python bindings:
        # rigexec must neither be on PYTHONPATH (asserted in check 0)
        # nor importable.
        try:
            import rigexec
        except ImportError:
            print("OK: rigexec is not importable (minimal composition)")
        else:
            return _fail("rigexec imports from %s in the minimal "
                         "composition" % rigexec.__file__)
    else:
        # 4. The Python bindings import.
        import rigexec
        print("OK: rigexec imports from %s"
              % os.path.dirname(os.path.abspath(rigexec.__file__)))

        # 5. End to end: author a one-mover rig, compile, evaluate.
        stage = Usd.Stage.CreateInMemory()
        dots = UsdGeom.Points.Define(stage, "/Model/Geom/Dots")
        stage.GetPrimAtPath(dots.GetPath()).GetAttribute("points").Set(
            [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (4.0, 0.0, 0.0)])
        builder = rigexec.Builder.create(stage, "/Rig")
        ctrl = builder.add_control("Ctrl")
        if not ctrl.valid:
            return _fail("Builder.add_control produced an invalid handle")
        w = builder.add_static_weight(
            "W", "/Model/Geom/Dots.points", [1.0, 0.5, 0.0])
        chain = builder.new_mover_chain("chain")
        mover = chain.add_matrix_mover(
            "move", ctrl.path, w.path, "/Model/Geom/Dots.points")
        if not mover.valid:
            return _fail("add_matrix_mover produced an invalid handle")
        tx = stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:tx")
        tx.Set(0.0, Usd.TimeCode(0))
        tx.Set(10.0, Usd.TimeCode(100))
        rig = rigexec.Rig(stage, "/Rig")
        rig.compile()
        pose = rig.evaluate(100)
        pts = [tuple(p) for p in
               pose.moved_property("/Model/Geom/Dots.points")]
        expected = [10.0, 7.0, 4.0]
        got = [p[0] for p in pts]
        if len(got) != 3 or any(abs(g - e) > 1e-6
                                for g, e in zip(got, expected)):
            return _fail("evaluate(100) x-channels %r, expected %r"
                         % (got, expected))
        print("OK: Builder rig compiles and evaluates (x = %s)" % got)

    # 6. The Maya file-format plugin opens a .ma fixture.
    maya_format = Sdf.FileFormat.FindById("mayaRig")
    if not maya_format:
        return _fail("SdfFileFormat mayaRig is unknown after loading plugins")
    fixture = os.path.normpath(os.path.join(
        maya_install, "..", "..", "tests", "fixtures", "rig.ma"))
    if not os.path.isfile(fixture):
        return _fail("sidecar fixture missing: %s" % fixture)
    ma_stage = Usd.Stage.Open(fixture)
    if not ma_stage:
        return _fail("Usd.Stage.Open failed on %s" % fixture)
    root_format = ma_stage.GetRootLayer().GetFileFormat().formatId
    if root_format != "mayaRig":
        return _fail("fixture root layer format is %r, not mayaRig"
                      % root_format)
    default = ma_stage.GetDefaultPrim()
    if not default or not default.HasAttribute("maya:diagnostics"):
        return _fail("translated rig lacks the maya:diagnostics attribute")
    diags = default.GetAttribute("maya:diagnostics").Get() or []
    print("OK: .ma fixture opens via mayaRig (%s, %d diagnostics)"
          % (default.GetPath(), len(diags)))

    print("ALL RIGEXEC HOUDINI CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
