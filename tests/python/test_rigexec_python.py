"""End-to-end Python binding test (spec section 4 authoring surface).

Authors a small rig entirely through the rigexec package -- a control, a
static weight object, and one matrix-mover chain over a Points prim -- then
compiles and evaluates it with the native Rig wrapper and asserts the moved
points match p' = q + w (T q - q) by hand.

Usage:
    python test_rigexec_python.py [schema_resources_dir]

The schema resources directory may also come from RIGEXEC_SCHEMA_RESOURCE_DIR;
without either, rigexec.load_schema_plugin() probes build- and source-tree
locations relative to the installed package.
"""

import os
import pathlib
import sys

# Keep os.add_dll_directory handles alive for the lifetime of the process:
# on Windows, CPython drops a DLL search directory when its handle is
# garbage-collected.
_DLL_DIR_HANDLES = []


def _close(a, b, eps=1e-5):
    return abs(a - b) < eps


def _setup_environment():
    """Make this test runnable from any environment (ctest, bare cmd, or the
    probe/run_e2e.bat runner).

    CTest's ENVIRONMENT property cannot carry ';' in values on Windows -- it
    splits assignments on every semicolon -- so a multi-entry PYTHONPATH
    arrives mangled and pxr would resolve from a global site-packages copy
    whose _tf.pyd cannot load against this USD tree. Fix sys.path, the DLL
    search directories, and PXR_PLUGINPATH_NAME here instead. Every step is
    idempotent when the ambient environment is already correct.
    """
    # This file lives at <repo>/usdRig/tests/python/.
    repo_root = pathlib.Path(__file__).resolve().parents[3]
    usd_install = pathlib.Path(
        os.environ.get("RIGEXEC_USD_INSTALL") or (repo_root / "usd-install"))

    # pxr must resolve from the USD install, never a global copy. The layout
    # differs by platform: Windows installs to <usd>/Lib/site-packages, while
    # POSIX installs under <usd>/lib/python*/site-packages.
    # Probe the Windows layout by its full site-packages path, never by the
    # bare Lib directory: some POSIX installs ship an unrelated Lib/ (here
    # OpenSubdiv artifacts), which must not shadow the lib/python*/ layout.
    site_candidates = []
    win_site = usd_install / "Lib" / "site-packages"
    if win_site.is_dir():
        site_candidates.append(win_site)
    site_candidates.extend(sorted(usd_install.glob("lib/python*/site-packages")))
    for site_packages in site_candidates:
        if site_packages.is_dir():
            sp = str(site_packages)
            if sp not in sys.path:
                sys.path.insert(0, sp)

    pluginpath = usd_install / "lib" / "usd"
    if pluginpath.is_dir() and not os.environ.get("PXR_PLUGINPATH_NAME"):
        os.environ["PXR_PLUGINPATH_NAME"] = str(pluginpath)

    # The build-tree python directory holds _rigexec plus the copied rigexec
    # package. Single-config generators put them in <build>/python directly;
    # multi-config ones (Visual Studio) under a per-configuration subdirectory.
    # ctest's PYTHONPATH only covers the single-config layout, so probe both.
    build_root = (
        pathlib.Path(sys.argv[1]).parents[2] if len(sys.argv) > 1
        else repo_root / "build")
    py_dir = build_root / "python"
    candidates = []
    if py_dir.is_dir():
        candidates.append(py_dir)
        for sub in sorted(py_dir.iterdir()):
            if sub.is_dir() and any(sub.glob("_rigexec*")):
                candidates.append(sub)
    for d in reversed(candidates):
        p = str(d)
        if p not in sys.path:
            sys.path.insert(0, p)

    # Python 3.8+ on Windows does not search PATH for extension-module
    # dependencies; register the native directories explicitly and retain the
    # handles (releasing one drops the directory from the search path).
    if hasattr(os, "add_dll_directory"):
        dll_dirs = [usd_install / "lib", usd_install / "bin", build_root]
        # Multi-config generators (Visual Studio) place the native libs in a
        # per-configuration subdirectory of the build root.
        if build_root.is_dir():
            for sub in sorted(build_root.iterdir()):
                if sub.is_dir() and (sub / "rigExec.dll").is_file():
                    dll_dirs.append(sub)
        for d in dll_dirs:
            try:
                if pathlib.Path(d).is_dir():
                    _DLL_DIR_HANDLES.append(os.add_dll_directory(str(d)))
            except OSError:
                pass  # already registered or unsupported; harmless


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None

    _setup_environment()

    from pxr import Usd, UsdGeom
    import rigexec

    # -- Schema registration ------------------------------------------------
    d = rigexec.load_schema_plugin(plugin_dir)
    print("schema plugin registered:", d)
    assert rigexec.load_schema_plugin() == d  # idempotent

    stage = Usd.Stage.CreateInMemory()

    # -- Geometry: three points along the X axis ----------------------------
    dots = UsdGeom.Points.Define(stage, "/Model/Geom/Dots")
    stage.GetPrimAtPath(dots.GetPath()).GetAttribute("points").Set(
        [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (4.0, 0.0, 0.0)])

    # -- Author the rig ------------------------------------------------------
    builder = rigexec.Builder.create(stage, "/Rig")
    assert builder.root_path == "/Rig"

    ctrl = builder.add_control("Ctrl")
    assert ctrl.valid and ctrl.path == "/Rig/Controls/Ctrl", ctrl.path

    w = builder.add_static_weight(
        "W", "/Model/Geom/Dots.points", [1.0, 0.5, 0.0])
    assert w.path == "/Rig/Weights/W", w.path

    chain = builder.new_mover_chain("chain")
    mv = chain.add_matrix_mover(
        "move", ctrl.path, w.path, "/Model/Geom/Dots.points")
    assert mv.valid and mv.path.startswith("/Rig/Movers/chain/"), mv.path

    # -- Animate the control: avars:tx = 10 at frame 100 ---------------------
    prim = stage.GetPrimAtPath(ctrl.path)
    tx = prim.GetAttribute("avars:tx")
    assert tx.IsValid(), "builder did not author avars:tx"
    # USD holds the first sample for times before it, so pin frame 0 to rest
    # explicitly; otherwise t=0 would already read the frame-100 value.
    tx.Set(0.0, Usd.TimeCode(0))
    tx.Set(10.0, Usd.TimeCode(100))

    # -- Compile + evaluate ---------------------------------------------------
    rig = rigexec.Rig(stage, "/Rig")
    rig.compile()

    order = rig.mover_order()
    assert len(order) == 1, order
    assert order[0]["type"] == "RigExecMatrixMover", order
    assert order[0]["targets"] == ["/Model/Geom/Dots.points"], order

    # At t=0 the control is at rest: nothing moves.
    pose0 = rig.evaluate(0)
    pts0 = [tuple(p) for p in pose0.moved_property("/Model/Geom/Dots.points")]
    assert pts0 == [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (4.0, 0.0, 0.0)], pts0

    # At t=100 the provider is a +X translation by 10:
    #   p' = q + w (T q - q) -> x' = x + 10w
    pose1 = rig.evaluate(100)
    pts1 = [tuple(p) for p in pose1.moved_property("/Model/Geom/Dots.points")]
    assert _close(pts1[0][0], 10.0), pts1   # weight 1: 0 + 10
    assert _close(pts1[1][0], 7.0), pts1    # weight .5: 2 + 0.5*10
    assert _close(pts1[2][0], 4.0), pts1    # weight 0: unchanged
    for p in pts1:                          # y/z untouched
        assert _close(p[1], 0.0) and _close(p[2], 0.0), pts1

    # The resolved weight field is published for overlay painting.
    wf = pose1.weight_field(w.path)
    assert [round(x, 6) for x in wf] == [1.0, 0.5, 0.0], wf

    print("OK: python bindings end-to-end")


if __name__ == "__main__":
    main()
