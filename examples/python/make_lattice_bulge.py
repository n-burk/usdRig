#!/usr/bin/env python3
"""Build a lattice-bulge deformation asset with the RigExec Python API.

Programmatic counterpart of examples/06_LatticeBulge.usda:

  * a native Points cage (bind at default time, bulged keyframe at frame 50),
  * a thin slab mesh inside the cage,
  * one root joint (as in the hand-authored example), and
  * a mover chain: bernstein lattice -> smooth -> volume correct.

After saving it re-opens the file, compiles it with the RigExec evaluator,
and verifies that at the bind cage (frames 0 and 100) only the smoothing /
volume-correction passes displace the slab modestly and identically, while
frame 50 bulges the middle of the slab outward more than its ends.

Usage:
    python make_lattice_bulge.py [output.usda]

The output defaults to <this directory>/out/lattice_bulge.usda. No
environment setup is required; the bootstrap below mirrors
tests/python/test_rigexec_python.py and locates the USD install and build
tree relative to this file.
"""

import os
import pathlib
import sys

# Keep os.add_dll_directory handles alive for the lifetime of the process:
# on Windows, CPython drops a DLL search directory when its handle is
# garbage-collected.
_DLL_DIR_HANDLES = []


def _bootstrap():
    """Make this script runnable with a bare python.exe (no env setup)."""
    here = pathlib.Path(__file__).resolve()
    repo_root = here.parents[2]            # .../usdRig/usdRig
    usd_install = pathlib.Path(
        os.environ.get("RIGEXEC_USD_INSTALL")
        or (repo_root.parent / "usd-install"))
    build_dir = repo_root / "build"

    # pxr must resolve from the USD install, never a global copy. The layout
    # differs by platform: Windows installs to <usd>/Lib/site-packages, while
    # POSIX installs under <usd>/lib/python*/site-packages.
    site_candidates = [usd_install / "Lib" / "site-packages"]
    if not (usd_install / "Lib").is_dir():
        site_candidates.extend(sorted(usd_install.glob("lib/python*/site-packages")))

    # _rigexec plus the copied rigexec package live in <build>/python for
    # single-config generators and under a per-configuration subdirectory for
    # multi-config ones (Visual Studio).
    py_dir = build_dir / "python"
    python_candidates = []
    if py_dir.is_dir():
        python_candidates.append(py_dir)
        for sub in sorted(py_dir.iterdir()):
            if sub.is_dir() and any(sub.glob("_rigexec*")):
                python_candidates.append(sub)

    for d in tuple(python_candidates) + tuple(site_candidates):
        p = str(d)
        if d.is_dir() and p not in sys.path:
            sys.path.insert(0, p)

    pluginpath = usd_install / "lib" / "usd"
    if pluginpath.is_dir():
        os.environ["PXR_PLUGINPATH_NAME"] = str(pluginpath)

    # Python 3.8+ (Windows) does not search PATH for extension-module deps;
    # register the native directories explicitly and retain the handles.
    if hasattr(os, "add_dll_directory"):
        dll_dirs = [build_dir, usd_install / "bin", usd_install / "lib"]
        # Multi-config generators (Visual Studio) place the native libs in a
        # per-configuration subdirectory of the build root.
        if build_dir.is_dir():
            for sub in sorted(build_dir.iterdir()):
                if sub.is_dir() and (sub / "rigExec.dll").is_file():
                    dll_dirs.append(sub)
        for d in dll_dirs:
            try:
                if d.is_dir():
                    _DLL_DIR_HANDLES.append(os.add_dll_directory(str(d)))
            except OSError:
                pass  # already registered or unsupported; harmless


_bootstrap()

from pxr import Usd, UsdGeom  # noqa: E402
import rigexec                 # noqa: E402

# ---------------------------------------------------------------------------
# Layout constants.
# ---------------------------------------------------------------------------

RIG = "/LatticeAsset/Rig"
CAGE = "/LatticeAsset/Geom/Cage"
SLAB_POINTS_ATTR = "/LatticeAsset/Geom/Slab.points"

BULGE_FRAME = 50


def _cage_layer(z, half):
    """One cage layer at height z with x/y extent +/-half (x-fastest)."""
    return [(-half, -half, z), (half, -half, z),
            (-half, half, z), (half, half, z)]


# Bind cage: 2x2x3 grid spanning x,y in [-1,1] and z in [0,4].
CAGE_BIND = _cage_layer(0.0, 1) + _cage_layer(2.0, 1) + _cage_layer(4.0, 1)

# Posed cage: the middle layer bulges outward to +/-2 (frame BULGE_FRAME).
CAGE_BULGED = _cage_layer(0.0, 1) + _cage_layer(2.0, 2) + _cage_layer(4.0, 1)

# Slab: a thin wall in the XZ plane (x = +/-0.5), z from 0 to 4.
SLAB_POINTS = [
    p for z in range(5)
    for p in ((-0.5, 0.0, float(z)), (0.5, 0.0, float(z)))
]


def dist(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


# ---------------------------------------------------------------------------
# Authoring.
# ---------------------------------------------------------------------------

def build(out_path):
    rigexec.load_schema_plugin()  # register the codeless schema first

    stage = Usd.Stage.CreateInMemory()

    asset = UsdGeom.Xform.Define(stage, "/LatticeAsset")
    stage.SetDefaultPrim(asset.GetPrim())

    # -- The cage: default-time points are the bind cage; timeSamples pose it.
    cage = UsdGeom.Points.Define(stage, CAGE)
    pts = cage.CreatePointsAttr()
    pts.Set(CAGE_BIND)                        # default time == bind cage
    pts.Set(CAGE_BIND, Usd.TimeCode(0))       # explicit rest key: without it,
                                               # t=0 would read the FIRST
                                               # sample (the bulged cage)
    pts.Set(CAGE_BULGED, Usd.TimeCode(BULGE_FRAME))
    pts.Set(CAGE_BIND, Usd.TimeCode(100))     # back to bind
    cage.CreateVisibilityAttr().Set("invisible")

    # -- The slab mesh deformed by the lattice. ------------------------------
    slab = UsdGeom.Mesh.Define(stage, "/LatticeAsset/Geom/Slab")
    slab.CreatePointsAttr().Set(SLAB_POINTS)
    faces = []
    for i in range(4):  # one quad per z step
        a = 2 * i
        faces += [a, a + 1, a + 3, a + 2]
    slab.CreateFaceVertexCountsAttr().Set([4] * 4)
    slab.CreateFaceVertexIndicesAttr().Set(faces)
    slab.CreateSubdivisionSchemeAttr().Set("none")

    # -- Rig: one root joint (as in the hand-authored example). --------------
    builder = rigexec.Builder.create(stage, RIG, "LatticeAsset")
    m = rigexec.identity()
    m[12], m[13], m[14] = 0.0, 0.0, 2.0
    builder.add_joint("SlabRoot", m)

    # -- Deformation chain: lattice -> smooth -> volume correct. -------------
    # Sibling movers execute in reverse composed (add) order, so the outermost
    # pass is added first and the lattice deformation last; execution then
    # applies deepest-first exactly like 06_LatticeBulge.usda's nested chain:
    # lattice -> smooth -> volume correct.
    chain = builder.new_mover_chain("Geometry")
    chain.add_volume_correct_mover("VolumeCorrect", 0.35, SLAB_POINTS_ATTR)
    chain.add_smooth_mover("Smooth", 0.6, SLAB_POINTS_ATTR)
    chain.add_lattice_mover(
        "CageDeform", CAGE, 2, 2, 3, "bernstein", SLAB_POINTS_ATTR)

    stage.Export(str(out_path))


# ---------------------------------------------------------------------------
# Verification: reopen the saved file and evaluate it.
# ---------------------------------------------------------------------------

def verify(path):
    stage = Usd.Stage.Open(str(path))
    rigexec.load_schema_plugin()
    rig = rigexec.Rig(stage, RIG)
    rig.compile()

    # -- Bind cage (frame 0): the lattice is identity, so only the smoothing
    # and volume-correction passes act. Every slab point sits on the boundary
    # of this two-wide strip, so Laplacian smoothing displaces them modestly;
    # a torn or unbounded result would show up here.
    p0 = rig.evaluate(0.0)
    assert p0.valid, "rest-pose evaluation failed"
    moved0 = p0.moved_property(SLAB_POINTS_ATTR)
    dev0 = max(dist(m, q) for m, q in zip(moved0, SLAB_POINTS))
    print(f"  t=0   max |moved - authored| over slab points: {dev0:.3e}")
    assert dev0 < 0.5, "bind-cage evaluation displaced the slab too far"

    # -- The cage returns to bind at frame 100: output must match t=0 exactly.
    p100 = rig.evaluate(100.0)
    moved100 = p100.moved_property(SLAB_POINTS_ATTR)
    dev100 = max(dist(m, q) for m, q in zip(moved100, moved0))
    print(f"  t=100 max |moved - frame-0| over slab points: {dev100:.3e}")
    assert dev100 < 1e-6, "cage did not return to bind at frame 100"

    # -- Posed at the bulge keyframe: measure the frame-to-frame animation.
    # The smoothing/volume passes leave a static offset from the authored
    # points (visible above), so distance-from-authored would hide the bulge;
    # comparing against the bind-cage output isolates what the cage animates:
    # when it bulges, the middle of the slab must move more than its ends.
    p50 = rig.evaluate(float(BULGE_FRAME))
    moved50 = p50.moved_property(SLAB_POINTS_ATTR)
    anim = [dist(m, q) for m, q in zip(moved50, moved0)]
    print(f"  t={BULGE_FRAME}   per-point |animated| displacement vs frame 0:")
    for z in range(5):
        print(f"          z={z}: {anim[2 * z]:.4f} / {anim[2 * z + 1]:.4f}")

    mid = max(anim[4:6])      # the z=2 row, at the bulge center
    ends = max(max(anim[0:2]), max(anim[8:10]))
    print(f"          middle animation {mid:.4f} vs end animation "
          f"{ends:.4f}")
    assert mid > 0.1, "middle of the slab did not animate at the bulge keyframe"
    assert mid > ends, "bulge is not concentrated in the middle"


def main(argv):
    out = pathlib.Path(argv[1]) if len(argv) > 1 else \
        pathlib.Path(__file__).resolve().parent / "out" / "lattice_bulge.usda"
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"authoring {out}")
    build(out)
    print("verifying")
    verify(out)
    print(f"OK: wrote and verified {out}")


if __name__ == "__main__":
    main(sys.argv)
