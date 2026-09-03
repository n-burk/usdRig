#!/usr/bin/env python3
"""Build a small FK-skinned arm asset with the RigExec Python API.

This script authors a complete, self-contained USD file:

  * a strip mesh (the "skin") under /ArmAsset/Geom,
  * three controls (Shoulder, Elbow, Wrist) with animated rz channels,
  * a nested joint hierarchy posed by an FK chain solver,
  * one static weight object per segment over the strip points, and
  * a mover chain of matrix movers that skinned the strip through each
    joint's final pose.

After saving it re-opens the file, compiles it with the RigExec evaluator,
and verifies:

  * rest pose in -> rest pose out (joint origins equal their authored rests
    and every strip point is unmoved at t=0),
  * at t=100 the FK solve preserves segment lengths, and every point that
    carries weight 1 on exactly one joint follows that joint's final
    rest-to-pose map rigidly.

Usage:
    python make_fk_arm.py [output.usda]

The output defaults to <this directory>/out/fk_arm.usda. No environment
setup is required; the bootstrap below mirrors
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
    build_dir = pathlib.Path(
        os.environ.get("RIGEXEC_BUILD_DIR") or (repo_root / "build"))

    # pxr must resolve from the USD install, never a global copy. The layout
    # differs by platform: Windows installs to <usd>/Lib/site-packages, while
    # POSIX installs under <usd>/lib/python*/site-packages.
    site_candidates = [usd_install / "Lib" / "site-packages"]
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

RIG = "/ArmAsset/Rig"
POINTS_ATTR = "/ArmAsset/Geom/ArmStrip.points"

# (name, rest position), root -> leaf; the arm runs along +X at y=0.
JOINTS = [
    ("Shoulder", (0.0, 0.0, 0.0)),
    ("Elbow", (2.0, 0.0, 0.0)),
    ("Wrist", (4.0, 0.0, 0.0)),
]

# rz channel animation per control: rest at frame 0, posed at frame 100.
POSE_DEG = {"Shoulder": 30.0, "Elbow": 60.0, "Wrist": 15.0}


def _joint_path(i):
    """Full path of the i-th joint (nested under Joints, root first)."""
    return RIG + "/Joints/" + "/".join(n for n, _ in JOINTS[:i + 1])


# ---------------------------------------------------------------------------
# Geometry and weights.
# ---------------------------------------------------------------------------

def strip_points():
    """Two rows of points along the arm: one column per unit x, z = +/-0.5."""
    pts = []
    for x in range(5):
        pts.append((float(x), 0.0, -0.5))
        pts.append((float(x), 0.0, +0.5))
    return pts


# Dense weights over the strip points (order as in strip_points()). Each
# joint's own column carries weight 1 on exactly its segment mover so those
# points follow their joint rigidly; mid-segment columns blend between the
# two adjacent joints.
WEIGHTS = {
    "Shoulder": [1, 1, .5, .5, 0, 0, 0, 0, 0, 0],
    "Elbow":    [0, 0, .5, .5, 1, 1, .5, .5, 0, 0],
    "Wrist":    [0, 0, 0, 0, 0, 0, .5, .5, 1, 1],
}


# ---------------------------------------------------------------------------
# Small math helpers (row-major matrix4d == USD convention).
# ---------------------------------------------------------------------------

def tx(x, y=0.0, z=0.0):
    """Row-major matrix4d translating by (x, y, z)."""
    m = rigexec.identity()
    m[12], m[13], m[14] = x, y, z
    return m


def transform(m, v):
    """Affine part of a row-major matrix4d applied to a 3-vector."""
    return (
        v[0] * m[0] + v[1] * m[4] + v[2] * m[8] + m[12],
        v[0] * m[1] + v[1] * m[5] + v[2] * m[9] + m[13],
        v[0] * m[2] + v[1] * m[6] + v[2] * m[10] + m[14],
    )


def dist(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


# ---------------------------------------------------------------------------
# Authoring.
# ---------------------------------------------------------------------------

def build(out_path):
    rigexec.load_schema_plugin()  # register the codeless schema first

    stage = Usd.Stage.CreateInMemory()

    asset = UsdGeom.Xform.Define(stage, "/ArmAsset")
    stage.SetDefaultPrim(asset.GetPrim())

    # -- Geometry: a flat strip the arm bends. ------------------------------
    pts = strip_points()
    mesh = UsdGeom.Mesh.Define(stage, "/ArmAsset/Geom/ArmStrip")
    mesh.CreatePointsAttr().Set(pts)
    faces = []
    for i in range(4):  # one quad per column pair
        a = 2 * i
        faces += [a, a + 1, a + 3, a + 2]
    mesh.CreateFaceVertexCountsAttr().Set([4] * 4)
    mesh.CreateFaceVertexIndicesAttr().Set(faces)
    mesh.CreateSubdivisionSchemeAttr().Set("none")

    # -- Rig: controls and joints at the same rest positions. ---------------
    builder = rigexec.Builder.create(stage, RIG, "ArmAsset")

    controls, joints = {}, {}
    parent_joint = None
    for name, pos in JOINTS:
        controls[name] = builder.add_control(name, tx(*pos))
        joints[name] = builder.add_joint(
            name, tx(*pos), parent_joint=parent_joint)
        parent_joint = joints[name]

    # Animate each control's rz channel. The explicit frame-0 key matters:
    # USD holds the first sample backward, so without it every time would
    # read the posed value.
    for name, deg in POSE_DEG.items():
        rz = stage.GetPrimAtPath(controls[name].path).GetAttribute("avars:rz")
        rz.Set(0.0, Usd.TimeCode(0))
        rz.Set(float(deg), Usd.TimeCode(100))

    # -- FK solver wiring controls to joints in root -> leaf order. ---------
    fk = builder.add_fk_chain("ArmFK")
    names = [n for n, _ in JOINTS]
    fk.set_controls([controls[n].path for n in names])
    fk.set_joints([joints[n].path for n in names])

    # -- One static weight object per segment. ------------------------------
    weights = {
        n: builder.add_static_weight(n + "W", POINTS_ATTR, WEIGHTS[n])
        for n in names
    }

    # -- Skin: one matrix mover per joint, reading the final pose. ----------
    chain = builder.new_mover_chain("Skin")
    # Sibling movers execute in reverse composed (add) order, so adding the
    # joints root-to-leaf makes execution leaf-first -- mirroring
    # 01_FkChainTail.usda's nested chain, whose deepest mover runs first.
    # Each segment's weights are disjoint, so any sibling order gives the
    # same result; this one matches the reference application order.
    for name in names:
        chain.add_matrix_mover(
            name + "Skin", joints[name].path, weights[name].path,
            POINTS_ATTR, read_phase="final")

    stage.Export(str(out_path))


# ---------------------------------------------------------------------------
# Verification: reopen the saved file and evaluate it.
# ---------------------------------------------------------------------------

def verify(path):
    stage = Usd.Stage.Open(str(path))
    rigexec.load_schema_plugin()
    rig = rigexec.Rig(stage, RIG)
    rig.compile()

    pts0 = strip_points()

    # -- Rest pose: everything at its authored position. --------------------
    p0 = rig.evaluate(0.0)
    assert p0.valid, "rest-pose evaluation failed"
    for i, (name, pos) in enumerate(JOINTS):
        o = p0.joint_frame(_joint_path(i)).origin
        assert dist(o, pos) < 1e-9, f"{name} rest origin {o} != {pos}"
    moved0 = p0.moved_property(POINTS_ATTR)
    dev0 = max(dist(m, q) for m, q in zip(moved0, pts0))
    print(f"  t=0   joint origins == rests; max strip deviation: {dev0:.3e}")
    assert dev0 < 1e-5

    # -- Posed at frame 100. --------------------------------------------------
    p1 = rig.evaluate(100.0)
    origins = [p1.joint_frame(_joint_path(i)).origin for i in range(len(JOINTS))]
    for (name, pos), o in zip(JOINTS, origins):
        print(f"  t=100 {name:<8} rest=({pos[0]:g},{pos[1]:g},{pos[2]:g}) "
              f"posed=({o[0]:.3f}, {o[1]:.3f}, {o[2]:.3f})")

    # Rigid FK: segment lengths are preserved by pure rotations.
    d1, d2 = dist(origins[1], origins[0]), dist(origins[2], origins[1])
    print(f"  t=100 segment lengths: {d1:.6f}, {d2:.6f} (rest: 2, 2)")
    assert abs(d1 - 2.0) < 1e-6 and abs(d2 - 2.0) < 1e-6

    # Points weighted 1 on exactly one joint follow that joint's final
    # rest-to-pose map rigidly (the x=0, x=2 and x=4 columns).
    moved1 = p1.moved_property(POINTS_ATTR)
    for i in range(len(JOINTS)):
        T = p1.joint_matrix(_joint_path(i))
        for idx in (4 * i, 4 * i + 1):
            expected = transform(T, pts0[idx])
            got = moved1[idx]
            assert dist(got, expected) < 1e-5, \
                f"point {idx} {got} != rigid expectation {expected}"
    print("  t=100 rigid points follow their joint's final map: OK")

    # And the arm actually bent.
    assert dist(origins[2], JOINTS[2][1]) > 0.5, "wrist did not move"


def main(argv):
    out = pathlib.Path(argv[1]) if len(argv) > 1 else \
        pathlib.Path(__file__).resolve().parent / "out" / "fk_arm.usda"
    out.parent.mkdir(parents=True, exist_ok=True)
    print(f"authoring {out}")
    build(out)
    print("verifying")
    verify(out)
    print(f"OK: wrote and verified {out}")


if __name__ == "__main__":
    main(sys.argv)
