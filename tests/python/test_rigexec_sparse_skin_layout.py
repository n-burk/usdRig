#!/usr/bin/env python
"""
What an interactive override is allowed to invalidate, and what it must not.

A gizmo drag pushes a RigExecValueOverride per mouse sample. Until Gate 3 of
docs/superpowers/specs/2026-09-13-sparse-evaluation-design.md, every one of
those calls cleared the epoch-scoped skin LAYOUT cache -- the resolved
rigExec:jointIndices / jointWeights / elementSize table -- so every sample
re-read and re-validated the whole binding table.

MEASURED 2026-09-13 on examples/biped/Biped.usda (26,276 points, 137
influences), `Assemble body_geo_skin`, ms/frame:

    override set once, then evaluated        0.17
    a fresh override every frame (a drag)    2.08
    the SAME override re-pushed every frame  2.23

The third row is the one that settles it: re-pushing an IDENTICAL value
changes no answer anywhere and cost 2.23 ms of assembly and 4.9 ms of
evaluate, purely to rebuild a table that could not have moved. A layout is a
function of exactly three attributes on the skin mover; an avar override
cannot reach any of them.

So there are two things to hold, and a gate that only checks the first is
worth nothing:

  * the layout must SURVIVE an override that cannot reach it, and the rig
    must answer BIT-IDENTICALLY to one that resolved the layout fresh; and
  * the layout must still be DROPPED by an override that can reach it,
    because an invalidation that never fires is indistinguishable from one
    that is wrong.

Usage: test_rigexec_sparse_skin_layout.py [<generated schema resources dir>]
Requires the native _rigexec binding (build/python on PYTHONPATH) and the
shipped biped, which is the only example with a skinned mesh big enough for
the layout cache to matter.
"""
import os
import sys

# The suite's shared bootstrap. It fixes sys.path (including the build tree's
# python directory, found from the resources argument the add_test entry
# passes), the Windows DLL search directories and PXR_PLUGINPATH_NAME.
#
# Not rigexec_test_env.SetupPluginTest: that one deliberately covers only the
# usdview-plugin tests and does NOT put <build>/python on sys.path, so a test
# importing rigexec passes in a shell that already exports PYTHONPATH and
# fails under a plain ctest. Which is exactly what these two did.
from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Plug, Usd  # noqa: E402

import rigexec  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_BIPED = os.path.join(_REPO, "examples", "biped", "Biped.usda")
_RIG = "/Biped/Rig"
_POINTS = "/Biped/Geom/body_geo.points"

# One probe per region of the rig the closure work found to be structurally
# different: a leaf that reaches 2% of the joints, a mid-limb, a deep spine
# control, and the root that reaches everything.
_PROBES = ("index_004_l_bind_fk", "arm_l_fk_wrist_l_bind", "spine_end_ctl",
           "hips_ctl")
_VALUES = (5.0, 17.5, -32.25)


def _RequireEngine(names):
    """Fail with something actionable when the build predates the test.

    A test that exercises a new binding against an older build dies with
    `AttributeError: '_rigexec.Pose' object has no attribute ...`, which says
    nothing about what to do. That happened: two build directories configured
    before these bindings landed reported this file as a failure, and the
    error pointed at the test rather than at the build.

    The suite has to be green for someone who did not write it, in a build
    they did not configure -- so when it cannot be, it has to say why.
    """
    import rigexec as _rigexec
    missing = [n for n in names if not hasattr(_rigexec.Pose, n)]
    if missing:
        raise AssertionError(
            "this build's _rigexec is older than this test: Pose is missing "
            "%s. Rebuild (cmake --build <builddir>) and re-run; the module "
            "in use is %s."
            % (", ".join(missing), getattr(_rigexec, "__file__", "?")))


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Open():
    stage = Usd.Stage.Open(_BIPED)
    rig = rigexec.Rig(stage, _RIG)
    rig.compile()
    controls = {p.GetName(): p.GetPath().pathString
                for p in stage.Traverse()
                if p.GetTypeName() == "RigExecControl"}
    skins = [p.GetPath().pathString for p in stage.Traverse()
             if p.GetTypeName() == "RigExecSkinMover"]
    # Held: the Rig does not own the stage, and a collected stage takes the
    # evaluator's world with it.
    return stage, rig, controls, skins


def _Snapshot(pose):
    joints = {}
    for path in pose.joint_paths():
        frame = pose.joint_frame(path, True)
        joints[path] = (tuple(frame.origin), tuple(frame.x_axis),
                        tuple(frame.y_axis), tuple(frame.z_axis))
    return joints, tuple(tuple(p) for p in pose.moved_property(_POINTS))


def _AssertIdentical(label, a, b):
    aj, ap = a
    bj, bp = b
    differing = sum(1 for k in aj if aj[k] != bj.get(k))
    assert not differing, (
        "%s: %d joint frame(s) differ from a freshly resolved layout"
        % (label, differing))
    assert len(ap) == len(bp), (
        "%s: point count %d vs %d" % (label, len(ap), len(bp)))
    moved = sum(1 for x, y in zip(ap, bp) if x != y)
    assert not moved, (
        "%s: %d of %d skinned point(s) differ from a freshly resolved layout"
        % (label, moved, len(ap)))


def TestDragIsBitIdenticalToAFreshLayout():
    """A retained layout must answer exactly as one resolved from scratch.

    The drag rig accumulates every sample below without ever re-resolving its
    layout; each result is compared against a rig that has never been dragged
    at all, which resolves the layout on its first evaluate.
    """
    _, dragged, controls, _ = _Open()
    dragged.evaluate(1.0)
    for name in _PROBES:
        for value in _VALUES:
            dragged.set_interactive_overrides(
                [(controls[name], "avars:rz", value)])
            warm = _Snapshot(dragged.evaluate(1.0))

            _, fresh, freshControls, _ = _Open()
            fresh.set_interactive_overrides(
                [(freshControls[name], "avars:rz", value)])
            _AssertIdentical("%s rz=%g" % (name, value), warm,
                             _Snapshot(fresh.evaluate(1.0)))


def TestReleaseReturnsTheAuthoredRig():
    """Clearing the overrides after a long drag is the authored rig again.

    The release edge is where a one-sided invalidation shows up: a layout kept
    across the drag has to be equally valid once the drag is gone.
    """
    _, reference, _, _ = _Open()
    expected = _Snapshot(reference.evaluate(1.0))

    _, dragged, controls, _ = _Open()
    dragged.evaluate(1.0)
    for name in _PROBES:
        for value in _VALUES:
            dragged.set_interactive_overrides(
                [(controls[name], "avars:rz", value)])
            dragged.evaluate(1.0)
    dragged.clear_interactive_overrides()
    _AssertIdentical("release after %d samples" % (len(_PROBES) * len(_VALUES)),
                     _Snapshot(dragged.evaluate(1.0)), expected)


def _AssembleMs(rig, setter, samples=8):
    rig.clear_profile()
    rig.profiling_enabled = True
    for i in range(samples):
        setter(i)
        rig.evaluate(1.0)
    rig.profiling_enabled = False
    total = sum(row["total_us"] for row in rig.profile_summary()
                if row["name"].startswith("Assemble "))
    return total / float(samples) / 1000.0


def TestALayoutReachingOverrideStillInvalidates():
    """The gate must still fire for an override that CAN reach the layout.

    Timing rather than a value assertion, because the layout resolves to the
    same table either way -- what is under test is whether the work was done,
    and the only observable difference is that it cost something. A gate that
    never fires would pass every value assertion ever written.
    """
    _, rig, controls, skins = _Open()
    assert skins, "the biped is expected to carry a skin mover"
    for _ in range(3):
        rig.evaluate(1.0)
    tip = controls["index_004_l_bind_fk"]

    held = _AssembleMs(rig, lambda i: rig.set_interactive_overrides(
        [(tip, "avars:rz", 1.0 + i)]))
    fired = _AssembleMs(rig, lambda i: rig.set_interactive_overrides(
        [(tip, "avars:rz", 1.0 + i),
         (skins[0], "rigExec:elementSize", 4.0)]))

    # Measured 0.06 vs 1.75 ms on the biped. Asserted as a wide ratio rather
    # than an absolute, so the gate survives a faster machine and a bigger
    # mesh; anything near 1.0 means the clear stopped happening.
    assert fired > held * 4.0, (
        "an override naming rigExec:elementSize did not re-resolve the skin "
        "layout: %.3f ms held vs %.3f ms fired" % (held, fired))


def main():
    _RegisterSchema()
    _RequireEngine(["moved_property"])
    if not os.path.exists(_BIPED):
        print("SKIP: %s not present" % _BIPED)
        return 0
    groups = [
        ("a drag is bit-identical to a fresh layout",
         TestDragIsBitIdenticalToAFreshLayout),
        ("release returns the authored rig",
         TestReleaseReturnsTheAuthoredRig),
        ("a layout-reaching override still invalidates",
         TestALayoutReachingOverrideStillInvalidates),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_SPARSE_SKIN_LAYOUT_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
