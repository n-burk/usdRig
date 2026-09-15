#!/usr/bin/env python
"""
What an interactive override is allowed to invalidate, and what it must not.

A gizmo drag pushes a RigExecValueOverride per mouse sample. Before the
override test was narrowed, every one of those calls cleared the epoch-scoped
skin LAYOUT cache -- the resolved rigExec:jointIndices / jointWeights /
elementSize table -- so every sample re-read and re-validated the whole
binding table.

MEASURED 2026-09-13 on examples/biped/Biped.usda (26,276 points, 137
influences), `Assemble body_geo_skin`, ms/frame:

    override set once, then evaluated        0.17
    a fresh override every frame (a drag)    2.08
    the SAME override re-pushed every frame  2.23

The third row is the one that settles it: re-pushing an IDENTICAL value
changes no answer anywhere and cost 2.23 ms of assembly and 4.9 ms of
evaluate, purely to rebuild a table that could not have moved. A layout is a
function of exactly three attributes on the skin mover, reached along the
connection walk the layout is read through
(RigExecRigEvaluator::_OverridesReachSkinLayout); an avar override cannot
reach any of them.

So there are two things to hold, and a gate that only checks the first is
worth nothing:

  * the layout must SURVIVE an override that cannot reach it, and the rig
    must answer BIT-IDENTICALLY to one that resolved the layout fresh; and
  * the layout must still be DROPPED by an override that can reach it, on
    BOTH edges of the drag, because an invalidation that never fires is
    indistinguishable from one that is wrong.

The second is asserted on the cache's own occupancy rather than on a timing:
dropping the layouts and re-reading them publishes exactly the same
deformation as keeping them, so only the count of answers the cache holds
says whether the drag paid for the re-read.

Usage: test_rigexec_skin_layout_overrides.py [<generated schema resources dir>]
Requires the native _rigexec binding (build/python on PYTHONPATH) and the
shipped biped, which is the only example with a skinned mesh big enough for
the layout cache to matter.
"""
import os
import sys

# The suite's shared bootstrap. It fixes sys.path (including the build tree's
# python directory, found from the resources argument the add_test entry
# passes), the Windows DLL search directories and PXR_PLUGINPATH_NAME.
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


def _RequireEngine():
    """Fail with something actionable when the build predates the test."""
    missing = []
    if not hasattr(rigexec.Pose, "moved_property"):
        missing.append("Pose.moved_property")
    if not hasattr(rigexec.Rig, "skin_topology_cache_size"):
        missing.append("Rig.skin_topology_cache_size")
    if missing:
        raise AssertionError(
            "this build's _rigexec is older than this test: it is missing "
            "%s. Rebuild (cmake --build <builddir>) and re-run; the module "
            "in use is %s."
            % (", ".join(missing), getattr(rigexec, "__file__", "?")))


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


def TestOnlyALayoutReachingOverrideInvalidates():
    """The layouts survive an avar drag and are dropped by a layout override.

    Both edges: an override going away invalidates a layout resolved under it
    exactly as its arrival did, and an avar override has nothing to put back
    going out either.
    """
    _, rig, controls, skins = _Open()
    assert skins, "the biped is expected to carry a skin mover"
    tip = controls["index_004_l_bind_fk"]

    rig.evaluate(1.0)
    held = rig.skin_topology_cache_size
    assert held > 0, "the first evaluate resolved no skin layout at all"

    # An avar drag, sample after sample: the layouts stay put throughout.
    for i in range(4):
        rig.set_interactive_overrides([(tip, "avars:rz", 1.0 + i)])
        assert rig.skin_topology_cache_size == held, (
            "an avar override dropped the skin layouts on the way in")
        rig.evaluate(1.0)
        assert rig.skin_topology_cache_size == held, (
            "an avar drag frame re-resolved the skin layouts")
    rig.clear_interactive_overrides()
    assert rig.skin_topology_cache_size == held, (
        "releasing an avar drag dropped the skin layouts")

    # An override that names a layout input drops them, on arrival ...
    rig.set_interactive_overrides(
        [(tip, "avars:rz", 3.0), (skins[0], "rigExec:elementSize", 4.0)])
    assert rig.skin_topology_cache_size == 0, (
        "an override naming rigExec:elementSize did not drop the skin layouts")
    rig.evaluate(1.0)
    assert rig.skin_topology_cache_size > 0, (
        "the frame under a layout override resolved no layout")
    # ... and on release, because the layout standing now was resolved
    # against the override.
    rig.clear_interactive_overrides()
    assert rig.skin_topology_cache_size == 0, (
        "releasing a layout-naming override kept a layout resolved under it")


def main():
    _RegisterSchema()
    _RequireEngine()
    if not os.path.exists(_BIPED):
        print("SKIP: %s not present" % _BIPED)
        return 0
    groups = [
        ("a drag is bit-identical to a fresh layout",
         TestDragIsBitIdenticalToAFreshLayout),
        ("release returns the authored rig",
         TestReleaseReturnsTheAuthoredRig),
        ("only a layout-reaching override invalidates",
         TestOnlyALayoutReachingOverrideInvalidates),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_SKIN_LAYOUT_OVERRIDES_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
