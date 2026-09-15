#!/usr/bin/env python
"""
Cone re-execution on the shipped biped, from Python: the baked program runs
the CLOSURE of what a drag moved and nothing else (docs/baked-step-graph.md
§7), and asking for it can never change an answer.

This is the step-graph twin of the dynamic-path Gate 1 / Gate 2 suite the
biped branch carried (docs/superpowers/specs/2026-09-13-sparse-evaluation-
design.md). There the constraint walk replayed recorded outputs and the
solver batches consulted a dirty set; here every step declares the slot
ranges it reads and writes, the edges follow, and a run executes the cone of
the sources that moved. The questions are the same ones, and a gate that
checks only the first is worth nothing:

  * BakedWithParityCheck must report ZERO mismatches across a drag, on a rig
    that really has a program -- a parity generation with no program to
    compare falls back to the dynamic path and agrees with itself;
  * a long baked drag must be BIT-IDENTICAL to a rig that never saw one; and
  * the cone must actually SKIP something for a leaf drag, and skip less for
    the root, which genuinely reaches every step through its propagation
    sets. A frame that re-ran everything publishes the same numbers as one
    that skipped the right half, so only the cluster counts say which
    happened.

Usage: test_rigexec_baked_cone.py [<generated schema resources dir>]
Requires the native _rigexec binding and the shipped biped, which is the only
example with enough steps for a cone to be measurable.
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

# One probe per region the closure work found to be structurally different,
# plus the root, which must skip the least.
_PROBES = ("index_004_l_bind_fk", "arm_l_fk_wrist_l_bind", "toe_l",
           "spine_end_ctl", "hips_ctl")
_VALUES = (4.0, 19.5, -27.25)


def _RequireEngine():
    """Fail with something actionable when the build predates the test.

    A test that exercises a new binding against an older build dies with
    `AttributeError: ... has no attribute ...`, which says nothing about what
    to do. The suite has to be green for someone who did not write it, in a
    build they did not configure -- so when it cannot be, it has to say why.
    """
    missing = [n for n in ("baked_cluster_count",
                           "baked_clusters_run_last_generation")
               if not hasattr(rigexec.Rig, n)]
    if missing:
        raise AssertionError(
            "this build's _rigexec is older than this test: Rig is missing "
            "%s. Rebuild (cmake --build <builddir>) and re-run; the module "
            "in use is %s."
            % (", ".join(missing), getattr(rigexec, "__file__", "?")))


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Open(mode):
    stage = Usd.Stage.Open(_BIPED)
    rig = rigexec.Rig(stage, _RIG)
    rig.compile()
    rig.evaluation_mode = mode
    controls = {p.GetName(): p.GetPath().pathString
                for p in stage.Traverse()
                if p.GetTypeName() == "RigExecControl"}
    # The stage is held: the Rig does not own it.
    return stage, rig, controls


def _Snapshot(pose):
    joints = {}
    for path in pose.joint_paths():
        frame = pose.joint_frame(path, True)
        joints[path] = (tuple(frame.origin), tuple(frame.x_axis),
                        tuple(frame.y_axis), tuple(frame.z_axis))
    controls = {p: tuple(pose.control_frame(p).origin)
                for p in pose.control_paths()}
    return joints, controls, tuple(tuple(p) for p in
                                   pose.moved_property(_POINTS))


def TestParityReportsNoMismatch():
    """The program and the dynamic walk, in one generation, agree exactly."""
    _, rig, controls = _Open("parity")
    assert rig.is_bakeable(), "the shipped biped is expected to bake"
    rig.evaluate(1.0)
    assert rig.baked_cluster_count > 0, (
        "parity ran with no program to compare; the check is vacuous")
    for name in _PROBES:
        for value in _VALUES:
            rig.set_interactive_overrides([(controls[name], "avars:rz", value)])
            pose = rig.evaluate(1.0)
            assert not pose.baked_parity_mismatches, (
                "%s rz=%g: %d baked parity mismatch(es)"
                % (name, value, pose.baked_parity_mismatches))
    # The release edge, where a one-sided invalidation would show up.
    rig.clear_interactive_overrides()
    pose = rig.evaluate(1.0)
    assert not pose.baked_parity_mismatches, (
        "release: %d baked parity mismatch(es)" % pose.baked_parity_mismatches)


def TestABakedDragIsBitIdenticalToAFreshRig():
    """A long baked drag must answer as a dynamic rig that never saw one."""
    _, dragged, controls = _Open("baked")
    dragged.evaluate(1.0)
    for name in _PROBES:
        for value in _VALUES:
            dragged.set_interactive_overrides(
                [(controls[name], "avars:rz", value)])
            baked = _Snapshot(dragged.evaluate(1.0))

            _, fresh, freshControls = _Open("dynamic")
            fresh.set_interactive_overrides(
                [(freshControls[name], "avars:rz", value)])
            full = _Snapshot(fresh.evaluate(1.0))

            label = "%s rz=%g" % (name, value)
            differing = sum(1 for k in baked[0] if baked[0][k] != full[0][k])
            assert not differing, (
                "%s: %d joint frame(s) differ from the dynamic walk"
                % (label, differing))
            differing = sum(1 for k in baked[1] if baked[1][k] != full[1][k])
            assert not differing, (
                "%s: %d control frame(s) differ" % (label, differing))
            assert len(baked[2]) == len(full[2]), "%s: point count" % label
            differing = sum(1 for a, b in zip(baked[2], full[2]) if a != b)
            assert not differing, (
                "%s: %d of %d skinned point(s) differ"
                % (label, differing, len(baked[2])))


def TestTheConeSkipsAndKnowsHowMuch():
    """A leaf drag runs a fraction of the clusters; the root runs more.

    The half that matters: a generation that skipped nothing agrees with the
    full program trivially and proves only that the comparison runs. And a
    root drag genuinely reaches every constraint through the joint hierarchy
    and every propagation set, so it must close over more of the graph than
    a fingertip does -- a cone that is too small is the failure that puts a
    limb in the wrong place.
    """
    _, rig, controls = _Open("baked")
    rig.evaluate(1.0)
    total = rig.baked_cluster_count
    assert total > 0, "the shipped biped is expected to bake"

    def _Drag(name):
        # Twice: the first generation after a control CHANGES compares the
        # avars and finds the whole cone; what is under test is the steady
        # state of a drag, where each sample moves the same control again.
        rig.set_interactive_overrides([(controls[name], "avars:rz", 7.0)])
        rig.evaluate(1.0)
        rig.set_interactive_overrides([(controls[name], "avars:rz", 11.0)])
        rig.evaluate(1.0)
        return rig.baked_clusters_run_last_generation

    leaf = _Drag("index_004_l_bind_fk")
    assert 0 < leaf < total, (
        "a fingertip drag ran %d of %d cluster(s); the cone skipped nothing"
        % (leaf, total))
    root = _Drag("hips_ctl")
    assert root > leaf, (
        "a root drag ran %d cluster(s) against a fingertip's %d; the root "
        "reaches every joint and the cone is too small" % (root, leaf))

    # And a frame in which nothing moved at all runs no more than the leaf.
    rig.evaluate(1.0)
    still = rig.baked_clusters_run_last_generation
    assert still <= leaf, (
        "a frame with nothing changed ran %d cluster(s), more than a "
        "fingertip drag's %d" % (still, leaf))


def main():
    _RegisterSchema()
    _RequireEngine()
    if not os.path.exists(_BIPED):
        print("SKIP: %s not present" % _BIPED)
        return 0
    groups = [
        ("parity reports no mismatch", TestParityReportsNoMismatch),
        ("a baked drag is bit-identical to a fresh rig",
         TestABakedDragIsBitIdenticalToAFreshRig),
        ("the cone skips, and knows how much",
         TestTheConeSkipsAndKnowsHowMuch),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_BAKED_CONE_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
