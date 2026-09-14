#!/usr/bin/env python
"""
Gate 1 of docs/superpowers/specs/2026-09-13-sparse-evaluation-design.md: a
constraint whose input closure AND output set are both clean replays the
outputs it recorded last generation instead of solving again.

WHY REPLAY AND NOT SKIP. finalFrames is seeded from the BASE frames and every
later step reads whatever stands there, so stepping over a clean constraint
would leave its targets at base and change what the rest of the walk sees.
Writing the recorded outputs at exactly the step where they would have been
solved re-orders nothing and avoids only the solve.

WHY THE RULE IS NOT "ITS INPUTS ARE CLEAN". commitConstraintFrames propagates
a revision onto every provider descendant of a target, composed against that
descendant's CURRENT frame -- so a dirty descendant forces the constraint to
re-run even though none of its inputs moved, because one of its OUTPUTS did.
On the biped that edge takes a fingertip drag from "0 of 98 constraints must
run" to 20 (tools/biped/spikes/r3_constraint_skippability.py). A gate built on
the inputs-only rule would have been wrong on every finger drag.

Three things have to hold, and a gate that checks only the first is worth
nothing:

  * SparseWithParityCheck must report ZERO mismatches, comparing values --
    not the work counters or diagnostics, which describe what a generation DID
    and cannot agree between two generations run back to back, because the
    second reuses every cache the first warmed;
  * a sparse drag must be BIT-IDENTICAL to a rig that never saw a drag; and
  * the gate must actually FIRE, and must NOT fire for a root drag. A sparse
    generation that replayed nothing is bit-identical to a dynamic one and
    would pass every parity check while proving the gate never worked.

Usage: test_rigexec_sparse_constraints.py [<generated schema resources dir>]
Requires the native _rigexec binding and the shipped biped, which is the only
example with enough constraints for the gate to be measurable.
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

# One probe per region the closure work found to be structurally different,
# plus the root, which must skip nothing.
_PROBES = ("index_004_l_bind_fk", "arm_l_fk_wrist_l_bind", "toe_l",
           "spine_end_ctl", "hips_ctl")
_VALUES = (4.0, 19.5, -27.25)


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
    """Sparse and the full walk, in one generation, must agree exactly."""
    _, rig, controls = _Open("sparseParity")
    rig.evaluate(1.0)
    for name in _PROBES:
        for value in _VALUES:
            rig.set_interactive_overrides([(controls[name], "avars:rz", value)])
            pose = rig.evaluate(1.0)
            assert not pose.sparse_parity_mismatches, (
                "%s rz=%g: %d sparse parity mismatch(es)"
                % (name, value, pose.sparse_parity_mismatches))
    # The release edge, where a one-sided invalidation would show up.
    rig.clear_interactive_overrides()
    pose = rig.evaluate(1.0)
    assert not pose.sparse_parity_mismatches, (
        "release: %d sparse parity mismatch(es)"
        % pose.sparse_parity_mismatches)


def TestSparseIsBitIdenticalToAFreshRig():
    """A long sparse drag must answer as a rig that never saw one."""
    _, dragged, controls = _Open("sparse")
    dragged.evaluate(1.0)
    for name in _PROBES:
        for value in _VALUES:
            dragged.set_interactive_overrides(
                [(controls[name], "avars:rz", value)])
            sparse = _Snapshot(dragged.evaluate(1.0))

            _, fresh, freshControls = _Open("dynamic")
            fresh.set_interactive_overrides(
                [(freshControls[name], "avars:rz", value)])
            full = _Snapshot(fresh.evaluate(1.0))

            label = "%s rz=%g" % (name, value)
            differing = sum(1 for k in sparse[0] if sparse[0][k] != full[0][k])
            assert not differing, (
                "%s: %d joint frame(s) differ from the full walk"
                % (label, differing))
            differing = sum(1 for k in sparse[1] if sparse[1][k] != full[1][k])
            assert not differing, (
                "%s: %d control frame(s) differ" % (label, differing))
            assert len(sparse[2]) == len(full[2]), "%s: point count" % label
            differing = sum(1 for a, b in zip(sparse[2], full[2]) if a != b)
            assert not differing, (
                "%s: %d of %d skinned point(s) differ"
                % (label, differing, len(sparse[2])))


def TestTheGateFiresAndKnowsWhenNotTo():
    """Replays must happen for a leaf drag and NOT for the root.

    The half that matters: a sparse generation that replayed nothing agrees
    with the full walk trivially and proves only that the comparison runs.
    And a root drag genuinely reaches every constraint, so replaying any of
    them would mean the dirty set is too small -- the failure that produces a
    limb in the wrong place.
    """
    _, rig, controls = _Open("sparse")
    rig.evaluate(1.0)

    def _Walk(name, value):
        rig.set_interactive_overrides([(controls[name], "avars:rz", value)])
        pose = rig.evaluate(1.0)
        return pose.sparse_constraints_replayed, pose.sparse_constraints_solved

    # A fingertip reaches almost nothing. Measured 66 of 98 replayed; asserted
    # as a wide floor so a rig edit that adds constraints does not fail it.
    replayed, solved = _Walk("index_004_l_bind_fk", 8.0)
    assert replayed + solved > 0, "no constraint steps were walked at all"
    assert replayed > solved, (
        "a fingertip drag replayed only %d of %d constraint steps"
        % (replayed, replayed + solved))

    # The root reaches everything, twice over: through the joint hierarchy and
    # through every constraint's propagation set.
    replayed, solved = _Walk("hips_ctl", 8.0)
    assert replayed == 0, (
        "a root drag replayed %d constraint step(s); the dirty set is too "
        "small and the rig can be posed wrongly" % replayed)


def TestSolverBatchesAreReusedAndKnowWhenNotTo():
    """Gate 2: a batch whose solvers cannot see what moved must not re-solve.

    The batch cache that already existed is value-based -- it compares the
    whole input vector against the one that produced the standing snapshot.
    That works perfectly for an UNCHANGED frame (0 solver evaluations) and
    misses on every DRAGGED one, because the interactive override rides in the
    shared baseOverrides prefix and makes every batch's vector differ whether
    or not the batch can see that avar. Measured before Gate 2: a fingertip
    drag re-solved all 24 aggregate solvers, exactly as many as a root drag.

    Both halves again: it must fire for a leaf, and must not fire at all for
    the root, whose drag genuinely reaches every solver.
    """
    _, rig, controls = _Open("sparse")
    rig.evaluate(1.0)

    def _Drag(name):
        # Twice: the first generation after a control CHANGES is dirty by
        # construction, and what is under test is the second one.
        rig.set_interactive_overrides([(controls[name], "avars:rz", 7.0)])
        rig.evaluate(1.0)
        rig.set_interactive_overrides([(controls[name], "avars:rz", 11.0)])
        pose = rig.evaluate(1.0)
        return pose.solver_evaluations, pose.sparse_solver_batches_reused

    solved, reused = _Drag("index_004_l_bind_fk")
    assert reused > 0, (
        "a fingertip drag reused no solver batch at all; the gate never fired")
    assert solved < 12, (
        "a fingertip drag re-solved %d aggregate solver(s); the closure says "
        "1 is downstream" % solved)

    solved, reused = _Drag("hips_ctl")
    assert reused == 0, (
        "a root drag reused %d solver batch(es); a root reaches every solver "
        "and reusing one means the dirty set is too small" % reused)


def main():
    _RegisterSchema()
    _RequireEngine(["sparse_parity_mismatches", "sparse_constraints_replayed", "sparse_constraints_solved", "sparse_solver_batches_reused"])
    if not os.path.exists(_BIPED):
        print("SKIP: %s not present" % _BIPED)
        return 0
    groups = [
        ("parity reports no mismatch", TestParityReportsNoMismatch),
        ("sparse is bit-identical to a fresh rig",
         TestSparseIsBitIdenticalToAFreshRig),
        ("the gate fires, and knows when not to",
         TestTheGateFiresAndKnowsWhenNotTo),
        ("solver batches are reused, and know when not to be",
         TestSolverBatchesAreReusedAndKnowWhenNotTo),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_SPARSE_CONSTRAINTS_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
