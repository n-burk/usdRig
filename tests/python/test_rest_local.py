#!/usr/bin/env python
"""
A joint's rest offset is relative to its parent frame provider.

Moving a parent's rest must carry its descendants, the way moving its
avars or default channels already does. Before this change
computeRestFrame declared no namespace ancestor among its inputs
(computations.cpp:415), so a parent's rest edit moved the parent alone
and every joint's rest was absolute in asset space.

Usage: test_rest_local.py [<generated schema resources dir>]
Requires the native _rigexec binding (build-python/python on PYTHONPATH).
"""
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.normpath(os.path.join(
    _HERE, "..", "..", "examples", "components", "spider_leg_ik.usd"))
_SOLVER = "/RigRoot/Solvers/RigExecTwoBoneIk1"
_JOINTS = ("/RigRoot/Joints/Shoulder",
           "/RigRoot/Joints/Shoulder/ankle",
           "/RigRoot/Joints/Shoulder/ankle/foot")


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecJoint"),
           "RigExecJoint schema is not registered")


def _Origins(stage):
    """World joint origins, with the IK solver unbound."""
    import _rigexec
    rig = _rigexec.Rig(stage, "/RigRoot")
    rig.compile()
    pose = rig.evaluate(0.0)
    return [tuple(pose.joint_frame(j, True).to_matrix4()[12:15])
            for j in _JOINTS]


def _OpenDisconnected():
    """The spider leg with the solver unbound, so rests drive directly."""
    stage = Usd.Stage.Open(_EXAMPLE)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    stage.GetPrimAtPath(_SOLVER).GetRelationship(
        "rigExec:joints").SetTargets([])
    return stage


def _Bump(stage, primPath, name, delta):
    prim = stage.GetPrimAtPath(primPath)
    attr = prim.GetAttribute(name) or prim.CreateAttribute(
        name, Sdf.ValueTypeNames.Double)
    attr.Set((attr.Get() or 0.0) + delta)


def TestParentRestCarriesDescendants():
    base = _Origins(_OpenDisconnected())
    stage = _OpenDisconnected()
    _Bump(stage, _JOINTS[0], "rest:tx", 3.0)
    moved = _Origins(stage)
    for index, joint in enumerate(_JOINTS):
        delta = moved[index][0] - base[index][0]
        _Check(abs(delta - 3.0) < 1e-9,
               "%s moved by %.6f in x, expected 3.0: a parent's rest edit "
               "must carry every descendant" % (joint, delta))
        for axis in (1, 2):
            _Check(abs(moved[index][axis] - base[index][axis]) < 1e-9,
                   "%s moved off the x axis" % joint)


def TestRestMatchesAvarPropagation():
    """A rest edit and an avar edit of the same size move the chain alike."""
    restStage = _OpenDisconnected()
    _Bump(restStage, _JOINTS[0], "rest:tx", 3.0)
    restOrigins = _Origins(restStage)
    avarStage = _OpenDisconnected()
    _Bump(avarStage, _JOINTS[0], "avars:tx", 3.0)
    avarOrigins = _Origins(avarStage)
    for index, joint in enumerate(_JOINTS):
        for axis in range(3):
            _Check(abs(restOrigins[index][axis]
                       - avarOrigins[index][axis]) < 1e-9,
                   "%s: rest:tx and avars:tx disagree on axis %d "
                   "(%.6f vs %.6f)" % (joint, axis,
                                       restOrigins[index][axis],
                                       avarOrigins[index][axis]))


def TestTopLevelProviderUnchanged():
    """A provider with no RigExec ancestor keeps its absolute rest frame."""
    stage = _OpenDisconnected()
    origin = _Origins(stage)[0]
    # Read from the asset rather than written down here. The property being
    # asserted is that a top-level provider's evaluated frame IS its authored
    # rest -- there is no parent to be relative to -- and a literal copy of the
    # number only asserts that as long as nobody re-authors the rig. Somebody
    # did (2e95684 "spider leg" moved the shoulder from 4.5459 to 4.6381) and
    # the test failed for describing an asset that no longer existed, which is
    # the opposite of what it is for.
    shoulder = stage.GetPrimAtPath(_JOINTS[0])
    authored = shoulder.GetAttribute("rest:ty").Get()
    _Check(authored is not None,
           "%s authors no rest:ty; this test needs one to compare against"
           % _JOINTS[0])
    _Check(abs(origin[1] - authored) < 1e-9,
           "Shoulder is at y=%.9f, not its authored height %.9f; the "
           "top-level rest frame changed meaning, which it must not"
           % (origin[1], authored))
    # The other two axes are authored zero, and the frame has to agree: a
    # top-level provider with a non-zero x or z would mean the chain product
    # had crept in somewhere above it.
    for axis, name in ((0, "rest:tx"), (2, "rest:tz")):
        want = shoulder.GetAttribute(name).Get() or 0.0
        _Check(abs(origin[axis] - want) < 1e-9,
               "Shoulder %s is %.9f, authored %.9f" % (name, origin[axis],
                                                       want))


if __name__ == "__main__":
    _RegisterSchema()
    TestParentRestCarriesDescendants()
    TestRestMatchesAvarPropagation()
    TestTopLevelProviderUnchanged()
    print("test_rest_local: OK")
