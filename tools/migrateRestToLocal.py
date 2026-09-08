#!/usr/bin/env python
"""
Rebake absolute rest transforms into parent-relative ones.

Joint rest offsets used to be absolute in asset space: computeRestFrame
declared no namespace ancestor among its inputs, so a chain authored its
joints' world bind positions directly and a parent's rest edit moved the
parent alone. Rest is now relative to the parent frame provider's rest
frame, which means every authored rest in an existing asset has to be
divided by its parent's.

For each frame provider with a frame-provider ancestor, parents first:

    R_local = R_absolute * R_parent_absolute^-1

R is the provider's whole composed rest -- compose(rest:t, rest:r) *
rest:space -- so one routine covers assets authoring rest:space matrices
and assets authoring rest:t/rest:r scalars alike. The result is written
back into whichever spelling the prim already uses.

The compose and decompose come from the usdview gizmo's gizmoMath, which
mirrors _ComposeAvars and is covered by testGizmoMath. Reimplementing
them here would be a second thing to keep in step with the evaluator.

Usage: migrateRestToLocal.py <stage.usd> [<stage.usd> ...]
"""
import os
import sys

from pxr import Gf, Sdf, Usd

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "plugin", "rigExecUsdview"))
import gizmoMath  # noqa: E402

RIG_VERSION_ATTR = "rigExec:restFrameVersion"
RIG_VERSION = 2

PROVIDER_TYPES = ("RigExecJoint", "RigExecControl")
_REST_T = ("rest:tx", "rest:ty", "rest:tz")
_REST_R = ("rest:rx", "rest:ry", "rest:rz")
_REST_SPACE = "rest:space"
_IDENTITY = Gf.Matrix4d(1.0)


def IsProvider(prim):
    return bool(prim) and prim.GetTypeName() in PROVIDER_TYPES


def ParentProvider(prim):
    """The nearest frame-provider ancestor, or None at the top of a chain."""
    parent = prim.GetParent()
    while parent and parent.IsValid():
        if IsProvider(parent):
            return parent
        parent = parent.GetParent()
    return None


def _Scalar(prim, name):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get()
        if value is not None:
            return float(value)
    return 0.0


def _MatrixAttr(prim, name):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get()
        if value is not None:
            return Gf.Matrix4d(value)
    return Gf.Matrix4d(1.0)


def _Scalars(prim):
    """compose(rest:t, rest:r) -- XYZ, no scale, no spin."""
    return gizmoMath.ComposeAvarMatrix(
        _Scalar(prim, _REST_T[0]), _Scalar(prim, _REST_T[1]),
        _Scalar(prim, _REST_T[2]), 1.0, 1.0, 1.0,
        _Scalar(prim, _REST_R[0]), _Scalar(prim, _REST_R[1]),
        _Scalar(prim, _REST_R[2]), 0.0, "XYZ")


def ComposedRest(prim):
    """
    The prim's own rest factor: orthonormalize(scalars * rest:space).

    Read off an UNMIGRATED stage this is the provider's absolute rest.
    Read off a migrated one it is the local factor.
    """
    return (_Scalars(prim) * _MatrixAttr(prim, _REST_SPACE))\
        .GetOrthonormalized(False)


def AbsoluteWorldRests(stage):
    """
    World rest frames under the OLD absolute semantics.

    The migration's correctness check needs the pre-change answer and the
    evaluator no longer produces it. Under the old rule a provider's world
    rest simply was its own composed rest, so this is that rule.
    """
    return {str(prim.GetPath()): ComposedRest(prim)
            for prim in stage.Traverse() if IsProvider(prim)}


def LocalWorldRests(stage):
    """World rest frames under the NEW parent-relative semantics."""
    frames = {}

    def resolve(prim):
        path = str(prim.GetPath())
        if path not in frames:
            parent = ParentProvider(prim)
            frames[path] = (ComposedRest(prim) * resolve(parent)) if parent \
                else ComposedRest(prim)
        return frames[path]

    for prim in stage.Traverse():
        if IsProvider(prim):
            resolve(prim)
    return frames


def _RigRoots(stage):
    return [p for p in stage.Traverse() if p.GetTypeName() == "RigExecRoot"]


def IsMigrated(stage):
    for root in _RigRoots(stage):
        attr = root.GetAttribute(RIG_VERSION_ATTR)
        if attr and (attr.Get() or 0) >= RIG_VERSION:
            return True
    return False


def _Stamp(stage):
    for root in _RigRoots(stage):
        attr = root.GetAttribute(RIG_VERSION_ATTR) or root.CreateAttribute(
            RIG_VERSION_ATTR, Sdf.ValueTypeNames.Int, custom=True)
        attr.Set(RIG_VERSION)


def _WriteRest(prim, local):
    """
    Write `local` back into the spelling the prim already uses.

    A prim carrying rest:space keeps carrying its transform there, with
    its rest:t/r scalars untouched, so the two never disagree. A prim
    authoring only scalars gets scalars back, so a hand-written rest:tx
    stays readable as a number instead of turning into a matrix.
    """
    space = prim.GetAttribute(_REST_SPACE)
    if space and space.HasAuthoredValue():
        space.Set(_Scalars(prim).GetInverse() * local)
        return

    translation = local.ExtractTranslation()
    rotation = gizmoMath.DecomposeEuler(local, "XYZ")
    for name, value in zip(_REST_T, translation):
        attr = prim.GetAttribute(name) or prim.CreateAttribute(
            name, Sdf.ValueTypeNames.Double)
        attr.Set(float(value))
    for name, value in zip(_REST_R, rotation):
        attr = prim.GetAttribute(name) or prim.CreateAttribute(
            name, Sdf.ValueTypeNames.Double)
        attr.Set(float(value))


def MigrateStage(stage):
    """Rewrite authored rests parent-relative. Returns providers rewritten."""
    if IsMigrated(stage):
        return 0

    providers = [p for p in stage.Traverse() if IsProvider(p)]
    # Parents before children: a child divides by its parent's ABSOLUTE
    # rest, so every parent's pre-migration value must be read before any
    # of them is overwritten. Snapshot first, then write.
    absolute = {str(p.GetPath()): ComposedRest(p) for p in providers}

    rewritten = 0
    for prim in providers:
        parent = ParentProvider(prim)
        if not parent:
            continue
        _WriteRest(prim, absolute[str(prim.GetPath())]
                   * absolute[str(parent.GetPath())].GetInverse())
        rewritten += 1

    _Stamp(stage)
    return rewritten


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    for path in argv[1:]:
        stage = Usd.Stage.Open(path)
        if not stage:
            print("could not open %s" % path)
            return 2
        before = AbsoluteWorldRests(stage)
        count = MigrateStage(stage)
        if not count:
            print("%s: already migrated" % path)
            continue
        after = LocalWorldRests(stage)
        for provider, frame in after.items():
            moved = (Gf.Vec3d(frame.ExtractTranslation())
                     - Gf.Vec3d(before[provider].ExtractTranslation()))
            if moved.GetLength() > 1e-9:
                print("%s: REFUSED -- %s would move by %s"
                      % (path, provider, moved))
                return 1
        stage.GetRootLayer().Save()
        print("%s: rebased %d provider(s)" % (path, count))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
