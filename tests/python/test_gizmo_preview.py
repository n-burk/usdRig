#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoPreview.py: the manipulation
preview protocol, and the Writer contract underneath it.

A drag collects its values and hands them to Hydra; a release authors them
(docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md).
The sink is faked here, so all of it runs with no rigExecImaging, no display
and no usdview -- which is also the configuration a session without the
library is in, and it has to behave.

Usage: test_gizmo_preview.py
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Sdf, Usd, UsdGeom  # noqa: E402

import gizmoMath  # noqa: E402
import gizmoPreview  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


class _FakeSink(object):
    """Records the protocol instead of calling into rigExecImaging."""

    def __init__(self, expected=None, accept=True):
        # What Begin() reports it expects; None means "count the paths".
        self._expected = expected
        self._accept = accept
        self.declarations = []
        self.samples = []
        self.ends = 0

    def Begin(self, packedPaths):
        self.declarations.append(packedPaths)
        if self._expected is not None:
            return self._expected
        # One double per path is the common case in this test; a caller that
        # wants another arity passes `expected`.
        return len(packedPaths.split("\n"))

    def Update(self, values):
        self.samples.append(list(values))
        return self._accept

    def End(self):
        self.ends += 1
        return True


def _Stage():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Asset", "Scope")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    return stage


def TestFlatten():
    """Arity has to match what the C++ side derived from the value TYPE."""
    _Check(gizmoPreview.Flatten(2.5) == [2.5], "a scalar is one double")
    _Check(gizmoPreview.Flatten(3) == [3.0], "an int is one double")
    _Check(gizmoPreview.Flatten(Gf.Vec3d(1, 2, 3)) == [1.0, 2.0, 3.0],
           "a Vec3d is three")
    _Check(gizmoPreview.Flatten(Gf.Vec3f(1, 2, 3)) == [1.0, 2.0, 3.0],
           "a Vec3f is three")
    flat = gizmoPreview.Flatten(Gf.Matrix4d(1.0))
    _Check(len(flat) == 16 and flat[0] == 1.0 and flat[1] == 0.0,
           "a Matrix4d is sixteen, row major: %s" % flat)
    # Kinds the channel does not carry say so rather than being guessed at: a
    # misjudged arity would shift every slot after it.
    _Check(gizmoPreview.Flatten(True) is None, "a bool is not a value here")
    _Check(gizmoPreview.Flatten("nope") is None, "nor is a string")
    _Check(gizmoPreview.Flatten(Gf.Vec2d(1, 2)) is None, "nor a Vec2d")


def TestPushDeclaresOnceAndSendsNumbers():
    sink = _FakeSink()
    gizmoPreview.SetSink(sink)
    try:
        path = Sdf.Path("/Asset/Rig/Ctl.avars:tx")
        other = Sdf.Path("/Asset/Rig/Ctl.avars:ty")
        _Check(gizmoPreview.Push({path: 1.0, other: 2.0}),
               "the sink took the first sample")
        _Check(sink.declarations == ["/Asset/Rig/Ctl.avars:tx\n"
                                     "/Asset/Rig/Ctl.avars:ty"],
               "one declaration, newline separated: %s" % sink.declarations)
        _Check(sink.samples == [[1.0, 2.0]], "the values, flattened in order")

        # Every sample after it is numbers only. This is the point of the
        # declaration being separate: a drag is hundreds of these.
        for i in range(3):
            gizmoPreview.Push({path: float(i), other: 2.0})
        _Check(len(sink.declarations) == 1,
               "the same attributes are declared once per drag, got %d"
               % len(sink.declarations))
        _Check(len(sink.samples) == 4, "every sample reached the sink")

        # A key set that grows mid-drag -- a rotate that starts writing rspin,
        # or an op created by the first Apply -- re-declares, once.
        third = Sdf.Path("/Asset/Rig/Ctl.avars:rspin")
        gizmoPreview.Push({path: 9.0, other: 2.0, third: 0.5})
        gizmoPreview.Push({path: 9.5, other: 2.0, third: 0.5})
        _Check(len(sink.declarations) == 2,
               "the new attribute re-declared exactly once, got %d"
               % len(sink.declarations))
        _Check(sink.samples[-1] == [9.5, 2.0, 0.5],
               "and the wider sample carries all three: %s" % sink.samples[-1])
    finally:
        gizmoPreview.SetSink(None)


def TestEndStopsPreviewing():
    sink = _FakeSink()
    gizmoPreview.SetSink(sink)
    try:
        path = Sdf.Path("/Asset/Rig/Ctl.avars:tx")
        gizmoPreview.Push({path: 1.0})
        _Check(gizmoPreview.IsPreviewing(), "a pushed drag is previewing")
        _Check(gizmoPreview.End(), "End reached the sink")
        _Check(sink.ends == 1, "the sink was told once")
        _Check(not gizmoPreview.IsPreviewing(), "and the preview is over")
        _Check(gizmoMath.PreviewValues() == {},
               "the frame maths is reading the stage again")
        # Idempotent: an abort and a commit both arrive here, and neither knows
        # what the other did. A second End has nothing begun to drop, so it
        # does not reach the host at all -- the host's End republishes the
        # rig, and every selection change aborts a drag.
        _Check(gizmoPreview.End(), "End is safe to repeat")
        _Check(sink.ends == 1, "and a repeat does not call the host again")
        # The NEXT drag declares again rather than assuming the old slots.
        gizmoPreview.Push({path: 2.0})
        _Check(len(sink.declarations) == 2,
               "a second drag re-declares: %s" % sink.declarations)
    finally:
        gizmoPreview.SetSink(None)


def TestArityDisagreementIsDropped():
    """
    A sample the two sides would read differently is dropped WHOLE.

    The C++ side expects a fixed number of doubles in declaration order, so a
    value of the wrong shape does not corrupt its own slot -- it shifts every
    slot after it. Refusing the sample costs one frame of preview; sending it
    would draw the rig somewhere nobody asked for.
    """
    sink = _FakeSink(expected=7)
    gizmoPreview.SetSink(sink)
    try:
        path = Sdf.Path("/Asset/Rig/Ctl.avars:tx")
        _Check(not gizmoPreview.Push({path: 1.0}),
               "a sample the sink sized differently is refused")
        _Check(sink.samples == [], "and nothing was sent")
    finally:
        gizmoPreview.SetSink(None)

    # A value of a kind Flatten does not carry refuses the same way.
    sink = _FakeSink()
    gizmoPreview.SetSink(sink)
    try:
        _Check(not gizmoPreview.Push({Sdf.Path("/Asset/Rig/Ctl.foo"): "text"}),
               "an unflattenable value is refused")
        _Check(sink.samples == [], "and nothing was sent")
    finally:
        gizmoPreview.SetSink(None)


def TestNoSinkStillWorks():
    """
    A session with no rigExecImaging previews nothing and breaks nothing.

    The viewport does not update until the release, which is the honest
    behaviour for a host that cannot be told about the preview; the values are
    still collected, and the release still authors them.
    """
    gizmoPreview.SetSink(None)
    stage = _Stage()
    prim = stage.DefinePrim("/Asset/Rig/Ctl", "RigExecControl")
    writer = gizmoMath.Writer(stage, Usd.TimeCode.Default(),
                              gizmoMath.WRITE_DEFAULT)
    attr = prim.GetAttribute("avars:tx")
    writer.Set(attr, 4.0)
    _Check(not gizmoPreview.Push(writer.Pending()),
           "no sink, nothing pushed")
    # ... and the value is still the one every read sees, because the Writer
    # publishes it for the frame maths itself.
    _Check(gizmoMath.ScalarAvar(prim, "avars:tx", Usd.TimeCode.Default(),
                                0.0) == 4.0,
           "the uncommitted value is what the gizmo reads")
    _Check(attr.Get() != 4.0, "while the stage still holds the authored one")
    gizmoPreview.End()
    writer.CommitToStage()
    _Check(attr.Get() == 4.0, "and the release authors it")


def TestWriterFeedsTheFrameMaths():
    """
    The uncommitted value is what the frame maths reads -- the property that
    keeps the handles on the geometry they are moving.
    """
    stage = _Stage()
    prim = stage.DefinePrim("/Asset/Rig/Ctl", "RigExecControl")
    time = Usd.TimeCode.Default()
    prim.GetAttribute("avars:tx").Set(1.0)
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    _Check(gizmoMath.ScalarAvar(prim, "avars:tx", time, 0.0) == 1.0,
           "the authored value, before anything is collected")
    writer.Set(prim.GetAttribute("avars:tx"), 5.0)
    _Check(gizmoMath.ScalarAvar(prim, "avars:tx", time, 0.0) == 5.0,
           "the collected value wins while it is uncommitted")
    _Check(prim.GetAttribute("avars:tx").Get(time) == 1.0,
           "and the stage is untouched")
    # An abandoned drag: the authored value is back with nothing to undo.
    writer.Clear()
    _Check(gizmoMath.ScalarAvar(prim, "avars:tx", time, 0.0) == 1.0,
           "a cleared drag leaves the authored value reading through")
    # A committed one: the stage holds it, and the preview no longer shadows
    # it -- if it did, a later edit to the same attribute would be invisible.
    writer.Set(prim.GetAttribute("avars:tx"), 6.0)
    writer.CommitToStage()
    _Check(gizmoMath.PreviewValues() == {},
           "the commit stopped previewing what it authored")
    _Check(prim.GetAttribute("avars:tx").Get(time) == 6.0, "and it landed")
    prim.GetAttribute("avars:tx").Set(7.0)
    _Check(gizmoMath.ScalarAvar(prim, "avars:tx", time, 0.0) == 7.0,
           "a later authored edit is not shadowed by a stale preview")


def TestXformOpsPreviewThroughTheVectors():
    """
    A plain Xformable's handles follow an uncommitted op value too.

    XformCommonAPI reads the stage, and a preview is not on the stage, so
    _XformTarget substitutes the collected op values into the vectors it
    reports. Without that the gizmo would snap back to the pre-drag transform
    on every sample while the geometry moved.
    """
    stage = Usd.Stage.CreateInMemory()
    time = Usd.TimeCode.Default()
    xform = UsdGeom.Xform.Define(stage, "/Box")
    api = UsdGeom.XformCommonAPI(xform)
    api.SetTranslate(Gf.Vec3d(1, 0, 0))
    api.SetRotate(Gf.Vec3f(0, 0, 0))
    api.SetScale(Gf.Vec3f(1, 1, 1))
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, xform.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None, reason)
    target.BeginDrag()
    start = target.GizmoMatrix().ExtractTranslation()

    target.ApplyTranslate(Gf.Vec3d(0, 3, 0))
    target.Refresh()
    moved = target.GizmoMatrix().ExtractTranslation()
    _Check((moved - start - Gf.Vec3d(0, 3, 0)).GetLength() < 1e-9,
           "the handles followed the uncommitted drag: %s -> %s"
           % (start, moved))
    _Check(api.GetXformVectors(time)[0] == Gf.Vec3d(1, 0, 0),
           "while the stage still holds the authored translate")

    writer.CommitToStage()
    target.Refresh()
    _Check(api.GetXformVectors(time)[0] == Gf.Vec3d(1, 3, 0),
           "and the release authored it: %s" % (api.GetXformVectors(time)[0],))
    _Check((target.GizmoMatrix().ExtractTranslation() - moved).GetLength()
           < 1e-9, "with the handles where they already were")


def TestGroupPushesOneSamplePerMove():
    """
    A GROUP drag is still ONE declaration and ONE Update per mouse
    sample, however many prims it moves.

    This is the whole reason the controller builds every member's target
    on a single Writer: Push() sends writer.Pending(), so N targets that
    shared one writer arrive as one packed sample, while N writers would
    have been N round trips to the C++ side per mouse-move -- and then N
    authored commits on release, where one authored edit already costs
    ~150 ms on the next evaluate.
    """
    stage = Usd.Stage.CreateInMemory()
    time = Usd.TimeCode.Default()
    UsdGeom.Xform.Define(stage, "/Group")
    prims = []
    for index, name in enumerate(("A", "B")):
        xform = UsdGeom.Xform.Define(stage, "/Group/%s" % name)
        api = UsdGeom.XformCommonAPI(xform)
        api.SetTranslate(Gf.Vec3d(index * 4, 0, 0))
        api.SetRotate(Gf.Vec3f(0, 0, 30.0 * index))
        api.SetScale(Gf.Vec3f(1, 1, 1))
        prims.append(xform.GetPrim())
    # Two prims x one Vec3d translate op = six doubles in one sample.
    sink = _FakeSink(expected=6)
    gizmoPreview.SetSink(sink)
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    group, reason = gizmoMath.MakeGroupTarget(
        stage, prims, gizmoMath.CHANNELS_POSE, writer)
    _Check(group is not None, reason)
    group.BeginDrag()
    group.ApplyTranslate(Gf.Vec3d(0, 2, 0))
    _Check(gizmoPreview.Push(writer.Pending()),
           "the sink took the group sample")
    _Check(len(sink.declarations) == 1,
           "one declaration for the whole group: %s" % sink.declarations)
    _Check(sink.declarations[0].split("\n")
           == ["/Group/A.xformOp:translate", "/Group/B.xformOp:translate"],
           "both members in it: %r" % sink.declarations[0])
    _Check(len(sink.samples) == 1 and len(sink.samples[0]) == 6,
           "one packed sample: %s" % sink.samples)
    # A second mouse-move re-declares nothing: the key set is unchanged.
    group.ApplyTranslate(Gf.Vec3d(0, 5, 0))
    _Check(gizmoPreview.Push(writer.Pending()), "second sample")
    _Check(len(sink.declarations) == 1 and len(sink.samples) == 2,
           "still one declaration, two samples: %s / %s"
           % (sink.declarations, sink.samples))
    # Nothing reached the stage: the release is what authors.
    for prim in prims:
        _Check(not stage.GetAttributeAtPath(
            prim.GetPath().AppendProperty("xformOp:translate"))
            .Get(time)[1],
            "%s is still where it was authored" % prim.GetName())
    writer.CommitToStage()
    gizmoPreview.End()
    for prim in prims:
        value = stage.GetAttributeAtPath(
            prim.GetPath().AppendProperty("xformOp:translate")).Get(time)
        _Check(abs(value[1] - 5.0) < 1e-9,
               "%s authored once, on release: %s" % (prim.GetName(), value))

def main():
    for name, fn in (("flatten", TestFlatten),
                     ("declare once", TestPushDeclaresOnceAndSendsNumbers),
                     ("end", TestEndStopsPreviewing),
                     ("arity", TestArityDisagreementIsDropped),
                     ("no sink", TestNoSinkStillWorks),
                     ("writer feeds the maths",
                      TestWriterFeedsTheFrameMaths),
                     ("xform ops", TestXformOpsPreviewThroughTheVectors),
                     ("group: one sample per move",
                      TestGroupPushesOneSamplePerMove)):
        try:
            fn()
        finally:
            # Module-level preview state must not leak between cases; a stale
            # entry would shadow the next test's stage reads.
            gizmoPreview.SetSink(None)
            gizmoMath.SetPreviewValues({})
        print("  ok: %s" % name)
    print("GIZMO_PREVIEW_OK (%d groups)" % 8)
    return 0


if __name__ == "__main__":
    sys.exit(main())
