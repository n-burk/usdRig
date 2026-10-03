#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/avarEditorModel.py: channel
discovery by prefix (schema channels AND custom ones), the kinds and
units, the fallback values, slider ranges, the two write modes, the
animated-channel promotion that keeps a file's keys when the session
layer edits one frame, reset semantics, and the undo bracket.

Everything here runs on an in-memory stage -- no Qt, no usdview.

Usage: test_avar_editor_model.py [schema resource dir]
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Usd, UsdGeom  # noqa: E402

import avarEditorModel as model  # noqa: E402
import rigExecUndo  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Stage():
    """
    A centimetre stage with one control carrying a custom ikfk dial that
    is keyed IN THE FILE, and a session layer on top as usdview has.
    """
    fileLayer = Sdf.Layer.CreateAnonymous("rig.usda")
    fileLayer.ImportFromString('''#usda 1.0
(
    metersPerUnit = 0.01
    startTimeCode = 1
    endTimeCode = 48
)
def RigExecRoot "Rig"
{
    def Scope "Controls"
    {
        def RigExecControl "Root"
        {
            custom float avars:ikfk
            float avars:ikfk.timeSamples = {
                1: 0,
                24: 0,
                48: 1,
            }
            custom bool avars:mirror = false
            custom int avars:fingerCount = 2
            custom matrix4d avars:oddMatrix = ( (1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1) )
        }
        def RigExecControl "Static"
        {
            custom float avars:ikfk = 0
        }
    }
}
''')
    stage = Usd.Stage.Open(fileLayer)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    return stage


def _Names(channels):
    return [c.name for c in channels]


def TestDiscovery(stage):
    prim = stage.GetPrimAtPath("/Rig/Controls/Root")
    _Check(model.HasAvars(prim), "the control has avars")
    channels, hidden = model.DiscoverChannels(prim, stage)
    names = _Names(channels)
    # Schema channels first, in canonical order, then the custom ones.
    _Check(names[:3] == ["avars:tx", "avars:ty", "avars:tz"], names)
    _Check(names[3:7] == ["avars:rx", "avars:ry", "avars:rz", "avars:rspin"],
           names)
    _Check(names[7:10] == ["avars:sx", "avars:sy", "avars:sz"], names)
    customs = [c for c in channels if c.kind == model.KIND_CUSTOM]
    _Check(_Names(customs) == ["avars:fingerCount", "avars:ikfk",
                               "avars:mirror"],
           "custom avars are discovered by prefix, alphabetically: %s"
           % _Names(customs))
    others = [c for c in channels if c.kind == model.KIND_OTHER]
    _Check(_Names(others) == ["avars:rotationOrder",
                              "avars:unitScaleFactor"], _Names(others))
    _Check(hidden == ["avars:defaultSpace", "avars:oddMatrix"],
           "matrix avars are reported, not edited: %s" % hidden)
    # Kinds, units, families.
    by = {c.name: c for c in channels}
    _Check(by["avars:tx"].kind == model.KIND_TRANSLATE
           and by["avars:tx"].unit == "cm", by["avars:tx"].unit)
    _Check(by["avars:rz"].unit == "deg" and by["avars:rspin"].kind ==
           model.KIND_ROTATE, "rotations in degrees")
    _Check(by["avars:sx"].kind == model.KIND_SCALE
           and by["avars:sx"].fallback == 1.0, "scale rests at 1")
    _Check(by["avars:ikfk"].family == model.VALUE_FLOAT
           and by["avars:ikfk"].custom, "ikfk is a custom float")
    _Check(by["avars:mirror"].family == model.VALUE_BOOL, "bool family")
    _Check(by["avars:fingerCount"].family == model.VALUE_INT
           and by["avars:fingerCount"].fallback == 2,
           "a custom int falls back to its authored default")
    _Check(by["avars:rotationOrder"].family == model.VALUE_TOKEN
           and by["avars:rotationOrder"].allowedTokens ==
           ["XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"],
           by["avars:rotationOrder"].allowedTokens)
    # Slider ranges.
    _Check(by["avars:tx"].SliderRange(stage) == (-100.0, 100.0),
           "one metre of centimetres either way")
    _Check(by["avars:rz"].SliderRange(stage) == (-180.0, 180.0), "degrees")
    _Check(by["avars:sx"].SliderRange(stage) == (0.0, 2.0), "around 1")
    _Check(by["avars:ikfk"].SliderRange(stage) == (0.0, 1.0),
           "a bare float avar is a 0..1 dial")
    _Check(by["avars:ikfk"].SliderRange(stage, 1.5) == (0.0, 1.5),
           "the range widens to hold the value")
    _Check(by["avars:rotationOrder"].SliderRange(stage) is None,
           "no slider for a token")
    # An attribute that states its own range wins over the per-kind guess:
    # a foot roll dial in degrees is not a 0..1 weight.
    # A throwaway dial, removed again so later tests see the fixture as it
    # was built.
    dialAttr = prim.CreateAttribute("avars:limitsProbe",
                                    Sdf.ValueTypeNames.Float)
    dial = model.Channel(prim, dialAttr, stage)
    _Check(dial.SliderRange(stage) == (0.0, 1.0),
           "a bare float with no limits is a 0..1 dial")
    dialAttr.SetCustomDataByKey("limits", {
        "soft": {"minimum": -120.0, "maximum": 120.0},
        "hard": {"minimum": -180.0, "maximum": 180.0}})
    _Check(dial.SliderRange(stage) == (-120.0, 120.0),
           "authored soft limits set the slider: %s"
           % (dial.SliderRange(stage),))
    dialAttr.SetCustomDataByKey("limits", {
        "hard": {"minimum": -90.0, "maximum": 90.0}})
    _Check(dial.SliderRange(stage) == (-90.0, 90.0),
           "hard limits stand in when there are no soft ones")
    _Check(dial.SliderRange(stage, 95.0) == (-90.0, 95.0),
           "and a value past them still widens the slider")
    prim.RemoveProperty("avars:limitsProbe")
    # A unit scale factor changes the translate label.
    prim.GetAttribute("avars:unitScaleFactor").Set(2.5)
    rebuilt, _ = model.DiscoverChannels(prim, stage)
    _Check({c.name: c for c in rebuilt}["avars:tx"].unit == "x2.5 cm",
           {c.name: c for c in rebuilt}["avars:tx"].unit)
    prim.GetAttribute("avars:unitScaleFactor").Clear()
    _Check(not model.HasAvars(stage.GetPrimAtPath("/Rig/Controls")),
           "a scope has no avars")
    _Check(model.UnitLabel(stage) == "cm", model.UnitLabel(stage))


def TestWriteAndReset(stage):
    prim = stage.GetPrimAtPath("/Rig/Controls/Static")
    channels, _ = model.DiscoverChannels(prim, stage)
    by = {c.name: c for c in channels}
    session = stage.GetSessionLayer()
    frame = Usd.TimeCode(5)
    rz = by["avars:rz"]

    # Animation mode on a static channel: a spline knot at the frame.
    _Check(model.WriteValue(rz, 30, frame, model.WRITE_ANIMATION) is None,
           "no warning")
    spec = session.GetAttributeAtPath("/Rig/Controls/Static.avars:rz")
    _Check(spec is not None and spec.HasSpline(), "a spline was authored")
    _Check(abs(rz.attr.Get(frame) - 30.0) < 1e-9, rz.attr.Get(frame))
    _Check(rz.Animated(), "the channel now reads as animated")
    # Reset on an animated channel in Animation mode keys the fallback.
    _Check(model.ResetValue(rz, frame, model.WRITE_ANIMATION) is None,
           "no warning")
    _Check(abs(rz.attr.Get(frame)) < 1e-9, "the rest value is keyed")
    _Check(spec.HasSpline(), "the spline itself survives")
    # Default mode on it warns that the spline outranks the default.
    warning = model.WriteValue(rz, 45, frame, model.WRITE_DEFAULT)
    _Check(warning and "outranked" in warning, warning)
    _Check(abs(rz.attr.Get(frame)) < 1e-9,
           "and the frame still reads the key: %s" % rz.attr.Get(frame))
    # Reset in Default mode clears the default; it says the keys still win.
    warning = model.ResetValue(rz, frame, model.WRITE_DEFAULT)
    _Check(warning and "outrank" in warning, warning)
    _Check(not spec.HasDefaultValue(), "the default opinion is gone")

    # A static channel, Default mode, then reset: Clear() at the target.
    tx = by["avars:tx"]
    _Check(model.WriteValue(tx, 12.5, frame, model.WRITE_DEFAULT) is None,
           "no warning on an unkeyed channel")
    _Check(abs(tx.attr.Get(frame) - 12.5) < 1e-9, tx.attr.Get(frame))
    _Check(model.ResetValue(tx, frame, model.WRITE_DEFAULT) is None, "ok")
    _Check(session.GetAttributeAtPath("/Rig/Controls/Static.avars:tx") is None
           or not session.GetAttributeAtPath(
               "/Rig/Controls/Static.avars:tx").HasDefaultValue(),
           "reset cleared the session opinion")
    _Check(abs(tx.attr.Get(frame)) < 1e-9, "back to the schema rest")

    # Scale is floored the way the evaluator floors it.
    sx = by["avars:sx"]
    model.WriteValue(sx, 0.0, frame, model.WRITE_DEFAULT)
    _Check(abs(sx.attr.Get(frame) - 1e-4) < 1e-12,
           "a zero scale is written as the evaluator's floor: %s"
           % sx.attr.Get(frame))
    model.ResetValue(sx, frame, model.WRITE_DEFAULT)

    # Non-numeric channels are always defaults, whatever the mode.
    order = by["avars:rotationOrder"]
    model.WriteValue(order, "ZYX", frame, model.WRITE_ANIMATION)
    spec = session.GetAttributeAtPath(
        "/Rig/Controls/Static.avars:rotationOrder")
    _Check(spec.HasDefaultValue() and spec.default == "ZYX"
           and not session.ListTimeSamplesForPath(spec.path),
           "the rotation order is a default, not a key")

    # A custom float: the authored default 0 is its rest.
    ikfk = by["avars:ikfk"]
    _Check(ikfk.fallback == 0.0, ikfk.fallback)
    model.WriteValue(ikfk, 1, frame, model.WRITE_ANIMATION)
    _Check(abs(ikfk.attr.Get(frame) - 1.0) < 1e-9, "keyed to 1")
    model.ResetValue(ikfk, frame, model.WRITE_ANIMATION)
    _Check(abs(ikfk.attr.Get(frame)) < 1e-9, "keyed back to 0")

    # A bad value is refused before anything is written.
    raised = False
    try:
        model.WriteValue(tx, "not a number", frame, model.WRITE_DEFAULT)
    except ValueError:
        raised = True
    _Check(raised, "a malformed value raises ValueError")


def TestPromotion(stage):
    """A file-keyed channel edited at one frame keeps its other keys."""
    prim = stage.GetPrimAtPath("/Rig/Controls/Root")
    channels, _ = model.DiscoverChannels(prim, stage)
    ikfk = {c.name: c for c in channels}["avars:ikfk"]
    session = stage.GetSessionLayer()
    path = Sdf.Path("/Rig/Controls/Root.avars:ikfk")
    _Check(ikfk.fallback == 0.0,
           "a custom float with keys and no default rests at 0: %s"
           % ikfk.fallback)
    _Check(abs(ikfk.attr.Get(Usd.TimeCode(36)) - 0.5) < 1e-9, "fixture")

    recorder = rigExecUndo.EditRecorder(stage, [path])
    recorder.Begin()
    _Check(model.WriteValue(ikfk, 0.25, Usd.TimeCode(36),
                            model.WRITE_ANIMATION) is None, "no warning")
    edit = recorder.Commit("key")
    values = [ikfk.attr.Get(Usd.TimeCode(t)) for t in (1, 24, 36, 48)]
    _Check(abs(values[2] - 0.25) < 1e-9, values)
    _Check(values[0] == 0.0 and values[1] == 0.0 and values[3] == 1.0,
           "the file's keys survived in the composed result: %s" % values)
    _Check(sorted(session.ListTimeSamplesForPath(path)) ==
           [1.0, 24.0, 36.0, 48.0],
           "the session layer holds the promoted keys and the new one")
    # A second edit does not re-promote (the target owns the keys now).
    model.WriteValue(ikfk, 0.75, Usd.TimeCode(36), model.WRITE_ANIMATION)
    _Check(abs(ikfk.attr.Get(Usd.TimeCode(36)) - 0.75) < 1e-9, "re-keyed")
    _Check(len(session.ListTimeSamplesForPath(path)) == 4, "still four")
    # Reset at 36 keys the fallback there and leaves the rest alone.
    model.ResetValue(ikfk, Usd.TimeCode(36), model.WRITE_ANIMATION)
    _Check(abs(ikfk.attr.Get(Usd.TimeCode(36))) < 1e-9, "reset keyed 0")
    _Check(ikfk.attr.Get(Usd.TimeCode(48)) == 1.0, "48 untouched")
    # Undo removes the whole session spec, promotion included.
    _Check(edit is not None, "the edit recorded something")
    edit.Undo()
    _Check(session.GetAttributeAtPath(path) is None,
           "undo removed the session spec entirely")
    _Check(abs(ikfk.attr.Get(Usd.TimeCode(36)) - 0.5) < 1e-9,
           "and the file's curve reads through again")

    # Default mode on the file-keyed channel never stomps SILENTLY. It
    # does stomp: a session default is a stronger opinion than the
    # file's time samples, so the whole shot now reads 0.9 -- and the
    # warning says exactly that rather than claiming the keys won.
    warning = model.WriteValue(ikfk, 0.9, Usd.TimeCode(36),
                               model.WRITE_DEFAULT)
    _Check(warning and "HIDES" in warning, warning)
    # 1e-6, not 1e-9: ikfk is a 32-bit float and 0.9 is not exact in it.
    _Check(abs(ikfk.attr.Get(Usd.TimeCode(36)) - 0.9) < 1e-6
           and abs(ikfk.attr.Get(Usd.TimeCode(48)) - 0.9) < 1e-6,
           "the session default hides the file's keys at every frame: %s"
           % [ikfk.attr.Get(Usd.TimeCode(t)) for t in (36, 48)])
    session.GetPrimAtPath("/Rig/Controls/Root").RemoveProperty(
        session.GetAttributeAtPath(path))
    _Check(abs(ikfk.attr.Get(Usd.TimeCode(36)) - 0.5) < 1e-9,
           "removing it lets the file's curve through again")


def TestEditScope(stage):
    prim = stage.GetPrimAtPath("/Rig/Controls/Static")
    channels, _ = model.DiscoverChannels(prim, stage)
    by = {c.name: c for c in channels}
    stack = rigExecUndo.UndoStack()
    frame = Usd.TimeCode(7)
    with model.EditScope(stage, [by["avars:ty"]], stack, "Drag ty") as scope:
        for value in (1.0, 2.0, 3.0):
            model.WriteValue(by["avars:ty"], value, frame,
                             model.WRITE_ANIMATION)
    _Check(scope.edit is not None and stack.CanUndo()
           and stack.UndoText() == "Drag ty",
           "three samples became one undo entry")
    _Check(abs(by["avars:ty"].attr.Get(frame) - 3.0) < 1e-9, "last wins")
    stack.Undo()
    _Check(abs(by["avars:ty"].attr.Get(frame)) < 1e-9, "undone as one")
    stack.Redo()
    _Check(abs(by["avars:ty"].attr.Get(frame) - 3.0) < 1e-9, "redone")
    # A scope that writes nothing pushes nothing.
    depthBefore = len(stack._undo)
    with model.EditScope(stage, [by["avars:tz"]], stack, "Nothing"):
        pass
    _Check(len(stack._undo) == depthBefore, "no empty entries")
    # An exception inside the scope rolls the layer back.
    try:
        with model.EditScope(stage, [by["avars:tz"]], stack, "Boom"):
            model.WriteValue(by["avars:tz"], 9.0, frame,
                             model.WRITE_ANIMATION)
            raise RuntimeError("boom")
    except RuntimeError:
        pass
    _Check(abs(by["avars:tz"].attr.Get(frame)) < 1e-9,
           "an aborted scope restored the channel")


def main():
    _RegisterSchema()
    stage = _Stage()
    _Check(stage.GetPrimAtPath("/Rig/Controls/Root").GetAttribute(
        "avars:tx"), "the RigExec schema is registered (pass its resource "
        "directory as argv[1])")
    TestDiscovery(stage)
    TestWriteAndReset(stage)
    TestPromotion(stage)
    TestEditScope(stage)
    print("test_avar_editor_model OK")


if __name__ == "__main__":
    main()
