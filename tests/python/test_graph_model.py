#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/graphModel.py.

Usage: test_graph_model.py [<generated schema resources dir>]

The discovery groups need the RigExec schema registered (an unanimated
`avars:tx` is only discoverable because the schema declares it), so this
script reads argv[1] the way test_gizmo_math.py does.
"""
import sys

# Sibling module: this script's own directory is sys.path[0]. It must run
# before the pxr import so pxr resolves from the configured USD install.
import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Ts, Usd, UsdGeom  # noqa: E402

import graphModel as gm  # noqa: E402
import rigExecUndo  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

# A gentle zig-zag: every knot is a local extremum, so the AutoEase
# slopes are all 0 and the Catmull-Rom ("spline") slopes are not. The two
# tangent modes are therefore distinguishable by their slope alone.
CURVE_POINTS = ((0.0, 0.0), (10.0, 10.0), (20.0, 5.0), (30.0, 8.0))


def _Spline(points=CURVE_POINTS, typeName="double"):
    spline = Ts.Spline(typeName)
    for time, value in points:
        knot = Ts.Knot(typeName=typeName, time=time, value=value,
                       nextInterp=Ts.InterpCurve)
        knot.SetPreTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        knot.SetPostTanAlgorithm(Ts.TangentAlgorithmAutoEase)
        spline.SetKnot(knot)
    return spline


def _Times(spline):
    return [knot.GetTime() for knot in spline.GetKnots().values()]


def _Curve(spline, lo=-5.0, hi=35.0, step=0.25):
    """The evaluated curve, for "the shape did not change" assertions."""
    samples = []
    t = lo
    while t <= hi:
        samples.append(spline.Eval(t))
        t += step
    return samples


def _RigStage():
    """A rig root with one control, plus a plain mesh outside the rig."""
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Asset")
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    control = stage.DefinePrim("/Asset/Rig/HandIK", "RigExecControl")
    mesh = UsdGeom.Mesh.Define(stage, "/Asset/Geo").GetPrim()
    return stage, control, mesh


# ---------------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------------

def TestDiscoverFromProperties():
    stage, control, _ = _RigStage()
    tx = control.GetAttribute("avars:tx").GetPath()
    space = control.GetAttribute("rest:space").GetPath()
    order = control.GetAttribute("avars:rotationOrder").GetPath()

    refs = gm.DiscoverCurves(stage, [tx], [control.GetPath()])
    _Check(len(refs) == 1, "one selected avar -> one curve: %s" % (refs,))
    _Check(refs[0].primPath == control.GetPath()
           and refs[0].attrName == "avars:tx", "curve identifies the avar")
    _Check(refs[0].attrPath == tx,
           "attrPath round trips: %s" % refs[0].attrPath)
    _Check(refs[0].Label() == "HandIK.avars:tx",
           "label is prim.attribute: %r" % refs[0].Label())
    _Check(refs[0].color == gm.CURVE_RED, "tx is red: %s" % (refs[0].color,))

    # A matrix4d is not a spline-capable type, and a token is uniform:
    # both are dropped, the scalar next to them still qualifies.
    refs = gm.DiscoverCurves(stage, [space, tx, order], [control.GetPath()])
    _Check([r.attrName for r in refs] == ["avars:tx"],
           "vector and uniform properties are ignored: %s"
           % [r.attrName for r in refs])

    # NOTHING in the selection qualifies -> fall back to the prims.
    refs = gm.DiscoverCurves(stage, [space, order], [control.GetPath()])
    _Check(len(refs) > 1 and any(r.attrName == "avars:ty" for r in refs),
           "a selection with no curve falls back to the prim: %s"
           % [r.attrName for r in refs])


def TestDiscoverFromPrims():
    stage, control, mesh = _RigStage()
    # An authored spline on a non-rig attribute, and one on an avar.
    custom = control.CreateAttribute("wobble", Sdf.ValueTypeNames.Double)
    custom.SetSpline(_Spline())
    control.GetAttribute("avars:rz").SetSpline(_Spline())

    refs = gm.DiscoverCurves(stage, [], [control.GetPath()])
    names = [r.attrName for r in refs]
    _Check(len(names) == len(set(names)), "no duplicates: %s" % names)
    # Usd.Prim.GetAttributes() is alphabetical, so avars:rz precedes wobble.
    _Check(names[0] == "avars:rz" and names[1] == "wobble",
           "splined attributes come first, in prim order: %s" % names)
    _Check(names[2:] == [n for n in gm.RIG_CHANNELS if n != "avars:rz"],
           "then the unanimated rig channels in channel order: %s" % names)

    colors = dict((r.attrName, r.color) for r in refs)
    _Check(colors["avars:tx"] == gm.CURVE_RED
           and colors["avars:rx"] == gm.CURVE_RED
           and colors["avars:sx"] == gm.CURVE_RED, "x channels are red")
    _Check(colors["avars:ty"] == gm.CURVE_GREEN
           and colors["rest:ry"] == gm.CURVE_GREEN, "y channels are green")
    _Check(colors["avars:tz"] == gm.CURVE_BLUE, "z channels are blue")
    _Check(colors["avars:rspin"] == gm.CURVE_YELLOW, "rspin is yellow")
    _Check(colors["wobble"] in gm.CURVE_PALETTE,
           "an unrecognised channel takes a palette colour: %s"
           % (colors["wobble"],))

    # A prim that is not rig-xformable contributes only its splines.
    hum = mesh.CreateAttribute("hum", Sdf.ValueTypeNames.Float)
    hum.SetSpline(_Spline(typeName="float"))
    mesh.CreateAttribute("silent", Sdf.ValueTypeNames.Double).Set(1.0)
    refs = gm.DiscoverCurves(stage, [], [mesh.GetPath()])
    _Check([r.attrName for r in refs] == ["hum"],
           "a plain prim lists only its splined attributes: %s"
           % [r.attrName for r in refs])

    # Two prims: no cross-prim deduplication of the same attribute name.
    refs = gm.DiscoverCurves(
        stage, [], [control.GetPath(), mesh.GetPath()])
    _Check(len([r for r in refs if r.attrName == "hum"]) == 1
           and len(refs) == len(gm.RIG_CHANNELS) + 2,
           "both prims contribute: %s" % [r.Label() for r in refs])


# ---------------------------------------------------------------------------
# Knot authoring
# ---------------------------------------------------------------------------

def TestAuthorKnot():
    spline = Ts.Spline("double")
    knot = gm.AuthorKnot(spline, 5.0, 2.0)
    _Check(len(spline.GetKnots()) == 1, "the knot landed on the spline")
    stored = spline.GetKnot(5.0)
    _Check(_Close(stored.GetValue(), 2.0), "value")
    _Check(stored.GetNextInterpolation() == Ts.InterpCurve, "curve segment")
    _Check(stored.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase
           and stored.GetPostTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "Maya's default new key is AutoEase on both sides")
    _Check(knot.GetTime() == 5.0, "the authored knot is returned")

    gm.AuthorKnot(spline, 5.0, 7.0)
    _Check(len(spline.GetKnots()) == 1 and _Close(spline.Eval(5.0), 7.0),
           "re-authoring the same frame updates the knot in place")

    # An unanimated attribute has to be keyable: Usd hands back an
    # UNTYPED spline for it, and Ts.Knot(typeName="") raises.
    # `stage` must stay bound: dropping it expires every prim on it.
    stage, control, _ = _RigStage()
    attr = control.GetAttribute("avars:ty")
    _Check(attr.GetSpline().GetValueTypeName() == "",
           "an unanimated attribute's spline is untyped")
    fresh = gm.SplineFor(attr)
    _Check(fresh.GetValueTypeName() == "double", "SplineFor types it")
    gm.AuthorKnot(fresh, 3.0, 1.5)
    attr.SetSpline(fresh)
    _Check(_Close(attr.Get(Usd.TimeCode(3.0)), 1.5),
           "the first key on an unanimated channel lands")
    _Check(gm.SplineFor(attr).GetValueTypeName() == "double",
           "an existing spline is returned as it is")

    # An existing knot keeps the tangents the artist gave it.
    spline = _Spline()
    gm.SetTangent(spline, 10.0, gm.SIDE_BOTH, 4.0, width=2.0)
    gm.AuthorKnot(spline, 10.0, 99.0)
    stored = spline.GetKnot(10.0)
    _Check(_Close(stored.GetValue(), 99.0), "value updated")
    _Check(stored.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom
           and _Close(stored.GetPostTanSlope(), 4.0),
           "an existing knot keeps its tangents")


# ---------------------------------------------------------------------------
# Edit operations
# ---------------------------------------------------------------------------

def TestMoveKeys():
    spline = _Spline()
    moved = gm.MoveKeys(spline, [10.0], 3.4, 1.0)
    _Check(moved == [13.0], "the moved time snaps to a whole frame: %s"
           % (moved,))
    _Check(_Times(spline) == [0.0, 13.0, 20.0, 30.0],
           "the old time is gone: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(13.0).GetValue(), 11.0),
           "the value moved by dv")

    spline = _Spline()
    moved = gm.MoveKeys(spline, [10.0], 0.4, 0.0, snapFrames=False)
    _Check(moved == [10.4], "unsnapped moves keep the fraction: %s" % (moved,))

    # Clamped: a key never crosses a neighbour, and stops one frame short
    # of it while snapping so the two frames stay distinct.
    spline = _Spline()
    _Check(gm.MoveKeys(spline, [10.0], 50.0, 0.0) == [19.0],
           "clamped below the next key: %s" % _Times(spline))
    spline = _Spline()
    _Check(gm.MoveKeys(spline, [10.0], -50.0, 0.0) == [1.0],
           "clamped above the previous key: %s" % _Times(spline))
    spline = _Spline()
    moved = gm.MoveKeys(spline, [10.0], 50.0, 0.0, snapFrames=False)
    _Check(len(moved) == 1 and 0.0 < moved[0] < 20.0,
           "unsnapped clamping stays inside the open interval: %s" % (moved,))

    # Two keys move as a block and stay apart from each other.
    spline = _Spline(((0.0, 0.0), (5.0, 1.0), (6.0, 2.0), (20.0, 3.0)))
    moved = gm.MoveKeys(spline, [5.0, 6.0], 50.0, 0.0)
    _Check(moved == [18.0, 19.0],
           "the block clamps without collapsing: %s" % (moved,))
    _Check(_Times(spline) == [0.0, 18.0, 19.0, 20.0],
           "and the knots landed there: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(18.0).GetValue(), 1.0)
           and _Close(spline.GetKnot(19.0).GetValue(), 2.0),
           "values travel with their keys")

    # Two keys dragged onto each other's frames: the block is rebuilt as
    # a whole, so neither is written over the other on the way past.
    spline = _Spline(((0.0, 0.0), (5.0, 1.0), (6.0, 2.0), (20.0, 3.0)))
    moved = gm.MoveKeys(spline, [5.0, 6.0], 1.0, 0.0)
    _Check(moved == [6.0, 7.0], "the block moves through itself: %s"
           % (moved,))
    _Check(_Times(spline) == [0.0, 6.0, 7.0, 20.0],
           "all four keys survive: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(6.0).GetValue(), 1.0)
           and _Close(spline.GetKnot(7.0).GetValue(), 2.0),
           "and stay in order, values with them")

    # The same going backwards, which takes the other placement branch.
    spline = _Spline(((0.0, 0.0), (5.0, 1.0), (6.0, 2.0), (20.0, 3.0)))
    moved = gm.MoveKeys(spline, [5.0, 6.0], -1.0, 0.0)
    _Check(moved == [4.0, 5.0], "backwards through itself: %s" % (moved,))
    _Check(_Times(spline) == [0.0, 4.0, 5.0, 20.0],
           "all four keys survive backwards: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(4.0).GetValue(), 1.0)
           and _Close(spline.GetKnot(5.0).GetValue(), 2.0),
           "values travel backwards too")

    # SUB-FRAME keys: snapping cannot find a free whole frame between the
    # neighbours, so the key does not move at all. A file authored
    # elsewhere, or this editor with snapping off, contains such keys, and
    # clamping one onto a neighbour would silently destroy that neighbour.
    spline = _Spline(((0.0, 0.0), (0.5, 1.0), (1.0, 2.0)))
    moved = gm.MoveKeys(spline, [0.5], 5.0, 0.25)
    _Check(moved == [0.5], "no free frame: the key keeps its time: %s"
           % (moved,))
    _Check(_Times(spline) == [0.0, 0.5, 1.0],
           "and no key is overwritten: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(0.5).GetValue(), 1.25),
           "the value delta still applies: %s"
           % spline.GetKnot(0.5).GetValue())
    _Check(_Close(spline.GetKnot(0.0).GetValue(), 0.0)
           and _Close(spline.GetKnot(1.0).GetValue(), 2.0),
           "the neighbours keep their own values")

    # Two sub-frame keys with no whole frame free between the fixed
    # neighbours at all: neither moves, and neither eats the other.
    spline = _Spline(((0.0, 0.0), (0.4, 1.0), (0.6, 2.0), (1.0, 3.0)))
    moved = gm.MoveKeys(spline, [0.4, 0.6], 5.0, 0.0)
    _Check(moved == [0.4, 0.6], "both crowded keys stay: %s" % (moved,))
    _Check(_Times(spline) == [0.0, 0.4, 0.6, 1.0],
           "and nothing is lost: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(0.4).GetValue(), 1.0)
           and _Close(spline.GetKnot(0.6).GetValue(), 2.0),
           "with their own values")

    # A key with no room becomes an obstacle for the rest of the block,
    # so a crowded pair does not half-move: here only frame 1 is free
    # between the neighbours, and one whole frame of clearance from the
    # key that has to stay puts it out of reach. Neither moves.
    spline = _Spline(((0.0, 0.0), (0.5, 1.0), (0.6, 2.0), (2.0, 3.0)))
    moved = gm.MoveKeys(spline, [0.5, 0.6], 5.0, 0.0)
    _Check(moved == [0.5, 0.6],
           "a pinned key blocks its neighbours too: %s" % (moved,))
    _Check(_Times(spline) == [0.0, 0.5, 0.6, 2.0],
           "four keys still: %s" % _Times(spline))
    _Check(_Close(spline.GetKnot(0.0).GetValue(), 0.0)
           and _Close(spline.GetKnot(2.0).GetValue(), 3.0),
           "the fixed neighbours are untouched")

    # With snapping OFF the same drag has room and does move.
    spline = _Spline(((0.0, 0.0), (0.5, 1.0), (1.0, 2.0)))
    moved = gm.MoveKeys(spline, [0.5], 5.0, 0.0, snapFrames=False)
    _Check(len(moved) == 1 and 0.5 < moved[0] < 1.0,
           "unsnapped, the sub-frame key still slides: %s" % (moved,))
    _Check(len(spline.GetKnots()) == 3, "and all three keys remain")

    # A time that is not a key is ignored rather than fatal.
    spline = _Spline()
    _Check(gm.MoveKeys(spline, [7.5], 1.0, 0.0) == [],
           "moving a non-key is a no-op")
    _Check(_Times(spline) == [0.0, 10.0, 20.0, 30.0], "spline untouched")


def TestInsertKey():
    spline = _Spline()
    expected = spline.Eval(5.0)
    knot = gm.InsertKey(spline, 5.0)
    _Check(knot is not None and _Close(knot.GetValue(), expected),
           "the inserted key takes the evaluated value")
    _Check(len(spline.GetKnots()) == 5, "one more knot")
    stored = spline.GetKnot(5.0)
    _Check(stored.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase
           and stored.GetPostTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "inserted keys get Auto tangents")

    before = len(spline.GetKnots())
    again = gm.InsertKey(spline, 5.0)
    _Check(len(spline.GetKnots()) == before
           and _Close(again.GetValue(), expected),
           "inserting on an existing key returns it unchanged")

    _Check(gm.InsertKey(Ts.Spline("double"), 1.0) is None,
           "an empty spline has no value to insert")


def TestDeleteKeys():
    spline = _Spline()
    removed = gm.DeleteKeys(spline, [10.0, 99.0, 30.0])
    _Check(removed == [10.0, 30.0],
           "only the keys that existed are removed: %s" % (removed,))
    _Check(_Times(spline) == [0.0, 20.0], "the rest survive: %s"
           % _Times(spline))
    _Check(gm.DeleteKeys(spline, [10.0]) == [],
           "deleting a missing key is a no-op, not an error")


def TestTangentTypes():
    flat = _Spline()
    gm.SetTangentType(flat, [10.0], gm.TANGENT_FLAT)
    knot = flat.GetKnot(10.0)
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmCustom
           and knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "flat authors both slopes")
    _Check(_Close(knot.GetPreTanSlope(), 0.0)
           and _Close(knot.GetPostTanSlope(), 0.0), "flat slope is zero")
    _Check(_Close(flat.EvalDerivative(10.0), 0.0), "the curve is flat there")

    # On a MONOTONE curve AutoEase is not flat, so the two modes are
    # visibly different between the keys. CURVE_POINTS would hide it:
    # every key there is a local extremum, where AutoEase is flat by
    # definition (ts/types.h:156-162).
    rising = _Spline(((0.0, 0.0), (10.0, 5.0), (20.0, 30.0)))
    autoMid = rising.Eval(5.0)
    gm.SetTangentType(rising, [10.0], gm.TANGENT_FLAT)
    _Check(not _Close(rising.Eval(5.0), autoMid, 1e-6),
           "flat and auto evaluate differently between keys: %s vs %s"
           % (rising.Eval(5.0), autoMid))

    # Catmull-Rom through the neighbours: (8 - 10) / (30 - 10) = -0.1.
    curve = _Spline()
    gm.SetTangentType(curve, [20.0], gm.TANGENT_SPLINE)
    knot = curve.GetKnot(20.0)
    _Check(_Close(knot.GetPreTanSlope(), -0.1)
           and _Close(knot.GetPostTanSlope(), -0.1),
           "spline slope is Catmull-Rom: %s" % knot.GetPostTanSlope())
    _Check(knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "spline slope is authored, not automatic")

    # Auto puts it back.
    gm.SetTangentType(curve, [20.0], gm.TANGENT_AUTO)
    knot = curve.GetKnot(20.0)
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase
           and knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "auto restores the automatic algorithm")

    # Linear: the out side changes the segment AFTER the key.
    linear = _Spline()
    gm.SetTangentType(linear, [10.0], gm.TANGENT_LINEAR, side=gm.SIDE_OUT)
    _Check(linear.GetKnot(10.0).GetNextInterpolation() == Ts.InterpLinear,
           "linear out sets the following segment")
    _Check(_Close(linear.Eval(15.0), 7.5), "the segment is a straight line")
    _Check(linear.GetKnot(0.0).GetNextInterpolation() == Ts.InterpCurve,
           "the preceding segment is untouched")
    gm.SetTangentType(linear, [10.0], gm.TANGENT_LINEAR, side=gm.SIDE_IN)
    _Check(linear.GetKnot(0.0).GetNextInterpolation() == Ts.InterpLinear,
           "linear in sets the preceding segment")
    _Check(_Close(linear.Eval(5.0), 5.0), "and it is a straight line too")

    # Step holds the segment after the key; a curve mode restores it.
    step = _Spline()
    gm.SetTangentType(step, [10.0], gm.TANGENT_STEP)
    _Check(step.GetKnot(10.0).GetNextInterpolation() == Ts.InterpHeld,
           "step holds the following segment")
    _Check(_Close(step.Eval(15.0), 10.0), "held value")
    gm.SetTangentType(step, [10.0], gm.TANGENT_FLAT)
    _Check(step.GetKnot(10.0).GetNextInterpolation() == Ts.InterpCurve,
           "a curve mode restores InterpCurve on the affected segment")

    # A one-sided request leaves the other side alone.
    sided = _Spline()
    gm.SetTangentType(sided, [10.0], gm.TANGENT_FLAT, side=gm.SIDE_OUT)
    knot = sided.GetKnot(10.0)
    _Check(knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom
           and knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "side='out' touches only the post tangent")

    # Edge knots have no neighbour on one side; spline must still work.
    edge = _Spline()
    gm.SetTangentType(edge, [0.0, 30.0], gm.TANGENT_SPLINE)
    _Check(_Close(edge.GetKnot(0.0).GetPostTanSlope(), 1.0),
           "the first key falls back to its one-sided slope: %s"
           % edge.GetKnot(0.0).GetPostTanSlope())
    _Check(_Close(edge.GetKnot(30.0).GetPreTanSlope(), 0.3),
           "and so does the last: %s" % edge.GetKnot(30.0).GetPreTanSlope())

    try:
        gm.SetTangentType(_Spline(), [10.0], "plateau")
    except ValueError:
        pass
    else:
        _Check(False, "an unknown tangent mode must raise")


def TestSetTangent():
    spline = _Spline()
    width = spline.GetKnot(10.0).GetPostTanWidth()
    gm.SetTangent(spline, 10.0, gm.SIDE_OUT, 2.0)
    knot = spline.GetKnot(10.0)
    _Check(knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom
           and _Close(knot.GetPostTanSlope(), 2.0), "the out slope is set")
    _Check(_Close(knot.GetPostTanWidth(), width),
           "width=None keeps the width: %s vs %s"
           % (knot.GetPostTanWidth(), width))
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmAutoEase,
           "the in side is untouched")

    gm.SetTangent(spline, 10.0, gm.SIDE_OUT, 2.0, width=5.0)
    _Check(_Close(spline.GetKnot(10.0).GetPostTanWidth(), 5.0),
           "an explicit width is applied")

    gm.SetTangent(spline, 10.0, gm.SIDE_IN, -1.0, width=1.0)
    knot = spline.GetKnot(10.0)
    _Check(_Close(knot.GetPreTanSlope(), -1.0)
           and _Close(knot.GetPreTanWidth(), 1.0), "the in side is set")
    _Check(_Close(knot.GetPostTanSlope(), 2.0), "the out side is kept")

    _Check(gm.SetTangent(spline, 7.5, gm.SIDE_OUT, 1.0) is None,
           "setting a tangent on a non-key is a no-op")


def TestBreakAndUnify():
    spline = _Spline()
    _Check(gm.IsUnified(spline.GetKnot(10.0)),
           "automatic tangents are unified")

    before = _Curve(spline)
    gm.BreakTangents(spline, [10.0])
    knot = spline.GetKnot(10.0)
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmCustom
           and knot.GetPostTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "breaking authors both sides")
    _Check(_Curve(spline) == before,
           "breaking does not change the curve")
    _Check(not gm.IsUnified(knot), "a broken knot reports broken")

    gm.SetTangent(spline, 10.0, gm.SIDE_OUT, 3.0)
    knot = spline.GetKnot(10.0)
    _Check(not _Close(knot.GetPreTanSlope(), 3.0),
           "the sides move independently once broken")

    gm.UnifyTangents(spline, [10.0])
    knot = spline.GetKnot(10.0)
    _Check(_Close(knot.GetPreTanSlope(), 3.0)
           and _Close(knot.GetPostTanSlope(), 3.0),
           "unify copies the out slope onto the in slope")
    _Check(knot.GetPreTanAlgorithm() == Ts.TangentAlgorithmCustom,
           "unified tangents stay authored")
    _Check(gm.IsUnified(knot), "and the knot reports unified again")

    # A knot whose two sides disagree is broken even without the marker.
    spline = _Spline()
    gm.SetTangent(spline, 20.0, gm.SIDE_OUT, 1.0)
    _Check(not gm.IsUnified(spline.GetKnot(20.0)),
           "one authored side and one automatic side is broken")


def TestExtrapolation():
    spline = _Spline()
    gm.SetExtrapolation(spline, pre="cycle", post="cycle_offset")
    _Check(spline.GetPreExtrapolation().mode == Ts.ExtrapLoopReset,
           "Maya Cycle repeats the curve exactly -> LoopReset")
    _Check(spline.GetPostExtrapolation().mode == Ts.ExtrapLoopRepeat,
           "Maya Cycle with Offset joins the ends -> LoopRepeat")

    gm.SetExtrapolation(spline, pre="constant")
    _Check(spline.GetPreExtrapolation().mode == Ts.ExtrapHeld, "constant")
    _Check(spline.GetPostExtrapolation().mode == Ts.ExtrapLoopRepeat,
           "post=None leaves the other side alone")
    gm.SetExtrapolation(spline, post="linear")
    _Check(spline.GetPostExtrapolation().mode == Ts.ExtrapLinear, "linear")
    gm.SetExtrapolation(spline, post="oscillate")
    _Check(spline.GetPostExtrapolation().mode == Ts.ExtrapLoopOscillate,
           "oscillate")
    _Check(gm.EXTRAP_NAMES[Ts.ExtrapLoopReset] == "cycle",
           "the mapping inverts for the combo boxes")

    try:
        gm.SetExtrapolation(spline, pre="bounce")
    except ValueError:
        pass
    else:
        _Check(False, "an unknown infinity name must raise")


def TestSnapAndNeighbours():
    _Check(gm.SnapTime(1.4) == 1.0 and gm.SnapTime(1.6) == 2.0, "rounding")
    _Check(gm.SnapTime(1.5) == 2.0 and gm.SnapTime(2.5) == 3.0,
           "halves go away from zero, not to even")
    _Check(gm.SnapTime(-1.5) == -2.0 and gm.SnapTime(-1.4) == -1.0,
           "and symmetrically below zero")

    spline = _Spline()
    _Check(gm.KeyNeighbours(spline, 10.0) == (0.0, 20.0), "both neighbours")
    _Check(gm.KeyNeighbours(spline, 0.0) == (None, 10.0), "no previous")
    _Check(gm.KeyNeighbours(spline, 30.0) == (20.0, None), "no next")
    _Check(gm.KeyNeighbours(spline, 15.0) == (10.0, 20.0),
           "a time between keys straddles them")
    _Check(gm.KeyNeighbours(Ts.Spline("double"), 0.0) == (None, None),
           "an empty spline has no neighbours")


# ---------------------------------------------------------------------------
# Writing
# ---------------------------------------------------------------------------

def _SessionStage():
    stage, control, _ = _RigStage()
    attr = control.GetAttribute("avars:tx")
    attr.SetSpline(_Spline())
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    return stage, attr


def TestApplySpline():
    stage, attr = _SessionStage()
    session = stage.GetSessionLayer()
    original = Ts.Spline(attr.GetSpline())
    stack = rigExecUndo.UndoStack()

    edited = Ts.Spline(attr.GetSpline())
    gm.InsertKey(edited, 5.0)
    _Check(gm.ApplySpline(stage, attr.GetPath(), edited, stack, "Insert Key"),
           "the edit was pushed")
    _Check(len(attr.GetSpline().GetKnots()) == 5, "the stage sees the key")
    _Check(session.GetAttributeAtPath(attr.GetPath()) is not None,
           "the write landed in the edit target (session) layer")
    _Check(stack.CanUndo() and stack.UndoText() == "Insert Key",
           "one labelled undo entry: %r" % stack.UndoText())

    _Check(stack.Undo(), "undo runs")
    _Check(attr.GetSpline() == original,
           "undo restores the previous spline exactly")
    _Check(session.GetAttributeAtPath(attr.GetPath()) is None,
           "and removes the session spec it created")

    _Check(stack.Redo() and len(attr.GetSpline().GetKnots()) == 5,
           "redo puts the key back")

    # Writing the spline the attribute already has changes nothing.
    stack = rigExecUndo.UndoStack()
    same = Ts.Spline(attr.GetSpline())
    _Check(not gm.ApplySpline(stage, attr.GetPath(), same, stack, "No-op"),
           "an unchanged spline reports no edit")
    _Check(not stack.CanUndo(), "and pushes nothing")

    _Check(not gm.ApplySpline(stage, "/Asset/Rig/HandIK.nope", _Spline(),
                              stack, "Missing"),
           "an unresolvable attribute is refused, not fatal")


def main():
    _RegisterSchema()
    groups = [
        ("discovery from properties", TestDiscoverFromProperties),
        ("discovery from prims", TestDiscoverFromPrims),
        ("author knot", TestAuthorKnot),
        ("move keys", TestMoveKeys),
        ("insert key", TestInsertKey),
        ("delete keys", TestDeleteKeys),
        ("tangent types", TestTangentTypes),
        ("set tangent", TestSetTangent),
        ("break + unify", TestBreakAndUnify),
        ("extrapolation", TestExtrapolation),
        ("snap + neighbours", TestSnapAndNeighbours),
        ("apply spline", TestApplySpline),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GRAPH_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
