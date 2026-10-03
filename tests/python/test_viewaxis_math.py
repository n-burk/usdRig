#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/viewAxisMath.py: where the axis
balls land for a camera basis, the back-to-front draw order, the hit test
(including the switch to the opposite ball when looking down an axis),
the depth-faded styling, and the camera direction a click asks for.

Usage: test_viewaxis_math.py [ignored]
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf  # noqa: E402

import viewAxisMath as va  # noqa: E402
import viewCubeMath as vm  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


CENTRE = (44.0, 44.0)
REACH = va.RADIUS * (1.0 - va.HANDLE_SIZE)


def _ByName(handles):
    return dict((h.name, h) for h in handles)


def TestFrontView():
    # Looking down -Z with Y up: X points right, Y up, Z at the viewer.
    handles = va.Handles(vm.IDENTITY_BASIS, CENTRE)
    names = _ByName(handles)
    _Check(len(handles) == 6, "six balls")
    x = names["+x"]
    _Check(abs(x.screen[0] - (CENTRE[0] + REACH)) < 1e-9 and
           abs(x.screen[1] - CENTRE[1]) < 1e-9,
           "+X sits right of centre: %s" % (x.screen,))
    y = names["+y"]
    _Check(abs(y.screen[1] - (CENTRE[1] - REACH)) < 1e-9,
           "+Y sits above centre (screen y is down): %s" % (y.screen,))
    _Check(handles[-1].name == "+z" and handles[0].name == "-z",
           "+Z faces the viewer and draws last, -Z first: %s"
           % [h.name for h in handles])
    _Check(va.AlignedAxis(handles) == "z", "the view looks down Z")


def TestHitTest():
    basis = vm.IDENTITY_BASIS
    hit = va.HitTest(basis, CENTRE, names_point(basis, "+x"))
    _Check(isinstance(hit, va.Handle) and hit.name == "+x",
           "a click on +X hits +X: %s" % hit)
    _Check(va.HitTest(basis, CENTRE, (CENTRE[0] + 200.0, CENTRE[1]))
           is None, "outside the circle is not ours")
    # Looking down -Z the +Z ball sits on the centre, facing the viewer:
    # a click there takes the opposite, -Z.
    hit = va.HitTest(basis, CENTRE, CENTRE)
    _Check(isinstance(hit, va.Handle) and hit.name == "-z",
           "the centre ball looking down an axis is its opposite: %s"
           % hit)


def names_point(basis, name):
    return _ByName(va.Handles(basis, CENTRE))[name].screen


def TestTiltedView():
    basis = vm.BasisForAngles(35.0, 25.0, False)
    handles = va.Handles(basis, CENTRE)
    _Check(va.AlignedAxis(handles) is None, "a tilted view aligns none")
    depths = [h.depth for h in handles]
    _Check(depths == sorted(depths), "drawn back to front")
    for name in ("+x", "-x", "+y", "-y", "+z", "-z"):
        hit = va.HitTest(basis, CENTRE, names_point(basis, name))
        _Check(isinstance(hit, va.Handle) and hit.name == name,
               "%s is hittable from a tilted view, got %s" % (name, hit))
    # Screen positions agree with the world axes the camera sees.
    names = _ByName(handles)
    for axis, world in (("x", Gf.Vec3d(1, 0, 0)), ("y", Gf.Vec3d(0, 1, 0)),
                        ("z", Gf.Vec3d(0, 0, 1))):
        view = basis.ToView(world)
        h = names["+" + axis]
        _Check(abs(h.screen[0] - (CENTRE[0] + view[0] * REACH)) < 1e-9 and
               abs(h.screen[1] - (CENTRE[1] - view[1] * REACH)) < 1e-9,
               "+%s projects from the world axis" % axis)


def TestStyle():
    background = (0.3, 0.3, 0.3, 1.0)
    handles = _ByName(va.Handles(vm.BasisForAngles(35.0, 25.0, False),
                                 CENTRE))
    front, back = handles["+x"], handles["-x"]
    fs = va.HandleStyle(front, None, background)
    bs = va.HandleStyle(back, None, background)
    _Check(fs["drawLine"] and fs["drawLabel"] and not bs["drawLine"] and
           not bs["drawLabel"], "positive axes have a line and a label")
    _Check(fs["ballRadius"] != bs["ballRadius"],
           "balls grow toward the viewer")
    _Check(bs["inner"] != bs["outline"], "negative balls are rings")
    aligned = _ByName(va.Handles(vm.IDENTITY_BASIS, CENTRE))
    _Check(not va.HandleStyle(aligned["-z"], "z", background)["drawBall"],
           "looking down -Z, the -Z ball behind the centre is hidden")


def TestLookDirection():
    handles = _ByName(va.Handles(vm.IDENTITY_BASIS, CENTRE))
    for name, want in (("+x", (1, 0, 0)), ("-y", (0, -1, 0)),
                       ("+z", (0, 0, 1))):
        got = va.ViewDirectionForHandle(handles[name])
        _Check(got == Gf.Vec3d(*want),
               "clicking %s puts the camera on %s: %s" % (name, want, got))
    # And the angles that asks for really look back down the axis, in
    # both up modes.
    for isZUp in (False, True):
        for name in ("+x", "-x", "+y", "-y", "+z", "-z"):
            world = va.ViewDirectionForHandle(handles[name])
            up = vm.WorldToUpSpace(isZUp).TransformDir(world)
            theta, phi = vm.AnglesForDirection(up, 0.0)
            view = vm.BasisForAngles(theta, phi, isZUp).view
            _Check((view + world).GetLength() < 1e-9,
                   "%s (zUp %s) looks down %s: %s"
                   % (name, isZUp, -world, view))


def main():
    TestFrontView()
    TestHitTest()
    TestTiltedView()
    TestStyle()
    TestLookDirection()
    print("test_viewaxis_math: ALL PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
