"""
Headless test for plugin/rigExecUsdview/pickerHoverModel.py: the hover
picker's screen-space layout and the arithmetic its gestures apply.

  1. A tab maps panel points to the screen and back, with its buttons
     hanging below its handle at its scale.
  2. Fresh tabs line their handles up side by side, at a scale that fits
     the viewport, and a tab already placed keeps its place.
  3. Shift-drag scales about the handle (right/up grows), middle-drag
     fades, and both stay inside their limits.
  4. A tab's drawn opacity is its own times the overall one.
  5. The layout survives a save and a load, and a damaged save loads as
     an empty layout rather than raising.

Usage: test_picker_hover_model.py [ignored]
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import pickerHoverModel as hm  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, eps=1e-9):
    return all(abs(x - y) <= eps for x, y in zip(a, b))


def TestMapping():
    tab = hm.TabLayout("Biped/Body", 300.0, 40.0, scale=0.5)
    origin = (13.0, 20.0)
    corner = tab.ButtonsCorner()
    _Check(_Close(corner, (300.0 - hm.HANDLE_RADIUS,
                           40.0 + hm.HANDLE_RADIUS + hm.HANDLE_GAP)),
           "the buttons hang below the handle: %r" % (corner,))
    _Check(_Close(tab.ToScreen(origin, origin), corner),
           "the content origin lands on that corner")
    _Check(_Close(tab.ToScreen(origin, (113.0, 220.0)),
                  (corner[0] + 50.0, corner[1] + 100.0)),
           "panel distances shrink by the scale")
    for point in ((0.0, 0.0), (412.5, -7.25), (88.0, 640.0)):
        back = tab.ToPanel(origin, tab.ToScreen(origin, point))
        _Check(_Close(back, point, 1e-9), "round trip %r -> %r"
               % (point, back))
    _Check(tab.HitsHandle((300.0, 40.0)), "the handle's centre hits")
    _Check(tab.HitsHandle((300.0 + hm.HANDLE_RADIUS + 2.0, 40.0)),
           "a near miss still grabs it")
    _Check(not tab.HitsHandle((300.0, 40.0 + 3.0 * hm.HANDLE_RADIUS)),
           "the buttons below it do not")
    tab.Moved(10.0, -5.0)
    _Check(_Close((tab.x, tab.y), (310.0, 35.0)), "a move carries it")
    _Check(_Close(tab.ToScreen(origin, origin), tab.ButtonsCorner()),
           "and the buttons follow")


def TestFreshLayout():
    layout = hm.Layout()
    layout.Ensure(["A/Body", "A/Face"], {"A/Body": 0.5})
    body, face = layout.Tab("A/Body"), layout.Tab("A/Face")
    _Check(_Close((body.x, body.y), hm.FIRST_HANDLE),
           "the first handle starts at the top: %r" % ((body.x, body.y),))
    _Check(abs(face.y - body.y) < 1e-9 and
           abs(face.x - body.x - hm.HANDLE_SPACING) < 1e-9,
           "the next sits beside it")
    _Check(abs(body.scale - 0.5) < 1e-9 and abs(face.scale - 1.0) < 1e-9,
           "each starts at its fit scale")
    _Check(not body.collapsed and face.collapsed,
           "only the first opens, so the groups do not overlap")
    body.Moved(100.0, 200.0)
    layout.Ensure(["A/Body", "A/Face", "B/Body"])
    _Check(_Close((body.x, body.y), (hm.FIRST_HANDLE[0] + 100.0,
                                     hm.FIRST_HANDLE[1] + 200.0)),
           "a placed tab keeps its place")
    newcomer = layout.Tab("B/Body")
    _Check(newcomer.collapsed, "a tab added later waits as a handle")
    _Check(abs(newcomer.x - face.x - hm.HANDLE_SPACING) < 1e-9 and
           abs(newcomer.y - hm.FIRST_HANDLE[1]) < 1e-9,
           "a new tab joins the row after the last one in it")
    _Check(abs(hm.FitScale(1200.0, 800.0) - 0.3) < 1e-9,
           "a tall tab fits %.0f%% of the view" % (hm.FIT_FRACTION * 100))
    _Check(hm.FitScale(100.0, 800.0) == 1.0, "a small one is never grown")


def TestGestures():
    tab = hm.TabLayout("k", scale=1.0)
    tab.Scaled(1.0, 100.0, 0.0)
    grown = tab.scale
    _Check(grown > 1.0, "dragging right grows: %r" % grown)
    tab.Scaled(1.0, 0.0, -100.0)
    _Check(abs(tab.scale - grown) < 1e-12, "dragging up grows the same")
    tab.Scaled(1.0, -100.0, 0.0)
    _Check(abs(tab.scale * grown - 1.0) < 1e-12,
           "left undoes right by the same ratio")
    tab.Scaled(1.0, 1e6, 0.0)
    _Check(tab.scale == hm.MAX_SCALE, "scale stops at its maximum")
    tab.Scaled(1.0, -1e6, 0.0)
    _Check(tab.scale == hm.MIN_SCALE, "and at its minimum")
    tab.Faded(1.0, -100.0)
    _Check(abs(tab.opacity - 0.5) < 1e-12, "middle-drag left fades")
    tab.Faded(0.5, -1e6)
    _Check(tab.opacity == hm.MIN_OPACITY, "never to invisible")
    tab.Faded(0.5, 1e6)
    _Check(tab.opacity == hm.MAX_OPACITY, "and never past solid")


def TestOpacity():
    layout = hm.Layout()
    layout.Ensure(["a"])
    layout.Tab("a").opacity = 0.5
    layout.Faded(1.0, -100.0)
    _Check(abs(layout.opacity - 0.5) < 1e-12, "the knob fades everything")
    _Check(abs(layout.TabOpacity("a") - 0.25) < 1e-12,
           "a tab draws at its own times the overall")
    cx, cy = hm.KnobCentre(1000.0, 600.0)
    _Check(cx < 1000.0 and cy < 600.0 - hm.KNOB_HUD_CLEARANCE,
           "the knob sits inside the corner, above the HUD text")
    _Check(hm.HitsKnob(1000.0, 600.0, (cx, cy)), "the knob hits")
    _Check(not hm.HitsKnob(1000.0, 600.0, (cx - 40.0, cy)),
           "and only the knob")


def TestPersistence():
    layout = hm.Layout()
    layout.enabled = True
    layout.Ensure(["A/Body", "A/Face"])
    layout.Tab("A/Face").collapsed = True
    layout.Tab("A/Body").Scaled(1.0, 50.0, 0.0)
    layout.Faded(1.0, -40.0)
    loaded = hm.Layout.FromJson(layout.ToJson())
    _Check(loaded.enabled and abs(loaded.opacity - layout.opacity) < 1e-12,
           "the switch and the overall opacity come back")
    for key in ("A/Body", "A/Face"):
        _Check(loaded.Tab(key).ToDict() == layout.Tab(key).ToDict(),
               "%s comes back as it was" % key)
    for junk in ("", "not json", "[1, 2]", '{"tabs": 7}'):
        empty = hm.Layout.FromJson(junk)
        _Check(not empty.tabs and not empty.enabled,
               "a damaged save loads empty: %r" % junk)


def main():
    groups = [
        ("mapping", TestMapping),
        ("fresh layout", TestFreshLayout),
        ("gestures", TestGestures),
        ("opacity", TestOpacity),
        ("persistence", TestPersistence),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("PICKER_HOVER_MODEL_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
