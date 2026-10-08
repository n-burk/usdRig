#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/markingMenuModel.py: which slot
each item takes and what overflows, which compass direction a cursor
offset means (boundaries and the +/-180 wrap included), the gesture
state machine driven by an injected clock -- flick, tap, hold, submenu,
cancel -- and the resolution of a declared menu against a context.

The clock is injected, so the timing rules are checked AT their
boundaries: 149 ms shows nothing and 150 ms shows the menu, in the same
microsecond of real time.

Usage: test_marking_menu_model.py [ignored]
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

import markingMenuDefs as defs  # noqa: E402
import markingMenuModel as mm  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


CENTRE = (300.0, 200.0)


class _Clock(object):
    """A clock a test sets by hand. `clock.t` is the millisecond now."""

    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


class _Ctx(object):
    """A stand-in for the selection summary the plugin will pass.

    Implements the whole documented interface; other tests below leave
    methods out on purpose, to prove the model tolerates a context that
    answers only what it cares about.
    """

    def __init__(self, flags=(), state=None, children=None, labels=None):
        self.flags = set(flags)
        self.state = dict(state or {})
        self.children = dict(children or {})
        self.labels = dict(labels or {})
        self.asked = []

    def Test(self, name):
        self.asked.append(name)
        return name in self.flags

    def Read(self, name):
        return self.state.get(name)

    def Label(self, itemId, default):
        return self.labels.get(itemId, default)

    def Children(self, name):
        return self.children.get(name, ())


def _Items(count, **common):
    """`count` plain action items named a0, a1, ... in declaration order."""
    return [dict(id="a%d" % i, label="A%d" % i, **common)
            for i in range(count)]


def _Kinds(resolved):
    return dict((d, item.id) for d, item in resolved.slots.items())


def _One(emissions, kind, where):
    """The single emission of that kind, insisting there is exactly one."""
    matches = [e for e in emissions if e.kind == kind]
    _Check(len(matches) == 1,
           "%s: expected one %s, got %s" % (where, kind, emissions))
    return matches[0]


def _Kind(emissions, where):
    _Check(len(emissions) == 1,
           "%s: expected a single emission, got %s" % (where, emissions))
    return emissions[0].kind


# --- Layout ------------------------------------------------------------


def TestFillOrder():
    resolved = mm.Resolve(mm.Menu("m", _Items(8)), _Ctx())
    _Check(not resolved.overflow, "eight items all fit the ring")
    for index, direction in enumerate(mm.FILL_ORDER):
        _Check(resolved.Slot(direction).id == "a%d" % index,
               "item %d takes %s, got %s"
               % (index, direction, resolved.Slot(direction)))
    # The ring reads back in fill order too, which is the order the
    # widget walks to draw it.
    _Check([r.id for r in resolved.items] == ["a%d" % i for i in range(8)],
           "the ring reads back in fill order: %s" % (resolved.items,))


def TestPinnedSlot():
    # a2 pins N, which is the fourth slot the fill would have reached.
    items = _Items(4)
    items[2]["dir"] = "N"
    resolved = mm.Resolve(mm.Menu("m", items), _Ctx())
    _Check(resolved.Slot("N").id == "a2", "the pin is honoured")
    _Check(resolved.Slot("W").id == "a0" and resolved.Slot("E").id == "a1",
           "the fill is unchanged before the pin: %s" % (_Kinds(resolved),))
    # a3 is the fourth unpinned item and would have taken N; the fill
    # skips the taken slot and gives it NW, the next one in FILL_ORDER.
    _Check(resolved.Slot("S").id == "a3",
           "the fill skips the pinned slot: %s" % (_Kinds(resolved),))
    _Check(resolved.Slot("NW") is None, "nothing invents a fifth item")

    # A pin declared last still beats a fill: a0..a7 would have taken
    # every slot, and the pinned one claims SE before the fill starts.
    items = _Items(8)
    items[7]["dir"] = "W"
    resolved = mm.Resolve(mm.Menu("m", items), _Ctx())
    _Check(resolved.Slot("W").id == "a7", "a late pin wins its slot")
    _Check(resolved.Slot("E").id == "a0",
           "the fill starts at the first FREE slot: %s" % (_Kinds(resolved),))

    # Two items pinned to the same slot: the first keeps it, the second
    # loses the pin rather than deleting anyone.
    items = _Items(3)
    items[0]["dir"] = "S"
    items[1]["dir"] = "S"
    resolved = mm.Resolve(mm.Menu("m", items), _Ctx())
    _Check(resolved.Slot("S").id == "a0", "the first pin to S keeps it")
    _Check(resolved.Slot("W").id == "a1" and resolved.Slot("E").id == "a2",
           "the loser joins the fill, in declaration order: %s"
           % (_Kinds(resolved),))


def TestOverflow():
    resolved = mm.Resolve(mm.Menu("m", _Items(10)), _Ctx())
    _Check(len(resolved.slots) == 8, "the ring takes exactly eight")
    _Check([r.id for r in resolved.overflow] == ["a8", "a9"],
           "the ninth and tenth overflow, in order: %s"
           % (resolved.overflow,))
    _Check(all(r.direction is None for r in resolved.overflow),
           "an overflow row has no direction to flick at")
    _Check(resolved.Find("a9") is not None,
           "an overflow row is still findable by id, which is how the "
           "widget confirms one")

    # An item may also declare itself into the column, and then it stays
    # there however much room the ring has.
    items = _Items(3)
    items[1]["overflow"] = True
    resolved = mm.Resolve(mm.Menu("m", items), _Ctx())
    _Check([r.id for r in resolved.overflow] == ["a1"],
           "a declared overflow row is never promoted into the ring")
    _Check(resolved.Slot("W").id == "a0" and resolved.Slot("E").id == "a2",
           "and the ring fills around it: %s" % (_Kinds(resolved),))


# --- Sectors -----------------------------------------------------------


def TestSectorCentres():
    # Straight along each compass direction, at the ring radius. y is
    # DOWN, so N is negative dy.
    for direction in mm.FILL_ORDER:
        dx, dy = mm.DIRECTION_VECTORS[direction]
        got = mm.SectorFor(dx * mm.RADIUS_PX, dy * mm.RADIUS_PX)
        _Check(got == direction,
               "the centre of %s resolves to it, got %s" % (direction, got))
    _Check(mm.SectorFor(0.0, -50.0) == "N",
           "negative dy is up the screen, which is North")
    _Check(mm.SectorFor(0.0, 50.0) == "S", "positive dy is South")


def TestSectorBoundaries():
    import math
    # Walk a whole turn at one-degree steps and count what each sector
    # gets: every sector must own exactly 45 of the 360 degrees, which
    # catches a shifted or doubled bucket that spot checks would miss.
    counts = {}
    for degrees in range(360):
        radians = math.radians(degrees + 0.5)
        got = mm.SectorFor(math.cos(radians) * 50.0,
                           -math.sin(radians) * 50.0)
        counts[got] = counts.get(got, 0) + 1
    _Check(sorted(counts) == sorted(mm.FILL_ORDER),
           "every sector is reachable and no other answer is: %s" % counts)
    _Check(set(counts.values()) == set([45]),
           "the turn divides into eight equal sectors: %s" % counts)

    # The boundary itself: 22.5 degrees above East belongs to NE, and a
    # hair under it belongs to E. Whichever side it falls, it must be the
    # SAME side every time, or a flick along the boundary flickers.
    just_under = math.radians(22.4)
    just_over = math.radians(22.6)
    _Check(mm.SectorFor(math.cos(just_under) * 50.0,
                        -math.sin(just_under) * 50.0) == "E",
           "just below the boundary is East")
    _Check(mm.SectorFor(math.cos(just_over) * 50.0,
                        -math.sin(just_over) * 50.0) == "NE",
           "just above the boundary is North-East")
    on_boundary = math.radians(22.5)
    first = mm.SectorFor(math.cos(on_boundary) * 50.0,
                         -math.sin(on_boundary) * 50.0)
    for _ in range(4):
        _Check(mm.SectorFor(math.cos(on_boundary) * 50.0,
                            -math.sin(on_boundary) * 50.0) == first,
               "the boundary answers the same every time")


def TestSectorWraparound():
    # Due West is atan2's +/-180 seam: a whisker either side of it must
    # still be West, not a jump to SW or NW.
    _Check(mm.SectorFor(-50.0, 0.0) == "W", "due West")
    _Check(mm.SectorFor(-50.0, -0.001) == "W", "a whisker above due West")
    _Check(mm.SectorFor(-50.0, 0.001) == "W", "a whisker below due West")
    # 21 degrees off the axis is still West (the sector is 45 wide,
    # centred on it); 30 degrees off is the diagonal.
    _Check(mm.SectorFor(-50.0, -19.0) == "W" and
           mm.SectorFor(-50.0, 19.0) == "W",
           "the West sector reaches 22.5 degrees either side of the axis")
    _Check(mm.SectorFor(-50.0, -30.0) == "NW" and
           mm.SectorFor(-50.0, 30.0) == "SW",
           "and it really does leave West for the diagonals beyond that")


def TestDeadZone():
    _Check(mm.SectorFor(0.0, 0.0) is None, "the press point itself")
    _Check(mm.SectorFor(mm.THRESHOLD_PX - 0.01, 0.0) is None,
           "just inside the dead zone")
    _Check(mm.SectorFor(mm.THRESHOLD_PX, 0.0) == "E",
           "exactly on the threshold circle is out of the dead zone")
    _Check(mm.SectorFor(8.0, 8.0) is None,
           "the dead zone is a circle, not a square: (8, 8) is 11.3 out")
    _Check(mm.SectorFor(9.0, -9.0) == "NE", "and (9, -9) is 12.7 out")


# --- The gesture -------------------------------------------------------
#
# Every test below builds the same eight-item menu so the directions can
# be read straight off the item ids, and drives the Gesture through an
# injected clock rather than passing timestamps, which is how the widget
# will drive it.


def _Gesture(clock, menu=None, context=None):
    return mm.Gesture(menu if menu is not None else mm.Menu("m", _Items(8)),
                      context if context is not None else _Ctx(),
                      clock=clock)


def _At(direction, distance, centre=CENTRE):
    dx, dy = mm.DIRECTION_VECTORS[direction]
    return (centre[0] + dx * distance, centre[1] + dy * distance)


def TestPressDrawsNothing():
    clock = _Clock()
    gesture = _Gesture(clock)
    _Check(_Kind(gesture.Press(*CENTRE), "press") == mm.IDLE,
           "a press draws nothing: which gesture this is, is not known yet")
    _Check(not gesture.shown, "and nothing is on screen")
    clock.t = mm.SHOW_MS - 1.0
    _Check(_Kind(gesture.Tick(), "tick at 149") == mm.IDLE,
           "one millisecond short of SHOW_MS, still nothing")
    _Check(not gesture.shown, "still nothing on screen at 149 ms")
    clock.t = mm.SHOW_MS
    show = _One(gesture.Tick(), mm.SHOW, "tick at 150")
    _Check(gesture.shown and show.centre == CENTRE,
           "at SHOW_MS exactly the ring appears, centred on the press")
    _Check(show.menu.Slot("W").id == "a0",
           "and it is the resolved menu, ready to draw")


def TestTapOpensClickMode():
    clock = _Clock()
    gesture = _Gesture(clock)
    gesture.Press(*CENTRE)
    clock.t = 40.0
    show = _One(gesture.Release(*CENTRE), mm.SHOW, "quick release")
    _Check(show.mode == mm.MODE_CLICK and gesture.mode == mm.MODE_CLICK,
           "a release inside TAP_MS with no direction is a tap: the menu "
           "comes up to be clicked")
    _Check(gesture.shown, "and it stays up")

    # In click mode a move highlights and a click confirms.
    clock.t = 60.0
    highlight = _One(gesture.Move(*_At("S", 60.0)), mm.HIGHLIGHT, "hover")
    _Check(highlight.direction == "S" and highlight.item.id == "a2",
           "hovering South highlights the South item: %s" % (highlight,))
    _Check(_Kind(gesture.Move(*_At("S", 70.0)), "same sector") == mm.IDLE,
           "moving inside the same sector says nothing new")
    clock.t = 80.0
    confirm = _One(gesture.Press(*_At("S", 60.0)), mm.CONFIRM, "click")
    _Check(confirm.item.id == "a2" and confirm.shown,
           "the click confirms the highlighted item: %s" % (confirm,))
    _Check(gesture.state == mm.STATE_IDLE, "and the gesture is over")


def TestHoldFlickRelease():
    clock = _Clock()
    gesture = _Gesture(clock)
    gesture.Press(*CENTRE)
    clock.t = 160.0
    _One(gesture.Tick(), mm.SHOW, "held past SHOW_MS")
    clock.t = 180.0
    emissions = gesture.Move(*_At("E", 40.0))
    highlight = _One(emissions, mm.HIGHLIGHT, "flick East")
    _Check(highlight.item.id == "a1", "East is the second item")
    clock.t = 200.0
    confirm = _One(gesture.Release(*_At("E", 60.0)), mm.CONFIRM, "release")
    _Check(confirm.item.id == "a1" and confirm.direction == "E",
           "releasing on a direction runs it: %s" % (confirm,))
    _Check(confirm.shown, "this one was read off the screen")


def TestFlickUnderShowMs():
    # The expert path, and the reason the menu is resolved on the press:
    # the whole gesture is over before anything could have been drawn.
    clock = _Clock()
    gesture = _Gesture(clock)
    gesture.Press(*CENTRE)
    clock.t = 40.0
    _Check(_Kind(gesture.Move(*_At("W", 50.0)), "mid-flick") == mm.IDLE,
           "nothing is highlighted, because nothing is drawn")
    clock.t = 80.0
    confirm = _One(gesture.Release(*_At("W", 80.0)), mm.CONFIRM, "flick")
    _Check(confirm.item.id == "a0" and confirm.direction == "W",
           "the flick ran the West item: %s" % (confirm,))
    _Check(confirm.shown is False,
           "and it ran with the menu never shown, so the widget has "
           "nothing to take down")


def TestHoldInDeadZoneCancels():
    clock = _Clock()
    gesture = _Gesture(clock)
    gesture.Press(*CENTRE)
    clock.t = mm.SHOW_MS
    _One(gesture.Tick(), mm.SHOW, "shown")
    clock.t = mm.TAP_MS + 50.0
    _Check(_Kind(gesture.Tick(), "still holding") == mm.IDLE,
           "holding still says nothing more")
    cancel = _One(gesture.Release(CENTRE[0] + 5.0, CENTRE[1]), mm.CANCEL,
                  "release in the dead zone")
    _Check(cancel.reason,
           "a cancel says why, for the log: %s" % (cancel,))
    _Check(gesture.state == mm.STATE_IDLE and not gesture.shown,
           "and the menu is gone")

    # The same release one millisecond before TAP_MS is a tap instead.
    gesture = _Gesture(clock)
    clock.t = 1000.0
    gesture.Press(*CENTRE)
    clock.t = 1000.0 + mm.TAP_MS - 1.0
    _One(gesture.Release(*CENTRE), mm.SHOW, "released at TAP_MS - 1")
    _Check(gesture.mode == mm.MODE_CLICK, "that one is a tap")


def TestSubmenuUnfolds():
    items = _Items(4)
    items[1] = dict(id="space", label="Space", kind=mm.SUBMENU,
                    children="spaces")
    context = _Ctx(children={"spaces": [
        dict(id="space.world", label="World", dir="W"),
        dict(id="space.parent", label="Parent", dir="E"),
    ]})
    clock = _Clock()
    gesture = _Gesture(clock, mm.Menu("m", items), context)
    gesture.Press(*CENTRE)
    clock.t = 160.0
    _One(gesture.Tick(), mm.SHOW, "shown")
    clock.t = 170.0
    highlight = _One(gesture.Move(*_At("E", 50.0)), mm.HIGHLIGHT, "aim East")
    _Check(highlight.item.id == "space" and highlight.item.isSubmenu,
           "the submenu is highlighted, and has NOT opened at 50 px")
    _Check(gesture.depth == 0, "nothing unfolded yet")

    clock.t = 180.0
    edge = _At("E", mm.RADIUS_PX)
    opened = _One(gesture.Move(*edge), mm.OPEN, "past the ring")
    _Check(gesture.depth == 1, "the child is open")
    _Check(opened.centre == edge,
           "and it is recentred where the cursor is, so the stroke just "
           "carries on: %s" % (opened.centre,))
    _Check([r.id for r in opened.menu.items] ==
           ["space.world", "space.parent"],
           "the children came from the context, at open time: %s"
           % (opened.menu.items,))
    _Check(gesture.direction is None,
           "the cursor is at the new centre, so it aims at nothing yet")

    # The stroke continues in the child, from the new centre.
    clock.t = 200.0
    highlight = _One(gesture.Move(*_At("W", 40.0, edge)), mm.HIGHLIGHT,
                     "aim in the child")
    _Check(highlight.item.id == "space.world",
           "West in the CHILD, measured from the child's centre: %s"
           % (highlight,))
    clock.t = 220.0
    confirm = _One(gesture.Release(*_At("W", 60.0, edge)), mm.CONFIRM,
                   "release in the child")
    _Check(confirm.item.id == "space.world",
           "and the child item is what runs: %s" % (confirm,))


def TestSubmenuUnfoldsOnDwell():
    # The novice path: the cursor stops on the submenu and never leaves
    # the ring. After UNFOLD_MS it opens anyway, on the item's label.
    items = _Items(4)
    items[0] = dict(id="space", label="Space", kind=mm.SUBMENU,
                    items=[dict(id="space.world", label="World")])
    clock = _Clock()
    gesture = _Gesture(clock, mm.Menu("m", items))
    gesture.Press(*CENTRE)
    clock.t = 160.0
    _One(gesture.Tick(), mm.SHOW, "shown")
    clock.t = 170.0
    _One(gesture.Move(*_At("W", 40.0)), mm.HIGHLIGHT, "resting on W")
    clock.t = 170.0 + mm.UNFOLD_MS - 1.0
    _Check(_Kind(gesture.Tick(), "one ms short") == mm.IDLE,
           "a millisecond short of the dwell, still folded")
    clock.t = 170.0 + mm.UNFOLD_MS
    opened = _One(gesture.Tick(), mm.OPEN, "dwelled")
    _Check(gesture.depth == 1, "the dwell opened it")
    _Check(opened.centre == mm.PointFor(CENTRE, "W"),
           "a dwell recentres on the item's own label, not on wherever "
           "in the sector the cursor happens to rest: %s" % (opened.centre,))


def TestReleaseOnAFoldedSubmenu():
    # Releasing on a submenu that never unfolded cannot "run" it: it
    # opens it and leaves it up to be clicked.
    items = _Items(4)
    items[0] = dict(id="space", label="Space", kind=mm.SUBMENU,
                    items=[dict(id="space.world", label="World")])
    clock = _Clock()
    gesture = _Gesture(clock, mm.Menu("m", items))
    gesture.Press(*CENTRE)
    clock.t = 160.0
    _One(gesture.Tick(), mm.SHOW, "shown")
    clock.t = 200.0
    point = _At("W", 40.0)
    opened = _One(gesture.Move(*point) + gesture.Release(*point), mm.OPEN,
                  "released on a folded submenu")
    _Check(opened.mode == mm.MODE_CLICK and gesture.state == mm.STATE_CLICK,
           "the child is up for clicking, not for stroking: %s" % (opened,))
    clock.t = 220.0
    confirm = _One(gesture.Press(*_At("W", 50.0, opened.centre)),
                   mm.CONFIRM, "click in the child")
    _Check(confirm.item.id == "space.world", "and it confirms in the child")


def TestEscapeAndOutsideClickCancel():
    clock = _Clock()
    gesture = _Gesture(clock)
    gesture.Press(*CENTRE)
    clock.t = 160.0
    _One(gesture.Tick(), mm.SHOW, "shown")
    _Check(_Kind(gesture.Key("Escape"), "escape") == mm.CANCEL,
           "Escape mid-stroke cancels")
    _Check(gesture.state == mm.STATE_IDLE, "and ends the gesture")
    _Check(_Kind(gesture.Key("Escape"), "escape again") == mm.IDLE,
           "a second Escape with nothing up is not a second cancel")

    # And in click mode: a click past the labels is a click at the scene.
    gesture = _Gesture(clock)
    clock.t = 0.0
    gesture.Press(*CENTRE)
    clock.t = 30.0
    _One(gesture.Release(*CENTRE), mm.SHOW, "tap")
    far = _At("E", mm.OUTSIDE_PX + 1.0)
    _Check(_Kind(gesture.Press(*far), "click outside") == mm.CANCEL,
           "a click beyond the whole widget dismisses it")

    # A click back in the dead zone is the artist deciding against it.
    gesture = _Gesture(clock)
    clock.t = 0.0
    gesture.Press(*CENTRE)
    clock.t = 30.0
    _One(gesture.Release(*CENTRE), mm.SHOW, "tap")
    _Check(_Kind(gesture.Press(CENTRE[0] + 2.0, CENTRE[1]), "click centre")
           == mm.CANCEL, "a click in the dead zone dismisses it too")


def TestSecondPressIgnored():
    clock = _Clock()
    gesture = _Gesture(clock)
    gesture.Press(*CENTRE)
    clock.t = 160.0
    _One(gesture.Tick(), mm.SHOW, "shown")
    clock.t = 170.0
    _Check(_Kind(gesture.Press(CENTRE[0] + 200.0, CENTRE[1]),
                 "second press") == mm.IDLE,
           "a second button going down mid-stroke is ignored")
    _Check(gesture.centre == CENTRE and gesture.state == mm.STATE_STROKE,
           "the stroke in progress keeps its centre and its state")
    clock.t = 180.0
    confirm = _One(gesture.Release(*_At("N", 60.0)), mm.CONFIRM, "release")
    _Check(confirm.item.id == "a3",
           "and it still finishes the stroke it was making: %s" % (confirm,))


def TestEmptySectorAndDisabledItem():
    # A four-item menu leaves four sectors empty; flicking into one of
    # them runs nothing.
    clock = _Clock()
    gesture = _Gesture(clock, mm.Menu("m", _Items(4)))
    gesture.Press(*CENTRE)
    clock.t = 80.0
    _Check(_Kind(gesture.Release(*_At("SE", 60.0)), "empty sector")
           == mm.CANCEL, "a flick into an empty sector cancels")

    # And an item its `when` disabled is not confirmable either.
    menu = mm.Menu("m", [dict(id="a0", label="A0", dir="W", when="limb")],
                   missing=mm.DISABLE)
    gesture = _Gesture(clock, menu, _Ctx())
    clock.t = 0.0
    gesture.Press(*CENTRE)
    clock.t = 80.0
    _Check(_Kind(gesture.Release(*_At("W", 60.0)), "disabled") == mm.CANCEL,
           "flicking a disabled item runs nothing")


def TestChooseConfirmsAnOverflowRow():
    clock = _Clock()
    gesture = _Gesture(clock, mm.Menu("m", _Items(9)))
    gesture.Press(*CENTRE)
    clock.t = 30.0
    _One(gesture.Release(*CENTRE), mm.SHOW, "tap")
    confirm = _One(gesture.Choose("a8"), mm.CONFIRM, "overflow row")
    _Check(confirm.item.id == "a8" and confirm.direction is None,
           "the overflow row confirms by id, with no direction: %s"
           % (confirm,))
    _Check(_Kind(gesture.Choose("a8"), "after the event") == mm.IDLE,
           "and once over, it is over")


# --- Resolution --------------------------------------------------------


def TestWhenHidesOrDisables():
    menu = mm.Menu("m", [
        dict(id="always", label="Always", dir="W"),
        dict(id="limbOnly", label="Limb only", dir="E", when="limb"),
    ])
    flick = mm.Resolve(menu, _Ctx(), mm.MODE_FLICK)
    _Check(flick.Find("limbOnly") is None,
           "mid-flick, an item that cannot apply is not drawn at all")
    _Check(flick.Slot("W").id == "always" and flick.Slot("E") is None,
           "and the survivors keep their pinned slots: %s" % (_Kinds(flick),))

    click = mm.Resolve(menu, _Ctx(), mm.MODE_CLICK)
    item = click.Find("limbOnly")
    _Check(item is not None and not item.enabled,
           "once it is up to be read, it is there and greyed out")
    _Check(click.Slot("W").enabled, "the other one is untouched")

    on = mm.Resolve(menu, _Ctx(flags=["limb"]), mm.MODE_FLICK)
    _Check(on.Slot("E") is not None and on.Slot("E").enabled,
           "and with a limb selected it is simply there")

    # A menu may override the mode's default either way.
    always = mm.Menu("m", menu.items, missing=mm.DISABLE)
    resolved = mm.Resolve(always, _Ctx(), mm.MODE_FLICK)
    _Check(resolved.Find("limbOnly") is not None,
           "a menu that declares DISABLE keeps the item even mid-flick")
    hidden = mm.Menu("m", menu.items, missing=mm.HIDE)
    _Check(mm.Resolve(hidden, _Ctx(), mm.MODE_CLICK).Find("limbOnly")
           is None, "and one that declares HIDE hides it even in click mode")


def TestChecksAndLabels():
    menu = mm.Menu("m", [
        dict(id="guides", label="Guides", kind=mm.TOGGLE, read="guides"),
        dict(id="plain", label="Plain"),
        dict(id="session", label="Session", kind=mm.RADIO,
             read="writeMode", value="session"),
        dict(id="rig", label="Rig", kind=mm.RADIO, read="writeMode",
             value="rig"),
    ])
    context = _Ctx(state={"guides": True, "writeMode": "rig"},
                   labels={"plain": "Plain, for this selection"})
    resolved = mm.Resolve(menu, context)
    _Check(resolved.Find("guides").checked is True, "the toggle is on")
    _Check(resolved.Find("plain").checked is None,
           "an action has no check mark at all, which is not the same as "
           "an unticked one")
    _Check(resolved.Find("rig").checked is True and
           not resolved.Find("session").checked,
           "the radio whose value matches the state is the checked one")
    _Check(resolved.Find("plain").label == "Plain, for this selection",
           "the context may rewrite a label for this selection")

    off = mm.Resolve(menu, _Ctx())
    _Check(off.Find("guides").checked is False,
           "a toggle the context knows nothing about reads as off")
    _Check(off.Find("rig").checked is False, "and so does a radio")


def TestContextNeedNotImplementEverything():
    class _Bare(object):
        """Answers predicates and nothing else."""

        def Test(self, name):
            return True

    menu = mm.Menu("m", [
        dict(id="t", label="T", kind=mm.TOGGLE, read="x", when="anything"),
        dict(id="s", label="S", kind=mm.SUBMENU, children="nothing"),
    ])
    resolved = mm.Resolve(menu, _Bare())
    _Check(resolved.Find("t").enabled and resolved.Find("t").label == "T",
           "the missing Label falls back to the authored one")
    _Check(resolved.Find("t").checked is False, "a missing Read is off")
    child = mm.ResolveChild(resolved.Find("s").item, _Bare())
    _Check(not child.items,
           "and a missing Children provider opens empty rather than "
           "raising in the middle of a stroke")


def TestDynamicSubmenuChildren():
    item = mm.ItemFrom(dict(
        id="space", label="Space", kind=mm.SUBMENU,
        items=[dict(id="space.world", label="World", dir="W")],
        children="spaces"))
    context = _Ctx(children={"spaces": [
        dict(id="space.hand", label="Hand"),
        dict(id="space.head", label="Head"),
    ]})
    child = mm.ResolveChild(item, context)
    _Check([r.id for r in child.items] ==
           ["space.world", "space.hand", "space.head"],
           "the declared head comes first, then whatever the context "
           "produced: %s" % (child.items,))
    _Check(child.Slot("W").id == "space.world",
           "the declared one keeps its pin")
    _Check(child.Slot("E").id == "space.hand",
           "and the dynamic ones fill around it: %s" % (_Kinds(child),))
    _Check(child.label == "Space", "the child menu is titled by its item")


def TestRecordsAcceptDicts():
    # The reason dicts are accepted at all: a menu authored in USD, or in
    # a preference file, should arrive as dicts and need no converter
    # beyond the read itself.
    menu = mm.MenuFrom(dict(id="m", label="M", items=[
        dict(id="a", label="A", dir="N"),
    ]))
    _Check(isinstance(menu.items[0], mm.Item) and menu.items[0].dir == "N",
           "a dict menu is a Menu of Items")
    _Check(mm.MenuFrom(menu) is menu, "and a Menu passes straight through")
    bare = mm.MenuFrom([dict(id="a", label="A")])
    _Check(len(bare.items) == 1, "a bare list of items is a menu too")
    for bad in (dict(id="a", label="A", kind="teleport"),
                dict(id="a", label="A", dir="NNE")):
        try:
            mm.ItemFrom(bad)
        except ValueError:
            continue
        raise AssertionError("a nonsense record is refused: %s" % (bad,))


# --- The built-in menus ------------------------------------------------


def TestBuiltInSelectionMenu():
    context = _Ctx(flags=["limb", "hasSpaces", "control"],
                   state={"limbIsIk": True},
                   children={"spaces": [dict(id="s.world", label="World")]})
    resolved = mm.Resolve(defs.SELECTION, context)
    expected = {"W": "selection.key", "E": "selection.reset",
                "S": "selection.zero", "N": "selection.frame",
                "NW": "selection.fkik", "NE": "selection.space",
                "SW": "selection.counterpart",
                "SE": "selection.addToPicker"}
    _Check(_Kinds(resolved) == expected,
           "the Selection ring is laid out as the plan names it: %s"
           % (_Kinds(resolved),))
    _Check([r.id for r in resolved.overflow] == ["selection.usdviewMenu"],
           "with usdview's own prim menu in the overflow column")
    _Check(resolved.Slot("NW").checked is True,
           "the FK/IK toggle reports the limb's current state")
    _Check(resolved.Slot("SW").stub and resolved.Slot("SE").stub,
           "and the two items with no action behind them say so")

    # Nothing useful selected: the four unconditional commands stay
    # exactly where the hand left them.
    empty = mm.Resolve(defs.SELECTION, _Ctx(), mm.MODE_FLICK)
    _Check(_Kinds(empty) == {"W": "selection.key", "E": "selection.reset",
                             "S": "selection.zero", "N": "selection.frame"},
           "pinning is what keeps the muscle memory honest: %s"
           % (_Kinds(empty),))


def TestBuiltInModesMenu():
    context = _Ctx(state={"guides": True, "writeMode": "edit"},
                   children={"tools": [dict(id="tool.move", label="Move")]})
    resolved = mm.Resolve(defs.MODES, context)
    expected = {"W": "modes.guides", "E": "modes.touchPoseLive",
                "S": "modes.touchPosePaint", "N": "modes.focus",
                "NW": "modes.write", "NE": "modes.tool",
                "SE": "modes.toolVisibility"}
    _Check(_Kinds(resolved) == expected,
           "the Modes ring is laid out as the plan names it: %s"
           % (_Kinds(resolved),))
    _Check(not resolved.overflow, "and nothing overflows")
    _Check(resolved.Slot("W").checked is True and
           resolved.Slot("E").checked is False,
           "the mode toggles read their own state")
    write = mm.ResolveChild(resolved.Slot("NW").item, context)
    _Check([r.id for r in write.items if r.checked] == ["modes.write.edit"],
           "exactly one radio in the Write mode group is checked: %s"
           % (write.items,))
    _Check(mm.ResolveChild(resolved.Slot("NE").item, context).items[0].id
           == "tool.move", "and the Tool submenu is filled at open time")
    for itemId in defs.STUBS:
        _Check(defs.MENUS[itemId.split(".")[0]].items,
               "every stub names a menu that exists: %s" % itemId)


def main():
    TestFillOrder()
    TestPinnedSlot()
    TestOverflow()
    TestSectorCentres()
    TestSectorBoundaries()
    TestSectorWraparound()
    TestDeadZone()
    TestPressDrawsNothing()
    TestTapOpensClickMode()
    TestHoldFlickRelease()
    TestFlickUnderShowMs()
    TestHoldInDeadZoneCancels()
    TestSubmenuUnfolds()
    TestSubmenuUnfoldsOnDwell()
    TestReleaseOnAFoldedSubmenu()
    TestEscapeAndOutsideClickCancel()
    TestSecondPressIgnored()
    TestEmptySectorAndDisabledItem()
    TestChooseConfirmsAnOverflowRow()
    TestWhenHidesOrDisables()
    TestChecksAndLabels()
    TestContextNeedNotImplementEverything()
    TestDynamicSubmenuChildren()
    TestRecordsAcceptDicts()
    TestBuiltInSelectionMenu()
    TestBuiltInModesMenu()
    print("test_marking_menu_model: ALL PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
