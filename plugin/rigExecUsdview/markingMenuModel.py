#
# RigExec usdview marking menu: what the gesture MEANS, with no Qt.
#
# A marking menu in the Maya sense: press and the menu does not appear at
# once; flick toward a compass direction and release and the command runs
# with nothing ever drawn; hesitate and the ring fades in around the press
# point so the same eight directions can be READ rather than remembered.
# The muscle memory of the expert and the discoverability of the novice
# are the same gesture at two speeds, which is the whole reason for the
# widget, and it only works if the timings and the sector geometry are
# exactly right. That is why they live here, in a module a test can drive
# a thousand gestures through in a second, rather than inside a QWidget
# where the only way to check a boundary is to move a real mouse.
#
# This module owns four things and nothing else:
#
#   * the RADIAL LAYOUT -- which of the eight slots each item lands in,
#     and what spills into the overflow column;
#   * the SECTOR RESOLUTION -- which direction a cursor offset means;
#   * the GESTURE STATE MACHINE -- press/move/release/tick in, emissions
#     out, with the clock injected so a test can hold a press for exactly
#     149 ms and then for exactly 150;
#   * RESOLUTION -- turning the declared menu into a menu for THIS
#     selection, with labels, enablement and check marks filled in.
#
# It owns none of: Qt, USD, the commands the items run, or where the menu
# records came from. Items are plain data and a `when` predicate is a
# STRING, not a callable, so a menu authored in a USD layer is a converter
# and not a sandbox. The caller resolves those names against a context
# object (see `Context` below) whose whole job is to answer questions
# about the current selection.
#
# THE Y AXIS POINTS DOWN. Every (x, y) here is Qt widget coordinates:
# x grows to the right, y grows DOWNWARD. So North -- the top of the
# screen -- is NEGATIVE dy, and `SectorFor(0, -30)` is "N". Getting this
# backwards flips the menu vertically and nothing else complains, so the
# sector code converts to a mathematical angle exactly once, in
# `SectorFor`, and everything else goes through `DIRECTION_VECTORS`,
# which is written in the same y-down convention.
#
import math
import time


# --- Timings and distances --------------------------------------------
#
# The numbers are Maya's, near enough: it shows the ring after about a
# sixth of a second and treats a quicker press-and-release as a click.
# THRESHOLD_PX has to be large enough that a hand-tremor press-release
# does not fire a random compass direction, and small enough that a
# deliberate flick is never swallowed; 12 logical pixels is about 3 mm on
# a typical display and behaves.
THRESHOLD_PX = 12.0     # dead zone around the centre: no direction here
RADIUS_PX = 96.0        # the ring the labels sit on, and the distance at
#                         which a highlighted submenu unfolds
TAP_MS = 200.0          # a press released before this, with no direction
#                         chosen, is a tap: it opens the menu for clicking
SHOW_MS = 150.0         # hold this long and the menu is drawn
UNFOLD_MS = 250.0       # dwell this long on a submenu and it opens even
#                         if the cursor never left the ring
# A click this far from the centre, once the menu is up for clicking, is
# a click at the scene behind it and cancels. Twice the ring keeps the
# whole widget (labels stick out past RADIUS_PX) inside the live area.
OUTSIDE_PX = RADIUS_PX * 2.0


# --- The eight directions ---------------------------------------------
#
# FILL_ORDER is the order unpinned items take slots in, and it is not
# alphabetical or clockwise: it is the order of the directions a wrist
# flicks most accurately. Left and right first, because a horizontal
# flick is the easiest stroke to make and the easiest to hit; then down
# and up; then the diagonals, which are the ones people miss. An item
# that pins itself to a direction keeps it and the fill skips that slot,
# so a menu can put "Reset to rest" on E forever and still let the rest
# of the items pack themselves.
FILL_ORDER = ("W", "E", "S", "N", "NW", "NE", "SW", "SE")
DIRECTIONS = frozenset(FILL_ORDER)
SLOT_COUNT = len(FILL_ORDER)

# Unit vectors in Qt widget space: y DOWN, so N is (0, -1).
_DIAGONAL = math.sqrt(0.5)
DIRECTION_VECTORS = {
    "E": (1.0, 0.0),
    "NE": (_DIAGONAL, -_DIAGONAL),
    "N": (0.0, -1.0),
    "NW": (-_DIAGONAL, -_DIAGONAL),
    "W": (-1.0, 0.0),
    "SW": (-_DIAGONAL, _DIAGONAL),
    "S": (0.0, 1.0),
    "SE": (_DIAGONAL, _DIAGONAL),
}

# Counter-clockwise from due East in the MATHEMATICAL sense (y up), which
# is what atan2 hands back once the screen dy has been negated. Index i
# covers the 45 degrees centred on i * 45.
_BY_ANGLE = ("E", "NE", "N", "NW", "W", "SW", "S", "SE")
SECTOR_DEGREES = 360.0 / len(_BY_ANGLE)


def SectorFor(dx, dy, threshold=THRESHOLD_PX):
    """Which compass direction an offset from the centre means, or None.

    `dx`, `dy` are in Qt widget coordinates -- y grows DOWNWARD -- so
    (0, -30) is North and (0, 30) is South.

    The eight sectors are 45 degrees each and are CENTRED on the compass
    directions, so due East is the middle of the East sector and not its
    edge; the boundaries fall at 22.5, 67.5, ... degrees. A point exactly
    on a boundary goes to the counter-clockwise sector (in screen terms,
    the more anticlockwise of the two), which is an arbitrary but fixed
    choice: what matters is that it is the same on every run, so a flick
    along a boundary never flickers between two commands.

    Inside `threshold` of the centre there is no direction at all and the
    answer is None: that is the dead zone the cursor starts in, and it is
    what makes "press and release without moving" mean "show me the menu"
    instead of firing whichever sector the press pixel happened to be in.
    A point exactly ON the threshold circle is outside the dead zone.
    """
    if dx * dx + dy * dy < threshold * threshold:
        return None
    # atan2 wants a mathematical y, so negate the screen one exactly
    # here. Adding half a sector before the floor divide is what centres
    # the sectors on the directions, and the two modulos make the +/-180
    # wrap a non-event: an angle of -179.9 and one of +179.9 both land in
    # the West bucket without a single comparison against 180.
    angle = math.degrees(math.atan2(-dy, dx)) % 360.0
    index = int(((angle + SECTOR_DEGREES / 2.0) % 360.0) / SECTOR_DEGREES)
    return _BY_ANGLE[index]


def PointFor(centre, direction, radius=RADIUS_PX):
    """Where the label for `direction` sits, in widget coordinates.

    Pure geometry, kept here rather than in the widget so the test can
    walk a cursor to the exact point a label is drawn at.
    """
    vector = DIRECTION_VECTORS[direction]
    return (centre[0] + vector[0] * radius, centre[1] + vector[1] * radius)


def Distance(a, b):
    """Plain 2D distance, so callers do not each write it out."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


# --- Item and menu records --------------------------------------------
#
# Plain data, and deliberately dumb. `kind` says what the item does to
# the world, `when`/`read`/`write` are NAMES the context resolves, and a
# submenu's children are either declared inline (`items`) or fetched by
# name at open time (`children`) -- the space list of a control cannot be
# written down in advance, so "Space" declares children="spaces" and the
# context produces them when the submenu unfolds.
ACTION = "action"      # run it and the menu closes
TOGGLE = "toggle"      # a checkbox: `read` says on or off, `write` flips
RADIO = "radio"        # one of a set: checked when read(read) == value
SUBMENU = "submenu"    # opens a child menu, recentred on the cursor
KINDS = (ACTION, TOGGLE, RADIO, SUBMENU)

# What a menu does with an item whose `when` predicate says no. Hiding
# keeps the ring uncluttered and, more importantly, keeps the SURVIVING
# items where the muscle remembers them only if they are pinned -- which
# is why the built-in menus pin every slot. Disabling keeps the geometry
# identical no matter what is selected, at the cost of grey labels.
HIDE = "hide"
DISABLE = "disable"

# The two ways the menu is being used, which is what decides the default
# of that policy. Mid-flick nobody is reading anything, so a greyed-out
# label is noise: hide it. Once the menu is up for clicking, the artist
# IS reading it, and "Space is greyed out because nothing with spaces is
# selected" teaches the rig; a silently absent item teaches nothing.
MODE_FLICK = "flick"
MODE_CLICK = "click"


class Item(object):
    """One entry in a marking menu, as authored.

    Fields:
      id       stable name, used by the caller to dispatch the command
               and by `Gesture.Choose` to pick an overflow row
      label    what the ring draws, before the context gets a say
      kind     ACTION / TOGGLE / RADIO / SUBMENU
      dir      a pinned compass direction, or None to take the next free
               slot in FILL_ORDER
      when     the NAME of a predicate the context answers; None means
               always available
      read     the NAME of the state a TOGGLE or RADIO reads its check
               mark from
      write    the NAME the caller writes through when the item fires;
               this module never calls it, it only carries it
      value    the value a RADIO stands for; checked when read == value
      items    a submenu's children, declared inline
      children the NAME of a child list the context produces at open time
      overflow True for an item that belongs in the stacked column even
               when the ring has room -- see Layout
      stub     True for an item the UI should draw but that has no action
               behind it yet (see markingMenuDefs); carried through to
               the resolved item so the caller can grey it or say so
    """

    def __init__(self, id, label, kind=ACTION, dir=None, when=None,
                 read=None, write=None, value=None, items=None,
                 children=None, overflow=False, stub=False):
        if kind not in KINDS:
            raise ValueError("unknown item kind %r" % (kind,))
        if dir is not None and dir not in DIRECTIONS:
            raise ValueError("unknown direction %r" % (dir,))
        self.id = id
        self.label = label
        self.kind = kind
        self.dir = dir
        self.when = when
        self.read = read
        self.write = write
        self.value = value
        self.items = list(items) if items else []
        self.children = children
        self.overflow = bool(overflow)
        self.stub = bool(stub)

    def __repr__(self):
        return "Item(%r, %s%s)" % (
            self.id, self.kind, ", " + self.dir if self.dir else "")


class Menu(object):
    """A menu as authored: an id, a label and its items in order.

    `missing` overrides the mode default for items whose `when` says no
    (HIDE or DISABLE); None leaves the decision to the mode, which is
    what almost every menu wants.
    """

    def __init__(self, id, items, label=None, missing=None):
        if missing not in (None, HIDE, DISABLE):
            raise ValueError("unknown missing policy %r" % (missing,))
        self.id = id
        self.label = label if label is not None else id
        self.items = [ItemFrom(item) for item in items]
        self.missing = missing

    def __repr__(self):
        return "Menu(%r, %d items)" % (self.id, len(self.items))


def ItemFrom(record):
    """An Item from either an Item or a plain dict.

    Accepting dicts is the point: a menu read off a USD prim, or pasted
    into a preference file, arrives as dicts, and the reader that fetches
    it should be a dozen lines of attribute plumbing rather than a second
    copy of the rules above.
    """
    if isinstance(record, Item):
        return record
    return Item(
        id=record.get("id") or "",
        label=record.get("label") or record.get("id") or "",
        kind=record.get("kind") or ACTION,
        dir=record.get("dir"),
        when=record.get("when"),
        read=record.get("read"),
        write=record.get("write"),
        value=record.get("value"),
        items=record.get("items"),
        children=record.get("children"),
        overflow=record.get("overflow"),
        stub=record.get("stub"))


def MenuFrom(record):
    """A Menu from either a Menu, a dict, or a bare list of items."""
    if isinstance(record, Menu):
        return record
    if isinstance(record, (list, tuple)):
        return Menu("", record)
    return Menu(
        id=record.get("id") or "",
        items=record.get("items") or [],
        label=record.get("label"),
        missing=record.get("missing"))


# --- The context ------------------------------------------------------


class Context(object):
    """What the model asks about the world. Duck-typed on purpose.

    The real one will be a summary of the current selection built by the
    usdview plugin -- "one FK control, on a limb, with spaces" -- and it
    has no business being importable from a headless test. So the model
    never imports a context, it only calls these four methods, every one
    of them optional: a context that does not define a method gets the
    documented default, which keeps a throwaway context in a test down to
    the one method it cares about.

      Test(name) -> bool
          Answer a `when` predicate. Default for a context without it:
          False, so an unanswered predicate hides or disables its item
          rather than offering a command that cannot work.

      Read(name) -> value
          The current value behind a TOGGLE's or RADIO's `read` name.
          Default: None, which reads as unchecked.

      Label(itemId, default) -> str
          A chance to rewrite one label for the current selection --
          "Switch to IK" rather than "FK/IK toggle". Default: the
          authored label, untouched.

      Children(name) -> sequence of item records
          The children of a submenu that declared `children`, produced
          when it unfolds and not before. Default: empty, so a submenu
          whose provider is missing opens empty instead of raising in
          the middle of a stroke.

    None of the four may have side effects: `Resolve` calls them once per
    item per resolve, and the gesture re-resolves whenever the mode
    changes under it.
    """

    def Test(self, name):
        return False

    def Read(self, name):
        return None

    def Label(self, itemId, default):
        return default

    def Children(self, name):
        return ()


def _Ask(context, method, default, *args):
    """Call `method` on a duck-typed context, or return `default`.

    A context is anything with some of the four methods; this is the one
    place that tolerates the rest being absent, so no call site has to.
    """
    if context is None:
        return default
    call = getattr(context, method, None)
    if call is None:
        return default
    return call(*args)


# --- Resolution and layout --------------------------------------------


class ResolvedItem(object):
    """An item as it will be drawn, for one selection at one moment.

    `direction` is the slot it took, or None when it went to the overflow
    column. `checked` is True/False for a TOGGLE or RADIO and None for
    everything else, which is not the same as False: None means "this
    item has no check mark", and the widget draws nothing rather than an
    empty box.
    """

    def __init__(self, item, label, enabled, checked, direction=None):
        self.item = item
        self.id = item.id
        self.kind = item.kind
        self.label = label
        self.enabled = enabled
        self.checked = checked
        self.direction = direction
        self.stub = item.stub

    @property
    def isSubmenu(self):
        return self.kind == SUBMENU

    def __repr__(self):
        return "ResolvedItem(%r, %s, %s)" % (
            self.id, self.direction or "overflow",
            "enabled" if self.enabled else "disabled")


class ResolvedMenu(object):
    """A menu laid out for one selection: eight slots and an overflow.

    `source` is kept so the gesture can resolve the SAME menu again when
    the mode changes under it -- a flick that turns into a click has to
    re-ask, because the two modes disagree about hidden versus disabled.
    """

    def __init__(self, source, mode, slots, overflow):
        self.source = source
        self.id = source.id
        self.label = source.label
        self.mode = mode
        self.slots = slots          # direction -> ResolvedItem
        self.overflow = list(overflow)

    @property
    def items(self):
        """Everything drawn, ring first in fill order, then overflow."""
        ring = [self.slots[d] for d in FILL_ORDER if d in self.slots]
        return ring + self.overflow

    def Slot(self, direction):
        """The item in that direction, or None for an empty sector."""
        return self.slots.get(direction)

    def Find(self, itemId):
        """Any drawn item by id, overflow included, or None."""
        for resolved in self.items:
            if resolved.id == itemId:
                return resolved
        return None

    def __repr__(self):
        return "ResolvedMenu(%r, %d in ring, %d overflow)" % (
            self.id, len(self.slots), len(self.overflow))


def Layout(items):
    """Place resolved items into the ring, and return (slots, overflow).

    Two passes, and the order matters. Pinned items claim their slots
    first, so a pin always wins over a fill even when the pinned item is
    declared last; then the unpinned items take what is left, in
    FILL_ORDER, in declaration order. A second item pinned to a slot
    already taken loses the pin and joins the fill -- an authoring
    mistake should cost that item its preferred place, not silently
    delete the item that got there first.

    Anything with no slot left goes to the overflow list in declaration
    order. Overflow is NOT a ninth direction: it is drawn as a stacked
    column beside the ring and can only be clicked, never flicked, which
    is exactly why the built-in menus keep the eight commands that
    deserve muscle memory in the ring and put the rest below it.

    An item may also declare `overflow` and go to the column even when
    the ring has room. Without that flag a menu's long tail moves: hide
    two conditional items and the eleventh-most-wanted command is
    suddenly a flick away, in a slot that was empty a moment ago and
    will be something else the moment the selection changes. Anything
    that must never surprise a hand mid-flick says so once, in the
    record, rather than depending on how many items happened to survive
    this resolve.

    `items` are ResolvedItem instances; this mutates their `direction`.
    """
    slots = {}
    fill = []
    column = []
    for resolved in items:
        if resolved.item.overflow:
            resolved.direction = None
            column.append(resolved)
            continue
        pin = resolved.item.dir
        if pin is not None and pin not in slots:
            resolved.direction = pin
            slots[pin] = resolved
        else:
            fill.append(resolved)
    free = [d for d in FILL_ORDER if d not in slots]
    overflow = column
    for resolved in fill:
        if free:
            direction = free.pop(0)
            resolved.direction = direction
            slots[direction] = resolved
        else:
            resolved.direction = None
            overflow.append(resolved)
    return slots, overflow


def Resolve(menu, context, mode=MODE_FLICK):
    """Turn an authored menu into the menu for THIS selection.

    Asks the context one question per item -- is your `when` satisfied,
    what is your label, are you checked -- then lays the survivors out.
    The mode decides what happens to an item whose `when` says no, unless
    the menu overrode it: hidden mid-flick, greyed out once the menu is
    up to be read (see HIDE/DISABLE above).
    """
    menu = MenuFrom(menu)
    policy = menu.missing or (HIDE if mode == MODE_FLICK else DISABLE)
    resolved = []
    for item in menu.items:
        available = True if item.when is None else \
            bool(_Ask(context, "Test", False, item.when))
        if not available and policy == HIDE:
            continue
        label = _Ask(context, "Label", item.label, item.id, item.label)
        resolved.append(ResolvedItem(
            item, label if label is not None else item.label,
            available, CheckedState(item, context)))
    slots, overflow = Layout(resolved)
    return ResolvedMenu(menu, mode, slots, overflow)


def CheckedState(item, context):
    """True/False for a TOGGLE or RADIO, None for anything else.

    A RADIO is checked when the state its `read` name points at equals
    the `value` it stands for, so a whole radio group reads one piece of
    state and no caller has to keep a "which one is on" flag in sync.
    """
    if item.kind == TOGGLE:
        return bool(_Ask(context, "Read", None, item.read))
    if item.kind == RADIO:
        return _Ask(context, "Read", None, item.read) == item.value
    return None


def ResolveChild(item, context, mode=MODE_FLICK):
    """The menu behind a SUBMENU item, resolved and laid out.

    Inline `items` and a `children` provider name are not alternatives:
    a submenu may declare a fixed head (say "Global" and "Local") and
    have the context append whatever the selected control actually has.
    The provider's records go through ItemFrom like any others, so it may
    hand back dicts and never import this module.
    """
    records = list(item.items)
    if item.children:
        records.extend(_Ask(context, "Children", (), item.children) or ())
    child = Menu(item.id, records, label=item.label)
    return Resolve(child, context, mode)


# --- The gesture ------------------------------------------------------
#
# What the widget gets back. Every call into the Gesture answers with a
# LIST of these, never a bare one, because a single move can both show
# the menu and light a direction inside it, and a caller that has to
# guess which of two things happened will get it wrong. The list is
# never empty: nothing-happened is a single IDLE, so `emits[0].kind` is
# always safe to switch on.
IDLE = "idle"           # nothing to draw differently
SHOW = "show"           # draw `menu` centred at `centre`
HIGHLIGHT = "highlight"  # `direction`/`item` are now under the cursor
#                          (both None when the cursor is in the dead zone)
OPEN = "open"           # a submenu unfolded: draw `menu` at `centre`
CONFIRM = "confirm"     # run `item`; the gesture is over
CANCEL = "cancel"       # nothing runs; the gesture is over


class Emission(object):
    """One thing the gesture is telling the widget to do."""

    def __init__(self, kind, menu=None, centre=None, direction=None,
                 item=None, mode=None, reason=None, shown=None):
        self.kind = kind
        self.menu = menu
        self.centre = centre
        self.direction = direction
        self.item = item
        self.mode = mode
        self.reason = reason
        # On a CONFIRM: was anything ever drawn? False is the flick that
        # ran a command with the ring never on screen, and the widget
        # needs to know so it does not try to fade out a menu it never
        # put up. None on every other kind.
        self.shown = shown

    def __repr__(self):
        parts = [self.kind]
        if self.direction:
            parts.append(self.direction)
        if self.item is not None:
            parts.append(self.item.id)
        if self.reason:
            parts.append("(%s)" % self.reason)
        return "Emission(%s)" % " ".join(parts)


# Gesture states. ARMED is the interesting one: the button is down, the
# clock is running, and NOTHING has been drawn -- the whole flick path
# lives and dies in this state.
STATE_IDLE = "idle"
STATE_ARMED = "armed"
STATE_STROKE = "stroke"   # shown, button still down
STATE_CLICK = "click"     # shown, button up, waiting to be clicked


def _WallClock():
    """Milliseconds off a monotonic clock.

    Only the default: every test injects its own, because a test that
    sleeps for 150 ms to check a 150 ms threshold is both slow and a
    liar -- it can never check the boundary itself, and it fails on a
    loaded machine.
    """
    return time.monotonic() * 1000.0


class Gesture(object):
    """The marking-menu state machine: events in, emissions out.

    Drive it with Press / Move / Release / Tick / Key / Choose, each of
    which takes an optional `t_ms`; when it is left out the injected
    clock is asked. Tick exists because two of the rules are about time
    passing with the mouse perfectly still -- the menu appears while the
    cursor has not moved a pixel, and a submenu unfolds under a resting
    cursor -- and a state machine fed only by mouse events can never
    notice either. The widget arms a timer and calls Tick; the test just
    calls Tick with the millisecond it wants to be at.

    The rules, in one place:

      * a press records where and when, and draws nothing;
      * once the press is SHOW_MS old the menu is shown, wherever the
        cursor is by then;
      * releasing with a direction confirms that direction's item --
        including a flick that finishes before SHOW_MS, which is the
        expert path and confirms having shown nothing at all;
      * releasing with no direction inside TAP_MS is a tap: the menu is
        shown (if it was not already) and stays up for clicking;
      * releasing with no direction after TAP_MS -- a hold that went
        nowhere -- cancels;
      * a highlighted SUBMENU unfolds when the cursor passes RADIUS_PX
        from the current centre, or dwells on it for UNFOLD_MS, and the
        child is recentred at that point so the stroke simply continues;
      * Escape, and a click outside the ring once it is up, cancel.
    """

    def __init__(self, menu, context, clock=None, resolver=None):
        self.menu = MenuFrom(menu)
        self.context = context
        self.clock = clock if clock is not None else _WallClock
        # Injected so a test can watch the resolves happen, and so a
        # caller with a cached resolve can hand one in.
        self.resolver = resolver if resolver is not None else Resolve
        self.Reset()

    # -- state ---------------------------------------------------------

    def Reset(self):
        """Back to nothing pressed and nothing drawn."""
        self.state = STATE_IDLE
        self.mode = MODE_FLICK
        self.resolved = None     # the ResolvedMenu currently in play
        self.stack = []          # (ResolvedMenu, centre) of the parents
        self.origin = None       # where the press landed
        self.centre = None       # where the CURRENT menu is centred
        self.point = None        # where the cursor is now
        self.pressTime = None
        self.direction = None
        self.shown = False
        self._highlighted = None  # the direction last reported as HIGHLIGHT
        self._dwellFrom = None   # when the current direction was entered

    @property
    def depth(self):
        """How many submenus deep the stroke has gone. 0 at the root."""
        return len(self.stack)

    def _Now(self, t_ms):
        return float(t_ms) if t_ms is not None else float(self.clock())

    def _Since(self, now):
        return now - self.pressTime if self.pressTime is not None else 0.0

    # -- events --------------------------------------------------------

    def Press(self, x, y, t_ms=None):
        """Mouse down.

        Draws nothing and emits nothing: whether this press turns into a
        flick, a tap or a hold is not knowable yet, and guessing would
        flash the ring on every quick command.

        The menu IS resolved here, though, and not at the show. A flick
        can be over in 80 ms, and it still has to know what W meant --
        resolving on the show would leave the fast path with no menu to
        read the direction out of.

        A press that arrives while a stroke is already live is ignored:
        that is a second button going down mid-gesture (or an event
        replayed by the platform), and abandoning the stroke half way
        through would lose the command the artist is in the middle of
        making. Once the menu is up for CLICKING, though, a press is the
        click, and it is handled as one.
        """
        now = self._Now(t_ms)
        if self.state == STATE_CLICK:
            return self._ClickPress(x, y, now)
        if self.state in (STATE_ARMED, STATE_STROKE):
            return [Emission(IDLE, reason="press during stroke")]
        self.Reset()
        self.state = STATE_ARMED
        self.mode = MODE_FLICK
        self.origin = (float(x), float(y))
        self.centre = self.origin
        self.point = self.origin
        self.pressTime = now
        self._dwellFrom = now
        self.resolved = self.resolver(self.menu, self.context, MODE_FLICK)
        return [Emission(IDLE, reason="armed")]

    def Move(self, x, y, t_ms=None):
        """Mouse moved -- with the button down, or not, in click mode."""
        if self.state == STATE_IDLE:
            return [Emission(IDLE)]
        now = self._Now(t_ms)
        self.point = (float(x), float(y))
        emissions = []
        if self.state in (STATE_ARMED, STATE_STROKE):
            emissions.extend(self._MaybeShow(now))
        emissions.extend(self._Aim(now))
        emissions.extend(self._MaybeUnfold(now))
        return emissions or [Emission(IDLE)]

    def Tick(self, t_ms=None):
        """Time passed with no mouse event.

        The only two rules that need it: the menu appearing under a still
        cursor, and a submenu unfolding under one.
        """
        if self.state == STATE_IDLE:
            return [Emission(IDLE)]
        now = self._Now(t_ms)
        emissions = []
        if self.state in (STATE_ARMED, STATE_STROKE):
            emissions.extend(self._MaybeShow(now))
            emissions.extend(self._Aim(now))
        emissions.extend(self._MaybeUnfold(now))
        return emissions or [Emission(IDLE)]

    def Release(self, x, y, t_ms=None):
        """Mouse up. Where the whole gesture is decided.

        In click mode the release is the tail of the click that already
        confirmed on the press, so it does nothing; everywhere else this
        confirms, opens, taps or cancels and the gesture ends.
        """
        if self.state in (STATE_IDLE, STATE_CLICK):
            return [Emission(IDLE)]
        now = self._Now(t_ms)
        self.point = (float(x), float(y))
        direction = SectorFor(self.point[0] - self.centre[0],
                              self.point[1] - self.centre[1])
        if direction is None:
            # Nothing was aimed at. Quick enough and it was a tap: put
            # the menu up and leave it up. Slow enough and it was a hold
            # that went nowhere, which is how a marking menu is meant to
            # be backed out of.
            if self._Since(now) < TAP_MS:
                return self._EnterClickMode(now)
            return self._Cancel("held with no direction")
        item = self.resolved.Slot(direction)
        if item is None:
            return self._Cancel("released on an empty sector")
        if not item.enabled:
            return self._Cancel("released on a disabled item")
        if item.isSubmenu:
            # A submenu the stroke never pulled far enough to unfold.
            # Confirming it would mean running a menu, which is not a
            # command; opening it under the cursor and leaving it up to
            # be clicked is the only reading of "release here" that does
            # what the artist was plainly trying to do. The mode changes
            # BEFORE the unfold so the child is resolved the way a menu
            # that is going to be read should be -- disabled, not hidden.
            self.mode = MODE_CLICK
            self.state = STATE_CLICK
            return self._Unfold(item, self.point, now)
        return self._Confirm(item)

    def Key(self, key, t_ms=None):
        """A key while the menu is live. Escape cancels; the rest pass."""
        if self.state == STATE_IDLE:
            return [Emission(IDLE)]
        if key in ("Escape", "Esc", "escape"):
            return self._Cancel("escape")
        return [Emission(IDLE)]

    def Choose(self, itemId, t_ms=None):
        """Confirm an item by id -- how an overflow row is clicked.

        The overflow column is outside the ring and has no direction, so
        the widget hit-tests its own rows and names the winner here
        rather than inventing a direction for it.
        """
        if self.state == STATE_IDLE or self.resolved is None:
            return [Emission(IDLE)]
        item = self.resolved.Find(itemId)
        if item is None or not item.enabled:
            return [Emission(IDLE, reason="no such item")]
        if item.isSubmenu:
            point = self.point or self.centre
            return self._Unfold(item, point, self._Now(t_ms))
        return self._Confirm(item)

    def Cancel(self):
        """Give up from outside -- focus lost, the window closed."""
        if self.state == STATE_IDLE:
            return [Emission(IDLE)]
        return self._Cancel("cancelled")

    # -- the pieces the events are made of -----------------------------

    def _MaybeShow(self, now):
        if self.shown or self._Since(now) < SHOW_MS:
            return []
        self.shown = True
        self.state = STATE_STROKE
        return [Emission(SHOW, menu=self.resolved, centre=self.centre,
                         mode=self.mode)]

    def _Aim(self, now):
        """Recompute the direction and report it if it changed.

        Nothing is reported before the menu is shown -- there is nothing
        on screen to highlight -- but the direction is tracked all the
        same, so the instant the menu does appear it appears with the
        right slot already lit. A direction that goes back to None is
        reported too: that is the cursor coming home to the dead zone and
        the highlight has to go out.
        """
        direction = SectorFor(self.point[0] - self.centre[0],
                              self.point[1] - self.centre[1])
        if direction != self.direction:
            self.direction = direction
            self._dwellFrom = now
        if not self.shown or direction == self._highlighted:
            return []
        self._highlighted = direction
        item = self.resolved.Slot(direction) if direction else None
        return [Emission(HIGHLIGHT, menu=self.resolved, centre=self.centre,
                         direction=direction, item=item, mode=self.mode)]

    def _MaybeUnfold(self, now):
        """Open the highlighted submenu if it has earned it.

        Two ways in, and they are for two different users. The stroke
        that keeps going past RADIUS_PX is the expert drawing one
        continuous mark through a submenu and out the other side; the
        cursor that just sits on the item for UNFOLD_MS is the novice
        who has stopped to read. Both recentre the child, but not in the
        same place: the stroke recentres under the cursor, where the hand
        already is, while the dwell recentres on the item's own label,
        because a cursor resting anywhere in a 45 degree sector is not
        the point it meant.
        """
        if not self.shown or self.direction is None:
            return []
        item = self.resolved.Slot(self.direction)
        if item is None or not item.isSubmenu or not item.enabled:
            return []
        reach = Distance(self.centre, self.point)
        if reach >= RADIUS_PX:
            return self._Unfold(item, self.point, now)
        if self._dwellFrom is not None and now - self._dwellFrom >= UNFOLD_MS:
            return self._Unfold(item, PointFor(self.centre, self.direction),
                                now)
        return []

    def _Unfold(self, item, centre, now):
        """Push the parent, resolve the child, recentre on `centre`.

        The recentring is what makes a submenu part of the same stroke:
        the child's dead zone starts under the cursor, so the artist
        flicks again from where they already are instead of travelling
        back to the original press point.
        """
        child = ResolveChild(item.item, self.context, self.mode)
        self.stack.append((self.resolved, self.centre))
        self.resolved = child
        self.centre = (float(centre[0]), float(centre[1]))
        self.direction = None
        self._highlighted = None
        self._dwellFrom = now
        if not self.shown:
            # Can only happen through Choose() before the ring appeared;
            # a child menu with no parent drawn would be a menu from
            # nowhere, so count it as shown.
            self.shown = True
        return [Emission(OPEN, menu=child, centre=self.centre,
                         mode=self.mode, item=item)]

    def _EnterClickMode(self, now):
        """The menu stops being a stroke and becomes a thing to click.

        The menu is resolved AGAIN, because the two modes disagree about
        what to do with an item whose `when` says no: what was hidden
        mid-flick becomes a greyed-out label the artist can see and ask
        about. That can change the layout, so a fresh SHOW goes out and
        the widget redraws from it rather than patching what it had.
        """
        self.mode = MODE_CLICK
        self.state = STATE_CLICK
        self.resolved = self.resolver(self.resolved.source, self.context,
                                      MODE_CLICK)
        self.direction = None
        self._highlighted = None
        self._dwellFrom = now
        self.shown = True
        return [Emission(SHOW, menu=self.resolved, centre=self.centre,
                         mode=self.mode)]

    def _ClickPress(self, x, y, now):
        """A press while the menu is up for clicking."""
        self.point = (float(x), float(y))
        direction = SectorFor(self.point[0] - self.centre[0],
                              self.point[1] - self.centre[1])
        reach = Distance(self.centre, self.point)
        if direction is None or reach > OUTSIDE_PX:
            # The dead zone in click mode is the centre of a menu the
            # artist has decided against, and anything out past the
            # labels is a click at the scene behind it. Both dismiss.
            return self._Cancel("clicked outside")
        item = self.resolved.Slot(direction)
        if item is None or not item.enabled:
            return [Emission(IDLE, reason="clicked a dead sector")]
        if item.isSubmenu:
            return self._Unfold(item, self.point, now)
        return self._Confirm(item)

    def _Confirm(self, item):
        emission = Emission(CONFIRM, menu=self.resolved, centre=self.centre,
                            direction=item.direction, item=item,
                            mode=self.mode, shown=self.shown)
        self.Reset()
        return [emission]

    def _Cancel(self, reason):
        emission = Emission(CANCEL, menu=self.resolved, centre=self.centre,
                            mode=self.mode, reason=reason)
        self.Reset()
        return [emission]
