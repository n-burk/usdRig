"""The hover picker's layout and gestures, with no Qt in sight.

The hover picker draws each picker tab straight into the viewport as a
group of buttons hanging off a small circle (the tab's handle), in 2D
screen space. This module owns where each group is, how big, how opaque,
and whether it is collapsed, plus the arithmetic the gestures apply to
those. The drawing and the mouse live in `pickerHoverUI`.

Coordinates are the viewport's logical pixels, origin top-left. A tab's
(x, y) is the centre of its handle; its buttons hang below it with their
top-left corner under the handle's left edge, at the tab's scale.

The layout is a viewer preference, never scene data: it is saved per user
(`ToJson` / `FromJson`), keyed by the picker and the tab it shows.
"""
import json
import math

# The handle's radius and the gap between it and the buttons, in screen
# pixels. Neither scales with the tab: the handle has to stay grabbable
# however small the buttons are made.
HANDLE_RADIUS = 7.0
HANDLE_GAP = 6.0
# Grabbing slop around the handle, so a 14 px circle is not a precision
# target.
HANDLE_SLOP = 3.0

MIN_SCALE = 0.2
MAX_SCALE = 4.0
MIN_OPACITY = 0.05
MAX_OPACITY = 1.0
# Shift-drag on a handle: the scale multiplies by exp(rate * px), so equal
# drags give equal ratios whatever the current size.
SCALE_RATE = 0.006
# Middle-drag: a full sweep from faint to solid is 200 px.
OPACITY_RATE = 1.0 / 200.0

# Where the first handle goes the first time hover mode is shown, and how
# far apart the handles sit. Right of usdview's top-left HUD block (250 px
# wide) so a fresh layout never sits on its text.
FIRST_HANDLE = (270.0, 24.0)
HANDLE_SPACING = 26.0
# A fresh tab is sized so its buttons take at most this fraction of the
# viewport's height, and never more than its authored size.
FIT_FRACTION = 0.45

# The overall-opacity knob, bottom-right: inset from the corner and lifted
# clear of usdview's bottom-right HUD block (two 15 px lines).
KNOB_RADIUS = 8.0
KNOB_INSET = 18.0
KNOB_HUD_CLEARANCE = 36.0


def _Clamp(value, low, high):
    return max(low, min(high, value))


def TabKey(pickerName, panelId):
    """The key a tab's layout is saved under."""
    return "%s/%s" % (pickerName or "", panelId or "")


class TabLayout(object):
    """One tab's place on screen."""

    def __init__(self, key, x=0.0, y=0.0, scale=1.0, opacity=1.0,
                 collapsed=False):
        self.key = key
        self.x = float(x)
        self.y = float(y)
        self.scale = _Clamp(float(scale), MIN_SCALE, MAX_SCALE)
        self.opacity = _Clamp(float(opacity), MIN_OPACITY, MAX_OPACITY)
        self.collapsed = bool(collapsed)

    def ToDict(self):
        return {"x": self.x, "y": self.y, "scale": self.scale,
                "opacity": self.opacity, "collapsed": self.collapsed}

    @classmethod
    def FromDict(cls, key, data):
        return cls(key, data.get("x", 0.0), data.get("y", 0.0),
                   data.get("scale", 1.0), data.get("opacity", 1.0),
                   data.get("collapsed", False))

    # -- geometry ---------------------------------------------------------

    def ButtonsCorner(self):
        """Screen point the tab's content origin is drawn at."""
        return (self.x - HANDLE_RADIUS,
                self.y + HANDLE_RADIUS + HANDLE_GAP)

    def ToScreen(self, origin, point):
        """A panel-space point as a screen point. `origin` is the panel's
        content origin (pickerModel.content_box)."""
        cx, cy = self.ButtonsCorner()
        return (cx + (point[0] - origin[0]) * self.scale,
                cy + (point[1] - origin[1]) * self.scale)

    def ToPanel(self, origin, point):
        """The inverse of ToScreen."""
        cx, cy = self.ButtonsCorner()
        return (origin[0] + (point[0] - cx) / self.scale,
                origin[1] + (point[1] - cy) / self.scale)

    def HitsHandle(self, point):
        return math.hypot(point[0] - self.x, point[1] - self.y) <= (
            HANDLE_RADIUS + HANDLE_SLOP)

    # -- gestures ---------------------------------------------------------

    def Moved(self, dx, dy):
        self.x += dx
        self.y += dy

    def Scaled(self, startScale, dx, dy):
        """Shift-drag from where it started: right or up grows, left or
        down shrinks. The handle stays put; the buttons follow it."""
        # The exponent is bounded first: the result clamps anyway, and a
        # drag far off screen must not overflow on the way there.
        power = _Clamp(SCALE_RATE * (dx - dy), -50.0, 50.0)
        self.scale = _Clamp(startScale * math.exp(power), MIN_SCALE,
                            MAX_SCALE)

    def Faded(self, startOpacity, dx):
        """Middle-drag from where it started: right is more opaque."""
        self.opacity = _Clamp(startOpacity + dx * OPACITY_RATE,
                              MIN_OPACITY, MAX_OPACITY)


def FitScale(contentHeight, viewHeight):
    """The scale a fresh tab starts at."""
    if contentHeight <= 0.0 or viewHeight <= 0.0:
        return 1.0
    return _Clamp(min(1.0, FIT_FRACTION * viewHeight / contentHeight),
                  MIN_SCALE, MAX_SCALE)


class Layout(object):
    """Every tab's place, the overall opacity, and whether hover is on."""

    def __init__(self):
        self.tabs = {}
        self.opacity = 1.0
        self.enabled = False

    def Tab(self, key):
        return self.tabs.get(key)

    def Ensure(self, keys, fitScales=None):
        """Give every key that has no layout yet one: handles side by side
        along the top, right of anything already placed in that row, all
        collapsed but the first tab of a fresh layout.

        `fitScales` maps a key to the scale it should start at."""
        fitScales = fitScales or {}
        placed = [t for t in self.tabs.values()
                  if abs(t.y - FIRST_HANDLE[1]) < 1e-6]
        nextX = FIRST_HANDLE[0]
        if placed:
            nextX = max(t.x for t in placed) + HANDLE_SPACING
        # Adjacent handles would open their groups on top of each other,
        # so a fresh layout opens only its first tab; the rest wait as
        # handles in the row.
        opened = bool(self.tabs)
        for key in keys:
            if key in self.tabs:
                continue
            self.tabs[key] = TabLayout(key, nextX, FIRST_HANDLE[1],
                                       fitScales.get(key, 1.0),
                                       collapsed=opened)
            opened = True
            nextX += HANDLE_SPACING

    def Faded(self, startOpacity, dx):
        """The overall opacity, from a drag on the knob."""
        self.opacity = _Clamp(startOpacity + dx * OPACITY_RATE,
                              MIN_OPACITY, MAX_OPACITY)

    def TabOpacity(self, key):
        """What a tab is drawn at: its own opacity times the overall."""
        tab = self.tabs.get(key)
        return (tab.opacity if tab else 1.0) * self.opacity

    # -- persistence ------------------------------------------------------

    def ToJson(self):
        return json.dumps({
            "enabled": self.enabled,
            "opacity": self.opacity,
            "tabs": {k: t.ToDict() for k, t in sorted(self.tabs.items())},
        }, sort_keys=True)

    @classmethod
    def FromJson(cls, text):
        layout = cls()
        try:
            data = json.loads(text) if text else {}
        except (TypeError, ValueError):
            data = {}
        if not isinstance(data, dict):
            data = {}
        layout.enabled = bool(data.get("enabled", False))
        try:
            layout.opacity = _Clamp(float(data.get("opacity", 1.0)),
                                    MIN_OPACITY, MAX_OPACITY)
        except (TypeError, ValueError):
            layout.opacity = 1.0
        tabs = data.get("tabs")
        for key, tab in (tabs.items() if isinstance(tabs, dict) else ()):
            if isinstance(tab, dict):
                try:
                    layout.tabs[key] = TabLayout.FromDict(key, tab)
                except (TypeError, ValueError):
                    pass
        return layout


def KnobCentre(viewWidth, viewHeight):
    """Where the overall-opacity knob sits in a viewport this size."""
    return (viewWidth - KNOB_INSET - KNOB_RADIUS,
            viewHeight - KNOB_HUD_CLEARANCE - KNOB_RADIUS)


def HitsKnob(viewWidth, viewHeight, point):
    cx, cy = KnobCentre(viewWidth, viewHeight)
    return math.hypot(point[0] - cx, point[1] - cy) <= (
        KNOB_RADIUS + HANDLE_SLOP)
