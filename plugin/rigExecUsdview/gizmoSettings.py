#
# RigExec usdview gizmo: the per-tool settings model behind the Tool
# Settings panel (design spec section 8.6).
#
# Deliberately Qt-free, and deliberately not a dict: the panel, the
# controller and the drag code all read the same fields, and a typo in a
# dict key would be a silent wrong default rather than an AttributeError.
# Keeping it out of gizmoUI.py is what lets "what does the Move tool
# start as" be tested headlessly, which is the part of manipulator parity a
# reviewer can actually check.
#
# The tool tokens come from gizmoScreen rather than being restated here
# so there is exactly one spelling of "translate" in the plugin;
# gizmoScreen is itself Qt-free (pure Gf), so importing it costs nothing
# a headless test cannot pay.
#
from gizmoScreen import TOOL_TRANSLATE, TOOL_ROTATE, TOOL_SCALE
# The pivot tokens live with the maths that honours them, for the same
# reason the tool tokens live in gizmoScreen: one spelling, and the
# module that acts on a value owns it.
from gizmoMath import (GROUP_PIVOT_CENTER, GROUP_PIVOT_LEAD,
                       GROUP_PIVOT_INDIVIDUAL, GROUP_PIVOT_MODES)

# gizmoUI re-exports this; it lives here because ToolDefaults and
# OrientationChoices both have to answer for it.
TOOL_SELECT = "select"

TOOLS = (TOOL_SELECT, TOOL_TRANSLATE, TOOL_ROTATE, TOOL_SCALE)

# the conventional Axis Orientation menu. World / Object / Parent for Move and
# Scale; Object / World / Gimbal for Rotate (the conventional Rotate Tool has no
# Parent entry -- its "Gimbal" is the parent-space Euler decomposition).
ORIENT_WORLD = "world"
ORIENT_OBJECT = "object"
ORIENT_PARENT = "parent"
ORIENT_GIMBAL = "gimbal"

# The two the toolbar's Global/Local toggle cycles between, in the order
# it cycles them. The toggle and the panel's Axis Orientation combo box
# write the SAME per-tool field, so there is one answer to "which axes
# am I on" however it was chosen; the toggle just covers the two an
# animator switches all day and leaves Parent and Gimbal to the panel.
# Named here rather than in gizmoUI so the pairing is testable headlessly.
ORIENT_TOGGLE = (ORIENT_WORLD, ORIENT_OBJECT)

# conventional snap modes for the viewport gizmo (snapping design
# section 1). The tokens alone let gizmoSnap re-export one spelling;
# the labels, per-tool choices, sticky field and grid size below are
# the Task 2 half, added together so no _FIELDS entry lacks its
# ToolDefaults default (a KeyError in every ToolSettings).
SNAP_OFF = "off"
SNAP_GRID = "grid"
SNAP_POINT = "point"
SNAP_EDGE = "edge"
SNAP_SURFACE = "surface"

_ORIENT_LABELS = {
    ORIENT_WORLD: "World",
    ORIENT_OBJECT: "Object",
    ORIENT_PARENT: "Parent",
    ORIENT_GIMBAL: "Gimbal",
}

# What the TOOLBAR calls them. the conventional tool says World/Object; every other DCC
# an animator is likely to have used says Global/Local, and the toolbar
# has room for one word. The panel keeps the conventional spelling because the
# rest of that panel is the authored data's.
_TOGGLE_LABELS = {
    ORIENT_WORLD: "Global",
    ORIENT_OBJECT: "Local",
}

_GROUP_PIVOT_LABELS = {
    GROUP_PIVOT_CENTER: "Selection Centre",
    GROUP_PIVOT_LEAD: "Last Selected",
    GROUP_PIVOT_INDIVIDUAL: "Individual Origins",
}

# What the TOOLBAR button says. The bar has room for one short phrase,
# and "Group:" rather than "Pivot:" on purpose: the row already has a
# Pose/Pivot pair two groups along that means something else entirely
# (which CHANNELS are edited), and two controls both saying "Pivot"
# would be read as one setting shown twice.
_GROUP_PIVOT_SHORT = {
    GROUP_PIVOT_CENTER: "Centre",
    GROUP_PIVOT_LEAD: "Lead",
    GROUP_PIVOT_INDIVIDUAL: "Each",
}

_ORIENT_CHOICES = {
    TOOL_TRANSLATE: (ORIENT_WORLD, ORIENT_OBJECT, ORIENT_PARENT),
    TOOL_SCALE: (ORIENT_WORLD, ORIENT_OBJECT, ORIENT_PARENT),
    TOOL_ROTATE: (ORIENT_OBJECT, ORIENT_WORLD, ORIENT_GIMBAL),
}

_SNAP_LABELS = {
    SNAP_OFF: "Off",
    SNAP_GRID: "Grid",
    SNAP_POINT: "Point",
    SNAP_EDGE: "Edge",
    SNAP_SURFACE: "Surface",
}

# Move offers every mode; Rotate offers off and the absolute degree
# grid; Scale and Select are absent so SnapChoices returns () for
# them, exactly as _ORIENT_CHOICES omits TOOL_SELECT (spec 4.5).
_SNAP_CHOICES = {
    TOOL_TRANSLATE: (SNAP_OFF, SNAP_GRID, SNAP_POINT, SNAP_EDGE,
                     SNAP_SURFACE),
    TOOL_ROTATE: (SNAP_OFF, SNAP_GRID),
}

# The manipulator's on-screen size, in LOGICAL pixels (design spec 8.1).
# The bounds exist because '+' / '-' scale by 10% without an operator
# watching: an unbounded shrink walks the handles down to a point that
# can no longer be grabbed to grow them back.
MANIPULATOR_SIZE_DEFAULT = 90.0
MANIPULATOR_SIZE_MIN = 20.0
MANIPULATOR_SIZE_MAX = 400.0

# The world grid spacing for Move snapping, in world units (snapping
# design 1.8 and 4.5). A session-wide GizmoSettings value beside
# manipulatorSize, not a per-tool field, so Reset(tool) leaves the
# grid where it was: resetting Move must not move the world grid.
GRID_SIZE_DEFAULT = 1.0
GRID_SIZE_MIN = 1e-4
GRID_SIZE_MAX = 1e5

# The fields every ToolSettings carries. Every tool carries all of them
# even where the conventional tool shows only some (Free Rotate is a Rotate-only row),
# so the drag code can read settings.freeRotate without first asking
# which tool it belongs to.
_FIELDS = ("orientation", "stepSnap", "stepSize", "freeRotate",
           "preventNegativeScale", "preserveChildren", "snapMode",
           "groupPivot")


def OrientationLabel(orientation):
    """The menu text for an orientation token."""
    return _ORIENT_LABELS.get(orientation, str(orientation).title())


def OrientationChoices(tool):
    """
    The Axis Orientation entries the conventional tool offers for `tool`, in the conventional own
    order (the default first). Empty for a tool with no manipulator, so
    the panel can simply omit the row.
    """
    return _ORIENT_CHOICES.get(tool, ())


def ToggleLabel(orientation):
    """The toolbar's word for an orientation token."""
    return _TOGGLE_LABELS.get(orientation, OrientationLabel(orientation))


def NextToggleOrientation(orientation):
    """
    What the toolbar's Global/Local toggle moves to from `orientation`.

    Anything outside the pair -- Parent, Gimbal, a value the panel set
    -- goes to World, so the toggle always has somewhere to go and
    always lands somewhere an artist can name.
    """
    if orientation == ORIENT_WORLD:
        return ORIENT_OBJECT
    return ORIENT_WORLD


def GroupPivotLabel(mode):
    """The menu text for a group pivot token."""
    return _GROUP_PIVOT_LABELS.get(mode, str(mode).title())


def GroupPivotShortLabel(mode):
    """The toolbar button's word for a group pivot token."""
    return _GROUP_PIVOT_SHORT.get(mode, GroupPivotLabel(mode))


def NextGroupPivot(mode, tool):
    """
    The next group pivot in `tool`'s cycle, wrapping.

    An unknown or out-of-cycle value lands on the first choice, so the
    hotkey always has somewhere to go; a tool with no choices answers
    with what it was given rather than inventing one.
    """
    choices = GroupPivotChoices(tool)
    if not choices:
        return mode
    try:
        return choices[(choices.index(mode) + 1) % len(choices)]
    except ValueError:
        return choices[0]


def GroupPivotChoices(tool):
    """
    The pivot points offered for `tool`, default first. Empty for
    Select, and for Move -- moving a selection by a world delta is the
    same motion whatever it is measured about, so offering the row there
    would be a control that does nothing.
    """
    if tool in (TOOL_ROTATE, TOOL_SCALE):
        return GROUP_PIVOT_MODES
    return ()


def SnapLabel(snapMode):
    """The menu text for a snap token."""
    return _SNAP_LABELS.get(snapMode, str(snapMode).title())


def SnapChoices(tool):
    """
    The Snap To entries offered for `tool`, in sticky-menu order
    (the default first). Move offers all five; Rotate offers off and
    grid; Scale and Select offer none, so the panel omits both snap
    rows there rather than building an empty combo.
    """
    return _SNAP_CHOICES.get(tool, ())


class ToolSettings(object):
    """
    One tool's options.

    Assigning a field notifies the owning GizmoSettings, which is how a
    checkbox reaches the controller without either of them knowing about
    the other. Writing a field the value it already holds is silent: the
    controller rebuilds its handles on every notification and refreshes
    the panel's widgets from the model, so a self-notifying write would
    be an endless loop the first time a combo box re-emitted its index.
    """

    def __init__(self, tool, owner=None, **values):
        object.__setattr__(self, "tool", tool)
        object.__setattr__(self, "_owner", owner)
        for name in _FIELDS:
            object.__setattr__(self, name, values[name])

    def __setattr__(self, name, value):
        if name in _FIELDS:
            if getattr(self, name) == value:
                return
            object.__setattr__(self, name, value)
            owner = getattr(self, "_owner", None)
            if owner is not None:
                owner.Notify()
            return
        object.__setattr__(self, name, value)

    def CopyFrom(self, other):
        """
        Take every field from `other` in one notification.

        Reset() goes through this rather than replacing the object: the
        panel's widgets and the controller hold references to the
        ToolSettings for their tool, and swapping it out from under them
        would leave them writing into an orphan.
        """
        changed = False
        for name in _FIELDS:
            value = getattr(other, name)
            if getattr(self, name) != value:
                object.__setattr__(self, name, value)
                changed = True
        if changed:
            owner = getattr(self, "_owner", None)
            if owner is not None:
                owner.Notify()
        return changed

    def __repr__(self):
        return "<ToolSettings %s %s>" % (
            self.tool, " ".join("%s=%s" % (n, getattr(self, n))
                                for n in _FIELDS))


def ToolDefaults(tool, owner=None):
    """
    A fresh ToolSettings carrying the conventional defaults for `tool` (design
    spec 8.2 Move, 8.3 Rotate, 8.4 Scale).
    """
    values = {
        "orientation": (OrientationChoices(tool) or (ORIENT_WORLD,))[0],
        "stepSnap": False,
        "stepSize": 15.0 if tool == TOOL_ROTATE else 1.0,
        "freeRotate": True,
        "preventNegativeScale": False,
        "preserveChildren": False,
        "snapMode": SNAP_OFF,
        # The centre, always. A multi-selection turns about its middle
        # until the artist says otherwise; see gizmoMath.GROUP_PIVOT_*.
        "groupPivot": GROUP_PIVOT_CENTER,
    }
    return ToolSettings(tool, owner, **values)


class GizmoSettings(object):
    """
    Every tool's options plus the session-wide manipulator size, with a
    listener list the controller uses to rebuild its handles.

    Session only: nothing here is written to disk, matching the rest of
    the RigExec usdview plugins.
    """

    def __init__(self):
        object.__setattr__(self, "_listeners", [])
        object.__setattr__(
            self, "_tools", dict((t, ToolDefaults(t, self)) for t in TOOLS))
        object.__setattr__(self, "manipulatorSize", MANIPULATOR_SIZE_DEFAULT)
        object.__setattr__(self, "gridSize", GRID_SIZE_DEFAULT)

    def __setattr__(self, name, value):
        if name == "manipulatorSize":
            value = max(MANIPULATOR_SIZE_MIN,
                        min(MANIPULATOR_SIZE_MAX, float(value)))
            if value == self.manipulatorSize:
                return
            object.__setattr__(self, name, value)
            self.Notify()
            return
        if name == "gridSize":
            value = max(GRID_SIZE_MIN,
                        min(GRID_SIZE_MAX, float(value)))
            if value == self.gridSize:
                return
            object.__setattr__(self, name, value)
            self.Notify()
            return
        object.__setattr__(self, name, value)

    def For(self, tool):
        """
        The live ToolSettings for `tool`, created on demand.

        An unknown tool gets a plain default set rather than a KeyError:
        the panel is rebuilt from whatever tool the controller reports,
        and a tool added later must not be able to crash it.
        """
        settings = self._tools.get(tool)
        if settings is None:
            settings = ToolDefaults(tool, self)
            self._tools[tool] = settings
        return settings

    def Reset(self, tool):
        """Restore the conventional defaults for one tool, in place."""
        self.For(tool).CopyFrom(ToolDefaults(tool))

    def ScaleManipulator(self, factor):
        """
        Grow or shrink the manipulator ('+' / '-', design spec 8.1).
        Returns the size actually taken, after clamping.
        """
        self.manipulatorSize = self.manipulatorSize * factor
        return self.manipulatorSize

    def AddListener(self, fn):
        self._listeners.append(fn)

    def Notify(self):
        for fn in list(self._listeners):
            fn()
