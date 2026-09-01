#
# RigExec usdview gizmo: the per-tool settings model behind the Tool
# Settings panel (design spec section 8.6).
#
# Deliberately Qt-free, and deliberately not a dict: the panel, the
# controller and the drag code all read the same fields, and a typo in a
# dict key would be a silent wrong default rather than an AttributeError.
# Keeping it out of gizmoUI.py is what lets "what does the Move tool
# start as" be tested headlessly, which is the part of Maya parity a
# reviewer can actually check.
#
# The tool tokens come from gizmoScreen rather than being restated here
# so there is exactly one spelling of "translate" in the plugin;
# gizmoScreen is itself Qt-free (pure Gf), so importing it costs nothing
# a headless test cannot pay.
#
from gizmoScreen import TOOL_TRANSLATE, TOOL_ROTATE, TOOL_SCALE

# gizmoUI re-exports this; it lives here because MayaDefaults and
# OrientationChoices both have to answer for it.
TOOL_SELECT = "select"

TOOLS = (TOOL_SELECT, TOOL_TRANSLATE, TOOL_ROTATE, TOOL_SCALE)

# Maya's Axis Orientation menu. World / Object / Parent for Move and
# Scale; Object / World / Gimbal for Rotate (Maya's Rotate Tool has no
# Parent entry -- its "Gimbal" is the parent-space Euler decomposition).
ORIENT_WORLD = "world"
ORIENT_OBJECT = "object"
ORIENT_PARENT = "parent"
ORIENT_GIMBAL = "gimbal"

_ORIENT_LABELS = {
    ORIENT_WORLD: "World",
    ORIENT_OBJECT: "Object",
    ORIENT_PARENT: "Parent",
    ORIENT_GIMBAL: "Gimbal",
}

_ORIENT_CHOICES = {
    TOOL_TRANSLATE: (ORIENT_WORLD, ORIENT_OBJECT, ORIENT_PARENT),
    TOOL_SCALE: (ORIENT_WORLD, ORIENT_OBJECT, ORIENT_PARENT),
    TOOL_ROTATE: (ORIENT_OBJECT, ORIENT_WORLD, ORIENT_GIMBAL),
}

# The manipulator's on-screen size, in LOGICAL pixels (design spec 8.1).
# The bounds exist because '+' / '-' scale by 10% without an operator
# watching: an unbounded shrink walks the handles down to a point that
# can no longer be grabbed to grow them back.
MANIPULATOR_SIZE_DEFAULT = 90.0
MANIPULATOR_SIZE_MIN = 20.0
MANIPULATOR_SIZE_MAX = 400.0

# The fields every ToolSettings carries. Every tool carries all of them
# even where Maya shows only some (Free Rotate is a Rotate-only row),
# so the drag code can read settings.freeRotate without first asking
# which tool it belongs to.
_FIELDS = ("orientation", "stepSnap", "stepSize", "freeRotate",
           "preventNegativeScale", "preserveChildren")


def OrientationLabel(orientation):
    """The menu text for an orientation token."""
    return _ORIENT_LABELS.get(orientation, str(orientation).title())


def OrientationChoices(tool):
    """
    The Axis Orientation entries Maya offers for `tool`, in Maya's own
    order (the default first). Empty for a tool with no manipulator, so
    the panel can simply omit the row.
    """
    return _ORIENT_CHOICES.get(tool, ())


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


def MayaDefaults(tool, owner=None):
    """
    A fresh ToolSettings carrying Maya's defaults for `tool` (design
    spec 8.2 Move, 8.3 Rotate, 8.4 Scale).
    """
    values = {
        "orientation": (OrientationChoices(tool) or (ORIENT_WORLD,))[0],
        "stepSnap": False,
        "stepSize": 15.0 if tool == TOOL_ROTATE else 1.0,
        "freeRotate": True,
        "preventNegativeScale": False,
        "preserveChildren": False,
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
            self, "_tools", dict((t, MayaDefaults(t, self)) for t in TOOLS))
        object.__setattr__(self, "manipulatorSize", MANIPULATOR_SIZE_DEFAULT)

    def __setattr__(self, name, value):
        if name == "manipulatorSize":
            value = max(MANIPULATOR_SIZE_MIN,
                        min(MANIPULATOR_SIZE_MAX, float(value)))
            if value == self.manipulatorSize:
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
            settings = MayaDefaults(tool, self)
            self._tools[tool] = settings
        return settings

    def Reset(self, tool):
        """Restore Maya's defaults for one tool, in place."""
        self.For(tool).CopyFrom(MayaDefaults(tool))

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
