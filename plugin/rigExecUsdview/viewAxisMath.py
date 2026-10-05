#
# RigExec usdview view axis: orientation math, with no Qt.
#
# A navigation axis gizmo: the world X, Y and Z
# axes as coloured balls around a circle, drawn back to front, with the
# negative axes as fainter balls opposite them. Clicking a ball looks
# down that axis, dragging anywhere inside the circle tumbles the view.
#
# Sizes are relative to an 80 px gizmo: the depth fade toward the
# viewport background, the ball growing slightly toward the viewer, the
# current axis's ball switching to its opposite. The code here is our
# own. Every axis is WORLD space,
# read straight off the camera's frustum, so the gizmo agrees with the
# stage's own axes and with the manipulators whatever the up axis.
#
import math

from pxr import Gf


# The gizmo's diameter in logical pixels, and everything sized off it.
GIZMO_SIZE = 80.0
RADIUS = GIZMO_SIZE / 2.0
HANDLE_SIZE = 0.20                 # ball radius, as a fraction of RADIUS
LINE_WIDTH = GIZMO_SIZE / 40.0
RING_WIDTH = GIZMO_SIZE / 60.0
TEXT_SIZE = RADIUS * HANDLE_SIZE * 1.25
DEPTH_BIAS = 0.01
# Room around the circle for the ball rims, and the offset from the
# viewport's bottom-left corner. The bottom offset clears the two lines
# of usdview's HUD that sit in that corner.
WIDGET_SIZE = GIZMO_SIZE + 8.0
MARGIN = 12.0
BOTTOM_MARGIN = 40.0

AXIS_COLORS = {
    "x": (1.0, 0.2, 0.322, 1.0),
    "y": (0.545, 0.863, 0.0, 1.0),
    "z": (0.157, 0.565, 1.0, 1.0),
}
# The backdrop drawn under the cursor.
HIGHLIGHT_COLOR = (1.0, 1.0, 1.0, 0.15)

_AXES = {"x": Gf.Vec3d(1, 0, 0), "y": Gf.Vec3d(0, 1, 0),
         "z": Gf.Vec3d(0, 0, 1)}


class Handle(object):
    """One axis ball: name ("+x", "-y", ...), screen position and depth.

    `depth` is the axis direction's component toward the viewer, in
    [-1, 1]; positive is in front of the circle's plane.
    """

    def __init__(self, axis, positive, x, y, depth, screen):
        self.axis = axis
        self.positive = positive
        self.x = x
        self.y = y
        self.depth = depth
        self.screen = screen

    @property
    def name(self):
        return ("+" if self.positive else "-") + self.axis

    @property
    def label(self):
        return self.axis.upper() if self.positive else \
            "-" + self.axis.upper()

    @property
    def direction(self):
        d = _AXES[self.axis]
        return d if self.positive else -d


def Handles(basis, centre, radius=RADIUS):
    """The six balls in draw order, back to front.

    `basis` is a viewCubeMath.Basis (world right, up and view vectors).
    """
    handles = []
    for axis in "xyz":
        coords = basis.ToView(_AXES[axis])
        for positive in (False, True):
            sign = 1.0 if positive else -1.0
            x, y, depth = coords[0] * sign, coords[1] * sign, \
                coords[2] * sign
            reach = radius * (1.0 - HANDLE_SIZE)
            screen = (centre[0] + x * reach, centre[1] - y * reach)
            handles.append(Handle(axis, positive, x, y, depth, screen))
    handles.sort(key=lambda h: h.depth)
    return handles


def AlignedAxis(handles):
    """The axis the view looks straight down, or None."""
    for handle in handles:
        if handle.x * handle.x + handle.y * handle.y < 1e-6:
            return handle.axis
    return None


def IsBehind(handle):
    bias = DEPTH_BIAS * (-1.0 if handle.positive else 1.0)
    return handle.depth <= bias


def HitTest(basis, centre, point, radius=RADIUS):
    """What a widget-local point is over.

    A Handle, "background" inside the circle but off every ball, or None
    outside the circle. The nearest ball wins, so the whole circle is
    divided between them; when the view looks straight down an axis, the
    ball facing the viewer is skipped and its opposite takes the click,
    since looking down the axis you already look down would do nothing.
    """
    dx = (point[0] - centre[0]) / radius
    dy = (point[1] - centre[1]) / radius
    if dx * dx + dy * dy > 1.0:
        return None
    best, bestDistance = None, float("inf")
    for handle in Handles(basis, centre, radius):
        if handle.x * handle.x + handle.y * handle.y < 1e-6 and \
                handle.depth > 0.0:
            continue
        hx, hy = handle.x, -handle.y
        distance = (hx - dx) ** 2 + (hy - dy) ** 2
        if distance < bestDistance:
            best, bestDistance = handle, distance
    return best if best is not None else "background"


def Mix(a, b, t):
    """a toward b by t, per channel."""
    return tuple(a[i] + (b[i] - a[i]) * t for i in range(4))


def HandleStyle(handle, aligned, background):
    """How to draw a ball: a dict of colours, sizes and flags.

    `background` is the viewport's clear colour (rgba). Front balls take
    the full axis colour and back ones fade halfway to the background;
    negative balls are hollow rings of that colour, unless their axis is
    the one being looked down, when the ring is filled and outlined
    toward white.
    """
    color = AXIS_COLORS[handle.axis]
    fading = Mix(background, color, (handle.depth + 1.0) * 0.25 + 0.5)
    middle = Mix(background, color, 0.75)
    behind = IsBehind(handle)
    alignedFront = aligned == handle.axis and not behind
    alignedBack = aligned == handle.axis and behind
    inner, outline = fading, fading
    if not handle.positive:
        alpha = min(handle.depth + 1.0, 1.0)
        if alignedFront:
            outline = Mix((1.0, 1.0, 1.0, 1.0), color, 0.5)[:3] + (alpha,)
        else:
            inner = Mix(background, color, 0.25)[:3] + (alpha,)
    scale = (handle.depth + 1.0) * 0.08 + 0.92
    return {
        "inner": inner,
        "outline": outline,
        "lineStart": middle,
        "lineEnd": fading,
        "drawLine": handle.positive or aligned is not None,
        "drawBall": not alignedBack,
        "drawLabel": not alignedBack and (handle.positive or
                                          aligned == handle.axis),
        "ballRadius": RADIUS * HANDLE_SIZE * scale,
    }


def ViewDirectionForHandle(handle):
    """The world direction from the orbit centre to the camera.

    Clicking +X puts the camera on +X looking back toward the origin, so
    the view is down -X: the "right" view.
    """
    return Gf.Vec3d(handle.direction)
