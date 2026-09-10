#
# RigExec usdview gizmo: the toolbar's glyphs.
#
# DRAWN, not loaded. Every icon here is a few QPainterPath strokes rendered at
# whatever size the style asks for, which buys three things a PNG set does not:
# it is crisp at any device pixel ratio, it costs no binary assets in the
# repository, and its colour comes from the palette, so the same glyph reads on
# the dark toolbar and on the blue checked background the stylesheet paints
# behind an active tool.
#
# WHY ICONS AT ALL. QToolBar folds whatever does not fit into an overflow
# chevron, from the end, and the row of text buttons did not fit at usdview's
# default viewport width (~600 logical px) on a platform whose UI font is wider
# than the one the labels were tuned against: Windows with Roboto 10pt wanted
# 856 px and put Snap, Undo, Redo, Settings and Graph where nobody finds them.
# Text width is a property of the platform's font; a glyph's is not. The row is
# ~390 px here and the same everywhere.
#
# The labels are NOT deleted. Every action keeps its text -- for its tooltip,
# for the overflow menu, for accessibility, and for the tests that look actions
# up by name -- and only the button's display style changes.
#
# HOUSE STYLE: a single stroke weight, round caps and joins, drawn inside a
# unit box with a small margin, no fills except where a shape means "solid"
# (the animation key). They are meant to read at 16 px, so each is three or
# four strokes at most; anything more turns to mud at that size.
#
import math

from pxr.Usdviewq.qt import QtCore, QtGui

# The unit box every glyph is drawn in, before scaling to the icon size.
_BOX = 100.0
# Stroke weight in those units. 9 reads as a clean hairline at 16 px and stays
# a deliberate line at 48.
_STROKE = 9.0
_MARGIN = 14.0

# Sizes baked into each QIcon. Qt picks the nearest and scales the rest, so
# covering the sizes a toolbar actually asks for keeps every one exact.
_SIZES = (16, 20, 24, 32, 48)

# Light enough to read on usdview's dark chrome, dark enough to read on the
# #4879b4 highlight the stylesheet paints behind a checked tool.
_INK = QtGui.QColor(226, 230, 235)


def _Pen(color, width=_STROKE):
    pen = QtGui.QPen(color)
    pen.setWidthF(width)
    pen.setCapStyle(QtCore.Qt.RoundCap)
    pen.setJoinStyle(QtCore.Qt.RoundJoin)
    return pen


def _Arrowhead(path, tip, direction, size=22.0):
    """A two-stroke open arrowhead at `tip`, opening against `direction`."""
    dx, dy = direction
    length = max((dx * dx + dy * dy) ** 0.5, 1e-6)
    dx, dy = dx / length, dy / length
    # The two barbs are the direction rotated by +/- 140 degrees, which is open
    # enough to stay legible when the whole glyph is 16 px across.
    for angle in (2.44, -2.44):
        c, s = math.cos(angle), math.sin(angle)
        bx = dx * c - dy * s
        by = dx * s + dy * c
        path.moveTo(tip[0], tip[1])
        path.lineTo(tip[0] + bx * size, tip[1] + by * size)


def _Select():
    """A pointer, the one glyph every application draws the same way."""
    path = QtGui.QPainterPath()
    path.moveTo(30, 16)
    path.lineTo(30, 78)
    path.lineTo(45, 63)
    path.lineTo(55, 84)
    path.lineTo(66, 79)
    path.lineTo(56, 59)
    path.lineTo(76, 57)
    path.closeSubpath()
    return [(path, False)]


def _Move():
    """Four arrows from a centre: the translate manipulator, seen head on."""
    path = QtGui.QPainterPath()
    path.moveTo(50, _MARGIN)
    path.lineTo(50, _BOX - _MARGIN)
    path.moveTo(_MARGIN, 50)
    path.lineTo(_BOX - _MARGIN, 50)
    for tip, direction in (((50, _MARGIN), (0, -1)),
                           ((50, _BOX - _MARGIN), (0, 1)),
                           ((_MARGIN, 50), (-1, 0)),
                           ((_BOX - _MARGIN, 50), (1, 0))):
        _Arrowhead(path, tip, direction, 18.0)
    return [(path, False)]


def _Rotate():
    """A ring with a gap and an arrowhead: the rotate manipulator's ring."""
    path = QtGui.QPainterPath()
    box = QtCore.QRectF(_MARGIN + 4, _MARGIN + 4,
                        _BOX - 2 * (_MARGIN + 4), _BOX - 2 * (_MARGIN + 4))
    # 300 degrees, leaving the gap the arrowhead sits in.
    path.arcMoveTo(box, 60)
    path.arcTo(box, 60, -300)
    _Arrowhead(path, (path.currentPosition().x(), path.currentPosition().y()),
               (0.5, -0.87), 20.0)
    return [(path, False)]


def _Scale():
    """A small box and a large box on a diagonal: one thing, two sizes."""
    path = QtGui.QPainterPath()
    path.addRect(QtCore.QRectF(_MARGIN, 56, 30, 30))
    path.addRect(QtCore.QRectF(56, _MARGIN, 30, 30))
    path.moveTo(46, 68)
    path.lineTo(68, 46)
    return [(path, False)]


def _Pose():
    """
    Two joints and the bone between them: the thing a pose drag moves.

    Not a key on a channel, which is what this was first drawn as -- at 16 px
    that is a diamond with whiskers, and the write-mode button two groups along
    is already a diamond. Two glyphs that differ only in their whiskers are one
    glyph as far as a reader is concerned.
    """
    path = QtGui.QPainterPath()
    path.moveTo(34, 66)
    path.lineTo(66, 34)
    joints = QtGui.QPainterPath()
    joints.addEllipse(QtCore.QPointF(26, 74), 12, 12)
    joints.addEllipse(QtCore.QPointF(74, 26), 12, 12)
    return [(path, False), (joints, False)]


def _Pivot():
    """A crosshair with an open centre: the offset the avars ride on."""
    path = QtGui.QPainterPath()
    path.moveTo(50, _MARGIN)
    path.lineTo(50, 34)
    path.moveTo(50, 66)
    path.lineTo(50, _BOX - _MARGIN)
    path.moveTo(_MARGIN, 50)
    path.lineTo(34, 50)
    path.moveTo(66, 50)
    path.lineTo(_BOX - _MARGIN, 50)
    path.addEllipse(QtCore.QPointF(50, 50), 16, 16)
    return [(path, False)]


def _Animation():
    """A solid key: Maya's filled keyframe diamond."""
    path = QtGui.QPainterPath()
    path.moveTo(50, 20)
    path.lineTo(80, 50)
    path.lineTo(50, 80)
    path.lineTo(20, 50)
    path.closeSubpath()
    return [(path, True)]


def _Default():
    """The same key, hollow: a default is the value with no key on it."""
    path = QtGui.QPainterPath()
    path.moveTo(50, 22)
    path.lineTo(78, 50)
    path.lineTo(50, 78)
    path.lineTo(22, 50)
    path.closeSubpath()
    return [(path, False)]


def _UndoPath(mirrored):
    """An arrow curving back on itself; mirrored for redo."""
    path = QtGui.QPainterPath()
    box = QtCore.QRectF(20, 30, 60, 46)
    path.arcMoveTo(box, 180)
    path.arcTo(box, 180, -200)
    path.moveTo(20, 53)
    path.lineTo(20, 24)
    _Arrowhead(path, (20, 24), (0, -1), 20.0)
    if mirrored:
        transform = QtGui.QTransform().translate(_BOX, 0).scale(-1, 1)
        path = transform.map(path)
    return [(path, False)]


def _Settings():
    """Three sliders. A gear at 16 px is a smudge; sliders stay readable."""
    path = QtGui.QPainterPath()
    for y, knob in ((30, 66), (50, 38), (70, 58)):
        path.moveTo(_MARGIN, y)
        path.lineTo(_BOX - _MARGIN, y)
        path.addEllipse(QtCore.QPointF(knob, y), 8, 8)
    return [(path, False)]


def _Graph():
    """An axis pair and a curve: the graph editor, in miniature."""
    path = QtGui.QPainterPath()
    path.moveTo(_MARGIN, _MARGIN)
    path.lineTo(_MARGIN, _BOX - _MARGIN)
    path.lineTo(_BOX - _MARGIN, _BOX - _MARGIN)
    curve = QtGui.QPainterPath()
    curve.moveTo(26, 72)
    curve.cubicTo(46, 72, 44, 28, 62, 28)
    curve.cubicTo(74, 28, 74, 50, 86, 50)
    return [(path, False), (curve, False)]


# name -> the strokes it is made of, as (path, filled) pairs.
_GLYPHS = {
    "select": _Select,
    "move": _Move,
    "rotate": _Rotate,
    "scale": _Scale,
    "pose": _Pose,
    "pivot": _Pivot,
    "animation": _Animation,
    "default": _Default,
    "undo": lambda: _UndoPath(False),
    "redo": lambda: _UndoPath(True),
    "settings": _Settings,
    "graph": _Graph,
}

_cache = {}


def _Pixmap(name, size, color):
    pixmap = QtGui.QPixmap(size, size)
    pixmap.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(pixmap)
    painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
    scale = size / _BOX
    painter.scale(scale, scale)
    for path, filled in _GLYPHS[name]():
        if filled:
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(color)
        else:
            painter.setPen(_Pen(color))
            painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawPath(path)
    painter.end()
    return pixmap


def Icon(name, color=None):
    """
    The named glyph as a QIcon, cached.

    Cached because a toolbar asks for the same icon on every style change and
    every repaint of the overflow menu, and drawing is cheap but not free.
    """
    color = _INK if color is None else color
    key = (name, color.rgba())
    if key in _cache:
        return _cache[key]
    icon = QtGui.QIcon()
    for size in _SIZES:
        icon.addPixmap(_Pixmap(name, size, color))
    _cache[key] = icon
    return icon


def Names():
    """Every glyph this module can draw, for the contact sheet and tests."""
    return sorted(_GLYPHS)
