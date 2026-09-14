#
# RigExec usdview gizmo: the toolbar's glyphs.
#
# ART FIRST, DRAWN AS A FALLBACK. The set under icons/ is the artwork -- white
# line-art on transparency, one file per glyph, normalised to a common extent
# so the row reads as a family (see tools/bakeGizmoIcons.py, which is what
# normalises them). It is tinted here rather than shipped in the toolbar's
# colours, so one file works on the dark chrome and on the blue checked
# background the stylesheet paints behind an active tool.
#
# Every glyph also exists as a few QPainterPath strokes below. That is not a
# second implementation of the same thing -- it is what the toolbar falls back
# to when a file is missing, because the alternative failure is a row of blank
# buttons, and a toolbar that cannot say what its buttons do is worse than one
# drawn a little more plainly. It costs nothing to keep: no file, no load, and
# it is the same code that drew the first version of this set.
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
import os

from pxr.Usdviewq.qt import QtCore, QtGui

# The artwork, beside this module so it travels with the plugin.
_ART_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")

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
    """A solid key: a filled keyframe diamond."""
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


def _Global():
    """
    A globe: the world axes the manipulator is drawn on in Global mode.

    A circle with one meridian and one parallel, and nothing else. The
    obvious alternative -- three labelled axes -- is four strokes and
    two glyph-sized letters at 16 px, which is mud; a globe reads at a
    glance and is what every DCC uses for "world".
    """
    path = QtGui.QPainterPath()
    radius = (_BOX - 2 * _MARGIN) / 2.0
    path.addEllipse(QtCore.QPointF(50, 50), radius, radius)
    # The meridian: a narrow ellipse on the same centre.
    path.addEllipse(QtCore.QPointF(50, 50), radius * 0.42, radius)
    path.moveTo(50 - radius, 50)
    path.lineTo(50 + radius, 50)
    return [(path, False)]


def _Local():
    """
    A box with one corner's axes drawn on it: an object's OWN frame.

    Deliberately the same two-stroke corner the world glyph does not
    have, so the pair reads as "everything" against "this one thing"
    rather than as two abstract diagrams.
    """
    box = QtGui.QPainterPath()
    box.addRect(QtCore.QRectF(_MARGIN, _MARGIN, _BOX - 2 * _MARGIN,
                              _BOX - 2 * _MARGIN))
    axes = QtGui.QPainterPath()
    axes.moveTo(32, 68)
    axes.lineTo(72, 68)
    axes.moveTo(32, 68)
    axes.lineTo(32, 30)
    return [(box, False), (axes, False)]


# Three controls in a triangle: the selection, for the group-pivot
# glyphs below. Kept as one list so the three read as the same picture
# with one thing changed, which is the only way a set of three says
# "same question, different answer" at 16 px.
_GROUP_DOTS = ((28.0, 72.0), (50.0, 30.0), (72.0, 72.0))
_GROUP_DOT_R = 10.0
_GROUP_MARK_R = 7.0


def _GroupRing(dots):
    path = QtGui.QPainterPath()
    for x, y in dots:
        path.addEllipse(QtCore.QPointF(x, y), _GROUP_DOT_R, _GROUP_DOT_R)
    return path


def _GroupCentre():
    """Three controls with the pivot on their centroid -- the default."""
    mark = QtGui.QPainterPath()
    cx = sum(d[0] for d in _GROUP_DOTS) / 3.0
    cy = sum(d[1] for d in _GROUP_DOTS) / 3.0
    mark.addEllipse(QtCore.QPointF(cx, cy), _GROUP_MARK_R, _GROUP_MARK_R)
    return [(_GroupRing(_GROUP_DOTS), False), (mark, True)]


def _GroupLead():
    """The same three, with the pivot on the LAST-selected one."""
    mark = QtGui.QPainterPath()
    x, y = _GROUP_DOTS[2]
    mark.addEllipse(QtCore.QPointF(x, y), _GROUP_MARK_R, _GROUP_MARK_R)
    return [(_GroupRing(_GROUP_DOTS), False), (mark, True)]


def _GroupEach():
    """Every control its own pivot: all three marked, none between."""
    mark = QtGui.QPainterPath()
    for x, y in _GROUP_DOTS:
        mark.addEllipse(QtCore.QPointF(x, y), _GROUP_MARK_R, _GROUP_MARK_R)
    return [(_GroupRing(_GROUP_DOTS), False), (mark, True)]


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
    "global": _Global,
    "local": _Local,
    "groupCentre": _GroupCentre,
    "groupLead": _GroupLead,
    "groupEach": _GroupEach,
}

_cache = {}


def _ArtPixmap(name, size, color):
    """The generated glyph at `size`, tinted `color`, or None if there is none."""
    path = os.path.join(_ART_DIR, "%s.png" % name)
    if not os.path.isfile(path):
        return None
    source = QtGui.QPixmap(path)
    if source.isNull():
        return None
    scaled = source.scaled(size, size, QtCore.Qt.KeepAspectRatio,
                           QtCore.Qt.SmoothTransformation)
    # SourceIn keeps the artwork's alpha and replaces its colour, which is what
    # lets one white file be drawn in whatever ink the chrome needs.
    tinted = QtGui.QPixmap(scaled.size())
    tinted.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(tinted)
    painter.drawPixmap(0, 0, scaled)
    painter.setCompositionMode(QtGui.QPainter.CompositionMode_SourceIn)
    painter.fillRect(tinted.rect(), color)
    painter.end()
    return tinted


def _DrawnPixmap(name, size, color):
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


def _Pixmap(name, size, color):
    art = _ArtPixmap(name, size, color)
    return art if art is not None else _DrawnPixmap(name, size, color)


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
    """Every glyph this module can supply, for the contact sheet and tests."""
    return sorted(_GLYPHS)


def HasArt(name):
    """Whether `name` is served by artwork rather than by the fallback."""
    return os.path.isfile(os.path.join(_ART_DIR, "%s.png" % name))
