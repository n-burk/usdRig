



































import os

from pxr.Usdviewq.qt import QtCore, QtGui

_ART_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "icons")
_SIZES = (16, 20, 24, 32)
_INK = QtGui.QColor(226, 230, 235)

# One hue per kind of opinion, used for the row glyph and nowhere else,
# so the colour means the same thing everywhere it appears.
KIND_COLORS = {
    "attribute": QtGui.QColor(110, 170, 240),
    "relationship": QtGui.QColor(240, 180, 90),
    "connection": QtGui.QColor(120, 210, 170),
    "info": QtGui.QColor(170, 170, 185),
    "arcItem": QtGui.QColor(200, 140, 240),
    "sublayer": QtGui.QColor(200, 140, 240),
    "relocate": QtGui.QColor(200, 140, 240),
    "variantSelection": QtGui.QColor(240, 130, 160),
    "layerInfo": QtGui.QColor(200, 140, 240),
}

# Which glyph marks which row kind.
KIND_GLYPHS = {
    "attribute": "attribute",
    "relationship": "relationship",
    "connection": "connection",
    "info": "metadata",
    "arcItem": "arc",
    "sublayer": "layer",
    "relocate": "arc",
    "variantSelection": "variant",
    "layerInfo": "arc",
}

_cache = {}


def _Pixmap(path, size, color):
    source = QtGui.QPixmap(path)
    if source.isNull():
        return None
    scaled = source.scaled(size, size, QtCore.Qt.KeepAspectRatio,
                           QtCore.Qt.SmoothTransformation)
    tinted = QtGui.QPixmap(scaled.size())
    tinted.fill(QtCore.Qt.transparent)
    painter = QtGui.QPainter(tinted)
    painter.drawPixmap(0, 0, scaled)
    # SourceIn keeps the art's alpha and replaces its colour.
    painter.setCompositionMode(QtGui.QPainter.CompositionMode_SourceIn)
    painter.fillRect(tinted.rect(), color)
    painter.end()
    return tinted


def Icon(name, color=None):
    """The named glyph tinted `color`, as a cached QIcon (empty if absent)."""
    color = _INK if color is None else QtGui.QColor(color)
    key = (name, color.rgba())
    if key in _cache:
        return _cache[key]
    icon = QtGui.QIcon()
    path = os.path.join(_ART_DIR, "%s.png" % name)
    if os.path.isfile(path):
        for size in _SIZES:
            pixmap = _Pixmap(path, size, color)
            if pixmap is not None:
                icon.addPixmap(pixmap)
    _cache[key] = icon
    return icon


def KindIcon(kind):
    """The glyph for one row kind, in that kind's colour."""
    glyph = KIND_GLYPHS.get(kind)
    if glyph is None:
        return QtGui.QIcon()
    return Icon(glyph, KIND_COLORS.get(kind, _INK))
