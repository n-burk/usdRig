"""The pseudo-3D head turn and nod: the TARGET displacement of every head
point for an angle, from a 3D proxy of the head.

Turn (AngleX, theta > 0 turns the face to screen right): each ROW of the
head is a curved surface of depth d(x) in front of the neck axis; a point
projects to x cos(theta) + d sin(theta). The projection is made MONOTONE
(a running maximum), so the far side never folds over itself -- the part of
the face that turns away is squeezed into the silhouette, and the far
silhouette is whatever projects furthest: the temple notch at eye level
(the eye socket is shallower) and the cheekbone bump come out of the depth
function, not out of a drawn outline. The near side keeps its silhouette
(the side plane of the head rotates into view there). Points outside the
head's surface (hair) continue rigidly, and every layer adds its own depth
offset -- the parallax: the fringe floats in front of the forehead, the
back hair moves the other way.

Nod (AngleY, phi > 0 tips the face DOWN) is the same construction per
COLUMN: the forehead and crown grow, the lower face shortens.
"""

import math

import numpy as np
from scipy.interpolate import RegularGridInterpolator

import design as D
from paint import smooth

YC = -1.0            # the nod pivot height
ROW_Y = np.linspace(-16.0, 16.0, 161)
COL_X = np.linspace(-12.0, 12.0, 121)
NU = 241


def head_R(y):
    """Half width of the head surface at row y: the face outline where
    there is face, an ellipse over the skull above it, the neck below."""
    y = np.asarray(y, np.float64)
    face = D.face_halfwidth(np.clip(y, -9.95, 9.8))
    skull = 7.85 * np.sqrt(np.clip(1 - ((y - 0.6) / 10.1) ** 2, 0.02, 1))
    r = np.where(y > 4.0, np.maximum(face, skull), face)
    r = np.where(y > 9.8, skull, r)
    r = np.where(y < -9.95, 1.2 + 0.0 * y, r)
    return np.maximum(r, 1.2)


def depth(x, y):
    """Depth of the head's front surface in front of the neck axis."""
    x = np.asarray(x, np.float64)
    y = np.asarray(y, np.float64)
    R = head_R(y)
    q = np.clip(np.abs(x) / R, 0, 1)
    D0 = 6.6 - 1.2 * smooth(-6.0, -10.0, y) - 1.2 * smooth(7.0, 11.0, y)
    # upper face: a rounded front meeting the side of the head at the
    # outline; lower face: the jaw is a shallow wedge whose edges (the jaw
    # line) stay well forward of the neck axis
    upper = D0 * (1 - q ** 2) ** 0.55
    edge = 2.2 + 2.7 * smooth(-6.0, -10.0, y)
    lower = D0 - (D0 - edge) * q ** 2
    j = smooth(-3.0, -7.5, y)
    d = upper * (1 - j) + lower * j
    ax = np.abs(x)
    # nose, cheekbones, eye sockets, brow ridge
    d = d + 1.9 * np.exp(-((x - 0.12) / 0.95) ** 2 - ((y + 3.4) / 1.9) ** 2)
    d = d + 0.55 * np.exp(-((ax - 5.9) / 1.2) ** 2 - ((y + 2.0) / 1.3) ** 2)
    d = d - 0.75 * np.exp(-((ax - 4.3) / 1.6) ** 2 - ((y + 0.5) / 1.15) ** 2)
    d = d + 0.25 * np.exp(-((ax - 3.0) / 2.5) ** 2 - ((y - 1.8) / 0.8) ** 2)
    # mouth/chin: the lips a little forward
    d = d + 0.35 * np.exp(-(x / 1.6) ** 2 - ((y + 6.9) / 0.9) ** 2)
    # over the top of the skull the surface turns up and away
    top = np.sqrt(np.clip(1 - (np.maximum(y - 0.6, 0) / 10.3) ** 2, 0, 1)) ** 0.8
    return np.maximum(d * np.where(y > 0.6, top, 1.0), 0.0)


def _row_map(y, theta):
    """(u grid, x' values) for one row: u = x / R in [-1, 1]."""
    R = float(head_R(y))
    u = np.linspace(-1, 1, NU)
    x = u * R
    s, c = math.sin(theta), math.cos(theta)
    p = x * c + depth(x, np.full_like(x, y)) * s
    # monotone: running max (for a turn to the right; mirrored for left)
    if theta >= 0:
        M = np.maximum.accumulate(p)
    else:
        M = np.minimum.accumulate(p[::-1])[::-1]
    # keep a minimum slope so nothing collapses to zero width
    M = M + 0.035 * R * c * u
    # the near side holds its silhouette: the side plane rotates into view
    near = -1.0 if theta >= 0 else 1.0
    edge_shift = (near * R) - M[0 if theta >= 0 else -1]
    w = 1.0 - smooth(0.0, 0.45, np.abs(u - near))
    ext = 0.60 * smooth(-9.0, -5.0, y) * (1 - smooth(1.0, 7.0, y))
    M = M + edge_shift * w * ext
    return u, M, R


def turn_grid(theta_deg):
    th = math.radians(theta_deg)
    F = np.zeros((len(ROW_Y), NU))
    Rs = np.zeros(len(ROW_Y))
    for i, y in enumerate(ROW_Y):
        u, M, R = _row_map(y, th)
        F[i] = M
        Rs[i] = R
    return F, Rs


def turn_displacement(pts, offsets, theta_deg, fade_below=(-10.5, -19.0)):
    """Displacement (N, 2) of rest points `pts` for a turn; `offsets` is each
    point's depth in front of (+) / behind (-) the head surface."""
    th = math.radians(theta_deg)
    F, Rs = turn_grid(theta_deg)
    x, y = pts[:, 0], pts[:, 1]
    yy = np.clip(y, -9.9, 9.2)
    R = np.interp(yy, ROW_Y, Rs)
    u = x / R
    uc = np.clip(u, -1, 1)
    interp = RegularGridInterpolator((ROW_Y, np.linspace(-1, 1, NU)), F)
    xe = interp(np.stack([yy, uc], axis=1))
    # outside the head surface: continue rigidly from the silhouette
    outside = x - uc * R
    xe = xe + outside * math.cos(th)
    dx = xe - x + offsets * math.sin(th)
    # long hair below the head follows less and less
    f = smooth(fade_below[1], fade_below[0], y)
    dx = dx * (0.18 + 0.82 * f)
    # a little vertical arc: the turn dips the far side a hair (perspective)
    dy = -0.04 * np.abs(dx) * smooth(-2.0, 4.0, y)
    return np.stack([dx, dy], axis=1)


def _col_map(x, phi):
    """(v grid, y' values) for one column."""
    ax = abs(x)
    top = 0.6 + 10.1 * math.sqrt(max(1 - (ax / 7.85) ** 2, 0.0004))
    bot = float(np.interp(ax, *_bottom_profile()))
    v = np.linspace(0, 1, NU)
    y = bot + (top - bot) * v
    s, c = math.sin(phi), math.cos(phi)
    d = depth(np.full_like(y, x), y)
    p = YC + (y - YC) * c - d * s
    # monotone increasing in y: looking down the chin's underside folds away
    # (the bottom silhouette is the lowest projection); looking up the
    # crown folds away (the top silhouette is the highest)
    if phi >= 0:
        M = np.minimum.accumulate(p[::-1])[::-1]
    else:
        M = np.maximum.accumulate(p)
    M = M + 0.03 * (top - bot) * c * (v - 0.5)
    return y, M, bot, top


_BP = None


def _bottom_profile():
    global _BP
    if _BP is None:
        o = D.face_outline()
        lo = o[o[:, 1] < -2.0]
        xs = np.linspace(0, 7.5, 76)
        ys = []
        for xv in xs:
            near = lo[np.abs(np.abs(lo[:, 0]) - xv) < 0.25]
            ys.append(near[:, 1].min() if len(near) else -2.0)
        _BP = (xs, np.array(ys))
    return _BP


def nod_grid(phi_deg):
    ph = math.radians(phi_deg)
    F = np.zeros((len(COL_X), NU))
    B = np.zeros(len(COL_X))
    T = np.zeros(len(COL_X))
    for i, x in enumerate(COL_X):
        y, M, bot, top = _col_map(x, ph)
        F[i] = M
        B[i] = bot
        T[i] = top
    return F, B, T


def nod_displacement(pts, offsets, phi_deg, fade_below=(-10.5, -19.0)):
    ph = math.radians(phi_deg)
    F, B, T = nod_grid(phi_deg)
    x, y = pts[:, 0], pts[:, 1]
    xx = np.clip(x, -7.2, 7.2)
    bot = np.interp(xx, COL_X, B)
    top = np.interp(xx, COL_X, T)
    v = (y - bot) / np.maximum(top - bot, 1e-6)
    vc = np.clip(v, 0, 1)
    interp = RegularGridInterpolator((COL_X, np.linspace(0, 1, NU)), F)
    ye = interp(np.stack([xx, vc], axis=1))
    outside = y - (bot + vc * (top - bot))
    ye = ye + outside * math.cos(ph)
    dy = ye - y - offsets * math.sin(ph)
    f = smooth(fade_below[1], fade_below[0], y)
    dy = dy * (0.15 + 0.85 * f)
    # below the chin (the neck region, hidden) keep it continuous
    dx = np.zeros_like(dy)
    return np.stack([dx, dy], axis=1)
