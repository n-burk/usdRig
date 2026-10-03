"""The pseudo-3D head the 2D art is turned with.

Every art point is lifted onto a 3D proxy of the head -- a face that is a
nearly flat front plane rounding off to its contour, a projecting nose, a
cheekbone ridge and a temple hollow, the ears at the sides, the hair a layer
in front of the skull and the back hair behind it -- rotated, and projected
orthographically again. Two refinements make it read as a solid head rather
than a decal on a balloon:

* the NEAR side of the face stretches back to the ear (its contour points
  are treated as lying at the side of the head), so the near jaw line runs
  to the ear;
* the FAR side is closed by the silhouette of the front surface: points that
  would rotate past it are squashed onto it, so the far cheek flattens and
  shows the cheekbone bump and temple notch instead of folding over.

The head cage's Turn/Nod keyforms are least-squares fits of this field; the
face meshes also get correctives = (this field) - (what the cage does), so
their silhouette and the nose are exact at the keyform angles.
"""

import math

import numpy as np

import design as D
from paint import smooth

NOD_PIVOT_Y = -3.0
FADE = (-15.0, -10.5)             # displacement fades out down the neck


def face_depth(x, y):
    """Front-surface depth of the face (model units in front of the neck
    axis) at (x, y); valid inside the face outline, continued outside."""
    x = np.asarray(x, np.float64)
    y = np.asarray(y, np.float64)
    a = np.maximum(D.face_halfwidth(np.clip(y, -9.99, 9.8)), 1.0)
    # the skull above the ears is wider than the face outline says
    a = np.where(y > 4.0, np.maximum(a, 7.8 * np.sqrt(np.clip(1 - ((y - 1.0) / 11.5) ** 2, 0.05, 1))), a)
    r = np.abs(x) / a
    dc = np.interp(y, [-12.0, -10.0, -8.0, -6.7, -4.6, -2.0, -0.8, 1.5, 5.0, 8.5, 11.0],
                   [3.2, 6.1, 6.8, 7.1, 7.3, 6.9, 6.6, 7.1, 6.9, 5.2, 2.0])
    rc = np.minimum(r, 1.0)
    g = 0.2 * rc ** 2 + 0.8 * np.clip((rc - 0.55) / 0.45, 0, None) ** 2
    # past the outline the surface keeps falling away, gently (sides of the head)
    d = dc - 4.0 * g - 2.2 * np.clip(r - 1.0, 0, 1.5)
    edge = smooth(0.6, 0.98, r)
    d += 0.55 * np.exp(-((y + 1.9) / 1.3) ** 2) * edge          # cheekbone ridge
    d -= 0.45 * np.exp(-((y - 1.3) / 1.0) ** 2) * edge          # temple hollow
    # the nose, the lips, the chin
    d += 1.9 * np.exp(-((x - 0.1) / 0.75) ** 2 - ((y + 4.3) / 1.1) ** 2) * (y < -1.0)
    d += 0.9 * np.exp(-((x - 0.1) / 0.55) ** 2 - ((y + 3.0) / 1.6) ** 2)
    d += 0.45 * np.exp(-(x / 1.6) ** 2 - ((y + 6.9) / 0.7) ** 2)
    d += 0.20 * np.exp(-(x / 3.0) ** 2 - ((y + 9.3) / 1.2) ** 2)
    return d


def proxy_z(x, y, kind):
    """Depth of an art point on the proxy for each kind of layer."""
    x = np.asarray(x, np.float64)
    y = np.asarray(y, np.float64)
    if kind in ("face", "feature"):
        return face_depth(x, y)
    if kind == "ear":
        return np.full_like(x, 0.6)
    if kind == "front_hair":
        base = face_depth(np.clip(x, -7.2, 7.2), np.maximum(y, -2.0))
        side = np.clip((np.abs(x) - 7.2) / 2.5, 0, 1)
        return base * (1 - side) + side * 0.4 + 1.1
    if kind == "side_hair":
        side = np.clip((np.abs(x) - 5.0) / 4.0, 0, 1)
        return face_depth(np.clip(x, -7.0, 7.0), np.clip(y, -9.0, 9.0)) * (1 - side) + side * 0.6 + 0.8
    if kind == "back_hair":
        top = np.clip((y - 4.0) / 8.0, 0, 1)
        return -2.8 + top * 2.2 - 0.1 * np.abs(x)
    if kind == "neck":
        # the top of the neck (under the chin) rides with the jaw
        return 2.6 + 3.4 * smooth(-12.5, -9.6, y)
    return np.zeros_like(x)


def head_field(xy, kind, ax, ay, envelope=None, soft=False):
    """Displacement (N x 2) of the points ``xy`` of a layer of ``kind`` for
    a head turned ``ax`` degrees (+ = the face turns to screen right) and
    nodded ``ay`` degrees (+ = up)."""
    xy = np.asarray(xy, np.float64)
    x, y = xy[:, 0], xy[:, 1]
    th = math.radians(ax)
    ps = math.radians(ay)
    Z = proxy_z(x, y, kind)
    if kind in ("face", "feature") and ax:
        # the near side stretches back toward the ear; in the SOFT variant
        # (what the head cage is fitted to) the far side rolls off the same
        # way, which keeps the warp smooth and invertible -- the per-mesh
        # correctives then add the exact far silhouette
        a = np.maximum(D.face_halfwidth(np.clip(y, -9.99, 9.8)), 1.0)
        r = np.abs(x) / a
        near = ((x * th) < 0) | soft
        w = smooth(0.50, 1.02, r) * near
        Z = Z * (1 - w) + 0.6 * w
    xr = x * math.cos(th) + Z * math.sin(th)
    Zr = -x * math.sin(th) + Z * math.cos(th)
    if envelope is not None and ax:
        # far side: points past the silhouette's source would fold back behind
        # it; squash them onto the silhouette instead
        env, src = envelope
        far = (x * th) > 0
        past = far & (np.abs(x) > np.abs(src(y)))
        xr = np.where(past, env(y) + np.sign(th) * 0.06 * (np.abs(x) - np.abs(src(y))), xr)
    yr = NOD_PIVOT_Y + (y - NOD_PIVOT_Y) * math.cos(ps) + Zr * math.sin(ps)
    fade = smooth(FADE[0], FADE[1], y)
    d = np.stack([(xr - x) * fade, (yr - y) * fade], axis=1)
    return d


def face_envelope(ax):
    """(x'(y), x_src(y)): the far-side silhouette of the face front surface
    at turn ax, and the rest-pose x that produces it."""
    th = math.radians(ax)
    ys = np.linspace(-10.5, 10.5, 211)
    env = np.zeros_like(ys)
    srcx = np.zeros_like(ys)
    for i, y in enumerate(ys):
        a = max(float(D.face_halfwidth(min(max(y, -9.99), 9.8))), 1.0)
        xs = np.linspace(0, a * 1.08, 120) * np.sign(th)
        Z = face_depth(xs, np.full_like(xs, y))
        xr = xs * math.cos(th) + Z * math.sin(th)
        k = int(np.argmax(xr)) if th > 0 else int(np.argmin(xr))
        env[i] = xr[k]
        srcx[i] = xs[k]
    return (lambda y: np.interp(y, ys, env)), (lambda y: np.interp(y, ys, srcx))
