"""A tiny vector-to-texture painting engine for the 2D bust's art layers.

Every art layer of the character is painted here, procedurally, as an RGBA
texture in MODEL space: a layer covers a model-space box at a fixed texel
density, so a texel's model position is known exactly and the layer's mesh
can take its UVs straight from its rest points.

Painting happens supersampled (``ss`` x the final density) in premultiplied
float RGBA, like a paint program's layer: shapes are filled with cv2's
anti-aliased polygon rasteriser, strokes are variable-width polygons built
along Catmull-Rom spines, line art is painted INSIDE a shape's silhouette
from its distance field, and shading uses analytic fields (gradients, cel
bands, clipped shapes). ``finish`` reduces the layer with an area filter,
un-premultiplies, and bleeds colour 8+ texels into the transparent area so
bilinear filtering on the GPU never pulls in a dark fringe.

Deterministic: no randomness except seeded numpy generators.
"""

import math

import cv2
import numpy as np


# colour helpers

def hexc(h):
    h = h.lstrip("#")
    return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4)], np.float32) / 255.0


def mix(a, b, t):
    a = hexc(a) if isinstance(a, str) else np.asarray(a, np.float32)
    b = hexc(b) if isinstance(b, str) else np.asarray(b, np.float32)
    return a + (b - a) * t


def smooth(e0, e1, x):
    t = np.clip((np.asarray(x, np.float64) - e0) / (e1 - e0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def lerp(a, b, t):
    return a + (b - a) * t


# curves

def catmull(points, n=24, closed=False, alpha=0.5):
    """Centripetal Catmull-Rom through ``points`` (N x 2), ``n`` samples per
    segment. Open curves pass through the first and last points."""
    P = np.asarray(points, np.float64)
    if closed:
        P = np.vstack([P[-1:], P, P[:2]])
    else:
        P = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]

        def tj(ti, a, b):
            return ti + max(np.linalg.norm(b - a), 1e-6) ** alpha
        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        last = (i == len(P) - 3) and not closed
        ts = np.linspace(t1, t2, n + 1 if last else n, endpoint=last)[:, None]
        a1 = (t1 - ts) / (t1 - t0) * p0 + (ts - t0) / (t1 - t0) * p1
        a2 = (t2 - ts) / (t2 - t1) * p1 + (ts - t1) / (t2 - t1) * p2
        a3 = (t3 - ts) / (t3 - t2) * p2 + (ts - t2) / (t3 - t2) * p3
        b1 = (t2 - ts) / (t2 - t0) * a1 + (ts - t0) / (t2 - t0) * a2
        b2 = (t3 - ts) / (t3 - t1) * a2 + (ts - t1) / (t3 - t1) * a3
        out.append((t2 - ts) / (t2 - t1) * b1 + (ts - t1) / (t2 - t1) * b2)
    return np.vstack(out)


def arclen(poly):
    d = np.linalg.norm(np.diff(poly, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(d)])


def resample(poly, n=None, spacing=None, closed=False):
    """Uniform arc-length resampling of a polyline."""
    P = np.asarray(poly, np.float64)
    if closed:
        P = np.vstack([P, P[:1]])
    s = arclen(P)
    L = s[-1]
    if n is None:
        n = max(2, int(math.ceil(L / spacing)) + 1)
    u = np.linspace(0, L, n, endpoint=not closed) if not closed else np.linspace(0, L, n, endpoint=False)
    x = np.interp(u, s, P[:, 0])
    y = np.interp(u, s, P[:, 1])
    return np.stack([x, y], axis=1)


def tangents(poly):
    d = np.gradient(np.asarray(poly, np.float64), axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
    return d


def normals(poly):
    """Left normals (tangent rotated +90 degrees)."""
    t = tangents(poly)
    return np.stack([-t[:, 1], t[:, 0]], axis=1)


def ribbon(spine, wl, wr=None):
    """Closed polygon of a variable-width stroke: ``wl``/``wr`` are the
    half-widths on the left/right of the spine (arrays or scalars)."""
    spine = np.asarray(spine, np.float64)
    n = normals(spine)
    wl = np.broadcast_to(np.asarray(wl, np.float64), (len(spine),))
    wr = wl if wr is None else np.broadcast_to(np.asarray(wr, np.float64), (len(spine),))
    left = spine + n * wl[:, None]
    right = spine - n * wr[:, None]
    return np.vstack([left, right[::-1]])


def taper(n, w0, w1=None, head=0.15, tail=0.35, power=1.0, floor=0.0):
    """Width profile along a stroke: rises over ``head`` and falls over
    ``tail`` (fractions of the length) with a pointed (or ``floor``) end."""
    t = np.linspace(0, 1, n)
    w1 = w0 if w1 is None else w1
    w = w0 + (w1 - w0) * t
    up = smooth(0, head, t) if head > 0 else np.ones(n)
    dn = smooth(1.0, 1.0 - tail, t) if tail > 0 else np.ones(n)
    k = np.minimum(up, dn) ** power
    return floor + (w - floor) * k


def star(cx, cy, r_out, r_in, points=4, rot=0.0):
    out = []
    for k in range(points * 2):
        a = rot + k * math.pi / points + math.pi / 2
        r = r_out if k % 2 == 0 else r_in
        out.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return np.array(out)


def ellipse(cx, cy, rx, ry, n=96, rot=0.0):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    x = rx * np.cos(a)
    y = ry * np.sin(a)
    c, s = math.cos(rot), math.sin(rot)
    return np.stack([cx + c * x - s * y, cy + s * x + c * y], axis=1)


# the layer

class Layer(object):
    """An RGBA art layer painted in model space (y up).

    ``bbox`` = (x0, y0, x1, y1) in model units; ``tpu`` final texels per
    unit; painting happens at ``tpu * ss``.
    """

    def __init__(self, name, bbox, tpu=50, ss=3):
        self.name = name
        x0, y0, x1, y1 = bbox
        self.tpu = tpu
        self.ss = ss
        self.W = int(math.ceil((x1 - x0) * tpu)) * ss
        self.H = int(math.ceil((y1 - y0) * tpu)) * ss
        # snap the box to whole final texels
        self.x0, self.y1 = x0, y1
        self.x1 = x0 + self.W / float(tpu * ss)
        self.y0 = y1 - self.H / float(tpu * ss)
        self.s = float(tpu * ss)
        self.col = np.zeros((self.H, self.W, 3), np.float32)
        self.a = np.zeros((self.H, self.W), np.float32)
        self._xy = None
        self.meta = {}

    # -- coordinates ------------------------------------------------------
    @property
    def bbox(self):
        return (self.x0, self.y0, self.x1, self.y1)

    def px(self, pts):
        pts = np.asarray(pts, np.float64)
        return np.stack([(pts[:, 0] - self.x0) * self.s, (self.y1 - pts[:, 1]) * self.s], axis=1)

    def XY(self):
        if self._xy is None:
            xs = self.x0 + (np.arange(self.W) + 0.5) / self.s
            ys = self.y1 - (np.arange(self.H) + 0.5) / self.s
            X, Y = np.meshgrid(xs.astype(np.float32), ys.astype(np.float32))
            self._xy = (X, Y)
        return self._xy

    # -- masks ------------------------------------------------------------
    def poly(self, pts, holes=()):
        m = np.zeros((self.H, self.W), np.uint8)
        polys = [np.round(self.px(pts) * 16).astype(np.int32)]
        cv2.fillPoly(m, polys, 255, cv2.LINE_AA, shift=4)
        out = m.astype(np.float32) / 255.0
        for h in holes:
            out *= 1.0 - self.poly(h)
        return out

    def polys(self, list_of_pts):
        out = np.zeros((self.H, self.W), np.float32)
        for p in list_of_pts:
            out = np.maximum(out, self.poly(p))
        return out

    def stroke(self, spine, wl, wr=None):
        return self.poly(ribbon(spine, wl, wr))

    def disc(self, cx, cy, r):
        X, Y = self.XY()
        d = np.hypot(X - cx, Y - cy)
        return np.clip((r - d) * self.s + 0.5, 0, 1)

    def blur(self, m, sigma_units):
        s = sigma_units * self.s
        k = int(s * 3) * 2 + 1
        return cv2.GaussianBlur(m, (k, k), s)

    def shift(self, m, dx, dy):
        M = np.float32([[1, 0, dx * self.s], [0, 1, -dy * self.s]])
        return cv2.warpAffine(m, M, (self.W, self.H), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    def inside_dist(self, m, thresh=0.5):
        """Distance (model units) from each inside texel to the silhouette of
        mask ``m``; sub-texel accurate enough for AA'd line art."""
        b = (m >= thresh).astype(np.uint8)
        d = cv2.distanceTransform(b, cv2.DIST_L2, 5).astype(np.float32)
        # centre the 0.5 crossing: a texel just inside is ~0.5 texel from the edge
        d = np.where(b > 0, d - 0.5, 0.0)
        return d / self.s

    def outside_dist(self, m, thresh=0.5):
        b = (m < thresh).astype(np.uint8)
        d = cv2.distanceTransform(b, cv2.DIST_L2, 5).astype(np.float32)
        d = np.where(b > 0, d - 0.5, 0.0)
        return d / self.s

    def band_inside(self, m, width):
        """Coverage of an inner band of (per-texel) ``width`` model units
        along the silhouette of ``m`` -- line art painted inside a shape."""
        d = self.inside_dist(m)
        w = np.asarray(width, np.float32)
        cov = np.clip((w - d) * self.s + 0.5, 0, 1)
        return cov * m

    # -- painting ---------------------------------------------------------
    def _rgb(self, color):
        if isinstance(color, str):
            color = hexc(color)
        c = np.asarray(color, np.float32)
        return c

    def paint(self, mask, color, alpha=1.0):
        """Normal blend of ``color`` through ``mask`` * ``alpha``."""
        a = np.clip(mask * alpha, 0, 1).astype(np.float32)
        c = self._rgb(color)
        if c.ndim == 1:
            c = c[None, None, :]
        self.col = self.col * (1 - a)[..., None] + c * a[..., None]
        self.a = self.a * (1 - a) + a

    def paint_in(self, mask, color, alpha=1.0):
        """Paint clipped to what is already on the layer (clipping mask)."""
        a = np.clip(mask * alpha, 0, 1).astype(np.float32)
        c = self._rgb(color)
        if c.ndim == 1:
            c = c[None, None, :]
        # premultiplied: replace a fraction a of the colour, keep coverage
        self.col = self.col * (1 - a)[..., None] + c * (a * self.a)[..., None]

    def multiply_in(self, mask, color, alpha=1.0):
        """Multiply blend, clipped to the layer."""
        a = np.clip(mask * alpha, 0, 1).astype(np.float32)[..., None]
        c = self._rgb(color)
        if c.ndim == 1:
            c = c[None, None, :]
        self.col = self.col * (1 - a) + self.col * c * a

    def screen_in(self, mask, color, alpha=1.0):
        a = np.clip(mask * alpha, 0, 1).astype(np.float32)[..., None]
        c = self._rgb(color)
        if c.ndim == 1:
            c = c[None, None, :]
        al = self.a[..., None]
        straight = self.col / np.maximum(al, 1e-6)
        scr = 1 - (1 - straight) * (1 - c)
        self.col = self.col * (1 - a) + scr * al * a

    def erase(self, mask):
        k = (1 - np.clip(mask, 0, 1)).astype(np.float32)
        self.col *= k[..., None]
        self.a *= k

    def coverage(self):
        return self.a.copy()

    # -- output -----------------------------------------------------------
    def finish(self, bleed=10):
        """Final-resolution straight-alpha RGBA uint8 (H x W x 4)."""
        ss = self.ss
        W, H = self.W // ss, self.H // ss
        col = cv2.resize(self.col, (W, H), interpolation=cv2.INTER_AREA)
        a = cv2.resize(self.a, (W, H), interpolation=cv2.INTER_AREA)
        straight = col / np.maximum(a[..., None], 1e-6)
        straight = np.where(a[..., None] > 1e-4, straight, 0)
        # bleed colour outward so bilinear sampling never pulls black in
        straight = bleed_colour(straight, a, bleed)
        rng = np.random.default_rng(7)
        dither = (rng.random(straight.shape, np.float32) + rng.random(straight.shape, np.float32) - 1.0) / 255.0 * 0.5
        rgb = np.clip(straight + dither, 0, 1)
        out = np.zeros((H, W, 4), np.uint8)
        out[..., :3] = np.round(rgb * 255).astype(np.uint8)
        out[..., 3] = np.round(np.clip(a, 0, 1) * 255).astype(np.uint8)
        return out

    def preview_rgba(self):
        """Supersampled straight RGBA float, for compositing previews."""
        straight = self.col / np.maximum(self.a[..., None], 1e-6)
        return np.dstack([np.clip(straight, 0, 1), self.a])


def bleed_colour(rgb, a, iters=10):
    """Push colour from covered texels into uncovered ones (edge padding)."""
    rgb = rgb.copy()
    known = (a > 0.02).astype(np.float32)
    if known.min() > 0:
        return rgb
    k = np.ones((3, 3), np.float32)
    cur = rgb * known[..., None]
    w = known.copy()
    for _ in range(iters):
        s = cv2.filter2D(cur, -1, k, borderType=cv2.BORDER_CONSTANT)
        n = cv2.filter2D(w, -1, k, borderType=cv2.BORDER_CONSTANT)
        grow = (w == 0) & (n > 0)
        if not grow.any():
            break
        cur[grow] = s[grow] / n[grow][:, None]
        w[grow] = 1.0
    # anything still unknown: average colour of the layer
    if (w == 0).any() and (w > 0).any():
        mean = cur[w > 0].mean(axis=0)
        cur[w == 0] = mean
    return cur


# strand fields (hair clumps, lashes, brows)

def strand_coords(layer, spine, half_width, region=None):
    """Per-texel (t, v, d) for a strand: t in [0, 1] along the spine (arc
    length), v the signed distance across in units of the local half width
    (+ = left of the spine direction), d the signed distance in model units.
    ``region`` (bool mask) limits the work."""
    from scipy.spatial import cKDTree
    spine = resample(spine, n=max(64, int(arclen(spine)[-1] * layer.s / 2)))
    L = arclen(spine)
    T = L / L[-1]
    N = normals(spine)
    hw = np.interp(T, np.linspace(0, 1, len(half_width)), half_width) if np.ndim(half_width) else np.full(len(T), half_width)
    X, Y = layer.XY()
    if region is None:
        region = np.ones(X.shape, bool)
    idx = np.nonzero(region)
    pts = np.stack([X[idx], Y[idx]], axis=1)
    tree = cKDTree(spine)
    _, k = tree.query(pts)
    rel = pts - spine[k]
    d = (rel * N[k]).sum(axis=1)
    along = (rel * tangents(spine)[k]).sum(axis=1)
    t = np.clip(T[k] + along / L[-1], 0, 1)
    v = d / np.maximum(hw[k], 1e-4)
    Tm = np.full(X.shape, -1.0, np.float32)
    Vm = np.full(X.shape, 9.0, np.float32)
    Dm = np.full(X.shape, 9.0, np.float32)
    Tm[idx] = t
    Vm[idx] = v
    Dm[idx] = d
    return Tm, Vm, Dm


def sawtooth(x, period, sharp=0.75):
    """0..1 sawtooth with a steep drop (hair shadow teeth)."""
    f = (x / period) % 1.0
    return np.where(f < sharp, f / sharp, (1 - f) / (1 - sharp))
