"""Geometry helpers and the character art for the 2D mesh-deform bust.

Everything here is plain numpy: 2D outlines built from Catmull-Rom and
Bezier curves, triangulated into dense art meshes (the "ArtMesh" of a
layered mesh deformation rig), stroked into tapered line art, and coloured per vertex
so soft gradients survive deformation.

Every feature is a FUNCTION OF ITS PARAMETERS with a FIXED TOPOLOGY: the
eye is built by ``eye(side, shape)``, the mouth by ``mouth(shape)`` and so
on, and calling one with another shape returns the same vertices in new
places. That is how the keyforms (blend targets) are sculpted -- the
rest mesh and every target come out of the same generator, so a target is
nothing but the generator evaluated at a different parameter value.
"""

import math

import numpy as np
from matplotlib.path import Path
from scipy.spatial import Delaunay

# Palette (display values: the flat look renders them as authored)

SKIN = (1.00, 0.918, 0.875)
SKIN_SHADE = (0.975, 0.800, 0.775)
SKIN_DEEP = (0.93, 0.70, 0.69)
SKIN_LINE = (0.56, 0.31, 0.32)
BLUSH = (1.00, 0.70, 0.72)
BLUSH_LINE = (0.96, 0.55, 0.60)

HAIR = (0.965, 0.640, 0.735)
HAIR_LIGHT = (1.00, 0.815, 0.860)
HAIR_HI = (1.00, 0.925, 0.945)
HAIR_SHADE = (0.855, 0.470, 0.610)
HAIR_DEEP = (0.665, 0.320, 0.480)
HAIR_LINE = (0.400, 0.150, 0.280)

LASH = (0.215, 0.090, 0.150)
LASH_TIP = (0.470, 0.170, 0.260)
BROW = (0.600, 0.290, 0.400)
IRIS_TOP = (0.060, 0.170, 0.330)
IRIS_MID = (0.090, 0.470, 0.640)
IRIS_LOW = (0.420, 0.900, 0.860)
IRIS_RING = (0.040, 0.110, 0.240)
PUPIL = (0.030, 0.080, 0.190)
WHITE = (1.0, 1.0, 1.0)
EYE_SHADE = (0.800, 0.820, 0.945)

MOUTH_IN = (0.560, 0.170, 0.240)
MOUTH_DEEP = (0.400, 0.100, 0.160)
TONGUE = (0.985, 0.560, 0.600)
TEETH = (1.0, 1.0, 1.0)
LIP_LINE = (0.520, 0.200, 0.250)

BLOUSE = (0.985, 0.985, 1.000)
BLOUSE_SHADE = (0.830, 0.845, 0.945)
BLOUSE_DEEP = (0.700, 0.720, 0.870)
COLLAR = (0.170, 0.215, 0.420)
COLLAR_SHADE = (0.110, 0.140, 0.300)
CLOTH_LINE = (0.180, 0.190, 0.340)
RIBBON = (0.960, 0.330, 0.420)
RIBBON_SHADE = (0.780, 0.200, 0.310)
RIBBON_LINE = (0.450, 0.100, 0.180)

GOLD = (1.00, 0.860, 0.380)
GOLD_SHADE = (0.960, 0.620, 0.260)
GOLD_LINE = (0.560, 0.330, 0.140)

# Curves


def _arr(points):
    return np.asarray(points, dtype=float)


def catmull_rom(ctrl, closed=False, per_seg=16, alpha=0.5):
    """Centripetal Catmull-Rom through ctrl; returns a dense polyline."""
    P = _arr(ctrl)
    if closed:
        P = np.vstack([P[-1], P, P[0], P[1]])
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
        for s in np.linspace(t1, t2, per_seg, endpoint=False):
            a1 = (t1 - s) / (t1 - t0) * p0 + (s - t0) / (t1 - t0) * p1
            a2 = (t2 - s) / (t2 - t1) * p1 + (s - t1) / (t2 - t1) * p2
            a3 = (t3 - s) / (t3 - t2) * p2 + (s - t2) / (t3 - t2) * p3
            b1 = (t2 - s) / (t2 - t0) * a1 + (s - t0) / (t2 - t0) * a2
            b2 = (t3 - s) / (t3 - t1) * a2 + (s - t1) / (t3 - t1) * a3
            out.append((t2 - s) / (t2 - t1) * b1 + (s - t1) / (t2 - t1) * b2)
    if not closed:
        out.append(P[-2])
    return np.array(out)


def bezier(p0, p1, p2, p3, n):
    t = np.linspace(0.0, 1.0, n)[:, None]
    p0, p1, p2, p3 = map(_arr, (p0, p1, p2, p3))
    return ((1 - t) ** 3 * p0 + 3 * (1 - t) ** 2 * t * p1 +
            3 * (1 - t) * t ** 2 * p2 + t ** 3 * p3)


def arclen(poly, closed=False):
    P = _arr(poly)
    if closed:
        P = np.vstack([P, P[:1]])
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def resample(poly, n, closed=False):
    """n points evenly spaced by arc length along poly."""
    P = _arr(poly)
    Q = np.vstack([P, P[:1]]) if closed else P
    s = arclen(P, closed)
    total = s[-1]
    ts = np.linspace(0.0, total, n, endpoint=not closed)
    x = np.interp(ts, s, Q[:, 0])
    y = np.interp(ts, s, Q[:, 1])
    return np.stack([x, y], axis=1)


def resample_spacing(poly, h, closed=False, minimum=8):
    total = arclen(poly, closed)[-1]
    n = max(minimum, int(math.ceil(total / h)) + (0 if closed else 1))
    return resample(poly, n, closed)


def normals(poly, closed=False):
    """Unit LEFT normals (tangent rotated +90 degrees)."""
    P = _arr(poly)
    if closed:
        T = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    else:
        T = np.gradient(P, axis=0)
    T /= np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-9)
    return np.stack([-T[:, 1], T[:, 0]], axis=1)


def signed_area(poly):
    P = _arr(poly)
    x, y = P[:, 0], P[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def ccw(poly):
    return poly if signed_area(poly) > 0 else poly[::-1].copy()


def mirror(points):
    P = _arr(points).copy()
    P[..., 0] *= -1.0
    return P


def smoothstep(a, b, x):
    t = np.clip((np.asarray(x, dtype=float) - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    a = _arr(a)
    b = _arr(b)
    t = np.asarray(t, dtype=float)
    if t.ndim == 1 and a.ndim == 1:
        t = t[:, None]
    return a + (b - a) * t


def mix_colors(c0, c1, t):
    t = np.clip(np.asarray(t, dtype=float), 0.0, 1.0)[..., None]
    return _arr(c0) * (1 - t) + _arr(c1) * t


# Meshes


class Part(object):
    """One art piece: 2D points, a depth, triangles and vertex colours.

    ``z`` is a scalar or per-vertex array: it is BOTH the draw order (the
    camera looks down -Z) and the depth coordinate the head lattice reads
    for parallax.
    """

    def __init__(self, name, pts, tris, colors, z, attrs=None):
        self.name = name
        self.pts = _arr(pts)
        self.tris = np.asarray(tris, dtype=np.int64).reshape(-1, 3)
        colors = _arr(colors)
        if colors.ndim == 1:
            colors = np.tile(colors, (len(self.pts), 1))
        self.colors = colors
        z = np.asarray(z, dtype=float)
        self.z = np.full(len(self.pts), float(z)) if z.ndim == 0 else z
        self.attrs = attrs or {}

    def moved(self, pts):
        """Same part (topology, colours, depth) with new 2D points."""
        return Part(self.name, pts, self.tris, self.colors, self.z,
                    self.attrs)

    @property
    def xyz(self):
        return np.column_stack([self.pts, self.z])


def fill_polygon(boundary, h):
    """Triangulate the inside of a closed outline with interior points.

    Boundary resampled at spacing h, a hexagonal lattice of interior
    points at the same spacing, Delaunay, and the triangles whose centroid
    falls outside the outline dropped. Returns (points, triangles,
    boundary_count): the first boundary_count points are the outline.
    """
    B = ccw(resample_spacing(boundary, h, closed=True))
    path = Path(np.vstack([B, B[:1]]))
    lo, hi = B.min(axis=0), B.max(axis=0)
    rows = []
    dy = h * math.sqrt(3) / 2
    y = lo[1] + dy * 0.5
    r = 0
    while y < hi[1]:
        xs = np.arange(lo[0] + (h * 0.5 if r % 2 else 0.0), hi[0], h)
        rows.append(np.stack([xs, np.full_like(xs, y)], axis=1))
        y += dy
        r += 1
    I = np.vstack(rows) if rows else np.zeros((0, 2))
    if len(I):
        inside = path.contains_points(I, radius=-1e-9)
        I = I[inside]
        # keep interior points away from the outline
        if len(I):
            d = np.min(np.linalg.norm(I[:, None, :] - B[None, :, :], axis=2),
                       axis=1)
            I = I[d > 0.55 * h]
    P = np.vstack([B, I])
    tri = Delaunay(P).simplices
    cent = P[tri].mean(axis=1)
    keep = path.contains_points(cent)
    tri = tri[keep]
    # orient counter-clockwise
    a = P[tri[:, 0]]
    b = P[tri[:, 1]]
    c = P[tri[:, 2]]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - \
        (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    tri[cross < 0] = tri[cross < 0][:, [0, 2, 1]]
    return P, tri, len(B)


def grid_tris(rows, cols, closed_cols=False):
    """Triangles of a rows x cols vertex grid (row-major)."""
    tris = []
    ncol = cols if closed_cols else cols - 1
    for r in range(rows - 1):
        for c in range(ncol):
            c1 = (c + 1) % cols
            a = r * cols + c
            b = r * cols + c1
            d = (r + 1) * cols + c
            e = (r + 1) * cols + c1
            tris.append((a, b, e))
            tris.append((a, e, d))
    return np.array(tris, dtype=np.int64)


def band(curve_a, curve_b, rows):
    """Grid between two parallel-parameterised curves (rows >= 2)."""
    A = _arr(curve_a)
    B = _arr(curve_b)
    v = np.linspace(0.0, 1.0, rows)
    pts = np.concatenate([A * (1 - s) + B * s for s in v], axis=0)
    return pts, grid_tris(rows, len(A)), v


def stroke(poly, width, closed=False, offset=0.0, rows=2):
    """A ribbon along poly: width per vertex (array or scalar).

    offset -1 puts the whole stroke on the right side of the curve, +1
    on the left, 0 centres it. Returns (points, triangles).
    """
    P = _arr(poly)
    W = np.broadcast_to(np.asarray(width, dtype=float), (len(P),))
    N = normals(P, closed)
    lo = (offset - 1.0) * 0.5
    hi = (offset + 1.0) * 0.5
    layers = []
    for s in np.linspace(lo, hi, rows):
        layers.append(P + N * (W * s)[:, None])
    pts = np.concatenate(layers, axis=0)
    return pts, grid_tris(rows, len(P), closed_cols=closed)


def taper(n, start=0.1, end=0.1, power=1.0):
    """Width multiplier ramping in over `start` and out over `end`."""
    t = np.linspace(0.0, 1.0, n)
    w = np.ones(n)
    if start > 0:
        w = np.minimum(w, (np.clip(t / start, 0, 1)) ** power)
    if end > 0:
        w = np.minimum(w, (np.clip((1 - t) / end, 0, 1)) ** power)
    return w


def ellipse_mesh(center, rx, ry, rings=6, segs=48, rot=0.0):
    """Polar grid: centre vertex + rings; returns (pts, tris, r, ang)."""
    cx, cy = center
    pts = [(cx, cy)]
    rr = [0.0]
    aa = [0.0]
    ca, sa = math.cos(rot), math.sin(rot)
    for i in range(1, rings + 1):
        r = i / float(rings)
        for j in range(segs):
            a = 2 * math.pi * j / segs
            x = rx * r * math.cos(a)
            y = ry * r * math.sin(a)
            pts.append((cx + ca * x - sa * y, cy + sa * x + ca * y))
            rr.append(r)
            aa.append(a)
    tris = []
    for j in range(segs):
        tris.append((0, 1 + j, 1 + (j + 1) % segs))
    for i in range(1, rings):
        base0 = 1 + (i - 1) * segs
        base1 = 1 + i * segs
        for j in range(segs):
            j1 = (j + 1) % segs
            tris.append((base0 + j, base1 + j, base1 + j1))
            tris.append((base0 + j, base1 + j1, base0 + j1))
    return np.array(pts), np.array(tris), np.array(rr), np.array(aa)


def strand(spine, widths, cols=5, lean=None):
    """A hair strand: grid along a root->tip spine, `cols` across.

    widths: per-spine-vertex full width. Returns (pts, tris, t, u) with t
    the along parameter (0 root .. 1 tip) and u the across one (0 left
    edge .. 1 right edge, looking root->tip).
    """
    S = _arr(spine)
    W = np.asarray(widths, dtype=float)
    N = normals(S)
    t = arclen(S)
    t = t / t[-1]
    u = np.linspace(0.0, 1.0, cols)
    rows = []
    for c in u:
        shift = (0.5 - c)
        if lean is not None:
            shift = shift + lean
        rows.append(S + N * (W * shift)[:, None])
    # vertex order: along-major within each across column -> transpose
    grid = np.stack(rows, axis=1)  # (n, cols, 2)
    pts = grid.reshape(-1, 2)
    tris = grid_tris(len(S), cols)
    tt = np.repeat(t, cols)
    uu = np.tile(u, len(S))
    return pts, tris, tt, uu


def edges_of_strand(spine, widths, cols=5, lean=None):
    """The two outline edges of a strand (left edge root->tip, right edge
    tip->root) as one open polyline."""
    pts, _, _, _ = strand(spine, widths, cols, lean)
    grid = pts.reshape(len(spine), cols, 2)
    left = grid[:, 0]
    right = grid[:, -1]
    return np.vstack([left, right[::-1][1:]])


# The character
#
# Units are centimetres-ish: the head is centred on the origin and about
# 21 wide; the bust is cut by the frame near y = -34.

FACE_CTRL_R = [(0.0, 9.0), (4.8, 8.6), (7.9, 6.4), (9.05, 2.8), (9.2, -0.6),
               (8.85, -3.6), (7.95, -6.4), (6.35, -8.9), (4.25, -11.0),
               (1.95, -12.3), (0.0, -12.7)]

# --- depths (draw order AND head-lattice parallax coordinate) -------------
Z = dict(
    backhair=-2.00, backhair_in=-1.98, backhair_line=-1.97,
    neck=-1.30, neck_shade=-1.28, neck_line=-1.27,
    torso=-1.00, torso_shade=-0.99, torso_line=-0.98,
    collar=-0.90, collar_shade=-0.89, collar_stripe=-0.885, collar_line=-0.88,
    ribbon_tail=-0.80, ribbon=-0.78, ribbon_shade=-0.775, ribbon_line=-0.77,
    ear=-0.60, ear_line=-0.59,
    face=0.00, face_line=0.01,
    eyewhite=0.10, iris=0.115, iris_ring=0.118, pupil=0.12, iris_glow=0.122,
    highlight=0.13,
    mask=0.16, blush=0.17, blush_line=0.172, foreshadow=0.175,
    lash_low=0.18, lash=0.20, crease=0.21,
    mouth_in=0.24, teeth=0.245, tongue=0.25, lip=0.26,
    brow=1.45, nose=0.50,
    sidelock=0.90, sidelock_line=0.905, ahoge=0.92, ahoge_line=0.925,
    cap=1.00, cap_hi=1.40, cap_line=1.03,
    bang=1.10, bang_step=0.03,
    clip=1.60, clip_shade=1.61, clip_line=1.62,
)


def face_outline():
    right = FACE_CTRL_R
    left = [(-x, y) for (x, y) in reversed(right[1:-1])]
    ctrl = right + left
    return catmull_rom(ctrl, closed=True, per_seg=14)


def jaw_line():
    """The visible face outline: cheek to cheek under the chin."""
    right = [p for p in FACE_CTRL_R if p[1] < 2.0]
    left = [(-x, y) for (x, y) in reversed(right[:-1])]
    ctrl = right + left
    return catmull_rom(ctrl, per_seg=14)



EYE_I = np.array([1.85, -3.55])
EYE_O = np.array([6.95, -2.40])
EYE_N = 33          # samples along the lid curves
EYE_SHAPES = {
    # upper lid Bezier (P0, P1, P2, P3), lower lid Bezier, lash scale
    "open": dict(U=[EYE_I, (2.15, -0.35), (5.30, 0.55), EYE_O],
                 L=[EYE_I, (2.55, -6.25), (5.85, -6.00), EYE_O],
                 lash=1.0, flick=1.0),
    "half": dict(U=[EYE_I + (0, -0.10), (2.45, -2.75), (5.40, -2.25),
                    EYE_O + (0, -0.20)],
                 L=[EYE_I, (2.55, -6.10), (5.85, -5.85), EYE_O],
                 lash=0.9, flick=0.85),
    "closed": dict(U=[EYE_I + (0.05, -0.35), (2.75, -5.15), (5.35, -4.95),
                      EYE_O + (-0.10, -0.95)],
                   L=[EYE_I + (0.05, -0.37), (2.75, -5.17), (5.35, -4.97),
                      EYE_O + (-0.10, -0.97)],
                   lash=0.62, flick=0.55),
    "smile": dict(U=[EYE_I + (0.20, -0.75), (2.85, -2.55), (5.40, -2.35),
                     EYE_O + (-0.20, -1.45)],
                  L=[EYE_I + (0.20, -0.77), (2.85, -2.57), (5.40, -2.37),
                     EYE_O + (-0.20, -1.47)],
                  lash=0.72, flick=0.45),
}
IRIS_C = np.array([4.40, -3.10])
IRIS_RX, IRIS_RY = 2.05, 2.62


def _eye_curves(shape):
    s = EYE_SHAPES[shape]
    U = bezier(*s["U"], n=EYE_N)
    L = bezier(*s["L"], n=EYE_N)
    return U, L, s


def _lash_line(U, s):
    """Upper lash: a thick tapered stroke over the lid, with a flick."""
    n = len(U)
    t = np.linspace(0, 1, n)
    # thickness profile: thin at the inner corner, heavy outer half
    w = 0.16 + 0.50 * smoothstep(0.0, 0.55, t) - 0.10 * smoothstep(0.9, 1.0, t)
    w = w * s["lash"]
    # flick: continue past the outer corner
    tan = U[-1] - U[-4]
    tan /= np.linalg.norm(tan)
    nrm = np.array([-tan[1], tan[0]])
    f = s["flick"]
    ext = [U[-1] + tan * 0.45 * f + nrm * 0.05 * f,
           U[-1] + tan * 0.85 * f - nrm * 0.12 * f,
           U[-1] + tan * 1.10 * f - nrm * 0.34 * f]
    spine = np.vstack([U, ext])
    wext = np.array([0.42, 0.26, 0.02]) * s["lash"]
    W = np.concatenate([w, wext])
    return spine, W


def eye(side, shape="open", look=(0.0, 0.0), look_scale=1.0):
    """All pieces of one eye. side=+1 right (screen right), -1 left.

    look shifts the iris group; highlights follow at 60%.
    """
    U, L, s = _eye_curves(shape)
    parts = []
    # --- eye white: grid between the lids, extended a touch past them
    n = len(U)
    mid = (U + L) * 0.5
    Ue = mid + (U - mid) * 1.06
    Le = mid + (L - mid) * 1.06
    pts, tris, v = band(Ue, Le, 6)
    vv = np.repeat(v, n)
    col = mix_colors(EYE_SHADE, WHITE, smoothstep(0.0, 0.45, vv))
    parts.append(Part("eyewhite", pts, tris, col, Z["eyewhite"]))

    # --- iris group (moves with look; look is in screen terms, so the
    # pre-mirror left eye takes it negated in x)
    lx, ly = look
    if side < 0:
        lx = -lx
    c = IRIS_C + np.array([lx, ly])
    sx = 1.0 - 0.10 * abs(lx) / 0.9
    pts, tris, r, a = ellipse_mesh(c, IRIS_RX * sx, IRIS_RY, rings=7, segs=44)
    yrel = (pts[:, 1] - c[1]) / IRIS_RY          # -1 bottom .. +1 top
    col = mix_colors(IRIS_LOW, IRIS_MID, smoothstep(-0.95, 0.05, yrel))
    col = mix_colors(col, IRIS_TOP, smoothstep(0.0, 0.75, yrel))
    col = mix_colors(col, IRIS_RING, smoothstep(0.80, 1.0, r))
    parts.append(Part("iris", pts, tris, col, Z["iris"]))
    # inner glow crescent (lower half, lighter)
    gp, gt, gr, ga = ellipse_mesh(c + (0.0, -0.95), 1.35 * sx, 0.95,
                                  rings=3, segs=32)
    gy = (gp[:, 1] - (c[1] - 0.95)) / 0.95
    gcol = mix_colors(IRIS_LOW, (0.62, 0.98, 0.92), 1 - gr)
    gcol = mix_colors(gcol, IRIS_MID, smoothstep(0.0, 1.0, gy) * 0.8 + gr * 0.2)
    parts.append(Part("iris_glow", gp, gt, gcol, Z["iris_glow"]))
    pp, pt, pr, pa = ellipse_mesh(c + (0.0, 0.18), 0.92 * sx, 1.30,
                                  rings=3, segs=32)
    pcol = mix_colors(PUPIL, IRIS_TOP, pr ** 3 * 0.6)
    parts.append(Part("pupil", pp, pt, pcol, Z["pupil"]))
    # iris outline ring (dark, thicker on top)
    ring = np.column_stack([c[0] + IRIS_RX * sx * np.cos(np.linspace(0, 2 * np.pi, 60, endpoint=False)),
                            c[1] + IRIS_RY * np.sin(np.linspace(0, 2 * np.pi, 60, endpoint=False))])
    wr = 0.10 + 0.10 * np.clip(np.sin(np.linspace(0, 2 * np.pi, 60, endpoint=False)), 0, 1)
    rp, rt = stroke(ring, wr, closed=True, offset=0.0)
    parts.append(Part("iris_ring", rp, rt, IRIS_RING, Z["iris_ring"]))

    # highlights: follow the look at 60%, and keep ONE light direction for
    # both eyes (the left eye is built here and mirrored below, so its
    # offsets are pre-mirrored)
    hc = IRIS_C + np.array([lx, ly]) * 0.6
    for name, off, rx, ry, rot in (("hi_big", (-0.80, 1.22), 0.70, 0.86, 0.35),
                                   ("hi_small", (0.78, -1.28), 0.33, 0.30, 0.0),
                                   ("hi_spark", (0.95, 1.35), 0.19, 0.19, 0.0)):
        o = np.array(off) * np.array([side, 1.0])
        hp, ht, hr, ha = ellipse_mesh(hc + o, rx, ry, rings=3, segs=28,
                                      rot=rot * side)
        hcol = mix_colors(WHITE, (0.86, 0.98, 1.0), hr ** 2 * 0.5)
        parts.append(Part(name, hp, ht, hcol, Z["highlight"]))

    # --- skin masks around the opening (what makes the lids "clip")
    inner = np.vstack([U, L[::-1][1:-1]])           # closed contour
    # outer contour: the OPEN opening pushed radially outward by a fixed
    # margin. It never moves with the lid, so a closing lid sweeps skin down
    # over the eye, and the margin covers every place the iris can reach.
    U0, L0, _ = _eye_curves("open")
    open_inner = np.vstack([U0, L0[::-1][1:-1]])
    c0 = open_inner.mean(axis=0) + np.array([0.0, -0.2])
    d = open_inner - c0
    r = np.linalg.norm(d, axis=1, keepdims=True)
    margin = np.where(d[:, 1:2] > 0, 2.35, 2.0)
    outer = c0 + d * (1.0 + margin / np.maximum(r, 1e-6))
    mp, mt, mv = band(inner, outer, 4)
    mt = grid_tris(4, len(inner), closed_cols=True)
    parts.append(Part("mask", mp, mt, SKIN, Z["mask"]))

    # --- lower lash (outer part of the lower lid)
    k0 = int(n * 0.45)
    lower = L[k0:]
    wl = (0.08 + 0.12 * np.linspace(0, 1, len(lower)) ** 1.5) * taper(len(lower), 0.35, 0.05)
    lp, lt = stroke(lower, wl * max(0.5, s["lash"]), offset=-0.6)
    lcol = mix_colors(LASH_TIP, LASH, np.repeat(np.linspace(0, 1, len(lower))[None, :], 2, axis=0).reshape(-1))
    parts.append(Part("lash_low", lp, lt, lcol, Z["lash_low"]))

    # --- upper lash
    spine, W = _lash_line(U, s)
    # stroke sits mostly above the lid line (left normal points up for an
    # inner->outer curve on the right eye; flip for the left eye later)
    sp, st = stroke(spine, W + 0.10, offset=0.78, rows=3)
    tt = np.tile(np.linspace(0, 1, len(spine)), 3)
    lcol = mix_colors(LASH, LASH_TIP, smoothstep(0.78, 1.0, tt))
    parts.append(Part("lash", sp, st, lcol, Z["lash"]))

    # --- double-eyelid crease
    k0, k1 = int(n * 0.30), int(n * 0.92)
    cr = U[k0:k1] + normals(U[k0:k1]) * 0.78 * max(s["lash"], 0.6)
    cr_w = 0.085 * taper(len(cr), 0.3, 0.3)
    cp, ct = stroke(cr, cr_w)
    parts.append(Part("crease", cp, ct, (0.70, 0.42, 0.44), Z["crease"]))

    if side < 0:
        parts = [p.moved(mirror(p.pts)) for p in parts]
        for p in parts:
            # mirroring flips triangle winding; renderers here draw both
            # sides, but keep the winding consistent anyway
            p.tris = p.tris[:, [0, 2, 1]]
    for p in parts:
        p.name = ("R_" if side > 0 else "L_") + p.name
    return parts



BROW_SHAPES = {
    "rest": [(1.70, 1.25), (3.90, 2.10), (6.45, 1.55)],
    "up": [(1.70, 2.35), (3.95, 3.25), (6.50, 2.60)],
    "down": [(1.75, 0.80), (3.95, 1.45), (6.40, 1.05)],
    "sad": [(1.65, 2.25), (3.85, 2.45), (6.40, 1.35)],
    "angry": [(1.85, 0.55), (4.00, 1.55), (6.45, 1.95)],
}


def brow(side, shape="rest"):
    spine = catmull_rom(BROW_SHAPES[shape], per_seg=14)
    spine = resample(spine, 30)
    t = np.linspace(0, 1, len(spine))
    w = (0.36 - 0.30 * t ** 1.2) * taper(len(spine), 0.06, 0.08, 0.6)
    pts, tris = stroke(spine, w, offset=0.0, rows=3)
    col = mix_colors(BROW, HAIR_LINE, np.tile(1 - t, 3) * 0.5)
    part = Part("brow", pts, tris, col, Z["brow"])
    if side < 0:
        part = part.moved(mirror(part.pts))
    part.name = ("R_" if side > 0 else "L_") + "brow"
    return [part]



MOUTH_N = 25
MOUTH_SHAPES = {
    # corner x (half width), corner y, upper centre y, lower centre y,
    # lower-curve roundness exponent
    "rest": dict(cx=1.22, cy=-8.88, uy=-9.08, ly=-9.08, p=2.0),
    "A": dict(cx=1.38, cy=-8.78, uy=-8.62, ly=-11.15, p=2.6),
    "I": dict(cx=1.78, cy=-8.86, uy=-8.70, ly=-9.78, p=2.2),
    "U": dict(cx=0.62, cy=-9.05, uy=-8.80, ly=-9.95, p=2.4),
    "E": dict(cx=1.58, cy=-8.82, uy=-8.66, ly=-10.30, p=2.3),
    "O": dict(cx=0.98, cy=-9.00, uy=-8.52, ly=-10.95, p=2.8),
    "smile": dict(cx=1.45, cy=-8.40, uy=-9.10, ly=-9.10, p=2.0),
    "frown": dict(cx=1.15, cy=-9.20, uy=-8.95, ly=-8.95, p=2.0),
}


def _mouth_curves(shape):
    m = MOUTH_SHAPES[shape]
    t = np.linspace(0, 1, MOUTH_N)
    x = -m["cx"] + 2 * m["cx"] * t
    bump = 1.0 - np.abs(2 * t - 1) ** 2.0
    U = np.stack([x, m["cy"] + (m["uy"] - m["cy"]) * bump], axis=1)
    bl = 1.0 - np.abs(2 * t - 1) ** m["p"]
    L = np.stack([x * (1.0 - 0.06 * bl), m["cy"] + (m["ly"] - m["cy"]) * bl], axis=1)
    return U, L


def mouth(shape="rest"):
    U, L = _mouth_curves(shape)
    parts = []
    rows = 8
    pts, tris, v = band(U, L, rows)
    vv = np.repeat(v, len(U))
    col = mix_colors(MOUTH_DEEP, MOUTH_IN, smoothstep(0.0, 0.6, vv))
    parts.append(Part("mouth_in", pts, tris, col, Z["mouth_in"]))
    # teeth: top band, inner 76% of the width
    k0, k1 = 4, len(U) - 4
    Ut, Lt = U[k0:k1], L[k0:k1]
    T0 = Ut + (Lt - Ut) * 0.02
    T1 = Ut + (Lt - Ut) * 0.26
    tp, tt, tv = band(T0, T1, 3)
    tcol = mix_colors(TEETH, (0.88, 0.88, 0.95), np.repeat(tv, len(T0)))
    parts.append(Part("teeth", tp, tt, tcol, Z["teeth"]))
    # tongue: a mound in the lower half
    k0, k1 = 5, len(U) - 5
    Ug, Lg = U[k0:k1], L[k0:k1]
    tn = np.linspace(0, 1, len(Ug))
    top = 0.62 + 0.20 * np.abs(2 * tn - 1) ** 2
    G0 = Ug + (Lg - Ug) * top[:, None]
    G1 = Ug + (Lg - Ug) * 0.97
    gp, gt, gv = band(G0, G1, 4)
    gcol = mix_colors(TONGUE, (0.90, 0.42, 0.48), np.repeat(1 - gv, len(G0)) * 0.6)
    parts.append(Part("tongue", gp, gt, gcol, Z["tongue"]))
    # lip line along the upper curve, lighter hint along the lower
    w = 0.15 * taper(len(U), 0.12, 0.12, 0.7)
    lp, lt = stroke(U, w, offset=0.2)
    parts.append(Part("lip_up", lp, lt, LIP_LINE, Z["lip"]))
    k0, k1 = 6, len(L) - 6
    wl = 0.10 * taper(k1 - k0, 0.3, 0.3)
    lp2, lt2 = stroke(L[k0:k1], wl, offset=-0.3)
    parts.append(Part("lip_low", lp2, lt2, (0.68, 0.34, 0.38), Z["lip"]))
    return parts




def face_parts():
    parts = []
    outline = face_outline()
    pts, tris, nb = fill_polygon(outline, 0.85)
    parts.append(Part("face", pts, tris, SKIN, Z["face"]))
    # jaw line art
    jl = resample(jaw_line(), 90)
    t = np.linspace(0, 1, len(jl))
    w = (0.13 + 0.07 * np.exp(-((t - 0.5) / 0.18) ** 2)) * taper(len(jl), 0.12, 0.12, 0.8)
    lp, lt = stroke(jl, w, offset=0.0)
    parts.append(Part("jaw_line", lp, lt, SKIN_LINE, Z["face_line"]))
    # ears (behind the face)
    for side in (1, -1):
        ear = catmull_rom([(8.4, -0.2), (9.9, 0.4), (10.55, -1.6), (10.15, -3.9),
                           (9.1, -5.0), (8.3, -4.2)], closed=True, per_seg=10)
        ep, et, _ = fill_polygon(ear, 0.55)
        ecol = mix_colors(SKIN, SKIN_SHADE, smoothstep(9.4, 10.4, ep[:, 0]) * 0.7)
        inner = catmull_rom([(9.05, -0.8), (9.9, -1.3), (9.75, -3.1), (9.15, -3.9)], per_seg=8)
        ip, it = stroke(inner, 0.09 * taper(len(inner), 0.2, 0.3))
        eo = resample(catmull_rom([(8.9, 0.1), (9.9, 0.4), (10.55, -1.6), (10.15, -3.9), (9.1, -5.0)], per_seg=10), 40)
        op, ot = stroke(eo, 0.11 * taper(40, 0.15, 0.25))
        pieces = [Part("ear", ep, et, ecol, Z["ear"]),
                  Part("ear_in", ip, it, SKIN_LINE, Z["ear_line"]),
                  Part("ear_line", op, ot, SKIN_LINE, Z["ear_line"])]
        if side < 0:
            pieces = [p.moved(mirror(p.pts)) for p in pieces]
        for p in pieces:
            p.name = ("R_" if side > 0 else "L_") + p.name
        parts += pieces
    # blush: pink centre fading to the skin colour at the rim
    for side in (1, -1):
        bp, bt, br, ba = ellipse_mesh((5.15 * side, -6.25), 1.85, 0.85, rings=6, segs=40)
        bcol = mix_colors(BLUSH, SKIN, smoothstep(0.15, 1.0, br))
        parts.append(Part(("R_" if side > 0 else "L_") + "blush", bp, bt, bcol, Z["blush"]))
        for k in range(3):
            x0 = 4.35 + 0.62 * k
            ln = np.array([(x0 + 0.28, -5.80), (x0 - 0.10, -6.55)])
            ln = resample(ln, 6)
            sp, st = stroke(ln, 0.09 * taper(6, 0.3, 0.3))
            if side < 0:
                sp = mirror(sp)
            parts.append(Part(("R_" if side > 0 else "L_") + "blush_line%d" % k,
                              sp, st, BLUSH_LINE, Z["blush_line"]))
    # nose: a small shadow stroke and a tip highlight
    nose = resample(np.array([(0.28, -6.15), (0.42, -6.70), (0.12, -6.95)]), 10)
    npts, ntris = stroke(nose, 0.11 * taper(10, 0.35, 0.25))
    parts.append(Part("nose", npts, ntris, (0.80, 0.52, 0.50), Z["nose"]))
    return parts


def forehead_shadow(bang_tips):
    """Soft cast shadow of the fringe on the forehead: a band under the
    hairline whose lower edge scallops with the strand tips."""
    xs = np.linspace(-8.6, 8.6, 70)
    tips = sorted(bang_tips, key=lambda p: p[0])
    tx = np.array([p[0] for p in tips])
    ty = np.array([p[1] for p in tips])
    # lower edge: follow tips, rise between them
    low = np.interp(xs, tx, ty)
    # scallop: between strand tips the shadow rides up
    base = np.full_like(xs, 5.6)
    for k in range(len(tx) - 1):
        m = (xs > tx[k]) & (xs < tx[k + 1])
        s = (xs[m] - tx[k]) / (tx[k + 1] - tx[k])
        low[m] = low[m] + (np.sin(np.pi * s) ** 1.2) * (3.2 + 0.0 * k)
    low = np.minimum(low - 0.55, base)
    low = np.maximum(low, -1.6)
    top = np.full_like(xs, 8.2)
    # keep inside the face at the temples
    top = np.minimum(top, 8.2 - 2.0 * smoothstep(6.0, 8.6, np.abs(xs)))
    low = np.minimum(low, top - 0.2)
    A = np.stack([xs, low], axis=1)
    B = np.stack([xs, top], axis=1)
    pts, tris, v = band(A, B, 4)
    col = np.tile(np.array(SKIN_SHADE), (len(pts), 1))
    return [Part("foreshadow", pts, tris, col, Z["foreshadow"])]



# Front fringe strands: root, control points, tip, root width, side lean.
BANGS = [
    # name,  spine points (root -> tip),                                          width
    ("Bang_C", [(0.3, 11.3), (0.45, 6.2), (0.05, 2.6), (-0.75, -0.5)], 3.5, 0.00),
    ("Bang_L1", [(-2.25, 11.1), (-2.55, 6.2), (-3.3, 2.9), (-4.35, 0.55)], 3.6, 0.05),
    ("Bang_R1", [(2.75, 11.0), (3.15, 6.2), (3.95, 3.1), (4.95, 1.05)], 3.6, -0.05),
    ("Bang_L2", [(-4.85, 10.5), (-5.7, 5.4), (-6.7, 2.2), (-7.55, -1.0)], 3.4, 0.08),
    ("Bang_R2", [(5.35, 10.2), (6.3, 5.1), (7.2, 1.9), (7.85, -0.8)], 3.2, -0.08),
    ("Bang_S1", [(1.5, 10.7), (1.8, 6.3), (2.15, 4.2), (2.6, 2.55)], 2.0, 0.00),
    ("Bang_S2", [(-1.0, 10.8), (-1.15, 6.4), (-1.45, 4.3), (-1.9, 2.85)], 1.9, 0.00),
]
SIDELOCKS = [
    ("Side_R", [(8.6, 5.6), (9.55, 1.2), (9.75, -3.6), (9.45, -8.6), (9.0, -13.0), (8.35, -17.2)], 3.5, 0.0),
    ("Side_L", [(-8.6, 5.6), (-9.55, 1.2), (-9.75, -3.6), (-9.45, -8.6), (-9.0, -13.0), (-8.35, -17.2)], 3.5, 0.0),
    ("SideIn_R", [(7.4, 4.2), (8.05, 0.6), (8.25, -3.0), (8.05, -6.6)], 1.55, 0.0),
    ("SideIn_L", [(-7.4, 4.2), (-8.05, 0.6), (-8.25, -3.0), (-8.05, -6.6)], 1.55, 0.0),
]
AHOGE = ("Ahoge", [(0.6, 13.0), (0.9, 15.3), (2.2, 17.1), (3.8, 17.3), (4.7, 16.2)], 1.05, 0.0)


def strand_spine(ctrl, n=26):
    return resample(catmull_rom(ctrl, per_seg=16), n)


def strand_width(n, root_w, tip_power=1.1, belly=0.22):
    """Full near the root, a slight belly, then a curved taper to a point."""
    t = np.linspace(0, 1, n)
    w = root_w * (1.0 - t) ** tip_power * (1.0 + belly * np.sin(np.pi * np.clip(t * 1.4, 0, 1)))
    return np.maximum(w, 0.0)


def cap_color(y):
    """The cap's vertical colour ramp, so strand roots can melt into it."""
    col = mix_colors(HAIR_SHADE, HAIR, smoothstep(5.0, 8.5, y))
    return mix_colors(col, HAIR_LIGHT, smoothstep(11.0, 13.6, y) * 0.6)


def hair_strand(name, ctrl, root_w, lean, z, base=HAIR, shade=HAIR_SHADE,
                n=26, cols=6, root_dark=True, root_blend=False, line_from=0.05):
    spine = strand_spine(ctrl, n)
    W = strand_width(n, root_w)
    pts, tris, t, u = strand(spine, W, cols)
    # volume: one side in shadow, darker at the root, light streak
    side = smoothstep(0.55, 0.05, u)         # left edge shaded
    col = mix_colors(base, shade, side * 0.75)
    streak = np.exp(-((u - 0.68) / 0.12) ** 2) * smoothstep(0.1, 0.35, t) * (1 - smoothstep(0.55, 0.9, t))
    col = mix_colors(col, HAIR_LIGHT, streak * 0.85)
    if root_dark:
        col = mix_colors(col, shade, (1 - smoothstep(0.0, 0.22, t)) * 0.35)
    if root_blend:
        # the root melts into the cap: same colour ramp, no visible edge
        col = mix_colors(col, cap_color(pts[:, 1]), 1 - smoothstep(0.08, 0.34, t))
    parts = [Part(name, pts, tris, col, z, attrs=dict(t=t, spine=spine))]
    edge = edges_of_strand(spine, W, cols)
    m = len(edge)
    we = 0.13 * np.ones(m)
    # fade the outline in at both root ends (the strand merges into the cap)
    k = np.concatenate([np.linspace(0, 1, n), np.linspace(1, 0, n)[1:]])
    we *= smoothstep(line_from, line_from + 0.18, k)
    ep, et = stroke(edge, we, offset=0.0)
    et_t = np.concatenate([np.linspace(0, 1, n), np.linspace(1, 0, n)[1:]])
    parts.append(Part(name + "_line", ep, et, HAIR_LINE, z + 0.005,
                      attrs=dict(t=np.tile(et_t, 2), spine=spine)))
    return parts


def cap_parts():
    outline_ctrl = [(-9.9, 1.6), (-11.35, 6.4), (-9.7, 10.6), (-5.7, 13.0),
                    (0.0, 13.75), (5.7, 13.0), (9.7, 10.6), (11.35, 6.4),
                    (9.9, 1.6), (8.4, 3.6), (6.9, 4.3), (4.2, 6.5), (0.0, 7.3),
                    (-4.2, 6.5), (-6.9, 4.3), (-8.4, 3.6)]
    outline = catmull_rom(outline_ctrl, closed=True, per_seg=12)
    pts, tris, nb = fill_polygon(outline, 0.85)
    yy = pts[:, 1]
    col = mix_colors(HAIR_SHADE, HAIR, smoothstep(5.0, 8.5, yy))
    col = mix_colors(col, HAIR_LIGHT, smoothstep(11.0, 13.6, yy) * 0.6)
    parts = [Part("cap", pts, tris, col, Z["cap"])]
    # angel-ring highlight: a band following the skull whose lower edge
    # breaks into sharp downward spikes, drawn OVER the fringe roots (the
    # sheen lies on the hair surface whichever lock it crosses)
    xs = np.linspace(-7.4, 7.4, 149)
    arc = 13.3 - 0.056 * xs ** 2
    top = arc - 1.05
    spikes = np.zeros_like(xs)
    centres = [-6.2, -4.9, -3.55, -2.3, -1.05, 0.2, 1.45, 2.7, 3.9, 5.2, 6.35]
    depths = [0.55, 0.95, 0.7, 1.15, 0.8, 1.25, 0.75, 1.1, 0.7, 0.9, 0.5]
    for c, d in zip(centres, depths):
        spikes = np.maximum(spikes, d * np.clip(1 - np.abs(xs - c) / 0.62, 0, 1) ** 1.6)
    zig = arc - 1.95 - spikes
    A = np.stack([xs, zig], axis=1)
    B = np.stack([xs, top], axis=1)
    hp, ht, hv = band(A, B, 5)
    vv = np.repeat(hv, len(xs))
    w = smoothstep(7.4, 5.0, np.abs(hp[:, 0])) * (1.0 - 0.8 * smoothstep(0.45, 1.0, vv))
    hcol = mix_colors(cap_color(hp[:, 1]), HAIR_HI, w * 0.97)
    parts.append(Part("cap_hi", hp, ht, hcol, Z["cap_hi"]))
    # outline over the crown
    k0 = 0
    crown = catmull_rom(outline_ctrl[:9], per_seg=12)
    crown = resample(crown, 90)
    wc = 0.15 * taper(90, 0.08, 0.08)
    cp, ct = stroke(crown, wc)
    parts.append(Part("cap_line", cp, ct, HAIR_LINE, Z["cap_line"]))
    return parts


def back_hair_parts():
    # outer silhouette, clockwise from the crown on the right
    right = [(0.0, 13.5), (6.6, 12.4), (10.9, 8.9), (12.9, 3.2), (13.35, -3.5),
             (13.55, -9.8), (14.3, -15.5), (15.0, -20.5), (15.35, -25.0),
             (16.0, -30.2), (13.6, -27.8), (12.7, -31.2), (10.9, -26.8),
             (9.7, -29.0), (8.9, -23.0), (6.6, -22.4), (3.2, -22.8),
             (0.0, -22.8)]
    left = [(-x, y) for (x, y) in reversed(right[1:-1])]
    outline = catmull_rom(right + left, closed=True, per_seg=8)
    pts, tris, nb = fill_polygon(outline, 1.05)
    ax = np.abs(pts[:, 0])
    # inner back-of-hair shade between the locks, lighter on the outer rims
    col = mix_colors(HAIR_DEEP, HAIR_SHADE, smoothstep(6.0, 11.5, ax))
    col = mix_colors(col, HAIR, smoothstep(11.5, 13.8, ax) * 0.8)
    parts = [Part("backhair", pts, tris, col, Z["backhair"])]
    # outline of the two outer flanks
    for side in (1, -1):
        flank = catmull_rom([(x * side, y) for (x, y) in right[1:12]], per_seg=10)
        flank = resample(flank, 110)
        w = 0.15 * taper(110, 0.05, 0.02)
        fp, ft = stroke(flank, w)
        parts.append(Part(("R_" if side > 0 else "L_") + "backhair_line", fp, ft,
                          HAIR_LINE, Z["backhair_line"]))
        # strand lines inside the mass
        for k, (x0, x1) in enumerate(((11.6, 13.4), (10.2, 11.4))):
            ln = catmull_rom([(x0 * side, -2.0 - 3 * k), ((x0 + 0.6) * side, -12.0),
                              (x1 * side, -22.0 - 2 * k)], per_seg=10)
            ln = resample(ln, 40)
            lp, lt = stroke(ln, 0.09 * taper(40, 0.3, 0.3))
            parts.append(Part(("R_" if side > 0 else "L_") + "backhair_strand%d" % k,
                              lp, lt, HAIR_DEEP, Z["backhair_line"]))
    return parts


def front_hair_parts():
    parts = []
    for name, ctrl, w, lean in SIDELOCKS:
        parts += hair_strand(name, ctrl, w, lean, Z["sidelock"], n=30, cols=5,
                             root_blend=True, line_from=0.06)
    name, ctrl, w, lean = AHOGE
    parts += hair_strand(name, ctrl, w, lean, Z["ahoge"], n=26, cols=4, root_dark=False)
    parts += cap_parts()
    for k, (name, ctrl, w, lean) in enumerate(BANGS):
        parts += hair_strand(name, ctrl, w, lean, Z["bang"] + Z["bang_step"] * k,
                             n=24, cols=5, root_blend=True, line_from=0.30)
    parts += hair_clip()
    return parts


def hair_clip():
    c = np.array([6.95, 7.55])
    rot = math.radians(14)
    pts = []
    for k in range(10):
        a = math.pi / 2 + k * math.pi / 5 + rot
        r = 1.45 if k % 2 == 0 else 0.66
        pts.append(c + r * np.array([math.cos(a), math.sin(a)]))
    star = catmull_rom(pts, closed=True, per_seg=3, alpha=0.0)
    sp, st, nb = fill_polygon(star, 0.22)
    d = sp - c
    shade = smoothstep(-0.2, 1.0, -(d[:, 0] * 0.6 + d[:, 1] * 0.8))
    col = mix_colors(GOLD, GOLD_SHADE, shade * 0.9)
    parts = [Part("clip", sp, st, col, Z["clip"])]
    outline = resample(star, 80, closed=True)
    op, ot = stroke(outline, 0.12, closed=True)
    parts.append(Part("clip_line", op, ot, GOLD_LINE, Z["clip_line"]))
    hp, ht, hr, ha = ellipse_mesh(c + (-0.25, 0.35), 0.28, 0.18, rings=2, segs=16, rot=0.5)
    parts.append(Part("clip_hi", hp, ht, (1.0, 0.98, 0.85), Z["clip_line"] + 0.005))
    return parts




def neck_parts():
    ctrl = [(-3.15, -6.5), (3.15, -6.5), (3.2, -11.0), (3.45, -15.0),
            (4.4, -17.6), (2.6, -20.5), (0.0, -23.5), (-2.6, -20.5),
            (-4.4, -17.6), (-3.45, -15.0), (-3.2, -11.0)]
    outline = catmull_rom(ctrl, closed=True, per_seg=10)
    pts, tris, nb = fill_polygon(outline, 0.8)
    parts = [Part("neck", pts, tris, SKIN, Z["neck"])]
    # shadow cast by the jaw: a band under the chin line
    jl = resample(jaw_line(), 60)
    m = np.abs(jl[:, 0]) < 3.3
    jl = jl[m]
    top = jl + np.array([0.0, 1.0])
    low = jl.copy()
    low[:, 1] -= 1.25 + 0.95 * np.cos(np.clip(jl[:, 0] / 3.3, -1, 1) * np.pi / 2)
    low[:, 0] *= 1.0
    sp, stri, v = band(low, top, 4)
    parts.append(Part("neck_shade", sp, stri, SKIN_SHADE, Z["neck_shade"]))
    # neck side lines
    for side in (1, -1):
        ln = catmull_rom([(3.2 * side, -10.8), (3.35 * side, -13.8), (3.9 * side, -16.4)], per_seg=10)
        ln = resample(ln, 24)
        lp, lt = stroke(ln, 0.11 * taper(24, 0.2, 0.3))
        parts.append(Part(("R_" if side > 0 else "L_") + "neck_line", lp, lt, SKIN_LINE, Z["neck_line"]))
    # collarbone hints in the V
    for side in (1, -1):
        ln = resample(np.array([(0.9 * side, -18.9), (2.4 * side, -18.3)]), 8)
        lp, lt = stroke(ln, 0.08 * taper(8, 0.4, 0.4))
        parts.append(Part(("R_" if side > 0 else "L_") + "clavicle", lp, lt, (0.86, 0.62, 0.60), Z["neck_line"]))
    return parts


def body_parts():
    parts = []
    # blouse / torso silhouette with a V-neck notch
    right = [(0.0, -22.4), (1.6, -20.0), (3.2, -17.1), (4.3, -16.25),
             (7.8, -17.2), (12.2, -18.4), (15.3, -20.2), (16.9, -23.4),
             (17.4, -28.0), (17.6, -33.0), (17.8, -37.5), (0.0, -37.5)]
    left = [(-x, y) for (x, y) in reversed(right[1:-1])]
    ctrl = right + left
    outline = catmull_rom(ctrl, closed=True, per_seg=8)
    pts, tris, nb = fill_polygon(outline, 1.1)
    ax = np.abs(pts[:, 0])
    col = mix_colors(BLOUSE, BLOUSE_SHADE, smoothstep(13.5, 17.0, ax) * 0.85)
    # arm crease shading
    col = mix_colors(col, BLOUSE_SHADE, np.exp(-((ax - 12.9) / 0.9) ** 2) * smoothstep(-24.0, -28.0, pts[:, 1]) * 0.9)
    parts.append(Part("torso", pts, tris, col, Z["torso"]))
    # the collar's cast shadow on the blouse, just under its outer edge
    for side in (1, -1):
        edge = catmull_rom([(0.9, -26.9), (5.0, -25.55), (9.4, -23.9), (13.6, -22.2),
                            (15.6, -20.6)], per_seg=10)
        edge = resample(edge, 50)
        low = edge + np.array([0.0, -0.95]) * taper(50, 0.25, 0.35)[:, None]
        top = edge + np.array([0.0, 0.4])
        if side < 0:
            low, top = mirror(low), mirror(top)
        sp, st, sv = band(low, top, 3)
        scol = mix_colors(BLOUSE, BLOUSE_SHADE, np.repeat(0.35 + 0.65 * sv, 50))
        parts.append(Part(("R_" if side > 0 else "L_") + "collar_shadow", sp, st, scol, Z["torso_shade"]))
    for side in (1, -1):
        sh = catmull_rom([(4.3, -16.25), (7.8, -17.2), (12.2, -18.4), (15.3, -20.2),
                          (16.9, -23.4), (17.4, -28.0), (17.6, -33.0), (17.8, -37.5)], per_seg=10)
        sh = resample(sh, 90)
        if side < 0:
            sh = mirror(sh)
        sp, st = stroke(sh, 0.16 * taper(90, 0.05, 0.0))
        parts.append(Part(("R_" if side > 0 else "L_") + "torso_line", sp, st, CLOTH_LINE, Z["torso_line"]))
        # arm/torso crease
        cr = catmull_rom([(12.4 * side, -25.2), (12.8 * side, -30.0), (13.0 * side, -37.0)], per_seg=10)
        cr = resample(cr, 30)
        cp, ct = stroke(cr, 0.12 * taper(30, 0.3, 0.0))
        parts.append(Part(("R_" if side > 0 else "L_") + "arm_line", cp, ct, CLOTH_LINE, Z["torso_line"]))
    # sailor collar flaps
    for side in (1, -1):
        flap = [(0.15, -26.6), (0.95, -24.2), (2.2, -20.4), (3.55, -16.75), (4.5, -15.95),
                (8.2, -16.85), (12.4, -18.15), (15.1, -19.95), (13.4, -22.0),
                (9.2, -23.6), (4.6, -25.3)]
        flap = catmull_rom(flap, closed=True, per_seg=8, alpha=0.5)
        if side < 0:
            flap = mirror(flap)
        fp, ft, nb = fill_polygon(flap, 0.7)
        fy = fp[:, 1]
        col = mix_colors(COLLAR, COLLAR_SHADE, smoothstep(-19.0, -16.6, fy) * 0.8)
        parts.append(Part(("R_" if side > 0 else "L_") + "collar", fp, ft, col, Z["collar"]))
        # white stripe parallel to the outer edge
        stripe = catmull_rom([(1.25, -25.9), (5.0, -24.5), (9.3, -22.85), (13.1, -21.3), (14.1, -20.4)], per_seg=10)
        stripe = resample(stripe, 50)
        if side < 0:
            stripe = mirror(stripe)
        sp, st = stroke(stripe, 0.24 * taper(50, 0.06, 0.12))
        parts.append(Part(("R_" if side > 0 else "L_") + "collar_stripe", sp, st, BLOUSE, Z["collar_stripe"]))
        ol = resample(flap, 160, closed=True)
        op, ot = stroke(ol, 0.12, closed=True)
        parts.append(Part(("R_" if side > 0 else "L_") + "collar_line", op, ot, CLOTH_LINE, Z["collar_line"]))
    parts += ribbon_parts()
    return parts


def ribbon_parts():
    parts = []
    knot_c = np.array([0.0, -25.9])
    # tails
    for side in (1, -1):
        tail = [(0.3, -26.4), (1.5, -28.8), (2.4, -31.6), (2.1, -34.3),
                (1.2, -33.2), (0.35, -34.0), (0.4, -31.0), (-0.1, -27.2)]
        tail = catmull_rom(tail, closed=True, per_seg=6)
        if side < 0:
            tail = mirror(tail)
        tp, tt, nb = fill_polygon(tail, 0.4)
        col = mix_colors(RIBBON, RIBBON_SHADE, smoothstep(-27.0, -33.5, tp[:, 1]) * 0.6)
        parts.append(Part(("R_" if side > 0 else "L_") + "ribbon_tail", tp, tt, col, Z["ribbon_tail"]))
        ol = resample(tail, 70, closed=True)
        op, ot = stroke(ol, 0.11, closed=True)
        parts.append(Part(("R_" if side > 0 else "L_") + "ribbon_tail_line", op, ot, RIBBON_LINE, Z["ribbon_tail"] + 0.005))
    # loops
    for side in (1, -1):
        loop = [(0.4, -25.7), (2.2, -24.2), (4.1, -24.0), (4.6, -25.6),
                (4.0, -27.4), (2.1, -27.2), (0.4, -26.2)]
        loop = catmull_rom(loop, closed=True, per_seg=7)
        if side < 0:
            loop = mirror(loop)
        lp, lt, nb = fill_polygon(loop, 0.35)
        d = np.abs(lp[:, 0])
        col = mix_colors(RIBBON_SHADE, RIBBON, smoothstep(0.4, 2.4, d))
        col = mix_colors(col, (1.0, 0.55, 0.6), np.exp(-((lp[:, 1] + 24.7) / 0.4) ** 2) * smoothstep(1.5, 3.5, d) * 0.7)
        parts.append(Part(("R_" if side > 0 else "L_") + "ribbon_loop", lp, lt, col, Z["ribbon"]))
        ol = resample(loop, 70, closed=True)
        op, ot = stroke(ol, 0.12, closed=True)
        parts.append(Part(("R_" if side > 0 else "L_") + "ribbon_loop_line", op, ot, RIBBON_LINE, Z["ribbon_line"]))
        fold = resample(catmull_rom([(1.0 * side, -25.7), (2.6 * side, -25.3), (3.6 * side, -25.9)], per_seg=8), 16)
        fp, ft = stroke(fold, 0.08 * taper(16, 0.3, 0.3))
        parts.append(Part(("R_" if side > 0 else "L_") + "ribbon_fold", fp, ft, RIBBON_SHADE, Z["ribbon_line"]))
    knot = catmull_rom([(-0.75, -25.2), (0.75, -25.2), (0.85, -26.6), (-0.85, -26.6)], closed=True, per_seg=6)
    kp, kt, nb = fill_polygon(knot, 0.25)
    parts.append(Part("ribbon_knot", kp, kt, mix_colors(RIBBON, RIBBON_SHADE, smoothstep(-25.4, -26.6, kp[:, 1]) * 0.7), Z["ribbon_line"] + 0.004))
    ol = resample(knot, 40, closed=True)
    op, ot = stroke(ol, 0.11, closed=True)
    parts.append(Part("ribbon_knot_line", op, ot, RIBBON_LINE, Z["ribbon_line"] + 0.008))
    return parts


# Assembly


def merge(parts):
    """Concatenate parts: returns dict with xyz, tris, colors, and the
    vertex range of every part."""
    xyz, tris, cols, ranges = [], [], [], {}
    off = 0
    for p in parts:
        xyz.append(p.xyz)
        tris.append(p.tris + off)
        cols.append(p.colors)
        ranges[p.name] = (off, off + len(p.pts))
        off += len(p.pts)
    return dict(xyz=np.vstack(xyz), tris=np.vstack(tris), colors=np.vstack(cols),
                ranges=ranges, parts=parts)


def mesh_groups():
    """The rest-pose art, grouped into the meshes the rig deforms."""
    bang_tips = [strand_spine(c)[-1] for (_, c, _, _) in BANGS]
    return {
        "BackHair": back_hair_parts(),
        "Body": body_parts(),
        "Neck": neck_parts(),
        "Face": face_parts() + forehead_shadow(bang_tips),
        "Eyes": eye(1) + eye(-1),
        "Brows": brow(1) + brow(-1),
        "Mouth": mouth("rest"),
        "FrontHair": front_hair_parts(),
    }
