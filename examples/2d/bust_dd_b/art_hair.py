"""Shion's hair.

Two kinds of pieces:

* CLUMPS -- a spine with a width profile and optional forked tips (the
  fringe, the long locks, the tucked side, the stray curl). Each is its own
  mesh on its own FK chain, so it can lag and swing independently.
* MASSES -- the crown and the back hair: one silhouette each, with the clump
  divisions painted inside as broken flow lines (they move with the head,
  and the back mass is skinned to two chains).

Painting follows the style guide's hair rules: base + shadow + deep, hard
edges; the shadow where an outer piece overlaps an inner one is the outer
piece's silhouette shifted down and away from the light (so its edge
zigzags along the tips above it); broken highlight DASHES that line up on
one ring round the skull; broken interior lines that fade before the root;
a thin tapered outline.

Design: a warm chestnut bob with a deep side part, asymmetric jaw-length
locks, a tucked ear, small brass pins, and a soft outward break at the nape.
"""

import math

import numpy as np

import paint as P
from paint import catmull, resample, smooth
from parts import Part, finish, painted, hull_mesh

HAIR = dict(base="#553A32", shadow="#392A2B", deep="#241F27", hi="#BC8A6B", hi2="#D5AD85",
            line="#221F26", under="#774E3D", pin="#DAB57C", pin_sh="#97734B", scalp="#D9A79A")
TPU = 60
SHADOW_OFF = np.array([0.30, -0.42])      # light from the upper left
HL_C, HL_R = np.array([-0.4, 2.2]), 8.35   # the highlight ring


class Clump(object):
    def __init__(self, name, ctrl, widths, z, n=72, wr=None, forks=(), shade=+1, chain=3, lines=2,
                 hi=True, deep_root=0.12, under=0.0, outline=1.0, group="Hair"):
        self.name = name
        self.ctrl = np.asarray(ctrl, np.float64)
        self.spine = resample(catmull(self.ctrl, n=24), n=n)
        t = np.linspace(0, 1, n)
        wt, wv = zip(*widths)
        self.wl = np.interp(t, wt, wv) * 0.5
        if wr is None:
            self.wr = self.wl.copy()
        else:
            rt, rv = zip(*wr)
            self.wr = np.interp(t, rt, rv) * 0.5
        self.forks = forks
        self.z = z
        self.shade = shade
        self.chain = chain
        self.lines = lines
        self.hi = hi
        self.deep_root = deep_root
        self.under = under
        self.outline = outline
        self.group = group
        self.kind = "clump"
        self._polys = None

    def polygons(self):
        if self._polys is not None:
            return self._polys
        polys = [P.ribbon(self.spine, self.wl, self.wr)]
        self.fork_spines = []
        for tb, ctrl, widths in self.forks:
            k = int(round(tb * (len(self.spine) - 1)))
            pts = np.vstack([self.spine[max(0, k - 3):k + 1], np.asarray(ctrl, np.float64)])
            sp = resample(catmull(pts, n=16), n=40)
            tt = np.linspace(0, 1, len(sp))
            wt, wv = zip(*widths)
            w = np.interp(tt, wt, wv) * 0.5
            self.fork_spines.append((tb, sp, w))
            polys.append(P.ribbon(sp, w, w))
        self._polys = polys
        return polys

    def bbox(self, pad=0.5):
        pts = np.vstack(self.polygons())
        lo = pts.min(axis=0) - pad
        hi = pts.max(axis=0) + pad
        return (lo[0], lo[1], hi[0], hi[1])


class Mass(object):
    """A painted hair mass: an outline + flow lines (clump divisions)."""

    def __init__(self, name, outline, z, flows=(), shade_fn=None, deep_fn=None, chain=0, group="Hair",
                 hi=True, lit_break=None):
        self.name = name
        self.outline = catmull(np.asarray(outline, np.float64), n=12, closed=True)
        self.z = z
        self.flows = [resample(catmull(np.asarray(f, np.float64), n=16), n=80) for f in flows]
        self.shade_fn = shade_fn
        self.deep_fn = deep_fn
        self.chain = chain
        self.group = group
        self.hi = hi
        self.lit_break = lit_break
        self.kind = "mass"

    def polygons(self):
        return [self.outline]

    def bbox(self, pad=0.5):
        lo = self.outline.min(axis=0) - pad
        hi = self.outline.max(axis=0) + pad
        return (lo[0], lo[1], hi[0], hi[1])


def union_mask(L, polys):
    m = np.zeros((L.H, L.W), np.float32)
    for p in polys:
        m = np.maximum(m, L.poly(p))
    return m


def dash_strokes(spine, wl, wr, shade, seed, count=3):
    """Short tapered highlight dashes where a spine crosses the ring, on the
    lit half of the clump, staggered along it."""
    rng = np.random.default_rng(seed)
    r = np.hypot(spine[:, 0] - HL_C[0], (spine[:, 1] - HL_C[1]) * 1.05)
    k = int(np.argmin(np.abs(r - HL_R)))
    if abs(r[k] - HL_R) > 0.6:
        return []
    n = len(spine)
    d = np.gradient(spine, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-9
    nrm = np.stack([-d[:, 1], d[:, 0]], axis=1)
    out = []
    for j in range(count):
        v = -shade * (0.15 + 0.55 * (j + 0.5) / count + rng.uniform(-0.08, 0.08))
        off = int(rng.uniform(-3, 3)) + (j - count // 2) * 2
        ln = int(max(4, (6 + rng.uniform(-1.5, 2.5)) * n / 72.0))
        a = int(np.clip(k + off - ln // 2, 0, n - 2))
        b = int(np.clip(a + ln, a + 2, n - 1))
        w = np.where(v > 0, wl[a:b], wr[a:b])
        sp = spine[a:b] + nrm[a:b] * (v * w)[:, None]
        out.append((resample(sp, n=16), 0.055 + 0.03 * rng.random()))
    return out


def paint_clump(c, above_polys, tpu=TPU):
    L = P.Layer(c.name, c.bbox(), tpu=tpu, ss=3)
    X, Y = L.XY()
    polys = c.polygons()
    m = union_mask(L, polys)
    region = m > 0.001
    hw = np.maximum(c.wl + c.wr, 1e-3) * 0.5
    T, V, Dd = P.strand_coords(L, c.spine, hw, region)
    nn = len(c.wl)
    V = np.where(Dd > 0, Dd / np.interp(np.clip(T, 0, 1), np.linspace(0, 1, nn), np.maximum(c.wl, 1e-3)),
                 Dd / np.interp(np.clip(T, 0, 1), np.linspace(0, 1, nn), np.maximum(c.wr, 1e-3)))
    for tb, sp, w in getattr(c, "fork_spines", []):
        Tf, Vf, Df = P.strand_coords(L, sp, w, region)
        closer = np.abs(Df) < np.abs(Dd)
        T = np.where(closer, tb + (1 - tb) * Tf, T)
        V = np.where(closer, Vf, V)
        Dd = np.where(closer, Df, Dd)
    s = V * c.shade                     # +1 = the shadow-side edge
    L.paint(m, HAIR["base"])
    # the form shadow: a hard band on the shadow side, its edge a long
    # shallow zigzag, running out before the tip
    phase = (sum(map(ord, c.name)) % 97) / 97.0
    teeth = P.sawtooth(T * 3.2 + phase, 1.0, 0.82) * 0.30
    edge = 0.18 + teeth + 0.9 * smooth(0.80, 1.0, T)
    L.paint_in(np.clip((s - edge) * 9.0, 0, 1), HAIR["shadow"])
    # the underside colour on the flicked ends
    if c.under > 0:
        L.paint_in(np.clip((T - (1.0 - c.under)) * 40, 0, 1) * np.clip((s + 0.2) * 6, 0, 1), HAIR["under"])
    # cast shadow of the pieces above, and the deep tone at the root
    if above_polys:
        sh = union_mask(L, [p + SHADOW_OFF for p in above_polys])
        L.paint_in(sh, HAIR["shadow"])
    length = float(np.sum(np.linalg.norm(np.diff(c.spine, axis=0), axis=1)))
    droot = min(c.deep_root, 0.9 / max(length, 1e-3))
    L.paint_in(np.clip((droot - T) / max(droot, 1e-3) * 4.0, 0, 1) * (T >= 0), HAIR["deep"])
    # broken interior lines: from the tip end, fading before the root
    rng = np.random.default_rng(sum(map(ord, c.name)) + 3)
    hwt = np.interp(np.clip(T, 0, 1), np.linspace(0, 1, len(hw)), hw)
    for k in range(c.lines):
        v0 = -0.5 + 1.0 * (k + 0.5) / max(c.lines, 1) + rng.uniform(-0.12, 0.12)
        t0 = rng.uniform(0.25, 0.5)
        t1 = rng.uniform(0.86, 0.98)
        wid = 0.034 * smooth(t0, t0 + 0.25, T) * (1 - smooth(t1 - 0.12, t1, T))
        line = np.clip((wid - np.abs(V - v0) * hwt) * L.s + 0.5, 0, 1)
        L.paint_in(line * (T >= 0), HAIR["line"], 0.85)
    # highlight dashes
    if c.hi:
        for sp, wd in dash_strokes(c.spine, c.wl, c.wr, c.shade, sum(map(ord, c.name))):
            L.paint_in(L.stroke(sp, P.taper(16, wd, head=0.45, tail=0.45)), HAIR["hi"])
    # line art inside the silhouette, tapering toward the tips
    lw = (0.07 * (1 - 0.72 * smooth(0.45, 1.0, T)) + 0.014) * c.outline
    lw = lw * smooth(0.0, 0.06, T)
    L.paint_in(L.band_inside(m, lw), HAIR["line"])
    return L


def paint_mass(M, above_polys, tpu=TPU):
    L = P.Layer(M.name, M.bbox(), tpu=tpu, ss=3)
    X, Y = L.XY()
    m = L.poly(M.outline)
    L.paint(m, HAIR["base"])
    if M.shade_fn is not None:
        L.paint_in(M.shade_fn(L, X, Y), HAIR["shadow"])
    if above_polys:
        L.paint_in(union_mask(L, [p + SHADOW_OFF for p in above_polys]), HAIR["shadow"])
    if M.deep_fn is not None:
        L.paint_in(M.deep_fn(L, X, Y), HAIR["deep"])
    # flow lines: broken, tapered, fading out toward their ends
    for k, f in enumerate(M.flows):
        n = len(f)
        t = np.linspace(0, 1, n)
        a, b = (0.08 + 0.1 * ((k * 7) % 3) / 2.0), (0.78 + 0.18 * ((k * 5) % 3) / 2.0)
        w = 0.036 * smooth(a, a + 0.18, t) * (1 - smooth(b - 0.2, b, t))
        L.paint_in(L.stroke(f, w), HAIR["line"], 0.9)
    if M.hi:
        for k, f in enumerate(M.flows):
            d = np.gradient(f, axis=0)
            d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-9
            nrm = np.stack([-d[:, 1], d[:, 0]], axis=1)
            r = np.hypot(f[:, 0] - HL_C[0], (f[:, 1] - HL_C[1]) * 1.05)
            i = int(np.argmin(np.abs(r - HL_R)))
            if abs(r[i] - HL_R) > 0.7:
                continue
            rng = np.random.default_rng(31 + k)
            for j in range(3):
                off = (j - 1) * 5 + int(rng.uniform(-2, 2))
                ln = int(7 + rng.uniform(-2, 3))
                a = int(np.clip(i + off - ln // 2, 0, len(f) - 2))
                b = int(np.clip(a + ln, a + 2, len(f) - 1))
                v = -(0.35 + 0.22 * j + rng.uniform(-0.1, 0.1))
                side = 1.0 if f[i, 0] > HL_C[0] else -1.0
                sp = f[a:b] + nrm[a:b] * v * side * 0.9
                L.paint_in(L.stroke(resample(sp, n=16), P.taper(16, 0.06 + 0.025 * rng.random(), head=0.45, tail=0.45)) * m,
                           HAIR["hi"])
    # outline (with a lit-side break where the light hits the top)
    d = L.inside_dist(m)
    lw = np.full(X.shape, 0.085, np.float32)
    if M.lit_break is not None:
        lw = lw * M.lit_break(X, Y)
    L.paint_in(np.clip((lw - d) * L.s + 0.5, 0, 1) * m, HAIR["line"])
    return L


# the design

PART = (-2.55, 11.9)            # where the part line goes over the top
PART_FRONT = (-2.75, 9.2)       # where it meets the hairline


def crown():
    outline = [(-8.35, 2.2), (-8.95, 5.6), (-8.35, 8.7), (-6.4, 10.85), (-3.3, 11.85), (-2.55, 11.95),
               (-1.9, 11.97), (1.2, 11.9), (4.4, 11.35), (7.2, 9.95), (8.95, 7.6), (9.55, 4.6), (9.45, 1.4),
               # hairline (inner edge, under the fringe): small pointed tips
               (8.6, 1.9), (8.3, 3.6), (7.7, 4.4), (7.3, 5.0), (6.2, 6.1), (5.4, 6.3), (4.2, 7.2),
               (2.4, 7.7), (1.6, 7.5), (0.2, 8.1), (-1.6, 8.25), (-2.75, 8.5), (-3.8, 8.1), (-5.2, 7.6),
               (-6.1, 6.6), (-6.9, 5.8), (-7.3, 4.3), (-7.6, 2.8)]
    flows = [
        # right of the part: sweeping over the top and down the right side
        [(-1.9, 11.7), (1.6, 11.3), (5.2, 9.9), (7.9, 7.2), (9.0, 3.8)],
        [(-2.0, 11.0), (1.0, 10.4), (4.3, 9.2), (6.8, 7.0), (8.0, 4.4)],
        [(-2.2, 10.1), (0.4, 9.5), (3.2, 8.6), (5.6, 7.1)],
        [(-0.6, 11.9), (3.2, 11.3), (6.4, 9.7), (8.6, 6.6)],
        # left of the part: down the left side
        [(-3.1, 11.4), (-5.6, 10.5), (-7.6, 8.4), (-8.3, 5.0)],
        [(-3.2, 10.4), (-5.2, 9.4), (-6.8, 7.6), (-7.5, 4.8)],
    ]

    def shade(L, X, Y):
        # the right side of the head turns from the light: a big shape with a
        # jagged edge that follows the flow of the clumps
        # a curved terminator following the skull, its edge broken by the
        # clump tips above it
        r = np.hypot(X + 2.5, (Y - 1.5) * 0.92)
        edge = 10.4 + 0.35 * P.sawtooth(np.arctan2(Y - 1.5, X + 2.5) * 7.0, 1.0, 0.75)
        right = np.clip((r - edge) * L.s * 0.5 + 0.5, 0, 1) * (X > 1.0)
        # plus the lower band along the hairline on the left
        low = np.clip((5.2 - Y + 0.35 * P.sawtooth(X * 0.9, 1.0, 0.8)) * L.s * 0.5 + 0.5, 0, 1) * (X < 0)
        return np.maximum(right, low)

    def deep(L, X, Y):
        # the part: a thin dark wedge; the hairline edge
        part = L.stroke(resample(np.array([PART, (-2.62, 10.6), PART_FRONT]), n=24),
                        P.taper(24, 0.10, head=0.2, tail=0.6))
        m = L.poly(catmull(np.array(crown_outline_pts()), n=12, closed=True))
        d = L.inside_dist(m)
        lowedge = np.clip((0.22 - d) * L.s + 0.5, 0, 1) * np.clip((6.5 - Y) * 2.0, 0, 1) * np.clip((8.85 - np.abs(X)) * 3, 0, 1)
        return np.maximum(part, lowedge * (Y < 8.6) * (np.hypot(X + 0.4, Y - 2.0) < 7.8))

    def lit_break(X, Y):
        a = np.arctan2(Y - 2.0, X + 0.5)
        return 1.0 - 0.85 * np.exp(-((a - 1.95) / 0.28) ** 2)

    return Mass("Crown", outline, z=0.40, flows=flows, shade_fn=shade, deep_fn=deep, lit_break=lit_break)


def crown_outline_pts():
    return crown_outline_cache


def back_mass():
    outline=[(-7.6,8.8),(-10.,4.),(-11.,-3.),(-10.7,-11.),(-10.,-15.),
             (-7.,-17.),(-3.,-16.8),(3.,-16.8),(8.,-17.),(11.,-13.),
             (11.5,-7.),(10.6,3.),(8.4,8.8),(0.,10.4)]
    flows=[[(-8.4,5.),(-9.4,-2.),(-9.5,-10.),(-7.8,-15.7)],
           [(-7.4,3.),(-8.4,-4.),(-8.5,-11.),(-6.4,-16.)],
           [(8.8,5.),(10.,-2.),(10.,-9.),(8.4,-15.7)],
           [(8.,3.),(8.8,-4.),(8.7,-11.),(6.5,-16.)]]
    def shade(L,X,Y):
        return (X>8.5-.08*(Y+4)).astype(float)
    def deep(L,X,Y):
        return (np.abs(X)<7.4)*(Y<1.)
    return Mass('BackHair',outline,z=-2.95,flows=flows,shade_fn=shade,deep_fn=deep,chain=4,hi=False)


def clumps():
    C = []
    C.append(Clump("TuckL", [(-7.3, 6.4), (-8.8, 3.4), (-9.5, -0.6), (-9.6, -5.0), (-9.0, -9.6), (-8.6, -12.0)],
                   [(0, 1.8), (0.3, 2.1), (0.7, 1.6), (1.0, 0.0)], z=-0.30, chain=3, lines=2, shade=+1,
                   hi=False))
    C.append(Clump("LockR_A", [(3.6, 10.75), (7.2, 8.5), (9.3, 3.), (10., -3.), (10.2, -10.5), (8.7, -15.2)],
                   [(0, 0.6), (0.06, 2.0), (0.2, 2.7), (0.5, 2.8), (0.82, 3.1), (0.94, 1.8), (1.0, 0.0)], z=0.54, chain=4,
                   lines=3, shade=-1, under=0.09,
                   forks=[(0.87, [(10.5, -14.3), (9.6, -16.4)], [(0, 1.0), (1, 0.0)])]))
    C.append(Clump("LockR_B", [(5.4, 9.7), (7.9, 5.), (8.9, -1.), (8.9, -8.), (7.5, -14.)],
                   [(0, 0.5), (0.07, 1.5), (0.25, 1.8), (0.6, 2.2), (0.88, 1.4), (1.0, 0.0)], z=0.56, chain=4, lines=2,
                   shade=+1, under=0.08))
    C.append(Clump("LooseL", [(-6.9, 6.6), (-7.9, 3.4), (-7.85, 0.0), (-7.35, -2.8)],
                   [(0, 0.6), (0.35, 0.52), (1.0, 0.0)], z=0.47, chain=3, lines=0, shade=+1, hi=False,
                   outline=0.9))
    C.append(Clump("F4", [(-3.05, 10.9), (-4.6, 9.2), (-5.8, 6.6), (-6.25, 3.3)],
                   [(0, 0.9), (0.3, 1.6), (0.6, 1.4), (1.0, 0.0)], z=0.62, chain=3, lines=1, shade=+1,
                   forks=[(0.66, [(-5.3, 4.5), (-5.05, 3.6)], [(0, 0.55), (1, 0.0)])]))
    C.append(Clump("F5", [(3.4, 8.4), (6.0, 6.1), (7.25, 2.8), (7.35, -0.8), (6.8, -3.5)],
                   [(0, 1.3), (0.3, 1.55), (0.7, 1.15), (1.0, 0.0)], z=0.64, chain=4, lines=1, shade=-1))
    C.append(Clump("F3", [(-2.8, 10.4), (-2.95, 8.2), (-2.4, 5.8), (-1.4, 3.6)],
                   [(0, 0.8), (0.3, 1.7), (0.7, 1.3), (1.0, 0.0)], z=0.66, chain=3, lines=1, shade=-1))
    C.append(Clump("F2", [(-2.55, 10.9), (-1.9, 8.6), (-0.3, 6.3), (1.4, 4.6), (2.6, 3.5)],
                   [(0, 0.9), (0.25, 2.2), (0.65, 2.1), (1.0, 0.0)], z=0.70, chain=3, lines=1, shade=-1,
                   forks=[(0.72, [(2.0, 4.4), (2.15, 3.65)], [(0, 0.55), (1, 0.0)])]))
    C.append(Clump("F1", [(-2.3, 11.35), (-0.4, 9.8), (2.6, 7.2), (5.1, 4.6), (6.9, 2.6)],
                   [(0, 1.0), (0.2, 2.4), (0.55, 2.6), (0.8, 1.8), (1.0, 0.0)], z=0.80, chain=4, lines=2,
                   shade=-1, forks=[(0.80, [(5.3, 3.4), (5.75, 2.5)], [(0, 0.75), (1, 0.0)])]))
    C.append(Clump("Stray", [(-2.2, 11.2), (0.4, 9.2), (3.0, 6.0), (4.6, 3.4)],
                   [(0, 0.20), (0.6, 0.16), (1.0, 0.0)], z=0.86, chain=3, lines=0, hi=False, outline=1.5,
                   shade=-1))
    C.append(Clump("Ahoge", [(-1.6, 11.3), (-1.0, 12.5), (0.3, 13.0), (1.1, 12.4)],
                   [(0, 0.24), (0.5, 0.2), (1.0, 0.0)], z=0.38, chain=3, lines=0, hi=False, outline=1.7))
    return C


crown_outline_cache = crown().outline


PINS = [((-6.75, 8.2), -38.0), ((-6.2, 7.35), -38.0)]


def paint_pins():
    L = P.Layer("Pins", (-8.5, 5.9, -4.4, 9.7), tpu=TPU * 2, ss=3)
    for (x, y), ang in PINS:
        a = math.radians(ang)
        d = np.array([math.cos(a), math.sin(a)])
        c = np.array([x, y])
        sp = resample(np.array([c - d * 0.8, c + d * 0.8]), n=20)
        m = L.stroke(sp, np.full(20, 0.10))
        L.paint(m, HAIR["pin"])
        nrm = np.array([-d[1], d[0]])
        L.paint_in(L.stroke(sp - nrm * 0.05, np.full(20, 0.035)), HAIR["pin_sh"])
        L.paint_in(L.band_inside(m, 0.028), "#3A2A16")
        g = c - d * 0.45 + nrm * 0.03
        L.paint_in(L.disc(g[0], g[1], 0.035), "#FFFFFF")
    return L


def pieces():
    return [back_mass(), crown()] + clumps()


def fringe_shadow_fn(pcs, names=("F1", "F2", "F3", "F4", "F5", "Stray", "Crown", "LooseL"), offset=(0.30, -0.46)):
    polys = []
    for c in pcs:
        if c.name in names:
            polys += [p + np.array(offset) for p in c.polygons()]

    def fn(L):
        m = np.zeros((L.H, L.W), np.float32)
        for p in polys:
            m = np.maximum(m, L.poly(p))
        return m
    return fn


def chain_points(c):
    """Joint positions along a clump's spine (the FK chain)."""
    if c.chain <= 0 or c.kind != "clump":
        return [], []
    n = len(c.spine)
    if c.chain == 1:
        ts = [0.0]
    else:
        ts = [0.0] + [0.2 + 0.68 * k / (c.chain - 2) for k in range(c.chain - 1)] if c.chain > 2 else [0.0, 0.45]
    ts = np.clip(np.array(ts), 0, 0.9)
    return [c.spine[int(round(t * (n - 1)))] for t in ts], list(ts)


def build(log=print):
    pcs = pieces()
    parts, texes = [], {}
    for c in pcs:
        above = []
        for o in pcs:
            if o.z > c.z and o.name not in ("Ahoge",):
                above += o.polygons()
        t = painted(c.name, lambda c=c, above=above: paint_clump(c, above) if c.kind == "clump" else paint_mass(c, above))
        texes[t.name] = t
        h = 0.34 if c.kind == "clump" else 0.55
        if c.name == "BackHair":
            h = 0.8
        pts, tris = hull_mesh(t.rgba, t.bbox, h, h * 1.3, pad_texels=3)
        parts.append(Part(c.name, c.group, c.z, c.name, pts, tris, extra=dict(piece=c)))
    t = painted("Pins", paint_pins)
    texes[t.name] = t
    pts, tris = hull_mesh(t.rgba, t.bbox, 0.18, 0.25, pad_texels=3)
    parts.append(Part("Pins", "Hair", 0.95, "Pins", pts, tris))
    return parts, texes, pcs
