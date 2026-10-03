"""Kaede's costume: an oversized butter-yellow cardigan worn open over a
white shirt (top button undone), a loosened cobalt ribbon, a lightning-bolt
enamel pin; plus the drop earring, the crossed hair pins and the effect
decals (anger vein, sweat drop, blush)."""

import math

import numpy as np

import design as D
import paint as P
from art import Piece, Strip, grid_from_alpha, sample_at, state
from paint import catmull, hexc, resample, smooth

BODY_TPU = 40


def mirror(pts):
    pts = np.asarray(pts, np.float64)
    m = pts[::-1].copy()
    m[:, 0] *= -1
    return m


CARDI_R = [(4.3, -13.3), (7.5, -14.6), (11.5, -15.9), (15.6, -17.2), (18.6, -18.6), (20.6, -21.0),
           (21.5, -24.5), (22.0, -29.0), (22.3, -34.0)]


def cardigan_outline():
    r = np.array(CARDI_R)
    l = mirror(r)
    l[:, 0] -= 0.25                           # a touch fuller on the screen-left
    return np.vstack([r, [(22.3, -35.0), (-22.6, -35.0)], l])


def v_edge(side):
    """The cardigan's front edge (its button band's inner line), from the
    back of the neck down to the bottom of the frame."""
    pts = [(4.5, -13.0), (4.7, -14.2), (4.1, -16.6), (3.2, -20.0), (2.3, -24.0), (1.6, -28.5), (1.1, -35.0)]
    if side < 0:
        pts = [(-x * 1.0 - 0.15, y) for x, y in pts]
    return resample(catmull(np.array(pts), n=12), n=140)


def shirt_pieces():
    out = []
    L = P.Layer("Shirt", (-9.0, -35.5, 9.0, -11.5), tpu=BODY_TPU, ss=2)
    X, Y = L.XY()
    poly = np.array([(-5.2, -13.0), (-3.5, -12.4), (3.5, -12.4), (5.2, -13.0), (8.5, -16.0),
                     (8.5, -35.5), (-8.5, -35.5), (-8.5, -16.0)])
    m = L.poly(poly)
    L.paint(m, D.SHIRT["base"])
    # shadow under the collar and ribbon, and the right side under the band
    under = (Y > -18.6 + 0.35 * np.sin(X * 1.7)) & (Y < -14.0)
    L.paint_in(under.astype(np.float32) * m, D.SHIRT["shadow"], 1.0)
    right = smooth(1.6, 2.1, X - (-(Y + 13) * 0.10))
    L.paint_in(right * m, D.SHIRT["shadow"], 1.0)
    # placket line and a few folds
    for pts, w0 in (([(0.15, -17.6), (0.05, -24.0), (-0.1, -35.0)], 0.035),
                    ([(-2.2, -21.0), (-1.4, -24.5), (-1.3, -27.5)], 0.03),
                    ([(1.6, -25.0), (0.9, -28.6)], 0.028)):
        sp = resample(catmull(np.array(pts), n=10), n=60)
        w = P.taper(60, w0, head=0.2, tail=0.5)
        L.paint_in(L.stroke(sp, w, w) * m, D.SHIRT["line"], 0.9)
    # buttons down the placket (the top one undone, so it sits open)
    for by in (-22.4, -28.4):
        d = L.disc(0.35, by, 0.20)
        L.paint(d * m, "#ECEBF2")
        L.paint(L.band_inside(d, 0.04) * m, D.SHIRT["line"])
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.9)
    out.append(Piece("Shirt", "Body", -1.00, L, v, t))
    return out


def crisp(L, d):
    return np.clip(d * L.s + 0.5, 0, 1).astype(np.float32)


def cardigan_pieces():
    o = cardigan_outline()
    L = P.Layer("Cardigan", (-23.2, -35.5, 23.0, -12.4), tpu=BODY_TPU, ss=2)
    X, Y = L.XY()
    m = L.poly(o)
    er = v_edge(+1)
    el = v_edge(-1)
    vcut = L.poly(np.vstack([er, el[::-1]]))
    m = m * (1 - vcut)
    L.paint(m, D.CARDI["base"])
    # a quiet knit rib: slightly darker vertical lines that follow the drape
    # (they fan out a little over the shoulders)
    xr = X + 0.035 * X * np.clip(-(Y + 17.0), 0, 20) * 0.0
    rib = np.abs(((xr + 0.3 * np.sin(Y * 0.35)) / 0.62) % 1.0 - 0.5)
    L.paint_in(crisp(L, 0.07 - rib * 0.62) * m * (Y < -16.0), D.CARDI["knit"], 1.0)
    # form shadows (key light upper left): the far (screen-right) arm and the
    # torso's right flank, the near arm's inner edge, under the dropped
    # shoulder, all hard-edged
    arm_r = crisp(L, X - (17.2 - 0.05 * (Y + 22.0) + 0.35 * np.sin((Y + 20) * 0.5)))
    L.paint_in(arm_r * m * crisp(L, -(Y + 18.6)), D.CARDI["shadow"], 1.0)
    flank = crisp(L, X - (11.3 + 0.28 * (Y + 22.0)))
    L.paint_in(flank * crisp(L, 15.6 - X) * m * crisp(L, -(Y + 19.5)), D.CARDI["shadow"], 1.0)
    arm_l_in = crisp(L, X - (-16.4 + 0.05 * (Y + 25))) * crisp(L, (-15.1) - X)
    L.paint_in(arm_l_in * m * crisp(L, -(Y + 21.0)), D.CARDI["shadow"], 1.0)
    # the neck/head casts a soft-cornered hard shadow on the upper chest right
    ns = crisp(L, 1.0 - np.hypot((X - 6.2) / 3.2, (Y + 16.4) / 1.6))
    L.paint_in(ns * m, D.CARDI["shadow"], 1.0)
    # arm / torso separation lines (the arms hang at the sides)
    for sgn in (+1, -1):
        armpit = np.array([(15.4 * sgn, -21.8), (15.2 * sgn, -25.0), (15.5 * sgn, -29.5), (15.9 * sgn, -35.5)])
        sp = resample(catmull(armpit, n=10), n=80)
        w2 = P.taper(80, 0.055, head=0.35, tail=0.0)
        L.paint_in(L.stroke(sp, w2, w2) * m, D.CARDI["line"], 1.0)
        if sgn > 0:
            _, N = sample_at(sp, np.linspace(0, 1, 80))
            w = P.taper(80, 0.9, head=0.3, tail=0.0)
            shp = np.vstack([sp, (sp - N * w[:, None])[::-1]])
            L.paint_in(L.poly(shp) * m, D.CARDI["deep"], 1.0)
    # dropped shoulder seams and two decisive folds hanging from them
    for sgn in (+1, -1):
        seam = np.array([(15.9 * sgn, -16.9), (16.5 * sgn, -18.7), (17.3 * sgn, -20.4)])
        sp = resample(catmull(seam, n=10), n=40)
        w = P.taper(40, 0.034, head=0.25, tail=0.4)
        L.paint_in(L.stroke(sp, w, w) * m, D.CARDI["line"], 0.85)
        fold = np.array([(17.2 * sgn, -20.2), (18.2 * sgn, -22.8), (18.7 * sgn, -26.0)])
        sp = resample(catmull(fold, n=8), n=40)
        w = P.taper(40, 0.045, head=0.12, tail=0.85)
        L.paint_in(L.stroke(sp, w, w) * m, D.CARDI["line"], 0.9)
    # the ribbed button bands along both front edges
    for side, e in ((+1, er), (-1, el)):
        ts = np.linspace(0, 1, 140)
        _, N = sample_at(e, ts)
        out_n = N * (1 if side > 0 else -1)
        tside = np.sign(np.mean(out_n[:, 0])) * side
        band_w = 1.2
        outer = e + out_n * (band_w * tside)
        band = L.poly(np.vstack([e, outer[::-1]])) * m
        L.paint_in(band, D.CARDI["base"], 1.0)
        if side > 0:
            L.paint_in(band * crisp(L, -(Y + 15.6)), D.CARDI["shadow"], 1.0)
        for k in range(1, 5):
            rl = e + out_n * (band_w * tside * k / 5.0)
            w = np.full(140, 0.022)
            L.paint_in(L.stroke(rl, w, w) * band, D.CARDI["rib"], 1.0)
        w = P.taper(140, 0.05, head=0.05, tail=0.0)
        L.paint_in(L.stroke(outer, w, w) * m, D.CARDI["line"], 1.0)
    # the lightning-bolt enamel pin on the screen-left chest
    bolt = np.array([(-10.1, -20.5), (-11.3, -22.6), (-10.4, -22.5), (-11.2, -24.6), (-9.3, -21.9),
                     (-10.3, -21.95), (-9.5, -20.5)])
    bm = L.poly(bolt)
    L.paint(bm, D.RIBBON["base"])
    L.paint_in(np.clip(bm - L.poly(bolt - np.array([0.12, -0.12])), 0, 1) * bm, D.RIBBON["shadow"], 1.0)
    L.paint(L.band_inside(bm, 0.07), D.RIBBON["line"])
    L.paint(L.disc(-10.0, -21.3, 0.1), "#FFFFFF")
    # silhouette line inside the outline (not along the bottom crop), and
    # along the V edges
    rim = L.band_inside(L.poly(o), 0.09) * (Y > -34.8)
    L.paint_in(rim * m, D.CARDI["line"], 1.0)
    for e in (er, el):
        w = P.taper(140, 0.06, head=0.1, tail=0.0)
        L.paint_in(L.stroke(e, w, w) * m, D.CARDI["line"], 1.0)
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.9)
    return [Piece("Cardigan", "Body", -0.92, L, v, t)]


# a shirt collar leaf: stand edge along the neck -> front centre -> point ->
# outer edge, where it slides under the cardigan
COLLAR_R = [(4.35, -12.45), (2.4, -13.45), (0.30, -14.35), (1.15, -15.35), (2.05, -16.55), (3.05, -15.55),
            (4.25, -14.55), (4.75, -13.35)]


def collar_pieces():
    L = P.Layer("Collar", (-5.6, -17.4, 5.6, -11.9), tpu=72, ss=2)
    X, Y = L.XY()
    out_m = np.zeros(X.shape, np.float32)
    for side in (+1, -1):
        pts = np.array(COLLAR_R)
        if side < 0:
            pts = pts.copy()
            pts[:, 0] = -pts[:, 0] - 0.05
            pts[3:6, 1] += np.array([0.08, 0.22, 0.12])        # a touch asymmetric
        poly = catmull(pts, n=8, closed=True)
        m = L.poly(poly)
        L.paint(m, D.SHIRT["base"])
        # the fold of the collar: its upper band (the stand, seen from above)
        stand = catmull(np.array([pts[0], pts[1], pts[2], pts[2] + np.array([0.25 * side, -0.35]),
                                  pts[1] + np.array([0.1 * side, -0.45]), pts[7]]), n=6, closed=True)
        L.paint_in(L.poly(stand) * m, D.SHIRT["shadow"], 1.0)
        if side > 0:
            L.paint_in(m * crisp(L, X - 3.0), D.SHIRT["shadow"], 1.0)
        L.paint(L.band_inside(m, 0.07), D.SHIRT["line"])
        out_m = np.maximum(out_m, m)
    # the collar band behind the neck (visible each side of it)
    band = np.array([(-4.4, -12.3), (-3.4, -12.1), (3.4, -12.1), (4.4, -12.3), (4.5, -12.9), (-4.5, -12.9)])
    bm = L.poly(catmull(band, n=6, closed=True)) * (1 - out_m)
    L.paint(bm, D.SHIRT["shadow"])
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.3)
    return [Piece("Collar", "Collar", -0.85, L, v, t)]


# the ribbon: a loosened bow with two tails (FK chains on the tails)

RIBBON_KNOT = (0.45, -15.55)


def ribbon_parts():
    kx, ky = RIBBON_KNOT
    loops = {
        # drooping, uneven loops: the bow has been loosened
        "LoopL": [(kx - 0.45, ky + 0.15), (kx - 1.7, ky + 0.85), (kx - 3.1, ky + 0.55), (kx - 3.45, ky - 0.65),
                  (kx - 2.6, ky - 1.35), (kx - 0.55, ky - 0.45)],
        "LoopR": [(kx + 0.45, ky + 0.2), (kx + 1.6, ky + 0.6), (kx + 2.8, ky + 0.1), (kx + 2.95, ky - 1.0),
                  (kx + 2.15, ky - 1.45), (kx + 0.55, ky - 0.5)],
    }
    tails = {
        "TailL": ([(kx - 0.3, ky - 0.45), (kx - 1.0, ky - 2.6), (kx - 1.25, ky - 4.9), (kx - 1.85, ky - 7.2)],
                  [(0, 0.46), (0.45, 0.62), (0.9, 0.68), (1.0, 0.68)]),
        "TailR": ([(kx + 0.35, ky - 0.45), (kx + 1.15, ky - 2.3), (kx + 1.75, ky - 4.3), (kx + 1.9, ky - 6.2)],
                  [(0, 0.44), (0.45, 0.58), (0.9, 0.64), (1.0, 0.64)]),
    }
    return loops, tails


def ribbon_pieces():
    out = []
    loops, tails = ribbon_parts()
    kx, ky = RIBBON_KNOT
    for name, (ctrl, widths) in tails.items():
        sp = resample(catmull(np.array(ctrl), n=16), n=80)
        t = np.linspace(0, 1, 80)
        a, b = zip(*widths)
        w = np.interp(t, a, b)
        end = sp[-1]
        _, N = sample_at(sp, np.array([1.0]))
        tan = (sp[-1] - sp[-3])
        tan /= np.linalg.norm(tan)
        L = P.Layer(name, tuple(np.concatenate([sp.min(axis=0) - 1.0, sp.max(axis=0) + 1.0])), tpu=64, ss=2)
        m = L.poly(P.ribbon(sp, w, w))
        # a swallow-tail (V cut) end
        notch = np.array([end + N[0] * 0.8 + tan * 0.3, end - tan * 0.6, end - N[0] * 0.8 + tan * 0.3])
        m = m * (1 - L.poly(notch))
        L.paint(m, D.RIBBON["base"])
        Tm, Vm, Dm = P.strand_coords(L, sp, w, m > 0.01)
        # the tail twists once: the back of the ribbon shows as a hard band
        twist = crisp(L, (Dm if name == "TailL" else -Dm) - 0.08 - 0.25 * np.sin(Tm * 3.0))
        L.paint_in(twist * m * crisp(L, (Tm - 0.3) * 6.0), D.RIBBON["shadow"], 1.0)
        L.paint_in(m * crisp(L, (0.14 - Tm) * 6.0), D.RIBBON["deep"], 1.0)
        L.paint(L.band_inside(m, 0.065), D.RIBBON["line"])
        rgba = L.finish()
        v, tr = grid_from_alpha(rgba, L.bbox, 0.25)
        pc = Piece("Ribbon_" + name, "Ribbon", -0.80 if name == "TailL" else -0.79, L, v, tr)
        pc.rgba = rgba
        pc.meta["spine"] = sp
        out.append(pc)
    # loops + knot: one piece (rides the body)
    L = P.Layer("Bow", (kx - 4.0, ky - 2.1, kx + 3.6, ky + 1.5), tpu=64, ss=2)
    X, Y = L.XY()
    for name, pts in loops.items():
        pts = np.array(pts)
        poly = catmull(pts, n=10, closed=True)
        m = L.poly(poly)
        L.paint(m, D.RIBBON["base"])
        sgn = -1.0 if name == "LoopL" else 1.0
        # the inside of the loop (seen through its opening) and the pinch
        # shadow toward the knot, both hard
        inner = catmull(np.array([pts[0] + np.array([0.3 * sgn, -0.1]), (pts[1] + pts[2]) * 0.5 + np.array([0, -0.35]),
                                  pts[3] + np.array([-0.55 * sgn, 0.05]), (pts[4] + pts[5]) * 0.5 + np.array([0, 0.3])]),
                        n=8, closed=True)
        L.paint_in(L.poly(inner) * m, D.RIBBON["deep"], 1.0)
        pinch = crisp(L, 1.05 - np.hypot(X - kx, (Y - ky) * 1.4))
        L.paint_in(pinch * m, D.RIBBON["shadow"], 1.0)
        # a single highlight streak along the top of the loop
        hl = resample(catmull(np.array([pts[1] + np.array([0.1 * sgn, -0.18]),
                                        (pts[1] + pts[2]) * 0.5 + np.array([0, -0.2]),
                                        pts[2] + np.array([-0.35 * sgn, -0.22])]), n=8), n=30)
        w = P.taper(30, 0.07, head=0.4, tail=0.5)
        L.paint_in(L.stroke(hl, w, w) * m, D.RIBBON["hi"], 1.0)
        L.paint(L.band_inside(m, 0.065), D.RIBBON["line"])
    knot = catmull(np.array([(kx - 0.6, ky + 0.42), (kx + 0.6, ky + 0.46), (kx + 0.68, ky - 0.58),
                             (kx - 0.55, ky - 0.62)]), n=6, closed=True)
    km = L.poly(knot)
    L.paint(km, D.RIBBON["base"])
    L.paint_in(km * crisp(L, X - (kx + 0.05)), D.RIBBON["shadow"], 1.0)
    L.paint(L.band_inside(km, 0.065), D.RIBBON["line"])
    rgba = L.finish()
    v, tr = grid_from_alpha(rgba, L.bbox, 0.25)
    pc = Piece("Ribbon_Bow", "Ribbon", -0.76, L, v, tr)
    pc.rgba = rgba
    out.append(pc)
    return out


# accessories on the head

EARRING = [(7.78, -4.05), (7.80, -4.75), (7.82, -5.45)]      # chain joints (lobe -> charm)


def earring_pieces():
    L = P.Layer("Earring", (7.0, -7.2, 8.6, -3.7), tpu=110, ss=3)
    # a fine chain and a small silver bolt charm
    ch = resample(np.array([EARRING[0], EARRING[1], EARRING[2]]), n=30)
    w = np.full(30, 0.028)
    L.paint(L.stroke(ch, w, w), D.SILVER["shadow"])
    bx, by = EARRING[2]
    bolt = np.array([(bx + 0.02, by + 0.05), (bx - 0.30, by - 0.55), (bx - 0.06, by - 0.52), (bx - 0.24, by - 1.02),
                     (bx + 0.28, by - 0.36), (bx + 0.03, by - 0.38), (bx + 0.22, by + 0.05)])
    m = L.poly(bolt)
    L.paint(m, D.SILVER["base"])
    L.paint_in(m * smooth(bx - 0.05, bx + 0.2, L.XY()[0]), D.SILVER["shadow"], 1.0)
    L.paint(L.band_inside(m, 0.035), D.SILVER["line"])
    L.paint(L.disc(bx - 0.05, by - 0.2, 0.04), D.SILVER["glint"])
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.12)
    pc = Piece("Earring", "Earring", -0.20, L, v, t)
    pc.rgba = rgba
    return [pc]


def pins_pieces():
    """Two lemon hair pins crossed in an X above the tucked (screen-right) side."""
    L = P.Layer("Pins", (5.2, 5.2, 9.2, 9.4), tpu=90, ss=3)
    for a, b in (((5.9, 8.9), (8.4, 6.0)), ((6.1, 6.1), (8.6, 8.7))):
        sp = resample(np.array([a, b]), n=40)
        w = np.full(40, 0.13)
        w[:3] *= np.array([0.5, 0.8, 0.95])
        w[-3:] *= np.array([0.95, 0.8, 0.5])
        m = L.stroke(sp, w, w)
        L.paint(m, D.PIN["base"])
        _, N = sample_at(sp, np.linspace(0, 1, 40))
        L.paint_in(L.stroke(sp - N * 0.06, w * 0.45, w * 0.45) * m, D.PIN["shadow"], 1.0)
        L.paint(L.band_inside(m, 0.04), D.PIN["line"])
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.2)
    pc = Piece("Pins", "Pins", 1.60, L, v, t)
    pc.rgba = rgba
    return [pc]


# effects: collapse keyforms (rest = a zero-area point, the keyform grows them)

def collapse_piece(name, group, z, L, spacing, anchor, key):
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, spacing)
    anchor = np.asarray(anchor, np.float64)
    rest = np.repeat(anchor[None], len(v), axis=0) + (v - anchor) * 0.001

    def pos(s, v=v, rest=rest, key=key):
        k = float(s.get(key, 0.0))
        return rest + (v - rest) * k
    pc = Piece(name, group, z, L, v, t, pos, rest=rest)
    pc.rgba = rgba
    return pc


def blush_pieces():
    out = []
    # the soft patch (translucent material)
    L = P.Layer("BlushPatch", (-7.2, -4.4, 7.2, -1.7), tpu=40, ss=2)
    X, Y = L.XY()
    a = np.zeros(X.shape, np.float32)
    for cx in (-4.6, 4.7):
        a = np.maximum(a, np.clip(1 - ((X - cx) / 1.55) ** 2 - ((Y + 3.05) / 0.62) ** 2, 0, 1) ** 0.8)
    L.col[:] = hexc(D.SKIN["blush"]) * a[..., None] * 0.62
    L.a = a * 0.62
    rgba = L.finish()
    L2 = P.Layer("BlushPatch", L.bbox, tpu=40, ss=1)
    v, t = grid_from_alpha(rgba, L.bbox, 0.35)
    rest = v.copy()
    rest[:, 1] = -3.05 + (v[:, 1] + 3.05) * 0.001
    rest[:, 0] = np.where(v[:, 0] < 0, -4.6, 4.7) + (v[:, 0] - np.where(v[:, 0] < 0, -4.6, 4.7)) * 0.6

    def patch_pos(s, v=v, rest=rest):
        return rest + (v - rest) * float(s.get("blush", 0.0))
    pc = Piece("BlushPatch", "BlushPatch", 0.16, None, v, t, patch_pos, rest=rest)
    pc.rgba = rgba
    pc.bbox = L.bbox
    out.append(pc)
    # the hatch strokes: 6 per cheek, each grows from its own centre
    L = P.Layer("BlushHatch", (-7.0, -4.2, 7.0, -1.9), tpu=90, ss=3)
    centres = []
    for cx in (-4.6, 4.7):
        for k in range(6):
            x = cx - 1.05 + k * 0.42 + (0.05 if k % 2 else 0.0)
            y = -3.0 + (0.08 if k % 2 else -0.04)
            ln = 0.72 if k in (1, 2, 3, 4) else 0.5
            ang = math.radians(90 - 28)
            d = np.array([math.cos(ang), math.sin(ang)])
            sp = resample(np.array([(x, y) - d * ln / 2, (x, y) + d * ln / 2]), n=20)
            w = P.taper(20, 0.045, head=0.45, tail=0.45)
            L.paint(L.stroke(sp, w, w), D.SKIN["hatch"])
            centres.append((x, y))
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.08, dilate=1)
    cen = np.array(centres)
    nearest = np.argmin(((v[:, None, :] - cen[None]) ** 2).sum(-1), axis=1)
    anchor = cen[nearest]
    rest = anchor + (v - anchor) * 0.001

    def hatch_pos(s, v=v, rest=rest):
        k = float(s.get("blush", 0.0))
        return rest + (v - rest) * k
    pc = Piece("BlushHatch", "BlushHatch", 0.17, L, v, t, hatch_pos, rest=rest)
    pc.rgba = rgba
    out.append(pc)
    return out


def fx_pieces():
    out = []
    # anger vein: four hooked strokes in a cross, top right of the forehead
    vx, vy = 5.6, 7.6
    L = P.Layer("Vein", (vx - 2.2, vy - 2.2, vx + 2.2, vy + 2.2), tpu=90, ss=3)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        d = np.array([math.cos(a), math.sin(a)])
        nrm = np.array([-d[1], d[0]])
        pts = np.array([(vx, vy) + d * 0.34 + nrm * 0.44, (vx, vy) + d * 0.85 + nrm * 0.18,
                        (vx, vy) + d * 1.45 + nrm * 0.62])
        sp = resample(catmull(pts, n=10), n=30)
        w = P.taper(30, 0.24, head=0.1, tail=0.5)
        m = L.stroke(sp, w, w)
        L.paint(m, D.FX["vein"])
        L.paint(L.band_inside(m, 0.05), D.FX["vein_line"])
    out.append(collapse_piece("Vein", "Fx", 1.90, L, 0.12, (vx, vy), "vein"))
    # sweat drop by the screen-right temple
    sx, sy = 9.6, 3.2
    L = P.Layer("Sweat", (sx - 0.9, sy - 1.6, sx + 0.9, sy + 1.2), tpu=90, ss=3)
    drop = np.vstack([catmull(np.array([(sx, sy + 0.9), (sx + 0.55, sy - 0.35), (sx, sy - 0.95),
                                        (sx - 0.55, sy - 0.35)]), n=10, closed=True)])
    m = L.poly(drop)
    L.paint(m, D.FX["sweat"])
    L.paint(L.band_inside(m, 0.07), D.FX["sweat_line"])
    L.paint(L.disc(sx - 0.2, sy - 0.35, 0.1), "#FFFFFF")
    out.append(collapse_piece("Sweat", "Fx", 1.92, L, 0.12, (sx, sy + 0.9), "sweat"))
    return out
