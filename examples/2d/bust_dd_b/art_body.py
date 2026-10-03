"""Shion's costume: a white shirt worn with the top button undone, a thin
slate-blue ribbon tie tied loosely, and an oversized camel cardigan worn
open over it (dropped shoulders, ribbed front bands, a few decisive folds).

Layers back to front: CollarBack (behind the neck) < Neck < Shirt <
Cardigan < Collar < Ribbon knot / tails. Hard cel: base + shadow (+ deep in
the armpits and under the collar); white cloth shadows are cool lavender,
the camel's shadows shift toward red-brown.
"""

import math

import numpy as np

import paint as P
from paint import catmull, resample, smooth
from parts import Part, finish, painted, hull_mesh

TPU = 40
CARD = dict(base="#C9A274", shadow="#A0775A", deep="#6F4C3E", line="#4A3026", rib="#A98056", hi="#DDBB8E")
SHIRT = dict(base="#F7F7F4", shadow="#D5D3E4", deep="#B7B3CC", line="#5E5870", button="#E9E6EE")
RIB = dict(base="#3E5A7E", shadow="#2A3F5C", hi="#6E8DB3", line="#1B2536")
OUT = "#2A1E22"

Z = dict(CardiganBack=-1.95, CollarBack=-1.80, Shirt=-1.40, Cardigan=-1.20, Collar=-1.00, RibbonKnot=-0.85,
         RibbonTailL=-0.90, RibbonTailR=-0.88)

BODY_BOX = (-24.0, -36.0, 24.0, -12.0)


def mirror(pts):
    p = np.asarray(pts, np.float64).copy()
    p[:, 0] *= -1
    return p


# silhouettes

def cardigan_outer():
    r = [(3.1, -15.0), (5.2, -14.6), (8.6, -15.6), (13.2, -16.9), (17.4, -18.2), (19.9, -19.8), (21.1, -22.8),
         (21.7, -27.0), (22.0, -32.0), (22.1, -36.0)]
    l = [(-3.0, -15.1), (-5.1, -14.5), (-8.8, -15.5), (-13.4, -16.8), (-17.6, -18.1), (-20.1, -19.7), (-21.3, -22.8),
         (-21.9, -27.0), (-22.2, -32.0), (-22.3, -36.0)]
    top = catmull(np.array(l[::-1] + r), n=16)
    return np.vstack([top, [(22.1, -37.0), (-22.3, -37.0)]])


def v_opening():
    """The open front of the cardigan (screen-left edge, screen-right edge)."""
    r = [(3.35, -15.2), (4.3, -18.5), (5.2, -22.5), (5.9, -27.0), (6.35, -31.5), (6.6, -37.0)]
    l = [(-3.25, -15.3), (-4.1, -18.7), (-4.9, -22.6), (-5.5, -27.0), (-5.9, -31.5), (-6.1, -37.0)]
    return np.array(l), np.array(r)


def shirt_outline():
    # the top edge dips into the open collar (top button undone): the neck
    # shows in the V down to the first fastened button
    return np.array([(-5.2, -14.4), (-1.4, -15.2), (0.05, -17.9), (1.45, -15.2), (5.2, -14.4),
                     (8.0, -37.0), (-8.0, -37.0)])


def cardigan_back():
    """The cardigan's back neckline, standing behind the neck."""
    return np.array([(-6.4, -15.2), (-4.9, -13.55), (-2.4, -12.95), (2.4, -12.95), (4.9, -13.55),
                     (6.4, -15.2), (0.0, -16.5)])


def collar_shapes():
    """Front collar points (left, right): each a closed polygon; the collar
    stands round the neck and lies open (top button undone)."""
    right = np.array([(0.35, -15.55), (1.6, -14.5), (3.4, -13.55), (4.25, -13.75), (4.05, -15.1),
                      (3.3, -16.8), (2.35, -18.35), (1.55, -17.2)])
    left = np.array([(-0.25, -15.6), (-1.5, -14.5), (-3.3, -13.5), (-4.2, -13.7), (-4.0, -15.0),
                     (-3.3, -16.7), (-2.45, -18.05), (-1.45, -17.05)])
    return left, right


def collar_back():
    return np.array([(-4.3, -12.95), (-2.0, -12.35), (2.0, -12.35), (4.35, -13.0), (4.5, -14.6), (-4.5, -14.6)])


# painting

def paint_collar_back():
    L = P.Layer("CollarBack", (-5.5, -16.0, 5.5, -12.0), tpu=TPU * 2, ss=3)
    m = L.poly(catmull(collar_back(), n=6, closed=True))
    L.paint(m, SHIRT["shadow"])
    X, Y = L.XY()
    L.paint_in(np.clip((-13.3 - Y) * L.s + 0.5, 0, 1), SHIRT["deep"])
    L.paint_in(L.band_inside(m, 0.07), SHIRT["line"])
    return L


def paint_cardigan_back():
    L = P.Layer("CardiganBack", (-7.2, -17.0, 7.2, -12.4), tpu=TPU * 2, ss=3)
    X, Y = L.XY()
    m = L.poly(catmull(cardigan_back(), n=6, closed=True))
    L.paint(m, CARD["shadow"])
    for k in range(-24, 25):
        x = k * 0.28
        stripe = np.clip((0.035 - np.abs(X - x)) * L.s + 0.5, 0, 1)
        L.paint_in(stripe * m, CARD["deep"], 0.55)
    L.paint_in(L.band_inside(m, 0.07), CARD["line"])
    return L


def paint_shirt():
    L = P.Layer("Shirt", (-8.5, -37.5, 8.5, -12.8), tpu=TPU, ss=3)
    X, Y = L.XY()
    m = L.poly(shirt_outline())
    L.paint(m, SHIRT["base"])
    # the head/collar/ribbon shadow on the chest: a hard band under the
    # collar, more on the shadow (right) side
    cut = -17.4 - 0.25 * X + 0.4 * np.sin(X * 1.3)
    L.paint_in(np.clip((Y - cut) * L.s + 0.5, 0, 1), SHIRT["shadow"])
    # the cardigan's shadow along both edges (a strip inside the V)
    lv, rv = v_opening()
    for edge, sgn in ((lv, 1), (rv, -1)):
        ex = np.interp(-Y, -edge[:, 1], edge[:, 0])
        L.paint_in(np.clip((ex + sgn * 0.55 - X) * sgn * L.s + 0.5, 0, 1) * (Y < -15), SHIRT["shadow"])
    # placket line and one button
    sp = resample(np.array([(0.15, -18.2), (0.25, -26.0), (0.35, -37.0)]), n=30)
    L.paint_in(L.stroke(sp, np.full(30, 0.03)), SHIRT["line"], 0.8)
    L.paint_in(L.disc(0.55, -22.4, 0.19), SHIRT["button"])
    ring = L.disc(0.55, -22.4, 0.21) * (1 - L.disc(0.55, -22.4, 0.15))
    L.paint_in(ring, SHIRT["line"], 0.9)
    # a skin V at the open neck (the top button undone) is the neck layer
    return L


def paint_cardigan(cast=None):
    L = P.Layer("Cardigan", BODY_BOX, tpu=TPU, ss=3)
    X, Y = L.XY()
    lv, rv = v_opening()
    outer = cardigan_outer()
    vpoly = np.vstack([lv, rv[::-1]])
    m = L.poly(outer) * (1 - L.poly(vpoly))
    L.paint(m, CARD["base"])
    # --- rib bands along the front edges -------------------------------------
    bands = np.zeros_like(m)
    for edge, sgn in ((lv, -1), (rv, 1)):
        ex = np.interp(-Y, -edge[:, 1], edge[:, 0])
        band = np.clip((X - ex) * sgn * L.s + 0.5, 0, 1) * np.clip((ex + sgn * 1.0 - X) * sgn * L.s + 0.5, 0, 1)
        bands = np.maximum(bands, band * (Y < -14.6))
        # rib lines: short vertical ticks across the band
        ribs = (np.abs(((Y + 0.0) * 3.2) % 1.0 - 0.5) < 0.09) * band
        L.paint_in(ribs * 0.0, CARD["rib"])
        seam = np.clip((0.05 - np.abs(X - (ex + sgn * 1.0))) * L.s + 0.5, 0, 1) * (Y < -15)
        L.paint_in(seam, CARD["line"], 0.8)
    # vertical rib stripes inside the bands
    for edge, sgn in ((lv, -1), (rv, 1)):
        ex = np.interp(-Y, -edge[:, 1], edge[:, 0])
        for f in (0.33, 0.66):
            stripe = np.clip((0.028 - np.abs(X - (ex + sgn * f))) * L.s + 0.5, 0, 1) * (Y < -15.3)
            L.paint_in(stripe * m, CARD["rib"])
    # buttons on the screen-left band
    for by in (-25.6, -32.4):
        bx = np.interp(-by, -lv[:, 1], lv[:, 0]) - 0.5
        L.paint_in(L.disc(bx, by, 0.36), "#6B4A36")
        L.paint_in(L.disc(bx - 0.08, by + 0.08, 0.12), "#8C6A50")
    # --- cel shading: light from the upper left -------------------------------
    # the right sleeve and side in shadow, a lit left shoulder
    shade_r = np.clip((X - (17.2 - 0.18 * (Y + 20))) * L.s * 0.5 + 0.5, 0, 1)
    L.paint_in(shade_r, CARD["shadow"])
    # dropped shoulder seams (curves from the shoulder down the arm)
    for sgn in (1, -1):
        seam = catmull(np.array([(14.6 * sgn, -17.4), (15.7 * sgn, -21.0), (16.1 * sgn, -26.0), (16.2 * sgn, -37.0)]), n=16)
        seam = resample(seam, n=60)
        L.paint_in(L.stroke(seam, P.taper(60, 0.05, head=0.1, tail=0.05)), CARD["line"], 0.9)
        # the armpit / side: deep fold shadow inside the seam
        sh = catmull(np.array([(15.9 * sgn, -23.0), (16.6 * sgn, -27.0), (16.8 * sgn, -37.0), (15.3 * sgn, -37.0),
                               (15.4 * sgn, -30.0)]), n=10, closed=True)
        L.paint_in(L.poly(sh), CARD["shadow"])
    # two decisive Y folds from the shoulder seam (tapered line + hard shadow)
    for sgn, pts in ((1, [(14.4, -18.2), (13.2, -21.6), (12.9, -25.4)]),
                     (-1, [(-14.6, -18.1), (-13.5, -21.8), (-13.2, -25.8)])):
        sp = resample(catmull(np.array(pts), n=10), n=30)
        L.paint_in(L.stroke(sp, P.taper(30, 0.05, head=0.15, tail=0.85)), CARD["line"], 0.9)
        shp = np.vstack([sp, sp[::-1] + np.array([0.34 * sgn, -0.10])])
        L.paint_in(L.poly(shp), CARD["shadow"], 0.95)
    # sleeve compression folds (lower arm, near the frame bottom)
    for sgn in (1, -1):
        for y0, ln in ((-29.5, 1.8), (-32.2, 2.2)):
            sp = resample(catmull(np.array([(18.2 * sgn, y0), (19.3 * sgn, y0 - 0.35), (18.2 * sgn + ln * 0.9 * sgn, y0 - 0.2)]), n=8), n=24)
            L.paint_in(L.stroke(sp, P.taper(24, 0.04, head=0.3, tail=0.7)), CARD["line"], 0.8)
    # the long lock's cast shadow on the cardigan
    if cast is not None:
        L.paint_in(cast(L), CARD["shadow"])
    # shadow under the collar / hair at the neckline
    L.paint_in(np.clip((Y - (-15.4 - 0.12 * np.abs(X))) * L.s + 0.5, 0, 1) * (np.abs(X) < 9.5), CARD["shadow"])
    # contour
    d = L.inside_dist(m)
    L.paint_in(np.clip((0.09 - d) * L.s + 0.5, 0, 1) * m, OUT)
    return L


def paint_collar():
    left, right = collar_shapes()
    L = P.Layer("Collar", (-5.0, -19.2, 5.2, -12.8), tpu=TPU * 2, ss=3)
    X, Y = L.XY()
    ml = L.poly(catmull(left, n=6, closed=True))
    mr = L.poly(catmull(right, n=6, closed=True))
    m = np.maximum(ml, mr)
    L.paint(m, SHIRT["base"])
    # the collar's fold (stand -> fall) as a shadow band near the neck
    L.paint_in(np.clip((Y - (-14.35 - 0.12 * np.abs(X))) * L.s + 0.5, 0, 1) * m, SHIRT["shadow"])
    L.paint_in(mr * np.clip((X - 2.6) * L.s * 0.5 + 0.5, 0, 1) * (Y < -15.2), SHIRT["shadow"])
    for mm in (ml, mr):
        L.paint_in(L.band_inside(mm, 0.065), SHIRT["line"])
    return L


def knot_shape():
    return np.array([(-0.35, -16.25), (0.95, -16.2), (1.1, -16.85), (0.75, -17.6), (-0.15, -17.65), (-0.5, -16.95)])


def tail_ctrl(side):
    if side < 0:
        return np.array([(0.05, -17.35), (-0.25, -19.6), (-0.45, -22.4), (-0.35, -25.4)])
    return np.array([(0.55, -17.35), (1.05, -19.3), (1.5, -21.6), (1.75, -24.1)])


def tail_polygon(side):
    sp = resample(catmull(tail_ctrl(side), n=12), n=40)
    t = np.linspace(0, 1, 40)
    w = 0.34 + 0.12 * t
    poly = P.ribbon(sp, w, w)
    # a V-cut end
    end = sp[-1]
    d = sp[-1] - sp[-2]
    d /= np.linalg.norm(d)
    n = np.array([-d[1], d[0]])
    left = end + n * w[-1]
    right = end - n * w[-1]
    notch = end - d * 0.35
    k = len(sp)
    poly = np.vstack([poly[:k], [left, notch, right], poly[k:]])
    return poly, sp


def paint_knot():
    L = P.Layer("RibbonKnot", (-1.3, -18.4, 1.9, -15.6), tpu=TPU * 3, ss=3)
    X, Y = L.XY()
    m = L.poly(catmull(knot_shape(), n=6, closed=True))
    L.paint(m, RIB["base"])
    L.paint_in(np.clip((X - 0.55) * L.s + 0.5, 0, 1) * np.clip((-16.7 - Y) * L.s + 0.5, 0, 1), RIB["shadow"])
    fold = resample(np.array([(0.1, -16.35), (0.35, -17.0), (0.25, -17.55)]), n=12)
    L.paint_in(L.stroke(fold, P.taper(12, 0.04, head=0.3, tail=0.5)), RIB["line"], 0.9)
    L.paint_in(L.band_inside(m, 0.05), RIB["line"])
    return L


def paint_tail(side):
    poly, sp = tail_polygon(side)
    lo, hi = poly.min(axis=0) - 0.4, poly.max(axis=0) + 0.4
    L = P.Layer("RibbonTail" + ("L" if side < 0 else "R"), (lo[0], lo[1], hi[0], hi[1]), tpu=TPU * 3, ss=3)
    X, Y = L.XY()
    m = L.poly(poly)
    L.paint(m, RIB["base"])
    # a twist: the tail turns showing its shadow side near the knot
    L.paint_in(np.clip((Y - (-18.8)) * L.s + 0.5, 0, 1) * m, RIB["shadow"])
    # the thin stripe (a woven edge) along one side
    L.paint_in(L.stroke(sp + np.array([0.18 * side, 0]), np.full(len(sp), 0.03)) * np.clip((-19.2 - Y) * L.s + 0.5, 0, 1),
               RIB["hi"], 0.9)
    L.paint_in(L.band_inside(m, 0.05), RIB["line"])
    return L


def build(log=print, cast=None):
    parts, texes = [], {}
    for painter, name, h in ((paint_cardigan_back, "CardiganBack", 0.4),
                             (paint_collar_back, "CollarBack", 0.4), (paint_shirt, "Shirt", 1.0),
                             (lambda: paint_cardigan(cast), "Cardigan", 1.0), (paint_collar, "Collar", 0.35),
                             (paint_knot, "RibbonKnot", 0.25)):
        t = painted(name, painter)
        texes[t.name] = t
        pts, tris = hull_mesh(t.rgba, t.bbox, h, h * 1.3, pad_texels=3)
        parts.append(Part(name, "Body", Z[name], name, pts, tris))
    for side in (-1, 1):
        t = painted("RibbonTail" + ("L" if side < 0 else "R"), lambda side=side: paint_tail(side))
        texes[t.name] = t
        pts, tris = hull_mesh(t.rgba, t.bbox, 0.22, 0.3, pad_texels=3)
        nm = t.name
        parts.append(Part(nm, "Ribbon", Z[nm], nm, pts, tris, extra=dict(ctrl=tail_ctrl(side))))
    return parts, texes
