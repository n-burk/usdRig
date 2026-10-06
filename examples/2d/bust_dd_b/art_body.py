"""Tailored teal jacket, ivory blouse and copper ribbon, designed in head units.

The torso has a shaped ribcage, a fitted waist and a cropped jacket. Sleeve attachments,
waist shaping and seam placement are defined with the arm landmarks, rather
than extending the old bust silhouette. Paint shares one palette with sleeves.
"""

import numpy as np
import paint as P
from parts import Part, painted, hull_mesh

TPU = 50
CARD = dict(
    base="#487B7F",
    shadow="#325C68",
    deep="#24434F",
    line="#233D46",
    rib="#30515B",
    hi="#85ACAA",
)
SHIRT = dict(
    base="#F3EEE3", shadow="#D1D7D3", deep="#ADBEBB", line="#667A79", button="#D3AA72"
)
RIB = dict(base="#B87553", shadow="#8C4F3F", hi="#DBAA7A", line="#59382F")
OUT = "#182F36"
BODY_BOX = (-22.0, -43.0, 22.0, -12.0)
Z = dict(
    CardiganBack=-1.95,
    CollarBack=-1.8,
    Shirt=-1.4,
    Cardigan=-0.60,
    Collar=-0.20,
    RibbonKnot=-0.85,
    RibbonTailL=-0.90,
    RibbonTailR=-0.88,
)


def mirror(p):
    return np.asarray(p) * (-1, 1)


def garment_outline():
    right = [
        (3.8, -14.6),
        (7.5, -16.0),
        (12.0, -17.8),
        (16.5, -20.0),
        (17.4, -24.0),
        (15.7, -28.0),
        (14.8, -33.0),
        (13.3, -39.0),
        (13.5, -41.4),
        (10.5, -42.0),
        (6.7, -40.8),
    ]
    return P.catmull([(-x, y) for x, y in right[::-1]] + right, n=16, closed=True)


def paint_jacket():
    L = P.Layer("Cardigan", BODY_BOX, tpu=TPU, ss=2)
    X, Y = L.XY()
    m = L.poly(garment_outline())
    opening = P.catmull(
        [
            (-3.9, -14.7),
            (-6.6, -22),
            (-7.2, -31),
            (-6.7, -43),
            (6.7, -43),
            (7.2, -31),
            (6.6, -22),
            (3.9, -14.7),
        ],
        n=14,
        closed=True,
    )
    m *= 1 - L.poly(opening)
    L.paint(m, CARD["base"])
    L.paint_in(Y < -39.2, CARD["rib"], 0.8)
    for side in (-1, 1):
        side_shadow = P.catmull(
            np.array(
                [
                    (16.8, -24),
                    (16, -30),
                    (13.9, -35),
                    (13.5, -39.5),
                    (12, -39.5),
                    (12.8, -33.5),
                    (14, -30),
                    (14.7, -27),
                ]
            )
            * (side, 1),
            n=12,
            closed=True,
        )
        L.paint_in(L.poly(side_shadow), CARD["shadow"], 0.7)
        lapel = np.array(
            [
                (4.2, -14.9),
                (8.3, -17.5),
                (7.4, -21.8),
                (9.0, -24.0),
                (6.8, -31),
                (6.3, -22),
            ]
        ) * (side, 1)
        L.paint_in(L.poly(lapel), CARD["hi"], 0.68)
        edge = P.resample(
            P.catmull(
                np.array([(4.2, -15), (6.5, -22), (7.1, -31), (6.6, -40.8)]) * (side, 1)
            ),
            n=65,
        )
        L.paint_in(L.stroke(edge, np.full(65, 0.06)), CARD["line"], 0.8)
        seam = P.resample(
            P.catmull(
                np.array([(16.3, -23), (14.2, -29), (11.6, -34), (11.8, -39)])
                * (side, 1)
            ),
            n=60,
        )
        L.paint_in(
            L.stroke(seam, P.taper(60, 0.065, head=0.2, tail=0.25)), CARD["line"], 0.65
        )
        fold = P.resample(
            P.catmull(np.array([(15.3, -30), (12.7, -32.7), (9.3, -33.8)]) * (side, 1)),
            n=45,
        )
        L.paint_in(
            L.stroke(fold, P.taper(45, 0.05, head=0.3, tail=0.1)), CARD["line"], 0.55
        )
        pocket = P.resample(np.array([(9.0, -36.8), (12.3, -35.6)]) * (side, 1), n=30)
        L.paint_in(L.stroke(pocket, np.full(30, 0.13)), CARD["shadow"])
        L.paint_in(
            L.stroke(pocket + np.array([0, 0.15]), np.full(30, 0.04)), CARD["hi"], 0.8
        )
    for y in (-27.0, -36.0):
        L.paint_in(L.disc(-7.6, y, 0.30), SHIRT["button"])
        L.paint_in(L.disc(-7.65, y + 0.06, 0.09), RIB["hi"])
    L.paint_in(L.band_inside(m, 0.075), OUT, 0.9)
    return L


def paint_shirt():
    L = P.Layer("Shirt", (-19, -56, 19, -13), tpu=TPU, ss=2)
    X, Y = L.XY()
    right = [
        (4.8, -13.8),
        (10.0, -18.0),
        (15.6, -22.5),
        (16.7, -27.5),
        (15.0, -33.0),
        (12.7, -39.0),
        (11.5, -45.0),
        (12.2, -49.0),
        (14.2, -55.5),
    ]
    outline = P.catmull(
        [(-x, y) for x, y in right[::-1]]
        + [(-1.2, -15.5), (0, -17), (1.2, -15.5)]
        + right,
        n=16,
        closed=True,
    )
    m = L.poly(outline)
    L.paint(m, SHIRT["base"])
    L.paint_in((np.abs(X) > 12.5 + 0.07 * (Y + 34)) * (Y < -23), SHIRT["shadow"], 0.65)
    L.paint_in(Y > -18 - 0.12 * np.abs(X), SHIRT["shadow"], 0.7)
    # Broad, quiet cloth shadows describe the ribcage and chest; the shirt
    # stays fully covered and the jacket's hem reveals the fitted waist.
    for side in (-1, 1):
        shadow = P.catmull(
            np.array(
                [
                    (14.5, -25),
                    (14.8, -29),
                    (11.0, -32.7),
                    (6.5, -33.5),
                    (3.7, -32.7),
                    (5.2, -34.6),
                    (10.2, -34.8),
                    (14.2, -32.1),
                    (16.0, -27),
                ]
            )
            * (side, 1),
            n=12,
            closed=True,
        )
        L.paint_in(L.poly(shadow), SHIRT["shadow"], 0.6)
        for pts in (
            [(11.5, -39), (8.7, -42.5), (8.9, -47)],
            [(10.5, -48.7), (7.0, -47.6), (4.8, -48.7)],
        ):
            sp = P.resample(P.catmull(np.array(pts) * (side, 1)), n=45)
            L.paint_in(
                L.stroke(sp, P.taper(45, 0.07, head=0.1, tail=0.1)), SHIRT["deep"], 0.5
            )
    placket = P.resample(
        P.catmull([(0, -20), (0.25, -28), (0.08, -37), (-0.1, -48.5)]), n=70
    )
    L.paint_in(L.stroke(placket, np.full(70, 0.035)), SHIRT["line"], 0.5)
    for y in (-25, -33, -41):
        L.paint_in(L.disc(0.6, y, 0.16), SHIRT["button"])
    waistband = Y < -50 + 0.07 * np.abs(X)
    L.paint_in(waistband, CARD["deep"])
    L.paint_in(np.abs(Y - (-49.5 + 0.07 * np.abs(X))) < 0.065, CARD["hi"], 0.7)
    L.paint_in(L.band_inside(m, 0.06), OUT, 0.65)
    return L


def paint_neck_piece(name, back=False):
    L = P.Layer(name, (-8, -20, 8, -12), tpu=90, ss=2)
    if back:
        points = [
            (-5, -13.5),
            (-3, -12.5),
            (3, -12.5),
            (5, -13.5),
            (5.4, -16),
            (-5.4, -16),
        ]
        m = L.poly(P.catmull(points, n=10, closed=True))
        L.paint(m, SHIRT["shadow"] if name == "CollarBack" else CARD["shadow"])
    else:
        m = np.zeros((L.H, L.W))
        for side in (-1, 1):
            p = np.array(
                [(0.2, -15.7), (3.7, -13.5), (5.0, -14.5), (3.9, -19.3), (2.0, -17.7)]
            ) * (side, 1)
            m = np.maximum(m, L.poly(p))
        L.paint(m, SHIRT["base"])
        X, Y = L.XY()
        L.paint_in(Y > -15.1, SHIRT["shadow"], 0.55)
    L.paint_in(L.band_inside(m, 0.06), SHIRT["line"], 0.75)
    return L


def tail_ctrl(side):
    return np.array(
        [
            (side * 0.2, -17.8),
            (side * 0.7, -20),
            (side * 1.2, -22.5),
            (side * 1.5, -24.5),
        ]
    )


def paint_ribbon(name, side=0):
    if not side:
        points = P.catmull(
            [(-0.9, -17.1), (0.8, -17.1), (1.0, -18.2), (-0.8, -18.6)],
            n=10,
            closed=True,
        )
    else:
        sp = P.resample(P.catmull(tail_ctrl(side)), n=50)
        points = P.ribbon(sp, np.linspace(0.5, 0.7, 50), np.linspace(0.5, 0.7, 50))
    lo, hi = points.min(0) - 0.4, points.max(0) + 0.4
    L = P.Layer(name, (*lo, *hi), tpu=120, ss=2)
    m = L.poly(points)
    L.paint(m, RIB["base"])
    X, Y = L.XY()
    L.paint_in(X > side * 0.35, RIB["shadow"], 0.6)
    L.paint_in(L.band_inside(m, 0.05), RIB["line"], 0.7)
    return L


def build(log=print, cast=None):
    parts, texes = [], {}
    layers = [
        ("CardiganBack", lambda: paint_neck_piece("CardiganBack", True), 0.4),
        ("CollarBack", lambda: paint_neck_piece("CollarBack", True), 0.4),
        ("Shirt", paint_shirt, 0.7),
        ("Cardigan", paint_jacket, 0.65),
        ("Collar", lambda: paint_neck_piece("Collar"), 0.3),
        ("RibbonKnot", lambda: paint_ribbon("RibbonKnot"), 0.2),
    ]
    for side, tag in ((-1, "L"), (1, "R")):
        layers.append(
            (
                "RibbonTail" + tag,
                lambda side=side, tag=tag: paint_ribbon("RibbonTail" + tag, side),
                0.2,
            )
        )
    for name, painter, spacing in layers:
        t = painted(name, painter)
        texes[name] = t
        pts, tris = hull_mesh(t.rgba, t.bbox, spacing, spacing * 1.3, pad_texels=3)
        group = "Ribbon" if name.startswith("RibbonTail") else "Body"
        parts.append(Part(name, group, Z[name], name, pts, tris))
    return parts, texes
