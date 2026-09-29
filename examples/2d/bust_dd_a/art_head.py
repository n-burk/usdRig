"""Face skin, the contour line, ears and neck of Kaede."""

import math

import numpy as np

import design as D
import paint as P
from art import Piece, Strip, grid_from_alpha, sample_at, state
from paint import catmull, resample, smooth

FACE_TPU = 56


def face_mask_poly(grow=0.05):
    o = D.face_outline(40)
    c = np.array([0.0, -0.5])
    d = o - c
    return o + d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-9) * grow


def paint_face(hair_shadow_polys=(), lock_shadow_polys=()):
    poly = face_mask_poly(0.07)
    L = P.Layer("Face", (-8.4, -10.7, 8.4, 10.6), tpu=FACE_TPU, ss=2)
    X, Y = L.XY()
    m = L.poly(poly)
    L.paint(m, D.SKIN["base"])
    # cast shadow of the fringe on the forehead (zigzag of the tips, offset
    # down and to the right), and of the side locks on the cheeks
    for pl in hair_shadow_polys:
        L.paint_in(L.poly(pl) * m, D.SKIN["shadow"], 1.0)
    for pl in lock_shadow_polys:
        L.paint_in(L.poly(pl) * m, D.SKIN["shadow"], 1.0)
    # deep occlusion along the hairline (only shows between fringe clumps)
    hl = np.clip((Y - (6.3 - 0.35 * (X / 8.0) ** 2)) * L.s * 0.5, 0, 1)
    L.paint_in(hl * m, D.SKIN["deep"], 1.0)
    # form shadow along the shadow-side jaw, under the ear
    jaw_r = np.array([(7.35, -1.6), (7.05, -3.4), (6.3, -5.3), (5.2, -6.8)])
    jw = resample(catmull(jaw_r, n=12), n=60)
    ts = np.linspace(0, 1, 60)
    _, N = sample_at(jw, ts)
    w = 0.42 * np.sin(ts * math.pi) ** 0.7
    shp = np.vstack([jw + N * 0.3, (jw - N * w[:, None])[::-1]])
    L.paint_in(L.poly(shp) * m, D.SKIN["shadow"], 1.0)
    # beauty mark under the outer corner of the screen-left eye
    L.paint_in(L.disc(-5.45, -2.55, 0.085), D.SKIN["mole"], 1.0)
    return L


def face_pieces(hair_shadow_polys=(), lock_shadow_polys=()):
    out = []
    Lf = paint_face(hair_shadow_polys, lock_shadow_polys)
    rgba = Lf.finish()
    v, t = grid_from_alpha(rgba, Lf.bbox, 0.42)
    face = Piece("Face", "Face", 0.0, Lf, v, t, lambda s, v=v: v + jaw_field(v, s.get("jaw", 0.0)))
    face.rgba = rgba
    out.append(face)

    # the contour line: a strip along the visible jaw line
    jl = D.jaw_line(121)
    L = P.Layer("FaceLine", (-8.3, -10.6, 8.3, 4.0), tpu=90, ss=2)
    ts = np.linspace(0, 1, 400)
    P0, N = sample_at(jl, ts)
    y = P0[:, 1]
    # thin at the top ends (under the hair), fuller along the jaw, lighter
    # on the chin flat, a light break on the lit (screen-left) jaw
    w = 0.060 * smooth(3.2, 0.5, y) + 0.012
    w *= 1.0 - 0.30 * np.exp(-((P0[:, 0]) / 0.9) ** 2)
    w *= 1.0 - 0.85 * np.exp(-((P0[:, 0] + 3.9) / 0.55) ** 2) * (P0[:, 1] < -6.0)
    w = np.maximum(w, 0.004)
    poly = np.vstack([P0 + N * (w * 0.9)[:, None], (P0 - N * (w * 1.1)[:, None])[::-1]])
    L.paint(L.poly(poly), D.LINE)
    st = Strip(lambda s: face_line_spine(s), np.linspace(0, 1, 90), [-0.16, 0.0, 0.16], up=1.0)
    out.append(Piece("FaceLine", "FaceLine", 0.02, L, st.pos(state()), st.tris(), st.pos))
    out[-1].meta["spine_ts"] = st.ts
    return out


def side_plane_pieces():
    """The far cheek's side plane: a hard shadow strip along the contour from
    the temple to the jaw whose inner edge follows the temple notch and the
    cheekbone bump. It rests collapsed onto the contour; the turn keyforms
    grow it on the far side (0.55 at 15 degrees, full at 30)."""
    out = []
    o = D.face_outline(60)
    for side in (+1, -1):
        tag = "R" if side > 0 else "L"
        half = o[(o[:, 0] * side > 0.3) & (o[:, 1] < 2.8) & (o[:, 1] > -8.9)]
        half = half[np.argsort(-half[:, 1])]                     # top -> bottom
        spine = resample(half, n=80)
        ts = np.linspace(0, 1, 80)
        P0, N = sample_at(spine, ts)
        inward = -N * side if np.mean((-N * side)[:, 0] * -side) > 0 else N * side
        # make sure "inward" points toward the face centre
        if np.mean(inward[:, 0] * side) > 0:
            inward = -inward
        y = P0[:, 1]
        w = (0.95 - 0.28 * np.exp(-((y - 0.6) / 0.8) ** 2) + 0.30 * np.exp(-((y + 2.0) / 1.0) ** 2)) * \
            smooth(-8.8, -6.0, y) * smooth(2.8, 1.6, y) + 0.02
        inner = P0 + inward * w[:, None]
        outer = P0 - inward * 0.12
        L = P.Layer("SidePlane" + tag, (min(inner[:, 0].min(), outer[:, 0].min()) - 0.3, -9.4,
                                        max(inner[:, 0].max(), outer[:, 0].max()) + 0.3, 3.3), tpu=FACE_TPU, ss=2)
        L.paint(L.poly(np.vstack([outer, inner[::-1]])), D.SKIN["shadow"])
        rgba = L.finish()
        # a strip mesh: 3 rows across (contour, middle, inner edge + margin)
        vs = np.array([-0.12, 0.0, 0.5, 1.0, 1.15])
        ref = np.concatenate([P0 + inward * (w * f)[:, None] if f >= 0 else P0 + inward * f for f in vs])
        rest = np.concatenate([P0 + inward * (0.002 * f) for f in vs])
        from art import grid_tris
        k = len(vs)
        # vertex order is row-major by v then t; build tris for (t along, v across)
        idx = np.arange(len(ref)).reshape(k, len(ts)).T.reshape(-1)
        tris = grid_tris(len(ts), k)
        tris = idx[tris]

        def pos(s, side=side, ref=ref, rest=rest):
            a = float(s.get("plane", {}).get(side, 0.0))
            return rest + (ref - rest) * a
        pc = Piece("SidePlane_" + tag, "FaceShade", 0.01, L, ref, tris, pos, rest=rest)
        pc.rgba = rgba
        out.append(pc)
    return out


def face_line_spine(s):
    """The jaw line at an expression state: the jaw drop moves the chin."""
    jl = D.jaw_line(121)
    return jl + jaw_field(jl, s.get("jaw", 0.0))


def jaw_field(xy, drop, widen=0.0):
    """Displacement of face points for a jaw drop of `drop` model units at
    the chin: the mouth-to-chin region swings down (a rotation about the
    ears, so the jaw corners move least)."""
    xy = np.asarray(xy, np.float64)
    if not drop and not widen:
        return np.zeros_like(xy)
    x, y = xy[:, 0], xy[:, 1]
    below = smooth(-5.9, -8.2, y)                   # 0 above the mouth, 1 at the chin
    lat = np.clip(1.0 - (np.abs(x) / 7.4) ** 2.2, 0, 1)
    d = np.zeros_like(xy)
    d[:, 1] = -drop * below * lat
    # the lower face narrows a little as it lengthens
    d[:, 0] = -x * 0.025 * drop * below + widen * x * below * lat
    return d


# ears

EAR_R = [(7.25, 0.95), (7.95, 1.05), (8.55, 0.35), (8.68, -0.85), (8.45, -2.2), (8.12, -3.45),
         (7.75, -4.35), (7.25, -4.55)]


def ear_outline(side):
    pts = np.array(EAR_R)
    pts = np.vstack([pts, [(6.6, -3.0), (6.6, 0.4)]])
    pts[:, 0] *= side
    return catmull(pts, n=14, closed=True)


def ear_pieces():
    out = []
    for side in (+1, -1):
        tag = "R" if side > 0 else "L"
        o = ear_outline(side)
        L = P.Layer("Ear" + tag, (min(6.3 * side, 9.2 * side), -5.2, max(6.3 * side, 9.2 * side), 1.8), tpu=90, ss=3)
        X, Y = L.XY()
        m = L.poly(o)
        L.paint(m, D.SKIN["base"])
        # inner ear: a shadow bowl and the coloured C line
        inner = np.array([(7.55, 0.35), (8.15, -0.2), (8.2, -1.6), (7.85, -2.9), (7.45, -3.35)])
        inner[:, 0] *= side
        bowl = np.vstack([catmull(inner, n=10), [(7.1 * side, -2.6), (7.1 * side, -0.2)]])
        L.paint_in(L.poly(bowl) * m, D.SKIN["shadow"], 1.0)
        deep = np.array([(7.55, -0.5), (7.95, -1.3), (7.7, -2.4), (7.3, -2.2), (7.3, -0.9)])
        deep[:, 0] *= side
        L.paint_in(L.poly(catmull(deep, n=8, closed=True)) * m, D.SKIN["deep"], 1.0)
        sp = resample(catmull(inner, n=10), n=50)
        w = P.taper(50, 0.035, head=0.3, tail=0.4)
        L.paint_in(L.stroke(sp, w, w) * m, D.SKIN["line"], 1.0)
        # outline (outer rim only)
        rim = resample(catmull(np.array(EAR_R[:-1]) * np.array([side, 1]), n=12), n=80)
        w = P.taper(80, 0.055, head=0.1, tail=0.3)
        L.paint(L.stroke(rim, w, w) * m, D.LINE)
        if side > 0:
            # silver helix cuff on the upper rim and two studs
            cuff = resample(catmull(np.array([(8.25, 0.72), (8.62, 0.2), (8.72, -0.45)]), n=10), n=30)
            cw = np.full(30, 0.11)
            cm = L.stroke(cuff, cw, cw)
            L.paint(cm, D.SILVER["base"])
            L.paint_in(L.stroke(cuff + np.array([0.05, -0.05]), cw * 0.45, cw * 0.45) * cm, D.SILVER["shadow"], 1.0)
            L.paint(L.band_inside(cm, 0.03) * cm, D.SILVER["line"])
            L.paint(L.disc(8.47, 0.53, 0.035), D.SILVER["glint"])
            for (sx, sy, r) in ((8.28, -1.2, 0.12), (7.78, -3.95, 0.14)):
                dm = L.disc(sx, sy, r)
                L.paint(dm, D.SILVER["base"])
                L.paint(L.band_inside(dm, 0.03), D.SILVER["line"])
                L.paint(L.disc(sx - 0.04, sy + 0.04, 0.035), D.SILVER["glint"])
        rgba = L.finish()
        v, t = grid_from_alpha(rgba, L.bbox, 0.25)
        out.append(Piece("Ear_" + tag, "Ears", -0.30, L, v, t))
    return out


# neck

NECK_R = [(3.55, -4.5), (3.6, -7.0), (3.72, -9.5), (3.95, -11.6), (4.6, -12.9), (6.2, -13.9),
          (8.5, -14.6), (8.5, -18.5)]


def neck_outline():
    r = np.array(NECK_R)
    l = r[::-1].copy()
    l[:, 0] *= -1
    return np.vstack([r, l])


def neck_pieces():
    o = neck_outline()
    L = P.Layer("Neck", (-9.0, -19.0, 9.0, -4.0), tpu=FACE_TPU, ss=2)
    X, Y = L.XY()
    m = L.poly(o)
    L.paint(m, D.SKIN["base"])
    # the jaw's shadow on the neck: everything the chin hides at rest is in
    # shadow (it shows on a nod up); the lower edge is a diagonal cut, lower
    # on the shadow side, plus the shadow-side edge of the neck
    jaw_y = np.interp(np.abs(X), [0, 1.5, 3.0, 4.5], [-10.0, -9.6, -8.4, -6.5])
    low = -10.55 - 0.55 * (X / 3.8) - 0.9 * smooth(1.0, 3.8, X)
    sh = (Y > low - 0.12 * np.sin(X * 2.2))
    L.paint_in(sh.astype(np.float32) * m, D.SKIN["shadow"], 1.0)
    side_sh = smooth(2.2, 2.55, X) * (Y > -13.3)
    L.paint_in(side_sh * m, D.SKIN["shadow"], 1.0)
    deep = (Y < jaw_y + 0.05) & (Y > jaw_y - 0.35 - 0.25 * smooth(0, 3, X)) & (np.abs(X) < 3.2)
    L.paint_in(deep.astype(np.float32) * m, D.SKIN["deep"], 1.0)
    # sternocleidomastoid strokes and the pit of the neck
    for sgn in (+1, -1):
        sp = resample(np.array([(2.55 * sgn, -10.9), (1.9 * sgn, -12.0), (1.0 * sgn, -13.1)]), n=24)
        w = P.taper(24, 0.032, head=0.5, tail=0.5)
        L.paint_in(L.stroke(sp, w, w) * m, D.SKIN["line"], 0.75)
    pit = resample(np.array([(-0.32, -13.55), (0.0, -13.9), (0.30, -13.58)]), n=20)
    w = P.taper(20, 0.03, head=0.3, tail=0.3)
    L.paint_in(L.stroke(pit, w, w) * m, D.SKIN["line"], 1.0)
    # contour lines along the neck sides only
    for sgn in (+1, -1):
        side = np.array([(3.62 * sgn, -8.2), (3.72 * sgn, -9.5), (3.95 * sgn, -11.6), (4.6 * sgn, -12.9),
                         (6.2 * sgn, -13.9)])
        sp = resample(catmull(side, n=10), n=60)
        _, N = sample_at(sp, np.linspace(0, 1, 60))
        w = P.taper(60, 0.055, head=0.35, tail=0.3)
        inward = -N * sgn
        poly = np.vstack([sp + inward * 0.0, (sp + inward * (2 * w)[:, None])[::-1]])
        L.paint_in(L.poly(poly) * m, D.LINE, 1.0)
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.5)
    return [Piece("Neck", "Neck", -1.30, L, v, t)]
