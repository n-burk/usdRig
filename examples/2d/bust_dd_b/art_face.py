"""Shion's face: skin, nose, eyes, brows, mouth, ear, blush -- painted as
textures at the rest (or a paint) pose, each on a mesh whose points are a
function of the expression state (see state.py).

Line and cel rules (bust style guide sections 3-7): warm near-black lines,
coloured interior lines on skin details, hard cel edges everywhere except
the blush patch, a heavy black upper lash mass with a winged flick, 3-tone
irises with one low catch light on the light (screen-left) side, thick low
brows, a two-mark nose, a wide mouth with a separate lower-lip stroke.
"""

import math

import numpy as np

import design as D
import paint as P
import state as ST
from paint import catmull, hexc, resample, smooth
from parts import Part, finish, painted, band_points, grid_tris, ribbon_points, fill_polygon

TPU_F = 60          # face / ear / neck texels per model unit
TPU_E = 128         # eyes, brows, mouth

SKIN = dict(base="#FCE6D8", shadow="#EDB7A4", deep="#D48A7C", line="#9A4E4A", blush="#F6A5A2",
            hatch="#D9535F", lip="#E3898A", lip_line="#B75A5C", gloss="#FFF3EE", mark="#5B3438",
            contour="#3A2228")
EYE = dict(lash="#17121A", lash_warm="#3A2230", top="#1C2433", mid="#4A6488", low="#9DB8D8",
           pupil="#0D1119", ring="#111722", white="#FFFFFF", lid_shadow="#D8D2EA", hl="#FFFFFF",
           lower="#2A1D24", crease="#9A4E4A", duct="#F2B4AC")
BROW = dict(fill="#2B1B27", edge="#140C13")
MOUTH = dict(inside="#5A1A26", deep="#3A0E18", tongue="#E5707A", tongue_sh="#BF4F5D",
             teeth="#FFFDF8", teeth_sh="#D8D5E3", line="#2E1A20")
OUTLINE = "#231A1F"

# z (draw order); the eye stack sits between the face and the brows
Z = dict(Neck=-1.60, Ear=-0.12, Earring=-0.10, Face=0.0, BlushBase=0.02, SidePlane=0.03,
         MouthInside=0.05, Tongue=0.06, Teeth=0.07,
         Sclera=0.06, LidShadow=0.065, Iris=0.07, Highlight=0.075,
         EyeMask=0.09, LipMask=0.09,
         FaceLine=0.10, MouthLine=0.11, LowerEdge=0.108, LowerLip=0.105,
         LowerLid=0.12, Crease=0.12, Lash=0.13, BlushPop=0.14, Hatch=0.15, Sweat=0.30,
         Brow=0.50)


def side_tag(side):
    return "R" if side > 0 else "L"


# the expression -> curves (the same linear model the rig evaluates)

_EYE_CACHE = {}


def _shape(name, i):
    k = (name, i)
    if k not in _EYE_CACHE:
        _EYE_CACHE[k] = D.lid_uw(name, i)
    return _EYE_CACHE[k]


def eye_curves_uw(side, st):
    c, w, s, f = ST.eye_params(st, side)
    out = []
    for i in (0, 1):
        o = _shape("open", i)
        H, C = _shape("half", i), _shape("closed", i)
        if c <= 0.5:
            ct = 2.0 * c * (H - o)
        else:
            ct = (H - o) + min(2.0 * c - 1.0, 1.0) * (C - H)
        v = (o + ct + w * (_shape("wide", i) - o) + s * (_shape("smile", i) - o)
             + f * (_shape("flat", i) - o)
             + min(c, 1.0) * s * (_shape("smile_closed", i) - C - _shape("smile", i) + o))
        out.append(v)
    return out


def eye_curves(side, st):
    up, lo = eye_curves_uw(side, st)
    return D.eye_to_model(side, up), D.eye_to_model(side, lo)


def up_normals(curve, side):
    return D.normals_up(curve, side)


def lash_spine(side, st):
    """The upper lid extended: 4 samples back past the inner corner (for the
    tear-duct hook) and 7 along the winged flick past the outer corner."""
    up, _ = eye_curves(side, st)
    t0 = up[0] - up[2]
    t0 /= np.linalg.norm(t0)
    pre = [up[0] + t0 * 0.07 * k for k in (3, 2, 1)]
    wd = D.wing_dir(side, up)
    post = [up[-1] + wd * 0.11 * k for k in range(1, 8)]
    return np.vstack([pre, up, post])


LASH_PRE, LASH_POST = 3, 7
LASH_V = [-0.30, -0.14, -0.03, 0.10, 0.26, 0.44, 0.66, 0.92]


def lash_points(side, st):
    sp = lash_spine(side, st)
    n = up_normals(sp, side)
    return ribbon_points(sp, LASH_V, n)


# the face texture

FACE_BOX = (-9.0, -11.2, 9.0, 10.4)


def paint_face(fringe_shadow=None, lock_shadows=()):
    L = P.Layer("Face", FACE_BOX, tpu=TPU_F, ss=3)
    X, Y = L.XY()
    out = D.face_outline()
    m = L.poly(out)
    L.paint(m, SKIN["base"])
    # --- hard cel: shadow side (screen right) along the jaw and under the
    # cheekbone, widening toward the chin -----------------------------------
    d_in = L.inside_dist(m)
    w = 0.10 + 0.32 * smooth(-2.8, -7.5, Y) - 0.18 * smooth(-8.6, -10.0, Y)
    cel = np.clip((w - d_in) * L.s + 0.5, 0, 1) * smooth(1.2, 3.2, X) * smooth(-1.8, -3.4, Y)
    L.paint_in(cel, SKIN["shadow"])
    # --- the hairline: the forehead under the fringe (hidden, but the fringe
    # sways) in the shadow tone, the roots in the deep tone ----------------
    # (the hairline shadow is the crown's and the fringe's cast shadow)
    L.paint_in(np.clip((Y - 9.2) * L.s + 0.5, 0, 1), SKIN["deep"])
    if fringe_shadow is not None:
        L.paint_in(fringe_shadow(L), SKIN["shadow"])
    for sh in lock_shadows:
        L.paint_in(sh(L), SKIN["shadow"])
    # --- nose: a hooked tip stroke on the shadow side, a hard shadow
    # triangle under it, a tiny nostril tick on the lit side ----------------
    nm = D.nose_marks()
    tri = catmull(nm["shadow"], n=6, closed=True)
    L.paint_in(L.poly(tri), SKIN["shadow"])
    sp = resample(catmull(nm["tip"], n=12), n=40)
    L.paint_in(L.stroke(sp, P.taper(40, 0.052, head=0.55, tail=0.30, power=0.9)), SKIN["line"])
    sp = resample(catmull(nm["nostril"], n=8), n=16)
    L.paint_in(L.stroke(sp, P.taper(16, 0.024, head=0.5, tail=0.5)), SKIN["line"], 0.85)
    # --- under the lower lip: a small crescent (rides the lip part, see
    # paint_lower_lip) -- here only the chin's hard lower-lip shadow shelf
    # --- beauty mark under her left eye (screen right) -------------------
    L.paint_in(L.disc(5.05, -2.55, 0.085), SKIN["mark"])
    L.meta["outline"] = out
    return L


FACE_LINE_Y = 2.6          # the contour line runs below this height


def face_line_indices():
    out = D.face_outline()
    idx = np.nonzero(out[:, 1] < FACE_LINE_Y)[0]
    # the outline is closed, starting top centre and running down the left:
    # the lower part is one contiguous run
    return idx


def face_line_width(sp):
    """Contour weight along the lower outline (left temple -> chin -> right
    temple): heavier on the shadow side and under the chin, thin and broken
    on the lit cheek, tapered into the hair at both ends."""
    n = len(sp)
    t = np.linspace(0, 1, n)
    x, y = sp[:, 0], sp[:, 1]
    w = 0.085 + 0.045 * smooth(-3.0, -9.0, y) + 0.03 * smooth(0.0, 5.0, x) * smooth(0.0, -5.0, y)
    w = w * (1.0 - 0.55 * smooth(-4.5, -1.0, y) * smooth(-2.0, -6.0, x))       # lit cheek: thinner
    w = w * smooth(0.0, 0.06, t) * smooth(1.0, 0.94, t)
    return w


def paint_face_line():
    out = D.face_outline()
    idx = face_line_indices()
    sp = out[idx]
    L = P.Layer("FaceLine", FACE_BOX, tpu=TPU_F * 2, ss=2)
    w = face_line_width(sp)
    n = P.normals(sp)                   # left normal = inward (CCW outline)
    poly = np.vstack([sp - n * 0.004, (sp + n * w[:, None])[::-1]])
    L.paint(L.poly(poly), SKIN["contour"])
    return L


# eyes

def eye_box(side, left=0.95, right=1.05, down=0.62, up=0.9):
    cx, cy = D.EYE_C[side]
    if side > 0:
        return (cx - left * D.EW * 0.7, cy - down * D.EW * 0.7, cx + right * D.EW * 0.7, cy + up * D.EW * 0.7)
    return (cx - right * D.EW * 0.7, cy - down * D.EW * 0.7, cx + left * D.EW * 0.7, cy + up * D.EW * 0.7)


SCLERA_ROWS = [0.35, 0.0, -0.5, -1.0]     # upper margin, lid, fractions, lower margin


def sclera_points(side, st):
    up, lo = eye_curves(side, st)
    nu = up_normals(up, side)
    nl = up_normals(lo, side)
    top = up + nu * 0.30
    bot = lo - nl * 0.30
    return band_points(top, bot, 5)


def paint_sclera(side):
    L = P.Layer("Sclera_" + side_tag(side), eye_box(side), tpu=TPU_E, ss=2)
    pts = sclera_points(side, ST.REST)
    top, bot = pts[:D.LID_N], pts[-D.LID_N:]
    L.paint(L.poly(np.vstack([top, bot[::-1]])), EYE["white"])
    # the tear duct: a small warm notch at the inner corner
    up, lo = eye_curves(side, ST.REST)
    ic = up[0] + np.array([0.14 * side, -0.02])
    L.paint_in(np.clip(1 - np.hypot(L.XY()[0] - ic[0], (L.XY()[1] - ic[1]) * 1.4) / 0.16, 0, 1) ** 0.5,
               EYE["duct"], 0.9)
    return L


LIDSH_V = [0.06, -0.08, -0.26, -0.46, -0.62]


def lid_shadow_points(side, st):
    up, _ = eye_curves(side, st)
    return ribbon_points(up, LIDSH_V, up_normals(up, side))


def paint_lid_shadow(side):
    L = P.Layer("LidShadow_" + side_tag(side), eye_box(side), tpu=TPU_E, ss=2)
    up, _ = eye_curves(side, ST.REST)
    n = up_normals(up, side)
    t = np.linspace(0, 1, len(up))
    h = 0.14 + 0.34 * np.sin(np.clip(t, 0, 1) * math.pi) ** 0.8
    poly = np.vstack([up + n * 0.05, (up - n * h[:, None])[::-1]])
    L.paint(L.poly(poly), EYE["lid_shadow"])
    return L


IRIS_RINGS, IRIS_SEGS = 7, 40


def iris_mesh():
    cx, cy = D.iris_centre(+1)
    rx, ry = D.IRIS["ru"] * D.EW + 0.06, D.IRIS["rw"] * D.EW + 0.06
    pts = [(0.0, 0.0)]
    for i in range(1, IRIS_RINGS + 1):
        r = i / float(IRIS_RINGS)
        for j in range(IRIS_SEGS):
            a = 2 * math.pi * j / IRIS_SEGS
            pts.append((rx * r * math.cos(a), ry * r * math.sin(a)))
    tris = []
    for j in range(IRIS_SEGS):
        tris.append((0, 1 + j, 1 + (j + 1) % IRIS_SEGS))
    for i in range(1, IRIS_RINGS):
        b0, b1 = 1 + (i - 1) * IRIS_SEGS, 1 + i * IRIS_SEGS
        for j in range(IRIS_SEGS):
            j1 = (j + 1) % IRIS_SEGS
            tris.append((b0 + j, b1 + j, b1 + j1))
            tris.append((b0 + j, b1 + j1, b0 + j1))
    return np.array(pts), np.array(tris, np.int64)


def iris_points(side, st, local):
    lx, ly = ST.look(st)
    c = D.iris_centre(side, (lx, ly))
    k = 1.0 - 0.58 * st["iris.shrink"]
    # the iris foreshortens a little as it rolls toward a corner
    sq = 1.0 - 0.10 * abs(lx)
    return c + local * np.array([k * sq, k])


def paint_iris(side):
    cx, cy = D.iris_centre(side)
    rx, ry = D.IRIS["ru"] * D.EW, D.IRIS["rw"] * D.EW
    L = P.Layer("Iris_" + side_tag(side), (cx - rx - 0.2, cy - ry - 0.2, cx + rx + 0.2, cy + ry + 0.2), tpu=TPU_E, ss=3)
    X, Y = L.XY()
    ex, ey = (X - cx) / rx, (Y - cy) / ry
    r = np.hypot(ex, ey)
    m = np.clip((1 - r) * min(rx, ry) * L.s + 0.5, 0, 1)
    L.paint(m, EYE["mid"])
    # lower crescent light: inside the iris, below an ellipse shifted up
    r2 = np.hypot(ex / 0.97, (ey - 0.30) / 0.88)
    cres = np.clip((r2 - 1.0) * min(rx, ry) * L.s + 0.5, 0, 1) * (ey < 0.2)
    L.paint_in(cres, EYE["low"])
    # the lid-shadow top band: ~38 %, hard, slightly curved edge
    edge = 0.24 - 0.10 * ex ** 2
    L.paint_in(np.clip((ey - edge) * ry * L.s + 0.5, 0, 1), EYE["top"])
    # pupil: a vertical oval inside the top band, merging into it
    pr = np.hypot(ex / 0.40, (ey - 0.16) / 0.47)
    L.paint_in(np.clip((1 - pr) * 0.35 * L.s + 0.5, 0, 1), EYE["pupil"])
    # iris ring on the lower half only (lost in the lid shadow at the top)
    ring_w = 0.085 * (1.0 - smooth(-0.05, 0.45, ey))
    L.paint_in(np.clip((r - (1 - ring_w / min(rx, ry))) * min(rx, ry) * L.s + 0.5, 0, 1) * (ring_w > 0.004), EYE["ring"])
    return L


HL_BOX = 1.2


def highlight_points(side, st, local):
    lx, ly = ST.look(st)
    c = D.iris_centre(side, (0.42 * lx, 0.42 * ly))
    k = 1.0 - 0.999 * st["iris.shrink"]
    return c + local * k


def highlight_mesh():
    xs = np.linspace(-HL_BOX, HL_BOX, 7)
    ys = np.linspace(-HL_BOX, HL_BOX, 7)
    pts = np.array([(x, y) for y in ys for x in xs])
    return pts, grid_tris(7, 7)


def paint_highlight(side):
    cx, cy = D.iris_centre(side)
    L = P.Layer("Highlight_" + side_tag(side), (cx - HL_BOX, cy - HL_BOX, cx + HL_BOX, cy + HL_BOX), tpu=TPU_E, ss=3)
    X, Y = L.XY()
    rx, ry = D.IRIS["ru"] * D.EW, D.IRIS["rw"] * D.EW
    # ONE main catch light, low on the light (screen-left) side, overlapping
    # the pupil/iris boundary -- same screen placement in both eyes
    hx, hy = cx - 0.30 * rx, cy - 0.18 * ry
    a = math.radians(-18)
    xr = (X - hx) * math.cos(a) + (Y - hy) * math.sin(a)
    yr = -(X - hx) * math.sin(a) + (Y - hy) * math.cos(a)
    e = np.hypot(xr / 0.25, yr / 0.19)
    L.paint(np.clip((1 - e) * 0.19 * L.s + 0.5, 0, 1), EYE["hl"])
    # pinpoint on the opposite upper side
    L.paint(L.disc(cx + 0.46 * rx, cy + 0.24 * ry, 0.075), EYE["hl"])
    return L



MASK_ROWS = [0.0, 0.10, 0.28, 0.55, 1.0]


def opening_loop(side, st):
    up, lo = eye_curves(side, st)
    return np.vstack([up, lo[::-1][1:-1]])


def mask_outer(side):
    cx, cy = D.EYE_C[side]
    loop = opening_loop(side, ST.REST)
    c = np.array([cx + 0.02 * side, cy + 0.10 * D.EW])
    ang = np.arctan2(loop[:, 1] - c[1], loop[:, 0] - c[0])
    rx, ry = 0.74 * D.EW, 0.47 * D.EW
    return np.stack([c[0] + rx * np.cos(ang), c[1] + ry * np.sin(ang)], axis=1)


def mask_points(side, st):
    inner = opening_loop(side, st)
    outer = mask_outer(side)
    return np.vstack([inner + (outer - inner) * f for f in MASK_ROWS])



def paint_lash(side):
    L = P.Layer("Lash_" + side_tag(side), eye_box(side, 0.95, 1.25, 0.5, 1.05), tpu=TPU_E, ss=3)
    up, lo = eye_curves(side, ST.REST)
    n = up_normals(up, side)
    t = np.linspace(0, 1, len(up))
    thk = D.lash_thickness(t)
    # the mass: lid line (a hair below it) up to a flatter top edge
    top = up + n * thk[:, None]
    bot = up - n * 0.035
    # the winged flick past the outer corner: the lid line runs on out and
    # up, the flat top edge comes down to meet it in a point
    wd = D.wing_dir(side, up)
    nw = np.array([-wd[1], wd[0]]) * side          # up-normal of the wing
    tipp = up[-1] + wd * 0.74
    lower = catmull(np.array([up[-3] - n[-3] * 0.035, up[-1] - n[-1] * 0.03, up[-1] + wd * 0.36 - nw * 0.01, tipp]), n=8)
    upper = catmull(np.array([top[-4], top[-1] + wd * 0.05, up[-1] + wd * 0.42 + nw * 0.20, tipp]), n=8)
    m = L.poly(np.vstack([bot[:-3], lower, upper[::-1], top[:-4][::-1]]))
    # two separate lash spikes on the outer quarter, curving outward
    for tt, ln, ang, wd0 in ((0.66, 0.34, 66.0, 0.12), (0.81, 0.46, 44.0, 0.13)):
        k = int(round(tt * (len(up) - 1)))
        base = top[k] - n[k] * 0.06
        a0 = math.radians(ang)
        d0 = np.array([math.cos(a0) * side, math.sin(a0)])
        a1 = math.radians(ang - 26)
        d1 = np.array([math.cos(a1) * side, math.sin(a1)])
        mid = base + d0 * ln * 0.55
        tip = mid + d1 * ln * 0.45
        sp = resample(catmull([base, mid, tip], n=10), n=24)
        m = np.maximum(m, L.stroke(sp, P.taper(24, wd0 * 0.5, head=0.0, tail=0.92, power=0.85)))
    # the inner end: a small downward hook for the tear duct
    ic = up[0]
    hook = np.array([ic + n[0] * 0.08, ic + np.array([-0.10 * side, -0.02]),
                     ic + np.array([-0.05 * side, -0.20]), ic + np.array([0.07 * side, -0.05])])
    m = np.maximum(m, L.poly(catmull(hook, n=6, closed=True)))
    L.paint(m, EYE["lash"])
    # a warm inner edge along the lower 30 % of the mass (reads as lid skin)
    X, Y = L.XY()
    lid_band = L.stroke(up[int(0.08 * len(up)):int(0.86 * len(up))],
                        P.taper(int(0.86 * len(up)) - int(0.08 * len(up)), 0.028, head=0.3, tail=0.3))
    L.paint_in(lid_band, EYE["lash_warm"], 0.55)
    return L


LOWLID_T0 = 0.30


def lower_lid_spine(side, st):
    _, lo = eye_curves(side, st)
    k0 = int(LOWLID_T0 * (len(lo) - 1))
    return lo[k0:]


LOWLID_V = [0.10, 0.0, -0.10, -0.24, -0.36]


def lower_lid_points(side, st):
    sp = lower_lid_spine(side, st)
    return ribbon_points(sp, LOWLID_V, up_normals(sp, side))


def paint_lower_lid(side):
    L = P.Layer("LowerLid_" + side_tag(side), eye_box(side), tpu=TPU_E, ss=3)
    sp = lower_lid_spine(side, ST.REST)
    n = up_normals(sp, side)
    k = len(sp)
    t = np.linspace(0, 1, k)
    # the outer 55-65 %: fades in from the middle, stops short of the corner
    w = 0.050 * smooth(0.18, 0.40, t) * (1 - smooth(0.90, 0.965, t)) + 0.012 * smooth(0.18, 0.4, t) * (1 - smooth(0.93, 0.97, t))
    poly = np.vstack([sp + n * (w * 0.35)[:, None], (sp - n * (w * 0.65)[:, None])[::-1]])
    m = L.poly(poly)
    # one tiny lower-lash tick near the outer end
    kk = int(0.80 * (k - 1))
    base = sp[kk] - n[kk] * 0.02
    d = -n[kk] * 0.75 + np.array([0.65 * side, 0.0])
    d /= np.linalg.norm(d)
    tick = resample(np.array([base, base + d * 0.20]), n=10)
    m = np.maximum(m, L.stroke(tick, P.taper(10, 0.022, head=0.0, tail=0.9)))
    L.paint(m, EYE["lower"])
    return L


def crease_spine(side, st):
    up_r, _ = eye_curves(side, ST.REST)
    up, _ = eye_curves(side, st)
    t = np.linspace(0, 1, len(up_r))
    n = up_normals(up_r, side)
    c = up_r + n * (D.lash_thickness(t) + 0.40)[:, None]
    c = c + 0.45 * (up - up_r)
    k0, k1 = int(0.18 * (len(c) - 1)), int(0.84 * (len(c) - 1))
    return c[k0:k1]


CREASE_V = [0.10, 0.0, -0.10]


def crease_points(side, st):
    sp = crease_spine(side, st)
    return ribbon_points(sp, CREASE_V, up_normals(sp, side))


def paint_crease(side):
    L = P.Layer("Crease_" + side_tag(side), eye_box(side), tpu=TPU_E, ss=3)
    sp = crease_spine(side, ST.REST)
    w = P.taper(len(sp), 0.034, head=0.35, tail=0.30)
    L.paint(L.stroke(sp, w), EYE["crease"])
    return L


# brows

BROW_V = [0.40, 0.20, 0.0, -0.20, -0.40]


def brow_curve(side, st):
    up, down, angry, worried = ST.brow_params(st, side)
    shp = {"rest": 1.0 - up - down - angry - worried, "up": up, "down": down, "angry": angry, "worried": worried}
    return D.brow_spine(side, {k: v for k, v in shp.items() if abs(v) > 1e-9})


def brow_points(side, st):
    sp = brow_curve(side, st)
    return ribbon_points(sp, BROW_V, up_normals(sp, side))


def paint_brow(side):
    sp = brow_curve(side, ST.REST)
    cx, cy = D.EYE_C[side]
    lo, hi = sp.min(axis=0) - 0.6, sp.max(axis=0) + 0.6
    L = P.Layer("Brow_" + side_tag(side), (lo[0], lo[1], hi[0], hi[1]), tpu=TPU_E, ss=3)
    n = up_normals(sp, side)
    t = np.linspace(0, 1, len(sp))
    w = D.brow_width(t)
    # a squarish head cut on a slight angle: the top edge starts a little
    # further in than the bottom edge
    tan = sp[1] - sp[0]
    tan /= np.linalg.norm(tan)
    top = sp + n * (0.55 * w)[:, None]
    bot = sp - n * (0.45 * w)[:, None]
    top[0] = top[0] - tan * 0.10
    bot[0] = bot[0] + tan * 0.05
    poly = np.vstack([bot, top[::-1]])
    m = L.poly(poly)
    L.paint(m, BROW["fill"])
    L.paint_in(L.band_inside(m, 0.045), BROW["edge"])
    # a few hair-direction flicks inside the head (subtle darker strokes)
    for k, (a, b) in enumerate(((0.02, 0.22), (0.06, 0.30), (0.10, 0.36))):
        i0, i1 = int(a * (len(sp) - 1)), int(b * (len(sp) - 1))
        s2 = sp[i0:i1 + 1] + n[i0:i1 + 1] * ((-0.12 + 0.12 * k) * w[i0:i1 + 1])[:, None]
        L.paint_in(L.stroke(resample(s2, n=16), P.taper(16, 0.022, head=0.2, tail=0.8)), BROW["edge"], 0.8)
    return L


# mouth (collapsed at rest; painted at the PAINT pose)

MOUTH_PAINT = dict(open=1.4, wide=0.35, smile=0.0, smirk=0.0)


def mouth_curves(st):
    mp = ST.mouth_params(st)
    return D.mouth(**mp)


def mouth_paint():
    return D.mouth(**MOUTH_PAINT)


MOUTH_BOX = (-6.2, -12.9, 6.4, -4.6)
INSIDE_ROWS = 7


def inside_points(st):
    c = mouth_curves(st)
    return band_points(c["U"], c["L"], INSIDE_ROWS)


def paint_inside():
    c = mouth_paint()
    L = P.Layer("MouthInside", MOUTH_BOX, tpu=TPU_E // 2, ss=3)
    X, Y = L.XY()
    poly = np.vstack([c["U"], c["L"][::-1]])
    m = L.poly(poly)
    L.paint(m, MOUTH["inside"])
    # the throat: the deep tone under the upper lip, a hard curved edge
    U = c["U"]
    top_y = np.interp(X, U[:, 0], U[:, 1])
    L.paint_in(np.clip((Y - (top_y - 0.75 - 0.35 * (X / 5.0) ** 2)) * L.s + 0.5, 0, 1), MOUTH["deep"])
    return L


def teeth_points(st):
    c = mouth_curves(st)
    return band_points(c["U"], c["T"], 3)


def paint_teeth():
    c = mouth_paint()
    L = P.Layer("Teeth", MOUTH_BOX, tpu=TPU_E // 2, ss=3)
    X, Y = L.XY()
    poly = np.vstack([c["U"] + np.array([0, 0.05]), c["T"][::-1]])
    m = L.poly(poly)
    L.paint(m, MOUTH["teeth"])
    # a cool shadow at the corners and under the lip
    mx = D.MOUTH_C[0]
    half = np.abs(c["U"][-1, 0] - c["U"][0, 0]) * 0.5
    L.paint_in(smooth(0.62, 0.82, np.abs(X - mx) / half), MOUTH["teeth_sh"])
    top_y = np.interp(X, c["U"][:, 0], c["U"][:, 1])
    L.paint_in(np.clip((Y - (top_y - 0.10)) * L.s + 0.5, 0, 1), MOUTH["teeth_sh"])
    return L


def tongue_points(st):
    c = mouth_curves(st)
    return band_points(c["G"], c["L"], 4)


def paint_tongue():
    c = mouth_paint()
    L = P.Layer("Tongue", MOUTH_BOX, tpu=TPU_E // 2, ss=3)
    X, Y = L.XY()
    poly = np.vstack([c["G"], c["L"][::-1]])
    m = L.poly(poly)
    L.paint(m, MOUTH["tongue"])
    # a darker back edge and a short centre groove
    g = c["G"]
    gy = np.interp(X, g[:, 0], g[:, 1])
    L.paint_in(np.clip((Y - (gy - 0.16)) * L.s + 0.5, 0, 1), MOUTH["tongue_sh"])
    mx = D.MOUTH_C[0]
    L.paint_in(np.clip(1 - np.hypot((X - mx) / 0.07, (Y - (gy.max() - 0.55)) / 0.42), 0, 1) ** 0.3,
               MOUTH["tongue_sh"], 0.9)
    return L


MLINE_V = [0.26, 0.12, 0.0, -0.10, -0.26]


def mouth_line_points(st):
    c = mouth_curves(st)
    U = c["U"]
    return ribbon_points(U, MLINE_V, D.normals_up(U, +1))


def paint_mouth_line():
    c = mouth_curves(ST.REST)
    U = c["U"]
    L = P.Layer("MouthLine", MOUTH_BOX, tpu=TPU_E, ss=3)
    n = D.normals_up(U, +1)
    k = len(U)
    t = np.linspace(0, 1, k)
    # thickest in the middle, tapering toward the corners
    w = 0.036 + 0.074 * np.sin(t * math.pi) ** 1.2
    w = w * smooth(0.0, 0.05, t) * smooth(1.0, 0.95, t)
    poly = np.vstack([U + n * (0.45 * w)[:, None], (U - n * (0.55 * w)[:, None])[::-1]])
    m = L.poly(poly)
    # the corner tick on the screen-right side (her attitude corner) and a
    # lighter one on the left
    for end, sgn, ln in ((k - 1, 1, 0.26), (0, -1, 0.16)):
        cpt = U[end]
        tick = resample(np.array([cpt + np.array([-0.06 * sgn, 0.05]), cpt + np.array([0.05 * sgn, -0.02]),
                                  cpt + np.array([0.02 * sgn, -ln])]), n=12)
        m = np.maximum(m, L.stroke(tick, P.taper(12, 0.028, head=0.1, tail=0.8)))
    L.paint(m, MOUTH["line"])
    # a thin upper-lip tint just above the line at the centre
    X, Y = L.XY()
    mx, my = D.MOUTH_C
    uy = np.interp(X, U[:, 0], U[:, 1])
    tint = np.clip((Y - uy - 0.03) * L.s + 0.5, 0, 1) * np.clip((uy + 0.12 - Y) * L.s + 0.5, 0, 1) \
        * (1 - smooth(0.55, 1.0, np.abs(X - mx) / 1.2))
    L.paint(tint * (1 - m), SKIN["lip"], 0.85)
    return L


LEDGE_V = [0.10, 0.0, -0.10]


def lower_edge_points(st):
    c = mouth_curves(st)
    Lc = c["L"]
    return ribbon_points(Lc, LEDGE_V, D.normals_up(Lc, +1))


def paint_lower_edge():
    c = mouth_paint()
    Lc = c["L"]
    L = P.Layer("LowerEdge", MOUTH_BOX, tpu=TPU_E // 2, ss=3)
    n = D.normals_up(Lc, +1)
    t = np.linspace(0, 1, len(Lc))
    w = 0.055 * np.sin(t * math.pi) ** 0.8
    poly = np.vstack([Lc + n * (0.5 * w)[:, None], (Lc - n * (0.5 * w)[:, None])[::-1]])
    L.paint(L.poly(poly), MOUTH["line"])
    return L


LLIP_V = [0.10, 0.0, -0.18, -0.40, -0.62, -0.80]


def lower_lip_points(st):
    c = mouth_curves(st)
    Lc = c["L"]
    k = len(Lc)
    i0, i1 = int(0.28 * (k - 1)), int(0.72 * (k - 1)) + 1
    sp = Lc[i0:i1]
    return ribbon_points(sp, LLIP_V, np.tile(np.array([[0.0, 1.0]]), (len(sp), 1)))


def paint_lower_lip():
    c = mouth_curves(ST.REST)
    Lc = c["L"]
    k = len(Lc)
    i0, i1 = int(0.28 * (k - 1)), int(0.72 * (k - 1)) + 1
    sp = Lc[i0:i1]
    L = P.Layer("LowerLip", MOUTH_BOX, tpu=TPU_E, ss=3)
    X, Y = L.XY()
    mx, my = D.MOUTH_C
    ly = np.interp(X, sp[:, 0], sp[:, 1])
    u = (X - mx) / 0.78
    # lip tint: a flat rosy crescent under the line
    depth = 0.30 * np.clip(1 - u ** 2, 0, 1) ** 0.7
    tint = np.clip((ly - 0.05 - Y) * L.s + 0.5, 0, 1) * np.clip((Y - (ly - 0.05 - depth)) * L.s + 0.5, 0, 1)
    L.paint(tint, SKIN["lip"], 0.9)
    # the lower-lip stroke (coloured line) and a tiny crescent shadow under it
    sy = my - 0.54
    stroke = resample(np.array([(mx - 0.62, sy + 0.05), (mx, sy - 0.02), (mx + 0.62, sy + 0.05)]), n=24)
    stroke = resample(catmull(stroke[[0, 12, 23]], n=10), n=24)
    L.paint(L.stroke(stroke, P.taper(24, 0.034, head=0.45, tail=0.45)), SKIN["lip_line"])
    cres = resample(catmull(np.array([(mx - 0.34, sy - 0.16), (mx + 0.02, sy - 0.22), (mx + 0.38, sy - 0.15)]), n=10), n=20)
    L.paint(L.stroke(cres, P.taper(20, 0.035, head=0.5, tail=0.5)), SKIN["shadow"])
    # one small hard gloss on the lower lip
    g = resample(np.array([(mx - 0.30, my - 0.28), (mx - 0.08, my - 0.31)]), n=10)
    L.paint(L.stroke(g, P.taper(10, 0.022, head=0.4, tail=0.4)), SKIN["gloss"], 0.9)
    return L


# blush (animated layers) and the sweat drop

BLUSH_C = {+1: (4.35, -3.20), -1: (-4.30, -3.20)}


def blush_box(side):
    cx, cy = BLUSH_C[side]
    return (cx - 1.7, cy - 0.9, cx + 1.7, cy + 0.9)


def blush_mesh(side):
    x0, y0, x1, y1 = blush_box(side)
    xs = np.linspace(x0, x1, 9)
    ys = np.linspace(y0, y1, 5)
    return np.array([(x, y) for y in ys for x in xs]), grid_tris(5, 9)


def paint_blush(side, name, alpha, sx=1.45, sy=0.58):
    L = P.Layer(name + "_" + side_tag(side), blush_box(side), tpu=TPU_F, ss=2)
    X, Y = L.XY()
    cx, cy = BLUSH_C[side]
    r = np.hypot((X - cx) / sx, (Y - cy) / sy)
    L.paint(np.clip(1 - r, 0, 1) ** 0.9, SKIN["blush"], alpha)
    return L


HATCH_N = 6


def hatch_strokes(side):
    cx, cy = BLUSH_C[side]
    out = []
    for k in range(HATCH_N):
        x = cx + (k - (HATCH_N - 1) / 2.0) * 0.34 + 0.06 * ((k * 7) % 3 - 1)
        ln = 0.72 + 0.14 * ((k * 5) % 3 - 1) - 0.12 * abs(k - (HATCH_N - 1) / 2.0) / 2.5
        a = math.radians(25.0)
        d = np.array([math.sin(a), math.cos(a)])
        mid = np.array([x, cy + 0.04 * ((k * 3) % 2)])
        out.append((mid - d * ln * 0.5, mid + d * ln * 0.5))
    return out


def hatch_points(side, st):
    """Each stroke is a 2x6 strip; at 0 it collapses onto its lower end (so
    the strokes draw in, bottom to top, as the blush rises)."""
    g = st["hatch"]
    pts = []
    for a, b in hatch_strokes(side):
        d = b - a
        d /= np.linalg.norm(d)
        n = np.array([-d[1], d[0]])
        for v in (-0.07, 0.07):
            for s in np.linspace(0, 1, 6):
                p = a + (b - a) * (s * g) + n * v * max(g, 1e-3)
                pts.append(p)
    return np.array(pts)


def hatch_rest_full(side):
    st = ST.S(hatch=1.0)
    return hatch_points(side, st)


def hatch_tris():
    tris = []
    for k in range(HATCH_N):
        base = k * 12
        t = grid_tris(2, 6) + base
        tris.append(t)
    return np.vstack(tris)


def paint_hatch(side):
    L = P.Layer("Hatch_" + side_tag(side), blush_box(side), tpu=TPU_E, ss=3)
    for a, b in hatch_strokes(side):
        sp = resample(np.array([a, b]), n=16)
        L.paint(L.stroke(sp, P.taper(16, 0.032, head=0.35, tail=0.35)), SKIN["hatch"])
    return L


def blush_pop_points(side, st, local):
    c = np.array(BLUSH_C[side])
    g = max(st["blush"], 1e-3)
    return c + (local - c) * np.array([0.35 + 0.65 * g, g])


SWEAT_A = (6.55, 3.3)


def sweat_shape():
    x, y = SWEAT_A
    return catmull(np.array([(x, y + 0.75), (x + 0.36, y - 0.12), (x + 0.2, y - 0.46), (x - 0.12, y - 0.5),
                             (x - 0.3, y - 0.18)]), n=10, closed=True)


def sweat_mesh():
    x, y = SWEAT_A
    xs = np.linspace(x - 0.55, x + 0.6, 5)
    ys = np.linspace(y - 0.75, y + 0.95, 6)
    return np.array([(a, b) for b in ys for a in xs]), grid_tris(6, 5)


def sweat_points(st, local):
    """0 hidden at the temple; 0.35 popped in; 0.9 slid down the temple;
    1.0 fallen off (collapsed) at the bottom -- so a take can reset it to 0
    while it is hidden."""
    g = st["sweat"]
    x, y = SWEAT_A
    size = min(g / 0.35, 1.0) * (1.0 - min(max((g - 0.9) / 0.1, 0.0), 1.0))
    slide = 1.45 * min(max((g - 0.35) / 0.55, 0.0), 1.0) + 0.25 * min(max((g - 0.9) / 0.1, 0.0), 1.0)
    c = np.array([x, y - 0.1])
    p = c + (local - c) * max(size, 1e-3)
    return p + np.array([0.10 * slide, -slide])


def paint_sweat():
    x, y = SWEAT_A
    L = P.Layer("Sweat", (x - 0.55, y - 0.75, x + 0.6, y + 0.95), tpu=TPU_E, ss=3)
    s = sweat_shape()
    m = L.poly(s)
    L.paint(m, "#CFEFFF")
    L.paint_in(L.band_inside(m, 0.07), "#3B6FB0")
    L.paint_in(L.disc(x - 0.08, y - 0.22, 0.09), "#FFFFFF")
    return L


# ear + earring (screen-left, the tucked side)

EAR_BOX = (-9.3, -5.3, -6.6, 1.4)


def paint_ear():
    L = P.Layer("Ear", EAR_BOX, tpu=TPU_F * 2, ss=3)
    X, Y = L.XY()
    o = D.ear_outline()
    m = L.poly(o)
    L.paint(m, SKIN["base"])
    # the inner ear: a C-shaped fold in the shadow tone, a deep bowl
    inner = catmull(np.array([(-7.55, 0.05), (-8.18, 0.0), (-8.38, -1.0), (-8.22, -2.3), (-7.86, -3.2),
                              (-7.55, -3.1), (-7.7, -2.0), (-7.62, -0.9)]), n=12, closed=True)
    L.paint_in(L.poly(inner), SKIN["shadow"])
    bowl = catmull(np.array([(-7.62, -0.7), (-7.98, -1.1), (-7.92, -2.1), (-7.6, -2.5), (-7.45, -1.6)]), n=10, closed=True)
    L.paint_in(L.poly(bowl), SKIN["deep"])
    fold = resample(catmull(np.array([(-7.62, 0.2), (-8.28, -0.1), (-8.42, -1.2), (-8.2, -2.6), (-7.78, -3.45)]), n=10), n=30)
    L.paint_in(L.stroke(fold, P.taper(30, 0.035, head=0.3, tail=0.4)), SKIN["line"])
    # contour, heavier on the outer rim
    d = L.inside_dist(m)
    lw = 0.05 + 0.03 * smooth(-7.2, -8.6, X)
    L.paint_in(np.clip((lw - d) * L.s + 0.5, 0, 1) * m, SKIN["contour"])
    return L


def paint_earring():
    x, y = D.EARRING_TOP
    L = P.Layer("Earring", (x - 0.5, y - 2.4, x + 0.5, y + 0.3), tpu=TPU_E, ss=3)
    X, Y = L.XY()
    gold, gold_sh, glint, stone, stone_sh = "#E3B35A", "#A87A2C", "#FFFFFF", "#5C7FB0", "#35507A"
    # a small hoop at the lobe, a thin bar, a slate-blue drop
    hoop = L.disc(x, y - 0.05, 0.16) * (1 - L.disc(x, y - 0.05, 0.08))
    bar = L.stroke(resample(np.array([(x, y - 0.18), (x, y - 1.55)]), n=12), 0.045)
    drop = L.poly(catmull(np.array([(x, y - 1.50), (x + 0.26, y - 1.85), (x, y - 2.25), (x - 0.26, y - 1.85)]), n=8, closed=True))
    L.paint(np.maximum(hoop, bar), gold)
    L.paint_in(np.clip((X - x) * L.s + 0.5, 0, 1) * np.maximum(hoop, bar), gold_sh)
    L.paint(drop, stone)
    L.paint_in(np.clip((X - x - 0.02) * L.s * 0.6 + 0.5, 0, 1) * drop, stone_sh)
    L.paint_in(L.disc(x - 0.08, y - 1.78, 0.05) * drop, glint)
    L.paint_in(L.band_inside(np.maximum(np.maximum(hoop, bar), drop), 0.022), OUTLINE)
    return L


# neck (behind the face, under the collar)

NECK_BOX = (-8.5, -20.0, 8.5, -3.5)


def paint_neck():
    L = P.Layer("Neck", NECK_BOX, tpu=TPU_F, ss=3)
    X, Y = L.XY()
    o = D.neck_outline()
    m = L.poly(o)
    L.paint(m, SKIN["base"])
    # the head's shadow: a big shape under the jaw, cut diagonally (high on
    # the shadow side), deepest right under the chin
    jaw = D.face_outline()
    jaw = jaw[jaw[:, 1] < -3.0]
    jaw = jaw[np.argsort(jaw[:, 0])]
    jy = np.interp(X, jaw[:, 0], jaw[:, 1])
    cut = jy - 2.3 + 0.28 * X          # diagonal: lower on the lit (left) side
    sh = np.clip((Y - cut) * L.s + 0.5, 0, 1)
    L.paint_in(sh, SKIN["shadow"])
    deep = np.clip((Y - (jy - 0.55 + 0.05 * X)) * L.s + 0.5, 0, 1) * (np.abs(X) < 2.8)
    L.paint_in(deep, SKIN["deep"])
    # sternocleidomastoid: two short converging strokes; the pit of the neck
    for sgn in (1, -1):
        sp = resample(np.array([(2.3 * sgn, -10.6), (1.5 * sgn, -12.4), (0.55 * sgn, -13.6)]), n=16)
        L.paint_in(L.stroke(sp, P.taper(16, 0.035, head=0.5, tail=0.4)), SKIN["line"], 0.8)
    pit = resample(np.array([(-0.32, -13.9), (0.0, -14.25), (0.32, -13.9)]), n=12)
    L.paint_in(L.stroke(pit, P.taper(12, 0.03, head=0.4, tail=0.4)), SKIN["line"], 0.9)
    # contour lines down the neck sides
    d = L.inside_dist(m)
    lw = 0.075 * (np.abs(X) > 2.5) * smooth(-4.0, -6.5, Y)
    L.paint_in(np.clip((lw - d) * L.s + 0.5, 0, 1) * m, SKIN["contour"])
    return L


# face mesh generator: rest + the jaw field of the mouth state

def jaw_field(pts, st):
    """Face-point displacement of the mouth state: the lower face drops
    with the jaw (smoothly, fading toward the ears), the cheeks follow the
    corners as the mouth widens. The skin right under the lips is covered
    by the lip parts, which ride the lip curves themselves."""
    c = mouth_curves(st)
    c0 = mouth_curves(ST.REST)
    Lc, L0 = c["L"], c0["L"]
    mx, my = D.MOUTH_C
    x, y = pts[:, 0], pts[:, 1]
    k = len(Lc) // 2
    J = 0.44 * max(L0[k, 1] - Lc[k, 1], 0.0)
    dy = -J * smooth(my + 0.6, my - 2.4, y) * (0.30 + 0.70 * np.exp(-(x / 4.8) ** 2))
    half = 0.5 * (Lc[-1, 0] - Lc[0, 0])
    half0 = 0.5 * (L0[-1, 0] - L0[0, 0])
    dw = half - half0
    dx = np.sign(x - mx) * dw * 0.45 * np.exp(-((y - my) / 1.9) ** 2) * np.exp(-((np.abs(x - mx) - half0) / 2.6) ** 2)
    return np.stack([dx, dy], axis=1)


def face_mesh():
    out = D.face_outline()
    pts, tris = fill_polygon(out, 0.32, 0.42)
    return pts, tris


# assemble

def build(fringe_shadow=None, lock_shadows=(), log=print):
    parts, texes = [], {}

    def tex(name, painter, bleed=12):
        t = painted(name, painter, bleed)
        texes[t.name] = t
        return t

    # face + contour
    tf = tex("Face", lambda: paint_face(fringe_shadow, lock_shadows))
    fp, ft = face_mesh()
    parts.append(Part("Face", "Face", Z["Face"], "Face", fp, ft,
                      gen=lambda st, p=fp: p + jaw_field(p, st)))
    tl = tex("FaceLine", paint_face_line)
    idx = face_line_indices()

    def face_line_pts(st):
        out = D.face_outline()
        out = out + jaw_field(out, st)
        sp = out[idx]
        n = P.normals(sp)
        return ribbon_points(sp, [-0.02, 0.24], n)
    fl = face_line_pts(ST.REST)
    parts.append(Part("FaceLine", "Face", Z["FaceLine"], "FaceLine", fl, grid_tris(2, len(idx)), gen=face_line_pts))

    for side in (+1, -1):
        tg = side_tag(side)
        tex("Sclera_" + tg, lambda: paint_sclera(side))
        p = sclera_points(side, ST.REST)
        parts.append(Part("Sclera_" + tg, "Eyes", Z["Sclera"], "Sclera_" + tg, p, grid_tris(5, D.LID_N),
                          gen=lambda st, s=side: sclera_points(s, st)))
        tex("LidShadow_" + tg, lambda: paint_lid_shadow(side))
        p = lid_shadow_points(side, ST.REST)
        parts.append(Part("LidShadow_" + tg, "Eyes", Z["LidShadow"], "LidShadow_" + tg, p,
                          grid_tris(len(LIDSH_V), D.LID_N), gen=lambda st, s=side: lid_shadow_points(s, st)))
        tex("Iris_" + tg, lambda: paint_iris(side))
        loc, tr = iris_mesh()
        p = iris_points(side, ST.REST, loc)
        parts.append(Part("Iris_" + tg, "Eyes", Z["Iris"], "Iris_" + tg, p, tr,
                          gen=lambda st, s=side, l=loc: iris_points(s, st, l)))
        tex("Highlight_" + tg, lambda: paint_highlight(side))
        loc, tr = highlight_mesh()
        p = highlight_points(side, ST.REST, loc)
        parts.append(Part("Highlight_" + tg, "Eyes", Z["Highlight"], "Highlight_" + tg, p, tr,
                          gen=lambda st, s=side, l=loc: highlight_points(s, st, l)))
        p = mask_points(side, ST.REST)
        nloop = len(opening_loop(side, ST.REST))
        parts.append(Part("EyeMask_" + tg, "Eyes", Z["EyeMask"], "Face", p, grid_tris(len(MASK_ROWS), nloop, closed=True),
                          gen=lambda st, s=side: mask_points(s, st)))
        tex("Lash_" + tg, lambda: paint_lash(side))
        p = lash_points(side, ST.REST)
        nsp = D.LID_N + LASH_PRE + LASH_POST
        parts.append(Part("Lash_" + tg, "Eyes", Z["Lash"], "Lash_" + tg, p, grid_tris(len(LASH_V), nsp),
                          gen=lambda st, s=side: lash_points(s, st)))
        tex("LowerLid_" + tg, lambda: paint_lower_lid(side))
        p = lower_lid_points(side, ST.REST)
        nsp = len(lower_lid_spine(side, ST.REST))
        parts.append(Part("LowerLid_" + tg, "Eyes", Z["LowerLid"], "LowerLid_" + tg, p, grid_tris(len(LOWLID_V), nsp),
                          gen=lambda st, s=side: lower_lid_points(s, st)))
        tex("Crease_" + tg, lambda: paint_crease(side))
        p = crease_points(side, ST.REST)
        nsp = len(crease_spine(side, ST.REST))
        parts.append(Part("Crease_" + tg, "Eyes", Z["Crease"], "Crease_" + tg, p, grid_tris(len(CREASE_V), nsp),
                          gen=lambda st, s=side: crease_points(s, st)))
        tex("Brow_" + tg, lambda: paint_brow(side))
        p = brow_points(side, ST.REST)
        parts.append(Part("Brow_" + tg, "Brows", Z["Brow"], "Brow_" + tg, p, grid_tris(len(BROW_V), D.BROW_N),
                          gen=lambda st, s=side: brow_points(s, st)))
        # blush: a constant faint base, a pop-in patch and the hatching
        tex("BlushBase_" + tg, lambda: paint_blush(side, "BlushBase", 0.34))
        bp, bt = blush_mesh(side)
        parts.append(Part("BlushBase_" + tg, "Face", Z["BlushBase"], "BlushBase_" + tg, bp, bt, material="blend"))
        tex("BlushPop_" + tg, lambda: paint_blush(side, "BlushPop", 0.55, 1.6, 0.66))
        parts.append(Part("BlushPop_" + tg, "Face", Z["BlushPop"], "BlushPop_" + tg,
                          blush_pop_points(side, ST.REST, bp), bt, uv_pts=bp,
                          gen=lambda st, s=side, l=bp: blush_pop_points(s, st, l), material="blend"))
        tex("Hatch_" + tg, lambda: paint_hatch(side))
        parts.append(Part("Hatch_" + tg, "Face", Z["Hatch"], "Hatch_" + tg, hatch_points(side, ST.REST),
                          hatch_tris(), uv_pts=hatch_rest_full(side), gen=lambda st, s=side: hatch_points(s, st)))

    # mouth
    for name, painter, fn, rows in (("MouthInside", paint_inside, inside_points, INSIDE_ROWS),
                                    ("Teeth", paint_teeth, teeth_points, 3),
                                    ("Tongue", paint_tongue, tongue_points, 4),
                                    ("LowerEdge", paint_lower_edge, lower_edge_points, len(LEDGE_V))):
        tex(name, painter)
        paint_st = ST.S({"mouth.open": MOUTH_PAINT["open"], "mouth.wide": MOUTH_PAINT["wide"]})
        parts.append(Part(name, "Mouth", Z[name], name, fn(ST.REST), grid_tris(rows, D.MOUTH_N),
                          uv_pts=fn(paint_st), gen=fn))
    tex("MouthLine", paint_mouth_line)
    parts.append(Part("MouthLine", "Mouth", Z["MouthLine"], "MouthLine", mouth_line_points(ST.REST),
                      grid_tris(len(MLINE_V), D.MOUTH_N), gen=mouth_line_points))
    tex("LowerLip", paint_lower_lip)
    p = lower_lip_points(ST.REST)
    parts.append(Part("LowerLip", "Mouth", Z["LowerLip"], "LowerLip", p,
                      grid_tris(len(LLIP_V), len(p) // len(LLIP_V)), gen=lower_lip_points))
    # sweat drop
    tex("Sweat", paint_sweat)
    sp_, stt = sweat_mesh()
    parts.append(Part("Sweat", "Fx", Z["Sweat"], "Sweat", sweat_points(ST.REST, sp_), stt, uv_pts=sp_,
                      gen=lambda st, l=sp_: sweat_points(st, l)))
    # ear, earring, neck: static art on hull meshes
    for name, painter, grp, h in (("Ear", paint_ear, "Face", 0.3), ("Earring", paint_earring, "Earring", 0.14),
                                  ("Neck", paint_neck, "Neck", 0.5)):
        t = tex(name, painter)
        from parts import hull_mesh
        p, tr = hull_mesh(t.rgba, t.bbox, h, h * 1.4, pad_texels=4)
        parts.append(Part(name, grp, Z[name], name, p, tr))
    return parts, texes
