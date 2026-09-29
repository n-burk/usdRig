"""Kaede -- the construction of the character.

Every feature is a set of curves that is a FUNCTION OF AN EXPRESSION STATE
sampled with a fixed count, so a keyform is simply the same construction
evaluated at another state and the painted texture rides along.

Model units: H = 20 (top of the skull y = +10, chin y = -10), y up, x to
screen right. ``side`` +1 is the screen-right feature (her left), -1 the
screen-left one. Eye-local coordinates are in model units: u runs from the
inner corner (-2.0) to the outer corner (+2.02), w is up.

The geometry uses the proportions specified by the constants below.
"""

import math

import numpy as np

from paint import catmull, resample, smooth

# palette (display sRGB)

SKIN = dict(base="#FCE6D8", shadow="#EDB7A4", deep="#D48A7C", line="#9A4E4A",
            blush="#F6A5A2", hatch="#D9535F", lip="#F3B7AD", lip_line="#B75A5C",
            gloss="#FFF3EE", mole="#5B3438", hi="#FFF4EC")
LINE = "#231A1F"                         # warm near-black contour
EYE = dict(lash="#17121A", lash_warm="#3B2230", lower="#3A2A32", crease="#9A4E4A",
           white="#FFFFFF", lid_shadow="#D8D2EA", ir_top="#241B3B", ir_mid="#54448F",
           ir_low="#A796E0", pupil="#120E20", ring="#140F22", catch="#FFFFFF",
           duct="#F1B3AE")
BROW = dict(fill="#232438", edge="#101019")
HAIR = dict(base="#272B47", shadow="#191B30", deep="#0F101D", hi="#5E6A9C",
            line="#0C0B15", inner="#7B5CCB", inner_sh="#523A99", inner_hi="#AF98EE")
MOUTH = dict(inside="#5A1A26", deep="#3A0E18", tongue="#E5707A", tongue_sh="#BF4F5D",
             teeth="#FFFDF8", teeth_sh="#D8D5E3", line="#3B1820")
CARDI = dict(base="#F4D06F", shadow="#D9A94A", deep="#B8843A", line="#4A3017",
             rib="#C99A43", knit="#EAC35E")
SHIRT = dict(base="#F7F7F4", shadow="#D5D3E4", deep="#B9B6CF", line="#3A3444")
RIBBON = dict(base="#2F4FC4", shadow="#20378F", deep="#16245E", line="#0E1538",
              hi="#6F88E8")
SILVER = dict(base="#D9DEE6", shadow="#8B92A0", glint="#FFFFFF", line="#3C4150")
PIN = dict(base="#F6D24A", shadow="#D39E2A", line="#3A2A10")
FX = dict(vein="#E0413C", vein_line="#7A1520", sweat="#CFEFFF", sweat_line="#3B6FB0")

# landmarks

EW = 4.0
EYE_C = {+1: (4.0, -0.80), -1: (-4.0, -0.80)}
NOSE_TIP = (0.12, -4.55)
MOUTH_C = (0.04, -6.72)
HEAD_PIVOT = (0.0, -8.5)
NECK_PIVOT = (0.0, -14.0)
BODY_PIVOT = (0.0, -40.0)

LID_N = 49
BROW_N = 41
MOUTH_N = 41


def lerp(a, b, t):
    return a + (b - a) * t


# face outline

FACE_R = [(0.0, 9.9), (4.7, 9.35), (7.0, 7.4), (7.6, 4.2), (7.55, 1.0), (7.35, -1.6),
          (6.95, -3.6), (6.05, -5.8), (4.3, -7.85), (2.35, -9.4), (0.9, -9.95)]


def face_outline(n=24):
    """Closed face outline, starting top centre, screen-right side first."""
    pts = FACE_R + [(0.0, -10.02)] + [(-x, y) for x, y in reversed(FACE_R[1:])]
    return catmull(pts, n=n, closed=True)


def face_halfwidth(y):
    o = face_outline(40)
    r = o[o[:, 0] >= 0]
    r = r[np.argsort(r[:, 1])]
    return np.interp(y, r[:, 1], r[:, 0])


def jaw_line(n=97):
    """The visible contour (FaceLine spine): from under the screen-left ear
    round the chin to under the screen-right ear. Starts at y=+3.2."""
    o = face_outline(48)
    # walk the closed outline from the right side at y=3.2 down and round
    k0 = np.argmin(np.where(o[:, 0] > 0, np.abs(o[:, 1] - 3.2), 1e9))
    k1 = np.argmin(np.where(o[:, 0] < 0, np.abs(o[:, 1] - 3.2), 1e9))
    if k1 < k0:
        seg = np.vstack([o[k0:], o[:k1 + 1]])
    else:
        seg = o[k0:k1 + 1]
    return resample(seg[::-1], n=n)           # screen-left ear -> screen-right ear


# eyes

EYE_STATES = {
    "open": ([(-2.0, -0.10), (-1.55, 0.40), (-0.8, 0.74), (0.1, 0.86), (0.95, 0.80), (1.6, 0.56), (2.02, 0.28)],
             [(-2.0, -0.10), (-1.45, -0.52), (-0.6, -0.84), (0.3, -0.90), (1.15, -0.72), (1.72, -0.30), (2.02, 0.28)]),
    "wide": ([(-2.0, -0.08), (-1.55, 0.66), (-0.8, 1.06), (0.1, 1.20), (0.95, 1.14), (1.6, 0.86), (2.05, 0.38)],
             [(-2.0, -0.12), (-1.45, -0.62), (-0.6, -1.00), (0.3, -1.06), (1.15, -0.88), (1.72, -0.44), (2.05, 0.38)]),
    "half": ([(-2.0, -0.13), (-1.55, 0.05), (-0.8, 0.22), (0.1, 0.29), (0.95, 0.27), (1.6, 0.17), (2.02, 0.08)],
             [(-2.0, -0.13), (-1.45, -0.49), (-0.6, -0.79), (0.3, -0.85), (1.15, -0.67), (1.72, -0.28), (2.02, 0.08)]),
    "closed": ([(-2.0, -0.16), (-1.55, -0.38), (-0.8, -0.56), (0.1, -0.61), (0.95, -0.53), (1.6, -0.32), (2.02, -0.10)],
               [(-2.0, -0.18), (-1.55, -0.40), (-0.8, -0.58), (0.1, -0.63), (0.95, -0.55), (1.6, -0.34), (2.02, -0.12)]),
    "smile": ([(-2.0, -0.46), (-1.55, -0.10), (-0.8, 0.16), (0.1, 0.25), (0.95, 0.17), (1.6, -0.07), (2.02, -0.36)],
              [(-2.0, -0.48), (-1.55, -0.12), (-0.8, 0.14), (0.1, 0.23), (0.95, 0.15), (1.6, -0.09), (2.02, -0.38)]),
    "squint": ([(-2.0, -0.10), (-1.55, 0.34), (-0.8, 0.64), (0.1, 0.74), (0.95, 0.69), (1.6, 0.48), (2.02, 0.26)],
               [(-2.0, -0.10), (-1.45, -0.30), (-0.6, -0.44), (0.3, -0.47), (1.15, -0.36), (1.72, -0.08), (2.02, 0.26)]),
    "flat": ([(-2.0, -0.11), (-1.55, 0.17), (-0.8, 0.36), (0.1, 0.45), (0.95, 0.46), (1.6, 0.38), (2.02, 0.25)],
             [(-2.0, -0.11), (-1.45, -0.48), (-0.6, -0.78), (0.3, -0.84), (1.15, -0.67), (1.72, -0.27), (2.02, 0.25)]),
}

IRIS = dict(cu=0.05, cw=0.26, rx=0.95, ry=1.15)
PUPIL = dict(du=0.0, dw=0.14, rx=0.34, ry=0.50)
LOOK = dict(x=0.62, up=0.34, down=0.36)          # iris travel (mu) at Look +-1
CATCH_FOLLOW = 0.4                               # catchlight moves 0.4 of the iris


def eye_to_model(side, uw):
    uw = np.asarray(uw, np.float64)
    cx, cy = EYE_C[side]
    return np.stack([cx + side * uw[:, 0], cy + uw[:, 1]], axis=1)


def lid_uw(state, which):
    ctrl = EYE_STATES[state][which]
    return resample(catmull(ctrl, n=24), n=LID_N)


def lid(side, state, which):
    """LID_N points inner -> outer, model units. which 0 upper, 1 lower."""
    return eye_to_model(side, lid_uw(state, which))


def lash_spine_uw(state):
    """Upper lid extended past the outer corner along its end tangent,
    bent up (the winged flick direction), plus a short inner run."""
    up = lid_uw(state, 0)
    d = up[-1] - up[-4]
    d /= np.linalg.norm(d)
    bend = {"closed": -0.25, "smile": -0.10}.get(state, 0.42)
    c, s = math.cos(bend), math.sin(bend)
    d = np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]])
    ext = [up[-1] + d * k for k in (0.14, 0.28, 0.42, 0.56, 0.70)]
    return np.vstack([up, ext])


def crease_uw(state):
    up_o = lid_uw("open", 0)
    up = lid_uw(state, 0)
    k = {"smile": 0.85, "closed": 0.55}.get(state, 0.6)
    lift = 0.72 + 0.10 * smooth(-2.0, 0.5, up_o[:, 0])
    return np.stack([up_o[:, 0] * 0.96 + up[:, 0] * 0.04, up_o[:, 1] + lift + k * (up[:, 1] - up_o[:, 1])], axis=1)


def iris_centre(side, look=(0.0, 0.0)):
    lx, ly = look
    du = lx * LOOK["x"] * side
    dw = ly * (LOOK["up"] if ly > 0 else LOOK["down"])
    cx, cy = EYE_C[side]
    return np.array([cx + side * IRIS["cu"] + side * du, cy + IRIS["cw"] + dw])


# brows

BROW_STATES = {
    # (x, y) of the brow spine, head (inner) -> tail, for the screen-right brow
    "rest": [(1.55, 1.40), (2.6, 1.56), (3.8, 1.67), (4.9, 1.68), (5.7, 1.50), (6.25, 1.20)],
    "up": [(1.55, 2.02), (2.6, 2.26), (3.8, 2.42), (4.9, 2.43), (5.7, 2.22), (6.25, 1.88)],
    "down": [(1.55, 0.98), (2.6, 1.10), (3.8, 1.20), (4.9, 1.23), (5.7, 1.08), (6.25, 0.82)],
    "angry": [(1.62, 0.78), (2.6, 1.12), (3.8, 1.46), (4.9, 1.70), (5.7, 1.66), (6.25, 1.42)],
    "sad": [(1.55, 2.02), (2.6, 2.00), (3.8, 1.82), (4.9, 1.62), (5.7, 1.36), (6.2, 1.02)],
}
BROW_W = [(0.0, 0.25, 0.23), (0.10, 0.24, 0.22), (0.45, 0.19, 0.16), (0.75, 0.13, 0.10),
          (0.92, 0.07, 0.05), (1.0, 0.012, 0.01)]     # (t, below, above) half widths


def brow_spine(side, state):
    ctrl = np.array(BROW_STATES[state], np.float64)
    ctrl[:, 0] *= side
    return resample(catmull(ctrl, n=24), n=BROW_N)


def brow_widths(n=BROW_N):
    t = np.linspace(0, 1, n)
    tt, lo, hi = zip(*BROW_W)
    return np.interp(t, tt, lo), np.interp(t, tt, hi)


# mouth

# a mouth state is (half width, centre dx, centre dy, corner lift L, corner
# lift R, upper-lip rise at the middle, opening depth, squareness of the
# bottom (0 round .. 1 flat), upper width fraction (trapezoid top))
MOUTH_STATES = {
    "rest": dict(hw=1.62, dx=0.0, dy=0.0, cl=-0.03, cr=0.02, up=0.02, dep=0.0, sq=0.5, top=1.0),
    "A": dict(hw=1.74, dx=0.0, dy=-0.10, cl=0.02, cr=0.02, up=0.20, dep=1.70, sq=0.55, top=0.86),
    "I": dict(hw=1.95, dx=0.0, dy=0.0, cl=0.08, cr=0.08, up=0.08, dep=0.48, sq=0.9, top=1.0),
    "U": dict(hw=0.92, dx=0.0, dy=-0.05, cl=-0.02, cr=-0.02, up=0.16, dep=0.62, sq=0.1, top=0.9),
    "E": dict(hw=1.84, dx=0.0, dy=-0.05, cl=0.05, cr=0.05, up=0.12, dep=0.95, sq=0.75, top=0.95),
    "O": dict(hw=1.16, dx=0.0, dy=-0.12, cl=-0.02, cr=-0.02, up=0.26, dep=1.30, sq=0.15, top=0.85),
    "yell": dict(hw=3.9, dx=0.0, dy=-0.5, cl=0.10, cr=0.10, up=0.30, dep=5.0, sq=0.9, top=0.64, trap=1.0),
    "smile": dict(hw=1.86, dx=0.0, dy=0.05, cl=0.40, cr=0.40, up=0.05, dep=0.0, sq=0.5, top=1.0),
    "frown": dict(hw=1.50, dx=0.0, dy=-0.02, cl=-0.34, cr=-0.34, up=0.06, dep=0.0, sq=0.5, top=1.0),
    "cornerL": dict(hw=1.70, dx=-0.16, dy=0.03, cl=0.60, cr=0.0, up=0.03, dep=0.0, sq=0.5, top=1.0),
    "cornerR": dict(hw=1.70, dx=0.16, dy=0.03, cl=-0.04, cr=0.64, up=0.03, dep=0.0, sq=0.5, top=1.0),
    "pout": dict(hw=0.78, dx=0.0, dy=-0.08, cl=-0.06, cr=-0.06, up=0.10, dep=0.0, sq=0.5, top=1.0),
    "shiftL": dict(hw=1.60, dx=-0.42, dy=0.0, cl=-0.03, cr=0.02, up=0.02, dep=0.0, sq=0.5, top=1.0),
    "shiftR": dict(hw=1.60, dx=0.42, dy=0.0, cl=-0.03, cr=0.02, up=0.02, dep=0.0, sq=0.5, top=1.0),
    # the reference layout the mouth-interior texture is painted in
    "ref": dict(hw=2.2, dx=0.0, dy=0.0, cl=0.0, cr=0.0, up=0.2, dep=2.2, sq=0.6, top=0.9),
}


def mouth_curves(state):
    """(upper, lower) lip curves, MOUTH_N points each, screen-left corner ->
    screen-right corner, model units. Upper == lower when closed."""
    p = MOUTH_STATES[state] if isinstance(state, str) else state
    s = np.linspace(-1, 1, MOUTH_N)
    cx, cy = MOUTH_C
    cx += p["dx"]
    cy += p["dy"]
    hw = p["hw"]
    # corner lift, linear across plus a gentle sag
    lift = np.where(s < 0, p["cl"], p["cr"]) * np.abs(s) ** 1.6
    sag = -0.05 * s * s
    base_y = cy + lift + sag
    # upper lip: a rise in the middle, flat toward the corners
    top = p["top"]
    a = np.abs(s)
    rise = p["up"] * np.clip(1 - a ** 2, 0, 1) ** 0.7
    upper = np.stack([cx + hw * top * s, base_y + rise], axis=1)
    # lower lip: the opening depth profile, round (sq 0) .. flat-bottomed;
    # the bottom is wider than the top for a trapezoid (top < 1)
    round_prof = np.sqrt(np.clip(1 - a ** 2, 0, 1))
    flat_prof = np.clip(1 - a ** 6, 0, 1) ** 0.5
    prof = lerp(round_prof, flat_prof, p["sq"])
    lower = np.stack([cx + hw * s * lerp(top, 1.0, prof), base_y - p["dep"] * prof], axis=1)
    trap = p.get("trap", 0.0)
    if trap:
        # a hard trapezoid: straight sides from the top corners out to the
        # bottom corners, a nearly flat bottom (the long edge)
        side = np.clip((a - 0.62) / 0.38, 0, 1)             # 1 at the corners
        bx = hw * np.sign(s) * np.where(a > 0.62, lerp(1.0, top, side), a / 0.62)
        by = -p["dep"] * np.where(a > 0.62, 1.0 - side, 1.0 + 0.04 * (1 - (a / 0.62) ** 2))
        corner = 0.10 * np.exp(-((a - 0.62) / 0.06) ** 2)      # round the bottom corners a touch
        trapz = np.stack([cx + bx * (1 - corner * 0.5), base_y + by + corner * p["dep"] * 0.3], axis=1)
        lower = lower * (1 - trap) + trapz * trap
    return upper, lower


def mouth_blend(weights):
    """Linear blend of mouth-state parameter sets (for previews only)."""
    keys = MOUTH_STATES["rest"].keys()
    out = dict(MOUTH_STATES["rest"])
    for k in keys:
        out[k] = MOUTH_STATES["rest"][k] + sum(w * (MOUTH_STATES[n][k] - MOUTH_STATES["rest"][k])
                                               for n, w in weights.items())
    return out
