"""Shion -- the construction of the character: every feature is a set of
curves that is a function of an expression STATE, with a fixed sampling, so
a keyform is simply the same construction evaluated at another state.

Model units: H = 20 (top of the skull y = +10, chin y = -10), y up, x to
screen right. ``side`` +1 is the screen-right feature (her left), -1 the
screen-left one. Eye-local units: EW (eye width) = 4, u runs from the inner
corner (-0.5) to the outer corner (+0.52), w up.

The geometry uses the proportions specified by the constants below.
"""

import math

import numpy as np

from paint import catmull, resample, smooth

# landmarks

EW = 4.2
EYE_C = {+1: (4.12, -0.80), -1: (-4.12, -0.80)}
NOSE_TIP = (0.18, -4.55)
MOUTH_C = (0.08, -6.72)
HEAD_PIVOT = (0.0, -8.6)
NECK_PIVOT = (0.0, -14.2)
BODY_PIVOT = (0.0, -42.0)

LID_N = 49          # samples per lid (inner -> outer)
BROW_N = 41
MOUTH_N = 49


def lerp(a, b, t):
    return a + (b - a) * t


# face outline

FACE_R = [(0.0, 9.9), (4.7, 9.35), (7.0, 7.4), (7.55, 4.2), (7.5, 1.0), (7.32, -1.6),
          (7.0, -3.5), (6.2, -5.65), (4.55, -7.7), (2.6, -9.3), (1.08, -9.93)]
CHIN = (0.0, -10.0)
OUTLINE_N = 200


def face_outline(n=OUTLINE_N, ctrl_r=None, ctrl_l=None, chin=CHIN):
    """Closed face silhouette (counter-clockwise from the top centre, down
    the screen-LEFT side, round the chin and up the right)."""
    r = np.asarray(ctrl_r if ctrl_r is not None else FACE_R, np.float64)
    l = np.asarray(ctrl_l if ctrl_l is not None else [(-x, y) for x, y in FACE_R], np.float64)
    pts = np.vstack([l, [chin], r[::-1][:-1]])
    c = catmull(pts, n=32, closed=True)
    return resample(c, n=n, closed=True)


def face_halfwidth(y, side=+1):
    o = face_outline()
    r = o[o[:, 0] * side >= 0]
    r = r[np.argsort(r[:, 1])]
    return np.interp(y, r[:, 1], np.abs(r[:, 0]))


# eyes
# (upper lid, lower lid), inner corner -> outer corner, eye units
EYE_SHAPES = {
    "open": ([(-0.50, 0.00), (-0.41, 0.105), (-0.23, 0.197), (0.02, 0.232), (0.26, 0.222), (0.42, 0.172), (0.52, 0.092)],
             [(-0.50, 0.00), (-0.41, -0.085), (-0.21, -0.172), (0.04, -0.203), (0.28, -0.168), (0.44, -0.07), (0.52, 0.092)]),
    "half": ([(-0.50, -0.005), (-0.41, 0.035), (-0.23, 0.075), (0.02, 0.092), (0.26, 0.088), (0.42, 0.07), (0.52, 0.078)],
             [(-0.50, -0.005), (-0.41, -0.08), (-0.21, -0.16), (0.04, -0.19), (0.28, -0.158), (0.44, -0.066), (0.52, 0.078)]),
    "closed": ([(-0.49, -0.045), (-0.39, -0.098), (-0.19, -0.142), (0.05, -0.152), (0.29, -0.125), (0.44, -0.065), (0.53, 0.02)],
               [(-0.49, -0.045), (-0.39, -0.098), (-0.19, -0.142), (0.05, -0.152), (0.29, -0.125), (0.44, -0.065), (0.53, 0.02)]),
    "wide": ([(-0.50, 0.00), (-0.41, 0.15), (-0.23, 0.27), (0.02, 0.312), (0.26, 0.298), (0.42, 0.228), (0.53, 0.10)],
             [(-0.50, 0.00), (-0.41, -0.105), (-0.21, -0.205), (0.04, -0.24), (0.28, -0.20), (0.44, -0.085), (0.53, 0.10)]),
    "smile": ([(-0.50, 0.00), (-0.41, 0.095), (-0.23, 0.18), (0.02, 0.212), (0.26, 0.204), (0.42, 0.16), (0.52, 0.09)],
              [(-0.50, 0.00), (-0.41, -0.03), (-0.21, -0.06), (0.04, -0.066), (0.28, -0.045), (0.44, 0.012), (0.52, 0.09)]),
    "smile_closed": ([(-0.51, -0.085), (-0.40, -0.005), (-0.20, 0.052), (0.04, 0.070), (0.28, 0.052), (0.44, 0.002), (0.53, -0.05)],
                     [(-0.51, -0.085), (-0.40, -0.005), (-0.20, 0.052), (0.04, 0.070), (0.28, 0.052), (0.44, 0.002), (0.53, -0.05)]),
    "flat": ([(-0.50, 0.00), (-0.41, 0.07), (-0.23, 0.108), (0.02, 0.122), (0.26, 0.12), (0.42, 0.105), (0.52, 0.088)],
             [(-0.50, 0.00), (-0.41, -0.075), (-0.21, -0.155), (0.04, -0.185), (0.28, -0.152), (0.44, -0.062), (0.52, 0.088)]),
}
IRIS = dict(cu=-0.025, cw=0.085, ru=0.226, rw=0.276)     # centre + radii, eye units


def eye_to_model(side, uw):
    uw = np.asarray(uw, np.float64)
    cx, cy = EYE_C[side]
    return np.stack([cx + side * uw[..., 0] * EW, cy + uw[..., 1] * EW], axis=-1)


def _curve(ctrl, n):
    return resample(catmull(ctrl, n=40), n=n)


def lid_uw(state, which):
    return _curve(EYE_SHAPES[state][which], LID_N)


def blend_state(states):
    """{state: weight} -> (upper, lower) eye-unit curves (weights sum 1)."""
    up = sum(w * lid_uw(s, 0) for s, w in states.items())
    lo = sum(w * lid_uw(s, 1) for s, w in states.items())
    return up, lo


def lids(side, state="open"):
    if isinstance(state, dict):
        up, lo = blend_state(state)
    else:
        up, lo = lid_uw(state, 0), lid_uw(state, 1)
    return eye_to_model(side, up), eye_to_model(side, lo)


def iris_centre(side, look=(0.0, 0.0)):
    u = IRIS["cu"] + 0.155 * look[0] * side
    w = IRIS["cw"] + 0.075 * look[1]
    return eye_to_model(side, np.array([u, w]))


def lash_thickness(t):
    """Thickness (model units) of the upper lash mass along the lid,
    t 0 inner corner .. 1 outer corner: thin at the inner third, heavy on
    the outer third."""
    return 0.11 + 0.17 * smooth(0.0, 0.45, t) + 0.16 * smooth(0.45, 0.88, t) - 0.04 * smooth(0.92, 1.0, t)


def normals_up(curve, side):
    """Unit normals of an inner->outer lid curve pointing UP (away from the
    eye opening)."""
    d = np.gradient(curve, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
    n = np.stack([-d[:, 1], d[:, 0]], axis=1) * side
    return n


def wing_dir(side, up_curve):
    """Direction of the winged flick past the outer corner: along the lid's
    end tangent, turned up and out."""
    t = up_curve[-1] - up_curve[-6]
    t /= np.linalg.norm(t)
    ang = math.radians(34.0) * side
    c, s = math.cos(ang), math.sin(ang)
    return np.array([c * t[0] - s * t[1], s * t[0] + c * t[1]])


# brows (centre line inner -> outer, eye units relative to the eye centre)

BROW_SHAPES = {
    "rest": [(-0.62, 0.575), (-0.36, 0.622), (-0.06, 0.66), (0.24, 0.688), (0.43, 0.664), (0.60, 0.566)],
    "up": [(-0.62, 0.745), (-0.36, 0.80), (-0.06, 0.845), (0.24, 0.872), (0.43, 0.846), (0.60, 0.74)],
    "down": [(-0.62, 0.465), (-0.36, 0.505), (-0.06, 0.54), (0.24, 0.565), (0.43, 0.545), (0.60, 0.46)],
    "angry": [(-0.60, 0.375), (-0.36, 0.49), (-0.06, 0.60), (0.24, 0.672), (0.43, 0.676), (0.60, 0.60)],
    "worried": [(-0.62, 0.745), (-0.36, 0.742), (-0.06, 0.705), (0.24, 0.645), (0.43, 0.582), (0.60, 0.47)],
}
BROW_THICK = (0.46, 0.16)       # head, tail (model units)


def brow_spine(side, shape="rest"):
    if isinstance(shape, dict):
        c = sum(w * _curve(BROW_SHAPES[s], BROW_N) for s, w in shape.items())
    else:
        c = _curve(BROW_SHAPES[shape], BROW_N)
    return eye_to_model(side, c)


def brow_width(t):
    """Full thickness along the brow: a squarish head, the break at the
    outer third, a pointed tail."""
    w = lerp(BROW_THICK[0], BROW_THICK[1], smooth(0.0, 1.0, t) ** 0.8)
    w = w * (1.0 - 0.85 * smooth(0.80, 1.0, t) ** 1.3)
    return w


# nose

def nose_marks():
    nx, ny = NOSE_TIP
    tip = np.array([(nx - 0.34, ny + 0.50), (nx + 0.05, ny + 0.03), (nx + 0.44, ny - 0.10), (nx + 0.66, ny - 0.01)])
    shadow = np.array([(nx + 0.02, ny - 0.07), (nx + 0.74, ny - 0.08), (nx + 0.40, ny - 0.48), (nx + 0.05, ny - 0.38)])
    nostril = np.array([(nx - 0.66, ny - 0.18), (nx - 0.48, ny - 0.28), (nx - 0.30, ny - 0.27)])
    return dict(tip=tip, shadow=shadow, nostril=nostril)


# mouth: upper lip U, lower lip L (left corner -> right corner), the teeth
# lower edge T and the tongue top G, for a state
#   open 0..1.4, wide -1 (round) .. +1 (wide), smile -1 (frown) .. +1,
#   smirk -1 (screen-left corner up) .. +1 (screen-right corner up)

MOUTH_HALF = 1.70


def mouth(open=0.0, wide=0.0, smile=0.0, smirk=0.0):
    o = max(open, 0.0)
    oa = min(o, 1.0)
    ob = max(o - 1.0, 0.0)             # past the "A": the shout
    w_p, w_n = max(wide, 0.0), max(-wide, 0.0)
    s_p, s_n = max(smile, 0.0), max(-smile, 0.0)
    r_p, r_n = max(smirk, 0.0), max(-smirk, 0.0)
    mx, my = MOUTH_C
    mx = mx + 0.24 * (r_p - r_n)
    half = MOUTH_HALF * (1.0 + 0.18 * w_p - 0.42 * w_n + 0.12 * s_p - 0.06 * s_n + 0.04 * oa)         + ob * (4.4 + 1.2 * w_p)
    u = np.linspace(-1.0, 1.0, MOUTH_N)
    au = np.abs(u)
    cy_l = 0.04 + 0.34 * s_p - 0.30 * s_n + 0.40 * r_n - 0.04 * r_p + 0.14 * oa * s_p - 0.55 * ob * (0.4 + s_n)
    cy_r = 0.04 + 0.34 * s_p - 0.30 * s_n + 0.40 * r_p - 0.04 * r_n + 0.14 * oa * s_p - 0.55 * ob * (0.4 + s_n)
    corner = np.where(u < 0, cy_l, cy_r)
    bow = (-0.06 - 0.24 * s_p + 0.20 * s_n - 0.05 * w_n) * (1.0 - au ** 2)
    base = corner * au ** 1.6 + bow
    # opening
    h = (2.1 * oa + 8.0 * ob) * (1.0 - 0.30 * w_p + 0.22 * w_n + 0.08 * s_p)
    lift = ((0.12 * oa + 0.42 * ob + 0.40 * w_n * oa) * (1.0 - au ** 2) ** 0.8
            + 0.22 * s_p * oa * (1.0 - au ** 2))
    p = 2.4 + 0.9 * w_p - 0.45 * w_n + 2.8 * ob + 0.6 * s_p
    drop = h * (1.0 - au ** p)
    flare = 0.20 * ob + 0.05 * s_p * oa
    U = np.stack([mx + u * half, my + base + lift], axis=1)
    L = np.stack([mx + u * half * (1.0 + flare * (1.0 - u ** 2)), my + base - drop], axis=1)
    if w_n > 0 and oa > 0:
        pinch = w_n * oa * 0.30 * (1 - au) * au
        U[:, 0] -= np.sign(u) * pinch * half
        L[:, 0] -= np.sign(u) * pinch * half
    gap = U[:, 1] - L[:, 1]
    teeth_h = np.minimum(0.48 + 0.55 * ob, 0.40 * gap) * (1.0 - smooth(0.80, 1.0, au))
    T = U - np.stack([np.zeros_like(u), teeth_h], axis=1)
    tongue_h = np.minimum(0.55 * h * (1.0 - au ** 2) ** 0.8, 0.45 * gap)
    G = L + np.stack([np.zeros_like(u), tongue_h], axis=1)
    return dict(U=U, L=L, T=T, G=G)


# ear (screen-left, the tucked side)

EAR_L = [(-7.30, 0.55), (-7.95, 0.72), (-8.55, 0.25), (-8.72, -0.95), (-8.52, -2.35), (-8.18, -3.55),
         (-7.82, -4.42), (-7.42, -4.62), (-7.10, -4.2)]
EARRING_TOP = (-7.62, -4.52)


def ear_outline():
    c = catmull(EAR_L, n=24)
    back = np.array([(-7.05, -3.6), (-7.0, -1.5), (-7.05, 0.2)])
    return np.vstack([c, catmull(np.vstack([c[-1:], back, c[:1]]), n=12)[1:-1]])


# neck and body

def neck_outline():
    r = [(3.55, -4.0), (3.62, -8.0), (3.72, -11.5), (4.10, -13.6), (5.3, -15.0), (7.5, -16.2)]
    l = [(-x, y) for x, y in r]
    bottom = [(4.0, -18.4), (0.0, -19.2), (-4.0, -18.4)]
    pts = np.vstack([r, bottom, l[::-1]])
    return catmull(pts, n=16, closed=True)
