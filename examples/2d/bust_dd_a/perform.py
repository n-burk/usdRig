"""Kaede's animation: a 6-second performance and a parameter sweep, authored
as smooth cubic Hermite splines through keys and sampled every frame.

Tangents are Catmull-Rom, clamped flat at a key that is a local extremum
(so an overshoot key eases into its settle), or given explicitly. Timing is
offset part by part -- eyes lead, lids and brows next, the head a few frames
later, the neck, body and shoulders after it, the hair last -- every stop
overshoots and settles, holds keep drifting, blinks are irregular and ride
the head turns.

The hair, ribbon tails and the earring are keyed from an offline spring
solve: each FK joint is a damped oscillator (natural frequency in Hz,
damping ratio) that follows the head's rotation with a per-joint stiffness
and is kicked by the lateral acceleration of its root, measured through the
same head/turn transforms the rig applies.
"""

import math
import os

import numpy as np

import art_body
import art_hair
import design as D
import field

FPS = 30.0
PERF_N = 180
SWEEP_N = 150


# curves

def spline(keys, F):
    """Cubic Hermite through keys [(frame, value[, tangent])], sampled at F.
    tangent: omitted -> clamped Catmull-Rom, 'f' -> flat, a number -> that
    slope (units per frame). Constant before the first / after the last key."""
    ks = sorted(keys, key=lambda k: k[0])
    f = np.array([k[0] for k in ks], float)
    v = np.array([k[1] for k in ks], float)
    n = len(ks)
    m = np.zeros(n)
    for i in range(n):
        spec = ks[i][2] if len(ks[i]) > 2 else None
        if spec == "f":
            m[i] = 0.0
        elif isinstance(spec, (int, float)):
            m[i] = float(spec)
        elif 0 < i < n - 1:
            d0 = v[i] - v[i - 1]
            d1 = v[i + 1] - v[i]
            if d0 * d1 <= 0:
                m[i] = 0.0                                    # extremum: ease
            else:
                m[i] = (v[i + 1] - v[i - 1]) / (f[i + 1] - f[i - 1])
                # keep it monotone inside each segment
                lim = 3.0 * min(abs(d0) / (f[i] - f[i - 1]), abs(d1) / (f[i + 1] - f[i]))
                m[i] = math.copysign(min(abs(m[i]), lim), m[i])
    out = np.empty(len(F))
    for j, x in enumerate(F):
        if x <= f[0]:
            out[j] = v[0]
            continue
        if x >= f[-1]:
            out[j] = v[-1]
            continue
        i = int(np.searchsorted(f, x, side="right") - 1)
        h = f[i + 1] - f[i]
        t = (x - f[i]) / h
        h00 = 2 * t ** 3 - 3 * t ** 2 + 1
        h10 = t ** 3 - 2 * t ** 2 + t
        h01 = -2 * t ** 3 + 3 * t ** 2
        h11 = t ** 3 - t ** 2
        out[j] = h00 * v[i] + h10 * h * m[i] + h01 * v[i + 1] + h11 * h * m[i + 1]
    return out


def delay(a, frames):
    """a delayed by a (fractional) number of frames, held at the start."""
    F = np.arange(len(a), dtype=float)
    return np.interp(F - frames, F, a)


def drift(F, amp, seed, periods=(2.3, 3.7, 5.3)):
    """A slow, never-repeating moving-hold wander (sum of three sines)."""
    rng = np.random.default_rng(seed)
    out = np.zeros(len(F))
    for k, p in enumerate(periods):
        ph = rng.uniform(0, 2 * math.pi)
        out += math.sin(0.0) + np.sin(2 * math.pi * F / (p * FPS) + ph) / (k + 1.2)
    return out * amp / 1.3


def blink(F, start, close=2, hold=1, open_=4, depth=1.0, half=0.52):
    """Eye.ty contribution of one blink: a fast close through the half-lid
    in-between, a hold, a slower open."""
    return spline([(start, 0.0, "f"), (start + close * 0.5, -half * depth), (start + close, -depth, "f"),
                   (start + close + hold, -depth, "f"), (start + close + hold + open_ * 0.45, -depth * 0.42),
                   (start + close + hold + open_, 0.0, "f")], F)


def pulse_train(F, keys):
    return spline(keys, F)


# the performance: cool -> notice -> brow flick & smirk -> provoked ->
# "HAAH?!" take -> laugh -> composed wink

def performance():
    F = np.arange(1, PERF_N + 1, dtype=float)
    A = {}
    S = lambda keys: spline(keys, F)  # noqa: E731

    turn = S([(1, -11.0), (16, -12.0), (27, -11.4), (31, -12.6), (35, -6.0), (38, 1.2), (40, 3.4), (45, 0.8),
              (52, 1.4), (56, 3.2), (61, 4.0), (67, 5.8), (72, 5.0), (80, 5.4), (86, 6.6), (90, 3.0),
              (93, 0.6), (95, -2.6), (97, -3.6), (101, -2.0), (114, -2.4), (118, 0.5), (121, 6.8), (124, 6.0),
              (146, 4.2), (150, 3.8), (153, -0.6), (156, -1.3), (161, 0.2), (165, 0.4), (170, 3.4), (173, 2.8),
              (180, 2.6)])
    nod = S([(1, 2.2), (27, 2.8), (31, 3.4), (34, 4.6), (39, 0.2), (42, -1.2), (47, 0.6), (52, 0.2),
             (55, -4.6), (58, -5.8), (63, -4.0), (68, -3.4), (72, -2.4), (80, -2.2), (86, -1.2),
             (90, 4.8), (93, 6.4), (95, -6.0), (97, -8.6), (102, -6.4), (114, -6.0), (118, -4.4),
             (121, -10.0), (125, -4.4), (129, -9.2), (133, -4.8), (137, -8.2), (141, -5.4), (145, -7.2),
             (150, -6.2),
             (153, 1.6), (156, 0.2), (161, 0.7), (165, 0.5), (170, -2.8), (174, -2.2), (180, -2.0)])
    tilt = S([(1, 4.0), (27, 4.6), (33, 3.4), (38, 0.2), (42, -2.6), (47, -1.0), (52, -1.2),
              (55, 3.0), (58, 7.2), (63, 6.2), (67, -5.6), (70, -6.4), (74, -4.4), (80, -4.0), (86, -3.2),
              (90, -0.4), (93, 1.2), (95, -2.6), (98, -3.4), (102, -1.8), (114, -2.0), (118, 1.0),
              (121, 8.6), (125, 4.8), (129, 8.0), (133, 5.2), (137, 7.4), (141, 5.6), (145, 6.8), (150, 6.2),
              (153, 1.2), (157, -1.2), (162, 0.0), (165, 0.2), (170, -6.2), (174, -5.0), (180, -5.0)])
    # moving holds: the head never freezes
    turn += drift(F, 0.35, 1)
    nod += drift(F, 0.30, 2)
    tilt += drift(F, 0.35, 3)
    # the tremble inside the held take (2.5-frame period, decaying)
    env = np.clip((F - 99) / 2.0, 0, 1) * np.clip((115 - F) / 6.0, 0, 1)
    tilt += 0.9 * env * np.sin(2 * math.pi * (F - 99) / 2.6)
    turn += 0.55 * env * np.sin(2 * math.pi * (F - 99) / 2.6 + 1.3)
    A["FaceAngle.ry"] = turn
    A["FaceAngle.rx"] = nod
    A["Head.rz"] = tilt
    A["Neck.rz"] = 0.35 * delay(tilt, 2.0)
    A["Body.rz"] = 0.22 * delay(tilt, 4.0) + drift(F, 0.25, 4)
    A["Body.tx"] = S([(1, 0.0), (40, 0.25), (86, 0.3), (93, 0.1), (97, -0.35), (110, -0.25), (121, 0.4),
                      (150, 0.3), (160, 0.0), (180, 0.05)]) + drift(F, 0.08, 5)
    A["Torso.ry"] = 0.30 * delay(turn, 4.0)

    # laughing pumps, a settle -----------------------------------------------------
    A["Torso.ty"] = S([(1, 0.35), (18, 0.9), (44, 0.1), (62, 0.85), (80, 0.2), (86, 0.25), (93, 1.05),
                       (96, 0.9), (112, 0.5), (118, 0.35), (121, 0.9), (125, 0.35), (129, 0.85), (133, 0.4),
                       (137, 0.75), (141, 0.45), (145, 0.65), (150, 0.3), (160, 0.9), (180, 0.3)])
    A["Shoulder_R.ty"] = S([(1, 0.0), (86, 0.05), (92, 0.62), (94, 0.66), (96, 0.0), (98, -0.08), (102, 0.08),
                            (116, 0.05), (121, 0.36), (125, 0.08), (129, 0.32), (133, 0.10), (137, 0.24),
                            (148, 0.05), (166, 0.0), (171, 0.22), (180, 0.14)])
    A["Shoulder_L.ty"] = delay(A["Shoulder_R.ty"], 1.5) * 0.85

    A["Look.tx"] = S([(1, -0.62), (10, -0.62, "f"), (12, -0.48), (13, -0.50, "f"), (20, -0.52), (28, -0.58, "f"),
                      (30, -0.58, "f"), (32, 0.14), (34, 0.05, "f"), (50, 0.02), (60, 0.08), (74, 0.06, "f"),
                      (76, 0.06, "f"), (78, 0.74), (80, 0.70, "f"), (85, 0.68, "f"), (87, 0.02), (89, 0.0, "f"),
                      (150, 0.0, "f"), (155, 0.0, "f"), (157, -0.30), (158, -0.28, "f"), (161, -0.28, "f"),
                      (163, 0.06), (165, 0.04, "f"), (180, 0.04)])
    A["Look.ty"] = S([(1, -0.18), (30, -0.18, "f"), (32, 0.04), (34, 0.0, "f"), (55, -0.12), (68, -0.18),
                      (76, -0.1, "f"), (78, 0.08), (85, 0.05), (89, 0.0), (157, -0.1), (163, 0.0), (180, 0.0)])
    A["Look.tz"] = S([(1, 0.0), (93, 0.0, "f"), (95, 0.85), (97, 0.72), (114, 0.72, "f"), (119, 0.0, "f")])
    # micro-saccades inside the holds
    sac = np.zeros(len(F))
    for f0, dx in ((44, 0.03), (63, -0.035), (104, 0.04), (109, -0.04), (172, -0.03)):
        sac += dx * np.clip((F - f0) / 2.0, 0, 1) * np.clip((f0 + 7 - F) / 2.0, 0, 1)
    A["Look.tx"] += sac

    open_R = S([(1, 0.0), (30, 0.0, "f"), (33, 0.22), (37, 0.26), (40, 0.34), (44, 0.12), (50, 0.0), (86, 0.0),
                (90, -0.30), (93, -0.34), (95, 1.05), (96, 1.12), (100, 1.0), (114, 1.0), (118, 0.0, "f")])
    open_L = delay(open_R, 0.6)
    blinks_R = blink(F, 17, open_=4) + blink(F, 33, open_=4) + blink(F, 76, open_=5, depth=0.95) + \
        blink(F, 151, open_=5)
    blinks_L = blink(F, 18, open_=4) + blink(F, 33.5, open_=4) + blink(F, 76.5, open_=5, depth=0.95) + \
        blink(F, 151.5, open_=5)
    # the wink: the screen-right eye closes on a smile curve, the other squints
    wink = S([(1, 0.0), (165, 0.0, "f"), (167, -1.0), (172, -1.0, "f"), (176, -0.2), (178, 0.0, "f")])
    blinks_R += wink
    A["Eye_R.ty"] = open_R + blinks_R
    A["Eye_L.ty"] = open_L + blinks_L
    # cool lids (flat, 0..20): dropped as she notices, back for the smirk
    flat = S([(1, 13.0), (30, 13.0, "f"), (35, 0.0, "f"), (54, 0.0), (60, 9.0), (70, 11.0), (80, 8.0),
              (86, 3.0), (93, 0.0, "f"), (150, 0.0, "f"), (156, 5.0), (180, 7.0)])
    closure_R = np.clip(-blinks_R, 0, 1)
    closure_L = np.clip(-blinks_L, 0, 1)
    A["Eye_R.rx"] = flat * (1 - closure_R) * 0.9
    A["Eye_L.rx"] = (flat + S([(1, 0.0), (54, 0.0), (60, 4.0), (75, 3.0), (86, 0.0)])) * (1 - closure_L)
    # squint (lower lid up)
    A["Eye_R.tx"] = S([(1, 0.08), (30, 0.08), (35, 0.0), (86, 0.0), (91, 0.75), (93, 0.8), (95, 0.0, "f"),
                       (118, 0.0), (122, 0.45), (148, 0.4), (153, 0.0, "f"), (180, 0.05)])
    A["Eye_L.tx"] = S([(1, 0.1), (30, 0.1), (35, 0.0), (54, 0.0), (60, 0.34), (75, 0.28), (86, 0.1),
                       (91, 0.8), (93, 0.82), (95, 0.0, "f"), (118, 0.0), (123, 0.5), (148, 0.45),
                       (153, 0.0, "f"), (165, 0.0), (169, 0.38), (176, 0.3), (180, 0.25)])
    # laughing ^^ eyes (0..20); a touch of smile curve on the wink
    smile = S([(1, 0.0), (118, 0.0, "f"), (121, 20.0), (148, 20.0, "f"), (152, 0.0, "f")])
    A["Eye_R.rz"] = smile + S([(1, 0.0), (165, 0.0, "f"), (168, 9.0), (173, 9.0), (177, 0.0, "f")])
    A["Eye_L.rz"] = delay(smile, 1.0)

    A["Brow_R.ty"] = S([(1, -0.15), (30, -0.12, "f"), (34, 0.28), (40, 0.12), (50, 0.1, "f"), (53, 1.02),
                        (55, 0.84), (60, 0.82), (72, 0.6), (80, 0.45), (86, 0.3), (91, -0.62), (93, -0.66),
                        (95, -0.25), (97, -0.45), (114, -0.4), (118, 0.2), (121, 0.55), (148, 0.45),
                        (153, 0.1), (164, 0.05), (168, -0.25), (174, -0.2), (180, -0.18)])
    A["Brow_L.ty"] = S([(1, 0.05), (31, 0.05, "f"), (35, 0.36), (41, 0.14), (50, 0.12, "f"), (54, -0.42),
                        (58, -0.35), (75, -0.3), (86, -0.1), (91, -0.6), (93, -0.64), (95, -0.2), (97, -0.42),
                        (114, -0.38), (118, 0.25), (122, 0.6), (148, 0.5), (153, 0.12), (164, 0.1), (168, 0.46),
                        (174, 0.4), (180, 0.38)])
    A["Brow_R.rz"] = S([(1, -3.0), (50, -2.0), (53, 5.0), (60, 4.0), (80, 2.0), (86, 0.0), (91, -14.0),
                        (93, -16.0), (95, -20.0), (97, -18.5), (114, -18.0), (118, 0.0), (121, 9.0), (148, 8.0),
                        (153, 0.0), (180, -1.0)])
    A["Brow_L.rz"] = S([(1, 0.0), (50, 0.0), (54, -10.0), (60, -9.0), (80, -6.0), (86, -2.0), (91, -15.0),
                        (93, -17.0), (95, -20.0), (97, -19.0), (114, -18.0), (118, 0.0), (122, 10.0),
                        (148, 9.0), (153, 0.0), (180, 2.0)])
    for k in ("Brow_R.ty", "Brow_L.ty"):
        A[k] += drift(F, 0.03, sum(map(ord, k)))

    # vowel pad (rx open "A", ry +wide "I" / -round "U"), smile dial, shift,
    # yell (-ty) / pout (+ty), and the corners
    A["Mouth.rx"] = S([(1, 0.0), (36, 0.0), (40, 7.0), (44, 5.0), (47, 0.0, "f"), (52, 0.0, "f"), (54, 25.0), (57, 17.0),
                       (59, 12.0), (61, 26.0), (63, 20.0), (65, 0.0, "f"),
                       (94, 0.0, "f"), (96, 10.0), (103, 8.0), (105, 2.0), (108, 12.0), (111, 6.0), (113, 10.0),
                       (116, 6.0), (118, 16.0),
                       (121, 28.0), (124, 15.0), (126, 26.0), (129, 13.0), (131, 23.0), (134, 11.0),
                       (136, 19.0), (139, 9.0), (141, 15.0), (144, 6.0), (148, 4.0), (152, 0.0, "f")])
    A["Mouth.ry"] = S([(1, 0.0), (52, 0.0), (54, 4.0), (59, 0.0), (61, -6.0), (65, 0.0), (103, 0.0),
                       (105, 16.0), (107, 0.0), (109, -14.0), (111, -10.0), (113, 4.0), (118, 0.0),
                       (121, 6.0), (148, 6.0), (152, 0.0)])
    A["Mouth.rz"] = S([(1, -3.0), (33, -3.0), (40, 0.0), (52, 0.0), (55, -6.0), (62, -3.0), (68, 4.0),
                       (78, 4.0), (82, -4.0), (86, -3.0), (91, -10.0), (93, -10.0), (95, 0.0), (118, 0.0),
                       (121, 20.0), (148, 19.0), (153, 3.0), (180, 3.0)])
    A["Mouth.tx"] = S([(1, 0.0), (55, 0.2), (62, 0.25), (68, 0.45), (78, 0.4), (84, 0.05), (93, 0.0),
                       (150, 0.0), (158, 0.25), (170, 0.35), (180, 0.3)])
    A["Mouth.ty"] = S([(1, 0.18), (30, 0.18), (36, 0.0), (86, 0.0), (91, 0.36), (93, 0.4), (94, 0.0),
                       (95, -1.0), (96, -1.08), (99, -0.95), (103, -1.0), (105, -0.55), (107, -0.6),
                       (109, -0.78), (111, -0.72), (113, -0.96), (116, -0.85), (118, -0.5), (121, -0.34),
                       (124, -0.12), (126, -0.30), (129, -0.10), (131, -0.26), (134, -0.08), (136, -0.20),
                       (139, -0.05), (141, -0.14), (144, -0.03), (148, 0.0, "f"), (180, 0.0)])
    A["Corner_R.ty"] = S([(1, 0.0), (62, 0.0, "f"), (66, 1.0, "f"), (69, 0.92), (78, 0.86), (83, 0.1), (86, 0.0),
                          (118, 0.0), (121, 0.5), (148, 0.45), (153, 0.4), (160, 0.55), (168, 0.9), (171, 0.8),
                          (180, 0.78)])
    A["Corner_L.ty"] = S([(1, 0.0), (118, 0.0), (121, 0.45), (148, 0.4), (153, 0.05), (180, 0.08)])

    A["Fx.tx"] = S([(1, 0.0), (121, 0.0, "f"), (125, 1.1), (128, 1.0), (150, 0.9), (178, 0.35), (180, 0.33)])
    A["Fx.ty"] = S([(1, 0.0), (95, 0.0, "f"), (97, 1.18), (99, 1.0), (116, 1.0), (119, 0.0, "f")])
    A["Fx.tz"] = S([(1, 0.0), (98, 0.0, "f"), (100, 0.5), (116, 1.0, 0.0), (118, 0.0, "f")])
    return F, A


# the parameter sweep (rig reveal): every deformer family in 5 seconds

def sweep():
    F = np.arange(1, SWEEP_N + 1, dtype=float)
    A = {}
    S = lambda keys: spline(keys, F)  # noqa: E731
    A["FaceAngle.ry"] = S([(1, 0.0), (8, 0.0, "f"), (22, 30.0, "f"), (28, 30.0, "f"), (46, -30.0, "f"),
                           (52, -30.0, "f"), (62, 0.0, "f"), (150, 0.0)])
    A["FaceAngle.rx"] = S([(1, 0.0), (62, 0.0, "f"), (72, -15.0, "f"), (76, -15.0, "f"), (86, 14.0, "f"),
                           (90, 14.0, "f"), (98, 0.0, "f"), (150, 0.0)])
    A["Head.rz"] = S([(1, 0.0), (8, 0.0, "f"), (22, -6.0), (28, -6.0), (46, 7.0), (52, 7.0), (62, 0.0),
                      (72, 5.0), (86, -5.0), (98, 0.0), (150, 0.0)])
    A["Neck.rz"] = 0.35 * delay(A["Head.rz"], 2)
    A["Body.rz"] = 0.2 * delay(A["Head.rz"], 4)
    A["Body.tx"] = np.zeros(len(F))
    A["Torso.ry"] = 0.3 * delay(A["FaceAngle.ry"], 4)
    A["Torso.ty"] = 0.5 - 0.5 * np.cos(2 * math.pi * (F - 1) / 55.0)
    A["Shoulder_R.ty"] = S([(1, 0.0), (112, 0.0), (116, 0.5), (122, 0.0), (150, 0.0)])
    A["Shoulder_L.ty"] = delay(A["Shoulder_R.ty"], 1.5)
    A["Look.tx"] = S([(1, 0.0), (8, 0.0, "f"), (10, 0.8), (26, 0.7), (30, 0.7, "f"), (32, -0.8), (50, -0.7),
                      (54, -0.7, "f"), (57, 0.0), (98, 0.0), (100, 1.0), (103, 1.0, "f"), (105, -1.0),
                      (108, -1.0, "f"), (110, 0.0), (150, 0.0)])
    A["Look.ty"] = S([(1, 0.0), (62, 0.0, "f"), (64, -0.5), (76, -0.5), (78, 0.7), (90, 0.6), (93, 0.0),
                      (150, 0.0)])
    A["Look.tz"] = S([(1, 0.0), (116, 0.0, "f"), (118, 0.85), (128, 0.85, "f"), (131, 0.0, "f"), (150, 0.0)])
    b = blink(F, 9) + blink(F, 31) + blink(F, 94, open_=5) + blink(F, 144)
    wide = S([(1, 0.0), (116, 0.0, "f"), (118, 1.1), (128, 1.0), (131, 0.0, "f")])
    A["Eye_R.ty"] = b + wide
    A["Eye_L.ty"] = delay(b, 0.5) + wide
    A["Eye_R.rx"] = np.zeros(len(F))
    A["Eye_L.rx"] = np.zeros(len(F))
    A["Eye_R.tx"] = np.zeros(len(F))
    A["Eye_L.tx"] = np.zeros(len(F))
    smile = S([(1, 0.0), (131, 0.0, "f"), (134, 20.0), (142, 20.0, "f"), (145, 0.0)])
    A["Eye_R.rz"] = smile
    A["Eye_L.rz"] = delay(smile, 0.5)
    A["Brow_R.ty"] = S([(1, 0.0), (62, 0.0), (72, 0.8), (86, -0.5), (98, 0.0), (116, 0.0), (118, 0.3),
                        (131, 0.3), (134, 0.5), (145, 0.0), (150, 0.0)])
    A["Brow_L.ty"] = delay(A["Brow_R.ty"], 1)
    A["Brow_R.rz"] = S([(1, 0.0), (116, 0.0, "f"), (118, -20.0), (130, -20.0), (134, 10.0), (145, 0.0)])
    A["Brow_L.rz"] = delay(A["Brow_R.rz"], 1)
    pad = dict(rest=(0, 0), A=(30, 0), I=(0, 30), U=(0, -30), E=(21, 21), O=(21, -21))
    talk = [(1, "rest"), (100, "rest"), (103, "A"), (106, "I"), (109, "U"), (112, "E"), (115, "O"),
            (117, "rest"), (131, "rest"), (134, "A"), (142, "A"), (145, "rest"), (150, "rest")]
    A["Mouth.rx"] = S([(f, pad[v][0]) for f, v in talk])
    A["Mouth.ry"] = S([(f, pad[v][1]) for f, v in talk])
    A["Mouth.rz"] = S([(1, 0.0), (131, 0.0), (134, 20.0), (142, 20.0), (145, 0.0)])
    A["Mouth.tx"] = np.zeros(len(F))
    A["Mouth.ty"] = S([(1, 0.0), (116, 0.0, "f"), (118, -1.0), (128, -1.0, "f"), (131, 0.0, "f"), (150, 0.0)])
    A["Corner_R.ty"] = S([(1, 0.0), (52, 0.0), (58, 0.9), (64, 0.0), (131, 0.0), (134, 0.4), (145, 0.0)])
    A["Corner_L.ty"] = S([(1, 0.0), (131, 0.0), (134, 0.4), (145, 0.0)])
    A["Fx.tx"] = S([(1, 0.0), (132, 0.0, "f"), (135, 1.0), (145, 1.0), (150, 0.6)])
    A["Fx.ty"] = S([(1, 0.0), (117, 0.0, "f"), (119, 1.0), (128, 1.0, "f"), (130, 0.0, "f")])
    A["Fx.tz"] = S([(1, 0.0), (118, 0.0, "f"), (120, 0.5), (129, 1.0, 0.0), (131, 0.0, "f")])
    return F, A


# the spring solve for hair, ribbon tails and the earring

CHAIN_KIND = {
    "fringe": dict(f=2.2, zeta=0.35, kick=0.55, stiff=[0.97, 0.85, 0.72]),
    "lock": dict(f=1.3, zeta=0.25, kick=0.85, stiff=[0.92, 0.72, 0.52, 0.38]),
    "back": dict(f=0.95, zeta=0.30, kick=0.9, stiff=[0.88, 0.62, 0.46, 0.32]),
    "stray": dict(f=3.0, zeta=0.20, kick=0.45, stiff=[1.0, 0.92]),
    "ribbon": dict(f=1.5, zeta=0.30, kick=0.9, stiff=[0.9, 0.7, 0.55]),
    "earring": dict(f=1.8, zeta=0.12, kick=0.22, stiff=[0.25, 0.2]),
}
KIND_FIELD = {"fringe": "front_hair", "cap": "front_hair", "stray": "front_hair", "lock": "side_hair",
              "back": "back_hair"}


def rot(p, c, deg):
    a = math.radians(deg)
    d = p - c
    return c + np.array([d[0] * math.cos(a) - d[1] * math.sin(a), d[0] * math.sin(a) + d[1] * math.cos(a)])


def root_track(root, kind, A, head=True):
    """Screen x of a chain root per frame through the same transforms the rig
    applies: the head field (turn/nod), the head/neck/body rotations, sway."""
    n = len(A["Head.rz"])
    xs = np.zeros(n)
    ys = np.zeros(n)
    for i in range(n):
        p = np.array(root, float)
        if head:
            d = field.head_field(p[None], KIND_FIELD.get(kind, "front_hair"),
                                 A["FaceAngle.ry"][i], -A["FaceAngle.rx"][i])[0]
            p = p + d
            p = rot(p, np.array(D.HEAD_PIVOT), A["Head.rz"][i])
            p = rot(p, np.array(D.NECK_PIVOT), A["Neck.rz"][i])
        else:
            p = p + np.array([0.13 * A["Torso.ry"][i], 0.0])
        p = rot(p, np.array(D.BODY_PIVOT), A["Body.rz"][i])
        p[0] += A["Body.tx"][i]
        xs[i], ys[i] = p
    return xs, ys


def spring_chain(A, root, kind, njoint, length, seed, head=True, pre=45, sub=4):
    prm = CHAIN_KIND[kind]
    stiff = np.array((prm["stiff"] + [prm["stiff"][-1]] * 6)[:njoint])
    w = 2 * math.pi * prm["f"]
    z = prm["zeta"]
    R = (A["Body.rz"] + A["Neck.rz"] + A["Head.rz"]) if head else A["Body.rz"].copy()
    xs, ys = root_track(root, kind, A, head)
    # pre-roll: hold the first pose so the chain starts settled
    R = np.concatenate([np.full(pre, R[0]), R])
    xs = np.concatenate([np.full(pre, xs[0]), xs])
    ys = np.concatenate([np.full(pre, ys[0]), ys])
    acc = np.gradient(np.gradient(xs)) * FPS * FPS          # mu / s^2
    accy = np.gradient(np.gradient(ys)) * FPS * FPS
    rng = np.random.default_rng(seed)
    per = rng.uniform(70, 130)
    ph = rng.uniform(0, 2 * math.pi)
    amp = 0.9 if kind != "earring" else 0.3
    lever = np.linspace(0.6, 1.4, njoint)
    phi = np.full(njoint, 0.0) + stiff * R[0]
    vel = np.zeros(njoint)
    out = np.zeros((len(R), njoint))
    dt = 1.0 / FPS / sub
    for i in range(len(R)):
        breeze = amp * math.sin(2 * math.pi * i / per + ph) * np.linspace(0.3, 1.0, njoint)
        target = stiff * R[i] + breeze
        kick = -prm["kick"] * (acc[i] + 0.35 * accy[i] * np.sign(root[0] + 1e-6)) / max(length, 1.0) * \
            (180 / math.pi) * lever
        for _ in range(sub):
            a = w * w * (target - phi) - 2 * z * w * vel + kick
            vel = vel + a * dt
            phi = phi + vel * dt
        out[i] = phi
    out = out[pre:]
    local = np.zeros_like(out)
    base = (A["Body.rz"] + A["Neck.rz"] + A["Head.rz"]) if head else A["Body.rz"]
    local[:, 0] = out[:, 0] - base
    for j in range(1, njoint):
        local[:, j] = out[:, j] - out[:, j - 1]
    return local


def secondary(builder, A):
    """{control path: rz values} for every hair, ribbon and earring chain."""
    from art import sample_at
    out = {}
    k = 0
    for c in art_hair.clumps():
        name = "Hair_" + c.name
        if name not in builder.chain_controls:
            continue
        pts = builder.chain_pts[name]
        length = float(np.linalg.norm(np.diff(c.spine, axis=0), axis=1).sum())
        kind = c.kind if c.kind in CHAIN_KIND else "fringe"
        local = spring_chain(A, pts[0], kind, len(pts), length, seed=11 + k)
        for j, cp in enumerate(builder.chain_controls[name]):
            out[cp] = local[:, j]
        k += 1
    for tname in ("TailL", "TailR"):
        name = "Ribbon_" + tname
        pts = builder.chain_pts[name]
        local = spring_chain(A, pts[0], "ribbon", len(pts), 6.0, seed=71 + k, head=False)
        for j, cp in enumerate(builder.chain_controls[name]):
            out[cp] = local[:, j]
        k += 1
    pts = builder.chain_pts["Earring"]
    local = spring_chain(A, pts[0], "earring", len(pts), 3.0, seed=5)
    for j, cp in enumerate(builder.chain_controls["Earring"]):
        out[cp] = local[:, j]
    return out


# writing the layers

def write_anim(builder, path, rig_layer_name, script, n, doc):
    from pxr import Sdf, Usd
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path)
    layer.subLayerPaths = ["./" + rig_layer_name]
    layer.startTimeCode = 1
    layer.endTimeCode = n
    layer.timeCodesPerSecond = FPS
    layer.framesPerSecond = FPS
    layer.defaultPrim = "Kaede"
    layer.documentation = doc
    stage = Usd.Stage.Open(layer)
    F, A = script()
    H = builder.head_ctl
    B = builder.body_ctl
    ctl = dict(builder.avar_ctl)
    ctl.update({"Head": H, "Neck": builder.neck_ctl, "Body": B})

    def key(prim_path, channel, values):
        prim = stage.OverridePrim(prim_path)
        a = prim.CreateAttribute("avars:" + channel, Sdf.ValueTypeNames.Double, False)
        for f, v in zip(F, values):
            a.Set(float(round(float(v), 4)), Usd.TimeCode(float(f)))

    for k, values in sorted(A.items()):
        c, ch = k.split(".")
        key(ctl[c], ch, values)
    for cp, values in sorted(secondary(builder, A).items()):
        key(cp, "rz", values)
    layer.Save()
    return A


def write_all(builder, here, name):
    rig = name + "_rig.usda"
    write_anim(builder, os.path.join(here, name + "_anim.usda"), rig, performance, PERF_N, doc=(
        "Kaede, a 6-second performance for %s: a cool side-glance, she notices the "
        "camera, flicks a brow and smirks, gets provoked into a 'HAAH?!' take, bursts "
        "out laughing and composes herself with a wink. Only control avars are "
        "authored here (the hair, ribbon and earring chains from an offline spring "
        "solve). Generated by build_bust.py." % rig))
    write_anim(builder, os.path.join(here, name + "_sweep.usda"), rig, sweep, SWEEP_N, doc=(
        "A 150-frame parameter sweep for %s (the rig reveal): angle X +-30, angle Y "
        "+-15, the tilt, eyes, brows, the five vowels, the yell take and the laugh. "
        "Only control avars are authored here. Generated by build_bust.py." % rig))
    print("wrote %s_anim.usda and %s_sweep.usda" % (name, name))
