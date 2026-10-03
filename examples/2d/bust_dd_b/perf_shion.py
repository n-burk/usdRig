"""Shion's 6-second performance (180 frames at 30 fps).

Beats (frame numbers are the rig's time codes):

    1-24     cool idle: head turned off to screen left, eyes further left,
             flat lids, a moving hold (breath, drift, one micro-saccade)
    24-44    the eyes snap to camera first (3 f), a blink rides the head
             turn, the head follows with an arc (chin dips mid-turn), a
             small overshoot and settle; neck, body and hair follow late
    38-52    the attitude: her left brow flicks up (and overshoots), then
             a one-sided smirk lands while the head cocks toward it
    54-84    a dry line, "Took you long enough.": 6 syllables, varied jaw,
             m/b/p-free so the mouth only closes at word ends; head accents
             on the stressed syllables, a glance aside mid-line
    84-96    realisation: something off-camera right; eyes first, then a
             slow drift of the head -- near stillness
    96-100   anticipation: chin tucks, eyes squeeze, shoulders rise
    100-103  THE TAKE (the fastest move in the piece): head snaps back,
             eyes go wide with pinpoint irises, brows shoot up, the mouth
             breaks into a shout; sweat, hatched blush; the hair whips
    103-122  the held take with a decaying tremble
    122-125  mode switch: she snaps into a laugh at herself (^^ eyes, grin)
    125-150  laugh bounces every ~5 f, decaying
    150-180  settle: exhale, eyes open and drift back off to the left,
             a blink on the return turn, composure (loops into frame 1)
"""

import math

import numpy as np

from perf import blink, drift, spline

LENGTH = 180


def performance():
    F = np.arange(1, LENGTH + 1, dtype=float)
    A = {}

    def S(keys):
        return spline(keys, F)

    A["FaceAngle.ry"] = S([
        (1, -14.0), (12, -13.6), (22, -14.2),                   # moving hold
        (25, -15.0),                                            # anticipation
        (29, -8.5), (33, 0.5), (36, 2.6), (39, 2.0), (43, 1.2),  # turn, overshoot, settle
        (48, 0.6), (53, 1.4),                                   # cock with the smirk
        (60, 1.0), (66, 2.2), (72, 0.8), (79, 1.6), (84, 1.2),  # talking drift
        (90, 3.4), (96, 6.0),                                   # the slow drift toward it
        (99, 5.2),
        (101, 2.0), (103, 1.0, "flat"),                         # the snap
        (112, 1.6), (121, 1.2),
        (124, -5.5), (127, -7.5), (131, -6.4),                  # laugh: turns a little away
        (140, -7.8), (150, -6.2),
        (156, -5.0), (160, -9.5), (166, -13.8), (170, -14.6), (175, -14.1), (180, -14.0)])
    A["FaceAngle.rx"] = S([
        (1, 3.0), (14, 2.6), (24, 3.2),
        (28, 4.6), (31, 5.2), (35, 1.6), (39, 0.4), (44, 0.9),  # arc: chin dips mid-turn
        (48, -2.0), (53, -3.2), (56, -2.6),                     # chin up with the smirk
        (61, -2.2), (63, 0.3), (66, -1.0),                      # nod accent on "long"
        (71, -1.6), (74, 0.6), (78, -0.8), (84, -1.2),          # accent on "-nough"
        (90, 0.2), (96, 1.2),
        (98, 4.8), (100, 5.4),                                  # anticipation: chin tucks
        (102, -8.6), (104, -10.2), (107, -9.0), (112, -9.4), (121, -8.8),
        (124, 2.0), (126, 4.5), (129, 1.0), (131, 3.4), (134, 0.8), (136, 2.4),   # laugh bobs
        (139, 0.4), (141, 1.5), (144, 0.2), (146, 0.9), (150, 0.2),
        (158, 1.6), (166, 3.2), (172, 2.8), (180, 3.0)])
    tremble = np.zeros_like(F)
    for i, f in enumerate(F):
        if 104 <= f <= 121:
            k = (f - 104) / 17.0
            tremble[i] = 0.8 * (1 - k) ** 1.2 * math.sin((f - 104) * 2 * math.pi / 3.0)
    A["Head.rz"] = S([
        (1, 3.2), (16, 2.8), (24, 3.4),
        (28, 2.0), (34, -1.0), (38, 0.4), (43, -0.2),
        (47, -2.8), (51, -5.0), (55, -4.4),                     # cock toward the smirk
        (64, -3.4), (74, -4.2), (84, -3.6),
        (92, -1.2), (97, 0.6),
        (99, 1.4), (102, -3.5), (105, -2.4), (121, -2.8),
        (124, 5.0), (127, 7.6), (131, 6.6), (136, 7.2), (144, 6.2), (150, 5.6),
        (158, 4.0), (168, 3.0), (180, 3.2)]) + tremble
    # a living drift under every hold (never a dead stop)
    A["FaceAngle.ry"] += drift(F, 0.45, phase=0.3)
    A["FaceAngle.rx"] += drift(F, 0.35, periods=(83.0, 127.0, 61.0), phase=1.1)
    A["Head.rz"] += drift(F, 0.30, periods=(97.0, 59.0, 139.0), phase=2.0)
    A["Neck.rz"] = 0.35 * np.roll(A["Head.rz"], 2)
    A["Neck.rz"][:2] = A["Neck.rz"][2]

    sway = S([(1, 0.0), (30, 0.10), (60, -0.05), (95, 0.0), (99, 0.15), (102, -0.35), (108, -0.25),
              (124, 0.10), (128, 0.25), (140, 0.15), (160, -0.05), (180, 0.0)])
    A["Body.tx"] = sway
    A["Body.rz"] = S([(1, 0.3), (40, -0.4), (70, -0.2), (99, 0.2), (102, -1.0), (110, -0.8),
                      (124, 0.4), (130, 0.9), (150, 0.6), (168, 0.2), (180, 0.3)])
    A["Torso.ry"] = 0.26 * np.concatenate([np.full(4, A["FaceAngle.ry"][0]), A["FaceAngle.ry"][:-4]])
    # breathing: an uneven cycle, a sharp inhale in the anticipation, a
    # held gasp in the take, laugh pumps, a long exhale into the settle
    br = 0.5 - 0.5 * np.cos(2 * np.pi * (F - 1) / 104.0)
    br *= 1.0 + 0.2 * np.sin(2 * np.pi * F / 61.0)
    extra = S([(1, 0), (95, 0, "flat"), (99, 0.55), (102, 0.75), (110, 0.6), (121, 0.5),
               (124, 0.3), (127, 0.55), (129, 0.2), (132, 0.45), (134, 0.15), (137, 0.35),
               (139, 0.1), (142, 0.22), (146, 0.05), (152, -0.25), (162, -0.35), (172, -0.1), (180, 0)])
    A["Torso.ty"] = np.clip(br * 0.75 + extra, -0.4, 1.2)

    look_x = S([
        (1, -0.58), (9, -0.58, "flat"), (11, -0.66), (13, -0.64, "flat"), (19, -0.64, "flat"),
        (21, -0.56), (23, -0.57, "flat"),
        (24, -0.57, "flat"), (26, 0.16), (27, 0.20), (29, 0.12, "flat"),        # the eyes lead the head
        (40, 0.05), (52, 0.0),
        (64, 0.0, "flat"), (66, -0.30), (67, -0.33), (70, -0.30, "flat"),      # a glance aside
        (71, -0.30, "flat"), (73, 0.02), (74, 0.05), (76, 0.0, "flat"),
        (85, 0.0, "flat"), (87, 0.62), (88, 0.70), (90, 0.66, "flat"),          # realisation
        (99, 0.6, "flat"), (101, 0.05), (103, 0.0, "flat"),
        (121, 0.0, "flat"), (124, 0.0), (150, 0.0, "flat"),
        (152, 0.0, "flat"), (154, 0.22), (155, 0.25), (157, 0.2, "flat"),       # a glance back...
        (160, 0.2, "flat"), (162, -0.52), (163, -0.60), (165, -0.56, "flat"),   # ...and away again
        (180, -0.58)])
    look_y = S([(1, -0.10), (24, -0.12, "flat"), (26, 0.05), (28, 0.02, "flat"), (64, 0.0, "flat"),
                (66, -0.18), (71, -0.16, "flat"), (73, 0.0), (85, 0.0, "flat"), (87, 0.12), (99, 0.1, "flat"),
                (101, 0.05), (121, 0.0), (160, 0.0, "flat"), (163, -0.10), (180, -0.10)])
    # eye micro-drift (fixational), tiny
    A["Look.tx"] = look_x + drift(F, 0.018, periods=(37.0, 53.0, 23.0), phase=0.7)
    A["Look.ty"] = look_y + drift(F, 0.015, periods=(41.0, 29.0, 67.0), phase=1.9)
    A["Look.tz"] = S([(1, 0), (100, 0, "flat"), (102, 0.82), (104, 0.72), (121, 0.74, "flat"),
                      (124, 0.0), (180, 0.0)])
    # blinks: irregular, one riding each big head turn, the laugh closes
    # the eyes into ^^, then they open slowly into composure
    bl_R = blink([26, 84, 158], F)
    bl_L = blink([26.6, 84.5, 158.4], F)
    wide = S([(1, 0), (86, 0, "flat"), (89, 0.15), (99, 0.12), (100, -0.30), (101, 0.9), (103, 1.08),
              (106, 0.98), (121, 1.0, "flat"), (123, 0.0), (180, 0.0)])
    laugh_close = S([(1, 0), (121, 0, "flat"), (123, 0.9), (125, 1.0, "flat"), (147, 1.0, "flat"),
                     (150, 0.75), (153, 0.25), (156, 0.0), (180, 0)])
    A["Eye_R.ty"] = wide - np.maximum(bl_R, laugh_close)
    A["Eye_L.ty"] = wide - np.maximum(bl_L, laugh_close * 0.98)
    A["Eye_R.tx"] = S([(1, 0), (121, 0, "flat"), (123, 0.8), (125, 1.0), (147, 1.0, "flat"), (153, 0.3), (158, 0), (180, 0)])
    A["Eye_L.tx"] = A["Eye_R.tx"] * 0.95
    flat_R = S([(1, 0.62), (24, 0.64, "flat"), (27, 0.22), (32, 0.18), (44, 0.30), (50, 0.52), (56, 0.48),
                (70, 0.40), (84, 0.42), (88, 0.05), (99, 0.0, "flat"), (121, 0.0, "flat"),
                (150, 0.0, "flat"), (158, 0.35), (170, 0.58), (180, 0.62)])
    A["Eye_R.tz"] = flat_R
    A["Eye_L.tz"] = np.clip(flat_R * 0.86 + 0.02, 0, 1)

    A["Brow_L.ty"] = S([(1, -0.18), (24, -0.2, "flat"), (30, -0.02), (38, 0.0),
                        (41, 0.98), (43, 1.16), (47, 0.94), (54, 0.90),         # the flick
                        (62, 0.72), (64, 0.88), (70, 0.66), (75, 0.8), (84, 0.6),
                        (88, 0.35), (97, 0.3), (99, -0.45), (101, 1.05), (103, 1.12), (106, 1.0), (121, 1.02),
                        (124, 0.55), (150, 0.45), (160, 0.1), (172, -0.15), (180, -0.18)])
    A["Brow_R.ty"] = S([(1, -0.30), (24, -0.32, "flat"), (30, -0.12), (40, -0.14),
                        (44, -0.42), (54, -0.36), (64, -0.2), (70, -0.32), (84, -0.22),
                        (88, 0.18), (97, 0.2), (99, -0.5), (101, 0.92), (103, 1.02), (106, 0.9), (121, 0.95),
                        (124, 0.45), (150, 0.38), (160, 0.0), (172, -0.26), (180, -0.30)])
    A["Brow_L.rz"] = S([(1, 0), (40, 0, "flat"), (43, 7.0), (54, 5.0), (84, 4.0), (97, 2.0),
                        (99, -8.0), (101, 8.0), (104, 11.0), (121, 10.0),
                        (124, 9.0), (150, 8.0), (165, 1.0), (180, 0)])
    A["Brow_R.rz"] = S([(1, -2.0), (40, -2.0, "flat"), (44, -10.0), (54, -9.0), (84, -6.0), (97, -2.0),
                        (99, -10.0), (101, 6.0), (104, 9.0), (121, 8.0),
                        (124, 8.5), (150, 7.0), (165, 0.0), (180, -2.0)])

    # (open is -ty; tx wide(+)/round(-); rz smile(+)/frown(-) deg; ry smirk)
    op = [(1, 0), (52, 0, "flat"),
          # "Took"          "you"          "long"                 "e-"            "nough"           (f)
          (54, 0.30), (56, 0.34), (58, 0.04), (60, 0.26), (62, 0.10), (64, 0.72), (66, 0.78), (68, 0.30),
          (70, 0.38), (72, 0.20), (74, 0.66), (76, 0.72), (78, 0.30), (79, 0.14), (82, 0.0, "flat"),
          (99, 0.0, "flat"), (100, 0.08),
          (101, 1.15), (102, 1.40), (104, 1.36), (108, 1.32), (112, 1.38), (117, 1.30), (121, 1.34),   # the shout
          (123, 0.55), (125, 0.72),                                                        # snap to the grin
          (127, 0.85), (129, 0.45), (131, 0.72), (134, 0.40), (136, 0.62), (139, 0.32),     # ha, ha, ha
          (141, 0.46), (144, 0.24), (146, 0.30), (150, 0.12), (155, 0.04), (160, 0.0, "flat"), (180, 0)]
    A["Mouth.ty"] = -np.clip(S(op), 0, 1.45)
    A["Mouth.tx"] = S([(1, 0), (52, 0, "flat"), (54, -0.45), (58, -0.2), (60, -0.8), (62, -0.6),
                       (64, -0.35), (68, -0.1), (70, 0.55), (72, 0.3), (74, 0.10), (78, 0.35), (80, 0.1),
                       (84, 0.0), (99, 0.0, "flat"), (101, 0.30), (121, 0.3),
                       (123, 0.55), (150, 0.5), (158, 0.1), (170, 0.0), (180, 0)])
    A["Mouth.rz"] = S([(1, 0.5), (52, 0.5, "flat"), (60, 1.5), (70, 2.5), (84, 2.0), (90, 0.0), (99, -1.0),
                       (101, -15.0), (103, -19.0), (121, -17.0),
                       (123, 12.0), (125, 17.0), (147, 17.0), (152, 10.0), (160, 3.0), (170, 1.0), (180, 0.5)])
    A["Mouth.ry"] = S([(1, 2.0), (24, 2.0, "flat"), (44, 1.5), (47, 12.0), (50, 19.5), (52, 17.5),
                       (56, 12.0), (66, 8.0), (76, 9.0), (84, 15.0), (88, 5.0), (96, 2.0), (99, 0.0),
                       (101, -2.0), (121, -1.0), (124, 2.0), (150, 3.0), (160, 5.0), (170, 3.0), (180, 2.0)])

    A["Fx.tx"] = S([(1, 0), (101, 0, "flat"), (103, 0.95), (105, 1.0, "flat"), (150, 1.0, "flat"),
                    (158, 0.55), (166, 0.12), (170, 0.0, "flat"), (180, 0)])
    A["Fx.tz"] = S([(1, 0), (102, 0, "flat"), (105, 1.0, "flat"), (152, 1.0, "flat"), (158, 0.0, "flat"), (180, 0)])
    sw = S([(1, 0), (101, 0, "flat"), (104, 0.35), (108, 0.40), (138, 0.88), (142, 0.9), (147, 1.0, "flat"), (180, 1.0)])
    sw[F >= 170] = 0.0          # the drop has slid off (collapsed) -- reset while hidden
    A["Fx.ty"] = sw
    return F, A
