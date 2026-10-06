"""Adult anime arms: head 20, upper arm 25, forearm 22, hand 16 units.

Separate sleeve pattern pieces overlap at the elbow without pinching.
Hands are drawn in a wrist-local frame shared by paint, keyforms and joints.
"""

import numpy as np
import art_body as AB
import art_face as AF
import paint as P
from parts import Part, painted, hull_mesh

FINGERS = ("Thumb", "Index", "Middle", "Ring", "Pinky")
ARM = np.array([(16.0, -22.0), (25.0, -45.3), (30.0, -23.9)])
HAND_SCALE = np.array([3.0, 3.15])
FINGER_PATHS = {
    "Thumb": [(-0.66, -0.75), (-1.27, -1.07), (-1.70, -1.53), (-1.83, -2.05)],
    "Index": [(-0.85, -2.20), (-0.95, -3.20), (-1.03, -4.10), (-0.98, -4.75)],
    "Middle": [(-0.17, -2.40), (-0.12, -3.55), (-0.13, -4.40), (-0.24, -5.00)],
    "Ring": [(0.48, -2.30), (0.63, -3.34), (0.70, -4.16), (0.63, -4.72)],
    "Pinky": [(1.05, -1.97), (1.30, -2.85), (1.48, -3.42), (1.43, -3.94)],
}
FOLDED_PATHS = {
    "Index": [(-0.85, -2.48), (-0.80, -2.1), (-0.69, -1.60)],
    "Middle": [(-0.19, -2.70), (-0.15, -2.15), (-0.12, -1.55)],
    "Ring": [(0.48, -2.57), (0.46, -2.03), (0.42, -1.48)],
    "Pinky": [(1.03, -2.18), (0.99, -1.81), (0.90, -1.40)],
}
_a = np.arctan2(*(ARM[-1] - ARM[-2])[::-1]) + np.pi / 2
HAND_ROTATION = np.array([[np.cos(_a), np.sin(_a)], [-np.sin(_a), np.cos(_a)]])


def mirror(points, side):
    return np.asarray(points, float) * (side, 1)


def arm_points(side):
    return mirror(ARM, side)


def hand_world(local, side):
    return mirror(np.asarray(local) * HAND_SCALE @ HAND_ROTATION + ARM[-1], side)


def hand_local(points, side):
    return (mirror(points, side) - ARM[-1]) @ HAND_ROTATION.T / HAND_SCALE


def finger_points(side, name):
    return hand_world(FINGER_PATHS[name], side)


def _bounds(points, pad):
    return (*tuple(np.min(points, axis=0) - pad), *tuple(np.max(points, axis=0) + pad))


def paint_sleeve(side, forearm):
    name = ("ForeSleeve_" if forearm else "UpperSleeve_") + ("R" if side > 0 else "L")
    start, end = arm_points(side)[1:] if forearm else arm_points(side)[:2]
    direction = (end - start) / np.linalg.norm(end - start)
    normal = np.array([-direction[1], direction[0]])
    length = np.linalg.norm(end - start)
    ts = np.linspace(0, 1, 80)
    spine = start + ts[:, None] * (end - start)
    widths = np.interp(
        ts,
        [0, 0.15, 0.7, 1],
        [3.15, 3.4, 3.05, 2.65] if forearm else [3.45, 3.7, 3.2, 3.1],
    )
    L = P.Layer(
        name,
        _bounds(
            np.vstack(
                [spine - widths[:, None] * normal, spine + widths[:, None] * normal]
            ),
            float(widths.max()) + 0.4,
        ),
        tpu=65,
        ss=2,
    )
    X, Y = L.XY()
    along = (X - start[0]) * direction[0] + (Y - start[1]) * direction[1]
    across = (X - start[0]) * normal[0] + (Y - start[1]) * normal[1]
    m = np.maximum(L.stroke(spine, widths), L.disc(*start, widths[0]))
    if not forearm:
        m = np.maximum(m, L.disc(*end, widths[-1]))
    else:
        m *= along <= length + 0.1
    L.paint(m, AB.CARD["base"])
    # Both pieces use flat colour at the overlap; the visible outer contour
    # and a short compression fold carry the elbow, not a cross-joint seam.
    fade = (
        P.smooth(2.0, 6.0, along)
        if forearm
        else P.smooth(2, 8, along) * (1 - P.smooth(length - 7, length - 2, along))
    )
    L.paint_in((across * side > 1.7) * fade, AB.CARD["shadow"], 0.85)
    contour = L.band_inside(m, 0.075) * (along > 3 if forearm else along < length - 3)
    if not forearm:
        contour *= along > 4
    L.paint_in(contour, AB.OUT, 0.85)
    if forearm:
        cuff = along > length - 3.6
        L.paint_in(cuff, AB.CARD["rib"])
        L.paint_in(np.abs(along - (length - 3.6)) < 0.055, AB.CARD["line"])
        for v in np.arange(-3.2, 3.2, 0.52):
            L.paint_in((np.abs(across - v) < 0.025) * cuff, AB.CARD["hi"], 0.55)
        for t in (5.0, 10.0):
            curve = start + np.array(
                [[t - 1.0, -2.5], [t, 0.2], [t - 0.4, 2.0]]
            ) @ np.stack([direction, normal])
            sp = P.resample(P.catmull(curve, n=10), n=35)
            L.paint_in(
                L.stroke(sp, P.taper(35, 0.05, head=0.2, tail=0.2)),
                AB.CARD["line"],
                0.62,
            )
    return L


def palm_outline(side):
    local = [
        (-0.80, 0.65),
        (0.80, 0.65),
        (0.89, -0.55),
        (1.20, -1.55),
        (1.34, -2.32),
        (1.04, -2.40),
        (0.84, -2.32),
        (0.52, -2.60),
        (0.17, -2.58),
        (-0.18, -2.73),
        (-0.53, -2.48),
        (-0.90, -2.61),
        (-1.12, -2.42),
        (-1.20, -1.45),
        (-0.97, -0.65),
    ]
    return hand_world(P.catmull(local, n=14, closed=True), side)


def paint_palm(side):
    tag = "R" if side > 0 else "L"
    outline = palm_outline(side)
    L = P.Layer("Palm_" + tag, _bounds(outline, 0.5), tpu=90, ss=2)
    X, Y = L.XY()
    local = hand_local(np.stack([X, Y], axis=-1), side)
    m = L.poly(outline)
    L.paint(m, AF.SKIN["base"])
    shadow = hand_world(
        P.catmull(
            [
                (0.88, -0.50),
                (1.20, -1.6),
                (1.18, -2.3),
                (0.88, -2.2),
                (0.80, -1.4),
                (0.63, -0.85),
            ],
            n=12,
            closed=True,
        ),
        side,
    )
    L.paint_in(L.poly(shadow), AF.SKIN["shadow"], 0.34)
    L.paint_in(
        L.band_inside(m, 0.075) * (local[..., 1] > -2.12), AF.SKIN["contour"], 0.65
    )
    for line in (
        [(-0.84, -0.58), (-0.52, -1.0), (-0.61, -1.68)],
        [(-0.80, -1.89), (-0.13, -2.03), (0.69, -1.83)],
        [(0.03, -0.69), (0.11, -1.13), (0.37, -1.57)],
    ):
        sp = P.resample(P.catmull(hand_world(line, side)), n=40)
        L.paint_in(
            L.stroke(sp, P.taper(40, 0.033, head=0.2, tail=0.3)), AF.SKIN["line"], 0.32
        )
    return L


def paint_finger(side, name):
    tag = "R" if side > 0 else "L"
    spine = P.resample(P.catmull(finger_points(side, name), n=14), n=64)
    width = HAND_SCALE[0] * (
        0.36 if name == "Thumb" else (0.255 if name == "Pinky" else 0.295)
    )
    L = P.Layer(name + "_" + tag, _bounds(spine, width + 0.3), tpu=90, ss=2)
    widths = width * np.interp(
        np.linspace(0, 1, len(spine)),
        [0, 0.12, 0.37, 0.49, 0.72, 0.84, 1],
        [1.18, 1.06, 1.0, 1.04, 0.87, 0.90, 0.69],
    )
    m = np.maximum(L.stroke(spine, widths), L.disc(*spine[-1], widths[-1]))
    L.paint(m, AF.SKIN["base"])
    X, Y = L.XY()
    local = hand_local(np.stack([X, Y], axis=-1), side)
    source = np.asarray(FINGER_PATHS[name])
    direction = source[-1] - source[0]
    t = ((local - source[0]) @ direction) / (direction @ direction)
    root_fade = P.smooth(
        0.22 if name == "Thumb" else 0.06, 0.55 if name == "Thumb" else 0.22, t
    )
    L.paint_in(
        (local[..., 0] > np.mean(source[:, 0]) + 0.15) * root_fade,
        AF.SKIN["shadow"],
        0.30,
    )
    L.paint_in(
        L.band_inside(m, 0.058) * root_fade,
        AF.SKIN["contour"],
        0.7,
    )
    for k in (1, 2):
        x, y = source[k]
        sp = P.resample(
            hand_world([(x - 0.13, y), (x, y - 0.045), (x + 0.13, y)], side), n=20
        )
        L.paint_in(
            L.stroke(sp, P.taper(20, 0.029, head=0.2, tail=0.3)), AF.SKIN["line"], 0.38
        )
    return L


def curl_points(points, side, name, amount):
    p = hand_local(points, side)
    root = np.asarray(FINGER_PATHS[name][0])
    p -= root
    if name == "Thumb":
        # Opposition folds the thumb across the palm. Uniform foreshortening
        # would leave a detached-looking horizontal sliver beside the fist.
        direction = np.asarray(FINGER_PATHS[name][-1]) - root
        direction /= np.linalg.norm(direction)
        along = (p @ direction)[:, None] * direction
        p = along * (1 - 0.12 * amount) + (p - along) * (1 - 0.08 * amount)
        angle = np.radians(85 * amount)
        p = p @ np.array(
            [[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]]
        )
    else:
        p += root
        # Map the finger's centreline, retaining the radial offset and end
        # cap. Affine scaling of the entire shape flattens the fingertip.
        source = P.resample(P.catmull(FINGER_PATHS[name], n=14), n=64)
        segments = np.diff(source, axis=0)
        lengths2 = np.sum(segments * segments, axis=1)
        delta = p[:, None, :] - source[:-1]
        ts = np.clip(np.sum(delta * segments, axis=2) / lengths2, 0, 1)
        closest = source[:-1] + ts[..., None] * segments
        pick = np.argmin(np.sum((p[:, None, :] - closest) ** 2, axis=2), axis=1)
        rows = np.arange(len(p))
        centre = closest[rows, pick]
        radial = p - centre
        compressed = root + (centre - root) * np.array(
            [1 - 0.90 * amount, 1 - 0.98 * amount]
        )
        return hand_world(compressed + radial, side)
    return hand_world(p + root, side)


def turn_points(points, side, degrees):
    p = hand_local(points, side)
    angle = np.radians(degrees)
    p[:, 0] = p[:, 0] * np.cos(angle) + 0.15 * np.sin(angle)
    return hand_world(p, side)


def paint_fold(side, finger):
    """The visible finger pad folds toward the palm behind the opposed thumb."""
    tag = "R" if side > 0 else "L"
    curve = hand_world(P.resample(P.catmull(FOLDED_PATHS[finger], n=14), n=48), side)
    width = HAND_SCALE[0] * (0.29 if finger == "Pinky" else 0.335)
    L = P.Layer(finger + "Fold_" + tag, _bounds(curve, width + 0.3), tpu=100, ss=2)
    m = np.maximum(
        L.stroke(curve, np.linspace(width, 0.86 * width, len(curve))),
        L.disc(*curve[0], width),
    )
    m = np.maximum(m, L.disc(*curve[-1], width * 0.86))
    L.paint(m, AF.SKIN["base"])
    X, Y = L.XY()
    local = hand_local(np.stack([X, Y], axis=-1), side)
    path = np.asarray(FOLDED_PATHS[finger])
    L.paint_in(local[..., 0] > np.mean(path[:, 0]) + 0.15, AF.SKIN["shadow"], 0.45)
    L.paint_in(L.band_inside(m, 0.052), AF.SKIN["line"], 0.60)
    x, y = path[1]
    line = hand_world(
        P.resample(P.catmull([(x - 0.15, y), (x, y + 0.035), (x + 0.15, y)]), n=24),
        side,
    )
    L.paint_in(
        L.stroke(line, P.taper(24, 0.035, head=0.2, tail=0.2)), AF.SKIN["line"], 0.36
    )
    return L


def folded_points(painted_points, side, finger, amount):
    p = hand_local(painted_points, side)
    root = np.asarray(FINGER_PATHS[finger][0])
    knuckle = np.asarray(FOLDED_PATHS[finger][0])
    tip = np.asarray(FINGER_PATHS[finger][-1])
    visible = P.smooth(0.50, 0.90, amount)
    projected_tip = root + (tip - root) * np.array(
        [1 - 0.90 * amount, 1 - 0.98 * amount]
    )
    centre = projected_tip * (1 - visible**5) + knuckle * visible**5
    return hand_world(
        centre + (p - knuckle) * np.array([min(1.0, 4 * visible), visible]), side
    )


def build():
    parts, textures = [], {}
    for side, tag in ((-1, "L"), (1, "R")):
        layers = [
            (
                "UpperSleeve_" + tag,
                lambda s=side: paint_sleeve(s, False),
                -0.65,
                0.34,
                "Arm_" + tag,
            ),
            (
                "ForeSleeve_" + tag,
                lambda s=side: paint_sleeve(s, True),
                -0.40,
                0.28,
                "Arm_" + tag,
            ),
            ("Palm_" + tag, lambda s=side: paint_palm(s), -0.45, 0.22, "Hand_" + tag),
        ]
        layers += [
            (
                n + "_" + tag,
                lambda s=side, n=n: paint_finger(s, n),
                -0.42 if n == "Thumb" else -0.50,
                0.18,
                "Hand_" + tag,
            )
            for n in FINGERS
        ]
        for name, painter, z, spacing, group in layers:
            t = painted(name, painter)
            textures[name] = t
            pts, tris = hull_mesh(t.rgba, t.bbox, spacing, spacing * 1.3, pad_texels=3)
            parts.append(Part(name, group, z, name, pts, tris))
        for finger in FOLDED_PATHS:
            name = finger + "Fold_" + tag
            t = painted(name, lambda s=side, f=finger: paint_fold(s, f))
            textures[name] = t
            pts, tris = hull_mesh(t.rgba, t.bbox, 0.16, 0.21, pad_texels=3)
            parts.append(
                Part(
                    name,
                    "Hand_" + tag,
                    -0.435,
                    name,
                    folded_points(pts, side, finger, 0),
                    tris,
                    uv_pts=pts,
                    extra={"fold_paint_points": pts},
                )
            )
    return parts, textures
