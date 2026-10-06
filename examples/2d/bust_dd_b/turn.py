"""Authored three-quarter head construction for limited 2D rotation.

The facial silhouette, eye planes and hair volumes use separate projections.
All maps are identity at zero, preserve local feature proportions, and are
sampled into ordinary RigExec lattice and corrective keyforms by the builder.
"""

import math
import numpy as np
from scipy.interpolate import PchipInterpolator
import design as D

# At +30 degrees: near cheek, far cheek, and facial centreline displacement.
# These landmarks describe the jaw/cheek silhouette, not a spherical squeeze.
_Y = np.array([-10, -9.3, -7.7, -5.65, -3.5, -1.6, 1, 4.2, 7.4, 9.35, 10.5])
_NEAR = PchipInterpolator(_Y, [1.85, 2.1, 2.0, 1.8, 1.1, 0.6, 0.2, 0.2, 0.3, 0.35, 0.3])
_FAR = PchipInterpolator(
    _Y, [1.85, 0.65, -0.10, -0.7, -0.4, -0.1, 0, 0, -0.05, 0.05, 0.3]
)
_CENTRE = PchipInterpolator(
    _Y, [1.85, 1.90, 2.25, 2.65, 2.9, 1.6, 1.1, 0.7, 0.45, 0.3, 0.3]
)
_FRONT_DEPTH = PchipInterpolator(
    [-17, -10, -7, -4, 0, 5, 10, 16], [4, 4.3, 5.3, 6.1, 5.8, 4.8, 2.3, 1.5]
)


def turn_displacement(pts, offsets, theta_deg, **unused):
    p = np.asarray(pts, float)
    x, y = p[:, 0], p[:, 1]
    sign = 1 if theta_deg >= 0 else -1
    a = abs(math.sin(math.radians(theta_deg))) / 0.5
    yy = np.clip(y, _Y[0], _Y[-1])
    radius = np.maximum(D.face_halfwidth(np.clip(y, -9.9, 9.8)), 1.0)
    u = np.clip(sign * x / radius, -1, 1)
    # Monotone Hermite interpolation through both cheeks and the centreline
    # prevents the far cheek folding when the nose moves forward.
    near, far, centre = _NEAR(yy), _FAR(yy), _CENTRE(yy)
    left, right = -radius + near, radius + far
    centre = np.clip(centre, left + 0.05 * radius, right - 0.05 * radius)
    d0, d1 = centre - left, right - centre
    middle = 2 * d0 * d1 / (d0 + d1)
    m0 = np.clip((3 * d0 - d1) / 2, 0.05 * radius, 2 * d0)
    m2 = np.clip((3 * d1 - d0) / 2, 0.05 * radius, 2 * d1)
    t = np.where(u < 0, u + 1, u)
    start, end = np.where(u < 0, left, centre), np.where(u < 0, centre, right)
    ds, de = np.where(u < 0, m0, middle), np.where(u < 0, middle, m2)
    projected = (
        (2 * t**3 - 3 * t * t + 1) * start
        + (t**3 - 2 * t * t + t) * ds
        + (-2 * t**3 + 3 * t * t) * end
        + (t**3 - t * t) * de
    )
    dx = projected - u * radius
    outside = x - np.clip(x, -radius, radius)
    dx = sign * a * dx + outside * (math.cos(math.radians(theta_deg)) - 1)
    dx += np.asarray(offsets) * math.sin(math.radians(theta_deg))
    return np.column_stack([dx, np.zeros(len(p))])


def nod_displacement(pts, offsets, phi_deg, **unused):
    p = np.asarray(pts, float)
    x, y = p[:, 0], p[:, 1]
    th = math.radians(phi_deg)
    radius = np.maximum(D.face_halfwidth(np.clip(y, -9.9, 9.8)), 1.0)
    depth = _FRONT_DEPTH(np.clip(y, -17, 16)) * (
        1 - 0.18 * np.minimum((x / radius) ** 2, 1)
    )
    dy = (y + 1) * (math.cos(th) - 1) - (depth + offsets) * math.sin(th)
    return np.column_stack([np.zeros(len(p)), dy])


def project_part(points, name, kind, angle):
    """Project a whole feature plane or hair volume with one coherent map."""
    p = np.asarray(points, float)
    th = math.radians(angle)
    fn = turn_displacement if kind == "turn" else nod_displacement
    out = p + fn(p, np.zeros(len(p)), angle)
    eye_side = None
    if name.startswith(
        (
            "Sclera_",
            "LidShadow_",
            "Iris_",
            "Highlight_",
            "EyeMask_",
            "LowerLid_",
            "Crease_",
            "Lash_",
            "Brow_",
        )
    ):
        eye_side = 1 if name.endswith("_R") else -1
    if eye_side is not None:
        cx, cy = D.EYE_C[eye_side]
        if kind == "turn":
            a = abs(math.sin(th)) / 0.5
            far = eye_side * angle > 0
            width = 1 - (0.18 if far else 0.025) * a
            centre = cx * (1 - 0.14 * a) + 1.45 * math.sin(th) / 0.5
            out[:, 0] = centre + (p[:, 0] - cx) * width
            out[:, 1] = p[:, 1]
        else:
            centre = cy + fn(np.array([[cx, cy]]), np.zeros(1), angle)[0, 1]
            out[:, 1] = centre + (p[:, 1] - cy) * (
                1 - 0.07 * abs(math.sin(th)) / math.sin(math.radians(20))
            )
        return out
    if name in (
        "MouthInside",
        "Tongue",
        "Teeth",
        "LowerLip",
        "LowerEdge",
        "MouthLine",
        "TongueOut",
    ):
        centre = np.array(D.MOUTH_C)
        moved = centre + fn(centre[None], np.zeros(1), angle)[0]
        out = p - centre
        out[:, 0] *= 1 - 0.12 * abs(math.sin(th)) / 0.5 if kind == "turn" else 1
        out[:, 1] *= 1 if kind == "turn" else math.cos(th)
        return out + moved
    hair_depth = {
        "BackHair": -1.0,
        "Crown": 2.1,
        "Ahoge": 2.1,
        "F1": 4.2,
        "F2": 4.2,
        "F3": 4.0,
        "F4": 3.8,
        "F5": 3.5,
        "Stray": 4.2,
        "Pins": 3.6,
        "LockR_A": 2.6,
        "LockR_B": 1.7,
        "LooseL": 1.5,
        "TuckL": -0.5,
        "Ear": -0.7,
        "Earring": -0.7,
    }
    if name in hair_depth:
        depth = hair_depth[name]
        out = p.copy()
        if kind == "turn":
            out[:, 0] = p[:, 0] * math.cos(th) + depth * math.sin(th)
        else:
            out[:, 1] = -1 + (p[:, 1] + 1) * math.cos(th) - depth * math.sin(th)
        return out
    return out
