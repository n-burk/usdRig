"""Curve and mesh helpers for the v2a anime bust (plain numpy).

Every art part is a FUNCTION OF ITS PARAMETERS WITH A FIXED TOPOLOGY, so a
keyform (blend target) is simply the generator evaluated at another
parameter value. The helpers here build the outlines (centripetal
Catmull-Rom), resample them by arc length, and triangulate them into the
dense "ArtMesh" a layered mesh deformation rig deforms.
"""

import math

import numpy as np
from matplotlib.path import Path
from scipy.spatial import Delaunay


def A(p):
    return np.asarray(p, dtype=float)


def smoothstep(a, b, x):
    t = np.clip((np.asarray(x, dtype=float) - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def lerp(a, b, t):
    return A(a) + (A(b) - A(a)) * t


def catmull_rom(ctrl, closed=False, per_seg=24, alpha=0.5):
    """Centripetal Catmull-Rom through ctrl; a dense polyline."""
    P = A(ctrl)
    if closed:
        P = np.vstack([P[-1], P, P[0], P[1]])
    else:
        P = np.vstack([2 * P[0] - P[1], P, 2 * P[-1] - P[-2]])
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]

        def tj(ti, a, b):
            return ti + max(np.linalg.norm(b - a), 1e-6) ** alpha

        t0 = 0.0
        t1 = tj(t0, p0, p1)
        t2 = tj(t1, p1, p2)
        t3 = tj(t2, p2, p3)
        s = np.linspace(t1, t2, per_seg, endpoint=False)[:, None]
        a1 = (t1 - s) / (t1 - t0) * p0 + (s - t0) / (t1 - t0) * p1
        a2 = (t2 - s) / (t2 - t1) * p1 + (s - t1) / (t2 - t1) * p2
        a3 = (t3 - s) / (t3 - t2) * p2 + (s - t2) / (t3 - t2) * p3
        b1 = (t2 - s) / (t2 - t0) * a1 + (s - t0) / (t2 - t0) * a2
        b2 = (t3 - s) / (t3 - t1) * a2 + (s - t1) / (t3 - t1) * a3
        out.append((t2 - s) / (t2 - t1) * b1 + (s - t1) / (t2 - t1) * b2)
    out = np.vstack(out)
    if not closed:
        out = np.vstack([out, P[-2]])
    return out


def arclen(poly, closed=False):
    P = A(poly)
    if closed:
        P = np.vstack([P, P[:1]])
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def resample(poly, n, closed=False):
    """n points evenly spaced by arc length along poly."""
    P = A(poly)
    Q = np.vstack([P, P[:1]]) if closed else P
    s = arclen(P, closed)
    ts = np.linspace(0.0, s[-1], n, endpoint=not closed)
    return np.stack([np.interp(ts, s, Q[:, 0]), np.interp(ts, s, Q[:, 1])], axis=1)


def resample_spacing(poly, h, closed=False, minimum=6):
    total = arclen(poly, closed)[-1]
    n = max(minimum, int(math.ceil(total / h)) + (0 if closed else 1))
    return resample(poly, n, closed)


def tangents(poly, closed=False):
    P = A(poly)
    if closed:
        T = np.roll(P, -1, axis=0) - np.roll(P, 1, axis=0)
    else:
        T = np.gradient(P, axis=0)
    return T / np.maximum(np.linalg.norm(T, axis=1, keepdims=True), 1e-12)


def normals(poly, closed=False):
    """Unit LEFT normals (tangent rotated +90 degrees)."""
    T = tangents(poly, closed)
    return np.stack([-T[:, 1], T[:, 0]], axis=1)


def signed_area(poly):
    P = A(poly)
    x, y = P[:, 0], P[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def ccw(poly):
    return A(poly) if signed_area(poly) > 0 else A(poly)[::-1].copy()


def mirror(points):
    P = A(points).copy()
    P[..., 0] *= -1.0
    return P


def rot(points, deg, about=(0.0, 0.0)):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    P = A(points) - A(about)
    return np.stack([c * P[:, 0] - s * P[:, 1], s * P[:, 0] + c * P[:, 1]], axis=1) + A(about)


def ellipse(center, rx, ry, n=64, rot_deg=0.0, start=0.0):
    a = start + np.linspace(0, 2 * np.pi, n, endpoint=False)
    P = np.stack([rx * np.cos(a), ry * np.sin(a)], axis=1)
    return rot(P, rot_deg) + A(center)


def offset_curve(poly, d, closed=False):
    return A(poly) + normals(poly, closed) * np.asarray(d, dtype=float).reshape(-1, 1)


def stroke_outline(spine, widths, cap_round=True, cap_n=6):
    """Closed polygon of a variable-width stroke along an open spine."""
    S = A(spine)
    W = np.broadcast_to(np.asarray(widths, dtype=float), (len(S),))
    N = normals(S)
    L = S + N * (W / 2)[:, None]
    R = S - N * (W / 2)[:, None]
    parts = [L]
    if cap_round and W[-1] > 1e-4:
        T = tangents(S)[-1]
        c = S[-1]
        n0 = N[-1]
        ang = np.linspace(0, np.pi, cap_n + 2)[1:-1]
        cap = [c + (n0 * math.cos(a) + T * math.sin(a)) * W[-1] / 2 for a in ang]
        parts.append(A(cap))
    parts.append(R[::-1])
    if cap_round and W[0] > 1e-4:
        T = tangents(S)[0]
        c = S[0]
        n0 = N[0]
        ang = np.linspace(0, np.pi, cap_n + 2)[1:-1]
        cap = [c + (-n0 * math.cos(a) - T * math.sin(a)) * W[0] / 2 for a in ang]
        parts.append(A(cap))
    return np.vstack(parts)


def taper(t, root=1.3, kind="root"):
    """Width multipliers: 'root' lines thin toward the tip, 'float' lines
    swell in the middle and vanish at both ends."""
    t = np.asarray(t, dtype=float)
    if kind == "root":
        return (1 - t) ** root
    return np.sin(np.pi * np.clip(t, 0, 1)) ** 0.6


# Meshes

def grid_tris(rows, cols, closed_cols=False):
    """Triangles of a rows x cols vertex grid (row-major)."""
    tris = []
    ncol = cols if closed_cols else cols - 1
    for r in range(rows - 1):
        for c in range(ncol):
            c1 = (c + 1) % cols
            a = r * cols + c
            b = r * cols + c1
            d = (r + 1) * cols + c
            e = (r + 1) * cols + c1
            tris.append((a, b, e))
            tris.append((a, e, d))
    return np.array(tris, dtype=np.int64).reshape(-1, 3)


def fill_polygon(boundary, hb, hi=None, closed=True, dense_near=None):
    """Triangulate the inside of a closed outline.

    Boundary resampled at spacing hb, a hexagonal lattice of interior points
    at spacing hi, Delaunay, and the triangles whose centroid falls outside
    dropped. Returns (points, triangles, boundary_count).
    """
    hi = hi or hb
    B = ccw(resample_spacing(boundary, hb, closed=True))
    path = Path(np.vstack([B, B[:1]]))
    lo, hi_ = B.min(axis=0), B.max(axis=0)
    rows = []
    dy = hi * math.sqrt(3) / 2
    y = lo[1] + dy * 0.5
    r = 0
    while y < hi_[1]:
        xs = np.arange(lo[0] + (hi * 0.5 if r % 2 else 0.25 * hi), hi_[0], hi)
        rows.append(np.stack([xs, np.full_like(xs, y)], axis=1))
        y += dy
        r += 1
    I = np.vstack(rows) if rows else np.zeros((0, 2))
    if len(I):
        I = I[path.contains_points(I, radius=-1e-9)]
    if len(I):
        from scipy.spatial import cKDTree
        d, _ = cKDTree(resample_spacing(B, min(hb, hi) * 0.5, closed=True)).query(I)
        I = I[d > 0.6 * hi]
    P = np.vstack([B, I]) if len(I) else B
    tri = Delaunay(P).simplices
    cent = P[tri].mean(axis=1)
    keep = path.contains_points(cent)
    tri = tri[keep]
    a, b, c = P[tri[:, 0]], P[tri[:, 1]], P[tri[:, 2]]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    tri[cross < 0] = tri[cross < 0][:, [0, 2, 1]]
    # drop degenerate slivers
    area = np.abs(cross) * 0.5
    tri = tri[area > 1e-9]
    return P, tri, len(B)


def ribbon(spine, widths, rows=3, offset=0.0):
    """A grid along an open spine: `rows` across (row 0 on the RIGHT of the
    spine direction, last row on the LEFT). Returns (pts, tris, uv) with
    uv = (along 0..1, across 0..1)."""
    S = A(spine)
    W = np.broadcast_to(np.asarray(widths, dtype=float), (len(S),))
    N = normals(S)
    t = arclen(S)
    t = t / max(t[-1], 1e-9)
    vs = np.linspace(0.0, 1.0, rows)
    layers = [S + N * (W * (v - 0.5 + offset))[:, None] for v in vs]
    pts = np.concatenate(layers, axis=0)
    tris = grid_tris(rows, len(S))
    uv = np.stack([np.tile(t, rows), np.repeat(vs, len(S))], axis=1)
    return pts, tris, uv


def band_between(curve_a, curve_b, rows):
    """Grid between two equally sampled curves (row 0 == curve_a)."""
    Ca, Cb = A(curve_a), A(curve_b)
    vs = np.linspace(0.0, 1.0, rows)
    pts = np.concatenate([Ca * (1 - v) + Cb * v for v in vs], axis=0)
    n = len(Ca)
    tris = grid_tris(rows, n)
    u = arclen(Ca)
    u = u / max(u[-1], 1e-9)
    uv = np.stack([np.tile(u, rows), np.repeat(vs, n)], axis=1)
    return pts, tris, uv


def ring_between(inner, outer, rows):
    """Closed band between two equally sampled closed loops."""
    Ci, Co = A(inner), A(outer)
    vs = np.linspace(0.0, 1.0, rows)
    pts = np.concatenate([Ci * (1 - v) + Co * v for v in vs], axis=0)
    tris = grid_tris(rows, len(Ci), closed_cols=True)
    return pts, tris


def polar_disc(center, rx, ry, rings=8, segs=48, rot_deg=0.0):
    cx, cy = center
    pts = [(0.0, 0.0)]
    for i in range(1, rings + 1):
        r = i / float(rings)
        for j in range(segs):
            a = 2 * math.pi * j / segs
            pts.append((rx * r * math.cos(a), ry * r * math.sin(a)))
    pts = rot(A(pts), rot_deg) + A(center)
    tris = []
    for j in range(segs):
        tris.append((0, 1 + j, 1 + (j + 1) % segs))
    for i in range(1, rings):
        b0 = 1 + (i - 1) * segs
        b1 = 1 + i * segs
        for j in range(segs):
            j1 = (j + 1) % segs
            tris.append((b0 + j, b1 + j, b1 + j1))
            tris.append((b0 + j, b1 + j1, b0 + j1))
    return pts, np.array(tris, dtype=np.int64)


def boundary_edges(tris):
    """Directed boundary edges of a triangle set."""
    from collections import Counter
    e = np.concatenate([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]]])
    key = np.sort(e, axis=1)
    cnt = Counter(map(tuple, key))
    return np.array([ed for ed, k in zip(e, key) if cnt[tuple(k)] == 1])
