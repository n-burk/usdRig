"""Art parts: a painted texture + a triangle mesh with UVs + (optionally) a
function that regenerates the mesh's points for any expression state.

Mesh kinds (all with a FIXED topology, so keyforms are just other points):

    ribbon  a grid along a spine: (u along, v across in model units)
    band    a grid between two equally sampled curves
    ring    a closed band between an inner loop and an outer loop
    hull    the painted texture's alpha hull, triangulated (Delaunay)

UVs are the positions of the vertices in the PAINT pose mapped into the
texture's model-space box, so the painted pixels land exactly where they
were painted; the rest pose may differ (a mouth interior rests collapsed).
"""

import math

import cv2
import numpy as np
from matplotlib.path import Path
from scipy.spatial import Delaunay, cKDTree

import paint as P

# grids


def grid_tris(rows, cols, closed=False):
    tris = []
    nc = cols if closed else cols - 1
    for r in range(rows - 1):
        for c in range(nc):
            c1 = (c + 1) % cols
            a, b = r * cols + c, r * cols + c1
            d, e = (r + 1) * cols + c, (r + 1) * cols + c1
            tris.append((a, b, e))
            tris.append((a, e, d))
    return np.array(tris, np.int64).reshape(-1, 3)


def curve_normals(c, closed=False):
    c = np.asarray(c, np.float64)
    if closed:
        d = np.roll(c, -1, axis=0) - np.roll(c, 1, axis=0)
    else:
        d = np.gradient(c, axis=0)
    d /= np.linalg.norm(d, axis=1, keepdims=True) + 1e-12
    return np.stack([-d[:, 1], d[:, 0]], axis=1)


def ribbon_points(spine, vs, normals=None):
    """rows = vs (offsets along the LEFT normal, model units, scalar or per
    spine sample) -> (len(vs) * len(spine), 2) points, row-major."""
    spine = np.asarray(spine, np.float64)
    n = curve_normals(spine) if normals is None else normals
    rows = []
    for v in vs:
        v = np.broadcast_to(np.asarray(v, np.float64), (len(spine),))
        rows.append(spine + n * v[:, None])
    return np.vstack(rows)


def band_points(a, b, rows):
    a, b = np.asarray(a, np.float64), np.asarray(b, np.float64)
    ts = np.linspace(0, 1, rows)
    return np.vstack([a + (b - a) * t for t in ts])


# hull meshes


def _ccw(poly):
    x, y = poly[:, 0], poly[:, 1]
    area = 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)
    return poly if area > 0 else poly[::-1].copy()


def fill_polygon(boundary, h, hi=None):
    """Triangulate a closed outline: boundary resampled at spacing h, a hex
    lattice inside at spacing hi, Delaunay, outside triangles dropped."""
    hi = hi or h
    B = _ccw(P.resample(boundary, spacing=h, closed=True))
    path = Path(np.vstack([B, B[:1]]))
    lo, up = B.min(axis=0), B.max(axis=0)
    pts = []
    dy = hi * math.sqrt(3) / 2
    y = lo[1] + dy * 0.5
    r = 0
    while y < up[1]:
        xs = np.arange(lo[0] + (0.5 * hi if r % 2 else 0.0), up[0], hi)
        pts.append(np.stack([xs, np.full_like(xs, y)], axis=1))
        y += dy
        r += 1
    I = np.vstack(pts) if pts else np.zeros((0, 2))
    if len(I):
        I = I[path.contains_points(I)]
    if len(I):
        d, _ = cKDTree(P.resample(B, spacing=min(h, hi) * 0.5, closed=True)).query(I)
        I = I[d > 0.55 * hi]
    Pts = np.vstack([B, I]) if len(I) else B
    tri = Delaunay(Pts).simplices
    keep = path.contains_points(Pts[tri].mean(axis=1))
    tri = tri[keep]
    a, b, c = Pts[tri[:, 0]], Pts[tri[:, 1]], Pts[tri[:, 2]]
    cross = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0])
    tri[cross < 0] = tri[cross < 0][:, [0, 2, 1]]
    tri = tri[np.abs(cross) > 1e-9]
    return Pts, tri


def alpha_hull(layer_rgba, bbox, pad_texels=3, simplify=0.8, min_area_texels=30):
    """Outer contours of the painted alpha (dilated by pad) in model units."""
    a = (layer_rgba[..., 3] > 2).astype(np.uint8)
    k = 2 * pad_texels + 1
    a = cv2.dilate(a, np.ones((k, k), np.uint8))
    cs, _ = cv2.findContours(a, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    H, W = a.shape
    x0, y0, x1, y1 = bbox
    out = []
    for c in cs:
        if cv2.contourArea(c) < min_area_texels:
            continue
        c = cv2.approxPolyDP(c, simplify, True)[:, 0, :].astype(np.float64)
        if len(c) < 3:
            continue
        x = x0 + (c[:, 0] + 0.5) / W * (x1 - x0)
        y = y1 - (c[:, 1] + 0.5) / H * (y1 - y0)
        out.append(np.stack([x, y], axis=1))
    return out


def hull_mesh(rgba, bbox, h, hi=None, pad_texels=3):
    pts, tris = [], []
    off = 0
    for poly in alpha_hull(rgba, bbox, pad_texels):
        p, t = fill_polygon(poly, h, hi)
        pts.append(p)
        tris.append(t + off)
        off += len(p)
    return np.vstack(pts), np.vstack(tris)


# the part


class Part(object):
    """One art mesh.

    name      prim name
    group     rig group (which mover stack deforms it)
    z         draw order / depth
    tex       texture key (parts may share a texture, e.g. the eye masks
              use the face's)
    rest      (N, 2) rest points
    tris      (M, 3)
    uv_pts    (N, 2) model-space positions the UVs are taken from
    gen       gen(state) -> (N, 2) points for an expression state (None
              for static parts)
    material  "mask" (cut-out, opacityThreshold) or "blend" (translucent)
    """

    def __init__(self, name, group, z, tex, rest, tris, uv_pts=None, gen=None, material="mask",
                 extra=None):
        self.name = name
        self.group = group
        self.z = float(z)
        self.tex = tex
        self.rest = np.asarray(rest, np.float64)
        self.tris = np.asarray(tris, np.int64)
        self.uv_pts = self.rest if uv_pts is None else np.asarray(uv_pts, np.float64)
        self.gen = gen
        self.material = material
        self.extra = extra or {}

    def uv(self, bbox):
        x0, y0, x1, y1 = bbox
        return np.stack([(self.uv_pts[:, 0] - x0) / (x1 - x0), (self.uv_pts[:, 1] - y0) / (y1 - y0)], axis=1)


class Tex(object):
    """A finished texture: straight-alpha RGBA uint8 + its model-space box."""

    def __init__(self, name, rgba, bbox):
        self.name = name
        self.rgba = rgba
        self.bbox = bbox


def finish(layer, bleed=12):
    return Tex(layer.name, layer.finish(bleed=bleed), layer.bbox)


# an optional on-disk cache of finished textures (set by the caller while
# iterating on geometry or the rig; the build itself always repaints)
TEX_CACHE = {"dir": None, "only": None}


def painted(name, painter, bleed=12):
    """Finished texture `name`, painted by `painter()` -- or loaded from the
    cache directory when caching is on and `name` is not being repainted."""
    import os
    d = TEX_CACHE["dir"]
    path = os.path.join(d, name + ".npz") if d else None
    redo = TEX_CACHE["only"]
    if path and os.path.exists(path) and not (redo and any(name.startswith(r) for r in redo)):
        z = np.load(path)
        return Tex(name, z["rgba"], tuple(z["bbox"]))
    t = finish(painter(), bleed)
    if path:
        np.savez_compressed(path, rgba=t.rgba, bbox=np.array(t.bbox))
    return t
