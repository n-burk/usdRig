"""CPU previews of the v2b bust's art: a rest-pose layer composite and a
small textured-triangle rasteriser (numba) for keyform checks, so the art
and its deformation fields can be iterated without building USD."""

import math
import os

import cv2
import numpy as np


def srgb_to_lin(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


def lin_to_srgb(c):
    c = np.clip(c, 0, 1)
    return np.where(c <= 0.0031308, c * 12.92, 1.055 * c ** (1 / 2.4) - 0.055)


def background(W, H, top=(169, 190, 245), bottom=(217, 204, 246)):
    t = np.linspace(0, 1, H)[:, None, None]
    a = np.array(top, np.float32)[None, None] / 255.0
    b = np.array(bottom, np.float32)[None, None] / 255.0
    return np.broadcast_to(a + (b - a) * t, (H, W, 3)).astype(np.float32).copy()


def composite(layers, region, ppu, bg=None, clip=None):
    """Composite finished layers (dicts with 'rgba' uint8, 'bbox', 'z')
    over a background, at rest. region = (x0, y0, x1, y1)."""
    x0, y0, x1, y1 = region
    W, H = int(round((x1 - x0) * ppu)), int(round((y1 - y0) * ppu))
    img = background(W, H) if bg is None else bg.copy()
    for L in sorted(layers, key=lambda l: l["z"]):
        rgba = L["rgba"].astype(np.float32) / 255.0
        bx0, by0, bx1, by1 = L["bbox"]
        # destination rect
        dx0 = (bx0 - x0) * ppu
        dy0 = (y1 - by1) * ppu
        dw = (bx1 - bx0) * ppu
        dh = (by1 - by0) * ppu
        sh, sw = rgba.shape[:2]
        M = np.float32([[dw / sw, 0, dx0], [0, dh / sh, dy0]])
        # premultiply for resampling
        pm = rgba.copy()
        pm[..., :3] *= pm[..., 3:4]
        interp = cv2.INTER_AREA if dw < sw else cv2.INTER_LINEAR
        if interp == cv2.INTER_AREA:
            # resize then place
            nw, nh = max(1, int(round(dw))), max(1, int(round(dh)))
            small = cv2.resize(pm, (nw, nh), interpolation=cv2.INTER_AREA)
            M2 = np.float32([[dw / nw, 0, dx0], [0, dh / nh, dy0]])
            warped = cv2.warpAffine(small, M2, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
        else:
            warped = cv2.warpAffine(pm, M, (W, H), flags=cv2.INTER_LINEAR, borderValue=0)
        a = warped[..., 3:4]
        if L.get("clip") is not None:
            cm = L["clip"]
            a = a * cm[..., None]
            warped = warped * cm[..., None]
        img = img * (1 - a) + warped[..., :3]
    return img


def save(img, path):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    out = np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8)
    cv2.imwrite(path, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))


def poly_mask(poly, region, ppu):
    x0, y0, x1, y1 = region
    W, H = int(round((x1 - x0) * ppu)), int(round((y1 - y0) * ppu))
    m = np.zeros((H, W), np.uint8)
    p = np.stack([(poly[:, 0] - x0) * ppu, (y1 - poly[:, 1]) * ppu], axis=1)
    cv2.fillPoly(m, [np.round(p * 16).astype(np.int32)], 255, cv2.LINE_AA, shift=4)
    return m.astype(np.float32) / 255.0


# textured triangle rasteriser

try:
    import numba

    @numba.njit(cache=True)
    def _raster(img, zbuf, P, UV, tris, tex, zval):
        H, W = img.shape[0], img.shape[1]
        th, tw = tex.shape[0], tex.shape[1]
        for f in range(tris.shape[0]):
            a, b, c = tris[f, 0], tris[f, 1], tris[f, 2]
            ax, ay = P[a, 0], P[a, 1]
            bx, by = P[b, 0], P[b, 1]
            cx, cy = P[c, 0], P[c, 1]
            den = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy)
            if abs(den) < 1e-12:
                continue
            xmin = max(int(math.floor(min(ax, bx, cx))), 0)
            xmax = min(int(math.ceil(max(ax, bx, cx))), W - 1)
            ymin = max(int(math.floor(min(ay, by, cy))), 0)
            ymax = min(int(math.ceil(max(ay, by, cy))), H - 1)
            for yy in range(ymin, ymax + 1):
                py = yy + 0.5
                for xx in range(xmin, xmax + 1):
                    px = xx + 0.5
                    l0 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / den
                    l1 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / den
                    l2 = 1.0 - l0 - l1
                    if l0 < -1e-6 or l1 < -1e-6 or l2 < -1e-6:
                        continue
                    if zbuf[yy, xx] > zval:
                        continue
                    u = l0 * UV[a, 0] + l1 * UV[b, 0] + l2 * UV[c, 0]
                    v = l0 * UV[a, 1] + l1 * UV[b, 1] + l2 * UV[c, 1]
                    # bilinear
                    fx = u * tw - 0.5
                    fy = (1.0 - v) * th - 0.5
                    x0 = int(math.floor(fx))
                    y0 = int(math.floor(fy))
                    tx = fx - x0
                    ty = fy - y0
                    acc0 = 0.0
                    acc1 = 0.0
                    acc2 = 0.0
                    acc3 = 0.0
                    for dy in range(2):
                        for dx in range(2):
                            sx = min(max(x0 + dx, 0), tw - 1)
                            sy = min(max(y0 + dy, 0), th - 1)
                            wgt = (tx if dx == 1 else 1 - tx) * (ty if dy == 1 else 1 - ty)
                            al = tex[sy, sx, 3]
                            acc0 += tex[sy, sx, 0] * al * wgt
                            acc1 += tex[sy, sx, 1] * al * wgt
                            acc2 += tex[sy, sx, 2] * al * wgt
                            acc3 += al * wgt
                    if acc3 <= 0.0:
                        continue
                    img[yy, xx, 0] = img[yy, xx, 0] * (1 - acc3) + acc0
                    img[yy, xx, 1] = img[yy, xx, 1] * (1 - acc3) + acc1
                    img[yy, xx, 2] = img[yy, xx, 2] * (1 - acc3) + acc2
    HAVE_NUMBA = True
except Exception:  # pragma: no cover
    HAVE_NUMBA = False


def render_meshes(meshes, region, ppu, ss=2, bg=None):
    """meshes: list of dicts with 'points' (N x 2 model), 'uv' (N x 2),
    'tris' (M x 3), 'tex' (uint8 RGBA), 'z'. Painter's order by z; each
    mesh is rasterised with supersampling ``ss``."""
    x0, y0, x1, y1 = region
    W, H = int(round((x1 - x0) * ppu)), int(round((y1 - y0) * ppu))
    Ws, Hs = W * ss, H * ss
    img = cv2.resize(background(W, H) if bg is None else bg, (Ws, Hs), interpolation=cv2.INTER_LINEAR)
    img = np.ascontiguousarray(img, np.float64)
    zbuf = np.full((Hs, Ws), -1e9)
    for m in sorted(meshes, key=lambda m: m["z"]):
        P = np.asarray(m["points"], np.float64)
        Pp = np.stack([(P[:, 0] - x0) * ppu * ss, (y1 - P[:, 1]) * ppu * ss], axis=1)
        tex = m["tex"].astype(np.float64) / 255.0
        _raster(img, zbuf, np.ascontiguousarray(Pp), np.ascontiguousarray(m["uv"], np.float64),
                np.ascontiguousarray(m["tris"], np.int64), np.ascontiguousarray(tex), float(m["z"]))
    return cv2.resize(img.astype(np.float32), (W, H), interpolation=cv2.INTER_AREA)


def render_parts(parts, texes, st=None, region=(-12, -14, 12, 13), ppu=40, ss=2, bg=None, only=None,
                 points=None):
    """Rasterise parts (rest, an expression state, or explicit points)."""
    meshes = []
    for p in parts:
        if only and not any(p.name.startswith(o) for o in only):
            continue
        t = texes[p.tex]
        if points is not None and p.name in points:
            pts = points[p.name]
        elif st is not None and p.gen is not None:
            pts = p.gen(st)
        else:
            pts = p.rest
        meshes.append(dict(points=pts, uv=p.uv(t.bbox), tris=p.tris, tex=t.rgba, z=p.z))
    return render_meshes(meshes, region, ppu, ss=ss, bg=bg)
