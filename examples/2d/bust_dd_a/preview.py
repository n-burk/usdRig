"""CPU previews of Kaede's pieces: a small textured-triangle rasteriser
(numba) that draws every piece's mesh with its own painted texture at any
expression state, in draw-depth order. Used to iterate on the art and the
keyforms without building USD; the real frames come from Storm."""

import math
import os

import cv2
import numpy as np

import numba


@numba.njit(cache=True)
def _raster(img, P, UV, tris, tex, soft):
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
                if l0 < -1e-7 or l1 < -1e-7 or l2 < -1e-7:
                    continue
                u = l0 * UV[a, 0] + l1 * UV[b, 0] + l2 * UV[c, 0]
                v = l0 * UV[a, 1] + l1 * UV[b, 1] + l2 * UV[c, 1]
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
                # masked (cut-out) like the Storm material: alpha >= 0.5 is
                # opaque; keep a narrow ramp for anti-aliasing
                k = min(max((acc3 - 0.25) * 2.0, 0.0), 1.0)
                if soft:
                    k = acc3
                if k <= 0.0:
                    continue
                img[yy, xx, 0] = img[yy, xx, 0] * (1 - k) + acc0 / acc3 * k
                img[yy, xx, 1] = img[yy, xx, 1] * (1 - k) + acc1 / acc3 * k
                img[yy, xx, 2] = img[yy, xx, 2] * (1 - k) + acc2 / acc3 * k


def srgb(c):
    return np.asarray(c, np.float64)


def background(W, H, top=(58, 66, 110), bottom=(36, 40, 70)):
    t = np.linspace(0, 1, H)[:, None, None]
    a = np.array(top, np.float64)[None, None] / 255.0
    b = np.array(bottom, np.float64)[None, None] / 255.0
    return np.ascontiguousarray(np.broadcast_to(a + (b - a) * t, (H, W, 3)))


def piece_uv(piece):
    x0, y0, x1, y1 = piece.bbox
    return np.stack([(piece.ref[:, 0] - x0) / (x1 - x0), (piece.ref[:, 1] - y0) / (y1 - y0)], axis=1)


def render(pieces, st, region, ppu, ss=2, bg=None, positions=None, wire=False):
    """Draw pieces at state ``st`` (or explicit ``positions`` {name: pts})."""
    x0, y0, x1, y1 = region
    W, H = int(round((x1 - x0) * ppu)), int(round((y1 - y0) * ppu))
    Ws, Hs = W * ss, H * ss
    img = background(Ws, Hs) if bg is None else cv2.resize(bg, (Ws, Hs))
    img = np.ascontiguousarray(img, np.float64)
    for pc in sorted(pieces, key=lambda p: p.z):
        if pc.rgba is None:
            continue
        pts = positions[pc.name] if positions and pc.name in positions else pc.pos(st)
        Pp = np.stack([(pts[:, 0] - x0) * ppu * ss, (y1 - pts[:, 1]) * ppu * ss], axis=1)
        tex = pc.rgba.astype(np.float64) / 255.0
        _raster(img, np.ascontiguousarray(Pp), np.ascontiguousarray(piece_uv(pc)),
                np.ascontiguousarray(pc.tris), np.ascontiguousarray(tex), bool(pc.meta.get("translucent")))
        if wire:
            for a, b, c in pc.tris[::1]:
                poly = np.round(Pp[[a, b, c]] * 4).astype(np.int32)
                cv2.polylines(img, [poly], True, (0.1, 0.9, 0.9), 1, cv2.LINE_AA, shift=2)
    return cv2.resize(img.astype(np.float32), (W, H), interpolation=cv2.INTER_AREA)


def save(img, path):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    out = np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8)
    cv2.imwrite(path, cv2.cvtColor(out, cv2.COLOR_RGB2BGR))


def tile(images, cols, pad=6, color=(1, 1, 1)):
    h = max(i.shape[0] for i in images)
    w = max(i.shape[1] for i in images)
    rows = int(math.ceil(len(images) / float(cols)))
    out = np.ones((rows * (h + pad) + pad, cols * (w + pad) + pad, 3), np.float32) * np.array(color, np.float32)
    for k, im in enumerate(images):
        r, c = divmod(k, cols)
        out[pad + r * (h + pad):pad + r * (h + pad) + im.shape[0], pad + c * (w + pad):pad + c * (w + pad) + im.shape[1]] = im
    return out
