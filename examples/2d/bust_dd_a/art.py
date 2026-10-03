"""Kaede's art: every part is a PIECE -- a painted RGBA layer plus the mesh
that carries it and a function giving that mesh's points for any expression
state.

Three mesh families (the ArtMesh kinds a layered mesh artist builds by hand):

* ``strip``: rows of vertices along a spine at fixed (t, v) -- arc-length
  fraction along, signed offset across in model units. A keyform re-evaluates
  the spine at the new state and puts every vertex back at its (t, v), so a
  painted line bends without swelling (lashes, lids, brows, lips, contour).
* ``grid``: a regular grid clipped to the painted alpha, moved by a smooth
  displacement field (face skin, neck, body, hair masses).
* ``lens`` / ``ring``: rows between two curves (eye whites, mouth interior)
  or between a moving inner loop and a fixed outer loop (the eye masks that
  close over the eyeball).

A piece's texture is painted in its REFERENCE layout (usually the rest pose;
the collapsed pieces -- mouth interior, blush, effects -- are painted open
and rest collapsed), and its UVs are those reference positions mapped into
the piece's rectangle of its group's texture atlas.
"""

import math

import numpy as np

import design as D
import paint as P
from paint import catmull, hexc, resample, smooth

# geometry helpers


def arc_param(poly):
    s = P.arclen(poly)
    return s / max(s[-1], 1e-9)


def sample_at(poly, ts):
    """Points and unit left-normals of a polyline at arc-length fractions."""
    poly = np.asarray(poly, np.float64)
    s = arc_param(poly)
    x = np.interp(ts, s, poly[:, 0])
    y = np.interp(ts, s, poly[:, 1])
    tan = np.gradient(poly, axis=0)
    tan /= np.linalg.norm(tan, axis=1, keepdims=True) + 1e-12
    tx = np.interp(ts, s, tan[:, 0])
    ty = np.interp(ts, s, tan[:, 1])
    n = np.hypot(tx, ty) + 1e-12
    return np.stack([x, y], axis=1), np.stack([-ty / n, tx / n], axis=1)


def grid_tris(m, k):
    """Triangles of an m x k vertex grid (row-major, m along, k across)."""
    tris = []
    for i in range(m - 1):
        for j in range(k - 1):
            a = i * k + j
            b = a + 1
            c = a + k
            d = c + 1
            tris.append((a, c, b))
            tris.append((b, c, d))
    return np.array(tris, np.int64)


class Strip(object):
    """A (t, v) vertex grid along a spine function spine(state)."""

    def __init__(self, spine_fn, ts, vs, up=1.0):
        self.spine_fn = spine_fn
        self.ts = np.asarray(ts, np.float64)
        self.vs = np.asarray(vs, np.float64)
        self.up = up

    def pos(self, st):
        P0, N = sample_at(self.spine_fn(st), self.ts)
        N = N * self.up
        pts = P0[:, None, :] + N[:, None, :] * self.vs[None, :, None]
        return pts.reshape(-1, 2)

    def tris(self):
        return grid_tris(len(self.ts), len(self.vs))


def dense_ts(poly, spacing, lo=0.0, hi=1.0, extra=()):
    L = P.arclen(poly)[-1] * (hi - lo)
    n = max(3, int(math.ceil(L / spacing)) + 1)
    ts = np.linspace(lo, hi, n)
    if extra:
        ts = np.unique(np.concatenate([ts, extra]))
    return ts


def grid_from_alpha(layer_rgba, bbox, spacing, dilate=1):
    """Vertices of a regular grid over bbox, keeping the cells that touch
    painted alpha (dilated by ``dilate`` cells). Returns (verts, tris)."""
    import cv2
    x0, y0, x1, y1 = bbox
    nx = max(2, int(math.ceil((x1 - x0) / spacing)) + 1)
    ny = max(2, int(math.ceil((y1 - y0) / spacing)) + 1)
    xs = np.linspace(x0, x1, nx)
    ys = np.linspace(y0, y1, ny)
    a = layer_rgba[..., 3].astype(np.float32) / 255.0
    H, W = a.shape
    # coverage per cell: max alpha over the texels in the cell
    cell = np.zeros((ny - 1, nx - 1), bool)
    cx = np.clip(((xs - x0) / (x1 - x0) * W).astype(int), 0, W)
    cy = np.clip(((y1 - ys) / (y1 - y0) * H).astype(int), 0, H)
    occ = (a > 0.01).astype(np.uint8)
    integ = cv2.integral(occ)
    for j in range(ny - 1):
        r0, r1 = cy[j + 1], cy[j]
        if r1 <= r0:
            r0, r1 = r1, r0
        r1 = max(r1, r0 + 1)
        for i in range(nx - 1):
            c0, c1 = cx[i], max(cx[i + 1], cx[i] + 1)
            s = integ[min(r1, H), min(c1, W)] - integ[r0, min(c1, W)] - integ[min(r1, H), c0] + integ[r0, c0]
            cell[j, i] = s > 0
    if dilate:
        k = np.ones((2 * dilate + 1, 2 * dilate + 1), np.uint8)
        cell = cv2.dilate(cell.astype(np.uint8), k).astype(bool)
    used = np.zeros((ny, nx), bool)
    quads = np.argwhere(cell)
    for j, i in quads:
        used[j:j + 2, i:i + 2] = True
    index = -np.ones((ny, nx), np.int64)
    vj, vi = np.nonzero(used)
    index[vj, vi] = np.arange(len(vj))
    verts = np.stack([xs[vi], ys[vj]], axis=1)
    tris = []
    for j, i in quads:
        a0, b0 = index[j, i], index[j, i + 1]
        c0, d0 = index[j + 1, i], index[j + 1, i + 1]
        tris.append((a0, b0, c0))
        tris.append((b0, d0, c0))
    return verts, np.array(tris, np.int64)


# the piece

DEFAULT_STATE = dict(eye={+1: "open", -1: "open"}, look=(0.0, 0.0), shrink=0.0,
                     brow={+1: "rest", -1: "rest"}, mouth="rest", jaw=0.0,
                     blush=0.0, vein=0.0, sweat=0.0, catch=1.0, plane={+1: 0.0, -1: 0.0})


def state(**kw):
    st = {k: (dict(v) if isinstance(v, dict) else v) for k, v in DEFAULT_STATE.items()}
    for k, v in kw.items():
        if isinstance(v, dict) and isinstance(st.get(k), dict):
            st[k].update(v)
        else:
            st[k] = v
    return st


class Piece(object):
    """name, group, z (draw depth), a painted layer, a mesh and pos(state)."""

    def __init__(self, name, group, z, layer, verts_ref, tris, posfn=None, rest=None,
                 zfn=None):
        self.name = name
        self.group = group
        self.z = float(z)
        self.layer = layer
        self.rgba = layer.finish() if layer is not None else None
        self.bbox = layer.bbox if layer is not None else None
        self.ref = np.asarray(verts_ref, np.float64)
        self.tris = np.asarray(tris, np.int64)
        self.posfn = posfn
        self.rest = self.ref.copy() if rest is None else np.asarray(rest, np.float64)
        self.zfn = zfn
        self.meta = {}
        owner = getattr(posfn, "__self__", None)
        if isinstance(owner, Strip):
            self.meta["strip"] = owner
        # release the supersampled buffers
        if layer is not None:
            layer.col = None
            layer.a = None
            layer._xy = None

    def pos(self, st=None):
        if self.posfn is None:
            return self.rest.copy()
        return np.asarray(self.posfn(state() if st is None else st), np.float64)

    def zs(self):
        if self.zfn is None:
            return np.full(len(self.rest), self.z)
        return self.zfn(self.rest)


def layer_for(name, pts, pad=0.3, tpu=100, ss=3):
    pts = np.asarray(pts, np.float64)
    lo = pts.min(axis=0) - pad
    hi = pts.max(axis=0) + pad
    return P.Layer(name, (lo[0], lo[1], hi[0], hi[1]), tpu=tpu, ss=ss)


# eyes

EYE_TPU = 120
LASH_T_CORNER = D.LID_N / float(D.LID_N + 5)      # fraction of the lash spine at the outer corner


def _eye_state(st, side):
    return st["eye"][side]


def lash_spine(side, s):
    return D.eye_to_model(side, D.lash_spine_uw(s))


def lash_profile(t):
    """(v_lo, v_hi) of the lash mass along the lash spine (v up, mu): a heavy
    mass, thickest over the outer third, whose top edge is flatter than the
    lid (extra thickness at both ends), ending in the winged flick."""
    tc = LASH_T_CORNER
    th = np.interp(t, [0.0, 0.04, 0.12, 0.40, 0.62, 0.80, 0.92, tc, 1.0],
                   [0.10, 0.20, 0.30, 0.33, 0.42, 0.54, 0.60, 0.60, 0.10])
    lo = np.where(t <= tc, -0.06, lerp_arr(-0.06, 0.16, smooth(tc, 1.0, t)))
    hi = np.where(t <= tc, th, lerp_arr(0.60, 0.18, smooth(tc, 1.0, t) ** 0.7))
    return lo, hi


def lerp_arr(a, b, t):
    return a + (b - a) * t


def paint_lash(side):
    spine = lash_spine(side, "open")
    L = layer_for("Lash%d" % side, spine, pad=1.0, tpu=EYE_TPU)
    n = 160
    ts = np.linspace(0, 1, n)
    P0, N = sample_at(spine, ts)
    N = N * side
    lo, hi = lash_profile(ts)
    top = P0 + N * hi[:, None]
    bot = P0 + N * lo[:, None]
    mass = L.poly(np.vstack([top, bot[::-1]]))
    L.paint(mass, D.EYE["lash"])
    # a warm band along the lid edge, hard-edged, middle of the lid only
    k = (ts > 0.22) & (ts < 0.8)
    band_top = P0 + N * (lo + 0.075 * np.sin(np.clip((ts - 0.22) / 0.58, 0, 1) * math.pi))[:, None]
    warm = L.poly(np.vstack([band_top[k], bot[k][::-1]]))
    L.paint_in(warm * mass, D.EYE["lash_warm"], 0.9)
    # lash spikes on the outer quarter, curving up and out
    for t0, length, bend in ((0.70, 0.46, 0.55), (0.86, 0.56, 0.45)):
        p0, n0 = sample_at(spine, np.array([t0]))
        p0 = p0[0]
        n0 = n0[0] * side
        tan = np.array([n0[1], -n0[0]]) * side * side
        tan = np.array([n0[1], -n0[0]])
        if side < 0:
            tan = -tan
        # direction: normal rotated toward the outer corner
        d0 = n0 * math.cos(0.75) + tan * math.sin(0.75)
        d1 = n0 * math.cos(0.75 + bend) + tan * math.sin(0.75 + bend)
        _, hi0 = lash_profile(np.array([t0]))
        start = p0 + n0 * (hi0[0] - 0.08)
        ctrl = [start, start + d0 * length * 0.5, start + (d0 * 0.5 + d1 * 0.5) * length * 0.85,
                start + d0 * length * 0.5 + d1 * length * 0.55]
        sp = resample(catmull(np.array(ctrl), n=12), n=40)
        w = P.taper(40, 0.06, 0.0, head=0.0, tail=0.95, power=0.9)
        L.paint(L.stroke(sp, w, w), D.EYE["lash"])
    # the inner hook (tear duct)
    p0, n0 = sample_at(spine, np.array([0.004]))
    p0 = p0[0]
    n0 = n0[0] * side
    tan = np.array([n0[1], -n0[0]]) * (-1 if side > 0 else 1)
    hook = resample(catmull(np.array([p0 + n0 * 0.05, p0 - n0 * 0.10 + tan * 0.04,
                                      p0 - n0 * 0.26 + tan * 0.02]), n=10), n=24)
    w = P.taper(24, 0.05, 0.0, head=0.0, tail=0.9)
    L.paint(L.stroke(hook, w, w), D.EYE["lash"])
    return L


def paint_lower(side):
    lo = D.lid(side, "open", 1)
    L = layer_for("LowerLid%d" % side, lo, pad=0.6, tpu=EYE_TPU)
    ts = np.linspace(0.40, 0.955, 90)
    P0, N = sample_at(lo, ts)
    w = P.taper(90, 0.050, 0.030, head=0.35, tail=0.18, power=1.0)
    L.paint(L.poly(np.vstack([P0 + N * w[:, None], (P0 - N * w[:, None])[::-1]])), D.EYE["lower"])
    # two tiny lower-lash ticks near the outer end, pointing down and out
    for t0, ln in ((0.80, 0.20), (0.90, 0.16)):
        p, n = sample_at(lo, np.array([t0]))
        p = p[0]
        n = -n[0] * side                       # outward = down
        tan = (np.array([1.0, 0.0]) * side)
        d = n * 0.75 + tan * 0.66
        d /= np.linalg.norm(d)
        sp = resample(np.array([p, p + d * ln]), n=12)
        ww = P.taper(12, 0.030, 0.0, head=0.0, tail=0.95)
        L.paint(L.stroke(sp, ww, ww), D.EYE["lower"])
    return L


def paint_crease(side):
    cr = D.eye_to_model(side, D.crease_uw("open"))
    L = layer_for("Crease%d" % side, cr, pad=0.4, tpu=EYE_TPU)
    ts = np.linspace(0.18, 0.80, 80)
    P0, N = sample_at(cr, ts)
    w = P.taper(80, 0.034, head=0.35, tail=0.35)
    L.paint(L.poly(np.vstack([P0 + N * w[:, None], (P0 - N * w[:, None])[::-1]])), D.EYE["crease"])
    return L


def lens_poly(side, state_name, grow=0.0):
    up = D.lid(side, state_name, 0)
    lo = D.lid(side, state_name, 1)
    c = np.array(D.EYE_C[side])
    poly = np.vstack([up, lo[::-1][1:-1]])
    if grow:
        d = poly - c
        poly = poly + d / (np.linalg.norm(d, axis=1, keepdims=True) + 1e-9) * grow
    return poly


def paint_white(side):
    poly = lens_poly(side, "wide", grow=0.12)
    L = layer_for("White%d" % side, poly, pad=0.3, tpu=EYE_TPU)
    L.paint(L.poly(poly), D.EYE["white"])
    # the tear duct: a small warm wedge at the inner corner
    ic = D.lid(side, "open", 0)[0]
    X, Y = L.XY()
    duct = np.clip(1.0 - np.hypot((X - ic[0] - side * 0.16) / 0.20, (Y - ic[1] + 0.02) / 0.13), 0, 1)
    L.paint_in(np.clip(duct * 6, 0, 1), D.EYE["duct"], 1.0)
    return L


def paint_lidshadow(side):
    up = D.lid(side, "open", 0)
    L = layer_for("LidShadow%d" % side, up, pad=0.8, tpu=EYE_TPU)
    ts = np.linspace(0, 1, 120)
    P0, N = sample_at(up, ts)
    N = N * side
    depth = 0.26 + 0.08 * np.sin(ts * math.pi)
    poly = np.vstack([P0 + N * 0.3, (P0 - N * depth[:, None])[::-1]])
    L.paint(L.poly(poly), D.EYE["lid_shadow"])
    return L


def paint_iris(side):
    c = D.iris_centre(side)
    rx, ry = D.IRIS["rx"], D.IRIS["ry"]
    L = P.Layer("Iris%d" % side, (c[0] - rx - 0.15, c[1] - ry - 0.15, c[0] + rx + 0.15, c[1] + ry + 0.15),
                tpu=EYE_TPU, ss=3)
    X, Y = L.XY()
    xr = (X - c[0]) / rx
    yr = (Y - c[1]) / ry
    disc = L.poly(P.ellipse(c[0], c[1], rx, ry, n=180))
    L.paint(disc, D.EYE["ir_mid"])
    # lower crescent light: the disc minus an ellipse lifted up
    inner = L.poly(P.ellipse(c[0] - 0.04 * side * 0, c[1] + 0.34 * ry, rx * 0.93, ry * 0.93, n=180))
    L.paint_in(disc * (1 - inner), D.EYE["ir_low"], 1.0)
    # top band: dark lid shadow with a curved hard lower edge
    edge = 0.22 - 0.22 * xr ** 2
    band = np.clip((yr - edge) * ry * L.s + 0.5, 0, 1)
    L.paint_in(band * disc, D.EYE["ir_top"], 1.0)
    # pupil
    pc = (c[0] + D.PUPIL["du"], c[1] + D.PUPIL["dw"])
    L.paint_in(L.poly(P.ellipse(pc[0], pc[1], D.PUPIL["rx"], D.PUPIL["ry"], n=120)), D.EYE["pupil"], 1.0)
    # iris ring on the lower half only
    ringw = 0.085 * smooth(0.35, -0.25, yr)
    L.paint_in(L.band_inside(disc, ringw), D.EYE["ring"], 1.0)
    return L


def catch_centre(side, look=(0.0, 0.0)):
    ic = D.iris_centre(side, (look[0] * D.CATCH_FOLLOW, look[1] * D.CATCH_FOLLOW))
    return ic


def paint_catch(side):
    c = catch_centre(side)
    L = P.Layer("Catch%d" % side, (c[0] - 1.0, c[1] - 1.0, c[0] + 1.0, c[1] + 1.0), tpu=EYE_TPU, ss=3)
    # the key light is camera-left: the main catchlight sits low on the
    # screen-left side of BOTH irises; a pinpoint on the upper right
    L.paint(L.poly(P.ellipse(c[0] - 0.36, c[1] - 0.28, 0.27, 0.20, n=60, rot=0.35)), D.EYE["catch"])
    L.paint(L.poly(P.ellipse(c[0] + 0.40, c[1] + 0.50, 0.085, 0.085, n=24)), D.EYE["catch"])
    return L


def eye_pieces(face_tex_piece=None):
    out = []
    for side in (+1, -1):
        tag = "R" if side > 0 else "L"

        Lw = paint_white(side)
        rgba = Lw.finish()
        v, t = grid_from_alpha(rgba, Lw.bbox, 0.18)
        out.append(Piece("White_" + tag, "EyeWhite", 0.10, Lw, v, t))

        Ls = paint_lidshadow(side)
        st = Strip(lambda s, side=side: D.lid(side, _eye_state(s, side), 0),
                   np.linspace(0, 1, 34), [-0.62, -0.3, 0.0, 0.35], up=side)
        out.append(Piece("LidShadow_" + tag, "EyeWhite", 0.11, Ls, st.pos(state()), st.tris(), st.pos))

        Li = paint_iris(side)
        c0 = D.iris_centre(side)
        ang = np.linspace(0, 2 * math.pi, 40, endpoint=False)
        rings = [0.0, 0.35, 0.62, 0.82, 0.94, 1.06, 1.14]
        ivs = [c0]
        for r in rings[1:]:
            ivs += [c0 + np.array([math.cos(a) * D.IRIS["rx"] * r, math.sin(a) * D.IRIS["ry"] * r]) for a in ang]
        ivs = np.array(ivs)
        itri = []
        na = len(ang)
        for k in range(na):
            itri.append((0, 1 + k, 1 + (k + 1) % na))
        for rr in range(len(rings) - 2):
            b0 = 1 + rr * na
            b1 = b0 + na
            for k in range(na):
                a0, a1 = b0 + k, b0 + (k + 1) % na
                c1, d1 = b1 + k, b1 + (k + 1) % na
                itri += [(a0, c1, a1), (a1, c1, d1)]

        def iris_pos(s, side=side, ivs=ivs, c0=c0):
            c = D.iris_centre(side, s["look"])
            pc = c0 + np.array([D.PUPIL["du"], D.PUPIL["dw"]])
            k = 1.0 - 0.58 * s["shrink"]
            p = ivs - c0
            p = (ivs - pc) * k + pc - c0
            return p + c
        out.append(Piece("Iris_" + tag, "Iris", 0.12, Li, ivs, np.array(itri), iris_pos))

        Lc = paint_catch(side)
        rgba = Lc.finish()
        v, t = grid_from_alpha(rgba, Lc.bbox, 0.08, dilate=1)
        cc0 = catch_centre(side)

        def catch_pos(s, side=side, v=v, cc0=cc0):
            c = catch_centre(side, s["look"])
            k = max(0.0, 1.0 - s["shrink"]) * s["catch"]
            return (v - cc0) * k + c
        out.append(Piece("Catch_" + tag, "Catch", 0.13, Lc, v, t, catch_pos))

        Ll = paint_lash(side)
        ts = np.unique(np.concatenate([np.linspace(0, LASH_T_CORNER, 40),
                                       np.linspace(LASH_T_CORNER, 1.0, 8)]))
        st = Strip(lambda s, side=side: lash_spine(side, _eye_state(s, side)), ts,
                   [-0.36, -0.12, 0.08, 0.3, 0.55, 0.85, 1.12], up=side)
        out.append(Piece("Lash_" + tag, "EyeLines", 0.20, Ll, st.pos(state()), st.tris(), st.pos))

        Lo = paint_lower(side)
        st = Strip(lambda s, side=side: D.lid(side, _eye_state(s, side), 1),
                   np.linspace(0.30, 1.0, 26), [-0.34, -0.1, 0.1, 0.28], up=side)
        out.append(Piece("Lower_" + tag, "EyeLines", 0.19, Lo, st.pos(state()), st.tris(), st.pos))

        Lr = paint_crease(side)
        st = Strip(lambda s, side=side: D.eye_to_model(side, D.crease_uw(_eye_state(s, side))),
                   np.linspace(0.08, 0.92, 22), [-0.16, 0.0, 0.16], up=side)
        out.append(Piece("Crease_" + tag, "EyeLines", 0.19, Lr, st.pos(state()), st.tris(), st.pos))
    return out


# brows

def paint_brow(side):
    sp = D.brow_spine(side, "rest")
    L = layer_for("Brow%d" % side, sp, pad=0.5, tpu=100)
    lo, hi = D.brow_widths(200)
    ts = np.linspace(0, 1, 200)
    P0, N = sample_at(sp, ts)
    N = N * side
    top = P0 + N * hi[:, None]
    bot = P0 - N * lo[:, None]
    # squarish head cut on a slant: the top corner sits further in
    head_top = top[0] - (P0[1] - P0[0]) / np.linalg.norm(P0[1] - P0[0]) * 0.10
    poly = np.vstack([head_top[None], top[1:], bot[::-1]])
    m = L.poly(poly)
    L.paint(m, D.BROW["fill"])
    L.paint_in(L.band_inside(m, 0.045), D.BROW["edge"], 1.0)
    # a few darker hair-direction flicks inside the head of the brow
    for k, (t0, dv) in enumerate(((0.05, -0.06), (0.11, 0.07), (0.2, -0.02))):
        a = np.array([t0, t0 + 0.12])
        pp, nn = sample_at(sp, a)
        nn = nn * side
        seg = resample(np.array([pp[0] + nn[0] * dv, pp[1] + nn[1] * (dv + 0.05)]), n=10)
        w = P.taper(10, 0.02, head=0.3, tail=0.6)
        L.paint_in(L.stroke(seg, w, w) * m, D.BROW["edge"], 0.8)
    return L


def brow_pieces():
    out = []
    for side in (+1, -1):
        tag = "R" if side > 0 else "L"
        Lb = paint_brow(side)
        st = Strip(lambda s, side=side: D.brow_spine(side, s["brow"][side]),
                   np.linspace(0, 1, 30), [-0.42, -0.2, 0.0, 0.2, 0.42], up=side)
        out.append(Piece("Brow_" + tag, "Brows", 1.70, Lb, st.pos(state()), st.tris(), st.pos))
    return out


# nose

def nose_pieces():
    nx, ny = D.NOSE_TIP
    L = P.Layer("Nose", (nx - 1.0, ny - 1.0, nx + 1.2, ny + 1.4), tpu=100, ss=3)
    # hard shadow triangle under the tip on the shadow side (screen right)
    tri = np.array([(nx + 0.02, ny - 0.26), (nx + 0.58, ny + 0.02), (nx + 0.44, ny - 0.46)])
    tri = catmull(np.vstack([tri, tri[:1]]), n=6, closed=False)
    L.paint(L.poly(tri), D.SKIN["shadow"])
    # the tip stroke: a short hook on the shadow side
    sp = resample(catmull(np.array([(nx + 0.40, ny + 0.62), (nx + 0.52, ny + 0.18), (nx + 0.38, ny - 0.14),
                                    (nx + 0.06, ny - 0.26)]), n=16), n=60)
    w = P.taper(60, 0.052, 0.036, head=0.45, tail=0.30)
    L.paint(L.stroke(sp, w, w), "#8A4342")
    # nostril tick
    sp = resample(np.array([(nx - 0.48, ny - 0.20), (nx - 0.24, ny - 0.28)]), n=16)
    w = P.taper(16, 0.03, head=0.4, tail=0.5)
    L.paint(L.stroke(sp, w, w), "#A0605A")
    rgba = L.finish()
    v, t = grid_from_alpha(rgba, L.bbox, 0.12)
    return [Piece("Nose", "Nose", 0.40, L, v, t)]


# mouth

MOUTH_TPU = 110
TEETH = 0.40            # teeth band depth below the upper lip (reference layout)
TONGUE = 0.70           # tongue height above the lower lip (reference layout)
INTERIOR_ROWS = np.array([0.0, 0.06, 0.12, 0.155, 0.19, 0.30, 0.45, 0.60, 0.72, 0.84, 0.93, 1.0])


def mouth_state(s):
    return s["mouth"]


def interior_rows(U, L, ref_h):
    """Row positions (MOUTH_N x rows x 2) of the mouth interior between the
    lip curves U and L: the teeth rows stay a fixed depth under the upper lip
    and the tongue rows a fixed height over the lower lip once the mouth is
    open wider than the reference; smaller openings scale everything."""
    h = np.maximum(U[:, 1] - L[:, 1], 0.0)
    # below the reference opening everything scales; above it teeth and
    # tongue grow at 40 % of the opening's rate
    ratio = h / np.maximum(ref_h, 1e-6)
    k = np.where(ratio < 1.0, ratio, 1.0 + 0.4 * (ratio - 1.0))
    out = np.zeros((len(U), len(INTERIOR_ROWS), 2))
    teeth_f = TEETH / 2.2
    tongue_f = TONGUE / 2.2
    for j, f in enumerate(INTERIOR_ROWS):
        if f <= teeth_f + 1e-9:
            d = f * ref_h * k                        # depth below U
            frac = np.where(h > 1e-6, d / np.maximum(h, 1e-6), f)
        elif f >= 1 - tongue_f - 1e-9:
            d = (1 - f) * ref_h * k                  # height above L
            frac = np.where(h > 1e-6, 1 - d / np.maximum(h, 1e-6), f)
        else:
            a = teeth_f * ref_h * k
            b = h - tongue_f * ref_h * k
            g = (f - teeth_f) / (1 - teeth_f - tongue_f)
            frac = np.where(h > 1e-6, (a + (b - a) * g) / np.maximum(h, 1e-6), f)
        frac = np.clip(frac, 0, 1)
        out[:, j, :] = U + (L - U) * frac[:, None]
    return out


def paint_interior():
    U, Lw = D.mouth_curves("ref")
    poly = np.vstack([U, Lw[::-1][1:-1]])
    L = layer_for("MouthInside", poly, pad=0.3, tpu=MOUTH_TPU)
    X, Y = L.XY()
    m = L.poly(poly)
    L.paint(m, D.MOUTH["inside"])
    # deep throat: a hard-edged shape toward the top middle
    th = L.poly(P.ellipse(D.MOUTH_C[0] + 0.15, U[:, 1].max() - 0.85, 1.25, 0.55, n=90))
    L.paint_in(th * m, D.MOUTH["deep"], 1.0)
    # upper teeth band: hangs a fixed depth under the upper lip
    ux = U[:, 0]
    uy = np.interp(X, ux, U[:, 1])
    s = np.clip((X - D.MOUTH_C[0]) / (U[:, 0].max() - D.MOUTH_C[0]), -1, 1)
    depth = TEETH * (1 - 0.35 * np.abs(s) ** 3)
    teeth = np.clip((Y - (uy - depth)) * L.s + 0.5, 0, 1) * smooth(0.95, 0.80, np.abs(s))
    L.paint_in(teeth * m, D.MOUTH["teeth"], 1.0)
    L.paint_in(teeth * m * smooth(0.45, 0.75, np.abs(s)), D.MOUTH["teeth_sh"], 1.0)
    # tongue at the bottom
    ly = np.interp(X, Lw[:, 0], Lw[:, 1])
    bottom = Lw[:, 1].min()
    tongue = L.poly(P.ellipse(D.MOUTH_C[0] - 0.12, bottom + 0.30, 1.35, 0.72, n=120))
    L.paint_in(tongue * m, D.MOUTH["tongue"], 1.0)
    tsh = L.poly(P.ellipse(D.MOUTH_C[0] + 0.28, bottom + 0.22, 1.25, 0.62, n=120))
    L.paint_in(tongue * m * (1 - tsh), D.MOUTH["tongue_sh"], 1.0)
    return L


def mouth_pieces():
    out = []
    ref_U, ref_L = D.mouth_curves("ref")
    ref_h = np.maximum(ref_U[:, 1] - ref_L[:, 1], 0.0)

    Li = paint_interior()
    ref_rows = interior_rows(ref_U, ref_L, ref_h)
    nrow = len(INTERIOR_ROWS)
    tris = grid_tris(D.MOUTH_N, nrow)

    def inside_pos(s):
        U, Lw = D.mouth_curves(mouth_state(s))
        return interior_rows(U, Lw, ref_h).reshape(-1, 2)
    out.append(Piece("Inside", "MouthInside", 0.14, Li, ref_rows.reshape(-1, 2), tris, inside_pos,
                     rest=inside_pos(state())))

    U0, L0 = D.mouth_curves("rest")
    Lt = layer_for("LipTint", L0, pad=0.9, tpu=MOUTH_TPU)
    ts = np.linspace(0.28, 0.72, 100)
    P0, N = sample_at(L0, ts)
    depth = 0.30 * np.sin(np.linspace(0, 1, 100) * math.pi) ** 0.8
    poly = np.vstack([P0 - N * 0.02, (P0 - N * (0.02 + depth[:, None]))[::-1]])
    Lt.paint(Lt.poly(poly), D.SKIN["lip"])
    # small gloss dash
    g0, gn = sample_at(L0, np.array([0.43, 0.53]))
    gsp = resample(g0 - gn * 0.22, n=12)
    gw = P.taper(12, 0.035, head=0.4, tail=0.4)
    Lt.paint(Lt.stroke(gsp, gw, gw), D.SKIN["gloss"])
    # lower-lip stroke and the crescent shadow under it
    ts2 = np.linspace(0.36, 0.64, 60)
    P2, N2 = sample_at(L0, ts2)
    stroke = P2 - N2 * 0.47
    w = P.taper(60, 0.032, head=0.4, tail=0.4)
    Lt.paint(Lt.stroke(stroke, w, w), D.SKIN["lip_line"])
    cres = np.vstack([P2 - N2 * 0.52, (P2 - N2 * (0.52 + 0.13 * np.sin(np.linspace(0, 1, 60) * math.pi))[:, None])[::-1]])
    Lt.paint(Lt.poly(cres), D.SKIN["shadow"])
    st = Strip(lambda s: D.mouth_curves(mouth_state(s))[1], np.linspace(0.12, 0.88, 26),
               [0.12, -0.05, -0.25, -0.45, -0.62, -0.78], up=1.0)
    out.append(Piece("LipTint", "MouthLines", 0.15, Lt, st.pos(state()), st.tris(), st.pos))

    Ll = layer_for("LowerLine", L0, pad=0.4, tpu=MOUTH_TPU)
    ts = np.linspace(0.14, 0.86, 80)
    P0, N = sample_at(L0, ts)
    w = P.taper(80, 0.034, head=0.4, tail=0.4)
    Ll.paint(Ll.poly(np.vstack([P0 + N * w[:, None], (P0 - N * w[:, None])[::-1]])), D.MOUTH["line"])
    st = Strip(lambda s: D.mouth_curves(mouth_state(s))[1], np.linspace(0.08, 0.92, 24),
               [-0.12, 0.0, 0.12], up=1.0)
    out.append(Piece("LowerLine", "MouthLines", 0.16, Ll, st.pos(state()), st.tris(), st.pos))

    Lu = layer_for("MouthLine", U0, pad=0.5, tpu=MOUTH_TPU)
    ts = np.linspace(0.0, 1.0, 120)
    P0, N = sample_at(U0, ts)
    w = 0.018 + 0.042 * np.sin(ts * math.pi) ** 0.7
    Lu.paint(Lu.poly(np.vstack([P0 + N * (w * 0.8)[:, None], (P0 - N * (w * 1.2)[:, None])[::-1]])), D.LINE)
    st = Strip(lambda s: D.mouth_curves(mouth_state(s))[0], np.linspace(0, 1, 30),
               [-0.14, -0.05, 0.05, 0.14], up=1.0)
    out.append(Piece("MouthLine", "MouthLines", 0.17, Lu, st.pos(state()), st.tris(), st.pos))

    for side in (+1, -1):
        tag = "R" if side > 0 else "L"

        def tick_spine(s, side=side):
            U, Lw = D.mouth_curves(mouth_state(s))
            c = U[-1] if side > 0 else U[0]
            p = D.MOUTH_STATES[mouth_state(s)] if isinstance(mouth_state(s), str) else mouth_state(s)
            lift = p["cr"] if side > 0 else p["cl"]
            ang = math.radians(-58 + 150 * lift)
            d0 = np.array([side * math.cos(ang), math.sin(ang)])
            d1 = np.array([side * math.cos(ang - 0.9), math.sin(ang - 0.9)])
            return np.array([c - d0 * 0.05, c + d0 * 0.16, c + d0 * 0.26 + d1 * 0.10])
        sp0 = tick_spine(state())
        Lk = layer_for("Tick" + tag, sp0, pad=0.35, tpu=MOUTH_TPU)
        spd = resample(catmull(sp0, n=10), n=30)
        w = P.taper(30, 0.030, head=0.25, tail=0.7)
        amt = 1.0 if side > 0 else 0.75
        Lk.paint(Lk.stroke(spd, w * amt, w * amt), "#8A4342")
        st = Strip(lambda s, side=side: resample(catmull(tick_spine(s, side), n=10), n=30),
                   np.linspace(0, 1, 8), [-0.1, 0.0, 0.1], up=1.0)
        out.append(Piece("Tick_" + tag, "MouthLines", 0.18, Lk, st.pos(state()), st.tris(), st.pos))
    return out


# eye masks: skin rings that close over the eyeball (clipping without a
# stencil). They sample the FACE texture at their rest positions, so at rest
# they are invisible; their inner edge is the lid opening and moves with every
# eye keyform, their outer edge never moves.

MASK_ROWS = np.array([0.0, 0.04, 0.10, 0.20, 0.34, 0.52, 0.74, 1.0])


def eye_loop(side, state_name):
    up = D.lid(side, state_name, 0)
    lo = D.lid(side, state_name, 1)
    return np.vstack([up, lo[::-1][1:-1]])


def eye_mask_pieces(face_piece):
    out = []
    for side in (+1, -1):
        tag = "R" if side > 0 else "L"
        c = np.array(D.EYE_C[side]) + np.array([0.06 * side, 0.12])
        inner0 = eye_loop(side, "open")
        ang = np.arctan2(inner0[:, 1] - c[1], inner0[:, 0] - c[0])
        outer = np.stack([c[0] + 3.05 * np.cos(ang), c[1] + 2.25 * np.sin(ang)], axis=1)
        n = len(inner0)

        def rows(inner, outer=outer):
            return np.concatenate([inner * (1 - f) + outer * f for f in MASK_ROWS], axis=0)

        rest = rows(inner0)
        tris = []
        for r in range(len(MASK_ROWS) - 1):
            for k in range(n):
                a0 = r * n + k
                a1 = r * n + (k + 1) % n
                b0 = a0 + n
                b1 = a1 + n
                tris += [(a0, b0, a1), (a1, b0, b1)]

        def mask_pos(s, side=side, rows=rows):
            return rows(eye_loop(side, _eye_state(s, side)))
        pc = Piece("EyeMask_" + tag, "EyeMask", 0.15, None, rest, np.array(tris), mask_pos)
        pc.rgba = face_piece.rgba
        pc.bbox = face_piece.bbox
        pc.meta["texture_of"] = face_piece.name
        out.append(pc)
    return out
