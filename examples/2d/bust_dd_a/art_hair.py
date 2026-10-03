"""Kaede's hair: every lock is a CLUMP -- a Catmull-Rom spine with a width
profile and sharp (sometimes forked) tips -- painted with hard cel tones
(base / shadow / deep), a broken highlight dash on the lit crest, tapered
line art and the violet inner colour on the undersides of the flicks. The
same spines carry the FK hair chains, so paint and rig agree on every lock.

Cut: a medium-length layered shag, side part on the screen-right; the
fringe sweeps across to the screen-left with one long clump falling between
the eyes; the screen-right side is tucked behind the ear (piercings show);
a long face-framing lock on the screen-left breaks the jaw line.
"""

import math

import numpy as np

import design as D
import paint as P
from art import Piece, grid_from_alpha, sample_at
from paint import catmull, hexc, resample, smooth

HAIR_TPU = 56
PART = (2.5, 10.9)
LIGHT = np.array([-0.62, 0.78])        # direction TOWARD the key light (screen upper-left)


class Clump(object):
    def __init__(self, name, ctrl, wl, wr=None, z=1.0, group="FrontHair", forks=(), chain=3,
                 joints=None, inner=0.0, hi=True, strands=1, root_open=True, line=1.0,
                 shade=0.22, deep_root=0.0, stiff=1.0, kind="fringe", poly=None):
        self.name = name
        self.poly = None if poly is None else catmull(np.asarray(poly, np.float64), n=12, closed=True)
        self.ctrl = np.asarray(ctrl, np.float64)
        self.spine = resample(catmull(self.ctrl, n=24), n=96)
        t = np.linspace(0, 1, 96)
        a, b = zip(*wl)
        self.wl = np.interp(t, a, b)
        if wr is None:
            self.wr = self.wl.copy()
        else:
            a, b = zip(*wr)
            self.wr = np.interp(t, a, b)
        self.z = z
        self.group = group
        self.forks = list(forks)          # [(t_branch, [ctrl after], [(t, w)])]
        self.chain = chain
        self.joints = joints
        self.inner = inner
        self.hi = hi
        self.strands = strands
        self.root_open = root_open
        self.line = line
        self.shade = shade
        self.deep_root = deep_root
        self.stiff = stiff
        self.kind = kind
        self._polys = None

    def polygons(self):
        if self._polys is not None:
            return self._polys
        polys = [P.ribbon(self.spine, self.wl, self.wr) if self.poly is None else self.poly]
        self.fork_spines = []
        for tb, ctrl, widths in self.forks:
            k = int(round(tb * (len(self.spine) - 1)))
            pts = np.vstack([self.spine[max(0, k - 3):k + 1:3], np.asarray(ctrl, np.float64)])
            sp = resample(catmull(pts, n=16), n=40)
            tt = np.linspace(0, 1, len(sp))
            a, b = zip(*widths)
            w = np.interp(tt, a, b)
            self.fork_spines.append((tb, sp, w))
            polys.append(P.ribbon(sp, w, w))
        self._polys = polys
        return polys

    def bbox(self, pad=0.5):
        pts = np.vstack(self.polygons())
        lo = pts.min(axis=0) - pad
        hi = pts.max(axis=0) + pad
        return (lo[0], lo[1], hi[0], hi[1])

    def joint_ts(self):
        if self.joints is not None:
            return list(self.joints)
        return list(np.linspace(0, 0.78, self.chain)) if self.chain > 1 else [0.0]


def highlight_y(x):
    """The broken highlight sits on one arc around the skull."""
    return 8.1 + 1.6 * np.sqrt(np.clip(1 - (np.asarray(x) / 10.5) ** 2, 0, 1))


def crisp(L, d):
    """Anti-aliased coverage of the region d > 0 (d in model units)."""
    return np.clip(d * L.s + 0.5, 0, 1).astype(np.float32)


def taper_stroke(L, spine, w0, head=0.2, tail=0.6):
    sp = resample(spine, n=max(12, len(spine)))
    w = P.taper(len(sp), w0, head=head, tail=tail)
    return L.stroke(sp, w, w)


def paint_clump(c, above=()):
    tpu = HAIR_TPU if c.kind != "back" else 40
    L = P.Layer(c.name, c.bbox(), tpu=tpu, ss=2)
    X, Y = L.XY()
    polys = c.polygons()
    m = np.zeros(X.shape, np.float32)
    for pl in polys:
        m = np.maximum(m, L.poly(pl))
    region = m > 0.002
    hw = np.maximum((c.wl + c.wr) * 0.5, 1e-3)
    T, V, Dd = P.strand_coords(L, c.spine, hw, region)
    for tb, sp, w in getattr(c, "fork_spines", []):
        Tf, Vf, Df = P.strand_coords(L, sp, w, region)
        closer = np.abs(Df) < np.abs(Dd)
        T = np.where(closer, tb + (1 - tb) * Tf, T)
        V = np.where(closer, Vf, V)
        Dd = np.where(closer, Df, Dd)
    Tc = np.clip(T, 0, 1)
    length = P.arclen(c.spine)[-1]
    # the shadow side: the side of the spine whose normal faces away from the
    # key light
    _, N = sample_at(c.spine, Tc.ravel())
    ndl = (N[:, 0] * LIGHT[0] + N[:, 1] * LIGHT[1]).reshape(T.shape)
    hwt = np.interp(Tc, np.linspace(0, 1, len(hw)), hw)
    s_mu = -Dd * np.sign(ndl + 1e-6)               # + on the shadow side, model units
    base_c, shadow_c = D.HAIR["base"], D.HAIR["shadow"]
    if c.kind == "back" and c.name != "BL1" and c.name != "BR1":
        base_c, shadow_c = D.HAIR["shadow"], D.HAIR["deep"]
    L.paint(m, base_c)
    # cel shadow: a crisp band on the shadow side; its edge steps in toward
    # the tip in a couple of teeth (the clump's twist)
    phase = (sum(map(ord, c.name)) % 17) / 17.0
    teeth = P.sawtooth(Tc * (3.0 + 2.0 * (c.kind != "back")) + phase, 1.0, 0.8)
    edge = hwt * (0.62 - c.shade) - 0.22 * teeth * np.minimum(hwt, 1.2) - 0.35 * hwt * smooth(0.7, 1.0, Tc)
    L.paint_in(crisp(L, s_mu - edge) * region, shadow_c, 1.0)
    # cast shadows of the clumps lying above this one (down-right of them)
    for pl in above:
        sh = L.poly(pl + np.array([0.22, -0.36]))
        L.paint_in(sh * region, D.HAIR["shadow"], 1.0)
        L.paint_in(L.poly(pl + np.array([0.08, -0.14])) * region, D.HAIR["deep"], 1.0)
    # deep tone: roots (hard band) and the far edge of the shadow side
    L.paint_in(crisp(L, c.deep_root * length - Tc * length) * (T >= 0), D.HAIR["deep"], 1.0)
    L.paint_in(crisp(L, s_mu - hwt * 0.80) * crisp(L, (Tc - 0.18) * length) * crisp(L, (0.82 - Tc) * length) * region,
               D.HAIR["deep"], 1.0)
    # inner colour on the underside of the flicked ends
    if c.inner > 0:
        diag = (Tc - (1.0 - c.inner)) * length + 0.9 * (s_mu / np.maximum(hwt, 1e-3))
        tip = crisp(L, diag)
        under = crisp(L, s_mu + hwt * 0.05 - 0.10 * np.sin(Tc * 23.0))
        L.paint_in(tip * under * region, D.HAIR["inner"], 1.0)
        L.paint_in(tip * crisp(L, s_mu - hwt * 0.50) * region, D.HAIR["inner_sh"], 1.0)
        L.paint_in(crisp(L, diag - 1.0) * crisp(L, -(s_mu - hwt * 0.2)) * crisp(L, s_mu + hwt * 0.05) * region,
                   D.HAIR["inner_hi"], 1.0)
    rng = np.random.default_rng(sum(map(ord, c.name)))
    # interior strand lines: from near the tip back toward the root, tapered
    for k in range(c.strands):
        v0 = rng.uniform(-0.45, 0.2)
        t0 = rng.uniform(0.25, 0.45)
        t1 = rng.uniform(0.80, 0.93)
        ts = np.linspace(t0, t1, 40)
        p, n = sample_at(c.spine, ts)
        hwk = np.interp(ts, np.linspace(0, 1, len(hw)), hw)
        sp = p + n * (v0 * hwk)[:, None]
        L.paint_in(taper_stroke(L, sp, 0.030, head=0.7, tail=0.25) * region, D.HAIR["line"], 1.0)
    # broken highlight: 1-2 short tapered dashes where the lock crosses the
    # skull arc, on its lit half, following the lock
    if c.hi:
        ts = np.linspace(0, 1, 200)
        p, n = sample_at(c.spine, ts)
        dy = p[:, 1] - highlight_y(p[:, 0])
        k = np.nonzero(np.diff(np.sign(dy)) != 0)[0]
        for kk in k[:1]:
            tc = ts[kk]
            hwc = np.interp(tc, np.linspace(0, 1, len(hw)), hw)
            _, nn = sample_at(c.spine, np.array([tc]))
            ndl0 = nn[0] @ LIGHT
            lit_side = 1.0 if ndl0 > 0 else -1.0
            for j, (off, ln, wd) in enumerate(((0.38, 0.95, 0.11), (0.72, 0.55, 0.08))):
                if j and rng.uniform() < 0.35:
                    continue
                dt = ln / max(length, 1e-3)
                tt = np.linspace(tc - dt * 0.5, tc + dt * 0.5, 16) + (0.02 * j)
                pp, nn2 = sample_at(c.spine, np.clip(tt, 0, 1))
                sp = pp + nn2 * (lit_side * off * hwc)
                L.paint_in(taper_stroke(L, sp, wd, head=0.45, tail=0.45) * region, D.HAIR["hi"], 1.0)
    # line art inside the silhouette: thinning to the tip, open at the root
    lw = (0.080 * (1 - 0.6 * smooth(0.5, 1.0, Tc)) + 0.022) * c.line
    if c.root_open:
        lw = lw * smooth(0.02, 0.10, Tc)
    L.paint_in(L.band_inside(m, lw) * region, D.HAIR["line"], 1.0)
    L.meta.update(spine=c.spine)
    return L


# the cut

def W(*pairs):
    return list(pairs)


def clumps():
    C = []
    px, py = PART
    C.append(Clump("BackCrown", [(0.0, 12.2), (0.0, 6.0), (0.0, -2.0), (0.0, -10.0)],
                   W((0, 9.6), (1.0, 7.5)), z=-2.10, group="BackHair",
                   chain=1, hi=False, strands=0, root_open=False, kind="back", line=0.9, inner=0.0,
                   deep_root=0.0, shade=0.2,
                   poly=[(0.0, 12.35), (5.8, 11.75), (8.8, 9.3), (9.4, 5.2), (9.5, 0.0), (9.1, -6.0),
                         (8.0, -11.5), (0.0, -12.5), (-8.0, -11.5), (-9.3, -6.0), (-9.7, 0.0),
                         (-9.6, 5.2), (-8.9, 9.3), (-5.8, 11.75)]))
    back = [
        ("BL1", [(-8.6, 8.6), (-10.0, 3.0), (-10.1, -3.0), (-10.6, -8.5), (-12.0, -12.2), (-13.9, -13.6)], 1.7, -1.98, [0, .35, .62, .84]),
        ("BL2", [(-7.6, 3.0), (-9.1, -3.5), (-9.4, -9.5), (-10.3, -13.8), (-11.9, -16.2)], 1.5, -1.96, [0, .38, .66, .86]),
        ("BL3", [(-5.6, -3.0), (-7.2, -8.6), (-7.9, -12.8), (-9.0, -15.6)], 1.2, -1.94, [0, .5, .8]),
        ("BR1", [(8.6, 8.6), (9.7, 3.0), (9.9, -3.0), (10.3, -8.0), (11.6, -11.4), (13.3, -12.4)], 1.6, -1.97, [0, .35, .62, .84]),
        ("BR2", [(7.4, 1.0), (8.8, -4.5), (9.1, -9.8), (10.2, -13.2), (11.6, -14.4)], 1.35, -1.95, [0, .4, .72]),
        ("BR3", [(5.8, -3.5), (7.1, -8.8), (7.8, -12.4), (8.8, -14.6)], 1.1, -1.93, [0, .55]),
    ]
    for name, ctrl, w, z, joints in back:
        C.append(Clump(name, ctrl, W((0, w * 0.7), (0.3, w), (0.65, w * 0.9), (0.86, w * 0.6), (1.0, 0.0)),
                       z=z, group="BackHair", chain=len(joints), joints=joints, inner=0.26, hi=False,
                       strands=1, kind="back", shade=0.34, deep_root=0.0))
    C.append(Clump("Cap_L", [(px - 0.2, py + 0.25), (-2.5, 12.0), (-6.9, 10.3), (-9.2, 6.6), (-9.7, 3.2)],
                   W((0, 1.0), (0.12, 2.5), (0.5, 2.35), (0.8, 1.4), (1.0, 0.0)), z=0.80, group="FrontHair",
                   chain=1, hi=True, strands=2, deep_root=0.0, kind="cap"))
    C.append(Clump("Cap_R", [(px + 0.3, py + 0.25), (6.0, 11.1), (8.5, 8.7), (9.3, 5.4), (9.0, 2.2)],
                   W((0, 1.0), (0.15, 2.0), (0.5, 1.85), (0.82, 1.2), (1.0, 0.0)), z=0.82, group="FrontHair",
                   chain=1, hi=True, strands=1, deep_root=0.0, kind="cap"))
    C.append(Clump("Lock_L", [(-7.2, 7.6), (-8.5, 3.0), (-8.9, -2.8), (-8.6, -8.4), (-9.0, -12.6), (-10.6, -16.0)],
                   W((0, 1.3), (0.15, 1.55), (0.5, 1.45), (0.8, 1.15), (0.93, 0.7), (1.0, 0.0)),
                   z=0.95, group="SideHair", chain=4, joints=[0.0, 0.3, 0.56, 0.8], inner=0.18, strands=2,
                   forks=[(0.80, [(-8.1, -13.6), (-7.6, -15.5)], W((0, 0.55), (1, 0.0)))], kind="lock"))
    C.append(Clump("LockIn_L", [(-6.2, 8.0), (-7.1, 3.5), (-7.3, -0.6), (-6.8, -4.4)],
                   W((0, 0.9), (0.4, 0.95), (0.8, 0.6), (1.0, 0.0)),
                   z=0.97, group="SideHair", chain=3, joints=[0.0, 0.4, 0.72], strands=1, kind="lock"))
    C.append(Clump("Strand_R", [(7.7, 5.6), (8.45, 2.6), (8.55, -0.6), (8.1, -3.5)],
                   W((0, 0.30), (0.5, 0.27), (0.85, 0.16), (1.0, 0.0)),
                   z=0.93, group="SideHair", chain=3, joints=[0.0, 0.4, 0.72], strands=0, hi=False,
                   line=0.8, kind="lock"))
    C.append(Clump("Fringe_3", [(-3.2, 10.9), (-6.3, 8.6), (-7.8, 5.6), (-8.35, 2.9)],
                   W((0, 1.25), (0.5, 1.3), (0.8, 0.95), (1.0, 0.0)), z=1.10, chain=3, strands=1, kind="fringe"))
    C.append(Clump("Fringe_5", [(5.4, 10.4), (7.2, 8.0), (7.9, 5.4), (7.6, 3.4)],
                   W((0, 1.05), (0.5, 1.1), (0.8, 0.8), (1.0, 0.0)), z=1.11, chain=3, strands=0, kind="fringe"))
    C.append(Clump("Fringe_4", [(3.5, 10.9), (4.5, 8.5), (5.05, 5.6), (4.85, 2.8)],
                   W((0, 1.3), (0.5, 1.35), (0.8, 0.95), (1.0, 0.0)), z=1.12, chain=3, strands=1, kind="fringe"))
    C.append(Clump("Fringe_2", [(-0.6, 11.0), (-3.4, 8.8), (-5.4, 5.8), (-6.3, 3.0), (-6.05, 1.8)],
                   W((0, 1.75), (0.5, 1.8), (0.78, 1.3), (1.0, 0.0)), z=1.14, chain=3, strands=1,
                   forks=[(0.70, [(-4.6, 3.8), (-4.55, 2.6)], W((0, 0.5), (1, 0.0)))], kind="fringe"))
    C.append(Clump("Fringe_1", [(1.8, 10.9), (-0.5, 8.6), (-2.3, 5.6), (-3.25, 2.6), (-3.1, 1.2)],
                   W((0, 2.2), (0.55, 2.15), (0.8, 1.45), (1.0, 0.0)), z=1.18, chain=3, strands=2,
                   forks=[(0.68, [(-1.5, 3.5), (-1.25, 2.2)], W((0, 0.6), (1, 0.0)))], kind="fringe"))
    C.append(Clump("Fringe_Long", [(2.4, 10.8), (1.4, 7.2), (0.4, 3.6), (0.3, 0.4), (0.7, -1.1)],
                   W((0, 1.0), (0.3, 0.9), (0.7, 0.55), (0.9, 0.3), (1.0, 0.0)), z=1.22, chain=3,
                   joints=[0.0, 0.42, 0.74], strands=1, kind="fringe"))
    C.append(Clump("Stray_Top", [(0.6, 11.9), (0.9, 13.2), (2.0, 13.7), (2.7, 13.0)],
                   W((0, 0.16), (0.6, 0.12), (1.0, 0.0)), z=0.78, chain=2, joints=[0.0, 0.5], strands=0,
                   hi=False, line=0.9, kind="stray"))
    C.append(Clump("Stray_L", [(-8.4, 9.0), (-10.0, 7.2), (-10.9, 4.6), (-11.5, 3.2)],
                   W((0, 0.13), (0.6, 0.10), (1.0, 0.0)), z=0.79, chain=2, joints=[0.0, 0.5], strands=0,
                   hi=False, line=0.9, kind="stray"))
    return C


def hair_pieces(only=None):
    out = []
    cl = clumps()
    for c in cl:
        if only and c.name not in only:
            continue
        above = []
        for o in cl:
            if o is not c and o.group == c.group and o.z > c.z and o.kind not in ("stray", "back"):
                above += o.polygons()
        L = paint_clump(c, above)
        rgba = L.finish()
        v, t = grid_from_alpha(rgba, L.bbox, 0.34 if c.kind != "back" else 0.55)
        pc = Piece("Hair_" + c.name, c.group, c.z, L, v, t)
        pc.rgba = rgba
        pc.meta["clump"] = c
        out.append(pc)
    return out


def face_shadow_polys():
    """Cast shadows of the hair on the skin (painted into the face): the
    fringe tips offset down-right, the side lock offset right."""
    fr, lk = [], []
    for c in clumps():
        if c.kind == "fringe" or c.name.startswith("Cap"):
            for pl in c.polygons():
                fr.append(pl + np.array([0.28, -0.42]))
        elif c.name in ("Lock_L", "LockIn_L"):
            for pl in c.polygons():
                lk.append(pl + np.array([0.32, -0.20]))
    return fr, lk
