#!/usr/bin/env python
"""Limb flip detector for Pip's rubber-hose arms and legs.

    source bin/_env.sh
    "$PY" examples/2d/rubberhose/check_limbs.py                 # the anim layer
    "$PY" examples/2d/rubberhose/check_limbs.py <stage.usda> --step 0.5 \
        --json out.json --plot out.png

Evaluates the stage with `build/rigExecPose --joints-out` (the rig's own
answer; nothing is re-implemented here) and measures, per sample and limb:

  * elbow/knee side: the signed distance of the two-bone IK's mid joint from
    the root->end line, as a fraction of that line's length.  Positive is
    the side the rest pole picks (elbows and knees bow OUT at rest);
  * bend: the angle between the IK's upper and lower bones (0 = straight);
  * hose bow: the signed mean offset of the 14 hose joints from the same
    line (fraction of its length);
  * hose shape: the turning angle between consecutive hose segments.  A
    KINK is one turn sharper than --kink degrees (a hook at a wrist or a
    hip); an S-CURVE is more than --s-turn degrees of turning against the
    hose's overall bend;
  * hose frames: every hose joint's frame must stay a rotation about the
    picture's Z axis -- its visible width (1 = full, 0 = edge-on) and
    neighbouring frames whose widths point opposite ways (a pinch).

A FLIP is a change of sign of the hose bow (or of the elbow side) outside
the dead zone |value| < --threshold.  Its DURATION is the time the limb
takes to straighten and re-curve: from the last sample still clearly bowed
(|value| >= --clear) on the old side to the first clearly bowed on the new
side.  A flip is INTENDED when it lies inside a window that the anim layer
declares in its customLayerData:

    "rubberhose:intendedBendChanges" = { "Arm_L": [57, 63, 85, 92] }

(pairs of frames).  A POP is an acceleration spike of the bow or elbow side
(|second difference| > --pop, chord fractions per frame^2) -- a snap, as
opposed to fast but smooth motion.  A spike within --contact-tol frames of
a foot contact the layer declares ("rubberhose:footContacts", per leg) is
the intended impact of a planted foot: it is listed, not failed.

CLEARANCE (staging) is measured on the rig's deformed meshes (a second
evaluation with `rigExecPose --pose-out`; skip it with --no-clearance).
Every shape is the convex hull of its evaluated points, so ink outlines
count, and a hose joint under a glove or its epaulette is hidden:

  * ARM OVER HEAD: a visible arm-hose joint whose ink (ARM_W / 2 + OL from
    the joint) overlaps the head -- an arm cutting across the face;
  * TANGENT: a hose running along the head outline -- at least 3
    consecutive visible joints (not counting the 2 where the hose leaves
    the shoulder and the 2 where it enters the glove) with less than
    --tangent units of background between arm ink and head ink, for longer
    than --tangent-frames frames (a hand passing the head may touch it);
  * GLOVE ON SHOE: a glove touching a shoe;
  * ARM ON LEG: 4 or more visible arm-hose joints within --merge units
    (ink to ink) of the same side's leg hose -- arm and leg read as one
    black mass;

INK MERGE (a raster check on the same meshes, skip it with --no-ink): the
picture is rasterised (--res units per pixel) with the renderer's draw
order (painter's algorithm on face depth) and per-face displayColor, so
what each arm is drawn OVER is known exactly.  The black 'body' is the
union of the Body mesh's black faces (pants fill, the jacket's ink outline,
belt, bowtie) and the leg hoses; gloves and epaulettes drawn over an arm
hide it.  Per sample and arm:

  * INK MERGE: visible arm-hose ink touching the black body (overlapping
    it, or closer than --ink-gap) over more than --ink-area square units,
    not counting the --root-zone around the shoulder where the hose
    leaves the epaulette -- a black hose on black pants or on the body
    outline has no ink separation: the arm vanishes into the body;
  * WRIST LOST: more than --wrist-dark of the visible hose within
    --wrist-r of the wrist joint (the last ~0.2 before the glove cuff)
    lies over black body fill -- the glove looks cut off from its arm;
  * GLOVE ON LEG: more than --glove-leg square units of a glove drawn
    over a leg hose (ink included) -- a hand parked on a knee or thigh.

Either one lasting longer than --ink-frames frames fails.

Exit status 1 on any unintended flip, flip faster than --min-frames, kink,
S-curve, pinched or edge-on strip, pop or clearance failure.
"""
import argparse
import json
import math
import os
import subprocess
import sys

from pxr import Sdf, Usd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.normpath(os.path.join(HERE, "..", "..", ".."))
DEFAULT_STAGE = os.path.join(HERE, "rubberhose_anim.usda")
LIMBS = ("Arm_L", "Arm_R", "Leg_L", "Leg_R")
HOSE_N = 14
LOOP_LEN = 120.0
# rest geometry (build_rubberhose.py): root, end and pole of every limb
REST = {
    "Arm_L": ((0.90, 5.02), (1.92, 2.98), (3.40, 5.10)),
    "Arm_R": ((-0.90, 5.02), (-1.92, 2.98), (-3.40, 5.10)),
    "Leg_L": ((0.50, 3.30), (1.00, 0.74), (2.90, 2.30)),
    "Leg_R": ((-0.50, 3.30), (-1.00, 0.74), (-2.90, 2.30)),
}


def cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def turn(a, b):
    """Signed angle (deg) from direction a to direction b."""
    return math.degrees(math.atan2(cross2(a, b), dot(a, b)))


def rest_sign(limb):
    r, e, p = REST[limb]
    return 1.0 if cross2(sub(e, r), sub(p, r)) > 0 else -1.0


def export_joints(stage, frames, out):
    exe = os.path.join(REPO, "build", "rigExecPose.exe")
    if not os.path.exists(exe):
        exe = os.path.join(REPO, "build", "rigExecPose")
    cmd = [exe, stage, "--frames", ",".join("%g" % f for f in frames),
           "--joints-out", out]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         universal_newlines=True)
    if res.returncode != 0:
        sys.stdout.write(res.stdout[-4000:])
        raise SystemExit("rigExecPose failed (%d)" % res.returncode)
    return res.stdout


def load_joints(path):
    st = Usd.Stage.Open(path)
    prim = st.GetPrimAtPath("/RigExecJoints")
    paths = [str(p) for p in prim.GetAttribute("rigExec:jointPaths").Get()]
    attr = prim.GetAttribute("rigExec:jointTransforms")
    times = attr.GetTimeSamples()
    leaf = {p.rsplit("/", 1)[-1]: i for i, p in enumerate(paths)}
    return leaf, times, {t: attr.Get(t) for t in times}


def analyse(leaf, times, data):
    out = {}
    for limb in LIMBS:
        kind, side = limb.split("_")
        ik = [leaf["%sIk_%s_%s" % (kind, side, n)]
              for n in ("Root", "Mid", "End")]
        hose = [leaf["%sHose_%s_%02d" % (kind, side, i)]
                for i in range(HOSE_N)]
        ref = rest_sign(limb)
        rows = []
        for t in times:
            m = data[t]
            P = lambda k: (m[k][3][0], m[k][3][1])  # noqa: E731
            R, M, E = P(ik[0]), P(ik[1]), P(ik[2])
            H = [P(k) for k in hose]
            c = sub(H[-1], H[0])
            L = math.hypot(*c)
            n = (-c[1] / L, c[0] / L) if L > 1e-9 else (0.0, 0.0)
            elbow = ref * dot(sub(M, R), n) / max(L, 1e-9)
            u, v = sub(M, R), sub(E, M)
            bend = 0.0
            if math.hypot(*u) > 1e-9 and math.hypot(*v) > 1e-9:
                bend = abs(turn(u, v))
            offs = [ref * dot(sub(h, H[0]), n) / max(L, 1e-9) for h in H]
            bow = sum(offs[1:-1]) / (HOSE_N - 2)
            # turning along the hose (positive = toward the rest bow side)
            segs = [sub(H[i + 1], H[i]) for i in range(HOSE_N - 1)]
            turns = [-ref * turn(segs[i], segs[i + 1])
                     for i in range(len(segs) - 1)]
            total = sum(turns)
            against = sum(abs(a) for a in turns
                          if a * total < 0) if abs(total) > 1e-9 else 0.0
            kink = max(abs(a) for a in turns)
            # visible width of every hose frame, and pinches between frames
            wmin, pinch, prev_y = 9.0, 0, None
            for k in hose:
                x = (m[k][0][0], m[k][0][1])
                y = (m[k][1][0], m[k][1][1])
                ly3 = math.sqrt(sum(m[k][1][i] ** 2 for i in range(3)))
                lx = math.hypot(*x)
                w = abs(cross2(x, y)) / max(lx * ly3, 1e-12) if lx > 1e-9 \
                    else 0.0
                wmin = min(wmin, w)
                if prev_y is not None and dot(prev_y, y) < 0:
                    pinch += 1
                prev_y = y
            rows.append({"t": t, "elbow": elbow, "bend": bend, "bow": bow,
                         "chord": L, "kink": kink, "against": against,
                         "turn": total, "width": wmin, "pinch": pinch})
        out[limb] = rows
    return out


def sign_of(v, thr):
    return 1 if v >= thr else (-1 if v <= -thr else 0)


def flips(rows, key, thr, clear):
    """Sign changes of rows[key] outside the dead zone; each with the time
    from the last clearly-bowed sample on the old side to the first on the
    new side."""
    ev = []
    last_sign, last_i = 0, None
    for i, r in enumerate(rows):
        s = sign_of(r[key], thr)
        if s == 0:
            continue
        if last_sign and s != last_sign:
            a = last_i
            while a > 0 and last_sign * rows[a][key] < clear:
                a -= 1
            b = i
            while b < len(rows) - 1 and s * rows[b][key] < clear:
                b += 1
            ev.append({"from": rows[a]["t"], "to": rows[b]["t"],
                       "cross": 0.5 * (rows[last_i]["t"] + r["t"]),
                       "frames": rows[b]["t"] - rows[a]["t"],
                       "dir": "%+d->%+d" % (last_sign, s)})
        last_sign, last_i = s, i
    return ev


def intended(ev, windows):
    return any(ev["from"] >= a - 1e-6 and ev["to"] <= b + 1e-6
               for a, b in windows)


# -- clearance ----------------------------------------------------------------

ARM_HW = 0.25 / 2 + 0.075        # build_rubberhose.py: ARM_W / 2 + OL
LEG_HW = 0.27 / 2 + 0.075        # LEG_W / 2 + OL
CLEAR_MESHES = ("Head", "Glove_L", "Glove_R", "Epaulet_L", "Epaulet_R",
                "Shoe_L", "Shoe_R")


def export_pose(stage, frames, out):
    exe = os.path.join(REPO, "build", "rigExecPose.exe")
    if not os.path.exists(exe):
        exe = os.path.join(REPO, "build", "rigExecPose")
    cmd = [exe, stage, "--frames", ",".join("%g" % f for f in frames),
           "--pose-out", out]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         universal_newlines=True)
    if res.returncode != 0:
        sys.stdout.write(res.stdout[-4000:])
        raise SystemExit("rigExecPose --pose-out failed (%d)" % res.returncode)


def iter_pose(path, full=False):
    """Yield (frame, hose joint positions by leaf, mesh points by name) from
    a --pose-out dump, one frame at a time.  Points are 2D (the picture
    plane) for the CLEAR_MESHES; with full=True every Geom mesh is also
    returned as an (N, 3) numpy array under ('3d', name)."""
    if full:
        import numpy as np
    cur_t, cur = None, None
    with open(path) as fh:
        for line in fh:
            if line.startswith("frame "):
                if cur is not None:
                    yield cur_t, cur[0], cur[1]
                cur_t, cur = float(line.split()[1]), ({}, {})
            elif line.startswith("jointFramesFinal "):
                parts = line.split(None, 4)
                leaf = parts[1].rsplit("/", 1)[-1]
                if "Hose_" in leaf:
                    cur[0][leaf] = (float(parts[2]), float(parts[3]))
            elif line.startswith("movedProperties /Pip/Geom/"):
                parts = line.split(" ", 3)
                name = parts[1][len("/Pip/Geom/"):]
                if not name.endswith(".points"):
                    continue
                name = name[:-7]
                if name not in CLEAR_MESHES and not full:
                    continue
                v = parts[3].split()
                if full:
                    a = np.array(v, dtype=float).reshape(-1, 3)
                    cur[1][("3d", name)] = a
                    if name in CLEAR_MESHES:
                        cur[1][name] = [tuple(p) for p in a[:, :2].tolist()]
                else:
                    cur[1][name] = [(float(v[i]), float(v[i + 1]))
                                    for i in range(0, len(v), 3)]
    if cur is not None:
        yield cur_t, cur[0], cur[1]


def load_pose(path):
    """{frame: (hose joint positions by leaf, mesh points by name)}."""
    return {t: (j, m) for t, j, m in iter_pose(path)}


def hull(points):
    """Convex hull, counter-clockwise (monotone chain)."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross2(sub(out[-1], out[-2]),
                                           sub(p, out[-2])) <= 0:
                out.pop()
            out.append(p)
        return out
    lo, hi = half(pts), half(reversed(pts))
    return lo[:-1] + hi[:-1]


def seg_dist(p, a, b):
    ab = sub(b, a)
    L2 = dot(ab, ab)
    t = 0.0 if L2 < 1e-18 else max(0.0, min(1.0, dot(sub(p, a), ab) / L2))
    q = (a[0] + ab[0] * t, a[1] + ab[1] * t)
    return math.hypot(p[0] - q[0], p[1] - q[1])


def inside(p, poly):
    return all(cross2(sub(poly[(i + 1) % len(poly)], poly[i]),
                      sub(p, poly[i])) >= 0 for i in range(len(poly)))


def signed_dist(p, poly):
    """Distance from p to a convex CCW polygon's outline, negative inside."""
    d = min(seg_dist(p, poly[i], poly[(i + 1) % len(poly)])
            for i in range(len(poly)))
    return -d if inside(p, poly) else d


def poly_dist(a, b):
    """0 when two convex polygons touch or overlap, else their distance."""
    if any(inside(p, b) for p in a) or any(inside(p, a) for p in b):
        return 0.0
    d = min(seg_dist(p, b[i], b[(i + 1) % len(b)])
            for p in a for i in range(len(b)))
    return min(d, min(seg_dist(p, a[i], a[(i + 1) % len(a)])
                      for p in b for i in range(len(a))))


def polyline_dist(p, line):
    return min(seg_dist(p, line[i], line[i + 1])
               for i in range(len(line) - 1))


def clearance(frames, tangent, merge):
    """Per sample and arm: head gap (ink to ink, visible joints), the
    longest tangent run along the head, glove-to-shoe distance and the
    number of arm joints merged with the leg."""
    return [clearance_row(t, frames[t][0], frames[t][1], tangent, merge)
            for t in sorted(frames)]


def clearance_row(t, joints, meshes, tangent, merge):
    if True:
        H = {k: hull(meshes[k]) for k in CLEAR_MESHES}
        row = {"t": t}
        for s in ("L", "R"):
            arm = [joints["ArmHose_%s_%02d" % (s, i)] for i in range(HOSE_N)]
            leg = [joints["LegHose_%s_%02d" % (s, i)] for i in range(HOSE_N)]
            covers = (H["Glove_L"], H["Glove_R"], H["Epaulet_" + s])
            vis = [not any(inside(p, c) for c in covers) for p in arm]
            gaps = [signed_dist(p, H["Head"]) - ARM_HW for p in arm]
            idx = [i for i in range(HOSE_N) if vis[i]]
            row["head_" + s] = min(gaps[i] for i in idx) if idx else 9.0
            run = best = 0
            for i in idx[2:-2]:
                run = run + 1 if 0.0 <= gaps[i] < tangent else 0
                best = max(best, run)
            row["tangent_" + s] = best
            row["shoe_" + s] = min(poly_dist(H["Glove_" + s], H[k])
                                   for k in ("Shoe_L", "Shoe_R"))
            row["merge_" + s] = sum(
                1 for i in idx
                if polyline_dist(arm[i], leg) - ARM_HW - LEG_HW < merge)
            # glove (hull, ink included) to either leg hose's ink
            g = H["Glove_" + s]
            gap = 9.0
            for side in ("L", "R"):
                lg = [joints["LegHose_%s_%02d" % (side, i)]
                      for i in range(HOSE_N)]
                if any(inside(p, g) for p in lg):
                    gap = min(gap, -LEG_HW)
                    continue
                d = min(min(polyline_dist(p, lg) for p in g),
                        min(seg_dist(q, g[i], g[(i + 1) % len(g)])
                            for q in lg for i in range(len(g))))
                gap = min(gap, d - LEG_HW)
            row["gleg_" + s] = gap
        return row


# -- ink merge (raster) --------------------------------------------------------

BLACK_BODY = ("Body", "Leg_L", "Leg_R")   # meshes whose black faces count
DARK = 0.2                                 # max(displayColor) below = black


class Topology(object):
    """Faces, per-face 'black' flags and rest points of every Geom mesh."""

    def __init__(self, stage_path):
        import numpy as np
        from pxr import UsdGeom
        st = Usd.Stage.Open(stage_path)
        self.meshes = {}
        for prim in st.GetPrimAtPath("/Pip/Geom").GetChildren():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            m = UsdGeom.Mesh(prim)
            if m.ComputeVisibility() == UsdGeom.Tokens.invisible:
                continue
            counts = list(m.GetFaceVertexCountsAttr().Get())
            idx = list(m.GetFaceVertexIndicesAttr().Get())
            cols = UsdGeom.PrimvarsAPI(m).GetPrimvar("displayColor").Get()
            nmax = max(counts)
            F = np.full((len(counts), nmax), -1, dtype=int)
            k = 0
            for i, c in enumerate(counts):
                F[i, :c] = idx[k:k + c]
                F[i, c:] = idx[k + c - 1]        # pad with the last vertex
                k += c
            black = np.array([max(cols[i]) < DARK if len(cols) == len(counts)
                              else False for i in range(len(counts))])
            pts = np.array([tuple(p) for p in m.GetPointsAttr().Get()],
                           dtype=float)
            self.meshes[prim.GetName()] = (F, np.array(counts), black, pts)


def _faces(topo, meshes, names):
    """Per face: mean depth, 2D polygon, bbox, black flag, mesh name."""
    import numpy as np
    out = []
    for name in names:
        F, counts, black, rest = topo.meshes[name]
        P = meshes.get(("3d", name), rest)
        V = P[F]                                    # (nf, nmax, 3)
        z = V[:, :, 2].mean(axis=1)
        lo = V[:, :, :2].min(axis=1)
        hi = V[:, :, :2].max(axis=1)
        out.append((name, z, V[:, :, :2], lo, hi, counts, black))
    return out


def _draw(img_draw, x0, y1, res, faces, box, label_fn, zsel):
    """Paint the faces selected by zsel(z) into img_draw in depth order,
    culled to box = (xmin, ymin, xmax, ymax)."""
    import numpy as np
    items = []
    for name, z, V, lo, hi, counts, black in faces:
        keep = ((hi[:, 0] >= box[0]) & (lo[:, 0] <= box[2]) &
                (hi[:, 1] >= box[1]) & (lo[:, 1] <= box[3]) & zsel(z))
        for i in np.nonzero(keep)[0]:
            items.append((z[i], name, i, V[i, :counts[i]], black[i]))
    items.sort(key=lambda it: it[0])
    for _, name, i, poly, blk in items:
        pts = [((x - x0) / res, (y1 - y) / res) for x, y in poly]
        img_draw.polygon(pts, fill=label_fn(name, blk))


def ink_row(t, joints, meshes, topo, res, gap, root_zone, wrist_r):
    """Per arm: area (square units) of visible arm ink touching the black
    body outside the shoulder zone, the fraction of the wrist's visible
    hose lying over black body fill, and the arm-over-other-arm area."""
    import numpy as np
    from PIL import Image, ImageDraw
    names = list(topo.meshes)
    faces = _faces(topo, meshes, names)
    zr = {n: (float(z.min()), float(z.max()))
          for n, z, _, _, _, _, _ in faces if n in ("Arm_L", "Arm_R")}
    row = {"t": t}
    for s in ("L", "R"):
        arm = "Arm_" + s
        a = [f for f in faces if f[0] == arm][0]
        lo, hi = a[3].min(axis=0), a[4].max(axis=0)
        m = 3 * res + gap
        box = (lo[0] - m, lo[1] - m, hi[0] + m, hi[1] + m)
        x0, y1 = box[0], box[3]
        W = int(math.ceil((box[2] - box[0]) / res)) + 1
        Hh = int(math.ceil((box[3] - box[1]) / res)) + 1
        zlo, zhi = zr[arm]
        # what the arm is drawn over: 1 = black body, 2 = the other arm
        under = Image.new("L", (W, Hh), 0)

        def lab(name, blk):
            if not blk:
                return 0
            if name in BLACK_BODY:
                return 1
            return 2 if name.startswith("Arm_") else 0
        _draw(ImageDraw.Draw(under), x0, y1, res,
              [f for f in faces if f[0] != arm], box, lab,
              lambda z: z < zlo - 1e-4)
        # the arm, minus whatever is drawn over it (gloves, epaulettes)
        vis = Image.new("L", (W, Hh), 0)
        _draw(ImageDraw.Draw(vis), x0, y1, res, [a], box,
              lambda n, b: 1, lambda z: z > -1e9)
        _draw(ImageDraw.Draw(vis), x0, y1, res,
              [f for f in faces if f[0] != arm], box,
              lambda n, b: 0, lambda z: z > zhi + 1e-4)
        U = np.asarray(under)
        A = np.asarray(vis).astype(bool)
        body = U == 1
        r = int(math.ceil(gap / res))
        dil = body.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r or (dx == 0 and dy == 0):
                    continue
                sh = np.zeros_like(body)
                ys = slice(max(dy, 0), Hh + min(dy, 0))
                yd = slice(max(-dy, 0), Hh + min(-dy, 0))
                xs = slice(max(dx, 0), W + min(dx, 0))
                xd = slice(max(-dx, 0), W + min(-dx, 0))
                sh[yd, xd] = body[ys, xs]
                dil |= sh
        yy, xx = np.mgrid[0:Hh, 0:W]
        X = x0 + xx * res
        Y = y1 - yy * res
        root = joints["ArmHose_%s_00" % s]
        wrist = joints["ArmHose_%s_%02d" % (s, HOSE_N - 1)]
        away = (X - root[0]) ** 2 + (Y - root[1]) ** 2 > root_zone ** 2
        touch = A & dil & away
        row["ink_" + s] = float(touch.sum()) * res * res
        if touch.any():
            row["ink_at_" + s] = [round(float(X[touch].mean()), 2),
                                  round(float(Y[touch].mean()), 2)]
        near = A & ((X - wrist[0]) ** 2 + (Y - wrist[1]) ** 2 <
                    wrist_r ** 2)
        n = int(near.sum())
        row["wrist_" + s] = float((near & body).sum()) / n if n >= 20 else 0.0
        row["cross_" + s] = float((A & (U == 2)).sum()) * res * res
        # the glove over the legs (gloves are drawn over everything below)
        gname = "Glove_" + s
        gf = [f for f in faces if f[0] == gname][0]
        glo, ghi = gf[3].min(axis=0), gf[4].max(axis=0)
        gbox = (glo[0], glo[1], ghi[0], ghi[1])
        gw = int(math.ceil((gbox[2] - gbox[0]) / res)) + 1
        gh = int(math.ceil((gbox[3] - gbox[1]) / res)) + 1
        gi = Image.new("L", (gw, gh), 0)
        _draw(ImageDraw.Draw(gi), gbox[0], gbox[3], res, [gf], gbox,
              lambda n, b: 1, lambda z: z > -1e9)
        li = Image.new("L", (gw, gh), 0)
        _draw(ImageDraw.Draw(li), gbox[0], gbox[3], res,
              [f for f in faces if f[0] in ("Leg_L", "Leg_R")], gbox,
              lambda n, b: 1, lambda z: z > -1e9)
        # shoes are drawn over the legs: a glove on a shoe is another test
        _draw(ImageDraw.Draw(li), gbox[0], gbox[3], res,
              [f for f in faces if f[0] in ("Shoe_L", "Shoe_R")], gbox,
              lambda n, b: 0, lambda z: z > -1e9)
        row["gleg_" + s] = float((np.asarray(gi).astype(bool) &
                                  np.asarray(li).astype(bool)).sum()) *             res * res
    return row


def spans(ts, step):
    out = []
    for t in ts:
        if out and t - out[-1][1] <= step * 1.5:
            out[-1][1] = t
        else:
            out.append([t, t])
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("stage", nargs="?", default=DEFAULT_STAGE)
    ap.add_argument("--start", type=float, default=1.0)
    ap.add_argument("--end", type=float, default=121.0)
    ap.add_argument("--step", type=float, default=0.2,
                    help="sample step (the layer is written every 0.2)")
    ap.add_argument("--joints", help="reuse a --joints-out layer")
    ap.add_argument("--keep", help="write the joints layer here")
    ap.add_argument("--threshold", type=float, default=0.03,
                    help="|bow| / chord below this counts as straight")
    ap.add_argument("--clear", type=float, default=0.08,
                    help="|bow| / chord from which a limb counts as bowed")
    ap.add_argument("--min-frames", type=float, default=3.0,
                    help="a flip must take at least this many frames")
    ap.add_argument("--kink", type=float, default=45.0,
                    help="max turn between two hose segments (deg)")
    ap.add_argument("--s-turn", type=float, default=20.0,
                    help="max turning against the hose's bend (deg)")
    ap.add_argument("--pop", type=float, default=0.25,
                    help="max |acceleration| of bow/elbow (chord/frame^2)")
    ap.add_argument("--contact-tol", type=float, default=0.6,
                    help="spikes this close to a declared foot contact are "
                         "intended impacts")
    ap.add_argument("--no-clearance", action="store_true",
                    help="skip the mesh clearance (staging) checks")
    ap.add_argument("--tangent", type=float, default=0.3,
                    help="less background than this between arm and head "
                         "ink is a near miss")
    ap.add_argument("--tangent-frames", type=float, default=2.0,
                    help="a near-miss run may last at most this long")
    ap.add_argument("--merge", type=float, default=0.12,
                    help="arm joints closer than this (ink to ink) to the "
                         "leg hose merge with it")
    ap.add_argument("--no-ink", action="store_true",
                    help="skip the raster ink-merge check")
    ap.add_argument("--res", type=float, default=0.012,
                    help="ink raster: units per pixel (~1 px at 1080p)")
    ap.add_argument("--ink-gap", type=float, default=0.03,
                    help="arm ink closer than this to black body ink "
                         "touches it")
    ap.add_argument("--ink-area", type=float, default=0.004,
                    help="touching area (square units) that counts as a "
                         "merge")
    ap.add_argument("--root-zone", type=float, default=0.5,
                    help="radius about the shoulder joint where the hose "
                         "may touch the body")
    ap.add_argument("--wrist-r", type=float, default=0.42,
                    help="radius about the wrist joint whose visible hose "
                         "must not lie over black body fill")
    ap.add_argument("--wrist-dark", type=float, default=0.3,
                    help="max fraction of that hose over black body fill")
    ap.add_argument("--ink-frames", type=float, default=2.0,
                    help="an ink merge may last at most this long")
    ap.add_argument("--glove-leg", type=float, default=0.01,
                    help="glove-over-leg area (square units) that counts "
                         "as a glove on the leg")
    ap.add_argument("--json")
    ap.add_argument("--plot")
    args = ap.parse_args()

    stage = os.path.abspath(args.stage)
    n = int(round((args.end - args.start) / args.step))
    frames = [round(args.start + args.step * k, 6) for k in range(n + 1)]
    jpath = args.joints
    if not jpath:
        jpath = args.keep or os.path.join(
            os.environ.get("TEMP", "/tmp"), "rubberhose_check_joints.usda")
        export_joints(stage, frames, jpath)
    leaf, times, data = load_joints(jpath)
    res = analyse(leaf, times, data)

    layer = Sdf.Layer.FindOrOpen(stage)
    intent = dict(layer.customLayerData.get(
        "rubberhose:intendedBendChanges", {}))
    windows = {k: [(float(v[i]), float(v[i + 1]))
                   for i in range(0, len(v) - 1, 2)]
               for k, v in intent.items()}
    contacts = {k: [float(x) for x in v] for k, v in dict(
        layer.customLayerData.get("rubberhose:footContacts", {})).items()}

    bad = 0
    report = {"stage": stage, "samples": len(times), "step": args.step,
              "limbs": {}}
    print("limb flip check: %s  (%d samples, step %g)" % (
        os.path.relpath(stage, REPO), len(times), args.step))
    for limb in LIMBS:
        rows = res[limb]
        W = windows.get(limb, [])
        rep = {}
        for key in ("bow", "elbow"):
            evs = flips(rows, key, args.threshold, args.clear)
            for ev in evs:
                ev["intended"] = intended(ev, W)
                ev["fast"] = ev["frames"] < args.min_frames - 1e-6
            rep[key + "_flips"] = evs
        pops = []
        for a, b, c in zip(rows, rows[1:], rows[2:]):
            dt = b["t"] - a["t"]
            for key in ("bow", "elbow"):
                acc = (a[key] - 2 * b[key] + c[key]) / (dt * dt)
                if abs(acc) > args.pop:
                    pops.append((b["t"], key, round(acc, 3)))
        near = lambda t: any(  # noqa: E731
            min(abs(t - c), LOOP_LEN - abs(t - c)) <= args.contact_tol
            for c in contacts.get(limb, []))
        impacts = [p for p in pops if near(p[0])]
        pops = [p for p in pops if not near(p[0])]
        kinks = [(r["t"], round(r["kink"], 1)) for r in rows
                 if r["kink"] > args.kink]
        s_frames = [(r["t"], round(r["against"], 1)) for r in rows
                    if r["against"] > args.s_turn]
        wmin = min(r["width"] for r in rows)
        pinches = [r["t"] for r in rows if r["pinch"]]
        straight = [r["t"] for r in rows if abs(r["bow"]) < args.threshold]
        worst = max(rows, key=lambda r: r["kink"])
        rep.update({
            "kinks": kinks, "s_curves": s_frames, "pops": pops,
            "min_width": wmin, "pinch_frames": pinches,
            "straight_frames": straight,
            "bow_range": [min(r["bow"] for r in rows),
                          max(r["bow"] for r in rows)],
            "bend_range": [min(r["bend"] for r in rows),
                           max(r["bend"] for r in rows)],
            "max_kink": [worst["t"], worst["kink"]],
            "max_against": max(r["against"] for r in rows),
            "max_pop": max([abs(p[2]) for p in pops] + [0.0]),
            "contact_impacts": impacts,
            "intended_windows": W,
        })
        evs = rep["bow_flips"] + rep["elbow_flips"]
        unint = [e for e in evs if not e["intended"]]
        fast = [e for e in evs if e["fast"]]
        limb_bad = (len(unint) + len(fast) + len(kinks) + len(s_frames) +
                    len(pinches) + len(pops) + (1 if wmin < 0.8 else 0))
        bad += limb_bad
        rep["ok"] = limb_bad == 0
        report["limbs"][limb] = rep
        print("\n%s  %s" % (limb, "OK" if rep["ok"] else "PROBLEMS"))
        print("  bow %+.3f..%+.3f  bend %.0f..%.0f deg  sharpest turn "
              "%.1f deg @%g  max counter-turn %.1f deg  min width %.2f" % (
                  rep["bow_range"][0], rep["bow_range"][1],
                  rep["bend_range"][0], rep["bend_range"][1], worst["kink"],
                  worst["t"], rep["max_against"], wmin))
        for key, name in (("bow_flips", "hose bow"),
                          ("elbow_flips", "elbow side")):
            for e in rep[key]:
                print("  %-10s flip %s at %6.2f: straightens/re-curves "
                      "%6.1f -> %6.1f (%4.1f frames)%s%s" % (
                          name, e["dir"], e["cross"], e["from"], e["to"],
                          e["frames"],
                          "  intended" if e["intended"] else "  UNINTENDED",
                          "  TOO FAST" if e["fast"] else ""))
        if straight:
            print("  straight (|bow| < %.2f): %d samples, %g..%g" % (
                args.threshold, len(straight), straight[0], straight[-1]))
        for label, lst in (("kink", kinks), ("S-curve", s_frames)):
            if lst:
                print("  %s: %d samples, first %s, last %s" % (
                    label, len(lst), lst[0], lst[-1]))
        if pinches:
            print("  pinched strip: %d samples, %g..%g" % (
                len(pinches), pinches[0], pinches[-1]))
        if impacts:
            print("  foot-contact impacts (intended): %d spikes, max %.3f "
                  "at %s" % (len(impacts), max(abs(p[2]) for p in impacts),
                             sorted(set(round(p[0]) for p in impacts))))
        for t, key, acc in pops[:10]:
            print("  pop  %s accel %+.3f/frame^2 at %g" % (key, acc, t))
        if len(pops) > 10:
            print("  ... %d pops" % len(pops))
    clear_rows = None
    ink_rows = None
    if not args.no_clearance or not args.no_ink:
        ppath = os.path.join(os.environ.get("TEMP", "/tmp"),
                             "rubberhose_check_pose.txt")
        export_pose(stage, frames, ppath)
        topo = None if args.no_ink else Topology(stage)
        clear_rows, ink_rows = [], []
        try:
            for t, joints, meshes in iter_pose(ppath, full=topo is not None):
                if not args.no_clearance:
                    clear_rows.append(clearance_row(t, joints, meshes,
                                                    args.tangent, args.merge))
                if topo is not None:
                    ink_rows.append(ink_row(t, joints, meshes, topo, args.res,
                                            args.ink_gap, args.root_zone,
                                            args.wrist_r))
        finally:
            os.remove(ppath)
        clear_rows = clear_rows or None
        ink_rows = ink_rows or None
    if clear_rows:
        rep = {}
        print("\nclearance (mesh hulls, ink to ink)")
        for s in ("L", "R"):
            over = [r["t"] for r in clear_rows if r["head_" + s] < 0]
            near = spans([r["t"] for r in clear_rows
                          if r["tangent_" + s] >= 3], args.step)
            long_near = [(a, b) for a, b in near
                         if b - a + args.step > args.tangent_frames + 1e-6]
            shoe = [r["t"] for r in clear_rows if r["shoe_" + s] <= 0.0]
            merged = [r["t"] for r in clear_rows if r["merge_" + s] >= 4]
            worst = min(clear_rows, key=lambda r: r["head_" + s])
            worst_g = min(clear_rows, key=lambda r: r["gleg_" + s])
            n_bad = len(over) + len(long_near) + len(shoe) + len(merged)
            bad += n_bad
            rep["Arm_" + s] = {
                "min_head_gap": [worst["t"], worst["head_" + s]],
                "over_head": over, "tangent_runs": near,
                "tangent_too_long": long_near, "glove_on_shoe": shoe,
                "arm_on_leg": merged,
                "min_glove_shoe": min(r["shoe_" + s] for r in clear_rows),
                "max_merged_joints": max(r["merge_" + s]
                                         for r in clear_rows),
                "min_glove_hull_leg": [worst_g["t"], worst_g["gleg_" + s]],
                "ok": n_bad == 0}
            print("Arm_%s  %s  head gap min %+.3f @%g  glove-shoe min %.3f"
                  "  arm-on-leg max %d joints  glove hull-leg min %+.3f @%g" % (
                      s, "OK" if n_bad == 0 else "PROBLEMS",
                      worst["head_" + s], worst["t"],
                      rep["Arm_" + s]["min_glove_shoe"],
                      rep["Arm_" + s]["max_merged_joints"],
                      worst_g["gleg_" + s], worst_g["t"]))
            for label, ts in (("ARM OVER HEAD", over),
                              ("GLOVE ON SHOE", shoe),
                              ("ARM ON LEG", merged)):
                if ts:
                    print("  %s: %d samples, %s" % (label, len(ts), ", ".join(
                        "%g-%g" % (a, b) for a, b in spans(ts, args.step))))
            for a, b in near:
                print("  near miss along the head %g-%g (%.1f frames)%s" % (
                    a, b, b - a + args.step,
                    "  TANGENT" if (a, b) in long_near else ""))
        report["clearance"] = rep
    if ink_rows:
        rep = {}
        print("\nink merge (raster %.3f units/px; arm ink vs pants, body "
              "outline and leg hoses)" % args.res)
        for s in ("L", "R"):
            touch = spans([r["t"] for r in ink_rows
                           if r["ink_" + s] > args.ink_area], args.step)
            lost = spans([r["t"] for r in ink_rows
                          if r["wrist_" + s] > args.wrist_dark], args.step)
            long_t = [(a, b) for a, b in touch
                      if b - a + args.step > args.ink_frames + 1e-6]
            long_w = [(a, b) for a, b in lost
                      if b - a + args.step > args.ink_frames + 1e-6]
            on_leg = spans([r["t"] for r in ink_rows
                            if r["gleg_" + s] > args.glove_leg], args.step)
            long_g = [(a, b) for a, b in on_leg
                      if b - a + args.step > args.ink_frames + 1e-6]
            worst_g = max(ink_rows, key=lambda r: r["gleg_" + s])
            worst = max(ink_rows, key=lambda r: r["ink_" + s])
            worst_w = max(ink_rows, key=lambda r: r["wrist_" + s])
            n_bad = len(long_t) + len(long_w) + len(long_g)
            bad += n_bad
            rep["Arm_" + s] = {
                "max_touch": [worst["t"], worst["ink_" + s],
                              worst.get("ink_at_" + s)],
                "max_wrist_dark": [worst_w["t"], worst_w["wrist_" + s]],
                "touch_runs": touch, "touch_too_long": long_t,
                "wrist_runs": lost, "wrist_too_long": long_w,
                "max_arm_cross": max(r["cross_" + s] for r in ink_rows),
                "max_glove_leg": [worst_g["t"], worst_g["gleg_" + s]],
                "glove_leg_runs": on_leg, "glove_leg_too_long": long_g,
                "ok": n_bad == 0}
            print("Arm_%s  %s  max touch %.4f sq units @%g %s  max wrist over "
                  "black %.0f%% @%g  arm-over-arm max %.3f  glove over "
                  "leg max %.3f @%g" % (
                      s, "OK" if n_bad == 0 else "PROBLEMS",
                      worst["ink_" + s], worst["t"],
                      worst.get("ink_at_" + s, ""),
                      100 * worst_w["wrist_" + s], worst_w["t"],
                      rep["Arm_" + s]["max_arm_cross"],
                      worst_g["gleg_" + s], worst_g["t"]))
            for a, b in on_leg:
                print("  glove over a leg %g-%g (%.1f frames)%s" % (
                    a, b, b - a + args.step,
                    "  GLOVE ON LEG" if (a, b) in long_g else ""))
            for a, b in touch:
                print("  arm ink touches the black body %g-%g (%.1f frames)%s"
                      % (a, b, b - a + args.step,
                         "  INK MERGE" if (a, b) in long_t else ""))
            for a, b in lost:
                print("  wrist over black fill %g-%g (%.1f frames)%s" % (
                    a, b, b - a + args.step,
                    "  WRIST LOST" if (a, b) in long_w else ""))
        report["ink"] = rep
    print("\n%s" % ("PASS: no unintended, abrupt or kinked limb flips" +
                    ("" if args.no_clearance else
                     ", no arm over the face, no tangents, no merged limbs") +
                    ("" if args.no_ink else ", no arm ink merged with the "
                     "body")
                    if bad == 0 else "FAIL: %d problems" % bad))

    if args.json:
        with open(args.json, "w") as fh:
            slim = dict(report)
            slim["series"] = {
                limb: [{k: (round(v, 5) if isinstance(v, float) else v)
                        for k, v in r.items()} for r in res[limb]]
                for limb in LIMBS}
            if clear_rows:
                slim["clearance_series"] = [
                    {k: (round(v, 4) if isinstance(v, float) else v)
                     for k, v in r.items()} for r in clear_rows]
            if ink_rows:
                slim["ink_series"] = [
                    {k: (round(v, 5) if isinstance(v, float) else v)
                     for k, v in r.items()} for r in ink_rows]
            json.dump(slim, fh, indent=1)
    if args.plot:
        plot(res, windows, args.plot, args.threshold, clear_rows,
             args.tangent, ink_rows, args.ink_area)
    return 0 if bad == 0 else 1


def plot(res, windows, out, thr, clear_rows=None, tangent=0.3,
         ink_rows=None, ink_area=0.004):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(LIMBS) + (1 if clear_rows else 0) + (1 if ink_rows else 0)
    fig, axes = plt.subplots(n, 1, figsize=(13, 2.4 * n), sharex=True)
    if ink_rows:
        ax = axes[-1]
        t = [r["t"] for r in ink_rows]
        ax.axhspan(0, ink_area, color="#fde9b0")
        for s, col in (("L", "#1f5fbf"), ("R", "#c0392b")):
            ax.plot(t, [r["ink_" + s] for r in ink_rows], color=col, lw=1.2,
                    label="Arm_%s ink touching black body (sq units)" % s)
            ax.plot(t, [0.05 * r["wrist_" + s] for r in ink_rows], color=col,
                    lw=0.8, ls="--",
                    label="Arm_%s wrist over black (x0.05)" % s)
        ax.set_ylim(0, 0.06)
        ax.set_ylabel("ink merge")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, ncol=2)
    if clear_rows:
        ax = axes[len(LIMBS)]
        t = [r["t"] for r in clear_rows]
        ax.axhspan(0, tangent, color="#fde9b0")
        for s, col in (("L", "#1f5fbf"), ("R", "#c0392b")):
            ax.plot(t, [r["head_" + s] for r in clear_rows], color=col,
                    lw=1.2, label="Arm_%s ink gap to head" % s)
            ax.plot(t, [r["shoe_" + s] for r in clear_rows], color=col,
                    lw=0.8, ls="--", label="Glove_%s to shoes" % s)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_ylim(-0.5, 2.5)
        ax.set_ylabel("clearance")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8, ncol=4)
    for ax, limb in zip(axes, LIMBS):
        rows = res[limb]
        t = [r["t"] for r in rows]
        ax.axhspan(-thr, thr, color="0.9")
        for a, b in windows.get(limb, []):
            ax.axvspan(a, b, color="#fde9b0")
        ax.plot(t, [r["bow"] for r in rows], color="#1f5fbf", lw=1.6,
                label="hose bow")
        ax.plot(t, [r["elbow"] for r in rows], color="#c0392b", lw=1.0,
                label="elbow side")
        ax2 = ax.twinx()
        ax2.plot(t, [r["kink"] for r in rows], color="#888888", lw=0.8,
                 ls=":", label="sharpest turn (deg)")
        ax2.set_ylim(0, 90)
        ax.axhline(0, color="k", lw=0.5)
        ax.set_ylabel(limb)
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper left", fontsize=8, ncol=2)
    axes[-1].set_xlabel("frame")
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print("wrote", out)


if __name__ == "__main__":
    sys.exit(main())
