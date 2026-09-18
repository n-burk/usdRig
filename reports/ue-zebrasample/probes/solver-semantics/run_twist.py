"""Run twist_swing.usda (dynamic + baked) and decompose each distributed
joint's rotation (rest = identity) into swing of the +X aim and twist about
+X. Rotation is read from the joint frame's handle points."""
import math
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
EXE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "build", "rigExecPose.exe")
STAGE = os.path.join(HERE, "twist_swing.usda")
LABELS = {1: "wrist rz=45 (swing about Z)", 2: "wrist ry=45 (swing about Y)",
          3: "wrist rx=90 (pure twist)", 4: "rest"}


def unit(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v]


def sub(a, b):
    return [p - q for p, q in zip(a, b)]


def dot(a, b):
    return sum(p * q for p, q in zip(a, b))


def mat_to_quat(ex, ey, ez):
    # columns are the rotated basis vectors: R = [ex ey ez]
    m = [[ex[0], ey[0], ez[0]], [ex[1], ey[1], ez[1]], [ex[2], ey[2], ez[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    w = math.sqrt(max(0.0, 1 + tr)) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + m[0][0] - m[1][1] - m[2][2])) / 2, m[2][1] - m[1][2])
    y = math.copysign(math.sqrt(max(0.0, 1 - m[0][0] + m[1][1] - m[2][2])) / 2, m[0][2] - m[2][0])
    z = math.copysign(math.sqrt(max(0.0, 1 - m[0][0] - m[1][1] + m[2][2])) / 2, m[1][0] - m[0][1])
    return w, x, y, z


def analyze(mode):
    out = os.path.join(HERE, "twist_swing_%s_pose.txt" % mode)
    proc = subprocess.run([EXE, STAGE, "--frames", "1,2,3,4", "--mode", mode,
                           "--pose-out", out], capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stdout, proc.stderr)
        raise SystemExit("failed")
    for l in proc.stdout.splitlines():
        if "compile:" in l or "baked:" in l or "warn" in l.lower():
            print("  [%s] %s" % (mode, l.strip()))
    rows = {}
    frame = None
    for line in open(out):
        p = line.split()
        if p and p[0] == "frame":
            frame = int(p[1])
        elif p and p[0] == "jointFramesFinal":
            pts = [float(x) for x in p[2:14]]
            o, px, py, pz = pts[0:3], pts[3:6], pts[6:9], pts[9:12]
            ex, ey, ez = unit(sub(px, o)), unit(sub(py, o)), unit(sub(pz, o))
            w, x, y, z = mat_to_quat(ex, ey, ez)
            total = math.degrees(2 * math.acos(min(1.0, abs(w))))
            # swing = angle between posed aim and rest aim (+X)
            swing = math.degrees(math.acos(max(-1.0, min(1.0, ex[0]))))
            # twist about +X from swing-twist decomposition q = swing*twist
            tlen = math.hypot(w, x)
            twist = math.degrees(2 * math.atan2(x, w)) if tlen > 1e-15 else 0.0
            rows.setdefault(frame, []).append(
                (p[1].rsplit("/", 1)[1], o, ex, total, swing, twist))
    return rows


def main():
    res = {m: analyze(m) for m in ("dynamic", "baked")}
    for f in (1, 2, 3, 4):
        print("frame %d: %s" % (f, LABELS[f]))
        print("   %-4s %-26s %-28s %-9s %-9s %-9s | baked total" % (
            "jnt", "origin", "aim(+X handle dir)", "totalDeg", "swingDeg",
            "twistDeg"))
        for a, b in zip(res["dynamic"][f], res["baked"][f]):
            name, o, ex, total, swing, twist = a
            print("   %-4s (%6.3f %6.3f %6.3f)      (%7.4f %7.4f %7.4f)     "
                  "%-9.4f %-9.4f %-9.4f | %.4f" % (
                      name, o[0], o[1], o[2], ex[0], ex[1], ex[2], total,
                      swing, twist, b[3]))
        print()


if __name__ == "__main__":
    main()
