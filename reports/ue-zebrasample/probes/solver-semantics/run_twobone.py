"""Run the two-bone probe stages through rigExecPose and tabulate
|Ankle - Hip| per effector ratio for dynamic and baked modes."""
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_twobone as g  # noqa: E402

EXE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "build", "rigExecPose.exe")
FRAMES = ",".join(str(k + 1) for k in range(len(g.RATIOS)))


def run(name, mode):
    stage = os.path.join(HERE, name + ".usda")
    out = os.path.join(HERE, "%s_%s_pose.txt" % (name, mode))
    proc = subprocess.run(
        [EXE, stage, "--frames", FRAMES, "--mode", mode, "--pose-out", out],
        capture_output=True, text=True)
    if proc.returncode != 0:
        print(proc.stdout, proc.stderr)
        raise SystemExit("rigExecPose failed: %s %s" % (name, mode))
    frames = {}
    cur = None
    for line in open(out):
        parts = line.split()
        if parts and parts[0] == "frame":
            cur = int(parts[1])
        elif (len(parts) > 4 and parts[0] == "jointFramesFinal"):
            frames.setdefault(cur, {})[parts[1]] = tuple(
                float(x) for x in parts[2:5])
    tail = [l for l in proc.stdout.splitlines() if "baked:" in l or
            "compile:" in l or "mismatch" in l.lower() and "0 mismatches" not in l]
    return frames, tail


def main():
    for name in g.VARIANTS:
        soft, stretch = g.VARIANTS[name]
        print("== %s (inputs:softness=%g, inputs:stretch=%g, chain=%g)"
              % (name, soft, stretch, g.CHAIN))
        results = {}
        for mode in ("dynamic", "baked"):
            frames, tail = run(name, mode)
            results[mode] = frames
            for t in tail:
                print("   [%s] %s" % (mode, t.strip()))
        print("   %-7s %-10s %-20s %-20s %-12s" %
              ("ratio", "effDist", "endDist(dynamic)", "endDist(baked)",
               "end/chain"))
        prev = None
        for k, r in enumerate(g.RATIOS):
            f = k + 1
            dists = []
            for mode in ("dynamic", "baked"):
                hip = results[mode][f]["/Asset/Rig/Joints/Hip"]
                ank = results[mode][f]["/Asset/Rig/Joints/Hip/Knee/Ankle"]
                dists.append(math.dist(hip, ank))
            step = "" if prev is None else "  (step %+.6f for eff step %+.6f)" % (
                dists[0] - prev[0], r * g.CHAIN - prev[1])
            print("   %-7g %-10.5g %-20.12f %-20.12f %-12.6f%s" % (
                r, r * g.CHAIN, dists[0], dists[1], dists[0] / g.CHAIN, step))
            prev = (dists[0], r * g.CHAIN)
        print()


if __name__ == "__main__":
    main()
