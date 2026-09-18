"""Epsilon sweep around full reach (softness 0.1, stretch 1) to show the
jump does not shrink with the effector step (a true discontinuity)."""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import gen_twobone as g  # noqa: E402

g.RATIOS = [1 - 1e-3, 1 - 1e-6, 1 - 1e-9, 1.0, 1 + 1e-9, 1 + 1e-6, 1 + 1e-3]
g.VARIANTS = {"tb_fine_soft0.1_stretch1": (0.1, 1.0)}
g.main()

import run_twobone as r  # noqa: E402
r.g = g
r.FRAMES = ",".join(str(k + 1) for k in range(len(g.RATIOS)))
frames, tail = r.run("tb_fine_soft0.1_stretch1", "dynamic")
for k, ratio in enumerate(g.RATIOS):
    f = frames[k + 1]
    d = math.dist(f["/Asset/Rig/Joints/Hip"],
                  f["/Asset/Rig/Joints/Hip/Knee/Ankle"])
    print("ratio 1%+.0e  effDist %.10f  endDist %.12f" % (
        ratio - 1, ratio * g.CHAIN, d))
soft = 0.1 * g.CHAIN
print("predicted jump soft*exp(-1) = %.12f" % (soft * math.exp(-1)))
