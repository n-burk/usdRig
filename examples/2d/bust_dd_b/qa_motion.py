"""Motion QA for the Shion bust: evaluate the rig every frame with the
python RigExec evaluator (the same values Hydra draws), track key points,
and plot position / speed curves and screen-space trails.

    source bin/_env.sh
    "$PY" examples/2d/bust_dd_b/qa_motion.py examples/2d/bust_dd_b/bust_dd_b_anim.usda OUT_DIR [--frames a:b]

Writes OUT_DIR/motion.npz, motion_plot.png (x, y and speed per frame for
every tracked point, with the curve extremes marked) and motion_trails.png
(the paths drawn over the rest silhouette). Prints dead stops (speed < 0.01
units/frame for more than 3 frames) of the head points.
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))


def track_points(stage):
    """{label: (mesh prim path, vertex index)} picked from the rest points."""
    from pxr import UsdGeom
    picks = {
        "nose": ("Face", (0.18, -4.55), None),
        "chin": ("Face", (0.0, -10.0), None),
        "iris_R": ("Eye_R", (4.05, -0.45), "Iris_R"),
        "iris_L": ("Eye_L", (-4.05, -0.45), "Iris_L"),
        "mouth": ("Mouth", (0.08, -6.75), "MouthLine"),
        "brow_L": ("Brow_L", (-2.2, 1.6), None),
        "fringe_tip": ("Hair", (6.9, 2.6), "F1"),
        "lock_tip": ("Hair", (9.0, -15.0), "LockR_A"),
        "back_tip": ("BackHair", (-7.0, -16.5), None),
        "earring": ("Earring", (-7.62, -6.7), None),
        "ribbon_tip": ("Ribbon", (1.75, -24.1), "RibbonTailR"),
    }
    out = {}
    for label, (mesh, xy, subset) in picks.items():
        if stage.GetPrimAtPath("/Shion/Geom/Character"):
            subset = subset or mesh
            mesh = "Character"
        prim = stage.GetPrimAtPath("/Shion/Geom/" + mesh)
        pts = np.array(UsdGeom.Mesh(prim).GetPointsAttr().Get())
        cand = np.arange(len(pts))
        if subset:
            sub = stage.GetPrimAtPath("/Shion/Geom/%s/%s" % (mesh, subset))
            faces = np.array(UsdGeom.Subset(sub).GetIndicesAttr().Get())
            idx = np.array(UsdGeom.Mesh(prim).GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
            cand = np.unique(idx[faces].ravel())
        d = np.linalg.norm(pts[cand, :2] - np.array(xy), axis=1)
        out[label] = (str(prim.GetPath()), int(cand[np.argmin(d)]))
    return out


def evaluate(path, frames):
    sys.path.insert(0, os.path.join(REPO, "docs"))
    import render_media as rm
    rigexec = rm._schema_plugin()
    from pxr import Usd
    stage = Usd.Stage.Open(path)
    picks = track_points(stage)
    rig = rigexec.Rig(stage, "/Shion/Rig")
    rig.compile()
    data = {k: np.zeros((len(frames), 2)) for k in picks}
    for i, f in enumerate(frames):
        pose = rig.evaluate(float(f))
        moved = pose.moved_properties()
        for label, (prim, vi) in picks.items():
            p = moved.get(prim + ".points")
            if p is None:
                continue
            data[label][i] = np.asarray(p)[vi, :2]
    return data


def plot(frames, data, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    labels = list(data)
    fig, axes = plt.subplots(len(labels), 3, figsize=(18, 2.0 * len(labels)), sharex=True)
    for r, k in enumerate(labels):
        p = data[k]
        v = np.linalg.norm(np.gradient(p, axis=0), axis=1)
        for c, (series, name) in enumerate(((p[:, 0], "x"), (p[:, 1], "y"), (v, "speed"))):
            ax = axes[r, c]
            ax.plot(frames, series, lw=1.2, color=("#2b6cb0", "#c05621", "#2f855a")[c])
            ax.set_ylabel("%s %s" % (k, name), fontsize=7)
            ax.tick_params(labelsize=6)
            ax.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("frame")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "motion_plot.png"), dpi=70)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(10, 11))
    cmap = plt.get_cmap("viridis")
    for k in labels:
        p = data[k]
        ax.scatter(p[:, 0], p[:, 1], c=frames, cmap=cmap, s=5)
        ax.plot(p[:, 0], p[:, 1], lw=0.5, color="#888888")
        ax.annotate(k, p[0], fontsize=7)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_title("trails (colour = frame)")
    fig.savefig(os.path.join(out, "motion_trails.png"), dpi=80)
    plt.close(fig)


def dead_stops(frames, data, keys=("nose", "iris_R", "chin"), thresh=0.01, run=4):
    msgs = []
    for k in keys:
        v = np.linalg.norm(np.gradient(data[k], axis=0), axis=1)
        still = v < thresh
        start = None
        for i, s in enumerate(list(still) + [False]):
            if s and start is None:
                start = i
            if not s and start is not None:
                if i - start >= run:
                    msgs.append("%s still %d-%d (%d f)" % (k, frames[start], frames[i - 1], i - start))
                start = None
    return msgs


def main():
    path = sys.argv[1]
    out = sys.argv[2]
    a, b = 1, 240
    if "--frames" in sys.argv:
        a, b = map(int, sys.argv[sys.argv.index("--frames") + 1].split(":"))
    os.makedirs(out, exist_ok=True)
    frames = np.arange(a, b + 1)
    data = evaluate(path, frames)
    np.savez(os.path.join(out, "motion.npz"), frames=frames, **data)
    plot(frames, data, out)
    for m in dead_stops(frames, data):
        print("  dead stop:", m)
    print("wrote", out)


if __name__ == "__main__":
    main()
