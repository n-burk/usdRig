"""A closing lid sweeps AROUND the eyeball, it does not cut through it.

`rigExec:weightBlend = "radial"` exists so a partially weighted point
takes a fraction of the transform's ROTATION about the pivot that
transform turns about, instead of a fraction of the resulting position.
A fraction of the position follows the chord of the arc, and a chord
falls inside its arc -- toward the axis. On a blink the axis is the
eyeball's centre, so a linear blend walks the lid straight into the eye.

That is what shipped. The radial flag was authored on 105 face clusters
and honoured by the dense kernel, but `RigExecApplyMatrixKernel` has a
sparse fast path -- a painted cluster names a few hundred of a body's
26276 points, so resolving a dense envelope is nearly all waste -- and
that path applied the linear blend unconditionally. Every painted
cluster on this rig is sparse, so the flag reached nothing. The zero-USD
runtime had the same hole in both of its paths and no radial support at
all.

Measured before the fix, at a full upper blink (L_UpLid ty=-2.735), on
an eyeball of fitted radius 2.4846: 50 lid points that start outside the
ball end up inside it, the worst 0.2951 deep. Both sides identically,
which is why the eye looked like it was being pierced rather than
covered.

The threshold is a small tolerance and not zero: the lid and the ball
are separate meshes with their own tessellation, the ball is only very
nearly a sphere (fitted min 2.4775, max 2.5071), and a point that grazes
the surface is allowed to graze it.

The second half gates the ZERO-USD RUNTIME on the same pose. The example
parity fixtures bake the biped at rest, where every cluster transform is
the identity and a weighting bug cannot show, so the runtime kept its own
copy of this fault with nothing to catch it. Here the lids are shut
first, which is when the weighting matters, and the runtime is required
to match the baked path bit for bit. It also gates the recorded read:
the runtime has no stage to ask what `rigExec:weightBlend` says, so an
unrecorded read replays as linear and this comparison is what notices.

Usage:
    python test_rigexec_blink_clearance.py [schema_resources_dir]
                                           [rigExecBake] [rigExecPose]
"""

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_REPO, "examples", "biped", "Biped_stack.usda")
_RIG = "/Biped/Rig"

# (control, eyeball mesh). Both sides, because a fault in the weighted
# blend is symmetric and a fault in one eye's bind frame is not.
_EYES = [("L_UpLid", "l_eye_geo"), ("R_UpLid", "r_eye_geo")]
# the lid travel a full blink asks for, read off the control's own range
_SHUT = -2.735
# the ball's own departure from a sphere is 0.023; this is comfortably
# over that and far under the 0.2951 the linear blend drove
_TOL = 0.05
# how far outside the ball a body point still counts as lid
_BAND = 1.2


def _fit_sphere(points):
    """Centre and mean radius of a nearly-spherical point set.

    The eyeball's CENTROID is 0.298 off its centre -- the tessellation is
    much denser at the cornea -- and 0.298 happens to be about the size
    of the fault being measured, which is a good way to chase a pivot
    offset that is not there. So the centre is fitted rather than
    averaged.
    """
    n = len(points)
    c = [sum(p[i] for p in points) / n for i in range(3)]
    for _ in range(60):
        lengths = [math.dist(p, c) for p in points]
        mean = sum(lengths) / n
        grad = [sum((c[i] - p[i]) / max(l, 1e-12)
                    for p, l in zip(points, lengths)) / n for i in range(3)]
        c = [sum(p[i] for p in points) / n + mean * grad[i] for i in range(3)]
    lengths = [math.dist(p, c) for p in points]
    return c, sum(lengths) / n


def _check_runtime_matches(failures, resources):
    """The zero-USD runtime deforms the shut lids exactly as the bake did.

    Skipped, loudly, when the two CLIs were not handed over: the test is
    still worth running for its first half from a bare checkout.
    """
    import subprocess
    import tempfile
    from pxr import Sdf, Usd

    if len(sys.argv) < 4:
        print("    (no rigExecBake/rigExecPose given; runtime half skipped)")
        return False
    bake, pose = sys.argv[2], sys.argv[3]
    out = tempfile.mkdtemp(prefix="rigexec-blink-")
    overlay = os.path.join(out, "blink_shut.usda")
    layer = Sdf.Layer.CreateNew(overlay)
    layer.subLayerPaths.append(_STACK.replace("\\", "/"))
    stage = Usd.Stage.Open(layer)
    shut = 0
    for prim in stage.Traverse():
        if (prim.GetName() in [c for c, _m in _EYES]
                and str(prim.GetTypeName()) == "RigExecControl"):
            attr = prim.GetAttribute("avars:ty")
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute("avars:ty",
                                            Sdf.ValueTypeNames.Double)
            attr.Set(_SHUT)
            shut += 1
    assert shut == len(_EYES), "shut %d of %d lids" % (shut, len(_EYES))
    layer.Save()

    env = dict(os.environ)
    if resources:
        env["PXR_PLUGINPATH_NAME"] = resources
    binary = os.path.join(out, "blink_shut.rigexec")
    for argv in ([bake, overlay, "--time", "1", "-o", binary],
                 [pose, overlay, "--verify-binary", binary, "--frames", "1"]):
        done = subprocess.run(argv, env=env, capture_output=True, text=True)
        if done.returncode != 0:
            failures.append("%s failed (%d):\n%s"
                            % (os.path.basename(argv[0]), done.returncode,
                               done.stderr[-2000:]))
            return True
        tail = done.stdout
    for line in tail.splitlines():
        if "binary==baked" in line or "verify-binary:" in line:
            print("    " + line.strip())
    if "verify-binary: 1 of 1 frame(s) match" not in tail:
        failures.append(
            "the zero-USD runtime does not reproduce the shut lids:\n"
            + tail[-2000:])
    return True


def main():
    _setup_environment()
    from pxr import Sdf, Usd
    import rigexec
    resources = sys.argv[1] if len(sys.argv) > 1 else None
    rigexec.load_schema_plugin(resources)

    failures = []
    for control, mesh in _EYES:
        stage = Usd.Stage.Open(_STACK)
        assert stage, _STACK
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        rig = rigexec.Rig(stage, _RIG)
        rig.cpu_reference = True

        prim = None
        for p in stage.Traverse():
            if (p.GetName() == control
                    and str(p.GetTypeName()) == "RigExecControl"):
                prim = p
                break
        assert prim, "no %s control" % control

        def read():
            pose = rig.evaluate(0)
            assert pose.valid, [d for d in pose.diagnostics
                                if not d.startswith("warning:")]
            return ([tuple(v) for v in pose.moved_property(
                        "/Biped/Geom/body_geo.points")],
                    [tuple(v) for v in pose.moved_property(
                        "/Biped/Geom/%s.points" % mesh)])

        restBody, restEye = read()
        centre, radius = _fit_sphere(restEye)
        # only the lid: body points that sit on or just outside the ball
        region = [i for i, q in enumerate(restBody)
                  if math.dist(q, centre) < radius + _BAND]
        assert region, "%s: no lid points found around %s" % (control, mesh)
        outside = {i for i in region
                   if math.dist(restBody[i], centre) >= radius}

        attr = prim.GetAttribute("avars:ty")
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("avars:ty", Sdf.ValueTypeNames.Double)
        attr.Set(_SHUT)
        shutBody, shutEye = read()
        centre, radius = _fit_sphere(shutEye)

        depths = [radius - math.dist(shutBody[i], centre) for i in outside]
        worst = max(depths) if depths else 0.0
        pierced = sum(1 for d in depths if d > _TOL)
        print("    %-9s ty=%.3f -> %-11s worst %.4f over %d of %d lid points"
              % (control, _SHUT, mesh, worst, pierced, len(outside)))
        if pierced:
            failures.append(
                "%s shut drives %d lid points up to %.4f INSIDE %s "
                "(radius %.4f) -- the weighted blend is cutting the chord "
                "instead of turning about the pivot"
                % (control, pierced, worst, mesh, radius))
        attr.Set(0.0)

    ran = _check_runtime_matches(failures, resources)
    assert not failures, failures
    print("OK: a closing lid stays outside the eyeball on both sides"
          + (", and the runtime agrees" if ran else ""))


if __name__ == "__main__":
    main()
