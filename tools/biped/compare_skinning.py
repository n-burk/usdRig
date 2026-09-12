#!/usr/bin/env python
"""Quantify how far RigExec's skinning diverges from Maya-style LBS/DQS.

Poses one joint the same way through both paths and measures the difference
per vertex, bucketed by how many influences each vertex has. The point is to
turn "RigExec's movers are not linear blend skinning" from an assertion into
a number, so you can decide per-shot whether it matters.

At rest both paths must agree exactly -- every mover's matrix is identity,
so `p' = p + w*(I*p - p) = p`. Divergence appears only once a joint moves,
and only on vertices with more than one influence.

Usage:
    compare_skinning.py <rigexec_skinned.usda> <usdskel.usda> [joint] [deg]
"""
import sys

from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

import rigexec

RIG_ROOT = "/Biped/Rig"
MESH = "/Biped/Geom/body_geo"
TARGET = MESH + ".points"


def rigexec_points(stage, joint_name, degrees):
    rig = rigexec.Rig(stage, RIG_ROOT)
    rig.compile()
    if joint_name:
        # A joint a solver claims ignores its own avars, so posing it here
        # would leave the RigExec mesh at rest while the UsdSkel side
        # rotates -- comparing two different poses and blaming the skinning
        # model. Refuse rather than report a meaningless number.
        for p in stage.Traverse():
            if not p.GetTypeName().startswith("RigExec"):
                continue
            rel = p.GetRelationship("rigExec:joints")
            if rel and any(str(t).rsplit("/", 1)[-1] == joint_name
                           for t in rel.GetTargets()):
                raise SystemExit(
                    "%s is claimed by %s (%s), so its avars do nothing and "
                    "the two paths would not be posed alike. Compare a rig "
                    "built with --no-ik --no-blend, or pick an unclaimed "
                    "joint." % (joint_name, p.GetName(), p.GetTypeName()))
        prim = None
        for p in stage.Traverse():
            if p.GetName() == joint_name and p.GetTypeName() == "RigExecJoint":
                prim = p
                break
        if prim is None:
            raise SystemExit("no RigExecJoint named %s" % joint_name)
        attr = prim.GetAttribute("avars:rz")
        if not attr or not attr.IsValid():
            from pxr import Sdf
            attr = prim.CreateAttribute("avars:rz", Sdf.ValueTypeNames.Double)
        attr.Set(float(degrees))
    pose = rig.evaluate(0.0)
    return pose.moved_property(TARGET), pose


def skel_points(stage, joint_name, degrees):
    """The same pose through UsdSkel, i.e. real LBS/DQS."""
    cache = UsdSkel.Cache()
    root = UsdSkel.Root(stage.GetPrimAtPath("/Biped"))
    cache.Populate(root, Usd.PrimDefaultPredicate)
    skel = UsdSkel.Skeleton(stage.GetPrimAtPath("/Biped/Skeleton"))
    query = cache.GetSkelQuery(skel)
    sq = cache.GetSkinningQuery(stage.GetPrimAtPath(MESH))
    if not sq:
        raise SystemExit("skinning query invalid on %s" % MESH)

    joints = list(skel.GetJointsAttr().Get())
    xf = list(query.ComputeSkinningTransforms(Usd.TimeCode.Default()))

    if joint_name:
        # Find the joint and every descendant, and apply the same local
        # rotation about its own origin so the two paths pose identically.
        idx = None
        for i, jp in enumerate(joints):
            if jp.rsplit("/", 1)[-1] == joint_name:
                idx = i
                break
        if idx is None:
            raise SystemExit("no skel joint named %s" % joint_name)
        binds = list(skel.GetBindTransformsAttr().Get())
        origin = binds[idx].ExtractTranslation()
        basis = binds[idx].ExtractRotationMatrix()
        # Rotate about the joint's local Z, expressed in world.
        axis = Gf.Vec3d(basis[2][0], basis[2][1], basis[2][2])
        rot = Gf.Matrix4d(1.0)
        rot.SetRotate(Gf.Rotation(axis, float(degrees)))
        about = (Gf.Matrix4d(1.0).SetTranslate(-origin) * rot *
                 Gf.Matrix4d(1.0).SetTranslate(origin))
        prefix = joints[idx] + "/"
        for i, jp in enumerate(joints):
            if i == idx or jp.startswith(prefix):
                xf[i] = xf[i] * about

    pts = UsdGeom.Mesh(stage.GetPrimAtPath(MESH)).GetPointsAttr().Get()
    out = Vt.Vec3fArray(list(pts))
    sq.ComputeSkinnedPoints(Vt.Matrix4dArray(xf), out,
                            Usd.TimeCode.Default())
    return out, sq


def main(argv):
    rig_path = argv[0]
    skel_path = argv[1]
    joint = argv[2] if len(argv) > 2 else "elbow_l_bind"
    deg = float(argv[3]) if len(argv) > 3 else 45.0

    rigexec.load_schema_plugin()

    print("=== rest pose: both paths must agree exactly ===")
    a, _ = rigexec_points(Usd.Stage.Open(rig_path), None, 0)
    b, sq = skel_points(Usd.Stage.Open(skel_path), None, 0)
    worst = max(max(abs(a[i][k] - b[i][k]) for k in range(3))
                for i in range(len(a)))
    print("  %d vs %d points, worst %.3g cm  [%s]"
          % (len(a), len(b), worst, "PASS" if worst < 1e-3 else "FAIL"))

    print("\n=== posed: %s rotated %.0f deg ===" % (joint, deg))
    a, _ = rigexec_points(Usd.Stage.Open(rig_path), joint, deg)
    b, _ = skel_points(Usd.Stage.Open(skel_path), joint, deg)

    # Bucket by influence count, which is what predicts divergence. Read it
    # off the primvar rather than a SkinningQuery: a query is only valid
    # while the stage it came from is alive.
    skel_stage = Usd.Stage.Open(skel_path)
    wpv = UsdSkel.BindingAPI(
        skel_stage.GetPrimAtPath(MESH)).GetJointWeightsPrimvar()
    wts = wpv.Get()
    n = wpv.GetElementSize()
    per_vertex = [sum(1 for k in range(n) if wts[v * n + k] != 0.0)
                  for v in range(len(a))]

    buckets = {}
    moved = 0
    for i in range(len(a)):
        d = max(abs(a[i][k] - b[i][k]) for k in range(3))
        c = per_vertex[i]
        e = buckets.setdefault(c, [0, 0.0, 0.0])
        e[0] += 1
        e[1] = max(e[1], d)
        e[2] += d
        if d > 0.01:
            moved += 1
    print("  vertices differing by >0.01cm: %d of %d (%.1f%%)"
          % (moved, len(a), 100.0 * moved / len(a)))
    print("  %-12s %8s %12s %12s" % ("influences", "verts", "worst cm",
                                     "mean cm"))
    for c in sorted(buckets):
        n, w, tot = buckets[c]
        print("  %-12d %8d %12.4f %12.5f" % (c, n, w, tot / n))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
