#!/usr/bin/env python
"""Drive a UsdSkel skin from a RigExec rig, by baking a SkelAnimation.

This is the second of the two skinning routes. `build_biped_rigexec.py
--skin-rigexec` deforms the mesh with RigExec's own movers, which is live and
interactive but composes joints sequentially rather than as a weighted sum,
so it disagrees with Maya at every blended vertex (run `compare_skinning.py`
for the numbers). This script instead lets the rigging happen in RigExec and
the *deformation* happen in UsdSkel, which is real LBS/DQS and therefore
matches Maya.

The catch is that it is a bake, not a live link. RigExec publishes joint
guides to Hydra but not joint transforms, and there is no scene-index bridge
in the repo, so the joint transforms are evaluated here and written out as
time samples. Re-run it after changing a pose.

How the transforms are converted: RigExec gives world (asset-space) joint
frames; `UsdSkelAnimation` wants parent-local translate/rotate/scale per
joint, so each joint's world frame is divided by its parent's:

    local = world * inverse(world_of_parent)

The joint order is taken from the Skeleton so the arrays line up by index.

Usage:
    bake_rigexec_to_skel.py <rig.usda> <skel.usda> <out.usda> [--frames a b]

`out.usda` sublayers the skin asset and carries the SkelAnimation, so the
original stays untouched.
"""
import argparse
import os
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt

import rigexec


def bake(rig_path, skel_path, out_path, frames, rig_root, skel_prim_path):
    rigexec.load_schema_plugin()

    rig_stage = Usd.Stage.Open(rig_path)
    rig = rigexec.Rig(rig_stage, rig_root)
    rig.compile()

    skel_stage = Usd.Stage.Open(skel_path)
    skel = UsdSkel.Skeleton(skel_stage.GetPrimAtPath(skel_prim_path))
    if not skel:
        raise SystemExit("no Skeleton at %s in %s"
                         % (skel_prim_path, skel_path))
    joints = list(skel.GetJointsAttr().Get())
    print("skeleton joints: %d" % len(joints))

    # Map the Skeleton's joint tokens onto RigExec joint prim paths. The
    # Skeleton stores hierarchical tokens ("hips_bind/spine_0_bind/..."),
    # and the RigExec joints are nested the same way under /Joints.
    rig_path_of = {}
    for token in joints:
        rig_path_of[token] = "%s/Joints/%s" % (rig_root, token)

    missing = [t for t in joints
               if not rig_stage.GetPrimAtPath(rig_path_of[t]).IsValid()]
    if missing:
        raise SystemExit("%d skeleton joints have no RigExec prim, first few: "
                         "%s" % (len(missing), ", ".join(missing[:5])))
    print("all %d joints resolve to RigExec prims" % len(joints))

    # Parent index per joint, from the token hierarchy.
    index_of = {t: i for i, t in enumerate(joints)}
    parent_index = []
    for t in joints:
        parent_index.append(index_of[t.rsplit("/", 1)[0]]
                            if "/" in t else -1)

    out = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.y)
    UsdGeom.SetStageMetersPerUnit(out, 0.01)
    rel = os.path.relpath(skel_path, os.path.dirname(
        os.path.abspath(out_path))).replace("\\", "/")
    out.GetRootLayer().subLayerPaths.append(rel)

    anim = UsdSkel.Animation.Define(out, "%s/Anim" % skel_prim_path)
    anim.CreateJointsAttr(Vt.TokenArray(joints))
    t_attr = anim.CreateTranslationsAttr()
    r_attr = anim.CreateRotationsAttr()
    s_attr = anim.CreateScalesAttr()

    for frame in frames:
        pose = rig.evaluate(float(frame))
        worlds = []
        for t in joints:
            m = pose.joint_frame(rig_path_of[t]).to_matrix4()
            worlds.append(Gf.Matrix4d(*m))

        trans, rots, scales = [], [], []
        for i, t in enumerate(joints):
            pi = parent_index[i]
            local = (worlds[i] if pi < 0
                     else worlds[i] * worlds[pi].GetInverse())
            trans.append(Gf.Vec3f(local.ExtractTranslation()))
            q = local.ExtractRotationQuat().GetNormalized()
            rots.append(Gf.Quatf(float(q.GetReal()),
                                 Gf.Vec3f(q.GetImaginary())))
            # Rests are orthonormalised by RigExec, so scale is unit; read
            # it back anyway rather than assuming.
            f = Gf.Matrix4d(local)
            sx = Gf.Vec3d(f[0][0], f[0][1], f[0][2]).GetLength()
            sy = Gf.Vec3d(f[1][0], f[1][1], f[1][2]).GetLength()
            sz = Gf.Vec3d(f[2][0], f[2][1], f[2][2]).GetLength()
            scales.append(Gf.Vec3h(sx, sy, sz))

        tc = Usd.TimeCode(float(frame)) if len(frames) > 1 \
            else Usd.TimeCode.Default()
        t_attr.Set(Vt.Vec3fArray(trans), tc)
        r_attr.Set(Vt.QuatfArray(rots), tc)
        s_attr.Set(Vt.Vec3hArray(scales), tc)
        print("  baked frame %s" % frame)

    # Bind the animation on the skeleton (an over, so the source asset is
    # not modified).
    over = out.OverridePrim(skel_prim_path)
    UsdSkel.BindingAPI.Apply(over).CreateAnimationSourceRel().SetTargets(
        [anim.GetPrim().GetPath()])

    if len(frames) > 1:
        out.SetStartTimeCode(float(min(frames)))
        out.SetEndTimeCode(float(max(frames)))
    out.GetRootLayer().Save()
    print("wrote %s (%.2f MB)" % (out_path, os.path.getsize(out_path) / 1e6))

    # --- Verify: with the rig at rest the skin must not move. ---
    check = Usd.Stage.Open(out_path)
    cache = UsdSkel.Cache()
    root = UsdSkel.Root(check.GetPrimAtPath("/Biped"))
    cache.Populate(root, Usd.PrimDefaultPredicate)
    cskel = UsdSkel.Skeleton(check.GetPrimAtPath(skel_prim_path))
    query = cache.GetSkelQuery(cskel)
    mesh_prim = None
    for p in check.Traverse():
        if p.GetTypeName() == "Mesh":
            mesh_prim = p
            break
    sq = cache.GetSkinningQuery(mesh_prim)
    if not (query and sq):
        print("WARNING: could not build a skel/skinning query to verify")
        return 0

    pts = UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()
    tc = Usd.TimeCode(float(frames[0])) if len(frames) > 1 \
        else Usd.TimeCode.Default()
    xf = query.ComputeSkinningTransforms(tc)
    moved = Vt.Vec3fArray(list(pts))
    sq.ComputeSkinnedPoints(Vt.Matrix4dArray(xf), moved, tc)
    worst = max(max(abs(moved[i][k] - pts[i][k]) for k in range(3))
                for i in range(len(pts)))
    print("\nrest check: skin moves by %.4g cm at frame %s  [%s]"
          % (worst, frames[0], "PASS" if worst < 1e-2 else "see note"))
    if worst >= 1e-2:
        print("  (non-zero is expected if the rig was posed rather than "
              "left at rest)")
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("rig")
    ap.add_argument("skel")
    ap.add_argument("out")
    ap.add_argument("--frames", nargs="*", type=float, default=[0.0])
    ap.add_argument("--rig-root", default="/Biped/Rig")
    ap.add_argument("--skel-prim", default="/Biped/Skeleton")
    args = ap.parse_args(argv)
    return bake(args.rig, args.skel, args.out, args.frames,
                args.rig_root, args.skel_prim)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
