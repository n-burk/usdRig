#!/usr/bin/env python
"""Convert the Maya biped's bind skeleton and skinCluster into UsdSkel.

Phase 1 of the Maya->RigExec biped port: bones and skinning only, with no
RigExec involvement. Stock UsdSkel is used deliberately here, because it is
the only path that both deforms correctly today and gives the later RigExec
work a ground-truth reference to be compared against -- RigExec's own
skinning composes one weighted matrix per joint over the *previous* revision,
which is not linear blend skinning and will not match Maya at blended
vertices.

Ground truth is the Maya NXT build (`biped_default_rig.nxt` ->
`body_rig.nxt` -> `base/anim_rig.nxt`), which:

  * imports `build/skeleton.ma` for the 112-joint bind hierarchy, then
    applies `build/joint_positions.data` in LOCAL space over it, then
    procedurally creates the twist/noTwist/toe helper joints;
  * binds with `mc.skinCluster(..., tsb=True)` and imports
    `build/skin_wts/*.xml` via `mc.deformerWeights(..., im=True)`.

So `joint_positions.data` is the authority for joint rests -- it is a
superset of `skeleton.ma` (228 entries vs 112) and it alone carries the
helper joints that make up 54 of the body's 137 skin influences. A skeleton
built from `skeleton.ma` would silently lose that bind.

Usage:
    build_biped_skel.py <mesh_manifest.json> <out.usda> [--mesh body_geo]

`mesh_manifest.json` comes from `extract_maya_mesh.py`, run under Blender.
Its points are in metres (the FBX is centimetres and Blender imports 1cm as
1m), so they are scaled by 100 back into the source centimetre space that
the joints and the weight XML both use; the stage carries
`metersPerUnit = 0.01` rather than rescaling the data.
"""
import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter

from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt

# The biped template's rig data. Overridable for other characters.
DATA = ("C:/Users/walte/Documents/dev/squarebit/templates/templates/maya/"
        "biped/rig/default/data/build")

_ROT_ORDERS = ("xyz", "yzx", "zxy", "xzy", "yxz", "zyx")

# Blender imports the centimetre FBX as though it were metres.
_FBX_TO_CM = 100.0


def _axis_rotation(axis, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(axis, degrees))
    return m


def euler_matrix(rot, order="xyz"):
    """Maya Euler triple (degrees) -> row-vector rotation matrix.

    Maya's rotate order names the order the axes are applied in, which in
    row-vector composition is the same left-to-right order: "xyz" is
    Rx * Ry * Rz. USD's GfMatrix4d is row-vector too, so no transpose is
    needed anywhere in this file.
    """
    axes = {
        "x": _axis_rotation(Gf.Vec3d(1, 0, 0), rot[0]),
        "y": _axis_rotation(Gf.Vec3d(0, 1, 0), rot[1]),
        "z": _axis_rotation(Gf.Vec3d(0, 0, 1), rot[2]),
    }
    m = Gf.Matrix4d(1.0)
    for ch in order:
        m = m * axes[ch]
    return m


def local_matrix(rec):
    """The joint's local (parent-relative) rest transform.

    Maya composes a joint as scale * rotateAxis * rotate * jointOrient *
    translate. No joint in this data sets rotateAxis, and every scale is 1,
    so this reduces to rotate * jointOrient * translate -- with `rotate`
    honouring the joint's own rotateOrder (only the two wrists differ) and
    jointOrient always XYZ.
    """
    order = _ROT_ORDERS[int(rec.get("rotateOrder", 0))]
    m = euler_matrix(rec.get("rotate", (0, 0, 0)), order)
    m = m * euler_matrix(rec.get("jointOrient", (0, 0, 0)), "xyz")
    m = m * Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(*rec["translate"]))
    return m


def load_joints(path):
    """Parse joint_positions.data into {name: record} plus parent links."""
    blob = json.load(open(path))
    if blob.get("type") != "JointData":
        raise SystemExit("%s is not JointData (got %r)"
                         % (path, blob.get("type")))
    data = blob["data"]
    parent = {}
    for name, rec in data.items():
        chain = [p for p in rec["dagPath"].split("|") if p]
        # dagPath is |bind|hips_bind|...|<name>; the parent is the entry
        # before it. Chains rooted somewhere other than `bind` (the foot and
        # pupil pivots) are handled by the reachability pass below.
        parent[name] = chain[-2] if len(chain) >= 2 else None
    return data, parent


def skeleton_order(data, parent, root_token="bind"):
    """Joints reachable from `bind`, parents strictly before children.

    UsdSkel requires every joint's ancestors to precede it, and requires the
    joint token paths to form a real hierarchy. Anything whose dagPath does
    not resolve up to `bind` is dropped -- that is the foot-roll and pupil
    pivot chains, plus the iris/pupil joints hanging off `iris_?_nul`
    transforms that the data file does not contain. None of those are skin
    influences.
    """
    ordered = []
    placed = {}
    pending = set(data)

    def resolve(name, seen):
        if name in placed:
            return placed[name]
        if name in seen or name not in data:
            return None
        seen.add(name)
        p = parent.get(name)
        if p is None or p == root_token:
            path = name
        else:
            ppath = resolve(p, seen)
            if ppath is None:
                return None
            path = ppath + "/" + name
        placed[name] = path
        ordered.append((name, path))
        return path

    for name in sorted(pending):
        resolve(name, set())
    # Sort by depth so parents precede children (stable within a depth).
    ordered.sort(key=lambda np: np[1].count("/"))
    return ordered


def parse_weights(path):
    """Maya deformerWeights XML -> (influence order, {influence: {vtx: w}}).

    Influences are identified by the `source` attribute; `layer` is their
    skinCluster matrix index at export time, which is NOT the skeleton's
    joint order, so callers must remap by name.
    """
    root = ET.parse(path).getroot()
    deformer = root.find("deformer")
    attrs = {}
    if deformer is not None:
        for a in deformer.findall("attribute"):
            attrs[a.get("name")] = a.get("value")

    shape = root.find("shape")
    rest = {}
    if shape is not None:
        for p in shape.findall("point"):
            rest[int(p.get("index"))] = [
                float(x) for x in p.get("value").split()]

    influences = []
    weights = {}
    for w in root.findall("weights"):
        src = w.get("source")
        influences.append((int(w.get("layer", len(influences))), src))
        table = {}
        for p in w.findall("point"):
            table[int(p.get("index"))] = float(p.get("value"))
        weights[src] = table
    influences.sort()
    return [s for _, s in influences], weights, attrs, rest


def build(manifest_path, out_path, mesh_name, reframe=True):
    data, parent = load_joints(os.path.join(DATA, "joint_positions.data"))
    ordered = skeleton_order(data, parent)
    joint_paths = [p for _, p in ordered]
    joint_names = [n for n, _ in ordered]
    index_of = {n: i for i, n in enumerate(joint_names)}
    print("joints: %d of %d entries reachable from 'bind'"
          % (len(ordered), len(data)))

    # Rest (local) and bind (world) transforms.
    #
    # By default the limb root/mid frames are replaced with the basis
    # RigExecTwoBoneIk publishes, which the right side otherwise misses by
    # 180 degrees. This MUST match what build_biped_rigexec.py authors as
    # rest:space, or bake_rigexec_to_skel.py bakes that roll into the skin
    # -- see tools/biped/limb_frames.py for the proof that it leaves
    # deformation unchanged. Origins are untouched either way.
    if reframe:
        from limb_frames import reframe_limbs
        print("re-framing limb joints for RigExecTwoBoneIk:")
        worlds, locals_, _poles = reframe_limbs(data, parent, ordered)
        rests = [locals_[n] for n in joint_names]
    else:
        print("NOT re-framing (--no-reframe): these rests keep Maya's "
              "frames and will NOT agree with a re-framed RigExec rig")
        rests = []
        worlds = {}
        for name, _path in ordered:
            lm = local_matrix(data[name])
            rests.append(lm)
            p = parent.get(name)
            worlds[name] = lm if p not in worlds else lm * worlds[p]
    binds = [worlds[n] for n in joint_names]

    # Cross-check the FK composition against the world_translate the data
    # file also stores. This catches a rotate-order or jointOrient mistake
    # immediately rather than as mysterious skin drift later.
    worst, worst_at = 0.0, None
    for name in joint_names:
        want = data[name].get("world_translate")
        if not want:
            continue
        got = worlds[name].ExtractTranslation()
        d = max(abs(got[i] - want[i]) for i in range(3))
        if d > worst:
            worst, worst_at = d, name
    print("FK vs stored world_translate: worst %.6g cm (%s)" % (worst, worst_at))
    if worst > 1e-3:
        raise SystemExit("FK composition disagrees with world_translate -- "
                         "rotate order or jointOrient handling is wrong")

    # Mesh.
    man = json.load(open(manifest_path))
    if mesh_name not in man:
        raise SystemExit("%s not in manifest (have: %s)"
                         % (mesh_name, ", ".join(sorted(man))))
    mesh = man[mesh_name]
    pts = [Gf.Vec3f(mesh["points"][i * 3] * _FBX_TO_CM,
                    mesh["points"][i * 3 + 1] * _FBX_TO_CM,
                    mesh["points"][i * 3 + 2] * _FBX_TO_CM)
           for i in range(mesh["pointCount"])]

    # Weights.
    wts_file = os.path.join(DATA, "skin_wts",
                            "%s__%s_skinCluster.xml" % (mesh_name, mesh_name))
    inf_order, tables, skin_attrs, rest_pts = parse_weights(wts_file)
    print("influences: %d (%s)" % (len(inf_order), wts_file.split("/")[-1]))

    missing = [s for s in inf_order if s not in index_of]
    if missing:
        raise SystemExit("influences absent from the skeleton: %s"
                         % ", ".join(missing))

    # Verify the mesh really is the one these weights were exported against.
    if rest_pts:
        worst = max(
            max(abs(pts[i][k] - rest_pts[i][k]) for k in range(3))
            for i in rest_pts if i < len(pts))
        print("mesh vs XML rest points: worst %.6g cm" % worst)
        if worst > 1e-2:
            raise SystemExit("mesh does not match the weight file's rest "
                             "points -- wrong mesh, scale or vertex order")

    # Invert to per-vertex influence lists.
    per_vertex = [[] for _ in pts]
    for src in inf_order:
        ji = index_of[src]
        for vtx, w in tables[src].items():
            if w != 0.0 and vtx < len(per_vertex):
                per_vertex[vtx].append((ji, w))

    counts = Counter(len(v) for v in per_vertex)
    element_size = max(counts)
    print("influences/vertex: max %d, histogram %s"
          % (element_size, dict(sorted(counts.items()))))
    unweighted = counts.get(0, 0)
    if unweighted:
        print("WARNING: %d vertices carry no weight" % unweighted)

    # Pad to a fixed elementSize and renormalise. The XML stores weights
    # rounded to 3 decimals, so per-vertex sums drift off 1.0 and must be
    # renormalised regardless of truncation.
    idx_arr, w_arr = [], []
    renormalised = 0
    for v in per_vertex:
        v = sorted(v, key=lambda iw: -iw[1])[:element_size]
        total = sum(w for _, w in v)
        if v and abs(total - 1.0) > 1e-6:
            v = [(i, w / total) for i, w in v]
            renormalised += 1
        while len(v) < element_size:
            v.append((0, 0.0))
        idx_arr.extend(i for i, _ in v)
        w_arr.extend(w for _, w in v)
    print("renormalised %d of %d vertices" % (renormalised, len(per_vertex)))

    # Author the stage.
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    # Source data is centimetres; declare the unit instead of rescaling so
    # points, joints and weights all stay byte-comparable with Maya.
    UsdGeom.SetStageMetersPerUnit(stage, 0.01)

    root = UsdSkel.Root.Define(stage, "/Biped")
    stage.SetDefaultPrim(root.GetPrim())

    skel = UsdSkel.Skeleton.Define(stage, "/Biped/Skeleton")
    skel.CreateJointsAttr(Vt.TokenArray(joint_paths))
    skel.CreateRestTransformsAttr(Vt.Matrix4dArray(rests))
    skel.CreateBindTransformsAttr(Vt.Matrix4dArray(binds))

    # Type the intermediate scope explicitly. UsdGeom.Mesh.Define would leave
    # /Biped/Geom typeless, and UsdSkelCache's traversal stops at a typeless
    # prim -- ComputeSkelBindings then returns nothing and the mesh is never
    # skinned, even though UsdSkelIsSkinnablePrim still reports it skinnable.
    UsdGeom.Scope.Define(stage, "/Biped/Geom")
    geom = UsdGeom.Mesh.Define(stage, "/Biped/Geom/%s" % mesh_name)
    geom.CreatePointsAttr(Vt.Vec3fArray(pts))
    geom.CreateFaceVertexCountsAttr(Vt.IntArray(mesh["faceVertexCounts"]))
    geom.CreateFaceVertexIndicesAttr(Vt.IntArray(mesh["faceVertexIndices"]))
    geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    # A skinned mesh's extent is the bind-pose extent; UsdSkelImaging
    # recomputes as it deforms.
    geom.CreateExtentAttr(Vt.Vec3fArray(
        UsdGeom.PointBased(geom).ComputeExtent(Vt.Vec3fArray(pts))))

    binding = UsdSkel.BindingAPI.Apply(geom.GetPrim())
    binding.CreateSkeletonRel().SetTargets([skel.GetPrim().GetPath()])
    binding.CreateJointIndicesPrimvar(False, element_size).Set(
        Vt.IntArray(idx_arr))
    binding.CreateJointWeightsPrimvar(False, element_size).Set(
        Vt.FloatArray(w_arr))
    # headerInfo worldMatrix in the XML is identity and the rest points are
    # already in skeleton space, so the bind transform is identity.
    binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))

    # skinningMethod=1 in Maya's enum is dual quaternion.
    method = skin_attrs.get("skinningMethod")
    if method == "1":
        binding.CreateSkinningMethodAttr(UsdSkel.Tokens.dualQuaternion)
        print("skinningMethod: dualQuaternion (Maya enum 1)")
    elif method is not None:
        binding.CreateSkinningMethodAttr(UsdSkel.Tokens.classicLinear)
        print("skinningMethod: classicLinear (Maya enum %s)" % method)

    # Materials from the FBX. body_geo carries five of them, so this
    # authors GeomSubsets rather than one binding -- see materials.py.
    try:
        from materials import apply_materials
        print("materials:")
        apply_materials(stage, mesh_name, "/Biped/Geom/%s" % mesh_name)
    except SystemExit as e:
        print("materials: skipped (%s)" % e)

    stage.GetRootLayer().Save()
    print("wrote %s (%.1f MB)"
          % (out_path, os.path.getsize(out_path) / 1e6))
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("manifest")
    ap.add_argument("out")
    ap.add_argument("--mesh", default="body_geo")
    ap.add_argument("--no-reframe", dest="reframe", action="store_false",
                    help="keep Maya's limb frames instead of the basis "
                         "RigExecTwoBoneIk expects; only for comparing "
                         "against the original data")
    args = ap.parse_args(argv)
    return build(args.manifest, args.out, args.mesh, args.reframe)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
