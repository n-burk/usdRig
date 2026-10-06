"""Shion's authored control ranges, character picker and TouchPose regions."""

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

import art_limbs as AL


def merge_character(b):
    """One drawable mesh lets TouchPose traverse the entire layered character.

    Each original deformation stack gets a disjoint sparse point envelope.
    Blend offsets and skin rows are remapped to the unified vertex domain.
    Materials retain their original per-layer subsets and depth ordering.
    """
    from build_bust import GEOM, MOV, RIG, TARGETS, attr, define, rel, v3f

    s = b.stage
    dest = GEOM + "/Character"
    points, indices, uv, parts = [], [], [], []
    offsets, ranges = {}, {}
    vo, fo = 0, 0
    for name, data in b.mesh.items():
        mesh = UsdGeom.Mesh(s.GetPrimAtPath(GEOM + "/" + name))
        p = np.array(mesh.GetPointsAttr().Get())
        f = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        points.append(p)
        indices.append(f + vo)
        uv.append(np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").Get()))
        offsets[name] = vo
        ranges[name] = (vo, vo + len(p))
        for sub in UsdGeom.Subset.GetAllGeomSubsets(mesh):
            mat, _ = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()
            parts.append(
                (
                    sub.GetPrim().GetName(),
                    np.array(sub.GetIndicesAttr().Get()) + fo,
                    mat,
                )
            )
        vo += len(p)
        fo += len(f)
    mesh = UsdGeom.Mesh.Define(s, dest)
    pts = np.vstack(points)
    mesh.CreatePointsAttr(v3f(pts))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([3] * fo))
    mesh.CreateFaceVertexIndicesAttr(
        Vt.IntArray.FromNumpy(np.vstack(indices).ravel().astype(np.int32))
    )
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim())
    mesh.CreateExtentAttr(v3f([pts.min(axis=0) - 5, pts.max(axis=0) + 5]))
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex"
    ).Set(Vt.Vec2fArray.FromNumpy(np.vstack(uv).astype(np.float32)))
    for name, faces, mat in parts:
        sub = UsdGeom.Subset.CreateGeomSubset(
            mesh,
            name,
            "face",
            Vt.IntArray.FromNumpy(faces.astype(np.int32)),
            "materialBind",
            "nonOverlapping",
        )
        UsdShade.MaterialBindingAPI.Apply(sub.GetPrim()).Bind(mat)
    UsdGeom.Subset.SetFamilyType(mesh, "materialBind", "partition")
    for name, (a, z) in ranges.items():
        path = RIG + "/Weights/" + name
        w = define(s, path, "RigExecStaticWeight")
        rel(w, "rigExec:weightTarget", [dest + ".points"])
        attr(w, "rigExec:representation", Sdf.ValueTypeNames.Token, "sparse", True)
        attr(
            w,
            "rigExec:indices",
            Sdf.ValueTypeNames.IntArray,
            Vt.IntArray.FromNumpy(np.arange(a, z, dtype=np.int32)),
            True,
        )
        attr(
            w,
            "rigExec:values",
            Sdf.ValueTypeNames.FloatArray,
            Vt.FloatArray([1.0] * (z - a)),
            True,
        )
        attr(w, "rigExec:defaultWeight", Sdf.ValueTypeNames.Float, 0.0, True)
        for m in s.GetPrimAtPath(MOV + "/Geometry/" + name).GetChildren():
            rel(m, "rigExec:moves", [dest + ".points"])
            rel(m, "rigExec:weightObject", [path])
            if m.GetTypeName() == "RigExecSkinMover":
                es = m.GetAttribute("rigExec:elementSize").Get()
                for prop, dtype in (
                    ("jointIndices", np.int32),
                    ("jointWeights", np.float32),
                ):
                    original = np.array(
                        m.GetAttribute("rigExec:" + prop).Get()
                    ).reshape(-1, es)
                    full = np.zeros((vo, es), dtype=dtype)
                    if prop == "jointWeights":
                        full[:, 0] = 1
                    full[a:z] = original
                    array = Vt.IntArray if dtype == np.int32 else Vt.FloatArray
                    m.GetAttribute("rigExec:" + prop).Set(array.FromNumpy(full.ravel()))
        root = s.GetPrimAtPath(TARGETS + "/" + name)
        if root:
            for shape in root.GetChildren():
                attr_indices = shape.GetAttribute("pointIndices")
                if attr_indices:
                    attr_indices.Set(
                        Vt.IntArray.FromNumpy(
                            (np.array(attr_indices.Get()) + a).astype(np.int32)
                        )
                    )
        s.RemovePrim(GEOM + "/" + name)
    b.character_parts = parts


def ranges(b):
    from build_bust import attr, T

    specs = {
        "Body": {"tx": (-2, 2, "Body sway"), "rz": (-8, 8, "Body lean")},
        "Neck": {"rz": (-8, 8, "Neck bend")},
        "Head": {"rz": (-18, 18, "Head roll")},
        "FaceAngle": {"ry": (-30, 30, "Head turn"), "rx": (-20, 20, "Head nod")},
        "Look": {
            "tx": (-1, 1, "Look horizontal"),
            "ty": (-1, 1, "Look vertical"),
            "tz": (0, 1, "Iris contraction"),
        },
        "Mouth": {
            "ty": (-1, 0, "Jaw open (negative)"),
            "tx": (-1, 1, "Round / wide"),
            "rz": (-20, 20, "Frown / smile"),
            "ry": (-20, 20, "Asymmetric smile"),
        },
        "Teeth": {"ty": (0, 1, "Teeth reveal")},
        "Tongue": {"ty": (0, 1, "Tongue protrusion")},
        "Lip": {"ty": (0, 1, "Lower lip fullness")},
        "Torso": {"ty": (0, 1, "Breathing"), "ry": (-10, 10, "Torso turn")},
        "Fx": {"tx": (0, 1, "Blush"), "ty": (0, 1, "Sweat"), "tz": (0, 1, "Hatching")},
    }
    for tag in ("L", "R"):
        specs["Eye_" + tag] = {
            "ty": (-1, 1, "Close / widen"),
            "tx": (0, 1, "Smile eye"),
            "tz": (0, 1, "Relax lid"),
        }
        specs["Brow_" + tag] = {
            "ty": (-1, 1, "Brow height"),
            "rz": (-20, 20, "Angry / worried"),
        }
        specs["Shoulder_" + tag] = {"rz": (-25, 25, "Shoulder swing")}
        specs["Elbow_" + tag] = {
            "rz": (-100 if tag == "L" else -35, 35 if tag == "L" else 100, "Elbow bend")
        }
        specs["Hand_" + tag] = {
            "rz": (-55, 55, "Wrist bend"),
            "tz": (-1, 2, "Hand depth"),
            "turn": (-35, 35, "Palm turn"),
            "curl": (0, 1, "Hand curl"),
        }
        for name in AL.FINGERS:
            for i in range(3):
                specs[name + "_" + tag + "_%d" % i] = {
                    "rz": (-25, 25, "Finger bend / spread")
                }
            specs[name + "_" + tag + "_0"]["curl"] = (-1, 1, "Finger curl offset")
    for key, path in b.ctl.items():
        p = b.stage.GetPrimAtPath(path)
        for axis, (lo, hi, label) in specs.get(key, {}).items():
            a = attr(p, "avars:" + axis, T.Double, 0.0)
            a.SetDisplayName(label)
            a.SetDisplayGroup("Pose")
            a.SetCustomDataByKey(
                "limits",
                dict(
                    soft=dict(minimum=float(lo), maximum=float(hi)),
                    hard=dict(
                        minimum=-1.5 if key == "Mouth" and axis == "ty" else float(lo),
                        maximum=float(hi),
                    ),
                ),
            )
        p.SetDocumentation(
            "Shion: "
            + key.replace("_", " ")
            + ". See the example README for supported channels and ranges."
        )


def picker(b):
    from build_bust import ROOT, RIG, attr, define, rel, T

    s = b.stage
    base = ROOT + "/Picker"
    root = define(s, base, "RigExecPicker")
    attr(root, "ui:label", T.String, "Shion", True)
    rel(root, "rigExec:picker:rig", [RIG])

    def panel(name, index, size=(520, 620)):
        p = define(s, base + "/" + name, "RigExecPickerPanel")
        attr(p, "ui:label", T.String, name, True)
        attr(p, "ui:order", T.Int, index, True)
        attr(p, "ui:size", T.Float2, Gf.Vec2f(*size), True)
        attr(p, "ui:background", T.Color4f, Gf.Vec4f(0.075, 0.09, 0.13, 1), True)
        return str(p.GetPath())

    def button(
        parent, name, label, keys, xy, size=(100, 32), color=(0.30, 0.49, 0.65, 1)
    ):
        p = define(s, parent + "/" + name, "RigExecPickerButton")
        for k, t, v in (
            ("position", T.Float2, Gf.Vec2f(*xy)),
            ("size", T.Float2, Gf.Vec2f(*size)),
            ("fill", T.Color4f, Gf.Vec4f(*color)),
            ("textColor", T.Color4f, Gf.Vec4f(1, 1, 1, 1)),
            ("text", T.String, label),
            ("fontSize", T.Float, 12.0),
            ("shape", T.Token, "roundedRectangle"),
            ("roundness", T.Float, 0.2),
        ):
            attr(p, "ui:" + k, t, v, True)
        if keys:
            rel(p, "rigExec:picker:controls", [b.ctl.get(k, k) for k in keys])
        return p

    p = panel("Face", 0)
    button(p, "Title", "SHION  /  FACE", [], (20, 15), (480, 36), (0.13, 0.18, 0.24, 1))
    layout = [
        ("FaceAngle", "Turn / nod", (205, 75)),
        ("Head", "Head roll", (205, 120)),
        ("Brow_L", "Brow L", (80, 180)),
        ("Brow_R", "Brow R", (340, 180)),
        ("Eye_L", "Eye L", (80, 230)),
        ("Eye_R", "Eye R", (340, 230)),
        ("Look", "Gaze", (205, 260)),
        ("Mouth", "Mouth", (205, 330)),
        ("Teeth", "Teeth", (75, 385)),
        ("Tongue", "Tongue", (205, 385)),
        ("Lip", "Lip fullness", (335, 385)),
        ("Fx", "Expression FX", (205, 445)),
    ]
    for k, label, xy in layout:
        button(p, k, label, [k], xy)
    button(p, "BothEyes", "Both eyes", ["Eye_L", "Eye_R"], (75, 510), (160, 32))
    button(p, "BothBrows", "Both brows", ["Brow_L", "Brow_R"], (275, 510), (160, 32))
    p = panel("Body", 1)
    for k, xy in (
        ("Head", (205, 65)),
        ("Neck", (205, 125)),
        ("Torso", (205, 230)),
        ("Body", (205, 410)),
        ("Shoulder_L", (65, 185)),
        ("Shoulder_R", (355, 185)),
        ("Elbow_L", (30, 280)),
        ("Elbow_R", (390, 280)),
        ("Hand_L", (65, 360)),
        ("Hand_R", (355, 360)),
    ):
        button(p, k, k.replace("_", " "), [k], xy)
    button(
        p,
        "AllBody",
        "Body + arms",
        [k for k in b.ctl if k.startswith(("Shoulder", "Elbow", "Hand"))]
        + ["Body", "Neck", "Head"],
        (140, 485),
        (240, 36),
    )
    p = panel("Hands", 2)
    for tag, x in (("L", 10), ("R", 270)):
        button(
            p,
            "Hand_" + tag,
            "HAND " + tag + " / curl + turn",
            ["Hand_" + tag],
            (x, 20),
            (240, 40),
        )
        for row, name in enumerate(AL.FINGERS):
            for i in range(3):
                key = name + "_" + tag + "_%d" % i
                button(
                    p,
                    key,
                    name if i == 0 else str(i + 1),
                    [key],
                    (x + i * 80, 90 + row * 75),
                    (75, 38),
                )
        button(
            p,
            "All_" + tag,
            "All fingers " + tag,
            [name + "_" + tag + "_%d" % i for name in AL.FINGERS for i in range(3)],
            (x, 490),
            (240, 38),
        )
    p = panel("Secondary", 3)
    for i, (name, chain) in enumerate(sorted(b.chains.items())):
        for j, path in enumerate(chain["controls"]):
            button(
                p,
                name + "_%d" % j,
                name + " " + str(j + 1),
                [path],
                (12 + j * 82, 15 + i * 35),
                (78, 29),
            )
    attr(
        s.GetPrimAtPath(p),
        "ui:size",
        T.Float2,
        Gf.Vec2f(600, 35 * len(b.chains) + 35),
        True,
    )


def touch(b):
    from build_bust import ROOT, GEOM, attr, define, rel, T

    scope = define(b.stage, ROOT + "/TouchRegions", "RigExecTouchRegions")
    rel(scope, "rigExec:touch:mesh", [GEOM + "/Character"])
    palette = [
        (0.3, 0.8, 0.95),
        (0.95, 0.64, 0.30),
        (0.7, 0.46, 0.86),
        (0.46, 0.82, 0.63),
    ]
    attr(
        scope,
        "rigExec:touch:palette",
        T.Color3fArray,
        Vt.Vec3fArray([Gf.Vec3f(*c) for c in palette]),
        True,
    )
    for name, faces, _ in b.character_parts:
        key = "Head"
        if name in b.chains:
            key = b.chains[name]["controls"][0]
        elif name.startswith("UpperSleeve_"):
            key = "Shoulder_" + name[-1]
        elif name.startswith("ForeSleeve_"):
            key = "Elbow_" + name[-1]
        elif name.startswith("Palm_"):
            key = "Hand_" + name[-1]
        elif name.split("_")[0].removesuffix("Fold") in AL.FINGERS:
            key = name.replace("Fold_", "_") + "_0"
        elif name.startswith(("Iris_", "Highlight_")):
            key = "Look"
        elif name.startswith(
            ("Sclera_", "EyeMask_", "Lash_", "LidShadow_", "Crease_", "LowerLid_")
        ):
            key = "Eye_" + name[-1]
        elif name.startswith("Brow_"):
            key = name
        elif name == "Teeth":
            key = "Teeth"
        elif name in ("Tongue", "TongueOut"):
            key = "Tongue"
        elif name == "LowerLip":
            key = "Lip"
        elif name.startswith(("Mouth", "LowerEdge")):
            key = "Mouth"
        elif name == "Neck":
            key = "Neck"
        elif name.startswith(("Cardigan", "Shirt", "Collar", "Ribbon")):
            key = "Torso" if name == "Shirt" else "Body"
        elif name == "Face":
            key = "FaceAngle"
        elif name.startswith(("Blush", "Hatch", "Sweat")):
            key = "Fx"
        region = define(
            b.stage, str(scope.GetPath()) + "/" + name, "RigExecTouchRegion"
        )
        attr(
            region,
            "rigExec:touch:faces",
            T.IntArray,
            Vt.IntArray.FromNumpy(faces.astype(np.int32)),
            True,
        )
        rel(region, "rigExec:touch:control", [b.ctl.get(key, key)])


def author(b):
    ranges(b)
    merge_character(b)
    picker(b)
    touch(b)
    camera = UsdGeom.Camera.Define(b.stage, "/Shion/Camera")
    camera.CreateProjectionAttr("orthographic")
    camera.CreateHorizontalApertureAttr(900.0)
    camera.CreateVerticalApertureAttr(700.0)
    camera.CreateClippingRangeAttr(Gf.Vec2f(1, 500))
    camera.AddTranslateOp().Set(Gf.Vec3d(0, -20, 100))
