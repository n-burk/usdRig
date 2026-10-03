"""Validate the generated Shion example using the native RigExec evaluator.

    python examples/2d/bust_dd_b/verify_bust.py

Checks structural coverage, actual deformation, dynamic/baked parity, source
immutability, and the lower-lip tightening contract. Requires a built RigExec.
"""

import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "docs"))
sys.path.insert(0, str(REPO / "plugin/rigExecUsdview"))
import render_media as rm


def main():
    rigexec = rm._schema_plugin()
    from pxr import Usd, UsdGeom
    import pickerScene
    import art_face as AF
    import state as ST
    import turn as TU
    from acting import POSES

    path = HERE / "bust_dd_b_rig.usda"
    source = path.read_bytes()
    s = Usd.Stage.Open(str(path))
    s.SetEditTarget(s.GetSessionLayer())
    mesh = UsdGeom.Mesh(s.GetPrimAtPath("/Shion/Geom/Character"))
    rest = np.array(mesh.GetPointsAttr().Get())
    faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
    subsets = {
        p.GetPrim().GetName(): np.array(p.GetIndicesAttr().Get())
        for p in UsdGeom.Subset.GetAllGeomSubsets(mesh)
    }
    vertices = {name: np.unique(faces[indices]) for name, indices in subsets.items()}
    assert np.isfinite(rest).all()
    assert faces.min() >= 0 and faces.max() < len(rest)
    assert len(np.concatenate(list(subsets.values()))) == len(faces)
    assert len(np.unique(np.concatenate(list(subsets.values())))) == len(faces)
    controls = {
        p.GetName().removesuffix("_ctl"): p
        for p in s.Traverse()
        if p.GetTypeName() == "RigExecControl"
    }
    pickers = pickerScene.load_all(s)
    assert len(pickers) == 1
    assert pickers[0].coverage()[1] == 0, "Dead picker targets"
    covered = {path for button in pickers[0].buttons for path in button.targets}
    assert all(str(p.GetPath()) in covered for p in controls.values()), (
        "Unpickable controls"
    )
    touch_faces = []
    for region in s.GetPrimAtPath("/Shion/TouchRegions").GetChildren():
        indices = np.array(region.GetAttribute("rigExec:touch:faces").Get())
        assert len(indices) and indices.min() >= 0 and indices.max() < len(faces)
        targets = region.GetRelationship("rigExec:touch:control").GetTargets()
        assert (
            len(targets) == 1
            and s.GetPrimAtPath(targets[0]).GetTypeName() == "RigExecControl"
        )
        touch_faces.extend(indices)
    assert len(touch_faces) == len(faces) and len(set(touch_faces)) == len(faces), (
        "Touch coverage/overlap"
    )

    rig = rigexec.Rig(s, "/Shion/Rig")
    rig.compile()
    assert rig.is_bakeable(), rig.bakeability_reasons()

    def pose(values, mode="dynamic"):
        s.GetSessionLayer().Clear()
        for key, value in values.items():
            name, axis = key.split(".")
            controls[name].GetAttribute("avars:" + axis).Set(value)
        rig.evaluation_mode = mode
        result = rig.evaluate(1.0)
        return np.array(result.moved_properties()["/Shion/Geom/Character.points"])

    neutral = pose({})
    assert np.max(np.abs(neutral - rest)) < 0.002, "Rest pose changed"
    # A turn must reproduce the authored feature planes, not stretch the
    # eyes or ripple the bob through a shared facial depth field.
    for axis, kind, angles in (
        ("ry", "turn", (-30.0, -15.0, 15.0, 30.0)),
        ("rx", "nod", (-20.0, 20.0)),
    ):
        for angle in angles:
            turned = pose({"FaceAngle." + axis: angle})
            for part in (
                "Face",
                "Lash_L",
                "Lash_R",
                "Iris_L",
                "Iris_R",
                "MouthLine",
                "BackHair",
                "Crown",
            ):
                ids = vertices[part]
                target = TU.project_part(rest[ids, :2], part, kind, angle)
                assert np.max(np.abs(turned[ids, :2] - target)) < 0.002, (
                    part,
                    axis,
                    angle,
                )

    def areas(points, part):
        triangle = points[faces[subsets[part]], :2]
        a, b = triangle[:, 1] - triangle[:, 0], triangle[:, 2] - triangle[:, 0]
        return a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]

    rest_face_area = areas(rest, "Face")
    fist = pose({"Hand_R.curl": 1.0})
    point = pose({"Hand_R.curl": 1.0, "Index_R_0.curl": -1.0})
    for finger in ("Index", "Middle", "Ring", "Pinky"):
        part = finger + "Fold_R"
        assert np.sum(np.abs(areas(rest, part))) < 1e-6, "Visible folded pad at rest"
        assert np.sum(np.abs(areas(fist, part))) > 1.0, "Missing folded finger pad"
        if finger == "Index":
            assert np.sum(np.abs(areas(point, part))) < 1e-6, (
                "Pointing finger remained folded"
            )
        else:
            assert np.max(np.abs(point[vertices[part]] - fist[vertices[part]])) < 0.002
    maximum_error = 0.0
    for name, values in POSES.items():
        dynamic = pose(values)
        baked = pose(values, "baked")
        assert dynamic.shape == rest.shape and np.isfinite(dynamic).all(), name
        assert np.max(np.abs(dynamic[:, :2])) < 65, "Exploding geometry: " + name
        error = float(np.max(np.abs(dynamic - baked)))
        maximum_error = max(maximum_error, error)
        assert error < 0.002, (name, error)
        if values:
            assert np.max(np.abs(dynamic - neutral)) > 0.01, "Inactive pose " + name

    for ctl, part, axis, value in [
        ("Elbow_R", "Palm_R", "rz", 30.0),
        ("Hand_L", "Index_L", "turn", 30.0),
        ("Index_R_0", "Index_R", "curl", 0.8),
        ("Index_R_1", "Index_R", "rz", 15.0),
        ("Teeth", "Teeth", "ty", 0.8),
        ("Tongue", "TongueOut", "ty", 0.8),
        ("Lip", "LowerLip", "ty", 0.8),
    ]:
        base = {"Mouth.ty": -0.5} if ctl in ("Teeth", "Lip") else {}
        before = pose(base)
        after = pose(dict(base, **{ctl + "." + axis: value}))
        assert np.max(np.abs(after[vertices[part]] - before[vertices[part]])) > 0.01, (
            ctl
        )

    # Test the art construction and the final evaluated lip, including the
    # exaggerated corner that previously inflated the lower lip.
    def thickness(pts):
        pts = pts.reshape(len(AF.LLIP_V), -1, 2)
        return float(np.mean(np.linalg.norm(pts[0] - pts[-1], axis=1)))

    base_thickness = thickness(AF.lower_lip_points(ST.REST))
    previous = base_thickness
    for opening, wide in ((0.15, 0), (0.5, 0.4), (1.0, 0.8), (1.5, 1.0)):
        state = ST.S({"mouth.open": opening, "mouth.wide": wide})
        t = thickness(AF.lower_lip_points(state))
        assert t < previous, (opening, wide, t, previous)
        previous = t
        p = pose({"Mouth.ty": -opening, "Mouth.tx": wide})
        # The lip's subset contains every row vertex, in creation order.
        lip = p[vertices["LowerLip"], :2]
        assert thickness(lip) < base_thickness + 0.005, "Evaluated lip inflated"
    thin = pose({"Mouth.ty": -0.5, "Mouth.tx": 0.8})
    full = pose({"Mouth.ty": -0.5, "Mouth.tx": 0.8, "Lip.ty": 1.0})
    assert (
        thickness(full[vertices["LowerLip"], :2])
        > thickness(thin[vertices["LowerLip"], :2]) * 1.5
    )

    # Every combination of supported turn/nod endpoints must remain finite.
    for turn in (-30, 0, 30):
        for nod in (-20, 0, 20):
            p = pose(
                {
                    "FaceAngle.ry": float(turn),
                    "FaceAngle.rx": float(nod),
                    "Mouth.ty": -0.7,
                }
            )
            assert np.isfinite(p).all() and np.max(np.abs(p)) < 65
            valid = np.abs(rest_face_area) > 1e-5
            assert np.min(areas(p, "Face")[valid] / rest_face_area[valid]) > 0.02, (
                "Folded face triangle",
                turn,
                nod,
            )
    assert path.read_bytes() == source, "Evaluation authored into the source rig"
    print(
        "PASS: %d controls; picker has no dead targets; all %d faces have TouchPose regions"
        % (len(controls), len(faces))
    )
    print(
        "PASS: %d acting poses, independent fingers, authored turn shapes, no folded face triangles, lip tightening"
        % len(POSES)
    )
    print(
        "PASS: dynamic/baked max point error %.7g; source rig unchanged" % maximum_error
    )


if __name__ == "__main__":
    main()
