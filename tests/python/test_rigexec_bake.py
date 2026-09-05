"""Standalone USD export, including nested posed frames and reset stacks."""

import os
import sys
import tempfile

from test_rigexec_python import _setup_environment
_setup_environment()

from pxr import Gf, Sdf, Usd, UsdGeom
import rigexec


def close(a, b, epsilon=1e-5):
    return all(abs(a[r][c] - b[r][c]) < epsilon for r in range(4) for c in range(4))


def main():
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    asset.AddRotateZOp().Set(20.0)
    asset.AddScaleOp().Set(Gf.Vec3f(2, 1.5, 1))
    group = UsdGeom.Xform.Define(stage, "/Asset/Group")
    group.AddTranslateOp().Set(Gf.Vec3d(3, 2, 1))
    group.AddScaleOp().Set(Gf.Vec3f(1, 2, 1))
    nested = UsdGeom.Points.Define(stage, "/Asset/Group/Nested")
    nested.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    nested.AddTranslateOp().Set(Gf.Vec3d(-1, 0, 0))
    nested.SetResetXformStack(True)
    primvar = UsdGeom.PrimvarsAPI(nested).CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant)
    primvar.Set([(0.2, 0.4, 0.7)])
    builder = rigexec.Builder.create(stage, "/Asset/Rig")
    ctrl = builder.add_control("Parent")
    child_path = ctrl.path + "/Child"
    child = rigexec.schema.Control.define(stage, child_path)
    child.set_attribute("rest:tx", 4.0)
    child.set_attribute("default:ty", 2.0)
    child.set_attribute("avars:unitScaleFactor", 2.0)
    joint = builder.add_joint("J0")
    joint_child = builder.add_joint("J1", parent_joint=joint)
    stage.GetPrimAtPath(joint_child.path).GetAttribute("rest:tx").Set(4.0)
    builder.add_fk_chain("Fk", [ctrl.path, child_path], [joint.path, joint_child.path])
    for sample, value in ((1, 1.0), (2, 5.0)):
        stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:tx").Set(value, sample)
        stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:rz").Set(10.0 * value, sample)
    chain = builder.new_mover_chain("Geometry")
    chain.add_matrix_mover("Points", ctrl.path, target=str(nested.GetPath()) + ".points")
    chain.add_parent_constraint("Parent", str(group.GetPath()), [ctrl.path])
    chain.add_parent_constraint("Child", str(nested.GetPath()), [child_path])
    rig = rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    snapshots = {time: rig.evaluate(time) for time in (1, 2)}
    assert all(pose.valid for pose in snapshots.values())
    original = stage.GetRootLayer().ExportToString()

    with tempfile.TemporaryDirectory(prefix="rigexec-bake-test-") as directory:
        path = os.path.join(directory, "baked.usda")
        baked = rigexec.export_baked(stage, ["/Asset/Rig"], [2, 1, 2], path)
        assert stage.GetRootLayer().ExportToString() == original, "bake authored the source stage"
        assert baked and baked.GetStartTimeCode() == 1 and baked.GetEndTimeCode() == 2
        assert baked.GetRootLayer().subLayerPaths == []
        text = baked.GetRootLayer().ExportToString()
        assert "RigExec" not in text and "rigExec:" not in text, text
        assert baked.GetPrimAtPath(child_path).GetTypeName() == "Xform"
        assert baked.GetPrimAtPath(joint_child.path).GetTypeName() == "Xform"
        assert baked.GetPrimAtPath(nested.GetPath()).GetTypeName() == "Points"
        assert UsdGeom.PrimvarsAPI(baked.GetPrimAtPath(nested.GetPath())).GetPrimvar(
            "displayColor").Get() == primvar.Get()
        for time, pose in snapshots.items():
            assert pose.provider_paths(), "fixture must exercise ordinary transform providers"
            asset_world = UsdGeom.XformCache(time).GetLocalToWorldTransform(asset.GetPrim())
            cache = UsdGeom.XformCache(time)
            frames = {p: pose.control_frame(p).to_matrix4() for p in pose.control_paths()}
            frames.update({p: pose.joint_frame(p).to_matrix4() for p in pose.joint_paths()})
            frames.update({p: pose.provider_xform(p) for p in pose.provider_paths()})
            for target, values in frames.items():
                expected = Gf.Matrix4d(*values) * asset_world
                actual = cache.GetLocalToWorldTransform(baked.GetPrimAtPath(target))
                assert close(actual, expected), (target, time, actual, expected)
            for target, values in pose.moved_properties().items():
                actual = baked.GetAttributeAtPath(target).Get(time)
                if target.endswith(".points") or target.endswith(".extent"):
                    assert len(actual) == len(values)
                    assert all((Gf.Vec3d(a) - Gf.Vec3d(b)).GetLength() < 1e-5
                               for a, b in zip(actual, values)), (target, actual, values)

        # Protect source files even when source and destination spelling differs.
        source_path = os.path.join(directory, "source.usda")
        stage.GetRootLayer().Export(source_path)
        file_stage = Usd.Stage.Open(source_path)
        try:
            rigexec.export_baked(file_stage, ["/Asset/Rig"], [1], source_path)
        except ValueError as error:
            assert "source layer" in str(error)
        else:
            raise AssertionError("bake overwrote its source")
        try:
            rigexec.export_baked(stage, [], [1], path)
        except ValueError:
            pass
        else:
            raise AssertionError("bake accepted missing rigs")
        assert stage.GetRootLayer().ExportToString() == original
        # Invalid evaluations do not replace an existing published artifact.
        with open(path, "rb") as stream:
            published = stream.read()
        bad_stage = Usd.Stage.Open(stage.Flatten())
        second = rigexec.Builder.create(bad_stage, "/Asset/OtherRig")
        source = second.add_control("Source")
        second.new_mover_chain("Conflict").add_matrix_mover(
            "Duplicate", source.path, target=str(nested.GetPath()) + ".points")
        before_failure = bad_stage.GetRootLayer().ExportToString()
        try:
            rigexec.export_baked(bad_stage, ["/Asset/Rig", "/Asset/OtherRig"], [1], path)
        except ValueError as error:
            assert "multiple rigs write" in str(error), error
        else:
            raise AssertionError("ambiguous rig output ownership was accepted")
        with open(path, "rb") as stream:
            assert stream.read() == published
        assert bad_stage.GetRootLayer().ExportToString() == before_failure
        default_path = os.path.join(directory, "default.usdc")
        default_stage = rigexec.export_baked(stage, ["/Asset/Rig"], [None], default_path)
        default_points = default_stage.GetAttributeAtPath(str(nested.GetPath()) + ".points")
        assert default_points.GetNumTimeSamples() == 0 and default_points.Get() is not None
        # Re-exporting while a consumer retains the previous stage must return
        # the new generation, even though USD caches the destination layer.
        stage.GetPrimAtPath(ctrl.path).GetAttribute("avars:tx").Set(9.0, 1)
        expected = rig.evaluate(1).moved_property(str(nested.GetPath()) + ".points")
        republished = rigexec.export_baked(stage, ["/Asset/Rig"], [1, 2], path)
        actual = republished.GetAttributeAtPath(str(nested.GetPath()) + ".points").Get(1)
        assert all((Gf.Vec3d(a) - Gf.Vec3d(b)).GetLength() < 1e-5
                   for a, b in zip(actual, expected)), (actual, expected)
    print("RIGEXEC_BAKE_OK")


if __name__ == "__main__":
    main()
