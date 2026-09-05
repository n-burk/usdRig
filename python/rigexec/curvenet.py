"""Native curvenet weight authoring helpers."""

import math
import numbers


def create_curvenet_weight(stage, path, curvenet, mesh, weights, auto_smooth=()):
    """Create/update a RigExecCurvenetWeight with explicit native dependencies.

    ``curvenet`` and ``mesh`` accept paths, USD prim/schema objects or RigExec
    handles. Weights contain one finite scalar per curvenet control-pool point;
    auto_smooth lists unique control indices whose values are interpolated.
    All arguments are checked before authoring. Returns a strict SchemaPrim.
    Basis and sampling attributes remain connected to the source curvenet so
    subsequent edits update the same native weight computation.
    """
    from pxr import Sdf, Usd, UsdGeom
    from . import schema, load_schema_plugin

    if not stage:
        raise ValueError("stage must be valid")
    load_schema_plugin()

    def source(value, expected):
        if hasattr(value, "GetPrim"):
            prim = value.GetPrim()
        elif isinstance(value, Usd.Prim):
            prim = value
        else:
            prim = stage.GetPrimAtPath(str(value.path if hasattr(value, "path") else value))
        if not prim or prim.GetStage() != stage:
            raise ValueError("%s source must be a valid prim on the destination stage" % expected)
        valid = prim.GetTypeName() == expected if expected == "RigExecCurvenet" else prim.IsA(UsdGeom.Mesh)
        if not valid:
            raise ValueError("expected %s source, got %s" % (expected, prim.GetTypeName()))
        return prim

    net = source(curvenet, "RigExecCurvenet")
    surface = source(mesh, "Mesh")
    target = Sdf.Path(str(path))
    if not target.IsAbsolutePath() or not target.IsPrimPath():
        raise ValueError("weight path must be an absolute prim path")
    existing = stage.GetPrimAtPath(target)
    if existing and existing.GetTypeName() != "RigExecCurvenetWeight":
        raise ValueError("weight path already has a different prim type")
    values = tuple(float(value) for value in weights)
    pool = net.GetAttribute("points").Get(Usd.TimeCode.EarliestTime())
    if pool is None or not pool or len(values) != len(pool) or not all(math.isfinite(v) for v in values):
        raise ValueError("weights must contain one finite value per curvenet control point")
    smooth = tuple(auto_smooth)
    if (any(isinstance(index, bool) or not isinstance(index, numbers.Integral)
            or not 0 <= index < len(pool) for index in smooth)
            or len(set(smooth)) != len(smooth)):
        raise ValueError("auto_smooth must contain unique valid control-pool indices")
    required_net = ("points", "rigExec:splineIndices", "rigExec:basis", "rigExec:samplesPerSpline")
    required_mesh = ("points", "faceVertexCounts", "faceVertexIndices")
    if (any(not net.GetAttribute(name) for name in required_net)
            or any(not surface.GetAttribute(name) for name in required_mesh)):
        raise ValueError("curvenet and mesh must expose their native layout attributes")

    result = schema.CurvenetWeight.define(stage, str(target))
    result.set_relationship("rigExec:weightTarget", [surface.GetPath().AppendProperty("points")])
    for name, prim, attr in (
            ("rigExec:curvenetPoints", net, "points"),
            ("rigExec:curvenetSplineIndices", net, "rigExec:splineIndices"),
            ("rigExec:meshFaceCounts", surface, "faceVertexCounts"),
            ("rigExec:meshFaceIndices", surface, "faceVertexIndices")):
        result.set_relationship(name, [prim.GetPath().AppendProperty(attr)])
    result.set_attribute("inputs:weights", list(values))
    result.set_attribute("rigExec:autoSmooth", [int(index) for index in smooth])
    result.set_attribute("rigExec:rangePolicy", "clamp")
    prim = stage.GetPrimAtPath(target)
    for name in ("rigExec:basis", "rigExec:samplesPerSpline"):
        prim.GetAttribute(name).SetConnections([net.GetPath().AppendProperty(name)])
    return result
