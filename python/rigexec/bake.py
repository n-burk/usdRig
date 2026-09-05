"""Export evaluated RigExec results as a standalone standard USD stage."""

import math
import os
import tempfile


def export_baked(stage, rig_paths, times, path):
    """Bake selected rigs and preserve the composed scene in a separate file.

    ``times`` contains nonnegative frame numbers and/or ``None`` for default
    time. All active RigExecRoot prims must be selected, so stripping runtime
    schemas cannot silently leave another rig unevaluated. The source stage is
    never authored. Each sample writes final moved attributes, joint/control
    frames and revised transform providers using ordinary USD values and local
    xform matrices. Geometry topology, materials, primvars and metadata survive
    the flattened copy. No RigExec plugin is required to read the result.

    Raises on invalid rigs, conflicting output ownership, invalid matrices or
    an attempt to overwrite a source layer. The destination is replaced only
    after every sample succeeds. Returns the reopened destination stage.
    """
    from pxr import Gf, Sdf, Usd, UsdGeom
    from . import Rig

    if not stage:
        raise ValueError("stage must be a valid USD stage")
    destination = os.path.realpath(os.fspath(path))
    for layer in stage.GetUsedLayers():
        if layer.realPath and os.path.normcase(os.path.realpath(layer.realPath)) == os.path.normcase(destination):
            raise ValueError("bake destination cannot overwrite a source layer")
    paths = tuple(dict.fromkeys(str(value) for value in rig_paths))
    roots = {str(prim.GetPath()) for prim in stage.Traverse()
             if prim.GetTypeName() == "RigExecRoot"}
    if not paths or set(paths) != roots:
        raise ValueError("rig_paths must name every active RigExecRoot in the stage")
    samples = []
    for value in times:
        if value is None or (isinstance(value, Usd.TimeCode) and value.IsDefault()):
            sample = None
        else:
            sample = value.GetValue() if isinstance(value, Usd.TimeCode) else float(value)
            if not math.isfinite(sample) or sample < 0:
                raise ValueError("bake times must be finite nonnegative frames or None")
        if sample not in samples:
            samples.append(sample)
    if not samples:
        raise ValueError("at least one bake time is required")
    samples.sort(key=lambda value: -math.inf if value is None else value)
    rigs = []
    for root in paths:
        rig = Rig(stage, root)
        rig.compile()
        rigs.append((root, rig))

    baked = Usd.Stage.Open(stage.Flatten())
    if not baked:
        raise RuntimeError("could not flatten source stage")
    # Preserve standard schema properties on geometry that merely carries a
    # RigExec API. Concrete runtime prims become ordinary hierarchy nodes.
    stripped_properties = set()
    for prim in list(baked.TraverseAll()):
        runtime_type = prim.GetTypeName().startswith("RigExec")
        runtime_apis = [name for name in prim.GetAppliedSchemas() if name.startswith("RigExec")]
        declared = set(prim.GetPrimDefinition().GetPropertyNames()) if runtime_type else set()
        if runtime_type:
            prim.SetTypeName("Xform" if prim.GetTypeName() in
                             ("RigExecControl", "RigExecJoint") else "Scope")
            standard = set(prim.GetPrimDefinition().GetPropertyNames())
            declared -= standard
        for name in runtime_apis:
            definition = Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(name)
            if definition:
                declared.update(definition.GetPropertyNames())
        stripped_properties.update(prim.GetPath().AppendProperty(name) for name in declared)
        if runtime_apis:
            prim.SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(
                [name for name in prim.GetAppliedSchemas() if not name.startswith("RigExec")]))
        for prop in list(prim.GetProperties()):
            name = prop.GetName()
            if name in declared or name.startswith("rigExec:") or (runtime_type and
                    name.split(":", 1)[0] in ("rest", "default", "posed", "parent", "avars", "inputs", "outputs", "out")):
                stripped_properties.add(prop.GetPath())
                prim.RemoveProperty(name)
    # Remove connections into execution-only properties, including consumers
    # outside the rig namespace. Their evaluated moved values are authored below.
    for prim in baked.TraverseAll():
        for attr in prim.GetAttributes():
            sources = attr.GetConnections()
            if any(source in stripped_properties for source in sources):
                attr.SetConnections([source for source in sources if source not in stripped_properties])
        for rel in prim.GetRelationships():
            targets = rel.GetTargets()
            if any(target in stripped_properties for target in targets):
                rel.SetTargets([target for target in targets if target not in stripped_properties])

    cleared = set()
    xform_ops = {}
    ownership = {}

    def claim(key, root):
        owner = ownership.setdefault(key, root)
        if owner != root:
            raise ValueError("multiple rigs write %s: %s and %s" % (key, owner, root))

    def matrix(value):
        result = Gf.Matrix4d(*value)
        if any(not math.isfinite(result[r][c]) for r in range(4) for c in range(4)):
            raise ValueError("cannot bake a non-finite transform")
        return result

    for sample in samples:
        time = Usd.TimeCode.Default() if sample is None else Usd.TimeCode(sample)
        source_cache = UsdGeom.XformCache(time)
        world_frames = {}
        for root, rig in rigs:
            pose = rig.evaluate(-1.0 if sample is None else sample)
            if not pose.valid:
                raise RuntimeError("rig evaluation failed at %s: %s" % (sample, "; ".join(pose.diagnostics)))
            asset = stage.GetPrimAtPath(root).GetParent()
            asset_world = (source_cache.GetLocalToWorldTransform(asset)
                           if not asset.IsPseudoRoot() else Gf.Matrix4d(1.0))
            for target, value in pose.moved_properties().items():
                claim(target, root)
                attr = baked.GetAttributeAtPath(target)
                if not attr:
                    # Scalar rig channels are execution inputs; their effects
                    # are already represented by final geometry and frames.
                    if Sdf.Path(target) in stripped_properties:
                        continue
                    raise ValueError("missing baked output attribute %s" % target)
                if target not in cleared:
                    attr.Clear()
                    attr.ClearConnections()
                    cleared.add(target)
                if attr.GetTypeName() == Sdf.ValueTypeNames.Matrix4d:
                    value = matrix(value)
                if not attr.Set(value, time):
                    raise RuntimeError("could not author baked output %s" % target)
            for target in pose.joint_paths():
                claim(target, root)
                world_frames[target] = matrix(pose.joint_frame(target).to_matrix4()) * asset_world
            for target in pose.control_paths():
                claim(target, root)
                world_frames[target] = matrix(pose.control_frame(target).to_matrix4()) * asset_world
            for target in pose.provider_paths():
                claim(target, root)
                world_frames[target] = matrix(pose.provider_xform(target)) * asset_world

        # Parents first: each child converts its absolute result against the
        # already baked parent, preserving nested provider overrides and scale.
        baked_cache = UsdGeom.XformCache(time)
        for target in sorted(world_frames, key=lambda value: (value.count("/"), value)):
            prim = baked.GetPrimAtPath(target)
            xf = UsdGeom.Xformable(prim)
            if not xf:
                raise ValueError("transform output is not xformable: %s" % target)
            baked_cache.Clear()
            parent = prim.GetParent()
            parent_world = (baked_cache.GetLocalToWorldTransform(parent)
                            if not parent.IsPseudoRoot() else Gf.Matrix4d(1.0))
            if abs(parent_world.GetDeterminant()) < 1e-12:
                raise ValueError("cannot convert baked frame through singular parent: %s" % target)
            local = world_frames[target] * parent_world.GetInverse()
            if target not in xform_ops:
                for attr in prim.GetAttributes():
                    if attr.GetName().startswith("xformOp:"):
                        prim.RemoveProperty(attr.GetName())
                op = xf.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, "baked")
                xf.SetXformOpOrder([op], False)
                xform_ops[target] = op
            xform_ops[target].Set(local, time)

    numeric_times = [sample for sample in samples if sample is not None]
    if numeric_times:
        baked.SetStartTimeCode(min(numeric_times))
        baked.SetEndTimeCode(max(numeric_times))
    directory = os.path.dirname(destination)
    if not os.path.isdir(directory):
        raise ValueError("destination directory does not exist")
    suffix = os.path.splitext(destination)[1]
    if suffix.lower() not in (".usd", ".usda", ".usdc"):
        raise ValueError("bake destination must be .usd, .usda or .usdc")
    handle, temporary = tempfile.mkstemp(prefix=".rigexec-bake-", suffix=suffix, dir=directory)
    os.close(handle)
    try:
        if not baked.GetRootLayer().Export(temporary):
            raise RuntimeError("could not export baked stage")
        os.replace(temporary, destination)
        # USD caches layers by identifier. An earlier returned stage may keep
        # the destination layer alive, so reopening alone can return old data.
        cached_layer = Sdf.Layer.Find(destination)
        if cached_layer:
            cached_layer.Reload(True)
    finally:
        if os.path.exists(temporary):
            os.remove(temporary)
    return Usd.Stage.Open(destination)
