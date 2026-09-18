"""
Dump an Unreal Engine project's rigs to text, so they can be read without the
editor.

Control Rig, Modular Rig, Deformer Graph, IK Rig and Anim Blueprint assets are
binary .uasset packages. The only reliable reader is the editor itself, so
this script runs INSIDE a headless editor and writes one directory per asset:

    <out>/<package path with / -> __>/
        summary.json            class, module settings, member variables,
                                static and runtime hierarchy counts, the
                                RigVM units and library functions it uses
        hierarchy.txt           the authored hierarchy, one element per line
        runtime_hierarchy.txt   the hierarchy AFTER the construction event,
                                from a live instance -- modular rigs spawn
                                almost everything procedurally, so this is
                                where their controls actually appear
        hierarchy_export.txt    RigHierarchyController.export_to_text
        modular_rig_model.txt   modules, parents, connections, bindings
        graphs.txt              every RigVM graph: nodes (unit struct,
                                dispatch notation, function reference) with
                                their unlinked input defaults, then links
        regen.py                the editor's own generate_python_commands()
        asset.t3d               a T3D text export (small assets only)
        kernels.hlsl            deformer graphs: the compute kernel sources
        bones.txt / sequence.txt  skeletal meshes / level sequences

plus asset_index.json, units_used.json and functions_used.json at the top.
docs/plans/ue-zebrasample-rig-gaps.md was written from this output.

Run it with the editor's commandlet (the Python plugin need not be enabled in
the project; -EnablePlugins turns it on for this run only):

    set UE_DUMP_OUT=C:/tmp/ue_dump
    UnrealEditor-Cmd.exe <project>.uproject -run=pythonscript
        -script=<repo>/tools/ueDumpRigs.py
        -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities
        -unattended -nop4 -nosplash -NoSound

Environment:
    UE_DUMP_OUT     output directory (default: ue_dump beside the project)
    UE_DUMP_ROOTS   comma-separated content roots (default: /Game plus every
                    project plugin mount that holds content)
    UE_DUMP_ONLY    substring filter on package paths, for a quick re-run

Most per-property reads are wrapped: the editor Python API differs between a
ControlRigBlueprint and the 5.8 ControlRigRuntimeAsset/ControlRigEditorAsset
split, and a missing accessor on one asset must not lose the rest of the dump.
Failed reads are recorded in _log.txt as ERR lines.
"""
import collections
import glob
import json
import os
import re
import traceback

import unreal

LOG = []


def _log(*parts):
    LOG.append(" ".join(str(p) for p in parts))


def _enum_name(value):
    return getattr(value, "name", str(value))


def _try(label, fn, default=None):
    try:
        return fn()
    except Exception as exc:
        _log("ERR", label, repr(exc)[:300])
        return default


def _text(value, limit=None):
    try:
        text = value.export_text()
    except Exception:
        text = str(value)
    if limit and len(text) > limit:
        text = text[:limit] + "...<trunc %d>" % len(text)
    return text


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def _dir_name(package):
    return package.strip("/").replace("/", "__")


def _content_roots():
    # Content mounts map a package root to a directory on disk; the sizes are
    # only used to skip T3D exports that would run to hundreds of megabytes.
    roots = {"/Game/": unreal.Paths.project_content_dir()}
    plugins = unreal.Paths.project_plugins_dir()
    for uplugin in glob.glob(os.path.join(plugins, "*", "*.uplugin")):
        content = os.path.join(os.path.dirname(uplugin), "Content")
        if os.path.isdir(content):
            name = os.path.splitext(os.path.basename(uplugin))[0]
            roots["/%s/" % name] = content + "/"
    return roots


ROOTS = _content_roots()
OUT = os.environ.get("UE_DUMP_OUT") or os.path.join(unreal.Paths.project_dir(), "ue_dump")
OUT = os.path.abspath(OUT)


def _package_size(package):
    for mount, directory in ROOTS.items():
        if package.startswith(mount):
            path = os.path.join(directory, package[len(mount):] + ".uasset")
            try:
                return os.path.getsize(path)
            except OSError:
                return None
    return None


def _small(package, limit):
    size = _package_size(package)
    return size is None or size < limit


def _export_t3d(obj, path):
    task = unreal.AssetExportTask()
    task.object = obj
    task.filename = path
    task.automated = True
    task.prompt = False
    task.replace_identical = True
    task.exporter = unreal.ObjectExporterT3D()
    if not unreal.Exporter.run_asset_export_task(task):
        _log("T3D FAIL", path)


def _format_transform(xf):
    try:
        t, r, s = xf.translation, xf.rotation.rotator(), xf.scale3d
        return "T(%.4g,%.4g,%.4g) R(%.4g,%.4g,%.4g) S(%.4g,%.4g,%.4g)" % (
            t.x, t.y, t.z, r.roll, r.pitch, r.yaw, s.x, s.y, s.z)
    except Exception:
        return "?"


UNIT_USE = collections.defaultdict(collections.Counter)
FUNC_USE = collections.defaultdict(collections.Counter)

_INPUT_DIRECTIONS = ("INPUT", "IO", "VISIBLE")


def _dump_graph(rig, graph, out):
    nodes = _try("nodes", graph.get_nodes, []) or []
    out.append("")
    out.append("### GRAPH %s | name=%s | nodes=%d | events=%s" % (
        graph.get_path_name(), graph.get_name(), len(nodes),
        _try("events", lambda: [str(e) for e in graph.get_event_names()], [])))
    for label, getter in (("LOCALVAR", graph.get_local_variables),
                          ("INARG", graph.get_input_arguments),
                          ("OUTARG", graph.get_output_arguments)):
        for var in _try(label, getter, []) or []:
            out.append("  %s %s" % (label, _text(var, 300)))

    for node in nodes:
        cls = type(node).__name__
        if cls == "RigVMUnitNode":
            struct = _try("struct", node.get_script_struct)
            ident = struct.get_name() if struct else "?"
            method = _try("method", lambda: str(node.get_method_name()), "")
            if method and method != "Execute":
                ident += "::" + method
            UNIT_USE[rig][ident] += 1
        elif cls in ("RigVMDispatchNode", "RigVMTemplateNode"):
            ident = _try("notation", lambda: str(node.get_notation()), "?")
            UNIT_USE[rig][ident.split("(")[0]] += 1
        elif cls == "RigVMFunctionReferenceNode":
            header = _try("header", node.get_referenced_function_header)
            name = _try("hname", lambda: str(header.get_editor_property("name")), "?") if header else "?"
            library = _try("hlib", lambda: _text(header.get_editor_property("library_pointer"), 300), "") if header else ""
            ident = "FUNC %s @ %s" % (name, library)
            FUNC_USE[rig][name] += 1
        elif cls in ("RigVMAggregateNode", "RigVMCollapseNode"):
            ident = "COLLAPSE " + _try("contained", lambda: node.get_contained_graph().get_path_name(), "?")
        elif cls == "RigVMVariableNode":
            ident = "VAR %s %s getter=%s" % (
                _try("vname", lambda: str(node.get_variable_name()), "?"),
                _try("vtype", node.get_cpp_type, ""),
                _try("getter", node.is_getter, ""))
        elif cls == "RigVMCommentNode":
            ident = "COMMENT " + _try("comment", node.get_comment_text, "").replace("\n", " | ")[:400]
        elif cls == "RigVMRerouteNode":
            ident = "REROUTE"
        else:
            ident = _try("notation2", lambda: str(node.get_notation()), "")

        # Only the inputs nothing is linked to: those are the authored values.
        defaults = []
        if cls != "RigVMCommentNode":
            for pin in _try("pins", node.get_pins, []) or []:
                try:
                    direction = _enum_name(pin.get_direction()).upper()
                    if not any(d in direction for d in _INPUT_DIRECTIONS):
                        continue
                    if pin.is_execute_context() or pin.get_links() or pin.get_source_links():
                        continue
                    value = pin.get_default_value()
                    if value in (None, "", "()"):
                        continue
                    if len(value) > 240:
                        value = value[:240] + "...<%d>" % len(value)
                    defaults.append("%s=%s" % (pin.get_name(), value))
                except Exception:
                    pass
        title = _try("title", lambda: str(node.get_node_title()), "")
        out.append("N %s [%s] %s | title=%s%s" % (
            node.get_name(), cls, ident, title,
            (" | " + "; ".join(defaults)) if defaults else ""))

    for link in _try("links", graph.get_links, []) or []:
        out.append("L " + _try("link", lambda: str(link.get_pin_path_representation()), "?"))


_CONTROL_SETTINGS = (
    "control_type", "animation_type", "primary_axis", "shape_visible", "shape_name",
    "display_name", "limit_enabled", "driven_controls", "preferred_rotation_order",
    "filtered_channels", "group_with_parent_control", "restrict_space_switching",
    "customization", "shape_visibility", "draw_limits", "is_transient_control")


def _hierarchy_lines(hierarchy, keys):
    lines = []
    for key in keys:
        try:
            parents = _try("parents", lambda: ["%s:%s" % (_enum_name(p.type), p.name)
                                               for p in hierarchy.get_parents(key)], [])
            weights = ""
            if len(parents) > 1:
                weights = _try("weights", lambda: " weights=" + ",".join(
                    _text(w, 80) for w in hierarchy.get_parent_weight_array(key, True)), "")
            line = "%s %s parents=%s%s init_global=%s" % (
                _enum_name(key.type), key.name, parents, weights,
                _try("global", lambda: _format_transform(hierarchy.get_global_transform(key, True)), "?"))
            tags = _try("tags", lambda: [str(t) for t in hierarchy.get_tags(key)], [])
            if tags:
                line += " tags=%s" % tags
            metadata = _try("metadata", lambda: ["%s:%s" % (m, _enum_name(hierarchy.get_metadata_type(key, m)))
                                                 for m in hierarchy.get_metadata_names(key)], [])
            if metadata:
                line += " metadata=%s" % metadata
            if _enum_name(key.type).upper() == "CONTROL":
                settings = hierarchy.get_control_settings(key)
                parts = []
                for field in _CONTROL_SETTINGS:
                    value = _try("settings." + field, lambda f=field: settings.get_editor_property(f))
                    if value is None:
                        continue
                    if isinstance(value, unreal.EnumBase):
                        text = _enum_name(value)
                    elif isinstance(value, (str, bool, int, float)):
                        text = str(value)
                    else:
                        text = _text(value, 400)
                    parts.append("%s=%s" % (field, text))
                line += " || " + " ".join(parts)
            lines.append(line)
        except Exception as exc:
            lines.append("ERR %s %s" % (key, exc))
    return lines


def _dump_control_rig(package, asset, directory):
    summary = {"package": package, "class": type(asset).__name__}
    # 5.8 splits some rigs into a runtime asset and an editor-only asset; the
    # graphs and the hierarchy controller live on the editor half.
    rig_bp = asset
    if isinstance(asset, unreal.ControlRigRuntimeAsset):
        rig_bp = asset.get_editor_asset()
        summary["runtime_asset"] = True
        summary["influences"] = _try("influences", lambda: _text(asset.get_editor_property("influences"), 20000))
        model = _try("runtime model", lambda: asset.get_editor_property("modular_rig_model"))
        if model is not None:
            _write(directory + "/modular_rig_model.txt", _text(model))
    rig = package.split("/")[-1]
    summary["is_module"] = _try("is_module", rig_bp.is_control_rig_module) if hasattr(rig_bp, "is_control_rig_module") else None
    summary["preview_mesh"] = _try("preview_mesh", lambda: str(rig_bp.get_preview_mesh().get_path_name()))
    summary["rig_module_settings"] = _try("module_settings", lambda: _text(rig_bp.get_editor_property("rig_module_settings"), 20000))
    variables = _try("member_variables", rig_bp.get_member_variables) if hasattr(rig_bp, "get_member_variables") else None
    if variables is None:
        variables = _try("asset_variables", rig_bp.get_asset_variables, []) or []
    # In full: face rigs keep per-bone tuning tables in variable defaults.
    summary["variables"] = [_text(v) for v in variables]

    model = _try("model", lambda: rig_bp.get_editor_property("modular_rig_model"))
    if model is not None and "Modules=(" in _text(model):
        _write(directory + "/modular_rig_model.txt", _text(model))
        summary["modular"] = True

    hierarchy = rig_bp.get_hierarchy()
    keys = _try("keys", hierarchy.get_all_keys, []) or []
    summary["hierarchy_counts"] = dict(collections.Counter(_enum_name(k.type) for k in keys))
    lines = _hierarchy_lines(hierarchy, keys)
    components = _try("components", hierarchy.get_all_component_keys, []) or []
    for component in components:
        lines.append("COMPONENT %s type=%s content=%s" % (
            _text(component, 200),
            _try("ctype", lambda: str(hierarchy.get_component_type(component)), "?"),
            _try("ccontent", lambda: str(hierarchy.get_component_content(component))[:2000], "")))
    summary["components"] = len(components)
    _write(directory + "/hierarchy.txt", "\n".join(lines))
    exported = _try("export_to_text", lambda: rig_bp.get_hierarchy_controller().export_to_text(keys), "")
    if exported:
        _write(directory + "/hierarchy_export.txt", exported)

    # A live instance runs the construction event, which is where modular
    # rigs create their controls, nulls and rig bones.
    if hasattr(rig_bp, "create_control_rig"):
        try:
            instance = rig_bp.create_control_rig()
            if instance is not None:
                _try("request_init", instance.request_init)
                _try("request_construction", instance.request_construction)
                _try("construction", lambda: instance.execute("Construction"))
                runtime = instance.get_hierarchy()
                runtime_keys = runtime.get_all_keys()
                summary["runtime_hierarchy_counts"] = dict(
                    collections.Counter(_enum_name(k.type) for k in runtime_keys))
                summary["runtime_instance_class"] = type(instance).__name__
                _write(directory + "/runtime_hierarchy.txt",
                       "\n".join(_hierarchy_lines(runtime, runtime_keys)))
        except Exception as exc:
            _log("instance", package, repr(exc)[:300])

    out = []
    seen = set()
    summary["graphs"] = []
    for graph in _try("models", rig_bp.get_all_models, []) or []:
        path = graph.get_path_name()
        if path in seen:
            continue
        seen.add(path)
        summary["graphs"].append(path)
        _dump_graph(rig, graph, out)
    library = _try("library", rig_bp.get_local_function_library)
    if library:
        functions = _try("functions", library.get_functions, []) or []
        summary["local_functions"] = [f.get_name() for f in functions]
        for function in functions:
            contained = _try("function graph", function.get_contained_graph)
            if contained and contained.get_path_name() not in seen:
                seen.add(contained.get_path_name())
                _dump_graph(rig, contained, out)
    _write(directory + "/graphs.txt", "\n".join(out))

    commands = _try("generate_python_commands", rig_bp.generate_python_commands)
    if commands:
        _write(directory + "/regen.py", "\n".join(str(c) for c in commands))
    summary["units_used"] = dict(UNIT_USE[rig])
    summary["functions_used"] = dict(FUNC_USE[rig])
    _write(directory + "/summary.json", json.dumps(summary, indent=1, default=str))
    if _small(package, 1500000):
        _export_t3d(asset, directory + "/asset.t3d")


def _dump_skeletal_mesh(package, mesh, directory):
    subsystem = unreal.get_editor_subsystem(unreal.SkeletalMeshEditorSubsystem)
    skeleton = _try("skeleton", lambda: mesh.get_editor_property("skeleton"))
    summary = {
        "package": package,
        "skeleton": skeleton.get_path_name() if skeleton else None,
        "morph_targets": [str(m) for m in _try("morphs", mesh.get_all_morph_target_names, [])],
        "skin_weight_profiles": [str(m) for m in _try("profiles", mesh.get_all_skin_weight_profile_names, [])],
        "default_mesh_deformer": _try("deformer", lambda: str(mesh.get_default_mesh_deformer())),
        "target_mesh_deformers": _try("target deformers", lambda: str(mesh.get_target_mesh_deformers())),
        "default_animating_rig": _try("animating rig", lambda: str(mesh.get_editor_property("default_animating_rig"))),
        "post_process_anim_blueprint": _try("post process", lambda: str(mesh.get_editor_property("post_process_anim_blueprint"))),
        "physics_asset": _try("physics", lambda: str(mesh.get_editor_property("physics_asset"))),
        "materials": _try("materials", lambda: [str(m.get_editor_property("material_slot_name"))
                                                for m in mesh.get_editor_property("materials")], []),
        "sockets": _try("sockets", lambda: ["%s on %s" % (mesh.get_socket_by_index(i).get_editor_property("socket_name"),
                                                           mesh.get_socket_by_index(i).get_editor_property("bone_name"))
                                            for i in range(mesh.num_sockets())], []),
        "asset_user_data": _try("user data", lambda: [str(x) for x in mesh.get_editor_property("asset_user_data")], []),
        "lods": _try("lods", lambda: subsystem.get_lod_count(mesh)),
        "num_verts_lod0": _try("verts", lambda: subsystem.get_num_verts(mesh, 0)),
    }
    bones = []
    try:
        pose = unreal.AnimPoseExtensions.get_reference_pose(skeleton)
        for name in unreal.AnimPoseExtensions.get_bone_names(pose):
            parent = _try("bone parent", lambda: str(mesh.get_bone_parent(name)), "?")
            local = _try("bone local", lambda: _format_transform(
                unreal.AnimPoseExtensions.get_ref_bone_pose(pose, name, unreal.AnimPoseSpaces.LOCAL)), "")
            bones.append("%s parent=%s local=%s" % (name, parent, local))
    except Exception as exc:
        _log("bones", package, repr(exc)[:300])
    summary["bone_count"] = len(bones)
    _write(directory + "/bones.txt", "\n".join(bones))
    _write(directory + "/summary.json", json.dumps(summary, indent=1, default=str))


def _dump_anim_sequence(package, anim, directory):
    lib = unreal.AnimationLibrary
    summary = {
        "package": package,
        "length": _try("length", lambda: lib.get_sequence_length(anim)),
        "frames": _try("frames", lambda: lib.get_num_frames(anim)),
        "float_curves": _try("float curves", lambda: [str(c) for c in lib.get_animation_curve_names(
            anim, unreal.RawCurveTrackTypes.RCT_FLOAT)], []),
        "transform_curves": _try("transform curves", lambda: [str(c) for c in lib.get_animation_curve_names(
            anim, unreal.RawCurveTrackTypes.RCT_TRANSFORM)], []),
        "bone_tracks": _try("bone tracks", lambda: [str(c) for c in lib.get_animation_track_names(anim)], []),
        "skeleton": _try("skeleton", lambda: anim.get_editor_property("skeleton").get_path_name()),
    }
    _write(directory + "/summary.json", json.dumps(summary, indent=1, default=str))


def _dump_level_sequence(package, sequence, directory):
    lines = []
    for binding in _try("bindings", sequence.get_bindings, []) or []:
        lines.append("BINDING %s" % _try("binding name", lambda: str(binding.get_display_name()), "?"))
        for track in _try("tracks", binding.get_tracks, []) or []:
            sections = _try("sections", track.get_sections, []) or []
            lines.append("  TRACK %s name=%s sections=%d" % (
                type(track).__name__, _try("track name", lambda: str(track.get_display_name()), ""), len(sections)))
            for section in sections[:3]:
                channels = _try("channels", lambda: [str(c.get_editor_property("channel_name"))
                                                     for c in section.get_all_channels()], [])
                if channels:
                    lines.append("    SECTION %s channels(%d)=%s" % (type(section).__name__, len(channels), channels[:400]))
    for track in _try("master tracks", sequence.get_tracks, []) or []:
        lines.append("MASTER TRACK %s %s" % (type(track).__name__,
                                            _try("master name", lambda: str(track.get_display_name()), "")))
    _write(directory + "/sequence.txt", "\n".join(lines))


# T3D escapes quotes, backslashes and newlines inside property strings; the
# kernel sources are the long strings that contain HLSL.
_T3D_STRING = re.compile(r'(\w+)="((?:[^"\\]|\\.)*)"')
_T3D_ESCAPES = {"n": "\n", "t": "\t", '"': '"', "\\": "\\", "r": ""}


def _unescape_t3d(text):
    out, i = [], 0
    while i < len(text):
        if text[i] == "\\" and i + 1 < len(text):
            out.append(_T3D_ESCAPES.get(text[i + 1], "\\" + text[i + 1]))
            i += 2
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def _extract_kernels(t3d_path, out_path):
    text = open(t3d_path, encoding="utf-8", errors="replace").read()
    seen, parts = set(), []
    for match in _T3D_STRING.finditer(text):
        name, value = match.group(1), match.group(2)
        if len(value) < 150 or "\\n" not in value:
            continue
        if not any(token in value for token in ("return", "void", "float", "#line", "KERNEL")):
            continue
        source = _unescape_t3d(value)
        if source in seen:
            continue
        seen.add(source)
        parts.append("// ===== %s (%d chars)\n%s\n" % (name, len(source), source))
    _write(out_path, "\n".join(parts))


_SKIP_CLASSES = {
    "Texture2D", "TextureCube", "TextureRenderTarget2D", "Material", "MaterialInstanceConstant",
    "MaterialFunction", "MaterialParameterCollection", "SubstrateMaterialFunction", "StaticMesh",
    "World", "MapBuildDataRegistry", "SoundWave", "Font", "PhysicalMaterial", "NiagaraSystem",
    "CurveFloat"}
_SKIP_PATHS = ("/Lighting/", "/Materials/", "/Textures/", "/Splash", "/Icons/")


def main():
    roots = [r.strip() for r in os.environ.get("UE_DUMP_ROOTS", "").split(",") if r.strip()]
    roots = roots or [mount.rstrip("/") for mount in ROOTS]
    only = os.environ.get("UE_DUMP_ONLY")
    registry = unreal.AssetRegistryHelpers.get_asset_registry()
    # Plugin content is not scanned yet when a commandlet starts.
    registry.scan_paths_synchronous(roots, True)
    index = []
    for root in roots:
        for data in registry.get_assets_by_path(root, recursive=True):
            package = str(data.package_name)
            index.append({"package": package,
                          "class": str(data.asset_class_path.asset_name),
                          "size": _package_size(package)})
    _write(OUT + "/asset_index.json", json.dumps(index, indent=1))

    for item in index:
        package, cls = item["package"], item["class"]
        if cls in _SKIP_CLASSES or any(p in package for p in _SKIP_PATHS):
            continue
        if only and only not in package:
            continue
        directory = OUT + "/" + _dir_name(package)
        try:
            asset = unreal.load_asset(package)
            if asset is None:
                _log("NULL", package)
                continue
            _log("DUMP", package, type(asset).__name__)
            if isinstance(asset, (unreal.ControlRigBlueprint, unreal.ControlRigRuntimeAsset)):
                _dump_control_rig(package, asset, directory)
            elif isinstance(asset, unreal.SkeletalMesh):
                _dump_skeletal_mesh(package, asset, directory)
                if _small(package, 1000000):
                    _export_t3d(asset, directory + "/asset.t3d")
            elif isinstance(asset, unreal.AnimSequence):
                _dump_anim_sequence(package, asset, directory)
            elif isinstance(asset, unreal.LevelSequence):
                _dump_level_sequence(package, asset, directory)
            elif isinstance(asset, (unreal.Texture, unreal.MaterialInterface, unreal.StaticMesh, unreal.World)):
                continue
            elif _small(package, 3000000):
                _export_t3d(asset, directory + "/asset.t3d")
                if cls == "OptimusDeformer" and os.path.exists(directory + "/asset.t3d"):
                    _extract_kernels(directory + "/asset.t3d", directory + "/kernels.hlsl")
            else:
                _log("SKIP large", package, type(asset).__name__)
        except Exception:
            _log("FAIL", package, traceback.format_exc()[-1500:])
        _write(OUT + "/_log.txt", "\n".join(LOG))

    for name, usage in (("units_used", UNIT_USE), ("functions_used", FUNC_USE)):
        total = collections.Counter()
        for counter in usage.values():
            total.update(counter)
        _write(OUT + "/%s.json" % name, json.dumps({
            "total": dict(total.most_common()),
            "per_rig": {rig: dict(c.most_common()) for rig, c in usage.items()}}, indent=1))
    _log("DONE")
    _write(OUT + "/_log.txt", "\n".join(LOG))


main()
