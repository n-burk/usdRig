"""Blender headless: report the biped FBX's materials and textures.

Companion to extract_maya_mesh.py. Writes a JSON sidecar describing each
mesh's material assignments plus whatever shading parameters and texture
paths the FBX carries, so build_biped_skel.py can author UsdPreviewSurface
materials instead of leaving the meshes unbound.

Reports rather than assumes: an FBX exported from a Maya rig scene often
carries only lambert/phong stubs with no maps, and it is worth knowing that
before writing a converter for shading that is not there.

Run:
  blender --background --factory-startup --python extract_maya_materials.py
"""
import json
import os

import bpy

FBX = ("C:/Users/walte/Documents/dev/squarebit/templates/templates/maya/"
       "biped/pub/default/model/biped_default_model.fbx")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=FBX, global_scale=1.0,
                         use_image_search=True)

report = {"materials": {}, "meshes": {}, "images": []}

for img in bpy.data.images:
    report["images"].append({
        "name": img.name,
        "filepath": img.filepath,
        "exists": bool(img.filepath) and os.path.exists(
            bpy.path.abspath(img.filepath)),
        "size": list(img.size),
    })

for mat in bpy.data.materials:
    entry = {"use_nodes": mat.use_nodes, "nodes": [], "inputs": {}}
    # Blender's FBX importer builds a Principled BSDF; read its sockets so
    # the values can be mapped onto UsdPreviewSurface directly.
    if mat.use_nodes and mat.node_tree:
        for node in mat.node_tree.nodes:
            entry["nodes"].append({"type": node.type, "name": node.name})
            if node.type == "BSDF_PRINCIPLED":
                for sock in node.inputs:
                    if sock.is_linked:
                        src = sock.links[0].from_node
                        val = "LINKED:%s" % src.type
                        if src.type == "TEX_IMAGE" and src.image:
                            val = "TEX:%s" % src.image.name
                        entry["inputs"][sock.name] = val
                    else:
                        try:
                            v = sock.default_value
                            entry["inputs"][sock.name] = (
                                list(v) if hasattr(v, "__len__") else v)
                        except Exception:
                            pass
    else:
        entry["inputs"]["diffuse_color"] = list(mat.diffuse_color)
        entry["inputs"]["roughness"] = mat.roughness
        entry["inputs"]["metallic"] = mat.metallic
    report["materials"][mat.name] = entry

for obj in bpy.data.objects:
    if obj.type != "MESH":
        continue
    # Per-polygon slot index, in the same polygon order extract_maya_mesh.py
    # writes faceVertexCounts/Indices, so the two manifests index alike.
    # body_geo carries five materials (pants/skin/hair/lip/lambert2), so a
    # single binding will not do -- USD needs a GeomSubset per material.
    per_poly = [p.material_index for p in obj.data.polygons]
    faces_by_slot = {}
    for face, slot in enumerate(per_poly):
        faces_by_slot.setdefault(slot, []).append(face)
    report["meshes"][obj.name] = {
        "materials": [s.material.name if s.material else None
                      for s in obj.material_slots],
        "polygonMaterialIndices": sorted(set(per_poly)),
        "polyCount": len(per_poly),
        "facesBySlot": {str(k): v for k, v in sorted(faces_by_slot.items())},
    }

out = os.path.join(OUT_DIR, "material_manifest.json")
with open(out, "w") as f:
    json.dump(report, f, indent=1)

print("=== materials: %d ===" % len(report["materials"]))
for name, m in sorted(report["materials"].items()):
    linked = {k: v for k, v in m["inputs"].items()
              if isinstance(v, str) and v.startswith(("TEX:", "LINKED:"))}
    print("  %-28s nodes=%-2d textured=%s"
          % (name, len(m["nodes"]), linked or "none"))
print("=== images: %d ===" % len(report["images"]))
for i in report["images"]:
    print("  %-28s exists=%-5s size=%s  %s"
          % (i["name"], i["exists"], i["size"], i["filepath"]))
print("=== mesh assignments ===")
for name, m in sorted(report["meshes"].items()):
    print("  %-24s %s" % (name, m["materials"]))
print("=== wrote %s ===" % out)
