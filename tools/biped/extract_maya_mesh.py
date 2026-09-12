"""Blender headless: pull body_geo topology out of the biped FBX.

Writes a JSON sidecar with exactly what USD authoring needs -- point
positions in source (centimetre) space plus the polygon topology -- and no
Blender-side transform applied, so the vertex order and coordinates can be
checked against the rest points in the skinCluster XML.

Run:
  blender --background --factory-startup --python extract_mesh.py
"""
import json
import os
import sys

import bpy

FBX = ("C:/Users/walte/Documents/dev/squarebit/templates/templates/maya/"
       "biped/pub/default/model/biped_default_model.fbx")
OUT_DIR = os.path.dirname(os.path.abspath(__file__))

# Empty the factory startup scene.
bpy.ops.wm.read_factory_settings(use_empty=True)

# global_scale=1 and no axis conversion: keep FBX/Maya coordinates verbatim so
# the points can be compared against the XML rest positions directly.
bpy.ops.import_scene.fbx(
    filepath=FBX,
    global_scale=1.0,
    use_manual_orientation=True,
    axis_forward="Y",
    axis_up="Z",
    use_custom_normals=False,
    use_image_search=False,
)

meshes = [o for o in bpy.data.objects if o.type == "MESH"]
print("=== imported meshes: %d ===" % len(meshes))
for o in sorted(meshes, key=lambda m: m.name):
    print("  %-32s verts=%-7d polys=%-7d" % (
        o.name, len(o.data.vertices), len(o.data.polygons)))

manifest = {}
for o in meshes:
    me = o.data
    # Vertices in object-local space; the importer left the object transform
    # on the object, so bake it in only if it is not identity.
    mat = o.matrix_world
    is_identity = all(
        abs(mat[r][c] - (1.0 if r == c else 0.0)) < 1e-9
        for r in range(4) for c in range(4)
    )
    pts = []
    for v in me.vertices:
        co = v.co if is_identity else mat @ v.co
        pts.extend([co.x, co.y, co.z])
    counts = [len(p.vertices) for p in me.polygons]
    indices = []
    for p in me.polygons:
        indices.extend(list(p.vertices))
    manifest[o.name] = {
        "pointCount": len(me.vertices),
        "polyCount": len(me.polygons),
        "objectTransformIdentity": is_identity,
        "points": pts,
        "faceVertexCounts": counts,
        "faceVertexIndices": indices,
    }
    print("  -> captured %s (identity_xform=%s)" % (o.name, is_identity))

out = os.path.join(OUT_DIR, "mesh_manifest.json")
with open(out, "w") as f:
    json.dump(manifest, f)
print("=== wrote %s (%.1f MB) ===" % (out, os.path.getsize(out) / 1e6))
