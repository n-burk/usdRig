#!/usr/bin/env python
"""Author the biped's materials as UsdPreviewSurface, from the FBX.

Reads `material_manifest.json` (produced by `extract_maya_materials.py`
under Blender) and binds the meshes. Two things about this asset shape the
code:

  * **`body_geo` carries five materials** -- `pants_shdr`, `skin_shdr`,
    `hair_shdr`, `lip_shdr` and `lambert2` -- across its 26,274 faces
    (23,470 skin, 1,356 pants, 1,120 lambert2, 284 lip, 44 hair). A single
    material binding cannot express that, so each slot becomes a
    `UsdGeomSubset` in the `materialBind` family and is bound separately.
  * **There are no textures.** The FBX carries 12 materials and zero
    images; the `NORMAL_MAP` nodes Blender's importer builds are empty
    stubs. So these are flat shading values, which is what the source has --
    the look comes from the studio's own shaders downstream (see the
    `sbclaude-usd` skills), not from this file.

One faithfulness caveat worth knowing rather than silently fixing: several
materials arrive with `Metallic = 0.5` (skin, lip, hair), which is a
Maya-phong-to-Principled conversion artifact rather than an authored
intent -- flesh is not half metal. The values are carried across verbatim
anyway, because this converter's job is to transport what the source says,
not to art-direct it. Override downstream if it reads wrong.
"""
import json
import os

from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

MANIFEST = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "material_manifest.json")

# Blender Principled socket -> (UsdPreviewSurface input, type, converter).
_SCALAR = Sdf.ValueTypeNames.Float
_COLOR = Sdf.ValueTypeNames.Color3f


def _rgb(v):
    return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))


_MAP = [
    ("Base Color", "diffuseColor", _COLOR, _rgb),
    ("Roughness", "roughness", _SCALAR, float),
    ("Metallic", "metallic", _SCALAR, float),
    ("IOR", "ior", _SCALAR, float),
    ("Alpha", "opacity", _SCALAR, float),
    ("Emission Color", "emissiveColor", _COLOR, _rgb),
]


def load_manifest(path=MANIFEST):
    if not os.path.exists(path):
        raise SystemExit(
            "%s not found -- run extract_maya_materials.py under Blender "
            "first" % path)
    return json.load(open(path))


def define_materials(stage, manifest, scope="/Biped/Materials"):
    """One UsdPreviewSurface material per FBX material."""
    UsdGeom.Scope.Define(stage, scope)
    out = {}
    for name, spec in sorted(manifest["materials"].items()):
        mat = UsdShade.Material.Define(stage, "%s/%s" % (scope, name))
        shader = UsdShade.Shader.Define(
            stage, "%s/%s/Surface" % (scope, name))
        shader.CreateIdAttr("UsdPreviewSurface")
        inputs = spec.get("inputs") or {}
        for src, dst, vtype, conv in _MAP:
            value = inputs.get(src)
            # Linked sockets arrive as "TEX:..." / "LINKED:..." strings;
            # there are no images in this asset, so skip rather than invent.
            if value is None or isinstance(value, str):
                continue
            try:
                shader.CreateInput(dst, vtype).Set(conv(value))
            except Exception:
                pass
        mat.CreateSurfaceOutput().ConnectToSource(
            shader.ConnectableAPI(), "surface")
        out[name] = mat
    print("  defined %d materials under %s" % (len(out), scope))
    return out


def bind_mesh(stage, mesh_path, mesh_name, manifest, materials):
    """Bind a mesh, using GeomSubsets when it has more than one material."""
    mesh_prim = stage.GetPrimAtPath(mesh_path)
    if not mesh_prim or not mesh_prim.IsValid():
        return 0
    info = (manifest.get("meshes") or {}).get(mesh_name)
    if not info:
        print("  %s: no material info in the manifest" % mesh_name)
        return 0

    slots = info.get("materials") or []
    faces_by_slot = info.get("facesBySlot") or {}
    named = [s for s in slots if s and s in materials]
    if not named:
        print("  %s: no usable materials" % mesh_name)
        return 0

    mesh = UsdGeom.Mesh(mesh_prim)
    if len(named) == 1:
        UsdShade.MaterialBindingAPI.Apply(mesh_prim).Bind(materials[named[0]])
        print("  %-16s bound %s" % (mesh_name, named[0]))
        return 1

    # Several materials: a subset per slot, in the materialBind family.
    binding_api = UsdShade.MaterialBindingAPI.Apply(mesh_prim)
    made = 0
    for slot_str, faces in sorted(faces_by_slot.items(),
                                  key=lambda kv: int(kv[0])):
        slot = int(slot_str)
        if slot >= len(slots) or not slots[slot]:
            continue
        mat_name = slots[slot]
        if mat_name not in materials:
            continue
        subset = UsdGeom.Subset.CreateGeomSubset(
            mesh, "%s_%s" % (mesh_name, mat_name), UsdGeom.Tokens.face,
            Vt.IntArray(faces), UsdShade.Tokens.materialBind)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(
            materials[mat_name])
        made += 1
        print("  %-16s subset %-14s %6d faces" % (mesh_name, mat_name,
                                                  len(faces)))
    # A whole-mesh fallback binding so nothing renders unbound if a subset
    # is ever dropped.
    binding_api.Bind(materials[named[0]])
    return made


def apply_materials(stage, mesh_name="body_geo",
                    mesh_path="/Biped/Geom/body_geo", manifest_path=MANIFEST):
    manifest = load_manifest(manifest_path)
    if not stage.GetPrimAtPath(mesh_path).IsValid():
        print("  no mesh at %s -- nothing to bind" % mesh_path)
        return 0
    materials = define_materials(stage, manifest)
    return bind_mesh(stage, mesh_path, mesh_name, manifest, materials)


if __name__ == "__main__":
    m = load_manifest()
    print("materials: %d, images: %d"
          % (len(m["materials"]), len(m["images"])))
    for name, info in sorted((m.get("meshes") or {}).items()):
        print("  %-24s %s" % (name, info.get("materials")))
