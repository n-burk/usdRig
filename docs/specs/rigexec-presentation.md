# Embedded presentation data

A `.rigexec` asset can contain the rig program together with geometry,
materials, textures, subdivision data, and exposed controller information.
The consumer reads these records to build its display objects and sends
controller values to the binary runtime.

The presentation is its own FlatBuffer, defined by
`libs/rigExecBinary/presentation.fbs` (root `Presentation`, file identifier
`REXP`). A `.rigexec` file carries it in the `presentation` field of its
`File` root (`libs/rigExecBinary/rigexec.fbs`) as a nested buffer aligned to
16 bytes; an empty field means the file has none. The runtime in
`libs/rigExecRuntime` never interprets it: a display consumer reads it and
sets the runtime inputs its controls name.

usdRig does not author presentations. An exporter builds the REXP buffer, and
`rigExecBake --presentation <file.rexp>` (`RigExecBakeOpts::presentation` in
`libs/rigExecBake/bake.h`) embeds it unchanged. The bake refuses a buffer
that does not verify as REXP, or a control whose input the file does not
list. `RigExecFormatOpen` bounds and verifies the nested buffer whenever it
opens a file. After a schema edit, regenerate with `bin/gen_flatbuffers.sh`
or `bin/gen_flatbuffers.bat` and regenerate any exporter's builder code.
Unsupported data must be diagnosed during export or loading rather than
silently discarded.

## Contents

* `version` is 1.
* Each `PresentationControl` has a public `name`, the `input` it sets (the
  attribute path of one of the carrying file's inputs) and a `unit`: asset
  units, degrees or ratio. A control holds no default; the input's default
  in the carrying file, read at the bake time, is the only one.
* Each `PresentationMesh` names its source mesh `path`, the `points_path`
  whose evaluated points it reads, and their `source_point_count`. Render
  vertices come from CSR stencils over those points: `stencil_offsets` holds
  three groups per render vertex (position, then the u and v derivatives
  that give the normal). Indices are `stencil_indices16` while
  `source_point_count` is at most 65535, else `stencil_indices32`, with one
  `stencil_weights` float each. `vertex_indices` lists triangles by render
  vertex, with two `uvs` per entry.
* A mesh's `PresentationMaterial` names its `model` (`UsdPreviewSurface`),
  embeds the texture as raw PNG bytes, and carries diffuse and emission
  scales, roughness, metallic, specular and the S and T wrap modes.
* `source_json` is the exporter's description of its source, as JSON text.

These floats are display data, so the schema is exempt from `rigexec.fbs`'s
rule against float scalars in tables.

## Current exporter and consumer

The Godot add-on verifies the nested buffer again before reading it. It
accepts version 1, at least one mesh, controls on `double` or `float`
inputs, and the `UsdPreviewSurface` model with repeat S and clamp T. Any
part it cannot read rejects the whole presentation, and the asset then
exposes no controls or geometry. Its exporter,
`tools/export_ball_assets.py` in the `godot_rigExec` checkout, supports only
the rolling-ball tutorial's graph: one Catmull-Clark mesh without creases or
holes, face-varying `st`, and a `UsdPreviewSurface` whose diffuse and
emissive inputs read the same `UsdUVTexture`. Its controls come from
`rigExec:exposedAvars` and `rigExec:publicName` on `RigExecControl` prims.

The [rolling-ball bake source](../examples/tutorial_rolling_ball_free.usda)
demonstrates a self-contained asset. That example's platform binaries and
external addon source have their own dependency and licensing requirements.
Use source builds for other platforms and verify assets with the matching
runtime version before deployment.
