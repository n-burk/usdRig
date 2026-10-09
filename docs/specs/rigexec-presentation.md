# Embedded presentation data

A `.rigexec` asset can contain the rig program together with geometry,
materials, textures, subdivision data, and exposed controller information.
The consumer reads these records to build its display objects and sends
controller values to the binary runtime.

The producer is `libs/rigExecBake`; binary records are defined in
`libs/rigExecBinary`; loading and evaluation live in `libs/rigExecRuntime`.
Keep all three compatible when changing record layouts. Unsupported data
must be diagnosed during export or loading rather than silently discarded.

`rigExecBake` writes the program and does not write section tag 13
(`Presentation`). The [Godot tutorial](../concepts/tutorial-godot-baked-rig.md)
exporter appends that section as JSON: public control names, render
geometry, and an embedded texture. The core runtime never decodes the tag,
so a program that carries it still loads. The Godot player reads it.

The [free-variant wrapper](../examples/tutorial_rolling_ball_free.usda)
references the striped ball in this checkout. That stage has vertex
`displayColor` and no UV primvar or preview surface. The tutorial exporter
paints a PNG from those colors and appends it. A stage that does author
face-varying `st` and a bound UsdPreviewSurface still takes the
OpenSubdiv path.

That example's platform binaries and external addon source have their own
dependency and licensing requirements. Use source builds for other platforms
and verify assets with the matching runtime version before deployment.
