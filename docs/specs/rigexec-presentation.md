# Embedded presentation data

A `.rigexec` asset can contain the rig program together with geometry,
materials, textures, subdivision data, and exposed controller information.
The consumer reads these records to build its display objects and sends
controller values to the binary runtime.

The producer is `libs/rigExecBake`; binary records are defined in
`libs/rigExecBinary`; loading and evaluation live in `libs/rigExecRuntime`.
Keep all three compatible when changing record layouts. Unsupported data
must be diagnosed during export or loading rather than silently discarded.

The [Godot tutorial](../concepts/tutorial-godot-baked-rig.md) demonstrates a
self-contained rolling-ball asset. That tutorial's platform binaries and
external addon source have their own dependency and licensing requirements.
Use source builds for other platforms and verify assets with the matching
runtime version before deployment.
