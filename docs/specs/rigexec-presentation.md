# Embedded presentation data

A `.rigexec` asset can contain the rig program together with geometry,
materials, textures, subdivision data, and exposed controller information.
The consumer reads these records to build its display objects and sends
controller values to the binary runtime.

The producer is `libs/rigExecBake`; binary records are defined in
`libs/rigExecBinary`; loading and evaluation live in `libs/rigExecRuntime`.
Keep all three compatible when changing record layouts. Unsupported data
must be diagnosed during export or loading rather than silently discarded.

The [rolling-ball tutorial](../concepts/tutorial-rolling-ball.md) includes a
free variant that bakes as a self-contained asset. Verify assets with the
matching runtime version before deployment.
